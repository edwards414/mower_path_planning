//! One WebSocket connection to ws_bridge and the two rosbridge v2 operations
//! mowerctl uses: `call_service` (answered by a `service_response` with the
//! request's id) and `subscribe` (no answer on success, a `status` error
//! frame with the request's id otherwise; then `publish` frames). The texts
//! of the bridge's errors are mapped to the exit codes (`subscribe_error`,
//! `service_error`); E3 keeps those texts for the bus backend.

use std::collections::VecDeque;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use futures::{SinkExt, StreamExt};
use serde_json::{json, Value};
use tokio::net::TcpStream;
use tokio_tungstenite::tungstenite::client::IntoClientRequest;
use tokio_tungstenite::tungstenite::http::{HeaderName, HeaderValue};
use tokio_tungstenite::tungstenite::protocol::WebSocketConfig;
use tokio_tungstenite::tungstenite::{Error as WsError, Message};
use tokio_tungstenite::{MaybeTlsStream, WebSocketStream};

use crate::cli::{ConnOpts, DEFAULT_TIMEOUT};
use crate::gate::{self, Pairing};
use crate::Fail;

/// The largest frame accepted: the map layers are ~0.2 MB of JSON today and
/// an OccupancyGrid would be ~0.6 MB (E3 design 2.5); ws_bridge's own limit
/// is 64 MB.
const MAX_FRAME: usize = 64 << 20;

/// One text frame from the bridge, parsed, with when it arrived.
#[derive(Debug, Clone)]
pub struct Frame {
    pub text: String,
    pub doc: Value,
    /// Arrival, unix seconds (what `echo` prints as `t`).
    pub unix: f64,
    /// Arrival on the monotonic clock (what `hz` measures with).
    pub at: Instant,
}

impl Frame {
    pub fn parse(text: String, unix: f64, at: Instant) -> Option<Frame> {
        let doc: Value = serde_json::from_str(&text).ok()?;
        Some(Frame { text, doc, unix, at })
    }

    pub fn op(&self) -> &str {
        self.doc.get("op").and_then(Value::as_str).unwrap_or("")
    }

    pub fn str_field(&self, key: &str) -> Option<&str> {
        self.doc.get(key).and_then(Value::as_str)
    }

    /// A `publish` frame of `topic`.
    pub fn is_publish_of(&self, topic: &str) -> bool {
        self.op() == "publish" && self.str_field("topic") == Some(topic)
    }

    /// The text of a `status` error frame answering request `id`.
    pub fn error_for(&self, id: &str) -> Option<&str> {
        if self.op() == "status" && self.str_field("level") == Some("error") && self.str_field("id") == Some(id) {
            Some(self.str_field("msg").unwrap_or(""))
        } else {
            None
        }
    }
}

pub fn unix_now() -> f64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0)
}

pub struct Conn {
    ws: WebSocketStream<MaybeTlsStream<TcpStream>>,
    pub url: String,
    next_id: u64,
    /// Frames read while waiting for something else (a reply), handed out
    /// by the next `recv`.
    backlog: VecDeque<Frame>,
}

impl Conn {
    pub async fn open(opts: &ConnOpts) -> Result<Conn, Fail> {
        let url = opts.url();
        let mut request = url.as_str().into_client_request().map_err(|e| Fail::usage(format!("bad URL {url:?}: {e}")))?;
        if let Some(path) = &opts.pair {
            let pairing = Pairing::load(path).map_err(Fail::failed)?;
            let nonce = gate::nonce().map_err(Fail::failed)?;
            for (name, v) in pairing.headers(gate::now_unix(), &nonce) {
                let v = HeaderValue::from_str(&v).map_err(|_| Fail::failed(format!("{path}: robot_id is not a valid header value")))?;
                // from_bytes lower-cases (from_static would panic on X-...)
                request.headers_mut().insert(HeaderName::from_bytes(name.as_bytes()).expect("a valid header name"), v);
            }
        }
        let config = WebSocketConfig { max_message_size: Some(MAX_FRAME), max_frame_size: Some(MAX_FRAME), ..Default::default() };
        let wait = opts.timeout_or(DEFAULT_TIMEOUT);
        let (ws, _) = tokio::time::timeout(wait, tokio_tungstenite::connect_async_with_config(request, Some(config), false))
            .await
            .map_err(|_| Fail::connect(format!("{url}: no WebSocket hand-shake within {} s", wait.as_secs_f64())))?
            .map_err(|e| connect_error(&url, e))?;
        Ok(Conn { ws, url, next_id: 1, backlog: VecDeque::new() })
    }

    /// A request id of this connection: `mowerctl:<what>:<n>`.
    pub fn id(&mut self, what: &str) -> String {
        let n = self.next_id;
        self.next_id += 1;
        format!("mowerctl:{what}:{n}")
    }

    pub async fn send(&mut self, doc: &Value) -> Result<(), Fail> {
        self.ws.send(Message::Text(doc.to_string())).await.map_err(|e| Fail::connect(format!("{}: send failed: {e}", self.url)))
    }

    /// The next text frame that parses as JSON, waiting until `deadline`
    /// (`Ok(None)` then). A closed connection is an error: the bridge never
    /// closes on its own except when it stops.
    pub async fn recv_until(&mut self, deadline: Option<Instant>) -> Result<Option<Frame>, Fail> {
        if let Some(f) = self.backlog.pop_front() {
            return Ok(Some(f));
        }
        loop {
            let next = match deadline {
                Some(d) => match tokio::time::timeout_at(d.into(), self.ws.next()).await {
                    Ok(item) => item,
                    Err(_) => return Ok(None),
                },
                None => self.ws.next().await,
            };
            let text = match next {
                Some(Ok(Message::Text(t))) => t,
                Some(Ok(Message::Binary(b))) => String::from_utf8_lossy(&b).into_owned(),
                Some(Ok(Message::Close(_))) | None => return Err(Fail::connect(format!("{}: the bridge closed the connection", self.url))),
                // tungstenite answers pings itself
                Some(Ok(_)) => continue,
                Some(Err(e)) => return Err(Fail::connect(format!("{}: {e}", self.url))),
            };
            let (unix, at) = (unix_now(), Instant::now());
            if let Some(f) = Frame::parse(text, unix, at) {
                return Ok(Some(f));
            }
        }
    }

    /// `call_service` and its `service_response` (the whole frame). Frames
    /// that arrive meanwhile are kept for `recv_until`.
    pub async fn call(&mut self, service: &str, args: Value, timeout: Duration) -> Result<Frame, Fail> {
        let id = self.id("call");
        self.send(&json!({"op": "call_service", "service": service, "args": args, "id": id})).await?;
        let deadline = Instant::now() + timeout;
        let mut kept = VecDeque::new();
        let result = loop {
            match self.recv_until(Some(deadline)).await {
                Ok(Some(f)) if f.op() == "service_response" && f.str_field("id") == Some(id.as_str()) => break Ok(f),
                Ok(Some(f)) => match f.error_for(&id) {
                    Some(msg) => break Err(Fail::failed(format!("{service}: {msg}"))),
                    None => kept.push_back(f),
                },
                Ok(None) => break Err(Fail::timeout(format!("{service}: no answer within {} s", timeout.as_secs_f64()))),
                Err(e) => break Err(e),
            }
        };
        kept.append(&mut self.backlog);
        self.backlog = kept;
        result
    }

    /// `call` and the reply's `values`; a `result: false` reply becomes the
    /// failure its text describes.
    pub async fn call_values(&mut self, service: &str, args: Value, timeout: Duration) -> Result<Value, Fail> {
        let reply = self.call(service, args, timeout).await?;
        let values = reply.doc.get("values").cloned().unwrap_or(Value::Null);
        if reply.doc.get("result").and_then(Value::as_bool) == Some(false) {
            return Err(service_error(service, values.as_str().unwrap_or(&values.to_string())));
        }
        Ok(values)
    }

    /// `subscribe` without a type: the bridge looks the type up and refuses
    /// a topic nobody publishes, instead of making a reader for a name that
    /// may be a typo. Returns the request id its error would carry.
    pub async fn subscribe(&mut self, topic: &str, throttle_ms: Option<u64>) -> Result<String, Fail> {
        let id = self.id("sub");
        let mut doc = json!({"op": "subscribe", "topic": topic, "id": id});
        if let Some(ms) = throttle_ms {
            doc["throttle_rate"] = json!(ms);
        }
        self.send(&doc).await?;
        Ok(id)
    }

    pub async fn close(mut self) {
        let _ = tokio::time::timeout(Duration::from_millis(500), self.ws.close(None)).await;
    }
}

fn connect_error(url: &str, e: WsError) -> Fail {
    match e {
        WsError::Http(response) => {
            let body = response.body().as_ref().map(|b| String::from_utf8_lossy(b).trim().to_string()).unwrap_or_default();
            let hint = if response.status().as_u16() == 401 { " (rejected by the pairing gate)" } else { "" };
            Fail::connect(format!("{url}: HTTP {}{hint}{}", response.status(), if body.is_empty() { String::new() } else { format!(": {body}") }))
        }
        WsError::Io(io) => Fail::connect(format!("cannot connect to {url}: {io}")),
        other => Fail::connect(format!("{url}: {other}")),
    }
}

/// The bridge's `subscribe: ...` error texts (mower_ws_bridge client.rs).
pub fn subscribe_error(topic: &str, msg: &str) -> Fail {
    if msg.ends_with("is not allowed") {
        Fail::missing(format!("{topic} is not served on this listener (its policy allows the app's topics only)"))
    } else if msg.contains("cannot infer type") {
        Fail::missing(format!("{topic}: no such topic (nothing publishes it)"))
    } else {
        Fail::failed(format!("{topic}: {msg}"))
    }
}

/// The bridge's `result: false` texts of a call (mower_ws_bridge client.rs,
/// hub.rs).
pub fn service_error(service: &str, text: &str) -> Fail {
    if text.ends_with("is not allowed") {
        Fail::missing(format!("{service} is not served on this listener (not in its policy)"))
    } else if text.ends_with("service unavailable") {
        Fail::missing(format!("{service}: no server answers it"))
    } else if text.contains("did not answer within") {
        Fail::timeout(format!("{service}: {text}"))
    } else {
        Fail::failed(format!("{service}: {text}"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{EXIT_CONNECT, EXIT_FAILED, EXIT_MISSING, EXIT_TIMEOUT};

    #[test]
    fn the_bridges_error_texts_map_to_exit_codes() {
        // the texts of mower_ws_bridge/src/client.rs and hub.rs
        assert_eq!(subscribe_error("/odom", "subscribe: /odom is not allowed").code, EXIT_MISSING);
        assert_eq!(subscribe_error("/odmo", "subscribe: cannot infer type of /odmo: not advertised yet").code, EXIT_MISSING);
        assert_eq!(subscribe_error("/x", "subscribe: subscribe /x (a/msg/B) failed: boom").code, EXIT_FAILED);
        assert_eq!(service_error("/rm", "service /rm is not allowed").code, EXIT_MISSING);
        assert_eq!(service_error("/check_nav_status", "/check_nav_status: service unavailable").code, EXIT_MISSING);
        assert_eq!(service_error("/s", "service /s did not answer within 30 s").code, EXIT_TIMEOUT);
        assert_eq!(service_error("/s", "/s: request failed: x").code, EXIT_FAILED);
    }

    #[test]
    fn frames_know_their_op_topic_and_errors() {
        let f = Frame::parse(r#"{"msg":{"data":true},"op":"publish","topic":"/robot/online"}"#.into(), 1.0, Instant::now()).unwrap();
        assert!(f.is_publish_of("/robot/online"));
        assert!(!f.is_publish_of("/robot/info"));
        let e = Frame::parse(r#"{"id":"mowerctl:sub:1","level":"error","msg":"subscribe: /x is not allowed","op":"status"}"#.into(), 1.0, Instant::now()).unwrap();
        assert_eq!(e.error_for("mowerctl:sub:1"), Some("subscribe: /x is not allowed"));
        assert_eq!(e.error_for("mowerctl:sub:2"), None);
        assert!(Frame::parse("not json".into(), 1.0, Instant::now()).is_none());
    }

    #[test]
    fn nothing_listening_is_a_connection_failure() {
        // a port nobody listens on (bound, then closed)
        let port = std::net::TcpListener::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port();
        let opts = ConnOpts { url: Some(format!("ws://127.0.0.1:{port}")), ..Default::default() };
        let e = crate::block_on(async { Conn::open(&opts).await.map(|_| ()) }).unwrap_err();
        assert_eq!(e.code, EXIT_CONNECT, "{e:?}");
        assert!(e.msg.contains("cannot connect"), "{e:?}");
    }
}
