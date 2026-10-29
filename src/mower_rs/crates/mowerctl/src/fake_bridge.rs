//! A stand-in for ws_bridge in the tests: the rosbridge frames and error
//! texts of mower_ws_bridge's client.rs / hub.rs for a few fixed topics and
//! services, optionally behind the pairing gate's check. It runs on its own
//! thread and runtime, as the commands under test build theirs.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use futures::{SinkExt, StreamExt};
use serde_json::{json, Value};
use tokio::net::TcpListener;
use tokio_tungstenite::tungstenite::handshake::server::{ErrorResponse, Request, Response};
use tokio_tungstenite::tungstenite::http::StatusCode;
use tokio_tungstenite::tungstenite::Message;

use crate::gate::Pairing;

/// The /fix frame of format.rs's byte-identity test, as ws_bridge sends it.
pub const FIX_FRAME: &str = r#"{"msg":{"altitude":null,"header":{"frame_id":"gps_link","stamp":{"nanosec":4294967,"sec":1761550000}},"latitude":25.033000012345678,"longitude":121.5654,"position_covariance":[0.000196,0.0,0.0],"status":{"service":15,"status":-1}},"op":"publish","topic":"/fix"}"#;

pub struct FakeBridge {
    pub url: String,
    /// Every frame a client sent, parsed.
    pub seen: Arc<Mutex<Vec<Value>>>,
    /// The headers of every hand-shake (lower-cased names).
    pub headers: Arc<Mutex<Vec<Vec<(String, String)>>>>,
}

impl FakeBridge {
    pub fn start(gate: Option<Pairing>) -> FakeBridge {
        let std_listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        std_listener.set_nonblocking(true).unwrap();
        let url = format!("ws://{}", std_listener.local_addr().unwrap());
        let seen = Arc::new(Mutex::new(Vec::new()));
        let headers = Arc::new(Mutex::new(Vec::new()));
        let (s, h) = (seen.clone(), headers.clone());
        std::thread::spawn(move || {
            let rt = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
            rt.block_on(async move {
                let listener = TcpListener::from_std(std_listener).unwrap();
                loop {
                    let Ok((stream, _)) = listener.accept().await else { continue };
                    tokio::spawn(serve(stream, gate.clone(), s.clone(), h.clone()));
                }
            });
        });
        FakeBridge { url, seen, headers }
    }

    pub fn seen_ops(&self, op: &str) -> Vec<Value> {
        self.seen.lock().unwrap().iter().filter(|d| d["op"] == op).cloned().collect()
    }
}

async fn serve(
    stream: tokio::net::TcpStream,
    gate: Option<Pairing>,
    seen: Arc<Mutex<Vec<Value>>>,
    headers: Arc<Mutex<Vec<Vec<(String, String)>>>>,
) {
    let check = move |req: &Request, resp: Response| -> Result<Response, ErrorResponse> {
        let h: Vec<(String, String)> =
            req.headers().iter().map(|(k, v)| (k.as_str().to_ascii_lowercase(), v.to_str().unwrap_or("").to_string())).collect();
        headers.lock().unwrap().push(h.clone());
        let Some(p) = &gate else { return Ok(resp) };
        let get = |k: &str| h.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone()).unwrap_or_default();
        let t: i64 = get("x-mower-time").parse().unwrap_or(0);
        let ok = get("x-mower-robot") == p.robot_id && get("x-mower-mac") == p.mac(&get("x-mower-client"), t, &get("x-mower-nonce"));
        if ok {
            return Ok(resp);
        }
        let mut err = ErrorResponse::new(Some(if get("x-mower-mac").is_empty() { "pairing required: missing pairing headers\n" } else { "pairing required: bad mac\n" }.to_string()));
        *err.status_mut() = StatusCode::UNAUTHORIZED;
        Err(err)
    };
    let Ok(ws) = tokio_tungstenite::accept_hdr_async(stream, check).await else { return };
    let (sink, mut source) = ws.split();
    let sink = Arc::new(tokio::sync::Mutex::new(sink));
    while let Some(Ok(Message::Text(text))) = source.next().await {
        let Ok(doc) = serde_json::from_str::<Value>(&text) else { continue };
        seen.lock().unwrap().push(doc.clone());
        let id = doc.get("id").cloned();
        let with_id = |mut v: Value| {
            if let Some(id) = &id {
                v["id"] = id.clone();
            }
            v.to_string()
        };
        let mut replies: Vec<(Duration, String)> = Vec::new();
        match (doc["op"].as_str(), doc["service"].as_str().or(doc["topic"].as_str())) {
            (Some("call_service"), Some(service)) => {
                let answer = match service {
                    "/rosapi/topics" => Some(Ok(json!({"topics": ["/fix", "/robot/online"], "types": ["sensor_msgs/msg/NavSatFix", "std_msgs/msg/Bool"]}))),
                    "/rosapi/get_time" => {
                        let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap();
                        Some(Ok(json!({"time": {"sec": now.as_secs(), "nanosec": now.subsec_nanos()}})))
                    }
                    "/check_nav_status" => Some(Ok(json!({"message": "{\"nav_state\":\"idle\"}", "success": true}))),
                    "/system/restart" => Some(Ok(json!({"message": "restart requested", "success": true}))),
                    "/coverage_progress_status" => Some(Err(format!("{service}: service unavailable"))),
                    "/slow" => None,
                    _ => Some(Err(format!("service {service} is not allowed"))),
                };
                if let Some(answer) = answer {
                    let reply = match answer {
                        Ok(values) => json!({"op": "service_response", "service": service, "values": values, "result": true}),
                        Err(e) => json!({"op": "service_response", "service": service, "values": e, "result": false}),
                    };
                    replies.push((Duration::ZERO, with_id(reply)));
                }
            }
            (Some("subscribe"), Some(topic)) => match topic {
                "/robot/online" => {
                    for k in 0..60u64 {
                        replies.push((Duration::from_millis(25 * k), json!({"op": "publish", "topic": topic, "msg": {"data": true}}).to_string()));
                    }
                }
                "/fix" => replies.push((Duration::ZERO, FIX_FRAME.to_string())),
                "/odom" => replies.push((Duration::ZERO, with_id(json!({"op": "status", "level": "error", "msg": format!("subscribe: {topic} is not allowed")})))),
                "/odmo" => replies.push((
                    Duration::ZERO,
                    with_id(json!({"op": "status", "level": "error", "msg": format!("subscribe: cannot infer type of {topic}: not advertised yet")})),
                )),
                _ => {}
            },
            _ => {}
        }
        let sink = sink.clone();
        tokio::spawn(async move {
            let start = tokio::time::Instant::now();
            for (at, text) in replies {
                tokio::time::sleep_until(start + at).await;
                if sink.lock().await.send(Message::Text(text)).await.is_err() {
                    return;
                }
            }
        });
    }
}
