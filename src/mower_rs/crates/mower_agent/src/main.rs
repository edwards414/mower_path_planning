//! mower_agent: robot side of the fleet backend (port of
//! `mower_mission/mower_agent.py`, docs/BACKEND_ARCHITECTURE.md).
//!
//! The robot never accepts inbound connections from the internet (4G is
//! behind carrier NAT). Instead this agent:
//!
//! 1. makes sure `identity.json` has a `device_key` and registers the robot
//!    with the backend (`POST /v1/robots/register`, provision token);
//! 2. keeps one outbound WebSocket to `/v1/relay/robot/<robot_id>` open and
//!    sends a heartbeat every 10 s with `/robot/info`, `/robot/telemetry`
//!    and the LAN address, so the app can see the robot online;
//! 3. when the hub says `{"t":"open"}` (a phone connected), opens a session to
//!    the local pairing gate (`ws://127.0.0.1:9090`) with the phone's
//!    X-Mower-* headers, so the robot still verifies every phone itself, and
//!    pipes mrelay1 frames ([`relay`]) both ways;
//! 4. answers `{"t":"http"}`: the phone's WHEP signaling (POST offer /
//!    DELETE session) is carried to MediaMTX on loopback and the answer sent
//!    back as `http_res`, so the camera works across networks;
//! 5. fetches TURN credentials from `GET /v1/robots/<id>/turn` and writes
//!    them into MediaMTX through its API.
//!
//! No `MOWER_BACKEND_URL` -> the agent exits quietly (simulation, bench). No
//! ROS: it reads `/robot/info` and `/robot/telemetry` through the local
//! bridge on loopback like any other client. Same log lines, control
//! messages, timeouts and back-offs as the Python agent.

mod relay;

use std::collections::HashMap;
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use futures::{SinkExt, StreamExt};
use hmac::{Hmac, Mac};
use serde_json::{json, Map, Value};
use sha2::Sha256;
use tokio::sync::mpsc;
use tokio_tungstenite::tungstenite::client::IntoClientRequest;
use tokio_tungstenite::tungstenite::protocol::WebSocketConfig;
use tokio_tungstenite::tungstenite::{Error as WsError, Message as WsMsg};

use crate::relay::Message;

const ROBOT_API_VERSION: i64 = 2;
const ROBOT_CLIENT: &str = "@robot";
const HEARTBEAT_S: u64 = 10;
const TELEMETRY_THROTTLE_MS: i64 = 5000;
const REGISTER_RETRY_S: u64 = 30;
const RECONNECT_MAX_S: u64 = 60;
const LOCAL_GATE: &str = "ws://127.0.0.1:9090";
const LOCAL_ROSBRIDGE: &str = "ws://127.0.0.1:9091";
const LOCAL_MEDIAMTX: &str = "http://127.0.0.1:8889";
const LOCAL_MEDIAMTX_API: &str = "http://127.0.0.1:9997";
const HTTP_RELAY_TIMEOUT_S: u64 = 10;
const HTTP_RELAY_MAX_BODY: usize = 64 * 1024;
const HTTP_RELAY_METHODS: [&str; 5] = ["GET", "POST", "PATCH", "DELETE", "OPTIONS"];
const TURN_REFRESH_S: i64 = 3600;
const TURN_RETRY_S: i64 = 60;
const TURN_ROBOT_TRANSPORTS: [&str; 2] = ["transport=udp", "transport=tcp"];
const IDENTITY_FILE: &str = "identity.json";

// ----------------------------------------------------------------- logging

static LOG_LEVEL: AtomicI32 = AtomicI32::new(20);

fn log_at(level: i32, name: &str, msg: &str) {
    if level >= LOG_LEVEL.load(Ordering::Relaxed) {
        println!("[mower_agent] {name} {msg}");
    }
}
macro_rules! debug { ($($a:tt)*) => { log_at(10, "DEBUG", &format!($($a)*)) } }
macro_rules! info { ($($a:tt)*) => { log_at(20, "INFO", &format!($($a)*)) } }
macro_rules! warning { ($($a:tt)*) => { log_at(30, "WARNING", &format!($($a)*)) } }
macro_rules! error { ($($a:tt)*) => { log_at(40, "ERROR", &format!($($a)*)) } }

fn now_unix() -> i64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs() as i64).unwrap_or(0)
}

fn random_bytes(n: usize) -> Vec<u8> {
    use std::io::Read;
    let mut b = vec![0u8; n];
    if let Ok(mut f) = std::fs::File::open("/dev/urandom") {
        let _ = f.read_exact(&mut b);
    }
    b
}

/// `%s` of a JSON value the way Python prints the parsed body field.
fn py_str(v: Option<&Value>) -> String {
    match v {
        None | Some(Value::Null) => "None".into(),
        Some(Value::String(s)) => s.clone(),
        Some(Value::Bool(true)) => "True".into(),
        Some(Value::Bool(false)) => "False".into(),
        Some(other) => other.to_string(),
    }
}

// ---------------------------------------------------------------- identity

#[derive(Clone)]
struct Identity {
    robot_id: String,
    name: String,
    secret: String,
    device_key: String,
    doc: Map<String, Value>,
}

fn state_dir_of(arg: Option<&str>) -> String {
    match arg {
        Some(d) if !d.is_empty() => d.to_string(),
        _ => mower_rs_common::state_dir(),
    }
}

/// `identity.load_identity`: parsed identity.json or None.
fn load_identity(dir: &str) -> Option<Identity> {
    let doc = mower_rs_common::read_state_json(dir, IDENTITY_FILE)?;
    let doc = doc.as_object()?.clone();
    let robot_id = doc.get("robot_id").and_then(Value::as_str).unwrap_or("").to_string();
    let valid = robot_id.len() == 9 && robot_id.starts_with("MW-") && robot_id[3..].chars().all(|c| c.is_ascii_uppercase() || c.is_ascii_digit());
    if !valid {
        return None;
    }
    let secret = doc.get("secret").and_then(Value::as_str).unwrap_or("").to_string();
    if secret.is_empty() {
        return None;
    }
    Some(Identity {
        robot_id,
        name: doc.get("name").and_then(Value::as_str).unwrap_or("").to_string(),
        secret,
        device_key: doc.get("device_key").and_then(Value::as_str).unwrap_or("").to_string(),
        doc,
    })
}

/// `new_device_key`: 256 random bits, base32 without padding.
fn new_device_key() -> String {
    data_encoding::BASE32_NOPAD.encode(&random_bytes(32))
}

/// `ensure_device_key`: add a device_key the first time (atomic rewrite, 0600).
fn ensure_device_key(dir: &str) -> Option<Identity> {
    let mut identity = load_identity(dir)?;
    if identity.device_key.is_empty() {
        identity.device_key = new_device_key();
        identity.doc.insert("device_key".into(), Value::String(identity.device_key.clone()));
        identity.doc.insert("device_key_created".into(), json!(now_unix()));
        let path = std::path::Path::new(dir).join(IDENTITY_FILE);
        let tmp = path.with_extension("json.tmp");
        let text = format!("{}\n", serde_json::to_string_pretty(&Value::Object(identity.doc.clone())).unwrap_or_default());
        let written = std::fs::write(&tmp, text)
            .and_then(|_| {
                use std::os::unix::fs::PermissionsExt;
                std::fs::set_permissions(&tmp, std::fs::Permissions::from_mode(0o600))
            })
            .and_then(|_| std::fs::rename(&tmp, &path));
        if let Err(e) = written {
            warning!("could not save device_key: {e}");
        }
        info!("created device_key for {}", identity.robot_id);
    }
    Some(identity)
}

/// `identity.secret_bytes`: upper-case, pad to a multiple of 8, base32 decode.
fn secret_bytes(secret: &str) -> Vec<u8> {
    let mut s = secret.trim().to_ascii_uppercase();
    while s.len() % 8 != 0 {
        s.push('=');
    }
    data_encoding::BASE32.decode(s.as_bytes()).unwrap_or_default()
}

fn compute_mac(secret: &str, robot_id: &str, client: &str, t: i64, nonce: &str) -> String {
    let mut mac = Hmac::<Sha256>::new_from_slice(&secret_bytes(secret)).expect("hmac accepts any key length");
    mac.update(format!("{robot_id}\n{client}\n{t}\n{nonce}").as_bytes());
    hex::encode(mac.finalize().into_bytes())
}

fn signed_headers(identity: &Identity) -> Vec<(String, String)> {
    let now = now_unix();
    let nonce = hex::encode(random_bytes(16));
    vec![
        ("X-Mower-Robot".into(), identity.robot_id.clone()),
        ("X-Mower-Client".into(), ROBOT_CLIENT.into()),
        ("X-Mower-Time".into(), now.to_string()),
        ("X-Mower-Nonce".into(), nonce.clone()),
        ("X-Mower-Mac".into(), compute_mac(&identity.device_key, &identity.robot_id, ROBOT_CLIENT, now, &nonce)),
    ]
}

fn user_agent(identity: &Identity) -> String {
    format!("mower-agent/{ROBOT_API_VERSION} ({})", identity.robot_id)
}

fn lan_address() -> String {
    let Ok(s) = std::net::UdpSocket::bind("0.0.0.0:0") else { return String::new() };
    if s.connect("10.255.255.255:1").is_err() {
        return String::new();
    }
    s.local_addr().map(|a| a.ip().to_string()).unwrap_or_default()
}

fn http_to_ws(base_url: &str) -> String {
    if let Some(rest) = base_url.strip_prefix("https://") {
        format!("wss://{rest}")
    } else if let Some(rest) = base_url.strip_prefix("http://") {
        format!("ws://{rest}")
    } else {
        base_url.to_string()
    }
}

// -------------------------------------------------------------------- HTTP

fn http_agent(timeout_s: u64) -> ureq::Agent {
    ureq::AgentBuilder::new().timeout(Duration::from_secs(timeout_s)).build()
}

fn json_body(resp: ureq::Response) -> Value {
    let text = resp.into_string().unwrap_or_default();
    if text.is_empty() {
        return json!({});
    }
    serde_json::from_str(&text).unwrap_or_else(|_| json!({}))
}

/// `(status, body)` of a JSON request; network trouble is `Err`.
fn json_request(req: ureq::Request, body: Option<&[u8]>) -> Result<(u16, Value), String> {
    let result = match body {
        Some(b) => req.send_bytes(b),
        None => req.call(),
    };
    match result {
        Ok(resp) => Ok((resp.status(), json_body(resp))),
        Err(ureq::Error::Status(code, resp)) => {
            let text = resp.into_string().unwrap_or_default();
            let body = if text.is_empty() { json!({}) } else { serde_json::from_str(&text).unwrap_or_else(|_| json!({})) };
            Ok((code, body))
        }
        Err(ureq::Error::Transport(t)) => Err(t.to_string()),
    }
}

fn register_body(identity: &Identity, model: &str) -> Value {
    json!({
        "robot_id": identity.robot_id,
        "name": identity.name,
        "model": model,
        "device_key": identity.device_key,
        "pairing_secret": identity.secret,
    })
}

/// `register_once`: POST /v1/robots/register.
fn register_once(base_url: &str, token: &str, identity: &Identity, model: &str) -> Result<(u16, Value), String> {
    let req = http_agent(15)
        .post(&format!("{}/v1/robots/register", base_url.trim_end_matches('/')))
        .set("Content-Type", "application/json")
        .set("Authorization", &format!("Bearer {token}"))
        .set("User-Agent", &user_agent(identity));
    json_request(req, Some(register_body(identity, model).to_string().as_bytes()))
}

/// `signed_get`: GET a backend URL with the robot's X-Mower-* headers.
fn signed_get(url: &str, identity: &Identity) -> Result<(u16, Value), String> {
    let mut req = http_agent(15).get(url);
    for (k, v) in signed_headers(identity) {
        req = req.set(&k, &v);
    }
    req = req.set("User-Agent", &user_agent(identity));
    json_request(req, None)
}

/// `relay_http_once`: one relayed request against MediaMTX.
fn relay_http_once(base_url: &str, method: &str, path: &str, headers: &[(String, String)], body: &[u8]) -> Result<(u16, Vec<(String, String)>, Vec<u8>), String> {
    let mut req = http_agent(HTTP_RELAY_TIMEOUT_S).request(method, &format!("{}{}", base_url.trim_end_matches('/'), path));
    for (k, v) in headers {
        req = req.set(k, v);
    }
    let result = if body.is_empty() { req.call() } else { req.send_bytes(body) };
    let resp = match result {
        Ok(r) => r,
        Err(ureq::Error::Status(_, r)) => r,
        Err(ureq::Error::Transport(t)) => return Err(t.to_string()),
    };
    let status = resp.status();
    let res_headers: Vec<(String, String)> = resp.headers_names().iter().filter_map(|n| resp.header(n).map(|v| (n.clone(), v.to_string()))).collect();
    let mut out = Vec::new();
    use std::io::Read;
    let _ = resp.into_reader().take((HTTP_RELAY_MAX_BODY * 16) as u64).read_to_end(&mut out);
    Ok((status, res_headers, out))
}

/// `mediamtx_ice_servers`: backend `iceServers` -> MediaMTX `webrtcICEServers2`.
fn mediamtx_ice_servers(ice_servers: &Value) -> Vec<Value> {
    let mut out = Vec::new();
    for server in ice_servers.as_array().cloned().unwrap_or_default() {
        let urls: Vec<String> = match server.get("urls") {
            Some(Value::String(s)) => vec![s.clone()],
            Some(Value::Array(a)) => a.iter().filter_map(|u| u.as_str().map(str::to_string)).collect(),
            _ => Vec::new(),
        };
        for transport in TURN_ROBOT_TRANSPORTS {
            for url in &urls {
                if url.starts_with("turn:") && url.ends_with(&format!("?{transport}")) {
                    out.push(json!({
                        "url": url,
                        "username": server.get("username").and_then(Value::as_str).unwrap_or(""),
                        "password": server.get("credential").and_then(Value::as_str).unwrap_or(""),
                        "clientOnly": false,
                    }));
                    break;
                }
            }
        }
    }
    out
}

/// `patch_mediamtx_config`: PATCH /v3/config/global/patch, the HTTP status.
fn patch_mediamtx_config(api_url: &str, patch: &Value) -> Result<u16, String> {
    let req = http_agent(10).request("PATCH", &format!("{}/v3/config/global/patch", api_url.trim_end_matches('/'))).set("Content-Type", "application/json");
    match req.send_bytes(patch.to_string().as_bytes()) {
        Ok(r) => Ok(r.status()),
        Err(ureq::Error::Status(code, _)) => Ok(code),
        Err(ureq::Error::Transport(t)) => Err(t.to_string()),
    }
}

/// `WHEP_PATH_RE`: ^/[A-Za-z0-9_-]+/whep(/[A-Za-z0-9_.-]+)?$
fn whep_path(path: &str) -> bool {
    let Some(rest) = path.strip_prefix('/') else { return false };
    let mut parts = rest.split('/');
    let name = parts.next().unwrap_or("");
    if name.is_empty() || !name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-') {
        return false;
    }
    if parts.next() != Some("whep") {
        return false;
    }
    match parts.next() {
        None => true,
        Some(s) => !s.is_empty() && s.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '.' || c == '-') && parts.next().is_none(),
    }
}

// ----------------------------------------------------------------- agent

type WsStream = tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>>;

struct Session {
    sid: String,
    inbox: mpsc::UnboundedSender<Option<Message>>,
    reassembler: Mutex<relay::Reassembler>,
    closer: Mutex<Option<mpsc::UnboundedSender<()>>>,
}

struct Agent {
    identity: Identity,
    backend_url: String,
    gate_url: String,
    rosbridge_url: String,
    model: String,
    mediamtx_url: String,
    mediamtx_api_url: String,
    ice_applied: Mutex<Option<Vec<Value>>>,
    sessions: Mutex<HashMap<String, Arc<Session>>>,
    relay: Mutex<Option<mpsc::UnboundedSender<WsMsg>>>,
    info: Mutex<Option<Value>>,
    telemetry: Mutex<Option<Value>>,
}

enum RelayEnd {
    /// `websockets.InvalidStatusCode`
    Refused(u16),
    Other(String),
}

impl Agent {
    fn relay_url(&self) -> String {
        format!("{}/v1/relay/robot/{}", http_to_ws(&self.backend_url), self.identity.robot_id)
    }

    fn send_control(&self, obj: Value) {
        if let Some(tx) = self.relay.lock().unwrap().as_ref() {
            let _ = tx.send(WsMsg::Text(obj.to_string()));
        }
    }

    fn send_raw(&self, frame: Vec<u8>) {
        if let Some(tx) = self.relay.lock().unwrap().as_ref() {
            let _ = tx.send(WsMsg::Binary(frame));
        }
    }

    fn heartbeat(&self) -> Value {
        json!({
            "t": "hb",
            "info": self.info.lock().unwrap().clone().unwrap_or(Value::Null),
            "telemetry": self.telemetry.lock().unwrap().clone().unwrap_or(Value::Null),
            "lan": lan_address(),
        })
    }

    async fn run_relay_forever(self: Arc<Self>) {
        let mut delay = 1u64;
        loop {
            match self.run_relay_once().await {
                Ok(()) => delay = 1,
                Err(RelayEnd::Refused(code)) => {
                    warning!("relay refused us: HTTP {code} (re-registering)");
                    self.register(false).await;
                }
                Err(RelayEnd::Other(e)) => warning!("relay connection ended: {e}"),
            }
            self.close_all_sessions().await;
            info!("reconnecting in {delay} s");
            tokio::time::sleep(Duration::from_secs(delay)).await;
            delay = (delay * 2).min(RECONNECT_MAX_S);
        }
    }

    async fn run_relay_once(self: &Arc<Self>) -> Result<(), RelayEnd> {
        let url = self.relay_url();
        info!("connecting to {url}");
        let mut request = url.as_str().into_client_request().map_err(|e| RelayEnd::Other(e.to_string()))?;
        for (k, v) in signed_headers(&self.identity) {
            request.headers_mut().insert(k.parse::<tokio_tungstenite::tungstenite::http::HeaderName>().unwrap(), v.parse().unwrap());
        }
        request.headers_mut().insert("User-Agent", user_agent(&self.identity).parse().unwrap());
        request.headers_mut().insert("Sec-WebSocket-Protocol", relay::SUBPROTOCOL.parse().unwrap());
        let config = WebSocketConfig { max_message_size: Some(2 * 1024 * 1024), max_frame_size: Some(2 * 1024 * 1024), ..Default::default() };
        let (ws, _) = match tokio_tungstenite::connect_async_with_config(request, Some(config), false).await {
            Ok(v) => v,
            Err(WsError::Http(resp)) => return Err(RelayEnd::Refused(resp.status().as_u16())),
            Err(e) => return Err(RelayEnd::Other(e.to_string())),
        };
        let (mut sink, mut stream) = ws.split();
        let (tx, mut rx) = mpsc::unbounded_channel::<WsMsg>();
        *self.relay.lock().unwrap() = Some(tx.clone());
        info!("relay connected");

        // writer: everything the agent sends, plus pings every 20 s
        let writer = tokio::spawn(async move {
            let mut ping = tokio::time::interval(Duration::from_secs(20));
            ping.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
            ping.tick().await;
            loop {
                tokio::select! {
                    m = rx.recv() => match m {
                        Some(m) => { if sink.send(m).await.is_err() { break; } }
                        None => break,
                    },
                    _ = ping.tick() => { if sink.send(WsMsg::Ping(Vec::new())).await.is_err() { break; } }
                }
            }
            let _ = sink.close().await;
        });
        // heartbeat
        let hb = {
            let agent = self.clone();
            tokio::spawn(async move {
                loop {
                    agent.send_control(agent.heartbeat());
                    tokio::time::sleep(Duration::from_secs(HEARTBEAT_S)).await;
                }
            })
        };
        let mut last_pong = Instant::now();
        let outcome = loop {
            let next = tokio::time::timeout(Duration::from_secs(5), stream.next()).await;
            match next {
                Err(_) => {
                    if last_pong.elapsed() > Duration::from_secs(40) {
                        break Err(RelayEnd::Other("ping timeout".into()));
                    }
                }
                Ok(None) => break Ok(()),
                Ok(Some(Err(e))) => break Err(RelayEnd::Other(e.to_string())),
                Ok(Some(Ok(msg))) => match msg {
                    WsMsg::Text(t) => self.on_control(&t).await,
                    WsMsg::Binary(b) => self.on_frame(&b),
                    WsMsg::Pong(_) => last_pong = Instant::now(),
                    WsMsg::Ping(_) => last_pong = Instant::now(),
                    WsMsg::Close(_) => break Ok(()),
                    WsMsg::Frame(_) => {}
                },
            }
        };
        hb.abort();
        *self.relay.lock().unwrap() = None;
        drop(tx);
        writer.abort();
        outcome
    }

    async fn on_control(self: &Arc<Self>, text: &str) {
        let Ok(msg) = serde_json::from_str::<Value>(text) else { return };
        match msg.get("t").and_then(Value::as_str) {
            Some("open") => {
                let sid = msg.get("sid").and_then(Value::as_str);
                let headers = msg.get("headers").cloned().unwrap_or(json!({}));
                let (Some(sid), Some(headers)) = (sid, headers.as_object()) else { return };
                if sid.len() != 2 * relay::SID_BYTES {
                    return;
                }
                if self.sessions.lock().unwrap().contains_key(sid) {
                    return;
                }
                let headers: Vec<(String, String)> = headers.iter().map(|(k, v)| (k.clone(), py_str(Some(v)))).collect();
                self.open_session(sid.to_string(), headers);
            }
            Some("close") => {
                let session = msg.get("sid").and_then(Value::as_str).and_then(|sid| self.sessions.lock().unwrap().get(sid).cloned());
                if let Some(s) = session {
                    self.close_session(&s, 1000, "", false).await;
                }
            }
            Some("http") => {
                let agent = self.clone();
                tokio::spawn(async move { agent.relay_http(msg).await });
            }
            Some(t @ ("clients" | "pair_confirm")) => info!("control {t} not implemented yet"),
            _ => {}
        }
    }

    fn on_frame(self: &Arc<Self>, frame: &[u8]) {
        let Some((sid, frame_type, payload)) = relay::decode_frame(frame) else { return };
        let session = self.sessions.lock().unwrap().get(&sid).cloned();
        let Some(session) = session else {
            self.send_control(json!({"t": "close", "sid": sid, "code": 1000, "reason": "no such session"}));
            return;
        };
        let fed = session.reassembler.lock().unwrap().feed(frame_type, payload);
        match fed {
            Err(e) => {
                warning!("session {} : {e}", session.sid);
                let (agent_session, msg) = (session.clone(), e);
                let _ = agent_session.closer.lock().unwrap().as_ref().map(|c| c.send(()));
                self.spawn_close(agent_session, 1009, msg, true);
            }
            Ok(Some(message)) => {
                let _ = session.inbox.send(Some(message));
            }
            Ok(None) => {}
        }
    }

    fn spawn_close(self: &Arc<Self>, session: Arc<Session>, code: u16, reason: String, notify: bool) {
        let agent = self.clone();
        tokio::spawn(async move { agent.close_session(&session, code, &reason, notify).await });
    }

    /// `Session.close`: only the first close of a session does anything.
    async fn close_session(&self, session: &Arc<Session>, code: u16, reason: &str, notify: bool) {
        if self.sessions.lock().unwrap().remove(&session.sid).is_none() {
            return;
        }
        if notify {
            self.send_control(json!({"t": "close", "sid": session.sid, "code": code, "reason": reason}));
        }
        let _ = session.inbox.send(None);
        let closer = session.closer.lock().unwrap().take();
        if let Some(c) = closer {
            let _ = c.send(());
        }
    }

    async fn close_all_sessions(&self) {
        let all: Vec<Arc<Session>> = self.sessions.lock().unwrap().values().cloned().collect();
        for s in all {
            self.close_session(&s, 1012, "relay lost", false).await;
        }
    }

    /// `Session.__init__` + `Session.run`: one phone connection relayed to the local gate.
    fn open_session(self: &Arc<Self>, sid: String, headers: Vec<(String, String)>) {
        let (inbox_tx, mut inbox_rx) = mpsc::unbounded_channel::<Option<Message>>();
        let (closer_tx, mut closer_rx) = mpsc::unbounded_channel::<()>();
        let session = Arc::new(Session { sid: sid.clone(), inbox: inbox_tx, reassembler: Mutex::new(relay::Reassembler::new(relay::MAX_MESSAGE)), closer: Mutex::new(Some(closer_tx)) });
        self.sessions.lock().unwrap().insert(sid.clone(), session.clone());
        let agent = self.clone();
        tokio::spawn(async move {
            let mut request = match agent.gate_url.as_str().into_client_request() {
                Ok(r) => r,
                Err(e) => {
                    warning!("session {sid}: gate unreachable: {e}");
                    agent.send_control(json!({"t": "open_err", "sid": sid, "code": 503, "reason": "gate unreachable"}));
                    agent.sessions.lock().unwrap().remove(&sid);
                    return;
                }
            };
            for (k, v) in &headers {
                if let (Ok(name), Ok(value)) = (k.parse::<tokio_tungstenite::tungstenite::http::HeaderName>(), v.parse::<tokio_tungstenite::tungstenite::http::HeaderValue>()) {
                    request.headers_mut().append(name, value);
                }
            }
            let config = WebSocketConfig { max_message_size: Some(relay::MAX_MESSAGE), max_frame_size: Some(relay::MAX_MESSAGE), ..Default::default() };
            let ws: WsStream = match tokio_tungstenite::connect_async_with_config(request, Some(config), false).await {
                Ok((ws, _)) => ws,
                Err(WsError::Http(resp)) => {
                    let code = resp.status().as_u16();
                    warning!("session {sid} refused by the gate: HTTP {code}");
                    agent.send_control(json!({"t": "open_err", "sid": sid, "code": code, "reason": "gate refused"}));
                    agent.sessions.lock().unwrap().remove(&sid);
                    return;
                }
                Err(e) => {
                    warning!("session {sid}: gate unreachable: {e}");
                    agent.send_control(json!({"t": "open_err", "sid": sid, "code": 503, "reason": "gate unreachable"}));
                    agent.sessions.lock().unwrap().remove(&sid);
                    return;
                }
            };
            agent.send_control(json!({"t": "opened", "sid": sid}));
            let client = headers.iter().find(|(k, _)| k.eq_ignore_ascii_case("x-mower-client")).map(|(_, v)| v.clone()).unwrap_or_default();
            info!("session {sid} open ({client})");
            let (mut sink, mut stream) = ws.split();
            loop {
                tokio::select! {
                    m = inbox_rx.recv() => match m {
                        Some(Some(Message::Text(t))) => { if sink.send(WsMsg::Text(t)).await.is_err() { break; } }
                        Some(Some(Message::Binary(b))) => { if sink.send(WsMsg::Binary(b)).await.is_err() { break; } }
                        Some(None) | None => break,
                    },
                    _ = closer_rx.recv() => break,
                    m = stream.next() => match m {
                        Some(Ok(WsMsg::Text(t))) => {
                            if let Ok(frames) = relay::encode_frames(&sid, &Message::Text(t), relay::CHUNK_SIZE) {
                                for f in frames { agent.send_raw(f); }
                            }
                        }
                        Some(Ok(WsMsg::Binary(b))) => {
                            if let Ok(frames) = relay::encode_frames(&sid, &Message::Binary(b), relay::CHUNK_SIZE) {
                                for f in frames { agent.send_raw(f); }
                            }
                        }
                        Some(Ok(WsMsg::Ping(p))) => { let _ = sink.send(WsMsg::Pong(p)).await; }
                        Some(Ok(WsMsg::Pong(_))) | Some(Ok(WsMsg::Frame(_))) => {}
                        Some(Ok(WsMsg::Close(_))) | Some(Err(_)) | None => break,
                    },
                }
            }
            let _ = sink.close().await;
            agent.close_session(&session, 1000, "session ended", true).await;
        });
    }

    // -- phase 3: WHEP signaling relayed to MediaMTX --

    async fn relay_http(&self, msg: Value) {
        let Some(rid) = msg.get("rid").and_then(Value::as_str).map(str::to_string) else { return };
        let method = py_str(msg.get("method")).to_uppercase();
        let method = if msg.get("method").is_none() { String::new() } else { method };
        let full_path = if msg.get("path").is_none() { String::new() } else { py_str(msg.get("path")) };
        let headers_v = msg.get("headers").cloned().unwrap_or(Value::Null);
        let headers_v = if headers_v.is_null() { json!({}) } else { headers_v };
        let (path, query) = match full_path.split_once('?') {
            Some((p, q)) => (p.to_string(), q.to_string()),
            None => (full_path.clone(), String::new()),
        };
        if !HTTP_RELAY_METHODS.contains(&method.as_str()) || !whep_path(&path) || !headers_v.is_object() {
            self.http_res(&rid, 403, vec![("Content-Type".into(), "application/json".into())], b"{\"error\": \"not relayed\"}".to_vec());
            return;
        }
        let body = {
            use base64::Engine;
            let raw = msg.get("body_b64").and_then(Value::as_str).unwrap_or("");
            base64::engine::general_purpose::STANDARD.decode(raw).unwrap_or_default()
        };
        if body.len() > HTTP_RELAY_MAX_BODY {
            self.http_res(&rid, 413, Vec::new(), Vec::new());
            return;
        }
        let target = if query.is_empty() { path.clone() } else { format!("{path}?{query}") };
        let headers: Vec<(String, String)> = headers_v.as_object().unwrap().iter().map(|(k, v)| (k.clone(), py_str(Some(v)))).collect();
        let (base, m, t) = (self.mediamtx_url.clone(), method.clone(), target.clone());
        let result = tokio::task::spawn_blocking(move || relay_http_once(&base, &m, &t, &headers, &body)).await.unwrap_or_else(|e| Err(e.to_string()));
        match result {
            Err(e) => {
                warning!("http {method} {target}: mediamtx unreachable: {e}");
                self.http_res(&rid, 502, vec![("Content-Type".into(), "application/json".into())], b"{\"error\": \"camera server unreachable\"}".to_vec());
            }
            Ok((status, res_headers, res_body)) => {
                info!("http {method} {target} -> {status}");
                self.http_res(&rid, status, res_headers, res_body);
            }
        }
    }

    fn http_res(&self, rid: &str, status: u16, headers: Vec<(String, String)>, body: Vec<u8>) {
        use base64::Engine;
        let mut h = Map::new();
        for (k, v) in headers {
            if matches!(k.to_ascii_lowercase().as_str(), "content-type" | "location" | "etag" | "accept-patch" | "link") {
                h.insert(k, Value::String(v));
            }
        }
        let body_b64 = if body.is_empty() { String::new() } else { base64::engine::general_purpose::STANDARD.encode(&body) };
        self.send_control(json!({"t": "http_res", "rid": rid, "status": status, "headers": h, "body_b64": body_b64}));
    }

    // -- phase 3: TURN credentials into MediaMTX --

    async fn refresh_turn_forever(self: Arc<Self>) {
        let mut failures: u32 = 0;
        loop {
            let mut delay = TURN_REFRESH_S;
            let mut failed = false;
            let (url, identity) = (format!("{}/v1/robots/{}/turn", self.backend_url, self.identity.robot_id), self.identity.clone());
            let (status, body) = match tokio::task::spawn_blocking(move || signed_get(&url, &identity)).await.unwrap_or_else(|e| Err(e.to_string())) {
                Ok(v) => v,
                Err(e) => {
                    debug!("turn: {e}");
                    (0, json!({}))
                }
            };
            if status == 200 {
                let servers = mediamtx_ice_servers(body.get("iceServers").unwrap_or(&Value::Null));
                if let Some(expires) = body.get("expires_at").and_then(Value::as_f64) {
                    delay = TURN_RETRY_S.max(TURN_REFRESH_S.min(((expires - now_unix() as f64) as i64) / 2));
                }
                if self.ice_applied.lock().unwrap().as_ref() != Some(&servers) {
                    let (api, patch) = (self.mediamtx_api_url.clone(), json!({"webrtcICEServers2": servers.clone()}));
                    let code = match tokio::task::spawn_blocking(move || patch_mediamtx_config(&api, &patch)).await.unwrap_or_else(|e| Err(e.to_string())) {
                        Ok(c) => c,
                        Err(e) => {
                            warning!("turn: mediamtx api unreachable: {e}");
                            0
                        }
                    };
                    if code == 200 {
                        info!("turn: {} ICE server(s) applied to mediamtx", servers.len());
                        *self.ice_applied.lock().unwrap() = Some(servers);
                    } else {
                        warning!("turn: mediamtx config patch failed: HTTP {code}");
                        failed = true;
                    }
                }
            } else {
                if status != 0 {
                    warning!("turn: HTTP {status} {}", body.get("error").filter(|e| !e.is_null()).map(|e| py_str(Some(e))).unwrap_or_else(|| py_dict(&body)));
                }
                failed = true;
            }
            if failed {
                delay = (TURN_RETRY_S << failures.min(20)).min(TURN_REFRESH_S);
                failures += 1;
            } else {
                failures = 0;
            }
            tokio::time::sleep(Duration::from_secs(delay.max(0) as u64)).await;
        }
    }

    /// `register`: until it works (network) or fails for good (409 / 401).
    async fn register(&self, wait: bool) -> bool {
        let token = std::env::var("MOWER_PROVISION_TOKEN").unwrap_or_default();
        if token.is_empty() {
            warning!("MOWER_PROVISION_TOKEN not set: skipping registration");
            return false;
        }
        loop {
            let (base, tok, identity, model) = (self.backend_url.clone(), token.clone(), self.identity.clone(), self.model.clone());
            let (status, body) = match tokio::task::spawn_blocking(move || register_once(&base, &tok, &identity, &model)).await.unwrap_or_else(|e| Err(e.to_string())) {
                Ok(v) => v,
                Err(e) => {
                    warning!("register: {e}");
                    (0, json!({}))
                }
            };
            match status {
                200 | 201 => {
                    info!("registered {} ({})", self.identity.robot_id, py_str(body.get("outcome")));
                    return true;
                }
                409 => {
                    error!("register: {} (another device_key holds this robot_id); relay will be refused", py_str(body.get("error")));
                    return false;
                }
                401 => {
                    error!("register: bad provision token");
                    return false;
                }
                0 => {}
                _ => warning!("register: HTTP {status} {}", body.get("error").filter(|e| !e.is_null()).map(|e| py_str(Some(e))).unwrap_or_else(|| py_dict(&body))),
            }
            if !wait {
                return false;
            }
            tokio::time::sleep(Duration::from_secs(REGISTER_RETRY_S)).await;
        }
    }

    /// `watch_topics_forever`: /robot/info and /robot/telemetry through the loopback bridge.
    async fn watch_topics_forever(self: Arc<Self>) {
        loop {
            let config = WebSocketConfig { max_message_size: Some(1 << 20), max_frame_size: Some(1 << 20), ..Default::default() };
            match self.rosbridge_url.as_str().into_client_request().map_err(|e| e.to_string()) {
                Ok(req) => match tokio_tungstenite::connect_async_with_config(req, Some(config), false).await {
                    Ok((mut ws, _)) => {
                        let subs = [
                            json!({"op": "subscribe", "topic": "/robot/info", "type": "std_msgs/msg/String"}),
                            json!({"op": "subscribe", "topic": "/robot/telemetry", "type": "std_msgs/msg/String", "throttle_rate": TELEMETRY_THROTTLE_MS}),
                        ];
                        let mut ok = true;
                        for s in subs {
                            if ws.send(WsMsg::Text(s.to_string())).await.is_err() {
                                ok = false;
                                break;
                            }
                        }
                        while ok {
                            match ws.next().await {
                                Some(Ok(WsMsg::Text(t))) => self.on_topic(&t),
                                Some(Ok(WsMsg::Binary(b))) => self.on_topic(&String::from_utf8_lossy(&b)),
                                Some(Ok(WsMsg::Ping(p))) => {
                                    let _ = ws.send(WsMsg::Pong(p)).await;
                                }
                                Some(Ok(_)) => {}
                                Some(Err(e)) => {
                                    debug!("rosbridge watch: {e}");
                                    break;
                                }
                                None => break,
                            }
                        }
                    }
                    Err(e) => debug!("rosbridge watch: {e}"),
                },
                Err(e) => debug!("rosbridge watch: {e}"),
            }
            tokio::time::sleep(Duration::from_secs(5)).await;
        }
    }

    fn on_topic(&self, message: &str) {
        let Ok(msg) = serde_json::from_str::<Value>(message) else { return };
        if msg.get("op").and_then(Value::as_str) != Some("publish") {
            return;
        }
        let Some(data_text) = msg.get("msg").and_then(|m| m.get("data")).and_then(Value::as_str) else { return };
        let Ok(data) = serde_json::from_str::<Value>(data_text) else { return };
        match msg.get("topic").and_then(Value::as_str) {
            Some("/robot/info") => *self.info.lock().unwrap() = Some(data),
            Some("/robot/telemetry") => *self.telemetry.lock().unwrap() = Some(data),
            _ => {}
        }
    }
}

/// Python `%s` of a dict body (`{'error': ...}` style).
fn py_dict(v: &Value) -> String {
    match v {
        Value::Object(m) => {
            let items: Vec<String> = m.iter().map(|(k, v)| format!("'{k}': {}", py_repr(v))).collect();
            format!("{{{}}}", items.join(", "))
        }
        other => py_repr(other),
    }
}

fn py_repr(v: &Value) -> String {
    match v {
        Value::Null => "None".into(),
        Value::Bool(true) => "True".into(),
        Value::Bool(false) => "False".into(),
        Value::String(s) => format!("'{s}'"),
        Value::Number(n) => n.to_string(),
        Value::Array(a) => format!("[{}]", a.iter().map(py_repr).collect::<Vec<_>>().join(", ")),
        Value::Object(_) => py_dict(v),
    }
}

// ------------------------------------------------------------------- main

struct Args {
    backend_url: Option<String>,
    state_dir: Option<String>,
    gate: String,
    rosbridge: String,
    model: String,
    mediamtx: String,
    mediamtx_api: String,
    log_level: String,
}

/// `argparse.parse_known_args`: known options, everything else ignored.
fn parse_args() -> Args {
    let env = |k: &str, d: &str| std::env::var(k).ok().filter(|v| !v.is_empty()).unwrap_or_else(|| d.to_string());
    let mut a = Args {
        backend_url: None,
        state_dir: None,
        gate: LOCAL_GATE.into(),
        rosbridge: LOCAL_ROSBRIDGE.into(),
        model: env("MOWER_MODEL", "lubancat"),
        mediamtx: env("MEDIAMTX_URL", LOCAL_MEDIAMTX),
        mediamtx_api: env("MEDIAMTX_API_URL", LOCAL_MEDIAMTX_API),
        log_level: "INFO".into(),
    };
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < argv.len() {
        let (key, inline) = match argv[i].split_once('=') {
            Some((k, v)) if k.starts_with("--") => (k.to_string(), Some(v.to_string())),
            _ => (argv[i].clone(), None),
        };
        let take = |i: &mut usize| -> Option<String> {
            if let Some(v) = inline.clone() {
                return Some(v);
            }
            *i += 1;
            argv.get(*i).cloned()
        };
        match key.as_str() {
            "--backend-url" => a.backend_url = take(&mut i),
            "--state-dir" => a.state_dir = take(&mut i),
            "--gate" => a.gate = take(&mut i).unwrap_or(a.gate),
            "--rosbridge" => a.rosbridge = take(&mut i).unwrap_or(a.rosbridge),
            "--model" => a.model = take(&mut i).unwrap_or(a.model),
            "--mediamtx" => a.mediamtx = take(&mut i).unwrap_or(a.mediamtx),
            "--mediamtx-api" => a.mediamtx_api = take(&mut i).unwrap_or(a.mediamtx_api),
            "--log-level" => a.log_level = take(&mut i).unwrap_or(a.log_level),
            _ => {}
        }
        i += 1;
    }
    a
}

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() {
    let args = parse_args();
    LOG_LEVEL.store(
        match args.log_level.to_ascii_uppercase().as_str() {
            "DEBUG" => 10,
            "WARNING" | "WARN" => 30,
            "ERROR" => 40,
            _ => 20,
        },
        Ordering::Relaxed,
    );
    let backend = args.backend_url.clone().filter(|b| !b.is_empty()).or_else(|| std::env::var("MOWER_BACKEND_URL").ok().filter(|b| !b.is_empty()));
    let Some(backend) = backend else {
        warning!("MOWER_BACKEND_URL not set: agent idle (development mode)");
        return;
    };
    let state_dir = state_dir_of(args.state_dir.as_deref());
    let Some(identity) = ensure_device_key(&state_dir) else {
        warning!("no identity.json in {state_dir}: agent idle");
        return;
    };
    let agent = Arc::new(Agent {
        identity,
        backend_url: backend.trim_end_matches('/').to_string(),
        // A gate bound to 0.0.0.0 is dialled on loopback.
        gate_url: args.gate.replace("://0.0.0.0:", "://127.0.0.1:"),
        rosbridge_url: args.rosbridge.clone(),
        model: args.model.clone(),
        mediamtx_url: args.mediamtx.clone(),
        mediamtx_api_url: args.mediamtx_api.clone(),
        ice_applied: Mutex::new(None),
        sessions: Mutex::new(HashMap::new()),
        relay: Mutex::new(None),
        info: Mutex::new(None),
        telemetry: Mutex::new(None),
    });

    agent.register(true).await;
    let tasks = vec![
        tokio::spawn(agent.clone().run_relay_forever()),
        tokio::spawn(agent.clone().watch_topics_forever()),
        tokio::spawn(agent.clone().refresh_turn_forever()),
    ];
    let mut sigterm = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate()).expect("signal handler");
    tokio::select! {
        _ = tokio::signal::ctrl_c() => {}
        _ = sigterm.recv() => {}
    }
    for t in tasks {
        t.abort();
    }
    agent.close_all_sessions().await;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn whep_paths() {
        assert!(whep_path("/front/whep"));
        assert!(whep_path("/front/whep/sess-1.a"));
        assert!(!whep_path("/front/whip"));
        assert!(!whep_path("/front/whep/"));
        assert!(!whep_path("/front/whep/a/b"));
        assert!(!whep_path("front/whep"));
        assert!(!whep_path("/fr ont/whep"));
    }

    #[test]
    fn http_to_ws_schemes() {
        assert_eq!(http_to_ws("https://api.example"), "wss://api.example");
        assert_eq!(http_to_ws("http://127.0.0.1:8787/"), "ws://127.0.0.1:8787/");
    }

    #[test]
    fn ice_servers_mapping() {
        let servers = json!([
            {"urls": ["stun:stun.example:3478", "turn:turn.example:3478?transport=udp", "turn:turn.example:3478?transport=tcp", "turns:turn.example:5349?transport=tcp"], "username": "u", "credential": "p"},
            {"urls": "turn:other.example:3478?transport=udp", "username": "x", "credential": "y"}
        ]);
        let out = mediamtx_ice_servers(&servers);
        assert_eq!(out.len(), 3);
        assert_eq!(out[0]["url"], "turn:turn.example:3478?transport=udp");
        assert_eq!(out[1]["url"], "turn:turn.example:3478?transport=tcp");
        assert_eq!(out[2]["username"], "x");
        assert_eq!(out[2]["password"], "y");
        assert_eq!(out[2]["clientOnly"], false);
    }

    #[test]
    fn mac_matches_python_vector() {
        // identity.compute_mac('AAAA...', 'MW-7K3Q9P', '@robot', 1700000000, 'bb..')
        let secret = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";
        let mac = compute_mac(secret, "MW-7K3Q9P", "@robot", 1700000000, &"b".repeat(32));
        assert_eq!(mac.len(), 64);
        assert_eq!(secret_bytes(secret).len(), 20);
        assert_eq!(secret_bytes(&new_device_key()).len(), 32);
    }
}
