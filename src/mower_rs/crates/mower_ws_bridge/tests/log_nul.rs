//! A NUL in a logged message (vendor/r2r's patch 4): it is written out as `\0`,
//! nothing panics, and logging goes on. A test binary of its own, because r2r's
//! logger is the process's. The bridge's own log lines are checked on both
//! backends.
//!
//! - **Isolation:** `ROS_DOMAIN_ID` 107, one test at a time.
//! - **Time** is real: only a frame that must come is waited for.

use std::future::Future;
use std::process::Command;
use std::sync::{Mutex, MutexGuard, OnceLock, PoisonError};
use std::time::{Duration, Instant};

use futures::{SinkExt, StreamExt};
use mower_rs_common::{ModuleCtx, Shutdown};
use serde_json::{json, Value};
use tokio_tungstenite::tungstenite::client::IntoClientRequest;
use tokio_tungstenite::tungstenite::Message;

const WAIT: Duration = Duration::from_secs(20);

/// Set when this binary runs itself, to read what its logger writes.
const AS_THE_LOGGING_CHILD: &str = "LOG_NUL_CHILD";

/// The process's r2r context, on domain 107 and rmw_cyclonedds_cpp.
fn context() -> r2r::Context {
    static CONTEXT: OnceLock<r2r::Context> = OnceLock::new();
    CONTEXT
        .get_or_init(|| {
            // rcl reads both when the context is created, and only then.
            std::env::set_var("ROS_DOMAIN_ID", "107");
            std::env::set_var("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp");
            r2r::Context::create().expect("an rcl context (source /opt/ros/jazzy/setup.bash)")
        })
        .clone()
}

/// One test at a time, on a runtime of its own, within a bound only a hang
/// reaches.
fn within(test: impl Future<Output = ()>) {
    static ONE_AT_A_TIME: Mutex<()> = Mutex::new(());
    let _serial: MutexGuard<'_, ()> = ONE_AT_A_TIME.lock().unwrap_or_else(PoisonError::into_inner);
    let runtime = tokio::runtime::Builder::new_multi_thread().worker_threads(2).enable_all().build().unwrap();
    runtime.block_on(async {
        tokio::time::timeout(Duration::from_secs(120), test).await.expect("the test hung");
    });
}

/// A state dir with a robot's identity, which turns the gate on.
struct StateDir(std::path::PathBuf);

impl StateDir {
    fn paired() -> StateDir {
        let dir = std::env::temp_dir().join(format!("ws_bridge_log_nul_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("identity.json"), r#"{"robot_id": "MW-LOGNUL", "secret": "mfrggzdf"}"#).unwrap();
        StateDir(dir)
    }

    fn path(&self) -> &str {
        self.0.to_str().unwrap()
    }
}

impl Drop for StateDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

type Socket = tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>>;

/// A client through the gate: the app's hand-shake with the robot's secret.
async fn through_the_gate(url: &str, state: &StateDir, nonce: u32) -> Socket {
    let identity = mower_ws_bridge::auth::Identity::load(state.path()).unwrap().expect("an identity");
    let t = mower_ws_bridge::auth::now_unix();
    let nonce = format!("{nonce:032x}");
    let mac = identity.compute_mac("log_nul_test", t, &nonce);
    let deadline = Instant::now() + WAIT;
    loop {
        let mut request = url.into_client_request().unwrap();
        for (k, v) in [("x-mower-robot", identity.robot_id.clone()), ("x-mower-client", "log_nul_test".to_string()), ("x-mower-time", t.to_string()), ("x-mower-nonce", nonce.clone()), ("x-mower-mac", mac.clone())] {
            request.headers_mut().insert(k, v.parse().unwrap());
        }
        match tokio_tungstenite::connect_async(request).await {
            Ok((ws, _)) => return ws,
            Err(e) => assert!(Instant::now() < deadline, "the gate does not take the client: {e}"),
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
}

async fn send(ws: &mut Socket, frame: Value) {
    ws.send(Message::Text(frame.to_string())).await.unwrap();
}

async fn next(ws: &mut Socket) -> Value {
    loop {
        let frame = tokio::time::timeout(WAIT, ws.next()).await.expect("a frame").expect("the connection");
        match frame.expect("the connection") {
            Message::Text(text) => return serde_json::from_str(&text).unwrap(),
            Message::Ping(_) | Message::Pong(_) => {}
            other => panic!("not a text frame: {other:?}"),
        }
    }
}

/// The connection and the bridge's logging still work: a call answered on it
/// (which also shows the frames sent before it were handled), and an error
/// answered and logged. Returns the frames that came before the answer.
async fn still_works(ws: &mut Socket, id: &str) -> Vec<Value> {
    send(ws, json!({"op": "call_service", "id": id, "service": "/rm_rf", "args": {}})).await;
    let mut before = Vec::new();
    loop {
        let frame = next(ws).await;
        if frame["id"] == id && frame["op"] == "service_response" {
            assert_eq!(frame["values"], "service /rm_rf is not allowed");
            return before;
        }
        before.push(frame);
    }
}

/// Every name and type a frame can spell with a NUL: each answered as the same
/// text without one would be, and the connection, a later client and logging
/// all go on.
#[test]
fn a_nul_in_a_name_or_a_type_is_refused_and_everything_goes_on() {
    within(async {
        for backend in ["r2r", "bus"] {
            let state = StateDir::paired();
            let free_port = || std::net::TcpListener::bind(("127.0.0.1", 0)).unwrap().local_addr().unwrap().port();
            let port = free_port();
            let policy = concat!(env!("CARGO_MANIFEST_DIR"), "/../../../mower_bringup/config/ws_bridge.yaml");
            let argv = format!("--address 127.0.0.1 --port {port} --loopback-port none --policy {policy} --state-dir {} --backend {backend}", state.path());
            let shutdown = Shutdown::new();
            let m = ModuleCtx::new("mower_ws_bridge", shutdown.clone()).with_args(argv.split_whitespace().map(str::to_string).collect());
            let door = mower_ws_bridge::run(context(), m);
            let check = async {
                let url = format!("ws://127.0.0.1:{port}");
                let mut phone = through_the_gate(&url, &state, 1).await;

                // a service name
                let name = "/a\u{0}b";
                send(&mut phone, json!({"op": "call_service", "id": "c", "service": name, "args": {}})).await;
                let answer = next(&mut phone).await;
                assert_eq!(answer, json!({"op": "service_response", "id": "c", "service": name, "values": format!("service {name} is not allowed"), "result": false}), "{backend}");

                // a topic name, to read and to write
                for (op, id) in [("subscribe", "s"), ("advertise", "a"), ("publish", "p")] {
                    send(&mut phone, json!({"op": op, "id": id, "topic": name, "msg": {}})).await;
                    let status = next(&mut phone).await;
                    assert_eq!(status, json!({"op": "status", "level": "error", "id": id, "msg": format!("{op}: {name} is not allowed")}), "{backend}");
                }

                // an operation
                send(&mut phone, json!({"op": name, "id": "o"})).await;
                assert_eq!(next(&mut phone).await["level"], "error");

                // the type of a name the policy allows: refused on r2r, which
                // looks it up, and taken on the bus, which logs what it was given
                send(&mut phone, json!({"op": "subscribe", "id": "t", "topic": "/mower_recorder/status", "type": "std_msgs/msg/S\u{0}tring"})).await;
                let came = still_works(&mut phone, "after").await;
                match backend {
                    "r2r" => assert_eq!(came.iter().map(|f| f["id"].as_str().unwrap_or("")).collect::<Vec<_>>(), ["t"], "{came:?}"),
                    _ => assert!(came.is_empty(), "{came:?}"),
                }

                // a client that comes later is logged like any other
                let mut later = through_the_gate(&url, &state, 2).await;
                assert!(still_works(&mut later, "later").await.is_empty());
                // and so is anybody else
                r2r::log_warn!("log_nul_test", "logging goes on after the {backend} backend's frames");
                shutdown.trigger();
            };
            let (result, ()) = tokio::join!(door, check);
            result.unwrap();
        }
    });
}

/// A panic under the logger's guard does not end the process's logging, which
/// a poisoned lock did. In a process of its own, as it means to poison it.
#[test]
fn a_panic_under_the_loggers_guard_does_not_end_logging() {
    if std::env::var_os(AS_THE_LOGGING_CHILD).is_some() {
        let _ = std::thread::spawn(|| {
            let _guard = r2r::log_guard();
            panic!("under the logger's guard");
        })
        .join();
        r2r::log_warn!("log_nul_test", "logging goes on");
        return;
    }
    let me = std::env::current_exe().unwrap();
    let test = "a_panic_under_the_loggers_guard_does_not_end_logging";
    let out = Command::new(me).args(["--exact", test, "--nocapture"]).env(AS_THE_LOGGING_CHILD, "1").output().unwrap();
    let (stdout, stderr) = (String::from_utf8_lossy(&out.stdout), String::from_utf8_lossy(&out.stderr));
    assert!(out.status.success(), "{stdout}\n{stderr}");
    assert!(format!("{stdout}{stderr}").contains("logging goes on"), "{stdout}\n{stderr}");
}

/// The logger itself, in a process of this binary whose output can be read.
#[test]
fn a_nul_is_written_out_and_the_next_message_still_logs() {
    if std::env::var_os(AS_THE_LOGGING_CHILD).is_some() {
        r2r::log_warn!("log_nul_test", "the message a\u{0}b ends here");
        r2r::log_warn!("log\u{0}nul\u{0}test", "the logger's name has them");
        r2r::log("the file's name has one", "log_nul_test", "lo\u{0}g.rs", 1, r2r::LogSeverity::Warn);
        r2r::log_warn!("log_nul_test", "the next message");
        return;
    }
    let me = std::env::current_exe().unwrap();
    let test = "a_nul_is_written_out_and_the_next_message_still_logs";
    let out = Command::new(me).args(["--exact", test, "--nocapture"]).env(AS_THE_LOGGING_CHILD, "1").output().unwrap();
    let (stdout, stderr) = (String::from_utf8_lossy(&out.stdout), String::from_utf8_lossy(&out.stderr));
    assert!(out.status.success(), "{stdout}\n{stderr}");
    let written = format!("{stdout}{stderr}");
    for line in ["the message a\\0b ends here", "[log\\0nul\\0test]: the logger's name has them", "the file's name has one", "the next message"] {
        assert!(written.contains(line), "{line:?} in:\n{written}");
    }
}
