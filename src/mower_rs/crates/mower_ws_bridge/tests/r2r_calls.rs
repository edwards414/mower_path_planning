//! The r2r backend's calls against real DDS (E3.3): a call to a slow server
//! holds up neither the client's other frames nor its other calls, on
//! ws_bridge's own r2r node and its spin thread, as src/bus_tests.rs shows
//! for the bus backend (with the cap and the order of every frame).
//!
//! - **Isolation:** `ROS_DOMAIN_ID` 106, one test at a time; peers are nodes
//!   of the process's one participant.
//! - **Time** is real: the slow server answers 10 s after a request.

use std::future::Future;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock, PoisonError};
use std::time::{Duration, Instant};

use futures::{SinkExt, StreamExt};
use mower_rs_common::{ModuleCtx, Shutdown};
use r2r::geometry_msgs::msg::TwistStamped;
use r2r::std_msgs::msg::String as Text;
use r2r::std_srvs::srv::Trigger;
use r2r::QosProfile;
use serde_json::{json, Value};
use tokio_tungstenite::tungstenite::Message;

const WAIT: Duration = Duration::from_secs(20);
/// How long the slow server takes to answer.
const SLOW: Duration = Duration::from_secs(10);

/// The process's r2r context, on domain 106 and rmw_cyclonedds_cpp.
fn context() -> r2r::Context {
    static CONTEXT: OnceLock<r2r::Context> = OnceLock::new();
    CONTEXT
        .get_or_init(|| {
            // rcl reads both when the context is created, and only then.
            std::env::set_var("ROS_DOMAIN_ID", "106");
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

/// A node of the process that is not ws_bridge's, spun on a thread of its
/// own.
struct Peer {
    node: Arc<Mutex<r2r::Node>>,
    running: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl Peer {
    fn new(name: &str) -> Peer {
        let node = Arc::new(Mutex::new(r2r::Node::create(context(), name, "").unwrap()));
        let running = Arc::new(AtomicBool::new(true));
        let thread = {
            let (node, running) = (node.clone(), running.clone());
            std::thread::spawn(move || {
                while running.load(Ordering::Acquire) {
                    node.lock().unwrap().spin_once(Duration::from_millis(5));
                    std::thread::sleep(Duration::from_millis(1));
                }
            })
        };
        Peer { node, running, thread: Some(thread) }
    }

    fn with<R>(&self, f: impl FnOnce(&mut r2r::Node) -> R) -> R {
        f(&mut self.node.lock().unwrap())
    }
}

impl Drop for Peer {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Release);
        if let Some(thread) = self.thread.take() {
            thread.join().unwrap();
        }
    }
}

type Socket = tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>>;

async fn ws_connect(url: &str) -> Socket {
    let deadline = Instant::now() + WAIT;
    loop {
        match tokio_tungstenite::connect_async(url).await {
            Ok((ws, _)) => return ws,
            Err(e) => assert!(Instant::now() < deadline, "ws_bridge does not listen: {e}"),
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
}

async fn ws_send(ws: &mut Socket, frame: Value) {
    ws.send(Message::Text(frame.to_string())).await.unwrap();
}

/// The next frame that is not a topic's message.
async fn next_answer(ws: &mut Socket) -> Value {
    loop {
        let frame = tokio::time::timeout(WAIT, ws.next()).await.expect("a frame").expect("the connection");
        if let Message::Text(text) = frame.unwrap() {
            let frame: Value = serde_json::from_str(&text).unwrap();
            if frame["op"] != "publish" {
                return frame;
            }
        }
    }
}

/// The app's joystick frame number `k`: its stamp's nanoseconds are `k`.
fn joystick(k: u32) -> Value {
    let msg = json!({"header": {"stamp": {"sec": 7, "nanosec": k}, "frame_id": "manual-1"},
                     "twist": {"linear": {"x": 0.2, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}}});
    json!({"op": "publish", "topic": "/app_joy_cmd", "msg": msg})
}

/// While a call waits for a slow DDS server (10 s), the same client's
/// publishes, a subscribe and calls to another server and to the same one are
/// handled at once; the slow answers then come with their `id`s.
#[test]
fn a_slow_call_holds_up_nothing_behind_it_on_the_r2r_backend() {
    within(async {
        let status = Peer::new("e33_test_robot_status");
        let mut updates = status.with(|n| n.create_service::<Trigger::Service>("/system/update", QosProfile::services_default())).unwrap();
        let slow = tokio::spawn(async move {
            while let Some(request) = updates.next().await {
                tokio::spawn(async move {
                    tokio::time::sleep(SLOW).await;
                    let _ = request.respond(Trigger::Response { success: true, message: "updated".into() });
                });
            }
        });
        let telemetry = status.with(|n| n.create_publisher::<Text>("/robot/telemetry", QosProfile::default())).unwrap();
        let nav = Peer::new("e33_test_nav_action_server");
        let mut checks = nav.with(|n| n.create_service::<Trigger::Service>("/check_nav_status", QosProfile::services_default())).unwrap();
        let fast = tokio::spawn(async move {
            while let Some(request) = checks.next().await {
                let _ = request.respond(Trigger::Response { success: true, message: "idle".into() });
            }
        });
        let guard = Peer::new("e33_test_manual_velocity_guard");
        let mut joy = guard.with(|n| n.subscribe::<TwistStamped>("/app_joy_cmd", QosProfile::default().keep_last(100))).unwrap();

        let port = std::net::TcpListener::bind(("127.0.0.1", 0)).unwrap().local_addr().unwrap().port();
        let policy = concat!(env!("CARGO_MANIFEST_DIR"), "/../../../mower_bringup/config/ws_bridge.yaml");
        let argv = format!("--address 127.0.0.1 --port {port} --loopback-port none --policy {policy} --state-dir /nonexistent/ws_bridge_test");
        let shutdown = Shutdown::new();
        let m = ModuleCtx::new("mower_ws_bridge", shutdown.clone()).with_args(argv.split_whitespace().map(str::to_string).collect());
        let door = mower_ws_bridge::run(context(), m);
        let check = async {
            let mut app = ws_connect(&format!("ws://127.0.0.1:{port}")).await;
            ws_send(&mut app, json!({"op": "call_service", "id": "slow-1", "service": "/system/update", "args": {}})).await;
            let asked = Instant::now();

            // the joystick: once the bridge's writer has matched the guard,
            // every message, in order
            ws_send(&mut app, json!({"op": "advertise", "topic": "/app_joy_cmd", "type": "geometry_msgs/msg/TwistStamped"})).await;
            let mut k = 0;
            let first = loop {
                k += 1;
                ws_send(&mut app, joystick(k)).await;
                if let Ok(Some(msg)) = tokio::time::timeout(Duration::from_millis(50), joy.next()).await {
                    break msg.header.stamp.nanosec;
                }
                assert!(asked.elapsed() < WAIT, "the joystick never reached DDS");
            };
            for k in k + 1..=k + 20 {
                ws_send(&mut app, joystick(k)).await;
            }
            let mut seen = vec![first];
            while seen.len() < 21 {
                let msg = tokio::time::timeout(WAIT, joy.next()).await.expect("every joystick message").unwrap();
                seen.push(msg.header.stamp.nanosec);
            }
            assert_eq!(seen, (first..first + 21).collect::<Vec<_>>(), "in order, none lost");

            // a subscribe
            ws_send(&mut app, json!({"op": "subscribe", "topic": "/robot/telemetry", "type": "std_msgs/msg/String"})).await;
            let deadline = Instant::now() + WAIT;
            loop {
                telemetry.publish(&Text { data: "{}".into() }).unwrap();
                if let Ok(Some(Ok(Message::Text(text)))) = tokio::time::timeout(Duration::from_millis(50), app.next()).await {
                    assert_eq!(text, json!({"op": "publish", "topic": "/robot/telemetry", "msg": {"data": "{}"}}).to_string());
                    break;
                }
                assert!(Instant::now() < deadline, "no telemetry");
            }
            ws_send(&mut app, json!({"op": "unsubscribe", "topic": "/robot/telemetry"})).await;

            // a call to another server, one to the same slow server, and ws_bridge's own
            ws_send(&mut app, json!({"op": "call_service", "id": "fast", "service": "/check_nav_status", "args": {}})).await;
            let fast_answer = json!({"op": "service_response", "id": "fast", "service": "/check_nav_status",
                                     "values": {"message": "idle", "success": true}, "result": true});
            assert_eq!(next_answer(&mut app).await, fast_answer);
            ws_send(&mut app, json!({"op": "call_service", "id": "slow-2", "service": "/system/update", "args": {}})).await;
            let asked_again = Instant::now();
            ws_send(&mut app, json!({"op": "call_service", "id": "time", "service": "/rosapi/get_time", "args": {}})).await;
            assert_eq!(next_answer(&mut app).await["id"], "time");
            // all of it while slow-1 waited: its answer would have come first
            let handled = asked.elapsed();
            assert!(handled < SLOW, "everything behind the slow call took {handled:?}");

            // the slow answers, each with its id, each 10 s after its request
            let updated = |id: &str| {
                json!({"op": "service_response", "id": id, "service": "/system/update", "values": {"message": "updated", "success": true}, "result": true})
            };
            let mut ids = Vec::new();
            for _ in 0..2 {
                let answer = next_answer(&mut app).await;
                let sent = match answer["id"].as_str() {
                    Some("slow-1") => asked,
                    Some("slow-2") => asked_again,
                    other => panic!("an answer to {other:?}: {answer}"),
                };
                assert_eq!(answer, updated(answer["id"].as_str().unwrap()));
                assert!(sent.elapsed() >= SLOW, "{} after {:?}", answer["id"], sent.elapsed());
                ids.push(answer["id"].as_str().unwrap().to_string());
            }
            ids.sort();
            assert_eq!(ids, ["slow-1", "slow-2"]);
            shutdown.trigger();
        };
        let (result, ()) = tokio::join!(door, check);
        result.unwrap();
        slow.abort();
        fast.abort();
    });
}
