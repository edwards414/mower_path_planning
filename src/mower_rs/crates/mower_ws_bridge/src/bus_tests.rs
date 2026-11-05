//! The bus backend end to end, without DDS (step E3.1; calls in flight,
//! E3.3): ws_bridge's own listener, gate and protocol, the bus hub on a
//! `Bus` with typed endpoints standing in for mower_rsd's modules and its
//! bus bridge, a manual clock, and WebSocket clients in the test.
//!
//! - **Real time** only carries frames over loopback: a frame that must come
//!   is waited for up to `WAIT`, one that must not for `QUIET`.
//! - **Bus time** is a manual clock: the throttle, the 5 s and 30 s bounds and
//!   the 10 s linger move only when a test moves it, once the deadline it
//!   expects is pending.

use std::any::Any;
use std::collections::{HashMap, VecDeque};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use futures::{SinkExt, StreamExt};
use mower_bus::{Bus, Clock, ManualClock, Message, Publisher, Qos, Scope, Subscriber, Time};
use mower_rs_common::{ModuleCtx, ModuleResult, Registry, Shutdown};
use r2r::geometry_msgs::msg::{PoseStamped, TwistStamped};
use r2r::mower_interface::action::Waypoint;
use r2r::mower_interface::msg::CoverageProgress;
use r2r::mower_interface::srv::{
    EditZone, GetCoverageProgress, ImportImageMask, PidAutotune, ResumeCoverage, SiteOp, ZoneExecPath, ZoneSequence,
};
use r2r::rcl_interfaces::srv::SetParameters;
use r2r::sensor_msgs::msg::{BatteryState, NavSatFix};
use r2r::std_msgs::msg::{Bool, Header, String as Text};
use r2r::std_srvs::srv::Trigger;
use serde_json::{json, Value};
use tokio::net::TcpStream;
use tokio::sync::mpsc;
use tokio_tungstenite::tungstenite::client::IntoClientRequest;
use tokio_tungstenite::tungstenite::Message as WsMessage;
use tokio_tungstenite::{MaybeTlsStream, WebSocketStream};

use crate::bus_hub::{manifest_names, BusHub, LINGER};
use crate::client::{ClientHandle, MAX_CALLS_IN_FLIGHT};
use crate::config::Policy;
use crate::golden::fixtures;
use crate::hub::Hub;
use crate::{load_loopback, parse_args, serve};

/// The reader of `manifest_names`' topics, R TL 10, for the check below.
const READER_QOS: Qos = Qos::latched().keep_last(10);

/// Real time for a frame that must come.
const WAIT: Duration = Duration::from_secs(10);
/// Real time in which a frame that must not come does not.
const QUIET: Duration = Duration::from_millis(200);
/// 2026-10-27 08:00:00 UTC, the manual clock's wall time at start.
const T0: Time = Time::from_ns(1_793_088_000_000_000_000);

fn policy_file() -> String {
    concat!(env!("CARGO_MANIFEST_DIR"), "/../../../mower_bringup/config/ws_bridge.yaml").to_string()
}

fn free_port() -> u16 {
    std::net::TcpListener::bind(("127.0.0.1", 0)).unwrap().local_addr().unwrap().port()
}

fn argv(line: &str) -> Vec<String> {
    line.split_whitespace().map(str::to_string).collect()
}

async fn eventually(what: &str, mut done: impl FnMut() -> bool) {
    let deadline = Instant::now() + WAIT;
    while !done() {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
}

/// ws_bridge on the bus backend, as mower_rsd runs it, on a port of its own
/// (127.0.0.1, so no LAN listener; no loopback listener; no identity, so
/// the gate is open).
struct Door {
    bus: Bus,
    manual: ManualClock,
    /// The manual clock's wall time at start, one of its own per door.
    start: Time,
    m: ModuleCtx,
    hub: BusHub,
    url: String,
    task: tokio::task::JoinHandle<ModuleResult>,
}

/// Doors opened so far in the process: each starts at a wall time of its
/// own, by which a test tells its door from another test's.
static DOORS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

impl Door {
    async fn open() -> Door {
        loop {
            let n = DOORS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            if let Some(door) = Door::try_open(T0 + Duration::from_secs(n)).await {
                return door;
            }
        }
    }

    /// A door on a port that was free a moment ago; `None` if another test
    /// took the port meanwhile (`serve` could not bind it, or the port
    /// answers with another door's clock).
    async fn try_open(start: Time) -> Option<Door> {
        let bus = Bus::new();
        let (clock, manual) = Clock::manual(start);
        let port = free_port();
        let line = format!(
            "--address 127.0.0.1 --port {port} --loopback-port none --policy {} --state-dir /nonexistent/ws_bridge_test --backend bus",
            policy_file()
        );
        let args = parse_args(&argv(&line)).unwrap();
        let policy = Arc::new(Policy::from_yaml(&std::fs::read_to_string(policy_file()).unwrap()).unwrap());
        let m = ModuleCtx::new("mower_ws_bridge", Shutdown::new()).with_bus(bus.scope("mower_ws_bridge")).with_clock(clock.clone());
        let hub = BusHub::new(m.bus.clone(), clock, manifest_names(&m).unwrap(), "mower_ws_bridge");
        let task = tokio::spawn(serve(args, policy, None, Hub::Bus(hub.clone()), "mower_ws_bridge".into(), m.clone()));
        let url = format!("ws://127.0.0.1:{port}");
        let door = Door { bus, manual, start, m, hub, url, task };
        let deadline = Instant::now() + WAIT;
        while TcpStream::connect(("127.0.0.1", port)).await.is_err() || door.task.is_finished() {
            if door.task.is_finished() {
                return None;
            }
            assert!(Instant::now() < deadline, "ws_bridge did not listen");
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        let mut probe = door.client().await;
        probe.call("which", "/rosapi/get_time", json!({})).await;
        let time = probe.recv_json().await;
        let (sec, nanosec) = start.to_parts();
        if time["values"]["time"] != json!({"sec": sec, "nanosec": nanosec}) || door.task.is_finished() {
            door.m.shutdown.trigger();
            return None;
        }
        Some(door)
    }

    async fn client(&self) -> Ws {
        let (stream, _) = tokio_tungstenite::connect_async(self.url.as_str()).await.unwrap();
        Ws { stream }
    }

    fn now(&self) -> mower_bus::Mono {
        self.manual.clock().mono()
    }

    /// Wait until a deadline `d` from now is the next one on the clock: the
    /// bridge waits on it.
    async fn deadline_in(&self, d: Duration) {
        let at = self.now() + d;
        eventually(&format!("a deadline {d:?} away"), || self.manual.next_deadline() == Some(at)).await;
    }

    /// The scopes of ws_bridge's readers on `topic`.
    fn readers(&self, topic: &str) -> usize {
        self.bus.topic(topic).map_or(0, |t| t.subscribers.iter().filter(|s| *s == "mower_ws_bridge").count())
    }

    async fn close(self) {
        self.m.shutdown.trigger();
        tokio::time::timeout(WAIT, self.task).await.expect("serve returned").unwrap().unwrap();
        self.m.bus.close();
    }
}

/// One WebSocket client.
struct Ws {
    stream: WebSocketStream<MaybeTlsStream<TcpStream>>,
}

impl Ws {
    async fn send(&mut self, frame: Value) {
        self.stream.send(WsMessage::Text(frame.to_string())).await.unwrap();
    }

    /// The next text frame, byte for byte.
    async fn recv(&mut self) -> String {
        let next = async {
            loop {
                match self.stream.next().await {
                    Some(Ok(WsMessage::Text(text))) => return text,
                    Some(Ok(WsMessage::Ping(_) | WsMessage::Pong(_))) => {}
                    other => panic!("no text frame: {other:?}"),
                }
            }
        };
        tokio::time::timeout(WAIT, next).await.expect("a frame")
    }

    async fn recv_json(&mut self) -> Value {
        serde_json::from_str(&self.recv().await).unwrap()
    }

    /// Nothing comes for a while.
    async fn quiet(&mut self) {
        if let Ok(frame) = tokio::time::timeout(QUIET, self.stream.next()).await {
            panic!("an unexpected frame: {frame:?}");
        }
    }

    async fn subscribe(&mut self, topic: &str, throttle_ms: u64) {
        self.send(json!({"op": "subscribe", "topic": topic, "throttle_rate": throttle_ms})).await;
    }

    async fn call(&mut self, id: &str, service: &str, args: Value) {
        self.send(json!({"op": "call_service", "id": id, "service": service, "args": args})).await;
    }
}

fn publish_frame(topic: &str, msg: Value) -> String {
    json!({"op": "publish", "topic": topic, "msg": msg}).to_string()
}

fn status(id: Option<&str>, msg: &str) -> String {
    let mut doc = json!({"op": "status", "level": "error", "msg": msg});
    if let Some(id) = id {
        doc["id"] = json!(id);
    }
    doc.to_string()
}

fn answer(id: &str, service: &str, values: Value, result: bool) -> String {
    json!({"op": "service_response", "id": id, "service": service, "values": values, "result": result}).to_string()
}

fn text(data: &str) -> Text {
    Text { data: data.to_string() }
}

/// A server of `name` answering each request with `answer(request)`.
fn serve_with<Req: Message, Resp: Message>(
    scope: &Scope, name: &str, answer: impl Fn(&Req) -> Resp + Send + 'static,
) -> tokio::task::JoinHandle<()> {
    let mut service = scope.service::<Req, Resp>(name).unwrap();
    tokio::spawn(async move {
        while let Some(request) = service.next().await {
            let response = answer(&request.message);
            let _ = request.respond(response);
        }
    })
}

fn trigger(message: &str) -> impl Fn(&Trigger::Request) -> Trigger::Response + Send + 'static {
    let message = message.to_string();
    move |_| Trigger::Response { success: true, message: message.clone() }
}

// ---------------------------------------------------------------------------
// The operations and their texts (E3_DESIGN 1.7)
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn subscribe_answers_as_the_r2r_backend_did_and_refuses_unknown_names() {
    let door = Door::open().await;
    let status_module = door.bus.scope("robot_status");
    let online = status_module.publisher::<Bool>("/robot/online", Qos::latched()).unwrap();
    online.publish(&Bool { data: true }).unwrap();
    let mut app = door.client().await;

    // the latched value, from the writer's history, through the throttle
    app.send(json!({"op": "subscribe", "topic": "/robot/online", "type": "std_msgs/msg/Bool", "throttle_rate": 200})).await;
    assert_eq!(app.recv().await, publish_frame("/robot/online", json!({"data": true})));

    app.send(json!({"op": "subscribe", "id": "s1"})).await;
    assert_eq!(app.recv().await, status(Some("s1"), "subscribe: missing topic"));
    app.send(json!({"op": "subscribe", "id": "sub:/odom", "topic": "/odom"})).await;
    assert_eq!(app.recv().await, status(Some("sub:/odom"), "subscribe: /odom is not allowed"));
    // allowed by a glob, but a name nothing has declared (E3 decision 7)
    app.send(json!({"op": "subscribe", "id": "s2", "topic": "/adapter/no_such_layer"})).await;
    assert_eq!(app.recv().await, status(Some("s2"), "subscribe: cannot infer type of /adapter/no_such_layer: not advertised yet"));
    // a hint alone types nothing: /gps/fix has no writer
    app.send(json!({"op": "subscribe", "id": "s3", "topic": "/gps/fix", "type": "sensor_msgs/msg/NavSatFix"})).await;
    assert_eq!(app.recv().await, status(Some("s3"), "subscribe: cannot infer type of /gps/fix: not advertised yet"));
    // another robot's namespace, on a shared graph (the app's fleet)
    app.send(json!({"op": "subscribe", "id": "s4", "topic": "/mw_2/online", "type": "std_msgs/msg/Bool"})).await;
    assert_eq!(app.recv().await, status(Some("s4"), "subscribe: cannot infer type of /mw_2/online: not advertised yet"));
    // a hint that is not the topic's type
    app.send(json!({"op": "subscribe", "id": "s5", "topic": "/robot/online", "type": "std_msgs/msg/String"})).await;
    assert_eq!(app.recv().await, status(Some("s5"), "subscribe: subscribe /robot/online (std_msgs/msg/String) failed: the topic is std_msgs/msg/Bool"));
    app.quiet().await;

    // A name of the manifest no typed endpoint has fixed yet is taken, and
    // delivers once a writer comes.
    app.send(json!({"op": "subscribe", "id": "s6", "topic": "/mower_recorder/status", "type": "std_msgs/msg/String"})).await;
    app.quiet().await;
    let recorder = door.bus.scope("recorder_manager");
    let rec = recorder.publisher::<Text>("/mower_recorder/status", Qos::latched()).unwrap();
    rec.publish(&text("{\"recording\":false}")).unwrap();
    assert_eq!(app.recv().await, publish_frame("/mower_recorder/status", json!({"data": "{\"recording\":false}"})));

    // unsubscribe ends the delivery, with no answer
    app.send(json!({"op": "unsubscribe", "topic": "/mower_recorder/status"})).await;
    app.quiet().await;
    rec.publish(&text("{\"recording\":true}")).unwrap();
    app.quiet().await;
    door.close().await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn publish_answers_as_the_r2r_backend_did() {
    let door = Door::open().await;
    // the guard's reader, as the bus bridge's to_ros reader stands for it
    let guard = door.bus.scope("manual_velocity_guard");
    let mut joy = guard.subscribe::<TwistStamped>("/app_joy_cmd", Qos::default().keep_last(1)).unwrap();
    let mut app = door.client().await;

    app.send(json!({"op": "advertise", "id": "adv:/app_joy_cmd", "topic": "/app_joy_cmd", "type": "geometry_msgs/msg/TwistStamped"})).await;
    let frame = json!({"header": {"stamp": {"sec": 7, "nanosec": 5}, "frame_id": "manual-1"},
                       "twist": {"linear": {"x": 0.3, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": -0.5}}});
    app.send(json!({"op": "publish", "topic": "/app_joy_cmd", "msg": frame})).await;
    let got = tokio::time::timeout(WAIT, joy.next()).await.unwrap().unwrap();
    assert_eq!((got.header.frame_id.as_str(), got.twist.linear.x, got.twist.angular.z), ("manual-1", 0.3, -0.5));
    app.send(json!({"op": "unadvertise", "topic": "/app_joy_cmd"})).await;

    app.send(json!({"op": "advertise", "id": "a1", "topic": "/cmd_vel", "type": "geometry_msgs/msg/Twist"})).await;
    assert_eq!(app.recv().await, status(Some("a1"), "advertise: /cmd_vel is not allowed"));
    app.send(json!({"op": "advertise", "id": "a2"})).await;
    assert_eq!(app.recv().await, status(Some("a2"), "advertise: missing topic"));
    app.send(json!({"op": "publish", "id": "p1", "topic": "/cmd_vel", "msg": {}})).await;
    assert_eq!(app.recv().await, status(Some("p1"), "publish: /cmd_vel is not allowed"));
    app.send(json!({"op": "publish", "id": "p2", "msg": {}})).await;
    assert_eq!(app.recv().await, status(Some("p2"), "publish: missing topic"));
    // no typed endpoint on the name yet
    app.send(json!({"op": "publish", "id": "p3", "topic": "/mower_recorder/command", "msg": {"data": "{}"}})).await;
    assert_eq!(app.recv().await, status(Some("p3"), "publish: cannot infer type of /mower_recorder/command: not advertised yet"));

    // With the bus bridge's reader (to_ros) the name is typed, and a publish
    // goes through whether the recorder runs or not (F7).
    let bridge = door.bus.scope("mower_bus_bridge");
    let mut command = bridge.subscribe::<Text>("/mower_recorder/command", Qos::default().keep_last(64)).unwrap();
    app.send(json!({"op": "publish", "id": "p4", "topic": "/mower_recorder/command", "msg": {"data": "{\"cmd\":\"list\"}"}})).await;
    assert_eq!(tokio::time::timeout(WAIT, command.next()).await.unwrap().unwrap().data, "{\"cmd\":\"list\"}");
    // a value of the wrong shape, with serde's text after `failed:`
    app.send(json!({"op": "publish", "id": "p5", "topic": "/mower_recorder/command", "msg": {"data": 5}})).await;
    assert_eq!(app.recv().await, status(Some("p5"), "publish: publish /mower_recorder/command failed: invalid type: integer `5`, expected a string"));
    // NUL is refused at the door (E3 decision 8): it reaches nobody
    app.send(json!({"op": "publish", "id": "p6", "topic": "/mower_recorder/command", "msg": {"data": "a\u{0}b"}})).await;
    assert_eq!(app.recv().await, status(Some("p6"), "publish: publish /mower_recorder/command failed: /data holds a NUL character"));
    app.quiet().await;
    assert!(command.try_recv().is_err(), "nothing published");
    door.close().await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn call_service_answers_as_the_r2r_backend_did_and_a_server_that_goes_fails_at_once() {
    let door = Door::open().await;
    let nav = door.bus.scope("nav_action_server");
    let _check = serve_with(&nav, "/check_nav_status", trigger("{\"state\":\"idle\"}"));
    let coverage = door.bus.scope("boustrophedon_coverage");
    let _exec = serve_with(&coverage, "/zone_exec_path", |r: &ZoneExecPath::Request| ZoneExecPath::Response {
        success: true,
        message: format!("zone {}", r.zone_id),
    });
    let mut app = door.client().await;

    app.call("c1", "/check_nav_status", json!({})).await;
    assert_eq!(app.recv().await, answer("c1", "/check_nav_status", json!({"message": "{\"state\":\"idle\"}", "success": true}), true));
    // no id, no args
    app.send(json!({"op": "call_service", "service": "/zone_exec_path"})).await;
    assert_eq!(
        app.recv().await,
        json!({"op": "service_response", "service": "/zone_exec_path", "values": {"message": "zone 0", "success": true}, "result": true}).to_string()
    );
    app.call("c2", "/rm_rf", json!({})).await;
    assert_eq!(app.recv().await, answer("c2", "/rm_rf", json!("service /rm_rf is not allowed"), false));
    app.send(json!({"op": "call_service", "id": "c3"})).await;
    assert_eq!(app.recv().await, status(Some("c3"), "call_service: missing service"));
    app.call("c4", "/zone_exec_path", json!({"zone_id": "three"})).await;
    assert_eq!(
        app.recv().await,
        answer("c4", "/zone_exec_path", json!("/zone_exec_path: request failed: invalid type: string \"three\", expected i32"), false)
    );
    app.call("c5", "/zone_exec_path", json!({"zone_id": 1, "x": "a\u{0}"})).await;
    assert_eq!(app.recv().await, answer("c5", "/zone_exec_path", json!("/zone_exec_path: request failed: /x holds a NUL character"), false));

    // A name declared on the bus with no server (the bus bridge's
    // service_from_ros while the recorder is down): 5 s, then unavailable.
    drop(door.bus.scope("mower_bus_bridge").client::<Trigger::Request, Trigger::Response>("/mower_recorder/stop").unwrap());
    app.call("c6", "/mower_recorder/stop", json!({})).await;
    door.deadline_in(Duration::from_secs(5)).await;
    door.manual.advance(Duration::from_millis(4_999));
    app.quiet().await;
    door.manual.advance(Duration::from_millis(1));
    assert_eq!(app.recv().await, answer("c6", "/mower_recorder/stop", json!("/mower_recorder/stop: service unavailable"), false));
    // a name nothing has declared at all: the same, after 5 s
    app.call("c7", "/system/check_update", json!({})).await;
    door.deadline_in(Duration::from_millis(100)).await;
    let asked = door.now();
    let frame = loop {
        door.manual.advance(Duration::from_millis(100));
        if let Ok(Some(Ok(WsMessage::Text(frame)))) = tokio::time::timeout(Duration::from_millis(20), app.stream.next()).await {
            break frame;
        }
        assert!(door.now().saturating_duration_since(asked) < Duration::from_secs(6), "no answer after 5 s");
    };
    assert_eq!(frame, answer("c7", "/system/check_update", json!("/system/check_update: service unavailable"), false));
    assert!(door.now().saturating_duration_since(asked) >= Duration::from_secs(5), "answered early");

    // A server that takes the call and never answers: 30 s, the wait included.
    let status_module = door.bus.scope("robot_status");
    let mut update = status_module.service::<Trigger::Request, Trigger::Response>("/system/update").unwrap();
    app.call("c8", "/system/update", json!({})).await;
    let held = tokio::time::timeout(WAIT, update.next()).await.unwrap().unwrap();
    door.deadline_in(Duration::from_secs(30)).await;
    door.manual.advance(Duration::from_secs(29));
    app.quiet().await;
    door.manual.advance(Duration::from_secs(1));
    assert_eq!(app.recv().await, answer("c8", "/system/update", json!("service /system/update did not answer within 30 s"), false));
    drop(held);

    // The server goes while it holds the call: unavailable at once, where
    // the r2r backend waited for its 30 s (E3 decision 15).
    app.call("c9", "/system/update", json!({})).await;
    let _held = tokio::time::timeout(WAIT, update.next()).await.unwrap().unwrap();
    status_module.close();
    assert_eq!(app.recv().await, answer("c9", "/system/update", json!("/system/update: service unavailable"), false));
    door.close().await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn rosapi_answers_from_the_bus_and_the_module_clock() {
    let door = Door::open().await;
    let world = door.bus.scope("world");
    let _online = world.publisher::<Bool>("/robot/online", Qos::latched()).unwrap();
    let _pose = world.publisher::<PoseStamped>("/adapter/robot_pose", Qos::default()).unwrap();
    // typed, but not allowed to clients
    let _odom = world.publisher::<Bool>("/odom_typed_but_not_allowed", Qos::default()).unwrap();
    // allowed and typed, but not ws_bridge's, and every endpoint gone
    drop(world.publisher::<Bool>("/mw_2/online", Qos::latched()).unwrap());
    // ws_bridge's, typed, every endpoint gone: listed, as a transient-local
    // name the bus bridge declared but nobody reads now
    drop(world.publisher::<Text>("/pid_autotune/status", Qos::latched()).unwrap());
    // a reader is an endpoint too
    let _clock = world.subscribe::<Header>("/manual_command_clock", Qos::default()).unwrap();
    // ws_bridge's, but untyped: no type to list
    drop(door.m.bus.subscribe_erased("/mower_recorder/bags", Qos::latched()).unwrap());
    let mut app = door.client().await;

    app.call("t", "/rosapi/topics", json!({})).await;
    let topics = json!({
        "topics": ["/adapter/robot_pose", "/manual_command_clock", "/pid_autotune/status", "/robot/online"],
        "types": ["geometry_msgs/msg/PoseStamped", "std_msgs/msg/Header", "std_msgs/msg/String", "std_msgs/msg/Bool"]
    });
    assert_eq!(app.recv().await, answer("t", "/rosapi/topics", topics, true));

    let (sec, nanosec) = door.start.to_parts();
    assert_eq!(nanosec, 0);
    app.call("g", "/rosapi/get_time", json!({})).await;
    assert_eq!(app.recv().await, answer("g", "/rosapi/get_time", json!({"time": {"nanosec": 0, "sec": sec}}), true));
    door.manual.advance(Duration::from_millis(1_250));
    app.call("g", "/rosapi/get_time", json!({})).await;
    assert_eq!(app.recv().await, answer("g", "/rosapi/get_time", json!({"time": {"nanosec": 250_000_000, "sec": sec + 1}}), true));
    door.close().await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn other_ops_and_frames_that_are_not_json() {
    let door = Door::open().await;
    let mut app = door.client().await;
    for op in ["set_level", "auth", "fragment"] {
        app.send(json!({"op": op, "id": "x"})).await;
    }
    app.quiet().await;
    app.send(json!({"op": "call_action", "id": "o1"})).await;
    assert_eq!(app.recv().await, status(Some("o1"), "unsupported op \"call_action\""));
    app.stream.send(WsMessage::Text("not json".into())).await.unwrap();
    assert_eq!(app.recv().await, status(None, "invalid JSON: expected ident at line 1 column 2"));
    app.stream.send(WsMessage::Binary(vec![0, 1, 2])).await.unwrap();
    assert_eq!(app.recv().await, status(None, "invalid JSON: EOF while parsing a value at line 1 column 0"));
    door.close().await;
}

// ---------------------------------------------------------------------------
// Calls in flight (E3.3): each call in a task of its own, every other frame
// in order
// ---------------------------------------------------------------------------

/// A server of `name` that answers each request `after` it came, on the
/// door's clock, holding several at once (a slow module: an update, a
/// recorder stop). How many it has taken so far is in the counter.
fn serve_slowly(scope: &Scope, clock: Clock, name: &str, after: Duration, message: &str) -> (tokio::task::JoinHandle<()>, Arc<AtomicUsize>) {
    let mut service = scope.service::<Trigger::Request, Trigger::Response>(name).unwrap();
    let taken = Arc::new(AtomicUsize::new(0));
    let (counter, message) = (taken.clone(), message.to_string());
    let task = tokio::spawn(async move {
        while let Some(request) = service.next().await {
            let at = clock.mono() + after;
            counter.fetch_add(1, Ordering::SeqCst);
            let (clock, message) = (clock.clone(), message.clone());
            tokio::spawn(async move {
                clock.sleep_until(at).await;
                let _ = request.respond(Trigger::Response { success: true, message });
            });
        }
    });
    (task, taken)
}

/// The app's joystick frame number `k`: its stamp's nanoseconds are `k`.
fn joystick(k: u32) -> Value {
    json!({"header": {"stamp": {"sec": 7, "nanosec": k}, "frame_id": "manual-1"},
           "twist": {"linear": {"x": 0.2, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0.001 * f64::from(k)}}})
}

fn too_many(id: &str, service: &str) -> String {
    let text = format!("{service}: too many calls in flight on this connection (at most {MAX_CALLS_IN_FLIGHT})");
    answer(id, service, json!(text), false)
}

/// While a call waits for a slow server (10 s), the same client's publish,
/// subscribe and other calls are handled at once, where each waited for it
/// before (F1); the slow answers then come with their `id`s.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_slow_call_holds_up_nothing_behind_it() {
    let door = Door::open().await;
    let status_module = door.bus.scope("robot_status");
    let (_update, taken) = serve_slowly(&status_module, door.m.clock.clone(), "/system/update", Duration::from_secs(10), "updated");
    let online = status_module.publisher::<Bool>("/robot/online", Qos::latched()).unwrap();
    online.publish(&Bool { data: true }).unwrap();
    let nav = door.bus.scope("nav_action_server");
    let _check = serve_with(&nav, "/check_nav_status", trigger("{\"state\":\"idle\"}"));
    let guard = door.bus.scope("manual_velocity_guard");
    let mut joy = guard.subscribe::<TwistStamped>("/app_joy_cmd", Qos::default().keep_last(64)).unwrap();
    let mut app = door.client().await;

    app.call("slow-1", "/system/update", json!({})).await;
    eventually("the server to hold the call", || taken.load(Ordering::SeqCst) == 1).await;
    // a publish
    app.send(json!({"op": "advertise", "topic": "/app_joy_cmd", "type": "geometry_msgs/msg/TwistStamped"})).await;
    app.send(json!({"op": "publish", "topic": "/app_joy_cmd", "msg": joystick(1)})).await;
    let got = tokio::time::timeout(WAIT, joy.next()).await.expect("the joystick at once").unwrap();
    assert_eq!(got.header.stamp.nanosec, 1);
    // a subscribe
    app.send(json!({"op": "subscribe", "topic": "/robot/online", "type": "std_msgs/msg/Bool"})).await;
    assert_eq!(app.recv().await, publish_frame("/robot/online", json!({"data": true})));
    // a second call, to another server
    app.call("fast", "/check_nav_status", json!({})).await;
    assert_eq!(app.recv().await, answer("fast", "/check_nav_status", json!({"message": "{\"state\":\"idle\"}", "success": true}), true));
    // and one more to the slow server, which holds both
    app.call("slow-2", "/system/update", json!({})).await;
    eventually("the server to hold the second call", || taken.load(Ordering::SeqCst) == 2).await;
    app.call("time", "/rosapi/get_time", json!({})).await;
    assert_eq!(app.recv_json().await["id"], "time");
    app.send(json!({"op": "publish", "topic": "/app_joy_cmd", "msg": joystick(2)})).await;
    let got = tokio::time::timeout(WAIT, joy.next()).await.expect("the joystick at once").unwrap();
    assert_eq!(got.header.stamp.nanosec, 2);

    // Not a moment of the server's 10 s has passed: both calls still wait.
    door.deadline_in(Duration::from_secs(10)).await;
    door.manual.advance(Duration::from_millis(9_999));
    app.quiet().await;
    door.manual.advance(Duration::from_millis(1));
    let mut answers = vec![app.recv().await, app.recv().await];
    answers.sort();
    let updated = json!({"message": "updated", "success": true});
    assert_eq!(answers, [answer("slow-1", "/system/update", updated.clone(), true), answer("slow-2", "/system/update", updated, true)]);
    app.quiet().await;
    door.close().await;
}

/// Answers come in the order the servers give them, each with its own
/// request's `id`, on its own connection: two clients may use the same ids.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn answers_carry_their_ids_in_the_order_the_servers_give_them() {
    let door = Door::open().await;
    let coverage = door.bus.scope("boustrophedon_coverage");
    let mut exec = coverage.service::<ZoneExecPath::Request, ZoneExecPath::Response>("/zone_exec_path").unwrap();
    let (mut app, mut studio) = (door.client().await, door.client().await);
    for zone in 1..=3 {
        app.call(&format!("z{zone}"), "/zone_exec_path", json!({"zone_id": zone})).await;
    }
    // the same id from another client, for another zone
    studio.call("z2", "/zone_exec_path", json!({"zone_id": 9})).await;
    // and one without an id
    app.send(json!({"op": "call_service", "service": "/zone_exec_path", "args": {"zone_id": 4}})).await;
    let mut held = Vec::new();
    for _ in 0..5 {
        held.push(tokio::time::timeout(WAIT, exec.next()).await.expect("every call at the server at once").unwrap());
    }
    app.quiet().await;
    // The server answers the last zone first.
    held.sort_by_key(|request| std::cmp::Reverse(request.message.zone_id));
    for request in held {
        let zone = request.message.zone_id;
        let _ = request.respond(ZoneExecPath::Response { success: true, message: format!("zone {zone}") });
        let values = json!({"message": format!("zone {zone}"), "success": true});
        match zone {
            9 => assert_eq!(studio.recv().await, answer("z2", "/zone_exec_path", values, true)),
            4 => assert_eq!(
                app.recv().await,
                json!({"op": "service_response", "service": "/zone_exec_path", "values": values, "result": true}).to_string()
            ),
            _ => assert_eq!(app.recv().await, answer(&format!("z{zone}"), "/zone_exec_path", values, true)),
        }
    }
    app.quiet().await;
    studio.quiet().await;
    door.close().await;
}

/// Subscribe, publish, advertise and every answer ws_bridge gives in line
/// keep the order of their requests while calls wait, and the joystick's
/// messages reach the guard all, in order. Only a call's answer comes when
/// its server gives it.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn every_other_frame_keeps_its_order_while_calls_wait() {
    let door = Door::open().await;
    let status_module = door.bus.scope("robot_status");
    let (_update, taken) = serve_slowly(&status_module, door.m.clock.clone(), "/system/update", Duration::from_secs(10), "updated");
    let online = status_module.publisher::<Bool>("/robot/online", Qos::latched()).unwrap();
    online.publish(&Bool { data: true }).unwrap();
    let info = status_module.publisher::<Text>("/robot/info", Qos::latched()).unwrap();
    info.publish(&text("{\"v\":1}")).unwrap();
    let nav = door.bus.scope("nav_action_server");
    let _check = serve_with(&nav, "/check_nav_status", trigger("idle"));
    let guard = door.bus.scope("manual_velocity_guard");
    let mut joy = guard.subscribe::<TwistStamped>("/app_joy_cmd", Qos::default().keep_last(64)).unwrap();
    // Another client has both latched topics already: a subscribe replays
    // from the cache, in line.
    let mut warm = door.client().await;
    warm.subscribe("/robot/online", 0).await;
    warm.recv().await;
    warm.subscribe("/robot/info", 0).await;
    warm.recv().await;

    let mut app = door.client().await;
    app.call("slow", "/system/update", json!({})).await;
    eventually("the server to hold the call", || taken.load(Ordering::SeqCst) == 1).await;
    let publish = |k: u32| json!({"op": "publish", "topic": "/app_joy_cmd", "msg": joystick(k)});
    let mut script = vec![
        json!({"op": "advertise", "id": "a1", "topic": "/cmd_vel", "type": "geometry_msgs/msg/Twist"}),
        json!({"op": "advertise", "topic": "/app_joy_cmd", "type": "geometry_msgs/msg/TwistStamped"}),
        publish(1),
        json!({"op": "subscribe", "id": "s1", "topic": "/robot/online", "type": "std_msgs/msg/Bool"}),
        publish(2),
        json!({"op": "publish", "id": "p1", "topic": "/cmd_vel", "msg": {}}),
        json!({"op": "call_service", "id": "c1", "service": "/check_nav_status", "args": {}}),
        publish(3),
        json!({"op": "subscribe", "id": "s2", "topic": "/robot/info"}),
        json!({"op": "call_service", "id": "c2", "service": "/rm_rf", "args": {}}),
        json!({"op": "subscribe", "id": "s3", "topic": "/odom"}),
        json!({"op": "call_service", "id": "c3", "service": "/system/update", "args": {}}),
    ];
    script.extend((4..=20).map(publish));
    script.push(json!({"op": "unsubscribe", "topic": "/robot/online"}));
    script.push(json!({"op": "advertise", "id": "a2"}));
    for frame in script {
        app.send(frame).await;
    }
    let in_line = [
        status(Some("a1"), "advertise: /cmd_vel is not allowed"),
        publish_frame("/robot/online", json!({"data": true})),
        status(Some("p1"), "publish: /cmd_vel is not allowed"),
        publish_frame("/robot/info", json!({"data": "{\"v\":1}"})),
        answer("c2", "/rm_rf", json!("service /rm_rf is not allowed"), false),
        status(Some("s3"), "subscribe: /odom is not allowed"),
        status(Some("a2"), "advertise: missing topic"),
    ];
    let mut got = Vec::new();
    for _ in 0..=in_line.len() {
        got.push(app.recv().await);
    }
    // c1's answer comes once its server gives it: after what came before c1
    let c1 = answer("c1", "/check_nav_status", json!({"message": "idle", "success": true}), true);
    let at = got.iter().position(|frame| *frame == c1).expect("c1's answer");
    assert!(at >= 3, "c1 answered before the frames sent before it: {got:#?}");
    got.remove(at);
    assert_eq!(got, in_line);
    for k in 1..=20 {
        let msg = tokio::time::timeout(WAIT, joy.next()).await.expect("every joystick message").unwrap();
        assert_eq!(msg.header.stamp.nanosec, k, "in order");
    }
    assert!(joy.try_recv().is_err(), "and no more");
    // the unsubscribe took effect in its place
    online.publish(&Bool { data: false }).unwrap();
    assert_eq!(warm.recv().await, publish_frame("/robot/online", json!({"data": false})));
    app.quiet().await;

    // The slow calls answer only now, the second as well.
    eventually("the server to hold c3", || taken.load(Ordering::SeqCst) == 2).await;
    door.deadline_in(Duration::from_secs(10)).await;
    door.manual.advance(Duration::from_secs(10));
    let mut answers = vec![app.recv().await, app.recv().await];
    answers.sort();
    let updated = json!({"message": "updated", "success": true});
    assert_eq!(answers, [answer("c3", "/system/update", updated.clone(), true), answer("slow", "/system/update", updated, true)]);
    door.close().await;
}

/// Past `MAX_CALLS_IN_FLIGHT` calls waiting on one connection, a call is
/// answered at once with an error and its `id`, and never reaches a server.
/// What ws_bridge answers itself and every other op go on; another
/// connection has places of its own; an answer frees a place.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn past_the_cap_a_call_is_refused_at_once() {
    let door = Door::open().await;
    let status_module = door.bus.scope("robot_status");
    let mut update = status_module.service::<Trigger::Request, Trigger::Response>("/system/update").unwrap();
    let nav = door.bus.scope("nav_action_server");
    let mut check = nav.service::<Trigger::Request, Trigger::Response>("/check_nav_status").unwrap();
    let mut app = door.client().await;
    for i in 0..MAX_CALLS_IN_FLIGHT {
        app.call(&format!("h{i}"), "/system/update", json!({})).await;
    }
    let mut held = Vec::new();
    for _ in 0..MAX_CALLS_IN_FLIGHT {
        held.push(tokio::time::timeout(WAIT, update.next()).await.expect("every call at the server").unwrap());
    }

    app.call("over", "/system/update", json!({})).await;
    assert_eq!(app.recv().await, too_many("over", "/system/update"));
    // the places are the connection's, whatever the service
    app.call("over-2", "/check_nav_status", json!({})).await;
    assert_eq!(app.recv().await, too_many("over-2", "/check_nav_status"));
    app.send(json!({"op": "call_service", "service": "/check_nav_status"})).await;
    assert_eq!(
        app.recv().await,
        json!({"op": "service_response", "service": "/check_nav_status", "result": false,
               "values": format!("/check_nav_status: too many calls in flight on this connection (at most {MAX_CALLS_IN_FLIGHT})")})
        .to_string()
    );
    // what is answered in line takes no place
    app.call("time", "/rosapi/get_time", json!({})).await;
    assert_eq!(app.recv_json().await["id"], "time");
    app.call("topics", "/rosapi/topics", json!({})).await;
    assert_eq!(app.recv_json().await["result"], true);
    app.call("no", "/rm_rf", json!({})).await;
    assert_eq!(app.recv().await, answer("no", "/rm_rf", json!("service /rm_rf is not allowed"), false));
    app.send(json!({"op": "publish", "id": "p", "topic": "/cmd_vel", "msg": {}})).await;
    assert_eq!(app.recv().await, status(Some("p"), "publish: /cmd_vel is not allowed"));
    assert!(tokio::time::timeout(QUIET, update.next()).await.is_err(), "nothing past the cap reached a server");
    assert!(tokio::time::timeout(QUIET, check.next()).await.is_err(), "nothing past the cap reached a server");

    // another connection has places of its own
    let mut studio = door.client().await;
    studio.call("s", "/check_nav_status", json!({})).await;
    let studio_call = tokio::time::timeout(WAIT, check.next()).await.expect("the other client's call").unwrap();
    let _ = studio_call.respond(Trigger::Response { success: true, message: "idle".into() });
    assert_eq!(studio.recv().await, answer("s", "/check_nav_status", json!({"message": "idle", "success": true}), true));

    // an answer frees a place at once
    let first = held.pop().unwrap();
    let _ = first.respond(Trigger::Response { success: true, message: "done".into() });
    let done = app.recv_json().await;
    assert_eq!((done["result"].clone(), done["values"]["message"].clone()), (json!(true), json!("done")));
    app.call("again", "/check_nav_status", json!({})).await;
    let again = tokio::time::timeout(WAIT, check.next()).await.expect("a call in the freed place").unwrap();
    let _ = again.respond(Trigger::Response { success: true, message: "idle".into() });
    assert_eq!(app.recv().await, answer("again", "/check_nav_status", json!({"message": "idle", "success": true}), true));
    // the place is taken again by the next held call
    app.call("over-3", "/check_nav_status", json!({})).await;
    let next = tokio::time::timeout(WAIT, check.next()).await.expect("the last place").unwrap();
    app.call("over-4", "/check_nav_status", json!({})).await;
    assert_eq!(app.recv().await, too_many("over-4", "/check_nav_status"));
    drop(next);
    assert_eq!(app.recv().await, answer("over-3", "/check_nav_status", json!("/check_nav_status: service unavailable"), false));

    let mut ids = Vec::new();
    for request in held {
        let _ = request.respond(Trigger::Response { success: true, message: "done".into() });
        ids.push(app.recv_json().await["id"].as_str().unwrap().to_string());
    }
    ids.push(done["id"].as_str().unwrap().to_string());
    ids.sort_by_key(|id| id[1..].parse::<usize>().unwrap());
    assert_eq!(ids, (0..MAX_CALLS_IN_FLIGHT).map(|i| format!("h{i}")).collect::<Vec<_>>(), "each held call answered once");
    door.close().await;
}

/// A call runs on after its client has gone, as it did when the connection
/// waited for it: a call sent just before the link went still reaches its
/// server, here one that only comes up afterwards.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_call_still_reaches_its_server_after_the_client_has_gone() {
    let door = Door::open().await;
    // the name is declared, with no server yet (the bus bridge's, while the
    // recorder is down)
    let _declared = door.bus.scope("mower_bus_bridge").client::<Trigger::Request, Trigger::Response>("/mower_recorder/stop").unwrap();
    let mut app = door.client().await;
    // a subscription, by which the test sees the connection end
    app.subscribe("/mower_recorder/status", 0).await;
    eventually("the subscription", || door.hub_clients("/mower_recorder/status") == 1).await;
    app.call("stop", "/mower_recorder/stop", json!({})).await;
    door.deadline_in(Duration::from_secs(5)).await;
    drop(app);
    eventually("the connection to end", || door.hub_clients("/mower_recorder/status") == 0).await;
    door.manual.advance(Duration::from_secs(2));
    let recorder = door.bus.scope("recorder_manager");
    let mut stop = recorder.service::<Trigger::Request, Trigger::Response>("/mower_recorder/stop").unwrap();
    let request = tokio::time::timeout(WAIT, stop.next()).await.expect("the stop reached the recorder").unwrap();
    // its answer goes nowhere, harmlessly
    assert!(request.respond(Trigger::Response { success: true, message: "stopped".into() }).is_ok());
    door.close().await;
}

// ---------------------------------------------------------------------------
// Delivery: throttle, replay, linger, slow consumers
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn the_throttle_thins_a_burst_and_ends_it_on_its_newest_message() {
    let door = Door::open().await;
    let coverage = door.bus.scope("boustrophedon_coverage");
    let layer = coverage.publisher::<Text>("/zone_sequence_status", Qos::default()).unwrap();
    let mut app = door.client().await;
    app.subscribe("/zone_sequence_status", 100).await;
    eventually("the reader", || door.readers("/zone_sequence_status") == 1).await;

    layer.publish(&text("clear")).unwrap();
    layer.publish(&text("path")).unwrap();
    assert_eq!(app.recv().await, publish_frame("/zone_sequence_status", json!({"data": "clear"})));
    door.deadline_in(Duration::from_millis(100)).await;
    app.quiet().await;
    door.manual.advance(Duration::from_millis(100));
    assert_eq!(app.recv().await, publish_frame("/zone_sequence_status", json!({"data": "path"})), "the newest, as the window closes");

    // a burst inside one window: the first at once if the window is over,
    // the newest when it closes
    door.manual.advance(Duration::from_millis(100));
    for p in ["a", "b", "c", "d"] {
        layer.publish(&text(p)).unwrap();
    }
    assert_eq!(app.recv().await, publish_frame("/zone_sequence_status", json!({"data": "a"})));
    door.deadline_in(Duration::from_millis(100)).await;
    door.manual.advance(Duration::from_millis(100));
    assert_eq!(app.recv().await, publish_frame("/zone_sequence_status", json!({"data": "d"})));
    app.quiet().await;

    // an unthrottled client gets everything
    let mut studio = door.client().await;
    studio.subscribe("/zone_sequence_status", 0).await;
    eventually("the second client", || door.hub_clients("/zone_sequence_status") == 2).await;
    layer.publish(&text("e")).unwrap();
    layer.publish(&text("f")).unwrap();
    assert_eq!(studio.recv().await, publish_frame("/zone_sequence_status", json!({"data": "e"})));
    assert_eq!(studio.recv().await, publish_frame("/zone_sequence_status", json!({"data": "f"})));
    door.close().await;
}

impl Door {
    /// The clients of ws_bridge's entry for `topic`.
    fn hub_clients(&self, topic: &str) -> usize {
        self.hub.entry(topic).map_or(0, |e| e.client_count())
    }
}

/// The first client gets a latched value from the writer's history, a later
/// one from the cache; once the writer has died, nobody gets it any more
/// (E3 decision 4), and a new writer's value comes alone.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_latched_value_is_replayed_while_its_writer_lives_and_never_after() {
    let door = Door::open().await;
    let record = door.bus.scope("path_record_node");
    let sites = record.publisher::<Text>("/site_list", Qos::latched()).unwrap();
    sites.publish(&text("v1")).unwrap();
    let v = |data: &str| publish_frame("/site_list", json!({"data": data}));

    let mut first = door.client().await;
    first.subscribe("/site_list", 200).await;
    assert_eq!(first.recv().await, v("v1"), "from the writer's history");
    let mut later = door.client().await;
    later.subscribe("/site_list", 0).await;
    assert_eq!(later.recv().await, v("v1"), "from the cache, at once");

    // the module dies: its scope closes, and its history with it
    record.close();
    eventually("the cache to go with the writer", || door.hub.entry("/site_list").is_some_and(|e| !e.has_cache())).await;
    let mut after = door.client().await;
    after.subscribe("/site_list", 0).await;
    after.quiet().await;

    // the module restarts and publishes: everybody gets the new value alone
    // (the first client's window over)
    door.manual.advance(Duration::from_millis(200));
    let restarted = door.bus.scope("path_record_node");
    let sites = restarted.publisher::<Text>("/site_list", Qos::latched()).unwrap();
    sites.publish(&text("v2")).unwrap();
    for client in [&mut first, &mut later, &mut after] {
        assert_eq!(client.recv().await, v("v2"));
    }
    let mut last = door.client().await;
    last.subscribe("/site_list", 0).await;
    assert_eq!(last.recv().await, v("v2"));
    last.quiet().await;
    door.close().await;
}

/// A subscribe before any writer exists does not leave the topic volatile
/// for good (F5): latched is what the writers are at each subscribe.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_topic_subscribed_before_its_writer_is_latched_once_the_writer_is() {
    let door = Door::open().await;
    let mut early = door.client().await;
    early.send(json!({"op": "subscribe", "topic": "/pid_autotune/status", "type": "std_msgs/msg/String"})).await;
    eventually("the reader", || door.readers("/pid_autotune/status") == 1).await;
    assert!(!door.hub.entry("/pid_autotune/status").unwrap().is_latched());

    let bridge = door.bus.scope("mower_bus_bridge");
    let status = bridge.publisher::<Text>("/pid_autotune/status", Qos::latched().keep_last(10)).unwrap();
    status.publish(&text("{\"state\":\"idle\"}")).unwrap();
    let frame = publish_frame("/pid_autotune/status", json!({"data": "{\"state\":\"idle\"}"}));
    assert_eq!(early.recv().await, frame);
    let mut studio = door.client().await;
    studio.subscribe("/pid_autotune/status", 0).await;
    assert_eq!(studio.recv().await, frame, "replayed: the writer keeps history");
    door.close().await;
}

/// A topic's reader stays 10 s after its last client (E3 decision 5): a
/// client that comes back in time reuses it, and then it goes.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_topic_s_reader_goes_ten_seconds_after_its_last_client() {
    let door = Door::open().await;
    let status_module = door.bus.scope("robot_status");
    let info = status_module.publisher::<Text>("/robot/info", Qos::latched()).unwrap();
    info.publish(&text("{}")).unwrap();
    let mut app = door.client().await;
    app.subscribe("/robot/info", 0).await;
    app.recv().await;
    assert_eq!(door.readers("/robot/info"), 1);

    app.send(json!({"op": "unsubscribe", "topic": "/robot/info"})).await;
    door.deadline_in(LINGER).await;
    door.manual.advance(LINGER - Duration::from_millis(1));
    tokio::time::sleep(QUIET).await;
    assert_eq!(door.readers("/robot/info"), 1, "still there just before");
    // back in time: the same reader, and the linger is off
    app.subscribe("/robot/info", 0).await;
    assert_eq!(app.recv().await, publish_frame("/robot/info", json!({"data": "{}"})), "the cache, kept meanwhile");
    door.manual.advance(Duration::from_secs(60));
    tokio::time::sleep(QUIET).await;
    assert_eq!(door.readers("/robot/info"), 1);

    // the client goes, as a closed connection
    drop(app);
    door.deadline_in(LINGER).await;
    door.manual.advance(LINGER);
    eventually("the reader to go", || door.readers("/robot/info") == 0).await;
    assert!(door.hub.entry("/robot/info").is_none());
    // a new client makes it afresh, and gets the value from the history
    let mut back = door.client().await;
    back.subscribe("/robot/info", 0).await;
    assert_eq!(back.recv().await, publish_frame("/robot/info", json!({"data": "{}"})));
    assert_eq!(door.readers("/robot/info"), 1);
    door.close().await;
}

/// A client that does not read loses frames once its queue of 512 is full,
/// and holds up neither the topic nor the other clients.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_slow_consumer_loses_frames_and_holds_up_nobody() {
    let door = Door::open().await;
    let status_module = door.bus.scope("robot_status");
    let telemetry = status_module.publisher::<Text>("/robot/telemetry", Qos::default()).unwrap();
    let (slow_tx, mut slow) = mpsc::channel(512);
    let (fast_tx, mut fast) = mpsc::channel(1024);
    door.hub.subscribe("/robot/telemetry", None, &ClientHandle::for_test(901, slow_tx), 0, false).unwrap();
    door.hub.subscribe("/robot/telemetry", None, &ClientHandle::for_test(902, fast_tx), 0, false).unwrap();
    // One at a time: the topic's reader keeps 10, as the r2r backend's did.
    for i in 0..600 {
        telemetry.publish(&text(&i.to_string())).unwrap();
        let frame = tokio::time::timeout(WAIT, fast.recv()).await.expect("the fast client").unwrap();
        assert_eq!(frame.as_ref(), publish_frame("/robot/telemetry", json!({"data": i.to_string()})));
    }
    let mut queued = 0;
    while slow.try_recv().is_ok() {
        queued += 1;
    }
    assert_eq!(queued, 512, "the slow client's queue, and nothing more");
    door.close().await;
}

/// Two clients of one topic: one frame, made once, the same bytes to both.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn two_clients_get_one_frame_made_once() {
    let door = Door::open().await;
    let battery = door.bus.scope("battery_state");
    let state = battery.publisher::<BatteryState>("/battery_state", Qos::default()).unwrap();
    let (mut app, mut studio) = (door.client().await, door.client().await);
    app.send(json!({"op": "subscribe", "topic": "/battery_state", "type": "sensor_msgs/msg/BatteryState", "throttle_rate": 1000})).await;
    studio.subscribe("/battery_state", 0).await;
    eventually("both clients", || door.hub_clients("/battery_state") == 2).await;
    let before = door.hub.framed();
    state.publish(&BatteryState { voltage: 25.1, ..Default::default() }).unwrap();
    let (a, b) = (app.recv().await, studio.recv().await);
    assert_eq!(a, b);
    assert!(a.contains("\"voltage\":25.100000381469727"), "an f32 widened, as r2r's path: {a}");
    assert_eq!(door.hub.framed() - before, 1);
    door.close().await;
}

// ---------------------------------------------------------------------------
// The golden corpus through the whole path, and the manifest
// ---------------------------------------------------------------------------

/// What the corpus asks for, collected so that the test can run it over a
/// WebSocket: each case publishes, serves or reads on the bus when run.
#[derive(Default)]
struct WsCorpus {
    writers: HashMap<String, Box<dyn Any + Send>>,
    publish: Vec<(String, String, Box<dyn FnOnce() + Send>)>,
    responses: HashMap<String, Box<dyn Any + Send>>,
    requests: Option<(mpsc::UnboundedSender<String>, mpsc::UnboundedReceiver<String>)>,
    service: Vec<(String, String, String, Value)>,
    readers: HashMap<String, Box<dyn Any + Send>>,
    inbound: Vec<(String, String, Value, Box<dyn FnMut() -> Option<String> + Send>)>,
    scope: Option<Scope>,
}

impl fixtures::Corpus for WsCorpus {
    fn publish<T: fixtures::Msg>(&mut self, case: &str, topic: &str, value: T) {
        let scope = self.scope.clone().unwrap();
        let writer = self
            .writers
            .entry(topic.to_string())
            .or_insert_with(|| Box::new(scope.publisher::<T>(topic, Qos::latched()).unwrap()))
            .downcast_ref::<Publisher<T>>()
            .expect("one type per topic")
            .clone();
        self.publish.push((case.to_string(), topic.to_string(), Box::new(move || writer.publish(&value).unwrap())));
    }

    fn service<S: fixtures::Srv>(&mut self, case: &str, service: &str, id: &str, args: Value, response: S::Response) {
        let scope = self.scope.clone().unwrap();
        let requests = self.requests.get_or_insert_with(mpsc::unbounded_channel).0.clone();
        let queue = self.responses.entry(service.to_string()).or_insert_with(|| {
            let queue: Arc<Mutex<VecDeque<S::Response>>> = Arc::default();
            let (answers, mut server) = (queue.clone(), scope.service::<S::Request, S::Response>(service).unwrap());
            tokio::spawn(async move {
                while let Some(request) = server.next().await {
                    let _ = requests.send(serde_json::to_value(&request.message).unwrap().to_string());
                    let response = answers.lock().unwrap().pop_front().expect("a response for each call");
                    let _ = request.respond(response);
                }
            });
            Box::new(queue)
        });
        queue.downcast_ref::<Arc<Mutex<VecDeque<S::Response>>>>().expect("one type per service").lock().unwrap().push_back(response);
        self.service.push((case.to_string(), service.to_string(), id.to_string(), args));
    }

    fn inbound<T: fixtures::Msg>(&mut self, case: &str, topic: &str, frame: Value) {
        let scope = self.scope.clone().unwrap();
        let reader = self
            .readers
            .entry(topic.to_string())
            .or_insert_with(|| Box::new(Arc::new(Mutex::new(scope.subscribe::<T>(topic, Qos::default()).unwrap()))))
            .downcast_ref::<Arc<Mutex<Subscriber<T>>>>()
            .expect("one type per topic")
            .clone();
        let decoded = move || reader.lock().unwrap().try_recv().ok().map(|msg| serde_json::to_value(&*msg).unwrap().to_string());
        self.inbound.push((case.to_string(), topic.to_string(), frame, Box::new(decoded)));
    }
}

/// Every golden case from the modules to the client's socket and back: the
/// bytes the app gets are the golden ones, which are r2r's (the corpus).
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn the_golden_corpus_comes_out_of_the_socket_byte_for_byte() {
    let door = Door::open().await;
    let mut corpus = WsCorpus { scope: Some(door.bus.scope("modules")), ..Default::default() };
    fixtures::corpus(&mut corpus);
    let golden = |file: &str| fixtures::parse_golden(file);
    let (published, answered, decoded_requests, decoded_inbound) = (
        golden(include_str!("../tests/golden/publish.txt")),
        golden(include_str!("../tests/golden/service_response.txt")),
        golden(include_str!("../tests/golden/request.txt")),
        golden(include_str!("../tests/golden/inbound.txt")),
    );
    let mut app = door.client().await;

    let topics: std::collections::BTreeSet<String> = corpus.publish.iter().map(|(_, topic, _)| topic.clone()).collect();
    for topic in &topics {
        app.subscribe(topic, 0).await;
    }
    eventually("a reader per topic", || topics.iter().all(|t| door.readers(t) == 1)).await;
    for (case, _topic, publish) in corpus.publish {
        publish();
        let frame = app.recv().await;
        let expected = fixtures::own_file(&case).map(str::to_string).unwrap_or_else(|| published[&case].clone());
        assert!(frame == expected, "{case}: the frame differs from the golden one\n  got:  {}", &frame[..frame.len().min(300)]);
    }

    let (_, mut requests) = corpus.requests.take().unwrap();
    for (case, service, id, args) in corpus.service {
        app.call(&id, &service, args).await;
        assert_eq!(app.recv().await, answered[&case], "{case}");
        let request = tokio::time::timeout(WAIT, requests.recv()).await.unwrap().unwrap();
        assert_eq!(request, decoded_requests[&case], "{case}: the request as the server got it");
    }

    for (case, topic, frame, mut decoded) in corpus.inbound {
        app.send(json!({"op": "publish", "topic": topic, "msg": frame})).await;
        let mut got = None;
        eventually(&format!("{case} on the bus"), || {
            got = decoded();
            got.is_some()
        })
        .await;
        assert_eq!(got.unwrap(), decoded_inbound[&case], "{case}");
    }
    app.quiet().await;
    door.close().await;
}

/// Every name the bus backend reaches, with the types the modules give
/// them: the writers of what ws_bridge reads, the readers of what it writes,
/// a server for each service.
fn world(scope: &Scope) -> Vec<Box<dyn Any + Send>> {
    let mut keep: Vec<Box<dyn Any + Send>> = Vec::new();
    fn writer<T: Message>(keep: &mut Vec<Box<dyn Any + Send>>, scope: &Scope, name: &str, qos: Qos) {
        keep.push(Box::new(scope.publisher::<T>(name, qos).unwrap()));
    }
    for name in [
        "/adapter/map_layers/map_grid",
        "/adapter/map_layers/free_space_inflated",
        "/adapter/map_layers/risk_map_inflated",
        "/adapter/map_layers/chennal_map_inflated",
        "/adapter/marker_layers/zones",
        "/adapter/marker_layers/risk_zones",
        "/adapter/marker_layers/channels",
        "/adapter/marker_layers/coverage_path",
        "/adapter/marker_layers/invalid_segments",
        "/adapter/marker_layers/connectors",
        "/adapter/coverage_settings",
        "/adapter/zone_summaries",
        "/adapter/map_datum",
        "/site_list",
        "/robot/info",
        "/pid_autotune/status",
        "/mower_recorder/status",
        "/mower_recorder/bags",
        "/zone_sequence_status",
    ] {
        writer::<Text>(&mut keep, scope, name, Qos::latched());
    }
    for name in ["/robot/telemetry", "/gps/status", "/mower_recorder/command_result"] {
        writer::<Text>(&mut keep, scope, name, Qos::default());
    }
    writer::<PoseStamped>(&mut keep, scope, "/adapter/robot_pose", Qos::default());
    writer::<Bool>(&mut keep, scope, "/robot/online", Qos::latched());
    writer::<BatteryState>(&mut keep, scope, "/battery_state", Qos::default());
    writer::<NavSatFix>(&mut keep, scope, "/fix", Qos::default());
    writer::<Header>(&mut keep, scope, "/manual_command_clock", Qos::default());
    writer::<CoverageProgress>(&mut keep, scope, "/coverage_progress", Qos::latched());
    keep.push(Box::new(scope.subscribe::<TwistStamped>("/app_joy_cmd", Qos::default()).unwrap()));
    for name in ["/mower_recorder/command", "/mower_base/servo_command", "/mower_base/blade_command"] {
        keep.push(Box::new(scope.subscribe::<Text>(name, Qos::default()).unwrap()));
    }
    fn server<Req: Message, Resp: Message + Default>(keep: &mut Vec<Box<dyn Any + Send>>, scope: &Scope, name: &str) {
        keep.push(Box::new(serve_with(scope, name, |_: &Req| Resp::default())));
    }
    for name in [
        "/record_zone_start",
        "/record_zone_end",
        "/risk_zone_start",
        "/risk_zone_end",
        "/channel_record_start",
        "/channel_record_end",
        "/record_cancel",
        "/save_zone_list",
        "/load_zone_list",
        "/create_free_space",
        "/create_risk_map",
        "/create_chennal_map",
        "/generate_coverage_path",
        "/restore_free_space_coverage",
        "/stop_zone_sequence",
        "/cancel_nav2",
        "/cencel_nav2",
        "/check_nav_status",
        "/system/update",
        "/system/restart",
        "/system/check_update",
        "/mower_recorder/start",
        "/mower_recorder/stop",
    ] {
        server::<Trigger::Request, Trigger::Response>(&mut keep, scope, name);
    }
    server::<ImportImageMask::Request, ImportImageMask::Response>(&mut keep, scope, "/import_image_mask");
    server::<EditZone::Request, EditZone::Response>(&mut keep, scope, "/edit_zone");
    server::<SiteOp::Request, SiteOp::Response>(&mut keep, scope, "/site_op");
    server::<ZoneExecPath::Request, ZoneExecPath::Response>(&mut keep, scope, "/zone_exec_path");
    server::<ZoneSequence::Request, ZoneSequence::Response>(&mut keep, scope, "/run_zone_sequence");
    server::<GetCoverageProgress::Request, GetCoverageProgress::Response>(&mut keep, scope, "/coverage_progress_status");
    server::<ResumeCoverage::Request, ResumeCoverage::Response>(&mut keep, scope, "/resume_coverage");
    server::<PidAutotune::Request, PidAutotune::Response>(&mut keep, scope, "/pid_autotune");
    server::<SetParameters::Request, SetParameters::Response>(&mut keep, scope, "/boustrophedon_coverage/set_parameters");
    keep
}

/// What the bus backend makes on its scope, for every name it reaches, is
/// what the manifest's `bridge` module says: the bus bridge carries it.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn every_endpoint_the_bus_backend_makes_is_in_the_manifest() {
    let door = Door::open().await;
    let _world = world(&door.bus.scope("modules"));
    let (module, bridge) = mower_bridge::instance("mower_ws_bridge").unwrap();
    assert_eq!(module.id, "bridge");
    let mut app = door.client().await;
    for e in bridge.endpoints {
        match e.kind {
            mower_bridge::types::Kind::Subscribe => app.subscribe(e.name, 0).await,
            mower_bridge::types::Kind::Publish => {
                let msg = if e.name == "/app_joy_cmd" { json!({}) } else { json!({"data": "{}"}) };
                app.send(json!({"op": "publish", "id": e.name, "topic": e.name, "msg": msg})).await;
            }
            mower_bridge::types::Kind::Call => {
                // Each call is answered (the servers answer defaults), nothing
                // else is. One at a time, as the app calls: more than
                // MAX_CALLS_IN_FLIGHT at once would be refused (E3.3).
                app.call(e.name, e.name, json!({})).await;
                loop {
                    let frame = app.recv_json().await;
                    if frame["op"] == "service_response" {
                        assert_eq!((&frame["id"], &frame["result"]), (&json!(e.name), &json!(true)), "{frame}");
                        break;
                    }
                    assert_eq!(frame["op"], json!("publish"), "{frame}");
                }
            }
            other => panic!("ws_bridge does not {}", other.verb()),
        }
    }
    let bus = &door.bus;
    eventually("ws_bridge's endpoints", || {
        bus.topics().iter().filter(|t| t.subscribers.iter().any(|s| s == "mower_ws_bridge")).count() == 28
            && bus.topics().iter().filter(|t| t.publishers.iter().any(|s| s == "mower_ws_bridge")).count() == 4
            && bus.services().iter().filter(|s| s.clients.iter().any(|c| c == "mower_ws_bridge")).count() == 32
    })
    .await;
    mower_bridge::coverage::assert_in_manifest(&door.m);
    door.close().await;
}

// ---------------------------------------------------------------------------
// The listeners
// ---------------------------------------------------------------------------

/// A fatal bind error of the public listener ends `serve` with every
/// listener it started aborted, so that a restart can bind their ports
/// again (F8).
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_fatal_bind_error_leaves_no_listener_behind() {
    for _attempt in 0..5 {
        let (port, loopback) = (free_port(), free_port());
        // 127.0.0.2 counts as a LAN address: the loopback gate and the
        // loopback listener are bound first, then the public bind fails, the
        // port taken.
        let _taken = std::net::TcpListener::bind(("127.0.0.2", port)).unwrap();
        let line = format!(
            "--address 127.0.0.2 --port {port} --loopback-port {loopback} --policy {} --state-dir /nonexistent/ws_bridge_test --backend bus",
            policy_file()
        );
        let args = parse_args(&argv(&line)).unwrap();
        let policy = Arc::new(Policy::from_yaml(&std::fs::read_to_string(policy_file()).unwrap()).unwrap());
        let bus = Bus::new();
        let m = ModuleCtx::new("mower_ws_bridge", Shutdown::new()).with_bus(bus.scope("mower_ws_bridge"));
        let hub = BusHub::new(m.bus.clone(), m.clock.clone(), manifest_names(&m).unwrap(), "mower_ws_bridge");
        let failed = serve(args, policy, None, Hub::Bus(hub), "mower_ws_bridge".into(), m).await.unwrap_err().to_string();
        if failed.starts_with("bind 127.0.0.1:") {
            continue; // another test had one of the loopback ports
        }
        assert!(failed.starts_with(&format!("bind 127.0.0.2:{port}: ")), "{failed}");
        eventually("the loopback listeners to be gone", || {
            std::net::TcpListener::bind(("127.0.0.1", port)).is_ok() && std::net::TcpListener::bind(("127.0.0.1", loopback)).is_ok()
        })
        .await;
        return;
    }
    panic!("no free ports in five attempts");
}

// ---------------------------------------------------------------------------
// The loopback listener (E8-2)
// ---------------------------------------------------------------------------

fn loopback_policy_file() -> String {
    concat!(env!("CARGO_MANIFEST_DIR"), "/../../../mower_bringup/config/ws_bridge_loopback.yaml").to_string()
}

/// The modules a robot has, for these tests: names of the public policy, a
/// name it does not allow (`/mower_base/telemetry`) and a service it does not
/// allow.
struct Modules {
    fix: Publisher<NavSatFix>,
    telemetry: Publisher<Text>,
    _ekf_reader: Subscriber<NavSatFix>,
    _nav: tokio::task::JoinHandle<()>,
    _private: tokio::task::JoinHandle<()>,
    _server: mower_bus::ActionServer<Wp>,
    _caller: mower_bus::Client<Trigger::Request, Trigger::Response>,
}

type Wp = (Waypoint::Goal, Waypoint::Result, Waypoint::Feedback);

impl Modules {
    fn on(bus: &Bus) -> Modules {
        let fix = bus.scope("gps").publisher::<NavSatFix>("/fix", Qos::system_default()).unwrap();
        let telemetry = bus.scope("mower_base").publisher::<Text>("/mower_base/telemetry", Qos::sensor_data()).unwrap();
        let _ekf_reader = bus.scope("navsat_transform").subscribe::<NavSatFix>("/fix", Qos::default().keep_last(5)).unwrap();
        let nav = bus.scope("nav_action_server");
        let _nav = serve_with(&nav, "/check_nav_status", trigger("{\"state\":\"idle\"}"));
        let _private = serve_with(&bus.scope("map_manage"), "/not_public", trigger("private"));
        let _server = nav.action_server::<Wp>("/nav_action").unwrap();
        let _caller = bus.scope("flutter_adapter").client::<Trigger::Request, Trigger::Response>("/not_public").unwrap();
        fix.publish(&NavSatFix::default()).unwrap();
        telemetry.publish(&text("{}")).unwrap();
        Modules { fix, telemetry, _ekf_reader, _nav, _private, _server, _caller }
    }
}

/// A state dir with a robot's identity, which turns the gate on.
struct StateDir(std::path::PathBuf);

impl StateDir {
    fn paired() -> StateDir {
        static N: AtomicUsize = AtomicUsize::new(0);
        let dir = std::env::temp_dir().join(format!("ws_bridge_e82_{}_{}", std::process::id(), N.fetch_add(1, Ordering::Relaxed)));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("identity.json"), r#"{"robot_id": "MW-E82TST", "secret": "mfrggzdf"}"#).unwrap();
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

/// ws_bridge as the robot runs it, on the bus backend: a public address
/// (127.0.0.2 counts as a LAN one), the gate on 127.0.0.1 for the relay agent,
/// and the loopback listener, with `--loopback-policy` if `policy` names a
/// file. The gate is open unless `state` has an identity.
struct Robot {
    bus: Bus,
    manual: ManualClock,
    m: ModuleCtx,
    registry: Registry,
    public: String,
    relay: String,
    loopback: String,
    state: Option<StateDir>,
    task: tokio::task::JoinHandle<ModuleResult>,
    modules: Modules,
}

impl Robot {
    async fn start(policy: Option<&str>, state: Option<StateDir>) -> Robot {
        for _attempt in 0..5 {
            let (port, loopback_port) = (free_port(), free_port());
            let bus = Bus::new();
            let (clock, manual) = Clock::manual(T0);
            let registry = Registry::default();
            let state_dir = state.as_ref().map_or("/nonexistent/ws_bridge_test", |s| s.path()).to_string();
            let mut line = format!(
                "--address 127.0.0.2 --port {port} --loopback-port {loopback_port} --policy {} --state-dir {state_dir} --backend bus",
                policy_file()
            );
            if let Some(file) = policy {
                line += &format!(" --loopback-policy {file}");
            }
            let args = parse_args(&argv(&line)).unwrap();
            let public = Arc::new(Policy::from_yaml(&std::fs::read_to_string(policy_file()).unwrap()).unwrap());
            let m = ModuleCtx::new("mower_ws_bridge", Shutdown::new())
                .with_bus(bus.scope("mower_ws_bridge"))
                .with_clock(clock.clone())
                .with_registry(registry.clone());
            let hub = BusHub::new(m.bus.clone(), clock, manifest_names(&m).unwrap(), "mower_ws_bridge");
            let view = load_loopback(&args, &public, &m, None, "mower_ws_bridge");
            let modules = Modules::on(&bus);
            let task = tokio::spawn(serve(args, public, view, Hub::Bus(hub), "mower_ws_bridge".into(), m.clone()));
            let deadline = Instant::now() + WAIT;
            let listening = loop {
                if task.is_finished() {
                    break false;
                }
                if TcpStream::connect(("127.0.0.2", port)).await.is_ok() && TcpStream::connect(("127.0.0.1", port)).await.is_ok() && TcpStream::connect(("127.0.0.1", loopback_port)).await.is_ok() {
                    break true;
                }
                assert!(Instant::now() < deadline, "ws_bridge did not listen");
                tokio::time::sleep(Duration::from_millis(5)).await;
            };
            if listening {
                return Robot {
                    bus,
                    manual,
                    m,
                    registry,
                    public: format!("ws://127.0.0.2:{port}"),
                    relay: format!("ws://127.0.0.1:{port}"),
                    loopback: format!("ws://127.0.0.1:{loopback_port}"),
                    state,
                    task,
                    modules,
                };
            }
            // another test had one of the ports
        }
        panic!("no free ports in five attempts");
    }

    async fn with_the_shipped_policy() -> Robot {
        Robot::start(Some(&loopback_policy_file()), None).await
    }

    /// A client of `url`, through the gate if the state dir has an identity.
    async fn client(&self, url: &str) -> Ws {
        let mut request = url.into_client_request().unwrap();
        if let Some(state) = self.state.as_ref().filter(|_| url != self.loopback) {
            let identity = crate::auth::Identity::load(state.path()).unwrap().unwrap();
            let t = crate::auth::now_unix();
            static NONCES: AtomicUsize = AtomicUsize::new(0);
            let nonce = format!("{:032x}", NONCES.fetch_add(1, Ordering::Relaxed));
            let mac = identity.compute_mac("test_phone", t, &nonce);
            for (k, v) in [("x-mower-robot", identity.robot_id.clone()), ("x-mower-client", "test_phone".to_string()), ("x-mower-time", t.to_string()), ("x-mower-nonce", nonce), ("x-mower-mac", mac)] {
                request.headers_mut().insert(k, v.parse().unwrap());
            }
        }
        let (stream, _) = tokio_tungstenite::connect_async(request).await.unwrap();
        Ws { stream }
    }

    fn scopes_reading(&self, topic: &str) -> Vec<String> {
        self.bus.topic(topic).map_or(Vec::new(), |t| t.subscribers)
    }

    async fn close(self) {
        self.m.shutdown.trigger();
        tokio::time::timeout(WAIT, self.task).await.expect("serve returned").unwrap().unwrap();
        self.m.bus.close();
    }
}

/// What the loopback policy opens, which the public listeners must keep shut.
const NOT_FOR_THE_GATE: [&str; 11] = [
    "/mowerd/topics",
    "/mowerd/services",
    "/mowerd/actions",
    "/mowerd/modules",
    "/mowerd/bridge",
    "/map_manage/list_parameters",
    "/map_manage/get_parameters",
    "/map_manage/describe_parameters",
    "/map_manage/get_parameter_types",
    "/boustrophedon_coverage/get_parameters",
    "/not_public",
];

/// **The security property of E8-2:** through the gate (the public address,
/// and 127.0.0.1:9090, where the relay agent's sessions arrive from off the
/// machine) nothing the loopback policy opens is reachable, gate open or on.
/// Every such call gets what a service that was never allowed gets, byte for
/// byte, and every such subscribe what a topic never allowed gets, and neither
/// leaves a reader or a client on the bus.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn nothing_the_loopback_policy_opens_is_reachable_through_the_gate() {
    for state in [None, Some(StateDir::paired())] {
        let paired = state.is_some();
        let robot = Robot::start(Some(&loopback_policy_file()), state).await;
        for url in [robot.public.clone(), robot.relay.clone()] {
            let mut phone = robot.client(&url).await;
            // the reference: a name no policy has ever had
            phone.call("id", "/rm_rf", json!({})).await;
            let refused = phone.recv().await;
            assert_eq!(refused, answer("id", "/rm_rf", json!("service /rm_rf is not allowed"), false));
            for service in NOT_FOR_THE_GATE {
                phone.call("id", service, json!({"names": ["/fix"]})).await;
                assert_eq!(phone.recv().await, refused.replace("/rm_rf", service), "{url} paired={paired}");
            }
            for topic in ["/mower_base/telemetry", "/imu/data", "/odom", "/rosout", "*"] {
                phone.send(json!({"op": "subscribe", "id": "s", "topic": topic})).await;
                assert_eq!(phone.recv().await, status(Some("s"), &format!("subscribe: {topic} is not allowed")), "{url}");
            }
            // a type hint and a throttle change nothing
            phone.send(json!({"op": "subscribe", "id": "s", "topic": "/mower_base/telemetry", "type": "std_msgs/msg/String", "throttle_rate": 100})).await;
            assert_eq!(phone.recv().await, status(Some("s"), "subscribe: /mower_base/telemetry is not allowed"));
            robot.modules.telemetry.publish(&text("{}")).unwrap();
            phone.quiet().await;
            // /rosapi/topics lists the public policy's names and no other
            phone.call("t", "/rosapi/topics", json!({})).await;
            let listed = phone.recv_json().await;
            let names: Vec<&str> = listed["values"]["topics"].as_array().unwrap().iter().map(|n| n.as_str().unwrap()).collect();
            assert!(names.contains(&"/fix"), "{names:?}");
            assert!(!names.contains(&"/mower_base/telemetry"), "{names:?}");
        }
        assert_eq!(robot.scopes_reading("/mower_base/telemetry"), Vec::<String>::new(), "no reader was made");
        for name in NOT_FOR_THE_GATE {
            assert!(!robot.bus.services().iter().any(|s| s.name == name && s.clients.iter().any(|c| c == "mower_ws_bridge")), "{name}");
        }
        // not because nothing answers them: the loopback listener does
        let mut local = robot.client(&robot.loopback).await;
        local.call("m", "/mowerd/topics", json!({"names": ["/mower_base/telemetry"]})).await;
        let answered = local.recv_json().await;
        assert_eq!(answered["result"], true, "{answered}");
        assert_eq!(answered["values"]["topics"][0]["name"], "/mower_base/telemetry");
        robot.close().await;
    }
}

/// Everything a phone sends, on a robot with the loopback policy and on one
/// without: the same bytes.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_phone_cannot_tell_whether_the_loopback_policy_is_on() {
    async fn session(robot: &Robot) -> Vec<String> {
        let mut phone = robot.client(&robot.public).await;
        let mut seen = Vec::new();
        for (service, args) in [
            ("/rosapi/topics", json!({})),
            ("/check_nav_status", json!({})),
            ("/mowerd/topics", json!({})),
            ("/mowerd/modules", json!({})),
            ("/map_manage/get_parameters", json!({"names": ["x"]})),
            ("/not_public", json!({})),
            ("/rm_rf", json!({})),
        ] {
            phone.call("id", service, args).await;
            seen.push(phone.recv().await);
        }
        phone.subscribe("/fix", 0).await;
        eventually("the app's reader", || robot.scopes_reading("/fix").iter().any(|s| s == "mower_ws_bridge")).await;
        robot.modules.fix.publish(&NavSatFix::default()).unwrap();
        seen.push(phone.recv().await);
        phone.subscribe("/mower_base/telemetry", 0).await;
        seen.push(phone.recv().await);
        phone.send(json!({"op": "publish", "topic": "/mower_base/telemetry", "msg": {"data": "x"}})).await;
        seen.push(phone.recv().await);
        seen
    }
    let without = Robot::start(None, None).await;
    let before = session(&without).await;
    without.close().await;
    let with = Robot::with_the_shipped_policy().await;
    let after = session(&with).await;
    assert_eq!(after, before);
    assert_eq!(after[2], answer("id", "/mowerd/topics", json!("service /mowerd/topics is not allowed"), false));
    with.close().await;
}

/// The loopback listener is 127.0.0.1's alone: a socket bound to the wildcard
/// would take these connections too.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn the_loopback_listener_takes_connections_on_127_0_0_1_alone() {
    let robot = Robot::with_the_shipped_policy().await;
    let port = robot.loopback.rsplit(':').next().unwrap().parse::<u16>().unwrap();
    assert!(TcpStream::connect(("127.0.0.1", port)).await.is_ok());
    assert!(TcpStream::connect(("127.0.0.2", port)).await.is_err(), "127.0.0.2:{port} is another address");
    // and any address of this machine that leads off it, where it has one
    let outward = std::net::UdpSocket::bind(("0.0.0.0", 0)).and_then(|s| s.connect(("10.255.255.255", 9)).and_then(|()| s.local_addr()));
    if let Ok(addr) = outward {
        if !addr.ip().is_loopback() && !addr.ip().is_unspecified() {
            assert!(TcpStream::connect((addr.ip(), port)).await.is_err(), "{}:{port}", addr.ip());
        }
    }
    robot.close().await;
}

/// Where the gate holds the loopback port (`--address 127.0.0.1`, the port
/// the same), there is no loopback listener, so no wider view.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_loopback_port_the_gate_holds_serves_no_wider_view() {
    for _attempt in 0..5 {
        let port = free_port();
        let line = format!(
            "--address 127.0.0.1 --port {port} --loopback-port {port} --policy {} --state-dir /nonexistent/ws_bridge_test --backend bus --loopback-policy {}",
            policy_file(),
            loopback_policy_file()
        );
        let args = parse_args(&argv(&line)).unwrap();
        let public = Arc::new(Policy::from_yaml(&std::fs::read_to_string(policy_file()).unwrap()).unwrap());
        let bus = Bus::new();
        let m = ModuleCtx::new("mower_ws_bridge", Shutdown::new()).with_bus(bus.scope("mower_ws_bridge"));
        let hub = BusHub::new(m.bus.clone(), m.clock.clone(), manifest_names(&m).unwrap(), "mower_ws_bridge");
        let view = load_loopback(&args, &public, &m, None, "mower_ws_bridge");
        assert!(view.is_some());
        let task = tokio::spawn(serve(args, public, view, Hub::Bus(hub), "mower_ws_bridge".into(), m.clone()));
        let deadline = Instant::now() + WAIT;
        while TcpStream::connect(("127.0.0.1", port)).await.is_err() && !task.is_finished() {
            assert!(Instant::now() < deadline);
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        if task.is_finished() {
            continue; // another test had the port
        }
        let (stream, _) = tokio_tungstenite::connect_async(format!("ws://127.0.0.1:{port}")).await.unwrap();
        let mut phone = Ws { stream };
        phone.call("id", "/mowerd/topics", json!({})).await;
        assert_eq!(phone.recv().await, answer("id", "/mowerd/topics", json!("service /mowerd/topics is not allowed"), false));
        m.shutdown.trigger();
        task.await.unwrap().unwrap();
        return;
    }
    panic!("no free port in five attempts");
}

/// Without `--loopback-policy`, or with a file it cannot use, the loopback
/// listener is what it was, and the door stands: a roll-back of E8-2 and a bad
/// file look the same.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn without_a_usable_loopback_policy_the_listener_serves_the_public_one() {
    let dir = StateDir::paired();
    let bad = dir.0.join("bad.yaml");
    std::fs::write(&bad, "topics_sub: ['*']\ntopics_pub: ['/cmd_vel']\n").unwrap();
    let bad = bad.to_str().unwrap().to_string();
    for policy in [None, Some("/nonexistent/ws_bridge_loopback.yaml"), Some(bad.as_str())] {
        let robot = Robot::start(policy, None).await;
        let mut local = robot.client(&robot.loopback).await;
        local.call("m", "/mowerd/topics", json!({})).await;
        assert_eq!(local.recv().await, answer("m", "/mowerd/topics", json!("service /mowerd/topics is not allowed"), false), "{policy:?}");
        local.send(json!({"op": "subscribe", "id": "s", "topic": "/mower_base/telemetry"})).await;
        assert_eq!(local.recv().await, status(Some("s"), "subscribe: /mower_base/telemetry is not allowed"));
        local.call("p", "/map_manage/get_parameters", json!({})).await;
        assert_eq!(local.recv().await, answer("p", "/map_manage/get_parameters", json!("service /map_manage/get_parameters is not allowed"), false));
        // and what it did serve, it serves
        local.call("c", "/check_nav_status", json!({})).await;
        assert_eq!(local.recv_json().await["result"], true);
        robot.close().await;
    }
}

/// The loopback policy only reads: what the listener may publish and which
/// other services it may call stay the public policy's.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn the_loopback_listener_writes_what_the_public_policy_writes_and_nothing_more() {
    let robot = Robot::with_the_shipped_policy().await;
    let mut local = robot.client(&robot.loopback).await;
    for topic in ["/mower_base/telemetry", "/cmd_vel", "/fix"] {
        local.send(json!({"op": "advertise", "id": "a", "topic": topic, "type": "std_msgs/msg/String"})).await;
        assert_eq!(local.recv().await, status(Some("a"), &format!("advertise: {topic} is not allowed")));
        local.send(json!({"op": "publish", "id": "p", "topic": topic, "msg": {"data": "x"}})).await;
        assert_eq!(local.recv().await, status(Some("p"), &format!("publish: {topic} is not allowed")));
    }
    for service in ["/not_public", "/map_manage/set_parameters", "/map_manage/set_parameters_atomically", "/rm_rf", "/mowerd/nonsense", "/mowerd", "/mowerd/topics/x"] {
        local.call("c", service, json!({})).await;
        assert_eq!(local.recv().await, answer("c", service, json!(format!("service {service} is not allowed")), false));
    }
    // what the app may do, it may do, as before
    local.call("c", "/check_nav_status", json!({})).await;
    assert_eq!(local.recv().await, answer("c", "/check_nav_status", json!({"message": "{\"state\":\"idle\"}", "success": true}), true));
    robot.close().await;
}

/// A parameter read reaches rcl only with a name rcl can hold, and only then
/// looks for a server; without a context (no DDS here) none answers.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn parameter_reads_take_real_names_only() {
    let robot = Robot::with_the_shipped_policy().await;
    let mut local = robot.client(&robot.loopback).await;
    // (a NUL in a name is `is_ros_name`'s test: this file's clients keep it off
    // the socket, because client.rs's log line of a refused name panics on it)
    for bad in ["/a b/get_parameters", "//get_parameters", "/1a/list_parameters", "/get_parameters"] {
        local.call("c", bad, json!({})).await;
        assert_eq!(local.recv().await, answer("c", bad, json!(format!("service {bad} is not allowed")), false));
    }
    local.call("c", "/map_manage/get_parameters", json!({"names": ["a"]})).await;
    assert_eq!(local.recv().await, answer("c", "/map_manage/get_parameters", json!("/map_manage/get_parameters: service unavailable"), false));
    local.call("c", "/map_manage/get_parameters", json!({"names": ["a\u{0}b"]})).await;
    assert_eq!(local.recv().await, answer("c", "/map_manage/get_parameters", json!("/map_manage/get_parameters: request failed: /names/0 holds a NUL character"), false));
    robot.close().await;
}

/// A name only the loopback policy allows: read in the scope the manifest
/// check does not look at, delivered as to any client, gone with the linger.
/// A typo is refused like any client's and leaves no channel behind.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn the_loopback_reads_what_the_bus_knows_and_a_typo_leaves_nothing() {
    let robot = Robot::with_the_shipped_policy().await;
    let mut local = robot.client(&robot.loopback).await;

    local.subscribe("/mower_base/telemetry", 0).await;
    eventually("the reader", || !robot.scopes_reading("/mower_base/telemetry").is_empty()).await;
    assert_eq!(robot.scopes_reading("/mower_base/telemetry"), [mower_bridge::coverage::LOOPBACK_SCOPE]);
    robot.modules.telemetry.publish(&text("{\"voltage\":24}")).unwrap();
    assert_eq!(local.recv().await, publish_frame("/mower_base/telemetry", json!({"data": "{\"voltage\":24}"})));

    // the manifest check has nothing to say about it, and would about a reader
    // of the module's own scope
    let running = mower_bridge::Running::rsd(&["bridge"], mower_bridge::on_bus_with(&["bridge"]));
    let gaps = |bus: &Bus| mower_bridge::coverage::gaps(bus, &running, &Default::default(), &[mower_bridge::NODE]);
    let own = |scope: &str, gap: &str| gap.starts_with(&format!("{scope} "));
    assert!(!gaps(&robot.bus).iter().any(|g| own(mower_bridge::coverage::LOOPBACK_SCOPE, g)), "{:?}", gaps(&robot.bus));
    let other = Bus::new();
    let _reader = other.scope("mower_ws_bridge").subscribe::<Text>("/mower_base/telemetry", READER_QOS).unwrap();
    assert!(gaps(&other).iter().any(|g| own("mower_ws_bridge", g)), "the check looks at the module's scope");

    // a typo: the texts of every client, and no channel
    local.send(json!({"op": "subscribe", "id": "s", "topic": "/odmo"})).await;
    assert_eq!(local.recv().await, status(Some("s"), "subscribe: cannot infer type of /odmo: not advertised yet"));
    assert_eq!(robot.bus.topic("/odmo"), None);
    local.send(json!({"op": "subscribe", "id": "s", "topic": "/mower_base/telemetry", "type": "std_msgs/msg/Bool"})).await;
    assert_eq!(local.recv().await, status(Some("s"), "subscribe: subscribe /mower_base/telemetry (std_msgs/msg/Bool) failed: the topic is std_msgs/msg/String"));

    // gone with the linger after the last client, as every entry
    local.send(json!({"op": "unsubscribe", "topic": "/mower_base/telemetry"})).await;
    let at = robot.manual.clock().mono() + LINGER;
    eventually("the linger", || robot.manual.next_deadline() == Some(at)).await;
    robot.manual.advance(LINGER);
    eventually("the reader to go", || robot.scopes_reading("/mower_base/telemetry").is_empty()).await;

    // a name the public policy allows is read as the app reads it
    local.subscribe("/fix", 0).await;
    eventually("the reader", || robot.scopes_reading("/fix").iter().any(|s| s == "mower_ws_bridge")).await;
    robot.close().await;
}

/// The fields of `/mowerd/*` that hold a time, replaced: the golden file holds
/// the rest. mowerctl's tests read the same file (its fake_bridge.rs), which
/// is what ties its parser to these answers.
fn without_times(value: &mut Value) {
    match value {
        Value::Object(map) => {
            for (k, v) in map.iter_mut() {
                if matches!(k.as_str(), "uptime_s" | "age_s" | "up_s") && !v.is_null() {
                    assert!(v.as_f64().is_some_and(|s| s >= 0.0), "{k}: {v}");
                    *v = json!(1.5);
                }
                without_times(v);
            }
        }
        Value::Array(items) => items.iter_mut().for_each(without_times),
        _ => {}
    }
}

/// What `/mowerd/*` says about a known set of endpoints, against
/// tests/golden/mowerd.json (`E82_WRITE_GOLDEN=1` rewrites it).
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn mowerd_answers_from_the_bus_and_the_registry() {
    let robot = Robot::with_the_shipped_policy().await;
    robot.registry.add_slot("gps/gps", "gps");
    robot.registry.slot_started("gps/gps");
    robot.registry.add_slot("imu/imu", "imu");
    robot.registry.slot_started("imu/imu");
    robot.registry.slot_failed("imu/imu", "imu/imu: no such device", true);
    let _probe = robot.registry.set_bridge_probe(Arc::new(|| {
        json!([{"name": "/fix", "direction": "to_ros", "forwarded": 12, "skipped": 3, "stale": 0, "resyncs": 1, "unanswered": 0, "dropped": 0,
                "invalid": 0, "dds_writer": true, "dds_reader": false, "dds_server": false, "bus_writer": false, "bus_server": false}])
    }));
    let mut local = robot.client(&robot.loopback).await;
    let mut golden = serde_json::Map::new();
    for (key, service, args) in [
        ("topics", "/mowerd/topics", json!({})),
        ("topics_named", "/mowerd/topics", json!({"names": ["/mower_base/telemetry", "/no_such_topic"]})),
        ("services", "/mowerd/services", json!({})),
        ("services_named", "/mowerd/services", json!({"names": ["/not_public", "/no_such_service"]})),
        ("actions", "/mowerd/actions", json!({})),
        ("modules", "/mowerd/modules", json!({})),
        ("bridge", "/mowerd/bridge", json!({})),
    ] {
        local.call("m", service, args).await;
        let frame = local.recv_json().await;
        assert_eq!(frame["result"], true, "{frame}");
        let mut values = frame["values"].clone();
        without_times(&mut values);
        golden.insert(key.to_string(), values);
    }
    let found = serde_json::to_string_pretty(&Value::Object(golden)).unwrap() + "\n";
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/golden/mowerd.json");
    if std::env::var_os("E82_WRITE_GOLDEN").is_some() {
        std::fs::write(path, &found).unwrap();
    }
    assert_eq!(found, std::fs::read_to_string(path).unwrap(), "{path}");

    // no bridge: says so
    drop(_probe);
    local.call("b", "/mowerd/bridge", json!({})).await;
    assert_eq!(local.recv_json().await["values"], json!({"running": false, "entries": []}));
    robot.close().await;
}

/// `published` and `age_s` move with the bus, and with nothing else: a client
/// that only reads the counters adds no reader (what makes `hz` free).
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn reading_the_counters_adds_no_reader() {
    let robot = Robot::with_the_shipped_policy().await;
    let mut local = robot.client(&robot.loopback).await;
    let counted = |frame: &Value| frame["values"]["topics"][0]["published"].as_u64().unwrap();
    local.call("m", "/mowerd/topics", json!({"names": ["/mower_base/telemetry"]})).await;
    let first = local.recv_json().await;
    for _ in 0..3 {
        robot.modules.telemetry.publish(&text("{}")).unwrap();
    }
    local.call("m", "/mowerd/topics", json!({"names": ["/mower_base/telemetry"]})).await;
    let second = local.recv_json().await;
    assert_eq!(counted(&second), counted(&first) + 3);
    assert!(second["values"]["uptime_s"].as_f64() > first["values"]["uptime_s"].as_f64());
    assert_eq!(second["values"]["topics"][0]["readers"], json!([]));
    assert!(robot.scopes_reading("/mower_base/telemetry").is_empty());
    robot.close().await;
}
