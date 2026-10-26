//! flutter_adapter on the bus (step E2): the module's real `run`, with its
//! inputs published and its services served from another scope of the same
//! `Bus`, as the other modules of `mower_rsd` do. The last test runs it with
//! the real map_manage and path_record_node, as production does.
//!
//! - **Its r2r node** is still created (parameters, `/rosout`) and polls the
//!   two parameter services over DDS, on ROS domain 85 so that no other DDS
//!   user on the host sees it. One test at a time: the context is
//!   process-wide and the services have fixed names.
//! - **Time** is a manual clock, except in the last test: the 1 Hz settings
//!   timer, the 2 Hz zone-list poll, the toLL retries and the pose age run
//!   on it.
//! - **The JSON** of a layer is compared as text with what `serde_json`
//!   makes of the DTO the Python adapter defined, which is the app's bytes.

use std::future::Future;
use std::panic::AssertUnwindSafe;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};
use std::time::Duration;

use futures::{FutureExt, StreamExt};
use mower_bus::{Bus, Clock, Message, Publisher, Qos, Scope, Subscriber, Time};
use mower_rs_common::{ModuleCtx, ModuleResult, Shutdown};
use r2r::geographic_msgs::msg::GeoPoint;
use r2r::geometry_msgs::msg::{Point, PoseStamped};
use r2r::mower_interface::msg::ZoneMap;
use r2r::mower_interface::srv::{MissionOperationLock, ZoneMapList};
use r2r::nav_msgs::msg::{OccupancyGrid, Odometry};
use r2r::rcl_interfaces::msg::ParameterValue;
use r2r::rcl_interfaces::srv::GetParameters;
use r2r::robot_localization::srv::ToLL;
use r2r::std_msgs::msg::{Bool, ColorRGBA, String as StringMsg};
use r2r::std_srvs::srv::Trigger;
use r2r::visualization_msgs::msg::{Marker, MarkerArray};
use r2r::QosProfile;
use serde_json::{json, Value};

/// For anything that waits in real time. A pass never waits this long.
const WAIT: Duration = Duration::from_secs(20);

/// 2027-01-15 08:00:00 UTC, the manual clocks' wall time at start.
const T0: Time = Time::from_ns(1_800_000_000_000_000_000);

const LAYERS: [&str; 10] = [
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
];

fn context() -> r2r::Context {
    static CONTEXT: OnceLock<r2r::Context> = OnceLock::new();
    CONTEXT
        .get_or_init(|| {
            // rcl reads it when the context is created, and only then.
            std::env::set_var("ROS_DOMAIN_ID", "85");
            r2r::Context::create().expect("an rcl context (source /opt/ros/jazzy/setup.bash)")
        })
        .clone()
}

fn serial() -> MutexGuard<'static, ()> {
    static SERIAL: Mutex<()> = Mutex::new(());
    SERIAL.lock().unwrap_or_else(|e| e.into_inner())
}

/// Run one test on a runtime of its own, within a bound only a hang reaches.
///
/// A test that fails half-way drops its harness, which only asks the module
/// to stop (`Drop for Harness`): the module's own task still has to stop its
/// r2r spin thread, and a runtime does not go while a blocking thread runs.
/// So the failure waits a moment for that, and the runtime is shut down with
/// a bound, before it is reported.
fn within(test: impl Future<Output = ()>) {
    let _serial = serial();
    let runtime = tokio::runtime::Builder::new_multi_thread().worker_threads(2).enable_all().build().unwrap();
    let outcome = runtime.block_on(async {
        let outcome = AssertUnwindSafe(tokio::time::timeout(Duration::from_secs(120), test)).catch_unwind().await;
        if !matches!(outcome, Ok(Ok(()))) {
            tokio::time::sleep(Duration::from_secs(1)).await;
        }
        outcome
    });
    runtime.shutdown_timeout(Duration::from_secs(5));
    match outcome {
        Ok(passed) => passed.expect("the test hung"),
        Err(panic) => std::panic::resume_unwind(panic),
    }
}

/// One module instance, as mower_rsd runs it: a scope of its own on the bus.
struct Module {
    scope: Scope,
    shutdown: Shutdown,
    task: tokio::task::JoinHandle<ModuleResult>,
}

impl Module {
    fn start<F>(bus: &Bus, node: &str, clock: Clock, run: impl FnOnce(r2r::Context, ModuleCtx) -> F) -> Module
    where
        F: Future<Output = ModuleResult> + Send + 'static,
    {
        let scope = bus.scope(node);
        let shutdown = Shutdown::new();
        let m = ModuleCtx::new(node, shutdown.clone()).with_bus(scope.clone()).with_clock(clock);
        let task = tokio::spawn(run(context(), m));
        Module { scope, shutdown, task }
    }

    /// What mower_rsd does: stop the module, then close its scope.
    async fn stop(mut self) {
        self.shutdown.trigger();
        (&mut self.task).await.expect("the module panicked").expect("the module failed");
        self.scope.close();
    }
}

impl Drop for Module {
    /// A test that fails half-way still stops the module (see `within`).
    fn drop(&mut self) {
        self.shutdown.trigger();
    }
}

/// The adapter and the scope the test talks to it from.
struct Harness {
    bus: Bus,
    test: Scope,
    adapter: Module,
}

impl Harness {
    /// Start flutter_adapter on `bus`, and wait until it is up: its last
    /// endpoint is the `/toLL` client.
    async fn start_on(bus: Bus, clock: Clock) -> Harness {
        let test = bus.scope("test");
        let adapter = Module::start(&bus, "flutter_adapter", clock, mower_adapter::run);
        eventually("the adapter's endpoints", || {
            bus.services().iter().any(|s| s.name == "/toLL" && s.clients.iter().any(|c| c == "flutter_adapter"))
        })
        .await;
        Harness { bus, test, adapter }
    }

    async fn start(clock: Clock) -> Harness {
        Harness::start_on(Bus::new(), clock).await
    }

    async fn stop(self) {
        self.adapter.stop().await;
    }

    fn latched<T: Message>(&self, topic: &str) -> Subscriber<T> {
        self.test.subscribe::<T>(topic, Qos::latched().keep_last(10)).unwrap()
    }

    /// A latched writer on `topic`, as map_manage, the recorder and coverage
    /// have, with `msg` published.
    fn publish_latched<T: Message>(&self, topic: &str, msg: &T) -> Publisher<T> {
        let publisher = self.test.publisher::<T>(topic, Qos::latched()).unwrap();
        publisher.publish(msg).unwrap();
        publisher
    }

    fn retained(&self, topic: &str) -> usize {
        self.bus.topics().into_iter().find(|t| t.name == topic).map_or(0, |t| t.retained)
    }
}

async fn eventually(what: &str, mut ready: impl FnMut() -> bool) {
    let give_up = std::time::Instant::now() + WAIT;
    while !ready() {
        assert!(std::time::Instant::now() < give_up, "{what} in time");
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
}

async fn next<T: Message>(sub: &mut Subscriber<T>) -> Arc<T> {
    tokio::time::timeout(WAIT, sub.next()).await.expect("a message in time").expect("the bus is open")
}

/// The next message on `sub` within a short while, if any.
async fn soon<T: Message>(sub: &mut Subscriber<T>) -> Option<Arc<T>> {
    tokio::time::timeout(Duration::from_millis(200), sub.next()).await.ok().flatten()
}

/// Move the clock on by `step` until `sub` brings a message `done` accepts.
/// An advance that comes before the module has armed its timer is lost,
/// which is why this keeps going rather than counting steps.
async fn advance_until<T: Message>(manual: &mower_bus::ManualClock, step: Duration, sub: &mut Subscriber<T>, done: impl Fn(&T) -> bool) {
    let give_up = std::time::Instant::now() + WAIT;
    loop {
        if soon(sub).await.is_some_and(|msg| done(&msg)) {
            return;
        }
        assert!(std::time::Instant::now() < give_up, "no such message in time");
        manual.advance(step);
    }
}

/// The text the adapter publishes for `value`: serde_json's, as in the module.
fn text(value: &Value) -> String {
    serde_json::to_string(value).unwrap()
}

fn grid() -> OccupancyGrid {
    let mut grid = OccupancyGrid::default();
    grid.info.resolution = 0.05;
    grid.info.width = 4;
    grid.info.height = 3;
    grid.info.origin.position.x = -1.5;
    grid.info.origin.position.y = 2.0;
    grid.data = vec![0, 100, -1, 50, 0, 0, 0, 0, 0, 0, 0, 0];
    grid
}

/// The DTO of [`grid`] (dto.py's MapLayer), named `name`.
fn grid_layer(name: &str) -> Value {
    json!({
        "name": name,
        "type": "occupancy_grid",
        "resolution": 0.05f32 as f64,
        "width": 4,
        "height": 3,
        "origin": {"x": -1.5, "y": 2.0},
        "encoding": "base64",
        "data": "AGT/MgAAAAAAAAAA",
    })
}

fn line_strip(ns: &str, id: i32, points: usize) -> Marker {
    Marker {
        ns: ns.into(),
        id,
        type_: 4,
        color: ColorRGBA { r: 0.6, g: 0.0, b: 1.0, a: 0.5 },
        points: (0..points).map(|i| Point { x: i as f64, y: 0.5, z: 0.0 }).collect(),
        ..Default::default()
    }
}

#[test]
fn map_and_marker_layers_are_relayed_as_latched_json() {
    within(async {
        let (clock, _manual) = Clock::manual(T0);
        let bus = Bus::new();
        // A latched source that published before the adapter started: it
        // gets the value when it subscribes, as from DDS.
        let early = bus.scope("early");
        let free = early.publisher::<OccupancyGrid>("/free_space_inflated", Qos::latched()).unwrap();
        free.publish(&grid()).unwrap();
        let adapter = Harness::start_on(bus, clock).await;

        let topics = adapter.bus.topics();
        for topic in LAYERS {
            let info = topics.iter().find(|t| t.name == topic).unwrap_or_else(|| panic!("{topic} on the bus"));
            assert_eq!(info.publishers, ["flutter_adapter"], "{topic}");
            assert_eq!(info.ros_type.as_deref(), Some("std_msgs/msg/String"));
        }
        // An empty marker layer from the start, so the app never keeps
        // another session's.
        for (topic, name) in [("/adapter/marker_layers/zones", "zones"), ("/adapter/marker_layers/connectors", "connectors")] {
            let layer = next(&mut adapter.latched::<StringMsg>(topic)).await;
            assert_eq!(layer.data, text(&json!({"name": name, "markers": []})));
        }
        let layer = next(&mut adapter.latched::<StringMsg>("/adapter/map_layers/free_space_inflated")).await;
        assert_eq!(layer.data, text(&grid_layer("free_space_inflated")));

        let mut map_grid = adapter.latched::<StringMsg>("/adapter/map_layers/map_grid");
        assert_eq!(adapter.retained("/adapter/map_layers/map_grid"), 0, "nothing until the source publishes");
        let _source = adapter.publish_latched("/map_grid", &grid());
        assert_eq!(next(&mut map_grid).await.data, text(&grid_layer("map_grid")));
        let mut markers = adapter.latched::<StringMsg>("/adapter/marker_layers/coverage_path");
        assert_eq!(next(&mut markers).await.data, text(&json!({"name": "coverage_path", "markers": []})));
        let _coverage = adapter.publish_latched("/coverage_path_markers", &MarkerArray { markers: vec![line_strip("zone_1_path", 7, 2)] });
        let expected = json!({"name": "coverage_path", "markers": [
            {"id": 7, "type": "line_strip", "color": "#9900ff", "points": [{"x": 0.0, "y": 0.5}, {"x": 1.0, "y": 0.5}]},
        ]});
        assert_eq!(next(&mut markers).await.data, text(&expected));

        // Late joiners, the app's and the recorder's, get the newest layer only.
        for (topic, layer) in [
            ("/adapter/map_layers/map_grid", text(&grid_layer("map_grid"))),
            ("/adapter/marker_layers/coverage_path", text(&expected)),
        ] {
            let mut late = adapter.latched::<StringMsg>(topic);
            assert_eq!(next(&mut late).await.data, layer, "{topic}");
            assert!(late.try_recv().is_err(), "{topic}: keep last 1");
            assert_eq!(adapter.retained(topic), 1);
        }
        adapter.stop().await;
    });
}

fn odom(stamp: Time, frame: &str, x: f64) -> Odometry {
    let mut msg = Odometry::default();
    (msg.header.stamp.sec, msg.header.stamp.nanosec) = stamp.to_parts();
    msg.header.frame_id = frame.into();
    msg.child_frame_id = "base_footprint".into();
    msg.pose.pose.position.x = x;
    msg.pose.pose.orientation.w = 1.0;
    msg
}

#[test]
fn the_robot_pose_is_relayed_when_fresh_and_in_the_map_frame() {
    within(async {
        let (clock, manual) = Clock::manual(T0);
        let adapter = Harness::start(clock.clone()).await;
        let source = adapter.test.publisher::<Odometry>("/odometry/global", Qos::default()).unwrap();
        let mut pose = adapter.test.subscribe::<PoseStamped>("/adapter/robot_pose", Qos::default()).unwrap();
        manual.advance(Duration::from_secs(10));
        let now = clock.now();
        source.publish(&odom(now - Duration::from_secs(2), "map", 1.0)).unwrap(); // stale on the module's clock
        source.publish(&odom(now, "odom", 2.0)).unwrap(); // another frame
        source.publish(&odom(now - Duration::from_millis(900), "map", 3.0)).unwrap();
        let got = next(&mut pose).await;
        assert_eq!((got.header.frame_id.as_str(), got.pose.position.x), ("map", 3.0));
        assert_eq!((got.header.stamp.sec, got.header.stamp.nanosec), (now - Duration::from_millis(900)).to_parts());
        // Volatile, keep last 10: no history for a late joiner.
        assert_eq!(adapter.retained("/adapter/robot_pose"), 0);
        assert!(soon(&mut pose).await.is_none());
        adapter.stop().await;
    });
}

fn zone(id: i32, path_poses: usize) -> ZoneMap {
    let mut zone = ZoneMap { zone_id: id, ..Default::default() };
    zone.mask_map_inflated = grid();
    zone.path.poses = vec![PoseStamped::default(); path_poses];
    zone
}

/// map_manage's zone list service, answering with `zones` and counting the
/// calls. Abort the task to take the server away.
fn serve_zones(test: &Scope, zones: Vec<ZoneMap>) -> (tokio::task::JoinHandle<()>, Arc<AtomicUsize>) {
    let calls = Arc::new(AtomicUsize::new(0));
    let mut srv = test.service::<ZoneMapList::Request, ZoneMapList::Response>("/get_zone_map_list_srv").unwrap();
    let count = calls.clone();
    let task = tokio::spawn(async move {
        while let Some(req) = srv.next().await {
            count.fetch_add(1, Ordering::SeqCst);
            let _ = req.respond(ZoneMapList::Response { zone_map_list: zones.clone() });
        }
    });
    (task, calls)
}

#[test]
fn zone_summaries_poll_the_zone_list_at_2_hz_and_ride_out_a_server_restart() {
    within(async {
        let (clock, manual) = Clock::manual(T0);
        let adapter = Harness::start(clock).await;
        let mut summaries = adapter.latched::<StringMsg>("/adapter/zone_summaries");
        let _coverage = adapter.publish_latched("/coverage_path_markers", &MarkerArray { markers: vec![line_strip("zone_4_path", 1, 7)] });
        let (server, calls) = serve_zones(&adapter.test, vec![zone(3, 1), zone(4, 2)]);
        let planned = text(&json!([
            {"zoneId": 3, "pointCount": 1, "hasMap": true, "hasCoveragePath": true},
            {"zoneId": 4, "pointCount": 7, "hasMap": true, "hasCoveragePath": true},
        ]));
        // The first poll can come before the coverage markers are in.
        advance_until(&manual, Duration::from_millis(500), &mut summaries, |s| s.data == planned).await;
        // The next poll waits for the module's clock.
        tokio::time::sleep(Duration::from_millis(100)).await;
        let polled = calls.load(Ordering::SeqCst);
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert_eq!(calls.load(Ordering::SeqCst), polled);
        manual.advance(Duration::from_millis(500));
        assert_eq!(next(&mut summaries).await.data, planned);
        assert_eq!(calls.load(Ordering::SeqCst), polled + 1);

        // map_manage restarts: no server for a while, then a new one.
        server.abort();
        let _ = server.await;
        for _ in 0..3 {
            manual.advance(Duration::from_millis(500));
            assert!(soon(&mut summaries).await.is_none(), "nothing to summarise without the server");
        }
        let (_server, _calls) = serve_zones(&adapter.test, vec![zone(5, 0)]);
        let restarted = text(&json!([{"zoneId": 5, "pointCount": 0, "hasMap": true, "hasCoveragePath": false}]));
        advance_until(&manual, Duration::from_millis(500), &mut summaries, |s| s.data == restarted).await;
        let mut late = adapter.latched::<StringMsg>("/adapter/zone_summaries");
        assert_eq!(next(&mut late).await.data, restarted);
        adapter.stop().await;
    });
}

#[test]
fn the_map_datum_comes_from_to_ll_once_navsat_has_one_and_stays_latched() {
    within(async {
        let (clock, manual) = Clock::manual(T0);
        let adapter = Harness::start(clock).await;
        let mut datum = adapter.latched::<StringMsg>("/adapter/map_datum");
        // navsat_transform: (0, 0, 0) until its datum exists (two calls),
        // then a fixed datum, 1e-5 degrees of latitude per metre of map x.
        let calls = Arc::new(AtomicUsize::new(0));
        let mut srv = adapter.test.service::<ToLL::Request, ToLL::Response>("/toLL").unwrap();
        let count = calls.clone();
        tokio::spawn(async move {
            while let Some(req) = srv.next().await {
                let n = count.fetch_add(1, Ordering::SeqCst);
                let ll_point = if n < 2 {
                    GeoPoint::default()
                } else {
                    GeoPoint { latitude: 23.5 + req.message.map_point.x * 1e-5, longitude: 120.5, altitude: 0.0 }
                };
                let _ = req.respond(ToLL::Response { ll_point });
            }
        });
        let expected = json!({"origin_lat": 23.5, "origin_lon": 120.5, "bearing_rad": 0.0, "source": "navsat"});
        // Every publish is the datum: with the fallback off, nothing else goes out.
        advance_until(&manual, Duration::from_secs(2), &mut datum, |d| {
            assert_eq!(d.data, text(&expected));
            true
        })
        .await;
        // origin and probe after the two answers without a datum
        assert_eq!(calls.load(Ordering::SeqCst), 4);
        // Locked: no more calls, and the datum stays for late joiners after
        // the task that published it has ended.
        manual.advance(Duration::from_secs(10));
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert_eq!(calls.load(Ordering::SeqCst), 4);
        let mut late = adapter.latched::<StringMsg>("/adapter/map_datum");
        assert_eq!(next(&mut late).await.data, text(&expected));
        assert_eq!(adapter.retained("/adapter/map_datum"), 1);
        adapter.stop().await;
    });
}

/// A node of the process that is not a module, spun on a thread of its own:
/// it serves parameter services over DDS, as boustrophedon_coverage does.
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

    /// Serve `name` with `values` for whatever names are asked.
    fn serve_parameters(&self, name: &str, values: Vec<ParameterValue>) {
        let mut srv = self.node.lock().unwrap().create_service::<GetParameters::Service>(name, QosProfile::services_default()).unwrap();
        tokio::spawn(async move {
            while let Some(req) = srv.next().await {
                let _ = req.respond(GetParameters::Response { values: values.clone() });
            }
        });
    }
}

impl Drop for Peer {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Release);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

fn double(v: f64) -> ParameterValue {
    ParameterValue { type_: 3, double_value: v, ..Default::default() }
}

fn boolean(v: bool) -> ParameterValue {
    ParameterValue { type_: 1, bool_value: v, ..Default::default() }
}

#[test]
fn coverage_settings_come_from_the_parameter_services_on_dds_at_1_hz() {
    within(async {
        let (clock, manual) = Clock::manual(T0);
        let adapter = Harness::start(clock).await;
        let mut settings = adapter.latched::<StringMsg>("/adapter/coverage_settings");
        let peer = Peer::new("adapter_bus_test_peer");
        // COVERAGE_PARAM_NAMES's order
        let string = ParameterValue { type_: 4, string_value: "zigzag".into(), ..Default::default() };
        let coverage = vec![double(0.35), double(0.2), double(30.0), boolean(false), boolean(false), string, boolean(true)];
        peer.serve_parameters("/boustrophedon_coverage/get_parameters", coverage);
        peer.serve_parameters("/map_manage/get_parameters", vec![double(0.75)]);
        let expected = text(&json!({
            "stripWidthM": 0.35,
            "waypointSpacingM": 0.2,
            "zigzagAngleDeg": 30.0,
            "zigzagAutoAngle": false,
            "inflateRadiusM": 0.75,
            "unknownAsObstacle": false,
            "coveragePattern": "zigzag",
            "boundaryRing": true,
        }));
        // Each tick publishes what the two polls have brought in so far.
        advance_until(&manual, Duration::from_secs(1), &mut settings, |s| s.data == expected).await;
        while soon(&mut settings).await.is_some() {}
        // Nothing between ticks: the timer is on the module's clock, one
        // tick per second from the module's start.
        manual.advance(Duration::from_millis(500));
        assert!(soon(&mut settings).await.is_none());
        manual.advance(Duration::from_millis(500));
        assert_eq!(next(&mut settings).await.data, expected);
        drop(peer);
        adapter.stop().await;
    });
}

/// nav_action_server's lease service: grants every request.
fn serve_lock(test: &Scope) {
    let mut srv = test.service::<MissionOperationLock::Request, MissionOperationLock::Response>("/mission_operation_lock").unwrap();
    tokio::spawn(async move {
        while let Some(req) = srv.next().await {
            let _ = req.respond(MissionOperationLock::Response { success: true, message: String::new() });
        }
    });
}

#[test]
fn with_map_manage_and_the_recorder_the_mission_data_flows_in_process() {
    within(async {
        // The recorder keeps its files under the working directory and HOME.
        let dir = std::env::temp_dir().join(format!("mower_adapter_bus_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let zone = json!([{"id": 1, "ns": "zones", "points": [[0.0, 0.0, 0.0], [4.0, 0.0, 0.0], [4.0, 4.0, 0.0], [0.0, 4.0, 0.0]]}]);
        let risk = json!([{"id": 1, "ns": "risk_zones", "points": [[1.0, 1.0, 0.0], [1.5, 1.0, 0.0], [1.5, 1.5, 0.0], [1.0, 1.5, 0.0]]}]);
        std::fs::create_dir_all(dir.join("zone_record")).unwrap();
        std::fs::write(dir.join("zone_record/zone_list.json"), zone.to_string()).unwrap();
        std::fs::write(dir.join("zone_record/risk_zone_list.json"), risk.to_string()).unwrap();
        std::fs::write(dir.join("zone_record/chennal_path_list.json"), "[]").unwrap();
        std::env::set_current_dir(&dir).unwrap();
        std::env::set_var("HOME", &dir);

        // All three on one bus and the system clock, as in mower_rsd.
        let bus = Bus::new();
        let test = bus.scope("test");
        serve_lock(&test);
        let nav = test.publisher::<Bool>("/nav_operation_active", Qos::latched()).unwrap();
        nav.publish(&Bool { data: false }).unwrap();
        let record = Module::start(&bus, "path_record_node", Clock::system(), mower_record::run);
        let map = Module::start(&bus, "map_manage", Clock::system(), mower_map::run);
        let adapter = Harness::start_on(bus.clone(), Clock::system()).await;
        let create = test.client::<Trigger::Request, Trigger::Response>("/create_free_space").unwrap();
        tokio::time::timeout(WAIT, create.wait_for_service()).await.unwrap().unwrap();

        // The recorder's restored zone reaches the app's layer.
        let mut zones = adapter.latched::<StringMsg>("/adapter/marker_layers/zones");
        loop {
            let layer: Value = serde_json::from_str(&next(&mut zones).await.data).unwrap();
            if layer["markers"].as_array().is_some_and(|m| m.len() == 1) {
                assert_eq!(layer["markers"][0]["id"], 1);
                assert_eq!(layer["markers"][0]["color"], "#9900ff");
                break;
            }
        }
        // map_manage builds free space from the recorder's zone list service;
        // the adapter relays the grid and summarises map_manage's zone list.
        let mut free = adapter.latched::<StringMsg>("/adapter/map_layers/free_space_inflated");
        let mut summaries = adapter.latched::<StringMsg>("/adapter/zone_summaries");
        let answer = loop {
            let answer = tokio::time::timeout(WAIT, create.request(&Trigger::Request {}).unwrap()).await.unwrap().unwrap();
            if !answer.message.contains("navigation state is not available") {
                break answer;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        };
        assert_eq!((answer.success, answer.message.as_str()), (true, "成功創建自由空間，包含 1 個區域"));
        let layer: Value = serde_json::from_str(&next(&mut free).await.data).unwrap();
        assert_eq!((layer["name"].as_str(), layer["width"].as_u64(), layer["height"].as_u64()), (Some("free_space_inflated"), Some(120), Some(120)));
        let summary = text(&json!([{"zoneId": 1, "pointCount": 0, "hasMap": true, "hasCoveragePath": false}]));
        while next(&mut summaries).await.data != summary {}

        // ... and its 1 Hz poll of map_manage's parameters goes over DDS.
        let mut settings = adapter.latched::<StringMsg>("/adapter/coverage_settings");
        let settings: Value = serde_json::from_str(&next(&mut settings).await.data).unwrap();
        assert_eq!(settings["inflateRadiusM"], 0.75);
        assert_eq!(settings["stripWidthM"], Value::Null, "no coverage node here");

        let risk_map = test.client::<Trigger::Request, Trigger::Response>("/create_risk_map").unwrap();
        let answer = tokio::time::timeout(WAIT, risk_map.request(&Trigger::Request {}).unwrap()).await.unwrap().unwrap();
        assert_eq!((answer.success, answer.message.as_str()), (true, "成功創建風險地圖，包含 1 個風險區域"));
        let layer: Value = serde_json::from_str(&next(&mut adapter.latched::<StringMsg>("/adapter/map_layers/risk_map_inflated")).await.data).unwrap();
        assert_eq!(layer["width"], 120);

        adapter.stop().await;
        map.stop().await;
        record.stop().await;
        std::env::set_current_dir(std::env::temp_dir()).unwrap();
        let _ = std::fs::remove_dir_all(&dir);
    });
}
