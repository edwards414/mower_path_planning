//! path_record_node on the bus (step E2): the module's real `run`, with its
//! inputs published and its services called from another scope of the same
//! `Bus`, as the other modules of `mower_rsd` do.
//!
//! - **Its r2r node** is still created (parameters, `/rosout`), on ROS
//!   domain 84 so that no other DDS user on the host sees it.
//! - **Files:** the recorder keeps its work files under `save_dir`
//!   (`zone_record`, relative) and its sites under `~/.mower/sites`, and a
//!   test cannot pass parameters. So each test runs in a directory of its
//!   own that is both the working directory and `HOME`, and the tests run
//!   one at a time.
//! - **Time** is a manual clock: the 0.5 s start-up check, the 10 Hz sampler
//!   and the rear light run on it, and so do the pose ages and stamps.

use std::future::Future;
use std::panic::AssertUnwindSafe;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};
use std::time::Duration;

use futures::{FutureExt, StreamExt};
use mower_bus::{Bus, Clock, ManualClock, Message, Publisher, Qos, Scope, Subscriber, Time};
use mower_rs_common::{ModuleCtx, ModuleResult, Shutdown};
use r2r::geometry_msgs::msg::Point;
use r2r::mower_interface::srv::{ChannelPathList, ChannelRoute, ChennalPathList, EditZone, GetZoneList, MissionOperationLock, SiteOp};
use r2r::nav_msgs::msg::{Odometry, Path as PathMsg};
use r2r::std_msgs::msg::{Bool, String as StringMsg};
use r2r::std_srvs::srv::Trigger;
use r2r::visualization_msgs::msg::MarkerArray;
use serde_json::{json, Value};

/// For anything that waits in real time. A pass never waits this long.
const WAIT: Duration = Duration::from_secs(20);

/// 2027-01-15 08:00:00 UTC, the manual clocks' wall time at start.
const T0: Time = Time::from_ns(1_800_000_000_000_000_000);

const SERVICES: [&str; 19] = [
    "/record_zone_start",
    "/record_zone_end",
    "/risk_zone_start",
    "/risk_zone_end",
    "/channel_record_start",
    "/chennal_record_start",
    "/channel_record_end",
    "/chennal_record_end",
    "/record_cancel",
    "/save_zone_list",
    "/load_zone_list",
    "/get_record_zone_info",
    "/edit_zone",
    "/get_record_zone_list",
    "/get_risk_zone_list",
    "/get_channel_path_list",
    "/get_chennal_path_list",
    "/get_channel_route",
    "/site_op",
];

fn context() -> r2r::Context {
    static CONTEXT: OnceLock<r2r::Context> = OnceLock::new();
    CONTEXT
        .get_or_init(|| {
            // rcl reads it when the context is created, and only then.
            std::env::set_var("ROS_DOMAIN_ID", "84");
            r2r::Context::create().expect("an rcl context (source /opt/ros/jazzy/setup.bash)")
        })
        .clone()
}

fn serial() -> MutexGuard<'static, ()> {
    static SERIAL: Mutex<()> = Mutex::new(());
    SERIAL.lock().unwrap_or_else(|e| e.into_inner())
}

/// Run one test on a runtime of its own, in the directory `dir` (made
/// fresh), within a bound only a hang reaches.
///
/// A test that fails half-way drops its harness, which only asks the module
/// to stop (`Drop for Harness`); the runtime is shut down with a bound, so
/// the failure is reported rather than waited on.
fn within<F: Future<Output = ()>>(dir: &str, test: impl FnOnce(PathBuf) -> F) {
    let _serial = serial();
    let dir = std::env::temp_dir().join(format!("mower_record_bus_{}_{dir}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    std::env::set_current_dir(&dir).unwrap();
    std::env::set_var("HOME", &dir);
    let runtime = tokio::runtime::Builder::new_multi_thread().worker_threads(2).enable_all().build().unwrap();
    let outcome = runtime.block_on(async {
        let outcome = AssertUnwindSafe(tokio::time::timeout(Duration::from_secs(120), test(dir.clone()))).catch_unwind().await;
        if !matches!(outcome, Ok(Ok(()))) {
            tokio::time::sleep(Duration::from_secs(1)).await;
        }
        outcome
    });
    runtime.shutdown_timeout(Duration::from_secs(5));
    let _ = std::fs::remove_dir_all(&dir);
    match outcome {
        Ok(passed) => passed.expect("the test hung"),
        Err(panic) => std::panic::resume_unwind(panic),
    }
}

/// The module under test and the scope the test talks to it from.
struct Harness {
    bus: Bus,
    test: Scope,
    scope: Scope,
    shutdown: Shutdown,
    task: tokio::task::JoinHandle<ModuleResult>,
}

impl Harness {
    /// Start path_record_node on a fresh bus, and wait until it is up: its
    /// last endpoint is the `/site_op` server.
    async fn start(clock: Clock) -> Harness {
        let bus = Bus::new();
        let scope = bus.scope("path_record_node");
        let test = bus.scope("test");
        let shutdown = Shutdown::new();
        let m = ModuleCtx::new("path_record_node", shutdown.clone()).with_bus(scope.clone()).with_clock(clock);
        let task = tokio::spawn(mower_record::run(context(), m));
        let up = test.client::<SiteOp::Request, SiteOp::Response>("/site_op").unwrap();
        tokio::time::timeout(WAIT, up.wait_for_service()).await.expect("the module in time").unwrap();
        Harness { bus, test, scope, shutdown, task }
    }

    /// What mower_rsd does: stop the module, then close its scope.
    async fn stop(mut self) {
        self.shutdown.trigger();
        (&mut self.task).await.expect("the module panicked").expect("the module failed");
        self.scope.close();
    }

    fn latched<T: Message>(&self, topic: &str) -> Subscriber<T> {
        self.test.subscribe::<T>(topic, Qos::latched().keep_last(10)).unwrap()
    }

    async fn call<Req: Message, Resp: Message>(&self, name: &str, request: Req) -> Resp {
        let client = self.test.client::<Req, Resp>(name).unwrap();
        tokio::time::timeout(WAIT, client.wait_for_service()).await.expect("the service in time").unwrap();
        tokio::time::timeout(WAIT, client.request(&request).unwrap()).await.expect("an answer in time").unwrap()
    }

    async fn trigger(&self, name: &str) -> (bool, String) {
        let response: Trigger::Response = self.call(name, Trigger::Request {}).await;
        (response.success, response.message)
    }

    fn nav_active(&self, active: bool) -> Publisher<Bool> {
        let publisher = self.test.publisher::<Bool>("/nav_operation_active", Qos::latched()).unwrap();
        publisher.publish(&Bool { data: active }).unwrap();
        publisher
    }

    fn retained(&self, topic: &str) -> usize {
        self.bus.topics().into_iter().find(|t| t.name == topic).map_or(0, |t| t.retained)
    }
}

impl Drop for Harness {
    /// A test that fails half-way still stops the module (see `within`).
    fn drop(&mut self) {
        self.shutdown.trigger();
    }
}

async fn next<T: Message>(sub: &mut Subscriber<T>) -> Arc<T> {
    tokio::time::timeout(WAIT, sub.next()).await.expect("a message in time").expect("the bus is open")
}

/// The first answer of `call` that the nav activity flag has reached: the
/// module takes it in on a task of its own, so a call can come first.
async fn once_nav_is_seen<Fut: Future<Output = (bool, String)>>(mut call: impl FnMut() -> Fut) -> (bool, String) {
    loop {
        let answer = call().await;
        if !answer.1.contains("navigation state is not available") {
            return answer;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
}

/// nav_action_server's lease service: grants every request and keeps it.
fn serve_lock(test: &Scope) -> Arc<Mutex<Vec<MissionOperationLock::Request>>> {
    let calls: Arc<Mutex<Vec<MissionOperationLock::Request>>> = Arc::default();
    let mut srv = test.service::<MissionOperationLock::Request, MissionOperationLock::Response>("/mission_operation_lock").unwrap();
    let log = calls.clone();
    tokio::spawn(async move {
        while let Some(req) = srv.next().await {
            log.lock().unwrap().push(req.message.clone());
            let _ = req.respond(MissionOperationLock::Response { success: true, message: String::new() });
        }
    });
    calls
}

fn ops(lock: &Mutex<Vec<MissionOperationLock::Request>>) -> Vec<(bool, String)> {
    lock.lock().unwrap().iter().map(|r| (r.acquire, r.operation.clone())).collect()
}

/// The EKF's map pose at `(x, y)`, stamped now.
fn odom(clock: &Clock, x: f64, y: f64) -> Odometry {
    let mut msg = Odometry::default();
    (msg.header.stamp.sec, msg.header.stamp.nanosec) = clock.now().to_parts();
    msg.header.frame_id = "map".into();
    msg.child_frame_id = "base_footprint".into();
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.orientation.w = 1.0;
    msg
}

fn write_json(path: &Path, value: &Value) {
    std::fs::create_dir_all(path.parent().unwrap()).unwrap();
    std::fs::write(path, serde_json::to_vec(value).unwrap()).unwrap();
}

fn square(x0: f64, y0: f64, side: f64) -> Value {
    json!([[x0, y0, 0.0], [x0 + side, y0, 0.0], [x0 + side, y0 + side, 0.0], [x0, y0 + side, 0.0]])
}

#[test]
fn restored_geometry_is_latched_and_served_to_the_map_and_coverage() {
    within("restored", |dir| async move {
        // What an earlier run left in zone_record/: two zones, a risk zone
        // and a channel between the zones.
        let work = dir.join("zone_record");
        write_json(&work.join("zone_list.json"), &json!([
            {"id": 1, "ns": "zones", "points": square(0.0, 0.0, 4.0)},
            {"id": 2, "ns": "zones", "points": square(10.0, 0.0, 4.0)},
        ]));
        write_json(&work.join("risk_zone_list.json"), &json!([{"id": 1, "ns": "risk_zones", "points": square(1.0, 1.0, 0.5)}]));
        write_json(&work.join("chennal_path_list.json"), &json!([
            {"id": 1, "ns": "chennal", "points": [[4.5, 2.0, 0.0], [9.5, 2.0, 0.0]], "color": {"r": 0.0, "g": 1.0, "b": 0.0, "a": 0.8}, "scale": 0.3},
        ]));
        let (clock, _manual) = Clock::manual(T0);
        let rec = Harness::start(clock).await;

        // Every latched list replays to a late joiner, once.
        for (topic, n) in [("/zone_list", 2), ("/risk_zone_list", 1), ("/chennal_path_array", 1), ("/channel_path_array", 1)] {
            let mut late = rec.latched::<MarkerArray>(topic);
            let list = next(&mut late).await;
            assert_eq!(list.markers.len(), n, "{topic}");
            assert_eq!((list.markers[0].header.stamp.sec, list.markers[0].header.stamp.nanosec), T0.to_parts());
            assert!(late.try_recv().is_err(), "{topic}: keep last 1");
            assert_eq!(rec.retained(topic), 1, "{topic}");
        }
        let sites = next(&mut rec.latched::<StringMsg>("/site_list")).await;
        assert_eq!(serde_json::from_str::<Value>(&sites.data).unwrap(), json!({"active": null, "sites": []}));

        let services = rec.bus.services();
        for name in SERVICES {
            let info = services.iter().find(|s| s.name == name).unwrap_or_else(|| panic!("{name} on the bus"));
            assert_eq!(info.server.as_deref(), Some("path_record_node"), "{name}");
        }

        // map_manage's three calls and coverage's route lookup.
        let zones: GetZoneList::Response = rec.call("/get_record_zone_list", GetZoneList::Request {}).await;
        assert!(zones.success);
        assert_eq!(zones.message, "成功獲取普通區域列表，共 2 個區域");
        assert_eq!(zones.zone_list.markers.iter().map(|m| m.id).collect::<Vec<_>>(), [1, 2]);
        let risks: GetZoneList::Response = rec.call("/get_risk_zone_list", GetZoneList::Request {}).await;
        assert_eq!((risks.success, risks.message.as_str(), risks.zone_list.markers.len()), (true, "成功獲取風險區域列表，共 1 個風險區域", 1));
        let chennal: ChennalPathList::Response = rec.call("/get_chennal_path_list", ChennalPathList::Request {}).await;
        assert_eq!((chennal.success, chennal.message.as_str(), chennal.chennal_path_array.markers.len()), (true, "成功獲取 chennal 路徑列表", 1));
        let channel: ChannelPathList::Response = rec.call("/get_channel_path_list", ChannelPathList::Request {}).await;
        assert_eq!((channel.success, channel.message.as_str(), channel.channel_path_array.markers.len()), (true, "成功獲取 channel 路徑列表", 1));

        let route: ChannelRoute::Response =
            rec.call("/get_channel_route", ChannelRoute::Request { zone_from_id: 2, zone_to_id: 1, proximity_m: 0.0 }).await;
        assert!(route.success, "{}", route.message);
        assert_eq!((route.matched_channel_id, route.message.as_str()), (1, "找到通道 #1，連接 zone 2 → zone 1，共 2 個路徑點"));
        // reversed to run from zone 2 to zone 1
        let xs: Vec<f64> = route.channel_path.poses.iter().map(|p| p.pose.position.x).collect();
        assert_eq!(xs, [9.5, 4.5]);
        assert_eq!((route.channel_path.header.stamp.sec, route.channel_path.header.stamp.nanosec), T0.to_parts());
        let none: ChannelRoute::Response =
            rec.call("/get_channel_route", ChannelRoute::Request { zone_from_id: 1, zone_to_id: 9, proximity_m: 0.0 }).await;
        assert_eq!((none.success, none.matched_channel_id), (false, -1));
        assert!(none.channel_path.poses.is_empty());

        assert_eq!(rec.trigger("/get_record_zone_info").await, (true, "返回test".to_string()));
        rec.stop().await;
    });
}

/// Drive the 0.5 s start-up check until the recorder has a pose: give it a
/// fresh one, let a tick pass, until the empty `/recorded_path` of a
/// finished start-up comes.
async fn initialise(clock: &Clock, manual: &ManualClock, pose: &Publisher<Odometry>, recorded: &mut Subscriber<PathMsg>) {
    for _ in 0..50 {
        pose.publish(&odom(clock, 0.0, 0.0)).unwrap();
        tokio::time::sleep(Duration::from_millis(10)).await;
        manual.advance(Duration::from_millis(500));
        if let Ok(Some(path)) = tokio::time::timeout(Duration::from_millis(200), recorded.next()).await {
            assert!(path.poses.is_empty());
            return;
        }
    }
    panic!("the recorder never initialised");
}

/// Move the robot to `(x, y)` and let the 10 Hz sampler take it, until it
/// shows up as the `n`-th pose of `/recorded_path`.
async fn sample(clock: &Clock, manual: &ManualClock, pose: &Publisher<Odometry>, recorded: &mut Subscriber<PathMsg>, (x, y): (f64, f64), n: usize) {
    for _ in 0..50 {
        pose.publish(&odom(clock, x, y)).unwrap();
        tokio::time::sleep(Duration::from_millis(10)).await;
        manual.advance(Duration::from_millis(100));
        if let Ok(Some(path)) = tokio::time::timeout(Duration::from_millis(200), recorded.next()).await {
            assert_eq!(path.poses.len(), n);
            let last = &path.poses[n - 1].pose.position;
            assert_eq!((last.x, last.y), (x, y));
            return;
        }
    }
    panic!("the sampler never took ({x}, {y})");
}

#[test]
fn a_zone_recorded_from_the_pose_topic_is_saved_listed_and_lit() {
    within("recording", |dir| async move {
        let (clock, manual) = Clock::manual(T0);
        let rec = Harness::start(clock.clone()).await;
        let lock = serve_lock(&rec.test);
        let _nav = rec.nav_active(false);
        let pose = rec.test.publisher::<Odometry>("/odometry/global", Qos::default()).unwrap();
        let mut recorded = rec.test.subscribe::<PathMsg>("/recorded_path", Qos::default()).unwrap();
        let mut zone_list = rec.latched::<MarkerArray>("/zone_list");
        let mut light = rec.latched::<StringMsg>("/mower_base/rear_light");

        initialise(&clock, &manual, &pose, &mut recorded).await;
        assert!(next(&mut zone_list).await.markers.is_empty(), "start-up publishes the empty list");

        let answer = once_nav_is_seen(|| rec.trigger("/record_zone_start")).await;
        assert_eq!(answer, (true, "成功記錄區域起始點".to_string()));
        for (i, xy) in [(2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.5)].into_iter().enumerate() {
            sample(&clock, &manual, &pose, &mut recorded, xy, i + 1).await;
        }
        assert_eq!(rec.trigger("/record_zone_end").await, (true, "成功記錄區域結束點 #1".to_string()));
        let list = next(&mut zone_list).await;
        assert_eq!(list.markers.iter().map(|m| (m.id, m.ns.as_str())).collect::<Vec<_>>(), [(1, "zones")]);
        assert_eq!(rec.retained("/zone_list"), 1);
        let zones: GetZoneList::Response = rec.call("/get_record_zone_list", GetZoneList::Request {}).await;
        assert_eq!(zones.zone_list.markers.len(), 1);
        let saved: Value = serde_json::from_slice(&std::fs::read(dir.join("zone_record/zone_list.json")).unwrap()).unwrap();
        assert_eq!(saved.as_array().map(Vec::len), Some(1));

        // The rear light breathed while recording and went off once after.
        manual.advance(Duration::from_millis(100));
        let mut effects = Vec::new();
        while effects.last().map(String::as_str) != Some("off") {
            let msg = next(&mut light).await;
            let doc: Value = serde_json::from_str(&msg.data).unwrap();
            assert_eq!(doc["source"], "path_record");
            effects.push(doc["effect"].as_str().unwrap().to_string());
            if effects.last().map(String::as_str) != Some("off") {
                manual.advance(Duration::from_millis(100));
            }
        }
        assert!(effects.len() >= 2 && effects[..effects.len() - 1].iter().all(|e| e == "recording"), "{effects:?}");
        let late = next(&mut rec.latched::<StringMsg>("/mower_base/rear_light")).await;
        assert_eq!(late.data, r#"{"effect":"off","source":"path_record"}"#);

        assert_eq!(ops(&lock), [(true, "record zone geometry".to_string()), (false, String::new())]);
        assert!(lock.lock().unwrap().iter().all(|r| r.owner.starts_with("/path_record_node:")));
        rec.stop().await;
    });
}

#[test]
fn edits_and_sites_take_the_lease_and_the_map_datum_from_the_bus() {
    within("sites", |_dir| async move {
        let (clock, _manual) = Clock::manual(T0);
        let rec = Harness::start(clock).await;
        let lock = serve_lock(&rec.test);
        let _nav = rec.nav_active(false);
        let mut zone_list = rec.latched::<MarkerArray>("/zone_list");
        let mut site_list = rec.latched::<StringMsg>("/site_list");
        next(&mut site_list).await;

        let square = [(0.0, 0.0), (3.0, 0.0), (3.0, 3.0), (0.0, 3.0)].map(|(x, y)| Point { x, y, z: 0.0 }).to_vec();
        let add = EditZone::Request { op: "add".into(), kind: "zone".into(), id: 0, points: square };
        let answer = once_nav_is_seen(|| async {
            let response: EditZone::Response = rec.call("/edit_zone", add.clone()).await;
            (response.success, format!("{}#{}", response.message, response.id))
        })
        .await;
        assert!(answer.0, "{answer:?}");
        assert!(answer.1.ends_with("#1"), "{answer:?}");
        assert_eq!(next(&mut zone_list).await.markers.len(), 1);

        // No datum from the adapter yet: a site cannot be anchored.
        let save = SiteOp::Request { op: "save".into(), name: "home".into(), new_name: String::new() };
        let response: SiteOp::Response = rec.call("/site_op", save.clone()).await;
        assert_eq!((response.success, response.message.as_str()), (false, "datum 尚未就緒（等待 /adapter/map_datum），無法儲存場地"));
        next(&mut site_list).await; // republished with the answer

        let datum = rec.test.publisher::<StringMsg>("/adapter/map_datum", Qos::latched()).unwrap();
        let doc = json!({"origin_lat": 23.5, "origin_lon": 120.5, "bearing_rad": 0.0, "source": "navsat"});
        datum.publish(&StringMsg { data: doc.to_string() }).unwrap();
        let response = loop {
            let response: SiteOp::Response = rec.call("/site_op", save.clone()).await;
            if !response.message.starts_with("datum 尚未就緒") {
                break response;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        };
        assert!(response.success, "{}", response.message);
        let sites: Value = serde_json::from_str(&response.sites_json).unwrap();
        assert_eq!(sites["active"], "home");
        assert_eq!(sites["sites"][0]["name"], "home");
        let published = loop {
            let msg = next(&mut site_list).await;
            let doc: Value = serde_json::from_str(&msg.data).unwrap();
            if doc["active"] == "home" {
                break doc;
            }
        };
        assert_eq!(published, sites);

        let list: SiteOp::Response = rec.call("/site_op", SiteOp::Request { op: "list".into(), ..Default::default() }).await;
        assert_eq!((list.success, list.message.as_str()), (true, "成功取得場地清單"));
        // The edit and every save held the lease and gave it back (a save
        // can be tried again until the datum is in); "list" takes none.
        let held: Vec<(bool, String)> = ops(&lock);
        assert_eq!(held[..2], [(true, "edit mission geometry".to_string()), (false, String::new())]);
        assert!(held.len() >= 6, "{held:?}");
        for pair in held[2..].chunks(2) {
            assert_eq!(pair, [(true, "save a named site".to_string()), (false, String::new())]);
        }
        rec.stop().await;
    });
}

/// Await `fut` while moving `manual` on by `step` every couple of
/// milliseconds: the module's timeouts run on it (as do its 10 Hz timers,
/// which is why this cannot jump to one deadline).
async fn driving<F: Future>(manual: &ManualClock, step: Duration, fut: F) -> F::Output {
    let mut fut = std::pin::pin!(fut);
    loop {
        tokio::select! {
            out = &mut fut => return out,
            _ = tokio::time::sleep(Duration::from_millis(2)) => manual.advance(step),
        }
    }
}

#[test]
fn recording_fails_closed_on_unknown_or_active_navigation_and_a_missing_lease_service() {
    within("closed", |_dir| async move {
        let (clock, manual) = Clock::manual(T0);
        let rec = Harness::start(clock.clone()).await;
        let answer = rec.trigger("/record_zone_start").await;
        assert_eq!(answer, (false, "Cannot record zone geometry: navigation state is not available".to_string()));

        let nav = rec.nav_active(true);
        let answer = once_nav_is_seen(|| rec.trigger("/risk_zone_start")).await;
        assert_eq!(answer, (false, "Cannot record risk geometry: navigation is active".to_string()));

        // Idle, but nobody serves the lease: the guard waits 1 s for it.
        nav.publish(&Bool { data: false }).unwrap();
        let start = clock.mono();
        let answer = loop {
            let answer = driving(&manual, Duration::from_millis(50), rec.trigger("/channel_record_start")).await;
            if !answer.1.contains("navigation is active") {
                break answer;
            }
        };
        assert_eq!(answer, (false, "Mission operation guard is unavailable".to_string()));
        assert!(clock.mono().saturating_duration_since(start) >= Duration::from_secs(1));
        // Nothing was recorded, so a cancel has nothing to give back.
        assert_eq!(rec.trigger("/record_cancel").await, (true, "目前沒有進行中的記錄".to_string()));
        rec.stop().await;
    });
}
