//! map_manage on the bus (step E2): the module's real `run`, with its inputs
//! published and its services called from another scope of the same `Bus`,
//! as the other modules of `mower_rsd` do.
//!
//! - **Its r2r node** is still created (parameters, `/rosout`, the
//!   `/map_manage/*` parameter services), on ROS domain 83 so that no other
//!   DDS user on the host sees it. One test at a time: the context is
//!   process-wide and the node and its DDS services have fixed names.
//! - **Time** is a manual clock wherever a timeout is under test, so that 5
//!   or 30 s pass at once; header stamps then come from it too.

use std::future::Future;
use std::panic::AssertUnwindSafe;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};
use std::time::Duration;

use futures::{FutureExt, StreamExt};
use mower_bus::{Bus, Clock, ManualClock, Message, Qos, Scope, Subscriber, Time};
use mower_rs_common::{ModuleCtx, ModuleResult, Shutdown};
use r2r::geometry_msgs::msg::Point;
use r2r::mower_interface::srv::{ChennalPathList, GetZoneList, ImportImageMask, MissionOperationLock, ZoneMapList};
use r2r::nav_msgs::msg::OccupancyGrid;
use r2r::rcl_interfaces::msg::{Parameter, ParameterValue};
use r2r::rcl_interfaces::srv::{GetParameters, SetParameters};
use r2r::std_msgs::msg::Bool;
use r2r::std_srvs::srv::Trigger;
use r2r::visualization_msgs::msg::{Marker, MarkerArray};
use r2r::QosProfile;

/// For anything that waits in real time. A pass never waits this long.
const WAIT: Duration = Duration::from_secs(20);

/// 2027-01-15 08:00:00 UTC, the manual clocks' wall time at start.
const T0: Time = Time::from_ns(1_800_000_000_000_000_000);

fn context() -> r2r::Context {
    static CONTEXT: OnceLock<r2r::Context> = OnceLock::new();
    CONTEXT
        .get_or_init(|| {
            // rcl reads it when the context is created, and only then.
            std::env::set_var("ROS_DOMAIN_ID", "83");
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

/// The module under test and the scope the test talks to it from.
struct Harness {
    bus: Bus,
    test: Scope,
    scope: Scope,
    shutdown: Shutdown,
    task: tokio::task::JoinHandle<ModuleResult>,
}

impl Harness {
    /// Start map_manage on a fresh bus, and wait until it is up: its last
    /// step publishes the start-up maps, after every endpoint exists.
    async fn start(clock: Clock) -> Harness {
        let bus = Bus::new();
        let scope = bus.scope("map_manage");
        let test = bus.scope("test");
        let shutdown = Shutdown::new();
        let m = ModuleCtx::new("map_manage", shutdown.clone()).with_bus(scope.clone()).with_clock(clock);
        let mut up = test.subscribe::<OccupancyGrid>("/map_grid", Qos::latched()).unwrap();
        let task = tokio::spawn(mower_map::run(context(), m));
        next(&mut up).await;
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
        call(&self.test, name, request).await
    }

    fn nav_active(&self, active: bool) -> mower_bus::Publisher<Bool> {
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

async fn call<Req: Message, Resp: Message>(scope: &Scope, name: &str, request: Req) -> Resp {
    let client = scope.client::<Req, Resp>(name).unwrap();
    tokio::time::timeout(WAIT, client.wait_for_service()).await.expect("the service in time").unwrap();
    tokio::time::timeout(WAIT, client.request(&request).unwrap()).await.expect("an answer in time").unwrap()
}

/// Await `fut` with the clock stopped, except that once the module waits on
/// the deadline `d` from now, and on nothing sooner, the clock jumps there.
/// So exactly that one of its timeouts passes, at once, and no other can.
async fn jump<F: Future>(manual: &ManualClock, d: Duration, fut: F) -> F::Output {
    let deadline = manual.clock().mono() + d;
    let mut fut = std::pin::pin!(fut);
    loop {
        tokio::select! {
            out = &mut fut => return out,
            _ = tokio::time::sleep(Duration::from_millis(1)) => {
                if manual.next_deadline() == Some(deadline) {
                    manual.advance_to(deadline);
                }
            }
        }
    }
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

fn trigger(response: Trigger::Response) -> (bool, String) {
    (response.success, response.message)
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

fn marker(id: i32, points: &[(f64, f64)]) -> Marker {
    Marker {
        id,
        points: points.iter().map(|&(x, y)| Point { x, y, z: 0.0 }).collect(),
        header: r2r::std_msgs::msg::Header { frame_id: "map".into(), ..Default::default() },
        ..Default::default()
    }
}

/// path_record_node's three list services, answering with one zone (a 4 m
/// square), one risk zone inside it and one channel across it.
fn serve_recorder(test: &Scope) {
    let mut zones = test.service::<GetZoneList::Request, GetZoneList::Response>("/get_record_zone_list").unwrap();
    let mut risks = test.service::<GetZoneList::Request, GetZoneList::Response>("/get_risk_zone_list").unwrap();
    let mut channels = test.service::<ChennalPathList::Request, ChennalPathList::Response>("/get_chennal_path_list").unwrap();
    tokio::spawn(async move {
        loop {
            tokio::select! {
                Some(req) = zones.next() => {
                    let zone_list = MarkerArray { markers: vec![marker(1, &[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)])] };
                    let _ = req.respond(GetZoneList::Response { success: true, message: "zones".into(), zone_list });
                }
                Some(req) = risks.next() => {
                    let zone_list = MarkerArray { markers: vec![marker(1, &[(1.0, 1.0), (1.5, 1.0), (1.5, 1.5), (1.0, 1.5)])] };
                    let _ = req.respond(GetZoneList::Response { success: true, message: "risks".into(), zone_list });
                }
                Some(req) = channels.next() => {
                    let chennal_path_array = MarkerArray { markers: vec![marker(1, &[(0.5, 3.0), (3.5, 3.0)])] };
                    let _ = req.respond(ChennalPathList::Response { success: true, message: "channels".into(), chennal_path_array });
                }
                else => break,
            }
        }
    });
}

#[test]
fn the_start_up_maps_are_latched_and_stamped_on_the_module_s_clock() {
    within(async {
        let (clock, _manual) = Clock::manual(T0);
        let map = Harness::start(clock).await;
        // Late joiners get the fully occupied start-up extent, once.
        for topic in ["/map_grid", "/map_grid_global"] {
            let mut late = map.latched::<OccupancyGrid>(topic);
            let grid = next(&mut late).await;
            assert_eq!((grid.info.width, grid.info.height, grid.info.resolution), (400, 400, 0.1f32), "{topic}");
            assert_eq!((grid.info.origin.position.x, grid.info.origin.position.y), (-20.0, -20.0));
            assert!(grid.data.iter().all(|&v| v == 100), "{topic} held occupied");
            assert_eq!(grid.header.frame_id, "map");
            assert_eq!((grid.header.stamp.sec, grid.header.stamp.nanosec), T0.to_parts(), "{topic} stamped on the clock");
            assert_eq!(late.try_recv().err(), Some(mower_bus::RecvError::Empty));
            assert_eq!(map.retained(topic), 1, "{topic}: keep last 1, transient local");
        }
        // The other grids have their latched writers, and nothing to replay yet.
        let topics = map.bus.topics();
        for topic in ["/free_space", "/free_space_inflated", "/risk_map", "/risk_map_inflated", "/chennal_map", "/chennal_map_inflated"] {
            let info = topics.iter().find(|t| t.name == topic).unwrap_or_else(|| panic!("{topic} on the bus"));
            assert_eq!(info.publishers, ["map_manage"], "{topic}");
            assert_eq!(info.ros_type.as_deref(), Some("nav_msgs/msg/OccupancyGrid"));
            assert_eq!(info.retained, 0, "{topic}");
        }
        let services = map.bus.services();
        for name in [
            "/create_risk_map",
            "/create_free_space",
            "/create_chennal_map",
            "/restore_free_space_coverage",
            "/import_image_mask",
            "/get_zone_map_list_srv",
        ] {
            let info = services.iter().find(|s| s.name == name).unwrap_or_else(|| panic!("{name} on the bus"));
            assert_eq!(info.server.as_deref(), Some("map_manage"), "{name}");
        }
        // The parameter services stay on DDS, not on the bus.
        assert!(!services.iter().any(|s| s.name.starts_with("/map_manage/")));
        map.stop().await;
    });
}

#[test]
fn free_space_risk_and_channel_maps_come_from_the_recorder_s_services() {
    within(async {
        let (clock, _manual) = Clock::manual(T0);
        let map = Harness::start(clock).await;
        let lock = serve_lock(&map.test);
        serve_recorder(&map.test);
        let _nav = map.nav_active(false);
        let mut map_grid = map.latched::<OccupancyGrid>("/map_grid");
        next(&mut map_grid).await; // the start-up map

        let answer = once_nav_is_seen(|| async { trigger(map.call("/create_free_space", Trigger::Request {}).await) }).await;
        assert_eq!(answer, (true, "成功創建自由空間，包含 1 個區域".to_string()));
        let free = next(&mut map.latched::<OccupancyGrid>("/free_space")).await;
        // 4 m plus a 1 m margin on each side, at 5 cm
        assert_eq!((free.info.width, free.info.height, free.info.resolution), (120, 120, 0.05f32));
        assert_eq!((free.header.stamp.sec, free.header.stamp.nanosec), T0.to_parts());
        let inflated = next(&mut map.latched::<OccupancyGrid>("/free_space_inflated")).await;
        let free_cells = |g: &OccupancyGrid| g.data.iter().filter(|&&v| v == 0).count();
        assert!(free_cells(&inflated) > 0 && free_cells(&inflated) < free_cells(&free));
        // Rebuilt, but held occupied until a risk map exists.
        let grid = next(&mut map_grid).await;
        assert_eq!(grid.info.width, 120);
        assert!(grid.data.iter().all(|&v| v == 100));

        let zones: ZoneMapList::Response = map.call("/get_zone_map_list_srv", ZoneMapList::Request {}).await;
        assert_eq!(zones.zone_map_list.iter().map(|z| z.zone_id).collect::<Vec<_>>(), [1]);
        assert!(!zones.zone_map_list[0].mask_map_inflated.data.is_empty());

        let answer = trigger(map.call("/create_risk_map", Trigger::Request {}).await);
        assert_eq!(answer, (true, "成功創建風險地圖，包含 1 個風險區域".to_string()));
        next(&mut map.latched::<OccupancyGrid>("/risk_map")).await;
        next(&mut map.latched::<OccupancyGrid>("/risk_map_inflated")).await;
        let grid = next(&mut map_grid).await;
        assert!(grid.data.contains(&0), "/map_grid opens once the risk map exists");

        let answer = trigger(map.call("/create_chennal_map", Trigger::Request {}).await);
        assert_eq!(answer, (true, "成功創建通道地圖，包含 1 條通道".to_string()));
        next(&mut map.latched::<OccupancyGrid>("/chennal_map")).await;
        next(&mut map.latched::<OccupancyGrid>("/chennal_map_inflated")).await;
        next(&mut map_grid).await;
        let mut global = map.latched::<OccupancyGrid>("/map_grid_global");
        let global = next(&mut global).await;
        assert_eq!(global.info.width, 120);
        assert_eq!(map.retained("/map_grid_global"), 1);

        // Each mutation held the nav server's lease, under the node's name.
        let lock = lock.lock().unwrap().clone();
        let ops: Vec<(bool, &str)> = lock.iter().map(|r| (r.acquire, r.operation.as_str())).collect();
        assert_eq!(ops, [
            (true, "rebuild free space"),
            (false, ""),
            (true, "rebuild the risk map"),
            (false, ""),
            (true, "rebuild the channel map"),
            (false, ""),
        ]);
        assert!(lock.iter().all(|r| r.owner.starts_with("/map_manage:") && r.owner == lock[0].owner));
        map.stop().await;
    });
}

#[test]
fn mutations_fail_closed_on_unknown_or_active_navigation_and_a_missing_lease_service() {
    within(async {
        let (clock, manual) = Clock::manual(T0);
        let map = Harness::start(clock.clone()).await;
        let answer = trigger(map.call("/create_free_space", Trigger::Request {}).await);
        assert_eq!(answer, (false, "Cannot rebuild free space: navigation state is not available".to_string()));

        let nav = map.nav_active(true);
        let answer = once_nav_is_seen(|| async { trigger(map.call("/restore_free_space_coverage", Trigger::Request {}).await) }).await;
        assert_eq!(answer, (false, "Cannot restore free-space coverage: navigation is active".to_string()));

        // Idle, but nobody serves the lease: the guard waits 1 s for it.
        nav.publish(&Bool { data: false }).unwrap();
        let start = clock.mono();
        let answer = loop {
            let answer = trigger(jump(&manual, Duration::from_secs(1), map.call("/create_risk_map", Trigger::Request {})).await);
            if !answer.1.contains("navigation is active") {
                break answer;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        };
        assert_eq!(answer, (false, "Mission operation guard is unavailable".to_string()));
        assert_eq!(clock.mono().saturating_duration_since(start), Duration::from_secs(1));
        map.stop().await;
    });
}

#[test]
fn a_missing_silent_or_failing_recorder_service_ends_the_call_as_before() {
    within(async {
        let (clock, manual) = Clock::manual(T0);
        let map = Harness::start(clock.clone()).await;
        let lock = serve_lock(&map.test);
        let _nav = map.nav_active(false);
        let create = || async { trigger(map.call("/create_free_space", Trigger::Request {}).await) };

        // No server: the wait for one gives up after 5 s.
        let start = clock.mono();
        let answer = once_nav_is_seen(|| jump(&manual, Duration::from_secs(5), create())).await;
        assert_eq!(answer, (false, "記錄區域列表服務不可用".to_string()));
        assert_eq!(clock.mono().saturating_duration_since(start), Duration::from_secs(5));
        // the lease is still taken and given back
        assert_eq!(lock.lock().unwrap().iter().map(|r| r.acquire).collect::<Vec<_>>(), [true, false]);

        // A server that holds the request: the call times out after 30 s.
        let mut silent = map.test.service::<GetZoneList::Request, GetZoneList::Response>("/get_record_zone_list").unwrap();
        let held = tokio::spawn(async move {
            let req = silent.next().await;
            std::future::pending::<()>().await;
            drop(req);
        });
        let start = clock.mono();
        let answer = jump(&manual, Duration::from_secs(30), create()).await;
        assert_eq!(answer, (false, "獲取記錄區域列表逾時".to_string()));
        assert_eq!(clock.mono().saturating_duration_since(start), Duration::from_secs(30));
        held.abort();
        let _ = held.await;

        // A server that drops the request: the bus says NoResponse at once,
        // which ends the call the way the timeout did.
        let mut dropping = map.test.service::<GetZoneList::Request, GetZoneList::Response>("/get_record_zone_list").unwrap();
        tokio::spawn(async move {
            while let Some(req) = dropping.next().await {
                drop(req);
            }
        });
        let start = clock.mono();
        assert_eq!(create().await, (false, "獲取記錄區域列表逾時".to_string()));
        assert_eq!(clock.mono(), start, "no clock time passes");
        assert_eq!(lock.lock().unwrap().len(), 6, "three leases, each given back");
        map.stop().await;
    });
}

#[test]
fn import_image_mask_is_served_on_the_bus_under_the_lease() {
    within(async {
        let (clock, _manual) = Clock::manual(T0);
        let map = Harness::start(clock).await;
        let lock = serve_lock(&map.test);
        let _nav = map.nav_active(false);
        let request = ImportImageMask::Request { zone_id: 7, mask_encoding: "png".into(), ..Default::default() };
        // (A refusal for want of the nav flag answers zone 0.)
        let answer = loop {
            let answer: ImportImageMask::Response = map.call("/import_image_mask", request.clone()).await;
            if !answer.message.contains("navigation state is not available") {
                break answer;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        };
        assert_eq!((answer.success, answer.message.as_str(), answer.zone_id), (false, "mask_encoding must be base64_u8_row_major", 7));
        let ops: Vec<(bool, String)> = lock.lock().unwrap().iter().map(|r| (r.acquire, r.operation.clone())).collect();
        assert_eq!(ops, [(true, "import an image mission".to_string()), (false, String::new())]);
        map.stop().await;
    });
}

/// A node of the process that is not the module, spun on a thread of its
/// own: a DDS client of the parameter services, as `ros2 param` is.
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

    async fn call<S: 'static + r2r::WrappedServiceTypeSupport>(&self, name: &str, request: S::Request) -> S::Response {
        let (client, ready) = {
            let mut node = self.node.lock().unwrap();
            let client = node.create_client::<S>(name, QosProfile::services_default()).unwrap();
            let ready = r2r::Node::is_available(&client).unwrap();
            (client, ready)
        };
        tokio::time::timeout(WAIT, ready).await.expect("the DDS service in time").unwrap();
        tokio::time::timeout(WAIT, client.request(&request).unwrap()).await.expect("a DDS answer in time").unwrap()
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

#[test]
fn the_parameter_services_stay_on_dds_and_a_new_radius_refreshes_the_bus_maps() {
    within(async {
        let (clock, _manual) = Clock::manual(T0);
        let map = Harness::start(clock).await;
        let _lock = serve_lock(&map.test);
        serve_recorder(&map.test);
        let _nav = map.nav_active(false);
        let answer = once_nav_is_seen(|| async { trigger(map.call("/create_free_space", Trigger::Request {}).await) }).await;
        assert!(answer.0, "{answer:?}");
        let mut inflated = map.latched::<OccupancyGrid>("/free_space_inflated");
        let before = next(&mut inflated).await;

        let peer = Peer::new("map_bus_test_peer");
        let names = vec!["inflate_radius_m".to_string()];
        let got = peer.call::<GetParameters::Service>("/map_manage/get_parameters", GetParameters::Request { names }).await;
        assert_eq!(got.values.iter().map(|v| (v.type_, v.double_value)).collect::<Vec<_>>(), [(3, 0.75)]);

        let value = ParameterValue { type_: 3, double_value: 1.0, ..Default::default() };
        let parameters = vec![Parameter { name: "inflate_radius_m".into(), value }];
        let set = peer.call::<SetParameters::Service>("/map_manage/set_parameters", SetParameters::Request { parameters }).await;
        assert_eq!(set.results.iter().map(|r| r.successful).collect::<Vec<_>>(), [true], "{:?}", set.results);
        // The refresh went out on the bus: a wider erosion leaves less free.
        let after = next(&mut inflated).await;
        let free_cells = |g: &OccupancyGrid| g.data.iter().filter(|&&v| v == 0).count();
        assert!(free_cells(&after) < free_cells(&before));
        drop(peer);
        map.stop().await;
    });
}
