//! boustrophedon_coverage on the bus (step E2): the module's real `run`, and
//! what it talks to played from other scopes of the same `Bus`: nav's action
//! server with its dispatch, status, progress and lock services and its
//! activity flag, map_manage's zone maps and risk map, record's channel
//! routes.
//!
//! - **Time** is a manual clock, so that coverage's bounds (the 2 s
//!   watchdog and retry, the 3 s acceptance, the 5 s cancel request, the leg
//!   timeout) pass at once; the scripted nav keeps its own delays on it too.
//!   `pass` moves it 20 ms at a time and lets the tasks run in between.
//! - **The scenarios** are coverage_cancel_check.py's, run against nav on the
//!   bus instead of an rclpy server, and what the bus adds: a definite
//!   rejection, no server at all, and nav's module ending mid-leg.
//! - **Its r2r node** is still made (parameters, the six parameter services,
//!   the log), on ROS domain 80. One test at a time: the context is
//!   process-wide and the node has a fixed name.

use std::future::Future;
use std::panic::AssertUnwindSafe;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};
use std::time::{Duration, Instant, SystemTime};

use futures::{FutureExt, StreamExt};
use mower_bus::{Bus, Clock, ManualClock, Message, Mono, Qos, Scope, Subscriber, Time};
use mower_rs_common::{ModuleCtx, ModuleResult, Shutdown};
use r2r::geometry_msgs::msg::PoseStamped;
use r2r::mower_interface::action::Waypoint;
use r2r::mower_interface::msg::ZoneMap;
use r2r::mower_interface::srv::{
    CancelNavigationDispatch, ChannelRoute, ConfirmNavigationDispatch, GetCoverageProgress, MissionOperationLock,
    ResumeCoverage, ZoneExecPath, ZoneMapList, ZoneSequence,
};
use r2r::nav_msgs::msg::{OccupancyGrid, Path};
use r2r::rcl_interfaces::srv::GetParameters;
use r2r::std_msgs::msg::{Bool, String as Text};
use r2r::std_srvs::srv::Trigger;
use r2r::QosProfile;
use serde_json::Value;

/// For anything that waits in real time. A pass never waits this long.
const WAIT: Duration = Duration::from_secs(20);

/// The bus side of `Waypoint::Action`.
type WaypointAction = (Waypoint::Goal, Waypoint::Result, Waypoint::Feedback);

fn context() -> r2r::Context {
    static CONTEXT: OnceLock<r2r::Context> = OnceLock::new();
    CONTEXT
        .get_or_init(|| {
            // rcl reads it when the context is created, and only then.
            std::env::set_var("ROS_DOMAIN_ID", "80");
            r2r::Context::create().expect("an rcl context (source /opt/ros/jazzy/setup.bash)")
        })
        .clone()
}

fn serial() -> MutexGuard<'static, ()> {
    static SERIAL: Mutex<()> = Mutex::new(());
    SERIAL.lock().unwrap_or_else(|e| e.into_inner())
}

/// Run one test on a runtime of its own, within a bound only a hang reaches
/// (see mower_map's tests/bus.rs for why a failure waits a moment).
fn within(test: impl Future<Output = ()>) {
    let _serial = serial();
    let runtime = tokio::runtime::Builder::new_multi_thread().worker_threads(4).enable_all().build().unwrap();
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

async fn wait<F: Future>(what: &str, fut: F) -> F::Output {
    match tokio::time::timeout(WAIT, fut).await {
        Ok(output) => output,
        Err(_) => panic!("timed out waiting for {what}"),
    }
}

async fn next<T: Message>(sub: &mut Subscriber<T>) -> Arc<T> {
    wait("a message", sub.next()).await.expect("the bus is open")
}

async fn call<Req: Message, Resp: Message>(scope: &Scope, name: &str, request: Req) -> Resp {
    let client = scope.client::<Req, Resp>(name).unwrap();
    wait("the service", client.wait_for_service()).await.unwrap();
    wait("an answer", client.request(&request).unwrap()).await.unwrap()
}

/// Let `d` pass on the manual clock, 20 ms at a time, and let the tasks run
/// after each step.
async fn pass(manual: &ManualClock, d: Duration) {
    let end = manual.clock().mono() + d;
    while manual.clock().mono() < end {
        let step = end.saturating_duration_since(manual.clock().mono()).min(Duration::from_millis(20));
        manual.advance(step);
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
}

/// Seconds from `t0` on the manual clock.
fn since(t0: Mono, t: Mono) -> f64 {
    t.saturating_duration_since(t0).as_secs_f64()
}

// ---------------------------------------------------------------------------
// What coverage talks to
// ---------------------------------------------------------------------------

/// How the scripted nav answers.
#[derive(Clone, Copy, PartialEq, Debug)]
enum CancelDispatch {
    /// success and terminal_confirmed.
    Terminal,
    /// success, but nothing proven: what a restarted nav says.
    Unconfirmed,
    /// Answers only after 12 s, past coverage's 5 s.
    Hang,
}

#[derive(Clone)]
struct Script {
    accept_after: Duration,
    reject: bool,
    confirm: bool,
    cancel_dispatch: CancelDispatch,
    /// A goal ends CANCELED once a cancel request is accepted.
    end_on_cancel: bool,
    /// A goal ends SUCCEEDED this long after its acceptance; `None`: it runs
    /// until something ends it.
    succeed_after: Option<Duration>,
    block_reason: &'static str,
    /// `(zone, completed, total)` of a resumable checkpoint.
    checkpoint: Option<(i32, i32, i32)>,
}

impl Default for Script {
    fn default() -> Self {
        Script {
            accept_after: Duration::ZERO,
            reject: false,
            confirm: true,
            cancel_dispatch: CancelDispatch::Terminal,
            end_on_cancel: false,
            succeed_after: None,
            block_reason: "",
            checkpoint: None,
        }
    }
}

/// What the scripted nav saw, on the manual clock.
#[derive(Default)]
struct Seen {
    goals: Vec<(Mono, Waypoint::Goal)>,
    action_cancels: Vec<Mono>,
    dispatch_cancels: Vec<(Mono, String)>,
    confirms: Vec<String>,
}

/// nav_action_server as coverage_cancel_check.py's FakeNavigation plays it,
/// on the bus: the action server and the four services coverage calls, the
/// mission lock and the latched activity flag.
struct FakeNav {
    scope: Scope,
    script: Arc<Mutex<Script>>,
    seen: Arc<Mutex<Seen>>,
    /// Ends every running goal ABORTED.
    end_all: Arc<tokio::sync::Notify>,
    tasks: Vec<tokio::task::JoinHandle<()>>,
}

impl FakeNav {
    fn start(bus: &Bus, clock: &Clock, script: Script) -> FakeNav {
        let scope = bus.scope("nav_action_server");
        let script = Arc::new(Mutex::new(script));
        let seen: Arc<Mutex<Seen>> = Arc::default();
        let end_all = Arc::new(tokio::sync::Notify::new());
        let mut tasks = Vec::new();

        let active = scope.publisher::<Bool>("/nav_operation_active", Qos::latched()).unwrap();
        active.publish(&Bool { data: false }).unwrap();
        let mut lock = scope.service::<MissionOperationLock::Request, MissionOperationLock::Response>("/mission_operation_lock").unwrap();
        tasks.push(tokio::spawn(async move {
            let _active = active;
            while let Some(request) = lock.next().await {
                let _ = request.respond(MissionOperationLock::Response { success: true, message: "ok".into() });
            }
        }));
        let mut status = scope.service::<Trigger::Request, Trigger::Response>("/check_nav_status").unwrap();
        let s = script.clone();
        tasks.push(tokio::spawn(async move {
            while let Some(request) = status.next().await {
                let reason = s.lock().unwrap().block_reason;
                let message = serde_json::json!({ "state": "idle", "block_reason": reason }).to_string();
                let _ = request.respond(Trigger::Response { success: true, message });
            }
        }));
        let mut progress = scope.service::<GetCoverageProgress::Request, GetCoverageProgress::Response>("/coverage_progress_status").unwrap();
        let s = script.clone();
        tasks.push(tokio::spawn(async move {
            while let Some(request) = progress.next().await {
                let mut response = GetCoverageProgress::Response { success: true, ..Default::default() };
                response.checkpoint.zone_id = -1;
                if let Some((zone, done, total)) = s.lock().unwrap().checkpoint {
                    response.checkpoint.checkpoint_available = true;
                    (response.checkpoint.zone_id, response.checkpoint.completed_segments, response.checkpoint.total_segments) = (zone, done, total);
                }
                let _ = request.respond(response);
            }
        }));
        let mut confirm = scope.service::<ConfirmNavigationDispatch::Request, ConfirmNavigationDispatch::Response>("/confirm_navigation_dispatch").unwrap();
        let (s, log) = (script.clone(), seen.clone());
        tasks.push(tokio::spawn(async move {
            while let Some(request) = confirm.next().await {
                log.lock().unwrap().confirms.push(request.message.dispatch_id.clone());
                let success = s.lock().unwrap().confirm;
                let message = if success { "confirmed" } else { "scripted confirmation failure" };
                let _ = request.respond(ConfirmNavigationDispatch::Response { success, message: message.into() });
            }
        }));
        let mut cancel = scope.service::<CancelNavigationDispatch::Request, CancelNavigationDispatch::Response>("/cancel_navigation_dispatch").unwrap();
        let (s, log, c) = (script.clone(), seen.clone(), clock.clone());
        tasks.push(tokio::spawn(async move {
            while let Some(request) = cancel.next().await {
                log.lock().unwrap().dispatch_cancels.push((c.mono(), request.message.dispatch_id.clone()));
                let mode = s.lock().unwrap().cancel_dispatch;
                let c = c.clone();
                tokio::spawn(async move {
                    let response = match mode {
                        CancelDispatch::Terminal => CancelNavigationDispatch::Response { success: true, terminal_confirmed: true, message: "terminal".into() },
                        CancelDispatch::Unconfirmed => CancelNavigationDispatch::Response {
                            success: true,
                            terminal_confirmed: false,
                            message: "Matching dispatch is not active, but terminal state is not confirmed".into(),
                        },
                        CancelDispatch::Hang => {
                            c.sleep(Duration::from_secs(12)).await;
                            CancelNavigationDispatch::Response { success: false, terminal_confirmed: false, message: "late answer".into() }
                        }
                    };
                    let _ = request.respond(response);
                });
            }
        }));
        let mut goals = scope.action_server::<WaypointAction>("/nav_action_follow_path").unwrap();
        let (s, log, c, end) = (script.clone(), seen.clone(), clock.clone(), end_all.clone());
        tasks.push(tokio::spawn(async move {
            while let Some(request) = goals.next().await {
                let script = s.lock().unwrap().clone();
                if script.reject {
                    let _ = request.reject();
                    continue;
                }
                let (log, c, end) = (log.clone(), c.clone(), end.clone());
                tokio::spawn(async move {
                    c.sleep(script.accept_after).await;
                    let at = c.mono();
                    log.lock().unwrap().goals.push((at, request.goal.clone()));
                    let Ok((mut goal, mut cancels)) = request.accept() else { return };
                    let succeed = async {
                        match script.succeed_after {
                            Some(d) => c.sleep(d).await,
                            None => std::future::pending().await,
                        }
                    };
                    let canceled = async {
                        while let Some(cancel) = cancels.next().await {
                            log.lock().unwrap().action_cancels.push(c.mono());
                            cancel.accept();
                            if script.end_on_cancel {
                                return;
                            }
                        }
                        std::future::pending().await
                    };
                    tokio::select! {
                        _ = succeed => { let _ = goal.succeed(Waypoint::Result { success: true }); }
                        _ = canceled => { let _ = goal.cancel(Waypoint::Result { success: false }); }
                        _ = end.notified() => { let _ = goal.abort(Waypoint::Result { success: false }); }
                    }
                });
            }
        }));
        FakeNav { scope, script, seen, end_all, tasks }
    }

    fn script(&self, change: impl FnOnce(&mut Script)) {
        change(&mut self.script.lock().unwrap());
    }

    fn seen<R>(&self, read: impl FnOnce(&Seen) -> R) -> R {
        read(&self.seen.lock().unwrap())
    }

    /// The correlated cancels since `t0`, in seconds from it.
    fn dispatch_cancels_since(&self, t0: Mono) -> Vec<f64> {
        self.seen(|s| s.dispatch_cancels.iter().filter(|(t, _)| *t >= t0).map(|(t, _)| since(t0, *t)).collect())
    }

    /// The module ends: its scope closes, and everything on it with it.
    fn stop(self) {
        self.scope.close();
        for task in &self.tasks {
            task.abort();
        }
    }
}

const RES: f32 = 0.05;
const CELLS: u32 = 80;

/// coverage_cancel_check.py's 4 x 4 m zone, free inside `inflate` + 0.3 m.
fn grid(inflate: f64) -> OccupancyGrid {
    let mut g = OccupancyGrid::default();
    g.header.frame_id = "map".into();
    g.info.resolution = RES;
    (g.info.width, g.info.height) = (CELLS, CELLS);
    g.info.origin.orientation.w = 1.0;
    let (lo, hi) = (0.3 + inflate, CELLS as f64 * RES as f64 - 0.3 - inflate);
    let inside = |i: u32| {
        let at = (i as f64 + 0.5) * RES as f64;
        lo < at && at < hi
    };
    g.data = (0..CELLS).flat_map(|r| (0..CELLS).map(move |c| if inside(r) && inside(c) { 0 } else { 100 })).collect();
    g
}

/// map_manage's and path_record_node's halves: zones 1 and 2 (the same 4 x 4
/// m grid), no risk, and a channel route between any two zones.
struct World {
    _scope: Scope,
    _risk: mower_bus::Publisher<OccupancyGrid>,
    routes: Arc<Mutex<Vec<(i32, i32)>>>,
    tasks: Vec<tokio::task::JoinHandle<()>>,
}

impl World {
    fn start(bus: &Bus) -> World {
        let scope = bus.scope("map_manage");
        let mut free = grid(0.0);
        free.data.iter_mut().for_each(|v| *v = 0);
        let risk = scope.publisher::<OccupancyGrid>("/risk_map_inflated", Qos::latched()).unwrap();
        risk.publish(&free).unwrap();
        let mut zones = scope.service::<ZoneMapList::Request, ZoneMapList::Response>("/get_zone_map_list_srv").unwrap();
        let zone = |zone_id| ZoneMap { zone_id, mask_map: grid(0.0), mask_map_inflated: grid(0.75), ..Default::default() };
        let list = vec![zone(1), zone(2)];
        let mut tasks = vec![tokio::spawn(async move {
            while let Some(request) = zones.next().await {
                let _ = request.respond(ZoneMapList::Response { zone_map_list: list.clone() });
            }
        })];
        let mut route = scope.service::<ChannelRoute::Request, ChannelRoute::Response>("/get_channel_route").unwrap();
        let routes: Arc<Mutex<Vec<(i32, i32)>>> = Arc::default();
        let asked = routes.clone();
        tasks.push(tokio::spawn(async move {
            while let Some(request) = route.next().await {
                asked.lock().unwrap().push((request.message.zone_from_id, request.message.zone_to_id));
                let mut path = Path::default();
                path.header.frame_id = "map".into();
                path.poses = [(3.5, 2.0), (4.5, 2.0), (5.5, 2.0)]
                    .iter()
                    .map(|&(x, y)| {
                        let mut p = PoseStamped::default();
                        p.header.frame_id = "map".into();
                        (p.pose.position.x, p.pose.position.y, p.pose.orientation.w) = (x, y, 1.0);
                        p
                    })
                    .collect();
                let _ = request.respond(ChannelRoute::Response { success: true, message: String::new(), channel_path: path, matched_channel_id: 4 });
            }
        }));
        World { _scope: scope, _risk: risk, routes, tasks }
    }
}

impl Drop for World {
    fn drop(&mut self) {
        for task in &self.tasks {
            task.abort();
        }
    }
}

/// One start of boustrophedon_coverage on a manual clock, with its zone
/// paths generated.
struct Coverage {
    test: Scope,
    scope: Scope,
    m: ModuleCtx,
    shutdown: Shutdown,
    task: Option<tokio::task::JoinHandle<ModuleResult>>,
    manual: ManualClock,
}

impl Coverage {
    async fn start(bus: &Bus, clock: Clock, manual: ManualClock) -> Coverage {
        let scope = bus.scope("boustrophedon_coverage");
        let shutdown = Shutdown::new();
        let m = ModuleCtx::new("boustrophedon_coverage", shutdown.clone()).with_bus(scope.clone()).with_clock(clock);
        let task = tokio::spawn(mower_coverage::run(context(), m.clone()));
        let test = bus.scope("test");
        // It refuses until nav's latched flag and the risk map have reached
        // it, on tasks of its own.
        let deadline = Instant::now() + WAIT;
        loop {
            let answer: Trigger::Response = call(&test, "/generate_coverage_path", Trigger::Request {}).await;
            if answer.success {
                break;
            }
            let early = answer.message.contains("navigation state is not available") || answer.message.contains("risk_map_inflated");
            assert!(early && Instant::now() < deadline, "{}", answer.message);
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        Coverage { test, scope, m, shutdown, task: Some(task), manual }
    }

    /// `/zone_exec_path` for zone 1, answered while the clock runs on.
    fn exec_zone(&self) -> tokio::task::JoinHandle<ZoneExecPath::Response> {
        let test = self.test.clone();
        tokio::spawn(async move { call(&test, "/zone_exec_path", ZoneExecPath::Request { zone_id: 1 }).await })
    }

    async fn exec_zone_now(&self) -> ZoneExecPath::Response {
        let answer = self.exec_zone();
        let manual = self.manual.clone();
        let mut answer = std::pin::pin!(answer);
        loop {
            tokio::select! {
                done = &mut answer => return done.unwrap(),
                _ = pass(&manual, Duration::from_millis(100)) => {}
            }
        }
    }

    async fn run_sequence(&self, zone_ids: &[i32]) -> ZoneSequence::Response {
        call(&self.test, "/run_zone_sequence", ZoneSequence::Request { zone_ids: zone_ids.to_vec(), channel_proximity_m: 0.0 }).await
    }

    /// What mower_rsd does: stop the module, then close its scope.
    async fn stop(mut self) {
        self.shutdown.trigger();
        let task = self.task.take().expect("running");
        wait("coverage to stop", task).await.expect("coverage panicked").expect("coverage failed");
        self.scope.close();
    }
}

impl Drop for Coverage {
    fn drop(&mut self) {
        self.shutdown.trigger();
    }
}

fn manual_clock() -> (Clock, ManualClock) {
    Clock::manual(Time::from_system(SystemTime::now()))
}

/// `/zone_sequence_status` until its state is `state`.
async fn sequence_state(status: &mut Subscriber<Text>, state: &str) -> Value {
    loop {
        let doc: Value = serde_json::from_str(&next(status).await.data).unwrap();
        if doc["state"] == state {
            return doc;
        }
    }
}

/// Wait in real time for `done`, the manual clock still.
async fn settled(what: &str, mut done: impl FnMut() -> bool) {
    let deadline = Instant::now() + WAIT;
    while !done() {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
}

// ---------------------------------------------------------------------------
// The tests
// ---------------------------------------------------------------------------

/// Every endpoint under its ROS name, as the manifest has it; the two writers
/// that never published are gone; the parameter services are still the r2r
/// node's, on DDS.
#[test]
fn coverage_s_endpoints_are_on_the_bus_under_their_ros_names_and_in_the_manifest() {
    within(async {
        let bus = Bus::new();
        let (clock, manual) = manual_clock();
        let nav = FakeNav::start(&bus, &clock, Script::default());
        let _world = World::start(&bus);
        let coverage = Coverage::start(&bus, clock, manual).await;
        mower_bridge::coverage::assert_in_manifest(&coverage.m);
        let topics = bus.topics();
        let writer = |name: &str| topics.iter().any(|t| t.name == name && t.publishers.iter().any(|p| p == "boustrophedon_coverage"));
        let reader = |name: &str| topics.iter().any(|t| t.name == name && t.subscribers.iter().any(|p| p == "boustrophedon_coverage"));
        for name in ["/coverage_path", "/coverage_path_markers", "/coverage_invalid_segments", "/coverage_connectors", "/zone_sequence_status"] {
            assert!(writer(name), "{name}");
        }
        for name in ["/nav_operation_active", "/risk_map", "/risk_map_inflated"] {
            assert!(reader(name), "{name}");
        }
        assert!(!writer("/free_space_inflated") && !writer("/risk_map_inflated"), "docs/BUS_TOPICS.md 5.8 #2");
        let action = bus.actions().into_iter().find(|a| a.name == "/nav_action_follow_path").unwrap();
        assert!(action.clients.iter().any(|c| c == "boustrophedon_coverage"));
        let served: Vec<String> =
            bus.services().into_iter().filter(|s| s.server.as_deref() == Some("boustrophedon_coverage")).map(|s| s.name).collect();
        assert_eq!(served, ["/generate_coverage_path", "/resume_coverage", "/run_zone_sequence", "/stop_zone_sequence", "/zone_exec_path"]);
        assert!(!bus.services().iter().any(|s| s.name.starts_with("/boustrophedon_coverage/")), "on r2r");

        // The app's set_parameters and the adapter's poll still reach the node.
        let mut node = r2r::Node::create(context(), "coverage_bus_test", "").unwrap();
        let client = node.create_client::<GetParameters::Service>("/boustrophedon_coverage/get_parameters", QosProfile::services_default()).unwrap();
        let running = Arc::new(AtomicBool::new(true));
        let spin = {
            let running = running.clone();
            std::thread::spawn(move || {
                while running.load(Ordering::Acquire) {
                    node.spin_once(Duration::from_millis(5));
                }
            })
        };
        wait("the DDS parameter service", r2r::Node::is_available(&client).unwrap()).await.unwrap();
        let request = GetParameters::Request { names: vec!["strip_width_m".into()] };
        let answer = wait("the DDS answer", client.request(&request).unwrap()).await.unwrap();
        assert_eq!(answer.values[0].double_value, 0.8);
        running.store(false, Ordering::Release);
        spin.join().unwrap();
        coverage.stop().await;
        nav.stop();
        assert!(!bus.services().iter().any(|s| s.server.as_deref() == Some("boustrophedon_coverage")), "gone with the scope");
    });
}

/// A zone goes to nav: the goal carries the zone and a fresh dispatch id,
/// that same id is confirmed, and the zone's service answers once nav has
/// accepted. A two-zone sequence runs zone, channel, zone and completes.
#[test]
fn zones_and_sequences_reach_nav_through_its_action_server() {
    within(async {
        let bus = Bus::new();
        let (clock, manual) = manual_clock();
        let nav = FakeNav::start(&bus, &clock, Script { succeed_after: Some(Duration::from_secs(1)), ..Script::default() });
        let world = World::start(&bus);
        let coverage = Coverage::start(&bus, clock, manual.clone()).await;
        let answer = coverage.exec_zone_now().await;
        assert_eq!((answer.success, answer.message.as_str()), (true, "Navigation action goal accepted"));
        let (zone, resume, id) = nav.seen(|s| {
            let goal = &s.goals[0].1;
            (goal.zone_id, goal.resume_segment_index, goal.dispatch_id.clone())
        });
        assert_eq!((zone, resume, id.len()), (1, 0, 32));
        assert_eq!(nav.seen(|s| s.confirms.clone()), [id]);
        pass(&manual, Duration::from_secs(2)).await;

        let mut status = coverage.test.subscribe::<Text>("/zone_sequence_status", Qos::latched().keep_last(100)).unwrap();
        assert!(coverage.run_sequence(&[1, 2]).await.success);
        let mut states: Vec<(String, String)> = Vec::new();
        let started = Instant::now();
        while !states.last().is_some_and(|(s, _)| s != "running" && s != "idle") {
            assert!(started.elapsed() < WAIT, "{states:?}");
            pass(&manual, Duration::from_millis(200)).await;
            while let Ok(msg) = status.try_recv() {
                let doc: Value = serde_json::from_str(&msg.data).unwrap();
                states.push((doc["state"].as_str().unwrap().to_string(), doc["leg"].as_str().unwrap().to_string()));
            }
        }
        assert_eq!(states.last().unwrap().0, "completed", "{states:?}");
        let running: Vec<&str> = states.iter().filter(|(s, _)| s == "running").map(|(_, leg)| leg.as_str()).collect();
        assert_eq!(running, ["zone", "channel", "zone"]);
        let zones: Vec<i32> = nav.seen(|s| s.goals.iter().map(|(_, g)| g.zone_id).collect());
        assert_eq!(zones, [1, 1, -1, 2], "the single zone, then zone 1, the channel and zone 2");
        assert_eq!(*world.routes.lock().unwrap(), [(1, 2)]);
        assert!(nav.seen(|s| s.dispatch_cancels.is_empty() && s.action_cancels.is_empty()));
        coverage.stop().await;
        nav.stop();
    });
}

/// coverage_cancel_check.py's A: the confirmation fails and the action
/// accepts the cancel but never ends the goal. The 2 s retry sends ONE
/// correlated cancel, whose terminal_confirmed answer ends the tracking.
#[test]
fn a_failed_confirmation_is_tracked_until_a_correlated_cancel_proves_the_end() {
    within(async {
        let bus = Bus::new();
        let (clock, manual) = manual_clock();
        let nav = FakeNav::start(&bus, &clock, Script { confirm: false, ..Script::default() });
        let _world = World::start(&bus);
        let coverage = Coverage::start(&bus, clock.clone(), manual.clone()).await;
        let t0 = clock.mono();
        let answer = coverage.exec_zone_now().await;
        assert!(!answer.success && answer.message.contains("confirmation failed (scripted confirmation failure)"), "{}", answer.message);
        pass(&manual, Duration::from_secs(9)).await;
        let cancels: Vec<f64> = nav.seen(|s| s.action_cancels.iter().map(|t| since(t0, *t)).collect());
        assert!(cancels.len() == 1 && cancels[0] < 0.2, "the action cancel at once: {cancels:?}");
        let requests = nav.dispatch_cancels_since(t0);
        assert!(requests.len() == 1 && (1.9..=2.3).contains(&requests[0]), "one correlated cancel, from the 2 s retry: {requests:?}");
        coverage.stop().await;
        nav.stop();
    });
}

/// coverage_cancel_check.py's B: the goal is accepted at 4 s, past the 3 s
/// acceptance bound, and the cancel service hangs. One request at 3 s, which
/// the late goal's tracker shares; the late goal is canceled; after the 5 s
/// request timeout the retry sends the second; once the goal is terminal no
/// more go out.
#[test]
fn a_late_acceptance_is_adopted_canceled_and_tracked_with_one_request_at_a_time() {
    within(async {
        let bus = Bus::new();
        let (clock, manual) = manual_clock();
        let script = Script { accept_after: Duration::from_secs(4), cancel_dispatch: CancelDispatch::Hang, ..Script::default() };
        let nav = FakeNav::start(&bus, &clock, script);
        let _world = World::start(&bus);
        let coverage = Coverage::start(&bus, clock.clone(), manual.clone()).await;
        let t0 = clock.mono();
        let answer = coverage.exec_zone_now().await;
        assert!(!answer.success && answer.message.contains("acceptance timed out"), "{}", answer.message);
        pass(&manual, Duration::from_secs(12)).await;
        let requests = nav.dispatch_cancels_since(t0);
        let first: Vec<f64> = requests.iter().copied().filter(|t| *t < 7.8).collect();
        let second: Vec<f64> = requests.iter().copied().filter(|t| *t >= 7.8).collect();
        assert!(first.len() == 1 && (2.9..=3.3).contains(&first[0]), "one request at the 3 s timeout, shared by the late goal: {requests:?}");
        assert_eq!(second.len(), 1, "one retry after the 5 s request timeout: {requests:?}");
        let ids: std::collections::BTreeSet<String> = nav.seen(|s| s.dispatch_cancels.iter().map(|(_, id)| id.clone()).collect());
        assert_eq!(ids.len(), 1, "every request names the same dispatch");
        let cancels: Vec<f64> = nav.seen(|s| s.action_cancels.iter().map(|t| since(t0, *t)).collect());
        assert!(cancels.len() == 1 && (4.0..=4.3).contains(&cancels[0]), "the late goal is canceled when it comes: {cancels:?}");

        nav.end_all.notify_waiters();
        pass(&manual, Duration::from_secs(1)).await;
        let t_end = clock.mono();
        pass(&manual, Duration::from_secs(6)).await;
        assert_eq!(nav.dispatch_cancels_since(t_end), Vec::<f64>::new(), "nothing once the goal is terminal");
        coverage.stop().await;
        nav.stop();
    });
}

/// coverage_cancel_check.py's C: the confirmation fails and the action ends
/// the goal on the cancel. Its result proves the end: no correlated cancel.
#[test]
fn a_goal_the_action_cancel_ends_needs_no_correlated_cancel() {
    within(async {
        let bus = Bus::new();
        let (clock, manual) = manual_clock();
        let nav = FakeNav::start(&bus, &clock, Script { confirm: false, end_on_cancel: true, ..Script::default() });
        let _world = World::start(&bus);
        let coverage = Coverage::start(&bus, clock.clone(), manual.clone()).await;
        let t0 = clock.mono();
        assert!(!coverage.exec_zone_now().await.success);
        pass(&manual, Duration::from_secs(7)).await;
        assert_eq!(nav.seen(|s| s.action_cancels.len()), 1);
        assert_eq!(nav.dispatch_cancels_since(t0), Vec::<f64>::new());
        coverage.stop().await;
        nav.stop();
    });
}

/// coverage_cancel_check.py's D: coverage stops while it tracks a goal with
/// a hanging cancel service. It stops at once, and sends nothing after.
#[test]
fn a_shutdown_while_tracking_stops_at_once_and_sends_nothing_more() {
    within(async {
        let bus = Bus::new();
        let (clock, manual) = manual_clock();
        let nav = FakeNav::start(&bus, &clock, Script { confirm: false, cancel_dispatch: CancelDispatch::Hang, ..Script::default() });
        let _world = World::start(&bus);
        let coverage = Coverage::start(&bus, clock.clone(), manual.clone()).await;
        assert!(!coverage.exec_zone_now().await.success);
        pass(&manual, Duration::from_secs(3)).await;
        assert_eq!(nav.seen(|s| s.dispatch_cancels.len()), 1);
        let stopped = Instant::now();
        coverage.stop().await;
        assert!(stopped.elapsed() < Duration::from_secs(2), "{:?}", stopped.elapsed());
        let t_stop = clock.mono();
        pass(&manual, Duration::from_secs(8)).await;
        assert_eq!(nav.dispatch_cancels_since(t_stop), Vec::<f64>::new());
        nav.stop();
    });
}

/// coverage_cancel_check.py's E: `/resume_coverage` asks nav's checkpoint,
/// refuses without one and for another zone, and otherwise sends the zone's
/// path from the first unfinished segment.
#[test]
fn resume_coverage_follows_nav_s_checkpoint() {
    within(async {
        let bus = Bus::new();
        let (clock, manual) = manual_clock();
        let nav = FakeNav::start(&bus, &clock, Script::default());
        let _world = World::start(&bus);
        let coverage = Coverage::start(&bus, clock, manual).await;
        let resume = |zone_id| call::<_, ResumeCoverage::Response>(&coverage.test, "/resume_coverage", ResumeCoverage::Request { zone_id });
        let none = resume(-1).await;
        assert!(!none.success && none.message.starts_with("No resumable coverage checkpoint"), "{}", none.message);
        nav.script(|s| s.checkpoint = Some((1, 2, 5)));
        let other = resume(2).await;
        assert!(!other.success && other.message.contains("zone 1"), "{}", other.message);
        assert!(nav.seen(|s| s.goals.is_empty()));
        let resumed = resume(-1).await;
        assert!(resumed.success, "{}", resumed.message);
        let goal = nav.seen(|s| s.goals[0].1.clone());
        assert_eq!((goal.zone_id, goal.resume_segment_index), (1, 3));
        coverage.stop().await;
        nav.stop();
    });
}

/// `GoalRejected` is the one definite refusal: coverage asks
/// `/check_nav_status` why and reports it, and tracks nothing.
#[test]
fn a_rejected_goal_reports_nav_s_reason() {
    within(async {
        let bus = Bus::new();
        let (clock, manual) = manual_clock();
        let nav = FakeNav::start(&bus, &clock, Script { reject: true, block_reason: "GPS fix is stale", ..Script::default() });
        let _world = World::start(&bus);
        let coverage = Coverage::start(&bus, clock.clone(), manual.clone()).await;
        let t0 = clock.mono();
        let answer = coverage.exec_zone_now().await;
        assert_eq!((answer.success, answer.message.as_str()), (false, "Navigation action goal rejected: GPS fix is stale"));
        pass(&manual, Duration::from_secs(5)).await;
        assert_eq!(nav.dispatch_cancels_since(t0), Vec::<f64>::new());
        coverage.stop().await;
        nav.stop();
    });
}

/// No nav at all: after the 2 s wait for the server, "unavailable", and
/// nothing sent, as against a missing r2r server (with the bridge standalone,
/// its bus server only exists while DDS has one).
#[test]
fn without_nav_the_server_is_unavailable() {
    within(async {
        let bus = Bus::new();
        let (clock, manual) = manual_clock();
        let nav = FakeNav::start(&bus, &clock, Script::default());
        let _world = World::start(&bus);
        let coverage = Coverage::start(&bus, clock.clone(), manual.clone()).await;
        nav.stop();
        let t0 = clock.mono();
        let answer = coverage.exec_zone_now().await;
        assert_eq!((answer.success, answer.message.as_str()), (false, "Navigation action server unavailable"));
        assert!(since(t0, clock.mono()) >= 2.0, "after its 2 s wait");
        coverage.stop().await;
    });
}

/// `/stop_zone_sequence` cancels the running leg; nav ends it CANCELED, and
/// the sequence says canceled.
#[test]
fn stopping_a_sequence_cancels_its_leg() {
    within(async {
        let bus = Bus::new();
        let (clock, manual) = manual_clock();
        let nav = FakeNav::start(&bus, &clock, Script { end_on_cancel: true, ..Script::default() });
        let _world = World::start(&bus);
        let coverage = Coverage::start(&bus, clock, manual.clone()).await;
        let mut status = coverage.test.subscribe::<Text>("/zone_sequence_status", Qos::latched().keep_last(100)).unwrap();
        assert!(coverage.run_sequence(&[1, 2]).await.success);
        settled("the first leg's goal", || nav.seen(|s| s.goals.len() == 1)).await;
        let stop: Trigger::Response = call(&coverage.test, "/stop_zone_sequence", Trigger::Request {}).await;
        assert!(stop.success && stop.message.starts_with("已立即送出"), "{}", stop.message);
        let canceled = sequence_state(&mut status, "canceled").await;
        assert_eq!((canceled["leg"].as_str(), canceled["zone_id"].as_i64()), (Some("zone"), Some(1)));
        pass(&manual, Duration::from_secs(3)).await;
        assert_eq!(nav.seen(|s| (s.goals.len(), s.action_cancels.len(), s.dispatch_cancels.len())), (1, 1, 0));
        coverage.stop().await;
        nav.stop();
    });
}

/// nav's module ends while a sequence leg runs, and mower_rsd starts it
/// again. The bus loses the goal at once (`ServiceUnavailable`), where r2r
/// said nothing until the leg's timeout (at least 10 min): the sequence fails
/// now, and the goal, whose end nobody can prove, is tracked. Its correlated
/// cancel finds no service while nav is down, then a nav that cannot
/// confirm a dispatch it never had: every 2 s, and a new sequence is refused,
/// until the end is proven.
#[test]
fn when_nav_ends_mid_leg_the_goal_is_uncertain_until_proven_terminal() {
    within(async {
        let bus = Bus::new();
        let (clock, manual) = manual_clock();
        let nav = FakeNav::start(&bus, &clock, Script::default());
        let _world = World::start(&bus);
        let coverage = Coverage::start(&bus, clock.clone(), manual.clone()).await;
        let mut status = coverage.test.subscribe::<Text>("/zone_sequence_status", Qos::latched().keep_last(100)).unwrap();
        assert!(coverage.run_sequence(&[1, 2]).await.success);
        settled("the first leg's goal", || nav.seen(|s| s.goals.len() == 1)).await;
        let id = nav.seen(|s| s.goals[0].1.dispatch_id.clone());

        let t0 = clock.mono();
        nav.stop();
        let failed = sequence_state(&mut status, "failed").await;
        assert_eq!((failed["leg"].as_str(), failed["zone_id"].as_i64()), (Some("zone"), Some(1)));
        assert_eq!(clock.mono(), t0, "without waiting for anything");

        // down for 2 s: nobody answers the correlated cancel
        pass(&manual, Duration::from_secs(2)).await;
        let nav = FakeNav::start(&bus, &clock, Script { cancel_dispatch: CancelDispatch::Unconfirmed, ..Script::default() });
        pass(&manual, Duration::from_secs(5)).await;
        // at the 2 s watchdog and every 2 s retry from then on (one of the
        // two at 2 s may come first, or both)
        let requests = nav.dispatch_cancels_since(t0);
        let slots: std::collections::BTreeSet<i64> = requests.iter().map(|t| (t / 2.0).round() as i64).collect();
        assert!(slots.len() >= 2 && requests.iter().all(|t| (t - 2.0 * (t / 2.0).round()).abs() < 0.2), "retried every 2 s: {requests:?}");
        assert!(nav.seen(|s| s.dispatch_cancels.iter().all(|(_, d)| *d == id)));
        let refused = coverage.run_sequence(&[1]).await;
        assert!(!refused.success && refused.message.contains("終止狀態尚未確認"), "{}", refused.message);

        // once the end is proven, the tracking stops and a sequence may start
        nav.script(|s| s.cancel_dispatch = CancelDispatch::Terminal);
        pass(&manual, Duration::from_secs(3)).await;
        let proven = clock.mono();
        pass(&manual, Duration::from_secs(5)).await;
        assert_eq!(nav.dispatch_cancels_since(proven), Vec::<f64>::new());
        assert!(coverage.run_sequence(&[1]).await.success);
        coverage.stop().await;
        nav.stop();
    });
}
