//! nav_action_server on the bus (step E2): the module's real `run_node`, its
//! callers on another scope of the same `Bus`, as mower_coverage and the
//! other modules of mower_rsd are, and nav2 on DDS: a fake nav2 node of the
//! test's own (`bt_navigator/get_state`, `navigate_to_pose`, `follow_path`,
//! as nav_progress_check.py fakes them), which nav reaches through its r2r
//! node, as it does until E9.
//!
//! - **ROS domain 82,** set before the process's one context exists: nav's r2r
//!   node and the fake nav2 talk DDS there, and nothing else on the host sees
//!   them. One test at a time: the context is process-wide and the nodes have
//!   fixed names.
//! - **Parameters** go into the node before `run_node` takes it, as the params
//!   file would put them: no health gate unless a test wants it, a checkpoint
//!   file of the test's own.
//! - **mower_coverage** runs against nav in the last tests: the one action pair
//!   of the process, with coverage's uncertain path when nav restarts mid-leg.

use std::future::Future;
use std::panic::AssertUnwindSafe;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};
use std::time::{Duration, Instant};

use futures::{FutureExt, StreamExt};
use mower_bus::{
    ActionClient, ActionClientGoal, Bus, BusError, Clock, FeedbackStream, GoalStatus, Message, Qos, ResultFuture, Scope,
    Subscriber, Time,
};
use mower_rs_common::{ModuleCtx, ModuleResult, Shutdown};
use r2r::geometry_msgs::msg::{Pose, PoseStamped, TwistStamped};
use r2r::lifecycle_msgs::srv::GetState;
use r2r::mower_interface::action::Waypoint;
use r2r::mower_interface::msg::{CoverageProgress, ZoneMap};
use r2r::mower_interface::srv::{
    CancelNavigationDispatch, ConfirmNavigationDispatch, GetCoverageProgress, MissionOperationLock, ZoneMapList,
    ZoneSequence,
};
use r2r::nav2_msgs::action::{FollowPath, NavigateToPose};
use r2r::nav_msgs::msg::{OccupancyGrid, Odometry, Path};
use r2r::sensor_msgs::msg::{Imu, NavSatFix};
use r2r::std_msgs::msg::{Bool, String as Text};
use r2r::std_srvs::srv::Trigger;
use r2r::{Parameter, ParameterValue, QosProfile};
use serde_json::Value;

/// For anything that waits in real time. A pass never waits this long.
const WAIT: Duration = Duration::from_secs(20);

/// The bus side of `Waypoint::Action`.
type WaypointAction = (Waypoint::Goal, Waypoint::Result, Waypoint::Feedback);

type Accepted = (ActionClientGoal<WaypointAction>, ResultFuture<WaypointAction>, FeedbackStream<WaypointAction>);

fn context() -> r2r::Context {
    static CONTEXT: OnceLock<r2r::Context> = OnceLock::new();
    CONTEXT
        .get_or_init(|| {
            // rcl reads it when the context is created, and only then.
            std::env::set_var("ROS_DOMAIN_ID", "82");
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
/// A test that fails half-way drops what it started, which only asks the
/// modules to stop: their own tasks still have to stop their r2r spin
/// threads, and a runtime does not go while a blocking thread runs. So the
/// failure waits a moment for that, and the runtime is shut down with a
/// bound, before it is reported.
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

async fn eventually(what: &str, mut done: impl FnMut() -> bool) {
    let deadline = Instant::now() + WAIT;
    while !done() {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        tokio::time::sleep(Duration::from_millis(10)).await;
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

/// A directory of the test's own (the checkpoint file).
fn dir(test: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("mower_nav_bus_{}_{test}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

// ---------------------------------------------------------------------------
// nav2 on DDS
// ---------------------------------------------------------------------------

/// A node of the process that is not nav's: spun on a thread of its own, its
/// endpoints made between two spins. Dropping it takes it off the graph.
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
                    // Let `with` in between: the lock is not fair.
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
            let _ = thread.join();
        }
    }
}

/// nav2 as nav_progress_check.py fakes it: bt_navigator is active,
/// `navigate_to_pose` succeeds, `follow_path` counts its distance down and
/// succeeds, or, while `hold` is set, runs until it is canceled.
struct FakeNav2 {
    _peer: Peer,
    hold: Arc<AtomicBool>,
    navigated: Arc<AtomicUsize>,
    followed: Arc<AtomicUsize>,
    canceled: Arc<AtomicUsize>,
    tasks: Vec<tokio::task::JoinHandle<()>>,
}

impl FakeNav2 {
    fn start() -> FakeNav2 {
        let peer = Peer::new("fake_nav2");
        let services = QosProfile::services_default();
        let mut state = peer.with(|n| n.create_service::<GetState::Service>("bt_navigator/get_state", services)).unwrap();
        let mut navigate = peer.with(|n| n.create_action_server::<NavigateToPose::Action>("navigate_to_pose")).unwrap();
        let mut follow = peer.with(|n| n.create_action_server::<FollowPath::Action>("follow_path")).unwrap();
        let hold = Arc::new(AtomicBool::new(false));
        let (navigated, followed, canceled) = (Arc::new(AtomicUsize::new(0)), Arc::new(AtomicUsize::new(0)), Arc::new(AtomicUsize::new(0)));
        let mut tasks = vec![tokio::spawn(async move {
            while let Some(request) = state.next().await {
                let mut response = GetState::Response::default();
                response.current_state.id = 3;
                response.current_state.label = "active".into();
                let _ = request.respond(response);
            }
        })];
        let counted = navigated.clone();
        tasks.push(tokio::spawn(async move {
            while let Some(request) = navigate.next().await {
                counted.fetch_add(1, Ordering::SeqCst);
                let Ok((mut goal, _cancels)) = request.accept() else { continue };
                tokio::spawn(async move {
                    tokio::time::sleep(Duration::from_millis(20)).await;
                    let _ = goal.succeed(NavigateToPose::Result::default());
                });
            }
        }));
        let (holding, counted, cancels_seen) = (hold.clone(), followed.clone(), canceled.clone());
        tasks.push(tokio::spawn(async move {
            while let Some(request) = follow.next().await {
                counted.fetch_add(1, Ordering::SeqCst);
                let length = path_length(&request.goal.path);
                let held = holding.load(Ordering::SeqCst);
                let Ok((mut goal, mut cancels)) = request.accept() else { continue };
                let canceled = cancels_seen.clone();
                tokio::spawn(async move {
                    for i in 1..=3 {
                        let left = length * (1.0 - i as f64 / 3.0);
                        let _ = goal.publish_feedback(FollowPath::Feedback { distance_to_goal: left as f32, speed: 0.3 });
                        tokio::time::sleep(Duration::from_millis(20)).await;
                    }
                    if !held {
                        let _ = goal.succeed(FollowPath::Result::default());
                    } else if let Some(cancel) = cancels.next().await {
                        canceled.fetch_add(1, Ordering::SeqCst);
                        cancel.accept();
                        let _ = goal.cancel(FollowPath::Result::default());
                    }
                });
            }
        }));
        FakeNav2 { _peer: peer, hold, navigated, followed, canceled, tasks }
    }

    fn hold(&self, hold: bool) {
        self.hold.store(hold, Ordering::SeqCst);
    }

    fn navigated(&self) -> usize {
        self.navigated.load(Ordering::SeqCst)
    }

    fn followed(&self) -> usize {
        self.followed.load(Ordering::SeqCst)
    }

    fn canceled(&self) -> usize {
        self.canceled.load(Ordering::SeqCst)
    }
}

impl Drop for FakeNav2 {
    fn drop(&mut self) {
        for task in &self.tasks {
            task.abort();
        }
    }
}

fn path_length(path: &Path) -> f64 {
    path.poses.windows(2).map(|w| (w[1].pose.position.x - w[0].pose.position.x).hypot(w[1].pose.position.y - w[0].pose.position.y)).sum()
}

// ---------------------------------------------------------------------------
// nav, and its callers
// ---------------------------------------------------------------------------

/// One start of nav_action_server on `bus`, as mower_rsd starts it, and a
/// scope for the test to call it from.
struct Nav {
    test: Scope,
    scope: Scope,
    m: ModuleCtx,
    shutdown: Shutdown,
    task: Option<tokio::task::JoinHandle<ModuleResult>>,
}

impl Nav {
    /// Start nav on `bus` and `clock` with `params` over the tests' defaults,
    /// and wait until it is up: its action servers come last.
    async fn start(bus: &Bus, clock: Clock, dir: &std::path::Path, params: &[(&str, ParameterValue)]) -> Nav {
        let node = r2r::Node::create(context(), "nav_action_server", "").unwrap();
        let checkpoint = dir.join("coverage_progress.json").display().to_string();
        let defaults = [
            ("require_navigation_health", ParameterValue::Bool(false)),
            ("progress_checkpoint_path", ParameterValue::String(checkpoint)),
            ("nav2_ready_poll_s", ParameterValue::Double(0.05)),
        ];
        for (key, value) in defaults.into_iter().chain(params.iter().cloned()) {
            node.params.lock().unwrap().insert(key.to_string(), Parameter::new(value));
        }
        let scope = bus.scope("nav_action_server");
        let shutdown = Shutdown::new();
        let m = ModuleCtx::new("nav_action_server", shutdown.clone()).with_bus(scope.clone()).with_clock(clock);
        let task = tokio::spawn(mower_nav::run_node(node, m.clone()));
        let test = bus.scope("test");
        let action = test.action_client::<WaypointAction>("/nav_action_follow_path").unwrap();
        wait("nav's action server", action.wait_for_server()).await.unwrap();
        Nav { test, scope, m, shutdown, task: Some(task) }
    }

    fn client(&self) -> ActionClient<WaypointAction> {
        self.test.action_client::<WaypointAction>("/nav_action_follow_path").unwrap()
    }

    async fn confirm(&self, dispatch_id: &str) -> ConfirmNavigationDispatch::Response {
        let request = ConfirmNavigationDispatch::Request { dispatch_id: dispatch_id.into() };
        call(&self.test, "/confirm_navigation_dispatch", request).await
    }

    async fn cancel_dispatch(&self, dispatch_id: &str) -> CancelNavigationDispatch::Response {
        let request = CancelNavigationDispatch::Request { dispatch_id: dispatch_id.into() };
        call(&self.test, "/cancel_navigation_dispatch", request).await
    }

    async fn trigger(&self, name: &str) -> Trigger::Response {
        call(&self.test, name, Trigger::Request {}).await
    }

    async fn status(&self) -> Value {
        serde_json::from_str(&self.trigger("/check_nav_status").await.message).expect("JSON")
    }

    /// Ask `/check_nav_status` until `done` says yes.
    async fn until_status(&self, what: &str, done: impl Fn(&Value) -> bool) {
        let deadline = Instant::now() + WAIT;
        loop {
            let status = self.status().await;
            if done(&status) {
                return;
            }
            assert!(Instant::now() < deadline, "timed out waiting for {what}: {status}");
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    }

    async fn lock(&self, owner: &str, acquire: bool) -> MissionOperationLock::Response {
        let request = MissionOperationLock::Request { owner: owner.into(), operation: "edit".into(), acquire };
        call(&self.test, "/mission_operation_lock", request).await
    }

    /// Send a goal and confirm its dispatch, as coverage does.
    async fn dispatch(&self, client: &ActionClient<WaypointAction>, goal: Waypoint::Goal) -> Accepted {
        let id = goal.dispatch_id.clone();
        let accepted = wait("the goal response", client.send_goal_request(goal).unwrap()).await.expect("accepted");
        assert!(self.confirm(&id).await.success, "the dispatch is confirmed");
        accepted
    }

    /// What mower_rsd does: stop the module, then close its scope.
    async fn stop(mut self) {
        self.shutdown.trigger();
        let task = self.task.take().expect("running");
        wait("nav to stop", task).await.expect("nav panicked").expect("nav failed");
        self.scope.close();
    }
}

impl Drop for Nav {
    /// A test that fails half-way still stops the module (see `within`).
    fn drop(&mut self) {
        self.shutdown.trigger();
    }
}

fn dispatch_id() -> String {
    uuid::Uuid::new_v4().simple().to_string()
}

fn pose(x: f64, y: f64) -> PoseStamped {
    let mut p = PoseStamped::default();
    p.header.frame_id = "map".into();
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation.w = 1.0;
    p
}

/// nav_progress_check.py's mission: a straight 6 m path with a pose every
/// 0.25 m, split at 2 and 4 m, so three 2 m FollowPath segments.
fn mission(dispatch_id: &str) -> Waypoint::Goal {
    let mut path = Path::default();
    path.header.frame_id = "map".into();
    path.poses = (0..=24).map(|i| pose(i as f64 * 0.25, 0.0)).collect();
    let split = |x: f64| {
        let mut p = Pose::default();
        p.position.x = x;
        p.orientation.w = 1.0;
        p
    };
    Waypoint::Goal { path, coverage_split_points: vec![split(2.0), split(4.0)], dispatch_id: dispatch_id.into(), zone_id: 7, resume_segment_index: 0 }
}

fn rejected(answer: Result<Accepted, BusError>) -> bool {
    matches!(answer, Err(BusError::GoalRejected { .. }))
}

// ---------------------------------------------------------------------------
// nav alone
// ---------------------------------------------------------------------------

/// Every endpoint under its ROS name, with its type and QoS as the manifest
/// has it, so that the bridge carries it; nav2's clients stay r2r.
#[test]
fn nav_s_endpoints_are_on_the_bus_under_their_ros_names_and_in_the_manifest() {
    within(async {
        let dir = dir("endpoints");
        let bus = Bus::new();
        let nav = Nav::start(&bus, Clock::system(), &dir, &[]).await;
        mower_bridge::coverage::assert_in_manifest(&nav.m);
        let actions = bus.actions();
        for name in ["/nav_action", "/nav_action_follow_path"] {
            let action = actions.iter().find(|a| a.name == name).unwrap_or_else(|| panic!("{name}"));
            assert_eq!((action.server.as_deref(), action.ros_type.as_deref()), (Some("nav_action_server"), Some("mower_interface/action/Waypoint")));
        }
        let services: Vec<String> = bus.services().into_iter().filter(|s| s.server.as_deref() == Some("nav_action_server")).map(|s| s.name).collect();
        assert_eq!(
            services,
            [
                "/cancel_nav2",
                "/cancel_navigation_dispatch",
                "/cencel_nav2",
                "/check_nav_status",
                "/confirm_navigation_dispatch",
                "/coverage_progress_status",
                "/mission_operation_lock"
            ]
        );
        // until E9 nav2 is reached through the node
        assert!(!bus.services().iter().any(|s| s.name.contains("bt_navigator")));
        assert!(!actions.iter().any(|a| a.name == "/navigate_to_pose" || a.name == "/follow_path"));
        let topics = bus.topics();
        let writes = |name: &str| topics.iter().any(|t| t.name == name && t.publishers.iter().any(|p| p == "nav_action_server"));
        let reads = |name: &str| topics.iter().any(|t| t.name == name && t.subscribers.iter().any(|p| p == "nav_action_server"));
        for name in ["/nav_operation_active", "/navigation_safety_stop", "/navigation_coordinator_lock", "/split_path", "/coverage_split_points", "/coverage_progress"] {
            assert!(writes(name), "{name}");
        }
        for name in ["/joy_cmd", "/physical_joy_cmd", "/keyboard_cmd_vel", "/adapter/robot_pose", "/imu/data", "/fix", "/odometry/gps", "/rosout"] {
            assert!(reads(name), "{name}");
        }
        nav.stop().await;
        // gone with the scope
        assert!(!bus.actions().iter().any(|a| a.server.is_some()));
        assert!(!bus.services().iter().any(|s| s.server.is_some()));
        let _ = std::fs::remove_dir_all(&dir);
    });
}

/// At idle autonomy is not authorised: twist_mux's lock says so at 20 Hz,
/// with a zero on the safety stop each time, and `/nav_operation_active` is
/// latched false. On a manual clock: exactly 20 beats a second.
#[test]
fn at_idle_the_lock_and_the_safety_stop_beat_at_20_hz() {
    within(async {
        let dir = dir("heartbeat");
        let bus = Bus::new();
        let (clock, manual) = Clock::manual(Time::from_system(std::time::SystemTime::now()));
        let early = bus.scope("early");
        let mut lock = early.subscribe::<Bool>("/navigation_coordinator_lock", Qos::default().keep_last(100)).unwrap();
        let mut stop = early.subscribe::<TwistStamped>("/navigation_safety_stop", Qos::default().keep_last(100)).unwrap();
        let nav = Nav::start(&bus, clock.clone(), &dir, &[]).await;
        let mut active = nav.test.subscribe::<Bool>("/nav_operation_active", Qos::latched()).unwrap();
        assert!(!next(&mut active).await.data, "latched false for a late joiner");
        // the first beat is at once, as tokio's interval had it
        eventually("the first beat", || lock.try_recv().is_ok()).await;
        while lock.try_recv().is_ok() {}
        while stop.try_recv().is_ok() {}
        let (mut locks, mut zeros) = (0, 0);
        for _ in 0..20 {
            let due = manual.clock().mono() + Duration::from_millis(50);
            manual.advance_to(due);
            eventually("a beat", || match lock.try_recv() {
                Ok(m) => {
                    assert!(m.data, "locked at idle");
                    locks += 1;
                    true
                }
                Err(_) => false,
            })
            .await;
            eventually("a zero", || match stop.try_recv() {
                Ok(z) => {
                    assert_eq!(z.header.frame_id, "base_footprint");
                    assert_eq!((z.twist.linear.x, z.twist.angular.z), (0.0, 0.0));
                    assert_eq!(mower_bus::Time::from_parts(z.header.stamp.sec, z.header.stamp.nanosec), clock.now(), "stamped on the module's clock");
                    zeros += 1;
                    true
                }
                Err(_) => false,
            })
            .await;
        }
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert!(lock.try_recv().is_err() && stop.try_recv().is_err(), "nothing between the beats");
        assert_eq!((locks, zeros), (20, 20));
        nav.stop().await;
        let _ = std::fs::remove_dir_all(&dir);
    });
}

/// A confirmed goal: nav2 drives to the start and follows the three
/// segments, the goal ends SUCCEEDED with success, and the progress and the
/// activity flag say so, before the caller learns the end.
#[test]
fn a_confirmed_goal_runs_its_segments_through_nav2_and_succeeds() {
    within(async {
        let dir = dir("succeeds");
        let nav2 = FakeNav2::start();
        let bus = Bus::new();
        let nav = Nav::start(&bus, Clock::system(), &dir, &[]).await;
        let mut progress = nav.test.subscribe::<CoverageProgress>("/coverage_progress", Qos::latched().keep_last(100)).unwrap();
        let mut active = nav.test.subscribe::<Bool>("/nav_operation_active", Qos::latched().keep_last(100)).unwrap();
        let client = nav.client();
        let id = dispatch_id();
        let (goal, result, _feedback) = nav.dispatch(&client, mission(&id)).await;
        assert_eq!(goal.get_status(), Ok(GoalStatus::Executing));
        let (status, outcome) = wait("the result", result).await.unwrap();
        assert_eq!((status, outcome.success), (GoalStatus::Succeeded, true));
        assert_eq!((nav2.navigated(), nav2.followed()), (1, 3));
        let mut last = None;
        while let Ok(msg) = progress.try_recv() {
            if msg.mission_id == id {
                last = Some(msg);
            }
        }
        let last = last.expect("progress of the mission");
        assert_eq!((last.status_text.as_str(), last.completed_segments, last.total_segments, last.zone_id), ("succeeded", 3, 3, 7));
        let flags: Vec<bool> = std::iter::from_fn(|| active.try_recv().ok()).map(|m| m.data).collect();
        assert_eq!((flags.first(), flags.last()), (Some(&false), Some(&false)), "{flags:?}");
        assert!(flags.contains(&true), "active while the goal ran: {flags:?}");
        let status = call::<_, GetCoverageProgress::Response>(&nav.test, "/coverage_progress_status", GetCoverageProgress::Request {}).await;
        assert_eq!((status.progress.mission_id.as_str(), status.progress.status_text.as_str()), (id.as_str(), "succeeded"));
        assert_eq!(nav.status().await["state"], "completed");
        nav.stop().await;
        let _ = std::fs::remove_dir_all(&dir);
    });
}

/// The caller's action cancel ends the goal CANCELED (what r2r delivered),
/// after nav2's goal is proven canceled. Nav has let the goal go by the time
/// the caller reads the result: a goal sent at once is not refused as
/// "another navigation goal is active".
#[test]
fn an_action_cancel_ends_the_goal_canceled_and_nav_is_free_when_the_caller_learns_it() {
    within(async {
        let dir = dir("cancel");
        let nav2 = FakeNav2::start();
        nav2.hold(true);
        let bus = Bus::new();
        let nav = Nav::start(&bus, Clock::system(), &dir, &[]).await;
        let client = nav.client();
        let (goal, result, _feedback) = nav.dispatch(&client, mission(&dispatch_id())).await;
        eventually("the first segment", || nav2.followed() == 1).await;
        assert_eq!(wait("the cancel", goal.cancel().unwrap()).await, Ok(()), "accepted");
        let (status, outcome) = wait("the result", result).await.unwrap();
        assert_eq!((status, outcome.success), (GoalStatus::Canceled, false));
        assert_eq!(nav2.canceled(), 1, "nav2's segment was canceled first");

        nav2.hold(false);
        let (_goal, result, _feedback) = nav.dispatch(&client, mission(&dispatch_id())).await;
        assert_eq!(wait("the next result", result).await.unwrap().0, GoalStatus::Succeeded);
        nav.stop().await;
        let _ = std::fs::remove_dir_all(&dir);
    });
}

/// What nav refuses is `GoalRejected`, the one definite refusal, and
/// `/check_nav_status` names the reason.
#[test]
fn goals_nav_refuses_are_rejected() {
    within(async {
        let dir = dir("rejected");
        let bus = Bus::new();
        let nav = Nav::start(&bus, Clock::system(), &dir, &[("dispatch_confirmation_timeout_s", ParameterValue::Double(1.0))]).await;
        let client = nav.client();
        let send = |goal: Waypoint::Goal| {
            let client = client.clone();
            async move { wait("the goal response", client.send_goal_request(goal).unwrap()).await }
        };
        let mut empty = mission(&dispatch_id());
        empty.path.poses.clear();
        assert!(rejected(send(empty).await), "an empty path");
        let mut no_id = mission("");
        no_id.dispatch_id = "not a uuid".into();
        assert!(rejected(send(no_id).await), "a dispatch id that is no UUID");

        let id = dispatch_id();
        let (_goal, result, _feedback) = send(mission(&id)).await.expect("accepted");
        assert!(rejected(send(mission(&dispatch_id())).await), "another goal is reserved");
        let status = nav.status().await;
        assert_eq!((status["state"].as_str(), status["block_reason"].as_str()), (Some("pending_confirmation"), Some("another navigation goal is active")));
        // never confirmed: it ends ABORTED, and its id is used up
        assert_eq!(wait("the result", result).await.unwrap().0, GoalStatus::Aborted);
        assert!(rejected(send(mission(&id)).await), "a reused dispatch id");
        nav.stop().await;
        let _ = std::fs::remove_dir_all(&dir);
    });
}

/// Nav keeps nav2 locked until the caller confirms the dispatch it was
/// given; without it the goal ends ABORTED and nav2 never gets a goal.
#[test]
fn an_unconfirmed_dispatch_aborts_without_starting_nav2() {
    within(async {
        let dir = dir("unconfirmed");
        let nav2 = FakeNav2::start();
        let bus = Bus::new();
        let nav = Nav::start(&bus, Clock::system(), &dir, &[("dispatch_confirmation_timeout_s", ParameterValue::Double(0.3))]).await;
        let client = nav.client();
        let id = dispatch_id();
        let (_goal, result, _feedback) = wait("the goal response", client.send_goal_request(mission(&id)).unwrap()).await.unwrap();
        let wrong = nav.confirm(&dispatch_id()).await;
        assert_eq!((wrong.success, wrong.message.as_str()), (false, "No pending navigation dispatch can be confirmed"));
        let (status, outcome) = wait("the result", result).await.unwrap();
        assert_eq!((status, outcome.success), (GoalStatus::Aborted, false));
        assert_eq!((nav2.navigated(), nav2.followed()), (0, 0));
        assert!(nav.status().await["message"].as_str().unwrap().contains("confirmation timed out"));
        nav.stop().await;
        let _ = std::fs::remove_dir_all(&dir);
    });
}

/// `/cancel_navigation_dispatch` (coverage's correlated cancel) and
/// `/cancel_nav2` (the app's) stop the running goal: nav2's segment is
/// canceled and the goal ends ABORTED, as no action cancel was accepted.
/// Afterwards the correlated cancel proves the dispatch terminal.
#[test]
fn the_cancel_services_abort_the_goal_and_then_prove_it_terminal() {
    within(async {
        let dir = dir("cancel_services");
        let nav2 = FakeNav2::start();
        nav2.hold(true);
        let bus = Bus::new();
        let nav = Nav::start(&bus, Clock::system(), &dir, &[]).await;
        let client = nav.client();
        let idle = nav.trigger("/cancel_nav2").await;
        assert_eq!((idle.success, idle.message.as_str()), (true, "No active Nav2 task to cancel"));

        let id = dispatch_id();
        let (_goal, result, _feedback) = nav.dispatch(&client, mission(&id)).await;
        eventually("the first segment", || nav2.followed() == 1).await;
        let other = nav.cancel_dispatch(&dispatch_id()).await;
        assert_eq!((other.success, other.terminal_confirmed), (true, false), "another dispatch: nothing to cancel, nothing proven");
        let answer = nav.cancel_dispatch(&id).await;
        assert_eq!((answer.success, answer.terminal_confirmed), (true, false));
        assert!(answer.message.starts_with("Correlated cancel requested"), "{}", answer.message);
        let (status, outcome) = wait("the result", result).await.unwrap();
        assert_eq!((status, outcome.success), (GoalStatus::Aborted, false));
        assert_eq!(nav2.canceled(), 1);
        let after = nav.cancel_dispatch(&id).await;
        assert_eq!((after.success, after.terminal_confirmed, after.message.as_str()), (true, true, "Navigation dispatch is terminal"));

        for name in ["/cancel_nav2", "/cencel_nav2"] {
            let (_goal, result, _feedback) = nav.dispatch(&client, mission(&dispatch_id())).await;
            nav.until_status("a running segment", |s| s["state"] == "running").await;
            let answer = nav.trigger(name).await;
            assert!(answer.success && answer.message.starts_with("Cancel requested for"), "{name}: {}", answer.message);
            assert_eq!(wait("the result", result).await.unwrap().0, GoalStatus::Aborted, "{name}");
        }
        nav.stop().await;
        let _ = std::fs::remove_dir_all(&dir);
    });
}


/// `/mission_operation_lock`: one lease at a time, released only by its
/// owner, none while a goal is reserved, and no goal while one is held.
#[test]
fn the_mission_operation_lock_is_one_lease_at_a_time_and_none_while_navigating() {
    within(async {
        let dir = dir("lock");
        let bus = Bus::new();
        let nav = Nav::start(&bus, Clock::system(), &dir, &[("dispatch_confirmation_timeout_s", ParameterValue::Double(1.0))]).await;
        let client = nav.client();
        let a = nav.lock("map_manage:a", true).await;
        assert_eq!((a.success, a.message.as_str()), (true, "Mutation lock acquired for edit"));
        let b = nav.lock("path_record_node:b", true).await;
        assert_eq!((b.success, b.message.as_str()), (false, "Another mission mutation is active: edit"));
        let goal = wait("the goal response", client.send_goal_request(mission(&dispatch_id())).unwrap()).await;
        assert!(rejected(goal), "no goal while a mutation is active");
        assert_eq!(nav.status().await["block_reason"], "mission mutation is active: edit");
        let b = nav.lock("path_record_node:b", false).await;
        assert_eq!((b.success, b.message.as_str()), (false, "Mutation lock is not owned by this requester"));
        assert!(nav.lock("map_manage:a", false).await.success);
        let empty = nav.lock(" ", true).await;
        assert_eq!((empty.success, empty.message.as_str()), (false, "Mutation lock owner must not be empty"));

        let (_goal, result, _feedback) = wait("the goal response", client.send_goal_request(mission(&dispatch_id())).unwrap()).await.unwrap();
        let busy = nav.lock("map_manage:a", true).await;
        assert_eq!((busy.success, busy.message.as_str()), (false, "Navigation/manual motion is active or termination is uncertain"));
        wait("the result", result).await.unwrap();
        assert!(nav.lock("map_manage:a", true).await.success, "free again once the goal ended");
        nav.stop().await;
        let _ = std::fs::remove_dir_all(&dir);
    });
}

/// A manual command on `/joy_cmd` (the manual guard's output) stops the
/// running goal, and blocks new ones while it holds.
#[test]
fn manual_motion_stops_a_goal_and_blocks_new_ones() {
    within(async {
        let dir = dir("manual");
        let nav2 = FakeNav2::start();
        nav2.hold(true);
        let bus = Bus::new();
        let nav = Nav::start(&bus, Clock::system(), &dir, &[]).await;
        let client = nav.client();
        let (_goal, result, _feedback) = nav.dispatch(&client, mission(&dispatch_id())).await;
        eventually("the first segment", || nav2.followed() == 1).await;
        let joy = nav.test.publisher::<TwistStamped>("/joy_cmd", Qos::default()).unwrap();
        let mut command = TwistStamped::default();
        command.twist.linear.x = 0.2;
        joy.publish(&command).unwrap();
        let (status, outcome) = wait("the result", result).await.unwrap();
        assert_eq!((status, outcome.success), (GoalStatus::Aborted, false));
        assert_eq!(nav2.canceled(), 1);
        joy.publish(&command).unwrap();
        nav.until_status("the hold", |s| s["block_reason"] == "manual velocity command is active").await;
        let goal = wait("the goal response", client.send_goal_request(mission(&dispatch_id())).unwrap()).await;
        assert!(rejected(goal));
        nav.stop().await;
        let _ = std::fs::remove_dir_all(&dir);
    });
}

/// With the health gate on (production), a goal needs a fresh pose, GPS fix,
/// GPS odometry and IMU, all read from the bus under their production names
/// and stamped on the module's clock.
#[test]
fn the_health_gate_reads_its_inputs_from_the_bus() {
    within(async {
        let dir = dir("health");
        let bus = Bus::new();
        let nav = Nav::start(&bus, Clock::system(), &dir, &[("require_navigation_health", ParameterValue::Bool(true))]).await;
        let client = nav.client();
        let goal = wait("the goal response", client.send_goal_request(mission(&dispatch_id())).unwrap()).await;
        assert!(rejected(goal));
        assert_eq!(nav.status().await["block_reason"], "robot pose/TF is unavailable or stale");

        let sensors = bus.scope("sensors");
        let (pose_pub, fix_pub, odom_pub, imu_pub) = (
            sensors.publisher::<PoseStamped>("/adapter/robot_pose", Qos::default()).unwrap(),
            sensors.publisher::<NavSatFix>("/fix", Qos::default()).unwrap(),
            sensors.publisher::<Odometry>("/odometry/gps", Qos::default()).unwrap(),
            sensors.publisher::<Imu>("/imu/data", Qos::default()).unwrap(),
        );
        let feeding = Arc::new(AtomicBool::new(true));
        let feeder = {
            let feeding = feeding.clone();
            tokio::spawn(async move {
                let clock = Clock::system();
                while feeding.load(Ordering::SeqCst) {
                    let (sec, nanosec) = clock.now().to_parts();
                    let stamp = r2r::builtin_interfaces::msg::Time { sec, nanosec };
                    let mut p = pose(1.0, 2.0);
                    p.header.stamp = stamp.clone();
                    let _ = pose_pub.publish(&p);
                    let mut fix = NavSatFix::default();
                    fix.header.stamp = stamp.clone();
                    (fix.latitude, fix.longitude, fix.status.status) = (25.0, 121.5, 2);
                    fix.position_covariance = vec![0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0004];
                    fix.position_covariance_type = 2;
                    let _ = fix_pub.publish(&fix);
                    let mut odom = Odometry::default();
                    odom.header.stamp = stamp.clone();
                    odom.header.frame_id = "map".into();
                    odom.pose.covariance = vec![0.0; 36];
                    (odom.pose.covariance[0], odom.pose.covariance[7]) = (0.0001, 0.0001);
                    let _ = odom_pub.publish(&odom);
                    let mut imu = Imu::default();
                    imu.header.stamp = stamp;
                    imu.header.frame_id = "imu_link".into();
                    imu.orientation.w = 1.0;
                    let cov = vec![0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01];
                    (imu.orientation_covariance, imu.angular_velocity_covariance) = (cov.clone(), cov.clone());
                    imu.linear_acceleration_covariance = cov;
                    let _ = imu_pub.publish(&imu);
                    tokio::time::sleep(Duration::from_millis(50)).await;
                }
            })
        };
        nav.until_status("a healthy picture", |s| s["ready"] == true).await;
        let (_goal, result, _feedback) = wait("the goal response", client.send_goal_request(mission(&dispatch_id())).unwrap()).await.expect("admitted");
        // The inputs stop: the gate closes once they are stale (0.3 s), and
        // cancels the reserved goal.
        feeding.store(false, Ordering::SeqCst);
        let _ = feeder.await;
        let (status, _) = wait("the result", result).await.unwrap();
        assert_eq!(status, GoalStatus::Aborted);
        assert!(nav.status().await["block_reason"].as_str().is_some_and(|r| r.contains("stale")));
        nav.stop().await;
        let _ = std::fs::remove_dir_all(&dir);
    });
}

/// nav's module ends mid-goal (mower_rsd then closes its scope and starts it
/// again 2 s later): the bus tells the caller at once, where r2r told it
/// nothing until its own timeout. The result fails with
/// `ServiceUnavailable`, the status is `Unknown`, a cancel fails too. The
/// restarted nav does not know the dispatch: its correlated cancel answers
/// "not active, terminal state not confirmed", which keeps coverage's
/// tracking (and the mower) blocked, as before.
#[test]
fn a_restart_mid_goal_loses_the_goal_for_its_caller_at_once() {
    within(async {
        let dir = dir("restart");
        let nav2 = FakeNav2::start();
        nav2.hold(true);
        let bus = Bus::new();
        let nav = Nav::start(&bus, Clock::system(), &dir, &[]).await;
        let client = nav.client();
        let id = dispatch_id();
        let (goal, result, mut feedback) = nav.dispatch(&client, mission(&id)).await;
        eventually("the first segment", || nav2.followed() == 1).await;
        nav.stop().await;
        let lost = wait("the result", result).await;
        assert!(matches!(lost, Err(BusError::ServiceUnavailable { .. })), "{lost:?}");
        assert_eq!(goal.get_status(), Ok(GoalStatus::Unknown));
        let cancel = wait("the cancel", goal.cancel().unwrap()).await;
        assert!(matches!(cancel, Err(BusError::ServiceUnavailable { .. })), "{cancel:?}");
        assert!(wait("the end of the feedback", feedback.next()).await.is_none());
        let refused = client.send_goal_request(mission(&dispatch_id()));
        assert!(matches!(refused, Err(BusError::ServiceUnavailable { .. })), "no server while nav is down: {refused:?}");

        let nav = Nav::start(&bus, Clock::system(), &dir, &[]).await;
        let answer = nav.cancel_dispatch(&id).await;
        assert_eq!((answer.success, answer.terminal_confirmed), (true, false));
        assert_eq!(answer.message, "Matching dispatch is not active, but terminal state is not confirmed");
        nav.stop().await;
        let _ = std::fs::remove_dir_all(&dir);
    });
}

// ---------------------------------------------------------------------------
// coverage and nav: the one action pair in the process
// ---------------------------------------------------------------------------

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

/// map_manage's half of what coverage needs: the zone map list and the
/// inflated risk map (none), from the test's scope.
fn serve_maps(scope: &Scope) -> (tokio::task::JoinHandle<()>, mower_bus::Publisher<OccupancyGrid>) {
    let mut risk_free = grid(0.0);
    risk_free.data.iter_mut().for_each(|v| *v = 0);
    let risk = scope.publisher::<OccupancyGrid>("/risk_map_inflated", Qos::latched()).unwrap();
    risk.publish(&risk_free).unwrap();
    let mut zones = scope.service::<ZoneMapList::Request, ZoneMapList::Response>("/get_zone_map_list_srv").unwrap();
    let zone = ZoneMap { zone_id: 1, mask_map: grid(0.0), mask_map_inflated: grid(0.75), ..Default::default() };
    let task = tokio::spawn(async move {
        while let Some(request) = zones.next().await {
            let _ = request.respond(ZoneMapList::Response { zone_map_list: vec![zone.clone()] });
        }
    });
    (task, risk)
}

/// One start of boustrophedon_coverage on `bus`, with its path generated.
struct Coverage {
    test: Scope,
    scope: Scope,
    m: ModuleCtx,
    shutdown: Shutdown,
    task: Option<tokio::task::JoinHandle<ModuleResult>>,
}

impl Coverage {
    async fn start(bus: &Bus) -> Coverage {
        let scope = bus.scope("boustrophedon_coverage");
        let shutdown = Shutdown::new();
        let m = ModuleCtx::new("boustrophedon_coverage", shutdown.clone()).with_bus(scope.clone());
        let task = tokio::spawn(mower_coverage::run(context(), m.clone()));
        let test = bus.scope("test");
        // it refuses until nav's latched activity flag has reached it
        let deadline = Instant::now() + WAIT;
        loop {
            let answer: Trigger::Response = call(&test, "/generate_coverage_path", Trigger::Request {}).await;
            if answer.success {
                break;
            }
            // nav's flag or the risk map has not reached it yet
            let early = answer.message.contains("navigation state is not available") || answer.message.contains("risk_map_inflated");
            assert!(early && Instant::now() < deadline, "{}", answer.message);
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        Coverage { test, scope, m, shutdown, task: Some(task) }
    }

    async fn run_sequence(&self, zone_ids: &[i32]) -> ZoneSequence::Response {
        call(&self.test, "/run_zone_sequence", ZoneSequence::Request { zone_ids: zone_ids.to_vec(), channel_proximity_m: 0.0 }).await
    }

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

/// `/zone_sequence_status`'s state, once it is `state`.
async fn sequence_state(status: &mut Subscriber<Text>, state: &str) -> Value {
    loop {
        let doc: Value = serde_json::from_str(&next(status).await.data).unwrap();
        if doc["state"] == state {
            return doc;
        }
    }
}

/// A zone sequence from coverage's service to nav2's fake: coverage plans
/// the zone under nav's lease, sends the goal to nav's server on the bus,
/// confirms the dispatch, and nav drives every segment through nav2.
#[test]
fn coverage_runs_a_zone_through_nav_on_the_bus() {
    within(async {
        let dir = dir("e2e");
        let nav2 = FakeNav2::start();
        let bus = Bus::new();
        let nav = Nav::start(&bus, Clock::system(), &dir, &[]).await;
        let maps = bus.scope("map_manage");
        let (_zones, _risk) = serve_maps(&maps);
        let coverage = Coverage::start(&bus).await;
        let mut status = coverage.test.subscribe::<Text>("/zone_sequence_status", Qos::latched().keep_last(100)).unwrap();
        assert!(coverage.run_sequence(&[1]).await.success);
        let done = sequence_state(&mut status, "completed").await;
        assert_eq!(done["zone_ids"], serde_json::json!([1]));
        assert!(nav2.followed() >= 1 && nav2.navigated() == 1, "nav2 drove it: {} segments", nav2.followed());
        mower_bridge::coverage::assert_in_manifest(&nav.m);
        mower_bridge::coverage::assert_in_manifest(&coverage.m);
        let action = bus.actions().into_iter().find(|a| a.name == "/nav_action_follow_path").unwrap();
        assert!(action.clients.iter().any(|c| c == "boustrophedon_coverage") && action.goals >= 1);
        coverage.stop().await;
        nav.stop().await;
        let _ = std::fs::remove_dir_all(&dir);
    });
}

/// nav's module ends while coverage's leg runs. On the bus coverage learns
/// it at once (r2r: not before its leg timeout, at least 10 min): the
/// sequence fails now, and the goal, whose end nobody can prove, goes to
/// coverage's tracker. It retries the correlated cancel every 2 s; the
/// restarted nav cannot confirm a dispatch it never had, so the tracker
/// stays, and a new sequence is refused, as after the timeout before.
#[test]
fn when_nav_restarts_mid_leg_coverage_keeps_the_goal_uncertain() {
    within(async {
        let dir = dir("e2e_restart");
        let nav2 = FakeNav2::start();
        nav2.hold(true);
        let bus = Bus::new();
        let nav = Nav::start(&bus, Clock::system(), &dir, &[]).await;
        let maps = bus.scope("map_manage");
        let (_zones, _risk) = serve_maps(&maps);
        let coverage = Coverage::start(&bus).await;
        let mut status = coverage.test.subscribe::<Text>("/zone_sequence_status", Qos::latched().keep_last(100)).unwrap();
        assert!(coverage.run_sequence(&[1]).await.success);
        eventually("the first segment", || nav2.followed() == 1).await;

        let stopped = Instant::now();
        nav.stop().await;
        let failed = sequence_state(&mut status, "failed").await;
        assert!(stopped.elapsed() < Duration::from_secs(2), "at once: {:?}", stopped.elapsed());
        assert_eq!(failed["leg"], "zone");

        let calls = || bus.services().into_iter().find(|s| s.name == "/cancel_navigation_dispatch").map_or(0, |s| s.calls);
        let nav = Nav::start(&bus, Clock::system(), &dir, &[]).await;
        let before = calls();
        eventually("two more correlated cancels", || calls() >= before + 2).await;
        let refused = coverage.run_sequence(&[1]).await;
        assert!(!refused.success && refused.message.contains("終止狀態尚未確認"), "{}", refused.message);
        coverage.stop().await;
        nav.stop().await;
        let _ = std::fs::remove_dir_all(&dir);
    });
}
