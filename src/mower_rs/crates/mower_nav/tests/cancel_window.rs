//! A cancel that reaches nav after it has decided how a goal ends, but before
//! it has ended it, from each path a cancel comes by: the action's own
//! cancel, the correlated cancel (`/cancel_navigation_dispatch`), a manual
//! command (`/joy_cmd`) and the health gate.
//!
//! Since step 5, `Execution::terminate` only decides the end, and `end_goal`
//! sends it after `finish_progress` and `finish_reserved_goal`: in between,
//! the goal is still reserved, and still executing for the bus. A cancel
//! there was taken. It left nav "canceling" while it was idle, with
//! `/nav_operation_active` latched true (which blocks mowing) whenever its
//! publish came after `finish_reserved_goal`'s, and an action cancel was
//! accepted while the goal still ended SUCCEEDED. With r2r, rcl answered an
//! action cancel there itself (the goal had ended in `terminate`); the other
//! paths had the same race, in a narrower window. Now each path is refused
//! once the end is decided, nav publishes nothing for it, and an action
//! cancel gets `GoalTerminated` once the goal has ended.
//!
//! **The window is held open, not raced for** ([`Window`]). The test reads
//! `/coverage_progress` with a waker of its own, which the bus calls on nav's
//! thread from inside each publish. On the mission's final progress, which
//! `finish_progress` publishes after the decision, the waker sends the cancel
//! from a thread of its own. It returns once nav says "canceling", as nav did
//! every time before the fix, or after [`HOLD`], and nav cannot get on to
//! `finish_reserved_goal` meanwhile.
//!
//! That thread is not one of the runtime's, so what it wakes does not queue
//! behind nav's blocked worker. It keeps time without tokio, too: with nav's
//! worker blocked (before the fix, also the one that ran the late cancel), no
//! worker may be left driving tokio's timers until a task wakes one up, and
//! the thread's requests are what wake them.
//!
//! The harness is tests/bus.rs's (nav's real `run_node` on a bus, its callers
//! on another scope, a fake nav2 on DDS), on **ROS domain 81**.

use std::future::Future;
use std::panic::AssertUnwindSafe;
use std::path::PathBuf;
use std::pin::Pin;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};
use std::task::{Context, Poll, Wake, Waker};
use std::time::{Duration, Instant};

use futures::{FutureExt, Stream, StreamExt};
use mower_bus::{
    ActionClient, ActionClientGoal, Bus, BusError, CancelFuture, Clock, FeedbackStream, GoalStatus, Message, Qos,
    ResponseFuture, ResultFuture, Scope, Subscriber,
};
use mower_rs_common::{ModuleCtx, ModuleResult, Shutdown};
use r2r::geometry_msgs::msg::{Pose, PoseStamped, TwistStamped};
use r2r::lifecycle_msgs::srv::GetState;
use r2r::mower_interface::action::Waypoint;
use r2r::mower_interface::msg::CoverageProgress;
use r2r::mower_interface::srv::{CancelNavigationDispatch, ConfirmNavigationDispatch};
use r2r::nav2_msgs::action::{FollowPath, NavigateToPose};
use r2r::nav_msgs::msg::{Odometry, Path};
use r2r::sensor_msgs::msg::{Imu, NavSatFix};
use r2r::std_msgs::msg::Bool;
use r2r::std_srvs::srv::Trigger;
use r2r::{Parameter, ParameterValue, QosProfile};
use serde_json::Value;

/// For anything that waits in real time. A pass never waits this long.
const WAIT: Duration = Duration::from_secs(20);

/// How long an open window waits for nav to say "canceling". A pass waits
/// all of it: nav no longer does.
const HOLD: Duration = Duration::from_millis(500);

/// The bus side of `Waypoint::Action`.
type WaypointAction = (Waypoint::Goal, Waypoint::Result, Waypoint::Feedback);

type Accepted = (ActionClientGoal<WaypointAction>, ResultFuture<WaypointAction>, FeedbackStream<WaypointAction>);

fn context() -> r2r::Context {
    static CONTEXT: OnceLock<r2r::Context> = OnceLock::new();
    CONTEXT
        .get_or_init(|| {
            // rcl reads it when the context is created, and only then.
            std::env::set_var("ROS_DOMAIN_ID", "81");
            r2r::Context::create().expect("an rcl context (source /opt/ros/jazzy/setup.bash)")
        })
        .clone()
}

fn serial() -> MutexGuard<'static, ()> {
    static SERIAL: Mutex<()> = Mutex::new(());
    SERIAL.lock().unwrap_or_else(|e| e.into_inner())
}

/// One test on a runtime of its own, within a bound only a hang reaches (as
/// in tests/bus.rs). Several workers: the window blocks nav's.
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

/// A directory of the test's own (the checkpoint file).
fn dir(test: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("mower_nav_cancel_window_{}_{test}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

// ---------------------------------------------------------------------------
// the window
// ---------------------------------------------------------------------------

/// What the test does in the window between nav's decision and its end of
/// the goal, run from inside nav's publish of the mission's final
/// `/coverage_progress`, and what it saw there.
struct Window {
    progress: Mutex<Option<Subscriber<CoverageProgress>>>,
    mission: String,
    scope: Scope,
    late: Mutex<Option<Late>>,
    /// The mission's `/coverage_progress` statuses, in order.
    statuses: Mutex<Vec<String>>,
    /// `late` ran, from inside nav's publish.
    opened: AtomicBool,
    /// nav said "canceling" while the window was open.
    saw_canceling: AtomicBool,
    /// What failed on the window's thread, which must not panic: nav's
    /// publish would.
    trouble: Mutex<Vec<String>>,
}

/// The cancel, sent from the window's thread through the test's scope.
type Late = Box<dyn FnOnce(&Scope) + Send>;

/// What the window's thread sent, for the test to wait on afterwards.
type Sent<T> = Arc<Mutex<Option<Result<T, BusError>>>>;

impl Window {
    /// Read `mission`'s progress from now on: before its goal is sent.
    fn new(nav: &Nav, mission: &str) -> Arc<Window> {
        let progress = nav.test.subscribe::<CoverageProgress>("/coverage_progress", Qos::default().keep_last(200)).unwrap();
        Arc::new(Window {
            progress: Mutex::new(Some(progress)),
            mission: mission.into(),
            scope: nav.test.clone(),
            late: Mutex::new(None),
            statuses: Mutex::new(Vec::new()),
            opened: AtomicBool::new(false),
            saw_canceling: AtomicBool::new(false),
            trouble: Mutex::new(Vec::new()),
        })
    }

    /// Do `late` in the window: from the mission's final progress on, which
    /// must still be to come.
    fn arm(self: &Arc<Self>, late: impl FnOnce(&Scope) + Send + 'static) {
        *lock(&self.late) = Some(Box::new(late));
        self.drain();
        assert!(!self.opened.load(Ordering::SeqCst), "armed too late: the mission had ended");
    }

    /// Read what has come, and register for what comes next. The bus calls
    /// it on the publisher's thread once a publish has delivered.
    fn drain(self: &Arc<Self>) {
        let waker = Waker::from(self.clone());
        let mut cx = Context::from_waker(&waker);
        loop {
            let polled = match lock(&self.progress).as_mut() {
                Some(progress) => Pin::new(progress).poll_next(&mut cx),
                None => return,
            };
            let Poll::Ready(Some(msg)) = polled else { return };
            if msg.mission_id != self.mission {
                continue;
            }
            lock(&self.statuses).push(msg.status_text.clone());
            if msg.status_text != "succeeded" {
                continue;
            }
            let late = lock(&self.late).take();
            if let Some(late) = late {
                self.opened.store(true, Ordering::SeqCst);
                self.hold_open(late);
            }
        }
    }

    /// Send the cancel from a thread of its own, and keep nav here until it
    /// says "canceling" or `HOLD` has passed.
    fn hold_open(&self, late: Late) {
        let scope = self.scope.clone();
        let thread = std::thread::spawn(move || {
            late(&scope);
            says_canceling_within(&scope, HOLD)
        });
        match thread.join() {
            Ok(Ok(canceling)) => self.saw_canceling.store(canceling, Ordering::SeqCst),
            Ok(Err(e)) => lock(&self.trouble).push(e),
            Err(_) => lock(&self.trouble).push("the window's thread panicked".into()),
        }
    }

    /// Stop reading, and check the window: it opened, nav never said
    /// "canceling" there, and published no progress for the late cancel.
    fn close(&self) {
        drop(lock(&self.progress).take());
        let trouble = lock(&self.trouble).clone();
        assert!(trouble.is_empty(), "the window: {trouble:?}");
        assert!(self.opened.load(Ordering::SeqCst), "the window never opened");
        assert!(!self.saw_canceling.load(Ordering::SeqCst), "a cancel after the decision put nav in \"canceling\"");
        let statuses = lock(&self.statuses).clone();
        assert!(!statuses.iter().any(|s| s == "canceling"), "{statuses:?}");
        assert_eq!(statuses.iter().filter(|s| *s == "succeeded").count(), 1, "{statuses:?}");
    }
}

impl Wake for Window {
    fn wake(self: Arc<Self>) {
        self.drain();
    }

    fn wake_by_ref(self: &Arc<Self>) {
        self.drain();
    }
}

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

/// Ask `/check_nav_status`, from a std thread, until it says "canceling"
/// (`true`) or `hold` has passed (`false`).
fn says_canceling_within(scope: &Scope, hold: Duration) -> Result<bool, String> {
    let client = scope.client::<Trigger::Request, Trigger::Response>("/check_nav_status").map_err(|e| format!("{e:?}"))?;
    let deadline = Instant::now() + hold;
    loop {
        let answer = client.request(&Trigger::Request {}).map_err(|e| format!("/check_nav_status: {e:?}"))?;
        let response = block_on_within(answer, WAIT)
            .ok_or_else(|| "no /check_nav_status answer".to_string())?
            .map_err(|e| format!("/check_nav_status: {e:?}"))?;
        let status: Value = serde_json::from_str(&response.message).map_err(|e| e.to_string())?;
        if status["state"] == "canceling" {
            return Ok(true);
        }
        if Instant::now() >= deadline {
            return Ok(false);
        }
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// `fut`'s output, polled on this thread until `timeout` has passed.
fn block_on_within<F: Future + Unpin>(mut fut: F, timeout: Duration) -> Option<F::Output> {
    struct Unpark(std::thread::Thread);
    impl Wake for Unpark {
        fn wake(self: Arc<Self>) {
            self.0.unpark();
        }
    }
    let deadline = Instant::now() + timeout;
    let waker = Waker::from(Arc::new(Unpark(std::thread::current())));
    let mut cx = Context::from_waker(&waker);
    loop {
        if let Poll::Ready(output) = Pin::new(&mut fut).poll(&mut cx) {
            return Some(output);
        }
        let left = deadline.saturating_duration_since(Instant::now());
        if left.is_zero() {
            return None;
        }
        std::thread::park_timeout(left);
    }
}

// ---------------------------------------------------------------------------
// nav2 on DDS, nav, its inputs
// ---------------------------------------------------------------------------

/// A node of the process that is not nav's, spun on a thread of its own.
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
            let _ = thread.join();
        }
    }
}

/// nav2 as nav_progress_check.py fakes it: bt_navigator is active,
/// `navigate_to_pose` succeeds, `follow_path` counts its distance down and
/// succeeds.
struct FakeNav2 {
    _peer: Peer,
    tasks: Vec<tokio::task::JoinHandle<()>>,
}

impl FakeNav2 {
    fn start() -> FakeNav2 {
        let peer = Peer::new("fake_nav2");
        let services = QosProfile::services_default();
        let mut state = peer.with(|n| n.create_service::<GetState::Service>("bt_navigator/get_state", services)).unwrap();
        let mut navigate = peer.with(|n| n.create_action_server::<NavigateToPose::Action>("navigate_to_pose")).unwrap();
        let mut follow = peer.with(|n| n.create_action_server::<FollowPath::Action>("follow_path")).unwrap();
        let mut tasks = vec![tokio::spawn(async move {
            while let Some(request) = state.next().await {
                let mut response = GetState::Response::default();
                response.current_state.id = 3;
                response.current_state.label = "active".into();
                let _ = request.respond(response);
            }
        })];
        tasks.push(tokio::spawn(async move {
            while let Some(request) = navigate.next().await {
                let Ok((mut goal, _cancels)) = request.accept() else { continue };
                tokio::spawn(async move {
                    tokio::time::sleep(Duration::from_millis(20)).await;
                    let _ = goal.succeed(NavigateToPose::Result::default());
                });
            }
        }));
        tasks.push(tokio::spawn(async move {
            while let Some(request) = follow.next().await {
                let length = path_length(&request.goal.path);
                let Ok((mut goal, _cancels)) = request.accept() else { continue };
                tokio::spawn(async move {
                    for i in 1..=3 {
                        let left = length * (1.0 - i as f64 / 3.0);
                        let _ = goal.publish_feedback(FollowPath::Feedback { distance_to_goal: left as f32, speed: 0.3 });
                        tokio::time::sleep(Duration::from_millis(20)).await;
                    }
                    let _ = goal.succeed(FollowPath::Result::default());
                });
            }
        }));
        FakeNav2 { _peer: peer, tasks }
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

/// One start of nav_action_server on `bus`, as mower_rsd starts it, and a
/// scope for the test to call it from.
struct Nav {
    test: Scope,
    scope: Scope,
    shutdown: Shutdown,
    task: Option<tokio::task::JoinHandle<ModuleResult>>,
}

impl Nav {
    async fn start(bus: &Bus, dir: &std::path::Path, params: &[(&str, ParameterValue)]) -> Nav {
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
        let m = ModuleCtx::new("nav_action_server", shutdown.clone()).with_bus(scope.clone()).with_clock(Clock::system());
        let task = tokio::spawn(mower_nav::run_node(node, m));
        let test = bus.scope("test");
        let action = test.action_client::<WaypointAction>("/nav_action_follow_path").unwrap();
        wait("nav's action server", action.wait_for_server()).await.unwrap();
        Nav { test, scope, shutdown, task: Some(task) }
    }

    fn client(&self) -> ActionClient<WaypointAction> {
        self.test.action_client::<WaypointAction>("/nav_action_follow_path").unwrap()
    }

    async fn status(&self) -> Value {
        let r: Trigger::Response = call(&self.test, "/check_nav_status", Trigger::Request {}).await;
        serde_json::from_str(&r.message).expect("JSON")
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

    async fn cancel_dispatch(&self, dispatch_id: &str) -> CancelNavigationDispatch::Response {
        let request = CancelNavigationDispatch::Request { dispatch_id: dispatch_id.into() };
        call(&self.test, "/cancel_navigation_dispatch", request).await
    }

    /// The latched `/nav_operation_active`.
    async fn operation_active(&self) -> bool {
        let mut sub = self.test.subscribe::<Bool>("/nav_operation_active", Qos::latched()).unwrap();
        next(&mut sub).await.data
    }

    /// Send a goal and confirm its dispatch, as coverage does.
    async fn dispatch(&self, client: &ActionClient<WaypointAction>, goal: Waypoint::Goal) -> Accepted {
        let id = goal.dispatch_id.clone();
        let accepted = wait("the goal response", client.send_goal_request(goal).unwrap()).await.expect("accepted");
        let request = ConfirmNavigationDispatch::Request { dispatch_id: id };
        let confirmed: ConfirmNavigationDispatch::Response = call(&self.test, "/confirm_navigation_dispatch", request).await;
        assert!(confirmed.success, "the dispatch is confirmed");
        accepted
    }

    /// The goal ended SUCCEEDED, and nav is idle and says so: "completed",
    /// and `/nav_operation_active` false.
    async fn assert_done(&self, result: ResultFuture<WaypointAction>) {
        let (status, outcome) = wait("the result", result).await.unwrap();
        let state = self.status().await;
        assert_eq!(state["state"], "completed", "{state}");
        assert!(!self.operation_active().await, "/nav_operation_active is latched true while nav is idle");
        assert_eq!((status, outcome.success), (GoalStatus::Succeeded, true));
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

fn stamp_now() -> r2r::builtin_interfaces::msg::Time {
    let (sec, nanosec) = Clock::system().now().to_parts();
    r2r::builtin_interfaces::msg::Time { sec, nanosec }
}

/// A healthy picture for the health gate at 20 Hz (pose, GPS fix, GPS
/// odometry, IMU), as tests/bus.rs feeds it, until `feeding` is cleared:
/// once that has returned, it publishes nothing more.
fn feed_health(bus: &Bus, feeding: Arc<Mutex<bool>>) -> tokio::task::JoinHandle<()> {
    let sensors = bus.scope("sensors");
    let (pose_pub, fix_pub, odom_pub, imu_pub) = (
        sensors.publisher::<PoseStamped>("/adapter/robot_pose", Qos::default()).unwrap(),
        sensors.publisher::<NavSatFix>("/fix", Qos::default()).unwrap(),
        sensors.publisher::<Odometry>("/odometry/gps", Qos::default()).unwrap(),
        sensors.publisher::<Imu>("/imu/data", Qos::default()).unwrap(),
    );
    tokio::spawn(async move {
        loop {
            {
                let feeding = lock(&feeding);
                if !*feeding {
                    return;
                }
                let stamp = stamp_now();
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
            }
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
    })
}

// ---------------------------------------------------------------------------
// the four paths
// ---------------------------------------------------------------------------

/// The action's own cancel: answered `GoalTerminated` once the goal has
/// ended SUCCEEDED, as rcl answered it before step 5. Before the fix it was
/// accepted, and the goal still ended SUCCEEDED.
#[test]
fn a_late_action_cancel_is_answered_goal_terminated_and_changes_nothing() {
    within(async {
        let dir = dir("action");
        let _nav2 = FakeNav2::start();
        let bus = Bus::new();
        let nav = Nav::start(&bus, &dir, &[]).await;
        let id = dispatch_id();
        let window = Window::new(&nav, &id);
        let (goal, result, _feedback) = nav.dispatch(&nav.client(), mission(&id)).await;
        let cancel: Sent<CancelFuture> = Arc::default();
        window.arm({
            let cancel = cancel.clone();
            move |_| *lock(&cancel) = Some(goal.cancel())
        });
        nav.assert_done(result).await;
        window.close();
        let answer = lock(&cancel).take().expect("the window sent the cancel").expect("the cancel is sent");
        let answer = wait("the cancel's answer", answer).await;
        assert!(matches!(answer, Err(BusError::GoalTerminated { .. })), "{answer:?}");
        nav.stop().await;
        let _ = std::fs::remove_dir_all(&dir);
    });
}

/// The correlated cancel: "not active, terminal state not confirmed" in the
/// window (coverage asks again), "terminal" once the goal has ended. Before
/// the fix: "Correlated cancel requested", and nav was left canceling.
#[test]
fn a_late_correlated_cancel_finds_the_dispatch_ending() {
    within(async {
        let dir = dir("correlated");
        let _nav2 = FakeNav2::start();
        let bus = Bus::new();
        let nav = Nav::start(&bus, &dir, &[]).await;
        let id = dispatch_id();
        let window = Window::new(&nav, &id);
        let (_goal, result, _feedback) = nav.dispatch(&nav.client(), mission(&id)).await;
        let answer: Sent<ResponseFuture<CancelNavigationDispatch::Response>> = Arc::default();
        window.arm({
            let (answer, id) = (answer.clone(), id.clone());
            move |scope| {
                let request = CancelNavigationDispatch::Request { dispatch_id: id };
                let sent = scope.client::<_, CancelNavigationDispatch::Response>("/cancel_navigation_dispatch").and_then(|c| c.request(&request));
                *lock(&answer) = Some(sent);
            }
        });
        nav.assert_done(result).await;
        window.close();
        let answer = lock(&answer).take().expect("the window sent the cancel").expect("the cancel is sent");
        let answer = wait("the cancel's answer", answer).await.expect("an answer");
        assert_eq!(
            (answer.success, answer.terminal_confirmed, answer.message.as_str()),
            (true, false, "Matching dispatch is not active, but terminal state is not confirmed")
        );
        let again = nav.cancel_dispatch(&id).await;
        assert_eq!((again.success, again.terminal_confirmed, again.message.as_str()), (true, true, "Navigation dispatch is terminal"));
        nav.stop().await;
        let _ = std::fs::remove_dir_all(&dir);
    });
}

/// A manual command: it holds new goals off, as ever, but does not stop the
/// one that has ended.
#[test]
fn a_late_manual_command_leaves_the_ended_goal_alone() {
    within(async {
        let dir = dir("manual");
        let _nav2 = FakeNav2::start();
        let bus = Bus::new();
        // A hold that outlasts the window, to see it at the end.
        let nav = Nav::start(&bus, &dir, &[("manual_command_hold_s", ParameterValue::Double(10.0))]).await;
        let joy = nav.test.publisher::<TwistStamped>("/joy_cmd", Qos::default()).unwrap();
        let id = dispatch_id();
        let window = Window::new(&nav, &id);
        let (_goal, result, _feedback) = nav.dispatch(&nav.client(), mission(&id)).await;
        window.arm(move |_| {
            let mut command = TwistStamped::default();
            command.twist.linear.x = 0.2;
            let _ = joy.publish(&command);
        });
        nav.assert_done(result).await;
        window.close();
        assert_eq!(nav.status().await["block_reason"], "manual velocity command is active");
        nav.stop().await;
        let _ = std::fs::remove_dir_all(&dir);
    });
}

/// The health gate (production): a pose it refuses, once the inputs have
/// stopped, closes the gate, but does not cancel the goal that has ended.
#[test]
fn a_late_health_stop_leaves_the_ended_goal_alone() {
    within(async {
        let dir = dir("health");
        let _nav2 = FakeNav2::start();
        let bus = Bus::new();
        let nav = Nav::start(&bus, &dir, &[("require_navigation_health", ParameterValue::Bool(true))]).await;
        let feeding = Arc::new(Mutex::new(true));
        let feeder = feed_health(&bus, feeding.clone());
        nav.until_status("a healthy picture", |s| s["ready"] == true).await;
        let pose_pub = bus.scope("bad_pose").publisher::<PoseStamped>("/adapter/robot_pose", Qos::default()).unwrap();
        let id = dispatch_id();
        let window = Window::new(&nav, &id);
        let (_goal, result, _feedback) = nav.dispatch(&nav.client(), mission(&id)).await;
        window.arm(move |_| {
            *lock(&feeding) = false;
            let mut refused = pose(1.0, 2.0);
            refused.header.frame_id = "odom".into();
            refused.header.stamp = stamp_now();
            let _ = pose_pub.publish(&refused);
        });
        nav.assert_done(result).await;
        window.close();
        wait("the feeder", feeder).await.unwrap();
        assert_eq!(nav.status().await["block_reason"], "robot pose frame must be map");
        nav.stop().await;
        let _ = std::fs::remove_dir_all(&dir);
    });
}
