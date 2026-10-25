//! In-process navigation (docs/ROS_FREE_PLAN.md section 8, D6):
//! mower_nav_core's `NavStack` (navigator + planner server + controller
//! server) and `VelocitySmoother` on a dedicated thread, in place of nav2's
//! `navigate_to_pose` / `follow_path` actions and `bt_navigator/get_state`.
//!
//! Selected with the node parameter `nav_backend: nav_core`
//! (`RUST_NAV_CORE=true`, which also stops nav2 from being launched);
//! `nav_backend: nav2`, the default, keeps the r2r action clients.
//!
//! What the runtime does (one "nav-core" `std::thread`, not the shared tokio
//! workers):
//!
//! * builds the global costmap from `/map_grid_global` and the 3 x 3 m
//!   rolling local costmap from `/map_grid` (both `nav_msgs/OccupancyGrid`,
//!   reliable + transient_local), keeps them current at 1 Hz / 2 Hz the way
//!   nav2's costmaps are, and applies `StackOutput::clears` (a cleared
//!   costmap is not current until its next update);
//! * reads the robot pose from mower_localize's in-process tfbus
//!   (`PoseSource::TfBus`: `map <- odom` and `odom <- base_footprint`, the
//!   latest of each) or from `/tf` (`PoseSource::TfTopic`, for tests and for
//!   a process without mower_localize), and the twist from `/odom`;
//! * ticks `NavStack` at least every 10 ms (`next_tick_ns`), feeds its
//!   commands to the velocity smoother and publishes the smoother's 20 Hz
//!   output on `/nav_cmd_vel` (`geometry_msgs/TwistStamped`, frame
//!   `base_footprint`, the controller's stamp kept: velocity_command_guard
//!   rejects stale or backward stamps), and `/plan` while navigating;
//! * routes results and feedback to the `GoalHandle` of the goal they belong
//!   to, and keeps a ring of recent warnings for mower_nav's failure messages.
//!
//! All of that is [`core_loop`], which is pure and runs on an explicit clock;
//! this file is its IO. The subscriptions are drained by tokio tasks into
//! shared state (latest odometry, maps not taken yet) and the transforms are
//! read where they live (the tfbus, or a graph fed from `/tf`). The thread
//! loops: take the new maps, the queued commands and the latest transforms
//! and twist, tick the loop at `SystemTime::now()`, publish, send outcomes and
//! feedback, then sleep until the loop is next due (at most 10 ms), or block
//! on the command channel while nothing runs. The publishers live on the
//! thread (an `r2r::Publisher` is `Send`, and `rcl_publish` is thread-safe).
//!
//! The API never waits for the thread: a goal or a cancel is a message on an
//! unbounded channel, and readiness, the events and the tick age are read
//! from state the thread only locks for a moment. A stalled thread therefore
//! shows in [`NavRuntime::last_tick_age`] and nowhere else.

use std::collections::{HashMap, HashSet};
use std::panic::AssertUnwindSafe;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{mpsc, Arc, Mutex, MutexGuard, PoisonError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use futures::{Stream, StreamExt};
use mower_localize::tfbus;
use mower_localize::tfgraph::TfGraph;
use mower_localize_core::tf::{Quaternion, Transform, Vector3};
pub use mower_nav_core::action::GoalStatus;
use mower_nav_core::costmap::OccupancyGrid;
use mower_nav_core::geometry::{quaternion_from_yaw, yaw_from_quaternion, Pose2D, Transform2D, Twist2D};
use r2r::builtin_interfaces::msg::Time;
use r2r::geometry_msgs::msg::{Point, Pose, PoseStamped, Quaternion as RQuaternion, Transform as RTransform, Twist, TwistStamped, Vector3 as RVector3};
use r2r::nav_msgs::msg::{OccupancyGrid as ROccupancyGrid, Odometry, Path};
use r2r::std_msgs::msg::Header;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::QosProfile;

pub mod core_loop;

use core_loop::{CoreLoop, Inputs, LoopParams, TickInput, TickOutput, SEC};

/// Where the robot pose comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PoseSource {
    /// mower_localize's process-global transform cache (no `/tf` traffic;
    /// requires the localize module in the same process).
    TfBus,
    /// Subscribe to `/tf` and `/tf_static`.
    TfTopic,
}

/// The runtime's parameters (`nav_core.*` on the nav node).
#[derive(Debug, Clone, PartialEq)]
pub struct CoreParams {
    /// `nav_core.pose_source`: `"tfbus"` (default) or `"tf"`.
    pub pose_source: PoseSource,
    pub odom_topic: String,
    pub map_topic: String,
    pub global_map_topic: String,
    pub cmd_vel_topic: String,
    pub plan_topic: String,
    pub map_frame: String,
    pub odom_frame: String,
    pub base_frame: String,
    /// Odometry older than this does not count as present (readiness).
    pub odom_timeout_s: f64,
}

impl Default for CoreParams {
    fn default() -> Self {
        Self {
            pose_source: PoseSource::TfBus,
            odom_topic: "/odom".into(),
            map_topic: "/map_grid".into(),
            global_map_topic: "/map_grid_global".into(),
            cmd_vel_topic: "/nav_cmd_vel".into(),
            plan_topic: "/plan".into(),
            map_frame: "map".into(),
            odom_frame: "odom".into(),
            base_frame: "base_footprint".into(),
            odom_timeout_s: 0.5,
        }
    }
}

/// A goal sent to the runtime. The runtime numbers its goals itself, so
/// sending never waits for the thread.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GoalKey {
    /// `navigate_to_pose`
    Navigate(u64),
    /// `follow_path`
    Follow(u64),
}

/// How a goal ended.
#[derive(Debug, Clone, PartialEq)]
pub struct GoalOutcome {
    pub status: GoalStatus,
    /// nav2's numbering: FollowPath 100–107, ComputePathToPose 200–208, or 0
    /// (success, cancel, and the empty result of a preempted goal).
    pub error_code: u16,
    /// Human-readable cause for a nonzero code (nav2 1.3.13 leaves it empty).
    pub error_msg: String,
}

/// Feedback while a goal runs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CoreFeedback {
    /// NavigateToPose feedback.
    Navigate { distance_remaining: f32 },
    /// FollowPath feedback; `distance_to_goal` is measured in `map`.
    Follow { distance_to_goal: f32, speed: f32 },
}

/// One goal's channels. `result` resolves exactly once; it errors only if the
/// runtime is dropped first (mower_nav then treats the outcome as unproven).
/// `feedback` closes when the goal ends, however it ends.
pub struct GoalHandle {
    pub key: GoalKey,
    pub result: tokio::sync::oneshot::Receiver<GoalOutcome>,
    pub feedback: tokio::sync::mpsc::UnboundedReceiver<CoreFeedback>,
}

/// What the runtime has received; `ready()` replaces nav2's
/// `bt_navigator/get_state == active`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Readiness {
    pub global_map: bool,
    pub local_map: bool,
    pub pose: bool,
    pub odometry: bool,
}

impl Readiness {
    pub fn ready(&self) -> bool {
        self.global_map && self.local_map && self.pose && self.odometry
    }
}

/// The longest the thread sleeps while something runs.
const MAX_SLEEP: Duration = Duration::from_millis(10);
/// A tick that takes longer than a controller period is reported, at most
/// once per `SLOW_TICK_REPORT` and server (a big map can make every replan
/// that slow, and the ring has room for 40 lines).
const SLOW_TICK: Duration = Duration::from_millis(50);
const SLOW_TICK_REPORT: Duration = Duration::from_secs(10);

/// The running stack. Dropping it stops the thread; goals still running end
/// without an outcome (their `result` receivers error).
pub struct NavRuntime {
    shared: Arc<Shared>,
    commands: mpsc::Sender<Command>,
    next_key: AtomicU64,
    thread: Option<JoinHandle<()>>,
    /// The subscription tasks.
    tasks: Vec<tokio::task::JoinHandle<()>>,
}

// mower_nav keeps the runtime in its shared context and calls it from any
// tokio task.
const _: () = {
    const fn shared<T: Send + Sync>() {}
    shared::<NavRuntime>()
};

impl NavRuntime {
    /// Creates the subscriptions and publishers on `node` and starts the
    /// thread. `logger` is the node's logger name. Call it inside a tokio
    /// runtime (the subscriptions are drained by tokio tasks); the caller
    /// spins `node`.
    pub fn start(node: &mut r2r::Node, params: CoreParams, logger: &str) -> Result<Arc<Self>, r2r::Error> {
        // reliable, depth 10: twist_mux subscribes reliable
        let cmd_pub = node.create_publisher::<TwistStamped>(&params.cmd_vel_topic, QosProfile::default())?;
        let plan_pub = node.create_publisher::<Path>(&params.plan_topic, QosProfile::default().keep_last(1))?;
        let odom = node.subscribe::<Odometry>(&params.odom_topic, QosProfile::default())?;
        let local_map = node.subscribe::<ROccupancyGrid>(&params.map_topic, latched())?;
        let global_map = node.subscribe::<ROccupancyGrid>(&params.global_map_topic, latched())?;
        let tf = match params.pose_source {
            PoseSource::TfBus => None,
            PoseSource::TfTopic => Some((
                node.subscribe::<TFMessage>("/tf", QosProfile::default().keep_last(100))?,
                node.subscribe::<TFMessage>("/tf_static", QosProfile::default().keep_last(100).reliable().transient_local())?,
            )),
        };

        let shared = Arc::new(Shared {
            topic_tf: tf.is_some().then(|| Mutex::new(TopicTf::default())),
            inputs: Mutex::new(Inputs::default()),
            live: Mutex::new(HashSet::new()),
            events: Mutex::new(Vec::new()),
            last_tick: Mutex::new(Instant::now()),
            idle: AtomicBool::new(false),
            alive: AtomicBool::new(true),
            params: params.clone(),
        });
        let core = CoreLoop::new(LoopParams {
            map_frame: params.map_frame.clone(),
            odom_frame: params.odom_frame.clone(),
            base_frame: params.base_frame.clone(),
            odom_timeout_ns: (params.odom_timeout_s * 1e9) as i64,
        });
        let (commands, rx) = mpsc::channel();
        let io = Io {
            shared: shared.clone(),
            rx,
            cmd_pub,
            plan_pub,
            logger: logger.to_string(),
            sinks: HashMap::new(),
            logged_seq: 0,
            events_revision: 0,
            publish_failed: HashSet::new(),
            slow_reported: HashMap::new(),
        };
        let thread_logger = logger.to_string();
        let thread = std::thread::Builder::new()
            .name("nav-core".into())
            .spawn(move || {
                if let Err(e) = std::panic::catch_unwind(AssertUnwindSafe(|| io.run(core))) {
                    let what = e.downcast_ref::<&str>().map(|s| s.to_string()).or_else(|| e.downcast_ref::<String>().cloned()).unwrap_or_default();
                    r2r::log_error!(&thread_logger, "nav_core: the nav-core thread panicked ({what}); navigation is stopped");
                }
            })
            .map_err(|_| r2r::Error::RCL_RET_ERROR)?;

        let mut tasks = vec![
            {
                let shared = shared.clone();
                let mut odom = odom;
                tokio::spawn(async move {
                    while let Some(msg) = odom.next().await {
                        let t = &msg.twist.twist;
                        lock(&shared.inputs).set_odom(Twist2D::new(t.linear.x, t.linear.y, t.angular.z), wall_ns());
                    }
                })
            },
            spawn_map(shared.clone(), local_map, false),
            spawn_map(shared.clone(), global_map, true),
        ];
        if let Some((tf, tf_static)) = tf {
            tasks.push(spawn_tf(shared.clone(), tf, false));
            tasks.push(spawn_tf(shared.clone(), tf_static, true));
        }
        let source = match params.pose_source {
            PoseSource::TfBus => "the tfbus",
            PoseSource::TfTopic => "/tf",
        };
        r2r::log_info!(
            logger,
            "nav_core: in-process navigation, robot pose from {source}, maps {} / {}, commands on {}",
            params.global_map_topic,
            params.map_topic,
            params.cmd_vel_topic
        );
        Ok(Arc::new(Self { shared, commands, next_key: AtomicU64::new(1), thread: Some(thread), tasks }))
    }

    pub fn readiness(&self) -> Readiness {
        if !self.shared.alive.load(Ordering::Acquire) {
            return Readiness::default();
        }
        let poses = self.shared.poses();
        let pose = poses.map_from_odom.is_some() && poses.odom_pose.is_some();
        let timeout = (self.shared.params.odom_timeout_s * 1e9) as i64;
        lock(&self.shared.inputs).readiness(wall_ns(), pose, timeout)
    }

    /// A `navigate_to_pose` goal (pose in `map`). Taken up at the thread's
    /// next tick (≤ 10 ms); every goal is accepted. A goal sent while one
    /// runs preempts it, as bt_navigator does (the old one ends `Aborted`,
    /// code 0).
    pub fn send_navigate(&self, goal: Pose2D) -> GoalHandle {
        let key = GoalKey::Navigate(self.next_key.fetch_add(1, Ordering::Relaxed));
        let (handle, sinks) = channels(key);
        self.submit(key, Command::Navigate { key, pose: goal, sinks });
        handle
    }

    /// A `follow_path` goal straight to the controller server (poses in
    /// `map`, as mower_nav densifies them). A goal sent while one runs
    /// preempts it at the controller's next pass (the old one ends
    /// `Aborted`, code 0), as controller_server does.
    pub fn send_follow_path(&self, path: Vec<Pose2D>) -> GoalHandle {
        let key = GoalKey::Follow(self.next_key.fetch_add(1, Ordering::Relaxed));
        let (handle, sinks) = channels(key);
        self.submit(key, Command::Follow { key, path, sinks });
        handle
    }

    /// Asks the goal to cancel. `false`: the key is unknown or the goal has
    /// already ended (its outcome is, or will be, on its `result`). The
    /// `Canceled` outcome arrives on `result` once the server has stopped
    /// (one controller pass, ≤ 50 ms).
    pub fn cancel(&self, key: GoalKey) -> bool {
        if !lock(&self.shared.live).contains(&key) {
            return false;
        }
        self.commands.send(Command::Cancel(key)).is_ok()
    }

    /// Warnings since `since`, oldest first (at most the last 40): tolerated
    /// NoValidControl, costmap waits and timeouts, cleared costmaps, planner
    /// overruns, stale inputs. Fills mower_nav's "recent Nav2 log" summary.
    /// Each is `[server] message`, the server being the nav2 node that would
    /// have logged it; one repeated back to back ends in `(xN)`.
    pub fn recent_events(&self, since: Instant) -> Vec<String> {
        lock(&self.shared.events).iter().filter(|(at, _)| *at >= since).map(|(_, text)| text.clone()).collect()
    }

    /// Time since the thread last completed a tick: a stalled runtime shows
    /// here (mower_nav's cancel timeout then latches "uncertain"). Zero while
    /// the thread waits with nothing to run.
    pub fn last_tick_age(&self) -> Duration {
        if self.shared.idle.load(Ordering::Acquire) {
            return Duration::ZERO;
        }
        lock(&self.shared.last_tick).elapsed()
    }

    fn submit(&self, key: GoalKey, command: Command) {
        lock(&self.shared.live).insert(key);
        if self.commands.send(command).is_err() {
            // the thread is gone; the goal's channels went with the command,
            // so its result errors at once
            lock(&self.shared.live).remove(&key);
        }
    }
}

impl Drop for NavRuntime {
    fn drop(&mut self) {
        for task in &self.tasks {
            task.abort();
        }
        let _ = self.commands.send(Command::Stop);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

// --- shared state --------------------------------------------------------------

/// What the subscription tasks, the thread and the API share. Every lock is
/// held for a copy or a swap, never across a tick.
struct Shared {
    params: CoreParams,
    inputs: Mutex<Inputs>,
    /// `PoseSource::TfTopic`'s transforms.
    topic_tf: Option<Mutex<TopicTf>>,
    /// Goals sent whose outcome has not been delivered.
    live: Mutex<HashSet<GoalKey>>,
    /// The loop's event ring, rendered, with when each happened.
    events: Mutex<Vec<(Instant, String)>>,
    last_tick: Mutex<Instant>,
    /// The thread waits for a command with nothing to run.
    idle: AtomicBool,
    /// Cleared when the thread ends, by return or panic.
    alive: AtomicBool,
}

/// The robot pose as the loop and readiness read it.
#[derive(Debug, Clone, Copy, Default)]
struct Poses {
    map_from_odom: Option<Transform2D>,
    odom_pose: Option<Pose2D>,
    /// Since the older of the two dynamic edges was updated.
    age: Option<Duration>,
}

impl Shared {
    /// `map <- odom` and `odom <- base`, the latest of each.
    fn poses(&self) -> Poses {
        let p = &self.params;
        let (m, o, b) = (p.map_frame.as_str(), p.odom_frame.as_str(), p.base_frame.as_str());
        let (map_from_odom, odom_from_base, set_at) = match &self.topic_tf {
            None => (tfbus::lookup(m, o), tfbus::lookup(o, b), [tfbus::last_set(m, o), tfbus::last_set(o, b)]),
            Some(tf) => {
                let tf = lock(tf);
                (tf.graph.lookup(m, o), tf.graph.lookup(o, b), [tf.received(m, o), tf.received(o, b)])
            }
        };
        Poses {
            map_from_odom: map_from_odom.map(|t| planar(&t)),
            odom_pose: odom_from_base.map(|t| {
                let t = planar(&t);
                Pose2D::new(t.x, t.y, t.yaw)
            }),
            age: set_at.into_iter().flatten().min().map(|at| at.elapsed()),
        }
    }
}

/// `/tf` and `/tf_static` for `PoseSource::TfTopic`: mower_localize's
/// graph (latest edge wins, as a `Time(0)` lookup), and when each `/tf` edge
/// last arrived (static edges do not age).
#[derive(Default)]
struct TopicTf {
    graph: TfGraph,
    received: HashMap<(String, String), Instant>,
}

impl TopicTf {
    fn set(&mut self, parent: &str, child: &str, transform: Transform, at: Option<Instant>) {
        self.graph.set(parent, child, transform);
        let key = (parent.to_string(), child.to_string());
        match at {
            Some(at) => {
                self.received.insert(key, at);
            }
            None => {
                self.received.remove(&key);
            }
        }
    }

    fn received(&self, parent: &str, child: &str) -> Option<Instant> {
        self.received.get(&(parent.to_string(), child.to_string())).copied()
    }
}

/// A poisoned lock still holds plain data; a panicking writer cannot leave it
/// half-written in a way that matters here.
fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

fn latched() -> QosProfile {
    QosProfile::default().keep_last(1).reliable().transient_local()
}

fn wall_ns() -> i64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.as_nanos() as i64)
}

fn stamp(ns: i64) -> Time {
    Time { sec: ns.div_euclid(SEC) as i32, nanosec: ns.rem_euclid(SEC) as u32 }
}

/// x, y and `tf2::getYaw` of a transform.
fn planar(t: &Transform) -> Transform2D {
    let q = t.rotation();
    Transform2D::new(t.origin.x, t.origin.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
}

fn transform_in(t: &RTransform) -> Transform {
    let (r, v) = (&t.rotation, &t.translation);
    Transform::from_parts(&Quaternion::new(r.x, r.y, r.z, r.w), Vector3::new(v.x, v.y, v.z))
}

/// The static layer reads the origin's position and the data; the
/// resolution is a `float32`, widened (not reparsed).
fn grid_in(msg: ROccupancyGrid) -> OccupancyGrid {
    let i = &msg.info;
    OccupancyGrid {
        width: i.width,
        height: i.height,
        resolution: i.resolution as f64,
        origin_x: i.origin.position.x,
        origin_y: i.origin.position.y,
        data: msg.data,
    }
}

fn spawn_map(
    shared: Arc<Shared>, mut stream: impl Stream<Item = ROccupancyGrid> + Unpin + Send + 'static, global: bool,
) -> tokio::task::JoinHandle<()> {
    tokio::spawn(async move {
        while let Some(msg) = stream.next().await {
            let grid = grid_in(msg);
            let mut inputs = lock(&shared.inputs);
            if global {
                inputs.set_global_map(grid);
            } else {
                inputs.set_local_map(grid);
            }
        }
    })
}

fn spawn_tf(
    shared: Arc<Shared>, mut stream: impl Stream<Item = TFMessage> + Unpin + Send + 'static, statics: bool,
) -> tokio::task::JoinHandle<()> {
    tokio::spawn(async move {
        while let Some(msg) = stream.next().await {
            let Some(tf) = &shared.topic_tf else { return };
            let at = (!statics).then(Instant::now);
            let mut tf = lock(tf);
            for t in &msg.transforms {
                tf.set(&t.header.frame_id, &t.child_frame_id, transform_in(&t.transform), at);
            }
        }
    })
}

// --- the thread ------------------------------------------------------------------

enum Command {
    Navigate { key: GoalKey, pose: Pose2D, sinks: Sinks },
    Follow { key: GoalKey, path: Vec<Pose2D>, sinks: Sinks },
    Cancel(GoalKey),
    Stop,
}

/// The sending halves of a `GoalHandle`; dropped when the goal ends, which
/// closes its feedback channel.
struct Sinks {
    result: tokio::sync::oneshot::Sender<GoalOutcome>,
    feedback: tokio::sync::mpsc::UnboundedSender<CoreFeedback>,
}

fn channels(key: GoalKey) -> (GoalHandle, Sinks) {
    let (result_tx, result) = tokio::sync::oneshot::channel();
    let (feedback_tx, feedback) = tokio::sync::mpsc::unbounded_channel();
    (GoalHandle { key, result, feedback }, Sinks { result: result_tx, feedback: feedback_tx })
}

/// Logs the first failed publish on each topic.
fn warn_once(failed: &mut HashSet<String>, logger: &str, topic: &str, r: Result<(), r2r::Error>) {
    if let Err(e) = r {
        if failed.insert(topic.to_string()) {
            r2r::log_warn!(logger, "nav_core: publishing on {topic} failed: {e}");
        }
    }
}

/// Clears `alive` when the thread ends, also by panic.
struct Alive(Arc<Shared>);

impl Drop for Alive {
    fn drop(&mut self) {
        self.0.alive.store(false, Ordering::Release);
    }
}

/// The thread's side: the command queue, the publishers, the goals' sinks.
struct Io {
    shared: Arc<Shared>,
    rx: mpsc::Receiver<Command>,
    cmd_pub: r2r::Publisher<TwistStamped>,
    plan_pub: r2r::Publisher<Path>,
    logger: String,
    sinks: HashMap<GoalKey, Sinks>,
    /// The newest event already logged.
    logged_seq: u64,
    events_revision: u64,
    /// Topics whose publish failed (logged once each).
    publish_failed: HashSet<String>,
    /// When a slow tick was last reported, per server.
    slow_reported: HashMap<&'static str, Instant>,
}

impl Io {
    fn run(mut self, mut core: CoreLoop) {
        let _alive = Alive(self.shared.clone());
        let mut woken_by = None;
        loop {
            let now = wall_ns();
            // maps first: one that came before a goal is the goal's map
            self.take_maps(&mut core, now);
            if !self.take_commands(&mut core, woken_by.take()) {
                return;
            }
            let input = self.inputs();
            let started = Instant::now();
            let out = core.tick(now, &input);
            let took = started.elapsed();
            if took > SLOW_TICK {
                // a planner pass is what usually takes this long (NavFn on a
                // big map); the controller waits for it on this thread
                let (server, which) = if out.plans.is_empty() { ("controller_server", "Control") } else { ("planner_server", "Planner") };
                if self.slow_reported.get(server).is_none_or(|at| at.elapsed() >= SLOW_TICK_REPORT) {
                    self.slow_reported.insert(server, Instant::now());
                    let hz = 1.0 / SLOW_TICK.as_secs_f64();
                    let rate = 1.0 / took.as_secs_f64();
                    core.record(now, format!("[{server}] {which} loop missed its desired rate of {hz:.4} Hz. Current loop rate is {rate:.4} Hz"));
                }
            }
            self.publish(&out, now);
            self.deliver(out);
            self.sync_events(&core);
            *lock(&self.shared.last_tick) = Instant::now();

            woken_by = match core.next_due_ns() {
                Some(due) => {
                    let wait = Duration::from_nanos((due - wall_ns()).max(0) as u64).min(MAX_SLEEP);
                    match self.rx.recv_timeout(wait) {
                        Ok(command) => Some(command),
                        Err(mpsc::RecvTimeoutError::Timeout) => None,
                        Err(mpsc::RecvTimeoutError::Disconnected) => return,
                    }
                }
                None => {
                    self.shared.idle.store(true, Ordering::Release);
                    let command = self.rx.recv();
                    // the age counts from the wake-up, not from before the wait
                    *lock(&self.shared.last_tick) = Instant::now();
                    self.shared.idle.store(false, Ordering::Release);
                    match command {
                        Ok(command) => Some(command),
                        Err(_) => return,
                    }
                }
            };
        }
    }

    /// Takes up every queued command; `false` once the runtime is dropped.
    fn take_commands(&mut self, core: &mut CoreLoop, first: Option<Command>) -> bool {
        let mut next = first;
        loop {
            let command = match next.take() {
                Some(command) => command,
                None => match self.rx.try_recv() {
                    Ok(command) => command,
                    Err(mpsc::TryRecvError::Empty) => return true,
                    Err(mpsc::TryRecvError::Disconnected) => return false,
                },
            };
            match command {
                Command::Navigate { key, pose, sinks } => {
                    core.send_navigate(key, pose);
                    self.sinks.insert(key, sinks);
                }
                Command::Follow { key, path, sinks } => {
                    core.send_follow_path(key, path);
                    self.sinks.insert(key, sinks);
                }
                Command::Cancel(key) => {
                    core.cancel(key);
                }
                Command::Stop => return false,
            }
        }
    }

    /// Maps that arrived since the last tick, into the loop.
    fn take_maps(&mut self, core: &mut CoreLoop, now: i64) {
        let (global, local, notes) = {
            let mut inputs = lock(&self.shared.inputs);
            (inputs.global_map.take(), inputs.local_map.take(), std::mem::take(&mut inputs.notes))
        };
        for note in notes {
            core.record(now, note);
        }
        if let Some(grid) = global {
            core.set_global_map(now, &grid);
        }
        if let Some(grid) = local {
            core.set_local_map(now, &grid);
        }
    }

    /// The latest transforms and twist.
    fn inputs(&self) -> TickInput {
        let poses = self.shared.poses();
        TickInput {
            map_from_odom: poses.map_from_odom,
            odom_pose: poses.odom_pose,
            pose_age_ns: poses.age.map(|a| a.as_nanos() as i64),
            odom: lock(&self.shared.inputs).odom,
        }
    }

    fn publish(&mut self, out: &TickOutput, now: i64) {
        let p = &self.shared.params;
        for c in &out.cmd_vel {
            let msg = TwistStamped {
                header: Header { stamp: stamp(c.stamp_ns), frame_id: c.frame_id.clone() },
                twist: Twist {
                    linear: RVector3 { x: c.twist.x, y: c.twist.y, z: 0.0 },
                    angular: RVector3 { x: 0.0, y: 0.0, z: c.twist.theta },
                },
            };
            warn_once(&mut self.publish_failed, &self.logger, &p.cmd_vel_topic, self.cmd_pub.publish(&msg));
        }
        for plan in &out.plans {
            let header = Header { stamp: stamp(now), frame_id: p.map_frame.clone() };
            let poses = plan
                .iter()
                .map(|pose| {
                    let (x, y, z, w) = quaternion_from_yaw(pose.yaw);
                    PoseStamped {
                        header: header.clone(),
                        pose: Pose { position: Point { x: pose.x, y: pose.y, z: 0.0 }, orientation: RQuaternion { x, y, z, w } },
                    }
                })
                .collect();
            warn_once(&mut self.publish_failed, &self.logger, &p.plan_topic, self.plan_pub.publish(&Path { header, poses }));
        }
    }

    fn deliver(&mut self, out: TickOutput) {
        for (key, feedback) in out.feedback {
            if let Some(sinks) = self.sinks.get(&key) {
                let _ = sinks.feedback.send(feedback);
            }
        }
        for (key, outcome) in out.outcomes {
            // no longer cancelable by the time the outcome can be seen
            lock(&self.shared.live).remove(&key);
            if let Some(sinks) = self.sinks.remove(&key) {
                let _ = sinks.result.send(outcome);
            }
        }
    }

    /// Mirrors the event ring for `recent_events`, and logs each new entry
    /// (repeats are only counted).
    fn sync_events(&mut self, core: &CoreLoop) {
        if core.events_revision() == self.events_revision {
            return;
        }
        self.events_revision = core.events_revision();
        let logged = self.logged_seq;
        for e in core.events().iter().filter(|e| e.seq > logged) {
            r2r::log_warn!(&self.logger, "{}", e.text);
            self.logged_seq = e.seq;
        }
        let (now, now_ns) = (Instant::now(), core.now_ns().unwrap_or_default());
        let events = core
            .events()
            .iter()
            .map(|e| {
                let ago = Duration::from_nanos((now_ns - e.at_ns).max(0) as u64);
                (now.checked_sub(ago).unwrap_or(now), e.render())
            })
            .collect();
        *lock(&self.shared.events) = events;
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::FRAC_PI_2;

    use r2r::nav_msgs::msg::MapMetaData;

    use super::*;

    #[test]
    fn tf_topic_edges_compose_and_only_dynamic_ones_age() {
        let mut tf = TopicTf::default();
        let edge = |x: f64, yaw: f64| Transform::from_parts(&Quaternion::from_rpy(0.0, 0.0, yaw), Vector3::new(x, 0.0, 0.0));
        tf.set("map", "odom", edge(10.0, FRAC_PI_2), None);
        let now = Instant::now();
        tf.set("odom", "base_footprint", edge(1.0, 0.0), Some(now));
        let map_from_odom = planar(&tf.graph.lookup("map", "odom").unwrap());
        assert!((map_from_odom.x - 10.0).abs() < 1e-12 && (map_from_odom.yaw - FRAC_PI_2).abs() < 1e-12);
        let base = planar(&tf.graph.lookup("map", "base_footprint").unwrap());
        assert!((base.x - 10.0).abs() < 1e-12 && (base.y - 1.0).abs() < 1e-12, "{base:?}");
        assert_eq!(tf.received("odom", "base_footprint"), Some(now));
        assert_eq!(tf.received("map", "odom"), None);
    }

    #[test]
    fn stamps_split_into_seconds_and_nanoseconds() {
        let t = stamp(1_792_900_000_123_456_789);
        assert_eq!((t.sec, t.nanosec), (1_792_900_000, 123_456_789));
    }

    fn grid_msg(frame: &str) -> ROccupancyGrid {
        // 6 x 4 m at 0.05 m from (-1, -1), free inside a lethal border
        let (w, h) = (120u32, 80u32);
        let data = (0..h).flat_map(|j| (0..w).map(move |i| if i == 0 || j == 0 || i == w - 1 || j == h - 1 { 100 } else { 0 })).collect();
        let mut info = MapMetaData { resolution: 0.05, width: w, height: h, ..MapMetaData::default() };
        info.origin.position.x = -1.0;
        info.origin.position.y = -1.0;
        info.origin.orientation.w = 1.0;
        ROccupancyGrid { header: Header { stamp: stamp(wall_ns()), frame_id: frame.into() }, info, data }
    }

    struct Robot {
        pose: Pose2D,
        twist: Twist2D,
        stamps: Vec<i64>,
        frames: HashSet<String>,
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn a_follow_path_round_trip_over_ros_with_the_pose_from_the_tfbus() {
        round_trip(PoseSource::TfBus, "tfbus").await;
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn a_follow_path_round_trip_over_ros_with_the_pose_from_tf() {
        round_trip(PoseSource::TfTopic, "tf").await;
    }

    /// The whole round trip on DDS: maps and `/odom` in, the pose on the
    /// tfbus or on `/tf` (dynamic only, no `/tf_static`), a kinematic robot
    /// driven by `/nav_cmd_vel`. Needs a ROS environment; returns early
    /// without one, like the navsat test.
    async fn round_trip(source: PoseSource, name: &str) {
        let ctx = match r2r::Context::create() {
            Ok(c) => c,
            Err(_) => return,
        };
        // topics and frames of its own: the tfbus is process-global, and
        // the graph (with its /tf) may be shared
        let ns = format!("/nav_core_runtime_test_{name}");
        let params = CoreParams {
            pose_source: source,
            odom_topic: format!("{ns}/odom"),
            map_topic: format!("{ns}/map_grid"),
            global_map_topic: format!("{ns}/map_grid_global"),
            cmd_vel_topic: format!("{ns}/nav_cmd_vel"),
            plan_topic: format!("{ns}/plan"),
            map_frame: format!("rt_test_{name}_map"),
            odom_frame: format!("rt_test_{name}_odom"),
            base_frame: format!("rt_test_{name}_base"),
            odom_timeout_s: 0.5,
        };
        let mut node = r2r::Node::create(ctx.clone(), &format!("nav_core_runtime_test_{name}"), "").unwrap();
        let logger = node.logger().to_string();
        let rt = NavRuntime::start(&mut node, params.clone(), &logger).unwrap();
        assert!(!rt.readiness().ready());

        let mut feeder = r2r::Node::create(ctx, &format!("nav_core_runtime_feeder_{name}"), "").unwrap();
        let map_pub = feeder.create_publisher::<ROccupancyGrid>(&params.map_topic, latched()).unwrap();
        let global_pub = feeder.create_publisher::<ROccupancyGrid>(&params.global_map_topic, latched()).unwrap();
        let odom_pub = feeder.create_publisher::<Odometry>(&params.odom_topic, QosProfile::default()).unwrap();
        let tf_pub = feeder.create_publisher::<TFMessage>("/tf", QosProfile::default().keep_last(100)).unwrap();
        let mut cmd = feeder.subscribe::<TwistStamped>(&params.cmd_vel_topic, QosProfile::default()).unwrap();
        map_pub.publish(&grid_msg(&params.map_frame)).unwrap();
        global_pub.publish(&grid_msg(&params.map_frame)).unwrap();

        let running = Arc::new(AtomicBool::new(true));
        let spin = {
            let running = running.clone();
            tokio::task::spawn_blocking(move || {
                while running.load(Ordering::Relaxed) {
                    node.spin_once(Duration::from_millis(5));
                    feeder.spin_once(Duration::from_millis(5));
                }
            })
        };
        let robot = Arc::new(Mutex::new(Robot { pose: Pose2D::new(1.0, 1.0, 0.0), twist: Twist2D::default(), stamps: Vec::new(), frames: HashSet::new() }));
        let listener = {
            let robot = robot.clone();
            tokio::spawn(async move {
                while let Some(m) = cmd.next().await {
                    let mut r = lock(&robot);
                    r.twist = Twist2D::new(m.twist.linear.x, 0.0, m.twist.angular.z);
                    r.stamps.push(m.header.stamp.sec as i64 * SEC + m.header.stamp.nanosec as i64);
                    r.frames.insert(m.header.frame_id.clone());
                }
            })
        };
        // 50 Hz: move, broadcast the pose, report the twist
        let mover = {
            let (robot, running, params) = (robot.clone(), running.clone(), params.clone());
            tokio::spawn(async move {
                let mut tick = tokio::time::interval(Duration::from_millis(20));
                while running.load(Ordering::Relaxed) {
                    tick.tick().await;
                    let (pose, twist) = {
                        let mut r = lock(&robot);
                        let (s, c) = r.pose.yaw.sin_cos();
                        let (v, w) = (r.twist.x, r.twist.theta);
                        r.pose = Pose2D::new(r.pose.x + v * c * 0.02, r.pose.y + v * s * 0.02, r.pose.yaw + w * 0.02);
                        (r.pose, r.twist)
                    };
                    match source {
                        PoseSource::TfBus => {
                            tfbus::set(&params.map_frame, &params.odom_frame, Transform::identity());
                            let base = Transform::from_parts(&Quaternion::from_rpy(0.0, 0.0, pose.yaw), Vector3::new(pose.x, pose.y, 0.0));
                            tfbus::set(&params.odom_frame, &params.base_frame, base);
                        }
                        PoseSource::TfTopic => {
                            let edge = |parent: &str, child: &str, p: Pose2D| {
                                let (x, y, z, w) = quaternion_from_yaw(p.yaw);
                                r2r::geometry_msgs::msg::TransformStamped {
                                    header: Header { stamp: stamp(wall_ns()), frame_id: parent.into() },
                                    child_frame_id: child.into(),
                                    transform: RTransform {
                                        translation: RVector3 { x: p.x, y: p.y, z: 0.0 },
                                        rotation: RQuaternion { x, y, z, w },
                                    },
                                }
                            };
                            let transforms = vec![
                                edge(&params.map_frame, &params.odom_frame, Pose2D::default()),
                                edge(&params.odom_frame, &params.base_frame, pose),
                            ];
                            let _ = tf_pub.publish(&TFMessage { transforms });
                        }
                    }
                    let mut odom = Odometry::default();
                    odom.header.stamp = stamp(wall_ns());
                    odom.twist.twist.linear.x = twist.x;
                    odom.twist.twist.angular.z = twist.theta;
                    let _ = odom_pub.publish(&odom);
                }
            })
        };

        tokio::time::timeout(Duration::from_secs(10), async {
            while !rt.readiness().ready() {
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
        })
        .await
        .unwrap_or_else(|_| panic!("never ready: {:?}", rt.readiness()));

        let path: Vec<Pose2D> = (0..=10).map(|i| Pose2D::new(1.0 + 0.1 * i as f64, 1.0, 0.0)).collect();
        let mut goal = rt.send_follow_path(path);
        let outcome = tokio::time::timeout(Duration::from_secs(20), &mut goal.result).await.expect("ended in time").expect("an outcome");
        assert_eq!(outcome, GoalOutcome { status: GoalStatus::Succeeded, error_code: 0, error_msg: String::new() });
        // the feedback channel closes with the goal
        let feedbacks = tokio::time::timeout(Duration::from_secs(1), async {
            let mut n = 0;
            while goal.feedback.recv().await.is_some() {
                n += 1;
            }
            n
        })
        .await
        .expect("the feedback channel closed");
        assert!(feedbacks > 10, "{feedbacks} feedbacks");
        assert!(!rt.cancel(goal.key), "an ended goal cannot be canceled");
        assert!(rt.last_tick_age() < Duration::from_millis(500));
        {
            let r = lock(&robot);
            assert!((r.pose.x - 2.0).abs() < 0.1 && (r.pose.y - 1.0).abs() < 0.05, "ended at {:?}", r.pose);
            assert!(r.stamps.len() > 50 && r.stamps.windows(2).all(|w| w[0] <= w[1]));
            assert_eq!(r.frames, HashSet::from([params.base_frame.clone()]));
        }

        // a goal still running when the runtime goes ends without an outcome
        let mut long = rt.send_follow_path((0..=20).map(|i| Pose2D::new(2.0 + 0.1 * i as f64, 1.0, 0.0)).collect());
        tokio::time::sleep(Duration::from_millis(300)).await;
        assert!(long.result.try_recv().is_err_and(|e| e == tokio::sync::oneshot::error::TryRecvError::Empty));
        drop(rt);
        assert!(long.result.await.is_err());
        assert!(long.feedback.recv().await.is_some(), "the feedback it had");
        while long.feedback.recv().await.is_some() {}

        running.store(false, Ordering::Relaxed);
        listener.abort();
        let _ = mover.await;
        let _ = spin.await;
    }
}
