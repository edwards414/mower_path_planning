//! In-process navigation (docs/ROS_FREE_PLAN.md section 8, D6):
//! mower_nav_core's `NavStack` (navigator + planner server + controller
//! server) and `VelocitySmoother` on dedicated threads, in place of nav2's
//! `navigate_to_pose` / `follow_path` actions and `bt_navigator/get_state`.
//!
//! Selected with the node parameter `nav_backend: nav_core`
//! (`RUST_NAV_CORE=true`, which also stops nav2 from being launched);
//! `nav_backend: nav2`, the default, keeps the r2r action clients.
//!
//! What the runtime does (two `std::thread`s, "nav-core" and "nav-planner",
//! not the shared tokio workers):
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
//! The work is pure and runs on an explicit clock: [`core_loop`] on nav-core
//! (the tree, the controller server, the local costmap, the smoother, the
//! goals and the events) and [`planner_loop`] on nav-planner (planner_server
//! and the global costmap), the split upstream has, so a NavFn search on a
//! big zone map (up to ~0.4 s on the RK3568) never holds up a controller pass
//! or a smoother output. This file is their IO:
//!
//! * the topics are on the module's bus (step E2 of docs/ROS_FREE_PLAN.md),
//!   under the names rcl gives the `nav_core.*` parameters on the node; the
//!   bus/DDS bridge carries what crosses the process boundary;
//! * the subscriptions are drained by tokio tasks: `/odom` and `/map_grid`
//!   into nav-core's shared inputs, `/map_grid_global` onto the planner's
//!   channel, `/tf` into a transform graph. The transforms are read where
//!   they live (the tfbus, or that graph), by each thread when it needs them;
//! * nav-core loops: take the new local map, the queued commands (goals,
//!   cancels, what the planner sent) and the latest transforms and twist,
//!   tick at `SystemTime::now()`, publish, send the planner its requests and
//!   clears and the goals their outcomes and feedback, then sleep until it is
//!   next due (at most 10 ms), or block on its channel while nothing runs.
//!   The publishers live on it (a bus publisher publishes from any thread);
//! * nav-planner loops the same way on its own channel: maps, clears and
//!   requests in, a pass when due with the robot pose of that moment, results
//!   and plans back onto nav-core's channel.
//!
//! The API never waits for either thread: a goal or a cancel is a message on
//! an unbounded channel, and readiness, the events and the tick age are read
//! from state the threads only lock for a moment. A stalled nav-core shows in
//! [`NavRuntime::last_tick_age`]; a thread that panicked is logged and takes
//! readiness down.

use std::collections::{HashMap, HashSet};
use std::panic::AssertUnwindSafe;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{mpsc, Arc, Mutex, MutexGuard, PoisonError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use futures::{Stream, StreamExt};
use mower_bus::{BusError, Qos};
use mower_localize::tfbus;
use mower_localize::tfgraph::TfGraph;
use mower_localize_core::tf::{Quaternion, Transform, Vector3};
pub use mower_nav_core::action::GoalStatus;
use mower_nav_core::costmap::OccupancyGrid;
use mower_nav_core::geometry::{quaternion_from_yaw, yaw_from_quaternion, Pose2D, Transform2D, Twist2D};
use mower_nav_core::nav_stack::StackLog;
use mower_nav_core::smoother::TwistStamped as StampedTwist;
use mower_rs_common::{BoxError, ModuleCtx};
use r2r::builtin_interfaces::msg::Time;
use r2r::geometry_msgs::msg::{Point, Pose, PoseStamped, Quaternion as RQuaternion, Transform as RTransform, Twist, TwistStamped, Vector3 as RVector3};
use r2r::nav_msgs::msg::{OccupancyGrid as ROccupancyGrid, Odometry, Path};
use r2r::std_msgs::msg::Header;
use r2r::tf2_msgs::msg::TFMessage;

pub mod core_loop;
pub mod planner_loop;

use core_loop::{CoreLoop, Inputs, LoopParams, TickInput, TickOutput, SEC};
use planner_loop::{FromPlanner, PlannerLoop, ToPlanner};

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


/// The longest a thread sleeps while it has something to run.
const MAX_SLEEP: Duration = Duration::from_millis(10);
/// A nav-core tick that takes longer than a controller period is reported,
/// and so is a plan that takes longer than `expected_planner_frequency`'s
/// period (also 50 ms), each at most once per `SLOW_TICK_REPORT`: on a big
/// map every replan is that slow.
const SLOW_TICK: Duration = Duration::from_millis(50);
const SLOW_TICK_REPORT: Duration = Duration::from_secs(10);

/// The running stack. Dropping it stops the threads; goals still running end
/// without an outcome (their `result` receivers error).
pub struct NavRuntime {
    shared: Arc<Shared>,
    /// nav-core's channel.
    commands: mpsc::Sender<Command>,
    /// nav-planner's channel.
    planner_tx: mpsc::Sender<PlannerMsg>,
    next_key: AtomicU64,
    threads: Vec<JoinHandle<()>>,
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
    /// Creates the subscriptions and publishers on the module's bus scope
    /// `m.bus`, under the names rcl gives them on the node, and starts the
    /// threads. `logger` is the node's logger name. Call it inside a tokio
    /// runtime (the subscriptions are drained by tokio tasks).
    pub fn start(m: &ModuleCtx, params: CoreParams, logger: &str) -> Result<Arc<Self>, BoxError> {
        let topic = |name: &str| m.resolve_topic(name);
        // reliable, depth 10: twist_mux subscribes reliable
        let cmd_pub = m.bus.publisher::<TwistStamped>(&topic(&params.cmd_vel_topic)?, Qos::default())?;
        let plan_pub = m.bus.publisher::<Path>(&topic(&params.plan_topic)?, Qos::default().keep_last(1))?;
        let odom = m.bus.subscribe::<Odometry>(&topic(&params.odom_topic)?, Qos::default())?;
        let local_map = m.bus.subscribe::<ROccupancyGrid>(&topic(&params.map_topic)?, latched())?;
        let global_map = m.bus.subscribe::<ROccupancyGrid>(&topic(&params.global_map_topic)?, latched())?;
        let tf = match params.pose_source {
            PoseSource::TfBus => None,
            PoseSource::TfTopic => Some((
                m.bus.subscribe::<TFMessage>(&topic("/tf")?, Qos::default().keep_last(100))?,
                m.bus.subscribe::<TFMessage>(&topic("/tf_static")?, Qos::default().keep_last(100).reliable().transient_local())?,
            )),
        };

        let outputs = BusOutputs { cmd_pub, plan_pub, params: params.clone(), logger: logger.to_string(), failed: HashSet::new() };
        let mut rt = Self::spawn(params.clone(), logger, Box::new(outputs))?;
        let shared = rt.shared.clone();
        rt.tasks.push({
            let shared = shared.clone();
            let mut odom = odom;
            tokio::spawn(async move {
                while let Some(msg) = odom.next().await {
                    let t = &msg.twist.twist;
                    lock(&shared.inputs).set_odom(Twist2D::new(t.linear.x, t.linear.y, t.angular.z), wall_ns());
                }
            })
        });
        rt.tasks.push({
            let shared = shared.clone();
            let mut local_map = local_map;
            tokio::spawn(async move {
                while let Some(msg) = local_map.next().await {
                    lock(&shared.inputs).set_local_map(grid_in(Arc::unwrap_or_clone(msg)));
                }
            })
        });
        rt.tasks.push({
            let planner = rt.planner_tx.clone();
            let mut global_map = global_map;
            tokio::spawn(async move {
                while let Some(msg) = global_map.next().await {
                    if planner.send(PlannerMsg::Map(grid_in(Arc::unwrap_or_clone(msg)))).is_err() {
                        return;
                    }
                }
            })
        });
        if let Some((tf, tf_static)) = tf {
            rt.tasks.push(spawn_tf(shared.clone(), tf, false));
            rt.tasks.push(spawn_tf(shared, tf_static, true));
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
        Ok(Arc::new(rt))
    }

    /// The two threads, their output going to `outputs`; the inputs are
    /// whoever writes the shared state and the planner's channel.
    fn spawn(params: CoreParams, logger: &str, outputs: Box<dyn Outputs>) -> Result<Self, r2r::Error> {
        let shared = Arc::new(Shared {
            topic_tf: (params.pose_source == PoseSource::TfTopic).then(|| Mutex::new(TopicTf::default())),
            inputs: Mutex::new(Inputs::default()),
            live: Mutex::new(HashSet::new()),
            events: Mutex::new(Vec::new()),
            last_tick: Mutex::new(Instant::now()),
            idle: AtomicBool::new(false),
            alive: AtomicBool::new(true),
            planner_alive: AtomicBool::new(true),
            global_map: AtomicBool::new(false),
            params: params.clone(),
        });
        let core = CoreLoop::new(LoopParams {
            map_frame: params.map_frame.clone(),
            odom_frame: params.odom_frame.clone(),
            base_frame: params.base_frame.clone(),
            odom_timeout_ns: (params.odom_timeout_s * 1e9) as i64,
        });
        let (commands, rx) = mpsc::channel();
        let (planner_tx, planner_rx) = mpsc::channel();
        let io = Io {
            shared: shared.clone(),
            rx,
            planner_tx: planner_tx.clone(),
            outputs,
            logger: logger.to_string(),
            sinks: HashMap::new(),
            logged_seq: 0,
            events_revision: 0,
            last_slow_report: None,
            last_cmd: None,
        };
        let planner = PlannerIo { shared: shared.clone(), rx: planner_rx, core_tx: commands.clone(), last_slow_report: None };
        let core_thread = spawn_named("nav-core", logger, move || io.run(core))?;
        let planner_thread = match spawn_named("nav-planner", logger, move || planner.run(PlannerLoop::new())) {
            Ok(t) => t,
            Err(e) => {
                let _ = commands.send(Command::Stop);
                let _ = core_thread.join();
                return Err(e);
            }
        };
        Ok(Self { shared, commands, planner_tx, next_key: AtomicU64::new(1), threads: vec![core_thread, planner_thread], tasks: Vec::new() })
    }

    pub fn readiness(&self) -> Readiness {
        let s = &self.shared;
        if !(s.alive.load(Ordering::Acquire) && s.planner_alive.load(Ordering::Acquire)) {
            return Readiness::default();
        }
        let poses = s.poses();
        let pose = poses.map_from_odom.is_some() && poses.odom_pose.is_some();
        let timeout = (s.params.odom_timeout_s * 1e9) as i64;
        lock(&s.inputs).readiness(wall_ns(), pose, s.global_map.load(Ordering::Acquire), timeout)
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

    /// Why the runtime can no longer run goals: one of its threads has ended
    /// (while the runtime lives only a panic ends them). mower_nav then
    /// refuses new goals and returns, so mower_rsd restarts the nav module
    /// with a new runtime.
    pub fn stopped(&self) -> Option<&'static str> {
        let s = &self.shared;
        if !s.alive.load(Ordering::Acquire) {
            Some("the nav-core thread stopped")
        } else if !s.planner_alive.load(Ordering::Acquire) {
            Some("the nav-planner thread stopped")
        } else {
            None
        }
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
        let _ = self.planner_tx.send(PlannerMsg::Stop);
        // the planner finishes the plan it is on first
        for thread in self.threads.drain(..) {
            let _ = thread.join();
        }
    }
}

/// A named thread whose panic is logged (its guards then take readiness
/// down and close its goals' channels).
fn spawn_named(name: &str, logger: &str, f: impl FnOnce() + Send + 'static) -> Result<JoinHandle<()>, r2r::Error> {
    let (thread_name, logger) = (name.to_string(), logger.to_string());
    std::thread::Builder::new()
        .name(name.into())
        .spawn(move || {
            if let Err(e) = std::panic::catch_unwind(AssertUnwindSafe(f)) {
                let what = e.downcast_ref::<&str>().map(|s| s.to_string()).or_else(|| e.downcast_ref::<String>().cloned()).unwrap_or_default();
                r2r::log_error!(&logger, "nav_core: the {thread_name} thread panicked ({what}); navigation is stopped");
            }
        })
        .map_err(|_| r2r::Error::RCL_RET_ERROR)
}

// --- shared state --------------------------------------------------------------

/// What the subscription tasks, the threads and the API share. Every lock is
/// held for a copy or a swap, never across a tick or a plan.
struct Shared {
    params: CoreParams,
    inputs: Mutex<Inputs>,
    /// `PoseSource::TfTopic`'s transforms.
    topic_tf: Option<Mutex<TopicTf>>,
    /// Goals sent whose outcome has not been delivered.
    live: Mutex<HashSet<GoalKey>>,
    /// nav-core's event ring, rendered, with when each happened.
    events: Mutex<Vec<(Instant, String)>>,
    last_tick: Mutex<Instant>,
    /// nav-core waits for a command with nothing to run.
    idle: AtomicBool,
    /// Cleared when nav-core ends, by return or panic.
    alive: AtomicBool,
    /// Cleared when nav-planner ends.
    planner_alive: AtomicBool,
    /// nav-planner has a global costmap (readiness' `global_map`).
    global_map: AtomicBool,
}

/// The robot pose as the loops and readiness read it.
#[derive(Debug, Clone, Copy, Default)]
struct Poses {
    map_from_odom: Option<Transform2D>,
    odom_pose: Option<Pose2D>,
    /// Since the older of the two dynamic edges was updated.
    age: Option<Duration>,
}

impl Poses {
    /// `map <- base_footprint`.
    fn map_pose(&self) -> Option<Pose2D> {
        Some(self.map_from_odom?.apply_pose(&self.odom_pose?))
    }
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

fn latched() -> Qos {
    Qos::default().keep_last(1).reliable().transient_local()
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

fn spawn_tf(
    shared: Arc<Shared>, mut stream: impl Stream<Item = Arc<TFMessage>> + Unpin + Send + 'static, statics: bool,
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

// --- outputs -----------------------------------------------------------------

/// Where nav-core's output goes: `/nav_cmd_vel` and `/plan`, or a test's
/// recorder.
trait Outputs: Send {
    fn cmd_vel(&mut self, cmd: &StampedTwist);
    fn plan(&mut self, plan: &[Pose2D], stamp_ns: i64);
}

struct BusOutputs {
    cmd_pub: mower_bus::Publisher<TwistStamped>,
    plan_pub: mower_bus::Publisher<Path>,
    params: CoreParams,
    logger: String,
    /// Topics whose publish failed (logged once each).
    failed: HashSet<String>,
}

impl BusOutputs {
    fn warn_once(&mut self, topic: &str, r: Result<(), BusError>) {
        if let Err(e) = r {
            if self.failed.insert(topic.to_string()) {
                r2r::log_warn!(&self.logger, "nav_core: publishing on {topic} failed: {e}");
            }
        }
    }
}

impl Outputs for BusOutputs {
    fn cmd_vel(&mut self, c: &StampedTwist) {
        let msg = TwistStamped {
            header: Header { stamp: stamp(c.stamp_ns), frame_id: c.frame_id.clone() },
            twist: Twist {
                linear: RVector3 { x: c.twist.x, y: c.twist.y, z: 0.0 },
                angular: RVector3 { x: 0.0, y: 0.0, z: c.twist.theta },
            },
        };
        let r = self.cmd_pub.publish(&msg);
        let topic = self.params.cmd_vel_topic.clone();
        self.warn_once(&topic, r);
    }

    fn plan(&mut self, plan: &[Pose2D], stamp_ns: i64) {
        let header = Header { stamp: stamp(stamp_ns), frame_id: self.params.map_frame.clone() };
        let poses = plan
            .iter()
            .map(|p| {
                let (x, y, z, w) = quaternion_from_yaw(p.yaw);
                PoseStamped { header: header.clone(), pose: Pose { position: Point { x: p.x, y: p.y, z: 0.0 }, orientation: RQuaternion { x, y, z, w } } }
            })
            .collect();
        let r = self.plan_pub.publish(&Path { header, poses });
        let topic = self.params.plan_topic.clone();
        self.warn_once(&topic, r);
    }
}

// --- nav-core --------------------------------------------------------------------

enum Command {
    Navigate { key: GoalKey, pose: Pose2D, sinks: Sinks },
    Follow { key: GoalKey, path: Vec<Pose2D>, sinks: Sinks },
    Cancel(GoalKey),
    /// Results, plans and log lines from nav-planner.
    Planner(FromPlanner),
    /// nav-planner has ended.
    PlannerGone,
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

/// Clears `alive` when nav-core ends, also by panic.
struct Alive(Arc<Shared>);

impl Drop for Alive {
    fn drop(&mut self) {
        self.0.alive.store(false, Ordering::Release);
    }
}

/// nav-core's side: its channel, the planner's, the outputs, the goals' sinks.
struct Io {
    shared: Arc<Shared>,
    rx: mpsc::Receiver<Command>,
    planner_tx: mpsc::Sender<PlannerMsg>,
    outputs: Box<dyn Outputs>,
    logger: String,
    sinks: HashMap<GoalKey, Sinks>,
    /// The newest event already logged.
    logged_seq: u64,
    events_revision: u64,
    last_slow_report: Option<Instant>,
    /// The last command published on `/nav_cmd_vel`.
    last_cmd: Option<StampedTwist>,
}

/// nav-core ending (dropped, stopped or by panic) right after telling the
/// robot to move: one zero, so the drive does not run on that command until
/// velocity_command_guard's 0.2 s watchdog. Never stamped before the last.
fn final_zero(last: Option<&StampedTwist>, now_ns: i64) -> Option<StampedTwist> {
    let last = last?;
    (last.twist != Twist2D::default())
        .then(|| StampedTwist { stamp_ns: now_ns.max(last.stamp_ns), twist: Twist2D::default(), other: [0.0; 3], ..last.clone() })
}

impl Drop for Io {
    fn drop(&mut self) {
        if let Some(zero) = final_zero(self.last_cmd.as_ref(), wall_ns()) {
            self.outputs.cmd_vel(&zero);
        }
    }
}

impl Io {
    fn run(mut self, mut core: CoreLoop) {
        let _alive = Alive(self.shared.clone());
        let mut woken_by = None;
        loop {
            let now = wall_ns();
            self.take_local_map(&mut core, now);
            if !self.take_commands(&mut core, woken_by.take(), now) {
                return;
            }
            let input = self.inputs();
            let started = Instant::now();
            let mut out = core.tick(now, &input);
            let took = started.elapsed();
            if took > SLOW_TICK && self.last_slow_report.is_none_or(|at| at.elapsed() >= SLOW_TICK_REPORT) {
                self.last_slow_report = Some(Instant::now());
                let (hz, rate) = (1.0 / SLOW_TICK.as_secs_f64(), 1.0 / took.as_secs_f64());
                core.record(now, format!("[controller_server] Control loop missed its desired rate of {hz:.4} Hz. Current loop rate is {rate:.4} Hz"));
            }
            for msg in out.to_planner.drain(..) {
                // a planner that is gone has said so (Command::PlannerGone)
                let _ = self.planner_tx.send(PlannerMsg::Core(msg));
            }
            for c in &out.cmd_vel {
                self.outputs.cmd_vel(c);
            }
            if let Some(c) = out.cmd_vel.last() {
                self.last_cmd = Some(c.clone());
            }
            for plan in &out.plans {
                self.outputs.plan(plan, now);
            }
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
    fn take_commands(&mut self, core: &mut CoreLoop, first: Option<Command>, now: i64) -> bool {
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
                Command::Planner(from) => core.on_planner(now, from),
                Command::PlannerGone => core.planner_gone(now),
                Command::Stop => return false,
            }
        }
    }

    /// A local map that arrived since the last tick, and the rejected ones.
    fn take_local_map(&mut self, core: &mut CoreLoop, now: i64) {
        let (local, notes) = {
            let mut inputs = lock(&self.shared.inputs);
            (inputs.local_map.take(), std::mem::take(&mut inputs.notes))
        };
        for note in notes {
            core.record(now, note);
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

// --- nav-planner -------------------------------------------------------------------

enum PlannerMsg {
    /// A `/map_grid_global`.
    Map(OccupancyGrid),
    /// From nav-core, in order.
    Core(ToPlanner),
    Stop,
}

/// When nav-planner ends (also by panic): readiness goes down and nav-core
/// fails the plans it waits for instead of waiting for ever.
struct PlannerGone {
    shared: Arc<Shared>,
    core_tx: mpsc::Sender<Command>,
}

impl Drop for PlannerGone {
    fn drop(&mut self) {
        self.shared.planner_alive.store(false, Ordering::Release);
        let _ = self.core_tx.send(Command::PlannerGone);
    }
}

/// nav-planner's side: its channel and nav-core's.
struct PlannerIo {
    shared: Arc<Shared>,
    rx: mpsc::Receiver<PlannerMsg>,
    core_tx: mpsc::Sender<Command>,
    last_slow_report: Option<Instant>,
}

impl PlannerIo {
    fn run(mut self, mut planner: PlannerLoop) {
        let _gone = PlannerGone { shared: self.shared.clone(), core_tx: self.core_tx.clone() };
        let mut woken_by = None;
        loop {
            let now = wall_ns();
            let mut next = woken_by.take();
            loop {
                let msg = match next.take() {
                    Some(msg) => msg,
                    None => match self.rx.try_recv() {
                        Ok(msg) => msg,
                        Err(mpsc::TryRecvError::Empty) => break,
                        Err(mpsc::TryRecvError::Disconnected) => return,
                    },
                };
                match msg {
                    PlannerMsg::Map(grid) => {
                        planner.set_global_map(now, &grid);
                        self.shared.global_map.store(planner.has_map(), Ordering::Release);
                    }
                    PlannerMsg::Core(msg) => planner.handle(now, msg),
                    PlannerMsg::Stop => return,
                }
            }
            // the robot where it is now, at the start of the plan
            let started = Instant::now();
            let mut out = planner.tick(wall_ns(), self.shared.poses().map_pose());
            let took = started.elapsed();
            if !out.plans.is_empty() && took > SLOW_TICK && self.last_slow_report.is_none_or(|at| at.elapsed() >= SLOW_TICK_REPORT) {
                // upstream times a plan on the wall clock, NavFn included;
                // the server's own figure is its costmap wait
                self.last_slow_report = Some(Instant::now());
                let waited = out.log.iter().find_map(|l| if let StackLog::PlannerMissedRate(s) = l { Some(*s) } else { None });
                out.log.retain(|l| !matches!(l, StackLog::PlannerMissedRate(_)));
                out.log.push(StackLog::PlannerMissedRate(waited.unwrap_or(0.0) + took.as_secs_f64()));
            }
            if !out.is_empty() && self.core_tx.send(Command::Planner(out)).is_err() {
                return;
            }
            woken_by = match planner.next_due_ns() {
                Some(due) => {
                    let wait = Duration::from_nanos((due - wall_ns()).max(0) as u64).min(MAX_SLEEP);
                    match self.rx.recv_timeout(wait) {
                        Ok(msg) => Some(msg),
                        Err(mpsc::RecvTimeoutError::Timeout) => None,
                        Err(mpsc::RecvTimeoutError::Disconnected) => return,
                    }
                }
                None => match self.rx.recv() {
                    Ok(msg) => Some(msg),
                    Err(_) => return,
                },
            };
        }
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
    fn nav_core_ending_while_moving_leaves_one_zero() {
        let moving = StampedTwist { stamp_ns: 2_000, frame_id: "base_footprint".into(), twist: Twist2D::new(0.2, 0.0, 0.1), other: [0.0; 3] };
        let zero = final_zero(Some(&moving), 3_000).unwrap();
        assert_eq!((zero.twist, zero.stamp_ns, zero.frame_id.as_str()), (Twist2D::default(), 3_000, "base_footprint"));
        // never stamped before the last command, even with the clock behind
        assert_eq!(final_zero(Some(&moving), 1_000).unwrap().stamp_ns, 2_000);
        // already stopped, or never moved: nothing more
        assert_eq!(final_zero(Some(&zero), 4_000), None);
        assert_eq!(final_zero(None, 4_000), None);
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
    async fn a_follow_path_round_trip_on_the_bus_with_the_pose_from_the_tfbus() {
        round_trip(PoseSource::TfBus, "tfbus").await;
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn a_follow_path_round_trip_on_the_bus_with_the_pose_from_tf() {
        round_trip(PoseSource::TfTopic, "tf").await;
    }

    /// The whole round trip on the bus, from another scope as the other
    /// modules of mower_rsd would: maps and `/odom` in, the pose on the tfbus
    /// or on `/tf` (dynamic only, no `/tf_static`), a kinematic robot driven
    /// by `/nav_cmd_vel`.
    async fn round_trip(source: PoseSource, name: &str) {
        // Topics of its own, as the nav_core.* parameters name them, and
        // frames of its own: the tfbus is process-global.
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
        let bus = mower_bus::Bus::new();
        let node = format!("nav_core_runtime_test_{name}");
        let m = ModuleCtx::new(&node, mower_rs_common::Shutdown::new()).with_bus(bus.scope(&node));
        let rt = NavRuntime::start(&m, params.clone(), &node).unwrap();
        assert!(!rt.readiness().ready());

        let feeder = bus.scope("feeder");
        let map_pub = feeder.publisher::<ROccupancyGrid>(&params.map_topic, latched()).unwrap();
        let global_pub = feeder.publisher::<ROccupancyGrid>(&params.global_map_topic, latched()).unwrap();
        let odom_pub = feeder.publisher::<Odometry>(&params.odom_topic, Qos::default()).unwrap();
        let tf_pub = feeder.publisher::<TFMessage>("/tf", Qos::default().keep_last(100)).unwrap();
        let mut cmd = feeder.subscribe::<TwistStamped>(&params.cmd_vel_topic, Qos::default()).unwrap();
        map_pub.publish(&grid_msg(&params.map_frame)).unwrap();
        global_pub.publish(&grid_msg(&params.map_frame)).unwrap();

        let running = Arc::new(AtomicBool::new(true));
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

        // its writers went with it
        let writers = |topic: &str| bus.topics().into_iter().find(|t| t.name == topic).map_or(0, |t| t.publishers.len());
        assert_eq!((writers(&params.cmd_vel_topic), writers(&params.plan_topic)), (0, 0));

        running.store(false, Ordering::Relaxed);
        listener.abort();
        let _ = mover.await;
    }

    /// What nav-core published, with when.
    #[derive(Clone, Default)]
    struct Recorder {
        cmd_vel: Arc<Mutex<Vec<(Instant, StampedTwist)>>>,
        plans: Arc<Mutex<Vec<Instant>>>,
    }

    impl Outputs for Recorder {
        fn cmd_vel(&mut self, cmd: &StampedTwist) {
            lock(&self.cmd_vel).push((Instant::now(), cmd.clone()));
        }

        fn plan(&mut self, _plan: &[Pose2D], _stamp_ns: i64) {
            lock(&self.plans).push(Instant::now());
        }
    }

    /// The two threads on the wall clock, without ROS: a 60 m zone at
    /// 0.05 m (1240 x 1240 cells) and a goal 40 m away, so every NavFn search
    /// covers most of the map (seconds in a debug build; ~0.4 s on the
    /// RK3568 in release). While the planner thread searches, the robot keeps
    /// following the previous plan: nav-core's 20 Hz output must not pause.
    #[test]
    fn a_long_plan_on_a_big_map_does_not_hold_up_the_control_loop() {
        let params = CoreParams {
            // frames of its own on the process-global tfbus
            map_frame: "slow_plan_map".into(),
            odom_frame: "slow_plan_odom".into(),
            base_frame: "slow_plan_base".into(),
            ..CoreParams::default()
        };
        let rec = Recorder::default();
        let rt = NavRuntime::spawn(params.clone(), "nav_core_slow_plan_test", Box::new(rec.clone())).unwrap();
        let n = 1240u32;
        let data = (0..n).flat_map(|j| (0..n).map(move |i| if i == 0 || j == 0 || i == n - 1 || j == n - 1 { 100 } else { 0 })).collect();
        let map = OccupancyGrid { width: n, height: n, resolution: 0.05f32 as f64, origin_x: -31.0, origin_y: -31.0, data };
        rt.planner_tx.send(PlannerMsg::Map(map.clone())).unwrap();
        lock(&rt.shared.inputs).set_local_map(map);

        // a 50 Hz kinematic robot driven by the recorded /nav_cmd_vel
        let running = Arc::new(AtomicBool::new(true));
        let robot = {
            let (rec, running, shared, params) = (rec.clone(), running.clone(), rt.shared.clone(), params.clone());
            std::thread::spawn(move || {
                let mut pose = Pose2D::new(-20.0, 0.0, 0.0);
                while running.load(Ordering::Relaxed) {
                    let twist = lock(&rec.cmd_vel).last().map(|(_, c)| c.twist).unwrap_or_default();
                    let (s, c) = pose.yaw.sin_cos();
                    pose = Pose2D::new(pose.x + twist.x * c * 0.02, pose.y + twist.x * s * 0.02, pose.yaw + twist.theta * 0.02);
                    tfbus::set(&params.map_frame, &params.odom_frame, Transform::identity());
                    let base = Transform::from_parts(&Quaternion::from_rpy(0.0, 0.0, pose.yaw), Vector3::new(pose.x, pose.y, 0.0));
                    tfbus::set(&params.odom_frame, &params.base_frame, base);
                    lock(&shared.inputs).set_odom(twist, wall_ns());
                    std::thread::sleep(Duration::from_millis(20));
                }
                pose
            })
        };
        let wait_for = |what: &str, limit: Duration, done: &dyn Fn() -> bool| {
            let start = Instant::now();
            while !done() {
                assert!(start.elapsed() < limit, "{what} within {limit:?}");
                std::thread::sleep(Duration::from_millis(20));
            }
        };
        wait_for("ready", Duration::from_secs(30), &|| rt.readiness().ready());
        let goal = rt.send_navigate(Pose2D::new(20.0, 0.0, 0.0));
        // the first plan is in once the robot moves
        wait_for("the first plan", Duration::from_secs(120), &|| lock(&rec.cmd_vel).iter().any(|(_, c)| c.twist.x > 0.1));
        let first_plans = lock(&rec.plans).len();
        let from = Instant::now();
        // until two more plans have come in, the second of them searched for
        // while the robot drove
        wait_for("two replans", Duration::from_secs(120), &|| lock(&rec.plans).len() >= first_plans + 2);
        let to = Instant::now();

        let outputs: Vec<Instant> = lock(&rec.cmd_vel).iter().map(|(at, _)| *at).filter(|at| (from..to).contains(at)).collect();
        let longest = outputs.windows(2).map(|w| w[1] - w[0]).max().expect("outputs while planning");
        // a result is seen one tick after it comes in, and the 1 Hz replan
        // timer restarts then: the spacing of the plans is ~1 s + a search
        let plans = lock(&rec.plans).clone();
        let spacing: Vec<Duration> = plans.windows(2).map(|w| w[1] - w[0]).collect();
        println!("{} outputs over {:?}, longest gap {longest:?}; plans every {spacing:?}", outputs.len(), to - from);
        // 20 Hz is a 50 ms gap; a plan here takes far longer than the bound
        assert!(longest < Duration::from_millis(150), "a {longest:?} gap in {} outputs over {:?}", outputs.len(), to - from);
        assert!(outputs.len() as f64 >= (to - from).as_secs_f64() * 15.0, "{} outputs over {:?}", outputs.len(), to - from);
        assert!(rt.last_tick_age() < Duration::from_millis(150));
        let events = rt.recent_events(from - Duration::from_secs(60));
        assert!(!events.iter().any(|e| e.contains("Control loop missed")), "{events:?}");

        assert!(rt.cancel(goal.key));
        drop(rt);
        running.store(false, Ordering::Relaxed);
        let end = robot.join().unwrap();
        assert!(end.x > -20.0 + 0.5, "the robot drove while the planner searched: {end:?}");
    }
}
