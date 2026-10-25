//! The runtime's pure half: everything the nav-core thread does between
//! reading its inputs and publishing, on an explicit clock (`now_ns`, wall
//! time in ns, which is also what ROS stamps with on this robot). No r2r, no
//! threads, no I/O, so it runs the same live and under test.
//!
//! [`CoreLoop`] owns:
//!
//! * the stack. `NavStack::mower` is built when the first `/map_grid_global`
//!   arrives; a goal that comes before any map builds it on a one-cell
//!   placeholder that is never current, so the goal waits and times out the
//!   way nav2's servers do without a map. A later map does not rebuild it:
//!   NavFn re-sizes its arrays to the costmap it is handed
//!   (`isPlannerOutOfDate`), so the planner follows a map of another size.
//! * the global costmap (1 Hz, `track_unknown_space: true`). As in
//!   `StaticLayer`, the map's own grid is kept apart from the master grid
//!   the planner reads, which NavFn's `clearRobotCell` writes into. An
//!   update repaints the master only for a newer map or after a clear
//!   (`has_updated_data_`), so the holes NavFn leaves stay until then, as
//!   upstream. A newer map that arrives during a navigate goal waits for the
//!   next update (`map_buffer_`); between goals, which is when mower_map
//!   changes them, it is taken at once, so the next goal never plans on the
//!   old map (nav2 could, for up to a second).
//! * the local costmap (2 Hz): `/map_grid` through the static layer
//!   (`track_unknown_space: false`) sampled into the 3 x 3 m rolling window
//!   around the odom pose through `map <- odom`, at once for a new map. An
//!   update needs the map, the pose and the transform and is skipped
//!   without them, as `Costmap2DROS::updateMap` is when `getRobotPose`
//!   fails.
//! * the current flags, with nav2's semantics: a costmap is current once it
//!   has been painted; a clear (`StackOutput::clears`) makes it not current
//!   until its next update, which repaints it. The servers wait for a
//!   current costmap (and time out after 1 s).
//! * the velocity smoother: every command of the stack goes in, in order.
//!   Its 20 Hz timer runs from the first command after rest until it times
//!   out at rest, and its output is `/nav_cmd_vel`, stamped with the
//!   controller's stamps (`stamp_smoothed_velocity_with_smoothing_time:
//!   false`), in the base frame.
//! * the goals: the runtime's [`GoalKey`]s against the navigator's ids
//!   (navigate) and the controller's ids of `send_follow_path` (follow).
//!   Results and feedback go back by key, a nonzero code with a cause in
//!   `error_msg`.
//! * the event ring: the last [`EVENT_RING`] things nav2 would have logged
//!   at WARN or ERROR (`StackOutput::log`, clears) and what the inputs
//!   lacked, each with its time and prefixed with the nav2 node it belongs
//!   to (`[controller_server] ...`), as mower_nav's failure summary quotes
//!   nav2's `/rosout`. An event repeated back to back is one entry with a
//!   count.
//!
//! The clock is held when the wall clock steps back, so no stamp on
//! `/nav_cmd_vel` ever goes backward (velocity_command_guard rejects those).

use std::collections::{HashMap, VecDeque};

use mower_nav_core::action::GoalId;
use mower_nav_core::controller_server;
use mower_nav_core::costmap::{static_costmap, Costmap2D, OccupancyGrid, RollingWindow, StaticLayerParams, NO_INFORMATION};
use mower_nav_core::geometry::{Pose2D, Transform2D, Twist2D};
use mower_nav_core::nav_stack::{NavStack, StackInput, StackLog, StackOutput};
use mower_nav_core::navigator::CostmapId;
use mower_nav_core::planner::PlannerError;
use mower_nav_core::planner_server;
use mower_nav_core::rpp::{ControllerError, RegulatedPurePursuit};
use mower_nav_core::shim::RotationShim;
use mower_nav_core::smoother::{TwistStamped, VelocitySmoother, VelocitySmootherParams};

use super::{CoreFeedback, GoalKey, GoalOutcome, Readiness};

type Stack = NavStack<RotationShim<RegulatedPurePursuit>>;

pub const MS: i64 = 1_000_000;
pub const SEC: i64 = 1_000 * MS;
/// `update_frequency` of the two costmaps (nav2_no_map_params.yaml).
const GLOBAL_UPDATE_NS: i64 = SEC;
const LOCAL_UPDATE_NS: i64 = SEC / 2;
/// How many events the ring keeps.
pub const EVENT_RING: usize = 40;
/// A robot pose this old is reported (and still used: nav2 takes the latest
/// transform whatever its age).
const POSE_STALE_NS: i64 = 500 * MS;
/// A smaller step back of the wall clock is held but not reported.
const CLOCK_STEP_REPORT_NS: i64 = MS;

/// What the loop takes from `CoreParams`.
#[derive(Debug, Clone, PartialEq)]
pub struct LoopParams {
    pub map_frame: String,
    pub odom_frame: String,
    pub base_frame: String,
    /// Odometry older than this is stale (readiness, events).
    pub odom_timeout_ns: i64,
}

/// What the subscriptions have delivered: written by the IO tasks, read by
/// the thread before every tick and by `NavRuntime::readiness`.
#[derive(Debug, Default)]
pub struct Inputs {
    /// The latest `/odom` twist (`linear.x`, `linear.y`, `angular.z`) and
    /// when it arrived (wall ns).
    pub odom: Option<(Twist2D, i64)>,
    /// Maps the loop has not taken yet.
    pub global_map: Option<OccupancyGrid>,
    pub local_map: Option<OccupancyGrid>,
    /// A valid map has arrived on the topic.
    pub has_global_map: bool,
    pub has_local_map: bool,
    /// For the event ring (rejected maps); the loop takes them.
    pub notes: Vec<String>,
}

impl Inputs {
    pub fn set_odom(&mut self, twist: Twist2D, now_ns: i64) {
        self.odom = Some((twist, now_ns));
    }

    pub fn set_global_map(&mut self, grid: OccupancyGrid) {
        if valid_grid(&grid) {
            self.global_map = Some(grid);
            self.has_global_map = true;
        } else {
            self.notes.push("[global_costmap] Received map message is malformed. Rejecting.".into());
        }
    }

    pub fn set_local_map(&mut self, grid: OccupancyGrid) {
        if valid_grid(&grid) {
            self.local_map = Some(grid);
            self.has_local_map = true;
        } else {
            self.notes.push("[local_costmap] Received map message is malformed. Rejecting.".into());
        }
    }

    /// Both maps, the pose (`pose`: `map <- odom` and `odom <- base` both
    /// resolve) and odometry no older than `odom_timeout_ns`.
    pub fn readiness(&self, now_ns: i64, pose: bool, odom_timeout_ns: i64) -> Readiness {
        Readiness {
            global_map: self.has_global_map,
            local_map: self.has_local_map,
            pose,
            odometry: self.odom.is_some_and(|(_, at)| now_ns - at <= odom_timeout_ns),
        }
    }
}

/// `nav2_util::validateMsg` on what the static layer reads of a grid.
fn valid_grid(g: &OccupancyGrid) -> bool {
    g.width > 0
        && g.height > 0
        && g.data.len() == g.width as usize * g.height as usize
        && g.resolution.is_finite()
        && g.resolution > 0.0
        && g.origin_x.is_finite()
        && g.origin_y.is_finite()
}

/// The latest of every input, read before a tick.
#[derive(Debug, Clone, Copy, Default)]
pub struct TickInput {
    /// tf `map <- odom`, latest.
    pub map_from_odom: Option<Transform2D>,
    /// tf `odom <- base_footprint`, latest: the robot in `odom`.
    pub odom_pose: Option<Pose2D>,
    /// How long ago the older of the two was last updated, when known.
    pub pose_age_ns: Option<i64>,
    /// `/odom`: the twist and when it arrived (wall ns).
    pub odom: Option<(Twist2D, i64)>,
}

impl TickInput {
    /// `map <- base_footprint`: `map <- odom` after `odom <- base_footprint`.
    pub fn map_pose(&self) -> Option<Pose2D> {
        Some(self.map_from_odom?.apply_pose(&self.odom_pose?))
    }
}

/// What a tick produced.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TickOutput {
    /// `/nav_cmd_vel`: the smoother's output, in the base frame.
    pub cmd_vel: Vec<TwistStamped>,
    /// The stack's commands, the smoother's input.
    pub commands: Vec<TwistStamped>,
    /// `/plan`, in `map`.
    pub plans: Vec<Vec<Pose2D>>,
    pub feedback: Vec<(GoalKey, CoreFeedback)>,
    /// Goals that ended, in order; their keys are free again.
    pub outcomes: Vec<(GoalKey, GoalOutcome)>,
}

/// One entry of the event ring.
#[derive(Debug, Clone, PartialEq)]
pub struct Event {
    /// When it last happened (wall ns).
    pub at_ns: i64,
    /// `[node] message`, the node being the nav2 one that would have logged it.
    pub text: String,
    /// Back-to-back occurrences.
    pub count: u32,
    /// Entries are numbered from 1 as they are added (repeats keep theirs).
    pub seq: u64,
}

impl Event {
    pub fn render(&self) -> String {
        if self.count > 1 {
            format!("{} (x{})", self.text, self.count)
        } else {
            self.text.clone()
        }
    }
}

/// `rcl_timer_call`'s advance: the next firing after the one due at `due`
/// ran at `now`, keeping the phase and skipping the periods it missed.
pub fn next_period(due: i64, period: i64, now: i64) -> i64 {
    let next = due + period;
    if next >= now {
        next
    } else {
        next + (1 + (now - next - 1) / period) * period
    }
}

/// `GoalOutcome::error_msg`: empty for code 0; the exception's class and
/// `what()` when the result carries it, else what the code stands for.
pub fn error_msg(code: u16, error: Option<ControllerError>) -> String {
    match (code, error) {
        (0, _) => String::new(),
        (_, Some(e)) => e.to_string(),
        _ => code_text(code).to_string(),
    }
}

/// The exception behind each FollowPath (1xx) and ComputePathToPose (2xx)
/// code, with the cause the servers throw it for.
pub fn code_text(code: u16) -> &'static str {
    use controller_server::error_code as follow;
    use planner_server::error_code as plan;
    match code {
        follow::UNKNOWN => "ControllerException: the controller failed",
        follow::INVALID_CONTROLLER => "InvalidController: no such controller",
        follow::TF_ERROR => "ControllerTFError: the robot pose or the path could not be transformed",
        follow::INVALID_PATH => "InvalidPath: the path is empty or the robot is too far from it",
        follow::PATIENCE_EXCEEDED => "PatienceExceeded: Controller patience exceeded",
        follow::FAILED_TO_MAKE_PROGRESS => "FailedToMakeProgress: Failed to make progress",
        follow::NO_VALID_CONTROL => "NoValidControl: no collision-free command",
        follow::CONTROLLER_TIMED_OUT => "ControllerTimedOut: Costmap timed out waiting for update",
        plan::UNKNOWN => "the planner failed",
        plan::INVALID_PLANNER => "InvalidPlanner: no such planner",
        plan::TF_ERROR => "PlannerTFError: Unable to get start pose",
        plan::START_OUTSIDE_MAP => "StartOutsideMapBounds: the robot is outside the global costmap",
        plan::GOAL_OUTSIDE_MAP => "GoalOutsideMapBounds: the goal is outside the global costmap",
        plan::START_OCCUPIED => "StartOccupied: the robot is in lethal cost",
        plan::GOAL_OCCUPIED => "GoalOccupied: the goal is in lethal cost",
        plan::TIMEOUT => "PlannerTimedOut: Costmap timed out waiting for update",
        plan::NO_VALID_PATH => "NoValidPathCouldBeFound: no path to the goal in the global costmap",
        _ => "unknown error code",
    }
}

/// The `what()` planner_server and NavFn throw each `PlannerError` with
/// (without the coordinates upstream prints).
fn planner_what(e: PlannerError) -> &'static str {
    match e {
        PlannerError::StartOutsideMapBounds => "Start Coordinates were outside bounds",
        PlannerError::GoalOutsideMapBounds => "Goal Coordinates were outside bounds",
        PlannerError::GoalOccupied => "Goal Coordinates were in lethal cost",
        PlannerError::NoValidPathCouldBeFound => "Failed to create plan with tolerance of: 0.000000",
        PlannerError::PlannerCancelled => "Goal was canceled. Canceling planning action.",
        PlannerError::InvalidPlanner => "Planner id is invalid",
        PlannerError::StartOccupied => "Start Coordinates were in lethal cost",
        PlannerError::PlannerTimedOut => "Costmap timed out waiting for update",
        PlannerError::PlannerTFError => "Unable to get start pose",
        PlannerError::NoViapointsGiven => "No viapoints given",
        PlannerError::Unknown => "Unknown planner error",
    }
}

/// The global costmap.
#[derive(Debug)]
struct Global {
    /// The static layer's own grid (`processMap`).
    layer: Costmap2D,
    /// The master grid the planner reads and NavFn writes into.
    master: Costmap2D,
    current: bool,
    /// Not current because of a clear (not for want of a map).
    cleared: bool,
    /// A newer map, taken at the next update (`map_buffer_`).
    buffered: Option<Costmap2D>,
    next_update_ns: i64,
}

/// The local costmap.
#[derive(Debug)]
struct Local {
    /// `/map_grid` through the static layer.
    map: Option<Costmap2D>,
    window: RollingWindow,
    current: bool,
    cleared: bool,
    /// From the first map on.
    next_update_ns: Option<i64>,
    /// Why the last update due did not run.
    skipped: Option<&'static str>,
}

/// Problems already in the ring; reported again once they went away.
#[derive(Debug, Default)]
struct Reported {
    odom: bool,
    odom_pose: bool,
    map_from_odom: bool,
    pose_age: bool,
    local_wait: bool,
    global_wait: bool,
    clock: bool,
}

impl Reported {
    /// The per-goal ones, when nothing runs any more.
    fn reset_goal_flags(&mut self) {
        *self = Self { clock: self.clock, ..Self::default() };
    }
}

/// See the module doc.
pub struct CoreLoop {
    p: LoopParams,
    stack: Option<Stack>,
    global: Option<Global>,
    /// What the stack is built on before the first global map.
    placeholder: Costmap2D,
    local: Local,
    smoother: VelocitySmoother,
    /// The smoother timer's next firing; `None` at rest.
    smoother_next_ns: Option<i64>,
    navigate: HashMap<GoalId, GoalKey>,
    follow: HashMap<GoalId, GoalKey>,
    ids: HashMap<GoalKey, GoalId>,
    /// The last failure per error code, `class: what()`, for the message of
    /// a navigate result (which carries only the code).
    failures: HashMap<u16, String>,
    events: VecDeque<Event>,
    events_revision: u64,
    next_seq: u64,
    now_ns: Option<i64>,
    reported: Reported,
}

impl CoreLoop {
    pub fn new(p: LoopParams) -> Self {
        Self {
            p,
            stack: None,
            global: None,
            placeholder: Costmap2D::new(1, 1, 0.05, 0.0, 0.0, NO_INFORMATION),
            local: Local {
                map: None,
                window: RollingWindow::local_costmap(),
                current: false,
                cleared: false,
                next_update_ns: None,
                skipped: None,
            },
            smoother: VelocitySmoother::new(VelocitySmootherParams::mower()).expect("the yaml's smoother parameters are valid"),
            smoother_next_ns: None,
            navigate: HashMap::new(),
            follow: HashMap::new(),
            ids: HashMap::new(),
            failures: HashMap::new(),
            events: VecDeque::new(),
            events_revision: 0,
            next_seq: 1,
            now_ns: None,
            reported: Reported::default(),
        }
    }

    /// The loop's time: the latest `now_ns` it was given.
    pub fn now_ns(&self) -> Option<i64> {
        self.now_ns
    }

    /// A `/map_grid_global`. The first one is painted at once (the static
    /// layer processes it in its callback) and builds the stack; a later one
    /// waits for the next update while a navigate goal runs, and is taken at
    /// once otherwise.
    pub fn set_global_map(&mut self, now_ns: i64, grid: &OccupancyGrid) {
        let now = self.clock(now_ns);
        let layer = static_costmap(grid, &StaticLayerParams::new(true));
        let navigating = self.stack.as_ref().is_some_and(|s| s.navigator.is_executing() || s.planner.current_goal().is_some());
        if let Some(g) = &mut self.global {
            if navigating {
                g.buffered = Some(layer);
            } else {
                g.layer = layer;
                g.master.clone_from(&g.layer);
                g.buffered = None;
                g.current = true;
                g.cleared = false;
            }
            return;
        }
        let g = self.global.insert(Global {
            master: layer.clone(),
            layer,
            current: true,
            cleared: false,
            buffered: None,
            next_update_ns: now + GLOBAL_UPDATE_NS,
        });
        if self.stack.is_none() {
            self.stack = Some(NavStack::mower(&g.master));
        }
    }

    /// A `/map_grid`, sampled into the window by an update at the next tick.
    pub fn set_local_map(&mut self, now_ns: i64, grid: &OccupancyGrid) {
        let now = self.clock(now_ns);
        self.local.map = Some(static_costmap(grid, &StaticLayerParams::new(false)));
        self.local.next_update_ns = Some(now);
    }

    pub fn send_navigate(&mut self, key: GoalKey, pose: Pose2D) {
        let id = self.stack_mut().send_navigate(pose);
        self.navigate.insert(id, key);
        self.ids.insert(key, id);
    }

    pub fn send_follow_path(&mut self, key: GoalKey, path: Vec<Pose2D>) {
        let id = self.stack_mut().send_follow_path(path);
        self.follow.insert(id, key);
        self.ids.insert(key, id);
    }

    /// `false` for a key that is unknown or has ended.
    pub fn cancel(&mut self, key: GoalKey) -> bool {
        let (Some(&id), Some(stack)) = (self.ids.get(&key), self.stack.as_mut()) else {
            return false;
        };
        match key {
            GoalKey::Navigate(_) => stack.cancel_navigate(id),
            GoalKey::Follow(_) => stack.cancel_follow_path(id),
        }
    }

    /// When the loop next has something to do; `None` while nothing runs
    /// (no goal, the smoother at rest): the costmaps then wait too, and an
    /// update that fell due meanwhile runs at the next tick.
    pub fn next_due_ns(&self) -> Option<i64> {
        let stack = self.stack.as_ref().and_then(Stack::next_tick_ns);
        if stack.is_none() && self.smoother_next_ns.is_none() {
            return None;
        }
        let global = self.global.as_ref().map(|g| g.next_update_ns);
        [stack, self.smoother_next_ns, global, self.local.next_update_ns].into_iter().flatten().min()
    }

    /// One pass at `now_ns`: the costmap updates that are due, the input
    /// checks, the stack when it is due, the smoother when it is due.
    pub fn tick(&mut self, now_ns: i64, input: &TickInput) -> TickOutput {
        let now = self.clock(now_ns);
        let mut out = TickOutput::default();
        self.update_costmaps(now, input);
        if self.stack.as_ref().and_then(Stack::next_tick_ns).is_some() {
            self.check_inputs(now, input);
        } else {
            self.reported.reset_goal_flags();
        }
        if self.stack.as_ref().and_then(Stack::next_tick_ns).is_some_and(|t| t <= now) {
            let s = self.tick_stack(now, input);
            self.apply(now, s, &mut out);
        }
        self.tick_smoother(now, &mut out);
        out
    }

    /// Adds `text` to the ring at `now_ns` (or counts it, when it repeats
    /// the newest entry).
    pub fn record(&mut self, now_ns: i64, text: impl Into<String>) {
        let text = text.into();
        self.events_revision += 1;
        if let Some(last) = self.events.back_mut().filter(|e| e.text == text) {
            last.count += 1;
            last.at_ns = now_ns;
            return;
        }
        if self.events.len() == EVENT_RING {
            self.events.pop_front();
        }
        self.events.push_back(Event { at_ns: now_ns, text, count: 1, seq: self.next_seq });
        self.next_seq += 1;
    }

    /// Oldest first.
    pub fn events(&self) -> &VecDeque<Event> {
        &self.events
    }

    /// Changes whenever [`CoreLoop::events`] does.
    pub fn events_revision(&self) -> u64 {
        self.events_revision
    }

    // --- the pass ----------------------------------------------------------

    /// The wall clock, held while it is behind the latest time seen.
    fn clock(&mut self, now_ns: i64) -> i64 {
        let last = *self.now_ns.get_or_insert(now_ns);
        if now_ns >= last {
            self.now_ns = Some(now_ns);
            self.reported.clock = false;
            return now_ns;
        }
        if !self.reported.clock && last - now_ns >= CLOCK_STEP_REPORT_NS {
            self.reported.clock = true;
            let step = (last - now_ns) as f64 / 1e9;
            self.record(last, format!("[controller_server] the wall clock went back {step:.3} s; navigation time is held until it catches up"));
        }
        last
    }

    fn stack_mut(&mut self) -> &mut Stack {
        let costmap = self.global.as_ref().map_or(&self.placeholder, |g| &g.master);
        self.stack.get_or_insert_with(|| NavStack::mower(costmap))
    }

    /// The two costmaps' update loops, when due.
    fn update_costmaps(&mut self, now: i64, input: &TickInput) {
        if let Some(g) = self.global.as_mut().filter(|g| g.next_update_ns <= now) {
            if let Some(layer) = g.buffered.take() {
                g.layer = layer;
                g.master.clone_from(&g.layer);
                g.current = true;
            } else if !g.current {
                g.master.clone_from(&g.layer);
                g.current = true;
            }
            g.cleared = false;
            g.next_update_ns = next_period(g.next_update_ns, GLOBAL_UPDATE_NS, now);
        }
        let Some(due) = self.local.next_update_ns.filter(|&t| t <= now) else {
            return;
        };
        let l = &mut self.local;
        l.next_update_ns = Some(next_period(due, LOCAL_UPDATE_NS, now));
        let why = match (&l.map, input.odom_pose, input.map_from_odom) {
            (Some(map), Some(pose), Some(map_from_odom)) => {
                l.window.update(pose.x, pose.y, map, &map_from_odom);
                l.current = true;
                l.cleared = false;
                l.skipped = None;
                return;
            }
            (None, _, _) => "no local map",
            (_, None, _) => "no robot pose in odom",
            (_, _, None) => "no map <- odom transform",
        };
        if l.skipped != Some(why) {
            l.skipped = Some(why);
            self.record(now, format!("[local_costmap] update skipped: {why}"));
        }
    }

    /// Stale or missing inputs while something runs, each reported once.
    fn check_inputs(&mut self, now: i64, input: &TickInput) {
        let mut found = Vec::new();
        let odom_age = input.odom.map(|(_, at)| now - at);
        let odom_stale = odom_age.is_none_or(|a| a > self.p.odom_timeout_ns);
        if odom_stale && !self.reported.odom {
            found.push(match odom_age {
                None => "[controller_server] no odometry yet: the controller takes the robot as standing".to_string(),
                Some(a) => format!("[controller_server] odometry is {:.1} s old: the controller keeps its last twist", a as f64 / 1e9),
            });
        }
        self.reported.odom = odom_stale;

        let (m, o, b) = (&self.p.map_frame, &self.p.odom_frame, &self.p.base_frame);
        let missing = input.odom_pose.is_none();
        if missing && !self.reported.odom_pose {
            found.push(format!("[controller_server] no transform {o} <- {b}: the robot pose is unavailable"));
        }
        self.reported.odom_pose = missing;
        let missing = input.map_from_odom.is_none();
        if missing && !self.reported.map_from_odom {
            found.push(format!("[bt_navigator] no transform {m} <- {o}: the robot pose in {m} is unavailable"));
        }
        self.reported.map_from_odom = missing;
        let stale = input.pose_age_ns.is_some_and(|a| a > POSE_STALE_NS);
        if stale && !self.reported.pose_age {
            let age = input.pose_age_ns.unwrap_or_default() as f64 / 1e9;
            found.push(format!("[controller_server] the robot pose ({m} <- {o} <- {b}) is {age:.1} s old: navigating on it"));
        }
        self.reported.pose_age = stale;

        // a server blocked on a costmap that is not current for want of an
        // update (a clear has its own entry)
        let stack = self.stack.as_ref().expect("checked while the stack runs");
        let local_wait = stack.controller.is_executing() && !self.local.current && !self.local.cleared;
        if local_wait && !self.reported.local_wait {
            let why = match (&self.local.map, self.local.skipped) {
                (None, _) => "no local map received",
                (_, Some(why)) => why,
                _ => "not updated yet",
            };
            found.push(format!("[controller_server] waiting for the local costmap: {why}"));
        }
        self.reported.local_wait = local_wait;
        let global_wait = stack.planner.current_goal().is_some() && self.global.as_ref().is_none_or(|g| !g.current && !g.cleared);
        if global_wait && !self.reported.global_wait {
            found.push("[planner_server] waiting for the global costmap: no global map received".to_string());
        }
        self.reported.global_wait = global_wait;

        for text in found {
            self.record(now, text);
        }
    }

    fn tick_stack(&mut self, now: i64, input: &TickInput) -> StackOutput {
        let Self { stack, global, placeholder, local, .. } = self;
        let stack = stack.as_mut().expect("ticked when due");
        let (global_costmap, global_costmap_current) = match global {
            Some(g) => (&mut g.master, g.current),
            None => (placeholder, false),
        };
        stack.tick(StackInput {
            now_ns: now,
            map_pose: input.map_pose(),
            odom_pose: input.odom_pose,
            odom_twist: input.odom.map(|(t, _)| t).unwrap_or_default(),
            odom_from_map: input.map_from_odom.map(|t| t.inverse()),
            local_costmap: local.window.costmap(),
            local_costmap_current: local.current,
            global_costmap,
            global_costmap_current,
        })
    }

    /// The stack's output: clears and log lines into the ring, commands into
    /// the smoother, results and feedback to their keys.
    fn apply(&mut self, now: i64, s: StackOutput, out: &mut TickOutput) {
        for c in &s.clears {
            let name = match c {
                CostmapId::Global => {
                    if let Some(g) = &mut self.global {
                        g.current = false;
                        g.cleared = true;
                    }
                    "global_costmap"
                }
                CostmapId::Local => {
                    self.local.current = false;
                    self.local.cleared = true;
                    "local_costmap"
                }
            };
            self.record(now, format!("[{name}] Received request to clear entirely the {name}"));
        }
        for l in &s.log {
            let text = self.log_text(l);
            self.record(now, text);
        }
        for cmd in &s.commands {
            match self.smoother.input_command(cmd, now) {
                Ok(()) => {
                    self.smoother_next_ns.get_or_insert(now);
                }
                Err(_) => self.record(now, "[velocity_smoother] Velocity message contains NaNs or Infs! Ignoring as invalid!"),
            }
        }
        out.commands.extend(s.commands);
        out.plans.extend(s.plans);
        if let Some((id, fb)) = s.navigate_feedback {
            if let Some(&key) = self.navigate.get(&id) {
                out.feedback.push((key, CoreFeedback::Navigate { distance_remaining: fb.distance_remaining }));
            }
        }
        if let Some((id, fb)) = s.follow_feedback {
            if let Some(&key) = self.follow.get(&id) {
                out.feedback.push((key, CoreFeedback::Follow { distance_to_goal: fb.distance_to_goal, speed: fb.speed }));
            }
        }
        for r in s.navigate_results {
            if let Some(key) = self.navigate.remove(&r.goal) {
                self.ids.remove(&key);
                let error_msg = match r.error_code {
                    0 => String::new(),
                    code => self.failures.get(&code).cloned().unwrap_or_else(|| code_text(code).to_string()),
                };
                out.outcomes.push((key, GoalOutcome { status: r.status, error_code: r.error_code, error_msg }));
            }
        }
        for r in s.follow_results {
            if let Some(key) = self.follow.remove(&r.goal) {
                self.ids.remove(&key);
                out.outcomes.push((key, GoalOutcome { status: r.status, error_code: r.error_code, error_msg: error_msg(r.error_code, r.error) }));
            }
        }
    }

    /// A log line as the server would have written it; failures are also
    /// kept for the navigate result's message.
    fn log_text(&mut self, l: &StackLog) -> String {
        match *l {
            StackLog::ControlTolerated(e) => format!("[controller_server] {}", e.what()),
            StackLog::FollowPathFailed(e) => {
                self.failures.insert(controller_server::error_code_of(&e), e.to_string());
                format!("[controller_server] {}", e.what())
            }
            StackLog::ComputePathFailed(e) => {
                self.failures.insert(planner_server::error_code_of(&e), format!("{e:?}: {}", planner_what(e)));
                format!("[planner_server] GridBased plugin failed to plan: \"{}\"", planner_what(e))
            }
            StackLog::PlannerMissedRate(s) => {
                let hz = self.stack.as_ref().map_or(20.0, |st| st.planner.params().expected_planner_frequency);
                format!("[planner_server] Planner loop missed its desired rate of {hz:.4} Hz. Current loop rate is {:.4} Hz", 1.0 / s)
            }
        }
    }

    /// The smoother's timer.
    fn tick_smoother(&mut self, now: i64, out: &mut TickOutput) {
        let Some(due) = self.smoother_next_ns.filter(|&t| t <= now) else {
            return;
        };
        match self.smoother.tick(now) {
            Some(mut cmd) => {
                cmd.frame_id.clone_from(&self.p.base_frame);
                out.cmd_vel.push(cmd);
                self.smoother_next_ns = Some(next_period(due, self.smoother.timer_period_ns(), now));
            }
            // timed out at rest: quiet until the next command
            None => self.smoother_next_ns = None,
        }
    }
}

#[cfg(test)]
impl CoreLoop {
    fn global_current(&self) -> bool {
        self.global.as_ref().is_some_and(|g| g.current)
    }

    fn global_next_update_ns(&self) -> Option<i64> {
        self.global.as_ref().map(|g| g.next_update_ns)
    }

    fn global_size(&self) -> Option<(u32, u32)> {
        self.global.as_ref().map(|g| (g.master.size_x(), g.master.size_y()))
    }

    fn local_current(&self) -> bool {
        self.local.current
    }

    fn texts(&self) -> Vec<String> {
        self.events.iter().map(Event::render).collect()
    }
}

#[cfg(test)]
mod tests {
    use std::collections::HashSet;

    use mower_nav_core::action::GoalStatus;

    use super::*;

    /// A plausible wall-clock time.
    const T0: i64 = 1_792_900_000_000_000_000;

    fn params() -> LoopParams {
        LoopParams { map_frame: "map".into(), odom_frame: "odom".into(), base_frame: "base_footprint".into(), odom_timeout_ns: 500 * MS }
    }

    /// `w` x `h` m at 0.05 m from (-1, -1): free, a lethal border, plus the
    /// lethal boxes `(x0, y0, x1, y1)`.
    fn grid(w_m: f64, h_m: f64, boxes: &[(f64, f64, f64, f64)]) -> OccupancyGrid {
        let (res, ox, oy) = (0.05, -1.0, -1.0);
        let (w, h) = ((w_m / res).round() as u32, (h_m / res).round() as u32);
        let mut data = vec![0i8; (w * h) as usize];
        for j in 0..h {
            for i in 0..w {
                let (x, y) = (ox + (i as f64 + 0.5) * res, oy + (j as f64 + 0.5) * res);
                let border = i == 0 || j == 0 || i == w - 1 || j == h - 1;
                if border || boxes.iter().any(|&(x0, y0, x1, y1)| x0 <= x && x < x1 && y0 <= y && y < y1) {
                    data[(j * w + i) as usize] = 100;
                }
            }
        }
        // MapMetaData.resolution is a float32
        OccupancyGrid { width: w, height: h, resolution: res as f32 as f64, origin_x: ox, origin_y: oy, data }
    }

    fn line(from: Pose2D, length: f64) -> Vec<Pose2D> {
        let n = (length / 0.1).round() as usize;
        let (s, c) = from.yaw.sin_cos();
        (0..=n).map(|i| Pose2D::new(from.x + c * 0.1 * i as f64, from.y + s * 0.1 * i as f64, from.yaw)).collect()
    }

    /// The loop on the IO thread's schedule (wake at the next due time, at
    /// most 10 ms later), and a kinematic robot driven by `/nav_cmd_vel`
    /// whose odometry reports what it was told.
    struct Sim {
        core: CoreLoop,
        now: i64,
        map_from_odom: Transform2D,
        /// The robot in `odom`.
        odom: Pose2D,
        twist: Twist2D,
        /// `/nav_cmd_vel`, with when it went out.
        cmd_vel: Vec<(i64, TwistStamped)>,
        commands: Vec<TwistStamped>,
        outcomes: Vec<(i64, GoalKey, GoalOutcome)>,
        feedback: Vec<(i64, GoalKey, CoreFeedback)>,
        plans: usize,
        next_key: u64,
    }

    impl Sim {
        fn new(start: Pose2D) -> Self {
            let map_from_odom = Transform2D::new(0.4, -0.3, 0.2);
            Self {
                core: CoreLoop::new(params()),
                now: T0,
                map_from_odom,
                odom: map_from_odom.inverse().apply_pose(&start),
                twist: Twist2D::default(),
                cmd_vel: Vec::new(),
                commands: Vec::new(),
                outcomes: Vec::new(),
                feedback: Vec::new(),
                plans: 0,
                next_key: 1,
            }
        }

        fn with_maps(start: Pose2D, global: &OccupancyGrid, local: &OccupancyGrid) -> Self {
            let mut sim = Self::new(start);
            sim.core.set_global_map(sim.now, global);
            sim.core.set_local_map(sim.now, local);
            sim
        }

        fn pose(&self) -> Pose2D {
            self.map_from_odom.apply_pose(&self.odom)
        }

        fn navigate(&mut self, goal: Pose2D) -> GoalKey {
            let key = GoalKey::Navigate(self.next_key);
            self.next_key += 1;
            self.core.send_navigate(key, goal);
            key
        }

        fn follow(&mut self, path: Vec<Pose2D>) -> GoalKey {
            let key = GoalKey::Follow(self.next_key);
            self.next_key += 1;
            self.core.send_follow_path(key, path);
            key
        }

        fn input(&self) -> TickInput {
            TickInput {
                map_from_odom: Some(self.map_from_odom),
                odom_pose: Some(self.odom),
                pose_age_ns: Some(0),
                odom: Some((self.twist, self.now)),
            }
        }

        /// One tick, then the robot moves until the next one.
        fn step(&mut self) -> TickOutput {
            let out = self.core.tick(self.now, &self.input());
            self.cmd_vel.extend(out.cmd_vel.iter().map(|c| (self.now, c.clone())));
            self.commands.extend(out.commands.iter().cloned());
            self.plans += out.plans.len();
            for (k, fb) in &out.feedback {
                self.feedback.push((self.now, *k, *fb));
            }
            for (k, o) in &out.outcomes {
                self.outcomes.push((self.now, *k, o.clone()));
            }
            if let Some(c) = out.cmd_vel.last() {
                self.twist = c.twist;
            }
            let next = self.core.next_due_ns().map_or(self.now + 10 * MS, |t| t.clamp(self.now + 1, self.now + 10 * MS));
            let dt = (next - self.now) as f64 / 1e9;
            let (s, c) = (self.odom.yaw + 0.5 * self.twist.theta * dt).sin_cos();
            self.odom = Pose2D::new(self.odom.x + self.twist.x * c * dt, self.odom.y + self.twist.x * s * dt, self.odom.yaw + self.twist.theta * dt);
            self.now = next;
            out
        }

        /// Steps until `key` ends (or `limit_s` of simulated time passed).
        fn run_until_done(&mut self, key: GoalKey, limit_s: f64) -> GoalOutcome {
            let end = self.now + (limit_s * 1e9) as i64;
            while self.now < end {
                self.step();
                if let Some((_, _, o)) = self.outcomes.iter().find(|(_, k, _)| *k == key) {
                    return o.clone();
                }
            }
            panic!("{key:?} did not end within {limit_s} s; events: {:#?}", self.core.texts());
        }

        fn run_for(&mut self, s: f64) {
            let end = self.now + (s * 1e9) as i64;
            while self.now < end {
                self.step();
            }
        }

        fn outcome(&self, key: GoalKey) -> Option<&GoalOutcome> {
            self.outcomes.iter().find(|(_, k, _)| *k == key).map(|(_, _, o)| o)
        }
    }

    fn succeeded() -> GoalOutcome {
        GoalOutcome { status: GoalStatus::Succeeded, error_code: 0, error_msg: String::new() }
    }

    #[test]
    fn navigate_then_follow_succeed_on_a_kinematic_robot() {
        let map = grid(10.0, 8.0, &[]);
        let mut sim = Sim::with_maps(Pose2D::new(1.0, 1.0, 0.0), &map, &map);
        let goal = Pose2D::new(5.0, 3.0, 0.5);
        let nav = sim.navigate(goal);
        assert_eq!(sim.run_until_done(nav, 60.0), succeeded());
        let p = sim.pose();
        assert!((p.x - goal.x).hypot(p.y - goal.y) < 0.1, "ended at {p:?}");
        assert!(sim.plans >= 1);
        let distances: Vec<f32> = sim
            .feedback
            .iter()
            .filter_map(|(_, k, f)| match f {
                CoreFeedback::Navigate { distance_remaining } if *k == nav => Some(*distance_remaining),
                _ => None,
            })
            .collect();
        assert!(distances.len() > 100, "{} navigate feedbacks", distances.len());
        assert!(distances.iter().cloned().fold(0.0, f32::max) > 4.0 && *distances.last().unwrap() < 0.2, "{distances:?}");

        let start = sim.pose();
        let path = line(start, 1.5);
        let end = *path.last().unwrap();
        let follow = sim.follow(path);
        assert_eq!(sim.run_until_done(follow, 30.0), succeeded());
        let p = sim.pose();
        assert!((p.x - end.x).hypot(p.y - end.y) < 0.1, "ended at {p:?}, path end {end:?}");
        assert!(sim.feedback.iter().any(|(_, k, f)| *k == follow && matches!(f, CoreFeedback::Follow { speed, .. } if *speed > 0.2)));

        // /nav_cmd_vel carries the controller's stamps, in the base frame,
        // and never goes back
        let stamps: HashSet<i64> = sim.commands.iter().map(|c| c.stamp_ns).collect();
        assert!(sim.cmd_vel.len() > 400, "{} commands published", sim.cmd_vel.len());
        for (_, c) in &sim.cmd_vel {
            assert!(stamps.contains(&c.stamp_ns), "{c:?} is not a controller stamp");
            assert_eq!(c.frame_id, "base_footprint");
        }
        assert!(sim.cmd_vel.windows(2).all(|w| w[0].1.stamp_ns <= w[1].1.stamp_ns));
        // the smoother rests once the zero after the goal has timed out
        sim.run_for(2.0);
        assert_eq!(sim.core.next_due_ns(), None);
        assert!(sim.core.texts().is_empty(), "{:#?}", sim.core.texts());
    }

    #[test]
    fn cancel_ends_a_goal_canceled_within_one_controller_pass() {
        let map = grid(10.0, 8.0, &[]);
        let mut sim = Sim::with_maps(Pose2D::new(1.0, 1.0, 0.0), &map, &map);
        let follow = sim.follow(line(sim.pose(), 3.0));
        sim.run_for(1.0);
        assert!(sim.outcome(follow).is_none());
        assert!(sim.core.cancel(follow));
        let asked = sim.now;
        let canceled = GoalOutcome { status: GoalStatus::Canceled, error_code: 0, error_msg: String::new() };
        assert_eq!(sim.run_until_done(follow, 1.0), canceled);
        let at = sim.outcomes.iter().find(|(_, k, _)| *k == follow).unwrap().0;
        assert!(at - asked <= 50 * MS, "canceled {} ms after the request", (at - asked) / MS);
        // publishZeroVelocity
        assert_eq!(sim.commands.last().unwrap().twist, Twist2D::default());
        assert!(!sim.core.cancel(follow), "an ended goal cannot be canceled");

        let nav = sim.navigate(Pose2D::new(6.0, 4.0, 0.0));
        sim.run_for(1.5);
        assert!(sim.core.cancel(nav));
        assert_eq!(sim.run_until_done(nav, 1.0), canceled);
    }

    #[test]
    fn after_a_cancel_the_output_ramps_to_zero_within_half_a_second_and_stays_there() {
        // cancelled while the shim turns in place (0.6 rad/s) towards a path
        // at 90 degrees: the angular ramp down at 3.2 rad/s^2 is the longer one
        let map = grid(10.0, 8.0, &[]);
        let mut sim = Sim::with_maps(Pose2D::new(2.0, 2.0, 0.0), &map, &map);
        let start = sim.pose();
        let follow = sim.follow(line(Pose2D::new(start.x, start.y, std::f64::consts::FRAC_PI_2), 2.0));
        sim.run_for(0.6);
        let turning = sim.cmd_vel.last().unwrap().1.twist;
        assert!(turning.theta.abs() > 0.5 && turning.x.abs() < 1e-9, "{turning:?}");
        assert!(sim.core.cancel(follow));
        let asked = sim.now;
        sim.run_for(3.0);
        assert_eq!(sim.outcome(follow).map(|o| o.status), Some(GoalStatus::Canceled));
        let zero = |t: &Twist2D| t.x.abs() < 1e-6 && t.theta.abs() < 1e-6;
        let after: Vec<&(i64, TwistStamped)> = sim.cmd_vel.iter().filter(|(at, _)| *at >= asked).collect();
        let first_zero = after.iter().position(|(_, c)| zero(&c.twist)).expect("a zero went out");
        assert!(after[first_zero].0 - asked <= 500 * MS, "zero {} ms after the cancel", (after[first_zero].0 - asked) / MS);
        assert!(after[first_zero..].iter().all(|(_, c)| zero(&c.twist)));
        // the zero is replayed until the smoother's 1 s timeout, then nothing
        let last = after.last().unwrap().0;
        assert!(last - asked < 1500 * MS && sim.now - last > SEC);
        assert_eq!(sim.core.next_due_ns(), None);
    }

    #[test]
    fn a_follow_path_preempts_the_running_one_which_ends_aborted_with_code_0() {
        let map = grid(10.0, 8.0, &[]);
        let mut sim = Sim::with_maps(Pose2D::new(1.0, 1.0, 0.0), &map, &map);
        let first = sim.follow(line(sim.pose(), 3.0));
        sim.run_for(1.0);
        let second = sim.follow(line(sim.pose(), 1.0));
        let preempted = sim.run_until_done(first, 0.2);
        assert_eq!(preempted, GoalOutcome { status: GoalStatus::Aborted, error_code: 0, error_msg: String::new() });
        assert_eq!(sim.run_until_done(second, 20.0), succeeded());
    }

    #[test]
    fn a_cleared_global_costmap_is_not_current_until_its_next_update() {
        // the goal sits inside a closed ring: NavFn finds no path (208), and
        // the tree clears the global costmap before it retries
        let ring = [(5.0, 1.0, 7.0, 1.2), (5.0, 2.8, 7.0, 3.0), (5.0, 1.0, 5.2, 3.0), (6.8, 1.0, 7.0, 3.0)];
        let global = grid(10.0, 8.0, &ring);
        let local = grid(10.0, 8.0, &[]);
        let mut sim = Sim::with_maps(Pose2D::new(1.0, 1.0, 0.0), &global, &local);
        let nav = sim.navigate(Pose2D::new(6.0, 2.0, 0.0));
        let mut cleared_at = None;
        while cleared_at.is_none() && sim.now < T0 + 5 * SEC {
            sim.step();
            if !sim.core.global_current() {
                cleared_at = Some(sim.now);
            }
        }
        let cleared_at = cleared_at.expect("the tree cleared the global costmap");
        let texts = sim.core.texts();
        assert!(texts.iter().any(|t| t.contains("Failed to create plan")), "{texts:#?}");
        assert!(texts.iter().any(|t| t == "[global_costmap] Received request to clear entirely the global_costmap"), "{texts:#?}");
        // current again exactly at the 1 Hz update after the clear
        let update = sim.core.global_next_update_ns().unwrap();
        assert!(update > cleared_at - 10 * MS && update - cleared_at <= SEC);
        while sim.now < update {
            sim.step();
            assert!(!sim.core.global_current(), "current before its update");
        }
        sim.step();
        assert!(sim.core.global_current());
        sim.core.cancel(nav);
        assert_eq!(sim.run_until_done(nav, 2.0).status, GoalStatus::Canceled);
    }

    #[test]
    fn a_cleared_local_costmap_is_not_current_until_its_next_update() {
        // a wall the global costmap does not have: RPP stops in front of it,
        // NoValidControl is tolerated for 2 s, the patience runs out (104)
        // and the tree clears the local costmap
        let global = grid(10.0, 8.0, &[]);
        let local = grid(10.0, 8.0, &[(3.0, -1.0, 3.3, 7.0)]);
        let mut sim = Sim::with_maps(Pose2D::new(1.0, 1.0, 0.0), &global, &local);
        let nav = sim.navigate(Pose2D::new(5.0, 1.0, 0.0));
        let mut cleared_at = None;
        while cleared_at.is_none() && sim.now < T0 + 30 * SEC {
            sim.step();
            if !sim.core.local_current() {
                cleared_at = Some(sim.now);
            }
        }
        let cleared_at = cleared_at.expect("the tree cleared the local costmap");
        let texts = sim.core.texts();
        let tolerated = "[controller_server] RegulatedPurePursuitController detected collision ahead!";
        assert!(texts.iter().any(|t| t.starts_with(tolerated) && t.ends_with(')')), "{texts:#?}");
        assert!(texts.iter().any(|t| t == "[controller_server] Controller patience exceeded"), "{texts:#?}");
        assert!(texts.iter().any(|t| t == "[local_costmap] Received request to clear entirely the local_costmap"), "{texts:#?}");
        assert!(sim.pose().x < 3.0 - 0.5, "drove into the wall: {:?}", sim.pose());
        // the next 2 Hz update makes it current again
        let mut current_at = None;
        while current_at.is_none() && sim.now < cleared_at + SEC {
            sim.step();
            if sim.core.local_current() {
                current_at = Some(sim.now);
            }
        }
        let current_at = current_at.expect("current again");
        assert!(current_at - cleared_at <= LOCAL_UPDATE_NS + 10 * MS, "{} ms", (current_at - cleared_at) / MS);
        sim.core.cancel(nav);
        assert_eq!(sim.run_until_done(nav, 2.0).status, GoalStatus::Canceled);
    }

    #[test]
    fn a_goal_before_any_map_waits_for_the_costmap_and_times_out() {
        let mut sim = Sim::new(Pose2D::new(1.0, 1.0, 0.0));
        let follow = sim.follow(line(sim.pose(), 1.0));
        let o = sim.run_until_done(follow, 2.0);
        assert_eq!((o.status, o.error_code), (GoalStatus::Aborted, controller_server::error_code::CONTROLLER_TIMED_OUT));
        assert_eq!(o.error_msg, "ControllerTimedOut: Costmap timed out waiting for update");
        let texts = sim.core.texts();
        assert!(texts.contains(&"[controller_server] waiting for the local costmap: no local map received".to_string()), "{texts:#?}");
        assert!(texts.contains(&"[controller_server] Costmap timed out waiting for update".to_string()), "{texts:#?}");
        // the maps arrive: the stack built on the placeholder takes them
        let map = grid(10.0, 8.0, &[]);
        sim.core.set_global_map(sim.now, &map);
        sim.core.set_local_map(sim.now, &map);
        let nav = sim.navigate(Pose2D::new(2.5, 1.5, 0.0));
        assert_eq!(sim.run_until_done(nav, 30.0), succeeded());
    }

    #[test]
    fn a_new_global_map_waits_for_its_update_during_a_goal_and_the_planner_follows_its_size() {
        let small = grid(10.0, 8.0, &[]);
        let big = grid(16.0, 8.0, &[]);
        let mut sim = Sim::with_maps(Pose2D::new(1.0, 1.0, 0.0), &small, &small);
        let nav = sim.navigate(Pose2D::new(3.0, 1.5, 0.0));
        sim.run_for(0.3);
        // during the goal: buffered until the next 1 Hz update (mower_map
        // publishes both maps; the local one is resampled at once)
        sim.core.set_global_map(sim.now, &big);
        sim.core.set_local_map(sim.now, &big);
        let update = sim.core.global_next_update_ns().unwrap();
        while sim.now <= update {
            assert_eq!(sim.core.global_size(), Some((200, 160)), "taken before its update");
            sim.step();
        }
        assert_eq!(sim.core.global_size(), Some((320, 160)));
        assert_eq!(sim.run_until_done(nav, 30.0), succeeded());
        // NavFn follows the size: a goal beyond the old map
        let nav = sim.navigate(Pose2D::new(10.0, 1.5, 0.0));
        assert_eq!(sim.run_until_done(nav, 60.0), succeeded());
        assert!(sim.pose().x > 9.9, "{:?}", sim.pose());
        // between goals a map is taken at once: the robot is off it now
        sim.core.set_global_map(sim.now, &small);
        assert_eq!(sim.core.global_size(), Some((200, 160)));
        let nav = sim.navigate(Pose2D::new(5.0, 1.5, 0.0));
        let o = sim.run_until_done(nav, 2.0);
        assert_eq!((o.status, o.error_code), (GoalStatus::Aborted, planner_server::error_code::START_OUTSIDE_MAP));
        assert_eq!(o.error_msg, "StartOutsideMapBounds: Start Coordinates were outside bounds");
    }

    #[test]
    fn stamps_never_go_backward_when_the_wall_clock_does() {
        let map = grid(10.0, 8.0, &[]);
        let mut sim = Sim::with_maps(Pose2D::new(1.0, 1.0, 0.0), &map, &map);
        let follow = sim.follow(line(sim.pose(), 2.0));
        sim.run_for(1.0);
        sim.now = sim.core.now_ns().unwrap() - 300 * MS;
        assert_eq!(sim.run_until_done(follow, 30.0), succeeded());
        assert!(sim.cmd_vel.windows(2).all(|w| w[0].1.stamp_ns <= w[1].1.stamp_ns));
        let texts = sim.core.texts();
        assert_eq!(texts, ["[controller_server] the wall clock went back 0.300 s; navigation time is held until it catches up"]);
    }

    #[test]
    fn missing_inputs_are_reported_once_while_a_goal_runs() {
        let map = grid(10.0, 8.0, &[]);
        let mut core = CoreLoop::new(params());
        core.set_global_map(T0, &map);
        core.set_local_map(T0, &map);
        let follow = GoalKey::Follow(1);
        core.send_follow_path(follow, line(Pose2D::new(1.0, 1.0, 0.0), 1.0));
        let blind = TickInput::default();
        let mut now = T0;
        for _ in 0..30 {
            core.tick(now, &blind);
            now += 10 * MS;
        }
        assert_eq!(
            core.texts(),
            [
                "[local_costmap] update skipped: no robot pose in odom",
                "[controller_server] no odometry yet: the controller takes the robot as standing",
                "[controller_server] no transform odom <- base_footprint: the robot pose is unavailable",
                "[bt_navigator] no transform map <- odom: the robot pose in map is unavailable",
                "[controller_server] waiting for the local costmap: no robot pose in odom",
            ]
        );
    }

    #[test]
    fn the_ring_keeps_the_last_40_and_counts_repeats() {
        let mut core = CoreLoop::new(params());
        for i in 0..45 {
            core.record(T0 + i, format!("e{i}"));
        }
        assert_eq!(core.events().len(), EVENT_RING);
        assert_eq!(core.events().front().unwrap().text, "e5");
        assert_eq!(core.events().front().unwrap().seq, 6);
        let rev = core.events_revision();
        core.record(T0 + 100, "e44");
        core.record(T0 + 200, "e44");
        assert_eq!(core.events().len(), EVENT_RING);
        let last = core.events().back().unwrap();
        assert_eq!((last.count, last.at_ns, last.seq, last.render()), (3, T0 + 200, 45, "e44 (x3)".to_string()));
        assert!(core.events_revision() > rev);
    }

    #[test]
    fn readiness_needs_both_maps_the_pose_and_fresh_odometry() {
        let mut inputs = Inputs::default();
        let timeout = 500 * MS;
        assert_eq!(inputs.readiness(T0, false, timeout), Readiness::default());
        let map = grid(2.0, 2.0, &[]);
        inputs.set_global_map(map.clone());
        let mut bad = map.clone();
        bad.data.pop();
        inputs.set_local_map(bad);
        assert_eq!(inputs.notes, ["[local_costmap] Received map message is malformed. Rejecting."]);
        let r = inputs.readiness(T0, true, timeout);
        assert_eq!(r, Readiness { global_map: true, local_map: false, pose: true, odometry: false });
        inputs.set_local_map(map);
        inputs.set_odom(Twist2D::default(), T0);
        assert!(inputs.readiness(T0 + timeout, true, timeout).ready());
        assert!(!inputs.readiness(T0 + timeout, false, timeout).ready());
        let stale = inputs.readiness(T0 + timeout + 1, true, timeout);
        assert_eq!(stale, Readiness { global_map: true, local_map: true, pose: true, odometry: false });
    }

    #[test]
    fn timers_keep_their_phase_and_skip_what_they_missed() {
        assert_eq!(next_period(0, 50, 0), 50);
        assert_eq!(next_period(0, 50, 50), 50);
        assert_eq!(next_period(0, 50, 51), 100);
        assert_eq!(next_period(0, 50, 120), 150);
        assert_eq!(next_period(0, 50, 150), 150);
    }

    #[test]
    fn error_messages_name_the_exception() {
        assert_eq!(error_msg(0, None), "");
        let collision = ControllerError::NoValidControl("RotationShimController detected collision ahead!");
        assert_eq!(error_msg(106, Some(collision)), "NoValidControl: RotationShimController detected collision ahead!");
        assert_eq!(error_msg(105, None), "FailedToMakeProgress: Failed to make progress");
        assert_eq!(error_msg(208, None), "NoValidPathCouldBeFound: no path to the goal in the global costmap");
        for code in (100..=107).chain(200..=208) {
            assert_ne!(code_text(code), "unknown error code", "{code}");
        }
    }
}
