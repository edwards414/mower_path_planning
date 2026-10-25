//! `nav2_bt_navigator`'s `navigate_to_pose`: the goal bookkeeping of its
//! `nav2_util::SimpleActionServer`, `BtActionServer::executeCallback`,
//! `BehaviorTreeEngine::run` and the behavior tree, ported node by node, as a
//! state machine the caller ticks every `bt_loop_duration` (10 ms).
//!
//! The tree is nav2's default `navigate_to_pose_w_replanning_and_recovery.xml`
//! without its three motion recoveries:
//!
//! * plan at 1 Hz (`RateController`) and follow the newest plan: every new
//!   plan is a new `follow_path` goal, which preempts the running one in the
//!   controller server;
//! * a failed plan is retried once after clearing the global costmap, a failed
//!   `follow_path` once after clearing the local costmap, when the error is
//!   one a recovery could help with (`WouldAPlannerRecoveryHelp`:
//!   UNKNOWN 200, TIMEOUT 207, NO_VALID_PATH 208; `WouldAControllerRecoveryHelp`:
//!   UNKNOWN 100, PATIENCE_EXCEEDED 104, FAILED_TO_MAKE_PROGRESS 105,
//!   NO_VALID_CONTROL 106);
//! * when the pipeline still fails with such an error, both costmaps are
//!   cleared and the whole pipeline is retried, up to 6 times; then the goal
//!   is aborted.
//!
//! Upstream's outer recovery is a `RoundRobin` of [clear both costmaps,
//! Spin 1.57 rad, Wait 5 s, BackUp 0.30 m]; here it is the clearing every
//! time (`docs/ROS_FREE_PLAN.md` §8 D5). Up to the second outer recovery the
//! two trees behave the same. The tree as XML, for running it on the real
//! bt_navigator, is `tests/oracle/navigator/navigate_to_pose_w_replanning_and_clearing.xml`.
//!
//! Kept as upstream does it, quirks included:
//!
//! * clearing a costmap makes it not current until its next update, so the
//!   retry after it waits for that update (the global costmap updates at
//!   1 Hz, and the planner's `costmap_update_timeout` is 1 s). The caller
//!   resets the costmaps the output names.
//! * `GoalUpdated` in the recovery's `ReactiveFallback` never succeeds: its
//!   status is reset on every tick (BehaviorTree.CPP 4.10 resets the
//!   fallback's children after each outcome), and it reports an update only
//!   when ticked while active. A preempting goal is planned to at the next
//!   replan, up to a second later; a recovery under way runs to the end.
//! * the result's error code is the smallest nonzero one of
//!   `compute_path_error_code` and `follow_path_error_code`
//!   (`populateErrorCode`), so a FollowPath code (1xx) beats a planner code
//!   (2xx), even one left over from an earlier attempt.
//! * every costmap clear counts in the feedback's `number_of_recoveries`.
//! * the navigate goal's own `SimpleActionServer` quirks are the controller
//!   server's (see [`crate::controller_server`]): one pending slot, a
//!   preempted goal is aborted with error code 0, `is_cancel_requested`
//!   looks at the pending goal when there is one. A pending goal is accepted
//!   only after a tick that leaves the tree running (`onPreempt` runs from
//!   the loop callback).
//!
//! Not modelled: the result and feedback callbacks' wake-up (upstream ticks
//! the tree again at once when a result arrives during a tick; here it is
//! seen at the next tick, up to `bt_loop_duration` later); service and goal
//! response timeouts; `behavior_tree` in the goal (mower_nav leaves it
//! empty: this tree); the feedback's `estimated_time_remaining` (no
//! consumer); the goal's frame (poses are in `map`, the caller transforms).
//!
//! Upstream: `nav2_bt_navigator/src/navigators/navigate_to_pose.cpp`,
//! `nav2_behavior_tree/include/nav2_behavior_tree/{bt_action_server_impl,bt_action_node}.hpp`,
//! `nav2_behavior_tree/src/behavior_tree_engine.cpp`,
//! `nav2_behavior_tree/plugins/{action,condition,control,decorator}/*`,
//! BehaviorTree.CPP 4.10 `src/controls/*`.

use std::collections::VecDeque;

use crate::action::{GoalId, GoalStatus};
use crate::controller_server::{self, FollowPathResult};
use crate::geometry::{euclidean_distance, Pose2D};
use crate::path_handler::{calculate_path_length, Path};
use crate::planner_server::{self, ComputePathResult, ComputePathToPose};

/// The two costmaps the tree clears.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CostmapId {
    /// `local_costmap/clear_entirely_local_costmap`
    Local,
    /// `global_costmap/clear_entirely_global_costmap`
    Global,
}

/// What the tree's action and service nodes call.
pub trait NavServers {
    /// `compute_path_to_pose` goal.
    fn send_compute_path(&mut self, goal: ComputePathToPose) -> GoalId;
    fn cancel_compute_path(&mut self, goal: GoalId);
    /// `follow_path` goal.
    fn send_follow_path(&mut self, path: Path) -> GoalId;
    fn cancel_follow_path(&mut self, goal: GoalId);
    /// `ClearEntireCostmap`: the costmap is reset and is not current until
    /// its next update repaints the static layer.
    fn clear_costmap(&mut self, costmap: CostmapId);
}

/// `bt_navigator`'s parameters and the tree's ports.
#[derive(Debug, Clone, PartialEq)]
pub struct NavigatorParams {
    /// `bt_loop_duration`, ms: the caller's tick period.
    pub bt_loop_duration_ms: i64,
    /// `RateController hz`.
    pub replan_frequency: f64,
    /// `NavigateRecovery number_of_retries`: whole-pipeline retries.
    pub number_of_retries: u32,
    /// The `ComputePathToPose` recovery's `number_of_retries`.
    pub planner_retries: u32,
    /// The `FollowPath` recovery's `number_of_retries`.
    pub controller_retries: u32,
    /// `PlannerSelector default_planner`.
    pub planner_id: String,
}

impl Default for NavigatorParams {
    /// Upstream's defaults, which `nav2_no_map_params.yaml` keeps.
    fn default() -> Self {
        Self {
            bt_loop_duration_ms: 10,
            replan_frequency: 1.0,
            number_of_retries: 6,
            planner_retries: 1,
            controller_retries: 1,
            planner_id: "GridBased".into(),
        }
    }
}

/// A goal's result (`NavigateToPose.action`; 1.3.13 leaves `error_msg` empty).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NavigateResult {
    pub goal: GoalId,
    pub status: GoalStatus,
    /// A FollowPath (1xx) or ComputePathToPose (2xx) code, or 0.
    pub error_code: u16,
}

/// `NavigateToPose.action`'s feedback, less `estimated_time_remaining`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavigateFeedback {
    /// The robot in `map`.
    pub current_pose: Pose2D,
    /// Along the current plan from its pose closest to the robot; 0 while
    /// there is none.
    pub distance_remaining: f32,
    pub navigation_time_ns: i64,
    pub number_of_recoveries: i16,
}

/// What a tick reads.
#[derive(Debug, Clone, Copy)]
pub struct NavInput {
    pub now_ns: i64,
    /// tf `map <- base_footprint` (latest), `None` if the lookup failed.
    pub robot_pose: Option<Pose2D>,
}

/// What a tick produced.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct NavOutput {
    /// Goals that ended, oldest first.
    pub results: Vec<NavigateResult>,
    pub feedback: Option<(GoalId, NavigateFeedback)>,
}

fn would_a_controller_recovery_help(code: u16) -> bool {
    use controller_server::error_code::*;
    matches!(code, UNKNOWN | PATIENCE_EXCEEDED | FAILED_TO_MAKE_PROGRESS | NO_VALID_CONTROL)
}

/// Also ComputePathThroughPoses' UNKNOWN 300, TIMEOUT 307, NO_VALID_PATH 308.
fn would_a_planner_recovery_help(code: u16) -> bool {
    use planner_server::error_code::*;
    matches!(code, UNKNOWN | NO_VALID_PATH | TIMEOUT | 300 | 307 | 308)
}

// --- the tree ------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Status {
    #[default]
    Idle,
    Running,
    Success,
    Failure,
}

/// The three `RecoveryNode`s.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Rec {
    /// `NavigateRecovery`: the pipeline, then the outer recovery.
    Navigate,
    /// `ComputePathToPose`, then clear the global costmap.
    ComputePath,
    /// `FollowPath`, then clear the local costmap.
    FollowPath,
}

#[derive(Debug, Clone, Copy, Default)]
struct RecoveryState {
    status: Status,
    current_child_idx: usize,
    retry_count: u32,
}

#[derive(Debug, Clone, Default)]
struct Blackboard {
    goal: Pose2D,
    path: Path,
    /// Bumped on every write of `path`: upstream compares paths with their
    /// stamps, so every new plan differs from the last.
    path_version: u64,
    compute_path_error_code: u16,
    follow_path_error_code: u16,
    number_recoveries: i16,
}

/// A `BtActionNode`.
#[derive(Debug, Clone)]
struct ActionNode<R> {
    status: Status,
    /// `goal_handle_`.
    goal: Option<GoalId>,
    /// The result, once it arrived (`goal_result_available_`).
    result: Option<R>,
}

impl<R> Default for ActionNode<R> {
    fn default() -> Self {
        Self { status: Status::Idle, goal: None, result: None }
    }
}

#[derive(Debug, Clone)]
struct Tree {
    params: NavigatorParams,
    bb: Blackboard,
    navigate: RecoveryState,
    /// `NavigateWithReplanning`'s status and `last_child_ticked_`.
    pipeline: (Status, usize),
    /// `RateController`'s status, `start_` and `first_time_`.
    rate: (Status, i64, bool),
    compute_recovery: RecoveryState,
    compute: ActionNode<ComputePathResult>,
    follow_recovery: RecoveryState,
    follow: ActionNode<FollowPathResult>,
    /// FollowPath's `goal_.path`, as its version.
    follow_path_version: u64,
    /// FollowPath's `goal_updated_` (not reset between goals upstream).
    follow_goal_updated: bool,
}

struct Ctx<'a> {
    now: i64,
    servers: &'a mut dyn NavServers,
}

impl Tree {
    fn new(params: NavigatorParams) -> Self {
        Self {
            params,
            bb: Blackboard::default(),
            navigate: RecoveryState::default(),
            pipeline: (Status::Idle, 0),
            rate: (Status::Idle, 0, false),
            compute_recovery: RecoveryState::default(),
            compute: ActionNode::default(),
            follow_recovery: RecoveryState::default(),
            follow: ActionNode::default(),
            follow_path_version: 0,
            follow_goal_updated: false,
        }
    }

    /// `tree.tickOnce()`.
    fn tick(&mut self, c: &mut Ctx) -> Status {
        self.tick_recovery(Rec::Navigate, c)
    }

    fn set_path(&mut self, path: Path) {
        self.bb.path = path;
        self.bb.path_version += 1;
    }

    fn clear(&mut self, costmap: CostmapId, c: &mut Ctx) {
        // ClearEntireCostmapService::on_tick: increment_recovery_count()
        self.bb.number_recoveries = self.bb.number_recoveries.saturating_add(1);
        c.servers.clear_costmap(costmap);
    }

    // RecoveryNode ------------------------------------------------------------

    fn recovery(&mut self, r: Rec) -> &mut RecoveryState {
        match r {
            Rec::Navigate => &mut self.navigate,
            Rec::ComputePath => &mut self.compute_recovery,
            Rec::FollowPath => &mut self.follow_recovery,
        }
    }

    fn tick_recovery(&mut self, r: Rec, c: &mut Ctx) -> Status {
        let s = self.recovery_tick(r, c);
        self.recovery(r).status = s;
        s
    }

    fn recovery_tick(&mut self, r: Rec, c: &mut Ctx) -> Status {
        let retries = match r {
            Rec::Navigate => self.params.number_of_retries,
            Rec::ComputePath => self.params.planner_retries,
            Rec::FollowPath => self.params.controller_retries,
        };
        self.recovery(r).status = Status::Running;
        while self.recovery(r).current_child_idx < 2 && self.recovery(r).retry_count <= retries {
            if self.recovery(r).current_child_idx == 0 {
                match self.tick_recovery_child(r, 0, c) {
                    Status::Success => {
                        self.halt_recovery_child(r, 1, c);
                        self.halt_recovery(r, c);
                        return Status::Success;
                    }
                    Status::Running => return Status::Running,
                    Status::Failure => {
                        if self.recovery(r).retry_count < retries {
                            self.halt_recovery_child(r, 0, c);
                            self.recovery(r).current_child_idx += 1;
                        } else {
                            self.halt_recovery(r, c);
                            return Status::Failure;
                        }
                    }
                    Status::Idle => unreachable!("a child never returns IDLE"),
                }
            } else {
                match self.tick_recovery_child(r, 1, c) {
                    Status::Running => return Status::Running,
                    Status::Success => {
                        self.halt_recovery_child(r, 1, c);
                        let st = self.recovery(r);
                        st.retry_count += 1;
                        st.current_child_idx = 0;
                    }
                    Status::Failure => {
                        self.halt_recovery(r, c);
                        return Status::Failure;
                    }
                    Status::Idle => unreachable!("a child never returns IDLE"),
                }
            }
        }
        self.halt_recovery(r, c);
        Status::Failure
    }

    fn tick_recovery_child(&mut self, r: Rec, i: usize, c: &mut Ctx) -> Status {
        match (r, i) {
            (Rec::Navigate, 0) => self.tick_pipeline(c),
            (Rec::Navigate, _) => self.recovery_actions(c),
            (Rec::ComputePath, 0) => self.tick_compute(c),
            // Sequence: WouldAPlannerRecoveryHelp, ClearGlobalCostmap-Context
            (Rec::ComputePath, _) => {
                if !would_a_planner_recovery_help(self.bb.compute_path_error_code) {
                    return Status::Failure;
                }
                self.clear(CostmapId::Global, c);
                Status::Success
            }
            (Rec::FollowPath, 0) => self.tick_follow(c),
            // Sequence: WouldAControllerRecoveryHelp, ClearLocalCostmap-Context
            (Rec::FollowPath, _) => {
                if !would_a_controller_recovery_help(self.bb.follow_path_error_code) {
                    return Status::Failure;
                }
                self.clear(CostmapId::Local, c);
                Status::Success
            }
        }
    }

    /// `ControlNode::haltChild(i)`: halt it if RUNNING, reset its status.
    /// The second children (conditions and clears) never stay RUNNING here.
    fn halt_recovery_child(&mut self, r: Rec, i: usize, c: &mut Ctx) {
        if i != 0 {
            return;
        }
        match r {
            Rec::Navigate => self.halt_child_pipeline(c),
            Rec::ComputePath => self.halt_child_action(true, c),
            Rec::FollowPath => self.halt_child_action(false, c),
        }
    }

    /// `RecoveryNode::halt`: `ControlNode::halt` (halt the RUNNING children,
    /// reset them and itself), then the retry count and the child index.
    fn halt_recovery(&mut self, r: Rec, c: &mut Ctx) {
        self.halt_recovery_child(r, 0, c);
        *self.recovery(r) = RecoveryState::default();
    }

    fn halt_child_recovery(&mut self, r: Rec, c: &mut Ctx) {
        if self.recovery(r).status == Status::Running {
            self.halt_recovery(r, c);
        }
        self.recovery(r).status = Status::Idle;
    }

    // The outer recovery ----------------------------------------------------

    /// `Sequence[Fallback[WouldAControllerRecoveryHelp,
    /// WouldAPlannerRecoveryHelp], ReactiveFallback[GoalUpdated,
    /// RoundRobin[ClearingActions]]]`. `GoalUpdated` fails here (see the
    /// module doc), and the round robin's one child clears both costmaps.
    fn recovery_actions(&mut self, c: &mut Ctx) -> Status {
        if !(would_a_controller_recovery_help(self.bb.follow_path_error_code)
            || would_a_planner_recovery_help(self.bb.compute_path_error_code))
        {
            return Status::Failure;
        }
        self.clear(CostmapId::Local, c);
        self.clear(CostmapId::Global, c);
        Status::Success
    }

    // PipelineSequence ------------------------------------------------------

    fn tick_pipeline(&mut self, c: &mut Ctx) -> Status {
        let s = self.pipeline_tick(c);
        self.pipeline.0 = s;
        s
    }

    fn pipeline_tick(&mut self, c: &mut Ctx) -> Status {
        // children 0 and 1, ControllerSelector and PlannerSelector, succeed:
        // nothing publishes on controller_selector / planner_selector
        for i in 2..4 {
            let s = if i == 2 { self.tick_rate(c) } else { self.tick_recovery(Rec::FollowPath, c) };
            match s {
                Status::Failure => {
                    self.halt_pipeline_children(c);
                    self.pipeline.1 = 0;
                    return Status::Failure;
                }
                Status::Success => {}
                Status::Running => {
                    if i >= self.pipeline.1 {
                        self.pipeline.1 = i;
                        return Status::Running;
                    }
                }
                Status::Idle => unreachable!("a child never returns IDLE"),
            }
        }
        self.halt_pipeline_children(c);
        self.pipeline.1 = 0;
        Status::Success
    }

    fn halt_pipeline_children(&mut self, c: &mut Ctx) {
        self.halt_child_rate(c);
        self.halt_child_recovery(Rec::FollowPath, c);
    }

    fn halt_child_pipeline(&mut self, c: &mut Ctx) {
        if self.pipeline.0 == Status::Running {
            // PipelineSequence::halt
            self.halt_pipeline_children(c);
            self.pipeline.1 = 0;
        }
        self.pipeline.0 = Status::Idle;
    }

    // RateController --------------------------------------------------------

    fn tick_rate(&mut self, c: &mut Ctx) -> Status {
        let s = self.rate_tick(c);
        self.rate.0 = s;
        s
    }

    fn rate_tick(&mut self, c: &mut Ctx) -> Status {
        if self.rate.0 == Status::Idle {
            self.rate.1 = c.now;
            self.rate.2 = true;
        }
        self.rate.0 = Status::Running;
        // a steady clock in float seconds upstream; the tick clock here
        let elapsed = (c.now - self.rate.1) as f64 / 1e9;
        let period = 1.0 / self.params.replan_frequency;
        if self.rate.2 || self.compute_recovery.status == Status::Running || elapsed >= period {
            self.rate.2 = false;
            let s = self.tick_recovery(Rec::ComputePath, c);
            if s == Status::Success {
                // Reset the timer
                self.rate.1 = c.now;
            }
            return s;
        }
        Status::Running
    }

    fn halt_child_rate(&mut self, c: &mut Ctx) {
        if self.rate.0 == Status::Running {
            // DecoratorNode::halt: resetChild
            self.halt_child_recovery(Rec::ComputePath, c);
        }
        self.rate.0 = Status::Idle;
    }

    // ComputePathToPose -----------------------------------------------------

    fn tick_compute(&mut self, c: &mut Ctx) -> Status {
        let s = self.compute_tick(c);
        self.compute.status = s;
        s
    }

    fn compute_tick(&mut self, c: &mut Ctx) -> Status {
        if self.compute.status == Status::Idle {
            // on_tick: goal = {goal}, planner_id = {selected_planner},
            // use_start = false; then send_new_goal
            let goal = ComputePathToPose { goal: self.bb.goal, start: None, planner_id: self.params.planner_id.clone() };
            self.compute.status = Status::Running;
            self.compute.result = None;
            self.compute.goal = Some(c.servers.send_compute_path(goal));
        }
        let Some(r) = self.compute.result.take() else {
            return Status::Running;
        };
        self.compute.goal = None;
        match r.status {
            GoalStatus::Succeeded => {
                self.set_path(r.path);
                self.bb.compute_path_error_code = planner_server::error_code::NONE;
                Status::Success
            }
            GoalStatus::Aborted => {
                self.set_path(Path::new());
                self.bb.compute_path_error_code = r.error_code;
                Status::Failure
            }
            GoalStatus::Canceled => {
                self.set_path(Path::new());
                self.bb.compute_path_error_code = planner_server::error_code::NONE;
                Status::Success
            }
        }
    }

    // FollowPath ------------------------------------------------------------

    fn tick_follow(&mut self, c: &mut Ctx) -> Status {
        let s = self.follow_tick(c);
        self.follow.status = s;
        s
    }

    fn follow_tick(&mut self, c: &mut Ctx) -> Status {
        if self.follow.status == Status::Idle {
            // on_tick: path = {path}, controller_id = {selected_controller}
            self.follow_path_version = self.bb.path_version;
            self.follow.status = Status::Running;
            self.follow.result = None;
            self.follow.goal = Some(c.servers.send_follow_path(self.bb.path.clone()));
        }
        if self.follow.result.is_none() {
            // on_wait_for_result: a new, non-empty plan updates the goal
            if self.bb.path_version != self.follow_path_version && !self.bb.path.is_empty() {
                self.follow_path_version = self.bb.path_version;
                self.follow_goal_updated = true;
            }
            // the goal is EXECUTING (or ACCEPTED) until its result arrives
            if self.follow_goal_updated {
                self.follow_goal_updated = false;
                self.follow.goal = Some(c.servers.send_follow_path(self.bb.path.clone()));
            }
            return Status::Running;
        }
        let r = self.follow.result.take().expect("checked above");
        self.follow.goal = None;
        match r.status {
            GoalStatus::Succeeded => {
                self.bb.follow_path_error_code = controller_server::error_code::NONE;
                Status::Success
            }
            GoalStatus::Aborted => {
                self.bb.follow_path_error_code = r.error_code;
                Status::Failure
            }
            GoalStatus::Canceled => {
                self.bb.follow_path_error_code = controller_server::error_code::NONE;
                Status::Success
            }
        }
    }

    /// `haltChild` of ComputePathToPose (`compute`) or FollowPath.
    fn halt_child_action(&mut self, compute: bool, c: &mut Ctx) {
        let running = if compute { self.compute.status } else { self.follow.status } == Status::Running;
        if running {
            self.halt_action(compute, c);
        }
        if compute {
            self.compute.status = Status::Idle;
        } else {
            self.follow.status = Status::Idle;
        }
    }

    /// `ComputePathToPoseAction::halt` / `BtActionNode::halt`: a goal still
    /// executing is cancelled (`on_cancelled`), then the status is reset.
    fn halt_action(&mut self, compute: bool, c: &mut Ctx) {
        if compute {
            self.set_path(Path::new());
            if self.compute.status == Status::Running && self.compute.result.is_none() {
                if let Some(id) = self.compute.goal {
                    c.servers.cancel_compute_path(id);
                    self.set_path(Path::new());
                    self.bb.compute_path_error_code = planner_server::error_code::NONE;
                }
            }
            self.compute = ActionNode::default();
        } else {
            if self.follow.status == Status::Running && self.follow.result.is_none() {
                if let Some(id) = self.follow.goal {
                    c.servers.cancel_follow_path(id);
                    self.bb.follow_path_error_code = controller_server::error_code::NONE;
                }
            }
            self.follow = ActionNode::default();
        }
    }

    /// `Tree::haltTree()`: the root's halt reaches every RUNNING node; the
    /// visitor then halts every node, which resets the rest and, through
    /// `ComputePathToPoseAction::halt`, empties `{path}`.
    fn halt_tree(&mut self, c: &mut Ctx) {
        self.halt_recovery(Rec::Navigate, c);
        self.halt_action(true, c);
        self.halt_action(false, c);
        self.pipeline = (Status::Idle, 0);
        self.rate.0 = Status::Idle;
        self.compute_recovery = RecoveryState::default();
        self.follow_recovery = RecoveryState::default();
    }

    /// `populateErrorCode`: the smallest nonzero code, or 0.
    fn error_code(&self) -> u16 {
        [self.bb.compute_path_error_code, self.bb.follow_path_error_code].into_iter().filter(|&c| c != 0).min().unwrap_or(0)
    }

    fn clean_error_codes(&mut self) {
        self.bb.compute_path_error_code = 0;
        self.bb.follow_path_error_code = 0;
    }
}

// --- the action server ---------------------------------------------------------

#[derive(Debug, Clone, Copy)]
struct GoalHandle {
    id: GoalId,
    pose: Pose2D,
    canceling: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BtStatus {
    Succeeded,
    Failed,
    Canceled,
}

/// `bt_navigator` serving `navigate_to_pose`.
#[derive(Debug, Clone)]
pub struct Navigator {
    tree: Tree,
    next_goal: GoalId,
    current: Option<GoalHandle>,
    pending: Option<GoalHandle>,
    preempt_requested: bool,
    /// `executeCallback` runs the tree for `current`.
    running: bool,
    /// `start_time_`: when the goal pose was set.
    start_time_ns: i64,
    backlog: VecDeque<NavigateResult>,
    runs_started: u64,
}

impl Navigator {
    /// `bt_navigator` as `nav2_no_map_params.yaml` configures it.
    pub fn mower() -> Self {
        Self::new(NavigatorParams::default())
    }

    pub fn new(params: NavigatorParams) -> Self {
        Self {
            tree: Tree::new(params),
            next_goal: 1,
            current: None,
            pending: None,
            preempt_requested: false,
            running: false,
            start_time_ns: 0,
            backlog: VecDeque::new(),
            runs_started: 0,
        }
    }

    pub fn params(&self) -> &NavigatorParams {
        &self.tree.params
    }

    pub fn current_goal(&self) -> Option<GoalId> {
        self.current.map(|g| g.id)
    }

    pub fn pending_goal(&self) -> Option<GoalId> {
        self.pending.map(|g| g.id)
    }

    /// A goal is executing: tick every `bt_loop_duration`.
    pub fn is_executing(&self) -> bool {
        self.current.is_some()
    }

    /// Something to do on the next tick (a goal, or a result to report).
    pub fn has_work(&self) -> bool {
        self.current.is_some() || !self.backlog.is_empty()
    }

    /// How many times `executeCallback` has started the tree. Upstream's
    /// loop rate (a `WallRate` built when it starts) starts over when this
    /// goes up; a preemption stays in the same run.
    pub fn runs_started(&self) -> u64 {
        self.runs_started
    }

    /// The `compute_path_to_pose` goal the tree waits on.
    pub fn compute_path_goal(&self) -> Option<GoalId> {
        self.tree.compute.goal
    }

    /// The `follow_path` goal the tree waits on.
    pub fn follow_path_goal(&self) -> Option<GoalId> {
        self.tree.follow.goal
    }

    /// `handle_goal` + `handle_accepted`, as in the controller server.
    pub fn send_goal(&mut self, pose: Pose2D) -> GoalId {
        let id = self.next_goal;
        self.next_goal += 1;
        let handle = GoalHandle { id, pose, canceling: false };
        if let Some(old) = self.pending.take() {
            self.backlog.push_back(terminate(old, 0));
        }
        if self.current.is_some() {
            self.pending = Some(handle);
            self.preempt_requested = true;
        } else {
            self.preempt_requested = false;
            self.current = Some(handle);
            self.running = false;
        }
        id
    }

    /// `handle_cancel`: accepted while the goal is executing or pending.
    pub fn cancel_goal(&mut self, goal: GoalId) -> bool {
        for handle in [&mut self.current, &mut self.pending].into_iter().flatten() {
            if handle.id == goal {
                handle.canceling = true;
                return true;
            }
        }
        false
    }

    /// The result of a `compute_path_to_pose` goal. Only the one the tree
    /// waits on is kept (`result_callback` compares goal ids).
    pub fn on_compute_path_result(&mut self, r: ComputePathResult) {
        if self.tree.compute.goal == Some(r.goal) && self.tree.compute.result.is_none() {
            self.tree.compute.result = Some(r);
        }
    }

    /// The result of a `follow_path` goal; as above.
    pub fn on_follow_path_result(&mut self, r: FollowPathResult) {
        if self.tree.follow.goal == Some(r.goal) && self.tree.follow.result.is_none() {
            self.tree.follow.result = Some(r);
        }
    }

    /// One iteration of `BehaviorTreeEngine::run` for the executing goal
    /// (starting it first if it is new), then the goals pending behind it
    /// when it ends, as `SimpleActionServer::work()` runs them.
    pub fn tick(&mut self, input: &NavInput, servers: &mut dyn NavServers) -> NavOutput {
        let mut out = NavOutput { results: self.backlog.drain(..).collect(), feedback: None };
        let mut c = Ctx { now: input.now_ns, servers };
        while self.current.is_some() {
            if !self.running {
                // executeCallback: on_goal_received (the tree is loaded),
                // initializeGoalPose
                self.runs_started += 1;
                let pose = self.current.map(|g| g.pose).expect("a current goal");
                if self.initialize_goal_pose(pose, input) {
                    self.running = true;
                } else {
                    // "Initial robot pose is not available."
                    self.terminate_current(0, &mut out);
                    self.tree.clean_error_codes();
                }
            }
            if self.running {
                let rc = if self.is_cancel_requested() {
                    self.tree.halt_tree(&mut c);
                    BtStatus::Canceled
                } else {
                    match self.tree.tick(&mut c) {
                        Status::Running => {
                            self.on_loop(input, &mut out);
                            return out;
                        }
                        Status::Success => BtStatus::Succeeded,
                        _ => BtStatus::Failed,
                    }
                };
                // haltAllActions, populateErrorCode, the result
                self.tree.halt_tree(&mut c);
                let error_code = self.tree.error_code();
                match rc {
                    BtStatus::Succeeded => {
                        if let Some(cur) = self.current.take() {
                            out.results.push(NavigateResult { goal: cur.id, status: GoalStatus::Succeeded, error_code });
                        }
                    }
                    BtStatus::Failed => self.terminate_current(error_code, &mut out),
                    BtStatus::Canceled => self.terminate_all(error_code, &mut out),
                }
                self.tree.clean_error_codes();
                self.running = false;
            }
            if let Some(cur) = self.current.take() {
                // "Current goal was not completed successfully."
                out.results.push(terminate(cur, 0));
            }
            if self.pending.is_some() {
                self.accept_pending_goal(&mut out);
            } else {
                break;
            }
        }
        out
    }

    /// `initializeGoalPose`: needs the robot pose; resets the recovery count
    /// and the navigation time and puts the goal on the blackboard.
    fn initialize_goal_pose(&mut self, pose: Pose2D, input: &NavInput) -> bool {
        if input.robot_pose.is_none() {
            return false;
        }
        // "Begin navigating from current location (x, y) to (x, y)"
        self.start_time_ns = input.now_ns;
        self.tree.bb.number_recoveries = 0;
        self.tree.bb.goal = pose;
        true
    }

    /// The loop callback after a tick that left the tree running: a pending
    /// goal preempts (`onPreempt`), then the feedback (`onLoop`).
    fn on_loop(&mut self, input: &NavInput, out: &mut NavOutput) {
        if self.preempt_requested {
            self.accept_pending_goal(out);
            let pose = self.current.map(|g| g.pose).expect("a current goal");
            // on failure upstream rejects the (already accepted) goal and
            // keeps the old pose on the blackboard
            self.initialize_goal_pose(pose, input);
        }
        let (Some(current_pose), Some(cur)) = (input.robot_pose, self.current) else {
            // "Robot pose is not available."
            return;
        };
        let path = &self.tree.bb.path;
        let mut distance_remaining = 0.0;
        if !path.is_empty() {
            let mut closest = 0;
            let mut min_dist = f64::MAX;
            for (i, p) in path.iter().enumerate() {
                let d = euclidean_distance(current_pose.x, current_pose.y, p.x, p.y);
                if d < min_dist {
                    min_dist = d;
                    closest = i;
                }
            }
            distance_remaining = calculate_path_length(&path[closest..]);
        }
        out.feedback = Some((
            cur.id,
            NavigateFeedback {
                current_pose,
                distance_remaining: distance_remaining as f32,
                navigation_time_ns: input.now_ns - self.start_time_ns,
                number_of_recoveries: self.tree.bb.number_recoveries,
            },
        ));
    }

    // --- SimpleActionServer ----------------------------------------------------

    fn is_cancel_requested(&self) -> bool {
        match (&self.current, &self.pending) {
            (None, _) => false,
            (Some(_), Some(pending)) => pending.canceling,
            (Some(current), None) => current.canceling,
        }
    }

    /// The running goal (if any) is aborted with an empty result.
    fn accept_pending_goal(&mut self, out: &mut NavOutput) {
        let Some(pending) = self.pending.take() else {
            return;
        };
        if let Some(cur) = self.current.take() {
            out.results.push(NavigateResult { goal: cur.id, status: GoalStatus::Aborted, error_code: 0 });
        }
        self.current = Some(pending);
        self.preempt_requested = false;
    }

    fn terminate_current(&mut self, error_code: u16, out: &mut NavOutput) {
        if let Some(cur) = self.current.take() {
            out.results.push(terminate(cur, error_code));
        }
    }

    fn terminate_all(&mut self, error_code: u16, out: &mut NavOutput) {
        for handle in [self.current.take(), self.pending.take()].into_iter().flatten() {
            out.results.push(terminate(handle, error_code));
        }
        self.preempt_requested = false;
    }
}

/// `terminate(handle, result)`: canceled if the client asked, else aborted.
fn terminate(handle: GoalHandle, error_code: u16) -> NavigateResult {
    let status = if handle.canceling { GoalStatus::Canceled } else { GoalStatus::Aborted };
    NavigateResult { goal: handle.id, status, error_code }
}
