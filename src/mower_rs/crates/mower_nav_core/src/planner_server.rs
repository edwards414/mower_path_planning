//! `nav2_planner::PlannerServer`: the `compute_path_to_pose` action
//! (`computePlan`) with its one plugin, `GridBased` =
//! [`crate::planner::NavfnPlanner`], and the goal bookkeeping of the
//! `nav2_util::SimpleActionServer` it runs in, as a state machine the caller
//! ticks.
//!
//! [`PlannerServer::tick`] runs `computePlan` for the executing goal, in
//! upstream's order: the cancel request, the wait for a current costmap, a
//! pending goal preempting the running one, the start pose (the robot unless
//! the goal gives one), the plugin, `createPlan`, the empty-path check. One
//! tick plans a goal to the end (the plan itself takes no time here; upstream
//! runs it on the action server's thread), and a goal that was pending
//! behind it starts right away, as upstream's worker loop does. While the
//! costmap is not current the goal waits: tick again (upstream polls at
//! 100 Hz) until it is current, or `costmap_update_timeout` has passed
//! (`PlannerTimedOut`, error code 207).
//!
//! The action server's semantics are kept, quirks included:
//!
//! * a goal sent while one executes waits in the single pending slot (a goal
//!   already waiting there is aborted, with an empty result).
//! * the pending goal is taken after the costmap wait, which aborts the
//!   running goal with an empty result, but the pass still plans for the goal
//!   it started with (`getPreemptedGoalIfRequested` assigns to a copy of the
//!   goal pointer) and reports that plan as the new goal's result.
//! * `is_cancel_requested` looks at the pending goal when there is one; a
//!   cancel ends every goal (`terminate_all`) with an empty result, canceled
//!   if its client asked, otherwise aborted. A cancel is seen when the goal
//!   starts and inside NavFn's propagation, not during the costmap wait.
//!
//! The ROS plumbing is the caller's: poses are in the global costmap's frame
//! (`map`; upstream transforms the goal there, the caller does here), the
//! robot pose comes in [`PlanInput`] (`None` when the lookup failed), and the
//! plan is returned for `/plan`, not published. Results carry no
//! `planning_time` beyond the costmap wait (planning is instant here).
//!
//! Upstream: `nav2_planner/src/planner_server.cpp`,
//! `nav2_util/simple_action_server.hpp`,
//! `nav2_msgs/action/ComputePathToPose.action`.

use std::collections::VecDeque;

use crate::action::{GoalId, GoalStatus};
use crate::checkers::duration_from_seconds;
use crate::costmap::Costmap2D;
use crate::geometry::Pose2D;
use crate::path_handler::Path;
use crate::planner::{compute_path_to_pose, NavfnParams, NavfnPlanner, PlannerError};

/// `ComputePathToPose.action`'s result error codes.
pub mod error_code {
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 200;
    pub const INVALID_PLANNER: u16 = 201;
    pub const TF_ERROR: u16 = 202;
    pub const START_OUTSIDE_MAP: u16 = 203;
    pub const GOAL_OUTSIDE_MAP: u16 = 204;
    pub const START_OCCUPIED: u16 = 205;
    pub const GOAL_OCCUPIED: u16 = 206;
    pub const TIMEOUT: u16 = 207;
    pub const NO_VALID_PATH: u16 = 208;
}

/// The error code `computePlan`'s catch blocks give each exception (a cancel
/// ends the goal with an empty result: `NONE`).
pub fn error_code_of(e: &PlannerError) -> u16 {
    e.compute_path_to_pose_code().unwrap_or(error_code::NONE)
}

/// The planner server's own parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct PlannerServerParams {
    /// Hz; a plan that took longer than its period is reported
    /// ([`PlanOutput::missed_rate`]). 0 turns that off.
    pub expected_planner_frequency: f64,
    /// seconds
    pub costmap_update_timeout: f64,
    /// `planner_plugins`: the name the one plugin answers to. A goal may
    /// also leave its `planner_id` empty.
    pub planner_id: String,
}

impl Default for PlannerServerParams {
    /// Upstream's declared defaults.
    fn default() -> Self {
        Self { expected_planner_frequency: 1.0, costmap_update_timeout: 1.0, planner_id: "GridBased".into() }
    }
}

impl PlannerServerParams {
    /// `planner_server` in `nav2_no_map_params.yaml`.
    pub fn mower() -> Self {
        Self { expected_planner_frequency: 20.0, ..Self::default() }
    }
}

/// A `compute_path_to_pose` goal.
#[derive(Debug, Clone, PartialEq)]
pub struct ComputePathToPose {
    /// In the global costmap's frame.
    pub goal: Pose2D,
    /// `use_start` with `start`; `None` plans from the robot.
    pub start: Option<Pose2D>,
    pub planner_id: String,
}

impl ComputePathToPose {
    /// From the robot to `goal`, on the only plugin.
    pub fn to(goal: Pose2D) -> Self {
        Self { goal, start: None, planner_id: String::new() }
    }
}

/// A goal's result.
#[derive(Debug, Clone, PartialEq)]
pub struct ComputePathResult {
    pub goal: GoalId,
    pub status: GoalStatus,
    /// [`error_code`]: `NONE` for success and for the empty results
    /// (cancelled, preempted, replaced in the pending slot).
    pub error_code: u16,
    /// The exception that ended it, for the log.
    pub error: Option<PlannerError>,
    /// In the global costmap's frame; empty unless it succeeded.
    pub path: Path,
    /// From the start of `computePlan` to the result: the costmap wait.
    pub planning_time_ns: i64,
}

/// What a pass reads from tf and the global costmap.
#[derive(Debug)]
pub struct PlanInput<'a> {
    pub now_ns: i64,
    /// `costmap_ros_->isCurrent()`.
    pub costmap_current: bool,
    /// `costmap_ros_->getRobotPose()`: the base in the costmap frame, `None`
    /// if the lookup failed.
    pub robot_pose: Option<Pose2D>,
    /// The global costmap. NavFn writes to it (`clearRobotCell`, see
    /// [`crate::planner`]).
    pub costmap: &'a mut Costmap2D,
}

/// What a pass produced.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PlanOutput {
    /// Goals that ended, oldest first, including a pending goal replaced by
    /// [`PlannerServer::send_goal`] since the last pass.
    pub results: Vec<ComputePathResult>,
    /// The plans found, in order (`publishPlan`, on `/plan`).
    pub plans: Vec<Path>,
    /// Seconds a plan took when that missed `expected_planner_frequency`
    /// (`RCLCPP_WARN`).
    pub missed_rate: Option<f64>,
    /// The goal waits for the costmap: tick again (upstream polls at
    /// 100 Hz) until it is current or the timeout ends the goal.
    pub waiting_for_costmap: bool,
}

#[derive(Debug, Clone)]
struct GoalHandle {
    id: GoalId,
    goal: ComputePathToPose,
    canceling: bool,
}

/// `computePlan`'s state while it runs.
#[derive(Debug, Clone)]
struct Run {
    /// The goal it plans for (see the module doc on preemption).
    goal: ComputePathToPose,
    start_ns: i64,
    waiting_since_ns: Option<i64>,
}

enum Pass {
    Waiting,
    Returned,
}

/// `PlannerServer` with its action server and its NavFn plugin.
#[derive(Debug, Clone)]
pub struct PlannerServer {
    params: PlannerServerParams,
    planner: NavfnPlanner,
    next_goal: GoalId,
    current: Option<GoalHandle>,
    pending: Option<GoalHandle>,
    preempt_requested: bool,
    /// `Some` once `computePlan` has started on `current`.
    run: Option<Run>,
    /// Results produced outside a pass, reported by the next one.
    backlog: VecDeque<ComputePathResult>,
}

impl PlannerServer {
    /// `planner_server` as `nav2_no_map_params.yaml` configures it, for a
    /// global costmap of this size.
    pub fn mower(costmap: &Costmap2D) -> Self {
        Self::new(PlannerServerParams::mower(), NavfnPlanner::new(NavfnParams::mower(), costmap))
    }

    pub fn new(params: PlannerServerParams, planner: NavfnPlanner) -> Self {
        Self {
            params,
            planner,
            next_goal: 1,
            current: None,
            pending: None,
            preempt_requested: false,
            run: None,
            backlog: VecDeque::new(),
        }
    }

    pub fn params(&self) -> &PlannerServerParams {
        &self.params
    }

    pub fn planner(&self) -> &NavfnPlanner {
        &self.planner
    }

    /// The executing goal.
    pub fn current_goal(&self) -> Option<GoalId> {
        self.current.as_ref().map(|g| g.id)
    }

    /// The goal waiting to preempt it.
    pub fn pending_goal(&self) -> Option<GoalId> {
        self.pending.as_ref().map(|g| g.id)
    }

    /// Goals accepted so far (every goal is accepted).
    pub fn goals_received(&self) -> u64 {
        self.next_goal - 1
    }

    /// A goal is executing (or a result waits to be reported): tick.
    pub fn has_work(&self) -> bool {
        self.current.is_some() || !self.backlog.is_empty()
    }

    /// `handle_goal` + `handle_accepted`: every goal is accepted. While one
    /// executes it goes to the pending slot (replacing, and aborting, a goal
    /// already there); otherwise it executes from the next pass.
    pub fn send_goal(&mut self, goal: ComputePathToPose) -> GoalId {
        let id = self.next_goal;
        self.next_goal += 1;
        let handle = GoalHandle { id, goal, canceling: false };
        if let Some(old) = self.pending.take() {
            self.backlog.push_back(terminate(old, error_code::NONE, None));
        }
        if self.current.is_some() {
            self.pending = Some(handle);
            self.preempt_requested = true;
        } else {
            self.preempt_requested = false;
            self.current = Some(handle);
            self.run = None;
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

    /// `SimpleActionServer::work()`: `computePlan` for the executing goal,
    /// then for the goals pending behind it.
    pub fn tick(&mut self, input: PlanInput<'_>) -> PlanOutput {
        let mut out = PlanOutput { results: self.backlog.drain(..).collect(), ..PlanOutput::default() };
        let PlanInput { now_ns, costmap_current, robot_pose, costmap } = input;
        while self.current.is_some() {
            if self.run.is_none() {
                // computePlan's entry: isCancelRequested
                if self.is_cancel_requested() {
                    // "Goal was canceled. Canceling planning action."
                    self.terminate_all(&mut out);
                } else {
                    let goal = self.current.as_ref().map(|g| g.goal.clone()).expect("a current goal");
                    self.run = Some(Run { goal, start_ns: now_ns, waiting_since_ns: None });
                }
            }
            if self.run.is_some() {
                if let Pass::Waiting = self.compute_plan(now_ns, costmap_current, robot_pose, costmap, &mut out) {
                    out.waiting_for_costmap = true;
                    return out;
                }
            }
            self.run = None;
            if let Some(cur) = self.current.take() {
                // "Current goal was not completed successfully."
                out.results.push(terminate(cur, error_code::NONE, None));
            }
            if self.pending.is_some() {
                self.accept_pending_goal(&mut out);
            } else {
                break;
            }
        }
        out
    }

    /// `computePlan` after its entry checks.
    fn compute_plan(
        &mut self,
        now: i64,
        costmap_current: bool,
        robot_pose: Option<Pose2D>,
        costmap: &mut Costmap2D,
        out: &mut PlanOutput,
    ) -> Pass {
        let run = self.run.as_mut().expect("compute_plan runs with a run");
        let start_ns = run.start_ns;
        // waitForCostmap: don't compute a plan until the costmap is valid
        // (after clear costmap)
        if !costmap_current {
            let since = *run.waiting_since_ns.get_or_insert(now);
            if now - since > duration_from_seconds(self.params.costmap_update_timeout) {
                self.fail(PlannerError::PlannerTimedOut, start_ns, now, out);
                return Pass::Returned;
            }
            return Pass::Waiting;
        }
        let goal = run.goal.clone();

        // getPreemptedGoalIfRequested
        if self.preempt_requested {
            self.accept_pending_goal(out);
        }

        let Some(start) = goal.start.or(robot_pose) else {
            self.fail(PlannerError::PlannerTFError, start_ns, now, out);
            return Pass::Returned;
        };
        // getPlan: one plugin, which an empty planner_id also selects
        if !goal.planner_id.is_empty() && goal.planner_id != self.params.planner_id {
            self.fail(PlannerError::InvalidPlanner, start_ns, now, out);
            return Pass::Returned;
        }
        let canceling = self.is_cancel_requested();
        match compute_path_to_pose(&mut self.planner, costmap, &start, &goal.goal, &mut || canceling) {
            Ok(path) => {
                out.plans.push(path.clone());
                let planning_time_ns = now - start_ns;
                let max = self.params.expected_planner_frequency;
                if max > 0.0 && planning_time_ns as f64 / 1e9 > 1.0 / max {
                    out.missed_rate = Some(planning_time_ns as f64 / 1e9);
                }
                if let Some(cur) = self.current.take() {
                    out.results.push(ComputePathResult {
                        goal: cur.id,
                        status: GoalStatus::Succeeded,
                        error_code: error_code::NONE,
                        error: None,
                        path,
                        planning_time_ns,
                    });
                }
            }
            Err(PlannerError::PlannerCancelled) => self.terminate_all(out),
            Err(e) => self.fail(e, start_ns, now, out),
        }
        Pass::Returned
    }

    /// A catch block: `terminate_current` with the exception's error code.
    fn fail(&mut self, e: PlannerError, start_ns: i64, now: i64, out: &mut PlanOutput) {
        if let Some(cur) = self.current.take() {
            let mut r = terminate(cur, error_code_of(&e), Some(e));
            r.planning_time_ns = now - start_ns;
            out.results.push(r);
        }
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
    fn accept_pending_goal(&mut self, out: &mut PlanOutput) {
        let Some(pending) = self.pending.take() else {
            return;
        };
        if let Some(cur) = self.current.take() {
            // "Cancelling the previous goal": abort, even if it is canceling
            out.results.push(ComputePathResult { status: GoalStatus::Aborted, ..terminate(cur, error_code::NONE, None) });
        }
        self.current = Some(pending);
        self.preempt_requested = false;
    }

    /// `terminate_all()`: both goals, with empty results.
    fn terminate_all(&mut self, out: &mut PlanOutput) {
        for handle in [self.current.take(), self.pending.take()].into_iter().flatten() {
            out.results.push(terminate(handle, error_code::NONE, None));
        }
        self.preempt_requested = false;
    }
}

/// `terminate(handle, result)`: canceled if the client asked, else aborted.
fn terminate(handle: GoalHandle, error_code: u16, error: Option<PlannerError>) -> ComputePathResult {
    let status = if handle.canceling { GoalStatus::Canceled } else { GoalStatus::Aborted };
    ComputePathResult { goal: handle.id, status, error_code, error, path: Path::new(), planning_time_ns: 0 }
}
