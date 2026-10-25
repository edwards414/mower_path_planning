//! `nav2_controller::ControllerServer`: the `follow_path` action's control
//! loop (`computeControl`), together with the goal bookkeeping of the
//! `nav2_util::SimpleActionServer` it runs in, as a state machine the caller
//! ticks.
//!
//! One [`ControllerServer::tick`] is one pass of `computeControl`'s `while`
//! loop, in upstream's order: the cancel request, the wait for a current
//! costmap, `updateGlobalPath` (a pending goal preempts the running one),
//! `isGoalReached`, then `computeAndPublishVelocity` (robot pose, progress
//! check, the controller, the `failure_tolerance` patience for
//! `NoValidControl`, the feedback). Every exception ends the goal with its
//! `FollowPath` error code after `publishZeroVelocity` (a zero command and a
//! `reset` of the controller); reaching the goal does the same with success.
//! The caller ticks at `controller_frequency` while a goal executes, and
//! right after [`ControllerServer::send_goal`] to start it the way upstream's
//! worker thread does: the first pass runs as soon as the goal is accepted.
//!
//! The action server's semantics are kept, quirks included:
//!
//! * a goal sent while one executes waits in the single pending slot (a goal
//!   already waiting there is aborted, with an empty result); the next pass
//!   accepts it, aborts the running goal with an **empty result** (error
//!   code 0, not a FollowPath error) and hands the new path to the
//!   controller. The goal checker is reset; the progress checker and the
//!   patience clock are not.
//! * `is_cancel_requested` looks at the pending goal when there is one, so
//!   cancelling the pending goal also ends the running one (aborted, error
//!   code 0), and cancelling the running goal while one is pending does
//!   nothing until the preemption, which aborts it.
//! * a cancelled goal ends `Canceled` with error code 0 once the controller's
//!   `cancel` agrees; a goal that fails while its cancel is pending ends
//!   `Canceled` too, with the failure's error code (`terminate` checks
//!   `is_canceling`).
//! * while the costmap is not current the loop is blocked: no cancel, no
//!   preemption, until it is current or `costmap_update_timeout` has passed
//!   (`ControllerTimedOut`).
//!
//! The ROS plumbing is the caller's: no lifecycle, no action server, no tf
//! (the robot pose and `odom <- map` come in [`CycleInput`], `None` when the
//! lookup fails), no odometry subscriber, no `speed_limit` subscription
//! ([`ControllerServer::set_speed_limit`]), no dynamic parameters, no clock
//! (every pass gets `now_ns`), and the commands are returned, not published
//! (upstream also skips publishing while `cmd_vel` has no subscriber). Goals
//! carry no `controller_id` / `goal_checker_id` / `progress_checker_id`: the
//! server has one controller and one goal checker, and the ids mower_nav sends
//! name exactly those. Results carry no `error_msg`; 1.3.13 never fills it.
//!
//! One deliberate change: the feedback's `distance_to_goal` is the path length
//! from the plan pose closest to the robot, and upstream compares the robot
//! pose in the costmap frame (`odom`) with plan poses in `map`. Here the robot
//! pose is taken into `map` first, so the distance does not drift with the
//! `map -> odom` offset. mower_nav turns this distance into coverage progress.
//!
//! Upstream: `nav2_controller/src/controller_server.cpp`,
//! `include/nav2_controller/controller_server.hpp`,
//! `nav2_util/simple_action_server.hpp`, `nav2_msgs/action/FollowPath.action`.

use std::collections::VecDeque;

use crate::checkers::{duration_from_seconds, SimpleGoalChecker, SimpleGoalCheckerParams, SimpleProgressChecker, SimpleProgressCheckerParams};
use crate::costmap::Costmap2D;
use crate::geometry::{euclidean_distance, Pose2D, Transform2D, Twist2D};
use crate::path_handler::{calculate_path_length, Path};
use crate::rpp::{ControlInput, Controller, ControllerError, RegulatedPurePursuit};
use crate::shim::RotationShim;
use crate::smoother::TwistStamped;

/// `FollowPath.action`'s result error codes.
pub mod error_code {
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 100;
    pub const INVALID_CONTROLLER: u16 = 101;
    pub const TF_ERROR: u16 = 102;
    pub const INVALID_PATH: u16 = 103;
    pub const PATIENCE_EXCEEDED: u16 = 104;
    pub const FAILED_TO_MAKE_PROGRESS: u16 = 105;
    pub const NO_VALID_CONTROL: u16 = 106;
    pub const CONTROLLER_TIMED_OUT: u16 = 107;
}

/// The error code `computeControl`'s catch blocks give each exception.
pub fn error_code_of(e: &ControllerError) -> u16 {
    match e {
        ControllerError::InvalidController(_) => error_code::INVALID_CONTROLLER,
        ControllerError::ControllerTFError(_) => error_code::TF_ERROR,
        ControllerError::NoValidControl(_) => error_code::NO_VALID_CONTROL,
        ControllerError::FailedToMakeProgress(_) => error_code::FAILED_TO_MAKE_PROGRESS,
        ControllerError::PatienceExceeded(_) => error_code::PATIENCE_EXCEEDED,
        ControllerError::InvalidPath(_) => error_code::INVALID_PATH,
        ControllerError::ControllerTimedOut(_) => error_code::CONTROLLER_TIMED_OUT,
        ControllerError::Controller(_) => error_code::UNKNOWN,
    }
}

/// The controller server's own parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct ControllerServerParams {
    /// Hz; the caller's tick rate (upstream's `WallRate`).
    pub controller_frequency: f64,
    pub min_x_velocity_threshold: f64,
    pub min_y_velocity_threshold: f64,
    pub min_theta_velocity_threshold: f64,
    /// seconds of `NoValidControl` before `PatienceExceeded`; 0 fails at
    /// once, -1 never.
    pub failure_tolerance: f64,
    pub publish_zero_velocity: bool,
    /// seconds
    pub costmap_update_timeout: f64,
    /// `costmap_ros_->getBaseFrameID()`: the commands' `frame_id`.
    pub robot_base_frame: String,
}

impl Default for ControllerServerParams {
    /// Upstream's declared defaults.
    fn default() -> Self {
        Self {
            controller_frequency: 20.0,
            min_x_velocity_threshold: 0.0001,
            min_y_velocity_threshold: 0.0001,
            min_theta_velocity_threshold: 0.0001,
            failure_tolerance: 0.0,
            publish_zero_velocity: true,
            costmap_update_timeout: 0.30,
            robot_base_frame: "base_link".into(),
        }
    }
}

impl ControllerServerParams {
    /// `controller_server` in `nav2_no_map_params.yaml`.
    pub fn mower() -> Self {
        Self {
            controller_frequency: 20.0,
            min_x_velocity_threshold: 0.001,
            min_y_velocity_threshold: 0.5,
            min_theta_velocity_threshold: 0.001,
            failure_tolerance: 2.0,
            costmap_update_timeout: 1.0,
            robot_base_frame: "base_footprint".into(),
            ..Self::default()
        }
    }
}

pub use crate::action::{GoalId, GoalStatus};

/// A goal's result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FollowPathResult {
    pub goal: GoalId,
    pub status: GoalStatus,
    /// [`error_code`]: `NONE` for success, a cancel and the empty results
    /// (preempted, replaced in the pending slot, ended with a cancelled
    /// pending goal).
    pub error_code: u16,
    /// The exception that ended it, for the log (`RCLCPP_ERROR(e.what())`).
    pub error: Option<ControllerError>,
}

/// `FollowPath.action`'s feedback (both `float32`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Feedback {
    pub distance_to_goal: f32,
    pub speed: f32,
}

/// What one pass reads from the robot, tf and the costmap.
#[derive(Debug, Clone, Copy)]
pub struct CycleInput<'a> {
    pub now_ns: i64,
    /// `costmap_ros_->isCurrent()`.
    pub costmap_current: bool,
    /// `costmap_ros_->getRobotPose()`: the base in the costmap frame
    /// (`odom`), `None` if the lookup failed.
    pub robot_pose: Option<Pose2D>,
    /// `odom_sub_->getTwist()`: the latest odometry twist, unthresholded.
    pub odom_twist: Twist2D,
    /// `odom <- map` (the plan's frame), `None` if the lookup failed.
    pub odom_from_map: Option<Transform2D>,
    /// `costmap_ros_->getCostmap()`: the local costmap.
    pub costmap: &'a Costmap2D,
}

/// What one pass produced, in the order upstream did it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CycleOutput {
    /// Published on `cmd_vel`: the command, a held zero while
    /// `NoValidControl` is tolerated, the zero of `publishZeroVelocity`.
    pub commands: Vec<TwistStamped>,
    pub feedback: Option<(GoalId, Feedback)>,
    /// Goals that ended, oldest first, including a pending goal replaced
    /// by [`ControllerServer::send_goal`] since the last pass.
    pub results: Vec<FollowPathResult>,
    /// `NoValidControl` tolerated this pass (`RCLCPP_WARN`).
    pub warning: Option<ControllerError>,
    /// The pass is blocked on the costmap: tick again (upstream polls at
    /// 100 Hz) until it is current or the timeout ends the goal.
    pub waiting_for_costmap: bool,
}

#[derive(Debug, Clone)]
struct GoalHandle {
    id: GoalId,
    path: Path,
    canceling: bool,
}

/// `computeControl`'s state while it runs.
#[derive(Debug, Clone)]
struct Run {
    current_path: Path,
    end_pose: Pose2D,
    last_valid_cmd_ns: i64,
    /// Inside the costmap wait since then.
    waiting_since_ns: Option<i64>,
}

enum Pass {
    /// `loop_rate.sleep()`, then the next pass.
    Continue,
    /// `computeControl` returned.
    Returned,
}

/// `ControllerServer` with its action server.
#[derive(Debug, Clone)]
pub struct ControllerServer<C: Controller> {
    params: ControllerServerParams,
    controller: C,
    goal_checker: SimpleGoalChecker,
    /// `None`: no progress checker configured, checking bypassed.
    progress_checker: Option<SimpleProgressChecker>,
    next_goal: GoalId,
    current: Option<GoalHandle>,
    pending: Option<GoalHandle>,
    preempt_requested: bool,
    /// `Some` once `computeControl` has started on `current`.
    run: Option<Run>,
    /// Results produced outside a pass, reported by the next one.
    backlog: VecDeque<FollowPathResult>,
    /// `computeControl` entries so far.
    loops_started: u64,
}

impl ControllerServer<RotationShim<RegulatedPurePursuit>> {
    /// `controller_server` as `nav2_no_map_params.yaml` configures it.
    pub fn mower() -> Self {
        Self::new(
            ControllerServerParams::mower(),
            RotationShim::mower(),
            SimpleGoalChecker::new(SimpleGoalCheckerParams::mower()),
            Some(SimpleProgressChecker::new(SimpleProgressCheckerParams::mower())),
        )
    }
}

impl<C: Controller> ControllerServer<C> {
    pub fn new(
        params: ControllerServerParams,
        controller: C,
        goal_checker: SimpleGoalChecker,
        progress_checker: Option<SimpleProgressChecker>,
    ) -> Self {
        Self {
            params,
            controller,
            goal_checker,
            progress_checker,
            next_goal: 1,
            current: None,
            pending: None,
            preempt_requested: false,
            run: None,
            backlog: VecDeque::new(),
            loops_started: 0,
        }
    }

    pub fn params(&self) -> &ControllerServerParams {
        &self.params
    }

    pub fn controller(&self) -> &C {
        &self.controller
    }

    pub fn controller_mut(&mut self) -> &mut C {
        &mut self.controller
    }

    pub fn goal_checker(&self) -> &SimpleGoalChecker {
        &self.goal_checker
    }

    pub fn progress_checker(&self) -> Option<&SimpleProgressChecker> {
        self.progress_checker.as_ref()
    }

    /// The executing goal.
    pub fn current_goal(&self) -> Option<GoalId> {
        self.current.as_ref().map(|g| g.id)
    }

    /// The goal waiting to preempt it.
    pub fn pending_goal(&self) -> Option<GoalId> {
        self.pending.as_ref().map(|g| g.id)
    }

    /// A goal is executing: tick at `controller_frequency`.
    pub fn is_executing(&self) -> bool {
        self.current.is_some()
    }

    /// Goals accepted so far (every goal is accepted).
    pub fn goals_received(&self) -> u64 {
        self.next_goal - 1
    }

    /// How many times `computeControl` has been entered. Upstream's loop
    /// rate (a `WallRate` built on entry) starts over when this goes up; a
    /// preemption stays in the same loop.
    pub fn loops_started(&self) -> u64 {
        self.loops_started
    }

    /// `handle_goal` + `handle_accepted`: every goal is accepted. While one
    /// executes it goes to the pending slot (replacing, and aborting, a goal
    /// already there); otherwise it executes from the next pass.
    pub fn send_goal(&mut self, path: Path) -> GoalId {
        let id = self.next_goal;
        self.next_goal += 1;
        let handle = GoalHandle { id, path, canceling: false };
        if self.current.is_some() {
            if let Some(old) = self.pending.take() {
                let r = terminate(old, error_code::NONE, None);
                self.backlog.push_back(r);
            }
            self.pending = Some(handle);
            self.preempt_requested = true;
        } else {
            // "Forgot to handle a preemption": unreachable, kept for the shape
            if let Some(old) = self.pending.take() {
                let r = terminate(old, error_code::NONE, None);
                self.backlog.push_back(r);
                self.preempt_requested = false;
            }
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

    /// `speedLimitCallback`.
    pub fn set_speed_limit(&mut self, speed_limit: f64, percentage: bool) {
        self.controller.set_speed_limit(speed_limit, percentage);
    }

    /// One pass of the control loop (or the start of a goal and its first
    /// pass, see the module doc). Does nothing while no goal executes.
    pub fn tick(&mut self, input: &CycleInput) -> CycleOutput {
        let mut out = CycleOutput { results: self.backlog.drain(..).collect(), ..CycleOutput::default() };
        // SimpleActionServer::work()
        while self.current.is_some() {
            if self.run.is_none() {
                if let Err(e) = self.start(input) {
                    self.fail(e, input, &mut out);
                }
            }
            if self.run.is_some() {
                match self.pass(input, &mut out) {
                    Pass::Continue => return out,
                    Pass::Returned => {}
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

    // --- computeControl ------------------------------------------------------

    /// Everything before the `while` loop.
    fn start(&mut self, input: &CycleInput) -> Result<(), ControllerError> {
        self.loops_started += 1;
        let path = self.current.as_ref().map(|g| g.path.clone()).unwrap_or_default();
        self.run = Some(Run { current_path: Vec::new(), end_pose: Pose2D::default(), last_valid_cmd_ns: input.now_ns, waiting_since_ns: None });
        if let Err(e) = self.set_planner_path(&path) {
            self.run = None;
            return Err(e);
        }
        if let Some(pc) = &mut self.progress_checker {
            pc.reset();
        }
        if let Some(run) = &mut self.run {
            run.last_valid_cmd_ns = input.now_ns;
        }
        Ok(())
    }

    /// One iteration of the `while` loop.
    fn pass(&mut self, input: &CycleInput, out: &mut CycleOutput) -> Pass {
        let now = input.now_ns;
        let waiting = self.run.as_ref().is_some_and(|r| r.waiting_since_ns.is_some());
        // a cancel the controller does not agree to yet ("Waiting for the
        // controller to finish cancellation") keeps the loop going
        if !waiting && self.is_cancel_requested() && self.controller.cancel() {
            // "Cancellation was successful. Stopping the robot."
            self.terminate_all(out);
            self.publish_zero_velocity(now, out);
            return Pass::Returned;
        }

        // Don't compute a trajectory until costmap is valid
        if !input.costmap_current {
            let run = self.run.as_mut().expect("pass runs with a run");
            let since = *run.waiting_since_ns.get_or_insert(now);
            if now - since > duration_from_seconds(self.params.costmap_update_timeout) {
                self.fail(ControllerError::ControllerTimedOut("Costmap timed out waiting for update"), input, out);
                return Pass::Returned;
            }
            out.waiting_for_costmap = true;
            return Pass::Continue;
        }
        if let Some(run) = &mut self.run {
            run.waiting_since_ns = None;
        }

        if let Err(e) = self.update_global_path(out) {
            self.fail(e, input, out);
            return Pass::Returned;
        }

        match self.is_goal_reached(input) {
            Err(e) => {
                self.fail(e, input, out);
                return Pass::Returned;
            }
            Ok(true) => {
                // "Reached the goal!"
                self.publish_zero_velocity(now, out);
                if let Some(cur) = self.current.take() {
                    out.results.push(FollowPathResult { goal: cur.id, status: GoalStatus::Succeeded, error_code: error_code::NONE, error: None });
                }
                return Pass::Returned;
            }
            Ok(false) => {}
        }

        if let Err(e) = self.compute_and_publish_velocity(input, out) {
            self.fail(e, input, out);
            return Pass::Returned;
        }
        Pass::Continue
    }

    /// A catch block: `publishZeroVelocity`, then `terminate_current` with
    /// the exception's error code.
    fn fail(&mut self, e: ControllerError, input: &CycleInput, out: &mut CycleOutput) {
        self.publish_zero_velocity(input.now_ns, out);
        if let Some(cur) = self.current.take() {
            out.results.push(terminate(cur, error_code_of(&e), Some(e)));
        }
    }

    fn set_planner_path(&mut self, path: &[Pose2D]) -> Result<(), ControllerError> {
        let Some(end_pose) = path.last().copied() else {
            return Err(ControllerError::InvalidPath("Path is empty."));
        };
        self.controller.set_plan(path);
        self.goal_checker.reset();
        if let Some(run) = &mut self.run {
            run.end_pose = end_pose;
            run.current_path = path.to_vec();
        }
        Ok(())
    }

    fn compute_and_publish_velocity(&mut self, input: &CycleInput, out: &mut CycleOutput) -> Result<(), ControllerError> {
        let now = input.now_ns;
        let Some(pose) = input.robot_pose else {
            return Err(ControllerError::ControllerTFError("Failed to obtain robot pose"));
        };
        if let Some(pc) = &mut self.progress_checker {
            if !pc.check(&pose, now) {
                return Err(ControllerError::FailedToMakeProgress("Failed to make progress"));
            }
        }
        let twist = self.thresholded_twist(&input.odom_twist);
        // isGoalReached already needed the transform when there is a pose;
        // without it RPP's own plan transform fails the same way
        let Some(odom_from_map) = input.odom_from_map else {
            return Err(ControllerError::ControllerTFError("Unable to transform plan pose into local frame"));
        };
        let control = ControlInput {
            pose,
            speed: twist,
            xy_goal_tolerance: self.goal_checker.get_tolerances().position[0],
            costmap_from_plan: odom_from_map,
            costmap: input.costmap,
        };
        let cmd = match self.controller.compute_velocity_commands(&control) {
            Ok(cmd) => {
                if let Some(run) = &mut self.run {
                    run.last_valid_cmd_ns = now;
                }
                cmd
            }
            // Only no valid control exception types are valid to attempt to
            // have control patience
            Err(ControllerError::NoValidControl(m)) if self.params.failure_tolerance > 0.0 || self.params.failure_tolerance == -1.0 => {
                out.warning = Some(ControllerError::NoValidControl(m));
                let last = self.run.as_ref().map_or(now, |r| r.last_valid_cmd_ns);
                if ns_to_secs(now - last) > self.params.failure_tolerance && self.params.failure_tolerance != -1.0 {
                    return Err(ControllerError::PatienceExceeded("Controller patience exceeded"));
                }
                Twist2D::default()
            }
            Err(e) => return Err(e),
        };

        let goal = self.current.as_ref().map_or(0, |g| g.id);
        let path = self.run.as_ref().map(|r| r.current_path.as_slice()).unwrap_or(&[]);
        // the closest plan pose, with the robot in the plan's frame (see the
        // module doc; upstream compares the odom pose with map poses)
        let robot = odom_from_map.inverse().apply_pose(&pose);
        let mut closest = 0;
        let mut min_dist = f64::MAX;
        for (i, p) in path.iter().enumerate() {
            let d = euclidean_distance(robot.x, robot.y, p.x, p.y);
            if d < min_dist {
                min_dist = d;
                closest = i;
            }
        }
        let feedback = Feedback {
            distance_to_goal: calculate_path_length(&path[closest.min(path.len())..]) as f32,
            speed: cmd.x.hypot(cmd.y) as f32,
        };
        out.feedback = Some((goal, feedback));
        out.commands.push(self.stamped(cmd, now));
        Ok(())
    }

    /// `updateGlobalPath`: a pending goal preempts the running one.
    fn update_global_path(&mut self, out: &mut CycleOutput) -> Result<(), ControllerError> {
        if !self.preempt_requested {
            return Ok(());
        }
        // "Passing new path to controller."
        self.accept_pending_goal(out);
        let path = self.current.as_ref().map(|g| g.path.clone()).unwrap_or_default();
        self.set_planner_path(&path)
    }

    fn is_goal_reached(&mut self, input: &CycleInput) -> Result<bool, ControllerError> {
        let Some(pose) = input.robot_pose else {
            return Ok(false);
        };
        let velocity = self.thresholded_twist(&input.odom_twist);
        let Some(odom_from_map) = input.odom_from_map else {
            return Err(ControllerError::ControllerTFError("Failed to transform end pose to global frame"));
        };
        let end_pose = self.run.as_ref().map(|r| r.end_pose).unwrap_or_default();
        let goal = odom_from_map.apply_pose(&end_pose);
        Ok(self.goal_checker.is_goal_reached(&pose, &goal, &velocity))
    }

    /// `publishZeroVelocity`: the zero (unless `publish_zero_velocity` is
    /// off), then every controller's `reset`.
    fn publish_zero_velocity(&mut self, now: i64, out: &mut CycleOutput) {
        if self.params.publish_zero_velocity {
            out.commands.push(self.stamped(Twist2D::default(), now));
        }
        self.controller.reset();
    }

    fn stamped(&self, twist: Twist2D, now: i64) -> TwistStamped {
        TwistStamped { stamp_ns: now, frame_id: self.params.robot_base_frame.clone(), twist, other: [0.0; 3] }
    }

    fn thresholded_twist(&self, twist: &Twist2D) -> Twist2D {
        let p = &self.params;
        Twist2D::new(
            thresholded(twist.x, p.min_x_velocity_threshold),
            thresholded(twist.y, p.min_y_velocity_threshold),
            thresholded(twist.theta, p.min_theta_velocity_threshold),
        )
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
    fn accept_pending_goal(&mut self, out: &mut CycleOutput) {
        let Some(pending) = self.pending.take() else {
            return;
        };
        if let Some(cur) = self.current.take() {
            // "Cancelling the previous goal": abort, even if it is canceling
            out.results.push(FollowPathResult { goal: cur.id, status: GoalStatus::Aborted, error_code: error_code::NONE, error: None });
        }
        self.current = Some(pending);
        self.preempt_requested = false;
    }

    /// `terminate_all()`: both goals, with empty results.
    fn terminate_all(&mut self, out: &mut CycleOutput) {
        for handle in [self.current.take(), self.pending.take()].into_iter().flatten() {
            out.results.push(terminate(handle, error_code::NONE, None));
        }
        self.preempt_requested = false;
    }
}

/// `terminate(handle, result)`: canceled if the client asked, else aborted.
fn terminate(handle: GoalHandle, error_code: u16, error: Option<ControllerError>) -> FollowPathResult {
    let status = if handle.canceling { GoalStatus::Canceled } else { GoalStatus::Aborted };
    FollowPathResult { goal: handle.id, status, error_code, error }
}

/// `getThresholdedVelocity`.
fn thresholded(velocity: f64, threshold: f64) -> f64 {
    if velocity.abs() > threshold { velocity } else { 0.0 }
}

/// `rclcpp::Duration::seconds()`.
fn ns_to_secs(ns: i64) -> f64 {
    ns as f64 / 1e9
}
