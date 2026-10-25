//! `controller_server`: the loop and the action-server bookkeeping against a
//! scripted controller (every branch of `computeControl`, `updateGlobalPath`,
//! `computeAndPublishVelocity` and the SimpleActionServer quirks), then the
//! mower's RotationShim + RPP in a closed kinematic loop.

use std::collections::VecDeque;

use mower_nav_core::checkers::{SimpleGoalChecker, SimpleGoalCheckerParams, SimpleProgressChecker, SimpleProgressCheckerParams};
use mower_nav_core::controller_server::{
    error_code, ControllerServer, ControllerServerParams, CycleInput, CycleOutput, FollowPathResult, GoalId, GoalStatus,
};
use mower_nav_core::costmap::{static_costmap, Costmap2D, OccupancyGrid, RollingWindow, StaticLayerParams};
use mower_nav_core::geometry::{Pose2D, Transform2D, Twist2D};
use mower_nav_core::rpp::{ControlInput, Controller, ControllerError};

const T0: i64 = 1_792_900_000_000_000_000;
const DT: i64 = 50_000_000;

#[derive(Debug, Default)]
struct Scripted {
    plans: Vec<Vec<Pose2D>>,
    replies: VecDeque<Result<Twist2D, ControllerError>>,
    /// once `replies` is empty
    otherwise: Option<Result<Twist2D, ControllerError>>,
    inputs: Vec<(Pose2D, Twist2D, f64, Transform2D)>,
    resets: usize,
    cancel_replies: VecDeque<bool>,
    cancels: usize,
    speed_limits: Vec<(f64, bool)>,
}

impl Controller for Scripted {
    fn set_plan(&mut self, path: &[Pose2D]) {
        self.plans.push(path.to_vec());
    }
    fn compute_velocity_commands(&mut self, input: &ControlInput) -> Result<Twist2D, ControllerError> {
        self.inputs.push((input.pose, input.speed, input.xy_goal_tolerance, input.costmap_from_plan));
        self.replies.pop_front().or(self.otherwise).unwrap_or(Ok(Twist2D::new(0.2, 0.0, 0.1)))
    }
    fn set_speed_limit(&mut self, speed_limit: f64, percentage: bool) {
        self.speed_limits.push((speed_limit, percentage));
    }
    fn reset(&mut self) {
        self.resets += 1;
    }
    fn cancel(&mut self) -> bool {
        self.cancels += 1;
        self.cancel_replies.pop_front().unwrap_or(true)
    }
}

fn server_with(params: ControllerServerParams, controller: Scripted) -> ControllerServer<Scripted> {
    ControllerServer::new(
        params,
        controller,
        SimpleGoalChecker::new(SimpleGoalCheckerParams::mower()),
        Some(SimpleProgressChecker::new(SimpleProgressCheckerParams::mower())),
    )
}

fn server(controller: Scripted) -> ControllerServer<Scripted> {
    server_with(ControllerServerParams::mower(), controller)
}

fn line(n: usize, step: f64) -> Vec<Pose2D> {
    (0..n).map(|i| Pose2D::new(i as f64 * step, 0.0, 0.0)).collect()
}

fn grid() -> Costmap2D {
    Costmap2D::new(60, 60, 0.05, -1.5, -1.5, 0)
}

fn at<'a>(costmap: &'a Costmap2D, k: i64, pose: Pose2D) -> CycleInput<'a> {
    CycleInput {
        now_ns: T0 + k * DT,
        costmap_current: true,
        robot_pose: Some(pose),
        odom_twist: Twist2D::default(),
        odom_from_map: Some(Transform2D::IDENTITY),
        costmap,
    }
}

fn result(goal: GoalId, status: GoalStatus, error_code: u16) -> (GoalId, GoalStatus, u16) {
    (goal, status, error_code)
}

fn results(out: &CycleOutput) -> Vec<(GoalId, GoalStatus, u16)> {
    out.results.iter().map(|r: &FollowPathResult| (r.goal, r.status, r.error_code)).collect()
}

fn is_zero(out: &CycleOutput, i: usize) -> bool {
    let c = &out.commands[i];
    c.twist == Twist2D::default() && c.frame_id == "base_footprint"
}

/// Ticks from `k` until a result comes, robot at `pose`; returns the tick.
fn run_until_result(s: &mut ControllerServer<Scripted>, costmap: &Costmap2D, from: i64, pose: Pose2D) -> (i64, CycleOutput) {
    for k in from..from + 2000 {
        let out = s.tick(&at(costmap, k, pose));
        if !out.results.is_empty() {
            return (k, out);
        }
    }
    panic!("no result");
}

#[test]
fn mower_parameters() {
    let p = ControllerServerParams::mower();
    assert_eq!((p.controller_frequency, p.failure_tolerance, p.costmap_update_timeout), (20.0, 2.0, 1.0));
    assert_eq!((p.min_x_velocity_threshold, p.min_y_velocity_threshold, p.min_theta_velocity_threshold), (0.001, 0.5, 0.001));
    assert!(p.publish_zero_velocity);
    assert_eq!(p.robot_base_frame, "base_footprint");
    let d = ControllerServerParams::default();
    assert_eq!((d.failure_tolerance, d.costmap_update_timeout, d.min_y_velocity_threshold), (0.0, 0.30, 0.0001));
}

#[test]
fn a_command_each_pass_with_feedback() {
    let c = grid();
    let mut s = server(Scripted::default());
    assert!(s.tick(&at(&c, 0, Pose2D::default())).commands.is_empty(), "idle: nothing");
    let g = s.send_goal(line(11, 0.1));
    assert_eq!(s.current_goal(), Some(g));
    let mut input = at(&c, 0, Pose2D::new(0.32, 0.0, 0.0));
    input.odom_twist = Twist2D::new(0.2, 0.3, 0.0005);
    let out = s.tick(&input);
    assert!(out.results.is_empty() && !out.waiting_for_costmap && out.warning.is_none());
    assert_eq!(out.commands.len(), 1);
    assert_eq!(out.commands[0].twist, Twist2D::new(0.2, 0.0, 0.1));
    assert_eq!((out.commands[0].stamp_ns, out.commands[0].frame_id.as_str()), (T0, "base_footprint"));
    let (goal, fb) = out.feedback.unwrap();
    assert_eq!(goal, g);
    assert_eq!(fb.speed, 0.2f32);
    // closest pose is index 3 (0.3 m): 0.7 m left
    assert!((fb.distance_to_goal as f64 - 0.7).abs() < 1e-6);
    let ctl = s.controller();
    assert_eq!(ctl.plans, vec![line(11, 0.1)]);
    // the controller gets the thresholded twist (min_y 0.5, min_theta 0.001)
    // and the goal checker's xy tolerance
    assert_eq!(ctl.inputs[0].1, Twist2D::new(0.2, 0.0, 0.0));
    assert_eq!(ctl.inputs[0].2, 0.05);
    assert_eq!(ctl.resets, 0);
}

#[test]
fn reaching_the_goal_succeeds_with_a_zero() {
    let c = grid();
    let mut s = server(Scripted::default());
    let g = s.send_goal(line(5, 0.1));
    let out = s.tick(&at(&c, 0, Pose2D::new(0.38, 0.02, 2.0)));
    assert_eq!(results(&out), vec![result(g, GoalStatus::Succeeded, error_code::NONE)]);
    assert_eq!(out.commands.len(), 1);
    assert!(is_zero(&out, 0));
    assert!(out.feedback.is_none(), "no compute on the pass that reaches the goal");
    assert!(s.controller().inputs.is_empty());
    assert_eq!(s.controller().resets, 1);
    assert!(!s.is_executing());
    // the end pose is taken into odom: map -> odom moves it
    let mut s = server(Scripted::default());
    s.send_goal(line(5, 0.1));
    let mut input = at(&c, 0, Pose2D::new(0.4, 0.0, 0.0));
    input.odom_from_map = Some(Transform2D::new(1.0, 0.0, 0.0));
    assert!(s.tick(&input).results.is_empty());
}

#[test]
fn an_empty_path_is_invalid() {
    let c = grid();
    let mut s = server(Scripted::default());
    let g = s.send_goal(Vec::new());
    let out = s.tick(&at(&c, 0, Pose2D::default()));
    assert_eq!(results(&out), vec![result(g, GoalStatus::Aborted, error_code::INVALID_PATH)]);
    assert_eq!(out.results[0].error.unwrap().what(), "Path is empty.");
    assert!(is_zero(&out, 0));
    assert_eq!(s.controller().resets, 1);
    assert!(s.controller().plans.is_empty());
}

#[test]
fn no_valid_control_is_held_at_zero_for_two_seconds() {
    let c = grid();
    let err = ControllerError::NoValidControl("RegulatedPurePursuitController detected collision ahead!");
    let mut s = server(Scripted { otherwise: Some(Err(err)), ..Scripted::default() });
    let g = s.send_goal(line(11, 0.1));
    for k in 0..=40 {
        let out = s.tick(&at(&c, k, Pose2D::default()));
        assert!(out.results.is_empty(), "tick {k}");
        assert_eq!(out.warning, Some(err));
        assert!(is_zero(&out, 0) && out.commands[0].stamp_ns == T0 + k * DT);
        assert_eq!(out.feedback.unwrap().1.speed, 0.0);
    }
    // 2.00 s is not more than failure_tolerance, 2.05 s is
    let out = s.tick(&at(&c, 41, Pose2D::default()));
    assert_eq!(results(&out), vec![result(g, GoalStatus::Aborted, error_code::PATIENCE_EXCEEDED)]);
    assert_eq!(out.results[0].error.unwrap().what(), "Controller patience exceeded");
    assert_eq!(out.commands.len(), 1, "only publishZeroVelocity's zero");
    assert_eq!(s.controller().resets, 1);
}

#[test]
fn a_valid_command_restarts_the_patience() {
    let c = grid();
    let err = ControllerError::NoValidControl("x");
    let mut replies: VecDeque<_> = (0..30).map(|_| Err(err)).collect();
    replies.push_back(Ok(Twist2D::new(0.1, 0.0, 0.0)));
    let mut s = server(Scripted { replies, otherwise: Some(Err(err)), ..Scripted::default() });
    let g = s.send_goal(line(11, 0.1));
    // tick 30 is valid, so the clock restarts there: aborted at 30 + 41
    let (k, out) = run_until_result(&mut s, &c, 0, Pose2D::default());
    assert_eq!((k, results(&out)), (71, vec![result(g, GoalStatus::Aborted, error_code::PATIENCE_EXCEEDED)]));
}

#[test]
fn failure_tolerance_zero_and_minus_one() {
    let c = grid();
    let err = ControllerError::NoValidControl("x");
    let params = ControllerServerParams { failure_tolerance: 0.0, ..ControllerServerParams::mower() };
    let mut s = server_with(params, Scripted { otherwise: Some(Err(err)), ..Scripted::default() });
    let g = s.send_goal(line(11, 0.1));
    let out = s.tick(&at(&c, 0, Pose2D::default()));
    assert_eq!(results(&out), vec![result(g, GoalStatus::Aborted, error_code::NO_VALID_CONTROL)]);
    assert_eq!(out.results[0].error, Some(err));

    let params = ControllerServerParams { failure_tolerance: -1.0, ..ControllerServerParams::mower() };
    let mut s = server_with(params, Scripted { otherwise: Some(Err(err)), ..Scripted::default() });
    s.send_goal(line(11, 0.1));
    // never patience; the progress checker ends it after 20 s instead
    let (k, out) = run_until_result(&mut s, &c, 0, Pose2D::default());
    assert_eq!((k, out.results[0].error_code), (401, error_code::FAILED_TO_MAKE_PROGRESS));
}

#[test]
fn other_controller_errors_end_the_goal_with_their_code() {
    let c = grid();
    for (e, code) in [
        (ControllerError::InvalidPath("Resulting plan has 0 poses in it."), error_code::INVALID_PATH),
        (ControllerError::ControllerTFError("Unable to transform plan pose into local frame"), error_code::TF_ERROR),
        (ControllerError::Controller("Path is out of costmap"), error_code::UNKNOWN),
        (ControllerError::InvalidController("x"), error_code::INVALID_CONTROLLER),
        (ControllerError::ControllerTimedOut("x"), error_code::CONTROLLER_TIMED_OUT),
    ] {
        let mut s = server(Scripted { otherwise: Some(Err(e)), ..Scripted::default() });
        let g = s.send_goal(line(11, 0.1));
        let out = s.tick(&at(&c, 0, Pose2D::default()));
        assert_eq!(results(&out), vec![result(g, GoalStatus::Aborted, code)], "{e}");
        assert_eq!(out.results[0].error, Some(e));
        assert!(out.feedback.is_none() && out.commands.len() == 1 && is_zero(&out, 0));
    }
}

#[test]
fn no_progress_for_twenty_seconds() {
    let c = grid();
    let mut s = server(Scripted::default());
    let g = s.send_goal(line(11, 0.1));
    // baseline at the first pass; 20.00 s is allowed, 20.05 s is not
    let (k, out) = run_until_result(&mut s, &c, 0, Pose2D::default());
    assert_eq!((k, results(&out)), (401, vec![result(g, GoalStatus::Aborted, error_code::FAILED_TO_MAKE_PROGRESS)]));
    // moving more than 0.1 m re-baselines
    let mut s = server(Scripted::default());
    s.send_goal(line(31, 0.1));
    for k in 0..300 {
        assert!(s.tick(&at(&c, k, Pose2D::default())).results.is_empty());
    }
    assert!(s.tick(&at(&c, 300, Pose2D::new(0.11, 0.0, 0.0))).results.is_empty());
    let (k, _) = run_until_result(&mut s, &c, 301, Pose2D::new(0.11, 0.0, 0.0));
    assert_eq!(k, 701);
}

#[test]
fn tf_failures() {
    let c = grid();
    // no robot pose: isGoalReached says no, computeAndPublishVelocity throws
    let mut s = server(Scripted::default());
    let g = s.send_goal(line(11, 0.1));
    let mut input = at(&c, 0, Pose2D::default());
    input.robot_pose = None;
    let out = s.tick(&input);
    assert_eq!(results(&out), vec![result(g, GoalStatus::Aborted, error_code::TF_ERROR)]);
    assert_eq!(out.results[0].error.unwrap().what(), "Failed to obtain robot pose");
    // no odom <- map: the end pose cannot be transformed
    let mut s = server(Scripted::default());
    let g = s.send_goal(line(11, 0.1));
    let mut input = at(&c, 0, Pose2D::default());
    input.odom_from_map = None;
    let out = s.tick(&input);
    assert_eq!(results(&out), vec![result(g, GoalStatus::Aborted, error_code::TF_ERROR)]);
    assert_eq!(out.results[0].error.unwrap().what(), "Failed to transform end pose to global frame");
}

#[test]
fn cancel() {
    let c = grid();
    let mut s = server(Scripted::default());
    let g = s.send_goal(line(11, 0.1));
    s.tick(&at(&c, 0, Pose2D::default()));
    assert!(s.cancel_goal(g));
    assert!(!s.cancel_goal(g + 1), "unknown goal");
    let out = s.tick(&at(&c, 1, Pose2D::default()));
    assert_eq!(results(&out), vec![result(g, GoalStatus::Canceled, error_code::NONE)]);
    assert!(out.commands.len() == 1 && is_zero(&out, 0) && out.feedback.is_none());
    assert_eq!((s.controller().cancels, s.controller().resets), (1, 1));
    assert!(!s.cancel_goal(g), "a finished goal cannot be cancelled");

    // the controller may ask to keep going (RPP's cancel deceleration)
    let mut s = server(Scripted { cancel_replies: VecDeque::from([false, false]), ..Scripted::default() });
    let g = s.send_goal(line(11, 0.1));
    s.cancel_goal(g);
    for k in 0..2 {
        let out = s.tick(&at(&c, k, Pose2D::default()));
        assert!(out.results.is_empty() && out.commands.len() == 1 && !is_zero(&out, 0));
    }
    let out = s.tick(&at(&c, 2, Pose2D::default()));
    assert_eq!(results(&out), vec![result(g, GoalStatus::Canceled, error_code::NONE)]);

    // failing while the cancel is pending: canceled, with the failure's code
    let mut s = server(Scripted {
        cancel_replies: VecDeque::from([false]),
        replies: VecDeque::from([Err(ControllerError::InvalidPath("x"))]),
        ..Scripted::default()
    });
    let g = s.send_goal(line(11, 0.1));
    s.cancel_goal(g);
    let out = s.tick(&at(&c, 0, Pose2D::default()));
    assert_eq!(results(&out), vec![result(g, GoalStatus::Canceled, error_code::INVALID_PATH)]);
}

#[test]
fn preemption() {
    let c = grid();
    let err = ControllerError::NoValidControl("x");
    let mut replies: VecDeque<_> = (0..30).map(|_| Err(err)).collect();
    replies.extend((0..30).map(|_| Err(err)));
    let mut s = server(Scripted { replies, ..Scripted::default() });
    let a = s.send_goal(line(11, 0.1));
    for k in 0..30 {
        assert!(s.tick(&at(&c, k, Pose2D::default())).results.is_empty());
    }
    let b = s.send_goal(line(21, 0.1));
    assert_eq!((s.current_goal(), s.pending_goal()), (Some(a), Some(b)));
    // the next pass aborts `a` with an empty result and runs `b`
    let out = s.tick(&at(&c, 30, Pose2D::default()));
    assert_eq!(results(&out), vec![result(a, GoalStatus::Aborted, error_code::NONE)]);
    assert_eq!(out.feedback.unwrap().0, b);
    assert!((out.feedback.unwrap().1.distance_to_goal as f64 - 2.0).abs() < 1e-6);
    assert_eq!(s.controller().plans.len(), 2);
    assert_eq!(s.controller().resets, 0, "a preemption is not the end of a task");
    // the patience clock was not restarted: it still counts from tick 0
    let (k, out) = run_until_result(&mut s, &c, 31, Pose2D::default());
    assert_eq!((k, results(&out)), (41, vec![result(b, GoalStatus::Aborted, error_code::PATIENCE_EXCEEDED)]));

    // nor the progress checker: still 20 s from the first goal's first pass
    let mut s = server(Scripted::default());
    s.send_goal(line(11, 0.1));
    for k in 0..200 {
        s.tick(&at(&c, k, Pose2D::default()));
    }
    let b = s.send_goal(line(11, 0.1));
    let (k, out) = run_until_result(&mut s, &c, 200, Pose2D::default());
    assert_eq!(k, 200);
    assert_eq!(out.results[0].error_code, error_code::NONE);
    let (k, out) = run_until_result(&mut s, &c, 201, Pose2D::default());
    assert_eq!((k, results(&out)), (401, vec![result(b, GoalStatus::Aborted, error_code::FAILED_TO_MAKE_PROGRESS)]));

    // but the goal checker is reset with the new path: stateful, so once xy
    // has passed only the yaw is checked. `a` ends at the robot facing the
    // other way (pi > 3.14: not reached, xy passed); `b` ends 1 m away.
    let mut s = server(Scripted::default());
    let a = s.send_goal(vec![Pose2D::new(0.0, 0.0, std::f64::consts::PI)]);
    let robot = Pose2D::new(0.01, 0.0, 0.0);
    assert!(s.tick(&at(&c, 0, robot)).results.is_empty());
    assert!(!s.goal_checker().checks_xy());
    let b = s.send_goal(line(11, 0.1));
    let out = s.tick(&at(&c, 1, robot));
    assert_eq!(results(&out), vec![result(a, GoalStatus::Aborted, error_code::NONE)]);
    assert_eq!(out.feedback.unwrap().0, b);
    assert!(s.goal_checker().checks_xy());
}

#[test]
fn preempting_with_an_empty_path() {
    let c = grid();
    let mut s = server(Scripted::default());
    let a = s.send_goal(line(11, 0.1));
    s.tick(&at(&c, 0, Pose2D::default()));
    let b = s.send_goal(Vec::new());
    let out = s.tick(&at(&c, 1, Pose2D::default()));
    assert_eq!(
        results(&out),
        vec![result(a, GoalStatus::Aborted, error_code::NONE), result(b, GoalStatus::Aborted, error_code::INVALID_PATH)]
    );
    assert!(!s.is_executing());
}

#[test]
fn a_second_pending_goal_replaces_the_first() {
    let c = grid();
    let mut s = server(Scripted::default());
    let a = s.send_goal(line(11, 0.1));
    s.tick(&at(&c, 0, Pose2D::default()));
    let b = s.send_goal(line(11, 0.1));
    let d = s.send_goal(line(21, 0.1));
    assert_eq!(s.pending_goal(), Some(d));
    let out = s.tick(&at(&c, 1, Pose2D::default()));
    assert_eq!(
        results(&out),
        vec![result(b, GoalStatus::Aborted, error_code::NONE), result(a, GoalStatus::Aborted, error_code::NONE)]
    );
    assert_eq!(s.current_goal(), Some(d));
}

#[test]
fn cancelling_around_a_pending_goal() {
    let c = grid();
    // cancelling the pending goal ends both: is_cancel_requested looks at it
    let mut s = server(Scripted::default());
    let a = s.send_goal(line(11, 0.1));
    s.tick(&at(&c, 0, Pose2D::default()));
    let b = s.send_goal(line(11, 0.1));
    assert!(s.cancel_goal(b));
    let out = s.tick(&at(&c, 1, Pose2D::default()));
    assert_eq!(
        results(&out),
        vec![result(a, GoalStatus::Aborted, error_code::NONE), result(b, GoalStatus::Canceled, error_code::NONE)]
    );
    assert!(!s.is_executing());

    // cancelling the running goal with one pending: not seen; the
    // preemption aborts it
    let mut s = server(Scripted::default());
    let a = s.send_goal(line(11, 0.1));
    s.tick(&at(&c, 0, Pose2D::default()));
    let b = s.send_goal(line(11, 0.1));
    assert!(s.cancel_goal(a));
    let out = s.tick(&at(&c, 1, Pose2D::default()));
    assert_eq!(results(&out), vec![result(a, GoalStatus::Aborted, error_code::NONE)]);
    assert_eq!(s.current_goal(), Some(b));
    assert_eq!(s.controller().cancels, 0);
}

#[test]
fn waiting_for_the_costmap() {
    let c = grid();
    let mut s = server(Scripted::default());
    let a = s.send_goal(line(11, 0.1));
    let stale = |k| CycleInput { costmap_current: false, ..at(&c, k, Pose2D::default()) };
    // 1.00 s is not more than the timeout; nothing is computed meanwhile
    for k in 0..=20 {
        let out = s.tick(&stale(k));
        assert!(out.waiting_for_costmap && out.commands.is_empty() && out.results.is_empty(), "tick {k}");
    }
    // a cancel during the wait is not seen until the wait is over
    s.cancel_goal(a);
    let out = s.tick(&at(&c, 21, Pose2D::default()));
    assert!(!out.waiting_for_costmap && out.results.is_empty() && out.commands.len() == 1);
    let out = s.tick(&at(&c, 22, Pose2D::default()));
    assert_eq!(results(&out), vec![result(a, GoalStatus::Canceled, error_code::NONE)]);

    // the wait restarts each time; past 1 s it ends the goal
    let mut s = server(Scripted::default());
    let a = s.send_goal(line(11, 0.1));
    for k in 0..=20 {
        assert!(s.tick(&stale(k)).waiting_for_costmap);
    }
    let out = s.tick(&stale(21));
    assert_eq!(results(&out), vec![result(a, GoalStatus::Aborted, error_code::CONTROLLER_TIMED_OUT)]);
    assert_eq!(out.results[0].error.unwrap().what(), "Costmap timed out waiting for update");

    // a goal pending when the wait times out runs right after, on the same pass
    let mut s = server(Scripted::default());
    let a = s.send_goal(line(11, 0.1));
    s.tick(&stale(0));
    let b = s.send_goal(line(11, 0.1));
    for k in 1..=20 {
        assert!(s.tick(&stale(k)).results.is_empty());
    }
    let mut input = stale(21);
    let out = s.tick(&input);
    assert_eq!(results(&out), vec![result(a, GoalStatus::Aborted, error_code::CONTROLLER_TIMED_OUT)]);
    assert_eq!(s.current_goal(), Some(b));
    assert!(out.waiting_for_costmap, "b's first pass waits from now");
    input.costmap_current = true;
    let out = s.tick(&input);
    assert_eq!(out.feedback.unwrap().0, b);
}

#[test]
fn publish_zero_velocity_off_and_speed_limits() {
    let c = grid();
    let params = ControllerServerParams { publish_zero_velocity: false, ..ControllerServerParams::mower() };
    let mut s = server_with(params, Scripted::default());
    let g = s.send_goal(Vec::new());
    let out = s.tick(&at(&c, 0, Pose2D::default()));
    assert_eq!(results(&out), vec![result(g, GoalStatus::Aborted, error_code::INVALID_PATH)]);
    assert!(out.commands.is_empty());
    assert_eq!(s.controller().resets, 1, "the controllers are reset regardless");
    s.set_speed_limit(50.0, true);
    assert_eq!(s.controller().speed_limits, vec![(50.0, true)]);
}

#[test]
fn without_a_progress_checker() {
    let c = grid();
    let mut s = ControllerServer::new(
        ControllerServerParams::mower(),
        Scripted::default(),
        SimpleGoalChecker::new(SimpleGoalCheckerParams::mower()),
        None,
    );
    s.send_goal(line(11, 0.1));
    for k in 0..1000 {
        assert!(s.tick(&at(&c, k, Pose2D::default())).results.is_empty());
    }
}

#[test]
fn feedback_measures_from_the_robot_in_the_plan_frame() {
    // map -> odom is 1 m along x: the robot at odom (0.5, 0) is at map (-0.5, 0)
    let c = grid();
    let mut s = server(Scripted::default());
    let path: Vec<Pose2D> = (0..21).map(|i| Pose2D::new(-1.0 + 0.1 * i as f64, 0.0, 0.0)).collect();
    s.send_goal(path);
    let mut input = at(&c, 0, Pose2D::new(0.5, 0.0, 0.0));
    input.odom_from_map = Some(Transform2D::new(1.0, 0.0, 0.0));
    let out = s.tick(&input);
    // closest map pose is index 5 (-0.5): 1.5 m left. upstream compares the
    // odom pose (0.5, 0) with map poses: index 15, 0.5 m.
    assert!((out.feedback.unwrap().1.distance_to_goal as f64 - 1.5).abs() < 1e-6);
    assert_eq!(s.controller().inputs[0].3, Transform2D::new(1.0, 0.0, 0.0));
}

// --- the mower's FollowPath in a closed loop -------------------------------------

struct Sim {
    /// the robot in odom
    pose: Pose2D,
    twist: Twist2D,
    window: RollingWindow,
    map: Costmap2D,
    map_from_odom: Transform2D,
}

impl Sim {
    fn new(lethal_from_x: Option<f64>) -> Self {
        // map frame: x -2..4, y -2..2 at 5 cm
        let (w, h) = (120u32, 80u32);
        let mut data = vec![0i8; (w * h) as usize];
        if let Some(x0) = lethal_from_x {
            for j in 0..h {
                for i in 0..w {
                    let x = -2.0 + (i as f64 + 0.5) * 0.05;
                    if x >= x0 && x < x0 + 0.2 {
                        data[(j * w + i) as usize] = 100;
                    }
                }
            }
        }
        let grid = OccupancyGrid { width: w, height: h, resolution: 0.05f32 as f64, origin_x: -2.0, origin_y: -2.0, data };
        let map_from_odom = Transform2D::new(0.3, -0.2, 0.1);
        let pose = map_from_odom.inverse().apply_pose(&Pose2D::new(0.0, 0.05, 0.05));
        Sim { pose, twist: Twist2D::default(), window: RollingWindow::local_costmap(), map: static_costmap(&grid, &StaticLayerParams::new(false)), map_from_odom }
    }

    fn step(&mut self, cmd: Option<&Twist2D>, dt: f64) {
        if let Some(c) = cmd {
            self.twist = *c;
        }
        let (s, c) = self.pose.yaw.sin_cos();
        self.pose = Pose2D::new(self.pose.x + self.twist.x * c * dt, self.pose.y + self.twist.x * s * dt, self.pose.yaw + self.twist.theta * dt);
    }
}

fn drive(sim: &mut Sim, path: Vec<Pose2D>, seconds: f64) -> (FollowPathResult, Vec<(i64, Twist2D)>, Vec<f32>) {
    let mut s = ControllerServer::mower();
    s.send_goal(path);
    let mut cmds = Vec::new();
    let mut distances = Vec::new();
    for k in 0..(seconds * 20.0) as i64 {
        sim.window.update(sim.pose.x, sim.pose.y, &sim.map, &sim.map_from_odom);
        let input = CycleInput {
            now_ns: T0 + k * DT,
            costmap_current: true,
            robot_pose: Some(sim.pose),
            odom_twist: sim.twist,
            odom_from_map: Some(sim.map_from_odom.inverse()),
            costmap: sim.window.costmap(),
        };
        let out = s.tick(&input);
        for c in &out.commands {
            cmds.push((c.stamp_ns, c.twist));
        }
        if let Some((_, f)) = out.feedback {
            distances.push(f.distance_to_goal);
        }
        if let Some(r) = out.results.first() {
            return (*r, cmds, distances);
        }
        sim.step(out.commands.last().map(|c| &c.twist), 0.05);
    }
    panic!("no result in {seconds} s");
}

fn straight_plan(length: f64) -> Vec<Pose2D> {
    (0..=(length / 0.1).round() as usize).map(|i| Pose2D::new(0.1 * i as f64, 0.0, 0.0)).collect()
}

#[test]
fn follows_a_straight_segment_to_the_end() {
    let mut sim = Sim::new(None);
    let (r, cmds, distances) = drive(&mut sim, straight_plan(1.5), 30.0);
    assert_eq!((r.status, r.error_code), (GoalStatus::Succeeded, error_code::NONE));
    let end = sim.map_from_odom.apply_pose(&sim.pose);
    println!(
        "straight 1.5 m: succeeded after {:.2} s at ({:.3}, {:.3}), {} commands, max {:.3} m/s",
        (cmds.last().unwrap().0 - T0) as f64 / 1e9,
        end.x,
        end.y,
        cmds.len(),
        cmds.iter().map(|c| c.1.x).fold(0.0, f64::max)
    );
    assert!((end.x - 1.5).hypot(end.y) <= 0.05 + 0.26 * 0.05, "ended at {end:?}");
    assert!(cmds.iter().all(|(_, t)| t.x <= 0.26 + 1e-12 && t.x >= 0.0));
    assert_eq!(cmds.last().unwrap().1, Twist2D::default());
    // the distance left only shrinks, up to one pose spacing of noise
    assert!(distances.windows(2).all(|w| w[1] <= w[0] + 1e-6), "{distances:?}");
    assert!(*distances.first().unwrap() > 1.3 && *distances.last().unwrap() < 0.15);
}

#[test]
fn a_wall_across_the_path_exhausts_the_patience() {
    let mut sim = Sim::new(Some(1.2));
    let (r, cmds, _) = drive(&mut sim, straight_plan(2.5), 60.0);
    assert_eq!((r.status, r.error_code), (GoalStatus::Aborted, error_code::PATIENCE_EXCEEDED));
    // stopped short of the wall (footprint radius 0.5 + padding)
    let stop = sim.map_from_odom.apply_pose(&sim.pose);
    assert!(stop.x < 1.2 - 0.5 && stop.x > 0.0, "stopped at {stop:?}");
    // the last moving command, then zeros, then the abort just after 2 s
    let last_moving = cmds.iter().rev().find(|(_, t)| t.x != 0.0).unwrap().0;
    let abort = cmds.last().unwrap().0;
    let held = (abort - last_moving) as f64 / 1e9;
    println!(
        "wall at x 1.2: stopped at ({:.3}, {:.3}), last command {:.2} s, patience exceeded {:.2} s after it",
        stop.x,
        stop.y,
        (last_moving - T0) as f64 / 1e9,
        held
    );
    assert!(held > 2.0 && held <= 2.05 + 1e-9, "aborted {held} s after the last command");
}
