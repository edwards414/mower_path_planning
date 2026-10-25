//! `planner_server`'s compute_path_to_pose loop: the costmap wait, the error
//! codes, cancel and the SimpleActionServer quirks. The plans themselves are
//! NavFn's, checked against nav2 in navfn_oracle.rs.

use mower_nav_core::action::GoalStatus;
use mower_nav_core::costmap::{Costmap2D, FREE_SPACE, LETHAL_OBSTACLE};
use mower_nav_core::geometry::Pose2D;
use mower_nav_core::planner_server::{error_code, ComputePathToPose, PlanInput, PlanOutput, PlannerServer};

const MS: i64 = 1_000_000;

/// 4 x 4 m, 0.1 m cells, origin (-2, -2); a wall at x in [0.5, 0.6) with
/// `wall`.
fn costmap(wall: bool) -> Costmap2D {
    let mut c = Costmap2D::new(40, 40, 0.1, -2.0, -2.0, FREE_SPACE);
    if wall {
        for my in 0..40 {
            c.set_cost(25, my, LETHAL_OBSTACLE);
        }
    }
    c
}

fn tick(s: &mut PlannerServer, c: &mut Costmap2D, ms: i64, current: bool, robot: Option<Pose2D>) -> PlanOutput {
    s.tick(PlanInput { now_ns: ms * MS, costmap_current: current, robot_pose: robot, costmap: c })
}

const ROBOT: Option<Pose2D> = Some(Pose2D { x: -1.0, y: 0.0, yaw: 0.0 });

#[test]
fn plans_from_the_robot_in_one_pass() {
    let mut c = costmap(false);
    let mut s = PlannerServer::mower(&c);
    let g = s.send_goal(ComputePathToPose::to(Pose2D::new(1.0, 0.5, 0.3)));
    let out = tick(&mut s, &mut c, 0, true, ROBOT);
    assert_eq!(out.results.len(), 1);
    let r = &out.results[0];
    assert_eq!((r.goal, r.status, r.error_code), (g, GoalStatus::Succeeded, error_code::NONE));
    assert_eq!(out.plans, vec![r.path.clone()]);
    assert_eq!(*r.path.last().unwrap(), Pose2D::new(1.0, 0.5, 0.3));
    assert!((r.path[0].x + 1.0).abs() < 0.06 && r.path[0].y.abs() < 0.06);
    assert!(!s.has_work());
}

#[test]
fn waits_for_the_costmap_then_times_out() {
    let mut c = costmap(false);
    let mut s = PlannerServer::mower(&c);
    let g = s.send_goal(ComputePathToPose::to(Pose2D::new(1.0, 0.0, 0.0)));
    for ms in (0..=1000).step_by(10) {
        let out = tick(&mut s, &mut c, ms, false, ROBOT);
        assert!(out.waiting_for_costmap && out.results.is_empty(), "{ms} ms");
    }
    // strictly more than costmap_update_timeout
    let out = tick(&mut s, &mut c, 1010, false, ROBOT);
    assert!(!out.waiting_for_costmap);
    let r = &out.results[0];
    assert_eq!((r.goal, r.status, r.error_code), (g, GoalStatus::Aborted, error_code::TIMEOUT));
    assert_eq!(r.planning_time_ns, 1010 * MS);

    // current within the timeout: planned, and the wait was the planning time
    let g = s.send_goal(ComputePathToPose::to(Pose2D::new(1.0, 0.0, 0.0)));
    tick(&mut s, &mut c, 2000, false, ROBOT);
    let out = tick(&mut s, &mut c, 2600, true, ROBOT);
    let r = &out.results[0];
    assert_eq!((r.goal, r.status), (g, GoalStatus::Succeeded));
    assert_eq!(r.planning_time_ns, 600 * MS);
    // slower than expected_planner_frequency (20 Hz)
    assert_eq!(out.missed_rate, Some(0.6));
}

#[test]
fn error_codes() {
    let mut c = costmap(true);
    let mut s = PlannerServer::mower(&c);
    let cases: [(ComputePathToPose, Option<Pose2D>, u16); 7] = [
        (ComputePathToPose::to(Pose2D::new(1.0, 0.0, 0.0)), None, error_code::TF_ERROR),
        (ComputePathToPose { planner_id: "Smac".into(), ..ComputePathToPose::to(Pose2D::new(-0.5, 0.0, 0.0)) }, ROBOT, error_code::INVALID_PLANNER),
        (ComputePathToPose::to(Pose2D::new(-0.5, 0.0, 0.0)), Some(Pose2D::new(-2.5, 0.0, 0.0)), error_code::START_OUTSIDE_MAP),
        (ComputePathToPose::to(Pose2D::new(2.5, 0.0, 0.0)), ROBOT, error_code::GOAL_OUTSIDE_MAP),
        (ComputePathToPose::to(Pose2D::new(0.55, 0.0, 0.0)), ROBOT, error_code::GOAL_OCCUPIED),
        // behind the wall
        (ComputePathToPose::to(Pose2D::new(1.0, 0.0, 0.0)), ROBOT, error_code::NO_VALID_PATH),
        // a start in the goal makes TF unnecessary
        (ComputePathToPose { start: Some(Pose2D::new(1.0, 0.0, 0.0)), ..ComputePathToPose::to(Pose2D::new(1.5, 0.0, 0.0)) }, None, error_code::NONE),
    ];
    for (i, (goal, robot, code)) in cases.into_iter().enumerate() {
        let g = s.send_goal(goal);
        let out = tick(&mut s, &mut c, i as i64 * 100, true, robot);
        let r = &out.results[0];
        assert_eq!((r.goal, r.error_code), (g, code), "case {i}");
        let status = if code == error_code::NONE { GoalStatus::Succeeded } else { GoalStatus::Aborted };
        assert_eq!(r.status, status, "case {i}");
        assert_eq!(out.plans.is_empty(), code != error_code::NONE, "case {i}");
    }
}

#[test]
fn a_cancel_ends_the_goal_with_an_empty_result() {
    let mut c = costmap(false);
    let mut s = PlannerServer::mower(&c);
    // before it starts
    let g = s.send_goal(ComputePathToPose::to(Pose2D::new(1.0, 0.0, 0.0)));
    assert!(s.cancel_goal(g));
    let out = tick(&mut s, &mut c, 0, true, ROBOT);
    assert_eq!((out.results[0].status, out.results[0].error_code), (GoalStatus::Canceled, 0));
    assert!(out.plans.is_empty());
    // while it waits for the costmap: not seen until NavFn runs
    let g = s.send_goal(ComputePathToPose::to(Pose2D::new(1.0, 0.0, 0.0)));
    tick(&mut s, &mut c, 100, false, ROBOT);
    s.cancel_goal(g);
    assert!(tick(&mut s, &mut c, 110, false, ROBOT).waiting_for_costmap);
    let out = tick(&mut s, &mut c, 120, true, ROBOT);
    assert_eq!((out.results[0].goal, out.results[0].status), (g, GoalStatus::Canceled));
    // NavFn never runs when the robot is at the goal: that plan succeeds
    let at = Pose2D::new(-1.0, 0.0, 0.5);
    let g = s.send_goal(ComputePathToPose::to(at));
    tick(&mut s, &mut c, 200, false, ROBOT);
    s.cancel_goal(g);
    let out = tick(&mut s, &mut c, 210, true, ROBOT);
    assert_eq!((out.results[0].goal, out.results[0].status), (g, GoalStatus::Succeeded));
    assert_eq!(out.results[0].path, vec![at]);
}

#[test]
fn a_preempting_goal_gets_the_plan_of_the_goal_it_preempted() {
    let mut c = costmap(false);
    let mut s = PlannerServer::mower(&c);
    let a = s.send_goal(ComputePathToPose::to(Pose2D::new(1.0, 0.0, 0.0)));
    tick(&mut s, &mut c, 0, false, ROBOT);
    let b = s.send_goal(ComputePathToPose::to(Pose2D::new(-1.0, 1.5, 0.0)));
    assert_eq!(s.pending_goal(), Some(b));
    let out = tick(&mut s, &mut c, 50, true, ROBOT);
    assert_eq!(out.results.len(), 2);
    assert_eq!((out.results[0].goal, out.results[0].status, out.results[0].error_code), (a, GoalStatus::Aborted, 0));
    assert_eq!((out.results[1].goal, out.results[1].status), (b, GoalStatus::Succeeded));
    // planned to a's goal
    assert_eq!(*out.results[1].path.last().unwrap(), Pose2D::new(1.0, 0.0, 0.0));
}

#[test]
fn pending_goals() {
    let mut c = costmap(false);
    let mut s = PlannerServer::mower(&c);
    let a = s.send_goal(ComputePathToPose::to(Pose2D::new(1.0, 0.0, 0.0)));
    tick(&mut s, &mut c, 0, false, ROBOT);
    let b = s.send_goal(ComputePathToPose::to(Pose2D::new(1.0, 1.0, 0.0)));
    let c2 = s.send_goal(ComputePathToPose::to(Pose2D::new(1.0, -1.0, 0.0)));
    // b was replaced in the pending slot
    let out = tick(&mut s, &mut c, 10, false, ROBOT);
    assert_eq!(out.results.len(), 1);
    assert_eq!((out.results[0].goal, out.results[0].status), (b, GoalStatus::Aborted));
    // cancelling the pending goal is what is_cancel_requested sees, but only
    // at the next goal's entry: here c2 preempts a and plans a's goal
    s.cancel_goal(c2);
    let out = tick(&mut s, &mut c, 20, true, ROBOT);
    let got: Vec<_> = out.results.iter().map(|r| (r.goal, r.status)).collect();
    assert_eq!(got, vec![(a, GoalStatus::Aborted), (c2, GoalStatus::Canceled)]);
    assert_eq!(s.goals_received(), 3);
    assert!(!s.has_work());
}
