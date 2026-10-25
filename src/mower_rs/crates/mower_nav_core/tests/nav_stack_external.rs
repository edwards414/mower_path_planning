//! `NavStack` with an external planner against the same stack with its
//! inline one: the tree's `compute_path_to_pose` goals leave as
//! `StackOutput::plan_requests`, a `PlannerServer` the test runs on the
//! global costmap answers them, and the results come back through
//! `NavStack::on_compute_path_result`.
//!
//! Answered at once (before the next tick), the external stack must do
//! exactly what the inline one does, tick for tick: the inline planner's pass
//! also comes after the tree's tick, and the tree sees its results at its next
//! tick. Answered later, as a planner on its own thread does after a long
//! search, it must still end every goal the same way.

use mower_nav_core::action::{GoalId, GoalStatus};
use mower_nav_core::costmap::{static_costmap, Costmap2D, OccupancyGrid, RollingWindow, StaticLayerParams};
use mower_nav_core::geometry::{Pose2D, Transform2D, Twist2D};
use mower_nav_core::nav_stack::{NavStack, PlanRequest, StackInput, StackOutput, COSTMAP_POLL_NS};
use mower_nav_core::navigator::CostmapId;
use mower_nav_core::planner_server::{ComputePathResult, PlanInput, PlannerServer};

const MS: i64 = 1_000_000;
const T0: i64 = 1_792_900_000_000_000_000;

/// 10 x 8 m at 0.05 m from (-1, -1): free inside a lethal border, plus the
/// lethal boxes `(x0, y0, x1, y1)`.
fn grid(boxes: &[(f64, f64, f64, f64)]) -> OccupancyGrid {
    let (res, ox, oy, w, h) = (0.05, -1.0, -1.0, 200u32, 160u32);
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
    OccupancyGrid { width: w, height: h, resolution: res as f32 as f64, origin_x: ox, origin_y: oy, data }
}

/// The external planner_server: the test's `PlannerServer` on the global
/// costmap, ticked on the schedule `NavStack` gives its inline one, its
/// results handed back `delay_ms` later.
struct Planner {
    server: PlannerServer,
    next_ns: Option<i64>,
    /// The stack's ids by the server's (the same, sent in the same order).
    ids: Vec<(GoalId, GoalId)>,
    outbox: Vec<(i64, ComputePathResult)>,
    delay_ns: i64,
    plans: usize,
}

#[derive(Debug, PartialEq)]
struct Played {
    /// (status, error code, result time in ms)
    outcome: (GoalStatus, u16, i64),
    recoveries: i16,
    plan_goals: u64,
    plans: usize,
    final_pose: Pose2D,
}

/// `delay_ms`: `None` plays the inline stack.
fn play(goal: Pose2D, global_grid: &OccupancyGrid, cancel_at_ms: Option<i64>, delay_ms: Option<i64>) -> Played {
    let fresh_global = || static_costmap(global_grid, &StaticLayerParams::new(true));
    let mut global: Costmap2D = fresh_global();
    let local_map = static_costmap(&grid(&[]), &StaticLayerParams::new(false));
    let map_from_odom = Transform2D::new(0.4, -0.3, 0.2);
    let mut odom = map_from_odom.inverse().apply_pose(&Pose2D::new(1.0, 1.0, 0.0));
    let mut twist = Twist2D::default();
    let mut window = RollingWindow::local_costmap();
    window.update(odom.x, odom.y, &local_map, &map_from_odom);
    let (mut global_current, mut local_current) = (true, true);
    let mut placeholder = Costmap2D::new(1, 1, 0.05, 0.0, 0.0, 255);

    let mut inline = NavStack::mower(&global);
    let mut external = NavStack::mower_external_planner();
    let mut planner = Planner {
        server: PlannerServer::mower(&global),
        next_ns: None,
        ids: Vec::new(),
        outbox: Vec::new(),
        delay_ns: delay_ms.unwrap_or(0) * MS,
        plans: 0,
    };
    let id = match delay_ms {
        None => inline.send_navigate(goal),
        Some(_) => external.send_navigate(goal),
    };
    let mut recoveries = -1;
    let mut plans = 0;
    for ms in 0..120_000 {
        let now = T0 + ms * MS;
        if Some(ms) == cancel_at_ms {
            match delay_ms {
                None => inline.cancel_navigate(id),
                Some(_) => external.cancel_navigate(id),
            };
        }
        // the 1 Hz global and 2 Hz local costmaps; a reset one is current
        // again, and repainted, at its next update
        if ms % 1000 == 0 && !global_current {
            global = fresh_global();
            global_current = true;
        }
        if ms % 500 == 0 {
            window.update(odom.x, odom.y, &local_map, &map_from_odom);
            local_current = true;
        }
        // results the planner sent back, before the tick
        if delay_ms.is_some() {
            let (due, later): (Vec<_>, Vec<_>) = planner.outbox.drain(..).partition(|(at, _)| *at <= now);
            planner.outbox = later;
            for (_, r) in due {
                external.on_compute_path_result(r);
            }
        }
        macro_rules! input {
            ($global:expr, $current:expr) => {
                StackInput {
                    now_ns: now,
                    map_pose: Some(map_from_odom.apply_pose(&odom)),
                    odom_pose: Some(odom),
                    odom_twist: twist,
                    odom_from_map: Some(map_from_odom.inverse()),
                    local_costmap: window.costmap(),
                    local_costmap_current: local_current,
                    global_costmap: $global,
                    global_costmap_current: $current,
                }
            };
        }
        let out: StackOutput = match delay_ms {
            None => inline.tick(input!(&mut global, global_current)),
            // the external stack never touches the global costmap
            Some(_) => external.tick(input!(&mut placeholder, false)),
        };
        for c in &out.clears {
            match c {
                CostmapId::Global => global_current = false,
                CostmapId::Local => local_current = false,
            }
        }
        plans += out.plans.len();
        if delay_ms.is_some() {
            assert!(out.plans.is_empty() && out.log.iter().all(|l| !matches!(l, mower_nav_core::nav_stack::StackLog::ComputePathFailed(_))));
            // the planner side: the tick's clears are applied above, then its
            // requests, then a pass when one is due (as NavStack schedules it)
            let was_idle = planner.server.current_goal().is_none();
            for r in out.plan_requests {
                match r {
                    PlanRequest::Send { id, goal } => {
                        let server_id = planner.server.send_goal(goal);
                        assert_eq!(server_id, id, "the stack numbers its goals as the server does");
                        planner.ids.push((server_id, id));
                    }
                    PlanRequest::Cancel(id) => {
                        let server_id = planner.ids.iter().find(|(_, ext)| *ext == id).unwrap().0;
                        planner.server.cancel_goal(server_id);
                    }
                }
            }
            if was_idle && planner.server.has_work() {
                planner.next_ns = Some(now);
            }
            if planner.next_ns.is_some_and(|t| t <= now) && planner.server.has_work() {
                let p = planner.server.tick(PlanInput {
                    now_ns: now,
                    costmap_current: global_current,
                    robot_pose: Some(map_from_odom.apply_pose(&odom)),
                    costmap: &mut global,
                });
                planner.plans += p.plans.len();
                for r in p.results {
                    planner.outbox.push((now + planner.delay_ns, r));
                }
                planner.next_ns = p.waiting_for_costmap.then_some(now + COSTMAP_POLL_NS);
            } else if !planner.server.has_work() {
                planner.next_ns = None;
            }
        } else {
            assert!(out.plan_requests.is_empty());
        }
        if let Some(c) = out.commands.last() {
            twist = c.twist;
        }
        if let Some((_, fb)) = out.navigate_feedback {
            recoveries = fb.number_of_recoveries;
        }
        if let Some(r) = out.navigate_results.first() {
            let plan_goals = match delay_ms {
                None => inline.planner.goals_received(),
                Some(_) => external.planner.goals_sent(),
            };
            let plans = if delay_ms.is_some() { planner.plans } else { plans };
            return Played { outcome: (r.status, r.error_code, ms), recoveries, plan_goals, plans, final_pose: odom };
        }
        // the 50 Hz robot
        if ms % 20 == 0 {
            let (s, c) = odom.yaw.sin_cos();
            odom = Pose2D::new(odom.x + twist.x * c * 0.02, odom.y + twist.x * s * 0.02, odom.yaw + twist.theta * 0.02);
        }
    }
    panic!("the goal did not end");
}

#[test]
fn answered_at_once_the_external_planner_plays_tick_for_tick_like_the_inline_one() {
    let free = grid(&[]);
    let ring = grid(&[(5.0, 1.0, 7.0, 1.2), (5.0, 2.8, 7.0, 3.0), (5.0, 1.0, 5.2, 3.0), (6.8, 1.0, 7.0, 3.0)]);
    let cases = [
        ("reach", Pose2D::new(5.0, 3.0, 0.5), &free, None),
        ("unreachable", Pose2D::new(6.0, 2.0, 0.0), &ring, None),
        ("cancel", Pose2D::new(6.0, 4.0, 0.0), &free, Some(3_000)),
    ];
    for (name, goal, map, cancel) in cases {
        let inline = play(goal, map, cancel, None);
        let external = play(goal, map, cancel, Some(0));
        assert_eq!(external, inline, "{name}");
        match name {
            "reach" => assert_eq!(inline.outcome.0, GoalStatus::Succeeded),
            // README: 7 x 2 plans and 19 clears for NO_VALID_PATH
            "unreachable" => assert_eq!((inline.outcome.0, inline.outcome.1, inline.recoveries, inline.plan_goals), (GoalStatus::Aborted, 208, 19, 14)),
            _ => assert_eq!(inline.outcome.0, GoalStatus::Canceled),
        }
    }
}

#[test]
fn answered_late_the_external_planner_still_ends_every_goal_the_same_way() {
    let free = grid(&[]);
    let ring = grid(&[(5.0, 1.0, 7.0, 1.2), (5.0, 2.8, 7.0, 3.0), (5.0, 1.0, 5.2, 3.0), (6.8, 1.0, 7.0, 3.0)]);
    for (name, goal, map) in [("reach", Pose2D::new(5.0, 3.0, 0.5), &free), ("unreachable", Pose2D::new(6.0, 2.0, 0.0), &ring)] {
        let inline = play(goal, map, None, None);
        let late = play(goal, map, None, Some(120));
        assert_eq!((late.outcome.0, late.outcome.1, late.recoveries), (inline.outcome.0, inline.outcome.1, inline.recoveries), "{name}");
        assert!(late.outcome.2 >= inline.outcome.2, "{name}: {late:?} ended before {inline:?}");
        if name == "reach" {
            let (a, b) = (late.final_pose, inline.final_pose);
            assert!((a.x - b.x).hypot(a.y - b.y) < 0.1, "{name}: {a:?} vs {b:?}");
        }
    }
}

#[test]
fn the_external_planner_numbers_goals_and_forwards_cancels() {
    let mut stack = NavStack::mower_external_planner();
    let map = static_costmap(&grid(&[]), &StaticLayerParams::new(true));
    let local = RollingWindow::local_costmap();
    let mut placeholder = map.clone();
    let id = stack.send_navigate(Pose2D::new(5.0, 3.0, 0.0));
    let tick = |stack: &mut NavStack<_, _>, now: i64, placeholder: &mut Costmap2D| {
        stack.tick(StackInput {
            now_ns: now,
            map_pose: Some(Pose2D::new(1.0, 1.0, 0.0)),
            odom_pose: Some(Pose2D::new(1.0, 1.0, 0.0)),
            odom_twist: Twist2D::default(),
            odom_from_map: Some(Transform2D::IDENTITY),
            local_costmap: local.costmap(),
            local_costmap_current: true,
            global_costmap: placeholder,
            global_costmap_current: false,
        })
    };
    let out = tick(&mut stack, T0, &mut placeholder);
    assert!(matches!(out.plan_requests.as_slice(), [PlanRequest::Send { id: 1, .. }]), "{:?}", out.plan_requests);
    assert_eq!(stack.next_tick_ns(), Some(T0 + 10 * MS), "the tree's 10 ms; nothing scheduled for the planner");
    stack.cancel_navigate(id);
    let out = tick(&mut stack, T0 + 10 * MS, &mut placeholder);
    assert_eq!(out.plan_requests, [PlanRequest::Cancel(1)]);
    assert_eq!(out.navigate_results.len(), 1);
    assert_eq!(out.navigate_results[0].status, GoalStatus::Canceled);
    assert_eq!(stack.planner.goals_sent(), 1);
}
