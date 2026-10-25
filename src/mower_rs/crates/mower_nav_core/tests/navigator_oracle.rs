//! `NavStack` (navigator + planner server + controller server) against the
//! real nav2 bt_navigator, planner_server and controller_server, black box:
//! tests/oracle/navigator/scenarios.json played by scenarios.py on the real
//! nodes (tests/data/navigator.json, wall clock) and here on the port
//! (simulated clock), with the same kinematic robot, maps and events. The
//! costmaps update with the phase the real ones had (their publications'
//! stamps), and a cleared costmap is not current until its next update.
//!
//! The real nodes run on their own threads, so their times carry a few ms of
//! scheduling per hand-over: every goal's status and error code must match
//! exactly, as must the number of recoveries and of compute_path_to_pose
//! goals; result times, plans and follow_path goals to within a little.

mod common;

use common::{f, load};
use mower_nav_core::action::GoalStatus;
use mower_nav_core::costmap::{static_costmap, Costmap2D, OccupancyGrid, RollingWindow, StaticLayerParams};
use mower_nav_core::geometry::{Pose2D, Transform2D, Twist2D};
use mower_nav_core::nav_stack::{NavStack, StackInput};
use mower_nav_core::navigator::CostmapId;
use serde_json::Value;

const MS: i64 = 1_000_000;

/// The scenario where nav2 itself is not repeatable. A replan whose result
/// reaches the tree while the controller's PATIENCE_EXCEEDED result for the
/// running goal is still in flight (a few ms of DDS latency) makes FollowPath
/// send the new plan as an update; the controller, idle by then, starts a
/// fresh loop with a fresh patience clock, and the tree drops the failure
/// because it no longer waits on that goal id. That round then lasts one
/// patience period (2.05 s) longer. Three runs of the real nodes took 35.05 s,
/// 37.04 s and 39.5 s (0, 1 and 2 such races); the port hands results over
/// in the same tick and always takes the race-free path.
const RACE_SCENARIO: &str = "wall_only_in_local_costmap";
const RACE_S: f64 = 2.1;

fn status_name(s: GoalStatus) -> &'static str {
    match s {
        GoalStatus::Succeeded => "succeeded",
        GoalStatus::Aborted => "aborted",
        GoalStatus::Canceled => "canceled",
    }
}

fn pose(v: &Value) -> Pose2D {
    Pose2D::new(f(&v[0]), f(&v[1]), f(&v[2]))
}

/// scenarios.py's `grid`.
fn grid(spec: &Value) -> OccupancyGrid {
    let (w, h) = (spec["width"].as_u64().unwrap() as u32, spec["height"].as_u64().unwrap() as u32);
    let res = f(&spec["resolution"]);
    let (ox, oy) = (f(&spec["origin"][0]), f(&spec["origin"][1]));
    let mut data = vec![0i8; (w * h) as usize];
    for j in 0..h {
        for i in 0..w {
            let (x, y) = (ox + (i as f64 + 0.5) * res, oy + (j as f64 + 0.5) * res);
            let mut lethal = false;
            if let Some(b) = spec["lethal_x"].as_array() {
                lethal |= f(&b[0]) <= x && x < f(&b[1]);
            }
            if let Some(r) = spec["ring"].as_array() {
                let d = (x - f(&r[0])).abs().max((y - f(&r[1])).abs());
                lethal |= f(&r[2]) <= d && d < f(&r[3]);
            }
            if lethal {
                data[(j * w + i) as usize] = 100;
            }
        }
    }
    // MapMetaData.resolution is a float32
    OccupancyGrid { width: w, height: h, resolution: res as f32 as f64, origin_x: ox, origin_y: oy, data }
}

/// When a costmap updated, as ms in its period (the circular mean of its
/// publications' stamps).
fn phase_ms(stamps: &Value, period_s: f64) -> i64 {
    let (mut s, mut c) = (0.0, 0.0);
    for t in stamps.as_array().unwrap() {
        let a = f(t) / period_s * std::f64::consts::TAU;
        s += a.sin();
        c += a.cos();
    }
    let phase = s.atan2(c).rem_euclid(std::f64::consts::TAU) / std::f64::consts::TAU * period_s;
    (phase * 1000.0).round() as i64 % (period_s * 1000.0).round() as i64
}

#[derive(Debug, Default)]
struct Played {
    /// (status, error code, result time)
    goals: Vec<Option<(&'static str, u16, f64)>>,
    /// The last feedback's number_of_recoveries, -1 before any.
    recoveries: Vec<i64>,
    final_pose: Pose2D,
    plans: usize,
    /// When each compute_path_to_pose / follow_path goal went out, s.
    compute_goals: Vec<f64>,
    follow_goals: Vec<f64>,
}

fn play(sc: &Value, maps: &Value, map_from_odom: Transform2D, global_phase: i64, local_phase: i64) -> Played {
    let global_grid = grid(&maps[sc["global"].as_str().unwrap()]);
    let local_map = static_costmap(&grid(&maps[sc["local"].as_str().unwrap()]), &StaticLayerParams::new(false));
    let fresh_global = || static_costmap(&global_grid, &StaticLayerParams::new(true));
    let mut global: Costmap2D = fresh_global();
    let (mut global_current, mut local_current) = (true, true);
    let kinematic = sc["robot"] == "kinematic";
    let mut odom = map_from_odom.inverse().apply_pose(&pose(&sc["start"]));
    let mut twist = Twist2D::default();
    let mut window = RollingWindow::local_costmap();
    window.update(odom.x, odom.y, &local_map, &map_from_odom);
    let mut stack = NavStack::mower(&global);
    let mut events: Vec<&Value> = sc["events"].as_array().unwrap().iter().collect();
    events.sort_by(|a, b| f(&a["t"]).partial_cmp(&f(&b["t"])).unwrap());
    let mut ids = Vec::new();
    let mut played = Played::default();
    let mut next_event = 0;
    let duration_ms = (f(&sc["duration"]) * 1000.0).round() as i64;
    let t0 = 1_792_900_000_000_000_000i64;
    for ms in 0..duration_ms + 5000 {
        let now = t0 + ms * MS;
        let secs = ms as f64 / 1000.0;
        while next_event < events.len() && (f(&events[next_event]["t"]) * 1000.0).round() as i64 == ms {
            let ev = events[next_event];
            next_event += 1;
            if ev["op"] == "send" {
                ids.push(stack.send_navigate(pose(&ev["pose"])));
                played.goals.push(None);
                played.recoveries.push(-1);
            } else {
                stack.cancel_navigate(ids[ev["goal"].as_u64().unwrap() as usize]);
            }
        }
        if ms == duration_ms {
            for (i, g) in played.goals.iter().enumerate() {
                if g.is_none() {
                    stack.cancel_navigate(ids[i]);
                }
            }
        }
        if ms > duration_ms && played.goals.iter().all(Option::is_some) {
            break;
        }
        // the 1 Hz global and 2 Hz local costmaps; a reset one is current
        // again, and repainted, at its next update
        if (ms - global_phase).rem_euclid(1000) == 0 && !global_current {
            global = fresh_global();
            global_current = true;
        }
        if (ms - local_phase).rem_euclid(500) == 0 {
            window.update(odom.x, odom.y, &local_map, &map_from_odom);
            local_current = true;
        }
        let out = stack.tick(StackInput {
            now_ns: now,
            map_pose: Some(map_from_odom.apply_pose(&odom)),
            odom_pose: Some(odom),
            odom_twist: if kinematic { twist } else { Twist2D::default() },
            odom_from_map: Some(map_from_odom.inverse()),
            local_costmap: window.costmap(),
            local_costmap_current: local_current,
            global_costmap: &mut global,
            global_costmap_current: global_current,
        });
        for c in out.clears {
            match c {
                CostmapId::Global => global_current = false,
                CostmapId::Local => local_current = false,
            }
        }
        played.plans += out.plans.len();
        for _ in played.compute_goals.len() as u64..stack.planner.goals_received() {
            played.compute_goals.push(secs);
        }
        for _ in played.follow_goals.len() as u64..stack.controller.goals_received() {
            played.follow_goals.push(secs);
        }
        if let Some(c) = out.commands.last() {
            twist = c.twist;
        }
        if let Some((goal, fb)) = out.navigate_feedback {
            let i = ids.iter().position(|&id| id == goal).unwrap();
            played.recoveries[i] = fb.number_of_recoveries as i64;
        }
        for r in &out.navigate_results {
            let i = ids.iter().position(|&id| id == r.goal).unwrap();
            played.goals[i] = Some((status_name(r.status), r.error_code, secs));
        }
        // the 50 Hz robot
        if kinematic && ms % 20 == 0 {
            let (s, c) = odom.yaw.sin_cos();
            odom = Pose2D::new(odom.x + twist.x * c * 0.02, odom.y + twist.x * s * 0.02, odom.yaw + twist.theta * 0.02);
        }
    }
    played.final_pose = map_from_odom.apply_pose(&odom);
    played
}

#[test]
fn scenarios_match_the_real_navigator() {
    let spec = load("../oracle/navigator/scenarios.json");
    let data = load("navigator.json");
    let m = &spec["map_from_odom"];
    let map_from_odom = Transform2D::new(f(&m[0]), f(&m[1]), f(&m[2]));
    let mut failures = Vec::new();
    for sc in spec["scenarios"].as_array().unwrap() {
        let name = sc["name"].as_str().unwrap();
        for tree in sc["compare"].as_array().unwrap() {
            let tree = tree.as_str().unwrap();
            let theirs = &data["trees"][tree][name];
            let (gp, lp) = (phase_ms(&theirs["global_updates"], 1.0), phase_ms(&theirs["local_updates"], 0.5));
            let ours = play(sc, &spec["maps"], map_from_odom, gp, lp);
            let their_goals = theirs["goals"].as_array().unwrap();
            assert_eq!(ours.goals.len(), their_goals.len(), "{name}: goal count");
            let mut line = format!("{tree:8} {name:28}");
            for (i, (o, t)) in ours.goals.iter().zip(their_goals).enumerate() {
                let (status, code, at) = o.unwrap_or_else(|| panic!("{name}: goal {i} has no result"));
                let recoveries = ours.recoveries[i];
                let t_rec = t["feedback"]["number_of_recoveries"].as_i64().unwrap_or(-1);
                let (t_status, t_code, t_at) = (t["status"].as_str().unwrap(), t["error_code"].as_u64().unwrap() as u16, f(&t["result_t"]));
                line += &format!(
                    " | {i}: {status} {code} at {at:.3} rec {recoveries} (nav2 {t_status} {t_code} at {t_at:.3} rec {t_rec})"
                );
                if (status, code) != (t_status, t_code) {
                    failures.push(format!("{tree} {name} goal {i}: {status} {code}, nav2 {t_status} {t_code}"));
                }
                // nav2's client drops feedback that beats the goal response
                if t_rec != -1 && recoveries != t_rec {
                    failures.push(format!("{tree} {name} goal {i}: {recoveries} recoveries, nav2 {t_rec}"));
                }
                // In the wall scenario nav2 can take a round twice as long
                // (see RACE_S): its result may come up to three of those later.
                let late_ok = if name == RACE_SCENARIO { 3.0 * RACE_S } else { 0.0 };
                if at - t_at > 0.3 || t_at - at > 0.3 + late_ok {
                    failures.push(format!("{tree} {name} goal {i}: ended at {at:.3} s, nav2 at {t_at:.3} s"));
                }
            }
            let t_plans = theirs["plans"].as_array().unwrap().len();
            let seen = |key: &str| -> Vec<f64> { theirs[key].as_array().unwrap().iter().map(|g| f(&g[0])).collect() };
            let (t_compute, t_follow) = (seen("compute_goals"), seen("follow_goals"));
            let (compute, follow) = (ours.compute_goals.len(), ours.follow_goals.len());
            line += &format!(
                " | plans {} ({t_plans}) compute {compute} ({}) follow {follow} ({})",
                ours.plans,
                t_compute.len(),
                t_follow.len()
            );
            if std::env::var("NAV_ORACLE_DEBUG").is_ok_and(|v| v == name) {
                println!("{tree} {name} compute ours {:?}\n  nav2 {t_compute:?}", ours.compute_goals);
                println!("{tree} {name} follow ours {:?}\n  nav2 {t_follow:?}", ours.follow_goals);
            }
            // In the wall scenario the replans and the FollowPath failures of a
            // round land within a few ms of each other, and nav2's plan takes
            // 10-30 ms from request to result (here one tick): which comes
            // first can flip, so the counts of plans and follow_path goals
            // differ by a few there (see RACE_S). Everywhere else they match.
            let (plan_slack, follow_slack) = if name == RACE_SCENARIO { (3, 7) } else { (0, 1) };
            if compute.abs_diff(t_compute.len()) > plan_slack
                || ours.plans.abs_diff(t_plans) > plan_slack
                || follow.abs_diff(t_follow.len()) > follow_slack
            {
                failures.push(format!(
                    "{tree} {name}: {} plans, {compute} compute_path_to_pose / {follow} follow_path goals; nav2 {t_plans}, {} / {}",
                    ours.plans,
                    t_compute.len(),
                    t_follow.len()
                ));
            }
            let fp = &theirs["final_pose"];
            let d = (ours.final_pose.x - f(&fp[0])).hypot(ours.final_pose.y - f(&fp[1]));
            line += &format!(" | end {:.3}, {:.3} ({d:.3} m off) | phases {gp} {lp} ms", ours.final_pose.x, ours.final_pose.y);
            println!("{line}");
            if d > 0.03 {
                failures.push(format!("{tree} {name}: robot ended {d:.3} m from where nav2 left it"));
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
