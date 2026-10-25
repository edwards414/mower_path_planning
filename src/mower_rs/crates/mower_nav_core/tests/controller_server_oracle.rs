//! `controller_server` against the real nav2 controller_server, black box:
//! tests/oracle/controller_server/scenarios.json played by scenarios.py on the
//! real node (tests/data/controller_server.json, wall clock) and here on the
//! port (simulated clock), with the same kinematic robot, map and events.
//!
//! The real node runs on its own threads, so its times carry a few ms of
//! scheduling: every goal's status and error code must match exactly, its
//! result time to within a cycle or two, and where the robot ended to within
//! a few centimetres.

mod common;

use common::{f, load};
use mower_nav_core::controller_server::{ControllerServer, CycleInput, GoalStatus};
use mower_nav_core::costmap::{static_costmap, OccupancyGrid, RollingWindow, StaticLayerParams};
use mower_nav_core::geometry::{Pose2D, Transform2D, Twist2D};
use serde_json::Value;

const MS: i64 = 1_000_000;
/// The real node's ticks are anchored to when it took the goal, a few ms
/// after the event; preemptions and cancels land on its next tick.
const TIME_TOL: f64 = 0.12;

fn status_name(s: GoalStatus) -> &'static str {
    match s {
        GoalStatus::Succeeded => "succeeded",
        GoalStatus::Aborted => "aborted",
        GoalStatus::Canceled => "canceled",
    }
}

fn path(ev: &Value) -> Vec<Pose2D> {
    if ev["length"].is_null() {
        return Vec::new();
    }
    let (x0, y0) = (f(&ev["from"][0]), f(&ev["from"][1]));
    let n = (f(&ev["length"]) / 0.1).round() as usize + 1;
    (0..n).map(|i| Pose2D::new(x0 + 0.1 * i as f64, y0, 0.0)).collect()
}

fn map_costmap(spec: &Value) -> mower_nav_core::costmap::Costmap2D {
    let (w, h) = (spec["width"].as_u64().unwrap() as u32, spec["height"].as_u64().unwrap() as u32);
    let res = f(&spec["resolution"]);
    let (ox, oy) = (f(&spec["origin"][0]), f(&spec["origin"][1]));
    let mut data = vec![0i8; (w * h) as usize];
    if let Some(band) = spec["lethal_x"].as_array() {
        let (x0, x1) = (f(&band[0]), f(&band[1]));
        for j in 0..h {
            for i in 0..w {
                let x = ox + (i as f64 + 0.5) * res;
                if x0 <= x && x < x1 {
                    data[(j * w + i) as usize] = 100;
                }
            }
        }
    }
    // MapMetaData.resolution is a float32
    let grid = OccupancyGrid { width: w, height: h, resolution: res as f32 as f64, origin_x: ox, origin_y: oy, data };
    static_costmap(&grid, &StaticLayerParams::new(false))
}

struct Played {
    /// (status, error code, result time) per goal, in send order
    goals: Vec<Option<(&'static str, u16, f64)>>,
    final_pose: Pose2D,
}

fn play(sc: &Value, maps: &Value, map_from_odom: Transform2D) -> Played {
    let map = map_costmap(&maps[sc["map"].as_str().unwrap()]);
    let kinematic = sc["robot"] == "kinematic";
    let start = Pose2D::new(f(&sc["start"][0]), f(&sc["start"][1]), f(&sc["start"][2]));
    let mut pose = map_from_odom.inverse().apply_pose(&start);
    let mut twist = Twist2D::default();
    let mut window = RollingWindow::local_costmap();
    window.update(pose.x, pose.y, &map, &map_from_odom);
    let mut server = ControllerServer::mower();
    let mut events: Vec<&Value> = sc["events"].as_array().unwrap().iter().collect();
    events.sort_by(|a, b| f(&a["t"]).partial_cmp(&f(&b["t"])).unwrap());
    let mut ids = Vec::new();
    let mut goals: Vec<Option<(&'static str, u16, f64)>> = Vec::new();
    let duration_ms = (f(&sc["duration"]) * 1000.0).round() as i64;
    let mut next_tick: Option<i64> = None;
    let mut next_event = 0;
    let mut cancelled_rest = false;
    let t0 = 1_792_900_000_000_000_000i64;
    for ms in 0..duration_ms + 5000 {
        let t = ms * MS;
        let secs = ms as f64 / 1000.0;
        // the scenario's events (scenarios.py: then cancel what still runs)
        while next_event < events.len() && (f(&events[next_event]["t"]) * 1000.0).round() as i64 == ms {
            let ev = events[next_event];
            next_event += 1;
            if ev["op"] == "send" {
                let idle = !server.is_executing();
                ids.push(server.send_goal(path(ev)));
                goals.push(None);
                if idle {
                    next_tick = Some(t); // the worker thread starts at once
                }
            } else {
                server.cancel_goal(ids[ev["goal"].as_u64().unwrap() as usize]);
            }
        }
        if ms == duration_ms && !cancelled_rest {
            cancelled_rest = true;
            for (i, g) in goals.iter().enumerate() {
                if g.is_none() {
                    server.cancel_goal(ids[i]);
                }
            }
        }
        if ms > duration_ms && goals.iter().all(Option::is_some) {
            break;
        }
        // the 2 Hz local costmap
        if ms % 500 == 0 {
            window.update(pose.x, pose.y, &map, &map_from_odom);
        }
        if next_tick.is_some_and(|n| n <= t) {
            let input = CycleInput {
                now_ns: t0 + t,
                costmap_current: true,
                robot_pose: Some(pose),
                odom_twist: if kinematic { twist } else { Twist2D::default() },
                odom_from_map: Some(map_from_odom.inverse()),
                costmap: window.costmap(),
            };
            let out = server.tick(&input);
            if let Some(c) = out.commands.last() {
                twist = c.twist;
            }
            for r in &out.results {
                let i = ids.iter().position(|&id| id == r.goal).unwrap();
                goals[i] = Some((status_name(r.status), r.error_code, secs));
            }
            next_tick = server.is_executing().then(|| t + 50 * MS);
        }
        // the 50 Hz robot
        if kinematic && ms % 20 == 0 {
            let (s, c) = pose.yaw.sin_cos();
            pose = Pose2D::new(pose.x + twist.x * c * 0.02, pose.y + twist.x * s * 0.02, pose.yaw + twist.theta * 0.02);
        }
    }
    Played { goals, final_pose: map_from_odom.apply_pose(&pose) }
}

#[test]
fn scenarios_match_the_real_controller_server() {
    let spec = load("../oracle/controller_server/scenarios.json");
    let data = load("controller_server.json");
    let m = &spec["map_from_odom"];
    let map_from_odom = Transform2D::new(f(&m[0]), f(&m[1]), f(&m[2]));
    let mut failures = Vec::new();
    for sc in spec["scenarios"].as_array().unwrap() {
        let name = sc["name"].as_str().unwrap();
        let theirs = &data["scenarios"][name];
        let ours = play(sc, &spec["maps"], map_from_odom);
        let their_goals = theirs["goals"].as_array().unwrap();
        assert_eq!(ours.goals.len(), their_goals.len(), "{name}: goal count");
        let mut line = format!("{name:30}");
        for (i, (o, t)) in ours.goals.iter().zip(their_goals).enumerate() {
            let (status, code, at) = o.unwrap_or_else(|| panic!("{name}: goal {i} has no result"));
            let (t_status, t_code, t_at) = (t["status"].as_str().unwrap(), t["error_code"].as_u64().unwrap() as u16, f(&t["result_t"]));
            line += &format!(" | {i}: {status} {code} at {at:.3} (nav2 {t_status} {t_code} at {t_at:.3})");
            if (status, code) != (t_status, t_code) {
                failures.push(format!("{name} goal {i}: {status} {code}, nav2 {t_status} {t_code}"));
            }
            // the approach is slower for nav2 by the few ms each tick of its
            // starts late; over 150 ticks of the straight run that adds up
            let tol = if name == "reach_straight" { 0.3 } else { TIME_TOL };
            if (at - t_at).abs() > tol {
                failures.push(format!("{name} goal {i}: ended at {at:.3} s, nav2 at {t_at:.3} s"));
            }
        }
        let fp = &theirs["final_pose"];
        let d = (ours.final_pose.x - f(&fp[0])).hypot(ours.final_pose.y - f(&fp[1]));
        line += &format!(" | end {:.3}, {:.3} ({d:.3} m from nav2's)", ours.final_pose.x, ours.final_pose.y);
        println!("{line}");
        if d > 0.03 {
            failures.push(format!("{name}: robot ended {d:.3} m from where nav2 left it"));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
