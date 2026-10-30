//! robot_state_publisher 3.3.4's output for real_robot.xacro, against this
//! crate's for the same URDF and the same joint states.
//!
//! `tests/rsp_capture.json` is what `tools/rsp_capture.py` recorded in the
//! runtime image: the URDF xacro made, rsp's `/tf_static`, and its `/tf` for a
//! script of `/joint_states` (wall-clock stamps at 25 Hz, then hand-made ones
//! around the 50 ms interval, single joints, the joints production never
//! sends, a fixed and an unknown joint, large angles, two invalid messages,
//! a zero and a future stamp), every double exact.
//!
//! - The same frames in the same order, translations equal to the bit,
//!   quaternion components within 1e-15 (rsp's KDL is compiled with fused
//!   multiply-adds; the worst difference is printed).
//! - The same messages out of the same joint states, with the same stamps.

use std::path::PathBuf;

use mower_static_tf::state::{JointState, Outcome, Throttle};
use mower_static_tf::{fixed_transforms, moving_transforms, urdf, Settings};
use r2r::geometry_msgs::msg::TransformStamped;
use serde_json::Value;

fn capture() -> Value {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/rsp_capture.json");
    serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap()
}

fn f(v: &Value) -> f64 {
    v.as_str().unwrap().parse().unwrap()
}

fn stamp_ns(v: &Value) -> i64 {
    v[0].as_i64().unwrap() * 1_000_000_000 + v[1].as_i64().unwrap()
}

/// The worst quaternion difference seen, and how many components were
/// bit-exact.
#[derive(Default)]
struct Errors {
    worst: f64,
    exact: usize,
    total: usize,
}

fn same(got: &TransformStamped, want: &Value, errors: &mut Errors, what: &str) {
    assert_eq!(got.header.frame_id, want["parent"].as_str().unwrap(), "{what}");
    assert_eq!(got.child_frame_id, want["child"].as_str().unwrap(), "{what}");
    assert_eq!(
        got.header.stamp.sec as i64 * 1_000_000_000 + got.header.stamp.nanosec as i64,
        stamp_ns(&want["stamp"]),
        "{what}: stamp"
    );
    let t = &got.transform.translation;
    let xyz: Vec<f64> = want["xyz"].as_array().unwrap().iter().map(f).collect();
    assert_eq!([t.x, t.y, t.z], [xyz[0], xyz[1], xyz[2]], "{what}: translation");
    let r = &got.transform.rotation;
    let q: Vec<f64> = want["q"].as_array().unwrap().iter().map(f).collect();
    for (a, b) in [r.x, r.y, r.z, r.w].iter().zip(&q) {
        let d = (a - b).abs();
        assert!(d <= 1e-15, "{what}: {:?} vs {q:?}", [r.x, r.y, r.z, r.w]);
        errors.worst = errors.worst.max(d);
        errors.exact += usize::from(a.to_bits() == b.to_bits());
        errors.total += 1;
    }
}

#[test]
fn the_fixed_joints_are_rsp_s_tf_static() {
    let c = capture();
    let robot = urdf::parse(c["urdf"].as_str().unwrap()).unwrap();
    assert!(c["robot_description_matches_urdf"].as_bool().unwrap());
    let messages = c["tf_static"].as_array().unwrap();
    assert_eq!(messages.len(), 1, "rsp publishes /tf_static once");
    let want = messages[0].as_array().unwrap();
    let stamp = stamp_ns(&want[0]["stamp"]);
    let got = fixed_transforms(&robot, &Settings::default(), stamp);
    let pairs: Vec<(&str, &str)> =
        got.transforms.iter().map(|t| (t.header.frame_id.as_str(), t.child_frame_id.as_str())).collect();
    assert_eq!(
        pairs,
        [
            ("base_link", "back_camera_link"),
            ("back_camera_link", "back_camera_rotated_frame"),
            ("base_footprint", "base_link"),
            ("base_link", "depth_camera_link"),
            ("base_link", "gps_link"),
            ("base_link", "imu_link"),
            ("connector", "blade_hub"),
        ]
    );
    assert_eq!(got.transforms.len(), want.len());
    let mut errors = Errors::default();
    for (g, w) in got.transforms.iter().zip(want) {
        same(g, w, &mut errors, &format!("/tf_static {}", g.child_frame_id));
    }
    println!(
        "/tf_static: {} of {} quaternion components bit for bit, worst {:e}",
        errors.exact, errors.total, errors.worst
    );
}

#[test]
fn the_joint_states_give_rsp_s_tf() {
    let c = capture();
    let robot = urdf::parse(c["urdf"].as_str().unwrap()).unwrap();
    let sent = c["joint_states"].as_array().unwrap();
    // rsp's wall clock when each arrived: the probe's monotonic send time
    // on the wall clock of its first wall-stamped message, plus a
    // millisecond of delivery
    let first = sent.iter().find(|s| s["kind"] == "now").unwrap();
    let offset = stamp_ns(&first["stamp"]) - (first["t"].as_f64().unwrap() * 1e9) as i64;
    let mut throttle = Throttle::new(20.0, false);
    let settings = Settings::default();
    let mut got = Vec::new();
    let (mut throttled, mut invalid) = (0, 0);
    for s in sent {
        let names: Vec<String> = s["name"].as_array().unwrap().iter().map(|n| n.as_str().unwrap().to_string()).collect();
        let positions: Vec<f64> = s["position"].as_array().unwrap().iter().map(f).collect();
        let stamp = stamp_ns(&s["stamp"]);
        let now = (s["t"].as_f64().unwrap() * 1e9) as i64 + offset + 1_000_000;
        let js = JointState { stamp_ns: stamp, name: &names, position: &positions };
        match throttle.on_joint_state(&robot, &js, now) {
            Outcome::Publish(joints) => got.push(moving_transforms(&robot, &settings, &joints, stamp)),
            Outcome::Throttled => throttled += 1,
            Outcome::Invalid { .. } => invalid += 1,
        }
    }
    let want = c["tf"].as_array().unwrap();
    assert_eq!((got.len(), throttled, invalid), (want.len(), sent.len() - want.len() - 2, 2));
    let mut errors = Errors::default();
    for (k, (g, w)) in got.iter().zip(want).enumerate() {
        let w = w.as_array().unwrap();
        assert_eq!(g.transforms.len(), w.len(), "/tf #{k}");
        for (gt, wt) in g.transforms.iter().zip(w) {
            same(gt, wt, &mut errors, &format!("/tf #{k} {}", gt.child_frame_id));
        }
    }
    // one of them is empty: a joint state of an unknown joint goes out so
    assert!(got.iter().any(|m| m.transforms.is_empty()));
    println!(
        "/tf: {} messages, {} of {} quaternion components bit for bit, worst {:e}",
        got.len(),
        errors.exact,
        errors.total,
        errors.worst
    );
}
