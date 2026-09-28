//! Differential test against ros2_controllers / control_toolbox (jazzy).
//!
//! `tests/vectors/diff_drive_oracle.json` is emitted by a harness that links
//! the upstream `diff_drive_controller/src/odometry.cpp` and the upstream
//! `control_toolbox::RateLimiter` (via `SpeedLimiter`), with the per-cycle
//! glue transcribed from `diff_drive_controller.cpp` and the parameters taken
//! from `src/mower_hardware/config/mower_controllers.yaml`. See the README.

use mower_base_core::diff_drive::{
    receive_command, Command, DiffDrive, DiffDriveParams, Received, Twist,
};
use mower_base_core::limiter::RateLimiter;
use mower_base_core::odometry::{Odometry, RollingMeanAccumulator};
use serde_json::Value;

/// Everything here has to agree to at least this much.
const TOL: f64 = 1e-12;

/// A command stamped the moment it is picked up, which is what the oracle's
/// transcription does.
fn fresh(twist: Twist, t: i64) -> Command {
    Command { twist, stamp_ns: t }
}

/// ROS time is a different clock from the control clock; the odometry must
/// carry the ROS stamp and never mix the two.
const ROS_OFFSET_NS: i64 = 1_790_000_000_000_000_000;

fn vectors() -> Value {
    serde_json::from_str(include_str!("vectors/diff_drive_oracle.json"))
        .expect("diff drive oracle vectors are valid JSON")
}

/// The oracle writes non-finite doubles as strings.
fn num(v: &Value) -> f64 {
    match v {
        Value::Number(n) => n.as_f64().unwrap(),
        Value::String(s) => match s.as_str() {
            "nan" => f64::NAN,
            "inf" => f64::INFINITY,
            "-inf" => f64::NEG_INFINITY,
            other => panic!("unexpected number literal {other}"),
        },
        other => panic!("not a number: {other}"),
    }
}

fn close(got: f64, want: f64, what: &str) {
    if want.is_nan() {
        assert!(got.is_nan(), "{what}: got {got}, want NaN");
        return;
    }
    let scale = want.abs().max(1.0);
    assert!(
        (got - want).abs() <= TOL * scale,
        "{what}: got {got:.17e}, want {want:.17e}, diff {:.3e}",
        (got - want).abs()
    );
}

#[test]
fn rolling_mean_matches_rcpputils() {
    let vs = vectors();
    for case in vs["rolling_mean"].as_array().unwrap() {
        let window = case["window"].as_u64().unwrap() as usize;
        let mut acc = RollingMeanAccumulator::new(window);
        let values = case["values"].as_array().unwrap();
        let means = case["means"].as_array().unwrap();
        for (v, m) in values.iter().zip(means) {
            acc.accumulate(num(v));
            close(acc.rolling_mean(), num(m), &format!("rolling mean window {window}"));
        }
    }
}

#[test]
fn speed_limiter_matches_control_toolbox() {
    let vs = vectors();
    let cases = vs["speed_limiter"].as_array().unwrap();
    assert!(cases.len() >= 60);
    for case in cases {
        let p: Vec<f64> = case["params"].as_array().unwrap().iter().map(num).collect();
        // SpeedLimiter's argument order, forwarded to RateLimiter's.
        let lim = RateLimiter::new(p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7])
            .unwrap_or_else(|e| panic!("cfg {} rejected: {e}", case["cfg"]));
        let inp: Vec<f64> = case["in"].as_array().unwrap().iter().map(num).collect();
        let (v_in, v0, v1, dt) = (inp[0], inp[1], inp[2], inp[3]);
        let name = case["cfg"].as_str().unwrap();

        let mut v = v_in;
        let factor = lim.limit(&mut v, v0, v1, dt);
        close(v, num(&case["limit"]["v"]), &format!("{name} limit v"));
        close(factor, num(&case["limit"]["factor"]), &format!("{name} limit factor"));

        let mut v = v_in;
        let factor = lim.limit_value(&mut v);
        close(v, num(&case["limit_value"]["v"]), &format!("{name} limit_value v"));
        close(
            factor,
            num(&case["limit_value"]["factor"]),
            &format!("{name} limit_value factor"),
        );

        let mut v = v_in;
        let factor = lim.limit_first_derivative(&mut v, v0, dt);
        close(
            v,
            num(&case["limit_first_derivative"]["v"]),
            &format!("{name} limit_first_derivative v"),
        );
        close(
            factor,
            num(&case["limit_first_derivative"]["factor"]),
            &format!("{name} limit_first_derivative factor"),
        );

        let mut v = v_in;
        let factor = lim.limit_second_derivative(&mut v, v0, v1, dt);
        close(
            v,
            num(&case["limit_second_derivative"]["v"]),
            &format!("{name} limit_second_derivative v"),
        );
        close(
            factor,
            num(&case["limit_second_derivative"]["factor"]),
            &format!("{name} limit_second_derivative factor"),
        );
    }
}

#[test]
fn odometry_position_feedback_matches_upstream() {
    let vs = vectors();
    let cases = vs["odometry_update"].as_array().unwrap();
    assert!(cases.len() >= 4);
    for case in cases {
        let name = case["name"].as_str().unwrap();
        let window = case["window"].as_u64().unwrap() as usize;
        let mut odom = Odometry::new(window);
        odom.set_wheel_params(
            num(&case["wheel_separation"]),
            num(&case["left_radius"]),
            num(&case["right_radius"]),
        );
        odom.set_velocity_rolling_window_size(window);
        if case["init"].as_bool().unwrap() {
            odom.init(case["t0_ns"].as_i64().unwrap());
        }
        for step in case["steps"].as_array().unwrap() {
            let ok = odom.update(
                num(&step["left_pos"]),
                num(&step["right_pos"]),
                step["t_ns"].as_i64().unwrap(),
            );
            assert_eq!(ok, step["ok"].as_bool().unwrap(), "{name} update() return");
            close(odom.x(), num(&step["x"]), &format!("{name} x"));
            close(odom.y(), num(&step["y"]), &format!("{name} y"));
            close(odom.heading(), num(&step["heading"]), &format!("{name} heading"));
            close(odom.linear(), num(&step["linear"]), &format!("{name} linear"));
            close(odom.angular(), num(&step["angular"]), &format!("{name} angular"));
        }
    }
}

#[test]
fn odometry_velocity_and_open_loop_match_upstream() {
    let vs = vectors();
    for case in vs["odometry_misc"].as_array().unwrap() {
        let name = case["name"].as_str().unwrap();
        let mut odom = Odometry::new(10);
        odom.set_wheel_params(0.40, 0.10, 0.10);
        odom.set_velocity_rolling_window_size(10);
        odom.init(case["t0_ns"].as_i64().unwrap());
        for step in case["steps"].as_array().unwrap() {
            let t = step["t_ns"].as_i64().unwrap();
            match name {
                "update_from_velocity" => {
                    let ok = odom.update_from_velocity(num(&step["left"]), num(&step["right"]), t);
                    assert_eq!(ok, step["ok"].as_bool().unwrap());
                    close(odom.linear(), num(&step["linear"]), &format!("{name} linear"));
                    close(odom.angular(), num(&step["angular"]), &format!("{name} angular"));
                }
                "update_open_loop" => {
                    odom.update_open_loop(num(&step["linear"]), num(&step["angular"]), t);
                    close(odom.linear(), num(&step["out_linear"]), &format!("{name} linear"));
                    close(odom.angular(), num(&step["out_angular"]), &format!("{name} angular"));
                }
                other => panic!("unknown odometry case {other}"),
            }
            close(odom.x(), num(&step["x"]), &format!("{name} x"));
            close(odom.y(), num(&step["y"]), &format!("{name} y"));
            close(odom.heading(), num(&step["heading"]), &format!("{name} heading"));
        }
    }
}

/// The whole controller cycle: cmd_vel timeout -> speed limits -> odometry ->
/// publish gating -> wheel velocity commands, against the same plant the
/// oracle drove (each wheel reaches the previous cycle's commanded velocity).
#[test]
fn controller_cycle_matches_upstream() {
    let vs = vectors();
    let cc = &vs["controller_cycle"];
    let t0 = cc["t0_ns"].as_i64().unwrap();
    let dt_ns = 40_000_000i64;

    let params = DiffDriveParams::mower();
    assert_eq!(params.wheel_separation, num(&cc["wheel_separation"]));
    assert_eq!(params.wheel_radius, num(&cc["wheel_radius"]));
    assert_eq!(params.cmd_vel_timeout, num(&cc["cmd_vel_timeout"]));
    assert_eq!(params.publish_rate, num(&cc["publish_rate"]));
    assert_eq!(
        params.velocity_rolling_window_size as u64,
        cc["velocity_rolling_window_size"].as_u64().unwrap()
    );

    let mut ddc = DiffDrive::new(params, t0).expect("mower limits are valid");
    let mut plant_left_vel = 0.0;
    let mut plant_right_vel = 0.0;
    let mut left_pos = 0.0;
    let mut right_pos = 0.0;
    let mut published = 0;

    let steps = cc["steps"].as_array().unwrap();
    assert_eq!(steps.len(), 200);
    for step in steps {
        let i = step["i"].as_i64().unwrap();
        let t = step["t_ns"].as_i64().unwrap();
        assert_eq!(t, t0 + i * dt_ns);
        // the plant the oracle ran, reproduced exactly
        close(left_pos, num(&step["left_pos"]), &format!("step {i} left_pos"));
        close(right_pos, num(&step["right_pos"]), &format!("step {i} right_pos"));

        let cmd = step["cmd"].as_array().map(|a| fresh(Twist::new(num(&a[0]), num(&a[1])), t));
        ddc.update_reference(t, cmd);
        assert_eq!(
            ddc.command_timed_out(),
            step["timed_out"].as_bool().unwrap(),
            "step {i} timed_out"
        );

        let out = ddc.update_and_write(t, t + ROS_OFFSET_NS, dt_ns as f64 / 1e9, left_pos, right_pos);
        assert_eq!(out.wheel.is_some(), step["wrote"].as_bool().unwrap(), "step {i} wrote");
        close(
            out.linear_command,
            num(&step["linear_command"]),
            &format!("step {i} linear_command"),
        );
        close(
            out.angular_command,
            num(&step["angular_command"]),
            &format!("step {i} angular_command"),
        );

        let (vl, vr) = match out.wheel {
            Some(w) => (w.left, w.right),
            None => (0.0, 0.0),
        };
        close(vl, num(&step["velocity_left"]), &format!("step {i} velocity_left"));
        close(vr, num(&step["velocity_right"]), &format!("step {i} velocity_right"));

        assert_eq!(
            out.odom.is_some(),
            step["published"].as_bool().unwrap(),
            "step {i} publish gate"
        );
        if let Some(o) = &out.odom {
            published += 1;
            close(o.x, num(&step["x"]), &format!("step {i} odom x"));
            close(o.y, num(&step["y"]), &format!("step {i} odom y"));
            close(o.yaw, num(&step["heading"]), &format!("step {i} odom yaw"));
            close(o.qz, num(&step["qz"]), &format!("step {i} odom qz"));
            close(o.qw, num(&step["qw"]), &format!("step {i} odom qw"));
            close(
                o.linear_x,
                num(&step["odom_linear"]),
                &format!("step {i} odom linear"),
            );
            close(
                o.angular_z,
                num(&step["odom_angular"]),
                &format!("step {i} odom angular"),
            );
            assert_eq!(o.stamp_ns, t + ROS_OFFSET_NS);
            assert_eq!(o.frame_id, "odom");
            assert_eq!(o.child_frame_id, "base_link");
            assert!(o.publish_tf);
        }

        left_pos += plant_left_vel * (dt_ns as f64 / 1e9);
        right_pos += plant_right_vel * (dt_ns as f64 / 1e9);
        plant_left_vel = vl;
        plant_right_vel = vr;
    }
    assert!(published > 100, "the 25 Hz publish gate should let most cycles through");
}

/// The covariance diagonals from the yaml land on indices 0, 7, 14, 21, 28, 35.
#[test]
fn covariance_diagonals_are_placed_like_upstream() {
    let params = DiffDriveParams::mower();
    let mut ddc = DiffDrive::new(params.clone(), 0).unwrap();
    ddc.update_reference(0, Some(fresh(Twist::new(0.1, 0.0), 0)));
    // second cycle so the publish gate has elapsed
    ddc.update_and_write(0, 0, 0.04, 0.0, 0.0);
    let out = ddc.update_and_write(100_000_000, 100_000_000, 0.04, 0.0, 0.0);
    let o = out.odom.expect("publish gate open after 100 ms");
    for index in 0..6 {
        assert_eq!(o.pose_covariance[6 * index + index], params.pose_covariance_diagonal[index]);
        assert_eq!(o.twist_covariance[6 * index + index], params.twist_covariance_diagonal[index]);
    }
    assert_eq!(o.pose_covariance.iter().filter(|v| **v != 0.0).count(), 6);
    assert_eq!(o.twist_covariance.iter().filter(|v| **v != 0.0).count(), 6);
}

// ---- the cmd_vel subscription (diff_drive_controller 4.42.1 on_configure) ----

const MS: i64 = 1_000_000;

fn accepted(r: Received) -> (Command, bool) {
    match r {
        Received::Accepted { command, zero_stamp } => (command, zero_stamp),
        other => panic!("expected Accepted, got {other:?}"),
    }
}

/// `now() - header.stamp < cmd_vel_timeout` goes in, anything else is
/// ignored; the accepted stamp keeps its age on the control clock.
#[test]
fn stale_commands_are_ignored_at_the_subscription() {
    let ros_now = ROS_OFFSET_NS;
    let control_now = 5_000 * MS;
    let tw = Twist::new(0.3, 0.0);

    // 100 ms old: accepted, and it is already 100 ms old on the control clock
    let (c, zero) = accepted(receive_command(tw, ros_now - 100 * MS, ros_now, control_now, 0.25));
    assert!(!zero);
    assert_eq!(c.stamp_ns, control_now - 100 * MS);
    assert_eq!(c.twist, tw);

    // exactly the timeout: `<` is strict upstream, so it is ignored
    assert!(matches!(
        receive_command(tw, ros_now - 250 * MS, ros_now, control_now, 0.25),
        Received::Stale { .. }
    ));
    match receive_command(tw, ros_now - 400 * MS, ros_now, control_now, 0.25) {
        Received::Stale { stamp_ns, age_s } => {
            assert_eq!(stamp_ns, ros_now - 400 * MS);
            assert!((age_s - 0.4).abs() < 1e-12);
        }
        other => panic!("a 0.4 s old command must be ignored, got {other:?}"),
    }

    // a zero stamp is "now"
    let (c, zero) = accepted(receive_command(tw, 0, ros_now, control_now, 0.25));
    assert!(zero);
    assert_eq!(c.stamp_ns, control_now);

    // a stamp from the future is accepted as is (negative age), like upstream
    let (c, _) = accepted(receive_command(tw, ros_now + 50 * MS, ros_now, control_now, 0.25));
    assert_eq!(c.stamp_ns, control_now + 50 * MS);

    // cmd_vel_timeout 0 disables the check
    accepted(receive_command(tw, ros_now - 60_000 * MS, ros_now, control_now, 0.0));
}

/// The timeout runs from the message stamp, not from the cycle that picks the
/// message up: a command that was 200 ms old on arrival brakes 50 ms later.
#[test]
fn the_command_ages_from_its_stamp() {
    let params = DiffDriveParams { cmd_vel_timeout: 0.25, ..DiffDriveParams::mower() };
    let mut ddc = DiffDrive::new(params, 0).unwrap();
    let t = 1_000 * MS;
    let (c, _) = accepted(receive_command(Twist::new(0.3, 0.0), t - 200 * MS, t, t, 0.25));
    ddc.update_reference(t, Some(c));
    assert!(!ddc.command_timed_out());
    ddc.update_reference(t + 40 * MS, None);
    assert!(!ddc.command_timed_out(), "240 ms old");
    ddc.update_reference(t + 80 * MS, None);
    assert!(ddc.command_timed_out(), "280 ms old is past 250 ms");
}

/// B6: open loop integrates the command over `now - odometry timestamp`. The
/// activation seeds that timestamp, so the first step after a (re)start
/// moves the pose by one period, not by the whole clock reading.
#[test]
fn open_loop_odometry_starts_at_activation() {
    let params = DiffDriveParams { open_loop: true, ..DiffDriveParams::mower() };
    let t0 = 86_400_000 * MS; // a day of uptime
    let mut ddc = DiffDrive::new(params, t0).unwrap();
    ddc.update_reference(t0 + 40 * MS, Some(fresh(Twist::new(0.5, 0.0), t0 + 40 * MS)));
    ddc.update_and_write(t0 + 40 * MS, 0, 0.04, 0.0, 0.0);
    // 0.8 m/s^2 for 40 ms = 0.032 m/s, integrated over the 40 ms since activation
    let x = ddc.odometry().x();
    assert!((x - 0.032 * 0.04).abs() < 1e-12, "first open-loop step moved {x} m");

    // re-activation (a fault cleared) seeds it again
    ddc.halt();
    let t1 = t0 + 600_000 * MS;
    ddc.activate(t1);
    ddc.update_reference(t1 + 40 * MS, Some(fresh(Twist::new(0.5, 0.0), t1 + 40 * MS)));
    ddc.update_and_write(t1 + 40 * MS, 0, 0.04, 0.0, 0.0);
    let dx = ddc.odometry().x() - x;
    assert!((dx - 0.032 * 0.04).abs() < 1e-12, "first step after re-activation moved {dx} m");
}
