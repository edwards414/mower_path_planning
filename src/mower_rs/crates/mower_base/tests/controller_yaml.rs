//! mower_base's built-in drivetrain parameters are the production
//! diff_controller yaml that ros2_control launches, not mower_hardware's
//! bench file: the two differ in open_loop, enable_odom_tf, the base frame
//! and the wheel geometry (E9-07, from test_controller_configuration.py).

use mower_base::production_diff_drive;

const CONTROLLER: &str = include_str!("../../../../mower_controller/controllers/diff_drive_controller.yaml");

/// The value of the first `key:` line, as test_controller_configuration.py's
/// `_scalar` reads it.
fn scalar(key: &str) -> &'static str {
    CONTROLLER
        .lines()
        .find_map(|line| {
            let value = line.trim_start().strip_prefix(key)?.strip_prefix(':')?;
            value.split(|c: char| c.is_whitespace() || c == '#').find(|w| !w.is_empty())
        })
        .unwrap_or_else(|| panic!("{key} missing from diff_drive_controller.yaml"))
}

fn number(key: &str) -> f64 {
    scalar(key).parse().unwrap_or_else(|e| panic!("{key}: {e}"))
}

#[test]
fn the_built_in_defaults_are_the_launched_diff_controller_yaml() {
    let p = production_diff_drive();
    for (key, value) in [
        ("wheel_separation", p.wheel_separation),
        ("wheel_radius", p.wheel_radius),
        ("publish_rate", p.publish_rate),
        ("cmd_vel_timeout", p.cmd_vel_timeout),
    ] {
        assert_eq!(value, number(key), "{key}");
    }
    assert_eq!(p.base_frame_id, scalar("base_frame_id"));
    assert_eq!(p.odom_frame_id, scalar("odom_frame_id"));
    // open loop, and the two EKFs own odom -> base_footprint
    assert!(p.open_loop && scalar("open_loop") == "true");
    assert!(!p.enable_odom_tf && scalar("enable_odom_tf") == "false");
    for (axis, limits) in [("linear.x", &p.linear), ("angular.z", &p.angular)] {
        for (key, value) in [
            ("max_velocity", limits.max_velocity),
            ("min_velocity", limits.min_velocity),
            ("max_acceleration", limits.max_acceleration),
            ("max_acceleration_reverse", limits.max_acceleration_reverse),
            ("max_deceleration", limits.max_deceleration),
            ("max_deceleration_reverse", limits.max_deceleration_reverse),
        ] {
            assert_eq!(value, number(&format!("{axis}.{key}")), "{axis}.{key}");
        }
    }
}
