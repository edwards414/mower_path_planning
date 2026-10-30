//! A command with NaN or infinity anywhere is refused before any other
//! payload rule, and is a stop (E9-07: velocity_command_guard.py's
//! `math.isfinite` rule, which test_controller_configuration.py checks only
//! on the Python node; core.rs's own vectors cover angular z).

use std::time::Instant;

use velocity_command_guard::core::{Action, Command, Guard, Limits};

#[test]
fn nan_or_infinity_in_any_component_is_refused() {
    let limits = Limits::new(0.20, 0.25, 0.05, 0.50, 1.00, false, 0.05).unwrap();
    let now_ns = 1_000_000_000_000;
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        for axis in 0..6 {
            let mut guard = Guard::new(limits.clone());
            let mut command = Command { stamp_sec: 0, stamp_nanosec: 0, session_matches: true, linear: [0.2, 0.0, 0.0], angular: [0.0; 3] };
            if axis < 3 {
                command.linear[axis] = bad;
            } else {
                command.angular[axis - 3] = bad;
            }
            assert_eq!(guard.on_command(&command, now_ns, Instant::now()), Action::Reject("velocity contains NaN or infinity"), "{bad} on axis {axis}");
        }
    }
}
