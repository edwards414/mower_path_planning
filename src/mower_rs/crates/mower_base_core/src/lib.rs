//! ROS-free core of the Rust base driver (Phase B of `docs/ROS_FREE_PLAN.md`).
//!
//! Everything a base driver needs except the transport wrapper, split the same
//! way as the other crates here: pure functions and pure state machines, no
//! clock, no IO, so every rule has a test vector.
//!
//! * [`protocol`] — the STM32 UART frame format, a byte-for-byte port of
//!   `mower_hardware/src/mower_protocol.cpp` plus the resync behaviour the
//!   `serial_port.cpp` / `mower_system.cpp` read path relies on.
//! * [`limiter`] — `control_toolbox::RateLimiter` (Jazzy), which is what
//!   `diff_drive_controller::SpeedLimiter` is a thin wrapper around.
//! * [`odometry`] — `diff_drive_controller/src/odometry.cpp` including the
//!   rolling-mean velocity filter from `rcpputils`.
//! * [`diff_drive`] — one `diff_drive_controller` update cycle: cmd_vel
//!   timeout, speed limits, odometry feed, publish gating, wheel commands.
//! * [`cycle`] — [`cycle::BaseCycle`], the `mower_system.cpp` read/write cycle
//!   expressed as a state machine over `(bytes, now)` and `(cmd_vel, now)`.
//! * [`record`] — the serial record/replay log used to verify the port against
//!   a capture taken from the running `ros2_control` stack.
//!
//! Time is an `i64` monotonic nanosecond count throughout; `rclcpp::Time` is
//! the same thing, and `seconds()` is `ns as f64 / 1e9` in both.

pub mod cycle;
pub mod diff_drive;
pub mod limiter;
pub mod odometry;
pub mod protocol;
pub mod record;

/// Monotonic nanoseconds. Matches `rclcpp::Time::nanoseconds()`.
pub type TimeNs = i64;

/// `rclcpp::Time::seconds()`: the nanosecond count divided by 1e9 in double.
#[inline]
pub fn seconds(ns: TimeNs) -> f64 {
    ns as f64 / 1e9
}

/// 2 * pi. `mower_system.cpp` spells it `6.283185307179586`, which is the
/// same double as `TAU`.
pub const TWO_PI: f64 = std::f64::consts::TAU;
