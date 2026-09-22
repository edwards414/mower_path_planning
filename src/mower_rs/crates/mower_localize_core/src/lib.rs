//! ROS-free port of the parts of `robot_localization` (Jazzy, 3.8.3) that this
//! mower actually runs: the 15-state EKF, the measurement preprocessing for the
//! odometry and IMU inputs, and the `navsat_transform` datum/UTM maths.
//!
//! Pure core: no ROS, no I/O, no clock. Everything takes explicit timestamps in
//! nanoseconds, so the same code runs live, in replay and under test.
//!
//! See `README.md` for what was deliberately *not* ported and why.

pub mod config;
pub mod ekf;
pub mod filter_common;
pub mod measurement;
pub mod msgs;
pub mod navsat;
pub mod prepare;
pub mod tf;
pub mod utm;

pub use ekf::Ekf;
pub use measurement::Measurement;
pub use navsat::{NavSatConfig, NavSatTransformCore};
pub use prepare::{RosFilterCore, SensorConfig, TransformTree};
