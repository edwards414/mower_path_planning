//! The configuration `tests/oracle/oracle.cpp` runs the real
//! robot_localization with, so the vectors in `tests/data` are compared
//! against a filter set up identically.
//!
//! It is the production `dual_ekf_navsat_params.yaml` as of the oracle run,
//! written out by hand because the oracle hard-codes it too; `yaml_config.rs`
//! fails if the yaml and this drift apart, which means the vectors no longer
//! cover what the robot runs and `run_oracle.sh` has to be re-run.
#![allow(dead_code)]

use mower_localize_core::filter_common::{Mat15, STATE_SIZE, ZERO_MAT15};
use mower_localize_core::navsat::NavSatConfig;
use mower_localize_core::prepare::{RosFilterCore, SensorConfig};

/// `ekf_filter_node_odom` / `ekf_filter_node_map`: everything that is not a
/// sensor config.
#[derive(Clone, Debug)]
pub struct EkfConfig {
    pub frequency: f64,
    pub two_d_mode: bool,
    pub publish_tf: bool,
    pub map_frame: &'static str,
    pub odom_frame: &'static str,
    pub base_link_frame: &'static str,
    pub world_frame: &'static str,
    pub use_control: bool,
    pub process_noise_covariance: Mat15,
}

fn diag(values: [f64; STATE_SIZE]) -> Mat15 {
    let mut m = ZERO_MAT15;
    for (i, v) in values.into_iter().enumerate() {
        m[i][i] = v;
    }
    m
}

/// `ekf_filter_node_odom`.
pub fn ekf_odom_config() -> EkfConfig {
    EkfConfig {
        frequency: 20.0,
        two_d_mode: true,
        publish_tf: true,
        map_frame: "map",
        odom_frame: "odom",
        base_link_frame: "base_footprint",
        world_frame: "odom",
        use_control: false,
        process_noise_covariance: diag([
            1e-3, 1e-3, 1e-3, 0.3, 0.3, 0.01, 0.5, 0.5, 0.1, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3,
        ]),
    }
}

/// `ekf_filter_node_map`.
pub fn ekf_map_config() -> EkfConfig {
    EkfConfig {
        frequency: 20.0,
        two_d_mode: true,
        publish_tf: true,
        map_frame: "map",
        odom_frame: "odom",
        base_link_frame: "base_footprint",
        world_frame: "map",
        use_control: false,
        process_noise_covariance: diag([
            1.0, 1.0, 1e-3, 0.3, 0.3, 0.01, 0.5, 0.5, 0.1, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3,
        ]),
    }
}

/// `navsat_transform`.
pub fn navsat_config() -> NavSatConfig {
    NavSatConfig {
        magnetic_declination_radians: 0.0,
        yaw_offset: 0.0,
        zero_altitude: true,
        publish_filtered_gps: true,
        use_odometry_yaw: false,
        wait_for_datum: false,
        broadcast_cartesian_transform: true,
        broadcast_cartesian_transform_as_parent_frame: false,
    }
}

const T: bool = true;
const F: bool = false;

/// `ekf_filter_node_odom`'s `odom0` (topic `odom`): vx, vy, vyaw.
pub fn odom_ekf_odom0() -> SensorConfig {
    SensorConfig::new("odom", [F, F, F, F, F, F, T, T, F, F, F, T, F, F, F])
}

/// `ekf_filter_node_odom`'s `imu0` (topic `imu`): yaw, yaw rate, ax, ay.
pub fn odom_ekf_imu0() -> SensorConfig {
    let mut c = SensorConfig::new("imu", [F, F, F, F, F, T, F, F, F, F, F, T, T, T, F]);
    c.remove_gravitational_acceleration = true;
    c
}

/// `ekf_filter_node_map`'s `odom0` (topic `odom`): vx, vy, vz, vyaw.
pub fn map_ekf_odom0() -> SensorConfig {
    SensorConfig::new("odom", [F, F, F, F, F, F, T, T, T, F, F, T, F, F, F])
}

/// `ekf_filter_node_map`'s `odom1` (topic `odometry/gps`): x, y.
pub fn map_ekf_odom1() -> SensorConfig {
    SensorConfig::new(
        "odometry/gps",
        [T, T, F, F, F, F, F, F, F, F, F, F, F, F, F],
    )
}

/// `ekf_filter_node_map`'s `imu0` (topic `imu`): yaw only.
pub fn map_ekf_imu0() -> SensorConfig {
    let mut c = SensorConfig::new("imu", [F, F, F, F, F, T, F, F, F, F, F, F, F, F, F]);
    c.remove_gravitational_acceleration = true;
    c
}

/// Build a filter wrapper already configured from an [`EkfConfig`].
pub fn build_filter(config: &EkfConfig) -> RosFilterCore {
    let mut core = RosFilterCore::new(config.world_frame, config.base_link_frame);
    core.two_d_mode = config.two_d_mode;
    core.map_frame_id = config.map_frame.to_string();
    core.odom_frame_id = config.odom_frame.to_string();
    core.filter.use_control = config.use_control;
    core.filter
        .set_process_noise_covariance(config.process_noise_covariance);
    // frequency 20 Hz -> the sensor timeout robot_localization derives when
    // `sensor_timeout` is not given is 1/frequency.
    core.filter.sensor_timeout_ns = (1e9 / config.frequency) as i64;
    core
}
