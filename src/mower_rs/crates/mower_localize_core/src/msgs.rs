//! Plain structs standing in for the ROS messages the filter consumes. Field
//! names and covariance layouts match the messages one-for-one so the port can
//! be read against `ros_filter.cpp`.

use crate::tf::{Quaternion, Vector3};

/// 6x6 covariance, row major, as in `geometry_msgs` (`double[36]`).
pub type Mat6 = [[f64; 6]; 6];
/// 3x3 covariance, row major, as in `sensor_msgs/Imu` (`double[9]`).
pub type Mat3 = [[f64; 3]; 3];

pub const ZERO_MAT6: Mat6 = [[0.0; 6]; 6];
pub const ZERO_MAT3: Mat3 = [[0.0; 3]; 3];

#[derive(Clone, Debug, Default)]
pub struct Header {
    pub frame_id: String,
    /// `header.stamp` in nanoseconds.
    pub stamp_ns: i64,
}

#[derive(Clone, Debug)]
pub struct PoseWithCovariance {
    pub position: Vector3,
    pub orientation: Quaternion,
    pub covariance: Mat6,
}

impl Default for PoseWithCovariance {
    fn default() -> Self {
        Self {
            position: Vector3::zero(),
            orientation: Quaternion::identity(),
            covariance: ZERO_MAT6,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct TwistWithCovariance {
    pub linear: Vector3,
    pub angular: Vector3,
    pub covariance: Mat6,
}

/// `nav_msgs/Odometry`.
#[derive(Clone, Debug, Default)]
pub struct Odometry {
    pub header: Header,
    pub child_frame_id: String,
    pub pose: PoseWithCovariance,
    pub twist: TwistWithCovariance,
}

/// `sensor_msgs/Imu`.
#[derive(Clone, Debug)]
pub struct Imu {
    pub header: Header,
    pub orientation: Quaternion,
    pub orientation_covariance: Mat3,
    pub angular_velocity: Vector3,
    pub angular_velocity_covariance: Mat3,
    pub linear_acceleration: Vector3,
    pub linear_acceleration_covariance: Mat3,
}

impl Default for Imu {
    fn default() -> Self {
        Self {
            header: Header::default(),
            orientation: Quaternion::identity(),
            orientation_covariance: ZERO_MAT3,
            angular_velocity: Vector3::zero(),
            angular_velocity_covariance: ZERO_MAT3,
            linear_acceleration: Vector3::zero(),
            linear_acceleration_covariance: ZERO_MAT3,
        }
    }
}

/// `sensor_msgs/NavSatFix` (only the fields navsat_transform reads).
#[derive(Clone, Debug, Default)]
pub struct NavSatFix {
    pub header: Header,
    /// `status.status`; -1 is STATUS_NO_FIX.
    pub status: i8,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    /// Row-major 3x3, as in the message.
    pub position_covariance: Mat3,
}

pub const STATUS_NO_FIX: i8 = -1;
pub const STATUS_FIX: i8 = 0;
pub const STATUS_GBAS_FIX: i8 = 2;

/// Intermediate message used by `preparePose` when a sensor is differential.
#[derive(Clone, Debug, Default)]
pub struct PoseStamped {
    pub header: Header,
    pub pose: PoseWithCovariance,
}

#[derive(Clone, Debug, Default)]
pub struct TwistStamped {
    pub header: Header,
    pub twist: TwistWithCovariance,
}
