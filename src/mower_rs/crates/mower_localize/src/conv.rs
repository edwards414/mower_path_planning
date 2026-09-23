//! ROS messages <-> `mower_localize_core` structs.
//!
//! The core takes plain structs with nanosecond stamps and row-major
//! covariance matrices; r2r hands out the generated message types with
//! `builtin_interfaces::Time` and flat `Vec<f64>` arrays. Nothing here makes
//! decisions, so the whole numeric contract stays in the oracle-tested core.

use mower_localize_core::msgs as cm;
use mower_localize_core::tf::{Quaternion, Transform, Vector3};

use r2r::builtin_interfaces::msg::Time;
use r2r::geometry_msgs::msg::{
    Point, Pose, PoseWithCovariance, Quaternion as RQuaternion, Transform as RTransform,
    TransformStamped, Twist, TwistWithCovariance, Vector3 as RVector3,
};
use r2r::nav_msgs::msg::Odometry as ROdometry;
use r2r::sensor_msgs::msg::{Imu as RImu, NavSatFix as RNavSatFix, NavSatStatus};
use r2r::std_msgs::msg::Header as RHeader;

pub fn to_ns(t: &Time) -> i64 {
    t.sec as i64 * 1_000_000_000 + t.nanosec as i64
}

pub fn from_ns(ns: i64) -> Time {
    Time {
        sec: ns.div_euclid(1_000_000_000) as i32,
        nanosec: ns.rem_euclid(1_000_000_000) as u32,
    }
}

/// Wall clock in nanoseconds; the filters run on the system clock exactly as
/// the C++ nodes do (`use_sim_time` is false everywhere in production, and
/// robot.launch.py refuses any other value).
pub fn now_ns() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as i64)
        .unwrap_or(0)
}

fn mat6(v: &[f64]) -> cm::Mat6 {
    let mut m = cm::ZERO_MAT6;
    for i in 0..6 {
        for j in 0..6 {
            m[i][j] = v.get(i * 6 + j).copied().unwrap_or(0.0);
        }
    }
    m
}

fn flat6(m: &cm::Mat6) -> Vec<f64> {
    let mut v = Vec::with_capacity(36);
    for row in m.iter() {
        v.extend_from_slice(row);
    }
    v
}

fn mat3(v: &[f64]) -> cm::Mat3 {
    let mut m = cm::ZERO_MAT3;
    for i in 0..3 {
        for j in 0..3 {
            m[i][j] = v.get(i * 3 + j).copied().unwrap_or(0.0);
        }
    }
    m
}

fn flat3(m: &cm::Mat3) -> Vec<f64> {
    let mut v = Vec::with_capacity(9);
    for row in m.iter() {
        v.extend_from_slice(row);
    }
    v
}

fn vec3(v: &RVector3) -> Vector3 {
    Vector3::new(v.x, v.y, v.z)
}

fn point(p: &Point) -> Vector3 {
    Vector3::new(p.x, p.y, p.z)
}

fn quat(q: &RQuaternion) -> Quaternion {
    Quaternion::new(q.x, q.y, q.z, q.w)
}

fn r_vec3(v: &Vector3) -> RVector3 {
    RVector3 { x: v.x, y: v.y, z: v.z }
}

fn r_point(v: &Vector3) -> Point {
    Point { x: v.x, y: v.y, z: v.z }
}

fn r_quat(q: &Quaternion) -> RQuaternion {
    RQuaternion { x: q.x, y: q.y, z: q.z, w: q.w }
}

pub fn odometry_in(msg: &ROdometry) -> cm::Odometry {
    cm::Odometry {
        header: cm::Header {
            frame_id: msg.header.frame_id.clone(),
            stamp_ns: to_ns(&msg.header.stamp),
        },
        child_frame_id: msg.child_frame_id.clone(),
        pose: cm::PoseWithCovariance {
            position: point(&msg.pose.pose.position),
            orientation: quat(&msg.pose.pose.orientation),
            covariance: mat6(&msg.pose.covariance),
        },
        twist: cm::TwistWithCovariance {
            linear: vec3(&msg.twist.twist.linear),
            angular: vec3(&msg.twist.twist.angular),
            covariance: mat6(&msg.twist.covariance),
        },
    }
}

pub fn odometry_out(odom: &cm::Odometry) -> ROdometry {
    ROdometry {
        header: RHeader {
            stamp: from_ns(odom.header.stamp_ns),
            frame_id: odom.header.frame_id.clone(),
        },
        child_frame_id: odom.child_frame_id.clone(),
        pose: PoseWithCovariance {
            pose: Pose {
                position: r_point(&odom.pose.position),
                orientation: r_quat(&odom.pose.orientation),
            },
            covariance: flat6(&odom.pose.covariance),
        },
        twist: TwistWithCovariance {
            twist: Twist {
                linear: r_vec3(&odom.twist.linear),
                angular: r_vec3(&odom.twist.angular),
            },
            covariance: flat6(&odom.twist.covariance),
        },
    }
}

pub fn imu_in(msg: &RImu) -> cm::Imu {
    cm::Imu {
        header: cm::Header {
            frame_id: msg.header.frame_id.clone(),
            stamp_ns: to_ns(&msg.header.stamp),
        },
        orientation: quat(&msg.orientation),
        orientation_covariance: mat3(&msg.orientation_covariance),
        angular_velocity: vec3(&msg.angular_velocity),
        angular_velocity_covariance: mat3(&msg.angular_velocity_covariance),
        linear_acceleration: vec3(&msg.linear_acceleration),
        linear_acceleration_covariance: mat3(&msg.linear_acceleration_covariance),
    }
}

pub fn fix_in(msg: &RNavSatFix) -> cm::NavSatFix {
    cm::NavSatFix {
        header: cm::Header {
            frame_id: msg.header.frame_id.clone(),
            stamp_ns: to_ns(&msg.header.stamp),
        },
        status: msg.status.status,
        latitude: msg.latitude,
        longitude: msg.longitude,
        altitude: msg.altitude,
        position_covariance: mat3(&msg.position_covariance),
    }
}

/// `/gps/filtered`. `prepareFilteredGps` fills only the fields below; the
/// service, covariance type and everything else keep the message defaults,
/// as they do upstream.
pub fn fix_out(fix: &cm::NavSatFix) -> RNavSatFix {
    RNavSatFix {
        header: RHeader {
            stamp: from_ns(fix.header.stamp_ns),
            frame_id: fix.header.frame_id.clone(),
        },
        status: NavSatStatus { status: fix.status, service: 0 },
        latitude: fix.latitude,
        longitude: fix.longitude,
        altitude: fix.altitude,
        position_covariance: flat3(&fix.position_covariance),
        position_covariance_type: 0,
    }
}

pub fn transform_in(t: &RTransform) -> Transform {
    Transform::from_parts(&quat(&t.rotation), vec3(&t.translation))
}

pub fn transform_out(t: &Transform) -> RTransform {
    RTransform {
        translation: r_vec3(&t.origin),
        rotation: r_quat(&t.rotation()),
    }
}

pub fn transform_stamped(
    stamp_ns: i64, parent: &str, child: &str, t: &Transform,
) -> TransformStamped {
    TransformStamped {
        header: RHeader { stamp: from_ns(stamp_ns), frame_id: parent.to_string() },
        child_frame_id: child.to_string(),
        transform: transform_out(t),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stamps_round_trip_including_the_zero_and_sub_second_cases() {
        for ns in [0i64, 1, 999_999_999, 1_000_000_000, 1_726_000_000_123_456_789] {
            assert_eq!(to_ns(&from_ns(ns)), ns, "{ns}");
        }
    }

    #[test]
    fn covariances_keep_their_row_major_order() {
        let flat: Vec<f64> = (0..36).map(|i| i as f64).collect();
        let m = mat6(&flat);
        assert_eq!(m[0][5], 5.0);
        assert_eq!(m[5][0], 30.0);
        assert_eq!(flat6(&m), flat);
        let flat3v: Vec<f64> = (0..9).map(|i| i as f64).collect();
        let m3 = mat3(&flat3v);
        assert_eq!(m3[2][0], 6.0);
        assert_eq!(flat3(&m3), flat3v);
    }

    #[test]
    fn a_short_covariance_array_is_zero_filled_instead_of_panicking() {
        // rosbridge and hand-written publishers do send malformed arrays.
        let m = mat6(&[1.0, 2.0]);
        assert_eq!(m[0][0], 1.0);
        assert_eq!(m[5][5], 0.0);
    }
}
