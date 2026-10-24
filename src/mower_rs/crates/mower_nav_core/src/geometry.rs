//! 2D poses, twists and transforms, and the angle helpers nav2 calls
//! (`angles::*`, `tf2::getYaw`, `tf2::Quaternion::setRPY`,
//! `nav2_util::geometry_utils::euclidean_distance`).
//!
//! Everything this robot navigates is planar: `two_d_mode` in the EKFs, z = 0
//! and a pure yaw in every pose. A pose is therefore stored as (x, y, yaw);
//! the IO layer converts quaternions with [`yaw_from_quaternion`] (tf2's exact
//! formula) and back with [`quaternion_from_yaw`] (`setRPY(0, 0, yaw)`).

use std::f64::consts::PI;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Pose2D {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl Pose2D {
    pub const fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self { x, y, yaw }
    }
}

/// `geometry_msgs/Twist` restricted to what a planar base uses:
/// `linear.x`, `linear.y` and `angular.z`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Twist2D {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

impl Twist2D {
    pub const fn new(x: f64, y: f64, theta: f64) -> Self {
        Self { x, y, theta }
    }
}

/// A rigid planar transform `target <- source`: `apply` maps a point or pose
/// expressed in the source frame into the target frame, which is what
/// `tf2::doTransform` does with the transform `lookupTransform(target, source)`
/// returns.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform2D {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl Default for Transform2D {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Transform2D {
    pub const IDENTITY: Transform2D = Transform2D { x: 0.0, y: 0.0, yaw: 0.0 };

    pub const fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self { x, y, yaw }
    }

    pub fn apply_point(&self, x: f64, y: f64) -> (f64, f64) {
        let (s, c) = self.yaw.sin_cos();
        (c * x - s * y + self.x, s * x + c * y + self.y)
    }

    /// The yaw is wrapped to (-pi, pi], as reading it back from the composed
    /// quaternion with `tf2::getYaw` would.
    pub fn apply_pose(&self, p: &Pose2D) -> Pose2D {
        let (x, y) = self.apply_point(p.x, p.y);
        Pose2D { x, y, yaw: normalize_angle(self.yaw + p.yaw) }
    }

    pub fn inverse(&self) -> Transform2D {
        let (s, c) = self.yaw.sin_cos();
        Transform2D { x: -(c * self.x + s * self.y), y: s * self.x - c * self.y, yaw: normalize_angle(-self.yaw) }
    }

    /// `self * other`: first `other`, then `self`.
    pub fn compose(&self, other: &Transform2D) -> Transform2D {
        let (x, y) = self.apply_point(other.x, other.y);
        Transform2D { x, y, yaw: normalize_angle(self.yaw + other.yaw) }
    }
}

/// `angles::normalize_angle_positive`: [0, 2 pi).
pub fn normalize_angle_positive(angle: f64) -> f64 {
    let result = angle % (2.0 * PI);
    if result < 0.0 {
        result + 2.0 * PI
    } else {
        result
    }
}

/// `angles::normalize_angle`: (-pi, pi].
pub fn normalize_angle(angle: f64) -> f64 {
    let result = (angle + PI) % (2.0 * PI);
    if result <= 0.0 {
        result + PI
    } else {
        result - PI
    }
}

/// `angles::shortest_angular_distance`.
pub fn shortest_angular_distance(from: f64, to: f64) -> f64 {
    normalize_angle(to - from)
}

/// `nav2_util::geometry_utils::euclidean_distance` (2D: `std::hypot(dx, dy)`).
pub fn euclidean_distance(ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    (ax - bx).hypot(ay - by)
}

/// `tf2::getYaw` (tf2/impl/utils.hpp), including its gimbal-lock branches.
pub fn yaw_from_quaternion(x: f64, y: f64, z: f64, w: f64) -> f64 {
    let sqx = x * x;
    let sqy = y * y;
    let sqz = z * z;
    let sqw = w * w;
    let sarg = -2.0 * (x * z - w * y) / (sqx + sqy + sqz + sqw);
    if sarg <= -0.99999 {
        -2.0 * y.atan2(x)
    } else if sarg >= 0.99999 {
        2.0 * y.atan2(x)
    } else {
        (2.0 * (x * y + w * z)).atan2(sqw + sqx - sqy - sqz)
    }
}

/// `tf2::Quaternion::setRPY(0, 0, yaw)` as (x, y, z, w); this is also
/// `nav2_util::geometry_utils::orientationAroundZAxis`.
pub fn quaternion_from_yaw(yaw: f64) -> (f64, f64, f64, f64) {
    // setRPY's products with cos(0) = 1 and sin(0) = 0 leave exactly these.
    let (sy, cy) = (yaw * 0.5).sin_cos();
    (0.0, 0.0, sy, cy)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalize_matches_angles_package() {
        assert_eq!(normalize_angle(PI), PI);
        assert_eq!(normalize_angle(-PI), PI);
        assert!((normalize_angle(3.0 * PI / 2.0) + PI / 2.0).abs() < 1e-15);
        assert_eq!(normalize_angle_positive(-0.0), -0.0 + 0.0);
        assert!((shortest_angular_distance(3.0, -3.0) - (2.0 * PI - 6.0)).abs() < 1e-15);
    }

    #[test]
    fn transform_roundtrip() {
        let t = Transform2D::new(1.5, -2.0, 0.7);
        let p = Pose2D::new(0.3, 0.4, -2.9);
        let q = t.inverse().apply_pose(&t.apply_pose(&p));
        assert!((q.x - p.x).abs() < 1e-12 && (q.y - p.y).abs() < 1e-12);
        assert!(shortest_angular_distance(q.yaw, p.yaw).abs() < 1e-12);
        let c = t.compose(&t.inverse());
        assert!(c.x.abs() < 1e-12 && c.y.abs() < 1e-12 && c.yaw.abs() < 1e-12);
    }

    #[test]
    fn yaw_quaternion_roundtrip() {
        for &yaw in &[0.0, 0.5, -1.2, 3.0, -3.1, PI] {
            let (x, y, z, w) = quaternion_from_yaw(yaw);
            assert!(shortest_angular_distance(yaw_from_quaternion(x, y, z, w), yaw).abs() < 1e-15);
        }
    }
}
