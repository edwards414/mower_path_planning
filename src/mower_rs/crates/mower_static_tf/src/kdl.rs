//! The arithmetic robot_state_publisher 3.3.4 does on a joint, operation for
//! operation as urdfdom and orocos KDL 1.5.1 write it, so that the doubles
//! come out as rsp's (tests/golden.rs: within 1e-15 of them; rsp's library
//! is compiled for aarch64 with fused multiply-adds, which this does not
//! guess, so a last bit can differ).
//!
//! - urdfdom: `rpy` to a normalised quaternion (`Rotation::setFromRPY`).
//! - kdl_parser: the joint origin as a frame (`Rotation::Quaternion`), the
//!   joint as KDL builds it (the axis turned into the parent frame and
//!   normalised with `Vector::Norm`), the segment's tip frame
//!   `joint.pose(0).Inverse() * origin`.
//! - KDL: `Segment::pose(q) = joint.pose(q) * f_tip`, `Rotation::Rot2`,
//!   `Rotation::GetQuaternion`.

/// A KDL rotation matrix, `data[3 * row + column]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rotation(pub [f64; 9]);

/// A KDL vector.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vector(pub [f64; 3]);

/// A KDL frame: `M`, then `p`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Frame {
    pub m: Rotation,
    pub p: Vector,
}

/// KDL's `epsilon` (utility.cxx): `Vector::Norm`'s default.
const EPSILON: f64 = 0.000001;

impl Rotation {
    pub const IDENTITY: Rotation = Rotation([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);

    /// `Rotation::Quaternion(x, y, z, w)`.
    pub fn quaternion(x: f64, y: f64, z: f64, w: f64) -> Rotation {
        let (x2, y2, z2, w2) = (x * x, y * y, z * z, w * w);
        Rotation([
            w2 + x2 - y2 - z2,
            2.0 * x * y - 2.0 * w * z,
            2.0 * x * z + 2.0 * w * y,
            2.0 * w * z + 2.0 * x * y,
            w2 - x2 + y2 - z2,
            2.0 * y * z - 2.0 * w * x,
            2.0 * x * z - 2.0 * w * y,
            2.0 * y * z + 2.0 * w * x,
            w2 - x2 - y2 + z2,
        ])
    }

    /// `Rotation::Rot2(rotvec, angle)`: about a normalised axis.
    pub fn rot2(v: Vector, angle: f64) -> Rotation {
        let [v0, v1, v2] = v.0;
        let ct = angle.cos();
        let st = angle.sin();
        let vt = 1.0 - ct;
        let m_vt_0 = vt * v0;
        let m_vt_1 = vt * v1;
        let m_vt_2 = vt * v2;
        let m_st_0 = v0 * st;
        let m_st_1 = v1 * st;
        let m_st_2 = v2 * st;
        let m_vt_0_1 = m_vt_0 * v1;
        let m_vt_0_2 = m_vt_0 * v2;
        let m_vt_1_2 = m_vt_1 * v2;
        Rotation([
            ct + m_vt_0 * v0,
            -m_st_2 + m_vt_0_1,
            m_st_1 + m_vt_0_2,
            m_st_2 + m_vt_0_1,
            ct + m_vt_1 * v1,
            -m_st_0 + m_vt_1_2,
            -m_st_1 + m_vt_0_2,
            m_st_0 + m_vt_1_2,
            ct + m_vt_2 * v2,
        ])
    }

    /// `operator*(Rotation, Rotation)`.
    pub fn mul(&self, r: &Rotation) -> Rotation {
        let (l, r) = (&self.0, &r.0);
        Rotation([
            l[0] * r[0] + l[1] * r[3] + l[2] * r[6],
            l[0] * r[1] + l[1] * r[4] + l[2] * r[7],
            l[0] * r[2] + l[1] * r[5] + l[2] * r[8],
            l[3] * r[0] + l[4] * r[3] + l[5] * r[6],
            l[3] * r[1] + l[4] * r[4] + l[5] * r[7],
            l[3] * r[2] + l[4] * r[5] + l[5] * r[8],
            l[6] * r[0] + l[7] * r[3] + l[8] * r[6],
            l[6] * r[1] + l[7] * r[4] + l[8] * r[7],
            l[6] * r[2] + l[7] * r[5] + l[8] * r[8],
        ])
    }

    /// `Rotation * Vector`.
    pub fn apply(&self, v: Vector) -> Vector {
        let (d, v) = (&self.0, &v.0);
        Vector([
            d[0] * v[0] + d[1] * v[1] + d[2] * v[2],
            d[3] * v[0] + d[4] * v[1] + d[5] * v[2],
            d[6] * v[0] + d[7] * v[1] + d[8] * v[2],
        ])
    }

    /// `Rotation::Inverse()`: the transpose.
    pub fn inverse(&self) -> Rotation {
        let d = &self.0;
        Rotation([d[0], d[3], d[6], d[1], d[4], d[7], d[2], d[5], d[8]])
    }

    /// `Rotation::Inverse(Vector)`: the transpose times `v`.
    pub fn inverse_apply(&self, v: Vector) -> Vector {
        let (d, v) = (&self.0, &v.0);
        Vector([
            d[0] * v[0] + d[3] * v[1] + d[6] * v[2],
            d[1] * v[0] + d[4] * v[1] + d[7] * v[2],
            d[2] * v[0] + d[5] * v[1] + d[8] * v[2],
        ])
    }

    /// `Rotation::GetQuaternion(x, y, z, w)`.
    pub fn get_quaternion(&self) -> [f64; 4] {
        let m = |r: usize, c: usize| self.0[3 * r + c];
        let trace = m(0, 0) + m(1, 1) + m(2, 2);
        let epsilon = 1e-12;
        if trace > epsilon {
            let s = 0.5 / (trace + 1.0).sqrt();
            let w = 0.25 / s;
            let x = (m(2, 1) - m(1, 2)) * s;
            let y = (m(0, 2) - m(2, 0)) * s;
            let z = (m(1, 0) - m(0, 1)) * s;
            [x, y, z, w]
        } else if m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2) {
            let s = 2.0 * (1.0 + m(0, 0) - m(1, 1) - m(2, 2)).sqrt();
            let w = (m(2, 1) - m(1, 2)) / s;
            let x = 0.25 * s;
            let y = (m(0, 1) + m(1, 0)) / s;
            let z = (m(0, 2) + m(2, 0)) / s;
            [x, y, z, w]
        } else if m(1, 1) > m(2, 2) {
            let s = 2.0 * (1.0 + m(1, 1) - m(0, 0) - m(2, 2)).sqrt();
            let w = (m(0, 2) - m(2, 0)) / s;
            let x = (m(0, 1) + m(1, 0)) / s;
            let y = 0.25 * s;
            let z = (m(1, 2) + m(2, 1)) / s;
            [x, y, z, w]
        } else {
            let s = 2.0 * (1.0 + m(2, 2) - m(0, 0) - m(1, 1)).sqrt();
            let w = (m(1, 0) - m(0, 1)) / s;
            let x = (m(0, 2) + m(2, 0)) / s;
            let y = (m(1, 2) + m(2, 1)) / s;
            let z = 0.25 * s;
            [x, y, z, w]
        }
    }
}

impl Vector {
    pub const ZERO: Vector = Vector([0.0, 0.0, 0.0]);

    pub fn add(&self, o: Vector) -> Vector {
        Vector([self.0[0] + o.0[0], self.0[1] + o.0[1], self.0[2] + o.0[2]])
    }

    pub fn neg(&self) -> Vector {
        Vector([-self.0[0], -self.0[1], -self.0[2]])
    }

    pub fn scale(&self, k: f64) -> Vector {
        Vector([self.0[0] * k, self.0[1] * k, self.0[2] * k])
    }

    pub fn div(&self, k: f64) -> Vector {
        Vector([self.0[0] / k, self.0[1] / k, self.0[2] / k])
    }

    /// `Vector::Norm(epsilon)`: overflow-safe, and 0 below epsilon.
    pub fn norm(&self) -> f64 {
        let d = &self.0;
        let sqr = |a: f64| a * a;
        let mut tmp1 = d[0].abs();
        let mut tmp2 = d[1].abs();
        if tmp1 >= tmp2 {
            tmp2 = d[2].abs();
            if tmp1 >= tmp2 {
                if tmp1 <= EPSILON {
                    return 0.0;
                }
                tmp1 * (1.0 + sqr(d[1] / d[0]) + sqr(d[2] / d[0])).sqrt()
            } else {
                tmp2 * (1.0 + sqr(d[0] / d[2]) + sqr(d[1] / d[2])).sqrt()
            }
        } else {
            tmp1 = d[2].abs();
            if tmp2 > tmp1 {
                tmp2 * (1.0 + sqr(d[0] / d[1]) + sqr(d[2] / d[1])).sqrt()
            } else {
                tmp1 * (1.0 + sqr(d[0] / d[2]) + sqr(d[1] / d[2])).sqrt()
            }
        }
    }
}

impl Frame {
    pub const IDENTITY: Frame = Frame { m: Rotation::IDENTITY, p: Vector::ZERO };

    /// `operator*(Frame, Frame)`.
    pub fn mul(&self, r: &Frame) -> Frame {
        Frame { m: self.m.mul(&r.m), p: self.m.apply(r.p).add(self.p) }
    }

    /// `Frame::Inverse()`.
    pub fn inverse(&self) -> Frame {
        Frame { m: self.m.inverse(), p: self.m.inverse_apply(self.p).neg() }
    }
}

/// urdfdom's `Rotation::setFromRPY` and `normalize`: (x, y, z, w).
pub fn quaternion_from_rpy(roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
    let (phi, the, psi) = (roll / 2.0, pitch / 2.0, yaw / 2.0);
    let x = phi.sin() * the.cos() * psi.cos() - phi.cos() * the.sin() * psi.sin();
    let y = phi.cos() * the.sin() * psi.cos() + phi.sin() * the.cos() * psi.sin();
    let z = phi.cos() * the.cos() * psi.sin() - phi.sin() * the.sin() * psi.cos();
    let w = phi.cos() * the.cos() * psi.cos() + phi.sin() * the.sin() * psi.sin();
    let s = (x * x + y * y + z * z + w * w).sqrt();
    if s == 0.0 {
        [0.0, 0.0, 0.0, 1.0]
    } else {
        [x / s, y / s, z / s, w / s]
    }
}

/// kdl_parser's `toKdl(urdf::Pose)`: the origin of a joint as a frame.
pub fn origin(xyz: [f64; 3], rpy: [f64; 3]) -> Frame {
    let [x, y, z, w] = quaternion_from_rpy(rpy[0], rpy[1], rpy[2]);
    Frame { m: Rotation::quaternion(x, y, z, w), p: Vector(xyz) }
}

/// A KDL joint as kdl_parser makes one from a URDF joint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Joint {
    /// `Joint::None`: fixed, and URDF's planar and floating.
    None,
    /// `RotAxis` (revolute, continuous): the axis in the parent frame,
    /// normalised, through `origin`.
    Rot { axis: Vector, origin: Vector },
    /// `TransAxis` (prismatic).
    Trans { axis: Vector, origin: Vector },
}

impl Joint {
    /// `KDL::Joint(name, F.p, F.M * axis, type)`: `axis(_axis / _axis.Norm())`.
    pub fn rot(parent_to_joint: &Frame, urdf_axis: [f64; 3]) -> Joint {
        let a = parent_to_joint.m.apply(Vector(urdf_axis));
        Joint::Rot { axis: a.div(a.norm()), origin: parent_to_joint.p }
    }

    pub fn trans(parent_to_joint: &Frame, urdf_axis: [f64; 3]) -> Joint {
        let a = parent_to_joint.m.apply(Vector(urdf_axis));
        Joint::Trans { axis: a.div(a.norm()), origin: parent_to_joint.p }
    }

    /// `Joint::pose(q)`, with KDL's scale 1 and offset 0.
    pub fn pose(&self, q: f64) -> Frame {
        let q = 1.0 * q + 0.0;
        match *self {
            Joint::None => Frame::IDENTITY,
            Joint::Rot { axis, origin } => Frame { m: Rotation::rot2(axis, q), p: origin },
            Joint::Trans { axis, origin } => Frame { m: Rotation::IDENTITY, p: origin.add(axis.scale(q)) },
        }
    }
}

/// A KDL segment: its joint, and its tip frame as the constructor keeps it,
/// `joint.pose(0).Inverse() * f_tip`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Segment {
    pub joint: Joint,
    pub f_tip: Frame,
}

impl Segment {
    pub fn new(joint: Joint, parent_to_joint: &Frame) -> Segment {
        Segment { joint, f_tip: joint.pose(0.0).inverse().mul(parent_to_joint) }
    }

    /// `Segment::pose(q)`.
    pub fn pose(&self, q: f64) -> Frame {
        self.joint.pose(q).mul(&self.f_tip)
    }
}

/// rsp's `kdlToTransform`: the translation and the (x, y, z, w) rotation.
pub fn to_transform(f: &Frame) -> ([f64; 3], [f64; 4]) {
    (f.p.0, f.m.get_quaternion())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_fixed_joint_is_its_origin_and_minus_zero_comes_out_plus() {
        // pad_joint: xyz="-0 0 -0.018" rpy="0 -0 -0"; rsp publishes 0.0
        let o = origin([-0.0, 0.0, -0.018], [0.0, -0.0, -0.0]);
        let s = Segment::new(Joint::None, &o);
        let (xyz, q) = to_transform(&s.pose(0.0));
        assert_eq!(xyz, [0.0, 0.0, -0.018]);
        assert!(xyz[0].is_sign_positive(), "KDL's identity products turn -0 into +0");
        assert_eq!(q, [0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn a_revolute_joint_turns_about_its_axis_in_the_parent_frame() {
        // a quarter turn about z of a joint whose origin is a quarter turn
        // about z: half a turn in all
        let o = origin([1.0, 2.0, 3.0], [0.0, 0.0, std::f64::consts::FRAC_PI_2]);
        let s = Segment::new(Joint::rot(&o, [0.0, 0.0, 1.0]), &o);
        let (xyz, q) = to_transform(&s.pose(std::f64::consts::FRAC_PI_2));
        assert_eq!(xyz, [1.0, 2.0, 3.0]);
        assert!((q[2].abs() - 1.0).abs() < 1e-15 && q[3].abs() < 1e-15, "{q:?}");
        // at 0 it is the origin
        let (_, q0) = to_transform(&s.pose(0.0));
        let (_, qo) = to_transform(&o);
        for k in 0..4 {
            assert!((q0[k] - qo[k]).abs() < 1e-15);
        }
    }

    #[test]
    fn norm_is_kdl_s() {
        assert_eq!(Vector([3.0, 4.0, 0.0]).norm(), 5.0);
        assert_eq!(Vector([0.0, 0.0, -2.0]).norm(), 2.0);
        assert_eq!(Vector([1e-7, 0.0, 0.0]).norm(), 0.0, "below epsilon");
    }

    #[test]
    fn a_prismatic_joint_slides_along_its_axis() {
        let o = origin([0.0, 0.0, 1.0], [0.0, 0.0, 0.0]);
        let s = Segment::new(Joint::trans(&o, [1.0, 0.0, 0.0]), &o);
        let (xyz, q) = to_transform(&s.pose(0.5));
        assert_eq!(xyz, [0.5, 0.0, 1.0]);
        assert_eq!(q, [0.0, 0.0, 0.0, 1.0]);
    }
}
