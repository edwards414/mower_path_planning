//! The slice of `tf2::LinearMath` that robot_localization's maths depends on,
//! transcribed from `tf2/LinearMath/{Vector3,Quaternion,Matrix3x3,Transform}.hpp`
//! (Jazzy). Operation order is kept identical to the C++ so the results are
//! bit-comparable, which is what the oracle tests check.

/// `tf2::Vector3` (the fourth `m_floats` slot is padding in tf2 and unused here).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3 {
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub const fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    pub fn dot(&self, o: &Vector3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn length2(&self) -> f64 {
        self.dot(self)
    }

    pub fn length(&self) -> f64 {
        self.length2().sqrt()
    }

    pub fn cross(&self, o: &Vector3) -> Vector3 {
        Vector3::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }

    pub fn add(&self, o: &Vector3) -> Vector3 {
        Vector3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }

    pub fn sub(&self, o: &Vector3) -> Vector3 {
        Vector3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }

    pub fn neg(&self) -> Vector3 {
        Vector3::new(-self.x, -self.y, -self.z)
    }

    pub fn scale(&self, s: f64) -> Vector3 {
        Vector3::new(self.x * s, self.y * s, self.z * s)
    }
}

/// `tf2::Quaternion`, stored (x, y, z, w) like tf2 does.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self::identity()
    }
}

impl Quaternion {
    pub const fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        Self { x, y, z, w }
    }

    pub const fn identity() -> Self {
        Self::new(0.0, 0.0, 0.0, 1.0)
    }

    /// `Quaternion::setRPY`.
    pub fn from_rpy(roll: f64, pitch: f64, yaw: f64) -> Self {
        let half_yaw = yaw * 0.5;
        let half_pitch = pitch * 0.5;
        let half_roll = roll * 0.5;
        let (sy, cy) = (half_yaw.sin(), half_yaw.cos());
        let (sp, cp) = (half_pitch.sin(), half_pitch.cos());
        let (sr, cr) = (half_roll.sin(), half_roll.cos());
        Self::new(
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
    }

    pub fn length2(&self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w
    }

    pub fn length(&self) -> f64 {
        self.length2().sqrt()
    }

    pub fn normalized(&self) -> Quaternion {
        let l = self.length();
        Quaternion::new(self.x / l, self.y / l, self.z / l, self.w / l)
    }

    /// tf2's `inverse()`: the conjugate (tf2 does not divide by the norm).
    pub fn inverse(&self) -> Quaternion {
        Quaternion::new(-self.x, -self.y, -self.z, self.w)
    }

    /// `operator*(Quaternion, Quaternion)`.
    pub fn mul(&self, o: &Quaternion) -> Quaternion {
        let (q1, q2) = (self, o);
        Quaternion::new(
            q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
            q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z,
            q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x,
            q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
        )
    }

    /// `operator*(Quaternion, Vector3)`.
    pub fn mul_vec(&self, w: &Vector3) -> Quaternion {
        let q = self;
        Quaternion::new(
            q.w * w.x + q.y * w.z - q.z * w.y,
            q.w * w.y + q.z * w.x - q.x * w.z,
            q.w * w.z + q.x * w.y - q.y * w.x,
            -q.x * w.x - q.y * w.y - q.z * w.z,
        )
    }
}

/// `tf2::quatRotate`.
pub fn quat_rotate(rotation: &Quaternion, v: &Vector3) -> Vector3 {
    let q = rotation.mul_vec(v).mul(&rotation.inverse());
    Vector3::new(q.x, q.y, q.z)
}

/// `tf2::Matrix3x3`, stored as three row vectors like tf2's `m_el`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Matrix3x3 {
    pub el: [Vector3; 3],
}

impl Default for Matrix3x3 {
    fn default() -> Self {
        Self::identity()
    }
}

impl Matrix3x3 {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        xx: f64,
        xy: f64,
        xz: f64,
        yx: f64,
        yy: f64,
        yz: f64,
        zx: f64,
        zy: f64,
        zz: f64,
    ) -> Self {
        Self {
            el: [
                Vector3::new(xx, xy, xz),
                Vector3::new(yx, yy, yz),
                Vector3::new(zx, zy, zz),
            ],
        }
    }

    pub const fn identity() -> Self {
        Self::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    }

    pub fn row(&self, i: usize) -> Vector3 {
        self.el[i]
    }

    /// `setRotation(q)`.
    pub fn from_quaternion(q: &Quaternion) -> Self {
        let d = q.length2();
        let s = 2.0 / d;
        let (xs, ys, zs) = (q.x * s, q.y * s, q.z * s);
        let (wx, wy, wz) = (q.w * xs, q.w * ys, q.w * zs);
        let (xx, xy, xz) = (q.x * xs, q.x * ys, q.x * zs);
        let (yy, yz, zz) = (q.y * ys, q.y * zs, q.z * zs);
        Self::new(
            1.0 - (yy + zz),
            xy - wz,
            xz + wy,
            xy + wz,
            1.0 - (xx + zz),
            yz - wx,
            xz - wy,
            yz + wx,
            1.0 - (xx + yy),
        )
    }

    /// `setEulerYPR(yaw, pitch, roll)`, i.e. `setRPY(roll, pitch, yaw)`.
    pub fn from_rpy(roll: f64, pitch: f64, yaw: f64) -> Self {
        let (ci, si) = (roll.cos(), roll.sin());
        let (cj, sj) = (pitch.cos(), pitch.sin());
        let (ch, sh) = (yaw.cos(), yaw.sin());
        let cc = ci * ch;
        let cs = ci * sh;
        let sc = si * ch;
        let ss = si * sh;
        Self::new(
            cj * ch,
            sj * sc - cs,
            sj * cc + ss,
            cj * sh,
            sj * ss + cc,
            sj * cs - sc,
            -sj,
            cj * si,
            cj * ci,
        )
    }

    /// `getRPY(roll, pitch, yaw)` with tf2's solution number 1.
    pub fn get_rpy(&self) -> (f64, f64, f64) {
        let (roll, pitch, yaw);
        if self.el[2].x.abs() >= 1.0 {
            yaw = 0.0;
            let delta = self.el[2].y.atan2(self.el[2].z);
            if self.el[2].x < 0.0 {
                pitch = std::f64::consts::PI / 2.0;
            } else {
                pitch = -std::f64::consts::PI / 2.0;
            }
            roll = delta;
        } else {
            pitch = -self.el[2].x.asin();
            roll = (self.el[2].y / pitch.cos()).atan2(self.el[2].z / pitch.cos());
            yaw = (self.el[1].x / pitch.cos()).atan2(self.el[0].x / pitch.cos());
        }
        (roll, pitch, yaw)
    }

    /// `getRotation(q)`: the basis back to a quaternion.
    pub fn get_rotation(&self) -> Quaternion {
        let m = &self.el;
        let trace = m[0].x + m[1].y + m[2].z;
        let mut temp = [0.0f64; 4];
        if trace > 0.0 {
            let mut s = (trace + 1.0).sqrt();
            temp[3] = s * 0.5;
            s = 0.5 / s;
            temp[0] = (m[2].y - m[1].z) * s;
            temp[1] = (m[0].z - m[2].x) * s;
            temp[2] = (m[1].x - m[0].y) * s;
        } else {
            let i = if m[0].x < m[1].y {
                if m[1].y < m[2].z {
                    2
                } else {
                    1
                }
            } else if m[0].x < m[2].z {
                2
            } else {
                0
            };
            let j = (i + 1) % 3;
            let k = (i + 2) % 3;
            let at = |r: usize, c: usize| -> f64 {
                match c {
                    0 => m[r].x,
                    1 => m[r].y,
                    _ => m[r].z,
                }
            };
            let mut s = (at(i, i) - at(j, j) - at(k, k) + 1.0).sqrt();
            temp[i] = s * 0.5;
            s = 0.5 / s;
            temp[3] = (at(k, j) - at(j, k)) * s;
            temp[j] = (at(j, i) + at(i, j)) * s;
            temp[k] = (at(k, i) + at(i, k)) * s;
        }
        Quaternion::new(temp[0], temp[1], temp[2], temp[3])
    }

    fn tdotx(&self, v: &Vector3) -> f64 {
        self.el[0].x * v.x + self.el[1].x * v.y + self.el[2].x * v.z
    }

    fn tdoty(&self, v: &Vector3) -> f64 {
        self.el[0].y * v.x + self.el[1].y * v.y + self.el[2].y * v.z
    }

    fn tdotz(&self, v: &Vector3) -> f64 {
        self.el[0].z * v.x + self.el[1].z * v.y + self.el[2].z * v.z
    }

    /// `operator*(Matrix3x3, Vector3)`.
    pub fn mul_vec(&self, v: &Vector3) -> Vector3 {
        Vector3::new(self.el[0].dot(v), self.el[1].dot(v), self.el[2].dot(v))
    }

    /// `operator*(Vector3, Matrix3x3)`, i.e. `v^T * M`.
    pub fn vec_mul(&self, v: &Vector3) -> Vector3 {
        Vector3::new(self.tdotx(v), self.tdoty(v), self.tdotz(v))
    }

    /// `operator*(Matrix3x3, Matrix3x3)`.
    pub fn mul(&self, m2: &Matrix3x3) -> Matrix3x3 {
        Matrix3x3::new(
            m2.tdotx(&self.el[0]),
            m2.tdoty(&self.el[0]),
            m2.tdotz(&self.el[0]),
            m2.tdotx(&self.el[1]),
            m2.tdoty(&self.el[1]),
            m2.tdotz(&self.el[1]),
            m2.tdotx(&self.el[2]),
            m2.tdoty(&self.el[2]),
            m2.tdotz(&self.el[2]),
        )
    }

    fn at(&self, r: usize, c: usize) -> f64 {
        match c {
            0 => self.el[r].x,
            1 => self.el[r].y,
            _ => self.el[r].z,
        }
    }

    /// `cofac(r1, c1, r2, c2)`.
    fn cofac(&self, r1: usize, c1: usize, r2: usize, c2: usize) -> f64 {
        self.at(r1, c1) * self.at(r2, c2) - self.at(r1, c2) * self.at(r2, c1)
    }

    /// `Matrix3x3::inverse()`: the adjugate over the determinant, exactly as
    /// tf2 computes it (this is *not* the transpose, even for a rotation).
    pub fn inverse(&self) -> Matrix3x3 {
        let co = Vector3::new(
            self.cofac(1, 1, 2, 2),
            self.cofac(1, 2, 2, 0),
            self.cofac(1, 0, 2, 1),
        );
        let det = self.el[0].dot(&co);
        let s = 1.0 / det;
        Matrix3x3::new(
            co.x * s,
            self.cofac(0, 2, 2, 1) * s,
            self.cofac(0, 1, 1, 2) * s,
            co.y * s,
            self.cofac(0, 0, 2, 2) * s,
            self.cofac(0, 2, 1, 0) * s,
            co.z * s,
            self.cofac(0, 1, 2, 0) * s,
            self.cofac(0, 0, 1, 1) * s,
        )
    }

    pub fn transpose(&self) -> Matrix3x3 {
        Matrix3x3::new(
            self.el[0].x,
            self.el[1].x,
            self.el[2].x,
            self.el[0].y,
            self.el[1].y,
            self.el[2].y,
            self.el[0].z,
            self.el[1].z,
            self.el[2].z,
        )
    }

    /// `transposeTimes(m)`.
    pub fn transpose_times(&self, m: &Matrix3x3) -> Matrix3x3 {
        let a = &self.el;
        let b = &m.el;
        Matrix3x3::new(
            a[0].x * b[0].x + a[1].x * b[1].x + a[2].x * b[2].x,
            a[0].x * b[0].y + a[1].x * b[1].y + a[2].x * b[2].y,
            a[0].x * b[0].z + a[1].x * b[1].z + a[2].x * b[2].z,
            a[0].y * b[0].x + a[1].y * b[1].x + a[2].y * b[2].x,
            a[0].y * b[0].y + a[1].y * b[1].y + a[2].y * b[2].y,
            a[0].y * b[0].z + a[1].y * b[1].z + a[2].y * b[2].z,
            a[0].z * b[0].x + a[1].z * b[1].x + a[2].z * b[2].x,
            a[0].z * b[0].y + a[1].z * b[1].y + a[2].z * b[2].y,
            a[0].z * b[0].z + a[1].z * b[1].z + a[2].z * b[2].z,
        )
    }
}

/// `tf2::Transform`: a basis plus an origin.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform {
    pub basis: Matrix3x3,
    pub origin: Vector3,
}

impl Default for Transform {
    fn default() -> Self {
        Self::identity()
    }
}

impl Transform {
    pub const fn identity() -> Self {
        Self {
            basis: Matrix3x3::identity(),
            origin: Vector3::zero(),
        }
    }

    pub fn from_parts(rotation: &Quaternion, origin: Vector3) -> Self {
        Self {
            basis: Matrix3x3::from_quaternion(rotation),
            origin,
        }
    }

    /// `setRotation`.
    pub fn set_rotation(&mut self, q: &Quaternion) {
        self.basis = Matrix3x3::from_quaternion(q);
    }

    /// `getRotation`.
    pub fn rotation(&self) -> Quaternion {
        self.basis.get_rotation()
    }

    /// `operator()(Vector3)`: apply the transform to a point.
    pub fn apply(&self, x: &Vector3) -> Vector3 {
        Vector3::new(
            self.basis.el[0].dot(x) + self.origin.x,
            self.basis.el[1].dot(x) + self.origin.y,
            self.basis.el[2].dot(x) + self.origin.z,
        )
    }

    /// `operator*(Transform)`.
    pub fn mul(&self, t: &Transform) -> Transform {
        Transform {
            basis: self.basis.mul(&t.basis),
            origin: self.apply(&t.origin),
        }
    }

    /// `inverse()`.
    pub fn inverse(&self) -> Transform {
        let inv = self.basis.transpose();
        Transform {
            origin: inv.mul_vec(&self.origin.neg()),
            basis: inv,
        }
    }

    /// `inverseTimes(t)` == `this->inverse() * t`, computed the way tf2 does.
    pub fn inverse_times(&self, t: &Transform) -> Transform {
        let v = t.origin.sub(&self.origin);
        Transform {
            basis: self.basis.transpose_times(&t.basis),
            origin: self.basis.vec_mul(&v),
        }
    }
}

/// `angles::normalize_angle` (the `angles` package's exact formulation).
pub fn normalize_angle(angle: f64) -> f64 {
    let result = (angle + std::f64::consts::PI) % (2.0 * std::f64::consts::PI);
    if result <= 0.0 {
        result + std::f64::consts::PI
    } else {
        result - std::f64::consts::PI
    }
}
