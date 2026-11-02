//! `robot_localization/src/navsat_transform.cpp` (Jazzy 3.8.3): the datum,
//! the UTM <-> map transform that produces `/odometry/gps`, and the inverse
//! that produces `/gps/filtered`.
//!
//! Ported for this robot's yaml: `use_local_cartesian` false (so everything
//! goes through UTM), `use_odometry_yaw` false (the IMU supplies the heading),
//! `wait_for_datum` false (the datum is the first good fix), `zero_altitude`
//! true, `publish_filtered_gps` true.
//!
//! One rule upstream does not have, off by default so the oracle vectors
//! hold: [`NavSatTransformCore::datum_requires_rtk_fixed`].

use crate::filter_common::{POSE_SIZE, POSITION_SIZE};
use crate::msgs::*;
use crate::prepare::TransformTree;
use crate::tf::*;
use crate::utm::{utmups_forward, utmups_reverse, UtmError, ZONE_STANDARD};

const RADIANS_PER_DEGREE: f64 = std::f64::consts::PI / 180.0;

/// The navsat_transform parameters this robot sets.
#[derive(Clone, Debug)]
pub struct NavSatConfig {
    pub magnetic_declination_radians: f64,
    pub yaw_offset: f64,
    pub zero_altitude: bool,
    pub publish_filtered_gps: bool,
    pub use_odometry_yaw: bool,
    pub wait_for_datum: bool,
    pub broadcast_cartesian_transform: bool,
    pub broadcast_cartesian_transform_as_parent_frame: bool,
}

impl Default for NavSatConfig {
    fn default() -> Self {
        Self {
            magnetic_declination_radians: 0.0,
            yaw_offset: 0.0,
            zero_altitude: false,
            publish_filtered_gps: true,
            use_odometry_yaw: false,
            wait_for_datum: false,
            broadcast_cartesian_transform: false,
            broadcast_cartesian_transform_as_parent_frame: false,
        }
    }
}

/// `NavSatTransform`, minus the ROS plumbing.
pub struct NavSatTransformCore {
    pub config: NavSatConfig,
    pub transforms: TransformTree,

    pub world_frame_id: String,
    pub base_link_frame_id: String,
    pub gps_frame_id: String,

    pub transform_good: bool,
    /// Lock the datum only on an RTK fixed fix (status `STATUS_GBAS_FIX`):
    /// a fix of any lower status still feeds `/odometry/gps` once the datum
    /// exists, but never becomes it. Upstream takes the first good fix of any
    /// status, so a plain 3D fix metres off could anchor the map (outdoors,
    /// 2026-11-02, the origin moved 0.9 m between restarts of a robot
    /// standing still). Not a robot_localization parameter: false here, as
    /// upstream; mower_localize sets it from `datum_requires_rtk_fixed`. A
    /// manual datum (`/datum`) is not gated.
    pub datum_requires_rtk_fixed: bool,
    fixes_ignored_for_datum: u64,
    has_transform_odom: bool,
    has_transform_gps: bool,
    has_transform_imu: bool,
    use_manual_datum: bool,
    force_user_utm: bool,

    pub utm_zone: i32,
    pub northp: bool,
    pub utm_meridian_convergence: f64,

    transform_cartesian_pose: Transform,
    transform_world_pose: Transform,
    transform_orientation: Quaternion,
    pub cartesian_world_transform: Transform,
    pub cartesian_world_trans_inverse: Transform,

    latest_cartesian_pose: Transform,
    latest_cartesian_covariance: Mat6,
    latest_world_pose: Transform,
    latest_odom_covariance: Mat6,

    gps_update_time_ns: i64,
    odom_update_time_ns: i64,
    gps_updated: bool,
    odom_updated: bool,

    manual_datum: Option<(f64, f64, f64, Quaternion)>,
}

impl NavSatTransformCore {
    pub fn new(config: NavSatConfig) -> Self {
        let use_manual_datum = config.wait_for_datum;
        Self {
            config,
            transforms: TransformTree::new(),
            world_frame_id: "odom".to_string(),
            base_link_frame_id: "base_link".to_string(),
            gps_frame_id: String::new(),
            transform_good: false,
            datum_requires_rtk_fixed: false,
            fixes_ignored_for_datum: 0,
            has_transform_odom: false,
            has_transform_gps: false,
            has_transform_imu: false,
            use_manual_datum,
            force_user_utm: false,
            utm_zone: 0,
            northp: true,
            utm_meridian_convergence: 0.0,
            transform_cartesian_pose: Transform::identity(),
            transform_world_pose: Transform::identity(),
            transform_orientation: Quaternion::identity(),
            cartesian_world_transform: Transform::identity(),
            cartesian_world_trans_inverse: Transform::identity(),
            latest_cartesian_pose: Transform::identity(),
            latest_cartesian_covariance: ZERO_MAT6,
            latest_world_pose: Transform::identity(),
            latest_odom_covariance: ZERO_MAT6,
            gps_update_time_ns: 0,
            odom_update_time_ns: 0,
            gps_updated: false,
            odom_updated: false,
            manual_datum: None,
        }
    }

    /// `NavSatTransform::datumCallback`: store a manual datum (lat, lon, alt,
    /// orientation) and force the transform to be recomputed from it.
    pub fn set_datum(
        &mut self,
        latitude: f64,
        longitude: f64,
        altitude: f64,
        orientation: Quaternion,
    ) {
        self.manual_datum = Some((latitude, longitude, altitude, orientation));
        self.use_manual_datum = true;
        self.transform_good = false;
    }

    /// `NavSatTransform::setTransformGps`.
    fn set_transform_gps(&mut self, fix: &NavSatFix) -> Result<(), UtmError> {
        let set_zone = if self.force_user_utm {
            self.utm_zone
        } else {
            ZONE_STANDARD
        };
        let (zone, northp, x, y, gamma_deg, _k) =
            utmups_forward(fix.latitude, fix.longitude, set_zone)?;
        self.utm_zone = zone;
        self.northp = northp;
        self.utm_meridian_convergence = gamma_deg * RADIANS_PER_DEGREE;
        self.transform_cartesian_pose = Transform::identity();
        self.transform_cartesian_pose.origin = Vector3::new(x, y, fix.altitude);
        self.has_transform_gps = true;
        Ok(())
    }

    /// `NavSatTransform::setTransformOdometry`.
    fn set_transform_odometry(&mut self, msg: &Odometry) {
        self.transform_world_pose = Transform::from_parts(&msg.pose.orientation, msg.pose.position);
        self.has_transform_odom = true;

        if !self.transform_good && self.config.use_odometry_yaw && !self.use_manual_datum {
            let imu = Imu {
                header: Header {
                    frame_id: msg.child_frame_id.clone(),
                    stamp_ns: msg.header.stamp_ns,
                },
                orientation: msg.pose.orientation,
                ..Default::default()
            };
            self.imu_callback(&imu);
        }
    }

    /// `NavSatTransform::odomCallback`.
    pub fn odom_callback(&mut self, msg: &Odometry) {
        self.world_frame_id = msg.header.frame_id.clone();
        self.base_link_frame_id = msg.child_frame_id.clone();

        if !self.transform_good {
            self.set_transform_odometry(msg);
        }

        self.latest_world_pose = Transform::from_parts(&msg.pose.orientation, msg.pose.position);
        self.latest_odom_covariance = msg.pose.covariance;
        self.odom_update_time_ns = msg.header.stamp_ns;
        self.odom_updated = true;
    }

    /// `NavSatTransform::imuCallback`.
    pub fn imu_callback(&mut self, msg: &Imu) {
        if self.transform_good && !self.config.use_odometry_yaw && !self.use_manual_datum {
            return;
        }
        if !self.has_transform_odom {
            return;
        }

        self.transform_orientation = msg.orientation;

        let target_frame_trans = match self
            .transforms
            .lookup(&self.base_link_frame_id, &msg.header.frame_id)
        {
            Some(t) => t,
            None => return,
        };

        let (roll_offset, pitch_offset, yaw_offset) =
            Matrix3x3::from_quaternion(&target_frame_trans.rotation()).get_rpy();
        let (roll, pitch, yaw) = Matrix3x3::from_quaternion(&self.transform_orientation).get_rpy();

        let rpy_angles = Vector3::new(
            normalize_angle(roll - roll_offset),
            normalize_angle(pitch - pitch_offset),
            normalize_angle(yaw - yaw_offset),
        );
        let mat = Matrix3x3::from_rpy(0.0, 0.0, yaw_offset);
        let rpy_angles = mat.mul_vec(&rpy_angles);
        self.transform_orientation = Quaternion::from_rpy(rpy_angles.x, rpy_angles.y, rpy_angles.z);
        self.has_transform_imu = true;
    }

    /// `NavSatTransform::gpsFixCallback`.
    pub fn gps_fix_callback(&mut self, msg: &NavSatFix) {
        self.gps_frame_id = msg.header.frame_id.clone();

        let good_gps = msg.status != STATUS_NO_FIX
            && !msg.altitude.is_nan()
            && !msg.latitude.is_nan()
            && !msg.longitude.is_nan();
        if !good_gps {
            return;
        }

        if !self.transform_good && !self.use_manual_datum {
            if self.datum_requires_rtk_fixed && msg.status != STATUS_GBAS_FIX {
                // Not RTK fixed: never the datum, and neither is an older
                // fixed fix the transform has not been built from yet. Like
                // upstream's, the datum is the latest fix when the timer
                // builds it, so it waits for the next fixed one. Nothing is
                // published before the datum, so this fix has no other use.
                self.has_transform_gps = false;
                self.fixes_ignored_for_datum += 1;
                return;
            }
            let _ = self.set_transform_gps(msg);
        }

        let set_zone = self.utm_zone;
        let (_, _, x, y, _, _) = match utmups_forward(msg.latitude, msg.longitude, set_zone) {
            Ok(v) => v,
            Err(_) => return,
        };

        self.latest_cartesian_pose = Transform::identity();
        self.latest_cartesian_pose.origin = Vector3::new(x, y, msg.altitude);
        self.latest_cartesian_covariance = ZERO_MAT6;
        for i in 0..POSITION_SIZE {
            for j in 0..POSITION_SIZE {
                self.latest_cartesian_covariance[i][j] = msg.position_covariance[i][j];
            }
        }
        self.gps_update_time_ns = msg.header.stamp_ns;
        self.gps_updated = true;
    }

    /// Good fixes `datum_requires_rtk_fixed` has kept from becoming the
    /// datum (0 while it is off). The node logs the first one.
    pub fn fixes_ignored_for_datum(&self) -> u64 {
        self.fixes_ignored_for_datum
    }

    /// Whether the datum is (or will be) a manual one, from `/datum`.
    pub fn manual_datum(&self) -> bool {
        self.use_manual_datum
    }

    /// `NavSatTransform::setManualDatum`.
    fn set_manual_datum(&mut self) {
        let Some((lat, lon, alt, orientation)) = self.manual_datum else {
            return;
        };
        let mut fix = NavSatFix {
            latitude: lat,
            longitude: lon,
            altitude: alt,
            status: STATUS_FIX,
            ..Default::default()
        };
        fix.position_covariance[0][0] = 0.1;
        fix.position_covariance[1][1] = 0.1;
        fix.position_covariance[2][2] = 0.1;
        let _ = self.set_transform_gps(&fix);

        let odom = Odometry {
            header: Header {
                frame_id: self.world_frame_id.clone(),
                stamp_ns: 0,
            },
            child_frame_id: self.base_link_frame_id.clone(),
            ..Default::default()
        };
        self.set_transform_odometry(&odom);

        let imu = Imu {
            header: Header {
                frame_id: self.base_link_frame_id.clone(),
                stamp_ns: 0,
            },
            orientation,
            ..Default::default()
        };
        self.imu_callback(&imu);
    }

    /// `NavSatTransform::getRobotOriginCartesianPose`.
    fn robot_origin_cartesian_pose(&self, gps_cartesian_pose: &Transform) -> Transform {
        let offset = match self
            .transforms
            .lookup(&self.base_link_frame_id, &self.gps_frame_id)
        {
            Some(t) => t,
            None => return *gps_cartesian_pose,
        };
        let cartesian_orientation = self.transform_orientation;
        let (roll, pitch, yaw) = Matrix3x3::from_quaternion(&cartesian_orientation).get_rpy();
        let yaw = yaw
            + self.config.magnetic_declination_radians
            + self.config.yaw_offset
            + self.utm_meridian_convergence;
        let cartesian_orientation = Quaternion::from_rpy(roll, pitch, yaw);

        let mut offset = offset;
        offset.origin = quat_rotate(&cartesian_orientation, &offset.origin);
        offset.set_rotation(&Quaternion::identity());
        offset.inverse().mul(gps_cartesian_pose)
    }

    /// `NavSatTransform::getRobotOriginWorldPose`.
    fn robot_origin_world_pose(&self, gps_odom_pose: &Transform) -> Transform {
        let mut gps_offset_rotated = match self
            .transforms
            .lookup(&self.base_link_frame_id, &self.gps_frame_id)
        {
            Some(t) => t,
            None => return Transform::identity(),
        };
        let robot_orientation = match self
            .transforms
            .lookup(&self.world_frame_id, &self.base_link_frame_id)
        {
            Some(t) => t,
            None => return Transform::identity(),
        };
        gps_offset_rotated.origin =
            quat_rotate(&robot_orientation.rotation(), &gps_offset_rotated.origin);
        gps_offset_rotated.set_rotation(&Quaternion::identity());
        gps_offset_rotated.inverse().mul(gps_odom_pose)
    }

    /// `NavSatTransform::computeTransform`.
    pub fn compute_transform(&mut self) {
        if !self.transform_good && self.has_transform_odom && self.use_manual_datum {
            self.set_manual_datum();
        }

        if !(!self.transform_good
            && self.has_transform_odom
            && self.has_transform_gps
            && self.has_transform_imu)
        {
            return;
        }

        let transform_cartesian_pose_corrected = if !self.use_manual_datum {
            self.robot_origin_cartesian_pose(&self.transform_cartesian_pose)
        } else {
            self.transform_cartesian_pose
        };

        let (_, _, imu_yaw) = Matrix3x3::from_quaternion(&self.transform_orientation).get_rpy();
        let imu_yaw = imu_yaw
            + self.config.magnetic_declination_radians
            + self.config.yaw_offset
            + self.utm_meridian_convergence;
        let imu_quat = Quaternion::from_rpy(0.0, 0.0, imu_yaw);

        let mut cartesian_pose_with_orientation = Transform::identity();
        cartesian_pose_with_orientation.origin = transform_cartesian_pose_corrected.origin;
        cartesian_pose_with_orientation.set_rotation(&imu_quat);

        let (_, _, odom_yaw) =
            Matrix3x3::from_quaternion(&self.transform_world_pose.rotation()).get_rpy();
        let odom_quat = Quaternion::from_rpy(0.0, 0.0, odom_yaw);
        let mut transform_world_pose_yaw_only = self.transform_world_pose;
        transform_world_pose_yaw_only.set_rotation(&odom_quat);

        self.cartesian_world_transform =
            transform_world_pose_yaw_only.mul(&cartesian_pose_with_orientation.inverse());
        self.cartesian_world_trans_inverse = self.cartesian_world_transform.inverse();
        self.transform_good = true;
    }

    /// `NavSatTransform::cartesianToMap`.
    fn cartesian_to_map(&self, cartesian_pose: &Transform) -> Odometry {
        let mut transformed = self.cartesian_world_transform.mul(cartesian_pose);
        transformed.set_rotation(&Quaternion::identity());

        let mut gps_odom = Odometry {
            header: Header {
                frame_id: self.world_frame_id.clone(),
                stamp_ns: self.gps_update_time_ns,
            },
            ..Default::default()
        };
        gps_odom.pose.position = transformed.origin;
        gps_odom.pose.orientation = transformed.rotation();
        if self.config.zero_altitude {
            gps_odom.pose.position.z = 0.0;
        }
        gps_odom
    }

    /// `NavSatTransform::toLLCallback`: `None` until the datum exists.
    ///
    /// Upstream returns `false` there, which rclcpp ignores, so the client
    /// receives a default-constructed `GeoPoint` (0, 0, 0) -- the answer both
    /// adapters take to mean "no navsat datum yet". Calling [`Self::map_to_ll`]
    /// instead would project through the identity transform and whatever UTM
    /// zone the first good fix set, e.g. (0.0, 118.51) in zone 51.
    pub fn to_ll(&self, point: &Vector3) -> Option<(f64, f64, f64)> {
        if !self.transform_good {
            return None;
        }
        Some(self.map_to_ll(point))
    }

    /// `NavSatTransform::mapToLL`. Returns (latitude, longitude, altitude).
    pub fn map_to_ll(&self, point: &Vector3) -> (f64, f64, f64) {
        let mut pose = Transform::identity();
        pose.origin = *point;
        let mut odom_as_cartesian = self.cartesian_world_trans_inverse.mul(&pose);
        odom_as_cartesian.set_rotation(&Quaternion::identity());

        match utmups_reverse(
            self.utm_zone,
            self.northp,
            odom_as_cartesian.origin.x,
            odom_as_cartesian.origin.y,
        ) {
            Ok((lat, lon, _, _)) => (lat, lon, odom_as_cartesian.origin.z),
            Err(_) => (f64::NAN, f64::NAN, f64::NAN),
        }
    }

    /// `NavSatTransform::fromLL`: a geographic point in the filter's world frame.
    pub fn from_ll(&self, latitude: f64, longitude: f64, altitude: f64) -> Option<Vector3> {
        if !self.transform_good {
            return None;
        }
        let (_, _, x, y, _, _) = utmups_forward(latitude, longitude, self.utm_zone).ok()?;
        let mut cartesian_pose = Transform::identity();
        cartesian_pose.origin = Vector3::new(x, y, altitude);
        Some(self.cartesian_to_map(&cartesian_pose).pose.position)
    }

    /// `NavSatTransform::prepareGpsOdometry`: what gets published on
    /// `/odometry/gps` and fused as `odom1` by the map EKF.
    pub fn prepare_gps_odometry(&mut self) -> Option<Odometry> {
        if !(self.transform_good && self.gps_updated && self.odom_updated) {
            return None;
        }

        let mut gps_odom = self.cartesian_to_map(&self.latest_cartesian_pose);
        let transformed_cartesian_gps =
            Transform::from_parts(&gps_odom.pose.orientation, gps_odom.pose.position);
        let transformed_cartesian_robot = self.robot_origin_world_pose(&transformed_cartesian_gps);

        let rot = Matrix3x3::from_quaternion(&self.cartesian_world_transform.rotation());
        let r6 = rot6d_local(&rot);
        self.latest_cartesian_covariance = mat6_mul_local(
            &mat6_mul_local(&r6, &self.latest_cartesian_covariance),
            &mat6_transpose_local(&r6),
        );

        gps_odom.pose.position = transformed_cartesian_robot.origin;
        gps_odom.pose.orientation = transformed_cartesian_robot.rotation();
        if self.config.zero_altitude {
            gps_odom.pose.position.z = 0.0;
        }
        for i in 0..POSE_SIZE {
            for j in 0..POSE_SIZE {
                gps_odom.pose.covariance[i][j] = self.latest_cartesian_covariance[i][j];
            }
        }

        self.gps_updated = false;
        Some(gps_odom)
    }

    /// `NavSatTransform::prepareFilteredGps`: what gets published on
    /// `/gps/filtered` when `publish_filtered_gps` is true.
    pub fn prepare_filtered_gps(&mut self) -> Option<NavSatFix> {
        if !(self.transform_good && self.odom_updated) {
            return None;
        }
        let (latitude, longitude, altitude) = self.map_to_ll(&self.latest_world_pose.origin);

        let rot = Matrix3x3::from_quaternion(&self.cartesian_world_trans_inverse.rotation());
        let r6 = rot6d_local(&rot);
        self.latest_odom_covariance = mat6_mul_local(
            &mat6_mul_local(&r6, &self.latest_odom_covariance),
            &mat6_transpose_local(&r6),
        );

        let mut fix = NavSatFix {
            header: Header {
                frame_id: self.base_link_frame_id.clone(),
                stamp_ns: self.odom_update_time_ns,
            },
            status: STATUS_GBAS_FIX,
            latitude,
            longitude,
            altitude,
            position_covariance: ZERO_MAT3,
        };
        for i in 0..POSITION_SIZE {
            for j in 0..POSITION_SIZE {
                fix.position_covariance[i][j] = self.latest_odom_covariance[i][j];
            }
        }

        self.odom_updated = false;
        Some(fix)
    }
}

fn rot6d_local(rot: &Matrix3x3) -> Mat6 {
    let mut m = ZERO_MAT6;
    for (i, row) in m.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    for r in 0..POSITION_SIZE {
        m[r][0] = rot.row(r).x;
        m[r][1] = rot.row(r).y;
        m[r][2] = rot.row(r).z;
        m[r + POSITION_SIZE][3] = rot.row(r).x;
        m[r + POSITION_SIZE][4] = rot.row(r).y;
        m[r + POSITION_SIZE][5] = rot.row(r).z;
    }
    m
}

fn mat6_mul_local(a: &Mat6, b: &Mat6) -> Mat6 {
    let mut c = ZERO_MAT6;
    for i in 0..6 {
        for j in 0..6 {
            let mut s = 0.0;
            for k in 0..6 {
                s += a[i][k] * b[k][j];
            }
            c[i][j] = s;
        }
    }
    c
}

fn mat6_transpose_local(a: &Mat6) -> Mat6 {
    let mut t = ZERO_MAT6;
    for i in 0..6 {
        for j in 0..6 {
            t[i][j] = a[j][i];
        }
    }
    t
}

/// `datum_requires_rtk_fixed`: off, the first good fix of any status is the
/// datum, as upstream; on, only an RTK fixed one is, and a manual datum
/// still needs none.
#[cfg(test)]
mod datum_requires_rtk_fixed {
    use super::*;

    /// Where the antenna really is.
    const TRUTH: (f64, f64) = (23.6940, 120.5377);

    /// A fix `north_m` north of the truth (no antenna offset: the GPS frame
    /// is the base frame).
    fn fix(status: i8, north_m: f64) -> NavSatFix {
        let mut f = NavSatFix {
            header: Header { frame_id: "base_footprint".to_string(), stamp_ns: 2 },
            status,
            latitude: TRUTH.0 + north_m / 110_800.0,
            longitude: TRUTH.1,
            altitude: 50.0,
            position_covariance: ZERO_MAT3,
        };
        f.position_covariance[0][0] = 0.04;
        f.position_covariance[1][1] = 0.04;
        f
    }

    /// The map EKF's odometry: the robot at the map origin, heading 0 (and
    /// the same as `map -> base_footprint`, which /odometry/gps looks up).
    fn odom(core: &mut NavSatTransformCore) {
        core.transforms.insert("map", "base_footprint", Transform::identity());
        core.odom_callback(&Odometry {
            header: Header { frame_id: "map".to_string(), stamp_ns: 1 },
            child_frame_id: "base_footprint".to_string(),
            ..Default::default()
        });
    }

    fn imu(core: &mut NavSatTransformCore) {
        core.imu_callback(&Imu {
            header: Header { frame_id: "base_footprint".to_string(), stamp_ns: 1 },
            orientation: Quaternion::identity(),
            ..Default::default()
        });
    }

    /// The core as the delay ends: the map EKF's odometry and the IMU in,
    /// only the datum's fix missing.
    fn ready(gated: bool) -> NavSatTransformCore {
        let mut core = NavSatTransformCore::new(NavSatConfig::default());
        core.datum_requires_rtk_fixed = gated;
        odom(&mut core);
        imu(&mut core);
        core
    }

    /// A fix, then the timer's tick.
    fn step(core: &mut NavSatTransformCore, f: NavSatFix) {
        core.gps_fix_callback(&f);
        core.compute_transform();
    }

    /// The latitude the map origin was locked at.
    fn origin_lat(core: &NavSatTransformCore) -> f64 {
        core.to_ll(&Vector3::new(0.0, 0.0, 0.0)).expect("a datum").0
    }

    #[test]
    fn off_the_first_good_fix_is_the_datum_whatever_its_status() {
        let mut core = ready(false);
        assert!(!core.datum_requires_rtk_fixed, "the test's own setting");
        assert!(!NavSatTransformCore::new(NavSatConfig::default()).datum_requires_rtk_fixed, "off by default");
        // a plain 3D fix, 3 m off
        step(&mut core, fix(STATUS_FIX, 3.0));
        assert!(core.transform_good);
        assert!((origin_lat(&core) - (TRUTH.0 + 3.0 / 110_800.0)).abs() < 1e-8);
        assert_eq!(core.fixes_ignored_for_datum(), 0);
    }

    #[test]
    fn on_the_datum_waits_through_3d_and_float_and_locks_at_fixed() {
        let mut core = ready(true);
        // 3D, 3 m off; RTK float, 0.3 m off (both status 0 on /fix); SBAS
        for f in [fix(STATUS_FIX, 3.0), fix(STATUS_FIX, 0.3), fix(1, 0.3)] {
            step(&mut core, f);
            assert!(!core.transform_good);
        }
        assert_eq!(core.fixes_ignored_for_datum(), 3);
        assert_eq!(core.to_ll(&Vector3::new(0.0, 0.0, 0.0)), None);
        assert!(core.prepare_gps_odometry().is_none());
        // no fix at all is no good fix, as upstream: not counted
        step(&mut core, fix(STATUS_NO_FIX, 0.0));
        assert_eq!(core.fixes_ignored_for_datum(), 3);

        step(&mut core, fix(STATUS_GBAS_FIX, 0.0));
        assert!(core.transform_good);
        assert!((origin_lat(&core) - TRUTH.0).abs() < 1e-8);

        // Once locked, a float fix feeds /odometry/gps but never moves the datum.
        core.gps_fix_callback(&fix(STATUS_FIX, 0.3));
        let gps = core.prepare_gps_odometry().expect("/odometry/gps from a float fix");
        assert!((gps.pose.position.y - 0.3).abs() < 0.01, "{:?}", gps.pose.position);
        core.compute_transform();
        assert!((origin_lat(&core) - TRUTH.0).abs() < 1e-8);
        assert_eq!(core.fixes_ignored_for_datum(), 3);
    }

    /// A fixed fix the transform has not been built from yet (the IMU was
    /// late) is not the datum once a float one has come after it: the datum
    /// is the latest fix, which must be fixed.
    #[test]
    fn on_a_later_float_fix_drops_a_fixed_one_not_built_on_yet() {
        let mut core = NavSatTransformCore::new(NavSatConfig::default());
        core.datum_requires_rtk_fixed = true;
        odom(&mut core);
        step(&mut core, fix(STATUS_GBAS_FIX, 1.0));
        assert!(!core.transform_good, "no IMU yet");
        core.gps_fix_callback(&fix(STATUS_FIX, 0.4));
        imu(&mut core);
        core.compute_transform();
        assert!(!core.transform_good, "the last fix was float");
        step(&mut core, fix(STATUS_GBAS_FIX, 0.0));
        assert!(core.transform_good);
        assert!((origin_lat(&core) - TRUTH.0).abs() < 1e-8);
    }

    /// `/datum` (datum_check.py --fix) sets a manual datum without RTK.
    #[test]
    fn on_a_manual_datum_needs_no_rtk_fixed() {
        let mut core = ready(true);
        step(&mut core, fix(STATUS_FIX, 3.0));
        assert!(!core.transform_good);
        core.set_datum(25.0330, 121.5654, 10.0, Quaternion::identity());
        core.compute_transform();
        assert!(core.transform_good && core.manual_datum());
        assert!((origin_lat(&core) - 25.0330).abs() < 1e-7);
    }
}
