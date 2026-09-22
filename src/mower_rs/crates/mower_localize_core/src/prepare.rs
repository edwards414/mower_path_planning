//! The `RosFilter` measurement preprocessing from
//! `robot_localization/src/ros_filter.cpp` (Jazzy 3.8.3), for the sensor types
//! this robot fuses: `nav_msgs/Odometry` (pose and twist) and `sensor_msgs/Imu`
//! (orientation, angular velocity, linear acceleration).
//!
//! ROS is replaced by two things: the plain structs in [`crate::msgs`], and a
//! [`TransformTree`] of fixed transforms instead of a tf2 buffer (the robot's
//! sensor frames are all rigidly attached, so every lookup this code path makes
//! resolves to a static transform or to the identity).

use std::collections::HashMap;

use crate::ekf::Ekf;
use crate::filter_common::*;
use crate::measurement::Measurement;
use crate::msgs::*;
use crate::tf::*;

/// Per-sensor configuration: one entry per `odomN`/`imuN` in the yaml.
#[derive(Clone, Debug)]
pub struct SensorConfig {
    pub topic_name: String,
    /// `<topic>_config`, in state order.
    pub update_vector: [bool; STATE_SIZE],
    pub differential: bool,
    pub relative: bool,
    /// `<topic>_rejection_threshold` in sigmas; upstream's default is +inf.
    pub rejection_threshold: f64,
    /// `<topic>_pose_use_child_frame`, odometry only.
    pub pose_use_child_frame: bool,
    /// `imuN_remove_gravitational_acceleration`.
    pub remove_gravitational_acceleration: bool,
}

impl SensorConfig {
    pub fn new(topic_name: &str, update_vector: [bool; STATE_SIZE]) -> Self {
        Self {
            topic_name: topic_name.to_string(),
            update_vector,
            differential: false,
            relative: false,
            rejection_threshold: f64::MAX,
            pose_use_child_frame: false,
            remove_gravitational_acceleration: false,
        }
    }

    /// The per-portion update vectors `loadParams` derives from a sensor's
    /// `<topic>_config`: each callback only ever sees its own slice.
    fn masked(&self, keep: &[std::ops::Range<usize>]) -> [bool; STATE_SIZE] {
        let mut v = [false; STATE_SIZE];
        for r in keep {
            for i in r.clone() {
                v[i] = self.update_vector[i];
            }
        }
        v
    }

    /// `odomN` pose: everything but the twist block (`ros_filter.cpp` zeroes
    /// `POSITION_V_OFFSET .. +TWIST_SIZE`).
    pub fn odom_pose_vector(&self) -> [bool; STATE_SIZE] {
        self.masked(&[0..POSE_SIZE, POSITION_A_OFFSET..STATE_SIZE])
    }

    /// `odomN` twist: everything but the pose block.
    pub fn odom_twist_vector(&self) -> [bool; STATE_SIZE] {
        self.masked(&[POSITION_V_OFFSET..POSITION_V_OFFSET + TWIST_SIZE])
    }

    /// `imuN` pose: orientation only.
    pub fn imu_pose_vector(&self) -> [bool; STATE_SIZE] {
        self.masked(&[ORIENTATION_OFFSET..ORIENTATION_OFFSET + ORIENTATION_SIZE])
    }

    /// `imuN` twist: angular velocity only.
    pub fn imu_twist_vector(&self) -> [bool; STATE_SIZE] {
        self.masked(&[ORIENTATION_V_OFFSET..ORIENTATION_V_OFFSET + ORIENTATION_SIZE])
    }

    /// `imuN` acceleration: linear acceleration only.
    pub fn imu_accel_vector(&self) -> [bool; STATE_SIZE] {
        self.masked(&[POSITION_A_OFFSET..POSITION_A_OFFSET + ACCELERATION_SIZE])
    }

    fn sum(v: &[bool; STATE_SIZE]) -> usize {
        v.iter().filter(|b| **b).count()
    }
}

/// The fixed transforms the preprocessing needs, standing in for `tf2_ros::Buffer`.
///
/// `lookupTransformSafe` falls back to the identity when target == source, and
/// reports failure otherwise; both behaviours are reproduced here.
#[derive(Clone, Debug, Default)]
pub struct TransformTree {
    transforms: HashMap<(String, String), Transform>,
}

impl TransformTree {
    pub fn new() -> Self {
        Self::default()
    }

    /// Insert `target <- source` and its inverse.
    pub fn insert(&mut self, target: &str, source: &str, transform: Transform) {
        self.transforms
            .insert((target.to_string(), source.to_string()), transform);
        self.transforms.insert(
            (source.to_string(), target.to_string()),
            transform.inverse(),
        );
    }

    /// `ros_filter_utilities::lookupTransformSafe`.
    pub fn lookup(&self, target: &str, source: &str) -> Option<Transform> {
        if let Some(t) = self
            .transforms
            .get(&(target.to_string(), source.to_string()))
        {
            return Some(*t);
        }
        if target == source {
            return Some(Transform::identity());
        }
        None
    }
}

/// `RosFilter::copyCovariance(const double*, ...)`. The diagnostics branch is
/// skipped: `print_diagnostics` is false in this robot's yaml.
fn copy_covariance6(src: &Mat6) -> Mat6 {
    *src
}

fn copy_covariance3(src: &Mat3) -> Mat3 {
    *src
}

fn mask_matrix(a: bool, b: bool, c: bool) -> Matrix3x3 {
    Matrix3x3::new(
        a as i32 as f64,
        0.0,
        0.0,
        0.0,
        b as i32 as f64,
        0.0,
        0.0,
        0.0,
        c as i32 as f64,
    )
}

/// Build the 6x6 block-diagonal rotation used to rotate pose/twist covariances.
fn rot6d(rot: &Matrix3x3) -> Mat6 {
    let mut m = [[0.0f64; 6]; 6];
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

fn mat6_mul(a: &Mat6, b: &Mat6) -> Mat6 {
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

fn mat6_transpose(a: &Mat6) -> Mat6 {
    let mut t = ZERO_MAT6;
    for i in 0..6 {
        for j in 0..6 {
            t[i][j] = a[j][i];
        }
    }
    t
}

fn mat3_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut c = ZERO_MAT3;
    for i in 0..3 {
        for j in 0..3 {
            let mut s = 0.0;
            for k in 0..3 {
                s += a[i][k] * b[k][j];
            }
            c[i][j] = s;
        }
    }
    c
}

fn rot3d(rot: &Matrix3x3) -> Mat3 {
    let mut m = ZERO_MAT3;
    for (r, row) in m.iter_mut().enumerate() {
        row[0] = rot.row(r).x;
        row[1] = rot.row(r).y;
        row[2] = rot.row(r).z;
    }
    m
}

/// `RosFilter::forceTwoD`.
fn force_two_d(
    measurement: &mut Vec15,
    covariance: &mut Mat15,
    update_vector: &mut [bool; STATE_SIZE],
) {
    const IDX: [usize; 7] = [
        STATE_Z,
        STATE_ROLL,
        STATE_PITCH,
        STATE_VZ,
        STATE_VROLL,
        STATE_VPITCH,
        STATE_AZ,
    ];
    for i in IDX {
        measurement[i] = 0.0;
    }
    for i in IDX {
        for k in 0..STATE_SIZE {
            covariance[k][i] = 0.0;
        }
        covariance[i] = [0.0; STATE_SIZE];
    }
    for i in IDX {
        covariance[i][i] = 1e-6;
        update_vector[i] = true;
    }
}

/// The state of the `RosFilter` wrapper: the filter plus everything the
/// preprocessing remembers between messages.
pub struct RosFilterCore {
    pub filter: Ekf,
    pub two_d_mode: bool,
    pub world_frame_id: String,
    pub map_frame_id: String,
    pub odom_frame_id: String,
    pub base_link_frame_id: String,
    pub base_link_output_frame_id: String,
    pub gravitational_acceleration: f64,
    pub transforms: TransformTree,
    /// `predict_to_current_time_`; false unless set in the yaml.
    pub predict_to_current_time: bool,

    last_message_times: HashMap<String, i64>,
    previous_measurements: HashMap<String, Transform>,
    previous_measurement_covariances: HashMap<String, Mat6>,
    initial_measurements: HashMap<String, Transform>,
    pub angular_acceleration: Vector3,
    last_state_twist_rot: Vector3,
    pub last_diff_time: f64,
    /// Ordered measurement queue; ties keep insertion order (upstream's
    /// `std::priority_queue` leaves ties unspecified).
    queue: Vec<(i64, u64, Measurement)>,
    seq: u64,
}

impl RosFilterCore {
    pub fn new(world_frame_id: &str, base_link_frame_id: &str) -> Self {
        Self {
            filter: Ekf::new(),
            two_d_mode: false,
            world_frame_id: world_frame_id.to_string(),
            map_frame_id: "map".to_string(),
            odom_frame_id: "odom".to_string(),
            base_link_frame_id: base_link_frame_id.to_string(),
            base_link_output_frame_id: base_link_frame_id.to_string(),
            gravitational_acceleration: 9.80665,
            transforms: TransformTree::new(),
            predict_to_current_time: false,
            last_message_times: HashMap::new(),
            previous_measurements: HashMap::new(),
            previous_measurement_covariances: HashMap::new(),
            initial_measurements: HashMap::new(),
            angular_acceleration: Vector3::zero(),
            last_state_twist_rot: Vector3::zero(),
            last_diff_time: 0.0,
            queue: Vec::new(),
            seq: 0,
        }
    }

    /// `RosFilter::enqueueMeasurement`.
    fn enqueue_measurement(&mut self, meas: Measurement) {
        let t = meas.time_ns;
        let s = self.seq;
        self.seq += 1;
        self.queue.push((t, s, meas));
        self.queue.sort_by(|a, b| (a.0, a.1).cmp(&(b.0, b.1)));
    }

    pub fn queue_len(&self) -> usize {
        self.queue.len()
    }

    /// The measurements waiting to be integrated, in the order they will be.
    pub fn queued(&self) -> Vec<&Measurement> {
        self.queue.iter().map(|(_, _, m)| m).collect()
    }

    /// `RosFilter::odometryCallback`.
    pub fn odometry_callback(&mut self, msg: &Odometry, config: &SensorConfig) {
        let pose_vector = config.odom_pose_vector();
        let twist_vector = config.odom_twist_vector();
        if SensorConfig::sum(&pose_vector) > 0 {
            let pose = PoseStamped {
                header: msg.header.clone(),
                pose: msg.pose.clone(),
            };
            let source = if config.pose_use_child_frame {
                msg.child_frame_id.clone()
            } else {
                self.base_link_frame_id.clone()
            };
            let world = self.world_frame_id.clone();
            self.pose_callback(&pose, config, pose_vector, &world, &source, false);
        }

        if SensorConfig::sum(&twist_vector) > 0 {
            let mut twist = TwistStamped {
                header: msg.header.clone(),
                twist: msg.twist.clone(),
            };
            twist.header.frame_id = msg.child_frame_id.clone();
            let target = self.base_link_frame_id.clone();
            self.twist_callback(&twist, config, twist_vector, &target);
        }
    }

    /// `RosFilter::imuCallback`.
    pub fn imu_callback(&mut self, msg: &Imu, config: &SensorConfig) {
        let pose_vector = config.imu_pose_vector();
        let twist_vector = config.imu_twist_vector();
        let accel_vector = config.imu_accel_vector();

        if SensorConfig::sum(&pose_vector) > 0
            && (msg.orientation_covariance[0][0] + 1.0).abs() >= 1e-9
        {
            let mut pose = PoseStamped {
                header: msg.header.clone(),
                pose: PoseWithCovariance {
                    position: Vector3::zero(),
                    orientation: msg.orientation,
                    covariance: ZERO_MAT6,
                },
            };
            for i in 0..ORIENTATION_SIZE {
                for j in 0..ORIENTATION_SIZE {
                    pose.pose.covariance[i + ORIENTATION_SIZE][j + ORIENTATION_SIZE] =
                        msg.orientation_covariance[i][j];
                }
            }
            let base = self.base_link_frame_id.clone();
            self.pose_callback(&pose, config, pose_vector, &base, &base, true);
        }

        if SensorConfig::sum(&twist_vector) > 0
            && (msg.angular_velocity_covariance[0][0] + 1.0).abs() >= 1e-9
        {
            let mut twist = TwistStamped {
                header: msg.header.clone(),
                twist: TwistWithCovariance {
                    linear: Vector3::zero(),
                    angular: msg.angular_velocity,
                    covariance: ZERO_MAT6,
                },
            };
            for i in 0..ORIENTATION_SIZE {
                for j in 0..ORIENTATION_SIZE {
                    twist.twist.covariance[i + ORIENTATION_SIZE][j + ORIENTATION_SIZE] =
                        msg.angular_velocity_covariance[i][j];
                }
            }
            let base = self.base_link_frame_id.clone();
            self.twist_callback(&twist, config, twist_vector, &base);
        }

        if SensorConfig::sum(&accel_vector) > 0
            && (msg.linear_acceleration_covariance[0][0] + 1.0).abs() >= 1e-9
        {
            let base = self.base_link_frame_id.clone();
            self.acceleration_callback(msg, config, accel_vector, &base);
        }
    }

    /// `RosFilter::poseCallback`.
    #[allow(clippy::too_many_arguments)]
    fn pose_callback(
        &mut self,
        msg: &PoseStamped,
        config: &SensorConfig,
        update_vector: [bool; STATE_SIZE],
        target_frame: &str,
        pose_source_frame: &str,
        imu_data: bool,
    ) {
        let topic = config.topic_name.clone();
        let last = *self
            .last_message_times
            .entry(topic.clone())
            .or_insert(msg.header.stamp_ns);
        if last > msg.header.stamp_ns {
            return;
        }

        let mut measurement = [0.0f64; STATE_SIZE];
        let mut covariance = ZERO_MAT15;
        let mut update_vector = update_vector;

        if self.prepare_pose(
            msg,
            config,
            target_frame,
            pose_source_frame,
            imu_data,
            &mut update_vector,
            &mut measurement,
            &mut covariance,
        ) {
            let mut meas = Measurement::new(&topic, msg.header.stamp_ns);
            meas.measurement = measurement;
            meas.covariance = covariance;
            meas.update_vector = update_vector;
            meas.mahalanobis_thresh = config.rejection_threshold;
            self.enqueue_measurement(meas);
        }
        self.last_message_times.insert(topic, msg.header.stamp_ns);
    }

    /// `RosFilter::twistCallback`.
    fn twist_callback(
        &mut self,
        msg: &TwistStamped,
        config: &SensorConfig,
        update_vector: [bool; STATE_SIZE],
        target_frame: &str,
    ) {
        let topic = config.topic_name.clone();
        let last = *self
            .last_message_times
            .entry(topic.clone())
            .or_insert(msg.header.stamp_ns);
        if last > msg.header.stamp_ns {
            return;
        }

        let mut measurement = [0.0f64; STATE_SIZE];
        let mut covariance = ZERO_MAT15;
        let mut update_vector = update_vector;

        if self.prepare_twist(
            msg,
            target_frame,
            &mut update_vector,
            &mut measurement,
            &mut covariance,
        ) {
            let mut meas = Measurement::new(&topic, msg.header.stamp_ns);
            meas.measurement = measurement;
            meas.covariance = covariance;
            meas.update_vector = update_vector;
            meas.mahalanobis_thresh = config.rejection_threshold;
            self.enqueue_measurement(meas);
        }
        self.last_message_times.insert(topic, msg.header.stamp_ns);
    }

    /// `RosFilter::accelerationCallback`.
    fn acceleration_callback(
        &mut self,
        msg: &Imu,
        config: &SensorConfig,
        update_vector: [bool; STATE_SIZE],
        target_frame: &str,
    ) {
        let topic = config.topic_name.clone();
        let last = *self
            .last_message_times
            .entry(topic.clone())
            .or_insert(msg.header.stamp_ns);
        if last > msg.header.stamp_ns {
            return;
        }

        let mut measurement = [0.0f64; STATE_SIZE];
        let mut covariance = ZERO_MAT15;
        let mut update_vector = update_vector;

        if self.prepare_acceleration(
            msg,
            config,
            target_frame,
            &mut update_vector,
            &mut measurement,
            &mut covariance,
        ) {
            let mut meas = Measurement::new(&topic, msg.header.stamp_ns);
            meas.measurement = measurement;
            meas.covariance = covariance;
            meas.update_vector = update_vector;
            meas.mahalanobis_thresh = config.rejection_threshold;
            self.enqueue_measurement(meas);
        }
        self.last_message_times.insert(topic, msg.header.stamp_ns);
    }

    /// `RosFilter::prepareTwist`.
    #[allow(clippy::too_many_arguments)]
    fn prepare_twist(
        &mut self,
        msg: &TwistStamped,
        target_frame: &str,
        update_vector: &mut [bool; STATE_SIZE],
        measurement: &mut Vec15,
        measurement_covariance: &mut Mat15,
    ) -> bool {
        let mut twist_lin = msg.twist.linear;
        let mut meas_twist_rot = msg.twist.angular;

        let state = &self.filter.state;
        let state_twist_rot =
            Vector3::new(state[STATE_VROLL], state[STATE_VPITCH], state[STATE_VYAW]);

        let msg_frame = if msg.header.frame_id.is_empty() {
            target_frame.to_string()
        } else {
            msg.header.frame_id.clone()
        };

        let mut mask_lin = mask_matrix(
            update_vector[STATE_VX],
            update_vector[STATE_VY],
            update_vector[STATE_VZ],
        );
        let mut mask_rot = mask_matrix(
            update_vector[STATE_VROLL],
            update_vector[STATE_VPITCH],
            update_vector[STATE_VYAW],
        );

        let mut covariance_rotated = copy_covariance6(&msg.twist.covariance);

        let target_frame_trans = match self.transforms.lookup(target_frame, &msg_frame) {
            Some(t) => t,
            None => return false,
        };

        meas_twist_rot = target_frame_trans.basis.mul_vec(&meas_twist_rot);
        twist_lin = target_frame_trans
            .basis
            .mul_vec(&twist_lin)
            .add(&target_frame_trans.origin.cross(&state_twist_rot));
        mask_lin = target_frame_trans.basis.mul(&mask_lin);
        mask_rot = target_frame_trans.basis.mul(&mask_rot);

        update_vector[STATE_VX] = mask_lin.row(STATE_VX - POSITION_V_OFFSET).length() >= 1e-6;
        update_vector[STATE_VY] = mask_lin.row(STATE_VY - POSITION_V_OFFSET).length() >= 1e-6;
        update_vector[STATE_VZ] = mask_lin.row(STATE_VZ - POSITION_V_OFFSET).length() >= 1e-6;
        update_vector[STATE_VROLL] =
            mask_rot.row(STATE_VROLL - ORIENTATION_V_OFFSET).length() >= 1e-6;
        update_vector[STATE_VPITCH] =
            mask_rot.row(STATE_VPITCH - ORIENTATION_V_OFFSET).length() >= 1e-6;
        update_vector[STATE_VYAW] =
            mask_rot.row(STATE_VYAW - ORIENTATION_V_OFFSET).length() >= 1e-6;

        let rot = Matrix3x3::from_quaternion(&target_frame_trans.rotation());
        let r6 = rot6d(&rot);
        covariance_rotated = mat6_mul(&mat6_mul(&r6, &covariance_rotated), &mat6_transpose(&r6));

        measurement[STATE_VX] = twist_lin.x;
        measurement[STATE_VY] = twist_lin.y;
        measurement[STATE_VZ] = twist_lin.z;
        measurement[STATE_VROLL] = meas_twist_rot.x;
        measurement[STATE_VPITCH] = meas_twist_rot.y;
        measurement[STATE_VYAW] = meas_twist_rot.z;

        for i in 0..TWIST_SIZE {
            for j in 0..TWIST_SIZE {
                measurement_covariance[POSITION_V_OFFSET + i][POSITION_V_OFFSET + j] =
                    covariance_rotated[i][j];
            }
        }

        if self.two_d_mode {
            force_two_d(measurement, measurement_covariance, update_vector);
        }
        true
    }

    /// `RosFilter::prepareAcceleration`.
    #[allow(clippy::too_many_arguments)]
    fn prepare_acceleration(
        &mut self,
        msg: &Imu,
        config: &SensorConfig,
        target_frame: &str,
        update_vector: &mut [bool; STATE_SIZE],
        measurement: &mut Vec15,
        measurement_covariance: &mut Mat15,
    ) -> bool {
        let mut acc_tmp = msg.linear_acceleration;

        let msg_frame = if msg.header.frame_id.is_empty() {
            self.base_link_frame_id.clone()
        } else {
            msg.header.frame_id.clone()
        };

        let mut mask_acc = mask_matrix(
            update_vector[STATE_AX],
            update_vector[STATE_AY],
            update_vector[STATE_AZ],
        );

        let mut covariance_rotated = copy_covariance3(&msg.linear_acceleration_covariance);

        let target_frame_trans = match self.transforms.lookup(target_frame, &msg_frame) {
            Some(t) => t,
            None => return false,
        };

        let state = &self.filter.state;
        let state_twist_rot =
            Vector3::new(state[STATE_VROLL], state[STATE_VPITCH], state[STATE_VYAW]);
        acc_tmp = target_frame_trans
            .basis
            .mul_vec(&acc_tmp)
            .add(&target_frame_trans.origin.cross(&self.angular_acceleration))
            .sub(
                &target_frame_trans
                    .origin
                    .cross(&state_twist_rot)
                    .cross(&state_twist_rot),
            );

        if config.remove_gravitational_acceleration {
            let norm_acc = Vector3::new(0.0, 0.0, self.gravitational_acceleration);
            let rot_norm;
            if (msg.orientation_covariance[0][0] + 1.0).abs() < 1e-9 {
                let state_tmp = Matrix3x3::from_rpy(
                    self.filter.state[STATE_ROLL],
                    self.filter.state[STATE_PITCH],
                    self.filter.state[STATE_YAW],
                );
                let basis = state_tmp.mul(&target_frame_trans.basis);
                rot_norm = basis.inverse().mul_vec(&norm_acc);
            } else {
                let mut cur_attitude = msg.orientation;
                if (cur_attitude.length() - 1.0).abs() > 0.01 {
                    cur_attitude = cur_attitude.normalized();
                }
                let basis = Matrix3x3::from_quaternion(&cur_attitude);
                if !config.relative {
                    rot_norm = basis.inverse().mul_vec(&norm_acc);
                } else {
                    rot_norm = target_frame_trans
                        .basis
                        .inverse()
                        .mul(&basis.inverse())
                        .mul_vec(&norm_acc);
                }
            }
            acc_tmp = acc_tmp.sub(&rot_norm);
        }

        mask_acc = target_frame_trans.basis.mul(&mask_acc);
        update_vector[STATE_AX] = mask_acc.row(STATE_AX - POSITION_A_OFFSET).length() >= 1e-6;
        update_vector[STATE_AY] = mask_acc.row(STATE_AY - POSITION_A_OFFSET).length() >= 1e-6;
        update_vector[STATE_AZ] = mask_acc.row(STATE_AZ - POSITION_A_OFFSET).length() >= 1e-6;

        let rot = Matrix3x3::from_quaternion(&target_frame_trans.rotation());
        let r3 = rot3d(&rot);
        let mut r3t = ZERO_MAT3;
        for i in 0..3 {
            for j in 0..3 {
                r3t[i][j] = r3[j][i];
            }
        }
        covariance_rotated = mat3_mul(&mat3_mul(&r3, &covariance_rotated), &r3t);

        measurement[STATE_AX] = acc_tmp.x;
        measurement[STATE_AY] = acc_tmp.y;
        measurement[STATE_AZ] = acc_tmp.z;
        for i in 0..ACCELERATION_SIZE {
            for j in 0..ACCELERATION_SIZE {
                measurement_covariance[POSITION_A_OFFSET + i][POSITION_A_OFFSET + j] =
                    covariance_rotated[i][j];
            }
        }

        if self.two_d_mode {
            force_two_d(measurement, measurement_covariance, update_vector);
        }
        true
    }

    /// `RosFilter::preparePose`.
    #[allow(clippy::too_many_arguments)]
    fn prepare_pose(
        &mut self,
        msg: &PoseStamped,
        config: &SensorConfig,
        target_frame: &str,
        source_frame: &str,
        imu_data: bool,
        update_vector: &mut [bool; STATE_SIZE],
        measurement: &mut Vec15,
        measurement_covariance: &mut Mat15,
    ) -> bool {
        let differential = config.differential;
        let relative = config.relative;
        let topic_name = config.topic_name.clone();

        let final_target_frame;
        let pose_frame_id;
        if target_frame.is_empty() {
            if msg.header.frame_id.is_empty() {
                final_target_frame = self.world_frame_id.clone();
            } else {
                final_target_frame = msg.header.frame_id.clone();
            }
            pose_frame_id = final_target_frame.clone();
        } else {
            final_target_frame = target_frame.to_string();
            pose_frame_id = if differential && !imu_data {
                final_target_frame.clone()
            } else {
                msg.header.frame_id.clone()
            };
        }

        let mut orientation = msg.pose.orientation;
        if orientation.x == 0.0
            && orientation.y == 0.0
            && orientation.z == 0.0
            && orientation.w == 0.0
        {
            orientation = Quaternion::identity();
        } else if (orientation.length() - 1.0).abs() > 0.01 {
            orientation = orientation.normalized();
        }

        let mut pose_tmp = Transform::from_parts(&orientation, msg.pose.position);

        let mut target_frame_trans =
            match self.transforms.lookup(&final_target_frame, &pose_frame_id) {
                Some(t) => t,
                None => return false,
            };

        let mut source_frame_trans = Transform::identity();
        let mut can_src_transform = false;
        if source_frame != self.base_link_frame_id {
            if let Some(t) = self
                .transforms
                .lookup(source_frame, &self.base_link_frame_id)
            {
                source_frame_trans = t;
                can_src_transform = true;
            }
        }

        let mut mask_position = mask_matrix(
            update_vector[STATE_X],
            update_vector[STATE_Y],
            update_vector[STATE_Z],
        );
        let mut mask_orientation = mask_matrix(
            update_vector[STATE_ROLL],
            update_vector[STATE_PITCH],
            update_vector[STATE_YAW],
        );

        if imu_data {
            let (_, _, yaw) = target_frame_trans.basis.get_rpy();
            let trans_tmp = Matrix3x3::from_rpy(0.0, 0.0, yaw);
            mask_position = trans_tmp.mul(&mask_position);
            mask_orientation = trans_tmp.mul(&mask_orientation);
        } else {
            mask_position = target_frame_trans.basis.mul(&mask_position);
            mask_orientation = target_frame_trans.basis.mul(&mask_orientation);
        }

        update_vector[STATE_X] = mask_position.row(STATE_X - POSITION_OFFSET).length() >= 1e-6;
        update_vector[STATE_Y] = mask_position.row(STATE_Y - POSITION_OFFSET).length() >= 1e-6;
        update_vector[STATE_Z] = mask_position.row(STATE_Z - POSITION_OFFSET).length() >= 1e-6;
        update_vector[STATE_ROLL] = mask_orientation
            .row(STATE_ROLL - ORIENTATION_OFFSET)
            .length()
            >= 1e-6;
        update_vector[STATE_PITCH] = mask_orientation
            .row(STATE_PITCH - ORIENTATION_OFFSET)
            .length()
            >= 1e-6;
        update_vector[STATE_YAW] = mask_orientation
            .row(STATE_YAW - ORIENTATION_OFFSET)
            .length()
            >= 1e-6;

        let mut covariance = copy_covariance6(&msg.pose.covariance);

        if can_src_transform {
            let rot = Matrix3x3::from_quaternion(&source_frame_trans.rotation());
            let r6 = rot6d(&rot);
            covariance = mat6_mul(&mat6_mul(&mat6_transpose(&r6), &covariance), &r6);
        }

        let rot = if imu_data {
            let (_, _, yaw) = target_frame_trans.basis.get_rpy();
            Matrix3x3::from_rpy(0.0, 0.0, yaw)
        } else {
            Matrix3x3::from_quaternion(&target_frame_trans.rotation())
        };
        let r6 = rot6d(&rot);
        let mut covariance_rotated = mat6_mul(&mat6_mul(&r6, &covariance), &mat6_transpose(&r6));

        if imu_data {
            let (roll_offset, pitch_offset, yaw_offset) =
                Matrix3x3::from_quaternion(&target_frame_trans.rotation()).get_rpy();
            let (roll, pitch, yaw) = Matrix3x3::from_quaternion(&pose_tmp.rotation()).get_rpy();
            let rpy_angles = Vector3::new(
                normalize_angle(roll - roll_offset),
                normalize_angle(pitch - pitch_offset),
                normalize_angle(yaw - yaw_offset),
            );
            let mat = Matrix3x3::from_rpy(0.0, 0.0, yaw_offset);
            let rpy_angles = mat.mul_vec(&rpy_angles);
            pose_tmp.basis = Matrix3x3::from_rpy(rpy_angles.x, rpy_angles.y, rpy_angles.z);
            target_frame_trans = Transform::identity();
        }

        if differential {
            let cur_measurement = pose_tmp;
            let mut success = false;

            if self.previous_measurements.contains_key(&topic_name)
                && self
                    .previous_measurement_covariances
                    .contains_key(&topic_name)
            {
                let prev_measurement = self.previous_measurements[&topic_name];
                pose_tmp = prev_measurement.inverse_times(&pose_tmp);

                target_frame_trans.origin = Vector3::zero();
                pose_tmp = target_frame_trans.mul(&pose_tmp);

                let last_time = *self.last_message_times.get(&topic_name).unwrap_or(&0);
                let dt = (msg.header.stamp_ns as f64) * 1e-9 - (last_time as f64) * 1e-9;
                let x_vel = pose_tmp.origin.x / dt;
                let y_vel = pose_tmp.origin.y / dt;
                let z_vel = pose_tmp.origin.z / dt;
                let (mut roll_vel, mut pitch_vel, mut yaw_vel) =
                    Matrix3x3::from_quaternion(&pose_tmp.rotation()).get_rpy();
                roll_vel /= dt;
                pitch_vel /= dt;
                yaw_vel /= dt;

                let mut twist = TwistStamped {
                    header: msg.header.clone(),
                    twist: TwistWithCovariance {
                        linear: Vector3::new(x_vel, y_vel, z_vel),
                        angular: Vector3::new(roll_vel, pitch_vel, yaw_vel),
                        covariance: ZERO_MAT6,
                    },
                };
                twist.header.frame_id = source_frame.to_string();

                let mut twist_update_vec = [false; STATE_SIZE];
                for i in 0..POSE_SIZE {
                    twist_update_vec[POSITION_V_OFFSET + i] = update_vector[POSITION_OFFSET + i];
                }
                *update_vector = twist_update_vec;

                let prev_cov = self.previous_measurement_covariances[&topic_name];
                let prev_covar_rotated = mat6_mul(&mat6_mul(&r6, &prev_cov), &mat6_transpose(&r6));
                for i in 0..POSE_SIZE {
                    for j in 0..POSE_SIZE {
                        covariance_rotated[i][j] =
                            (covariance_rotated[i][j] + prev_covar_rotated[i][j]) * dt;
                    }
                }
                twist.twist.covariance = covariance_rotated;

                let base = self.base_link_frame_id.clone();
                success = self.prepare_twist(
                    &twist,
                    &base,
                    update_vector,
                    measurement,
                    measurement_covariance,
                );
            }

            self.previous_measurements
                .insert(topic_name.clone(), cur_measurement);
            self.previous_measurement_covariances
                .insert(topic_name, covariance);
            return success;
        }

        if can_src_transform {
            pose_tmp = pose_tmp.mul(&source_frame_trans);
        }

        if relative {
            let initial = *self
                .initial_measurements
                .entry(topic_name)
                .or_insert(pose_tmp);
            pose_tmp = initial.inverse_times(&pose_tmp);
        }

        pose_tmp = target_frame_trans.mul(&pose_tmp);

        measurement[STATE_X] = pose_tmp.origin.x;
        measurement[STATE_Y] = pose_tmp.origin.y;
        measurement[STATE_Z] = pose_tmp.origin.z;
        let (roll, pitch, yaw) = Matrix3x3::from_quaternion(&pose_tmp.rotation()).get_rpy();
        measurement[STATE_ROLL] = roll;
        measurement[STATE_PITCH] = pitch;
        measurement[STATE_YAW] = yaw;

        for i in 0..POSE_SIZE {
            for j in 0..POSE_SIZE {
                measurement_covariance[i][j] = covariance_rotated[i][j];
            }
        }

        if self.two_d_mode {
            force_two_d(measurement, measurement_covariance, update_vector);
        }
        true
    }

    /// `RosFilter::integrateMeasurements`, without the lagged-data history
    /// (`smooth_lagged_data` is off in this robot's yaml).
    pub fn integrate_measurements(&mut self, current_time_ns: i64) {
        let mut predict_to_current_time = self.predict_to_current_time;

        if !self.queue.is_empty() {
            while !self.queue.is_empty() {
                if current_time_ns < self.queue[0].0 {
                    break;
                }
                let (_, _, meas) = self.queue.remove(0);
                self.filter.process_measurement(&meas);
            }
        } else if self.filter.initialized {
            let last_update_delta = current_time_ns - self.filter.last_measurement_time_ns;
            if last_update_delta >= self.filter.sensor_timeout_ns {
                predict_to_current_time = true;
            }
        }

        if self.filter.initialized && predict_to_current_time {
            let last_update_delta = current_time_ns - self.filter.last_measurement_time_ns;
            self.filter.predict(current_time_ns, last_update_delta);
            self.filter.last_measurement_time_ns += last_update_delta;
        }
    }

    /// `RosFilter::differentiateMeasurements`.
    pub fn differentiate_measurements(&mut self, current_time_ns: i64) {
        if self.filter.initialized {
            let time_now = current_time_ns as f64 * 1e-9;
            let dt = time_now - self.last_diff_time;
            let state = &self.filter.state;
            let new_state_twist_rot =
                Vector3::new(state[STATE_VROLL], state[STATE_VPITCH], state[STATE_VYAW]);
            self.angular_acceleration = new_state_twist_rot
                .sub(&self.last_state_twist_rot)
                .scale(1.0 / dt);
            self.last_state_twist_rot = new_state_twist_rot;
            self.last_diff_time = time_now;
        }
    }

    /// `RosFilter::getFilteredOdometryMessage`.
    pub fn filtered_odometry(&self) -> Option<Odometry> {
        if !self.filter.initialized {
            return None;
        }
        let state = &self.filter.state;
        let p = &self.filter.estimate_error_covariance;
        let quat = Quaternion::from_rpy(state[STATE_ROLL], state[STATE_PITCH], state[STATE_YAW]);
        let mut odom = Odometry {
            header: Header {
                frame_id: self.world_frame_id.clone(),
                stamp_ns: self.filter.last_measurement_time_ns,
            },
            child_frame_id: self.base_link_output_frame_id.clone(),
            ..Default::default()
        };
        odom.pose.position = Vector3::new(state[STATE_X], state[STATE_Y], state[STATE_Z]);
        odom.pose.orientation = quat;
        odom.twist.linear = Vector3::new(state[STATE_VX], state[STATE_VY], state[STATE_VZ]);
        odom.twist.angular =
            Vector3::new(state[STATE_VROLL], state[STATE_VPITCH], state[STATE_VYAW]);
        for i in 0..POSE_SIZE {
            for j in 0..POSE_SIZE {
                odom.pose.covariance[i][j] = p[i][j];
                odom.twist.covariance[i][j] = p[i + POSITION_V_OFFSET][j + POSITION_V_OFFSET];
            }
        }
        Some(odom)
    }

    /// The `map -> odom` transform `periodicUpdate` broadcasts when the filter's
    /// world frame is `map`: `world_base_link_trans * base_link_odom_trans`.
    pub fn map_to_odom(&self) -> Option<Transform> {
        let odom = self.filtered_odometry()?;
        if odom.header.frame_id != self.map_frame_id {
            return None;
        }
        let world_base_link = Transform::from_parts(&odom.pose.orientation, odom.pose.position);
        let base_link_odom = self
            .transforms
            .lookup(&self.base_link_frame_id, &self.odom_frame_id)?;
        Some(world_base_link.mul(&base_link_odom))
    }
}
