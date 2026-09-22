//! One `diff_drive_controller` update cycle, without ROS.
//!
//! Port of `ros2_controllers/diff_drive_controller/src/diff_drive_controller.cpp`
//! (jazzy): `update_reference_from_subscribers()` (the cmd_vel timeout) and
//! `update_and_write_commands()` (speed limits, odometry feed, publish gating,
//! wheel velocity commands), parameterised from
//! `src/mower_hardware/config/mower_controllers.yaml`.

use crate::limiter::RateLimiter;
use crate::odometry::Odometry;
use crate::{seconds, TimeNs};

/// One axis' worth of `linear.x` / `angular.z` parameters.
///
/// `has_*_limits` are deprecated upstream: when false the controller
/// overwrites the corresponding limits with NaN (= no limit) *before*
/// building the limiter, which is what [`LimitParams::limiter`] does.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LimitParams {
    pub has_velocity_limits: bool,
    pub has_acceleration_limits: bool,
    pub has_jerk_limits: bool,
    pub min_velocity: f64,
    pub max_velocity: f64,
    pub max_acceleration: f64,
    pub max_acceleration_reverse: f64,
    pub max_deceleration: f64,
    pub max_deceleration_reverse: f64,
    pub min_jerk: f64,
    pub max_jerk: f64,
}

impl Default for LimitParams {
    /// The generated-parameter defaults: everything off, every bound NaN.
    fn default() -> Self {
        Self {
            has_velocity_limits: false,
            has_acceleration_limits: false,
            has_jerk_limits: false,
            min_velocity: f64::NAN,
            max_velocity: f64::NAN,
            max_acceleration: f64::NAN,
            max_acceleration_reverse: f64::NAN,
            max_deceleration: f64::NAN,
            max_deceleration_reverse: f64::NAN,
            min_jerk: f64::NAN,
            max_jerk: f64::NAN,
        }
    }
}

impl LimitParams {
    pub fn limiter(&self) -> Result<RateLimiter, String> {
        let nan = f64::NAN;
        let (min_velocity, max_velocity) = if self.has_velocity_limits {
            (self.min_velocity, self.max_velocity)
        } else {
            (nan, nan)
        };
        let (max_acceleration, max_acceleration_reverse, max_deceleration, max_deceleration_reverse) =
            if self.has_acceleration_limits {
                (
                    self.max_acceleration,
                    self.max_acceleration_reverse,
                    self.max_deceleration,
                    self.max_deceleration_reverse,
                )
            } else {
                (nan, nan, nan, nan)
            };
        let (min_jerk, max_jerk) = if self.has_jerk_limits {
            (self.min_jerk, self.max_jerk)
        } else {
            (nan, nan)
        };
        RateLimiter::new(
            min_velocity,
            max_velocity,
            max_acceleration_reverse,
            max_acceleration,
            max_deceleration,
            max_deceleration_reverse,
            min_jerk,
            max_jerk,
        )
    }
}

/// `diff_drive_controller` parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct DiffDriveParams {
    pub wheel_separation: f64,
    pub wheel_radius: f64,
    pub wheel_separation_multiplier: f64,
    pub left_wheel_radius_multiplier: f64,
    pub right_wheel_radius_multiplier: f64,
    /// Hz; gates /odom and the odom->base_link transform.
    pub publish_rate: f64,
    /// s; 0.0 disables the timeout.
    pub cmd_vel_timeout: f64,
    pub open_loop: bool,
    /// true: wheel *positions* feed the odometry (`Odometry::update`).
    pub position_feedback: bool,
    pub enable_odom_tf: bool,
    pub velocity_rolling_window_size: usize,
    pub odom_frame_id: String,
    pub base_frame_id: String,
    pub pose_covariance_diagonal: [f64; 6],
    pub twist_covariance_diagonal: [f64; 6],
    pub linear: LimitParams,
    pub angular: LimitParams,
}

impl DiffDriveParams {
    /// Exactly `src/mower_hardware/config/mower_controllers.yaml`.
    pub fn mower() -> Self {
        Self {
            wheel_separation: 0.40,
            wheel_radius: 0.10,
            wheel_separation_multiplier: 1.0,
            left_wheel_radius_multiplier: 1.0,
            right_wheel_radius_multiplier: 1.0,
            publish_rate: 25.0,
            cmd_vel_timeout: 0.5,
            open_loop: false,
            position_feedback: true,
            enable_odom_tf: true,
            velocity_rolling_window_size: 10,
            odom_frame_id: "odom".to_string(),
            base_frame_id: "base_link".to_string(),
            pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01],
            twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01],
            // 58 rpm max wheel speed -> 6.07 rad/s at the wheel. With
            // r = 0.10 m that is 0.61 m/s linear, ~3.0 rad/s yaw at 0.40 m.
            linear: LimitParams {
                has_velocity_limits: true,
                has_acceleration_limits: true,
                has_jerk_limits: false,
                min_velocity: -0.55,
                max_velocity: 0.55,
                max_acceleration: 0.8,
                ..LimitParams::default()
            },
            angular: LimitParams {
                has_velocity_limits: true,
                has_acceleration_limits: true,
                has_jerk_limits: false,
                min_velocity: -2.0,
                max_velocity: 2.0,
                max_acceleration: 2.0,
                ..LimitParams::default()
            },
        }
    }

    pub fn effective_wheel_separation(&self) -> f64 {
        self.wheel_separation_multiplier * self.wheel_separation
    }
    pub fn effective_left_wheel_radius(&self) -> f64 {
        self.left_wheel_radius_multiplier * self.wheel_radius
    }
    pub fn effective_right_wheel_radius(&self) -> f64 {
        self.right_wheel_radius_multiplier * self.wheel_radius
    }
}

/// `geometry_msgs/Twist`, the two components a diff drive listens to.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Twist {
    pub linear_x: f64,
    pub angular_z: f64,
}

impl Twist {
    pub fn new(linear_x: f64, angular_z: f64) -> Self {
        Self { linear_x, angular_z }
    }
}

/// Wheel velocity commands, rad/s at the wheel.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct WheelCommand {
    pub left: f64,
    pub right: f64,
}

/// What `/odom` (and the odom->base_link transform) carries.
#[derive(Debug, Clone, PartialEq)]
pub struct OdomSample {
    pub stamp_ns: TimeNs,
    pub frame_id: String,
    pub child_frame_id: String,
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    /// Quaternion from yaw: x = y = 0, z = sin(yaw/2), w = cos(yaw/2),
    /// which is `tf2::Quaternion::setRPY(0, 0, yaw)`.
    pub qz: f64,
    pub qw: f64,
    pub linear_x: f64,
    pub angular_z: f64,
    /// Row-major 6x6, only the diagonal is set (indices 0, 7, 14, 21, 28, 35).
    pub pose_covariance: [f64; 36],
    pub twist_covariance: [f64; 36],
    /// `enable_odom_tf`: whether the odom->base_link transform goes out too.
    pub publish_tf: bool,
}

/// Result of one `update_and_write_commands()`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CycleOutput {
    /// `None` when the reference is still NaN (no command yet): the C++
    /// returns early and leaves the wheel command interfaces untouched.
    pub wheel: Option<WheelCommand>,
    /// `Some` only on the cycles the publish rate lets through.
    pub odom: Option<OdomSample>,
    /// The speed-limited command actually used this cycle.
    pub linear_command: f64,
    pub angular_command: f64,
}

/// The stateful half of `DiffDriveController`.
#[derive(Debug, Clone)]
pub struct DiffDrive {
    params: DiffDriveParams,
    limiter_linear: RateLimiter,
    limiter_angular: RateLimiter,
    odometry: Odometry,
    /// `previous_two_commands_`: [older, newer] of [linear, angular].
    prev_front: [f64; 2],
    prev_back: [f64; 2],
    /// The exported reference interfaces. NaN until the first command.
    ref_linear: f64,
    ref_angular: f64,
    /// `command_msg_`: last received command and its stamp.
    cmd: Twist,
    cmd_stamp_ns: TimeNs,
    command_timed_out: bool,
    previous_publish_ns: TimeNs,
    publish_period_ns: i64,
    pose_covariance: [f64; 36],
    twist_covariance: [f64; 36],
}

impl DiffDrive {
    /// `on_configure` + `on_activate`. `now` is what `previous_publish_timestamp_`
    /// and the NaN command's stamp are seeded with.
    pub fn new(params: DiffDriveParams, now: TimeNs) -> Result<Self, String> {
        let limiter_linear = params.linear.limiter()?;
        let limiter_angular = params.angular.limiter()?;

        let mut odometry = Odometry::new(params.velocity_rolling_window_size);
        odometry.set_wheel_params(
            params.effective_wheel_separation(),
            params.effective_left_wheel_radius(),
            params.effective_right_wheel_radius(),
        );
        odometry.set_velocity_rolling_window_size(params.velocity_rolling_window_size);

        let publish_period_ns = duration_from_seconds_ns(1.0 / params.publish_rate);

        let mut pose_covariance = [0.0f64; 36];
        let mut twist_covariance = [0.0f64; 36];
        for index in 0..6 {
            let diagonal_index = 6 * index + index;
            pose_covariance[diagonal_index] = params.pose_covariance_diagonal[index];
            twist_covariance[diagonal_index] = params.twist_covariance_diagonal[index];
        }

        Ok(Self {
            params,
            limiter_linear,
            limiter_angular,
            odometry,
            // reset_buffers(): zeros, "not NaN, to catch early accelerations"
            prev_front: [0.0, 0.0],
            prev_back: [0.0, 0.0],
            ref_linear: f64::NAN,
            ref_angular: f64::NAN,
            cmd: Twist::new(f64::NAN, f64::NAN),
            cmd_stamp_ns: now,
            command_timed_out: false,
            previous_publish_ns: now,
            publish_period_ns,
            pose_covariance,
            twist_covariance,
        })
    }

    pub fn params(&self) -> &DiffDriveParams {
        &self.params
    }
    pub fn odometry(&self) -> &Odometry {
        &self.odometry
    }
    pub fn command_timed_out(&self) -> bool {
        self.command_timed_out
    }

    /// `update_reference_from_subscribers()`.
    ///
    /// `cmd` is a command that arrived this cycle (its stamp is `now`;
    /// the controller drops anything older than `cmd_vel_timeout` at the
    /// subscription, so a stamp in the past never reaches here).
    /// `None` means nothing new arrived and the stored command ages.
    pub fn update_reference(&mut self, now: TimeNs, cmd: Option<Twist>) {
        if let Some(c) = cmd {
            self.cmd = c;
            self.cmd_stamp_ns = now;
        }
        let age_of_last_command = seconds(now) - seconds(self.cmd_stamp_ns);
        let cmd_vel_timeout_disabled = self.params.cmd_vel_timeout == 0.0;
        // Brake if cmd_vel has timed out, overriding the stored command.
        if !cmd_vel_timeout_disabled && age_of_last_command > self.params.cmd_vel_timeout {
            self.ref_linear = 0.0;
            self.ref_angular = 0.0;
            self.command_timed_out = true;
        } else if self.cmd.linear_x.is_finite() && self.cmd.angular_z.is_finite() {
            self.command_timed_out = false;
            self.ref_linear = self.cmd.linear_x;
            self.ref_angular = self.cmd.angular_z;
        }
        // else: NaNs in the message, reference interfaces left alone.
    }

    /// `update_and_write_commands()`. `period_s` is the control period the
    /// caller measured (what ros2_control passes in).
    pub fn update_and_write(
        &mut self,
        now: TimeNs,
        period_s: f64,
        left_feedback: f64,
        right_feedback: f64,
    ) -> CycleOutput {
        let mut linear_command = self.ref_linear;
        let mut angular_command = self.ref_angular;

        // NaNs occur on initialization when the reference interfaces are unset
        if !linear_command.is_finite() || !angular_command.is_finite() {
            return CycleOutput {
                wheel: None,
                odom: None,
                linear_command,
                angular_command,
            };
        }

        self.limiter_linear.limit(
            &mut linear_command,
            self.prev_back[0],
            self.prev_front[0],
            period_s,
        );
        self.limiter_angular.limit(
            &mut angular_command,
            self.prev_back[1],
            self.prev_front[1],
            period_s,
        );
        self.prev_front = self.prev_back;
        self.prev_back = [linear_command, angular_command];

        let wheel_separation = self.params.effective_wheel_separation();
        let left_wheel_radius = self.params.effective_left_wheel_radius();
        let right_wheel_radius = self.params.effective_right_wheel_radius();

        if self.params.open_loop {
            self.odometry
                .update_open_loop(linear_command, angular_command, now);
        } else if self.params.position_feedback {
            self.odometry.update(left_feedback, right_feedback, now);
        } else {
            self.odometry.update_from_velocity(
                left_feedback * left_wheel_radius * period_s,
                right_feedback * right_wheel_radius * period_s,
                now,
            );
        }

        let should_publish = if self.previous_publish_ns + self.publish_period_ns < now {
            self.previous_publish_ns += self.publish_period_ns;
            true
        } else {
            false
        };

        let odom = should_publish.then(|| {
            let yaw = self.odometry.heading();
            OdomSample {
                stamp_ns: now,
                frame_id: self.params.odom_frame_id.clone(),
                child_frame_id: self.params.base_frame_id.clone(),
                x: self.odometry.x(),
                y: self.odometry.y(),
                yaw,
                qz: (yaw * 0.5).sin(),
                qw: (yaw * 0.5).cos(),
                linear_x: self.odometry.linear(),
                angular_z: self.odometry.angular(),
                pose_covariance: self.pose_covariance,
                twist_covariance: self.twist_covariance,
                publish_tf: self.params.enable_odom_tf,
            }
        });

        // Compute wheels velocities:
        let velocity_left =
            (linear_command - angular_command * wheel_separation / 2.0) / left_wheel_radius;
        let velocity_right =
            (linear_command + angular_command * wheel_separation / 2.0) / right_wheel_radius;

        CycleOutput {
            wheel: Some(WheelCommand {
                left: velocity_left,
                right: velocity_right,
            }),
            odom,
            linear_command,
            angular_command,
        }
    }

    /// `on_deactivate`: halt the wheels and empty the command buffers.
    pub fn halt(&mut self) {
        self.prev_front = [0.0, 0.0];
        self.prev_back = [0.0, 0.0];
        self.ref_linear = f64::NAN;
        self.ref_angular = f64::NAN;
        self.cmd = Twist::new(f64::NAN, f64::NAN);
        self.command_timed_out = false;
    }
}

/// `rclcpp::Duration::from_seconds`: truncates towards zero into int64 ns.
fn duration_from_seconds_ns(seconds: f64) -> i64 {
    (seconds * 1e9) as i64
}
