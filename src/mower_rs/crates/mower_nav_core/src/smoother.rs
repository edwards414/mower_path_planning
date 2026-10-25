//! `nav2_velocity_smoother::VelocitySmoother` (D4) as the `velocity_smoother`
//! section of `nav2_no_map_params.yaml` configures it: OPEN_LOOP feedback,
//! 20 Hz, `scale_velocities: false`, zero deadband, a 1 s timeout and the
//! input command's own stamp on the output
//! (`stamp_smoothed_velocity_with_smoothing_time: false`).
//!
//! The node is two callbacks around one state:
//! [`VelocitySmoother::input_command`] is `inputCommandStampedCallback`
//! (`cmd_vel`, `enable_stamped_cmd_vel: true`) and [`VelocitySmoother::tick`]
//! is one firing of the `smootherTimer` wall timer, returning what it
//! publishes on `cmd_vel_smoothed`, if anything. Times are the node's ROS
//! clock (`now()`) in nanoseconds; the caller fires `tick` every
//! [`VelocitySmoother::timer_period_ns`] of wall time. The per-tick velocity
//! step is `accel / smoothing_frequency` whatever the real interval.
//!
//! Stamps and the timeout:
//! * every tick after the first command publishes, replaying the latest
//!   command, with that command's `header.stamp` and frame, until the
//!   timeout. A zero stamp (unset) stays zero on the output; only the
//!   timeout clock (`last_command_time_`) falls back to the arrival `now()`;
//! * the timeout runs from the command's stamp, not from its arrival, and is
//!   strict (`now - stamp > velocity_timeout`): a stamp more than 1 s old
//!   times out on the next tick, one in the future postpones it by as much;
//! * on timeout the stored command becomes a zero twist stamped `now()` with
//!   an empty frame. The first timed-out tick still carries the old command's
//!   stamp and frame, each later one the previous tick's `now()` and "". The
//!   ramp continues until the open-loop velocity is exactly zero (that zero
//!   is published once), then nothing is published until a new command. A
//!   robot already at rest publishes nothing at all on timeout, so a stale
//!   command arriving at rest is never published;
//! * a zero command (controller_server's `publishZeroVelocity`) is ramped
//!   to, then republished as zeros with its own stamp until it times out.
//!
//! With `stamp_smoothed_velocity_with_smoothing_time: true` the stamp is
//! instead `command stamp + (now - last_command_time_)`.
//!
//! Upstream: `nav2_velocity_smoother/src/velocity_smoother.cpp`,
//! `nav2_util::validateTwist` (`nav2_util/src/robot_utils.cpp`).
//!
//! Not ported: `feedback: CLOSED_LOOP` (`nav2_util::OdomSmoother` on
//! `odom_topic` over `odom_duration`), `dynamicParametersCallback`, the
//! unstamped `Twist` publisher (`enable_stamped_cmd_vel: false`),
//! `use_realtime_priority`, the lifecycle and bond. With this robot's
//! parameters the `scale_velocities` branch (`findEtaConstraint`), the
//! deadband and `stamp_smoothed_velocity_with_smoothing_time: true` are not
//! reached; they are ported and checked against the oracle with other
//! parameters.

use crate::geometry::Twist2D;

/// The smoother's ROS parameters. Arrays are (x, y, theta).
#[derive(Debug, Clone, PartialEq)]
pub struct VelocitySmootherParams {
    /// Hz: the timer rate and the divisor of every acceleration limit.
    pub smoothing_frequency: f64,
    pub scale_velocities: bool,
    pub max_velocity: [f64; 3],
    pub min_velocity: [f64; 3],
    /// >= 0
    pub max_accel: [f64; 3],
    /// <= 0
    pub max_decel: [f64; 3],
    pub deadband_velocity: [f64; 3],
    /// seconds
    pub velocity_timeout: f64,
    pub stamp_smoothed_velocity_with_smoothing_time: bool,
}

impl Default for VelocitySmootherParams {
    /// `on_configure`'s `declare_parameter_if_not_declared` defaults
    /// (`feedback` defaults to OPEN_LOOP, the only mode ported).
    fn default() -> Self {
        Self {
            smoothing_frequency: 20.0,
            scale_velocities: false,
            max_velocity: [0.50, 0.0, 2.5],
            min_velocity: [-0.50, 0.0, -2.5],
            max_accel: [2.5, 0.0, 3.2],
            max_decel: [-2.5, 0.0, -3.2],
            deadband_velocity: [0.0, 0.0, 0.0],
            velocity_timeout: 1.0,
            stamp_smoothed_velocity_with_smoothing_time: false,
        }
    }
}

impl VelocitySmootherParams {
    /// The `velocity_smoother` section of `nav2_no_map_params.yaml`.
    pub fn mower() -> Self {
        Self {
            smoothing_frequency: 20.0,
            scale_velocities: false,
            max_velocity: [0.26, 0.0, 1.0],
            min_velocity: [-0.26, 0.0, -1.0],
            max_accel: [2.5, 0.0, 3.2],
            max_decel: [-2.5, 0.0, -3.2],
            deadband_velocity: [0.0, 0.0, 0.0],
            velocity_timeout: 1.0,
            stamp_smoothed_velocity_with_smoothing_time: false,
        }
    }
}

/// Why `on_configure` returns FAILURE; the axis (0 x, 1 y, 2 theta) is the
/// first one that fails, checked in this order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError {
    PositiveDecel(usize),
    NegativeAccel(usize),
    PositiveMinVelocity(usize),
    NegativeMaxVelocity(usize),
    /// Cannot happen once the sign checks before it pass; kept for parity.
    MinAboveMax(usize),
}

/// Why a `cmd_vel` message changed nothing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputError {
    /// `validateTwist`: a NaN or infinity in any of the six fields
    /// ("Velocity message contains NaNs or Infs! Ignoring as invalid!").
    NonFinite,
    /// A negative `header.stamp.sec`. Upstream stores the command and then
    /// `rclcpp::Time` throws out of the callback, which takes the node down;
    /// here the message is dropped instead.
    NegativeStamp,
}

/// `geometry_msgs/TwistStamped` on `cmd_vel` and `cmd_vel_smoothed`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct TwistStamped {
    /// `header.stamp` as `sec * 1e9 + nanosec`; 0 is the unset stamp.
    pub stamp_ns: i64,
    pub frame_id: String,
    /// `linear.x`, `linear.y`, `angular.z`.
    pub twist: Twist2D,
    /// `linear.z`, `angular.x`, `angular.y`: never used, but a NaN or
    /// infinity here also drops the message. Always zero on the output.
    pub other: [f64; 3],
}

/// `nav2_velocity_smoother::VelocitySmoother`, configured and active.
#[derive(Debug, Clone)]
pub struct VelocitySmoother {
    p: VelocitySmootherParams,
    velocity_timeout_ns: i64,
    /// `command_`: the latest valid command, rewritten in place by the timer
    /// (clamped to the velocity limits, zeroed on timeout).
    command: Option<TwistStamped>,
    /// `last_command_time_`
    last_command_time_ns: i64,
    /// `last_cmd_.twist`: the last smoothed velocity before the deadband,
    /// which is the open-loop feedback.
    last_cmd: Twist2D,
    stopped: bool,
}

impl VelocitySmoother {
    /// `on_configure`'s parameter checks, then the state `on_activate`
    /// leaves (no command yet, `stopped_` true, zero last command).
    pub fn new(params: VelocitySmootherParams) -> Result<Self, ConfigError> {
        for i in 0..3 {
            if params.max_decel[i] > 0.0 {
                return Err(ConfigError::PositiveDecel(i));
            }
            if params.max_accel[i] < 0.0 {
                return Err(ConfigError::NegativeAccel(i));
            }
            if params.min_velocity[i] > 0.0 {
                return Err(ConfigError::PositiveMinVelocity(i));
            }
            if params.max_velocity[i] < 0.0 {
                return Err(ConfigError::NegativeMaxVelocity(i));
            }
            if params.min_velocity[i] > params.max_velocity[i] {
                return Err(ConfigError::MinAboveMax(i));
            }
        }
        Ok(Self {
            velocity_timeout_ns: duration_from_seconds(params.velocity_timeout),
            p: params,
            command: None,
            last_command_time_ns: 0,
            last_cmd: Twist2D::default(),
            stopped: true,
        })
    }

    pub fn params(&self) -> &VelocitySmootherParams {
        &self.p
    }

    /// The timer period `on_activate` creates:
    /// `milliseconds(static_cast<int>(1000.0 / smoothing_frequency))`, 50 ms here.
    pub fn timer_period_ns(&self) -> i64 {
        (1000.0 / self.p.smoothing_frequency) as i32 as i64 * 1_000_000
    }

    /// `stopped_`: the last tick timed out with nothing to publish.
    pub fn stopped(&self) -> bool {
        self.stopped
    }

    /// `last_cmd_`'s twist: the open-loop estimate of the current velocity.
    pub fn last_cmd(&self) -> Twist2D {
        self.last_cmd
    }

    /// The stored command (`command_`), as the timer has left it.
    pub fn command(&self) -> Option<&TwistStamped> {
        self.command.as_ref()
    }

    /// `inputCommandStampedCallback`: keep the command, and time it from its
    /// stamp, or from `now_ns` when the stamp is zero.
    pub fn input_command(&mut self, msg: &TwistStamped, now_ns: i64) -> Result<(), InputError> {
        let t = &msg.twist;
        if ![t.x, t.y, t.theta, msg.other[0], msg.other[1], msg.other[2]].iter().all(|v| v.is_finite()) {
            return Err(InputError::NonFinite);
        }
        if msg.stamp_ns < 0 {
            return Err(InputError::NegativeStamp);
        }
        self.command = Some(msg.clone());
        self.last_command_time_ns = if msg.stamp_ns == 0 { now_ns } else { msg.stamp_ns };
        Ok(())
    }

    /// `inputCommandCallback` (unstamped `cmd_vel`, `enable_stamped_cmd_vel:
    /// false`): a zero stamp and an empty frame.
    pub fn input_command_unstamped(&mut self, twist: &Twist2D, other: [f64; 3], now_ns: i64) -> Result<(), InputError> {
        self.input_command(&TwistStamped { stamp_ns: 0, frame_id: String::new(), twist: *twist, other }, now_ns)
    }

    /// One `smootherTimer` call at ROS time `now_ns`: the message published
    /// on `cmd_vel_smoothed`, or `None` (no command yet, or timed out at rest).
    pub fn tick(&mut self, now_ns: i64) -> Option<TwistStamped> {
        let command = self.command.as_mut()?;
        let delta = now_ns - self.last_command_time_ns;
        let frame_id = command.frame_id.clone();
        let stamp_ns = if self.p.stamp_smoothed_velocity_with_smoothing_time {
            command.stamp_ns + delta
        } else {
            command.stamp_ns
        };

        if delta > self.velocity_timeout_ns {
            if self.last_cmd == Twist2D::default() || self.stopped {
                self.stopped = true;
                return None;
            }
            // `last_command_time_` is left alone, so every later tick lands
            // here again and re-stamps the zero command with its own `now()`.
            *command = TwistStamped { stamp_ns: now_ns, ..TwistStamped::default() };
        }
        self.stopped = false;

        let current = self.last_cmd;
        let (min, max) = (self.p.min_velocity, self.p.max_velocity);
        command.twist.x = clamp(command.twist.x, min[0], max[0]);
        command.twist.y = clamp(command.twist.y, min[1], max[1]);
        command.twist.theta = clamp(command.twist.theta, min[2], max[2]);
        let cmd = command.twist;

        let (accel, decel) = (self.p.max_accel, self.p.max_decel);
        let mut eta: f64 = 1.0;
        if self.p.scale_velocities {
            for (i, (v_curr, v_cmd)) in
                [(current.x, cmd.x), (current.y, cmd.y), (current.theta, cmd.theta)].into_iter().enumerate()
            {
                let curr_eta = self.find_eta_constraint(v_curr, v_cmd, accel[i], decel[i]);
                if curr_eta > 0.0 && (1.0 - curr_eta).abs() > (1.0 - eta).abs() {
                    eta = curr_eta;
                }
            }
        }

        let out = Twist2D {
            x: self.apply_constraints(current.x, cmd.x, accel[0], decel[0], eta),
            y: self.apply_constraints(current.y, cmd.y, accel[1], decel[1], eta),
            theta: self.apply_constraints(current.theta, cmd.theta, accel[2], decel[2], eta),
        };
        self.last_cmd = out;

        let db = self.p.deadband_velocity;
        let deadband = |v: f64, d: f64| if v.abs() < d { 0.0 } else { v };
        Some(TwistStamped {
            stamp_ns,
            frame_id,
            twist: Twist2D { x: deadband(out.x, db[0]), y: deadband(out.y, db[1]), theta: deadband(out.theta, db[2]) },
            other: [0.0; 3],
        })
    }

    /// `findEtaConstraint`: the factor that brings `v_cmd - v_curr` onto this
    /// axis' per-tick limit, or -1 when it is already within it.
    pub fn find_eta_constraint(&self, v_curr: f64, v_cmd: f64, accel: f64, decel: f64) -> f64 {
        let dv = v_cmd - v_curr;
        let (v_component_max, v_component_min) = self.component_limits(v_curr, v_cmd, accel, decel);
        if dv > v_component_max {
            return v_component_max / dv;
        }
        if dv < v_component_min {
            return v_component_min / dv;
        }
        -1.0
    }

    /// `applyConstraints`: `v_curr + clamp(eta * dv)` to this axis' per-tick limit.
    pub fn apply_constraints(&self, v_curr: f64, v_cmd: f64, accel: f64, decel: f64, eta: f64) -> f64 {
        let dv = v_cmd - v_curr;
        let (v_component_max, v_component_min) = self.component_limits(v_curr, v_cmd, accel, decel);
        v_curr + clamp(eta * dv, v_component_min, v_component_max)
    }

    /// Accelerating (|cmd| >= |curr| with no sign change) uses `accel`,
    /// anything else, including passing through zero, `decel`; per tick.
    fn component_limits(&self, v_curr: f64, v_cmd: f64, accel: f64, decel: f64) -> (f64, f64) {
        let f = self.p.smoothing_frequency;
        if v_cmd.abs() >= v_curr.abs() && v_curr * v_cmd >= 0.0 {
            (accel / f, -accel / f)
        } else {
            (-decel / f, decel / f)
        }
    }
}

/// libstdc++'s `std::clamp`: `min(max(v, lo), hi)`.
fn clamp(v: f64, lo: f64, hi: f64) -> f64 {
    let m = if v < lo { lo } else { v };
    if hi < m {
        hi
    } else {
        m
    }
}

/// `rclcpp::Duration::from_seconds`: `static_cast<int64_t>(s * 1e9)`.
fn duration_from_seconds(seconds: f64) -> i64 {
    (seconds * 1e9) as i64
}
