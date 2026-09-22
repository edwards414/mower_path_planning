//! Differential-drive odometry.
//!
//! Port of `ros2_controllers/diff_drive_controller/src/odometry.cpp` (jazzy)
//! together with `rcpputils::RollingMeanAccumulator`, which is what filters
//! the reported twist.
//!
//! The integration method is "exact" (arc) whenever `|angular| >= 1e-6` and
//! second-order Runge-Kutta below that, exactly as upstream.

use crate::{seconds, TimeNs};

/// `rcpputils::RollingMeanAccumulator<double>`: a fixed ring buffer plus a
/// running sum, so the mean is `sum / min(samples_seen, window)`.
///
/// Note the running sum is never recomputed, so it carries the same floating
/// point history the C++ one does; the port keeps the same operation order.
#[derive(Debug, Clone, PartialEq)]
pub struct RollingMeanAccumulator {
    buffer: Vec<f64>,
    next_insert: usize,
    sum: f64,
    buffer_filled: bool,
}

impl RollingMeanAccumulator {
    pub fn new(rolling_window_size: usize) -> Self {
        Self {
            buffer: vec![0.0; rolling_window_size],
            next_insert: 0,
            sum: 0.0,
            buffer_filled: false,
        }
    }

    pub fn accumulate(&mut self, val: f64) {
        self.sum -= self.buffer[self.next_insert];
        self.sum += val;
        self.buffer[self.next_insert] = val;
        self.next_insert += 1;
        self.buffer_filled |= self.next_insert >= self.buffer.len();
        self.next_insert %= self.buffer.len();
    }

    /// Undefined in C++ (an assert) before the first sample; here it is 0.0.
    pub fn rolling_mean(&self) -> f64 {
        let valid = if self.buffer_filled {
            self.buffer.len()
        } else {
            self.next_insert
        };
        if valid == 0 {
            return 0.0;
        }
        self.sum / valid as f64
    }
}

/// `diff_drive_controller::Odometry`.
#[derive(Debug, Clone)]
pub struct Odometry {
    timestamp_ns: TimeNs,
    x: f64,
    y: f64,
    heading: f64,
    linear: f64,
    angular: f64,
    wheel_separation: f64,
    left_wheel_radius: f64,
    right_wheel_radius: f64,
    left_wheel_old_pos: f64,
    right_wheel_old_pos: f64,
    velocity_rolling_window_size: usize,
    linear_accumulator: RollingMeanAccumulator,
    angular_accumulator: RollingMeanAccumulator,
}

impl Odometry {
    pub fn new(velocity_rolling_window_size: usize) -> Self {
        Self {
            // The C++ ctor is `timestamp_(0.0)`: the epoch, not "now". The
            // controller never calls init(), so the first update sees a dt of
            // however many seconds the clock reads, and the first rolling-mean
            // sample is therefore ~0.
            timestamp_ns: 0,
            x: 0.0,
            y: 0.0,
            heading: 0.0,
            linear: 0.0,
            angular: 0.0,
            wheel_separation: 0.0,
            left_wheel_radius: 0.0,
            right_wheel_radius: 0.0,
            left_wheel_old_pos: 0.0,
            right_wheel_old_pos: 0.0,
            velocity_rolling_window_size,
            linear_accumulator: RollingMeanAccumulator::new(velocity_rolling_window_size),
            angular_accumulator: RollingMeanAccumulator::new(velocity_rolling_window_size),
        }
    }

    /// Reset the accumulators and the timestamp.
    pub fn init(&mut self, time: TimeNs) {
        self.reset_accumulators();
        self.timestamp_ns = time;
    }

    /// Feed wheel *positions* (rad). This is the `position_feedback: true`
    /// path. Returns false when the interval is too small to integrate with.
    pub fn update(&mut self, left_pos: f64, right_pos: f64, time: TimeNs) -> bool {
        // We cannot estimate the speed with very small time intervals:
        let dt = seconds(time) - seconds(self.timestamp_ns);
        if dt < 0.0001 {
            return false;
        }

        let left_wheel_cur_pos = left_pos * self.left_wheel_radius;
        let right_wheel_cur_pos = right_pos * self.right_wheel_radius;

        // "est_vel" upstream, but it is a distance travelled over the step.
        let left_wheel_est_vel = left_wheel_cur_pos - self.left_wheel_old_pos;
        let right_wheel_est_vel = right_wheel_cur_pos - self.right_wheel_old_pos;

        self.left_wheel_old_pos = left_wheel_cur_pos;
        self.right_wheel_old_pos = right_wheel_cur_pos;

        self.update_from_velocity(left_wheel_est_vel, right_wheel_est_vel, time);
        true
    }

    /// Feed per-wheel *distances* travelled over the step (metres).
    pub fn update_from_velocity(&mut self, left_vel: f64, right_vel: f64, time: TimeNs) -> bool {
        let dt = seconds(time) - seconds(self.timestamp_ns);
        if dt < 0.0001 {
            return false;
        }
        let linear = (left_vel + right_vel) * 0.5;
        let angular = (right_vel - left_vel) / self.wheel_separation;

        self.integrate_exact(linear, angular);

        self.timestamp_ns = time;

        // Estimate speeds using a rolling mean to filter them out:
        self.linear_accumulator.accumulate(linear / dt);
        self.angular_accumulator.accumulate(angular / dt);

        self.linear = self.linear_accumulator.rolling_mean();
        self.angular = self.angular_accumulator.rolling_mean();
        true
    }

    /// `open_loop: true` path: integrate the command, report it verbatim.
    pub fn update_open_loop(&mut self, linear: f64, angular: f64, time: TimeNs) {
        self.linear = linear;
        self.angular = angular;
        let dt = seconds(time) - seconds(self.timestamp_ns);
        self.timestamp_ns = time;
        self.integrate_exact(linear * dt, angular * dt);
    }

    pub fn reset_odometry(&mut self) {
        self.x = 0.0;
        self.y = 0.0;
        self.heading = 0.0;
    }

    pub fn set_wheel_params(
        &mut self,
        wheel_separation: f64,
        left_wheel_radius: f64,
        right_wheel_radius: f64,
    ) {
        self.wheel_separation = wheel_separation;
        self.left_wheel_radius = left_wheel_radius;
        self.right_wheel_radius = right_wheel_radius;
    }

    pub fn set_velocity_rolling_window_size(&mut self, velocity_rolling_window_size: usize) {
        self.velocity_rolling_window_size = velocity_rolling_window_size;
        self.reset_accumulators();
    }

    pub fn x(&self) -> f64 {
        self.x
    }
    pub fn y(&self) -> f64 {
        self.y
    }
    pub fn heading(&self) -> f64 {
        self.heading
    }
    /// Rolling-mean linear velocity, m/s.
    pub fn linear(&self) -> f64 {
        self.linear
    }
    /// Rolling-mean angular velocity, rad/s.
    pub fn angular(&self) -> f64 {
        self.angular
    }
    pub fn timestamp_ns(&self) -> TimeNs {
        self.timestamp_ns
    }

    /// Runge-Kutta 2nd order integration.
    fn integrate_runge_kutta2(&mut self, linear: f64, angular: f64) {
        let direction = self.heading + angular * 0.5;
        self.x += linear * direction.cos();
        self.y += linear * direction.sin();
        self.heading += angular;
    }

    /// Exact (arc) integration; falls back to RK2 when the arc degenerates.
    fn integrate_exact(&mut self, linear: f64, angular: f64) {
        if angular.abs() < 1e-6 {
            self.integrate_runge_kutta2(linear, angular);
        } else {
            let heading_old = self.heading;
            let r = linear / angular;
            self.heading += angular;
            self.x += r * (self.heading.sin() - heading_old.sin());
            self.y += -r * (self.heading.cos() - heading_old.cos());
        }
    }

    fn reset_accumulators(&mut self) {
        self.linear_accumulator = RollingMeanAccumulator::new(self.velocity_rolling_window_size);
        self.angular_accumulator = RollingMeanAccumulator::new(self.velocity_rolling_window_size);
    }
}
