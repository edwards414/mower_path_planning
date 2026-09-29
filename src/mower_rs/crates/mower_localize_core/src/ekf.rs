//! `robot_localization/src/ekf.cpp` + `src/filter_base.cpp` (Jazzy 3.8.3),
//! transcribed to Rust. 15 states, the same transfer function and analytic
//! Jacobian, the Joseph-form covariance update and the Mahalanobis gate.
//!
//! What is left out of `filter_base.cpp` is only the debug streaming
//! (`FB_DEBUG`) and `validateDelta`, which is a no-op upstream (its body is
//! commented out). See the crate README for the full list.

use crate::filter_common::*;
use crate::measurement::Measurement;
use crate::tf::normalize_angle;

/// `FilterBase` + `Ekf`.
#[derive(Clone, Debug)]
pub struct Ekf {
    pub state: Vec15,
    pub predicted_state: Vec15,
    pub estimate_error_covariance: Mat15,
    pub process_noise_covariance: Mat15,
    pub dynamic_process_noise_covariance: Mat15,
    transfer_function: Mat15,
    transfer_function_jacobian: Mat15,
    identity: Mat15,
    pub initialized: bool,
    /// `sensor_timeout_`, nanoseconds.
    pub sensor_timeout_ns: i64,
    /// `last_measurement_time_`, nanoseconds.
    pub last_measurement_time_ns: i64,
    pub use_control: bool,
    pub use_dynamic_process_noise_covariance: bool,
    /// `control_timeout_`, nanoseconds.
    pub control_timeout_ns: i64,
    pub control_update_vector: [bool; TWIST_SIZE],
    pub acceleration_limits: [f64; TWIST_SIZE],
    pub acceleration_gains: [f64; TWIST_SIZE],
    pub deceleration_limits: [f64; TWIST_SIZE],
    pub deceleration_gains: [f64; TWIST_SIZE],
    latest_control: [f64; TWIST_SIZE],
    latest_control_time_ns: i64,
    control_acceleration: [f64; TWIST_SIZE],
}

impl Default for Ekf {
    fn default() -> Self {
        Self::new()
    }
}

fn to_sec(nanoseconds: i64) -> f64 {
    // filter_utilities::nanosecToSec
    nanoseconds as f64 * 1e-9
}

impl Ekf {
    /// `FilterBase::FilterBase()` + `FilterBase::reset()`.
    pub fn new() -> Self {
        let mut process_noise_covariance = ZERO_MAT15;
        let defaults = [
            (STATE_X, 0.05),
            (STATE_Y, 0.05),
            (STATE_Z, 0.06),
            (STATE_ROLL, 0.03),
            (STATE_PITCH, 0.03),
            (STATE_YAW, 0.06),
            (STATE_VX, 0.025),
            (STATE_VY, 0.025),
            (STATE_VZ, 0.04),
            (STATE_VROLL, 0.01),
            (STATE_VPITCH, 0.01),
            (STATE_VYAW, 0.02),
            (STATE_AX, 0.01),
            (STATE_AY, 0.01),
            (STATE_AZ, 0.015),
        ];
        for (i, v) in defaults {
            process_noise_covariance[i][i] = v;
        }

        let mut estimate_error_covariance = identity_mat15();
        for (i, row) in estimate_error_covariance.iter_mut().enumerate() {
            row[i] = 1e-9;
        }

        Self {
            state: [0.0; STATE_SIZE],
            predicted_state: [0.0; STATE_SIZE],
            estimate_error_covariance,
            process_noise_covariance,
            dynamic_process_noise_covariance: process_noise_covariance,
            transfer_function: identity_mat15(),
            transfer_function_jacobian: ZERO_MAT15,
            identity: identity_mat15(),
            initialized: false,
            // 0.033333333 s, as upstream.
            sensor_timeout_ns: 33_333_333,
            last_measurement_time_ns: 0,
            use_control: false,
            use_dynamic_process_noise_covariance: false,
            control_timeout_ns: 0,
            control_update_vector: [false; TWIST_SIZE],
            acceleration_limits: [0.0; TWIST_SIZE],
            acceleration_gains: [0.0; TWIST_SIZE],
            deceleration_limits: [0.0; TWIST_SIZE],
            deceleration_gains: [0.0; TWIST_SIZE],
            latest_control: [0.0; TWIST_SIZE],
            latest_control_time_ns: 0,
            control_acceleration: [0.0; TWIST_SIZE],
        }
    }

    pub fn set_process_noise_covariance(&mut self, q: Mat15) {
        self.process_noise_covariance = q;
        self.dynamic_process_noise_covariance = q;
    }

    pub fn set_control(&mut self, control: [f64; TWIST_SIZE], control_time_ns: i64) {
        self.latest_control = control;
        self.latest_control_time_ns = control_time_ns;
    }

    /// `FilterBase::computeDynamicProcessNoiseCovariance`.
    fn compute_dynamic_process_noise_covariance(&mut self) {
        let norm = (self.state[POSITION_V_OFFSET..POSITION_V_OFFSET + TWIST_SIZE]
            .iter()
            .map(|v| v * v)
            .sum::<f64>())
        .sqrt();
        for i in 0..TWIST_SIZE {
            for j in 0..TWIST_SIZE {
                self.dynamic_process_noise_covariance[POSITION_OFFSET + i][POSITION_OFFSET + j] =
                    norm * self.process_noise_covariance[POSITION_OFFSET + i][POSITION_OFFSET + j]
                        * norm;
            }
        }
    }

    /// `FilterBase::wrapStateAngles`.
    fn wrap_state_angles(&mut self) {
        self.state[STATE_ROLL] = normalize_angle(self.state[STATE_ROLL]);
        self.state[STATE_PITCH] = normalize_angle(self.state[STATE_PITCH]);
        self.state[STATE_YAW] = normalize_angle(self.state[STATE_YAW]);
    }

    /// `FilterBase::computeControlAcceleration`.
    fn compute_control_acceleration(
        state: f64,
        control: f64,
        acceleration_limit: f64,
        acceleration_gain: f64,
        deceleration_limit: f64,
        deceleration_gain: f64,
    ) -> f64 {
        let error = control - state;
        let same_sign = error.abs() <= control.abs() + 0.01;
        let set_point = if same_sign { control } else { 0.0 };
        let decelerating = set_point.abs() < state.abs();
        let mut limit = acceleration_limit;
        let mut gain = acceleration_gain;
        if decelerating {
            limit = deceleration_limit;
            gain = deceleration_gain;
        }
        (gain * error).max(-limit).min(limit)
    }

    /// `FilterBase::prepareControl`.
    fn prepare_control(&mut self, reference_time_ns: i64) {
        self.control_acceleration = [0.0; TWIST_SIZE];
        if self.use_control {
            let timed_out =
                (reference_time_ns - self.latest_control_time_ns) >= self.control_timeout_ns;
            for i in 0..TWIST_SIZE {
                if self.control_update_vector[i] {
                    self.control_acceleration[i] = Self::compute_control_acceleration(
                        self.state[i + POSITION_V_OFFSET],
                        if timed_out {
                            0.0
                        } else {
                            self.latest_control[i]
                        },
                        self.acceleration_limits[i],
                        self.acceleration_gains[i],
                        self.deceleration_limits[i],
                        self.deceleration_gains[i],
                    );
                }
            }
        }
    }

    /// `FilterBase::checkMahalanobisThreshold`.
    fn check_mahalanobis_threshold(
        innovation: &[f64],
        innovation_covariance: &Mat15,
        n: usize,
        n_sigmas: f64,
    ) -> bool {
        let mut squared_mahalanobis = 0.0;
        for i in 0..n {
            let mut s = 0.0;
            for j in 0..n {
                s += innovation_covariance[i][j] * innovation[j];
            }
            squared_mahalanobis += innovation[i] * s;
        }
        let threshold = n_sigmas * n_sigmas;
        squared_mahalanobis < threshold
    }

    /// `Ekf::predict`.
    pub fn predict(&mut self, reference_time_ns: i64, delta_ns: i64) {
        let delta_sec = to_sec(delta_ns);

        let roll = self.state[STATE_ROLL];
        let pitch = self.state[STATE_PITCH];
        let yaw = self.state[STATE_YAW];
        let x_vel = self.state[STATE_VX];
        let y_vel = self.state[STATE_VY];
        let z_vel = self.state[STATE_VZ];
        let pitch_vel = self.state[STATE_VPITCH];
        let yaw_vel = self.state[STATE_VYAW];
        let x_acc = self.state[STATE_AX];
        let y_acc = self.state[STATE_AY];
        let z_acc = self.state[STATE_AZ];

        let sp = pitch.sin();
        let cp = pitch.cos();
        let cpi = 1.0 / cp;
        let tp = sp * cpi;

        let sr = roll.sin();
        let cr = roll.cos();

        let sy = yaw.sin();
        let cy = yaw.cos();

        self.prepare_control(reference_time_ns);

        let tf = &mut self.transfer_function;
        tf[STATE_X][STATE_VX] = cy * cp * delta_sec;
        tf[STATE_X][STATE_VY] = (cy * sp * sr - sy * cr) * delta_sec;
        tf[STATE_X][STATE_VZ] = (cy * sp * cr + sy * sr) * delta_sec;
        tf[STATE_X][STATE_AX] = 0.5 * tf[STATE_X][STATE_VX] * delta_sec;
        tf[STATE_X][STATE_AY] = 0.5 * tf[STATE_X][STATE_VY] * delta_sec;
        tf[STATE_X][STATE_AZ] = 0.5 * tf[STATE_X][STATE_VZ] * delta_sec;
        tf[STATE_Y][STATE_VX] = sy * cp * delta_sec;
        tf[STATE_Y][STATE_VY] = (sy * sp * sr + cy * cr) * delta_sec;
        tf[STATE_Y][STATE_VZ] = (sy * sp * cr - cy * sr) * delta_sec;
        tf[STATE_Y][STATE_AX] = 0.5 * tf[STATE_Y][STATE_VX] * delta_sec;
        tf[STATE_Y][STATE_AY] = 0.5 * tf[STATE_Y][STATE_VY] * delta_sec;
        tf[STATE_Y][STATE_AZ] = 0.5 * tf[STATE_Y][STATE_VZ] * delta_sec;
        tf[STATE_Z][STATE_VX] = -sp * delta_sec;
        tf[STATE_Z][STATE_VY] = cp * sr * delta_sec;
        tf[STATE_Z][STATE_VZ] = cp * cr * delta_sec;
        tf[STATE_Z][STATE_AX] = 0.5 * tf[STATE_Z][STATE_VX] * delta_sec;
        tf[STATE_Z][STATE_AY] = 0.5 * tf[STATE_Z][STATE_VY] * delta_sec;
        tf[STATE_Z][STATE_AZ] = 0.5 * tf[STATE_Z][STATE_VZ] * delta_sec;
        tf[STATE_ROLL][STATE_VROLL] = delta_sec;
        tf[STATE_ROLL][STATE_VPITCH] = sr * tp * delta_sec;
        tf[STATE_ROLL][STATE_VYAW] = cr * tp * delta_sec;
        tf[STATE_PITCH][STATE_VPITCH] = cr * delta_sec;
        tf[STATE_PITCH][STATE_VYAW] = -sr * delta_sec;
        tf[STATE_YAW][STATE_VPITCH] = sr * cpi * delta_sec;
        tf[STATE_YAW][STATE_VYAW] = cr * cpi * delta_sec;
        tf[STATE_VX][STATE_AX] = delta_sec;
        tf[STATE_VY][STATE_AY] = delta_sec;
        tf[STATE_VZ][STATE_AZ] = delta_sec;

        let one_half_at_squared = 0.5 * delta_sec * delta_sec;

        let mut y_coeff = cy * sp * cr + sy * sr;
        let mut z_coeff = -cy * sp * sr + sy * cr;
        let d_fx_dr = (y_coeff * y_vel + z_coeff * z_vel) * delta_sec
            + (y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;
        let d_fr_dr = 1.0 + (cr * tp * pitch_vel - sr * tp * yaw_vel) * delta_sec;

        let mut x_coeff = -cy * sp;
        y_coeff = cy * cp * sr;
        z_coeff = cy * cp * cr;
        let d_fx_dp = (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_sec
            + (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;
        let d_fr_dp = (cpi * cpi * sr * pitch_vel + cpi * cpi * cr * yaw_vel) * delta_sec;

        x_coeff = -sy * cp;
        y_coeff = -sy * sp * sr - cy * cr;
        z_coeff = -sy * sp * cr + cy * sr;
        let d_fx_dy = (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_sec
            + (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;

        y_coeff = sy * sp * cr - cy * sr;
        z_coeff = -sy * sp * sr - cy * cr;
        let d_fy_dr = (y_coeff * y_vel + z_coeff * z_vel) * delta_sec
            + (y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;
        let d_fp_dr = (-sr * pitch_vel - cr * yaw_vel) * delta_sec;

        x_coeff = -sy * sp;
        y_coeff = sy * cp * sr;
        z_coeff = sy * cp * cr;
        let d_fy_dp = (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_sec
            + (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;

        x_coeff = cy * cp;
        y_coeff = cy * sp * sr - sy * cr;
        z_coeff = cy * sp * cr + sy * sr;
        let d_fy_dy = (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_sec
            + (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;

        y_coeff = cp * cr;
        z_coeff = -cp * sr;
        let d_fz_dr = (y_coeff * y_vel + z_coeff * z_vel) * delta_sec
            + (y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;
        let d_fy_dr2 = (cr * cpi * pitch_vel - sr * cpi * yaw_vel) * delta_sec; // dFY_dR

        x_coeff = -cp;
        y_coeff = -sp * sr;
        z_coeff = -sp * cr;
        let d_fz_dp = (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_sec
            + (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;
        let d_fyaw_dp = (sr * tp * cpi * pitch_vel + cr * tp * cpi * yaw_vel) * delta_sec;

        self.transfer_function_jacobian = self.transfer_function;
        let j = &mut self.transfer_function_jacobian;
        j[STATE_X][STATE_ROLL] = d_fx_dr;
        j[STATE_X][STATE_PITCH] = d_fx_dp;
        j[STATE_X][STATE_YAW] = d_fx_dy;
        j[STATE_Y][STATE_ROLL] = d_fy_dr;
        j[STATE_Y][STATE_PITCH] = d_fy_dp;
        j[STATE_Y][STATE_YAW] = d_fy_dy;
        j[STATE_Z][STATE_ROLL] = d_fz_dr;
        j[STATE_Z][STATE_PITCH] = d_fz_dp;
        j[STATE_ROLL][STATE_ROLL] = d_fr_dr;
        j[STATE_ROLL][STATE_PITCH] = d_fr_dp;
        j[STATE_PITCH][STATE_ROLL] = d_fp_dr;
        j[STATE_YAW][STATE_ROLL] = d_fy_dr2;
        j[STATE_YAW][STATE_PITCH] = d_fyaw_dp;

        if self.use_dynamic_process_noise_covariance {
            self.compute_dynamic_process_noise_covariance();
        }
        let process_noise_covariance = if self.use_dynamic_process_noise_covariance {
            self.dynamic_process_noise_covariance
        } else {
            self.process_noise_covariance
        };

        // (1) Apply the control terms, which are accelerations.
        self.state[STATE_VROLL] += self.control_acceleration[CONTROL_VROLL] * delta_sec;
        self.state[STATE_VPITCH] += self.control_acceleration[CONTROL_VPITCH] * delta_sec;
        self.state[STATE_VYAW] += self.control_acceleration[CONTROL_VYAW] * delta_sec;
        if self.control_update_vector[CONTROL_VX] {
            self.state[STATE_AX] = self.control_acceleration[CONTROL_VX];
        }
        if self.control_update_vector[CONTROL_VY] {
            self.state[STATE_AY] = self.control_acceleration[CONTROL_VY];
        }
        if self.control_update_vector[CONTROL_VZ] {
            self.state[STATE_AZ] = self.control_acceleration[CONTROL_VZ];
        }

        // (2) Project the state forward.
        self.state = matvec15(&self.transfer_function, &self.state);
        self.wrap_state_angles();

        // (3) Project the error forward: P = J P J' + Q * dt.
        let jp = matmul15(
            &self.transfer_function_jacobian,
            &self.estimate_error_covariance,
        );
        self.estimate_error_covariance = matmul15_transpose(&jp, &self.transfer_function_jacobian);
        for i in 0..STATE_SIZE {
            for j2 in 0..STATE_SIZE {
                self.estimate_error_covariance[i][j2] +=
                    delta_sec * process_noise_covariance[i][j2];
            }
        }
    }

    /// `Ekf::correct`.
    pub fn correct(&mut self, measurement: &Measurement) {
        // Which state values are we updating?
        let mut update_indices: [usize; STATE_SIZE] = [0; STATE_SIZE];
        let mut update_size = 0usize;
        for i in 0..STATE_SIZE {
            if measurement.update_vector[i] {
                let v = measurement.measurement[i];
                if v.is_nan() || v.is_infinite() {
                    continue;
                }
                update_indices[update_size] = i;
                update_size += 1;
            }
        }
        let n = update_size;
        if n == 0 {
            return;
        }

        let mut state_subset = [0.0f64; STATE_SIZE];
        let mut measurement_subset = [0.0f64; STATE_SIZE];
        let mut measurement_covariance_subset = ZERO_MAT15;

        for i in 0..n {
            measurement_subset[i] = measurement.measurement[update_indices[i]];
            state_subset[i] = self.state[update_indices[i]];
            for j in 0..n {
                measurement_covariance_subset[i][j] =
                    measurement.covariance[update_indices[i]][update_indices[j]];
            }
            if measurement_covariance_subset[i][i] < 0.0 {
                measurement_covariance_subset[i][i] = measurement_covariance_subset[i][i].abs();
            }
            if measurement_covariance_subset[i][i] < 1e-9 {
                measurement_covariance_subset[i][i] = 1e-9;
            }
        }

        // H is the (n x 15) selection matrix, so PH' is just the selected
        // columns of P and HPH' the selected block.
        let mut pht = [[0.0f64; STATE_SIZE]; STATE_SIZE]; // 15 x n
        for (i, row) in pht.iter_mut().enumerate() {
            for j in 0..n {
                row[j] = self.estimate_error_covariance[i][update_indices[j]];
            }
        }
        let mut hphr = ZERO_MAT15; // n x n
        for i in 0..n {
            for j in 0..n {
                hphr[i][j] = pht[update_indices[i]][j] + measurement_covariance_subset[i][j];
            }
        }
        let hphr_inverse = inverse_partial_piv_lu(&hphr, n);

        // K = PH' * (HPH' + R)^-1, a 15 x n matrix.
        let mut kalman_gain = [[0.0f64; STATE_SIZE]; STATE_SIZE];
        for (i, row) in kalman_gain.iter_mut().enumerate() {
            for j in 0..n {
                let mut s = 0.0;
                for k in 0..n {
                    s += pht[i][k] * hphr_inverse[k][j];
                }
                row[j] = s;
            }
        }

        let mut innovation = [0.0f64; STATE_SIZE];
        for i in 0..n {
            innovation[i] = measurement_subset[i] - state_subset[i];
            let idx = update_indices[i];
            if idx == STATE_ROLL || idx == STATE_PITCH || idx == STATE_YAW {
                innovation[i] = normalize_angle(innovation[i]);
            }
        }

        if !Self::check_mahalanobis_threshold(
            &innovation,
            &hphr_inverse,
            n,
            measurement.mahalanobis_thresh,
        ) {
            return;
        }

        // x = x + K(z - Hx)
        for i in 0..STATE_SIZE {
            let mut s = 0.0;
            for j in 0..n {
                s += kalman_gain[i][j] * innovation[j];
            }
            self.state[i] += s;
        }

        // Joseph form: (I - KH) P (I - KH)' + K R K'
        let mut gain_residual = self.identity;
        for i in 0..STATE_SIZE {
            for j in 0..n {
                gain_residual[i][update_indices[j]] -= kalman_gain[i][j];
            }
        }
        let gp = matmul15(&gain_residual, &self.estimate_error_covariance);
        self.estimate_error_covariance = matmul15_transpose(&gp, &gain_residual);

        // K R K'
        let mut kr = [[0.0f64; STATE_SIZE]; STATE_SIZE]; // 15 x n
        for (i, row) in kr.iter_mut().enumerate() {
            for j in 0..n {
                let mut s = 0.0;
                for k in 0..n {
                    s += kalman_gain[i][k] * measurement_covariance_subset[k][j];
                }
                row[j] = s;
            }
        }
        for i in 0..STATE_SIZE {
            for j in 0..STATE_SIZE {
                let mut s = 0.0;
                for k in 0..n {
                    s += kr[i][k] * kalman_gain[j][k];
                }
                self.estimate_error_covariance[i][j] += s;
            }
        }

        self.wrap_state_angles();
    }

    /// `FilterBase::processMeasurement`.
    pub fn process_measurement(&mut self, measurement: &Measurement) {
        let mut delta_ns: i64 = 0;

        if self.initialized {
            delta_ns = measurement.time_ns - self.last_measurement_time_ns;
            if delta_ns > 0 {
                self.predict(measurement.time_ns, delta_ns);
                self.predicted_state = self.state;
            }
            self.correct(measurement);
        } else {
            for i in 0..STATE_SIZE {
                if measurement.update_vector[i] {
                    self.state[i] = measurement.measurement[i];
                }
            }
            for i in 0..STATE_SIZE {
                for j in 0..STATE_SIZE {
                    if measurement.update_vector[i] && measurement.update_vector[j] {
                        self.estimate_error_covariance[i][j] = measurement.covariance[i][j];
                    }
                }
            }
            self.initialized = true;
        }

        if delta_ns >= 0 {
            self.last_measurement_time_ns = measurement.time_ns;
        }
    }
}
