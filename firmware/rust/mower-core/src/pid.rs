//! Discrete PID with clamped integral and output (port of `pid_controller.cpp`).

use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout};

/// Same layout as `pid_gains_t` (7 × f32) so it can live inside the flash
/// settings record unchanged.
#[derive(Clone, Copy, Debug, PartialEq, FromBytes, IntoBytes, Immutable, KnownLayout)]
#[repr(C)]
pub struct Gains {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub integral_min: f32,
    pub integral_max: f32,
    pub output_min: f32,
    pub output_max: f32,
}

const _: () = assert!(core::mem::size_of::<Gains>() == 28);

impl Default for Gains {
    fn default() -> Self {
        Self {
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
            integral_min: -200.0,
            integral_max: 200.0,
            output_min: -200.0,
            output_max: 200.0,
        }
    }
}

impl Gains {
    /// Swap any inverted min/max pair, as `PidController::Configure` did.
    fn normalised(mut self) -> Self {
        if self.integral_min > self.integral_max {
            core::mem::swap(&mut self.integral_min, &mut self.integral_max);
        }
        if self.output_min > self.output_max {
            core::mem::swap(&mut self.output_min, &mut self.output_max);
        }
        self
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Pid {
    gains: Gains,
    integral: f32,
    previous_error: Option<f32>,
}

impl Pid {
    pub fn new(gains: Gains) -> Self {
        Self { gains: gains.normalised(), integral: 0.0, previous_error: None }
    }

    pub fn configure(&mut self, gains: Gains) {
        self.gains = gains.normalised();
        self.integral = self.integral.clamp(self.gains.integral_min, self.gains.integral_max);
    }

    pub fn gains(&self) -> &Gains {
        &self.gains
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_error = None;
    }

    pub fn update(&mut self, setpoint: f32, measurement: f32, dt_s: f32) -> f32 {
        if dt_s <= 0.0 {
            return 0.0;
        }
        let g = &self.gains;
        let error = setpoint - measurement;
        self.integral = (self.integral + error * dt_s).clamp(g.integral_min, g.integral_max);
        let derivative = match self.previous_error {
            Some(prev) => (error - prev) / dt_s,
            None => 0.0,
        };
        self.previous_error = Some(error);
        let output = g.kp * error + g.ki * self.integral + g.kd * derivative;
        output.clamp(g.output_min, g.output_max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gains(kp: f32, ki: f32, kd: f32) -> Gains {
        Gains { kp, ki, kd, ..Gains::default() }
    }

    #[test]
    fn proportional_only() {
        let mut pid = Pid::new(gains(2.0, 0.0, 0.0));
        assert_eq!(pid.update(10.0, 4.0, 0.02), 12.0);
    }

    #[test]
    fn integral_accumulates_and_clamps() {
        let mut pid = Pid::new(Gains { integral_min: -1.0, integral_max: 1.0, ..gains(0.0, 1.0, 0.0) });
        for _ in 0..1000 {
            pid.update(100.0, 0.0, 0.02);
        }
        assert_eq!(pid.update(100.0, 0.0, 0.02), 1.0);
    }

    #[test]
    fn derivative_is_zero_on_first_sample() {
        let mut pid = Pid::new(gains(0.0, 0.0, 1.0));
        assert_eq!(pid.update(1.0, 0.0, 0.1), 0.0);
        // error goes 1 -> 3 over 0.1 s => d = 20
        assert_eq!(pid.update(3.0, 0.0, 0.1), 20.0);
    }

    #[test]
    fn output_clamped_and_inverted_limits_fixed() {
        let mut pid = Pid::new(Gains { output_min: 5.0, output_max: -5.0, ..gains(100.0, 0.0, 0.0) });
        assert_eq!(pid.gains().output_min, -5.0);
        assert_eq!(pid.update(1.0, 0.0, 0.02), 5.0);
    }

    #[test]
    fn non_positive_dt_is_ignored() {
        let mut pid = Pid::new(gains(1.0, 1.0, 1.0));
        assert_eq!(pid.update(1.0, 0.0, 0.0), 0.0);
        assert_eq!(pid.update(1.0, 0.0, -1.0), 0.0);
    }
}
