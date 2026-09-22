//! `control_toolbox::RateLimiter<double>` (Jazzy), which is all that
//! `diff_drive_controller::SpeedLimiter` is: the Jazzy SpeedLimiter forwards
//! every call straight to a `RateLimiter`.
//!
//! Port of `control_toolbox/include/control_toolbox/rate_limiter.hpp`
//! (ros-controls/control_toolbox @ jazzy). NaN means "no limit"; an
//! unspecified `min_*` defaults to `-max_*`, and unspecified asymmetric
//! first-derivative limits fall back to the symmetric ones.

/// `std::clamp`, which is `v < lo ? lo : (hi < v ? hi : v)`. Rust's
/// `f64::clamp` panics when `lo > hi` and orders NaN differently, and the
/// limiter relies on the C++ ordering (a NaN `v` passes through unchanged).
#[inline]
fn clamp(v: f64, lo: f64, hi: f64) -> f64 {
    if v < lo {
        lo
    } else if hi < v {
        hi
    } else {
        v
    }
}

/// Limits a value, its first derivative and its second derivative.
///
/// For a velocity that is: velocity, acceleration and jerk.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RateLimiter {
    has_value_limits: bool,
    has_first_derivative_limits: bool,
    has_second_derivative_limits: bool,
    min_value: f64,
    max_value: f64,
    min_first_derivative_neg: f64,
    max_first_derivative_pos: f64,
    min_first_derivative_pos: f64,
    max_first_derivative_neg: f64,
    min_second_derivative: f64,
    max_second_derivative: f64,
}

impl Default for RateLimiter {
    fn default() -> Self {
        Self::new(
            f64::NAN,
            f64::NAN,
            f64::NAN,
            f64::NAN,
            f64::NAN,
            f64::NAN,
            f64::NAN,
            f64::NAN,
        )
        .expect("all-NaN limits are always valid")
    }
}

impl RateLimiter {
    /// Argument order and meaning are the C++ ones. What
    /// `diff_drive_controller::SpeedLimiter(min_velocity, max_velocity,
    /// max_acceleration_reverse, max_acceleration, max_deceleration,
    /// max_deceleration_reverse, min_jerk, max_jerk)` forwards is:
    ///
    /// | SpeedLimiter arg | RateLimiter arg |
    /// |---|---|
    /// | `min_velocity` | `min_value` |
    /// | `max_velocity` | `max_value` |
    /// | `max_acceleration_reverse` | `min_first_derivative_neg` |
    /// | `max_acceleration` | `max_first_derivative_pos` |
    /// | `max_deceleration` | `min_first_derivative_pos` |
    /// | `max_deceleration_reverse` | `max_first_derivative_neg` |
    /// | `min_jerk` | `min_second_derivative` |
    /// | `max_jerk` | `max_second_derivative` |
    ///
    /// Returns `Err` where the C++ throws `std::invalid_argument` (which the
    /// controller turns into a failed `on_configure`).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        min_value: f64,
        max_value: f64,
        min_first_derivative_neg: f64,
        max_first_derivative_pos: f64,
        min_first_derivative_pos: f64,
        max_first_derivative_neg: f64,
        min_second_derivative: f64,
        max_second_derivative: f64,
    ) -> Result<Self, String> {
        let mut min_value = min_value;
        let mut min_first_derivative_neg = min_first_derivative_neg;
        let mut min_first_derivative_pos = min_first_derivative_pos;
        let mut max_first_derivative_neg = max_first_derivative_neg;
        let mut min_second_derivative = min_second_derivative;

        // The C++ starts from `true` for all three and only ever clears them.
        let mut has_value_limits = true;
        let mut has_first_derivative_limits = true;
        let mut has_second_derivative_limits = true;

        if max_value.is_nan() {
            has_value_limits = false;
        }
        if min_value.is_nan() {
            min_value = -max_value;
        }
        if has_value_limits && min_value > max_value {
            return Err("Invalid value limits".to_string());
        }

        if max_first_derivative_pos.is_nan() {
            has_first_derivative_limits = false;
        }
        if min_first_derivative_neg.is_nan() {
            min_first_derivative_neg = -max_first_derivative_pos;
        }
        if has_first_derivative_limits && min_first_derivative_neg > max_first_derivative_pos {
            return Err("Invalid first derivative limits".to_string());
        }
        if has_first_derivative_limits {
            if max_first_derivative_neg.is_nan() {
                max_first_derivative_neg = max_first_derivative_pos;
            }
            if min_first_derivative_pos.is_nan() {
                min_first_derivative_pos = min_first_derivative_neg;
            }
            if min_first_derivative_pos > max_first_derivative_neg {
                return Err("Invalid first derivative limits".to_string());
            }
        }

        if max_second_derivative.is_nan() {
            has_second_derivative_limits = false;
        }
        if min_second_derivative.is_nan() {
            min_second_derivative = -max_second_derivative;
        }
        if has_second_derivative_limits && min_second_derivative > max_second_derivative {
            return Err("Invalid second derivative limits".to_string());
        }

        Ok(Self {
            has_value_limits,
            has_first_derivative_limits,
            has_second_derivative_limits,
            min_value,
            max_value,
            min_first_derivative_neg,
            max_first_derivative_pos,
            min_first_derivative_pos,
            max_first_derivative_neg,
            min_second_derivative,
            max_second_derivative,
        })
    }

    /// Second derivative, then first derivative, then value — in that order,
    /// which is what makes the acceleration limit win over the jerk limit.
    /// Returns the limiting factor (`1.0` when the input was 0).
    pub fn limit(&self, v: &mut f64, v0: f64, v1: f64, dt: f64) -> f64 {
        let tmp = *v;
        self.limit_second_derivative(v, v0, v1, dt);
        self.limit_first_derivative(v, v0, dt);
        self.limit_value(v);
        if tmp != 0.0 {
            *v / tmp
        } else {
            1.0
        }
    }

    pub fn limit_value(&self, v: &mut f64) -> f64 {
        let tmp = *v;
        if self.has_value_limits {
            *v = clamp(*v, self.min_value, self.max_value);
        }
        if tmp != 0.0 {
            *v / tmp
        } else {
            1.0
        }
    }

    /// The bound depends on the sign of the *previous* value, which is how
    /// "deceleration" is told apart from "acceleration".
    pub fn limit_first_derivative(&self, v: &mut f64, v0: f64, dt: f64) -> f64 {
        let tmp = *v;
        if self.has_first_derivative_limits {
            let (dv_min, dv_max) = if v0 > 0.0 {
                (self.min_first_derivative_pos * dt, self.max_first_derivative_pos * dt)
            } else if v0 < 0.0 {
                (self.min_first_derivative_neg * dt, self.max_first_derivative_neg * dt)
            } else {
                (self.min_first_derivative_neg * dt, self.max_first_derivative_pos * dt)
            };
            let dv = clamp(*v - v0, dv_min, dv_max);
            *v = v0 + dv;
        }
        if tmp != 0.0 {
            *v / tmp
        } else {
            1.0
        }
    }

    /// Only applied when accelerating or reverse-accelerating
    /// (`sign(jerk * accel) > 0`); see control_toolbox issue #240.
    pub fn limit_second_derivative(&self, v: &mut f64, v0: f64, v1: f64, dt: f64) -> f64 {
        let tmp = *v;
        if self.has_second_derivative_limits {
            let dv = *v - v0;
            let dv0 = v0 - v1;
            if (dv - dv0) * (*v - v0) > 0.0 {
                let dt2 = dt * dt;
                let da_min = self.min_second_derivative * dt2;
                let da_max = self.max_second_derivative * dt2;
                let da = clamp(dv - dv0, da_min, da_max);
                *v = v0 + dv0 + da;
            }
        }
        if tmp != 0.0 {
            *v / tmp
        } else {
            1.0
        }
    }
}
