//! Wheel-speed PID auto-tune maths: step-response identification + SIMC PI
//! (line-by-line port of `mower_mission/pid_tuning.py`, ROS-free).
//!
//! The node feeds it the (t, pwm counts, rpm) samples the STM32 reports in
//! 0x85 while the wheel is driven open loop, gets a first-order-plus-dead-time
//! (FOPDT) model back and turns that into PI gains for the firmware's PID
//! (error in rpm, output in PWM counts, 20 ms tick).
//!
//! Model:  y(t) = y0 + K * du * (1 - exp(-(t - t_step - L) / tau))  for t > t_step + L
//!
//! Tuning (Skogestad SIMC, PI form):
//!     tau_c = max(L, tau_c_factor * tau)
//!     Kp    = tau / (K * (tau_c + L))
//!     Ti    = min(tau, 4 * (tau_c + L))
//!     Ki    = Kp / Ti
//!     Kd    = 0

use serde::{Deserialize, Serialize};

use mower_rs_common::round_to;

/// The recorded response cannot be turned into gains (with a user-facing reason).
#[derive(Debug, Clone, PartialEq)]
pub struct TuningError(pub String);

impl std::fmt::Display for TuningError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

fn err<T>(msg: impl Into<String>) -> Result<T, TuningError> {
    Err(TuningError(msg.into()))
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    /// s, any monotonic base
    pub t: f64,
    /// PWM counts actually applied (0x85 pid_output)
    pub pwm: f64,
    /// measured wheel speed (0x85 measured_rpm)
    pub rpm: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FopdtModel {
    /// rpm per PWM count
    pub gain: f64,
    /// s
    pub tau: f64,
    /// s
    pub delay: f64,
    /// rpm before the step
    pub y0: f64,
    /// rpm after the step settled
    pub y_ss: f64,
    /// counts before the step
    pub u0: f64,
    /// counts after the step
    pub u1: f64,
    /// 1 - SSE/SST over the post-step window (informational)
    pub fit_r2: f64,
    /// rpm, residual of the fit
    #[serde(default)]
    pub fit_rmse: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Gains {
    pub kp: f64,
    pub ki: f64,
    #[serde(default)]
    pub kd: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StepMetrics {
    /// rpm
    pub target: f64,
    /// (peak - target) / target * 100, 0 if none
    pub overshoot_pct: f64,
    /// first time after which |y - target| <= band, None if never
    pub settle_s: Option<f64>,
    /// mean of the last window minus target, rpm
    pub ss_error: f64,
    /// 10 % -> 90 % of target
    pub rise_s: Option<f64>,
}

// Physical plausibility of a geared brushed DC wheel motor read at 50 ms.
pub const GAIN_RANGE: (f64, f64) = (0.02, 5.0); // rpm / count (58 rpm at 200 counts is 0.29)
pub const TAU_RANGE: (f64, f64) = (0.02, 3.0); // s
pub const DELAY_MAX: f64 = 0.6; // s
pub const MIN_RESPONSE_RPM: f64 = 2.0;
pub const MAX_FIT_RMSE_FRACTION: f64 = 0.10; // of the response amplitude, plus the pre-step noise

/// A wheel whose speed settles within one 50 ms sample fits as tau at the
/// bottom of TAU_RANGE (seen on the real base: tau ~0.02 s, delay ~0.04 s).
/// SIMC would then set Ti = tau = the control period itself, a pure-integral
/// controller with no margin. Design against at least one sample of tau.
pub const TAU_DESIGN_MIN: f64 = 0.05;

fn mean(values: impl Iterator<Item = f64>) -> f64 {
    let (mut sum, mut n) = (0.0, 0usize);
    for v in values {
        sum += v;
        n += 1;
    }
    sum / n as f64
}

fn window(samples: &[Sample], t_from: f64, t_to: f64) -> Vec<Sample> {
    samples.iter().filter(|s| t_from <= s.t && s.t <= t_to).copied().collect()
}

fn sorted_by_t(mut data: Vec<Sample>) -> Vec<Sample> {
    // Python's sort is stable; so is sort_by.
    data.sort_by(|a, b| a.t.partial_cmp(&b.t).unwrap_or(std::cmp::Ordering::Equal));
    data
}

/// Fit a FOPDT model to one step in the applied PWM.
///
/// `samples` must cover `pre_window_s` before `t_step` and enough after it
/// for the speed to (nearly) settle; `t_end` cuts the record where the step
/// stopped being held (anything after, e.g. the ramp down, is ignored). The
/// step is located from the samples themselves, `t_step` only says where to
/// look. The steady-state speed is a fitted parameter, not the last sample,
/// so a slightly unsettled response still gives the right gain; one that is
/// clearly still rising is rejected.
pub fn fit_fopdt(samples: &[Sample], t_step: f64, t_end: Option<f64>) -> Result<FopdtModel, TuningError> {
    fit_fopdt_with(samples, t_step, t_end, 0.5, 0.8)
}

pub fn fit_fopdt_with(
    samples: &[Sample],
    t_step: f64,
    t_end: Option<f64>,
    pre_window_s: f64,
    settle_window_s: f64,
) -> Result<FopdtModel, TuningError> {
    let data = sorted_by_t(samples.iter().filter(|s| t_end.map_or(true, |e| s.t <= e)).copied().collect());
    if data.len() < 12 {
        return err("not enough samples");
    }
    let pre = window(&data, t_step - pre_window_s, t_step - 1e-9);
    let post: Vec<Sample> = data.iter().filter(|s| s.t >= t_step).copied().collect();
    if pre.len() < 3 || post.len() < 8 {
        return err("step is not inside the recorded window");
    }

    let t_end = post[post.len() - 1].t;
    let tail = window(&post, t_end - settle_window_s, t_end);
    if tail.len() < 4 {
        return err("response too short to judge steady state");
    }

    let u0 = mean(pre.iter().map(|s| s.pwm));
    let u1 = mean(tail.iter().map(|s| s.pwm));
    let du = u1 - u0;
    if du.abs() < 1.0 {
        return err("PWM did not change across the step");
    }
    let y0 = mean(pre.iter().map(|s| s.rpm));
    let pre_noise = mean(pre.iter().map(|s| (s.rpm - y0).powi(2))).sqrt();
    let dy_tail = mean(tail.iter().map(|s| s.rpm)) - y0;
    if dy_tail.abs() < MIN_RESPONSE_RPM.max(3.0 * pre_noise) {
        return err("wheel speed did not respond to the step (no encoder signal, wheel blocked, or motor not driven)");
    }
    if dy_tail / du < 0.0 {
        return err("speed went the wrong way for the PWM step (encoder or motor sign)");
    }

    // When did the PWM actually change? The first post sample whose pwm is
    // past the midpoint marks the step; the delay is measured from there.
    let mid = u0 + 0.5 * du;
    let mut t0 = post[0].t;
    for s in &post {
        if (s.pwm - mid) * du >= 0.0 {
            t0 = s.t;
            break;
        }
    }

    // Least squares over a (tau, delay) grid, coarse then refined; for each
    // pair the response amplitude dy is linear and solved in closed form.
    // Cheap (a few hundred samples) and immune to noisy 28 % / 63 % points.
    let ts: Vec<f64> = post.iter().map(|s| s.t - t0).collect();
    let ys: Vec<f64> = post.iter().map(|s| s.rpm - y0).collect();

    let solve = |tau: f64, delay: f64| -> (f64, f64) {
        let fs: Vec<f64> = ts.iter().map(|&t| if t <= delay { 0.0 } else { 1.0 - (-(t - delay) / tau).exp() }).collect();
        let ff: f64 = fs.iter().map(|f| f * f).sum();
        if ff <= 0.0 {
            return (f64::INFINITY, 0.0);
        }
        let dy = fs.iter().zip(&ys).map(|(f, y)| f * y).sum::<f64>() / ff;
        let e: f64 = fs.iter().zip(&ys).map(|(f, y)| (y - dy * f).powi(2)).sum();
        (e, dy)
    };

    // (err, tau, delay, dy)
    let mut best = (f64::INFINITY, TAU_RANGE.0, 0.0, 0.0);
    // coarse grid (40 x 31, then refined below)
    let taus: Vec<f64> = (0..40).map(|i| TAU_RANGE.0 * (TAU_RANGE.1 / TAU_RANGE.0).powf(i as f64 / 39.0)).collect();
    let n_delays = (DELAY_MAX / 0.02) as i64 + 1; // int(0.6 / 0.02) + 1, as Python truncates it
    let delays: Vec<f64> = (0..n_delays).map(|i| i as f64 * 0.02).collect();
    for &tau in &taus {
        for &delay in &delays {
            let (e, dy) = solve(tau, delay);
            if e < best.0 {
                best = (e, tau, delay, dy);
            }
        }
    }
    for _ in 0..2 {
        // refine around the coarse optimum, inside the plausible ranges
        let (_, tau, delay, _) = best;
        for k in -6..=6 {
            let tau_c = (tau * (1.0 + 0.05 * k as f64)).max(TAU_RANGE.0).min(TAU_RANGE.1);
            for j in -5..=5 {
                let delay_c = (delay + 0.004 * j as f64).max(0.0).min(DELAY_MAX);
                let (e, dy) = solve(tau_c, delay_c);
                if e < best.0 {
                    best = (e, tau_c, delay_c, dy);
                }
            }
        }
    }
    let (e, tau, delay, dy) = best;
    if dy / du < 0.0 {
        return err("speed went the wrong way for the PWM step (encoder or motor sign)");
    }

    // The record has to reach most of the way to steady state, otherwise the
    // gain is an extrapolation: ask for a longer hold instead of guessing.
    let reached = 1.0 - (-(ts[ts.len() - 1] - delay).max(0.0) / tau).exp();
    if reached < 0.9 {
        return err(format!(
            "speed was still rising at the end of the step (time constant ~{tau:.2} s); hold the step longer"
        ));
    }

    let rmse = (e / ys.len() as f64).sqrt();
    let mean_y = mean(ys.iter().copied());
    let mut sst: f64 = ys.iter().map(|y| (y - mean_y).powi(2)).sum();
    if sst == 0.0 {
        sst = 1e-9;
    }
    let r2 = 1.0 - e / sst;
    let model = FopdtModel { gain: dy / du, tau, delay, y0, y_ss: y0 + dy, u0, u1, fit_r2: r2, fit_rmse: rmse };
    check_model(&model, pre_noise)?;
    Ok(model)
}

pub fn check_model(m: &FopdtModel, noise: f64) -> Result<(), TuningError> {
    if !(GAIN_RANGE.0 <= m.gain && m.gain <= GAIN_RANGE.1) {
        return err(format!("plant gain {:.3} rpm/count is outside ({:?}, {:?})", m.gain, GAIN_RANGE.0, GAIN_RANGE.1));
    }
    if !(TAU_RANGE.0 <= m.tau && m.tau <= TAU_RANGE.1) {
        return err(format!("time constant {:.3} s is outside ({:?}, {:?})", m.tau, TAU_RANGE.0, TAU_RANGE.1));
    }
    if m.delay > DELAY_MAX {
        return err(format!("dead time {:.3} s is implausibly long", m.delay));
    }
    let allowed = MAX_FIT_RMSE_FRACTION * (m.y_ss - m.y0).abs() + 2.0 * noise + 0.3;
    if m.fit_rmse > allowed {
        return err(format!("step response does not look first order (fit residual {:.2} rpm)", m.fit_rmse));
    }
    Ok(())
}

pub struct SimcOptions {
    pub tau_c_factor: f64,
    pub kp_max: f64,
    pub ki_max: f64,
    pub tau_min: f64,
}

impl Default for SimcOptions {
    fn default() -> Self {
        SimcOptions { tau_c_factor: 1.0, kp_max: 50.0, ki_max: 100.0, tau_min: TAU_DESIGN_MIN }
    }
}

/// SIMC PI gains for the firmware loop (counts per rpm, counts per rpm*s).
pub fn simc_pi(m: &FopdtModel, o: &SimcOptions) -> Result<Gains, TuningError> {
    let tau = m.tau.max(o.tau_min);
    let tau_c = m.delay.max(o.tau_c_factor * tau);
    if tau_c + m.delay <= 0.0 {
        return err("degenerate model");
    }
    let mut kp = tau / (m.gain * (tau_c + m.delay));
    let ti = tau.min(4.0 * (tau_c + m.delay));
    let mut ki = kp / ti;
    kp = kp.min(o.kp_max);
    ki = ki.min(o.ki_max);
    Ok(Gains { kp: round_to(kp, 4), ki: round_to(ki, 4), kd: 0.0 })
}

/// Closed-loop quality of a 0 -> target step (samples in [t_step, t_end]).
pub fn step_metrics(samples: &[Sample], t_step: f64, target: f64, t_end: Option<f64>) -> Result<StepMetrics, TuningError> {
    step_metrics_with(samples, t_step, target, t_end, 5.0, 0.8)
}

pub fn step_metrics_with(
    samples: &[Sample],
    t_step: f64,
    target: f64,
    t_end: Option<f64>,
    band_pct: f64,
    settle_window_s: f64,
) -> Result<StepMetrics, TuningError> {
    let post = sorted_by_t(
        samples.iter().filter(|s| s.t >= t_step && t_end.map_or(true, |e| s.t <= e)).copied().collect(),
    );
    if post.len() < 8 || target <= 0.0 {
        return err("not enough closed-loop samples");
    }
    let band = target.abs() * band_pct / 100.0;
    let peak = post.iter().map(|s| s.rpm).fold(f64::NEG_INFINITY, f64::max);
    let overshoot = ((peak - target) / target * 100.0).max(0.0);
    let mut settle = None;
    for (i, s) in post.iter().enumerate() {
        if post[i..].iter().all(|x| (x.rpm - target).abs() <= band) {
            settle = Some(s.t - t_step);
            break;
        }
    }
    let t_end = post[post.len() - 1].t;
    let tail = window(&post, t_end - settle_window_s, t_end);
    let ss_error = mean(tail.iter().map(|s| s.rpm)) - target;
    let t10 = post.iter().find(|s| s.rpm >= 0.1 * target).map(|s| s.t);
    let t90 = post.iter().find(|s| s.rpm >= 0.9 * target).map(|s| s.t);
    let rise = match (t10, t90) {
        (Some(a), Some(b)) => Some(b - a),
        _ => None,
    };
    Ok(StepMetrics { target, overshoot_pct: overshoot, settle_s: settle, ss_error, rise_s: rise })
}

/// Discrete PI on a FOPDT plant, mirroring the firmware loop (used by tests
/// and to sanity-check gains before they are sent to the robot).
#[cfg_attr(not(test), allow(dead_code))]
pub fn simulate_closed_loop(m: &FopdtModel, g: &Gains, target: f64, dt: f64, duration: f64) -> Vec<Sample> {
    let output_max = 200.0;
    let integral_max = 200.0;
    let n = (duration / dt) as i64;
    let delay_steps = (m.delay / dt).round() as usize;
    let mut u_hist = vec![0.0; delay_steps + 1];
    let mut y = 0.0;
    let mut integral: f64 = 0.0;
    let mut out = Vec::with_capacity(n.max(0) as usize);
    for i in 0..n {
        let t = i as f64 * dt;
        let e = target - y;
        integral = (integral + e * dt).max(-integral_max).min(integral_max);
        let u = (g.kp * e + g.ki * integral).max(-output_max).min(output_max);
        u_hist.push(u);
        let u_delayed = u_hist[u_hist.len() - 1 - delay_steps];
        // first-order plant, exact discretisation
        let a = (-dt / m.tau).exp();
        y = a * y + (1.0 - a) * m.gain * u_delayed;
        out.push(Sample { t, pwm: u, rpm: y });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    /// Deterministic Gaussian noise (the Python tests use random.Random(seed).gauss;
    /// the exact stream does not matter, every assertion carries a tolerance).
    struct Noise(u64);
    impl Noise {
        fn uniform(&mut self) -> f64 {
            self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
        }
        fn gauss(&mut self, sigma: f64) -> f64 {
            let u1 = self.uniform().max(1e-12);
            let u2 = self.uniform();
            sigma * (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
        }
    }

    /// Simulated STM32 0x85 stream for a u0 -> u1 PWM step at t=pre.
    fn open_loop_response(gain: f64, tau: f64, delay: f64, u0: f64, u1: f64, dt: f64, noise: f64) -> Vec<Sample> {
        let (pre, post) = (0.5, 2.5);
        let mut rng = Noise(1);
        let mut y = gain * u0;
        let n = ((pre + post) / dt) as i64;
        let d = (delay / dt).round() as usize;
        let mut hist = vec![u0; d + 1];
        let mut samples = Vec::new();
        for i in 0..n {
            let t = i as f64 * dt;
            let u = if t >= pre { u1 } else { u0 };
            hist.push(u);
            let u_d = hist[hist.len() - 1 - d];
            let a = (-dt / tau).exp();
            y = a * y + (1.0 - a) * gain * u_d;
            samples.push(Sample { t, pwm: u, rpm: y + rng.gauss(noise) });
        }
        samples
    }

    fn model(gain: f64, tau: f64, delay: f64) -> FopdtModel {
        FopdtModel { gain, tau, delay, y0: 0.0, y_ss: 0.0, u0: 0.0, u1: 0.0, fit_r2: 1.0, fit_rmse: 0.0 }
    }

    #[test]
    fn fit_recovers_plant_within_tolerance() {
        let s = open_loop_response(0.29, 0.25, 0.05, 80.0, 140.0, 0.05, 0.3);
        let m = fit_fopdt(&s, 0.5, None).unwrap();
        assert!((m.gain - 0.29).abs() / 0.29 < 0.10, "{m:?}");
        assert!((m.tau - 0.25).abs() / 0.25 < 0.15, "{m:?}");
        assert!(m.delay <= 0.12, "{m:?}");
        assert!(m.fit_rmse < 0.5, "{m:?}");
    }

    #[test]
    fn fit_ignores_samples_after_t_end() {
        let s = open_loop_response(0.29, 0.25, 0.05, 80.0, 140.0, 0.05, 0.2);
        let t_end = s[s.len() - 1].t;
        let mut all = s.clone();
        for i in 1..8 {
            all.push(Sample { t: t_end + 0.05 * i as f64, pwm: 0.0, rpm: 40.6 * (-0.05 * i as f64 / 0.25).exp() });
        }
        assert!(fit_fopdt(&all, 0.5, None).is_err());
        let m = fit_fopdt(&all, 0.5, Some(t_end)).unwrap();
        assert!((m.gain - 0.29).abs() / 0.29 < 0.10);
        let met = step_metrics(&all, 0.5, 40.6, Some(t_end)).unwrap();
        assert!(met.settle_s.is_some());
    }

    #[test]
    fn fit_rejects_no_response() {
        let s: Vec<Sample> = (0..60)
            .map(|i| Sample { t: i as f64 * 0.05, pwm: if i as f64 * 0.05 >= 0.5 { 140.0 } else { 80.0 }, rpm: 0.0 })
            .collect();
        let e = fit_fopdt(&s, 0.5, None).unwrap_err();
        assert!(e.0.contains("did not respond"), "{e}");
    }

    #[test]
    fn fit_rejects_wrong_sign() {
        let s = open_loop_response(0.29, 0.25, 0.05, 80.0, 140.0, 0.05, 0.0);
        let flipped: Vec<Sample> = s.iter().map(|x| Sample { t: x.t, pwm: x.pwm, rpm: -x.rpm }).collect();
        let e = fit_fopdt(&flipped, 0.5, None).unwrap_err();
        assert!(e.0.contains("wrong way"), "{e}");
    }

    #[test]
    fn fit_rejects_missing_step() {
        let s = open_loop_response(0.29, 0.25, 0.05, 80.0, 140.0, 0.05, 0.0);
        assert!(fit_fopdt(&s, 2.9, None).is_err());
    }

    #[test]
    fn fit_rejects_unsettled_response() {
        // tau 1.2 s needs ~4 s to settle; a 2.5 s hold must not silently extrapolate
        let s = open_loop_response(0.29, 1.2, 0.2, 80.0, 140.0, 0.05, 0.2);
        let e = fit_fopdt(&s, 0.5, None).unwrap_err();
        assert!(e.0.contains("still rising"), "{e}");
    }

    #[test]
    fn fit_tolerates_slightly_unsettled_response() {
        // tau 0.9 s reaches 92 % in 2.5 s; the fitted steady state is still right
        let s = open_loop_response(0.29, 0.9, 0.1, 80.0, 140.0, 0.05, 0.2);
        let m = fit_fopdt(&s, 0.5, None).unwrap();
        assert!((m.gain - 0.29).abs() / 0.29 < 0.10, "{m:?}");
        assert!((m.tau - 0.9).abs() / 0.9 < 0.20, "{m:?}");
    }

    #[test]
    fn simc_gains_give_a_well_damped_loop() {
        for (tau, delay) in [(0.1, 0.0), (0.25, 0.05), (0.6, 0.1), (0.9, 0.15)] {
            let s = open_loop_response(0.29, tau, delay, 80.0, 140.0, 0.05, 0.2);
            let m = fit_fopdt(&s, 0.5, None).unwrap();
            let g = simc_pi(&m, &SimcOptions::default()).unwrap();
            assert!(g.kp > 0.0 && g.ki > 0.0 && g.kd == 0.0);
            let sim = simulate_closed_loop(&model(0.29, tau, delay), &g, 29.0, 0.02, 4.0);
            let met = step_metrics(&sim, 0.0, 29.0, None).unwrap();
            assert!(met.overshoot_pct < 25.0, "{tau} {delay}: {met:?}");
            assert!(matches!(met.settle_s, Some(t) if t < 3.0), "{tau} {delay}: {met:?}");
            assert!(met.ss_error.abs() < 0.5, "{tau} {delay}: {met:?}");
        }
    }

    #[test]
    fn step_metrics_on_ideal_response() {
        let s: Vec<Sample> =
            (0..60).map(|i| Sample { t: i as f64 * 0.05, pwm: 100.0, rpm: if i >= 4 { 29.0 } else { 0.0 } }).collect();
        let met = step_metrics(&s, 0.0, 29.0, None).unwrap();
        assert_eq!(met.overshoot_pct, 0.0);
        assert!((met.settle_s.unwrap() - 0.2).abs() < 1e-9);
        assert_eq!(met.ss_error, 0.0);
    }

    #[test]
    fn step_metrics_detects_overshoot_and_never_settling() {
        let s: Vec<Sample> = (0..60)
            .map(|i| Sample { t: i as f64 * 0.05, pwm: 100.0, rpm: 29.0 * if i % 2 == 1 { 1.3 } else { 0.7 } })
            .collect();
        let met = step_metrics(&s, 0.0, 29.0, None).unwrap();
        assert!((met.overshoot_pct - 30.0).abs() < 1e-6, "{met:?}");
        assert!(met.settle_s.is_none());
    }

    #[test]
    fn fit_of_a_sample_fast_wheel_stays_in_range() {
        // the real base (2026-09-19 run): the speed settles within one 50 ms sample,
        // the refinement must not walk tau below TAU_RANGE and then reject it
        let s = open_loop_response(0.3, 0.03, 0.04, 80.0, 140.0, 0.05, 0.3);
        let m = fit_fopdt(&s, 0.5, None).unwrap();
        assert!(TAU_RANGE.0 <= m.tau && m.tau <= 0.12, "{m:?}");
        assert!((m.gain - 0.3).abs() / 0.3 < 0.1, "{m:?}");
    }

    #[test]
    fn simc_designs_against_a_tau_floor() {
        // the real base: tau at the fit's lower bound, 44 ms dead time, K 0.3
        let m = FopdtModel { gain: 0.296, tau: 0.0207, delay: 0.044, y0: 21.0, y_ss: 39.0, u0: 80.0, u1: 140.0, fit_r2: 0.6, fit_rmse: 0.0 };
        let g = simc_pi(&m, &SimcOptions::default()).unwrap();
        assert!(((g.kp / g.ki) - TAU_DESIGN_MIN).abs() / TAU_DESIGN_MIN < 1e-3, "{g:?}"); // Ti = tau floor
        assert!(1.0 < g.kp && g.kp < 3.0 && 20.0 < g.ki && g.ki < 60.0, "{g:?}");
        let sim = simulate_closed_loop(&model(0.296, 0.04, 0.04), &g, 29.0, 0.02, 3.0);
        let met = step_metrics(&sim, 0.0, 29.0, None).unwrap();
        assert!(met.overshoot_pct < 25.0 && matches!(met.settle_s, Some(t) if t < 1.0), "{met:?}");
    }

    #[test]
    fn gains_are_clamped() {
        let m = model(0.02, 3.0, 0.0);
        let g = simc_pi(&m, &SimcOptions { tau_c_factor: 0.01, kp_max: 10.0, ki_max: 5.0, ..SimcOptions::default() }).unwrap();
        assert_eq!(g.kp, 10.0);
        assert_eq!(g.ki, 5.0);
    }

    /// Golden vectors produced by `mower_mission.pid_tuning` on fixed inputs
    /// (`tests/pid_tuning_golden.json`, generated by the differential harness):
    /// the same numbers to 1e-9, so the Rust maths is the Python maths.
    #[test]
    fn matches_python_golden_vectors() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/pid_tuning_golden.json");
        let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("golden file {path}: {e}"));
        let cases: Vec<serde_json::Value> = serde_json::from_str(&text).unwrap();
        assert!(cases.len() >= 15);
        let close = |a: f64, b: f64| (a - b).abs() <= 1e-9 * (1.0 + a.abs().max(b.abs()));
        let close_opt = |name: &str, what: &str, a: Option<f64>, b: Option<f64>| match (a, b) {
            (None, None) => {}
            (Some(a), Some(b)) => assert!(close(a, b), "{name}: {what} rust {a} python {b}"),
            _ => panic!("{name}: {what} rust {a:?} python {b:?}"),
        };
        let check_metrics = |name: &str, met: &StepMetrics, want: &StepMetrics| {
            assert!(close(met.target, want.target), "{name}: target");
            assert!(close(met.overshoot_pct, want.overshoot_pct), "{name}: overshoot rust {} python {}", met.overshoot_pct, want.overshoot_pct);
            assert!(close(met.ss_error, want.ss_error), "{name}: ss_error rust {} python {}", met.ss_error, want.ss_error);
            close_opt(name, "settle_s", met.settle_s, want.settle_s);
            close_opt(name, "rise_s", met.rise_s, want.rise_s);
        };
        let mut checked = 0;
        for case in &cases {
            let name = case["name"].as_str().unwrap();
            if let Some(m) = case.get("fixed_model") {
                let m: FopdtModel = serde_json::from_value(m.clone()).unwrap();
                let g = simc_pi(&m, &SimcOptions::default()).unwrap();
                let want_g: Gains = serde_json::from_value(case["gains"].clone()).unwrap();
                assert_eq!(g, want_g, "{name}: gains");
                checked += 1;
                continue;
            }
            let samples: Vec<Sample> = case["samples"]
                .as_array()
                .unwrap()
                .iter()
                .map(|s| Sample { t: s[0].as_f64().unwrap(), pwm: s[1].as_f64().unwrap(), rpm: s[2].as_f64().unwrap() })
                .collect();
            let t_step = case["t_step"].as_f64().unwrap();
            let t_end = case["t_end"].as_f64();
            if !case.get("metrics_only").and_then(Value::as_bool).unwrap_or(false) {
                match fit_fopdt(&samples, t_step, t_end) {
                    Ok(m) => {
                        let want: FopdtModel = serde_json::from_value(case["model"].clone())
                            .unwrap_or_else(|_| panic!("{name}: python raised {:?} but rust fitted {m:?}", case["error"]));
                        for (k, a, b) in [
                            ("gain", m.gain, want.gain),
                            ("tau", m.tau, want.tau),
                            ("delay", m.delay, want.delay),
                            ("y0", m.y0, want.y0),
                            ("y_ss", m.y_ss, want.y_ss),
                            ("u0", m.u0, want.u0),
                            ("u1", m.u1, want.u1),
                            ("fit_r2", m.fit_r2, want.fit_r2),
                            ("fit_rmse", m.fit_rmse, want.fit_rmse),
                        ] {
                            assert!(close(a, b), "{name}: {k} rust {a} python {b}");
                        }
                        let g = simc_pi(&m, &SimcOptions::default()).unwrap();
                        let want_g: Gains = serde_json::from_value(case["gains"].clone()).unwrap();
                        assert_eq!(g, want_g, "{name}: gains");
                    }
                    Err(e) => {
                        let want = case["error"].as_str().unwrap_or_else(|| panic!("{name}: python fitted but rust raised {e}"));
                        assert_eq!(e.0, want, "{name}: error text");
                    }
                }
                checked += 1;
            }
            if let Some(target) = case["target"].as_f64() {
                match step_metrics(&samples, t_step, target, t_end) {
                    Ok(met) => {
                        let want: StepMetrics = serde_json::from_value(case["metrics"].clone())
                            .unwrap_or_else(|_| panic!("{name}: python raised {:?} for metrics but rust computed {met:?}", case["metrics_error"]));
                        check_metrics(name, &met, &want);
                    }
                    Err(e) => {
                        let want = case["metrics_error"].as_str().unwrap_or_else(|| panic!("{name}: python computed metrics but rust raised {e}"));
                        assert_eq!(e.0, want, "{name}: metrics error text");
                    }
                }
                checked += 1;
            }
        }
        assert!(checked >= cases.len());
    }
}
