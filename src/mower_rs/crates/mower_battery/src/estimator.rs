//! State-of-charge estimate for the 6S Li-ion main pack: a line-for-line
//! port of `mower_mission/battery_estimator.py` (see its docstring and
//! docs/BATTERY.md for the model). All the arithmetic is f64 like Python's.

/// Per-cell open-circuit voltage -> fraction of charge, typical NMC 18650
/// (3.0 V cut-off, 4.2 V full). Sorted by voltage.
pub const DEFAULT_OCV_TABLE: [(f64, f64); 12] = [
    (3.00, 0.00),
    (3.30, 0.03),
    (3.50, 0.10),
    (3.60, 0.20),
    (3.66, 0.30),
    (3.72, 0.40),
    (3.78, 0.50),
    (3.85, 0.60),
    (3.92, 0.70),
    (4.00, 0.80),
    (4.10, 0.90),
    (4.20, 1.00),
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Unknown,
    Discharging,
    Charging,
    Full,
    /// Charger present but no current.
    NotCharging,
}

/// Interpolate the OCV table; clamps outside its range.
pub fn ocv_fraction(cell_v: f64, table: &[(f64, f64)]) -> f64 {
    if !cell_v.is_finite() {
        return 0.0;
    }
    if cell_v <= table[0].0 {
        return table[0].1;
    }
    let last = table[table.len() - 1];
    if cell_v >= last.0 {
        return last.1;
    }
    for w in table.windows(2) {
        let ((v0, f0), (v1, f1)) = (w[0], w[1]);
        if v0 <= cell_v && cell_v <= v1 {
            if v1 == v0 {
                return f1;
            }
            return f0 + (f1 - f0) * (cell_v - v0) / (v1 - v0);
        }
    }
    last.1
}

#[derive(Clone, Debug)]
pub struct Config {
    pub cell_count: usize,
    pub filter_tau_s: f64,
    /// 1 %/min while discharging.
    pub recovery_rate_per_s: f64,
    /// 0.5 %/min = ~3.3 h charge.
    pub charge_rate_per_s: f64,
    pub post_charge_settle_s: f64,
    /// NaN = unknown (voltage only).
    pub capacity_ah: f64,
    pub rest_current_a: f64,
    pub rest_hold_s: f64,
    pub full_tail_current_a: f64,
    pub full_hold_s: f64,
    pub full_min_cell_v: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            cell_count: 6,
            filter_tau_s: 20.0,
            recovery_rate_per_s: 0.01 / 60.0,
            charge_rate_per_s: 0.005 / 60.0,
            post_charge_settle_s: 300.0,
            capacity_ah: f64::NAN,
            rest_current_a: 0.1,
            rest_hold_s: 300.0,
            full_tail_current_a: 0.2,
            full_hold_s: 60.0,
            full_min_cell_v: 4.10,
        }
    }
}

/// Track SOC from pack voltage, charger presence and (optionally) current.
#[derive(Clone, Debug)]
pub struct BatteryEstimator {
    pub cfg: Config,
    /// Filtered pack voltage.
    pub voltage_v: f64,
    pub raw_voltage_v: f64,
    /// 0..1
    pub fraction: f64,
    pub status: Status,
    /// Signed, + = charging.
    pub current_a: f64,
    pub charger_present: bool,
    last_t: Option<f64>,
    tail_since: Option<f64>,
    rest_since: Option<f64>,
    /// Pending post-charge re-anchor.
    charger_removed_t: Option<f64>,
}

impl BatteryEstimator {
    pub fn new(cfg: Config) -> Result<Self, String> {
        if cfg.cell_count < 1 {
            return Err("cell_count must be >= 1".into());
        }
        if cfg.filter_tau_s < 0.0 {
            return Err("filter_tau_s must be non-negative".into());
        }
        Ok(Self {
            cfg,
            voltage_v: f64::NAN,
            raw_voltage_v: f64::NAN,
            fraction: f64::NAN,
            status: Status::Unknown,
            current_a: f64::NAN,
            charger_present: false,
            last_t: None,
            tail_since: None,
            rest_since: None,
            charger_removed_t: None,
        })
    }

    pub fn cell_count(&self) -> usize {
        self.cfg.cell_count
    }

    pub fn capacity_ah(&self) -> f64 {
        self.cfg.capacity_ah
    }

    pub fn percentage(&self) -> f64 {
        if self.fraction.is_nan() { f64::NAN } else { self.fraction * 100.0 }
    }

    pub fn cell_voltage_v(&self) -> f64 {
        self.voltage_v / self.cfg.cell_count as f64
    }

    /// True when the last sample carried a current and the capacity is known.
    pub fn coulomb_counting(&self) -> bool {
        self.current_a.is_finite() && self.cfg.capacity_ah > 0.0
    }

    /// Feed one sample; returns the new fraction (0..1).
    pub fn update(&mut self, t: f64, pack_voltage_v: f64, charger_present: bool, current_a: f64) -> f64 {
        if !pack_voltage_v.is_finite() || pack_voltage_v <= 0.0 {
            return self.fraction;
        }
        let dt = match self.last_t {
            None => 0.0,
            Some(last) => (t - last).max(0.0),
        };
        self.last_t = Some(t);
        self.raw_voltage_v = pack_voltage_v;
        self.current_a = if current_a.is_finite() { current_a } else { f64::NAN };

        if self.voltage_v.is_nan() || self.cfg.filter_tau_s <= 0.0 {
            self.voltage_v = self.raw_voltage_v;
        } else {
            let alpha = if dt > 0.0 { dt / (self.cfg.filter_tau_s + dt) } else { 0.0 };
            self.voltage_v += alpha * (self.raw_voltage_v - self.voltage_v);
        }

        let ocv = ocv_fraction(self.cell_voltage_v(), &DEFAULT_OCV_TABLE);

        if charger_present && !self.charger_present {
            self.charger_removed_t = None;
        } else if self.charger_present && !charger_present {
            self.charger_removed_t = Some(t);
        }
        self.charger_present = charger_present;

        if self.coulomb_counting() && dt > 0.0 && !self.fraction.is_nan() {
            self.fraction += self.current_a * dt / (self.cfg.capacity_ah * 3600.0);
        }

        if charger_present {
            self.rest_since = None;
            self.update_charging(t, dt, ocv);
        } else {
            self.tail_since = None;
            self.update_discharging(t, dt, ocv);
        }
        if !self.fraction.is_nan() {
            self.fraction = self.fraction.clamp(0.0, 1.0);
        }
        self.fraction
    }

    fn update_charging(&mut self, t: f64, dt: f64, ocv: f64) {
        // Terminal voltage under charge is the charger's CV and says nothing
        // about the charge state, so the OCV is never looked up here.
        if self.fraction.is_nan() {
            // First sample ever and already on the charger: the OCV is an
            // over-estimate, so start below it.
            self.fraction = ocv.min(0.99);
        } else if !self.coulomb_counting() && (!self.current_a.is_finite() || self.current_a >= 0.05) {
            // No capacity to integrate against: assume the charger's C-rate.
            self.fraction += self.cfg.charge_rate_per_s * dt;
        }

        let tail = self.current_a.is_finite()
            && self.current_a <= self.cfg.full_tail_current_a
            && self.cell_voltage_v() >= self.cfg.full_min_cell_v;
        if tail {
            if self.tail_since.is_none() {
                self.tail_since = Some(t);
            }
            if t - self.tail_since.unwrap() >= self.cfg.full_hold_s {
                self.fraction = 1.0;
                self.status = Status::Full;
                return;
            }
        } else {
            self.tail_since = None;
        }

        if self.status == Status::Full && (!self.current_a.is_finite() || tail) {
            // Stay full while parked on the charger.
            return;
        }
        if self.current_a.is_finite() && self.current_a < 0.05 && !tail {
            // Charger connected but nothing flowing and not at CV: a charger
            // that has cut out, or one that has not started.
            self.status = Status::NotCharging;
        } else {
            self.status = Status::Charging;
        }
        self.fraction = self.fraction.min(0.99);
    }

    fn update_discharging(&mut self, t: f64, dt: f64, ocv: f64) {
        self.status = Status::Discharging;
        if self.fraction.is_nan() {
            self.fraction = ocv;
            return;
        }

        if self.coulomb_counting() {
            // Drift correction: re-anchor to the OCV once the pack has rested.
            if self.current_a.abs() < self.cfg.rest_current_a {
                if self.rest_since.is_none() {
                    self.rest_since = Some(t);
                }
                if t - self.rest_since.unwrap() >= self.cfg.rest_hold_s {
                    self.fraction = ocv;
                }
            } else {
                self.rest_since = None;
            }
            self.charger_removed_t = None;
            return;
        }

        // Voltage only. Right after the charger is removed the terminal
        // voltage is still inflated; wait for it to relax, then snap once.
        if let Some(removed) = self.charger_removed_t {
            if t - removed >= self.cfg.post_charge_settle_s {
                self.fraction = ocv;
                self.charger_removed_t = None;
            }
            return;
        }

        if ocv < self.fraction {
            self.fraction = ocv;
        } else {
            self.fraction = ocv.min(self.fraction + self.cfg.recovery_rate_per_s * dt);
        }
    }
}

#[cfg(test)]
mod tests {
    //! The vectors of `test/test_battery_estimator.py`, verbatim.
    use super::*;

    fn approx(a: f64, b: f64, abs: f64) -> bool {
        (a - b).abs() <= abs
    }
    fn est(cfg: Config) -> BatteryEstimator {
        BatteryEstimator::new(cfg).unwrap()
    }
    fn cfg6(filter_tau_s: f64) -> Config {
        Config { cell_count: 6, filter_tau_s, ..Config::default() }
    }
    const NAN: f64 = f64::NAN;

    #[test]
    fn ocv_table_endpoints_and_interpolation() {
        let t = &DEFAULT_OCV_TABLE;
        assert!(approx(ocv_fraction(2.5, t), 0.0, 1e-9));
        assert!(approx(ocv_fraction(3.0, t), 0.0, 1e-9));
        assert!(approx(ocv_fraction(4.2, t), 1.0, 1e-9));
        assert!(approx(ocv_fraction(4.5, t), 1.0, 1e-9));
        assert!(approx(ocv_fraction(3.75, t), 0.45, 1e-9));
        assert!(approx(ocv_fraction(NAN, t), 0.0, 1e-9));
    }

    #[test]
    fn first_sample_is_taken_as_is() {
        let mut e = est(cfg6(20.0));
        let frac = e.update(0.0, 6.0 * 3.85, false, NAN);
        assert!(approx(frac, 0.60, 1e-9));
        assert!(approx(e.percentage(), 60.0, 1e-6));
        assert_eq!(e.status, Status::Discharging);
        assert!(approx(e.voltage_v, 23.1, 1e-9));
    }

    #[test]
    fn ignores_invalid_voltage() {
        let mut e = est(Config::default());
        assert!(e.update(0.0, 0.0, false, NAN).is_nan());
        assert!(e.update(1.0, NAN, false, NAN).is_nan());
        e.update(2.0, 24.0, false, NAN);
        assert!(!e.fraction.is_nan());
    }

    #[test]
    fn short_sag_barely_moves_filtered_estimate() {
        let mut e = est(cfg6(20.0));
        e.update(0.0, 6.0 * 3.85, false, NAN);
        let mut t = 0.0;
        for _ in 0..10 {
            t += 0.2;
            e.update(t, 6.0 * 3.65, false, NAN);
        }
        assert!(e.fraction > 0.55);
        assert!(e.voltage_v > 6.0 * 3.80);
    }

    #[test]
    fn sustained_drop_lowers_estimate_and_rebound_is_rate_limited() {
        let mut e = est(Config { cell_count: 6, filter_tau_s: 1.0, recovery_rate_per_s: 0.01 / 60.0, ..Config::default() });
        e.update(0.0, 6.0 * 3.85, false, NAN);
        let mut t = 0.0;
        for _ in 0..60 {
            t += 1.0;
            e.update(t, 6.0 * 3.66, false, NAN);
        }
        assert!(approx(e.fraction, 0.30, 0.01));
        for _ in 0..60 {
            t += 1.0;
            e.update(t, 6.0 * 3.85, false, NAN);
        }
        assert!(approx(e.fraction, 0.31, 0.005));
        assert_eq!(e.status, Status::Discharging);
    }

    #[test]
    fn voltage_only_charging_ramps_from_last_estimate() {
        let mut e = est(Config { cell_count: 6, filter_tau_s: 0.0, charge_rate_per_s: 0.005 / 60.0, ..Config::default() });
        e.update(0.0, 6.0 * 3.72, false, NAN);
        e.update(1.0, 25.6, true, NAN);
        assert_eq!(e.status, Status::Charging);
        assert!(approx(e.fraction, 0.40, 0.001));
        e.update(1.0 + 20.0 * 60.0, 25.6, true, NAN);
        assert!(approx(e.fraction, 0.50, 0.001));
        e.update(1.0 + 10.0 * 3600.0, 25.6, true, NAN);
        assert!(approx(e.fraction, 0.99, 1e-9));
        assert_eq!(e.status, Status::Charging);
        assert!(e.current_a.is_nan());
    }

    #[test]
    fn charger_removed_reanchors_to_ocv_after_settling() {
        let mut e = est(Config { cell_count: 6, filter_tau_s: 0.0, post_charge_settle_s: 300.0, ..Config::default() });
        e.update(0.0, 6.0 * 3.72, false, NAN);
        e.update(1.0, 25.6, true, NAN);
        e.update(3600.0, 25.6, true, NAN);
        assert!(approx(e.fraction, 0.70, 0.001));
        e.update(3601.0, 25.2, false, NAN);
        assert_eq!(e.status, Status::Discharging);
        assert!(approx(e.fraction, 0.70, 0.001));
        e.update(3601.0 + 200.0, 24.2, false, NAN);
        assert!(approx(e.fraction, 0.70, 0.001));
        e.update(3601.0 + 300.0, 24.04, false, NAN);
        assert!(approx(e.fraction, 0.8, 0.01));
        e.update(3601.0 + 301.0, 24.04, false, NAN);
        assert!(approx(e.fraction, 0.8, 0.01));
    }

    #[test]
    fn first_sample_on_charger_starts_below_full() {
        let mut e = est(cfg6(0.0));
        e.update(0.0, 25.6, true, NAN);
        assert!(approx(e.fraction, 0.99, 1e-9));
        assert_eq!(e.status, Status::Charging);
    }

    #[test]
    fn full_after_tail_current_held_at_cv() {
        let mut e = est(Config { cell_count: 6, filter_tau_s: 0.0, full_tail_current_a: 0.2, full_hold_s: 60.0, ..Config::default() });
        e.update(0.0, 6.0 * 3.72, false, NAN);
        e.update(1.0, 6.0 * 4.20, true, 0.15);
        assert_eq!(e.status, Status::Charging);
        e.update(31.0, 6.0 * 4.20, true, 0.15);
        assert!(e.fraction < 1.0);
        e.update(61.0, 6.0 * 4.20, true, 0.15);
        assert!(approx(e.fraction, 1.0, 1e-9));
        assert_eq!(e.status, Status::Full);
        e.update(120.0, 6.0 * 4.20, true, 0.0);
        assert_eq!(e.status, Status::Full);
        assert!(approx(e.fraction, 1.0, 1e-9));
        e.update(121.0, 6.0 * 4.20, true, 0.5);
        assert_eq!(e.status, Status::Charging);
    }

    #[test]
    fn charger_present_without_current_below_cv_is_not_charging() {
        let mut e = est(cfg6(0.0));
        e.update(0.0, 6.0 * 3.85, false, NAN);
        e.update(1.0, 6.0 * 3.90, true, 0.0);
        assert_eq!(e.status, Status::NotCharging);
        assert!(approx(e.fraction, 0.60, 1e-9));
    }

    #[test]
    fn unplugged_mid_charge_with_current_is_not_full() {
        let mut e = est(Config { cell_count: 6, filter_tau_s: 0.0, capacity_ah: 20.0, ..Config::default() });
        e.update(0.0, 6.0 * 3.72, false, NAN);
        e.update(1.0, 25.6, true, 2.0);
        assert_eq!(e.status, Status::Charging);
        for t in [2.0, 30.0, 61.0, 120.0] {
            e.update(t, 6.0 * 3.72, false, 0.0);
        }
        assert_eq!(e.status, Status::Discharging);
        assert!(approx(e.fraction, 0.40, 0.01));
    }

    #[test]
    fn coulomb_counting_discharge_and_rest_reanchor() {
        let mut e = est(Config { cell_count: 6, filter_tau_s: 0.0, capacity_ah: 10.0, rest_current_a: 0.1, rest_hold_s: 300.0, ..Config::default() });
        e.update(0.0, 6.0 * 4.00, false, NAN);
        let mut t = 0.0;
        for _ in 0..30 {
            t += 60.0;
            e.update(t, 6.0 * 3.70, false, -5.0);
        }
        assert!(approx(e.fraction, 0.55, 0.005));
        assert_eq!(e.status, Status::Discharging);
        for _ in 0..5 {
            t += 60.0;
            e.update(t, 6.0 * 3.80, false, 0.02);
        }
        assert!(approx(e.fraction, 0.55, 0.005));
        t += 60.0;
        e.update(t, 6.0 * 3.80, false, 0.02);
        assert!(approx(e.fraction, ocv_fraction(3.80, &DEFAULT_OCV_TABLE), 0.005));
    }

    #[test]
    fn coulomb_counting_charge_then_full() {
        let mut e = est(Config { cell_count: 6, filter_tau_s: 0.0, capacity_ah: 10.0, full_tail_current_a: 0.2, full_hold_s: 60.0, ..Config::default() });
        e.update(0.0, 6.0 * 3.72, false, NAN);
        let mut t = 0.0;
        for _ in 0..60 {
            t += 60.0;
            e.update(t, 25.2, true, 2.0);
        }
        assert!(approx(e.fraction, 0.60, 0.005));
        assert_eq!(e.status, Status::Charging);
        for _ in 0..300 {
            t += 60.0;
            e.update(t, 25.2, true, 1.0);
        }
        assert!(approx(e.fraction, 0.99, 1e-9));
        e.update(t + 1.0, 25.2, true, 0.1);
        e.update(t + 61.0, 25.2, true, 0.1);
        assert_eq!(e.status, Status::Full);
        assert!(approx(e.fraction, 1.0, 1e-9));
    }

    #[test]
    fn single_cell_aon_battery() {
        let mut e = est(Config { cell_count: 1, filter_tau_s: 0.0, ..Config::default() });
        assert!(approx(e.update(0.0, 3.78, false, NAN), 0.5, 1e-9));
        assert!(approx(e.cell_voltage_v(), 3.78, 1e-9));
    }
}
