//! The truth world: one robot on a flat field.
//!
//! The wheels are fake_base.py's: each a first-order lag (0.15 s) towards
//! the commanded rpm, integrated into revolutions and encoder counts. The
//! body is a differential drive on the wheels mower_base drives with
//! (geometry::Wheels), so /odom is the truth unless `slip` says otherwise.
//! The pack drains with what the wheels and the blade draw.

use crate::geometry::Wheels;

pub const WHEEL_TAU_S: f64 = 0.15;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Wheel {
    pub target_rpm: f64,
    pub rpm: f64,
    /// revolutions since the start
    pub rev: f64,
}

impl Wheel {
    fn step(&mut self, dt: f64) {
        let alpha = 1.0 - (-dt / WHEEL_TAU_S).exp();
        self.rpm += (self.target_rpm - self.rpm) * alpha;
        self.rev += self.rpm / 60.0 * dt;
    }

    /// The STM32's signed 32-bit total, wrapping like the firmware's.
    pub fn counts(&self, counts_per_rev: f64) -> i32 {
        ((self.rev * counts_per_rev).round() as i64) as i32
    }
}

/// Per-cell open-circuit voltage -> fraction of charge: mower_battery's
/// DEFAULT_OCV_TABLE (tests::the_ocv_table_is_mower_batterys).
pub const OCV_TABLE: [(f64, f64); 12] = [
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

fn interpolate(x: f64, points: impl Iterator<Item = (f64, f64)> + Clone) -> f64 {
    let pts: Vec<(f64, f64)> = points.collect();
    if x <= pts[0].0 {
        return pts[0].1;
    }
    for w in pts.windows(2) {
        if x <= w[1].0 {
            return w[0].1 + (w[1].1 - w[0].1) * (x - w[0].0) / (w[1].0 - w[0].0);
        }
    }
    pts[pts.len() - 1].1
}

/// The 6S main pack: charge drawn down by the current, voltage from the OCV
/// curve minus the internal resistance.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Battery {
    pub fraction: f64,
    pub capacity_ah: f64,
    pub cells: u32,
    pub r_ohm: f64,
    pub current_a: f64,
}

impl Battery {
    pub const IDLE_A: f64 = 0.6;
    /// per wheel at max_rpm
    pub const WHEEL_A: f64 = 1.2;
    /// the blade at 1000 permille
    pub const BLADE_A: f64 = 5.0;

    /// A pack resting at `volts`.
    pub fn at_rest(volts: f64, capacity_ah: f64) -> Battery {
        let cells = 6;
        let fraction = interpolate(volts / cells as f64, OCV_TABLE.iter().copied());
        Battery { fraction, capacity_ah, cells, r_ohm: 0.04, current_a: Self::IDLE_A }
    }

    pub fn voltage(&self) -> f64 {
        let cell = interpolate(self.fraction, OCV_TABLE.iter().map(|&(v, f)| (f, v)));
        (cell * self.cells as f64 - self.current_a * self.r_ohm).max(0.0)
    }

    fn draw(&mut self, current_a: f64, dt: f64) {
        self.current_a = current_a;
        self.fraction = (self.fraction - current_a * dt / 3600.0 / self.capacity_ah).max(0.0);
    }
}

#[derive(Debug, Clone)]
pub struct World {
    pub wheels: Wheels,
    pub left: Wheel,
    pub right: Wheel,
    /// base_footprint in the start's east / north plane, metres, and its
    /// ENU heading, radians
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    /// body velocities (forward m/s, yaw rate rad/s)
    pub v: f64,
    pub w: f64,
    /// body-frame acceleration (forward, left), m/s^2
    pub accel: [f64; 2],
    /// fraction of the wheels' travel lost on the ground
    pub slip: f64,
    /// blade duty, 0..1
    pub blade: f64,
    pub battery: Battery,
    /// metres driven (truth)
    pub odometer: f64,
}

impl World {
    pub fn new(wheels: Wheels, yaw: f64, battery: Battery) -> World {
        World {
            wheels,
            left: Wheel::default(),
            right: Wheel::default(),
            x: 0.0,
            y: 0.0,
            yaw,
            v: 0.0,
            w: 0.0,
            accel: [0.0; 2],
            slip: 0.0,
            blade: 0.0,
            battery,
            odometer: 0.0,
        }
    }

    /// 0x01's permille: the wheels' new targets.
    pub fn command(&mut self, left_permille: i16, right_permille: i16) {
        self.left.target_rpm = left_permille as f64 / 1000.0 * self.wheels.max_rpm;
        self.right.target_rpm = right_permille as f64 / 1000.0 * self.wheels.max_rpm;
    }

    pub fn step(&mut self, dt: f64) {
        self.left.step(dt);
        self.right.step(dt);
        let rad_s = |rpm: f64| rpm / 60.0 * std::f64::consts::TAU;
        let vl = rad_s(self.left.rpm) * self.wheels.left_radius;
        let vr = rad_s(self.right.rpm) * self.wheels.right_radius;
        let ground = 1.0 - self.slip;
        let v = (vr + vl) / 2.0 * ground;
        let w = (vr - vl) / self.wheels.separation * ground;
        self.accel = [(v - self.v) / dt, v * w];
        // exact arc for the step
        if w.abs() < 1e-9 {
            self.x += v * self.yaw.cos() * dt;
            self.y += v * self.yaw.sin() * dt;
        } else {
            let yaw1 = self.yaw + w * dt;
            self.x += v / w * (yaw1.sin() - self.yaw.sin());
            self.y -= v / w * (yaw1.cos() - self.yaw.cos());
        }
        self.yaw = (self.yaw + w * dt + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI;
        self.v = v;
        self.w = w;
        self.odometer += v.abs() * dt;
        let max = self.wheels.max_rpm;
        let current = Battery::IDLE_A
            + Battery::WHEEL_A * (self.left.rpm.abs() + self.right.rpm.abs()) / max
            + Battery::BLADE_A * self.blade;
        self.battery.draw(current, dt);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wheels() -> Wheels {
        Wheels::from_params(None).unwrap()
    }

    fn run(world: &mut World, seconds: f64) {
        for _ in 0..(seconds / 0.005).round() as usize {
            world.step(0.005);
        }
    }

    #[test]
    fn straight_and_turning_like_a_diff_drive() {
        let mut w = World::new(wheels(), 0.0, Battery::at_rest(24.3, 10.0));
        w.command(500, 500);
        run(&mut w, 10.0);
        // 29 rpm on 0.09 m wheels: 0.2733 m/s, the 0.15 s lag costs 0.041 m
        let v = 29.0 / 60.0 * std::f64::consts::TAU * 0.09;
        assert!((w.v - v).abs() < 1e-9, "{}", w.v);
        assert!((w.x - (v * 10.0 - v * 0.15)).abs() < 2e-3, "{}", w.x);
        assert!(w.y.abs() < 1e-12 && w.yaw.abs() < 1e-12);
        // spin in place: 2 v / separation
        let mut w = World::new(wheels(), 0.0, Battery::at_rest(24.3, 10.0));
        w.command(-500, 500);
        run(&mut w, 3.0);
        assert!((w.w - 2.0 * v / 0.35).abs() < 1e-6, "{}", w.w);
        assert!(w.x.abs() < 1e-9 && w.y.abs() < 1e-9);
        // a circle comes back to the start
        let mut w = World::new(wheels(), 0.3, Battery::at_rest(24.3, 10.0));
        w.command(300, 600);
        run(&mut w, 3.0);
        let r = w.v / w.w;
        let period = std::f64::consts::TAU / w.w;
        let (x0, y0, yaw0) = (w.x, w.y, w.yaw);
        run(&mut w, (period / 0.005).round() * 0.005);
        assert!(((w.x - x0).powi(2) + (w.y - y0).powi(2)).sqrt() < 0.01 * r, "{} {}", w.x - x0, w.y - y0);
        assert!((w.yaw - yaw0).abs() < 0.02);
        assert!((w.accel[1] - w.v * w.w).abs() < 1e-9);
    }

    #[test]
    fn counts_follow_the_wheels_and_wrap() {
        let mut wheel = Wheel { target_rpm: 0.0, rpm: 0.0, rev: 2.5 };
        assert_eq!(wheel.counts(8896.0), 22240);
        wheel.rev = -0.25;
        assert_eq!(wheel.counts(8896.0), -2224);
        wheel.rev = (i32::MAX as f64 + 10.0) / 8896.0;
        assert_eq!(wheel.counts(8896.0), i32::MIN + 9);
    }

    #[test]
    fn slip_loses_ground_and_the_pack_drains() {
        let mut w = World::new(wheels(), 0.0, Battery::at_rest(24.3, 10.0));
        w.slip = 0.1;
        w.command(1000, 1000);
        let v0 = w.battery.voltage();
        run(&mut w, 5.0);
        let full = 58.0 / 60.0 * std::f64::consts::TAU * 0.09;
        assert!((w.v - 0.9 * full).abs() < 1e-6);
        assert!(w.battery.voltage() < v0, "driving draws the pack down");
        let f = w.battery.fraction;
        w.blade = 1.0;
        run(&mut w, 1.0);
        assert!((w.battery.current_a - (0.6 + 2.0 * 1.2 + 5.0)).abs() < 1e-3);
        assert!(w.battery.fraction < f);
    }

    #[test]
    fn a_resting_pack_reads_its_voltage_back_and_mower_battery_agrees() {
        let b = Battery { current_a: 0.0, ..Battery::at_rest(24.3, 10.0) };
        assert!((b.voltage() - 24.3).abs() < 1e-9);
        // 4.05 V per cell: 85 % on mower_battery's curve
        assert!((b.fraction - 0.85).abs() < 1e-9);
    }

    #[test]
    fn the_ocv_table_is_mower_batterys() {
        let src = include_str!("../../mower_battery/src/estimator.rs");
        let start = src.find("pub const DEFAULT_OCV_TABLE").unwrap();
        let body = &src[start..start + src[start..].find("];").unwrap()];
        let rows: Vec<(f64, f64)> = body
            .lines()
            .filter_map(|l| {
                let l = l.trim().strip_prefix('(')?.strip_suffix("),")?;
                let (v, f) = l.split_once(',')?;
                Some((v.trim().parse().ok()?, f.trim().parse().ok()?))
            })
            .collect();
        assert_eq!(rows, OCV_TABLE.to_vec());
    }
}
