//! The u-blox ZED-F9P (/dev/gps_rtk, UBX over USB), as fake_sensors.py plays
//! it, positioned by the truth world:
//!
//! - every CFG message (class 0x06: RATE, MSG, VALSET) is acknowledged, and
//!   CFG-RATE and the NAV messages' CFG-MSG rates are followed, so the
//!   stream is what mower_gps configured (by default 250 ms epochs with
//!   NAV-PVT, NAV-HPPOSLLH, NAV-RELPOSNED and NAV-EOE, NAV-SAT every 4th);
//! - the position is the antenna's (gps_link of the URDF, turned with the
//!   robot) plus noise, on WGS84 around the start point;
//! - `Mode`: RTK fixed (default; hAcc 14 mm, the app wants <= 15 mm to start
//!   a mission), RTK float, plain 3D, or no fix, switchable at run time;
//! - NAV-RELPOSNED is the vector from a base station placed so that it reads
//!   fake_sensors.py's (12.34, -5.67, 0.89) m at the start.

use mower_ubx::{class, encode, id, Parser};

use crate::geometry::{Mount, Origin};
use crate::rng::Rng;
use crate::world::World;

/// GPS time = unix + 18 s (leap seconds since 2017) from 1980-01-06.
const GPS_EPOCH_UNIX: f64 = 315_964_800.0;
const GPS_LEAP_S: f64 = 18.0;
const WEEK_MS: u64 = 604_800_000;
/// Height of the geoid above the ellipsoid at the start (fake_sensors.py's
/// 45.123 m ellipsoid vs 30.000 m MSL).
const GEOID_SEPARATION_M: f64 = 15.123;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Fixed,
    Float,
    ThreeD,
    NoFix,
}

struct Solution {
    fix_type: u8,
    flags: u8,
    num_sv: u8,
    h_acc_mm: u32,
    v_acc_mm: u32,
    sigma_m: f64,
    relpos_flags: u32,
    /// N, E, D accuracy of NAV-RELPOSNED, 0.1 mm
    relpos_acc: [u32; 3],
}

impl Mode {
    pub fn parse(text: &str) -> Option<Mode> {
        match text.trim() {
            "fixed" | "rtk" => Some(Mode::Fixed),
            "float" => Some(Mode::Float),
            "3d" => Some(Mode::ThreeD),
            "none" | "nofix" => Some(Mode::NoFix),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Mode::Fixed => "fixed",
            Mode::Float => "float",
            Mode::ThreeD => "3d",
            Mode::NoFix => "none",
        }
    }

    fn solution(self) -> Solution {
        // NAV-RELPOSNED flags: gnssFixOK, diffSoln, relPosValid, carrSoln
        // (bits 3-4), relPosNormalized, as fake_sensors.py sends them
        let rtk = |carr: u32| 0x001 | 0x002 | 0x004 | 0x200 | (carr << 3);
        match self {
            Mode::Fixed => Solution { fix_type: 3, flags: 0x83, num_sv: 17, h_acc_mm: 14, v_acc_mm: 23, sigma_m: 0.005, relpos_flags: rtk(2), relpos_acc: [140, 150, 300] },
            Mode::Float => Solution { fix_type: 3, flags: 0x43, num_sv: 16, h_acc_mm: 250, v_acc_mm: 420, sigma_m: 0.12, relpos_flags: rtk(1), relpos_acc: [2500, 2500, 4200] },
            Mode::ThreeD => Solution { fix_type: 3, flags: 0x01, num_sv: 12, h_acc_mm: 1600, v_acc_mm: 2600, sigma_m: 0.9, relpos_flags: 0x001, relpos_acc: [16000, 16000, 26000] },
            Mode::NoFix => Solution { fix_type: 0, flags: 0x00, num_sv: 3, h_acc_mm: 4_294_967, v_acc_mm: 3_750_082, sigma_m: 0.0, relpos_flags: 0, relpos_acc: [0, 0, 0] },
        }
    }
}

/// The NAV messages whose rate CFG-MSG sets, in the order they go out.
const NAV: [u8; 5] = [id::PVT, id::HPPOSLLH, id::RELPOSNED, id::SAT, id::EOE];

pub struct Gnss {
    parser: Parser,
    pub mode: Mode,
    /// metres; the mode's own when None
    pub sigma: Option<f64>,
    period_ms: u32,
    every: [u8; 5],
    epoch: u64,
    next_epoch: f64,
    /// the RTK base station, east / north / up of the start, metres
    base: [f64; 3],
    pub acks: u64,
    pub epochs: u64,
}

/// (year, month, day, hour, minute, second) of a unix time, UTC.
pub fn civil(unix: f64) -> (i64, u32, u32, u32, u32, u32) {
    let secs = unix.floor() as i64;
    let (days, sod) = (secs.div_euclid(86_400), secs.rem_euclid(86_400));
    // Howard Hinnant's civil_from_days
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let month = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let year = yoe + era * 400 + i64::from(month <= 2);
    (year, month, day, (sod / 3600) as u32, (sod % 3600 / 60) as u32, (sod % 60) as u32)
}

/// metres -> (whole unit, remainder in 1/`parts` of it), both truncated
/// towards zero as u-blox splits its high-precision fields.
fn split(value: f64, scale: f64, parts: i64) -> (i32, i8) {
    let n = (value * scale * parts as f64).round() as i64;
    ((n / parts) as i32, (n % parts) as i8)
}

impl Gnss {
    pub fn new(mode: Mode, sigma: Option<f64>) -> Gnss {
        Gnss {
            parser: Parser::default(),
            mode,
            sigma,
            period_ms: 250,
            every: [1, 1, 1, 4, 1],
            epoch: 0,
            next_epoch: 0.0,
            base: [5.67, -12.34, 0.89],
            acks: 0,
            epochs: 0,
        }
    }

    pub fn period_ms(&self) -> u32 {
        self.period_ms
    }

    /// Bytes the driver wrote: CFG messages, each acknowledged.
    pub fn on_bytes(&mut self, data: &[u8], out: &mut Vec<u8>) {
        for f in self.parser.push_all(data) {
            if f.class != class::CFG {
                continue;
            }
            out.extend_from_slice(&encode(class::ACK, id::ACK_ACK, &[f.class, f.id]));
            self.acks += 1;
            let p = &f.payload;
            match f.id {
                id::CFG_RATE if p.len() >= 6 => {
                    let meas = u16::from_le_bytes([p[0], p[1]]) as u32;
                    let nav = u16::from_le_bytes([p[2], p[3]]).max(1) as u32;
                    self.period_ms = (meas * nav).clamp(25, 10_000);
                }
                id::CFG_MSG if p.len() == 3 && p[0] == class::NAV => {
                    if let Some(i) = NAV.iter().position(|m| *m == p[1]) {
                        self.every[i] = p[2];
                    }
                }
                _ => {}
            }
        }
    }

    /// One epoch when it is due.
    #[allow(clippy::too_many_arguments)]
    pub fn tick(&mut self, now: f64, unix: f64, world: &World, mount: &Mount, origin: &Origin, rng: &mut Rng, out: &mut Vec<u8>) {
        if now < self.next_epoch {
            return;
        }
        let period = self.period_ms as f64 / 1000.0;
        self.next_epoch += period;
        if self.next_epoch < now {
            self.next_epoch = now + period;
        }
        let gps_ms = ((unix + GPS_LEAP_S - GPS_EPOCH_UNIX) * 1000.0) as u64;
        let itow = ((gps_ms % WEEK_MS) / self.period_ms as u64 * self.period_ms as u64) as u32;
        let sol = self.mode.solution();
        let fix = sol.fix_type != 0;
        let sigma = if fix { self.sigma.unwrap_or(sol.sigma_m) } else { 0.0 };
        let (s, c) = world.yaw.sin_cos();
        let (ax, ay) = (mount.antenna[0], mount.antenna[1]);
        let e = world.x + c * ax - s * ay + sigma * rng.normal();
        let n = world.y + s * ax + c * ay + sigma * rng.normal();
        let u = 1.5 * sigma * rng.normal();
        let (lat, lon, h) = origin.llh(e, n, u);
        let vel = (world.v * c, world.v * s);
        let due = |i: usize| self.every[i] != 0 && self.epoch % self.every[i] as u64 == 0;
        if due(0) {
            out.extend(encode(class::NAV, id::PVT, &pvt(itow, unix, &sol, fix.then_some((lat, lon, h)), vel)));
        }
        if due(1) {
            out.extend(encode(class::NAV, id::HPPOSLLH, &hpposllh(itow, &sol, fix.then_some((lat, lon, h)))));
        }
        if due(2) {
            let rel = [n - self.base[1], e - self.base[0], -(u - self.base[2])];
            out.extend(encode(class::NAV, id::RELPOSNED, &relposned(itow, &sol, rel)));
        }
        if due(3) {
            out.extend(encode(class::NAV, id::SAT, &navsat(itow)));
        }
        if due(4) {
            out.extend(encode(class::NAV, id::EOE, &itow.to_le_bytes()));
        }
        self.epoch += 1;
        self.epochs += 1;
    }
}

fn put_i32(p: &mut [u8], at: usize, v: i32) {
    p[at..at + 4].copy_from_slice(&v.to_le_bytes());
}
fn put_u32(p: &mut [u8], at: usize, v: u32) {
    p[at..at + 4].copy_from_slice(&v.to_le_bytes());
}

fn pvt(itow: u32, unix: f64, sol: &Solution, pos: Option<(f64, f64, f64)>, (ve, vn): (f64, f64)) -> Vec<u8> {
    let mut p = vec![0u8; 92];
    put_u32(&mut p, 0, itow);
    let (y, mo, d, h, mi, s) = civil(unix);
    p[4..6].copy_from_slice(&(y as u16).to_le_bytes());
    p[6..11].copy_from_slice(&[mo as u8, d as u8, h as u8, mi as u8, s as u8]);
    p[11] = 0x07; // validDate | validTime | fullyResolved
    p[20] = sol.fix_type;
    p[21] = sol.flags;
    p[23] = sol.num_sv;
    if let Some((lat, lon, hgt)) = pos {
        put_i32(&mut p, 24, (lon * 1e7).round() as i32);
        put_i32(&mut p, 28, (lat * 1e7).round() as i32);
        put_i32(&mut p, 32, (hgt * 1e3).round() as i32);
        put_i32(&mut p, 36, ((hgt - GEOID_SEPARATION_M) * 1e3).round() as i32);
        put_i32(&mut p, 48, (vn * 1e3).round() as i32);
        put_i32(&mut p, 52, (ve * 1e3).round() as i32);
        put_i32(&mut p, 60, (ve.hypot(vn) * 1e3).round() as i32);
        // heading of motion, clockwise from north
        put_i32(&mut p, 64, (ve.atan2(vn).to_degrees().rem_euclid(360.0) * 1e5).round() as i32);
    }
    put_u32(&mut p, 40, sol.h_acc_mm);
    put_u32(&mut p, 44, sol.v_acc_mm);
    p[76..78].copy_from_slice(&150u16.to_le_bytes()); // pDOP 1.50
    p
}

fn hpposllh(itow: u32, sol: &Solution, pos: Option<(f64, f64, f64)>) -> Vec<u8> {
    let mut p = vec![0u8; 36];
    p[3] = if pos.is_none() { 0x01 } else { 0x00 }; // invalidLlh
    put_u32(&mut p, 4, itow);
    if let Some((lat, lon, hgt)) = pos {
        let (lon7, lon_hp) = split(lon, 1e7, 100);
        let (lat7, lat_hp) = split(lat, 1e7, 100);
        let (h_mm, h_hp) = split(hgt, 1e3, 10);
        let (msl_mm, msl_hp) = split(hgt - GEOID_SEPARATION_M, 1e3, 10);
        put_i32(&mut p, 8, lon7);
        put_i32(&mut p, 12, lat7);
        put_i32(&mut p, 16, h_mm);
        put_i32(&mut p, 20, msl_mm);
        p[24..28].copy_from_slice(&[lon_hp as u8, lat_hp as u8, h_hp as u8, msl_hp as u8]);
    }
    put_u32(&mut p, 28, sol.h_acc_mm * 10);
    put_u32(&mut p, 32, sol.v_acc_mm * 10);
    p
}

fn relposned(itow: u32, sol: &Solution, ned: [f64; 3]) -> Vec<u8> {
    let mut p = vec![0u8; 64];
    p[0] = 1;
    put_u32(&mut p, 4, itow);
    let length = (ned[0] * ned[0] + ned[1] * ned[1] + ned[2] * ned[2]).sqrt();
    for (i, v) in ned.iter().enumerate() {
        let (cm, hp) = split(*v, 1e2, 100);
        put_i32(&mut p, 8 + 4 * i, cm);
        p[32 + i] = hp as u8;
    }
    let (len_cm, len_hp) = split(length, 1e2, 100);
    put_i32(&mut p, 20, len_cm);
    p[35] = len_hp as u8;
    put_i32(&mut p, 24, (ned[1].atan2(ned[0]).to_degrees().rem_euclid(360.0) * 1e5).round() as i32);
    for (i, a) in sol.relpos_acc.iter().enumerate() {
        put_u32(&mut p, 36 + 4 * i, *a);
    }
    put_u32(&mut p, 48, sol.relpos_acc[0].max(sol.relpos_acc[1]));
    put_u32(&mut p, 52, if sol.relpos_flags & 0x004 != 0 { 1_000_000 } else { 0 });
    put_u32(&mut p, 60, sol.relpos_flags);
    p
}

/// fake_sensors.py's sky: five satellites, used, with corrections.
fn navsat(itow: u32) -> Vec<u8> {
    let sats: [(u8, u8, u8, i8, i16); 5] = [(0, 3, 42, 55, 120), (0, 7, 38, 30, 250), (2, 11, 40, 70, 10), (3, 21, 36, 25, 300), (6, 9, 33, 15, 80)];
    let mut p = Vec::with_capacity(8 + 12 * sats.len());
    p.extend_from_slice(&itow.to_le_bytes());
    p.extend_from_slice(&[1, sats.len() as u8, 0, 0]);
    for (gnss, sv, cno, elev, azim) in sats {
        p.extend_from_slice(&[gnss, sv, cno, elev as u8]);
        p.extend_from_slice(&azim.to_le_bytes());
        p.extend_from_slice(&(-12i16).to_le_bytes());
        p.extend_from_slice(&(0x08u32 | 0x40 | 7).to_le_bytes());
    }
    p
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Wheels;
    use crate::world::Battery;
    use mower_ubx::{Ack, CarrierSolution, Fix, NavHpPosLlh, NavPvt, NavRelPosNed, NavSat};

    use crate::gps_epoch as epoch;

    const REPO_URDF: &str = include_str!("../../../../mower_description/mower_robot/robot.urdf.xacro");

    fn setup() -> (World, Mount, Origin, Rng) {
        let world = World::new(Wheels::from_params(None).unwrap(), 120f64.to_radians(), Battery::at_rest(24.3, 10.0));
        (world, Mount::from_urdf(REPO_URDF).unwrap(), Origin::new(25.033, 121.5654, 45.123), Rng::new(1))
    }

    fn frames(bytes: &[u8]) -> Vec<mower_ubx::Frame> {
        let mut p = Parser::default();
        let f = p.push_all(bytes);
        assert_eq!(p.checksum_failures, 0);
        f
    }

    #[test]
    fn the_drivers_configuration_is_acknowledged_and_followed() {
        let mut g = Gnss::new(Mode::Fixed, None);
        let mut out = Vec::new();
        // what mower_gps sends at start: UART1 off (VALSET), 4 Hz, the NAV set
        let mut cfg = encode(class::CFG, 0x8A, &[0, 1, 0, 0, 1, 0, 0x74, 0x10, 0]);
        cfg.extend(mower_ubx::cfg::rate(100, 1));
        cfg.extend(mower_ubx::cfg::msg_rate(class::NAV, id::SAT, 10));
        cfg.extend(mower_ubx::cfg::msg_rate(class::NAV, id::RELPOSNED, 0));
        g.on_bytes(&cfg, &mut out);
        let acks: Vec<Ack> = frames(&out).iter().filter_map(Ack::parse).collect();
        assert_eq!(acks.len(), 4);
        assert!(acks.iter().all(|a| a.ok && a.class == class::CFG));
        assert_eq!([acks[0].id, acks[1].id, acks[2].id], [0x8A, id::CFG_RATE, id::CFG_MSG]);
        assert_eq!(g.period_ms(), 100);
        let (world, mount, origin, mut rng) = setup();
        let mut out = Vec::new();
        for k in 0..20 {
            // just after each 100 ms deadline, whatever the float sums do
            let t = k as f64 * 0.1 + 1e-3;
            g.tick(t, 1.7e9 + t, &world, &mount, &origin, &mut rng, &mut out);
        }
        let f = frames(&out);
        let count = |i: u8| f.iter().filter(|x| x.id == i).count();
        assert_eq!((count(id::PVT), count(id::HPPOSLLH), count(id::RELPOSNED), count(id::SAT), count(id::EOE)), (20, 20, 0, 2, 20));
    }

    #[test]
    fn epochs_decode_into_mower_gps_fixes_at_the_antenna() {
        let (mut world, mount, origin, mut rng) = setup();
        world.x = 3.0;
        world.y = -4.0;
        world.v = 0.5;
        let mut g = Gnss::new(Mode::Fixed, Some(0.0));
        let mut out = Vec::new();
        for k in 0..8 {
            g.tick(k as f64 * 0.25, 1_793_000_000.0 + k as f64 * 0.25, &world, &mount, &origin, &mut rng, &mut out);
        }
        let mut asm = epoch::Assembler::new(true, true);
        let mut fixes = Vec::new();
        let mut sats = 0;
        for f in frames(&out) {
            match f.id {
                id::PVT => fixes.extend(asm.on_pvt(NavPvt::parse(&f.payload).unwrap(), std::time::SystemTime::now())),
                id::HPPOSLLH => fixes.extend(NavHpPosLlh::parse(&f.payload).and_then(|h| asm.on_hp(h))),
                id::EOE => fixes.extend(mower_ubx::nav_eoe_itow(&f.payload).and_then(|t| asm.on_eoe(t))),
                id::SAT => sats += NavSat::parse(&f.payload).unwrap().num_used(),
                id::RELPOSNED => {
                    let r = NavRelPosNed::parse(&f.payload).unwrap();
                    assert!(r.rel_pos_valid() && r.carrier_solution() == CarrierSolution::Fixed);
                    // 12.34 m north, -5.67 m east of the base at the start, plus the drive
                    assert!((r.n_m() - (12.34 - 4.0)).abs() < 1e-3 && (r.e_m() - (-5.67 + 3.0)).abs() < 1e-3, "{r:?}");
                    assert!((r.length_m().unwrap() - (8.34f64.powi(2) + 2.67f64.powi(2) + 0.89f64.powi(2)).sqrt()).abs() < 1e-3);
                }
                _ => {}
            }
        }
        assert_eq!(fixes.len(), 8, "one fix per epoch, closed by its NAV-HPPOSLLH");
        assert_eq!(asm.late, 0);
        assert_eq!(sats, 2 * 5);
        let fix = Fix::from_epoch(&fixes[0].pvt, fixes[0].hp.as_ref());
        assert_eq!(fix.status, 2, "RTK fixed reads GBAS_FIX");
        assert!(fix.high_precision);
        assert!((fix.h_var_m2.sqrt() - 0.014).abs() < 1e-9, "the app's 15 mm");
        let (lat, lon, _) = origin.llh(3.0, -4.0, 0.0);
        assert!((fix.latitude_deg - lat).abs() < 2e-9 && (fix.longitude_deg - lon).abs() < 2e-9, "{fix:?}");
        assert!((fix.altitude_m - 45.123).abs() < 1e-4);
        let pvt = fixes[0].pvt;
        assert_eq!(pvt.utc().as_deref(), Some("2026-10-26T07:33:20Z"));
        // 0.5 m/s heading 120 deg ENU: 330 deg from north
        assert!((pvt.ground_speed_mm_s - 500).abs() <= 1);
    }

    #[test]
    fn the_other_modes() {
        let (world, mount, origin, mut rng) = setup();
        for (mode, status, carrier) in [(Mode::Float, 0, CarrierSolution::Float), (Mode::ThreeD, 0, CarrierSolution::None), (Mode::NoFix, -1, CarrierSolution::None)] {
            let mut g = Gnss::new(mode, None);
            let mut out = Vec::new();
            g.tick(0.0, 1.8e9, &world, &mount, &origin, &mut rng, &mut out);
            let f = frames(&out);
            let pvt = NavPvt::parse(&f.iter().find(|x| x.id == id::PVT).unwrap().payload).unwrap();
            let hp = NavHpPosLlh::parse(&f.iter().find(|x| x.id == id::HPPOSLLH).unwrap().payload).unwrap();
            assert_eq!((pvt.navsat_status(), pvt.carrier_solution()), (status, carrier), "{mode:?}");
            let fix = Fix::from_epoch(&pvt, Some(&hp));
            if mode == Mode::NoFix {
                assert!(fix.latitude_deg.is_nan() && hp.invalid);
            } else {
                assert!((fix.latitude_deg - 25.033).abs() < 1e-4, "{mode:?} {fix:?}");
                assert!(fix.h_var_m2.sqrt() > 0.015, "{mode:?}: not good enough to start a mission");
            }
            assert_eq!(Mode::parse(mode.name()), Some(mode));
        }
        assert_eq!(Mode::parse("bogus"), None);
    }

    #[test]
    fn calendar_and_splits() {
        assert_eq!(civil(1_700_000_000.0), (2023, 11, 14, 22, 13, 20));
        assert_eq!(civil(951_782_400.0), (2000, 2, 29, 0, 0, 0));
        assert_eq!(civil(0.5), (1970, 1, 1, 0, 0, 0));
        assert_eq!(split(121.565_400_123, 1e7, 100), (1_215_654_001, 23));
        assert_eq!(split(-0.000_000_123_45, 1e7, 100), (-1, -23));
        assert_eq!(split(45.12346, 1e3, 10), (45_123, 5));
    }
}
