//! u-blox UBX protocol, the part `mower_gps` needs: a resynchronising frame
//! parser, the 8-bit Fletcher checksum, encoders for the legacy CFG
//! messages that set the output rate on the current port, and decoders for
//! NAV-PVT, NAV-HPPOSLLH, NAV-SAT, NAV-RELPOSNED, NAV-EOE and ACK-ACK/NAK.
//! No ROS or I/O in here so it is unit-tested on any host with plain cargo.
//!
//! References: u-blox ZED-F9P interface description (UBX-18010854), which
//! still accepts UBX-CFG-MSG / UBX-CFG-RATE next to the newer CFG-VALSET.

pub const SYNC1: u8 = 0xB5;
pub const SYNC2: u8 = 0x62;

/// Payloads longer than this are not something we asked for (RXM-RAWX is
/// ~1.4 kB); anything bigger is treated as a false sync.
pub const MAX_PAYLOAD: usize = 8192;

pub mod class {
    pub const NAV: u8 = 0x01;
    pub const ACK: u8 = 0x05;
    pub const CFG: u8 = 0x06;
}

pub mod id {
    // NAV
    pub const PVT: u8 = 0x07;
    pub const HPPOSLLH: u8 = 0x14;
    pub const SAT: u8 = 0x35;
    pub const RELPOSNED: u8 = 0x3C;
    pub const EOE: u8 = 0x61;
    // ACK
    pub const ACK_NAK: u8 = 0x00;
    pub const ACK_ACK: u8 = 0x01;
    // CFG
    pub const CFG_MSG: u8 = 0x01;
    pub const CFG_RATE: u8 = 0x08;
}

/// One complete, checksum-verified UBX frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    pub class: u8,
    pub id: u8,
    pub payload: Vec<u8>,
}

/// 8-bit Fletcher checksum over class, id, length and payload.
pub fn checksum(bytes: &[u8]) -> (u8, u8) {
    let mut a: u8 = 0;
    let mut b: u8 = 0;
    for &x in bytes {
        a = a.wrapping_add(x);
        b = b.wrapping_add(a);
    }
    (a, b)
}

/// Wire encoding of a frame (sync, header, payload, checksum).
pub fn encode(class: u8, id: u8, payload: &[u8]) -> Vec<u8> {
    let len = payload.len() as u16;
    let mut out = Vec::with_capacity(payload.len() + 8);
    out.extend_from_slice(&[SYNC1, SYNC2, class, id, len as u8, (len >> 8) as u8]);
    out.extend_from_slice(payload);
    let (a, b) = checksum(&out[2..]);
    out.push(a);
    out.push(b);
    out
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Sync1,
    Sync2,
    Class,
    Id,
    Len1,
    Len2,
    Payload,
    CkA,
    CkB,
}

/// Byte-at-a-time parser. Anything that is not a well-formed UBX frame
/// (NMEA sentences, RTCM, a truncated frame after a USB hiccup) is skipped
/// by resynchronising on the next 0xB5 0x62.
#[derive(Debug)]
pub struct Parser {
    state: State,
    class: u8,
    id: u8,
    len: usize,
    payload: Vec<u8>,
    ck_a: u8,
    /// Frames dropped because their checksum did not match.
    pub checksum_failures: u64,
    /// Headers announcing more than MAX_PAYLOAD bytes (false syncs).
    pub oversize_headers: u64,
}

impl Default for Parser {
    fn default() -> Self {
        Self {
            state: State::Sync1,
            class: 0,
            id: 0,
            len: 0,
            payload: Vec::new(),
            ck_a: 0,
            checksum_failures: 0,
            oversize_headers: 0,
        }
    }
}

impl Parser {
    pub fn reset(&mut self) {
        self.state = State::Sync1;
        self.payload.clear();
    }

    /// Feed one byte; returns a frame when it completes one.
    pub fn push(&mut self, byte: u8) -> Option<Frame> {
        match self.state {
            State::Sync1 => {
                if byte == SYNC1 {
                    self.state = State::Sync2;
                }
            }
            State::Sync2 => {
                self.state = if byte == SYNC2 { State::Class } else if byte == SYNC1 { State::Sync2 } else { State::Sync1 };
            }
            State::Class => {
                self.class = byte;
                self.state = State::Id;
            }
            State::Id => {
                self.id = byte;
                self.state = State::Len1;
            }
            State::Len1 => {
                self.len = byte as usize;
                self.state = State::Len2;
            }
            State::Len2 => {
                self.len |= (byte as usize) << 8;
                if self.len > MAX_PAYLOAD {
                    self.oversize_headers += 1;
                    self.reset();
                    return None;
                }
                self.payload.clear();
                self.state = if self.len == 0 { State::CkA } else { State::Payload };
            }
            State::Payload => {
                self.payload.push(byte);
                if self.payload.len() == self.len {
                    self.state = State::CkA;
                }
            }
            State::CkA => {
                self.ck_a = byte;
                self.state = State::CkB;
            }
            State::CkB => {
                self.state = State::Sync1;
                let mut header = [self.class, self.id, self.len as u8, (self.len >> 8) as u8].to_vec();
                header.extend_from_slice(&self.payload);
                if checksum(&header) == (self.ck_a, byte) {
                    return Some(Frame { class: self.class, id: self.id, payload: std::mem::take(&mut self.payload) });
                }
                self.checksum_failures += 1;
            }
        }
        None
    }

    /// Convenience for tests and bulk reads.
    pub fn push_all(&mut self, bytes: &[u8]) -> Vec<Frame> {
        bytes.iter().filter_map(|&b| self.push(b)).collect()
    }
}

fn u16_at(p: &[u8], i: usize) -> u16 {
    u16::from_le_bytes([p[i], p[i + 1]])
}
fn i16_at(p: &[u8], i: usize) -> i16 {
    u16_at(p, i) as i16
}
fn u32_at(p: &[u8], i: usize) -> u32 {
    u32::from_le_bytes([p[i], p[i + 1], p[i + 2], p[i + 3]])
}
fn i32_at(p: &[u8], i: usize) -> i32 {
    u32_at(p, i) as i32
}

/// Legacy configuration messages. The 3-byte CFG-MSG form sets the output
/// rate of one message on the port the command arrives on (USB for us),
/// nothing is saved to flash: the receiver keeps its stored configuration
/// across power cycles and this driver re-applies its own every start.
pub mod cfg {
    use super::{class, encode, id};

    /// UBX-CFG-RATE: measurement period in ms, navigation solutions every
    /// `nav_rate` measurements, time reference GPS.
    pub fn rate(meas_ms: u16, nav_rate: u16) -> Vec<u8> {
        let mut p = Vec::with_capacity(6);
        p.extend_from_slice(&meas_ms.to_le_bytes());
        p.extend_from_slice(&nav_rate.to_le_bytes());
        p.extend_from_slice(&1u16.to_le_bytes());
        encode(class::CFG, id::CFG_RATE, &p)
    }

    /// UBX-CFG-MSG (3-byte form): message `msg_class`/`msg_id` every
    /// `every_n` navigation solutions on the current port (0 = off).
    pub fn msg_rate(msg_class: u8, msg_id: u8, every_n: u8) -> Vec<u8> {
        encode(class::CFG, id::CFG_MSG, &[msg_class, msg_id, every_n])
    }
}

/// UBX-ACK-ACK / UBX-ACK-NAK for a configuration message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ack {
    pub class: u8,
    pub id: u8,
    pub ok: bool,
}

impl Ack {
    pub fn parse(frame: &Frame) -> Option<Ack> {
        if frame.class != class::ACK || frame.payload.len() < 2 {
            return None;
        }
        let ok = match frame.id {
            id::ACK_ACK => true,
            id::ACK_NAK => false,
            _ => return None,
        };
        Some(Ack { class: frame.payload[0], id: frame.payload[1], ok })
    }
}

/// Carrier phase range solution status (NAV-PVT flags bits 6..7).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CarrierSolution {
    None,
    Float,
    Fixed,
}

impl CarrierSolution {
    pub fn as_str(self) -> &'static str {
        match self {
            CarrierSolution::None => "none",
            CarrierSolution::Float => "float",
            CarrierSolution::Fixed => "fixed",
        }
    }
}

/// UBX-NAV-PVT, the fields the fix and the status topic use.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavPvt {
    pub itow_ms: u32,
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub min: u8,
    pub sec: u8,
    pub valid: u8,
    pub fix_type: u8,
    pub flags: u8,
    pub flags2: u8,
    pub num_sv: u8,
    /// 1e-7 degrees
    pub lon_1e7: i32,
    pub lat_1e7: i32,
    /// Height above ellipsoid, mm
    pub height_mm: i32,
    pub h_msl_mm: i32,
    /// Horizontal / vertical accuracy estimate, mm
    pub h_acc_mm: u32,
    pub v_acc_mm: u32,
    pub ground_speed_mm_s: i32,
    pub pdop_0_01: u16,
}

impl NavPvt {
    pub const LEN: usize = 92;
    pub const FIX_2D: u8 = 2;
    pub const FLAG_GNSS_FIX_OK: u8 = 0x01;
    pub const FLAG_DIFF_SOLN: u8 = 0x02;
    pub const FLAG_CARRIER_FLOAT: u8 = 0x40;
    pub const FLAG_CARRIER_FIXED: u8 = 0x80;

    pub fn parse(payload: &[u8]) -> Option<NavPvt> {
        if payload.len() < Self::LEN {
            return None;
        }
        let p = payload;
        Some(NavPvt {
            itow_ms: u32_at(p, 0),
            year: u16_at(p, 4),
            month: p[6],
            day: p[7],
            hour: p[8],
            min: p[9],
            sec: p[10],
            valid: p[11],
            fix_type: p[20],
            flags: p[21],
            flags2: p[22],
            num_sv: p[23],
            lon_1e7: i32_at(p, 24),
            lat_1e7: i32_at(p, 28),
            height_mm: i32_at(p, 32),
            h_msl_mm: i32_at(p, 36),
            h_acc_mm: u32_at(p, 40),
            v_acc_mm: u32_at(p, 44),
            ground_speed_mm_s: i32_at(p, 60),
            pdop_0_01: u16_at(p, 76),
        })
    }

    pub fn fix_ok(&self) -> bool {
        self.flags & Self::FLAG_GNSS_FIX_OK != 0
    }

    pub fn diff_soln(&self) -> bool {
        self.flags & Self::FLAG_DIFF_SOLN != 0
    }

    pub fn carrier_solution(&self) -> CarrierSolution {
        match self.flags & 0xC0 {
            Self::FLAG_CARRIER_FIXED => CarrierSolution::Fixed,
            Self::FLAG_CARRIER_FLOAT => CarrierSolution::Float,
            _ => CarrierSolution::None,
        }
    }

    /// True for the fix types that carry a position: 2D, 3D and GNSS+dead
    /// reckoning (not "dead reckoning only" or "time only").
    pub fn has_position(&self) -> bool {
        matches!(self.fix_type, 2 | 3 | 4)
    }

    /// sensor_msgs/NavSatStatus.status with the rule of the ROS `ublox_gps`
    /// driver: -1 (NO_FIX) unless gnssFixOK and a position fix, 2
    /// (GBAS_FIX) when the carrier solution is fixed, else 0 (FIX). An RTK
    /// float or DGNSS solution is therefore a plain FIX and the covariance
    /// (from hAcc) carries the precision. Stricter than ublox_gps in one
    /// respect: a time-only fix (type 5) is never reported as a position.
    pub fn navsat_status(&self) -> i8 {
        if !(self.fix_ok() && self.has_position()) {
            return -1;
        }
        if self.carrier_solution() == CarrierSolution::Fixed {
            2
        } else {
            0
        }
    }

    pub fn latitude_deg(&self) -> f64 {
        self.lat_1e7 as f64 * 1e-7
    }

    pub fn longitude_deg(&self) -> f64 {
        self.lon_1e7 as f64 * 1e-7
    }

    /// Height above the WGS84 ellipsoid in metres (what NavSatFix carries).
    pub fn altitude_m(&self) -> f64 {
        self.height_mm as f64 * 1e-3
    }

    pub fn h_acc_m(&self) -> f64 {
        self.h_acc_mm as f64 * 1e-3
    }

    pub fn v_acc_m(&self) -> f64 {
        self.v_acc_mm as f64 * 1e-3
    }

    pub fn pdop(&self) -> f64 {
        self.pdop_0_01 as f64 * 0.01
    }

    /// "YYYY-MM-DDTHH:MM:SSZ" when the receiver reports a valid UTC
    /// date and time (valid bits 0 and 1), else None.
    pub fn utc(&self) -> Option<String> {
        if self.valid & 0x03 != 0x03 {
            return None;
        }
        Some(format!(
            "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}Z",
            self.year, self.month, self.day, self.hour, self.min, self.sec
        ))
    }
}

/// UBX-NAV-HPPOSLLH: the same position with the extra 1e-9 deg / 0.1 mm
/// digits an RTK fix actually has (NAV-PVT's 1e-7 deg is ~1 cm).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavHpPosLlh {
    pub itow_ms: u32,
    pub invalid: bool,
    pub lon_1e7: i32,
    pub lat_1e7: i32,
    pub height_mm: i32,
    pub h_msl_mm: i32,
    /// 1e-9 degrees, -99..=99
    pub lon_hp: i8,
    pub lat_hp: i8,
    /// 0.1 mm, -9..=9
    pub height_hp: i8,
    pub h_msl_hp: i8,
    /// 0.1 mm
    pub h_acc_0_1mm: u32,
    pub v_acc_0_1mm: u32,
}

impl NavHpPosLlh {
    pub const LEN: usize = 36;

    pub fn parse(payload: &[u8]) -> Option<NavHpPosLlh> {
        if payload.len() < Self::LEN {
            return None;
        }
        let p = payload;
        Some(NavHpPosLlh {
            itow_ms: u32_at(p, 4),
            invalid: p[3] & 0x01 != 0,
            lon_1e7: i32_at(p, 8),
            lat_1e7: i32_at(p, 12),
            height_mm: i32_at(p, 16),
            h_msl_mm: i32_at(p, 20),
            lon_hp: p[24] as i8,
            lat_hp: p[25] as i8,
            height_hp: p[26] as i8,
            h_msl_hp: p[27] as i8,
            h_acc_0_1mm: u32_at(p, 28),
            v_acc_0_1mm: u32_at(p, 32),
        })
    }

    pub fn latitude_deg(&self) -> f64 {
        self.lat_1e7 as f64 * 1e-7 + self.lat_hp as f64 * 1e-9
    }

    pub fn longitude_deg(&self) -> f64 {
        self.lon_1e7 as f64 * 1e-7 + self.lon_hp as f64 * 1e-9
    }

    pub fn altitude_m(&self) -> f64 {
        self.height_mm as f64 * 1e-3 + self.height_hp as f64 * 1e-4
    }

    pub fn h_acc_m(&self) -> f64 {
        self.h_acc_0_1mm as f64 * 1e-4
    }

    pub fn v_acc_m(&self) -> f64 {
        self.v_acc_0_1mm as f64 * 1e-4
    }
}

/// One satellite of UBX-NAV-SAT.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SatInfo {
    /// 0 GPS, 1 SBAS, 2 Galileo, 3 BeiDou, 5 QZSS, 6 GLONASS, 7 NavIC
    pub gnss_id: u8,
    pub sv_id: u8,
    /// Carrier-to-noise ratio, dBHz (0 = not tracked)
    pub cno: u8,
    /// Degrees; None when the receiver does not know where the satellite is
    pub elev_deg: Option<i8>,
    pub azim_deg: Option<i16>,
    /// Pseudorange residual, 0.1 m
    pub pr_res_dm: i16,
    pub flags: u32,
}

impl SatInfo {
    pub const FLAG_SV_USED: u32 = 0x08;
    pub const FLAG_DIFF_CORR: u32 = 0x40;

    /// Signal quality, 0 no signal .. 4 code locked .. 5-7 code and carrier locked.
    pub fn quality(&self) -> u8 {
        (self.flags & 0x07) as u8
    }

    /// Used in the navigation solution.
    pub fn sv_used(&self) -> bool {
        self.flags & Self::FLAG_SV_USED != 0
    }

    /// 0 unknown, 1 healthy, 2 unhealthy.
    pub fn health(&self) -> u8 {
        ((self.flags >> 4) & 0x03) as u8
    }

    /// Differential correction data is available for this satellite.
    pub fn diff_corr(&self) -> bool {
        self.flags & Self::FLAG_DIFF_CORR != 0
    }
}

/// UBX-NAV-SAT: every satellite the receiver tracks or has orbit data for,
/// with signal strength and position in the sky.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NavSat {
    pub itow_ms: u32,
    pub version: u8,
    pub sats: Vec<SatInfo>,
}

impl NavSat {
    pub const HEADER_LEN: usize = 8;
    pub const SAT_LEN: usize = 12;

    pub fn parse(payload: &[u8]) -> Option<NavSat> {
        if payload.len() < Self::HEADER_LEN {
            return None;
        }
        let p = payload;
        let num_svs = p[5] as usize;
        if p.len() != Self::HEADER_LEN + Self::SAT_LEN * num_svs {
            return None;
        }
        let sats = p[Self::HEADER_LEN..]
            .chunks_exact(Self::SAT_LEN)
            .map(|s| {
                let elev = s[3] as i8;
                let azim = i16_at(s, 4);
                // elevation out of range means the position is unknown, and
                // the azimuth with it
                let known = (-90..=90).contains(&elev);
                SatInfo {
                    gnss_id: s[0],
                    sv_id: s[1],
                    cno: s[2],
                    elev_deg: known.then_some(elev),
                    azim_deg: (known && (0..=360).contains(&azim)).then_some(azim),
                    pr_res_dm: i16_at(s, 6),
                    flags: u32_at(s, 8),
                }
            })
            .collect();
        Some(NavSat { itow_ms: u32_at(p, 0), version: p[4], sats })
    }

    pub fn num_used(&self) -> usize {
        self.sats.iter().filter(|s| s.sv_used()).count()
    }
}

/// UBX-NAV-RELPOSNED: where the antenna is relative to the RTK reference
/// station (base -> rover, north / east / down). Version 1 is the ZED-F9P
/// layout; version 0 (NEO-M8P) has no length / heading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NavRelPosNed {
    pub version: u8,
    pub ref_station_id: u16,
    pub itow_ms: u32,
    /// cm, plus the 0.1 mm part (-99..=99)
    pub rel_pos_n_cm: i32,
    pub rel_pos_e_cm: i32,
    pub rel_pos_d_cm: i32,
    pub rel_pos_hp_n: i8,
    pub rel_pos_hp_e: i8,
    pub rel_pos_hp_d: i8,
    /// Version 1 only: cm + 0.1 mm, and 1e-5 degrees
    pub rel_pos_length_cm: Option<i32>,
    pub rel_pos_hp_length: Option<i8>,
    pub rel_pos_heading_1e5: Option<i32>,
    /// 0.1 mm
    pub acc_n_0_1mm: u32,
    pub acc_e_0_1mm: u32,
    pub acc_d_0_1mm: u32,
    /// Version 1 only: 0.1 mm, and 1e-5 degrees
    pub acc_length_0_1mm: Option<u32>,
    pub acc_heading_1e5: Option<u32>,
    pub flags: u32,
}

impl NavRelPosNed {
    pub const LEN_V0: usize = 40;
    pub const LEN_V1: usize = 64;
    pub const FLAG_GNSS_FIX_OK: u32 = 0x001;
    pub const FLAG_DIFF_SOLN: u32 = 0x002;
    pub const FLAG_REL_POS_VALID: u32 = 0x004;
    pub const FLAG_IS_MOVING: u32 = 0x020;
    pub const FLAG_REF_POS_MISS: u32 = 0x040;
    pub const FLAG_REF_OBS_MISS: u32 = 0x080;
    pub const FLAG_HEADING_VALID: u32 = 0x100;
    pub const FLAG_NORMALIZED: u32 = 0x200;

    pub fn parse(payload: &[u8]) -> Option<NavRelPosNed> {
        let p = payload;
        match p.first()? {
            0 if p.len() >= Self::LEN_V0 => Some(NavRelPosNed {
                version: 0,
                ref_station_id: u16_at(p, 2),
                itow_ms: u32_at(p, 4),
                rel_pos_n_cm: i32_at(p, 8),
                rel_pos_e_cm: i32_at(p, 12),
                rel_pos_d_cm: i32_at(p, 16),
                rel_pos_hp_n: p[20] as i8,
                rel_pos_hp_e: p[21] as i8,
                rel_pos_hp_d: p[22] as i8,
                rel_pos_length_cm: None,
                rel_pos_hp_length: None,
                rel_pos_heading_1e5: None,
                acc_n_0_1mm: u32_at(p, 24),
                acc_e_0_1mm: u32_at(p, 28),
                acc_d_0_1mm: u32_at(p, 32),
                acc_length_0_1mm: None,
                acc_heading_1e5: None,
                flags: u32_at(p, 36),
            }),
            1 if p.len() >= Self::LEN_V1 => Some(NavRelPosNed {
                version: 1,
                ref_station_id: u16_at(p, 2),
                itow_ms: u32_at(p, 4),
                rel_pos_n_cm: i32_at(p, 8),
                rel_pos_e_cm: i32_at(p, 12),
                rel_pos_d_cm: i32_at(p, 16),
                rel_pos_hp_n: p[32] as i8,
                rel_pos_hp_e: p[33] as i8,
                rel_pos_hp_d: p[34] as i8,
                rel_pos_length_cm: Some(i32_at(p, 20)),
                rel_pos_hp_length: Some(p[35] as i8),
                rel_pos_heading_1e5: Some(i32_at(p, 24)),
                acc_n_0_1mm: u32_at(p, 36),
                acc_e_0_1mm: u32_at(p, 40),
                acc_d_0_1mm: u32_at(p, 44),
                acc_length_0_1mm: Some(u32_at(p, 48)),
                acc_heading_1e5: Some(u32_at(p, 52)),
                flags: u32_at(p, 60),
            }),
            _ => None,
        }
    }

    /// Metres from cm + 0.1 mm, computed in 0.1 mm integers so the value
    /// prints with at most four decimals.
    fn metres(cm: i32, hp: i8) -> f64 {
        (cm as i64 * 100 + hp as i64) as f64 / 1e4
    }

    pub fn n_m(&self) -> f64 {
        Self::metres(self.rel_pos_n_cm, self.rel_pos_hp_n)
    }

    pub fn e_m(&self) -> f64 {
        Self::metres(self.rel_pos_e_cm, self.rel_pos_hp_e)
    }

    pub fn d_m(&self) -> f64 {
        Self::metres(self.rel_pos_d_cm, self.rel_pos_hp_d)
    }

    pub fn length_m(&self) -> Option<f64> {
        Some(Self::metres(self.rel_pos_length_cm?, self.rel_pos_hp_length?))
    }

    /// Heading of the base -> rover vector, only when the receiver flags it valid.
    pub fn heading_deg(&self) -> Option<f64> {
        let h = self.rel_pos_heading_1e5.filter(|_| self.heading_valid())?;
        Some(h as f64 / 1e5)
    }

    pub fn acc_n_m(&self) -> f64 {
        self.acc_n_0_1mm as f64 / 1e4
    }

    pub fn acc_e_m(&self) -> f64 {
        self.acc_e_0_1mm as f64 / 1e4
    }

    pub fn acc_d_m(&self) -> f64 {
        self.acc_d_0_1mm as f64 / 1e4
    }

    pub fn acc_length_m(&self) -> Option<f64> {
        Some(self.acc_length_0_1mm? as f64 / 1e4)
    }

    pub fn acc_heading_deg(&self) -> Option<f64> {
        let a = self.acc_heading_1e5.filter(|_| self.heading_valid())?;
        Some(a as f64 / 1e5)
    }

    pub fn gnss_fix_ok(&self) -> bool {
        self.flags & Self::FLAG_GNSS_FIX_OK != 0
    }

    pub fn diff_soln(&self) -> bool {
        self.flags & Self::FLAG_DIFF_SOLN != 0
    }

    /// False without corrections from a reference station.
    pub fn rel_pos_valid(&self) -> bool {
        self.flags & Self::FLAG_REL_POS_VALID != 0
    }

    pub fn carrier_solution(&self) -> CarrierSolution {
        match (self.flags >> 3) & 0x03 {
            2 => CarrierSolution::Fixed,
            1 => CarrierSolution::Float,
            _ => CarrierSolution::None,
        }
    }

    pub fn is_moving(&self) -> bool {
        self.flags & Self::FLAG_IS_MOVING != 0
    }

    /// Extrapolated reference position used (the base position message is missing).
    pub fn ref_pos_miss(&self) -> bool {
        self.flags & Self::FLAG_REF_POS_MISS != 0
    }

    /// Extrapolated reference observations used (base observations are missing).
    pub fn ref_obs_miss(&self) -> bool {
        self.flags & Self::FLAG_REF_OBS_MISS != 0
    }

    pub fn heading_valid(&self) -> bool {
        self.version >= 1 && self.flags & Self::FLAG_HEADING_VALID != 0
    }
}

/// UBX-NAV-EOE: end of the navigation epoch `itow`; sent after every other
/// enabled NAV message of that epoch.
pub fn nav_eoe_itow(payload: &[u8]) -> Option<u32> {
    (payload.len() >= 4).then(|| u32_at(payload, 0))
}

/// A NavSatFix worth of data: PVT status and covariance, HPPOSLLH position
/// when a valid one for the same epoch is available.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fix {
    pub status: i8,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_m: f64,
    pub h_var_m2: f64,
    pub v_var_m2: f64,
    pub high_precision: bool,
}

impl Fix {
    pub fn from_epoch(pvt: &NavPvt, hp: Option<&NavHpPosLlh>) -> Fix {
        let status = pvt.navsat_status();
        let hp = hp.filter(|h| h.itow_ms == pvt.itow_ms && !h.invalid);
        let (lat, lon, alt, h_acc, v_acc) = match hp {
            Some(h) => (h.latitude_deg(), h.longitude_deg(), h.altitude_m(), h.h_acc_m(), h.v_acc_m()),
            None => (pvt.latitude_deg(), pvt.longitude_deg(), pvt.altitude_m(), pvt.h_acc_m(), pvt.v_acc_m()),
        };
        // Without a fix the receiver reports zeros (or its last position);
        // NaN keeps navsat_transform and the app from treating it as a place.
        let (lat, lon, alt) = if status < 0 { (f64::NAN, f64::NAN, f64::NAN) } else { (lat, lon, alt) };
        Fix {
            status,
            latitude_deg: lat,
            longitude_deg: lon,
            altitude_m: alt,
            h_var_m2: h_acc * h_acc,
            v_var_m2: v_acc * v_acc,
            high_precision: hp.is_some(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pvt_payload(itow: u32, fix_type: u8, flags: u8, lat_1e7: i32, lon_1e7: i32, h_acc_mm: u32) -> Vec<u8> {
        let mut p = vec![0u8; NavPvt::LEN];
        p[0..4].copy_from_slice(&itow.to_le_bytes());
        p[4..6].copy_from_slice(&2026u16.to_le_bytes());
        p[6] = 9;
        p[7] = 19;
        p[8] = 15;
        p[9] = 30;
        p[10] = 5;
        p[11] = 0x07; // validDate | validTime | fullyResolved
        p[20] = fix_type;
        p[21] = flags;
        p[23] = 17;
        p[24..28].copy_from_slice(&lon_1e7.to_le_bytes());
        p[28..32].copy_from_slice(&lat_1e7.to_le_bytes());
        p[32..36].copy_from_slice(&45_123i32.to_le_bytes()); // 45.123 m ellipsoid
        p[36..40].copy_from_slice(&30_000i32.to_le_bytes());
        p[40..44].copy_from_slice(&h_acc_mm.to_le_bytes());
        p[44..48].copy_from_slice(&(h_acc_mm * 2).to_le_bytes());
        p[60..64].copy_from_slice(&123i32.to_le_bytes());
        p[76..78].copy_from_slice(&150u16.to_le_bytes()); // pDOP 1.50
        p
    }

    #[test]
    fn checksum_matches_the_interface_description_example() {
        // UBX-CFG-MSG poll for NAV-PVT: B5 62 06 01 02 00 01 07 11 3A
        let frame = encode(0x06, 0x01, &[0x01, 0x07]);
        assert_eq!(frame, vec![0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0x01, 0x07, 0x11, 0x3A]);
    }

    #[test]
    fn parser_round_trips_and_resynchronises_past_garbage() {
        let a = encode(class::NAV, id::EOE, &1000u32.to_le_bytes());
        let b = encode(class::ACK, id::ACK_ACK, &[0x06, 0x08]);
        let mut stream = b"$GNGGA,000,,,*4F\r\n".to_vec();
        stream.extend_from_slice(&a);
        stream.extend_from_slice(&[0xB5, 0x62, 0x01]); // truncated frame
        stream.extend_from_slice(&b);
        let mut parser = Parser::default();
        let frames = parser.push_all(&stream);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0], Frame { class: class::NAV, id: id::EOE, payload: 1000u32.to_le_bytes().to_vec() });
        // The truncated header took b's bytes as id/length (0x0562 = 1378
        // bytes of bogus payload); UBX has no escaping, so recovery costs
        // that many bytes, after which the bogus frame fails its checksum
        // and the next real frame parses.
        assert!(parser.push_all(&vec![0u8; 1378 + 2]).is_empty());
        assert_eq!(parser.checksum_failures, 1);
        let frames = parser.push_all(&b);
        assert_eq!(frames.len(), 1);
        assert_eq!(Ack::parse(&frames[0]), Some(Ack { class: 0x06, id: 0x08, ok: true }));
    }

    #[test]
    fn corrupted_frame_is_counted_not_delivered() {
        let mut bytes = encode(class::NAV, id::EOE, &[1, 2, 3, 4]);
        bytes[7] ^= 0xFF;
        let mut parser = Parser::default();
        assert!(parser.push_all(&bytes).is_empty());
        assert_eq!(parser.checksum_failures, 1);
        // and a false sync announcing a huge payload does not stall it
        let mut parser = Parser::default();
        assert!(parser.push_all(&[0xB5, 0x62, 0x01, 0x07, 0xFF, 0xFF]).is_empty());
        assert_eq!(parser.oversize_headers, 1);
        assert_eq!(parser.push_all(&encode(class::NAV, id::EOE, &[0; 4])).len(), 1);
    }

    #[test]
    fn cfg_messages_have_the_documented_layout() {
        // 250 ms, every measurement, GPS time
        assert_eq!(cfg::rate(250, 1), encode(0x06, 0x08, &[0xFA, 0x00, 0x01, 0x00, 0x01, 0x00]));
        assert_eq!(cfg::msg_rate(0x01, 0x14, 1), encode(0x06, 0x01, &[0x01, 0x14, 0x01]));
    }

    #[test]
    fn nav_pvt_decodes_and_maps_status_like_ublox_gps() {
        let p = pvt_payload(123_456, 3, NavPvt::FLAG_GNSS_FIX_OK, 250_123_456, 1_215_432_100, 12);
        let pvt = NavPvt::parse(&p).unwrap();
        assert_eq!(pvt.itow_ms, 123_456);
        assert_eq!(pvt.num_sv, 17);
        assert!((pvt.latitude_deg() - 25.0123456).abs() < 1e-9);
        assert!((pvt.longitude_deg() - 121.54321).abs() < 1e-9);
        assert!((pvt.altitude_m() - 45.123).abs() < 1e-9);
        assert_eq!(pvt.h_acc_m(), 0.012);
        assert_eq!(pvt.pdop(), 1.5);
        assert_eq!(pvt.utc().as_deref(), Some("2026-09-19T15:30:05Z"));
        assert_eq!(pvt.navsat_status(), 0);
        assert_eq!(pvt.carrier_solution(), CarrierSolution::None);

        let fixed = NavPvt::parse(&pvt_payload(1, 3, NavPvt::FLAG_GNSS_FIX_OK | NavPvt::FLAG_DIFF_SOLN | NavPvt::FLAG_CARRIER_FIXED, 0, 0, 9)).unwrap();
        assert_eq!(fixed.navsat_status(), 2);
        assert_eq!(fixed.carrier_solution(), CarrierSolution::Fixed);
        assert!(fixed.diff_soln());
        let float = NavPvt::parse(&pvt_payload(1, 3, NavPvt::FLAG_GNSS_FIX_OK | NavPvt::FLAG_CARRIER_FLOAT, 0, 0, 9)).unwrap();
        assert_eq!(float.navsat_status(), 0);
        assert_eq!(float.carrier_solution(), CarrierSolution::Float);
        // 3D solution but outside the DOP/accuracy masks -> no fix
        let masked = NavPvt::parse(&pvt_payload(1, 3, 0, 0, 0, 9)).unwrap();
        assert_eq!(masked.navsat_status(), -1);
        // time-only / dead-reckoning-only fixes are not positions
        let time_only = NavPvt::parse(&pvt_payload(1, 5, NavPvt::FLAG_GNSS_FIX_OK, 0, 0, 9)).unwrap();
        assert_eq!(time_only.navsat_status(), -1);
        let dr_only = NavPvt::parse(&pvt_payload(1, 1, NavPvt::FLAG_GNSS_FIX_OK, 0, 0, 9)).unwrap();
        assert_eq!(dr_only.navsat_status(), -1);
        let gnss_dr = NavPvt::parse(&pvt_payload(1, 4, NavPvt::FLAG_GNSS_FIX_OK, 0, 0, 9)).unwrap();
        assert_eq!(gnss_dr.navsat_status(), 0);
        assert!(NavPvt::parse(&p[..91]).is_none());
    }

    #[test]
    fn nav_hpposllh_adds_the_high_precision_digits() {
        let mut p = vec![0u8; NavHpPosLlh::LEN];
        p[4..8].copy_from_slice(&777u32.to_le_bytes());
        p[8..12].copy_from_slice(&1_215_432_100i32.to_le_bytes());
        p[12..16].copy_from_slice(&250_123_456i32.to_le_bytes());
        p[16..20].copy_from_slice(&45_123i32.to_le_bytes());
        p[24] = (-25i8) as u8; // lonHp
        p[25] = 37; // latHp
        p[26] = (-4i8) as u8; // heightHp
        p[28..32].copy_from_slice(&141u32.to_le_bytes()); // 14.1 mm
        p[32..36].copy_from_slice(&230u32.to_le_bytes());
        let hp = NavHpPosLlh::parse(&p).unwrap();
        assert_eq!(hp.itow_ms, 777);
        assert!(!hp.invalid);
        assert!((hp.latitude_deg() - 25.012345637).abs() < 1e-12);
        assert!((hp.longitude_deg() - 121.543209975).abs() < 1e-12);
        assert!((hp.altitude_m() - 45.1226).abs() < 1e-9);
        assert!((hp.h_acc_m() - 0.0141).abs() < 1e-12);
        p[3] = 0x01;
        assert!(NavHpPosLlh::parse(&p).unwrap().invalid);
    }

    #[test]
    fn fix_prefers_a_matching_valid_hpposllh_and_nans_without_a_fix() {
        let pvt = NavPvt::parse(&pvt_payload(777, 3, NavPvt::FLAG_GNSS_FIX_OK | NavPvt::FLAG_CARRIER_FIXED, 250_123_456, 1_215_432_100, 12)).unwrap();
        let mut hp = NavHpPosLlh {
            itow_ms: 777,
            invalid: false,
            lon_1e7: 1_215_432_100,
            lat_1e7: 250_123_456,
            height_mm: 45_123,
            h_msl_mm: 0,
            lon_hp: 0,
            lat_hp: 50,
            height_hp: 0,
            h_msl_hp: 0,
            h_acc_0_1mm: 100,
            v_acc_0_1mm: 200,
        };
        let fix = Fix::from_epoch(&pvt, Some(&hp));
        assert_eq!(fix.status, 2);
        assert!(fix.high_precision);
        assert!((fix.latitude_deg - 25.01234565).abs() < 1e-12);
        assert!((fix.h_var_m2 - 0.01 * 0.01).abs() < 1e-15);
        // another epoch's HPPOSLLH is ignored
        hp.itow_ms = 778;
        let fix = Fix::from_epoch(&pvt, Some(&hp));
        assert!(!fix.high_precision);
        assert!((fix.latitude_deg - 25.0123456).abs() < 1e-12);
        assert!((fix.h_var_m2 - 0.012 * 0.012).abs() < 1e-15);
        // invalid HPPOSLLH too
        hp.itow_ms = 777;
        hp.invalid = true;
        assert!(!Fix::from_epoch(&pvt, Some(&hp)).high_precision);
        // no fix -> NaN position, status -1, covariance still reported
        let none = NavPvt::parse(&pvt_payload(777, 0, 0, 0, 0, 4_294_967)).unwrap();
        let fix = Fix::from_epoch(&none, None);
        assert_eq!(fix.status, -1);
        assert!(fix.latitude_deg.is_nan() && fix.longitude_deg.is_nan() && fix.altitude_m.is_nan());
        assert!(fix.h_var_m2 > 1e6);
    }

    fn sat(gnss_id: u8, sv_id: u8, cno: u8, elev: i8, azim: i16, flags: u32) -> Vec<u8> {
        let mut s = vec![gnss_id, sv_id, cno, elev as u8];
        s.extend_from_slice(&azim.to_le_bytes());
        s.extend_from_slice(&(-12i16).to_le_bytes()); // prRes -1.2 m
        s.extend_from_slice(&flags.to_le_bytes());
        s
    }

    #[test]
    fn nav_sat_decodes_every_satellite_and_unknown_positions() {
        let mut p = vec![0u8; NavSat::HEADER_LEN];
        p[0..4].copy_from_slice(&345_600_000u32.to_le_bytes());
        p[4] = 1;
        p[5] = 3;
        p.extend(sat(0, 12, 44, 56, 123, 0x0000_001F)); // GPS, used, healthy, quality 7
        p.extend(sat(6, 3, 0, -91, 400, 0x0000_0001)); // GLONASS, searching, position unknown
        p.extend(sat(3, 25, 38, 10, 360, 0x0000_0044)); // BeiDou, quality 4, diff corr, not used
        let nav = NavSat::parse(&p).unwrap();
        assert_eq!(nav.itow_ms, 345_600_000);
        assert_eq!(nav.version, 1);
        assert_eq!(nav.sats.len(), 3);
        assert_eq!(nav.num_used(), 1);
        let g = nav.sats[0];
        assert_eq!((g.gnss_id, g.sv_id, g.cno, g.elev_deg, g.azim_deg, g.pr_res_dm), (0, 12, 44, Some(56), Some(123), -12));
        assert!(g.sv_used() && !g.diff_corr());
        assert_eq!((g.quality(), g.health()), (7, 1));
        let r = nav.sats[1];
        assert_eq!((r.elev_deg, r.azim_deg, r.quality()), (None, None, 1));
        let c = nav.sats[2];
        assert_eq!((c.elev_deg, c.azim_deg), (Some(10), Some(360)));
        assert!(!c.sv_used() && c.diff_corr());
        assert_eq!((c.quality(), c.health()), (4, 0));
        // numSvs and the length must agree
        assert!(NavSat::parse(&p[..p.len() - 1]).is_none());
        let mut extra = p.clone();
        extra.extend(vec![0u8; NavSat::SAT_LEN]);
        assert!(NavSat::parse(&extra).is_none());
        assert!(NavSat::parse(&p[..7]).is_none());
        let empty = NavSat::parse(&[0, 0, 0, 0, 1, 0, 0, 0]).unwrap();
        assert!(empty.sats.is_empty());
    }

    #[test]
    fn nav_relposned_v1_combines_the_high_precision_parts() {
        let mut p = vec![0u8; NavRelPosNed::LEN_V1];
        p[0] = 1;
        p[2..4].copy_from_slice(&7u16.to_le_bytes());
        p[4..8].copy_from_slice(&1000u32.to_le_bytes());
        p[8..12].copy_from_slice(&1234i32.to_le_bytes()); // N 12.34 m
        p[12..16].copy_from_slice(&(-456i32).to_le_bytes()); // E -4.56 m
        p[16..20].copy_from_slice(&12i32.to_le_bytes()); // D 0.12 m
        p[20..24].copy_from_slice(&1316i32.to_le_bytes()); // length 13.16 m
        p[24..28].copy_from_slice(&33_973_000i32.to_le_bytes()); // 339.73 deg
        p[32] = 56; // +5.6 mm
        p[33] = (-78i8) as u8; // -7.8 mm
        p[34] = (-5i8) as u8;
        p[35] = 9;
        p[36..40].copy_from_slice(&120u32.to_le_bytes()); // 12.0 mm
        p[40..44].copy_from_slice(&110u32.to_le_bytes());
        p[44..48].copy_from_slice(&200u32.to_le_bytes());
        p[48..52].copy_from_slice(&150u32.to_le_bytes());
        p[52..56].copy_from_slice(&52_000u32.to_le_bytes()); // 0.52 deg
        // gnssFixOK | diffSoln | relPosValid | carrSoln fixed | headingValid
        p[60..64].copy_from_slice(&(0x001 | 0x002 | 0x004 | (2 << 3) | 0x100u32).to_le_bytes());
        let r = NavRelPosNed::parse(&p).unwrap();
        assert_eq!((r.version, r.ref_station_id, r.itow_ms), (1, 7, 1000));
        assert_eq!(r.n_m(), 12.3456);
        assert_eq!(r.e_m(), -4.5678);
        assert_eq!(r.d_m(), 0.1195);
        assert_eq!(r.length_m(), Some(13.1609));
        assert_eq!(r.heading_deg(), Some(339.73));
        assert_eq!((r.acc_n_m(), r.acc_e_m(), r.acc_d_m()), (0.012, 0.011, 0.02));
        assert_eq!(r.acc_length_m(), Some(0.015));
        assert_eq!(r.acc_heading_deg(), Some(0.52));
        assert!(r.gnss_fix_ok() && r.diff_soln() && r.rel_pos_valid() && r.heading_valid());
        assert_eq!(r.carrier_solution(), CarrierSolution::Fixed);
        assert!(!r.is_moving() && !r.ref_pos_miss() && !r.ref_obs_miss());

        // no corrections: not valid, no carrier solution, heading withheld
        p[60..64].copy_from_slice(&(0x001 | 0x040 | 0x080u32).to_le_bytes());
        let r = NavRelPosNed::parse(&p).unwrap();
        assert!(!r.rel_pos_valid());
        assert_eq!(r.carrier_solution(), CarrierSolution::None);
        assert!(r.ref_pos_miss() && r.ref_obs_miss());
        assert_eq!((r.heading_deg(), r.acc_heading_deg()), (None, None));
        assert_eq!(r.length_m(), Some(13.1609), "length does not depend on the heading flag");
        p[60..64].copy_from_slice(&(0x004 | (1 << 3) | 0x020u32).to_le_bytes());
        let r = NavRelPosNed::parse(&p).unwrap();
        assert_eq!(r.carrier_solution(), CarrierSolution::Float);
        assert!(r.is_moving());

        assert!(NavRelPosNed::parse(&p[..63]).is_none());
        p[0] = 2;
        assert!(NavRelPosNed::parse(&p).is_none(), "unknown version");
        assert!(NavRelPosNed::parse(&[]).is_none());
    }

    #[test]
    fn nav_relposned_v0_has_no_length_or_heading() {
        let mut p = vec![0u8; NavRelPosNed::LEN_V0];
        p[2..4].copy_from_slice(&3u16.to_le_bytes());
        p[4..8].copy_from_slice(&2000u32.to_le_bytes());
        p[8..12].copy_from_slice(&(-100i32).to_le_bytes()); // N -1.00 m
        p[12..16].copy_from_slice(&250i32.to_le_bytes());
        p[16..20].copy_from_slice(&(-3i32).to_le_bytes());
        p[20] = (-25i8) as u8; // -> -1.0025 m
        p[21] = 1;
        p[22] = (-9i8) as u8;
        p[24..28].copy_from_slice(&80u32.to_le_bytes());
        p[28..32].copy_from_slice(&90u32.to_le_bytes());
        p[32..36].copy_from_slice(&300u32.to_le_bytes());
        // relPosValid | carrSoln float, plus bit 8 which v0 does not define
        p[36..40].copy_from_slice(&(0x004 | (1 << 3) | 0x100u32).to_le_bytes());
        let r = NavRelPosNed::parse(&p).unwrap();
        assert_eq!((r.version, r.ref_station_id, r.itow_ms), (0, 3, 2000));
        assert_eq!((r.n_m(), r.e_m(), r.d_m()), (-1.0025, 2.5001, -0.0309));
        assert_eq!((r.acc_n_m(), r.acc_e_m(), r.acc_d_m()), (0.008, 0.009, 0.03));
        assert_eq!((r.length_m(), r.heading_deg(), r.acc_length_m(), r.acc_heading_deg()), (None, None, None, None));
        assert!(r.rel_pos_valid() && !r.heading_valid());
        assert_eq!(r.carrier_solution(), CarrierSolution::Float);
        assert!(NavRelPosNed::parse(&p[..39]).is_none());
    }

    #[test]
    fn eoe_and_ack_parse() {
        assert_eq!(nav_eoe_itow(&5u32.to_le_bytes()), Some(5));
        assert_eq!(nav_eoe_itow(&[1, 2]), None);
        let nak = Frame { class: class::ACK, id: id::ACK_NAK, payload: vec![0x06, 0x01] };
        assert_eq!(Ack::parse(&nak), Some(Ack { class: 0x06, id: 0x01, ok: false }));
        let other = Frame { class: class::NAV, id: id::PVT, payload: vec![0; 92] };
        assert_eq!(Ack::parse(&other), None);
    }
}
