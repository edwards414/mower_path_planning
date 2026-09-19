//! u-blox UBX protocol, the part `mower_gps` needs: a resynchronising frame
//! parser, the 8-bit Fletcher checksum, encoders for the legacy CFG
//! messages that set the output rate on the current port, and decoders for
//! NAV-PVT, NAV-HPPOSLLH, NAV-EOE and ACK-ACK/NAK. No ROS or I/O in here so
//! it is unit-tested on any host with plain cargo.
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
