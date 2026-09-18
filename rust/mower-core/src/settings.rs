//! Controller settings and their flash record (port of `settings_storage.cpp`).
//!
//! The record is decoded from raw flash bytes with `zerocopy`, so the
//! `reinterpret_cast<const settings_record_t *>(FLASH_ADDRESS)` of the C++
//! version becomes a checked, alignment-free copy.

use crate::pid::Gains;
use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout};

pub const FLASH_ADDRESS: u32 = 0x0806_0000;
pub const FLASH_SECTOR: u8 = 7;
pub const VERSION: u32 = 1;
const MAGIC: u32 = 0x4D57_5243; // "MWRC"
pub const WHEEL_DEFAULT_MAX_RPM: f32 = 58.0;

#[derive(Clone, Copy, Debug, PartialEq, FromBytes, IntoBytes, Immutable, KnownLayout)]
#[repr(C)]
pub struct ControllerSettings {
    pub left_wheel_pid: Gains,
    pub right_wheel_pid: Gains,
    pub wheel_max_rpm: f32,
    /// 0 or 1; u32 to keep the C layout.
    pub closed_loop_enabled: u32,
}

const _: () = assert!(core::mem::size_of::<ControllerSettings>() == 64);

impl Default for ControllerSettings {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl ControllerSettings {
    /// Factory defaults (`SettingsStorage_LoadDefaults`). `const` so the
    /// firmware can seed a `static` with it.
    pub const DEFAULT: Self = {
        let wheel = Gains {
            kp: 2.0,
            ki: 0.6,
            kd: 0.0,
            integral_min: -80.0,
            integral_max: 80.0,
            output_min: -200.0,
            output_max: 200.0,
        };
        Self {
            left_wheel_pid: wheel,
            right_wheel_pid: wheel,
            wheel_max_rpm: WHEEL_DEFAULT_MAX_RPM,
            closed_loop_enabled: 1,
        }
    };

    pub fn closed_loop(&self) -> bool {
        self.closed_loop_enabled != 0
    }

    /// Replace NaN / out-of-range values with the defaults, field by field.
    pub fn sanitized(mut self) -> Self {
        let defaults = Self::default();
        self.left_wheel_pid = sanitize_gains(self.left_wheel_pid, &defaults.left_wheel_pid);
        self.right_wheel_pid = sanitize_gains(self.right_wheel_pid, &defaults.right_wheel_pid);
        self.wheel_max_rpm = sanitize_f32(self.wheel_max_rpm, defaults.wheel_max_rpm, 1.0, 300.0);
        self.closed_loop_enabled = u32::from(self.closed_loop_enabled != 0);
        self
    }
}

fn sanitize_f32(value: f32, fallback: f32, min: f32, max: f32) -> f32 {
    if value.is_nan() || value < min || value > max {
        fallback
    } else {
        value
    }
}

fn sanitize_gains(mut g: Gains, fallback: &Gains) -> Gains {
    g.kp = sanitize_f32(g.kp, fallback.kp, 0.0, 1000.0);
    g.ki = sanitize_f32(g.ki, fallback.ki, 0.0, 1000.0);
    g.kd = sanitize_f32(g.kd, fallback.kd, 0.0, 1000.0);
    g.integral_min = sanitize_f32(g.integral_min, fallback.integral_min, -1000.0, 1000.0);
    g.integral_max = sanitize_f32(g.integral_max, fallback.integral_max, -1000.0, 1000.0);
    g.output_min = sanitize_f32(g.output_min, fallback.output_min, -1000.0, 1000.0);
    g.output_max = sanitize_f32(g.output_max, fallback.output_max, -1000.0, 1000.0);
    if g.integral_min > g.integral_max {
        g.integral_min = fallback.integral_min;
        g.integral_max = fallback.integral_max;
    }
    if g.output_min > g.output_max {
        g.output_min = fallback.output_min;
        g.output_max = fallback.output_max;
    }
    g
}

/// On-flash record. Word-aligned (84 bytes) so it can be programmed with
/// 32-bit writes.
#[derive(Clone, Copy, Debug, PartialEq, FromBytes, IntoBytes, Immutable, KnownLayout)]
#[repr(C)]
pub struct Record {
    pub magic: u32,
    pub version: u32,
    pub size: u32,
    pub sequence: u32,
    pub settings: ControllerSettings,
    pub crc32: u32,
}

pub const RECORD_SIZE: usize = core::mem::size_of::<Record>();
const _: () = assert!(RECORD_SIZE == 84);
const _: () = assert!(RECORD_SIZE.is_multiple_of(4), "settings record must be word aligned");
const CRC_OFFSET: usize = core::mem::offset_of!(Record, crc32);

impl Record {
    pub fn new(settings: ControllerSettings, sequence: u32) -> Self {
        let mut record =
            Self { magic: MAGIC, version: VERSION, size: RECORD_SIZE as u32, sequence, settings, crc32: 0 };
        record.crc32 = crc32_ieee(&record.as_bytes()[..CRC_OFFSET]);
        record
    }

    /// Decode a record from the first [`RECORD_SIZE`] bytes of `flash`.
    /// `None` if too short, wrong magic/version/size or CRC mismatch.
    pub fn from_flash(flash: &[u8]) -> Option<Self> {
        let bytes = flash.get(..RECORD_SIZE)?;
        let record = Self::read_from_bytes(bytes).ok()?;
        let header_ok = record.magic == MAGIC && record.version == VERSION && record.size as usize == RECORD_SIZE;
        if !header_ok || crc32_ieee(&bytes[..CRC_OFFSET]) != record.crc32 {
            return None;
        }
        Some(record)
    }
}

/// CRC-32 (IEEE 802.3, reflected, same as `zlib.crc32`).
pub fn crc32_ieee(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &byte in data {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xEDB8_8320 } else { crc >> 1 };
        }
    }
    !crc
}

/// Whether the last load/save succeeded — feeds the status flags.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StorageStatus {
    pub flash_valid: bool,
    pub last_save_ok: bool,
    pub sequence: u32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc32_reference_vector() {
        assert_eq!(crc32_ieee(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn record_round_trip() {
        let record = Record::new(ControllerSettings::default(), 5);
        let decoded = Record::from_flash(record.as_bytes()).expect("valid");
        assert_eq!(decoded, record);
        assert_eq!(decoded.sequence, 5);
    }

    #[test]
    fn erased_flash_is_invalid() {
        assert!(Record::from_flash(&[0xFF; RECORD_SIZE]).is_none());
        assert!(Record::from_flash(&[0xFF; RECORD_SIZE - 1]).is_none());
    }

    #[test]
    fn bit_flip_is_detected() {
        let record = Record::new(ControllerSettings::default(), 1);
        let mut bytes = [0u8; RECORD_SIZE];
        bytes.copy_from_slice(record.as_bytes());
        bytes[20] ^= 0x10; // inside settings
        assert!(Record::from_flash(&bytes).is_none());
    }

    #[test]
    fn sanitize_replaces_bad_values() {
        let mut s = ControllerSettings::default();
        s.left_wheel_pid.kp = f32::NAN;
        s.right_wheel_pid.kd = -1.0;
        s.wheel_max_rpm = 1e6;
        s.closed_loop_enabled = 7;
        s.left_wheel_pid.integral_min = 50.0;
        s.left_wheel_pid.integral_max = -50.0;
        let s = s.sanitized();
        let d = ControllerSettings::default();
        assert_eq!(s.left_wheel_pid.kp, d.left_wheel_pid.kp);
        assert_eq!(s.right_wheel_pid.kd, d.right_wheel_pid.kd);
        assert_eq!(s.wheel_max_rpm, d.wheel_max_rpm);
        assert_eq!(s.closed_loop_enabled, 1);
        assert_eq!(s.left_wheel_pid.integral_min, d.left_wheel_pid.integral_min);
    }
}
