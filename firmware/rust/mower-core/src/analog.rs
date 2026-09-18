//! Analog monitor — the hardware-free half of `Module/Src/analog_monitor.cpp`:
//! raw ADC counts from the 4-channel mux → voltages, battery voltages through
//! their dividers, NTC temperature, the MG996 current-limit decision and the
//! `0x8A` frame.
//!
//! The firmware task selects the mux channel, reads ADC1 and calls
//! [`Snapshot::record`]; before each scan it reads VREFINT and calls
//! [`Snapshot::calibrate_vdda`] so nothing here assumes a 3.3 V rail.

use crate::protocol::{analog_status_flag, AnalogStatus};

pub const CHANNEL_COUNT: usize = 4;
/// One full scan every this many ms (the C++ `board_modules` cadence).
pub const SCAN_PERIOD_MS: u64 = 200;

/// Nominal rail, used until VREFINT has been read.
pub const NOMINAL_VDDA_V: f32 = 3.3;
/// Vref+ at which the factory wrote VREFINT_CAL (RM0383).
pub const VREFINT_CAL_VREF_V: f32 = 3.3;
const VDDA_MIN_PLAUSIBLE_V: f32 = 2.4;
const VDDA_MAX_PLAUSIBLE_V: f32 = 3.6;
const ADC_FULL_SCALE: f32 = 4095.0;

/// 24 V main battery: 270k over 33k.
pub const MAIN_BATTERY_DIVIDER_GAIN: f32 = (270_000.0 + 33_000.0) / 33_000.0;
/// 3.7 V AON battery: 100k over 300k.
pub const AON_BATTERY_DIVIDER_GAIN: f32 = (100_000.0 + 300_000.0) / 300_000.0;
const NTC_PULLUP_OHMS: f32 = 10_000.0;
const NTC_NOMINAL_OHMS: f32 = 10_000.0;
const NTC_BETA: f32 = 3950.0;
const NTC_NOMINAL_K: f32 = 298.15;
/// What the C++ build reports when the NTC reads open / shorted.
pub const TEMP_INVALID_C: f32 = -273.15;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Channel {
    Mg996Current = 0,
    BoardTemp = 1,
    MainBattery = 2,
    AonBattery = 3,
}

impl Channel {
    pub const ALL: [Channel; CHANNEL_COUNT] =
        [Channel::Mg996Current, Channel::BoardTemp, Channel::MainBattery, Channel::AonBattery];

    /// (S0, S1) mux select levels.
    pub fn select_bits(self) -> (bool, bool) {
        let raw = self as u8;
        ((raw & 0x01) != 0, (raw & 0x02) != 0)
    }
}

/// VDDA from a VREFINT conversion and the factory calibration word.
/// `None` when either value is implausible (blank calibration, ADC stuck).
pub fn vdda_from_vrefint(raw: u16, cal: u16) -> Option<f32> {
    if raw == 0 || cal == 0 || cal == 0xFFFF {
        return None;
    }
    let vdda = VREFINT_CAL_VREF_V * cal as f32 / raw as f32;
    (VDDA_MIN_PLAUSIBLE_V..=VDDA_MAX_PLAUSIBLE_V).contains(&vdda).then_some(vdda)
}

/// NTC (10k, beta 3950) with a 10k pull-up to the ADC rail. Only the ratio
/// matters, so the absolute rail voltage cancels out.
pub fn ntc_voltage_to_c(voltage: f32, vdda: f32) -> f32 {
    if voltage <= 0.01 || voltage >= vdda - 0.01 {
        return TEMP_INVALID_C;
    }
    let ntc_ohms = NTC_PULLUP_OHMS * voltage / (vdda - voltage);
    let inv_t = 1.0 / NTC_NOMINAL_K + libm::logf(ntc_ohms / NTC_NOMINAL_OHMS) / NTC_BETA;
    1.0 / inv_t - 273.15
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Snapshot {
    pub raw: [u16; CHANNEL_COUNT],
    pub valid: [bool; CHANNEL_COUNT],
    pub vdda_v: f32,
    pub vdda_calibrated: bool,
    /// MG996 current sense raw threshold; 4095 = never trips (C++ default).
    pub mg996_limit_raw: u16,
}

impl Snapshot {
    pub const ZERO: Self = Self {
        raw: [0; CHANNEL_COUNT],
        valid: [false; CHANNEL_COUNT],
        vdda_v: NOMINAL_VDDA_V,
        vdda_calibrated: false,
        mg996_limit_raw: 4095,
    };

    /// Result of one VREFINT read; keeps the previous VDDA on `None`.
    pub fn calibrate_vdda(&mut self, vdda: Option<f32>) {
        if let Some(v) = vdda {
            self.vdda_v = v;
            self.vdda_calibrated = true;
        }
    }

    /// Result of one mux channel read (`None` = conversion failed).
    pub fn record(&mut self, channel: Channel, raw: Option<u16>) {
        let i = channel as usize;
        match raw {
            Some(r) => {
                self.raw[i] = r;
                self.valid[i] = true;
            }
            None => self.valid[i] = false,
        }
    }

    pub fn adc_available(&self) -> bool {
        self.valid.iter().any(|v| *v)
    }

    pub fn adc_voltage(&self, channel: Channel) -> f32 {
        self.raw[channel as usize] as f32 * self.vdda_v / ADC_FULL_SCALE
    }

    fn scaled(&self, channel: Channel, gain: f32) -> f32 {
        if self.valid[channel as usize] {
            self.adc_voltage(channel) * gain
        } else {
            0.0
        }
    }

    pub fn main_battery_v(&self) -> f32 {
        self.scaled(Channel::MainBattery, MAIN_BATTERY_DIVIDER_GAIN)
    }

    pub fn aon_battery_v(&self) -> f32 {
        self.scaled(Channel::AonBattery, AON_BATTERY_DIVIDER_GAIN)
    }

    pub fn board_temperature_c(&self) -> f32 {
        if self.valid[Channel::BoardTemp as usize] {
            ntc_voltage_to_c(self.adc_voltage(Channel::BoardTemp), self.vdda_v)
        } else {
            TEMP_INVALID_C
        }
    }

    pub fn mg996_current_limit(&self) -> bool {
        self.valid[Channel::Mg996Current as usize] && self.raw[Channel::Mg996Current as usize] >= self.mg996_limit_raw
    }

    /// The `0x8A` payload, same rounding as `uart_send_analog_status()`.
    pub fn status(&self) -> AnalogStatus {
        let mut flags = 0u8;
        let mut st = AnalogStatus {
            main_battery_cv: 0,
            aon_battery_cv: 0,
            board_temp_dc: i16::MIN,
            vdda_mv: (self.vdda_v * 1000.0 + 0.5) as u16,
            mg996_current_raw: 0,
            flags: 0,
            reserved: 0,
        };
        if self.valid[Channel::MainBattery as usize] {
            flags |= analog_status_flag::MAIN_BATTERY_VALID;
            st.main_battery_cv = volts_to_cv(self.main_battery_v());
        }
        if self.valid[Channel::AonBattery as usize] {
            flags |= analog_status_flag::AON_BATTERY_VALID;
            st.aon_battery_cv = volts_to_cv(self.aon_battery_v());
        }
        let temp = self.board_temperature_c();
        if self.valid[Channel::BoardTemp as usize] && temp > -100.0 {
            flags |= analog_status_flag::BOARD_TEMP_VALID;
            let dc = (temp * 10.0).min(3000.0);
            st.board_temp_dc = (dc + if dc >= 0.0 { 0.5 } else { -0.5 }) as i16;
        }
        if self.valid[Channel::Mg996Current as usize] {
            flags |= analog_status_flag::MG996_CURRENT_VALID;
            st.mg996_current_raw = self.raw[Channel::Mg996Current as usize];
        }
        if self.vdda_calibrated {
            flags |= analog_status_flag::VDDA_CALIBRATED;
        }
        if self.mg996_current_limit() {
            flags |= analog_status_flag::MG996_LIMIT_ACTIVE;
        }
        st.flags = flags;
        st
    }
}

fn volts_to_cv(volts: f32) -> u16 {
    if volts.is_nan() || volts <= 0.0 {
        return 0;
    }
    let cv = volts * 100.0 + 0.5;
    if cv >= 65535.0 {
        u16::MAX
    } else {
        cv as u16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mux_select_bits_follow_channel_index() {
        assert_eq!(Channel::Mg996Current.select_bits(), (false, false));
        assert_eq!(Channel::BoardTemp.select_bits(), (true, false));
        assert_eq!(Channel::MainBattery.select_bits(), (false, true));
        assert_eq!(Channel::AonBattery.select_bits(), (true, true));
    }

    #[test]
    fn vrefint_calibration() {
        // cal 1500 at 3.3 V; reading 1700 => rail is 2.912 V
        let v = vdda_from_vrefint(1700, 1500).unwrap();
        assert!((v - 2.9118).abs() < 1e-3);
        assert_eq!(vdda_from_vrefint(0, 1500), None);
        assert_eq!(vdda_from_vrefint(1700, 0xFFFF), None);
        assert_eq!(vdda_from_vrefint(100, 1500), None); // 49 V, nonsense
    }

    #[test]
    fn battery_voltages_use_the_calibrated_rail() {
        let mut s = Snapshot::ZERO;
        // 24.0 V through 270k/33k => 2.614 V at the pin; at a 3.3 V rail that is 3244 counts
        s.record(Channel::MainBattery, Some(3244));
        assert!((s.main_battery_v() - 24.0).abs() < 0.05);
        // same counts with the rail really at 2.9 V => the pack is lower
        s.calibrate_vdda(Some(2.9));
        assert!((s.main_battery_v() - 21.09).abs() < 0.05);
        assert!(s.vdda_calibrated);
        s.calibrate_vdda(None);
        assert!((s.vdda_v - 2.9).abs() < 1e-6);

        s.record(Channel::AonBattery, Some(3915)); // 2.773 V * 4/3 = 3.70 V at 2.9 V rail
        assert!((s.aon_battery_v() - 3.698).abs() < 0.02);
    }

    #[test]
    fn ntc_nominal_is_25c_and_open_is_invalid() {
        let vdda = 3.0;
        assert!((ntc_voltage_to_c(vdda / 2.0, vdda) - 25.0).abs() < 0.05);
        assert_eq!(ntc_voltage_to_c(0.0, vdda), TEMP_INVALID_C);
        assert_eq!(ntc_voltage_to_c(vdda, vdda), TEMP_INVALID_C);
        // hotter => lower resistance => lower voltage
        assert!(ntc_voltage_to_c(1.0, vdda) > 25.0);
    }

    #[test]
    fn current_limit_and_status_flags() {
        let mut s = Snapshot::ZERO;
        let empty = s.status();
        let (flags, temp, vdda) = (empty.flags, empty.board_temp_dc, empty.vdda_mv);
        assert_eq!(flags, 0);
        assert_eq!(temp, i16::MIN);
        assert_eq!(vdda, 3300);

        s.mg996_limit_raw = 2000;
        s.record(Channel::Mg996Current, Some(2500));
        s.record(Channel::BoardTemp, Some(2047)); // ~25 C
        s.record(Channel::MainBattery, Some(3244));
        s.calibrate_vdda(Some(3.3));
        assert!(s.mg996_current_limit());
        let st = s.status();
        // copy the packed fields out before comparing (unaligned references)
        let (flags, raw, main, temp, aon) =
            (st.flags, st.mg996_current_raw, st.main_battery_cv, st.board_temp_dc, st.aon_battery_cv);
        assert_eq!(
            flags,
            analog_status_flag::MAIN_BATTERY_VALID
                | analog_status_flag::BOARD_TEMP_VALID
                | analog_status_flag::MG996_CURRENT_VALID
                | analog_status_flag::VDDA_CALIBRATED
                | analog_status_flag::MG996_LIMIT_ACTIVE
        );
        assert_eq!(raw, 2500);
        assert!((main as i32 - 2400).abs() <= 5);
        assert!((temp - 250).abs() <= 2);
        assert_eq!(aon, 0);

        s.record(Channel::Mg996Current, None);
        assert!(!s.mg996_current_limit());
        let flags = s.status().flags;
        assert_eq!(flags & analog_status_flag::MG996_CURRENT_VALID, 0);
    }

    #[test]
    fn volts_to_cv_saturates() {
        assert_eq!(volts_to_cv(-1.0), 0);
        assert_eq!(volts_to_cv(f32::NAN), 0);
        assert_eq!(volts_to_cv(24.124), 2412);
        assert_eq!(volts_to_cv(1e6), u16::MAX);
    }
}
