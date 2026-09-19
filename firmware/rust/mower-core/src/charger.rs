//! RS485 charge-line meter (voltage / current / temperature, no CC/CV
//! settings) — the hardware-free half of `Module/Src/charger_rs485.cpp`:
//! what the last poll said, whether the meter counts as online, and how
//! that maps onto the `0x89` frame.
//!
//! The firmware task does the UART work (request, reply, timeout) and calls
//! [`Snapshot::record_success`] / [`Snapshot::record_failure`].

use crate::modbus;
use crate::protocol::{charger_status_flag, saturating_u16, ChargerStatus};

pub const SLAVE_ADDR: u8 = 0x01;
/// Holding registers 0-4: voltage (x0.01 V), current (x0.01 A),
/// temperature (degC), then two constants of unknown meaning.
pub const REG_COUNT: usize = 5;
pub const POLL_PERIOD_MS: u64 = 500;
pub const REPLY_TIMEOUT_MS: u64 = 200;
/// Consecutive failed polls before `online` drops.
pub const OFFLINE_AFTER_FAILS: u8 = 3;
/// Current at or above this counts as "current present" (x0.01 A); the meter
/// sits in the pack lead so this is charge or discharge.
pub const CURRENT_MIN_CA: u16 = 5;
/// Line voltage at or above this counts as "input present" (x0.01 V).
pub const INPUT_PRESENT_MIN_CV: u16 = 500;

/// The FC03 request the poller sends every period.
pub fn read_request() -> [u8; modbus::READ_REQUEST_LEN] {
    modbus::build_read_holding(SLAVE_ADDR, 0, REG_COUNT as u16)
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Snapshot {
    pub voltage_cv: u16,
    pub current_ca: u16,
    pub temp_c: u16,
    pub reg3: u16,
    pub reg4: u16,
    /// Last poll(s) answered with a valid frame.
    pub online: bool,
    /// `Some(tick)` of the last valid reply; `None` until the first one.
    pub last_ok_ms: Option<u32>,
    /// Wraps; timeouts + bad frames since boot.
    pub comm_error_count: u8,
    pub consecutive_fails: u8,
    pub last_exception_code: u8,
}

impl Snapshot {
    pub const ZERO: Self = Self {
        voltage_cv: 0,
        current_ca: 0,
        temp_c: 0,
        reg3: 0,
        reg4: 0,
        online: false,
        last_ok_ms: None,
        comm_error_count: 0,
        consecutive_fails: 0,
        last_exception_code: 0,
    };

    /// A reply with at least [`REG_COUNT`] registers arrived at `now_ms`.
    pub fn record_success(&mut self, regs: &[u16; REG_COUNT], now_ms: u32) {
        self.voltage_cv = regs[0];
        self.current_ca = regs[1];
        self.temp_c = regs[2];
        self.reg3 = regs[3];
        self.reg4 = regs[4];
        self.online = true;
        self.last_ok_ms = Some(now_ms);
        self.consecutive_fails = 0;
        self.last_exception_code = 0;
    }

    /// Timeout, bad frame, short reply or (with `exception`) a Modbus
    /// exception. The last good values are kept; `online` drops after
    /// [`OFFLINE_AFTER_FAILS`] in a row.
    pub fn record_failure(&mut self, exception: Option<u8>) {
        self.comm_error_count = self.comm_error_count.wrapping_add(1);
        self.consecutive_fails = self.consecutive_fails.saturating_add(1);
        if let Some(code) = exception {
            self.last_exception_code = code;
        }
        if self.consecutive_fails >= OFFLINE_AFTER_FAILS {
            self.online = false;
        }
    }

    /// While polling is paused (rail off) nothing refreshes the snapshot;
    /// age out `online` as if the replies had stopped.
    pub fn age_out(&mut self, now_ms: u32) {
        const STALE_MS: u32 = (POLL_PERIOD_MS * OFFLINE_AFTER_FAILS as u64) as u32;
        if let Some(t) = self.last_ok_ms {
            if self.online && now_ms.wrapping_sub(t) > STALE_MS {
                self.online = false;
            }
        }
    }

    pub fn ever_seen(&self) -> bool {
        self.last_ok_ms.is_some()
    }

    pub fn is_current_present(&self) -> bool {
        self.online && self.current_ca >= CURRENT_MIN_CA
    }

    pub fn is_input_present(&self) -> bool {
        self.online && self.voltage_cv >= INPUT_PRESENT_MIN_CV
    }

    pub fn flags(&self) -> u8 {
        (if self.online { charger_status_flag::ONLINE } else { 0 })
            | (if self.is_current_present() { charger_status_flag::CURRENT_PRESENT } else { 0 })
            | (if self.is_input_present() { charger_status_flag::INPUT_PRESENT } else { 0 })
            | (if self.ever_seen() { charger_status_flag::EVER_SEEN } else { 0 })
    }

    /// The `0x89` payload as of `now_ms`.
    pub fn status(&self, now_ms: u32) -> ChargerStatus {
        ChargerStatus {
            voltage_cv: self.voltage_cv,
            current_ca: self.current_ca,
            temp_c: self.temp_c,
            reg3: self.reg3,
            reg4: self.reg4,
            flags: self.flags(),
            comm_error_count: self.comm_error_count,
            age_ms: match self.last_ok_ms {
                Some(t) => saturating_u16(now_ms.wrapping_sub(t)),
                None => u16::MAX,
            },
            last_exception_code: self.last_exception_code,
            reserved: 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Measured 2026-09-19 against the meter's display: 25.63 V, 0.00 A, 35 degC
    const REGS: [u16; REG_COUNT] = [2563, 0, 35, 11, 48961];

    #[test]
    fn request_targets_regs_0_to_4_of_addr_1() {
        assert_eq!(read_request(), [0x01, 0x03, 0x00, 0x00, 0x00, 0x05, 0x85, 0xC9]);
    }

    #[test]
    fn never_seen_reports_max_age_and_no_flags() {
        let s = Snapshot::ZERO;
        let st = s.status(1000);
        let (age, flags) = (st.age_ms, st.flags);
        assert_eq!(age, u16::MAX);
        assert_eq!(flags, 0);
    }

    #[test]
    fn success_sets_values_online_and_flags() {
        let mut s = Snapshot::ZERO;
        s.record_success(&REGS, 1000);
        let st = s.status(1120);
        assert_eq!((st.voltage_cv, st.current_ca, st.temp_c, st.reg3, st.reg4), (2563, 0, 35, 11, 48961));
        let (age, flags) = (st.age_ms, st.flags);
        assert_eq!(age, 120);
        // online, input present, no current (0 A); the CV_PHASE bit is never set
        assert_eq!(
            flags,
            charger_status_flag::ONLINE | charger_status_flag::INPUT_PRESENT | charger_status_flag::EVER_SEEN
        );
    }

    #[test]
    fn current_present_needs_current_and_online() {
        let mut s = Snapshot::ZERO;
        s.record_success(&[2450, 150, 36, 11, 48961], 0);
        assert!(s.is_current_present());
        for _ in 0..OFFLINE_AFTER_FAILS {
            s.record_failure(None);
        }
        assert!(!s.online);
        assert!(!s.is_current_present());
        // values are kept for the host to see
        assert_eq!(s.current_ca, 150);
        let flags = s.status(0).flags;
        assert_eq!(flags, charger_status_flag::EVER_SEEN);
    }

    #[test]
    fn offline_only_after_three_consecutive_failures() {
        let mut s = Snapshot::ZERO;
        s.record_success(&REGS, 0);
        s.record_failure(None);
        s.record_failure(Some(2));
        assert!(s.online);
        assert_eq!(s.last_exception_code, 2);
        assert_eq!(s.comm_error_count, 2);
        s.record_success(&REGS, 1500);
        assert_eq!(s.consecutive_fails, 0);
        assert_eq!(s.last_exception_code, 0);
        s.record_failure(None);
        s.record_failure(None);
        s.record_failure(None);
        assert!(!s.online);
    }

    #[test]
    fn error_counter_wraps() {
        let mut s = Snapshot::ZERO;
        s.comm_error_count = u8::MAX;
        s.record_failure(None);
        assert_eq!(s.comm_error_count, 0);
    }

    #[test]
    fn paused_polling_ages_online_out() {
        let mut s = Snapshot::ZERO;
        s.record_success(&REGS, 1000);
        s.age_out(1000 + 1500);
        assert!(s.online);
        s.age_out(1000 + 1501);
        assert!(!s.online);
    }
}
