//! RS485 charger (数控 30V5A CC/CV module) — the hardware-free half of
//! `Module/Src/charger_rs485.cpp`: what the last poll said, whether the
//! module counts as online, and how that maps onto the `0x89` frame.
//!
//! The firmware task does the UART work (request, reply, timeout) and calls
//! [`Snapshot::record_success`] / [`Snapshot::record_failure`].

use crate::modbus;
use crate::protocol::{charger_status_flag, saturating_u16, ChargerStatus};

pub const SLAVE_ADDR: u8 = 0x01;
/// Holding registers 0-4: Vin, Vout, Iout, set CC, set CV (all x0.01).
pub const REG_COUNT: usize = 5;
pub const POLL_PERIOD_MS: u64 = 500;
pub const REPLY_TIMEOUT_MS: u64 = 200;
/// Consecutive failed polls before `online` drops.
pub const OFFLINE_AFTER_FAILS: u8 = 3;
/// Iout at or above this counts as charging (x0.01 A).
pub const CHARGING_MIN_CA: u16 = 5;
/// Vout within this of set CV counts as the CV (top-off) phase (x0.01 V).
pub const CV_BAND_CV: u16 = 10;
/// Vin at or above this counts as "input present" (x0.01 V).
pub const INPUT_PRESENT_MIN_CV: u16 = 500;

/// The FC03 request the poller sends every period.
pub fn read_request() -> [u8; modbus::READ_REQUEST_LEN] {
    modbus::build_read_holding(SLAVE_ADDR, 0, REG_COUNT as u16)
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Snapshot {
    pub vin_cv: u16,
    pub vout_cv: u16,
    pub iout_ca: u16,
    pub set_cc_ca: u16,
    pub set_cv_cv: u16,
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
        vin_cv: 0,
        vout_cv: 0,
        iout_ca: 0,
        set_cc_ca: 0,
        set_cv_cv: 0,
        online: false,
        last_ok_ms: None,
        comm_error_count: 0,
        consecutive_fails: 0,
        last_exception_code: 0,
    };

    /// A reply with at least [`REG_COUNT`] registers arrived at `now_ms`.
    pub fn record_success(&mut self, regs: &[u16; REG_COUNT], now_ms: u32) {
        self.vin_cv = regs[0];
        self.vout_cv = regs[1];
        self.iout_ca = regs[2];
        self.set_cc_ca = regs[3];
        self.set_cv_cv = regs[4];
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

    pub fn is_charging(&self) -> bool {
        self.online && self.iout_ca >= CHARGING_MIN_CA
    }

    pub fn is_cv_phase(&self) -> bool {
        self.online && self.set_cv_cv != 0 && self.vout_cv >= self.set_cv_cv.saturating_sub(CV_BAND_CV)
    }

    pub fn is_input_present(&self) -> bool {
        self.online && self.vin_cv >= INPUT_PRESENT_MIN_CV
    }

    pub fn flags(&self) -> u8 {
        (if self.online { charger_status_flag::ONLINE } else { 0 })
            | (if self.is_charging() { charger_status_flag::CHARGING } else { 0 })
            | (if self.is_cv_phase() { charger_status_flag::CV_PHASE } else { 0 })
            | (if self.is_input_present() { charger_status_flag::INPUT_PRESENT } else { 0 })
            | (if self.ever_seen() { charger_status_flag::EVER_SEEN } else { 0 })
    }

    /// The `0x89` payload as of `now_ms`.
    pub fn status(&self, now_ms: u32) -> ChargerStatus {
        ChargerStatus {
            vin_cv: self.vin_cv,
            vout_cv: self.vout_cv,
            iout_ca: self.iout_ca,
            set_cc_ca: self.set_cc_ca,
            set_cv_cv: self.set_cv_cv,
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

    // Vendor example: Vin 12.59 V, Vout 4.97 V, Iout 0, CC 2.50 A, CV 5.00 V
    const REGS: [u16; REG_COUNT] = [1259, 497, 0, 250, 500];

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
        assert_eq!((st.vin_cv, st.vout_cv, st.iout_ca, st.set_cc_ca, st.set_cv_cv), (1259, 497, 0, 250, 500));
        let (age, flags) = (st.age_ms, st.flags);
        assert_eq!(age, 120);
        // online, input present, CV phase (4.97 >= 5.00 - 0.10), not charging (0 A)
        assert_eq!(
            flags,
            charger_status_flag::ONLINE
                | charger_status_flag::INPUT_PRESENT
                | charger_status_flag::CV_PHASE
                | charger_status_flag::EVER_SEEN
        );
    }

    #[test]
    fn charging_needs_current_and_online() {
        let mut s = Snapshot::ZERO;
        s.record_success(&[2400, 2300, 150, 300, 2520], 0);
        assert!(s.is_charging());
        assert!(!s.is_cv_phase());
        for _ in 0..OFFLINE_AFTER_FAILS {
            s.record_failure(None);
        }
        assert!(!s.online);
        assert!(!s.is_charging());
        // values are kept for the host to see
        assert_eq!(s.vout_cv, 2300);
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
