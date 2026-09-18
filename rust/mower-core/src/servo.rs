//! MG996 servo command state (mirrors `Module/Src/mg996_servo.cpp` minus
//! the timer): pulse clamp, hold timeout, current-limit gating and the
//! `0x88` status. The firmware owns the TIM10 edges and only asks this type
//! two things — which pulse width, and whether pulses should go out at all.

use crate::protocol::{saturating_u16, servo_status_flag, ServoStatus};

pub const MIN_PULSE_US: u16 = 500;
pub const CENTER_PULSE_US: u16 = 1500;
pub const MAX_PULSE_US: u16 = 2500;
pub const PERIOD_US: u16 = 20_000;

pub fn clamp_pulse(pulse_us: u16) -> u16 {
    pulse_us.clamp(MIN_PULSE_US, MAX_PULSE_US)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Servo {
    pub pulse_us: u16,
    /// 0 = hold until the next command.
    pub hold_timeout_ms: u16,
    pub last_rx_seq: u8,
    enabled: bool,
    limit_active: bool,
    timed_out: bool,
    /// `None` until the first command.
    received_at_ms: Option<u32>,
}

impl Default for Servo {
    fn default() -> Self {
        Self::new()
    }
}

impl Servo {
    pub const fn new() -> Self {
        Self {
            pulse_us: CENTER_PULSE_US,
            hold_timeout_ms: 0,
            last_rx_seq: 0,
            enabled: false,
            limit_active: false,
            timed_out: false,
            received_at_ms: None,
        }
    }

    /// A `0x06` arrived. `pulse_us == 0` releases the servo (no pulses).
    pub fn command(&mut self, pulse_us: u16, hold_timeout_ms: u16, rx_seq: u8, now_ms: u32) {
        if pulse_us == 0 {
            self.disable();
            return;
        }
        self.pulse_us = clamp_pulse(pulse_us);
        self.hold_timeout_ms = hold_timeout_ms;
        self.last_rx_seq = rx_seq;
        self.received_at_ms = Some(now_ms);
        self.enabled = true;
        self.timed_out = false;
    }

    pub fn disable(&mut self) {
        self.enabled = false;
    }

    /// ADC current limit: pulses are held off while active, and resume by
    /// themselves when it clears — no new command needed.
    pub fn set_limit_active(&mut self, active: bool) {
        self.limit_active = active;
    }

    /// Age the hold timeout. Call periodically (the firmware does it on the
    /// 20 ms control tick).
    pub fn tick(&mut self, now_ms: u32) {
        if self.enabled && self.hold_timeout_ms != 0 && self.age_ms(now_ms) >= u32::from(self.hold_timeout_ms) {
            self.enabled = false;
            self.timed_out = true;
        }
    }

    /// Whether the timer should be emitting pulses right now.
    pub fn output_active(&self) -> bool {
        self.enabled && !self.limit_active
    }

    fn age_ms(&self, now_ms: u32) -> u32 {
        self.received_at_ms.map_or(0, |t| now_ms.wrapping_sub(t))
    }

    pub fn flags(&self) -> u8 {
        (if self.enabled { servo_status_flag::ENABLED } else { 0 })
            | (if self.limit_active { servo_status_flag::LIMIT_ACTIVE } else { 0 })
            | (if self.output_active() { servo_status_flag::OUTPUT_ACTIVE } else { 0 })
            | (if self.timed_out { servo_status_flag::TIMED_OUT } else { 0 })
    }

    /// The `0x88` payload as of `now_ms`.
    pub fn status(&self, now_ms: u32) -> ServoStatus {
        ServoStatus {
            pulse_us: self.pulse_us,
            hold_timeout_ms: self.hold_timeout_ms,
            command_age_ms: saturating_u16(self.age_ms(now_ms)),
            flags: self.flags(),
            last_rx_seq: self.last_rx_seq,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn idle_until_first_command() {
        let s = Servo::new();
        assert!(!s.output_active());
        let st = s.status(5000);
        let (flags, age) = (st.flags, st.command_age_ms);
        assert_eq!(flags, 0);
        assert_eq!(age, 0);
        assert_eq!(s.pulse_us, CENTER_PULSE_US);
    }

    #[test]
    fn command_clamps_and_enables() {
        let mut s = Servo::new();
        s.command(100, 0, 7, 1000);
        assert_eq!(s.pulse_us, MIN_PULSE_US);
        assert!(s.output_active());
        s.command(9000, 0, 8, 1000);
        assert_eq!(s.pulse_us, MAX_PULSE_US);
        let st = s.status(1250);
        let (age, seq, flags) = (st.command_age_ms, st.last_rx_seq, st.flags);
        assert_eq!(age, 250);
        assert_eq!(seq, 8);
        assert_eq!(flags, servo_status_flag::ENABLED | servo_status_flag::OUTPUT_ACTIVE);
    }

    #[test]
    fn zero_pulse_releases() {
        let mut s = Servo::new();
        s.command(1500, 0, 1, 0);
        s.command(0, 0, 2, 10);
        assert!(!s.output_active());
        // the last target is kept for the status frame
        assert_eq!(s.pulse_us, 1500);
        assert_eq!(s.last_rx_seq, 1);
    }

    #[test]
    fn hold_timeout_stops_pulses_and_flags_it() {
        let mut s = Servo::new();
        s.command(1500, 300, 1, 1000);
        s.tick(1299);
        assert!(s.output_active());
        s.tick(1300);
        assert!(!s.output_active());
        assert_eq!(s.flags(), servo_status_flag::TIMED_OUT);
        // a fresh command clears the timeout
        s.command(1600, 300, 2, 1300);
        assert!(s.output_active());
        assert_eq!(s.flags(), servo_status_flag::ENABLED | servo_status_flag::OUTPUT_ACTIVE);
    }

    #[test]
    fn zero_hold_timeout_holds_forever() {
        let mut s = Servo::new();
        s.command(1500, 0, 1, 0);
        s.tick(u32::MAX);
        assert!(s.output_active());
    }

    #[test]
    fn current_limit_gates_output_without_losing_the_command() {
        let mut s = Servo::new();
        s.command(2000, 0, 1, 0);
        s.set_limit_active(true);
        assert!(!s.output_active());
        assert_eq!(s.flags(), servo_status_flag::ENABLED | servo_status_flag::LIMIT_ACTIVE);
        s.set_limit_active(false);
        assert!(s.output_active());
        assert_eq!(s.pulse_us, 2000);
    }
}
