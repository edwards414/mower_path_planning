//! MG996 servo command state (mirrors `Module/Src/mg996_servo.cpp` minus
//! the timer): pulse slewing, hold timeout, limit-switch gating and the
//! `0x88` status. The firmware owns the TIM10 edges and only asks this type
//! two things — which pulse width, and whether pulses should go out at all.
//!
//! The mechanism has a microswitch at each end of travel. The servo has no
//! position feedback, so the pulse is slewed towards the host's target at a
//! rate the servo can follow; the commanded pulse then tracks the horn
//! closely enough that "stop where you are" means something. When a switch
//! closes while the pulse is moving towards it, the target is replaced by a
//! short back-off away from the switch (`LIMIT_ACTIVE`) so the servo settles
//! just clear of the end stop instead of leaning on it; motion into a
//! pressed switch is refused, the other direction always works.

use crate::protocol::{saturating_u16, servo_status_flag, ServoStatus};

pub const MIN_PULSE_US: u16 = 500;
pub const CENTER_PULSE_US: u16 = 1500;
pub const MAX_PULSE_US: u16 = 2500;
pub const PERIOD_US: u16 = 20_000;
/// Slew rate of the applied pulse: 25 µs / 10 ms = the full 500–2500 travel
/// in 0.8 s, a little slower than an MG996R at 5 V (~0.2 s / 60° ≈ 33 µs /
/// 10 ms) so the pulse never runs ahead of the horn.
pub const SLEW_US_PER_10MS: u32 = 25;
/// How far to back away from a limit switch once it trips: ~4.5° of horn,
/// enough to unload the switch and the end stop. Tune on the real mechanism
/// (`MG996_SERVO_LIMIT_BACKOFF_US`).
pub const LIMIT_BACKOFF_US: u16 = 50;
/// A tick that arrives later than this (task starved, debugger halt) slews
/// one bounded step rather than teleporting the pulse.
const MAX_SLEW_STEP_MS: u32 = 100;
/// A longer pulse drives the mechanism towards the UP switch. Flip if the
/// horn is mounted the other way round (`MG996_SERVO_UP_IS_LONGER_PULSE`).
pub const UP_IS_LONGER_PULSE: bool = true;

pub fn clamp_pulse(pulse_us: u16) -> u16 {
    pulse_us.clamp(MIN_PULSE_US, MAX_PULSE_US)
}

/// Two-sample debounce for the limit switches: a reading has to repeat on
/// two consecutive samples before it becomes the debounced state. Seed it
/// with the first reading so a switch already pressed at boot counts at once.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LimitDebounce {
    raw: (bool, bool),
    stable: (bool, bool),
}

impl LimitDebounce {
    pub const fn seeded(up: bool, dn: bool) -> Self {
        Self { raw: (up, dn), stable: (up, dn) }
    }

    /// Feed one `(up, dn)` sample; returns the debounced `(up, dn)`.
    pub fn sample(&mut self, up: bool, dn: bool) -> (bool, bool) {
        if up == self.raw.0 {
            self.stable.0 = up;
        }
        if dn == self.raw.1 {
            self.stable.1 = dn;
        }
        self.raw = (up, dn);
        self.stable
    }

    pub fn pressed(&self) -> (bool, bool) {
        self.stable
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Servo {
    /// Pulse on the wire, slewing towards `target_us`.
    pub pulse_us: u16,
    /// Where the host wants the pulse to end up.
    pub target_us: u16,
    /// 0 = hold until the next command.
    pub hold_timeout_ms: u16,
    pub last_rx_seq: u8,
    enabled: bool,
    /// The last target was cut short by a switch; cleared by the next command.
    limit_clamped: bool,
    timed_out: bool,
    limit_up: bool,
    limit_dn: bool,
    /// `None` until the first command.
    received_at_ms: Option<u32>,
    /// Last tick that slewed the pulse; `None` while output is off.
    last_slew_ms: Option<u32>,
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
            target_us: CENTER_PULSE_US,
            hold_timeout_ms: 0,
            last_rx_seq: 0,
            enabled: false,
            limit_clamped: false,
            timed_out: false,
            limit_up: false,
            limit_dn: false,
            received_at_ms: None,
            last_slew_ms: None,
        }
    }

    /// A `0x07` arrived. `pulse_us == 0` releases the servo (no pulses).
    /// The applied pulse slews from where it is to the new target; from a
    /// released state it restarts at the last applied value (centre after
    /// power-on).
    pub fn command(&mut self, pulse_us: u16, hold_timeout_ms: u16, rx_seq: u8, now_ms: u32) {
        if pulse_us == 0 {
            self.disable();
            return;
        }
        self.target_us = clamp_pulse(pulse_us);
        self.hold_timeout_ms = hold_timeout_ms;
        self.last_rx_seq = rx_seq;
        self.received_at_ms = Some(now_ms);
        self.enabled = true;
        self.timed_out = false;
        self.limit_clamped = false;
        // Into a pressed switch: clamp now so the host sees it in the very
        // next status frame rather than after the first slew tick.
        self.apply_limits();
    }

    pub fn disable(&mut self) {
        self.enabled = false;
        self.last_slew_ms = None;
    }

    /// Debounced state of the two limit switches (see `LimitDebounce`).
    pub fn set_limits(&mut self, up_pressed: bool, dn_pressed: bool) {
        self.limit_up = up_pressed;
        self.limit_dn = dn_pressed;
    }

    /// Age the hold timeout and slew the pulse one step. Call periodically
    /// (the firmware does it on the 20 ms control tick).
    pub fn tick(&mut self, now_ms: u32) {
        if self.enabled && self.hold_timeout_ms != 0 && self.age_ms(now_ms) >= u32::from(self.hold_timeout_ms) {
            self.enabled = false;
            self.timed_out = true;
        }
        // Only move while pulses are going out: a released servo is not
        // where the pulse says, so slewing the number would be fiction.
        if !self.output_active() {
            self.last_slew_ms = None;
            return;
        }
        self.apply_limits();
        let elapsed = self.last_slew_ms.map_or(0, |t| now_ms.wrapping_sub(t)).min(MAX_SLEW_STEP_MS);
        self.last_slew_ms = Some(now_ms);
        let step = elapsed * SLEW_US_PER_10MS / 10;
        let step = u16::try_from(step).unwrap_or(u16::MAX);
        if self.pulse_us < self.target_us {
            self.pulse_us = self.pulse_us.saturating_add(step).min(self.target_us);
        } else if self.pulse_us > self.target_us {
            self.pulse_us = self.pulse_us.saturating_sub(step).max(self.target_us);
        }
    }

    /// Is a move from the current pulse to the target heading for the UP switch?
    fn moves_up(&self) -> bool {
        if UP_IS_LONGER_PULSE {
            self.target_us > self.pulse_us
        } else {
            self.target_us < self.pulse_us
        }
    }

    /// Target `LIMIT_BACKOFF_US` away from the UP (or DOWN) switch from the
    /// current pulse, clamped to the pulse range.
    fn backoff_target(&self, from_up: bool) -> u16 {
        if from_up == UP_IS_LONGER_PULSE {
            self.pulse_us.saturating_sub(LIMIT_BACKOFF_US).max(MIN_PULSE_US)
        } else {
            self.pulse_us.saturating_add(LIMIT_BACKOFF_US).min(MAX_PULSE_US)
        }
    }

    /// Refuse motion into a pressed switch: the target becomes a short
    /// back-off away from it, so the servo settles just clear of the end
    /// stop instead of leaning on it. Both switches pressed (travel shorter
    /// than the back-off, or a wiring fault) holds the current pulse rather
    /// than ping-ponging between the two back-offs.
    fn apply_limits(&mut self) {
        if self.target_us == self.pulse_us {
            return;
        }
        if self.limit_up && self.limit_dn {
            self.target_us = self.pulse_us;
            self.limit_clamped = true;
            return;
        }
        let up = self.moves_up();
        if (up && self.limit_up) || (!up && self.limit_dn) {
            self.target_us = self.backoff_target(up);
            self.limit_clamped = true;
        }
    }

    /// Whether the timer should be emitting pulses right now.
    pub fn output_active(&self) -> bool {
        self.enabled
    }

    fn age_ms(&self, now_ms: u32) -> u32 {
        self.received_at_ms.map_or(0, |t| now_ms.wrapping_sub(t))
    }

    pub fn flags(&self) -> u8 {
        (if self.enabled { servo_status_flag::ENABLED } else { 0 })
            | (if self.limit_clamped { servo_status_flag::LIMIT_ACTIVE } else { 0 })
            | (if self.output_active() { servo_status_flag::OUTPUT_ACTIVE } else { 0 })
            | (if self.timed_out { servo_status_flag::TIMED_OUT } else { 0 })
            | (if self.limit_up { servo_status_flag::LIMIT_UP } else { 0 })
            | (if self.limit_dn { servo_status_flag::LIMIT_DN } else { 0 })
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

    /// Drive `ms` of 10 ms ticks starting after `from_ms`.
    fn run(s: &mut Servo, from_ms: u32, ms: u32) -> u32 {
        let mut now = from_ms;
        for _ in 0..ms / 10 {
            now += 10;
            s.tick(now);
        }
        now
    }

    #[test]
    fn command_clamps_and_enables() {
        let mut s = Servo::new();
        s.command(100, 0, 7, 1000);
        assert_eq!(s.target_us, MIN_PULSE_US);
        assert!(s.output_active());
        s.command(9000, 0, 8, 1000);
        assert_eq!(s.target_us, MAX_PULSE_US);
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
        // the last pulse is kept for the status frame
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
    fn pulse_slews_towards_the_target() {
        let mut s = Servo::new();
        s.command(2500, 0, 1, 0);
        // the first tick only stamps the clock
        s.tick(10);
        assert_eq!(s.pulse_us, CENTER_PULSE_US);
        s.tick(20);
        assert_eq!(s.pulse_us, CENTER_PULSE_US + 25);
        s.tick(40);
        assert_eq!(s.pulse_us, CENTER_PULSE_US + 75);
        let now = run(&mut s, 40, 1000);
        assert_eq!(s.pulse_us, 2500);
        assert_eq!(s.flags(), servo_status_flag::ENABLED | servo_status_flag::OUTPUT_ACTIVE);
        // and back down, landing exactly on the target
        s.command(2490, 0, 2, now);
        run(&mut s, now, 30);
        assert_eq!(s.pulse_us, 2490);
    }

    #[test]
    fn late_tick_is_bounded() {
        let mut s = Servo::new();
        s.command(2500, 0, 1, 0);
        s.tick(10);
        s.tick(5010);
        assert_eq!(s.pulse_us, CENTER_PULSE_US + 250);
    }

    #[test]
    fn released_servo_does_not_slew() {
        let mut s = Servo::new();
        s.command(2500, 100, 1, 0);
        s.tick(10);
        s.tick(200); // hold timeout: pulses off, position unknown
        assert!(!s.output_active());
        assert_eq!(s.pulse_us, CENTER_PULSE_US);
        // re-enabling restarts the clock rather than jumping
        s.command(2500, 0, 2, 200);
        s.tick(210);
        assert_eq!(s.pulse_us, CENTER_PULSE_US);
    }

    #[test]
    fn up_switch_stops_upward_motion_and_backs_off() {
        let mut s = Servo::new();
        s.command(2500, 0, 1, 0);
        let now = run(&mut s, 0, 200);
        let here = s.pulse_us;
        assert!(here > CENTER_PULSE_US && here < 2500);
        s.set_limits(true, false);
        s.tick(now + 10);
        // the tick that sees the switch already starts backing off
        assert_eq!(s.target_us, here - LIMIT_BACKOFF_US);
        assert_eq!(s.pulse_us, here - 25);
        assert_eq!(
            s.flags(),
            servo_status_flag::ENABLED
                | servo_status_flag::OUTPUT_ACTIVE
                | servo_status_flag::LIMIT_ACTIVE
                | servo_status_flag::LIMIT_UP
        );
        // the switch opens as it backs off; it still settles on the back-off
        s.set_limits(false, false);
        let now = run(&mut s, now + 10, 100);
        assert_eq!(s.pulse_us, here - LIMIT_BACKOFF_US);
        assert!(s.flags() & servo_status_flag::LIMIT_ACTIVE != 0);
        // moving away from the switch is fine and clears LIMIT_ACTIVE
        s.command(1000, 0, 3, now + 10);
        assert_eq!(s.target_us, 1000);
        assert!(s.flags() & servo_status_flag::LIMIT_ACTIVE == 0);
    }

    #[test]
    fn command_into_a_pressed_switch_is_refused() {
        let mut s = Servo::new();
        s.command(2000, 0, 1, 0);
        run(&mut s, 0, 1000);
        assert_eq!(s.pulse_us, 2000);
        s.set_limits(true, false);
        s.command(2500, 0, 2, 2000);
        assert_eq!(s.target_us, 2000 - LIMIT_BACKOFF_US);
        assert!(s.flags() & servo_status_flag::LIMIT_ACTIVE != 0);
    }

    #[test]
    fn down_switch_pressed_at_boot_blocks_only_downward() {
        let mut s = Servo::new();
        s.set_limits(false, true);
        s.command(500, 0, 1, 0);
        assert_eq!(s.target_us, CENTER_PULSE_US + LIMIT_BACKOFF_US);
        assert_eq!(s.flags() & servo_status_flag::LIMIT_DN, servo_status_flag::LIMIT_DN);
        s.command(2000, 0, 2, 10);
        assert_eq!(s.target_us, 2000);
    }

    #[test]
    fn backoff_is_clamped_to_the_range() {
        let mut s = Servo::new();
        s.command(500, 0, 1, 0);
        run(&mut s, 0, 1000);
        assert_eq!(s.pulse_us, 500);
        // UP pressed at the bottom of the range: nowhere to back off to
        s.set_limits(true, false);
        s.command(2500, 0, 2, 2000);
        assert_eq!(s.target_us, 500);
        assert!(s.flags() & servo_status_flag::LIMIT_ACTIVE != 0);
    }

    #[test]
    fn both_switches_pressed_holds_without_ping_pong() {
        let mut s = Servo::new();
        s.command(2000, 0, 1, 0);
        run(&mut s, 0, 1000);
        s.set_limits(true, true);
        s.command(2500, 0, 2, 2000);
        assert_eq!(s.target_us, 2000);
        run(&mut s, 2000, 100);
        assert_eq!(s.pulse_us, 2000);
        s.command(500, 0, 3, 2200);
        assert_eq!(s.target_us, 2000);
        assert!(s.flags() & servo_status_flag::LIMIT_ACTIVE != 0);
    }

    #[test]
    fn debounce_needs_two_matching_samples() {
        let mut d = LimitDebounce::seeded(false, false);
        assert_eq!(d.sample(true, false), (false, false));
        assert_eq!(d.sample(true, false), (true, false));
        // a one-sample glitch does not open it again
        assert_eq!(d.sample(false, false), (true, false));
        assert_eq!(d.sample(true, false), (true, false));
        assert_eq!(d.sample(false, true), (true, false));
        assert_eq!(d.sample(false, true), (false, true));
        assert_eq!(LimitDebounce::seeded(true, true).pressed(), (true, true));
    }
}
