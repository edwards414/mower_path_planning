//! Host commands with a dead-man timeout.
//!
//! Both the wheel command (`Motor_SetOpenLoopCommand`) and the blade command
//! (`lawer_set_open_loop_command`) are "the last value the host sent, valid
//! until `timeout_ms` after it arrived". This type captures that once, so
//! the firmware does not carry two copies of the same timeout arithmetic.

pub const PERMILLE_LIMIT: i16 = 1000;
pub const DEFAULT_TIMEOUT_MS: u16 = 200;
pub const PWM_MAX_COUNTS: i16 = 200;

/// Clamp a host command to ±1000 ‰.
pub fn clamp_permille(value: i16) -> i16 {
    value.clamp(-PERMILLE_LIMIT, PERMILLE_LIMIT)
}

/// Open-loop mapping ‰ → PWM counts (±200).
pub fn permille_to_pwm(command_permille: i16) -> i16 {
    (i32::from(clamp_permille(command_permille)) * i32::from(PWM_MAX_COUNTS) / i32::from(PERMILLE_LIMIT)) as i16
}

/// A value the host must keep refreshing.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Timed<T: Copy> {
    pub value: T,
    pub timeout_ms: u16,
    pub last_rx_seq: u8,
    /// `None` until the first command arrives.
    pub received_at_ms: Option<u32>,
}

/// What the control loop sees after evaluating a [`Timed`] against `now`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Evaluated<T: Copy> {
    /// The commanded value, or `T::default()` if nothing valid was received.
    pub value: T,
    pub valid: bool,
    pub timed_out: bool,
    /// `u32::MAX` when never received (matches the C++ `0xFFFFFFFF`).
    pub age_ms: u32,
    pub last_rx_seq: u8,
}

impl<T: ConstDefault> Timed<T> {
    /// `const` so the firmware can keep one in a `static`.
    pub const fn new() -> Self {
        Self { value: T::DEFAULT, timeout_ms: DEFAULT_TIMEOUT_MS, last_rx_seq: 0, received_at_ms: None }
    }
}

impl<T: Copy + Default> Timed<T> {
    /// Store a fresh command. A zero timeout means "use the default".
    pub fn set(&mut self, value: T, timeout_ms: u16, rx_seq: u8, now_ms: u32) {
        self.value = value;
        self.timeout_ms = if timeout_ms == 0 { DEFAULT_TIMEOUT_MS } else { timeout_ms };
        self.last_rx_seq = rx_seq;
        self.received_at_ms = Some(now_ms);
    }

    pub fn evaluate(&self, now_ms: u32) -> Evaluated<T> {
        match self.received_at_ms {
            Some(t) => {
                let age_ms = now_ms.wrapping_sub(t);
                let timed_out = age_ms > u32::from(self.timeout_ms);
                Evaluated { value: self.value, valid: true, timed_out, age_ms, last_rx_seq: self.last_rx_seq }
            }
            None => Evaluated {
                value: T::default(),
                valid: false,
                timed_out: true,
                age_ms: u32::MAX,
                last_rx_seq: self.last_rx_seq,
            },
        }
    }
}

/// `Default::default()` is not `const`, so the plain-data command types
/// provide their zero value here for use in `Timed::new()`.
pub trait ConstDefault: Copy + Default {
    const DEFAULT: Self;
}

impl ConstDefault for i16 {
    const DEFAULT: Self = 0;
}

impl ConstDefault for (i16, i16) {
    const DEFAULT: Self = (0, 0);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn never_received_is_timed_out() {
        let t: Timed<(i16, i16)> = Timed::new();
        let e = t.evaluate(1234);
        assert!(!e.valid);
        assert!(e.timed_out);
        assert_eq!(e.age_ms, u32::MAX);
        assert_eq!(e.value, (0, 0));
    }

    #[test]
    fn times_out_after_timeout_ms() {
        let mut t: Timed<(i16, i16)> = Timed::new();
        t.set((100, -100), 300, 9, 1000);
        let e = t.evaluate(1300);
        assert!(e.valid && !e.timed_out);
        assert_eq!(e.age_ms, 300);
        assert_eq!(e.last_rx_seq, 9);
        assert!(t.evaluate(1301).timed_out);
    }

    #[test]
    fn zero_timeout_uses_default() {
        let mut t: Timed<i16> = Timed::new();
        t.set(5, 0, 0, 0);
        assert_eq!(t.timeout_ms, DEFAULT_TIMEOUT_MS);
    }

    #[test]
    fn tick_wraparound_is_handled() {
        let mut t: Timed<i16> = Timed::new();
        t.set(5, 200, 0, u32::MAX - 10);
        assert_eq!(t.evaluate(20).age_ms, 31);
    }

    #[test]
    fn permille_mapping() {
        assert_eq!(permille_to_pwm(1000), 200);
        assert_eq!(permille_to_pwm(-1000), -200);
        assert_eq!(permille_to_pwm(2000), 200);
        assert_eq!(permille_to_pwm(500), 100);
        assert_eq!(permille_to_pwm(1), 0);
    }
}
