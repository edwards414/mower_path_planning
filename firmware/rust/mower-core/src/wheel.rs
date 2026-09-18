//! Closed/open-loop wheel control (port of `wheel_controller.cpp`).
//!
//! The controller is pure: it takes raw encoder counter values and returns
//! PWM counts. The firmware owns the timers and calls [`WheelController::update`]
//! every [`CONTROL_PERIOD_MS`].

use crate::command::{clamp_permille, permille_to_pwm, PERMILLE_LIMIT, PWM_MAX_COUNTS};
use crate::pid::Pid;
use crate::settings::{ControllerSettings, StorageStatus};

pub const CONTROL_PERIOD_MS: u32 = 20;
pub const ENCODER_COUNTS_PER_REV: f32 = 8896.0;
/// Encoder count direction relative to a positive (vehicle-forward) command.
/// Measured 2026-09-11, see `wheel_controller.hpp`.
pub const LEFT_ENCODER_SIGN: i32 = 1;
pub const RIGHT_ENCODER_SIGN: i32 = -1;
pub const COMMAND_DEADBAND_PERMILLE: i16 = 5;

pub mod flag {
    pub const ENABLED: u8 = 0x01;
    pub const CLOSED_LOOP: u8 = 0x02;
    pub const FLASH_SETTINGS_VALID: u8 = 0x04;
    pub const LAST_SAVE_OK: u8 = 0x08;
}

const DT_S: f32 = CONTROL_PERIOD_MS as f32 / 1000.0;

/// Width of the hardware counter behind an encoder. The C++ firmware ran
/// TIM5 (left) as a 32-bit counter and TIM1 (right) as 16-bit; embassy's
/// `Qei` reads 16 bits on both, which is plenty at < 200 counts per tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CounterWidth {
    Bits16,
    Bits32,
}

impl CounterWidth {
    /// Signed change between two raw counter reads, handling wrap-around.
    pub fn delta(self, current: u32, previous: u32) -> i32 {
        match self {
            CounterWidth::Bits16 => i32::from((current as u16).wrapping_sub(previous as u16) as i16),
            CounterWidth::Bits32 => current.wrapping_sub(previous) as i32,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WheelStatus {
    pub target_rpm: f32,
    pub measured_rpm: f32,
    pub pid_output: f32,
    pub applied_pwm: i16,
    /// counts in the last tick (sign-corrected)
    pub delta_counts: i32,
    /// accumulated since boot, sign-corrected; wraps at i32
    pub total_counts: i32,
    pub raw_counter: u32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Status {
    pub left: WheelStatus,
    pub right: WheelStatus,
    pub flags: u8,
}

impl Status {
    pub const ZERO: Self = Self { left: WheelStatus::ZERO, right: WheelStatus::ZERO, flags: 0 };
}

impl WheelStatus {
    pub const ZERO: Self = Self {
        target_rpm: 0.0,
        measured_rpm: 0.0,
        pid_output: 0.0,
        applied_pwm: 0,
        delta_counts: 0,
        total_counts: 0,
        raw_counter: 0,
    };
}

/// PWM counts to apply, already sign-corrected per wheel by the caller.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Output {
    pub left_pwm: i16,
    pub right_pwm: i16,
}

struct Wheel {
    pid: Pid,
    width: CounterWidth,
    sign: i32,
    previous_counter: u32,
    total_counts: i32,
    status: WheelStatus,
}

impl Wheel {
    fn new(pid: Pid, width: CounterWidth, sign: i32, initial_counter: u32) -> Self {
        Self { pid, width, sign, previous_counter: initial_counter, total_counts: 0, status: WheelStatus::default() }
    }

    fn update(
        &mut self,
        counter: u32,
        command_permille: i16,
        output_enabled: bool,
        settings: &ControllerSettings,
    ) -> i16 {
        let delta = self.sign * self.width.delta(counter, self.previous_counter);
        self.previous_counter = counter;
        self.total_counts = self.total_counts.wrapping_add(delta);

        let measured_rpm = counts_to_rpm(delta);
        let target_rpm = permille_to_rpm(command_permille, settings.wheel_max_rpm);

        let (pwm, pid_output) = if !output_enabled {
            self.pid.reset();
            (0, 0.0)
        } else if settings.closed_loop() {
            if command_is_zero(command_permille) {
                self.pid.reset();
                (0, 0.0)
            } else {
                let out = self.pid.update(target_rpm, measured_rpm, DT_S);
                (clamp_pwm(out), out)
            }
        } else {
            let pwm = permille_to_pwm(command_permille);
            (pwm, f32::from(pwm))
        };

        self.status = WheelStatus {
            target_rpm,
            measured_rpm,
            pid_output,
            applied_pwm: pwm,
            delta_counts: delta,
            total_counts: self.total_counts,
            raw_counter: counter,
        };
        pwm
    }

    fn stop(&mut self) {
        self.pid.reset();
        self.status.applied_pwm = 0;
        self.status.pid_output = 0.0;
    }
}

pub struct WheelController {
    left: Wheel,
    right: Wheel,
    settings: ControllerSettings,
    flags: u8,
}

impl WheelController {
    /// `left` / `right` give each encoder counter's width and current value,
    /// so the first tick does not see a bogus jump.
    pub fn new(settings: ControllerSettings, left: (CounterWidth, u32), right: (CounterWidth, u32)) -> Self {
        let mut c = Self {
            left: Wheel::new(Pid::new(settings.left_wheel_pid), left.0, LEFT_ENCODER_SIGN, left.1),
            right: Wheel::new(Pid::new(settings.right_wheel_pid), right.0, RIGHT_ENCODER_SIGN, right.1),
            settings,
            flags: 0,
        };
        c.stop();
        c
    }

    pub fn settings(&self) -> &ControllerSettings {
        &self.settings
    }

    /// Runtime settings changed (PID gains, closed-loop flag). Resets the
    /// integrators like `WheelController_ApplySettings` did.
    pub fn apply_settings(&mut self, settings: ControllerSettings) {
        self.settings = settings;
        self.left.pid.configure(settings.left_wheel_pid);
        self.right.pid.configure(settings.right_wheel_pid);
        self.left.pid.reset();
        self.right.pid.reset();
    }

    /// One 20 ms control tick.
    pub fn update(
        &mut self,
        left_counter: u32,
        right_counter: u32,
        left_command_permille: i16,
        right_command_permille: i16,
        output_enabled: bool,
        storage: StorageStatus,
    ) -> Output {
        let left_pwm = self.left.update(left_counter, left_command_permille, output_enabled, &self.settings);
        let right_pwm = self.right.update(right_counter, right_command_permille, output_enabled, &self.settings);
        self.update_flags(output_enabled, storage);
        Output { left_pwm, right_pwm }
    }

    pub fn stop(&mut self) {
        self.left.stop();
        self.right.stop();
        self.update_flags(false, StorageStatus::default());
    }

    pub fn status(&self) -> Status {
        Status { left: self.left.status, right: self.right.status, flags: self.flags }
    }

    fn update_flags(&mut self, output_enabled: bool, storage: StorageStatus) {
        let mut flags = 0;
        if output_enabled {
            flags |= flag::ENABLED;
        }
        if self.settings.closed_loop() {
            flags |= flag::CLOSED_LOOP;
        }
        if storage.flash_valid {
            flags |= flag::FLASH_SETTINGS_VALID;
        }
        if storage.last_save_ok {
            flags |= flag::LAST_SAVE_OK;
        }
        self.flags = flags;
    }
}

fn clamp_pwm(pwm: f32) -> i16 {
    // `as` saturates, so no explicit range check is needed before the cast.
    (pwm as i16).clamp(-PWM_MAX_COUNTS, PWM_MAX_COUNTS)
}

fn permille_to_rpm(command_permille: i16, wheel_max_rpm: f32) -> f32 {
    f32::from(clamp_permille(command_permille)) * wheel_max_rpm / f32::from(PERMILLE_LIMIT)
}

fn counts_to_rpm(delta_counts: i32) -> f32 {
    (delta_counts as f32 * 60.0) / (ENCODER_COUNTS_PER_REV * DT_S)
}

fn command_is_zero(command_permille: i16) -> bool {
    (-COMMAND_DEADBAND_PERMILLE..=COMMAND_DEADBAND_PERMILLE).contains(&command_permille)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn open_loop() -> ControllerSettings {
        ControllerSettings { closed_loop_enabled: 0, ..ControllerSettings::default() }
    }

    #[test]
    fn counter_deltas_handle_wrap() {
        assert_eq!(CounterWidth::Bits16.delta(5, 0xFFFB), 10);
        assert_eq!(CounterWidth::Bits16.delta(0xFFFB, 5), -10);
        assert_eq!(CounterWidth::Bits32.delta(3, u32::MAX - 1), 5);
        assert_eq!(CounterWidth::Bits32.delta(u32::MAX - 1, 3), -5);
    }

    #[test]
    fn open_loop_maps_permille_directly() {
        let mut c = WheelController::new(open_loop(), (CounterWidth::Bits32, 0), (CounterWidth::Bits16, 0));
        let out = c.update(0, 0, 500, -1000, true, StorageStatus::default());
        assert_eq!(out, Output { left_pwm: 100, right_pwm: -200 });
        assert_eq!(c.status().flags, flag::ENABLED);
    }

    #[test]
    fn disabled_output_is_zero_and_resets_pid() {
        let mut c =
            WheelController::new(ControllerSettings::default(), (CounterWidth::Bits32, 0), (CounterWidth::Bits16, 0));
        assert_eq!(c.update(0, 0, 1000, 1000, false, StorageStatus::default()), Output::default());
        assert_eq!(c.status().flags & flag::ENABLED, 0);
    }

    #[test]
    fn closed_loop_drives_towards_target() {
        let mut c =
            WheelController::new(ControllerSettings::default(), (CounterWidth::Bits32, 0), (CounterWidth::Bits16, 0));
        // wheels stationary, forward command → positive PWM on both
        let out = c.update(0, 0, 1000, 1000, true, StorageStatus::default());
        assert!(out.left_pwm > 0 && out.right_pwm > 0);
        let s = c.status();
        assert_eq!(s.left.target_rpm, 58.0);
        assert_eq!(s.left.measured_rpm, 0.0);
        assert_eq!(s.flags, flag::ENABLED | flag::CLOSED_LOOP);
    }

    #[test]
    fn encoder_signs_and_rpm() {
        let mut c = WheelController::new(
            ControllerSettings::default(),
            (CounterWidth::Bits32, 1000),
            (CounterWidth::Bits16, 1000),
        );
        // One revolution per second = 8896 counts/s = 177.92 counts / 20 ms.
        // Left counts up (sign +1), right counts *down* for forward (sign -1).
        c.update(1000 + 178, 1000 - 178, 0, 0, true, StorageStatus::default());
        let s = c.status();
        assert_eq!(s.left.delta_counts, 178);
        assert_eq!(s.right.delta_counts, 178);
        assert!((s.left.measured_rpm - 60.0).abs() < 0.1);
        assert_eq!(s.left.total_counts, 178);
        assert_eq!(s.right.raw_counter, 1000 - 178);
    }

    #[test]
    fn deadband_holds_pwm_at_zero_in_closed_loop() {
        let mut c =
            WheelController::new(ControllerSettings::default(), (CounterWidth::Bits32, 0), (CounterWidth::Bits16, 0));
        assert_eq!(c.update(0, 0, 5, -5, true, StorageStatus::default()), Output::default());
    }

    #[test]
    fn storage_flags_are_reported() {
        let mut c = WheelController::new(open_loop(), (CounterWidth::Bits32, 0), (CounterWidth::Bits16, 0));
        c.update(0, 0, 0, 0, true, StorageStatus { flash_valid: true, last_save_ok: true, sequence: 3 });
        assert_eq!(c.status().flags, flag::ENABLED | flag::FLASH_SETTINGS_VALID | flag::LAST_SAVE_OK);
    }
}
