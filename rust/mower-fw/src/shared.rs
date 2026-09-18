//! State shared between the UART RX task, the motor task and the TX task.
//!
//! This replaces every `volatile` global + `__disable_irq()` pair in the
//! C++ modules. The rule the compiler now enforces: nothing here can be
//! read or written outside `lock(|s| ...)`, and the closure cannot `await`,
//! so the critical section is always short and always balanced.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use mower_core::charger;
use mower_core::command::Timed;
use mower_core::protocol::{self, FrameBytes, LawerMotorStatus, MotorStatus, PidConfigCommand};
use mower_core::servo::Servo;
use mower_core::settings::{ControllerSettings, StorageStatus};
use mower_core::wheel;

/// Commands written by the RX task, read by the motor task.
pub struct Commands {
    /// (left, right) wheel command in ‰.
    pub wheels: Timed<(i16, i16)>,
    /// Blade command in ‰.
    pub blade: Timed<i16>,
    /// Sequence number of the last accepted 0x05 power frame.
    pub power_last_rx_seq: u8,
    /// MG996 target + hold timeout, applied by the motor task each tick.
    pub servo: Servo,
}

/// Status written by the motor task, read wherever a status frame is built.
pub struct Telemetry {
    pub motor: MotorStatus,
    pub wheels: wheel::Status,
    pub blade: LawerMotorStatus,
    pub settings: ControllerSettings,
    pub storage: StorageStatus,
    pub pid_last_rx_seq: u8,
    pub pid_last_apply_ok: bool,
}

pub static COMMANDS: Mutex<CriticalSectionRawMutex, RefCell<Commands>> = Mutex::new(RefCell::new(Commands {
    wheels: Timed::new(),
    blade: Timed::new(),
    power_last_rx_seq: 0,
    servo: Servo::new(),
}));

pub static TELEMETRY: Mutex<CriticalSectionRawMutex, RefCell<Telemetry>> = Mutex::new(RefCell::new(Telemetry {
    motor: MotorStatus {
        commanded_left_permille: 0,
        commanded_right_permille: 0,
        applied_left_pwm: 0,
        applied_right_pwm: 0,
        command_age_ms: 0,
        flags: protocol::motor_status_flag::COMMAND_TIMEOUT,
        last_rx_seq: 0,
    },
    wheels: wheel::Status::ZERO,
    blade: LawerMotorStatus { commanded_permille: 0, applied_pwm: 0, command_age_ms: 0, flags: 0, last_rx_seq: 0 },
    settings: ControllerSettings::DEFAULT,
    storage: StorageStatus { flash_valid: false, last_save_ok: false, sequence: 0 },
    pid_last_rx_seq: 0,
    pid_last_apply_ok: true,
}));

/// Last RS485 charger poll, written by the charger task, read for 0x87.
pub static CHARGER: Mutex<CriticalSectionRawMutex, RefCell<charger::Snapshot>> =
    Mutex::new(RefCell::new(charger::Snapshot::ZERO));

/// A PID configuration request. The motor task owns the controller and the
/// flash, so the RX task hands the request over instead of applying it.
/// `Signal` keeps only the latest request, which is the C++ behaviour too.
pub static PID_CONFIG_REQUEST: Signal<CriticalSectionRawMutex, (PidConfigCommand, u8)> = Signal::new();

/// Outgoing UART traffic, consumed by the TX task.
pub enum TxItem {
    Frame(FrameBytes),
    /// Send the 0x8F ack for `seq`, then reset into the bootloader.
    EnterBootloader {
        seq: u8,
    },
}

pub const TX_QUEUE_LEN: usize = 16;
pub static TX_QUEUE: Channel<CriticalSectionRawMutex, TxItem, TX_QUEUE_LEN> = Channel::new();

/// Queue a frame for transmission; dropped if the queue is full, like the
/// C++ `osMessageQueuePut(..., 0)`.
pub fn send_frame(frame: FrameBytes) {
    let _ = TX_QUEUE.try_send(TxItem::Frame(frame));
}

pub fn with_commands<R>(f: impl FnOnce(&mut Commands) -> R) -> R {
    COMMANDS.lock(|c| f(&mut c.borrow_mut()))
}

pub fn with_telemetry<R>(f: impl FnOnce(&mut Telemetry) -> R) -> R {
    TELEMETRY.lock(|t| f(&mut t.borrow_mut()))
}

pub fn with_charger<R>(f: impl FnOnce(&mut charger::Snapshot) -> R) -> R {
    CHARGER.lock(|c| f(&mut c.borrow_mut()))
}
