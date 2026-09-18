//! Mower robot firmware — Rust/Embassy port of the UART + motor slice.
//!
//! Four tasks on one thread-mode executor:
//!
//! * `uart_rx_task` — DMA ring buffer → frame parser → shared command state
//! * `motor_task` — fixed 20 ms tick: encoders → PID → PWM, blade timeout,
//!   servo hold timeout, PID-config/flash requests, 50 ms status burst
//! * `uart_tx_task` — drains the TX queue over DMA; performs the bootloader
//!   hand-off
//! * `charger_task` — polls the RS485 charger every 500 ms (`charger.rs`)
//!
//! The MG996 pulse itself comes from the TIM10 interrupt (`servo.rs`).
//!
//! Not ported yet (the C++ firmware still owns them): power button / shutdown
//! state machine, WS2812 light shows, buzzer, ADC monitor (so the servo
//! current limit never trips here). See `rust/README.md`.

#![no_std]
#![no_main]

mod board;
mod charger;
mod fault;
mod servo;
mod shared;

use embassy_executor::Spawner;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{self, RingBufferedUartRx, Uart, UartTx};
use embassy_stm32::{bind_interrupts, dma, peripherals};
use embassy_time::{Duration, Instant, Ticker};
use mower_core::command::{clamp_permille, permille_to_pwm};
use mower_core::protocol::{
    self, encode_payload, f32_to_i16, f32_to_i16_x100, frame_type, motor_status_flag, pid_status_flag, saturating_u16,
    BootEnterAck, Frame, LawerMotorStatus, MotorStatus, PidConfigStatus, PowerStatus, Received, WheelFeedbackStatus,
    BOOT_REQUEST_MAGIC,
};
use mower_core::settings::{ControllerSettings, Record, StorageStatus};
use mower_core::wheel::{CounterWidth, WheelController};
use shared::{send_frame, with_charger, with_commands, with_telemetry, TxItem, PID_CONFIG_REQUEST, TX_QUEUE};
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    DMA2_STREAM2 => dma::InterruptHandler<peripherals::DMA2_CH2>;
    DMA2_STREAM7 => dma::InterruptHandler<peripherals::DMA2_CH7>;
    USART6 => usart::InterruptHandler<peripherals::USART6>;
    DMA2_STREAM1 => dma::InterruptHandler<peripherals::DMA2_CH1>;
    DMA2_STREAM6 => dma::InterruptHandler<peripherals::DMA2_CH6>;
});

/// Start of this image (see `memory.x`); the bootloader also sets VTOR
/// before jumping, this just makes the image self-sufficient.
const APP_VECTOR_TABLE: u32 = 0x0800_8000;
/// Reset-surviving mailbox shared with the bootloader (`boot_shared.h`).
const BOOT_SHARED_MAGIC_PTR: *mut u32 = 0x2000_0000 as *mut u32;

const UART_RX_DMA_BUF_SIZE: usize = 256;
const CONTROL_PERIOD: Duration = Duration::from_millis(mower_core::wheel::CONTROL_PERIOD_MS as u64);
const STATUS_PERIOD: Duration = Duration::from_millis(protocol::STATUS_PERIOD_MS);

/// Bootloader hand-off request from the RX task to the motor task.
static BOOT_REQUEST: embassy_sync::signal::Signal<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, u8> =
    embassy_sync::signal::Signal::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // SAFETY: VTOR is written once, before any interrupt is enabled, with the
    // address of this image's vector table.
    unsafe { (*cortex_m::peripheral::SCB::PTR).vtor.write(APP_VECTOR_TABLE) };

    let p = embassy_stm32::init(board::clock_config());
    let board = board::init(p);

    let mut uart_config = usart::Config::default();
    uart_config.baudrate = 115_200;
    let uart = Uart::new(
        board.usart1,
        board.uart_rx_pin,
        board.uart_tx_pin,
        board.uart_tx_dma,
        board.uart_rx_dma,
        Irqs,
        uart_config,
    )
    .expect("USART1 config");
    let (tx, rx) = uart.split();

    static RX_DMA_BUF: StaticCell<[u8; UART_RX_DMA_BUF_SIZE]> = StaticCell::new();
    let rx = rx.into_ring_buffered(RX_DMA_BUF.init([0; UART_RX_DMA_BUF_SIZE]));

    // Dropping an `Output` releases the pin; the rail-hold pins must live
    // for the whole run, so hand them to a static instead.
    static POWER_PINS: StaticCell<board::PowerPins> = StaticCell::new();
    POWER_PINS.init(board.power);

    let charger_uart = Uart::new(
        board.charger.usart6,
        board.charger.rx_pin,
        board.charger.tx_pin,
        board.charger.tx_dma,
        board.charger.rx_dma,
        Irqs,
        charger::ChargerLink::uart_config(),
    )
    .expect("USART6 config");
    let charger_link = charger::ChargerLink::new(charger_uart, board.charger.de);

    let servo_drive = servo::ServoDrive::new(board.servo);

    // Each task has a pool size of 1 and is spawned once, so these cannot fail.
    spawner.spawn(uart_rx_task(rx).expect("rx task"));
    spawner.spawn(uart_tx_task(tx).expect("tx task"));
    spawner.spawn(motor_task(board.motor, servo_drive).expect("motor task"));
    spawner.spawn(charger::charger_task(charger_link).expect("charger task"));
}

fn now_ms() -> u32 {
    Instant::now().as_millis() as u32
}

// ---------------------------------------------------------------------------
// UART RX
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn uart_rx_task(mut rx: RingBufferedUartRx<'static>) {
    let mut parser = protocol::Parser::new();
    let mut chunk = [0u8; 64];
    loop {
        match rx.read(&mut chunk).await {
            Ok(n) => {
                for &byte in &chunk[..n] {
                    if let Some(received) = parser.push(byte) {
                        handle_frame(received);
                    }
                }
            }
            // Overrun / framing / noise: the driver has stopped DMA; the next
            // `read` restarts it. Whatever was in flight is unrecoverable, so
            // resync the parser rather than risk gluing two frames together.
            Err(_) => parser.reset(),
        }
    }
}

fn handle_frame(Received { seq, frame }: Received) {
    let now = now_ms();
    match frame {
        Frame::MotorOpenLoop(cmd) => with_commands(|c| {
            let value = (clamp_permille(cmd.left_command_permille), clamp_permille(cmd.right_command_permille));
            c.wheels.set(value, cmd.command_timeout_ms, seq, now);
        }),
        Frame::LawerMotor(cmd) => with_commands(|c| {
            c.blade.set(clamp_permille(cmd.command_permille), cmd.command_timeout_ms, seq, now);
        }),
        // LEDs are not ported; the command is accepted and ignored.
        Frame::Ws2812(_) => {}
        Frame::PidConfig(cmd) => PID_CONFIG_REQUEST.signal((cmd, seq)),
        Frame::Power(_) => {
            // Power manager not ported: record the seq and answer so the
            // host's request/reply logic keeps working.
            with_commands(|c| c.power_last_rx_seq = seq);
            if let Some(f) = power_status_frame() {
                send_frame(f);
            }
        }
        Frame::Servo(cmd) => with_commands(|c| c.servo.command(cmd.pulse_us, cmd.hold_timeout_ms, seq, now)),
        Frame::EnterBootloader(_) => BOOT_REQUEST.signal(seq),
    }
}

// ---------------------------------------------------------------------------
// Motor / control loop
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn motor_task(mut hw: board::MotorPeripherals, mut servo_drive: servo::ServoDrive) {
    let (settings, mut storage) = match hw.settings_flash.load() {
        Some(record) => (
            record.settings.sanitized(),
            StorageStatus { flash_valid: true, last_save_ok: true, sequence: record.sequence },
        ),
        None => (ControllerSettings::default(), StorageStatus::default()),
    };
    with_telemetry(|t| {
        t.settings = settings;
        t.storage = storage;
    });

    let (left0, right0) = hw.encoders.read();
    let mut controller = WheelController::new(settings, (CounterWidth::Bits16, left0), (CounterWidth::Bits16, right0));

    hw.wheels.set(0, 0);
    hw.wheels.set_enabled(true);
    hw.blade.stop();

    // `Ticker` schedules from the previous deadline, so the loop does not
    // drift the way `osDelay(20)` after a variable amount of work did.
    let mut ticker = Ticker::every(CONTROL_PERIOD);
    let mut last_status = Instant::now();

    loop {
        ticker.next().await;
        let now = now_ms();

        if let Some((cmd, seq)) = PID_CONFIG_REQUEST.try_take() {
            let mut s = *controller.settings();
            s.left_wheel_pid.kp = cmd.left_kp;
            s.left_wheel_pid.ki = cmd.left_ki;
            s.left_wheel_pid.kd = cmd.left_kd;
            s.right_wheel_pid.kp = cmd.right_kp;
            s.right_wheel_pid.ki = cmd.right_ki;
            s.right_wheel_pid.kd = cmd.right_kd;
            s.closed_loop_enabled = u32::from(cmd.closed_loop_enabled != 0);
            let s = s.sanitized();
            controller.apply_settings(s);

            let mut ok = true;
            if cmd.persist_to_flash != 0 {
                let record = Record::new(s, storage.sequence.wrapping_add(1));
                ok = hw.settings_flash.store(&record);
                storage.last_save_ok = ok;
                if ok {
                    storage.flash_valid = true;
                    storage.sequence = record.sequence;
                }
            }
            with_telemetry(|t| {
                t.settings = s;
                t.storage = storage;
                t.pid_last_rx_seq = seq;
                t.pid_last_apply_ok = ok;
            });
        }

        if let Some(seq) = BOOT_REQUEST.try_take() {
            hold_for_bootloader(hw, servo_drive, seq).await;
        }

        let (wheel_cmd, blade_cmd, servo_cmd) = with_commands(|c| {
            c.servo.tick(now);
            (c.wheels.evaluate(now), c.blade.evaluate(now), c.servo)
        });
        // The ADC monitor is not ported, so the current limit never trips;
        // the pulse follows the command and its hold timeout only.
        servo_drive.apply(&servo_cmd);
        let alarm = hw.alarms.any_active();
        // MOTOR_ALARM_DISABLES_OUTPUT is 0 in the C++ build: IS pins are
        // advisory (they trip on inrush), so only the timeout gates output.
        let output_enabled = !wheel_cmd.timed_out;

        let (left_cmd, right_cmd) = wheel_cmd.value;
        let (left_counter, right_counter) = hw.encoders.read();
        let out = controller.update(left_counter, right_counter, left_cmd, right_cmd, output_enabled, storage);
        hw.wheels.set(out.left_pwm, out.right_pwm);
        let wheels = controller.status();

        // Blade: single direction, negative commands refused (reported as 0).
        let blade_pwm = if blade_cmd.timed_out { 0 } else { permille_to_pwm(blade_cmd.value).max(0) };
        hw.blade.set(blade_pwm as u16, 1);

        let motor = MotorStatus {
            commanded_left_permille: if wheel_cmd.valid { left_cmd } else { 0 },
            commanded_right_permille: if wheel_cmd.valid { right_cmd } else { 0 },
            applied_left_pwm: out.left_pwm,
            applied_right_pwm: out.right_pwm,
            command_age_ms: saturating_u16(wheel_cmd.age_ms),
            flags: (if wheel_cmd.valid { motor_status_flag::COMMAND_VALID } else { 0 })
                | (if wheel_cmd.timed_out { motor_status_flag::COMMAND_TIMEOUT } else { 0 })
                | (if alarm { motor_status_flag::DRIVER_ALARM } else { 0 }),
            last_rx_seq: wheel_cmd.last_rx_seq,
        };
        let blade = LawerMotorStatus {
            commanded_permille: if blade_cmd.valid { blade_cmd.value } else { 0 },
            applied_pwm: blade_pwm,
            command_age_ms: saturating_u16(blade_cmd.age_ms),
            flags: (if blade_cmd.valid { motor_status_flag::COMMAND_VALID } else { 0 })
                | (if blade_cmd.timed_out { motor_status_flag::COMMAND_TIMEOUT } else { 0 }),
            last_rx_seq: blade_cmd.last_rx_seq,
        };
        with_telemetry(|t| {
            t.motor = motor;
            t.wheels = wheels;
            t.blade = blade;
        });

        if last_status.elapsed() >= STATUS_PERIOD {
            send_status_burst();
            last_status = Instant::now();
        }
    }
}

/// Everything off, hand the ack + reset over to the TX task, then hold the
/// outputs in their safe state until the reset lands.
async fn hold_for_bootloader(mut hw: board::MotorPeripherals, mut servo_drive: servo::ServoDrive, seq: u8) -> ! {
    hw.wheels.set(0, 0);
    hw.wheels.set_enabled(false);
    hw.blade.stop();
    with_commands(|c| {
        c.servo.disable();
        servo_drive.apply(&c.servo);
    });
    TX_QUEUE.send(TxItem::EnterBootloader { seq }).await;
    loop {
        embassy_time::Timer::after(CONTROL_PERIOD).await;
        hw.blade.stop();
    }
}

// ---------------------------------------------------------------------------
// Status frames
// ---------------------------------------------------------------------------

fn send_status_burst() {
    let frames = with_telemetry(|t| {
        let feedback = WheelFeedbackStatus {
            left_target_rpm_x100: f32_to_i16_x100(t.wheels.left.target_rpm),
            left_measured_rpm_x100: f32_to_i16_x100(t.wheels.left.measured_rpm),
            right_target_rpm_x100: f32_to_i16_x100(t.wheels.right.target_rpm),
            right_measured_rpm_x100: f32_to_i16_x100(t.wheels.right.measured_rpm),
            left_pid_output: f32_to_i16(t.wheels.left.pid_output),
            right_pid_output: f32_to_i16(t.wheels.right.pid_output),
            left_total_counts: t.wheels.left.total_counts,
            right_total_counts: t.wheels.right.total_counts,
            flags: t.wheels.flags,
            reserved0: 0,
            reserved1: 0,
            reserved2: 0,
        };
        let pid = PidConfigStatus {
            left_kp: t.settings.left_wheel_pid.kp,
            left_ki: t.settings.left_wheel_pid.ki,
            left_kd: t.settings.left_wheel_pid.kd,
            right_kp: t.settings.right_wheel_pid.kp,
            right_ki: t.settings.right_wheel_pid.ki,
            right_kd: t.settings.right_wheel_pid.kd,
            flags: (if t.settings.closed_loop() { pid_status_flag::CLOSED_LOOP_ENABLED } else { 0 })
                | (if t.storage.flash_valid { pid_status_flag::FLASH_VALID } else { 0 })
                | (if t.storage.last_save_ok { pid_status_flag::LAST_SAVE_OK } else { 0 })
                | (if t.pid_last_apply_ok { pid_status_flag::LAST_APPLY_OK } else { 0 }),
            last_rx_seq: t.pid_last_rx_seq,
            reserved: 0,
        };
        [
            encode_payload(frame_type::MOTOR_STATUS, t.motor.last_rx_seq, &t.motor),
            encode_payload(frame_type::WHEEL_FEEDBACK_STATUS, t.motor.last_rx_seq, &feedback),
            encode_payload(frame_type::LAWER_MOTOR_STATUS, t.blade.last_rx_seq, &t.blade),
            encode_payload(frame_type::PID_CONFIG_STATUS, t.pid_last_rx_seq, &pid),
        ]
    });
    let now = now_ms();
    let charger = with_charger(|c| c.status(now));
    let servo = with_commands(|c| c.servo.status(now));
    let extra = [
        power_status_frame(),
        encode_payload(frame_type::CHARGER_STATUS, 0, &charger),
        encode_payload(frame_type::SERVO_STATUS, servo.last_rx_seq, &servo),
    ];
    for frame in frames.into_iter().chain(extra).flatten() {
        send_frame(frame);
    }
}

/// 0x86 with the only state this port can be in: running, rail on.
fn power_status_frame() -> Option<protocol::FrameBytes> {
    const STATE_RUNNING: u8 = 0;
    const FLAG_MAIN_POWER_ENABLED: u8 = 0x02;
    let seq = with_commands(|c| c.power_last_rx_seq);
    let status = PowerStatus {
        state: STATE_RUNNING,
        flags: FLAG_MAIN_POWER_ENABLED,
        shutdown_reason: 0,
        last_rx_seq: seq,
        press_ms: 0,
        shutdown_elapsed_ms: 0,
    };
    encode_payload(frame_type::POWER_STATUS, seq, &status)
}

// ---------------------------------------------------------------------------
// UART TX
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn uart_tx_task(mut tx: UartTx<'static, Async>) {
    loop {
        match TX_QUEUE.receive().await {
            TxItem::Frame(frame) => {
                // A DMA error here loses one status frame; the next burst
                // replaces it, so there is nothing useful to do with it.
                let _ = tx.write(&frame).await;
            }
            TxItem::EnterBootloader { seq } => enter_bootloader(&mut tx, seq),
        }
    }
}

/// Send the 0x8F ack synchronously, leave the magic in the RAM mailbox and
/// reset. Mirrors `enter_bootloader()` in `uart_interface.cpp`.
fn enter_bootloader(tx: &mut UartTx<'static, Async>, seq: u8) -> ! {
    const BOOT_STATUS_OK: u8 = 0;
    let ack = BootEnterAck { status: BOOT_STATUS_OK, reserved: [0; 3] };
    let frame = encode_payload(frame_type::ENTER_BOOTLOADER_ACK, seq, &ack).expect("ack fits");

    cortex_m::interrupt::disable();
    let _ = tx.blocking_write(&frame);
    let _ = tx.blocking_flush();

    // SAFETY: the mailbox is 32 bytes at the start of SRAM that `memory.x`
    // keeps out of this image's RAM, so nothing else references it.
    unsafe { core::ptr::write_volatile(BOOT_SHARED_MAGIC_PTR, BOOT_REQUEST_MAGIC) };
    cortex_m::asm::dsb();
    cortex_m::peripheral::SCB::sys_reset();
}
