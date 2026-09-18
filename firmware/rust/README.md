# Rust port (experimental)

A Rust/Embassy rewrite of the **UART protocol + motor control slice** of the
C++ firmware (plus the RS485 charger and the MG996 servo), built to evaluate
what memory/concurrency safety Rust buys on this board. It is **not yet flashed or bench-tested**; the C++ firmware in
the repo root is still the one that runs the mower.

```
rust/
├── mower-core/   no_std, hardware-free logic — unit-tested on the host
│   ├── protocol  frame parser, CRC-16, zerocopy payload structs
│   ├── pid       PID with clamped integral/output
│   ├── wheel     encoder → RPM → PID → PWM (20 ms tick)
│   ├── command   dead-man timeout for host commands
│   ├── settings  flash record (CRC-32), sanitising, defaults
│   ├── modbus    Modbus RTU CRC-16, FC03/FC16 requests, echo-tolerant reply scan
│   ├── charger   RS485 charger snapshot, online logic, 0x89 flags
│   ├── servo     MG996 command state: clamp, hold timeout, limit gating, 0x88
│   └── analog    ADC mux maths: VREFINT → VDDA, dividers, NTC, 0x8A
└── mower-fw/     Embassy firmware for STM32F411CE
    ├── board.rs   pin map / timers / flash (mirrors the .ioc)
    ├── shared.rs  cross-task state (Mutex<RefCell>, Channel, Signal)
    ├── fault.rs   panic + HardFault → motors off, blade braked, servo/RS485 released
    ├── charger.rs USART6 + MAX485 DE poll task (DMA, idle-line, 200 ms timeout)
    ├── servo.rs   TIM10 update/compare ISR → PB10 pulse
    ├── analog.rs  ADC1 + PB2/PA6 mux select scan task (200 ms)
    └── main.rs    uart_rx_task · motor_task · uart_tx_task · charger_task · analog_task
```

## What is ported

| C++ module                 | Rust                         | Notes                                              |
|----------------------------|------------------------------|----------------------------------------------------|
| `uart_interface.cpp`       | `protocol.rs`, `main.rs`     | same framing, CRC, payload layouts (byte-for-byte) |
| `motor.cpp`                | `board.rs`, `command.rs`     | TIM2 wheel PWM, BTS7960 EN, direction signs        |
| `wheel_controller.cpp`     | `wheel.rs`                   | TIM5/TIM1 encoders, closed/open loop, deadband     |
| `pid_controller.cpp`       | `pid.rs`                     |                                                    |
| blade (`Grass_cutting_motor`) | `board.rs::Blade`         | TIM4 CH3, PB7 dir, PC13 BRK                        |
| `settings_storage.cpp`     | `settings.rs`, `board.rs`    | same record layout in sector 7                     |
| bootloader hand-off (0x0F) | `main.rs::enter_bootloader`  | same RAM mailbox + reset                           |
| `modbus_rtu.cpp`           | `modbus.rs`                  | vendor example frames as unit tests                |
| `charger_rs485.cpp`        | `charger.rs` (both crates)   | USART6 PA11/PA12, PA5 DE, 500 ms poll, `0x89`     |
| `mg996_servo.cpp`          | `servo.rs` (both crates)     | TIM10 ISR on PB10, `0x07` command, `0x88` status   |
| `analog_monitor.cpp`       | `analog.rs` (both crates)    | ADC1 IN9 behind the mux, VREFINT calibration, `0x8A`; feeds the servo current limit |

Status frames sent every 50 ms: `0x81` motor, `0x85` wheel feedback, `0x82`
blade, `0x84` PID config, `0x86` power (always "running, rail on"), `0x88`
servo, `0x89` charger, `0x8A` analog (batteries, board temperature).

## Not ported (yet)

Power button / shutdown hand-shake, WS2812 light shows and boot animation,
buzzer. `0x03` WS2812 commands are accepted and ignored; the `0x83` status
frame is not sent so the host can tell the feature is absent. Without the
power manager the charger is polled for the whole run (the C++ build pauses
while the rail is off). The MG996 current-limit threshold is fixed at the
C++ default (4095 = never trips) because no host command sets it yet. PC15 `MAIN_POWER_EN` is held high and
PC14 `LEBANCAT_WAKE` low for the whole run.

## Where the safety comes from

* **Shared state** (`shared.rs`): everything the RX task hands to the motor
  task lives in `Mutex<CriticalSectionRawMutex, RefCell<_>>`. There is no way
  to read a command field outside `lock(|s| …)` — the nine hand-written
  "snapshot under `__disable_irq()`" blocks in C++ are gone.
* **Frame decoding** (`protocol.rs`): payloads are decoded with `zerocopy`,
  so a wrong length is a `None`, not a short `memcpy`. The parser buffer is a
  `heapless::Vec`; an index bug is a dropped frame, not a stack write. Frame
  types are an enum, so an unhandled type is a compile error.
* **Flash record** (`settings.rs`): the `reinterpret_cast` of the flash
  address becomes a checked, alignment-free `Record::from_flash`.
* **Ownership of peripherals** (`board.rs`): each pin/timer is claimed once;
  a second driver on the same pin does not compile.
* **Fault path** (`fault.rs`): a panic or HardFault disables the BTS7960s and
  brakes the blade before halting. The C++ build has no equivalent.
* **Control timing**: `Ticker::every(20 ms)` schedules from the previous
  deadline, so PID `dt` does not drift the way `osDelay(20)` did.
* **Servo pulse** (`servo.rs`): the TIM10 ISR shares one `AtomicBool` with
  the tasks and writes GPIOB's BSRR — no lock, no shared mutable state. The
  C++ version guards three task/ISR writers with `__disable_irq()`.
* **Charger poll** (`charger.rs`): the C++ state machine (idle / waiting,
  ISR re-arming, timeout tick) is one `async fn` with `with_timeout` and a
  `join`; the RX buffer is only ever sliced, never indexed by an ISR.
* `mower-core` is `#![deny(unsafe_code)]`; the firmware has exactly three
  `unsafe` blocks (VTOR write, bootloader mailbox write, enabling the TIM10
  interrupt vector), all commented.

## Build

Toolchain is pinned by `rust-toolchain.toml` (1.95.0 + `thumbv7em-none-eabihf`).
On this Mac the Homebrew `cargo` shadows rustup's; use the rustup one:

```sh
export PATH="$HOME/.rustup/toolchains/1.95.0-aarch64-apple-darwin/bin:$PATH"

cd rust
cargo test                       # mower-core unit tests on the host (63 tests)
cd mower-fw
cargo build --release            # -> ../target.nosync/thumbv7em-none-eabihf/release/mower-fw
cargo clippy --release
```

The build directory is `target.nosync/` on purpose: the repo lives in an
iCloud-synced Desktop folder and cargo's file clones stall inside it.

### Flash layout

`memory.x` matches `Module/Inc/boot_shared.h`: the image links at
`0x0800_8000` behind the 32 KB UART bootloader, RAM starts at `0x2000_0020`
after the boot mailbox, sector 7 is left for settings. The bootloader must
already be in sectors 0–1 (it is on the bench board); on a blank chip
nothing would jump to the app.

### Flash the board

Same OpenOCD config as the C++ build; `cargo run --release` invokes it:

```sh
openocd -f ../../mower_robot_debug.cfg \
  -c 'program target.nosync/thumbv7em-none-eabihf/release/mower-fw verify reset exit'
```

Or over the existing UART bootloader with a raw binary:

```sh
LT=$HOME/.rustup/toolchains/1.95.0-aarch64-apple-darwin/lib/rustlib/aarch64-apple-darwin/bin
$LT/llvm-objcopy -O binary target.nosync/thumbv7em-none-eabihf/release/mower-fw mower-fw.bin
```

## Bench-test checklist (first power-up)

1. Wheels off the ground, blade disconnected.
2. Confirm `0x81`/`0x85` frames arrive at 20 Hz and `command_age_ms` counts
   up with the COMMAND_TIMEOUT flag set.
3. Send `0x01` with small ‰ values and check encoder sign: forward command →
   positive `measured_rpm` on both wheels. If a wheel reads negative, flip
   `LEFT/RIGHT_ENCODER_SIGN` in `wheel.rs`; if it spins backwards, flip
   `LEFT/RIGHT_DIRECTION_SIGN` in `board.rs`.
4. `0x04` with `persist_to_flash = 1`, power-cycle, check `FLASH_VALID`.
5. `0x0F` bootloader request → `0x8F` ack, board reappears as bootloader.
6. Servo: `tools/mower_uart.py PORT servo 1500` → scope on PB10 shows a
   20.00 ms period, 1.500 ms high; `0x88` reports `ENABLED|OUTPUT`. Try 500 /
   2500 for the MG996R end stops, then `servo 0` releases it.
7. Charger (MAX485 on PA11/PA12, DE+RE on PA5, module at Modbus addr 1):
   `0x89` should show `ONLINE|SEEN` within ~1 s and `Vout` close to the
   battery voltage; `comm_error_count` must stay put. If it climbs, scope
   PA5: DE has to fall before the module's reply starts (≈ 3–5 ms after the
   request). With RE tied to GND instead of PA5 the echoed request is
   expected and harmless.
