//! Hardware-independent logic for the mower firmware.
//!
//! Everything in this crate is `no_std`, allocation-free and has no
//! dependency on the MCU, so it is unit-tested on the host with
//! `cargo test`. The firmware crate (`mower-fw`) wires these pieces to the
//! STM32 peripherals.
//!
//! Mirrors the C++ modules under `Module/`:
//!
//! | C++                    | here                      |
//! |------------------------|---------------------------|
//! | `uart_interface.cpp`   | [`protocol`]              |
//! | `pid_controller.cpp`   | [`pid`]                   |
//! | `wheel_controller.cpp` | [`wheel`]                 |
//! | `motor.cpp` (timeouts) | [`command`]               |
//! | `settings_storage.cpp` | [`settings`]              |
//! | `modbus_rtu.cpp`       | [`modbus`]                |
//! | `charger_rs485.cpp`    | [`charger`] (state only)  |
//! | `mg996_servo.cpp`      | [`servo`] (state only)    |
//! | `analog_monitor.cpp`   | [`analog`] (maths only)   |

#![cfg_attr(not(test), no_std)]
#![deny(unsafe_code)]

pub mod analog;
pub mod charger;
pub mod command;
pub mod modbus;
pub mod pid;
pub mod protocol;
pub mod servo;
pub mod settings;
pub mod wheel;

/// Wire structs are read/written with the MCU's native byte order, exactly
/// like the C++ `memcpy` into `__attribute__((packed))` structs. The host
/// side (ROS 2) assumes little-endian, so refuse to build anywhere else.
const _: () = assert!(cfg!(target_endian = "little"), "wire format is little-endian");
