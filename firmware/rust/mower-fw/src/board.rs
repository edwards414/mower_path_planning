//! Peripheral bring-up for the mower board (STM32F411CE).
//!
//! Pin map and timer roles follow `mower_robot_firmware.ioc` and
//! `wire.md`; every pin that the C++ firmware drives and that this port
//! implements is claimed here, so a second driver on the same pin is a
//! move-after-use compile error rather than a wiring surprise.
//!
//! | Peripheral | Pins            | Role                                  |
//! |------------|-----------------|---------------------------------------|
//! | USART1     | PB6 TX, PA10 RX | host link, 115200 8N1, DMA2 ch7 / ch2 |
//! | TIM2       | PA15 PB3 PA2 PA3| wheel PWM 20 kHz (LL LR RL RR)        |
//! | TIM5       | PA0 PA1         | left encoder (FT-555)                 |
//! | TIM1       | PA8 PA9         | right encoder                         |
//! | TIM4 CH3   | PB8             | blade PWM 2 kHz (BLD120A)             |
//! | TIM9       | —               | embassy time driver (free timer)      |
//! | TIM10      | —               | MG996 pulse timing, 1 µs / 20 ms      |
//! | USART6     | PA11 TX, PA12 RX| RS485 charger, 9600 8N1, DMA2 ch6/ch1 |
//! | PA5        |                 | MAX485 DE+RE, high = drive the bus    |
//! | PB10       |                 | MG996 signal, edges from the TIM10 ISR|
//! | PA4        |                 | BTS7960 EN (shared, active high)      |
//! | PB7        |                 | BLD120A F/R, open-drain, low = run    |
//! | PC13       |                 | BLD120A BRK, open-drain, low = brake  |
//! | PB12–PB15  |                 | BTS7960 IS, advisory only             |
//! | PB2, PA6   |                 | MG996 limit switches UP / DN, pull-up, close to GND |
//! | PC15       |                 | MAIN_POWER_EN, held high              |
//! | PC14       |                 | LEBANCAT_WAKE, held low               |

use embassy_stm32::flash::{Blocking, Flash};
use embassy_stm32::gpio::{Input, Level, Output, OutputOpenDrain, OutputType, Pull, Speed};
use embassy_stm32::peripherals::{TIM1, TIM10, TIM2, TIM4, TIM5};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::{CountingMode, Timer};
use embassy_stm32::timer::qei::{Qei, QeiMode};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm, SimplePwmChannel, SimplePwmChannels};
use embassy_stm32::{rcc, Config, Peripherals};
use mower_core::command::PWM_MAX_COUNTS;
use mower_core::settings::{self, Record, RECORD_SIZE};
use zerocopy::IntoBytes;

/// 25 MHz HSE → PLL → 100 MHz SYSCLK, APB1 50 MHz, APB2 100 MHz.
/// Same tree as `SystemClock_Config()` in `Core/Src/main.c`.
pub fn clock_config() -> Config {
    let mut config = Config::default();
    config.rcc.hse = Some(rcc::Hse { freq: Hertz::mhz(25), mode: rcc::HseMode::Oscillator });
    config.rcc.pll_src = rcc::PllSource::HSE;
    config.rcc.pll = Some(rcc::Pll {
        prediv: rcc::PllPreDiv::DIV12,
        mul: rcc::PllMul::MUL96,
        divp: Some(rcc::PllPDiv::DIV2), // 100 MHz
        divq: Some(rcc::PllQDiv::DIV4), // 50 MHz (unused: no USB/SDIO)
        divr: None,
    });
    config.rcc.sys = rcc::Sysclk::PLL1_P;
    config.rcc.ahb_pre = rcc::AHBPrescaler::DIV1;
    config.rcc.apb1_pre = rcc::APBPrescaler::DIV2;
    config.rcc.apb2_pre = rcc::APBPrescaler::DIV1;
    config
}

/// Wheel motors are mirror-mounted (measured 2026-09-11): a positive
/// command must drive the left wheel through its "R" half-bridge and the
/// right wheel through its "L" half-bridge for both to go vehicle-forward.
const LEFT_DIRECTION_SIGN: i32 = -1;
const RIGHT_DIRECTION_SIGN: i32 = 1;

/// Both BTS7960 pairs on TIM2 plus the shared enable pin.
pub struct WheelDrive {
    /// ch1 = left "L" (PA15), ch2 = left "R" (PB3),
    /// ch3 = right "L" (PA2), ch4 = right "R" (PA3).
    ch: SimplePwmChannels<'static, TIM2>,
    max_duty: u32,
    enable: Output<'static>,
}

impl WheelDrive {
    fn new(pwm: SimplePwm<'static, TIM2>, enable: Output<'static>) -> Self {
        let max_duty = pwm.max_duty_cycle();
        let mut ch = pwm.split();
        for c in [&mut ch.ch1, &mut ch.ch2, &mut ch.ch3, &mut ch.ch4] {
            c.set_duty_cycle(0);
            c.enable();
        }
        Self { ch, max_duty, enable }
    }

    /// Apply signed PWM counts (±200) per wheel; positive = vehicle forward.
    pub fn set(&mut self, left_pwm: i16, right_pwm: i16) {
        let left = Self::half_bridges(left_pwm, LEFT_DIRECTION_SIGN);
        let right = Self::half_bridges(right_pwm, RIGHT_DIRECTION_SIGN);
        let max = self.max_duty;
        self.ch.ch1.set_duty_cycle(Self::duty(left.0, max));
        self.ch.ch2.set_duty_cycle(Self::duty(left.1, max));
        self.ch.ch3.set_duty_cycle(Self::duty(right.0, max));
        self.ch.ch4.set_duty_cycle(Self::duty(right.1, max));
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        self.enable.set_level(if enabled { Level::High } else { Level::Low });
    }

    /// (counts on the L half-bridge, counts on the R half-bridge)
    fn half_bridges(pwm: i16, sign: i32) -> (u16, u16) {
        let signed = (i32::from(pwm) * sign).clamp(-i32::from(PWM_MAX_COUNTS), i32::from(PWM_MAX_COUNTS));
        if signed >= 0 {
            (signed as u16, 0)
        } else {
            (0, (-signed) as u16)
        }
    }

    /// Rescale 0..=200 counts to the timer's actual ARR.
    fn duty(counts: u16, max_duty: u32) -> u32 {
        u32::from(counts) * max_duty / u32::from(PWM_MAX_COUNTS as u16)
    }
}

/// BLD120A blade driver: PWM on TIM4 CH3, direction and brake as open-drain.
pub struct Blade {
    pwm: SimplePwmChannel<'static, TIM4>,
    max_duty: u32,
    /// PB7 F/R. Only "low" runs (2026-09-10: the other direction is dead).
    direction: OutputOpenDrain<'static>,
    /// PC13 BRK. Low = brake, high-Z = release.
    brake: OutputOpenDrain<'static>,
}

impl Blade {
    /// `pwm` in timer counts 0..=200 like the C++ `Grass_cutting_motor`.
    /// `dir == 1` runs forward; anything else brakes.
    pub fn set(&mut self, pwm: u16, dir: u8) {
        if pwm == 0 || dir != 1 {
            self.stop();
            return;
        }
        self.brake.set_high();
        self.direction.set_low();
        self.pwm.set_duty_cycle(
            u32::from(pwm.min(PWM_MAX_COUNTS as u16)) * self.max_duty / u32::from(PWM_MAX_COUNTS as u16),
        );
    }

    /// PWM 0 and brake engaged.
    pub fn stop(&mut self) {
        self.pwm.set_duty_cycle(0);
        self.brake.set_low();
    }
}

pub struct Encoders {
    left: Qei<'static, TIM5>,
    right: Qei<'static, TIM1>,
}

impl Encoders {
    /// Raw (left, right) counter values. Both are 16-bit on this HAL.
    pub fn read(&self) -> (u32, u32) {
        (u32::from(self.left.count()), u32::from(self.right.count()))
    }
}

/// BTS7960 IS pins. Analog current-sense read as digital, so advisory only
/// (they trip on normal inrush — see `motor.hpp`).
pub struct Alarms {
    pins: [Input<'static>; 4],
}

impl Alarms {
    pub fn any_active(&self) -> bool {
        self.pins.iter().any(|p| p.is_high())
    }
}

/// Sector 7 (0x0806_0000, 128 KB) holds one settings record at its start.
pub struct SettingsFlash {
    flash: Flash<'static, Blocking>,
}

impl SettingsFlash {
    const OFFSET: u32 = settings::FLASH_ADDRESS - embassy_stm32::flash::FLASH_BASE as u32;
    const SECTOR_END: u32 = Self::OFFSET + 128 * 1024;

    pub fn load(&mut self) -> Option<Record> {
        let mut bytes = [0u8; RECORD_SIZE];
        self.flash.blocking_read(Self::OFFSET, &mut bytes).ok()?;
        Record::from_flash(&bytes)
    }

    /// Erase the sector and write `record`. Blocks the CPU for the erase
    /// (~1–2 s on a 128 KB sector), exactly like the HAL version did.
    pub fn store(&mut self, record: &Record) -> bool {
        self.flash.blocking_erase(Self::OFFSET, Self::SECTOR_END).is_ok()
            && self.flash.blocking_write(Self::OFFSET, record.as_bytes()).is_ok()
            && self.load().as_ref() == Some(record)
    }
}

/// Everything the motor task owns.
pub struct MotorPeripherals {
    pub wheels: WheelDrive,
    pub blade: Blade,
    pub encoders: Encoders,
    pub alarms: Alarms,
    pub settings_flash: SettingsFlash,
}

/// Pins the power manager would drive; held in their "board on" state until
/// that module is ported. Never read, only kept alive (dropping an `Output`
/// would float the rail-enable pin).
#[allow(dead_code)]
pub struct PowerPins {
    pub main_power_en: Output<'static>,
    pub lebancat_wake: Output<'static>,
}

/// USART6 + MAX485 direction pin, owned by the charger task.
pub struct ChargerPeripherals {
    pub usart6: embassy_stm32::Peri<'static, embassy_stm32::peripherals::USART6>,
    pub tx_pin: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA11>,
    pub rx_pin: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA12>,
    pub tx_dma: embassy_stm32::Peri<'static, embassy_stm32::peripherals::DMA2_CH6>,
    pub rx_dma: embassy_stm32::Peri<'static, embassy_stm32::peripherals::DMA2_CH1>,
    /// PA5 → MAX485 DE and RE (tied). High = drive the bus, low = listen.
    pub de: Output<'static>,
}

/// PB10 plus the timer that times its pulses (see `servo.rs`), and the two
/// travel limit microswitches.
pub struct ServoPeripherals {
    pub pin: Output<'static>,
    pub timer: Timer<'static, TIM10>,
    pub limits: LimitSwitches,
}

/// One microswitch per end of the mechanism, between the pin and GND with
/// the internal pull-up (`SERVO_LIMIT_UP` / `SERVO_LIMIT_DN` in the C++
/// build). `PRESSED_LEVEL_LOW` mirrors `SERVO_LIMIT_PRESSED_LEVEL`: true for
/// an NO contact, false for an NC one (broken wire reads as pressed).
pub struct LimitSwitches {
    up: Input<'static>,
    dn: Input<'static>,
}

impl LimitSwitches {
    const PRESSED_LEVEL_LOW: bool = true;

    /// Raw `(up, dn)` — debounce it with `mower_core::servo::LimitDebounce`.
    pub fn read(&self) -> (bool, bool) {
        let pressed = |pin: &Input<'static>| pin.is_low() == Self::PRESSED_LEVEL_LOW;
        (pressed(&self.up), pressed(&self.dn))
    }
}

pub struct Board {
    pub motor: MotorPeripherals,
    pub power: PowerPins,
    pub charger: ChargerPeripherals,
    pub servo: ServoPeripherals,
    pub usart1: embassy_stm32::Peri<'static, embassy_stm32::peripherals::USART1>,
    pub uart_tx_pin: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PB6>,
    pub uart_rx_pin: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA10>,
    pub uart_tx_dma: embassy_stm32::Peri<'static, embassy_stm32::peripherals::DMA2_CH7>,
    pub uart_rx_dma: embassy_stm32::Peri<'static, embassy_stm32::peripherals::DMA2_CH2>,
}

pub fn init(p: Peripherals) -> Board {
    // Power rail first, like PowerManager_Init(): the board must not brown
    // out while the rest is being configured.
    let power = PowerPins {
        main_power_en: Output::new(p.PC15, Level::High, Speed::Low),
        lebancat_wake: Output::new(p.PC14, Level::Low, Speed::Low),
    };

    let wheel_pwm = SimplePwm::new(
        p.TIM2,
        Some(PwmPin::new(p.PA15, OutputType::PushPull)),
        Some(PwmPin::new(p.PB3, OutputType::PushPull)),
        Some(PwmPin::new(p.PA2, OutputType::PushPull)),
        Some(PwmPin::new(p.PA3, OutputType::PushPull)),
        Hertz::khz(20),
        CountingMode::EdgeAlignedUp,
    );
    let wheels = WheelDrive::new(wheel_pwm, Output::new(p.PA4, Level::High, Speed::Low));

    let mut blade_pwm = SimplePwm::new(
        p.TIM4,
        None,
        None,
        Some(PwmPin::new(p.PB8, OutputType::PushPull)),
        None, // PB9 buzzer: not ported yet
        Hertz::khz(2),
        CountingMode::EdgeAlignedUp,
    );
    blade_pwm.ch3().set_duty_cycle(0);
    blade_pwm.ch3().enable();
    let max_duty = blade_pwm.max_duty_cycle();
    let blade = Blade {
        pwm: blade_pwm.split().ch3,
        max_duty,
        direction: OutputOpenDrain::new(p.PB7, Level::High, Speed::Low),
        brake: OutputOpenDrain::new(p.PC13, Level::Low, Speed::Low),
    };

    let qei = |mode| embassy_stm32::timer::qei::Config {
        ch1_pull: Pull::None,
        ch2_pull: Pull::None,
        mode,
        auto_reload: u16::MAX,
    };
    let encoders = Encoders {
        left: Qei::new(p.TIM5, p.PA0, p.PA1, qei(QeiMode::Mode3)),
        right: Qei::new(p.TIM1, p.PA8, p.PA9, qei(QeiMode::Mode3)),
    };

    let alarms = Alarms {
        pins: [
            Input::new(p.PB12, Pull::Down),
            Input::new(p.PB13, Pull::Down),
            Input::new(p.PB14, Pull::Down),
            Input::new(p.PB15, Pull::Down),
        ],
    };

    let settings_flash = SettingsFlash { flash: Flash::new_blocking(p.FLASH) };

    let charger = ChargerPeripherals {
        usart6: p.USART6,
        tx_pin: p.PA11,
        rx_pin: p.PA12,
        tx_dma: p.DMA2_CH6,
        rx_dma: p.DMA2_CH1,
        de: Output::new(p.PA5, Level::Low, Speed::Low),
    };

    let servo = ServoPeripherals {
        pin: Output::new(p.PB10, Level::Low, Speed::Low),
        timer: Timer::new(p.TIM10),
        limits: LimitSwitches { up: Input::new(p.PB2, Pull::Up), dn: Input::new(p.PA6, Pull::Up) },
    };

    Board {
        motor: MotorPeripherals { wheels, blade, encoders, alarms, settings_flash },
        power,
        charger,
        servo,
        usart1: p.USART1,
        uart_tx_pin: p.PB6,
        uart_rx_pin: p.PA10,
        uart_tx_dma: p.DMA2_CH7,
        uart_rx_dma: p.DMA2_CH2,
    }
}
