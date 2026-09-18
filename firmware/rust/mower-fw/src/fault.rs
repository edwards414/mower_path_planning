//! What happens when the firmware itself fails.
//!
//! The C++ build has no fault handler: a HardFault leaves every PWM output
//! at its last duty cycle with the drivers enabled. Here both a Rust panic
//! and a HardFault go through [`safe_stop`], which uses raw register writes
//! (no driver state to trust any more) to kill the wheel drive and brake the
//! blade before parking the core.

use cortex_m_rt::{exception, ExceptionFrame};
use embassy_stm32::pac;

/// Wheel PWM 0, BTS7960 EN low, blade PWM 0, BLD120A BRK low, servo signal
/// low, RS485 driver released.
fn safe_stop() {
    cortex_m::interrupt::disable();
    for channel in 0..4 {
        pac::TIM2.ccr(channel).write_value(0);
    }
    pac::TIM4.ccr(2).write(|w| w.set_ccr(0));
    pac::GPIOA.bsrr().write(|w| w.set_br(4, true));
    pac::GPIOC.bsrr().write(|w| w.set_br(13, true));
    pac::GPIOB.bsrr().write(|w| w.set_br(10, true));
    pac::GPIOA.bsrr().write(|w| w.set_br(5, true));
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    safe_stop();
    loop {
        cortex_m::asm::wfi();
    }
}

#[exception]
unsafe fn HardFault(_frame: &ExceptionFrame) -> ! {
    safe_stop();
    loop {
        cortex_m::asm::wfi();
    }
}
