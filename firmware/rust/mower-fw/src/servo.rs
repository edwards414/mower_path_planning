//! MG996 pulse generation on PB10, timed by TIM10 (port of the timer half
//! of `Module/Src/mg996_servo.cpp`).
//!
//! PB10 has no timer channel of its own, so the pulse is made with two
//! interrupts on an otherwise idle timer: TIM10 runs at 1 µs per tick with
//! a 20 ms period, the update interrupt raises the pin and the CH1 compare
//! interrupt (compare-only, no pin) drops it `pulse_us` later. Jitter is
//! the interrupt latency; the executor never sees the pulse at all.
//!
//! The ISR reads one atomic (`OUTPUT_ACTIVE`) and writes GPIOB's BSRR, so it
//! shares nothing with the tasks that needs a lock. CCR1 is preloaded: a new
//! width written mid-pulse only takes effect at the next period edge and can
//! never skip the compare match of the pulse in progress.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_stm32::gpio::Output;
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::pac;
use embassy_stm32::pac::timer::regs::Sr1ch;
use embassy_stm32::peripherals::TIM10;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::{OutputCompareMode, Timer};
use embassy_stm32::timer::Channel;
use mower_core::servo::{Servo, CENTER_PULSE_US, PERIOD_US};

use crate::board::ServoPeripherals;

/// Read by the TIM10 ISR on every update event: true = emit this pulse.
static OUTPUT_ACTIVE: AtomicBool = AtomicBool::new(false);

const SR_UIF: u32 = 1 << 0;
const SR_CC1IF: u32 = 1 << 1;
const PB10: usize = 10;

pub struct ServoDrive {
    timer: Timer<'static, TIM10>,
    /// Kept alive so PB10 stays a push-pull output; the ISR drives it via BSRR.
    pin: Output<'static>,
}

impl ServoDrive {
    pub fn new(hw: ServoPeripherals) -> Self {
        let ServoPeripherals { pin, mut timer } = hw;
        timer.set_tick_freq(Hertz::mhz(1));
        timer.set_max_compare_value(PERIOD_US - 1);
        timer.set_output_compare_mode(Channel::Ch1, OutputCompareMode::Frozen);
        timer.set_output_compare_preload(Channel::Ch1, true);
        timer.set_compare_value(Channel::Ch1, CENTER_PULSE_US);
        timer.clear_update_interrupt();
        timer.clear_input_interrupt(Channel::Ch1);
        timer.enable_update_interrupt(true);
        timer.enable_input_interrupt(Channel::Ch1, true);

        interrupt::TIM1_UP_TIM10.set_priority(interrupt::Priority::P5);
        // SAFETY: the handler below touches only TIM10's own status flags and
        // PB10's set/reset register; the pin and timer are configured above,
        // and nothing else registers this vector (TIM1 runs as an encoder
        // with its update interrupt disabled).
        unsafe { interrupt::TIM1_UP_TIM10.enable() };
        timer.start();

        Self { timer, pin }
    }

    /// Push the current command to the hardware. Called on the control tick
    /// after `Servo::tick`, and after every command / limit change.
    pub fn apply(&mut self, servo: &Servo) {
        self.timer.set_compare_value(Channel::Ch1, servo.pulse_us);
        let active = servo.output_active();
        OUTPUT_ACTIVE.store(active, Ordering::Release);
        if !active {
            // Cut a pulse in flight short rather than leave the pin high for
            // up to a full period; one short pulse is harmless to the servo.
            self.pin.set_low();
        }
    }
}

#[cortex_m_rt::interrupt]
fn TIM1_UP_TIM10() {
    let tim = pac::TIM10;
    let sr = tim.sr().read();
    // Flags are rc_w0: writing all ones except the flag clears only that
    // flag (the HAL's `__HAL_TIM_CLEAR_FLAG`), so a compare match landing
    // between the read and the write is never lost.
    if sr.uif() {
        tim.sr().write_value(Sr1ch(!SR_UIF));
        if OUTPUT_ACTIVE.load(Ordering::Acquire) {
            pac::GPIOB.bsrr().write(|w| w.set_bs(PB10, true));
        }
    }
    if sr.ccif(0) {
        tim.sr().write_value(Sr1ch(!SR_CC1IF));
        pac::GPIOB.bsrr().write(|w| w.set_br(PB10, true));
    }
}
