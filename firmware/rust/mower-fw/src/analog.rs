//! Analog monitor task (port of `Module/Src/analog_monitor.cpp`).
//!
//! Four slow signals share `PB1 / ADC1_IN9` through a 4-channel analog mux
//! selected by `PB2` (S0) and `PA6` (S1; the UFQFPN48 part has no PB11):
//! MG996 current sense, board NTC, 24 V main battery divider, 3.7 V AON
//! battery divider. Every
//! [`analog::SCAN_PERIOD_MS`] the task reads VREFINT to learn the real
//! rail voltage, then walks the four channels with a 1 ms settle after each
//! mux switch, and publishes the snapshot for the `0x8A` frame and for the
//! servo current limit.
//!
//! The ADC is blocking and the whole scan is ~5 ms of mostly sleeping, so
//! it stays a plain task instead of a DMA sequence.

use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals::{ADC1, PB1};
use embassy_stm32::Peri;
use embassy_time::{Duration, Ticker, Timer};
use mower_core::analog::{self, Channel, Snapshot};

use crate::shared::with_analog;

const SCAN_PERIOD: Duration = Duration::from_millis(analog::SCAN_PERIOD_MS);
/// Mux settle after switching channels (the C++ `HAL_Delay(1)`).
const SETTLE: Duration = Duration::from_millis(1);

/// Factory VREFINT reading at 3.3 V / 30 °C (RM0383 §6.1.5).
const VREFINT_CAL_ADDR: *const u16 = 0x1FFF_7A2A as *const u16;

pub struct AnalogPeripherals {
    pub adc: Peri<'static, ADC1>,
    pub mux_out: Peri<'static, PB1>,
    pub mux_s0: Output<'static>,
    pub mux_s1: Output<'static>,
}

pub struct AnalogMonitor {
    adc: Adc<'static, ADC1>,
    mux_out: Peri<'static, PB1>,
    mux_s0: Output<'static>,
    mux_s1: Output<'static>,
    vrefint_cal: u16,
}

impl AnalogMonitor {
    pub fn new(p: AnalogPeripherals) -> Self {
        let adc = Adc::new(p.adc);
        // SAFETY: fixed system-memory address holding the factory
        // calibration word; read-only, always mapped, 2-byte aligned.
        let vrefint_cal = unsafe { core::ptr::read_volatile(VREFINT_CAL_ADDR) };
        Self { adc, mux_out: p.mux_out, mux_s0: p.mux_s0, mux_s1: p.mux_s1, vrefint_cal }
    }

    fn read_vdda(&mut self) -> Option<f32> {
        // TSVREFE stays set once enabled; 480 cycles at ADC clock 25 MHz is
        // ~19 µs, above the 10 µs VREFINT sampling minimum.
        let mut vrefint = self.adc.enable_vrefint();
        let raw = self.adc.blocking_read(&mut vrefint, SampleTime::CYCLES480);
        analog::vdda_from_vrefint(raw, self.vrefint_cal)
    }

    async fn read_channel(&mut self, channel: Channel) -> u16 {
        let (s0, s1) = channel.select_bits();
        self.mux_s0.set_level(s0.into());
        self.mux_s1.set_level(s1.into());
        Timer::after(SETTLE).await;
        self.adc.blocking_read(&mut self.mux_out, SampleTime::CYCLES84)
    }

    async fn scan(&mut self, snapshot: &mut Snapshot) {
        snapshot.calibrate_vdda(self.read_vdda());
        for channel in Channel::ALL {
            let raw = self.read_channel(channel).await;
            snapshot.record(channel, Some(raw));
        }
    }
}

#[embassy_executor::task]
pub async fn analog_task(mut monitor: AnalogMonitor) {
    let mut ticker = Ticker::every(SCAN_PERIOD);
    let mut snapshot = Snapshot::ZERO;
    loop {
        ticker.next().await;
        // Threshold can be changed from the shared copy (C++ SetMg996CurrentLimitRaw).
        snapshot.mg996_limit_raw = with_analog(|a| a.mg996_limit_raw);
        monitor.scan(&mut snapshot).await;
        with_analog(|a| *a = snapshot);
    }
}
