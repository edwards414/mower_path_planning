//! RS485 charger poller (port of `Module/Src/charger_rs485.cpp`).
//!
//! Every [`charger::POLL_PERIOD_MS`] the task sends one Modbus FC03 read of
//! registers 0-4 to the 数控 30V5A module on USART6 and collects the reply
//! with DMA + idle-line detection. The MAX485 direction pin is raised for
//! the request and dropped as soon as the TC interrupt says the last stop
//! bit is out (`flush().await`), so the slave's turnaround is not eaten.
//!
//! Compared with the C++ state machine there is no state machine: the poll
//! is one straight-line `async fn`, the 200 ms reply timeout is
//! `with_timeout`, and RX is armed before TX by polling the receive future
//! first in a `join`, so an echoed request (RE tied low) lands in the buffer
//! instead of overrunning the data register.
//!
//! The C++ build pauses polling while the main rail is off; the power
//! manager is not ported here, so this task polls for the whole run.

use embassy_futures::join::join;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{self, Uart, UartRx, UartTx};
use embassy_time::{with_timeout, Duration, Ticker};
use mower_core::charger::{self, REG_COUNT};
use mower_core::modbus::{find_read_reply, ReadReply};

use crate::shared::with_charger;

/// Request (8) + echo (8) + reply (15) with margin; a longer burst of
/// garbage just ends the poll as a bad frame.
const RX_BUF_SIZE: usize = 64;
const POLL_PERIOD: Duration = Duration::from_millis(charger::POLL_PERIOD_MS);
const REPLY_TIMEOUT: Duration = Duration::from_millis(charger::REPLY_TIMEOUT_MS);

pub struct ChargerLink {
    tx: UartTx<'static, Async>,
    rx: UartRx<'static, Async>,
    de: Output<'static>,
}

impl ChargerLink {
    /// USART6 settings for the charger: 9600 8N1. RO goes high-Z while the
    /// MAX485 receiver is disabled (RE tied to DE), so RX is pulled up to
    /// keep that window from being read as noise.
    pub fn uart_config() -> usart::Config {
        let mut config = usart::Config::default();
        config.baudrate = 9600;
        config.rx_pull = embassy_stm32::gpio::Pull::Up;
        config
    }

    /// `uart` is USART6 opened with [`Self::uart_config`]; `de` is PA5.
    pub fn new(uart: Uart<'static, Async>, de: Output<'static>) -> Self {
        let (tx, rx) = uart.split();
        Self { tx, rx, de }
    }

    /// One request/reply exchange. `Ok(regs)` on a complete reply,
    /// `Err(Some(code))` on a Modbus exception, `Err(None)` on timeout,
    /// short frame or bus error.
    async fn poll_once(&mut self) -> Result<[u16; REG_COUNT], Option<u8>> {
        let Self { tx, rx, de } = self;
        let request = charger::read_request();
        let mut buf = [0u8; RX_BUF_SIZE];
        let mut regs = [0u16; REG_COUNT];

        let receive = async {
            let mut len = 0usize;
            loop {
                // A fresh DMA read each time the line goes idle: the echo
                // and the reply arrive as separate idle-terminated chunks.
                let n = rx.read_until_idle(&mut buf[len..]).await.map_err(|_| None)?;
                len = (len + n).min(RX_BUF_SIZE);
                match find_read_reply(&buf[..len], charger::SLAVE_ADDR, &mut regs) {
                    ReadReply::Ok { count, .. } if count >= REG_COUNT => return Ok(()),
                    ReadReply::Ok { .. } => return Err(None),
                    ReadReply::Exception { code, .. } => return Err(Some(code)),
                    ReadReply::None if len >= RX_BUF_SIZE => return Err(None),
                    ReadReply::None => {}
                }
            }
        };
        let transmit = async {
            de.set_high();
            let sent = tx.write(&request).await;
            // TC interrupt: the last stop bit has left the shifter.
            let flushed = tx.flush().await;
            de.set_low();
            sent.and(flushed).map_err(|_| None::<u8>)
        };

        // `join` polls `receive` first, so RX DMA is armed before the first
        // byte of the request can be echoed back.
        let (received, sent) = with_timeout(REPLY_TIMEOUT, join(receive, transmit)).await.map_err(|_| None)?;
        sent?;
        received?;
        Ok(regs)
    }
}

#[embassy_executor::task]
pub async fn charger_task(mut link: ChargerLink) {
    let mut ticker = Ticker::every(POLL_PERIOD);
    loop {
        // First poll one period after boot so the module has time to power up.
        ticker.next().await;
        let result = link.poll_once().await;
        // Any bytes still on the wire after a timeout belong to this poll,
        // not the next: make sure the driver is idle before we go round.
        link.de.set_low();
        let now = embassy_time::Instant::now().as_millis() as u32;
        with_charger(|snapshot| match result {
            Ok(regs) => snapshot.record_success(&regs, now),
            Err(exception) => snapshot.record_failure(exception),
        });
    }
}
