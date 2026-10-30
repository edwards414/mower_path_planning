//! The serial link and the clocks, behind traits so the protocol can run on a
//! simulated board and a simulated clock in the tests.
//!
//! [`SerialLink`] keeps pyserial 3.5's semantics (`serialposix.py`), because
//! the read cadence is part of what the flasher must reproduce (E6 design D2):
//!
//! * `read(n)` with a timeout returns once `n` bytes have arrived or the
//!   timeout is over, whichever is first; a 14-byte ACK therefore ends the
//!   50 ms window `FrameReader.read` opens, not its own arrival;
//! * a port that polls readable and then reads 0 bytes is "device reports
//!   readiness to read but returned no data" (a pty whose other end closed,
//!   a USB adapter pulled), a failed read is "read failed: [Errno N] ...";
//!   EAGAIN / EINTR are retried;
//! * `write` queues the frame and returns; nothing waits for it to drain.
//!
//! Opening goes through `serialport` (the same code `mower_base` opens
//! `/dev/stmcom` with): 115200 8N1 raw, no flow control, DTR and RTS raised
//! like pyserial does, the input flushed twice like `mower_flash.py` does. One
//! deliberate difference: serialport takes the port exclusively (TIOCEXCL and
//! a non-blocking flock), so a port someone else holds fails at once instead
//! of sharing its bytes.

use std::os::unix::io::AsRawFd;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serialport::{ClearBuffer, SerialPort, TTYPort};

use crate::Failure;

/// What the flasher needs from a serial port.
pub trait Link {
    /// Queue `data` for sending.
    fn write(&mut self, data: &[u8]) -> Result<(), Failure>;
    /// pyserial's `port.timeout = window; port.read(max)`.
    fn read(&mut self, max: usize, window: f64) -> Result<Vec<u8>, Failure>;
}

/// Python's `time.monotonic()`, `time.time()` and `time.sleep()`.
pub trait Clock {
    fn monotonic(&self) -> f64;
    fn time(&self) -> f64;
    fn sleep(&self, seconds: f64);
}

/// The real clocks.
pub struct SystemClock {
    start: Instant,
}

impl Default for SystemClock {
    fn default() -> Self {
        Self { start: Instant::now() }
    }
}

impl Clock for SystemClock {
    fn monotonic(&self) -> f64 {
        self.start.elapsed().as_secs_f64()
    }
    fn time(&self) -> f64 {
        SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0)
    }
    fn sleep(&self, seconds: f64) {
        if seconds > 0.0 {
            std::thread::sleep(Duration::from_secs_f64(seconds));
        }
    }
}

fn errno_text(errno: i32) -> String {
    let s = std::io::Error::from_raw_os_error(errno).to_string();
    match s.rfind(" (os error ") {
        Some(i) => s[..i].to_string(),
        None => s,
    }
}

/// `[Errno N] text`, the way Python prints an OSError.
pub fn os_error(errno: i32) -> String {
    format!("[Errno {errno}] {}", errno_text(errno))
}

fn serial_exception(msg: String) -> Failure {
    Failure::Uncaught(format!("serial.serialutil.SerialException: {msg}"))
}

/// The errno behind a serialport error, which keeps only the text: found by
/// the text, as nix and libc spell errors alike.
fn errno_of(err: &serialport::Error) -> Option<i32> {
    (1..134).find(|&n| errno_text(n) == err.description)
}

/// An open serial port.
pub struct SerialLink {
    port: TTYPort,
    clock: SystemClock,
}

impl SerialLink {
    /// `serial.Serial(port, baud, timeout=0.05)`, then `reset_input_buffer()`.
    pub fn open(path: &str, baud: u32) -> Result<SerialLink, Failure> {
        let could_not_open = |errno: i32| {
            serial_exception(format!(
                "[Errno {errno}] could not open port {path}: [Errno {errno}] {}: {}",
                errno_text(errno),
                crate::pyjson::str_repr(path)
            ))
        };
        if let Err(e) = std::fs::metadata(path) {
            if let Some(errno) = e.raw_os_error() {
                return Err(could_not_open(errno));
            }
        }
        let mut port = serialport::new(path, baud).timeout(Duration::from_millis(50)).open_native().map_err(|e| {
            match errno_of(&e) {
                // the flock someone else holds: the deliberate difference
                None | Some(libc::EAGAIN) => {
                    serial_exception(format!("could not open port {path} exclusively: {}", e.description))
                }
                // the open itself (EBUSY: another process holds TIOCEXCL), or termios
                Some(errno) => could_not_open(errno),
            }
        })?;
        // pyserial: DTR and RTS on unless flow control owns them; ptys refuse
        let _ = port.write_data_terminal_ready(true);
        let _ = port.write_request_to_send(true);
        // pyserial flushes in open(), mower_flash.py once more in main()
        let _ = port.clear(ClearBuffer::Input);
        let _ = port.clear(ClearBuffer::Input);
        Ok(SerialLink { port, clock: SystemClock::default() })
    }

    fn fd(&self) -> i32 {
        self.port.as_raw_fd()
    }

    /// select() on the port: Some(true) when readable (data, hang-up or
    /// error), Some(false) when the time ran out, None when a signal
    /// interrupted it (Python retries select, PEP 475).
    fn wait_readable(&self, seconds: f64) -> Result<Option<bool>, Failure> {
        let mut pfd = libc::pollfd { fd: self.fd(), events: libc::POLLIN, revents: 0 };
        #[cfg(target_os = "linux")]
        let n = {
            let ts = libc::timespec {
                tv_sec: seconds.trunc() as libc::time_t,
                tv_nsec: ((seconds.fract() * 1e9).ceil() as libc::c_long).min(999_999_999),
            };
            unsafe { libc::ppoll(&mut pfd, 1, &ts, std::ptr::null()) }
        };
        #[cfg(not(target_os = "linux"))]
        let n = unsafe { libc::poll(&mut pfd, 1, (seconds * 1000.0).ceil() as libc::c_int) };
        if n < 0 {
            let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
            if errno == libc::EINTR {
                return Ok(None);
            }
            return Err(serial_exception(format!("read failed: {}", os_error(errno))));
        }
        Ok(Some(n > 0 && pfd.revents != 0))
    }
}

impl Link for SerialLink {
    fn write(&mut self, data: &[u8]) -> Result<(), Failure> {
        let mut rest = data;
        while !rest.is_empty() {
            let n = unsafe { libc::write(self.fd(), rest.as_ptr() as *const libc::c_void, rest.len()) };
            if n < 0 {
                let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
                if matches!(errno, libc::EAGAIN | libc::EINTR | libc::EINPROGRESS | libc::EALREADY) {
                    continue;
                }
                return Err(serial_exception(format!("write failed: {}", os_error(errno))));
            }
            rest = &rest[n as usize..];
        }
        Ok(())
    }

    fn read(&mut self, max: usize, window: f64) -> Result<Vec<u8>, Failure> {
        // serialposix.py Serial.read: a Timeout(window), select() for what is
        // left of it, read what is there, stop at `max` bytes or at the end
        let deadline = self.clock.monotonic() + window;
        let mut out = Vec::new();
        let mut buf = [0u8; 256];
        while out.len() < max {
            let left = (deadline - self.clock.monotonic()).max(0.0);
            match self.wait_readable(left)? {
                None => continue,
                Some(false) => break,
                Some(true) => {}
            }
            let want = (max - out.len()).min(buf.len());
            let n = unsafe { libc::read(self.fd(), buf.as_mut_ptr() as *mut libc::c_void, want) };
            if n < 0 {
                let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
                if !matches!(errno, libc::EAGAIN | libc::EINTR | libc::EINPROGRESS | libc::EALREADY) {
                    return Err(serial_exception(format!("read failed: {}", os_error(errno))));
                }
            } else if n == 0 {
                return Err(serial_exception(
                    "device reports readiness to read but returned no data \
                     (device disconnected or multiple access on port?)"
                        .to_string(),
                ));
            } else {
                out.extend_from_slice(&buf[..n as usize]);
            }
            if self.clock.monotonic() >= deadline {
                break;
            }
        }
        Ok(out)
    }
}
