//! The modem's AT port, the way `deploy/host/mower-link-status.py` drives it:
//! raw 8N1 at 115200, non-blocking, one command at a time, reading until the
//! bytes `OK` or `ERROR` turn up anywhere or 1.5 s pass, whichever comes first.

use std::ffi::CString;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::time::{Duration, Instant};

use crate::Raised;

/// One command's answer, or the errno that ended the session.
pub trait Modem {
    fn at(&mut self, cmd: &str) -> Result<String, i32>;
}

pub struct AtPort {
    fd: OwnedFd,
    timeout: Duration,
}

fn errno() -> i32 {
    std::io::Error::last_os_error().raw_os_error().unwrap_or(libc::EIO)
}

fn termios_error() -> Raised {
    let e = errno();
    Raised(format!("({e}, {})", crate::py::repr_str(&crate::py::strerror(e))))
}

/// What `cfsetispeed` + `cfsetospeed` do, in the bits the kernel reads. On Linux
/// not by those functions: the libc crate binds them to versioned glibc symbols
/// (`cfsetispeed@GLIBC_2.17`), which a static binary cannot link.
#[cfg(target_os = "linux")]
fn set_115200(t: &mut libc::termios) {
    t.c_cflag = (t.c_cflag & !(libc::CBAUD | libc::CBAUDEX)) | libc::B115200;
}

#[cfg(not(target_os = "linux"))]
fn set_115200(t: &mut libc::termios) {
    // SAFETY: t is a valid termios; neither fails for a constant the system has.
    unsafe {
        libc::cfsetispeed(t, libc::B115200);
        libc::cfsetospeed(t, libc::B115200);
    }
}

impl AtPort {
    /// `os.open(path, O_RDWR | O_NOCTTY | O_NONBLOCK)`: the errno on failure.
    pub fn open(path: &str, timeout: Duration) -> Result<AtPort, i32> {
        let c = CString::new(path).map_err(|_| libc::EINVAL)?;
        // SAFETY: a NUL-terminated path; the descriptor is owned from here on.
        let fd = unsafe { libc::open(c.as_ptr(), libc::O_RDWR | libc::O_NOCTTY | libc::O_NONBLOCK | libc::O_CLOEXEC) };
        if fd < 0 {
            return Err(errno());
        }
        // SAFETY: fd was just opened and nothing else owns it.
        Ok(AtPort { fd: unsafe { OwnedFd::from_raw_fd(fd) }, timeout })
    }

    /// `configure_port`, then `tcflush(TCIOFLUSH)`. A freshly enumerated modem
    /// tty comes up cooked at 9600 baud with CRTSCTS set, on which a
    /// non-blocking write fails with EAGAIN before the modem sees a byte
    /// (SIM7600G-H, 2026-10-07). In Python `termios.error` is not an `OSError`:
    /// it ends the whole sample, and this does too.
    pub fn configure(&self) -> Result<(), Raised> {
        let fd = self.fd.as_raw_fd();
        // SAFETY: termios is plain data; tcgetattr fills it before it is read.
        unsafe {
            let mut t: libc::termios = std::mem::zeroed();
            if libc::tcgetattr(fd, &mut t) != 0 {
                return Err(termios_error());
            }
            t.c_iflag &= !(libc::IGNBRK
                | libc::BRKINT
                | libc::PARMRK
                | libc::ISTRIP
                | libc::INLCR
                | libc::IGNCR
                | libc::ICRNL
                | libc::IXON
                | libc::IXOFF
                | libc::IXANY);
            t.c_oflag &= !libc::OPOST;
            t.c_lflag &= !(libc::ECHO | libc::ECHONL | libc::ICANON | libc::ISIG | libc::IEXTEN);
            t.c_cflag &= !(libc::CSIZE | libc::PARENB | libc::CSTOPB | libc::CRTSCTS);
            t.c_cflag |= libc::CS8 | libc::CREAD | libc::CLOCAL;
            t.c_cc[libc::VMIN] = 0;
            t.c_cc[libc::VTIME] = 0;
            set_115200(&mut t);
            if libc::tcsetattr(fd, libc::TCSANOW, &t) != 0 {
                return Err(termios_error());
            }
            if libc::tcflush(fd, libc::TCIOFLUSH) != 0 {
                return Err(termios_error());
            }
        }
        Ok(())
    }

    fn write(&self, data: &[u8]) -> Result<(), i32> {
        loop {
            // SAFETY: the buffer is valid for its length. One write, as Python's
            // os.write: a short write is not retried, EINTR is (PEP 475).
            let n = unsafe { libc::write(self.fd.as_raw_fd(), data.as_ptr().cast(), data.len()) };
            if n >= 0 {
                return Ok(());
            }
            match errno() {
                libc::EINTR => continue,
                e => return Err(e),
            }
        }
    }
}

fn contains(hay: &[u8], needle: &[u8]) -> bool {
    hay.windows(needle.len()).any(|w| w == needle)
}

impl Modem for AtPort {
    fn at(&mut self, cmd: &str) -> Result<String, i32> {
        self.write(format!("{cmd}\r\n").as_bytes())?;
        let fd = self.fd.as_raw_fd();
        let mut out = Vec::new();
        let deadline = Instant::now() + self.timeout;
        while let Some(left) = deadline.checked_duration_since(Instant::now()).filter(|d| !d.is_zero()) {
            let mut p = libc::pollfd { fd, events: libc::POLLIN, revents: 0 };
            let ms = left.as_millis().saturating_add(1).min(i32::MAX as u128) as i32;
            // SAFETY: one pollfd, valid for the call.
            let r = unsafe { libc::poll(&mut p, 1, ms) };
            if r < 0 {
                match errno() {
                    libc::EINTR => continue,
                    e => return Err(e),
                }
            }
            if r == 0 {
                break;
            }
            let mut buf = [0u8; 1024];
            // SAFETY: the buffer is valid for its length.
            let n = unsafe { libc::read(fd, buf.as_mut_ptr().cast(), buf.len()) };
            if n < 0 {
                match errno() {
                    libc::EINTR => continue,
                    e => return Err(e),
                }
            }
            out.extend_from_slice(&buf[..n as usize]);
            if contains(&out, b"OK") || contains(&out, b"ERROR") {
                break;
            }
        }
        Ok(String::from_utf8_lossy(&out).into_owned())
    }
}
