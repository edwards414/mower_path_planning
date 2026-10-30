//! A pseudo-terminal per device, its slave linked at the device's path
//! (fake_base.py's --pty-link): the driver opens the link like the udev
//! name of the real device and never knows the difference.
//!
//! The slave stays open here, raw from the start, so a driver that closes
//! and reopens the port (a restart) finds it as it left it and the master
//! never sees a hang-up. mower_sim only ever creates ptys and links: a path
//! that already exists and is anything but a link to a pty (a real device,
//! the robot's udev link to ttyS3) makes it refuse to start.

use std::ffi::CStr;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::path::{Path, PathBuf};

pub struct Pty {
    master: OwnedFd,
    _slave: OwnedFd,
    pub link: PathBuf,
    pub slave_name: String,
}

/// Whether `link` may be (re)made: nothing there, or a link to a pty (a
/// previous run's).
pub fn check_link(link: &Path) -> Result<(), String> {
    match std::fs::symlink_metadata(link) {
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(e) => Err(format!("{}: {e}", link.display())),
        Ok(m) if m.file_type().is_symlink() => {
            let target = std::fs::read_link(link).map_err(|e| format!("{}: {e}", link.display()))?;
            if target.starts_with("/dev/pts/") {
                Ok(())
            } else {
                Err(format!("{} -> {}: a real device's link; mower_sim never takes one over (is this the robot?)", link.display(), target.display()))
            }
        }
        Ok(_) => Err(format!("{} exists and is not a link (a real device?): mower_sim never takes one over", link.display())),
    }
}

impl Pty {
    /// A raw pty whose slave is linked at `link`; `owner` gets the slave
    /// (the stack's user, when mower_sim runs as root to write /dev).
    pub fn open(link: &Path, owner: Option<(u32, u32)>) -> Result<Pty, String> {
        check_link(link)?;
        let (mut m, mut s) = (-1, -1);
        // SAFETY: openpty writes two descriptors; the other pointers may be null.
        let r = unsafe { libc::openpty(&mut m, &mut s, std::ptr::null_mut(), std::ptr::null(), std::ptr::null()) };
        if r != 0 {
            return Err(format!("openpty: {}", std::io::Error::last_os_error()));
        }
        // SAFETY: both were just opened and are owned by nobody else.
        let (master, slave) = unsafe { (OwnedFd::from_raw_fd(m), OwnedFd::from_raw_fd(s)) };
        // SAFETY: plain termios calls on descriptors we own.
        unsafe {
            let mut t: libc::termios = std::mem::zeroed();
            if libc::tcgetattr(slave.as_raw_fd(), &mut t) != 0 {
                return Err(format!("tcgetattr: {}", std::io::Error::last_os_error()));
            }
            libc::cfmakeraw(&mut t);
            if libc::tcsetattr(slave.as_raw_fd(), libc::TCSANOW, &t) != 0 {
                return Err(format!("tcsetattr: {}", std::io::Error::last_os_error()));
            }
            let flags = libc::fcntl(master.as_raw_fd(), libc::F_GETFL);
            libc::fcntl(master.as_raw_fd(), libc::F_SETFL, flags | libc::O_NONBLOCK);
        }
        // SAFETY: ttyname returns a pointer to static storage or null.
        let name = unsafe {
            let p = libc::ttyname(slave.as_raw_fd());
            if p.is_null() {
                return Err(format!("ttyname: {}", std::io::Error::last_os_error()));
            }
            CStr::from_ptr(p).to_string_lossy().into_owned()
        };
        if let Some((uid, gid)) = owner {
            // SAFETY: fchown on a descriptor we own.
            if unsafe { libc::fchown(slave.as_raw_fd(), uid, gid) } != 0 {
                return Err(format!("chown {name} to {uid}:{gid}: {}", std::io::Error::last_os_error()));
            }
        }
        if std::fs::symlink_metadata(link).is_ok() {
            std::fs::remove_file(link).map_err(|e| format!("{}: {e}", link.display()))?;
        }
        std::os::unix::fs::symlink(&name, link).map_err(|e| format!("link {} -> {name}: {e}", link.display()))?;
        Ok(Pty { master, _slave: slave, link: link.to_path_buf(), slave_name: name })
    }

    pub fn fd(&self) -> i32 {
        self.master.as_raw_fd()
    }

    /// What the driver wrote, if anything (never blocks).
    pub fn read(&self, buf: &mut [u8]) -> usize {
        // SAFETY: read into a buffer we own, on a non-blocking descriptor.
        let n = unsafe { libc::read(self.master.as_raw_fd(), buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
        n.max(0) as usize
    }

    /// Write what fits. A port nobody reads fills its buffer, and a UART
    /// nobody listens to loses the bytes: the rest is dropped (returns the
    /// count written).
    pub fn write(&self, data: &[u8]) -> usize {
        let mut done = 0;
        while done < data.len() {
            // SAFETY: write from a slice we own, on a non-blocking descriptor.
            let n = unsafe { libc::write(self.master.as_raw_fd(), data[done..].as_ptr() as *const libc::c_void, data.len() - done) };
            if n <= 0 {
                break;
            }
            done += n as usize;
        }
        done
    }
}

impl Drop for Pty {
    fn drop(&mut self) {
        // only our own link
        if std::fs::read_link(&self.link).map(|t| t == Path::new(&self.slave_name)).unwrap_or(false) {
            let _ = std::fs::remove_file(&self.link);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{Read, Write};

    #[test]
    fn a_driver_opens_the_link_and_both_ends_talk() {
        let dir = std::env::temp_dir().join(format!("mower_sim_pty_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let link = dir.join("stmcom");
        let pty = Pty::open(&link, None).unwrap();
        assert!(pty.slave_name.starts_with("/dev/pts/") || pty.slave_name.starts_with("/dev/tty"), "{}", pty.slave_name);
        let mut port = std::fs::OpenOptions::new().read(true).write(true).open(&link).unwrap();
        port.write_all(b"\xA5\x5Ahost").unwrap();
        // The tty layer hands the bytes on from a work queue, which on a
        // loaded host can take longer than any fixed sleep.
        let mut buf = [0u8; 64];
        let mut n = 0;
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
        while n < 6 && std::time::Instant::now() < deadline {
            n += pty.read(&mut buf[n..]);
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        assert_eq!(&buf[..n], b"\xA5\x5Ahost", "raw: nothing translated or echoed");
        assert_eq!(pty.write(b"board\n\x0d"), 7);
        let mut got = [0u8; 7];
        port.read_exact(&mut got).unwrap();
        assert_eq!(&got, b"board\n\x0d");
        // a driver restart: the port closes and opens again
        drop(port);
        assert_eq!(pty.write(b"x"), 1, "no hang-up while the driver is away");
        assert_eq!(pty.read(&mut buf), 0);
        drop(pty);
        assert!(std::fs::symlink_metadata(&link).is_err(), "the link goes with the pty");
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn a_real_device_or_its_link_is_never_taken_over() {
        let dir = std::env::temp_dir().join(format!("mower_sim_pty_refuse_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("imu_usb");
        std::fs::write(&file, "").unwrap();
        assert!(Pty::open(&file, None).err().unwrap().contains("not a link"));
        let link = dir.join("stmcom");
        std::os::unix::fs::symlink("/dev/ttyS3", &link).unwrap();
        assert!(Pty::open(&link, None).err().unwrap().contains("is this the robot"));
        assert!(std::fs::read_link(&link).unwrap() == Path::new("/dev/ttyS3"), "left alone");
        // a stale link of a previous run is replaced
        let stale = dir.join("gps_rtk");
        std::os::unix::fs::symlink("/dev/pts/999", &stale).unwrap();
        let pty = Pty::open(&stale, None).unwrap();
        assert_eq!(std::fs::read_link(&stale).unwrap(), Path::new(&pty.slave_name));
        drop(pty);
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
