//! A pseudo-terminal per device, its slave linked at the device's path
//! (fake_base.py's --pty-link): the driver opens the link like the udev
//! name of the real device and never knows the difference.
//!
//! The slave stays open here, raw from the start, so a driver that closes
//! and reopens the port (a restart) finds it as it left it and the master
//! never sees a hang-up. mower_sim only ever creates ptys and links: a path
//! that already exists and is anything but a link to a pty (a real device,
//! the robot's udev link to ttyS3) makes it refuse to start.
//!
//! A `Plug` is a device that can be unplugged (a USB device behind the hub):
//! unplugging drops its pty, which hangs the slave up under the driver (its
//! reads see EOF or POLLHUP, its ioctls EIO, as on a USB serial device that
//! went away) and removes the link, so opening the path fails as it does
//! once udev has removed the node; plugging it back in makes a fresh pty and
//! links it again.

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
        // ttyname_r, not ttyname: ttyname's buffer is shared by every thread,
        // so two ptys opened at once (tests run in parallel) could each be
        // handed the other's name and link the other's slave.
        let mut buf = [0 as libc::c_char; 128];
        // SAFETY: ttyname_r writes a NUL-terminated name into the buffer it is given.
        let name = unsafe {
            let r = libc::ttyname_r(slave.as_raw_fd(), buf.as_mut_ptr(), buf.len());
            if r != 0 {
                return Err(format!("ttyname_r: {}", std::io::Error::from_raw_os_error(r)));
            }
            CStr::from_ptr(buf.as_ptr()).to_string_lossy().into_owned()
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

/// A device behind the USB hub: its pty while plugged in, nothing at all
/// (no pty, no link) while out.
pub struct Plug {
    link: PathBuf,
    owner: Option<(u32, u32)>,
    pty: Option<Pty>,
    /// the last failure to plug back in, said once
    failed: Option<String>,
}

impl Plug {
    pub fn open(link: &Path, owner: Option<(u32, u32)>) -> Result<Plug, String> {
        Ok(Plug { link: link.to_path_buf(), owner, pty: Some(Pty::open(link, owner)?), failed: None })
    }

    pub fn pty(&self) -> Option<&Pty> {
        self.pty.as_ref()
    }

    /// The master's descriptor for poll(2); -1 (ignored there) while out.
    pub fn fd(&self) -> i32 {
        self.pty.as_ref().map_or(-1, Pty::fd)
    }

    /// Unplug or plug back in. `Ok(true)` when it changed; an error, once
    /// per distinct one, when a fresh pty could not be made (tried again on
    /// every call).
    pub fn set(&mut self, plugged: bool) -> Result<bool, String> {
        if plugged == self.pty.is_some() {
            return Ok(false);
        }
        if !plugged {
            self.pty = None;
            return Ok(true);
        }
        match Pty::open(&self.link, self.owner) {
            Ok(p) => {
                self.pty = Some(p);
                self.failed = None;
                Ok(true)
            }
            Err(e) if self.failed.as_deref() == Some(&e) => Ok(false),
            Err(e) => {
                self.failed = Some(e.clone());
                Err(e)
            }
        }
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

    /// Ptys opened on many threads at once each link their own slave: the
    /// link's device is the slave descriptor's (ttyname's shared buffer could
    /// hand one thread another's name).
    #[test]
    fn ptys_opened_at_once_each_link_their_own_slave() {
        use std::os::unix::fs::MetadataExt;
        let dir = std::env::temp_dir().join(format!("mower_sim_pty_many_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let threads: Vec<_> = (0..16)
            .map(|t| {
                let dir = dir.clone();
                std::thread::spawn(move || {
                    for i in 0..20 {
                        let link = dir.join(format!("dev{t}_{i}"));
                        let pty = Pty::open(&link, None).unwrap();
                        // SAFETY: fstat into a struct we own, on a descriptor the pty owns.
                        let mut st: libc::stat = unsafe { std::mem::zeroed() };
                        assert_eq!(unsafe { libc::fstat(pty._slave.as_raw_fd(), &mut st) }, 0);
                        let linked = std::fs::metadata(&link).unwrap().rdev();
                        assert_eq!(linked, st.st_rdev as u64, "{} links another pty", link.display());
                    }
                })
            })
            .collect();
        for t in threads {
            t.join().unwrap();
        }
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

    /// Unplugged, the driver's open port reads EOF, its writes and ioctls
    /// fail, and the path is gone; plugged back in, a fresh pty answers on
    /// the same path. serialport (the drivers' library) sees the hang-up as
    /// an error both ways mower_gps (read) and mower_imu (bytes_to_read)
    /// look at the port.
    #[test]
    fn an_unplugged_device_hangs_up_under_the_driver_and_comes_back_fresh() {
        let dir = std::env::temp_dir().join(format!("mower_sim_pty_plug_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let link = dir.join("gps_rtk");
        let mut dev = Plug::open(&link, None).unwrap();
        let first = dev.pty().unwrap().slave_name.clone();
        let mut port = std::fs::OpenOptions::new().read(true).write(true).open(&link).unwrap();
        let mut sp = serialport::new(link.to_str().unwrap(), 38_400).timeout(std::time::Duration::from_millis(200)).open().unwrap();
        assert_eq!(dev.pty().unwrap().write(b"fix"), 3);
        let mut got = [0u8; 3];
        port.read_exact(&mut got).unwrap();
        assert_eq!(&got, b"fix");

        assert_eq!(dev.set(false), Ok(true));
        assert!(dev.pty().is_none() && dev.fd() == -1);
        assert_eq!(dev.set(false), Ok(false), "already out");
        let mut buf = [0u8; 16];
        match port.read(&mut buf) {
            Ok(0) => {}
            Err(e) if e.raw_os_error() == Some(libc::EIO) => {}
            other => panic!("the driver's read after the unplug: {other:?}"),
        }
        assert!(port.write_all(b"cfg").is_err(), "a write to a device that went away fails");
        let e = sp.read(&mut buf).unwrap_err();
        assert_eq!(e.kind(), std::io::ErrorKind::BrokenPipe, "mower_gps's read: {e}");
        assert!(sp.bytes_to_read().is_err(), "mower_imu's bytes_to_read");
        assert!(std::fs::symlink_metadata(&link).is_err(), "the path is gone");
        let reopen = std::fs::OpenOptions::new().read(true).write(true).open(&link).unwrap_err();
        assert_eq!(reopen.kind(), std::io::ErrorKind::NotFound);

        assert_eq!(dev.set(true), Ok(true));
        assert_eq!(dev.set(true), Ok(false), "already in");
        let second = dev.pty().unwrap().slave_name.clone();
        assert_ne!(first, second, "a fresh pty (the old one is still held open by the driver)");
        assert_eq!(std::fs::read_link(&link).unwrap(), Path::new(&second));
        // the old descriptor stays hung up; a new open talks to the new pty
        assert!(matches!(port.read(&mut buf), Ok(0) | Err(_)));
        drop((port, sp));
        let mut sp = serialport::new(link.to_str().unwrap(), 38_400).timeout(std::time::Duration::from_secs(2)).open().unwrap();
        assert_eq!(dev.pty().unwrap().write(b"again"), 5);
        let mut got = [0u8; 5];
        sp.read_exact(&mut got).unwrap();
        assert_eq!(&got, b"again");
        sp.write_all(b"\xB5\x62").unwrap();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
        let mut n = 0;
        while n < 2 && std::time::Instant::now() < deadline {
            n += dev.pty().unwrap().read(&mut buf[n..]);
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        assert_eq!(&buf[..n], b"\xB5\x62");
        drop(sp);
        drop(dev);
        assert!(std::fs::symlink_metadata(&link).is_err(), "the link goes with the device");

        // a path taken over while out (a real device) is never replaced:
        // the error is said once and tried again
        let mut dev = Plug::open(&link, None).unwrap();
        dev.set(false).unwrap();
        std::fs::write(&link, "").unwrap();
        assert!(dev.set(true).unwrap_err().contains("not a link"));
        assert_eq!(dev.set(true), Ok(false), "said once");
        std::fs::remove_file(&link).unwrap();
        assert_eq!(dev.set(true), Ok(true));
        drop(dev);
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
