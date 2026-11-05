//! The state dir as a tool running as root has to use it: it belongs to the robot
//! user and the container writes it, so a name there can change at any moment.
//! Everything goes through the open directory and the open file: no link is
//! followed, at the directory's own name or in it, and nothing is done by a name
//! after the name could have changed. `mower-link-status.py` and `mower-pair` do
//! the same; `deploy/test/golden/host/` holds both to it.

use std::ffi::CString;
use std::fs::File;
use std::io;
use std::os::fd::{AsRawFd, FromRawFd};
use std::os::unix::fs::{MetadataExt, OpenOptionsExt};

pub struct StateDir {
    dir: File,
}

fn c_name(name: &str) -> io::Result<CString> {
    CString::new(name).map_err(|_| io::Error::from_raw_os_error(libc::EINVAL))
}

fn checked(r: libc::c_int) -> io::Result<libc::c_int> {
    if r < 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(r)
    }
}

impl StateDir {
    /// The directory itself. A trailing slash would make the kernel follow a link
    /// whatever the flags say, so it goes; an empty path is the current directory.
    pub fn open(path: &str) -> io::Result<StateDir> {
        let trimmed = path.trim_end_matches('/');
        let target = match (trimmed.is_empty(), path.is_empty()) {
            (false, _) => trimmed,
            (true, true) => ".",
            (true, false) => "/",
        };
        let dir = std::fs::OpenOptions::new().read(true).custom_flags(libc::O_DIRECTORY | libc::O_NOFOLLOW).open(target)?;
        Ok(StateDir { dir })
    }

    /// Its owner's uid and gid.
    pub fn owner(&self) -> io::Result<(u32, u32)> {
        let st = self.dir.metadata()?;
        Ok((st.uid(), st.gid()))
    }

    fn at(&self, name: &str, flags: libc::c_int, mode: libc::mode_t) -> io::Result<File> {
        let c = c_name(name)?;
        // SAFETY: a NUL-terminated name relative to an open directory; the
        // descriptor returned is owned by the File from here on.
        let fd = checked(unsafe { libc::openat(self.dir.as_raw_fd(), c.as_ptr(), flags | libc::O_NOFOLLOW | libc::O_CLOEXEC, libc::c_uint::from(mode)) })?;
        // SAFETY: fd was just opened and nothing else owns it.
        Ok(unsafe { File::from_raw_fd(fd) })
    }

    /// A file of that name for reading, if it is not a link, without waiting on
    /// something that is not a file.
    pub fn open_read(&self, name: &str) -> io::Result<File> {
        self.at(name, libc::O_RDONLY | libc::O_NONBLOCK, 0)
    }

    /// A new file of that name: what was there is gone first, whatever it was
    /// (a link goes, not what it leads to).
    pub fn create_fresh(&self, name: &str, mode: u32) -> io::Result<File> {
        let c = c_name(name)?;
        // SAFETY: a NUL-terminated name relative to an open directory.
        match checked(unsafe { libc::unlinkat(self.dir.as_raw_fd(), c.as_ptr(), 0) }) {
            Err(e) if e.raw_os_error() != Some(libc::ENOENT) => return Err(e),
            _ => {}
        }
        self.create_new(name, mode)
    }

    /// A file that was not there: if the name is taken, this fails rather than
    /// open what it is.
    pub fn create_new(&self, name: &str, mode: u32) -> io::Result<File> {
        self.at(name, libc::O_WRONLY | libc::O_CREAT | libc::O_EXCL, mode as libc::mode_t)
    }

    /// `from` becomes `to`, over what is there; links are renamed, not followed.
    pub fn rename(&self, from: &str, to: &str) -> io::Result<()> {
        let (a, b) = (c_name(from)?, c_name(to)?);
        let d = self.dir.as_raw_fd();
        // SAFETY: two NUL-terminated names relative to an open directory.
        checked(unsafe { libc::renameat(d, a.as_ptr(), d, b.as_ptr()) }).map(|_| ())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{Read, Write};
    use std::os::unix::fs::PermissionsExt;
    use std::path::PathBuf;

    fn scratch(tag: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("mower_link_statedir_{}_{tag}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(d.join("state")).unwrap();
        std::fs::write(d.join("other"), "other").unwrap();
        std::fs::set_permissions(d.join("other"), std::fs::Permissions::from_mode(0o640)).unwrap();
        d
    }

    fn mode(p: &std::path::Path) -> u32 {
        std::fs::metadata(p).unwrap().mode() & 0o7777
    }

    #[test]
    fn a_link_at_the_directorys_name_is_refused_with_or_without_a_slash() {
        let d = scratch("dirlink");
        std::os::unix::fs::symlink(d.join("state"), d.join("link")).unwrap();
        let link = d.join("link").to_string_lossy().into_owned();
        assert!(StateDir::open(&link).is_err());
        assert!(StateDir::open(&format!("{link}/")).is_err());
        assert!(StateDir::open(&d.join("state").to_string_lossy()).is_ok());
        assert!(StateDir::open(&format!("{}/", d.join("state").display())).is_ok());
        assert!(StateDir::open(&d.join("other").to_string_lossy()).is_err());
        assert!(StateDir::open("").is_ok());
        assert!(StateDir::open("/").is_ok());
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_link_at_a_name_is_replaced_and_what_it_leads_to_stays() {
        let d = scratch("namelink");
        let state = StateDir::open(&d.join("state").to_string_lossy()).unwrap();
        std::os::unix::fs::symlink(d.join("other"), d.join("state/tmp")).unwrap();
        let mut f = state.create_fresh("tmp", 0o666).unwrap();
        f.write_all(b"new").unwrap();
        drop(f);
        assert_eq!(std::fs::read_to_string(d.join("other")).unwrap(), "other");
        assert!(!d.join("state/tmp").is_symlink());
        std::os::unix::fs::symlink(d.join("other"), d.join("state/final")).unwrap();
        state.rename("tmp", "final").unwrap();
        assert!(!d.join("state/final").is_symlink());
        assert_eq!(std::fs::read_to_string(d.join("state/final")).unwrap(), "new");
        assert_eq!(std::fs::read_to_string(d.join("other")).unwrap(), "other");
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_name_taken_again_is_not_opened() {
        let d = scratch("taken");
        let state = StateDir::open(&d.join("state").to_string_lossy()).unwrap();
        std::os::unix::fs::symlink(d.join("other"), d.join("state/tmp")).unwrap();
        let e = state.create_new("tmp", 0o666).err().expect("the name is taken");
        assert_eq!(e.kind(), io::ErrorKind::AlreadyExists);
        assert_eq!(std::fs::read_to_string(d.join("other")).unwrap(), "other");
        let _ = std::fs::remove_dir_all(&d);
    }

    /// What the name turns into while the file is open does not matter.
    #[test]
    fn a_name_that_changes_does_not_redirect_what_is_done_on_the_open_file() {
        let d = scratch("swap");
        let state = StateDir::open(&d.join("state").to_string_lossy()).unwrap();
        let f = state.create_fresh("tmp", 0o600).unwrap();
        std::fs::remove_file(d.join("state/tmp")).unwrap();
        std::os::unix::fs::symlink(d.join("other"), d.join("state/tmp")).unwrap();
        f.set_permissions(std::fs::Permissions::from_mode(0o644)).unwrap();
        // SAFETY: geteuid has no failure mode.
        let (uid, gid) = unsafe { (libc::geteuid(), libc::getegid()) };
        std::os::unix::fs::fchown(&f, Some(uid), Some(gid)).unwrap();
        state.rename("tmp", "final").unwrap();
        assert_eq!(mode(&d.join("other")), 0o640);
        assert_eq!(std::fs::read_to_string(d.join("other")).unwrap(), "other");
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_link_is_not_read_through_and_a_fifo_does_not_wait() {
        let d = scratch("read");
        let state = StateDir::open(&d.join("state").to_string_lossy()).unwrap();
        std::os::unix::fs::symlink(d.join("other"), d.join("state/lnk")).unwrap();
        assert!(state.open_read("lnk").is_err());
        let fifo = std::ffi::CString::new(d.join("state/fifo").to_string_lossy().as_bytes()).unwrap();
        // SAFETY: a NUL-terminated path.
        assert_eq!(unsafe { libc::mkfifo(fifo.as_ptr(), 0o600) }, 0);
        let mut text = String::new();
        let mut f = state.open_read("fifo").unwrap();
        let _ = f.read_to_string(&mut text);
        assert!(text.is_empty());
        std::fs::write(d.join("state/plain"), "x").unwrap();
        let mut text = String::new();
        state.open_read("plain").unwrap().read_to_string(&mut text).unwrap();
        assert_eq!(text, "x");
        let _ = std::fs::remove_dir_all(&d);
    }
}
