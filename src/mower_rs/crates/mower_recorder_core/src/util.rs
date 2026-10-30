//! What several modules share: local time as Python's naive `datetime` sees
//! it, `isoformat`, `run_manifest._atomic_write`, `_dir_size`, file hashes.

use std::fs;
use std::io::{self, Read, Write};
use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};

/// A naive local time to the microsecond (`datetime.datetime.now()`), with
/// the zone's offset kept for the times that are written with one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LocalTime {
    pub year: i32,
    pub month: u32,
    pub day: u32,
    pub hour: u32,
    pub minute: u32,
    pub second: u32,
    pub microsecond: u32,
    /// Seconds east of UTC.
    pub utc_offset: i64,
}

impl LocalTime {
    /// The local time of `t` (`localtime_r`, the process's zone: UTC in the
    /// robot's containers).
    pub fn of(t: SystemTime) -> LocalTime {
        let d = t.duration_since(UNIX_EPOCH).unwrap_or_default();
        Self::from_unix(d.as_secs() as i64, d.subsec_micros())
    }

    pub fn now() -> LocalTime {
        Self::of(SystemTime::now())
    }

    pub fn from_unix(secs: i64, micros: u32) -> LocalTime {
        let mut tm: libc::tm = unsafe { std::mem::zeroed() };
        let t: libc::time_t = secs as libc::time_t;
        unsafe { libc::localtime_r(&t, &mut tm) };
        LocalTime {
            year: tm.tm_year + 1900,
            month: tm.tm_mon as u32 + 1,
            day: tm.tm_mday as u32,
            hour: tm.tm_hour as u32,
            minute: tm.tm_min as u32,
            second: tm.tm_sec as u32,
            microsecond: micros,
            utc_offset: tm.tm_gmtoff as i64,
        }
    }

    /// `strftime('%Y%m%dT%H%M%S')`, the run id's time.
    pub fn compact(&self) -> String {
        format!("{:04}{:02}{:02}T{:02}{:02}{:02}", self.year, self.month, self.day, self.hour, self.minute, self.second)
    }

    /// `isoformat()` of the naive datetime: `.ffffff` only when the
    /// microseconds are not 0.
    pub fn isoformat(&self) -> String {
        let mut s = format!(
            "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}",
            self.year, self.month, self.day, self.hour, self.minute, self.second
        );
        if self.microsecond != 0 {
            s.push_str(&format!(".{:06}", self.microsecond));
        }
        s
    }

    /// `isoformat(timespec='seconds')` of the aware datetime: `+HH:MM`.
    pub fn isoformat_seconds_with_offset(&self) -> String {
        format!(
            "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}{}",
            self.year, self.month, self.day, self.hour, self.minute, self.second,
            offset_text(self.utc_offset)
        )
    }
}

/// The wall time `secs` (Unix) shows at a fixed offset (seconds east).
pub fn civil(secs: i64, utc_offset: i64) -> LocalTime {
    let t = secs + utc_offset;
    let (days, rem) = (t.div_euclid(86_400), t.rem_euclid(86_400));
    // Howard Hinnant's days-to-civil
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let month = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let year = (yoe + era * 400 + if month <= 2 { 1 } else { 0 }) as i32;
    LocalTime {
        year,
        month,
        day,
        hour: (rem / 3600) as u32,
        minute: ((rem % 3600) / 60) as u32,
        second: (rem % 60) as u32,
        microsecond: 0,
        utc_offset,
    }
}

/// Python's `+HH:MM` (with `:SS` when the offset has seconds).
pub fn offset_text(off: i64) -> String {
    let sign = if off < 0 { '-' } else { '+' };
    let a = off.abs();
    let (h, m, s) = (a / 3600, (a % 3600) / 60, a % 60);
    if s != 0 {
        format!("{sign}{h:02}:{m:02}:{s:02}")
    } else {
        format!("{sign}{h:02}:{m:02}")
    }
}

/// `datetime.fromtimestamp(ns / 1e9, tz=utc).astimezone(tz)`'s whole
/// seconds: the float division, then the microseconds rounded half to even
/// (CPython `_PyTime_ROUND_HALF_EVEN`), which can carry into the second.
pub fn fromtimestamp_seconds(ns: i64) -> i64 {
    let t = ns as f64 / 1e9;
    let secs = t.floor();
    let frac = t - secs;
    let us = (frac * 1e6).round_ties_even();
    secs as i64 + if us >= 1e6 { 1 } else { 0 }
}

/// `tempfile.mkstemp(prefix='.tmp_', dir=...)`, write, fsync, `os.replace`.
pub fn atomic_write(path: &Path, data: &[u8]) -> io::Result<()> {
    let dir = path.parent().filter(|p| !p.as_os_str().is_empty()).unwrap_or(Path::new("."));
    let mut n: u64 = SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default().as_nanos() as u64;
    loop {
        let tmp = dir.join(format!(".tmp_{:x}{:x}", std::process::id(), n));
        match fs::OpenOptions::new().write(true).create_new(true).open(&tmp) {
            Ok(mut f) => {
                let r = f.write_all(data).and_then(|_| f.sync_all()).and_then(|_| fs::rename(&tmp, path));
                if r.is_err() {
                    let _ = fs::remove_file(&tmp);
                }
                return r;
            }
            Err(e) if e.kind() == io::ErrorKind::AlreadyExists => n = n.wrapping_add(1),
            Err(e) => return Err(e),
        }
    }
}

/// The sizes of every file under `path` (`os.walk` + `getsize`; errors
/// skipped, a missing dir is 0).
pub fn dir_size(path: &Path) -> u64 {
    let mut total = 0;
    let Ok(rd) = fs::read_dir(path) else { return 0 };
    for e in rd.flatten() {
        let p = e.path();
        match e.file_type() {
            Ok(t) if t.is_dir() => total += dir_size(&p),
            // getsize follows a symlink to a file
            Ok(_) => total += fs::metadata(&p).map(|m| m.len()).unwrap_or(0),
            Err(_) => {}
        }
    }
    total
}

pub fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

/// `run_manifest.sha256_file` (4 MiB reads).
pub fn sha256_file(path: &Path) -> io::Result<String> {
    use sha2::{Digest, Sha256};
    let mut f = fs::File::open(path)?;
    let mut h = Sha256::new();
    let mut buf = vec![0u8; 4 * 1024 * 1024];
    loop {
        let n = f.read(&mut buf)?;
        if n == 0 {
            break;
        }
        h.update(&buf[..n]);
    }
    Ok(hex(&h.finalize()))
}

pub fn sha256_hex(data: &[u8]) -> String {
    use sha2::{Digest, Sha256};
    hex(&Sha256::digest(data))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn isoformat_drops_zero_microseconds() {
        let t = LocalTime { year: 2026, month: 10, day: 27, hour: 2, minute: 15, second: 0, microsecond: 123456, utc_offset: 0 };
        assert_eq!(t.isoformat(), "2026-10-27T02:15:00.123456");
        assert_eq!(LocalTime { microsecond: 0, ..t }.isoformat(), "2026-10-27T02:15:00");
        assert_eq!(LocalTime { utc_offset: 8 * 3600, ..t }.isoformat_seconds_with_offset(), "2026-10-27T02:15:00+08:00");
        assert_eq!(offset_text(-(5 * 3600 + 30 * 60)), "-05:30");
        assert_eq!(t.compact(), "20261027T021500");
        let c = civil(1_793_348_331, 8 * 3600);
        assert_eq!(c.isoformat_seconds_with_offset(), "2026-10-30T16:18:51+08:00");
    }

    #[test]
    fn fromtimestamp_rounds_into_the_next_second() {
        assert_eq!(fromtimestamp_seconds(1_793_348_331_264_673_677), 1_793_348_331);
        assert_eq!(fromtimestamp_seconds(1_000_000_000_999_999_700), 1_000_000_001);
    }
}
