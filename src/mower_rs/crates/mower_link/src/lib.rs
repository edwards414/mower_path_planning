//! `<state_dir>/link_status.json`: what `deploy/host/mower-link-status.py`
//! samples and writes for the robot container, without Python (E6 design 3.4).
//!
//! `robot_status` reads the file (`/robot/telemetry.link`), and through that
//! Studio and the agent's heartbeat. The contract is the parsed document, its
//! keys and the type of every number (rsrp and rsrq are ints from Quectel's
//! `+QCSQ` and floats from SIMCom's `+CPSI`), the atomic replace and the file
//! given to the state dir's owner. This writes the same bytes as well, so a
//! difference shows up in a plain `cmp`; `tests/golden.rs` holds it to what the
//! Python tool did in each case of `deploy/test/golden/host/g4_link_status.json`.
//!
//! The entry point is `mowerctl link-status`; nothing runs it yet (E6 design D5:
//! the host keeps the Python tool until Phase F).

pub mod at;
pub mod parse;
pub mod py;
pub mod statedir;

use std::io::{Read, Write};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use mower_pyfmt::json::{self, JsonOptions};
use mower_pyfmt::Py;

use at::{AtPort, Modem};
use statedir::StateDir;

pub const DEFAULT_STATE_DIR: &str = "/home/cat/.mower";
pub const DEFAULT_AT_PORT: &str = "/dev/lte_at";
pub const DEFAULT_PERIOD_S: f64 = 5.0;
pub const WIRELESS: &str = "/proc/net/wireless";
pub const AT_TIMEOUT: Duration = Duration::from_millis(1500);
pub const IP_TIMEOUT: Duration = Duration::from_secs(3);

/// What made Python's `write()` raise: the sample ends, nothing is written, and
/// the file already there stays. The loop logs it and carries on.
#[derive(Debug, Clone, PartialEq)]
pub struct Raised(pub String);

impl std::fmt::Display for Raised {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

/// Where a sample comes from.
#[derive(Debug, Clone)]
pub struct Sources {
    pub at_port: String,
    pub wireless: String,
    pub ip: String,
    pub at_timeout: Duration,
}

impl Sources {
    /// `MOWER_LTE_AT`, as the Python tool; `MOWER_LINK_WIRELESS` only for tests,
    /// like `MOWER_CPUFREQ_DIR` in mower-update.sh.
    pub fn from_env() -> Sources {
        Sources {
            at_port: std::env::var("MOWER_LTE_AT").unwrap_or_else(|_| DEFAULT_AT_PORT.to_string()),
            wireless: std::env::var("MOWER_LINK_WIRELESS").unwrap_or_else(|_| WIRELESS.to_string()),
            ip: "ip".to_string(),
            at_timeout: AT_TIMEOUT,
        }
    }
}

/// `float(os.environ.get('MOWER_LINK_PERIOD_S', '5'))`
pub fn period_from_env() -> Result<f64, String> {
    match std::env::var("MOWER_LINK_PERIOD_S") {
        Err(_) => Ok(DEFAULT_PERIOD_S),
        Ok(v) => py::float(&v).ok_or_else(|| format!("could not convert string to float: {}", py::repr_str(&v))),
    }
}

/// The `lte` object: `present` alone without a modem, every key otherwise.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Lte {
    pub present: bool,
    pub rssi_dbm: Option<i64>,
    pub csq: Option<i64>,
    pub ber: Option<i64>,
    pub registered: Option<bool>,
    pub operator: Option<String>,
    pub tech: Option<String>,
    pub error: Option<String>,
    pub band: Option<String>,
    pub plmn: Option<String>,
    /// `Py::Int` from +QCSQ, `Py::Float` from +CPSI, `Py::None` without.
    pub rsrp_dbm: Option<Py>,
    pub sinr_db: Option<f64>,
    pub rsrq_db: Option<Py>,
}

impl Lte {
    pub fn to_py(&self) -> Py {
        if !self.present {
            return Py::dict([("present", Py::Bool(false))]);
        }
        Py::dict([
            ("present", Py::Bool(true)),
            ("rssi_dbm", self.rssi_dbm.into()),
            ("csq", self.csq.into()),
            ("ber", self.ber.into()),
            ("registered", self.registered.into()),
            ("operator", self.operator.clone().into()),
            ("tech", self.tech.clone().into()),
            ("error", self.error.clone().into()),
            ("band", self.band.clone().into()),
            ("plmn", self.plmn.clone().into()),
            ("rsrp_dbm", self.rsrp_dbm.clone().unwrap_or(Py::None)),
            ("sinr_db", self.sinr_db.into()),
            ("rsrq_db", self.rsrq_db.clone().unwrap_or(Py::None)),
        ])
    }
}

/// The commands, in their order; `+CPSI?` only if `+QCSQ` gave no rsrp. The
/// first failed write or read ends it, with what was read so far kept.
pub fn ask<M: Modem>(modem: &mut M, info: &mut Lte) -> Result<(), i32> {
    if let Some(m) = parse::csq(&modem.at("AT+CSQ")?) {
        info.csq = Some(m.csq);
        info.ber = Some(m.ber);
        info.rssi_dbm = m.rssi_dbm;
    }
    if let Some(registered) = parse::creg(&modem.at("AT+CREG?")?) {
        info.registered = Some(registered);
    }
    if let Some(m) = parse::cops(&modem.at("AT+COPS?")?) {
        info.operator = Some(m.operator);
        info.tech = m.tech;
    }
    if let Some(m) = parse::qnwinfo(&modem.at("AT+QNWINFO")?) {
        info.band = Some(m.band);
        info.plmn = Some(m.plmn);
    }
    if let Some(m) = parse::qcsq(&modem.at("AT+QCSQ")?) {
        info.rsrp_dbm = Some(Py::Int(m.rsrp_dbm));
        info.sinr_db = Some(m.sinr_db);
        info.rsrq_db = Some(Py::Int(m.rsrq_db));
    }
    if info.rsrp_dbm.is_none() {
        if let Some(m) = parse::cpsi(&modem.at("AT+CPSI?")?) {
            info.band = Some(m.band);
            info.plmn = Some(m.plmn);
            info.rsrq_db = Some(Py::Float(m.rsrq_db));
            info.rsrp_dbm = Some(Py::Float(m.rsrp_dbm));
            info.sinr_db = Some(m.sinr_db);
        }
    }
    Ok(())
}

/// `lte()`: no port, no modem; a port that does not open or fails mid-way, an
/// `error` beside what was read; a port that is no tty raises.
pub fn lte(port: &str, timeout: Duration) -> Result<Lte, Raised> {
    if std::fs::metadata(port).is_err() {
        return Ok(Lte::default());
    }
    let mut info = Lte { present: true, ..Lte::default() };
    let mut modem = match AtPort::open(port, timeout) {
        Ok(m) => m,
        Err(e) => {
            info.error = Some(py::os_error(e, Some(port)));
            return Ok(info);
        }
    };
    modem.configure()?;
    if let Err(e) = ask(&mut modem, &mut info) {
        info.error = Some(py::os_error(e, None));
    }
    Ok(info)
}

/// `wifi()`: a file that cannot be read is no Wi-Fi; one that is not UTF-8 raises.
pub fn wifi(path: &str) -> Result<Vec<parse::Wifi>, Raised> {
    let Ok(bytes) = std::fs::read(path) else {
        return Ok(Vec::new());
    };
    let text = std::str::from_utf8(&bytes).map_err(|e| Raised(e.to_string()))?;
    parse::wireless(text)
}

/// `ip -j -4 addr`'s stdout whatever its exit code; `None` if it does not start
/// or does not end within the timeout.
pub fn run_ip(program: &str, timeout: Duration) -> Option<Vec<u8>> {
    let mut child = Command::new(program)
        .args(["-j", "-4", "addr"])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .ok()?;
    let mut out = child.stdout.take()?;
    let mut err = child.stderr.take()?;
    let reader = std::thread::spawn(move || {
        let mut buf = Vec::new();
        let _ = out.read_to_end(&mut buf);
        buf
    });
    std::thread::spawn(move || {
        let _ = std::io::copy(&mut err, &mut std::io::sink());
    });
    let deadline = Instant::now() + timeout;
    loop {
        match child.try_wait() {
            Ok(Some(_)) => return reader.join().ok(),
            Ok(None) if Instant::now() < deadline => std::thread::sleep(Duration::from_millis(10)),
            _ => {
                let _ = child.kill();
                let _ = child.wait();
                return None;
            }
        }
    }
}

/// The whole document, sampled in Python's order. `now` is taken before the
/// sampling starts: `{'time': int(time.time()), 'lte': lte(), ...}`.
pub fn sample(src: &Sources, now: i64) -> Result<Py, Raised> {
    let lte = lte(&src.at_port, src.at_timeout)?;
    let wifi = wifi(&src.wireless)?;
    let ifaces = parse::ifaces(run_ip(&src.ip, IP_TIMEOUT).as_deref())?;
    Ok(Py::dict([
        ("time", Py::Int(now)),
        ("lte", lte.to_py()),
        (
            "wifi",
            Py::List(
                wifi.into_iter()
                    .map(|w| {
                        Py::dict([
                            ("iface", Py::Str(w.iface)),
                            ("link", Py::Float(w.link)),
                            ("rssi_dbm", Py::Float(w.rssi_dbm)),
                        ])
                    })
                    .collect(),
            ),
        ),
        (
            "ifaces",
            Py::List(
                ifaces
                    .into_iter()
                    .map(|i| Py::dict([("name", Py::Str(i.name)), ("up", Py::Bool(i.up)), ("addr", i.addr)]))
                    .collect(),
            ),
        ),
    ]))
}

/// `json.dumps(doc)`: what the file holds, no newline at the end.
pub fn file_text(doc: &Py) -> String {
    json::dumps(doc, &JsonOptions::default()).expect("a sample holds only JSON's types")
}

/// `json.dumps(doc, indent=1)`: what `--once` prints.
pub fn once_text(doc: &Py) -> String {
    json::dumps(doc, &JsonOptions { indent: Some(1), ..JsonOptions::default() }).expect("a sample holds only JSON's types")
}

/// `write()`: sample, write `link_status.json.tmp` and give it to the state dir's
/// owner (a chown that fails is not an error), rename it over `link_status.json`.
/// All through the open directory and the open file (statedir.rs). What the sample
/// or the write raises leaves the old file alone.
pub fn write(state_dir: &str, src: &Sources, now: i64) -> Result<Py, Raised> {
    const TMP: &str = "link_status.json.tmp";
    let doc = sample(src, now)?;
    let dir = StateDir::open(state_dir).map_err(|e| Raised(py::io_error(&e, state_dir)))?;
    let mut f = dir.create_fresh(TMP, 0o666).map_err(|e| Raised(py::io_error(&e, TMP)))?;
    f.write_all(file_text(&doc).as_bytes()).map_err(|e| Raised(py::io_error(&e, TMP)))?;
    if let Ok((uid, gid)) = dir.owner() {
        let _ = std::os::unix::fs::fchown(&f, Some(uid), Some(gid));
    }
    drop(f);
    dir.rename(TMP, "link_status.json").map_err(|e| Raised(py::io_error(&e, TMP)))?;
    Ok(doc)
}

pub fn now_unix() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs() as i64)
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Script(Vec<(&'static str, Result<&'static str, i32>)>);

    impl Modem for Script {
        fn at(&mut self, cmd: &str) -> Result<String, i32> {
            let (want, answer) = self.0.remove(0);
            assert_eq!(cmd, want);
            answer.map(str::to_string)
        }
    }

    #[test]
    fn a_failed_read_keeps_what_came_before_it() {
        let mut m = Script(vec![("AT+CSQ", Ok("+CSQ: 15,99\r\nOK")), ("AT+CREG?", Err(libc::EIO))]);
        let mut info = Lte { present: true, ..Lte::default() };
        assert_eq!(ask(&mut m, &mut info), Err(libc::EIO));
        assert_eq!((info.csq, info.rssi_dbm, info.registered), (Some(15), Some(-83), None));
    }

    #[test]
    fn cpsi_is_only_asked_when_qcsq_gave_no_rsrp() {
        let mut m = Script(vec![
            ("AT+CSQ", Ok("")),
            ("AT+CREG?", Ok("")),
            ("AT+COPS?", Ok("")),
            ("AT+QNWINFO", Ok("")),
            ("AT+QCSQ", Ok("+QCSQ: \"LTE\",-60,-90,150,-9")),
        ]);
        let mut info = Lte { present: true, ..Lte::default() };
        ask(&mut m, &mut info).unwrap();
        assert_eq!(info.rsrp_dbm, Some(Py::Int(-90)));
        assert!(m.0.is_empty());
    }

    #[test]
    fn no_modem_is_one_key() {
        assert_eq!(file_text(&Lte::default().to_py()), r#"{"present": false}"#);
    }
}
