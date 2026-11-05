//! `mower_link` against `deploy/test/golden/host/g4_link_status.json`, what
//! `deploy/host/mower-link-status.py --once` did in each case: the same bytes in
//! the file and on stdout, what the modem heard, and the cases where Python
//! raised leaving the old file. mowerctl's `tests/link_status.rs` runs the same
//! table through the command line.

mod support;

use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Duration;

use mower_link::{once_text, write, Sources, AT_TIMEOUT, DEFAULT_AT_PORT, DEFAULT_PERIOD_S, DEFAULT_STATE_DIR, IP_TIMEOUT};
use serde_json::Value;
use sha2::{Digest, Sha256};
use support::*;

/// The cases that are about the command line, not the library.
fn command_line_case(given: &Value) -> bool {
    given.get("argv").is_some() || given.get("env").is_some()
}

fn run_one(case: &str, want: &Value) {
    let given = &want["given"];
    let mut s = setup("lib", case, given);
    let owner = give_state_dir_away(&s.state);
    let src = Sources {
        at_port: s.at_port.clone(),
        wireless: s.wireless.clone(),
        // the path of an `ip` that is not there is a failed spawn, as no `ip` on the PATH
        ip: s.ip.clone().unwrap_or_else(|| s.bin.join("ip").to_string_lossy().into_owned()),
        at_timeout: AT_TIMEOUT,
    };
    let (rc, stdout) = match write(&s.state_arg, &src, NOW) {
        Ok(doc) => (0, once_text(&doc) + "\n"),
        Err(_) => (1, String::new()),
    };
    let commands = s.modem.take().map(|m| m.finish()).unwrap_or_default();
    check(case, want, &Run { rc, stdout, commands, real_clock: false }, &s, owner);
    drop_scratch(&s.dir);
}

#[test]
fn every_case_does_what_python_did() {
    umask_022();
    let g = golden("g4_link_status.json");
    let cases: Vec<(&String, &Value)> =
        g["cases"].as_object().unwrap().iter().filter(|(_, v)| !command_line_case(&v["given"])).collect();
    assert!(cases.len() > 50, "the golden's table: {}", cases.len());
    // the slow ones wait for the modem; none of this is CPU
    let next = AtomicUsize::new(0);
    std::thread::scope(|sc| {
        for _ in 0..6 {
            sc.spawn(|| {
                while let Some((case, want)) = cases.get(next.fetch_add(1, Ordering::Relaxed)) {
                    run_one(case, want);
                }
            });
        }
    });
}

#[test]
fn the_defaults_are_the_tools() {
    let g = golden("g4_link_status.json");
    let d = &g["defaults"];
    assert_eq!(d["at_port"], DEFAULT_AT_PORT);
    assert_eq!(d["state_dir"], DEFAULT_STATE_DIR);
    assert_eq!(d["period_s"].as_f64().unwrap(), DEFAULT_PERIOD_S);
    assert_eq!(d["at_timeout_s"].as_f64().unwrap(), AT_TIMEOUT.as_secs_f64());
    let skip: Vec<&str> = d["skip_ifaces"].as_array().unwrap().iter().map(|v| v.as_str().unwrap()).collect();
    assert_eq!(skip, mower_link::parse::SKIP_IFACES);
    assert_eq!(IP_TIMEOUT, Duration::from_secs(3));
}

/// The port's settings, down to the bits, from a port left the way a freshly
/// enumerated modem's is: Python's own `configure_port` on one pty, this on another.
#[test]
fn the_port_is_set_up_like_pythons() {
    fn cooked_flow_controlled(fd: i32) {
        // SAFETY: termios is plain data, filled by tcgetattr.
        unsafe {
            let mut t: libc::termios = std::mem::zeroed();
            assert_eq!(libc::tcgetattr(fd, &mut t), 0);
            t.c_lflag |= libc::ICANON | libc::ECHO;
            t.c_cflag |= libc::CRTSCTS;
            assert_eq!(libc::tcsetattr(fd, libc::TCSANOW, &t), 0);
        }
    }
    let (master, slave, path) = open_pty();
    cooked_flow_controlled(slave);
    let port = mower_link::at::AtPort::open(&path, AT_TIMEOUT).unwrap();
    port.configure().unwrap();
    // SAFETY: as above.
    let t = unsafe {
        let mut t: libc::termios = std::mem::zeroed();
        assert_eq!(libc::tcgetattr(slave, &mut t), 0);
        t
    };
    assert_eq!(t.c_cc[libc::VMIN], 0);
    assert_eq!(t.c_cc[libc::VTIME], 0);
    assert_eq!(t.c_cflag & libc::CBAUD, libc::B115200);
    assert_eq!(t.c_cflag & libc::CRTSCTS, 0);
    assert_eq!(t.c_cflag & libc::CSIZE, libc::CS8);
    assert_eq!(t.c_lflag & (libc::ICANON | libc::ECHO | libc::ISIG), 0);
    assert_eq!(t.c_oflag & libc::OPOST, 0);
    assert_eq!(t.c_iflag & (libc::IXON | libc::ICRNL), 0);

    if !std::process::Command::new("python3").arg("--version").output().is_ok_and(|o| o.status.success()) {
        eprintln!("the comparison with Python skipped: no python3");
        return;
    }
    let tool = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../deploy/host/mower-link-status.py");
    let script = r#"
import importlib.util, json, os, sys, termios
spec = importlib.util.spec_from_file_location('tool', sys.argv[1])
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
master, slave = os.openpty()
a = termios.tcgetattr(slave)
a[3] |= termios.ICANON | termios.ECHO
a[2] |= termios.CRTSCTS
termios.tcsetattr(slave, termios.TCSANOW, a)
fd = os.open(os.ttyname(slave), os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
mod.configure_port(fd)
i, o, c, l, ispeed, ospeed, cc = termios.tcgetattr(slave)
print(json.dumps([i, o, c, l, ispeed, ospeed, cc[termios.VMIN], cc[termios.VTIME]]))
"#;
    let out = std::process::Command::new("python3")
        .env("PYTHONDONTWRITEBYTECODE", "1")
        .args(["-c", script])
        .arg(tool)
        .output()
        .unwrap();
    assert!(out.status.success(), "{out:?}");
    let py: Vec<u64> = serde_json::from_slice(&out.stdout).unwrap();
    let cc = |i: usize| match i {
        0 => u64::from(t.c_cc[libc::VMIN]),
        _ => u64::from(t.c_cc[libc::VTIME]),
    };
    assert_eq!(
        py,
        vec![
            u64::from(t.c_iflag),
            u64::from(t.c_oflag),
            u64::from(t.c_cflag),
            u64::from(t.c_lflag),
            u64::from(libc::B115200),
            u64::from(libc::B115200),
            cc(0),
            cc(1)
        ],
        "iflag oflag cflag lflag ispeed ospeed vmin vtime"
    );
    // SAFETY: both are this test's.
    unsafe {
        libc::close(master);
        libc::close(slave);
    }
}

fn digest(values: impl Iterator<Item = f64>) -> String {
    let joined: Vec<String> = values.map(mower_pyfmt::repr::float).collect();
    let bytes = Sha256::digest(joined.join("\n").as_bytes());
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

/// The numbers of +QCSQ and +CPSI through the parsers, over more than a modem can
/// say, printed by Python's repr, against the digest of what Python printed.
#[test]
fn the_number_sweeps_match_python() {
    use mower_link::parse;
    let g = golden("g4_link_status.json");
    let qcsq = digest((-2000..=2000).map(|i: i32| parse::qcsq(&format!("+QCSQ: \"LTE\",-60,-90,{i},-9\r\n")).unwrap().sinr_db));
    assert_eq!(g["sweeps"]["qcsq_sinr_-2000..=2000"], qcsq);
    let cpsi = (-2000..=2000).flat_map(|i: i32| {
        let c = parse::cpsi(&format!("+CPSI: LTE,Online,466-92,0x1,1,1,B,1,5,5,{i},{i},-1,{i}\r\n")).unwrap();
        [c.rsrq_db, c.rsrp_dbm, c.sinr_db]
    });
    assert_eq!(g["sweeps"]["cpsi_rsrq_rsrp_sinr_-2000..=2000"], digest(cpsi));
}

/// The parsers on the golden's own replies: a mutation of a pattern shows up
/// here by name, not as one document differing.
#[test]
fn the_parsers_read_the_goldens_replies() {
    use mower_link::parse;
    let g = golden("g4_link_status.json");
    let reply = |case: &str, cmd: &str| -> String {
        let bytes = latin1(g["cases"][case]["given"]["modem"][cmd][0].as_str().unwrap());
        String::from_utf8_lossy(&bytes).into_owned()
    };
    assert_eq!(parse::csq(&reply("ec25", "AT+CSQ")).unwrap().rssi_dbm, Some(-83));
    assert_eq!(parse::csq(&reply("csq-unknown", "AT+CSQ")).unwrap().rssi_dbm, None);
    assert_eq!(parse::creg(&reply("creg-roaming-long-form", "AT+CREG?")), Some(true));
    assert_eq!(parse::creg(&reply("creg-searching", "AT+CREG?")), Some(false));
    assert_eq!(parse::creg(&reply("creg-one-number", "AT+CREG?")), None);
    assert_eq!(parse::cops(&reply("ec25", "AT+COPS?")).unwrap().tech.as_deref(), Some("LTE"));
    assert_eq!(parse::cops(&reply("cops-unknown-tech", "AT+COPS?")).unwrap().tech.as_deref(), Some("13"));
    assert_eq!(parse::cops(&reply("cops-no-tech", "AT+COPS?")).unwrap().tech, None);
    assert_eq!(parse::qnwinfo(&reply("ec25", "AT+QNWINFO")).unwrap().band, "LTE BAND 7");
    let q = parse::qcsq(&reply("ec25", "AT+QCSQ")).unwrap();
    assert_eq!((q.rsrp_dbm, q.sinr_db, q.rsrq_db), (-90, 10.0, -9));
    let c = parse::cpsi(&reply("sim7600", "AT+CPSI?")).unwrap();
    assert_eq!((c.rsrp_dbm, c.rsrq_db, c.sinr_db), (-99.8, -10.3, 13.0));
}
