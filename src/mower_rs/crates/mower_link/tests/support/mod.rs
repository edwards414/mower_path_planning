//! Shared by `tests/golden.rs` here and mowerctl's `tests/link_status.rs`: one
//! case of `deploy/test/golden/host/g4_link_status.json` set up the way
//! `src/mower_mission/test/test_host_contracts.py` sets it up for Python: the
//! modem on a real pty, recorded wireless and ip output, a stale file to keep.

#![allow(dead_code)]

use std::collections::HashMap;
use std::os::fd::RawFd;
use std::os::unix::fs::PermissionsExt;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::Value;

pub const NOW: i64 = 1_790_000_000;
pub const STATE: &str = "/home/cat/.mower";
pub const AT_PORT: &str = "/dev/lte_at";
pub const STALE: &str = "{\"stale\": true}";
/// What a link in the state dir leads to.
pub const OTHER_FILE: &str = "{\"robot_id\": \"MW-OTHER\", \"secret\": \"OTHER\", \"stale\": false}\n";

pub fn golden(name: &str) -> Value {
    // mowerctl's manifest dir, which also compiles this, is as deep as this one's
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../deploy/test/golden/host").join(name);
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    serde_json::from_str(&text).expect("the golden is JSON")
}

/// Bytes kept in JSON as latin-1 characters.
pub fn latin1(s: &str) -> Vec<u8> {
    s.chars().map(|c| u8::try_from(c as u32).expect("a latin-1 character")).collect()
}

/// What Python's test fixes for every child: the modes are part of the contract.
pub fn umask_022() {
    // SAFETY: umask has no failure mode; the whole test process wants this one.
    unsafe {
        libc::umask(0o022);
    }
}

pub fn scratch(tag: &str, case: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("mower_link_{}_{tag}", std::process::id())).join(case);
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// A case's directory gone, and the run's parent too once it is the last.
pub fn drop_scratch(dir: &Path) {
    let _ = std::fs::remove_dir_all(dir);
    if let Some(parent) = dir.parent() {
        let _ = std::fs::remove_dir(parent);
    }
}

/// The modem side of a pty.
pub struct FakeModem {
    pub path: String,
    slave: RawFd,
    stop: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
    pub commands: Arc<Mutex<Vec<String>>>,
}

/// A pty pair that no child inherits (openpty's would: a sampler holding the
/// master itself would not see the modem unplugged): master, slave, slave's path.
pub fn open_pty() -> (RawFd, RawFd, String) {
    // SAFETY: flags and a NUL-terminated name; each call's result is checked.
    unsafe {
        let master = libc::posix_openpt(libc::O_RDWR | libc::O_NOCTTY | libc::O_CLOEXEC);
        assert!(master >= 0, "posix_openpt");
        assert_eq!(libc::grantpt(master), 0, "grantpt");
        assert_eq!(libc::unlockpt(master), 0, "unlockpt");
        let mut name = [0 as libc::c_char; 256];
        assert_eq!(libc::ptsname_r(master, name.as_mut_ptr(), name.len()), 0, "ptsname_r");
        let slave = libc::open(name.as_ptr(), libc::O_RDWR | libc::O_NOCTTY | libc::O_CLOEXEC);
        assert!(slave >= 0, "the slave opens");
        (master, slave, std::ffi::CStr::from_ptr(name.as_ptr()).to_string_lossy().into_owned())
    }
}

fn set_raw(fd: RawFd) {
    // SAFETY: termios is plain data, filled by tcgetattr.
    unsafe {
        let mut t: libc::termios = std::mem::zeroed();
        assert_eq!(libc::tcgetattr(fd, &mut t), 0);
        libc::cfmakeraw(&mut t);
        assert_eq!(libc::tcsetattr(fd, libc::TCSANOW, &t), 0);
    }
}

impl FakeModem {
    pub fn start(script: &Value, delay_ms: Option<&Value>, close_after: Option<&str>) -> FakeModem {
        let mut replies: HashMap<String, Vec<Vec<u8>>> = HashMap::new();
        for (cmd, pieces) in script.as_object().expect("a modem script") {
            replies.insert(cmd.clone(), pieces.as_array().unwrap().iter().map(|p| latin1(p.as_str().unwrap())).collect());
        }
        let delays: HashMap<String, u64> = delay_ms
            .and_then(Value::as_object)
            .map(|m| m.iter().map(|(k, v)| (k.clone(), v.as_u64().unwrap())).collect())
            .unwrap_or_default();
        let close_after = close_after.map(str::to_string);
        let (master, slave, path) = open_pty();
        set_raw(slave);
        let stop = Arc::new(AtomicBool::new(false));
        let commands = Arc::new(Mutex::new(Vec::new()));
        let (stop2, commands2) = (stop.clone(), commands.clone());
        let thread = std::thread::spawn(move || {
            serve(master, &replies, &delays, close_after.as_deref(), &stop2, &commands2);
            // SAFETY: the master is this thread's to close.
            unsafe {
                libc::close(master);
            }
        });
        FakeModem { path, slave, stop, thread: Some(thread), commands }
    }

    pub fn finish(mut self) -> Vec<String> {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            t.join().unwrap();
        }
        // SAFETY: the slave is this struct's to close.
        unsafe {
            libc::close(self.slave);
        }
        let v = self.commands.lock().unwrap().clone();
        v
    }
}

fn serve(
    master: RawFd,
    replies: &HashMap<String, Vec<Vec<u8>>>,
    delays: &HashMap<String, u64>,
    close_after: Option<&str>,
    stop: &AtomicBool,
    commands: &Mutex<Vec<String>>,
) {
    let mut buf: Vec<u8> = Vec::new();
    let mut due: Vec<(Instant, Vec<u8>)> = Vec::new();
    while !stop.load(Ordering::Relaxed) {
        let now = Instant::now();
        let mut i = 0;
        while i < due.len() {
            if due[i].0 <= now {
                let (_, bytes) = due.remove(i);
                // SAFETY: the buffer is valid for its length.
                let n = unsafe { libc::write(master, bytes.as_ptr().cast(), bytes.len()) };
                assert_eq!(n as usize, bytes.len(), "the modem's write");
            } else {
                i += 1;
            }
        }
        let mut p = libc::pollfd { fd: master, events: libc::POLLIN, revents: 0 };
        // SAFETY: one pollfd, valid for the call.
        if unsafe { libc::poll(&mut p, 1, 5) } <= 0 || p.revents & libc::POLLIN == 0 {
            continue;
        }
        let mut chunk = [0u8; 4096];
        // SAFETY: the buffer is valid for its length.
        let n = unsafe { libc::read(master, chunk.as_mut_ptr().cast(), chunk.len()) };
        if n <= 0 {
            continue;
        }
        buf.extend_from_slice(&chunk[..n as usize]);
        while let Some(at) = buf.windows(2).position(|w| w == b"\r\n") {
            let cmd: String = buf[..at].iter().map(|b| *b as char).collect();
            buf.drain(..at + 2);
            let mut log = commands.lock().unwrap();
            let unplug = close_after.is_some_and(|c| log.iter().any(|x| x == c));
            log.push(cmd.clone());
            drop(log);
            if unplug {
                return;
            }
            let at = Instant::now() + Duration::from_millis(delays.get(&cmd).copied().unwrap_or(0));
            if let Some(pieces) = replies.get(&cmd) {
                for (k, piece) in pieces.iter().enumerate() {
                    due.push((at + Duration::from_millis(200 * k as u64), piece.clone()));
                }
            }
        }
    }
}

/// One case's world: the state dir with its stale file, `ip`, the wireless
/// file, the AT port.
pub struct Setup {
    pub dir: PathBuf,
    pub state: PathBuf,
    pub state_arg: String,
    pub bin: PathBuf,
    pub ip: Option<String>,
    pub wireless: String,
    pub at_port: String,
    pub modem: Option<FakeModem>,
    pub other: PathBuf,
    pub elsewhere: PathBuf,
}

/// As in Python's run_link.
pub fn setup(tag: &str, case: &str, given: &Value) -> Setup {
    let dir = scratch(tag, case);
    let state = dir.join("state");
    let other = dir.join("other-file");
    std::fs::write(&other, OTHER_FILE).unwrap();
    std::fs::set_permissions(&other, std::fs::Permissions::from_mode(0o640)).unwrap();
    let elsewhere = dir.join("elsewhere");
    if given.get("state_is_link").and_then(Value::as_bool).unwrap_or(false) {
        std::fs::create_dir(&elsewhere).unwrap();
        std::os::unix::fs::symlink(&elsewhere, &state).unwrap();
    } else if !given.get("no_state_dir").and_then(Value::as_bool).unwrap_or(false) {
        std::fs::create_dir(&state).unwrap();
        std::fs::write(state.join("link_status.json"), STALE).unwrap();
    }
    if let Some(name) = given.get("link_at").and_then(Value::as_str) {
        let _ = std::fs::remove_file(state.join(name));
        std::os::unix::fs::symlink(&other, state.join(name)).unwrap();
    }
    if let Some(text) = given.get("tmp_before").and_then(Value::as_str) {
        std::fs::write(state.join("link_status.json.tmp"), text).unwrap();
    }
    let bin = dir.join("bin");
    std::fs::create_dir(&bin).unwrap();
    let default_ip = serde_json::json!({"stdout": "[]"});
    let ip_spec = match given.get("ip") {
        None => Some(&default_ip),
        Some(Value::Null) => None,
        Some(v) => Some(v),
    };
    let mut ip = None;
    if let Some(spec) = ip_spec {
        let tool = bin.join("ip");
        if spec.get("hang").and_then(Value::as_bool).unwrap_or(false) {
            std::fs::write(&tool, "#!/bin/sh\nexec /bin/sleep 30\n").unwrap();
        } else {
            let text = spec["stdout"].as_str().unwrap();
            let data = if spec.get("latin1").and_then(Value::as_bool).unwrap_or(false) { latin1(text) } else { text.as_bytes().to_vec() };
            let out = dir.join("ip.out");
            std::fs::write(&out, data).unwrap();
            let rc = spec.get("rc").and_then(Value::as_i64).unwrap_or(0);
            std::fs::write(
                &tool,
                format!("#!/bin/sh\n[ \"$*\" = \"-j -4 addr\" ] || exit 99\n/bin/cat '{}'\nexit {rc}\n", out.display()),
            )
            .unwrap();
        }
        std::fs::set_permissions(&tool, std::fs::Permissions::from_mode(0o755)).unwrap();
        ip = Some(tool.to_string_lossy().into_owned());
    }
    let wireless = dir.join("wireless");
    if let Some(text) = given.get("wireless").and_then(Value::as_str) {
        std::fs::write(&wireless, text).unwrap();
    }
    let mut at_port = dir.join("no-modem").to_string_lossy().into_owned();
    let mut modem = None;
    match given.get("at_port").and_then(Value::as_str) {
        Some("directory") => {
            let p = dir.join("portdir");
            std::fs::create_dir(&p).unwrap();
            at_port = p.to_string_lossy().into_owned();
        }
        Some("regular-file") => {
            let p = dir.join("portfile");
            std::fs::write(&p, "").unwrap();
            at_port = p.to_string_lossy().into_owned();
        }
        Some(other) => panic!("at_port {other}"),
        None => {
            if let Some(script) = given.get("modem") {
                let m = FakeModem::start(script, given.get("delay_ms"), given.get("close_after").and_then(Value::as_str));
                at_port = m.path.clone();
                modem = Some(m);
            }
        }
    }
    let suffix = given.get("state_arg_suffix").and_then(Value::as_str).unwrap_or("");
    let state_arg = format!("{}{suffix}", state.display());
    Setup { wireless: wireless.to_string_lossy().into_owned(), state_arg, state, bin, ip, at_port, modem, other, elsewhere, dir }
}

/// When the test runs as root (the dev container, CI's job): the state dir goes
/// to someone else, so the file's owner shows the chown happened. Python's
/// golden says it does; as anyone else there is nobody to give it to.
pub fn give_state_dir_away(state: &Path) -> Option<u32> {
    // SAFETY: geteuid has no failure mode.
    if unsafe { libc::geteuid() } != 0 || !state.is_dir() || state.is_symlink() {
        return None;
    }
    std::os::unix::fs::chown(state, Some(4321), Some(4321)).unwrap();
    Some(4321)
}

/// What Python's run_link records of the files, as golden shows it.
pub fn name(text: &str, at_port: &str, state: &Path) -> String {
    text.replace(at_port, AT_PORT).replace(&state.to_string_lossy().into_owned(), STATE)
}

/// The real clock's `"time": N` as the golden's: for a run of the real binary.
pub fn mask_time(text: &str) -> String {
    let Some(at) = text.find("\"time\": ") else {
        return text.to_string();
    };
    let digits = at + "\"time\": ".len();
    let end = text[digits..].find(|c: char| !c.is_ascii_digit()).map_or(text.len(), |i| digits + i);
    format!("{}{NOW}{}", &text[..digits], &text[end..])
}

/// Compare one run with its golden case.
pub struct Run {
    pub rc: i32,
    pub stdout: String,
    pub commands: Vec<String>,
    pub real_clock: bool,
}

pub fn check(case: &str, want: &Value, run: &Run, s: &Setup, owner: Option<u32>) {
    use std::os::unix::fs::MetadataExt;
    let target = s.state.join("link_status.json");
    assert_eq!(i64::from(run.rc), want["rc"].as_i64().unwrap(), "{case}: rc; stdout {:?}", run.stdout);
    let clock = |t: String| if run.real_clock { mask_time(&t) } else { t };
    assert_eq!(clock(name(&run.stdout, &s.at_port, &s.state)), want["stdout"].as_str().unwrap(), "{case}: stdout");
    let got_commands: Vec<&str> = run.commands.iter().map(String::as_str).collect();
    let want_commands: Vec<&str> = want["commands"].as_array().unwrap().iter().map(|c| c.as_str().unwrap()).collect();
    assert_eq!(got_commands, want_commands, "{case}: the modem heard");
    let text = std::fs::read(&target).ok().map(|b| String::from_utf8(b).expect("ASCII"));
    let text = text.map(|t| clock(name(&t, &s.at_port, &s.state)));
    assert_eq!(text.as_deref(), want["file"].as_str(), "{case}: the file");
    if let Some(mode) = want["mode"].as_u64() {
        let st = std::fs::metadata(&target).unwrap();
        assert_eq!(u64::from(st.mode() & 0o7777), mode, "{case}: its mode");
    }
    assert_eq!(s.state.join("link_status.json.tmp").exists(), want["tmp_left"].as_bool().unwrap(), "{case}: the tmp");
    let st = std::fs::metadata(&s.other).unwrap();
    assert_eq!(std::fs::read_to_string(&s.other).unwrap(), OTHER_FILE, "{case}: what a link leads to");
    assert_eq!(st.mode() & 0o7777, 0o640, "{case}: its mode");
    // SAFETY: geteuid has no failure mode.
    assert_eq!(st.uid(), unsafe { libc::geteuid() }, "{case}: its owner");
    if let Some(empty) = want["elsewhere_empty"].as_bool() {
        assert_eq!(std::fs::read_dir(&s.elsewhere).unwrap().next().is_none(), empty, "{case}: where a link leads");
    }
    let chowned = !want["chown_to_the_dir_owner"].as_array().unwrap().is_empty();
    if let (true, Some(uid)) = (chowned, owner) {
        let st = std::fs::metadata(&target).unwrap();
        assert_eq!((st.uid(), st.gid()), (uid, uid), "{case}: its owner");
    }
}
