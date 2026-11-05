//! The real binary's `pair` and `link-status` against the goldens of the Python
//! tools (`deploy/test/golden/host/`), on what can be fixed from outside a
//! process, and side by side with those tools where python3 is there (CI's job
//! and the dev container): what depends on this machine has to come out the same.

#[path = "../../mower_link/tests/support/mod.rs"]
mod support;

use std::collections::HashSet;
use std::os::unix::fs::{MetadataExt, PermissionsExt};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serde_json::Value;
use support::*;

const BIN: &str = env!("CARGO_BIN_EXE_mowerctl");

fn repo() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../..").canonicalize().unwrap()
}

// ---- link-status -------------------------------------------------------------

fn link_case(case: &str, want: &Value) {
    let given = &want["given"];
    let mut s = setup("cli", case, given);
    let owner = give_state_dir_away(&s.state);
    let argv: Vec<String> = match given.get("argv") {
        Some(list) => list
            .as_array()
            .unwrap()
            .iter()
            .map(|a| a.as_str().unwrap().replace("<state>", &s.state_arg))
            .collect(),
        None => vec![s.state_arg.clone(), "--once".to_string()],
    };
    let mut cmd = Command::new(BIN);
    cmd.arg("link-status")
        .args(&argv)
        .env_clear()
        .env("PATH", &s.bin)
        .env("MOWER_LTE_AT", &s.at_port)
        .env("MOWER_LINK_WIRELESS", &s.wireless)
        .current_dir(&s.dir);
    if let Some(env) = given.get("env").and_then(Value::as_object) {
        for (k, v) in env {
            cmd.env(k, v.as_str().unwrap());
        }
    }
    let out = cmd.output().expect("mowerctl runs");
    let commands = s.modem.take().map(|m| m.finish()).unwrap_or_default();
    let run = Run {
        rc: out.status.code().unwrap_or(-1),
        stdout: String::from_utf8(out.stdout).unwrap(),
        commands,
        real_clock: true,
    };
    check(case, want, &run, &s, owner);
    if want["rc"] == 1 {
        assert!(!out.stderr.is_empty(), "{case}: says why");
    }
    drop_scratch(&s.dir);
}

#[test]
fn link_status_does_what_python_did_in_every_case() {
    umask_022();
    let g = golden("g4_link_status.json");
    let cases: Vec<(&String, &Value)> = g["cases"].as_object().unwrap().iter().collect();
    assert!(cases.len() > 50);
    let next = AtomicUsize::new(0);
    std::thread::scope(|sc| {
        for _ in 0..6 {
            sc.spawn(|| {
                while let Some((case, want)) = cases.get(next.fetch_add(1, Ordering::Relaxed)) {
                    link_case(case, want);
                }
            });
        }
    });
}

fn now_s() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs()
}

/// Without --once it goes on, whatever one round does, like the Python loop.
#[test]
fn link_status_keeps_going_through_a_failed_round() {
    let dir = scratch("cli", "loop");
    let state = dir.join("state");
    let mut child = Command::new(BIN)
        .arg("link-status")
        .arg(&state)
        .env_clear()
        .env("PATH", "/nonexistent")
        .env("MOWER_LINK_PERIOD_S", "0.1")
        .env("MOWER_LTE_AT", dir.join("no-modem"))
        .env("MOWER_LINK_WIRELESS", dir.join("no-wireless"))
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    // no state dir yet: rounds fail; then there is one
    std::thread::sleep(Duration::from_millis(400));
    assert!(child.try_wait().unwrap().is_none(), "the loop stopped");
    std::fs::create_dir(&state).unwrap();
    let target = state.join("link_status.json");
    let deadline = Instant::now() + Duration::from_secs(10);
    while !target.exists() {
        assert!(Instant::now() < deadline, "no file came");
        std::thread::sleep(Duration::from_millis(20));
    }
    let doc: Value = serde_json::from_slice(&std::fs::read(&target).unwrap()).unwrap();
    assert!(doc["time"].as_u64().unwrap() + 60 > now_s());
    assert_eq!(doc["lte"], serde_json::json!({"present": false}));
    let first = doc["time"].clone();
    // and on
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        // a reader never sees half a file
        let doc: Value = serde_json::from_slice(&std::fs::read(&target).unwrap()).unwrap();
        if doc["time"] != first {
            break;
        }
        assert!(Instant::now() < deadline, "the file stopped changing");
        std::thread::sleep(Duration::from_millis(20));
    }
    child.kill().unwrap();
    let out = child.wait_with_output().unwrap();
    let stderr = String::from_utf8_lossy(&out.stderr);
    let lines: Vec<&str> = stderr.lines().collect();
    assert!(lines.len() >= 2, "{stderr}");
    assert!(lines[0].starts_with("link-status: [Errno 2] No such file or directory: '"), "{stderr}");
    drop_scratch(&dir);
}

#[test]
fn link_status_help_starts_nothing() {
    let out = Command::new(BIN).args(["link-status", "--help"]).output().unwrap();
    assert!(out.status.success());
    assert!(String::from_utf8_lossy(&out.stdout).starts_with("usage: mowerctl link-status"));
}

// ---- pair --------------------------------------------------------------------

/// The real machine's LAN address is not the golden's: leave it out of both.
fn without_lan(text: &str) -> String {
    let mut out = String::new();
    for line in text.lines() {
        if line.starts_with("lan      : ") || line.trim_start().starts_with("\"lan\": ") {
            continue;
        }
        match line.find("&l=") {
            Some(at) => {
                out.push_str(&line[..at]);
                // what follows the address: nothing, or the end of a JSON string
                if line.ends_with("\",") {
                    out.push_str("\",");
                } else if line.ends_with('"') {
                    out.push('"');
                }
            }
            None => out.push_str(line),
        }
        out.push('\n');
    }
    out
}

fn strs(v: &Value) -> Vec<String> {
    v.as_array().unwrap().iter().map(|x| x.as_str().unwrap().to_string()).collect()
}

/// A case the real machine cannot change: an identity that stays, no clock and
/// no randomness, an owner that is left to `/etc/passwd` here.
fn pair_case_is_machine_free(want: &Value) -> bool {
    let given = &want["given"];
    given.get("identity").and_then(Value::as_str).is_some()
        && want["draws"].as_array().unwrap().is_empty()
        && !want["given"]["argv"].as_array().unwrap().iter().any(|a| a == "--rotate")
        && given.get("chown_error").is_none()
        && given.get("lan").is_none()
        && given.get("passwd").is_none()
}

fn write_tool(path: &Path, text: &str) {
    std::fs::write(path, text).unwrap();
    std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o755)).unwrap();
}

fn pair_case(i: usize, case: &str, want: &Value) {
    let given = &want["given"];
    let dir = scratch("pair", &format!("{i}"));
    let state = dir.join("state");
    let flag = |k: &str| given.get(k).and_then(Value::as_bool).unwrap_or(false);
    std::fs::create_dir(&state).unwrap();
    let path = state.join("identity.json");
    std::fs::write(&path, latin1(given["identity"].as_str().unwrap())).unwrap();
    let mode = given.get("identity_mode").and_then(Value::as_u64).unwrap_or(0o600) as u32;
    std::fs::set_permissions(&path, std::fs::Permissions::from_mode(mode)).unwrap();
    let old = UNIX_EPOCH + Duration::from_secs(1_000_000_000);
    std::fs::File::options().write(true).open(&path).unwrap().set_modified(old).unwrap();
    let env_path = dir.join("env");
    if flag("env_is_dir") {
        std::fs::create_dir(&env_path).unwrap();
    } else if let Some(text) = given.get("env_file").and_then(Value::as_str) {
        std::fs::write(&env_path, latin1(text)).unwrap();
    }
    let bin = dir.join("bin");
    std::fs::create_dir(&bin).unwrap();
    let qr_log = dir.join("qr-log");
    if flag("qrencode") {
        write_tool(
            &bin.join("qrencode"),
            &format!("#!/bin/sh\nfor a in \"$@\"; do printf '%s\\n' \"$a\" >> '{}'; done\necho '<QR>'\n", qr_log.display()),
        );
    }
    let suffix = given.get("state_arg_suffix").and_then(Value::as_str).unwrap_or("");
    let mut cmd = Command::new(BIN);
    cmd.arg("pair").env_clear().env("PATH", &bin).current_dir(&dir);
    if flag("state_from_env") {
        cmd.env("MOWER_STATE_DIR", &state);
    } else {
        cmd.arg("--state-dir").arg(format!("{}{suffix}", state.display()));
    }
    cmd.arg("--env").arg(&env_path).args(strs(&given["argv"]));
    let out = cmd.output().unwrap();
    let label = |s: &str| s.replace(&state.to_string_lossy().into_owned(), STATE);
    assert_eq!(i64::from(out.status.code().unwrap_or(-1)), want["rc"].as_i64().unwrap(), "{case}: rc; {out:?}");
    let want_stdout = strs(&want["stdout"]);
    if !want_stdout.is_empty() {
        let got = without_lan(&label(&String::from_utf8(out.stdout).unwrap()));
        let want_text = without_lan(&(want_stdout.join("\n") + "\n"));
        assert_eq!(got, want_text, "{case}: stdout");
    }
    let got_file: String = std::fs::read(&path).unwrap().iter().map(|b| *b as char).collect();
    assert_eq!(Some(got_file.as_str()), want["identity"].as_str(), "{case}: the file");
    let st = std::fs::metadata(&path).unwrap();
    assert_eq!(u64::from(st.mode() & 0o7777), want["mode"].as_u64().unwrap(), "{case}: its mode");
    assert_eq!(st.mtime() != 1_000_000_000, want["rewritten"].as_bool().unwrap(), "{case}: rewritten");
    assert!(!state.join("identity.json.tmp").exists(), "{case}: the tmp");
    if flag("qrencode") && !want["qrencode"].as_array().unwrap().is_empty() {
        let args: Vec<String> = std::fs::read_to_string(&qr_log).unwrap().lines().map(str::to_string).collect();
        let want_args = strs(&want["qrencode"][0]);
        assert_eq!(args[..4], want_args[..4], "{case}: qrencode's options");
        assert_eq!(without_lan(&args[4]), without_lan(&want_args[4]), "{case}: qrencode's url");
    }
    drop_scratch(&dir);
}

#[test]
fn pair_does_what_python_did_where_the_machine_does_not_matter() {
    umask_022();
    let g = golden("g5_pair.json");
    let mut ran = 0;
    for (i, (case, want)) in g["cases"].as_object().unwrap().iter().enumerate() {
        if pair_case_is_machine_free(want) {
            pair_case(i, case, want);
            ran += 1;
        }
    }
    assert!(ran > 40, "only {ran} cases");
}

/// Python holds this on a pipe (the golden); a person is on a terminal, where
/// the text comes first and qrencode writes after it.
#[test]
fn on_a_terminal_the_text_comes_before_the_qr() {
    let dir = scratch("pair", "terminal");
    let state = dir.join("state");
    std::fs::create_dir(&state).unwrap();
    std::fs::write(state.join("identity.json"), r#"{"robot_id": "MW-1", "secret": "S"}"#).unwrap();
    let bin = dir.join("bin");
    std::fs::create_dir(&bin).unwrap();
    write_tool(&bin.join("qrencode"), "#!/bin/sh\necho '<QR>'\n");
    let (master, slave, _) = open_pty();
    // SAFETY: slave is ours and open; Stdio takes the duplicate.
    let (out, err) = unsafe {
        use std::os::fd::FromRawFd;
        (Stdio::from_raw_fd(libc::dup(slave)), Stdio::from_raw_fd(slave))
    };
    let status = Command::new(BIN)
        .args(["pair", "--state-dir"])
        .arg(&state)
        .arg("--env")
        .arg(dir.join("no-env"))
        .env_clear()
        .env("PATH", &bin)
        .stdout(out)
        .stderr(err)
        .status()
        .unwrap();
    assert!(status.success());
    let mut text = Vec::new();
    loop {
        let mut p = libc::pollfd { fd: master, events: libc::POLLIN, revents: 0 };
        // SAFETY: one pollfd, valid for the call.
        if unsafe { libc::poll(&mut p, 1, 500) } <= 0 {
            break;
        }
        let mut buf = [0u8; 4096];
        // SAFETY: the buffer is valid for its length.
        let n = unsafe { libc::read(master, buf.as_mut_ptr().cast(), buf.len()) };
        if n <= 0 {
            break;
        }
        text.extend_from_slice(&buf[..n as usize]);
    }
    // SAFETY: the master is ours.
    unsafe {
        libc::close(master);
    }
    let text = String::from_utf8(text).unwrap();
    let (id, qr) = (text.find("robot_id : MW-1").expect("the text"), text.find("<QR>").expect("the QR"));
    assert!(id < qr, "{text}");
    drop_scratch(&dir);
}

// ---- side by side with the Python tools ---------------------------------------

fn have_python() -> bool {
    Command::new("python3").arg("--version").output().is_ok_and(|o| o.status.success())
}

/// The owner to ask for: on this machine, anyone real. Root (the dev container,
/// CI's job) and whoever else runs this both own what they make.
fn me() -> String {
    // SAFETY: geteuid has no failure mode.
    let uid = unsafe { libc::geteuid() };
    let passwd = std::fs::read_to_string("/etc/passwd").unwrap_or_default();
    passwd
        .lines()
        .find(|l| l.split(':').nth(2) == Some(uid.to_string().as_str()))
        .and_then(|l| l.split(':').next())
        .unwrap_or("root")
        .to_string()
}

/// A new identity on this machine: its robot id, name, LAN address and files.
#[test]
fn a_new_identity_comes_out_like_pythons_on_this_machine() {
    if !have_python() {
        eprintln!("skipped: needs python3");
        return;
    }
    umask_022();
    let dir = scratch("pair", "side-by-side");
    let tool = repo().join("deploy/host/mower-pair");
    let run = |side: &str, cmd: &mut Command| -> (Value, String, u32) {
        let state = dir.join(side);
        let out = cmd
            .arg("--state-dir")
            .arg(&state)
            .arg("--env")
            .arg(dir.join("no-env"))
            .arg("--owner")
            .arg(me())
            .arg("--json")
            .env("PYTHONDONTWRITEBYTECODE", "1")
            .output()
            .unwrap();
        assert!(out.status.success(), "{side}: {out:?}");
        let doc: Value = serde_json::from_slice(&out.stdout).unwrap();
        let file = std::fs::read_to_string(state.join("identity.json")).unwrap();
        let mode = std::fs::metadata(state.join("identity.json")).unwrap().mode() & 0o7777;
        (doc, file, mode)
    };
    let (a, file_a, mode_a) = run("python", Command::new("python3").arg(&tool));
    let (b, file_b, mode_b) = run("rust", Command::new(BIN).arg("pair"));
    assert_eq!(mode_a, 0o600);
    assert_eq!(mode_b, 0o600);
    for key in ["name", "relay", "lan"] {
        assert_eq!(a[key], b[key], "{key}");
    }
    // A container may have no machine-id: then the seed is random, on both sides.
    let has_machine_id = ["/etc/machine-id", "/var/lib/dbus/machine-id"]
        .iter()
        .any(|f| std::fs::read_to_string(f).is_ok_and(|t| !t.trim().is_empty()));
    if has_machine_id {
        assert_eq!(a["robot_id"], b["robot_id"]);
    }
    let keys = |d: &Value| d.as_object().unwrap().keys().cloned().collect::<HashSet<_>>();
    assert_eq!(keys(&a), keys(&b));
    let secret = b["secret"].as_str().unwrap();
    assert_eq!(secret.len(), 32);
    assert!(secret.chars().all(|c| matches!(c, 'A'..='Z' | '2'..='7')), "{secret}");
    assert_ne!(a["secret"], b["secret"]);
    assert!(b["created"].as_u64().unwrap().abs_diff(a["created"].as_u64().unwrap()) < 60);
    // the same text but for what is this run's own
    let plain = |d: &Value, text: &str| {
        text.replace(d["secret"].as_str().unwrap(), "S")
            .replace(&d["created"].to_string(), "T")
            .replace(d["robot_id"].as_str().unwrap(), "R")
    };
    assert_eq!(plain(&a, &file_a), plain(&b, &file_b));
    assert_eq!(plain(&a, a["url"].as_str().unwrap()), plain(&b, b["url"].as_str().unwrap()));
    drop_scratch(&dir);
}

/// A sample on this machine: no modem, its own /proc/net/wireless and `ip`.
#[test]
fn a_sample_comes_out_like_pythons_on_this_machine() {
    if !have_python() {
        eprintln!("skipped: needs python3");
        return;
    }
    let dir = scratch("cli", "side-by-side");
    let tool = repo().join("deploy/host/mower-link-status.py");
    for side in ["python", "rust"] {
        std::fs::create_dir(dir.join(side)).unwrap();
    }
    let no_modem = dir.join("no-modem");
    let a = Command::new("python3")
        .arg(&tool)
        .arg(dir.join("python"))
        .arg("--once")
        .env("MOWER_LTE_AT", &no_modem)
        .env("PYTHONDONTWRITEBYTECODE", "1")
        .output()
        .unwrap();
    let b = Command::new(BIN).arg("link-status").arg(dir.join("rust")).arg("--once").env("MOWER_LTE_AT", &no_modem).output().unwrap();
    assert!(a.status.success() && b.status.success(), "{a:?} {b:?}");
    assert_eq!(mask_time(&String::from_utf8(a.stdout).unwrap()), mask_time(&String::from_utf8(b.stdout).unwrap()));
    let file = |side: &str| mask_time(&std::fs::read_to_string(dir.join(side).join("link_status.json")).unwrap());
    assert_eq!(file("python"), file("rust"));
    drop_scratch(&dir);
}

// ---- no r2r ------------------------------------------------------------------

/// What `Cargo.lock` lets mowerctl reach: no r2r, nothing that binds a C library
/// (a `-sys` crate outside the two OS ones), so the static binary for the host
/// keeps building (README.md, "Where it lives"). A dependency of a dependency
/// that brought r2r in would fail here, not on a robot.
#[test]
fn nothing_mowerctl_can_reach_is_r2r_or_links_c() {
    let lock = std::fs::read_to_string(repo().join("src/mower_rs/Cargo.lock")).unwrap();
    let mut packages: std::collections::HashMap<String, Vec<String>> = std::collections::HashMap::new();
    for block in lock.split("[[package]]\n").skip(1) {
        let name = block.lines().find_map(|l| l.strip_prefix("name = \"")).and_then(|n| n.strip_suffix('"')).unwrap().to_string();
        let mut deps = Vec::new();
        if let Some(start) = block.find("dependencies = [\n") {
            for line in block[start..].lines().skip(1) {
                let Some(dep) = line.trim().strip_prefix('"') else {
                    break;
                };
                deps.push(dep.split(['"', ' ']).next().unwrap().to_string());
            }
        }
        packages.entry(name).or_default().extend(deps);
    }
    let mut seen = HashSet::new();
    let mut todo = vec!["mowerctl".to_string()];
    while let Some(p) = todo.pop() {
        if seen.insert(p.clone()) {
            todo.extend(packages.get(&p).cloned().unwrap_or_default());
        }
    }
    for p in ["mowerctl", "mower_link", "mower_pyfmt", "libc"] {
        assert!(seen.contains(p), "{p} should be in reach, is the parse right?");
    }
    assert!(!seen.iter().any(|p| p.starts_with("r2r")), "{seen:?}");
    let sys: Vec<&String> = seen.iter().filter(|p| p.ends_with("-sys") && !p.starts_with("windows")).collect();
    assert!(sys.is_empty(), "{sys:?}");
}
