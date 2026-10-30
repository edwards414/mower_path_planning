//! The command line of `mower_flash.py` (argparse: `-p/--port` and
//! `-b/--baud` before the subcommand, unique prefixes of long options,
//! `--opt=value`, exit 2 on a usage error) and its `main()`: what each
//! subcommand prints and how it exits.
//!
//! Two additions the Python tool does not have, neither used by
//! `utils/firmware-sync`: `--trace FILE` (every frame sent and received, one
//! JSON line each, for on-robot runs) and `manifest FILE`, which prints the
//! `version sha8` the wrapper's log line shows.

use mower_base_core::boot::BL_RUN_APP;

use crate::flasher::{check, describe_bl_info, Session};
use crate::image::load_manifest;
use crate::link::{Clock, Link, SerialLink, SystemClock};
use crate::pyjson;
use crate::{Failure, Out, Trace};

pub const USAGE: &str = "usage: mower_flash [-h] -p PORT [-b BAUD] [--trace FILE] {flash,info,enter,run,app-info,sync,manifest} ...";

pub const HELP: &str = "\
Flash the mower STM32F411 application over the host UART (the Rust port of
firmware/tools/mower_flash.py: the same frames, timing, output, report and
exit codes).

    mower_flash -p /dev/ttyUSB0 flash build/mower_robot_firmware.bin
    mower_flash -p /dev/ttyUSB0 info
    mower_flash -p /dev/ttyUSB0 enter     # reboot into bootloader and stay
    mower_flash -p /dev/ttyUSB0 run       # jump to the application
    mower_flash -p /dev/ttyS3 app-info    # 0x87 build identity of the running app
    mower_flash -p /dev/ttyS3 sync build/mower_robot_firmware.bin
        # flash only if the running app differs from the bundled build (firmware-sync)
    mower_flash manifest build/mower_robot_firmware.json   # `version sha8`

positional arguments:
  {flash,info,enter,run,app-info,sync,manifest}
    flash               write a .bin into the app region and start it
    info                print bootloader info (reboots the app into it)
    enter               reboot into the bootloader and stay there
    run                 jump from the bootloader to the application
    app-info            print the running application's build identity (0x87)
    sync                flash only if the running app is a different build than IMAGE
    manifest            print the manifest's `version sha8`

options:
  -h, --help            show this help message and exit
  -p PORT, --port PORT  serial port, e.g. /dev/ttyUSB0
  -b BAUD, --baud BAUD
  --trace FILE          append every frame sent and received to FILE (JSON lines)

flash IMAGE [--no-run]                 --no-run: stay in the bootloader afterwards
sync IMAGE [--manifest M] [--report R] [--force]
                                       --manifest: JSON from tools/fw_manifest.py (default: IMAGE with .json)
                                       --report: write a JSON report of what happened here
                                       --force: flash even if the build matches

Exit codes as mower_flash.py: sync 0 up to date / flashed, 1 failed, 2 no STM32;
1 on any other error; 2 on a usage error.
";

/// A parsed command line.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Args {
    pub port: Option<String>,
    pub baud: u32,
    pub trace: Option<String>,
    pub cmd: Cmd,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Cmd {
    Flash { image: String, no_run: bool },
    Info,
    Enter,
    Run,
    AppInfo,
    Sync { image: String, manifest: Option<String>, report: Option<String>, force: bool },
    Manifest { path: String },
}

/// Why parsing stopped: help (exit 0) or a usage error (exit 2).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Stop {
    Help,
    Usage(String),
}

/// Python's `int(text)`: blanks around, a sign, `_` between digits.
fn py_int(text: &str) -> Option<i64> {
    let t = text.trim();
    let (neg, digits) = match t.as_bytes().first() {
        Some(b'-') => (true, &t[1..]),
        Some(b'+') => (false, &t[1..]),
        _ => (false, t),
    };
    if digits.is_empty() || digits.starts_with('_') || digits.ends_with('_') || digits.contains("__") {
        return None;
    }
    let clean: String = digits.chars().filter(|&c| c != '_').collect();
    if !clean.chars().all(|c| c.is_ascii_digit()) {
        return None;
    }
    clean.parse::<i64>().ok().map(|v| if neg { -v } else { v })
}

/// Resolve `--x` against long options the way argparse does: exact, or the
/// one option it is a prefix of.
fn long_option<'a>(given: &str, options: &[&'a str]) -> Result<&'a str, Stop> {
    if let Some(o) = options.iter().find(|o| **o == given) {
        return Ok(o);
    }
    let hits: Vec<&&str> = options.iter().filter(|o| o.starts_with(given)).collect();
    match hits.len() {
        1 => Ok(hits[0]),
        0 => Err(Stop::Usage(format!("unrecognized arguments: {given}"))),
        _ => Err(Stop::Usage(format!(
            "ambiguous option: {given} could match {}",
            hits.iter().map(|o| o.to_string()).collect::<Vec<_>>().join(", ")
        ))),
    }
}

/// One option token: (canonical name, attached value).
fn option(token: &str, longs: &[&'static str], shorts: &[(char, &'static str)]) -> Result<(&'static str, Option<String>), Stop> {
    if let Some(rest) = token.strip_prefix("--") {
        let (name, value) = match rest.split_once('=') {
            Some((n, v)) => (n, Some(v.to_string())),
            None => (rest, None),
        };
        return Ok((long_option(&format!("--{name}"), longs)?, value));
    }
    let mut chars = token[1..].chars();
    let c = chars.next().unwrap_or('-');
    let attached: String = chars.collect();
    match shorts.iter().find(|(s, _)| *s == c) {
        Some((_, long)) => Ok((long, if attached.is_empty() { None } else { Some(attached) })),
        None => Err(Stop::Usage(format!("unrecognized arguments: {token}"))),
    }
}

pub fn parse(argv: &[String]) -> Result<Args, Stop> {
    let mut port = None;
    let mut baud = 115200u32;
    let mut trace = None;
    let mut i = 0;
    let need_value = |i: &mut usize, name: &str, attached: Option<String>| -> Result<String, Stop> {
        if let Some(v) = attached {
            return Ok(v);
        }
        *i += 1;
        argv.get(*i).cloned().ok_or_else(|| Stop::Usage(format!("argument {name}: expected one argument")))
    };
    let cmd_name = loop {
        let Some(token) = argv.get(i) else {
            return Err(Stop::Usage("the following arguments are required: cmd".into()));
        };
        if token.starts_with('-') && token.len() > 1 && token != "--" {
            let (name, attached) = option(token, &["--help", "--port", "--baud", "--trace"], &[('h', "--help"), ('p', "--port"), ('b', "--baud")])?;
            match name {
                "--help" => return Err(Stop::Help),
                "--port" => port = Some(need_value(&mut i, "-p/--port", attached)?),
                "--baud" => {
                    let v = need_value(&mut i, "-b/--baud", attached)?;
                    baud = py_int(&v)
                        .and_then(|b| u32::try_from(b).ok())
                        .ok_or_else(|| Stop::Usage(format!("argument -b/--baud: invalid int value: {}", pyjson::str_repr(&v))))?;
                }
                _ => trace = Some(need_value(&mut i, "--trace", attached)?),
            }
            i += 1;
            continue;
        }
        if token == "--" {
            i += 1;
            continue;
        }
        break token.clone();
    };
    i += 1;
    let rest = &argv[i..];
    let mut positional = Vec::new();
    let mut flags = std::collections::BTreeMap::new();
    let (longs, takes_value, nargs): (&[&'static str], &[&str], usize) = match cmd_name.as_str() {
        "flash" => (&["--help", "--no-run"], &[], 1),
        "sync" => (&["--help", "--manifest", "--report", "--force"], &["--manifest", "--report"], 1),
        "manifest" => (&["--help"], &[], 1),
        "info" | "enter" | "run" | "app-info" => (&["--help"], &[], 0),
        other => {
            return Err(Stop::Usage(format!(
                "argument cmd: invalid choice: {} (choose from 'flash', 'info', 'enter', 'run', 'app-info', 'sync', 'manifest')",
                pyjson::str_repr(other)
            )))
        }
    };
    let mut k = 0;
    let mut only_positional = false;
    while k < rest.len() {
        let token = &rest[k];
        if !only_positional && token == "--" {
            only_positional = true;
        } else if !only_positional && token.starts_with('-') && token.len() > 1 {
            let (name, attached) = option(token, longs, &[('h', "--help")])?;
            if name == "--help" {
                return Err(Stop::Help);
            }
            if takes_value.contains(&name) {
                let v = match attached {
                    Some(v) => v,
                    None => {
                        k += 1;
                        rest.get(k).cloned().ok_or_else(|| Stop::Usage(format!("argument {name}: expected one argument")))?
                    }
                };
                flags.insert(name, Some(v));
            } else {
                if attached.is_some() {
                    return Err(Stop::Usage(format!("argument {name}: ignored explicit argument {}", pyjson::str_repr(attached.as_deref().unwrap_or("")))));
                }
                flags.insert(name, None);
            }
        } else {
            positional.push(token.clone());
        }
        k += 1;
    }
    if positional.len() > nargs {
        return Err(Stop::Usage(format!("unrecognized arguments: {}", positional[nargs..].join(" "))));
    }
    if positional.len() < nargs {
        let what = if cmd_name == "manifest" { "file" } else { "image" };
        return Err(Stop::Usage(format!("the following arguments are required: {what}")));
    }
    if port.is_none() && cmd_name != "manifest" {
        return Err(Stop::Usage("the following arguments are required: -p/--port".into()));
    }
    let value = |name: &str| flags.get(name).cloned().flatten();
    let cmd = match cmd_name.as_str() {
        "flash" => Cmd::Flash { image: positional[0].clone(), no_run: flags.contains_key("--no-run") },
        "sync" => Cmd::Sync {
            image: positional[0].clone(),
            manifest: value("--manifest"),
            report: value("--report"),
            force: flags.contains_key("--force"),
        },
        "manifest" => Cmd::Manifest { path: positional[0].clone() },
        "info" => Cmd::Info,
        "enter" => Cmd::Enter,
        "run" => Cmd::Run,
        _ => Cmd::AppInfo,
    };
    Ok(Args { port, baud, trace, cmd })
}

/// The `try:` block of `main()` on an open port: the exit code, or the
/// exception that ends the process (`error: ...` for the two it catches).
pub fn run_command<L: Link, C: Clock>(s: &mut Session<L, C>, cmd: &Cmd) -> Result<i32, Failure> {
    let result = match cmd {
        Cmd::Flash { image, no_run } => s.do_flash(image, !no_run).map(|_| 0),
        Cmd::Info | Cmd::Enter => s.ensure_bootloader(25).map(|info| {
            s.out.println(&describe_bl_info(&info));
            0
        }),
        Cmd::Run => s.command(BL_RUN_APP, &[], 2.0).and_then(|(status, value)| check(status, "run", value)).map(|_| {
            s.out.println("application started");
            0
        }),
        Cmd::AppInfo => (|| {
            let (info, saw_app) = s.read_app_info(1.5)?;
            if let Some(info) = info {
                s.out.println(&info.describe());
                s.out.println(&pyjson::dumps(&info.to_py(), None));
                return Ok(0);
            }
            if saw_app {
                return Err(Failure::Exit("application is running but does not answer 0x06 (firmware predates 0x87)".into()));
            }
            if s.ping(0.5)?.is_some() {
                return Err(Failure::Exit("board is in the bootloader, no application running".into()));
            }
            Err(Failure::Exit("no STM32 detected on this port".into()))
        })(),
        Cmd::Sync { image, manifest, report, force } => s.do_sync(image, manifest.as_deref(), report.as_deref(), *force),
        Cmd::Manifest { .. } => unreachable!("manifest needs no port"),
    };
    match result {
        Err(Failure::Runtime(msg)) | Err(Failure::Timeout(msg)) => Err(Failure::Exit(format!("error: {msg}"))),
        other => other,
    }
}

/// How the process ends: Python prints a `sys.exit` text or the uncaught
/// exception to stderr first and flushes stdout last.
pub fn exit_code(result: Result<i32, Failure>, out: &mut Out) -> i32 {
    let code = match result {
        Ok(code) => code,
        Err(Failure::Exit(msg)) | Err(Failure::Uncaught(msg)) | Err(Failure::Runtime(msg)) | Err(Failure::Timeout(msg)) => {
            eprintln!("{msg}");
            1
        }
    };
    out.flush();
    code
}

/// `print(d['version'], d['git_sha'][:8])` of the wrapper's log line.
fn manifest_line(path: &str) -> Result<String, Failure> {
    let d = load_manifest("", Some(path))?;
    let version = pyjson::getitem(&d, "version").map_err(Failure::Uncaught)?;
    let sha = pyjson::getitem(&d, "git_sha").and_then(pyjson::head8).map_err(Failure::Uncaught)?;
    Ok(format!("{} {}", pyjson::py_str(version), pyjson::py_str(&sha)))
}

pub fn main(argv: &[String]) -> i32 {
    let args = match parse(argv) {
        Ok(a) => a,
        Err(Stop::Help) => {
            println!("{USAGE}\n\n{HELP}");
            return 0;
        }
        Err(Stop::Usage(msg)) => {
            eprintln!("{USAGE}\nmower_flash: error: {msg}");
            return 2;
        }
    };
    let mut out = Out::stdout();
    if let Cmd::Manifest { path } = &args.cmd {
        let r = manifest_line(path).map(|line| {
            out.println(&line);
            0
        });
        return exit_code(r, &mut out);
    }
    let trace = match args.trace.as_deref().map(Trace::create) {
        None => None,
        Some(Ok(t)) => Some(t),
        Some(Err(e)) => return exit_code(Err(crate::image::open_error(&e, args.trace.as_deref().unwrap_or(""))), &mut out),
    };
    let port = args.port.clone().unwrap_or_default();
    let link = match SerialLink::open(&port, args.baud) {
        Ok(l) => l,
        Err(e) => return exit_code(Err(e), &mut out),
    };
    let mut session = Session::new(link, SystemClock::default(), out, trace);
    let result = run_command(&mut session, &args.cmd);
    exit_code(result, &mut session.out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(args: &str) -> Result<Args, Stop> {
        parse(&args.split_whitespace().map(String::from).collect::<Vec<_>>())
    }

    #[test]
    fn the_wrapper_line() {
        let a = p("-p /dev/stmcom sync /opt/mower/firmware/mower_robot_firmware.bin --report /home/mower/.mower/firmware_sync.json").unwrap();
        assert_eq!(a.port.as_deref(), Some("/dev/stmcom"));
        assert_eq!(a.baud, 115200);
        assert_eq!(
            a.cmd,
            Cmd::Sync {
                image: "/opt/mower/firmware/mower_robot_firmware.bin".into(),
                manifest: None,
                report: Some("/home/mower/.mower/firmware_sync.json".into()),
                force: false
            }
        );
    }

    #[test]
    fn argparse_spellings() {
        assert_eq!(p("--port=/dev/x -b 9600 info").unwrap().baud, 9600);
        assert_eq!(p("-p/dev/x --ba 1_200 enter").unwrap().baud, 1200);
        assert_eq!(p("--po /dev/x run").unwrap().port.as_deref(), Some("/dev/x"));
        assert_eq!(p("-p x sync --force --rep r i.bin").unwrap().cmd, Cmd::Sync { image: "i.bin".into(), manifest: None, report: Some("r".into()), force: true });
        assert_eq!(p("-p x flash --no-run a.bin").unwrap().cmd, Cmd::Flash { image: "a.bin".into(), no_run: true });
        assert_eq!(p("manifest m.json").unwrap().cmd, Cmd::Manifest { path: "m.json".into() });
        assert_eq!(p("-p x --trace t.jsonl app-info").unwrap().trace.as_deref(), Some("t.jsonl"));
    }

    #[test]
    fn usage_errors_exit_2() {
        for bad in ["sync a.bin", "-p x", "-p x sync", "-p x frob", "-p x info extra", "-p x -b fast info", "-p x sync a --bogus"] {
            assert!(matches!(p(bad), Err(Stop::Usage(_))), "{bad}");
        }
        assert_eq!(p("-h"), Err(Stop::Help));
        assert_eq!(p("-p x sync -h"), Err(Stop::Help));
    }
}
