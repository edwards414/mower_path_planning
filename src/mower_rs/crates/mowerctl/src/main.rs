//! mowerctl: a command-line view of a running robot, through the rosbridge
//! protocol ws_bridge serves (docs: README.md next to this crate; design:
//! E8 §2 of the ROS-removal plan).
//!
//! It connects to ws_bridge's loopback listener (127.0.0.1:9091, no gate)
//! by default, or with `--pair FILE` to the pairing gate (9090) with the
//! app's HMAC hand-shake, and speaks what the app speaks: `subscribe`,
//! `call_service`, nothing else. So it sees what that listener's policy
//! serves (today the app's topics and services), and costs the robot what an
//! app client costs. It needs no ROS: no DDS participant, no discovery.
//!
//! Read-only by default: no `topic pub`, no `param set`, and `service call`
//! refuses a service that may change the robot's state unless `--yes` is
//! given (service.rs).
//!
//! Exit codes: 0 done, 1 failed (bad arguments, a call answered with an
//! error), 2 no connection (refused, closed, or rejected by the gate), 3 no
//! such topic or service on that listener, 4 timed out.

mod cli;
mod conn;
#[cfg(test)]
mod fake_bridge;
mod format;
mod gate;
mod hz;
mod ping;
mod service;
mod topic;

use std::process::ExitCode;

pub const EXIT_FAILED: u8 = 1;
pub const EXIT_CONNECT: u8 = 2;
pub const EXIT_MISSING: u8 = 3;
pub const EXIT_TIMEOUT: u8 = 4;

/// Why a command did not succeed, and the exit code that says so.
#[derive(Debug, Clone, PartialEq)]
pub struct Fail {
    pub code: u8,
    pub msg: String,
}

impl Fail {
    pub fn failed(msg: impl Into<String>) -> Self {
        Fail { code: EXIT_FAILED, msg: msg.into() }
    }
    pub fn connect(msg: impl Into<String>) -> Self {
        Fail { code: EXIT_CONNECT, msg: msg.into() }
    }
    pub fn missing(msg: impl Into<String>) -> Self {
        Fail { code: EXIT_MISSING, msg: msg.into() }
    }
    pub fn timeout(msg: impl Into<String>) -> Self {
        Fail { code: EXIT_TIMEOUT, msg: msg.into() }
    }
    /// A bad command line; the message ends with where to look.
    pub fn usage(msg: impl Into<String>) -> Self {
        Fail { code: EXIT_FAILED, msg: format!("{} (mowerctl help)", msg.into()) }
    }
}

pub const USAGE: &str = "\
mowerctl: look at a running robot through ws_bridge (no ROS needed)

usage: mowerctl [connection options] <command> [options]

commands:
  topic list [-v] [--json]          topics the listener serves (-v: with types)
  topic info NAME [--json]          one topic's type
  topic echo NAME [options]         messages as they arrive, one JSON line each
      -n N            stop after N messages
      --throttle MS   at most one message per MS (done by the bridge)
      --field A.B     print only this field of each message
      --no-arr        print arrays as their length
      --yaml          YAML for people instead of JSON lines
      --raw           the frames exactly as received
      --timeout S     give up after S s (exit 4 if fewer than N arrived)
  topic hz NAME... [options]        receive rate, as `ros2 topic hz` prints it
      --window N      intervals per statistic (default 10000)
      --duration S    stop after S s (exit 4 if a topic sent nothing)
      --json          one JSON line per report
  service list [--json] [--policy FILE]
                                    services the listener's policy allows
  service call NAME [ARGS] [--yes] [--raw] [--timeout S]
                                    call one (ARGS: a JSON object, default {});
                                    a service that may change the robot's
                                    state needs --yes
  ping [-c N] [-i S]                round trip and the robot's clock offset
  help                              this text

connection options (before or after the command):
  --url URL       ws://HOST:PORT (default ws://127.0.0.1:9091, ws_bridge's
                  loopback listener; $MOWERCTL_URL overrides the default)
  --pair FILE     authenticate like the app with a pairing file
                  ({\"robot_id\", \"secret\"}, e.g. ~/.mower/identity.json);
                  the default URL is then ws://127.0.0.1:9090
  --timeout S     connection and answer timeout (default 5 s; 35 s for a
                  service call, whose bridge gives up after 30 s)

From another machine: ssh -L 9091:127.0.0.1:9091 cat@ROBOT, then mowerctl
here. Exit codes: 0 done, 1 failed, 2 no connection, 3 no such topic or
service, 4 timed out.
";

fn main() -> ExitCode {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    match run(argv) {
        Ok(()) => ExitCode::SUCCESS,
        // stdout's reader went away (format::Out): done, not failed
        Err(f) if f.code == 0 => ExitCode::SUCCESS,
        Err(f) => {
            eprintln!("mowerctl: {}", f.msg);
            ExitCode::from(f.code)
        }
    }
}

fn run(argv: Vec<String>) -> Result<(), Fail> {
    let argv = cli::expand_equals(argv);
    // Connection options may come before the command word; they are handed
    // to the command, which parses them with its own (cli::ConnOpts).
    let (lead, rest) = cli::split_leading_options(&argv)?;
    let Some((command, args)) = rest.split_first() else {
        print!("{USAGE}");
        return Err(Fail::usage("no command"));
    };
    // after the command's own arguments, so a subcommand stays first
    let mut args_with_lead = args.to_vec();
    args_with_lead.extend(lead.iter().cloned());
    match command.as_str() {
        "topic" => topic::main(args_with_lead),
        "service" => service::main(args_with_lead),
        "ping" => ping::main(args_with_lead),
        "help" | "--help" | "-h" => {
            print!("{USAGE}");
            Ok(())
        }
        "--version" | "-V" | "version" => {
            println!("mowerctl {}", env!("CARGO_PKG_VERSION"));
            Ok(())
        }
        // New commands go here. The ones that need no connection (E6's
        // `pair` and `link-status`) take `args` as they are and reject
        // `lead`; the ones that read the bridge (E8-2's `node`, `bridge`,
        // `graph`, `param`) parse cli::ConnOpts like the three above.
        other => Err(Fail::usage(format!("unknown command {other:?}"))),
    }
}

/// Run one command's future on a single-threaded runtime: a CLI has one
/// connection and nothing to parallelise.
pub fn block_on<F: std::future::Future<Output = Result<(), Fail>>>(fut: F) -> Result<(), Fail> {
    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .map_err(|e| Fail::failed(format!("tokio runtime: {e}")))?
        .block_on(fut)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(list: &[&str]) -> Vec<String> {
        list.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn an_unknown_or_missing_command_is_a_usage_error() {
        let e = run(args(&["frobnicate"])).unwrap_err();
        assert_eq!(e.code, EXIT_FAILED);
        assert!(e.msg.contains("unknown command \"frobnicate\""), "{e:?}");
        assert!(e.msg.ends_with("(mowerctl help)"));
        assert_eq!(run(args(&[])).unwrap_err().code, EXIT_FAILED);
        assert_eq!(run(args(&["--url", "ws://x:1"])).unwrap_err().code, EXIT_FAILED);
    }

    #[test]
    fn help_and_version_need_no_robot() {
        assert_eq!(run(args(&["help"])), Ok(()));
        assert_eq!(run(args(&["--help"])), Ok(()));
        assert_eq!(run(args(&["--version"])), Ok(()));
    }

    #[test]
    fn the_usage_lists_every_command_the_dispatcher_knows() {
        for c in ["topic list", "topic info", "topic echo", "topic hz", "service list", "service call", "ping", "help"] {
            assert!(USAGE.contains(&format!("  {c}")), "{c} missing from USAGE");
        }
        // no writing commands (E8 design D2)
        assert!(!USAGE.contains("topic pub") && !USAGE.contains("param set"));
    }
}
