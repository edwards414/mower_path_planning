//! `mower_rsd`: any subset of the mower_rs nodes in one process.
//!
//! Phase A5 of docs/ROS_FREE_PLAN.md. On the robot the thirteen mower_rs
//! binaries are eleven to fourteen processes, each with its own DDS
//! participant and six to ten threads, costing ~35 % of a core almost
//! entirely in DDS and executor overhead. This runs the same modules — the
//! same `run()` bodies, unchanged — on one `r2r::Context`, i.e. one
//! participant, with one tokio runtime and one spin thread per node.
//!
//! Nothing an outside observer can see changes: every module keeps its node
//! name, namespace, topics, services, actions and parameters, so the app and
//! the other nodes still address `/flutter_adapter`, `/nav_action_server`,
//! `/boustrophedon_coverage`, `/map_manage/set_parameters` and the rest.
//!
//! Usage
//! -----
//! ```text
//! mower_rsd --modules status,adapter,battery,guards \
//!           [--module-args bridge=--address 0.0.0.0 --port 9090 ...] \
//!           [--worker-threads 4] \
//!           --ros-args --params-file /path/mower_rsd.yaml \
//!                      -r imu:imu/data_raw:=imu/data
//! ```
//!
//! **Parameters.** Not invented here: `--params-file` is parsed by rcl into
//! the shared context, and `r2r::Node::create` hands each node the section
//! addressed to *its* node name (or to `/**`). So one file with a section per
//! node — `mower_bringup/config/mower_rsd.yaml` — replaces the per-process
//! `--ros-args`, and every module reads its parameters exactly as before,
//! including the per-node parameter services (`/map_manage/get_parameters`,
//! `/boustrophedon_coverage/set_parameters`, ...) that the app calls.
//!
//! **Remapping.** Also rcl's: a rule prefixed with a node name,
//! `-r imu:imu/data_raw:=imu/data`, applies to that node only. That is how
//! the launch `remappings=` of the IMU driver and of the two velocity guards
//! are reproduced here. A bare `-r __node:=x` must never be passed: it would
//! rename every node in the process, which is also why the module table below
//! carries the production node names instead of relying on launch's `name=`.
//!
//! **Supervision.** Every module is a tokio task wrapped in `catch_unwind`.
//! A module that panics, returns `Err`, or returns at all before it was asked
//! to stop is reported and started again 2 s later on its own (the launch
//! files' `respawn_delay`), while the other modules keep running — the same
//! failure model as the separate binaries, where one crashing driver (an IMU
//! with its USB port gone) never took the bridge or the guards down with it.
//! Restarting inside the process also keeps the DDS participant, so a driver
//! stuck in a restart loop no longer floods discovery. A failure during a
//! shutdown, or a module that does not stop within `--stop-timeout`, makes
//! the process exit non-zero so the launch `respawn` restarts the set.

use std::collections::BTreeSet;
use std::future::Future;
use std::pin::Pin;
use std::process::ExitCode;
use std::time::Duration;

use futures::future::FutureExt;
use mower_rs_common::{ModuleCtx, ModuleResult, Shutdown};

type BoxFut = Pin<Box<dyn Future<Output = ModuleResult> + Send>>;
type Runner = fn(r2r::Context, ModuleCtx) -> BoxFut;

/// Pause before a failed module is started again: the same 2 s the launch
/// files use as `respawn_delay` for the separate binaries.
const RESTART_DELAY: Duration = Duration::from_secs(2);

/// One node inside a module. `guards` is the only module with two.
struct Instance {
    /// ROS node name. Must match what the separate binary ends up with after
    /// the launch file's `name=`, because that is what everything else calls.
    node: &'static str,
    run: Runner,
}

struct Module {
    /// What `--modules` and the `rust_*` launch switches call it.
    id: &'static str,
    /// The launch argument that selects this module today.
    switch: &'static str,
    instances: &'static [Instance],
}

/// Every module `mower_rsd` can run. The order is the start order.
const MODULES: &[Module] = &[
    Module {
        id: "status",
        switch: "rust_status",
        instances: &[Instance { node: "robot_status", run: |c, m| Box::pin(robot_status::run(c, m)) }],
    },
    Module {
        id: "guards",
        switch: "rust_guards",
        // Two instances of the same code, as twist_mux.launch.py runs them:
        // the manual guard (app joystick -> mux, session required) and the
        // final guard (mux -> ros2_control). Their topic names stay relative
        // and are remapped per node name from --ros-args.
        instances: &[
            Instance { node: "manual_velocity_guard", run: |c, m| Box::pin(velocity_command_guard::run(c, m)) },
            Instance { node: "velocity_command_guard", run: |c, m| Box::pin(velocity_command_guard::run(c, m)) },
        ],
    },
    Module {
        id: "imu",
        switch: "rust_imu",
        instances: &[Instance { node: "imu", run: |c, m| Box::pin(mower_imu::run(c, m)) }],
    },
    Module {
        id: "gps",
        switch: "enable_gps",
        instances: &[Instance { node: "gps", run: |c, m| Box::pin(mower_gps::run(c, m)) }],
    },
    Module {
        id: "map",
        switch: "rust_map",
        instances: &[Instance { node: "map_manage", run: |c, m| Box::pin(mower_map::run(c, m)) }],
    },
    Module {
        id: "coverage",
        switch: "rust_coverage",
        instances: &[Instance { node: "boustrophedon_coverage", run: |c, m| Box::pin(mower_coverage::run(c, m)) }],
    },
    Module {
        id: "nav",
        switch: "rust_nav",
        instances: &[Instance { node: "nav_action_server", run: |c, m| Box::pin(mower_nav::run(c, m)) }],
    },
    Module {
        id: "record",
        // mission.launch.py names the Rust recorder path_record_node, not the
        // binary's own default path_recorder.
        switch: "rust_record",
        instances: &[Instance { node: "path_record_node", run: |c, m| Box::pin(mower_record::run(c, m)) }],
    },
    Module {
        id: "adapter",
        switch: "rust_adapter",
        instances: &[Instance { node: "flutter_adapter", run: |c, m| Box::pin(mower_adapter::run(c, m)) }],
    },
    Module {
        id: "battery",
        switch: "rust_battery",
        instances: &[Instance { node: "battery_state", run: |c, m| Box::pin(mower_battery::run(c, m)) }],
    },
    Module {
        id: "pid_autotune",
        switch: "rust_pid_autotune",
        instances: &[Instance { node: "pid_autotune", run: |c, m| Box::pin(mower_pid_autotune::run(c, m)) }],
    },
    Module {
        id: "bridge",
        switch: "rust_bridge",
        instances: &[Instance { node: "mower_ws_bridge", run: |c, m| Box::pin(mower_ws_bridge::run(c, m)) }],
    },
    Module {
        id: "agent",
        switch: "rust_agent",
        // No ROS node of its own: it reaches the graph through the bridge's
        // loopback WebSocket. The name is only used in logs.
        instances: &[Instance { node: "mower_agent", run: |_c, m| Box::pin(mower_agent::run(m)) }],
    },
];

struct Args {
    modules: Vec<String>,
    /// `module id -> argv` for the modules that take one (bridge, agent).
    module_args: Vec<(String, Vec<String>)>,
    worker_threads: usize,
    /// How long a module gets to unwind after the shutdown is tripped.
    stop_timeout: Duration,
}

const USAGE: &str = "usage: mower_rsd --modules a,b,c [--module-args <id>=<args>] \
[--worker-threads N] [--stop-timeout S] [--list-modules] [--ros-args ...]";

fn parse_args(argv: &[String]) -> Result<Args, String> {
    let mut a = Args {
        modules: Vec::new(),
        module_args: Vec::new(),
        worker_threads: 4,
        stop_timeout: Duration::from_secs(10),
    };
    let mut it = argv.iter().cloned();
    while let Some(arg) = it.next() {
        // `--key=value` as well as `--key value`, like the other binaries.
        let (key, inline) = match arg.split_once('=') {
            Some((k, v)) if k.starts_with("--") => (k.to_string(), Some(v.to_string())),
            _ => (arg.clone(), None),
        };
        let mut value = |name: &str| -> Result<String, String> {
            match inline.clone() {
                Some(v) => Ok(v),
                None => it.next().ok_or_else(|| format!("{name} needs a value")),
            }
        };
        match key.as_str() {
            "--modules" => {
                for m in value("--modules")?.split(',') {
                    let m = m.trim();
                    if !m.is_empty() {
                        a.modules.push(m.to_string());
                    }
                }
            }
            "--module-args" => {
                let v = value("--module-args")?;
                let (id, rest) = v
                    .split_once('=')
                    .ok_or_else(|| "--module-args takes <id>=<args>".to_string())?;
                a.module_args.push((
                    id.trim().to_string(),
                    rest.split_whitespace().map(|s| s.to_string()).collect(),
                ));
            }
            "--worker-threads" => {
                a.worker_threads = value("--worker-threads")?
                    .parse()
                    .map_err(|_| "--worker-threads takes a number".to_string())?;
                if a.worker_threads == 0 {
                    return Err("--worker-threads must be >= 1".into());
                }
            }
            "--stop-timeout" => {
                let s: f64 = value("--stop-timeout")?
                    .parse()
                    .map_err(|_| "--stop-timeout takes seconds".to_string())?;
                a.stop_timeout = Duration::from_secs_f64(s.max(0.1));
            }
            "--list-modules" => {
                for m in MODULES {
                    let nodes: Vec<&str> = m.instances.iter().map(|i| i.node).collect();
                    println!("{:<13} {:<18} {}", m.id, m.switch, nodes.join(", "));
                }
                std::process::exit(0);
            }
            // launch appends the ROS block; rcl already read it from argv when
            // the context was created.
            "--ros-args" => break,
            other => return Err(format!("unknown argument {other}\n{USAGE}")),
        }
    }
    if a.modules.is_empty() {
        return Err(format!("--modules is required\n{USAGE}"));
    }
    let known: BTreeSet<&str> = MODULES.iter().map(|m| m.id).collect();
    for m in &a.modules {
        if !known.contains(m.as_str()) {
            return Err(format!(
                "unknown module {m:?}; known: {}",
                known.iter().cloned().collect::<Vec<_>>().join(", ")
            ));
        }
    }
    for (id, _) in &a.module_args {
        if !a.modules.iter().any(|m| m == id) {
            return Err(format!("--module-args for {id:?}, which is not in --modules"));
        }
    }
    Ok(a)
}

/// How a module task ended.
enum Outcome {
    /// Asked to stop and stopped.
    Stopped(String),
    /// Returned early, returned an error, or panicked: the process must go.
    Failed(String),
}

/// Run one module and classify how it ended.
///
/// A panic in one module must not take the others down silently: it is caught
/// here, reported, and the caller stops everything in order. Returning at all
/// before the shutdown was tripped counts as a failure too — a module that
/// quietly gives up would leave a process that still looks alive to the
/// health gates.
async fn supervised(
    label: String, stopping: Shutdown,
    fut: impl Future<Output = ModuleResult>,
) -> Outcome {
    match std::panic::AssertUnwindSafe(fut).catch_unwind().await {
        Ok(Ok(())) if stopping.is_triggered() => Outcome::Stopped(label),
        Ok(Ok(())) => Outcome::Failed(format!("{label}: returned before shutdown")),
        Ok(Err(e)) => Outcome::Failed(format!("{label}: {e}")),
        Err(p) => {
            let what = p
                .downcast_ref::<&str>()
                .map(|s| s.to_string())
                .or_else(|| p.downcast_ref::<String>().cloned())
                .unwrap_or_else(|| "panic".to_string());
            Outcome::Failed(format!("{label}: panicked: {what}"))
        }
    }
}

fn main() -> ExitCode {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let args = match parse_args(&argv) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("[mower_rsd] {e}");
            return ExitCode::FAILURE;
        }
    };

    let runtime = match tokio::runtime::Builder::new_multi_thread()
        .worker_threads(args.worker_threads)
        .thread_name("mower_rsd")
        .enable_all()
        .build()
    {
        Ok(rt) => rt,
        Err(e) => {
            eprintln!("[mower_rsd] tokio runtime: {e}");
            return ExitCode::FAILURE;
        }
    };

    runtime.block_on(async move {
        // One context for the whole process: one DDS participant, one set of
        // discovery threads, one copy of the parameter overrides.
        let ctx = match r2r::Context::create() {
            Ok(c) => c,
            Err(e) => {
                eprintln!("[mower_rsd] r2r context: {e}");
                return ExitCode::FAILURE;
            }
        };
        let shutdown = Shutdown::new();
        if let Err(e) = shutdown.install_signal_handlers() {
            eprintln!("[mower_rsd] signal handler: {e}");
            return ExitCode::FAILURE;
        }

        // One slot per node instance. A slot reports every time its module
        // ends; the supervisor restarts it (after RESTART_DELAY, the launch
        // files' respawn_delay) unless the process is shutting down.
        struct Slot {
            label: String,
            node: &'static str,
            run: Runner,
            extra: Vec<String>,
        }
        let (report, mut reports) = tokio::sync::mpsc::unbounded_channel::<(usize, Outcome)>();
        let mut slots: Vec<Slot> = Vec::new();
        for id in &args.modules {
            let module = MODULES.iter().find(|m| m.id == *id).expect("checked in parse_args");
            let extra: Vec<String> = args
                .module_args
                .iter()
                .filter(|(mid, _)| mid == id)
                .flat_map(|(_, v)| v.clone())
                .collect();
            for inst in module.instances {
                slots.push(Slot {
                    label: format!("{}/{}", module.id, inst.node),
                    node: inst.node,
                    run: inst.run,
                    extra: extra.clone(),
                });
            }
        }
        let launch = |slot_idx: usize, delay: Duration| {
            let slot = &slots[slot_idx];
            let m = ModuleCtx::new(slot.node, shutdown.clone()).with_args(slot.extra.clone());
            let ctx = ctx.clone();
            let run = slot.run;
            let label = slot.label.clone();
            let stopping = shutdown.clone();
            let report = report.clone();
            tokio::spawn(async move {
                if !delay.is_zero() {
                    tokio::time::sleep(delay).await;
                    if stopping.is_triggered() {
                        let _ = report.send((slot_idx, Outcome::Stopped(label)));
                        return;
                    }
                    println!("[mower_rsd] restarting {label}");
                } else {
                    println!("[mower_rsd] starting {label}");
                }
                let _ = report.send((slot_idx, supervised(label, stopping, run(ctx, m)).await));
            });
        };
        for i in 0..slots.len() {
            launch(i, Duration::ZERO);
        }
        let mut alive = slots.len();

        // Supervise. A module that fails while the process is running is
        // restarted on its own, like launch respawns one crashing binary; the
        // others keep running. Once the shutdown is tripped (signal, or a
        // failure during the stop) everyone gets `stop_timeout` to unwind. A
        // module that wedges must not keep the process from restarting.
        let mut failures: Vec<String> = Vec::new();
        let mut restarts = 0usize;
        while alive > 0 {
            let next = if shutdown.is_triggered() {
                match tokio::time::timeout(args.stop_timeout, reports.recv()).await {
                    Ok(v) => v,
                    Err(_) => {
                        let why = format!(
                            "{} module(s) did not stop within {:.1} s",
                            alive,
                            args.stop_timeout.as_secs_f64()
                        );
                        eprintln!("[mower_rsd] {why}");
                        failures.push(why);
                        break;
                    }
                }
            } else {
                reports.recv().await
            };
            let Some((slot_idx, outcome)) = next else { break };
            match outcome {
                Outcome::Stopped(label) => {
                    alive -= 1;
                    println!("[mower_rsd] {label} stopped");
                }
                Outcome::Failed(why) if shutdown.is_triggered() => {
                    alive -= 1;
                    eprintln!("[mower_rsd] {why}");
                    failures.push(why);
                }
                Outcome::Failed(why) => {
                    restarts += 1;
                    eprintln!(
                        "[mower_rsd] {why}; restarting in {:.1} s (restart #{restarts})",
                        RESTART_DELAY.as_secs_f64()
                    );
                    launch(slot_idx, RESTART_DELAY);
                }
            }
        }
        drop(report);

        if failures.is_empty() {
            println!("[mower_rsd] stopped");
            ExitCode::SUCCESS
        } else {
            eprintln!("[mower_rsd] exiting non-zero after {} failure(s)", failures.len());
            ExitCode::FAILURE
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn argv(s: &[&str]) -> Vec<String> {
        s.iter().map(|x| x.to_string()).collect()
    }

    #[test]
    fn every_module_id_and_node_name_is_unique() {
        let mut ids = BTreeSet::new();
        let mut nodes = BTreeSet::new();
        for m in MODULES {
            assert!(ids.insert(m.id), "duplicate module id {}", m.id);
            for i in m.instances {
                assert!(nodes.insert(i.node), "duplicate node name {}", i.node);
            }
        }
    }

    /// The node names here are the contract the app and the other nodes use;
    /// they are the launch `name=` of the separate binary, not the binary's
    /// internal default (path_record_node, imu and gps all differ).
    #[test]
    fn module_table_carries_the_production_node_names() {
        let expected = [
            ("status", vec!["robot_status"]),
            ("guards", vec!["manual_velocity_guard", "velocity_command_guard"]),
            ("imu", vec!["imu"]),
            ("gps", vec!["gps"]),
            ("map", vec!["map_manage"]),
            ("coverage", vec!["boustrophedon_coverage"]),
            ("nav", vec!["nav_action_server"]),
            ("record", vec!["path_record_node"]),
            ("adapter", vec!["flutter_adapter"]),
            ("battery", vec!["battery_state"]),
            ("pid_autotune", vec!["pid_autotune"]),
            ("bridge", vec!["mower_ws_bridge"]),
            ("agent", vec!["mower_agent"]),
        ];
        assert_eq!(expected.len(), MODULES.len());
        for (id, nodes) in expected {
            let m = MODULES.iter().find(|m| m.id == id).unwrap_or_else(|| panic!("no module {id}"));
            let got: Vec<&str> = m.instances.iter().map(|i| i.node).collect();
            assert_eq!(got, nodes, "{id}");
        }
    }

    fn outcome(label: &str, triggered: bool, fut: impl Future<Output = ModuleResult>) -> Outcome {
        let rt = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
        let s = Shutdown::new();
        if triggered {
            s.trigger();
        }
        rt.block_on(supervised(label.to_string(), s, fut))
    }

    #[test]
    fn a_panicking_module_is_caught_and_named() {
        let o = outcome("imu/imu", true, async { panic!("serial exploded") });
        match o {
            Outcome::Failed(why) => {
                assert!(why.contains("imu/imu"), "{why}");
                assert!(why.contains("panicked: serial exploded"), "{why}");
            }
            Outcome::Stopped(_) => panic!("a panic must not look like a clean stop"),
        }
    }

    #[test]
    fn failing_and_early_returns_are_failures_but_an_asked_for_stop_is_not() {
        let err = outcome("gps/gps", true, async { Err("port went away".into()) });
        assert!(matches!(err, Outcome::Failed(ref w) if w.contains("port went away")));
        // returned on its own, nobody asked it to
        let early = outcome("nav/nav_action_server", false, async { Ok(()) });
        assert!(matches!(early, Outcome::Failed(ref w) if w.contains("before shutdown")));
        // returned because the shutdown was tripped
        let clean = outcome("status/robot_status", true, async { Ok(()) });
        assert!(matches!(clean, Outcome::Stopped(ref l) if l == "status/robot_status"));
    }

    #[test]
    fn modules_are_comma_separated_and_validated() {
        let a = parse_args(&argv(&["--modules", "status, battery ,guards"])).unwrap();
        assert_eq!(a.modules, vec!["status", "battery", "guards"]);
        assert!(parse_args(&argv(&["--modules", "status,nope"])).is_err());
        assert!(parse_args(&argv(&[])).is_err());
    }

    #[test]
    fn module_args_are_split_and_must_name_a_selected_module() {
        let a = parse_args(&argv(&[
            "--modules",
            "bridge,agent",
            "--module-args",
            "bridge=--address 0.0.0.0 --port 9090",
            "--module-args",
            "agent=--gate ws://127.0.0.1:9090",
        ]))
        .unwrap();
        assert_eq!(
            a.module_args[0].1,
            vec!["--address", "0.0.0.0", "--port", "9090"]
        );
        assert_eq!(a.module_args[1].1, vec!["--gate", "ws://127.0.0.1:9090"]);
        assert!(parse_args(&argv(&["--modules", "status", "--module-args", "bridge=--port 1"])).is_err());
        assert!(parse_args(&argv(&["--modules", "status", "--module-args", "oops"])).is_err());
    }

    #[test]
    fn the_ros_block_is_left_to_rcl() {
        let a = parse_args(&argv(&[
            "--modules=status",
            "--ros-args",
            "--params-file",
            "/tmp/x.yaml",
            "-r",
            "imu:imu/data_raw:=imu/data",
        ]))
        .unwrap();
        assert_eq!(a.modules, vec!["status"]);
        assert_eq!(a.worker_threads, 4);
    }
}
