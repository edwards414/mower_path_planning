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
//! are reproduced here. The bus gets the same rules: every `ModuleCtx` holds
//! them, and its `resolve_topic` / `resolve_service` give the name rcl gives. A bare `-r __node:=x` must never be passed: it would
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
//!
//! **Start order.** The instances start in the table's order, all at once,
//! with one exception: with static_tf in the set, localize's three wait
//! until its `/tf_static` is on the bus ([`waits_for_static_tf`], E4-4).
//!
//! **Bus.** The process also has one `mower_bus::Bus` and one clock. Every
//! start of an instance runs on a fresh scope of the bus named after its
//! node, and the supervisor closes that scope as soon as the start ends (see
//! `supervise`). Step E2 moves the modules' IO onto it, one module at a time:
//! mower_bridge's `on_bus.rs` says which have moved. When a module of the set
//! has, a bridge module (`bus_bridge`, node `mower_bus_bridge`) is started
//! first and forwards between the bus and DDS what mower_bridge's policy
//! derives for the set; with none, there is no bridge and the process is what
//! it was before E2. A set that would need a name bridged both ways does not
//! start. Whatever the flags, every endpoint an instance makes on the bus is
//! checked against mower_bridge's manifest, and one that is not in it (and so
//! not bridged) is reported on stderr.
//!
//! **ws_bridge's flag is set here, at start.** Its backend is a module
//! argument (`--module-args bridge=... --backend bus`, the launch's
//! `ws_bridge_bus`), so that the app's door can be moved back without a new
//! image. With the bus backend, `bridge` counts as on the bus for the plan,
//! which the bus bridge gets in its arguments (`--on-bus bridge`), and for
//! the manifest check.

use std::collections::BTreeSet;
use std::future::Future;
use std::pin::Pin;
use std::process::ExitCode;
use std::sync::Arc;
use std::time::Duration;

use futures::future::FutureExt;
use mower_bus::{Bus, Clock, Scope};
use mower_rs_common::{ModuleCtx, ModuleResult, Remaps, Shutdown};

/// The bus bridge's module id: not for `--modules`, started when needed.
const BUS_BRIDGE: &str = "bus_bridge";

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
    /// The launch argument that selects this module today, or [`ALWAYS`]
    /// for a module with no rclpy fallback that robot.launch.py always puts
    /// in the set. Only printed by `--list-modules`.
    switch: &'static str,
    instances: &'static [Instance],
}

/// `Module::switch` of a module that has no launch switch: it always runs
/// (coverage: the Python planner it replaced is gone).
const ALWAYS: &str = "(always)";

/// `Module::switch` of the bus bridge, which runs when a module of the set is
/// on the bus.
const AUTO: &str = "(auto)";

/// Every module `mower_rsd` can run. The order is the start order.
const MODULES: &[Module] = &[
    Module {
        // The drivetrain first: it is what everything else waits on, and on
        // a restart the firmware's 300 ms command timeout has already
        // stopped the wheels (docs/ROS_FREE_PLAN.md Phase B).
        id: "base",
        switch: "rust_base",
        instances: &[Instance { node: "mower_base", run: |c, m| Box::pin(mower_base::run(c, m)) }],
    },
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
        // twist_mux (E4, rust_mux, off by default): between the two guards,
        // it reads the manual guard's output and feeds the final guard. Born
        // on the bus; rust_daemon:=false runs the C++ twist_mux.
        id: "mux",
        switch: "rust_mux",
        instances: &[Instance { node: "twist_mux", run: |c, m| Box::pin(mower_mux::run(c, m)) }],
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
        // robot_state_publisher (E4, rust_static_tf, off by default): before
        // localize, whose instances wait for its /tf_static
        // (`waits_for_static_tf`). Born on the bus; rust_daemon:=false runs
        // the C++ rsp.
        id: "static_tf",
        switch: "rust_static_tf",
        instances: &[Instance { node: "static_tf_publisher", run: |c, m| Box::pin(mower_static_tf::run(c, m)) }],
    },
    Module {
        id: "localize",
        switch: "rust_localize",
        // The three nodes of dual_ekf_navsat.launch.py, kept as three
        // instances so a failing filter restarts on its own, exactly as the
        // three launch Nodes did. They share nothing but the process-local
        // transform cache (mower_localize::tfbus).
        instances: &[
            Instance { node: "ekf_filter_node_odom", run: |c, m| Box::pin(mower_localize::run(c, m)) },
            Instance { node: "ekf_filter_node_map", run: |c, m| Box::pin(mower_localize::run(c, m)) },
            Instance { node: "navsat_transform", run: |c, m| Box::pin(mower_localize::run(c, m)) },
        ],
    },
    Module {
        id: "map",
        switch: "rust_map",
        instances: &[Instance { node: "map_manage", run: |c, m| Box::pin(mower_map::run(c, m)) }],
    },
    Module {
        id: "coverage",
        switch: ALWAYS,
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
    Module {
        // The bus/DDS bridge of step E2, never in --modules: main starts it
        // first when a module of the set is on the bus, with the set's ids as
        // its arguments (`bus_bridge`).
        id: BUS_BRIDGE,
        switch: AUTO,
        instances: &[Instance { node: mower_bridge::NODE, run: |c, m| Box::pin(mower_bridge::run(c, m)) }],
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
                let v = value("--stop-timeout")?;
                let s: f64 = v.parse().map_err(|_| format!("--stop-timeout takes seconds, not {v:?}"))?;
                // `try_from_secs_f64` refuses what `from_secs_f64` panics on:
                // a negative, NaN, infinite or too large number of seconds.
                let d = Duration::try_from_secs_f64(s).map_err(|_| {
                    format!("--stop-timeout takes a non-negative, finite number of seconds, not {v:?}")
                })?;
                a.stop_timeout = d.max(Duration::from_millis(100));
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
    let known: BTreeSet<&str> = MODULES.iter().map(|m| m.id).filter(|id| *id != BUS_BRIDGE).collect();
    for m in &a.modules {
        if m == BUS_BRIDGE {
            return Err(format!("{BUS_BRIDGE} is not for --modules: it starts on its own when a module is on the bus"));
        }
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

/// The modules on the bus by how they run in this process rather than by
/// mower_bridge's on_bus.rs (`mower_bridge::RUNTIME`): ws_bridge when its
/// module arguments ask for the bus backend. A `--backend` value ws_bridge
/// refuses counts as r2r here; ws_bridge reports it when it starts.
fn on_bus_now(args: &Args) -> Vec<&'static str> {
    let bridge_args: Vec<String> = args.module_args.iter().filter(|(id, _)| id == "bridge").flat_map(|(_, v)| v.clone()).collect();
    let ws_bus = args.modules.iter().any(|m| m == "bridge")
        && mower_ws_bridge::backend_of(&bridge_args) == Ok(mower_ws_bridge::Backend::Bus);
    if ws_bus {
        vec!["bridge"]
    } else {
        Vec::new()
    }
}

/// The bus bridge's arguments (`mower_bridge::rsd_args`: the ids of
/// `--modules` and the run-time flags `on_now`), when a module of the set is
/// on the bus; `None` when none is, and then no bridge runs. An error for a
/// set mower_bridge's policy cannot bridge (a name both ways).
fn bus_bridge(modules: &[String], on_now: &[&str], remaps: &Remaps) -> Result<Option<Vec<String>>, String> {
    let plan = mower_bridge::rsd_plan_with(modules, on_now, remaps).map_err(|e| e.to_string())?;
    Ok((!plan.is_empty()).then(|| mower_bridge::rsd_args(modules, on_now)))
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
        Err(p) => Outcome::Failed(format!("{label}: panicked: {}", panic_message(&*p))),
    }
}

/// What a caught panic says, for the log.
fn panic_message(panic: &(dyn std::any::Any + Send)) -> String {
    panic
        .downcast_ref::<&str>()
        .map(|s| s.to_string())
        .or_else(|| panic.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "panic".to_string())
}

/// Close an instance's scope, logging a panic instead of passing it on. A
/// close runs user code (drop glue of the messages it frees, wakers of the
/// tasks it wakes); if that panics, the close still ends everything else in
/// the scope and then re-raises (mower_bus README, "Model"), and the
/// supervisor must carry on.
fn close_scope(label: &str, scope: &Scope) {
    if let Err(p) = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| scope.close())) {
        eprintln!("[mower_rsd] {label}: closing its bus scope panicked: {}", panic_message(&*p));
    }
}

/// One instance's module body, bound to the process's r2r context. The
/// supervisor calls it once per start.
type Start = Arc<dyn Fn(ModuleCtx) -> BoxFut + Send + Sync>;

/// Whether module `id`'s instances wait for static_tf's `/tf_static` before
/// they start: localize's do while static_tf is in the set (E4-4).
///
/// navsat builds the datum `delay` (3 s) after it starts, from the map EKF's
/// heading, and the EKFs fuse the IMU only once they know how it is mounted:
/// imu_link and base_link, fixed joints of the URDF, on `/tf_static`. From
/// DDS, robot_state_publisher's message can come seconds late on a loaded
/// host, and a datum built before the map EKF had it is rotated by up to the
/// robot's heading. With static_tf in the process the order is certain: its
/// latched message is on the bus before localize makes a reader, so the
/// EKFs have the mounts from their first IMU sample and navsat's delay is
/// theirs to converge in. Without static_tf (the C++ rsp, its frames from
/// DDS) nothing waits, as before: holding localize back for rsp would change
/// what production does.
fn waits_for_static_tf(id: &str, set: &[String]) -> bool {
    id == "localize" && set.iter().any(|m| m == "static_tf")
}

/// The slot of instance `node` of module `module` in the set `set`, held
/// back for static_tf's `/tf_static` when [`waits_for_static_tf`] says so.
fn slot(
    module: &str, node: &'static str, set: &[String], extra: Vec<String>, remaps: Arc<Remaps>, start: Start,
) -> Slot {
    let label = format!("{module}/{node}");
    let start = if waits_for_static_tf(module, set) { after_static_tf(label.clone(), start) } else { start };
    Slot { label, node, extra, remaps, start }
}

/// `start`, once static_tf's `/tf_static` is on the bus: on its first poll,
/// a reader made on the instance's scope has the latched message.
fn after_static_tf(label: String, start: Start) -> Start {
    Arc::new(move |m: ModuleCtx| -> BoxFut {
        let (label, start) = (label.clone(), start.clone());
        Box::pin(async move {
            if !wait_for_tf_static(&label, &m).await? {
                // the process stops: nothing was started
                return Ok(());
            }
            start(m).await
        })
    })
}

/// Wait on `m`'s scope until a `/tf_static` message with transforms is on
/// the bus; false when the process stops first. The reader has the EKFs' and
/// navsat's QoS (R TL 100), the endpoint the manifest has for these nodes,
/// and is gone before the instance starts.
async fn wait_for_tf_static(label: &str, m: &ModuleCtx) -> Result<bool, mower_rs_common::BoxError> {
    let topic = m.resolve_topic("/tf_static")?;
    let qos = mower_bus::Qos::default().keep_last(100).reliable().transient_local();
    let mut tf_static = m.bus.subscribe::<r2r::tf2_msgs::msg::TFMessage>(&topic, qos)?;
    let mut waited = Duration::ZERO;
    loop {
        tokio::select! {
            biased;
            () = m.shutdown.wait() => return Ok(false),
            msg = futures::StreamExt::next(&mut tf_static) => match msg {
                Some(msg) if !msg.transforms.is_empty() => {
                    if !waited.is_zero() {
                        println!("[mower_rsd] {label}: static_tf's {topic} is on the bus");
                    }
                    return Ok(true);
                }
                Some(_) => {}
                None => return Err(format!("the bus closed while waiting for static_tf's {topic}").into()),
            },
            () = tokio::time::sleep(WAIT_REPORT) => {
                waited += WAIT_REPORT;
                println!(
                    "[mower_rsd] {label}: waiting for static_tf's {topic} ({:.0} s): navsat's datum needs the \
                     sensor mounts",
                    waited.as_secs_f64()
                );
            }
        }
    }
}

/// How often [`wait_for_tf_static`] says it is still waiting. On a healthy
/// start static_tf publishes within milliseconds and nothing is printed.
const WAIT_REPORT: Duration = Duration::from_secs(5);

/// One node instance under supervision. A slot reports every time its
/// instance ends; [`supervise`] starts it again (after RESTART_DELAY, the
/// launch files' respawn_delay) unless the process is shutting down.
struct Slot {
    label: String,
    node: &'static str,
    /// Its module's `--module-args`.
    extra: Vec<String>,
    /// The process's `-r` rules, for the bus names (`ModuleCtx::remaps`).
    remaps: Arc<Remaps>,
    start: Start,
}

/// Run the slots until the process stops, and return the failures that must
/// make it exit non-zero.
///
/// Every start of an instance runs on a fresh scope of `bus` named after its
/// node. The scope is closed as soon as that start ends, and the scopes still
/// open when the supervisor stops waiting are closed on the way out.
async fn supervise(
    slots: Vec<Slot>, bus: Bus, clock: Clock, shutdown: Shutdown, stop_timeout: Duration,
) -> Vec<String> {
    let (report, mut reports) = tokio::sync::mpsc::unbounded_channel::<(usize, Outcome)>();
    // Start a slot (after `delay` for a restart); returns the scope it runs on.
    let launch = |slot_idx: usize, delay: Duration| -> Scope {
        let slot = &slots[slot_idx];
        let scope = bus.scope(slot.node);
        let m = ModuleCtx::new(slot.node, shutdown.clone())
            .with_args(slot.extra.clone())
            .with_bus(scope.clone())
            .with_clock(clock.clone())
            .with_remaps(slot.remaps.clone());
        let start = slot.start.clone();
        let label = slot.label.clone();
        let stopping = shutdown.clone();
        let report = report.clone();
        let ended = scope.clone();
        tokio::spawn(async move {
            if !delay.is_zero() {
                // A shutdown cuts the delay short: the slot reports at once
                // instead of spending the stop window on a restart that is
                // not coming (with `--stop-timeout` under RESTART_DELAY, it
                // counted as a module that did not stop).
                tokio::select! {
                    () = tokio::time::sleep(delay) => {}
                    () = stopping.wait() => {}
                }
                if stopping.is_triggered() {
                    // Never started: its scope is closed with the rest on the
                    // way out.
                    let _ = report.send((slot_idx, Outcome::Stopped(label)));
                    return;
                }
                println!("[mower_rsd] restarting {label}");
            } else {
                println!("[mower_rsd] starting {label}");
            }
            let outcome = supervised(label.clone(), stopping, start(m)).await;
            // However the instance ended, close its scope now, before any
            // restart delay. Nothing aborts the tasks it spawned: with r2r its
            // dropped node made their publishes fail, and a closed scope does
            // the same on the bus (their streams end, publishes fail, the
            // services vanish, timer loops end), so they cannot carry on next
            // to the restart. A close that panics must not cost the report:
            // without it the slot would never be restarted.
            close_scope(&label, &ended);
            let _ = report.send((slot_idx, outcome));
        });
        scope
    };
    let mut scopes: Vec<Scope> = (0..slots.len()).map(|i| launch(i, Duration::ZERO)).collect();
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
            match tokio::time::timeout(stop_timeout, reports.recv()).await {
                Ok(v) => v,
                Err(_) => {
                    let why = format!(
                        "{} module(s) did not stop within {:.1} s",
                        alive,
                        stop_timeout.as_secs_f64()
                    );
                    eprintln!("[mower_rsd] {why}");
                    failures.push(why);
                    break;
                }
            }
        } else {
            // A shutdown that trips while nothing reports must still start
            // the stop window above, or one module that ignores it keeps the
            // process up forever. `recv` is cancel-safe: a report that races
            // the shutdown is read on the next turn.
            tokio::select! {
                next = reports.recv() => next,
                () = shutdown.wait() => continue,
            }
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
                scopes[slot_idx] = launch(slot_idx, RESTART_DELAY);
            }
        }
    }
    drop(report);
    // Every instance has returned or had its `stop_timeout`. Close the scopes
    // of those that did not return, for the reason above: what they still
    // run must not go on using the bus while the process goes down. One that
    // panics does not keep the others open.
    for (slot, scope) in slots.iter().zip(&scopes) {
        close_scope(&slot.label, scope);
    }
    failures
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
    // The `-r` rules rcl applies to every module's node, for their bus names.
    let remaps = match Remaps::from_args(&argv) {
        Ok(r) => Arc::new(r),
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
        // One bus and one clock for the process as well; step E2 moves the
        // modules' IO from r2r onto them.
        let bus = Bus::new();
        let clock = Clock::system();
        let on_now = on_bus_now(&args);
        let bridged = match bus_bridge(&args.modules, &on_now, &remaps) {
            Ok(bridged) => bridged,
            Err(e) => {
                eprintln!("[mower_rsd] {e}");
                return ExitCode::FAILURE;
            }
        };

        let mut slots: Vec<Slot> = Vec::new();
        let mut add = |module: &Module, extra: Vec<String>| {
            for inst in module.instances {
                let (ctx, run) = (ctx.clone(), inst.run);
                let start: Start = Arc::new(move |m: ModuleCtx| run(ctx.clone(), m));
                slots.push(slot(module.id, inst.node, &args.modules, extra.clone(), remaps.clone(), start));
            }
        };
        // The bridge first, so that its bus ends are there when the modules
        // look for them.
        if let Some(ids) = bridged {
            add(MODULES.iter().find(|m| m.id == BUS_BRIDGE).expect("in the table"), ids);
        }
        for id in &args.modules {
            let module = MODULES.iter().find(|m| m.id == *id).expect("checked in parse_args");
            let extra: Vec<String> = args
                .module_args
                .iter()
                .filter(|(mid, _)| mid == id)
                .flat_map(|(_, v)| v.clone())
                .collect();
            add(module, extra);
        }
        // An endpoint a module makes on the bus that the manifest does not
        // have is a name nothing bridges: say so, with a bridge or without.
        let running = mower_bridge::Running::rsd(&args.modules, mower_bridge::on_bus_with(&on_now));
        let bridge_scopes = vec![mower_bridge::NODE.to_string()];
        tokio::spawn(mower_bridge::coverage::watch(bus.clone(), running, remaps.clone(), bridge_scopes, |gap| {
            eprintln!("[mower_rsd] bus bridge: {gap}")
        }));
        let failures = supervise(slots, bus, clock, shutdown, args.stop_timeout).await;

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
    use std::sync::Mutex;
    use std::task::{Context, Wake, Waker};

    use futures::Stream;
    use mower_bus::{Qos, Subscriber};
    use r2r::geometry_msgs::msg::TransformStamped;
    use r2r::tf2_msgs::msg::TFMessage;

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
            ("base", vec!["mower_base"]),
            ("status", vec!["robot_status"]),
            ("guards", vec!["manual_velocity_guard", "velocity_command_guard"]),
            ("mux", vec!["twist_mux"]),
            ("imu", vec!["imu"]),
            ("gps", vec!["gps"]),
            ("static_tf", vec!["static_tf_publisher"]),
            ("localize", vec!["ekf_filter_node_odom", "ekf_filter_node_map", "navsat_transform"]),
            ("map", vec!["map_manage"]),
            ("coverage", vec!["boustrophedon_coverage"]),
            ("nav", vec!["nav_action_server"]),
            ("record", vec!["path_record_node"]),
            ("adapter", vec!["flutter_adapter"]),
            ("battery", vec!["battery_state"]),
            ("pid_autotune", vec!["pid_autotune"]),
            ("bridge", vec!["mower_ws_bridge"]),
            ("agent", vec!["mower_agent"]),
            ("bus_bridge", vec!["mower_bus_bridge"]),
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

    /// What one start of a test instance does.
    #[derive(Clone, Copy, Debug)]
    enum Run {
        /// Returns `Err` at once.
        Fail,
        /// Panics at once.
        Panic,
        /// Returns `Ok` at once, without being asked to stop.
        Quit,
        /// Returns `Ok` once asked to stop.
        Serve,
        /// Never returns, not even when asked to stop.
        Wedge,
    }

    /// A slot for `node` whose n-th start does `runs[n]` (the last entry for
    /// every later start) and leaves the scope it was given in `scopes`.
    fn test_slot(node: &'static str, runs: &[Run], scopes: &Arc<Mutex<Vec<Scope>>>) -> Slot {
        let (runs, scopes) = (runs.to_vec(), scopes.clone());
        Slot {
            label: format!("test/{node}"),
            node,
            extra: Vec::new(),
            remaps: Arc::default(),
            start: Arc::new(move |m: ModuleCtx| -> BoxFut {
                let mut seen = scopes.lock().unwrap();
                let run = runs[seen.len().min(runs.len() - 1)];
                seen.push(m.bus.clone());
                Box::pin(async move {
                    match run {
                        Run::Fail => Err("port went away".into()),
                        Run::Panic => panic!("serial exploded"),
                        Run::Quit => Ok(()),
                        Run::Serve => {
                            m.shutdown.wait().await;
                            Ok(())
                        }
                        Run::Wedge => std::future::pending().await,
                    }
                })
            }),
        }
    }

    /// tokio's paused clock: once every task is idle, time jumps to the next
    /// timer, so RESTART_DELAY and the stop timeout pass at once and in order.
    fn paused() -> tokio::runtime::Runtime {
        tokio::runtime::Builder::new_current_thread().enable_all().start_paused(true).build().unwrap()
    }

    /// What static_tf publishes that the localize nodes look up: the robot's
    /// mounts of the GPS antenna and the IMU (the real one has 7 joints).
    fn mounts() -> TFMessage {
        let joint = |parent: &str, child: &str| TransformStamped {
            header: r2r::std_msgs::msg::Header { frame_id: parent.into(), ..Default::default() },
            child_frame_id: child.into(),
            ..Default::default()
        };
        TFMessage {
            transforms: vec![
                joint("base_footprint", "base_link"),
                joint("base_link", "gps_link"),
                joint("base_link", "imu_link"),
            ],
        }
    }

    fn has_the_mounts(msg: &TFMessage) -> bool {
        ["base_link", "gps_link", "imu_link"].iter().all(|c| msg.transforms.iter().any(|t| t.child_frame_id == *c))
    }

    /// A static_tf that publishes [`mounts`] latched `after` its start, as
    /// the module does at once.
    fn static_tf_slot(set: &[String], after: Duration) -> Slot {
        let start: Start = Arc::new(move |m: ModuleCtx| -> BoxFut {
            Box::pin(async move {
                tokio::select! {
                    () = tokio::time::sleep(after) => {}
                    () = m.shutdown.wait() => return Ok(()),
                }
                let tf_static = m.bus.publisher::<TFMessage>(&m.resolve_topic("/tf_static")?, Qos::latched())?;
                tf_static.publish_owned(mounts())?;
                m.shutdown.wait().await;
                Ok(())
            })
        });
        slot("static_tf", "static_tf_publisher", set, Vec::new(), Arc::default(), start)
    }

    /// Each start of a localize instance: when it began, and whether the
    /// latched reader it makes first (the EKFs' and navsat's, R TL 100) had
    /// the mounts on its first poll. Its n-th start does `runs[n]`.
    type Starts = Arc<Mutex<Vec<(&'static str, Duration, bool)>>>;

    fn localize_slots(set: &[String], runs: &[Run], starts: &Starts) -> Vec<Slot> {
        let t0 = tokio::time::Instant::now();
        ["ekf_filter_node_odom", "ekf_filter_node_map", "navsat_transform"]
            .into_iter()
            .map(|node| {
                let (runs, starts) = (runs.to_vec(), starts.clone());
                let start: Start = Arc::new(move |m: ModuleCtx| -> BoxFut {
                    let (runs, starts) = (runs.clone(), starts.clone());
                    Box::pin(async move {
                        let qos = Qos::default().keep_last(100).reliable().transient_local();
                        let mut tf_static = m.bus.subscribe::<TFMessage>(&m.resolve_topic("/tf_static")?, qos)?;
                        let has = matches!(
                            futures::StreamExt::next(&mut tf_static).now_or_never(),
                            Some(Some(msg)) if has_the_mounts(&msg)
                        );
                        let run = {
                            let mut seen = starts.lock().unwrap();
                            let n = seen.iter().filter(|(of, _, _)| *of == node).count();
                            seen.push((node, t0.elapsed(), has));
                            runs[n.min(runs.len() - 1)]
                        };
                        match run {
                            Run::Fail => Err("filter diverged".into()),
                            _ => {
                                m.shutdown.wait().await;
                                Ok(())
                            }
                        }
                    })
                });
                slot("localize", node, set, Vec::new(), Arc::default(), start)
            })
            .collect()
    }

    /// E4-4, the datum race: navsat's datum takes the map EKF's heading 3 s
    /// after the start, and the EKFs fuse the IMU only once they have its
    /// mount from /tf_static. With static_tf in the set, localize's three
    /// instances start once its message is on the bus, however late static_tf
    /// is (here 3.5 s, past navsat's delay: what DDS did to rsp's frames on a
    /// loaded host), and each has the mounts on its first poll. Without
    /// static_tf (the C++ rsp) they start at once, as before.
    #[test]
    fn with_static_tf_in_the_set_localize_starts_once_the_mounts_are_on_the_bus() {
        let set = argv(&["imu", "gps", "static_tf", "localize", "nav"]);
        assert!(waits_for_static_tf("localize", &set));
        for id in ["base", "status", "guards", "imu", "gps", "static_tf", "map", "coverage", "nav", "bridge"] {
            assert!(!waits_for_static_tf(id, &set), "{id} waits");
        }
        let without = argv(&["imu", "gps", "localize", "nav"]);
        assert!(!waits_for_static_tf("localize", &without), "the C++ rsp: nothing waits");

        let late = Duration::from_millis(3500);
        paused().block_on(async {
            let starts: Starts = Arc::default();
            let shutdown = Shutdown::new();
            let mut slots = vec![static_tf_slot(&set, late)];
            slots.extend(localize_slots(&set, &[Run::Serve], &starts));
            let stop_timeout = Duration::from_secs(1);
            let supervisor =
                tokio::spawn(supervise(slots, Bus::new(), Clock::system(), shutdown.clone(), stop_timeout));
            tokio::time::sleep(late / 2).await;
            assert!(starts.lock().unwrap().is_empty(), "localize started before static_tf published");
            tokio::time::sleep(late).await;
            {
                let seen = starts.lock().unwrap();
                assert_eq!(seen.len(), 3, "{seen:?}");
                for (node, at, has) in seen.iter() {
                    assert!(*at >= late, "{node} started at {at:?}, before static_tf's message at {late:?}");
                    assert!(*has, "{node}: no mounts on the first poll of its latched reader");
                }
            }
            shutdown.trigger();
            assert!(supervisor.await.unwrap().is_empty());
        });

        // without static_tf in the set: at once, and nothing on the bus yet
        paused().block_on(async {
            let starts: Starts = Arc::default();
            let shutdown = Shutdown::new();
            let slots = localize_slots(&without, &[Run::Serve], &starts);
            let stop_timeout = Duration::from_secs(1);
            let supervisor =
                tokio::spawn(supervise(slots, Bus::new(), Clock::system(), shutdown.clone(), stop_timeout));
            tokio::time::sleep(Duration::from_millis(10)).await;
            {
                let seen = starts.lock().unwrap();
                assert_eq!(seen.len(), 3, "{seen:?}");
                assert!(seen.iter().all(|(_, at, has)| at.is_zero() && !has), "{seen:?}");
            }
            shutdown.trigger();
            assert!(supervisor.await.unwrap().is_empty());
        });
    }

    /// The wait is part of every start: a localize instance restarted after a
    /// failure finds the latched message at once. One that is still waiting
    /// when the process stops ends at once, never having started.
    #[test]
    fn a_restarted_localize_instance_finds_the_mounts_and_a_waiting_one_stops_at_once() {
        let set = argv(&["static_tf", "localize"]);
        paused().block_on(async {
            let starts: Starts = Arc::default();
            let shutdown = Shutdown::new();
            let mut slots = vec![static_tf_slot(&set, Duration::from_millis(100))];
            slots.extend(localize_slots(&set, &[Run::Fail, Run::Serve], &starts));
            let stop_timeout = Duration::from_secs(1);
            let supervisor =
                tokio::spawn(supervise(slots, Bus::new(), Clock::system(), shutdown.clone(), stop_timeout));
            tokio::time::sleep(Duration::from_millis(100) + RESTART_DELAY * 2).await;
            {
                let seen = starts.lock().unwrap();
                assert_eq!(seen.len(), 6, "each instance failed once and started again: {seen:?}");
                assert!(seen.iter().all(|(_, _, has)| *has), "{seen:?}");
                let second: Vec<Duration> = seen[3..].iter().map(|(_, at, _)| *at).collect();
                assert!(second.iter().all(|at| *at == Duration::from_millis(100) + RESTART_DELAY), "{seen:?}");
            }
            shutdown.trigger();
            assert!(supervisor.await.unwrap().is_empty());
        });

        // static_tf never publishes: localize waits, saying so, and a stop
        // ends it within the stop window
        paused().block_on(async {
            let starts: Starts = Arc::default();
            let shutdown = Shutdown::new();
            let mut slots = vec![static_tf_slot(&set, Duration::from_secs(3600))];
            slots.extend(localize_slots(&set, &[Run::Serve], &starts));
            let stop_timeout = Duration::from_secs(1);
            let supervisor =
                tokio::spawn(supervise(slots, Bus::new(), Clock::system(), shutdown.clone(), stop_timeout));
            tokio::time::sleep(WAIT_REPORT * 3).await;
            assert!(starts.lock().unwrap().is_empty());
            shutdown.trigger();
            let failures = tokio::time::timeout(stop_timeout / 2, supervisor).await.expect("stopped at once").unwrap();
            assert!(failures.is_empty(), "{failures:?}");
            assert!(starts.lock().unwrap().is_empty(), "started after the stop");
        });
    }

    /// Every start resolves its bus names with the process's `-r` rules,
    /// under its own node name (docs/BUS_TOPICS.md 5.4).
    #[test]
    fn every_start_resolves_bus_names_with_the_process_remap_rules() {
        let argv: Vec<String> = "--modules imu,guards --ros-args -r imu:imu/data_raw:=imu/data \
                                 -r velocity_command_guard:cmd_vel_in:=/cmd_vel_guard_input"
            .split_whitespace()
            .map(str::to_string)
            .collect();
        let remaps = Arc::new(Remaps::from_args(&argv).unwrap());
        paused().block_on(async {
            let seen: Arc<Mutex<Vec<String>>> = Arc::default();
            let shutdown = Shutdown::new();
            let slots = ["imu", "velocity_command_guard"]
                .into_iter()
                .map(|node| {
                    let (seen, remaps) = (seen.clone(), remaps.clone());
                    Slot {
                        label: format!("test/{node}"),
                        node,
                        extra: Vec::new(),
                        remaps,
                        start: Arc::new(move |m: ModuleCtx| -> BoxFut {
                            let names = [m.resolve_topic("imu/data_raw"), m.resolve_topic("cmd_vel_in")];
                            seen.lock().unwrap().extend(names.map(Result::unwrap));
                            Box::pin(async move {
                                m.shutdown.wait().await;
                                Ok(())
                            })
                        }),
                    }
                })
                .collect();
            let run = tokio::spawn(supervise(slots, Bus::new(), Clock::system(), shutdown.clone(), Duration::from_secs(1)));
            tokio::time::sleep(Duration::from_millis(10)).await;
            shutdown.trigger();
            assert!(run.await.unwrap().is_empty());
            assert_eq!(*seen.lock().unwrap(), ["/imu/data", "/cmd_vel_in", "/imu/data_raw", "/cmd_vel_guard_input"]);
        });
    }

    /// Tasks a failed instance spawned are not aborted, so the closed scope
    /// is what ends their publishes; it must be closed before the restart.
    #[test]
    fn however_an_instance_ends_its_scope_is_closed_and_the_restart_gets_a_fresh_one() {
        for first in [Run::Fail, Run::Panic, Run::Quit] {
            paused().block_on(async {
                let scopes = Arc::new(Mutex::new(Vec::new()));
                let shutdown = Shutdown::new();
                let slots = vec![test_slot("imu", &[first, Run::Serve], &scopes)];
                let stop_timeout = Duration::from_secs(10);
                let supervisor =
                    tokio::spawn(supervise(slots, Bus::new(), Clock::system(), shutdown.clone(), stop_timeout));

                // The first start ended at once; the restart is RESTART_DELAY away.
                tokio::time::sleep(RESTART_DELAY / 2).await;
                {
                    let s = scopes.lock().unwrap();
                    assert_eq!(s.len(), 1, "{first:?}: restarted before RESTART_DELAY");
                    assert!(s[0].is_closed(), "{first:?}: scope still open during the restart delay");
                }
                tokio::time::sleep(RESTART_DELAY).await;
                {
                    let s = scopes.lock().unwrap();
                    assert_eq!(s.len(), 2, "{first:?}: not restarted");
                    assert_eq!(s[1].name(), "imu");
                    assert!(!s[1].is_closed(), "{first:?}: the restart runs on a closed scope");
                }

                // asked to stop and stopped: closed as well
                shutdown.trigger();
                let failures = supervisor.await.unwrap();
                assert!(failures.is_empty(), "{first:?}: {failures:?}");
                assert!(scopes.lock().unwrap()[1].is_closed(), "{first:?}: stopped, scope still open");
            });
        }
    }

    /// The stop window is the instances' to unwind in, so a scope stays open
    /// until its instance returns or the window is over.
    #[test]
    fn an_instance_that_does_not_stop_in_time_has_its_scope_closed_when_the_window_ends() {
        paused().block_on(async {
            let status = Arc::new(Mutex::new(Vec::new()));
            let imu = Arc::new(Mutex::new(Vec::new()));
            let shutdown = Shutdown::new();
            let slots = vec![
                test_slot("robot_status", &[Run::Serve], &status),
                test_slot("imu", &[Run::Wedge], &imu),
            ];
            let stop_timeout = Duration::from_secs(10);
            let supervisor =
                tokio::spawn(supervise(slots, Bus::new(), Clock::system(), shutdown.clone(), stop_timeout));
            tokio::time::sleep(Duration::from_secs(1)).await;
            {
                let (s, i) = (status.lock().unwrap(), imu.lock().unwrap());
                assert_eq!((s[0].name(), i[0].name()), ("robot_status", "imu"));
                assert!(!s[0].is_closed() && !i[0].is_closed());
            }

            shutdown.trigger();
            tokio::time::sleep(stop_timeout / 2).await;
            assert!(status.lock().unwrap()[0].is_closed(), "robot_status stopped, scope still open");
            assert!(!imu.lock().unwrap()[0].is_closed(), "imu's scope closed inside its stop window");

            let failures = supervisor.await.unwrap();
            assert!(matches!(&failures[..], [why] if why.contains("did not stop")), "{failures:?}");
            assert!(imu.lock().unwrap()[0].is_closed(), "the window is over, imu's scope still open");
        });
    }

    /// The supervisor used to wait for a report without a timeout when the
    /// shutdown tripped during that wait, so an instance that ignored the
    /// shutdown, with no other instance left to report, kept the process up.
    #[test]
    fn a_shutdown_with_no_instance_reporting_still_ends_after_the_stop_window() {
        paused().block_on(async {
            let imu = Arc::new(Mutex::new(Vec::new()));
            let shutdown = Shutdown::new();
            let slots = vec![test_slot("imu", &[Run::Wedge], &imu)];
            let stop_timeout = Duration::from_secs(10);
            let supervisor =
                tokio::spawn(supervise(slots, Bus::new(), Clock::system(), shutdown.clone(), stop_timeout));
            tokio::time::sleep(Duration::from_secs(1)).await;

            shutdown.trigger();
            let failures = tokio::time::timeout(stop_timeout * 2, supervisor)
                .await
                .expect("the stop window never started")
                .unwrap();
            assert!(matches!(&failures[..], [why] if why.contains("did not stop")), "{failures:?}");
            assert!(imu.lock().unwrap()[0].is_closed(), "the window is over, imu's scope still open");
        });
    }

    /// With `--stop-timeout` under RESTART_DELAY, a slot waiting out its
    /// restart delay when the shutdown came used to count as a module that
    /// did not stop: it reported only once the delay was over.
    #[test]
    fn a_shutdown_during_a_restart_delay_stops_the_slot_at_once() {
        paused().block_on(async {
            let scopes = Arc::new(Mutex::new(Vec::new()));
            let shutdown = Shutdown::new();
            let slots = vec![test_slot("imu", &[Run::Fail, Run::Serve], &scopes)];
            let stop_timeout = RESTART_DELAY / 2;
            let supervisor =
                tokio::spawn(supervise(slots, Bus::new(), Clock::system(), shutdown.clone(), stop_timeout));
            // The first start failed at once; the slot is in its restart delay.
            tokio::time::sleep(RESTART_DELAY / 4).await;

            shutdown.trigger();
            let tripped = tokio::time::Instant::now();
            let failures = supervisor.await.unwrap();
            assert!(failures.is_empty(), "{failures:?}");
            assert!(tripped.elapsed() < stop_timeout, "stopped after {:?}", tripped.elapsed());
            let s = scopes.lock().unwrap();
            assert_eq!(s.len(), 1, "restarted after the shutdown");
            assert!(s[0].is_closed());
        });
    }

    /// A waker that panics when woken: user code that blows up inside a
    /// scope's close, which wakes every task waiting on the scope's readers.
    struct Exploding;

    impl Wake for Exploding {
        fn wake(self: Arc<Self>) {
            panic!("waker exploded");
        }
    }

    /// The instance task used to unwind out of the scope's close before it
    /// sent its report, so the slot was never restarted and, at shutdown,
    /// counted as a module that did not stop.
    #[test]
    fn a_close_that_panics_is_logged_and_the_instance_still_reports_and_restarts() {
        paused().block_on(async {
            let scopes = Arc::new(Mutex::new(Vec::new()));
            let parked: Arc<Mutex<Vec<Subscriber<String>>>> = Arc::default();
            let shutdown = Shutdown::new();
            let slot = {
                let (scopes, parked) = (scopes.clone(), parked.clone());
                Slot {
                    label: "test/imu".into(),
                    node: "imu",
                    extra: Vec::new(),
                    remaps: Arc::default(),
                    start: Arc::new(move |m: ModuleCtx| -> BoxFut {
                        let mut seen = scopes.lock().unwrap();
                        seen.push(m.bus.clone());
                        if seen.len() > 1 {
                            return Box::pin(async move {
                                m.shutdown.wait().await;
                                Ok(())
                            });
                        }
                        // the first start leaves a reader whose task's waker
                        // panics when the close wakes it, then fails
                        let mut sub = m.bus.subscribe::<String>("/imu/data_raw", Qos::default()).unwrap();
                        let waker = Waker::from(Arc::new(Exploding));
                        assert!(Pin::new(&mut sub).poll_next(&mut Context::from_waker(&waker)).is_pending());
                        parked.lock().unwrap().push(sub);
                        Box::pin(async { Err("port went away".into()) })
                    }),
                }
            };
            let stop_timeout = Duration::from_secs(10);
            let supervisor =
                tokio::spawn(supervise(vec![slot], Bus::new(), Clock::system(), shutdown.clone(), stop_timeout));

            tokio::time::sleep(RESTART_DELAY * 2).await;
            {
                let s = scopes.lock().unwrap();
                assert_eq!(s.len(), 2, "not restarted after its close panicked");
                assert!(s[0].is_closed(), "the panic left the scope open");
                assert!(!s[1].is_closed());
            }
            shutdown.trigger();
            let failures = supervisor.await.unwrap();
            assert!(failures.is_empty(), "{failures:?}");
        });
    }

    #[test]
    fn stop_timeout_takes_a_non_negative_finite_number_of_seconds() {
        let stop = |v: &str| parse_args(&argv(&["--modules", "status", "--stop-timeout", v])).map(|a| a.stop_timeout);
        assert_eq!(stop("2.5"), Ok(Duration::from_millis(2500)));
        assert_eq!(stop("0"), Ok(Duration::from_millis(100)), "at least 0.1 s, as before");
        assert_eq!(
            parse_args(&argv(&["--modules=status", "--stop-timeout=3"])).map(|a| a.stop_timeout),
            Ok(Duration::from_secs(3))
        );
        // `inf` used to panic in Duration::from_secs_f64; negative and NaN
        // values were quietly taken as 0.1 s
        for bad in ["inf", "-inf", "infinity", "NaN", "nan", "-1", "-0.5", "1e300", "soon", ""] {
            assert!(stop(bad).is_err(), "{bad:?} was taken");
        }
    }

    /// With every flag of mower_bridge's on_bus.rs false, no bridge runs and
    /// the process is what it was before E2; one runs, with the set's ids, as
    /// soon as a module of the set is on the bus.
    #[test]
    fn the_bus_bridge_runs_only_for_a_set_with_a_module_on_the_bus() {
        let production: Vec<String> =
            MODULES.iter().map(|m| m.id.to_string()).filter(|id| id != BUS_BRIDGE).collect();
        let remaps = mower_bridge::production_remaps();
        let bridged = bus_bridge(&production, &[], &remaps).unwrap();
        assert_eq!(bridged.is_some(), production.iter().any(|id| mower_bridge::on_bus(id)));
        if let Some(ids) = bridged {
            assert_eq!(ids, production);
        }
        // ws_bridge on r2r and the agent have nothing on the bus
        assert_eq!(bus_bridge(&argv(&["bridge", "agent"]), &[], &remaps), Ok(None));
        // ws_bridge on the bus has: a bridge runs for it, told so
        assert_eq!(
            bus_bridge(&argv(&["bridge", "agent"]), &["bridge"], &remaps),
            Ok(Some(argv(&["bridge", "agent", "--on-bus", "bridge"])))
        );
        assert!(parse_args(&argv(&["--modules", "status,bus_bridge"])).is_err(), "not for --modules");
    }

    /// ws_bridge's backend, from its module arguments as robot.launch.py
    /// writes them: the bus only with `--backend bus`, and only while
    /// ws_bridge runs here.
    #[test]
    fn ws_bridge_is_on_the_bus_with_its_bus_backend() {
        let policy = "--address 10.77.0.2 --port 9090 --loopback-port 9091 --policy /x/ws_bridge.yaml";
        let with = |modules: &str, bridge_args: Option<String>| {
            let mut a = vec!["--modules".to_string(), modules.to_string()];
            if let Some(b) = bridge_args {
                a.extend(["--module-args".to_string(), format!("bridge={b}")]);
            }
            on_bus_now(&parse_args(&a).unwrap())
        };
        assert_eq!(with("status,bridge,agent", Some(policy.to_string())), Vec::<&str>::new());
        assert_eq!(with("status,bridge,agent", Some(format!("{policy} --backend r2r"))), Vec::<&str>::new());
        assert_eq!(with("status,bridge,agent", Some(format!("{policy} --backend bus"))), ["bridge"]);
        // ws_bridge refuses it when it starts; the plan stays E2's
        assert_eq!(with("status,bridge", Some(format!("{policy} --backend dds"))), Vec::<&str>::new());
        assert_eq!(with("status,bridge", None), Vec::<&str>::new());
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
