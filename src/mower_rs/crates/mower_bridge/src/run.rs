//! The two ways a bridge runs: as mower_rsd's bridge module, for the
//! instances of `--modules`, and next to a module in its own binary, where
//! every endpoint of the module faces DDS (README.md, "Running").

use std::future::Future;

use mower_bus::{Bus, Scope};
use mower_bus_ros::{BridgeStats, Direction, EntryStats, RosBridge};
use mower_rs_common::{BoxError, ModuleCtx, ModuleResult, Remaps};
use serde_json::{json, Value};

use crate::policy::{plan, Plan, PlanError, Running};

/// The bridge module's node in mower_rsd. A node of its own: a moved module
/// keeps its r2r node, and two nodes of one name must not be (mower_bus_ros
/// README, "The node").
pub const NODE: &str = "mower_bus_bridge";

/// The namespace of a standalone binary's bridge, whose node takes the
/// module's node name. Launch's `name=` gives a binary a bare `-r __node:=`,
/// which renames every node of the process: a bridge node of a name of its
/// own would come out with the module's, and two nodes would share a fully
/// qualified name. In this namespace it never does.
pub const STANDALONE_NAMESPACE: &str = "/mower_bus_bridge";

/// The plan of mower_rsd's bridge for the modules of `--modules`, with the
/// flags of `on_bus.rs`. Empty while none of them is on the bus: mower_rsd
/// then starts no bridge.
pub fn rsd_plan<S: AsRef<str>>(ids: &[S], remaps: &Remaps) -> Result<Plan, PlanError> {
    rsd_plan_with(ids, &[] as &[&str], remaps)
}

/// [`rsd_plan`] with the run-time flags `on_now` set as well: the modules of
/// [`crate::RUNTIME`] that run on the bus in this process (ws_bridge with
/// `--backend bus`). Any other id there is an error: its flag is
/// on_bus.rs's.
pub fn rsd_plan_with<S: AsRef<str>, T: AsRef<str>>(ids: &[S], on_now: &[T], remaps: &Remaps) -> Result<Plan, PlanError> {
    let problems: Vec<String> = on_now
        .iter()
        .map(AsRef::as_ref)
        .filter(|id| !crate::RUNTIME.contains(id))
        .map(|id| format!("{id}: its flag is on_bus.rs's, not set at run time"))
        .collect();
    if !problems.is_empty() {
        return Err(PlanError { problems });
    }
    let plan = plan(&Running::rsd(ids, crate::on_bus_with(on_now)), remaps)?;
    plan.check_names(remaps, NODE, "")?;
    Ok(plan)
}

/// The bus bridge module's arguments in mower_rsd: the ids of `--modules`,
/// then `--on-bus <id>` for each run-time flag that is set ([`rsd_plan_with`]).
pub fn rsd_args<S: AsRef<str>, T: AsRef<str>>(ids: &[S], on_now: &[T]) -> Vec<String> {
    let mut args: Vec<String> = ids.iter().map(|id| id.as_ref().to_string()).collect();
    for id in on_now {
        args.push("--on-bus".to_string());
        args.push(id.as_ref().to_string());
    }
    args
}

/// (E5-4) The flag mower_rsd puts in front of the bag module's arguments:
/// `--rsd-process <ids> <run-time flags>`, comma-separated (`-` for none),
/// so that the recorder applies the source rule (design 3.5) to the same
/// set and flags as the bridge.
pub const PROCESS_FLAG: &str = "--rsd-process";

/// The scopes of the recorder's bus taps (E5-4): one per group, named as the
/// group's node (`mower_recorder_core::profile`, design D12). Readers the
/// policy does not plan for (a tapped name has no writer off the bus, so it
/// needs nothing from DDS), so mower_rsd's manifest check skips them as it
/// skips the bridge's own.
pub const RECORDER_TAPS: &[&str] = &["mower_full_recorder", "mower_camera_recorder", "mower_snapshot_recorder"];

/// [`PROCESS_FLAG`] and its two values, for `ids` (`--modules`) and the
/// run-time flags `on_now`.
pub fn process_args<S: AsRef<str>, T: AsRef<str>>(ids: &[S], on_now: &[T]) -> Vec<String> {
    let join = |v: Vec<&str>| if v.is_empty() { "-".to_string() } else { v.join(",") };
    vec![
        PROCESS_FLAG.to_string(),
        join(ids.iter().map(AsRef::as_ref).collect()),
        join(on_now.iter().map(AsRef::as_ref).collect()),
    ]
}

/// The instances of this process as module `id` (instance `node`) sees
/// them, and its arguments without [`PROCESS_FLAG`]'s: under mower_rsd the
/// set [`process_args`] gave (as [`rsd_plan_with`] plans it); standalone,
/// where nothing put the flag in, the module's own binary
/// ([`Running::standalone`]), whose bus has only the module.
pub fn running_here(args: &[String], id: &str, node: &str) -> Result<(Vec<Running>, Vec<String>), String> {
    let Some(i) = args.iter().position(|a| a == PROCESS_FLAG) else {
        return Ok((Running::standalone(id, node)?, args.to_vec()));
    };
    let (Some(ids), Some(on_now)) = (args.get(i + 1), args.get(i + 2)) else {
        return Err(format!("{PROCESS_FLAG} needs the module ids and the run-time flags"));
    };
    let split = |v: &str| -> Vec<String> { if v == "-" { Vec::new() } else { v.split(',').map(str::to_string).collect() } };
    let (ids, on_now) = (split(ids), split(on_now));
    let mut rest = args.to_vec();
    rest.drain(i..i + 3);
    Ok((Running::rsd(&ids, crate::on_bus_with(&on_now)), rest))
}

/// What [`rsd_args`] made: the ids, and the run-time flags.
pub(crate) fn parse_rsd_args(args: &[String]) -> Result<(Vec<String>, Vec<String>), String> {
    let (mut ids, mut on_now) = (Vec::new(), Vec::new());
    let mut it = args.iter();
    while let Some(arg) = it.next() {
        if arg == "--on-bus" {
            on_now.push(it.next().ok_or("--on-bus needs a module id")?.clone());
        } else {
            ids.push(arg.clone());
        }
    }
    Ok((ids, on_now))
}

/// mower_rsd's bridge module. `m.args` are what [`rsd_args`] made of
/// `--modules` and the run-time flags; it forwards what [`rsd_plan_with`]
/// derives for them, on node `m.node_name` ([`NODE`]) and scope `m.bus`,
/// until the shutdown.
pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let (ids, on_now) = parse_rsd_args(&m.args)?;
    let plan = rsd_plan_with(&ids, &on_now, &m.remaps)?;
    let bridge = new_bridge(ctx, &m.node_name, &m.namespace, &plan, &m.remaps, m.bus.clone())?;
    let _probe = shown_in_registry(&bridge, &m);
    bridge.run(m.shutdown.wait()).await?;
    Ok(())
}

/// `bridge`'s counters for `/mowerd/bridge`, read when asked, until the guard
/// goes (E8-2).
fn shown_in_registry(bridge: &RosBridge, m: &ModuleCtx) -> mower_rs_common::registry::BridgeProbeGuard {
    let stats = bridge.stats();
    m.registry.set_bridge_probe(std::sync::Arc::new(move || stats_json(&stats)))
}

/// The counters of every bridged name, as `/mowerd/bridge` answers them.
pub(crate) fn stats_json(stats: &BridgeStats) -> Value {
    let entry = |(name, e): (String, EntryStats)| {
        json!({
            "name": name,
            "direction": e.direction.to_string(),
            "forwarded": e.forwarded,
            "skipped": e.skipped,
            "stale": e.stale,
            "resyncs": e.resyncs,
            "unanswered": e.unanswered,
            "dropped": e.dropped,
            "invalid": e.invalid,
            "dds_writer": e.dds_writer,
            "dds_reader": e.dds_reader,
            "dds_server": e.dds_server,
            "bus_writer": e.bus_writer,
            "bus_server": e.bus_server,
        })
    };
    Value::Array(stats.entries().into_iter().map(entry).collect())
}

/// A bridge of node `node` in `namespace` and scope `scope`, with `plan`
/// declared, once its names are the module's on this node too.
fn new_bridge(
    ctx: r2r::Context, node: &str, namespace: &str, plan: &Plan, remaps: &Remaps, scope: Scope,
) -> Result<RosBridge, BoxError> {
    plan.check_names(remaps, node, namespace)?;
    let node_handle = r2r::Node::create(ctx, node, namespace)?;
    let logger = node_handle.logger().to_string();
    let named = node_handle.fully_qualified_name()?;
    let meant = format!("{}/{node}", namespace.trim_end_matches('/'));
    if named != meant {
        // `__node:=` and `__ns:=` rules rename every node of the process.
        r2r::log_warn!(
            &logger,
            "mower_bridge: the bridge's node is {named}, not {meant}: a `-r __node:=` or `__ns:=` rule renamed it, \
             and it may now share its name with another node of the process"
        );
    }
    let mut bridge = RosBridge::new(node_handle, scope);
    plan.declare(&mut bridge)?;
    let count = |direction| plan.declarations.iter().filter(|d| d.direction == direction).count();
    r2r::log_info!(
        &logger,
        "mower_bridge: {} to_ros, {} from_ros, {} service_to_ros, {} service_from_ros, {} action_to_ros, {} action_from_ros",
        count(Direction::ToRos),
        count(Direction::FromRos),
        count(Direction::ServiceToRos),
        count(Direction::ServiceFromRos),
        count(Direction::ActionToRos),
        count(Direction::ActionFromRos)
    );
    for d in &plan.declarations {
        r2r::log_debug!(&logger, "mower_bridge: {d} ({})", d.why);
    }
    for note in &plan.notes {
        r2r::log_info!(&logger, "mower_bridge: {note}");
    }
    Ok(bridge)
}

/// The body of a moved module's `main.rs`, in place of
/// [`mower_rs_common::module_main`]: the same, with [`run_standalone`]
/// around the module. `id` is the module's mower_rsd id; `node_name` and
/// `worker_threads` are what the binary passed before.
pub fn module_main<F, Fut>(id: &'static str, node_name: &str, worker_threads: usize, run: F) -> std::process::ExitCode
where
    F: FnOnce(r2r::Context, ModuleCtx) -> Fut,
    Fut: Future<Output = ModuleResult>,
{
    mower_rs_common::module_main(node_name, worker_threads, move |ctx, m| run_standalone(id, ctx, m, run))
}

/// The body of the `main.rs` of a module whose flag may still be false (the
/// drivetrain, E2 steps 6 and 7, merged with their flags off): with its
/// instance on the bus ([`crate::instance_on_bus`] of the node rcl names the
/// binary's, after launch's `-r __node:=`), [`module_main`]; otherwise
/// [`mower_rs_common::module_main`], the module alone on r2r with no bridge,
/// as before E2. The module's `run` reads the same flag to pick its IO.
pub fn module_main_by_flag<F, Fut>(
    id: &'static str, node_name: &str, worker_threads: usize, run: F,
) -> std::process::ExitCode
where
    F: FnOnce(r2r::Context, ModuleCtx) -> Fut,
    Fut: Future<Output = ModuleResult>,
{
    mower_rs_common::module_main(node_name, worker_threads, move |ctx, m| async move {
        if crate::instance_on_bus(&m.remaps.node_name(&m.node_name)) {
            run_standalone(id, ctx, m, run).await
        } else {
            run(ctx, m).await
        }
    })
}

/// Module `id` as its own process. Its instances (the one rcl names after
/// `m.node_name`, or all of them for a binary named after the module) run on
/// a bus of their own, and a bridge, node `<node>` in [`STANDALONE_NAMESPACE`],
/// carries every endpoint they have in the manifest: [`crate::plan`] with
/// every other module elsewhere. The manifest check reports on stderr.
///
/// A bridge that fails stops the module, and the process then exits
/// non-zero, so that launch's `respawn` restarts both, as it restarts a
/// module that fails.
pub async fn run_standalone<F, Fut>(id: &str, ctx: r2r::Context, m: ModuleCtx, run: F) -> ModuleResult
where
    F: FnOnce(r2r::Context, ModuleCtx) -> Fut,
    Fut: Future<Output = ModuleResult>,
{
    let node = m.remaps.node_name(&m.node_name);
    let running = Running::standalone(id, &node)?;
    let plan = plan(&running, &m.remaps)?;
    let bus = Bus::new();
    let module_scope = bus.scope(&node);
    let bridge = new_bridge(ctx.clone(), &node, STANDALONE_NAMESPACE, &plan, &m.remaps, bus.scope(NODE))?;
    let _probe = shown_in_registry(&bridge, &m);
    let (stop, stopped) = futures::channel::oneshot::channel::<()>();
    let mut bridging = tokio::spawn(bridge.run(async move {
        let _ = stopped.await;
    }));
    let watcher = {
        let (bus, remaps) = (bus.clone(), m.remaps.clone());
        let report = move |gap: &str| eprintln!("[{node}] mower_bridge: {gap}");
        tokio::spawn(crate::coverage::watch(bus, running, remaps, vec![NODE.to_string()], report))
    };

    let shutdown = m.shutdown.clone();
    let mut module = std::pin::pin!(run(ctx, m.with_bus(module_scope.clone())));
    let (result, early) = tokio::select! {
        result = &mut module => (result, None),
        // It runs until it is told to stop: ending first is a failure.
        ended = &mut bridging => {
            shutdown.trigger();
            (module.await, Some(ended))
        }
    };
    // As mower_rsd's supervisor does: what the module's tasks still run ends
    // with its scope.
    module_scope.close();
    let _ = stop.send(());
    let stopped_early = early.is_some();
    let bridged = match early {
        Some(ended) => ended,
        None => bridging.await,
    };
    watcher.abort();
    result?;
    match bridged {
        Ok(Ok(())) if !stopped_early => Ok(()),
        Ok(Ok(())) => Err("the bus bridge stopped before the module".into()),
        Ok(Err(e)) => Err(format!("the bus bridge: {e}").into()),
        Err(e) => Err(format!("the bus bridge panicked: {e}").into()),
    }
}
