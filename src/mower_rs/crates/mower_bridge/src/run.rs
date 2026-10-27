//! The two ways a bridge runs: as mower_rsd's bridge module, for the
//! instances of `--modules`, and next to a module in its own binary, where
//! every endpoint of the module faces DDS (README.md, "Running").

use std::future::Future;

use mower_bus::{Bus, Scope};
use mower_bus_ros::{Direction, RosBridge};
use mower_rs_common::{BoxError, ModuleCtx, ModuleResult, Remaps};

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
    let plan = plan(&Running::rsd(ids, crate::on_bus), remaps)?;
    plan.check_names(remaps, NODE, "")?;
    Ok(plan)
}

/// mower_rsd's bridge module. `m.args` are the module ids of `--modules`;
/// it forwards what [`rsd_plan`] derives for them, on node `m.node_name`
/// ([`NODE`]) and scope `m.bus`, until the shutdown.
pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let plan = rsd_plan(&m.args, &m.remaps)?;
    let bridge = new_bridge(ctx, &m.node_name, &m.namespace, &plan, &m.remaps, m.bus.clone())?;
    bridge.run(m.shutdown.wait()).await?;
    Ok(())
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
