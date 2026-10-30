//! mower_recorder's three rclpy nodes (`src/mower_recorder/mower_recorder/
//! {recorder_manager,bag_store,graph_snapshot}_node.py`) as one module of
//! `mower_rsd`: module `bag`, launch switch `rust_recorder`, compose
//! `RUST_RECORDER` (step E5 of docs/ROS_FREE_PLAN.md; design D13: `record` is
//! the zone path recorder's id, `RUST_RECORD` its switch).
//!
//! **E5-3a: idle only.** Everything the three nodes do while nothing records
//! is here; recording itself comes with E5-3b. Until then `RUST_RECORDER`
//! stays off (it is by default): with it on, `/mower_recorder/start` answers
//! `success: false` with [`manager::NOT_YET`].
//!
//! - [`graph`]: `graph_snapshot`, complete: every 2 s the DDS graph as
//!   `/graph_snapshot` JSON, the node diff on `/graph_events`, the watch rule
//!   (which never fires, design 1.12 #1).
//! - [`store`]: `bag_store`, complete: `/mower_recorder/bags` every 5 s and
//!   after each command, `/mower_recorder/command` → `command_result`, the R2
//!   upload every 30 s on a thread of its own, `upload_now`, delete.
//! - [`manager`]: `recorder_manager`, idle: `/mower_recorder/status` and the
//!   rear light every 2 s, MediaMTX's `record: false` at the start and its
//!   retry, `/fix` and the fault, `start` / `stop` / `snapshot` answered as the
//!   Python answers them while nothing records.
//! - [`owner`]: one owner of `/mower_recorder/*` (design 4.2, R6): while a
//!   DDS server of `/mower_recorder/start` that is not a bus bridge's exists
//!   (the Python recorder), none of the three serves its endpoints, and each
//!   says so in an ERROR.
//!
//! Everything a node writes or answers is `mower_recorder_core`'s, byte for
//! byte the Python's; this crate wires it to the bus, the r2r nodes and the
//! clock.
//!
//! **On the bus**, born there: every topic and service of the three is a bus
//! endpoint under the name rcl gives it on its node ([`ModuleCtx::resolve_topic`]),
//! and reaches DDS through mower_bridge (manifest module `bag`). Each instance
//! keeps an r2r node of its name for its parameters, log and graph presence
//! (crates/mower_bridge/README.md, decision 1); graph_snapshot's also reads
//! the DDS graph, and every one looks for a foreign recorder on it.
//!
//! [`ModuleCtx::resolve_topic`]: mower_rs_common::ModuleCtx::resolve_topic

pub mod graph;
pub mod manager;
pub mod owner;
pub mod params;
pub mod store;

use std::panic::AssertUnwindSafe;

use futures::FutureExt;
use mower_rs_common::{ModuleCtx, ModuleResult};

pub const NODE_MANAGER: &str = "recorder_manager";
pub const NODE_STORE: &str = "bag_store";
pub const NODE_GRAPH: &str = "graph_snapshot";

/// The three node names, in the order record.launch.py starts them, which is
/// mower_rsd's module table's too.
pub const NODES: [&str; 3] = [NODE_GRAPH, NODE_MANAGER, NODE_STORE];

/// One of the three nodes, chosen by `m.node_name` (mower_rsd's instances).
pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    match m.node_name.as_str() {
        NODE_MANAGER => manager::run(ctx, m).await,
        NODE_STORE => store::run(ctx, m).await,
        NODE_GRAPH => graph::run(ctx, m).await,
        other => Err(format!("mower_recorder: no such node {other:?}; expected one of {}", NODES.join(", ")).into()),
    }
}

/// All three nodes in this process: the `mower_recorder` binary
/// (`rust_daemon:=false`, mission.launch.py). As mower_localize's `run_all`:
/// the first role to fail, panic or return early trips the shared shutdown,
/// and the process exits non-zero for launch's `respawn`. Each role runs on a
/// child scope of `m.bus`, closed as soon as it ends.
pub async fn run_all(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let mut tasks = Vec::with_capacity(NODES.len());
    for name in NODES {
        let ctx = ctx.clone();
        let mc = ModuleCtx::new(name, m.shutdown.clone())
            .with_bus(m.bus.child(name))
            .with_clock(m.clock.clone())
            .with_remaps(m.remaps.clone());
        let scope = mc.bus.clone();
        tasks.push(tokio::spawn(async move {
            let ended = AssertUnwindSafe(run(ctx, mc)).catch_unwind().await;
            scope.close();
            match ended {
                Ok(result) => (name, result),
                Err(panic) => std::panic::resume_unwind(panic),
            }
        }));
    }
    let mut failure: Option<String> = None;
    let mut note = |why: String, shutdown: &mower_rs_common::Shutdown| {
        eprintln!("[mower_recorder] {why}");
        if failure.is_none() {
            failure = Some(why);
        }
        shutdown.trigger();
    };
    for task in tasks {
        match task.await {
            Ok((name, Err(e))) => note(format!("{name}: {e}"), &m.shutdown),
            Ok((name, Ok(()))) if !m.shutdown.is_triggered() => note(format!("{name}: returned before shutdown"), &m.shutdown),
            Ok(_) => {}
            Err(e) => note(format!("recorder task did not join: {e}"), &m.shutdown),
        }
    }
    match failure {
        Some(why) => Err(why.into()),
        None => Ok(()),
    }
}

/// `os.path.expanduser`: a leading `~` or `~/` is `$HOME`.
pub fn expanduser(path: &str) -> String {
    if path == "~" || path.starts_with("~/") {
        if let Ok(home) = std::env::var("HOME") {
            return format!("{home}{}", &path[1..]);
        }
    }
    path.to_string()
}
