//! mower_recorder's three rclpy nodes (`src/mower_recorder/mower_recorder/
//! {recorder_manager,bag_store,graph_snapshot}_node.py`) as one module of
//! `mower_rsd`: module `bag`, launch switch `rust_recorder`, compose
//! `RUST_RECORDER` (step E5 of docs/ROS_FREE_PLAN.md; design D13: `record` is
//! the zone path recorder's id, `RUST_RECORD` its switch).
//!
//! **E5-3b: it records** the default profile from DDS. `RUST_RECORDER`
//! stays off (it is by default) until the owner's robot checks; the
//! data_collection profile comes with E5-5.
//!
//! - [`graph`]: `graph_snapshot`, complete: every 2 s the DDS graph as
//!   `/graph_snapshot` JSON, the node diff on `/graph_events`, the watch rule
//!   (which never fires, design 1.12 #1).
//! - [`store`]: `bag_store`, complete: `/mower_recorder/bags` every 5 s and
//!   after each command, `/mower_recorder/command` → `command_result`, the R2
//!   upload every 30 s on a thread of its own, `upload_now`, delete.
//! - [`manager`]: `recorder_manager` around `mower_recorder_core::manager`:
//!   start, stop, snapshot, the 2 s status and rear light with the disk guard,
//!   MediaMTX, `run_metadata.yaml`, the parameter dump, the fault.
//! - [`sources`]: a run's DDS subscriptions, a thread and a node per group
//!   (`mower_full_recorder`, ...; design D12), found as rosbag2 finds them.
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

pub mod camera;
pub mod graph;
pub mod manager;
pub mod owner;
pub mod params;
pub mod sources;
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

/// (E5-6) The camera instance, gscam's node name; on the graph only while
/// data collection asks for it (`camera.rs`).
pub const NODE_CAMERA: &str = camera::NODE;

/// Every instance of the module: the three, then the camera.
pub const INSTANCES: [&str; 4] = [NODE_GRAPH, NODE_MANAGER, NODE_STORE, NODE_CAMERA];

/// One of the three nodes, chosen by `m.node_name` (mower_rsd's instances).
pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    match m.node_name.as_str() {
        NODE_MANAGER => manager::run(ctx, m).await,
        NODE_STORE => store::run(ctx, m).await,
        NODE_GRAPH => graph::run(ctx, m).await,
        NODE_CAMERA => camera::run(ctx, m).await,
        other => Err(format!("mower_recorder: no such node {other:?}; expected one of {}", INSTANCES.join(", ")).into()),
    }
}

/// All three nodes in this process: the `mower_recorder` binary
/// (`rust_daemon:=false`, mission.launch.py). As mower_localize's `run_all`:
/// the first role to fail, panic or return early trips the shared shutdown,
/// and the process exits non-zero for launch's `respawn`. Each role runs on a
/// child scope of `m.bus`, closed as soon as it ends.
pub async fn run_all(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let mut tasks = Vec::with_capacity(INSTANCES.len());
    for name in INSTANCES {
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
/// (E5-5) data_collection's request file (`mower_recorder_core::dc_request`):
/// the node's `profile_request_file`, else `$MOWER_STATE_DIR/recorder_profile.json`,
/// else `~/.mower/recorder_profile.json`.
pub fn request_file(node: &r2r::Node) -> std::path::PathBuf {
    let name = mower_recorder_core::dc_request::FILE_NAME;
    match mower_rs_common::params::string(node, "profile_request_file", "") {
        p if !p.is_empty() => std::path::PathBuf::from(expanduser(&p)),
        _ => match std::env::var("MOWER_STATE_DIR") {
            Ok(dir) if !dir.is_empty() => std::path::PathBuf::from(dir).join(name),
            _ => std::path::PathBuf::from(expanduser("~/.mower")).join(name),
        },
    }
}

/// How long `/robot_description` is waited for (robot_state_publisher's,
/// latched).
pub const URDF_WAIT: std::time::Duration = std::time::Duration::from_secs(3);

/// robot_state_publisher's `/robot_description` (latched), read on `node`,
/// spun here while it waits: the URDF the data_collection launch got from
/// xacro (E5-5's recorder_manager, E5-6's camera).
pub fn robot_description(node: &mut r2r::Node) -> Result<String, String> {
    use futures::StreamExt;
    let qos = r2r::QosProfile::default().keep_last(1).reliable().transient_local();
    let mut sub = node
        .subscribe::<r2r::std_msgs::msg::String>("/robot_description", qos)
        .map_err(|e| e.to_string())?;
    let deadline = std::time::Instant::now() + URDF_WAIT;
    let mut got = None;
    while got.is_none() && std::time::Instant::now() < deadline {
        node.spin_once(std::time::Duration::from_millis(20));
        while let Some(Some(msg)) = sub.next().now_or_never() {
            got = Some(msg.data);
        }
    }
    node.destroy_subscription(sub);
    got.ok_or_else(|| format!("no /robot_description within {} s", URDF_WAIT.as_secs()))
}

/// mower_recorder's `config/` in the installed workspace
/// (`AMENT_PREFIX_PATH`'s `share/mower_recorder/config`), for an instance
/// without a `config_file` parameter (the camera).
pub fn share_config() -> Option<std::path::PathBuf> {
    let prefixes = std::env::var("AMENT_PREFIX_PATH").ok()?;
    prefixes
        .split(':')
        .map(|p| std::path::Path::new(p).join("share/mower_recorder/config"))
        .find(|d| d.join("data_collection.yaml").is_file())
}

pub fn expanduser(path: &str) -> String {
    if path == "~" || path.starts_with("~/") {
        if let Ok(home) = std::env::var("HOME") {
            return format!("{home}{}", &path[1..]);
        }
    }
    path.to_string()
}
