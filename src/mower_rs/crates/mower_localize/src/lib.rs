//! `mower_localize`: the two `ekf_node`s and `navsat_transform_node` of
//! `mower_nav2/launch/dual_ekf_navsat.launch.py`, as one mower_rs module.
//!
//! Phase C of `docs/ROS_FREE_PLAN.md`. On the LubanCat the three C++ processes
//! cost 23 % of a core (2 x EKF 13 %, navsat 6 % before PR #13's 30 -> 20 Hz),
//! almost none of it arithmetic: a 15-state EKF at 20 Hz is about 1 %, the
//! rest is DDS and executor overhead paid per message, per process.
//!
//! Every computation is `mower_localize_core`, which is a line-by-line port of
//! robot_localization 3.8.3 checked against the installed `librl_lib.so` on
//! generated vectors (worst case 1.3e-15 on the filter state, 1.6e-9 m on the
//! navsat output). This crate is only the ROS shell: subscriptions, timers,
//! publishers, transforms and services.
//!
//! **Three nodes, one module.** The node names stay
//! `ekf_filter_node_odom`, `ekf_filter_node_map` and `navsat_transform`, so
//! `ros2 node list`, the logs and the per-node parameter services look exactly
//! as they did. They are separate `r2r::Node`s on the shared context (one DDS
//! participant either way) and separate supervised tasks under `mower_rsd`, so
//! one of them failing restarts only itself — the same blast radius as the
//! three launch `Node`s. What they do share, in process, is the transform
//! cache: see [`tfbus`].
//!
//! Which node a task runs is decided by its node name, which is how
//! `mower_rsd`'s module table addresses an instance; [`run_all`] is the same
//! three roles for the standalone binary.

mod conv;
mod ekf;
mod navsat;
mod paramsrv;
mod tfbus;
mod tfgraph;

use mower_rs_common::{ModuleCtx, ModuleResult};

pub const NODE_EKF_ODOM: &str = "ekf_filter_node_odom";
pub const NODE_EKF_MAP: &str = "ekf_filter_node_map";
pub const NODE_NAVSAT: &str = "navsat_transform";

/// The three node names, in the order `dual_ekf_navsat.launch.py` starts them.
pub const NODES: [&str; 3] = [NODE_EKF_ODOM, NODE_EKF_MAP, NODE_NAVSAT];

/// One of the three localization nodes, chosen by `m.node_name`.
pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    match m.node_name.as_str() {
        // Which instance is which is the params file's business: the node
        // name picks the section, and `world_frame` in it the behaviour.
        NODE_EKF_ODOM | NODE_EKF_MAP => ekf::run(ctx, m).await,
        NODE_NAVSAT => navsat::run(ctx, m).await,
        other => Err(format!(
            "mower_localize: no such node {other:?}; expected one of {}",
            NODES.join(", ")
        )
        .into()),
    }
}

/// All three nodes in this process (what the `mower_localize` binary is).
///
/// Failing closed: the first role to fail, panic or return early trips the
/// shared shutdown so the other two unwind, and the process exits non-zero for
/// the launch `respawn` / `mower_rsd` supervisor to restart the set. Under
/// `mower_rsd` this function is not used at all — there the three roles are
/// three supervised instances that restart one at a time.
pub async fn run_all(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let mut tasks = Vec::with_capacity(NODES.len());
    for name in NODES {
        let ctx = ctx.clone();
        let mc = ModuleCtx::new(name, m.shutdown.clone());
        tasks.push(tokio::spawn(async move { (name, run(ctx, mc).await) }));
    }
    let mut failure: Option<String> = None;
    let mut note = |why: String, shutdown: &mower_rs_common::Shutdown| {
        eprintln!("[mower_localize] {why}");
        if failure.is_none() {
            failure = Some(why);
        }
        shutdown.trigger();
    };
    for task in tasks {
        match task.await {
            Ok((name, Err(e))) => note(format!("{name}: {e}"), &m.shutdown),
            Ok((name, Ok(()))) if !m.shutdown.is_triggered() => {
                note(format!("{name}: returned before shutdown"), &m.shutdown)
            }
            Ok(_) => {}
            Err(e) => note(format!("localize task did not join: {e}"), &m.shutdown),
        }
    }
    match failure {
        Some(why) => Err(why.into()),
        None => Ok(()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_node_names_are_the_ones_the_launch_file_gives_the_cpp_nodes() {
        // dual_ekf_navsat.launch.py: name='ekf_filter_node_odom', etc. They
        // are the contract for the parameter services and every log line.
        assert_eq!(NODES, ["ekf_filter_node_odom", "ekf_filter_node_map", "navsat_transform"]);
    }

    #[tokio::test]
    async fn an_unknown_node_name_fails_instead_of_running_the_wrong_filter() {
        let ctx = match r2r::Context::create() {
            Ok(c) => c,
            // No ROS middleware in this environment: the dispatch is still the
            // thing under test, and it never gets as far as the context.
            Err(_) => return,
        };
        let m = ModuleCtx::new("ekf_filter_node_typo", mower_rs_common::Shutdown::new());
        let err = run(ctx, m).await.expect_err("must not silently start a node");
        assert!(err.to_string().contains("ekf_filter_node_typo"), "{err}");
    }
}
