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
//! **On the bus (step E2).** Each node's topics, navsat's four services and
//! the timers are on the in-process bus (`ModuleCtx::bus`), under the names
//! rcl gives them on the node, `-r` rules included
//! ([`ModuleCtx::resolve_topic`]; docs/BUS_TOPICS.md 5.4). The r2r nodes stay
//! for the parameters and their services, the log and the graph; navsat's
//! also publishes the `utm` frame on `/tf_static` over DDS until E4.
//! `/odom` and robot_state_publisher's `/tf_static` reach the bus from DDS
//! through the bridge, and `/tf` and what nav2, the recorder or the tools
//! read go out through it.
//!
//! Which node a task runs is decided by its node name, which is how
//! `mower_rsd`'s module table addresses an instance; [`run_all`] is the same
//! three roles for the standalone binary.
//!
//! **One process more goes away.** `ekf_filter_node_map` also publishes the
//! 5 Hz `/odometry/global_slow` itself (`mower_rs_common::throttle`), which
//! `mission.launch.py`'s `global_odom_throttle` (`topic_tools throttle`)
//! otherwise makes from `/odometry/global`; with `rust_localize:=true` that
//! launch file does not start the throttle.

mod conv;
mod ekf;
mod navsat;
mod paramsrv;
pub mod tfbus;
pub mod tfgraph;

use std::panic::AssertUnwindSafe;

use futures::FutureExt;
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
///
/// Each role runs on a child scope of `m.bus`, closed as soon as the role
/// ends; closing `m.bus` closes all three.
pub async fn run_all(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let mut tasks = Vec::with_capacity(NODES.len());
    for name in NODES {
        let ctx = ctx.clone();
        let mc = role(&m, name);
        let scope = mc.bus.clone();
        tasks.push(tokio::spawn(async move {
            let ended = AssertUnwindSafe(run(ctx, mc)).catch_unwind().await;
            // However the role ended, its scope closes now: what its tasks
            // still run must not go on using the bus next to the other two.
            scope.close();
            match ended {
                Ok(result) => (name, result),
                Err(panic) => std::panic::resume_unwind(panic),
            }
        }));
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

/// [`run_all`]'s context for the role `name`. Three nodes, so three scopes,
/// and one clock. The scopes are children of this module's own, so whoever
/// closes that one ends all three roles' endpoints with it. The process's
/// `-r` rules go with them: rcl applies them to each role's node by its name,
/// and the bus names must come out the same.
fn role(m: &ModuleCtx, name: &str) -> ModuleCtx {
    ModuleCtx::new(name, m.shutdown.clone())
        .with_bus(m.bus.child(name))
        .with_clock(m.clock.clone())
        .with_remaps(m.remaps.clone())
}

#[cfg(test)]
pub(crate) mod testing {
    //! What the tests of every role share.

    use std::sync::{Arc, OnceLock};

    use mower_rs_common::{ModuleCtx, Remaps};

    /// The process's r2r context, on ROS domain 86: the roles still create
    /// their r2r nodes (parameters, parameter services, log), and other
    /// containers on a dev host speak DDS. Every test goes through this, so
    /// that the domain is set before the process's one context exists.
    pub fn context() -> Option<r2r::Context> {
        static CONTEXT: OnceLock<Option<r2r::Context>> = OnceLock::new();
        CONTEXT
            .get_or_init(|| {
                // rcl reads it when the context is created, and only then.
                std::env::set_var("ROS_DOMAIN_ID", "86");
                r2r::Context::create().ok()
            })
            .clone()
    }

    /// The `-r` rules robot.launch.py passes mower_rsd for this module
    /// (`_DAEMON_REMAPS['localize']` and navsat's `gps/fix` rule with the
    /// production `gps_fix_topic`), and an imu rule that must not apply to
    /// these nodes, as the production argv has it.
    pub fn production_remaps() -> Arc<Remaps> {
        let argv: Vec<String> = "--modules imu,gps,localize --ros-args --params-file /x/mower_rsd.yaml \
             -r imu:imu/data_raw:=imu/data \
             -r ekf_filter_node_odom:odometry/filtered:=odometry/local \
             -r ekf_filter_node_odom:imu:=imu/data \
             -r ekf_filter_node_map:odometry/filtered:=odometry/global \
             -r ekf_filter_node_map:imu:=imu/data \
             -r navsat_transform:imu:=imu/data \
             -r navsat_transform:odometry/filtered:=odometry/global \
             -r navsat_transform:gps/fix:=/fix"
            .split_whitespace()
            .map(str::to_string)
            .collect();
        Arc::new(Remaps::from_args(&argv).unwrap())
    }

    /// A node named like the role, in a namespace of the test's own so that
    /// nothing on the graph meets it, with `params` as its overrides (what
    /// the params file gives it in production).
    pub fn node(ctx: &r2r::Context, name: &str, ns: &str, params: &[(&str, r2r::ParameterValue)]) -> r2r::Node {
        let node = r2r::Node::create(ctx.clone(), name, ns).unwrap();
        for (key, value) in params {
            node.params.lock().unwrap().insert(key.to_string(), r2r::Parameter::new(value.clone()));
        }
        node
    }

    /// A role's context on `scope` and `clock`, with the production rules.
    pub fn module(name: &str, scope: &mower_bus::Scope, clock: &mower_bus::Clock) -> ModuleCtx {
        ModuleCtx::new(name, mower_rs_common::Shutdown::new())
            .with_bus(scope.clone())
            .with_clock(clock.clone())
            .with_remaps(production_remaps())
    }

    /// Run one test on a runtime of its own, within a bound that only a hang
    /// reaches. A role's node spins on a blocking task until the role stops,
    /// and dropping a runtime waits for its blocking tasks: so a test that
    /// fails before it has stopped its roles lets the runtime go without
    /// waiting, and its failure is reported instead of hanging.
    pub fn run(test: impl std::future::Future<Output = ()>) {
        let runtime = tokio::runtime::Builder::new_multi_thread().worker_threads(4).enable_all().build().unwrap();
        let ended = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            runtime.block_on(async {
                tokio::time::timeout(std::time::Duration::from_secs(120), test).await.expect("the test hung")
            })
        }));
        runtime.shutdown_background();
        if let Err(panic) = ended {
            std::panic::resume_unwind(panic);
        }
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
        let ctx = match testing::context() {
            Some(c) => c,
            // No ROS middleware in this environment: the dispatch is still the
            // thing under test, and it never gets as far as the context.
            None => return,
        };
        let m = ModuleCtx::new("ekf_filter_node_typo", mower_rs_common::Shutdown::new());
        let err = run(ctx, m).await.expect_err("must not silently start a node");
        assert!(err.to_string().contains("ekf_filter_node_typo"), "{err}");
    }

    /// The standalone binary's three roles resolve their names as mower_rsd's
    /// three instances do: under the process's rules, each by its own node
    /// name, on a child of the module's scope.
    #[test]
    fn each_role_of_run_all_resolves_names_under_the_process_rules() {
        let m = ModuleCtx::new("localize", mower_rs_common::Shutdown::new())
            .with_remaps(testing::production_remaps());
        let odom = role(&m, NODE_EKF_ODOM);
        let map = role(&m, NODE_EKF_MAP);
        let navsat = role(&m, NODE_NAVSAT);
        assert_eq!(odom.resolve_topic("odometry/filtered").unwrap(), "/odometry/local");
        assert_eq!(map.resolve_topic("odometry/filtered").unwrap(), "/odometry/global");
        assert_eq!(navsat.resolve_topic("gps/fix").unwrap(), "/fix");
        assert_eq!(navsat.resolve_topic("imu").unwrap(), "/imu/data");
        assert_eq!((odom.bus.name(), navsat.node_name.as_str()), (NODE_EKF_ODOM, NODE_NAVSAT));
        m.bus.close();
        assert!(odom.bus.is_closed() && map.bus.is_closed() && navsat.bus.is_closed());
    }

    /// The three nodes on one bus, as mower_rsd runs them: the production
    /// parameters (dual_ekf_navsat_params.yaml's topic keys, mower_rsd.yaml's
    /// slow copy) and `-r` rules make them meet on exactly the names, types
    /// and ends of docs/BUS_TOPICS.md 2.3 and 5.4, and serve navsat's four
    /// services (3.1). What the bridge forwards is decided from these.
    #[test]
    fn the_three_nodes_meet_on_the_names_bus_topics_lists() {
        testing::run(async {
            use r2r::ParameterValue as P;
            let Some(ctx) = testing::context() else { return };
            let s = |v: &str| P::String(v.to_string());
            // A dual_ekf_navsat_params.yaml `*_config`: true at these indices.
            let config = |on: &[usize]| P::BoolArray((0..15).map(|i| on.contains(&i)).collect());
            let ekf = |world: &str| vec![
                ("frequency", P::Double(20.0)),
                ("two_d_mode", P::Bool(true)),
                ("publish_tf", P::Bool(true)),
                ("map_frame", s("map")),
                ("odom_frame", s("odom")),
                ("base_link_frame", s("base_footprint")),
                ("world_frame", s(world)),
                ("odom0", s("odom")),
                ("odom0_config", config(&[6, 7, 8, 11])),
                ("imu0", s("imu")),
                ("imu0_config", config(&[5])),
            ];
            let odom = ekf("odom");
            let mut map = ekf("map");
            map.extend([
                ("odom1", s("odometry/gps")),
                ("odom1_config", config(&[0, 1])),
                ("odometry_slow_topic", s("/odometry/global_slow")),
                ("odometry_slow_rate_hz", P::Double(5.0)),
            ]);
            let navsat = vec![
                ("frequency", P::Double(30.0)),
                ("delay", P::Double(3.0)),
                ("zero_altitude", P::Bool(true)),
                ("broadcast_cartesian_transform", P::Bool(true)),
                ("publish_filtered_gps", P::Bool(true)),
                ("use_odometry_yaw", P::Bool(false)),
                ("wait_for_datum", P::Bool(false)),
            ];

            let bus = mower_bus::Bus::new();
            // Never moved: no tick after the first, navsat inside its delay.
            let (clock, _manual) = mower_bus::Clock::manual(mower_bus::Time::from_ns(1_790_000_000_000_000_000));
            let mut tasks = Vec::new();
            let mut shutdowns = Vec::new();
            for (name, params) in [(NODE_EKF_ODOM, odom), (NODE_EKF_MAP, map), (NODE_NAVSAT, navsat)] {
                let m = testing::module(name, &bus.scope(name), &clock);
                shutdowns.push(m.shutdown.clone());
                let node = testing::node(&ctx, name, "/localize_names_test", &params);
                tasks.push(tokio::spawn(match name {
                    NODE_NAVSAT => navsat::run_node(node, m).boxed(),
                    _ => ekf::run_node(node, m).boxed(),
                }));
            }

            let (o, g, n) = (NODE_EKF_ODOM, NODE_EKF_MAP, NODE_NAVSAT);
            let expected: Vec<(&str, &str, Vec<&str>, Vec<&str>)> = vec![
                ("/fix", "sensor_msgs/msg/NavSatFix", vec![], vec![n]),
                ("/gps/filtered", "sensor_msgs/msg/NavSatFix", vec![n], vec![]),
                ("/imu/data", "sensor_msgs/msg/Imu", vec![], vec![g, o, n]),
                ("/odom", "nav_msgs/msg/Odometry", vec![], vec![g, o]),
                ("/odometry/global", "nav_msgs/msg/Odometry", vec![g], vec![n]),
                ("/odometry/global_slow", "nav_msgs/msg/Odometry", vec![g], vec![]),
                ("/odometry/gps", "nav_msgs/msg/Odometry", vec![n], vec![g]),
                ("/odometry/local", "nav_msgs/msg/Odometry", vec![o], vec![]),
                ("/tf", "tf2_msgs/msg/TFMessage", vec![g, o], vec![]),
                ("/tf_static", "tf2_msgs/msg/TFMessage", vec![], vec![g, o, n]),
            ];
            let expected_services = vec![
                ("/datum".to_string(), "robot_localization/srv/SetDatum".to_string()),
                ("/fromLL".to_string(), "robot_localization/srv/FromLL".to_string()),
                ("/fromLLArray".to_string(), "robot_localization/srv/FromLLArray".to_string()),
                ("/toLL".to_string(), "robot_localization/srv/ToLL".to_string()),
            ];
            let snapshot = || {
                let topics: Vec<(String, String, Vec<String>, Vec<String>)> = bus
                    .topics()
                    .into_iter()
                    .map(|mut t| {
                        t.publishers.sort();
                        t.subscribers.sort();
                        (t.name, t.ros_type.unwrap_or_default(), t.publishers, t.subscribers)
                    })
                    .collect();
                let services: Vec<(String, String)> = bus
                    .services()
                    .into_iter()
                    .filter(|s| s.server.as_deref() == Some(NODE_NAVSAT))
                    .map(|s| (s.name, s.ros_type.unwrap_or_default()))
                    .collect();
                (topics, services)
            };
            let expected_topics: Vec<(String, String, Vec<String>, Vec<String>)> = expected
                .into_iter()
                .map(|(name, ty, mut publishers, mut subscribers)| {
                    publishers.sort();
                    subscribers.sort();
                    let own = |v: Vec<&str>| v.into_iter().map(str::to_string).collect();
                    (name.to_string(), ty.to_string(), own(publishers), own(subscribers))
                })
                .collect();
            // Every node makes its endpoints before anything else; wait for all.
            let deadline = tokio::time::Instant::now() + std::time::Duration::from_secs(20);
            while snapshot() != (expected_topics.clone(), expected_services.clone())
                && tokio::time::Instant::now() < deadline
            {
                tokio::time::sleep(std::time::Duration::from_millis(10)).await;
            }
            let (topics, services) = snapshot();
            assert_eq!(topics, expected_topics);
            assert_eq!(services, expected_services);

            for shutdown in shutdowns {
                shutdown.trigger();
            }
            for task in tasks {
                tokio::time::timeout(std::time::Duration::from_secs(20), task)
                    .await
                    .expect("a node did not stop")
                    .expect("node task")
                    .expect("node result");
            }
        });
    }
}
