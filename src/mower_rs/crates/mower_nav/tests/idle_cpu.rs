//! nav_action_server at idle with `nav_backend: nav_core` does not spin.
//!
//! On the robot (2026-11-02) the thread that spun nav's r2r node took a whole
//! core: under nav_core the node holds no client, and r2r's spin_once returns
//! at once on an empty wait set. A test binary of its own, so the process's
//! CPU time is nav's alone (`cargo test` runs a binary's tests on threads of
//! one process).

use std::path::PathBuf;
use std::sync::OnceLock;
use std::time::{Duration, Instant};

use mower_bus::{Bus, Clock};
use mower_rs_common::{ModuleCtx, Shutdown};
use r2r::mower_interface::action::Waypoint;
use r2r::{Parameter, ParameterValue};

/// For anything that waits in real time. A pass never waits this long.
const WAIT: Duration = Duration::from_secs(20);
/// How long the idle CPU is measured, after nav is up.
const WINDOW: Duration = Duration::from_secs(3);

type WaypointAction = (Waypoint::Goal, Waypoint::Result, Waypoint::Feedback);

fn context() -> r2r::Context {
    static CONTEXT: OnceLock<r2r::Context> = OnceLock::new();
    CONTEXT
        .get_or_init(|| {
            // rcl reads it when the context is created, and only then; not
            // tests/bus.rs's 82, in case both binaries run at once.
            std::env::set_var("ROS_DOMAIN_ID", "83");
            r2r::Context::create().expect("an rcl context (source /opt/ros/jazzy/setup.bash)")
        })
        .clone()
}

fn dir(test: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("mower_nav_idle_{}_{test}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// The CPU time of the whole process, user and system: utime and stime of
/// /proc/self/stat, in USER_HZ ticks (100 a second on Linux).
fn process_cpu() -> Duration {
    let stat = std::fs::read_to_string("/proc/self/stat").unwrap();
    // the command is in parentheses and may hold spaces: count after it
    let fields: Vec<&str> = stat[stat.rfind(')').unwrap() + 2..].split_whitespace().collect();
    let ticks: u64 = fields[11].parse::<u64>().unwrap() + fields[12].parse::<u64>().unwrap();
    Duration::from_millis(ticks * 10)
}

#[test]
fn nav_core_at_idle_stays_well_under_one_core() {
    let runtime = tokio::runtime::Builder::new_multi_thread().worker_threads(4).enable_all().build().unwrap();
    runtime.block_on(async {
        let dir = dir("nav_core");
        let bus = Bus::new();
        let node = r2r::Node::create(context(), "nav_action_server", "").unwrap();
        let checkpoint = dir.join("coverage_progress.json").display().to_string();
        for (key, value) in [
            ("nav_backend", ParameterValue::String("nav_core".into())),
            ("require_navigation_health", ParameterValue::Bool(false)),
            ("progress_checkpoint_path", ParameterValue::String(checkpoint)),
        ] {
            node.params.lock().unwrap().insert(key.to_string(), Parameter::new(value));
        }
        let shutdown = Shutdown::new();
        let m = ModuleCtx::new("nav_action_server", shutdown.clone())
            .with_bus(bus.scope("nav_action_server"))
            .with_clock(Clock::system());
        let task = tokio::spawn(mower_nav::run_node(node, m));
        // its action servers come last: up once they are
        let action = bus.scope("test").action_client::<WaypointAction>("/nav_action_follow_path").unwrap();
        tokio::time::timeout(WAIT, action.wait_for_server()).await.expect("nav's action server").unwrap();
        tokio::time::sleep(Duration::from_secs(1)).await;

        let (cpu, started) = (process_cpu(), Instant::now());
        tokio::time::sleep(WINDOW).await;
        let (used, wall) = (process_cpu() - cpu, started.elapsed());

        shutdown.trigger();
        tokio::time::timeout(WAIT, task).await.expect("nav to stop").expect("nav panicked").expect("nav failed");
        // A spinning thread alone is one core, `wall`; idle nav is a few
        // percent of one.
        eprintln!("nav_core at idle: {used:?} of CPU in {wall:?}");
        assert!(used < wall / 4, "nav_core at idle used {used:?} of CPU in {wall:?}: a thread is spinning");
    });
    runtime.shutdown_timeout(Duration::from_secs(5));
}
