//! A moved module's own binary as robot.launch.py's roll-back
//! (`rust_daemon:=false`) launches it: `name=battery_state`, which puts a bare
//! `-r __node:=battery_state` in argv. `mower_bridge::module_main` end to end,
//! with a battery module that keeps its r2r node and has its IO on the bus.
//!
//! - The bare rule renames every node of the process. The module's node
//!   becomes `/battery_state`; the bridge's keeps a fully qualified name of
//!   its own, `/mower_bus_bridge/battery_state`.
//! - The module's input comes from DDS and its outputs reach DDS.
//!
//! rcl reads argv when the context is made, and libtest refuses arguments it
//! does not know, so this test runs itself again with the rule, on ROS domain
//! 78 and rmw_cyclonedds_cpp, and that run does the work.

use std::process::{Command, ExitCode};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use futures::{FutureExt, StreamExt};
use mower_bridge::types::{BE_V_1, R_V_10};
use mower_rs_common::{ModuleCtx, ModuleResult};
use r2r::sensor_msgs::msg::BatteryState;
use r2r::std_msgs::msg::String as Text;

const NAME: &str = "a_moved_module_s_binary_runs_under_launch_s_node_name";
/// Set in the run that does the work.
const CHILD: &str = "MOWER_BRIDGE_STANDALONE";
const WAIT: Duration = Duration::from_secs(20);

/// Set by the check once it is done: the module then returns.
static DONE: AtomicBool = AtomicBool::new(false);
static FAILURES: Mutex<Vec<String>> = Mutex::new(Vec::new());

fn fail(why: String) {
    FAILURES.lock().unwrap().push(why);
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if std::env::var_os(CHILD).is_some() {
        return binary();
    }
    if args.iter().any(|a| a == "--list") {
        println!("{NAME}: test");
        return ExitCode::SUCCESS;
    }
    let filters: Vec<&String> = args.iter().filter(|a| !a.starts_with('-')).collect();
    if !filters.is_empty() && !filters.iter().any(|f| NAME.contains(f.as_str())) {
        println!("running 0 tests");
        return ExitCode::SUCCESS;
    }
    println!("running 1 test");
    let status = Command::new(std::env::current_exe().expect("the test binary"))
        .args(["--ros-args", "-r", "__node:=battery_state"])
        .env(CHILD, "1")
        .env("ROS_DOMAIN_ID", "78")
        .env("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
        .status()
        .expect("the binary runs");
    if status.success() {
        println!("test {NAME} ... ok");
        ExitCode::SUCCESS
    } else {
        println!("test {NAME} ... FAILED ({status})");
        ExitCode::FAILURE
    }
}

/// What `mower_battery`'s `main.rs` becomes once battery moves, with the
/// check on a thread of its own.
fn binary() -> ExitCode {
    let check = std::thread::spawn(check);
    let code = mower_bridge::module_main("battery", "mower_battery", 2, battery);
    if check.join().is_err() {
        fail("the check panicked".into());
    }
    let failures = FAILURES.lock().unwrap();
    if failures.is_empty() && code == ExitCode::SUCCESS {
        ExitCode::SUCCESS
    } else {
        eprintln!("module_main ended with {code:?}:\n{}", failures.join("\n"));
        ExitCode::FAILURE
    }
}

/// A battery module moved to the bus: its r2r node stays (E2 decision 1), its
/// IO is on the bus. It ends when the check is done.
async fn battery(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let node = r2r::Node::create(ctx, &m.node_name, &m.namespace)?;
    let named = node.fully_qualified_name()?;
    if named != "/battery_state" {
        fail(format!("launch's rule should have named the module's node /battery_state, not {named}"));
    }
    let mut telemetry = m.bus.subscribe::<Text>(&m.resolve_topic("/mower_base/telemetry")?, BE_V_1.bus())?;
    let battery = m.bus.publisher::<BatteryState>(&m.resolve_topic("/battery_state")?, R_V_10.bus())?;
    let aon = m.bus.publisher::<BatteryState>(&m.resolve_topic("/aon_battery_state")?, R_V_10.bus())?;
    while !DONE.load(Ordering::Acquire) {
        if let Ok(Some(msg)) = tokio::time::timeout(Duration::from_millis(20), telemetry.next()).await {
            let state = BatteryState { voltage: msg.data.parse().unwrap_or(f32::NAN), ..Default::default() };
            battery.publish(&state)?;
            aon.publish(&state)?;
        }
    }
    drop(node);
    Ok(())
}

/// Whatever the checks find, the module is then told to end, so that
/// `module_main` returns.
fn check() {
    if std::panic::catch_unwind(checks).is_err() {
        fail("the check panicked".into());
    }
    DONE.store(true, Ordering::Release);
}

/// The base and robot_status, still on DDS. Their nodes are renamed by the
/// same rule, which does not matter here.
fn checks() {
    let ctx = r2r::Context::create().expect("an rcl context (source /opt/ros/jazzy/setup.bash)");
    let mut base = r2r::Node::create(ctx.clone(), "check_base", "").unwrap();
    let telemetry = base.create_publisher::<Text>("/mower_base/telemetry", BE_V_1.ros()).unwrap();
    let mut status = r2r::Node::create(ctx, "check_status", "").unwrap();
    let mut states = status.subscribe::<BatteryState>("/battery_state", R_V_10.ros()).unwrap();
    let deadline = Instant::now() + WAIT;
    let state = loop {
        telemetry.publish(&Text { data: "25.5".into() }).unwrap();
        base.spin_once(Duration::from_millis(1));
        status.spin_once(Duration::from_millis(50));
        if let Some(Some(state)) = states.next().now_or_never() {
            break Some(state);
        }
        if Instant::now() > deadline {
            break None;
        }
    };
    match state {
        Some(state) if state.voltage == 25.5 => {}
        other => fail(format!("no /battery_state of 25.5 V from the module: {other:?}")),
    }
    let writers: Vec<String> = status
        .get_publishers_info_by_topic("/battery_state", false)
        .unwrap()
        .into_iter()
        .map(|w| format!("{}/{}", w.node_namespace.trim_end_matches('/'), w.node_name))
        .collect();
    if writers != ["/mower_bus_bridge/battery_state"] {
        fail(format!("/battery_state should be written by the bridge's /mower_bus_bridge/battery_state, not {writers:?}"));
    }
}
