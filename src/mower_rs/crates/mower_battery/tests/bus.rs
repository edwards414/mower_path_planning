//! mower_battery on the bus (step E2 of docs/ROS_FREE_PLAN.md): the module's own `run`, its input published and its
//! two outputs read through another scope of the same bus, the way mower_rsd's other modules and the bus/DDS bridge
//! reach it.
//!
//! - **r2r:** the module keeps a node for its parameters and logs, so the tests need the process's r2r context. It is
//!   on ROS domain 80, set before the context exists, apart from other DDS users of the host; no message goes
//!   through DDS.
//! - **Turns:** one module at a time (`Battery::start`), so that two nodes named `battery_state` never share the
//!   process's rosout publisher.
//! - **Time:** the 1 Hz publish runs on tokio's clock, as before E2, so the tests wait for it in real time.

use std::sync::{Arc, Mutex, MutexGuard, OnceLock, PoisonError};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use futures::StreamExt;
use mower_bus::{Bus, Message, Qos, Scope, Subscriber};
use mower_rs_common::{ModuleCtx, ModuleResult, Remaps, Shutdown};
use r2r::sensor_msgs::msg::BatteryState;
use r2r::std_msgs::msg::String as StringMsg;

/// For anything the module has to do. A pass never waits this long.
const WAIT: Duration = Duration::from_secs(10);

/// The process's r2r context, on ROS domain 80.
fn context() -> r2r::Context {
    static CONTEXT: OnceLock<r2r::Context> = OnceLock::new();
    CONTEXT
        .get_or_init(|| {
            // rcl reads it when the context is created, and only then.
            std::env::set_var("ROS_DOMAIN_ID", "80");
            r2r::Context::create().expect("an rcl context (source /opt/ros/jazzy/setup.bash)")
        })
        .clone()
}

/// mower_base's `/mower_base/telemetry` writer: best effort, keep last 1.
fn base_qos() -> Qos {
    Qos::default().keep_last(1).best_effort()
}

/// An STM32 frame: the pack meter online at `pack_v` and the always-on cell at 3.78 V. The ADC's main channel says
/// 21 V, which must lose to the meter.
fn frame(pack_v: f64) -> StringMsg {
    let doc = serde_json::json!({
        "charger": {"valid": true, "online": true, "voltage_v": pack_v, "current_a": 1.5},
        "analog": {"valid": true, "main_battery_valid": true, "main_battery_v": 21.0,
                   "aon_battery_valid": true, "aon_battery_v": 3.78},
    });
    StringMsg { data: doc.to_string() }
}

/// One start of the module, on `bus` with the remap rules `rules` (`-r` arguments). It holds the turn until it is
/// dropped.
struct Battery {
    shutdown: Shutdown,
    scope: Scope,
    task: tokio::task::JoinHandle<ModuleResult>,
    _turn: MutexGuard<'static, ()>,
}

impl Battery {
    fn start(bus: &Bus, rules: &[&str]) -> Battery {
        static TURN: Mutex<()> = Mutex::new(());
        let turn = TURN.lock().unwrap_or_else(PoisonError::into_inner);
        let argv: Vec<String> = std::iter::once("--ros-args")
            .chain(rules.iter().flat_map(|rule| ["-r", *rule]))
            .map(str::to_string)
            .collect();
        let remaps = Arc::new(Remaps::from_args(&argv).unwrap());
        let shutdown = Shutdown::new();
        let scope = bus.scope("battery_state");
        let m = ModuleCtx::new("battery_state", shutdown.clone()).with_bus(scope.clone()).with_remaps(remaps);
        let task = tokio::spawn(mower_battery::run(context(), m));
        Battery { shutdown, scope, task, _turn: turn }
    }

    /// What mower_rsd does: the module stops when asked, then its scope is closed.
    async fn stop(self) {
        self.shutdown.trigger();
        let result = tokio::time::timeout(WAIT, self.task).await.expect("the module did not stop");
        result.expect("the module panicked").expect("the module failed");
        self.scope.close();
    }
}

async fn next<T: Message>(sub: &mut Subscriber<T>) -> Arc<T> {
    tokio::time::timeout(WAIT, sub.next()).await.expect("nothing arrived").expect("the test's scope closed")
}

fn close(a: f32, b: f64) -> bool {
    (a as f64 - b).abs() < 1e-4
}

fn stamp_s(msg: &BatteryState) -> f64 {
    msg.header.stamp.sec as f64 + msg.header.stamp.nanosec as f64 * 1e-9
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn the_base_telemetry_becomes_both_battery_states_once_a_second() {
    let bus = Bus::new();
    let test = bus.scope("test");
    let mut main = test.subscribe::<BatteryState>("/battery_state", Qos::default()).unwrap();
    let mut aon = test.subscribe::<BatteryState>("/aon_battery_state", Qos::default()).unwrap();
    let base = test.publisher::<StringMsg>("/mower_base/telemetry", base_qos()).unwrap();
    let battery = Battery::start(&bus, &[]);
    let t0 = Instant::now();

    // The first publish comes at once (tokio's interval), before any telemetry: nothing is known yet.
    let (first, first_aon) = (next(&mut main).await, next(&mut aon).await);
    assert!(t0.elapsed() < Duration::from_millis(900), "{:?}", t0.elapsed());
    assert!(!first.present && !first_aon.present);
    assert!(first.voltage.is_nan() && first.percentage.is_nan() && first.current.is_nan());
    assert_eq!((first.power_supply_status, first.power_supply_health), (0, 0), "UNKNOWN, UNKNOWN");
    assert_eq!(first.power_supply_technology, 2, "LION");
    assert_eq!(first.header.frame_id, "base_footprint");
    assert!(first.cell_voltage.is_empty());

    // 22.68 V is 3.78 V a cell: 50 % on the OCV table, discharging (below charger_present_min_v).
    base.publish(&frame(22.68)).unwrap();
    let (msg, aon_msg) = (next(&mut main).await, next(&mut aon).await);
    // The next one a period later, by the stamps the ticks put on them.
    let period = stamp_s(&msg) - stamp_s(&first);
    assert!((0.95..1.5).contains(&period), "{period}");
    assert!(msg.present);
    assert!(close(msg.voltage, 22.68) && close(msg.percentage, 0.5), "{msg:?}");
    assert!(msg.current.is_nan(), "meter_current_wired is off: no current");
    assert_eq!((msg.power_supply_status, msg.power_supply_health), (2, 1), "DISCHARGING, GOOD");
    assert_eq!(msg.cell_voltage.len(), 6);
    assert!(msg.cell_voltage.iter().all(|v| close(*v, 3.78)));
    assert!(msg.design_capacity.is_nan() && msg.capacity.is_nan() && msg.charge.is_nan(), "capacity unknown");
    let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs() as i64;
    assert!((msg.header.stamp.sec as i64 - now).abs() <= 2, "stamped with the wall clock");
    assert!(aon_msg.present);
    assert!(close(aon_msg.voltage, 3.78) && close(aon_msg.percentage, 0.5), "{aon_msg:?}");
    assert_eq!(aon_msg.cell_voltage.len(), 1);
    assert_eq!(aon_msg.header.frame_id, "base_footprint");

    // Both writers are volatile, as before: a reader that joins late gets no history.
    let mut late = test.subscribe::<BatteryState>("/battery_state", Qos::latched()).unwrap();
    assert!(late.try_recv().is_err());

    // The endpoints, as the bridge sees them: one reader, two writers, the r2r types.
    let topics = bus.topics();
    let topic = |name: &str| topics.iter().find(|t| t.name == name).unwrap_or_else(|| panic!("{name}"));
    for name in ["/battery_state", "/aon_battery_state"] {
        assert_eq!(topic(name).publishers, ["battery_state"], "{name}");
        assert_eq!(topic(name).ros_type.as_deref(), Some("sensor_msgs/msg/BatteryState"));
    }
    assert_eq!(topic("/mower_base/telemetry").subscribers, ["battery_state"]);
    assert_eq!(topic("/mower_base/telemetry").ros_type.as_deref(), Some("std_msgs/msg/String"));
    assert!(bus.services().is_empty());

    // Every endpoint is one mower_bridge's manifest has, so that the bridge carries it (its README, "Moving a module").
    mower_bridge::coverage::assert_in_manifest(&ModuleCtx::new("battery_state", Shutdown::new()).with_bus(battery.scope.clone()));
    battery.stop().await;
    // The closed scope ends the module's endpoints and its publish loop.
    let topics = bus.topics();
    assert!(topics.iter().all(|t| !t.publishers.iter().chain(&t.subscribers).any(|s| s == "battery_state")));
}

/// The telemetry reader keeps only the newest frame (KEEP_LAST 1, as on DDS): two frames that arrive before the
/// module reads give the second one's voltage. With a deeper queue the module would read both, and the first one,
/// with no time to filter towards the second, would stay.
#[tokio::test(flavor = "current_thread")]
async fn the_telemetry_reader_keeps_the_newest_frame_only() {
    let bus = Bus::new();
    let test = bus.scope("test");
    let mut main = test.subscribe::<BatteryState>("/battery_state", Qos::default()).unwrap();
    let base = test.publisher::<StringMsg>("/mower_base/telemetry", base_qos()).unwrap();
    let battery = Battery::start(&bus, &[]);
    assert!(!next(&mut main).await.present);
    // One thread: the module's reader runs only when the test awaits, after both frames are in.
    base.publish(&frame(22.68)).unwrap();
    base.publish(&frame(23.4)).unwrap();
    let msg = next(&mut main).await;
    assert!(close(msg.voltage, 23.4), "{msg:?}");
    battery.stop().await;
}

/// Every name goes through the process's remap rules for the node, the absolute ones too; a rule for another node
/// changes nothing.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn the_names_follow_the_remap_rules_of_the_node() {
    let bus = Bus::new();
    let test = bus.scope("test");
    let mut main = test.subscribe::<BatteryState>("/power/main", Qos::default()).unwrap();
    let base = test.publisher::<StringMsg>("/base/telemetry", base_qos()).unwrap();
    let battery = Battery::start(
        &bus,
        &[
            "battery_state:/battery_state:=/power/main",
            "battery_state:/mower_base/telemetry:=/base/telemetry",
            "robot_status:/aon_battery_state:=/power/aon",
        ],
    );
    assert!(!next(&mut main).await.present);
    base.publish(&frame(22.68)).unwrap();
    assert!(next(&mut main).await.present);
    let topics = bus.topics();
    let writers = |name: &str| topics.iter().find(|t| t.name == name).map(|t| t.publishers.clone()).unwrap_or_default();
    assert_eq!(writers("/aon_battery_state"), ["battery_state"]);
    assert!(writers("/battery_state").is_empty() && writers("/power/aon").is_empty());
    battery.stop().await;
}
