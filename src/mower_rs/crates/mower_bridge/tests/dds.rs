//! A planned bridge against real DDS, in one process (the pattern of
//! mower_bus_ros/tests/dds.rs): synthetic bus modules that make the
//! endpoints a converted module would, the bridge the policy plans for them,
//! and peer nodes on the process's one participant standing in for the
//! modules still on r2r, for nav2 and for the app.
//!
//! - **Isolation:** `ROS_DOMAIN_ID` 78 (mower_bus_ros's tests use 77, the
//!   resolver oracle 79), and one test at a time: the names a bridge claims
//!   are process-wide (mower_bus_ros README, "One direction per name").
//! - **Waiting:** discovery is polled for, never slept for.

use std::future::Future;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock, PoisonError};
use std::time::{Duration, Instant};

use futures::{FutureExt, Stream, StreamExt};
use mower_bridge::types::{BE_V_1, BE_V_5, R_TL_1, R_V_10};
use mower_bridge::{coverage, plan, run_standalone, Running, NODE};
use mower_bus::{Bus, BusError};
use mower_bus_ros::RosBridge;
use mower_rs_common::{ModuleCtx, ModuleResult, Remaps, Shutdown};
use r2r::geographic_msgs::msg::GeoPoint;
use r2r::qos::{DurabilityPolicy, ReliabilityPolicy};
use r2r::robot_localization::srv::ToLL;
use r2r::sensor_msgs::msg::{BatteryState, NavSatFix};
use r2r::std_msgs::msg::{Bool, String as Text};
use r2r::std_srvs::srv::Trigger;
use r2r::QosProfile;

const WAIT: Duration = Duration::from_secs(20);

/// The process's r2r context, on domain 78 and rmw_cyclonedds_cpp.
fn context() -> r2r::Context {
    static CONTEXT: OnceLock<r2r::Context> = OnceLock::new();
    CONTEXT
        .get_or_init(|| {
            assert!(
                rmw_installed("rmw_cyclonedds_cpp"),
                "these tests run on the robot's RMW: apt-get install ros-jazzy-rmw-cyclonedds-cpp"
            );
            // rcl reads both when the context is created, and only then.
            std::env::set_var("ROS_DOMAIN_ID", "78");
            std::env::set_var("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp");
            r2r::Context::create().expect("an rcl context (source /opt/ros/jazzy/setup.bash)")
        })
        .clone()
}

fn rmw_installed(rmw: &str) -> bool {
    let prefixes = std::env::var("AMENT_PREFIX_PATH").unwrap_or_default();
    prefixes.split(':').any(|prefix| std::path::Path::new(prefix).join(format!("lib/lib{rmw}.so")).exists())
}

/// One test at a time, on a runtime of its own, within a bound only a hang
/// reaches.
fn within(test: impl Future<Output = ()>) {
    static ONE_AT_A_TIME: Mutex<()> = Mutex::new(());
    let _serial: MutexGuard<'_, ()> = ONE_AT_A_TIME.lock().unwrap_or_else(PoisonError::into_inner);
    let runtime = tokio::runtime::Builder::new_multi_thread().worker_threads(2).enable_all().build().unwrap();
    runtime.block_on(async {
        tokio::time::timeout(Duration::from_secs(120), test).await.expect("the test hung");
    });
}

/// A node of the process that is not the bridge, spun on a thread of its
/// own; dropping it takes the node off the graph.
struct Peer {
    node: Arc<Mutex<r2r::Node>>,
    running: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl Peer {
    fn new(name: &str) -> Peer {
        let node = Arc::new(Mutex::new(r2r::Node::create(context(), name, "").unwrap()));
        let running = Arc::new(AtomicBool::new(true));
        let thread = {
            let (node, running) = (node.clone(), running.clone());
            std::thread::spawn(move || {
                while running.load(Ordering::Acquire) {
                    node.lock().unwrap().spin_once(Duration::from_millis(5));
                    std::thread::sleep(Duration::from_millis(1));
                }
            })
        };
        Peer { node, running, thread: Some(thread) }
    }

    fn with<R>(&self, f: impl FnOnce(&mut r2r::Node) -> R) -> R {
        f(&mut self.node.lock().unwrap())
    }

    /// The writers on `topic` the graph shows: their node's full name and
    /// their QoS.
    fn writers(&self, topic: &str) -> Vec<(String, QosProfile)> {
        self.with(|node| node.get_publishers_info_by_topic(topic, false).unwrap())
            .into_iter()
            .map(|info| (format!("{}/{}", info.node_namespace.trim_end_matches('/'), info.node_name), info.qos_profile))
            .collect()
    }
}

impl Drop for Peer {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Release);
        if let Some(thread) = self.thread.take() {
            thread.join().unwrap();
        }
    }
}

async fn eventually(what: &str, mut done: impl FnMut() -> bool) {
    let deadline = Instant::now() + WAIT;
    while !done() {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

async fn next<S: Stream + Unpin>(what: &str, stream: &mut S) -> S::Item {
    match tokio::time::timeout(WAIT, stream.next()).await {
        Ok(Some(item)) => item,
        Ok(None) => panic!("{what}: the stream ended"),
        Err(_) => panic!("timed out waiting for {what}"),
    }
}

/// The first item of `stream` that comes within 50 ms of a `publish`,
/// publishing again until one does: discovery may still be under way.
async fn published<S: Stream + Unpin>(what: &str, stream: &mut S, mut publish: impl FnMut()) -> S::Item {
    let deadline = Instant::now() + WAIT;
    loop {
        publish();
        if let Ok(Some(item)) = tokio::time::timeout(Duration::from_millis(50), stream.next()).await {
            return item;
        }
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
    }
}

/// robot_status and flutter_adapter moved to the bus, in a process that runs
/// only them: the policy's bridge carries a latched and a volatile topic out,
/// a topic in, a service the module serves and one it calls.
#[test]
fn a_planned_bridge_carries_a_bus_module_s_topics_and_services_to_and_from_dds() {
    within(async {
        let bus = Bus::new();
        // What the converted modules would make, with the manifest's names
        // and QoS. robot_status has published its latched value before the
        // bridge exists.
        let status = bus.scope("robot_status");
        let online = status.publisher::<Bool>("/robot/online", R_TL_1.bus()).unwrap();
        online.publish(&Bool { data: true }).unwrap();
        let telemetry = status.publisher::<Text>("/robot/telemetry", R_V_10.bus()).unwrap();
        let mut fixes = status.subscribe::<NavSatFix>("/fix", BE_V_5.bus()).unwrap();
        let mut check_update = status.service::<Trigger::Request, Trigger::Response>("/system/check_update").unwrap();
        let server = tokio::spawn(async move {
            while let Some(request) = check_update.next().await {
                let _ = request.respond(Trigger::Response { success: true, message: "up to date".into() });
            }
        });
        let adapter = bus.scope("flutter_adapter");
        let to_ll = adapter.client::<ToLL::Request, ToLL::Response>("/toLL").unwrap();

        let running = Running::rsd(&["status", "adapter"], |_| true);
        let planned = plan(&running, &Remaps::default()).unwrap();
        let lines = planned.lines();
        for line in [
            "to_ros /robot/online std_msgs/msg/Bool R TL 1",
            "to_ros /robot/telemetry std_msgs/msg/String R V 10",
            "from_ros /fix sensor_msgs/msg/NavSatFix BE V 5",
            "service_to_ros /system/check_update std_srvs/srv/Trigger",
            "service_from_ros /toLL robot_localization/srv/ToLL",
        ] {
            assert!(lines.iter().any(|l| l == line), "{line} in {lines:#?}");
        }
        let mut bridge = RosBridge::new(r2r::Node::create(context(), NODE, "").unwrap(), bus.scope(NODE));
        planned.declare(&mut bridge).unwrap();
        let stats = bridge.stats();
        let (stop, stopped) = tokio::sync::oneshot::channel::<()>();
        let bridging = tokio::spawn(bridge.run(stopped.map(|_| ())));

        // A latched value reaches a DDS late joiner, from a writer that is
        // transient local on the graph too.
        let app = Peer::new("dds_test_app");
        eventually("the bridge's DDS writer", || stats.topic("/robot/online").is_some_and(|s| s.dds_writer)).await;
        let mut late = app.with(|n| n.subscribe::<Bool>("/robot/online", R_TL_1.ros())).unwrap();
        assert!(next("the latched value", &mut late).await.data);
        let writers = app.writers("/robot/online");
        assert_eq!(writers.len(), 1, "{writers:?}");
        assert_eq!(writers[0].0, format!("/{NODE}"));
        assert_eq!((writers[0].1.reliability.clone(), writers[0].1.durability.clone()), (ReliabilityPolicy::Reliable, DurabilityPolicy::TransientLocal));

        // A volatile one, once a DDS reader is there.
        let mut sub = app.with(|n| n.subscribe::<Text>("/robot/telemetry", R_V_10.ros())).unwrap();
        let got = published("telemetry on DDS", &mut sub, || {
            telemetry.publish(&Text { data: "{\"ok\":true}".into() }).unwrap();
        })
        .await;
        assert_eq!(got.data, "{\"ok\":true}");
        let durability = app.writers("/robot/telemetry")[0].1.durability.clone();
        assert_eq!(durability, DurabilityPolicy::Volatile);

        // A DDS topic reaches the bus reader: the GPS driver, still on r2r.
        let gps = Peer::new("dds_test_gps");
        let fix_writer = gps.with(|n| n.create_publisher::<NavSatFix>("/fix", R_V_10.ros())).unwrap();
        let fix = published("a fix on the bus", &mut fixes, || {
            fix_writer.publish(&NavSatFix { latitude: 23.69, ..Default::default() }).unwrap();
        })
        .await;
        assert_eq!(fix.latitude, 23.69);

        // A DDS client gets the bus server's answer.
        let client = app.with(|n| n.create_client::<Trigger::Service>("/system/check_update", QosProfile::services_default())).unwrap();
        let available = r2r::Node::is_available(&client).unwrap();
        tokio::time::timeout(WAIT, available).await.expect("the bridge's DDS server").unwrap();
        let answer = tokio::time::timeout(WAIT, client.request(&Trigger::Request {}).unwrap()).await.unwrap().unwrap();
        assert_eq!((answer.success, answer.message.as_str()), (true, "up to date"));

        // A bus client reaches the DDS server: navsat, still on r2r. While
        // there is none, the bus has no server either, as r2r had none.
        eventually("the bridge to look for /toLL", || stats.service("/toLL").is_some_and(|s| s.resyncs >= 2)).await;
        assert!(!stats.service("/toLL").unwrap().bus_server);
        let refused = to_ll.request(&ToLL::Request::default());
        assert!(matches!(refused, Err(BusError::ServiceUnavailable { .. })), "refused at the call");
        let navsat = Peer::new("dds_test_navsat");
        let mut requests = navsat.with(|n| n.create_service::<ToLL::Service>("/toLL", QosProfile::services_default())).unwrap();
        let serving = tokio::spawn(async move {
            while let Some(request) = requests.next().await {
                let ll_point = GeoPoint { latitude: request.message.map_point.x, longitude: 120.5, altitude: 0.0 };
                let _ = request.respond(ToLL::Response { ll_point });
            }
        });
        // The bus server comes once the bridge's client is matched with the
        // DDS server as well: the first call is answered.
        tokio::time::timeout(WAIT, to_ll.wait_for_service()).await.expect("the bus server").unwrap();
        let point = r2r::geometry_msgs::msg::Point { x: 23.5, y: 0.0, z: 0.0 };
        let call = to_ll.request(&ToLL::Request { map_point: point }).unwrap();
        let answer = tokio::time::timeout(WAIT, call).await.expect("the answer").unwrap();
        assert_eq!((answer.ll_point.latitude, answer.ll_point.longitude), (23.5, 120.5));

        // What the synthetic modules made is what the manifest has.
        assert_eq!(coverage::gaps(&bus, &running, &Remaps::default(), &[NODE]), Vec::<String>::new());

        let _ = stop.send(());
        tokio::time::timeout(WAIT, bridging).await.expect("run returned").unwrap().unwrap();
        server.abort();
        serving.abort();
    });
}

/// A battery module in a process of its own (what `mower_bridge::module_main`
/// runs): its input comes from DDS, its outputs go out from the bridge's
/// node, and the shutdown ends both. tests/standalone.rs runs `module_main`
/// itself, with launch's `-r __node:=` in argv.
#[test]
fn the_standalone_runner_puts_every_endpoint_of_its_module_on_dds() {
    within(async {
        let shutdown = Shutdown::new();
        let m = ModuleCtx::new("battery_state", shutdown.clone());
        let runner = run_standalone("battery", context(), m, fake_battery);
        let check = async {
            let base = Peer::new("dds_test_base");
            let telemetry = base.with(|n| n.create_publisher::<Text>("/mower_base/telemetry", BE_V_1.ros())).unwrap();
            let status = Peer::new("dds_test_status");
            let mut battery = status.with(|n| n.subscribe::<BatteryState>("/battery_state", R_V_10.ros())).unwrap();
            let state = published("/battery_state from the module", &mut battery, || {
                telemetry.publish(&Text { data: "25.5".into() }).unwrap();
            })
            .await;
            assert_eq!(state.voltage, 25.5);
            // the module's node name, in the bridge's namespace
            assert_eq!(status.writers("/battery_state")[0].0, "/mower_bus_bridge/battery_state");
            shutdown.trigger();
            eventually("the bridge to leave the graph", || status.writers("/battery_state").is_empty()).await;
        };
        let (result, ()) = tokio::join!(runner, check);
        result.unwrap();
    });
}

/// What a battery module converted to the bus does, in short: a voltage in,
/// two battery states out.
async fn fake_battery(_ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let mut telemetry = m.bus.subscribe::<Text>(&m.resolve_topic("/mower_base/telemetry")?, BE_V_1.bus())?;
    let battery = m.bus.publisher::<BatteryState>(&m.resolve_topic("/battery_state")?, R_V_10.bus())?;
    let aon = m.bus.publisher::<BatteryState>(&m.resolve_topic("/aon_battery_state")?, R_V_10.bus())?;
    loop {
        tokio::select! {
            () = m.shutdown.wait() => return Ok(()),
            msg = telemetry.next() => {
                let Some(msg) = msg else { return Ok(()) };
                let state = BatteryState { voltage: msg.data.parse().unwrap_or(f32::NAN), ..Default::default() };
                battery.publish(&state)?;
                aon.publish(&state)?;
            }
        }
    }
}
