//! The bridge against real DDS, in one process: the bridge's node and a peer
//! node that stands in for nav2, the app's ws_bridge, robot_state_publisher
//! and the rest, all on the process's one participant, as in mower_rsd.
//!
//! - **Isolation:** `ROS_DOMAIN_ID` 77, set before the process's r2r context
//!   exists (other containers on a dev host speak DDS), and names of its own
//!   for every test (the context, and so the participant, is process-wide).
//! - **RMW:** rmw_cyclonedds_cpp, the robot's. What a system-default QoS
//!   resolves to, and which readers a writer counts, are the RMW's.
//! - **Waiting:** discovery is polled for, never slept for. An absence is
//!   asserted only after the bridge has said what it did (its stats) or
//!   through an order that a stale value would break.

use std::future::Future;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use futures::{FutureExt, Stream, StreamExt};
use mower_bus::{Bus, BusError, Qos, Scope};
use mower_bus_ros::{BridgeError, BridgeMessage, BridgeStats, Direction, EntryStats, RosBridge};
use r2r::builtin_interfaces::msg::Time;
use r2r::std_msgs::msg::{Header, String as Text};
use r2r::std_srvs::srv::SetBool;
use r2r::QosProfile;

/// For anything that waits on DDS. Generous: CI runners are small and
/// shared, and a pass never waits this long.
const WAIT: Duration = Duration::from_secs(20);

/// The process's r2r context, on domain 77 and rmw_cyclonedds_cpp.
fn context() -> r2r::Context {
    static CONTEXT: OnceLock<r2r::Context> = OnceLock::new();
    CONTEXT
        .get_or_init(|| {
            assert!(
                rmw_installed("rmw_cyclonedds_cpp"),
                "these tests run on the robot's RMW: apt-get install ros-jazzy-rmw-cyclonedds-cpp"
            );
            // rcl reads both when the context is created, and only then.
            std::env::set_var("ROS_DOMAIN_ID", "77");
            std::env::set_var("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp");
            r2r::Context::create().expect("an rcl context (source /opt/ros/jazzy/setup.bash)")
        })
        .clone()
}

fn rmw_installed(rmw: &str) -> bool {
    let prefixes = std::env::var("AMENT_PREFIX_PATH").unwrap_or_default();
    prefixes.split(':').any(|prefix| std::path::Path::new(prefix).join(format!("lib/lib{rmw}.so")).exists())
}

/// Run one test on a runtime of its own, within a bound that only a hang
/// reaches.
fn within(test: impl Future<Output = ()>) {
    let runtime = tokio::runtime::Builder::new_multi_thread().worker_threads(2).enable_all().build().unwrap();
    runtime.block_on(async {
        tokio::time::timeout(Duration::from_secs(120), test).await.expect("the test hung");
    });
}

/// A node of the process that is not the bridge: spun on a thread of its
/// own, its endpoints made between two spins. Dropping it takes the node
/// off the graph, as a ROS process that exits.
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
                    // Let `with` in between: the lock is not fair.
                    std::thread::sleep(Duration::from_millis(1));
                }
            })
        };
        Peer { node, running, thread: Some(thread) }
    }

    fn with<R>(&self, f: impl FnOnce(&mut r2r::Node) -> R) -> R {
        f(&mut self.node.lock().unwrap())
    }

    /// The writers on `topic` that the graph shows this node.
    fn writers(&self, topic: &str) -> Vec<r2r::qos::QosProfile> {
        self.with(|node| node.get_publishers_info_by_topic(topic, false).unwrap())
            .into_iter()
            .map(|info| info.qos_profile)
            .collect()
    }

    fn writer_gids(&self, topic: &str) -> Vec<Vec<u8>> {
        self.with(|node| node.get_publishers_info_by_topic(topic, false).unwrap())
            .into_iter()
            .map(|info| info.endpoint_gid.to_vec())
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

fn new_bridge(node: &str, scope: &Scope) -> RosBridge {
    RosBridge::new(r2r::Node::create(context(), node, "").unwrap(), scope.clone())
}

/// A bridge running on the test's runtime, as a module would run it.
struct Running {
    stats: BridgeStats,
    stop: Option<tokio::sync::oneshot::Sender<()>>,
    task: tokio::task::JoinHandle<Result<(), BridgeError>>,
}

fn start(bridge: RosBridge) -> Running {
    let stats = bridge.stats();
    let (stop, stopped) = tokio::sync::oneshot::channel::<()>();
    let task = tokio::spawn(bridge.run(stopped.map(|_| ())));
    Running { stats, stop: Some(stop), task }
}

impl Running {
    fn topic(&self, name: &str) -> EntryStats {
        self.stats.topic(name).expect("a bridged topic")
    }

    fn service(&self, name: &str) -> EntryStats {
        self.stats.service(name).expect("a bridged service")
    }

    /// Trip the shutdown and wait for `run` to return.
    async fn stop(mut self) -> Result<(), BridgeError> {
        let _ = self.stop.take().expect("not stopped yet").send(());
        self.ended().await
    }

    async fn ended(self) -> Result<(), BridgeError> {
        tokio::time::timeout(WAIT, self.task).await.expect("run returned").expect("run did not panic")
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

fn text(data: impl ToString) -> Text {
    Text { data: data.to_string() }
}

/// What the modules' `latched()` helpers build: R TL 1.
fn latched() -> QosProfile {
    QosProfile::default().keep_last(1).reliable().transient_local()
}

/// Transient local with room for two: a stale value left in the bridge's
/// DDS writer would reach a late joiner *before* the fresh one, whatever
/// the timing of its match.
fn latched_2() -> QosProfile {
    QosProfile::default().keep_last(2).reliable().transient_local()
}

// ---------------------------------------------------------------------------
// What the bridge relies on in r2r and rmw_cyclonedds
// ---------------------------------------------------------------------------

/// The volatile skip (README.md, "volatile topics skip work nobody wants")
/// rests on this: production is one participant, so the readers that matter
/// are mostly other nodes of the same process. And the presence proposal
/// (README.md, "Graph presence") rests on the graph query seeing their
/// writers, and losing them with their node.
#[test]
fn a_writer_counts_the_readers_of_other_nodes_in_the_process_and_the_graph_shows_their_writers() {
    within(async {
        let topic = "/bridge_test/same_process";
        let writer = Peer::new("same_process_writer");
        let publisher = writer.with(|n| n.create_publisher::<Text>(topic, QosProfile::default())).unwrap();
        assert_eq!(publisher.get_inter_process_subscription_count().unwrap(), 0);
        let reader = Peer::new("same_process_reader");
        let subscription = reader.with(|n| n.subscribe::<Text>(topic, QosProfile::default())).unwrap();
        eventually("the reader of another node of the process to count", || {
            publisher.get_inter_process_subscription_count().unwrap() == 1
        })
        .await;
        eventually("the graph to show the other node's writer", || reader.writers(topic).len() == 1).await;
        drop(subscription);
        drop(reader);
        eventually("the count to drop with the reader's node", || {
            publisher.get_inter_process_subscription_count().unwrap() == 0
        })
        .await;
        let observer = Peer::new("same_process_observer");
        eventually("the graph to show the writer", || observer.writers(topic).len() == 1).await;
        drop(publisher);
        drop(writer);
        eventually("the graph to lose the writer with its node", || observer.writers(topic).is_empty()).await;
    });
}

// ---------------------------------------------------------------------------
// to_ros
// ---------------------------------------------------------------------------

#[test]
fn a_volatile_topic_reaches_dds_only_while_a_dds_reader_is_matched() {
    within(async {
        let topic = "/bridge_test/volatile";
        let bus = Bus::new();
        let module = bus.scope("robot_status");
        let publisher = module.publisher::<Text>(topic, Qos::default()).unwrap();
        let mut bridge = new_bridge("volatile_bridge", &bus.scope("bridge"));
        bridge.to_ros::<Text>(topic, QosProfile::default()).unwrap();
        let running = start(bridge);
        eventually("the DDS writer", || running.topic(topic).dds_writer).await;

        for i in 0..5 {
            publisher.publish(&text(i)).unwrap();
        }
        eventually("the five to be skipped", || running.topic(topic).skipped == 5).await;
        assert_eq!(running.topic(topic).forwarded, 0);

        let app = Peer::new("volatile_app");
        let mut sub = app.with(|n| n.subscribe::<Text>(topic, QosProfile::default())).unwrap();
        // Publish until the match is complete, then in order.
        let deadline = Instant::now() + WAIT;
        for i in 100.. {
            publisher.publish(&text(i)).unwrap();
            if let Ok(Some(_)) = tokio::time::timeout(Duration::from_millis(50), sub.next()).await {
                break;
            }
            assert!(Instant::now() < deadline, "timed out waiting for the match");
        }
        for word in ["a", "b", "c"] {
            publisher.publish(&text(word)).unwrap();
        }
        while next("a", &mut sub).await.data != "a" {}
        assert_eq!(next("b", &mut sub).await.data, "b");
        assert_eq!(next("c", &mut sub).await.data, "c");
        eventually("the four counted", || running.topic(topic).forwarded >= 4).await;
        assert!(running.topic(topic).skipped >= 5);
        running.stop().await.unwrap();
    });
}

/// README.md, "QoS", on the DDS side (the bus side is `bus_qos`'s unit
/// tests): the bridge's DDS writer announces the source's reliability and
/// durability. The graph carries no history or depth.
#[test]
fn the_dds_writer_announces_the_source_s_qos() {
    use r2r::qos::{DurabilityPolicy::*, ReliabilityPolicy::*};
    within(async {
        let bus = Bus::new();
        let module = bus.scope("mower_base");
        let mut bridge = new_bridge("announce_bridge", &bus.scope("bridge"));
        let cases = [
            // mower_base's /odom: what rmw_cyclonedds makes of the system default
            ("/bridge_test/announce_system_default", QosProfile::system_default(), Reliable, Volatile),
            ("/bridge_test/announce_sensor_data", QosProfile::sensor_data(), BestEffort, Volatile),
            ("/bridge_test/announce_latched", latched(), Reliable, TransientLocal),
        ];
        let mut publishers = Vec::new();
        for (topic, qos, _, _) in &cases {
            publishers.push(module.publisher::<Text>(topic, mower_bus_ros::bus_qos(qos).unwrap()).unwrap());
            bridge.to_ros::<Text>(topic, qos.clone()).unwrap();
        }
        let running = start(bridge);
        let graph = Peer::new("announce_graph");
        for (topic, _, reliability, durability) in cases {
            eventually("the bridge's DDS writer", || graph.writers(topic).len() == 1).await;
            let announced = graph.writers(topic).remove(0);
            assert_eq!((announced.reliability, announced.durability), (reliability, durability), "{topic}");
        }
        running.stop().await.unwrap();
    });
}

#[test]
fn a_late_dds_joiner_gets_the_latest_bus_value_and_a_latched_topic_is_never_skipped() {
    within(async {
        let topic = "/bridge_test/latest";
        let bus = Bus::new();
        let module = bus.scope("map_manage");
        let publisher = module.publisher::<Text>(topic, Qos::latched()).unwrap();
        publisher.publish(&text("v1")).unwrap();
        publisher.publish(&text("v2")).unwrap();
        let mut bridge = new_bridge("latest_bridge", &bus.scope("bridge"));
        bridge.to_ros::<Text>(topic, latched()).unwrap();
        let running = start(bridge);
        // No DDS reader yet: a latched value is published all the same.
        eventually("the retained value to reach DDS", || running.topic(topic).forwarded == 1).await;

        let nav2 = Peer::new("latest_nav2");
        let mut late = nav2.with(|n| n.subscribe::<Text>(topic, latched())).unwrap();
        assert_eq!(next("the latest value", &mut late).await.data, "v2");
        publisher.publish(&text("v3")).unwrap();
        assert_eq!(next("a live value", &mut late).await.data, "v3");
        eventually("v3 counted", || running.topic(topic).forwarded == 2).await;
        let stats = running.topic(topic);
        assert_eq!((stats.skipped, stats.stale), (0, 0));
        // transient local on DDS too, or nav2's StaticLayer would never match
        let writers = nav2.writers(topic);
        assert_eq!(writers.len(), 1);
        assert_eq!(writers[0].durability, r2r::qos::DurabilityPolicy::TransientLocal);
        assert_eq!(writers[0].reliability, r2r::qos::ReliabilityPolicy::Reliable);
        running.stop().await.unwrap();
    });
}

#[test]
fn a_late_dds_joiner_gets_nothing_once_the_last_bus_writer_is_gone_and_a_new_writer_s_value_again() {
    within(async {
        let topic = "/bridge_test/writer_gone";
        let bus = Bus::new();
        let module = bus.scope("robot_status");
        let publisher = module.publisher::<Text>(topic, Qos::latched().keep_last(2)).unwrap();
        let mut bridge = new_bridge("writer_gone_bridge", &bus.scope("bridge"));
        bridge.to_ros::<Text>(topic, latched_2()).unwrap();
        let running = start(bridge);
        let app = Peer::new("writer_gone_app");
        let mut early = app.with(|n| n.subscribe::<Text>(topic, latched_2())).unwrap();
        publisher.publish(&text("a")).unwrap();
        publisher.publish(&text("b")).unwrap();
        assert_eq!(next("a", &mut early).await.data, "a");
        assert_eq!(next("b", &mut early).await.data, "b");

        // The module ends: on the bus its history goes with its scope, and
        // on DDS the bridge's writer goes with it.
        module.close();
        eventually("the bridge to drop its DDS writer", || !running.topic(topic).dds_writer).await;
        eventually("the graph to lose it", || app.writers(topic).is_empty()).await;
        let mut late = app.with(|n| n.subscribe::<Text>(topic, latched_2())).unwrap();

        // The module comes back. Had "a" or "b" survived anywhere on DDS,
        // `late` would get it before "c".
        let restarted = bus.scope("robot_status");
        let publisher = restarted.publisher::<Text>(topic, Qos::latched().keep_last(2)).unwrap();
        publisher.publish(&text("c")).unwrap();
        assert_eq!(next("the new writer's value", &mut late).await.data, "c");
        assert_eq!(next("the new writer's value, on the early reader", &mut early).await.data, "c");
        let mut later = app.with(|n| n.subscribe::<Text>(topic, latched_2())).unwrap();
        assert_eq!(next("the new writer's value, for a later joiner", &mut later).await.data, "c");
        assert!(running.topic(topic).resyncs >= 2);
        running.stop().await.unwrap();
    });
}

#[test]
fn a_writer_replaced_before_the_bridge_looks_leaves_no_stale_value_on_dds() {
    within(async {
        let topic = "/bridge_test/replaced";
        let bus = Bus::new();
        let module = bus.scope("flutter_adapter");
        let publisher = module.publisher::<Text>(topic, Qos::latched().keep_last(2)).unwrap();
        let mut bridge = new_bridge("replaced_bridge", &bus.scope("bridge"));
        bridge.to_ros::<Text>(topic, latched_2()).unwrap();
        let running = start(bridge);
        publisher.publish(&text("old")).unwrap();
        eventually("the old value on DDS", || running.topic(topic).forwarded == 1).await;
        let app = Peer::new("replaced_app");
        eventually("the bridge's writer on the graph", || app.writers(topic).len() == 1).await;
        let old_writer = app.writer_gids(topic);
        let resyncs = running.topic(topic).resyncs;

        // A restart in one go: the bridge may see one change with an
        // unchanged count, which still says the writer is another.
        module.close();
        let restarted = bus.scope("flutter_adapter");
        let publisher = restarted.publisher::<Text>(topic, Qos::latched().keep_last(2)).unwrap();
        eventually("the bridge to look", || running.topic(topic).resyncs > resyncs).await;
        eventually("a new DDS writer", || {
            let now = app.writer_gids(topic);
            now.len() == 1 && now != old_writer
        })
        .await;
        let mut late = app.with(|n| n.subscribe::<Text>(topic, latched_2())).unwrap();
        publisher.publish(&text("new")).unwrap();
        assert_eq!(next("the new writer's value, not the old one", &mut late).await.data, "new");
        running.stop().await.unwrap();
    });
}

#[test]
fn with_several_bus_writers_dds_keeps_what_the_bus_retains() {
    within(async {
        let topic = "/bridge_test/several";
        let bus = Bus::new();
        let status = bus.scope("robot_status");
        let autotune = bus.scope("pid_autotune");
        let a = status.publisher::<Text>(topic, Qos::latched()).unwrap();
        let b = autotune.publisher::<Text>(topic, Qos::latched()).unwrap();
        let mut bridge = new_bridge("several_bridge", &bus.scope("bridge"));
        bridge.to_ros::<Text>(topic, latched()).unwrap();
        let running = start(bridge);
        eventually("the DDS writer", || running.topic(topic).dds_writer).await;
        a.publish(&text("from status")).unwrap();
        b.publish(&text("from autotune")).unwrap();
        eventually("both on DDS", || running.topic(topic).forwarded == 2).await;

        // pid_autotune ends: a bus late joiner now gets robot_status's
        // value, and so must a DDS late joiner.
        autotune.close();
        eventually("the retained value to go out again", || running.topic(topic).forwarded == 3).await;
        let base = Peer::new("several_base");
        let mut late = base.with(|n| n.subscribe::<Text>(topic, latched())).unwrap();
        assert_eq!(next("what the bus retains", &mut late).await.data, "from status");
        let mut on_bus = bus.scope("probe").subscribe::<Text>(topic, Qos::latched()).unwrap();
        assert_eq!(on_bus.try_recv().unwrap().data, "from status");
        running.stop().await.unwrap();
    });
}

// ---------------------------------------------------------------------------
// from_ros
// ---------------------------------------------------------------------------

#[test]
fn from_ros_forwards_dds_samples_to_bus_readers_in_order() {
    within(async {
        let topic = "/bridge_test/joy";
        let app = Peer::new("joy_app");
        let publisher = app.with(|n| n.create_publisher::<Header>(topic, QosProfile::default())).unwrap();
        let bus = Bus::new();
        let mut guard = bus.scope("manual_velocity_guard").subscribe::<Header>(topic, Qos::default()).unwrap();
        let mut bridge = new_bridge("joy_bridge", &bus.scope("bridge"));
        bridge.from_ros::<Header>(topic, QosProfile::default()).unwrap();
        let running = start(bridge);
        eventually("the bridge's DDS reader to match", || publisher.get_inter_process_subscription_count().unwrap() == 1).await;
        for sec in 1..=5 {
            publisher.publish(&Header { stamp: Time { sec, nanosec: 0 }, frame_id: "joy".into() }).unwrap();
        }
        for sec in 1..=5 {
            let msg = tokio::time::timeout(WAIT, guard.recv()).await.unwrap().unwrap();
            assert_eq!(msg.stamp.sec, sec);
        }
        // counted once the publish has returned, which a reader may beat
        eventually("all five counted", || running.topic(topic).forwarded == 5).await;
        running.stop().await.unwrap();
    });
}

#[test]
fn from_ros_replays_a_latched_dds_value_to_a_late_bus_subscriber() {
    within(async {
        let topic = "/bridge_test/static_frames";
        let rsp = Peer::new("static_frames_rsp");
        let publisher = rsp.with(|n| n.create_publisher::<Text>(topic, latched())).unwrap();
        // before the bridge exists
        publisher.publish(&text("imu_link")).unwrap();
        let bus = Bus::new();
        let mut bridge = new_bridge("static_frames_bridge", &bus.scope("bridge"));
        bridge.from_ros::<Text>(topic, latched()).unwrap();
        let running = start(bridge);
        eventually("the DDS replay to reach the bus", || running.topic(topic).forwarded == 1).await;

        // The bridge's bus writer retains it, so a bus late joiner has it at
        // once.
        let localize = bus.scope("ekf_filter_node_map");
        let mut late = localize.subscribe::<Text>(topic, Qos::latched()).unwrap();
        assert_eq!(late.try_recv().unwrap().data, "imu_link");
        publisher.publish(&text("gps_link")).unwrap();
        assert_eq!(tokio::time::timeout(WAIT, late.recv()).await.unwrap().unwrap().data, "gps_link");
        running.stop().await.unwrap();
    });
}

/// docs/BUS_TOPICS.md 5.5 f: the bridge's bus writer is a publisher for as
/// long as the bridge runs, whether or not DDS has one.
#[test]
fn a_from_ros_bus_writer_counts_as_a_publisher_while_the_bridge_runs() {
    within(async {
        let topic = "/bridge_test/guarded_cmd_vel";
        let bus = Bus::new();
        let base = bus.scope("mower_base");
        let sub = base.subscribe::<Text>(topic, Qos::default()).unwrap();
        let mut bridge = new_bridge("presence_bridge", &bus.scope("bridge"));
        bridge.from_ros::<Text>(topic, QosProfile::default()).unwrap();
        assert_eq!(sub.publisher_count(), 1, "from the declaration on, with no DDS writer anywhere");
        let running = start(bridge);
        running.stop().await.unwrap();
        assert_eq!(sub.publisher_count(), 0);
    });
}

// ---------------------------------------------------------------------------
// One direction per name
// ---------------------------------------------------------------------------

#[test]
fn a_name_crosses_once_in_one_direction_and_a_refusal_leaves_nothing_behind() {
    within(async {
        let name = "/bridge_test/one_way";
        let bus = Bus::new();
        let mut bridge = new_bridge("one_way_bridge", &bus.scope("bridge"));
        bridge.to_ros::<Text>(name, QosProfile::default()).unwrap();
        let refused = |result: Result<(), BridgeError>, existing: Direction, requested: Direction| match result {
            Err(BridgeError::AlreadyBridged { name: n, existing: e, requested: r }) => {
                assert_eq!((n.as_str(), e, r), (name, existing, requested));
            }
            other => panic!("{requested} {name} must be refused, got {other:?}"),
        };
        refused(bridge.from_ros::<Text>(name, QosProfile::default()), Direction::ToRos, Direction::FromRos);
        refused(bridge.to_ros::<Text>(name, QosProfile::default()), Direction::ToRos, Direction::ToRos);
        // A service of the same name is another namespace, with the same
        // rule.
        bridge.service_to_ros::<SetBool::Service>(name).unwrap();
        refused(bridge.service_from_ros::<SetBool::Service>(name), Direction::ServiceToRos, Direction::ServiceFromRos);
        refused(bridge.service_to_ros::<SetBool::Service>(name), Direction::ServiceToRos, Direction::ServiceToRos);
        // Another bridge of the process: its DDS reader would take this
        // one's samples back.
        let mut other = new_bridge("one_way_other", &bus.scope("other"));
        refused(other.from_ros::<Text>(name, QosProfile::default()), Direction::ToRos, Direction::FromRos);

        // Refused by the QoS or the bus: the name stays free.
        let free = "/bridge_test/one_way_free";
        assert!(matches!(other.to_ros::<Text>(free, QosProfile::default().keep_all()), Err(BridgeError::UnsupportedQos { .. })));
        assert!(matches!(other.to_ros::<Text>("one_way_relative", QosProfile::default()), Err(BridgeError::Bus(BusError::InvalidName { .. }))));
        let _header = bus.scope("module").publisher::<Header>(free, Qos::default()).unwrap();
        assert!(matches!(other.to_ros::<Text>(free, QosProfile::default()), Err(BridgeError::Bus(BusError::TypeMismatch { .. }))));
        assert!(other.stats().topic(free).is_none());
        other.to_ros::<Header>(free, QosProfile::default()).unwrap();

        // A bridge that goes gives its names back.
        drop(bridge);
        other.from_ros::<Text>(name, QosProfile::default()).unwrap();
        other.service_from_ros::<SetBool::Service>(name).unwrap();
    });
}

// ---------------------------------------------------------------------------
// Services
// ---------------------------------------------------------------------------

fn set_bool(data: bool) -> SetBool::Request {
    SetBool::Request { data }
}

async fn wait_for_service(client: &r2r::Client<SetBool::Service>) {
    let available = r2r::Node::is_available(client).unwrap();
    tokio::time::timeout(WAIT, available).await.expect("timed out waiting for the service").unwrap();
}

/// A bus server that answers every call with what it was asked.
fn serve_on_bus(scope: &Scope, name: &str) -> tokio::task::JoinHandle<()> {
    let mut service = scope.service::<SetBool::Request, SetBool::Response>(name).unwrap();
    tokio::spawn(async move {
        while let Some(request) = service.next().await {
            let data = request.message.data;
            let _ = request.respond(SetBool::Response { success: data, message: format!("bus got {data}") });
        }
    })
}

#[test]
fn service_to_ros_answers_dds_clients_from_the_bus_server() {
    within(async {
        let name = "/bridge_test/save_map";
        let bus = Bus::new();
        let mut bridge = new_bridge("save_map_bridge", &bus.scope("bridge"));
        bridge.service_to_ros::<SetBool::Service>(name).unwrap();
        let running = start(bridge);
        let _server = serve_on_bus(&bus.scope("map_manage"), name);
        let app = Peer::new("save_map_app");
        let client = app.with(|n| n.create_client::<SetBool::Service>(name, QosProfile::services_default())).unwrap();
        wait_for_service(&client).await;
        // concurrent calls each get their own answer
        let calls = [true, false, true].map(|data| client.request(&set_bool(data)).unwrap());
        let answers = tokio::time::timeout(WAIT, futures::future::join_all(calls)).await.unwrap();
        let answers: Vec<String> = answers.into_iter().map(|a| a.unwrap().message).collect();
        assert_eq!(answers, ["bus got true", "bus got false", "bus got true"]);
        eventually("the three counted", || running.service(name).forwarded == 3).await;
        assert_eq!(running.service(name).unanswered, 0);
        running.stop().await.unwrap();
    });
}

#[test]
fn service_to_ros_leaves_a_dds_call_unanswered_when_the_bus_call_fails() {
    within(async {
        let name = "/bridge_test/start_recording";
        let bus = Bus::new();
        let mut bridge = new_bridge("start_recording_bridge", &bus.scope("bridge"));
        bridge.service_to_ros::<SetBool::Service>(name).unwrap();
        let running = start(bridge);
        let app = Peer::new("start_recording_app");
        let client = app.with(|n| n.create_client::<SetBool::Service>(name, QosProfile::services_default())).unwrap();
        wait_for_service(&client).await;

        // No bus server: ServiceUnavailable on the bus, no answer on DDS.
        let mut unavailable = Box::pin(client.request(&set_bool(true)).unwrap());
        eventually("the call to be given up", || running.service(name).unanswered == 1).await;
        // A bus server that drops its requests: NoResponse, no answer.
        let module = bus.scope("recorder");
        let mut dropping = module.service::<SetBool::Request, SetBool::Response>(name).unwrap();
        let dropper = tokio::spawn(async move { while let Some(request) = dropping.next().await { drop(request) } });
        let mut dropped = Box::pin(client.request(&set_bool(false)).unwrap());
        eventually("the call to be given up", || running.service(name).unanswered == 2).await;
        // Neither will ever be answered: the bridge has let go of them.
        tokio::time::sleep(Duration::from_millis(200)).await;
        assert!((&mut unavailable).now_or_never().is_none());
        assert!((&mut dropped).now_or_never().is_none());

        // The forwarder carries on once the bus server answers.
        dropper.abort();
        let _ = dropper.await;
        let _server = serve_on_bus(&module, name);
        let answer = tokio::time::timeout(WAIT, client.request(&set_bool(true)).unwrap()).await.unwrap().unwrap();
        assert_eq!(answer.message, "bus got true");
        eventually("the answer counted", || running.service(name).forwarded == 1).await;
        assert_eq!(running.service(name).unanswered, 2);
        running.stop().await.unwrap();
    });
}

/// A DDS server on `peer` that answers every call with what it was asked.
fn serve_on_dds(peer: &Peer, name: &str) -> tokio::task::JoinHandle<()> {
    let mut requests = peer.with(|n| n.create_service::<SetBool::Service>(name, QosProfile::services_default())).unwrap();
    tokio::spawn(async move {
        while let Some(request) = requests.next().await {
            let data = request.message.data;
            let _ = request.respond(SetBool::Response { success: data, message: format!("dds got {data}") });
        }
    })
}

#[test]
fn service_from_ros_calls_the_dds_server_for_bus_clients() {
    within(async {
        let name = "/bridge_test/get_state";
        let nav2 = Peer::new("get_state_nav2");
        let _server = serve_on_dds(&nav2, name);
        let bus = Bus::new();
        let mut bridge = new_bridge("get_state_bridge", &bus.scope("bridge"));
        bridge.service_from_ros::<SetBool::Service>(name).unwrap();
        let running = start(bridge);
        let client = bus.scope("nav_action_server").client::<SetBool::Request, SetBool::Response>(name).unwrap();
        // Discovery may still be under way: the first calls may come back
        // NoResponse, never anything else.
        let deadline = Instant::now() + WAIT;
        let answer = loop {
            match client.request(&set_bool(true)).unwrap().await {
                Ok(answer) => break answer,
                Err(BusError::NoResponse { .. }) => assert!(Instant::now() < deadline, "the DDS server never matched"),
                Err(e) => panic!("{e}"),
            }
        };
        assert_eq!(answer.message, "dds got true");
        let calls = [false, true].map(|data| client.request(&set_bool(data)).unwrap());
        let answers = tokio::time::timeout(WAIT, futures::future::join_all(calls)).await.unwrap();
        let answers: Vec<String> = answers.into_iter().map(|a| a.unwrap().message).collect();
        assert_eq!(answers, ["dds got false", "dds got true"]);
        running.stop().await.unwrap();
    });
}

#[test]
fn service_from_ros_gives_no_response_while_no_dds_server_is_matched_or_once_it_goes() {
    within(async {
        let name = "/bridge_test/navigate";
        let bus = Bus::new();
        let mut bridge = new_bridge("navigate_bridge", &bus.scope("bridge"));
        bridge.service_from_ros::<SetBool::Service>(name).unwrap();
        let running = start(bridge);
        let client = bus.scope("nav_action_server").client::<SetBool::Request, SetBool::Response>(name).unwrap();
        let started = Instant::now();
        let answer = tokio::time::timeout(WAIT, client.request(&set_bool(true)).unwrap()).await.unwrap();
        assert_eq!(answer, Err(BusError::NoResponse { name: name.into() }));
        assert!(started.elapsed() < Duration::from_secs(5), "given up within the probe window: {:?}", started.elapsed());
        assert_eq!(running.service(name).unanswered, 1);

        // A server that takes the call and then goes away with its process.
        let nav2 = Peer::new("navigate_nav2");
        let mut requests = nav2.with(|n| n.create_service::<SetBool::Service>(name, QosProfile::services_default())).unwrap();
        let (taken, held) = tokio::sync::oneshot::channel();
        let holder = tokio::spawn(async move {
            if let Some(request) = requests.next().await {
                let _ = taken.send(());
                // Never answered; dropped when the stream ends with the node.
                let _request = request;
                while requests.next().await.is_some() {}
            }
        });
        let deadline = Instant::now() + WAIT;
        let pending = loop {
            let call = client.request(&set_bool(true)).unwrap();
            let mut call = Box::pin(call);
            // Until the server is matched, calls come back NoResponse.
            match tokio::time::timeout(Duration::from_secs(2), &mut call).await {
                Ok(Err(BusError::NoResponse { .. })) => assert!(Instant::now() < deadline, "the DDS server never matched"),
                Ok(other) => panic!("a held call cannot be answered: {other:?}"),
                Err(_) => break call,
            }
        };
        tokio::time::timeout(WAIT, held).await.unwrap().unwrap();
        drop(nav2);
        let answer = tokio::time::timeout(WAIT, pending).await.unwrap();
        assert_eq!(answer, Err(BusError::NoResponse { name: name.into() }));
        let _ = tokio::time::timeout(WAIT, holder).await;
        running.stop().await.unwrap();
    });
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

/// A bridge with one name of each kind, a bus module on the other end of
/// each, and a peer on DDS.
struct Everything {
    bus: Bus,
    bridge_scope: Scope,
    peer: Peer,
    names: [String; 4],
    /// The module's latched writer (to_ros), its reader (from_ros) and its
    /// server (service_to_ros).
    publisher: mower_bus::Publisher<Text>,
    reader: mower_bus::Subscriber<Text>,
    _server: tokio::task::JoinHandle<()>,
    /// The peer's writer on the from_ros name, which counts the bridge's
    /// DDS reader.
    dds_writer: r2r::Publisher<Text>,
}

fn everything(tag: &str) -> (Everything, RosBridge) {
    let names = ["to_ros", "from_ros", "service_to_ros", "service_from_ros"].map(|d| format!("/bridge_test/{tag}_{d}"));
    let bus = Bus::new();
    let module = bus.scope("module");
    let publisher = module.publisher::<Text>(&names[0], Qos::latched()).unwrap();
    publisher.publish(&text("latched")).unwrap();
    let reader = module.subscribe::<Text>(&names[1], Qos::default()).unwrap();
    let server = serve_on_bus(&module, &names[2]);
    let bridge_scope = bus.scope("bridge");
    let mut bridge = new_bridge(&format!("{tag}_bridge"), &bridge_scope);
    bridge.to_ros::<Text>(&names[0], latched()).unwrap();
    bridge.from_ros::<Text>(&names[1], QosProfile::default()).unwrap();
    bridge.service_to_ros::<SetBool::Service>(&names[2]).unwrap();
    bridge.service_from_ros::<SetBool::Service>(&names[3]).unwrap();
    let peer = Peer::new(&format!("{tag}_peer"));
    let dds_writer = peer.with(|n| n.create_publisher::<Text>(&names[1], QosProfile::default())).unwrap();
    (Everything { bus, bridge_scope, peer, names, publisher, reader, _server: server, dds_writer }, bridge)
}

impl Everything {
    async fn assert_up(&self, running: &Running) {
        eventually("the DDS writer", || self.peer.writers(&self.names[0]).len() == 1).await;
        eventually("the latched value on DDS", || running.topic(&self.names[0]).forwarded == 1).await;
        eventually("the bridge's DDS reader", || self.dds_writer.get_inter_process_subscription_count().unwrap() == 1).await;
        assert_eq!(self.publisher.subscription_count(), 1, "the bridge reads the module's topic");
        assert_eq!(self.reader.publisher_count(), 1, "the bridge writes the module's topic");
    }

    /// Everything the bridge had is gone, on DDS and on the bus, and its
    /// names are free again.
    async fn assert_gone(&self, tag: &str) {
        eventually("the DDS writer to go with the node", || self.peer.writers(&self.names[0]).is_empty()).await;
        eventually("the bridge's DDS reader to go with the node", || {
            self.dds_writer.get_inter_process_subscription_count().unwrap() == 0
        })
        .await;
        assert_eq!(self.publisher.subscription_count(), 0, "the bridge's bus reader is gone");
        assert_eq!(self.reader.publisher_count(), 0, "the bridge's bus writer is gone");
        let client = self.bus.scope("caller").client::<SetBool::Request, SetBool::Response>(&self.names[3]).unwrap();
        assert!(!client.is_available(), "the bridge's bus server is gone");
        let _again = self.bus.scope("other").service::<SetBool::Request, SetBool::Response>(&self.names[3]).unwrap();
        let mut again = new_bridge(&format!("{tag}_again"), &self.bus.scope("again"));
        again.to_ros::<Text>(&self.names[0], latched()).unwrap();
        again.from_ros::<Text>(&self.names[1], QosProfile::default()).unwrap();
        again.service_to_ros::<SetBool::Service>(&self.names[2]).unwrap();
    }
}

#[test]
fn run_returns_on_shutdown_once_its_endpoints_and_its_spin_thread_are_gone() {
    within(async {
        let (everything, bridge) = everything("shutdown");
        let running = start(bridge);
        everything.assert_up(&running).await;
        let stopping = Instant::now();
        running.stop().await.unwrap();
        assert!(stopping.elapsed() < Duration::from_secs(2), "stopped in {:?}", stopping.elapsed());
        assert!(!everything.bridge_scope.is_closed(), "the scope is the module's to close");
        everything.assert_gone("shutdown").await;
    });
}

#[test]
fn run_returns_when_its_scope_closes() {
    within(async {
        let (everything, bridge) = everything("scope_closed");
        let running = start(bridge);
        everything.assert_up(&running).await;
        everything.bridge_scope.close();
        running.ended().await.unwrap();
        everything.assert_gone("scope_closed").await;
    });
}

// ---------------------------------------------------------------------------
// JSON parity (E3 prep)
// ---------------------------------------------------------------------------

/// What the app gets today and what it will get from the bus. ws_bridge
/// takes JSON from r2r's untyped subscription (mower_ws_bridge/src/hub.rs);
/// E3 takes it from the bus's erased subscription, which is
/// `serde_json::to_value` of the typed message. Each message goes bus ->
/// bridge -> DDS, so the DDS side carries exactly what the bus did.
#[test]
fn r2r_untyped_json_equals_the_bus_s_erased_json() {
    within(async {
        let bus = Bus::new();
        let module = bus.scope("module");
        let mut bridge = new_bridge("json_bridge", &bus.scope("bridge"));
        let peer = Peer::new("json_ws_bridge");
        let checks = vec![
            parity(&module, &mut bridge, &peer, "/bridge_test/json_header", header()),
            parity(&module, &mut bridge, &peer, "/bridge_test/json_fix", nav_sat_fix()),
            parity(&module, &mut bridge, &peer, "/bridge_test/json_grid", occupancy_grid()),
            parity(&module, &mut bridge, &peer, "/bridge_test/json_progress", coverage_progress()),
        ];
        let running = start(bridge);
        let mut differences = Vec::new();
        for check in checks {
            differences.extend(check.await);
        }
        assert!(differences.is_empty(), "the app would see these differ:\n{}", differences.join("\n"));
        running.stop().await.unwrap();
    });
}

/// Declare `name` on the bridge now; the returned check runs once the bridge
/// does, and lists what differs.
fn parity<T: BridgeMessage>(
    module: &Scope, bridge: &mut RosBridge, peer: &Peer, name: &str, msg: T,
) -> std::pin::Pin<Box<dyn Future<Output = Vec<String>>>> {
    let ros_type = mower_bus::ros_type_name(std::any::type_name::<T>()).expect("an r2r message type");
    let publisher = module.publisher::<T>(name, Qos::latched()).unwrap();
    publisher.publish(&msg).unwrap();
    bridge.to_ros::<T>(name, latched()).unwrap();
    let mut erased = module.subscribe_erased(name, Qos::latched()).unwrap();
    let mut untyped = peer.with(|n| n.subscribe_untyped(name, &ros_type, latched())).unwrap();
    let topic_types = peer.node.clone();
    let name = name.to_string();
    Box::pin(async move {
        let _publisher = publisher;
        let typed = serde_json::to_value(&msg).unwrap();
        let from_bus = erased.try_recv().expect("retained").to_json().unwrap();
        let from_dds = next(&format!("{name} on DDS"), &mut untyped).await.unwrap();
        let mut differences = Vec::new();
        for (what, value) in [("serde_json::to_value", &typed), ("the bus's erased JSON", &from_bus)] {
            diff(&format!("{name}: r2r untyped vs {what}"), &from_dds, value, &mut differences);
            // what ws_bridge sends is the text of the value
            let (dds_text, other_text) = (from_dds.to_string(), value.to_string());
            if dds_text != other_text {
                differences.push(format!("{name}: r2r untyped vs {what}: the text differs"));
            }
        }
        // /rosapi/topics (E3) names the type the way the DDS graph does. The
        // graph learns of the writer after the match, so it is waited for.
        let graph_types = || topic_types.lock().unwrap().get_topic_names_and_types().unwrap().remove(&name);
        eventually("the graph to show the topic", || graph_types().is_some()).await;
        if graph_types() != Some(vec![ros_type.clone()]) {
            differences.push(format!("{name}: the graph says {:?}, ros_type_name {ros_type}", graph_types()));
        }
        differences
    })
}

/// Every JSON pointer at which `a` and `b` differ.
fn diff(at: &str, a: &serde_json::Value, b: &serde_json::Value, out: &mut Vec<String>) {
    use serde_json::Value;
    match (a, b) {
        (Value::Object(a), Value::Object(b)) => {
            for key in a.keys().chain(b.keys().filter(|k| !a.contains_key(*k))) {
                match (a.get(key), b.get(key)) {
                    (Some(x), Some(y)) => diff(&format!("{at}/{key}"), x, y, out),
                    (x, y) => out.push(format!("{at}/{key}: {x:?} vs {y:?}")),
                }
            }
        }
        (Value::Array(x), Value::Array(y)) if x.len() == y.len() => {
            for (i, (x, y)) in x.iter().zip(y).enumerate() {
                diff(&format!("{at}/{i}"), x, y, out);
            }
        }
        (x, y) if x != y => out.push(format!("{at}: {x} vs {y}")),
        _ => {}
    }
}

fn header() -> Header {
    Header { stamp: Time { sec: 1_761_400_000, nanosec: 123_456_789 }, frame_id: "割草機/base_link".into() }
}

fn nav_sat_fix() -> r2r::sensor_msgs::msg::NavSatFix {
    let mut fix = r2r::sensor_msgs::msg::NavSatFix { header: header(), ..Default::default() };
    fix.status.status = -1; // STATUS_NO_FIX, a negative i8
    fix.status.service = 1 | 8; // GPS | GALILEO, a u16 bit set
    fix.latitude = 23.695_012_345_678_9;
    fix.longitude = 120.538_765_432_1;
    // "unknown" altitude, as receivers report it: JSON has no NaN
    fix.altitude = f64::NAN;
    fix.position_covariance = vec![0.000_4, 0.0, 0.0, 0.0, 0.000_4, 0.0, 0.0, 0.0, 1e-9];
    fix.position_covariance_type = 2;
    fix
}

fn occupancy_grid() -> r2r::nav_msgs::msg::OccupancyGrid {
    let mut grid = r2r::nav_msgs::msg::OccupancyGrid { header: header(), ..Default::default() };
    grid.info.resolution = 0.05; // f32
    grid.info.width = 4;
    grid.info.height = 3;
    grid.info.origin.position.x = -10.25;
    grid.info.origin.orientation.w = 1.0;
    grid.data = vec![-1, 0, 100, 50, -1, -1, 0, 0, 100, 100, 1, 99];
    grid
}

fn coverage_progress() -> r2r::mower_interface::msg::CoverageProgress {
    let mut progress = r2r::mower_interface::msg::CoverageProgress { header: header(), ..Default::default() };
    progress.status = 2; // STATUS_RUNNING, a u8
    progress.status_text = "running 割草中".into();
    progress.mission_id = "zone-3/2026-10-26".into();
    progress.zone_id = -1;
    progress.current_segment_index = 2;
    progress.total_segments = 7;
    progress.completed_segments = 1;
    progress.current_segment_progress = 0.1; // f32 fields from here
    progress.overall_progress = 0.333_333_34;
    progress.current_segment_distance_m = 12.7;
    progress.completed_distance_m = 101.0;
    progress.total_distance_m = 420.5;
    progress.remaining_distance_m = 319.5;
    progress.current_pose.pose.position.x = 1.5;
    progress.checkpoint_available = true;
    progress.message = "line 1\nline \"2\"".into();
    progress
}

/// For E3: the app's bytes are those of a `serde_json::Value`, which ws_bridge
/// builds today and the bus's erased JSON builds too. Serializing the typed
/// message straight to text is not the same: it keeps the declaration order
/// of the fields, where a `Value` sorts them, and prints an f32 as an f32,
/// where a `Value` holds it as the f64 it widens to.
#[test]
fn the_app_s_bytes_are_those_of_a_json_value_not_of_the_typed_message() {
    let progress = coverage_progress();
    let via_value = serde_json::to_value(&progress).unwrap().to_string();
    let direct = serde_json::to_string(&progress).unwrap();
    assert_ne!(via_value, direct);
    assert!(via_value.contains("\"current_segment_progress\":0.10000000149011612"), "{via_value}");
    assert!(direct.contains("\"current_segment_progress\":0.1,"), "{direct}");
    assert!(via_value.starts_with("{\"checkpoint_available\":true,"), "keys sorted: {via_value}");
    assert!(direct.starts_with("{\"header\":"), "declaration order: {direct}");
    // NaN is `null` either way, so the app cannot tell it from a missing value
    let fix = serde_json::to_value(nav_sat_fix()).unwrap();
    assert_eq!(fix["altitude"], serde_json::Value::Null);
}
