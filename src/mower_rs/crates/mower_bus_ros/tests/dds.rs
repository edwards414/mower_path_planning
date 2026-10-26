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
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use futures::{FutureExt, Stream, StreamExt};
use mower_bus::{Bus, BusError, Qos, Scope};
use mower_bus_ros::{BridgeError, BridgeMessage, BridgeStats, Direction, EntryStats, RosBridge, ServiceTimeouts};
use r2r::builtin_interfaces::msg::Time;
use r2r::mower_interface::action::Waypoint;
use r2r::nav_msgs::srv::SetMap;
use r2r::robot_localization::srv::GetState;
use r2r::sensor_msgs::msg::NavSatFix;
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

/// Like `within`, on one thread: a spawned task (the bridge's forwarders)
/// runs only when the test awaits, so two bus events with no await between
/// them reach a forwarder together.
fn on_one_thread(test: impl Future<Output = ()>) {
    let runtime = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
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

    /// How many readers of `topic` the graph shows this node.
    fn readers(&self, topic: &str) -> usize {
        self.with(|node| node.get_subscriptions_info_by_topic(topic, false).unwrap()).len()
    }

    fn writer_gids(&self, topic: &str) -> Vec<Vec<u8>> {
        self.with(|node| node.get_publishers_info_by_topic(topic, false).unwrap())
            .into_iter()
            .map(|info| info.endpoint_gid.to_vec())
            .collect()
    }

    /// What the servers of service `name` have on the graph: their reply
    /// writers (`rr<name>Reply`) and request readers (`rq<name>Request`). A
    /// client has the other two.
    fn service_servers(&self, name: &str) -> (usize, usize) {
        self.with(|node| {
            let writers = node.get_publishers_info_by_topic(&format!("rr{name}Reply"), true).unwrap();
            let readers = node.get_subscriptions_info_by_topic(&format!("rq{name}Request"), true).unwrap();
            (writers.len(), readers.len())
        })
    }

    /// Every endpoint the servers of action `name` have on the graph: two
    /// for each of its three services, its feedback and status writers.
    fn action_server_endpoints(&self, name: &str) -> usize {
        let services = ["send_goal", "cancel_goal", "get_result"].map(|s| self.service_servers(&format!("{name}/_action/{s}")));
        let topics = ["feedback", "status"].map(|t| self.writers(&format!("{name}/_action/{t}")).len());
        services.iter().map(|(w, r)| w + r).sum::<usize>() + topics.iter().sum::<usize>()
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

/// The vendored r2r's `destroy_service` (vendor/r2r/VENDORED.md), which
/// `service_to_ros` uses on its spin thread: on a node that spins, a service
/// leaves the graph as it does with its node, and one made again under its
/// name works. A request held across the destroy answers with an error.
#[test]
fn r2r_destroys_a_service_on_a_spinning_node_and_makes_it_again_under_its_name() {
    within(async {
        let name = "/bridge_test/r2r_destroy_service";
        let server = Peer::new("r2r_destroy_service_server");
        let app = Peer::new("r2r_destroy_service_app");
        let client = app.with(|n| n.create_client::<SetBool::Service>(name, QosProfile::services_default())).unwrap();
        for round in 0..3 {
            let mut requests = server.with(|n| n.create_service::<SetBool::Service>(name, QosProfile::services_default())).unwrap();
            eventually("the service on the graph", || app.service_servers(name) == (1, 1)).await;
            wait_for_service(&client).await;
            let call = client.request(&set_bool(true)).unwrap();
            let message = format!("round {round}");
            next("a request", &mut requests).await.respond(SetBool::Response { success: true, message: message.clone() }).unwrap();
            assert_eq!(tokio::time::timeout(WAIT, call).await.unwrap().unwrap().message, message);

            let _unanswered = client.request(&set_bool(false)).unwrap();
            let held = next("a request to hold", &mut requests).await;
            server.with(|n| n.destroy_service(requests));
            eventually("nothing of the service left on the graph", || app.service_servers(name) == (0, 0)).await;
            assert!(held.respond(SetBool::Response::default()).is_err(), "round {round}: answered after the destroy");
        }
    });
}

/// The same for `destroy_action_server`, for `action_to_ros`: its three
/// services and two topics leave the graph, a goal request held across the
/// destroy answers `accept` with an error (upstream r2r panics there), and
/// so does a goal of the destroyed server.
#[test]
fn r2r_destroys_an_action_server_on_a_spinning_node_and_makes_it_again_under_its_name() {
    within(async {
        let name = "/bridge_test/r2r_destroy_action";
        let server = Peer::new("r2r_destroy_action_server");
        let app = Peer::new("r2r_destroy_action_app");
        let client = app.with(|n| n.create_action_client::<Waypoint::Action>(name)).unwrap();
        let goal = |id: &str| Waypoint::Goal { dispatch_id: id.into(), ..Default::default() };
        for round in 0..3 {
            let mut requests = server.with(|n| n.create_action_server::<Waypoint::Action>(name)).unwrap();
            eventually("the action server on the graph", || app.action_server_endpoints(name) == 8).await;
            let available = r2r::Node::is_available(&client).unwrap();
            tokio::time::timeout(WAIT, available).await.expect("timed out waiting for the action server").unwrap();

            // a goal that runs to its end
            let sent = client.send_goal_request(goal("done")).unwrap();
            let (mut done, _cancels) = next("a goal request", &mut requests).await.accept().unwrap();
            let (_goal, result, _feedback) = tokio::time::timeout(WAIT, sent).await.unwrap().unwrap();
            done.succeed(Waypoint::Result { success: true }).unwrap();
            let (status, result) = tokio::time::timeout(WAIT, result).await.unwrap().unwrap();
            assert_eq!((status, result.success), (r2r::GoalStatus::Succeeded, true), "round {round}");

            // a goal running and a goal request unanswered, held across the
            // destroy
            let sent = client.send_goal_request(goal("running")).unwrap();
            let (mut running, mut cancels) = next("a goal request", &mut requests).await.accept().unwrap();
            let _accepted = tokio::time::timeout(WAIT, sent).await.unwrap().unwrap();
            let _unanswered = client.send_goal_request(goal("held")).unwrap();
            let held = next("a goal request to hold", &mut requests).await;
            server.with(|n| n.destroy_action_server(requests));
            eventually("nothing of the action server left on the graph", || app.action_server_endpoints(name) == 0).await;
            assert!(held.accept().is_err(), "round {round}: accepted after the destroy");
            assert!(running.succeed(Waypoint::Result { success: true }).is_err(), "round {round}: ended after the destroy");
            assert!(running.is_cancelling().is_err());
            assert!(next_or_end(&mut cancels).await.is_none(), "round {round}: its cancel requests end");
        }
    });
}

/// The same for `destroy_subscription`, for `from_ros`'s DDS reader: it
/// leaves the graph and its writers at once, and one made again under its
/// name gets the latched value.
#[test]
fn r2r_destroys_a_subscription_on_a_spinning_node_and_makes_it_again_under_its_name() {
    within(async {
        let topic = "/bridge_test/r2r_destroy_subscription";
        let reader = Peer::new("r2r_destroy_subscription_reader");
        let base = Peer::new("r2r_destroy_subscription_base");
        let publisher = base.with(|n| n.create_publisher::<Text>(topic, latched())).unwrap();
        for round in 0..3 {
            publisher.publish(&text(round)).unwrap();
            let mut sub = reader.with(|n| n.subscribe::<Text>(topic, latched())).unwrap();
            assert_eq!(next("the latched value", &mut sub).await.data, round.to_string());
            eventually("the reader on the graph", || base.readers(topic) == 1).await;
            reader.with(|n| n.destroy_subscription(sub));
            eventually("nothing of the reader left on the graph", || base.readers(topic) == 0).await;
            assert_eq!(publisher.get_inter_process_subscription_count().unwrap(), 0, "round {round}");
        }
    });
}

/// The next item of `stream`, or `None` once it ends.
async fn next_or_end<S: Stream + Unpin>(stream: &mut S) -> Option<S::Item> {
    tokio::time::timeout(WAIT, stream.next()).await.expect("timed out waiting for the stream")
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

/// A forwarder that falls more than its reader's depth (64) behind loses
/// messages on the bus side; the stats say how many. On one thread nothing
/// runs between the publishes, so all 100 reach the reader's queue before
/// the forwarder takes any.
#[test]
fn a_forwarder_that_falls_behind_counts_what_its_reader_lost() {
    on_one_thread(async {
        let topic = "/bridge_test/fell_behind";
        let bus = Bus::new();
        let module = bus.scope("mower_base");
        let publisher = module.publisher::<Text>(topic, Qos::default()).unwrap();
        let mut bridge = new_bridge("fell_behind_bridge", &bus.scope("bridge"));
        bridge.to_ros::<Text>(topic, QosProfile::default()).unwrap();
        let running = start(bridge);
        eventually("the DDS writer", || running.topic(topic).dds_writer).await;
        for i in 0..100 {
            publisher.publish(&text(i)).unwrap();
        }
        eventually("the 64 that fitted handled", || running.topic(topic).skipped == 64).await;
        assert_eq!(running.topic(topic).dropped, 36);
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

/// The DDS writer holds v1. The module publishes v2 and, before the forwarder
/// has handled it, some other writer comes on the name (a ws_bridge
/// session's `publisher_json`). The change is taken first, and the fresh
/// reader's replay is [v2]: v1 is gone from the bus, but only because v2,
/// still queued, superseded it. Re-creating the DDS writer for that would
/// make nav2's StaticLayer see its publisher leave and come back, and a
/// volatile DDS reader miss v2.
#[test]
fn an_unrelated_writer_change_while_a_latched_message_is_queued_keeps_the_dds_writer() {
    on_one_thread(async {
        let topic = "/bridge_test/queued_at_a_change";
        let bus = Bus::new();
        let module = bus.scope("map_manage");
        let publisher = module.publisher::<Text>(topic, Qos::latched()).unwrap();
        let mut bridge = new_bridge("queued_at_a_change_bridge", &bus.scope("bridge"));
        bridge.to_ros::<Text>(topic, latched()).unwrap();
        let running = start(bridge);
        publisher.publish(&text("v1")).unwrap();
        eventually("v1 on DDS", || running.topic(topic).forwarded == 1).await;
        let nav2 = Peer::new("queued_at_a_change_nav2");
        eventually("the bridge's DDS writer on the graph", || nav2.writer_gids(topic).len() == 1).await;
        let writer = nav2.writer_gids(topic);
        let resyncs = running.topic(topic).resyncs;

        // No await between the two: the forwarder finds both at once.
        publisher.publish(&text("v2")).unwrap();
        let _session = bus.scope("ws_bridge_session").publisher_json(topic, Qos::default()).unwrap();

        eventually("the change handled and v2 on DDS", || {
            let stats = running.topic(topic);
            stats.resyncs > resyncs && stats.forwarded >= 2
        })
        .await;
        // Long enough for the graph to show a replacement, had there been one.
        // The same writer, given v2: a matched volatile reader got it too.
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline {
            let now = nav2.writer_gids(topic);
            assert!(now == writer, "the DDS writer was replaced for an unrelated writer change: {now:?} (was {writer:?})");
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        running.stop().await.unwrap();
    });
}

/// The same writer change once v2 has been forwarded keeps the DDS writer
/// too, as it always did: the case above differs only by v2 being queued.
#[test]
fn a_writer_change_after_the_message_is_forwarded_keeps_the_dds_writer() {
    on_one_thread(async {
        let topic = "/bridge_test/change_after_forwarding";
        let bus = Bus::new();
        let module = bus.scope("map_manage");
        let publisher = module.publisher::<Text>(topic, Qos::latched()).unwrap();
        let mut bridge = new_bridge("change_after_forwarding_bridge", &bus.scope("bridge"));
        bridge.to_ros::<Text>(topic, latched()).unwrap();
        let running = start(bridge);
        publisher.publish(&text("v1")).unwrap();
        eventually("v1 on DDS", || running.topic(topic).forwarded == 1).await;
        let nav2 = Peer::new("change_after_forwarding_nav2");
        eventually("the bridge's DDS writer on the graph", || nav2.writer_gids(topic).len() == 1).await;
        let writer = nav2.writer_gids(topic);
        let resyncs = running.topic(topic).resyncs;

        publisher.publish(&text("v2")).unwrap();
        eventually("v2 on DDS", || running.topic(topic).forwarded == 2).await;
        let _session = bus.scope("ws_bridge_session").publisher_json(topic, Qos::default()).unwrap();
        eventually("the change handled", || running.topic(topic).resyncs > resyncs).await;
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline {
            assert_eq!(nav2.writer_gids(topic), writer, "replaced");
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
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

/// A latched DDS value published before the bridge exists and before any
/// module reads it: the first bus reader makes the bridge's DDS reader, to
/// which the DDS writer replays its history, as it did to the module's own
/// r2r reader (mower_rsd starts the bridge first, and its modules subscribe
/// as they start). The bridge's bus writer retains it, so the next bus late
/// joiner has it at once.
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

        let localize = bus.scope("ekf_filter_node_map");
        let mut first = localize.subscribe::<Text>(topic, Qos::latched()).unwrap();
        assert_eq!(tokio::time::timeout(WAIT, first.recv()).await.unwrap().unwrap().data, "imu_link");
        assert!(running.topic(topic).dds_reader);
        let mut late = bus.scope("navsat").subscribe::<Text>(topic, Qos::latched()).unwrap();
        assert_eq!(late.try_recv().unwrap().data, "imu_link");
        publisher.publish(&text("gps_link")).unwrap();
        assert_eq!(tokio::time::timeout(WAIT, late.recv()).await.unwrap().unwrap().data, "gps_link");
        running.stop().await.unwrap();
    });
}

/// The bridge's DDS reader exists while the bus has a reader of the name, as
/// a module's own r2r reader existed while it subscribed: nav's `/rosout`
/// reader with nav_core, or its nav_core inputs in a standalone nav, leave no
/// DDS reader behind that every writer of the name sends to for nothing.
#[test]
fn from_ros_reads_dds_only_while_the_bus_has_a_reader() {
    within(async {
        let topic = "/bridge_test/rosout_like";
        // read all along: once its DDS reader is on the graph, the bridge
        // runs, and the other name's absence means something
        let witness = "/bridge_test/rosout_like_witness";
        let bus = Bus::new();
        let nav = bus.scope("nav");
        let _witness_reader = nav.subscribe::<Text>(witness, Qos::default()).unwrap();
        let mut bridge = new_bridge("rosout_like_bridge", &bus.scope("bridge"));
        bridge.from_ros::<Text>(topic, QosProfile::default()).unwrap();
        bridge.from_ros::<Text>(witness, QosProfile::default()).unwrap();
        let running = start(bridge);
        let nav2 = Peer::new("rosout_like_nav2");
        let writer = nav2.with(|n| n.create_publisher::<Text>(topic, QosProfile::default())).unwrap();
        eventually("the witness's DDS reader", || nav2.readers(witness) == 1).await;
        assert_eq!(nav2.readers(topic), 0);
        assert_eq!(writer.get_inter_process_subscription_count().unwrap(), 0);
        assert!(!running.topic(topic).dds_reader);

        for round in 0..3 {
            // A bus reader comes: so does the DDS reader, and samples reach it.
            // (The stats say a DDS endpoint exists once it does: the graph
            // can show it first.)
            let mut reader = nav.subscribe::<Text>(topic, Qos::default()).unwrap();
            eventually("the DDS reader", || running.topic(topic).dds_reader).await;
            eventually("its match", || writer.get_inter_process_subscription_count().unwrap() == 1).await;
            let deadline = Instant::now() + WAIT;
            loop {
                writer.publish(&text(round)).unwrap();
                if let Ok(Some(msg)) = tokio::time::timeout(Duration::from_millis(50), reader.recv()).await {
                    assert_eq!(msg.data, round.to_string());
                    break;
                }
                assert!(Instant::now() < deadline, "round {round}: nothing reached the bus reader");
            }
            // It goes: so does the DDS reader.
            drop(reader);
            eventually("the DDS reader to leave the graph", || nav2.readers(topic) == 0).await;
            assert!(!running.topic(topic).dds_reader);
            assert_eq!(writer.get_inter_process_subscription_count().unwrap(), 0);
        }
        running.stop().await.unwrap();
    });
}

/// The last bus reader goes, and the DDS writer publishes after it: the
/// bridge did not hear that. A bus reader that comes next must get what DDS
/// has now, as an r2r reader that subscribed then did, not what the bridge
/// heard before: the transient-local bus writer goes with the DDS reader.
/// The bus readers keep 2, so a stale value would come before the new one.
#[test]
fn a_latched_value_heard_before_the_last_bus_reader_went_does_not_reach_the_next() {
    within(async {
        let topic = "/bridge_test/firmware_info";
        let bus = Bus::new();
        let mut bridge = new_bridge("firmware_info_bridge", &bus.scope("bridge"));
        bridge.from_ros::<Text>(topic, latched()).unwrap();
        let running = start(bridge);
        let base = Peer::new("firmware_info_base");
        let writer = base.with(|n| n.create_publisher::<Text>(topic, latched())).unwrap();
        writer.publish(&text("t0")).unwrap();
        let status = bus.scope("robot_status");
        let mut first = status.subscribe::<Text>(topic, Qos::latched().keep_last(2)).unwrap();
        assert_eq!(tokio::time::timeout(WAIT, first.recv()).await.unwrap().unwrap().data, "t0");

        drop(first);
        eventually("the DDS reader to go with the last bus reader", || base.readers(topic) == 0).await;
        assert!(!running.topic(topic).bus_writer, "the bus writer stayed, and with it t0");
        writer.publish(&text("t1")).unwrap();
        let mut next = status.subscribe::<Text>(topic, Qos::latched().keep_last(2)).unwrap();
        assert_eq!(tokio::time::timeout(WAIT, next.recv()).await.unwrap().unwrap().data, "t1", "a stale value first");
        assert!(running.topic(topic).dds_reader);
        running.stop().await.unwrap();
    });
}

/// README.md, "Graph presence": a transient-local `from_ros` bus writer
/// exists while DDS has a writer, so the history a bus late joiner gets goes
/// with the DDS writer, as it went for the module's own r2r reader: coverage
/// crashes, and the adapter that restarts after it must not relay the dead
/// instance's path to the app. Depth 2: a stale value would reach a late
/// joiner before the fresh one.
#[test]
fn a_latched_dds_value_leaves_the_bus_with_its_writer_and_a_new_writer_s_comes_alone() {
    within(async {
        let topic = "/bridge_test/coverage_path_markers";
        let bus = Bus::new();
        let mut bridge = new_bridge("path_markers_bridge", &bus.scope("bridge"));
        bridge.from_ros::<Text>(topic, latched_2()).unwrap();
        let running = start(bridge);
        let adapter = bus.scope("flutter_adapter");
        let mut live = adapter.subscribe::<Text>(topic, Qos::latched().keep_last(10)).unwrap();
        eventually("the bridge to look at the graph", || running.topic(topic).resyncs >= 2).await;
        assert!(!running.topic(topic).bus_writer, "no DDS writer, no bus writer");
        assert_eq!(live.publisher_count(), 0);

        // coverage, still on r2r, publishes its path
        let coverage = Peer::new("path_markers_coverage");
        let publisher = coverage.with(|n| n.create_publisher::<Text>(topic, latched_2())).unwrap();
        publisher.publish(&text("old path")).unwrap();
        assert_eq!(tokio::time::timeout(WAIT, live.recv()).await.unwrap().unwrap().data, "old path");
        assert!(running.topic(topic).bus_writer);
        let mut late = adapter.subscribe::<Text>(topic, Qos::latched().keep_last(2)).unwrap();
        assert_eq!(late.try_recv().unwrap().data, "old path");

        // coverage dies: its path leaves the bus with its writer
        drop(publisher);
        drop(coverage);
        eventually("the bridge to drop its bus writer", || !running.topic(topic).bus_writer).await;
        assert_eq!(live.publisher_count(), 0);
        let mut fresh = adapter.subscribe::<Text>(topic, Qos::latched().keep_last(2)).unwrap();
        assert!(fresh.try_recv().is_err(), "a bus late joiner got the dead writer's value");

        // coverage comes back: a late joiner gets the new path alone
        let restarted = Peer::new("path_markers_coverage");
        let publisher = restarted.with(|n| n.create_publisher::<Text>(topic, latched_2())).unwrap();
        publisher.publish(&text("new path")).unwrap();
        assert_eq!(tokio::time::timeout(WAIT, live.recv()).await.unwrap().unwrap().data, "new path");
        let mut later = adapter.subscribe::<Text>(topic, Qos::latched().keep_last(2)).unwrap();
        assert_eq!(later.try_recv().unwrap().data, "new path");
        assert!(later.try_recv().is_err(), "the dead writer's value came back");
        // counted once the publish has returned, which a reader may beat
        eventually("both counted", || running.topic(topic).forwarded == 2).await;
        running.stop().await.unwrap();
    });
}

/// docs/BUS_TOPICS.md 5.5 f: a volatile `from_ros` bus writer is a publisher
/// for as long as the bridge runs, whether or not DDS has one. It has no
/// history to keep, and no module asks for its presence.
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
        // A bus server that drops its requests: NoResponse, no answer.
        let module = bus.scope("recorder");
        let mut dropping = module.service::<SetBool::Request, SetBool::Response>(name).unwrap();
        let dropper = tokio::spawn(async move { while let Some(request) = dropping.next().await { drop(request) } });
        let app = Peer::new("start_recording_app");
        let client = app.with(|n| n.create_client::<SetBool::Service>(name, QosProfile::services_default())).unwrap();
        wait_for_service(&client).await;
        let mut dropped = Box::pin(client.request(&set_bool(false)).unwrap());
        eventually("the call to be given up", || running.service(name).unanswered == 1).await;
        // It will never be answered: the bridge has let go of it.
        tokio::time::sleep(Duration::from_millis(200)).await;
        assert!((&mut dropped).now_or_never().is_none());

        // The forwarder carries on once a bus server answers: the dropping
        // one goes, and its DDS server with it, then the answering one comes.
        // (Until the graph has lost the old DDS server, the client could
        // still call it.)
        dropper.abort();
        let _ = dropper.await;
        eventually("the DDS server to go with the bus server", || app.service_servers(name) == (0, 0)).await;
        let _server = serve_on_bus(&module, name);
        wait_for_service(&client).await;
        let answer = tokio::time::timeout(WAIT, client.request(&set_bool(true)).unwrap()).await.unwrap().unwrap();
        assert_eq!(answer.message, "bus got true");
        eventually("the answer counted", || running.service(name).forwarded == 1).await;
        assert_eq!(running.service(name).unanswered, 1);
        running.stop().await.unwrap();
    });
}

/// A DDS client's `is_available` does not complete within a second: what
/// ws_bridge (hub.rs, 5 s) turns into "service unavailable" for the app.
async fn assert_no_dds_server(client: &r2r::Client<SetBool::Service>, what: &str) {
    let available = r2r::Node::is_available(client).unwrap();
    assert!(tokio::time::timeout(Duration::from_secs(1), available).await.is_err(), "{what}: a DDS server is advertised");
}

/// The DDS server exists only while the bus has a server, as the module's
/// own r2r server lived and died with its node: a module that is down (or
/// in a crash loop) leaves no server that DDS clients call in vain, and
/// ws_bridge's 5 s availability check fails, where a call to an advertised
/// server would wait out its 30 s `service_timeout`.
#[test]
fn service_to_ros_serves_on_dds_only_while_the_bus_has_a_server() {
    within(async {
        let name = "/bridge_test/save_zone";
        // up all along: once it is on the graph, the bridge runs and the
        // graph is current, so the other name's absence means something
        let witness = "/bridge_test/save_zone_witness";
        let bus = Bus::new();
        let _witness_server = serve_on_bus(&bus.scope("record"), witness);
        let mut bridge = new_bridge("save_zone_bridge", &bus.scope("bridge"));
        bridge.service_to_ros::<SetBool::Service>(name).unwrap();
        bridge.service_to_ros::<SetBool::Service>(witness).unwrap();
        let running = start(bridge);
        let app = Peer::new("save_zone_app");
        let client = app.with(|n| n.create_client::<SetBool::Service>(name, QosProfile::services_default())).unwrap();

        // No bus server: no DDS server.
        eventually("the witness's DDS server", || app.service_servers(witness) == (1, 1)).await;
        assert_eq!(app.service_servers(name), (0, 0));
        assert!(!running.service(name).dds_server);
        assert_no_dds_server(&client, "no module").await;

        for round in 0..3 {
            // The module comes up: so does the DDS server, which answers.
            let module = bus.scope("map_manage");
            let _server = serve_on_bus(&module, name);
            eventually("the DDS server", || running.service(name).dds_server).await;
            wait_for_service(&client).await;
            eventually("one DDS server on the graph, none left over", || app.service_servers(name) == (1, 1)).await;
            let answer = tokio::time::timeout(WAIT, client.request(&set_bool(round % 2 == 0)).unwrap()).await.unwrap().unwrap();
            assert_eq!(answer.message, format!("bus got {}", round % 2 == 0));

            // It fails: within about a second its DDS server is gone.
            module.close();
            let failed = Instant::now();
            eventually("the DDS server to leave the graph", || app.service_servers(name) == (0, 0)).await;
            assert!(failed.elapsed() < Duration::from_secs(1), "round {round}: gone after {:?}", failed.elapsed());
            assert!(!running.service(name).dds_server);
            assert_no_dds_server(&client, &format!("round {round}, the module down")).await;
        }
        let stats = running.service(name);
        assert_eq!((stats.forwarded, stats.unanswered), (3, 0));
        running.stop().await.unwrap();
    });
}

/// A bus server that comes and goes every few milliseconds (a module that
/// keeps dropping its `Service`; a failed module waits 2 s for its restart):
/// the DDS server changes at most once per 200 ms (`DWELL`), so the spin
/// thread does not thrash. Once the module stays up there is one DDS
/// server, which answers, and once it stays down none: nothing is left
/// behind on the graph.
#[test]
fn a_bus_server_that_flaps_leaves_one_dds_server_or_none() {
    within(async {
        let name = "/bridge_test/flapping";
        let bus = Bus::new();
        let mut bridge = new_bridge("flapping_bridge", &bus.scope("bridge"));
        bridge.service_to_ros::<SetBool::Service>(name).unwrap();
        let running = start(bridge);
        let app = Peer::new("flapping_app");
        let client = app.with(|n| n.create_client::<SetBool::Service>(name, QosProfile::services_default())).unwrap();
        let module = bus.scope("map_manage");
        let flapping = Instant::now();
        for _ in 0..100 {
            let server = module.service::<SetBool::Request, SetBool::Response>(name).unwrap();
            tokio::time::sleep(Duration::from_millis(5)).await;
            drop(server);
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        let flapped = flapping.elapsed();
        let changes = running.service(name).resyncs;
        let bound = flapped.as_millis() as u64 / 200 + 2;
        assert!(changes <= bound, "{changes} DDS server changes in {flapped:?}");

        // It stays down, then up. The DDS server lags the bus by up to one
        // dwell, so a call to it in between could be lost with it, as one to
        // a module's own r2r server that it destroys and makes again.
        eventually("the DDS server to go once the module stays down", || app.service_servers(name) == (0, 0)).await;
        let _server = serve_on_bus(&module, name);
        wait_for_service(&client).await;
        let answer = tokio::time::timeout(WAIT, client.request(&set_bool(true)).unwrap()).await.unwrap().unwrap();
        assert_eq!(answer.message, "bus got true");
        eventually("one DDS server on the graph, none left over", || app.service_servers(name) == (1, 1)).await;
        module.close();
        eventually("the DDS server to leave the graph", || app.service_servers(name) == (0, 0)).await;
        assert!(!running.service(name).dds_server);
        running.stop().await.unwrap();
    });
}

/// A DDS call made as the module fails may reach the bridge before its DDS
/// server goes, and then finds no bus server. It must not be made to the
/// module that comes up next: the DDS server that took it is gone, so the
/// client would never get the answer, but a dispatch or a lock would have
/// run. It is lost, as a call to a crashed r2r server was.
#[test]
fn a_dds_call_made_as_the_bus_server_goes_is_not_made_to_the_next_one() {
    on_one_thread(async {
        let name = "/bridge_test/dispatch_as_it_goes";
        let bus = Bus::new();
        let mut bridge = new_bridge("dispatch_as_it_goes_bridge", &bus.scope("bridge"));
        bridge.service_to_ros::<SetBool::Service>(name).unwrap();
        let running = start(bridge);
        let runs = Arc::new(AtomicUsize::new(0));
        let counting = |scope: &Scope| {
            let mut service = scope.service::<SetBool::Request, SetBool::Response>(name).unwrap();
            let runs = runs.clone();
            tokio::spawn(async move {
                while let Some(request) = service.next().await {
                    runs.fetch_add(1, Ordering::SeqCst);
                    let _ = request.respond(SetBool::Response { success: true, message: "ran".into() });
                }
            })
        };
        let first = bus.scope("mission_manager");
        let _first = counting(&first);
        let app = Peer::new("dispatch_as_it_goes_app");
        let client = app.with(|n| n.create_client::<SetBool::Service>(name, QosProfile::services_default())).unwrap();
        wait_for_service(&client).await;

        // No await in between: the bridge has not looked yet.
        first.close();
        let mut lost = Box::pin(client.request(&set_bool(true)).unwrap());
        eventually("the DDS server to go", || app.service_servers(name) == (0, 0)).await;
        assert!(!running.service(name).dds_server);
        let second = bus.scope("mission_manager");
        let _second = counting(&second);
        wait_for_service(&client).await;
        let answer = tokio::time::timeout(WAIT, client.request(&set_bool(false)).unwrap()).await.unwrap().unwrap();
        assert_eq!(answer.message, "ran");
        assert_eq!(runs.load(Ordering::SeqCst), 1, "the call made as the first server went ran on the second");
        tokio::time::sleep(Duration::from_millis(200)).await;
        assert!((&mut lost).now_or_never().is_none(), "answered by a DDS server that is gone");
        assert_eq!(runs.load(Ordering::SeqCst), 1);
        running.stop().await.unwrap();
    });
}

/// A bus server that takes a call and ends before answering (its scope
/// closes while it holds the call, as when its module fails), while its
/// replacement is already up. The call must not be made again: the
/// handler may have run, and dispatch, start-coverage or the lock services
/// are not idempotent. A direct r2r server never runs a request twice; the
/// DDS request is lost, as a crash of that server would lose it.
#[test]
fn a_dds_call_a_bus_server_took_is_never_made_again() {
    on_one_thread(async {
        let name = "/bridge_test/dispatch_once";
        let bus = Bus::new();
        let mut bridge = new_bridge("dispatch_once_bridge", &bus.scope("bridge"));
        bridge.service_to_ros::<SetBool::Service>(name).unwrap();
        let running = start(bridge);
        let runs = Arc::new(AtomicUsize::new(0));

        // The first instance runs its handler for the call and holds it.
        let first = bus.scope("mission_manager");
        let mut service = first.service::<SetBool::Request, SetBool::Response>(name).unwrap();
        let (taken, held) = tokio::sync::oneshot::channel();
        let _holder = {
            let runs = runs.clone();
            tokio::spawn(async move {
                if let Some(request) = service.next().await {
                    runs.fetch_add(1, Ordering::SeqCst);
                    let _ = taken.send(());
                    let _request = request;
                    while service.next().await.is_some() {}
                }
            })
        };
        let app = Peer::new("dispatch_once_app");
        let client = app.with(|n| n.create_client::<SetBool::Service>(name, QosProfile::services_default())).unwrap();
        wait_for_service(&client).await;
        let mut call = Box::pin(client.request(&set_bool(true)).unwrap());
        tokio::time::timeout(WAIT, held).await.expect("the first instance took the call").unwrap();

        // It ends holding the call, and its replacement is up before the
        // bridge can look: no await in between, on one thread.
        first.close();
        let second = bus.scope("mission_manager");
        let mut replacement = second.service::<SetBool::Request, SetBool::Response>(name).unwrap();
        let _answering = {
            let runs = runs.clone();
            tokio::spawn(async move {
                while let Some(request) = replacement.next().await {
                    runs.fetch_add(1, Ordering::SeqCst);
                    let _ = request.respond(SetBool::Response { success: true, message: "replacement".into() });
                }
            })
        };

        eventually("the bridge to give the call up, or to make it again", || {
            running.service(name).unanswered == 1 || runs.load(Ordering::SeqCst) > 1
        })
        .await;
        assert_eq!(runs.load(Ordering::SeqCst), 1, "the handler ran again for a call a server had taken");
        assert_eq!(running.service(name).unanswered, 1);
        // The bridge has let go of it, so nothing can answer it any more.
        tokio::time::sleep(Duration::from_millis(200)).await;
        assert!((&mut call).now_or_never().is_none(), "the DDS request ends unanswered");
        assert_eq!(runs.load(Ordering::SeqCst), 1);

        // The replacement serves the calls that come after.
        let answer = tokio::time::timeout(WAIT, client.request(&set_bool(false)).unwrap()).await.unwrap().unwrap();
        assert_eq!(answer.message, "replacement");
        assert_eq!(runs.load(Ordering::SeqCst), 2);
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
        // The bus server comes once the graph shows the DDS server and the
        // bridge's client is matched with it: the first call is answered.
        tokio::time::timeout(WAIT, client.wait_for_service()).await.expect("the bus server").unwrap();
        let answer = tokio::time::timeout(WAIT, client.request(&set_bool(true)).unwrap()).await.unwrap().unwrap();
        assert_eq!(answer.message, "dds got true");
        let calls = [false, true].map(|data| client.request(&set_bool(data)).unwrap());
        let answers = tokio::time::timeout(WAIT, futures::future::join_all(calls)).await.unwrap();
        let answers: Vec<String> = answers.into_iter().map(|a| a.unwrap().message).collect();
        assert_eq!(answers, ["dds got false", "dds got true"]);
        running.stop().await.unwrap();
    });
}

/// Until the bus caller's call is in flight, there is no bus server while
/// there is no DDS server; a held call ends when its server goes with its
/// process.
#[test]
fn a_call_held_by_a_dds_server_that_goes_ends_and_the_bus_server_goes_with_it() {
    within(async {
        let name = "/bridge_test/navigate";
        let bus = Bus::new();
        let mut bridge = new_bridge("navigate_bridge", &bus.scope("bridge"));
        bridge.service_from_ros::<SetBool::Service>(name).unwrap();
        let running = start(bridge);
        let client = bus.scope("nav_action_server").client::<SetBool::Request, SetBool::Response>(name).unwrap();
        eventually("the bridge to look at the graph", || running.service(name).resyncs >= 2).await;
        assert!(!running.service(name).bus_server);
        assert!(matches!(client.request(&set_bool(true)), Err(BusError::ServiceUnavailable { .. })), "refused at the call");

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
        tokio::time::timeout(WAIT, client.wait_for_service()).await.expect("the bus server").unwrap();
        // The bridge's client is matched by then: the server takes the call.
        let mut pending = Box::pin(client.request(&set_bool(true)).unwrap());
        tokio::time::timeout(WAIT, held).await.unwrap().unwrap();
        assert!((&mut pending).now_or_never().is_none(), "a held call cannot be answered");
        drop(nav2);
        // The bus server goes with it (ServiceUnavailable), or a check of the
        // pending call finds it gone first (NoResponse).
        let answer = tokio::time::timeout(WAIT, pending).await.unwrap();
        assert!(
            matches!(answer, Err(BusError::ServiceUnavailable { .. } | BusError::NoResponse { .. })),
            "{answer:?}"
        );
        eventually("the bus server to go", || !running.service(name).bus_server).await;
        assert!(matches!(client.request(&set_bool(true)), Err(BusError::ServiceUnavailable { .. })));
        let _ = tokio::time::timeout(WAIT, holder).await;
        running.stop().await.unwrap();
    });
}

/// mower_rs_common's guard waits a second for the lock service, then says
/// "unavailable", which a caller retries; a call that gets no answer is a
/// lock that may have been granted, and it stays latched until a restart.
/// Through the bridge it must see what it saw on r2r: no server while nav is
/// down, one when it is up, and none again once nav goes, never a server
/// that takes calls nobody answers. The graph shows nav's server before the
/// bridge's client is matched with it, which is why the bus server waits for
/// a probe: a call in that window would come back `NoResponse`.
#[test]
fn service_from_ros_serves_on_the_bus_only_while_a_dds_server_is_there() {
    within(async {
        let name = "/bridge_test/mission_operation_lock";
        let bus = Bus::new();
        let mut bridge = new_bridge("comes_and_goes_bridge", &bus.scope("bridge"));
        bridge.service_from_ros::<SetBool::Service>(name).unwrap();
        let running = start(bridge);
        let client = bus.scope("map_manage").client::<SetBool::Request, SetBool::Response>(name).unwrap();
        // The guard's two outcomes: the call's answer, or "unavailable".
        let guard = || async {
            match tokio::time::timeout(Duration::from_secs(1), client.wait_for_service()).await {
                Err(_) => None,
                Ok(waited) => {
                    waited.unwrap();
                    Some(client.request(&set_bool(true)))
                }
            }
        };

        // nav is down
        eventually("the bridge to look at the graph", || running.service(name).resyncs >= 2).await;
        assert!(guard().await.is_none(), "unavailable, retryable");
        assert!(matches!(client.request(&set_bool(true)), Err(BusError::ServiceUnavailable { .. })));

        for round in 0..2 {
            // nav comes up
            let nav = Peer::new(&format!("comes_and_goes_nav_{round}"));
            let server = serve_on_dds(&nav, name);
            let deadline = Instant::now() + WAIT;
            let answer = loop {
                match guard().await {
                    None => assert!(Instant::now() < deadline, "round {round}: the bus server never came"),
                    // Whatever the timing, the first call the guard makes is
                    // answered.
                    Some(call) => match call.unwrap().await {
                        Ok(answer) => break answer,
                        Err(e) => panic!("round {round}: the guard's call failed ({e}): it would stay latched"),
                    },
                }
            };
            assert_eq!(answer.message, "dds got true");
            assert!(running.service(name).bus_server);

            // nav goes
            server.abort();
            drop(nav);
            eventually("the bus server to go", || !running.service(name).bus_server).await;
            assert!(guard().await.is_none(), "round {round}: unavailable again");
        }
        running.stop().await.unwrap();
    });
}

/// A DDS server that takes a call and never answers, while it stays matched:
/// no availability check can end the call, the deadline does.
#[test]
fn a_dds_call_that_is_never_answered_ends_at_its_deadline() {
    within(async {
        let name = "/bridge_test/never_answered";
        let deadline = Duration::from_secs(1);
        let bus = Bus::new();
        let timeouts = ServiceTimeouts { dds_call: deadline, ..ServiceTimeouts::default() };
        let mut bridge = new_bridge("never_answered_bridge", &bus.scope("bridge")).with_service_timeouts(timeouts);
        bridge.service_from_ros::<SetBool::Service>(name).unwrap();
        let running = start(bridge);
        let client = bus.scope("nav_action_server").client::<SetBool::Request, SetBool::Response>(name).unwrap();
        let nav2 = Peer::new("never_answered_nav2");
        let mut requests = nav2.with(|n| n.create_service::<SetBool::Service>(name, QosProfile::services_default())).unwrap();
        let (taken, mut held) = tokio::sync::mpsc::unbounded_channel();
        let _holder = tokio::spawn(async move {
            // Takes every call and keeps it, unanswered, for as long as it runs.
            while let Some(request) = requests.next().await {
                let _ = taken.send(request);
            }
        });
        // Once the bus server is there, the server is matched: it takes the
        // first call, which runs into the deadline.
        tokio::time::timeout(WAIT, client.wait_for_service()).await.expect("the bus server").unwrap();
        let started = Instant::now();
        let answer = tokio::time::timeout(WAIT, client.request(&set_bool(true)).unwrap()).await.unwrap();
        assert_eq!(answer, Err(BusError::NoResponse { name: name.into() }));
        assert!(held.try_recv().is_ok(), "the DDS server never took the call");
        let took = started.elapsed();
        assert!(took >= deadline && took < deadline + Duration::from_secs(5), "ended after {took:?}");
        eventually("the call counted", || running.service(name).unanswered >= 1).await;
        running.stop().await.unwrap();
    });
}

/// A DDS server that takes the call and is replaced (it crashes and launch
/// respawns it, or a second instance comes up before the first dies) leaves
/// a server matched at every check, so only the deadline ends the call.
#[test]
fn a_dds_call_whose_server_is_replaced_mid_call_still_ends() {
    within(async {
        let name = "/bridge_test/replaced_get_state";
        let bus = Bus::new();
        let timeouts = ServiceTimeouts { dds_call: Duration::from_secs(3), ..ServiceTimeouts::default() };
        let mut bridge = new_bridge("replaced_server_bridge", &bus.scope("bridge")).with_service_timeouts(timeouts);
        bridge.service_from_ros::<SetBool::Service>(name).unwrap();
        let running = start(bridge);
        let client = bus.scope("nav_action_server").client::<SetBool::Request, SetBool::Response>(name).unwrap();

        // The first server takes calls and never answers them.
        let first = Peer::new("replaced_server_first");
        let mut requests = first.with(|n| n.create_service::<SetBool::Service>(name, QosProfile::services_default())).unwrap();
        let (taken, held) = tokio::sync::oneshot::channel();
        let holder = tokio::spawn(async move {
            if let Some(request) = requests.next().await {
                let _ = taken.send(());
                let _request = request;
                while requests.next().await.is_some() {}
            }
        });
        tokio::time::timeout(WAIT, client.wait_for_service()).await.expect("the bus server").unwrap();
        let mut pending = Box::pin(client.request(&set_bool(true)).unwrap());
        tokio::time::timeout(WAIT, held).await.unwrap().unwrap();
        assert!((&mut pending).now_or_never().is_none(), "a held call cannot be answered");

        // Its replacement comes up, then the first one goes with its process.
        let second = Peer::new("replaced_server_second");
        let _answering = serve_on_dds(&second, name);
        tokio::time::sleep(Duration::from_millis(300)).await; // the bridge's client matches it
        drop(first);
        let _ = tokio::time::timeout(WAIT, holder).await;

        let answer = tokio::time::timeout(Duration::from_secs(8), pending).await;
        assert!(answer.is_ok(), "the call the first server took never ended (unanswered so far: {})", running.service(name).unanswered);
        assert_eq!(answer.unwrap(), Err(BusError::NoResponse { name: name.into() }));
        running.stop().await.unwrap();
    });
}

// ---------------------------------------------------------------------------
// Values r2r cannot convert
// ---------------------------------------------------------------------------

/// A fix whose covariance has 8 entries where `NavSatFix` fixes 9: r2r
/// asserts the length when it converts the value to its C message.
fn short_covariance(latitude: f64) -> NavSatFix {
    NavSatFix { latitude, position_covariance: vec![0.0; 8], ..Default::default() }
}

fn fix(latitude: f64) -> NavSatFix {
    NavSatFix { latitude, ..Default::default() }
}

/// A module that makes a value its ROS type cannot carry used to panic in its
/// own publish or respond, and only it restarted. Through the bridge the
/// value is dropped, counted (`invalid`) and logged, and the forwarder, the
/// spin thread and every other name carry on.
#[test]
fn a_value_r2r_cannot_convert_is_dropped_and_the_bridge_carries_on() {
    within(async {
        let [fixes_name, latched_name, get_state, set_map] =
            ["fixes", "latched_fix", "get_state", "set_map"].map(|n| format!("/bridge_test/invalid_{n}"));
        let bus = Bus::new();
        let gps = bus.scope("gps");
        let fixes = gps.publisher::<NavSatFix>(&fixes_name, Qos::default()).unwrap();
        let latched_fixes = gps.publisher::<NavSatFix>(&latched_name, Qos::latched()).unwrap();
        // An EKF whose state has 14 entries where GetState fixes 15, when it
        // is asked for frame "bad".
        let mut states = gps.service::<GetState::Request, GetState::Response>(&get_state).unwrap();
        let _ekf = tokio::spawn(async move {
            while let Some(request) = states.next().await {
                let n = if request.message.frame_id == "bad" { 14 } else { 15 };
                let _ = request.respond(GetState::Response { state: vec![1.0; n], ..Default::default() });
            }
        });
        let mut bridge = new_bridge("invalid_bridge", &bus.scope("bridge"));
        bridge.to_ros::<NavSatFix>(&fixes_name, QosProfile::default()).unwrap();
        bridge.to_ros::<NavSatFix>(&latched_name, latched()).unwrap();
        bridge.service_to_ros::<GetState::Service>(&get_state).unwrap();
        bridge.service_from_ros::<SetMap::Service>(&set_map).unwrap();
        let running = start(bridge);
        let app = Peer::new("invalid_app");

        // to_ros, volatile: the bad fix is dropped, the next one goes out.
        let mut sub = app.with(|n| n.subscribe::<NavSatFix>(&fixes_name, QosProfile::default())).unwrap();
        let deadline = Instant::now() + WAIT;
        loop {
            fixes.publish(&fix(1.0)).unwrap();
            if let Ok(Some(_)) = tokio::time::timeout(Duration::from_millis(50), sub.next()).await {
                break;
            }
            assert!(Instant::now() < deadline, "timed out waiting for the match");
        }
        fixes.publish(&short_covariance(2.0)).unwrap();
        fixes.publish(&fix(3.0)).unwrap();
        while next("the fix after the bad one", &mut sub).await.latitude != 3.0 {}
        eventually("the bad fix counted", || running.topic(&fixes_name).invalid == 1).await;

        // to_ros, transient local: the bad value supersedes on the bus what
        // the DDS writer held, so a DDS late joiner gets neither, as when the
        // module died of it; then the next value.
        latched_fixes.publish(&fix(4.0)).unwrap();
        eventually("the latched fix on DDS", || running.topic(&latched_name).forwarded == 1).await;
        eventually("the bridge's DDS writer on the graph", || app.writer_gids(&latched_name).len() == 1).await;
        let writer = app.writer_gids(&latched_name);
        latched_fixes.publish(&short_covariance(5.0)).unwrap();
        eventually("the bad latched fix counted", || running.topic(&latched_name).invalid == 1).await;
        eventually("a DDS writer without the superseded value", || {
            let now = app.writer_gids(&latched_name);
            now.len() == 1 && now != writer
        })
        .await;
        let mut late = app.with(|n| n.subscribe::<NavSatFix>(&latched_name, latched())).unwrap();
        assert!(tokio::time::timeout(Duration::from_millis(500), late.next()).await.is_err(), "the superseded value stayed on DDS");
        latched_fixes.publish(&fix(6.0)).unwrap();
        assert_eq!(next("the next latched fix", &mut late).await.latitude, 6.0);

        // service_to_ros: the bad answer leaves the DDS call unanswered, the
        // next call is answered.
        let state_client = app.with(|n| n.create_client::<GetState::Service>(&get_state, QosProfile::services_default())).unwrap();
        let available = r2r::Node::is_available(&state_client).unwrap();
        tokio::time::timeout(WAIT, available).await.expect("the DDS server").unwrap();
        let mut bad = Box::pin(state_client.request(&GetState::Request { frame_id: "bad".into(), ..Default::default() }).unwrap());
        eventually("the bad answer counted", || running.service(&get_state).invalid == 1).await;
        assert_eq!(running.service(&get_state).unanswered, 1);
        let good = GetState::Request { frame_id: "good".into(), ..Default::default() };
        let answer = tokio::time::timeout(WAIT, state_client.request(&good).unwrap()).await.unwrap().unwrap();
        assert_eq!(answer.state.len(), 15);
        assert!((&mut bad).now_or_never().is_none(), "the bad answer reached DDS");

        // service_from_ros: a bus request with a 35-entry covariance where
        // the type fixes 36 gets NoResponse, the next one its answer.
        let map_server = Peer::new("invalid_map_server");
        let mut maps = map_server.with(|n| n.create_service::<SetMap::Service>(&set_map, QosProfile::services_default())).unwrap();
        let _map_server = tokio::spawn(async move {
            while let Some(request) = maps.next().await {
                let _ = request.respond(SetMap::Response { success: true });
            }
        });
        let map_client = bus.scope("map_manage").client::<SetMap::Request, SetMap::Response>(&set_map).unwrap();
        tokio::time::timeout(WAIT, map_client.wait_for_service()).await.expect("the bus server").unwrap();
        let mut bad_map = SetMap::Request::default();
        bad_map.initial_pose.pose.covariance = vec![0.0; 35];
        let refused = tokio::time::timeout(WAIT, map_client.request(&bad_map).unwrap()).await.unwrap();
        assert_eq!(refused, Err(BusError::NoResponse { name: set_map.clone() }));
        assert_eq!(running.service(&set_map).invalid, 1);
        let answer = tokio::time::timeout(WAIT, map_client.request(&SetMap::Request::default()).unwrap()).await.unwrap().unwrap();
        assert!(answer.success);

        // Nothing stopped: the bridge still runs, and stops cleanly.
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
        // The latched reader is a child of the bridge's scope: ended by the
        // close itself, before the forwarder has even looked.
        assert_eq!(everything.publisher.subscription_count(), 0);
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

fn nav_sat_fix() -> NavSatFix {
    let mut fix = NavSatFix { header: header(), ..Default::default() };
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
