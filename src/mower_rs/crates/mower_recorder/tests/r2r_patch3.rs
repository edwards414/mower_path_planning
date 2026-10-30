//! vendor/r2r's patch 3 (VENDORED.md) against real DDS: what the recorder
//! needs of r2r that upstream 0.9.7 does not give.
//!
//! - `subscribe_raw_with_info`: the bytes as rmw took them (the CDR padded to
//!   4, as on the wire and in rosbag2's bags), the source and received
//!   timestamps, the writer's GID, a queue of the caller's size and a count
//!   of what a full queue dropped; `destroy_raw_subscription`.
//! - `get_node_names_and_namespaces`, `get_service_names_and_types`,
//!   `count_publishers` / `count_subscribers`, and `get_topic_names_and_types`
//!   with a topic of two types (the patch fixes upstream's reading of the
//!   types).
//! - `TopicEndpointInfo::topic_type_hash`, `RIHS01_<hex>` as rosbag2 writes
//!   it: the hashes of E5-1's table, which were read from rosbag2's bags.
//!
//! - **Isolation:** ROS domain 104 (the other crates' DDS tests use 77-93,
//!   tests/module.rs 105), rmw_cyclonedds_cpp, set before the context exists;
//!   names of their own. A cyclonedds port base (`CYCLONEDDS_URI`) separates
//!   parallel runs on one host.
//! - **Two writers' processes:** a node of this process (one participant, as
//!   mower_rsd's modules), and this test binary started again as a child
//!   (`child_publisher`), another participant, as rosbag2 sees the stack.

use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock, PoisonError};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use futures::StreamExt;
use r2r::std_msgs::msg::{Bool, String as Text};
use r2r::{QosProfile, WrappedTypesupport};

const WAIT: Duration = Duration::from_secs(20);
const DOMAIN: &str = "104";

fn context() -> r2r::Context {
    static CONTEXT: OnceLock<r2r::Context> = OnceLock::new();
    CONTEXT
        .get_or_init(|| {
            // rcl reads both when the context is created, and only then
            std::env::set_var("ROS_DOMAIN_ID", DOMAIN);
            std::env::set_var("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp");
            r2r::Context::create().expect("an rcl context (source /opt/ros/jazzy/setup.bash)")
        })
        .clone()
}

/// One test at a time: they share the process's participant.
fn serial() -> MutexGuard<'static, ()> {
    static ONE: Mutex<()> = Mutex::new(());
    ONE.lock().unwrap_or_else(PoisonError::into_inner)
}

fn within(test: impl std::future::Future<Output = ()>) {
    let _one = serial();
    let runtime = tokio::runtime::Builder::new_multi_thread().worker_threads(2).enable_all().build().unwrap();
    runtime.block_on(async {
        tokio::time::timeout(Duration::from_secs(120), test).await.expect("the test hung");
    });
}

/// A node spun on a thread of its own, its endpoints made between spins.
struct Peer {
    node: Arc<Mutex<r2r::Node>>,
    running: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl Peer {
    fn new(name: &str, namespace: &str) -> Peer {
        let node = Arc::new(Mutex::new(r2r::Node::create(context(), name, namespace).unwrap()));
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

fn now_ns() -> i64 {
    SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_nanos() as i64
}

/// What rmw puts on the wire for `msg`: its serialization, zero-padded to a
/// multiple of 4 (design 2.1: every payload rosbag2 recorded is).
fn on_the_wire<T: WrappedTypesupport>(msg: &T) -> Vec<u8> {
    let mut bytes = msg.to_serialized_bytes().unwrap();
    while bytes.len() % 4 != 0 {
        bytes.push(0);
    }
    bytes
}

fn reliable(depth: usize) -> QosProfile {
    QosProfile::default().keep_last(depth).reliable()
}

/// The texts `child_publisher` sends.
fn child_text(i: usize) -> String {
    format!("from the child #{i} {}", "x".repeat(i))
}
const CHILD_TOPIC: &str = "/e5p3/child";
const CHILD_COUNT: usize = 7;

/// Not a test of its own: this binary, started again by
/// `a_writer_in_another_process_arrives_padded_with_its_times_and_gid` with
/// `E5P3_CHILD=1`, is the writer there. It waits for that reader, sends
/// [`CHILD_COUNT`] texts and waits until the reader has gone.
#[test]
fn child_publisher() {
    if std::env::var("E5P3_CHILD").as_deref() != Ok("1") {
        return;
    }
    let peer = Peer::new("e5p3_child", "");
    let publisher = peer.with(|n| n.create_publisher::<Text>(CHILD_TOPIC, reliable(100)).unwrap());
    let readers = || peer.with(|n| n.get_subscriptions_info_by_topic(CHILD_TOPIC, false).unwrap().len());
    let deadline = Instant::now() + WAIT;
    while readers() == 0 {
        assert!(Instant::now() < deadline, "no reader came");
        std::thread::sleep(Duration::from_millis(10));
    }
    for i in 0..CHILD_COUNT {
        publisher.publish(&Text { data: child_text(i) }).unwrap();
        std::thread::sleep(Duration::from_millis(20));
    }
    let deadline = Instant::now() + WAIT;
    while readers() != 0 && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(10));
    }
}

/// What rmw puts in `publisher_gid` for the writer whose endpoint GID (the
/// graph's, its DDS GUID) is `endpoint`: that GUID, or, on rmw_cyclonedds,
/// the writer's DDS instance handle in the reader's participant, 8 bytes and
/// zeros (rmw_cyclonedds copies `dds_sample_info_t::publication_handle`).
fn writer_id_of(gid: &[u8; 16], endpoint: &[u8; 16]) -> bool {
    gid == endpoint || (gid[8..] == [0; 8] && gid[..8] != [0; 8])
}

/// rosbag2's case: the writer is another participant. The bytes are its
/// CDR padded to 4, `source_timestamp` is its publish time and comes before
/// `received_timestamp`, and `publisher_gid` names the writer: one value for
/// all its messages (on rmw_cyclonedds its instance handle, not the GUID the
/// graph shows: [`writer_id_of`]).
#[test]
fn a_writer_in_another_process_arrives_padded_with_its_times_and_gid() {
    within(async {
        let reader = Peer::new("e5p3_reader", "");
        let mut sub = reader
            .with(|n| n.subscribe_raw_with_info(CHILD_TOPIC, "std_msgs/msg/String", reliable(100), 100))
            .unwrap();
        let started = now_ns();
        let mut child = Command::new(std::env::current_exe().unwrap())
            .args(["--exact", "child_publisher", "--nocapture", "--test-threads", "1"])
            .env("E5P3_CHILD", "1")
            .stdout(Stdio::null())
            .spawn()
            .unwrap();
        let mut got = Vec::new();
        while got.len() < CHILD_COUNT {
            match tokio::time::timeout(WAIT, sub.next()).await {
                Ok(Some(m)) => got.push(m),
                other => panic!("the child's texts: {} of {CHILD_COUNT}, then {other:?}", got.len()),
            }
        }
        let ended = now_ns();
        let writers = reader.with(|n| n.get_publishers_info_by_topic(CHILD_TOPIC, false).unwrap());
        let child_gid: Vec<[u8; 16]> = writers.iter().filter(|w| w.node_name == "e5p3_child").map(|w| w.endpoint_gid).collect();
        assert_eq!(child_gid.len(), 1, "the child's one writer");
        for (i, m) in got.iter().enumerate() {
            let text = Text { data: child_text(i) };
            assert_eq!(m.data, on_the_wire(&text), "#{i}: the bytes on the wire");
            assert_eq!(m.data.len() % 4, 0);
            assert!(started <= m.source_timestamp && m.source_timestamp <= m.received_timestamp, "#{i}: {m:?}");
            assert!(m.received_timestamp <= ended, "#{i}: received after the test saw it");
            assert!(m.received_timestamp - m.source_timestamp < 1_000_000_000, "#{i}: a second on loopback");
            assert!(writer_id_of(&m.publisher_gid, &child_gid[0]), "#{i}: {:?}", m.publisher_gid);
            assert_eq!(m.publisher_gid, got[0].publisher_gid, "#{i}: one writer, one id");
        }
        // they come in the order they were sent, a source time each
        assert!(got.windows(2).all(|w| w[0].source_timestamp < w[1].source_timestamp));
        assert_eq!(sub.drops().get(), 0);
        reader.with(|n| n.destroy_raw_subscription(sub));
        let status = child.wait().unwrap();
        assert!(status.success(), "the child: {status}");
    });
}

/// Writers of the reader's own participant (mower_rsd's case, and design
/// G2's open question): the same bytes, padded too, the same times, and a
/// `publisher_gid` per writer: two writers of one topic, two ids, each the
/// same for all of its messages. A Bool, whose CDR is 5 bytes unpadded.
#[test]
fn a_writer_in_this_process_arrives_the_same_way() {
    within(async {
        let reader = Peer::new("e5p3_same_reader", "");
        let one = Peer::new("e5p3_same_one", "");
        let two = Peer::new("e5p3_same_two", "");
        let topic = "/e5p3/same";
        let mut sub = reader.with(|n| n.subscribe_raw_with_info(topic, "std_msgs/msg/Bool", reliable(10), 10)).unwrap();
        let p1 = one.with(|n| n.create_publisher::<Bool>(topic, reliable(10)).unwrap());
        let p2 = two.with(|n| n.create_publisher::<Bool>(topic, reliable(10)).unwrap());
        eventually("both writers to match", || reader.with(|n| n.get_publishers_info_by_topic(topic, false).unwrap().len()) == 2).await;
        let endpoint = |name: &str| -> [u8; 16] {
            reader.with(|n| n.get_publishers_info_by_topic(topic, false).unwrap()).into_iter().find(|w| w.node_name == name).unwrap().endpoint_gid
        };
        let (e1, e2) = (endpoint("e5p3_same_one"), endpoint("e5p3_same_two"));
        assert_ne!(e1, e2);
        let mut ids = Vec::new();
        for (publisher, data, endpoint) in [(&p1, true, e1), (&p2, false, e2), (&p1, true, e1)] {
            let before = now_ns();
            publisher.publish(&Bool { data }).unwrap();
            let m = tokio::time::timeout(WAIT, sub.next()).await.unwrap().unwrap();
            assert_eq!(m.data, [0, 1, 0, 0, data as u8, 0, 0, 0], "padded to 4, as rosbag2 records it");
            assert!(before <= m.source_timestamp && m.source_timestamp <= m.received_timestamp, "{m:?}");
            assert!(m.received_timestamp <= now_ns());
            assert!(writer_id_of(&m.publisher_gid, &endpoint), "{:?}", m.publisher_gid);
            ids.push(m.publisher_gid);
        }
        assert_eq!(Bool { data: true }.to_serialized_bytes().unwrap(), [0, 1, 0, 0, 1], "rmw's own, unpadded");
        assert_eq!(ids[0], ids[2], "one writer, one id");
        assert_ne!(ids[0], ids[1], "two writers, two ids");
        drop(sub);
    });
}

/// The queue holds what the caller asked for, a full queue drops the newest
/// and counts it, and a destroyed subscription leaves the graph at once.
#[test]
fn the_queue_is_the_callers_and_what_it_drops_is_counted() {
    within(async {
        let reader = Peer::new("e5p3_queue_reader", "");
        let writer = Peer::new("e5p3_queue_writer", "");
        let topic = "/e5p3/queue";
        // DDS keeps all 20 (reliable, depth 100): only the queue drops
        let sub = reader.with(|n| n.subscribe_raw_with_info(topic, "std_msgs/msg/String", reliable(100), 3)).unwrap();
        let drops = sub.drops();
        let publisher = writer.with(|n| n.create_publisher::<Text>(topic, reliable(100)).unwrap());
        eventually("the reader to match", || writer.with(|n| n.get_subscriptions_info_by_topic(topic, false).unwrap().len()) == 1).await;
        for i in 0..20 {
            publisher.publish(&Text { data: format!("#{i}") }).unwrap();
        }
        // nothing reads the stream: 3 queued, 17 dropped
        eventually("the 17 drops", || drops.get() == 17).await;
        tokio::time::sleep(Duration::from_millis(200)).await;
        assert_eq!(drops.get(), 17, "and no more");
        let mut sub = sub;
        let mut queued = Vec::new();
        while let Ok(Some(m)) = tokio::time::timeout(Duration::from_millis(300), sub.next()).await {
            queued.push(m.data);
        }
        let first: Vec<Vec<u8>> = (0..3).map(|i| on_the_wire(&Text { data: format!("#{i}") })).collect();
        assert_eq!(queued, first, "the first three, the rest dropped");
        // the next one fits again, counted nowhere
        publisher.publish(&Text { data: "#20".into() }).unwrap();
        let m = tokio::time::timeout(WAIT, sub.next()).await.unwrap().unwrap();
        assert_eq!(m.data, on_the_wire(&Text { data: "#20".into() }));
        assert_eq!(drops.get(), 17);
        // destroyed: off the graph, the counter keeps its count
        reader.with(|n| n.destroy_raw_subscription(sub));
        eventually("the reader to leave the graph", || writer.with(|n| n.get_subscriptions_info_by_topic(topic, false).unwrap().is_empty()))
            .await;
        assert_eq!(drops.get(), 17);
        assert!(reader.with(|n| n.subscribe_raw_with_info(topic, "std_msgs/msg/String", reliable(1), 0)).is_err(), "capacity 0");
    });
}

/// `get_node_names_and_namespaces`: every node, namespaces as rcl gives
/// them, a node that goes leaves the list.
#[test]
fn the_node_names_and_namespaces_of_the_graph() {
    within(async {
        let looker = Peer::new("e5p3_looker", "");
        let other = Peer::new("e5p3_other", "/e5p3_ns/deeper");
        let names = || looker.with(|n| n.get_node_names_and_namespaces().unwrap());
        eventually("both nodes", || {
            let all = names();
            all.contains(&("e5p3_looker".to_string(), "/".to_string()))
                && all.contains(&("e5p3_other".to_string(), "/e5p3_ns/deeper".to_string()))
        })
        .await;
        drop(other);
        eventually("the other node to go", || !names().iter().any(|(n, _)| n == "e5p3_other")).await;
        assert!(names().contains(&("e5p3_looker".to_string(), "/".to_string())));
    });
}

/// `get_service_names_and_types`: every service with its type, in rmw's
/// order, which is by name; a service that goes leaves the list.
#[test]
fn the_service_names_and_types_of_the_graph() {
    within(async {
        let looker = Peer::new("e5p3_svc_looker", "");
        let server = Peer::new("e5p3_svc_server", "");
        let trigger = server.with(|n| n.create_service::<r2r::std_srvs::srv::Trigger::Service>("/e5p3/svc/b_trigger", QosProfile::services_default()).unwrap());
        let set_bool = server.with(|n| n.create_service::<r2r::std_srvs::srv::SetBool::Service>("/e5p3/svc/a_set_bool", QosProfile::services_default()).unwrap());
        let ours = || -> Vec<(String, Vec<String>)> {
            looker.with(|n| n.get_service_names_and_types().unwrap()).into_iter().filter(|(name, _)| name.starts_with("/e5p3/svc/")).collect()
        };
        eventually("both services", || ours().len() == 2).await;
        assert_eq!(
            ours(),
            [
                ("/e5p3/svc/a_set_bool".to_string(), vec!["std_srvs/srv/SetBool".to_string()]),
                ("/e5p3/svc/b_trigger".to_string(), vec!["std_srvs/srv/Trigger".to_string()]),
            ]
        );
        let all: Vec<String> = looker.with(|n| n.get_service_names_and_types().unwrap()).into_iter().map(|(n, _)| n).collect();
        let mut sorted = all.clone();
        sorted.sort();
        assert_eq!(all, sorted, "rmw's order is by name");
        server.with(|n| n.destroy_service(trigger));
        eventually("the trigger to go", || ours().len() == 1).await;
        drop(set_bool);
    });
}

/// A topic of two types gets both, and its neighbour only its own (upstream
/// gave the first one the neighbour's type and read past the last); and
/// `count_publishers` / `count_subscribers` agree with the endpoints' info.
#[test]
fn a_topic_of_two_types_has_both() {
    within(async {
        let a = Peer::new("e5p3_types_a", "");
        let b = Peer::new("e5p3_types_b", "");
        let _p1 = a.with(|n| n.create_publisher::<Text>("/e5p3/types/two", reliable(1)).unwrap());
        let _p2 = b.with(|n| n.create_publisher::<Bool>("/e5p3/types/two", reliable(1)).unwrap());
        let _p3 = a.with(|n| n.create_publisher::<Text>("/e5p3/types/zz_one", reliable(1)).unwrap());
        let types = |name: &str| a.with(|n| n.get_topic_names_and_types().unwrap()).get(name).cloned();
        // both topics: discovery can bring zz_one after two's second type
        eventually("both types", || {
            types("/e5p3/types/two").is_some_and(|t| t.len() == 2) && types("/e5p3/types/zz_one").is_some()
        })
        .await;
        assert_eq!(types("/e5p3/types/two").unwrap(), ["std_msgs/msg/Bool", "std_msgs/msg/String"]);
        assert_eq!(types("/e5p3/types/zz_one").unwrap(), ["std_msgs/msg/String"]);
        // the counts rclpy's graph_snapshot takes, as the endpoints' info has them
        let _r = b.with(|n| n.subscribe::<Text>("/e5p3/types/zz_one", reliable(1)).unwrap());
        let _r2 = a.with(|n| n.subscribe::<Text>("/e5p3/types/zz_one", reliable(1)).unwrap());
        eventually("two readers", || a.with(|n| n.count_subscribers("/e5p3/types/zz_one").unwrap()) == 2).await;
        for topic in ["/e5p3/types/two", "/e5p3/types/zz_one", "/e5p3/types/none"] {
            let (pubs, subs) = a.with(|n| (n.count_publishers(topic).unwrap(), n.count_subscribers(topic).unwrap()));
            let infos = a.with(|n| {
                (n.get_publishers_info_by_topic(topic, false).unwrap().len(), n.get_subscriptions_info_by_topic(topic, false).unwrap().len())
            });
            assert_eq!((pubs, subs), infos, "{topic}");
        }
        assert_eq!(a.with(|n| n.count_publishers("/e5p3/types/two").unwrap()), 2);
    });
}

/// `topic_type_hash` is the RIHS01 hash rosbag2 wrote into its bags' channel
/// metadata (E5-1's table holds them, read from the fixtures), for a writer
/// and a reader alike.
#[test]
fn the_endpoints_carry_their_type_hash() {
    within(async {
        let peer = Peer::new("e5p3_hash", "");
        let _text = peer.with(|n| n.create_publisher::<Text>("/e5p3/hash/text", reliable(1)).unwrap());
        let _bool = peer.with(|n| n.subscribe::<Bool>("/e5p3/hash/bool", reliable(1)).unwrap());
        let _fix = peer.with(|n| n.create_publisher::<r2r::sensor_msgs::msg::NavSatFix>("/e5p3/hash/fix", reliable(1)).unwrap());
        let writer = peer.with(|n| n.get_publishers_info_by_topic("/e5p3/hash/text", false).unwrap());
        let reader = peer.with(|n| n.get_subscriptions_info_by_topic("/e5p3/hash/bool", false).unwrap());
        let fix = peer.with(|n| n.get_publishers_info_by_topic("/e5p3/hash/fix", false).unwrap());
        assert_eq!(writer[0].topic_type_hash, mower_schema::type_hash("std_msgs/msg/String").unwrap());
        assert_eq!(
            writer[0].topic_type_hash,
            "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18",
            "design 2.1's String"
        );
        assert_eq!(reader[0].topic_type_hash, mower_schema::type_hash("std_msgs/msg/Bool").unwrap());
        assert_eq!(fix[0].topic_type_hash, mower_schema::type_hash("sensor_msgs/msg/NavSatFix").unwrap());
    });
}
