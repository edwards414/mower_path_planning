//! The bag module as mower_rsd runs it: its three instances on
//! a bus of their own and a manual clock, each with its r2r node, and, where
//! DDS matters, mower_rsd's bus bridge for the set next to them.
//!
//! - graph_snapshot: the JSON of a tick (the graph as the r2r node sees it,
//!   the stamp), the node diff, the watch rule both ways.
//! - recorder_manager: the idle status and the rear light at the start and
//!   every 2 s, the three services' answers, MediaMTX's off-PATCH and its
//!   retry (a stub API), autostart; a run recorded from DDS (E5-3b): the
//!   groups' nodes while it runs, the names as rosbag2 finds them, the bags,
//!   the snapshot, run_metadata.yaml and the parameter dump.
//! - bag_store: the bag list, every command and its answer, the upload of
//!   `upload_now` and of the 30 s tick to an S3 stub, the R2 delete.
//! - One owner: a foreign server of `/mower_recorder/start` holds all three
//!   back, and its going lets them serve.
//! - Design D5: the rear light reaches DDS as mower_base reads it (R TL 8),
//!   a late joiner included, and parses as mower_base parses it.
//! - Every endpoint against mower_bridge's manifest.
//!
//! - **Isolation:** ROS domain 105 (tests/r2r_patch3.rs has 104),
//!   rmw_cyclonedds_cpp; one test at a time, the nodes in a namespace of the
//!   test's own; files in a scratch directory of the test's own.
//! - **Waiting:** the clock is advanced; what the module does is waited for,
//!   never slept for.

use std::collections::HashMap;
use std::io::{BufRead, BufReader, Read, Write};
use std::net::TcpListener;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock, PoisonError};
use std::time::{Duration, Instant};

use futures::StreamExt;
use mower_bus::{Bus, Clock, ManualClock, Qos, Scope, Subscriber, Time};
use mower_recorder::{graph, manager, store, NODES, NODE_GRAPH, NODE_MANAGER, NODE_STORE};
use mower_rs_common::{ModuleCtx, ModuleResult, Shutdown};
use r2r::std_msgs::msg::{Bool, String as Text};
use r2r::std_srvs::srv::Trigger;
use r2r::{Parameter, ParameterValue as P, QosProfile};

const WAIT: Duration = Duration::from_secs(20);
const T0_NS: i64 = 1_793_000_000_123_456_789;

fn context() -> r2r::Context {
    static CONTEXT: OnceLock<r2r::Context> = OnceLock::new();
    CONTEXT
        .get_or_init(|| {
            // rcl reads both when the context is created, and only then
            std::env::set_var("ROS_DOMAIN_ID", "105");
            std::env::set_var("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp");
            r2r::Context::create().expect("an rcl context (source /opt/ros/jazzy/setup.bash)")
        })
        .clone()
}

fn within(test: impl std::future::Future<Output = ()>) {
    static ONE: Mutex<()> = Mutex::new(());
    let _one: MutexGuard<'_, ()> = ONE.lock().unwrap_or_else(PoisonError::into_inner);
    let runtime = tokio::runtime::Builder::new_multi_thread().worker_threads(3).enable_all().build().unwrap();
    runtime.block_on(async {
        tokio::time::timeout(Duration::from_secs(120), test).await.expect("the test hung");
    });
}

fn repo() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../..")
}

/// A fresh directory of the test's own.
fn scratch(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("e5-3a-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

async fn eventually(what: &str, mut done: impl FnMut() -> bool) {
    let deadline = Instant::now() + WAIT;
    while !done() {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
}

async fn next<T: mower_bus::Message>(what: &str, sub: &mut Subscriber<T>) -> Arc<T> {
    match tokio::time::timeout(WAIT, sub.recv()).await {
        Ok(Some(m)) => m,
        Ok(None) => panic!("{what}: the bus closed"),
        Err(_) => panic!("timed out waiting for {what}"),
    }
}

/// Nothing more on `sub` for a moment.
async fn quiet<T: mower_bus::Message>(sub: &mut Subscriber<T>) {
    tokio::time::sleep(Duration::from_millis(100)).await;
    assert!(sub.try_recv().is_err(), "unexpected message");
}

type Params = Vec<(&'static str, P)>;

fn s(v: &str) -> P {
    P::String(v.to_string())
}

/// mower_rsd.yaml's sections and robot.launch.py's files for the three nodes,
/// with the state under `root`, MediaMTX off unless `video` is given.
fn production(root: &Path, video: Option<&str>) -> Vec<(&'static str, Params)> {
    let share = repo().join("src/mower_recorder/config");
    vec![
        (
            NODE_GRAPH,
            vec![
                ("period_s", P::Double(2.0)),
                ("watch_nodes", P::StringArray(vec!["map_manage".into(), "boustrophedon_coverage".into()])),
                ("fault_topic", s("/mower_recorder/fault")),
            ],
        ),
        (
            NODE_MANAGER,
            vec![
                ("robot_id", s("MW-E53A")),
                ("output_root", s(root.join("bags").to_str().unwrap())),
                ("config_file", s(share.join("record_topics.yaml").to_str().unwrap())),
                ("qos_overrides_path", s(share.join("qos_overrides.yaml").to_str().unwrap())),
                ("autostart", P::Bool(false)),
                ("params_dump_nodes", P::StringArray(vec!["/boustrophedon_coverage".into(), "/map_manage".into()])),
                ("gps_topic", s("/fix")),
                ("fault_topic", s("/mower_recorder/fault")),
                ("video_api_url", s(video.unwrap_or(""))),
                ("min_free_mb", P::Integer(2048)),
            ],
        ),
        (
            NODE_STORE,
            vec![
                ("robot_id", s("MW-E53A")),
                ("output_root", s(root.join("bags").to_str().unwrap())),
                ("r2_env_file", s(root.join("r2.env").to_str().unwrap())),
                ("upload_policy", s("manual")),
                ("wifi_interfaces", P::StringArray(vec!["wlan0".into()])),
                ("dock_topic", s("")),
            ],
        ),
    ]
}

fn set(params: &mut [(&'static str, Params)], node: &str, key: &'static str, value: P) {
    let section = &mut params.iter_mut().find(|(n, _)| *n == node).unwrap().1;
    section.retain(|(k, _)| *k != key);
    section.push((key, value));
}

/// The module as mower_rsd runs it: three instances, each on a scope of its
/// node's name, their nodes in `ns`; with `bridged`, mower_rsd's bus bridge
/// for `--modules bag` next to them (node `/mower_bus_bridge`).
struct Bag {
    nodes: Vec<&'static str>,
    bus: Bus,
    manual: ManualClock,
    feeder: Scope,
    ms: Vec<ModuleCtx>,
    shutdown: Shutdown,
    tasks: Vec<tokio::task::JoinHandle<ModuleResult>>,
}

impl Bag {
    async fn start(ns: &str, params: &[(&'static str, Params)], bridged: bool) -> Bag {
        Bag::start_only(ns, params, bridged, &NODES).await
    }

    /// Only the instances `nodes` (a test that stands in for the others).
    async fn start_only(ns: &str, params: &[(&'static str, Params)], bridged: bool, nodes: &[&'static str]) -> Bag {
        let (clock, manual) = Clock::manual(Time::from_ns(T0_NS));
        let bus = Bus::new();
        let shutdown = Shutdown::new();
        let mut tasks = Vec::new();
        if bridged {
            let m = ModuleCtx::new(mower_bridge::NODE, shutdown.clone())
                .with_bus(bus.scope(mower_bridge::NODE))
                .with_args(mower_bridge::rsd_args(&["bag"], &[] as &[&str]))
                .with_clock(clock.clone());
            tasks.push(tokio::spawn(mower_bridge::run(context(), m)));
        }
        let mut ms = Vec::new();
        for &name in nodes {
            let node = r2r::Node::create(context(), name, ns).unwrap();
            for (key, value) in params.iter().filter(|(n, _)| *n == name).flat_map(|(_, p)| p) {
                node.params.lock().unwrap().insert(key.to_string(), Parameter::new(value.clone()));
            }
            let m = ModuleCtx::new(name, shutdown.clone()).with_bus(bus.scope(name)).with_clock(clock.clone());
            let task = match name {
                NODE_GRAPH => tokio::spawn(graph::run_node(node, m.clone())),
                NODE_MANAGER => tokio::spawn(manager::run_node(context(), node, m.clone())),
                _ => tokio::spawn(store::run_node(node, m.clone())),
            };
            tasks.push(task);
            ms.push(m);
        }
        Bag { nodes: nodes.to_vec(), feeder: bus.scope("feeder"), bus, manual, ms, shutdown, tasks }
    }

    fn m(&self, node: &str) -> &ModuleCtx {
        self.ms.iter().find(|m| m.node_name == node).unwrap()
    }

    /// Which of the three hold endpoints on the bus.
    fn serving(&self) -> Vec<&'static str> {
        let topics = self.bus.topics();
        let services = self.bus.services();
        self.nodes
            .iter()
            .copied()
            .filter(|n| {
                topics.iter().any(|t| t.publishers.iter().chain(&t.subscribers).any(|s| s == n))
                    || services.iter().any(|v| v.server.as_deref() == Some(*n))
            })
            .collect()
    }

    /// Advance the clock to the instances' first look (owner::SETTLE after
    /// their start), and on until all three serve.
    async fn until_serving(&self) {
        for _ in 0..5 {
            self.manual.advance(mower_recorder::owner::SETTLE);
            let deadline = Instant::now() + Duration::from_secs(2);
            while Instant::now() < deadline {
                if self.serving().len() == self.nodes.len() {
                    return;
                }
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }
        panic!("{:?} did not start serving: {:?}", self.nodes, self.serving());
    }

    /// Move the clock and give what is due a moment.
    async fn advance(&self, d: Duration) {
        self.manual.advance(d);
        tokio::time::sleep(Duration::from_millis(20)).await;
    }

    async fn stop(self) {
        self.shutdown.trigger();
        for task in self.tasks {
            let ended = tokio::time::timeout(WAIT, task).await.expect("an instance did not stop").expect("it panicked");
            ended.expect("it failed");
        }
        for m in &self.ms {
            m.bus.close();
        }
    }
}

/// A node spun on a thread of its own: the DDS side.
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

fn idle_status(num_topics: i64) -> String {
    format!(
        "{{\"recording\": false, \"run_id\": null, \"start_time\": null, \"elapsed_s\": 0.0, \"bag_bytes\": 0, \
         \"num_topics\": {num_topics}, \"profile\": \"default\", \"video\": false, \"stop_reason\": null}}"
    )
}

const REAR_OFF: &str = r#"{"effect": "off", "source": "bag"}"#;

async fn call(m: &Scope, name: &str) -> (bool, String) {
    let client = m.client::<Trigger::Request, Trigger::Response>(name).unwrap();
    let r = tokio::time::timeout(WAIT, client.request(&Trigger::Request {}).unwrap()).await.unwrap().unwrap();
    (r.success, r.message.clone())
}

// ---------------------------------------------------------------------------
// recorder_manager
// ---------------------------------------------------------------------------

/// The idle node: the status and the rear light as soon as it serves and
/// every 2 s; the answers of stop (a status first) and snapshot with nothing
/// recording; the fault and `/fix` read; its endpoints in the manifest.
#[test]
fn recorder_manager_idle() {
    within(async {
        let root = scratch("manager");
        // recorder_manager alone, so that only its timers are on the clock
        let bag = Bag::start_only("/e53a_manager", &production(&root, None), false, &[NODE_MANAGER]).await;
        let mut status = bag.feeder.subscribe::<Text>("/mower_recorder/status", Qos::latched()).unwrap();
        let mut rear = bag.feeder.subscribe::<Text>("/mower_base/rear_light", Qos::latched()).unwrap();
        assert!(bag.serving().is_empty(), "nothing before the first look");
        bag.until_serving().await;
        // record_topics.yaml: 32 topics, the two hidden ones counted
        assert_eq!(next("the first status", &mut status).await.data, idle_status(30));
        assert_eq!(next("the first rear light", &mut rear).await.data, REAR_OFF);
        quiet(&mut status).await;
        bag.advance(manager::STATUS_PERIOD).await;
        assert_eq!(next("the tick's status", &mut status).await.data, idle_status(30));
        assert_eq!(next("the tick's rear light", &mut rear).await.data, REAR_OFF);

        assert_eq!(call(&bag.feeder, "/mower_recorder/stop").await, (false, "目前沒有在錄製".to_string()));
        assert_eq!(next("stop's status", &mut status).await.data, idle_status(30), "_stop_recording publishes one");
        assert_eq!(next("stop's rear light", &mut rear).await.data, REAR_OFF);
        assert_eq!(call(&bag.feeder, "/mower_recorder/snapshot").await, (false, "snapshot 觸發失敗".to_string()));

        let fault = bag.feeder.publisher::<Bool>("/mower_recorder/fault", Qos::default()).unwrap();
        fault.publish(&Bool { data: true }).unwrap();
        let fix = bag.feeder.publisher::<r2r::sensor_msgs::msg::NavSatFix>("/fix", Qos::default()).unwrap();
        fix.publish(&Default::default()).unwrap();
        eventually("the fault and the fix taken", || {
            bag.bus.topics().iter().filter(|t| t.name == "/mower_recorder/fault" || t.name == "/fix").all(|t| t.published == 1)
        })
        .await;
        bag.advance(manager::STATUS_PERIOD).await;
        assert_eq!(next("still idle", &mut status).await.data, idle_status(30));
        mower_bridge::coverage::assert_in_manifest(bag.m(NODE_MANAGER));
        bag.stop().await;
        std::fs::remove_dir_all(&root).unwrap();
    });
}

/// No profile file: the Python's `config_file not found`, 0 topics; and
/// autostart (production false): 3 s after the start, refused, idle.
#[test]
fn recorder_manager_without_a_profile_and_with_autostart() {
    within(async {
        let root = scratch("autostart");
        let mut params = production(&root, None);
        set(&mut params, NODE_MANAGER, "config_file", s(""));
        set(&mut params, NODE_MANAGER, "autostart", P::Bool(true));
        let bag = Bag::start_only("/e53a_autostart", &params, false, &[NODE_MANAGER]).await;
        let mut status = bag.feeder.subscribe::<Text>("/mower_recorder/status", Qos::latched()).unwrap();
        bag.until_serving().await;
        assert_eq!(next("the status", &mut status).await.data, idle_status(0));
        bag.advance(manager::AUTOSTART_DELAY).await;
        // the autostart is refused and nothing records: the tick's status
        assert_eq!(next("the status after autostart", &mut status).await.data, idle_status(0));
        bag.stop().await;
        std::fs::remove_dir_all(&root).unwrap();
    });
}

/// A stub of MediaMTX's API: every request as (method, path, body); the
/// first `fail` PATCHes answer 500.
struct Mediamtx {
    url: String,
    requests: Arc<Mutex<Vec<(String, String, String)>>>,
}

impl Mediamtx {
    fn start(fail: usize) -> Mediamtx {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}", listener.local_addr().unwrap());
        let requests = Arc::new(Mutex::new(Vec::new()));
        let log = requests.clone();
        std::thread::spawn(move || {
            let mut failed = 0;
            for stream in listener.incoming() {
                let Ok(mut stream) = stream else { break };
                let Some((method, path, _, body)) = read_request(&mut stream) else { continue };
                log.lock().unwrap().push((method.clone(), path, String::from_utf8_lossy(&body).into_owned()));
                let status = if method == "PATCH" && failed < fail {
                    failed += 1;
                    "500 Internal Server Error"
                } else {
                    "200 OK"
                };
                let _ = write!(stream, "HTTP/1.1 {status}\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
            }
        });
        Mediamtx { url, requests }
    }

    fn patches(&self) -> Vec<(String, String)> {
        self.requests.lock().unwrap().iter().filter(|r| r.0 == "PATCH").map(|r| (r.1.clone(), r.2.clone())).collect()
    }
}

/// One HTTP/1.1 request: method, path, headers (lower-case names), body.
fn read_request(stream: &mut std::net::TcpStream) -> Option<(String, String, HashMap<String, String>, Vec<u8>)> {
    let mut reader = BufReader::new(stream.try_clone().ok()?);
    let mut line = String::new();
    reader.read_line(&mut line).ok()?;
    let mut parts = line.split_whitespace();
    let (method, path) = (parts.next()?.to_string(), parts.next()?.to_string());
    let mut headers = HashMap::new();
    loop {
        let mut h = String::new();
        reader.read_line(&mut h).ok()?;
        let h = h.trim_end();
        if h.is_empty() {
            break;
        }
        if let Some((k, v)) = h.split_once(':') {
            headers.insert(k.trim().to_lowercase(), v.trim().to_string());
        }
    }
    let len: usize = headers.get("content-length").and_then(|v| v.parse().ok()).unwrap_or(0);
    let mut body = vec![0; len];
    reader.read_exact(&mut body).ok()?;
    Some((method, path, headers, body))
}

/// MediaMTX: `record: false` before the node serves (a recorder that died
/// mid-run must not leave the camera recording), again on each tick while it
/// fails, and never once it worked; stop sends none while video is off.
#[test]
fn recorder_manager_turns_mediamtx_s_recording_off_until_it_worked() {
    within(async {
        let root = scratch("video");
        let api = Mediamtx::start(3);
        let bag = Bag::start_only("/e53a_video", &production(&root, Some(&api.url)), false, &[NODE_MANAGER]).await;
        let mut status = bag.feeder.subscribe::<Text>("/mower_recorder/status", Qos::latched()).unwrap();
        bag.until_serving().await;
        next("the first status", &mut status).await;
        let off = ("/v3/config/paths/patch/front".to_string(), r#"{"record": false}"#.to_string());
        // __init__'s PATCH, then the first _publish_status's retry: both 500
        assert_eq!(api.patches(), [off.clone(), off.clone()], "the start's two PATCHes");
        bag.advance(manager::STATUS_PERIOD).await;
        next("the tick's status", &mut status).await;
        assert_eq!(api.patches().len(), 3, "retried on the tick (500 again)");
        bag.advance(manager::STATUS_PERIOD).await;
        next("the tick's status", &mut status).await;
        assert_eq!(api.patches().len(), 4, "retried, and this one worked");
        bag.advance(manager::STATUS_PERIOD).await;
        next("the tick's status", &mut status).await;
        assert_eq!(call(&bag.feeder, "/mower_recorder/stop").await.0, false);
        next("stop's status", &mut status).await;
        assert_eq!(api.patches().len(), 4, "nothing more once off");
        assert!(api.patches().iter().all(|p| *p == off));
        let headers_ok = api.requests.lock().unwrap().iter().all(|r| r.0 == "PATCH");
        assert!(headers_ok, "PATCHes only: no readiness GET while idle");
        bag.stop().await;
        std::fs::remove_dir_all(&root).unwrap();
    });
}

/// The topics of a node as the graph shows them, by node.
fn subscribed_by(peer: &Peer, topic: &str) -> Vec<String> {
    peer.with(|n| n.get_subscriptions_info_by_topic(topic, false).unwrap()).into_iter().map(|e| e.node_name).collect()
}

fn reliable(depth: usize) -> QosProfile {
    QosProfile::default().keep_last(depth).reliable()
}

/// What rmw puts on the wire, and so in the bag: the CDR padded to 4.
fn on_the_wire(text: &str) -> Vec<u8> {
    use r2r::WrappedTypesupport;
    let mut b = Text { data: text.into() }.to_serialized_bytes().unwrap();
    while b.len() % 4 != 0 {
        b.push(0);
    }
    b
}

/// Every message of an MCAP file: (topic, log time, publish time, payload
/// with per-message zstd undone).
fn messages(path: &Path) -> Vec<(String, u64, u64, Vec<u8>)> {
    let buf = std::fs::read(path).unwrap();
    mcap::MessageStream::new(&buf)
        .unwrap()
        .map(|m| {
            let m = m.unwrap();
            let data = if m.data.starts_with(&[0x28, 0xb5, 0x2f, 0xfd]) { zstd::decode_all(&m.data[..]).unwrap() } else { m.data.to_vec() };
            (m.channel.topic.clone(), m.log_time, m.publish_time, data)
        })
        .collect()
}

/// E5-3b: a run recorded from DDS. The start's reply and status; the groups'
/// nodes (`mower_full_recorder`, `mower_snapshot_recorder`) on the graph while
/// it runs and their subscriptions, as rosbag2's (D12); a latched sample
/// written before the start (the reader adapted to its offer, transient
/// local); every message's bytes as the writer sent them (per-message zstd
/// undone), its log time after its publish time; a hidden name and an
/// unpublished one not recorded; the snapshot group flushed by
/// `/mower_recorder/snapshot`; run_metadata.yaml and the parameter dump
/// (`ros2 param dump`'s format, from the node's own parameter services);
/// the stop's reply, and the nodes gone.
#[test]
fn recorder_manager_records_a_run_from_dds() {
    within(async {
        let root = scratch("record");
        let profile = root.join("profile.yaml");
        std::fs::write(
            &profile,
            "topics:\n  - /e53b/text\n  - /e53b/latched\n  - /e53b/nobody\n  - /e53b/_hidden/x\nsnapshot_topics:\n  - /e53b/snap\n",
        )
        .unwrap();
        let mut params = production(&root, None);
        set(&mut params, NODE_MANAGER, "config_file", s(profile.to_str().unwrap()));
        set(&mut params, NODE_MANAGER, "params_dump_nodes", P::StringArray(vec!["/e53b_dumped".into()]));
        let writer = Peer::new("e53b_writer", "");
        // depth 100: a reliable keep-last writer may drop what its readers
        // have not acknowledged when a burst outgrows its depth (DDS, for
        // rosbag2 too), so the burst of 20 below stays under it
        let text = writer.with(|n| n.create_publisher::<Text>("/e53b/text", reliable(100)).unwrap());
        let latched = writer.with(|n| n.create_publisher::<Text>("/e53b/latched", reliable(1).transient_local()).unwrap());
        let snap = writer.with(|n| n.create_publisher::<Text>("/e53b/snap", reliable(100)).unwrap());
        let hidden = writer.with(|n| n.create_publisher::<Text>("/e53b/_hidden/x", reliable(10)).unwrap());
        latched.publish(&Text { data: "before the start".into() }).unwrap();
        // the dumped node, with rclcpp-like parameter services (r2r's handler)
        let dumped = Peer::new("e53b_dumped", "");
        let handler = dumped.with(|n| {
            let mut p = n.params.lock().unwrap();
            p.insert("a".into(), Parameter::new(P::Integer(1)));
            p.insert("b.c".into(), Parameter::new(s("x")));
            drop(p);
            n.make_parameter_handler().unwrap().0
        });
        let handler = tokio::spawn(handler);
        let bag = Bag::start_only("/e53b_rec", &params, false, &[NODE_MANAGER]).await;
        let mut status = bag.feeder.subscribe::<Text>("/mower_recorder/status", Qos::latched().keep_last(100)).unwrap();
        bag.until_serving().await;
        // four names listed, the hidden one not counted (num_topics, 2026-10-31)
        assert_eq!(next("the idle status", &mut status).await.data, idle_status(3));

        let (ok, reply) = call(&bag.feeder, "/mower_recorder/start").await;
        let run = reply.strip_prefix("開始錄製 run=").unwrap_or_else(|| panic!("{reply}")).to_string();
        assert!(ok && run.starts_with("MW-E53A_"), "{reply}");
        let run_dir = root.join("bags").join(&run);
        let started = next("the start's status", &mut status).await.data.clone();
        assert!(started.starts_with(&format!(r#"{{"recording": true, "run_id": "{run}", "start_time": "#)), "{started}");
        // D12: the groups' nodes, and the names subscribed on them
        let names = || writer.with(|n| n.get_node_names_and_namespaces().unwrap()).into_iter().map(|(n, _)| n).collect::<Vec<_>>();
        eventually("the groups' nodes", || ["mower_full_recorder", "mower_snapshot_recorder"].iter().all(|n| names().contains(&n.to_string()))).await;
        eventually("the subscriptions", || {
            subscribed_by(&writer, "/e53b/text") == ["mower_full_recorder"]
                && subscribed_by(&writer, "/e53b/latched") == ["mower_full_recorder"]
                && subscribed_by(&writer, "/e53b/snap") == ["mower_snapshot_recorder"]
        })
        .await;
        assert!(subscribed_by(&writer, "/e53b/_hidden/x").is_empty(), "hidden names are not recorded");
        // the writers have matched their readers (volatile: nothing earlier reaches them)
        eventually("the matches", || {
            text.get_inter_process_subscription_count().unwrap() == 1 && snap.get_inter_process_subscription_count().unwrap() == 1
        })
        .await;
        for i in 0..20 {
            text.publish(&Text { data: format!("#{i}") }).unwrap();
            snap.publish(&Text { data: format!("snap #{i}") }).unwrap();
            hidden.publish(&Text { data: "hidden".into() }).unwrap();
        }
        eventually("the parameter dump", || run_dir.join("params/e53b_dumped.yaml").is_file()).await;
        bag.advance(manager::STATUS_PERIOD).await;
        let tick = next("the tick's status", &mut status).await.data.clone();
        assert!(tick.contains(r#""recording": true"#) && tick.contains(r#""num_topics": 3"#), "{tick}");
        tokio::time::sleep(Duration::from_millis(300)).await;
        assert_eq!(call(&bag.feeder, "/mower_recorder/snapshot").await, (true, "已觸發 snapshot".to_string()));
        assert_eq!(call(&bag.feeder, "/mower_recorder/stop").await, (true, format!("已停止並封存: full, snapshot (run={run})")));
        eventually("the groups' nodes to go", || !names().iter().any(|n| n.starts_with("mower_") && n.ends_with("_recorder"))).await;

        // the bags
        let full = messages(&run_dir.join("bag/bag_0.mcap"));
        let of = |t: &str| full.iter().filter(|m| m.0 == t).collect::<Vec<_>>();
        assert_eq!(of("/e53b/latched").iter().map(|m| m.3.clone()).collect::<Vec<_>>(), [on_the_wire("before the start")]);
        let texts: Vec<Vec<u8>> = of("/e53b/text").iter().map(|m| m.3.clone()).collect();
        assert_eq!(texts, (0..20).map(|i| on_the_wire(&format!("#{i}"))).collect::<Vec<_>>());
        assert!(full.iter().all(|m| m.1 >= m.2), "log time after publish time");
        assert!(full.iter().all(|m| m.0 == "/e53b/text" || m.0 == "/e53b/latched"), "no hidden, no unpublished name");
        let snaps = messages(&run_dir.join("snapshots/snapshots_0.mcap"));
        assert_eq!(snaps.len(), 20, "the snapshot ring flushed once");
        assert!(run_dir.join("bag/metadata.yaml").is_file() && run_dir.join("snapshots/metadata.yaml").is_file());
        let meta = std::fs::read_to_string(run_dir.join("run_metadata.yaml")).unwrap();
        assert!(meta.starts_with(&format!("run_id: {run}\nrobot_id: MW-E53A\nstart_time: ")), "{meta}");
        assert!(meta.contains("recorded_topics:\n- /e53b/text\n- /e53b/latched\n- /e53b/nobody\n- /e53b/_hidden/x\n"), "{meta}");
        let dump = std::fs::read_to_string(run_dir.join("params/e53b_dumped.yaml")).unwrap();
        assert!(dump.starts_with("/e53b_dumped:\n  ros__parameters:\n"), "{dump}");
        assert!(dump.contains("    a: 1\n") && dump.contains("    b:\n      c: x\n"), "{dump}");
        let after = next("the stop's status", &mut status).await.data.clone();
        assert!(after.starts_with(r#"{"recording": false, "run_id": null"#), "{after}");
        handler.abort();
        bag.stop().await;
        std::fs::remove_dir_all(&root).unwrap();
    });
}

/// E5-5: data_collection by the request file (design 4.7, D7), as
/// mower-data-collection.sh writes it: the manager takes the profile while
/// idle and says so in the status; the camera group records the camera
/// topics from DDS (gscam stays outside); run_metadata.yaml is spec 1.6's,
/// with the camera block the launch would have passed (the URDF from
/// robot_state_publisher's latched `/robot_description`), and `calib/` the
/// calibration's copy; the request's going brings the default back.
#[test]
fn recorder_manager_takes_data_collection_from_the_request_file() {
    within(async {
        let root = scratch("dc");
        let golden = repo().join("deploy/test/golden/recorder/F3");
        let run = golden.join("MW-E5FIX_20261030T080906");
        let calib = root.join("calib");
        std::fs::create_dir_all(&calib).unwrap();
        std::fs::copy(run.join("calib/camera_front.yaml"), calib.join("camera_front.yaml")).unwrap();
        std::fs::copy(run.join("calib/extrinsics.yaml"), calib.join("extrinsics.yaml")).unwrap();
        let request = root.join("recorder_profile.json");
        let pipeline = "videotestsrc is-live=true ! image/jpeg,width=80,height=60,framerate=25/1";
        std::fs::write(
            &request,
            format!(
                r#"{{"profile": "data_collection", "camera": {{"camera_size": "80x60", "gscam_pipeline": "{pipeline}",
                    "calib_dir": "{}"}}, "session_file": "{}", "run_info": "{{weather: cloudy}}", "upload_policy": "manual"}}"#,
                calib.display(),
                golden.join("session.yaml").display()
            ),
        )
        .unwrap();
        let mut params = production(&root, None);
        set(&mut params, NODE_MANAGER, "profile_request_file", s(request.to_str().unwrap()));
        // robot_state_publisher's URDF, latched
        let urdf: serde_json::Value = serde_json::from_str(
            &std::fs::read_to_string(repo().join("src/mower_rs/crates/mower_recorder_core/tests/vectors/extrinsics.json")).unwrap(),
        )
        .unwrap();
        let rsp = Peer::new("e55_robot_state_publisher", "");
        let description = rsp.with(|n| n.create_publisher::<Text>("/robot_description", reliable(1).transient_local()).unwrap());
        description.publish(&Text { data: urdf["texts"]["real_robot_urdf"].as_str().unwrap().to_string() }).unwrap();
        let camera = Peer::new("e55_camera_front", "");
        let image = camera.with(|n| {
            n.create_publisher::<r2r::sensor_msgs::msg::CompressedImage>("/camera/front/image_raw/compressed", reliable(100)).unwrap()
        });
        let info = camera.with(|n| n.create_publisher::<r2r::sensor_msgs::msg::CameraInfo>("/camera/front/camera_info", reliable(100)).unwrap());

        let bag = Bag::start_only("/e55_dc", &params, false, &[NODE_MANAGER]).await;
        let mut status = bag.feeder.subscribe::<Text>("/mower_recorder/status", Qos::latched().keep_last(100)).unwrap();
        bag.until_serving().await;
        let mut seen = String::new();
        let deadline = Instant::now() + WAIT;
        while !seen.contains(r#""profile": "data_collection""#) {
            assert!(Instant::now() < deadline, "no data_collection status: {seen}");
            seen = next("a status", &mut status).await.data.clone();
        }

        let (ok, reply) = call(&bag.feeder, "/mower_recorder/start").await;
        let run_id = reply.strip_prefix("開始錄製 run=").unwrap_or_else(|| panic!("{reply}")).to_string();
        assert!(ok);
        let run_dir = root.join("bags").join(&run_id);
        eventually("the camera group's reader", || subscribed_by(&camera, "/camera/front/image_raw/compressed") == ["mower_camera_recorder"]).await;
        eventually("the matches", || {
            image.get_inter_process_subscription_count().unwrap() == 1 && info.get_inter_process_subscription_count().unwrap() == 1
        })
        .await;
        for i in 0..5u32 {
            let mut m = r2r::sensor_msgs::msg::CompressedImage::default();
            m.header.frame_id = "camera_front_optical_frame".into();
            m.header.stamp.sec = 1_793_000_000 + i as i32;
            m.format = "jpeg".into();
            m.data = vec![0xff, 0xd8, i as u8, 0xff, 0xd9];
            image.publish(&m).unwrap();
            let mut c = r2r::sensor_msgs::msg::CameraInfo::default();
            c.header = m.header.clone();
            info.publish(&c).unwrap();
        }
        tokio::time::sleep(Duration::from_millis(500)).await;
        let (ok, reply) = call(&bag.feeder, "/mower_recorder/stop").await;
        assert!(ok && reply == format!("已停止並封存: full, camera (run={run_id})"), "{reply}");

        let images = messages(&run_dir.join("camera/camera_0.mcap"));
        assert_eq!(images.iter().filter(|m| m.0 == "/camera/front/image_raw/compressed").count(), 5);
        assert_eq!(images.iter().filter(|m| m.0 == "/camera/front/camera_info").count(), 5);
        let meta = std::fs::read_to_string(run_dir.join("run_metadata.yaml")).unwrap();
        assert!(meta.contains("\nprofile: data_collection\ncamera:\n"), "{meta}");
        for line in [
            "  device: /dev/video0\n  width: 80\n  height: 60\n  fps_native: 25\n  fps_recorded: 10\n  format: jpeg\n  autofocus: 'off'\n",
            "  driver: gscam\n",
            "camera_height_m: 0.585\ncamera_pitch_deg: 17.5\nextrinsics_source: placeholder\n",
            "weather: cloudy\n",
        ] {
            assert!(meta.contains(line), "{line:?} in {meta}");
        }
        let golden_derived = std::fs::read_to_string(run.join("calib/extrinsics.yaml")).unwrap();
        let ours = std::fs::read_to_string(run_dir.join("calib/extrinsics.yaml")).unwrap();
        let derived = |t: &str| t.split("\nderived:\n").nth(1).map(str::to_string);
        assert_eq!(derived(&ours), derived(&golden_derived), "the derived block: the URDF's reference");
        assert!(run_dir.join("calib/camera_front.yaml").is_file());

        // the request gone: the default profile again
        std::fs::remove_file(&request).unwrap();
        let deadline = Instant::now() + WAIT;
        loop {
            bag.advance(manager::STATUS_PERIOD).await;
            let latest = next("a status", &mut status).await.data.clone();
            if latest.contains(r#""profile": "default""#) {
                break;
            }
            assert!(Instant::now() < deadline, "the default did not come back: {latest}");
        }
        bag.stop().await;
        std::fs::remove_dir_all(&root).unwrap();
    });
}

// ---------------------------------------------------------------------------
// graph_snapshot
// ---------------------------------------------------------------------------

/// A tick's JSON: the stamp on the module's clock, the graph's nodes sorted
/// (its own among them), topics and services by name with their types and
/// counts; then the diff: a node that comes, a node that goes; a watched
/// name without its slash never faults. (The A/B's on-run has the three
/// instances' nodes in it: e2-verify scripts/e5_probe.py.)
#[test]
fn graph_snapshot_ticks_and_diffs() {
    within(async {
        let root = scratch("graph");
        let mut params = production(&root, None);
        // one watched name as production writes them (no slash), one with it
        set(&mut params, NODE_GRAPH, "watch_nodes", P::StringArray(vec!["e53a_watched".into(), "/e53a_g/e53a_watched_too".into()]));
        // graph_snapshot alone: its timer is the only one on the clock
        let bag = Bag::start_only("/e53a_g", &params, false, &[NODE_GRAPH]).await;
        let mut snap = bag.feeder.subscribe::<Text>("/graph_snapshot", Qos::default().keep_last(100)).unwrap();
        let mut events = bag.feeder.subscribe::<Text>("/graph_events", Qos::default().keep_last(100)).unwrap();
        let mut fault = bag.feeder.subscribe::<Bool>("/mower_recorder/fault", Qos::default().keep_last(100)).unwrap();
        let watched = Peer::new("e53a_watched", "/e53a_g");
        let watched_too = Peer::new("e53a_watched_too", "/e53a_g");
        let topic_peer = Peer::new("e53a_topics", "/e53a_g");
        let _w = topic_peer.with(|n| n.create_publisher::<Text>("/e53a_g/topic", QosProfile::default()).unwrap());
        let _r = topic_peer.with(|n| n.subscribe::<Text>("/e53a_g/topic", QosProfile::default()).unwrap());
        let _r2 = watched.with(|n| n.subscribe::<Text>("/e53a_g/topic", QosProfile::default()).unwrap());
        bag.until_serving().await;
        quiet(&mut snap).await;
        bag.advance(Duration::from_secs(2)).await;
        let first = next("the first snapshot", &mut snap).await.data.clone();
        let v: serde_json_lite::Value = serde_json_lite::parse(&first);
        let (sec, nanosec) = bag.m(NODE_GRAPH).clock.now().to_parts();
        assert!(first.starts_with(&format!("{{\"stamp\": {{\"sec\": {sec}, \"nanosec\": {nanosec}}}, \"nodes\": [")), "{first}");
        let nodes = v.strings("nodes");
        for n in ["/e53a_g/graph_snapshot", "/e53a_g/e53a_watched", "/e53a_g/e53a_topics"] {
            assert!(nodes.contains(&n.to_string()), "{n} in {nodes:?}");
        }
        let mut sorted = nodes.clone();
        sorted.sort();
        assert_eq!(nodes, sorted);
        assert!(first.contains(r#"{"name": "/e53a_g/topic", "types": ["std_msgs/msg/String"], "pubs": 1, "subs": 2}"#), "{first}");
        let topic_names = v.names("topics");
        let mut by_name = topic_names.clone();
        by_name.sort();
        assert_eq!(topic_names, by_name, "rmw's order");
        let services = v.names("services");
        let mut by_name = services.clone();
        by_name.sort();
        assert_eq!(services, by_name);
        quiet(&mut events).await;

        // a node comes and the watched ones go
        let newcomer = Peer::new("e53a_newcomer", "/e53a_g");
        drop(watched);
        drop(watched_too);
        eventually("the graph to show it", || {
            let all = topic_peer.with(|n| n.get_node_names_and_namespaces().unwrap());
            all.iter().any(|(n, _)| n == "e53a_newcomer") && !all.iter().any(|(n, _)| n.starts_with("e53a_watched"))
        })
        .await;
        bag.advance(Duration::from_secs(2)).await;
        next("the second snapshot", &mut snap).await;
        let mut got = Vec::new();
        for _ in 0..3 {
            got.push(next("an event", &mut events).await.data.clone());
        }
        assert_eq!(
            got,
            [
                r#"{"event": "node_disappeared", "target": "/e53a_g/e53a_watched"}"#,
                r#"{"event": "node_disappeared", "target": "/e53a_g/e53a_watched_too"}"#,
                r#"{"event": "node_appeared", "target": "/e53a_g/e53a_newcomer"}"#,
            ]
        );
        // the slashed name is watched; the production-style one is not
        assert!(next("the fault", &mut fault).await.data);
        quiet(&mut fault).await;
        drop(newcomer);
        mower_bridge::coverage::assert_in_manifest(bag.m(NODE_GRAPH));
        bag.stop().await;
        std::fs::remove_dir_all(&root).unwrap();
    });
}

/// Just enough JSON reading for the snapshot's lists.
mod serde_json_lite {
    pub struct Value(pub mower_pyfmt::Py);
    pub fn parse(s: &str) -> Value {
        Value(mower_pyfmt::json::loads(s).unwrap())
    }
    impl Value {
        pub fn strings(&self, key: &str) -> Vec<String> {
            match self.0.get(key) {
                Some(mower_pyfmt::Py::List(l)) => {
                    l.iter().filter_map(|v| if let mower_pyfmt::Py::Str(s) = v { Some(s.clone()) } else { None }).collect()
                }
                _ => Vec::new(),
            }
        }
        pub fn names(&self, key: &str) -> Vec<String> {
            match self.0.get(key) {
                Some(mower_pyfmt::Py::List(l)) => l
                    .iter()
                    .filter_map(|v| match v.get("name") {
                        Some(mower_pyfmt::Py::Str(s)) => Some(s.clone()),
                        _ => None,
                    })
                    .collect(),
                _ => Vec::new(),
            }
        }
    }
}

// ---------------------------------------------------------------------------
// bag_store
// ---------------------------------------------------------------------------

/// A run directory as recorder_manager leaves it after a stop: metadata and
/// one finalized bag.
fn make_run(root: &Path, run_id: &str) {
    let dir = root.join(run_id);
    std::fs::create_dir_all(dir.join("bag")).unwrap();
    std::fs::write(
        dir.join("run_metadata.yaml"),
        format!("run_id: {run_id}\nrobot_id: MW-E53A\nstart_time: '2026-10-30T10:00:00.5'\nros_distro: jazzy\n"),
    )
    .unwrap();
    std::fs::write(dir.join("bag/bag_0.mcap"), b"\x89MCAP0\r\n not really, but the uploader only hashes it").unwrap();
    std::fs::write(
        dir.join("bag/metadata.yaml"),
        "rosbag2_bagfile_information:\n  version: 9\n  storage_identifier: mcap\n  duration:\n    nanoseconds: 1\n  \
         starting_time:\n    nanoseconds_since_epoch: 1793000000000000000\n  message_count: 0\n  \
         topics_with_message_count:\n    []\n  compression_format: \"\"\n  compression_mode: \"\"\n  \
         relative_file_paths:\n    - bag_0.mcap\n  files:\n    - path: bag_0.mcap\n      starting_time:\n        \
         nanoseconds_since_epoch: 1793000000000000000\n      duration:\n        nanoseconds: 1\n      message_count: 0\n  \
         custom_data: ~\n  ros_distro: jazzy",
    )
    .unwrap();
}

/// The list at the start and every 5 s; refresh, rename, delete (the run
/// being recorded refused), an unknown action, a message that is not JSON,
/// and one the Python would not answer.
#[test]
fn bag_store_lists_and_answers_commands() {
    within(async {
        let root = scratch("store");
        let bags_dir = root.join("bags");
        make_run(&bags_dir, "MW-E53A_20261030T100000");
        make_run(&bags_dir, "MW-E53A_20261030T110000");
        std::fs::create_dir_all(bags_dir.join("lost+found")).unwrap();
        // bag_store alone: the test writes the status recorder_manager would
        let bag = Bag::start_only("/e53a_store", &production(&root, None), false, &[NODE_STORE]).await;
        let mut bags = bag.feeder.subscribe::<Text>("/mower_recorder/bags", Qos::latched()).unwrap();
        let mut result = bag.feeder.subscribe::<Text>("/mower_recorder/command_result", Qos::default()).unwrap();
        let commands = bag.feeder.publisher::<Text>("/mower_recorder/command", Qos::default()).unwrap();
        bag.until_serving().await;
        let first = next("the first list", &mut bags).await.data.clone();
        assert!(first.starts_with(r#"{"robot_id": "MW-E53A", "network_ok": false, "r2_configured": false, "bags": [{"run_id": "MW-E53A_20261030T100000""#), "{first}");
        assert!(first.contains(r#""run_id": "lost+found", "display_name": "lost+found", "robot_id": "MW-E53A", "start_time": """#), "every directory: {first}");
        bag.advance(Duration::from_secs(5)).await;
        assert_eq!(next("the 5 s list", &mut bags).await.data.as_str(), first);

        let send = |text: &str| commands.publish(&Text { data: text.to_string() }).unwrap();
        send(r#"{"action": "refresh", "req_id": "app_1_1"}"#);
        next("refresh's list", &mut bags).await;
        assert_eq!(next("refresh", &mut result).await.data, r#"{"req_id": "app_1_1", "ok": true, "message": "refreshed"}"#);

        send(r#"{"action": "rename", "run_id": "MW-E53A_20261030T100000", "new_name": "  草地 A  ", "req_id": "app_1_2"}"#);
        let renamed = next("rename's list", &mut bags).await.data.clone();
        assert!(renamed.contains(r#""display_name": "草地 A""#), "{renamed}");
        assert_eq!(next("rename", &mut result).await.data, r#"{"req_id": "app_1_2", "ok": true, "message": "已更名為「草地 A」"}"#);
        let meta = std::fs::read_to_string(bags_dir.join("MW-E53A_20261030T100000/run_metadata.yaml")).unwrap();
        assert!(meta.ends_with("ros_distro: jazzy\ndisplay_name: 草地 A\n"), "{meta}");
        send(r#"{"action": "rename", "run_id": "nope", "new_name": "x", "req_id": 7}"#);
        assert_eq!(next("rename nope", &mut result).await.data, r#"{"req_id": 7, "ok": false, "message": "找不到 run: nope"}"#);
        quiet(&mut bags).await;

        // the run being recorded cannot go
        let status = bag.feeder.publisher::<Text>("/mower_recorder/status", Qos::latched()).unwrap();
        status
            .publish(&Text { data: r#"{"recording": true, "run_id": "MW-E53A_20261030T110000"}"#.into() })
            .unwrap();
        eventually("the status taken", || bag.bus.topics().iter().any(|t| t.name == "/mower_recorder/status" && t.published == 1)).await;
        bag.advance(Duration::from_secs(5)).await;
        let listed = next("the list with the recording run", &mut bags).await.data.clone();
        assert!(listed.contains(r#""run_id": "MW-E53A_20261030T110000", "display_name": "MW-E53A_20261030T110000", "robot_id": "MW-E53A", "start_time": "2026-10-30T10:00:00.5""#));
        assert!(listed.contains(r#""uploading": false, "recording": true"#), "{listed}");
        send(r#"{"action": "delete", "run_id": "MW-E53A_20261030T110000", "req_id": "app_1_3"}"#);
        assert_eq!(next("delete refused", &mut result).await.data, r#"{"req_id": "app_1_3", "ok": false, "message": "這個 run 正在錄製中,無法刪除"}"#);
        send(r#"{"action": "delete", "run_id": "MW-E53A_20261030T100000", "req_id": "app_1_4"}"#);
        next("delete's list", &mut bags).await;
        assert_eq!(next("delete", &mut result).await.data, r#"{"req_id": "app_1_4", "ok": true, "message": "已刪除 MW-E53A_20261030T100000"}"#);
        assert!(!bags_dir.join("MW-E53A_20261030T100000").exists());
        send(r#"{"action": "delete", "run_id": "../x", "req_id": "app_1_5"}"#);
        assert_eq!(next("delete ../x", &mut result).await.data, r#"{"req_id": "app_1_5", "ok": false, "message": "找不到 run: ../x"}"#);

        send(r#"{"action": "fly", "req_id": "app_1_6"}"#);
        assert_eq!(next("unknown", &mut result).await.data, r#"{"req_id": "app_1_6", "ok": false, "message": "unknown action: fly"}"#);
        send("not json");
        assert_eq!(
            next("bad json", &mut result).await.data,
            r#"{"req_id": "", "ok": false, "message": "bad command json: Expecting value: line 1 column 1 (char 0)"}"#
        );
        send("[1, 2]");
        quiet(&mut result).await;
        mower_bridge::coverage::assert_in_manifest(bag.m(NODE_STORE));
        bag.stop().await;
        std::fs::remove_dir_all(&root).unwrap();
    });
}

/// A minimal S3: objects by path, PUT, HEAD, GET, DELETE and ListObjectsV2;
/// every request kept as `METHOD path`.
struct S3Stub {
    url: String,
    objects: Arc<Mutex<HashMap<String, Vec<u8>>>>,
    log: Arc<Mutex<Vec<String>>>,
}

impl S3Stub {
    fn start() -> S3Stub {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}", listener.local_addr().unwrap());
        let objects: Arc<Mutex<HashMap<String, Vec<u8>>>> = Arc::default();
        let log: Arc<Mutex<Vec<String>>> = Arc::default();
        let (objs, requests) = (objects.clone(), log.clone());
        std::thread::spawn(move || {
            for stream in listener.incoming() {
                let Ok(mut stream) = stream else { break };
                let Some((method, target, _, body)) = read_request(&mut stream) else { continue };
                let (path, query) = target.split_once('?').unwrap_or((&target, ""));
                requests.lock().unwrap().push(format!("{method} {path}"));
                let mut objs = objs.lock().unwrap();
                let (status, headers, payload): (&str, String, Vec<u8>) = match method.as_str() {
                    "PUT" => {
                        objs.insert(path.to_string(), body);
                        ("200 OK", "ETag: \"x\"\r\n".into(), Vec::new())
                    }
                    "HEAD" => match objs.get(path) {
                        Some(b) => ("200 OK", format!("Content-Length: {}\r\n", b.len()), Vec::new()),
                        None => ("404 Not Found", String::new(), Vec::new()),
                    },
                    "DELETE" => {
                        objs.remove(path);
                        ("204 No Content", String::new(), Vec::new())
                    }
                    "GET" if query.contains("list-type=2") => {
                        let prefix = query
                            .split('&')
                            .find_map(|kv| kv.strip_prefix("prefix="))
                            .map(mower_recorder_core::s3::unquote)
                            .unwrap_or_default();
                        let bucket = path.trim_start_matches('/');
                        let mut keys: Vec<String> = objs
                            .keys()
                            .filter_map(|p| p.strip_prefix(&format!("/{bucket}/")).map(str::to_string))
                            .filter(|k| k.starts_with(&prefix))
                            .collect();
                        keys.sort();
                        let contents: String = keys.iter().map(|k| format!("<Contents><Key>{k}</Key></Contents>")).collect();
                        let xml = format!("<ListBucketResult><IsTruncated>false</IsTruncated>{contents}</ListBucketResult>");
                        ("200 OK", String::new(), xml.into_bytes())
                    }
                    "GET" => match objs.get(path) {
                        Some(b) => ("200 OK", String::new(), b.clone()),
                        None => ("404 Not Found", String::new(), Vec::new()),
                    },
                    _ => ("400 Bad Request", String::new(), Vec::new()),
                };
                drop(objs);
                let length = if method == "HEAD" { String::new() } else { format!("Content-Length: {}\r\n", payload.len()) };
                let _ = write!(stream, "HTTP/1.1 {status}\r\n{headers}{length}Connection: close\r\n\r\n");
                let _ = stream.write_all(&payload);
            }
        });
        S3Stub { url, objects, log }
    }

    fn keys(&self) -> Vec<String> {
        let mut k: Vec<String> = self.objects.lock().unwrap().keys().cloned().collect();
        k.sort();
        k
    }
}

/// `upload_now` sends a run past the network gate (policy manual), the
/// manifest last and the marker after it, and the list says uploaded; a
/// delete of that run takes its objects off R2, the manifest first. With
/// policy `always` the 30 s tick does it on its own.
#[test]
fn bag_store_uploads_and_deletes_on_r2() {
    within(async {
        let root = scratch("upload");
        let bags_dir = root.join("bags");
        make_run(&bags_dir, "MW-E53A_20261030T100000");
        let s3 = S3Stub::start();
        std::fs::write(
            root.join("r2.env"),
            "R2_ACCOUNT_ID=acct\nR2_BUCKET=mower-bags\nR2_ACCESS_KEY_ID=AKID\nR2_SECRET_ACCESS_KEY='secret'\n",
        )
        .unwrap();
        let mut params = production(&root, None);
        set(&mut params, NODE_STORE, "r2_endpoint_url", s(&s3.url));
        let bag = Bag::start_only("/e53a_upload", &params, false, &[NODE_STORE]).await;
        let mut bags = bag.feeder.subscribe::<Text>("/mower_recorder/bags", Qos::latched().keep_last(100)).unwrap();
        let mut result = bag.feeder.subscribe::<Text>("/mower_recorder/command_result", Qos::default()).unwrap();
        let commands = bag.feeder.publisher::<Text>("/mower_recorder/command", Qos::default()).unwrap();
        bag.until_serving().await;
        let run = "MW-E53A_20261030T100000";
        assert!(next("the first list", &mut bags).await.data.contains(r#""r2_configured": true"#));
        commands.publish(&Text { data: r#"{"action": "upload_now", "req_id": "u1"}"#.into() }).unwrap();
        assert_eq!(next("upload_now", &mut result).await.data, r#"{"req_id": "u1", "ok": true, "message": "已排入上傳"}"#);
        // the list while it goes, then after
        let during = next("the list during the upload", &mut bags).await.data.clone();
        assert!(during.contains(r#""uploaded": false, "uploading": true"#), "{during}");
        let after = next("the list after it", &mut bags).await.data.clone();
        assert!(after.contains(r#""uploaded": true, "uploading": false"#), "{after}");
        let base = format!("/mower-bags/bags/MW-E53A/{run}");
        assert_eq!(
            s3.keys(),
            [format!("{base}/_manifest.json"), format!("{base}/bag/bag_0.mcap"), format!("{base}/bag/metadata.yaml"), format!("{base}/run_metadata.yaml")]
        );
        let puts: Vec<String> = s3.log.lock().unwrap().iter().filter(|l| l.starts_with("PUT ")).cloned().collect();
        assert_eq!(puts.last().unwrap(), &format!("PUT {base}/_manifest.json"), "the manifest last");
        assert!(bags_dir.join(run).join(".uploaded").is_file());

        commands.publish(&Text { data: format!(r#"{{"action": "delete", "run_id": "{run}", "req_id": "d1"}}"#) }).unwrap();
        assert_eq!(next("delete", &mut result).await.data, format!(r#"{{"req_id": "d1", "ok": true, "message": "已刪除 {run}"}}"#));
        assert!(s3.keys().is_empty(), "R2 cleared");
        let deletes: Vec<String> = s3.log.lock().unwrap().iter().filter(|l| l.starts_with("DELETE ")).cloned().collect();
        assert_eq!(deletes[0], format!("DELETE {base}/_manifest.json"), "the manifest first");
        assert_eq!(deletes.len(), 4);
        bag.stop().await;

        // policy always: the 30 s tick (std time) uploads by itself
        make_run(&bags_dir, "MW-E53A_20261030T120000");
        set(&mut params, NODE_STORE, "upload_policy", s("always"));
        set(&mut params, NODE_STORE, "upload_period_s", P::Double(0.2));
        let bag = Bag::start_only("/e53a_upload2", &params, false, &[NODE_STORE]).await;
        bag.until_serving().await;
        eventually("the tick's upload", || bags_dir.join("MW-E53A_20261030T120000/.uploaded").is_file()).await;
        bag.stop().await;
        std::fs::remove_dir_all(&root).unwrap();
    });
}

// ---------------------------------------------------------------------------
// DDS: one owner, the rear light, the endpoints through the bridge
// ---------------------------------------------------------------------------

/// A foreign server of `/mower_recorder/start` (the Python recorder) holds
/// all three instances back: none has an endpoint on the bus, so the bridge
/// serves and writes nothing of theirs on DDS. When it goes they serve at
/// their next look; when it comes back they step back again.
#[test]
fn one_owner_a_foreign_recorder_holds_the_module_back() {
    within(async {
        let root = scratch("owner");
        let python = Peer::new("recorder_manager_py", "/e53a_owner_py");
        let foreign = python
            .with(|n| n.create_service::<Trigger::Service>("/mower_recorder/start", QosProfile::services_default()))
            .unwrap();
        let bag = Bag::start("/e53a_owner", &production(&root, None), true).await;
        // the first look sees it
        for _ in 0..3 {
            bag.advance(mower_recorder::owner::SETTLE).await;
        }
        tokio::time::sleep(Duration::from_millis(300)).await;
        assert!(bag.serving().is_empty(), "{:?}", bag.serving());
        let looker = Peer::new("e53a_owner_looker", "");
        let servers = || looker.with(|n| n.get_publishers_info_by_topic("rr/mower_recorder/startReply", true).unwrap().len());
        assert_eq!(servers(), 1, "the foreign one only");
        // it goes: the next look serves
        python.with(|n| n.destroy_service(foreign));
        eventually("the foreign server to go", || servers() == 0).await;
        bag.advance(mower_recorder::owner::PERIOD).await;
        eventually("all three to serve", || bag.serving().len() == 3).await;
        eventually("the bridge's server on DDS", || servers() == 1).await;
        // it comes back: the module steps back at its next look
        let again = python
            .with(|n| n.create_service::<Trigger::Service>("/mower_recorder/start", QosProfile::services_default()))
            .unwrap();
        eventually("two servers", || servers() == 2).await;
        bag.advance(mower_recorder::owner::PERIOD).await;
        eventually("all three to step back", || bag.serving().is_empty()).await;
        eventually("the bridge's server to go", || servers() == 1).await;
        drop(again);
        bag.stop().await;
        std::fs::remove_dir_all(&root).unwrap();
    });
}

/// Design D5: the rear light reaches DDS through the bridge, read as
/// mower_base reads it (R TL 8 on an r2r node of the process), every 2 s and
/// latched for a late joiner, and mower_base's parser takes it as source
/// `bag`, off. The app's endpoints work through the bridge too: the latched
/// status and bag list for a late reader, start and stop over DDS.
#[test]
fn the_rear_light_and_the_app_s_endpoints_reach_dds() {
    within(async {
        let root = scratch("dds");
        let bag = Bag::start("/e53a_dds", &production(&root, None), true).await;
        let base = Peer::new("e53a_base", "/e53a_dds");
        let mut rear = base
            .with(|n| n.subscribe::<Text>("/mower_base/rear_light", QosProfile::default().keep_last(8).transient_local().reliable()))
            .unwrap();
        bag.until_serving().await;
        let first = tokio::time::timeout(WAIT, rear.next()).await.unwrap().unwrap();
        assert_eq!(first.data, REAR_OFF);
        assert_eq!(mower_base::requests::rear_light(&first.data), Some(("bag".to_string(), false)));
        bag.advance(manager::STATUS_PERIOD).await;
        assert_eq!(tokio::time::timeout(WAIT, rear.next()).await.unwrap().unwrap().data, REAR_OFF);
        // a late joiner: the latched value
        let late = Peer::new("e53a_late", "/e53a_dds");
        let mut late_rear = late
            .with(|n| n.subscribe::<Text>("/mower_base/rear_light", QosProfile::default().keep_last(8).transient_local().reliable()))
            .unwrap();
        assert_eq!(tokio::time::timeout(WAIT, late_rear.next()).await.unwrap().unwrap().data, REAR_OFF);
        // ws_bridge's readers (R TL 10): the latched status and list
        let latched = QosProfile::default().keep_last(10).transient_local().reliable();
        let mut status = late.with(|n| n.subscribe::<Text>("/mower_recorder/status", latched.clone())).unwrap();
        let mut bags = late.with(|n| n.subscribe::<Text>("/mower_recorder/bags", latched)).unwrap();
        assert_eq!(tokio::time::timeout(WAIT, status.next()).await.unwrap().unwrap().data, idle_status(30));
        assert!(tokio::time::timeout(WAIT, bags.next()).await.unwrap().unwrap().data.starts_with(r#"{"robot_id": "MW-E53A""#));
        // start and stop over DDS, as ws_bridge on r2r calls them
        for node in NODES {
            mower_bridge::coverage::assert_in_manifest(bag.m(node));
        }
        let mut answers = Vec::new();
        for service in ["/mower_recorder/start", "/mower_recorder/stop"] {
            let client = late.with(|n| n.create_client::<Trigger::Service>(service, QosProfile::services_default())).unwrap();
            let available = r2r::Node::is_available(&client).unwrap();
            tokio::time::timeout(WAIT, available).await.unwrap().unwrap();
            let r = tokio::time::timeout(WAIT, client.request(&Trigger::Request {}).unwrap()).await.unwrap().unwrap();
            answers.push((r.success, r.message.clone()));
        }
        let run = answers[0].1.strip_prefix("開始錄製 run=").unwrap_or_else(|| panic!("{answers:?}")).to_string();
        assert!(answers[0].0 && run.starts_with("MW-E53A_"), "{answers:?}");
        assert_eq!(answers[1], (true, format!("已停止並封存: full, snapshot (run={run})")));
        bag.stop().await;
        std::fs::remove_dir_all(&root).unwrap();
    });
}
