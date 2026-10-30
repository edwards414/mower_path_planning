//! A run's DDS sources (E5-3b): what the `ros2 bag record` children of the
//! Python recorder subscribed to, as `mower_recorder_core::manager::Sources`.
//!
//! - **A thread per group** (`bag-src-<group>`), as each group was a process
//!   of its own: it owns the group's node, named as rosbag2's
//!   (`mower_full_recorder`, `mower_snapshot_recorder`,
//!   `mower_camera_recorder`: design D12, so that the graph and
//!   `/graph_snapshot` show the run's recorders as before), spins it, and
//!   polls discovery every 100 ms as rosbag2 does.
//! - **A name appears** for a group when the graph shows at least one writer
//!   and exactly one type (rosbag2 skips unpublished names and names of two
//!   types): `Feed::topic_appeared` with the writers' offered QoS as rmw
//!   reports them and the first writer's type hash, then a raw subscription
//!   (vendor/r2r patch 3) with `recording::reader_qos` (the override, else
//!   adapted to the offers). Hidden names never are (`GroupSpec::recorded`
//!   leaves them out, and `Feed` refuses them).
//! - **A name in two groups** (`/tf_static` in `bag/` and `camera/`) has one
//!   subscription, on the node of the group that saw it first, feeding both:
//!   the groups share which names have one.
//! - **Each message** goes to its groups' feeds with rmw's receive time
//!   (rosbag2's log time) and source time (its publish time) and the bytes
//!   as rmw took them (CDR padded to 4, as rosbag2 records them). A feed
//!   never blocks: what does not fit its queue is dropped and counted there.
//! - **A group stops** (`Sources::stop`, the Python's SIGINT): its thread
//!   ends, its subscriptions and node go, and its feed leaves the others'
//!   subscriptions; a group that fed from one of its subscriptions subscribes
//!   on its own node at its next poll.

use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use futures::{FutureExt, StreamExt};
use mower_mcap::Qos;
use mower_recorder_core::manager::Sources;
use mower_recorder_core::profile::GroupSpec;
use mower_recorder_core::recording::{self, Feed, QosOverrides};
use r2r::qos::{DurabilityPolicy, HistoryPolicy, LivelinessPolicy, ReliabilityPolicy};
use r2r::QosProfile;

/// rosbag2's discovery period (`topic_polling_interval`, 100 ms).
pub const DISCOVERY: Duration = Duration::from_millis(100);

/// The raw queue of one subscription, drained after every spin.
const SUBSCRIPTION_CAPACITY: usize = 4096;

/// How long a group's `spin_once` waits for data.
const SPIN: Duration = Duration::from_millis(20);

/// The feeds of one subscribed name.
type Feeds = Arc<Mutex<Vec<Feed>>>;

/// Which names have a subscription (on some group's node), and its feeds.
type Registry = Arc<Mutex<HashMap<String, Feeds>>>;

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

struct Running {
    stop: Arc<AtomicBool>,
    thread: JoinHandle<()>,
}

/// The [`Sources`] the module gives its manager.
pub struct DdsSources {
    ctx: r2r::Context,
    logger: String,
    registry: Registry,
    groups: HashMap<String, Running>,
}

impl DdsSources {
    pub fn new(ctx: r2r::Context, logger: &str) -> DdsSources {
        DdsSources { ctx, logger: logger.to_string(), registry: Arc::default(), groups: HashMap::new() }
    }
}

impl Sources for DdsSources {
    fn start(&mut self, spec: &GroupSpec, overrides: &QosOverrides, feed: Feed) -> Result<(), String> {
        self.stop(spec.name);
        let (ready, started) = mpsc::channel::<Result<(), String>>();
        let stop = Arc::new(AtomicBool::new(false));
        let group = GroupThread {
            ctx: self.ctx.clone(),
            logger: self.logger.clone(),
            registry: self.registry.clone(),
            name: spec.name.to_string(),
            node_name: spec.node_name,
            wanted: spec.recorded.clone(),
            overrides: overrides.clone(),
            feed,
            stop: stop.clone(),
        };
        let thread = std::thread::Builder::new()
            .name(format!("bag-src-{}", spec.name))
            .spawn(move || group.run(ready))
            .map_err(|e| format!("no thread for {}: {e}", spec.name))?;
        // the node exists (or failed) before the group is reported started
        match started.recv() {
            Ok(Ok(())) => {
                self.groups.insert(spec.name.to_string(), Running { stop, thread });
                Ok(())
            }
            Ok(Err(e)) => {
                let _ = thread.join();
                Err(e)
            }
            Err(_) => {
                let _ = thread.join();
                Err(format!("the {} sources thread ended at its start", spec.name))
            }
        }
    }

    fn stop(&mut self, group: &str) {
        if let Some(r) = self.groups.remove(group) {
            r.stop.store(true, Ordering::Release);
            let _ = r.thread.join();
        }
    }
}

impl Drop for DdsSources {
    fn drop(&mut self) {
        let names: Vec<String> = self.groups.keys().cloned().collect();
        for g in names {
            self.stop(&g);
        }
    }
}

struct GroupThread {
    ctx: r2r::Context,
    logger: String,
    registry: Registry,
    name: String,
    node_name: &'static str,
    wanted: Vec<String>,
    overrides: QosOverrides,
    feed: Feed,
    stop: Arc<AtomicBool>,
}

/// A subscription of this group's node.
struct Own {
    stream: r2r::RawSubscription,
    feeds: Feeds,
}

impl GroupThread {
    fn run(self, ready: mpsc::Sender<Result<(), String>>) {
        let mut node = match r2r::Node::create(self.ctx.clone(), self.node_name, "") {
            Ok(n) => n,
            Err(e) => {
                let _ = ready.send(Err(format!("node {}: {e}", self.node_name)));
                return;
            }
        };
        let _ = ready.send(Ok(()));
        let mut appeared: HashSet<String> = HashSet::new();
        let mut own: HashMap<String, Own> = HashMap::new();
        // names whose subscription failed (a type r2r does not have): said once
        let mut failed: HashSet<String> = HashSet::new();
        let mut next_discovery = Instant::now();
        while !self.stop.load(Ordering::Acquire) {
            if Instant::now() >= next_discovery {
                self.discover(&mut node, &mut appeared, &mut own, &mut failed);
                next_discovery = Instant::now() + DISCOVERY;
            }
            node.spin_once(SPIN);
            for (name, sub) in &mut own {
                while let Some(Some(m)) = sub.stream.next().now_or_never() {
                    let r2r::RawMessage { data, source_timestamp, received_timestamp, .. } = m;
                    let feeds = lock(&sub.feeds);
                    if let Some((last, rest)) = feeds.split_last() {
                        for f in rest {
                            f.message(name, received_timestamp, source_timestamp, data.clone());
                        }
                        last.message(name, received_timestamp, source_timestamp, data);
                    }
                }
            }
        }
        // the end: this group's names leave the registry, its feed the others'
        let mut registry = lock(&self.registry);
        for (name, sub) in own.drain() {
            registry.remove(&name);
            node.destroy_raw_subscription(sub.stream);
        }
        for feeds in registry.values() {
            lock(feeds).retain(|f| f.group() != self.name);
        }
        drop(registry);
        drop(node);
    }

    /// rosbag2's poll: a wanted name with one type and a writer appears in
    /// the group, and is subscribed unless another group has it.
    fn discover(&self, node: &mut r2r::Node, appeared: &mut HashSet<String>, own: &mut HashMap<String, Own>, failed: &mut HashSet<String>) {
        let mut types: Option<HashMap<String, Vec<String>>> = None;
        for name in &self.wanted {
            if own.contains_key(name) {
                continue;
            }
            let shared = {
                let registry = lock(&self.registry);
                registry.get(name).cloned()
            };
            if appeared.contains(name) {
                if let Some(feeds) = &shared {
                    // fed by another group's subscription
                    let mut feeds = lock(feeds);
                    if !feeds.iter().any(|f| f.group() == self.name) {
                        feeds.push(self.feed.clone());
                    }
                    continue;
                }
            }
            if types.is_none() {
                match node.get_topic_names_and_types() {
                    Ok(t) => types = Some(t),
                    Err(e) => {
                        r2r::log_warn!(&self.logger, "{} discovery: {e}", self.node_name);
                        return;
                    }
                }
            }
            let Some(type_names) = types.as_ref().and_then(|t| t.get(name)) else { continue };
            if type_names.len() != 1 {
                continue;
            }
            let type_name = type_names[0].clone();
            let writers = match node.get_publishers_info_by_topic(name, false) {
                Ok(w) if !w.is_empty() => w,
                _ => continue,
            };
            let offers: Vec<Qos> = writers.iter().map(|w| offered(&w.qos_profile)).collect();
            if !appeared.contains(name) {
                let hash = writers.iter().map(|w| w.topic_type_hash.clone()).find(|h| !h.is_empty()).unwrap_or_default();
                self.feed.topic_appeared(recording::topic_metadata(name, &type_name, &offers, &hash));
                appeared.insert(name.clone());
            }
            let mut registry = lock(&self.registry);
            if let Some(feeds) = registry.get(name) {
                let mut feeds = lock(feeds);
                if !feeds.iter().any(|f| f.group() == self.name) {
                    feeds.push(self.feed.clone());
                }
                continue;
            }
            let qos = reader_profile(&recording::reader_qos(name, &offers, &self.overrides));
            match node.subscribe_raw_with_info(name, &type_name, qos, SUBSCRIPTION_CAPACITY) {
                Ok(stream) => {
                    let feeds: Feeds = Arc::new(Mutex::new(vec![self.feed.clone()]));
                    registry.insert(name.clone(), feeds.clone());
                    own.insert(name.clone(), Own { stream, feeds });
                }
                Err(e) => {
                    if failed.insert(name.clone()) {
                        r2r::log_error!(&self.logger, "{}: cannot subscribe to {name} ({type_name}): {e}", self.node_name);
                    }
                }
            }
        }
    }
}

fn duration(d: std::time::Duration) -> (u64, u64) {
    (d.as_secs(), d.subsec_nanos() as u64)
}

/// A writer's QoS as rosbag2 writes it into `offered_qos_profiles`: rmw's
/// report, its policies by their rmw names.
pub fn offered(p: &QosProfile) -> Qos {
    Qos {
        history: match p.history {
            HistoryPolicy::KeepLast => "keep_last",
            HistoryPolicy::KeepAll => "keep_all",
            HistoryPolicy::SystemDefault => "system_default",
            HistoryPolicy::Unknown => "unknown",
        }
        .into(),
        depth: p.depth as u64,
        reliability: match p.reliability {
            ReliabilityPolicy::Reliable => "reliable",
            ReliabilityPolicy::BestEffort => "best_effort",
            ReliabilityPolicy::SystemDefault => "system_default",
            ReliabilityPolicy::BestAvailable => "best_available",
            ReliabilityPolicy::Unknown => "unknown",
        }
        .into(),
        durability: match p.durability {
            DurabilityPolicy::TransientLocal => "transient_local",
            DurabilityPolicy::Volatile => "volatile",
            DurabilityPolicy::SystemDefault => "system_default",
            DurabilityPolicy::BestAvailable => "best_available",
            DurabilityPolicy::Unknown => "unknown",
        }
        .into(),
        deadline: duration(p.deadline),
        lifespan: duration(p.lifespan),
        liveliness: match p.liveliness {
            LivelinessPolicy::Automatic => "automatic",
            LivelinessPolicy::ManualByNode => "manual_by_node",
            LivelinessPolicy::ManualByTopic => "manual_by_topic",
            LivelinessPolicy::SystemDefault => "system_default",
            LivelinessPolicy::BestAvailable => "best_available",
            LivelinessPolicy::Unknown => "unknown",
        }
        .into(),
        liveliness_lease_duration: duration(p.liveliness_lease_duration),
        avoid_ros_namespace_conventions: p.avoid_ros_namespace_conventions,
    }
}

/// The reader QoS (`recording::reader_qos`, rmw's names) as r2r's profile;
/// `(0, 0)` durations are rmw's defaults.
pub fn reader_profile(q: &Qos) -> QosProfile {
    let d = |(s, n): (u64, u64)| std::time::Duration::new(s, n as u32);
    let mut p = QosProfile::default();
    p.history = match q.history.as_str() {
        "keep_last" => HistoryPolicy::KeepLast,
        "keep_all" => HistoryPolicy::KeepAll,
        _ => HistoryPolicy::SystemDefault,
    };
    p.depth = q.depth as usize;
    p.reliability = match q.reliability.as_str() {
        "reliable" => ReliabilityPolicy::Reliable,
        "best_effort" => ReliabilityPolicy::BestEffort,
        "best_available" => ReliabilityPolicy::BestAvailable,
        _ => ReliabilityPolicy::SystemDefault,
    };
    p.durability = match q.durability.as_str() {
        "transient_local" => DurabilityPolicy::TransientLocal,
        "volatile" => DurabilityPolicy::Volatile,
        "best_available" => DurabilityPolicy::BestAvailable,
        _ => DurabilityPolicy::SystemDefault,
    };
    p.deadline = d(q.deadline);
    p.lifespan = d(q.lifespan);
    p.liveliness = match q.liveliness.as_str() {
        "automatic" => LivelinessPolicy::Automatic,
        "manual_by_topic" => LivelinessPolicy::ManualByTopic,
        "best_available" => LivelinessPolicy::BestAvailable,
        _ => LivelinessPolicy::SystemDefault,
    };
    p.liveliness_lease_duration = d(q.liveliness_lease_duration);
    p.avoid_ros_namespace_conventions = q.avoid_ros_namespace_conventions;
    p
}
