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
//! - **A name in two groups** has one subscription, on the node of the group
//!   that saw it first, feeding both: the groups share which names have one.
//!   A latched name (`/tf_static` in `bag/` and `camera/`) is the exception: a
//!   reader per group, so that each takes the whole replay, as each
//!   `ros2 bag record` did. A group joining a shared reader would get only
//!   what comes after it joined, so a camera bag could lack a static
//!   transform, depending on which thread got there first.
//! - **Each message** goes to its groups' feeds with rmw's receive time
//!   (rosbag2's log time) and source time (its publish time) and the bytes
//!   as rmw took them (CDR padded to 4, as rosbag2 records them). A feed
//!   never blocks: what does not fit its queue is dropped and counted there.
//! - **A group stops** (`Sources::stop`, the Python's SIGINT): its thread
//!   ends, its subscriptions and node go, and its feed leaves the others'
//!   subscriptions; a group that fed from one of its subscriptions subscribes
//!   on its own node at its next poll.
//!
//! **E5-4, `source: auto`** ([`BusSide`]): a name every writer of which is on
//! the bus in this process (mower_bridge's [`RecordSources`], design 3.5) is
//! read from the bus instead, by a tap on the group's own scope (named as
//! its node, `mower_bridge::RECORDER_TAPS`), at the same 100 ms poll:
//!
//! - **when:** the bus has a writer of it, its type is in `mower_schema`, and
//!   DDS shows no writer but the bus bridge's. A writer the manifest does
//!   not know (nav2's `/plan`) keeps the name on DDS, as anything else that
//!   makes the bus incomplete would (an unknown type). That look is made
//!   once, before the tap: a DDS-only writer that comes later is not recorded
//!   (the design's A8), so a writer the bags need belongs in the manifest
//!   (nav2's `/diagnostics`);
//! - **the offers** (D3): one per bus writer, in node order, its QoS as rmw
//!   reports a writer's (keep last, the depth, its reliability and
//!   durability, infinite deadline, lifespan and lease, automatic
//!   liveliness); **the type hash**: `mower_schema`'s RIHS01;
//! - **the tap** (`subscribe_erased_with`): history = the override's depth,
//!   else the deepest transient-local writer's (what the bridge's one DDS
//!   writer keeps, so a late joiner gets what rosbag2 got); transient local
//!   as rosbag2 adapts it (the override's, else every writer's); a queue of
//!   [`TAP_CAPACITY`] apart from the history;
//! - **each sample** on the group's thread `bag-tap-<group>`: CDR from the
//!   bus value by `mower_cdr` (padded to 4, as rmw's bytes; never through
//!   JSON, which loses NaN), `log_time` = when the tap took it (rosbag2's:
//!   its receive time), `publish_time` = the bus publish instant
//!   (`ErasedMsg::published`, the bus clock; E8 5.8 #2). A value r2r would
//!   refuse (a fixed array's length, a bound, a NUL) is skipped and counted,
//!   as the bridge's `to_ros` drops it.
//! - **G3** (`dds_shadow_dir`, verification only): the same names also from
//!   DDS (through the bridge) into `<dir>/<run>/<group dir>`, so that one run
//!   gives both.

use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use std::path::PathBuf;
use std::sync::atomic::AtomicU64;
use std::task::Poll;

use futures::channel::mpsc as fmpsc;
use futures::{FutureExt, StreamExt};
use mower_bridge::{RecordSource, RecordSources};
use mower_bus::{Durability, ErasedSubscriber, Reliability, TopicInfo};
use mower_mcap::Qos;
use mower_recorder_core::manager::Sources;
use mower_recorder_core::profile::GroupSpec;
use mower_recorder_core::recording::{self, Feed, QosOverrides, Recording};
use r2r::qos::{DurabilityPolicy, HistoryPolicy, LivelinessPolicy, ReliabilityPolicy};
use r2r::QosProfile;

/// rosbag2's discovery period (`topic_polling_interval`, 100 ms).
pub const DISCOVERY: Duration = Duration::from_millis(100);

/// The raw queue of one subscription, drained after every spin.
const SUBSCRIPTION_CAPACITY: usize = 4096;

/// How long a group's `spin_once` waits for data.
const SPIN: Duration = Duration::from_millis(20);

/// A tap's queue, apart from its history: what its thread may fall behind
/// by before the oldest is dropped (counted by the bus).
pub const TAP_CAPACITY: usize = 4096;

/// rmw's `RMW_DURATION_INFINITE` as rosbag2 writes it: `{9223372036,
/// 854775807}`.
const INFINITE: (u64, u64) = (9_223_372_036, 854_775_807);

/// `source:` (recorder_manager's parameter, mower_rsd.yaml).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    /// E5-3: every name from DDS.
    Dds,
    /// E5-4: a name every writer of which is on the bus here, from the bus.
    Auto,
}

impl Source {
    pub fn parse(s: &str) -> Option<Source> {
        match s.trim() {
            "dds" => Some(Source::Dds),
            "auto" => Some(Source::Auto),
            _ => None,
        }
    }
}

/// What `source: auto` needs (E5-4): the bus and where each name comes from.
#[derive(Clone)]
pub struct BusSide {
    /// recorder_manager's scope: the taps are on child scopes of it.
    pub scope: mower_bus::Scope,
    pub sources: Arc<RecordSources>,
    /// G3: also record the bus names from DDS into `<dir>/<run>/<group dir>`.
    pub shadow: Option<PathBuf>,
}

/// What a group's taps did, for the log at its end.
#[derive(Default)]
struct TapCounts {
    taken: AtomicU64,
    /// values r2r would refuse (`mower_cdr::Error::{FixedLength, Bound, Nul}`)
    invalid: AtomicU64,
    /// anything else mower_cdr could not encode
    unencodable: AtomicU64,
    /// pushed out of a tap's queue ([`TAP_CAPACITY`]) before the thread
    /// took them
    dropped: AtomicU64,
}

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
    bus: Option<BusSide>,
}

impl DdsSources {
    pub fn new(ctx: r2r::Context, logger: &str) -> DdsSources {
        DdsSources { ctx, logger: logger.to_string(), registry: Arc::default(), groups: HashMap::new(), bus: None }
    }

    /// `source: auto`: the names [`RecordSources`] gives the bus are tapped.
    pub fn with_bus(mut self, bus: BusSide) -> DdsSources {
        self.bus = Some(bus);
        self
    }
}

/// G3's shadow of a group: its own bag of the group's bus names, from DDS.
fn shadow_recording(bus: &BusSide, spec: &GroupSpec) -> Result<Option<Recording>, String> {
    let Some(root) = &bus.shadow else { return Ok(None) };
    if spec.name == "snapshot" || !spec.recorded.iter().any(|n| bus.sources.of(n) == RecordSource::Bus) {
        return Ok(None);
    }
    let bag = spec.bag.clone()?;
    let run = spec.out_dir.parent().and_then(|p| p.file_name()).ok_or("no run directory")?;
    let dir = root.join(run).join(spec.out_dir.file_name().ok_or("no group directory")?);
    std::fs::create_dir_all(dir.parent().unwrap()).map_err(|e| format!("{}: {e}", dir.display()))?;
    let o = recording::Options::new(&format!("{}-shadow", spec.name), &dir, bag);
    Recording::start(o).map(Some).map_err(|e| format!("{}: {e}", dir.display()))
}

impl Sources for DdsSources {
    fn start(&mut self, spec: &GroupSpec, overrides: &QosOverrides, feed: Feed) -> Result<(), String> {
        self.stop(spec.name);
        let (ready, started) = mpsc::channel::<Result<(), String>>();
        let stop = Arc::new(AtomicBool::new(false));
        let bus = match &self.bus {
            Some(side) => {
                let shadow = match shadow_recording(side, spec) {
                    Ok(s) => s,
                    Err(e) => {
                        r2r::log_error!(&self.logger, "{}: the G3 shadow did not start: {e}", spec.name);
                        None
                    }
                };
                Some(GroupBus { side: side.clone(), scope: side.scope.child(spec.node_name), shadow })
            }
            None => None,
        };
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
            bus,
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
    bus: Option<GroupBus>,
}

/// A group's side of `source: auto`.
struct GroupBus {
    side: BusSide,
    /// The taps' scope, named as the group's node; closed at the end.
    scope: mower_bus::Scope,
    shadow: Option<Recording>,
}

/// A name this group subscribed to, and the feeds it goes to.
struct Own {
    /// A DDS raw subscription of this group's node, or `None` for a bus tap
    /// (which its tap thread holds).
    stream: Option<r2r::RawSubscription>,
    feeds: Feeds,
}

/// A bus tap, handed to the group's tap thread.
struct Tap {
    name: String,
    ros_type: String,
    sub: ErasedSubscriber,
    feeds: Feeds,
    /// the first refused value of the name is logged
    warned: bool,
}

/// What a group's thread holds while it runs, besides its node.
struct State {
    appeared: HashSet<String>,
    own: HashMap<String, Own>,
    /// names whose subscription failed (a type r2r does not have): said once
    failed: HashSet<String>,
    /// bus names kept on DDS, and why: said once
    kept: HashSet<String>,
    /// G3: the bus names' DDS subscriptions, into the shadow
    shadow: HashMap<String, r2r::RawSubscription>,
    shadow_appeared: HashSet<String>,
    taps: Option<fmpsc::UnboundedSender<Tap>>,
    tap_thread: Option<JoinHandle<()>>,
    counts: Arc<TapCounts>,
}

impl GroupThread {
    fn run(mut self, ready: mpsc::Sender<Result<(), String>>) {
        let mut node = match r2r::Node::create(self.ctx.clone(), self.node_name, "") {
            Ok(n) => n,
            Err(e) => {
                let _ = ready.send(Err(format!("node {}: {e}", self.node_name)));
                return;
            }
        };
        let _ = ready.send(Ok(()));
        let counts = Arc::new(TapCounts::default());
        let (taps, tap_thread) = match &self.bus {
            Some(_) => {
                let (tx, rx) = fmpsc::unbounded::<Tap>();
                let (logger, counts) = (self.logger.clone(), counts.clone());
                let spawned = std::thread::Builder::new()
                    .name(format!("bag-tap-{}", self.name))
                    .spawn(move || tap_thread(rx, logger, counts));
                match spawned {
                    Ok(t) => (Some(tx), Some(t)),
                    Err(e) => {
                        r2r::log_error!(&self.logger, "{}: no tap thread ({e}): every name from DDS", self.node_name);
                        (None, None)
                    }
                }
            }
            None => (None, None),
        };
        let mut st = State {
            appeared: HashSet::new(),
            own: HashMap::new(),
            failed: HashSet::new(),
            kept: HashSet::new(),
            shadow: HashMap::new(),
            shadow_appeared: HashSet::new(),
            taps,
            tap_thread,
            counts,
        };
        let mut next_discovery = Instant::now();
        while !self.stop.load(Ordering::Acquire) {
            if Instant::now() >= next_discovery {
                self.discover(&mut node, &mut st);
                next_discovery = Instant::now() + DISCOVERY;
            }
            node.spin_once(SPIN);
            for (name, sub) in &mut st.own {
                let Some(stream) = sub.stream.as_mut() else { continue };
                while let Some(Some(m)) = stream.next().now_or_never() {
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
            if let Some(shadow) = self.bus.as_ref().and_then(|b| b.shadow.as_ref()) {
                let feed = shadow.feed();
                for (name, stream) in &mut st.shadow {
                    while let Some(Some(m)) = stream.next().now_or_never() {
                        feed.message(name, m.received_timestamp, m.source_timestamp, m.data);
                    }
                }
            }
        }
        // the end: this group's names leave the registry, its feed the others'
        let mut registry = lock(&self.registry);
        for (name, sub) in st.own.drain() {
            // (a latched name's reader is this group's alone: not registered)
            if registry.get(&name).is_some_and(|feeds| Arc::ptr_eq(feeds, &sub.feeds)) {
                registry.remove(&name);
            }
            if let Some(stream) = sub.stream {
                node.destroy_raw_subscription(stream);
            }
        }
        for feeds in registry.values() {
            lock(feeds).retain(|f| f.group() != self.name);
        }
        drop(registry);
        for (_, stream) in st.shadow.drain() {
            node.destroy_raw_subscription(stream);
        }
        // the tap thread takes what its taps still hold, then ends (its
        // sender gone); the taps' scope closes after, so that every sample
        // published before the stop is fed, as a DDS reader's queue is taken
        drop(st.taps.take());
        if let Some(t) = st.tap_thread.take() {
            let _ = t.join();
        }
        if let Some(bus) = &self.bus {
            bus.scope.close();
        }
        let c = &st.counts;
        let (taken, invalid, bad, dropped) = (
            c.taken.load(Ordering::Relaxed),
            c.invalid.load(Ordering::Relaxed),
            c.unencodable.load(Ordering::Relaxed),
            c.dropped.load(Ordering::Relaxed),
        );
        if self.bus.is_some() {
            r2r::log_info!(
                &self.logger,
                "{}: {taken} samples from the bus, {invalid} refused as r2r refuses them, {bad} not encodable, {dropped} lost to a full tap queue",
                self.node_name
            );
        }
        if let Some(shadow) = self.bus.as_mut().and_then(|b| b.shadow.take()) {
            match shadow.stop() {
                Ok(f) => r2r::log_info!(&self.logger, "{}: G3 shadow {} written", self.node_name, f.group),
                Err(e) => r2r::log_error!(&self.logger, "{}: G3 shadow: {e}", self.node_name),
            }
        }
        drop(node);
    }

    /// rosbag2's poll: a wanted name with one type and a writer appears in
    /// the group, and is subscribed unless another group has it. With
    /// `source: auto`, a bus name is tapped instead ([`GroupThread::tap`]).
    fn discover(&mut self, node: &mut r2r::Node, st: &mut State) {
        let mut types: Option<HashMap<String, Vec<String>>> = None;
        for name in self.wanted.clone() {
            if let Some(o) = st.own.get(&name) {
                if o.stream.is_none() {
                    self.shadow_subscribe(node, st, &name, &mut types);
                }
                continue;
            }
            let shared = {
                let registry = lock(&self.registry);
                registry.get(&name).cloned()
            };
            if st.appeared.contains(&name) {
                if let Some(feeds) = &shared {
                    // fed by another group's subscription
                    let mut feeds = lock(feeds);
                    if !feeds.iter().any(|f| f.group() == self.name) {
                        feeds.push(self.feed.clone());
                    }
                    continue;
                }
            }
            match self.tap(node, st, &name) {
                Tapped::Yes => {
                    // G3: the shadow's DDS reader joins in the same poll as
                    // the tap, so that both take the same latched history
                    // (a poll later, a sample published in between would be
                    // the only one the DDS reader replays)
                    self.shadow_subscribe(node, st, &name, &mut types);
                    continue;
                }
                Tapped::NotYet => continue,
                Tapped::Dds => {}
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
            let Some(type_names) = types.as_ref().and_then(|t| t.get(&name)) else { continue };
            if type_names.len() != 1 {
                continue;
            }
            let type_name = type_names[0].clone();
            let writers = match node.get_publishers_info_by_topic(&name, false) {
                Ok(w) if !w.is_empty() => w,
                _ => continue,
            };
            let offers: Vec<Qos> = writers.iter().map(|w| offered(&w.qos_profile)).collect();
            if !st.appeared.contains(&name) {
                let hash = writers.iter().map(|w| w.topic_type_hash.clone()).find(|h| !h.is_empty()).unwrap_or_default();
                self.feed.topic_appeared(recording::topic_metadata(&name, &type_name, &offers, &hash));
                st.appeared.insert(name.clone());
            }
            let qos = recording::reader_qos(&name, &offers, &self.overrides);
            let latched = qos.durability == "transient_local";
            let mut registry = lock(&self.registry);
            if !latched {
                if let Some(feeds) = registry.get(&name) {
                    let mut feeds = lock(feeds);
                    if !feeds.iter().any(|f| f.group() == self.name) {
                        feeds.push(self.feed.clone());
                    }
                    continue;
                }
            }
            match node.subscribe_raw_with_info(&name, &type_name, reader_profile(&qos), SUBSCRIPTION_CAPACITY) {
                Ok(stream) => {
                    let feeds: Feeds = Arc::new(Mutex::new(vec![self.feed.clone()]));
                    if !latched {
                        registry.insert(name.clone(), feeds.clone());
                    }
                    st.own.insert(name.clone(), Own { stream: Some(stream), feeds });
                }
                Err(e) => {
                    if st.failed.insert(name.clone()) {
                        r2r::log_error!(&self.logger, "{}: cannot subscribe to {name} ({type_name}): {e}", self.node_name);
                    }
                }
            }
        }
    }

    /// `source: auto`: tap `name` on the bus if [`RecordSources`] gives it
    /// the bus, DDS shows no writer of it but the bridge's, and the bus has
    /// a writer of a type `mower_schema` has. `Dds` hands it to rosbag2's
    /// way; `NotYet` waits for the bus's first writer.
    fn tap(&mut self, node: &mut r2r::Node, st: &mut State, name: &str) -> Tapped {
        let Some(bus) = &self.bus else { return Tapped::Dds };
        let Some(taps) = st.taps.clone() else { return Tapped::Dds };
        if st.kept.contains(name) {
            return Tapped::Dds;
        }
        if let RecordSource::Dds(_) = bus.side.sources.of(name) {
            return Tapped::Dds;
        }
        let keep = |st: &mut State, why: String| {
            if st.kept.insert(name.to_string()) {
                r2r::log_info!(&self.logger, "{}: {name} from DDS: {why}", self.node_name);
            }
            Tapped::Dds
        };
        let foreign: Vec<String> = node
            .get_publishers_info_by_topic(name, false)
            .unwrap_or_default()
            .into_iter()
            // the bridge's own writers carry the bus's samples
            .filter(|w| !crate::owner::is_bridge(&w.node_name, &w.node_namespace))
            .map(|w| format!("{}/{}", w.node_namespace.trim_end_matches('/'), w.node_name))
            .collect();
        if !foreign.is_empty() {
            return keep(st, format!("written on DDS by {} too, which the manifest does not know", foreign.join(", ")));
        }
        let Some(info) = bus.side.scope.bus().topic(name) else { return Tapped::NotYet };
        if info.publishers.is_empty() {
            return Tapped::NotYet;
        }
        let Some(ros_type) = info.ros_type.clone() else {
            return keep(st, format!("its bus type {:?} has no ROS name", info.type_name));
        };
        let Some(hash) = mower_schema::type_hash(&ros_type) else {
            return keep(st, format!("{ros_type} is not in mower_schema"));
        };
        let offers = bus_offers(&info);
        if !st.appeared.contains(name) {
            self.feed.topic_appeared(recording::topic_metadata(name, &ros_type, &offers, hash));
            st.appeared.insert(name.to_string());
        }
        let qos = tap_qos(name, &info, &self.overrides);
        let latched = qos.is_transient_local();
        let mut registry = lock(&self.registry);
        if !latched {
            if let Some(feeds) = registry.get(name) {
                let mut feeds = lock(feeds);
                if !feeds.iter().any(|f| f.group() == self.name) {
                    feeds.push(self.feed.clone());
                }
                return Tapped::Yes;
            }
        }
        match bus.scope.subscribe_erased_with(name, qos, TAP_CAPACITY) {
            Ok(sub) => {
                let feeds: Feeds = Arc::new(Mutex::new(vec![self.feed.clone()]));
                if !latched {
                    registry.insert(name.to_string(), feeds.clone());
                }
                st.own.insert(name.to_string(), Own { stream: None, feeds: feeds.clone() });
                let _ = taps.unbounded_send(Tap { name: name.to_string(), ros_type, sub, feeds, warned: false });
                Tapped::Yes
            }
            Err(e) => {
                drop(registry);
                keep(st, format!("no tap: {e}"))
            }
        }
    }

    /// G3: a tapped name's DDS samples (the bridge's) into the shadow bag.
    fn shadow_subscribe(&self, node: &mut r2r::Node, st: &mut State, name: &str, types: &mut Option<HashMap<String, Vec<String>>>) {
        let Some(shadow) = self.bus.as_ref().and_then(|b| b.shadow.as_ref()) else { return };
        if st.shadow.contains_key(name) {
            return;
        }
        if types.is_none() {
            match node.get_topic_names_and_types() {
                Ok(t) => *types = Some(t),
                Err(_) => return,
            }
        }
        let Some([type_name]) = types.as_ref().and_then(|t| t.get(name)).map(Vec::as_slice) else { return };
        let writers = match node.get_publishers_info_by_topic(name, false) {
            Ok(w) if !w.is_empty() => w,
            _ => return,
        };
        let offers: Vec<Qos> = writers.iter().map(|w| offered(&w.qos_profile)).collect();
        if st.shadow_appeared.insert(name.to_string()) {
            let hash = writers.iter().map(|w| w.topic_type_hash.clone()).find(|h| !h.is_empty()).unwrap_or_default();
            shadow.feed().topic_appeared(recording::topic_metadata(name, type_name, &offers, &hash));
        }
        let qos = reader_profile(&recording::reader_qos(name, &offers, &self.overrides));
        if let Ok(stream) = node.subscribe_raw_with_info(name, type_name, qos, SUBSCRIPTION_CAPACITY) {
            st.shadow.insert(name.to_string(), stream);
        }
    }
}

/// What [`GroupThread::tap`] did with a name.
enum Tapped {
    Yes,
    NotYet,
    Dds,
}

/// D3: one offer per bus writer, as rmw reports a writer's QoS, in the
/// writers' node order (stable across restarts).
pub fn bus_offers(info: &TopicInfo) -> Vec<Qos> {
    let mut writers: Vec<(&String, &mower_bus::Qos)> = info.publishers.iter().zip(&info.publisher_qos).collect();
    writers.sort_by(|a, b| a.0.cmp(b.0).then_with(|| a.1.depth.cmp(&b.1.depth)));
    writers.into_iter().map(|(_, q)| bus_offer(q)).collect()
}

/// A bus writer's QoS as rmw_cyclonedds reports a DDS writer's (the
/// bridge's `to_ros` writer with that profile): keep last its depth (at
/// least 1), its reliability and durability, every duration infinite,
/// automatic liveliness.
pub fn bus_offer(q: &mower_bus::Qos) -> Qos {
    Qos {
        history: "keep_last".into(),
        depth: q.effective_depth() as u64,
        reliability: match q.reliability {
            Reliability::Reliable => "reliable",
            Reliability::BestEffort => "best_effort",
        }
        .into(),
        durability: match q.durability {
            Durability::TransientLocal => "transient_local",
            Durability::Volatile => "volatile",
        }
        .into(),
        deadline: INFINITE,
        lifespan: INFINITE,
        liveliness: "automatic".into(),
        liveliness_lease_duration: INFINITE,
        avoid_ros_namespace_conventions: false,
    }
}

/// The tap's QoS: what rosbag2's reader would be (`reader_qos`: the
/// override, else adapted to the offers) as the bus reads it. Its history
/// is the override's depth (keep all: the deepest writer's), else the
/// deepest transient-local writer's: what the bridge's one DDS writer keeps,
/// so that a late joiner gets on the bus what rosbag2 got on DDS.
pub fn tap_qos(name: &str, info: &TopicInfo, overrides: &QosOverrides) -> mower_bus::Qos {
    let offers = bus_offers(info);
    let r = recording::reader_qos(name, &offers, overrides);
    let deepest = info.publisher_qos.iter().filter(|q| q.is_transient_local()).map(|q| q.effective_depth()).max().unwrap_or(1);
    let depth = match overrides.get(name) {
        Some(o) if o.history != "keep_all" && o.depth > 0 => o.depth as usize,
        _ => deepest,
    };
    let q = mower_bus::Qos::default().keep_last(depth);
    let q = if r.reliability == "best_effort" { q.best_effort() } else { q.reliable() };
    if r.durability == "transient_local" {
        q.transient_local()
    } else {
        q.volatile()
    }
}

/// A group's taps, on its thread `bag-tap-<group>`: every sample of every
/// tap, encoded by `mower_cdr`, to its feeds with its receive time and the
/// bus publish instant. Once the group thread has dropped its sender (the
/// group's stop), what each tap still holds is taken, and the thread ends;
/// a tap whose scope closed earlier (the module stopping) ends on its own.
fn tap_thread(mut rx: fmpsc::UnboundedReceiver<Tap>, logger: String, counts: Arc<TapCounts>) {
    let mut taps: Vec<Tap> = Vec::new();
    futures::executor::block_on(std::future::poll_fn(|cx| {
        loop {
            match rx.poll_next_unpin(cx) {
                Poll::Ready(Some(tap)) => taps.push(tap),
                Poll::Ready(None) => {
                    // the stop: the rest of every queue, then the end
                    for tap in &mut taps {
                        while let Ok(msg) = tap.sub.try_recv() {
                            take(tap, &msg, &logger, &counts);
                        }
                    }
                    return Poll::Ready(());
                }
                Poll::Pending => break,
            }
        }
        taps.retain_mut(|tap| loop {
            match tap.sub.poll_next_unpin(cx) {
                Poll::Ready(Some(msg)) => take(tap, &msg, &logger, &counts),
                Poll::Ready(None) => {
                    counts.dropped.fetch_add(tap.sub.dropped(), Ordering::Relaxed);
                    return false;
                }
                Poll::Pending => return true,
            }
        });
        Poll::Pending
    }));
    let dropped: u64 = taps.iter().map(|t| t.sub.dropped()).sum();
    counts.dropped.fetch_add(dropped, Ordering::Relaxed);
}

/// One sample of `tap`: CDR, then to every feed.
fn take(tap: &mut Tap, msg: &mower_bus::ErasedMsg, logger: &str, counts: &TapCounts) {
    let recv = mower_bus::Time::from_system(std::time::SystemTime::now()).ns();
    match mower_cdr::to_vec(&tap.ros_type, msg) {
        Ok(cdr) => {
            counts.taken.fetch_add(1, Ordering::Relaxed);
            let send = msg.published().ns();
            let feeds = lock(&tap.feeds);
            if let Some((last, rest)) = feeds.split_last() {
                for f in rest {
                    f.message(&tap.name, recv, send, cdr.clone());
                }
                last.message(&tap.name, recv, send, cdr);
            }
        }
        Err(e) => {
            let refused = matches!(e, mower_cdr::Error::FixedLength { .. } | mower_cdr::Error::Bound { .. } | mower_cdr::Error::Nul { .. });
            let n = if refused { &counts.invalid } else { &counts.unencodable };
            n.fetch_add(1, Ordering::Relaxed);
            if !tap.warned {
                tap.warned = true;
                r2r::log_warn!(logger, "{}: a sample skipped ({}): {e}", tap.name, if refused { "r2r refuses it too" } else { "not encodable" });
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
