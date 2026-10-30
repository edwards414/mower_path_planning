//! One group of a run (`full` → `bag/`, `camera` → `camera/`, `snapshot` →
//! `snapshots/`): what one `ros2 bag record` child of recorder_manager did,
//! in the process (E5 design 4.1, 4.2).
//!
//! - **A writer thread** named `bag-<group>`, niced +10 on Linux, owns the
//!   group's [`mower_mcap::BagWriter`] (rosbag2 0.26.11's writer, byte for
//!   byte): it compresses (per-message zstd, `zstd_fast` chunks), writes,
//!   splits, and in snapshot mode keeps rosbag2's ring buffer.
//! - **Producers** (DDS callbacks, bus taps) hold a [`Feed`] and never
//!   block: the queue between them and the writer is bounded in bytes
//!   ([`QUEUE_BYTES`], 64 MiB). A message that does not fit is dropped,
//!   counted, and the first drop is a WARN. Topic, snapshot and split
//!   requests are never dropped. rosbag2's own drops (its compression queue
//!   of 1, E5-0 F1) are not reproduced: everything that fits is recorded
//!   (owner decision A12).
//! - **The input** is what rosbag2's recorder gives its writer:
//!   [`Feed::topic_appeared`] when the group first sees a writer of a name
//!   (`create_topic`: the type, every writer's offered QoS and the type
//!   hash at that moment, writers seen later are not added), then
//!   [`Feed::message`] (receive time, source time, CDR). Hidden names
//!   ([`profile::is_hidden`]) are refused, as rosbag2 skips them without
//!   `--include-hidden-topics`.
//! - **The end:** [`Recording::stop`] closes the input, drains the queue and
//!   finalizes (the last file's summary and footer, `metadata.yaml`).
//!   Dropping a [`Recording`] does the same, so a panic elsewhere (the
//!   manager, the module) still leaves finished bags (design R5). A panic
//!   in the writer thread itself leaves the file as SIGKILL leaves
//!   rosbag2's: unfinalized.
//!
//! The functions the source side calls besides: [`reader_qos`] (the QoS
//! rosbag2 subscribes with: the override, else
//! [`adapt_request_to_offers`]), [`QosOverrides`] (`qos_overrides.yaml` as
//! `ros2 bag record --qos-profile-overrides-path` reads it) and
//! [`topic_metadata`].
//!
//! What the source side does (E5-3a): per group, poll discovery every
//! 100 ms as rosbag2 does; a recorded, not hidden name with at least one
//! writer and one type (rosbag2 skips unpublished names and names with two
//! types) gets `topic_appeared` with the writers known now, then a
//! subscription with `reader_qos(name, offers, overrides)`; each sample goes
//! to `message` with its rmw receive and source timestamps. A name in two
//! groups (`/tf_static` in `bag/` and `camera/`) has one subscription
//! feeding both.

use std::collections::{HashSet, VecDeque};
use std::io;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{mpsc, Arc, Condvar, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::Duration;

use mower_mcap::{BagMetadata, BagOptions, BagWriter, Qos, TopicMetadata};
use mower_pyfmt::Py;

use crate::profile;

/// The writer queue's bound (design 4.1).
pub const QUEUE_BYTES: usize = 64 << 20;

/// How long a snapshot or split request waits for the writer (the Python
/// recorder's `ros2 service call` had 15 s).
pub const REQUEST_TIMEOUT: Duration = Duration::from_secs(15);

/// A log line's level.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    Info,
    Warn,
    Error,
}

/// Where a recording logs (the module passes its r2r logger).
pub type Log = Arc<dyn Fn(Level, &str) + Send + Sync>;

/// A [`Log`] that drops everything.
pub fn no_log() -> Log {
    Arc::new(|_, _| {})
}

/// A group's counters.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Stats {
    /// topics created
    pub topics: u64,
    /// messages taken into the queue
    pub queued: u64,
    /// messages the writer took (written, or buffered in snapshot mode)
    pub written: u64,
    /// refused: the queue was over its bound
    pub dropped_full: u64,
    /// refused: after the stop, or with the writer gone
    pub dropped_closed: u64,
    /// refused: the name had not appeared (or is hidden)
    pub dropped_unknown: u64,
    /// the writer failed on one
    pub write_errors: u64,
}

enum Item {
    Topic(TopicMetadata),
    Message { topic: String, recv: i64, send: i64, cdr: Vec<u8> },
    Snapshot(mpsc::Sender<io::Result<bool>>),
    Split(mpsc::Sender<io::Result<()>>),
}

impl Item {
    /// What a queued item counts against the bound.
    fn bytes(&self) -> usize {
        match self {
            Item::Message { topic, cdr, .. } => cdr.len() + topic.len(),
            _ => 0,
        }
    }
}

struct Queue {
    items: VecDeque<Item>,
    bytes: usize,
    closed: bool,
    topics: HashSet<String>,
    warned_full: bool,
}

struct Counters {
    topics: AtomicU64,
    queued: AtomicU64,
    written: AtomicU64,
    dropped_full: AtomicU64,
    dropped_closed: AtomicU64,
    dropped_unknown: AtomicU64,
    write_errors: AtomicU64,
}

struct Shared {
    group: String,
    capacity: usize,
    include_hidden: bool,
    q: Mutex<Queue>,
    cv: Condvar,
    n: Counters,
    log: Log,
}

impl Shared {
    fn lock(&self) -> MutexGuard<'_, Queue> {
        self.q.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn stats(&self) -> Stats {
        let g = |a: &AtomicU64| a.load(Ordering::Relaxed);
        Stats {
            topics: g(&self.n.topics),
            queued: g(&self.n.queued),
            written: g(&self.n.written),
            dropped_full: g(&self.n.dropped_full),
            dropped_closed: g(&self.n.dropped_closed),
            dropped_unknown: g(&self.n.dropped_unknown),
            write_errors: g(&self.n.write_errors),
        }
    }

    fn close(&self) {
        self.lock().closed = true;
        self.cv.notify_all();
    }

    /// A request the writer answers; refused once the input is closed.
    fn request<T>(&self, make: impl FnOnce(mpsc::Sender<io::Result<T>>) -> Item) -> io::Result<T> {
        let (tx, rx) = mpsc::channel();
        {
            let mut q = self.lock();
            if q.closed {
                return Err(io::Error::other(format!("bag-{}: the recording is stopped", self.group)));
            }
            q.items.push_back(make(tx));
        }
        self.cv.notify_one();
        match rx.recv_timeout(REQUEST_TIMEOUT) {
            Ok(r) => r,
            Err(mpsc::RecvTimeoutError::Timeout) => Err(io::Error::new(io::ErrorKind::TimedOut, format!("bag-{}: no answer in {} s", self.group, REQUEST_TIMEOUT.as_secs()))),
            Err(mpsc::RecvTimeoutError::Disconnected) => Err(io::Error::other(format!("bag-{}: the writer is gone", self.group))),
        }
    }
}

/// A group's input, for any number of producers (cheap to clone).
#[derive(Clone)]
pub struct Feed(Arc<Shared>);

impl Feed {
    /// `full`, `camera` or `snapshot`
    pub fn group(&self) -> &str {
        &self.0.group
    }

    /// Whether the group records this name at all: not a hidden one
    /// (rosbag2's `topic_is_hidden`), unless the group includes them.
    pub fn wants(&self, topic: &str) -> bool {
        self.0.include_hidden || !profile::is_hidden(topic)
    }

    /// Whether `topic_appeared` was called for this name.
    pub fn has_topic(&self, topic: &str) -> bool {
        self.0.lock().topics.contains(topic)
    }

    /// rosbag2's `create_topic`: the first writer of a name was seen. `false`
    /// for a hidden name, a name already created, or a stopped recording.
    pub fn topic_appeared(&self, topic: TopicMetadata) -> bool {
        if !self.wants(&topic.name) {
            return false;
        }
        {
            let mut q = self.0.lock();
            if q.closed || !q.topics.insert(topic.name.clone()) {
                return false;
            }
            q.items.push_back(Item::Topic(topic));
        }
        self.0.n.topics.fetch_add(1, Ordering::Relaxed);
        self.0.cv.notify_one();
        true
    }

    /// One message of a created topic: the receive time (`log_time`) and
    /// the source time (`publish_time`), ns since the epoch, and its CDR.
    /// Never blocks; `false` when it was dropped (see [`Stats`]).
    pub fn message(&self, topic: &str, recv_ns: i64, send_ns: i64, cdr: Vec<u8>) -> bool {
        let n = cdr.len() + topic.len();
        let mut warn = false;
        let ok = {
            let mut q = self.0.lock();
            if q.closed {
                self.0.n.dropped_closed.fetch_add(1, Ordering::Relaxed);
                false
            } else if !q.topics.contains(topic) {
                self.0.n.dropped_unknown.fetch_add(1, Ordering::Relaxed);
                false
            } else if q.bytes + n > self.0.capacity {
                self.0.n.dropped_full.fetch_add(1, Ordering::Relaxed);
                warn = !q.warned_full;
                q.warned_full = true;
                false
            } else {
                q.bytes += n;
                q.items.push_back(Item::Message { topic: topic.to_string(), recv: recv_ns, send: send_ns, cdr });
                true
            }
        };
        if ok {
            self.0.n.queued.fetch_add(1, Ordering::Relaxed);
            self.0.cv.notify_one();
        } else if warn {
            (self.0.log)(Level::Warn, &format!("bag-{}: the writer queue is full ({}), dropping messages", self.0.group, size_text(self.0.capacity)));
        }
        ok
    }

    /// Whether messages are still taken.
    pub fn is_open(&self) -> bool {
        !self.0.lock().closed
    }

    pub fn stats(&self) -> Stats {
        self.0.stats()
    }
}

fn size_text(n: usize) -> String {
    if n > 0 && n % (1 << 20) == 0 {
        format!("{} MiB", n >> 20)
    } else {
        format!("{n} bytes")
    }
}

/// How a group is recorded.
#[derive(Clone)]
pub struct Options {
    /// `full`, `camera` or `snapshot`: the thread is `bag-<group>`
    pub group: String,
    /// The bag directory (`<run>/bag`, ...); it must not exist yet.
    pub dir: PathBuf,
    pub bag: BagOptions,
    /// The queue's bound in bytes.
    pub queue_bytes: usize,
    /// `--include-hidden-topics`
    pub include_hidden: bool,
    pub log: Log,
}

impl Options {
    pub fn new(group: &str, dir: &Path, bag: BagOptions) -> Options {
        Options { group: group.to_string(), dir: dir.to_path_buf(), bag, queue_bytes: QUEUE_BYTES, include_hidden: false, log: no_log() }
    }
}

/// What a finished group wrote.
#[derive(Debug, Clone)]
pub struct Finished {
    pub group: String,
    /// What went into `metadata.yaml`.
    pub metadata: BagMetadata,
    pub stats: Stats,
}

/// A group being recorded.
pub struct Recording {
    feed: Feed,
    thread: Option<JoinHandle<io::Result<BagMetadata>>>,
}

impl Recording {
    /// Creates the bag (its directory and first file, as rosbag2 does when
    /// it starts) and the writer thread.
    pub fn start(o: Options) -> io::Result<Recording> {
        let bag = BagWriter::create(&o.dir, o.bag.clone())?;
        let shared = Arc::new(Shared {
            group: o.group.clone(),
            capacity: o.queue_bytes,
            include_hidden: o.include_hidden,
            q: Mutex::new(Queue { items: VecDeque::new(), bytes: 0, closed: false, topics: HashSet::new(), warned_full: false }),
            cv: Condvar::new(),
            n: Counters {
                topics: AtomicU64::new(0),
                queued: AtomicU64::new(0),
                written: AtomicU64::new(0),
                dropped_full: AtomicU64::new(0),
                dropped_closed: AtomicU64::new(0),
                dropped_unknown: AtomicU64::new(0),
                write_errors: AtomicU64::new(0),
            },
            log: o.log,
        });
        let s = shared.clone();
        let thread = std::thread::Builder::new().name(format!("bag-{}", o.group)).spawn(move || writer(s, bag))?;
        Ok(Recording { feed: Feed(shared), thread: Some(thread) })
    }

    pub fn group(&self) -> &str {
        self.feed.group()
    }

    /// The producers' handle.
    pub fn feed(&self) -> Feed {
        self.feed.clone()
    }

    /// The writer thread runs (what `proc.poll() is None` told the Python
    /// recorder about its `ros2 bag record` child).
    pub fn is_alive(&self) -> bool {
        self.thread.as_ref().is_some_and(|t| !t.is_finished())
    }

    /// Snapshot mode: writes the buffer and starts the next file
    /// (`/<node>/snapshot`, rosbag2's `take_snapshot`), after everything
    /// queued before. `Ok(false)` when the group is not in snapshot mode.
    pub fn take_snapshot(&self) -> io::Result<bool> {
        self.feed.0.request(Item::Snapshot)
    }

    /// Starts the next file now, after everything queued before (rosbag2's
    /// `split_bagfile` service).
    pub fn split(&self) -> io::Result<()> {
        self.feed.0.request(Item::Split)
    }

    /// No more input: what is queued is still written. The first half of
    /// [`Recording::stop`], for stopping several groups at once.
    pub fn close_input(&self) {
        self.feed.0.close();
    }

    pub fn stats(&self) -> Stats {
        self.feed.stats()
    }

    fn finish(&mut self) -> Option<io::Result<BagMetadata>> {
        let t = self.thread.take()?;
        self.close_input();
        Some(match t.join() {
            Ok(r) => r,
            Err(_) => Err(io::Error::other(format!("bag-{}: the writer thread panicked", self.feed.0.group))),
        })
    }

    /// Closes the input, writes what is queued and finalizes the bag.
    pub fn stop(mut self) -> io::Result<Finished> {
        let r = self.finish().expect("a recording is stopped once");
        let metadata = r?;
        Ok(Finished { group: self.feed.0.group.clone(), metadata, stats: self.feed.stats() })
    }
}

impl Drop for Recording {
    /// R5: a recording dropped without [`Recording::stop`] (a panic in the
    /// caller, a module shut down) is finalized all the same.
    fn drop(&mut self) {
        if let Some(Err(e)) = self.finish() {
            (self.feed.0.log)(Level::Error, &format!("bag-{}: finalize failed: {e}", self.feed.0.group));
        }
    }
}

/// `nice(10)`: on Linux it applies to the calling thread only.
fn lower_priority() {
    #[cfg(target_os = "linux")]
    unsafe {
        libc::nice(10);
    }
}

/// Marks the input closed however the writer ends, so that producers stop
/// queueing for a writer that is gone.
struct CloseOnExit(Arc<Shared>);

impl Drop for CloseOnExit {
    fn drop(&mut self) {
        let mut q = self.0.lock();
        q.closed = true;
        q.items.clear();
        q.bytes = 0;
    }
}

fn writer(s: Arc<Shared>, mut bag: BagWriter) -> io::Result<BagMetadata> {
    lower_priority();
    let _closed = CloseOnExit(s.clone());
    let mut first_error = true;
    loop {
        let item = {
            let mut q = s.lock();
            loop {
                if let Some(it) = q.items.pop_front() {
                    q.bytes -= it.bytes();
                    break Some(it);
                }
                if q.closed {
                    break None;
                }
                q = s.cv.wait(q).unwrap_or_else(|e| e.into_inner());
            }
        };
        let Some(item) = item else { break };
        let r = match item {
            Item::Topic(t) => bag.create_topic(t),
            Item::Message { topic, recv, send, cdr } => {
                let r = bag.write(&topic, recv, send, &cdr);
                if r.is_ok() {
                    s.n.written.fetch_add(1, Ordering::Relaxed);
                } else {
                    s.n.write_errors.fetch_add(1, Ordering::Relaxed);
                }
                r
            }
            Item::Snapshot(reply) => {
                let _ = reply.send(bag.take_snapshot());
                Ok(())
            }
            Item::Split(reply) => {
                let _ = reply.send(bag.split());
                Ok(())
            }
        };
        if let Err(e) = r {
            if first_error {
                first_error = false;
                (s.log)(Level::Error, &format!("bag-{}: write failed: {e}", s.group));
            }
        }
    }
    let st = s.stats();
    if st.dropped_full > 0 {
        (s.log)(Level::Warn, &format!("bag-{}: {} messages dropped, the writer queue was full", s.group, st.dropped_full));
    }
    if st.write_errors > 0 {
        (s.log)(Level::Error, &format!("bag-{}: {} messages failed to write", s.group, st.write_errors));
    }
    bag.close()
}

/// A topic as the source side reports it when it appears: the type, the
/// offered QoS of every writer known now (rmw's order) and the type hash
/// (`RIHS01_...`, or `""` without one).
pub fn topic_metadata(name: &str, type_name: &str, offers: &[Qos], type_hash: &str) -> TopicMetadata {
    TopicMetadata {
        name: name.to_string(),
        type_name: type_name.to_string(),
        serialization_format: "cdr".to_string(),
        offered_qos_profiles: offers.to_vec(),
        type_description_hash: type_hash.to_string(),
    }
}

/// An rmw duration in ns as rosbag2 stores it (`RMW_DURATION_INFINITE` =
/// `i64::MAX` ns is `{9223372036, 854775807}`).
pub fn duration_from_ns(ns: u64) -> (u64, u64) {
    (ns / 1_000_000_000, ns % 1_000_000_000)
}

/// What rclcpp leaves to the rmw: `system_default` liveliness and the
/// `{0, 0}` (default) durations, which Cyclone DDS reports as automatic and
/// infinite.
fn request(history: &str, depth: u64, reliability: &str, durability: &str) -> Qos {
    Qos {
        history: history.into(),
        depth,
        reliability: reliability.into(),
        durability: durability.into(),
        deadline: (0, 0),
        lifespan: (0, 0),
        liveliness: "system_default".into(),
        liveliness_lease_duration: (0, 0),
        avoid_ros_namespace_conventions: false,
    }
}

/// `Rosbag2QoS{}`: `rclcpp::QoS(rmw_qos_profile_default.depth)`, keep_last
/// 10, reliable, volatile. rosbag2 never subscribes to a name without a
/// writer (`include_unpublished_topics` is off), so the recorder does not
/// use it.
pub fn default_reader_qos() -> Qos {
    request("keep_last", 10, "reliable", "volatile")
}

/// rosbag2_storage qos.cpp `Rosbag2QoS::adapt_request_to_offers`: reliable
/// iff every offer is, transient local iff every offer is,
/// `keep_last(max offered depth)` if that depth is above 1, else
/// `keep_all` (depth 0, as rmw reports it).
pub fn adapt_request_to_offers(offers: &[Qos]) -> Qos {
    if offers.is_empty() {
        return default_reader_qos();
    }
    let all = |f: &dyn Fn(&Qos) -> bool| offers.iter().all(f);
    let depth = offers.iter().map(|q| q.depth).max().unwrap_or(0);
    let (history, depth) = if depth > 1 { ("keep_last", depth) } else { ("keep_all", 0) };
    request(
        history,
        depth,
        if all(&|q| q.reliability == "reliable") { "reliable" } else { "best_effort" },
        if all(&|q| q.durability == "transient_local") { "transient_local" } else { "volatile" },
    )
}

/// rosbag2's `subscription_qos_for_topic`: the override for the name if
/// there is one, else [`adapt_request_to_offers`] of the writers known when
/// it subscribes.
pub fn reader_qos(topic: &str, offers: &[Qos], overrides: &QosOverrides) -> Qos {
    match overrides.get(topic) {
        Some(q) => q.clone(),
        None => adapt_request_to_offers(offers),
    }
}

/// `--qos-profile-overrides-path`: per name, the reader QoS that replaces
/// the adapted one.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct QosOverrides(Vec<(String, Qos)>);

impl QosOverrides {
    pub fn empty() -> QosOverrides {
        QosOverrides(Vec::new())
    }

    pub fn get(&self, topic: &str) -> Option<&Qos> {
        self.0.iter().find(|(t, _)| t == topic).map(|(_, q)| q)
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// The names, in the file's order.
    pub fn topics(&self) -> impl Iterator<Item = &str> {
        self.0.iter().map(|(t, _)| t.as_str())
    }

    /// The file `ros2 bag record` reads (`yaml.safe_load`, then ros2bag's
    /// `convert_yaml_to_qos_profile`). An `Err` is where `ros2 bag record`
    /// would have exited.
    pub fn load(path: &Path) -> Result<QosOverrides, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        QosOverrides::parse(&text)
    }

    /// ros2bag `convert_yaml_to_qos_profile`: a mapping of names to
    /// profiles, each read by `interpret_dict_as_qos_profile` (short keys,
    /// any case) into rclpy's `QoSProfile` (history keep_last when only a
    /// depth is given, depth 10, reliable and volatile by default).
    pub fn parse(text: &str) -> Result<QosOverrides, String> {
        let doc = mower_pyfmt::yaml::safe_load(text).map_err(|e| e.to_string())?;
        let Py::Dict(items) = doc else {
            return Err(format!("'{}' object has no attribute 'items'", doc.type_name()));
        };
        let mut out = Vec::new();
        for (topic, profile) in items {
            let q = qos_profile(&profile)?;
            match out.iter_mut().find(|(t, _): &&mut (String, Qos)| *t == topic) {
                Some(slot) => slot.1 = q,
                None => out.push((topic, q)),
            }
        }
        Ok(QosOverrides(out))
    }
}

/// `Policy.get_from_short_key(value)`: `cls[name.upper()]`.
fn short_key(policy: &str, value: &Py, names: &[&str]) -> Result<String, String> {
    let Py::Str(s) = value else {
        return Err(format!("{policy}: '{}' object has no attribute 'upper'", value.type_name()));
    };
    let lower = s.to_lowercase();
    if names.contains(&lower.as_str()) {
        Ok(lower)
    } else {
        Err(format!("KeyError: '{}'", s.to_uppercase()))
    }
}

/// `dict_to_duration`: `{sec, nsec}`, or the default for a falsy value.
fn duration(value: &Py) -> Result<(u64, u64), String> {
    if !profile::truthy(Some(value)) {
        return Ok((0, 0));
    }
    let (Some(sec), Some(nsec)) = (value.get("sec"), value.get("nsec")) else {
        return Err("Time overrides must include both seconds (sec) and nanoseconds (nsec).".into());
    };
    let n = |v: &Py| match v {
        Py::Int(i) if *i >= 0 => Ok(*i as u64),
        Py::Bool(b) => Ok(*b as u64),
        other => Err(format!("a duration needs non-negative integers, not {}", other.type_name())),
    };
    let total = n(sec)? as u128 * 1_000_000_000 + n(nsec)? as u128;
    Ok(((total / 1_000_000_000) as u64, (total % 1_000_000_000) as u64))
}

/// `interpret_dict_as_qos_profile` + `QoSProfile(**d)`.
fn qos_profile(profile: &Py) -> Result<Qos, String> {
    let Py::Dict(items) = profile else {
        return Err(format!("'{}' object has no attribute 'items'", profile.type_name()));
    };
    let mut q = request("keep_last", 10, "reliable", "volatile");
    let (mut history, mut depth) = (None, None);
    for (k, v) in items {
        match k.as_str() {
            "history" => history = Some(short_key(k, v, &["keep_last", "keep_all", "system_default", "unknown"])?),
            "reliability" => q.reliability = short_key(k, v, &["reliable", "best_effort", "system_default", "best_available", "unknown"])?,
            "durability" => q.durability = short_key(k, v, &["volatile", "transient_local", "system_default", "best_available", "unknown"])?,
            "liveliness" => q.liveliness = short_key(k, v, &["automatic", "manual_by_topic", "system_default", "best_available", "unknown"])?,
            "depth" => {
                depth = Some(match v {
                    Py::Int(i) if *i >= 0 => *i as u64,
                    Py::Bool(b) => *b as u64,
                    other => return Err(format!("depth: an int is needed, not {}", other.type_name())),
                })
            }
            "deadline" => q.deadline = duration(v)?,
            "lifespan" => q.lifespan = duration(v)?,
            "liveliness_lease_duration" => q.liveliness_lease_duration = duration(v)?,
            "avoid_ros_namespace_conventions" => q.avoid_ros_namespace_conventions = profile::truthy(Some(v)),
            other => return Err(format!("Unexpected key `{other}` for QoS profile.")),
        }
    }
    let history = match (history, depth) {
        (None, None) => return Err("History and/or depth settings are required.".into()),
        (None, Some(_)) => "keep_last".to_string(),
        (Some(h), None) if h == "keep_last" => return Err("History set to KEEP_LAST without a depth setting.".into()),
        (Some(h), _) => h,
    };
    q.depth = if history == "keep_all" { 0 } else { depth.unwrap_or(10) };
    q.history = history;
    Ok(q)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmp(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("mower_recording_{name}_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    fn string_topic(name: &str) -> TopicMetadata {
        topic_metadata(name, "std_msgs/msg/String", &[Qos::new("keep_last", 1, "reliable", "volatile")], "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18")
    }

    #[test]
    fn an_overflow_drops_counts_and_warns_once() {
        let d = tmp("overflow");
        let logged = Arc::new(Mutex::new(Vec::new()));
        let l = logged.clone();
        let mut o = Options::new("test", &d.join("bag"), BagOptions::new("t"));
        o.queue_bytes = 100;
        o.log = Arc::new(move |lv, m| l.lock().unwrap().push((lv, m.to_string())));
        let rec = Recording::start(o).unwrap();
        let f = rec.feed();
        assert!(!f.message("/a", 1, 1, vec![0; 8]), "before its topic appeared");
        assert!(f.topic_appeared(string_topic("/a")));
        assert!(!f.topic_appeared(string_topic("/a")), "created once");
        assert!(!f.topic_appeared(string_topic("/x/_action/status")), "hidden");
        for _ in 0..3 {
            assert!(!f.message("/a", 2, 2, vec![0; 200]));
        }
        assert!(f.message("/a", 3, 3, vec![0, 1, 0, 0, 2, 0, 0, 0, 104, 0, 0, 0]));
        let fin = rec.stop().unwrap();
        assert!(!f.message("/a", 4, 4, vec![0; 4]), "after the stop");
        assert_eq!(fin.stats, Stats { topics: 1, queued: 1, written: 1, dropped_full: 3, dropped_closed: 0, dropped_unknown: 1, write_errors: 0 });
        assert_eq!(f.stats().dropped_closed, 1);
        assert!(!f.is_open() && !f.topic_appeared(string_topic("/b")));
        assert_eq!(fin.metadata.message_count, 1);
        let warns: Vec<String> = logged.lock().unwrap().iter().filter(|(l, _)| *l == Level::Warn).map(|(_, m)| m.clone()).collect();
        assert_eq!(warns, ["bag-test: the writer queue is full (100 bytes), dropping messages", "bag-test: 3 messages dropped, the writer queue was full"]);
        assert!(d.join("bag/metadata.yaml").is_file());
        std::fs::remove_dir_all(&d).unwrap();
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn the_writer_thread_is_named_and_niced() {
        // field 19 of /proc/<tid>/stat (after the parenthesised name)
        fn nice_of(stat: &str) -> i64 {
            stat.rsplit_once(") ").unwrap().1.split(' ').nth(16).unwrap().parse().unwrap()
        }
        let d = tmp("nice");
        let rec = Recording::start(Options::new("nicetest", &d.join("bag"), BagOptions::new("t"))).unwrap();
        let mine = nice_of(&std::fs::read_to_string("/proc/thread-self/stat").unwrap());
        let mut found = None;
        for _ in 0..100 {
            for e in std::fs::read_dir("/proc/self/task").unwrap().flatten() {
                if std::fs::read_to_string(e.path().join("comm")).unwrap_or_default().trim() == "bag-nicetest" {
                    let n = nice_of(&std::fs::read_to_string(e.path().join("stat")).unwrap());
                    if n != mine {
                        found = Some(n);
                    }
                }
            }
            if found.is_some() {
                break;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        assert_eq!(found, Some((mine + 10).min(19)));
        rec.stop().unwrap();
        std::fs::remove_dir_all(&d).unwrap();
    }

    #[test]
    fn the_override_file_as_ros2bag_reads_it() {
        let o = QosOverrides::parse("/a:\n  history: keep_last\n  depth: 1\n  reliability: reliable\n  durability: transient_local\n/b:\n  depth: 5\n  reliability: BEST_EFFORT\n/c:\n  history: keep_all\n  deadline: {sec: 1, nsec: 2}\n").unwrap();
        assert_eq!(o.topics().collect::<Vec<_>>(), ["/a", "/b", "/c"]);
        assert_eq!(o.get("/a").unwrap(), &request("keep_last", 1, "reliable", "transient_local"));
        assert_eq!(o.get("/b").unwrap(), &request("keep_last", 5, "best_effort", "volatile"));
        assert_eq!(o.get("/c").unwrap(), &Qos { deadline: (1, 2), ..request("keep_all", 0, "reliable", "volatile") });
        assert_eq!(QosOverrides::parse("/a:\n  history: keep_last\n").unwrap_err(), "History set to KEEP_LAST without a depth setting.");
        assert_eq!(QosOverrides::parse("/a:\n  reliability: reliable\n").unwrap_err(), "History and/or depth settings are required.");
        assert_eq!(QosOverrides::parse("/a:\n  depth: 1\n  colour: red\n").unwrap_err(), "Unexpected key `colour` for QoS profile.");
        assert_eq!(QosOverrides::parse("/a:\n  depth: 1\n  deadline: {sec: 1}\n").unwrap_err(), "Time overrides must include both seconds (sec) and nanoseconds (nsec).");
        assert!(QosOverrides::parse("/a:\n  depth: 1\n  durability: sticky\n").is_err());
        assert_eq!(reader_qos("/a", &[], &o), request("keep_last", 1, "reliable", "transient_local"));
        assert_eq!(reader_qos("/z", &[], &o), default_reader_qos());
        let offers = [Qos::new("keep_last", 1, "reliable", "transient_local"), Qos::new("keep_last", 100, "best_effort", "transient_local")];
        assert_eq!(reader_qos("/z", &offers, &o), request("keep_last", 100, "best_effort", "transient_local"));
        assert_eq!(adapt_request_to_offers(&offers[..1]), request("keep_all", 0, "reliable", "transient_local"));
        assert_eq!(duration_from_ns(i64::MAX as u64), mower_mcap::INFINITE);
    }
}
