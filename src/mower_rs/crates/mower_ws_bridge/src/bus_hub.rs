//! The bus backend (step E3): what the clients reach, on mower_bus instead
//! of on an r2r node of ws_bridge's own. Names that are only on DDS (the
//! modules still on r2r, the recorder) reach it through mower_rsd's bus
//! bridge, which the manifest's `bridge` module plans for.
//!
//! - **A topic** a client subscribes to is an entry with a child scope of
//!   the module's, and in it one erased reader, R TL 10: a writer's
//!   retained history reaches it, and so the first client. A task frames
//!   each message once for all clients, and only while one wants it.
//! - **The cache follows the writers** (E3 decision 4): the entry replays to
//!   a later client while every live writer keeps history, and forgets the
//!   last message when the last writer goes.
//! - **The entry goes 10 s after its last client** (decision 5), and its
//!   reader with it, so that no name is read for nobody.
//! - **Only known names** (decision 7): a topic of ws_bridge's manifest
//!   entry, or one a typed endpoint has fixed; the bus never forgets a name.
//! - **No NUL** (decision 8): a client string that holds one is refused
//!   before it reaches a module or the bridge's r2r conversions.
//! - **A publish** goes through a JSON writer, R V 10, made at the first
//!   publish and kept; **a call** through a JSON client, made at the first
//!   call and kept. A call waits 5 s for a server, and fails at once when
//!   the server goes while it holds the call (decision 15).
//! - **`/rosapi/topics`** lists the typed names with a live endpoint, and
//!   the typed names of the manifest entry with or without one.
//! - **Time** is the module's clock: the throttle, the 5 s and 30 s bounds
//!   and the linger.

use std::collections::{BTreeSet, HashMap};
use std::pin::Pin;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError, Weak};
use std::time::Duration;

use futures::StreamExt;
use mower_bus::{BusError, Clock, ErasedMsg, ErasedSubscriber, JsonClient, JsonPublisher, Qos, Scope};
use mower_rs_common::ModuleCtx;
use serde_json::Value;
use tokio::sync::Notify;

use crate::client::ClientHandle;
use crate::entry::{publish_frame, Frame, TopicEntry};

/// How long a call waits for a server, as the r2r backend's `is_available`.
const AVAILABILITY_TIMEOUT: Duration = Duration::from_secs(5);
/// How long a topic's entry and its reader stay after the last client.
pub const LINGER: Duration = Duration::from_secs(10);
/// How often a call looks again for a service name no endpoint has declared.
const NAME_POLL: Duration = Duration::from_millis(100);
/// A topic's reader: R TL 10 (the manifest's `bridge` module).
const READER_QOS: Qos = Qos::latched().keep_last(10);
/// What clients publish: R V 10, as the r2r backend's untyped writers.
const WRITER_QOS: Qos = Qos::default();

/// The topics ws_bridge's manifest entry reads (mower_bridge, module
/// `bridge`), under the names rcl gives them on ws_bridge's node: what a
/// client may subscribe to before a typed endpoint has fixed the type.
pub fn manifest_names(m: &ModuleCtx) -> Result<BTreeSet<String>, String> {
    let module = mower_bridge::module("bridge").ok_or("mower_bridge's manifest has no module bridge")?;
    let subscribed = module.instances.iter().flat_map(|i| i.endpoints).filter(|e| e.kind == mower_bridge::types::Kind::Subscribe);
    subscribed.map(|e| m.resolve_topic(e.code).map_err(|err| format!("{}: {err}", e.code))).collect()
}

#[derive(Clone)]
pub struct BusHub {
    inner: Arc<Inner>,
}

struct Inner {
    /// The module's scope: its JSON writers and clients are made in it, each
    /// topic's reader in a child of it of the same name.
    scope: Scope,
    clock: Clock,
    known: BTreeSet<String>,
    logger: String,
    state: Mutex<State>,
    /// Messages framed so far, over all topics.
    framed: AtomicU64,
}

#[derive(Default)]
struct State {
    topics: HashMap<String, Topic>,
    publishers: HashMap<String, JsonPublisher>,
    clients: HashMap<String, JsonClient>,
}

struct Topic {
    entry: Arc<TopicEntry>,
    /// The scope of the topic's reader, closed when the entry goes.
    scope: Scope,
}

impl Inner {
    /// A poisoned lock still holds consistent maps: nothing under it panics
    /// half-way.
    fn lock(&self) -> MutexGuard<'_, State> {
        self.state.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// `entry` leaves the hub if it still has no client, and the scope of
    /// its reader is handed back to be closed. Under the lock subscribes
    /// attach their clients under, so no client attaches to it after.
    fn retire(&self, entry: &Arc<TopicEntry>) -> Option<Scope> {
        let mut state = self.lock();
        let current = state.topics.get(&entry.topic)?;
        if !Arc::ptr_eq(&current.entry, entry) || entry.client_count() != 0 {
            return None;
        }
        state.topics.remove(&entry.topic).map(|topic| topic.scope)
    }
}

impl BusHub {
    /// The hub of the module whose scope is `scope` and clock `clock`;
    /// `known` are the topic names of its manifest entry.
    pub fn new(scope: Scope, clock: Clock, known: BTreeSet<String>, logger: &str) -> BusHub {
        let inner = Inner { scope, clock, known, logger: logger.to_string(), state: Mutex::default(), framed: AtomicU64::new(0) };
        BusHub { inner: Arc::new(inner) }
    }

    /// Messages framed so far, over all topics: one per message, however
    /// many clients get it.
    #[cfg(test)]
    pub fn framed(&self) -> u64 {
        self.inner.framed.load(Ordering::Relaxed)
    }

    /// The entry of `topic`, while it has one.
    #[cfg(test)]
    pub fn entry(&self, topic: &str) -> Option<Arc<TopicEntry>> {
        self.inner.lock().topics.get(topic).map(|t| t.entry.clone())
    }

    /// The topic's entry, made if it is new, with `client` attached.
    pub fn subscribe(&self, topic: &str, type_hint: Option<String>, client: &ClientHandle, throttle_ms: u64) -> Result<Arc<TopicEntry>, String> {
        let inner = &self.inner;
        let info = inner.scope.bus().topic(topic);
        let rust_type = info.as_ref().and_then(|t| t.type_name);
        if rust_type.is_none() && !inner.known.contains(topic) {
            return Err(format!("cannot infer type of {topic}: not advertised yet"));
        }
        let ros_type = rust_type.map(|t| mower_bus::ros_type_name(t).unwrap_or_else(|| t.to_string()));
        let hint = type_hint.filter(|h| !h.is_empty());
        if let (Some(hint), Some(ros_type)) = (&hint, &ros_type) {
            if hint != ros_type {
                return Err(format!("subscribe {topic} ({hint}) failed: the topic is {ros_type}"));
            }
        }
        // As DDS matches a transient-local reader: history only while every
        // writer keeps it.
        let latched = info.as_ref().is_some_and(|t| !t.publisher_qos.is_empty() && t.publisher_qos.iter().all(|q| q.is_transient_local()));
        let shown = ros_type.or(hint).unwrap_or_else(|| "?".to_string());
        let mut state = inner.lock();
        let entry = match state.topics.get(topic) {
            Some(existing) => existing.entry.clone(),
            None => {
                let scope = inner.scope.child(inner.scope.name());
                let reader = match scope.subscribe_erased(topic, READER_QOS) {
                    Ok(reader) => reader,
                    Err(e) => {
                        drop(state);
                        scope.close();
                        return Err(format!("subscribe {topic} ({shown}) failed: {e}"));
                    }
                };
                let changed = Arc::new(Notify::new());
                let entry = Arc::new(TopicEntry::with_clock(topic, &shown, latched, inner.clock.clone(), Some(changed.clone())));
                r2r::log_info!(&inner.logger, "subscribed {} [{}]{}", topic, shown, if latched { " latched" } else { "" });
                tokio::spawn(pump(Arc::downgrade(&self.inner), entry.clone(), reader, changed));
                state.topics.insert(topic.to_string(), Topic { entry: entry.clone(), scope });
                entry
            }
        };
        entry.set_latched(latched);
        entry.add_client(client.clone(), throttle_ms);
        Ok(entry)
    }

    pub fn publish(&self, topic: &str, msg: Value) -> Result<(), String> {
        if let Some(at) = nul_at(&msg) {
            return Err(format!("publish {topic} failed: {} holds a NUL character", shown_pointer(&at)));
        }
        let inner = &self.inner;
        let publisher = {
            let mut state = inner.lock();
            match state.publishers.get(topic) {
                Some(publisher) => publisher.clone(),
                None => {
                    let rust_type = inner.scope.bus().topic(topic).and_then(|t| t.type_name);
                    let Some(rust_type) = rust_type else {
                        return Err(format!("cannot infer type of {topic}: not advertised yet"));
                    };
                    let ros_type = mower_bus::ros_type_name(rust_type).unwrap_or_else(|| rust_type.to_string());
                    let publisher = inner
                        .scope
                        .publisher_json(topic, WRITER_QOS)
                        .map_err(|e| format!("advertise {topic} ({ros_type}) failed: {e}"))?;
                    r2r::log_info!(&inner.logger, "advertised {} [{}]", topic, ros_type);
                    state.publishers.insert(topic.to_string(), publisher.clone());
                    publisher
                }
            }
        };
        publisher.publish(&msg).map_err(|e| match e {
            BusError::UnknownName { .. } => format!("cannot infer type of {topic}: not advertised yet"),
            BusError::Json { error, .. } => format!("publish {topic} failed: {error}"),
            other => format!("publish {topic} failed: {other}"),
        })
    }

    /// Call `service`, bounded by `timeout` as a whole, the wait for a
    /// server included.
    pub async fn call_service(&self, service: &str, args: Value, timeout: Duration) -> Result<Value, String> {
        match self.inner.clock.timeout(timeout, self.call(service, args)).await {
            Ok(result) => result,
            Err(_) => Err(format!("service {service} did not answer within {} s", timeout.as_secs())),
        }
    }

    async fn call(&self, service: &str, args: Value) -> Result<Value, String> {
        if let Some(at) = nul_at(&args) {
            return Err(format!("{service}: request failed: {} holds a NUL character", shown_pointer(&at)));
        }
        let clock = &self.inner.clock;
        let deadline = clock.mono() + AVAILABILITY_TIMEOUT;
        // A name no endpoint has declared yet (the bus bridge declares the
        // names on DDS when it starts) is waited for like a server.
        let client = loop {
            match self.client(service) {
                Ok(client) => break client,
                Err(BusError::UnknownName { .. }) if clock.mono() < deadline => clock.sleep(NAME_POLL).await,
                Err(BusError::UnknownName { .. }) => return Err(format!("{service}: service unavailable")),
                Err(e) => return Err(format!("{service}: {e}")),
            }
        };
        match clock.timeout_at(deadline, client.wait_for_service()).await {
            Ok(Ok(())) => {}
            Ok(Err(e)) => return Err(format!("{service}: {e}")),
            Err(_) => return Err(format!("{service}: service unavailable")),
        }
        client.call(args).await.map_err(|e| match e {
            BusError::Json { error, .. } => format!("{service}: request failed: {error}"),
            // No server any more, or it dropped the call: at once, where the
            // r2r backend waited for its 30 s.
            BusError::ServiceUnavailable { .. } | BusError::NoResponse { .. } => format!("{service}: service unavailable"),
            other => format!("{service}: {other}"),
        })
    }

    /// The service's JSON client, made at the first call and kept.
    fn client(&self, service: &str) -> Result<JsonClient, BusError> {
        let mut state = self.inner.lock();
        if let Some(client) = state.clients.get(service) {
            return Ok(client.clone());
        }
        let client = self.inner.scope.client_json(service)?;
        state.clients.insert(service.to_string(), client.clone());
        Ok(client)
    }

    /// `/rosapi/topics` from the bus: every name with a fixed type that has a
    /// live endpoint or is a name of ws_bridge's manifest entry, typed by its
    /// ROS type. The manifest's names count with no endpoint: a
    /// transient-local name the bus bridge brings in from DDS has a bus
    /// writer only while somebody reads it, where the r2r backend listed it
    /// while its DDS writer lived and, once subscribed, for good.
    pub fn topics_and_types(&self) -> HashMap<String, Vec<String>> {
        let topics = self.inner.scope.bus().topics().into_iter();
        topics
            .filter(|t| self.inner.known.contains(&t.name) || !t.publishers.is_empty() || !t.subscribers.is_empty())
            .filter_map(|t| Some((t.name, vec![t.ros_type?])))
            .collect()
    }

    /// The module clock's wall time.
    pub fn now(&self) -> Duration {
        self.inner.clock.now().to_system().duration_since(std::time::UNIX_EPOCH).unwrap_or_default()
    }

    /// End every topic's reader.
    pub fn shutdown(&self) {
        let topics: Vec<Topic> = self.inner.lock().topics.drain().map(|(_, topic)| topic).collect();
        for topic in topics {
            topic.scope.close();
        }
    }
}

/// One topic's messages to its entry, until its scope closes: framed only
/// for clients (`TopicEntry::offer`), the cache cleared when the last writer
/// goes, and the entry retired `LINGER` after its last client.
async fn pump(hub: Weak<Inner>, entry: Arc<TopicEntry>, mut reader: ErasedSubscriber, changed: Arc<Notify>) {
    let Some((clock, logger)) = hub.upgrade().map(|inner| (inner.clock.clone(), inner.logger.clone())) else { return };
    let mut writers = reader.publisher_changes();
    let mut linger: Option<Pin<Box<mower_bus::Sleep>>> = None;
    loop {
        tokio::select! {
            msg = reader.recv() => match msg {
                Some(msg) => entry.offer(frame(&hub, &entry.topic, msg)),
                // its scope closed: the module ends
                None => return,
            },
            count = writers.next() => match count {
                Some(0) => {
                    // What is queued was published while a writer lived.
                    while let Ok(msg) = reader.try_recv() {
                        entry.offer(frame(&hub, &entry.topic, msg));
                    }
                    if reader.publisher_count() == 0 {
                        entry.clear_cache();
                    }
                }
                Some(_) => {}
                None => return,
            },
            () = changed.notified() => {
                linger = (entry.client_count() == 0).then(|| Box::pin(clock.sleep(LINGER)));
            }
            () = async {
                match linger.as_mut() {
                    Some(sleep) => sleep.await,
                    None => std::future::pending().await,
                }
            } => {
                linger = None;
                let Some(inner) = hub.upgrade() else { return };
                if let Some(scope) = inner.retire(&entry) {
                    r2r::log_info!(&logger, "unsubscribed {} (no client for {} s)", entry.topic, LINGER.as_secs());
                    drop(inner);
                    scope.close();
                    return;
                }
            }
        }
    }
}

/// `msg`'s frame, made when the entry wants it.
fn frame(hub: &Weak<Inner>, topic: &str, msg: ErasedMsg) -> Frame {
    let (hub, topic) = (hub.clone(), topic.to_string());
    Box::new(move || {
        let inner = hub.upgrade();
        match msg.to_json() {
            Ok(json) => {
                if let Some(inner) = &inner {
                    inner.framed.fetch_add(1, Ordering::Relaxed);
                }
                Some(publish_frame(&topic, &json))
            }
            Err(e) => {
                if let Some(inner) = &inner {
                    r2r::log_warn!(&inner.logger, "{}: bad message: {}", topic, e);
                }
                None
            }
        }
    })
}

/// Where `value` holds a string with a NUL character: its JSON pointer.
fn nul_at(value: &Value) -> Option<String> {
    match value {
        Value::String(s) => s.contains('\0').then(String::new),
        Value::Array(items) => items.iter().enumerate().find_map(|(i, v)| nul_at(v).map(|at| format!("/{i}{at}"))),
        Value::Object(fields) => fields
            .iter()
            .find_map(|(k, v)| nul_at(v).map(|at| format!("/{}{at}", k.replace('~', "~0").replace('/', "~1")))),
        _ => None,
    }
}

fn shown_pointer(at: &str) -> &str {
    if at.is_empty() {
        "the message"
    } else {
        at
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn nul_is_found_anywhere_in_a_value() {
        assert_eq!(nul_at(&json!({"data": "a\u{0}b"})).as_deref(), Some("/data"));
        assert_eq!(nul_at(&json!({"parameters": [{"name": "x"}, {"name": "y\u{0}"}]})).as_deref(), Some("/parameters/1/name"));
        assert_eq!(nul_at(&json!({"a/b": {"c~d": "\u{0}"}})).as_deref(), Some("/a~1b/c~0d"));
        assert_eq!(nul_at(&json!("\u{0}")).as_deref(), Some(""));
        assert_eq!(nul_at(&json!({"data": "fine", "n": 0, "list": [1, null, true]})), None);
    }
}
