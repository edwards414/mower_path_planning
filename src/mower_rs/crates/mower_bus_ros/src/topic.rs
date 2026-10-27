//! Topic forwarders: `to_ros` (the bus's writers to one DDS writer) and
//! `from_ros` (one DDS reader to one bus writer). Contract: README.md,
//! "Topics".
//!
//! `to_ros` keeps the DDS writer's lifetime and, for a transient-local
//! topic, its history in line with the bus writers (docs/BUS_TOPICS.md 5.5
//! b). Its view of the bus writers is a `publisher_changes` stream, which
//! coalesces and is not ordered with the message queue, so every item is a
//! "resync": the forwarder looks at the bus as it is now, not at what the
//! item says changed.
//! - **Volatile:** the DDS writer exists while some bus writer is live. A
//!   resync first forwards what is queued (it was published before the
//!   change), then creates or destroys the writer. A message is published
//!   only while DDS has a matched reader.
//! - **Transient local:** a resync also replaces the bus reader with a fresh
//!   one, which starts with exactly the retained history of the writers live
//!   now; the old reader may still hold messages of writers that are gone.
//!   If, once given the messages still queued, the DDS writer would hold
//!   something that history no longer has (or lack something it has), it
//!   is destroyed and created again with the history, so a DDS late joiner
//!   never gets a value the bus has dropped. A held value that a queued one
//!   supersedes is no reason: the writer stays. With no live bus writer,
//!   nothing is published and no DDS writer exists.
//!
//! The bus reader stands in for every DDS reader, so its queue is at least
//! `READER_DEPTH` deep, whatever the source's depth: with the source's depth
//! of 1, two writers publishing at the same moment (`/mower_base/led_command`
//! has two), or a module publishing twice in a row, would reach DDS as one
//! message. The source's depth is what the DDS writer keeps, which is what
//! DDS readers see of it.
//!
//! A message r2r cannot convert (convert.rs) is dropped. Transient local, it
//! is also left out of every history the DDS writer is given from then on,
//! as if it had never been published: a DDS late joiner gets what the other
//! writers retain, and none of the values the dropped one superseded.
//!
//! `from_ros` mirrors the bus readers' presence with its DDS reader, and,
//! for a transient-local name, the DDS writers' with its bus writer: the
//! history a bus late joiner gets exists only while the graph shows a DDS
//! writer and the DDS reader hears it (README.md, "Graph presence").

use std::collections::VecDeque;
use std::sync::{Arc, Weak};

use futures::{FutureExt, StreamExt};
use mower_bus::{BusError, Publisher, PublisherChanges, Qos, Scope, Subscriber, SubscriptionChanges};
use r2r::QosProfile;

use crate::convert::{convertible, guarded};
use crate::log::{Failure, Log};
use crate::node::{NodeHandle, DWELL, PRESENCE_PERIOD};
use crate::stats::Counters;
use crate::{BridgeError, BridgeMessage, Forwarder};

/// The least queue of a `to_ros` bus reader: what a forwarder that waits on
/// the spin thread (one spin, 20 ms) must hold without dropping, with room
/// for bursts.
const READER_DEPTH: usize = 64;

/// What a forwarder needs besides its endpoints.
pub(crate) struct Env {
    pub node: NodeHandle,
    pub counters: Arc<Counters>,
    pub log: Log,
}

/// `to_ros`: the bus endpoints now, so that the name and the type are
/// checked at once and no writer change is missed; the DDS writer once the
/// bridge runs.
pub(crate) fn to_ros<T: BridgeMessage>(
    scope: &Scope, name: &str, dds_qos: QosProfile, bus_qos: Qos, env: Env,
) -> Result<Forwarder, BridgeError> {
    let reader_qos = bus_qos.keep_last(bus_qos.effective_depth().max(READER_DEPTH));
    let (reader, changes) = if bus_qos.is_transient_local() {
        // The watch comes from a reader of the bridge's own scope, so that it
        // ends when that scope closes. That reader goes at once (dropping it
        // empties its queue, which the watch keeps); the readers that carry
        // messages each live in a child scope (`Reader`).
        let sentinel = scope.subscribe::<T>(name, Qos::default().keep_last(1))?;
        let changes = sentinel.publisher_changes();
        drop(sentinel);
        (Reader::latched(scope, name, reader_qos)?, changes)
    } else {
        let reader = scope.subscribe::<T>(name, reader_qos)?;
        let changes = reader.publisher_changes();
        (Reader { sub: reader, own_scope: None }, changes)
    };
    let forwarder = ToRos {
        name: name.to_string(),
        dds_qos,
        depth: bus_qos.effective_depth(),
        latched: bus_qos.is_transient_local(),
        reader_qos,
        scope: scope.clone(),
        changes,
        reader,
        dds: None,
        sent: VecDeque::new(),
        failed: VecDeque::new(),
        dropped_before: 0,
        env,
    };
    Ok(Box::pin(forwarder.run()))
}

/// The bus reader a `to_ros` forwarder takes messages from.
struct Reader<T: BridgeMessage> {
    sub: Subscriber<T>,
    /// Transient local: the child scope this reader alone lives in, closed
    /// when the reader is replaced. A child takes no close hook in the
    /// bridge's scope, where a reader of that scope would leave one at every
    /// resync (mower_bus README, "Hooks of dropped endpoints"); and it closes
    /// with that scope, or comes out closed once that scope is closing.
    own_scope: Option<Scope>,
}

impl<T: BridgeMessage> Reader<T> {
    fn latched(scope: &Scope, name: &str, qos: Qos) -> Result<Self, BusError> {
        let own = scope.child(scope.name());
        let sub = own.subscribe::<T>(name, qos)?;
        Ok(Reader { sub, own_scope: Some(own) })
    }
}

impl<T: BridgeMessage> Drop for Reader<T> {
    fn drop(&mut self) {
        if let Some(scope) = &self.own_scope {
            scope.close();
        }
    }
}

struct ToRos<T: BridgeMessage> {
    name: String,
    dds_qos: QosProfile,
    /// What the DDS writer keeps: the source's KEEP_LAST depth.
    depth: usize,
    latched: bool,
    reader_qos: Qos,
    /// The bridge's scope: its close ends `changes`, and with it the
    /// forwarder.
    scope: Scope,
    changes: PublisherChanges,
    reader: Reader<T>,
    dds: Option<r2r::Publisher<T>>,
    /// Transient local: what this DDS writer was given, oldest first, up to
    /// the reader's depth. Its newest `depth` are what the writer holds; the
    /// rest tells, at a resync, the messages given already from the queued
    /// ones (`keep_writer`). Identity, not value, decides: the bus hands
    /// every reader the publisher's own `Arc`. `Weak`, so that a map the bus
    /// has dropped is not kept alive here.
    sent: VecDeque<Weak<T>>,
    /// Transient local: the messages r2r could not convert, by identity, up
    /// to the reader's depth, which a fresh reader's history is given to DDS
    /// without (`reject`).
    failed: VecDeque<Weak<T>>,
    /// What the readers a latched resync replaced had lost to a full queue.
    dropped_before: u64,
    env: Env,
}

impl<T: BridgeMessage> ToRos<T> {
    async fn run(mut self) {
        self.sync_presence().await;
        loop {
            tokio::select! {
                // Writer changes first: a transient-local message queued
                // before a change may come from a writer that is gone.
                biased;
                change = self.changes.next() => match change {
                    Some(_) => self.resync().await,
                    // The bridge's scope closed.
                    None => return,
                },
                msg = self.reader.sub.next() => match msg {
                    Some(msg) => self.on_message(msg).await,
                    None => return,
                },
            }
            self.env.counters.set_dropped(self.dropped_before + self.reader.sub.dropped());
        }
    }

    fn bus_writers(&self) -> usize {
        self.reader.sub.publisher_count()
    }

    async fn on_message(&mut self, msg: Arc<T>) {
        if self.dds.is_none() {
            // Either its writer is gone (and the change that says so is next
            // in line), or a new writer published before its arrival was
            // handled.
            if self.bus_writers() == 0 {
                self.env.counters.stale(1);
                return;
            }
            // A message r2r cannot convert makes no DDS writer.
            if let Err(why) = convertible(&*msg) {
                return self.reject(&msg, why);
            }
            self.create().await;
        }
        if !self.publish(&msg) && self.latched {
            // On the bus it superseded what the DDS writer may still hold.
            // A fresh history goes to DDS without it, as one does when a
            // writer goes: what the other writers retain stays, what its own
            // writer retained before it does not.
            self.resync_latched().await;
        }
    }

    async fn resync(&mut self) {
        self.env.counters.resynced();
        if self.latched {
            self.resync_latched().await;
        } else {
            while let Ok(msg) = self.reader.sub.try_recv() {
                self.on_message(msg).await;
            }
            self.sync_presence().await;
        }
    }

    async fn resync_latched(&mut self) {
        let fresh = match Reader::latched(&self.scope, &self.name, self.reader_qos) {
            Ok(fresh) => fresh,
            // The bridge's scope is closing: `changes` ends next.
            Err(BusError::Closed { .. }) => return,
            Err(e) => {
                self.env.log.error(Failure::BusOther, || format!("a fresh bus reader: {e}"));
                return;
            }
        };
        let replaced = std::mem::replace(&mut self.reader, fresh);
        self.dropped_before += replaced.sub.dropped();
        drop(replaced);
        let mut replay = Vec::new();
        while let Ok(msg) = self.reader.sub.try_recv() {
            replay.push(msg);
        }
        // What r2r cannot convert never reaches DDS, now or in a history.
        replay.retain(|msg| !self.failed.iter().any(|failed| std::ptr::eq(failed.as_ptr(), Arc::as_ptr(msg))));
        if self.bus_writers() == 0 {
            // Their writers have gone since the reader was made.
            self.env.counters.stale(replay.len());
            self.destroy().await;
            return;
        }
        let kept = match self.dds {
            Some(_) => keep_writer(&replay, &self.sent, self.depth),
            None => None,
        };
        let queued = match kept {
            Some(queued) => queued,
            None => {
                self.destroy().await;
                self.create().await;
                0
            }
        };
        // One r2r cannot convert is left out (`reject`), so the writer holds
        // the history without it.
        for msg in &replay[queued..] {
            self.publish(msg);
        }
    }

    /// A DDS writer while a bus writer is live, none otherwise.
    async fn sync_presence(&mut self) {
        if self.bus_writers() == 0 {
            self.destroy().await;
        } else if self.dds.is_none() {
            self.create().await;
        }
    }

    /// Publish on the DDS writer, if there is one. `false`: r2r cannot
    /// convert `msg`, which is dropped (`reject`).
    fn publish(&mut self, msg: &Arc<T>) -> bool {
        let Some(dds) = &self.dds else {
            return true;
        };
        if !self.latched {
            match dds.get_inter_process_subscription_count() {
                Ok(0) => {
                    self.env.counters.skipped();
                    return true;
                }
                Ok(_) => {}
                // Publish anyway: losing the message would be worse than the
                // work.
                Err(e) => self.env.log.error(Failure::Dds, || format!("counting the DDS readers failed: {e}")),
            }
        }
        match guarded(|| dds.publish(msg)) {
            Ok(Ok(())) => {
                self.env.counters.forwarded();
                self.env.log.ok();
                if self.latched {
                    if self.sent.len() >= self.reader_qos.depth {
                        self.sent.pop_front();
                    }
                    self.sent.push_back(Arc::downgrade(msg));
                }
            }
            Ok(Err(e)) => self.env.log.error(Failure::Dds, || format!("publishing on DDS failed: {e}")),
            Err(why) => {
                self.reject(msg, why);
                return false;
            }
        }
        true
    }

    /// A message r2r cannot convert: dropped, counted and logged.
    /// Transient local, also remembered, so that a fresh reader's history
    /// goes to DDS without it (`resync_latched`).
    fn reject(&mut self, msg: &Arc<T>, why: String) {
        self.env.counters.invalid();
        self.env.log.error(Failure::Invalid, || format!("r2r cannot convert a bus message ({why}), so it was dropped"));
        if self.latched {
            if self.failed.len() >= self.reader_qos.depth {
                self.failed.pop_front();
            }
            self.failed.push_back(Arc::downgrade(msg));
        }
    }

    async fn create(&mut self) {
        let (name, qos) = (self.name.clone(), self.dds_qos.clone());
        match self.env.node.call(move |node| node.create_publisher::<T>(&name, qos)).await {
            Some(Ok(publisher)) => {
                self.dds = Some(publisher);
                self.env.counters.set_dds_writer(true);
            }
            Some(Err(e)) => self.env.log.error(Failure::Dds, || format!("creating the DDS writer failed: {e}")),
            // The bridge is stopping.
            None => {}
        }
    }

    async fn destroy(&mut self) {
        self.sent.clear();
        if let Some(publisher) = self.dds.take() {
            self.env.counters.set_dds_writer(false);
            // Awaited, so that the next writer never lives next to this one.
            self.env.node.call(move |node| node.destroy_publisher(publisher)).await;
        }
    }
}

/// Whether the DDS writer can stay, given `replay` (what a fresh reader got:
/// the retained history in publish order, then anything published since) and
/// `sent` (what this writer was given, oldest first).
///
/// Messages are forwarded in publish order, so everything in `replay` up to
/// the last message it shares with `sent` has been given (or was lost to a
/// full queue before it could be), and the rest is queued. The writer can
/// stay when, once given the queued messages, it holds the newest `depth` of
/// `replay`, which is what a bus late joiner gets. Then `Some(i)`:
/// `replay[i..]` is what is queued. `None`: it would still hold a value the
/// bus no longer retains, or lack one the bus does, so it must be created
/// again with `replay`.
///
/// A held message that a queued one pushes out is no reason to re-create it
/// (a value superseded, not dropped): every DDS reader would see its
/// publisher leave and come back, and a volatile one would miss the queued
/// message.
fn keep_writer<T>(replay: &[Arc<T>], sent: &VecDeque<Weak<T>>, depth: usize) -> Option<usize> {
    let given = |msg: &Arc<T>| sent.iter().any(|sent| std::ptr::eq(sent.as_ptr(), Arc::as_ptr(msg)));
    let queued = replay.iter().rposition(given).map_or(0, |last| last + 1);
    let held = sent.iter().skip(sent.len().saturating_sub(depth)).map(Weak::as_ptr);
    let after: Vec<*const T> = held.chain(replay[queued..].iter().map(Arc::as_ptr)).collect();
    let newest = replay[replay.len().saturating_sub(depth)..].iter().map(Arc::as_ptr);
    after[after.len().saturating_sub(depth)..].iter().copied().eq(newest).then_some(queued)
}

/// `from_ros`: a DDS reader with the source's QoS while the bus has a reader
/// of the name, and a bus writer (README.md, "Graph presence"). Volatile,
/// the bus writer lives as long as the bridge runs. Transient local, it
/// exists only while the DDS reader does and the graph shows a DDS writer.
pub(crate) fn from_ros<T: BridgeMessage>(
    scope: &Scope, node: &mut r2r::Node, name: &str, dds_qos: QosProfile, bus_qos: Qos, env: Env,
) -> Result<Forwarder, BridgeError> {
    // The name and the type are checked now; the DDS reader comes with a bus
    // reader, once the bridge runs. The watch comes first, so that no reader
    // falls between it and the count.
    let publisher = scope.publisher::<T>(name, bus_qos)?;
    let readers = publisher.subscription_changes();
    let bus_readers = publisher.subscription_count();
    let latched = bus_qos.is_transient_local();
    let writer = if latched {
        // A transient-local bus writer comes with a DDS writer. The watch
        // lives on, in the bridge's scope.
        drop(publisher);
        None
    } else {
        env.counters.set_bus_writer(true);
        Some(Writer { publisher, own_scope: None })
    };
    let own = (node.name()?, node.namespace()?);
    let forwarder = FromRos {
        name: name.to_string(),
        scope: scope.clone(),
        dds_qos,
        bus_qos,
        latched,
        readers,
        bus_readers,
        dds: None,
        changed: None,
        writer,
        heard: false,
        own,
        env,
    };
    Ok(Box::pin(forwarder.run()))
}

/// A `from_ros` forwarder.
///
/// - **The DDS reader** exists while the bus has a reader of the name, as a
///   module's own r2r reader existed while it subscribed: no DDS writer is
///   matched with a reader nothing stands behind. The plan includes readers
///   a module makes only in some configurations (nav's `/rosout` reader, its
///   nav_core inputs). It is made and destroyed on the spin thread, at most
///   once per `DWELL`.
/// - **Transient local, the bus writer**, and with it the history a bus late
///   joiner gets, exists only while DDS has a writer, as the DDS writer's
///   history did for the module's own r2r reader: a dead module's latched
///   value (coverage's markers, nav's `/nav_operation_active`) leaves the bus
///   with its writer. And only while the DDS reader exists: a bus reader that
///   comes after the last one went gets the DDS writers' histories through
///   the next DDS reader, as a later r2r reader did, not what the bridge
///   heard before and might have missed since.
///
/// r2r has no graph event, so the graph is looked at every `PRESENCE_PERIOD`
/// on the spin thread. The graph learns of a writer after the match, and the
/// replay of its history may come first: a sample is proof of a writer, so
/// it makes the bus writer if there is none, and keeps it through a look
/// that sees no writer (the next look decides).
struct FromRos<T: BridgeMessage> {
    name: String,
    scope: Scope,
    dds_qos: QosProfile,
    bus_qos: Qos,
    latched: bool,
    /// The bus's readers of the name, and their count as `readers` last
    /// gave it.
    readers: SubscriptionChanges,
    bus_readers: usize,
    /// The DDS reader, while the bus has a reader.
    dds: Option<r2r::Subscription<T>>,
    /// When the DDS reader last came or went.
    changed: Option<tokio::time::Instant>,
    writer: Option<Writer<T>>,
    /// Transient local: a sample came since the last look.
    heard: bool,
    /// The bridge's node, `(name, namespace)`: its own endpoints never
    /// count. One direction per name keeps it from writing a name it reads,
    /// but a filter by writer (the `/tf_static` question, option 3) would
    /// not.
    own: (String, String),
    env: Env,
}

impl<T: BridgeMessage> FromRos<T> {
    async fn run(mut self) {
        let mut look = tokio::time::interval(PRESENCE_PERIOD);
        look.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        loop {
            let follow = (self.bus_readers > 0) != self.dds.is_some();
            let due = self.changed.map_or_else(tokio::time::Instant::now, |changed| changed + DWELL);
            let going = tokio::select! {
                count = self.readers.next() => match count {
                    Some(count) => {
                        self.bus_readers = count;
                        true
                    }
                    // The bridge's scope closed.
                    None => false,
                },
                () = tokio::time::sleep_until(due), if follow => self.follow().await,
                _ = look.tick(), if self.latched && self.dds.is_some() => self.look().await,
                msg = next_sample(&mut self.dds) => match msg {
                    Some(msg) => self.forward(msg),
                    // Only with the node: the bridge is stopping.
                    None => false,
                },
            };
            if !going {
                return;
            }
        }
    }

    /// The DDS reader for the bus's first reader, none after its last.
    /// `false`: the bridge is stopping.
    async fn follow(&mut self) -> bool {
        self.changed = Some(tokio::time::Instant::now());
        match self.dds.take() {
            None => {
                let (name, qos) = (self.name.clone(), self.dds_qos.clone());
                match self.env.node.call(move |node| node.subscribe::<T>(&name, qos)).await {
                    Some(Ok(dds)) => {
                        self.dds = Some(dds);
                        self.heard = false;
                        self.env.counters.set_dds_reader(true);
                    }
                    Some(Err(e)) => self.env.log.error(Failure::Dds, || format!("creating the DDS reader failed: {e}")),
                    None => return false,
                }
            }
            Some(dds) => {
                self.env.counters.set_dds_reader(false);
                // Transient local, the bus writer goes with it, and its
                // history too.
                if self.latched && self.writer.take().is_some() {
                    self.env.counters.set_bus_writer(false);
                }
                // What it has queued has no bus reader to go to. Awaited, so
                // that the next DDS reader never lives next to this one.
                if self.env.node.call(move |node| node.destroy_subscription(dds)).await.is_none() {
                    return false;
                }
            }
        }
        true
    }

    /// One look at the graph's writers. `false`: the bridge is stopping, or
    /// its scope closed.
    async fn look(&mut self) -> bool {
        let (name, own) = (self.name.clone(), self.own.clone());
        let shown = self
            .env
            .node
            .call(move |node| {
                let writers = node.get_publishers_info_by_topic(&name, false)?;
                Ok::<_, r2r::Error>(writers.iter().filter(|w| (&w.node_name, &w.node_namespace) != (&own.0, &own.1)).count())
            })
            .await;
        self.env.counters.resynced();
        match shown {
            // The bridge is stopping.
            None => false,
            Some(Ok(0)) => {
                // What r2r has queued was published before the writer went:
                // live bus readers get it, as the module's own reader did.
                while let Some(Some(msg)) = self.dds.as_mut().and_then(|dds| dds.next().now_or_never()) {
                    if !self.forward(msg) {
                        return false;
                    }
                }
                if !std::mem::take(&mut self.heard) && self.writer.take().is_some() {
                    // Its history goes with it.
                    self.env.counters.set_bus_writer(false);
                }
                true
            }
            Some(Ok(_)) => {
                self.heard = false;
                self.writer.is_some() || self.create()
            }
            Some(Err(e)) => {
                self.env.log.error(Failure::Dds, || format!("looking at the graph failed: {e}"));
                true
            }
        }
    }

    /// `false`: the bridge's scope closed.
    fn forward(&mut self, msg: T) -> bool {
        self.heard = true;
        if self.writer.is_none() && !self.create() {
            return false;
        }
        let Some(writer) = &self.writer else {
            return true;
        };
        // Only `Closed`: the bridge's scope closed, and `run` returns.
        if writer.publisher.publish_owned(msg).is_err() {
            return false;
        }
        self.env.counters.forwarded();
        true
    }

    /// A transient-local bus writer; `false`: the bridge's scope closed.
    fn create(&mut self) -> bool {
        match Writer::latched(&self.scope, &self.name, self.bus_qos) {
            Ok(writer) => {
                self.writer = Some(writer);
                self.env.counters.set_bus_writer(true);
                true
            }
            Err(BusError::Closed { .. }) => false,
            Err(e) => {
                self.env.log.error(Failure::BusOther, || format!("a bus writer: {e}"));
                true
            }
        }
    }
}

/// The next sample of the DDS reader; while there is none, nothing.
async fn next_sample<T: BridgeMessage>(dds: &mut Option<r2r::Subscription<T>>) -> Option<T> {
    match dds {
        Some(dds) => dds.next().await,
        None => std::future::pending().await,
    }
}

/// The bus writer a `from_ros` forwarder publishes on.
struct Writer<T: BridgeMessage> {
    publisher: Publisher<T>,
    /// Transient local: the child scope this writer alone lives in, closed
    /// when the writer goes, as `Reader` does for `to_ros`: the bridge's
    /// scope keeps no close hook of it, however often DDS writers come and
    /// go. Volatile: none, the writer lives in the bridge's scope all along.
    own_scope: Option<Scope>,
}

impl<T: BridgeMessage> Writer<T> {
    fn latched(scope: &Scope, name: &str, qos: Qos) -> Result<Self, BusError> {
        let own_scope = scope.child(scope.name());
        match own_scope.publisher::<T>(name, qos) {
            Ok(publisher) => Ok(Writer { publisher, own_scope: Some(own_scope) }),
            Err(e) => {
                own_scope.close();
                Err(e)
            }
        }
    }
}

impl<T: BridgeMessage> Drop for Writer<T> {
    fn drop(&mut self) {
        if let Some(scope) = &self.own_scope {
            scope.close();
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::time::{Duration, Instant};

    use futures::Stream;
    use r2r::std_msgs::msg::String as Text;

    use super::*;
    use crate::node;

    fn sent(msgs: &[&Arc<u32>]) -> VecDeque<Weak<u32>> {
        msgs.iter().map(|msg| Arc::downgrade(msg)).collect()
    }

    fn text(data: &str) -> Text {
        Text { data: data.into() }
    }

    /// How many writers of `name` the graph shows, waited for until `n`.
    async fn shown(handle: &NodeHandle, name: &'static str, n: usize) {
        let deadline = Instant::now() + Duration::from_secs(20);
        loop {
            let writers = handle.call(move |node| node.get_publishers_info_by_topic(name, false).unwrap().len()).await;
            if writers == Some(n) {
                return;
            }
            assert!(Instant::now() < deadline, "timed out waiting for {n} writers of {name}");
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    }

    /// How many readers of `name` the graph shows, waited for until `n`.
    async fn reading(handle: &NodeHandle, name: &'static str, n: usize) {
        let deadline = Instant::now() + Duration::from_secs(20);
        loop {
            let readers = handle.call(move |node| node.get_subscriptions_info_by_topic(name, false).unwrap().len()).await;
            if readers == Some(n) {
                return;
            }
            assert!(Instant::now() < deadline, "timed out waiting for {n} readers of {name}");
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    }

    /// A transient-local `from_ros` on a node that the test spins itself
    /// (`node::serve`), so that what r2r has taken from DDS when the
    /// forwarder looks at the graph is the test's to say.
    #[test]
    fn a_latched_from_ros_hands_on_what_is_queued_as_its_writer_goes_and_follows_its_bus_readers() {
        let runtime = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
        runtime.block_on(async {
            let name = "/bridge_unit/from_ros_queued";
            let dds_qos = QosProfile::default().keep_last(2).reliable().transient_local();
            let node = r2r::Node::create(crate::tests::context(), "from_ros_unit_bridge", "").unwrap();
            let own = (node.name().unwrap(), node.namespace().unwrap());
            let (handle, commands) = node::queue();
            let running = Arc::new(AtomicBool::new(true));
            let serving = {
                let running = running.clone();
                std::thread::spawn(move || node::serve(node, commands, running))
            };
            let bus = mower_bus::Bus::new();
            let scope = bus.scope("bridge");
            let bus_qos = Qos::latched().keep_last(2);
            let readers = scope.publisher::<Text>(name, bus_qos).unwrap().subscription_changes();
            let adapter = bus.scope("flutter_adapter");
            let mut live = adapter.subscribe::<Text>(name, Qos::latched().keep_last(10)).unwrap();
            let env = Env { node: handle.clone(), counters: Arc::default(), log: Log::new("test", format!("from_ros {name}")) };
            let mut from_ros = FromRos::<Text> {
                name: name.into(),
                scope,
                dds_qos: dds_qos.clone(),
                bus_qos,
                latched: true,
                readers,
                bus_readers: 1,
                dds: None,
                changed: None,
                writer: None,
                heard: false,
                own,
                env,
            };
            // A bus reader: the DDS reader comes.
            assert!(from_ros.follow().await);
            assert!(from_ros.dds.is_some());
            reading(&handle, name, 1).await;

            // A writer of another node publishes twice and goes; r2r has
            // taken both when the graph shows it gone. The live reader gets
            // them, and the bus writer they made stays until a look has
            // heard nothing.
            let mut coverage = r2r::Node::create(crate::tests::context(), "from_ros_unit_coverage", "").unwrap();
            let writer = coverage.create_publisher::<Text>(name, dds_qos.clone()).unwrap();
            writer.publish(&text("a")).unwrap();
            writer.publish(&text("b")).unwrap();
            let deadline = Instant::now() + Duration::from_secs(20);
            while from_ros.dds.as_ref().unwrap().size_hint().0 < 2 {
                handle.call(|node| node.spin_once(Duration::from_millis(10))).await.unwrap();
                assert!(Instant::now() < deadline, "r2r never took both samples");
            }
            coverage.destroy_publisher(writer);
            shown(&handle, name, 0).await;
            assert!(from_ros.look().await);
            assert_eq!(live.try_recv().unwrap().data, "a");
            assert_eq!(live.try_recv().unwrap().data, "b");
            assert!(from_ros.writer.is_some(), "a sample is proof of a writer");
            assert!(from_ros.look().await);
            assert!(from_ros.writer.is_none(), "kept with no writer and nothing heard");
            let mut late = adapter.subscribe::<Text>(name, Qos::latched().keep_last(2)).unwrap();
            assert!(late.try_recv().is_err(), "its history outlived it");

            // The bridge's own writer does not count; another node's does.
            let _mine = handle.call(move |node| node.create_publisher::<Text>(name, QosProfile::default()).unwrap()).await.unwrap();
            shown(&handle, name, 1).await;
            assert!(from_ros.look().await);
            assert!(from_ros.writer.is_none(), "the bridge's own writer counted");
            let _theirs = coverage.create_publisher::<Text>(name, QosProfile::default()).unwrap();
            shown(&handle, name, 2).await;
            assert!(from_ros.look().await);
            assert!(from_ros.writer.is_some(), "another node's writer did not count");

            // No bus reader any more: the DDS reader goes, and the bus writer
            // with its history.
            from_ros.bus_readers = 0;
            assert!(from_ros.follow().await);
            assert!(from_ros.dds.is_none() && from_ros.writer.is_none());
            reading(&handle, name, 0).await;

            running.store(false, Ordering::Release);
            serving.join().unwrap();
        });
    }

    #[test]
    fn the_dds_writer_stays_unless_it_would_hold_what_the_bus_dropped_or_lack_what_it_retains() {
        let [a, b, c, a_again] = [1u32, 2, 3, 1].map(Arc::new);
        let one = |m: &Arc<u32>| vec![m.clone()];
        // one writer, depth 1: unchanged, or something new queued after it
        assert_eq!(keep_writer(&one(&a), &sent(&[&a]), 1), Some(1));
        assert_eq!(keep_writer(&[a.clone(), b.clone()], &sent(&[&a]), 1), Some(1));
        // the writer holds a, and b, which supersedes it, is still queued (an
        // unrelated writer change came first): it stays, and gets b
        assert_eq!(keep_writer(&one(&b), &sent(&[&a]), 1), Some(0));
        // a restarted module's first value supersedes the old one the same way
        assert_eq!(keep_writer(&one(&a_again), &sent(&[&a]), 1), Some(0));
        // its writer is gone and nothing supersedes a: it must not stay
        assert_eq!(keep_writer(&[], &sent(&[&a]), 1), None);
        // two writers, depth 1: the DDS writer holds the newer, b
        assert_eq!(keep_writer(&[a.clone(), b.clone()], &sent(&[&a, &b]), 1), Some(2));
        // b's writer goes: a is retained, b is not, nothing queued
        assert_eq!(keep_writer(&one(&a), &sent(&[&a, &b]), 1), None);
        // a given so long ago that `sent` has let it go: it comes before b,
        // which was given, so it counts as given too
        assert_eq!(keep_writer(&[a.clone(), b.clone()], &sent(&[&b]), 1), Some(2));
        // depth 2: what the queued message pushes out is fine
        assert_eq!(keep_writer(&[b.clone(), c.clone()], &sent(&[&a, &b]), 2), Some(1));
        // b's writer goes: the writer holds b and lacks a among the newest two
        assert_eq!(keep_writer(&[a.clone(), c.clone()], &sent(&[&a, &b, &c]), 2), None);
        // depth 2, the writer was given c alone, but the bus retains b and c
        assert_eq!(keep_writer(&[b.clone(), c.clone()], &sent(&[&c]), 2), None);
        // nothing given yet: it stays and gets everything
        assert_eq!(keep_writer(&[a.clone(), b.clone()], &VecDeque::new(), 1), Some(0));
        assert_eq!(keep_writer(&[], &VecDeque::<Weak<u32>>::new(), 1), Some(0));
        // one Arc published twice: what it holds cannot be told, so it goes
        assert_eq!(keep_writer(&[a.clone(), a.clone()], &sent(&[&a]), 2), None);
        // a dropped message matches nothing new: the Weak keeps its
        // allocation, so its address is not reused
        let gone = sent(&[&Arc::new(4u32)]);
        assert_eq!(keep_writer(&[], &gone, 1), None);
    }
}
