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

use std::collections::VecDeque;
use std::sync::{Arc, Weak};

use futures::StreamExt;
use mower_bus::{BusError, PublisherChanges, Qos, Scope, Subscriber};
use r2r::QosProfile;

use crate::log::{Failure, Log};
use crate::node::NodeHandle;
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
            self.create().await;
        }
        self.publish(&msg);
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

    fn publish(&mut self, msg: &Arc<T>) {
        let Some(dds) = &self.dds else {
            return;
        };
        if !self.latched {
            match dds.get_inter_process_subscription_count() {
                Ok(0) => {
                    self.env.counters.skipped();
                    return;
                }
                Ok(_) => {}
                // Publish anyway: losing the message would be worse than the
                // work.
                Err(e) => self.env.log.error(Failure::Dds, || format!("counting the DDS readers failed: {e}")),
            }
        }
        match dds.publish(msg) {
            Ok(()) => {
                self.env.counters.forwarded();
                self.env.log.ok();
                if self.latched {
                    if self.sent.len() >= self.reader_qos.depth {
                        self.sent.pop_front();
                    }
                    self.sent.push_back(Arc::downgrade(msg));
                }
            }
            Err(e) => self.env.log.error(Failure::Dds, || format!("publishing on DDS failed: {e}")),
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

/// `from_ros`: a DDS reader with the source's QoS, and a bus writer that
/// lives as long as the bridge runs (README.md, "Graph presence").
pub(crate) fn from_ros<T: BridgeMessage>(
    scope: &Scope, node: &mut r2r::Node, name: &str, dds_qos: QosProfile, bus_qos: Qos, env: Env,
) -> Result<Forwarder, BridgeError> {
    let publisher = scope.publisher::<T>(name, bus_qos)?;
    let mut samples = node.subscribe::<T>(name, dds_qos)?;
    Ok(Box::pin(async move {
        while let Some(msg) = samples.next().await {
            // Only `Closed`: the bridge's scope closed, and `run` returns.
            if publisher.publish_owned(msg).is_err() {
                return;
            }
            env.counters.forwarded();
        }
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sent(msgs: &[&Arc<u32>]) -> VecDeque<Weak<u32>> {
        msgs.iter().map(|msg| Arc::downgrade(msg)).collect()
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
