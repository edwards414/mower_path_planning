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
//!   If the DDS writer holds something that history no longer has, it is
//!   destroyed and created again with the history, so a DDS late joiner
//!   never gets a value the bus has dropped. With no live bus writer,
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

use futures::{Stream, StreamExt};
use mower_bus::{Qos, Scope, Subscriber};
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
        // ends when that scope closes. That reader goes at once (volatile and
        // of depth 1, it holds at most one message meanwhile); the readers
        // that carry messages each live in a scope of their own (`Reader`).
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
        changes: Box::new(changes),
        reader,
        dds: None,
        held: VecDeque::new(),
        env,
    };
    Ok(Box::pin(forwarder.run()))
}

/// The bus reader a `to_ros` forwarder takes messages from.
struct Reader<T: BridgeMessage> {
    sub: Subscriber<T>,
    /// Transient local: the scope this reader alone lives in, closed when the
    /// reader is replaced. A reader of the bridge's long-lived scope would
    /// leave its close hook there at every resync (mower_bus README, "Hooks
    /// of dropped endpoints").
    own_scope: Option<Scope>,
}

impl<T: BridgeMessage> Reader<T> {
    fn latched(scope: &Scope, name: &str, qos: Qos) -> Result<Self, BridgeError> {
        let own = scope.bus().scope(scope.name());
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
    changes: Box<dyn Stream<Item = usize> + Send + Unpin>,
    reader: Reader<T>,
    dds: Option<r2r::Publisher<T>>,
    /// Transient local: what the DDS writer holds, oldest first, at most
    /// `depth` messages. Identity, not value, tells whether the bus still
    /// retains them: the bus hands every reader the publisher's own `Arc`.
    /// `Weak`, so that a map the bus has dropped is not kept alive here.
    held: VecDeque<Weak<T>>,
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
            // Only `Closed`: the bridge's scope closed, and `changes` ends next.
            Err(e) => {
                self.env.log.error(Failure::BusOther, || format!("a fresh bus reader: {e}"));
                return;
            }
        };
        drop(std::mem::replace(&mut self.reader, fresh));
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
        let agreed = match self.dds {
            Some(_) => agreed_end(&replay, &self.held, self.depth),
            None => None,
        };
        let skip = match agreed {
            Some(end) => end,
            None => {
                self.destroy().await;
                self.create().await;
                0
            }
        };
        for msg in &replay[skip..] {
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
                    if self.held.len() >= self.depth {
                        self.held.pop_front();
                    }
                    self.held.push_back(Arc::downgrade(msg));
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
        self.held.clear();
        if let Some(publisher) = self.dds.take() {
            self.env.counters.set_dds_writer(false);
            // Awaited, so that the next writer never lives next to this one.
            self.env.node.call(move |node| node.destroy_publisher(publisher)).await;
        }
    }
}

/// Whether the DDS writer still agrees with the bus, given `replay` (what a
/// fresh reader got: the retained history, oldest first, then anything
/// published since) and `held` (what the DDS writer keeps of what it was
/// given, at most `depth`). It agrees when `held` is a run of `replay` and
/// is all of the newest `depth` messages up to the run's end; then the
/// returned end is where the messages it has not been given begin. `None`:
/// it holds a value the bus has dropped, or lacks one the bus retains.
///
/// The last matching run is taken: only a module that publishes one `Arc`
/// twice makes two, and for the same `Arc` either gives DDS the same values.
fn agreed_end<T>(replay: &[Arc<T>], held: &VecDeque<Weak<T>>, depth: usize) -> Option<usize> {
    if held.is_empty() {
        return Some(0);
    }
    let same = |held: &Weak<T>, msg: &Arc<T>| std::ptr::eq(held.as_ptr(), Arc::as_ptr(msg));
    let last_start = replay.len().checked_sub(held.len())?;
    let start = (0..=last_start)
        .rev()
        .find(|&start| held.iter().zip(&replay[start..]).all(|(held, msg)| same(held, msg)))?;
    let end = start + held.len();
    (start == end.saturating_sub(depth)).then_some(end)
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

    fn held(msgs: &[&Arc<u32>]) -> VecDeque<Weak<u32>> {
        msgs.iter().map(|msg| Arc::downgrade(msg)).collect()
    }

    #[test]
    fn identity_not_value_decides_what_the_dds_writer_still_agrees_with() {
        let (a, b, c, a_again) = (Arc::new(1u32), Arc::new(2u32), Arc::new(3u32), Arc::new(1u32));
        // one writer, depth 1: unchanged, or something new after it
        assert_eq!(agreed_end(std::slice::from_ref(&a), &held(&[&a]), 1), Some(1));
        assert_eq!(agreed_end(&[a.clone(), b.clone()], &held(&[&a]), 1), Some(1));
        // an equal value from another publish is another message
        assert_eq!(agreed_end(std::slice::from_ref(&a_again), &held(&[&a]), 1), None);
        // its writer is gone: nothing, or another writer's older value, retained
        assert_eq!(agreed_end(&[], &held(&[&a]), 1), None);
        assert_eq!(agreed_end(std::slice::from_ref(&c), &held(&[&a]), 1), None);
        // two writers, depth 1: the DDS writer keeps the newer, which is the
        // run's end; a writer that comes changes nothing
        assert_eq!(agreed_end(&[a.clone(), b.clone()], &held(&[&b]), 1), Some(2));
        // b retained but not given yet (it was still queued): given next
        assert_eq!(agreed_end(&[a.clone(), b.clone()], &held(&[&a]), 1), Some(1));
        // depth 2: all of the newest two, in order
        assert_eq!(agreed_end(&[a.clone(), b.clone()], &held(&[&a, &b]), 2), Some(2));
        assert_eq!(agreed_end(&[a.clone(), b.clone(), c.clone()], &held(&[&b, &c]), 2), Some(3));
        assert_eq!(agreed_end(&[a.clone(), b.clone(), c.clone()], &held(&[&c]), 2), None, "b is among the newest two");
        assert_eq!(agreed_end(&[b.clone(), a.clone()], &held(&[&a, &b]), 2), None, "reordered");
        // a DDS writer given nothing yet agrees with anything
        assert_eq!(agreed_end(&[a.clone(), b.clone()], &VecDeque::new(), 1), Some(0));
        // a dropped message matches nothing new: the Weak keeps its
        // allocation, so its address is not reused
        let gone = held(&[&Arc::new(4u32)]);
        assert_eq!(agreed_end(&[Arc::new(4u32)], &gone, 1), None);
    }
}
