//! What each forwarder has done, readable while the bridge runs: for
//! diagnostics (E8's `mowerctl`) and for the tests, which cannot see a
//! skipped publish from DDS.

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

use crate::{Direction, Kind};

/// One forwarder's counters, updated without a lock.
#[derive(Debug, Default)]
pub(crate) struct Counters {
    forwarded: AtomicU64,
    skipped: AtomicU64,
    stale: AtomicU64,
    resyncs: AtomicU64,
    unanswered: AtomicU64,
    dropped: AtomicU64,
    dds_writer: AtomicBool,
    bus_server: AtomicBool,
}

impl Counters {
    pub(crate) fn forwarded(&self) {
        self.forwarded.fetch_add(1, Ordering::Relaxed);
    }

    pub(crate) fn skipped(&self) {
        self.skipped.fetch_add(1, Ordering::Relaxed);
    }

    pub(crate) fn stale(&self, n: usize) {
        self.stale.fetch_add(n as u64, Ordering::Relaxed);
    }

    pub(crate) fn resynced(&self) {
        self.resyncs.fetch_add(1, Ordering::Relaxed);
    }

    pub(crate) fn unanswered(&self) {
        self.unanswered.fetch_add(1, Ordering::Relaxed);
    }

    pub(crate) fn set_dropped(&self, n: u64) {
        self.dropped.store(n, Ordering::Relaxed);
    }

    pub(crate) fn set_dds_writer(&self, exists: bool) {
        self.dds_writer.store(exists, Ordering::Relaxed);
    }

    pub(crate) fn set_bus_server(&self, exists: bool) {
        self.bus_server.store(exists, Ordering::Relaxed);
    }
}

/// A snapshot of one bridged name.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EntryStats {
    pub direction: Direction,
    /// Messages published on the other side; for services, calls answered;
    /// for actions, goal requests answered (accepted or rejected).
    pub forwarded: u64,
    /// `to_ros`, volatile: bus messages not published because no DDS reader
    /// was matched.
    pub skipped: u64,
    /// `to_ros`: bus messages not published because no bus writer was live
    /// any more when they came up (README.md, "Writer lifetime").
    pub stale: u64,
    /// `to_ros`: writer changes handled (`publisher_changes` items).
    /// `service_from_ros`, `action_from_ros`: looks at the graph for a DDS
    /// server.
    pub resyncs: u64,
    /// Services: calls the bridge left unanswered (README.md, "Services").
    /// Actions: goal requests left unanswered, and accepted goals whose
    /// result never reached the other side (README.md, "Actions").
    pub unanswered: u64,
    /// `to_ros`: bus messages the bridge's reader lost to its full queue,
    /// because the forwarder fell more than its depth (at least 64) behind.
    /// Counted when the forwarder next handles a message or a change.
    pub dropped: u64,
    /// `to_ros`: whether the bridge's DDS writer exists now.
    pub dds_writer: bool,
    /// `service_from_ros`, `action_from_ros`: whether the bridge's bus server
    /// exists now, which it does while the graph shows a DDS server.
    pub bus_server: bool,
}

/// Every forwarder's counters, by kind and name.
type Entries = BTreeMap<(Kind, String), (Direction, Arc<Counters>)>;

/// The counters of every name a bridge forwards. Cheap to clone.
#[derive(Clone, Debug, Default)]
pub struct BridgeStats {
    entries: Arc<Mutex<Entries>>,
}

impl BridgeStats {
    pub(crate) fn register(&self, name: &str, direction: Direction) -> Arc<Counters> {
        let counters = Arc::new(Counters::default());
        self.lock().insert((direction.kind(), name.to_string()), (direction, counters.clone()));
        counters
    }

    /// A declaration that failed after `register`.
    pub(crate) fn unregister(&self, name: &str, direction: Direction) {
        self.lock().remove(&(direction.kind(), name.to_string()));
    }

    /// The forwarder of topic `name`, if the bridge has one.
    pub fn topic(&self, name: &str) -> Option<EntryStats> {
        self.get(Kind::Topic, name)
    }

    /// The forwarder of service `name`, if the bridge has one.
    pub fn service(&self, name: &str) -> Option<EntryStats> {
        self.get(Kind::Service, name)
    }

    /// The forwarder of action `name`, if the bridge has one.
    pub fn action(&self, name: &str) -> Option<EntryStats> {
        self.get(Kind::Action, name)
    }

    fn get(&self, kind: Kind, name: &str) -> Option<EntryStats> {
        let entries = self.lock();
        let (direction, c) = entries.get(&(kind, name.to_string()))?;
        let load = |n: &AtomicU64| n.load(Ordering::Relaxed);
        Some(EntryStats {
            direction: *direction,
            forwarded: load(&c.forwarded),
            skipped: load(&c.skipped),
            stale: load(&c.stale),
            resyncs: load(&c.resyncs),
            unanswered: load(&c.unanswered),
            dropped: load(&c.dropped),
            dds_writer: c.dds_writer.load(Ordering::Relaxed),
            bus_server: c.bus_server.load(Ordering::Relaxed),
        })
    }

    fn lock(&self) -> MutexGuard<'_, Entries> {
        self.entries.lock().unwrap_or_else(PoisonError::into_inner)
    }
}
