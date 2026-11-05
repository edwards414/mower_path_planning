//! What a process knows about itself that no bus name carries (E8-2): its
//! supervisor's slots and its bus bridge's counters. `mower_rsd` and the
//! bridge module write here; ws_bridge's loopback listener reads it to answer
//! `/mowerd/modules` and `/mowerd/bridge` (mowerctl's `module list` and
//! `bridge`).
//!
//! It touches neither DDS nor the bus, and it is written only when an
//! instance starts or ends, so it costs nothing while the robot runs. The
//! bridge's counters are read through a probe, when somebody asks.

use std::fmt;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::Instant;

use serde_json::Value;

/// Where one supervised instance is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SlotState {
    /// Known to the supervisor, its first start not reached yet.
    Starting,
    Running,
    /// Failed, and to be started again after the restart delay.
    Restarting,
    /// Asked to stop, and stopped.
    Stopped,
    /// Failed during a shutdown: not started again.
    Failed,
}

impl SlotState {
    pub fn as_str(self) -> &'static str {
        match self {
            SlotState::Starting => "starting",
            SlotState::Running => "running",
            SlotState::Restarting => "restarting",
            SlotState::Stopped => "stopped",
            SlotState::Failed => "failed",
        }
    }
}

/// One supervised instance, as the supervisor last reported it.
#[derive(Clone, Debug, PartialEq)]
pub struct SlotInfo {
    /// `<module id>/<node>`, the label of mower_rsd's log lines.
    pub label: String,
    pub node: String,
    pub state: SlotState,
    /// How many times it was started, the first start included.
    pub starts: u64,
    pub failures: u64,
    /// When its latest start began.
    pub started_at: Option<Instant>,
    /// Why it last failed, and when.
    pub last_failure: Option<(String, Instant)>,
}

/// Reads the bus bridge's counters: a JSON array, one object per bridged name
/// (mower_bridge's run.rs).
pub type BridgeProbe = Arc<dyn Fn() -> Value + Send + Sync>;

#[derive(Default)]
struct Inner {
    slots: Vec<SlotInfo>,
    bridge: Option<BridgeProbe>,
}

/// The process's registry. Cheap to clone; a [`crate::ModuleCtx`] on its own
/// (a standalone binary, a test) has one of its own that nobody writes.
#[derive(Clone, Default)]
pub struct Registry {
    inner: Arc<Mutex<Inner>>,
}

impl fmt::Debug for Registry {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let inner = self.lock();
        f.debug_struct("Registry").field("slots", &inner.slots.len()).field("bridge", &inner.bridge.is_some()).finish()
    }
}

impl Registry {
    /// Nothing under the lock panics half-way, so a poisoned lock still holds
    /// a consistent registry.
    fn lock(&self) -> MutexGuard<'_, Inner> {
        self.inner.lock().unwrap_or_else(PoisonError::into_inner)
    }

    fn with_slot(&self, label: &str, f: impl FnOnce(&mut SlotInfo)) {
        if let Some(slot) = self.lock().slots.iter_mut().find(|s| s.label == label) {
            f(slot);
        }
    }

    /// A slot the supervisor is about to run, in start order.
    pub fn add_slot(&self, label: &str, node: &str) {
        self.lock().slots.push(SlotInfo {
            label: label.to_string(),
            node: node.to_string(),
            state: SlotState::Starting,
            starts: 0,
            failures: 0,
            started_at: None,
            last_failure: None,
        });
    }

    /// The slot's instance starts now, the first time or again.
    pub fn slot_started(&self, label: &str) {
        self.with_slot(label, |s| {
            s.state = SlotState::Running;
            s.starts += 1;
            s.started_at = Some(Instant::now());
        });
    }

    /// The slot's instance failed; `restarting` when it will be started again.
    pub fn slot_failed(&self, label: &str, why: &str, restarting: bool) {
        self.with_slot(label, |s| {
            s.state = if restarting { SlotState::Restarting } else { SlotState::Failed };
            s.failures += 1;
            s.last_failure = Some((why.to_string(), Instant::now()));
        });
    }

    /// The slot's instance was asked to stop and did.
    pub fn slot_stopped(&self, label: &str) {
        self.with_slot(label, |s| s.state = SlotState::Stopped);
    }

    /// Every slot, in start order.
    pub fn slots(&self) -> Vec<SlotInfo> {
        self.lock().slots.clone()
    }

    /// The running bridge's counters are read through `probe` until the guard
    /// goes, which is when that bridge ends.
    pub fn set_bridge_probe(&self, probe: BridgeProbe) -> BridgeProbeGuard {
        self.lock().bridge = Some(probe);
        BridgeProbeGuard { registry: self.clone() }
    }

    /// The running bridge's counters; `None` while no bridge runs.
    pub fn bridge(&self) -> Option<Value> {
        // not under the lock: the probe takes the bridge's own
        let probe = self.lock().bridge.clone()?;
        Some(probe())
    }
}

/// See [`Registry::set_bridge_probe`].
pub struct BridgeProbeGuard {
    registry: Registry,
}

impl Drop for BridgeProbeGuard {
    fn drop(&mut self) {
        self.registry.lock().bridge = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn a_slot_is_followed_through_its_starts_and_failures() {
        let r = Registry::default();
        r.add_slot("guards/velocity_command_guard", "velocity_command_guard");
        r.add_slot("imu/imu", "imu");
        let imu = |r: &Registry| r.slots()[1].clone();
        assert_eq!(imu(&r).state, SlotState::Starting);
        assert_eq!((imu(&r).starts, imu(&r).failures, imu(&r).started_at), (0, 0, None));

        r.slot_started("imu/imu");
        assert_eq!((imu(&r).state, imu(&r).starts), (SlotState::Running, 1));
        assert!(imu(&r).started_at.is_some());

        r.slot_failed("imu/imu", "imu/imu: no such device", true);
        assert_eq!((imu(&r).state, imu(&r).failures), (SlotState::Restarting, 1));
        assert_eq!(imu(&r).last_failure.unwrap().0, "imu/imu: no such device");

        r.slot_started("imu/imu");
        assert_eq!((imu(&r).state, imu(&r).starts), (SlotState::Running, 2));
        assert!(imu(&r).last_failure.is_some(), "the last failure stays after the restart");

        r.slot_failed("imu/imu", "again", false);
        assert_eq!(imu(&r).state, SlotState::Failed);
        r.slot_stopped("imu/imu");
        assert_eq!(imu(&r).state, SlotState::Stopped);

        // the other slot saw none of it, and the order is the start order
        assert_eq!(r.slots()[0].label, "guards/velocity_command_guard");
        assert_eq!((r.slots()[0].starts, r.slots()[0].state), (0, SlotState::Starting));
        // a label nobody added is ignored
        r.slot_started("nobody");
        assert_eq!(r.slots().len(), 2);
    }

    #[test]
    fn the_bridge_is_read_through_its_probe_while_it_runs() {
        let r = Registry::default();
        assert_eq!(r.bridge(), None);
        let guard = r.set_bridge_probe(Arc::new(|| json!([{"name": "/odom"}])));
        assert_eq!(r.bridge(), Some(json!([{"name": "/odom"}])));
        let clone = r.clone();
        assert!(clone.bridge().is_some(), "clones share it");
        drop(guard);
        assert_eq!(r.bridge(), None);
    }
}
