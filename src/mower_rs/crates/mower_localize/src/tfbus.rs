//! The transforms the three localization nodes exchange, without `/tf`.
//!
//! The ported code makes exactly four lookups on this robot:
//!
//! | who | lookup | where the edge comes from |
//! |---|---|---|
//! | both EKFs | `base_footprint <- imu_link` | `/tf_static` (robot_state_publisher) |
//! | navsat | `base_footprint <- gps_link` | `/tf_static` |
//! | map EKF | `base_footprint <- odom` | the odom EKF's own broadcast |
//! | navsat | `map <- base_footprint` | map EKF's `map -> odom` ∘ odom EKF's `odom -> base_footprint` |
//!
//! The two dynamic edges are produced by this very module, so they are shared
//! in process rather than round-tripped through `/tf`: that topic also carries
//! robot_state_publisher's wheel joints at ~77 Hz, and every ROS event on the
//! RK3568 was measured at about 1 ms of CPU (docs/ROS_FREE_PLAN.md section 1),
//! which is most of the budget this phase is trying to win back. Each node
//! still *publishes* its transform on `/tf` for nav2 and everyone else.
//!
//! The cache is process-global because `mower_rsd` supervises the three nodes
//! as separate tasks that can restart independently; a restarted map EKF finds
//! the odom EKF's latest transform already there, exactly as it would have
//! found it in a `tf2_ros::Buffer`.
//!
//! Consequence, and the one behavioural difference from the C++ nodes: a
//! transform published by *another* node under these names would be invisible
//! here. Nothing on this robot publishes `odom -> base_footprint` or
//! `map -> odom` except these two filters (ros2_control's diff-drive
//! controller has `enable_odom_tf: false`, see mower_controller's
//! controllers.yaml).
//!
//! Other modules of the same process read it too: mower_nav's in-process
//! navigation takes the robot pose from [`lookup`] (`map <- odom`,
//! `odom <- base_footprint`) instead of subscribing to `/tf`, and how fresh
//! each edge is from [`last_set`].

use std::collections::BTreeMap;
use std::sync::{Mutex, OnceLock};
use std::time::Instant;

use mower_localize_core::prepare::TransformTree;
use mower_localize_core::tf::Transform;

use crate::tfgraph::TfGraph;

#[derive(Default)]
struct Bus {
    graph: TfGraph,
    /// When each edge was last set, keyed like the graph's edges.
    set_at: BTreeMap<(String, String), Instant>,
}

impl Bus {
    fn set(&mut self, parent: &str, child: &str, transform: Transform) {
        self.graph.set(parent, child, transform);
        self.set_at.insert((parent.to_string(), child.to_string()), Instant::now());
    }

    fn last_set(&self, parent: &str, child: &str) -> Option<Instant> {
        self.set_at.get(&(parent.to_string(), child.to_string())).copied()
    }
}

static BUS: OnceLock<Mutex<Bus>> = OnceLock::new();

fn bus() -> &'static Mutex<Bus> {
    BUS.get_or_init(|| Mutex::new(Bus::default()))
}

/// Record `parent <- child`, from `/tf_static` or from our own broadcast.
pub fn set(parent: &str, child: &str, transform: Transform) {
    if let Ok(mut b) = bus().lock() {
        b.set(parent, child, transform);
    }
}

/// `(revision, tree)` with `base <- every known frame` plus `extra`, or `None`
/// when nothing has changed since `since`.
pub fn tree_if_changed(
    since: Option<u64>, base: &str, extra: &[(&str, &str)],
) -> Option<(u64, TransformTree)> {
    let b = bus().lock().ok()?;
    let rev = b.graph.revision();
    if since == Some(rev) {
        return None;
    }
    Some((rev, b.graph.tree_for(base, extra)))
}

/// `lookupTransform(target, source, Time(0))` over the cached edges: the pose
/// of `source` in `target` from the latest of each edge on the way, `None`
/// when the two frames are not connected (see [`TfGraph::lookup`]).
pub fn lookup(target: &str, source: &str) -> Option<Transform> {
    bus().lock().ok()?.graph.lookup(target, source)
}

/// When the edge `parent <- child` was last [`set`], on this process's
/// monotonic clock; `None` if it never was.
pub fn last_set(parent: &str, child: &str) -> Option<Instant> {
    bus().lock().ok()?.last_set(parent, child)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mower_localize_core::tf::{Quaternion, Vector3};

    #[test]
    fn a_broadcast_edge_is_visible_to_the_other_nodes_and_refreshes_the_tree() {
        let t = Transform::from_parts(&Quaternion::identity(), Vector3::new(3.0, 0.0, 0.0));
        set("odom", "base_footprint", t);
        let (rev, tree) = tree_if_changed(None, "base_footprint", &[]).unwrap();
        assert!(tree.lookup("base_footprint", "odom").is_some());
        // Nothing moved -> no rebuild.
        assert!(tree_if_changed(Some(rev), "base_footprint", &[]).is_none());
        set("map", "odom", Transform::identity());
        assert!(tree_if_changed(Some(rev), "base_footprint", &[]).is_some());
        assert!(lookup("map", "base_footprint").is_some());
    }

    #[test]
    fn the_bus_composes_the_latest_edges_and_remembers_when_each_was_set() {
        // A bus of its own: the global one is shared with the test above,
        // which checks that nothing else moved it.
        let xlate = |x: f64| Transform::from_parts(&Quaternion::identity(), Vector3::new(x, 0.0, 0.0));
        let mut b = Bus::default();
        assert!(b.last_set("map", "odom").is_none());
        let before = Instant::now();
        b.set("map", "odom", xlate(10.0));
        b.set("odom", "base_footprint", xlate(1.0));
        b.set("odom", "base_footprint", xlate(2.0));
        let t = b.graph.lookup("map", "base_footprint").unwrap();
        assert!((t.origin.x - 12.0).abs() < 1e-12, "{t:?}");
        let at = b.last_set("odom", "base_footprint").unwrap();
        assert!(at >= before && at <= Instant::now());
        assert!(b.last_set("odom", "base_footprint") >= b.last_set("map", "odom"));
        // keyed by direction, as the graph's edges are
        assert!(b.last_set("base_footprint", "odom").is_none());
    }
}
