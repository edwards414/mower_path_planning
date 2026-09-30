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

use std::sync::{Arc, Mutex, OnceLock};

use mower_localize_core::prepare::TransformTree;
use mower_localize_core::tf::Transform;

use crate::tfgraph::TfGraph;

static BUS: OnceLock<Arc<Mutex<TfGraph>>> = OnceLock::new();

fn bus() -> &'static Arc<Mutex<TfGraph>> {
    BUS.get_or_init(|| Arc::new(Mutex::new(TfGraph::new())))
}

/// Record `parent <- child`, from `/tf_static` or from our own broadcast.
pub fn set(parent: &str, child: &str, transform: Transform) {
    if let Ok(mut g) = bus().lock() {
        g.set(parent, child, transform);
    }
}

/// `(revision, tree)` with `base <- every known frame` plus `extra`, or `None`
/// when nothing has changed since `since`.
pub fn tree_if_changed(
    since: Option<u64>, base: &str, extra: &[(&str, &str)],
) -> Option<(u64, TransformTree)> {
    let g = bus().lock().ok()?;
    let rev = g.revision();
    if since == Some(rev) {
        return None;
    }
    Some((rev, g.tree_for(base, extra)))
}

#[cfg(test)]
pub fn lookup(target: &str, source: &str) -> Option<Transform> {
    bus().lock().ok()?.lookup(target, source)
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
}
