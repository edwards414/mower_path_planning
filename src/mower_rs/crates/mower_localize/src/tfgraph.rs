//! The little bit of `tf2_ros::Buffer` this module needs.
//!
//! `mower_localize_core::prepare::TransformTree` is a flat map of
//! already-resolved transforms, because every lookup the ported code makes is
//! either the identity or between two frames that are one rigid link apart in
//! spirit. In the live graph they are not always one *message* apart:
//! `base_footprint -> imu_link` can be several `/tf_static` edges of the URDF,
//! and `map -> base_footprint` is the composition of the two EKF broadcasts.
//!
//! So this keeps the raw edges from `/tf` and `/tf_static` (latest wins, which
//! is what `lookupTransformSafe(..., Time(0))` asks tf2 for) and walks them,
//! then hands the core a `TransformTree` holding exactly the pairs the core
//! will look up. No time interpolation: tf2 at `Time(0)` returns the latest
//! available transform, and every edge here is either static or republished at
//! 20 Hz or faster.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use mower_localize_core::prepare::TransformTree;
use mower_localize_core::tf::Transform;

/// Edges keyed `(parent, child)`, i.e. exactly a `TransformStamped`'s
/// `header.frame_id` and `child_frame_id`.
#[derive(Clone, Debug, Default)]
pub struct TfGraph {
    edges: BTreeMap<(String, String), Transform>,
    /// Bumped on every change so a consumer can rebuild its tree only when
    /// something actually moved.
    revision: u64,
}

impl TfGraph {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn revision(&self) -> u64 {
        self.revision
    }

    pub fn set(&mut self, parent: &str, child: &str, transform: Transform) {
        self.edges
            .insert((parent.to_string(), child.to_string()), transform);
        self.revision += 1;
    }

    /// `tf2::BufferCore::lookupTransform(target, source, Time(0))`: the pose of
    /// `source` expressed in `target`. Breadth-first over the undirected graph,
    /// so the shortest chain wins and a cycle cannot loop.
    pub fn lookup(&self, target: &str, source: &str) -> Option<Transform> {
        if target == source {
            return Some(Transform::identity());
        }
        let mut seen: BTreeSet<&str> = BTreeSet::new();
        seen.insert(target);
        let mut queue: VecDeque<(&str, Transform)> = VecDeque::new();
        queue.push_back((target, Transform::identity()));
        while let Some((frame, acc)) = queue.pop_front() {
            for ((parent, child), t) in &self.edges {
                // acc is target <- frame; each edge extends it by one link.
                let (next, step) = if parent == frame {
                    (child.as_str(), *t)
                } else if child == frame {
                    (parent.as_str(), t.inverse())
                } else {
                    continue;
                };
                if !seen.insert(next) {
                    continue;
                }
                let reached = acc.mul(&step);
                if next == source {
                    return Some(reached);
                }
                queue.push_back((next, reached));
            }
        }
        None
    }

    /// Every frame name the graph knows.
    pub fn frames(&self) -> BTreeSet<String> {
        let mut out = BTreeSet::new();
        for (parent, child) in self.edges.keys() {
            out.insert(parent.clone());
            out.insert(child.clone());
        }
        out
    }

    /// A `TransformTree` holding `base <- f` for every frame `f` in the graph,
    /// plus any extra pairs the caller names. That covers every lookup the
    /// ported code makes: the sensor offsets (`base_footprint <- imu_link`,
    /// `<- gps_link`), `base_footprint <- odom` for the map EKF's `map -> odom`
    /// composition, and `map <- base_footprint` for navsat's
    /// `getRobotOriginWorldPose`.
    pub fn tree_for(&self, base: &str, extra: &[(&str, &str)]) -> TransformTree {
        let mut tree = TransformTree::new();
        for frame in self.frames() {
            if let Some(t) = self.lookup(base, &frame) {
                tree.insert(base, &frame, t);
            }
        }
        for (target, source) in extra {
            if let Some(t) = self.lookup(target, source) {
                tree.insert(target, source, t);
            }
        }
        tree
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mower_localize_core::tf::{Quaternion, Vector3};

    fn xlate(x: f64, y: f64, z: f64) -> Transform {
        Transform::from_parts(&Quaternion::identity(), Vector3::new(x, y, z))
    }

    fn close(a: &Transform, b: &Transform) -> bool {
        (a.origin.x - b.origin.x).abs() < 1e-12
            && (a.origin.y - b.origin.y).abs() < 1e-12
            && (a.origin.z - b.origin.z).abs() < 1e-12
    }

    #[test]
    fn a_chain_of_static_links_composes_in_both_directions() {
        let mut g = TfGraph::new();
        g.set("base_footprint", "base_link", xlate(0.0, 0.0, 0.1));
        g.set("base_link", "imu_link", xlate(0.2, 0.0, 0.05));
        let t = g.lookup("base_footprint", "imu_link").unwrap();
        assert!(close(&t, &xlate(0.2, 0.0, 0.15)), "{t:?}");
        let back = g.lookup("imu_link", "base_footprint").unwrap();
        assert!(close(&back, &xlate(-0.2, 0.0, -0.15)), "{back:?}");
    }

    #[test]
    fn identity_for_the_same_frame_and_none_for_a_disconnected_one() {
        let mut g = TfGraph::new();
        g.set("odom", "base_footprint", xlate(1.0, 2.0, 0.0));
        assert!(close(&g.lookup("odom", "odom").unwrap(), &Transform::identity()));
        assert!(g.lookup("odom", "utm").is_none());
    }

    #[test]
    fn the_latest_edge_wins_like_a_time_zero_lookup() {
        let mut g = TfGraph::new();
        g.set("odom", "base_footprint", xlate(1.0, 0.0, 0.0));
        let first = g.revision();
        g.set("odom", "base_footprint", xlate(2.0, 0.0, 0.0));
        assert!(g.revision() > first);
        assert!(close(&g.lookup("odom", "base_footprint").unwrap(), &xlate(2.0, 0.0, 0.0)));
    }

    #[test]
    fn the_tree_carries_the_pairs_the_core_looks_up() {
        let mut g = TfGraph::new();
        g.set("map", "odom", xlate(10.0, 0.0, 0.0));
        g.set("odom", "base_footprint", xlate(1.0, 0.0, 0.0));
        g.set("base_footprint", "gps_link", xlate(0.0, 0.3, 0.4));
        let tree = g.tree_for("base_footprint", &[("map", "base_footprint")]);
        assert!(close(&tree.lookup("base_footprint", "odom").unwrap(), &xlate(-1.0, 0.0, 0.0)));
        assert!(close(&tree.lookup("base_footprint", "gps_link").unwrap(), &xlate(0.0, 0.3, 0.4)));
        assert!(close(&tree.lookup("map", "base_footprint").unwrap(), &xlate(11.0, 0.0, 0.0)));
    }
}
