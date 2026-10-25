//! `nav2_regulated_pure_pursuit_controller::PathHandler`: the global plan RPP
//! follows, its pruning, and its transform into the robot base frame.
//!
//! Frames are implicit. The plan is kept in the frame it arrived in (`map`);
//! every cycle brings the robot pose in the costmap frame (`odom`) and the
//! `costmap <- plan` transform, which together stand in for the two tf2
//! lookups upstream makes (`plan <- costmap` for the robot pose, `base <- plan`
//! for the plan poses). With the transforms given, `transformPose` cannot fail,
//! so the `ControllerTFError` branches are gone.
//!
//! Upstream: `src/path_handler.cpp`, `nav2_util/geometry_utils.hpp`.

use crate::costmap::Costmap2D;
use crate::geometry::{euclidean_distance, Pose2D, Transform2D};
use crate::rpp::ControllerError;

/// `nav_msgs/Path` as planar poses, in a frame the caller keeps track of.
pub type Path = Vec<Pose2D>;

/// The robot pose as the transform `costmap <- base`.
pub fn pose_as_transform(pose: &Pose2D) -> Transform2D {
    Transform2D::new(pose.x, pose.y, pose.yaw)
}

/// `lookupTransform(base, plan)`: the robot pose (`costmap <- base`) chained
/// with the `costmap <- plan` transform.
pub fn base_from_plan(pose: &Pose2D, costmap_from_plan: &Transform2D) -> Transform2D {
    pose_as_transform(pose).inverse().compose(costmap_from_plan)
}

/// `nav2_util::geometry_utils::first_after_integrated_distance`: the index of
/// the first pose whose integrated distance from the start exceeds `max`, or
/// `len` if none does.
pub fn first_after_integrated_distance(poses: &[Pose2D], max: f64) -> usize {
    if poses.is_empty() {
        return 0;
    }
    let mut dist = 0.0;
    for i in 0..poses.len() - 1 {
        dist += euclidean_distance(poses[i].x, poses[i].y, poses[i + 1].x, poses[i + 1].y);
        if dist > max {
            return i + 1;
        }
    }
    poses.len()
}

/// `nav2_util::geometry_utils::min_by`: the index of the minimum, the *last*
/// one on ties (`comp <= lowest`); `len` for an empty range.
pub fn min_by(poses: &[Pose2D], value: impl Fn(&Pose2D) -> f64) -> usize {
    if poses.is_empty() {
        return 0;
    }
    let mut lowest = value(&poses[0]);
    let mut lowest_i = 0;
    for (i, p) in poses.iter().enumerate().skip(1) {
        let comp = value(p);
        if comp <= lowest {
            lowest = comp;
            lowest_i = i;
        }
    }
    lowest_i
}

/// `nav2_util::geometry_utils::calculate_path_length` from the first pose.
pub fn calculate_path_length(path: &[Pose2D]) -> f64 {
    let mut path_length = 0.0;
    for w in path.windows(2) {
        path_length += euclidean_distance(w[0].x, w[0].y, w[1].x, w[1].y);
    }
    path_length
}

/// `PathHandler`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PathHandler {
    global_plan: Path,
}

impl PathHandler {
    pub fn new() -> Self {
        Self::default()
    }

    /// `setPlan`.
    pub fn set_plan(&mut self, path: &[Pose2D]) {
        self.global_plan = path.to_vec();
    }

    /// `getPlan`: the plan as pruned so far, in the plan frame.
    pub fn plan(&self) -> &[Pose2D] {
        &self.global_plan
    }

    /// `getCostmapMaxExtent`: half the larger side, with `getSizeInMetersX`'s
    /// half-cell-short size (1.4875 m for the 3 x 3 m local costmap).
    pub fn costmap_max_extent(costmap: &Costmap2D) -> f64 {
        let max_costmap_dim_meters = costmap.size_in_meters_x().max(costmap.size_in_meters_y());
        max_costmap_dim_meters / 2.0
    }

    /// `transformGlobalPlan`: find the plan pose closest to the robot within
    /// `max_robot_pose_search_dist` of integrated path length, prune the plan
    /// up to it, and return the poses from there up to the first one farther
    /// than the costmap extent from the robot, in the robot base frame.
    ///
    /// The pruning happens before the empty-result check, so a plan that ends
    /// in `InvalidPath("Resulting plan has 0 poses in it.")` has still been
    /// pruned, as upstream.
    pub fn transform_global_plan(
        &mut self,
        pose: &Pose2D,
        costmap_from_plan: &Transform2D,
        costmap: &Costmap2D,
        max_robot_pose_search_dist: f64,
        reject_unit_path: bool,
    ) -> Result<Path, ControllerError> {
        if self.global_plan.is_empty() {
            return Err(ControllerError::InvalidPath("Received plan with zero length"));
        }
        if reject_unit_path && self.global_plan.len() == 1 {
            return Err(ControllerError::InvalidPath("Received plan with length of one"));
        }

        // transformPose(global_plan_.header.frame_id, pose, robot_pose)
        let robot_pose = costmap_from_plan.inverse().apply_pose(pose);

        let closest_pose_upper_bound = first_after_integrated_distance(&self.global_plan, max_robot_pose_search_dist);

        // The closest pose, bounded so that a later portion of a path that
        // turns back is not picked.
        let mut transformation_begin = min_by(&self.global_plan[..closest_pose_upper_bound], |ps| {
            euclidean_distance(robot_pose.x, robot_pose.y, ps.x, ps.y)
        });

        // Keep at least 2 poses (to interpolate the end-of-path direction).
        // Upstream steps before begin() when the bound is the second pose (a
        // first segment longer than max_robot_pose_search_dist): undefined
        // behaviour there, clamped to the first pose here.
        if closest_pose_upper_bound != 0
            && self.global_plan.len() > 1
            && transformation_begin == closest_pose_upper_bound - 1
        {
            transformation_begin = closest_pose_upper_bound.saturating_sub(2);
        }

        // Discard the poses outside the local costmap: everything from the
        // first one beyond the extent, even if the plan comes back later.
        let max_costmap_extent = Self::costmap_max_extent(costmap);
        let transformation_end = self.global_plan[transformation_begin..]
            .iter()
            .position(|p| euclidean_distance(p.x, p.y, robot_pose.x, robot_pose.y) > max_costmap_extent)
            .map_or(self.global_plan.len(), |i| transformation_begin + i);

        let base_from_plan = base_from_plan(pose, costmap_from_plan);
        let transformed_plan: Path = self.global_plan[transformation_begin..transformation_end]
            .iter()
            .map(|p| base_from_plan.apply_pose(p))
            .collect();

        // Path pruning: drop what is already behind.
        self.global_plan.drain(..transformation_begin);

        if transformed_plan.is_empty() {
            return Err(ControllerError::InvalidPath("Resulting plan has 0 poses in it."));
        }
        Ok(transformed_plan)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(n: usize, step: f64) -> Path {
        (0..n).map(|i| Pose2D::new(i as f64 * step, 0.0, 0.0)).collect()
    }

    #[test]
    fn integrated_distance_bound() {
        let p = line(5, 1.0);
        assert_eq!(first_after_integrated_distance(&p, 2.5), 3);
        assert_eq!(first_after_integrated_distance(&p, 10.0), 5);
        assert_eq!(first_after_integrated_distance(&p, 0.5), 1);
        assert_eq!(first_after_integrated_distance(&p[..1], 0.5), 1);
    }

    #[test]
    fn min_by_takes_the_last_tie() {
        let p = vec![Pose2D::new(-1.0, 0.0, 0.0), Pose2D::new(1.0, 0.0, 0.0), Pose2D::new(3.0, 0.0, 0.0)];
        assert_eq!(min_by(&p, |q| q.x.abs()), 1);
    }

    #[test]
    fn closest_last_pose_keeps_two() {
        let costmap = Costmap2D::new(60, 60, 0.05, -1.5, -1.5, 0);
        let mut h = PathHandler::new();
        h.set_plan(&line(5, 0.1));
        let t = h
            .transform_global_plan(&Pose2D::new(0.5, 0.0, 0.0), &Transform2D::IDENTITY, &costmap, 10.0, false)
            .unwrap();
        assert_eq!(t.len(), 2);
        assert_eq!(h.plan().len(), 2);
        assert!((t[0].x + 0.2).abs() < 1e-12 && (t[1].x + 0.1).abs() < 1e-12);
    }

    #[test]
    fn far_from_the_plan_is_invalid_but_pruned() {
        let costmap = Costmap2D::new(60, 60, 0.05, -1.5, -1.5, 0);
        let mut h = PathHandler::new();
        h.set_plan(&line(10, 0.1));
        let r = h.transform_global_plan(&Pose2D::new(0.43, 2.0, 0.0), &Transform2D::IDENTITY, &costmap, 10.0, false);
        assert_eq!(r, Err(ControllerError::InvalidPath("Resulting plan has 0 poses in it.")));
        assert_eq!(h.plan().len(), 6);
    }
}
