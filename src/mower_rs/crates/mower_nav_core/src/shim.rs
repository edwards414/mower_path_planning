//! `nav2_rotation_shim_controller::RotationShimController`: rotates the robot
//! in place towards a new path before handing over to the primary controller
//! (RPP here).
//!
//! After every `setPlan` (`rotate_to_heading_once` is false) the shim looks
//! at the plan pose `forward_sampling_distance` along the plan *from the
//! plan's first pose* (not from the robot), and while the angle to it in the
//! robot frame exceeds `angular_dist_threshold` (0.6) it commands an
//! accel-limited in-place rotation; once rotating it keeps going until the
//! angle is within `angular_disengage_threshold` (0.35). From then on, until
//! the next `setPlan`, every cycle goes to the primary controller: turns
//! *within* a plan are RPP's alone. A rotation that would collide (or a plan
//! of one pose) is not an error: the shim gives up on rotating and passes the
//! cycle to the primary controller.
//!
//! `rotate_to_goal_heading` (false here) and `use_path_orientations` (false
//! here) are ported too. No dynamic parameters, no mutexes; tf2 is replaced
//! by the transform in [`ControlInput`], which also means the shim's lookup
//! at `clock_->now()` (it waits up to 0.1 s for a newer `odom -> base`) is
//! the lookup at the pose's time here.
//!
//! Upstream: `nav2_rotation_shim_controller/src/nav2_rotation_shim_controller.cpp`,
//! `nav2_controller/plugins/position_goal_checker.cpp`.

use crate::costmap::NO_INFORMATION;
use crate::footprint::FootprintCollisionChecker;
use crate::geometry::{shortest_angular_distance, Pose2D, Twist2D};
use crate::path_handler::{base_from_plan, Path};
use crate::rpp::{std_clamp, ControlInput, Controller, ControllerError, CostmapContext, RegulatedPurePursuit};

/// The shim's parameters plus the controller server's
/// `controller_frequency`. `Default` is upstream's defaults.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShimParams {
    pub angular_dist_threshold: f64,
    pub angular_disengage_threshold: f64,
    pub forward_sampling_distance: f64,
    pub rotate_to_heading_angular_vel: f64,
    pub max_angular_accel: f64,
    pub max_cost_threshold: f64,
    pub simulate_ahead_time: f64,
    pub rotate_to_goal_heading: bool,
    pub rotate_to_heading_once: bool,
    pub closed_loop: bool,
    pub use_path_orientations: bool,
    /// The internal `PositionGoalChecker`'s `stateful`. Upstream initialises
    /// it before setting `plugin_name_`, so its parameters live under
    /// `.position_checker.*`, never in the yaml; its `xy_goal_tolerance` is
    /// overwritten from the goal checker every cycle anyway.
    pub position_checker_stateful: bool,
    /// The controller server's `controller_frequency`.
    pub controller_frequency: f64,
}

impl Default for ShimParams {
    fn default() -> Self {
        Self {
            angular_dist_threshold: 0.785, // 45 deg
            angular_disengage_threshold: 0.785 / 2.0,
            forward_sampling_distance: 0.5,
            rotate_to_heading_angular_vel: 1.8,
            max_angular_accel: 3.2,
            max_cost_threshold: crate::costmap::LETHAL_OBSTACLE as f64,
            simulate_ahead_time: 1.0,
            rotate_to_goal_heading: false,
            rotate_to_heading_once: false,
            closed_loop: true,
            use_path_orientations: false,
            position_checker_stateful: true,
            controller_frequency: 20.0,
        }
    }
}

impl ShimParams {
    /// `controller_server.FollowPath` of `nav2_no_map_params.yaml`.
    pub fn mower() -> Self {
        Self {
            angular_dist_threshold: 0.6,
            forward_sampling_distance: 0.25,
            angular_disengage_threshold: 0.35,
            rotate_to_heading_angular_vel: 0.6,
            max_angular_accel: 1.2,
            simulate_ahead_time: 1.0,
            rotate_to_goal_heading: false,
            controller_frequency: 20.0,
            ..Self::default()
        }
    }
}

/// `nav2_controller::PositionGoalChecker`, the shim's own.
#[derive(Debug, Clone, Copy, PartialEq)]
struct PositionGoalChecker {
    xy_goal_tolerance_sq: f64,
    stateful: bool,
    position_reached: bool,
}

impl PositionGoalChecker {
    fn new(stateful: bool) -> Self {
        // the parameter default 0.25, never set from the yaml
        Self { xy_goal_tolerance_sq: 0.25 * 0.25, stateful, position_reached: false }
    }

    fn reset(&mut self) {
        self.position_reached = false;
    }

    fn set_xy_goal_tolerance(&mut self, tolerance: f64) {
        self.xy_goal_tolerance_sq = tolerance * tolerance;
    }

    fn is_goal_reached(&mut self, query_pose: &Pose2D, goal_pose: &Pose2D) -> bool {
        if self.stateful && self.position_reached {
            return true;
        }
        let dx = query_pose.x - goal_pose.x;
        let dy = query_pose.y - goal_pose.y;
        let position_reached = dx * dx + dy * dy <= self.xy_goal_tolerance_sq;
        if self.stateful && position_reached {
            self.position_reached = true;
        }
        position_reached
    }
}

/// `RotationShimController` around a primary [`Controller`].
#[derive(Debug, Clone)]
pub struct RotationShim<C: Controller> {
    params: ShimParams,
    context: CostmapContext,
    primary: C,
    control_duration: f64,
    path_updated: bool,
    current_path: Path,
    in_rotation: bool,
    last_angular_vel: f64,
    position_goal_checker: PositionGoalChecker,
}

impl RotationShim<RegulatedPurePursuit> {
    /// This robot's `FollowPath`: the shim around RPP on the local costmap.
    pub fn mower() -> Self {
        Self::new(ShimParams::mower(), CostmapContext::local_costmap(), RegulatedPurePursuit::mower())
    }
}

impl<C: Controller> RotationShim<C> {
    /// `configure` followed by `activate`.
    pub fn new(params: ShimParams, context: CostmapContext, primary: C) -> Self {
        let mut position_goal_checker = PositionGoalChecker::new(params.position_checker_stateful);
        position_goal_checker.reset();
        Self {
            control_duration: 1.0 / params.controller_frequency,
            params,
            context,
            primary,
            path_updated: false,
            current_path: Path::new(),
            in_rotation: false,
            last_angular_vel: f64::MAX,
            position_goal_checker,
        }
    }

    pub fn params(&self) -> &ShimParams {
        &self.params
    }

    pub fn primary(&self) -> &C {
        &self.primary
    }

    pub fn primary_mut(&mut self) -> &mut C {
        &mut self.primary
    }

    /// `path_updated_`: the next cycles still check the heading to the plan.
    pub fn path_updated(&self) -> bool {
        self.path_updated
    }

    /// `in_rotation_`: the last cycle was an in-place rotation.
    pub fn in_rotation(&self) -> bool {
        self.in_rotation
    }

    /// `getSampledPathPt`: the first plan pose at least
    /// `forward_sampling_distance` from the plan's first pose, else the last.
    fn sampled_path_pt(&self) -> Result<Pose2D, ControllerError> {
        if self.current_path.len() < 2 {
            return Err(ControllerError::Controller(
                "Path is too short to find a valid sampled path point for rotation.",
            ));
        }
        let start = self.current_path[0];
        for p in &self.current_path[1..] {
            let dx = p.x - start.x;
            let dy = p.y - start.y;
            if dx.hypot(dy) >= self.params.forward_sampling_distance {
                return Ok(*p);
            }
        }
        Ok(self.current_path[self.current_path.len() - 1])
    }

    /// `getSampledPathGoal`.
    fn sampled_path_goal(&self) -> Result<Pose2D, ControllerError> {
        match self.current_path.last() {
            Some(goal) => Ok(*goal),
            None => Err(ControllerError::InvalidPath("Path is empty - cannot find a goal point")),
        }
    }

    /// `computeRotateToHeadingCommand`.
    fn compute_rotate_to_heading_command(
        &self,
        angular_distance_to_heading: f64,
        input: &ControlInput,
    ) -> Result<Twist2D, ControllerError> {
        let mut current = if self.params.closed_loop { input.speed.theta } else { self.last_angular_vel };
        if current == f64::MAX {
            current = 0.0;
        }

        let sign = if angular_distance_to_heading > 0.0 { 1.0 } else { -1.0 };
        let angular_vel = sign * self.params.rotate_to_heading_angular_vel;
        let dt = self.control_duration;
        let min_feasible_angular_speed = current - self.params.max_angular_accel * dt;
        let max_feasible_angular_speed = current + self.params.max_angular_accel * dt;
        let mut w = std_clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);

        // Slow down so as not to overshoot.
        let max_vel_to_stop = (2.0 * self.params.max_angular_accel * angular_distance_to_heading.abs()).sqrt();
        if w.abs() > max_vel_to_stop {
            w = sign * max_vel_to_stop;
        }

        let cmd_vel = Twist2D::new(0.0, 0.0, w);
        self.is_collision_free(&cmd_vel, angular_distance_to_heading, &input.pose, input)?;
        Ok(cmd_vel)
    }

    /// `isCollisionFree`: the footprint at the robot's position, rotated by
    /// the command in `control_duration` steps for `simulate_ahead_time`, up
    /// to where the primary controller would take over. That stop is
    /// one-sided upstream: `shortest_angular_distance(yaw, initial_yaw)` is
    /// negative for a counter-clockwise command, so a CCW rotation is always
    /// simulated for the full time and a CW one stops at the threshold.
    fn is_collision_free(
        &self,
        cmd_vel: &Twist2D,
        angular_distance_to_heading: f64,
        pose: &Pose2D,
        input: &ControlInput,
    ) -> Result<(), ControllerError> {
        let checker = FootprintCollisionChecker::new(input.costmap);
        let mut simulated_time = 0.0;
        let initial_yaw = pose.yaw;
        let remaining_rotation_before_thresh = angular_distance_to_heading.abs() - self.params.angular_dist_threshold;

        while simulated_time < self.params.simulate_ahead_time {
            simulated_time += self.control_duration;
            let yaw = initial_yaw + cmd_vel.theta * simulated_time;

            // Stop past the point it would be passed to the primary controller.
            if shortest_angular_distance(yaw, initial_yaw) >= remaining_rotation_before_thresh {
                break;
            }

            let footprint_cost = checker.footprint_cost_at_pose(pose.x, pose.y, yaw, &self.context.footprint);
            if footprint_cost == NO_INFORMATION as f64 && self.context.tracking_unknown {
                return Err(ControllerError::NoValidControl(
                    "RotationShimController detected a potential collision ahead!",
                ));
            }
            if footprint_cost >= self.params.max_cost_threshold {
                return Err(ControllerError::NoValidControl("RotationShimController detected collision ahead!"));
            }
        }
        Ok(())
    }

    /// The `rotate_to_goal_heading` block: `Ok(Some)` rotates towards the
    /// plan's last orientation once within the xy tolerance of it.
    fn rotate_to_goal_heading_command(&mut self, input: &ControlInput) -> Result<Option<Twist2D>, ControllerError> {
        let goal = self.sampled_path_goal()?;
        // transformPoseInTargetFrame(goal, pose.header.frame_id)
        let goal = input.costmap_from_plan.apply_pose(&goal);
        self.position_goal_checker.set_xy_goal_tolerance(input.xy_goal_tolerance);
        if self.position_goal_checker.is_goal_reached(&input.pose, &goal) {
            let angular_distance_to_heading = shortest_angular_distance(input.pose.yaw, goal.yaw);
            return self.compute_rotate_to_heading_command(angular_distance_to_heading, input).map(Some);
        }
        Ok(None)
    }

    /// `isGoalChanged` (for `rotate_to_heading_once`).
    fn is_goal_changed(&self, path: &[Pose2D]) -> bool {
        if self.in_rotation || self.current_path.is_empty() {
            return true;
        }
        self.current_path.last() != path.last()
    }
}

impl<C: Controller> Controller for RotationShim<C> {
    /// `setPlan`: every new plan re-arms the heading check.
    fn set_plan(&mut self, path: &[Pose2D]) {
        self.path_updated = if self.params.rotate_to_heading_once { self.is_goal_changed(path) } else { true };
        self.current_path = path.to_vec();
        self.primary.set_plan(path);
        self.position_goal_checker.reset();
    }

    /// `computeVelocityCommands`.
    fn compute_velocity_commands(&mut self, input: &ControlInput) -> Result<Twist2D, ControllerError> {
        if self.params.rotate_to_goal_heading {
            // Any failure (no plan, a rotational collision) is logged and the
            // cycle carries on below.
            if let Ok(Some(cmd_vel)) = self.rotate_to_goal_heading_command(input) {
                self.last_angular_vel = cmd_vel.theta;
                return Ok(cmd_vel);
            }
        }

        if self.path_updated {
            match self.sampled_path_pt() {
                Ok(sampled_pt) => {
                    let angular_distance_to_heading = if self.params.use_path_orientations {
                        // upstream compares the odom yaw with the plan-frame yaw
                        shortest_angular_distance(input.pose.yaw, sampled_pt.yaw)
                    } else {
                        let (x, y) = base_from_plan(&input.pose, &input.costmap_from_plan)
                            .apply_point(sampled_pt.x, sampled_pt.y);
                        y.atan2(x)
                    };

                    let angular_thresh = if self.in_rotation {
                        self.params.angular_disengage_threshold
                    } else {
                        self.params.angular_dist_threshold
                    };
                    if angular_distance_to_heading.abs() > angular_thresh {
                        // Not within the new path's rough heading: rotate.
                        self.in_rotation = true;
                        match self.compute_rotate_to_heading_command(angular_distance_to_heading, input) {
                            Ok(cmd_vel) => {
                                self.last_angular_vel = cmd_vel.theta;
                                return Ok(cmd_vel);
                            }
                            Err(_) => self.path_updated = false,
                        }
                    } else {
                        self.path_updated = false;
                    }
                }
                Err(_) => self.path_updated = false,
            }
        }

        // From here the primary controller tracks the path.
        self.in_rotation = false;
        let cmd_vel = self.primary.compute_velocity_commands(input)?;
        self.last_angular_vel = cmd_vel.theta;
        Ok(cmd_vel)
    }

    /// `setSpeedLimit`: the primary's.
    fn set_speed_limit(&mut self, speed_limit: f64, percentage: bool) {
        self.primary.set_speed_limit(speed_limit, percentage);
    }

    /// `reset`: `in_rotation_` and `path_updated_` survive it.
    fn reset(&mut self) {
        self.last_angular_vel = f64::MAX;
        self.primary.reset();
        self.position_goal_checker.reset();
    }

    // `cancel` is nav2_core's default (true): the shim does not forward it,
    // so RPP's cancel deceleration is never reached through the shim.
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::costmap::Costmap2D;
    use crate::geometry::Transform2D;

    fn input(costmap: &Costmap2D, pose: Pose2D, w: f64) -> ControlInput<'_> {
        ControlInput {
            pose,
            speed: Twist2D::new(0.0, 0.0, w),
            xy_goal_tolerance: 0.05,
            costmap_from_plan: Transform2D::IDENTITY,
            costmap,
        }
    }

    #[test]
    fn engages_rotates_and_disengages() {
        let costmap = Costmap2D::new(60, 60, 0.05, -1.5, -1.5, 0);
        let mut shim = RotationShim::mower();
        let plan: Vec<Pose2D> = (0..20).map(|i| Pose2D::new(0.0, 0.1 * i as f64, 0.0)).collect();
        shim.set_plan(&plan);
        // the plan heads +y, the robot faces +x: 90 deg to the left
        let cmd = shim.compute_velocity_commands(&input(&costmap, Pose2D::new(0.0, 0.0, 0.0), 0.0)).unwrap();
        assert!(shim.in_rotation() && cmd.x == 0.0);
        assert!((cmd.theta - 1.2 * 0.05).abs() < 1e-15, "accel limited: {}", cmd.theta);
        // within the disengage threshold: RPP takes over
        let cmd = shim.compute_velocity_commands(&input(&costmap, Pose2D::new(0.0, 0.0, 1.3), 0.5)).unwrap();
        assert!(!shim.in_rotation() && !shim.path_updated() && cmd.x > 0.0);
    }

    #[test]
    fn a_one_pose_plan_goes_to_the_primary() {
        let costmap = Costmap2D::new(60, 60, 0.05, -1.5, -1.5, 0);
        let mut shim = RotationShim::mower();
        shim.set_plan(&[Pose2D::new(0.0, 1.0, 0.0)]);
        let cmd = shim.compute_velocity_commands(&input(&costmap, Pose2D::new(0.0, 0.0, 0.0), 0.0)).unwrap();
        assert!(!shim.path_updated() && cmd.x > 0.0);
    }
}
