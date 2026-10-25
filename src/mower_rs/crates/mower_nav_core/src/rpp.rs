//! `nav2_regulated_pure_pursuit_controller` (RPP) and the `nav2_core`
//! controller interface it implements and `RotationShimController` wraps.
//!
//! The controller the mower runs is `FollowPath`: a [`RotationShim`] around
//! [`RegulatedPurePursuit`], on the local costmap (frame `odom`, the
//! [`RollingWindow`], `robot_radius: 0.5`, `track_unknown_space: false`), with
//! the plan in `map`. [`RppParams::mower`] is that yaml section.
//!
//! What this configuration reaches each cycle: transform and prune the plan
//! (`PathHandler`), fixed 0.25 m lookahead, carrot by circle / segment
//! intersection, curvature, the curvature speed limit (radius < 0.6 m), the
//! 0.08 m/s floor, the approach slow-down (< 0.4 m of path left, floor
//! 0.03 m/s), `angular = linear * curvature`, then the forward-simulated
//! collision check up to the carrot. Rotate-to-heading, reversing, velocity
//! scaled lookahead, the fixed curvature lookahead, cost regulation and
//! cancel deceleration are ported line by line but off here.
//!
//! The ROS plumbing is gone: no publishers ([`RegulatedPurePursuit::transformed_plan`]
//! and [`RegulatedPurePursuit::carrot`] keep what `received_global_plan` and
//! `lookahead_point` showed), no dynamic parameters, no mutexes, and tf2 is
//! replaced by the transform passed in [`ControlInput`].
//!
//! Upstream: `src/regulated_pure_pursuit_controller.cpp`,
//! `src/collision_checker.cpp`, `src/parameter_handler.cpp`,
//! `include/.../regulation_functions.hpp`, `nav2_core/controller.hpp`,
//! `nav2_core/controller_exceptions.hpp`.
//!
//! [`RotationShim`]: crate::shim::RotationShim
//! [`RollingWindow`]: crate::costmap::RollingWindow

use std::fmt;

use crate::costmap::{Costmap2D, RollingWindow, FREE_SPACE, LETHAL_OBSTACLE, NO_INFORMATION};
use crate::footprint::{calculate_min_and_max_distances, robot_footprint, Footprint, FootprintCollisionChecker};
use crate::geometry::{normalize_angle, Pose2D, Transform2D, Twist2D};
use crate::path_handler::{calculate_path_length, Path, PathHandler};

// --- nav2_core -------------------------------------------------------------

/// The `nav2_core` controller exceptions (`controller_exceptions.hpp`), each
/// carrying upstream's `what()`. All of them derive from
/// `ControllerException`, so a `catch (std::runtime_error &)` (the shim's)
/// catches every one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControllerError {
    /// `ControllerException` itself.
    Controller(&'static str),
    InvalidController(&'static str),
    ControllerTFError(&'static str),
    FailedToMakeProgress(&'static str),
    PatienceExceeded(&'static str),
    InvalidPath(&'static str),
    NoValidControl(&'static str),
    ControllerTimedOut(&'static str),
}

impl ControllerError {
    /// The C++ class name.
    pub fn class_name(&self) -> &'static str {
        match self {
            Self::Controller(_) => "ControllerException",
            Self::InvalidController(_) => "InvalidController",
            Self::ControllerTFError(_) => "ControllerTFError",
            Self::FailedToMakeProgress(_) => "FailedToMakeProgress",
            Self::PatienceExceeded(_) => "PatienceExceeded",
            Self::InvalidPath(_) => "InvalidPath",
            Self::NoValidControl(_) => "NoValidControl",
            Self::ControllerTimedOut(_) => "ControllerTimedOut",
        }
    }

    /// `what()`.
    pub fn what(&self) -> &'static str {
        match *self {
            Self::Controller(m)
            | Self::InvalidController(m)
            | Self::ControllerTFError(m)
            | Self::FailedToMakeProgress(m)
            | Self::PatienceExceeded(m)
            | Self::InvalidPath(m)
            | Self::NoValidControl(m)
            | Self::ControllerTimedOut(m) => m,
        }
    }
}

impl fmt::Display for ControllerError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.class_name(), self.what())
    }
}

impl std::error::Error for ControllerError {}

/// One `computeVelocityCommands` call's inputs.
#[derive(Debug, Clone, Copy)]
pub struct ControlInput<'a> {
    /// `pose`: the robot in the costmap's global frame (`odom`).
    pub pose: Pose2D,
    /// `velocity`: the odometry twist, after the controller server's
    /// `getThresholdedTwist`.
    pub speed: Twist2D,
    /// `goal_checker->getTolerances()`'s `pose_tolerance.position.x` (the
    /// `xy_goal_tolerance`; nav2's goal checkers always return true).
    pub xy_goal_tolerance: f64,
    /// `lookupTransform(costmap frame, plan frame)`: maps plan poses into
    /// the costmap frame. Together with `pose` it stands in for tf2.
    pub costmap_from_plan: Transform2D,
    /// `costmap_ros->getCostmap()`.
    pub costmap: &'a Costmap2D,
}

/// `nav2_core::Controller` without the lifecycle (`configure` is the
/// constructor, `activate` / `deactivate` / `cleanup` have nothing left to do).
pub trait Controller {
    /// `setPlan`: the global plan, in the frame `costmap_from_plan` maps from.
    fn set_plan(&mut self, path: &[Pose2D]);

    /// `computeVelocityCommands`: the command in the robot base frame.
    fn compute_velocity_commands(&mut self, input: &ControlInput) -> Result<Twist2D, ControllerError>;

    /// `setSpeedLimit`: `speed_limit` in m/s, or in percent of the configured
    /// speed when `percentage`; [`NO_SPEED_LIMIT`] restores the configured one.
    fn set_speed_limit(&mut self, speed_limit: f64, percentage: bool);

    /// `reset`: called by the controller server after every task.
    fn reset(&mut self);

    /// `cancel`: true when the robot may be stopped now (nav2_core's default).
    fn cancel(&mut self) -> bool {
        true
    }
}

/// `nav2_costmap_2d::NO_SPEED_LIMIT` (`costmap_filters/filter_values.hpp`).
pub const NO_SPEED_LIMIT: f64 = 0.0;

/// What the controllers read from `Costmap2DROS` besides the grid.
#[derive(Debug, Clone, PartialEq)]
pub struct CostmapContext {
    /// `getRobotFootprint()`: the padded footprint.
    pub footprint: Footprint,
    /// `getLayeredCostmap()->getInscribedRadius()`.
    pub inscribed_radius: f64,
    /// `getLayeredCostmap()->getCircumscribedRadius()`.
    pub circumscribed_radius: f64,
    /// `getLayeredCostmap()->isTrackingUnknown()`.
    pub tracking_unknown: bool,
}

impl CostmapContext {
    /// `LayeredCostmap::setFootprint` derives both radii from the padded
    /// footprint.
    pub fn new(footprint: Footprint, tracking_unknown: bool) -> Self {
        let (inscribed_radius, circumscribed_radius) = calculate_min_and_max_distances(&footprint);
        Self { footprint, inscribed_radius, circumscribed_radius, tracking_unknown }
    }

    /// This robot's local costmap: `robot_radius: 0.5`, `track_unknown_space`
    /// left at false.
    pub fn local_costmap() -> Self {
        Self::new(robot_footprint(0.5), false)
    }
}

// --- parameters --------------------------------------------------------------

/// `Parameters` (`parameter_handler.hpp`) plus the controller server's
/// `controller_frequency`. `Default` is upstream's defaults.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RppParams {
    pub desired_linear_vel: f64,
    pub lookahead_dist: f64,
    pub min_lookahead_dist: f64,
    pub max_lookahead_dist: f64,
    pub lookahead_time: f64,
    pub rotate_to_heading_angular_vel: f64,
    /// Only bounds tf2 lookups upstream; unused here.
    pub transform_tolerance: f64,
    pub use_velocity_scaled_lookahead_dist: bool,
    pub min_approach_linear_velocity: f64,
    pub approach_velocity_scaling_dist: f64,
    pub max_allowed_time_to_collision_up_to_carrot: f64,
    pub use_regulated_linear_velocity_scaling: bool,
    pub use_cost_regulated_linear_velocity_scaling: bool,
    pub cost_scaling_dist: f64,
    pub cost_scaling_gain: f64,
    pub inflation_cost_scaling_factor: f64,
    pub regulated_linear_scaling_min_radius: f64,
    pub regulated_linear_scaling_min_speed: f64,
    pub use_fixed_curvature_lookahead: bool,
    pub curvature_lookahead_dist: f64,
    pub use_rotate_to_heading: bool,
    pub rotate_to_heading_min_angle: f64,
    pub max_angular_accel: f64,
    pub use_cancel_deceleration: bool,
    pub cancel_deceleration: f64,
    pub allow_reversing: bool,
    /// Upstream's default is half the costmap's `getSizeInMetersX()`.
    pub max_robot_pose_search_dist: f64,
    pub interpolate_curvature_after_goal: bool,
    pub use_collision_detection: bool,
    pub stateful: bool,
    /// The controller server's `controller_frequency` (RPP falls back to 20
    /// when it is not declared).
    pub controller_frequency: f64,
}

impl RppParams {
    /// Upstream's defaults for a costmap `costmap_size_x` metres wide
    /// (`getSizeInMetersX()`).
    pub fn defaults_for_costmap(costmap_size_x: f64) -> Self {
        Self {
            desired_linear_vel: 0.5,
            lookahead_dist: 0.6,
            min_lookahead_dist: 0.3,
            max_lookahead_dist: 0.9,
            lookahead_time: 1.5,
            rotate_to_heading_angular_vel: 1.8,
            transform_tolerance: 0.1,
            use_velocity_scaled_lookahead_dist: false,
            min_approach_linear_velocity: 0.05,
            approach_velocity_scaling_dist: 0.6,
            max_allowed_time_to_collision_up_to_carrot: 1.0,
            use_regulated_linear_velocity_scaling: true,
            use_cost_regulated_linear_velocity_scaling: true,
            cost_scaling_dist: 0.6,
            cost_scaling_gain: 1.0,
            inflation_cost_scaling_factor: 3.0,
            regulated_linear_scaling_min_radius: 0.90,
            regulated_linear_scaling_min_speed: 0.25,
            use_fixed_curvature_lookahead: false,
            curvature_lookahead_dist: 0.6,
            use_rotate_to_heading: true,
            rotate_to_heading_min_angle: 0.785,
            max_angular_accel: 3.2,
            use_cancel_deceleration: false,
            cancel_deceleration: 3.2,
            allow_reversing: false,
            max_robot_pose_search_dist: costmap_size_x / 2.0,
            interpolate_curvature_after_goal: false,
            use_collision_detection: true,
            stateful: true,
            controller_frequency: 20.0,
        }
    }

    /// `controller_server.FollowPath` of `nav2_no_map_params.yaml`. The shim
    /// and RPP share the `FollowPath.` namespace, so RPP also reads the
    /// shim's `rotate_to_heading_angular_vel` and `max_angular_accel`.
    /// (`min_distance_to_obstacle` is not a 1.3.13 parameter and is ignored
    /// upstream too.)
    pub fn mower() -> Self {
        Self {
            desired_linear_vel: 0.26,
            lookahead_dist: 0.25,
            min_lookahead_dist: 0.2,
            max_lookahead_dist: 0.35,
            lookahead_time: 1.5,
            rotate_to_heading_angular_vel: 0.6,
            transform_tolerance: 0.1,
            use_velocity_scaled_lookahead_dist: false,
            min_approach_linear_velocity: 0.03,
            approach_velocity_scaling_dist: 0.4,
            max_allowed_time_to_collision_up_to_carrot: 1.0,
            use_regulated_linear_velocity_scaling: true,
            use_cost_regulated_linear_velocity_scaling: false,
            cost_scaling_dist: 0.3,
            cost_scaling_gain: 1.0,
            regulated_linear_scaling_min_radius: 0.6,
            regulated_linear_scaling_min_speed: 0.08,
            use_fixed_curvature_lookahead: false,
            curvature_lookahead_dist: 0.25,
            use_rotate_to_heading: false,
            rotate_to_heading_min_angle: 1.57,
            max_angular_accel: 1.2,
            allow_reversing: false,
            max_robot_pose_search_dist: 10.0,
            use_collision_detection: true,
            stateful: true,
            controller_frequency: 20.0,
            ..Self::default()
        }
    }
}

impl Default for RppParams {
    /// Upstream's defaults on this robot's 3 x 3 m local costmap
    /// (`max_robot_pose_search_dist` = 2.975 / 2).
    fn default() -> Self {
        Self::defaults_for_costmap(RollingWindow::local_costmap().costmap().size_in_meters_x())
    }
}

// --- std helpers ---------------------------------------------------------------

/// `std::min(a, b)`: `b < a ? b : a`.
fn std_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// `std::max(a, b)`: `a < b ? b : a`.
fn std_max(a: f64, b: f64) -> f64 {
    if a < b {
        b
    } else {
        a
    }
}

/// `std::clamp(v, lo, hi)`; unlike `f64::clamp` it does not panic on
/// `lo > hi` (e.g. a negative speed limit).
pub(crate) fn std_clamp(v: f64, lo: f64, hi: f64) -> f64 {
    if v < lo {
        lo
    } else if hi < v {
        hi
    } else {
        v
    }
}

// --- regulation_functions.hpp ------------------------------------------------

/// `log(253.0f)` in `costConstraint`: the float overload, folded by GCC to
/// this double (0x40162230e0000000, read from the 1.3.13 aarch64 binary).
const LOG_253_F32: f64 = 5.533389568328857;

/// `heuristics::curvatureConstraint`.
pub fn curvature_constraint(raw_linear_vel: f64, curvature: f64, min_radius: f64) -> f64 {
    let radius = (1.0 / curvature).abs();
    if radius < min_radius {
        raw_linear_vel * (1.0 - ((radius - min_radius).abs() / min_radius))
    } else {
        raw_linear_vel
    }
}

/// `heuristics::costConstraint`.
pub fn cost_constraint(raw_linear_vel: f64, pose_cost: f64, inscribed_radius: f64, params: &RppParams) -> f64 {
    if pose_cost != NO_INFORMATION as f64 && pose_cost != FREE_SPACE as f64 {
        let min_distance_to_obstacle = (params.inflation_cost_scaling_factor * inscribed_radius - pose_cost.ln()
            + LOG_253_F32)
            / params.inflation_cost_scaling_factor;
        if min_distance_to_obstacle < params.cost_scaling_dist {
            return raw_linear_vel * (params.cost_scaling_gain * min_distance_to_obstacle / params.cost_scaling_dist);
        }
    }
    raw_linear_vel
}

/// `heuristics::approachVelocityScalingFactor`: the integrated length of the
/// transformed plan (from its first pose, not from the robot) decides, the
/// straight distance to the last pose scales.
pub fn approach_velocity_scaling_factor(transformed_path: &[Pose2D], approach_velocity_scaling_dist: f64) -> f64 {
    let remaining_distance = calculate_path_length(transformed_path);
    if remaining_distance < approach_velocity_scaling_dist {
        let last = transformed_path[transformed_path.len() - 1];
        last.x.hypot(last.y) / approach_velocity_scaling_dist
    } else {
        1.0
    }
}

/// `heuristics::approachVelocityConstraint`.
pub fn approach_velocity_constraint(
    constrained_linear_vel: f64,
    path: &[Pose2D],
    min_approach_velocity: f64,
    approach_velocity_scaling_dist: f64,
) -> f64 {
    let velocity_scaling = approach_velocity_scaling_factor(path, approach_velocity_scaling_dist);
    let mut approach_vel = constrained_linear_vel * velocity_scaling;
    if approach_vel < min_approach_velocity {
        approach_vel = min_approach_velocity;
    }
    std_min(constrained_linear_vel, approach_vel)
}

// --- geometry of the lookahead ---------------------------------------------------

/// `calculateCurvature`: 1 / R of the arc from the robot (origin, facing +x)
/// through the point; 0 within ~3 cm.
pub fn calculate_curvature(x: f64, y: f64) -> f64 {
    let carrot_dist2 = x * x + y * y;
    if carrot_dist2 > 0.001 {
        2.0 * y / carrot_dist2
    } else {
        0.0
    }
}

/// `circleSegmentIntersection`: the point of segment p1 -> p2 at distance `r`
/// from the origin (p1 inside the circle, p2 outside).
pub fn circle_segment_intersection(p1: (f64, f64), p2: (f64, f64), r: f64) -> (f64, f64) {
    let (x1, y1) = p1;
    let (x2, y2) = p2;
    let dx = x2 - x1;
    let dy = y2 - y1;
    let dr2 = dx * dx + dy * dy;
    let d = x1 * y2 - x2 * y1;

    // Augmentation to only return the point within the segment.
    let d1 = x1 * x1 + y1 * y1;
    let d2 = x2 * x2 + y2 * y2;
    let dd = d2 - d1;

    let sqrt_term = (r * r * dr2 - d * d).sqrt();
    let sign = 1.0f64.copysign(dd);
    ((d * dy + sign * dx * sqrt_term) / dr2, (-d * dx + sign * dy * sqrt_term) / dr2)
}

/// `getLookAheadPoint`: on the segment that crosses `lookahead_dist`, else
/// the first pose (when it is already beyond), else the last pose, or the
/// last segment projected beyond the end with `interpolate_after_goal`.
/// Returns the position in the robot base frame.
pub fn get_look_ahead_point(
    lookahead_dist: f64,
    transformed_plan: &[Pose2D],
    interpolate_after_goal: bool,
) -> (f64, f64) {
    let goal = transformed_plan.iter().position(|ps| ps.x.hypot(ps.y) >= lookahead_dist);
    match goal {
        None => {
            let last = transformed_plan[transformed_plan.len() - 1];
            // Upstream reads before begin() for a one-pose plan here
            // (undefined behaviour); this returns the last pose instead.
            if interpolate_after_goal && transformed_plan.len() >= 2 {
                let prev = transformed_plan[transformed_plan.len() - 2];
                let end_path_orientation = (last.y - prev.y).atan2(last.x - prev.x);
                // Project the last segment out to be sure it reaches beyond
                // the lookahead distance.
                let projected = (
                    last.x + end_path_orientation.cos() * lookahead_dist,
                    last.y + end_path_orientation.sin() * lookahead_dist,
                );
                circle_segment_intersection((last.x, last.y), projected, lookahead_dist)
            } else {
                (last.x, last.y)
            }
        }
        Some(0) => (transformed_plan[0].x, transformed_plan[0].y),
        Some(i) => {
            let prev = transformed_plan[i - 1];
            let goal = transformed_plan[i];
            circle_segment_intersection((prev.x, prev.y), (goal.x, goal.y), lookahead_dist)
        }
    }
}

/// `findVelocitySignChange`: the distance to the first cusp (a direction
/// reversal or an in-place rotation) of the transformed plan, else
/// `f64::MAX`. Orientations are compared as yaws (upstream compares
/// quaternions, which only differs for q vs -q).
pub fn find_velocity_sign_change(transformed_plan: &[Pose2D]) -> f64 {
    for pose_id in 1..transformed_plan.len().saturating_sub(1) {
        let (a, o, b) = (transformed_plan[pose_id - 1], transformed_plan[pose_id], transformed_plan[pose_id + 1]);
        let oa_x = o.x - a.x;
        let oa_y = o.y - a.y;
        let ab_x = b.x - o.x;
        let ab_y = b.y - o.y;

        let dot_prod = (oa_x * ab_x) + (oa_y * ab_y);
        if dot_prod < 0.0 {
            return o.x.hypot(o.y);
        }

        if (oa_x.hypot(oa_y) == 0.0 && a.yaw != o.yaw) || (ab_x.hypot(ab_y) == 0.0 && o.yaw != b.yaw) {
            // overlapping poses that rotate in place
            return o.x.hypot(o.y);
        }
    }
    f64::MAX
}

// --- collision_checker.cpp -------------------------------------------------------

/// `CollisionChecker::inCollision`: a centre off the costmap is *not* a
/// collision (upstream warns, throttled, and goes on), a footprint vertex off
/// it is (`footprintCost` makes it lethal).
pub fn in_collision(costmap: &Costmap2D, context: &CostmapContext, x: f64, y: f64, theta: f64) -> bool {
    if costmap.world_to_map(x, y).is_none() {
        return false;
    }
    let footprint_cost =
        FootprintCollisionChecker::new(costmap).footprint_cost_at_pose(x, y, theta, &context.footprint);
    if footprint_cost == NO_INFORMATION as f64 && context.tracking_unknown {
        return false;
    }
    footprint_cost >= LETHAL_OBSTACLE as f64
}

/// `CollisionChecker::isCollisionImminent`: the current pose, then the
/// command integrated forward in one-cell steps until
/// `max_allowed_time_to_collision_up_to_carrot` or the carrot distance.
pub fn is_collision_imminent(
    costmap: &Costmap2D,
    context: &CostmapContext,
    max_allowed_time_to_collision_up_to_carrot: f64,
    robot_pose: &Pose2D,
    linear_vel: f64,
    angular_vel: f64,
    carrot_dist: f64,
) -> bool {
    // robot_pose is in odom, the carrot in the robot base frame.
    if in_collision(costmap, context, robot_pose.x, robot_pose.y, robot_pose.yaw) {
        return true;
    }

    let projection_time = if linear_vel.abs() < 0.01 && angular_vel.abs() > 0.01 {
        // Rotating in place: the time for the circumscribed radius to sweep
        // into the next cell.
        let max_radius = context.circumscribed_radius;
        2.0 * ((costmap.resolution() / 2.0) / max_radius).sin() / angular_vel.abs()
    } else {
        costmap.resolution() / linear_vel.abs()
    };

    let mut curr = *robot_pose;
    let mut i = 1;
    while i as f64 * projection_time < max_allowed_time_to_collision_up_to_carrot {
        i += 1;
        curr.x += projection_time * (linear_vel * curr.yaw.cos());
        curr.y += projection_time * (linear_vel * curr.yaw.sin());
        curr.yaw += projection_time * angular_vel;

        // Past the carrot the command means nothing any more.
        if (curr.x - robot_pose.x).hypot(curr.y - robot_pose.y) > carrot_dist {
            break;
        }
        if in_collision(costmap, context, curr.x, curr.y, curr.yaw) {
            return true;
        }
    }
    false
}

/// `CollisionChecker::costAtPose`: the cell under (x, y).
pub fn cost_at_pose(costmap: &Costmap2D, x: f64, y: f64) -> Result<f64, ControllerError> {
    match costmap.world_to_map(x, y) {
        Some((mx, my)) => Ok(costmap.cost(mx, my) as f64),
        None => Err(ControllerError::Controller(
            "RegulatedPurePursuitController: Dimensions of the costmap are too small to encapsulate the robot \
             footprint at current speeds!",
        )),
    }
}

// --- the controller ---------------------------------------------------------------

/// `RegulatedPurePursuitController`.
#[derive(Debug, Clone)]
pub struct RegulatedPurePursuit {
    params: RppParams,
    base_desired_linear_vel: f64,
    context: CostmapContext,
    path_handler: PathHandler,
    control_duration: f64,
    goal_dist_tol: f64,
    cancelling: bool,
    finished_cancelling: bool,
    has_reached_xy_tolerance: bool,
    is_rotating_to_heading: bool,
    transformed_plan: Path,
    carrot: Option<(f64, f64)>,
}

impl RegulatedPurePursuit {
    /// `configure` (with `ParameterHandler`'s derived values).
    pub fn new(params: RppParams, context: CostmapContext) -> Self {
        let mut params = params;
        if params.max_robot_pose_search_dist < 0.0 {
            // "Max robot search distance is negative": search the whole path.
            params.max_robot_pose_search_dist = f64::MAX;
        }
        if !params.use_fixed_curvature_lookahead && params.interpolate_curvature_after_goal {
            params.interpolate_curvature_after_goal = false;
        }
        if params.inflation_cost_scaling_factor <= 0.0 {
            params.use_cost_regulated_linear_velocity_scaling = false;
        }
        Self {
            base_desired_linear_vel: params.desired_linear_vel,
            control_duration: 1.0 / params.controller_frequency,
            params,
            context,
            path_handler: PathHandler::new(),
            goal_dist_tol: 0.25, // reasonable default before the first update
            cancelling: false,
            finished_cancelling: false,
            has_reached_xy_tolerance: false,
            is_rotating_to_heading: false,
            transformed_plan: Path::new(),
            carrot: None,
        }
    }

    /// This robot's `FollowPath` primary controller on its local costmap.
    pub fn mower() -> Self {
        Self::new(RppParams::mower(), CostmapContext::local_costmap())
    }

    /// The parameters in effect (after the derived values and any speed limit).
    pub fn params(&self) -> &RppParams {
        &self.params
    }

    /// The global plan as pruned so far, in the plan frame.
    pub fn plan(&self) -> &[Pose2D] {
        self.path_handler.plan()
    }

    /// The last transformed plan (`received_global_plan`), robot base frame.
    pub fn transformed_plan(&self) -> &[Pose2D] {
        &self.transformed_plan
    }

    /// The last lookahead point (`lookahead_point`), robot base frame.
    pub fn carrot(&self) -> Option<(f64, f64)> {
        self.carrot
    }

    /// `is_rotating_to_heading` as last published.
    pub fn is_rotating_to_heading(&self) -> bool {
        self.is_rotating_to_heading
    }

    /// `getLookAheadDistance`.
    fn get_look_ahead_distance(&self, speed: &Twist2D) -> f64 {
        let mut lookahead_dist = self.params.lookahead_dist;
        if self.params.use_velocity_scaled_lookahead_dist {
            lookahead_dist = speed.x.abs() * self.params.lookahead_time;
            lookahead_dist = std_clamp(lookahead_dist, self.params.min_lookahead_dist, self.params.max_lookahead_dist);
        }
        lookahead_dist
    }

    /// `shouldRotateToPath`: (rotate?, angle_to_path).
    fn should_rotate_to_path(&self, carrot: (f64, f64), x_vel_sign: f64) -> (bool, f64) {
        let mut angle_to_path = carrot.1.atan2(carrot.0);
        if x_vel_sign < 0.0 {
            angle_to_path = normalize_angle(angle_to_path + std::f64::consts::PI);
        }
        (
            self.params.use_rotate_to_heading && angle_to_path.abs() > self.params.rotate_to_heading_min_angle,
            angle_to_path,
        )
    }

    /// `shouldRotateToGoalHeading`; `stateful` latches the xy tolerance.
    fn should_rotate_to_goal_heading(&mut self, carrot: (f64, f64)) -> bool {
        if !self.params.use_rotate_to_heading {
            return false;
        }
        let dist_to_goal = carrot.0.hypot(carrot.1);
        if self.params.stateful {
            if !self.has_reached_xy_tolerance && dist_to_goal < self.goal_dist_tol {
                self.has_reached_xy_tolerance = true;
            }
            return self.has_reached_xy_tolerance;
        }
        dist_to_goal < self.goal_dist_tol
    }

    /// `rotateToHeading`: (linear, angular).
    fn rotate_to_heading(&self, angle_to_path: f64, curr_speed: &Twist2D) -> (f64, f64) {
        let sign = if angle_to_path > 0.0 { 1.0 } else { -1.0 };
        let mut angular_vel = sign * self.params.rotate_to_heading_angular_vel;

        let dt = self.control_duration;
        let min_feasible_angular_speed = curr_speed.theta - self.params.max_angular_accel * dt;
        let max_feasible_angular_speed = curr_speed.theta + self.params.max_angular_accel * dt;
        angular_vel = std_clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);

        // Slow down so as not to overshoot.
        let max_vel_to_stop = (2.0 * self.params.max_angular_accel * angle_to_path.abs()).sqrt();
        if angular_vel.abs() > max_vel_to_stop {
            angular_vel = sign * max_vel_to_stop;
        }
        (0.0, angular_vel)
    }

    /// `applyConstraints`.
    fn apply_constraints(&self, curvature: f64, pose_cost: f64, path: &[Pose2D], linear_vel: f64, sign: f64) -> f64 {
        let p = &self.params;
        let mut curvature_vel = linear_vel;
        let mut cost_vel = linear_vel;

        if p.use_regulated_linear_velocity_scaling {
            curvature_vel = curvature_constraint(linear_vel, curvature, p.regulated_linear_scaling_min_radius);
        }
        if p.use_cost_regulated_linear_velocity_scaling {
            cost_vel = cost_constraint(linear_vel, pose_cost, self.context.inscribed_radius, p);
        }

        // The lower of the two, but above the minimum speed.
        let mut linear_vel = std_min(cost_vel, curvature_vel);
        linear_vel = std_max(linear_vel, p.regulated_linear_scaling_min_speed);

        // Slow down on approach to the final goal pose.
        linear_vel = approach_velocity_constraint(
            linear_vel,
            path,
            p.min_approach_linear_velocity,
            p.approach_velocity_scaling_dist,
        );

        linear_vel = std_clamp(linear_vel.abs(), 0.0, p.desired_linear_vel);
        sign * linear_vel
    }
}

impl Controller for RegulatedPurePursuit {
    /// `setPlan`.
    fn set_plan(&mut self, path: &[Pose2D]) {
        self.has_reached_xy_tolerance = false;
        self.path_handler.set_plan(path);
    }

    /// `computeVelocityCommands`.
    fn compute_velocity_commands(&mut self, input: &ControlInput) -> Result<Twist2D, ControllerError> {
        let p = self.params;
        let pose = input.pose;
        let speed = input.speed;

        // getTolerances
        self.goal_dist_tol = input.xy_goal_tolerance;

        let transformed_plan: Path = self.path_handler.transform_global_plan(
            &pose,
            &input.costmap_from_plan,
            input.costmap,
            p.max_robot_pose_search_dist,
            p.interpolate_curvature_after_goal,
        )?;
        self.transformed_plan.clone_from(&transformed_plan);

        let mut lookahead_dist = self.get_look_ahead_distance(&speed);
        let mut curv_lookahead_dist = p.curvature_lookahead_dist;

        if p.allow_reversing {
            // Cusp check: never look beyond the cusp.
            let dist_to_cusp = find_velocity_sign_change(&transformed_plan);
            if dist_to_cusp < lookahead_dist {
                lookahead_dist = dist_to_cusp;
            }
            if dist_to_cusp < curv_lookahead_dist {
                curv_lookahead_dist = dist_to_cusp;
            }
        }

        let carrot = get_look_ahead_point(lookahead_dist, &transformed_plan, false);
        self.carrot = Some(carrot);
        let mut rotate_to_path_carrot = carrot;

        let lookahead_curvature = calculate_curvature(carrot.0, carrot.1);
        let mut regulation_curvature = lookahead_curvature;
        if p.use_fixed_curvature_lookahead {
            let curvature_lookahead =
                get_look_ahead_point(curv_lookahead_dist, &transformed_plan, p.interpolate_curvature_after_goal);
            rotate_to_path_carrot = curvature_lookahead;
            regulation_curvature = calculate_curvature(curvature_lookahead.0, curvature_lookahead.1);
        }

        let mut x_vel_sign = 1.0;
        if p.allow_reversing {
            x_vel_sign = if carrot.0 >= 0.0 { 1.0 } else { -1.0 };
        }

        let mut linear_vel = p.desired_linear_vel;
        let angular_vel;

        let (rotate_to_path, angle_to_heading) = self.should_rotate_to_path(rotate_to_path_carrot, x_vel_sign);
        if self.should_rotate_to_goal_heading(carrot) {
            self.is_rotating_to_heading = true;
            let angle_to_goal = transformed_plan[transformed_plan.len() - 1].yaw;
            (linear_vel, angular_vel) = self.rotate_to_heading(angle_to_goal, &speed);
        } else if rotate_to_path {
            self.is_rotating_to_heading = true;
            (linear_vel, angular_vel) = self.rotate_to_heading(angle_to_heading, &speed);
        } else {
            self.is_rotating_to_heading = false;
            // costAtPose is evaluated (and throws off the costmap) even with
            // cost regulation off.
            let pose_cost = cost_at_pose(input.costmap, pose.x, pose.y)?;
            linear_vel =
                self.apply_constraints(regulation_curvature, pose_cost, &transformed_plan, linear_vel, x_vel_sign);

            if self.cancelling {
                let dt = self.control_duration;
                linear_vel = speed.x - x_vel_sign * dt * p.cancel_deceleration;
                if x_vel_sign > 0.0 {
                    if linear_vel <= 0.0 {
                        linear_vel = 0.0;
                        self.finished_cancelling = true;
                    }
                } else if linear_vel >= 0.0 {
                    linear_vel = 0.0;
                    self.finished_cancelling = true;
                }
            }

            // Curvature applied after the linear velocity is constrained.
            angular_vel = linear_vel * regulation_curvature;
        }

        // Collision checking on this velocity heading.
        let carrot_dist = carrot.0.hypot(carrot.1);
        if p.use_collision_detection
            && is_collision_imminent(
                input.costmap,
                &self.context,
                p.max_allowed_time_to_collision_up_to_carrot,
                &pose,
                linear_vel,
                angular_vel,
                carrot_dist,
            )
        {
            return Err(ControllerError::NoValidControl("RegulatedPurePursuitController detected collision ahead!"));
        }

        Ok(Twist2D::new(linear_vel, 0.0, angular_vel))
    }

    /// `setSpeedLimit`.
    fn set_speed_limit(&mut self, speed_limit: f64, percentage: bool) {
        if speed_limit == NO_SPEED_LIMIT {
            self.params.desired_linear_vel = self.base_desired_linear_vel;
        } else if percentage {
            self.params.desired_linear_vel = self.base_desired_linear_vel * speed_limit / 100.0;
        } else {
            self.params.desired_linear_vel = speed_limit;
        }
    }

    /// `reset`.
    fn reset(&mut self) {
        self.cancelling = false;
        self.finished_cancelling = false;
        self.has_reached_xy_tolerance = false;
    }

    /// `cancel`: with `use_cancel_deceleration` the robot decelerates at
    /// `cancel_deceleration` first.
    fn cancel(&mut self) -> bool {
        if !self.params.use_cancel_deceleration {
            return true;
        }
        self.cancelling = true;
        self.finished_cancelling
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn yaml_overrides_and_shared_keys() {
        let p = RppParams::mower();
        assert_eq!(p.desired_linear_vel, 0.26);
        assert_eq!(p.max_angular_accel, 1.2);
        assert_eq!(p.rotate_to_heading_angular_vel, 0.6);
        assert_eq!(p.inflation_cost_scaling_factor, 3.0);
        assert!(!p.use_rotate_to_heading && !p.use_cost_regulated_linear_velocity_scaling);
        assert_eq!(RppParams::default().max_robot_pose_search_dist, 2.975 / 2.0);
    }

    #[test]
    fn derived_parameters() {
        let rpp = RegulatedPurePursuit::new(
            RppParams {
                max_robot_pose_search_dist: -1.0,
                interpolate_curvature_after_goal: true,
                inflation_cost_scaling_factor: 0.0,
                ..RppParams::default()
            },
            CostmapContext::local_costmap(),
        );
        assert_eq!(rpp.params().max_robot_pose_search_dist, f64::MAX);
        assert!(!rpp.params().interpolate_curvature_after_goal);
        assert!(!rpp.params().use_cost_regulated_linear_velocity_scaling);
    }

    #[test]
    fn log_253_is_the_float_one() {
        assert_eq!(LOG_253_F32.to_bits(), 0x40162230e0000000);
        assert_eq!((253.0f32).ln() as f64, LOG_253_F32);
    }

    #[test]
    fn lookahead_on_a_straight_line() {
        let plan: Vec<Pose2D> = (0..10).map(|i| Pose2D::new(-0.05 + 0.1 * i as f64, 0.02, 0.0)).collect();
        let (x, y) = get_look_ahead_point(0.25, &plan, false);
        assert!((x.hypot(y) - 0.25).abs() < 1e-12 && (y - 0.02).abs() < 1e-12 && x > 0.0);
        // nothing far enough: the last pose
        assert_eq!(get_look_ahead_point(2.0, &plan, false), (plan[9].x, plan[9].y));
        // interpolated beyond the end
        let (x, y) = get_look_ahead_point(2.0, &plan, true);
        assert!((x.hypot(y) - 2.0).abs() < 1e-12);
    }

    #[test]
    fn cusps_and_in_place_rotations() {
        let p = |x: f64, y: f64, yaw: f64| Pose2D::new(x, y, yaw);
        let straight = [p(0.0, 0.0, 0.0), p(0.1, 0.0, 0.0), p(0.2, 0.0, 0.0)];
        assert_eq!(find_velocity_sign_change(&straight), f64::MAX);
        let back = [p(0.0, 0.0, 0.0), p(0.1, 0.0, 0.0), p(0.2, 0.0, 0.0), p(0.1, 0.05, 0.0)];
        assert_eq!(find_velocity_sign_change(&back), 0.2);
        let spin = [p(0.0, 0.0, 0.0), p(0.1, 0.0, 0.0), p(0.1, 0.0, 0.5), p(0.2, 0.0, 0.5)];
        assert_eq!(find_velocity_sign_change(&spin), 0.1);
        // a duplicated pose is not a rotation
        let dup = [p(0.0, 0.0, 0.0), p(0.1, 0.0, 0.0), p(0.1, 0.0, 0.0), p(0.2, 0.0, 0.0)];
        assert_eq!(find_velocity_sign_change(&dup), f64::MAX);
        assert_eq!(find_velocity_sign_change(&straight[..1]), f64::MAX);
    }

    #[test]
    fn collision_edges_of_the_window() {
        let ctx = CostmapContext::local_costmap();
        let mut c = Costmap2D::new(60, 60, 0.05, -1.5, -1.5, 0);
        // centre off the costmap: not a collision; footprint over the edge: lethal
        assert!(!in_collision(&c, &ctx, 2.0, 0.0, 0.0));
        assert!(in_collision(&c, &ctx, 1.2, 0.0, 0.0));
        assert!(!in_collision(&c, &ctx, 0.0, 0.0, 0.0));
        assert!(cost_at_pose(&c, 2.0, 0.0).is_err());
        // a lethal cell 0.4 m ahead: hit by the projection, not by the pose
        c.set_cost(38, 30, LETHAL_OBSTACLE);
        assert!(!in_collision(&c, &ctx, -0.2, 0.0, 0.0));
        assert!(is_collision_imminent(&c, &ctx, 1.0, &Pose2D::new(-0.2, 0.0, 0.0), 0.26, 0.0, 0.25));
        // at 0.03 m/s one projection step is 1.67 s: only the pose is checked
        assert!(!is_collision_imminent(&c, &ctx, 1.0, &Pose2D::new(-0.2, 0.0, 0.0), 0.03, 0.0, 0.25));
    }

    #[test]
    fn cancel_decelerates_to_zero() {
        let costmap = Costmap2D::new(60, 60, 0.05, -1.5, -1.5, 0);
        let mut rpp = RegulatedPurePursuit::new(
            RppParams { use_cancel_deceleration: true, ..RppParams::mower() },
            CostmapContext::local_costmap(),
        );
        rpp.set_plan(&(0..20).map(|i| Pose2D::new(0.1 * i as f64, 0.0, 0.0)).collect::<Vec<_>>());
        let input = |v: f64| ControlInput {
            pose: Pose2D::new(0.0, 0.0, 0.0),
            speed: Twist2D::new(v, 0.0, 0.0),
            xy_goal_tolerance: 0.05,
            costmap_from_plan: Transform2D::IDENTITY,
            costmap: &costmap,
        };
        assert!(!rpp.cancel());
        let cmd = rpp.compute_velocity_commands(&input(0.2)).unwrap();
        assert!((cmd.x - (0.2 - 0.05 * 3.2)).abs() < 1e-15);
        assert!(!rpp.cancel());
        assert_eq!(rpp.compute_velocity_commands(&input(cmd.x)).unwrap().x, 0.0);
        assert!(rpp.cancel());
        rpp.reset();
        assert!(!rpp.cancel());
    }

    #[test]
    fn speed_limit_roundtrip() {
        let mut rpp = RegulatedPurePursuit::mower();
        rpp.set_speed_limit(50.0, true);
        assert_eq!(rpp.params().desired_linear_vel, 0.13);
        rpp.set_speed_limit(0.1, false);
        assert_eq!(rpp.params().desired_linear_vel, 0.1);
        rpp.set_speed_limit(NO_SPEED_LIMIT, true);
        assert_eq!(rpp.params().desired_linear_vel, 0.26);
    }
}
