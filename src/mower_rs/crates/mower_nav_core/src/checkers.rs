//! The `nav2_controller` checker plugins this robot loads (D3):
//! `SimpleGoalChecker` (as `general_goal_checker`) and `SimpleProgressChecker`
//! (as `progress_checker`), with `controller_server`'s settings from
//! `nav2_no_map_params.yaml`.
//!
//! With `xy_goal_tolerance: 0.05`, `yaw_goal_tolerance: 3.14` and `stateful`,
//! the goal counts as reached on the first check within 5 cm, unless the
//! robot then faces within 0.0016 rad of exactly away from the goal heading;
//! in that case the following checks look at the yaw only. The
//! `symmetric_yaw_tolerance` and non-stateful branches are not reached here;
//! they are ported and checked against the oracle with other parameters.
//!
//! Upstream: `nav2_controller/plugins/simple_goal_checker.cpp`,
//! `nav2_controller/plugins/simple_progress_checker.cpp`.
//!
//! Not ported: `dynamicParametersCallback` (nothing changes these at run
//! time; build a new checker instead).

use crate::geometry::{normalize_angle, quaternion_from_yaw, shortest_angular_distance, Pose2D, Twist2D};
use std::f64::consts::PI;

/// `SimpleGoalChecker`'s parameters (`<plugin>.xy_goal_tolerance` etc.).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimpleGoalCheckerParams {
    pub xy_goal_tolerance: f64,
    pub yaw_goal_tolerance: f64,
    pub stateful: bool,
    pub symmetric_yaw_tolerance: bool,
}

impl Default for SimpleGoalCheckerParams {
    /// The `declare_parameter_if_not_declared` defaults.
    fn default() -> Self {
        Self { xy_goal_tolerance: 0.25, yaw_goal_tolerance: 0.25, stateful: true, symmetric_yaw_tolerance: false }
    }
}

impl SimpleGoalCheckerParams {
    /// `controller_server.general_goal_checker` in `nav2_no_map_params.yaml`.
    #[allow(clippy::approx_constant)] // the yaml's 3.14, not pi
    pub fn mower() -> Self {
        Self { xy_goal_tolerance: 0.05, yaw_goal_tolerance: 3.14, stateful: true, ..Self::default() }
    }
}

/// What `SimpleGoalChecker::getTolerances` fills in (it always returns true).
/// The fields it marks invalid are `f64::MIN`
/// (`std::numeric_limits<double>::lowest()`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GoalTolerances {
    /// `pose_tolerance.position` (x, y, z): the xy tolerance twice, z invalid.
    pub position: [f64; 3],
    /// `pose_tolerance.orientation` (x, y, z, w):
    /// `orientationAroundZAxis(yaw_goal_tolerance)`.
    pub orientation: [f64; 4],
    /// `vel_tolerance.linear`: all invalid.
    pub linear: [f64; 3],
    /// `vel_tolerance.angular`: all invalid.
    pub angular: [f64; 3],
}

/// `nav2_controller::SimpleGoalChecker`.
///
/// Stateful (the default, and this robot): once the robot has been inside
/// the xy tolerance, only the yaw is checked until [`reset`](Self::reset),
/// however far it drifts afterwards.
#[derive(Debug, Clone)]
pub struct SimpleGoalChecker {
    params: SimpleGoalCheckerParams,
    xy_goal_tolerance_sq: f64,
    check_xy: bool,
}

impl SimpleGoalChecker {
    /// The constructor followed by `initialize`.
    pub fn new(params: SimpleGoalCheckerParams) -> Self {
        Self { params, xy_goal_tolerance_sq: params.xy_goal_tolerance * params.xy_goal_tolerance, check_xy: true }
    }

    pub fn params(&self) -> &SimpleGoalCheckerParams {
        &self.params
    }

    /// False while the stateful latch has dropped the xy check.
    pub fn checks_xy(&self) -> bool {
        self.check_xy
    }

    /// `SimpleGoalChecker::reset`: re-arm the xy check.
    pub fn reset(&mut self) {
        self.check_xy = true;
    }

    /// `SimpleGoalChecker::isGoalReached`. The yaws are what `tf2::getYaw`
    /// reads from the poses' quaternions; `velocity` is ignored, as upstream.
    pub fn is_goal_reached(&mut self, query: &Pose2D, goal: &Pose2D, _velocity: &Twist2D) -> bool {
        if self.check_xy {
            let dx = query.x - goal.x;
            let dy = query.y - goal.y;
            // The aarch64 build fuses `dx * dx + dy * dy` into
            // fmadd(dx, dx, dy * dy); doing the same keeps the boundary exact.
            if dx.mul_add(dx, dy * dy) > self.xy_goal_tolerance_sq {
                return false;
            }
            if self.params.stateful {
                self.check_xy = false;
            }
        }
        let tol = self.params.yaw_goal_tolerance;
        if self.params.symmetric_yaw_tolerance {
            let forward = shortest_angular_distance(query.yaw, goal.yaw);
            let backward = shortest_angular_distance(query.yaw, normalize_angle(goal.yaw + PI));
            forward.abs() <= tol || backward.abs() <= tol
        } else {
            shortest_angular_distance(query.yaw, goal.yaw).abs() <= tol
        }
    }

    /// `SimpleGoalChecker::getTolerances`.
    pub fn get_tolerances(&self) -> GoalTolerances {
        let invalid = f64::MIN;
        let xy = self.params.xy_goal_tolerance;
        let (qx, qy, qz, qw) = quaternion_from_yaw(self.params.yaw_goal_tolerance);
        GoalTolerances {
            position: [xy, xy, invalid],
            orientation: [qx, qy, qz, qw],
            linear: [invalid; 3],
            angular: [invalid; 3],
        }
    }
}

/// `SimpleProgressChecker`'s parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimpleProgressCheckerParams {
    /// metres
    pub required_movement_radius: f64,
    /// seconds
    pub movement_time_allowance: f64,
}

impl Default for SimpleProgressCheckerParams {
    fn default() -> Self {
        Self { required_movement_radius: 0.5, movement_time_allowance: 10.0 }
    }
}

impl SimpleProgressCheckerParams {
    /// `controller_server.progress_checker` in `nav2_no_map_params.yaml`.
    pub fn mower() -> Self {
        Self { required_movement_radius: 0.1, movement_time_allowance: 20.0 }
    }
}

/// `nav2_controller::SimpleProgressChecker`. Times are the controller node's
/// clock (`clock_->now()`) in nanoseconds.
#[derive(Debug, Clone)]
pub struct SimpleProgressChecker {
    radius: f64,
    time_allowance_ns: i64,
    baseline_x: f64,
    baseline_y: f64,
    baseline_time_ns: i64,
    baseline_pose_set: bool,
}

impl SimpleProgressChecker {
    pub fn new(params: SimpleProgressCheckerParams) -> Self {
        Self {
            radius: params.required_movement_radius,
            time_allowance_ns: duration_from_seconds(params.movement_time_allowance),
            baseline_x: 0.0,
            baseline_y: 0.0,
            baseline_time_ns: 0,
            baseline_pose_set: false,
        }
    }

    /// The baseline (x, y, time) the next check measures from, if set.
    pub fn baseline(&self) -> Option<(f64, f64, i64)> {
        self.baseline_pose_set.then_some((self.baseline_x, self.baseline_y, self.baseline_time_ns))
    }

    /// `SimpleProgressChecker::check`: false once the robot has stayed within
    /// `required_movement_radius` (strictly: not moved further than it) of the
    /// baseline for more than `movement_time_allowance`. The first check
    /// after construction or [`reset`](Self::reset), and every check that has
    /// moved far enough, re-baselines at (`pose`, `now_ns`) and passes. Only
    /// x and y are used.
    pub fn check(&mut self, pose: &Pose2D, now_ns: i64) -> bool {
        if !self.baseline_pose_set || (pose.x - self.baseline_x).hypot(pose.y - self.baseline_y) > self.radius {
            self.baseline_x = pose.x;
            self.baseline_y = pose.y;
            self.baseline_time_ns = now_ns;
            self.baseline_pose_set = true;
            return true;
        }
        now_ns - self.baseline_time_ns <= self.time_allowance_ns
    }

    /// `SimpleProgressChecker::reset`: the next check re-baselines.
    pub fn reset(&mut self) {
        self.baseline_pose_set = false;
    }
}

/// `rclcpp::Duration::from_seconds`: `static_cast<int64_t>(s * 1e9)`.
fn duration_from_seconds(seconds: f64) -> i64 {
    (seconds * 1e9) as i64
}
