//! `nav2_navfn_planner::NavfnPlanner`, the `GridBased` plugin of
//! `planner_server`, on the global costmap ([`crate::costmap::static_costmap`]
//! with `track_unknown_space: true`), and the error codes `planner_server`
//! turns its exceptions into.
//!
//! `nav2_no_map_params.yaml` sets `tolerance: 0.0`, `use_astar: false`,
//! `allow_unknown: true` ([`NavfnParams::mower`]). With those:
//!
//! * a goal is rejected (`GoalOccupied`) only if its cell is LETHAL; a goal
//!   in unknown space is planned to, through unknown cells at NavFn cost 253;
//! * the cell checks use `Costmap2D::worldToMap` (truncation), but the plan
//!   uses the planner's own `worldToMap`, which *rounds*: NavFn's grid points
//!   are the cell corners, `mapToWorld` is `origin + m * resolution`, and the
//!   cell planned from or to can be the next one up or right;
//! * the rounded start cell is set FREE in the costmap the planner is given
//!   (`clearRobotCell`), and stays so: on the robot that is the global
//!   costmap's master grid, which the static layer only rewrites when a new
//!   map arrives;
//! * every pose but the last has the identity orientation; the last is the
//!   goal pose itself.
//!
//! Upstream: `nav2_navfn_planner/src/navfn_planner.cpp`,
//! `nav2_core/planner_exceptions.hpp`, `nav2_planner/src/planner_server.cpp`.

use crate::costmap::{Costmap2D, FREE_SPACE, LETHAL_OBSTACLE};
use crate::geometry::Pose2D;
use crate::navfn::{Cancelled, NavFn, POT_HIGH};

/// The plugin's parameters (`<name>.tolerance` etc.). `Default` is
/// upstream's: tolerance 0.5, Dijkstra, unknown allowed, no final approach
/// orientation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavfnParams {
    /// Metres the goal may move in x and y when it is unreachable.
    pub tolerance: f64,
    pub use_astar: bool,
    pub allow_unknown: bool,
    /// Give the last pose the direction of the final approach instead of the
    /// goal's orientation.
    pub use_final_approach_orientation: bool,
}

impl Default for NavfnParams {
    fn default() -> Self {
        Self { tolerance: 0.5, use_astar: false, allow_unknown: true, use_final_approach_orientation: false }
    }
}

impl NavfnParams {
    /// `planner_server.GridBased` in `nav2_no_map_params.yaml`.
    pub fn mower() -> Self {
        Self { tolerance: 0.0, ..Self::default() }
    }
}

/// The `nav2_core` planner exceptions. `NavfnPlanner::createPlan` throws
/// the first five; the rest come from `planner_server` itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlannerError {
    StartOutsideMapBounds,
    GoalOutsideMapBounds,
    GoalOccupied,
    NoValidPathCouldBeFound,
    PlannerCancelled,
    InvalidPlanner,
    StartOccupied,
    PlannerTimedOut,
    PlannerTFError,
    NoViapointsGiven,
    /// Any other `std::exception`.
    Unknown,
}

impl PlannerError {
    /// The `ComputePathToPose` result `error_code` `planner_server::computePlan`
    /// sets for it; `None` for a cancel, which ends the goal with
    /// `terminate_all` and no code. (`NoViapointsGiven` is thrown only for
    /// `ComputePathThroughPoses`, code 309; `computePlan` would catch it as a
    /// plain `std::exception`, UNKNOWN.)
    pub fn compute_path_to_pose_code(&self) -> Option<u16> {
        match self {
            PlannerError::Unknown | PlannerError::NoViapointsGiven => Some(200),
            PlannerError::InvalidPlanner => Some(201),
            PlannerError::PlannerTFError => Some(202),
            PlannerError::StartOutsideMapBounds => Some(203),
            PlannerError::GoalOutsideMapBounds => Some(204),
            PlannerError::StartOccupied => Some(205),
            PlannerError::GoalOccupied => Some(206),
            PlannerError::PlannerTimedOut => Some(207),
            PlannerError::NoValidPathCouldBeFound => Some(208),
            PlannerError::PlannerCancelled => None,
        }
    }
}

impl From<Cancelled> for PlannerError {
    fn from(_: Cancelled) -> Self {
        PlannerError::PlannerCancelled
    }
}

/// `squared_distance`: `dx * dx + dy * dy`, fused as shipped.
fn squared_distance(p1: &Pose2D, p2: &Pose2D) -> f64 {
    let dx = p1.x - p2.x;
    let dy = p1.y - p2.y;
    dx.mul_add(dx, dy * dy)
}

/// `NavfnPlanner`: the parameters and the NavFn it keeps between plans.
#[derive(Debug, Clone)]
pub struct NavfnPlanner {
    params: NavfnParams,
    planner: NavFn,
}

impl NavfnPlanner {
    /// `configure`: a NavFn the size of the costmap.
    pub fn new(params: NavfnParams, costmap: &Costmap2D) -> Self {
        Self { params, planner: NavFn::new(costmap.size_x() as i32, costmap.size_y() as i32) }
    }

    pub fn params(&self) -> &NavfnParams {
        &self.params
    }

    /// `dynamicParametersCallback`.
    pub fn set_params(&mut self, params: NavfnParams) {
        self.params = params;
    }

    /// The NavFn of the last plan (its potential, path in grid coordinates).
    pub fn navfn(&self) -> &NavFn {
        &self.planner
    }

    /// `createPlan(start, goal, cancel_checker)` with both poses in the
    /// costmap's frame. The path is in that frame. `costmap` is written to:
    /// see `clearRobotCell` in the module doc.
    pub fn create_plan(
        &mut self,
        costmap: &mut Costmap2D,
        start: &Pose2D,
        goal: &Pose2D,
        cancel: &mut dyn FnMut() -> bool,
    ) -> Result<Vec<Pose2D>, PlannerError> {
        if costmap.world_to_map(start.x, start.y).is_none() {
            return Err(PlannerError::StartOutsideMapBounds);
        }
        let Some((mx_goal, my_goal)) = costmap.world_to_map(goal.x, goal.y) else {
            return Err(PlannerError::GoalOutsideMapBounds);
        };
        if self.params.tolerance == 0.0 && costmap.cost(mx_goal, my_goal) == LETHAL_OBSTACLE {
            return Err(PlannerError::GoalOccupied);
        }

        if self.is_planner_out_of_date(costmap) {
            self.planner.set_nav_arr(costmap.size_x() as i32, costmap.size_y() as i32);
        }

        // corner case start (x, y) == goal (x, y): the start pose, with the
        // goal's orientation unless use_final_approach_orientation
        if start.x == goal.x && start.y == goal.y {
            let yaw =
                if start.yaw != goal.yaw && !self.params.use_final_approach_orientation { goal.yaw } else { start.yaw };
            return Ok(vec![Pose2D::new(start.x, start.y, yaw)]);
        }

        let mut path = Vec::new();
        if !self.make_plan(costmap, start, goal, self.params.tolerance, cancel, &mut path)? {
            return Err(PlannerError::NoValidPathCouldBeFound);
        }
        Ok(path)
    }

    /// `isPlannerOutOfDate`.
    fn is_planner_out_of_date(&self, costmap: &Costmap2D) -> bool {
        self.planner.nx() != costmap.size_x() as i32 || self.planner.ny() != costmap.size_y() as i32
    }

    /// `makePlan`: seed the potential at the robot (NavFn's goal), propagate
    /// until the goal cell (NavFn's start) is reached, and descend from the
    /// goal, or from the reachable point nearest to it within `tolerance`.
    fn make_plan(
        &mut self,
        costmap: &mut Costmap2D,
        start: &Pose2D,
        goal: &Pose2D,
        tolerance: f64,
        cancel: &mut dyn FnMut() -> bool,
        plan: &mut Vec<Pose2D>,
    ) -> Result<bool, PlannerError> {
        plan.clear();

        // createPlan has checked both poses against the origin, so these always
        // set the cell; upstream ignores the result (a cell one past the grid).
        let (mx, my, _) = self.world_to_map(costmap, start.x, start.y);
        Self::clear_robot_cell(costmap, mx, my);

        self.planner.set_nav_arr(costmap.size_x() as i32, costmap.size_y() as i32);
        self.planner.set_costmap(costmap.data(), true, self.params.allow_unknown);

        let map_start = [mx as i32, my as i32];
        let (mx, my, _) = self.world_to_map(costmap, goal.x, goal.y);
        let map_goal = [mx as i32, my as i32];

        self.planner.set_start(map_goal);
        self.planner.set_goal(map_start);
        if self.params.use_astar {
            self.planner.calc_nav_fn_astar(cancel)?;
        } else {
            self.planner.calc_nav_fn_dijkstra(cancel, true)?;
        }

        let resolution = costmap.resolution();
        let mut best_pose = Pose2D::default();
        let mut found_legal = false;

        let mut p = *goal;
        let potential = self.get_point_potential(costmap, p.x, p.y);
        if potential < POT_HIGH as f64 {
            // goal is reachable by itself
            best_pose = p;
            found_legal = true;
        } else {
            // the reachable point nearest to the goal on a resolution-spaced
            // grid within its tolerance square
            let mut best_sdist = f64::MAX;
            p.y = goal.y - tolerance;
            while p.y <= goal.y + tolerance {
                p.x = goal.x - tolerance;
                while p.x <= goal.x + tolerance {
                    let potential = self.get_point_potential(costmap, p.x, p.y);
                    let sdist = squared_distance(&p, goal);
                    if potential < POT_HIGH as f64 && sdist < best_sdist {
                        best_sdist = sdist;
                        best_pose = p;
                        found_legal = true;
                    }
                    p.x += resolution;
                }
                p.y += resolution;
            }
        }

        // a legal goal without a path from the potential ("this shouldn't
        // happen" upstream, but calcPath can fail) leaves the plan empty
        if found_legal && self.get_plan_from_potential(costmap, &best_pose, plan) {
            Self::smooth_approach_to_goal(&best_pose, plan);

            if self.params.use_final_approach_orientation {
                let n = plan.len();
                if n == 1 {
                    plan[0].yaw = start.yaw;
                } else if n > 1 {
                    let last = plan[n - 1];
                    let mut approach = plan[n - 2];
                    // NavFn can end a path with two equal poses
                    if (last.x - approach.x).abs() < 0.0001 && (last.y - approach.y).abs() < 0.0001 && n > 2 {
                        approach = plan[n - 3];
                    }
                    plan[n - 1].yaw = (last.y - approach.y).atan2(last.x - approach.x);
                }
            }
        }

        Ok(!plan.is_empty())
    }

    /// `smoothApproachToGoal`: the last pose becomes the goal if it is
    /// further from the one before than the goal is, or at the goal already;
    /// otherwise the goal is appended.
    fn smooth_approach_to_goal(goal: &Pose2D, plan: &mut Vec<Pose2D>) {
        if plan.len() >= 2 {
            let second_to_last = plan[plan.len() - 2];
            let last = plan[plan.len() - 1];
            if squared_distance(&last, &second_to_last) > squared_distance(goal, &second_to_last) {
                *plan.last_mut().unwrap() = *goal;
                return;
            }
            if squared_distance(&last, goal) < 1e-6 {
                *plan.last_mut().unwrap() = *goal;
                return;
            }
        }
        plan.push(*goal);
    }

    /// `getPlanFromPotential`: `calcPath` from the goal's cell for at most
    /// four times the larger grid side, reversed and converted to world
    /// poses with the identity orientation.
    fn get_plan_from_potential(&mut self, costmap: &Costmap2D, goal: &Pose2D, plan: &mut Vec<Pose2D>) -> bool {
        plan.clear();

        let (mx, my, _) = self.world_to_map(costmap, goal.x, goal.y);
        self.planner.set_start([mx as i32, my as i32]);

        let max_cycles = 4 * costmap.size_x().max(costmap.size_y()) as i32;
        let path_len = self.planner.calc_path(max_cycles, None);
        if path_len == 0 {
            return false;
        }

        let len = self.planner.path_len() as usize;
        let (x, y) = (self.planner.path_x(), self.planner.path_y());
        for i in (0..len).rev() {
            let (wx, wy) = Self::map_to_world(costmap, x[i] as f64, y[i] as f64);
            plan.push(Pose2D::new(wx, wy, 0.0));
        }
        !plan.is_empty()
    }

    /// `getPointPotential`: the potential at the rounded cell, `f64::MAX`
    /// off the grid.
    fn get_point_potential(&self, costmap: &Costmap2D, wx: f64, wy: f64) -> f64 {
        let (mx, my, inside) = self.world_to_map(costmap, wx, wy);
        if !inside {
            return f64::MAX;
        }
        let index = my as usize * self.planner.nx() as usize + mx as usize;
        self.planner.potarr()[index] as f64
    }

    /// The plugin's own `worldToMap`: rounds to the nearest cell corner.
    /// Returns the cell even when it is one past the grid (the result upstream
    /// writes before returning false); below the origin it is (0, 0), which
    /// upstream leaves unset.
    fn world_to_map(&self, costmap: &Costmap2D, wx: f64, wy: f64) -> (u32, u32, bool) {
        if wx < costmap.origin_x() || wy < costmap.origin_y() {
            return (0, 0, false);
        }
        // static_cast<int>(std::round(..)) into unsigned: fcvtas, saturating
        let mx = ((wx - costmap.origin_x()) / costmap.resolution()).round() as i32 as u32;
        let my = ((wy - costmap.origin_y()) / costmap.resolution()).round() as i32 as u32;
        (mx, my, mx < costmap.size_x() && my < costmap.size_y())
    }

    /// The plugin's own `mapToWorld`: no half-cell offset, one fmadd.
    fn map_to_world(costmap: &Costmap2D, mx: f64, my: f64) -> (f64, f64) {
        (costmap.resolution().mul_add(mx, costmap.origin_x()), costmap.resolution().mul_add(my, costmap.origin_y()))
    }

    /// `clearRobotCell`: `setCost(mx, my, FREE_SPACE)` on the raw index
    /// `my * size_x + mx`, so a start rounded to `mx == size_x` frees cell
    /// (0, my + 1). Upstream writes past the costmap when the index is past
    /// its end (a start in the top half-row); that write is dropped here.
    fn clear_robot_cell(costmap: &mut Costmap2D, mx: u32, my: u32) {
        let sx = costmap.size_x() as usize;
        let i = my as usize * sx + mx as usize;
        if i < costmap.data().len() {
            costmap.set_cost((i % sx) as u32, (i / sx) as u32, FREE_SPACE);
        }
    }
}

/// What `planner_server::computePlan` does around `createPlan` once the
/// poses are in the global frame: an empty path is `NoValidPathCouldBeFound`
/// (`validatePath`). Map the error to the action result with
/// [`PlannerError::compute_path_to_pose_code`].
pub fn compute_path_to_pose(
    planner: &mut NavfnPlanner,
    costmap: &mut Costmap2D,
    start: &Pose2D,
    goal: &Pose2D,
    cancel: &mut dyn FnMut() -> bool,
) -> Result<Vec<Pose2D>, PlannerError> {
    let path = planner.create_plan(costmap, start, goal, cancel)?;
    if path.is_empty() {
        return Err(PlannerError::NoValidPathCouldBeFound);
    }
    Ok(path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::costmap::NO_INFORMATION;

    fn open(sx: u32, sy: u32) -> Costmap2D {
        Costmap2D::new(sx, sy, 0.1, -1.0, -1.0, FREE_SPACE)
    }

    #[test]
    fn robot_params() {
        let p = NavfnParams::mower();
        assert_eq!((p.tolerance, p.use_astar, p.allow_unknown), (0.0, false, true));
        assert_eq!(NavfnParams::default().tolerance, 0.5);
    }

    #[test]
    fn early_errors() {
        let mut c = open(20, 20);
        c.set_cost(10, 10, LETHAL_OBSTACLE);
        let mut p = NavfnPlanner::new(NavfnParams::mower(), &c);
        let ok = Pose2D::new(0.0, 0.0, 0.0);
        let mut never = || false;
        assert_eq!(
            p.create_plan(&mut c, &Pose2D::new(-1.01, 0.0, 0.0), &ok, &mut never),
            Err(PlannerError::StartOutsideMapBounds)
        );
        assert_eq!(
            p.create_plan(&mut c, &ok, &Pose2D::new(1.0, 0.0, 0.0), &mut never),
            Err(PlannerError::GoalOutsideMapBounds)
        );
        assert_eq!(
            p.create_plan(&mut c, &ok, &Pose2D::new(0.05, 0.05, 0.0), &mut never),
            Err(PlannerError::GoalOccupied)
        );
        assert_eq!(PlannerError::GoalOccupied.compute_path_to_pose_code(), Some(206));
        assert_eq!(PlannerError::PlannerCancelled.compute_path_to_pose_code(), None);
    }

    #[test]
    fn start_equals_goal_takes_goal_orientation() {
        let mut c = open(10, 10);
        let mut p = NavfnPlanner::new(NavfnParams::mower(), &c);
        let path =
            p.create_plan(&mut c, &Pose2D::new(-0.5, -0.5, 0.3), &Pose2D::new(-0.5, -0.5, 1.2), &mut || false).unwrap();
        assert_eq!(path, vec![Pose2D::new(-0.5, -0.5, 1.2)]);
    }

    #[test]
    fn plan_ends_at_goal_and_clears_start_cell() {
        let mut c = open(30, 20);
        c.set_cost(5, 5, NO_INFORMATION);
        let mut p = NavfnPlanner::new(NavfnParams::mower(), &c);
        // (-0.5 + 1) / 0.1 = 5: the start rounds to cell (5, 5)
        let start = Pose2D::new(-0.5, -0.5, 0.0);
        let goal = Pose2D::new(0.9, 0.4, -2.0);
        let path = compute_path_to_pose(&mut p, &mut c, &start, &goal, &mut || false).unwrap();
        assert_eq!(*path.last().unwrap(), goal);
        assert!(path[..path.len() - 1].iter().all(|q| q.yaw == 0.0));
        assert!((path[0].x - start.x).abs() < 1e-9 && (path[0].y - start.y).abs() < 1e-9);
        assert_eq!(c.cost(5, 5), FREE_SPACE);
    }

    /// A pose in the top half-row rounds to row `size_y`, where upstream reads
    /// and writes past its arrays (not in the oracle): here the write is
    /// dropped and nothing is reached.
    #[test]
    fn top_half_row_is_unreachable_without_panicking() {
        let mut c = open(20, 10);
        let before = c.clone();
        let mut p = NavfnPlanner::new(NavfnParams::mower(), &c);
        let top = Pose2D::new(0.0, -1.0 + 9.8 * 0.1, 0.0);
        let inside = Pose2D::new(0.0, -0.5, 0.0);
        assert_eq!(p.create_plan(&mut c, &top, &inside, &mut || false), Err(PlannerError::NoValidPathCouldBeFound));
        assert_eq!(c, before);
        assert_eq!(p.create_plan(&mut c, &inside, &top, &mut || false), Err(PlannerError::NoValidPathCouldBeFound));
    }

    #[test]
    fn follows_a_resized_costmap() {
        let mut c = open(10, 10);
        let mut p = NavfnPlanner::new(NavfnParams::mower(), &c);
        let mut bigger = open(40, 30);
        let goal = Pose2D::new(2.5, 1.5, 0.0);
        assert_eq!(
            p.create_plan(&mut c, &Pose2D::new(-0.5, -0.5, 0.0), &goal, &mut || false).err(),
            Some(PlannerError::GoalOutsideMapBounds)
        );
        let path = p.create_plan(&mut bigger, &Pose2D::new(-0.5, -0.5, 0.0), &goal, &mut || false).unwrap();
        assert_eq!(*path.last().unwrap(), goal);
        assert_eq!((p.navfn().nx(), p.navfn().ny()), (40, 30));
    }
}
