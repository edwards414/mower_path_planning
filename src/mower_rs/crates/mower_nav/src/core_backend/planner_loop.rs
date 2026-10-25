//! The planner thread's pure half: planner_server on the global costmap, the
//! way upstream runs it on a thread of its own, so a NavFn search on a big
//! zone map (0.25-0.4 s at 1000-1240 cells a side on the RK3568) never holds
//! up the controller or the smoother on nav-core.
//!
//! [`PlannerLoop`] owns:
//!
//! * the `PlannerServer` (`PlannerServer::mower`), which takes the tree's
//!   requests ([`ToPlanner`], from nav-core's `NavStack` with its
//!   `ExternalPlanner`) and is ticked the way `NavStack` ticks its inline
//!   planner: as soon as it has a goal, then every 10 ms while it waits for
//!   the costmap. The stack numbers its goals as the server does; the results
//!   go back in the stack's numbering all the same ([`FromPlanner`]);
//! * the global costmap (1 Hz, `track_unknown_space: true`). As in
//!   `StaticLayer`, the map's own grid is kept apart from the master grid the
//!   planner reads, which NavFn's `clearRobotCell` writes into. An update
//!   repaints the master only for a newer map or after a clear
//!   (`has_updated_data_`), so the holes NavFn leaves stay until then, as
//!   upstream. A clear (forwarded from the tree) makes it not current until
//!   its next update. A newer map waits for the next update while a goal is
//!   being planned (`map_buffer_`) and is taken at once otherwise, which is
//!   when mower_map changes them: between missions. The first map is painted
//!   at once; NavFn re-sizes its arrays to a map of another size
//!   (`isPlannerOutOfDate`). Before any map the server plans on a one-cell
//!   placeholder that is never current, so a goal waits and times out as
//!   upstream's does.
//!
//! nav-core applies a tick's global clears before the tick's requests (the
//! order an inline planner sees them in), and sends both in that order.

use std::collections::HashMap;

use mower_nav_core::action::GoalId;
use mower_nav_core::costmap::{static_costmap, Costmap2D, OccupancyGrid, StaticLayerParams, NO_INFORMATION};
use mower_nav_core::geometry::Pose2D;
use mower_nav_core::nav_stack::{PlanRequest, StackLog, COSTMAP_POLL_NS};
use mower_nav_core::path_handler::Path;
use mower_nav_core::planner_server::{ComputePathResult, PlanInput, PlannerServer};

use super::core_loop::{next_period, valid_grid, SEC};

/// `update_frequency` of the global costmap (nav2_no_map_params.yaml).
const GLOBAL_UPDATE_NS: i64 = SEC;

/// nav-core to the planner, in the order the tree produced them.
#[derive(Debug, Clone, PartialEq)]
pub enum ToPlanner {
    /// The tree cleared the global costmap (`clear_entirely_global_costmap`).
    ClearGlobal,
    /// From `StackOutput::plan_requests`.
    Request(PlanRequest),
}

/// The planner to nav-core.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FromPlanner {
    /// In the stack's numbering.
    pub results: Vec<ComputePathResult>,
    /// `/plan`.
    pub plans: Vec<Path>,
    /// planner_server's WARN lines: `ComputePathFailed`, `PlannerMissedRate`.
    pub log: Vec<StackLog>,
    /// Other `[server] ...` lines for the event ring.
    pub notes: Vec<String>,
}

impl FromPlanner {
    pub fn is_empty(&self) -> bool {
        self.results.is_empty() && self.plans.is_empty() && self.log.is_empty() && self.notes.is_empty()
    }
}

/// The global costmap.
#[derive(Debug)]
struct Global {
    /// The static layer's own grid (`processMap`).
    layer: Costmap2D,
    /// The master grid the planner reads and NavFn writes into.
    master: Costmap2D,
    current: bool,
    /// A newer map, taken at the next update (`map_buffer_`).
    buffered: Option<Costmap2D>,
    next_update_ns: i64,
}

/// See the module doc.
pub struct PlannerLoop {
    server: PlannerServer,
    global: Option<Global>,
    /// What the server plans on before the first map.
    placeholder: Costmap2D,
    /// The server's next pass; `None` while it has nothing to do.
    next_pass_ns: Option<i64>,
    /// The stack's goal ids by the server's, and back.
    stack_ids: HashMap<GoalId, GoalId>,
    server_ids: HashMap<GoalId, GoalId>,
    /// A wait for a first map already reported.
    waiting_reported: bool,
    notes: Vec<String>,
    now_ns: Option<i64>,
}

impl Default for PlannerLoop {
    fn default() -> Self {
        Self::new()
    }
}

impl PlannerLoop {
    pub fn new() -> Self {
        let placeholder = Costmap2D::new(1, 1, 0.05, 0.0, 0.0, NO_INFORMATION);
        Self {
            server: PlannerServer::mower(&placeholder),
            global: None,
            placeholder,
            next_pass_ns: None,
            stack_ids: HashMap::new(),
            server_ids: HashMap::new(),
            waiting_reported: false,
            notes: Vec::new(),
            now_ns: None,
        }
    }

    /// A valid global map has arrived (readiness' `global_map`).
    pub fn has_map(&self) -> bool {
        self.global.is_some()
    }

    /// A `/map_grid_global`; see the module doc for when it is taken.
    pub fn set_global_map(&mut self, now_ns: i64, grid: &OccupancyGrid) {
        let now = self.clock(now_ns);
        if !valid_grid(grid) {
            self.notes.push("[global_costmap] Received map message is malformed. Rejecting.".into());
            return;
        }
        let layer = static_costmap(grid, &StaticLayerParams::new(true));
        let planning = self.server.has_work();
        match &mut self.global {
            Some(g) if planning => g.buffered = Some(layer),
            Some(g) => {
                g.layer = layer;
                g.master.clone_from(&g.layer);
                g.buffered = None;
                g.current = true;
            }
            None => {
                self.global = Some(Global {
                    master: layer.clone(),
                    layer,
                    current: true,
                    buffered: None,
                    next_update_ns: now + GLOBAL_UPDATE_NS,
                })
            }
        }
    }

    /// A message from nav-core.
    pub fn handle(&mut self, now_ns: i64, msg: ToPlanner) {
        let now = self.clock(now_ns);
        match msg {
            ToPlanner::ClearGlobal => {
                if let Some(g) = &mut self.global {
                    g.current = false;
                }
            }
            ToPlanner::Request(PlanRequest::Send { id, goal }) => {
                let was_idle = self.server.current_goal().is_none();
                let server_id = self.server.send_goal(goal);
                self.stack_ids.insert(server_id, id);
                self.server_ids.insert(id, server_id);
                // a goal for an idle server starts its worker at once
                if was_idle && self.server.has_work() {
                    self.next_pass_ns = Some(now);
                }
            }
            ToPlanner::Request(PlanRequest::Cancel(id)) => {
                if let Some(&server_id) = self.server_ids.get(&id) {
                    self.server.cancel_goal(server_id);
                }
            }
        }
    }

    /// When the loop next has something to do; `None` while the server has
    /// nothing (an update that fell due meanwhile runs at the next tick).
    pub fn next_due_ns(&self) -> Option<i64> {
        if !self.server.has_work() {
            return None;
        }
        [self.next_pass_ns, self.global.as_ref().map(|g| g.next_update_ns)].into_iter().flatten().min()
    }

    /// The costmap update and the server's pass, when due. `robot_pose` is
    /// the latest `map <- base_footprint` (the start of a plan).
    pub fn tick(&mut self, now_ns: i64, robot_pose: Option<Pose2D>) -> FromPlanner {
        let now = self.clock(now_ns);
        let mut out = FromPlanner::default();
        self.update_costmap(now);
        if self.next_pass_ns.is_some_and(|t| t <= now) && self.server.has_work() {
            let (costmap, costmap_current) = match &mut self.global {
                Some(g) => (&mut g.master, g.current),
                None => (&mut self.placeholder, false),
            };
            let p = self.server.tick(PlanInput { now_ns: now, costmap_current, robot_pose, costmap });
            for r in p.results {
                if let Some(e) = r.error {
                    out.log.push(StackLog::ComputePathFailed(e));
                }
                self.server_ids.retain(|_, s| *s != r.goal);
                if let Some(id) = self.stack_ids.remove(&r.goal) {
                    out.results.push(ComputePathResult { goal: id, ..r });
                }
            }
            if let Some(s) = p.missed_rate {
                out.log.push(StackLog::PlannerMissedRate(s));
            }
            out.plans.extend(p.plans);
            self.next_pass_ns = p.waiting_for_costmap.then_some(now + COSTMAP_POLL_NS);
            if p.waiting_for_costmap && self.global.is_none() && !self.waiting_reported {
                self.waiting_reported = true;
                out.notes.push("[planner_server] waiting for the global costmap: no global map received".into());
            }
        } else if !self.server.has_work() {
            self.next_pass_ns = None;
        }
        if !self.server.has_work() {
            self.waiting_reported = false;
        }
        out.notes.append(&mut self.notes);
        out
    }

    /// The wall clock, held while it is behind the latest time seen (nav-core
    /// reports the step).
    fn clock(&mut self, now_ns: i64) -> i64 {
        let now = self.now_ns.map_or(now_ns, |last| last.max(now_ns));
        self.now_ns = Some(now);
        now
    }

    /// The global costmap's update loop, when due.
    fn update_costmap(&mut self, now: i64) {
        let Some(g) = self.global.as_mut().filter(|g| g.next_update_ns <= now) else {
            return;
        };
        if let Some(layer) = g.buffered.take() {
            g.layer = layer;
            g.master.clone_from(&g.layer);
            g.current = true;
        } else if !g.current {
            g.master.clone_from(&g.layer);
            g.current = true;
        }
        g.next_update_ns = next_period(g.next_update_ns, GLOBAL_UPDATE_NS, now);
    }
}

#[cfg(test)]
impl PlannerLoop {
    pub(super) fn current(&self) -> bool {
        self.global.as_ref().is_some_and(|g| g.current)
    }

    pub(super) fn next_update_ns(&self) -> Option<i64> {
        self.global.as_ref().map(|g| g.next_update_ns)
    }

    pub(super) fn size(&self) -> Option<(u32, u32)> {
        self.global.as_ref().map(|g| (g.master.size_x(), g.master.size_y()))
    }
}

#[cfg(test)]
mod tests {
    use mower_nav_core::action::GoalStatus;
    use mower_nav_core::planner_server::{error_code, ComputePathToPose};

    use super::super::core_loop::MS;
    use super::*;

    const T0: i64 = 1_792_900_000_000_000_000;

    /// `w` x `h` cells at 0.05 m from (-1, -1), free inside a lethal border.
    fn grid(w: u32, h: u32) -> OccupancyGrid {
        let data = (0..h).flat_map(|j| (0..w).map(move |i| if i == 0 || j == 0 || i == w - 1 || j == h - 1 { 100 } else { 0 })).collect();
        OccupancyGrid { width: w, height: h, resolution: 0.05f32 as f64, origin_x: -1.0, origin_y: -1.0, data }
    }

    fn send(id: GoalId, x: f64) -> ToPlanner {
        ToPlanner::Request(PlanRequest::Send { id, goal: ComputePathToPose::to(Pose2D::new(x, 1.0, 0.0)) })
    }

    const ROBOT: Option<Pose2D> = Some(Pose2D::new(1.0, 1.0, 0.0));

    #[test]
    fn a_goal_is_planned_at_once_and_answered_in_the_stacks_numbering() {
        let mut p = PlannerLoop::new();
        p.set_global_map(T0, &grid(200, 160));
        assert!(p.has_map());
        assert_eq!(p.next_due_ns(), None);
        p.handle(T0, send(1, 5.0));
        assert_eq!(p.next_due_ns(), Some(T0));
        let out = p.tick(T0, ROBOT);
        assert_eq!(out.results.len(), 1);
        let r = &out.results[0];
        assert_eq!((r.goal, r.status, r.error_code), (1, GoalStatus::Succeeded, 0));
        assert_eq!(out.plans.len(), 1);
        assert!(out.log.is_empty() && out.notes.is_empty());
        assert_eq!(p.next_due_ns(), None);
    }

    #[test]
    fn a_clear_holds_the_next_goal_until_the_update_repaints_the_costmap() {
        let mut p = PlannerLoop::new();
        p.set_global_map(T0, &grid(200, 160));
        let update = p.next_update_ns().unwrap();
        p.handle(T0 + 300 * MS, ToPlanner::ClearGlobal);
        p.handle(T0 + 300 * MS, send(1, 5.0));
        assert!(!p.current());
        let mut now = T0 + 300 * MS;
        let mut answered = None;
        while answered.is_none() {
            let out = p.tick(now, ROBOT);
            if let Some(r) = out.results.first() {
                answered = Some((now, r.status));
            }
            now = p.next_due_ns().unwrap_or(now + 10 * MS);
        }
        let (at, status) = answered.unwrap();
        assert_eq!(status, GoalStatus::Succeeded);
        assert!(p.current());
        assert_eq!(at, update, "planned at the 1 Hz update after the clear");
    }

    #[test]
    fn a_new_map_waits_for_the_update_while_a_goal_waits_and_is_taken_at_once_otherwise() {
        let mut p = PlannerLoop::new();
        p.set_global_map(T0, &grid(200, 160));
        // between goals: at once
        p.set_global_map(T0 + 100 * MS, &grid(240, 160));
        assert_eq!(p.size(), Some((240, 160)));
        // while a goal waits for the costmap: at the next update
        p.handle(T0 + 200 * MS, ToPlanner::ClearGlobal);
        p.handle(T0 + 200 * MS, send(1, 12.5));
        p.tick(T0 + 200 * MS, ROBOT);
        p.set_global_map(T0 + 300 * MS, &grid(320, 160));
        assert_eq!(p.size(), Some((240, 160)));
        let update = p.next_update_ns().unwrap();
        p.tick(update - MS, ROBOT);
        assert_eq!(p.size(), Some((240, 160)));
        p.tick(update, ROBOT);
        assert_eq!(p.size(), Some((320, 160)));
        // planned at its next pass (it polls every 10 ms), on the new map
        // (x = 12.5 is beyond the old one's 11 m): NavFn re-sized
        let out = p.tick(p.next_due_ns().unwrap(), ROBOT);
        assert_eq!(out.results.len(), 1);
        assert_eq!(out.results[0].status, GoalStatus::Succeeded);
        assert!(out.plans[0].last().is_some_and(|q| (q.x - 12.5).abs() < 1e-9));
    }

    #[test]
    fn without_a_map_a_goal_waits_and_times_out() {
        let mut p = PlannerLoop::new();
        p.handle(T0, send(7, 5.0));
        let out = p.tick(T0, ROBOT);
        assert_eq!(out.notes, ["[planner_server] waiting for the global costmap: no global map received"]);
        let (now, timed_out) = loop {
            let now = p.next_due_ns().expect("waits");
            let out = p.tick(now, ROBOT);
            if let Some(r) = out.results.first() {
                assert_eq!(out.log, [StackLog::ComputePathFailed(mower_nav_core::planner::PlannerError::PlannerTimedOut)]);
                break (now, r.clone());
            }
            assert!(out.notes.is_empty(), "reported once");
        };
        assert_eq!((timed_out.goal, timed_out.status, timed_out.error_code), (7, GoalStatus::Aborted, error_code::TIMEOUT));
        assert!(now - T0 > SEC);
        // the server does not see a cancel during the costmap wait (as
        // upstream): the goal ends at the timeout, canceled
        p.handle(now, send(8, 5.0));
        p.tick(now, ROBOT);
        p.handle(now + MS, ToPlanner::Request(PlanRequest::Cancel(8)));
        let r = loop {
            let out = p.tick(p.next_due_ns().expect("waits"), ROBOT);
            if let Some(r) = out.results.first() {
                break r.clone();
            }
        };
        assert_eq!((r.goal, r.status, r.error_code), (8, GoalStatus::Canceled, error_code::TIMEOUT));
        // a malformed map is rejected with a note
        let mut bad = grid(10, 10);
        bad.data.pop();
        p.set_global_map(now, &bad);
        assert!(!p.has_map());
        assert_eq!(p.tick(now, ROBOT).notes, ["[global_costmap] Received map message is malformed. Rejecting."]);
    }
}
