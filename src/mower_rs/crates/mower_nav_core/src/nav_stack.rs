//! The three servers mower_nav talks to, as one unit on one clock:
//! `bt_navigator` ([`Navigator`]), `planner_server` ([`PlannerServer`]) and
//! `controller_server` ([`ControllerServer`]).
//!
//! Upstream each runs its goals on its own thread at its own rate; here
//! [`NavStack::tick`] runs whichever is due, in this order: the tree (every
//! `bt_loop_duration`, from when its goal started), the planner (as soon as
//! it has a goal, then every 10 ms while it waits for the costmap, upstream's
//! `rclcpp::Rate r(100)`), the controller (as soon as a goal starts its loop,
//! then at `controller_frequency`, every 10 ms while it waits for the
//! costmap). The loop rates are `rclcpp::WallRate`s: built when a loop starts
//! and slept on after every pass, so a pass that overran is followed at once
//! by the next. Tick at least every 10 ms; ticking more often only adds
//! idle calls.
//!
//! Results go where the goal came from: the tree's `compute_path_to_pose`
//! and `follow_path` goals to the tree, which sees them at its next tick (one
//! `bt_loop_duration` later at most, see [`crate::navigator`]); the
//! `follow_path` goals sent with [`NavStack::send_follow_path`] (mower_nav's
//! coverage segments, which upstream go straight to the controller server)
//! to the output. Both share the one controller server, as upstream.
//!
//! The costmaps are the caller's. A clear the tree asked for is in
//! [`StackOutput::clears`]: the costmap is not current from then until its
//! next update (and that tick's later passes already see it so).
//!
//! What the servers would have logged at WARN or ERROR is in
//! [`StackOutput::log`], for the caller's diagnostics; nothing here reads it.
//!
//! The planner is either inline ([`NavStack::mower`]: the stack's own
//! [`PlannerServer`], ticked by [`NavStack::tick`] on the stack's clock) or
//! external ([`NavStack::mower_external_planner`]: an [`ExternalPlanner`]).
//! An external planner_server runs elsewhere, on its own thread as upstream's
//! does, so a long NavFn search does not hold up the controller. The tree's
//! `compute_path_to_pose` goals and cancels then come out in
//! [`StackOutput::plan_requests`], numbered by the stack from 1 as
//! `PlannerServer` numbers them, and the results go back in through
//! [`NavStack::on_compute_path_result`]; the tree sees a result at its next
//! tick, as it sees an inline planner's. The planner owns the global costmap
//! then: it applies the tick's [`StackOutput::clears`] of it before the
//! tick's requests (the inline planner's pass, too, comes after the tree's and
//! sees them), and the stack reads neither `StackInput::global_costmap` nor
//! `global_costmap_current`.

use crate::action::GoalId;
use crate::controller_server::{ControllerServer, CycleInput, Feedback, FollowPathResult};
use crate::costmap::Costmap2D;
use crate::geometry::{Pose2D, Transform2D, Twist2D};
use crate::navigator::{CostmapId, NavInput, NavServers, NavigateFeedback, NavigateResult, Navigator};
use crate::path_handler::Path;
use crate::planner::PlannerError;
use crate::planner_server::{ComputePathResult, ComputePathToPose, PlanInput, PlanOutput, PlannerServer};
use crate::rpp::{Controller, ControllerError, RegulatedPurePursuit};
use crate::shim::RotationShim;
use crate::smoother::TwistStamped;

/// `rclcpp::Rate r(100)`: how often a server checks a costmap it waits for.
pub const COSTMAP_POLL_NS: i64 = 10_000_000;

/// `rclcpp::WallRate`.
#[derive(Debug, Clone, Copy)]
struct Rate {
    period_ns: i64,
    last_interval_ns: i64,
}

impl Rate {
    fn new(period_ns: i64, now: i64) -> Self {
        Self { period_ns, last_interval_ns: now }
    }

    /// `sleep()` at `now`: when the next pass starts.
    fn sleep(&mut self, now: i64) -> i64 {
        let mut next = self.last_interval_ns + self.period_ns;
        if now < self.last_interval_ns {
            next = now + self.period_ns;
        }
        self.last_interval_ns += self.period_ns;
        if next <= now {
            // an entire cycle missed: reset the next interval
            if now > next + self.period_ns {
                self.last_interval_ns = now + self.period_ns;
            }
            return now;
        }
        next
    }
}

/// What a tick reads from tf, odometry and the costmaps.
#[derive(Debug)]
pub struct StackInput<'a> {
    pub now_ns: i64,
    /// tf `map <- base_footprint` (latest), `None` if the lookup failed: the
    /// planner's start and bt_navigator's robot pose.
    pub map_pose: Option<Pose2D>,
    /// tf `odom <- base_footprint` (latest): the controller's robot pose.
    pub odom_pose: Option<Pose2D>,
    /// The latest odometry twist.
    pub odom_twist: Twist2D,
    /// tf `odom <- map`.
    pub odom_from_map: Option<Transform2D>,
    pub local_costmap: &'a Costmap2D,
    pub local_costmap_current: bool,
    /// NavFn writes to it (`clearRobotCell`). Not read with an external
    /// planner (any costmap will do).
    pub global_costmap: &'a mut Costmap2D,
    pub global_costmap_current: bool,
}

/// What a tick produced.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct StackOutput {
    /// `cmd_vel`, in order.
    pub commands: Vec<TwistStamped>,
    pub navigate_results: Vec<NavigateResult>,
    pub navigate_feedback: Option<(GoalId, NavigateFeedback)>,
    /// Results of the goals sent with [`NavStack::send_follow_path`].
    pub follow_results: Vec<FollowPathResult>,
    pub follow_feedback: Option<(GoalId, Feedback)>,
    /// Costmaps the tree cleared: not current until their next update.
    pub clears: Vec<CostmapId>,
    /// `/plan`.
    pub plans: Vec<Path>,
    /// What the servers logged at WARN or ERROR, in order.
    pub log: Vec<StackLog>,
    /// With an [`ExternalPlanner`]: the tree's `compute_path_to_pose` goals
    /// and cancels, in order, for after this tick's [`StackOutput::clears`].
    pub plan_requests: Vec<PlanRequest>,
}

/// A request of the tree for an external planner_server.
#[derive(Debug, Clone, PartialEq)]
pub enum PlanRequest {
    /// A `compute_path_to_pose` goal, numbered from 1 in the order sent.
    Send { id: GoalId, goal: ComputePathToPose },
    /// `handle_cancel` for a goal sent before.
    Cancel(GoalId),
}

/// A server's WARN or ERROR line (upstream's `/rosout`), as data.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StackLog {
    /// controller_server tolerated a `NoValidControl` (`RCLCPP_WARN`, once
    /// per pass until `failure_tolerance` runs out).
    ControlTolerated(ControllerError),
    /// The exception that ended a `follow_path` goal, the tree's or one
    /// sent with [`NavStack::send_follow_path`] (`RCLCPP_ERROR`).
    FollowPathFailed(ControllerError),
    /// The exception that ended a `compute_path_to_pose` goal
    /// (`exceptionWarning`).
    ComputePathFailed(PlannerError),
    /// "Planner loop missed its desired rate": seconds the plan took.
    PlannerMissedRate(f64),
}

/// The stack's planner_server: [`PlannerServer`], ticked inside
/// [`NavStack::tick`], or an [`ExternalPlanner`] standing in for one that runs
/// elsewhere (see the module doc).
pub trait StackPlanner {
    /// `handle_goal` + `handle_accepted`: the goal's id.
    fn send_goal(&mut self, goal: ComputePathToPose) -> GoalId;
    /// `handle_cancel`.
    fn cancel_goal(&mut self, goal: GoalId) -> bool;
    /// The executing goal.
    fn current_goal(&self) -> Option<GoalId>;
    /// A pass has something to do: tick.
    fn has_work(&self) -> bool;
    /// One worker pass.
    fn tick(&mut self, input: PlanInput<'_>) -> PlanOutput;
    /// The requests for a planner elsewhere since the last call, in order.
    fn take_requests(&mut self) -> Vec<PlanRequest> {
        Vec::new()
    }
}

impl StackPlanner for PlannerServer {
    fn send_goal(&mut self, goal: ComputePathToPose) -> GoalId {
        PlannerServer::send_goal(self, goal)
    }

    fn cancel_goal(&mut self, goal: GoalId) -> bool {
        PlannerServer::cancel_goal(self, goal)
    }

    fn current_goal(&self) -> Option<GoalId> {
        PlannerServer::current_goal(self)
    }

    fn has_work(&self) -> bool {
        PlannerServer::has_work(self)
    }

    fn tick(&mut self, input: PlanInput<'_>) -> PlanOutput {
        PlannerServer::tick(self, input)
    }
}

/// The stack's side of a planner_server that runs elsewhere: it numbers the
/// tree's goals from 1, as [`PlannerServer`] does, and hands them and their
/// cancels out as [`PlanRequest`]s. It plans nothing and logs nothing (the
/// planner's [`PlanOutput`] is the planner's side's to report), and is never
/// busy as far as the stack's schedule goes.
#[derive(Debug, Clone)]
pub struct ExternalPlanner {
    next_goal: GoalId,
    requests: Vec<PlanRequest>,
}

impl Default for ExternalPlanner {
    fn default() -> Self {
        Self::new()
    }
}

impl ExternalPlanner {
    pub fn new() -> Self {
        Self { next_goal: 1, requests: Vec::new() }
    }

    /// Goals sent so far (`PlannerServer::goals_received`).
    pub fn goals_sent(&self) -> u64 {
        self.next_goal - 1
    }
}

impl StackPlanner for ExternalPlanner {
    fn send_goal(&mut self, goal: ComputePathToPose) -> GoalId {
        let id = self.next_goal;
        self.next_goal += 1;
        self.requests.push(PlanRequest::Send { id, goal });
        id
    }

    fn cancel_goal(&mut self, goal: GoalId) -> bool {
        self.requests.push(PlanRequest::Cancel(goal));
        true
    }

    fn current_goal(&self) -> Option<GoalId> {
        None
    }

    fn has_work(&self) -> bool {
        false
    }

    fn tick(&mut self, _input: PlanInput<'_>) -> PlanOutput {
        PlanOutput::default()
    }

    fn take_requests(&mut self) -> Vec<PlanRequest> {
        std::mem::take(&mut self.requests)
    }
}

/// The servers, their schedules, and who sent which goal.
#[derive(Debug, Clone)]
pub struct NavStack<C: Controller, P: StackPlanner = PlannerServer> {
    pub navigator: Navigator,
    pub planner: P,
    pub controller: ControllerServer<C>,
    bt_rate: Option<Rate>,
    next_bt_ns: Option<i64>,
    next_planner_ns: Option<i64>,
    controller_rate: Option<Rate>,
    next_controller_ns: Option<i64>,
    /// `follow_path` goals the tree sent whose result is still out.
    tree_follow_goals: Vec<GoalId>,
}

impl NavStack<RotationShim<RegulatedPurePursuit>> {
    /// The three servers as `nav2_no_map_params.yaml` configures them, for a
    /// global costmap of this size.
    pub fn mower(global_costmap: &Costmap2D) -> Self {
        Self::new(Navigator::mower(), PlannerServer::mower(global_costmap), ControllerServer::mower())
    }
}

impl NavStack<RotationShim<RegulatedPurePursuit>, ExternalPlanner> {
    /// The tree and the controller server as `nav2_no_map_params.yaml`
    /// configures them, the planner_server elsewhere (`PlannerServer::mower`
    /// on the global costmap, there).
    pub fn mower_external_planner() -> Self {
        Self::new(Navigator::mower(), ExternalPlanner::new(), ControllerServer::mower())
    }
}

impl<C: Controller> NavStack<C, ExternalPlanner> {
    /// The result of a goal from [`StackOutput::plan_requests`]. The tree
    /// sees it at its next tick, as it sees an inline planner's (a result for
    /// a goal it no longer waits on is dropped there).
    pub fn on_compute_path_result(&mut self, r: ComputePathResult) {
        self.navigator.on_compute_path_result(r);
    }
}

/// [`NavServers`] for the tree: the stack's planner and controller.
struct Servers<'a, C: Controller, P: StackPlanner> {
    planner: &'a mut P,
    controller: &'a mut ControllerServer<C>,
    tree_follow_goals: &'a mut Vec<GoalId>,
    clears: &'a mut Vec<CostmapId>,
}

impl<C: Controller, P: StackPlanner> NavServers for Servers<'_, C, P> {
    fn send_compute_path(&mut self, goal: ComputePathToPose) -> GoalId {
        self.planner.send_goal(goal)
    }

    fn cancel_compute_path(&mut self, goal: GoalId) {
        self.planner.cancel_goal(goal);
    }

    fn send_follow_path(&mut self, path: Path) -> GoalId {
        let id = self.controller.send_goal(path);
        self.tree_follow_goals.push(id);
        id
    }

    fn cancel_follow_path(&mut self, goal: GoalId) {
        self.controller.cancel_goal(goal);
    }

    fn clear_costmap(&mut self, costmap: CostmapId) {
        self.clears.push(costmap);
    }
}

impl<C: Controller, P: StackPlanner> NavStack<C, P> {
    pub fn new(navigator: Navigator, planner: P, controller: ControllerServer<C>) -> Self {
        Self {
            navigator,
            planner,
            controller,
            bt_rate: None,
            next_bt_ns: None,
            next_planner_ns: None,
            controller_rate: None,
            next_controller_ns: None,
            tree_follow_goals: Vec::new(),
        }
    }

    /// A `navigate_to_pose` goal, in `map`. It starts on the next tick when
    /// nothing navigates.
    pub fn send_navigate(&mut self, pose: Pose2D) -> GoalId {
        if self.next_bt_ns.is_none() {
            self.next_bt_ns = Some(i64::MIN);
        }
        self.navigator.send_goal(pose)
    }

    pub fn cancel_navigate(&mut self, goal: GoalId) -> bool {
        self.navigator.cancel_goal(goal)
    }

    /// A `follow_path` goal straight to the controller server (mower_nav's
    /// coverage segments). It starts on the next tick when the controller is
    /// idle, otherwise it preempts at the controller's next pass.
    pub fn send_follow_path(&mut self, path: Path) -> GoalId {
        if self.next_controller_ns.is_none() {
            self.next_controller_ns = Some(i64::MIN);
        }
        self.controller.send_goal(path)
    }

    pub fn cancel_follow_path(&mut self, goal: GoalId) -> bool {
        self.controller.cancel_goal(goal)
    }

    /// When a tick next has something to do; `None` while all is idle.
    pub fn next_tick_ns(&self) -> Option<i64> {
        [self.next_bt_ns, self.next_planner_ns, self.next_controller_ns].into_iter().flatten().min()
    }

    pub fn tick(&mut self, input: StackInput<'_>) -> StackOutput {
        let now = input.now_ns;
        let mut out = StackOutput::default();

        // bt_navigator: one loop iteration of the tree
        if self.next_bt_ns.is_some_and(|t| t <= now) && self.navigator.has_work() {
            let runs = self.navigator.runs_started();
            let planner_was_idle = self.planner.current_goal().is_none();
            let controller_was_idle = !self.controller.is_executing();
            let nav = {
                let mut servers = Servers {
                    planner: &mut self.planner,
                    controller: &mut self.controller,
                    tree_follow_goals: &mut self.tree_follow_goals,
                    clears: &mut out.clears,
                };
                self.navigator.tick(&NavInput { now_ns: now, robot_pose: input.map_pose }, &mut servers)
            };
            out.navigate_results.extend(nav.results);
            out.navigate_feedback = nav.feedback;
            if self.navigator.runs_started() != runs {
                self.bt_rate = Some(Rate::new(self.navigator.params().bt_loop_duration_ms * 1_000_000, now));
            }
            self.next_bt_ns = match (&mut self.bt_rate, self.navigator.is_executing()) {
                (Some(rate), true) => Some(rate.sleep(now)),
                _ => None,
            };
            // a goal for an idle server starts its worker at once
            if planner_was_idle && self.planner.has_work() {
                self.next_planner_ns = Some(now);
            }
            if controller_was_idle && self.controller.is_executing() {
                self.next_controller_ns = Some(now);
            }
        } else if !self.navigator.has_work() {
            self.next_bt_ns = None;
        }
        out.plan_requests.extend(self.planner.take_requests());

        // planner_server's worker
        if self.next_planner_ns.is_some_and(|t| t <= now) && self.planner.has_work() {
            let global_current = input.global_costmap_current && !out.clears.contains(&CostmapId::Global);
            let p = self.planner.tick(PlanInput {
                now_ns: now,
                costmap_current: global_current,
                robot_pose: input.map_pose,
                costmap: input.global_costmap,
            });
            for r in p.results {
                if let Some(e) = r.error {
                    out.log.push(StackLog::ComputePathFailed(e));
                }
                self.navigator.on_compute_path_result(r);
            }
            if let Some(s) = p.missed_rate {
                out.log.push(StackLog::PlannerMissedRate(s));
            }
            out.plans.extend(p.plans);
            self.next_planner_ns = p.waiting_for_costmap.then_some(now + COSTMAP_POLL_NS);
        } else if !self.planner.has_work() {
            self.next_planner_ns = None;
        }

        // controller_server's worker; a pass that overran its period is
        // followed at once by the next (WallRate::sleep returns false)
        let local_current = input.local_costmap_current && !out.clears.contains(&CostmapId::Local);
        let mut passes = 0;
        while self.next_controller_ns.is_some_and(|t| t <= now) && passes < 2 {
            passes += 1;
            let loops = self.controller.loops_started();
            let c = self.controller.tick(&CycleInput {
                now_ns: now,
                costmap_current: local_current,
                robot_pose: input.odom_pose,
                odom_twist: input.odom_twist,
                odom_from_map: input.odom_from_map,
                costmap: input.local_costmap,
            });
            if let Some(w) = c.warning {
                out.log.push(StackLog::ControlTolerated(w));
            }
            out.commands.extend(c.commands);
            for r in c.results {
                if let Some(e) = r.error {
                    out.log.push(StackLog::FollowPathFailed(e));
                }
                if let Some(i) = self.tree_follow_goals.iter().position(|&g| g == r.goal) {
                    self.tree_follow_goals.swap_remove(i);
                    self.navigator.on_follow_path_result(r);
                } else {
                    out.follow_results.push(r);
                }
            }
            if let Some((goal, fb)) = c.feedback {
                if !self.tree_follow_goals.contains(&goal) {
                    out.follow_feedback = Some((goal, fb));
                }
            }
            if self.controller.loops_started() != loops {
                let period = (1e9 / self.controller.params().controller_frequency).round() as i64;
                self.controller_rate = Some(Rate::new(period, now));
            }
            self.next_controller_ns = if !self.controller.is_executing() {
                None
            } else if c.waiting_for_costmap {
                Some(now + COSTMAP_POLL_NS)
            } else {
                self.controller_rate.as_mut().map(|rate| rate.sleep(now))
            };
        }
        out
    }
}
