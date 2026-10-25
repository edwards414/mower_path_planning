//! The navigate_to_pose tree against scripted servers: every plan answers on
//! the next tick, every follow_path goal after a scripted time, so the tick
//! counts, retries, clears and error codes are exact.

use mower_nav_core::action::{GoalId, GoalStatus};
use mower_nav_core::controller_server::{error_code as follow_code, FollowPathResult};
use mower_nav_core::geometry::Pose2D;
use mower_nav_core::navigator::{CostmapId, NavInput, NavServers, NavigateFeedback, NavigateResult, Navigator, NavigatorParams};
use mower_nav_core::path_handler::Path;
use mower_nav_core::planner_server::{error_code as plan_code, ComputePathResult, ComputePathToPose};

const MS: i64 = 1_000_000;
const TICK_MS: i64 = 10;

#[derive(Default)]
struct Fake {
    next: GoalId,
    plans: Vec<(i64, GoalId, ComputePathToPose)>,
    follows: Vec<(i64, GoalId, Path)>,
    canceled_plans: Vec<GoalId>,
    canceled_follows: Vec<GoalId>,
    clears: Vec<CostmapId>,
    now_ms: i64,
}

impl NavServers for Fake {
    fn send_compute_path(&mut self, goal: ComputePathToPose) -> GoalId {
        self.next += 1;
        self.plans.push((self.now_ms, self.next, goal));
        self.next
    }

    fn cancel_compute_path(&mut self, goal: GoalId) {
        self.canceled_plans.push(goal);
    }

    fn send_follow_path(&mut self, path: Path) -> GoalId {
        self.next += 1;
        self.follows.push((self.now_ms, self.next, path));
        self.next
    }

    fn cancel_follow_path(&mut self, goal: GoalId) {
        self.canceled_follows.push(goal);
    }

    fn clear_costmap(&mut self, costmap: CostmapId) {
        self.clears.push(costmap);
    }
}

enum Due {
    Plan(ComputePathResult),
    Follow(FollowPathResult),
}

type PlanScript = Box<dyn FnMut(usize) -> u16>;
/// (ms after the goal, status, error code) of the n-th follow_path goal;
/// `None` never ends.
type FollowScript = Box<dyn FnMut(usize) -> Option<(i64, GoalStatus, u16)>>;

/// The plan every successful planning returns: 0.5 m steps along x.
fn plan_path() -> Path {
    (0..5).map(|i| Pose2D::new(0.5 * i as f64, 0.0, 0.0)).collect()
}

struct Sim {
    nav: Navigator,
    fake: Fake,
    plan: PlanScript,
    follow: FollowScript,
    due: Vec<(i64, GoalId, Due)>,
    open_follow: Option<GoalId>,
    /// How long a plan takes.
    plan_ms: i64,
    seen: (usize, usize),
    t: i64,
    robot: Option<Pose2D>,
    results: Vec<(i64, NavigateResult)>,
    feedback: Vec<(i64, GoalId, NavigateFeedback)>,
}

impl Sim {
    fn new(params: NavigatorParams, plan: PlanScript, follow: FollowScript) -> Self {
        Sim {
            nav: Navigator::new(params),
            fake: Fake::default(),
            plan,
            follow,
            due: Vec::new(),
            open_follow: None,
            plan_ms: TICK_MS,
            seen: (0, 0),
            t: 0,
            robot: Some(Pose2D::new(0.0, 0.0, 0.0)),
            results: Vec::new(),
            feedback: Vec::new(),
        }
    }

    fn mower(plan: PlanScript, follow: FollowScript) -> Self {
        Self::new(NavigatorParams::default(), plan, follow)
    }

    /// Ticks every 10 ms up to and including `end_ms`.
    fn run_until(&mut self, end_ms: i64) {
        while self.t <= end_ms {
            let now = self.t;
            let (ready, later): (Vec<_>, Vec<_>) = self.due.drain(..).partition(|d| d.0 <= now);
            self.due = later;
            for (_, _, d) in ready {
                match d {
                    Due::Plan(r) => self.nav.on_compute_path_result(r),
                    Due::Follow(r) => {
                        if self.open_follow == Some(r.goal) {
                            self.open_follow = None;
                        }
                        self.nav.on_follow_path_result(r)
                    }
                }
            }
            self.fake.now_ms = now;
            if self.nav.has_work() {
                let out = self.nav.tick(&NavInput { now_ns: now * MS, robot_pose: self.robot }, &mut self.fake);
                self.results.extend(out.results.into_iter().map(|r| (now, r)));
                if let Some((g, fb)) = out.feedback {
                    self.feedback.push((now, g, fb));
                }
            }
            while self.seen.0 < self.fake.plans.len() {
                let (_, id, _) = &self.fake.plans[self.seen.0];
                let code = (self.plan)(self.seen.0);
                let status = if code == 0 { GoalStatus::Succeeded } else { GoalStatus::Aborted };
                let path = if code == 0 { plan_path() } else { Path::new() };
                let r = ComputePathResult { goal: *id, status, error_code: code, error: None, path, planning_time_ns: 0 };
                self.due.push((now + self.plan_ms, *id, Due::Plan(r)));
                self.seen.0 += 1;
            }
            while self.seen.1 < self.fake.follows.len() {
                let (_, id, _) = self.fake.follows[self.seen.1];
                // a goal for a busy controller preempts: the old goal is
                // aborted with an empty result at its next pass
                if let Some(old) = self.open_follow.replace(id) {
                    self.due.retain(|d| d.1 != old);
                    let r = FollowPathResult { goal: old, status: GoalStatus::Aborted, error_code: 0, error: None };
                    self.due.push((now + TICK_MS, old, Due::Follow(r)));
                }
                if let Some((after, status, error_code)) = (self.follow)(self.seen.1) {
                    let r = FollowPathResult { goal: id, status, error_code, error: None };
                    self.due.push((now + after, id, Due::Follow(r)));
                }
                self.seen.1 += 1;
            }
            self.t += TICK_MS;
        }
    }

    fn plan_times(&self) -> Vec<i64> {
        self.fake.plans.iter().map(|p| p.0).collect()
    }

    fn follow_times(&self) -> Vec<i64> {
        self.fake.follows.iter().map(|p| p.0).collect()
    }
}

fn always_plan(code: u16) -> PlanScript {
    Box::new(move |_| code)
}

fn follow_never() -> FollowScript {
    Box::new(|_| None)
}

fn follow_all(after: i64, status: GoalStatus, code: u16) -> FollowScript {
    Box::new(move |_| Some((after, status, code)))
}

/// No replanning while a FollowPath runs (a 1000 s period).
fn no_replan() -> NavigatorParams {
    NavigatorParams { replan_frequency: 0.001, ..NavigatorParams::default() }
}

#[test]
fn plans_then_follows_then_succeeds() {
    let mut sim = Sim::mower(always_plan(0), follow_all(500, GoalStatus::Succeeded, 0));
    let goal = sim.nav.send_goal(Pose2D::new(2.0, 0.0, 0.0));
    sim.run_until(1000);
    assert_eq!(sim.plan_times(), vec![0]);
    assert_eq!(sim.fake.plans[0].2.goal, Pose2D::new(2.0, 0.0, 0.0));
    assert_eq!(sim.fake.plans[0].2.planner_id, "GridBased");
    // the plan is seen on the next tick, FollowPath starts in the same tick
    assert_eq!(sim.follow_times(), vec![10]);
    assert_eq!(sim.fake.follows[0].2, plan_path());
    // its result is seen on the tick after it arrived
    assert_eq!(sim.results, vec![(510, NavigateResult { goal, status: GoalStatus::Succeeded, error_code: 0 })]);
    assert!(sim.fake.clears.is_empty());
    assert!(!sim.nav.is_executing());
}

#[test]
fn replans_every_second_and_hands_each_plan_to_the_controller() {
    let mut sim = Sim::mower(always_plan(0), follow_never());
    sim.nav.send_goal(Pose2D::new(2.0, 0.0, 0.0));
    sim.run_until(3500);
    // the 1 Hz timer restarts when a plan is seen, 10 ms after it was asked for
    assert_eq!(sim.plan_times(), vec![0, 1010, 2020, 3030]);
    // the first FollowPath goal, then one per new plan (the preempted goals'
    // aborted results are not the tree's any more and are ignored)
    assert_eq!(sim.follow_times(), vec![10, 1020, 2030, 3040]);
    assert!(sim.results.is_empty());
    let (_, _, fb) = sim.feedback.last().unwrap();
    assert_eq!(fb.number_of_recoveries, 0);
    assert_eq!(fb.navigation_time_ns, 3500 * MS);
}

#[test]
fn an_error_no_recovery_helps_with_aborts_at_once() {
    let mut sim = Sim::mower(always_plan(plan_code::GOAL_OCCUPIED), follow_never());
    let goal = sim.nav.send_goal(Pose2D::new(2.0, 0.0, 0.0));
    sim.run_until(200);
    assert_eq!(sim.plan_times(), vec![0]);
    assert!(sim.fake.follows.is_empty() && sim.fake.clears.is_empty());
    assert_eq!(sim.results, vec![(10, NavigateResult { goal, status: GoalStatus::Aborted, error_code: 206 })]);
}

#[test]
fn no_valid_path_is_retried_after_clearing_until_the_retries_run_out() {
    let mut sim = Sim::mower(always_plan(plan_code::NO_VALID_PATH), follow_never());
    let goal = sim.nav.send_goal(Pose2D::new(2.0, 0.0, 0.0));
    sim.run_until(1000);
    // 7 attempts (6 retries) of two plans each, the second after clearing the
    // global costmap; between attempts both costmaps are cleared
    assert_eq!(sim.fake.plans.len(), 14);
    let mut expected = Vec::new();
    for attempt in 0..7 {
        expected.push(CostmapId::Global);
        if attempt < 6 {
            expected.extend([CostmapId::Local, CostmapId::Global]);
        }
    }
    assert_eq!(sim.fake.clears, expected);
    // the second plan of an attempt goes out on the tick the first failed,
    // and so does the next attempt's first
    assert_eq!(sim.plan_times(), (0..14).map(|i| i * 10).collect::<Vec<_>>());
    assert_eq!(sim.results, vec![(140, NavigateResult { goal, status: GoalStatus::Aborted, error_code: 208 })]);
    // every clear is a recovery
    assert_eq!(sim.feedback.last().unwrap().2.number_of_recoveries, 19);
}

#[test]
fn a_failing_follow_path_is_retried_after_clearing_the_local_costmap() {
    let mut sim = Sim::new(no_replan(), always_plan(0), follow_all(2000, GoalStatus::Aborted, follow_code::PATIENCE_EXCEEDED));
    let goal = sim.nav.send_goal(Pose2D::new(2.0, 0.0, 0.0));
    sim.run_until(40_000);
    // per attempt: plan, FollowPath, its retry after clearing the local
    // costmap, both failing 2 s after they start
    assert_eq!(sim.fake.plans.len(), 7);
    assert_eq!(sim.fake.follows.len(), 14);
    let attempt = 10 + 2000 + 2000;
    assert_eq!(sim.plan_times(), (0..7).map(|i| i * attempt).collect::<Vec<_>>());
    assert_eq!(&sim.follow_times()[..4], &[10, 2010, attempt + 10, attempt + 2010]);
    assert_eq!(sim.fake.clears.len(), 7 + 2 * 6);
    assert_eq!(sim.fake.clears[..3], [CostmapId::Local, CostmapId::Local, CostmapId::Global]);
    assert_eq!(sim.results, vec![(7 * attempt, NavigateResult { goal, status: GoalStatus::Aborted, error_code: 104 })]);
}

#[test]
fn an_old_follow_path_code_keeps_the_recovery_going_and_wins() {
    // attempt 1 plans, then FollowPath fails twice with 104; every later plan
    // fails with GOAL_OCCUPIED, which no recovery helps with, but the stale
    // follow_path_error_code still does
    let plan: PlanScript = Box::new(|n| if n == 0 { 0 } else { plan_code::GOAL_OCCUPIED });
    let mut sim = Sim::mower(plan, follow_all(100, GoalStatus::Aborted, follow_code::PATIENCE_EXCEEDED));
    let goal = sim.nav.send_goal(Pose2D::new(2.0, 0.0, 0.0));
    sim.run_until(2000);
    assert_eq!(sim.fake.plans.len(), 7);
    assert_eq!(sim.fake.follows.len(), 2);
    assert_eq!(sim.fake.clears.len(), 1 + 2 * 6);
    // the smallest nonzero code: 104 beats 206
    let (_, r) = sim.results[0];
    assert_eq!(r, NavigateResult { goal, status: GoalStatus::Aborted, error_code: 104 });
}

#[test]
fn cancel_halts_the_tree_and_cancels_its_goals() {
    let mut sim = Sim::mower(always_plan(0), follow_never());
    let goal = sim.nav.send_goal(Pose2D::new(2.0, 0.0, 0.0));
    sim.run_until(500);
    assert!(sim.nav.cancel_goal(goal));
    sim.run_until(600);
    let follow = sim.fake.follows[0].1;
    assert_eq!(sim.fake.canceled_follows, vec![follow]);
    assert!(sim.fake.canceled_plans.is_empty());
    assert_eq!(sim.results, vec![(510, NavigateResult { goal, status: GoalStatus::Canceled, error_code: 0 })]);
}

#[test]
fn cancel_while_planning_cancels_the_plan() {
    let mut sim = Sim::mower(always_plan(0), follow_never());
    sim.plan_ms = 50;
    let goal = sim.nav.send_goal(Pose2D::new(2.0, 0.0, 0.0));
    sim.run_until(1060); // the second plan was asked for at 1050
    assert_eq!(sim.plan_times(), vec![0, 1050]);
    sim.nav.cancel_goal(goal);
    sim.run_until(1070);
    assert_eq!(sim.fake.canceled_plans, vec![sim.fake.plans[1].1]);
    assert_eq!(sim.fake.canceled_follows, vec![sim.fake.follows[0].1]);
    assert_eq!(sim.results[0].1.status, GoalStatus::Canceled);
}

#[test]
fn a_new_goal_preempts_and_is_planned_to_at_the_next_replan() {
    let mut sim = Sim::mower(always_plan(0), follow_never());
    let a = sim.nav.send_goal(Pose2D::new(2.0, 0.0, 0.0));
    sim.run_until(300);
    let b = sim.nav.send_goal(Pose2D::new(-1.0, 1.0, 0.0));
    sim.run_until(1500);
    // accepted after the next tick (onPreempt runs in the loop callback)
    assert_eq!(sim.results, vec![(310, NavigateResult { goal: a, status: GoalStatus::Aborted, error_code: 0 })]);
    assert_eq!(sim.nav.current_goal(), Some(b));
    // the tree goes on; the new pose is planned to when the timer fires
    assert_eq!(sim.plan_times(), vec![0, 1010]);
    assert_eq!(sim.fake.plans[1].2.goal, Pose2D::new(-1.0, 1.0, 0.0));
    // and the navigation time starts over
    let (t, g, fb) = sim.feedback.last().unwrap();
    assert_eq!((*t, *g), (1500, b));
    assert_eq!(fb.navigation_time_ns, (1500 - 310) * MS);
}

#[test]
fn a_goal_without_a_robot_pose_is_aborted() {
    let mut sim = Sim::mower(always_plan(0), follow_never());
    sim.robot = None;
    let goal = sim.nav.send_goal(Pose2D::new(2.0, 0.0, 0.0));
    sim.run_until(100);
    assert!(sim.fake.plans.is_empty());
    assert_eq!(sim.results, vec![(0, NavigateResult { goal, status: GoalStatus::Aborted, error_code: 0 })]);
}

#[test]
fn feedback_measures_the_plan_from_the_pose_closest_to_the_robot() {
    let mut sim = Sim::mower(always_plan(0), follow_never());
    sim.robot = Some(Pose2D::new(0.9, 0.2, 0.0));
    sim.nav.send_goal(Pose2D::new(2.0, 0.0, 0.0));
    sim.run_until(0);
    // no plan yet
    assert_eq!(sim.feedback[0].2.distance_remaining, 0.0);
    sim.run_until(20);
    // closest is (1.0, 0): 1.0 m of the 2 m plan left
    let fb = sim.feedback.last().unwrap().2;
    assert_eq!(fb.distance_remaining, 1.0);
    assert_eq!(fb.current_pose, Pose2D::new(0.9, 0.2, 0.0));
}

#[test]
fn a_follow_path_canceled_from_outside_counts_as_success() {
    // FollowPath's on_cancelled is SUCCESS: the pipeline completes
    let mut sim = Sim::mower(always_plan(0), follow_all(300, GoalStatus::Canceled, 0));
    let goal = sim.nav.send_goal(Pose2D::new(2.0, 0.0, 0.0));
    sim.run_until(1000);
    assert_eq!(sim.results, vec![(310, NavigateResult { goal, status: GoalStatus::Succeeded, error_code: 0 })]);
}
