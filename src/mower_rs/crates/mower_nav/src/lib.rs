//! mower_nav: the navigation action server (port of
//! `mower_mission/navigation/nav_action_server.py`, node name
//! `nav_action_server`, same actions, services, topics and messages).
//!
//! What it guards, in the same order as the Python node: a goal is admitted
//! only with a valid path, a fresh dispatch token, no uncertain previous Nav2
//! task, no other goal, no mission mutation, no manual motion and a healthy
//! pose / GPS / IMU picture; it then waits for the caller's dispatch
//! confirmation, checks bt_navigator is active, navigates to the coverage
//! start and follows every split segment with bounded waits; any cancel or
//! lost response either proves the Nav2 goal terminal or latches an
//! `uncertain` fault that keeps twist_mux locked until it can be proven.
//! The 20 Hz coordinator-lock heartbeat and the health monitor run on their
//! own task; everything shares one state mutex that is never held across an
//! await.
//!
//! `nav_backend` picks what runs the goals: nav2's `navigate_to_pose` /
//! `follow_path` actions (`nav2`, the default) or mower_nav_core's stack in
//! this process (`nav_core`, see [`core_backend`]), behind the same protocol.

pub mod core_backend;
mod geometry;
mod nav2;
mod progress;
mod state;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use futures::StreamExt;
use mower_rs_common::{params, ModuleCtx, ModuleResult};
use r2r::builtin_interfaces::msg::Time;
use r2r::geometry_msgs::msg::{Point, Pose, PoseStamped, TwistStamped};
use r2r::lifecycle_msgs::srv::GetState;
use r2r::mower_interface::action::Waypoint;
use r2r::mower_interface::msg::CoverageProgress;
use r2r::mower_interface::srv::{CancelNavigationDispatch, ConfirmNavigationDispatch, GetCoverageProgress, MissionOperationLock};
use r2r::nav2_msgs::action::{FollowPath, NavigateToPose};
use r2r::nav_msgs::msg::{Odometry, Path};
use r2r::rcl_interfaces::msg::Log;
use r2r::sensor_msgs::msg::{Imu, NavSatFix};
use r2r::std_msgs::msg::{Bool, ColorRGBA};
use r2r::std_srvs::srv::Trigger;
use r2r::visualization_msgs::msg::Marker;
use r2r::{ActionServerGoal, QosProfile};
use tokio::sync::Notify;

use crate::core_backend::{CoreFeedback, CoreParams, GoalHandle, GoalKey, GoalOutcome, NavRuntime, PoseSource, Readiness};
use crate::nav2::{terminal_from_core, terminal_from_status, CancelableGoal, Nav2Tracker, Terminal, TerminalSlot};
use crate::progress::{Checkpoint, Progress, Status as ProgressStatus};
use crate::state::{NavState, SafetyParams, TaskResult};

const CONTROLLER_ID: &str = "FollowPath";
/// Pose spacing of every FollowPath goal (see geometry::densify_path).
const FOLLOW_PATH_MAX_STEP_M: f64 = 0.1;
const GOAL_CHECKER_ID: &str = "general_goal_checker";

struct ExecParams {
    split_tolerance_m: f64,
    max_follow_segment_length_m: f64,
    turn_split_angle_rad: f64,
    turn_split_min_segment_length_m: f64,
    nav2_ready_timeout_s: f64,
    nav2_ready_poll_s: f64,
    nav2_cancel_timeout_s: f64,
    nav2_cancel_poll_s: f64,
    nav2_action_server_timeout_s: f64,
    nav2_goal_response_timeout_s: f64,
    dispatch_confirmation_timeout_s: f64,
    /// Coverage progress checkpoint file; `None` = disabled.
    checkpoint_path: Option<std::path::PathBuf>,
    checkpoint_interval_s: f64,
}

impl ExecParams {
    fn split_params(&self) -> [f64; 4] {
        [self.split_tolerance_m, self.max_follow_segment_length_m, self.turn_split_angle_rad, self.turn_split_min_segment_length_m]
    }

    /// [`progress::path_hash`] of a goal under the current split parameters.
    fn path_hash(&self, zone_id: i32, path: &Path, split_points: &[Pose]) -> String {
        let poses: Vec<[f64; 3]> = path.poses.iter().map(|p| [p.pose.position.x, p.pose.position.y, p.pose.position.z]).collect();
        let splits: Vec<[f64; 2]> = split_points.iter().map(|p| [p.position.x, p.position.y]).collect();
        progress::path_hash(zone_id, self.split_params(), &poses, &splits)
    }
}

fn expand_user(path: &str) -> std::path::PathBuf {
    match path.strip_prefix("~/") {
        Some(rest) => std::path::PathBuf::from(std::env::var("HOME").unwrap_or_else(|_| "/".into())).join(rest),
        None => path.into(),
    }
}

/// Write via a temporary file and rename, so a power cut never leaves half a checkpoint.
fn write_checkpoint(path: &std::path::Path, json: &str) -> std::io::Result<()> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let tmp = path.with_extension("json.tmp");
    std::fs::write(&tmp, json)?;
    std::fs::rename(&tmp, path)
}

struct Ctx {
    logger: String,
    safety: SafetyParams,
    exec: ExecParams,
    state: Mutex<NavState>,
    nav2: Mutex<Nav2Tracker>,
    /// Wakes the execute task on dispatch confirmation or a cancel.
    confirm: Notify,
    nav_operation_active_pub: Mutex<r2r::Publisher<Bool>>,
    safety_stop_pub: Mutex<r2r::Publisher<TwistStamped>>,
    coordinator_lock_pub: Mutex<r2r::Publisher<Bool>>,
    split_path_pub: Mutex<r2r::Publisher<Path>>,
    split_points_pub: Mutex<r2r::Publisher<Marker>>,
    progress: Mutex<ProgressState>,
    progress_pub: Mutex<r2r::Publisher<CoverageProgress>>,
    backend: Backend,
    /// `run()` has returned: the periodic tasks end, so this context (and a
    /// nav_core runtime with it) is dropped before a restarted module builds
    /// its own.
    exiting: AtomicBool,
}

/// What runs the goals (`nav_backend`).
enum Backend {
    /// nav2's actions; `bt_navigator/get_state` is the readiness check.
    Nav2 {
        navigate_client: r2r::ActionClient<NavigateToPose::Action>,
        follow_client: r2r::ActionClient<FollowPath::Action>,
        state_client: r2r::Client<GetState::Service>,
    },
    /// mower_nav_core's NavStack on its own thread in this process.
    Core(Arc<NavRuntime>),
}

/// Minimum spacing of feedback-driven `/coverage_progress` messages; state
/// transitions publish immediately.
const PROGRESS_FEEDBACK_PERIOD: Duration = Duration::from_millis(500);

/// nav_core: a task whose runtime thread has not completed a tick for this
/// long is stalled (the thread ticks at least every 10 ms while a goal runs).
/// The drive is safe either way, since `/nav_cmd_vel` comes from that thread
/// and twist_mux (0.5 s) and the final guard (0.2 s) zero a silent input;
/// this keeps the task from waiting forever on a result that cannot come.
const CORE_STALL_TIMEOUT: Duration = Duration::from_secs(1);

/// How long the runtime has not ticked, once both that and the task's own
/// age exceed `limit`. The thread sleeps while idle, so right after a goal is
/// sent its last tick can be old without anything being wrong.
fn core_stall_age(task_age: Duration, tick_age: Duration, limit: Duration) -> Option<Duration> {
    (task_age > limit && tick_age > limit).then_some(tick_age)
}

/// A goal for a runtime whose thread has ended would never be taken up: a
/// definite rejection instead (and `run()` restarts the module).
fn core_running(runtime: &NavRuntime) -> Result<(), String> {
    runtime.stopped().map_or(Ok(()), |cause| Err(format!("in-process navigation stopped ({cause})")))
}

#[derive(Default)]
struct ProgressState {
    progress: Progress<PoseStamped>,
    /// Latest `/adapter/robot_pose`, reported as `current_pose`.
    robot_pose: PoseStamped,
    last_published: Option<Instant>,
    /// The checkpoint on disk (loaded at start, then the last one written).
    checkpoint: Option<Checkpoint>,
    last_checkpoint_write: Option<Instant>,
}

impl ProgressState {
    /// The checkpoint as a message, for `/coverage_progress_status`.
    fn checkpoint_msg(&self) -> CoverageProgress {
        match &self.checkpoint {
            None => {
                let mut msg = CoverageProgress::default();
                msg.zone_id = -1;
                msg.message = "No coverage checkpoint".into();
                msg
            }
            Some(c) => {
                let restored = ProgressState { progress: c.restored_progress(), checkpoint: Some(c.clone()), ..Default::default() };
                let mut msg = restored.to_msg();
                msg.status_text = c.status.clone();
                msg.message = if c.resumable() {
                    format!("Resumable at segment {}/{}", c.resume_segment_index(), c.total_segments)
                } else {
                    "Checkpoint is not resumable".into()
                };
                msg
            }
        }
    }

    fn to_msg(&self) -> CoverageProgress {
        let p = &self.progress;
        let mut msg = CoverageProgress::default();
        msg.header.stamp = stamp_now();
        msg.header.frame_id = "map".into();
        msg.status = p.status as u8;
        msg.status_text = p.status.text().into();
        msg.mission_id = p.mission_id.clone();
        msg.zone_id = p.zone_id;
        msg.current_segment_index = p.current_segment_index;
        msg.total_segments = p.total_segments();
        msg.completed_segments = p.completed_segments;
        msg.current_segment_progress = p.current_segment_progress as f32;
        msg.overall_progress = p.overall_progress as f32;
        msg.current_segment_distance_m = p.current_segment_distance_m() as f32;
        msg.completed_distance_m = p.completed_distance_m as f32;
        msg.total_distance_m = p.total_distance_m as f32;
        msg.remaining_distance_m = p.remaining_distance_m as f32;
        msg.current_pose = self.robot_pose.clone();
        msg.current_segment_start = p.current_segment_start.clone();
        msg.current_segment_goal = p.current_segment_goal.clone();
        msg.checkpoint_available = !p.path_hash.is_empty()
            && self.checkpoint.as_ref().is_some_and(|c| c.resumable() && c.path_hash == p.path_hash && c.mission_id == p.mission_id);
        msg.message = p.message.clone();
        msg
    }
}

fn wall_ns() -> i64 {
    std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_nanos() as i64).unwrap_or(0)
}

fn stamp_now() -> Time {
    let ns = wall_ns();
    Time { sec: (ns / 1_000_000_000) as i32, nanosec: (ns % 1_000_000_000) as u32 }
}

/// `_source_stamp_age_s`: None for a zero stamp.
fn stamp_age_s(stamp: &Time) -> Option<f64> {
    let ns = stamp.sec as i64 * 1_000_000_000 + stamp.nanosec as i64;
    if ns <= 0 {
        None
    } else {
        Some((wall_ns() - ns) as f64 / 1e9)
    }
}

impl Ctx {
    fn info(&self, m: &str) {
        r2r::log_info!(&self.logger, "{}", m);
    }
    fn warn(&self, m: &str) {
        r2r::log_warn!(&self.logger, "{}", m);
    }
    fn error(&self, m: &str) {
        r2r::log_error!(&self.logger, "{}", m);
    }

    fn publish_nav_operation_active(&self, active: bool) {
        let _ = self.nav_operation_active_pub.lock().unwrap().publish(&Bool { data: active });
    }

    fn publish_safety_zero(&self) {
        let mut stop = TwistStamped::default();
        // A forced zero is also an ordering barrier for the final guard.
        stop.header.stamp = stamp_now();
        stop.header.frame_id = "base_footprint".into();
        let _ = self.safety_stop_pub.lock().unwrap().publish(&stop);
    }

    /// Heartbeat the fail-safe Nav2 lock and publish an immediate zero.
    /// Decision and publish stay linearised under the state lock.
    fn publish_safety_stop_if_needed(&self) {
        let lock_active = {
            let mut st = self.state.lock().unwrap();
            let authorized = st.autonomy_authorized(&self.safety, Instant::now());
            let lock_active = !authorized;
            let _ = self.coordinator_lock_pub.lock().unwrap().publish(&Bool { data: lock_active });
            lock_active
        };
        if lock_active {
            self.publish_safety_zero();
        }
    }

    fn set_nav_state(&self, state: &str, message: &str) {
        let (uncertain, reserved) = {
            let mut st = self.state.lock().unwrap();
            st.nav_state = state.to_string();
            st.last_status_message = message.to_string();
            (st.nav2_task_uncertain, st.goal_reserved)
        };
        self.publish_nav_operation_active(reserved || uncertain);
        if state == "canceling" || state == "uncertain" || uncertain {
            self.publish_safety_zero();
        }
        self.publish_safety_stop_if_needed();
    }

    /// Apply `f` to the coverage progress and publish it (feedback-driven
    /// updates, `force == false`, at most every PROGRESS_FEEDBACK_PERIOD).
    /// A checkpoint is written with every forced update once the plan is
    /// known, and from feedback at most every `checkpoint_interval_s`.
    fn update_progress(&self, force: bool, f: impl FnOnce(&mut Progress<PoseStamped>) -> bool) {
        let (msg, checkpoint) = {
            let mut ps = self.progress.lock().unwrap();
            if !f(&mut ps.progress) {
                return;
            }
            let now = Instant::now();
            if !force && ps.last_published.is_some_and(|t| now.duration_since(t) < PROGRESS_FEEDBACK_PERIOD) {
                return;
            }
            ps.last_published = Some(now);
            let interval = Duration::from_secs_f64(self.exec.checkpoint_interval_s.max(0.0));
            let due = force || ps.last_checkpoint_write.is_none_or(|t| now.duration_since(t) >= interval);
            let checkpoint = if self.exec.checkpoint_path.is_some() && !ps.progress.path_hash.is_empty() && due {
                let unix = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0);
                let c = ps.progress.checkpoint(unix);
                ps.checkpoint = Some(c.clone());
                ps.last_checkpoint_write = Some(now);
                Some(c)
            } else {
                None
            };
            (ps.to_msg(), checkpoint)
        };
        let _ = self.progress_pub.lock().unwrap().publish(&msg);
        if let (Some(c), Some(path)) = (checkpoint, &self.exec.checkpoint_path) {
            if let Err(e) = write_checkpoint(path, &c.to_json()) {
                self.warn(&format!("coverage checkpoint {} not written: {e}", path.display()));
            }
        }
    }

    /// Atomically match a goal, request cancel, and revoke its velocity.
    fn request_navigation_cancel(&self, message: &str, expected_dispatch_id: Option<&str>) -> bool {
        {
            let mut st = self.state.lock().unwrap();
            if !st.goal_reserved {
                return false;
            }
            if let Some(expected) = expected_dispatch_id {
                if st.pending_dispatch_id.as_deref() != Some(expected) {
                    return false;
                }
            }
            st.external_cancel_requested = true;
            st.nav_state = "canceling".into();
            st.last_status_message = message.to_string();
        }
        self.update_progress(true, |p| {
            p.canceling(message);
            true
        });
        self.confirm.notify_waiters();
        self.publish_nav_operation_active(true);
        self.publish_safety_zero();
        self.publish_safety_stop_if_needed();
        true
    }

    /// Latch a motion fault until Nav2 termination is actually observed.
    fn mark_nav2_task_uncertain(&self, message: &str, expected_generation: Option<u64>) -> bool {
        {
            let mut st = self.state.lock().unwrap();
            if let Some(g) = expected_generation {
                if st.nav2_active_generation != Some(g) {
                    return false;
                }
            }
            st.nav2_task_uncertain = true;
            st.nav_state = "uncertain".into();
            st.last_status_message = format!("{message}; Nav2 task termination remains unconfirmed");
        }
        self.publish_nav_operation_active(true);
        self.publish_safety_zero();
        self.publish_safety_stop_if_needed();
        true
    }

    /// Clear the dispatch latch only after a proven terminal result.
    fn mark_nav2_dispatch_terminal(&self, expected_generation: Option<u64>) -> bool {
        let mut st = self.state.lock().unwrap();
        let Some(g) = expected_generation else { return false };
        if st.nav2_active_generation != Some(g) {
            return false;
        }
        st.nav2_dispatch_in_flight_or_active = false;
        st.nav2_active_generation = None;
        st.nav2_goal_correlated = false;
        drop(st);
        self.nav2.lock().unwrap().clear();
        true
    }

    /// Atomically resolve only the matching late rejected dispatch.
    fn resolve_definitive_nav2_rejection(&self, generation: u64) {
        let (was_uncertain, reserved) = {
            let mut st = self.state.lock().unwrap();
            if st.nav2_active_generation != Some(generation) {
                return;
            }
            let was_uncertain = st.nav2_task_uncertain;
            st.nav2_dispatch_in_flight_or_active = false;
            st.nav2_active_generation = None;
            st.nav2_goal_correlated = false;
            if was_uncertain {
                st.nav2_task_uncertain = false;
                st.external_cancel_requested = false;
                if let Some(id) = st.uncertain_dispatch_id.take() {
                    st.terminal_dispatch_ids.insert(id);
                }
                st.nav_state = "idle".into();
                st.last_status_message = "Late Nav2 goal response definitively rejected; ready".into();
            }
            (was_uncertain, st.goal_reserved)
        };
        self.nav2.lock().unwrap().clear();
        if was_uncertain {
            self.publish_nav_operation_active(reserved);
            self.publish_safety_stop_if_needed();
        }
    }

    fn clear_uncertain_nav2_task(&self, result: Option<TaskResult>, source: &str) {
        {
            let mut st = self.state.lock().unwrap();
            if !st.nav2_task_uncertain {
                return;
            }
            st.nav2_task_uncertain = false;
            st.external_cancel_requested = false;
            if let Some(id) = st.uncertain_dispatch_id.take() {
                st.terminal_dispatch_ids.insert(id);
            }
        }
        let result_text = result.map(|r| r.to_string()).unwrap_or_else(|| "None".into());
        let message = format!("Previous Nav2 fault is terminal ({source}, result={result_text}); ready");
        self.set_nav_state("idle", &message);
        self.warn(&message);
    }

    /// Non-blocking snapshot of the correlated Nav2 result.
    fn navigator_terminal_snapshot(&self) -> (bool, Option<TaskResult>) {
        let (generation, unconfirmed, correlated) = {
            let st = self.state.lock().unwrap();
            (st.nav2_active_generation, st.nav2_dispatch_in_flight_or_active, st.nav2_goal_correlated)
        };
        if generation.is_none() || (unconfirmed && !correlated) {
            return (false, None);
        }
        let terminal = self.nav2.lock().unwrap().terminal.clone().and_then(|t| t.get());
        match terminal {
            Some(Ok(t)) => {
                if self.mark_nav2_dispatch_terminal(generation) {
                    (true, Some(t.result))
                } else {
                    (false, None)
                }
            }
            _ => (false, None),
        }
    }

    /// Cancel the current Nav2 goal and await its terminal result, bounded.
    async fn cancel_nav_task_and_wait(&self) -> (bool, Option<TaskResult>) {
        let timeout_s = self.exec.nav2_cancel_timeout_s.max(0.1);
        let poll = Duration::from_secs_f64(self.exec.nav2_cancel_poll_s.max(0.01));
        let deadline = Instant::now() + Duration::from_secs_f64(timeout_s);
        let (generation, unconfirmed, correlated) = {
            let st = self.state.lock().unwrap();
            (st.nav2_active_generation, st.nav2_dispatch_in_flight_or_active, st.nav2_goal_correlated)
        };
        // Never mistake the previous task's completed future for proof that a
        // just-dispatched goal with a lost response is terminal.
        if generation.is_none() || (unconfirmed && !correlated) {
            return (false, None);
        }
        let (goal, slot) = {
            let t = self.nav2.lock().unwrap();
            (t.goal.clone(), t.terminal.clone())
        };
        let check = |slot: &Option<Arc<TerminalSlot>>| slot.as_ref().and_then(|s| s.get());
        match check(&slot) {
            Some(Ok(t)) => {
                return if self.mark_nav2_dispatch_terminal(generation) { (true, Some(t.result)) } else { (false, None) };
            }
            Some(Err(())) => return (false, None),
            None => {}
        }
        let Some(goal) = goal else { return (false, None) };
        let cancel = goal.cancel();
        let remaining = deadline.saturating_duration_since(Instant::now());
        match tokio::time::timeout(remaining, cancel).await {
            Err(_) => return (false, None),
            Ok(Err(e)) => {
                // Rejected / already terminated: only a proven terminal result counts.
                if !e.contains("AlreadyTerminated") && !e.contains("Rejected") {
                    self.error(&format!("Nav2 cancel request failed: {e}"));
                    return (false, None);
                }
            }
            Ok(Ok(())) => {}
        }
        while Instant::now() < deadline {
            match check(&slot) {
                Some(Err(())) => return (false, None),
                Some(Ok(t)) => {
                    return if self.mark_nav2_dispatch_terminal(generation) { (true, Some(t.result)) } else { (false, None) };
                }
                None => tokio::time::sleep(poll.min(deadline.saturating_duration_since(Instant::now()))).await,
            }
        }
        (false, None)
    }

    /// Cancel an accepted mission if pose or GPS health becomes stale.
    fn monitor_navigation_health(&self) {
        let (should_cancel, reason, dispatch_id) = {
            let st = self.state.lock().unwrap();
            let reason = st.navigation_health_block_reason(&self.safety, Instant::now());
            let should_cancel = st.goal_reserved && reason.is_some();
            (should_cancel, reason, st.pending_dispatch_id.clone())
        };
        if should_cancel {
            let message = format!("Navigation safety stop: {}", reason.unwrap_or_default());
            self.request_navigation_cancel(&message, dispatch_id.as_deref());
            let status = self.state.lock().unwrap().last_status_message.clone();
            self.error(&status);
        }
    }

    /// Automatically release the fault only after a terminal observation.
    async fn monitor_uncertain_nav2_task(&self) {
        let (uncertain, retry_cancel, orphan) = {
            let mut st = self.state.lock().unwrap();
            let uncertain = st.nav2_task_uncertain;
            let retry_cancel = uncertain && (st.external_cancel_requested || st.nav2_goal_correlated);
            let orphan = uncertain && st.nav2_dispatch_in_flight_or_active;
            if retry_cancel {
                st.external_cancel_requested = false;
            }
            (uncertain, retry_cancel, orphan)
        };
        if !uncertain {
            return;
        }
        if retry_cancel {
            let (confirmed, result) = self.cancel_nav_task_and_wait().await;
            if confirmed {
                self.clear_uncertain_nav2_task(result, "correlated safety cancel retry");
            } else {
                self.mark_nav2_task_uncertain("Safety cancel could not confirm Nav2 terminal state", None);
            }
            return;
        }
        if orphan && !self.state.lock().unwrap().nav2_goal_correlated {
            // No correlated handle to prove the possibly accepted goal
            // stopped: keep the mux locked until the full stack restarts.
            return;
        }
        let (terminal, result) = self.navigator_terminal_snapshot();
        if terminal {
            self.clear_uncertain_nav2_task(result, "monitor");
        }
    }

    /// Resolves once the nav_core runtime can no longer run goals (never for
    /// nav2). `run()` then returns an error, and mower_rsd restarts the
    /// module with a new runtime: nothing else brings a dead thread back.
    async fn core_stopped(&self) -> &'static str {
        let Backend::Core(runtime) = &self.backend else { return std::future::pending().await };
        let mut tick = tokio::time::interval(Duration::from_millis(500));
        loop {
            tick.tick().await;
            if let Some(cause) = runtime.stopped() {
                return cause;
            }
        }
    }

    /// The "recent Nav2 log" part of a task failure message: nav2's `/rosout`
    /// warnings, or the in-process runtime's own.
    fn recent_nav2_log_summary(&self, since: Instant) -> String {
        match &self.backend {
            Backend::Nav2 { .. } => self.state.lock().unwrap().recent_nav2_log_summary(since),
            Backend::Core(runtime) => state::nav2_log_summary(&runtime.recent_events(since)),
        }
    }

    // ── Nav2 dispatch ────────────────────────────────────────────────────────

    /// `_start_nav2_dispatch` + `_send_nav2_goal_bounded`: Ok(true) accepted
    /// and correlated, Ok(false) definitively rejected, Err = uncertain.
    async fn dispatch_navigate(self: &Arc<Self>, goal: NavigateToPose::Goal) -> Result<bool, String> {
        let client = match &self.backend {
            Backend::Nav2 { navigate_client, .. } => navigate_client.clone(),
            Backend::Core(runtime) => {
                let generation = self.begin_dispatch();
                let handle = core_running(runtime)
                    .and_then(|()| geometry::core_goal_pose(&goal.pose))
                    .map(|pose| runtime.send_navigate(pose));
                return self.await_core_goal(generation, runtime, handle, "Nav to coverage start").await;
            }
        };
        let generation = self.begin_dispatch();
        let available = r2r::Node::is_available(&client).map_err(|e| format!("{e:?}"))?;
        let server_timeout = Duration::from_secs_f64(self.exec.nav2_action_server_timeout_s.max(0.1));
        if tokio::time::timeout(server_timeout, available).await.is_err() {
            self.resolve_definitive_nav2_rejection(generation);
            return Ok(false);
        }
        let fut = adapt_navigate(client.send_goal_request(goal).map_err(|e| format!("Nav2 goal request failed: {e:?}"))?);
        self.await_goal_response(generation, fut, "Nav to coverage start").await
    }

    async fn dispatch_follow(self: &Arc<Self>, goal: FollowPath::Goal) -> Result<bool, String> {
        let client = match &self.backend {
            Backend::Nav2 { follow_client, .. } => follow_client.clone(),
            // the runtime has just the controller and goal checker the goal's ids name
            Backend::Core(runtime) => {
                let generation = self.begin_dispatch();
                let handle = core_running(runtime)
                    .and_then(|()| geometry::core_goal_path(&goal.path))
                    .map(|path| runtime.send_follow_path(path));
                return self.await_core_goal(generation, runtime, handle, "Follow path").await;
            }
        };
        let generation = self.begin_dispatch();
        let available = r2r::Node::is_available(&client).map_err(|e| format!("{e:?}"))?;
        let server_timeout = Duration::from_secs_f64(self.exec.nav2_action_server_timeout_s.max(0.1));
        if tokio::time::timeout(server_timeout, available).await.is_err() {
            self.resolve_definitive_nav2_rejection(generation);
            return Ok(false);
        }
        let fut = adapt_follow(client.send_goal_request(goal).map_err(|e| format!("Nav2 goal request failed: {e:?}"))?);
        self.await_goal_response(generation, fut, "Follow path").await
    }

    /// nav_core: the runtime takes every goal at once, so there is no server
    /// to wait for and the response is immediate; a goal it cannot take (not
    /// in `map`) is a definite rejection.
    async fn await_core_goal(self: &Arc<Self>, generation: u64, runtime: &Arc<NavRuntime>, handle: Result<GoalHandle, String>, label: &str) -> Result<bool, String> {
        match handle {
            Ok(handle) => self.await_goal_response(generation, adapt_core(runtime.clone(), handle), label).await,
            Err(e) => {
                self.error(&format!("{label} not sent to nav_core: {e}"));
                self.resolve_definitive_nav2_rejection(generation);
                Ok(false)
            }
        }
    }

    fn begin_dispatch(&self) -> u64 {
        let mut st = self.state.lock().unwrap();
        st.nav2_dispatch_generation += 1;
        let g = st.nav2_dispatch_generation;
        st.nav2_active_generation = Some(g);
        st.nav2_dispatch_in_flight_or_active = true;
        st.nav2_goal_correlated = false;
        drop(st);
        self.nav2.lock().unwrap().clear();
        g
    }

    /// Bound the acceptance wait; adopt and cancel every late acceptance.
    async fn await_goal_response<G, R, F>(self: &Arc<Self>, generation: u64, fut: F, label: &str) -> Result<bool, String>
    where
        G: CancelableGoal + 'static,
        R: std::future::Future<Output = Result<(r2r::GoalStatus, Terminal), ()>> + Send + 'static,
        F: std::future::Future<Output = Result<(G, R, Option<Box<dyn futures::Stream<Item = (String, Option<f64>)> + Send + Unpin>>), r2r::Error>> + Send + 'static,
    {
        let response_timeout = Duration::from_secs_f64(self.exec.nav2_goal_response_timeout_s.max(0.1));
        let ctx = self.clone();
        let label = label.to_string();
        let boxed: nav2::BoxFuture<_> = Box::pin(fut);
        let mut boxed = boxed;
        match tokio::time::timeout(response_timeout, &mut boxed).await {
            Ok(Ok((goal, result, feedback))) => {
                // Retain the accepted handle first, then correlate.
                let matches = {
                    let mut st = self.state.lock().unwrap();
                    if st.nav2_active_generation == Some(generation) {
                        st.nav2_goal_correlated = true;
                        true
                    } else {
                        false
                    }
                };
                let goal: Arc<dyn CancelableGoal> = Arc::new(goal);
                if !matches {
                    let _ = goal.cancel().await;
                    return Err("Nav2 dispatch generation changed after accept".into());
                }
                let slot = Arc::new(TerminalSlot::default());
                {
                    let mut t = self.nav2.lock().unwrap();
                    t.goal = Some(goal);
                    t.terminal = Some(slot.clone());
                    t.last_feedback = None;
                    t.last_feedback_distance = None;
                }
                tokio::spawn(async move {
                    let outcome = result.await;
                    slot.set(outcome.map(|(_, t)| t));
                });
                if let Some(feedback) = feedback {
                    let mut feedback = feedback;
                    let ctx2 = self.clone();
                    tokio::spawn(async move {
                        while let Some((summary, distance)) = feedback.next().await {
                            let mut t = ctx2.nav2.lock().unwrap();
                            t.last_feedback = Some(summary);
                            t.last_feedback_distance = distance;
                        }
                    });
                }
                Ok(true)
            }
            Ok(Err(_)) => {
                self.resolve_definitive_nav2_rejection(generation);
                Ok(false)
            }
            Err(_) => {
                // Late acceptance: adopt the handle, mark cancel, cancel it.
                tokio::spawn(async move {
                    match boxed.await {
                        Err(_) => ctx.resolve_definitive_nav2_rejection(generation),
                        Ok((goal, result, _)) => {
                            let goal: Arc<dyn CancelableGoal> = Arc::new(goal);
                            let matches = {
                                let mut st = ctx.state.lock().unwrap();
                                if st.nav2_active_generation == Some(generation) {
                                    st.nav2_goal_correlated = true;
                                    st.external_cancel_requested = true;
                                    true
                                } else {
                                    false
                                }
                            };
                            if matches {
                                let slot = Arc::new(TerminalSlot::default());
                                {
                                    let mut t = ctx.nav2.lock().unwrap();
                                    t.goal = Some(goal.clone());
                                    t.terminal = Some(slot.clone());
                                }
                                tokio::spawn(async move {
                                    let outcome = result.await;
                                    slot.set(outcome.map(|(_, t)| t));
                                });
                                ctx.error("Nav2 accepted a goal after the bounded response deadline; correlated cancellation is being tracked");
                            }
                            if let Err(e) = goal.cancel().await {
                                ctx.error(&format!("Late Nav2 cancel dispatch failed: {e}"));
                            }
                        }
                    }
                });
                Err(format!("Nav2 goal response timed out; acceptance is uncertain ({label})"))
            }
        }
    }
}

// ── cancelable goal adapters ─────────────────────────────────────────────────

struct NavigateGoal(r2r::ActionClientGoal<NavigateToPose::Action>);
struct FollowGoal(r2r::ActionClientGoal<FollowPath::Action>);

impl CancelableGoal for NavigateGoal {
    fn cancel(&self) -> nav2::BoxFuture<Result<(), String>> {
        match self.0.cancel() {
            Ok(f) => Box::pin(async move { f.await.map_err(|e| format!("{e:?}")) }),
            Err(e) => Box::pin(async move { Err(format!("{e:?}")) }),
        }
    }
}

impl CancelableGoal for FollowGoal {
    fn cancel(&self) -> nav2::BoxFuture<Result<(), String>> {
        match self.0.cancel() {
            Ok(f) => Box::pin(async move { f.await.map_err(|e| format!("{e:?}")) }),
            Err(e) => Box::pin(async move { Err(format!("{e:?}")) }),
        }
    }
}

type GoalResponse<G> = Result<(G, nav2::BoxFuture<Result<(r2r::GoalStatus, Terminal), ()>>, Option<Box<dyn futures::Stream<Item = (String, Option<f64>)> + Send + Unpin>>), r2r::Error>;

/// Adapt r2r's typed goal future into the type-erased shape await_goal_response takes.
fn adapt_navigate(
    fut: impl std::future::Future<Output = Result<(r2r::ActionClientGoal<NavigateToPose::Action>, impl std::future::Future<Output = Result<(r2r::GoalStatus, NavigateToPose::Result), r2r::Error>> + Send + 'static, impl futures::Stream<Item = NavigateToPose::Feedback> + Send + Unpin + 'static), r2r::Error>> + Send + 'static,
) -> impl std::future::Future<Output = GoalResponse<NavigateGoal>> + Send + 'static {
    async move {
        let (goal, result, feedback) = fut.await?;
        let result: nav2::BoxFuture<Result<(r2r::GoalStatus, Terminal), ()>> = Box::pin(async move {
            match result.await {
                Ok((status, r)) => terminal_from_status(status, r.error_code, r.error_msg.clone()).map(|t| (status, t)),
                Err(_) => Err(()),
            }
        });
        let feedback: Box<dyn futures::Stream<Item = (String, Option<f64>)> + Send + Unpin> = Box::new(feedback.map(|f| (format!("distance_remaining={:.3}", f.distance_remaining), None)));
        Ok((NavigateGoal(goal), result, Some(feedback)))
    }
}

fn adapt_follow(
    fut: impl std::future::Future<Output = Result<(r2r::ActionClientGoal<FollowPath::Action>, impl std::future::Future<Output = Result<(r2r::GoalStatus, FollowPath::Result), r2r::Error>> + Send + 'static, impl futures::Stream<Item = FollowPath::Feedback> + Send + Unpin + 'static), r2r::Error>> + Send + 'static,
) -> impl std::future::Future<Output = GoalResponse<FollowGoal>> + Send + 'static {
    async move {
        let (goal, result, feedback) = fut.await?;
        let result: nav2::BoxFuture<Result<(r2r::GoalStatus, Terminal), ()>> = Box::pin(async move {
            match result.await {
                Ok((status, r)) => terminal_from_status(status, r.error_code, r.error_msg.clone()).map(|t| (status, t)),
                Err(_) => Err(()),
            }
        });
        // FollowPath's distance_to_goal is the path length left; it drives the coverage progress.
        let feedback: Box<dyn futures::Stream<Item = (String, Option<f64>)> + Send + Unpin> = Box::new(
            feedback.map(|f| (format!("distance_to_goal={:.3}, speed={:.3}", f.distance_to_goal, f.speed), Some(f.distance_to_goal as f64))),
        );
        Ok((FollowGoal(goal), result, Some(feedback)))
    }
}

/// A goal on the in-process runtime. Cancel only asks (the runtime takes it
/// up at its next tick): the `Canceled` outcome arrives on the result channel,
/// as does the outcome of a goal that had already ended (`cancel` false), so
/// the callers' bounded waits decide either way.
struct CoreGoal {
    runtime: Arc<NavRuntime>,
    key: GoalKey,
}

impl CancelableGoal for CoreGoal {
    fn cancel(&self) -> nav2::BoxFuture<Result<(), String>> {
        let _ = self.runtime.cancel(self.key);
        Box::pin(std::future::ready(Ok(())))
    }
}

/// A runtime goal in the same type-erased shape; the runtime accepts every
/// goal, so the response is ready at once.
fn adapt_core(runtime: Arc<NavRuntime>, handle: GoalHandle) -> impl std::future::Future<Output = GoalResponse<CoreGoal>> + Send + 'static {
    let GoalHandle { key, result, feedback } = handle;
    std::future::ready(Ok((CoreGoal { runtime, key }, core_result(result), Some(core_feedback(feedback)))))
}

/// The goal's outcome as terminal evidence; a dropped sender (the runtime
/// stopped first) is none.
fn core_result(result: tokio::sync::oneshot::Receiver<GoalOutcome>) -> nav2::BoxFuture<Result<(r2r::GoalStatus, Terminal), ()>> {
    Box::pin(async move { result.await.map_err(|_| ()).and_then(terminal_from_core) })
}

/// The runtime's feedback in the nav2 adapters' text; FollowPath's
/// distance_to_goal drives the coverage progress here too.
fn core_feedback(mut feedback: tokio::sync::mpsc::UnboundedReceiver<CoreFeedback>) -> Box<dyn futures::Stream<Item = (String, Option<f64>)> + Send + Unpin> {
    Box::new(futures::stream::poll_fn(move |cx| feedback.poll_recv(cx)).map(|f| match f {
        CoreFeedback::Navigate { distance_remaining } => (format!("distance_remaining={distance_remaining:.3}"), None),
        CoreFeedback::Follow { distance_to_goal, speed } => (format!("distance_to_goal={distance_to_goal:.3}, speed={speed:.3}"), Some(distance_to_goal as f64)),
    }))
}

// ── the action execution ────────────────────────────────────────────────────

struct Execution {
    ctx: Arc<Ctx>,
    /// r2r's goal handle is Send but not Sync; the mutex makes the executor
    /// task's borrows Send across awaits.
    goal: Mutex<ActionServerGoal<Waypoint::Action>>,
    /// Set once an action-level cancel request for this goal was accepted:
    /// r2r then drops its handle bookkeeping and only `cancel()` can still
    /// deliver a result, and rcl allows the CANCELED transition only there.
    action_cancel_accepted: Arc<AtomicBool>,
    #[allow(dead_code)]
    dispatch_id: String,
}

enum Outcome {
    Succeeded,
    Canceled,
    Aborted,
}

impl Execution {
    /// nav_core only: the runtime thread's stall, see [`CORE_STALL_TIMEOUT`].
    fn core_stall(&self, task_started_at: Instant) -> Option<Duration> {
        let Backend::Core(runtime) = &self.ctx.backend else { return None };
        core_stall_age(task_started_at.elapsed(), runtime.last_tick_age(), CORE_STALL_TIMEOUT)
    }

    fn action_cancel_requested(&self) -> bool {
        self.action_cancel_accepted.load(Ordering::SeqCst)
    }

    fn cancel_requested(&self) -> bool {
        self.action_cancel_requested() || self.ctx.state.lock().unwrap().external_cancel_requested
    }

    /// Terminal transition of the reserved goal (`goal_handle.succeed/abort/
    /// canceled()` in Python). rcl reaches CANCELED only from CANCELING, i.e.
    /// after an accepted action cancel; a cancel that came through
    /// `/cancel_nav2` or a manual command can only end the goal ABORTED (the
    /// Python server does the same), while the navigation state still says
    /// `canceled`. After an accepted action cancel r2r can only deliver the
    /// result through `cancel()`, so that is the fallback for the other two.
    fn terminate(&self, outcome: &Outcome) {
        let mut goal = self.goal.lock().unwrap();
        let result = || Waypoint::Result { success: matches!(outcome, Outcome::Succeeded) };
        let first = match outcome {
            Outcome::Succeeded => goal.succeed(result()),
            Outcome::Aborted => goal.abort(result()),
            Outcome::Canceled if self.action_cancel_requested() => goal.cancel(result()),
            Outcome::Canceled => goal.abort(result()),
        };
        if let Err(e) = first {
            if let Err(e2) = goal.cancel(result()) {
                self.ctx.error(&format!("navigation goal could not be terminated: {e:?} / {e2:?}"));
            }
        }
    }

    fn abort(&mut self, message: &str) -> Outcome {
        self.ctx.error(message);
        self.terminate(&Outcome::Aborted);
        self.ctx.set_nav_state("failed", message);
        Outcome::Aborted
    }

    fn canceled_before_task(&mut self) -> Outcome {
        self.terminate(&Outcome::Canceled);
        self.ctx.set_nav_state("canceled", "Navigation canceled before task start");
        Outcome::Canceled
    }

    /// Keep Nav2 locked until the action caller confirms goal receipt.
    async fn wait_for_dispatch_confirmation(&mut self) -> Option<Outcome> {
        let timeout = Duration::from_secs_f64(self.ctx.exec.dispatch_confirmation_timeout_s.max(0.1));
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            let remaining = deadline.saturating_duration_since(Instant::now());
            let _ = tokio::time::timeout(remaining.min(Duration::from_millis(50)), self.ctx.confirm.notified()).await;
            let (confirmed, canceled) = {
                let st = self.ctx.state.lock().unwrap();
                (st.dispatch_confirmed, st.external_cancel_requested)
            };
            if canceled || self.action_cancel_requested() {
                return Some(self.canceled_before_task());
            }
            if confirmed {
                self.ctx.set_nav_state("starting", "Navigation dispatch confirmed; preparing Nav2 task");
                return None;
            }
        }
        Some(self.abort("Navigation dispatch confirmation timed out; Nav2 was not started"))
    }

    /// Wait boundedly for bt_navigator while honouring cancellation.
    async fn wait_for_nav2_ready(&self) -> (bool, String) {
        let timeout_s = self.ctx.exec.nav2_ready_timeout_s.max(0.1);
        let poll = Duration::from_secs_f64(self.ctx.exec.nav2_ready_poll_s.max(0.05));
        let deadline = Instant::now() + Duration::from_secs_f64(timeout_s);
        let state_client = match &self.ctx.backend {
            Backend::Nav2 { state_client, .. } => state_client,
            Backend::Core(runtime) => return self.wait_for_core_ready(runtime, timeout_s, poll, deadline).await,
        };
        let mut last_state = "unavailable".to_string();
        while Instant::now() < deadline {
            if self.cancel_requested() {
                return (false, "canceled".into());
            }
            let remaining = deadline.saturating_duration_since(Instant::now());
            let available = match r2r::Node::is_available(state_client) {
                Ok(f) => tokio::time::timeout(remaining.min(poll), f).await.is_ok(),
                Err(_) => false,
            };
            if !available {
                continue;
            }
            let Ok(fut) = state_client.request(&GetState::Request::default()) else { continue };
            let mut fut = Box::pin(fut);
            let mut answered = None;
            while Instant::now() < deadline {
                if self.cancel_requested() {
                    return (false, "canceled".into());
                }
                let step = poll.min(deadline.saturating_duration_since(Instant::now()));
                match tokio::time::timeout(step, &mut fut).await {
                    Ok(r) => {
                        answered = Some(r);
                        break;
                    }
                    Err(_) => continue,
                }
            }
            let Some(answered) = answered else { break };
            last_state = match answered {
                Ok(resp) => {
                    if resp.current_state.label.is_empty() {
                        "unknown".into()
                    } else {
                        resp.current_state.label
                    }
                }
                Err(e) => format!("error: {e:?}"),
            };
            if last_state.eq_ignore_ascii_case("active") {
                return (true, "Nav2 is active".into());
            }
            tokio::time::sleep(poll.min(deadline.saturating_duration_since(Instant::now()))).await;
        }
        (false, format!("Nav2 readiness timed out after {timeout_s:.1}s (last state: {last_state})"))
    }

    /// nav_core: the same bounded wait for the runtime's inputs (both
    /// costmaps, a pose and odometry) in place of bt_navigator's state.
    async fn wait_for_core_ready(&self, runtime: &NavRuntime, timeout_s: f64, poll: Duration, deadline: Instant) -> (bool, String) {
        let mut readiness = Readiness::default();
        while Instant::now() < deadline {
            if self.cancel_requested() {
                return (false, "canceled".into());
            }
            if let Some(cause) = runtime.stopped() {
                return (false, format!("Nav2 readiness failed: in-process navigation stopped ({cause}); the nav module restarts"));
            }
            readiness = runtime.readiness();
            if readiness.ready() {
                return (true, "Nav2 is active".into());
            }
            tokio::time::sleep(poll.min(deadline.saturating_duration_since(Instant::now()))).await;
        }
        let mut message = format!("Nav2 readiness timed out after {timeout_s:.1}s (last state: inactive); missing: {}", missing_inputs(&readiness));
        let tick_age = runtime.last_tick_age();
        if tick_age > CORE_STALL_TIMEOUT {
            message += &format!("; the nav-core thread has not ticked for {:.1}s", tick_age.as_secs_f64());
        }
        (false, message)
    }

    fn clear_active_task(&self) {
        let mut st = self.ctx.state.lock().unwrap();
        st.active_task_name = None;
    }

    /// Run one dispatched Nav2 task to its terminal result.
    async fn wait_for_nav_task(&mut self, task_name: &str) -> bool {
        let started_at = Instant::now();
        let mut last_feedback: Option<String> = None;
        let (canceled_before_running, reserved, uncertain) = {
            let mut st = self.ctx.state.lock().unwrap();
            st.active_task_name = Some(task_name.to_string());
            st.last_task_name = Some(task_name.to_string());
            let canceled = self.action_cancel_requested() || st.external_cancel_requested || st.nav_state == "canceling";
            if !canceled {
                st.nav_state = "running".into();
                st.last_status_message = format!("Navigation running: {task_name}");
            }
            (canceled, st.goal_reserved, st.nav2_task_uncertain)
        };
        self.ctx.publish_nav_operation_active(reserved || uncertain);
        if canceled_before_running {
            self.ctx.publish_safety_zero();
        }
        self.ctx.publish_safety_stop_if_needed();

        let terminal_generation;
        let terminal: Terminal;
        loop {
            let (generation, slot) = {
                let st = self.ctx.state.lock().unwrap();
                let t = self.ctx.nav2.lock().unwrap();
                (st.nav2_active_generation, t.terminal.clone())
            };
            match slot.as_ref().and_then(|s| s.get()) {
                Some(Err(())) => {
                    let message = format!("{task_name} result channel ended without a proven terminal Nav2 status");
                    self.terminate(&Outcome::Aborted);
                    self.clear_active_task();
                    self.ctx.mark_nav2_task_uncertain(&message, None);
                    return false;
                }
                Some(Ok(t)) => {
                    terminal_generation = generation;
                    terminal = t;
                    break;
                }
                None => {
                    if let Some(age) = self.core_stall(started_at) {
                        // The goal is still correlated: monitor_uncertain_nav2_task
                        // keeps retrying its cancel until the runtime proves it
                        // ended, and the mux stays locked until then.
                        let message = format!("{task_name}: in-process navigation stopped ticking {:.1}s ago", age.as_secs_f64());
                        self.ctx.error(&message);
                        self.terminate(&Outcome::Aborted);
                        self.clear_active_task();
                        self.ctx.mark_nav2_task_uncertain(&message, None);
                        return false;
                    }
                }
            }
            let external = self.ctx.state.lock().unwrap().external_cancel_requested;
            if self.action_cancel_requested() || external {
                let source = if external { "external service" } else { "action client" };
                self.ctx.warn(&format!("{task_name} 已取消 ({source})"));
                let (confirmed, _) = self.ctx.cancel_nav_task_and_wait().await;
                if !confirmed {
                    let message = format!("{task_name} cancel confirmation timed out; refusing to continue navigation");
                    self.ctx.error(&message);
                    self.terminate(&Outcome::Aborted);
                    self.clear_active_task();
                    self.ctx.mark_nav2_task_uncertain(&message, None);
                    return false;
                }
                self.terminate(&Outcome::Canceled);
                {
                    let mut st = self.ctx.state.lock().unwrap();
                    st.external_cancel_requested = false;
                    st.active_task_name = None;
                }
                self.ctx.set_nav_state("canceled", &format!("{task_name} canceled"));
                return false;
            }
            let (feedback, feedback_distance) = {
                let t = self.ctx.nav2.lock().unwrap();
                (t.last_feedback.clone(), t.last_feedback_distance)
            };
            if let Some(remaining) = feedback_distance {
                self.ctx.update_progress(false, |p| p.update_remaining(remaining));
            }
            if let Some(fb) = feedback {
                if last_feedback.as_deref() != Some(fb.as_str()) {
                    self.ctx.state.lock().unwrap().last_feedback_message = fb.clone();
                    self.ctx.info(&format!("{task_name} 反饋: {fb}"));
                    last_feedback = Some(fb);
                }
            }
            tokio::time::sleep(Duration::from_millis(50)).await;
        }

        let cancel_after_completion = self.ctx.state.lock().unwrap().external_cancel_requested;
        if !self.ctx.mark_nav2_dispatch_terminal(terminal_generation) {
            let message = format!("{task_name} terminal result belonged to a stale Nav2 dispatch generation; refusing to continue");
            self.ctx.error(&message);
            self.terminate(&Outcome::Aborted);
            self.clear_active_task();
            self.ctx.mark_nav2_task_uncertain(&message, None);
            return false;
        }
        if self.action_cancel_requested() || cancel_after_completion {
            self.terminate(&Outcome::Canceled);
            {
                let mut st = self.ctx.state.lock().unwrap();
                st.external_cancel_requested = false;
                st.active_task_name = None;
            }
            self.ctx.set_nav_state("canceled", &format!("{task_name} canceled"));
            return false;
        }
        match terminal.result {
            TaskResult::Succeeded => {
                self.ctx.set_nav_state("starting", &format!("{task_name} completed; preparing the next task"));
                self.clear_active_task();
                true
            }
            TaskResult::Canceled => {
                self.terminate(&Outcome::Canceled);
                {
                    let mut st = self.ctx.state.lock().unwrap();
                    st.external_cancel_requested = false;
                    st.active_task_name = None;
                }
                self.ctx.set_nav_state("canceled", &format!("{task_name} canceled"));
                false
            }
            TaskResult::Failed => {
                let elapsed = started_at.elapsed().as_secs_f64();
                let task_error = format!("Nav2 task error: code={}, msg=\"{}\"", terminal.error_code, terminal.error_msg);
                let feedback_summary = last_feedback.clone().unwrap_or_else(|| "last_feedback=None".into());
                let log_summary = self.ctx.recent_nav2_log_summary(started_at);
                self.ctx.error(&format!("{task_name} 失敗: {}，耗時 {elapsed:.1}s，{feedback_summary}，{task_error}，{log_summary}", terminal.result));
                self.terminate(&Outcome::Aborted);
                self.clear_active_task();
                self.ctx.set_nav_state("failed", &format!("{task_name} failed: {}, {task_error}", terminal.result));
                false
            }
        }
    }

    fn stamp_path_for_execution(path: &mut Path) {
        let frame = geometry::path_frame_id(path).to_string();
        let now = stamp_now();
        path.header.frame_id = frame.clone();
        path.header.stamp = now.clone();
        for pose in &mut path.poses {
            if pose.header.frame_id.is_empty() {
                pose.header.frame_id = frame.clone();
            }
            pose.header.stamp = now.clone();
        }
    }

    fn publish_split_points_marker(&self, points: &[Pose]) {
        let mut marker = Marker::default();
        marker.header.frame_id = "map".into();
        marker.header.stamp = stamp_now();
        marker.ns = "coverage_split_points".into();
        marker.id = 0;
        marker.type_ = 7; // SPHERE_LIST
        marker.action = 0;
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
        marker.color = ColorRGBA { r: 1.0, g: 0.0, b: 0.0, a: 0.8 };
        for p in points {
            marker.points.push(Point { x: p.position.x, y: p.position.y, z: 0.1 });
            marker.colors.push(ColorRGBA { r: 1.0, g: 0.0, b: 0.0, a: 0.8 });
        }
        self.ctx.info(&format!("发布 {} 个分割点到 RViz", points.len()));
        let _ = self.ctx.split_points_pub.lock().unwrap().publish(&marker);
    }

    /// The whole action: `single_path_execute_callback`.
    async fn run(&mut self) -> Outcome {
        if let Some(o) = self.wait_for_dispatch_confirmation().await {
            return o;
        }
        self.ctx.info("執行目標");
        let (mut path, split_points, zone_id, resume_index) = {
            let g = self.goal.lock().unwrap();
            (g.goal.path.clone(), g.goal.coverage_split_points.clone(), g.goal.zone_id, g.goal.resume_segment_index)
        };
        self.ctx.info(&format!("path 長度: {}", path.poses.len()));
        self.ctx.info(&format!("coverage_split_points 長度: {}", split_points.len()));
        if path.poses.is_empty() {
            return self.abort("收到空路徑，取消導航");
        }
        if self.cancel_requested() {
            return self.canceled_before_task();
        }
        self.ctx.info("等待 Nav2 啟用中…");
        let (ready, readiness) = self.wait_for_nav2_ready().await;
        if !ready {
            if readiness == "canceled" && self.cancel_requested() {
                return self.canceled_before_task();
            }
            return self.abort(&readiness);
        }
        self.ctx.info(&readiness);
        if self.cancel_requested() {
            return self.canceled_before_task();
        }
        self.publish_split_points_marker(&split_points);

        let e = &self.ctx.exec;
        let coverage_parameters = [
            ("split_tolerance_m", e.split_tolerance_m),
            ("max_follow_segment_length_m", e.max_follow_segment_length_m),
            ("turn_split_angle_rad", e.turn_split_angle_rad),
            ("turn_split_min_segment_length_m", e.turn_split_min_segment_length_m),
        ];
        for (name, value) in coverage_parameters {
            if !value.is_finite() || value < 0.0 {
                return self.abort(&format!("{name} must be finite and non-negative"));
            }
        }
        let (split_tolerance, max_segment_length, turn_split_angle, turn_split_min_length) =
            (e.split_tolerance_m, e.max_follow_segment_length_m, e.turn_split_angle_rad, e.turn_split_min_segment_length_m);

        let split_paths = geometry::split_path_by_coverage_points(&path, &split_points, split_tolerance);
        let coverage_split_count = split_paths.len();
        let split_paths = geometry::split_paths_by_turn_angle(&split_paths, turn_split_angle, turn_split_min_length);
        let turn_split_count = split_paths.len();
        let mut split_paths = geometry::split_paths_by_max_distance(&split_paths, max_segment_length);
        if let Some(reason) = geometry::coverage_segments_block_reason(&split_paths, 0.15) {
            return self.abort(&reason);
        }
        if resume_index > split_paths.len() as i32 {
            return self.abort(&format!("resume segment {resume_index} does not exist ({} segments)", split_paths.len()));
        }
        let first = resume_index.max(1) as usize;
        let distances = split_paths.iter().map(geometry::path_distance).collect();
        let hash = self.ctx.exec.path_hash(zone_id, &path, &split_points);
        self.ctx.update_progress(true, |p| {
            p.set_segments(distances);
            p.path_hash = hash;
            if resume_index > 0 {
                p.resume_from(resume_index);
            }
            p.message = if resume_index > 0 {
                format!("Resuming: navigating to the start of segment {resume_index}")
            } else {
                "Navigating to the coverage start".into()
            };
            true
        });

        Self::stamp_path_for_execution(&mut path);
        let mut start_pose = path.poses[0].clone();
        if first > 1 {
            self.ctx.info(&format!("從檢查點續割：導航到第 {first} 段起點"));
            start_pose = split_paths[first - 1].poses[0].clone();
            start_pose.header = path.poses[0].header.clone();
        } else {
            self.ctx.info("導航到覆蓋路徑起點");
        }
        let navigate_goal = NavigateToPose::Goal { pose: start_pose, behavior_tree: String::new() };
        match self.ctx.dispatch_navigate(navigate_goal).await {
            Ok(true) => {}
            Ok(false) => return self.abort("Nav2 rejected navigation to the coverage start"),
            Err(message) => return self.fail_uncertain(&message),
        }
        if !self.wait_for_nav_task("Nav to coverage start").await {
            return Outcome::Aborted;
        }
        self.ctx.info(&format!(
            "coverage path 依 split points 切成 {coverage_split_count} 段，再依轉角 >= {turn_split_angle:.2} rad 切成 {turn_split_count} 段，再依最大 {max_segment_length:.1} m 切成 {} 段",
            split_paths.len()
        ));
        let total = split_paths.len();
        for (i, split_path) in split_paths.iter_mut().enumerate().skip(first - 1) {
            let idx = i + 1;
            if self.cancel_requested() {
                return self.canceled_before_task();
            }
            *split_path = geometry::densify_path(split_path, FOLLOW_PATH_MAX_STEP_M);
            Self::stamp_path_for_execution(split_path);
            let _ = self.ctx.split_path_pub.lock().unwrap().publish(split_path);
            let distance = geometry::path_distance(split_path);
            let (start_pose, goal_pose) = (split_path.poses[0].clone(), split_path.poses[split_path.poses.len() - 1].clone());
            self.ctx.update_progress(true, |p| {
                p.start_segment(idx as i32, start_pose, goal_pose);
                true
            });
            let start = &split_path.poses[0].pose.position;
            let goal_p = &split_path.poses[split_path.poses.len() - 1].pose.position;
            self.ctx.info(&format!(
                "執行第 {idx}/{total} 段 coverage path，共 {} 點，長度 {distance:.3} m，起點 ({:.3}, {:.3})，終點 ({:.3}, {:.3})",
                split_path.poses.len(),
                start.x,
                start.y,
                goal_p.x,
                goal_p.y
            ));
            let follow_goal = FollowPath::Goal {
                path: split_path.clone(),
                controller_id: CONTROLLER_ID.into(),
                goal_checker_id: GOAL_CHECKER_ID.into(),
                progress_checker_id: String::new(),
            };
            match self.ctx.dispatch_follow(follow_goal).await {
                Ok(true) => {}
                Ok(false) => return self.abort(&format!("Nav2 rejected coverage segment {idx}")),
                Err(message) => return self.fail_uncertain(&message),
            }
            if !self.wait_for_nav_task(&format!("Follow coverage segment {idx}")).await {
                return Outcome::Aborted;
            }
            self.ctx.update_progress(true, |p| {
                p.complete_segment();
                true
            });
        }
        if self.cancel_requested() {
            return self.canceled_before_task();
        }
        self.terminate(&Outcome::Succeeded);
        self.clear_active_task();
        self.ctx.set_nav_state("completed", "Coverage navigation completed");
        Outcome::Succeeded
    }

    /// `_run_reserved_goal`'s exception path: a dispatch whose outcome is
    /// unknown latches the fault, otherwise it is a plain failure.
    fn fail_uncertain(&mut self, message: &str) -> Outcome {
        self.ctx.error(&format!("Unhandled navigation action error: {message}"));
        self.terminate(&Outcome::Aborted);
        let (generation, unconfirmed, correlated) = {
            let st = self.ctx.state.lock().unwrap();
            (st.nav2_active_generation, st.nav2_dispatch_in_flight_or_active, st.nav2_goal_correlated)
        };
        let (terminal, _) = self.ctx.navigator_terminal_snapshot();
        if ((generation.is_some() && unconfirmed) || correlated) && !terminal {
            if !self.ctx.mark_nav2_task_uncertain(&format!("Navigation action crashed: {message}"), generation) {
                self.ctx.set_nav_state("failed", &format!("Navigation failed after terminal dispatch: {message}"));
            }
        } else {
            self.ctx.set_nav_state("failed", &format!("Navigation failed: {message}"));
        }
        Outcome::Aborted
    }
}

/// What a runtime that is not ready still lacks.
fn missing_inputs(r: &Readiness) -> String {
    let inputs = [(r.global_map, "global map"), (r.local_map, "local map"), (r.pose, "robot pose"), (r.odometry, "odometry")];
    inputs.iter().filter(|(present, _)| !present).map(|(_, name)| *name).collect::<Vec<_>>().join(", ")
}

/// Goal admission (`_goal_callback`) + the reserved-goal bookkeeping around
/// one execution (`_run_reserved_goal`).
async fn handle_goal_request(ctx: Arc<Ctx>, req: r2r::ActionServerGoalRequest<Waypoint::Action>) {
    let goal = &req.goal;
    let reject = |ctx: &Ctx, why: &str, req: r2r::ActionServerGoalRequest<Waypoint::Action>| {
        ctx.warn(&format!("Rejecting navigation goal: {why}"));
        let _ = req.reject();
    };
    if let Some(reason) = geometry::navigation_path_block_reason(&goal.path) {
        return reject(&ctx, &reason, req);
    }
    if !goal.coverage_split_points.iter().all(|p| p.position.x.is_finite() && p.position.y.is_finite() && p.position.z.is_finite()) {
        ctx.warn("Rejecting navigation goal with non-finite split points");
        let _ = req.reject();
        return;
    }
    let Some(dispatch_id) = geometry::canonical_dispatch_id(&goal.dispatch_id) else {
        ctx.warn("Rejecting navigation goal without a valid dispatch_id");
        let _ = req.reject();
        return;
    };
    if goal.resume_segment_index != 0 {
        let hash = ctx.exec.path_hash(goal.zone_id, &goal.path, &goal.coverage_split_points);
        let reason = match &ctx.progress.lock().unwrap().checkpoint {
            None => Some("no coverage checkpoint to resume".to_string()),
            Some(c) => c.resume_block_reason(&hash, goal.resume_segment_index),
        };
        if let Some(reason) = reason {
            return reject(&ctx, &format!("cannot resume: {reason}"), req);
        }
    }
    {
        let mut st = ctx.state.lock().unwrap();
        if let Some(reason) = st.navigation_admission_block_reason(&ctx.safety, Instant::now()) {
            drop(st);
            return reject(&ctx, &reason, req);
        }
        if st.seen_dispatch_ids.contains(&dispatch_id) {
            drop(st);
            ctx.warn("Rejecting navigation goal with a reused dispatch_id");
            let _ = req.reject();
            return;
        }
        st.seen_dispatch_ids.insert(dispatch_id.clone());
        st.goal_reserved = true;
        st.external_cancel_requested = false;
        st.dispatch_confirmed = false;
        st.pending_dispatch_id = Some(dispatch_id.clone());
        st.last_task_name = Some("Navigation goal".into());
        st.last_feedback_message = "last_feedback=None".into();
        st.nav_state = "pending_confirmation".into();
        st.last_status_message = "Navigation goal accepted; waiting for dispatch confirmation".into();
    }
    ctx.publish_nav_operation_active(true);
    let zone_id = req.goal.zone_id;
    let (goal_handle, mut cancel_requests) = match req.accept() {
        Ok(v) => v,
        Err(e) => {
            ctx.error(&format!("accepting navigation goal failed: {e:?}"));
            finish_reserved_goal(&ctx);
            return;
        }
    };
    // `_cancel_callback`: accept a cancel only for the reserved dispatch.
    let action_cancel_accepted = Arc::new(AtomicBool::new(false));
    {
        let ctx = ctx.clone();
        let dispatch_id = dispatch_id.clone();
        let accepted = action_cancel_accepted.clone();
        tokio::spawn(async move {
            while let Some(request) = cancel_requests.next().await {
                if ctx.request_navigation_cancel("Navigation action cancel requested", Some(&dispatch_id)) {
                    ctx.warn("Navigation action cancel requested");
                    accepted.store(true, Ordering::SeqCst);
                    request.accept();
                } else {
                    request.reject();
                }
            }
        });
    }
    ctx.update_progress(true, |p| {
        p.begin(&dispatch_id, zone_id);
        true
    });
    let mut execution = Execution { ctx: ctx.clone(), goal: Mutex::new(goal_handle), action_cancel_accepted, dispatch_id };
    let outcome = execution.run().await;
    finish_progress(&ctx, &outcome);
    finish_reserved_goal(&ctx);
}

/// Final `/coverage_progress` of an execution. A cancel during a Nav2 task
/// makes run() return `Aborted` too, so the navigation state decides.
fn finish_progress(ctx: &Ctx, outcome: &Outcome) {
    let (nav_state, message) = {
        let st = ctx.state.lock().unwrap();
        (st.nav_state.clone(), st.last_status_message.clone())
    };
    let status = match outcome {
        Outcome::Succeeded => ProgressStatus::Succeeded,
        Outcome::Canceled => ProgressStatus::Canceled,
        Outcome::Aborted if nav_state == "canceled" => ProgressStatus::Canceled,
        Outcome::Aborted => ProgressStatus::Failed,
    };
    ctx.update_progress(true, |p| {
        p.finish(status, &message);
        true
    });
}

fn finish_reserved_goal(ctx: &Ctx) {
    let uncertain = {
        let mut st = ctx.state.lock().unwrap();
        let finished = st.pending_dispatch_id.take();
        st.goal_reserved = false;
        st.active_task_name = None;
        st.external_cancel_requested = false;
        st.dispatch_confirmed = false;
        let uncertain = st.nav2_task_uncertain;
        if let Some(id) = finished {
            if uncertain {
                st.uncertain_dispatch_id = Some(id);
            } else {
                st.terminal_dispatch_ids.insert(id);
            }
        }
        uncertain
    };
    ctx.publish_nav_operation_active(uncertain);
}

fn latched() -> QosProfile {
    QosProfile::default().keep_last(1).reliable().transient_local()
}

/// `nav_core.*`: what the in-process runtime reads and publishes.
fn core_params(node: &r2r::Node) -> Result<CoreParams, String> {
    let d = CoreParams::default();
    Ok(CoreParams {
        pose_source: pose_source(&params::string(node, "nav_core.pose_source", "tfbus"))?,
        odom_topic: params::string(node, "nav_core.odom_topic", &d.odom_topic),
        map_topic: params::string(node, "nav_core.map_topic", &d.map_topic),
        global_map_topic: params::string(node, "nav_core.global_map_topic", &d.global_map_topic),
        cmd_vel_topic: params::string(node, "nav_core.cmd_vel_topic", &d.cmd_vel_topic),
        plan_topic: params::string(node, "nav_core.plan_topic", &d.plan_topic),
        map_frame: params::string(node, "nav_core.map_frame", &d.map_frame),
        odom_frame: params::string(node, "nav_core.odom_frame", &d.odom_frame),
        base_frame: params::string(node, "nav_core.base_frame", &d.base_frame),
        odom_timeout_s: params::f64(node, "nav_core.odom_timeout_s", d.odom_timeout_s),
    })
}

/// `nav_core.pose_source`: `tfbus` (mower_localize in this process) or `tf`.
fn pose_source(value: &str) -> Result<PoseSource, String> {
    match value {
        "tfbus" => Ok(PoseSource::TfBus),
        "tf" => Ok(PoseSource::TfTopic),
        other => Err(format!("Unknown nav_core.pose_source \"{other}\"; expected \"tfbus\" or \"tf\"")),
    }
}

/// The `nav_action_server` node. Runs either as the `mower_nav` binary or as one
/// module of `mower_rsd`; the node name, namespace, parameters and every
/// topic / service / action name are the same either way.
pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let mut node = r2r::Node::create(ctx, &m.node_name, &m.namespace)?;
    let logger = node.logger().to_string();
    r2r::log_info!(&logger, "NavActionServer initialized (mower_rs)");

    let safety = SafetyParams {
        manual_command_hold_s: params::f64(&node, "manual_command_hold_s", 0.75),
        require_navigation_health: params::bool(&node, "require_navigation_health", true),
        navigation_health_timeout_s: params::f64(&node, "navigation_health_timeout_s", 0.30),
        navigation_health_max_future_skew_s: params::f64(&node, "navigation_health_max_future_skew_s", 0.5),
        max_gps_horizontal_sigma_m: params::f64(&node, "max_gps_horizontal_sigma_m", 0.015),
        max_imu_orientation_sigma_rad: params::f64(&node, "max_imu_orientation_sigma_rad", 0.35),
        max_imu_angular_velocity_sigma_rad_s: params::f64(&node, "max_imu_angular_velocity_sigma_rad_s", 0.10),
        max_imu_linear_acceleration_sigma_m_s2: params::f64(&node, "max_imu_linear_acceleration_sigma_m_s2", 0.50),
    };
    let exec = ExecParams {
        split_tolerance_m: params::f64(&node, "split_tolerance_m", 0.1),
        max_follow_segment_length_m: params::f64(&node, "max_follow_segment_length_m", 0.0),
        turn_split_angle_rad: params::f64(&node, "turn_split_angle_rad", 0.8),
        turn_split_min_segment_length_m: params::f64(&node, "turn_split_min_segment_length_m", 0.25),
        nav2_ready_timeout_s: params::f64(&node, "nav2_ready_timeout_s", 30.0),
        nav2_ready_poll_s: params::f64(&node, "nav2_ready_poll_s", 0.2),
        nav2_cancel_timeout_s: params::f64(&node, "nav2_cancel_timeout_s", 3.0),
        nav2_cancel_poll_s: params::f64(&node, "nav2_cancel_poll_s", 0.05),
        nav2_action_server_timeout_s: params::f64(&node, "nav2_action_server_timeout_s", 3.0),
        nav2_goal_response_timeout_s: params::f64(&node, "nav2_goal_response_timeout_s", 3.0),
        dispatch_confirmation_timeout_s: params::f64(&node, "dispatch_confirmation_timeout_s", 5.0),
        checkpoint_path: params::bool(&node, "progress_checkpoint_enabled", true)
            .then(|| expand_user(&params::string(&node, "progress_checkpoint_path", "~/.ros/mower_mission/coverage_progress.json"))),
        checkpoint_interval_s: params::f64(&node, "progress_checkpoint_interval_sec", 1.0),
    };
    let restored = exec.checkpoint_path.as_ref().and_then(|path| {
        let c = Checkpoint::from_json(&std::fs::read_to_string(path).ok()?);
        if c.is_none() {
            r2r::log_warn!(&logger, "ignoring unreadable coverage checkpoint {}", path.display());
        }
        c
    });
    if let Some(c) = &restored {
        r2r::log_info!(&logger, "coverage checkpoint: mission {} zone {} {} ({}/{} segments){}", c.mission_id, c.zone_id, c.status, c.completed_segments, c.total_segments, if c.resumable() { ", resumable" } else { "" });
    }
    let gps_fix_topic = {
        let t = params::string(&node, "gps_fix_topic", "/fix");
        if t.trim().is_empty() { "/fix".to_string() } else { t.trim().to_string() }
    };
    let gps_odometry_topic = {
        let t = params::string(&node, "gps_odometry_topic", "/odometry/gps");
        if t.trim().is_empty() { "/odometry/gps".to_string() } else { t.trim().to_string() }
    };
    let imu_topic = {
        let t = params::string(&node, "imu_topic", "/imu/data");
        if t.trim().is_empty() { "/imu/data".to_string() } else { t.trim().to_string() }
    };
    let nav_core = match params::string(&node, "nav_backend", "nav2").as_str() {
        "nav2" => None,
        "nav_core" => Some(core_params(&node)?),
        other => return Err(format!("Unknown nav_backend \"{other}\"; expected \"nav2\" or \"nav_core\"").into()),
    };

    let ctx = Arc::new(Ctx {
        logger: logger.clone(),
        safety,
        exec,
        state: Mutex::new(NavState::new()),
        nav2: Mutex::new(Nav2Tracker::default()),
        confirm: Notify::new(),
        nav_operation_active_pub: Mutex::new(node.create_publisher::<Bool>("/nav_operation_active", latched())?),
        safety_stop_pub: Mutex::new(node.create_publisher::<TwistStamped>("/navigation_safety_stop", QosProfile::default())?),
        coordinator_lock_pub: Mutex::new(node.create_publisher::<Bool>("/navigation_coordinator_lock", QosProfile::default())?),
        split_path_pub: Mutex::new(node.create_publisher::<Path>("/split_path", QosProfile::default().keep_last(1))?),
        split_points_pub: Mutex::new(node.create_publisher::<Marker>("/coverage_split_points", QosProfile::default().keep_last(1))?),
        progress: Mutex::new(ProgressState {
            progress: restored.as_ref().map(Checkpoint::restored_progress).unwrap_or_default(),
            checkpoint: restored,
            ..Default::default()
        }),
        progress_pub: Mutex::new(node.create_publisher::<CoverageProgress>("/coverage_progress", latched())?),
        backend: match nav_core {
            None => Backend::Nav2 {
                navigate_client: node.create_action_client::<NavigateToPose::Action>("navigate_to_pose")?,
                follow_client: node.create_action_client::<FollowPath::Action>("follow_path")?,
                state_client: node.create_client::<GetState::Service>("bt_navigator/get_state", QosProfile::services_default())?,
            },
            Some(core) => {
                r2r::log_info!(&logger, "nav_backend nav_core: mower_nav_core in process, pose source {:?}", core.pose_source);
                Backend::Core(NavRuntime::start(&mut node, core, &logger)?)
            }
        },
        exiting: AtomicBool::new(false),
    });
    ctx.publish_nav_operation_active(false);
    ctx.update_progress(true, |_| true);

    // ---- manual sources: manual motion and autonomy are mutually exclusive
    for topic in ["/joy_cmd", "/physical_joy_cmd", "/keyboard_cmd_vel"] {
        let mut stream = node.subscribe::<TwistStamped>(topic, QosProfile::default())?;
        let ctx = ctx.clone();
        tokio::spawn(async move {
            while let Some(msg) = stream.next().await {
                let t = &msg.twist;
                let invalid = ![t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z].iter().all(|v| v.is_finite());
                let cancel_dispatch = {
                    let mut st = ctx.state.lock().unwrap();
                    let hold = Duration::from_secs_f64(ctx.safety.manual_command_hold_s.max(0.5));
                    let deadline = Instant::now() + hold;
                    st.manual_command_deadlines.insert(topic.to_string(), deadline);
                    if invalid {
                        st.invalid_manual_command_deadlines.insert(topic.to_string(), deadline);
                    } else {
                        st.invalid_manual_command_deadlines.remove(topic);
                    }
                    if st.nav2_task_uncertain {
                        st.external_cancel_requested = true;
                    }
                    if st.goal_reserved { st.pending_dispatch_id.clone() } else { None }
                };
                if invalid {
                    ctx.error(&format!("Non-finite manual velocity received on {topic}; forcing safety stop"));
                }
                if let Some(id) = cancel_dispatch {
                    ctx.request_navigation_cancel(&format!("Manual velocity on {topic} requested navigation stop"), Some(&id));
                    let status = ctx.state.lock().unwrap().last_status_message.clone();
                    ctx.warn(&status);
                }
            }
        });
    }
    // ---- health sources ------------------------------------------------------
    {
        let mut stream = node.subscribe::<PoseStamped>("/adapter/robot_pose", QosProfile::default())?;
        let ctx = ctx.clone();
        tokio::spawn(async move {
            while let Some(msg) = stream.next().await {
                let p = &msg.pose;
                let reason = state::robot_pose_block_reason(
                    stamp_age_s(&msg.header.stamp),
                    &msg.header.frame_id,
                    [p.position.x, p.position.y, p.position.z],
                    [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w],
                    &ctx.safety,
                );
                if reason.is_none() {
                    ctx.progress.lock().unwrap().robot_pose = msg.clone();
                }
                let mut st = ctx.state.lock().unwrap();
                st.last_robot_pose_received_at = if reason.is_none() { Some(Instant::now()) } else { None };
                st.robot_pose_rejection_reason = reason;
            }
        });
    }
    {
        let mut stream = node.subscribe::<Imu>(&imu_topic, QosProfile::sensor_data())?;
        let ctx = ctx.clone();
        tokio::spawn(async move {
            while let Some(msg) = stream.next().await {
                let (o, w, a) = (&msg.orientation, &msg.angular_velocity, &msg.linear_acceleration);
                let reason = state::imu_block_reason(
                    stamp_age_s(&msg.header.stamp),
                    &msg.header.frame_id,
                    [o.x, o.y, o.z, o.w],
                    [w.x, w.y, w.z],
                    [a.x, a.y, a.z],
                    &msg.orientation_covariance,
                    &msg.angular_velocity_covariance,
                    &msg.linear_acceleration_covariance,
                    &ctx.safety,
                );
                let mut st = ctx.state.lock().unwrap();
                st.last_imu_received_at = if reason.is_none() { Some(Instant::now()) } else { None };
                st.imu_rejection_reason = reason;
            }
        });
    }
    {
        let mut stream = node.subscribe::<NavSatFix>(&gps_fix_topic, QosProfile::sensor_data())?;
        let ctx = ctx.clone();
        let source = gps_fix_topic.clone();
        tokio::spawn(async move {
            while let Some(msg) = stream.next().await {
                let reason = state::gps_fix_block_reason(
                    stamp_age_s(&msg.header.stamp),
                    &source,
                    msg.status.status,
                    msg.latitude,
                    msg.longitude,
                    &msg.position_covariance,
                    msg.position_covariance_type,
                    &ctx.safety,
                );
                let mut st = ctx.state.lock().unwrap();
                match reason {
                    None => {
                        st.valid_fix_received_at_by_topic.insert(source.clone(), Instant::now());
                        st.fix_rejections_by_topic.remove(&source);
                    }
                    Some(r) => {
                        st.valid_fix_received_at_by_topic.remove(&source);
                        st.fix_rejections_by_topic.insert(source.clone(), r);
                    }
                }
            }
        });
    }
    {
        let mut stream = node.subscribe::<Odometry>(&gps_odometry_topic, QosProfile::default())?;
        let ctx = ctx.clone();
        tokio::spawn(async move {
            while let Some(msg) = stream.next().await {
                let p = &msg.pose.pose.position;
                let reason = state::gps_odometry_block_reason(stamp_age_s(&msg.header.stamp), &msg.header.frame_id, [p.x, p.y, p.z], &msg.pose.covariance, &ctx.safety);
                let mut st = ctx.state.lock().unwrap();
                st.last_gps_odometry_received_at = if reason.is_none() { Some(Instant::now()) } else { None };
                st.gps_odometry_rejection_reason = reason;
            }
        });
    }
    // nav2's warnings for the failure messages (the runtime keeps its own)
    if matches!(ctx.backend, Backend::Nav2 { .. }) {
        let mut stream = node.subscribe::<Log>("/rosout", QosProfile::default().keep_last(100))?;
        let ctx = ctx.clone();
        tokio::spawn(async move {
            while let Some(msg) = stream.next().await {
                ctx.state.lock().unwrap().record_nav2_log(msg.level, &msg.name, &msg.msg, Instant::now());
            }
        });
    }

    // ---- services ------------------------------------------------------------
    for name in ["/cancel_nav2", "/cencel_nav2"] {
        let mut stream = node.create_service::<Trigger::Service>(name, QosProfile::services_default())?;
        let ctx = ctx.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let (nav_state, uncertain, task_name) = {
                    let st = ctx.state.lock().unwrap();
                    (st.nav_state.clone(), st.nav2_task_uncertain, st.active_task_name.clone().or_else(|| st.last_task_name.clone()).unwrap_or_else(|| "Nav2 task".into()))
                };
                let (success, message) = if uncertain {
                    let (confirmed, result) = ctx.cancel_nav_task_and_wait().await;
                    if confirmed {
                        ctx.clear_uncertain_nav2_task(result, "cancel retry");
                        (true, format!("{task_name} is terminal; navigation fault cleared"))
                    } else {
                        let message = format!("{task_name} termination is still unconfirmed; new navigation remains blocked");
                        ctx.mark_nav2_task_uncertain(&message, None);
                        (false, message)
                    }
                } else if !matches!(nav_state.as_str(), "pending_confirmation" | "starting" | "running" | "canceling") {
                    (true, "No active Nav2 task to cancel".to_string())
                } else if !ctx.request_navigation_cancel(&format!("Cancel requested for {task_name}"), None) {
                    (true, "No active Nav2 task to cancel".to_string())
                } else {
                    let m = format!("Cancel requested for {task_name}");
                    ctx.warn(&m);
                    (true, m)
                };
                let _ = req.respond(Trigger::Response { success, message });
            }
        });
    }
    {
        let mut stream = node.create_service::<GetCoverageProgress::Service>("/coverage_progress_status", QosProfile::services_default())?;
        let ctx = ctx.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let (progress, checkpoint) = {
                    let ps = ctx.progress.lock().unwrap();
                    (ps.to_msg(), ps.checkpoint_msg())
                };
                let message = if progress.mission_id.is_empty() { "No coverage execution yet".to_string() } else { progress.status_text.clone() };
                let _ = req.respond(GetCoverageProgress::Response { success: true, message, progress, checkpoint });
            }
        });
    }
    {
        let mut stream = node.create_service::<Trigger::Service>("/check_nav_status", QosProfile::services_default())?;
        let ctx = ctx.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let json = {
                    let mut st = ctx.state.lock().unwrap();
                    let message = if st.nav_state == "running" || st.nav_state == "canceling" {
                        Some(format!("{}; {}", st.last_status_message, st.last_feedback_message))
                    } else {
                        None
                    };
                    st.status_json(&ctx.safety, Instant::now(), message.as_deref())
                };
                let _ = req.respond(Trigger::Response { success: true, message: json });
            }
        });
    }
    {
        let mut stream = node.create_service::<ConfirmNavigationDispatch::Service>("/confirm_navigation_dispatch", QosProfile::services_default())?;
        let ctx = ctx.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let dispatch_id = geometry::canonical_dispatch_id(&req.message.dispatch_id);
                let ok = {
                    let mut st = ctx.state.lock().unwrap();
                    if !st.goal_reserved || st.nav_state != "pending_confirmation" || st.external_cancel_requested || dispatch_id.is_none() || dispatch_id != st.pending_dispatch_id {
                        false
                    } else {
                        st.dispatch_confirmed = true;
                        true
                    }
                };
                if ok {
                    ctx.confirm.notify_waiters();
                }
                let _ = req.respond(ConfirmNavigationDispatch::Response {
                    success: ok,
                    message: if ok { "Navigation dispatch confirmed".into() } else { "No pending navigation dispatch can be confirmed".into() },
                });
            }
        });
    }
    {
        let mut stream = node.create_service::<CancelNavigationDispatch::Service>("/cancel_navigation_dispatch", QosProfile::services_default())?;
        let ctx = ctx.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let mut resp = CancelNavigationDispatch::Response::default();
                let Some(dispatch_id) = geometry::canonical_dispatch_id(&req.message.dispatch_id) else {
                    resp.success = false;
                    resp.message = "Invalid navigation dispatch token".into();
                    let _ = req.respond(resp);
                    continue;
                };
                let (terminal, task_name, matching_uncertain) = {
                    let st = ctx.state.lock().unwrap();
                    (
                        st.terminal_dispatch_ids.contains(&dispatch_id),
                        st.active_task_name.clone().or_else(|| st.last_task_name.clone()).unwrap_or_else(|| "Nav2 task".into()),
                        st.nav2_task_uncertain && st.uncertain_dispatch_id.as_deref() == Some(dispatch_id.as_str()),
                    )
                };
                if terminal {
                    resp.success = true;
                    resp.terminal_confirmed = true;
                    resp.message = "Navigation dispatch is terminal".into();
                } else if matching_uncertain {
                    let (confirmed, result) = ctx.cancel_nav_task_and_wait().await;
                    if confirmed {
                        ctx.clear_uncertain_nav2_task(result, "correlated cancel retry");
                        resp.success = true;
                        resp.terminal_confirmed = true;
                        resp.message = format!("{task_name} is terminal; correlated fault cleared");
                    } else {
                        let message = format!("{task_name} termination is still unconfirmed; correlated navigation remains blocked");
                        ctx.mark_nav2_task_uncertain(&message, None);
                        resp.success = false;
                        resp.message = message;
                    }
                } else if !ctx.request_navigation_cancel(&format!("Correlated cancel requested for {task_name}"), Some(&dispatch_id)) {
                    let terminal = ctx.state.lock().unwrap().terminal_dispatch_ids.contains(&dispatch_id);
                    resp.success = true;
                    resp.terminal_confirmed = terminal;
                    resp.message = if terminal { "Navigation dispatch is terminal".into() } else { "Matching dispatch is not active, but terminal state is not confirmed".into() };
                } else {
                    resp.success = true;
                    resp.terminal_confirmed = false;
                    resp.message = format!("Correlated cancel requested for {task_name}");
                    ctx.warn(&resp.message);
                }
                let _ = req.respond(resp);
            }
        });
    }
    {
        let mut stream = node.create_service::<MissionOperationLock::Service>("/mission_operation_lock", QosProfile::services_default())?;
        let ctx = ctx.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let owner = req.message.owner.trim().to_string();
                let operation = {
                    let o = req.message.operation.trim();
                    if o.is_empty() { "mission mutation".to_string() } else { o.to_string() }
                };
                let (success, message) = if owner.is_empty() {
                    (false, "Mutation lock owner must not be empty".to_string())
                } else {
                    let mut st = ctx.state.lock().unwrap();
                    if req.message.acquire {
                        if st.goal_reserved || st.nav2_task_uncertain || st.manual_motion_active(Instant::now()) {
                            (false, "Navigation/manual motion is active or termination is uncertain".to_string())
                        } else if st.mutation_owner.is_some() {
                            (false, format!("Another mission mutation is active: {}", st.mutation_operation.clone().unwrap_or_else(|| "unknown".into())))
                        } else {
                            st.mutation_owner = Some(owner);
                            st.mutation_operation = Some(operation.clone());
                            (true, format!("Mutation lock acquired for {operation}"))
                        }
                    } else if st.mutation_owner.as_deref() != Some(owner.as_str()) {
                        (false, "Mutation lock is not owned by this requester".to_string())
                    } else {
                        st.mutation_owner = None;
                        st.mutation_operation = None;
                        (true, "Mutation lock released".to_string())
                    }
                };
                let _ = req.respond(MissionOperationLock::Response { success, message });
            }
        });
    }

    // ---- action servers (both names run the same bounded execution) --------
    for name in ["nav_action", "nav_action_follow_path"] {
        let mut requests = node.create_action_server::<Waypoint::Action>(name)?;
        let ctx = ctx.clone();
        tokio::spawn(async move {
            while let Some(req) = requests.next().await {
                let ctx = ctx.clone();
                tokio::spawn(handle_goal_request(ctx, req));
            }
        });
    }

    // ---- 20 Hz health tick + 2 Hz uncertain monitor -------------------------
    {
        let ctx = ctx.clone();
        tokio::spawn(async move {
            // Skip, not Delay: a late tick must not shift the whole schedule
            // (that drifted the heartbeat to ~18.5 Hz under VM jitter), and
            // not Burst: back-to-back publishes buy nothing for a freshness
            // watchdog. This mirrors the Python thread's resync loop.
            let mut tick = tokio::time::interval(Duration::from_millis(50));
            tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
            loop {
                tick.tick().await;
                if ctx.exiting.load(Ordering::Relaxed) {
                    break;
                }
                ctx.monitor_navigation_health();
                ctx.publish_safety_stop_if_needed();
            }
        });
    }
    {
        let ctx = ctx.clone();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_millis(500));
            loop {
                tick.tick().await;
                if ctx.exiting.load(Ordering::Relaxed) {
                    break;
                }
                ctx.monitor_uncertain_nav2_task().await;
            }
        });
    }

    // ---- spin until SIGINT / SIGTERM, or until the nav_core runtime stops --
    let running = Arc::new(AtomicBool::new(true));
    let spin = {
        let running = running.clone();
        tokio::task::spawn_blocking(move || {
            while running.load(Ordering::Relaxed) {
                node.spin_once(Duration::from_millis(20));
            }
            drop(node);
        })
    };
    let stopped = tokio::select! {
        _ = m.shutdown.wait() => None,
        cause = ctx.core_stopped() => Some(cause),
    };
    ctx.exiting.store(true, Ordering::Relaxed);
    running.store(false, Ordering::Relaxed);
    let _ = spin.await;
    match stopped {
        // the node is gone, so twist_mux's lock goes stale (locked) and the
        // action server with it; mower_rsd starts the module again in 2 s
        Some(cause) => Err(format!("nav_core: {cause}; restarting the nav module").into()),
        None => Ok(()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core_backend::GoalStatus as CoreStatus;

    #[test]
    fn nav_core_pose_source_parameter() {
        assert_eq!(pose_source("tfbus"), Ok(PoseSource::TfBus));
        assert_eq!(pose_source("tf"), Ok(PoseSource::TfTopic));
        assert_eq!(pose_source("TF"), Err("Unknown nav_core.pose_source \"TF\"; expected \"tfbus\" or \"tf\"".into()));
    }

    #[test]
    fn core_readiness_timeout_names_the_missing_inputs() {
        assert_eq!(missing_inputs(&Readiness::default()), "global map, local map, robot pose, odometry");
        assert_eq!(missing_inputs(&Readiness { global_map: true, pose: true, ..Default::default() }), "local map, odometry");
    }

    #[test]
    fn core_stall_needs_both_an_old_tick_and_an_old_task() {
        let s = Duration::from_secs_f64;
        let limit = CORE_STALL_TIMEOUT;
        // just sent: the thread may still be waking from its idle sleep
        assert_eq!(core_stall_age(s(0.05), s(30.0), limit), None);
        // running and ticking
        assert_eq!(core_stall_age(s(12.0), s(0.01), limit), None);
        assert_eq!(core_stall_age(s(12.0), s(1.0), limit), None);
        // running, silent for longer than the limit
        assert_eq!(core_stall_age(s(12.0), s(1.2), limit), Some(s(1.2)));
    }

    #[tokio::test]
    async fn core_results_are_terminal_evidence_until_the_runtime_is_gone() {
        let (tx, rx) = tokio::sync::oneshot::channel();
        tx.send(GoalOutcome { status: CoreStatus::Aborted, error_code: 105, error_msg: "Failed to make progress".into() }).unwrap();
        let (status, t) = core_result(rx).await.unwrap();
        assert_eq!(status, r2r::GoalStatus::Aborted);
        assert_eq!((t.result, t.error_code, t.error_msg.as_str()), (TaskResult::Failed, 105, "Failed to make progress"));
        // the runtime was dropped before the goal ended: no proof either way
        let (tx, rx) = tokio::sync::oneshot::channel::<GoalOutcome>();
        drop(tx);
        assert!(core_result(rx).await.is_err());
    }

    #[tokio::test]
    async fn core_feedback_reads_like_the_nav2_adapters() {
        let (tx, rx) = tokio::sync::mpsc::unbounded_channel();
        tx.send(CoreFeedback::Navigate { distance_remaining: 12.34567 }).unwrap();
        tx.send(CoreFeedback::Follow { distance_to_goal: 3.5, speed: 0.2 }).unwrap();
        drop(tx);
        let items: Vec<(String, Option<f64>)> = core_feedback(rx).collect().await;
        assert_eq!(items, [("distance_remaining=12.346".to_string(), None), ("distance_to_goal=3.500, speed=0.200".to_string(), Some(3.5))]);
    }
}
