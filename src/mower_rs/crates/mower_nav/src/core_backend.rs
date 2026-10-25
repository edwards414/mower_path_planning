//! In-process navigation (docs/ROS_FREE_PLAN.md section 8, D6):
//! mower_nav_core's `NavStack` (navigator + planner server + controller
//! server) and `VelocitySmoother` on a dedicated thread, in place of nav2's
//! `navigate_to_pose` / `follow_path` actions and `bt_navigator/get_state`.
//!
//! Selected with the node parameter `nav_backend: nav_core`
//! (`RUST_NAV_CORE=true`, which also stops nav2 from being launched);
//! `nav_backend: nav2`, the default, keeps the r2r action clients.
//!
//! **This file fixes the interface for the D6 work split.** The bodies are
//! placeholders (`unimplemented!`) until the runtime lands; mower_nav's
//! integration codes against these signatures and the semantics below.
//!
//! What the runtime does (one "nav-core" `std::thread`, not the shared tokio
//! workers):
//!
//! * builds the global costmap from `/map_grid_global` and the 3 x 3 m
//!   rolling local costmap from `/map_grid` (both `nav_msgs/OccupancyGrid`,
//!   reliable + transient_local), keeps them current at 1 Hz / 2 Hz the way
//!   nav2's costmaps are, and applies `StackOutput::clears` (a cleared
//!   costmap is not current until its next update);
//! * reads the robot pose from mower_localize's in-process tfbus
//!   (`PoseSource::TfBus`: `map <- odom` and `odom <- base_footprint`, the
//!   latest of each) or from `/tf` (`PoseSource::TfTopic`, for tests and for
//!   a process without mower_localize), and the twist from `/odom`;
//! * ticks `NavStack` at least every 10 ms (`next_tick_ns`), feeds its
//!   commands to `VelocitySmoother::mower()` and publishes the smoother's
//!   20 Hz output on `/nav_cmd_vel` (`geometry_msgs/TwistStamped`, frame
//!   `base_footprint`, the controller's stamp kept: velocity_command_guard
//!   rejects stale or backward stamps), and `/plan` while navigating;
//! * routes results and feedback to the `GoalHandle` of the goal they belong
//!   to, and keeps a ring of recent warnings for mower_nav's failure messages.

use std::sync::Arc;
use std::time::{Duration, Instant};

pub use mower_nav_core::action::GoalStatus;
use mower_nav_core::geometry::Pose2D;

/// Where the robot pose comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PoseSource {
    /// mower_localize's process-global transform cache (no `/tf` traffic;
    /// requires the localize module in the same process).
    TfBus,
    /// Subscribe to `/tf` and `/tf_static`.
    TfTopic,
}

/// The runtime's parameters (`nav_core.*` on the nav node).
#[derive(Debug, Clone, PartialEq)]
pub struct CoreParams {
    /// `nav_core.pose_source`: `"tfbus"` (default) or `"tf"`.
    pub pose_source: PoseSource,
    pub odom_topic: String,
    pub map_topic: String,
    pub global_map_topic: String,
    pub cmd_vel_topic: String,
    pub plan_topic: String,
    pub map_frame: String,
    pub odom_frame: String,
    pub base_frame: String,
    /// Odometry older than this does not count as present (readiness).
    pub odom_timeout_s: f64,
}

impl Default for CoreParams {
    fn default() -> Self {
        Self {
            pose_source: PoseSource::TfBus,
            odom_topic: "/odom".into(),
            map_topic: "/map_grid".into(),
            global_map_topic: "/map_grid_global".into(),
            cmd_vel_topic: "/nav_cmd_vel".into(),
            plan_topic: "/plan".into(),
            map_frame: "map".into(),
            odom_frame: "odom".into(),
            base_frame: "base_footprint".into(),
            odom_timeout_s: 0.5,
        }
    }
}

/// A goal sent to the runtime. The runtime numbers its goals itself, so
/// sending never waits for the thread.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GoalKey {
    /// `navigate_to_pose`
    Navigate(u64),
    /// `follow_path`
    Follow(u64),
}

/// How a goal ended.
#[derive(Debug, Clone, PartialEq)]
pub struct GoalOutcome {
    pub status: GoalStatus,
    /// nav2's numbering: FollowPath 100–107, ComputePathToPose 200–208, or 0
    /// (success, cancel, and the empty result of a preempted goal).
    pub error_code: u16,
    /// Human-readable cause for a nonzero code (nav2 1.3.13 leaves it empty).
    pub error_msg: String,
}

/// Feedback while a goal runs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CoreFeedback {
    /// NavigateToPose feedback.
    Navigate { distance_remaining: f32 },
    /// FollowPath feedback; `distance_to_goal` is measured in `map`.
    Follow { distance_to_goal: f32, speed: f32 },
}

/// One goal's channels. `result` resolves exactly once; it errors only if the
/// runtime is dropped first (mower_nav then treats the outcome as unproven).
pub struct GoalHandle {
    pub key: GoalKey,
    pub result: tokio::sync::oneshot::Receiver<GoalOutcome>,
    pub feedback: tokio::sync::mpsc::UnboundedReceiver<CoreFeedback>,
}

/// What the runtime has received; `ready()` replaces nav2's
/// `bt_navigator/get_state == active`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Readiness {
    pub global_map: bool,
    pub local_map: bool,
    pub pose: bool,
    pub odometry: bool,
}

impl Readiness {
    pub fn ready(&self) -> bool {
        self.global_map && self.local_map && self.pose && self.odometry
    }
}

/// The running stack. Dropping it stops the thread; goals still running end
/// without an outcome (their `result` receivers error).
pub struct NavRuntime {
    _private: (),
}

impl NavRuntime {
    /// Creates the subscriptions and publishers on `node` and starts the
    /// thread. `logger` is the node's logger name.
    pub fn start(node: &mut r2r::Node, params: CoreParams, logger: &str) -> Result<Arc<Self>, r2r::Error> {
        let _ = (node, params, logger);
        unimplemented!("D6 runtime")
    }

    pub fn readiness(&self) -> Readiness {
        unimplemented!("D6 runtime")
    }

    /// A `navigate_to_pose` goal (pose in `map`). Taken up at the thread's
    /// next tick (≤ 10 ms); every goal is accepted. A goal sent while one
    /// runs preempts it, as bt_navigator does (the old one ends `Aborted`,
    /// code 0).
    pub fn send_navigate(&self, goal: Pose2D) -> GoalHandle {
        let _ = goal;
        unimplemented!("D6 runtime")
    }

    /// A `follow_path` goal straight to the controller server (poses in
    /// `map`, as mower_nav densifies them). A goal sent while one runs
    /// preempts it at the controller's next pass (the old one ends
    /// `Aborted`, code 0), as controller_server does.
    pub fn send_follow_path(&self, path: Vec<Pose2D>) -> GoalHandle {
        let _ = path;
        unimplemented!("D6 runtime")
    }

    /// Asks the goal to cancel. `false`: the key is unknown or the goal has
    /// already ended (its outcome is, or will be, on its `result`). The
    /// `Canceled` outcome arrives on `result` once the server has stopped
    /// (one controller pass, ≤ 50 ms).
    pub fn cancel(&self, key: GoalKey) -> bool {
        let _ = key;
        unimplemented!("D6 runtime")
    }

    /// Warnings since `since`, oldest first (at most the last 40): tolerated
    /// NoValidControl, costmap waits and timeouts, cleared costmaps, planner
    /// overruns, stale inputs. Fills mower_nav's "recent Nav2 log" summary.
    pub fn recent_events(&self, since: Instant) -> Vec<String> {
        let _ = since;
        unimplemented!("D6 runtime")
    }

    /// Time since the thread last completed a tick: a stalled runtime shows
    /// here (mower_nav's cancel timeout then latches "uncertain").
    pub fn last_tick_age(&self) -> Duration {
        unimplemented!("D6 runtime")
    }
}
