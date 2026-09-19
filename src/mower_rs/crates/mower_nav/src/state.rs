//! The coordinator's shared state and its pure decisions (the `_state_lock`
//! part of `NavActionServer`). Everything that decides whether the mower
//! may move lives here so it can be unit tested without ROS.

use std::collections::{HashMap, HashSet, VecDeque};
use std::time::Instant;

use serde_json::json;

use crate::geometry;

#[derive(Debug, Clone)]
pub struct SafetyParams {
    pub manual_command_hold_s: f64,
    pub require_navigation_health: bool,
    pub navigation_health_timeout_s: f64,
    pub navigation_health_max_future_skew_s: f64,
    pub max_gps_horizontal_sigma_m: f64,
    pub max_imu_orientation_sigma_rad: f64,
    pub max_imu_angular_velocity_sigma_rad_s: f64,
    pub max_imu_linear_acceleration_sigma_m_s2: f64,
}

impl SafetyParams {
    pub fn health_timeout_s(&self) -> f64 {
        self.navigation_health_timeout_s.max(0.10)
    }
}

/// Terminal outcome of one Nav2 task (rclpy `TaskResult` names).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TaskResult {
    Succeeded,
    Canceled,
    Failed,
}

impl std::fmt::Display for TaskResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Python printed the enum: TaskResult.SUCCEEDED etc.
        f.write_str(match self {
            TaskResult::Succeeded => "TaskResult.SUCCEEDED",
            TaskResult::Canceled => "TaskResult.CANCELED",
            TaskResult::Failed => "TaskResult.FAILED",
        })
    }
}

#[derive(Default)]
pub struct NavState {
    pub goal_reserved: bool,
    pub active_task_name: Option<String>,
    pub last_task_name: Option<String>,
    pub external_cancel_requested: bool,
    pub dispatch_confirmed: bool,
    pub pending_dispatch_id: Option<String>,
    pub seen_dispatch_ids: HashSet<String>,
    pub terminal_dispatch_ids: HashSet<String>,
    pub uncertain_dispatch_id: Option<String>,
    pub nav_state: String,
    pub last_feedback_message: String,
    pub last_status_message: String,
    /// Latched when an action returns without proving that its Nav2 goal is
    /// terminal.
    pub nav2_task_uncertain: bool,
    /// True from immediately before a Nav2 dispatch until a definite
    /// rejection or terminal result.
    pub nav2_dispatch_in_flight_or_active: bool,
    pub nav2_dispatch_generation: u64,
    pub nav2_active_generation: Option<u64>,
    /// A correlated Nav2 goal exists (cancelable) for the active generation.
    pub nav2_goal_correlated: bool,
    pub mutation_owner: Option<String>,
    pub mutation_operation: Option<String>,
    pub manual_command_deadlines: HashMap<String, Instant>,
    pub invalid_manual_command_deadlines: HashMap<String, Instant>,
    pub valid_fix_received_at_by_topic: HashMap<String, Instant>,
    pub fix_rejections_by_topic: HashMap<String, String>,
    pub last_gps_odometry_received_at: Option<Instant>,
    pub gps_odometry_rejection_reason: Option<String>,
    pub last_robot_pose_received_at: Option<Instant>,
    pub robot_pose_rejection_reason: Option<String>,
    pub last_imu_received_at: Option<Instant>,
    pub imu_rejection_reason: Option<String>,
    pub recent_nav2_logs: VecDeque<(Instant, String, String)>,
}

impl NavState {
    pub fn new() -> Self {
        NavState {
            nav_state: "idle".into(),
            last_feedback_message: "last_feedback=None".into(),
            last_status_message: "Navigation idle".into(),
            ..Default::default()
        }
    }

    fn stale(t: Option<Instant>, now: Instant, timeout_s: f64) -> bool {
        t.map(|t| now.duration_since(t).as_secs_f64() > timeout_s).unwrap_or(true)
    }

    pub fn navigation_health_block_reason(&self, params: &SafetyParams, now: Instant) -> Option<String> {
        if !params.require_navigation_health {
            return None;
        }
        let timeout_s = params.health_timeout_s();
        if Self::stale(self.last_robot_pose_received_at, now, timeout_s) {
            return Some(self.robot_pose_rejection_reason.clone().unwrap_or_else(|| "robot pose/TF is unavailable or stale".into()));
        }
        if self.valid_fix_received_at_by_topic.is_empty() {
            if !self.fix_rejections_by_topic.is_empty() {
                let mut reasons: Vec<&String> = self.fix_rejections_by_topic.values().collect();
                reasons.sort();
                reasons.dedup();
                return Some(reasons.iter().map(|s| s.as_str()).collect::<Vec<_>>().join("; "));
            }
            return Some("GPS fix is unavailable".into());
        }
        let newest = self.valid_fix_received_at_by_topic.values().max().copied();
        if Self::stale(newest, now, timeout_s) {
            return Some("GPS fix is stale".into());
        }
        if Self::stale(self.last_gps_odometry_received_at, now, timeout_s) {
            return Some(self.gps_odometry_rejection_reason.clone().unwrap_or_else(|| "GPS odometry from navsat_transform is unavailable or stale".into()));
        }
        if Self::stale(self.last_imu_received_at, now, timeout_s) {
            return Some(self.imu_rejection_reason.clone().unwrap_or_else(|| "IMU is unavailable or stale".into()));
        }
        None
    }

    /// Whether any muxed manual source is still within its timeout (expired
    /// entries are dropped).
    pub fn manual_motion_active(&mut self, now: Instant) -> bool {
        let expired: Vec<String> = self.manual_command_deadlines.iter().filter(|(_, d)| **d <= now).map(|(k, _)| k.clone()).collect();
        for k in expired {
            self.manual_command_deadlines.remove(&k);
            self.invalid_manual_command_deadlines.remove(&k);
        }
        !self.manual_command_deadlines.is_empty()
    }

    /// The exact reason a new autonomous goal is currently unsafe.
    pub fn navigation_admission_block_reason(&mut self, params: &SafetyParams, now: Instant) -> Option<String> {
        if self.nav2_task_uncertain {
            return Some("previous Nav2 task termination is unconfirmed".into());
        }
        if self.goal_reserved {
            return Some("another navigation goal is active".into());
        }
        if self.mutation_owner.is_some() {
            return Some(format!("mission mutation is active: {}", self.mutation_operation.clone().unwrap_or_else(|| "unknown".into())));
        }
        if self.manual_motion_active(now) {
            return Some("manual velocity command is active".into());
        }
        self.navigation_health_block_reason(params, now)
    }

    /// Whether autonomy is authorised right now (the coordinator lock is the
    /// inverse). twist_mux treats true OR timeout as locked.
    pub fn autonomy_authorized(&mut self, params: &SafetyParams, now: Instant) -> bool {
        self.manual_motion_active(now);
        self.goal_reserved
            && self.nav_state == "running"
            && !self.external_cancel_requested
            && self.mutation_owner.is_none()
            && !self.nav2_task_uncertain
            && self.invalid_manual_command_deadlines.is_empty()
            && self.navigation_health_block_reason(params, now).is_none()
    }

    /// The stable machine-readable /check_nav_status payload.
    pub fn status_json(&mut self, params: &SafetyParams, now: Instant, message: Option<&str>) -> String {
        let block_reason = self.navigation_admission_block_reason(params, now);
        let payload = json!({
            "state": self.nav_state,
            "task": self.active_task_name.clone().or_else(|| self.last_task_name.clone()),
            "message": message.map(str::to_string).unwrap_or_else(|| self.last_status_message.clone()),
            "ready": block_reason.is_none(),
            "block_reason": block_reason,
        });
        serde_json::to_string(&payload).expect("json")
    }

    pub fn recent_nav2_log_summary(&self, since: Instant) -> String {
        let recent: Vec<&(Instant, String, String)> = self.recent_nav2_logs.iter().filter(|(t, _, _)| *t >= since).collect();
        if recent.is_empty() {
            return "recent_nav2_logs=None".into();
        }
        let lines: Vec<String> = recent.iter().rev().take(5).rev().map(|(_, name, msg)| format!("[{name}] {msg}")).collect();
        format!("recent_nav2_logs={}", lines.join(" | "))
    }

    pub fn record_nav2_log(&mut self, level: u8, name: &str, msg: &str, now: Instant) {
        // rcl_interfaces/Log WARN = 30
        if level < 30 {
            return;
        }
        const NAV2_NODES: [&str; 7] = ["controller_server", "planner_server", "bt_navigator", "behavior_server", "velocity_smoother", "local_costmap", "global_costmap"];
        if !NAV2_NODES.iter().any(|n| name.contains(n)) {
            return;
        }
        if self.recent_nav2_logs.len() >= 40 {
            self.recent_nav2_logs.pop_front();
        }
        self.recent_nav2_logs.push_back((now, name.to_string(), msg.to_string()));
    }
}

/// `_source_stamp_block_reason`: `age_s` is (now - stamp) in seconds, None when
/// the stamp is zero / unusable.
pub fn source_stamp_block_reason(age_s: Option<f64>, source: &str, params: &SafetyParams) -> Option<String> {
    let Some(age_s) = age_s.filter(|a| a.is_finite()) else {
        return Some(format!("{source} source timestamp is unavailable"));
    };
    if age_s < -params.navigation_health_max_future_skew_s.max(0.0) {
        return Some(format!("{source} source timestamp is in the future"));
    }
    if age_s > params.health_timeout_s() {
        return Some(format!("{source} source timestamp is stale"));
    }
    None
}

/// `_gps_health_callback` decision.
pub fn gps_fix_block_reason(
    stamp_age_s: Option<f64>,
    source: &str,
    status: i8,
    lat: f64,
    lon: f64,
    covariance: &[f64],
    covariance_type: u8,
    params: &SafetyParams,
) -> Option<String> {
    if let Some(r) = source_stamp_block_reason(stamp_age_s, &format!("GPS {source}"), params) {
        return Some(r);
    }
    // NavSatStatus: FIX 0, SBAS 1, GBAS 2
    let valid_status = matches!(status, 0 | 1 | 2);
    if !valid_status || !lat.is_finite() || !lon.is_finite() || !(-90.0..=90.0).contains(&lat) || !(-180.0..=180.0).contains(&lon) || (lat.abs() < 1e-9 && lon.abs() < 1e-9) {
        return Some("GPS has no valid fix".into());
    }
    // COVARIANCE_TYPE_APPROXIMATED 1, DIAGONAL_KNOWN 2, KNOWN 3
    if !matches!(covariance_type, 1 | 2 | 3) || covariance.len() < 9 {
        return Some("GPS covariance is unknown".into());
    }
    let max_sigma = params.max_gps_horizontal_sigma_m.max(0.0);
    match geometry::horizontal_eigen(covariance[0], covariance[4], covariance[1], covariance[3]) {
        None => Some("GPS horizontal covariance is invalid".into()),
        Some((_, eigen_min)) if eigen_min <= 1e-12 => Some("GPS horizontal covariance is degenerate".into()),
        Some((eigen_max, _)) if eigen_max > max_sigma * max_sigma => Some(format!("GPS horizontal uncertainty exceeds {max_sigma:.3} m")),
        Some(_) => None,
    }
}

/// `_gps_odometry_health_callback` decision.
pub fn gps_odometry_block_reason(stamp_age_s: Option<f64>, frame_id: &str, position: [f64; 3], covariance: &[f64], params: &SafetyParams) -> Option<String> {
    if let Some(r) = source_stamp_block_reason(stamp_age_s, "GPS odometry", params) {
        return Some(r);
    }
    if frame_id != "map" {
        return Some("GPS odometry frame_id must be map".into());
    }
    if !position.iter().all(|v| v.is_finite()) {
        return Some("GPS odometry position is invalid".into());
    }
    if covariance.len() < 36 {
        return Some("GPS odometry covariance is unavailable".into());
    }
    let max_sigma = params.max_gps_horizontal_sigma_m.max(0.0);
    match geometry::horizontal_eigen(covariance[0], covariance[7], covariance[1], covariance[6]) {
        None => Some("GPS odometry covariance is invalid".into()),
        Some((_, eigen_min)) if eigen_min <= 1e-12 => Some("GPS odometry horizontal covariance is degenerate".into()),
        Some((eigen_max, _)) if eigen_max > max_sigma * max_sigma => Some(format!("GPS odometry horizontal uncertainty exceeds {max_sigma:.3} m")),
        Some(_) => None,
    }
}

/// `_imu_health_callback` decision.
pub fn imu_block_reason(
    stamp_age_s: Option<f64>,
    frame_id: &str,
    orientation: [f64; 4],
    angular_velocity: [f64; 3],
    linear_acceleration: [f64; 3],
    orientation_cov: &[f64],
    angular_cov: &[f64],
    linear_cov: &[f64],
    params: &SafetyParams,
) -> Option<String> {
    if let Some(r) = source_stamp_block_reason(stamp_age_s, "IMU", params) {
        return Some(r);
    }
    if frame_id != "imu_link" {
        return Some("IMU frame_id must be imu_link".into());
    }
    if !orientation.iter().chain(angular_velocity.iter()).chain(linear_acceleration.iter()).all(|v| v.is_finite()) {
        return Some("IMU contains non-finite values".into());
    }
    let norm = orientation.iter().map(|v| v * v).sum::<f64>().sqrt();
    if (norm - 1.0).abs() > 1e-2 {
        return Some("IMU orientation quaternion is not normalized".into());
    }
    for (cov, max_sigma, label) in [
        (orientation_cov, params.max_imu_orientation_sigma_rad, "orientation"),
        (angular_cov, params.max_imu_angular_velocity_sigma_rad_s, "angular velocity"),
        (linear_cov, params.max_imu_linear_acceleration_sigma_m_s2, "linear acceleration"),
    ] {
        if let Some(r) = geometry::imu_covariance_block_reason(cov, max_sigma, label) {
            return Some(r);
        }
    }
    None
}

/// `_robot_pose_health_callback` decision.
pub fn robot_pose_block_reason(stamp_age_s: Option<f64>, frame_id: &str, position: [f64; 3], orientation: [f64; 4], params: &SafetyParams) -> Option<String> {
    if frame_id != "map" {
        return Some("robot pose frame must be map".into());
    }
    if !position.iter().chain(orientation.iter()).all(|v| v.is_finite()) {
        return Some("robot pose contains non-finite values".into());
    }
    let norm = orientation.iter().map(|v| v * v).sum::<f64>().sqrt();
    if !norm.is_finite() || (norm - 1.0).abs() > 1e-2 {
        return Some("robot pose quaternion is not normalized".into());
    }
    source_stamp_block_reason(stamp_age_s, "robot pose", params)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn params(health: bool) -> SafetyParams {
        SafetyParams {
            manual_command_hold_s: 0.75,
            require_navigation_health: health,
            navigation_health_timeout_s: 0.30,
            navigation_health_max_future_skew_s: 0.5,
            max_gps_horizontal_sigma_m: 0.015,
            max_imu_orientation_sigma_rad: 0.35,
            max_imu_angular_velocity_sigma_rad_s: 0.10,
            max_imu_linear_acceleration_sigma_m_s2: 0.50,
        }
    }

    #[test]
    fn admission_and_lock_follow_the_python_order() {
        let p = params(false);
        let mut s = NavState::new();
        let now = Instant::now();
        assert_eq!(s.navigation_admission_block_reason(&p, now), None);
        assert!(!s.autonomy_authorized(&p, now));
        s.goal_reserved = true;
        s.nav_state = "running".into();
        assert!(s.autonomy_authorized(&p, now));
        assert_eq!(s.navigation_admission_block_reason(&p, now).as_deref(), Some("another navigation goal is active"));
        s.external_cancel_requested = true;
        assert!(!s.autonomy_authorized(&p, now));
        s.external_cancel_requested = false;
        s.mutation_owner = Some("x".into());
        s.mutation_operation = Some("edit".into());
        assert!(!s.autonomy_authorized(&p, now));
        s.goal_reserved = false;
        assert_eq!(s.navigation_admission_block_reason(&p, now).as_deref(), Some("mission mutation is active: edit"));
        s.mutation_owner = None;
        s.manual_command_deadlines.insert("/joy_cmd".into(), now + Duration::from_millis(500));
        assert_eq!(s.navigation_admission_block_reason(&p, now).as_deref(), Some("manual velocity command is active"));
        assert_eq!(s.navigation_admission_block_reason(&p, now + Duration::from_secs(1)), None);
        s.nav2_task_uncertain = true;
        assert_eq!(s.navigation_admission_block_reason(&p, now).as_deref(), Some("previous Nav2 task termination is unconfirmed"));
        let status: serde_json::Value = serde_json::from_str(&s.status_json(&p, now, None)).unwrap();
        assert_eq!(status["ready"], json!(false));
        assert_eq!(status["state"], json!("running"));
        assert_eq!(status["block_reason"], json!("previous Nav2 task termination is unconfirmed"));
    }

    #[test]
    fn health_gate_reports_the_first_missing_source() {
        let p = params(true);
        let mut s = NavState::new();
        let now = Instant::now();
        assert_eq!(s.navigation_health_block_reason(&p, now).as_deref(), Some("robot pose/TF is unavailable or stale"));
        s.last_robot_pose_received_at = Some(now);
        assert_eq!(s.navigation_health_block_reason(&p, now).as_deref(), Some("GPS fix is unavailable"));
        s.fix_rejections_by_topic.insert("/fix".into(), "GPS has no valid fix".into());
        assert_eq!(s.navigation_health_block_reason(&p, now).as_deref(), Some("GPS has no valid fix"));
        s.fix_rejections_by_topic.clear();
        s.valid_fix_received_at_by_topic.insert("/fix".into(), now - Duration::from_millis(400));
        assert_eq!(s.navigation_health_block_reason(&p, now).as_deref(), Some("GPS fix is stale"));
        s.valid_fix_received_at_by_topic.insert("/fix".into(), now);
        assert_eq!(s.navigation_health_block_reason(&p, now).as_deref(), Some("GPS odometry from navsat_transform is unavailable or stale"));
        s.last_gps_odometry_received_at = Some(now);
        s.imu_rejection_reason = Some("IMU frame_id must be imu_link".into());
        assert_eq!(s.navigation_health_block_reason(&p, now).as_deref(), Some("IMU frame_id must be imu_link"));
        s.imu_rejection_reason = None;
        s.last_imu_received_at = Some(now);
        assert_eq!(s.navigation_health_block_reason(&p, now), None);
        assert_eq!(s.navigation_health_block_reason(&params(false), now), None);
    }

    #[test]
    fn sensor_decisions() {
        let p = params(true);
        assert_eq!(source_stamp_block_reason(None, "IMU", &p).as_deref(), Some("IMU source timestamp is unavailable"));
        assert_eq!(source_stamp_block_reason(Some(-0.6), "IMU", &p).as_deref(), Some("IMU source timestamp is in the future"));
        assert_eq!(source_stamp_block_reason(Some(0.31), "IMU", &p).as_deref(), Some("IMU source timestamp is stale"));
        assert_eq!(source_stamp_block_reason(Some(0.1), "IMU", &p), None);
        let cov = [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0004];
        assert_eq!(gps_fix_block_reason(Some(0.05), "/fix", 2, 25.0, 121.5, &cov, 2, &p), None);
        assert_eq!(gps_fix_block_reason(Some(0.05), "/fix", -1, 25.0, 121.5, &cov, 2, &p).as_deref(), Some("GPS has no valid fix"));
        assert_eq!(gps_fix_block_reason(Some(0.05), "/fix", 2, 0.0, 0.0, &cov, 2, &p).as_deref(), Some("GPS has no valid fix"));
        assert_eq!(gps_fix_block_reason(Some(0.05), "/fix", 2, 25.0, 121.5, &cov, 0, &p).as_deref(), Some("GPS covariance is unknown"));
        let loose = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01];
        assert_eq!(gps_fix_block_reason(Some(0.05), "/fix", 2, 25.0, 121.5, &loose, 2, &p).as_deref(), Some("GPS horizontal uncertainty exceeds 0.015 m"));
        let mut odom_cov = vec![0.0; 36];
        odom_cov[0] = 0.0001;
        odom_cov[7] = 0.0001;
        assert_eq!(gps_odometry_block_reason(Some(0.05), "map", [1.0, 2.0, 0.0], &odom_cov, &p), None);
        assert_eq!(gps_odometry_block_reason(Some(0.05), "odom", [1.0, 2.0, 0.0], &odom_cov, &p).as_deref(), Some("GPS odometry frame_id must be map"));
        let ok_cov = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01];
        assert_eq!(imu_block_reason(Some(0.05), "imu_link", [0.0, 0.0, 0.0, 1.0], [0.0; 3], [0.0, 0.0, 9.8], &ok_cov, &ok_cov, &ok_cov, &p), None);
        assert_eq!(imu_block_reason(Some(0.05), "base_link", [0.0, 0.0, 0.0, 1.0], [0.0; 3], [0.0; 3], &ok_cov, &ok_cov, &ok_cov, &p).as_deref(), Some("IMU frame_id must be imu_link"));
        assert_eq!(imu_block_reason(Some(0.05), "imu_link", [0.0, 0.0, 0.0, 0.5], [0.0; 3], [0.0; 3], &ok_cov, &ok_cov, &ok_cov, &p).as_deref(), Some("IMU orientation quaternion is not normalized"));
        assert_eq!(robot_pose_block_reason(Some(0.05), "map", [0.0; 3], [0.0, 0.0, 0.0, 1.0], &p), None);
        assert_eq!(robot_pose_block_reason(Some(0.05), "odom", [0.0; 3], [0.0, 0.0, 0.0, 1.0], &p).as_deref(), Some("robot pose frame must be map"));
    }
}
