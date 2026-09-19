//! mower_coverage: the coverage planner node (port of
//! `mower_mission/coverage_node.py`, node name `boustrophedon_coverage`,
//! same services, latched topics, parameters, messages and logs).
//!
//! * `/generate_coverage_path` -- for every zone map (`/get_zone_map_list_srv`)
//!   build the safe map (inflated mask AND inflated risk, resampled when the
//!   grids differ), keep the largest safe component, plan a zigzag or spiral
//!   with `mower_coverage_core`, repair unsafe jumps with A* connectors,
//!   validate, optionally prepend the boundary ring, and publish the markers.
//! * `/zone_exec_path` -- dispatch one zone's path to `nav_action_follow_path`
//!   (accept, register, confirm) and report acceptance.
//! * `/run_zone_sequence` / `/stop_zone_sequence` -- zone after zone with the
//!   recorded channel between them, cancellable at any point.
//!
//! Every dispatch is bounded: a goal accepted after the caller's deadline, an
//! unreadable result, a failed confirmation or a stop request all enter the
//! same tracker, which cancels the action, retries the correlated
//! `/cancel_navigation_dispatch` every two seconds and only forgets the goal
//! once a terminal state is proven -- exactly the Python node's
//! `_track_and_cancel_navigation_goal` / `_request_nav2_cancel_fallback`.

mod contours;

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex as StdMutex};
use std::time::Duration;

use futures::StreamExt;
use mower_rs_common::guard::{Guard, RELEASE_UNCONFIRMED};
use mower_rs_common::params;
use ndarray::Array2;
use r2r::builtin_interfaces::msg::Time;
use r2r::geometry_msgs::msg::{Point, Pose, PoseStamped};
use r2r::mower_interface::action::Waypoint;
use r2r::mower_interface::msg::ZoneMap;
use r2r::mower_interface::srv::{CancelNavigationDispatch, ChannelRoute, ConfirmNavigationDispatch, MissionOperationLock, ZoneExecPath, ZoneMapList, ZoneSequence};
use r2r::nav_msgs::msg::{MapMetaData, OccupancyGrid, Path};
use r2r::rcl_interfaces::msg::{ListParametersResult, Parameter, ParameterDescriptor, ParameterValue, SetParametersResult};
use r2r::rcl_interfaces::srv::{DescribeParameters, GetParameterTypes, GetParameters, ListParameters, SetParameters, SetParametersAtomically};
use r2r::std_msgs::msg::{Bool, Header};
use r2r::std_srvs::srv::Trigger;
use r2r::visualization_msgs::msg::{Marker, MarkerArray};
use r2r::{GoalStatus, QosProfile};
use tokio::sync::Mutex;

use mower_coverage_core::connector_planner::plan_connector_rs;
use mower_coverage_core::path_validator::validate_path_rs;
use mower_coverage_core::safe_map_filter::filter_safe_components_rs;
use mower_coverage_core::spiral::plan_spiral_coverage_rs;
use mower_coverage_core::types::{SafeMap, ValidationResult};
use mower_coverage_core::zigzag::generate_coverage_zigzag_path_rs;

// visualization_msgs/Marker constants (r2r does not export them)
const MARKER_ARROW: i32 = 0;
const MARKER_LINE_STRIP: i32 = 4;
const MARKER_ADD: i32 = 0;
const MARKER_DELETEALL: i32 = 3;

// rcl_interfaces/msg/ParameterType
const PARAMETER_NOT_SET: u8 = 0;
const PARAMETER_BOOL: u8 = 1;
const PARAMETER_DOUBLE: u8 = 3;
const PARAMETER_STRING: u8 = 4;

const VIVID_COLORS: [(f32, f32, f32); 10] = [
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
    (1.0, 1.0, 0.0),
    (1.0, 0.0, 1.0),
    (0.0, 1.0, 1.0),
    (1.0, 0.5, 0.0),
    (0.5, 0.0, 1.0),
    (0.0, 0.5, 1.0),
    (0.5, 1.0, 0.0),
];

fn stamp_now() -> Time {
    let ns = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_nanos() as i64).unwrap_or(0);
    Time { sec: (ns / 1_000_000_000) as i32, nanosec: (ns % 1_000_000_000) as u32 }
}

fn latched() -> QosProfile {
    QosProfile::default().keep_last(1).reliable().transient_local()
}

fn uuid_hex() -> String {
    use std::io::Read;
    let mut bytes = [0u8; 16];
    if let Ok(mut f) = std::fs::File::open("/dev/urandom") {
        let _ = f.read_exact(&mut bytes);
    }
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

/// Python `repr()` of a float inside a tuple / list.
fn py_float(x: f64) -> String {
    if x.is_finite() && x == x.trunc() && x.abs() < 1e16 {
        format!("{x:.1}")
    } else {
        format!("{x:?}")
    }
}

fn py_point(p: (f64, f64)) -> String {
    format!("({}, {})", py_float(p.0), py_float(p.1))
}

fn py_int_list(v: &[i32]) -> String {
    format!("[{}]", v.iter().map(|x| x.to_string()).collect::<Vec<_>>().join(", "))
}

fn py_usize_list(v: &[usize]) -> String {
    format!("[{}]", v.iter().map(|x| x.to_string()).collect::<Vec<_>>().join(", "))
}

// ------------------------------------------------------------ path utils

/// `_euler_to_quaternion(0, 0, yaw)`.
fn yaw_quaternion(yaw: f64) -> (f64, f64, f64, f64) {
    let (roll, pitch) = (0.0f64, 0.0f64);
    let (sr, cr) = ((roll / 2.0).sin(), (roll / 2.0).cos());
    let (sp, cp) = ((pitch / 2.0).sin(), (pitch / 2.0).cos());
    let (sy, cy) = ((yaw / 2.0).sin(), (yaw / 2.0).cos());
    let qx = sr * cp * cy - cr * sp * sy;
    let qy = cr * sp * cy + sr * cp * sy;
    let qz = cr * cp * sy - sr * sp * cy;
    let qw = cr * cp * cy + sr * sp * sy;
    (qx, qy, qz, qw)
}

/// `_transform_coverage_path_points`.
fn path_from_points(points: &[(f64, f64)], map_header: &Header) -> Path {
    let mut header = map_header.clone();
    header.frame_id = "map".into();
    let mut path = Path { header: header.clone(), poses: Vec::with_capacity(points.len()) };
    for (idx, &(x, y)) in points.iter().enumerate() {
        let (qx, qy, qz, qw) = if idx + 1 < points.len() {
            let (x2, y2) = points[idx + 1];
            yaw_quaternion((y2 - y).atan2(x2 - x))
        } else if idx > 0 {
            let (x2, y2) = points[idx - 1];
            yaw_quaternion((y - y2).atan2(x - x2))
        } else {
            yaw_quaternion(0.0)
        };
        let mut ps = PoseStamped::default();
        ps.header = header.clone();
        ps.header.stamp = map_header.stamp.clone();
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.position.z = 0.0;
        ps.pose.orientation.x = qx;
        ps.pose.orientation.y = qy;
        ps.pose.orientation.z = qz;
        ps.pose.orientation.w = qw;
        path.poses.push(ps);
    }
    path
}

/// `_transform_coverage_split_points`.
fn split_poses(points: &[(f64, f64)]) -> Vec<Pose> {
    points
        .iter()
        .map(|&(x, y)| {
            let mut p = Pose::default();
            p.position.x = x;
            p.position.y = y;
            p
        })
        .collect()
}

// ------------------------------------------------------------- parameters

#[derive(Clone)]
struct Params {
    strip_width_m: f64,
    waypoint_spacing_m: f64,
    zigzag_angle_deg: f64,
    unknown_as_obstacle: bool,
    min_safe_component_area_m2: f64,
    coverage_pattern: String,
    coverage_backend: String,
    allow_backend_fallback: bool,
    fallback_cancel_request_timeout_s: f64,
    boundary_ring: bool,
    use_sim_time: bool,
    start_type_description_service: bool,
}

const PARAM_NAMES: [&str; 12] = [
    "allow_backend_fallback",
    "boundary_ring",
    "coverage_backend",
    "coverage_pattern",
    "fallback_cancel_request_timeout_s",
    "min_safe_component_area_m2",
    "start_type_description_service",
    "strip_width_m",
    "unknown_as_obstacle",
    "use_sim_time",
    "waypoint_spacing_m",
    "zigzag_angle_deg",
];

fn declared_type(name: &str) -> Option<u8> {
    match name {
        "strip_width_m" | "waypoint_spacing_m" | "zigzag_angle_deg" | "min_safe_component_area_m2" | "fallback_cancel_request_timeout_s" => Some(PARAMETER_DOUBLE),
        "unknown_as_obstacle" | "allow_backend_fallback" | "boundary_ring" | "use_sim_time" | "start_type_description_service" => Some(PARAMETER_BOOL),
        "coverage_pattern" | "coverage_backend" => Some(PARAMETER_STRING),
        _ => None,
    }
}

fn type_name(t: u8) -> &'static str {
    match t {
        0 => "NOT_SET",
        1 => "BOOL",
        2 => "INTEGER",
        3 => "DOUBLE",
        4 => "STRING",
        5 => "BYTE_ARRAY",
        6 => "BOOL_ARRAY",
        7 => "INTEGER_ARRAY",
        8 => "DOUBLE_ARRAY",
        9 => "STRING_ARRAY",
        _ => "NOT_SET",
    }
}

impl Params {
    fn value(&self, name: &str) -> ParameterValue {
        let mut v = ParameterValue::default();
        let d = |v: &mut ParameterValue, x: f64| {
            v.type_ = PARAMETER_DOUBLE;
            v.double_value = x;
        };
        let b = |v: &mut ParameterValue, x: bool| {
            v.type_ = PARAMETER_BOOL;
            v.bool_value = x;
        };
        let s = |v: &mut ParameterValue, x: &str| {
            v.type_ = PARAMETER_STRING;
            v.string_value = x.to_string();
        };
        match name {
            "strip_width_m" => d(&mut v, self.strip_width_m),
            "waypoint_spacing_m" => d(&mut v, self.waypoint_spacing_m),
            "zigzag_angle_deg" => d(&mut v, self.zigzag_angle_deg),
            "min_safe_component_area_m2" => d(&mut v, self.min_safe_component_area_m2),
            "fallback_cancel_request_timeout_s" => d(&mut v, self.fallback_cancel_request_timeout_s),
            "unknown_as_obstacle" => b(&mut v, self.unknown_as_obstacle),
            "allow_backend_fallback" => b(&mut v, self.allow_backend_fallback),
            "boundary_ring" => b(&mut v, self.boundary_ring),
            "use_sim_time" => b(&mut v, self.use_sim_time),
            "start_type_description_service" => b(&mut v, self.start_type_description_service),
            "coverage_pattern" => s(&mut v, &self.coverage_pattern),
            "coverage_backend" => s(&mut v, &self.coverage_backend),
            _ => v.type_ = PARAMETER_NOT_SET,
        }
        v
    }

    fn assign(&mut self, p: &Parameter) {
        let v = &p.value;
        match p.name.as_str() {
            "strip_width_m" => self.strip_width_m = v.double_value,
            "waypoint_spacing_m" => self.waypoint_spacing_m = v.double_value,
            "zigzag_angle_deg" => self.zigzag_angle_deg = v.double_value,
            "min_safe_component_area_m2" => self.min_safe_component_area_m2 = v.double_value,
            "fallback_cancel_request_timeout_s" => self.fallback_cancel_request_timeout_s = v.double_value,
            "unknown_as_obstacle" => self.unknown_as_obstacle = v.bool_value,
            "allow_backend_fallback" => self.allow_backend_fallback = v.bool_value,
            "boundary_ring" => self.boundary_ring = v.bool_value,
            "use_sim_time" => self.use_sim_time = v.bool_value,
            "start_type_description_service" => self.start_type_description_service = v.bool_value,
            "coverage_pattern" => self.coverage_pattern = v.string_value.clone(),
            "coverage_backend" => self.coverage_backend = v.string_value.clone(),
            _ => {}
        }
    }
}

// ------------------------------------------------------- goal bookkeeping

type Handle = Arc<StdMutex<r2r::ActionClientGoal<Waypoint::Action>>>;

/// The action result, delivered once and readable by every tracker
/// (Python re-attaches callbacks to the same future object).
struct ResultSlot {
    id: u64,
    tx: tokio::sync::watch::Sender<Option<Result<(GoalStatus, Waypoint::Result), String>>>,
}

impl ResultSlot {
    fn new(id: u64) -> (Arc<ResultSlot>, tokio::sync::watch::Receiver<Option<Result<(GoalStatus, Waypoint::Result), String>>>) {
        let (tx, rx) = tokio::sync::watch::channel(None);
        (Arc::new(ResultSlot { id, tx }), rx)
    }
    async fn wait(&self) -> Result<(GoalStatus, Waypoint::Result), String> {
        let mut rx = self.tx.subscribe();
        loop {
            if let Some(v) = rx.borrow().clone() {
                return v;
            }
            if rx.changed().await.is_err() {
                return Err("result channel closed".into());
            }
        }
    }
}

/// `_proven_action_result`: only SUCCEEDED / CANCELED / ABORTED prove a terminal state.
fn proven(res: &Result<(GoalStatus, Waypoint::Result), String>) -> Result<(GoalStatus, Waypoint::Result), String> {
    match res {
        Ok((status, r)) => {
            if matches!(status, GoalStatus::Succeeded | GoalStatus::Canceled | GoalStatus::Aborted) {
                Ok((*status, r.clone()))
            } else {
                Err(format!("action result status {} is not terminal proof", status_code(*status)))
            }
        }
        Err(e) => Err(e.clone()),
    }
}

fn status_code(s: GoalStatus) -> i8 {
    match s {
        GoalStatus::Unknown => 0,
        GoalStatus::Accepted => 1,
        GoalStatus::Executing => 2,
        GoalStatus::Canceling => 3,
        GoalStatus::Succeeded => 4,
        GoalStatus::Canceled => 5,
        GoalStatus::Aborted => 6,
    }
}

struct ActiveGoal {
    handle_id: u64,
    dispatch_id: String,
    slot: Arc<ResultSlot>,
    cancel_marked: bool,
}

/// One live correlated-cancel request (`_request_nav2_cancel_fallback`'s `attempt`).
struct Attempt {
    claimed: bool,
    finished_callbacks: Vec<Box<dyn FnOnce() + Send>>,
    terminal_callbacks: Vec<Box<dyn FnOnce() + Send>>,
    terminal_confirmed: bool,
    request_abort: Option<tokio::task::AbortHandle>,
    timeout_abort: Option<tokio::task::AbortHandle>,
}

type SharedAttempt = Arc<StdMutex<Attempt>>;

/// One unconfirmed goal being driven to a proven terminal state.
struct Tracker {
    terminal: bool,
    timers: Vec<tokio::task::AbortHandle>,
    fallback_token: Option<u64>,
    fallback_attempt: Option<SharedAttempt>,
}

type SharedTracker = Arc<StdMutex<Tracker>>;

struct Tracking {
    unconfirmed: HashMap<u64, SharedTracker>,
    attempts: HashMap<String, SharedAttempt>,
    shutdown: bool,
    exec_goal_handle: Option<u64>,
    sequence_active_goal: Option<ActiveGoal>,
}

struct Pubs {
    path: r2r::Publisher<Path>,
    path_markers: r2r::Publisher<MarkerArray>,
    invalid_segments: r2r::Publisher<MarkerArray>,
    connectors: r2r::Publisher<MarkerArray>,
    #[allow(dead_code)]
    free_space_inflated: r2r::Publisher<OccupancyGrid>,
    #[allow(dead_code)]
    risk_map_inflated: r2r::Publisher<OccupancyGrid>,
}

struct Ctx {
    logger: String,
    params: StdMutex<Params>,
    guard: Mutex<Guard>,
    risk_map: StdMutex<Option<OccupancyGrid>>,
    risk_map_inflated: StdMutex<Option<OccupancyGrid>>,
    zone_map_list: Mutex<Vec<ZoneMap>>,
    last_nav_dispatch_error: StdMutex<String>,
    tracking: StdMutex<Tracking>,
    sequence_cancel: AtomicBool,
    sequence_task: StdMutex<Option<tokio::task::JoinHandle<()>>>,
    next_id: AtomicU64,
    pubs: StdMutex<Pubs>,
    zone_map_list_client: r2r::Client<ZoneMapList::Service>,
    confirm_client: r2r::Client<ConfirmNavigationDispatch::Service>,
    cancel_client: r2r::Client<CancelNavigationDispatch::Service>,
    nav_status_client: r2r::Client<Trigger::Service>,
    channel_route_client: r2r::Client<ChannelRoute::Service>,
    follow_client: r2r::ActionClient<Waypoint::Action>,
}

impl Ctx {
    fn info(&self, m: impl AsRef<str>) {
        r2r::log_info!(&self.logger, "{}", m.as_ref());
    }
    fn warn(&self, m: impl AsRef<str>) {
        r2r::log_warn!(&self.logger, "{}", m.as_ref());
    }
    fn error(&self, m: impl AsRef<str>) {
        r2r::log_error!(&self.logger, "{}", m.as_ref());
    }
    fn set_error(&self, m: &str) {
        *self.last_nav_dispatch_error.lock().unwrap() = m.to_string();
    }
    fn last_error(&self) -> String {
        self.last_nav_dispatch_error.lock().unwrap().clone()
    }
    fn params(&self) -> Params {
        self.params.lock().unwrap().clone()
    }
    fn sequence_alive(&self) -> bool {
        self.sequence_task.lock().unwrap().as_ref().map_or(false, |t| !t.is_finished())
    }

    // ------------------------------------------------------ service calls

    /// `_blocking_service_call`: wait 5 s for the server, then `timeout_s`
    /// for the answer; `None` on either (logged).
    async fn blocking_call<T: r2r::WrappedServiceTypeSupport + 'static>(&self, client: &r2r::Client<T>, req: &T::Request, timeout_s: f64) -> Option<T::Response> {
        let available = match r2r::Node::is_available(client) {
            Ok(f) => tokio::time::timeout(Duration::from_secs(5), f).await.is_ok(),
            Err(_) => false,
        };
        if !available {
            self.error("Service 不可用");
            return None;
        }
        let fut = match client.request(req) {
            Ok(f) => f,
            Err(e) => {
                self.error(format!("Service call dispatch failed: {e:?}"));
                return None;
            }
        };
        match tokio::time::timeout(Duration::from_secs_f64(timeout_s), fut).await {
            Ok(Ok(r)) => Some(r),
            Ok(Err(e)) => {
                self.error(format!("Service call 異常: {e:?}"));
                None
            }
            Err(_) => {
                self.error(format!("Service call 超時（{timeout_s:.0}s）"));
                None
            }
        }
    }

    // --------------------------------------------------------- planning

    fn clear_path_visuals(&self) {
        let mut marker = Marker::default();
        marker.action = MARKER_DELETEALL;
        let arr = MarkerArray { markers: vec![marker] };
        let pubs = self.pubs.lock().unwrap();
        let _ = pubs.path_markers.publish(&arr);
        let _ = pubs.invalid_segments.publish(&arr);
        let _ = pubs.connectors.publish(&arr);
        let mut empty = Path::default();
        empty.header.frame_id = "map".into();
        empty.header.stamp = stamp_now();
        let _ = pubs.path.publish(&empty);
    }

    fn publish_connectors(&self, viz: &[Vec<(f64, f64)>], zone_idx: usize, frame_id: &str) {
        let mut arr = MarkerArray::default();
        for (k, pts) in viz.iter().enumerate() {
            let mut m = Marker::default();
            m.header.frame_id = frame_id.into();
            m.header.stamp = stamp_now();
            m.ns = format!("zone_{zone_idx}_connector");
            m.id = (zone_idx * 10000 + k) as i32;
            m.type_ = MARKER_LINE_STRIP;
            m.action = MARKER_ADD;
            m.scale.x = 0.04;
            m.color.r = 1.0;
            m.color.g = 1.0;
            m.color.b = 0.0;
            m.color.a = 1.0;
            for &(x, y) in pts {
                m.points.push(Point { x, y, z: 0.0 });
            }
            arr.markers.push(m);
        }
        let _ = self.pubs.lock().unwrap().connectors.publish(&arr);
    }

    fn publish_invalid_segments(&self, points: &[(f64, f64)], segments: &[(usize, usize)], zone_idx: usize, frame_id: &str) {
        let mut arr = MarkerArray::default();
        for (k, &(i, j)) in segments.iter().enumerate() {
            let mut m = Marker::default();
            m.header.frame_id = frame_id.into();
            m.header.stamp = stamp_now();
            m.ns = format!("zone_{zone_idx}_invalid");
            m.id = (zone_idx * 10000 + k) as i32;
            m.type_ = MARKER_LINE_STRIP;
            m.action = MARKER_ADD;
            m.scale.x = 0.05;
            m.color.r = 1.0;
            m.color.g = 0.0;
            m.color.b = 0.0;
            m.color.a = 1.0;
            for idx in [i, j] {
                let (x, y) = points[idx];
                m.points.push(Point { x, y, z: 0.0 });
            }
            arr.markers.push(m);
        }
        let _ = self.pubs.lock().unwrap().invalid_segments.publish(&arr);
    }

    /// `_apply_connectors`: replace each invalid direct connection with an A* path.
    fn apply_connectors(&self, points: &[(f64, f64)], invalid: &[(usize, usize)], sm: &SafeMap, zone_idx: usize) -> (Vec<(f64, f64)>, Vec<Vec<(f64, f64)>>, Vec<(usize, usize)>) {
        let invalid_set: std::collections::HashSet<(usize, usize)> = invalid.iter().cloned().collect();
        let mut new_points = Vec::with_capacity(points.len());
        let mut viz = Vec::new();
        let mut unresolved = Vec::new();
        for (i, &pt) in points.iter().enumerate() {
            new_points.push(pt);
            if i + 1 >= points.len() || !invalid_set.contains(&(i, i + 1)) {
                continue;
            }
            match plan_connector_rs(pt, points[i + 1], sm, 0.2) {
                Some(connector) if connector.len() > 2 => {
                    for cp in &connector[1..connector.len() - 1] {
                        new_points.push(*cp);
                    }
                    self.info(format!("zone {zone_idx}: connector ({i}→{}) {} pts", i + 1, connector.len()));
                    viz.push(connector);
                }
                _ => {
                    unresolved.push((i, i + 1));
                    self.error(format!("zone {zone_idx}: no safe connector from {} to {}", py_point(pt), py_point(points[i + 1])));
                }
            }
        }
        (new_points, viz, unresolved)
    }

    /// `_risk_map_data_for_zone`: inflated risk aligned to the zone grid.
    fn risk_map_data_for_zone(&self, zone: &OccupancyGrid, unknown_as_obstacle: bool) -> Option<Vec<i16>> {
        let risk = self.risk_map_inflated.lock().unwrap().clone();
        let Some(risk) = risk else {
            self.error("缺少 risk_map_inflated 地圖數據");
            return None;
        };
        let ri = &risk.info;
        let expected = ri.height as usize * ri.width as usize;
        if risk.data.len() != expected {
            self.error(format!("risk_map_inflated data size does not match metadata: {} != {expected}", risk.data.len()));
            return None;
        }
        let risk_data: Vec<i16> = risk.data.iter().map(|&v| v as i16).collect();
        let zi = &zone.info;
        let same = zi.height == ri.height
            && zi.width == ri.width
            && (zi.resolution as f64 - ri.resolution as f64).abs() <= 1e-6
            && (zi.origin.position.x - ri.origin.position.x).abs() <= 1e-6
            && (zi.origin.position.y - ri.origin.position.y).abs() <= 1e-6;
        if same {
            return Some(risk_data);
        }
        self.warn("risk_map_inflated metadata differs from zone map; resampling risk map into zone grid");
        Some(self.resample_risk_map_to_zone(&risk_data, ri, zi, unknown_as_obstacle))
    }

    fn resample_risk_map_to_zone(&self, risk_data: &[i16], ri: &MapMetaData, zi: &MapMetaData, unknown_as_obstacle: bool) -> Vec<i16> {
        let (zh, zw) = (zi.height as usize, zi.width as usize);
        let zone_res = zi.resolution as f64;
        let (zox, zoy) = (zi.origin.position.x, zi.origin.position.y);
        let risk_res = ri.resolution as f64;
        let (rox, roy) = (ri.origin.position.x, ri.origin.position.y);
        let (rh, rw) = (ri.height as i64, ri.width as i64);
        let mut sampled = vec![0i16; zh * zw];
        let mut outside = 0usize;
        for r in 0..zh {
            let world_y = zoy + (r as f64 + 0.5) * zone_res;
            let row = ((world_y - roy) / risk_res).floor() as i64;
            for c in 0..zw {
                let world_x = zox + (c as f64 + 0.5) * zone_res;
                let col = ((world_x - rox) / risk_res).floor() as i64;
                let inside = row >= 0 && row < rh && col >= 0 && col < rw;
                if inside {
                    sampled[r * zw + c] = risk_data[(row * rw + col) as usize];
                } else {
                    outside += 1;
                    if unknown_as_obstacle {
                        sampled[r * zw + c] = 100;
                    }
                }
            }
        }
        if outside > 0 {
            self.warn(format!("{outside} zone cells are outside risk_map_inflated bounds"));
        }
        sampled
    }

    /// `generate_coverage_path`: plan every zone; false on the first failure.
    async fn generate_coverage_path(&self) -> bool {
        self.clear_path_visuals();
        let resp = self.blocking_call(&self.zone_map_list_client, &ZoneMapList::Request::default(), 10.0).await;
        let mut zones: Vec<ZoneMap> = resp.map(|r| r.zone_map_list).unwrap_or_default();
        *self.zone_map_list.lock().await = zones.clone();
        if zones.is_empty() {
            self.error("沒有可用的 zone map");
            return false;
        }
        let p = self.params();
        let pattern = p.coverage_pattern.to_lowercase();
        if pattern != "zigzag" && pattern != "spiral" {
            self.error(format!("unsupported coverage_pattern \"{pattern}\"; expected zigzag or spiral"));
            return false;
        }
        self.info(format!("coverage_pattern={pattern}"));
        let strip_width_m = p.strip_width_m;
        let waypoint_spacing_m = p.waypoint_spacing_m;
        let zigzag_angle_deg = p.zigzag_angle_deg;
        if strip_width_m <= 0.0 || waypoint_spacing_m <= 0.0 {
            self.error("strip_width_m and waypoint_spacing_m must both be positive");
            return false;
        }
        if !(0.0..=180.0).contains(&zigzag_angle_deg) {
            self.error("zigzag_angle_deg must be between 0 and 180 degrees");
            return false;
        }

        for i in 0..zones.len() {
            let zone_id = zones[i].zone_id;
            let info = zones[i].mask_map.info.clone();
            let (h, w) = (info.height as usize, info.width as usize);
            let res = info.resolution as f64;
            let (ox, oy) = (info.origin.position.x, info.origin.position.y);
            let Some(risk_data) = self.risk_map_data_for_zone(&zones[i].mask_map, p.unknown_as_obstacle) else {
                return false;
            };
            let inflated = &zones[i].mask_map_inflated.data;
            let mut safe = Array2::<bool>::default((h, w));
            for r in 0..h {
                for c in 0..w {
                    let k = r * w + c;
                    safe[[r, c]] = inflated.get(k).map_or(false, |&v| v == 0) && risk_data.get(k).map_or(false, |&v| v == 0);
                }
            }
            let (safe, component_sizes, kept) = filter_safe_components_rs(safe.view(), res, p.min_safe_component_area_m2, true);
            if kept.is_empty() {
                self.error(format!("zone {zone_id}: no safe component remains after filtering"));
                return false;
            }
            if component_sizes != kept {
                let head: Vec<usize> = component_sizes.iter().take(8).cloned().collect();
                self.warn(format!("zone {zone_id}: safe components {}, keeping {}", py_usize_list(&head), py_usize_list(&kept)));
            }
            let safe_cells = safe.iter().filter(|&&v| v).count();
            let risk_cells = risk_data.iter().filter(|&&v| v != 0).count();
            self.info(format!("zone {zone_id}: safe_cells={safe_cells}, risk_cells={risk_cells}"));

            let (mut coverage_pts, split_pts, invalid_segs) = if pattern == "spiral" {
                plan_spiral_coverage_rs(safe.view(), strip_width_m, waypoint_spacing_m, res, h, w, ox, oy)
            } else {
                generate_coverage_zigzag_path_rs(safe.view(), strip_width_m, waypoint_spacing_m, res, h, w, ox, oy, zigzag_angle_deg)
            };
            let sm = SafeMap { grid: safe.view(), resolution: res, origin_x: ox, origin_y: oy };
            let frame_id = if zones[i].mask_map.header.frame_id.is_empty() { "map".to_string() } else { zones[i].mask_map.header.frame_id.clone() };

            if !invalid_segs.is_empty() {
                self.warn(format!("zone {zone_id}: {} unsafe segment(s) — running ConnectorPlanner", invalid_segs.len()));
                let raw = coverage_pts.clone();
                let (pts, viz, unresolved) = self.apply_connectors(&coverage_pts, &invalid_segs, &sm, i);
                coverage_pts = pts;
                if !unresolved.is_empty() {
                    self.publish_invalid_segments(&raw, &unresolved, i, &frame_id);
                    self.error(format!("zone {zone_id}: {} unsafe connector(s) unresolved; coverage path not published", unresolved.len()));
                    return false;
                }
                if !viz.is_empty() {
                    self.publish_connectors(&viz, i, &frame_id);
                }
            }
            let final_validation: ValidationResult = validate_path_rs(&coverage_pts, &sm);
            if !final_validation.valid {
                self.publish_invalid_segments(&coverage_pts, &final_validation.invalid_segments, i, &frame_id);
                self.error(format!("zone {zone_id}: final path is unsafe: {}; coverage path not published", final_validation.message));
                return false;
            }

            if p.boundary_ring {
                let safe_u8: Vec<u8> = safe.iter().map(|&v| v as u8).collect();
                let ring = contours::outer_boundary_ring(&safe_u8, w, h, res, ox, oy);
                if !ring.is_empty() {
                    let mut combined = ring.clone();
                    combined.extend(coverage_pts.iter().cloned());
                    coverage_pts = combined;
                    self.info(format!("zone {zone_id}: added boundary ring ({} pts); re-validating combined path", ring.len()));
                    let ring_validation = validate_path_rs(&coverage_pts, &sm);
                    if !ring_validation.invalid_segments.is_empty() {
                        let raw = coverage_pts.clone();
                        let (pts, viz, unresolved) = self.apply_connectors(&coverage_pts, &ring_validation.invalid_segments, &sm, i);
                        coverage_pts = pts;
                        if !unresolved.is_empty() {
                            self.publish_invalid_segments(&raw, &unresolved, i, &frame_id);
                            self.error(format!("zone {zone_id}: {} unsafe boundary-ring connector(s) unresolved; coverage path not published", unresolved.len()));
                            return false;
                        }
                        if !viz.is_empty() {
                            self.publish_connectors(&viz, i, &frame_id);
                        }
                    }
                    let ring_final = validate_path_rs(&coverage_pts, &sm);
                    if !ring_final.valid {
                        self.publish_invalid_segments(&coverage_pts, &ring_final.invalid_segments, i, &frame_id);
                        self.error(format!("zone {zone_id}: boundary-ring path unsafe: {}; not published", ring_final.message));
                        return false;
                    }
                }
            }

            zones[i].path = path_from_points(&coverage_pts, &zones[i].mask_map.header);
            zones[i].coverage_split_points = split_poses(&split_pts);
        }

        let mut arr = MarkerArray::default();
        let mut color_i = 0usize;
        for (zone_idx, zone) in zones.iter().enumerate() {
            if !zone.path.poses.is_empty() {
                let mut line = Marker::default();
                line.header.frame_id = zone.path.header.frame_id.clone();
                line.header.stamp = stamp_now();
                line.ns = format!("zone_{}_path", zone.zone_id);
                line.id = zone_idx as i32;
                line.type_ = MARKER_LINE_STRIP;
                line.action = MARKER_ADD;
                line.scale.x = 0.03;
                let rgb = VIVID_COLORS[color_i % VIVID_COLORS.len()];
                line.color.r = rgb.0;
                line.color.g = rgb.1;
                line.color.b = rgb.2;
                line.color.a = 0.8;
                for ps in &zone.path.poses {
                    line.points.push(Point { x: ps.pose.position.x, y: ps.pose.position.y, z: ps.pose.position.z });
                }
                arr.markers.push(line);
                for (i, ps) in zone.path.poses.iter().step_by(5).enumerate() {
                    let mut arrow = Marker::default();
                    arrow.header.frame_id = zone.path.header.frame_id.clone();
                    arrow.header.stamp = stamp_now();
                    arrow.ns = format!("zone_{}_arrows", zone.zone_id);
                    arrow.id = (zone_idx * 1000 + i) as i32;
                    arrow.type_ = MARKER_ARROW;
                    arrow.action = MARKER_ADD;
                    arrow.pose = ps.pose.clone();
                    arrow.scale.x = 0.15;
                    arrow.scale.y = 0.04;
                    arrow.scale.z = 0.08;
                    arrow.color.r = if zone_idx == 0 { 0.5 } else { 0.0 };
                    arrow.color.g = if zone_idx == 0 { 0.0 } else { 0.5 };
                    arrow.color.b = 0.5;
                    arrow.color.a = 0.8;
                    arr.markers.push(arrow);
                }
            }
            color_i += 1;
        }
        let n = arr.markers.len();
        let _ = self.pubs.lock().unwrap().path_markers.publish(&arr);
        self.info(format!("發布了 {n} 個路徑markers"));
        *self.zone_map_list.lock().await = zones;
        true
    }

    // ------------------------------------------------- cancel tracking

    fn fallback_timeout_s(&self) -> f64 {
        let t = self.params().fallback_cancel_request_timeout_s;
        if t.is_finite() && (0.5..=30.0).contains(&t) { t } else { 5.0 }
    }

    /// `_request_nav2_cancel_fallback`: one bounded correlated-cancel request per
    /// dispatch id; returns the idempotent cancel of that attempt while it lives.
    async fn request_nav2_cancel_fallback(
        self: &Arc<Self>,
        reason: &str,
        dispatch_id: &str,
        on_terminal_confirmed: Option<Box<dyn FnOnce() + Send>>,
        on_request_finished: Option<Box<dyn FnOnce() + Send>>,
    ) -> Option<Box<dyn FnOnce() + Send>> {
        let timeout_s = self.fallback_timeout_s();
        let attempt: SharedAttempt = Arc::new(StdMutex::new(Attempt {
            claimed: false,
            finished_callbacks: Vec::new(),
            terminal_callbacks: Vec::new(),
            terminal_confirmed: false,
            request_abort: None,
            timeout_abort: None,
        }));
        {
            let mut a = attempt.lock().unwrap();
            if let Some(cb) = on_request_finished {
                a.finished_callbacks.push(cb);
            }
            if let Some(cb) = on_terminal_confirmed {
                a.terminal_callbacks.push(cb);
            }
        }
        let ctx = self.clone();
        let dispatch = dispatch_id.to_string();
        let reason_s = reason.to_string();

        // existing attempt for this dispatch id?
        let (existing, terminal_now) = {
            let mut tr = self.tracking.lock().unwrap();
            if tr.shutdown {
                return None;
            }
            match tr.attempts.get(dispatch_id).cloned() {
                None => {
                    tr.attempts.insert(dispatch.clone(), attempt.clone());
                    (None, None)
                }
                Some(existing) => {
                    let mut mine = attempt.lock().unwrap();
                    let mut ex = existing.lock().unwrap();
                    ex.finished_callbacks.append(&mut mine.finished_callbacks);
                    let mut terminal_now: Option<Box<dyn FnOnce() + Send>> = None;
                    for cb in mine.terminal_callbacks.drain(..) {
                        if ex.terminal_confirmed {
                            terminal_now = Some(cb);
                        } else {
                            ex.terminal_callbacks.push(cb);
                        }
                    }
                    drop(ex);
                    drop(mine);
                    (Some(existing), terminal_now)
                }
            }
        };
        if let Some(existing) = existing {
            if let Some(cb) = terminal_now {
                cb();
            }
            return Some(make_attempt_cancel(self.clone(), dispatch, existing));
        }

        let available = match r2r::Node::is_available(&self.cancel_client) {
            Ok(f) => tokio::time::timeout(Duration::from_millis(250), f).await.is_ok(),
            Err(_) => false,
        };
        if !available {
            if claim(&attempt) {
                self.error(format!("{reason}; correlated cancel service unavailable and navigation outcome remains uncertain"));
                finish_attempt(self, &dispatch, &attempt);
            }
            return None;
        }
        let fut = {
            let a = attempt.lock().unwrap();
            if a.claimed {
                return None;
            }
            self.cancel_client.request(&CancelNavigationDispatch::Request { dispatch_id: dispatch.clone() })
        };
        let fut = match fut {
            Ok(f) => f,
            Err(e) => {
                if claim(&attempt) {
                    self.error(format!("{reason}; fallback cancel dispatch failed: {e:?}; navigation outcome remains uncertain"));
                    finish_attempt(self, &dispatch, &attempt);
                }
                return None;
            }
        };
        if self.tracking.lock().unwrap().shutdown {
            let cancel = make_attempt_cancel(self.clone(), dispatch, attempt);
            cancel();
            return None;
        }
        // response task
        let req_task = {
            let ctx = ctx.clone();
            let attempt = attempt.clone();
            let dispatch = dispatch.clone();
            let reason = reason_s.clone();
            tokio::spawn(async move {
                let result = fut.await;
                if !claim(&attempt) {
                    return;
                }
                match result {
                    Ok(response) if response.success && response.terminal_confirmed => {
                        let callbacks = {
                            let mut a = attempt.lock().unwrap();
                            a.terminal_confirmed = true;
                            std::mem::take(&mut a.terminal_callbacks)
                        };
                        for cb in callbacks {
                            cb();
                        }
                    }
                    Ok(response) if !response.success => {
                        ctx.error(format!("{reason}; fallback cancel was not confirmed: {}", response.message));
                    }
                    Ok(_) => {}
                    Err(e) => {
                        ctx.error(format!("{reason}; fallback cancel failed: {e:?}"));
                    }
                }
                finish_attempt(&ctx, &dispatch, &attempt);
            })
        };
        let timeout_task = {
            let ctx = ctx.clone();
            let attempt = attempt.clone();
            let dispatch = dispatch.clone();
            let reason = reason_s.clone();
            tokio::spawn(async move {
                tokio::time::sleep(Duration::from_secs_f64(timeout_s)).await;
                if !claim(&attempt) {
                    return;
                }
                if let Some(h) = attempt.lock().unwrap().request_abort.take() {
                    h.abort();
                }
                ctx.error(format!("{reason}; fallback cancel response timed out after {timeout_s:.1}s; navigation outcome remains uncertain"));
                finish_attempt(&ctx, &dispatch, &attempt);
            })
        };
        {
            let mut a = attempt.lock().unwrap();
            a.request_abort = Some(req_task.abort_handle());
            a.timeout_abort = Some(timeout_task.abort_handle());
        }
        Some(make_attempt_cancel(self.clone(), dispatch, attempt))
    }

    /// `_register_active_navigation_goal`.
    fn register_active_goal(&self, handle_id: u64, dispatch_id: &str, slot: &Arc<ResultSlot>, sequence_owned: bool) -> bool {
        let mut tr = self.tracking.lock().unwrap();
        if sequence_owned && tr.sequence_active_goal.is_some() {
            return false;
        }
        tr.exec_goal_handle = Some(handle_id);
        if sequence_owned {
            tr.sequence_active_goal = Some(ActiveGoal { handle_id, dispatch_id: dispatch_id.to_string(), slot: slot.clone(), cancel_marked: false });
        }
        true
    }

    /// `_clear_active_navigation_goal`: only the matching generation.
    fn clear_active_goal(&self, handle_id: u64, dispatch_id: &str, slot_id: u64) {
        let mut tr = self.tracking.lock().unwrap();
        if tr.exec_goal_handle == Some(handle_id) {
            tr.exec_goal_handle = None;
        }
        if let Some(active) = &tr.sequence_active_goal {
            if active.handle_id == handle_id && active.dispatch_id == dispatch_id && active.slot.id == slot_id {
                tr.sequence_active_goal = None;
            }
        }
    }

    /// `_cancel_sequence_active_goal`: mark cancel-once, then cancel.
    async fn cancel_sequence_active_goal(self: &Arc<Self>, reason: &str, handles: &Handles) -> bool {
        let (handle_id, dispatch_id, slot) = {
            let mut tr = self.tracking.lock().unwrap();
            match tr.sequence_active_goal.as_mut() {
                None => return false,
                Some(a) if a.cancel_marked => return false,
                Some(a) => {
                    a.cancel_marked = true;
                    (a.handle_id, a.dispatch_id.clone(), a.slot.clone())
                }
            }
        };
        let handle = handles.get(handle_id);
        self.track_and_cancel_goal(handle, handle_id, reason, &dispatch_id, slot).await;
        true
    }

    /// `_track_and_cancel_navigation_goal`: retain a goal until terminal and
    /// verify its cancellation, with the correlated fallback on every gap.
    async fn track_and_cancel_goal(self: &Arc<Self>, handle: Option<Handle>, handle_id: u64, reason: &str, dispatch_id: &str, slot: Arc<ResultSlot>) -> bool {
        let tracker: SharedTracker = Arc::new(StdMutex::new(Tracker { terminal: false, timers: Vec::new(), fallback_token: None, fallback_attempt: None }));
        {
            let mut tr = self.tracking.lock().unwrap();
            if tr.shutdown || tr.unconfirmed.contains_key(&handle_id) {
                return false;
            }
            tr.unconfirmed.insert(handle_id, tracker.clone());
        }
        let ctx = self.clone();
        let reason_s = reason.to_string();
        let dispatch = dispatch_id.to_string();
        let cancel_acknowledged = Arc::new(AtomicBool::new(false));

        let mark_terminal: Arc<dyn Fn() + Send + Sync> = {
            let ctx = ctx.clone();
            let tracker = tracker.clone();
            let dispatch = dispatch.clone();
            let slot_id = slot.id;
            Arc::new(move || {
                let (timers, fallback) = {
                    let mut t = tracker.lock().unwrap();
                    if t.terminal {
                        return;
                    }
                    t.terminal = true;
                    let mut tr = ctx.tracking.lock().unwrap();
                    if tr.unconfirmed.get(&handle_id).map_or(false, |x| Arc::ptr_eq(x, &tracker)) {
                        tr.unconfirmed.remove(&handle_id);
                    }
                    t.fallback_token = None;
                    (std::mem::take(&mut t.timers), t.fallback_attempt.take())
                };
                for h in timers {
                    h.abort();
                }
                if let Some(a) = fallback {
                    make_attempt_cancel(ctx.clone(), dispatch.clone(), a)();
                }
                ctx.clear_active_goal(handle_id, &dispatch, slot_id);
            })
        };

        // request_fallback(detail)
        let request_fallback: Arc<dyn Fn(String) -> futures::future::BoxFuture<'static, bool> + Send + Sync> = {
            let ctx = ctx.clone();
            let tracker = tracker.clone();
            let reason = reason_s.clone();
            let dispatch = dispatch.clone();
            let mark_terminal = mark_terminal.clone();
            Arc::new(move |detail: String| {
                let ctx = ctx.clone();
                let tracker = tracker.clone();
                let reason = reason.clone();
                let dispatch = dispatch.clone();
                let mark_terminal = mark_terminal.clone();
                Box::pin(async move {
                    let token = ctx.next_id.fetch_add(1, Ordering::Relaxed);
                    {
                        let mut t = tracker.lock().unwrap();
                        if t.terminal || ctx.tracking.lock().unwrap().shutdown || t.fallback_token.is_some() {
                            return false;
                        }
                        t.fallback_token = Some(token);
                    }
                    let finish = {
                        let tracker = tracker.clone();
                        Box::new(move || {
                            let mut t = tracker.lock().unwrap();
                            if t.fallback_token == Some(token) {
                                t.fallback_token = None;
                                t.fallback_attempt = None;
                            }
                        }) as Box<dyn FnOnce() + Send>
                    };
                    let on_terminal = {
                        let m = mark_terminal.clone();
                        Box::new(move || m()) as Box<dyn FnOnce() + Send>
                    };
                    let cancel = ctx.request_nav2_cancel_fallback(&format!("{reason}; {detail}"), &dispatch, Some(on_terminal), Some(finish)).await;
                    let Some(cancel) = cancel else {
                        let mut t = tracker.lock().unwrap();
                        if t.fallback_token == Some(token) {
                            t.fallback_token = None;
                            t.fallback_attempt = None;
                        }
                        return false;
                    };
                    let keep = {
                        let mut t = tracker.lock().unwrap();
                        let keep = !t.terminal && !ctx.tracking.lock().unwrap().shutdown && t.fallback_token == Some(token);
                        if keep {
                            t.fallback_attempt = ctx.tracking.lock().unwrap().attempts.get(&dispatch).cloned();
                        }
                        keep
                    };
                    if !keep {
                        cancel();
                    }
                    keep
                })
            })
        };

        // terminal watcher on the shared result
        {
            let ctx = ctx.clone();
            let reason = reason_s.clone();
            let slot = slot.clone();
            let mark_terminal = mark_terminal.clone();
            let request_fallback = request_fallback.clone();
            tokio::spawn(async move {
                let res = slot.wait().await;
                match proven(&res) {
                    Ok(_) => {
                        ctx.warn(format!("{reason}; navigation action is now terminal"));
                        mark_terminal();
                    }
                    Err(e) => {
                        ctx.error(format!("{reason}; terminal result could not be read: {e}"));
                        request_fallback("action terminal result could not be confirmed".into()).await;
                    }
                }
            });
        }

        // action-level cancel
        match handle.as_ref().and_then(|h| h.lock().unwrap().cancel().ok()) {
            Some(fut) => {
                let ctx = ctx.clone();
                let reason = reason_s.clone();
                let ack = cancel_acknowledged.clone();
                let request_fallback = request_fallback.clone();
                tokio::spawn(async move {
                    match fut.await {
                        Ok(()) => ack.store(true, Ordering::SeqCst),
                        Err(e) => {
                            ctx.error(format!("{reason}; action cancel request failed: {e:?}"));
                            request_fallback("action server did not acknowledge cancellation".into()).await;
                        }
                    }
                });
            }
            None => {
                self.error(format!("{reason}; action cancellation could not be sent: no goal handle"));
                request_fallback("action cancellation dispatch failed".into()).await;
            }
        }

        // 2 s watchdog on the acknowledgment
        {
            let tracker_c = tracker.clone();
            let ack = cancel_acknowledged.clone();
            let request_fallback = request_fallback.clone();
            let task = tokio::spawn(async move {
                tokio::time::sleep(Duration::from_secs(2)).await;
                let terminal = tracker_c.lock().unwrap().terminal;
                if !terminal && !ack.load(Ordering::SeqCst) {
                    request_fallback("action cancellation acknowledgment timed out".into()).await;
                }
            });
            tracker.lock().unwrap().timers.push(task.abort_handle());
        }
        // 2 s retry until terminal
        {
            let tracker_c = tracker.clone();
            let ctx_c = ctx.clone();
            let request_fallback = request_fallback.clone();
            let task = tokio::spawn(async move {
                loop {
                    tokio::time::sleep(Duration::from_secs(2)).await;
                    if tracker_c.lock().unwrap().terminal || ctx_c.tracking.lock().unwrap().shutdown {
                        return;
                    }
                    request_fallback("action is still not terminal; retrying correlated cancel".into()).await;
                }
            });
            tracker.lock().unwrap().timers.push(task.abort_handle());
        }
        if tracker.lock().unwrap().terminal {
            for h in std::mem::take(&mut tracker.lock().unwrap().timers) {
                h.abort();
            }
        }
        true
    }

    // ------------------------------------------------------- dispatching

    /// `_send_follow_path`: dispatch a goal and wait at least for acceptance.
    async fn send_follow_path(self: &Arc<Self>, handles: &Handles, path: Path, split_points: Vec<Pose>, block: bool, sequence_owned: bool) -> bool {
        let timeout_s = 600.0;
        let acceptance_timeout_s = 3.0;
        self.set_error("");
        if sequence_owned && self.sequence_cancel.load(Ordering::SeqCst) {
            self.set_error("Zone sequence was canceled before goal dispatch");
            return false;
        }
        if path.poses.is_empty() {
            self.set_error("Navigation path is empty");
            self.error("Navigation path is empty");
            return false;
        }
        let available = match r2r::Node::is_available(&self.follow_client) {
            Ok(f) => tokio::time::timeout(Duration::from_secs(2), f).await.is_ok(),
            Err(_) => false,
        };
        if !available {
            self.set_error("Navigation action server unavailable");
            self.error("Navigation action server unavailable");
            return false;
        }
        let dispatch_id = uuid_hex();
        let goal = Waypoint::Goal { path, coverage_split_points: split_points, dispatch_id: dispatch_id.clone() };
        let fut = match self.follow_client.send_goal_request(goal) {
            Ok(f) => f,
            Err(e) => {
                let msg = format!("Navigation action goal dispatch failed: {e:?}");
                self.set_error(&msg);
                self.error(&msg);
                return false;
            }
        };
        let mut boxed: futures::future::BoxFuture<'static, _> = Box::pin(fut);
        let accepted = match tokio::time::timeout(Duration::from_secs_f64(acceptance_timeout_s), &mut boxed).await {
            Ok(r) => r,
            Err(_) => {
                let msg = "Navigation action goal acceptance timed out; dispatch outcome is uncertain and cancellation is being tracked";
                self.set_error(msg);
                self.error(msg);
                self.request_nav2_cancel_fallback(msg, &dispatch_id, None, None).await;
                // a late acceptance is adopted and canceled
                let ctx = self.clone();
                let handles = handles.clone();
                let dispatch = dispatch_id.clone();
                tokio::spawn(async move {
                    if let Ok((goal, result, _feedback)) = boxed.await {
                        ctx.warn("Canceling navigation goal accepted after timeout");
                        let (handle_id, handle, slot) = handles.adopt(&ctx, goal, result);
                        ctx.track_and_cancel_goal(Some(handle), handle_id, "Navigation goal was accepted after its caller timed out", &dispatch, slot).await;
                    }
                });
                return false;
            }
        };
        let (goal, result, _feedback) = match accepted {
            Ok(v) => v,
            Err(r2r::Error::RCL_RET_ACTION_GOAL_REJECTED) => {
                let mut reason: Option<String> = None;
                if let Some(status) = self.blocking_call(&self.nav_status_client, &Trigger::Request::default(), 1.0).await {
                    if status.success {
                        if let Ok(payload) = serde_json::from_str::<serde_json::Value>(&status.message) {
                            if let Some(s) = payload.get("block_reason").and_then(|v| v.as_str()) {
                                reason = Some(s.trim().to_string());
                            }
                        }
                    }
                }
                let msg = format!("Navigation action goal rejected: {}", reason.filter(|r| !r.is_empty()).unwrap_or_else(|| "busy or a safety precondition failed".into()));
                self.set_error(&msg);
                self.warn(&msg);
                return false;
            }
            Err(e) => {
                let msg = format!("Navigation action goal response failed ({e:?}); dispatch outcome is uncertain and correlated cancellation was requested");
                self.set_error(&msg);
                self.error(&msg);
                self.request_nav2_cancel_fallback(&msg, &dispatch_id, None, None).await;
                return false;
            }
        };
        let (handle_id, handle, slot) = handles.adopt(self, goal, result);

        if !self.register_active_goal(handle_id, &dispatch_id, &slot, sequence_owned) {
            let msg = "A previous zone-sequence goal is not terminal; the new accepted goal is being canceled";
            self.set_error(msg);
            self.track_and_cancel_goal(Some(handle), handle_id, msg, &dispatch_id, slot).await;
            return false;
        }
        if sequence_owned && self.sequence_cancel.load(Ordering::SeqCst) {
            let msg = "Zone sequence was canceled before dispatch confirmation";
            self.set_error(msg);
            self.cancel_sequence_active_goal(msg, handles).await;
            return false;
        }
        let confirm = self.blocking_call(&self.confirm_client, &ConfirmNavigationDispatch::Request { dispatch_id: dispatch_id.clone() }, 2.0).await;
        let confirmed = confirm.as_ref().map_or(false, |c| c.success);
        if !confirmed {
            let detail = confirm.map(|c| c.message).unwrap_or_else(|| "confirmation response timed out".into());
            let msg = format!("Navigation dispatch confirmation failed ({detail}); outcome is uncertain, navigation may be active, and cancellation is being tracked");
            self.set_error(&msg);
            self.error(&msg);
            self.track_and_cancel_goal(Some(handle), handle_id, &msg, &dispatch_id, slot).await;
            return false;
        }
        if sequence_owned && self.sequence_cancel.load(Ordering::SeqCst) {
            let msg = "Zone sequence was canceled while confirmation was pending";
            self.set_error(msg);
            self.cancel_sequence_active_goal(msg, handles).await;
            return false;
        }

        if !block {
            let ctx = self.clone();
            let slot_c = slot.clone();
            let dispatch = dispatch_id.clone();
            tokio::spawn(async move {
                let res = slot_c.wait().await;
                match proven(&res) {
                    Ok((status, r)) => {
                        let success = status == GoalStatus::Succeeded && r.success;
                        ctx.info(format!("Background navigation completed: success={}", if success { "True" } else { "False" }));
                        ctx.clear_active_goal(handle_id, &dispatch, slot_c.id);
                    }
                    Err(e) => {
                        let msg = format!("Background navigation result could not be read ({e}); outcome is uncertain, navigation may be active, and cancellation is being tracked");
                        ctx.set_error(&msg);
                        ctx.error(&msg);
                        ctx.track_and_cancel_goal(Some(handle), handle_id, &msg, &dispatch, slot_c).await;
                    }
                }
            });
            return true;
        }

        let outcome = tokio::time::timeout(Duration::from_secs_f64(timeout_s), slot.wait()).await;
        let result = match outcome {
            Err(_) => {
                let msg = format!("Navigation action timed out after {timeout_s:.0}s; outcome is uncertain, navigation may be active, and cancellation is being tracked");
                self.set_error(&msg);
                self.error(&msg);
                self.track_and_cancel_goal(Some(handle), handle_id, &msg, &dispatch_id, slot).await;
                return false;
            }
            Ok(res) => match proven(&res) {
                Ok((status, r)) => {
                    self.clear_active_goal(handle_id, &dispatch_id, slot.id);
                    status == GoalStatus::Succeeded && r.success
                }
                Err(e) => {
                    let msg = format!("Navigation action result could not be read ({e}); outcome is uncertain, navigation may be active, and cancellation is being tracked");
                    self.set_error(&msg);
                    self.error(&msg);
                    self.track_and_cancel_goal(Some(handle), handle_id, &msg, &dispatch_id, slot).await;
                    false
                }
            },
        };
        if !result && self.last_error().is_empty() {
            self.set_error("Navigation action completed unsuccessfully");
        }
        result
    }

    // --------------------------------------------------------- sequence

    async fn get_zone_map(&self, zone_id: i32) -> Option<ZoneMap> {
        self.zone_map_list.lock().await.iter().find(|z| z.zone_id == zone_id).cloned()
    }

    /// `_run_sequence`: coverage, channel, coverage, ...
    async fn run_sequence(self: Arc<Self>, handles: Handles, zone_ids: Vec<i32>, channel_proximity_m: f64) {
        self.info(format!("任務序列開始: zones={}", py_int_list(&zone_ids)));
        for (i, &zone_id) in zone_ids.iter().enumerate() {
            if self.sequence_cancel.load(Ordering::SeqCst) {
                self.warn(format!("任務序列在 zone {zone_id} 前已取消"));
                return;
            }
            let zone = match self.get_zone_map(zone_id).await {
                Some(z) => z,
                None => {
                    self.error(format!("Zone {zone_id} 覆蓋路徑執行失敗或被取消，任務序列中止"));
                    return;
                }
            };
            self.info(format!("[{}/{}] 執行 zone {zone_id} 覆蓋路徑，共 {} 個路徑點", i + 1, zone_ids.len(), zone.path.poses.len()));
            let ok = self.send_follow_path(&handles, zone.path.clone(), zone.coverage_split_points.clone(), true, true).await;
            if !ok {
                self.error(format!("Zone {zone_id} 覆蓋路徑執行失敗或被取消，任務序列中止"));
                return;
            }
            self.info(format!("Zone {zone_id} 覆蓋完成"));
            if self.sequence_cancel.load(Ordering::SeqCst) || i + 1 >= zone_ids.len() {
                continue;
            }
            let next_zone_id = zone_ids[i + 1];
            self.info(format!("尋找通道: zone {zone_id} → zone {next_zone_id}"));
            let req = ChannelRoute::Request { zone_from_id: zone_id, zone_to_id: next_zone_id, proximity_m: channel_proximity_m as f32 };
            let route = self.blocking_call(&self.channel_route_client, &req, 10.0).await;
            let route = match route {
                Some(r) if r.success => r,
                other => {
                    let msg = other.map(|r| r.message).unwrap_or_else(|| "服務呼叫超時".into());
                    self.error(format!("取得通道路徑失敗: {msg}，任務序列中止"));
                    return;
                }
            };
            self.info(format!("走通道 #{} (zone {zone_id} → zone {next_zone_id})，共 {} 個路徑點", route.matched_channel_id, route.channel_path.poses.len()));
            let ok = self.send_follow_path(&handles, route.channel_path, Vec::new(), true, true).await;
            if !ok {
                self.error(format!("通道 {zone_id}→{next_zone_id} 導航失敗或被取消，任務序列中止"));
                return;
            }
            self.info(format!("通道 zone {zone_id} → zone {next_zone_id} 完成"));
        }
        self.info(format!("任務序列全部完成: zones={}", py_int_list(&zone_ids)));
    }

    // ------------------------------------------------------- parameters

    /// `_on_planning_parameters_changed` after rclpy's type check.
    async fn set_parameters(&self, params: &[Parameter]) -> SetParametersResult {
        let reject = |reason: String| SetParametersResult { successful: false, reason };
        for p in params {
            let Some(expected) = declared_type(&p.name) else {
                return reject("Invalid access to undeclared parameter(s): []".to_string());
            };
            if p.value.type_ != expected {
                return reject(format!("Wrong parameter type, expected 'Type.{}' got 'Type.{}'", type_name(expected), type_name(p.value.type_)));
            }
        }
        let immutable: Vec<&str> = params.iter().filter(|p| p.name == "coverage_backend" || p.name == "allow_backend_fallback").map(|p| p.name.as_str()).collect();
        if !immutable.is_empty() {
            return reject(format!("{} is startup-only; restart coverage_node with the desired backend", immutable.join(", ")));
        }
        let guarded = ["strip_width_m", "waypoint_spacing_m", "zigzag_angle_deg", "unknown_as_obstacle", "min_safe_component_area_m2", "coverage_pattern", "boundary_ring"];
        if params.iter().any(|p| guarded.contains(&p.name.as_str())) {
            let g = self.guard.lock().await;
            if g.blocked() || g.busy() {
                return reject("coverage parameters cannot change while navigation or coverage generation is active/unknown".into());
            }
        }
        let mut pr = self.params.lock().unwrap();
        for p in params {
            pr.assign(p);
        }
        SetParametersResult { successful: true, reason: String::new() }
    }
}

fn claim(attempt: &SharedAttempt) -> bool {
    let mut a = attempt.lock().unwrap();
    if a.claimed {
        return false;
    }
    a.claimed = true;
    if let Some(h) = a.timeout_abort.take() {
        h.abort();
    }
    true
}

/// `_finish_attempt`: drop the registry entry and run the finished callbacks.
fn finish_attempt(ctx: &Ctx, dispatch_id: &str, attempt: &SharedAttempt) {
    let callbacks = {
        let mut tr = ctx.tracking.lock().unwrap();
        if tr.attempts.get(dispatch_id).map_or(false, |a| Arc::ptr_eq(a, attempt)) {
            tr.attempts.remove(dispatch_id);
        }
        let mut a = attempt.lock().unwrap();
        a.terminal_callbacks.clear();
        std::mem::take(&mut a.finished_callbacks)
    };
    for cb in callbacks {
        cb();
    }
}

/// The attempt's idempotent cancel: claim, abort the request, finish.
fn make_attempt_cancel(ctx: Arc<Ctx>, dispatch_id: String, attempt: SharedAttempt) -> Box<dyn FnOnce() + Send> {
    Box::new(move || {
        if !claim(&attempt) {
            return;
        }
        if let Some(h) = attempt.lock().unwrap().request_abort.take() {
            h.abort();
        }
        finish_attempt(&ctx, &dispatch_id, &attempt);
    })
}

/// Goal handles by id (`id(handle)` in Python) so trackers can address them.
#[derive(Clone)]
struct Handles(Arc<StdMutex<HashMap<u64, Handle>>>);

impl Handles {
    fn get(&self, id: u64) -> Option<Handle> {
        self.0.lock().unwrap().get(&id).cloned()
    }

    /// Wrap an accepted goal: retain the handle, forward its result once
    /// into a shared slot, and hand back the ids the trackers use.
    fn adopt(
        &self,
        ctx: &Arc<Ctx>,
        goal: r2r::ActionClientGoal<Waypoint::Action>,
        result: impl std::future::Future<Output = Result<(GoalStatus, Waypoint::Result), r2r::Error>> + Send + 'static,
    ) -> (u64, Handle, Arc<ResultSlot>) {
        let id = ctx.next_id.fetch_add(1, Ordering::Relaxed);
        let handle: Handle = Arc::new(StdMutex::new(goal));
        self.0.lock().unwrap().insert(id, handle.clone());
        let (slot, _rx) = ResultSlot::new(ctx.next_id.fetch_add(1, Ordering::Relaxed));
        let slot_c = slot.clone();
        let registry = self.0.clone();
        tokio::spawn(async move {
            let outcome = result.await.map_err(|e| format!("{e:?}"));
            let _ = slot_c.tx.send(Some(outcome));
            // keep the handle a little longer for late trackers, then drop it
            tokio::time::sleep(Duration::from_secs(30)).await;
            registry.lock().unwrap().remove(&id);
        });
        (id, handle, slot)
    }
}

#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let r2r_ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(r2r_ctx, "boustrophedon_coverage", "")?;
    let logger = node.logger().to_string();
    r2r::log_info!(&logger, "boustrophedon_coverage 初始化");

    let params_v = Params {
        strip_width_m: params::f64(&node, "strip_width_m", 0.8),
        waypoint_spacing_m: params::f64(&node, "waypoint_spacing_m", 0.2),
        zigzag_angle_deg: params::f64(&node, "zigzag_angle_deg", 0.0),
        unknown_as_obstacle: params::bool(&node, "unknown_as_obstacle", true),
        min_safe_component_area_m2: params::f64(&node, "min_safe_component_area_m2", 0.05),
        coverage_pattern: params::string(&node, "coverage_pattern", "zigzag"),
        coverage_backend: params::string(&node, "coverage_backend", "rust"),
        allow_backend_fallback: params::bool(&node, "allow_backend_fallback", true),
        fallback_cancel_request_timeout_s: params::f64(&node, "fallback_cancel_request_timeout_s", 5.0),
        boundary_ring: params::bool(&node, "boundary_ring", false),
        use_sim_time: params::bool(&node, "use_sim_time", false),
        start_type_description_service: params::bool(&node, "start_type_description_service", true),
    };
    if params_v.coverage_backend != "rust" && params_v.coverage_backend != "python" {
        return Err(format!("Unknown coverage_backend \"{}\"; expected \"python\" or \"rust\"", params_v.coverage_backend).into());
    }
    r2r::log_info!(&logger, "coverage backend: RustBackend");

    let lock_client = node.create_client::<MissionOperationLock::Service>("/mission_operation_lock", QosProfile::services_default())?;
    let owner = format!("{}:{}", node.fully_qualified_name()?, uuid_hex());
    let guard = Guard::new(owner, lock_client, logger.clone());

    let pubs = Pubs {
        path: node.create_publisher::<Path>("/coverage_path", latched())?,
        path_markers: node.create_publisher::<MarkerArray>("/coverage_path_markers", latched())?,
        invalid_segments: node.create_publisher::<MarkerArray>("/coverage_invalid_segments", latched())?,
        connectors: node.create_publisher::<MarkerArray>("/coverage_connectors", latched())?,
        free_space_inflated: node.create_publisher::<OccupancyGrid>("/free_space_inflated", latched())?,
        risk_map_inflated: node.create_publisher::<OccupancyGrid>("/risk_map_inflated", latched())?,
    };
    let ctx = Arc::new(Ctx {
        logger: logger.clone(),
        params: StdMutex::new(params_v),
        guard: Mutex::new(guard),
        risk_map: StdMutex::new(None),
        risk_map_inflated: StdMutex::new(None),
        zone_map_list: Mutex::new(Vec::new()),
        last_nav_dispatch_error: StdMutex::new(String::new()),
        tracking: StdMutex::new(Tracking { unconfirmed: HashMap::new(), attempts: HashMap::new(), shutdown: false, exec_goal_handle: None, sequence_active_goal: None }),
        sequence_cancel: AtomicBool::new(false),
        sequence_task: StdMutex::new(None),
        next_id: AtomicU64::new(1),
        pubs: StdMutex::new(pubs),
        zone_map_list_client: node.create_client::<ZoneMapList::Service>("/get_zone_map_list_srv", QosProfile::services_default())?,
        confirm_client: node.create_client::<ConfirmNavigationDispatch::Service>("/confirm_navigation_dispatch", QosProfile::services_default())?,
        cancel_client: node.create_client::<CancelNavigationDispatch::Service>("/cancel_navigation_dispatch", QosProfile::services_default())?,
        nav_status_client: node.create_client::<Trigger::Service>("/check_nav_status", QosProfile::services_default())?,
        channel_route_client: node.create_client::<ChannelRoute::Service>("/get_channel_route", QosProfile::services_default())?,
        follow_client: node.create_action_client::<Waypoint::Action>("nav_action_follow_path")?,
    });
    // `/record_path_status` client of the Python node (never used) is not created.
    let handles = Handles(Arc::new(StdMutex::new(HashMap::new())));

    // ---- subscriptions ----------------------------------------------------
    {
        let mut stream = node.subscribe::<Bool>("/nav_operation_active", latched())?;
        let ctx = ctx.clone();
        tokio::spawn(async move {
            while let Some(msg) = stream.next().await {
                let mut g = ctx.guard.lock().await;
                g.nav_active = msg.data;
                g.nav_seen = true;
            }
        });
    }
    {
        let mut stream = node.subscribe::<OccupancyGrid>("/risk_map", latched())?;
        let ctx = ctx.clone();
        tokio::spawn(async move {
            while let Some(msg) = stream.next().await {
                *ctx.risk_map.lock().unwrap() = Some(msg);
                ctx.info("收到 risk_map 地圖");
            }
        });
    }
    {
        let mut stream = node.subscribe::<OccupancyGrid>("/risk_map_inflated", latched())?;
        let ctx = ctx.clone();
        tokio::spawn(async move {
            while let Some(msg) = stream.next().await {
                *ctx.risk_map_inflated.lock().unwrap() = Some(msg);
                ctx.info("收到 risk_map_inflated 地圖");
            }
        });
    }

    // ---- services ---------------------------------------------------------
    {
        let mut stream = node.create_service::<Trigger::Service>("/generate_coverage_path", QosProfile::services_default())?;
        let ctx = ctx.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let (success, message) = {
                    let mut g = ctx.guard.lock().await;
                    match g.acquire("regenerate the coverage path").await {
                        Err(message) => (false, message),
                        Ok(()) => {
                            drop(g);
                            let (mut success, mut message) = if ctx.risk_map_inflated.lock().unwrap().is_none() {
                                (false, "缺少 risk_map_inflated 地圖數據".to_string())
                            } else if ctx.generate_coverage_path().await {
                                (true, "覆蓋路徑生成成功".to_string())
                            } else {
                                (false, "覆蓋路徑生成失敗".to_string())
                            };
                            let mut g = ctx.guard.lock().await;
                            if !g.release().await {
                                let previous = message.trim().to_string();
                                success = false;
                                message = if previous.is_empty() { RELEASE_UNCONFIRMED.to_string() } else { format!("{previous}; {RELEASE_UNCONFIRMED}") };
                                ctx.error(&message);
                            }
                            (success, message)
                        }
                    }
                };
                let _ = req.respond(Trigger::Response { success, message });
            }
        });
    }
    {
        let mut stream = node.create_service::<ZoneExecPath::Service>("/zone_exec_path", QosProfile::services_default())?;
        let ctx = ctx.clone();
        let handles = handles.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let ctx = ctx.clone();
                let handles = handles.clone();
                tokio::spawn(async move {
                    let response = {
                        let rejected = ctx.guard.lock().await.reject_reason("start another navigation mission");
                        if let Some(message) = rejected {
                            ZoneExecPath::Response { success: false, message }
                        } else {
                            ctx.info(format!("zone_exec_path_srv start: {}", req.message.zone_id));
                            match ctx.get_zone_map(req.message.zone_id).await {
                                None => ZoneExecPath::Response { success: false, message: "Zone not found".into() },
                                Some(zone) if zone.path.poses.is_empty() => ZoneExecPath::Response { success: false, message: "Zone coverage path is empty".into() },
                                Some(zone) => {
                                    let dispatched = ctx.send_follow_path(&handles, zone.path.clone(), zone.coverage_split_points.clone(), false, false).await;
                                    ZoneExecPath::Response { success: dispatched, message: if dispatched { "Navigation action goal accepted".into() } else { ctx.last_error() } }
                                }
                            }
                        }
                    };
                    let _ = req.respond(response);
                });
            }
        });
    }
    {
        let mut stream = node.create_service::<ZoneSequence::Service>("/run_zone_sequence", QosProfile::services_default())?;
        let ctx = ctx.clone();
        let handles = handles.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let response = (|| async {
                    if let Some(message) = ctx.guard.lock().await.reject_reason("start a zone sequence") {
                        return ZoneSequence::Response { success: false, message };
                    }
                    if ctx.sequence_alive() {
                        return ZoneSequence::Response { success: false, message: "已有任務序列執行中，請先呼叫 /stop_zone_sequence".into() };
                    }
                    if ctx.tracking.lock().unwrap().sequence_active_goal.is_some() {
                        return ZoneSequence::Response { success: false, message: "前一序列的導航終止狀態尚未確認，拒絕啟動新序列".into() };
                    }
                    let zone_ids = req.message.zone_ids.clone();
                    if zone_ids.is_empty() {
                        return ZoneSequence::Response { success: false, message: "zone_ids 不可為空".into() };
                    }
                    let mut missing = Vec::new();
                    for &z in &zone_ids {
                        match ctx.get_zone_map(z).await {
                            Some(m) if !m.path.poses.is_empty() => {}
                            _ => missing.push(z),
                        }
                    }
                    if !missing.is_empty() {
                        return ZoneSequence::Response { success: false, message: format!("以下 zone 尚無覆蓋路徑，請先呼叫 /generate_coverage_path: {}", py_int_list(&missing)) };
                    }
                    let proximity_m = if req.message.channel_proximity_m > 0.0 { req.message.channel_proximity_m as f64 } else { 1.5 };
                    ctx.sequence_cancel.store(false, Ordering::SeqCst);
                    let task = tokio::spawn(ctx.clone().run_sequence(handles.clone(), zone_ids.clone(), proximity_m));
                    *ctx.sequence_task.lock().unwrap() = Some(task);
                    ZoneSequence::Response { success: true, message: format!("任務序列已啟動: zones={}, channel_proximity={proximity_m:.2}m", py_int_list(&zone_ids)) }
                })()
                .await;
                let _ = req.respond(response);
            }
        });
    }
    {
        let mut stream = node.create_service::<Trigger::Service>("/stop_zone_sequence", QosProfile::services_default())?;
        let ctx = ctx.clone();
        let handles = handles.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                ctx.sequence_cancel.store(true, Ordering::SeqCst);
                let cancel_started = ctx.cancel_sequence_active_goal("Zone sequence stop requested", &handles).await;
                let message = if cancel_started {
                    "已立即送出目前導航目標的取消請求；正追蹤至終止狀態"
                } else if ctx.tracking.lock().unwrap().sequence_active_goal.is_some() {
                    "目前導航目標的取消已在追蹤中"
                } else if ctx.sequence_alive() {
                    "已送出取消請求；序列尚未派送或正在步驟間切換"
                } else {
                    "目前無執行中的任務序列"
                };
                let _ = req.respond(Trigger::Response { success: true, message: message.into() });
            }
        });
    }

    // ---- parameter services (/boustrophedon_coverage/*) --------------------
    {
        let mut stream = node.create_service::<GetParameters::Service>("/boustrophedon_coverage/get_parameters", QosProfile::services_default())?;
        let ctx = ctx.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let values = if req.message.names.iter().all(|n| declared_type(n).is_some()) {
                    let p = ctx.params();
                    req.message.names.iter().map(|n| p.value(n)).collect()
                } else {
                    Vec::new()
                };
                let _ = req.respond(GetParameters::Response { values });
            }
        });
    }
    {
        let mut stream = node.create_service::<GetParameterTypes::Service>("/boustrophedon_coverage/get_parameter_types", QosProfile::services_default())?;
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let types = req.message.names.iter().map(|n| declared_type(n).unwrap_or(PARAMETER_NOT_SET)).collect();
                let _ = req.respond(GetParameterTypes::Response { types });
            }
        });
    }
    {
        let mut stream = node.create_service::<ListParameters::Service>("/boustrophedon_coverage/list_parameters", QosProfile::services_default())?;
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let prefixes = &req.message.prefixes;
                let names = PARAM_NAMES.iter().filter(|n| prefixes.is_empty() || prefixes.iter().any(|p| n.starts_with(p.as_str()))).map(|n| n.to_string()).collect();
                let _ = req.respond(ListParameters::Response { result: ListParametersResult { names, prefixes: Vec::new() } });
            }
        });
    }
    {
        let mut stream = node.create_service::<DescribeParameters::Service>("/boustrophedon_coverage/describe_parameters", QosProfile::services_default())?;
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let descriptors = req.message.names.iter().map(|n| ParameterDescriptor { name: n.clone(), type_: declared_type(n).unwrap_or(PARAMETER_NOT_SET), ..Default::default() }).collect();
                let _ = req.respond(DescribeParameters::Response { descriptors });
            }
        });
    }
    {
        let mut stream = node.create_service::<SetParameters::Service>("/boustrophedon_coverage/set_parameters", QosProfile::services_default())?;
        let ctx = ctx.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let mut results = Vec::with_capacity(req.message.parameters.len());
                for p in &req.message.parameters {
                    results.push(ctx.set_parameters(std::slice::from_ref(p)).await);
                }
                let _ = req.respond(SetParameters::Response { results });
            }
        });
    }
    {
        let mut stream = node.create_service::<SetParametersAtomically::Service>("/boustrophedon_coverage/set_parameters_atomically", QosProfile::services_default())?;
        let ctx = ctx.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let result = ctx.set_parameters(&req.message.parameters).await;
                let _ = req.respond(SetParametersAtomically::Response { result });
            }
        });
    }

    // ---- spin until SIGINT / SIGTERM ----------------------------------------
    let running = Arc::new(AtomicBool::new(true));
    let spin = {
        let running = running.clone();
        tokio::task::spawn_blocking(move || {
            while running.load(Ordering::Relaxed) {
                node.spin_once(Duration::from_millis(50));
            }
            drop(node);
        })
    };
    let mut sigterm = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
    tokio::select! {
        _ = tokio::signal::ctrl_c() => {}
        _ = sigterm.recv() => {}
    }
    // `_shutdown_navigation_goal_tracking`
    {
        let (timers, attempts) = {
            let mut tr = ctx.tracking.lock().unwrap();
            tr.shutdown = true;
            let timers: Vec<_> = tr.unconfirmed.values().flat_map(|t| std::mem::take(&mut t.lock().unwrap().timers)).collect();
            for t in tr.unconfirmed.values() {
                let mut t = t.lock().unwrap();
                t.terminal = true;
                t.fallback_token = None;
                t.fallback_attempt = None;
            }
            tr.unconfirmed.clear();
            let attempts: Vec<(String, SharedAttempt)> = tr.attempts.drain().collect();
            (timers, attempts)
        };
        for h in timers {
            h.abort();
        }
        for (id, a) in attempts {
            make_attempt_cancel(ctx.clone(), id, a)();
        }
    }
    running.store(false, Ordering::Relaxed);
    let _ = spin.await;
    Ok(())
}
