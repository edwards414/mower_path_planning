//! mower_adapter: the Flutter-facing relay (port of
//! `mower_mission/adapters/flutter_adapter_node.py`, node name kept as
//! `flutter_adapter` so launch parameters are unchanged).
//!
//! * `/adapter/map_layers/<name>` and `/adapter/marker_layers/<name>`: latched
//!   JSON snapshots of the OccupancyGrid / MarkerArray topics.
//! * `/adapter/robot_pose`: validated map -> base_footprint pose from the
//!   EKF map odometry (`robot_pose_source_topic`).
//! * `/adapter/coverage_settings` (1 Hz), `/adapter/zone_summaries` (2 Hz)
//!   and `/adapter/map_datum` (0.5 Hz until navsat answers) from the
//!   coverage / map_manage parameter services, the zone map list service
//!   and navsat_transform's toLL.
//!
//! Encoding a 214 KB map grid takes ~60 ms in Python and ~2 ms here.

mod dto;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use futures::StreamExt;
use mower_rs_common::params;
use r2r::geometry_msgs::msg::{Point, PoseStamped};
use r2r::mower_interface::srv::ZoneMapList;
use r2r::nav_msgs::msg::{OccupancyGrid, Odometry};
use r2r::rcl_interfaces::srv::GetParameters;
use r2r::robot_localization::srv::ToLL;
use r2r::std_msgs::msg::String as StringMsg;
use r2r::visualization_msgs::msg::MarkerArray;
use r2r::QosProfile;
use serde_json::{json, Map, Value};

const MAP_TOPICS: [(&str, &str); 4] = [
    ("/map_grid", "map_grid"),
    ("/free_space_inflated", "free_space_inflated"),
    ("/risk_map_inflated", "risk_map_inflated"),
    ("/chennal_map_inflated", "chennal_map_inflated"),
];

const MARKER_TOPICS: [(&str, &str); 6] = [
    ("/zone_list", "zones"),
    ("/risk_zone_list", "risk_zones"),
    ("/chennal_path_array", "channels"),
    ("/coverage_path_markers", "coverage_path"),
    ("/coverage_invalid_segments", "invalid_segments"),
    ("/coverage_connectors", "connectors"),
];

const COVERAGE_PARAM_NAMES: [&str; 6] = [
    "strip_width_m",
    "waypoint_spacing_m",
    "zigzag_angle_deg",
    "unknown_as_obstacle",
    "coverage_pattern",
    "boundary_ring",
];
const MAP_PARAM_NAMES: [&str; 1] = ["inflate_radius_m"];

/// Metres along map +X used to derive the datum bearing.
const DATUM_PROBE_M: f64 = 10.0;
/// A service call that does not answer within this is treated as failed.
const SERVICE_TIMEOUT: Duration = Duration::from_secs(3);

fn latched() -> QosProfile {
    QosProfile::default().keep_last(1).reliable().transient_local()
}

fn json_string(v: &Value) -> StringMsg {
    StringMsg { data: serde_json::to_string(v).expect("json") }
}

fn now_ns() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as i64)
        .unwrap_or(0)
}

struct PoseGate {
    max_age_s: f64,
    source_frame: String,
    child_frame: String,
}

impl PoseGate {
    /// The relayed pose, or `None` when the sample must not become a fresh
    /// 5 Hz pose relay (wrong frames, non-finite, bad quaternion, stale).
    fn relay(&self, odom: &Odometry) -> Option<PoseStamped> {
        let stamp_ns = odom.header.stamp.sec as i64 * 1_000_000_000 + odom.header.stamp.nanosec as i64;
        let age_s = if stamp_ns > 0 { (now_ns() - stamp_ns) as f64 / 1e9 } else { f64::INFINITY };
        let p = &odom.pose.pose.position;
        let q = &odom.pose.pose.orientation;
        let values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w];
        let quaternion_norm = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();
        if odom.header.frame_id != self.source_frame
            || odom.child_frame_id != self.child_frame
            || !values.iter().all(|v| v.is_finite())
            || !quaternion_norm.is_finite()
            || (quaternion_norm - 1.0).abs() > 1e-2
            || !(-0.5..=self.max_age_s).contains(&age_s)
        {
            return None;
        }
        Some(PoseStamped { header: odom.header.clone(), pose: odom.pose.pose.clone() })
    }
}

/// Fetch `names` from a node's get_parameters service into `cache`.
async fn refresh_param_cache(
    client: &r2r::Client<GetParameters::Service>,
    names: &[&str],
    cache: &Arc<Mutex<Option<Map<String, Value>>>>,
) {
    let req = GetParameters::Request { names: names.iter().map(|n| n.to_string()).collect() };
    let Ok(fut) = client.request(&req) else { return };
    let Ok(Ok(resp)) = tokio::time::timeout(SERVICE_TIMEOUT, fut).await else { return };
    let mut m = Map::new();
    for (name, value) in names.iter().zip(resp.values.iter()) {
        m.insert(name.to_string(), dto::parameter_value_to_json(value));
    }
    *cache.lock().unwrap() = Some(m);
}

fn publish_datum(publisher: &r2r::Publisher<StringMsg>, lat: f64, lon: f64, bearing: f64, source: &str) {
    let _ = publisher.publish(&json_string(&json!({
        "origin_lat": lat, "origin_lon": lon, "bearing_rad": bearing, "source": source,
    })));
}

struct DatumFallback {
    enabled: bool,
    lat: f64,
    lon: f64,
    bearing: f64,
}

impl DatumFallback {
    fn publish(&self, publisher: &r2r::Publisher<StringMsg>, logger: &str, logged: &mut bool) {
        if self.enabled {
            publish_datum(publisher, self.lat, self.lon, self.bearing, "fallback");
        } else if !*logged {
            r2r::log_warn!(logger, "map datum unavailable; fixed fallback is disabled");
            *logged = true;
        }
    }
}

/// One toLL call; `None` when the service is unavailable or does not answer.
async fn to_ll(client: &r2r::Client<ToLL::Service>, x: f64) -> Option<ToLL::Response> {
    let req = ToLL::Request { map_point: Point { x, y: 0.0, z: 0.0 } };
    let fut = client.request(&req).ok()?;
    tokio::time::timeout(SERVICE_TIMEOUT, fut).await.ok()?.ok()
}

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "flutter_adapter", "")?;
    let logger = node.logger().to_string();
    r2r::log_info!(&logger, "flutter_adapter init (mower_rs)");

    let pose_gate = Arc::new(PoseGate {
        max_age_s: params::f64(&node, "robot_pose_max_age_s", 1.0).max(0.1),
        source_frame: params::string(&node, "robot_pose_source_frame", "map"),
        child_frame: params::string(&node, "robot_pose_child_frame", "base_footprint"),
    });
    let pose_source_topic = params::string(&node, "robot_pose_source_topic", "/odometry/global");
    // A configured fallback datum is opt-in only: publishing a real-looking
    // fixed coordinate when GPS is unavailable can project a saved site
    // onto the wrong physical location.
    let fallback_enabled = params::bool(&node, "enable_map_datum_fallback", false);
    let fallback = (
        params::f64(&node, "map_datum_fallback_lat", 23.6939508),
        params::f64(&node, "map_datum_fallback_lon", 120.5376539),
        params::f64(&node, "map_datum_fallback_bearing_deg", 0.0).to_radians(),
    );

    // ---- map and marker relays ---------------------------------------------
    for (src, name) in MAP_TOPICS {
        let publisher = node.create_publisher::<StringMsg>(&format!("/adapter/map_layers/{name}"), latched())?;
        let mut stream = node.subscribe::<OccupancyGrid>(src, latched())?;
        let logger = logger.clone();
        tokio::spawn(async move {
            while let Some(grid) = stream.next().await {
                let dto = dto::occupancy_grid_to_map_layer(name, &grid);
                if let Err(e) = publisher.publish(&json_string(&dto)) {
                    r2r::log_warn!(&logger, "failed to publish {}: {:?}", name, e);
                }
            }
        });
    }
    for (src, name) in MARKER_TOPICS {
        // Marker arrays are complete layer snapshots. Keep both relay legs
        // latched so an adapter or app that starts later receives the same
        // current layer instead of retaining demo/previous-session data.
        let publisher = node.create_publisher::<StringMsg>(&format!("/adapter/marker_layers/{name}"), latched())?;
        let mut stream = node.subscribe::<MarkerArray>(src, latched())?;
        publisher.publish(&json_string(&dto::empty_marker_layer(name)))?;
        let logger = logger.clone();
        tokio::spawn(async move {
            while let Some(markers) = stream.next().await {
                let dto = dto::marker_array_to_marker_layer(name, &markers);
                if let Err(e) = publisher.publish(&json_string(&dto)) {
                    r2r::log_warn!(&logger, "failed to publish markers {}: {:?}", name, e);
                }
            }
        });
    }

    // ---- robot pose --------------------------------------------------------
    {
        let publisher = node.create_publisher::<PoseStamped>("/adapter/robot_pose", QosProfile::default())?;
        let mut stream = node.subscribe::<Odometry>(&pose_source_topic, QosProfile::default())?;
        let gate = pose_gate.clone();
        tokio::spawn(async move {
            while let Some(odom) = stream.next().await {
                if let Some(pose) = gate.relay(&odom) {
                    let _ = publisher.publish(&pose);
                }
            }
        });
    }

    // ---- coverage settings snapshot (1 Hz) ---------------------------------
    {
        let publisher = node.create_publisher::<StringMsg>("/adapter/coverage_settings", latched())?;
        let coverage_client = node.create_client::<GetParameters::Service>(
            "/boustrophedon_coverage/get_parameters",
            QosProfile::services_default(),
        )?;
        let map_client = node.create_client::<GetParameters::Service>("/map_manage/get_parameters", QosProfile::services_default())?;
        let coverage_ready = r2r::Node::is_available(&coverage_client)?;
        let map_ready = r2r::Node::is_available(&map_client)?;
        let mut timer = node.create_wall_timer(Duration::from_secs(1))?;
        let coverage_cache: Arc<Mutex<Option<Map<String, Value>>>> = Arc::new(Mutex::new(None));
        let map_cache: Arc<Mutex<Option<Map<String, Value>>>> = Arc::new(Mutex::new(None));
        // Parameter snapshots are refreshed in the background once each
        // service is up; every tick publishes the latest cached values.
        {
            let cache = coverage_cache.clone();
            tokio::spawn(async move {
                if coverage_ready.await.is_err() {
                    return;
                }
                loop {
                    refresh_param_cache(&coverage_client, &COVERAGE_PARAM_NAMES, &cache).await;
                    tokio::time::sleep(Duration::from_secs(1)).await;
                }
            });
        }
        {
            let cache = map_cache.clone();
            tokio::spawn(async move {
                if map_ready.await.is_err() {
                    return;
                }
                loop {
                    refresh_param_cache(&map_client, &MAP_PARAM_NAMES, &cache).await;
                    tokio::time::sleep(Duration::from_secs(1)).await;
                }
            });
        }
        tokio::spawn(async move {
            while timer.tick().await.is_ok() {
                let coverage = coverage_cache.lock().unwrap().clone();
                let map = map_cache.lock().unwrap().clone();
                if coverage.is_none() && map.is_none() {
                    continue;
                }
                let dto = dto::params_to_coverage_settings(&coverage.unwrap_or_default(), &map.unwrap_or_default());
                let _ = publisher.publish(&json_string(&dto));
            }
        });
    }

    // ---- zone summaries (2 Hz) ---------------------------------------------
    {
        let publisher = node.create_publisher::<StringMsg>("/adapter/zone_summaries", latched())?;
        let client = node.create_client::<ZoneMapList::Service>("/get_zone_map_list_srv", QosProfile::services_default())?;
        let ready = r2r::Node::is_available(&client)?;
        let logger = logger.clone();
        tokio::spawn(async move {
            if ready.await.is_err() {
                return;
            }
            loop {
                if let Ok(fut) = client.request(&ZoneMapList::Request::default()) {
                    match tokio::time::timeout(SERVICE_TIMEOUT, fut).await {
                        Ok(Ok(resp)) => {
                            let dto = dto::zone_map_list_to_summaries(&resp.zone_map_list);
                            let _ = publisher.publish(&json_string(&dto));
                        }
                        Ok(Err(e)) => r2r::log_warn!(&logger, "zone_map_list call failed: {:?}", e),
                        Err(_) => r2r::log_warn!(&logger, "zone_map_list call timed out"),
                    }
                }
                tokio::time::sleep(Duration::from_millis(500)).await;
            }
        });
    }

    // ---- map datum (satellite geo-reference) -------------------------------
    {
        let publisher = node.create_publisher::<StringMsg>("/adapter/map_datum", latched())?;
        let client = node.create_client::<ToLL::Service>("/toLL", QosProfile::services_default())?;
        let ready = r2r::Node::is_available(&client)?;
        let logger = logger.clone();
        let fallback = DatumFallback { enabled: fallback_enabled, lat: fallback.0, lon: fallback.1, bearing: fallback.2 };
        tokio::spawn(async move {
            let mut fallback_logged = false;
            // Until navsat's toLL service exists the fallback is all we have.
            let mut ready = Box::pin(ready);
            loop {
                tokio::select! {
                    r = &mut ready => { if r.is_err() { return; } break; }
                    _ = tokio::time::sleep(Duration::from_secs(2)) => {
                        fallback.publish(&publisher, &logger, &mut fallback_logged);
                    }
                }
            }
            // Once navsat gives a real datum we lock it (latched pub keeps it live).
            loop {
                let origin = to_ll(&client, 0.0).await.filter(|r| {
                    // navsat datum not established (no valid GPS fix) -> fallback.
                    r.ll_point.latitude.abs() >= 1e-6 || r.ll_point.longitude.abs() >= 1e-6
                });
                if let Some(origin) = origin {
                    if let Some(probe) = to_ll(&client, DATUM_PROBE_M).await {
                        let (lat0, lon0) = (origin.ll_point.latitude, origin.ll_point.longitude);
                        let bearing = dto::bearing_to_north(lat0, lon0, probe.ll_point.latitude, probe.ll_point.longitude);
                        publish_datum(&publisher, lat0, lon0, bearing, "navsat");
                        r2r::log_info!(
                            &logger,
                            "map datum from navsat: ({:.6}, {:.6}), bearing {:.1} deg",
                            lat0,
                            lon0,
                            bearing.to_degrees()
                        );
                        return;
                    }
                }
                fallback.publish(&publisher, &logger, &mut fallback_logged);
                tokio::time::sleep(Duration::from_secs(2)).await;
            }
        });
    }

    // ---- spin until SIGINT / SIGTERM ---------------------------------------
    let running = Arc::new(AtomicBool::new(true));
    let spin = {
        let running = running.clone();
        tokio::task::spawn_blocking(move || {
            while running.load(Ordering::Relaxed) {
                node.spin_once(Duration::from_millis(100));
            }
            drop(node);
        })
    };
    let mut sigterm = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
    tokio::select! {
        _ = tokio::signal::ctrl_c() => {}
        _ = sigterm.recv() => {}
    }
    running.store(false, Ordering::Relaxed);
    let _ = spin.await;
    Ok(())
}
