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
//! Its topics, the zone map list and toLL clients and its timers are on the
//! bus (`m.bus`, step E2). The two parameter services are polled through
//! the r2r node, which is spun for them: parameter services stay on DDS in
//! E2 (docs/BUS_TOPICS.md 3.1).
//!
//! Encoding a 214 KB map grid takes ~60 ms in Python and ~2 ms here.

mod dto;

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use futures::StreamExt;
use mower_bus::{Clock, Qos};
use mower_rs_common::{params, ModuleCtx, ModuleResult};
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

const COVERAGE_PARAM_NAMES: [&str; 7] = [
    "strip_width_m",
    "waypoint_spacing_m",
    "zigzag_angle_deg",
    "zigzag_auto_angle",
    "unknown_as_obstacle",
    "coverage_pattern",
    "boundary_ring",
];
const MAP_PARAM_NAMES: [&str; 1] = ["inflate_radius_m"];

/// Metres along map +X used to derive the datum bearing.
const DATUM_PROBE_M: f64 = 10.0;
/// A service call that does not answer within this is treated as failed.
const SERVICE_TIMEOUT: Duration = Duration::from_secs(3);

fn latched() -> Qos {
    Qos::default().keep_last(1).reliable().transient_local()
}

fn json_string(v: &Value) -> StringMsg {
    StringMsg { data: serde_json::to_string(v).expect("json") }
}

struct PoseGate {
    max_age_s: f64,
    source_frame: String,
    child_frame: String,
    /// Its wall time is what a sample's stamp is aged against.
    clock: Clock,
}

impl PoseGate {
    /// The relayed pose, or `None` when the sample must not become a fresh
    /// 5 Hz pose relay (wrong frames, non-finite, bad quaternion, stale).
    fn relay(&self, odom: &Odometry) -> Option<PoseStamped> {
        let stamp_ns = odom.header.stamp.sec as i64 * 1_000_000_000 + odom.header.stamp.nanosec as i64;
        let age_s = if stamp_ns > 0 { (self.clock.now().ns() - stamp_ns) as f64 / 1e9 } else { f64::INFINITY };
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
    clock: &Clock,
    names: &[&str],
    cache: &Arc<Mutex<Option<Map<String, Value>>>>,
) {
    let req = GetParameters::Request { names: names.iter().map(|n| n.to_string()).collect() };
    let Ok(fut) = client.request(&req) else { return };
    let Ok(Ok(resp)) = clock.timeout(SERVICE_TIMEOUT, fut).await else { return };
    let mut m = Map::new();
    for (name, value) in names.iter().zip(resp.values.iter()) {
        m.insert(name.to_string(), dto::parameter_value_to_json(value));
    }
    *cache.lock().unwrap() = Some(m);
}

fn publish_datum(publisher: &mower_bus::Publisher<StringMsg>, lat: f64, lon: f64, bearing: f64, source: &str) {
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
    fn publish(&self, publisher: &mower_bus::Publisher<StringMsg>, logger: &str, logged: &mut bool) {
        if self.enabled {
            publish_datum(publisher, self.lat, self.lon, self.bearing, "fallback");
        } else if !*logged {
            r2r::log_warn!(logger, "map datum unavailable; fixed fallback is disabled");
            *logged = true;
        }
    }
}

/// One toLL call; `None` when the service is unavailable or does not answer
/// (on the bus: no server at call time, a request it dropped, or no answer
/// in time).
async fn to_ll(client: &mower_bus::Client<ToLL::Request, ToLL::Response>, clock: &Clock, x: f64) -> Option<ToLL::Response> {
    let req = ToLL::Request { map_point: Point { x, y: 0.0, z: 0.0 } };
    let fut = client.request(&req).ok()?;
    clock.timeout(SERVICE_TIMEOUT, fut).await.ok()?.ok()
}

/// The `flutter_adapter` node. Runs either as the `mower_adapter` binary or as one
/// module of `mower_rsd`; the node name, namespace, parameters and every
/// topic / service / action name are the same either way.
///
/// Every endpoint but the two parameter clients is on the bus (crate docs).
pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let mut node = r2r::Node::create(ctx, &m.node_name, &m.namespace)?;
    let logger = node.logger().to_string();
    r2r::log_info!(&logger, "flutter_adapter init (mower_rs)");

    let pose_gate = Arc::new(PoseGate {
        max_age_s: params::f64(&node, "robot_pose_max_age_s", 1.0).max(0.1),
        source_frame: params::string(&node, "robot_pose_source_frame", "map"),
        child_frame: params::string(&node, "robot_pose_child_frame", "base_footprint"),
        clock: m.clock.clone(),
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
        let publisher = m.bus.publisher::<StringMsg>(&m.resolve_topic(&format!("/adapter/map_layers/{name}"))?, latched())?;
        let mut stream = m.bus.subscribe::<OccupancyGrid>(&m.resolve_topic(src)?, latched())?;
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
    // Points of the planned path per zone, from the latest coverage markers;
    // the zone summaries below report them (see dto::coverage_points_by_zone).
    let coverage_points: Arc<Mutex<HashMap<i32, usize>>> = Arc::default();
    for (src, name) in MARKER_TOPICS {
        // Marker arrays are complete layer snapshots. Keep both relay legs
        // latched so an adapter or app that starts later receives the same
        // current layer instead of retaining demo/previous-session data.
        let publisher = m.bus.publisher::<StringMsg>(&m.resolve_topic(&format!("/adapter/marker_layers/{name}"))?, latched())?;
        let mut stream = m.bus.subscribe::<MarkerArray>(&m.resolve_topic(src)?, latched())?;
        publisher.publish(&json_string(&dto::empty_marker_layer(name)))?;
        let logger = logger.clone();
        let coverage_points = (src == "/coverage_path_markers").then(|| coverage_points.clone());
        tokio::spawn(async move {
            while let Some(markers) = stream.next().await {
                if let Some(points) = &coverage_points {
                    *points.lock().unwrap() = dto::coverage_points_by_zone(&markers);
                }
                let dto = dto::marker_array_to_marker_layer(name, &markers);
                if let Err(e) = publisher.publish(&json_string(&dto)) {
                    r2r::log_warn!(&logger, "failed to publish markers {}: {:?}", name, e);
                }
            }
        });
    }

    // ---- robot pose --------------------------------------------------------
    {
        let publisher = m.bus.publisher::<PoseStamped>(&m.resolve_topic("/adapter/robot_pose")?, Qos::default())?;
        let mut stream = m.bus.subscribe::<Odometry>(&m.resolve_topic(&pose_source_topic)?, Qos::default())?;
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
        let publisher = m.bus.publisher::<StringMsg>(&m.resolve_topic("/adapter/coverage_settings")?, latched())?;
        // The parameter services stay on r2r (see the crate docs).
        let coverage_client = node.create_client::<GetParameters::Service>(
            "/boustrophedon_coverage/get_parameters",
            QosProfile::services_default(),
        )?;
        let map_client = node.create_client::<GetParameters::Service>("/map_manage/get_parameters", QosProfile::services_default())?;
        let coverage_ready = r2r::Node::is_available(&coverage_client)?;
        let map_ready = r2r::Node::is_available(&map_client)?;
        let mut timer = m.bus.wall_timer(&m.clock, Duration::from_secs(1))?;
        let coverage_cache: Arc<Mutex<Option<Map<String, Value>>>> = Arc::new(Mutex::new(None));
        let map_cache: Arc<Mutex<Option<Map<String, Value>>>> = Arc::new(Mutex::new(None));
        // Parameter snapshots are refreshed in the background once each
        // service is up; every tick publishes the latest cached values.
        {
            let cache = coverage_cache.clone();
            let clock = m.clock.clone();
            tokio::spawn(async move {
                if coverage_ready.await.is_err() {
                    return;
                }
                loop {
                    refresh_param_cache(&coverage_client, &clock, &COVERAGE_PARAM_NAMES, &cache).await;
                    clock.sleep(Duration::from_secs(1)).await;
                }
            });
        }
        {
            let cache = map_cache.clone();
            let clock = m.clock.clone();
            tokio::spawn(async move {
                if map_ready.await.is_err() {
                    return;
                }
                loop {
                    refresh_param_cache(&map_client, &clock, &MAP_PARAM_NAMES, &cache).await;
                    clock.sleep(Duration::from_secs(1)).await;
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
        let publisher = m.bus.publisher::<StringMsg>(&m.resolve_topic("/adapter/zone_summaries")?, latched())?;
        let client =
            m.bus.client::<ZoneMapList::Request, ZoneMapList::Response>(&m.resolve_service("/get_zone_map_list_srv")?)?;
        let logger = logger.clone();
        let coverage_points = coverage_points.clone();
        let clock = m.clock.clone();
        tokio::spawn(async move {
            if client.wait_for_service().await.is_err() {
                return;
            }
            loop {
                // No server at call time (map_manage restarting) fails here at
                // once, where r2r lost the request and timed out.
                if let Ok(fut) = client.request(&ZoneMapList::Request::default()) {
                    match clock.timeout(SERVICE_TIMEOUT, fut).await {
                        Ok(Ok(resp)) => {
                            let points = coverage_points.lock().unwrap().clone();
                            let dto = dto::zone_map_list_to_summaries(&resp.zone_map_list, &points);
                            let _ = publisher.publish(&json_string(&dto));
                        }
                        Ok(Err(e)) => r2r::log_warn!(&logger, "zone_map_list call failed: {:?}", e),
                        Err(_) => r2r::log_warn!(&logger, "zone_map_list call timed out"),
                    }
                }
                clock.sleep(Duration::from_millis(500)).await;
            }
        });
    }

    // ---- map datum (satellite geo-reference) -------------------------------
    // The task below ends once it has published the datum. On the bus a
    // writer's latched history goes with its last clone (r2r's node kept the
    // writer), so this one keeps the datum for late joiners until the module
    // stops.
    let map_datum = m.bus.publisher::<StringMsg>(&m.resolve_topic("/adapter/map_datum")?, latched())?;
    {
        let publisher = map_datum.clone();
        let client = m.bus.client::<ToLL::Request, ToLL::Response>(&m.resolve_service("/toLL")?)?;
        let logger = logger.clone();
        let fallback = DatumFallback { enabled: fallback_enabled, lat: fallback.0, lon: fallback.1, bearing: fallback.2 };
        let clock = m.clock.clone();
        tokio::spawn(async move {
            let mut fallback_logged = false;
            // Until navsat's toLL service exists the fallback is all we have.
            let mut ready = Box::pin(client.wait_for_service());
            loop {
                tokio::select! {
                    r = &mut ready => { if r.is_err() { return; } break; }
                    _ = clock.sleep(Duration::from_secs(2)) => {
                        fallback.publish(&publisher, &logger, &mut fallback_logged);
                    }
                }
            }
            drop(ready);
            // Once navsat gives a real datum we lock it (latched pub keeps it live).
            loop {
                // navsat datum not established (default (0, 0, 0) answer, or a
                // non-finite one) -> fallback.
                let is_datum =
                    |r: &ToLL::Response| dto::is_navsat_datum_point(r.ll_point.latitude, r.ll_point.longitude);
                let origin = to_ll(&client, &clock, 0.0).await.filter(is_datum);
                if let Some(origin) = origin {
                    if let Some(probe) = to_ll(&client, &clock, DATUM_PROBE_M).await.filter(is_datum) {
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
                clock.sleep(Duration::from_secs(2)).await;
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
    m.shutdown.wait().await;
    running.store(false, Ordering::Relaxed);
    let _ = spin.await;
    drop(map_datum);
    Ok(())
}

/// What flutter_adapter_node.py relays as the robot pose, and its opt-in
/// fixed datum (E9-07: rules test_qos_configuration.py checks only on the
/// Python node).
#[cfg(test)]
mod relay_rules {
    use super::*;
    use mower_bus::Time;

    const T0: Time = Time::from_ns(1_800_000_000_000_000_000);

    fn gate(clock: &Clock) -> PoseGate {
        PoseGate { max_age_s: 1.0, source_frame: "map".into(), child_frame: "base_footprint".into(), clock: clock.clone() }
    }

    /// The EKF's map pose of the base, stamped `age_s` before now.
    fn odom(age_s: f64) -> Odometry {
        let mut o = Odometry::default();
        let (sec, nanosec) = Time::from_ns(T0.ns() - (age_s * 1e9) as i64).to_parts();
        o.header.stamp = r2r::builtin_interfaces::msg::Time { sec, nanosec };
        o.header.frame_id = "map".into();
        o.child_frame_id = "base_footprint".into();
        o.pose.pose.position.x = 1.0;
        o.pose.pose.orientation.w = 1.0;
        o
    }

    /// Only a fresh pose of base_footprint in map is relayed: no other
    /// frame, nothing older than robot_pose_max_age_s or more than 0.5 s
    /// ahead of the robot's clock, nothing non-finite.
    #[test]
    fn only_a_fresh_map_pose_of_the_base_is_relayed() {
        let (clock, _manual) = Clock::manual(T0);
        let g = gate(&clock);
        let relayed = g.relay(&odom(0.0)).expect("fresh");
        assert_eq!((relayed.header.frame_id.as_str(), relayed.pose.position.x), ("map", 1.0));
        let mut other = odom(0.0);
        other.header.frame_id = "odom".into();
        assert!(g.relay(&other).is_none(), "another source frame");
        let mut other = odom(0.0);
        other.child_frame_id = "base_link".into();
        assert!(g.relay(&other).is_none(), "another child frame");
        assert!(g.relay(&odom(0.9)).is_some() && g.relay(&odom(1.1)).is_none(), "the age limit");
        assert!(g.relay(&odom(-0.4)).is_some() && g.relay(&odom(-0.6)).is_none(), "the future limit");
        let mut bad = odom(0.0);
        bad.pose.pose.position.y = f64::NAN;
        assert!(g.relay(&bad).is_none(), "non-finite");
        let mut bad = odom(0.0);
        bad.pose.pose.orientation.w = 0.5;
        assert!(g.relay(&bad).is_none(), "not a unit quaternion");
        let mut unstamped = odom(0.0);
        unstamped.header.stamp = Default::default();
        assert!(g.relay(&unstamped).is_none(), "no stamp");
    }

    /// The configured fixed datum goes out only when it was enabled: a
    /// real-looking coordinate without GPS could put a saved site in the
    /// wrong place (it is off by default, tests/bus.rs).
    #[test]
    fn the_fixed_datum_is_published_only_when_enabled() {
        let bus = mower_bus::Bus::new();
        let publisher = bus.scope("flutter_adapter").publisher::<StringMsg>("/adapter/map_datum", Qos::latched()).unwrap();
        let mut datum = bus.scope("reader").subscribe::<StringMsg>("/adapter/map_datum", Qos::latched().keep_last(10)).unwrap();
        let mut logged = false;
        DatumFallback { enabled: false, lat: 23.6939508, lon: 120.5376539, bearing: 0.0 }.publish(&publisher, "relay_rules", &mut logged);
        assert!(logged && datum.try_recv().is_err(), "off: nothing, once logged");
        DatumFallback { enabled: true, lat: 23.6939508, lon: 120.5376539, bearing: 0.0 }.publish(&publisher, "relay_rules", &mut logged);
        let doc: Value = serde_json::from_str(&datum.try_recv().expect("the fixed datum").data).unwrap();
        assert_eq!((doc["origin_lat"].as_f64(), doc["origin_lon"].as_f64(), doc["source"].as_str()), (Some(23.6939508), Some(120.5376539), Some("fallback")));
    }
}
