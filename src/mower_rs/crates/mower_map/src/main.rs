//! mower_map: the mission map manager (port of
//! `mower_mission/map_manage_node.py`, node name `map_manage`, same services,
//! latched topics, parameters and occupancy bytes).
//!
//! * `/create_free_space`   -- rasterise the recorded zones (`/get_record_zone_list`)
//!   into `/free_space` + per-zone maps, invalidate risk / channel
//! * `/create_risk_map`     -- `/get_risk_zone_list` polygons into `/risk_map`
//! * `/create_chennal_map`  -- `/get_chennal_path_list` corridors into `/chennal_map`
//! * `/import_image_mask`   -- an app-drawn mask as the active zone (clipped to the
//!   collected free space), `/restore_free_space_coverage` undoes it
//! * `/get_zone_map_list_srv`, `/map_manage/{get,set,list,describe}_parameters`
//!
//! Every generated map has an inflated twin (`inflate_radius_m`, never below
//! 0.75 m) and the two Nav2 snapshots `/map_grid` (raw geometry) and
//! `/map_grid_global` (configuration space) are rebuilt fail-closed on each
//! change: held fully occupied until a matching risk map exists. Mutations go
//! through the nav server's mission operation lock ([`mower_rs_common::guard`]).
//!
//! The raster primitives are pixel-exact OpenCV 4.6.0 ports ([`raster`]); the
//! grid maths mirrors numpy ([`grid`]).

mod grid;
mod raster;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use futures::StreamExt;
use mower_rs_common::guard::{Guard, RELEASE_UNCONFIRMED};
use mower_rs_common::params;
use r2r::builtin_interfaces::msg::Time;
use r2r::geometry_msgs::msg::Pose;
use r2r::mower_interface::msg::ZoneMap;
use r2r::mower_interface::srv::{ChennalPathList, GetZoneList, ImportImageMask, MissionOperationLock, ZoneMapList};
use r2r::nav_msgs::msg::{MapMetaData, OccupancyGrid};
use r2r::rcl_interfaces::msg::{ListParametersResult, Parameter, ParameterDescriptor, ParameterValue, SetParametersResult};
use r2r::rcl_interfaces::srv::{DescribeParameters, GetParameterTypes, GetParameters, ListParameters, SetParameters, SetParametersAtomically};
use r2r::std_msgs::msg::{Bool, Header};
use r2r::std_srvs::srv::Trigger;
use r2r::visualization_msgs::msg::MarkerArray;
use r2r::QosProfile;
use tokio::sync::Mutex;

use crate::grid::Geometry;

const MIN_SAFE_INFLATE_RADIUS_M: f64 = 0.75;

// rcl_interfaces/msg/ParameterType
const PARAMETER_NOT_SET: u8 = 0;
const PARAMETER_BOOL: u8 = 1;
const PARAMETER_DOUBLE: u8 = 3;

fn stamp_now() -> Time {
    let ns = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_nanos() as i64).unwrap_or(0);
    Time { sec: (ns / 1_000_000_000) as i32, nanosec: (ns % 1_000_000_000) as u32 }
}

fn latched() -> QosProfile {
    QosProfile::default().keep_last(1).reliable().transient_local()
}

/// An OccupancyGrid plus the resolution as the node computed it. rclpy keeps
/// the Python float in the float32 field until the message is serialised, so
/// every internal computation (cell indices, kernel radii, fusion) sees the
/// exact value (0.05, or the request's `resolution_m`), not its float32
/// rounding; the published message carries the float32.
#[derive(Clone)]
struct Map {
    msg: OccupancyGrid,
    res: f64,
}

impl Map {
    fn new(msg: OccupancyGrid, res: f64) -> Map {
        Map { msg, res }
    }
    fn with_data(&self, data: Vec<i8>) -> Map {
        Map { msg: OccupancyGrid { header: self.msg.header.clone(), info: self.msg.info.clone(), data }, res: self.res }
    }
    fn width(&self) -> u32 {
        self.msg.info.width
    }
    fn height(&self) -> u32 {
        self.msg.info.height
    }
    fn data(&self) -> &[i8] {
        &self.msg.data
    }
}

/// A ZoneMap with the resolution of its mask map (see [`Map`]).
#[derive(Clone)]
struct Zone {
    msg: ZoneMap,
    res: f64,
}

fn geometry_of(m: &Map) -> Geometry {
    let q = &m.msg.info.origin.orientation;
    Geometry {
        width: m.msg.info.width,
        height: m.msg.info.height,
        resolution: m.res,
        origin_x: m.msg.info.origin.position.x,
        origin_y: m.msg.info.origin.position.y,
        origin_yaw: grid::yaw_from_quaternion(q.x, q.y, q.z, q.w),
    }
}

fn frame_or_map(frame: &str) -> &str {
    if frame.is_empty() { "map" } else { frame }
}

fn occupancy_grid(header: Header, resolution: f64, width: u32, height: u32, origin_x: f64, origin_y: f64, data: Vec<i8>) -> Map {
    let mut origin = Pose::default();
    origin.position.x = origin_x;
    origin.position.y = origin_y;
    origin.position.z = 0.0;
    origin.orientation.w = 1.0;
    Map::new(OccupancyGrid { header, info: MapMetaData { map_load_time: Time::default(), resolution: resolution as f32, width, height, origin }, data }, resolution)
}

fn free_cells(m: &Map) -> usize {
    m.data().iter().filter(|&&v| v == 0).count()
}

/// `build_demo_map`: the synthetic startup extent, fully occupied so Nav2
/// cannot infer traversable space before the first snapshots.
fn build_demo_map() -> Map {
    let (resolution, width, height) = (0.1, 400u32, 400u32);
    let header = Header { stamp: Time::default(), frame_id: "map".into() };
    occupancy_grid(header, resolution, width, height, -20.0, -20.0, vec![100; (width * height) as usize])
}

struct Pubs {
    free_space: r2r::Publisher<OccupancyGrid>,
    free_space_inflated: r2r::Publisher<OccupancyGrid>,
    risk_map: r2r::Publisher<OccupancyGrid>,
    risk_map_inflated: r2r::Publisher<OccupancyGrid>,
    chennal_map: r2r::Publisher<OccupancyGrid>,
    chennal_map_inflated: r2r::Publisher<OccupancyGrid>,
    nav_base: r2r::Publisher<OccupancyGrid>,
    nav_global: r2r::Publisher<OccupancyGrid>,
}

struct App {
    logger: String,
    guard: Guard,
    inflate_radius_m: f64,
    chennal_width_m: f64,
    use_sim_time: bool,
    /// rclpy declares this on every node; listed and readable like there.
    start_type_description_service: bool,
    zone_map_list: Vec<Zone>,
    base_map: Option<Map>,
    /// Robot-collected free space, never overwritten by an image import.
    collected_free_space: Option<Map>,
    risk_map: Option<Map>,
    risk_map_inflated: Option<Map>,
    chennal_map: Option<Map>,
    free_zone_backup: Option<Vec<Zone>>,
    free_risk_backup: Option<Map>,
    map_msg: Map,
    global_map_msg: Map,
    pubs: Pubs,
    risk_zone_list_client: r2r::Client<GetZoneList::Service>,
    record_zone_list_client: r2r::Client<GetZoneList::Service>,
    chennal_path_list_client: r2r::Client<ChennalPathList::Service>,
}

type Shared = Arc<Mutex<App>>;

impl App {
    fn info(&self, msg: impl AsRef<str>) {
        r2r::log_info!(&self.logger, "{}", msg.as_ref());
    }
    fn warn(&self, msg: impl AsRef<str>) {
        r2r::log_warn!(&self.logger, "{}", msg.as_ref());
    }
    fn error(&self, msg: impl AsRef<str>) {
        r2r::log_error!(&self.logger, "{}", msg.as_ref());
    }

    fn inflate_radius(&self, override_m: Option<f64>) -> f64 {
        override_m.unwrap_or(self.inflate_radius_m)
    }

    // ------------------------------------------------------- map builders

    /// `_create_free_space_inflated`: erode the free cells by the radius.
    fn create_free_space_inflated(&self, map: &Map, override_m: Option<f64>) -> Result<Map, String> {
        let data = grid::erode_free_space_grid(map.data(), map.width(), map.height(), map.res, self.inflate_radius(override_m))?;
        Ok(map.with_data(data))
    }

    /// `_create_risk_map_inflated`: dilate the occupied cells by the radius.
    fn create_risk_map_inflated(&self, map: &Map, override_m: Option<f64>) -> Map {
        let r_cells = grid::radius_cells(self.inflate_radius(override_m), map.res);
        match grid::dilate_risk(map.data(), map.width(), map.height(), r_cells) {
            Some(data) => map.with_data(data),
            None => map.clone(),
        }
    }

    /// `_create_chennal_map_inflated`: erode the corridor by the radius.
    fn create_chennal_map_inflated(&self, map: &Map, override_m: Option<f64>) -> Map {
        let r_cells = grid::radius_cells(self.inflate_radius(override_m), map.res);
        match grid::erode_channel(map.data(), map.width(), map.height(), r_cells) {
            Some(data) => map.with_data(data),
            None => map.clone(),
        }
    }

    fn occupied_navigation_map(&self, base: &Map) -> Map {
        let mut nav = base.with_data(vec![100; (base.width() as usize) * (base.height() as usize)]);
        nav.msg.header.stamp = stamp_now();
        nav
    }

    /// `_navigation_base_with_channel`: union a matching channel into the raw
    /// free-space geometry.
    fn navigation_base_with_channel(&self, base: &Map, chennal: Option<&Map>) -> Result<Map, String> {
        let Some(ch) = chennal else {
            return Ok(base.with_data(grid::union_free_space_grids(base.data(), None)?));
        };
        let base_yaw = geometry_of(base).origin_yaw;
        let channel_yaw = geometry_of(ch).origin_yaw;
        let matches = frame_or_map(&base.msg.header.frame_id) == frame_or_map(&ch.msg.header.frame_id)
            && ch.height() == base.height()
            && ch.width() == base.width()
            && grid::isclose_abs(ch.res, base.res)
            && grid::isclose_abs(ch.msg.info.origin.position.x, base.msg.info.origin.position.x)
            && grid::isclose_abs(ch.msg.info.origin.position.y, base.msg.info.origin.position.y)
            && grid::yaw_delta(channel_yaw, base_yaw).abs() <= 1e-9;
        if !matches {
            return Err("base and channel grid geometries must match".into());
        }
        Ok(base.with_data(grid::union_free_space_grids(base.data(), Some(ch.data()))?))
    }

    /// `_fuse_navigation_map`: one fail-closed Nav2 snapshot in base geometry.
    fn fuse_navigation_map(&self, base: &Map, risk: Option<&Map>, topic: &str) -> (Map, bool) {
        let Some(risk) = risk else {
            self.info(format!("{topic} held occupied until its risk map is ready"));
            return (self.occupied_navigation_map(base), false);
        };
        let fused = (|| -> Result<Vec<i8>, String> {
            let base_frame = frame_or_map(&base.msg.header.frame_id);
            let risk_frame = frame_or_map(&risk.msg.header.frame_id);
            if base_frame != risk_frame {
                return Err(format!("grid frame mismatch: {base_frame} != {risk_frame}"));
            }
            grid::fuse_navigation_grid(base.data(), &geometry_of(base), Some((risk.data(), &geometry_of(risk))))
        })();
        match fused {
            Ok(data) => {
                let mut nav = base.with_data(data);
                nav.msg.header.stamp = stamp_now();
                (nav, true)
            }
            Err(e) => {
                self.error(format!("cannot fuse {topic} safely; using occupied map: {e}"));
                (self.occupied_navigation_map(base), false)
            }
        }
    }

    /// `_build_navigation_maps`: independent local (raw) and global
    /// (configuration-space) maps.
    fn build_navigation_maps(
        &self,
        base: &Map,
        risk: Option<&Map>,
        risk_inflated: Option<&Map>,
        chennal: Option<&Map>,
        override_m: Option<f64>,
    ) -> (Map, Map, bool, bool) {
        let combined = match self.navigation_base_with_channel(base, chennal) {
            Ok(c) => c,
            Err(e) => {
                self.error(format!("cannot union channel into navigation base: {e}"));
                let occupied = self.occupied_navigation_map(base);
                return (occupied.clone(), occupied, false, false);
            }
        };
        let (local_map, local_ok) = self.fuse_navigation_map(&combined, risk, "/map_grid");
        let (global_map, global_ok) = match self.create_free_space_inflated(&combined, override_m) {
            Ok(global_base) => self.fuse_navigation_map(&global_base, risk_inflated, "/map_grid_global"),
            Err(e) => {
                self.error(format!("cannot build /map_grid_global safely; using occupied map: {e}"));
                (self.occupied_navigation_map(&combined), false)
            }
        };
        (local_map, global_map, local_ok, global_ok)
    }

    fn commit_navigation_maps(&mut self, local: Map, global: Map) {
        let _ = self.pubs.nav_base.publish(&local.msg);
        let _ = self.pubs.nav_global.publish(&global.msg);
        self.map_msg = local;
        self.global_map_msg = global;
    }

    fn publish_navigation_maps(&mut self, override_m: Option<f64>) -> bool {
        let Some(base) = self.base_map.clone() else { return false };
        let (local, global, l, g) = self.build_navigation_maps(&base, self.risk_map.as_ref(), self.risk_map_inflated.as_ref(), self.chennal_map.as_ref(), override_m);
        self.commit_navigation_maps(local, global);
        l && g
    }

    fn publish_current_navigation_maps(&self) {
        let stamp = stamp_now();
        let mut m = self.map_msg.msg.clone();
        let mut g = self.global_map_msg.msg.clone();
        m.header.stamp = stamp.clone();
        g.header.stamp = stamp;
        let _ = self.pubs.nav_base.publish(&m);
        let _ = self.pubs.nav_global.publish(&g);
    }

    /// `refresh_inflated_maps`: recalculate every inflated map for a new radius.
    fn refresh_inflated_maps(&mut self, override_m: Option<f64>) -> Result<(), String> {
        let mut refreshed: Vec<&str> = Vec::new();
        if let Some(base) = self.base_map.clone() {
            let mut zones = std::mem::take(&mut self.zone_map_list);
            for z in zones.iter_mut() {
                match self.create_free_space_inflated(&Map::new(z.msg.mask_map.clone(), z.res), override_m) {
                    Ok(m) => z.msg.mask_map_inflated = m.msg,
                    Err(e) => {
                        self.zone_map_list = zones;
                        return Err(e);
                    }
                }
            }
            self.zone_map_list = zones;
            let source = self.collected_free_space.clone().unwrap_or(base);
            let inflated = self.create_free_space_inflated(&source, override_m)?;
            let _ = self.pubs.free_space_inflated.publish(&inflated.msg);
            refreshed.push("/free_space_inflated");
        }
        if let Some(risk) = self.risk_map.clone() {
            let inflated = self.create_risk_map_inflated(&risk, override_m);
            let _ = self.pubs.risk_map_inflated.publish(&inflated.msg);
            self.risk_map_inflated = Some(inflated);
            refreshed.push("/risk_map_inflated");
        }
        if let Some(ch) = self.chennal_map.clone() {
            let inflated = self.create_chennal_map_inflated(&ch, override_m);
            let _ = self.pubs.chennal_map_inflated.publish(&inflated.msg);
            refreshed.push("/chennal_map_inflated");
        }
        if self.base_map.is_some() {
            self.publish_navigation_maps(override_m);
            refreshed.push("/map_grid");
            refreshed.push("/map_grid_global");
        }
        let radius = self.inflate_radius(override_m);
        if refreshed.is_empty() {
            self.info(format!("inflate_radius_m updated to {radius:.2}; no generated maps to refresh yet"));
        } else {
            self.info(format!("inflate_radius_m updated to {radius:.2}; refreshed {}", refreshed.join(", ")));
        }
        Ok(())
    }

    // ------------------------------------------------------------ services

    /// `_create_risk_map_sync`.
    async fn create_risk_map_sync(&mut self) -> Result<(bool, String), String> {
        if !service_available(&self.risk_zone_list_client, 5.0).await {
            self.error("風險區域列表服務不可用");
            return Ok((false, "風險區域列表服務不可用".into()));
        }
        let resp = match call_with_timeout(self.risk_zone_list_client.request(&GetZoneList::Request::default()), 30.0).await {
            Some(r) => r,
            None => {
                self.error("獲取風險區域列表逾時");
                return Ok((false, "獲取風險區域列表逾時".into()));
            }
        };
        if !resp.success {
            self.error(format!("獲取風險區域列表失敗: {}", resp.message));
            return Ok((false, format!("獲取風險區域列表失敗: {}", resp.message)));
        }
        let Some(risk_map) = self.generate_risk_map(&resp.zone_list) else {
            return Ok((false, "生成風險地圖失敗（可能尚未建立自由空間，請先呼叫 /create_free_space）".into()));
        };
        let risk_inflated = self.create_risk_map_inflated(&risk_map, None);
        if risk_inflated.data().is_empty() {
            return Ok((false, "生成膨脹風險地圖失敗".into()));
        }
        let base = self.base_map.clone().expect("base map exists when the risk map could be generated");
        let (local, global, l, g) = self.build_navigation_maps(&base, Some(&risk_map), Some(&risk_inflated), self.chennal_map.as_ref(), None);
        if !l || !g {
            return Ok((false, "風險地圖已生成，但 Nav2 安全地圖生成失敗".into()));
        }
        let _ = self.pubs.risk_map.publish(&risk_map.msg);
        let _ = self.pubs.risk_map_inflated.publish(&risk_inflated.msg);
        self.risk_map = Some(risk_map);
        self.risk_map_inflated = Some(risk_inflated);
        self.commit_navigation_maps(local, global);
        let n = resp.zone_list.markers.len();
        self.info(format!("成功創建風險地圖，包含 {n} 個風險區域"));
        Ok((true, format!("成功創建風險地圖，包含 {n} 個風險區域")))
    }

    /// `_generate_risk_map`: base geometry, every risk polygon burnt in.
    fn generate_risk_map(&self, zones: &MarkerArray) -> Option<Map> {
        let Some(base) = &self.base_map else {
            self.error("風險地圖需要先建立自由空間（請先呼叫 /create_free_space）");
            return None;
        };
        let polys: Vec<Vec<(f64, f64)>> = zones.markers.iter().map(|m| m.points.iter().map(|p| (p.x, p.y)).collect()).collect();
        let data = grid::risk_map_data(&geometry_of(base), &polys);
        let mut risk = base.with_data(data);
        risk.msg.header.frame_id = "map".into();
        Some(risk)
    }

    /// `_create_free_space_sync`.
    async fn create_free_space_sync(&mut self) -> Result<(bool, String), String> {
        if !service_available(&self.record_zone_list_client, 5.0).await {
            self.error("記錄區域列表服務不可用");
            return Ok((false, "記錄區域列表服務不可用".into()));
        }
        let resp = match call_with_timeout(self.record_zone_list_client.request(&GetZoneList::Request::default()), 30.0).await {
            Some(r) => r,
            None => {
                self.error("獲取記錄區域列表逾時");
                return Ok((false, "獲取記錄區域列表逾時".into()));
            }
        };
        if !resp.success {
            self.error(format!("獲取記錄區域列表失敗: {}", resp.message));
            return Ok((false, format!("獲取記錄區域列表失敗: {}", resp.message)));
        }
        let Some((overall, zone_maps)) = self.create_zone_maps_and_freespace(&resp.zone_list)? else {
            self.error("生成自由空間失敗");
            return Ok((false, "生成自由空間失敗".into()));
        };
        let overall_inflated = self.create_free_space_inflated(&overall, None)?;
        if overall_inflated.data().is_empty() {
            self.error("生成膨脹自由空間失敗");
            return Ok((false, "生成膨脹自由空間失敗".into()));
        }
        let (local, global, _, _) = self.build_navigation_maps(&overall, None, None, None, None);

        // Commit only after every candidate artifact is valid.
        let n = zone_maps.len();
        self.zone_map_list = zone_maps;
        self.base_map = Some(overall.clone());
        self.collected_free_space = Some(overall.clone());
        self.risk_map = None;
        self.risk_map_inflated = None;
        self.chennal_map = None;
        self.free_zone_backup = None;
        self.free_risk_backup = None;
        let _ = self.pubs.free_space.publish(&overall.msg);
        let _ = self.pubs.free_space_inflated.publish(&overall_inflated.msg);
        self.commit_navigation_maps(local, global);
        self.info(format!("成功創建自由空間，包含 {n} 個區域"));
        Ok((true, format!("成功創建自由空間，包含 {n} 個區域")))
    }

    /// `_create_zone_maps_and_freespace`; `Err` = a raised ValueError,
    /// `Ok(None)` = the Python `return None` after a warning.
    fn create_zone_maps_and_freespace(&self, zone_list: &MarkerArray) -> Result<Option<(Map, Vec<Zone>)>, String> {
        let zones: Vec<(i32, Vec<(f64, f64)>)> = zone_list.markers.iter().map(|m| (m.id, m.points.iter().map(|p| (p.x, p.y)).collect())).collect();
        let raster = match grid::zone_maps_and_freespace(&zones) {
            Ok(r) => r,
            Err(msg) => {
                self.warn(msg);
                return Ok(None);
            }
        };
        for (id, n) in &raster.skipped {
            self.warn(format!("zone {id}: 僅 {n} 個點，無法構成多邊形，略過"));
        }
        let g = raster.geometry;
        let header = Header { stamp: stamp_now(), frame_id: "map".into() };
        let masked_template = occupancy_grid(header, g.resolution, g.width, g.height, g.origin_x, g.origin_y, Vec::new());
        let mut zone_maps = Vec::with_capacity(raster.zones.len());
        for (id, data) in raster.zones {
            let mut zone = ZoneMap::default();
            zone.zone_id = id;
            let mask = masked_template.with_data(data);
            zone.mask_map_inflated = self.create_free_space_inflated(&mask, None)?.msg;
            zone.mask_map = mask.msg;
            zone_maps.push(Zone { msg: zone, res: g.resolution });
            self.info(format!("成功創建zone map，包含 {} 個區域", zone_maps.len()));
        }
        let masked = masked_template.with_data(raster.free_space);
        Ok(Some((masked, zone_maps)))
    }

    /// `_create_chennal_map_sync`.
    async fn create_chennal_map_sync(&mut self) -> Result<(bool, String), String> {
        if !service_available(&self.chennal_path_list_client, 5.0).await {
            self.error("通道路徑列表服務不可用");
            return Ok((false, "通道路徑列表服務不可用".into()));
        }
        let resp = match call_with_timeout(self.chennal_path_list_client.request(&ChennalPathList::Request::default()), 30.0).await {
            Some(r) => r,
            None => {
                self.error("獲取通道路徑列表逾時");
                return Ok((false, "獲取通道路徑列表逾時".into()));
            }
        };
        if !resp.success {
            self.error(format!("獲取通道路徑列表失敗: {}", resp.message));
            return Ok((false, format!("獲取通道路徑列表失敗: {}", resp.message)));
        }
        let Some(chennal_map) = self.generate_chennal_map(&resp.chennal_path_array) else {
            self.error("生成通道地圖失敗");
            return Ok((false, "生成通道地圖失敗".into()));
        };
        let chennal_inflated = self.create_chennal_map_inflated(&chennal_map, None);
        if chennal_inflated.data().is_empty() {
            self.error("生成膨脹通道地圖失敗");
            return Ok((false, "生成膨脹通道地圖失敗".into()));
        }
        let base = self.base_map.clone().expect("base map exists when the channel map could be generated");
        let (local, global, l, g) = self.build_navigation_maps(&base, self.risk_map.as_ref(), self.risk_map_inflated.as_ref(), Some(&chennal_map), None);
        if !l || !g {
            self.error("通道無法安全合併至 Nav2 地圖");
            return Ok((false, "通道已生成，但無法安全合併至 Nav2 地圖".into()));
        }
        let _ = self.pubs.chennal_map.publish(&chennal_map.msg);
        let _ = self.pubs.chennal_map_inflated.publish(&chennal_inflated.msg);
        self.chennal_map = Some(chennal_map);
        self.commit_navigation_maps(local, global);
        let count = resp.chennal_path_array.markers.len();
        let message = format!("成功創建通道地圖，包含 {count} 條通道");
        self.info(&message);
        Ok((true, message))
    }

    /// `_generate_chennal_map`.
    fn generate_chennal_map(&self, paths: &MarkerArray) -> Option<Map> {
        if paths.markers.is_empty() {
            self.warn("没有通道路径数据");
            return None;
        }
        let Some(base) = &self.base_map else {
            self.error("没有基础地图");
            return None;
        };
        if paths.markers.iter().any(|m| m.points.len() < 2) {
            self.error("每條通道路徑至少需要兩個點；拒絕不完整資料");
            return None;
        }
        if paths.markers.iter().all(|m| m.points.is_empty()) {
            self.warn("没有有效的路径点数据");
            return None;
        }
        let width = self.chennal_width_m;
        if !width.is_finite() || width <= 0.0 {
            self.error("chennal_width_m 必須是有限正數");
            return None;
        }
        let base_frame = frame_or_map(&base.msg.header.frame_id).to_string();
        let segments: Vec<(String, Vec<(f64, f64)>)> = paths.markers.iter().map(|m| (m.header.frame_id.clone(), m.points.iter().map(|p| (p.x, p.y)).collect())).collect();
        match grid::chennal_map_data(&geometry_of(base), &base_frame, &segments, width) {
            Ok(data) => {
                let mut m = base.with_data(data);
                m.msg.header = Header { stamp: stamp_now(), frame_id: base.msg.header.frame_id.clone() };
                Some(m)
            }
            Err(e) => {
                self.error(format!("生成通道地图时发生错误: {e}"));
                None
            }
        }
    }

    /// `_create_image_mask_maps`.
    fn create_image_mask_maps(&self, req: &ImportImageMask::Request) -> Result<(Map, Map, Zone, f64), String> {
        if req.mask_encoding != "base64_u8_row_major" {
            return Err("mask_encoding must be base64_u8_row_major".into());
        }
        let (width, height) = (req.width, req.height);
        let resolution = req.resolution_m;
        if width == 0 || height == 0 {
            return Err("圖片 mask 尺寸無效".into());
        }
        if resolution <= 0.0 {
            return Err("resolution_m must be > 0".into());
        }
        let free_mask = grid::decode_u8_mask(&req.free_mask_data, width, height, "free_mask_data", false)?;
        let risk_mask = grid::decode_u8_mask(&req.risk_mask_data, width, height, "risk_mask_data", true)?;
        if !free_mask.iter().any(|&v| v == 255) {
            return Err("free_mask_data 沒有可割草白色區域".into());
        }
        let q = &req.robot_pose_map.orientation;
        let placement = grid::ImagePlacement {
            resolution,
            robot_x: req.robot_pose_map.position.x,
            robot_y: req.robot_pose_map.position.y,
            robot_yaw: grid::yaw_from_quaternion(q.x, q.y, q.z, q.w),
            start_x: req.start_x_m,
            start_y: req.start_y_m,
            image_heading: req.image_heading_rad,
        };
        let r = grid::rasterize_image_masks(&free_mask, &risk_mask, width, height, &placement);
        let had_collected = self.collected_free_space.is_some();
        let free_grid = match &self.collected_free_space {
            Some(c) => grid::clip_free_grid_to_collected(&r.free_grid, r.width, r.height, r.min_x, r.min_y, resolution, c.data(), &geometry_of(c)),
            None => r.free_grid,
        };
        if had_collected && !free_grid.iter().any(|&v| v == 0) {
            return Err("圖片與採集的 freespace 沒有重疊，無法產生路徑".into());
        }
        let mut header = req.robot_pose_header.clone();
        header.frame_id = frame_or_map(&header.frame_id).to_string();
        if header.frame_id != "map" {
            return Err("robot_pose_header.frame_id must be map".into());
        }
        header.stamp = stamp_now();
        let free_map = occupancy_grid(header.clone(), resolution, r.width, r.height, r.min_x, r.min_y, free_grid);
        let risk_map = occupancy_grid(header.clone(), resolution, r.width, r.height, r.min_x, r.min_y, r.risk_grid);
        let mut zone = ZoneMap::default();
        zone.header = header;
        zone.zone_id = if req.zone_id > 0 { req.zone_id } else { 9001 };
        zone.mask_map = free_map.msg.clone();
        let area_m2 = free_cells(&free_map) as f64 * resolution * resolution;
        Ok((free_map, risk_map, Zone { msg: zone, res: resolution }, area_m2))
    }

    /// `import_image_mask_srv` body (inside the mutation guard).
    fn import_image_mask(&mut self, req: &ImportImageMask::Request) -> ImportImageMask::Response {
        let fail = |message: &str, zone_id: i32, area_m2: f64| ImportImageMask::Response { success: false, message: message.into(), zone_id, area_m2 };
        let (free_map, risk_map, mut zone_map, area_m2) = match self.create_image_mask_maps(req) {
            Ok(v) => v,
            Err(e) => return fail(&e, req.zone_id, 0.0),
        };
        // An erosion error means impossible geometry: the Python node's
        // generic "unexpected error" answer.
        let free_space_inflated = match self.create_free_space_inflated(&free_map, None) {
            Ok(m) => m,
            Err(e) => {
                self.error(format!("import image mask failed: {e:?}"));
                return fail("圖片 mask 匯入失敗", req.zone_id, 0.0);
            }
        };
        let risk_map_inflated = self.create_risk_map_inflated(&risk_map, None);
        if free_space_inflated.data().is_empty() || risk_map_inflated.data().is_empty() {
            return fail("圖片 mask 的衍生安全地圖生成失敗", zone_map.msg.zone_id, 0.0);
        }
        let any_safe = free_space_inflated.data().iter().zip(risk_map_inflated.data()).any(|(&f, &r)| f == 0 && r == 0);
        if !any_safe {
            return fail("圖片 mask 安全內縮後沒有可用割草區域", zone_map.msg.zone_id, 0.0);
        }
        zone_map.msg.mask_map_inflated = free_space_inflated.msg.clone();

        let candidate_base = match &self.collected_free_space {
            None => free_map.clone(),
            Some(c) => c.clone(),
        };
        let (local, global, l, g) = self.build_navigation_maps(&candidate_base, Some(&risk_map), Some(&risk_map_inflated), self.chennal_map.as_ref(), None);
        if !l || !g {
            return fail("圖片已解析，但 Nav2 安全地圖生成失敗", zone_map.msg.zone_id, area_m2);
        }

        if self.free_zone_backup.is_none() {
            self.free_zone_backup = Some(std::mem::take(&mut self.zone_map_list));
            self.free_risk_backup = self.risk_map.clone();
        }
        let zone_id = zone_map.msg.zone_id;
        self.zone_map_list = vec![zone_map];
        self.risk_map = Some(risk_map.clone());
        self.risk_map_inflated = Some(risk_map_inflated.clone());
        self.base_map = Some(candidate_base);
        if self.collected_free_space.is_none() {
            let _ = self.pubs.free_space.publish(&free_map.msg);
            let _ = self.pubs.free_space_inflated.publish(&free_space_inflated.msg);
        }
        let _ = self.pubs.risk_map.publish(&risk_map.msg);
        let _ = self.pubs.risk_map_inflated.publish(&risk_map_inflated.msg);
        self.commit_navigation_maps(local, global);
        self.info(format!("imported image mask zone={zone_id}, area={area_m2:.2} m^2, size={}x{}", free_map.width(), free_map.height()));
        ImportImageMask::Response { success: true, message: "圖片 mask 匯入成功".into(), zone_id, area_m2 }
    }

    /// `restore_free_space_srv` body (inside the mutation guard).
    fn restore_free_space(&mut self) -> (bool, String) {
        if self.free_zone_backup.is_none() {
            return (true, "目前已是自由空間覆蓋".into());
        }
        let Some(collected) = self.collected_free_space.clone() else {
            return (false, "沒有採集的自由空間可還原；保留目前圖片任務".into());
        };
        let Some(restored_risk) = self.free_risk_backup.clone() else {
            return (false, "原自由空間缺少風險地圖；保留目前圖片任務".into());
        };
        let risk_inflated = self.create_risk_map_inflated(&restored_risk, None);
        if risk_inflated.data().is_empty() {
            return (false, "無法還原原自由空間的風險地圖；保留目前圖片任務".into());
        }
        let (local, global, l, g) = self.build_navigation_maps(&collected, Some(&restored_risk), Some(&risk_inflated), self.chennal_map.as_ref(), None);
        if !l || !g {
            return (false, "無法安全還原 Nav2 地圖；保留目前圖片任務".into());
        }
        self.zone_map_list = self.free_zone_backup.take().unwrap_or_default();
        self.free_risk_backup = None;
        let _ = self.pubs.risk_map.publish(&restored_risk.msg);
        let _ = self.pubs.risk_map_inflated.publish(&risk_inflated.msg);
        self.risk_map = Some(restored_risk);
        self.risk_map_inflated = Some(risk_inflated);
        self.base_map = Some(collected);
        self.commit_navigation_maps(local, global);
        self.info("restored freespace coverage (image discarded)");
        (true, "已還原為完整自由空間覆蓋".into())
    }

    // ---------------------------------------------------------- parameters

    fn parameter_value(&self, name: &str) -> ParameterValue {
        let mut v = ParameterValue::default();
        match name {
            "inflate_radius_m" => {
                v.type_ = PARAMETER_DOUBLE;
                v.double_value = self.inflate_radius_m;
            }
            "chennal_width_m" => {
                v.type_ = PARAMETER_DOUBLE;
                v.double_value = self.chennal_width_m;
            }
            "use_sim_time" => {
                v.type_ = PARAMETER_BOOL;
                v.bool_value = self.use_sim_time;
            }
            "start_type_description_service" => {
                v.type_ = PARAMETER_BOOL;
                v.bool_value = self.start_type_description_service;
            }
            _ => v.type_ = PARAMETER_NOT_SET,
        }
        v
    }

    /// `on_parameters_changed` (the pre-set validation callback) followed by
    /// the assignment rclpy performs when it succeeds.
    async fn set_parameters(&mut self, params: &[Parameter]) -> SetParametersResult {
        let ok = SetParametersResult { successful: true, reason: String::new() };
        let reject = |reason: &str| SetParametersResult { successful: false, reason: reason.into() };
        for p in params {
            let Some(expected) = declared_type(&p.name) else {
                return reject(&format!("Parameter not declared: {}", p.name));
            };
            // rclpy checks the declared type before any callback runs
            if p.value.type_ != expected {
                return reject(&format!("Wrong parameter type, expected 'Type.{}' got 'Type.{}'", type_name(expected), type_name(p.value.type_)));
            }
        }
        if params.iter().any(|p| p.name == "inflate_radius_m") && self.guard.blocked() {
            return reject("inflate_radius_m cannot change while navigation is active or unknown");
        }
        let mut next_inflate: Option<f64> = None;
        for p in params {
            if p.name != "inflate_radius_m" {
                continue;
            }
            let Some(value) = number_of(&p.value) else {
                return reject("inflate_radius_m must be a number");
            };
            if !value.is_finite() {
                return reject("inflate_radius_m must be finite");
            }
            if value < MIN_SAFE_INFLATE_RADIUS_M {
                return reject(&format!("inflate_radius_m must be >= {MIN_SAFE_INFLATE_RADIUS_M:.2} m"));
            }
            next_inflate = Some(value);
        }
        if let Some(radius) = next_inflate {
            if let Err(message) = self.guard.acquire("refresh inflated maps").await {
                return reject(&message);
            }
            let refresh_error = match self.refresh_inflated_maps(Some(radius)) {
                Ok(()) => None,
                Err(e) => {
                    self.error(format!("inflated-map refresh failed: {e}"));
                    Some(e)
                }
            };
            let release_confirmed = self.guard.release().await;
            if !release_confirmed {
                return reject("maps may have refreshed, but mutation-lock release is unconfirmed; navigation remains blocked. Inspect the robot, then restart map_manage and nav_action_server");
            }
            if let Some(e) = refresh_error {
                return reject(&format!("inflated-map refresh failed: {e}"));
            }
        }
        // rclpy assigns the values once every callback accepted them
        for p in params {
            match p.name.as_str() {
                "inflate_radius_m" => self.inflate_radius_m = number_of(&p.value).unwrap_or(self.inflate_radius_m),
                "chennal_width_m" => self.chennal_width_m = number_of(&p.value).unwrap_or(self.chennal_width_m),
                "use_sim_time" => self.use_sim_time = p.value.bool_value,
                "start_type_description_service" => self.start_type_description_service = p.value.bool_value,
                _ => {}
            }
        }
        ok
    }
}

fn declared_type(name: &str) -> Option<u8> {
    match name {
        "inflate_radius_m" | "chennal_width_m" => Some(PARAMETER_DOUBLE),
        "use_sim_time" | "start_type_description_service" => Some(PARAMETER_BOOL),
        _ => None,
    }
}

/// `rclpy.Parameter.Type` names for the rclpy-style type error.
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

/// `float(param.value)` for the numeric parameter types.
fn number_of(v: &ParameterValue) -> Option<f64> {
    match v.type_ {
        PARAMETER_DOUBLE => Some(v.double_value),
        2 => Some(v.integer_value as f64),
        PARAMETER_BOOL => Some(v.bool_value as i64 as f64),
        _ => None,
    }
}

async fn service_available<T: r2r::WrappedServiceTypeSupport + 'static>(client: &r2r::Client<T>, timeout_s: f64) -> bool {
    match r2r::Node::is_available(client) {
        Ok(f) => tokio::time::timeout(Duration::from_secs_f64(timeout_s), f).await.is_ok(),
        Err(_) => false,
    }
}

async fn call_with_timeout<R>(fut: r2r::Result<impl std::future::Future<Output = r2r::Result<R>>>, timeout_s: f64) -> Option<R> {
    let fut = fut.ok()?;
    tokio::time::timeout(Duration::from_secs_f64(timeout_s), fut).await.ok()?.ok()
}

#[derive(Clone, Copy)]
enum Op {
    RiskMap,
    FreeSpace,
    ChennalMap,
    Restore,
}

impl Op {
    fn operation(self) -> &'static str {
        match self {
            Op::RiskMap => "rebuild the risk map",
            Op::FreeSpace => "rebuild free space",
            Op::ChennalMap => "rebuild the channel map",
            Op::Restore => "restore free-space coverage",
        }
    }
}

/// `guarded_mission_mutation(operation)`: run a Trigger service body inside
/// the central mutation lease; a lost release turns the answer into a failure.
async fn run_guarded(app: &Shared, op: Op) -> (bool, String) {
    let mut a = app.lock().await;
    if let Err(message) = a.guard.acquire(op.operation()).await {
        return (false, message);
    }
    let (mut success, mut message) = match op {
        Op::RiskMap => {
            a.info("(service)create_risk_map_srv call");
            match a.create_risk_map_sync().await {
                Ok(r) => r,
                Err(e) => {
                    a.error(format!("創建風險地圖時發生錯誤: {e}"));
                    (false, format!("創建風險地圖時發生錯誤: {e}"))
                }
            }
        }
        Op::FreeSpace => {
            a.info("(service)create_free_space_srv call");
            match a.create_free_space_sync().await {
                Ok(r) => r,
                Err(e) => {
                    a.error(format!("創建自由空間時發生錯誤: {e}"));
                    (false, format!("創建自由空間時發生錯誤: {e}"))
                }
            }
        }
        Op::ChennalMap => {
            a.info("(service)create_chennal_map_srv call");
            match a.create_chennal_map_sync().await {
                Ok(r) => r,
                Err(e) => {
                    a.error(format!("創建通道地圖時發生錯誤: {e}"));
                    (false, format!("創建通道地圖時發生錯誤: {e}"))
                }
            }
        }
        Op::Restore => a.restore_free_space(),
    };
    if !a.guard.release().await {
        let previous = message.trim().to_string();
        success = false;
        message = if previous.is_empty() { RELEASE_UNCONFIRMED.to_string() } else { format!("{previous}; {RELEASE_UNCONFIRMED}") };
        a.error(&message);
    }
    (success, message)
}

fn uuid_hex() -> String {
    use std::io::Read;
    let mut bytes = [0u8; 16];
    if let Ok(mut f) = std::fs::File::open("/dev/urandom") {
        let _ = f.read_exact(&mut bytes);
    }
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "map_manage", "")?;
    let logger = node.logger().to_string();
    r2r::log_info!(&logger, "map_manage init");

    let inflate_radius_m = params::f64(&node, "inflate_radius_m", MIN_SAFE_INFLATE_RADIUS_M);
    if !inflate_radius_m.is_finite() || inflate_radius_m < MIN_SAFE_INFLATE_RADIUS_M {
        return Err(format!("inflate_radius_m startup value must be finite and >= {MIN_SAFE_INFLATE_RADIUS_M:.2} m").into());
    }
    let chennal_width_m = params::f64(&node, "chennal_width_m", 1.2);
    let use_sim_time = params::bool(&node, "use_sim_time", false);

    let pubs = Pubs {
        free_space: node.create_publisher::<OccupancyGrid>("/free_space", latched())?,
        free_space_inflated: node.create_publisher::<OccupancyGrid>("/free_space_inflated", latched())?,
        risk_map: node.create_publisher::<OccupancyGrid>("/risk_map", latched())?,
        risk_map_inflated: node.create_publisher::<OccupancyGrid>("/risk_map_inflated", latched())?,
        chennal_map: node.create_publisher::<OccupancyGrid>("/chennal_map", latched())?,
        chennal_map_inflated: node.create_publisher::<OccupancyGrid>("/chennal_map_inflated", latched())?,
        nav_base: node.create_publisher::<OccupancyGrid>("/map_grid", latched())?,
        nav_global: node.create_publisher::<OccupancyGrid>("/map_grid_global", latched())?,
    };
    let lock_client = node.create_client::<MissionOperationLock::Service>("/mission_operation_lock", QosProfile::services_default())?;
    let owner = format!("{}:{}", node.fully_qualified_name()?, uuid_hex());
    let guard = Guard::new(owner, lock_client, logger.clone());
    let map_msg = build_demo_map();
    let app: Shared = Arc::new(Mutex::new(App {
        logger: logger.clone(),
        guard,
        inflate_radius_m,
        chennal_width_m,
        use_sim_time,
        start_type_description_service: params::bool(&node, "start_type_description_service", true),
        zone_map_list: Vec::new(),
        base_map: None,
        collected_free_space: None,
        risk_map: None,
        risk_map_inflated: None,
        chennal_map: None,
        free_zone_backup: None,
        free_risk_backup: None,
        global_map_msg: map_msg.clone(),
        map_msg,
        pubs,
        risk_zone_list_client: node.create_client::<GetZoneList::Service>("/get_risk_zone_list", QosProfile::services_default())?,
        record_zone_list_client: node.create_client::<GetZoneList::Service>("/get_record_zone_list", QosProfile::services_default())?,
        chennal_path_list_client: node.create_client::<ChennalPathList::Service>("/get_chennal_path_list", QosProfile::services_default())?,
    }));

    // ---- navigation activity flag (fail-closed until seen) ----------------
    {
        let mut stream = node.subscribe::<Bool>("/nav_operation_active", latched())?;
        let app = app.clone();
        tokio::spawn(async move {
            while let Some(msg) = stream.next().await {
                let mut a = app.lock().await;
                a.guard.nav_active = msg.data;
                a.guard.nav_seen = true;
            }
        });
    }

    // ---- services ---------------------------------------------------------
    for (name, op) in [
        ("/create_risk_map", Op::RiskMap),
        ("/create_free_space", Op::FreeSpace),
        ("/create_chennal_map", Op::ChennalMap),
        ("/restore_free_space_coverage", Op::Restore),
    ] {
        let mut stream = node.create_service::<Trigger::Service>(name, QosProfile::services_default())?;
        let app = app.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let (success, message) = run_guarded(&app, op).await;
                let _ = req.respond(Trigger::Response { success, message });
            }
        });
    }
    {
        let mut stream = node.create_service::<ImportImageMask::Service>("/import_image_mask", QosProfile::services_default())?;
        let app = app.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let mut a = app.lock().await;
                let response = match a.guard.acquire("import an image mission").await {
                    Err(message) => ImportImageMask::Response { success: false, message, zone_id: 0, area_m2: 0.0 },
                    Ok(()) => {
                        let mut res = a.import_image_mask(&req.message);
                        if !a.guard.release().await {
                            let previous = res.message.trim().to_string();
                            res.success = false;
                            res.message = if previous.is_empty() { RELEASE_UNCONFIRMED.to_string() } else { format!("{previous}; {RELEASE_UNCONFIRMED}") };
                            a.error(&res.message);
                        }
                        res
                    }
                };
                drop(a);
                let _ = req.respond(response);
            }
        });
    }
    {
        let mut stream = node.create_service::<ZoneMapList::Service>("/get_zone_map_list_srv", QosProfile::services_default())?;
        let app = app.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let zone_map_list = app.lock().await.zone_map_list.iter().map(|z| z.msg.clone()).collect();
                let _ = req.respond(ZoneMapList::Response { zone_map_list });
            }
        });
    }

    // ---- parameter services (/map_manage/*) --------------------------------
    const PARAM_NAMES: [&str; 4] = ["chennal_width_m", "inflate_radius_m", "start_type_description_service", "use_sim_time"];
    {
        let mut stream = node.create_service::<GetParameters::Service>("/map_manage/get_parameters", QosProfile::services_default())?;
        let app = app.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let a = app.lock().await;
                // rclpy raises ParameterNotDeclaredException for an unknown
                // name and the service answers with no values at all
                let values = if req.message.names.iter().all(|n| declared_type(n).is_some()) {
                    req.message.names.iter().map(|n| a.parameter_value(n)).collect()
                } else {
                    Vec::new()
                };
                drop(a);
                let _ = req.respond(GetParameters::Response { values });
            }
        });
    }
    {
        let mut stream = node.create_service::<GetParameterTypes::Service>("/map_manage/get_parameter_types", QosProfile::services_default())?;
        let app = app.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let a = app.lock().await;
                let types = req.message.names.iter().map(|n| a.parameter_value(n).type_).collect();
                drop(a);
                let _ = req.respond(GetParameterTypes::Response { types });
            }
        });
    }
    {
        let mut stream = node.create_service::<ListParameters::Service>("/map_manage/list_parameters", QosProfile::services_default())?;
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let prefixes = &req.message.prefixes;
                let names = PARAM_NAMES.iter().filter(|n| prefixes.is_empty() || prefixes.iter().any(|p| n.starts_with(p.as_str()))).map(|n| n.to_string()).collect();
                let _ = req.respond(ListParameters::Response { result: ListParametersResult { names, prefixes: Vec::new() } });
            }
        });
    }
    {
        let mut stream = node.create_service::<DescribeParameters::Service>("/map_manage/describe_parameters", QosProfile::services_default())?;
        let app = app.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let a = app.lock().await;
                let descriptors = req
                    .message
                    .names
                    .iter()
                    .map(|n| ParameterDescriptor { name: n.clone(), type_: a.parameter_value(n).type_, ..Default::default() })
                    .collect();
                drop(a);
                let _ = req.respond(DescribeParameters::Response { descriptors });
            }
        });
    }
    {
        let mut stream = node.create_service::<SetParameters::Service>("/map_manage/set_parameters", QosProfile::services_default())?;
        let app = app.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let mut results = Vec::with_capacity(req.message.parameters.len());
                for p in &req.message.parameters {
                    let mut a = app.lock().await;
                    results.push(a.set_parameters(std::slice::from_ref(p)).await);
                }
                let _ = req.respond(SetParameters::Response { results });
            }
        });
    }
    {
        let mut stream = node.create_service::<SetParametersAtomically::Service>("/map_manage/set_parameters_atomically", QosProfile::services_default())?;
        let app = app.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let result = app.lock().await.set_parameters(&req.message.parameters).await;
                let _ = req.respond(SetParametersAtomically::Response { result });
            }
        });
    }

    app.lock().await.publish_current_navigation_maps();
    r2r::log_info!(&logger, "Publishing OccupancyGrid on /map_grid and /map_grid_global (latched, on change)");

    // ---- spin until SIGINT / SIGTERM ----------------------------------------
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
