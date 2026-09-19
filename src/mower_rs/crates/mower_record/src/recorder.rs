//! The recorder's state and every service body (port of `PathRecorder` in
//! `path_record_node.py`), free of the ROS transport: publishing goes
//! through the [`Outputs`] trait so the logic is testable without a graph.
//! Messages to the app are kept verbatim (the UI shows them).

use std::path::Path;
use std::time::{Duration, Instant};

use r2r::builtin_interfaces::msg::Time;
use r2r::geometry_msgs::msg::{Point, PoseStamped};
use r2r::nav_msgs::msg::Path as PathMsg;
use r2r::std_msgs::msg::ColorRGBA;
use r2r::visualization_msgs::msg::{Marker, MarkerArray};
use serde_json::{json, Value};

use crate::geometry::{self, Xy};
use crate::site_store::{self, Datum, XyObject};

pub const LINE_STRIP: i32 = 4;
pub const ADD: i32 = 0;
pub const DELETEALL: i32 = 3;

pub const ZONE_STYLE: ((f64, f64, f64, f64), f64) = ((0.6, 0.0, 1.0, 0.5), 0.02);
pub const RISK_STYLE: ((f64, f64, f64, f64), f64) = ((1.0, 0.0, 0.0, 0.8), 0.03);
pub const CHANNEL_EDIT_STYLE: ((f64, f64, f64, f64), f64) = ((0.0, 0.7, 1.0, 0.8), 0.03);

/// Where the recorder publishes; the node implements it with r2r publishers.
pub trait Outputs {
    fn recorded_path(&self, path: &PathMsg);
    fn zone_marker(&self, m: &Marker);
    fn zone_list(&self, m: &MarkerArray);
    fn risk_marker(&self, m: &Marker);
    fn risk_list(&self, m: &MarkerArray);
    fn channel_path(&self, p: &PathMsg);
    fn channel_list(&self, m: &MarkerArray);
    fn site_list(&self, json: &str);
    fn info(&self, msg: &str);
    fn warn(&self, msg: &str);
    fn error(&self, msg: &str);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Zone,
    Risk,
    Channel,
}

impl Kind {
    pub fn label(self) -> &'static str {
        match self {
            Kind::Zone => "zone",
            Kind::Risk => "risk",
            Kind::Channel => "channel",
        }
    }
}

pub struct Config {
    pub frame_id: String,
    pub save_dir: String,
    pub sites_dir: String,
    pub min_dist: f64,
    pub min_dt: f64,
    pub polygon_simplify_dist: f64,
    pub max_site_datum_distance_m: f64,
    pub robot_pose_max_age_s: f64,
}

/// Service reply: (success, message).
pub type Reply = (bool, String);

pub struct Recorder {
    pub cfg: Config,
    now: Box<dyn Fn() -> Time + Send>,
    pub datum: Option<Datum>,
    pub active_site: Option<String>,
    pub manifest_blocked: Option<String>,
    pub working_state_ready: bool,
    pub initialized: bool,
    pose_source: Option<PoseStamped>,
    pose_unavailable_logged: bool,
    last_robot_pos: Option<PoseStamped>,
    // zone recording
    pub recording_zone: bool,
    pub zone_id: i32,
    zone_marker: Option<Marker>,
    pub zone_list: MarkerArray,
    path: PathMsg,
    // risk recording
    pub recording_risk: bool,
    pub risk_id: i32,
    risk_marker: Option<Marker>,
    pub risk_list: MarkerArray,
    risk_path: PathMsg,
    // channel recording
    pub recording_channel: bool,
    pub channel_id: i32,
    pub channel_list: MarkerArray,
    channel_path: PathMsg,
}

fn stamp_to_ns(t: &Time) -> i64 {
    t.sec as i64 * 1_000_000_000 + t.nanosec as i64
}

fn xy_of(m: &Marker) -> Vec<Xy> {
    m.points.iter().map(|p| (p.x, p.y)).collect()
}

fn empty_path(frame: &str) -> PathMsg {
    let mut p = PathMsg::default();
    p.header.frame_id = frame.to_string();
    p
}

impl Recorder {
    pub fn new(cfg: Config, now: Box<dyn Fn() -> Time + Send>) -> Self {
        let frame = cfg.frame_id.clone();
        Recorder {
            cfg,
            now,
            datum: None,
            active_site: None,
            manifest_blocked: None,
            working_state_ready: false,
            initialized: false,
            pose_source: None,
            pose_unavailable_logged: false,
            last_robot_pos: None,
            recording_zone: false,
            zone_id: 0,
            zone_marker: None,
            zone_list: MarkerArray::default(),
            path: empty_path(&frame),
            recording_risk: false,
            risk_id: 0,
            risk_marker: None,
            risk_list: MarkerArray::default(),
            risk_path: empty_path(&frame),
            recording_channel: false,
            channel_id: 0,
            channel_list: MarkerArray::default(),
            channel_path: empty_path(&frame),
        }
    }

    fn now(&self) -> Time {
        (self.now)()
    }

    pub fn recording(&self) -> bool {
        self.recording_zone || self.recording_risk || self.recording_channel
    }

    fn active_kind(&self) -> Option<Kind> {
        if self.recording_zone {
            Some(Kind::Zone)
        } else if self.recording_risk {
            Some(Kind::Risk)
        } else if self.recording_channel {
            Some(Kind::Channel)
        } else {
            None
        }
    }

    // ── robot pose ─────────────────────────────────────────────────────────

    pub fn on_pose_source(&mut self, frame_id: &str, child_frame_id: &str, pose: PoseStamped) {
        if frame_id != self.cfg.frame_id || child_frame_id != "base_footprint" {
            return;
        }
        let p = &pose.pose.position;
        let q = &pose.pose.orientation;
        let values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w];
        if !values.iter().all(|v| v.is_finite()) {
            return;
        }
        if ((q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt() - 1.0).abs() > 1e-2 {
            return;
        }
        self.pose_source = Some(pose);
    }

    /// 回傳最新的 map 座標系機器人位置（過期或尚未收到時回傳 None）.
    pub fn robot_pos(&mut self, out: &dyn Outputs) -> Option<PoseStamped> {
        if let Some(pose) = &self.pose_source {
            let age_s = (stamp_to_ns(&self.now()) - stamp_to_ns(&pose.header.stamp)) as f64 * 1e-9;
            if (-0.5..=self.cfg.robot_pose_max_age_s.max(0.1)).contains(&age_s) {
                self.pose_unavailable_logged = false;
                return Some(pose.clone());
            }
        }
        if !self.pose_unavailable_logged {
            self.pose_unavailable_logged = true;
            out.warn("機器人位置不可用: 尚未收到或已過期");
        }
        None
    }

    /// 0.5 s until the first pose: clear stale RViz markers, publish lists.
    /// Returns true once initialised (the caller stops the timer).
    pub fn try_initialize(&mut self, out: &dyn Outputs) -> bool {
        if self.initialized {
            return true;
        }
        let Some(pos) = self.robot_pos(out) else {
            out.warn("等待機器人位置（map 座標）以完成初始化...");
            return false;
        };
        self.last_robot_pos = Some(pos.clone());
        self.initialized = true;
        for (ns, publish_zone) in [("zones", true), ("risk_zones", false)] {
            let mut del = Marker::default();
            del.header.frame_id = self.cfg.frame_id.clone();
            del.header.stamp = self.now();
            del.ns = ns.to_string();
            del.action = DELETEALL;
            if publish_zone {
                out.zone_marker(&del);
            } else {
                out.risk_marker(&del);
            }
        }
        out.info("✅ 已清空 RViz zone_markers 與 risk_zone_markers");
        // 若 TF 可用前已有 load（場地或 zone_record），不可清掉已載入的清單。
        if self.zone_list.markers.is_empty() && self.risk_list.markers.is_empty() && self.channel_list.markers.is_empty() {
            self.zone_list = MarkerArray::default();
            self.risk_list = MarkerArray::default();
        }
        out.zone_list(&self.zone_list);
        out.risk_list(&self.risk_list);
        out.info(&format!("機器人位置初始化成功: x={:.3}, y={:.3}", pos.pose.position.x, pos.pose.position.y));
        self.path.header.frame_id = self.cfg.frame_id.clone();
        self.path.header.stamp = self.now();
        out.recorded_path(&self.path);
        true
    }

    // ── markers ────────────────────────────────────────────────────────────

    fn marker_from_xy(&self, ns: &str, id: i32, color: (f64, f64, f64, f64), scale: f64, pts: &[Xy], closed: bool) -> Marker {
        let mut m = Marker::default();
        m.header.frame_id = self.cfg.frame_id.clone();
        m.header.stamp = self.now();
        m.ns = ns.to_string();
        m.id = id;
        m.type_ = LINE_STRIP;
        m.action = ADD;
        m.scale.x = scale;
        m.color = ColorRGBA { r: color.0 as f32, g: color.1 as f32, b: color.2 as f32, a: color.3 as f32 };
        m.points = pts.iter().map(|(x, y)| Point { x: *x, y: *y, z: 0.0 }).collect();
        if closed && pts.len() >= 3 {
            let (fx, fy) = pts[0];
            let (lx, ly) = pts[pts.len() - 1];
            if ((fx - lx).powi(2) + (fy - ly).powi(2)).sqrt() > 1e-6 {
                m.points.push(Point { x: fx, y: fy, z: 0.0 });
            }
        }
        m
    }

    /// 根據路徑創建多邊形 (zones / risk_zones): simplify, close if the ends
    /// are within a metre.
    fn polygon_from_path(&self, poses: &[PoseStamped], ns: &str, id: i32, style: ((f64, f64, f64, f64), f64)) -> Option<Marker> {
        if poses.len() < 3 {
            return None;
        }
        let pts: Vec<Xy> = poses.iter().map(|p| (p.pose.position.x, p.pose.position.y)).collect();
        let simplified = geometry::simplify(&pts, self.cfg.polygon_simplify_dist);
        let mut m = Marker::default();
        m.header.frame_id = self.cfg.frame_id.clone();
        m.header.stamp = self.now();
        m.ns = ns.to_string();
        m.id = id;
        m.type_ = LINE_STRIP;
        m.action = ADD;
        m.scale.x = style.1;
        m.color = ColorRGBA { r: style.0 .0 as f32, g: style.0 .1 as f32, b: style.0 .2 as f32, a: style.0 .3 as f32 };
        m.points = simplified.iter().map(|(x, y)| Point { x: *x, y: *y, z: 0.0 }).collect();
        if simplified.len() > 2 {
            let (fx, fy) = simplified[0];
            let (lx, ly) = simplified[simplified.len() - 1];
            if ((fx - lx).powi(2) + (fy - ly).powi(2)).sqrt() < 1.0 {
                m.points.push(Point { x: fx, y: fy, z: 0.0 });
            }
        }
        Some(m)
    }

    fn path_to_marker(&self, path: &PathMsg, ns: &str, id: i32, color: (f64, f64, f64), scale: f64) -> Marker {
        let mut m = Marker::default();
        m.header = path.header.clone();
        m.ns = ns.to_string();
        m.id = id;
        m.type_ = LINE_STRIP;
        m.action = ADD;
        m.scale.x = scale;
        m.color = ColorRGBA { r: color.0 as f32, g: color.1 as f32, b: color.2 as f32, a: 0.8 };
        m.points = path.poses.iter().map(|p| Point { x: p.pose.position.x, y: p.pose.position.y, z: p.pose.position.z }).collect();
        m
    }

    // ── 10 Hz sampler ──────────────────────────────────────────────────────

    /// One sampling tick; returns false when nothing is being recorded (the
    /// caller then stops the timer).
    pub fn sample(&mut self, out: &dyn Outputs) -> bool {
        if !self.recording() {
            return false;
        }
        if !self.initialized {
            return true;
        }
        let Some(pos) = self.robot_pos(out) else { return true };
        let now = self.now();
        if self.recording_zone {
            let Some(last) = self.last_robot_pos.clone() else { return true };
            if self.moved_enough(&pos, &last, &now) {
                self.path.poses.push(pos.clone());
                self.last_robot_pos = Some(pos.clone());
                self.path.header.stamp = self.now();
                out.recorded_path(&self.path);
                if self.path.poses.len() >= 3 {
                    if let Some(m) = self.polygon_from_path(&self.path.poses, "zones", self.zone_id, ZONE_STYLE) {
                        out.zone_marker(&m);
                        self.zone_marker = Some(m);
                    }
                }
            }
        }
        if self.recording_risk {
            let Some(last) = self.last_robot_pos.clone() else {
                self.last_robot_pos = Some(pos);
                return true;
            };
            if self.moved_enough(&pos, &last, &now) {
                self.risk_path.poses.push(pos.clone());
                self.last_robot_pos = Some(pos.clone());
                self.risk_path.header.stamp = self.now();
                if self.risk_path.poses.len() >= 3 {
                    if let Some(m) = self.polygon_from_path(&self.risk_path.poses, "risk_zones", self.risk_id, RISK_STYLE) {
                        out.risk_marker(&m);
                        self.risk_marker = Some(m);
                    }
                }
            }
        }
        if self.recording_channel {
            let Some(last) = self.last_robot_pos.clone() else {
                self.last_robot_pos = Some(pos);
                return true;
            };
            if self.moved_enough(&pos, &last, &now) {
                self.channel_path.poses.push(pos.clone());
                self.last_robot_pos = Some(pos);
                self.channel_path.header.stamp = self.now();
                out.channel_path(&self.channel_path);
            }
        }
        true
    }

    fn moved_enough(&self, pos: &PoseStamped, last: &PoseStamped, now: &Time) -> bool {
        let dt = (stamp_to_ns(now) - stamp_to_ns(&last.header.stamp)) as f64 * 1e-9;
        let dx = pos.pose.position.x - last.pose.position.x;
        let dy = pos.pose.position.y - last.pose.position.y;
        (dx * dx + dy * dy).sqrt() >= self.cfg.min_dist && dt >= self.cfg.min_dt
    }

    // ── recording services ─────────────────────────────────────────────────

    /// Reject a start while another recording runs; `Ok(())` means go ahead
    /// (the caller has to hold the mutation lease and clear the site link).
    pub fn check_start(&self, requested: Kind) -> Result<(), String> {
        match self.active_kind() {
            None => Ok(()),
            Some(active) => Err(format!("無法開始 {} 記錄：目前正在進行 {} 記錄", requested.label(), active.label())),
        }
    }

    pub fn blocked_manifest_reply(&self, operation: &str) -> Option<String> {
        self.manifest_blocked
            .as_ref()
            .map(|why| format!("無法{operation}：持久化場地狀態未就緒（{why}），請先使用 /site_op load 重新載入場地"))
    }

    pub fn start_zone(&mut self, out: &dyn Outputs) -> Reply {
        out.info("記錄區域起始點");
        self.recording_zone = true;
        self.zone_id += 1;
        self.path = empty_path(&self.cfg.frame_id);
        if let Some(pos) = self.robot_pos(out) {
            self.last_robot_pos = Some(pos);
        }
        self.zone_marker = None;
        out.zone_marker(&Marker::default());
        (true, "成功記錄區域起始點".into())
    }

    /// Returns (reply, needs_release): the caller releases the mutation lease
    /// and appends the lock-release warning when release is unconfirmed.
    pub fn end_zone(&mut self, out: &dyn Outputs) -> Result<String, String> {
        if !self.recording_zone {
            return Err("沒有正在進行的 zone 記錄".into());
        }
        let Some(marker) = self.zone_marker.clone().filter(|m| m.points.len() >= 3) else {
            return Err("區域點數不足，至少需要 3 個點".into());
        };
        out.info("記錄區域結束點");
        self.zone_list.markers.push(marker);
        if !self.save_zone_list(out) {
            self.zone_list.markers.pop();
            let msg = "區域已收尾但持久化失敗；錄製仍保留，請檢查磁碟後重試結束".to_string();
            out.error(&msg);
            return Err(msg);
        }
        self.recording_zone = false;
        self.zone_marker = None;
        self.path = empty_path(&self.cfg.frame_id);
        self.last_robot_pos = None;
        self.working_state_ready = true;
        out.zone_list(&self.zone_list);
        Ok(format!("成功記錄區域結束點 #{}", self.zone_list.markers.len()))
    }

    pub fn start_risk(&mut self, out: &dyn Outputs) -> Reply {
        out.info("開始記錄風險區域");
        self.recording_risk = true;
        self.risk_id += 1;
        self.risk_path = empty_path(&self.cfg.frame_id);
        if let Some(pos) = self.robot_pos(out) {
            self.last_robot_pos = Some(pos);
        }
        self.risk_marker = None;
        out.risk_marker(&Marker::default());
        (true, "成功開始記錄風險區域".into())
    }

    pub fn end_risk(&mut self, out: &dyn Outputs) -> Result<String, String> {
        if !self.recording_risk {
            return Err("沒有正在進行的 risk 記錄".into());
        }
        let Some(marker) = self.risk_marker.clone().filter(|m| m.points.len() >= 3) else {
            return Err("風險區域點數不足，至少需要 3 個點".into());
        };
        out.info("結束記錄風險區域");
        self.risk_list.markers.push(marker);
        if !self.save_risk_list(out) {
            self.risk_list.markers.pop();
            let msg = "風險區域已收尾但持久化失敗；錄製仍保留，請檢查磁碟後重試結束".to_string();
            out.error(&msg);
            return Err(msg);
        }
        self.recording_risk = false;
        out.risk_list(&self.risk_list);
        self.risk_marker = None;
        self.risk_path = empty_path(&self.cfg.frame_id);
        self.last_robot_pos = None;
        self.working_state_ready = true;
        Ok(format!("成功結束記錄風險區域 #{}", self.risk_list.markers.len()))
    }

    pub fn start_channel(&mut self, out: &dyn Outputs) -> Reply {
        out.info("開始記錄 chennal 路徑");
        self.recording_channel = true;
        self.channel_id += 1;
        self.channel_path = empty_path(&self.cfg.frame_id);
        self.channel_path.header.stamp = self.now();
        if let Some(pos) = self.robot_pos(out) {
            self.last_robot_pos = Some(pos.clone());
            self.channel_path.poses.push(pos);
            out.channel_path(&self.channel_path);
        }
        (true, format!("成功開始記錄 chennal 路徑 #{}", self.channel_id))
    }

    pub fn end_channel(&mut self, out: &dyn Outputs) -> Result<String, String> {
        out.info("結束記錄 chennal 路徑");
        if !self.recording_channel {
            return Err("沒有正在進行的 chennal 路徑記錄".into());
        }
        if let Some(pos) = self.robot_pos(out) {
            if let Some(last) = self.channel_path.poses.last() {
                let dx = pos.pose.position.x - last.pose.position.x;
                let dy = pos.pose.position.y - last.pose.position.y;
                if (dx * dx + dy * dy).sqrt() >= self.cfg.min_dist {
                    self.channel_path.poses.push(pos);
                }
            }
        }
        if self.channel_path.poses.len() < 2 {
            return Err("chennal 路徑點數不足，至少需要 2 個點".into());
        }
        let completed = self.path_to_marker(&self.channel_path, "chennal_path", self.channel_id, (0.0, 1.0, 0.0), 0.1);
        self.channel_list.markers.push(completed);
        if !self.save_channel_list(out) {
            self.channel_list.markers.pop();
            let msg = "channel 路徑已收尾但持久化失敗；錄製仍保留，請檢查磁碟後重試結束".to_string();
            out.error(&msg);
            return Err(msg);
        }
        self.recording_channel = false;
        out.channel_list(&self.channel_list);
        self.channel_path = empty_path(&self.cfg.frame_id);
        self.last_robot_pos = None;
        self.working_state_ready = true;
        Ok("成功結束記錄 chennal 路徑".into())
    }

    /// 取消目前進行中的記錄：停止取樣、清空 in-progress 路徑，不加入清單。
    /// Returns the cancelled kind (the caller releases the lease then).
    pub fn cancel(&mut self, out: &dyn Outputs) -> Option<Kind> {
        let mut cancelled = None;
        if self.recording_zone {
            self.recording_zone = false;
            self.path = empty_path(&self.cfg.frame_id);
            self.zone_marker = None;
            out.zone_marker(&Marker::default());
            cancelled = Some(Kind::Zone);
        }
        if self.recording_risk {
            self.recording_risk = false;
            self.risk_path = empty_path(&self.cfg.frame_id);
            self.risk_marker = None;
            out.risk_marker(&Marker::default());
            cancelled = Some(Kind::Risk);
        }
        if self.recording_channel {
            self.recording_channel = false;
            self.channel_path = empty_path(&self.cfg.frame_id);
            out.channel_path(&self.channel_path);
            cancelled = Some(Kind::Channel);
        }
        self.last_robot_pos = None;
        cancelled
    }

    // ── edit_zone ──────────────────────────────────────────────────────────

    /// App 直接編輯物件：新增 / 刪除 / 更新。Returns (message, id).
    pub fn edit(&mut self, out: &dyn Outputs, op: &str, kind: &str, id: i32, pts: &[Xy]) -> Result<(String, i32), String> {
        if let Some(msg) = self.blocked_manifest_reply("編輯任務幾何") {
            out.error(&msg);
            return Err(msg);
        }
        let kind = match kind {
            "zone" => Kind::Zone,
            "risk" => Kind::Risk,
            "channel" => Kind::Channel,
            other => return Err(format!("未知 kind: {other}（要 zone/risk/channel）")),
        };
        let (ns, style, closed) = match kind {
            Kind::Zone => ("zones", ZONE_STYLE, true),
            Kind::Risk => ("risk_zones", RISK_STYLE, true),
            Kind::Channel => ("channels", CHANNEL_EDIT_STYLE, false),
        };
        let min_pts = if closed { 3 } else { 2 };
        let original = self.list_of(kind).clone();
        let (message, result_id) = match op {
            "delete" => {
                let list = self.list_of_mut(kind);
                let before = list.markers.len();
                list.markers.retain(|m| m.id != id);
                if list.markers.len() == before {
                    return Err(format!("找不到 {} id={id}", kind.label()));
                }
                (format!("已刪除 {} id={id}", kind.label()), id)
            }
            "add" => {
                if pts.len() < min_pts {
                    return Err(format!("{} 頂點不足（{} < {min_pts}）", kind.label(), pts.len()));
                }
                let new_id = self.list_of(kind).markers.iter().map(|m| m.id).max().unwrap_or(0) + 1;
                let marker = self.marker_from_xy(ns, new_id, style.0, style.1, pts, closed);
                self.list_of_mut(kind).markers.push(marker);
                (format!("已新增 {} id={new_id}", kind.label()), new_id)
            }
            "update" => {
                if pts.len() < min_pts {
                    return Err(format!("{} 頂點不足（{} < {min_pts}）", kind.label(), pts.len()));
                }
                let rebuilt = self.marker_from_xy(ns, id, style.0, style.1, pts, closed);
                let stamp = self.now();
                let Some(target) = self.list_of_mut(kind).markers.iter_mut().find(|m| m.id == id) else {
                    return Err(format!("找不到 {} id={id}", kind.label()));
                };
                target.points = rebuilt.points;
                target.header.stamp = stamp;
                (format!("已更新 {} id={id}", kind.label()), id)
            }
            other => return Err(format!("未知 op: {other}（要 add/delete/update）")),
        };
        let persisted = self.save_list(kind, out);
        if !persisted {
            *self.list_of_mut(kind) = original;
            let msg = format!("{message}；持久化失敗，變更已回復");
            out.error(&msg);
            return Err(msg);
        }
        if !self.update_active_site(out) {
            *self.list_of_mut(kind) = original;
            let rollback_persisted = self.save_list(kind, out);
            let mut msg = format!("{message}；場地檔同步失敗，變更已回復");
            if !rollback_persisted {
                self.manifest_blocked = Some("編輯回復後的工作檔狀態無法確認".into());
                msg.push_str("（工作檔回復也失敗，請立即檢查磁碟）");
            }
            out.error(&msg);
            return Err(msg);
        }
        // App-created markers choose their id from the current list rather
        // than from the physical recorder's counters; keep them aligned.
        self.restore_id_counters();
        match kind {
            Kind::Zone => out.zone_list(&self.zone_list),
            Kind::Risk => out.risk_list(&self.risk_list),
            Kind::Channel => out.channel_list(&self.channel_list),
        }
        out.info(&message);
        Ok((message, result_id))
    }

    fn list_of(&self, kind: Kind) -> &MarkerArray {
        match kind {
            Kind::Zone => &self.zone_list,
            Kind::Risk => &self.risk_list,
            Kind::Channel => &self.channel_list,
        }
    }

    fn list_of_mut(&mut self, kind: Kind) -> &mut MarkerArray {
        match kind {
            Kind::Zone => &mut self.zone_list,
            Kind::Risk => &mut self.risk_list,
            Kind::Channel => &mut self.channel_list,
        }
    }

    fn save_list(&self, kind: Kind, out: &dyn Outputs) -> bool {
        match kind {
            Kind::Zone => self.save_zone_list(out),
            Kind::Risk => self.save_risk_list(out),
            Kind::Channel => self.save_channel_list(out),
        }
    }

    // ── work files (zone_record/*.json) ───────────────────────────────────

    fn work_path(&self, name: &str) -> std::path::PathBuf {
        Path::new(&self.cfg.save_dir).join(name)
    }

    fn markers_json(list: &MarkerArray, with_style: bool) -> Value {
        Value::Array(
            list.markers
                .iter()
                .map(|m| {
                    let mut o = json!({
                        "id": m.id,
                        "ns": m.ns,
                        "points": m.points.iter().map(|p| json!([p.x, p.y, p.z])).collect::<Vec<_>>(),
                    });
                    if with_style {
                        o["color"] = json!({"r": m.color.r, "g": m.color.g, "b": m.color.b, "a": m.color.a});
                        o["scale"] = json!(m.scale.x);
                    }
                    o
                })
                .collect(),
        )
    }

    fn save_json(&self, name: &str, payload: &Value, what: &str, out: &dyn Outputs) -> bool {
        match site_store::write_json_atomic(&self.work_path(name), payload) {
            Ok(()) => true,
            Err(e) => {
                out.error(&format!("{what}失敗: {e}"));
                false
            }
        }
    }

    pub fn save_zone_list(&self, out: &dyn Outputs) -> bool {
        self.save_json("zone_list.json", &Self::markers_json(&self.zone_list, false), "儲存區域列表", out)
    }

    pub fn save_risk_list(&self, out: &dyn Outputs) -> bool {
        self.save_json("risk_zone_list.json", &Self::markers_json(&self.risk_list, false), "儲存風險區域列表", out)
    }

    pub fn save_channel_list(&self, out: &dyn Outputs) -> bool {
        self.save_json("chennal_path_list.json", &Self::markers_json(&self.channel_list, true), "保存 chennal 路径列表", out)
    }

    pub fn save_all_working_state(&self, out: &dyn Outputs) -> bool {
        let r = (self.save_zone_list(out), self.save_risk_list(out), self.save_channel_list(out));
        r.0 && r.1 && r.2
    }

    fn markers_from_json(&self, doc: &Value, default_style: ((f64, f64, f64, f64), f64), with_style: bool) -> Result<MarkerArray, String> {
        let arr = doc.as_array().ok_or("list file is not an array")?;
        let mut list = MarkerArray::default();
        for md in arr {
            let mut m = Marker::default();
            m.header.frame_id = self.cfg.frame_id.clone();
            m.header.stamp = self.now();
            m.ns = md.get("ns").and_then(|v| v.as_str()).ok_or("marker without ns")?.to_string();
            m.id = md.get("id").and_then(|v| v.as_i64()).ok_or("marker without id")? as i32;
            m.type_ = LINE_STRIP;
            m.action = ADD;
            let (color, scale) = if with_style {
                let c = md.get("color").and_then(|v| v.as_object());
                let f = |k: &str, d: f64| c.and_then(|c| c.get(k)).and_then(|v| v.as_f64()).unwrap_or(d);
                ((f("r", 0.0), f("g", 1.0), f("b", 0.0), f("a", 0.8)), md.get("scale").and_then(|v| v.as_f64()).unwrap_or(0.1))
            } else {
                default_style
            };
            m.scale.x = scale;
            m.color = ColorRGBA { r: color.0 as f32, g: color.1 as f32, b: color.2 as f32, a: color.3 as f32 };
            for p in md.get("points").and_then(|v| v.as_array()).ok_or("marker without points")? {
                let a = p.as_array().filter(|a| a.len() >= 3).ok_or("bad point")?;
                m.points.push(Point {
                    x: a[0].as_f64().ok_or("bad x")?,
                    y: a[1].as_f64().ok_or("bad y")?,
                    z: a[2].as_f64().ok_or("bad z")?,
                });
            }
            list.markers.push(m);
        }
        Ok(list)
    }

    fn load_list(&self, name: &str, default_style: ((f64, f64, f64, f64), f64), with_style: bool) -> Result<MarkerArray, String> {
        let text = std::fs::read_to_string(self.work_path(name)).map_err(|e| e.to_string())?;
        let doc: Value = serde_json::from_str(&text).map_err(|e| e.to_string())?;
        self.markers_from_json(&doc, default_style, with_style)
    }

    pub fn load_zone_list(&mut self, out: &dyn Outputs) -> bool {
        match self.load_list("zone_list.json", ZONE_STYLE, false) {
            Ok(l) => {
                self.zone_list = l;
                true
            }
            Err(e) => {
                out.error(&format!("載入區域列表失敗: {e}"));
                false
            }
        }
    }

    pub fn load_risk_list(&mut self, out: &dyn Outputs) -> bool {
        match self.load_list("risk_zone_list.json", RISK_STYLE, false) {
            Ok(l) => {
                self.risk_list = l;
                true
            }
            Err(e) => {
                out.error(&format!("載入風險區域列表失敗: {e}"));
                false
            }
        }
    }

    pub fn load_channel_list(&mut self, out: &dyn Outputs) -> bool {
        match self.load_list("chennal_path_list.json", ((0.0, 1.0, 0.0, 0.8), 0.1), true) {
            Ok(l) => {
                self.channel_list = l;
                true
            }
            Err(e) => {
                out.error(&format!("加载 chennal 路径列表失败: {e}"));
                false
            }
        }
    }

    /// 載入後把遞增計數器對齊清單裡的最大 id，避免之後錄製撞號.
    pub fn restore_id_counters(&mut self) {
        self.zone_id = self.zone_list.markers.iter().map(|m| m.id).max().unwrap_or(0);
        self.risk_id = self.risk_list.markers.iter().map(|m| m.id).max().unwrap_or(0);
        self.channel_id = self.channel_list.markers.iter().map(|m| m.id).max().unwrap_or(0);
    }

    pub fn publish_geometry_lists(&self, out: &dyn Outputs) {
        out.zone_list(&self.zone_list);
        out.risk_list(&self.risk_list);
        out.channel_list(&self.channel_list);
    }

    /// /save_zone_list
    pub fn save_zone_list_service(&mut self, out: &dyn Outputs) -> Reply {
        let mut ok = 0;
        let mut errors = Vec::new();
        if self.save_zone_list(out) { ok += 1 } else { errors.push("储存普通区域列表失败") }
        if self.save_risk_list(out) { ok += 1 } else { errors.push("储存风险区域列表失败") }
        if self.save_channel_list(out) { ok += 1 } else { errors.push("储存 channel 路径列表失败") }
        if ok == 3 {
            self.working_state_ready = true;
            (true, "成功储存所有列表（普通区域 + 风险区域 + channel 路径）".into())
        } else if ok > 0 {
            (false, format!("部分成功储存列表。错误: {}", errors.join("; ")))
        } else {
            (false, format!("储存列表失败: {}", errors.join("; ")))
        }
    }

    /// /load_zone_list body (the caller holds the mutation lease).
    pub fn load_zone_list_service(&mut self, out: &dyn Outputs) -> Reply {
        if self.recording() {
            return (false, "錄製進行中，請先結束或取消錄製再載入".into());
        }
        let original = (self.zone_list.clone(), self.risk_list.clone(), self.channel_list.clone());
        let mut errors = Vec::new();
        let zone_ok = self.load_zone_list(out);
        if !zone_ok { errors.push("載入普通區域列表失敗") }
        let risk_ok = self.load_risk_list(out);
        if !risk_ok { errors.push("載入風險區域列表失敗") }
        let channel_ok = self.load_channel_list(out);
        if !channel_ok { errors.push("載入 channel 路徑列表失敗") }
        if !(zone_ok && risk_ok && channel_ok) {
            (self.zone_list, self.risk_list, self.channel_list) = original;
            return (false, format!("載入列表失敗，既有資料已保留: {}", errors.join("; ")));
        }
        self.restore_id_counters();
        self.working_state_ready = true;
        self.publish_geometry_lists(out);
        (true, "成功載入所有列表（普通區域 + 風險區域 + channel 路徑）".into())
    }

    // ── named sites ────────────────────────────────────────────────────────

    pub fn on_map_datum(&mut self, text: &str, out: &dyn Outputs) {
        let parsed = serde_json::from_str::<Value>(text).map_err(|e| e.to_string()).and_then(|doc| Datum::from_map_datum(&doc));
        match parsed {
            Ok(d) => self.datum = Some(d),
            Err(e) => out.warn(&format!("解析 /adapter/map_datum 失敗: {e}")),
        }
    }

    pub fn site_list_json(&self) -> String {
        serde_json::to_string(&site_store::list_sites(&self.cfg.sites_dir, self.active_site.as_deref())).expect("json")
    }

    pub fn publish_site_list(&self, out: &dyn Outputs) {
        out.site_list(&self.site_list_json());
    }

    fn site_state_objects(&self) -> (Vec<XyObject>, Vec<XyObject>, Vec<XyObject>) {
        let polys = |l: &MarkerArray| -> Vec<XyObject> {
            l.markers.iter().map(|m| XyObject { id: m.id, ns: m.ns.clone(), points: xy_of(m), color: None, scale: None }).collect()
        };
        let channels = self
            .channel_list
            .markers
            .iter()
            .map(|m| XyObject {
                id: m.id,
                ns: m.ns.clone(),
                points: xy_of(m),
                color: Some((m.color.r as f64, m.color.g as f64, m.color.b as f64, m.color.a as f64)),
                scale: Some(m.scale.x),
            })
            .collect();
        (polys(&self.zone_list), polys(&self.risk_list), channels)
    }

    /// 用 site 內容（已轉回當下 map frame 的 XY）重建三個 MarkerArray.
    fn apply_site_objects(&mut self, zones: &[XyObject], risks: &[XyObject], channels: &[XyObject]) {
        let mut zone_list = MarkerArray::default();
        for o in zones {
            zone_list.markers.push(self.marker_from_xy(&o.ns, o.id, ZONE_STYLE.0, ZONE_STYLE.1, &o.points, true));
        }
        let mut risk_list = MarkerArray::default();
        for o in risks {
            risk_list.markers.push(self.marker_from_xy(&o.ns, o.id, RISK_STYLE.0, RISK_STYLE.1, &o.points, true));
        }
        let mut channel_list = MarkerArray::default();
        for o in channels {
            let c = o.color.unwrap_or((0.0, 1.0, 0.0, 0.8));
            channel_list.markers.push(self.marker_from_xy(&o.ns, o.id, c, o.scale.unwrap_or(0.1), &o.points, false));
        }
        self.zone_list = zone_list;
        self.risk_list = risk_list;
        self.channel_list = channel_list;
    }

    /// 編輯物件後同步覆寫啟用中的場地檔，讓場地與工作狀態一致.
    fn update_active_site(&mut self, out: &dyn Outputs) -> bool {
        if self.manifest_blocked.is_some() {
            out.error("啟用場地 manifest 未就緒，拒絕自動同步");
            return false;
        }
        let Some(active) = self.active_site.clone() else { return true };
        let Some(datum) = self.datum.clone() else {
            out.error("無 datum，無法同步場地檔");
            return false;
        };
        let (created, existing_source) = match site_store::read_active_site(&self.cfg.sites_dir) {
            Ok(manifest) if manifest.as_deref() == Some(active.as_str()) => match site_store::read_site(&self.cfg.sites_dir, &active) {
                Ok(existing) => (
                    existing.get("created_at").and_then(|v| v.as_str()).map(str::to_string),
                    existing.get("datum").and_then(|d| d.get("source")).and_then(|v| v.as_str()).map(str::to_string),
                ),
                Err(e) => {
                    self.manifest_blocked = Some(format!("無法讀取啟用場地: {e}"));
                    return false;
                }
            },
            Ok(_) => {
                self.manifest_blocked = Some("manifest 與記憶體的啟用場地不一致".into());
                return false;
            }
            Err(e) => {
                self.manifest_blocked = Some(format!("無法讀取啟用場地: {e}"));
                return false;
            }
        };
        // datum 來源改變（如開機後 fallback → navsat 鎖定）時不自動覆寫。
        if let Some(src) = &existing_source {
            if *src != datum.source {
                out.warn(&format!("datum 來源已由 {src} 變為 {}，跳過場地「{active}」自動同步", datum.source));
                return false;
            }
        }
        let (zones, risks, channels) = self.site_state_objects();
        let site = site_store::build_site(&active, &datum, &zones, &risks, &channels, created.as_deref());
        if let Err(e) = site_store::write_site(&self.cfg.sites_dir, &site) {
            self.manifest_blocked = Some(format!("場地檔同步結果無法確認: {e}"));
            out.error(&format!("場地檔同步失敗: {e}"));
            return false;
        }
        self.publish_site_list(out);
        true
    }

    pub fn site_save(&mut self, name: &str, out: &dyn Outputs) -> Reply {
        let Some(datum) = self.datum.clone() else {
            return (false, "datum 尚未就緒（等待 /adapter/map_datum），無法儲存場地".into());
        };
        if datum.source != "navsat" {
            return (false, "GPS datum 尚未由 NavSatFix 確認，拒絕儲存可執行場地".into());
        }
        let (zones, risks, channels) = self.site_state_objects();
        if zones.is_empty() && risks.is_empty() && channels.is_empty() {
            return (false, "目前沒有任何物件可存成場地".into());
        }
        if !self.save_all_working_state(out) {
            return (false, "三份工作檔未能完整儲存，拒絕啟用場地".into());
        }
        let dir = self.cfg.sites_dir.clone();
        let old_manifest = site_store::read_active_site(&dir).ok().flatten();
        let target_existed = site_store::site_path(&dir, name).exists();
        let mut old_site = None;
        let mut created = None;
        if target_existed {
            match site_store::read_site(&dir, name) {
                Ok(s) => {
                    created = s.get("created_at").and_then(|v| v.as_str()).map(str::to_string);
                    old_site = Some(s);
                }
                Err(e) => return (false, format!("既有場地檔損壞，拒絕覆寫: {e}")),
            }
        }
        let site = site_store::build_site(name, &datum, &zones, &risks, &channels, created.as_deref());
        if let Err(e) = site_store::write_site(&dir, &site) {
            return (false, format!("場地操作失敗: {e}"));
        }
        if let Err(e) = site_store::write_active_site(&dir, name) {
            let mut rollback = Vec::new();
            let r = match &old_site {
                Some(s) => site_store::write_site(&dir, s),
                None if site_store::site_path(&dir, name).exists() => site_store::delete_site(&dir, name),
                None => Ok(()),
            };
            if let Err(re) = r {
                rollback.push(format!("場地檔: {re}"));
            }
            let r = match &old_manifest {
                None => site_store::clear_active_site(&dir),
                Some(m) => site_store::write_active_site(&dir, m),
            };
            if let Err(re) = r {
                rollback.push(format!("manifest: {re}"));
            }
            let mut message = format!("啟用場地 manifest 寫入失敗: {e}");
            if !rollback.is_empty() {
                self.manifest_blocked = Some(rollback.join("; "));
                message.push_str(&format!("；回復失敗: {}", rollback.join("; ")));
            }
            return (false, message);
        }
        self.active_site = Some(name.to_string());
        self.manifest_blocked = None;
        self.working_state_ready = true;
        (true, format!("已儲存場地「{name}」（{} 工作區 / {} 禁區 / {} 通道，datum: {}）", zones.len(), risks.len(), channels.len(), datum.source))
    }

    pub fn site_load(&mut self, name: &str, out: &dyn Outputs) -> Reply {
        let Some(datum) = self.datum.clone() else {
            return (false, "datum 尚未就緒（等待 /adapter/map_datum），無法載入場地".into());
        };
        if self.recording() {
            return (false, "錄製進行中，請先結束或取消錄製再載入場地".into());
        }
        let dir = self.cfg.sites_dir.clone();
        if !site_store::site_path(&dir, name).exists() {
            return (false, format!("找不到場地「{name}」"));
        }
        let site = match site_store::read_site(&dir, name) {
            Ok(s) => s,
            Err(e) => return (false, format!("場地操作失敗: {e}")),
        };
        let site_source = site.get("datum").and_then(|d| d.get("source")).and_then(|v| v.as_str()).unwrap_or("").to_string();
        if datum.source != "navsat" {
            let cur = if datum.source.is_empty() { "未知".to_string() } else { datum.source.clone() };
            return (false, format!("GPS 尚未由 NavSatFix 確認（目前 datum 為 {cur}），拒絕啟用可執行場地"));
        }
        if site_source != datum.source {
            return (false, format!("此場地以 {site_source} datum 儲存，與目前 {} 不相容 — 請在相同定位條件下重新錄製或另存", datum.source));
        }
        let site_datum = site.get("datum").cloned().unwrap_or(Value::Null);
        let (Some(site_lat), Some(site_lon)) = (site_datum.get("lat").and_then(|v| v.as_f64()), site_datum.get("lon").and_then(|v| v.as_f64())) else {
            return (false, "場地 datum 格式無效，拒絕載入".into());
        };
        let max_distance_m = self.cfg.max_site_datum_distance_m;
        if ![site_lat, site_lon, datum.lat, datum.lon, max_distance_m].iter().all(|v| v.is_finite())
            || !(-90.0..=90.0).contains(&site_lat)
            || !(-180.0..=180.0).contains(&site_lon)
            || max_distance_m <= 0.0
        {
            return (false, "場地 datum 或允許距離設定無效，拒絕載入".into());
        }
        let (dx, dy) = site_store::xy_from_ll(site_lat, site_lon, &datum);
        let datum_distance_m = (dx * dx + dy * dy).sqrt();
        if datum_distance_m > max_distance_m {
            return (false, format!("場地「{name}」原點距目前定位約 {datum_distance_m:.1} m，超過安全上限 {max_distance_m:.1} m，拒絕啟用"));
        }
        let (old_manifest, old_manifest_valid, old_manifest_error) = match site_store::read_active_site(&dir) {
            Ok(m) => (m, true, String::new()),
            Err(e) => (None, false, e),
        };
        let old_state = (
            self.zone_list.clone(),
            self.risk_list.clone(),
            self.channel_list.clone(),
            self.active_site.clone(),
            self.manifest_blocked.clone(),
            self.working_state_ready,
        );
        let (zones, risks, channels) = match site_store::site_to_xy(&site, &datum) {
            Ok(t) => t,
            Err(e) => return (false, format!("場地操作失敗: {e}")),
        };
        self.apply_site_objects(&zones, &risks, &channels);
        self.restore_id_counters();
        let restore = |this: &mut Recorder, old: &(MarkerArray, MarkerArray, MarkerArray, Option<String>, Option<String>, bool)| {
            this.zone_list = old.0.clone();
            this.risk_list = old.1.clone();
            this.channel_list = old.2.clone();
            this.active_site = old.3.clone();
            this.manifest_blocked = old.4.clone();
            this.working_state_ready = old.5;
            this.restore_id_counters();
        };
        if !self.save_all_working_state(out) {
            restore(self, &old_state);
            let repaired = self.save_all_working_state(out);
            let mut message = "工作檔同步失敗；場地未啟用，既有資料已回復".to_string();
            if !repaired {
                self.manifest_blocked = Some("舊工作檔回復失敗".into());
                message.push_str("（舊工作檔回復也失敗，請立即備份並檢查磁碟）");
            }
            return (false, message);
        }
        if let Err(e) = site_store::write_active_site(&dir, name) {
            restore(self, &old_state);
            let repaired = self.save_all_working_state(out);
            let mut manifest_repaired = old_manifest_valid;
            if old_manifest_valid {
                let r = match &old_manifest {
                    None => site_store::clear_active_site(&dir),
                    Some(m) => site_store::write_active_site(&dir, m),
                };
                if let Err(me) = r {
                    manifest_repaired = false;
                    out.error(&format!("舊 manifest 回復失敗: {me}"));
                }
            }
            if !repaired {
                self.manifest_blocked = Some("manifest 寫入失敗且舊工作檔回復失敗".into());
            }
            if !manifest_repaired {
                self.manifest_blocked = Some(if old_manifest_valid { "舊 manifest 回復失敗".into() } else { old_manifest_error.clone() });
            }
            let mut message = format!("啟用場地 manifest 寫入失敗: {e}");
            if !repaired {
                message.push_str("；舊工作檔回復也失敗，請立即檢查磁碟");
            }
            if !manifest_repaired {
                message.push_str("；舊 manifest 狀態無法確認");
            }
            return (false, message);
        }
        self.active_site = Some(name.to_string());
        self.manifest_blocked = None;
        self.working_state_ready = true;
        self.publish_geometry_lists(out);
        (true, format!("已載入場地「{name}」（{} 工作區 / {} 禁區 / {} 通道）", zones.len(), risks.len(), channels.len()))
    }

    pub fn site_delete(&mut self, name: &str) -> Reply {
        let dir = self.cfg.sites_dir.clone();
        let path = site_store::site_path(&dir, name);
        if !path.exists() {
            return (false, format!("找不到場地「{name}」"));
        }
        let backup = match site_store::read_site(&dir, name) {
            Ok(s) => s,
            Err(e) => return (false, format!("場地操作失敗: {e}")),
        };
        let manifest = match site_store::read_active_site(&dir) {
            Ok(m) => m,
            Err(e) => return (false, format!("場地操作失敗: {e}")),
        };
        if self.active_site != manifest {
            let why = "manifest 與記憶體的啟用場地不一致".to_string();
            self.manifest_blocked = Some(why.clone());
            return (false, why);
        }
        let was_active = manifest.as_deref() == Some(name);
        let result = (|| -> Result<(), String> {
            if was_active {
                site_store::clear_active_site(&dir)?;
            }
            site_store::delete_site(&dir, name)
        })();
        if let Err(e) = result {
            let mut rollback = Vec::new();
            if !path.exists() {
                if let Err(re) = site_store::write_site(&dir, &backup) {
                    rollback.push(format!("場地檔: {re}"));
                }
            }
            if was_active {
                if let Err(re) = site_store::write_active_site(&dir, name) {
                    rollback.push(format!("manifest: {re}"));
                }
            }
            if !rollback.is_empty() {
                self.manifest_blocked = Some(rollback.join("; "));
            }
            let mut message = format!("刪除場地失敗: {e}");
            if !rollback.is_empty() {
                message.push_str(&format!("；回復失敗: {}", rollback.join("; ")));
            }
            return (false, message);
        }
        if was_active {
            self.active_site = None;
        }
        (true, format!("已刪除場地「{name}」"))
    }

    pub fn site_rename(&mut self, name: &str, new_name: Option<&str>) -> Reply {
        let Some(new_name) = new_name else { return (false, "新場地名稱無效".into()) };
        let dir = self.cfg.sites_dir.clone();
        let old_path = site_store::site_path(&dir, name);
        let new_path = site_store::site_path(&dir, new_name);
        if !old_path.exists() {
            return (false, format!("找不到場地「{name}」"));
        }
        if new_path.exists() {
            return (false, format!("場地「{new_name}」已存在"));
        }
        let old_site = match site_store::read_site(&dir, name) {
            Ok(s) => s,
            Err(e) => return (false, format!("場地操作失敗: {e}")),
        };
        let mut new_site = old_site.clone();
        new_site["name"] = json!(new_name);
        let manifest = match site_store::read_active_site(&dir) {
            Ok(m) => m,
            Err(e) => return (false, format!("場地操作失敗: {e}")),
        };
        if self.active_site != manifest {
            let why = "manifest 與記憶體的啟用場地不一致".to_string();
            self.manifest_blocked = Some(why.clone());
            return (false, why);
        }
        let was_active = manifest.as_deref() == Some(name);
        let result = (|| -> Result<(), String> {
            site_store::write_site(&dir, &new_site)?;
            if was_active {
                site_store::write_active_site(&dir, new_name)?;
            }
            site_store::delete_site(&dir, name)
        })();
        if let Err(e) = result {
            let mut rollback = Vec::new();
            let r = (|| -> Result<(), String> {
                if !old_path.exists() {
                    site_store::write_site(&dir, &old_site)?;
                }
                if new_path.exists() {
                    site_store::delete_site(&dir, new_name)?;
                }
                Ok(())
            })();
            if let Err(re) = r {
                rollback.push(format!("場地檔: {re}"));
            }
            if was_active {
                if let Err(re) = site_store::write_active_site(&dir, name) {
                    rollback.push(format!("manifest: {re}"));
                }
            }
            if !rollback.is_empty() {
                self.manifest_blocked = Some(rollback.join("; "));
            }
            let mut message = format!("場地改名失敗: {e}");
            if !rollback.is_empty() {
                message.push_str(&format!("；回復失敗: {}", rollback.join("; ")));
            }
            return (false, message);
        }
        if was_active {
            self.active_site = Some(new_name.to_string());
        }
        (true, format!("已將場地「{name}」改名為「{new_name}」"))
    }

    /// Atomically restore all work files and their named-site association.
    pub fn restore_startup_persistence(&mut self, out: &dyn Outputs) {
        let manifest_path = site_store::active_site_path(&self.cfg.sites_dir);
        let manifest_exists = manifest_path.exists();
        let work_exists = ["zone_list.json", "risk_zone_list.json", "chennal_path_list.json"].iter().any(|n| self.work_path(n).exists());
        if !manifest_exists && !work_exists {
            // A pristine installation: create all three empty snapshots now.
            if self.save_all_working_state(out) {
                self.working_state_ready = true;
            } else {
                self.manifest_blocked = Some("無法初始化三份工作檔".into());
                out.error("任務幾何持久化目錄未就緒");
            }
            return;
        }
        let original = (self.zone_list.clone(), self.risk_list.clone(), self.channel_list.clone());
        let work_ok = {
            let a = self.load_zone_list(out);
            let b = self.load_risk_list(out);
            let c = self.load_channel_list(out);
            a && b && c
        };
        if work_ok {
            self.working_state_ready = true;
            self.restore_id_counters();
            self.publish_geometry_lists(out);
        } else {
            (self.zone_list, self.risk_list, self.channel_list) = original;
            if !manifest_exists {
                self.manifest_blocked = Some("三份工作檔不完整".into());
            }
        }
        let mut manifest_name = None;
        if manifest_exists {
            match site_store::read_active_site(&self.cfg.sites_dir) {
                Ok(m) => manifest_name = m,
                Err(e) => self.manifest_blocked = Some(format!("啟用場地 manifest 損壞: {e}")),
            }
            if manifest_name.is_none() && self.manifest_blocked.is_none() {
                self.manifest_blocked = Some("啟用場地 manifest 在啟動時消失".into());
            }
        }
        if let Some(name) = manifest_name {
            let site_exists = site_store::site_path(&self.cfg.sites_dir, &name).exists();
            if work_ok && site_exists {
                self.active_site = Some(name.clone());
                out.info(&format!("已復原工作檔與啟用場地「{name}」"));
            } else {
                self.manifest_blocked = Some(if !work_ok { "三份工作檔不完整".into() } else { format!("場地檔「{name}」不存在") });
            }
        }
        if let Some(why) = &self.manifest_blocked {
            self.active_site = None;
            out.error(&format!("持久化狀態未就緒: {why}；請使用 /site_op load 重新啟用場地"));
        }
    }

    // ── channel router ─────────────────────────────────────────────────────

    fn zone_polygon(&self, zone_id: i32) -> Option<Vec<Xy>> {
        self.zone_list.markers.iter().find(|m| m.id == zone_id).map(xy_of)
    }

    /// 返回連接兩個 zone 的通道路徑，方向保證從 zone_from 走向 zone_to.
    pub fn channel_route(&self, zone_from: i32, zone_to: i32, proximity_m: f32, out: &dyn Outputs) -> Result<(PathMsg, i32, String), String> {
        let proximity = if proximity_m > 0.0 { proximity_m as f64 } else { 1.5 };
        out.info(&format!("get_channel_route: zone_from={zone_from}, zone_to={zone_to}, proximity={proximity:.2}m"));
        if self.channel_list.markers.is_empty() {
            return Err("尚無通道數據，請先錄製或載入通道路徑".into());
        }
        let Some(from_pts) = self.zone_polygon(zone_from) else {
            out.error(&format!("找不到 zone {zone_from} 的多邊形數據"));
            return Err(format!("找不到連接 zone {zone_from} → zone {zone_to} 的通道，共搜尋 {} 條通道", self.channel_list.markers.len()));
        };
        let Some(to_pts) = self.zone_polygon(zone_to) else {
            out.error(&format!("找不到 zone {zone_to} 的多邊形數據"));
            return Err(format!("找不到連接 zone {zone_from} → zone {zone_to} 的通道，共搜尋 {} 條通道", self.channel_list.markers.len()));
        };
        for m in &self.channel_list.markers {
            if m.points.len() < 2 {
                continue;
            }
            let (s, e) = (&m.points[0], &m.points[m.points.len() - 1]);
            if geometry::point_near_polygon(s.x, s.y, &from_pts, proximity) && geometry::point_near_polygon(e.x, e.y, &to_pts, proximity) {
                out.info(&format!("通道 #{}: start 近 zone {zone_from}，end 近 zone {zone_to}，正向匹配", m.id));
                let path = self.channel_marker_to_path(m, false);
                let n = path.poses.len();
                return Ok((path, m.id, format!("找到通道 #{}，連接 zone {zone_from} → zone {zone_to}，共 {n} 個路徑點", m.id)));
            }
            if geometry::point_near_polygon(s.x, s.y, &to_pts, proximity) && geometry::point_near_polygon(e.x, e.y, &from_pts, proximity) {
                out.info(&format!("通道 #{}: start 近 zone {zone_to}，end 近 zone {zone_from}，反向匹配，翻轉路徑", m.id));
                let path = self.channel_marker_to_path(m, true);
                let n = path.poses.len();
                return Ok((path, m.id, format!("找到通道 #{}，連接 zone {zone_from} → zone {zone_to}，共 {n} 個路徑點", m.id)));
            }
        }
        Err(format!("找不到連接 zone {zone_from} → zone {zone_to} 的通道，共搜尋 {} 條通道", self.channel_list.markers.len()))
    }

    /// 將通道 Marker (LINE_STRIP) 轉換為帶航向角的 nav_msgs/Path.
    fn channel_marker_to_path(&self, m: &Marker, reverse: bool) -> PathMsg {
        let mut path = empty_path(&self.cfg.frame_id);
        path.header.stamp = self.now();
        let mut points: Vec<&Point> = m.points.iter().collect();
        if reverse {
            points.reverse();
        }
        let n = points.len();
        for (i, pt) in points.iter().enumerate() {
            let mut pose = PoseStamped::default();
            pose.header = path.header.clone();
            pose.pose.position.x = pt.x;
            pose.pose.position.y = pt.y;
            let (dx, dy) = if i + 1 < n {
                (points[i + 1].x - pt.x, points[i + 1].y - pt.y)
            } else if i > 0 {
                (pt.x - points[i - 1].x, pt.y - points[i - 1].y)
            } else {
                (1.0, 0.0)
            };
            let yaw = dy.atan2(dx);
            pose.pose.orientation.z = (yaw / 2.0).sin();
            pose.pose.orientation.w = (yaw / 2.0).cos();
            path.poses.push(pose);
        }
        path
    }

    pub fn zone_ids(&self) -> Vec<i32> {
        self.zone_list.markers.iter().map(|m| m.id).collect()
    }
}

/// Wall-clock stamp helper for the node.
pub fn wall_stamp() -> Time {
    let d = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or(Duration::ZERO);
    Time { sec: d.as_secs() as i32, nanosec: d.subsec_nanos() }
}

#[allow(dead_code)]
pub fn since(t: Instant) -> f64 {
    t.elapsed().as_secs_f64()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;

    #[derive(Default)]
    struct Log {
        zone_lists: RefCell<Vec<usize>>,
        risk_lists: RefCell<Vec<usize>>,
        channel_lists: RefCell<Vec<usize>>,
        markers: RefCell<Vec<usize>>,
        paths: RefCell<Vec<usize>>,
        messages: RefCell<Vec<String>>,
    }

    impl Outputs for Log {
        fn recorded_path(&self, p: &PathMsg) { self.paths.borrow_mut().push(p.poses.len()); }
        fn zone_marker(&self, m: &Marker) { self.markers.borrow_mut().push(m.points.len()); }
        fn zone_list(&self, m: &MarkerArray) { self.zone_lists.borrow_mut().push(m.markers.len()); }
        fn risk_marker(&self, m: &Marker) { self.markers.borrow_mut().push(m.points.len()); }
        fn risk_list(&self, m: &MarkerArray) { self.risk_lists.borrow_mut().push(m.markers.len()); }
        fn channel_path(&self, p: &PathMsg) { self.paths.borrow_mut().push(p.poses.len()); }
        fn channel_list(&self, m: &MarkerArray) { self.channel_lists.borrow_mut().push(m.markers.len()); }
        fn site_list(&self, _json: &str) {}
        fn info(&self, m: &str) { self.messages.borrow_mut().push(m.to_string()); }
        fn warn(&self, m: &str) { self.messages.borrow_mut().push(m.to_string()); }
        fn error(&self, m: &str) { self.messages.borrow_mut().push(format!("E {m}")); }
    }

    fn recorder(dir: &Path) -> Recorder {
        let cfg = Config {
            frame_id: "map".into(),
            save_dir: dir.join("zone_record").to_string_lossy().into_owned(),
            sites_dir: dir.join("sites").to_string_lossy().into_owned(),
            min_dist: 0.05,
            min_dt: 0.0,
            polygon_simplify_dist: 0.1,
            max_site_datum_distance_m: 100.0,
            robot_pose_max_age_s: 1.0,
        };
        Recorder::new(cfg, Box::new(wall_stamp))
    }

    fn pose(x: f64, y: f64) -> PoseStamped {
        let mut p = PoseStamped::default();
        p.header.stamp = wall_stamp();
        p.header.frame_id = "map".into();
        p.pose.position.x = x;
        p.pose.position.y = y;
        p.pose.orientation.w = 1.0;
        p
    }

    fn tmp(name: &str) -> std::path::PathBuf {
        let d = std::env::temp_dir().join(format!("mower_record_{name}_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    #[test]
    fn zone_recording_round_trip_and_persistence() {
        let dir = tmp("zone");
        let mut r = recorder(&dir);
        let log = Log::default();
        r.restore_startup_persistence(&log);
        assert!(r.working_state_ready);
        assert!(dir.join("zone_record/zone_list.json").exists());
        r.on_pose_source("map", "base_footprint", pose(0.0, 0.0));
        assert!(r.try_initialize(&log));
        assert!(r.check_start(Kind::Zone).is_ok());
        assert_eq!(r.start_zone(&log).0, true);
        assert!(r.check_start(Kind::Risk).is_err());
        assert_eq!(r.end_zone(&log), Err("區域點數不足，至少需要 3 個點".into()));
        for (x, y) in [(2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.1), (1.9, 0.05)] {
            r.on_pose_source("map", "base_footprint", pose(x, y));
            assert!(r.sample(&log));
        }
        let msg = r.end_zone(&log).unwrap();
        assert_eq!(msg, "成功記錄區域結束點 #1");
        assert!(!r.recording());
        assert!(!r.sample(&log));
        assert_eq!(r.zone_list.markers.len(), 1);
        // closed polygon: the trace ends within 1 m of its start, so the
        // first vertex is appended again (as in create_polygon_from_path)
        let pts = &r.zone_list.markers[0].points;
        assert!(pts.len() >= 5, "{}", pts.len());
        assert_eq!((pts[0].x, pts[0].y), (pts[pts.len() - 1].x, pts[pts.len() - 1].y));
        assert_eq!((pts[0].x, pts[0].y), (2.0, 0.0));
        // persisted with the Python layout
        let saved: Value = serde_json::from_str(&std::fs::read_to_string(dir.join("zone_record/zone_list.json")).unwrap()).unwrap();
        assert_eq!(saved[0]["id"], json!(1));
        assert_eq!(saved[0]["ns"], json!("zones"));
        assert_eq!(saved[0]["points"][0], json!([2.0, 0.0, 0.0]));
        // reload into a fresh recorder
        let mut r2 = recorder(&dir);
        r2.restore_startup_persistence(&log);
        assert_eq!(r2.zone_list.markers.len(), 1);
        assert_eq!(r2.zone_id, 1);
        assert_eq!(r2.zone_list.markers[0].color.b, 1.0);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn edit_zone_add_update_delete_and_ids() {
        let dir = tmp("edit");
        let mut r = recorder(&dir);
        let log = Log::default();
        r.restore_startup_persistence(&log);
        let (msg, id) = r.edit(&log, "add", "risk", 0, &[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]).unwrap();
        assert_eq!((msg.as_str(), id), ("已新增 risk id=1", 1));
        assert_eq!(r.risk_list.markers[0].points.len(), 4); // closed
        assert_eq!(r.risk_id, 1);
        assert_eq!(r.edit(&log, "add", "channel", 0, &[(0.0, 0.0)]).unwrap_err(), "channel 頂點不足（1 < 2）");
        let (_, cid) = r.edit(&log, "add", "channel", 0, &[(0.0, 0.0), (5.0, 0.0)]).unwrap();
        assert_eq!(cid, 1);
        assert_eq!(r.channel_list.markers[0].points.len(), 2); // open
        assert_eq!(r.edit(&log, "update", "risk", 9, &[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]).unwrap_err(), "找不到 risk id=9");
        r.edit(&log, "update", "risk", 1, &[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]).unwrap();
        assert_eq!(r.risk_list.markers[0].points[1].x, 2.0);
        assert_eq!(r.edit(&log, "delete", "zone", 3, &[]).unwrap_err(), "找不到 zone id=3");
        r.edit(&log, "delete", "risk", 1, &[]).unwrap();
        assert!(r.risk_list.markers.is_empty());
        assert_eq!(r.edit(&log, "add", "blob", 0, &[]).unwrap_err(), "未知 kind: blob（要 zone/risk/channel）");
        let saved: Value = serde_json::from_str(&std::fs::read_to_string(dir.join("zone_record/chennal_path_list.json")).unwrap()).unwrap();
        assert_eq!(saved[0]["scale"], json!(0.03));
        assert_eq!(saved[0]["color"]["g"], json!(0.7f32 as f64));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn channel_route_direction_and_site_save_load() {
        let dir = tmp("route");
        let mut r = recorder(&dir);
        let log = Log::default();
        r.restore_startup_persistence(&log);
        r.edit(&log, "add", "zone", 0, &[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]).unwrap();
        r.edit(&log, "add", "zone", 0, &[(10.0, 0.0), (14.0, 0.0), (14.0, 4.0), (10.0, 4.0)]).unwrap();
        r.edit(&log, "add", "channel", 0, &[(12.0, 2.0), (7.0, 2.0), (2.0, 2.0)]).unwrap();
        let (path, id, msg) = r.channel_route(1, 2, 0.0, &log).unwrap();
        assert_eq!(id, 1);
        assert!(msg.starts_with("找到通道 #1"));
        // reversed so that it starts in zone 1 and heads +x
        assert_eq!(path.poses[0].pose.position.x, 2.0);
        assert!(path.poses[0].pose.orientation.z.abs() < 1e-9 && path.poses[0].pose.orientation.w > 0.99);
        assert!(r.channel_route(1, 7, 0.0, &log).unwrap_err().starts_with("找不到連接"));
        // sites need a navsat datum
        assert_eq!(r.site_save("後院", &log).0, false);
        r.on_map_datum(r#"{"origin_lat": 23.6939508, "origin_lon": 120.5376539, "bearing_rad": 0.0, "source": "fallback"}"#, &log);
        assert_eq!(r.site_save("後院", &log).1, "GPS datum 尚未由 NavSatFix 確認，拒絕儲存可執行場地");
        r.on_map_datum(r#"{"origin_lat": 23.6939508, "origin_lon": 120.5376539, "bearing_rad": 0.0, "source": "navsat"}"#, &log);
        let (ok, msg) = r.site_save("後院", &log);
        assert!(ok, "{msg}");
        assert_eq!(msg, "已儲存場地「後院」（2 工作區 / 0 禁區 / 1 通道，datum: navsat）");
        assert_eq!(r.active_site.as_deref(), Some("後院"));
        assert_eq!(site_store::read_active_site(&r.cfg.sites_dir).unwrap().as_deref(), Some("後院"));
        // an edit syncs the active site file
        r.edit(&log, "add", "risk", 0, &[(1.0, 1.0), (2.0, 1.0), (2.0, 2.0)]).unwrap();
        let site = site_store::read_site(&r.cfg.sites_dir, "後院").unwrap();
        assert_eq!(site["meta"]["risk_count"], json!(1));
        // load under a rotated datum: geometry reprojects, ids restored
        r.on_map_datum(r#"{"origin_lat": 23.6942, "origin_lon": 120.5379, "bearing_rad": 0.5236, "source": "navsat"}"#, &log);
        let (ok, msg) = r.site_load("後院", &log);
        assert!(ok, "{msg}");
        assert_eq!(r.zone_list.markers.len(), 2);
        assert_eq!(r.zone_id, 2);
        assert_ne!(r.zone_list.markers[0].points[0].x, 0.0);
        assert_eq!(r.site_load("nope", &log).1, "找不到場地「nope」");
        let (ok, msg) = r.site_rename("後院", Some("前院"));
        assert!(ok, "{msg}");
        assert_eq!(r.active_site.as_deref(), Some("前院"));
        assert_eq!(r.site_delete("前院").0, true);
        assert_eq!(r.active_site, None);
        assert!(r.site_list_json().contains("\"sites\":[]"));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
