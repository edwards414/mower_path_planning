//! mower_record: zone / risk-zone / channel recorder and the named-site
//! library (port of `mower_mission/path_record_node.py`, node name
//! `path_recorder`, same topics, services, files and messages).
//!
//! The Python node needed a MultiThreadedExecutor because its service
//! callbacks block on the mission-operation lock client; here every
//! service is an async task that awaits the lock, and all handlers share
//! one async mutex so they run one at a time exactly as the rclpy callback
//! group did. The 10 Hz pose sampler is a tokio interval that only does
//! work while a recording is active.

mod geometry;
mod recorder;
mod site_store;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use futures::StreamExt;
use mower_rs_common::params;
use r2r::mower_interface::srv::{ChannelPathList, ChannelRoute, ChennalPathList, EditZone, GetZoneList, MissionOperationLock, SiteOp};
use r2r::nav_msgs::msg::{Odometry, Path as PathMsg};
use r2r::std_msgs::msg::{Bool, String as StringMsg};
use r2r::std_srvs::srv::Trigger;
use r2r::visualization_msgs::msg::{Marker, MarkerArray};
use r2r::QosProfile;
use tokio::sync::Mutex;

use mower_rs_common::guard::{Guard, RELEASE_UNCONFIRMED};
use crate::recorder::{Config, Kind, Outputs, Recorder};

struct Publishers {
    logger: String,
    recorded_path: r2r::Publisher<PathMsg>,
    zone_marker: r2r::Publisher<Marker>,
    zone_list: r2r::Publisher<MarkerArray>,
    risk_marker: r2r::Publisher<Marker>,
    risk_list: r2r::Publisher<MarkerArray>,
    chennal_path: r2r::Publisher<PathMsg>,
    channel_path: r2r::Publisher<PathMsg>,
    chennal_path_array: r2r::Publisher<MarkerArray>,
    channel_path_array: r2r::Publisher<MarkerArray>,
    site_list: r2r::Publisher<StringMsg>,
}

impl Outputs for Publishers {
    fn recorded_path(&self, p: &PathMsg) {
        let _ = self.recorded_path.publish(p);
    }
    fn zone_marker(&self, m: &Marker) {
        let _ = self.zone_marker.publish(m);
    }
    fn zone_list(&self, m: &MarkerArray) {
        let _ = self.zone_list.publish(m);
    }
    fn risk_marker(&self, m: &Marker) {
        let _ = self.risk_marker.publish(m);
    }
    fn risk_list(&self, m: &MarkerArray) {
        let _ = self.risk_list.publish(m);
    }
    fn channel_path(&self, p: &PathMsg) {
        let _ = self.chennal_path.publish(p);
        let _ = self.channel_path.publish(p);
    }
    fn channel_list(&self, m: &MarkerArray) {
        let _ = self.chennal_path_array.publish(m);
        let _ = self.channel_path_array.publish(m);
    }
    fn site_list(&self, json: &str) {
        let _ = self.site_list.publish(&StringMsg { data: json.to_string() });
    }
    fn info(&self, msg: &str) {
        r2r::log_info!(&self.logger, "{}", msg);
    }
    fn warn(&self, msg: &str) {
        r2r::log_warn!(&self.logger, "{}", msg);
    }
    fn error(&self, msg: &str) {
        r2r::log_error!(&self.logger, "{}", msg);
    }
}

struct App {
    rec: Recorder,
    guard: Guard,
    out: Publishers,
}

type Shared = Arc<Mutex<App>>;

fn latched() -> QosProfile {
    QosProfile::default().keep_last(1).reliable().transient_local()
}

fn expand_user(path: &str) -> String {
    if let Some(rest) = path.strip_prefix("~/") {
        let home = std::env::var("HOME").unwrap_or_else(|_| "/".into());
        return format!("{}/{}", home.trim_end_matches('/'), rest);
    }
    path.to_string()
}

/// Lock-release suffix appended by every recorder service that ends a
/// lease (kept verbatim from the Python node; the app shows it).
const RELEASE_SUFFIX_ZH: &str = "；操作鎖釋放未確認，導航仍被禁止；請檢查機器後重啟 path_record_node 與 nav_action_server";
const RELEASE_SUFFIX_SAVED_ZH: &str = "；資料已儲存，但操作鎖釋放未確認，導航仍被禁止；請檢查機器後重啟 path_record_node 與 nav_action_server";

/// `_reject_start_while_recording`: manifest check, lease, detach the site.
async fn begin_recording(app: &mut App, kind: Kind) -> Result<(), String> {
    app.rec.check_start(kind)?;
    if let Some(msg) = app.rec.blocked_manifest_reply(&format!("開始 {} 錄製", kind.label())) {
        app.out.error(&msg);
        return Err(msg);
    }
    app.guard.acquire(&format!("record {} geometry", kind.label())).await?;
    // Recording may describe a new field. Detach the prior named site
    // durably so a later edit cannot overwrite that site by accident.
    if app.rec.active_site.is_some() {
        if let Err(e) = site_store::clear_active_site(&app.rec.cfg.sites_dir) {
            let mut msg = format!("無法清除舊場地關聯，拒絕開始錄製: {e}");
            if !app.guard.release().await {
                msg.push_str(RELEASE_SUFFIX_ZH);
            }
            return Err(msg);
        }
        app.rec.active_site = None;
        app.rec.publish_site_list(&app.out);
    }
    Ok(())
}

async fn end_recording(app: &mut App, result: Result<String, String>) -> (bool, String) {
    match result {
        Err(msg) => (false, msg),
        Ok(mut msg) => {
            let released = app.guard.release().await;
            if !released {
                msg.push_str(RELEASE_SUFFIX_SAVED_ZH);
            }
            (released, msg)
        }
    }
}

macro_rules! trigger_service {
    ($node:expr, $shared:expr, $name:expr, |$app:ident| $body:expr) => {{
        let mut stream = $node.create_service::<Trigger::Service>($name, QosProfile::services_default())?;
        let shared: Shared = $shared.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let (success, message) = {
                    let mut guard = shared.lock().await;
                    let $app: &mut App = &mut guard;
                    $body
                };
                let _ = req.respond(Trigger::Response { success, message });
            }
        });
    }};
}

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "path_recorder", "")?;
    let logger = node.logger().to_string();
    let cfg = Config {
        frame_id: params::string(&node, "frame_id", "map"),
        save_dir: params::string(&node, "save_dir", "zone_record"),
        sites_dir: expand_user(&params::string(&node, "sites_dir", "~/.mower/sites")),
        min_dist: params::f64(&node, "min_dist", 0.05),
        min_dt: params::f64(&node, "min_dt", 0.10),
        polygon_simplify_dist: params::f64(&node, "polygon_simplify_dist", 0.1),
        max_site_datum_distance_m: params::f64(&node, "max_site_datum_distance_m", 100.0),
        robot_pose_max_age_s: params::f64(&node, "robot_pose_max_age_s", 1.0),
    };
    let pose_topic = params::string(&node, "robot_pose_source_topic", "/odometry/global");
    std::fs::create_dir_all(&cfg.save_dir)?;
    r2r::log_info!(&logger, "path_recorder ready (mower_rs).");

    let out = Publishers {
        logger: logger.clone(),
        recorded_path: node.create_publisher::<PathMsg>("/recorded_path", QosProfile::default())?,
        zone_marker: node.create_publisher::<Marker>("/zone_markers", latched())?,
        zone_list: node.create_publisher::<MarkerArray>("/zone_list", latched())?,
        risk_marker: node.create_publisher::<Marker>("/risk_zone_markers", latched())?,
        risk_list: node.create_publisher::<MarkerArray>("/risk_zone_list", latched())?,
        chennal_path: node.create_publisher::<PathMsg>("/chennal_path", QosProfile::default())?,
        channel_path: node.create_publisher::<PathMsg>("/channel_path", QosProfile::default())?,
        chennal_path_array: node.create_publisher::<MarkerArray>("/chennal_path_array", latched())?,
        channel_path_array: node.create_publisher::<MarkerArray>("/channel_path_array", latched())?,
        site_list: node.create_publisher::<StringMsg>("/site_list", latched())?,
    };
    let lock_client = node.create_client::<MissionOperationLock::Service>("/mission_operation_lock", QosProfile::services_default())?;
    let owner = format!("{}:{}", node.fully_qualified_name()?, uuid_hex());
    let guard = Guard::new(owner, lock_client, logger.clone());

    let mut rec = Recorder::new(cfg, Box::new(recorder::wall_stamp));
    rec.restore_startup_persistence(&out);
    rec.publish_site_list(&out);
    let shared: Shared = Arc::new(Mutex::new(App { rec, guard, out }));

    // ---- inputs ------------------------------------------------------------
    {
        let mut stream = node.subscribe::<Odometry>(&pose_topic, QosProfile::default())?;
        let shared = shared.clone();
        tokio::spawn(async move {
            while let Some(odom) = stream.next().await {
                let pose = r2r::geometry_msgs::msg::PoseStamped { header: odom.header.clone(), pose: odom.pose.pose.clone() };
                shared.lock().await.rec.on_pose_source(&odom.header.frame_id, &odom.child_frame_id, pose);
            }
        });
    }
    {
        let mut stream = node.subscribe::<StringMsg>("/adapter/map_datum", latched())?;
        let shared = shared.clone();
        tokio::spawn(async move {
            while let Some(msg) = stream.next().await {
                let mut app = shared.lock().await;
                let App { rec, out, .. } = &mut *app;
                rec.on_map_datum(&msg.data, out);
            }
        });
    }
    {
        let mut stream = node.subscribe::<Bool>("/nav_operation_active", latched())?;
        let shared = shared.clone();
        tokio::spawn(async move {
            while let Some(msg) = stream.next().await {
                let mut app = shared.lock().await;
                app.guard.nav_active = msg.data;
                app.guard.nav_seen = true;
            }
        });
    }
    // 0.5 s until the first pose, then 10 Hz sampling while recording.
    {
        let shared = shared.clone();
        tokio::spawn(async move {
            let mut init = tokio::time::interval(Duration::from_millis(500));
            loop {
                init.tick().await;
                let mut app = shared.lock().await;
                let App { rec, out, .. } = &mut *app;
                if rec.try_initialize(out) {
                    break;
                }
            }
            let mut tick = tokio::time::interval(Duration::from_millis(100));
            loop {
                tick.tick().await;
                let mut app = shared.lock().await;
                if app.rec.recording() {
                    let App { rec, out, .. } = &mut *app;
                    rec.sample(out);
                }
            }
        });
    }

    // ---- recording services ------------------------------------------------
    trigger_service!(node, shared, "/record_zone_start", |app| match begin_recording(app, Kind::Zone).await {
        Err(msg) => (false, msg),
        Ok(()) => {
            let App { rec, out, .. } = app;
            rec.start_zone(out)
        }
    });
    trigger_service!(node, shared, "/record_zone_end", |app| {
        let result = {
            let App { rec, out, .. } = &mut *app;
            rec.end_zone(out)
        };
        end_recording(app, result).await
    });
    trigger_service!(node, shared, "/risk_zone_start", |app| match begin_recording(app, Kind::Risk).await {
        Err(msg) => (false, msg),
        Ok(()) => {
            let App { rec, out, .. } = app;
            rec.start_risk(out)
        }
    });
    trigger_service!(node, shared, "/risk_zone_end", |app| {
        let result = {
            let App { rec, out, .. } = &mut *app;
            rec.end_risk(out)
        };
        end_recording(app, result).await
    });
    for name in ["/channel_record_start", "/chennal_record_start"] {
        trigger_service!(node, shared, name, |app| match begin_recording(app, Kind::Channel).await {
            Err(msg) => (false, msg),
            Ok(()) => {
                let App { rec, out, .. } = app;
                rec.start_channel(out)
            }
        });
    }
    for name in ["/channel_record_end", "/chennal_record_end"] {
        trigger_service!(node, shared, name, |app| {
            let result = {
                let App { rec, out, .. } = &mut *app;
                rec.end_channel(out)
            };
            end_recording(app, result).await
        });
    }
    trigger_service!(node, shared, "/record_cancel", |app| {
        let cancelled = {
            let App { rec, out, .. } = &mut *app;
            rec.cancel(out)
        };
        let (mut success, mut message) = match cancelled {
            None => (true, "目前沒有進行中的記錄".to_string()),
            Some(kind) => (true, format!("已取消 {} 記錄", kind.label())),
        };
        if cancelled.is_some() && !app.guard.release().await {
            success = false;
            message.push_str(RELEASE_SUFFIX_ZH);
        }
        app.out.info(&message);
        (success, message)
    });
    trigger_service!(node, shared, "/save_zone_list", |app| {
        let App { rec, out, .. } = app;
        rec.save_zone_list_service(out)
    });
    trigger_service!(node, shared, "/load_zone_list", |app| {
        // @guarded_mission_mutation('load mission geometry')
        match app.guard.acquire("load mission geometry").await {
            Err(msg) => (false, msg),
            Ok(()) => {
                let (mut success, mut message) = {
                    let App { rec, out, .. } = &mut *app;
                    rec.load_zone_list_service(out)
                };
                if !app.guard.release().await {
                    success = false;
                    message = format!("{}; {RELEASE_UNCONFIRMED}", message.trim());
                    app.out.error(&message);
                }
                (success, message)
            }
        }
    });
    trigger_service!(node, shared, "/get_record_zone_info", |app| {
        app.out.info(&format!("record_zone_list marker ids: {:?}", app.rec.zone_ids()));
        (true, "返回test".to_string())
    });

    // ---- typed services ----------------------------------------------------
    {
        let mut stream = node.create_service::<EditZone::Service>("/edit_zone", QosProfile::services_default())?;
        let shared = shared.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let mut app = shared.lock().await;
                let mut resp = EditZone::Response::default();
                match app.guard.acquire("edit mission geometry").await {
                    Err(msg) => {
                        resp.success = false;
                        resp.message = msg;
                    }
                    Ok(()) => {
                        let pts: Vec<(f64, f64)> = req.message.points.iter().map(|p| (p.x, p.y)).collect();
                        let result = {
                            let App { rec, out, .. } = &mut *app;
                            rec.edit(out, &req.message.op, &req.message.kind, req.message.id, &pts)
                        };
                        match result {
                            Ok((message, id)) => {
                                resp.success = true;
                                resp.message = message;
                                resp.id = id;
                            }
                            Err(message) => {
                                resp.success = false;
                                resp.message = message;
                            }
                        }
                        if !app.guard.release().await {
                            resp.success = false;
                            resp.message = format!("{}; {RELEASE_UNCONFIRMED}", resp.message.trim());
                            app.out.error(&resp.message);
                        }
                    }
                }
                let _ = req.respond(resp);
            }
        });
    }
    {
        let mut stream = node.create_service::<GetZoneList::Service>("/get_record_zone_list", QosProfile::services_default())?;
        let shared = shared.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let app = shared.lock().await;
                let _ = req.respond(GetZoneList::Response {
                    success: true,
                    message: format!("成功獲取普通區域列表，共 {} 個區域", app.rec.zone_list.markers.len()),
                    zone_list: app.rec.zone_list.clone(),
                });
            }
        });
    }
    {
        let mut stream = node.create_service::<GetZoneList::Service>("/get_risk_zone_list", QosProfile::services_default())?;
        let shared = shared.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let app = shared.lock().await;
                let _ = req.respond(GetZoneList::Response {
                    success: true,
                    message: format!("成功獲取風險區域列表，共 {} 個風險區域", app.rec.risk_list.markers.len()),
                    zone_list: app.rec.risk_list.clone(),
                });
            }
        });
    }
    {
        let mut stream = node.create_service::<ChannelPathList::Service>("/get_channel_path_list", QosProfile::services_default())?;
        let shared = shared.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let app = shared.lock().await;
                let _ = req.respond(ChannelPathList::Response { success: true, message: "成功獲取 channel 路徑列表".into(), channel_path_array: app.rec.channel_list.clone() });
            }
        });
    }
    {
        let mut stream = node.create_service::<ChennalPathList::Service>("/get_chennal_path_list", QosProfile::services_default())?;
        let shared = shared.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let app = shared.lock().await;
                let _ = req.respond(ChennalPathList::Response { success: true, message: "成功獲取 chennal 路徑列表".into(), chennal_path_array: app.rec.channel_list.clone() });
            }
        });
    }
    {
        let mut stream = node.create_service::<ChannelRoute::Service>("/get_channel_route", QosProfile::services_default())?;
        let shared = shared.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let app = shared.lock().await;
                let resp = match app.rec.channel_route(req.message.zone_from_id, req.message.zone_to_id, req.message.proximity_m, &app.out) {
                    Ok((path, id, message)) => ChannelRoute::Response { success: true, message, channel_path: path, matched_channel_id: id },
                    Err(message) => ChannelRoute::Response { success: false, message, channel_path: PathMsg::default(), matched_channel_id: -1 },
                };
                let _ = req.respond(resp);
            }
        });
    }
    {
        let mut stream = node.create_service::<SiteOp::Service>("/site_op", QosProfile::services_default())?;
        let shared = shared.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let mut app = shared.lock().await;
                let op = req.message.op.as_str();
                let name = site_store::valid_name(&req.message.name);
                let mut resp = SiteOp::Response::default();
                let blocked = if op != "list" && op != "load" { app.rec.blocked_manifest_reply(&format!("執行場地 {op}")) } else { None };
                if let Some(msg) = blocked {
                    app.out.error(&msg);
                    resp.success = false;
                    resp.message = msg;
                    resp.sites_json = app.rec.site_list_json();
                    app.rec.publish_site_list(&app.out);
                    let _ = req.respond(resp);
                    continue;
                }
                let mut lease = false;
                if op != "list" {
                    match app.guard.acquire(&format!("{} a named site", if op.is_empty() { "unknown" } else { op })).await {
                        Ok(()) => lease = true,
                        Err(msg) => {
                            resp.success = false;
                            resp.message = msg;
                            resp.sites_json = app.rec.site_list_json();
                            app.rec.publish_site_list(&app.out);
                            let _ = req.respond(resp);
                            continue;
                        }
                    }
                }
                let (success, message) = {
                    let App { rec, out, .. } = &mut *app;
                    match (op, name.as_deref()) {
                        ("list", _) => (true, "成功取得場地清單".to_string()),
                        (_, None) => (false, format!("場地名稱無效: {:?}", req.message.name)),
                        ("save", Some(n)) => rec.site_save(n, out),
                        ("load", Some(n)) => rec.site_load(n, out),
                        ("delete", Some(n)) => rec.site_delete(n),
                        ("rename", Some(n)) => rec.site_rename(n, site_store::valid_name(&req.message.new_name).as_deref()),
                        (other, _) => (false, format!("未知 op: {other}（要 save/load/delete/rename/list）")),
                    }
                };
                resp.success = success;
                resp.message = message;
                if lease && !app.guard.release().await {
                    resp.success = false;
                    let previous = resp.message.clone();
                    resp.message = format!("{}場地操作可能已完成，但操作鎖釋放未確認，導航仍被禁止；請檢查機器後重啟 path_record_node 與 nav_action_server", if previous.is_empty() { String::new() } else { format!("{previous}；") });
                }
                resp.sites_json = app.rec.site_list_json();
                app.rec.publish_site_list(&app.out);
                if resp.success {
                    app.out.info(&resp.message);
                } else {
                    app.out.error(&resp.message);
                }
                let _ = req.respond(resp);
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

fn uuid_hex() -> String {
    use std::io::Read;
    let mut bytes = [0u8; 16];
    if let Ok(mut f) = std::fs::File::open("/dev/urandom") {
        let _ = f.read_exact(&mut bytes);
    }
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}
