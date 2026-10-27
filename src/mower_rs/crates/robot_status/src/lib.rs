//! robot_status: one process for the three status publishers that used to be
//! separate rclpy nodes (heartbeat_node, robot_info_node, telemetry_node).
//!
//! * `/robot/online` (std_msgs/Bool, latched, 2 Hz): true while the liveness
//!   source topic (default `/odom`) has been received within
//!   `heartbeat_stale_timeout_s`. If this process dies the topic stops, so a
//!   consumer's own receive timeout also flags the robot offline.
//! * `/robot/info` (std_msgs/String JSON, latched, 1 Hz and on firmware
//!   change) plus `<state_dir>/robot_status.json`, the update lights on
//!   `/mower_base/led_command` and the `/system/update` / `/system/restart` /
//!   `/system/check_update`
//!   Trigger services (host.request hand-off, refused while moving).
//! * `/robot/telemetry` (std_msgs/String JSON, reliable, 10 Hz): the
//!   dashboard feed built from `/fix`, `/imu/data`, odometry, the base
//!   telemetry, the battery topics, `link_status.json` and the host's CPU,
//!   memory, disks and busiest processes from /proc (`host.rs`).
//!
//! rclpy needed 5-12 ms of CPU per delivered message on the LubanCat; this
//! process handles the same ~45 events/s for well under 1 % of a core.
//! Timestamps use the monotonic and wall clocks (the Python nodes ran with
//! use_sim_time:=false for the same reason).
//!
//! The topics, services and timers are on the in-process bus
//! (`ModuleCtx::bus`, step E2 of docs/ROS_FREE_PLAN.md); the bus/DDS bridge
//! carries what crosses the process boundary. The r2r node stays for the
//! parameters, the /rosout logs and the ROS graph.

mod host;
mod info;
mod state;
mod telemetry;

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use futures::StreamExt;
use mower_bus::Qos;
use mower_rs_common::{hostname, params, read_json_file, state_dir as default_state_dir, write_host_request, ModuleCtx, ModuleResult};
use r2r::nav_msgs::msg::Odometry;
use r2r::sensor_msgs::msg::{BatteryState, Imu, NavSatFix};
use r2r::std_msgs::msg::{Bool, String as StringMsg};
use r2r::std_srvs::srv::Trigger;
use serde_json::Value;
use tokio::sync::Notify;

use crate::info::InfoConfig;
use crate::state::State;

type Shared = Arc<Mutex<State>>;

struct Config {
    state_dir: String,
    heartbeat_source_topic: String,
    heartbeat_stale_timeout_s: f64,
    heartbeat_publish_rate_hz: f64,
    info_publish_rate_hz: f64,
    firmware_info_topic: String,
    nav_active_topic: String,
    odom_topic: String,
    moving_speed_threshold: f64,
    led_topic: String,
    telemetry_publish_rate_hz: f64,
    gps_fix_topic: String,
    gps_filtered_topic: String,
    imu_topic: String,
    base_telemetry_topic: String,
    battery_topic: String,
    aon_battery_topic: String,
}

fn rgb(node: &r2r::Node, name: &str, default: [i64; 3]) -> [i64; 3] {
    let v = params::i64_array(node, name, &default);
    if v.len() == 3 {
        [v[0], v[1], v[2]]
    } else {
        r2r::log_warn!(node.logger(), "parameter {name}: expected 3 integers, using default");
        default
    }
}

fn load_config(node: &r2r::Node) -> (Config, InfoConfig) {
    let state_dir = params::string(node, "state_dir", &default_state_dir());
    let identity = info::load_identity(&state_dir);
    let id_field = |k: &str| identity.as_ref().and_then(|i| i.get(k)).and_then(|v| v.as_str()).map(str::to_string);
    let env_robot_id = std::env::var("MOWER_ROBOT_ID").ok().filter(|s| !s.is_empty());
    let robot_id_default = id_field("robot_id").or(env_robot_id).unwrap_or_else(hostname);
    let robot_name_default = id_field("name").unwrap_or_else(hostname);
    let firmware_manifest_default = std::env::var("MOWER_FIRMWARE_MANIFEST")
        .ok()
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| "/opt/mower/firmware/mower_robot_firmware.json".to_string());
    let firmware_manifest = params::string(node, "firmware_manifest", &firmware_manifest_default);
    let cfg = Config {
        state_dir: state_dir.clone(),
        heartbeat_source_topic: params::string(node, "heartbeat_source_topic", "/odom"),
        heartbeat_stale_timeout_s: params::f64(node, "heartbeat_stale_timeout_s", 2.0),
        heartbeat_publish_rate_hz: params::f64(node, "heartbeat_publish_rate_hz", 2.0).max(0.1),
        info_publish_rate_hz: params::f64(node, "info_publish_rate_hz", 1.0).max(0.1),
        firmware_info_topic: params::string(node, "firmware_info_topic", "/mower_base/firmware_info"),
        nav_active_topic: params::string(node, "nav_active_topic", "/nav_operation_active"),
        odom_topic: params::string(node, "odom_topic", "/odom"),
        moving_speed_threshold: params::f64(node, "moving_speed_threshold", 0.02),
        led_topic: params::string(node, "led_topic", "/mower_base/led_command"),
        telemetry_publish_rate_hz: params::f64(node, "telemetry_publish_rate_hz", 10.0).max(0.5),
        gps_fix_topic: params::string(node, "gps_fix_topic", "/fix"),
        gps_filtered_topic: params::string(node, "gps_filtered_topic", "/gps/filtered"),
        imu_topic: params::string(node, "imu_topic", "/imu/data"),
        base_telemetry_topic: params::string(node, "base_telemetry_topic", "/mower_base/telemetry"),
        battery_topic: params::string(node, "battery_topic", "/battery_state"),
        aon_battery_topic: params::string(node, "aon_battery_topic", "/aon_battery_state"),
    };
    let info = InfoConfig {
        robot_id: params::string(node, "robot_id", &robot_id_default),
        robot_name: params::string(node, "robot_name", &robot_name_default),
        paired: identity.is_some(),
        state_dir,
        software: info::software_identity(),
        bundled_firmware: read_json_file(&firmware_manifest),
        busy_timeout_s: params::f64(node, "busy_timeout_s", 2.0),
        led_update_rgb: rgb(node, "led_update_rgb", [255, 180, 0]),
        led_update_period_ms: params::i64(node, "led_update_period_ms", 1600),
        led_normal_rgb: rgb(node, "led_normal_rgb", [110, 110, 110]),
    };
    (cfg, info)
}

fn latched() -> Qos {
    Qos::default().keep_last(1).reliable().transient_local()
}

fn period(rate_hz: f64) -> Duration {
    Duration::from_secs_f64(1.0 / rate_hz)
}

fn json_string(v: &Value) -> StringMsg {
    StringMsg { data: serde_json::to_string(v).expect("json") }
}

/// The `robot_status` node. Runs either as the `robot_status` binary or as one
/// module of `mower_rsd`; the node name, namespace, parameters and every
/// topic / service / action name are the same either way.
pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let node = r2r::Node::create(ctx, &m.node_name, &m.namespace)?;
    let logger = node.logger().to_string();
    let (cfg, info_cfg) = load_config(&node);
    let info_cfg = Arc::new(info_cfg);
    let shared: Shared = Arc::new(Mutex::new(State::default()));
    let start = Instant::now();

    // ---- publishers --------------------------------------------------------
    // Every bus name below is what rcl makes of it on the node, the absolute
    // ones too (ModuleCtx::resolve_topic / resolve_service).
    let pub_online = m.bus.publisher::<Bool>(&m.resolve_topic("/robot/online")?, latched())?;
    let pub_info = m.bus.publisher::<StringMsg>(&m.resolve_topic("/robot/info")?, latched())?;
    // reliable (not best-effort): rosbridge picks its subscriber QoS from the
    // publishers it has discovered, and a best-effort publisher that shows up
    // late leaves the app on a reliable subscription with no data.
    let pub_telemetry = m.bus.publisher::<StringMsg>(&m.resolve_topic("/robot/telemetry")?, Qos::default())?;
    let pub_led = if cfg.led_topic.is_empty() {
        None
    } else {
        Some(m.bus.publisher::<StringMsg>(&m.resolve_topic(&cfg.led_topic)?, latched())?)
    };

    // ---- subscriptions -----------------------------------------------------
    let hb_shares_odom = cfg.heartbeat_source_topic == cfg.odom_topic;
    let mut sub_heartbeat = if hb_shares_odom {
        None
    } else {
        Some(m.bus.subscribe::<Odometry>(&m.resolve_topic(&cfg.heartbeat_source_topic)?, Qos::default())?)
    };
    let mut sub_odom = m.bus.subscribe::<Odometry>(&m.resolve_topic(&cfg.odom_topic)?, Qos::default())?;
    let mut sub_fix = m.bus.subscribe::<NavSatFix>(&m.resolve_topic(&cfg.gps_fix_topic)?, Qos::sensor_data())?;
    let mut sub_filtered = m.bus.subscribe::<NavSatFix>(&m.resolve_topic(&cfg.gps_filtered_topic)?, Qos::sensor_data())?;
    let mut sub_imu = m.bus.subscribe::<Imu>(&m.resolve_topic(&cfg.imu_topic)?, Qos::sensor_data())?;
    let base_qos = Qos::default().keep_last(1).best_effort();
    let mut sub_base = m.bus.subscribe::<StringMsg>(&m.resolve_topic(&cfg.base_telemetry_topic)?, base_qos)?;
    let mut sub_battery = m.bus.subscribe::<BatteryState>(&m.resolve_topic(&cfg.battery_topic)?, Qos::default())?;
    let mut sub_aon = m.bus.subscribe::<BatteryState>(&m.resolve_topic(&cfg.aon_battery_topic)?, Qos::default())?;
    let mut sub_firmware = if cfg.firmware_info_topic.is_empty() {
        None
    } else {
        Some(m.bus.subscribe::<StringMsg>(&m.resolve_topic(&cfg.firmware_info_topic)?, latched())?)
    };
    let mut sub_nav_active = m.bus.subscribe::<Bool>(&m.resolve_topic(&cfg.nav_active_topic)?, latched())?;

    // ---- services ----------------------------------------------------------
    let mut srv_update = m.bus.service::<Trigger::Request, Trigger::Response>(&m.resolve_service("/system/update")?)?;
    let mut srv_restart = m.bus.service::<Trigger::Request, Trigger::Response>(&m.resolve_service("/system/restart")?)?;
    let mut srv_check = m.bus.service::<Trigger::Request, Trigger::Response>(&m.resolve_service("/system/check_update")?)?;

    // ---- timers ------------------------------------------------------------
    // rcl's wall timers on the module's clock: the first tick one period
    // after creation, overruns skipped, and no ticks once the scope closes.
    let mut timer_heartbeat = m.bus.wall_timer(&m.clock, period(cfg.heartbeat_publish_rate_hz))?;
    let mut timer_info = m.bus.wall_timer(&m.clock, period(cfg.info_publish_rate_hz))?;
    let mut timer_telemetry = m.bus.wall_timer(&m.clock, period(cfg.telemetry_publish_rate_hz))?;
    let info_now = Arc::new(Notify::new());

    r2r::log_info!(
        &logger,
        "robot_status: id={} software={} api={} state_dir={} bundled_firmware={} heartbeat={}@{}Hz(stale {}s) telemetry@{}Hz",
        info_cfg.robot_id,
        info_cfg.software.get("version").and_then(|v| v.as_str()).unwrap_or("dev"),
        info::ROBOT_API_VERSION,
        cfg.state_dir,
        info_cfg
            .bundled_firmware
            .as_ref()
            .and_then(|b| b.get("version"))
            .and_then(|v| v.as_str())
            .unwrap_or("none"),
        cfg.heartbeat_source_topic,
        cfg.heartbeat_publish_rate_hz,
        cfg.heartbeat_stale_timeout_s,
        cfg.telemetry_publish_rate_hz,
    );

    // ---- input tasks -------------------------------------------------------
    macro_rules! store_task {
        ($stream:expr, $field:ident) => {{
            let shared = shared.clone();
            tokio::spawn(async move {
                while let Some(msg) = $stream.next().await {
                    shared.lock().unwrap().$field.set(msg, Instant::now());
                }
            });
        }};
    }
    store_task!(sub_fix, fix);
    store_task!(sub_filtered, filtered);
    store_task!(sub_imu, imu);
    store_task!(sub_battery, battery);
    store_task!(sub_aon, aon_battery);

    if let Some(mut stream) = sub_heartbeat.take() {
        let shared = shared.clone();
        tokio::spawn(async move {
            while stream.next().await.is_some() {
                shared.lock().unwrap().heartbeat_last = Some(Instant::now());
            }
        });
    }
    {
        let shared = shared.clone();
        let threshold = cfg.moving_speed_threshold;
        tokio::spawn(async move {
            while let Some(msg) = sub_odom.next().await {
                let now = Instant::now();
                let v = &msg.twist.twist;
                let moving = v.linear.x.abs() > threshold || v.angular.z.abs() > threshold;
                let mut st = shared.lock().unwrap();
                if moving {
                    st.last_moving = Some(now);
                }
                if hb_shares_odom {
                    st.heartbeat_last = Some(now);
                }
                st.odom.set(msg, now);
            }
        });
    }
    {
        let shared = shared.clone();
        let logger = logger.clone();
        tokio::spawn(async move {
            let mut last_warn: Option<Instant> = None;
            while let Some(msg) = sub_base.next().await {
                match serde_json::from_str::<Value>(&msg.data) {
                    Ok(doc) => shared.lock().unwrap().base.set(doc, Instant::now()),
                    Err(_) => {
                        let now = Instant::now();
                        if last_warn.map(|t| now.duration_since(t).as_secs() >= 10).unwrap_or(true) {
                            last_warn = Some(now);
                            r2r::log_warn!(&logger, "bad JSON on base telemetry topic");
                        }
                    }
                }
            }
        });
    }
    if let Some(mut stream) = sub_firmware.take() {
        let shared = shared.clone();
        let logger = logger.clone();
        let info_now = info_now.clone();
        tokio::spawn(async move {
            while let Some(msg) = stream.next().await {
                let Ok(doc) = serde_json::from_str::<Value>(&msg.data) else {
                    r2r::log_warn!(&logger, "ignoring malformed firmware_info payload");
                    continue;
                };
                let changed = {
                    let mut st = shared.lock().unwrap();
                    if st.firmware_running.as_ref() != Some(&doc) {
                        st.firmware_running = Some(doc.clone());
                        true
                    } else {
                        false
                    }
                };
                if changed {
                    r2r::log_info!(
                        &logger,
                        "STM32 firmware {}+{}",
                        doc.get("version").and_then(|v| v.as_str()).unwrap_or("?"),
                        doc.get("git_sha").and_then(|v| v.as_str()).unwrap_or("?")
                    );
                    info_now.notify_one();
                }
            }
        });
    }
    {
        let shared = shared.clone();
        tokio::spawn(async move {
            while let Some(msg) = sub_nav_active.next().await {
                shared.lock().unwrap().nav_running = msg.data;
            }
        });
    }

    // ---- host request services --------------------------------------------
    macro_rules! host_service {
        ($stream:expr, $action:literal) => {
            host_service!($stream, $action, true)
        };
        ($stream:expr, $action:literal, $check_busy:expr) => {{
            let shared = shared.clone();
            let info_cfg = info_cfg.clone();
            let logger = logger.clone();
            tokio::spawn(async move {
                while let Some(req) = $stream.next().await {
                    let busy = $check_busy && info::busy(&shared.lock().unwrap(), info_cfg.busy_timeout_s, Instant::now());
                    let resp = if busy {
                        Trigger::Response {
                            success: false,
                            message: "robot is busy (moving or navigating); stop it first".into(),
                        }
                    } else {
                        match write_host_request(&info_cfg.state_dir, $action, "app") {
                            Ok(path) => {
                                r2r::log_warn!(&logger, "host action {:?} requested via {}", $action, path);
                                Trigger::Response {
                                    success: true,
                                    message: format!("{} requested; watch /robot/info update.state", $action),
                                }
                            }
                            Err(e) => {
                                let message = format!("could not write host request: {e}");
                                r2r::log_error!(&logger, "{}", message);
                                Trigger::Response { success: false, message }
                            }
                        }
                    };
                    if let Err(e) = req.respond(resp) {
                        r2r::log_error!(&logger, "could not answer /system/{}: {:?}", $action, e);
                    }
                }
            });
        }};
    }
    host_service!(srv_update, "update");
    host_service!(srv_restart, "restart");
    // A registry lookup only: allowed while the robot is moving.
    host_service!(srv_check, "check", false);

    // ---- heartbeat ---------------------------------------------------------
    {
        let shared = shared.clone();
        let logger = logger.clone();
        let stale = cfg.heartbeat_stale_timeout_s;
        tokio::spawn(async move {
            let mut last_published: Option<bool> = None;
            while timer_heartbeat.tick().await.is_ok() {
                let online = shared
                    .lock()
                    .unwrap()
                    .heartbeat_last
                    .map(|t| Instant::now().duration_since(t).as_secs_f64() <= stale)
                    .unwrap_or(false);
                if let Err(e) = pub_online.publish(&Bool { data: online }) {
                    r2r::log_error!(&logger, "publish /robot/online failed: {:?}", e);
                    break;
                }
                if last_published != Some(online) {
                    last_published = Some(online);
                    r2r::log_info!(&logger, "/robot/online -> {}", online);
                }
            }
        });
    }

    // ---- /robot/info -------------------------------------------------------
    {
        let shared = shared.clone();
        let info_cfg = info_cfg.clone();
        let logger = logger.clone();
        let info_now = info_now.clone();
        tokio::spawn(async move {
            let mut status_warned: Option<Instant> = None;
            let mut first = true;
            loop {
                if !first {
                    tokio::select! {
                        t = timer_info.tick() => { if t.is_err() { break; } }
                        _ = info_now.notified() => {}
                    }
                }
                first = false;
                let now = Instant::now();
                let (snap, led) = {
                    let mut st = shared.lock().unwrap();
                    let snap = info::snapshot(&info_cfg, &st, start, now);
                    let update = snap.get("update").cloned().unwrap_or(Value::Null);
                    let led = if pub_led.is_some() { info::led_request(&info_cfg, &mut st, &update) } else { None };
                    st.info = Some(snap.clone());
                    (snap, led)
                };
                if let Err(e) = pub_info.publish(&json_string(&snap)) {
                    r2r::log_error!(&logger, "publish /robot/info failed: {:?}", e);
                    break;
                }
                if let (Some(pub_led), Some((updating, req))) = (&pub_led, led) {
                    let _ = pub_led.publish(&json_string(&req));
                    r2r::log_info!(&logger, "lights: {}", if updating { "update orbit" } else { "normal" });
                }
                let st = shared.lock().unwrap();
                if let Err(e) = info::write_robot_status(&info_cfg, &st, &snap, now) {
                    if status_warned.map(|t| now.duration_since(t).as_secs() >= 60).unwrap_or(true) {
                        status_warned = Some(now);
                        r2r::log_warn!(&logger, "cannot write robot_status.json: {}", e);
                    }
                }
            }
        });
    }

    // ---- /robot/telemetry --------------------------------------------------
    {
        let shared = shared.clone();
        let logger = logger.clone();
        let robot_id = info_cfg.robot_id.clone();
        let state_dir = cfg.state_dir.clone();
        tokio::spawn(async move {
            let mut seq: u64 = 0;
            let mut host = host::Sampler::new(&state_dir);
            while timer_telemetry.tick().await.is_ok() {
                let now = Instant::now();
                seq += 1;
                // The /proc scan (~2 ms every two seconds) runs outside the lock.
                let host_block = host.poll(now);
                let doc = {
                    let mut st = shared.lock().unwrap();
                    telemetry::poll_link(&mut st, &state_dir, now);
                    if let Some(block) = host_block {
                        st.host = block;
                    }
                    telemetry::build_document(&st, &robot_id, seq, start, now)
                };
                if let Err(e) = pub_telemetry.publish(&json_string(&doc)) {
                    r2r::log_error!(&logger, "publish /robot/telemetry failed: {:?}", e);
                    break;
                }
            }
        });
    }

    // ---- run until SIGINT / SIGTERM ----------------------------------------
    // The bus needs no spinning, and the node has nothing to serve: it only
    // has to live as long as the module does.
    m.shutdown.wait().await;
    r2r::log_info!(&logger, "robot_status: shutting down");
    drop(node);
    Ok(())
}
