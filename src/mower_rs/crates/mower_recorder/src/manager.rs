//! `recorder_manager` (recorder_manager_node.py), **idle only** (E5-3a):
//! everything the node does while nothing records. Recording (the groups,
//! `run_metadata.yaml`, the parameter dump, the snapshot buffer, the disk
//! guard, MediaMTX's "record: true") comes with E5-3b; until then
//! `/mower_recorder/start` answers `success: false` with [`NOT_YET`], and
//! `RUST_RECORDER` must stay off on a robot that records.
//!
//! - **`/mower_recorder/status`** (R TL 1) and **the rear light**
//!   (`rear_light_topic`, R TL 1, `{"effect": "off", "source": "bag"}`) at the
//!   start of serving and every 2 s, as `_publish_status` sends them. The rear
//!   light reaches DDS through the bridge, where mower_base reads it (R TL 8,
//!   on its r2r node): a source's "recording" holds its overlay 6 s, so the 2 s
//!   refresh is what keeps it on (design D5).
//! - **MediaMTX** (`video_api_url`): `record: false` before anything else, so
//!   that a recorder that died mid-run does not leave the camera recording,
//!   and again on every tick while that has not worked; the failure is logged
//!   once per text.
//! - **`/fix`** (R V 1): the last fix, for `gps_start` (E5-3b).
//! - **The fault** (`fault_topic`, R V 10): `true` would flush the snapshot
//!   buffer; with no run there is none, as the Python logs.
//! - **`stop`** and **`snapshot`** (Trigger) answer as the Python does when
//!   nothing records: `目前沒有在錄製` (after a status), `snapshot 觸發失敗`.
//! - **`autostart`** (production false): 3 s after the start of serving, the
//!   start's answer in the log, an ERROR since it is refused.
//! - **Parameters** as record.launch.py passes them: `robot_id`,
//!   `output_root`, `config_file` and `qos_overrides_path` (mower_recorder's
//!   share), `git_repo_dir` from the launch; `autostart` false,
//!   `params_dump_nodes`, `gps_topic` `/fix`, `fault_topic`, `video_api_url`
//!   `http://127.0.0.1:9997`, `min_free_mb` 2048 from mower_rsd.yaml. The
//!   profile (`config_file`) gives `num_topics` and `profile` in the status.

use std::path::Path;
use std::pin::Pin;
use std::time::Duration;

use mower_bus::{BusError, Publisher, Qos, Service, Sleep, Subscriber, Timer};
use mower_pyfmt::Py;
use mower_recorder_core::profile::{self, ConfigLog};
use mower_recorder_core::run::{self, Fix, Status};
use mower_recorder_core::mediamtx;
use mower_rs_common::{params, BoxError, ModuleCtx, ModuleResult};
use r2r::sensor_msgs::msg::NavSatFix;
use r2r::std_msgs::msg::{Bool, String as Text};
use r2r::std_srvs::srv::Trigger;

use crate::owner::Owner;

/// The start's answer until E5-3b adds recording (shown by the app in a
/// snackbar, so in its language).
pub const NOT_YET: &str =
    "這版的 Rust 錄製器（RUST_RECORDER，E5-3a）還不能錄製，錄製在下一步（E5-3b）加入；要錄製請把 RUST_RECORDER 設回 false";

/// The status tick (`create_timer(2.0, self._publish_status)`).
pub const STATUS_PERIOD: Duration = Duration::from_secs(2);

/// `autostart`'s delay (`create_timer(3.0, self._autostart_once)`).
pub const AUTOSTART_DELAY: Duration = Duration::from_secs(3);

pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let node = r2r::Node::create(ctx, &m.node_name, &m.namespace)?;
    run_node(node, m).await
}

struct Settings {
    cfg: Py,
    autostart: bool,
    video_api: String,
    video_path: String,
    status_topic: String,
    rear_topic: Option<String>,
    gps_topic: String,
    fault_topic: String,
    start_service: String,
    stop_service: String,
    snapshot_service: String,
}

type Trig = Service<Trigger::Request, Trigger::Response>;

/// What the node holds while it serves.
struct Active {
    status: Publisher<Text>,
    rear: Option<Publisher<Text>>,
    fix: Subscriber<NavSatFix>,
    fault: Subscriber<Bool>,
    start: Trig,
    stop: Trig,
    snapshot: Trig,
    tick: Timer,
    autostart: Option<Pin<Box<Sleep>>>,
}

/// The run state (idle in E5-3a) and the video bookkeeping.
struct State {
    video_on: bool,
    video_error: Option<String>,
    last_fix: Option<Fix>,
    stop_reason: Option<String>,
}

pub async fn run_node(node: r2r::Node, m: ModuleCtx) -> ModuleResult {
    let logger = node.logger().to_string();
    let config_file = params::string(&node, "config_file", "");
    let mut logs = Vec::new();
    let cfg = profile::load_config((!config_file.is_empty()).then(|| Path::new(&config_file)), &mut logs);
    for line in logs {
        match line {
            ConfigLog::Warn(t) => r2r::log_warn!(&logger, "{t}"),
            ConfigLog::Error(t) => r2r::log_error!(&logger, "{t}"),
        }
    }
    let rear = params::string(&node, "rear_light_topic", "/mower_base/rear_light");
    let s = Settings {
        cfg,
        autostart: params::bool(&node, "autostart", true),
        video_api: params::string(&node, "video_api_url", ""),
        video_path: params::string(&node, "video_path", "front"),
        status_topic: m.resolve_topic("/mower_recorder/status")?,
        rear_topic: if rear.is_empty() { None } else { Some(m.resolve_topic(&rear)?) },
        gps_topic: m.resolve_topic(&params::string(&node, "gps_topic", "/fix"))?,
        fault_topic: m.resolve_topic(&params::string(&node, "fault_topic", "/mower_recorder/fault"))?,
        start_service: m.resolve_service("/mower_recorder/start")?,
        stop_service: m.resolve_service("/mower_recorder/stop")?,
        snapshot_service: m.resolve_service("/mower_recorder/snapshot")?,
    };
    let mut owner = Owner::new(&m, &logger, "the status, the rear light and /mower_recorder/start|stop|snapshot")?;
    let mut active: Option<Active> = None;
    let mut state = State { video_on: false, video_error: None, last_fix: None, stop_reason: None };
    enum Next {
        Stop,
        Look,
        Tick(Result<(), BusError>),
        Autostart,
        Fix(Option<std::sync::Arc<NavSatFix>>),
        Fault(Option<std::sync::Arc<Bool>>),
        Start(Option<mower_bus::Request<Trigger::Request, Trigger::Response>>),
        StopCall(Option<mower_bus::Request<Trigger::Request, Trigger::Response>>),
        Snapshot(Option<mower_bus::Request<Trigger::Request, Trigger::Response>>),
    }
    loop {
        let next = match active.as_mut() {
            None => tokio::select! {
                biased;
                () = m.shutdown.wait() => Next::Stop,
                () = owner.tick() => Next::Look,
            },
            Some(a) => tokio::select! {
                biased;
                () = m.shutdown.wait() => Next::Stop,
                () = owner.tick() => Next::Look,
                r = a.start.recv() => Next::Start(r),
                r = a.stop.recv() => Next::StopCall(r),
                r = a.snapshot.recv() => Next::Snapshot(r),
                msg = a.fault.recv() => Next::Fault(msg),
                msg = a.fix.recv() => Next::Fix(msg),
                tick = a.tick.tick() => Next::Tick(tick.map(|_| ())),
                () = async {
                    match a.autostart.as_mut() {
                        Some(sleep) => sleep.as_mut().await,
                        None => std::future::pending().await,
                    }
                } => Next::Autostart,
            },
        };
        match next {
            Next::Stop => break,
            Next::Look => {
                if !owner.may_serve(&node) {
                    active = None;
                } else if active.is_none() {
                    active = Some(start(&m, &s, &mut state, &logger).await?);
                }
            }
            // a closed scope: the module is being stopped
            Next::Tick(Err(_)) | Next::Fix(None) | Next::Fault(None) | Next::Start(None) | Next::StopCall(None)
            | Next::Snapshot(None) => break,
            Next::Tick(Ok(())) => {
                if let Some(a) = &active {
                    publish_status(a, &s, &mut state, &logger).await?;
                }
            }
            Next::Autostart => {
                if let Some(a) = active.as_mut() {
                    a.autostart = None;
                    let (ok, message) = start_recording(&logger);
                    if ok {
                        r2r::log_info!(&logger, "autostart: {message}");
                    } else {
                        r2r::log_error!(&logger, "autostart: {message}");
                    }
                }
            }
            Next::Fix(Some(fix)) => {
                state.last_fix = Some(Fix { latitude: fix.latitude, longitude: fix.longitude, altitude: fix.altitude });
            }
            Next::Fault(Some(msg)) => {
                if msg.data {
                    r2r::log_warn!(&logger, "fault received -> flushing snapshot buffer");
                    trigger_snapshot(&logger);
                }
            }
            Next::Start(Some(req)) => {
                let (success, message) = start_recording(&logger);
                req.respond(Trigger::Response { success, message })?;
            }
            Next::StopCall(Some(req)) => {
                if let Some(a) = &active {
                    // _stop_recording_locked with no recorder running
                    if state.video_on {
                        set_video_off(&s, &mut state, &logger).await;
                    }
                    publish_status(a, &s, &mut state, &logger).await?;
                }
                let (success, message) = run::stop_message(&[], "");
                req.respond(Trigger::Response { success, message })?;
            }
            Next::Snapshot(Some(req)) => {
                let ok = trigger_snapshot(&logger);
                req.respond(Trigger::Response { success: ok, message: run::snapshot_message(ok).to_string() })?;
            }
        }
    }
    drop(active);
    drop(node);
    Ok(())
}

/// The start of serving, in `__init__`'s order: MediaMTX off, the readers,
/// the services, the writers, the tick, the first status.
async fn start(m: &ModuleCtx, s: &Settings, state: &mut State, logger: &str) -> Result<Active, BoxError> {
    if !s.video_api.is_empty() {
        // a recorder that died mid-run must not leave MediaMTX recording
        state.video_on = true;
        set_video_off(s, state, logger).await;
    }
    let fix = m.bus.subscribe::<NavSatFix>(&s.gps_topic, Qos::default().keep_last(1))?;
    let fault = m.bus.subscribe::<Bool>(&s.fault_topic, Qos::default())?;
    let start = m.bus.service::<Trigger::Request, Trigger::Response>(&s.start_service)?;
    let stop = m.bus.service::<Trigger::Request, Trigger::Response>(&s.stop_service)?;
    let snapshot = m.bus.service::<Trigger::Request, Trigger::Response>(&s.snapshot_service)?;
    let status = m.bus.publisher::<Text>(&s.status_topic, Qos::latched())?;
    let rear = match &s.rear_topic {
        Some(t) => Some(m.bus.publisher::<Text>(t, Qos::latched())?),
        None => None,
    };
    let tick = m.bus.wall_timer(&m.clock, STATUS_PERIOD)?;
    let autostart = s.autostart.then(|| Box::pin(m.clock.sleep(AUTOSTART_DELAY)));
    let active = Active { status, rear, fix, fault, start, stop, snapshot, tick, autostart };
    publish_status(&active, s, state, logger).await?;
    Ok(active)
}

/// `_publish_status` with nothing recording: the off-PATCH again if an
/// earlier one failed, then the status and the rear light.
async fn publish_status(a: &Active, s: &Settings, state: &mut State, logger: &str) -> Result<(), BusError> {
    if state.video_on {
        set_video_off(s, state, logger).await;
    }
    let st = Status::idle(&s.cfg, state.stop_reason.clone());
    a.status.publish_owned(Text { data: st.to_json() })?;
    if let Some(rear) = &a.rear {
        rear.publish_owned(Text { data: run::rear_light_json(false) })?;
    }
    Ok(())
}

/// `_set_video(False)`: the PATCH (3 s timeout, off the runtime); a failure
/// is logged when its text changes, and retried by the next tick.
async fn set_video_off(s: &Settings, state: &mut State, logger: &str) {
    if !state.video_on {
        return;
    }
    let (api, path) = (s.video_api.clone(), s.video_path.clone());
    let sent = tokio::task::spawn_blocking(move || mediamtx::patch_path(&api, &path, &mediamtx::stop_patch())).await;
    let (ok, message) = sent.unwrap_or_else(|e| (false, format!("the PATCH did not run: {e}")));
    if ok {
        state.video_on = false;
        state.video_error = None;
        r2r::log_info!(logger, "video off: {}", s.video_path);
    } else if state.video_error.as_deref() != Some(message.as_str()) {
        r2r::log_warn!(logger, "video off failed ({}): {message}", s.video_path);
        state.video_error = Some(message);
    }
}

/// `_start_recording`, E5-3a: refused, recording is E5-3b's.
fn start_recording(logger: &str) -> (bool, String) {
    r2r::log_warn!(
        logger,
        "start refused: the Rust recorder records from E5-3b on (RUST_RECORDER=false records with the Python recorder)"
    );
    (false, NOT_YET.to_string())
}

/// `_trigger_snapshot` with no snapshot group running.
fn trigger_snapshot(logger: &str) -> bool {
    r2r::log_warn!(logger, "no snapshot recorder running");
    false
}
