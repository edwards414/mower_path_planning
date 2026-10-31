//! `recorder_manager` (recorder_manager_node.py), E5-3b: the node around
//! `mower_recorder_core::manager::Manager`, which does everything the Python
//! node did (start, stop, snapshot, the 2 s tick with the disk guard, the
//! status and the rear light, the video, the run metadata and the parameter
//! dump) in its order and with its texts.
//!
//! What the node provides (`manager::Hooks`):
//!
//! - **Sources:** [`crate::sources::DdsSources`], DDS raw subscriptions
//!   (vendor/r2r patch 3) found as rosbag2 finds its names, with a node per
//!   group (`mower_full_recorder`, `mower_snapshot_recorder`) while it runs
//!   (design D12).
//! - **The parameter dump:** [`RosParams`], `ListParameters` and
//!   `GetParameters` of each dumped node through clients on this node, as
//!   `ros2 param dump` asked them, on the dump's own thread.
//! - **MediaMTX** at `video_api_url` (`manager::MediaMtx`), the local clock,
//!   `statvfs`.
//! - **Outputs:** `/mower_recorder/status` and the rear light (R TL 1) on the
//!   bus, the log on the r2r node.
//!
//! The node's endpoints, on the bus: the status and the rear light
//! (`rear_light_topic`), `/fix` (R V 1) and the fault (R V 10) read, and
//! `/mower_recorder/start|stop|snapshot` (Trigger) served. The manager's
//! calls block (MediaMTX's 3 s, a finalize), so each runs on a blocking
//! thread, one at a time, in the order the events came: the Python's
//! start/stop lock. A run still going when the node stops, or steps back for
//! a foreign recorder ([`crate::owner`]), is stopped and its bags finalized,
//! as `destroy_node` did.
//!
//! - **Parameters** as record.launch.py passes them: `robot_id`,
//!   `output_root`, `config_file` and `qos_overrides_path` (mower_recorder's
//!   share), `git_repo_dir` from the launch; `autostart` false,
//!   `params_dump_nodes`, `gps_topic` `/fix`, `fault_topic`, `video_api_url`
//!   `http://127.0.0.1:9997`, `min_free_mb` 2048 from mower_rsd.yaml. The
//!   data_collection parameters (`session_file`, `camera_params`, ...) are
//!   read as the Python's were, for a node started with that profile.
//! - **data_collection (E5-5, design 4.7 and D7):** the request file
//!   (`profile_request_file`, default `$MOWER_STATE_DIR/recorder_profile.json`,
//!   else `~/.mower/recorder_profile.json`), which mower-data-collection.sh
//!   writes at `start` and deletes at `stop`. It is looked at when the node
//!   serves and on every tick, and applied while no run records: the
//!   manager is made again with data_collection.yaml (next to `config_file`),
//!   the EKF and navsat dumps, and the camera parameters
//!   `mower_recorder_core::dc_request::resolve` works out of it as
//!   data_collection.launch.py did (the URDF from robot_state_publisher's
//!   latched `/robot_description`); the status then says
//!   `"profile": "data_collection"`. Its deletion brings the default back.
//!   The camera itself is gscam, in the camera-only compose.

use std::collections::HashMap;
use std::path::PathBuf;
use std::pin::Pin;
use std::sync::{Arc, Mutex, MutexGuard};
use std::task::{Context, Poll};
use std::time::{Duration, Instant};

use mower_bus::{BusError, Publisher, Qos, Service, Sleep, Subscriber, Timer};
use mower_recorder_core::manager::{self as core, Config, DcParams, Hooks, Manager, MediaMtx, Outputs, ParamSource};
use mower_recorder_core::params::ParamValue;
use mower_recorder_core::recording::Level;
use mower_recorder_core::run::Fix;
use mower_rs_common::{params, BoxError, ModuleCtx, ModuleResult};
use r2r::rcl_interfaces::srv::{GetParameters, ListParameters};
use r2r::sensor_msgs::msg::NavSatFix;
use r2r::std_msgs::msg::{Bool, String as Text};
use r2r::std_srvs::srv::Trigger;
use r2r::QosProfile;

use crate::owner::Owner;

/// The status tick (`create_timer(2.0, self._publish_status)`).
pub const STATUS_PERIOD: Duration = Duration::from_secs(2);

/// `autostart`'s delay (`create_timer(3.0, self._autostart_once)`).
pub const AUTOSTART_DELAY: Duration = Duration::from_secs(3);

/// How long the parameter dump waits for one service or answer.
pub const PARAM_TIMEOUT: Duration = Duration::from_secs(5);

pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let node = r2r::Node::create(ctx.clone(), &m.node_name, &m.namespace)?;
    run_node(ctx, node, m).await
}

struct Settings {
    cfg: Config,
    /// E5-5: the request file (`mower_recorder_core::dc_request`)
    request_file: PathBuf,
    /// mower_recorder's `config/` (data_collection.yaml, the extrinsics
    /// placeholder, the session template): `config_file`'s directory
    share_config: Option<PathBuf>,
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
type Call = mower_bus::Request<Trigger::Request, Trigger::Response>;

/// What the node holds while it serves.
struct Active {
    manager: Arc<Mutex<Manager>>,
    /// what makes a manager again (E5-5: another profile)
    out: Arc<BusOutputs>,
    params: Arc<RosParams>,
    /// the request file's text in force (`None`: the default profile), and
    /// the last one refused (said once)
    applied: Option<String>,
    refused: Option<String>,
    fix: Subscriber<NavSatFix>,
    fault: Subscriber<Bool>,
    start: Trig,
    stop: Trig,
    snapshot: Trig,
    tick: Timer,
    autostart: Option<Pin<Box<Sleep>>>,
}

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

/// Runs `f` on the manager, off the runtime (its calls block).
async fn with_manager<R: Send + 'static>(manager: &Arc<Mutex<Manager>>, f: impl FnOnce(&mut Manager) -> R + Send + 'static) -> R {
    let manager = manager.clone();
    tokio::task::spawn_blocking(move || f(&mut lock(&manager))).await.expect("a manager call panicked")
}

pub async fn run_node(ctx: r2r::Context, node: r2r::Node, m: ModuleCtx) -> ModuleResult {
    let logger = node.logger().to_string();
    let rear = params::string(&node, "rear_light_topic", "/mower_base/rear_light");
    let mut cfg = Config::new();
    cfg.robot_id = params::string(&node, "robot_id", "mower");
    cfg.output_root = PathBuf::from(crate::expanduser(&params::string(&node, "output_root", "~/mower_bags")));
    cfg.config_file = params::string(&node, "config_file", "");
    cfg.qos_overrides_path = params::string(&node, "qos_overrides_path", "");
    cfg.git_repo_dir = crate::expanduser(&params::string(&node, "git_repo_dir", ""));
    cfg.params_dump_nodes =
        crate::params::strings(&node, "params_dump_nodes", &[""]).into_iter().filter(|n| !n.is_empty()).collect();
    cfg.rear_light = !rear.is_empty();
    cfg.min_free_mb = params::i64(&node, "min_free_mb", 2048);
    // the Python wrote os.environ.get('ROS_DISTRO', ''); without ROS, jazzy (D10)
    cfg.ros_distro = std::env::var("ROS_DISTRO").unwrap_or_else(|_| "jazzy".into());
    cfg.data_collection = DcParams {
        session_file: params::string(&node, "session_file", ""),
        run_info: params::string(&node, "run_info", ""),
        camera_params: params::string(&node, "camera_params", ""),
        intrinsics_file: params::string(&node, "intrinsics_file", ""),
        intrinsics_placeholder: params::bool(&node, "intrinsics_placeholder", false),
        extrinsics_file: params::string(&node, "extrinsics_file", ""),
        extrinsics_derived: params::string(&node, "extrinsics_derived", ""),
    };
    let request_file = crate::request_file(&node);
    let share_config = (!cfg.config_file.is_empty())
        .then(|| PathBuf::from(&cfg.config_file).parent().map(|p| p.to_path_buf()))
        .flatten();
    let s = Settings {
        cfg,
        request_file,
        share_config,
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
    // shared with the parameter dump, which spins it while it waits
    let node = Arc::new(Mutex::new(node));
    let mut owner = Owner::new(&m, &logger, "the status, the rear light and /mower_recorder/start|stop|snapshot")?;
    let mut active: Option<Active> = None;
    enum Next {
        Stop,
        Look,
        Tick(Result<(), BusError>),
        Autostart,
        Fix(Option<Arc<NavSatFix>>),
        Fault(Option<Arc<Bool>>),
        Start(Option<Call>),
        StopCall(Option<Call>),
        Snapshot(Option<Call>),
    }
    let result: ModuleResult = async {
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
                    let serve = owner.may_serve(&lock(&node));
                    if !serve {
                        if let Some(a) = active.take() {
                            // a run going on is finalized before the other recorder takes over
                            with_manager(&a.manager, |mg| mg.stop()).await;
                        }
                    } else if active.is_none() {
                        let mut a = start(&ctx, &m, &s, &node, &logger).await?;
                        apply_request(&ctx, &s, &node, &logger, &mut a).await;
                        active = Some(a);
                    }
                }
                // a closed scope: the module is being stopped
                Next::Tick(Err(_))
                | Next::Fix(None)
                | Next::Fault(None)
                | Next::Start(None)
                | Next::StopCall(None)
                | Next::Snapshot(None) => break,
                Next::Tick(Ok(())) => {
                    if let Some(a) = active.as_mut() {
                        with_manager(&a.manager, |mg| mg.tick()).await;
                        apply_request(&ctx, &s, &node, &logger, a).await;
                    }
                }
                Next::Autostart => {
                    if let Some(a) = active.as_mut() {
                        a.autostart = None;
                        with_manager(&a.manager, |mg| mg.autostart()).await;
                    }
                }
                Next::Fix(Some(fix)) => {
                    if let Some(a) = &active {
                        let f = Fix { latitude: fix.latitude, longitude: fix.longitude, altitude: fix.altitude };
                        with_manager(&a.manager, move |mg| mg.on_fix(f)).await;
                    }
                }
                Next::Fault(Some(msg)) => {
                    if let Some(a) = &active {
                        let data = msg.data;
                        with_manager(&a.manager, move |mg| mg.on_fault(data)).await;
                    }
                }
                Next::Start(Some(req)) => reply(&active, req, |mg| mg.start()).await?,
                Next::StopCall(Some(req)) => reply(&active, req, |mg| mg.stop()).await?,
                Next::Snapshot(Some(req)) => reply(&active, req, |mg| mg.snapshot()).await?,
            }
        }
        Ok(())
    }
    .await;
    // destroy_node: a run still going is stopped and finalized
    if let Some(a) = active.take() {
        with_manager(&a.manager, |mg| mg.stop()).await;
    }
    result
}

/// A Trigger call answered with what the manager says.
async fn reply(
    active: &Option<Active>, req: Call, f: impl FnOnce(&mut Manager) -> (bool, String) + Send + 'static,
) -> Result<(), BusError> {
    let Some(a) = active else { return Ok(()) };
    let (success, message) = with_manager(&a.manager, f).await;
    req.respond(Trigger::Response { success, message })
}

/// The start of serving, in `__init__`'s order but for the manager, which
/// turns MediaMTX off and sends the first status when it is made: the
/// readers, the services, the writers, the manager, the tick.
async fn start(
    ctx: &r2r::Context, m: &ModuleCtx, s: &Settings, node: &Arc<Mutex<r2r::Node>>, logger: &str,
) -> Result<Active, BoxError> {
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
    let video: Option<Box<dyn core::Video>> = if s.video_api.is_empty() {
        None
    } else {
        Some(Box::new(MediaMtx { api_url: s.video_api.clone(), path: s.video_path.clone() }))
    };
    let params = Arc::new(RosParams { node: node.clone(), clients: Mutex::new(HashMap::new()) });
    let out = Arc::new(BusOutputs { status, rear, logger: logger.to_string() });
    let hooks = hooks(ctx, logger, video, &params, &out);
    // the timers from the start of serving, before the manager's first status
    let tick = m.bus.wall_timer(&m.clock, STATUS_PERIOD)?;
    let autostart = s.autostart.then(|| Box::pin(m.clock.sleep(AUTOSTART_DELAY)));
    let cfg = s.cfg.clone();
    // MediaMTX's PATCH blocks
    let manager = tokio::task::spawn_blocking(move || Manager::new(cfg, hooks)).await.expect("the manager's start panicked");
    Ok(Active {
        manager: Arc::new(Mutex::new(manager)),
        out,
        params,
        applied: None,
        refused: None,
        fix,
        fault,
        start,
        stop,
        snapshot,
        tick,
        autostart,
    })
}

/// What a manager is made with: the node's outputs and parameter clients,
/// new sources, MediaMTX, the clock and the disk.
fn hooks(
    ctx: &r2r::Context, logger: &str, video: Option<Box<dyn core::Video>>, params: &Arc<RosParams>, out: &Arc<BusOutputs>,
) -> Hooks {
    Hooks {
        clock: Box::new(core::SystemClock),
        disk: Box::new(core::Statvfs),
        video,
        params: params.clone(),
        sources: Box::new(crate::sources::DdsSources::new(ctx.clone(), logger)),
        out: out.clone(),
    }
}

/// E5-5: the request file as it is now, if it differs from what the
/// manager runs with and no run records, applied: the manager is made again
/// with the profile it asks for, or the default when it is gone.
async fn apply_request(ctx: &r2r::Context, s: &Settings, node: &Arc<Mutex<r2r::Node>>, logger: &str, a: &mut Active) {
    let now = match std::fs::read_to_string(&s.request_file) {
        Ok(text) => Some(text),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => None,
        Err(e) => {
            let why = format!("unreadable: {e}");
            if a.refused.as_deref() != Some(why.as_str()) {
                r2r::log_error!(logger, "{}: {why}", s.request_file.display());
                a.refused = Some(why);
            }
            return;
        }
    };
    if now == a.applied || (now.is_some() && now == a.refused) {
        return;
    }
    if with_manager(&a.manager, |mg| mg.recording()).await {
        // applied once the run has stopped
        return;
    }
    let cfg = match &now {
        None => {
            r2r::log_info!(logger, "{} is gone: the default profile", s.request_file.display());
            s.cfg.clone()
        }
        Some(text) => {
            let resolved = (|| {
                let req = mower_recorder_core::dc_request::Request::parse(text)?;
                let share = s.share_config.clone().ok_or("no config_file, so no data_collection.yaml next to it")?;
                let node = node.clone();
                Ok::<_, String>((req, share, node))
            })();
            let (req, share, node2) = match resolved {
                Ok(v) => v,
                Err(e) => {
                    r2r::log_error!(logger, "{}: {e}; the profile stays as it is", s.request_file.display());
                    a.refused = now.clone();
                    return;
                }
            };
            let share2 = share.clone();
            let r = tokio::task::spawn_blocking(move || {
                let urdf = crate::robot_description(&mut lock(&node2));
                mower_recorder_core::dc_request::resolve(&req, &share2, urdf.as_deref().map_err(|e| e.clone()))
            })
            .await
            .expect("the request's resolution panicked");
            let r = match r {
                Ok(r) => r,
                Err(e) => {
                    r2r::log_error!(logger, "{}: {e}; the profile stays as it is", s.request_file.display());
                    a.refused = now.clone();
                    return;
                }
            };
            for note in &r.notes {
                r2r::log_warn!(logger, "[data_collection] WARNING: {note}");
            }
            if let Some((path, body)) = &r.placeholder {
                if let Err(e) = std::fs::write(path, body) {
                    r2r::log_error!(logger, "{}: {e}", path.display());
                }
            }
            r2r::log_info!(logger, "{}: profile data_collection, camera: {}", s.request_file.display(), r.pipeline);
            let mut cfg = s.cfg.clone();
            cfg.config_file = share.join("data_collection.yaml").to_string_lossy().into_owned();
            cfg.params_dump_nodes = mower_recorder_core::dc_request::PARAMS_DUMP_NODES.iter().map(|n| n.to_string()).collect();
            cfg.data_collection = r.dc;
            cfg
        }
    };
    // data_collection.launch.py gave its recorder_manager no video_api_url:
    // the camera is gscam's then (mower-camera.service, MediaMTX's source,
    // is stopped for the run), so no MediaMTX recording either
    let video: Option<Box<dyn core::Video>> = if s.video_api.is_empty() || now.is_some() {
        None
    } else {
        Some(Box::new(MediaMtx { api_url: s.video_api.clone(), path: s.video_path.clone() }))
    };
    let hooks = hooks(ctx, logger, video, &a.params, &a.out);
    let manager = tokio::task::spawn_blocking(move || Manager::new(cfg, hooks)).await.expect("the manager's start panicked");
    *lock(&a.manager) = manager;
    a.applied = now;
    a.refused = None;
}

/// The status and the rear light on the bus, the log on the r2r node.
struct BusOutputs {
    status: Publisher<Text>,
    rear: Option<Publisher<Text>>,
    logger: String,
}

impl Outputs for BusOutputs {
    fn status(&self, json: &str) {
        // a closed scope: the instance is stopping
        let _ = self.status.publish_owned(Text { data: json.to_string() });
    }

    fn rear_light(&self, json: &str) {
        if let Some(rear) = &self.rear {
            let _ = rear.publish_owned(Text { data: json.to_string() });
        }
    }

    fn log(&self, level: Level, msg: &str) {
        match level {
            Level::Info => r2r::log_info!(&self.logger, "{msg}"),
            Level::Warn => r2r::log_warn!(&self.logger, "{msg}"),
            Level::Error => r2r::log_error!(&self.logger, "{msg}"),
        }
    }
}

type ListClient = r2r::Client<ListParameters::Service>;
type GetClient = r2r::Client<GetParameters::Service>;

/// `ros2 param dump <node>`'s two calls (`ListParameters` with no prefix,
/// recursive; then `GetParameters` of every name), through clients made on
/// recorder_manager's node the first time a node is dumped. The dump's
/// thread spins the node while it waits, so nothing else has to.
pub struct RosParams {
    node: Arc<Mutex<r2r::Node>>,
    clients: Mutex<HashMap<String, (Arc<ListClient>, Arc<GetClient>)>>,
}

impl RosParams {
    fn clients(&self, target: &str) -> Result<(Arc<ListClient>, Arc<GetClient>), String> {
        let mut clients = lock(&self.clients);
        if !clients.contains_key(target) {
            let mut node = lock(&self.node);
            let list = node.create_client::<ListParameters::Service>(&format!("{target}/list_parameters"), QosProfile::services_default());
            let get = node.create_client::<GetParameters::Service>(&format!("{target}/get_parameters"), QosProfile::services_default());
            match (list, get) {
                (Ok(l), Ok(g)) => {
                    clients.insert(target.to_string(), (Arc::new(l), Arc::new(g)));
                }
                (Err(e), _) | (_, Err(e)) => return Err(format!("{target}: {e}")),
            }
        }
        let (l, g) = clients.get(target).expect("made");
        Ok((l.clone(), g.clone()))
    }

    /// Polls `fut`, spinning the node between polls, until it is ready or
    /// `timeout` has passed.
    fn drive<F: std::future::Future>(&self, fut: F, timeout: Duration) -> Option<F::Output> {
        let mut fut = Box::pin(fut);
        let waker = futures::task::noop_waker();
        let mut cx = Context::from_waker(&waker);
        let deadline = Instant::now() + timeout;
        loop {
            if let Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
                return Some(v);
            }
            if Instant::now() >= deadline {
                return None;
            }
            lock(&self.node).spin_once(Duration::from_millis(10));
        }
    }

    fn ready<T: r2r::WrappedServiceTypeSupport + 'static>(&self, client: &r2r::Client<T>, what: &str) -> Result<(), String> {
        let available = r2r::Node::is_available(client).map_err(|e| format!("{what}: {e}"))?;
        match self.drive(available, PARAM_TIMEOUT) {
            Some(Ok(())) => Ok(()),
            Some(Err(e)) => Err(format!("{what}: {e}")),
            None => Err(format!("{what}: service not available")),
        }
    }
}

impl ParamSource for RosParams {
    fn parameters(&self, target: &str) -> Result<Vec<(String, ParamValue)>, String> {
        let (list, get) = self.clients(target)?;
        self.ready(&*list, &format!("{target}/list_parameters"))?;
        let request = ListParameters::Request { prefixes: Vec::new(), depth: 0 };
        let names = match self.drive(list.request(&request).map_err(|e| e.to_string())?, PARAM_TIMEOUT) {
            Some(Ok(r)) => r.result.names,
            Some(Err(e)) => return Err(format!("{target}/list_parameters: {e}")),
            None => return Err(format!("{target}/list_parameters: no answer")),
        };
        self.ready(&*get, &format!("{target}/get_parameters"))?;
        let request = GetParameters::Request { names: names.clone() };
        let values = match self.drive(get.request(&request).map_err(|e| e.to_string())?, PARAM_TIMEOUT) {
            Some(Ok(r)) => r.values,
            Some(Err(e)) => return Err(format!("{target}/get_parameters: {e}")),
            None => return Err(format!("{target}/get_parameters: no answer")),
        };
        if values.len() != names.len() {
            return Err(format!("{target}/get_parameters: {} values for {} names", values.len(), names.len()));
        }
        Ok(names.into_iter().zip(values.iter().map(param_value)).collect())
    }
}

/// `rcl_interfaces/msg/ParameterValue` as the dump writes it
/// (`parameter_value_to_python`; a byte array is left out as not set).
pub fn param_value(v: &r2r::rcl_interfaces::msg::ParameterValue) -> ParamValue {
    match v.type_ {
        1 => ParamValue::Bool(v.bool_value),
        2 => ParamValue::Integer(v.integer_value),
        3 => ParamValue::Double(v.double_value),
        4 => ParamValue::String(v.string_value.clone()),
        6 => ParamValue::BoolArray(v.bool_array_value.clone()),
        7 => ParamValue::IntegerArray(v.integer_array_value.clone()),
        8 => ParamValue::DoubleArray(v.double_array_value.clone()),
        9 => ParamValue::StringArray(v.string_array_value.clone()),
        _ => ParamValue::NotSet,
    }
}
