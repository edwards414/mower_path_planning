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
//!   data_collection parameters are read (`session_file`, `camera_params`,
//!   ...), but that profile comes with E5-5 (mower-data-collection.sh refuses
//!   while RUST_RECORDER is on).

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
                        active = Some(start(&ctx, &m, &s, &node, &logger).await?);
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
                    if let Some(a) = &active {
                        with_manager(&a.manager, |mg| mg.tick()).await;
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
    let hooks = Hooks {
        clock: Box::new(core::SystemClock),
        disk: Box::new(core::Statvfs),
        video,
        params: Arc::new(RosParams { node: node.clone(), clients: Mutex::new(HashMap::new()) }),
        sources: Box::new(crate::sources::DdsSources::new(ctx.clone(), logger)),
        out: Arc::new(BusOutputs { status, rear, logger: logger.to_string() }),
    };
    // the timers from the start of serving, before the manager's first status
    let tick = m.bus.wall_timer(&m.clock, STATUS_PERIOD)?;
    let autostart = s.autostart.then(|| Box::pin(m.clock.sleep(AUTOSTART_DELAY)));
    let cfg = s.cfg.clone();
    // MediaMTX's PATCH blocks
    let manager = tokio::task::spawn_blocking(move || Manager::new(cfg, hooks)).await.expect("the manager's start panicked");
    Ok(Active { manager: Arc::new(Mutex::new(manager)), fix, fault, start, stop, snapshot, tick, autostart })
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
