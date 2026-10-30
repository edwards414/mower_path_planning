//! `bag_store` (bag_store_node.py): the run list the app shows, its
//! commands, and the R2 upload. The JSON, the texts, the R2 settings and the
//! upload are `mower_recorder_core::{store, manifest, s3}`'s, byte for byte
//! the Python's; this is the node around them.
//!
//! - **`/mower_recorder/bags`** (R TL 1): at the start of serving, every
//!   `scan_period_s` (5 s), after a refresh, a rename and a delete, and
//!   around each run the uploader sends.
//! - **`/mower_recorder/command`** (R V 10) → **`command_result`** (R V 10):
//!   `refresh`, `rename`, `delete` (the R2 objects too, manifest first, when
//!   the run went up or started to), `upload_now` (a thread of its own,
//!   past the network gate), anything else `unknown action: <a>`. A command
//!   the Python's callback would have raised on (not an object, a field of
//!   another type) gets no answer, and an ERROR in the log.
//! - **`/mower_recorder/status`** (R TL 1, recorder_manager's): the run being
//!   recorded, which is neither uploaded nor deleted.
//! - **The upload** every `upload_period_s` (30 s) on a std thread
//!   (`bag-upload`), when `upload_policy` allows (`wifi_or_dock`: WiFi up
//!   with an IPv4 address, or docked). Which run is being uploaded is known
//!   process-wide, so that an instance that restarts while its old thread
//!   still uploads does not send the same run twice.
//! - **R2:** `R2_*` from the environment, overridden by `r2_env_file`'s lines
//!   (bag_store's precedence), read once at the start. `r2_endpoint_url`
//!   (empty: `https://<account>.r2.cloudflarestorage.com`) is for tests.
//! - **Parameters** as record.launch.py passes them: `output_root`,
//!   `robot_id`, `r2_env_file` from the launch, `upload_policy`
//!   `wifi_or_dock`, `wifi_interfaces` `[wlan0]`, `dock_topic` `''`
//!   (mower_rsd.yaml).

use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};
use std::time::{Duration, Instant};

use mower_bus::{BusError, Publisher, Qos, Subscriber, Timer};
use mower_pyfmt::Py;
use mower_recorder_core::manifest::{self, UploadError, MANIFEST_NAME};
use mower_recorder_core::s3;
use mower_recorder_core::store::{self as core, Command, ListState, Policy, R2Config};
use mower_rs_common::{params, ModuleCtx, ModuleResult};
use r2r::std_msgs::msg::{Bool, String as Text};

use crate::owner::Owner;

pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let node = r2r::Node::create(ctx, &m.node_name, &m.namespace)?;
    run_node(node, m).await
}

/// The run directories some instance of this process is uploading now.
fn uploading() -> MutexGuard<'static, HashSet<PathBuf>> {
    static UPLOADING: OnceLock<Mutex<HashSet<PathBuf>>> = OnceLock::new();
    UPLOADING.get_or_init(Default::default).lock().unwrap_or_else(|e| e.into_inner())
}

/// What the node and its upload threads share.
struct Shared {
    root: PathBuf,
    robot_id: String,
    policy: Policy,
    policy_text: String,
    wifi: Vec<String>,
    r2: R2Config,
    endpoint: String,
    logger: String,
    bags: Publisher<Text>,
    state: Mutex<State>,
}

#[derive(Default)]
struct State {
    /// The run being recorded (`/mower_recorder/status`).
    recording: Option<String>,
    docked: bool,
    upload_errors: HashMap<String, String>,
}

impl Shared {
    fn state(&self) -> MutexGuard<'_, State> {
        self.state.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn list_state(&self) -> ListState {
        let st = self.state();
        let uploading: HashSet<String> = uploading()
            .iter()
            .filter(|d| d.parent() == Some(self.root.as_path()))
            .filter_map(|d| d.file_name().map(|n| n.to_string_lossy().into_owned()))
            .collect();
        ListState {
            robot_id: self.robot_id.clone(),
            uploading,
            recording: st.recording.clone(),
            upload_errors: st.upload_errors.clone(),
        }
    }

    fn network_ok(&self) -> bool {
        let docked = self.state().docked;
        core::network_ok(self.policy, docked, &self.wifi)
    }

    /// `_publish_bags`
    fn publish_bags(&self) {
        let st = self.list_state();
        match core::bags_json(&self.root, &st, self.network_ok(), self.r2.configured()) {
            Ok(text) => {
                // a closed scope: the instance is stopping, the next start publishes again
                let _ = self.bags.publish_owned(Text { data: text });
            }
            Err(e) => r2r::log_error!(&self.logger, "bag list: {e}"),
        }
    }

    /// `_r2_client`: a bucket and an account, else none.
    fn client(&self) -> Option<s3::Client> {
        if self.endpoint.is_empty() {
            return self.r2.client();
        }
        if self.r2.bucket.is_empty() || self.r2.account_id.is_empty() {
            return None;
        }
        let cred = s3::Credentials {
            access_key: self.r2.access_key_id.clone(),
            secret_key: self.r2.secret_access_key.clone(),
        };
        Some(s3::Client::new(&self.endpoint, cred))
    }

    /// `_upload_tick(force)`
    fn upload_tick(&self, force: bool) {
        if !(force || self.network_ok()) {
            return;
        }
        let Some(client) = self.client() else { return };
        let st = self.list_state();
        for run in core::list_runs(&self.root, &st) {
            let Some(Py::Str(rid)) = run.get("run_id").cloned() else { continue };
            let flag = |k: &str| matches!(run.get(k), Some(Py::Bool(true)));
            if flag("uploaded") || flag("recording") || flag("uploading") {
                continue;
            }
            let run_dir = self.root.join(&rid);
            if !uploading().insert(run_dir.clone()) {
                continue;
            }
            self.publish_bags();
            match self.upload_run(&client, &rid, &run_dir) {
                Ok(()) => {
                    self.state().upload_errors.remove(&rid);
                }
                Err(UploadError::NotReady(why)) => {
                    let mut st = self.state();
                    if st.upload_errors.get(&rid) != Some(&why) {
                        r2r::log_warn!(&self.logger, "not uploading {rid}: {why}");
                    }
                    st.upload_errors.insert(rid.clone(), why);
                }
                Err(e) => {
                    r2r::log_error!(
                        &self.logger,
                        "upload {rid} interrupted ({}: {e}); will resume on the next tick",
                        e.type_name()
                    );
                    self.state().upload_errors.insert(rid.clone(), e.upload_error());
                }
            }
            uploading().remove(&run_dir);
            self.publish_bags();
        }
    }

    /// `_upload_run`
    fn upload_run(&self, client: &s3::Client, run_id: &str, run_dir: &Path) -> Result<(), UploadError> {
        let base_key = core::base_key(run_dir, &self.r2.prefix, &self.robot_id, run_id);
        let robot = match core::read_meta(run_dir).get("robot_id") {
            Some(Py::Str(s)) if !s.is_empty() => s.clone(),
            _ => self.robot_id.clone(),
        };
        let logger = self.logger.clone();
        let mut debug = |line: String| r2r::log_debug!(&logger, "{line}");
        let res = manifest::upload_run(client, &self.r2.bucket, &base_key, run_dir, run_id, &robot, None, &mut debug)?;
        r2r::log_info!(
            &self.logger,
            "uploaded {run_id} -> r2://{}/{base_key} ({} files, {} new, {} resumed) + {MANIFEST_NAME}",
            self.r2.bucket,
            res.files,
            res.uploaded,
            res.skipped
        );
        Ok(())
    }

    /// `_delete_r2`
    fn delete_r2(&self, base_key: &str) {
        let Some(client) = self.client() else { return };
        match manifest::delete_run_objects(&client, &self.r2.bucket, base_key) {
            Ok(n) => r2r::log_info!(&self.logger, "deleted r2 {base_key} ({n} objects)"),
            Err(e) => r2r::log_error!(&self.logger, "delete r2 failed: {e}"),
        }
    }
}

/// The 30 s upload thread; dropping it stops the thread at its next wake
/// (an upload under way finishes first).
struct Uploader {
    stop: Arc<AtomicBool>,
    thread: std::thread::Thread,
}

impl Uploader {
    fn spawn(shared: Arc<Shared>, period: Duration) -> std::io::Result<Uploader> {
        let stop = Arc::new(AtomicBool::new(false));
        let stopped = stop.clone();
        let handle = std::thread::Builder::new().name("bag-upload".into()).spawn(move || loop {
            let due = Instant::now() + period;
            while !stopped.load(Ordering::Acquire) {
                let now = Instant::now();
                if now >= due {
                    break;
                }
                std::thread::park_timeout(due - now);
            }
            if stopped.load(Ordering::Acquire) {
                return;
            }
            shared.upload_tick(false);
        })?;
        Ok(Uploader { stop, thread: handle.thread().clone() })
    }
}

impl Drop for Uploader {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        self.thread.unpark();
    }
}

struct Settings {
    root: PathBuf,
    robot_id: String,
    policy: String,
    wifi: Vec<String>,
    r2: R2Config,
    endpoint: String,
    dock_topic: String,
    scan_period: Duration,
    upload_period: Duration,
    bags_topic: String,
    result_topic: String,
    command_topic: String,
    status_topic: String,
}

/// What the node holds while it serves.
struct Active {
    shared: Arc<Shared>,
    result: Publisher<Text>,
    commands: Subscriber<Text>,
    status: Subscriber<Text>,
    dock: Option<Subscriber<Bool>>,
    scan: Timer,
    _uploader: Uploader,
}

fn seconds(node: &r2r::Node, name: &str, default: f64) -> Result<Duration, String> {
    let s = params::f64(node, name, default);
    if s.is_finite() && s > 0.0 {
        Ok(Duration::from_secs_f64(s))
    } else {
        Err(format!("{name} must be a positive number of seconds, not {s}"))
    }
}

pub async fn run_node(node: r2r::Node, m: ModuleCtx) -> ModuleResult {
    let logger = node.logger().to_string();
    let env_file = crate::expanduser(&params::string(&node, "r2_env_file", ""));
    let env = |k: &str| std::env::var(k).ok();
    let r2 = match core::load_r2_config(&env, Some(Path::new(&env_file))) {
        Ok(c) => c,
        Err(e) => {
            r2r::log_error!(&logger, "{e}");
            core::load_r2_config(&env, None).unwrap_or_default()
        }
    };
    let s = Settings {
        root: PathBuf::from(crate::expanduser(&params::string(&node, "output_root", "~/mower_bags"))),
        robot_id: params::string(&node, "robot_id", "mower"),
        policy: params::string(&node, "upload_policy", "wifi_or_dock"),
        wifi: crate::params::strings(&node, "wifi_interfaces", &["wlan0"]),
        r2,
        endpoint: params::string(&node, "r2_endpoint_url", ""),
        dock_topic: params::string(&node, "dock_topic", ""),
        scan_period: seconds(&node, "scan_period_s", 5.0)?,
        upload_period: seconds(&node, "upload_period_s", 30.0)?,
        bags_topic: m.resolve_topic("/mower_recorder/bags")?,
        result_topic: m.resolve_topic("/mower_recorder/command_result")?,
        command_topic: m.resolve_topic("/mower_recorder/command")?,
        status_topic: m.resolve_topic("/mower_recorder/status")?,
    };
    let mut owner = Owner::new(&m, &logger, "the bag list, the commands and the uploads")?;
    let mut active: Option<Active> = None;
    enum Next {
        Stop,
        Look,
        Command(Option<Arc<Text>>),
        Status(Option<Arc<Text>>),
        Dock(Option<Arc<Bool>>),
        Scan(Result<(), BusError>),
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
                msg = a.status.recv() => Next::Status(msg),
                msg = async {
                    match a.dock.as_mut() {
                        Some(d) => d.recv().await,
                        None => std::future::pending().await,
                    }
                } => Next::Dock(msg),
                msg = a.commands.recv() => Next::Command(msg),
                tick = a.scan.tick() => Next::Scan(tick.map(|_| ())),
            },
        };
        match next {
            Next::Stop => break,
            Next::Look => {
                if !owner.may_serve(&node) {
                    active = None;
                } else if active.is_none() {
                    active = Some(start(&m, &s, &logger)?);
                }
            }
            // a closed scope: the module is being stopped
            Next::Command(None) | Next::Status(None) | Next::Dock(None) | Next::Scan(Err(_)) => break,
            Next::Status(Some(msg)) => {
                if let Some(a) = &active {
                    a.shared.state().recording = recording_run(&msg.data);
                }
            }
            Next::Dock(Some(msg)) => {
                if let Some(a) = &active {
                    a.shared.state().docked = msg.data;
                }
            }
            Next::Scan(Ok(())) => {
                if let Some(a) = &active {
                    a.shared.publish_bags();
                }
            }
            Next::Command(Some(msg)) => {
                if let Some(a) = &active {
                    a.command(&msg.data).await?;
                }
            }
        }
    }
    drop(active);
    drop(node);
    Ok(())
}

fn start(m: &ModuleCtx, s: &Settings, logger: &str) -> Result<Active, Box<dyn std::error::Error + Send + Sync>> {
    if let Err(e) = std::fs::create_dir_all(&s.root) {
        return Err(format!("output_root {}: {e}", s.root.display()).into());
    }
    let bags = m.bus.publisher::<Text>(&s.bags_topic, Qos::latched())?;
    let result = m.bus.publisher::<Text>(&s.result_topic, Qos::default())?;
    let commands = m.bus.subscribe::<Text>(&s.command_topic, Qos::default())?;
    let status = m.bus.subscribe::<Text>(&s.status_topic, Qos::latched())?;
    let dock = if s.dock_topic.is_empty() {
        None
    } else {
        Some(m.bus.subscribe::<Bool>(&m.resolve_topic(&s.dock_topic)?, Qos::default())?)
    };
    let scan = m.bus.wall_timer(&m.clock, s.scan_period)?;
    let shared = Arc::new(Shared {
        root: s.root.clone(),
        robot_id: s.robot_id.clone(),
        policy: Policy::parse(&s.policy),
        policy_text: s.policy.clone(),
        wifi: s.wifi.clone(),
        r2: s.r2.clone(),
        endpoint: s.endpoint.clone(),
        logger: logger.to_string(),
        bags,
        state: Mutex::new(State::default()),
    });
    let uploader = Uploader::spawn(shared.clone(), s.upload_period)?;
    r2r::log_info!(
        logger,
        "bag_store up: root={} policy={} r2={}",
        s.root.display(),
        shared.policy_text,
        if shared.r2.configured() { "configured" } else { "NOT configured" }
    );
    shared.publish_bags();
    Ok(Active { shared, result, commands, status, dock, scan, _uploader: uploader })
}

/// `_on_status`: the run being recorded, `None` for anything else.
pub fn recording_run(text: &str) -> Option<String> {
    let st = mower_pyfmt::json::loads(text).ok()?;
    if !matches!(st, Py::Dict(_)) || !mower_recorder_core::profile::truthy(st.get("recording")) {
        return None;
    }
    match st.get("run_id") {
        Some(Py::Str(s)) => Some(s.clone()),
        _ => None,
    }
}

impl Active {
    /// `_on_command`
    async fn command(&self, text: &str) -> Result<(), BusError> {
        let sh = &self.shared;
        let (cmd, req_id) = match core::parse_command(text) {
            None => {
                r2r::log_error!(&sh.logger, "command not answered (the Python bag_store's callback raises on it): {text}");
                return Ok(());
            }
            Some(Err(why)) => return self.reply(&Py::str(""), false, &why),
            Some(Ok(c)) => c,
        };
        match cmd {
            Command::Refresh => {
                sh.publish_bags();
                self.reply(&req_id, true, core::REFRESHED)
            }
            Command::Rename { run_id, new_name } => {
                let (ok, message) = core::rename(&sh.root, &run_id, &new_name);
                if ok {
                    sh.publish_bags();
                }
                self.reply(&req_id, ok, &message)
            }
            Command::Delete { run_id } => {
                let recording = sh.state().recording.clone();
                let deleted = core::delete_local(&sh.root, &run_id, recording.as_deref(), &sh.r2.prefix, &sh.robot_id);
                if deleted.ok {
                    if let Some(base_key) = deleted.r2_base_key.clone() {
                        // the R2 requests block: off the runtime, and the
                        // answer after them, as the Python's
                        let shared = sh.clone();
                        let _ = tokio::task::spawn_blocking(move || shared.delete_r2(&base_key)).await;
                    }
                    sh.state().upload_errors.remove(&run_id);
                    sh.publish_bags();
                }
                self.reply(&req_id, deleted.ok, &deleted.message)
            }
            Command::UploadNow => {
                let shared = sh.clone();
                if let Err(e) = std::thread::Builder::new().name("bag-upload-now".into()).spawn(move || shared.upload_tick(true)) {
                    r2r::log_error!(&sh.logger, "upload_now: no thread: {e}");
                }
                self.reply(&req_id, true, core::UPLOAD_QUEUED)
            }
            Command::Unknown(action) => self.reply(&req_id, false, &core::unknown_action(&action)),
        }
    }

    /// `_result`
    fn reply(&self, req_id: &Py, ok: bool, message: &str) -> Result<(), BusError> {
        self.result.publish_owned(Text { data: core::result_json(req_id, ok, message) })?;
        if ok {
            r2r::log_info!(&self.shared.logger, "command_result: {message}");
        } else {
            r2r::log_warn!(&self.shared.logger, "command_result: {message}");
        }
        Ok(())
    }
}
