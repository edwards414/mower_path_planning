//! recorder_manager (`recorder_manager_node.py`) without ROS: a run's start,
//! stop and snapshot, the 2 s tick (disk guard, `/mower_recorder/status`,
//! the rear light, the video-off retry), the fault input, the run metadata
//! and the parameter dump, in the Python's order and with its texts (every
//! reply, status and file goes through [`crate::run`], `dc_metadata` and
//! `params`).
//!
//! What the module provides is behind small traits ([`Hooks`]): the local
//! clock, the disk check, MediaMTX, the parameter source for the dumps,
//! the groups' sources (DDS or bus subscriptions feeding a
//! [`crate::recording::Feed`]) and the outputs (status, rear light, log).
//! The module owns one [`Manager`] behind a mutex (the Python's start/stop
//! lock) and calls:
//!
//! - [`Manager::new`] when the node starts (it turns MediaMTX's recording
//!   off, as after a crash, and publishes the first status);
//! - [`Manager::start`], [`Manager::stop`], [`Manager::snapshot`] for
//!   `/mower_recorder/{start,stop,snapshot}` (the `Trigger` replies);
//! - [`Manager::tick`] every 2 s; [`Manager::autostart`] 3 s after the
//!   start when `autostart` is on;
//! - [`Manager::on_fault`] for `/mower_recorder/fault`, [`Manager::on_fix`]
//!   for `/fix`;
//! - [`Manager::stop`] when the node goes down (`destroy_node`).
//!
//! The calls block (MediaMTX's 3 s timeouts, a finalize): the module runs
//! them off its async threads. The parameter dump runs on its own thread
//! after the start, as the Python's did.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::thread::JoinHandle;

use mower_pyfmt::{yaml, Py};

use crate::dc_metadata::{self, MetadataInputs};
use crate::extrinsics::py_str;
use crate::mediamtx;
use crate::params::{self, ParamValue};
use crate::profile::{self, str_list, truthy, ConfigLog, GroupSpec};
use crate::recording::{self, Feed, Level, Log, QosOverrides, Recording};
use crate::run::{self, Fix, Status, VideoNote};
use crate::util::{expand_user, LocalTime};

/// `datetime.datetime.now()`
pub trait Clock: Send {
    fn now(&self) -> LocalTime;
}

/// The process's local time.
pub struct SystemClock;

impl Clock for SystemClock {
    fn now(&self) -> LocalTime {
        LocalTime::now()
    }
}

/// `record_profile.disk_low(path, min_free_mb)`
pub trait Disk: Send {
    fn low(&self, path: &Path, min_free_mb: i64) -> bool;
}

/// `statvfs` of the file system holding the path.
pub struct Statvfs;

impl Disk for Statvfs {
    fn low(&self, path: &Path, min_free_mb: i64) -> bool {
        profile::disk_low(path, min_free_mb)
    }
}

/// MediaMTX's API for the camera path (`video_record.py`).
pub trait Video: Send {
    /// `video_path` (`front`), as the log lines name it.
    fn path(&self) -> &str;
    /// `patch_path`: (ok, message).
    fn patch(&self, body: &Py) -> (bool, String);
    /// `path_ready`: `None` when MediaMTX cannot be asked.
    fn ready(&self) -> Option<bool>;
}

/// The real MediaMTX at `video_api_url`.
pub struct MediaMtx {
    pub api_url: String,
    pub path: String,
}

impl Video for MediaMtx {
    fn path(&self) -> &str {
        &self.path
    }
    fn patch(&self, body: &Py) -> (bool, String) {
        mediamtx::patch_path(&self.api_url, &self.path, body)
    }
    fn ready(&self) -> Option<bool> {
        mediamtx::path_ready(&self.api_url, &self.path)
    }
}

/// What `ros2 param dump <node>` asks the node (`ListParameters`, then
/// `GetParameters`). `node` is the absolute name; an `Err` is logged as the
/// Python logged a failed dump.
pub trait ParamSource: Send + Sync {
    fn parameters(&self, node: &str) -> Result<Vec<(String, ParamValue)>, String>;
}

/// The groups' sources: what subscribes to a group's names and feeds its
/// recording (recording.rs says how).
pub trait Sources: Send {
    /// A group starts (in start order, after its bag exists): subscribe to
    /// `spec.recorded` (the hidden names already left out) with
    /// `recording::reader_qos(name, offers, overrides)`, feed `feed`, and
    /// show `spec.node_name` in the graph while it runs (D12).
    fn start(&mut self, spec: &GroupSpec, overrides: &QosOverrides, feed: Feed) -> Result<(), String>;
    /// Stop feeding the group (the Python's SIGINT): called for every
    /// running group before any of them is finalized.
    fn stop(&mut self, group: &str);
}

/// What the node publishes and logs.
pub trait Outputs: Send + Sync {
    /// `/mower_recorder/status` (reliable, transient local, depth 1)
    fn status(&self, json: &str);
    /// `rear_light_topic` (`/mower_base/rear_light`, the same QoS)
    fn rear_light(&self, json: &str);
    fn log(&self, level: Level, msg: &str);
}

/// What the module provides.
pub struct Hooks {
    pub clock: Box<dyn Clock>,
    pub disk: Box<dyn Disk>,
    /// `None` when `video_api_url` is empty.
    pub video: Option<Box<dyn Video>>,
    pub params: Arc<dyn ParamSource>,
    pub sources: Box<dyn Sources>,
    pub out: Arc<dyn Outputs>,
}

/// The profile-run parameters (data_collection.launch.py; `''` = not used).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct DcParams {
    pub session_file: String,
    pub run_info: String,
    pub camera_params: String,
    pub intrinsics_file: String,
    pub intrinsics_placeholder: bool,
    pub extrinsics_file: String,
    pub extrinsics_derived: String,
}

/// The node's parameters.
#[derive(Clone)]
pub struct Config {
    pub robot_id: String,
    /// `output_root`, `~` expanded
    pub output_root: PathBuf,
    pub config_file: String,
    pub qos_overrides_path: String,
    pub git_repo_dir: String,
    /// `params_dump_nodes` (empty entries are ignored)
    pub params_dump_nodes: Vec<String>,
    /// `rear_light_topic` is not empty
    pub rear_light: bool,
    pub min_free_mb: i64,
    /// `run_metadata.yaml`'s `ros_distro` (the Python read `ROS_DISTRO`;
    /// D10: `jazzy` once it is gone)
    pub ros_distro: String,
    /// The MCAP header's `library` (D1).
    pub library: String,
    pub data_collection: DcParams,
    /// `os.environ.get`, for the data_collection `build` block
    /// (`MOWER_VERSION`, `MOWER_GIT_SHA`, `MOWER_IMAGE`).
    pub env: Arc<dyn Fn(&str) -> Option<String> + Send + Sync>,
}

impl Config {
    /// The node's defaults (rm:72-96).
    pub fn new() -> Config {
        Config {
            robot_id: "mower".into(),
            output_root: PathBuf::from(expand_user("~/mower_bags")),
            config_file: String::new(),
            qos_overrides_path: String::new(),
            git_repo_dir: String::new(),
            params_dump_nodes: Vec::new(),
            rear_light: true,
            min_free_mb: 2048,
            ros_distro: "jazzy".into(),
            library: format!("mower_recorder {}", env!("CARGO_PKG_VERSION")),
            data_collection: DcParams::default(),
            env: Arc::new(|k| std::env::var(k).ok()),
        }
    }
}

impl Default for Config {
    fn default() -> Self {
        Config::new()
    }
}

struct Group {
    spec: GroupSpec,
    rec: Recording,
}

/// One recorder_manager.
pub struct Manager {
    cfg: Config,
    profile: Py,
    hooks: Hooks,
    /// the groups of the run, in start order (`self._procs`)
    groups: Vec<Group>,
    run_id: Option<String>,
    run_dir: Option<PathBuf>,
    start_time: Option<LocalTime>,
    last_fix: Option<Fix>,
    video_on: bool,
    video_error: Option<String>,
    stop_reason: Option<String>,
    dumps: Vec<JoinHandle<()>>,
}

impl Manager {
    /// The node's constructor: the profile is loaded (its problems logged),
    /// MediaMTX's recording is turned off (a recorder that died mid-run must
    /// not leave it on), and the first status goes out.
    pub fn new(cfg: Config, hooks: Hooks) -> Manager {
        let mut logs = Vec::new();
        let path = (!cfg.config_file.is_empty()).then(|| PathBuf::from(&cfg.config_file));
        let profile = profile::load_config(path.as_deref(), &mut logs);
        let mut m = Manager {
            cfg,
            profile,
            hooks,
            groups: Vec::new(),
            run_id: None,
            run_dir: None,
            start_time: None,
            last_fix: None,
            video_on: false,
            video_error: None,
            stop_reason: None,
            dumps: Vec::new(),
        };
        for l in logs {
            match l {
                ConfigLog::Warn(s) => m.log(Level::Warn, &s),
                ConfigLog::Error(s) => m.log(Level::Error, &s),
            }
        }
        if m.hooks.video.is_some() {
            m.video_on = true;
            m.set_video(false);
        }
        m.publish_status();
        m
    }

    fn log(&self, level: Level, msg: &str) {
        self.hooks.out.log(level, msg);
    }

    fn log_fn(&self) -> Log {
        let out = self.hooks.out.clone();
        Arc::new(move |l, m| out.log(l, m))
    }

    /// The loaded profile (`self._cfg`).
    pub fn profile(&self) -> &Py {
        &self.profile
    }

    pub fn run_id(&self) -> Option<&str> {
        self.run_id.as_deref()
    }

    pub fn run_dir(&self) -> Option<&Path> {
        self.run_dir.as_deref()
    }

    fn alive(&self, name: &str) -> bool {
        self.groups.iter().any(|g| g.spec.name == name && g.rec.is_alive())
    }

    /// `_recording()`: the full or the camera group runs.
    pub fn recording(&self) -> bool {
        profile::DATA_RECORDERS.iter().any(|n| self.alive(n))
    }

    /// The running groups' counters.
    pub fn stats(&self) -> Vec<(&'static str, recording::Stats)> {
        self.groups.iter().map(|g| (g.spec.name, g.rec.stats())).collect()
    }

    /// `_autostart_once`
    pub fn autostart(&mut self) {
        let (ok, msg) = self.start();
        self.log(if ok { Level::Info } else { Level::Error }, &format!("autostart: {msg}"));
    }

    /// `/mower_recorder/start` (`_start_recording_locked`).
    pub fn start(&mut self) -> (bool, String) {
        if self.recording() {
            return (false, run::ALREADY_RECORDING.into());
        }
        if !truthy(self.profile.get("topics")) {
            return (false, run::NO_TOPICS.into());
        }
        if self.hooks.disk.low(&self.cfg.output_root, self.cfg.min_free_mb) {
            return (false, run::disk_low_message(self.cfg.min_free_mb));
        }
        let run_id = run::run_id(&self.cfg.robot_id, &self.hooks.clock.now());
        let run_dir = self.cfg.output_root.join(&run_id);
        if let Err(e) = std::fs::create_dir_all(&run_dir) {
            // the Python's service callback raised here (A9)
            let msg = format!("{}: {e}", run_dir.display());
            self.log(Level::Error, &msg);
            return (false, msg);
        }
        self.run_id = Some(run_id.clone());
        self.run_dir = Some(run_dir.clone());
        self.start_time = Some(self.hooks.clock.now());
        self.stop_reason = None;
        self.write_metadata(&run_id, &run_dir);

        let mut note = VideoNote::None;
        if self.hooks.video.is_some() {
            let _ = std::fs::create_dir_all(run_dir.join(mediamtx::VIDEO_DIR));
            let (ok, msg) = self.set_video(true);
            note = if !ok {
                VideoNote::Failed(msg)
            } else if self.hooks.video.as_ref().unwrap().ready() == Some(false) {
                VideoNote::NotReady
            } else {
                VideoNote::Recording
            };
        }

        self.start_groups(&run_dir);
        self.publish_status();
        self.spawn_param_dump(&run_dir);
        (true, run::started_message(&run_id, &note))
    }

    /// full (bag/), [camera (camera/)], [snapshot (snapshots/)]: a group that
    /// cannot start is logged and left out, as a `ros2 bag record` child
    /// that exits at once was.
    fn start_groups(&mut self, run_dir: &Path) {
        // a group left from an earlier run (alive only if its full group
        // died) is finalized first rather than orphaned
        for g in std::mem::take(&mut self.groups) {
            self.hooks.sources.stop(g.spec.name);
            drop(g.rec);
        }
        let qos_path = PathBuf::from(&self.cfg.qos_overrides_path);
        let qos = (!self.cfg.qos_overrides_path.is_empty() && qos_path.is_file()).then_some(qos_path.as_path());
        let specs = match profile::recorder_specs(&self.profile, run_dir, qos, &self.cfg.library) {
            Ok(s) => s,
            Err(e) => {
                self.log(Level::Error, &format!("record profile: {e}"));
                return;
            }
        };
        let overrides = match qos {
            Some(p) => QosOverrides::load(p),
            None => Ok(QosOverrides::empty()),
        };
        for spec in specs {
            let started = (|| {
                let bag = spec.bag.clone()?;
                let ov = overrides.as_ref().map_err(|e| format!("qos overrides: {e}"))?;
                let mut o = recording::Options::new(spec.name, &spec.out_dir, bag);
                o.log = self.log_fn();
                let rec = Recording::start(o).map_err(|e| e.to_string())?;
                Ok::<_, String>((rec, ov))
            })();
            match started {
                Ok((rec, ov)) => {
                    if let Err(e) = self.hooks.sources.start(&spec, ov, rec.feed()) {
                        self.log(Level::Error, &format!("{} sources: {e}", spec.name));
                    }
                    self.log(Level::Info, &format!("{} record -> {} ({} topics)", spec.name, spec.out_dir.display(), spec.topics.len()));
                    self.groups.push(Group { spec, rec });
                }
                Err(e) => self.log(Level::Error, &format!("{} record -> {} failed: {e}", spec.name, spec.out_dir.display())),
            }
        }
    }

    /// `_set_video`: MediaMTX records the camera path into the current run,
    /// or stops; a failure is retried by the tick and logged once per text.
    fn set_video(&mut self, on: bool) -> (bool, String) {
        if !on && !self.video_on {
            return (true, "unchanged".into());
        }
        let Some(v) = self.hooks.video.as_ref() else { return (true, "unchanged".into()) };
        let body = if on {
            mediamtx::record_patch(&self.run_dir.as_deref().map(|p| p.to_string_lossy().into_owned()).unwrap_or_default())
        } else {
            mediamtx::stop_patch()
        };
        let (ok, msg) = v.patch(&body);
        let path = v.path().to_string();
        let word = if on { "on" } else { "off" };
        if ok {
            self.video_on = on;
            self.video_error = None;
            self.log(Level::Info, &format!("video {word}: {path}"));
        } else if self.video_error.as_deref() != Some(msg.as_str()) {
            self.video_error = Some(msg.clone());
            self.log(Level::Warn, &format!("video {word} failed ({path}): {msg}"));
        }
        (ok, msg)
    }

    /// `/mower_recorder/stop` (`_stop_recording_locked`): video off, every
    /// running group's sources stopped at once, then each group finalized
    /// (start order), the status, the reply.
    pub fn stop(&mut self) -> (bool, String) {
        if self.video_on {
            self.set_video(false);
        }
        let groups = std::mem::take(&mut self.groups);
        let (running, dead): (Vec<Group>, Vec<Group>) = groups.into_iter().partition(|g| g.rec.is_alive());
        for g in running.iter().chain(&dead) {
            self.hooks.sources.stop(g.spec.name);
        }
        for g in &running {
            g.rec.close_input();
        }
        let mut stopped = Vec::new();
        for g in running {
            if let Err(e) = g.rec.stop() {
                self.log(Level::Error, &format!("{}: {e}", g.spec.name));
            }
            stopped.push(g.spec.name);
        }
        drop(dead);
        self.start_time = None;
        self.publish_status();
        let (ok, msg) = run::stop_message(&stopped, self.run_id.as_deref().unwrap_or("None"));
        (ok, msg)
    }

    /// The 2 s tick (`_publish_status`).
    pub fn tick(&mut self) {
        self.publish_status();
    }

    /// `_publish_status`: the disk guard (a run stops with `stop_reason:
    /// disk_low`), the video-off retry, the status and the rear light.
    fn publish_status(&mut self) {
        let recording = self.recording();
        if recording {
            if let Some(dir) = self.run_dir.clone() {
                if self.hooks.disk.low(&dir, self.cfg.min_free_mb) {
                    let id = self.run_id.clone().unwrap_or_default();
                    self.log(Level::Error, &format!("under {} MB free: stopping run {id}", self.cfg.min_free_mb));
                    self.stop_reason = Some("disk_low".into());
                    self.stop();
                    return;
                }
            }
        }
        if !recording && self.video_on {
            self.set_video(false);
        }
        let (mut elapsed, mut bag_bytes) = (0.0, 0);
        if let (true, Some(start)) = (recording, self.start_time) {
            elapsed = seconds_between(&start, &self.hooks.clock.now());
            bag_bytes = self.run_dir.as_deref().map_or(0, run::bag_bytes);
        }
        let st = Status {
            recording,
            run_id: self.run_id.clone(),
            start_time: self.start_time,
            elapsed_s: elapsed,
            bag_bytes,
            num_topics: run::num_topics(&self.profile),
            profile: run::profile_name(&self.profile),
            video: self.video_on,
            stop_reason: self.stop_reason.clone(),
        };
        self.hooks.out.status(&st.to_json());
        if self.cfg.rear_light {
            self.hooks.out.rear_light(&run::rear_light_json(recording));
        }
    }

    /// `_trigger_snapshot`: flushes the snapshot group's buffer.
    pub fn trigger_snapshot(&mut self) -> bool {
        let Some(g) = self.groups.iter().find(|g| g.spec.name == "snapshot" && g.rec.is_alive()) else {
            self.log(Level::Warn, "no snapshot recorder running");
            return false;
        };
        match g.rec.take_snapshot() {
            Ok(_) => true,
            Err(e) => {
                self.log(Level::Error, &format!("snapshot trigger failed: {e}"));
                false
            }
        }
    }

    /// `/mower_recorder/snapshot`
    pub fn snapshot(&mut self) -> (bool, String) {
        let ok = self.trigger_snapshot();
        (ok, run::snapshot_message(ok).into())
    }

    /// `/mower_recorder/fault` (`std_msgs/Bool`)
    pub fn on_fault(&mut self, data: bool) {
        if data {
            self.log(Level::Warn, "fault received -> flushing snapshot buffer");
            self.trigger_snapshot();
        }
    }

    /// `/fix`: the last one is the run's `gps_start`.
    pub fn on_fix(&mut self, fix: Fix) {
        self.last_fix = Some(fix);
    }

    /// `_write_metadata`
    fn write_metadata(&mut self, run_id: &str, run_dir: &Path) {
        let start = self.hooks.clock.now();
        let git = git_info(&expand_user(&self.cfg.git_repo_dir));
        let mut meta = run::base_metadata(run_id, &self.cfg.robot_id, &start, &self.cfg.ros_distro, git, self.last_fix, &self.profile);
        if let Some(p) = self.profile.get("profile").filter(|p| truthy(Some(p))) {
            let p = py_str(p);
            match self.profile_metadata(meta, &p, run_dir) {
                Ok(m) => meta = m,
                Err(e) => {
                    // the Python raised out of the start here (A9)
                    self.log(Level::Error, &format!("write metadata failed: {e}"));
                    return;
                }
            }
        }
        let path = run_dir.join("run_metadata.yaml");
        match std::fs::write(&path, run::metadata_yaml(&meta)) {
            Ok(()) => self.log(Level::Info, &format!("metadata -> {}", path.display())),
            Err(e) => self.log(Level::Error, &format!("write metadata failed: {e}")),
        }
    }

    /// `_profile_metadata`: the spec 1.6 fields and `calib/`; a problem is
    /// logged and never blocks the recording.
    fn profile_metadata(&self, meta: Py, profile_name: &str, run_dir: &Path) -> Result<Py, String> {
        let dc = &self.cfg.data_collection;
        let safe = |what: &str, r: Result<Py, String>| match r {
            Ok(v) => v,
            Err(e) => {
                self.log(Level::Error, &format!("{what}: {e}"));
                Py::Dict(Vec::new())
            }
        };
        let session = safe("session_file", profile::load_yaml_file(Path::new(&expand_user(&dc.session_file))));
        let run_info = safe("run_info", profile::parse_mapping(&dc.run_info, "run_info"));
        let camera = safe("camera_params", profile::json_param(&dc.camera_params, "camera_params"));
        let derived = safe("extrinsics_derived", profile::json_param(&dc.extrinsics_derived, "extrinsics_derived"));
        let load = |t: &str| yaml::safe_load(t).map_err(|e| e.to_string());
        let calib = safe(
            "calib copy",
            dc_metadata::copy_calibration(
                run_dir,
                &expand_user(&dc.intrinsics_file),
                &expand_user(&dc.extrinsics_file),
                derived.get("block"),
                dc.intrinsics_placeholder,
                &load,
            ),
        );
        if let Some(Py::List(problems)) = calib.get("problems") {
            for p in problems {
                self.log(Level::Warn, &format!("calib: {}", py_str(p)));
            }
        }
        let cam_rec = self.profile.get("camera_recorder");
        let camera_topics = cam_rec.filter(|c| truthy(c.get("enabled"))).map(|c| str_list(c.get("topics")));
        dc_metadata::build_run_metadata(
            &meta,
            profile_name,
            &MetadataInputs {
                session: Some(&session),
                run_info: Some(&run_info),
                camera_params: Some(&camera),
                extrinsics: derived.get("summary"),
                calibration: Some(&calib),
                camera_topics: camera_topics.as_deref(),
            },
            &*self.cfg.env,
        )
    }

    /// `_dump_params`, on its own thread: `params/<node>.yaml` per node, as
    /// `ros2 param dump` printed it.
    fn spawn_param_dump(&mut self, run_dir: &Path) {
        let nodes: Vec<String> = self.cfg.params_dump_nodes.iter().filter(|n| !n.is_empty()).cloned().collect();
        if nodes.is_empty() {
            return;
        }
        let pdir = run_dir.join("params");
        let src = self.hooks.params.clone();
        let out = self.hooks.out.clone();
        let spawned = std::thread::Builder::new().name("param-dump".into()).spawn(move || {
            if let Err(e) = std::fs::create_dir_all(&pdir) {
                out.log(Level::Warn, &format!("param dump: {}: {e}", pdir.display()));
                return;
            }
            for node in nodes {
                // `ros2 param dump` names the node absolutely
                let full = if node.starts_with('/') { node.clone() } else { format!("/{node}") };
                let r = src
                    .parameters(&full)
                    .and_then(|ps| params::dump(&full, &ps))
                    .and_then(|text| std::fs::write(pdir.join(run::params_file_name(&node)), text).map_err(|e| e.to_string()));
                if let Err(e) = r {
                    out.log(Level::Warn, &format!("param dump {node} failed: {e}"));
                }
            }
        });
        match spawned {
            Ok(h) => {
                self.dumps.retain(|h| !h.is_finished());
                self.dumps.push(h);
            }
            Err(e) => self.log(Level::Warn, &format!("param dump: {e}")),
        }
    }

    /// Waits for the parameter dumps started so far.
    pub fn join_param_dumps(&mut self) {
        for h in self.dumps.drain(..) {
            let _ = h.join();
        }
    }
}

/// `(b - a).total_seconds()` of two naive datetimes: whole microseconds,
/// then a true division by 10**6.
pub fn seconds_between(a: &LocalTime, b: &LocalTime) -> f64 {
    (micros(b) - micros(a)) as f64 / 1e6
}

fn micros(t: &LocalTime) -> i64 {
    // days from the civil date (Howard Hinnant's days_from_civil)
    let (y, m, d) = (t.year as i64 - if t.month <= 2 { 1 } else { 0 }, t.month as i64, t.day as i64);
    let era = y.div_euclid(400);
    let yoe = y.rem_euclid(400);
    let doy = (153 * (if m > 2 { m - 3 } else { m + 9 }) + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    let days = era * 146_097 + doe - 719_468;
    ((days * 86_400 + t.hour as i64 * 3600 + t.minute as i64 * 60 + t.second as i64) * 1_000_000) + t.microsecond as i64
}

/// `_git_info`: `{sha, describe}` of `git_repo_dir`, `{sha: unknown}`
/// without one or when git fails.
fn git_info(dir: &str) -> Py {
    if dir.is_empty() {
        return run::unknown_git();
    }
    let git = |args: &[&str]| -> Option<String> {
        let o = std::process::Command::new("git").arg("-C").arg(dir).args(args).stderr(std::process::Stdio::null()).output().ok()?;
        o.status.success().then(|| String::from_utf8_lossy(&o.stdout).trim().to_string())
    };
    let Some(sha) = git(&["rev-parse", "HEAD"]) else { return run::unknown_git() };
    let Some(desc) = git(&["describe", "--always", "--dirty", "--tags"]) else { return run::unknown_git() };
    Py::dict([("sha", Py::Str(sha)), ("describe", Py::Str(desc))])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn total_seconds_of_naive_datetimes() {
        let t = |d, h, mi, s, us| LocalTime { year: 2026, month: 10, day: d, hour: h, minute: mi, second: s, microsecond: us, utc_offset: 0 };
        assert_eq!(seconds_between(&t(30, 8, 18, 51, 15536), &t(30, 8, 18, 53, 115536)), 2.1);
        assert_eq!(seconds_between(&t(31, 0, 0, 0, 0), &t(30, 23, 59, 59, 500000)), -0.5);
        let a = LocalTime { year: 2024, month: 2, day: 28, ..t(1, 0, 0, 0, 0) };
        let b = LocalTime { year: 2024, month: 3, day: 1, ..t(1, 0, 0, 0, 0) };
        assert_eq!(seconds_between(&a, &b), 2.0 * 86400.0);
    }
}
