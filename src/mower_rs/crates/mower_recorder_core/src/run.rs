//! One run as recorder_manager makes it (`recorder_manager_node.py`): the
//! run id and directory, `run_metadata.yaml`, `/mower_recorder/status`, the
//! rear light, the texts of the start/stop/snapshot replies, and the
//! parameter dump's file name. Everything is the Python's, byte for byte.

use std::path::Path;

use mower_pyfmt::json::{self, JsonOptions};
use mower_pyfmt::yaml::{self, YamlOptions};
use mower_pyfmt::Py;

use crate::profile::{dict_set, str_list, truthy};
use crate::util::{dir_size, LocalTime};

/// `{robot_id}_{%Y%m%dT%H%M%S}` in the process's local time, no zone.
pub fn run_id(robot_id: &str, now: &LocalTime) -> String {
    format!("{robot_id}_{}", now.compact())
}

/// The last `/fix` as `gps_start` (`{lat, lon, alt}`), `None` before one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fix {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
}

/// `_write_metadata`'s dict for the default profile (a data_collection run
/// adds the spec 1.6 fields: `dc_metadata::build_run_metadata`). `git` is
/// `{sha: unknown}` without a git dir (production).
pub fn base_metadata(run_id: &str, robot_id: &str, start_time: &LocalTime, ros_distro: &str, git: Py, gps: Option<Fix>, cfg: &Py) -> Py {
    let gps = match gps {
        Some(f) => Py::dict([("lat", Py::Float(f.latitude)), ("lon", Py::Float(f.longitude)), ("alt", Py::Float(f.altitude))]),
        None => Py::None,
    };
    let list = |k: &str| Py::List(str_list(cfg.get(k)).into_iter().map(Py::Str).collect());
    Py::dict([
        ("run_id", Py::str(run_id)),
        ("robot_id", Py::str(robot_id)),
        ("start_time", Py::Str(start_time.isoformat())),
        ("ros_distro", Py::str(ros_distro)),
        ("git", git),
        ("gps_start", gps),
        ("recorded_topics", list("topics")),
        ("snapshot_topics", list("snapshot_topics")),
    ])
}

/// `{sha: unknown}`
pub fn unknown_git() -> Py {
    Py::dict([("sha", Py::str("unknown"))])
}

/// `yaml.safe_dump(meta, f, allow_unicode=True, sort_keys=False)`
pub fn metadata_yaml(meta: &Py) -> String {
    yaml::safe_dump(meta, &YamlOptions { allow_unicode: true, sort_keys: false })
}

/// The names the run records: the profile's topics, plus the camera
/// group's when it is on, without the hidden ones nobody records
/// (`/navigate_to_pose/_action/*`, design 1.12 #8). Owner decision
/// 2026-10-31: 30 for the default profile, where the Python recorder said
/// 32 (it counted the listed names); data_collection stays 19.
pub fn num_topics(cfg: &Py) -> i64 {
    let recorded = |v: Option<&Py>| str_list(v).iter().filter(|t| !crate::profile::is_hidden(t)).count();
    let cam_n = match cfg.get("camera_recorder") {
        Some(c) if truthy(c.get("enabled")) => recorded(c.get("topics")),
        _ => 0,
    };
    (recorded(cfg.get("topics")) + cam_n) as i64
}

/// `cfg.get('profile') or 'default'`
pub fn profile_name(cfg: &Py) -> Py {
    match cfg.get("profile") {
        Some(p) if truthy(Some(p)) => p.clone(),
        _ => Py::str("default"),
    }
}

/// What `/mower_recorder/status` shows.
#[derive(Debug, Clone, PartialEq)]
pub struct Status {
    pub recording: bool,
    pub run_id: Option<String>,
    pub start_time: Option<LocalTime>,
    /// `(now - start).total_seconds()`
    pub elapsed_s: f64,
    pub bag_bytes: u64,
    pub num_topics: i64,
    pub profile: Py,
    pub video: bool,
    pub stop_reason: Option<String>,
}

impl Status {
    pub fn idle(cfg: &Py, stop_reason: Option<String>) -> Status {
        Status {
            recording: false,
            run_id: None,
            start_time: None,
            elapsed_s: 0.0,
            bag_bytes: 0,
            num_topics: num_topics(cfg),
            profile: profile_name(cfg),
            video: false,
            stop_reason,
        }
    }

    /// `json.dumps(st, ensure_ascii=False)` (rm:288-303).
    pub fn to_json(&self) -> String {
        let opt = |s: &Option<String>| s.clone().map(Py::Str).unwrap_or(Py::None);
        let st = Py::dict([
            ("recording", Py::Bool(self.recording)),
            ("run_id", if self.recording { opt(&self.run_id) } else { Py::None }),
            (
                "start_time",
                match (&self.start_time, self.recording) {
                    (Some(t), true) => Py::Str(t.isoformat()),
                    _ => Py::None,
                },
            ),
            ("elapsed_s", Py::Float(crate::extrinsics::py_round(self.elapsed_s, 1))),
            ("bag_bytes", Py::Int(self.bag_bytes as i64)),
            ("num_topics", Py::Int(self.num_topics)),
            ("profile", self.profile.clone()),
            ("video", Py::Bool(self.recording && self.video)),
            ("stop_reason", opt(&self.stop_reason)),
        ]);
        json::dumps(&st, &JsonOptions::unicode()).expect("no dates")
    }
}

/// `bag/`, `camera/` and `video/` of a run, while it records.
pub fn bag_bytes(run_dir: &Path) -> u64 {
    ["bag", "camera", crate::mediamtx::VIDEO_DIR].iter().map(|d| dir_size(&run_dir.join(d))).sum()
}

/// `/mower_base/rear_light` (rm:305-308; default `json.dumps`).
pub fn rear_light_json(recording: bool) -> String {
    let v = Py::dict([("effect", Py::str(if recording { "recording" } else { "off" })), ("source", Py::str("bag"))]);
    json::dumps(&v, &JsonOptions::default()).expect("no dates")
}

/// The start reply's texts (rm:180-221).
pub const ALREADY_RECORDING: &str = "已在錄製中";
pub const NO_TOPICS: &str = "record_topics.yaml 沒有指定 topics";
pub fn disk_low_message(min_free_mb: i64) -> String {
    format!("磁碟剩不到 {min_free_mb} MB，不能開始錄製")
}

/// What the video setup reported at the start.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum VideoNote {
    /// No `video_api_url`.
    None,
    /// The PATCH failed with this message.
    Failed(String),
    /// The camera path has no stream now.
    NotReady,
    Recording,
}

pub fn started_message(run_id: &str, video: &VideoNote) -> String {
    let note = match video {
        VideoNote::None => String::new(),
        VideoNote::Failed(m) => format!("（前鏡頭影像沒有錄：{m}）"),
        VideoNote::NotReady => "（前鏡頭目前沒有畫面，有畫面時才會錄進去）".into(),
        VideoNote::Recording => "，含前鏡頭影像".into(),
    };
    format!("開始錄製 run={run_id}{note}")
}

/// The stop reply (rm:266-268): the groups stopped, in start order.
pub fn stop_message(stopped: &[&str], run_id: &str) -> (bool, String) {
    if stopped.is_empty() {
        (false, "目前沒有在錄製".into())
    } else {
        (true, format!("已停止並封存: {} (run={run_id})", stopped.join(", ")))
    }
}

pub fn snapshot_message(ok: bool) -> &'static str {
    if ok { "已觸發 snapshot" } else { "snapshot 觸發失敗" }
}

/// `params/<name>.yaml` of a dumped node: `node.strip('/').replace('/', '__') or 'node'`.
pub fn params_file_name(node: &str) -> String {
    let s = node.trim_matches('/').replace('/', "__");
    format!("{}.yaml", if s.is_empty() { "node" } else { &s })
}

/// Sets `display_name` on a run's metadata (bag_store `_rename`).
pub fn with_display_name(meta: &Py, name: &str) -> Py {
    let mut m = if matches!(meta, Py::Dict(_)) { meta.clone() } else { Py::Dict(Vec::new()) };
    dict_set(&mut m, "display_name", Py::str(name));
    m
}
