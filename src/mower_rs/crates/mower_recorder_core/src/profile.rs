//! Recording profiles (mower_recorder `record_profile.py`): the profile YAML
//! merged over the defaults, and per group (full, camera, snapshot) what it
//! records and how: the `ros2 bag record` command line the Python recorder
//! runs (pinned by test_record_profile.py, so the A/B side A keeps it) and the
//! same as `mower_mcap::BagOptions` for the Rust writer.
//!
//! The names a group records are the profile's minus the hidden ones
//! (`/navigate_to_pose/_action/*`): rosbag2 skips a hidden name without
//! `--include-hidden-topics`, and so does the Rust recorder (owner decision
//! 2026-10-30; design 1.12 #8), and `num_topics` leaves them out (2026-10-31).

use std::path::{Path, PathBuf};

use mower_mcap::{BagOptions, Preset};
use mower_pyfmt::Py;

pub const FULL_RECORDER_NODE: &str = "mower_full_recorder";
pub const CAMERA_RECORDER_NODE: &str = "mower_camera_recorder";
pub const SNAPSHOT_RECORDER_NODE: &str = "mower_snapshot_recorder";
/// The groups whose processes mean "this run is being recorded".
pub const DATA_RECORDERS: [&str; 2] = ["full", "camera"];

/// `DEFAULT_CONFIG`
pub fn default_config() -> Py {
    Py::dict([
        ("topics", Py::List(Vec::new())),
        ("snapshot_topics", Py::List(Vec::new())),
        ("compression", Py::Bool(true)),
        ("max_bag_size_mb", Py::Int(512)),
        ("max_bag_duration_s", Py::Int(3600)),
        ("enable_snapshot", Py::Bool(true)),
        ("snapshot_max_cache_mb", Py::Int(256)),
    ])
}

/// `DEFAULT_CAMERA_RECORDER`
pub fn default_camera_recorder() -> Py {
    Py::dict([
        ("enabled", Py::Bool(false)),
        ("topics", Py::List(Vec::new())),
        ("max_bag_size_mb", Py::Int(256)),
        ("max_bag_duration_s", Py::Int(0)),
        ("storage_preset_profile", Py::str("")),
    ])
}

/// `dict.update`: existing keys keep their place, new ones go last.
pub fn dict_update(base: &mut Py, extra: &Py) {
    let (Py::Dict(b), Py::Dict(e)) = (base, extra) else { return };
    for (k, v) in e {
        match b.iter_mut().find(|(bk, _)| bk == k) {
            Some(slot) => slot.1 = v.clone(),
            None => b.push((k.clone(), v.clone())),
        }
    }
}

pub fn dict_set(d: &mut Py, key: &str, value: Py) {
    dict_update(d, &Py::dict([(key, value)]));
}

/// Python truthiness.
pub fn truthy(v: Option<&Py>) -> bool {
    match v {
        None | Some(Py::None) | Some(Py::Bool(false)) | Some(Py::Int(0)) => false,
        Some(Py::Float(f)) => *f != 0.0,
        Some(Py::Str(s)) => !s.is_empty(),
        Some(Py::List(l)) => !l.is_empty(),
        Some(Py::Dict(d)) => !d.is_empty(),
        _ => true,
    }
}

/// `int(v)` for a profile number (an int, a float, a bool or a numeric string).
pub fn py_int(v: Option<&Py>) -> Result<i64, String> {
    match v {
        Some(Py::Int(i)) => Ok(*i),
        Some(Py::Bool(b)) => Ok(*b as i64),
        Some(Py::Float(f)) if f.is_finite() => Ok(f.trunc() as i64),
        Some(Py::Str(s)) => s.trim().replace('_', "").parse().map_err(|_| format!("invalid literal for int() with base 10: {s:?}")),
        Some(Py::None) | None => Err("int() argument must be a string, a bytes-like object or a real number, not 'NoneType'".into()),
        Some(o) => Err(format!("int() argument must be a string, a bytes-like object or a real number, not '{}'", o.type_name())),
    }
}

/// The strings of a list (anything else in it as Python's `str()` would not
/// be a topic name: left out).
pub fn str_list(v: Option<&Py>) -> Vec<String> {
    match v {
        Some(Py::List(l)) => l.iter().filter_map(|t| if let Py::Str(s) = t { Some(s.clone()) } else { None }).collect(),
        _ => Vec::new(),
    }
}

/// What `load_config` logged.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConfigLog {
    Warn(String),
    Error(String),
}

/// `load_config(path)`: the file merged over the defaults (a top-level
/// `dict.update`), then `camera_recorder` over its own defaults.
pub fn load_config(path: Option<&Path>, log: &mut Vec<ConfigLog>) -> Py {
    let mut cfg = default_config();
    match path {
        Some(p) if p.is_file() => {
            let r = std::fs::read_to_string(p).map_err(|e| e.to_string()).and_then(|t| {
                mower_pyfmt::yaml::safe_load(&t).map_err(|e| e.to_string())
            });
            match r {
                Ok(v) if !truthy(Some(&v)) => {}
                Ok(v @ Py::Dict(_)) => dict_update(&mut cfg, &v),
                // dict.update of a non-mapping
                Ok(v) => log.push(ConfigLog::Error(format!(
                    "load config failed: cannot convert dictionary update sequence element #0 to a sequence ({})",
                    v.type_name()
                ))),
                Err(e) => log.push(ConfigLog::Error(format!("load config failed: {e}"))),
            }
        }
        other => {
            let shown = other.map(|p| p.to_string_lossy().into_owned()).unwrap_or_default();
            log.push(ConfigLog::Warn(format!("config_file not found: {}", py_repr_str(&shown))))
        }
    }
    let mut cam = default_camera_recorder();
    if let Some(c @ Py::Dict(_)) = cfg.get("camera_recorder") {
        let c = c.clone();
        dict_update(&mut cam, &c);
    }
    dict_set(&mut cfg, "camera_recorder", cam);
    cfg
}

/// `repr(str)` for the plain paths a config log shows.
fn py_repr_str(s: &str) -> String {
    if s.contains('\'') && !s.contains('"') {
        format!("\"{s}\"")
    } else {
        format!("'{}'", s.replace('\\', "\\\\").replace('\'', "\\'"))
    }
}

/// rosbag2's `topic_is_hidden`: a token of the name starts with `_`.
pub fn is_hidden(topic: &str) -> bool {
    topic.split('/').any(|t| t.starts_with('_'))
}

/// One group of a run: what it records and how.
#[derive(Debug, Clone)]
pub struct GroupSpec {
    /// `full`, `camera` or `snapshot`
    pub name: &'static str,
    pub node_name: &'static str,
    /// `<run>/bag`, `<run>/camera` or `<run>/snapshots`
    pub out_dir: PathBuf,
    /// The profile's names, as listed (hidden ones included).
    pub topics: Vec<String>,
    /// What is recorded: the hidden names left out.
    pub recorded: Vec<String>,
    /// The Python recorder's `ros2 bag record` command line.
    pub cmd: Vec<String>,
    /// The same for mower_mcap (`Err` for a storage preset it does not have).
    pub bag: Result<BagOptions, String>,
}

fn preset_of(cfg: &Py) -> String {
    let v = cfg.get("storage_preset_profile");
    if !truthy(v) {
        return String::new();
    }
    match v.unwrap() {
        Py::Str(s) => s.trim().to_string(),
        other => mower_pyfmt::json::dumps(other, &Default::default()).unwrap_or_default(),
    }
}

/// `_compression_args`
fn compression_args(cfg: &Py) -> Vec<String> {
    let preset = preset_of(cfg);
    if !preset.is_empty() && preset.to_lowercase() != "none" {
        return vec!["--storage-preset-profile".into(), preset];
    }
    if cfg.get("compression").map_or(true, |v| truthy(Some(v))) {
        return ["--compression-mode", "message", "--compression-format", "zstd"].map(String::from).to_vec();
    }
    Vec::new()
}

fn bag_options(args: &[String], max_size: u64, max_duration: u64, snapshot: Option<u64>, library: &str) -> Result<BagOptions, String> {
    let mut o = BagOptions::new(library);
    o.max_bagfile_size = max_size;
    o.max_bagfile_duration = max_duration;
    o.snapshot_cache_size = snapshot;
    if let Some(i) = args.iter().position(|a| a == "--storage-preset-profile") {
        match args[i + 1].as_str() {
            "zstd_fast" => o.preset = Preset::ZstdFast,
            "none" => {}
            other => return Err(format!("storage preset {other:?} is not supported by mower_mcap")),
        }
    }
    o.message_compression = args.iter().any(|a| a == "--compression-mode");
    Ok(o)
}

/// `recorder_specs(cfg, run_dir, qos_path)`: the groups in start order.
/// `library` is the MCAP header's (D1).
pub fn recorder_specs(cfg: &Py, run_dir: &Path, qos_path: Option<&Path>, library: &str) -> Result<Vec<GroupSpec>, String> {
    let qos: Vec<String> = match qos_path {
        Some(q) if !q.as_os_str().is_empty() && q.is_file() => {
            vec!["--qos-profile-overrides-path".into(), q.to_string_lossy().into_owned()]
        }
        _ => Vec::new(),
    };
    let join = |d: &str| run_dir.join(d);
    let s = |p: &Path| p.to_string_lossy().into_owned();
    let mut specs = Vec::new();

    let topics = str_list(cfg.get("topics"));
    let bag_dir = join("bag");
    let comp = compression_args(cfg);
    let size = py_int(cfg.get("max_bag_size_mb"))? * 1024 * 1024;
    let dur = py_int(cfg.get("max_bag_duration_s"))?;
    let mut cmd: Vec<String> =
        ["ros2", "bag", "record", "-s", "mcap", "-o"].map(String::from).to_vec();
    cmd.extend([s(&bag_dir), "--node-name".into(), FULL_RECORDER_NODE.into()]);
    cmd.extend(comp.iter().cloned());
    cmd.extend(["--max-bag-size".into(), size.to_string(), "--max-bag-duration".into(), dur.to_string()]);
    cmd.extend(qos.iter().cloned());
    cmd.extend(topics.iter().cloned());
    specs.push(GroupSpec {
        name: "full",
        node_name: FULL_RECORDER_NODE,
        out_dir: bag_dir,
        recorded: topics.iter().filter(|t| !is_hidden(t)).cloned().collect(),
        topics,
        bag: bag_options(&comp, size.max(0) as u64, dur.max(0) as u64, None, library),
        cmd,
    });

    let cam = cfg.get("camera_recorder").cloned().unwrap_or(Py::Dict(Vec::new()));
    if truthy(cam.get("enabled")) && truthy(cam.get("topics")) {
        let cam_topics = str_list(cam.get("topics"));
        let cam_dir = join("camera");
        let mut ccfg = Py::dict([("compression", Py::Bool(false))]);
        dict_set(&mut ccfg, "storage_preset_profile", cam.get("storage_preset_profile").cloned().unwrap_or(Py::str("")));
        let ccomp = compression_args(&ccfg);
        let mut ccmd: Vec<String> = ["ros2", "bag", "record", "-s", "mcap", "-o"].map(String::from).to_vec();
        ccmd.extend([s(&cam_dir), "--node-name".into(), CAMERA_RECORDER_NODE.into()]);
        ccmd.extend(ccomp.iter().cloned());
        let csize = if truthy(cam.get("max_bag_size_mb")) { py_int(cam.get("max_bag_size_mb"))? } else { 0 };
        let cdur = if truthy(cam.get("max_bag_duration_s")) { py_int(cam.get("max_bag_duration_s"))? } else { 0 };
        if csize > 0 {
            ccmd.extend(["--max-bag-size".into(), (csize * 1024 * 1024).to_string()]);
        }
        if cdur > 0 {
            ccmd.extend(["--max-bag-duration".into(), cdur.to_string()]);
        }
        ccmd.extend(qos.iter().cloned());
        ccmd.extend(cam_topics.iter().cloned());
        specs.push(GroupSpec {
            name: "camera",
            node_name: CAMERA_RECORDER_NODE,
            out_dir: cam_dir,
            recorded: cam_topics.iter().filter(|t| !is_hidden(t)).cloned().collect(),
            topics: cam_topics,
            bag: bag_options(&ccomp, (csize.max(0) * 1024 * 1024) as u64, cdur.max(0) as u64, None, library),
            cmd: ccmd,
        });
    }

    if cfg.get("enable_snapshot").map_or(true, |v| truthy(Some(v))) && truthy(cfg.get("snapshot_topics")) {
        let snap_topics = str_list(cfg.get("snapshot_topics"));
        let snap_dir = join("snapshots");
        let cache = py_int(cfg.get("snapshot_max_cache_mb"))? * 1024 * 1024;
        let mut scmd: Vec<String> = ["ros2", "bag", "record", "-s", "mcap", "-o"].map(String::from).to_vec();
        scmd.extend([s(&snap_dir), "--node-name".into(), SNAPSHOT_RECORDER_NODE.into(), "--snapshot-mode".into()]);
        scmd.extend(["--max-cache-size".into(), cache.to_string()]);
        scmd.extend(qos.iter().cloned());
        scmd.extend(snap_topics.iter().cloned());
        specs.push(GroupSpec {
            name: "snapshot",
            node_name: SNAPSHOT_RECORDER_NODE,
            out_dir: snap_dir,
            recorded: snap_topics.iter().filter(|t| !is_hidden(t)).cloned().collect(),
            topics: snap_topics,
            bag: bag_options(&[], 0, 0, Some(cache.max(0) as u64), library),
            cmd: scmd,
        });
    }
    Ok(specs)
}

/// `disk_low(path, min_free_mb)`: under `min_free_mb` MB free on the file
/// system holding `path` (its nearest existing parent); 0 = never.
pub fn disk_low(path: &Path, min_free_mb: i64) -> bool {
    if min_free_mb <= 0 {
        return false;
    }
    let mut probe = std::path::absolute(path).unwrap_or_else(|_| path.to_path_buf());
    while !probe.exists() {
        match probe.parent() {
            Some(p) if p != probe => probe = p.to_path_buf(),
            _ => break,
        }
    }
    match free_bytes(&probe) {
        Some(free) => (free as i128) < min_free_mb as i128 * 1024 * 1024,
        // shutil.disk_usage raises: the Python tick would fail; not low
        None => false,
    }
}

/// `shutil.disk_usage(p).free`: `f_bavail * f_frsize`.
pub fn free_bytes(p: &Path) -> Option<u64> {
    use std::os::unix::ffi::OsStrExt;
    let c = std::ffi::CString::new(p.as_os_str().as_bytes()).ok()?;
    let mut st: libc::statvfs = unsafe { std::mem::zeroed() };
    if unsafe { libc::statvfs(c.as_ptr(), &mut st) } != 0 {
        return None;
    }
    Some(st.f_bavail as u64 * st.f_frsize as u64)
}

/// `parse_mapping(text, what)`: a YAML/JSON mapping string ('' → {}).
pub fn parse_mapping(text: &str, what: &str) -> Result<Py, String> {
    let t = text.trim();
    if t.is_empty() {
        return Ok(Py::Dict(Vec::new()));
    }
    match mower_pyfmt::yaml::safe_load(t).map_err(|e| e.to_string())? {
        Py::None => Ok(Py::Dict(Vec::new())),
        d @ Py::Dict(_) => Ok(d),
        other => Err(format!("{what} must be a mapping, got {}", other.type_name())),
    }
}

/// `load_yaml_file(path)`: {} when there is no such file.
pub fn load_yaml_file(path: &Path) -> Result<Py, String> {
    if path.as_os_str().is_empty() || !path.is_file() {
        return Ok(Py::Dict(Vec::new()));
    }
    let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    match mower_pyfmt::yaml::safe_load(&text).map_err(|e| e.to_string())? {
        v if !truthy(Some(&v)) => Ok(Py::Dict(Vec::new())),
        d @ Py::Dict(_) => Ok(d),
        _ => Err(format!("{}: top level must be a mapping", path.display())),
    }
}

/// `json_param(text, what)`: a launch parameter's JSON object ('' → {}).
pub fn json_param(text: &str, what: &str) -> Result<Py, String> {
    if text.is_empty() {
        return Ok(Py::Dict(Vec::new()));
    }
    match mower_pyfmt::json::loads(text)? {
        d @ Py::Dict(_) => Ok(d),
        _ => Err(format!("{what} must be a JSON object")),
    }
}
