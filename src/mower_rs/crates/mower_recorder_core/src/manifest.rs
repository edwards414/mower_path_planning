//! A run's upload to R2 (mower_recorder `run_manifest.py`, line for line):
//! every file of the run first, `_manifest.json` strictly last; a resumable
//! `.upload_state.json`; a HEAD of every object before the manifest; the
//! `.uploaded` marker; deleting a run manifest first. The files it writes are
//! byte for byte the Python's (`json.dumps` through mower_pyfmt), so a run
//! half uploaded by one uploader is finished by the other (design G5).

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::Path;

use mower_pyfmt::json::{self, JsonOptions};
use mower_pyfmt::Py;

use crate::s3::{S3Error, S3};
use crate::util::{atomic_write, fromtimestamp_seconds, sha256_file, sha256_hex, LocalTime};

pub const MANIFEST_NAME: &str = "_manifest.json";
pub const MANIFEST_SCHEMA: &str = "mower.run_manifest/v1";
pub const UPLOADED_MARKER: &str = ".uploaded";
pub const UPLOAD_STATE: &str = ".upload_state.json";
pub const BAG_METADATA: &str = "metadata.yaml";
pub const STATE_VERSION: i64 = 1;

/// What can stop an upload, with the text bag_store shows as `upload_error`
/// ([`UploadError::upload_error`]).
#[derive(Debug, Clone, PartialEq)]
pub enum UploadError {
    /// `RunNotReady`: a bag is not finalized; nothing was sent.
    NotReady(String),
    /// boto3's `S3UploadFailedError` (an `upload_file` that failed).
    UploadFailed { path: String, bucket: String, key: String, error: S3Error },
    /// botocore's `ClientError` (any other request).
    Client(S3Error),
    /// A local file could not be read or written (`OSError`).
    Io(String),
    /// `run_metadata.yaml` or a `metadata.yaml` did not parse.
    Parse(String),
    /// `RuntimeError` (the size still differs after a re-upload).
    Runtime(String),
    /// `TypeError`: the manifest holds a value JSON cannot (a date).
    NotSerializable(String),
}

impl std::fmt::Display for UploadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            UploadError::NotReady(m) | UploadError::Io(m) | UploadError::Parse(m) | UploadError::Runtime(m) | UploadError::NotSerializable(m) => {
                f.write_str(m)
            }
            UploadError::UploadFailed { path, bucket, key, error } => write!(f, "Failed to upload {path} to {bucket}/{key}: {error}"),
            UploadError::Client(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for UploadError {}

impl UploadError {
    /// The Python exception's class name.
    pub fn type_name(&self) -> &'static str {
        match self {
            UploadError::NotReady(_) => "RunNotReady",
            UploadError::UploadFailed { .. } => "S3UploadFailedError",
            UploadError::Client(e) if e.status.is_none() => "EndpointConnectionError",
            UploadError::Client(_) => "ClientError",
            UploadError::Io(_) => "OSError",
            UploadError::Parse(_) => "YAMLError",
            UploadError::Runtime(_) => "RuntimeError",
            UploadError::NotSerializable(_) => "TypeError",
        }
    }

    /// bag_store's `upload_error` (bs:315-324): the readiness text alone for
    /// `RunNotReady`, `"<Type>: <message>"` for anything else.
    pub fn upload_error(&self) -> String {
        match self {
            UploadError::NotReady(m) => m.clone(),
            e => format!("{}: {e}", e.type_name()),
        }
    }
}

fn io_err(e: std::io::Error) -> UploadError {
    UploadError::Io(e.to_string())
}

// ---- the run on disk ------------------------------------------------------------

/// Relative POSIX paths of every file that goes to R2, sorted: dot files, dot
/// directories and `_manifest.json` left out (`list_run_files`).
pub fn list_run_files(run_dir: &Path) -> Vec<String> {
    fn walk(dir: &Path, rel: &str, out: &mut Vec<String>) {
        let Ok(rd) = fs::read_dir(dir) else { return };
        let mut entries: Vec<_> = rd.flatten().collect();
        entries.sort_by_key(|e| e.file_name());
        for e in entries {
            let name = e.file_name().to_string_lossy().into_owned();
            if name.starts_with('.') {
                continue;
            }
            let r = if rel.is_empty() { name.clone() } else { format!("{rel}/{name}") };
            // os.walk lists a symlink to a directory without entering it
            match e.file_type() {
                Ok(t) if t.is_dir() => walk(&e.path(), &r, out),
                Ok(t) if t.is_symlink() && e.path().is_dir() => {}
                Ok(_) => out.push(r),
                Err(_) => {}
            }
        }
    }
    let mut out = Vec::new();
    walk(run_dir, "", &mut out);
    out.retain(|r| r != MANIFEST_NAME);
    out.sort();
    out
}

/// Directories (relative, `.` for the run dir) holding rosbag2 storage files.
pub fn bag_dirs<'a>(files: impl IntoIterator<Item = &'a String>) -> Vec<String> {
    let mut dirs = BTreeSet::new();
    for rel in files {
        if [".mcap", ".db3", ".mcap.zstd", ".db3.zstd"].iter().any(|s| rel.ends_with(s)) {
            dirs.insert(match rel.rsplit_once('/') {
                Some((d, _)) => d.to_string(),
                None => ".".to_string(),
            });
        }
    }
    dirs.into_iter().collect()
}

/// What `read_bag_metadata` takes from a rosbag2 `metadata.yaml`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct BagInfo {
    pub start_ns: Option<i64>,
    pub duration_ns: Option<i64>,
    pub message_count: i64,
    /// name → (type, count), in the file's order
    pub topics: Vec<(String, Py, i64)>,
    pub relative_file_paths: Vec<Py>,
    pub ros_distro: String,
}

/// Python `int(v or 0)` for the values a metadata file holds.
fn py_int(v: Option<&Py>) -> Result<i64, String> {
    match v {
        None | Some(Py::None) => Ok(0),
        Some(Py::Bool(b)) => Ok(*b as i64),
        Some(Py::Int(i)) => Ok(*i),
        Some(Py::Float(f)) if f.is_finite() => Ok(f.trunc() as i64),
        Some(Py::Str(s)) if s.is_empty() => Ok(0),
        Some(Py::Str(s)) => s.trim().parse().map_err(|_| format!("invalid literal for int() with base 10: {s:?}")),
        Some(other) => Err(format!("int() argument must be a string or a number, not '{}'", other.type_name())),
    }
}

fn get<'a>(d: &'a Py, k: &str) -> Option<&'a Py> {
    match d.get(k) {
        Some(Py::None) => None,
        v => v,
    }
}

fn truthy(v: Option<&Py>) -> bool {
    match v {
        None | Some(Py::None) | Some(Py::Bool(false)) => false,
        Some(Py::Int(0)) => false,
        Some(Py::Float(f)) => *f != 0.0,
        Some(Py::Str(s)) => !s.is_empty(),
        Some(Py::List(l)) => !l.is_empty(),
        Some(Py::Dict(d)) => !d.is_empty(),
        _ => true,
    }
}

/// `str(v or '')`
fn py_str_or_empty(v: Option<&Py>) -> String {
    if !truthy(v) {
        return String::new();
    }
    match v.unwrap() {
        Py::Str(s) => s.clone(),
        Py::Int(i) => i.to_string(),
        Py::Float(f) => mower_pyfmt::repr::float(*f),
        Py::Bool(b) => if *b { "True".into() } else { "False".into() },
        other => other.isoformat().unwrap_or_default(),
    }
}

pub fn read_bag_metadata(path: &Path) -> Result<BagInfo, String> {
    let text = fs::read_to_string(path).map_err(|e| e.to_string())?;
    let doc = mower_pyfmt::yaml::safe_load(&text).map_err(|e| e.to_string())?;
    let empty = Py::Dict(Vec::new());
    let doc = if truthy(Some(&doc)) { doc } else { empty.clone() };
    let info = get(&doc, "rosbag2_bagfile_information").filter(|v| truthy(Some(v))).unwrap_or(&empty);
    let mut topics: Vec<(String, Py, i64)> = Vec::new();
    if let Some(Py::List(entries)) = get(info, "topics_with_message_count") {
        for entry in entries {
            let md = get(entry, "topic_metadata").filter(|v| truthy(Some(v))).unwrap_or(&empty);
            let Some(Py::Str(name)) = get(md, "name").filter(|v| truthy(Some(v))) else { continue };
            let ty = md.get("type").cloned().unwrap_or(Py::str(""));
            let count = py_int(get(entry, "message_count"))?;
            // a dict: a repeated name keeps its first place and its last value
            if let Some(t) = topics.iter_mut().find(|t| t.0 == *name) {
                t.1 = ty;
                t.2 = count;
            } else {
                topics.push((name.clone(), ty, count));
            }
        }
    }
    let ns = |section: &str, key: &str| -> Result<Option<i64>, String> {
        match get(info, section).and_then(|s| get(s, key)) {
            None => Ok(None),
            Some(v) => py_int(Some(v)).map(Some),
        }
    };
    Ok(BagInfo {
        start_ns: ns("starting_time", "nanoseconds_since_epoch")?,
        duration_ns: ns("duration", "nanoseconds")?,
        message_count: py_int(get(info, "message_count"))?,
        topics,
        relative_file_paths: match get(info, "relative_file_paths") {
            Some(Py::List(l)) => l.clone(),
            _ => Vec::new(),
        },
        ros_distro: py_str_or_empty(info.get("ros_distro")),
    })
}

/// What blocks an upload (`check_ready`); empty = ready. The texts are the
/// Python's (a parse error's own text differs, A9).
pub fn check_ready(run_dir: &Path, files: &[String]) -> Vec<String> {
    let mut problems = Vec::new();
    for d in bag_dirs(files) {
        let meta = run_dir.join(&d).join(BAG_METADATA);
        if !meta.is_file() {
            problems.push(format!("{d}/ 沒有 {BAG_METADATA}（錄製未正常結束？先執行 `ros2 bag reindex {d} -s mcap`）"));
            continue;
        }
        let info = match read_bag_metadata(&meta) {
            Ok(i) => i,
            Err(e) => {
                problems.push(format!("{d}/{BAG_METADATA} 無法解析：{e}"));
                continue;
            }
        };
        for rel in &info.relative_file_paths {
            let rel = py_str_or_empty(Some(rel));
            if !run_dir.join(&d).join(&rel).is_file() {
                problems.push(format!("{d}/{rel} 列在 metadata.yaml 但檔案不存在"));
            }
        }
    }
    problems
}

/// `run_metadata.yaml` as a dict (`{}` when missing or not a mapping).
pub fn read_run_metadata(run_dir: &Path) -> Result<Py, UploadError> {
    let path = run_dir.join("run_metadata.yaml");
    if !path.is_file() {
        return Ok(Py::Dict(Vec::new()));
    }
    let text = fs::read_to_string(&path).map_err(io_err)?;
    let data = mower_pyfmt::yaml::safe_load(&text).map_err(|e| UploadError::Parse(e.to_string()))?;
    Ok(if matches!(data, Py::Dict(_)) { data } else { Py::Dict(Vec::new()) })
}

/// `{prefix.strip('/') or 'bags'}/{robot_id}/{run_id}` (`(prefix or 'bags').strip('/')`).
pub fn run_key(prefix: &str, robot_id: &str, run_id: &str) -> String {
    let p = if prefix.is_empty() { "bags" } else { prefix };
    format!("{}/{robot_id}/{run_id}", p.trim_matches('/'))
}

/// `_iso(ns, tz)`: the seconds of `fromtimestamp(ns / 1e9)` in `tz` (seconds
/// east of UTC; `None` = the local zone), `isoformat(timespec='seconds')`.
pub fn iso(ns: i64, tz: Option<i64>) -> String {
    let secs = fromtimestamp_seconds(ns);
    match tz {
        None => LocalTime::from_unix(secs, 0).isoformat_seconds_with_offset(),
        Some(off) => crate::util::civil(secs, off).isoformat_seconds_with_offset(),
    }
}

/// One file of the manifest: (size, sha256).
pub type Entry = (u64, String);

/// `build_manifest`: the manifest dict, keys in the Python's order.
pub fn build_manifest(
    run_dir: &Path,
    run_id: &str,
    robot_id: &str,
    base_key: &str,
    entries: &BTreeMap<String, Entry>,
    run_meta: &Py,
    tz: Option<i64>,
) -> Result<Py, UploadError> {
    let mut topics: BTreeMap<String, (Py, i64)> = BTreeMap::new();
    let (mut starts, mut ends) = (Vec::new(), Vec::new());
    let mut distro = String::new();
    for d in bag_dirs(entries.keys()) {
        let meta = run_dir.join(&d).join(BAG_METADATA);
        if !meta.is_file() {
            continue;
        }
        let info = read_bag_metadata(&meta).map_err(UploadError::Parse)?;
        if distro.is_empty() {
            distro = info.ros_distro.clone();
        }
        if let Some(s) = info.start_ns {
            starts.push(s);
            ends.push(s + info.duration_ns.unwrap_or(0));
        }
        for (name, ty, count) in info.topics {
            topics.entry(name).or_insert((ty, 0)).1 += count;
        }
    }
    let empty = Py::Dict(Vec::new());
    let git = match run_meta.get("git") {
        Some(g @ Py::Dict(_)) => g,
        _ => &empty,
    };
    let mut git_sha = git.get("sha").filter(|v| truthy(Some(v))).cloned().unwrap_or(Py::str(""));
    if !truthy(Some(&git_sha)) || git_sha == Py::str("unknown") {
        let build = match run_meta.get("build") {
            Some(b @ Py::Dict(_)) => b,
            _ => &empty,
        };
        git_sha = build
            .get("git_sha")
            .filter(|v| truthy(Some(v)))
            .cloned()
            .or_else(|| truthy(Some(&git_sha)).then(|| git_sha.clone()))
            .unwrap_or(Py::str("unknown"));
    }
    let camera = match run_meta.get("camera") {
        Some(c @ Py::Dict(_)) => Py::dict(
            ["width", "height", "fps_recorded", "format"].map(|k| (k, c.get(k).cloned().unwrap_or(Py::None))),
        ),
        _ => Py::None,
    };
    let started = match starts.iter().min() {
        Some(s) => Py::Str(iso(*s, tz)),
        None => run_meta.get("start_time").cloned().unwrap_or(Py::None),
    };
    let ended = ends.iter().max().map(|e| Py::Str(iso(*e, tz))).unwrap_or(Py::None);
    let or = |k: &str, default: Py| run_meta.get(k).filter(|v| truthy(Some(v))).cloned().unwrap_or(default);
    Ok(Py::dict([
        ("schema", Py::str(MANIFEST_SCHEMA)),
        ("robot_id", Py::str(robot_id)),
        ("run_id", Py::str(run_id)),
        ("profile", or("profile", Py::str("default"))),
        ("started_at", started),
        ("ended_at", ended),
        ("ros_distro", or("ros_distro", Py::Str(distro))),
        ("git_sha", git_sha),
        (
            "files",
            Py::List(
                entries
                    .iter()
                    .map(|(rel, (size, sha))| {
                        Py::dict([
                            ("key", Py::Str(format!("{base_key}/{rel}"))),
                            ("size", Py::Int(*size as i64)),
                            ("sha256", Py::Str(sha.clone())),
                        ])
                    })
                    .collect(),
            ),
        ),
        (
            "topics",
            Py::dict(topics.into_iter().map(|(n, (ty, c))| (n, Py::dict([("type", ty), ("count", Py::Int(c))])))),
        ),
        ("camera", camera),
        ("camera_height_m", run_meta.get("camera_height_m").cloned().unwrap_or(Py::None)),
    ]))
}

// ---- the state file ----------------------------------------------------------------

#[derive(Debug, Clone, PartialEq)]
pub struct FileState {
    pub size: u64,
    pub mtime_ns: i64,
    pub sha256: String,
    pub uploaded: bool,
}

/// `.upload_state.json` v1.
#[derive(Debug, Clone, PartialEq)]
pub struct State {
    pub bucket: Option<String>,
    pub base_key: Option<String>,
    pub files: BTreeMap<String, FileState>,
    pub manifest_uploaded: bool,
    pub manifest_sha256: Option<String>,
}

impl State {
    fn fresh(bucket: Option<&str>, base_key: Option<&str>) -> State {
        State {
            bucket: bucket.map(String::from),
            base_key: base_key.map(String::from),
            files: BTreeMap::new(),
            manifest_uploaded: false,
            manifest_sha256: None,
        }
    }

    /// `json.dumps(state, indent=1, sort_keys=True)`
    pub fn to_json(&self) -> String {
        let opt = |v: &Option<String>| v.clone().map(Py::Str).unwrap_or(Py::None);
        let mut items = vec![
            ("version", Py::Int(STATE_VERSION)),
            ("bucket", opt(&self.bucket)),
            ("base_key", opt(&self.base_key)),
            (
                "files",
                Py::dict(self.files.iter().map(|(rel, f)| {
                    (
                        rel.clone(),
                        Py::dict([
                            ("size", Py::Int(f.size as i64)),
                            ("mtime_ns", Py::Int(f.mtime_ns)),
                            ("sha256", Py::Str(f.sha256.clone())),
                            ("uploaded", Py::Bool(f.uploaded)),
                        ]),
                    )
                })),
            ),
            ("manifest_uploaded", Py::Bool(self.manifest_uploaded)),
        ];
        if let Some(s) = &self.manifest_sha256 {
            items.push(("manifest_sha256", Py::Str(s.clone())));
        }
        json::dumps(&Py::dict(items), &JsonOptions { ensure_ascii: true, indent: Some(1), sort_keys: true }).expect("no dates")
    }

    /// `load_state`: the file if it is version 1, else a fresh state.
    pub fn load(run_dir: &Path) -> State {
        let fresh = State::fresh(None, None);
        let Ok(text) = fs::read_to_string(run_dir.join(UPLOAD_STATE)) else { return fresh };
        let Ok(v) = json::loads(&text) else { return fresh };
        if v.get("version") != Some(&Py::Int(STATE_VERSION)) {
            return fresh;
        }
        let s = |k: &str| match v.get(k) {
            Some(Py::Str(s)) => Some(s.clone()),
            _ => None,
        };
        let mut files = BTreeMap::new();
        if let Some(Py::Dict(fs_)) = v.get("files") {
            for (rel, f) in fs_ {
                files.insert(
                    rel.clone(),
                    FileState {
                        size: match f.get("size") { Some(Py::Int(i)) => *i as u64, _ => 0 },
                        mtime_ns: match f.get("mtime_ns") { Some(Py::Int(i)) => *i, _ => -1 },
                        sha256: match f.get("sha256") { Some(Py::Str(s)) => s.clone(), _ => String::new() },
                        uploaded: truthy(f.get("uploaded")),
                    },
                );
            }
        }
        State {
            bucket: s("bucket"),
            base_key: s("base_key"),
            files,
            manifest_uploaded: truthy(v.get("manifest_uploaded")),
            manifest_sha256: s("manifest_sha256"),
        }
    }

    pub fn save(&self, run_dir: &Path) -> Result<(), UploadError> {
        atomic_write(&run_dir.join(UPLOAD_STATE), self.to_json().as_bytes()).map_err(io_err)
    }
}

/// What `upload_run` returns.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Outcome {
    pub uploaded: usize,
    pub skipped: usize,
    pub files: usize,
    pub manifest_key: String,
}

fn stat(path: &Path) -> Result<(u64, i64), UploadError> {
    let m = fs::metadata(path).map_err(io_err)?;
    let mtime = m.modified().map_err(io_err)?;
    let ns = match mtime.duration_since(std::time::UNIX_EPOCH) {
        Ok(d) => d.as_nanos() as i64,
        Err(e) => -(e.duration().as_nanos() as i64),
    };
    Ok((m.len(), ns))
}

/// Uploads one run, `_manifest.json` last; safe to call again
/// (`upload_run`). `log` gets the Python's debug lines.
#[allow(clippy::too_many_arguments)]
pub fn upload_run(
    client: &dyn S3,
    bucket: &str,
    base_key: &str,
    run_dir: &Path,
    run_id: &str,
    robot_id: &str,
    tz: Option<i64>,
    log: &mut dyn FnMut(String),
) -> Result<Outcome, UploadError> {
    let files = list_run_files(run_dir);
    let problems = check_ready(run_dir, &files);
    if !problems.is_empty() {
        return Err(UploadError::NotReady(problems.join("; ")));
    }
    let mut state = State::load(run_dir);
    if state.bucket.as_deref() != Some(bucket) || state.base_key.as_deref() != Some(base_key) {
        state = State::fresh(Some(bucket), Some(base_key));
    }
    let mut entries: BTreeMap<String, Entry> = BTreeMap::new();
    let mut skipped: BTreeSet<String> = BTreeSet::new();
    let mut uploaded = 0usize;

    // _stat_sha: the sha256 is reused while size and mtime are unchanged
    let stat_sha = |state: &State, rel: &str| -> Result<(u64, i64, String, bool), UploadError> {
        let path = run_dir.join(rel);
        let (size, mtime) = stat(&path)?;
        let prev = state.files.get(rel);
        let same = prev.is_some_and(|p| !p.sha256.is_empty() && p.size == size && p.mtime_ns == mtime);
        let sha = if same { prev.unwrap().sha256.clone() } else { sha256_file(&path).map_err(io_err)? };
        Ok((size, mtime, sha, same && prev.unwrap().uploaded))
    };
    let mut put = |state: &mut State, entries: &mut BTreeMap<String, Entry>, rel: &str, size: u64, mtime: i64, sha: String,
                   log: &mut dyn FnMut(String)|
     -> Result<(), UploadError> {
        let key = format!("{base_key}/{rel}");
        let path = run_dir.join(rel);
        client.upload_file(&path, bucket, &key, &[("sha256".to_string(), sha.clone())]).map_err(|error| {
            UploadError::UploadFailed { path: path.to_string_lossy().into_owned(), bucket: bucket.into(), key: key.clone(), error }
        })?;
        state.files.insert(rel.to_string(), FileState { size, mtime_ns: mtime, sha256: sha.clone(), uploaded: true });
        state.manifest_uploaded = false;
        state.save(run_dir)?;
        entries.insert(rel.to_string(), (size, sha));
        uploaded += 1;
        log(format!("uploaded {key} ({size} B)"));
        Ok(())
    };

    for rel in &files {
        let (size, mtime, sha, done) = stat_sha(&state, rel)?;
        entries.insert(rel.clone(), (size, sha.clone()));
        if done {
            skipped.insert(rel.clone());
            continue;
        }
        put(&mut state, &mut entries, rel, size, mtime, sha, log)?;
    }
    // everything the state claims must really be on R2 before the manifest
    for rel in &files {
        let key = format!("{base_key}/{rel}");
        let size = client.head_object(bucket, &key).map_err(UploadError::Client)?;
        let want = entries[rel].0;
        if size == Some(want) {
            continue;
        }
        let shown = size.map_or("None".to_string(), |s| s.to_string());
        log(format!("re-upload {rel}: remote size {shown} != {want}"));
        state.files.remove(rel);
        skipped.remove(rel);
        let (size, mtime, sha, _) = stat_sha(&state, rel)?;
        put(&mut state, &mut entries, rel, size, mtime, sha, log)?;
        if client.head_object(bucket, &key).map_err(UploadError::Client)? != Some(size) {
            return Err(UploadError::Runtime(format!("{rel}: size mismatch on R2 after re-upload")));
        }
    }
    let run_meta = read_run_metadata(run_dir)?;
    let manifest = build_manifest(run_dir, run_id, robot_id, base_key, &entries, &run_meta, tz)?;
    let body = json::dumps(&manifest, &JsonOptions { ensure_ascii: false, indent: Some(2), sort_keys: false })
        .map_err(|e| UploadError::NotSerializable(e.0))?
        + "\n";
    atomic_write(&run_dir.join(MANIFEST_NAME), body.as_bytes()).map_err(io_err)?;
    let manifest_key = format!("{base_key}/{MANIFEST_NAME}");
    client.put_object(bucket, &manifest_key, body.as_bytes(), Some("application/json")).map_err(UploadError::Client)?;
    let manifest_sha = sha256_hex(body.as_bytes());
    state.manifest_uploaded = true;
    state.manifest_sha256 = Some(manifest_sha.clone());
    state.save(run_dir)?;
    let marker = Py::dict([
        ("r2_prefix", Py::str(base_key)),
        ("files", Py::Int(files.len() as i64)),
        ("manifest_key", Py::Str(manifest_key.clone())),
        ("manifest_sha256", Py::Str(manifest_sha)),
    ]);
    atomic_write(&run_dir.join(UPLOADED_MARKER), json::dumps(&marker, &JsonOptions::default()).expect("no dates").as_bytes())
        .map_err(io_err)?;
    log(format!(
        "manifest -> {manifest_key} ({} files, {uploaded} uploaded, {} already there)",
        files.len(),
        skipped.len()
    ));
    Ok(Outcome { uploaded, skipped: skipped.len(), files: files.len(), manifest_key })
}

/// Every key under `prefix` (`iter_keys`, paged).
pub fn iter_keys(client: &dyn S3, bucket: &str, prefix: &str) -> Result<Vec<String>, S3Error> {
    let mut out = Vec::new();
    let mut token: Option<String> = None;
    loop {
        let page = client.list_objects_v2(bucket, prefix, token.as_deref())?;
        out.extend(page.keys);
        if !page.is_truncated {
            break;
        }
        token = page.next_token;
    }
    Ok(out)
}

/// Deletes a run from R2, the manifest first; the number deleted
/// (`delete_run_objects`).
pub fn delete_run_objects(client: &dyn S3, bucket: &str, base_key: &str) -> Result<usize, S3Error> {
    let prefix = format!("{}/", base_key.trim_end_matches('/'));
    let manifest_key = format!("{prefix}{MANIFEST_NAME}");
    let keys = iter_keys(client, bucket, &prefix)?;
    let mut n = 0;
    if keys.contains(&manifest_key) {
        client.delete_object(bucket, &manifest_key)?;
        n += 1;
    }
    for k in keys.iter().filter(|k| **k != manifest_key) {
        client.delete_object(bucket, k)?;
        n += 1;
    }
    Ok(n)
}

/// The R2 manifest against the objects next to it, sizes only
/// (`verify_remote`): (ok, problems, manifest).
pub fn verify_remote(client: &dyn S3, bucket: &str, base_key: &str) -> Result<(bool, Vec<String>, Option<Py>), UploadError> {
    let key = format!("{}/{MANIFEST_NAME}", base_key.trim_end_matches('/'));
    let Some(body) = client.get_object(bucket, &key).map_err(UploadError::Client)? else {
        return Ok((false, vec![format!("沒有 {MANIFEST_NAME}（上傳未完成）")], None));
    };
    let manifest = json::loads(&String::from_utf8_lossy(&body)).map_err(UploadError::Parse)?;
    let mut problems = Vec::new();
    if let Some(Py::List(files)) = manifest.get("files") {
        for f in files {
            let (Some(Py::Str(k)), Some(Py::Int(want))) = (f.get("key"), f.get("size")) else { continue };
            match client.head_object(bucket, k).map_err(UploadError::Client)? {
                None => problems.push(format!("缺少 {k}")),
                Some(s) if s as i64 != *want => problems.push(format!("{k} 大小 {s} != manifest {want}")),
                Some(_) => {}
            }
        }
    }
    Ok((problems.is_empty(), problems, Some(manifest)))
}
