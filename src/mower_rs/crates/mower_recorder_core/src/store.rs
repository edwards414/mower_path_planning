//! The bag store (mower_recorder `bag_store_node.py`): the run list the app
//! shows (`/mower_recorder/bags`), its commands (refresh, rename, delete,
//! upload_now) and their replies, the R2 configuration and the upload gate.
//! Texts and JSON are the Python's.

use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};

use mower_pyfmt::json::{self, JsonOptions};
use mower_pyfmt::Py;

use crate::manifest::{UPLOADED_MARKER, UPLOAD_STATE};
use crate::profile::truthy;
use crate::util::dir_size;

/// The R2 settings (bag_store `_load_r2_config`).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct R2Config {
    pub account_id: String,
    pub bucket: String,
    pub access_key_id: String,
    pub secret_access_key: String,
    pub prefix: String,
}

impl R2Config {
    /// `r2_configured` (bs:204): a bucket is set.
    pub fn configured(&self) -> bool {
        !self.bucket.is_empty()
    }

    /// `_r2_client` needs the account too.
    pub fn client(&self) -> Option<crate::s3::Client> {
        if self.bucket.is_empty() || self.account_id.is_empty() {
            return None;
        }
        Some(crate::s3::Client::r2(
            &self.account_id,
            crate::s3::Credentials { access_key: self.access_key_id.clone(), secret_key: self.secret_access_key.clone() },
        ))
    }
}

fn env_line(line: &str) -> Option<(String, String)> {
    let line = line.trim();
    if line.is_empty() || line.starts_with('#') || !line.contains('=') {
        return None;
    }
    let (k, v) = line.split_once('=').unwrap();
    Some((k.trim().to_string(), v.trim().trim_matches('"').trim_matches('\'').to_string()))
}

/// bag_store's precedence: the env, then every line of `env_file`
/// *overrides* it (the key lower-cased, `r2_` removed).
pub fn load_r2_config(env: &dyn Fn(&str) -> Option<String>, env_file: Option<&Path>) -> Result<R2Config, String> {
    let e = |k: &str| env(k).unwrap_or_default();
    let mut c = R2Config {
        account_id: e("R2_ACCOUNT_ID"),
        bucket: e("R2_BUCKET"),
        access_key_id: e("R2_ACCESS_KEY_ID"),
        secret_access_key: e("R2_SECRET_ACCESS_KEY"),
        prefix: env("R2_PREFIX").unwrap_or_else(|| "bags".into()),
    };
    if let Some(p) = env_file.filter(|p| !p.as_os_str().is_empty() && p.is_file()) {
        let text = std::fs::read_to_string(p).map_err(|e| format!("load r2_env_file failed: {e}"))?;
        for line in text.lines() {
            let Some((k, v)) = env_line(line) else { continue };
            match k.to_lowercase().replace("r2_", "").as_str() {
                "account_id" => c.account_id = v,
                "bucket" => c.bucket = v,
                "access_key_id" => c.access_key_id = v,
                "secret_access_key" => c.secret_access_key = v,
                "prefix" => c.prefix = v,
                _ => {}
            }
        }
    }
    Ok(c)
}

/// mower-bag's precedence (cli `_load_env`): the env wins; the first file of
/// the candidates that exists only fills what the env left empty.
pub fn load_cli_env(env: &dyn Fn(&str) -> Option<String>, candidates: &[PathBuf]) -> R2Config {
    let keys = ["R2_ACCOUNT_ID", "R2_BUCKET", "R2_ACCESS_KEY_ID", "R2_SECRET_ACCESS_KEY", "R2_PREFIX"];
    let mut vals: HashMap<&str, String> = keys.iter().map(|k| (*k, env(k).unwrap_or_default())).collect();
    if let Some(p) = candidates.iter().find(|p| p.is_file()) {
        if let Ok(text) = std::fs::read_to_string(p) {
            for line in text.lines() {
                let Some((k, v)) = env_line(line) else { continue };
                if let Some(slot) = keys.iter().find(|x| **x == k).and_then(|x| vals.get_mut(x)) {
                    if slot.is_empty() {
                        *slot = v;
                    }
                }
            }
        }
    }
    let prefix = vals["R2_PREFIX"].clone();
    R2Config {
        account_id: vals["R2_ACCOUNT_ID"].clone(),
        bucket: vals["R2_BUCKET"].clone(),
        access_key_id: vals["R2_ACCESS_KEY_ID"].clone(),
        secret_access_key: vals["R2_SECRET_ACCESS_KEY"].clone(),
        prefix: if prefix.is_empty() { "bags".into() } else { prefix },
    }
}

/// `upload_policy`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Policy {
    WifiOrDock,
    Always,
    Manual,
}

impl Policy {
    /// Anything but `always` and `manual` is `wifi_or_dock`, as the Python's
    /// final branch is.
    pub fn parse(s: &str) -> Policy {
        match s {
            "always" => Policy::Always,
            "manual" => Policy::Manual,
            _ => Policy::WifiOrDock,
        }
    }
}

/// `_network_ok`
pub fn network_ok(policy: Policy, docked: bool, wifi_ifaces: &[String]) -> bool {
    match policy {
        Policy::Always => true,
        Policy::Manual => false,
        Policy::WifiOrDock => docked || wifi_ifaces.iter().any(|i| iface_up(i)),
    }
}

/// `_iface_up`: `operstate` is `up` and the interface has an IPv4 address
/// (the Python asks `ip -4 addr show`; this asks getifaddrs).
pub fn iface_up(iface: &str) -> bool {
    match std::fs::read_to_string(format!("/sys/class/net/{iface}/operstate")) {
        Ok(s) if s.trim() == "up" => has_ipv4(iface),
        _ => false,
    }
}

fn has_ipv4(iface: &str) -> bool {
    let mut addrs: *mut libc::ifaddrs = std::ptr::null_mut();
    if unsafe { libc::getifaddrs(&mut addrs) } != 0 {
        return false;
    }
    let mut found = false;
    let mut p = addrs;
    while !p.is_null() {
        let a = unsafe { &*p };
        if !a.ifa_addr.is_null() && unsafe { (*a.ifa_addr).sa_family } as i32 == libc::AF_INET {
            let name = unsafe { std::ffi::CStr::from_ptr(a.ifa_name) };
            if name.to_bytes() == iface.as_bytes() {
                found = true;
                break;
            }
        }
        p = a.ifa_next;
    }
    unsafe { libc::freeifaddrs(addrs) };
    found
}

/// `_read_meta`: `run_metadata.yaml`, `{}` on any failure (a document that
/// is not a mapping too: the Python would fail later on `.get`).
pub fn read_meta(run_dir: &Path) -> Py {
    std::fs::read_to_string(run_dir.join("run_metadata.yaml"))
        .ok()
        .and_then(|t| mower_pyfmt::yaml::safe_load(&t).ok())
        .filter(|v| matches!(v, Py::Dict(_)))
        .unwrap_or(Py::Dict(Vec::new()))
}

/// `_write_meta`
pub fn write_meta(run_dir: &Path, meta: &Py) -> std::io::Result<()> {
    std::fs::write(run_dir.join("run_metadata.yaml"), crate::run::metadata_yaml(meta))
}

/// What the bag list needs besides the disk.
#[derive(Debug, Clone, Default)]
pub struct ListState {
    pub robot_id: String,
    pub uploading: HashSet<String>,
    pub recording: Option<String>,
    pub upload_errors: HashMap<String, String>,
}

/// `_list_runs`: every directory of the root, sorted by name.
pub fn list_runs(root: &Path, st: &ListState) -> Vec<Py> {
    let mut names: Vec<String> = std::fs::read_dir(root)
        .map(|rd| rd.flatten().map(|e| e.file_name().to_string_lossy().into_owned()).collect())
        .unwrap_or_default();
    names.sort();
    let mut runs = Vec::new();
    for name in names {
        let run_dir = root.join(&name);
        if !run_dir.is_dir() {
            continue;
        }
        let meta = read_meta(&run_dir);
        let or = |k: &str, default: Py| match meta.get(k) {
            Some(v) if truthy(Some(v)) => v.clone(),
            _ => default,
        };
        runs.push(Py::dict([
            ("run_id", Py::str(&name)),
            ("display_name", or("display_name", Py::str(&name))),
            ("robot_id", meta.get("robot_id").cloned().unwrap_or(Py::str(&st.robot_id))),
            ("start_time", meta.get("start_time").cloned().unwrap_or(Py::str(""))),
            ("size_bytes", Py::Int(dir_size(&run_dir) as i64)),
            ("uploaded", Py::Bool(run_dir.join(UPLOADED_MARKER).is_file())),
            ("uploading", Py::Bool(st.uploading.contains(&name))),
            ("recording", Py::Bool(st.recording.as_deref() == Some(name.as_str()))),
            ("profile", or("profile", Py::str("default"))),
            ("upload_error", st.upload_errors.get(&name).map(|e| Py::str(e)).unwrap_or(Py::None)),
        ]));
    }
    runs
}

/// `/mower_recorder/bags` (`json.dumps(payload, ensure_ascii=False)`).
pub fn bags_json(root: &Path, st: &ListState, network_ok: bool, r2_configured: bool) -> Result<String, String> {
    let payload = Py::dict([
        ("robot_id", Py::str(&st.robot_id)),
        ("network_ok", Py::Bool(network_ok)),
        ("r2_configured", Py::Bool(r2_configured)),
        ("bags", Py::List(list_runs(root, st))),
    ]);
    json::dumps(&payload, &JsonOptions::unicode()).map_err(|e| e.0)
}

/// `/mower_recorder/command_result`
pub fn result_json(req_id: &Py, ok: bool, message: &str) -> String {
    let v = Py::dict([("req_id", req_id.clone()), ("ok", Py::Bool(ok)), ("message", Py::str(message))]);
    json::dumps(&v, &JsonOptions::unicode()).unwrap_or_default()
}

/// A command from the app.
#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    Refresh,
    Rename { run_id: String, new_name: String },
    Delete { run_id: String },
    UploadNow,
    Unknown(String),
}

/// `_on_command`'s parse: the command and its `req_id`, or the reply to a
/// message that is not JSON (`bad command json: <CPython's error>`), with
/// an empty `req_id`. `None`: the Python's callback would raise (not an
/// object, or a field of another type) and answer nothing.
pub fn parse_command(text: &str) -> Option<Result<(Command, Py), String>> {
    let cmd = match json::loads(text) {
        Ok(c) => c,
        Err(e) => return Some(Err(format!("bad command json: {e}"))),
    };
    let Py::Dict(_) = cmd else { return None };
    let s = |k: &str| -> Option<String> {
        match cmd.get(k) {
            None => Some(String::new()),
            Some(Py::Str(s)) => Some(s.clone()),
            _ => None,
        }
    };
    let req_id = cmd.get("req_id").cloned().unwrap_or(Py::str(""));
    let action = match cmd.get("action") {
        None => String::new(),
        Some(Py::Str(a)) => a.clone(),
        Some(other) => return Some(Ok((Command::Unknown(json::dumps(other, &JsonOptions::default()).unwrap_or_default()), req_id))),
    };
    let c = match action.as_str() {
        "refresh" => Command::Refresh,
        "rename" => Command::Rename { run_id: s("run_id")?, new_name: s("new_name")? },
        "delete" => Command::Delete { run_id: s("run_id")? },
        "upload_now" => Command::UploadNow,
        other => Command::Unknown(other.to_string()),
    };
    Some(Ok((c, req_id)))
}

/// The fixed replies.
pub const REFRESHED: &str = "refreshed";
pub const UPLOAD_QUEUED: &str = "已排入上傳";
pub fn unknown_action(a: &str) -> String {
    format!("unknown action: {a}")
}

/// `_safe_run_dir`
pub fn safe_run_dir(root: &Path, run_id: &str) -> Option<PathBuf> {
    if run_id.is_empty() || run_id.contains('/') || run_id.starts_with('.') {
        return None;
    }
    let d = root.join(run_id);
    d.is_dir().then_some(d)
}

/// `_rename`: (ok, message).
pub fn rename(root: &Path, run_id: &str, new_name: &str) -> (bool, String) {
    let Some(run_dir) = safe_run_dir(root, run_id) else { return (false, format!("找不到 run: {run_id}")) };
    let name = crate::extrinsics::py_strip(new_name);
    if name.is_empty() {
        return (false, "新名稱不可空白".into());
    }
    let meta = crate::run::with_display_name(&read_meta(&run_dir), &name);
    if let Err(e) = write_meta(&run_dir, &meta) {
        return (false, format!("{e}"));
    }
    (true, format!("已更名為「{name}」"))
}

/// What `_delete` did: the reply, and the R2 prefix to clear when the run
/// may have objects there (`.uploaded` or `.upload_state.json`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Deleted {
    pub ok: bool,
    pub message: String,
    pub r2_base_key: Option<String>,
}

pub fn delete_local(root: &Path, run_id: &str, recording: Option<&str>, r2_prefix: &str, robot_id: &str) -> Deleted {
    let no = |m: String| Deleted { ok: false, message: m, r2_base_key: None };
    if Some(run_id) == recording {
        return no("這個 run 正在錄製中,無法刪除".into());
    }
    let Some(run_dir) = safe_run_dir(root, run_id) else { return no(format!("找不到 run: {run_id}")) };
    let on_r2 = [UPLOADED_MARKER, UPLOAD_STATE].iter().any(|f| run_dir.join(f).is_file());
    let base_key = base_key(&run_dir, r2_prefix, robot_id, run_id);
    if let Err(e) = std::fs::remove_dir_all(&run_dir) {
        return no(format!("刪除本機失敗: {e}"));
    }
    Deleted { ok: true, message: format!("已刪除 {run_id}"), r2_base_key: on_r2.then_some(base_key) }
}

/// `_base_key`: the robot id from the run's metadata, else ours.
pub fn base_key(run_dir: &Path, r2_prefix: &str, robot_id: &str, run_id: &str) -> String {
    let meta = read_meta(run_dir);
    let rid = match meta.get("robot_id") {
        Some(Py::Str(s)) if !s.is_empty() => s.clone(),
        _ => robot_id.to_string(),
    };
    crate::manifest::run_key(r2_prefix, &rid, run_id)
}
