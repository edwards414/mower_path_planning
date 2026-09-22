//! Shared helpers for the mower_rs nodes.
//!
//! Everything here is ROS-free except [`params`], which reads the initial
//! parameter overrides r2r loaded from `--ros-args` / `--params-file`.

use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use serde_json::Value;

pub mod guard;
pub mod module;

pub use module::{module_main, BoxError, ModuleCtx, ModuleResult, ModuleRun, Shutdown};

/// Typed access to a node's initial parameters with defaults.
///
/// r2r only knows the overrides that were passed on the command line; a
/// parameter that was not passed is `NotSet`, so every getter takes a
/// default, exactly like `declare_parameter(name, default)` in rclpy.
pub mod params {
    use r2r::{Node, ParameterValue};

    fn value(node: &Node, name: &str) -> ParameterValue {
        node.params
            .lock()
            .unwrap()
            .get(name)
            .map(|p| p.value.clone())
            .unwrap_or(ParameterValue::NotSet)
    }

    pub fn string(node: &Node, name: &str, default: &str) -> String {
        match value(node, name) {
            ParameterValue::String(s) => s,
            ParameterValue::NotSet => default.to_string(),
            other => {
                r2r::log_warn!(node.logger(), "parameter {name}: expected string, got {other:?}; using default");
                default.to_string()
            }
        }
    }

    pub fn f64(node: &Node, name: &str, default: f64) -> f64 {
        match value(node, name) {
            ParameterValue::Double(v) => v,
            ParameterValue::Integer(v) => v as f64,
            ParameterValue::NotSet => default,
            other => {
                r2r::log_warn!(node.logger(), "parameter {name}: expected number, got {other:?}; using default");
                default
            }
        }
    }

    pub fn bool(node: &Node, name: &str, default: bool) -> bool {
        match value(node, name) {
            ParameterValue::Bool(v) => v,
            ParameterValue::NotSet => default,
            other => {
                r2r::log_warn!(node.logger(), "parameter {name}: expected bool, got {other:?}; using default");
                default
            }
        }
    }

    pub fn i64(node: &Node, name: &str, default: i64) -> i64 {
        match value(node, name) {
            ParameterValue::Integer(v) => v,
            ParameterValue::Double(v) => v as i64,
            ParameterValue::NotSet => default,
            other => {
                r2r::log_warn!(node.logger(), "parameter {name}: expected integer, got {other:?}; using default");
                default
            }
        }
    }

    pub fn i64_array(node: &Node, name: &str, default: &[i64]) -> Vec<i64> {
        match value(node, name) {
            ParameterValue::IntegerArray(v) => v,
            ParameterValue::DoubleArray(v) => v.into_iter().map(|x| x as i64).collect(),
            ParameterValue::NotSet => default.to_vec(),
            other => {
                r2r::log_warn!(node.logger(), "parameter {name}: expected integer array, got {other:?}; using default");
                default.to_vec()
            }
        }
    }
}

/// `round(x, digits)` as Python does it for JSON output (NaN/inf pass through
/// and become `null` when serialised, see [`json_number`]).
pub fn round_to(x: f64, digits: i32) -> f64 {
    if !x.is_finite() {
        return x;
    }
    let scale = 10f64.powi(digits);
    (x * scale).round() / scale
}

/// A JSON number, or `null` for NaN / inf (Dart's json.decode rejects bare NaN).
pub fn json_number(x: f64) -> Value {
    if x.is_finite() {
        serde_json::json!(x)
    } else {
        Value::Null
    }
}

/// Seconds since the Unix epoch as f64.
pub fn unix_time_s() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

/// Directory shared with the host (bind-mounted ~/.mower), as in
/// `mower_mission.host_request.state_dir()`.
pub fn state_dir() -> String {
    if let Ok(dir) = std::env::var("MOWER_STATE_DIR") {
        if !dir.is_empty() {
            return dir;
        }
    }
    let home = std::env::var("HOME").unwrap_or_else(|_| "/".to_string());
    format!("{}/.mower", home.trim_end_matches('/'))
}

/// Parsed JSON file, or `None` when it is missing or malformed.
pub fn read_json_file(path: impl AsRef<Path>) -> Option<Value> {
    let text = fs::read_to_string(path).ok()?;
    serde_json::from_str(&text).ok()
}

/// Same as [`read_json_file`] but for a file inside the state dir.
pub fn read_state_json(dir: &str, name: &str) -> Option<Value> {
    read_json_file(Path::new(dir).join(name))
}

/// Write `contents` to `path` atomically (temp file + rename), creating the
/// parent directory if needed.
pub fn write_atomic(path: impl AsRef<Path>, contents: &[u8]) -> std::io::Result<()> {
    let path = path.as_ref();
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    let mut tmp = PathBuf::from(path);
    tmp.set_file_name(format!(
        "{}.tmp",
        path.file_name().and_then(|n| n.to_str()).unwrap_or("file")
    ));
    {
        let mut f = fs::File::create(&tmp)?;
        f.write_all(contents)?;
    }
    fs::rename(&tmp, path)
}

/// Ask the host to `update` / `restart` / `reboot` / `poweroff` by writing
/// `<state_dir>/host.request` (see deploy/host/mower-host-request). Returns
/// the request path.
pub fn write_host_request(dir: &str, action: &str, requested_by: &str) -> std::io::Result<String> {
    const ACTIONS: [&str; 5] = ["update", "check", "restart", "reboot", "poweroff"];
    if !ACTIONS.contains(&action) {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            format!("unknown host action {action:?}"),
        ));
    }
    let path = Path::new(dir).join("host.request");
    let doc = serde_json::json!({
        "action": action,
        "time": unix_time_s() as i64,
        "requested_by": requested_by,
    });
    let mut body = serde_json::to_vec(&doc).expect("json");
    body.push(b'\n');
    write_atomic(&path, &body)?;
    Ok(path.to_string_lossy().into_owned())
}

/// The machine's hostname (what Python's `socket.gethostname()` returns).
pub fn hostname() -> String {
    fs::read_to_string("/proc/sys/kernel/hostname")
        .map(|s| s.trim().to_string())
        .ok()
        .filter(|s| !s.is_empty())
        .or_else(|| std::env::var("HOSTNAME").ok())
        .unwrap_or_else(|| "robot".to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_matches_python_for_common_values() {
        assert_eq!(round_to(0.12345, 3), 0.123);
        assert_eq!(round_to(-1.23456, 2), -1.23);
        assert!(round_to(f64::NAN, 3).is_nan());
    }

    #[test]
    fn non_finite_numbers_become_null() {
        assert_eq!(json_number(f64::NAN), Value::Null);
        assert_eq!(json_number(f64::INFINITY), Value::Null);
        assert_eq!(json_number(1.5), serde_json::json!(1.5));
    }

    #[test]
    fn host_request_is_atomic_json() {
        let dir = std::env::temp_dir().join(format!("mower_rs_{}", std::process::id()));
        let dir_s = dir.to_string_lossy().into_owned();
        let path = write_host_request(&dir_s, "update", "app").unwrap();
        let doc = read_json_file(&path).unwrap();
        assert_eq!(doc["action"], "update");
        assert_eq!(doc["requested_by"], "app");
        assert!(doc["time"].as_i64().unwrap() > 1_700_000_000);
        assert!(!dir.join("host.request.tmp").exists());
        assert!(write_host_request(&dir_s, "format-disk", "app").is_err());
        let _ = fs::remove_dir_all(&dir);
    }
}
