//! `/robot/info` snapshot, `robot_status.json` and the update lights (port of
//! `robot_info_node.py`). See docs/ROBOT_API.md for the document layout.

use std::time::Instant;

use mower_rs_common::{read_state_json, round_to, unix_time_s, write_atomic};
use serde_json::{json, Map, Value};

use crate::state::State;

/// Contract between the robot and the app; must match
/// `mower_mission/version.py` ROBOT_API_VERSION (checked by
/// src/mower_mission/test/test_robot_info.py).
pub const ROBOT_API_VERSION: i64 = 2;

/// Build identity of the running robot software, from the image env
/// (`mower_mission.version.software_identity`).
pub fn software_identity() -> Map<String, Value> {
    let env = |k: &str| std::env::var(k).unwrap_or_default();
    let build_unix = env("MOWER_BUILD_UNIX").parse::<i64>().ok();
    let mut m = Map::new();
    m.insert("version".into(), json!(if env("MOWER_VERSION").is_empty() { "dev".to_string() } else { env("MOWER_VERSION") }));
    m.insert("git_sha".into(), json!(env("MOWER_GIT_SHA")));
    m.insert("build_unix".into(), build_unix.map(|v| json!(v)).unwrap_or(Value::Null));
    m.insert("image".into(), json!(env("MOWER_IMAGE")));
    m
}

/// `identity.json` written by deploy/host/mower-pair, or `None` when this is
/// an unpaired development robot (`mower_mission.identity.load_identity`).
pub fn load_identity(state_dir: &str) -> Option<Value> {
    let data = read_state_json(state_dir, "identity.json")?;
    let obj = data.as_object()?;
    let robot_id = obj.get("robot_id")?.as_str()?;
    let valid_id = robot_id.len() == 9
        && robot_id.starts_with("MW-")
        && robot_id[3..].chars().all(|c| c.is_ascii_uppercase() || c.is_ascii_digit());
    if !valid_id {
        return None;
    }
    match obj.get("secret") {
        Some(Value::String(s)) if !s.is_empty() => Some(data),
        _ => None,
    }
}

pub struct InfoConfig {
    pub robot_id: String,
    pub robot_name: String,
    pub paired: bool,
    pub state_dir: String,
    pub software: Map<String, Value>,
    pub bundled_firmware: Option<Value>,
    pub busy_timeout_s: f64,
    pub led_update_rgb: [i64; 3],
    pub led_update_period_ms: i64,
    pub led_normal_rgb: [i64; 3],
}

fn truthy(v: Option<&Value>) -> bool {
    match v {
        None | Some(Value::Null) => false,
        Some(Value::Bool(b)) => *b,
        Some(Value::String(s)) => !s.is_empty(),
        Some(Value::Number(n)) => n.as_f64().map(|f| f != 0.0).unwrap_or(true),
        Some(Value::Array(a)) => !a.is_empty(),
        Some(Value::Object(o)) => !o.is_empty(),
    }
}

pub fn moving(st: &State, busy_timeout_s: f64, now: Instant) -> bool {
    st.last_moving
        .map(|t| now.duration_since(t).as_secs_f64() <= busy_timeout_s)
        .unwrap_or(false)
}

pub fn busy(st: &State, busy_timeout_s: f64, now: Instant) -> bool {
    moving(st, busy_timeout_s, now) || st.nav_running
}

/// `update` = update_status.json (the host update script's progress) plus
/// the last registry check (update_check.json, written by
/// `mower-update.sh --check` every 5 min and on demand): `remote_digest`,
/// `checked_at`, `check_error` and `available` = the channel's digest is
/// known and differs from the running image. Missing files leave the
/// pre-check shape untouched.
pub fn update_block(update: Option<Value>, check: Option<Value>, running_digest: Option<&Value>) -> Value {
    let mut block = match update {
        Some(Value::Object(m)) => m,
        Some(other) if check.is_none() => return other,
        _ => Map::new(),
    };
    if let Some(Value::Object(c)) = check {
        let remote = c.get("remote_digest").and_then(|v| v.as_str()).unwrap_or("");
        let error = c.get("error").and_then(|v| v.as_str()).unwrap_or("");
        let running = running_digest.and_then(|v| v.as_str()).unwrap_or("");
        block.insert("remote_digest".into(), if remote.is_empty() { Value::Null } else { json!(remote) });
        block.insert("checked_at".into(), c.get("time").cloned().unwrap_or(Value::Null));
        block.insert("check_error".into(), if error.is_empty() { Value::Null } else { json!(error) });
        block.insert("available".into(), json!(!remote.is_empty() && !running.is_empty() && remote != running));
    }
    if block.is_empty() {
        return Value::Null;
    }
    Value::Object(block)
}

/// The `/robot/info` document.
pub fn snapshot(cfg: &InfoConfig, st: &State, start: Instant, now: Instant) -> Value {
    let image = read_state_json(&cfg.state_dir, "image.json").unwrap_or(Value::Null);
    let sync = read_state_json(&cfg.state_dir, "firmware_sync.json");
    let update = read_state_json(&cfg.state_dir, "update_status.json");
    let check = read_state_json(&cfg.state_dir, "update_check.json");

    let mut software = cfg.software.clone();
    if let Some(image) = image.as_object() {
        for key in ["image", "digest", "tag", "pulled_at"] {
            if truthy(image.get(key)) && !truthy(software.get(key)) {
                software.insert(key.into(), image[key].clone());
            }
        }
    }

    let bundled = cfg.bundled_firmware.as_ref().and_then(|b| b.as_object()).map(|b| {
        let mut m = Map::new();
        for k in ["version", "semver", "git_sha", "build_unix", "dirty", "size", "crc32"] {
            m.insert(k.into(), b.get(k).cloned().unwrap_or(Value::Null));
        }
        Value::Object(m)
    });

    let running = st.firmware_running.clone();
    let up_to_date = match (running.as_ref().and_then(|r| r.as_object()), bundled.as_ref().and_then(|b| b.as_object())) {
        (Some(r), Some(b)) if !r.is_empty() && !b.is_empty() => {
            let bundled_sha: String = b
                .get("git_sha")
                .and_then(|v| v.as_str())
                .unwrap_or("")
                .chars()
                .take(8)
                .collect();
            let eq = |k: &str| r.get(k).unwrap_or(&Value::Null) == b.get(k).unwrap_or(&Value::Null);
            json!(
                !truthy(r.get("unversioned"))
                    && eq("semver")
                    && r.get("git_sha").and_then(|v| v.as_str()) == Some(bundled_sha.as_str())
                    && eq("build_unix")
                    && truthy(r.get("dirty")) == truthy(b.get("dirty"))
            )
        }
        _ => Value::Null,
    };

    let sync_summary = sync.as_ref().and_then(|s| s.as_object()).map(|s| {
        json!({
            "action": s.get("action").cloned().unwrap_or(Value::Null),
            "time": s.get("time").cloned().unwrap_or(Value::Null),
            "error": s.get("error").cloned().unwrap_or(Value::Null),
        })
    });

    let update = update_block(update, check, software.get("digest"));
    json!({
        "robot_id": cfg.robot_id,
        "name": cfg.robot_name,
        "pairing_required": cfg.paired,
        "api_version": ROBOT_API_VERSION,
        "software": Value::Object(software),
        "firmware": {
            "running": running.unwrap_or(Value::Null),
            "bundled": bundled.unwrap_or(Value::Null),
            "sync": sync_summary.unwrap_or(Value::Null),
            "up_to_date": up_to_date,
        },
        "update": update,
        "busy": busy(st, cfg.busy_timeout_s, now),
        "uptime_s": round_to(now.duration_since(start).as_secs_f64(), 1),
    })
}

/// LED request for the base when the host's update state changes, or `None`
/// when nothing changed. Amber orbit (mode 6) while updating, steady white
/// (mode 1) afterwards; see firmware/LED_COMMAND_MODES.md.
pub fn led_request(cfg: &InfoConfig, st: &mut State, update: &Value) -> Option<(bool, Value)> {
    let state = update.get("state").and_then(|v| v.as_str()).unwrap_or("");
    let updating = matches!(state, "pulling" | "restarting" | "rebooting");
    if st.led_updating == Some(updating) {
        return None;
    }
    st.led_updating = Some(updating);
    let req = if updating {
        json!({
            "mode": 6,
            "r": cfg.led_update_rgb[0], "g": cfg.led_update_rgb[1], "b": cfg.led_update_rgb[2],
            "period_ms": cfg.led_update_period_ms,
        })
    } else {
        json!({
            "mode": 1,
            "r": cfg.led_normal_rgb[0], "g": cfg.led_normal_rgb[1], "b": cfg.led_normal_rgb[2],
            "period_ms": 0,
        })
    };
    Some((updating, req))
}

/// `<state_dir>/robot_status.json`: idle/busy for the host auto-update timer
/// (deploy/host/mower-update.sh).
pub fn write_robot_status(cfg: &InfoConfig, st: &State, snapshot: &Value, now: Instant) -> std::io::Result<()> {
    let doc = json!({
        "time": unix_time_s() as i64,
        "busy": truthy(snapshot.get("busy")),
        "moving": moving(st, cfg.busy_timeout_s, now),
        "nav_running": st.nav_running,
        "software": cfg.software.get("version").cloned().unwrap_or(Value::Null),
    });
    write_atomic(
        format!("{}/robot_status.json", cfg.state_dir),
        serde_json::to_string(&doc).expect("json").as_bytes(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg(dir: &str) -> InfoConfig {
        let mut software = Map::new();
        software.insert("version".into(), json!("0.6.0"));
        software.insert("git_sha".into(), json!("abc"));
        software.insert("build_unix".into(), Value::Null);
        software.insert("image".into(), json!(""));
        InfoConfig {
            robot_id: "MW-ABC123".into(),
            robot_name: "lawn".into(),
            paired: true,
            state_dir: dir.into(),
            software,
            bundled_firmware: Some(json!({
                "version": "1.2.0", "semver": "1.2.0", "git_sha": "0123456789abcdef",
                "build_unix": 100, "dirty": false, "size": 1, "crc32": 2,
            })),
            busy_timeout_s: 2.0,
            led_update_rgb: [255, 180, 0],
            led_update_period_ms: 1600,
            led_normal_rgb: [110, 110, 110],
        }
    }

    #[test]
    fn snapshot_reports_firmware_match_and_image_fallbacks() {
        let dir = std::env::temp_dir().join(format!("mower_rs_info_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("image.json"), r#"{"image":"ghcr.io/x:main","digest":"sha256:1","tag":"main"}"#).unwrap();
        std::fs::write(dir.join("update_status.json"), r#"{"state":"pulling","message":"m","time":5}"#).unwrap();
        let cfg = cfg(dir.to_str().unwrap());
        let mut st = State::default();
        st.firmware_running = Some(json!({
            "semver": "1.2.0", "git_sha": "01234567", "build_unix": 100, "dirty": false,
        }));
        let now = Instant::now();
        let snap = snapshot(&cfg, &st, now, now);
        assert_eq!(snap["api_version"], json!(2));
        assert_eq!(snap["firmware"]["up_to_date"], json!(true));
        assert_eq!(snap["software"]["image"], json!("ghcr.io/x:main"));
        assert_eq!(snap["software"]["version"], json!("0.6.0"));
        assert_eq!(snap["update"]["state"], json!("pulling"));
        // no registry check yet: the block is the plain update_status.json
        assert!(snap["update"].get("available").is_none());
        std::fs::write(dir.join("update_check.json"), r#"{"time":9,"remote_digest":"sha256:2","error":""}"#).unwrap();
        let snap = snapshot(&cfg, &st, now, now);
        assert_eq!(snap["update"]["available"], json!(true));
        assert_eq!(snap["update"]["remote_digest"], json!("sha256:2"));
        assert_eq!(snap["update"]["checked_at"], json!(9));
        assert_eq!(snap["update"]["check_error"], Value::Null);
        std::fs::write(dir.join("update_check.json"), r#"{"time":10,"remote_digest":"sha256:1","error":""}"#).unwrap();
        assert_eq!(snapshot(&cfg, &st, now, now)["update"]["available"], json!(false));
        std::fs::write(dir.join("update_check.json"), r#"{"time":11,"remote_digest":"","error":"registry lookup failed"}"#).unwrap();
        let snap = snapshot(&cfg, &st, now, now);
        assert_eq!(snap["update"]["available"], json!(false));
        assert_eq!(snap["update"]["check_error"], json!("registry lookup failed"));
        assert_eq!(snap["update"]["state"], json!("pulling"));
        assert_eq!(snap["busy"], json!(false));
        assert_eq!(snap["pairing_required"], json!(true));
        // lights: amber orbit while pulling, then back to white, then silent
        let (updating, req) = led_request(&cfg, &mut st, &snap["update"]).unwrap();
        assert!(updating);
        assert_eq!(req["mode"], json!(6));
        assert!(led_request(&cfg, &mut st, &snap["update"]).is_none());
        let (updating, req) = led_request(&cfg, &mut st, &json!({"state": "idle"})).unwrap();
        assert!(!updating);
        assert_eq!(req["mode"], json!(1));
        write_robot_status(&cfg, &st, &snap, now).unwrap();
        let status = mower_rs_common::read_json_file(dir.join("robot_status.json")).unwrap();
        assert_eq!(status["software"], json!("0.6.0"));
        assert_eq!(status["busy"], json!(false));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn firmware_mismatch_and_missing_files() {
        let dir = std::env::temp_dir().join(format!("mower_rs_info2_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let cfg = cfg(dir.to_str().unwrap());
        let mut st = State::default();
        let now = Instant::now();
        assert_eq!(snapshot(&cfg, &st, now, now)["firmware"]["up_to_date"], Value::Null);
        st.firmware_running = Some(json!({"semver": "1.1.0", "git_sha": "01234567", "build_unix": 100, "dirty": false}));
        assert_eq!(snapshot(&cfg, &st, now, now)["firmware"]["up_to_date"], json!(false));
        st.nav_running = true;
        assert_eq!(snapshot(&cfg, &st, now, now)["busy"], json!(true));
        assert_eq!(snapshot(&cfg, &st, now, now)["update"], Value::Null);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn identity_requires_valid_id_and_secret() {
        let dir = std::env::temp_dir().join(format!("mower_rs_id_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let d = dir.to_str().unwrap();
        assert!(load_identity(d).is_none());
        std::fs::write(dir.join("identity.json"), r#"{"robot_id":"MW-ABC123","secret":"s","name":"lawn"}"#).unwrap();
        assert_eq!(load_identity(d).unwrap()["name"], json!("lawn"));
        std::fs::write(dir.join("identity.json"), r#"{"robot_id":"mw-abc123","secret":"s"}"#).unwrap();
        assert!(load_identity(d).is_none());
        std::fs::write(dir.join("identity.json"), r#"{"robot_id":"MW-ABC123"}"#).unwrap();
        assert!(load_identity(d).is_none());
        let _ = std::fs::remove_dir_all(&dir);
    }
}
