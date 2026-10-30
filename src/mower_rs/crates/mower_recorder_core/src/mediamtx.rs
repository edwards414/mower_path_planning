//! The front camera's video for a run (mower_recorder `video_record.py`):
//! MediaMTX records its own stream into `<run>/video/` while the run goes.
//! The bodies, URLs, headers and the 3 s timeout are the Python's; a
//! transport error's text is ureq's, not urllib's (A9).

use std::time::Duration;

use mower_pyfmt::json::{self, JsonOptions};
use mower_pyfmt::Py;

pub const VIDEO_DIR: &str = "video";
pub const TIMEOUT: Duration = Duration::from_secs(3);

/// `record_patch(run_dir)`
pub fn record_patch(run_dir: &str) -> Py {
    Py::dict([
        ("record", Py::Bool(true)),
        ("recordPath", Py::Str(format!("{run_dir}/{VIDEO_DIR}/%path_%Y-%m-%d_%H-%M-%S-%f"))),
        ("recordFormat", Py::str("fmp4")),
        ("recordDeleteAfter", Py::str("0s")),
    ])
}

/// `STOP_PATCH`
pub fn stop_patch() -> Py {
    Py::dict([("record", Py::Bool(false))])
}

/// The PATCH body: `json.dumps(body)` (defaults).
pub fn body_bytes(body: &Py) -> Vec<u8> {
    json::dumps(body, &JsonOptions::default()).expect("no dates").into_bytes()
}

fn agent() -> ureq::Agent {
    ureq::AgentBuilder::new().timeout(TIMEOUT).build()
}

/// `patch_path`: PATCH `/v3/config/paths/patch/<path>`; (ok, message).
pub fn patch_path(api_url: &str, path: &str, body: &Py) -> (bool, String) {
    let url = format!("{}/v3/config/paths/patch/{path}", api_url.trim_end_matches('/'));
    match agent().request("PATCH", &url).set("Content-Type", "application/json").send_bytes(&body_bytes(body)) {
        Ok(r) => (true, format!("HTTP {}", r.status())),
        Err(ureq::Error::Status(code, r)) => {
            let detail = r.into_string().unwrap_or_default();
            (false, format!("HTTP {code} {}", detail.trim()).trim().to_string())
        }
        Err(ureq::Error::Transport(t)) => (false, t.to_string()),
    }
}

/// `path_ready`: whether the camera path has a live stream; `None` when
/// MediaMTX cannot be asked (a path without a publisher is a 404: `false`).
pub fn path_ready(api_url: &str, path: &str) -> Option<bool> {
    let url = format!("{}/v3/paths/get/{path}", api_url.trim_end_matches('/'));
    match agent().get(&url).call() {
        Ok(r) => {
            let text = r.into_string().ok()?;
            let v = json::loads(if text.is_empty() { "{}" } else { &text }).ok()?;
            Some(crate::profile::truthy(v.get("ready")))
        }
        Err(ureq::Error::Status(404, _)) => Some(false),
        Err(_) => None,
    }
}
