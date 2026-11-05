//! G2 of the E6 design: `write_host_request` against the golden the Python and
//! shell writers are held to (`deploy/test/golden/host/g2_host_request.json`,
//! written by `src/mower_mission/test/test_host_contracts.py`, which also runs
//! this writer's text through `deploy/host/mower-host-request.sh`).

use std::path::{Path, PathBuf};

use mower_rs_common::write_host_request;
use serde_json::Value;

fn golden() -> Value {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../deploy/test/golden/host/g2_host_request.json");
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    serde_json::from_str(&text).expect("the golden is JSON")
}

fn scratch(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("mower_rs_common_g2_{}_{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

#[test]
fn accepted_actions_write_what_the_golden_says() {
    let g = golden();
    let dir = scratch("accept");
    let d = dir.to_string_lossy().into_owned();
    let accept = g["writers"]["rust"]["accept"].as_array().expect("accept");
    assert!(!accept.is_empty());
    for row in accept {
        let (action, by) = (row[0].as_str().unwrap(), row[1].as_str().unwrap());
        let path = write_host_request(&d, action, by).unwrap();
        assert_eq!(Path::new(&path), dir.join("host.request"));
        let text = std::fs::read_to_string(&path).unwrap();
        assert!(text.ends_with('\n'), "{text:?}");
        let doc: Value = serde_json::from_str(&text).unwrap();
        let obj = doc.as_object().unwrap();
        assert_eq!(obj.len(), 3, "{text}");
        assert_eq!(doc["action"], action);
        assert_eq!(doc["requested_by"], by);
        assert!(doc["time"].is_i64(), "{text}");
        assert!(!dir.join("host.request.tmp").exists());
    }
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn the_text_is_the_one_the_consumer_was_run_on() {
    let g = golden();
    let dir = scratch("text");
    let d = dir.to_string_lossy().into_owned();
    let path = write_host_request(&d, "update", "app").unwrap();
    let text = std::fs::read_to_string(&path).unwrap();
    let doc: Value = serde_json::from_str(&text).unwrap();
    let want: Value = serde_json::from_str(g["writers"]["rust"]["text_update_app"].as_str().unwrap()).unwrap();
    let golden_text = g["writers"]["rust"]["text_update_app"].as_str().unwrap();
    assert_eq!(text, golden_text.replace(&want["time"].to_string(), &doc["time"].to_string()));
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn refused_actions_write_nothing() {
    let g = golden();
    let dir = scratch("reject");
    let d = dir.to_string_lossy().into_owned();
    let reject = g["writers"]["rust"]["reject"].as_array().expect("reject");
    assert!(!reject.is_empty());
    for row in reject {
        let action = row.as_str().unwrap();
        let err = write_host_request(&d, action, "app").unwrap_err();
        assert_eq!(err.kind(), std::io::ErrorKind::InvalidInput, "{action:?}");
        assert!(!dir.join("host.request").exists(), "{action:?}");
    }
    let _ = std::fs::remove_dir_all(&dir);
}
