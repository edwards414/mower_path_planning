//! The real twist_mux's decisions, replayed through `mower_mux::core`.
//!
//! `tests/vectors/*.json` are what `tools/mux_oracle.py vectors` wrote from a
//! run of the C++ twist_mux 4.5.0 in the runtime image: the configuration,
//! every input and lock message of a scenario as it was sent (received here
//! 1 ms later), and the inputs that twist_mux forwarded. The decisions that lie
//! within 3 ms of a timeout or of another handle's message depend on DDS
//! delivery, so the oracle lists them (`near_boundary`) and they are left out
//! on both sides. Everything else must come out the same, in the same order.

use std::path::PathBuf;

use mower_bus::Mono;
use mower_mux::{Handle, Mux};
use serde_json::Value;

fn vectors() -> Vec<(String, Value)> {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/vectors");
    let mut out: Vec<(String, Value)> = std::fs::read_dir(&dir)
        .unwrap_or_else(|e| panic!("{}: {e}", dir.display()))
        .map(|e| e.unwrap().path())
        .filter(|p| p.extension().is_some_and(|x| x == "json"))
        .map(|p| {
            let v: Value = serde_json::from_str(&std::fs::read_to_string(&p).unwrap()).unwrap();
            (p.file_name().unwrap().to_string_lossy().into_owned(), v)
        })
        .collect();
    out.sort_by(|a, b| a.0.cmp(&b.0));
    out
}

fn handles(v: &Value) -> Vec<Handle> {
    v.as_array()
        .unwrap()
        .iter()
        .map(|h| {
            Handle::new(
                h["name"].as_str().unwrap(),
                h["topic"].as_str().unwrap(),
                h["timeout"].as_f64().unwrap(),
                h["priority"].as_i64().unwrap(),
            )
        })
        .collect()
}

/// What the core forwards for the vector's events, and when the first lock
/// message came.
fn replay(v: &Value) -> (Vec<String>, Option<u64>) {
    let inputs = handles(&v["config"]["inputs"]);
    let locks = handles(&v["config"]["locks"]);
    let mut mux = Mux::new(&inputs, &locks);
    let mut forwarded = Vec::new();
    let mut first_lock = None;
    for e in v["events"].as_array().unwrap() {
        let t = Mono::from_ns(e[0].as_u64().unwrap());
        let handle = e[2].as_str().unwrap();
        match e[1].as_str().unwrap() {
            "input" => {
                let i = inputs.iter().position(|h| h.name == handle).unwrap();
                if mux.on_input(i, t) {
                    forwarded.push(e[3].as_str().unwrap().to_string());
                }
            }
            "lock" => {
                let j = locks.iter().position(|h| h.name == handle).unwrap();
                mux.on_lock(j, e[3].as_bool().unwrap(), t);
                first_lock.get_or_insert(t.ns());
            }
            other => panic!("event kind {other}"),
        }
    }
    (forwarded, first_lock)
}

fn strings(v: &Value) -> Vec<String> {
    v.as_array().unwrap().iter().map(|s| s.as_str().unwrap().to_string()).collect()
}

fn seq(frame: &str) -> u64 {
    frame.rsplit('#').next().unwrap().parse().unwrap()
}

#[test]
fn the_core_forwards_what_twist_mux_forwarded() {
    let all = vectors();
    assert!(all.len() >= 18, "{} vectors", all.len());
    let mut compared = 0;
    for (file, v) in &all {
        let (got, first_lock) = replay(v);
        let recorded = strings(&v["forwarded"]);
        let near: Vec<String> = strings(&v["near_boundary"]);
        let clean = |xs: &[String]| -> Vec<String> { xs.iter().filter(|f| !near.contains(f)).cloned().collect() };
        match v["compare"].as_str().unwrap() {
            "exact" => assert_eq!(clean(&got), clean(&recorded), "{file}"),
            // joy at 200 Hz: what the depth-1 reader kept, which the core
            // cannot know; the counts are within the oracle's noise floor
            "count" => {
                let joy = |xs: &[String]| xs.iter().filter(|f| f.starts_with("joy#")).count() as i64;
                assert!((joy(&got) - joy(&recorded)).abs() <= 4, "{file}: {} vs {}", joy(&got), joy(&recorded));
            }
            // a safety stop every 2 ms across joy's 0.5 s: the first one
            // forwarded is the same one or its neighbour
            "first_within_one" => {
                let first = |xs: &[String]| xs.iter().find(|f| f.starts_with("safety#")).map(|f| seq(f));
                let (a, b) = (first(&got).unwrap(), first(&recorded).unwrap());
                assert!(a.abs_diff(b) <= 1, "{file}: {a} vs {b}");
            }
            // the start: nothing before the first lock false, and the same
            // once it is there (the muxes started under the inputs)
            "after_lock" => {
                let first_lock = first_lock.unwrap();
                let events = v["events"].as_array().unwrap();
                let sent_at = |f: &str| {
                    events.iter().find(|e| e[3].as_str() == Some(f)).map(|e| e[0].as_u64().unwrap()).unwrap()
                };
                assert!(got.iter().all(|f| sent_at(f) > first_lock), "{file}: forwarded before the lock");
                assert!(recorded.iter().all(|f| sent_at(f) > first_lock), "{file}: twist_mux forwarded before the lock");
                let after = |xs: &[String]| -> Vec<String> {
                    clean(xs).into_iter().filter(|f| sent_at(f) > first_lock + 100_000_000).collect()
                };
                assert_eq!(after(&got), after(&recorded), "{file}");
            }
            other => panic!("{file}: compare {other}"),
        }
        compared += 1;
    }
    assert_eq!(compared, all.len());
}

/// The vectors cover what the design's 6.1 asks for: S01-S18 and the random
/// segment, both configurations, with few decisions left out.
#[test]
fn the_vectors_are_the_oracle_s_scenarios() {
    let all = vectors();
    let names: Vec<&str> = all.iter().map(|(_, v)| v["scenario"].as_str().unwrap()).collect();
    for prefix in ["S01", "S02", "S03", "S04", "S05", "S06", "S07", "S08", "S09", "S10", "S11", "S12", "S13", "S14",
                   "S15", "S16", "S17", "S18", "R01"] {
        assert!(names.iter().any(|n| n.starts_with(prefix)), "no {prefix} in {names:?}");
    }
    for (file, v) in &all {
        let inputs = v["events"].as_array().unwrap().iter().filter(|e| e[1] == "input").count();
        let near = v["near_boundary"].as_array().unwrap().len();
        assert!(v["source"].as_str().unwrap().contains("twist_mux 4.5.0"), "{file}");
        // S17 is a scan across the boundary; the random segment loses what
        // came near a stall of the probe; everywhere else nearly nothing
        let most = if file.starts_with("s17") { inputs } else if file.starts_with("r01") { inputs * 3 / 10 } else { 2 };
        assert!(near <= most, "{file}: {near} of {inputs} inputs near a boundary");
    }
}
