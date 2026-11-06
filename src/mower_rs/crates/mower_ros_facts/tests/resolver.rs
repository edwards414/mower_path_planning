//! rcl's resolver oracle as vectors: `mower_rs_common::names` must give every
//! name of `resolver.json`, which `mower_bridge/tests/resolver.rs` proved on
//! rcl's graph, with no rcl here.
//!
//! The module is compiled in by path: `mower_rs_common` itself links r2r until
//! E9-25, and `names.rs` uses nothing of it.

#[path = "../../mower_rs_common/src/names.rs"]
#[allow(dead_code)]
mod names;

use names::{NameKind, Remaps};
use serde_json::Value;
use std::path::Path;

fn resolver() -> Value {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../deploy/test/golden/ros_facts/resolver.json");
    serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap()
}

fn production(doc: &Value) -> Remaps {
    let mut argv = vec!["--ros-args".to_string()];
    for rule in doc["rules"].as_array().unwrap() {
        argv.push("-r".to_string());
        argv.push(rule.as_str().unwrap().to_string());
    }
    Remaps::from_args(&argv).unwrap()
}

#[test]
fn every_name_resolves_as_rcl_resolved_it() {
    let doc = resolver();
    let remaps = production(&doc);
    let names = doc["names"].as_array().unwrap();
    assert!(names.len() >= 205, "the oracle had 205 names when the design was written, the file has {}", names.len());
    let mut wrong = Vec::new();
    for n in names {
        let (node, code, want) = (n["node"].as_str().unwrap(), n["code"].as_str().unwrap(), n["name"].as_str().unwrap());
        // rcl_action resolves an action's name with the topic rules
        let kind = match n["space"].as_str().unwrap() {
            "topic" | "action" => NameKind::Topic,
            "service" => NameKind::Service,
            other => panic!("{other}: no such space"),
        };
        match remaps.resolve(kind, node, "", code) {
            Ok(got) if got == want => {}
            got => wrong.push(format!("{node} {code}: rcl {want}, names.rs {got:?}")),
        }
    }
    assert!(wrong.is_empty(), "{} of {} names differ:\n{}", wrong.len(), names.len(), wrong.join("\n"));
}

/// A vector file that held only the names that need no rule would pass the
/// test above with any rules. This one needs them: each rule has to be used.
#[test]
fn every_rule_is_exercised() {
    let doc = resolver();
    let all = production(&doc);
    let names = doc["names"].as_array().unwrap();
    let rules: Vec<&str> = doc["rules"].as_array().unwrap().iter().map(|r| r.as_str().unwrap()).collect();
    let bare = Remaps::from_args(&[]).unwrap();
    let changed = |n: &Value, r: &Remaps| {
        let kind = if n["space"] == "service" { NameKind::Service } else { NameKind::Topic };
        let (node, code) = (n["node"].as_str().unwrap(), n["code"].as_str().unwrap());
        r.resolve(kind, node, "", code).unwrap() != bare.resolve(kind, node, "", code).unwrap()
    };
    let mut unused = Vec::new();
    for skip in &rules {
        let mut argv = vec!["--ros-args".to_string()];
        for rule in rules.iter().filter(|r| r != &skip) {
            argv.push("-r".to_string());
            argv.push(rule.to_string());
        }
        let without = Remaps::from_args(&argv).unwrap();
        if !names.iter().any(|n| changed(n, &all) != changed(n, &without)) {
            unused.push(*skip);
        }
    }
    assert!(unused.is_empty(), "no vector needs these rules: {unused:?}");
}
