//! The snapshots themselves: nothing host-specific in them, they agree with
//! each other and with `mower_schema`, and no file changed behind
//! `SHA256SUMS` (a wrong value in the graph or a parameter dump has nothing
//! else to fail against until the oracle job regenerates them).

use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

fn repo() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../..").canonicalize().unwrap()
}

fn facts() -> PathBuf {
    repo().join("deploy/test/golden/ros_facts")
}

fn json(name: &str) -> Value {
    serde_json::from_str(&std::fs::read_to_string(facts().join(name)).unwrap_or_else(|e| panic!("{name}: {e}")))
        .unwrap_or_else(|e| panic!("{name}: {e}"))
}

fn files(dir: &Path, out: &mut Vec<PathBuf>) {
    let mut entries: Vec<PathBuf> = std::fs::read_dir(dir).unwrap().map(|e| e.unwrap().path()).collect();
    entries.sort();
    for p in entries {
        if p.is_dir() {
            files(&p, out);
        } else {
            out.push(p);
        }
    }
}

fn facts_files() -> Vec<PathBuf> {
    let mut out = Vec::new();
    files(&facts(), &mut out);
    out
}

fn sums() -> BTreeMap<String, String> {
    std::fs::read_to_string(facts().join("SHA256SUMS"))
        .unwrap()
        .lines()
        .map(|l| {
            let (hash, path) = l.split_once("  ").unwrap_or_else(|| panic!("{l}"));
            (path.to_string(), hash.to_string())
        })
        .collect()
}

#[test]
fn no_file_changed_behind_the_manifest() {
    let sums = sums();
    assert!(sums.len() >= 15, "{}", sums.len());
    let mut wrong = Vec::new();
    for (path, want) in &sums {
        let bytes = std::fs::read(repo().join(path)).unwrap_or_else(|e| panic!("{path}: {e}"));
        let got: String = Sha256::digest(&bytes).iter().map(|b| format!("{b:02x}")).collect();
        if &got != want {
            wrong.push(path.as_str());
        }
    }
    assert!(wrong.is_empty(), "changed by hand? {wrong:?}: tools/oracle/regen.sh facts and freeze_stack.py write them, tools/oracle/facts_manifest.py lists them");
    let root = repo();
    for f in facts_files() {
        let rel = f.strip_prefix(&root).unwrap().to_string_lossy().to_string();
        let exempt = rel.ends_with("/README.md") || rel.ends_with("/PROVENANCE.txt") || rel.ends_with("/SHA256SUMS");
        assert!(exempt || sums.contains_key(&rel), "{rel} is not in SHA256SUMS");
    }
    // E3's corpus is frozen with the rest
    for name in ["publish.txt", "inbound.txt", "request.txt", "service_response.txt", "map_grid.frame"] {
        let rel = format!("src/mower_rs/crates/mower_ws_bridge/tests/golden/{name}");
        assert!(sums.contains_key(&rel), "{rel} is not in SHA256SUMS");
    }
}

/// The repo is public. What the stack was started with on purpose stays out:
/// the VPN address, home directories, the pairing fixture's secret and code.
#[test]
fn nothing_host_specific_or_secret() {
    let fixture = |name: &str| -> Value {
        serde_json::from_str(&std::fs::read_to_string(repo().join("tools/ab/fixtures").join(name)).unwrap()).unwrap()
    };
    let mut secrets: Vec<String> = Vec::new();
    for doc in [fixture("identity.json"), fixture("pair.json")] {
        for (key, value) in doc.as_object().unwrap() {
            if let Some(s) = value.as_str() {
                if s.len() >= 8 && key != "name" {
                    secrets.push(s.to_string());
                }
            }
        }
    }
    assert!(!secrets.is_empty());
    for f in facts_files() {
        if f.ends_with("README.md") {
            continue; // it says which of these it removes
        }
        let text = std::fs::read_to_string(&f).unwrap_or_default();
        let name = f.display();
        for needle in ["/Users/", "/home/", "/root/", "/runner/", "10.77.", "MOWER_PROVISION", "password", "token"] {
            assert!(!text.contains(needle), "{name} holds {needle:?}");
        }
        for secret in &secrets {
            assert!(!text.contains(secret.as_str()), "{name} holds a fixture's secret");
        }
        for word in text.split(|c: char| !(c.is_ascii_digit() || c == '.')) {
            let parts: Vec<&str> = word.split('.').collect();
            let ipv4 = parts.len() == 4 && parts.iter().all(|p| !p.is_empty() && p.len() <= 3 && p.parse::<u8>().is_ok());
            assert!(!ipv4 || word.starts_with("127."), "{name} holds the address {word}");
        }
    }
}

fn endpoints(graph: &Value) -> Vec<(&str, &Value)> {
    let mut out = Vec::new();
    for (topic, t) in graph["topics"].as_object().unwrap() {
        for side in ["publishers", "subscribers"] {
            for e in t[side].as_array().unwrap() {
                out.push((topic.as_str(), e));
            }
        }
    }
    out
}

/// The graph's types against the table E9-22 generates from: where the table
/// has a type, its RIHS01 hash is the one on the wire.
#[test]
fn the_graph_s_types_hash_as_mower_schema_says() {
    let graph = json("graph.json");
    let mut checked = BTreeSet::new();
    for (topic, e) in endpoints(&graph) {
        let ty = e["type"].as_str().unwrap();
        if let Some(hash) = mower_schema::type_hash(ty) {
            assert_eq!(e["hash"].as_str(), Some(hash), "{topic} {ty}");
            checked.insert(ty);
        }
    }
    assert!(checked.len() >= 15, "{checked:?}");
}

#[test]
fn the_graph_has_the_shape_later_steps_read() {
    let graph = json("graph.json");
    let nodes: BTreeSet<&str> = graph["nodes"].as_array().unwrap().iter().map(|n| n.as_str().unwrap()).collect();
    assert!(nodes.len() >= 30, "{}", nodes.len());
    assert!(nodes.contains("/mower_bus_bridge") && nodes.contains("/bt_navigator"), "not the production set: {nodes:?}");
    for (topic, e) in endpoints(&graph) {
        assert!(nodes.contains(e["node"].as_str().unwrap()), "{topic}: {e}");
        let q = &e["qos"];
        for field in ["reliability", "durability", "history", "liveliness"] {
            assert!(q[field].is_string(), "{topic}: {e}");
        }
        for field in ["depth", "deadline_ns", "lifespan_ns", "liveliness_lease_ns"] {
            assert!(q[field].is_number(), "{topic}: {e}");
        }
    }
    let services = graph["services"].as_object().unwrap();
    assert!(services.len() >= 100, "{}", services.len());
    for (name, s) in services {
        assert!(!s["types"].as_array().unwrap().is_empty(), "{name}");
        assert!(name.starts_with('/'), "{name}");
    }
}

/// launch's rules are the ones the resolver vectors were made with.
#[test]
fn launch_s_rules_are_the_manifest_s() {
    let args = json("launch/args.json");
    let resolver = json("resolver.json");
    let set = |v: &Value| -> BTreeSet<String> { v.as_array().unwrap().iter().map(|r| r.as_str().unwrap().to_string()).collect() };
    assert_eq!(set(&args["remaps"]), set(&resolver["rules"]));
    // each file the launch line names is there
    for f in args["params_files"].as_array().unwrap() {
        let f = f.as_str().unwrap();
        let path = if f.starts_with("src/") { repo().join(f) } else { facts().join(f) };
        assert!(path.exists(), "{f}");
    }
}

fn yaml_to_json(y: &serde_yaml::Value) -> Value {
    serde_json::to_value(y).unwrap()
}

/// `{"double": 0.75}` -> 0.75, the way a dump's YAML reads.
fn override_value(entry: &Value) -> &Value {
    entry.as_object().unwrap().values().next().unwrap()
}

/// What a parameter dump says a node runs with, where the production files set
/// the parameter, is what rcl read from them for that node. So a dump's value
/// cannot be altered alone, and neither can an override.
#[test]
fn a_live_dump_agrees_with_the_overrides_rcl_read() {
    let overrides = json("param_overrides.json");
    let mut dumps = Vec::new();
    files(&facts().join("params"), &mut dumps);
    assert!(dumps.len() >= 5, "{dumps:?}");
    let mut compared = 0;
    let mut wrong = Vec::new();
    for path in dumps {
        let doc: serde_yaml::Value = serde_yaml::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        let (node, body) = doc.as_mapping().unwrap().iter().next().unwrap();
        let node = node.as_str().unwrap().trim_start_matches('/');
        let params = yaml_to_json(&body["ros__parameters"]);
        let mut flat = BTreeMap::new();
        flatten("", &params, &mut flat);
        let set = overrides[node].as_object().unwrap_or_else(|| panic!("{node}: not in param_overrides.json"));
        for (name, entry) in set {
            let Some(live) = flat.get(name.as_str()) else { continue };
            compared += 1;
            let want = override_value(entry);
            // YAML cannot tell 1.0 from 1 after a round through a dump's double
            let same = live == want || (live.as_f64().is_some() && live.as_f64() == want.as_f64());
            if !same {
                wrong.push(format!("{node} {name}: dump {live}, rcl read {want}"));
            }
        }
    }
    assert!(compared >= 20, "only {compared} parameters are in both");
    assert!(wrong.is_empty(), "{}", wrong.join("\n"));
}

fn flatten(prefix: &str, v: &Value, out: &mut BTreeMap<String, Value>) {
    match v {
        Value::Object(map) => {
            for (k, v) in map {
                let name = if prefix.is_empty() { k.clone() } else { format!("{prefix}.{k}") };
                flatten(&name, v, out);
            }
        }
        other => {
            out.insert(prefix.to_string(), other.clone());
        }
    }
}

/// D5 of E9's design: the console format E9-24 must keep.
#[test]
fn every_log_line_has_rcutils_format_or_is_a_known_plain_line() {
    let text = std::fs::read_to_string(facts().join("log/startup.txt")).unwrap();
    let mut loggers = BTreeSet::new();
    for line in text.lines() {
        if line.starts_with("[mower_rsd] ") || line.starts_with("[mower_agent] ") {
            continue; // println!, not the logger
        }
        let rest = ["[DEBUG] ", "[INFO] ", "[WARN] ", "[ERROR] ", "[FATAL] "]
            .iter()
            .find_map(|l| line.strip_prefix(l))
            .unwrap_or_else(|| panic!("no level: {line}"));
        let rest = rest.strip_prefix("[T] [").unwrap_or_else(|| panic!("no stamp: {line}"));
        let (logger, msg) = rest.split_once("]: ").unwrap_or_else(|| panic!("no logger: {line}"));
        assert!(!logger.is_empty() && !msg.is_empty(), "{line}");
        loggers.insert(logger.to_string());
    }
    let overrides = json("param_overrides.json");
    let known: BTreeSet<String> = overrides.as_object().unwrap().keys().cloned().collect();
    let foreign: Vec<&String> = loggers.difference(&known).collect();
    assert!(loggers.len() >= 15, "{loggers:?}");
    assert!(foreign.is_empty(), "loggers that are no module node: {foreign:?}");
}
