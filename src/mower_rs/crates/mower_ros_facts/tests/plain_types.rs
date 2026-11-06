//! That the generated types are the ones that matter (E9-22): every type a crate
//! names through r2r, every type of `resolver.json`, and of `graph.json` all but
//! what no crate names; the hashes where the graph has one. A new `r2r::...` type in a
//! crate fails here until `gen_schema.py` has it, so the list cannot go stale before
//! the crates switch.

use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

fn crates_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("..").canonicalize().unwrap()
}

fn json(name: &str) -> Value {
    let path = crates_dir().join("../../../deploy/test/golden/ros_facts").join(name);
    serde_json::from_str(&std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()))).unwrap()
}

/// Every interface there is here, as `pkg/msg/T`, `pkg/srv/T`, `pkg/action/T`.
fn generated() -> BTreeSet<String> {
    let mut out: BTreeSet<String> = mower_schema::interfaces().iter().map(|i| i.name.to_string()).collect();
    out.extend(mower_schema::message_names().into_iter().filter(|n| n.contains("/msg/")));
    out
}

fn rust_files(dir: &Path, out: &mut Vec<PathBuf>) {
    for entry in std::fs::read_dir(dir).unwrap() {
        let path = entry.unwrap().path();
        if path.is_dir() {
            if path.file_name().is_some_and(|n| n != "target") {
                rust_files(&path, out);
            }
        } else if path.extension().is_some_and(|e| e == "rs") {
            out.push(path);
        }
    }
}

fn without_comments(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let (b, mut i) = (text.as_bytes(), 0);
    let (mut in_string, mut depth) = (false, 0);
    while i < b.len() {
        let two = b.get(i..i + 2);
        if depth > 0 {
            if two == Some(b"*/") {
                depth -= 1;
                i += 2;
            } else if two == Some(b"/*") {
                depth += 1;
                i += 2;
            } else {
                i += 1;
            }
        } else if in_string {
            let c = text[i..].chars().next().unwrap();
            out.push(c);
            i += c.len_utf8();
            match c {
                '\\' => {
                    if let Some(next) = text[i..].chars().next() {
                        out.push(next);
                        i += next.len_utf8();
                    }
                }
                '"' => in_string = false,
                _ => {}
            }
        } else if two == Some(b"//") {
            while i < b.len() && b[i] != b'\n' {
                i += 1;
            }
        } else if two == Some(b"/*") {
            depth += 1;
            i += 2;
        } else if text[i..].starts_with("'\"'") {
            // a quote as a char starts no string
            i += 3;
        } else {
            in_string = b[i] == b'"';
            let c = text[i..].chars().next().unwrap();
            out.push(c);
            i += c.len_utf8();
        }
    }
    out
}

/// The paths of a `use` tree's text below `prefix`: `a::{b, c::d as e, f::*}`. Each with
/// the name it is known by.
fn use_paths(tree: &str, prefix: &str, out: &mut Vec<(String, String)>) {
    let mut item = String::new();
    let mut depth = 0;
    let mut items = Vec::new();
    for c in tree.chars() {
        match c {
            '{' => depth += 1,
            '}' => depth -= 1,
            ',' if depth == 0 => {
                items.push(std::mem::take(&mut item));
                continue;
            }
            _ => {}
        }
        item.push(c);
    }
    items.push(item);
    for item in items {
        let item: String = item.split_whitespace().collect::<Vec<_>>().join(" ");
        if item.is_empty() {
            continue;
        }
        if let (Some(open), true) = (item.find('{'), item.ends_with('}')) {
            let head = item[..open].trim_end_matches("::");
            let next = if head.is_empty() { prefix.to_string() } else { format!("{prefix}::{head}") };
            use_paths(&item[open + 1..item.len() - 1], &next, out);
            continue;
        }
        let (path, alias) = match item.split_once(" as ") {
            Some((p, a)) => (p.trim(), Some(a.trim())),
            None => (item.as_str(), None),
        };
        let path = path.strip_suffix("::self").unwrap_or(path);
        let name = alias.unwrap_or_else(|| path.rsplit("::").next().unwrap_or(path)).to_string();
        out.push((format!("{prefix}::{path}"), name));
    }
}

/// `r2r::pkg::kind::Type...` -> `pkg/kind/Type`.
fn interface(path: &str) -> Option<String> {
    let parts: Vec<&str> = path.split("::").collect();
    match parts.as_slice() {
        ["r2r", pkg, kind @ ("msg" | "srv" | "action"), ty, ..] => Some(format!("{pkg}/{kind}/{ty}")),
        _ => None,
    }
}

fn ident(c: char) -> bool {
    c.is_ascii_alphanumeric() || c == '_'
}

/// Every interface a source text names through r2r: in a `use`, on the spot, or by a
/// module it has imported (`use r2r::rcl_interfaces::msg; ... msg::Log`).
fn named_in(text: &str) -> BTreeSet<String> {
    let text = without_comments(text);
    let mut found = BTreeSet::new();
    let mut rest = text.clone();
    let mut modules = Vec::new();
    let mut plain_text = String::new();
    // `use` statements first, taken out of the text
    while let Some(at) = rest.find("use r2r::") {
        let before = rest[..at].chars().last();
        let tail = &rest[at + "use r2r::".len()..];
        let end = tail.find(';').unwrap_or(tail.len());
        if before.is_some_and(ident) {
            plain_text.push_str(&rest[..at + 4]);
            rest = rest[at + 4..].to_string();
            continue;
        }
        let mut paths = Vec::new();
        use_paths(&tail[..end], "r2r", &mut paths);
        for (path, name) in paths {
            if let Some(i) = interface(&path) {
                found.insert(i);
            } else if ["msg", "srv", "action"].iter().any(|k| path.ends_with(&format!("::{k}"))) {
                modules.push((path, name));
            }
        }
        plain_text.push_str(&rest[..at]);
        rest = tail[end..].to_string();
    }
    plain_text.push_str(&rest);
    // then every path left
    let path_at = |from: usize, prefix: &str, found: &mut BTreeSet<String>| {
        let tail = &plain_text[from..];
        let end = tail.find(|c: char| !(ident(c) || c == ':')).unwrap_or(tail.len());
        if let Some(i) = interface(&format!("{prefix}{}", &tail[..end])) {
            found.insert(i);
        }
    };
    for (at, _) in plain_text.match_indices("r2r::") {
        if !plain_text[..at].chars().last().is_some_and(ident) {
            path_at(at + "r2r::".len(), "r2r::", &mut found);
        }
    }
    for (path, name) in modules {
        let word = format!("{name}::");
        for (at, _) in plain_text.match_indices(&word) {
            if !plain_text[..at].chars().last().is_some_and(|c| ident(c) || c == ':') {
                path_at(at + word.len(), &format!("{path}::"), &mut found);
            }
        }
    }
    found
}

#[test]
fn the_reader_of_r2r_paths_reads_them() {
    let text = r#"
        // use r2r::comment_msgs::msg::NotThis;
        use r2r::a_msgs::msg::{One, Two as Deux, three::Four};
        use r2r::{b_msgs::srv::Five, c_msgs::{action::Six, msg::Seven}};
        use r2r::d_msgs::msg as dm;
        use r2r::QosProfile;
        fn f(x: r2r::e_msgs::msg::Eight, y: dm::Nine, z: r2r::f_msgs::srv::Ten::Request) -> r2r::Result<()> { "no r2r::g_msgs::msg::Str"; }
    "#;
    let found: Vec<String> = named_in(text).into_iter().collect();
    assert_eq!(found, [
        "a_msgs/msg/One", "a_msgs/msg/Two", "a_msgs/msg/three", "b_msgs/srv/Five", "c_msgs/action/Six", "c_msgs/msg/Seven",
        "d_msgs/msg/Nine", "e_msgs/msg/Eight", "f_msgs/srv/Ten", "g_msgs/msg/Str",
    ]);
}

#[test]
fn every_type_a_crate_names_through_r2r_is_generated() {
    let root = crates_dir();
    let mut files = Vec::new();
    rust_files(&root, &mut files);
    assert!(files.len() > 200, "{}", files.len());
    let have = generated();
    let mut named: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for file in files {
        let rel = file.strip_prefix(&root).unwrap().display().to_string();
        // this file's own examples, and the conversions, which say r2r's name of each type
        if rel.starts_with("mower_msgs") || rel == "mower_ros_facts/tests/plain_types.rs" {
            continue;
        }
        for name in named_in(&std::fs::read_to_string(&file).unwrap()) {
            named.entry(name).or_default().push(rel.clone());
        }
    }
    let missing: Vec<String> = named.iter().filter(|(n, _)| !have.contains(*n)).map(|(n, f)| format!("{n} in {f:?}")).collect();
    assert!(missing.is_empty(), "not generated: add to INTERFACE_ROOTS in gen_schema.py:\n{}", missing.join("\n"));
    assert!(named.len() >= 80, "{}", named.len());
    // and what is a root without a crate naming it says why
    let unused: Vec<&&str> = mower_schema::interface_roots().iter().filter(|r| !named.contains_key(**r)).collect();
    assert_eq!(unused, [&"action_msgs/msg/GoalStatusArray", &"action_msgs/srv/CancelGoal", &"rcl_interfaces/msg/ParameterEvent"],
        "these are generated for the graph alone (gen_schema.py); a root neither names is stale");
}

#[test]
fn every_type_of_the_resolver_is_generated() {
    let have = generated();
    let resolver = json("resolver.json");
    let names = resolver["names"].as_array().map(|a| a.to_vec()).unwrap_or_else(|| {
        resolver["names"].as_object().expect("names").values().cloned().collect()
    });
    let mut types: Vec<String> = Vec::new();
    fn walk(v: &Value, out: &mut Vec<String>) {
        match v {
            Value::Object(o) => {
                for (k, v) in o {
                    match (k.as_str(), v) {
                        ("type", Value::String(t)) => out.push(t.clone()),
                        _ => walk(v, out),
                    }
                }
            }
            Value::Array(a) => a.iter().for_each(|v| walk(v, out)),
            _ => {}
        }
    }
    walk(&Value::Array(names), &mut types);
    types.sort();
    types.dedup();
    assert!(types.len() >= 45, "{}", types.len());
    let missing: Vec<&String> = types.iter().filter(|t| !have.contains(*t)).collect();
    assert!(missing.is_empty(), "{missing:?}");
}

/// On the graph, but no crate names them: nav2's own nodes and rcl's, whose endpoints
/// go when they go. Listed, so that one that appears has to be looked at.
fn left_out(ty: &str) -> bool {
    ty.starts_with("composition_interfaces/")
        || ty.starts_with("type_description_interfaces/")
        || ty.starts_with("map_msgs/")
        || (ty.starts_with("nav2_msgs/") && !ty.contains("/FollowPath_") && !ty.contains("/NavigateToPose_"))
        || ["geometry_msgs/msg/PointStamped", "geometry_msgs/msg/Polygon", "geometry_msgs/msg/PolygonStamped",
            "lifecycle_msgs/msg/TransitionEvent", "lifecycle_msgs/srv/ChangeState",
            "lifecycle_msgs/srv/GetAvailableStates", "lifecycle_msgs/srv/GetAvailableTransitions"].contains(&ty)
}

#[test]
fn every_type_of_the_graph_is_generated_or_nav2_s_and_its_hash_is_the_graph_s() {
    let graph = json("graph.json");
    let have = generated();
    // an action's own endpoints have the part's name, the parts rosidl makes
    let known = |ty: &str| {
        have.contains(ty) || mower_schema::any_type_hash(ty).is_some() || mower_schema::message_names().iter().any(|n| n == ty)
    };
    let (mut types, mut hashes) = (BTreeSet::new(), 0);
    for (name, topic) in graph["topics"].as_object().expect("topics") {
        for ty in topic["types"].as_array().unwrap() {
            types.insert(ty.as_str().unwrap().to_string());
        }
        for side in ["publishers", "subscribers"] {
            for end in topic[side].as_array().unwrap() {
                let ty = end["type"].as_str().unwrap();
                if let (Some(hash), false) = (end["hash"].as_str(), left_out(ty)) {
                    assert_eq!(mower_schema::any_type_hash(ty), Some(hash), "{name}: {ty}");
                    hashes += 1;
                }
            }
        }
    }
    for service in graph["services"].as_object().expect("services").values() {
        for ty in service["types"].as_array().unwrap() {
            types.insert(ty.as_str().unwrap().to_string());
        }
    }
    let missing: Vec<&String> = types.iter().filter(|t| !known(t) && !left_out(t)).collect();
    assert!(missing.is_empty(), "on the graph, neither generated nor listed as nav2's or rcl's: {missing:?}");
    assert!(hashes > 100, "{hashes}");
}
