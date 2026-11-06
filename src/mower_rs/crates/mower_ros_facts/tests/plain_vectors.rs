//! `mower_msgs` against what r2r and rmw wrote, with no ROS (E9-22). For every
//! type of the frozen vectors, `cdr_vectors.txt` and `interface_vectors.txt`: the
//! plain value read from rmw's bytes must serialize to r2r's JSON of that value, and
//! to rmw's bytes. Its default must be r2r's, in JSON and in bytes
//! (`interface_defaults.txt`), and its constants r2r's (`interface_constants.txt`).
//!
//! `mower_msgs_ros/tests/parity.rs` writes the `interface_*` files and holds them to
//! r2r on every `cargo test` while r2r exists.

use mower_msgs::MessageType;
use std::collections::BTreeMap;
use std::path::Path;

fn facts(name: &str) -> String {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../deploy/test/golden/ros_facts").join(name);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()))
}

fn unhex(s: &str) -> Vec<u8> {
    assert!(s.len() % 2 == 0, "odd hex length");
    (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
}

/// One value of `T`, from rmw's bytes.
fn check_vector<T: MessageType>(name: &str, label: &str, rmw: &[u8], r2r_json: &str) {
    let value = mower_cdr::decode(name, rmw).unwrap_or_else(|e| panic!("{name} {label}: decode: {e}"));
    let typed: T = mower_cdr::from_value(&value).unwrap_or_else(|e| panic!("{name} {label}: from_value: {e}"));
    let json = serde_json::to_value(&typed).unwrap().to_string();
    assert!(json == r2r_json, "{name} {label}: not r2r's JSON\n  r2r:   {r2r_json}\n  plain: {json}");
    let ours = mower_cdr::to_vec(name, &typed).unwrap_or_else(|e| panic!("{name} {label}: to_vec: {e}"));
    assert!(ours == mower_cdr::pad4(rmw.to_vec()), "{name} {label}: not what rmw wrote");
}

fn check_default<T: MessageType>(name: &str, rmw: &[u8], r2r_json: &str) {
    let plain = T::default();
    let json = serde_json::to_value(&plain).unwrap().to_string();
    assert!(json == r2r_json, "{name}: its default is not r2r's\n  r2r:   {r2r_json}\n  plain: {json}");
    let ours = mower_cdr::to_vec(name, &plain).unwrap_or_else(|e| panic!("{name}: to_vec: {e}"));
    assert!(ours == mower_cdr::pad4(rmw.to_vec()), "{name}: the bytes of its default are not rmw's");
    // `#[serde(default)]`: nothing in, the default out
    let empty: T = serde_json::from_str("{}").unwrap();
    assert!(empty == plain, "{name}: {{}} is not its default");
}

struct Entry {
    name: &'static str,
    vector: fn(&str, &str, &[u8], &str),
    default: fn(&str, &[u8], &str),
}

macro_rules! entries {
    ($($name:literal => $($seg:ident)::+;)*) => {
        vec![$(Entry {
            name: $name,
            vector: check_vector::<mower_msgs::$($seg)::+>,
            default: check_default::<mower_msgs::$($seg)::+>,
        }),*]
    };
}

fn table() -> BTreeMap<&'static str, Entry> {
    let table: Vec<Entry> = mower_msgs::for_each_message!(entries);
    table.into_iter().map(|e| (e.name, e)).collect()
}

/// A generated struct that is `T` by name must be what the schema's file says, in
/// both of the frozen sets: a type left out here would be left out of the proof.
#[test]
fn the_types_are_the_schema_s() {
    let names: Vec<String> = table().into_keys().map(str::to_string).collect();
    assert_eq!(names, mower_schema::message_names());
}

#[test]
fn plain_values_serialize_to_r2r_s_json_and_rmw_s_bytes() {
    let table = table();
    let (mut seen, mut total) = (BTreeMap::<String, usize>::new(), 0usize);
    for file in ["cdr_vectors.txt", "interface_vectors.txt"] {
        for line in facts(file).lines() {
            let mut cols = line.split('\t');
            let (name, index, hex, r2r_json) = (cols.next().unwrap(), cols.next().unwrap(), cols.next().unwrap(), cols.next().unwrap());
            let entry = table.get(name).unwrap_or_else(|| panic!("{file}: {name} is not generated"));
            (entry.vector)(name, index, &unhex(hex), r2r_json);
            *seen.entry(name.to_string()).or_default() += 1;
            total += 1;
        }
    }
    let names: Vec<String> = seen.keys().cloned().collect();
    assert_eq!(names, mower_schema::message_names(), "a type has no vector");
    assert!(seen.values().all(|n| *n == 10), "{seen:?}");
    eprintln!("{} types, {total} vectors", seen.len());
}

#[test]
fn a_plain_default_is_r2r_s() {
    let table = table();
    let text = facts("interface_defaults.txt");
    let mut names = Vec::new();
    for line in text.lines() {
        let mut cols = line.split('\t');
        let (name, hex, r2r_json) = (cols.next().unwrap(), cols.next().unwrap(), cols.next().unwrap());
        (table.get(name).unwrap_or_else(|| panic!("{name} is not generated")).default)(name, &unhex(hex), r2r_json);
        names.push(name.to_string());
    }
    assert_eq!(names, mower_schema::message_names(), "a type has no frozen default");
}

macro_rules! constants {
    ($($ty:literal $name:literal $($seg:ident)::+;)*) => {
        vec![$(($ty, $name, mower_msgs::$($seg)::+ as i128)),*]
    };
}

#[test]
fn plain_constants_are_r2r_s() {
    let plain: Vec<(&str, &str, i128)> = mower_msgs::for_each_constant!(constants);
    let plain: Vec<String> = plain.iter().map(|(ty, name, v)| format!("{ty}\t{name}\t{v}")).collect();
    let frozen = facts("interface_constants.txt");
    let frozen: Vec<&str> = frozen.lines().collect();
    assert_eq!(plain, frozen);
    assert!(frozen.len() >= 100, "{}", frozen.len());
}
