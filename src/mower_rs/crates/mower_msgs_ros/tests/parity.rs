//! Every type of `mower_msgs` against r2r's (E9-22): while r2r is here it is the
//! oracle, so a value of each type, its default, its JSON both ways, what serde
//! refuses, its CDR and its constants must be r2r's. The conversion's every field is
//! named at compile time; here it must also carry every bit, NaN included.
//!
//! `the_frozen_files_are_what_r2r_and_rmw_say` writes what `mower_ros_facts` replays
//! with no ROS (`MOWER_GOLDEN=write`, `tools/oracle/regen.sh facts`), for the types
//! `cdr_vectors.txt` does not have, and the defaults and constants of all of them.

#[path = "../../mower_cdr/tests/support/random_value.rs"]
#[allow(dead_code)]
mod random_value;

use mower_cdr::Value;
use mower_msgs::MessageType;
use mower_msgs_ros::{ActionMirror, RosMirror, ServiceMirror};
use r2r::WrappedTypesupport;
use random_value::{random, Rng};
use serde_json::{json, Value as Json};
use std::any::type_name;
use std::path::{Path, PathBuf};
use std::sync::Once;

/// As `mower_cdr/tests/rmw_oracle.rs` says why: the robot's RMW.
fn robot_rmw() {
    static ONCE: Once = Once::new();
    ONCE.call_once(|| {
        let lib = std::env::var("AMENT_PREFIX_PATH")
            .unwrap_or_default()
            .split(':')
            .any(|p| Path::new(p).join("lib/librmw_cyclonedds_cpp.so").exists());
        assert!(lib, "the oracle runs on the robot's RMW: apt-get install ros-jazzy-rmw-cyclonedds-cpp");
        std::env::set_var("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp");
    });
}

fn facts(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../deploy/test/golden/ros_facts").join(name)
}

/// rosidl's member of an empty message: r2r has no such field, and `random`'s value
/// for it would be skipped by serde anyway.
const SYNTHETIC: &str = "structure_needs_at_least_one_member";

fn without_synthetic(v: Value) -> Value {
    match v {
        Value::Struct(fields) => Value::Struct(
            fields.into_iter().filter(|(n, _)| n != SYNTHETIC).map(|(n, v)| (n, without_synthetic(v))).collect(),
        ),
        Value::Array(items) => Value::Array(items.into_iter().map(without_synthetic).collect()),
        other => other,
    }
}

/// What serde makes of `text`: a value, as its JSON and its bytes, or the error's text.
fn parsed<T: MessageType, R: WrappedTypesupport>(name: &str, text: &str, what: &str) {
    let t = serde_json::from_str::<T>(text);
    let r = serde_json::from_str::<R>(text);
    match (t, r) {
        (Ok(t), Ok(r)) => {
            assert_eq!(serde_json::to_string(&t).unwrap(), serde_json::to_string(&r).unwrap(), "{name} {what}: {text}");
        }
        (Err(t), Err(r)) => assert_eq!(t.to_string(), r.to_string(), "{name} {what}: {text}"),
        (t, r) => panic!("{name} {what}: {text}\n  plain {:?}\n  r2r   {:?}", t.map(|_| ()), r.map(|_| ())),
    }
}

/// serde's verdict on every way a client can get a frame wrong, for one object.
fn check_parsing<T: MessageType, R: WrappedTypesupport>(name: &str, value: &Json) {
    for text in ["{}", "[]", "null", "1", "\"x\"", "true", "{\"no_such_field\":1}", "[1]", "[[],[]]"] {
        parsed::<T, R>(name, text, "whole");
    }
    let Json::Object(fields) = value else { panic!("{name}: not an object") };
    for key in fields.keys() {
        let mut less = fields.clone();
        less.remove(key);
        parsed::<T, R>(name, &Json::Object(less).to_string(), "a field left out");
        for wrong in [json!(null), json!("x"), json!(true), json!(1), json!(-1), json!(1.5), json!(1e40), json!(300),
            json!(70000), json!(5000000000u64), json!([]), json!([1]), json!(["x"]), json!({}), json!({"a": 1})]
        {
            let mut changed = fields.clone();
            changed.insert(key.clone(), wrong);
            parsed::<T, R>(name, &Json::Object(changed).to_string(), "a field of the wrong kind");
        }
    }
}

fn check_default<T: RosMirror>(name: &str) {
    let ctx = |what: &str| format!("{name}: default: {what}");
    let (t, r) = (T::default(), T::Ros::default());
    assert_eq!(serde_json::to_string(&t).unwrap(), serde_json::to_string(&r).unwrap(), "{}", ctx("JSON"));
    let rmw = r.to_serialized_bytes().unwrap();
    assert!(mower_cdr::to_vec(name, &t).unwrap() == mower_cdr::pad4(rmw.clone()), "{}", ctx("CDR"));
    assert!(t.clone().into_ros() == r, "{}", ctx("into_ros"));
    assert!(T::from_ros(r) == t, "{}", ctx("from_ros"));
}

fn check_values<T: RosMirror>(name: &str, rng: &mut Rng, n: usize) {
    assert_eq!(T::NAME, name);
    // bus introspection reads the Rust path: it must say what it says of r2r's
    assert_eq!(mower_bus::ros_type_name(type_name::<T>()), mower_bus::ros_type_name(type_name::<T::Ros>()), "{name}");
    check_default::<T>(name);
    for i in 0..n {
        let v = without_synthetic(random(rng, name, 0));
        let ctx = || format!("{name} #{i}: {v:?}");
        let t: T = mower_cdr::from_value(&v).unwrap_or_else(|e| panic!("{}: from_value: {e}", ctx()));
        let r: T::Ros = mower_cdr::from_value(&v).unwrap_or_else(|e| panic!("{}: from_value (r2r): {e}", ctx()));
        // what the app sees: in struct order, and sorted as a Value
        let (text_t, text_r) = (serde_json::to_string(&t).unwrap(), serde_json::to_string(&r).unwrap());
        assert_eq!(text_t, text_r, "{}: JSON", ctx());
        let tree = serde_json::to_value(&t).unwrap();
        assert_eq!(tree, serde_json::to_value(&r).unwrap(), "{}: JSON tree", ctx());
        // what is on the wire: bit for bit, NaN too
        let rmw = r.to_serialized_bytes().unwrap();
        let want = mower_cdr::pad4(rmw.clone());
        assert!(mower_cdr::to_vec(name, &t).unwrap() == want, "{}: mower_cdr of the plain value", ctx());
        assert!(t.clone().into_ros().to_serialized_bytes().unwrap() == rmw, "{}: into_ros", ctx());
        assert!(mower_cdr::to_vec(name, &T::from_ros(r.clone())).unwrap() == want, "{}: from_ros", ctx());
        // JSON back in: a text with null (NaN) in it must be refused by both, or read alike
        parsed::<T, T::Ros>(name, &text_r, "its own text");
        parsed::<T, T::Ros>(name, &tree.to_string(), "its own sorted text");
        if i == 0 {
            check_parsing::<T, T::Ros>(name, &tree);
        }
        // and a type r2r reads back from the bytes it wrote
        let again = T::Ros::from_serialized_bytes(&want).unwrap();
        assert!(again.to_serialized_bytes().unwrap() == rmw, "{}: r2r read it back differently", ctx());
    }
}

fn vector_lines<T: RosMirror>(name: &str, text: &mut String) {
    use std::fmt::Write;
    for i in 0..VECTORS_PER_TYPE {
        let v = without_synthetic(random(&mut Rng::for_vector(name, i), name, 0));
        let r: T::Ros = mower_cdr::from_value(&v).unwrap();
        let hex: String = r.to_serialized_bytes().unwrap().iter().map(|b| format!("{b:02x}")).collect();
        writeln!(text, "{name}\t{i}\t{hex}\t{}", serde_json::to_value(&r).unwrap()).unwrap();
    }
}

fn default_line<T: RosMirror>(name: &str, text: &mut String) {
    use std::fmt::Write;
    let r = T::Ros::default();
    let hex: String = r.to_serialized_bytes().unwrap().iter().map(|b| format!("{b:02x}")).collect();
    writeln!(text, "{name}\t{hex}\t{}", serde_json::to_value(&r).unwrap()).unwrap();
}

const VECTORS_PER_TYPE: u32 = 10;

struct Entry {
    name: &'static str,
    values: fn(&str, &mut Rng, usize),
    vectors: fn(&str, &mut String),
    default: fn(&str, &mut String),
}

macro_rules! entries {
    ($($name:literal => $($seg:ident)::+;)*) => {
        vec![$(Entry {
            name: $name,
            values: check_values::<mower_msgs::$($seg)::+>,
            vectors: vector_lines::<mower_msgs::$($seg)::+>,
            default: default_line::<mower_msgs::$($seg)::+>,
        }),*]
    };
}

fn messages() -> Vec<Entry> {
    let table = mower_msgs::for_each_message!(entries);
    let names: Vec<String> = table.iter().map(|e| e.name.to_string()).collect();
    assert_eq!(names, mower_schema::message_names(), "a type of mower_schema is not generated, or not checked here");
    table
}

#[test]
fn every_message_type_is_r2r_s() {
    robot_rmw();
    let mut rng = Rng::new(0x0e9_22);
    for e in messages() {
        (e.values)(e.name, &mut rng, 60);
    }
}

fn check_service<S: ServiceMirror>(name: &str) {
    assert_eq!(S::NAME, name);
    for rust in [type_name::<S::Request>(), type_name::<S::Response>()] {
        assert_eq!(mower_bus::ros_type_name(rust).as_deref(), Some(name), "{rust}");
    }
    // r2r asks for these by value: a request and response through r2r's own service type
    let request = <S::Ros as r2r::WrappedServiceTypeSupport>::Request::default();
    assert!(<S::Request as RosMirror>::from_ros(request) == S::Request::default(), "{name}");
}

fn check_action<A: ActionMirror>(name: &str) {
    assert_eq!(A::NAME, name);
    for rust in [type_name::<A::Goal>(), type_name::<A::Result>(), type_name::<A::Feedback>()] {
        assert_eq!(mower_bus::ros_type_name(rust).as_deref(), Some(name), "{rust}");
    }
    let goal = <A::Ros as r2r::WrappedActionTypeSupport>::Goal::default();
    assert!(<A::Goal as RosMirror>::from_ros(goal) == A::Goal::default(), "{name}");
}

macro_rules! services {
    ($($name:literal => $($seg:ident)::+;)*) => {{
        let mut names = Vec::new();
        $(check_service::<mower_msgs::$($seg)::+::Service>($name); names.push($name);)*
        names
    }};
}

macro_rules! actions {
    ($($name:literal => $($seg:ident)::+;)*) => {{
        let mut names = Vec::new();
        $(check_action::<mower_msgs::$($seg)::+::Action>($name); names.push($name);)*
        names
    }};
}

#[test]
fn every_service_and_action_is_r2r_s() {
    robot_rmw();
    let mut names = mower_msgs::for_each_service!(services);
    names.extend(mower_msgs::for_each_action!(actions));
    names.sort_unstable();
    let want: Vec<&str> = mower_schema::interfaces().iter().map(|i| i.name).collect();
    assert_eq!(names, want);
}

macro_rules! constants {
    ($($ty:literal $name:literal $($seg:ident)::+;)*) => {
        vec![$(($ty, $name, mower_msgs::$($seg)::+ as i128, r2r::$($seg)::+ as i128)),*]
    };
}

/// Each constant's value. r2r binds them as enums of bindgen's, so only by a cast; and
/// a constant r2r did not bind at all would not compile here.
fn constants() -> Vec<(&'static str, &'static str, i128)> {
    let all: Vec<(&str, &str, i128, i128)> = mower_msgs::for_each_constant!(constants);
    all.into_iter()
        .map(|(ty, name, plain, ros)| {
            assert_eq!(plain, ros, "{ty}: {name}");
            (ty, name, plain)
        })
        .collect()
}

#[test]
fn every_constant_is_r2r_s() {
    assert!(constants().len() >= 100);
}

fn frozen(file: &str, text: String) {
    let path = facts(file);
    if std::env::var("MOWER_GOLDEN").as_deref() == Ok("write") {
        std::fs::write(&path, &text).unwrap();
        return;
    }
    assert!(
        std::fs::read_to_string(&path).unwrap_or_default() == text,
        "{} is not what r2r and rmw say (tools/oracle/regen.sh facts rewrites it)",
        path.display()
    );
}

#[test]
fn the_frozen_files_are_what_r2r_and_rmw_say() {
    robot_rmw();
    let (mut vectors, mut defaults) = (String::new(), String::new());
    for e in messages() {
        if !mower_schema::names().contains(&e.name) {
            (e.vectors)(e.name, &mut vectors);
        }
        (e.default)(e.name, &mut defaults);
    }
    frozen("interface_vectors.txt", vectors);
    frozen("interface_defaults.txt", defaults);
    let lines: String = constants().iter().map(|(ty, name, v)| format!("{ty}\t{name}\t{v}\n")).collect();
    frozen("interface_constants.txt", lines);
}
