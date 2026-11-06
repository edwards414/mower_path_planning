//! What rcl makes of production's parameter files, node by node (E9-06, the
//! oracle of E9-23).
//!
//! Most module nodes serve no parameter services, so `ros2 param dump` cannot
//! ask them; what they run with comes from `node.params`, which r2r fills from
//! `rcl_arguments_get_param_overrides` when the node is made. So: one r2r
//! context with the production `--params-file` list (in launch's order,
//! `deploy/test/golden/ros_facts/launch/args.json`: the repo's files and the
//! node-scoped ones launch writes, frozen beside it), every node of the manifest
//! made under its production name, and each one's `node.params` must be
//! `param_overrides.json`. A parameter file changed without that file fails
//! here until `MOWER_GOLDEN=write` writes it again (tools/oracle/regen.sh facts).
//!
//! The production files have no `/**` section, and use few of the forms the
//! parser takes. `param_rules/` holds two files that do, and
//! `param_rules.json` what the same path makes of them: which section wins when
//! a node has several, and how each form is typed.
//!
//! rcl reads argv when the context is made, and libtest refuses arguments it
//! does not know, so this test has a `main` of its own, like `resolver.rs`: it
//! runs itself again with the files in argv, on ROS domain 109, once per set of
//! files.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};

use mower_bridge::{MODULES, NODE};
use r2r::ParameterValue;
use serde_json::{json, Value};

const NAME: &str = "param_overrides";
const CHILD: &str = "MOWER_BRIDGE_PARAM_OVERRIDES";

fn repo() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../..")
}

fn facts() -> PathBuf {
    repo().join("deploy/test/golden/ros_facts")
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if std::env::var_os(CHILD).is_some() {
        return oracle();
    }
    if args.iter().any(|a| a == "--list") {
        println!("{NAME}: test");
        return ExitCode::SUCCESS;
    }
    let filters: Vec<&String> = args.iter().filter(|a| !a.starts_with('-')).collect();
    if !filters.is_empty() && !filters.iter().any(|f| NAME.contains(f.as_str())) {
        println!("running 0 tests");
        return ExitCode::SUCCESS;
    }
    let launch: Value = serde_json::from_str(&std::fs::read_to_string(facts().join("launch/args.json")).expect("launch/args.json"))
        .expect("launch/args.json");
    let production: Vec<PathBuf> = launch["params_files"]
        .as_array()
        .expect("params_files")
        .iter()
        .map(|file| {
            let file = file.as_str().expect("a path");
            if file.starts_with("src/") { repo().join(file) } else { facts().join(file) }
        })
        .collect();
    let rules = vec![facts().join("param_rules/a.yaml"), facts().join("param_rules/b.yaml")];
    println!("running 1 test");
    for (mode, files) in [("production", production), ("rules", rules)] {
        let mut command = Command::new(std::env::current_exe().expect("the test binary"));
        command.arg("--ros-args");
        for path in files {
            assert!(path.exists(), "{}", path.display());
            command.arg("--params-file").arg(path);
        }
        command.env(CHILD, mode).env("ROS_DOMAIN_ID", "109");
        let status = command.status().expect("the oracle runs");
        if !status.success() {
            println!("test {NAME} ... FAILED ({mode}: {status})");
            return ExitCode::FAILURE;
        }
    }
    println!("test {NAME} ... ok");
    ExitCode::SUCCESS
}

/// A double as JSON; JSON has no NaN or infinities, so those are their names.
fn double(v: f64) -> Value {
    if v.is_finite() {
        json!(v)
    } else if v.is_nan() {
        json!("nan")
    } else {
        json!(if v > 0.0 { "inf" } else { "-inf" })
    }
}

/// `{"double": 0.75}`: the value with the type r2r read it as, since 1 and 1.0
/// are different parameters to a module's getter.
fn entry(value: &ParameterValue) -> Value {
    match value {
        ParameterValue::NotSet => json!({"not_set": null}),
        ParameterValue::Bool(v) => json!({"bool": v}),
        ParameterValue::Integer(v) => json!({"integer": v}),
        ParameterValue::Double(v) => json!({"double": double(*v)}),
        ParameterValue::String(v) => json!({"string": v}),
        ParameterValue::BoolArray(v) => json!({"bool_array": v}),
        ParameterValue::ByteArray(v) => json!({"byte_array": v}),
        ParameterValue::IntegerArray(v) => json!({"integer_array": v}),
        ParameterValue::DoubleArray(v) => json!({"double_array": v.iter().map(|x| double(*x)).collect::<Vec<_>>()}),
        ParameterValue::StringArray(v) => json!({"string_array": v}),
    }
}

fn section(ctx: &r2r::Context, name: &str, namespace: &str) -> String {
    let node = r2r::Node::create(ctx.clone(), name, namespace).expect("a node");
    let params = node.params.lock().unwrap();
    let sorted: BTreeMap<&String, &r2r::Parameter> = params.iter().collect();
    let lines: Vec<String> = sorted.iter().map(|(k, p)| format!("  {}: {}", Value::from(k.as_str()), entry(&p.value))).collect();
    let body = if lines.is_empty() { "{}".to_string() } else { format!("{{\n{}\n }}", lines.join(",\n")) };
    let key = if namespace.is_empty() { name.to_string() } else { format!("{namespace}/{name}") };
    format!(" {}: {body}", Value::from(key))
}

fn oracle() -> ExitCode {
    let ctx = r2r::Context::create().expect("an rcl context (source /opt/ros/jazzy/setup.bash)");
    let mode = std::env::var(CHILD).unwrap();
    let (file, sections): (&str, Vec<String>) = if mode == "production" {
        let mut nodes: Vec<&str> = MODULES.iter().flat_map(|m| m.instances).map(|i| i.node).collect();
        nodes.push(NODE);
        nodes.sort();
        nodes.dedup();
        ("param_overrides.json", nodes.into_iter().map(|n| section(&ctx, n, "")).collect())
    } else {
        let nodes = [("rule_node", ""), ("other_node", ""), ("third_node", ""), ("unmentioned", ""), ("in_ns", ""), ("in_ns", "/ns"), ("deep_node", "/ns")];
        ("param_rules.json", nodes.into_iter().map(|(n, ns)| section(&ctx, n, ns)).collect())
    };
    let text = format!("{{\n{}\n}}\n", sections.join(",\n"));
    let path = facts().join(file);
    if std::env::var("MOWER_GOLDEN").as_deref() == Ok("write") {
        std::fs::write(&path, &text).expect("writes the file");
        return ExitCode::SUCCESS;
    }
    if std::fs::read_to_string(&path).unwrap_or_default() != text {
        eprintln!(
            "{} is not what rcl reads from its parameter files (a changed file? MOWER_GOLDEN=write rewrites it; tools/oracle/regen.sh facts)",
            path.display()
        );
        return ExitCode::FAILURE;
    }
    ExitCode::SUCCESS
}
