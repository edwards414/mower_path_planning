//! Rules of mower_pid_autotune that only its source shows (E9-07, from
//! test_controller_configuration.py; see test_support/source_check.rs). The
//! fit and the controller design are tuning.rs's, with their tests and the
//! Python golden vectors; the flag bits and the fit limits are the unit test
//! `rclpy_parity` in src/lib.rs. What is left is how `run` makes its node,
//! service and parameters and names its states, which takes a running rcl
//! graph and a base to see.

#[path = "../../../test_support/source_check.rs"]
mod source_check;

use source_check::Source;

fn lib() -> Source {
    Source::new("mower_pid_autotune/src/lib.rs", include_str!("../src/lib.rs"))
}

/// The value of a parameter default as the Rust code writes it.
fn rust_literal(value: &serde_json::Value) -> String {
    match value {
        serde_json::Value::String(s) => format!("{s:?}"),
        serde_json::Value::Array(items) => format!("&[{}]", items.iter().map(rust_literal).collect::<Vec<_>>().join(", ")),
        other => other.to_string(),
    }
}

/// The node is named by the module context, and serves /pid_autotune.
#[test]
fn the_node_is_named_by_the_module_context_and_serves_pid_autotune() {
    lib().body("run(ctx").has("r2r::Node::create(ctx, &m.node_name, &m.namespace)");
    // on r2r; tests/bus.rs serves it on the bus under the same name
    lib().body("run_r2r(").has("create_service::<PidAutotune::Service>(\"/pid_autotune\"");
}

/// Every parameter pid_autotune_node.py declares, with its default
/// (tests/rclpy_parity.json, which the Python test checks against the Python
/// node).
#[test]
fn every_parameter_has_the_python_node_s_default() {
    let parity: serde_json::Value = serde_json::from_str(include_str!("rclpy_parity.json")).expect("json");
    let parameters = parity["parameters"].as_array().expect("parameters");
    assert!(!parameters.is_empty());
    // read_config: what both IOs (r2r, the bus) start from
    let run = lib().body("read_config(");
    for p in parameters {
        let name = p[0].as_str().expect("a name");
        run.has(&format!("\"{name}\", {}", rust_literal(&p[1])));
    }
}

/// The status states the app shows, and the flash-save diagnostic.
#[test]
fn the_states_and_the_flash_diagnostic_are_the_python_node_s() {
    let lib = lib();
    for state in ["precheck", "open_loop", "fitting", "verify", "review", "saving", "done", "failed", "aborted"] {
        lib.says(&format!("\"{state}\""));
    }
    lib.says("flash save attempt {attempt}, flash_diag=0x{diag:04x}");
}

/// No parameter handler: the step sizes and the verification limits are
/// read once at start.
#[test]
fn the_parameters_cannot_change_at_runtime() {
    lib().lacks("make_parameter_handler");
}

/// The module takes the bus IO when its flag says so (on_bus.rs, by its node
/// name after mower_rsd's remap rules), r2r otherwise. The flags are
/// process-wide and read once, so a test cannot set them; tests/bus.rs runs
/// the bus IO itself.
#[test]
fn its_flag_picks_its_io() {
    lib().body("run(ctx").in_order(&[
        "if mower_bridge::instance_on_bus(&m.remaps.node_name(&m.node_name))",
        "return run_bus(node, m).await;",
        "run_r2r(node, m).await",
    ]);
}
