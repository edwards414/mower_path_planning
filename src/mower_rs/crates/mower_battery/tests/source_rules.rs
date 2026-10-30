//! Rules of mower_battery that only its source shows (E9-07, from
//! test_controller_configuration.py; see test_support/source_check.rs). The
//! topics, the frame, the 0..1 percentage and the technology are
//! tests/bus.rs's, the OCV table and the health values the unit test
//! `rclpy_parity` in src/lib.rs; what is left is how `run` makes its node and
//! the defaults no bus test waits out.

#[path = "../../../test_support/source_check.rs"]
mod source_check;

use source_check::Source;

fn lib() -> Source {
    Source::new("mower_battery/src/lib.rs", include_str!("../src/lib.rs"))
}

/// The node is named by the module context: /battery_state, with that
/// section of the parameter file.
#[test]
fn the_node_is_named_by_the_module_context() {
    lib().body("run(ctx").has("r2r::Node::create(ctx, &m.node_name, &m.namespace)");
}

/// battery_state_node.py's defaults for what no bus test waits out: a
/// reading goes stale after 5 s, 25 V and more means the charger is on, and
/// 20 % is low.
#[test]
fn the_staleness_charger_and_low_battery_defaults_are_the_python_node_s() {
    let run = lib().body("run(ctx");
    for (name, default) in [("stale_timeout_s", "5.0"), ("charger_present_min_v", "25.0"), ("low_battery_pct", "20.0")] {
        run.has(&format!("\"{name}\", {default}"));
    }
}

/// No parameter handler: the model's settings are read once at start.
#[test]
fn the_parameters_cannot_change_at_runtime() {
    lib().lacks("make_parameter_handler");
}
