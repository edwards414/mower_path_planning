//! Rules of velocity_command_guard that only its source shows (E9-07, from
//! test_controller_configuration.py; see test_support/source_check.rs).
//! Every decision is core.rs's, with a test vector each, and the bus IO (E2
//! steps 6-7) runs in tests/bus.rs; what is left is how the r2r IO reads
//! its limits and when it publishes a stop, which takes a running rcl graph
//! to see, and which IO an instance takes.

#[path = "../../../test_support/source_check.rs"]
mod source_check;

use source_check::Source;

fn lib() -> Source {
    Source::new("velocity_command_guard/src/lib.rs", include_str!("../src/lib.rs"))
}

fn bus() -> Source {
    Source::new("velocity_command_guard/src/bus.rs", include_str!("../src/bus.rs"))
}

/// The limits default to the Python guard's. The final guard runs on these:
/// twist_mux.launch.py and mower_rsd.yaml set only the manual guard's
/// session and input age. Both IOs read them with `read_limits`.
#[test]
fn the_limits_default_to_the_python_guard_s() {
    let run = lib().body("read_limits(");
    for (name, default) in [
        ("command_timeout_s", "0.20"),
        ("max_input_age_s", "0.25"),
        ("max_future_skew_s", "0.05"),
        ("max_linear_x_m_s", "0.50"),
        ("max_angular_z_rad_s", "1.00"),
    ] {
        run.has(&format!("\"{name}\", {default}"));
    }
}

/// No parameter handler, so no parameter service: the limits are read once
/// at start and nothing can change them while the guard runs.
#[test]
fn the_limits_cannot_change_at_runtime() {
    lib().lacks("make_parameter_handler");
    bus().lacks("make_parameter_handler");
}

/// A stop goes out before anything can move, and on every rejection, receipt
/// timeout and shutdown (the four zeros of the Python node).
#[test]
fn a_stop_goes_out_at_start_and_on_every_rejection_timeout_and_shutdown() {
    let run = lib().body("run_r2r(");
    let stop = "publisher.publish(&twist(0.0, 0.0))";
    run.before(stop, "commands.next()");
    assert!(run.count(stop) >= 4, "{} stops", run.count(stop));
}

/// Each instance takes the bus IO when its own flag says so (on_bus.rs, by
/// its node name after mower_rsd's remap rules: the manual and the final
/// guard are two instances), r2r otherwise. The flags are process-wide and
/// read once, so a test cannot set them; tests/bus.rs runs the bus IO
/// itself.
#[test]
fn each_instance_s_flag_picks_its_io() {
    lib().body("run(ctx").in_order(&[
        "if mower_bridge::instance_on_bus(&m.remaps.node_name(&m.node_name))",
        "return bus::run_node(node, m).await;",
        "run_r2r(node, m).await",
    ]);
}
