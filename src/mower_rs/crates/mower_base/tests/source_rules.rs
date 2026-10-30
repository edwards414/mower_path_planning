//! Rules of mower_base that only its source shows (E9-07, from
//! test_controller_configuration.py; see test_support/source_check.rs). The
//! drivetrain itself is mower_base_core's, with its safety vectors; what is
//! left is what `run` reads and publishes, which takes the STM32 on a serial
//! port to see.

#[path = "../../../test_support/source_check.rs"]
mod source_check;

use source_check::Source;

/// The whole of lib.rs, as the Python checks read it: its first `fn run` is
/// a method of the serial reader, not the module's `pub async fn run`.
fn run() -> Source {
    Source::new("mower_base/src/lib.rs", include_str!("../src/lib.rs"))
}

/// The guarded command topic it drives from, and the two side channels the
/// app and the auto-tune use, by default.
#[test]
fn the_command_input_and_the_side_channels_are_the_production_topics() {
    let run = run();
    for topic in ["/drivetrain_guarded_cmd_vel", "/mower_base/telemetry", "/mower_base/firmware_info", "/mower_base/wheel_override"] {
        run.says(&format!("\"{topic}\""));
    }
}

/// /odom_slow is the copy mission.launch.py's odom_throttle made
/// (`messages /odom 5.0 /odom_slow`), by default.
#[test]
fn the_slow_odometry_copy_defaults_to_the_throttle_s_topic_and_rate() {
    let run = run();
    run.has("params::string(&node, \"odom_slow_topic\", \"/odom_slow\")");
    run.has("params::f64(&node, \"odom_slow_rate_hz\", 5.0)");
}

/// The recorders' rear light as mower_hardware read it: a source that sends
/// no refresh for 6 s drops, by default, and the reader keeps 8 latched
/// values (R TL 8), so the recorders' last ones reach a restarted driver.
/// tests/rear_light.rs runs the light on the default topic with a 1.5 s
/// timeout, to stay quick; the 6 s default and the reader's depth (what a
/// restarted driver gets back) are checked in the source.
#[test]
fn the_rear_light_keeps_mower_hardware_s_timeout_and_reader_qos() {
    let run = run();
    run.has("params::f64(&node, \"rear_light_timeout_s\", 6.0)");
    run.has("node.subscribe::<StringMsg>(&rear_light_topic, QosProfile::default().keep_last(8).transient_local().reliable()");
}

/// The module takes the bus IO when its flag says so (on_bus.rs, by its node
/// name after mower_rsd's remap rules), r2r otherwise; `run` goes through
/// `run_node`. The flags are process-wide and read once, so a test cannot
/// set them; tests/bus.rs runs the bus IO itself, under the production
/// names.
#[test]
fn its_flag_picks_its_io() {
    run().has("pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult { let node = r2r::Node::create(ctx, &m.node_name, &m.namespace)?; run_node(node, m).await }");
    run().in_order(&[
        "let on_bus = mower_bridge::instance_on_bus(&m.remaps.node_name(&m.node_name));",
        "run_io(node, m, on_bus).await",
    ]);
}
