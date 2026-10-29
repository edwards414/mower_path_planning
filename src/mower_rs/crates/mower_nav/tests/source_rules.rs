//! Rules of mower_nav that only its source shows (E9-07, from
//! test_controller_configuration.py; see test_support/source_check.rs).
//! The admission and health decisions are state.rs's, with their tests; the
//! endpoints, the 20 Hz beat and the safety defaults are tests/bus.rs's
//! (`the_health_gate_and_the_manual_hold_run_on_the_production_defaults`);
//! what is left is how `run` makes its node (the bus tests hand `run_node` a
//! node of their own), the absent parameter handler and the bounded waits.

#[path = "../../../test_support/source_check.rs"]
mod source_check;

use source_check::Source;

fn lib() -> Source {
    Source::new("mower_nav/src/lib.rs", include_str!("../src/lib.rs"))
}

/// The node is named by the module context -- the launch `name=` or
/// mower_rsd's module table -- so it is /nav_action_server, with that
/// section of the parameter file.
#[test]
fn the_node_is_named_by_the_module_context() {
    lib().body("run(ctx").has("r2r::Node::create(ctx, &m.node_name, &m.namespace)");
}

/// No parameter handler, so no parameter service: the safety defaults and
/// whatever the parameter file sets cannot change while nav runs.
#[test]
fn the_safety_parameters_cannot_change_at_runtime() {
    lib().lacks("make_parameter_handler");
}

/// Every wait on Nav2 and on the dispatch confirmation is bounded by a
/// parameter, and terminal evidence counts only for the generation it
/// belongs to; a task whose end is not proven latches uncertain.
#[test]
fn nav2_waits_are_bounded_and_terminal_evidence_is_generation_correlated() {
    let lib = lib();
    let run = lib.body("run_node");
    for name in ["nav2_action_server_timeout_s", "nav2_goal_response_timeout_s", "nav2_cancel_timeout_s", "dispatch_confirmation_timeout_s"] {
        run.has(&format!("\"{name}\","));
    }
    lib.has("mark_nav2_dispatch_terminal(terminal_generation)");
    lib.has("mark_nav2_task_uncertain(");
}
