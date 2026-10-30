//! A rule of mower_rsd's module table that only its source shows (E9-07,
//! from test_controller_configuration.py; see test_support/source_check.rs).
//! The ids, switches and node names are run in tests/module_table.rs
//! (`mower_rsd --list-modules`) and in src/main.rs's tests.

#[path = "../../../test_support/source_check.rs"]
mod source_check;

use source_check::Source;

/// The mux module's one instance, twist_mux, runs mower_mux's module (E4)
/// and no other crate's. Which code an instance runs only shows on a
/// running rcl graph, so it is checked in the source.
#[test]
fn the_mux_module_runs_mower_mux() {
    Source::new("mower_rsd/src/main.rs", include_str!("../src/main.rs")).in_order(&[
        "id: \"mux\"",
        "switch: \"rust_mux\"",
        "instances: &[Instance { node: \"twist_mux\", run: |c, m| Box::pin(mower_mux::run(c, m)) }]",
    ]);
}

/// The bag module's three instances run mower_recorder's three roles (E5),
/// each its own: which code an instance runs only shows on a running rcl
/// graph, so it is checked in the source.
#[test]
fn the_bag_module_runs_mower_recorder_s_three_roles() {
    Source::new("mower_rsd/src/main.rs", include_str!("../src/main.rs")).in_order(&[
        "id: \"bag\"",
        "switch: \"rust_recorder\"",
        "Instance { node: \"graph_snapshot\", run: |c, m| Box::pin(mower_recorder::graph::run(c, m)) }",
        "Instance { node: \"recorder_manager\", run: |c, m| Box::pin(mower_recorder::manager::run(c, m)) }",
        "Instance { node: \"bag_store\", run: |c, m| Box::pin(mower_recorder::store::run(c, m)) }",
    ]);
}
