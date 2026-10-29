//! A rule of mower_ws_bridge that only its source shows (E9-07, from
//! test_controller_configuration.py; see test_support/source_check.rs). The
//! `--backend` switch itself runs in src/lib.rs's tests (`backend_of` and
//! `parse_args`) and in mower_rsd's src/main.rs tests (the bus bridge's plan).

#[path = "../../../test_support/source_check.rs"]
mod source_check;

use source_check::Source;

/// The binary on its own (rust_daemon:=false) reads `--backend` with the
/// parser mower_rsd uses, and runs the bus backend as a module moved to the
/// bus runs on its own: mower_bridge's `module_main`, a bus of its own next
/// to a bridge that faces DDS. Which runner `main` picks only shows in a
/// running rcl graph, so it is checked in the source.
#[test]
fn the_binary_runs_its_bus_backend_through_mower_bridge_s_module_main() {
    Source::new("mower_ws_bridge/src/main.rs", include_str!("../src/main.rs")).in_order(&[
        "mower_ws_bridge::backend_of(&argv)",
        "Ok(mower_ws_bridge::Backend::Bus) => mower_bridge::module_main(\"bridge\", \"mower_ws_bridge\", 4, mower_ws_bridge::run)",
    ]);
}
