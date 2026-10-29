//! Rules of mower_localize that only its source shows (E9-07, from
//! test_controller_configuration.py; see test_support/source_check.rs).

#[path = "../../../test_support/source_check.rs"]
mod source_check;

use source_check::Source;

/// The filter's `frequency` timer skips the ticks it missed, as the rcl wall
/// timer of `ros_filter_node.cpp` does, and never bursts them: after a
/// scheduling hiccup the filter resumes at the current time instead of
/// running a backlog of predicts. A source check: on the manual clock the
/// tests run the filter on, a burst of ticks at one instant changes nothing
/// anyone can see.
#[test]
fn the_filter_timer_skips_missed_ticks() {
    let run = Source::new("mower_localize/src/ekf.rs", include_str!("../src/ekf.rs")).body("run_node");
    run.has("m.clock.interval(");
    let behaviours = run.calls("set_missed_tick_behavior");
    assert!(!behaviours.is_empty() && behaviours.iter().all(|b| b.ends_with("::Skip")), "{behaviours:?}");
}
