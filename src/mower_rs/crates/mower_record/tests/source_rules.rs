//! A rule of mower_record that only its source shows (E9-07: what
//! test_qos_configuration.py checks only on path_record_node.py; see
//! test_support/source_check.rs). The refusal itself is the unit test
//! `persistence_rules::a_site_is_loaded_only_near_the_current_datum` in
//! src/recorder.rs.

#[path = "../../../test_support/source_check.rs"]
mod source_check;

use source_check::Source;

/// A saved site is loaded only within 100 m of the current datum, by
/// default. A source check: `run` reads it from the node it creates, which a
/// test cannot hand parameters to.
#[test]
fn a_site_is_loaded_only_within_100_m_by_default() {
    Source::new("mower_record/src/lib.rs", include_str!("../src/lib.rs")).body("run(ctx").has("\"max_site_datum_distance_m\", 100.0");
}

/// A recording with an RTK pause over 10 s is not saved, by default (owner,
/// 2026-11-07). The bus test `a_recording_with_a_long_rtk_outage_is_refused`
/// shows the refusal on the code's defaults; this pins the number.
#[test]
fn a_recording_is_saved_only_without_an_rtk_pause_over_10_s_by_default() {
    Source::new("mower_record/src/lib.rs", include_str!("../src/lib.rs")).body("run(ctx").has("\"max_rtk_pause_s\", 10.0");
}
