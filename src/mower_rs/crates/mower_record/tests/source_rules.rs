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
