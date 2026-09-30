//! Regression oracle: the Rust coverage core against recorded outputs of the
//! former Python reference implementation.
//!
//! Provenance: `tests/data/python_reference_oracle.json` was recorded from the
//! Python implementation in `src/mower_mission/mower_mission/coverage/`
//! (safe_map_filter, path_validator, connector_planner, cell_decomposition)
//! and `.../path_generators/` (zigzag, spiral) at commit dff480b. That Python
//! code was removed afterwards; this crate is now the only implementation. The
//! inputs are the conftest.py fixtures (open, U-shape, two islands, rectangle
//! obstacle, narrow passage), the inputs of the old test_backend_parity.py
//! (ids starting with `parity/`) and 16 seeded random obstacle maps
//! (`numpy.random.RandomState(seed)`, several resolutions and non-zero
//! origins). `tests/data/gen_python_reference_oracle.py` recorded it and
//! reproduces it byte for byte from a checkout of dff480b (usage in its
//! docstring); each map's `source` field names its construction.
//!
//! One deliberate change since: `world_to_grid` rounds down (Python
//! truncated), so validate cases with a point just left of or below the grid
//! may report it unsafe; `check_validate` (tests/common) spells out what may
//! differ. Everything else:
//! integers, bools and strings must match exactly; floats within 1e-9. The
//! Python and Rust implementations agreed on every recorded case when the file
//! was generated, so a mismatch means a behaviour change (or a harness bug),
//! not float noise: investigate rather than loosen the tolerance.
//!
//! Parameters avoid strip/res and spacing/res ratios of exactly .5: Python's
//! `round()` rounds half to even, Rust's `f64::round` half away from zero, so
//! those inputs were never equal between the two implementations.
//!
//! The rotated-zigzag cases (`zigzag/.../a30`, `.../a90`, `parity/zigzag/
//! rotated_open_grid`) pin CURRENT behaviour, which is known to be poor: inside
//! each rotated band the path zig-zags sideways across the band instead of
//! running one straight lane. A deliberate fix of the rotated generator must
//! regenerate or replace those cases rather than tolerate the diff.

mod common;

use common::*;

/// Each section must hold at least this many cases, so an emptied or
/// truncated data file cannot pass silently.
fn assert_section(section: &str, min_cases: usize, check: fn(&serde_json::Value) -> Vec<String>) {
    let (checked, errs) = check_section(section, check);
    assert!(checked >= min_cases, "{section}: only {checked} cases in the oracle (expected >= {min_cases})");
    assert_no_mismatches(section, checked, &errs);
}

#[test]
fn oracle_provenance_is_recorded() {
    let p = &oracle()["provenance"];
    assert!(p["python_reference"].as_str().unwrap_or("").contains("dff480b"));
    assert!(oracle()["maps"].as_object().map_or(0, |m| m.len()) >= 21);
}

#[test]
fn filter_safe_components_matches_python() {
    assert_section("filter", 60, check_filter);
}

#[test]
fn validate_path_matches_python() {
    assert_section("validate", 50, check_validate);
}

#[test]
fn connector_planner_matches_python() {
    assert_section("connector", 120, check_connector);
}

#[test]
fn zigzag_matches_python() {
    assert_section("zigzag", 90, check_zigzag);
}

#[test]
fn zigzag_oracle_covers_axis_rotated_and_u_turn_cases() {
    let zz = cases("zigzag");
    for angle in [0.0, 30.0, 90.0] {
        let n = zz.iter().filter(|c| num(&c["angle_deg"]) == angle).count();
        assert!(n >= 20, "only {n} zigzag cases at angle {angle}");
    }
    // Wide strips (lane gap / 2 >= 0.35 m) are the U-turn cases.
    let wide = zz.iter().filter(|c| num(&c["strip_width_m"]) >= 0.7 && num(&c["angle_deg"]) == 0.0).count();
    assert!(wide >= 20, "only {wide} axis-aligned wide-strip (U-turn) zigzag cases");
}

#[test]
fn spiral_matches_python() {
    assert_section("spiral", 40, check_spiral);
}

#[test]
fn cell_decomposition_matches_python() {
    assert_section("decompose", 20, check_decompose);
}
