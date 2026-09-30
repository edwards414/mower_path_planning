//! Port of mower_mission/test/test_safe_map_filter.py (commit dff480b).
//! Test names are the Python ones without the `test_` prefix.

mod common;

use common::*;
use mower_coverage_core::safe_map_filter::filter_safe_components_rs;

#[test]
fn filter_safe_components_removes_tiny_island() {
    let mut safe = zeros(6, 6);
    set(&mut safe, 0..3, 0..3, true);
    safe[[5, 5]] = true;

    // Python defaults: keep_largest_only=True.
    let (filtered, sizes, kept_sizes) = filter_safe_components_rs(safe.view(), 0.1, 0.05, true);

    assert_eq!(sizes, vec![9, 1]);
    assert_eq!(kept_sizes, vec![9]);
    assert!(filtered.slice(ndarray::s![0..3, 0..3]).iter().all(|&v| v));
    assert!(!filtered[[5, 5]]);
}

#[test]
fn filter_safe_components_keeps_largest_only() {
    let mut safe = zeros(8, 8);
    set(&mut safe, 0..3, 0..3, true);
    set(&mut safe, 5..7, 5..7, true);

    let (filtered, sizes, kept_sizes) = filter_safe_components_rs(safe.view(), 0.1, 0.01, true);

    assert_eq!(sizes, vec![9, 4]);
    assert_eq!(kept_sizes, vec![9]);
    assert_eq!(count_true(&filtered), 9);
}
