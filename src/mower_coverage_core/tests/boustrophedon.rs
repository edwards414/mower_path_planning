//! Tests of the boustrophedon planner (`plan_boustrophedon_rs`), the
//! replacement of the legacy zigzag generator in the coverage node.

mod common;

use common::*;
use mower_coverage_core::boustrophedon::*;
use mower_coverage_core::connector_planner::plan_connector_rs;
use mower_coverage_core::nav_split::{nav_short_segment, nav_segments};
use mower_coverage_core::path_validator::validate_path_rs;
use mower_coverage_core::safe_map_filter::filter_safe_components_rs;
use mower_coverage_core::types::SafeMap;
use mower_coverage_core::zigzag::generate_coverage_zigzag_path_rs;
use ndarray::Array2;

const STRIP: f64 = 0.8;
const SPACING: f64 = 0.2;

/// Every oracle map as the node sees it: largest safe component only.
fn node_maps() -> Vec<(String, Array2<bool>, f64, f64, f64)> {
    let names: Vec<String> = oracle()["maps"].as_object().unwrap().keys().cloned().collect();
    names
        .into_iter()
        .filter_map(|name| {
            let m = oracle_map(&name);
            let (safe, _, kept) = filter_safe_components_rs(m.grid.view(), m.res, 0.05, true);
            (!kept.is_empty()).then(|| (name, safe, m.res, m.ox, m.oy))
        })
        .collect()
}

fn strictly_inside(p: (f64, f64), g: &Array2<bool>, res: f64, ox: f64, oy: f64) -> bool {
    let c = ((p.0 - ox) / res).floor();
    let r = ((p.1 - oy) / res).floor();
    c >= 0.0 && r >= 0.0 && (c as usize) < g.ncols() && (r as usize) < g.nrows() && g[[r as usize, c as usize]]
}

/// A rectangle of `len_m x wid_m` whose long side points at `deg`, centred in
/// an `n x n` grid, eroded by nothing (every cell inside is safe).
fn rotated_rect(n: usize, res: f64, len_m: f64, wid_m: f64, deg: f64) -> Array2<bool> {
    let t = deg.to_radians();
    let c = n as f64 * res / 2.0;
    Array2::from_shape_fn((n, n), |(r, col)| {
        let (x, y) = ((col as f64 + 0.5) * res - c, (r as f64 + 0.5) * res - c);
        let (u, v) = (x * t.cos() + y * t.sin(), -x * t.sin() + y * t.cos());
        u.abs() < len_m / 2.0 && v.abs() < wid_m / 2.0
    })
}

#[test]
fn every_plan_is_safe_and_inside_the_grid() {
    for (name, g, res, ox, oy) in node_maps() {
        let sm = SafeMap { grid: g.view(), resolution: res, origin_x: ox, origin_y: oy };
        for ang in [Some(0.0), Some(30.0), Some(45.0), Some(90.0), Some(137.0), None] {
            let p = plan_boustrophedon_rs(g.view(), STRIP, SPACING, res, ox, oy, ang);
            assert!(!p.points.is_empty(), "{name} {ang:?}: empty plan");
            assert!(p.invalid_segments.is_empty(), "{name} {ang:?}: invalid {:?}", p.invalid_segments);
            let v = validate_path_rs(&p.points, &sm);
            assert!(v.valid, "{name} {ang:?}: {}", v.message);
            for &q in &p.points {
                assert!(strictly_inside(q, &g, res, ox, oy), "{name} {ang:?}: {q:?} outside the safe grid");
            }
            for sp in &p.split_points {
                assert!(p.points.iter().any(|q| (q.0 - sp.0).abs() < TOL && (q.1 - sp.1).abs() < TOL),
                    "{name} {ang:?}: split point {sp:?} is not a path point");
            }
            assert!((p.length_m - path_length_rs(&p.points)).abs() < 1e-9);
            assert_eq!(nav_short_segment(&p.points, &p.split_points), None,
                "{name} {ang:?}: the navigation server would refuse a segment");
        }
    }
}

#[test]
fn plans_are_deterministic() {
    for (name, g, res, ox, oy) in node_maps() {
        let a = plan_boustrophedon_rs(g.view(), STRIP, SPACING, res, ox, oy, None);
        let b = plan_boustrophedon_rs(g.view(), STRIP, SPACING, res, ox, oy, None);
        assert_eq!(a.points, b.points, "{name}");
        assert_eq!(a.angle_deg, b.angle_deg, "{name}");
    }
}

#[test]
fn coverage_is_high_at_the_axis_angle_and_at_least_the_legacy_one() {
    for (name, g, res, ox, oy) in node_maps() {
        let sm = SafeMap { grid: g.view(), resolution: res, origin_x: ox, origin_y: oy };
        let (h, w) = g.dim();
        let p = plan_boustrophedon_rs(g.view(), STRIP, SPACING, res, ox, oy, Some(0.0));
        let (lp, _, _) = generate_coverage_zigzag_path_rs(g.view(), STRIP, SPACING, res, h, w, ox, oy, 0.0);
        let cov = coverage_ratio_rs(&p.points, &sm, STRIP);
        let legacy = coverage_ratio_rs(&lp, &sm, STRIP);
        assert!(cov >= 0.9, "{name}: coverage {cov:.3}");
        // a noise map can lose a few cells to the split coalescing at a path end
        assert!(cov + 0.01 >= legacy, "{name}: coverage {cov:.3} < legacy {legacy:.3}");
    }
}

/// The legacy rotated branch emitted every safe cell of a band sorted by
/// (y, x), so the path zig-zagged sideways; here lanes are straight.
#[test]
fn rotated_lanes_are_straight() {
    let g = ones(60, 60);
    let sm = safe_map(&g);
    for ang in [30.0, 45.0, 75.0, 120.0] {
        let p = plan_boustrophedon_rs(g.view(), STRIP, SPACING, RES, OX, OY, Some(ang));
        let area = 6.0 * 6.0;
        // ideal lane length is area / strip; allow the turns and edge lanes
        assert!(p.length_m < 1.4 * area / STRIP, "{ang}: length {:.1}", p.length_m);
        assert!(coverage_ratio_rs(&p.points, &sm, STRIP) > 0.97, "{ang}");
        // lanes: consecutive steps are either along the lane direction or a turn
        let (tx, ty) = (-(ang as f64).to_radians().sin(), (ang as f64).to_radians().cos());
        let along = p.points.windows(2).filter(|s| {
            let (dx, dy) = (s[1].0 - s[0].0, s[1].1 - s[0].1);
            let l = (dx * dx + dy * dy).sqrt();
            l > 0.0 && ((dx * tx + dy * ty) / l).abs() > 1.0 - 1e-6
        }).count();
        assert!(along * 2 > p.points.len(), "{ang}: only {along} of {} steps run along the lanes", p.points.len());
        let (lp, _, _) = generate_coverage_zigzag_path_rs(g.view(), STRIP, SPACING, RES, 60, 60, OX, OY, ang);
        assert!(p.length_m * 3.0 < path_length_rs(&lp), "{ang}: legacy {:.1} vs {:.1}", path_length_rs(&lp), p.length_m);
    }
}

/// Lanes along the long side need the fewest turns: the searched angle puts
/// the lane direction within a few degrees of the rectangle's long axis.
#[test]
fn auto_angle_follows_the_long_axis() {
    for long_axis in [0.0, 25.0, 60.0, 100.0, 150.0] {
        let g = rotated_rect(120, 0.05, 5.0, 1.8, long_axis);
        let p = plan_boustrophedon_rs(g.view(), STRIP, SPACING, 0.05, 0.0, 0.0, None);
        // lane direction is angle + 90
        let lane_dir = (p.angle_deg + 90.0).rem_euclid(180.0);
        let diff = (lane_dir - long_axis).rem_euclid(180.0);
        let diff = diff.min(180.0 - diff);
        assert!(diff <= 6.0, "long axis {long_axis}: chose angle {} (lanes at {lane_dir})", p.angle_deg);
        let fixed = plan_boustrophedon_rs(g.view(), STRIP, SPACING, 0.05, 0.0, 0.0, Some((long_axis + 90.0) % 180.0));
        let across = plan_boustrophedon_rs(g.view(), STRIP, SPACING, 0.05, 0.0, 0.0, Some(long_axis % 180.0));
        assert!(fixed.turns < across.turns, "{long_axis}: along {} turns, across {}", fixed.turns, across.turns);
    }
}

/// An obstacle in the middle splits the sweep into cells; every cell is
/// mowed and the path stays valid.
#[test]
fn obstacles_split_the_area_into_cells() {
    let mut g = ones(40, 40);
    set(&mut g, 15..25, 15..25, false);
    let sm = safe_map(&g);
    let p = plan_boustrophedon_rs(g.view(), STRIP, SPACING, RES, OX, OY, Some(0.0));
    assert!(p.cells >= 3, "cells {}", p.cells);
    assert!(validate_path_rs(&p.points, &sm).valid);
    assert!(coverage_ratio_rs(&p.points, &sm, STRIP) > 0.97);
}

/// The band along the safe region's edge is mowed even when the region does
/// not start on the legacy grid stride (the legacy lanes sat on columns
/// strip/2 + k*strip of the grid).
#[test]
fn edge_band_is_not_skipped() {
    let mut g = zeros(40, 40);
    set(&mut g, 5..35, 3..23, true); // 2.0 m wide, starting at column 3
    let sm = safe_map(&g);
    let p = plan_boustrophedon_rs(g.view(), STRIP, SPACING, RES, OX, OY, Some(0.0));
    let (lp, _, _) = generate_coverage_zigzag_path_rs(g.view(), STRIP, SPACING, RES, 40, 40, OX, OY, 0.0);
    let cov = coverage_ratio_rs(&p.points, &sm, STRIP);
    assert!(cov > 0.99, "{cov}");
    assert!(cov > coverage_ratio_rs(&lp, &sm, STRIP));
}

#[test]
fn empty_and_single_cell_maps() {
    let g = zeros(10, 10);
    let p = plan_boustrophedon_rs(g.view(), STRIP, SPACING, RES, OX, OY, None);
    assert!(p.points.is_empty() && p.split_points.is_empty());
    let mut g = zeros(10, 10);
    g[[4, 4]] = true;
    let p = plan_boustrophedon_rs(g.view(), STRIP, SPACING, RES, OX, OY, Some(0.0));
    assert!(!p.points.is_empty());
    assert!(validate_path_rs(&p.points, &safe_map(&g)).valid);
}

#[test]
fn simplify_pulls_a_staircase_straight_but_not_through_an_obstacle() {
    let mut g = ones(20, 20);
    let stair: Vec<(f64, f64)> = (0..12).map(|i| (0.05 + 0.1 * i as f64, 0.05 + 0.1 * (i / 2) as f64)).collect();
    assert_eq!(simplify_path_rs(&stair, &safe_map(&g)).len(), 2);
    // a wall across the straight line keeps the detour
    let detour = vec![(0.15, 0.15), (0.15, 1.85), (1.85, 1.85), (1.85, 0.15)];
    set(&mut g, 0..15, 8..12, false);
    let s = simplify_path_rs(&detour, &safe_map(&g));
    assert!(validate_path_rs(&s, &safe_map(&g)).valid);
    assert!(s.len() >= 3);
}

/// Gardens like the robot's: a rectangle or an ellipse with a few round trees,
/// already eroded by the robot radius. The navigation server must accept
/// every plan (no FollowPath segment of 0.15 m or less) at every angle; the
/// review found 30 deg / 135 deg / auto refused most of the time before
/// split points were coalesced.
#[test]
fn navigation_accepts_every_garden_plan() {
    let mut refused = Vec::new();
    for seed in 0..24u64 {
        let res = if seed % 2 == 0 { 0.05 } else { 0.1 };
        let n = (8.0 / res) as usize;
        let ellipse = seed % 3 == 0;
        let trees: Vec<(f64, f64, f64)> = (0..(seed % 5))
            .map(|k| {
                let f = |m: u64| ((seed * 7919 + k * 104729 + m) % 1000) as f64 / 1000.0;
                (1.5 + 5.0 * f(1), 1.5 + 5.0 * f(2), 0.3 + 0.5 * f(3))
            })
            .collect();
        let g = Array2::from_shape_fn((n, n), |(r, c)| {
            let (x, y) = ((c as f64 + 0.5) * res, (r as f64 + 0.5) * res);
            let inside = if ellipse {
                ((x - 4.0) / 3.2).powi(2) + ((y - 4.0) / 2.4).powi(2) < 1.0
            } else {
                (0.8..7.2).contains(&x) && (1.2..6.6).contains(&y)
            };
            inside && trees.iter().all(|&(tx, ty, tr)| (x - tx).powi(2) + (y - ty).powi(2) > (tr + 0.75).powi(2))
        });
        let (g, _, kept) = filter_safe_components_rs(g.view(), res, 0.05, true);
        if kept.is_empty() {
            continue;
        }
        let sm = SafeMap { grid: g.view(), resolution: res, origin_x: 0.0, origin_y: 0.0 };
        for ang in [Some(0.0), Some(30.0), Some(90.0), Some(135.0), None] {
            let p = plan_boustrophedon_rs(g.view(), STRIP, SPACING, res, 0.0, 0.0, ang);
            assert!(validate_path_rs(&p.points, &sm).valid, "seed {seed} {ang:?}");
            if let Some(bad) = nav_short_segment(&p.points, &p.split_points) {
                refused.push((seed, ang, bad));
            }
            assert!(nav_segments(&p.points, &p.split_points).len() >= 1);
        }
    }
    assert!(refused.is_empty(), "refused plans: {refused:?}");
}

/// The review's reproductions: a corner stub as the first lane (45 deg,
/// 30 deg) and a thin region (0 deg) produced segments of 0.10-0.15 m.
#[test]
fn review_reproductions_are_accepted_by_navigation() {
    for (h, w, res, ang) in [(150, 200, 0.05, 45.0), (150, 200, 0.05, 30.0), (10, 61, 0.05, 0.0)] {
        let g = ones(h, w);
        let p = plan_boustrophedon_rs(g.view(), STRIP, SPACING, res, 0.0, 0.0, Some(ang));
        assert_eq!(nav_short_segment(&p.points, &p.split_points), None, "{h}x{w} {ang}");
    }
}

/// A* forbids passing diagonally between two unsafe cells; pulling its path
/// straight must not undo that (the straight line through the zero-width gap
/// at (1.0, 1.0) passes the shared Bresenham check).
#[test]
fn simplify_keeps_the_no_corner_cut_rule() {
    let mut g = ones(20, 20);
    set(&mut g, 1..10, 10..19, false);
    set(&mut g, 10..20, 0..10, false);
    let sm = safe_map(&g);
    let a = plan_connector_rs((0.55, 0.55), (1.45, 1.45), &sm, 0.2).expect("A* path");
    let s = simplify_path_rs(&a, &sm);
    assert!(s.len() > 2, "pulled straight through the corner gap: {s:?}");
    assert!(validate_path_rs(&s, &sm).valid);
}
