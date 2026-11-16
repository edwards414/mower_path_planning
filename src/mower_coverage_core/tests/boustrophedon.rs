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

/// The robot's safe map of a 22 x 10 m zone whose long side points at 30 deg
/// (`tests/data/robot_rotated_rect_safe.json`): OpenCV's fillPoly and erosion
/// leave its slanted edges an irregular staircase, unlike `rotated_rect`.
fn robot_rotated_rect() -> (Array2<bool>, f64, f64, f64) {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data/robot_rotated_rect_safe.json");
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    let dim = |k: &str| v[k].as_u64().unwrap() as usize;
    let mut g = Array2::from_elem((dim("height"), dim("width")), false);
    for run in v["runs"].as_array().unwrap() {
        let at = |i: usize| run[i].as_u64().unwrap() as usize;
        for c in at(1)..at(2) {
            g[[at(0), c]] = true;
        }
    }
    let num = |k: &str| v[k].as_f64().unwrap();
    (g, num("resolution"), num("origin_x"), num("origin_y"))
}

/// Lanes parallel to a slanted edge of that map (120 deg) rode the edge's
/// staircase: on the robot, 12 lanes made 109 cells, 118 turns and 231 jogs
/// a few centimetres long. Lanes half a degree off fared no better. Each
/// lane is one stretch now, and the searched angle follows the long side.
#[test]
fn lanes_along_a_slanted_edge_stay_whole() {
    let (g, res, ox, oy) = robot_rotated_rect();
    let sm = SafeMap { grid: g.view(), resolution: res, origin_x: ox, origin_y: oy };
    for ang in [119.5, 120.0, 120.5] {
        let p = plan_boustrophedon_rs(g.view(), STRIP, SPACING, res, ox, oy, Some(ang));
        assert_eq!(p.cells, 1, "{ang}: {} cells", p.cells);
        assert_eq!(p.turns + 1, p.lanes, "{ang}: {} turns for {} lanes", p.turns, p.lanes);
        assert!(validate_path_rs(&p.points, &sm).valid, "{ang}");
        let cov = coverage_ratio_rs(&p.points, &sm, STRIP);
        assert!(cov > 0.99, "{ang}: coverage {cov:.3}");
    }
    let auto = plan_boustrophedon_rs(g.view(), STRIP, SPACING, res, ox, oy, None);
    let lane_dir = (auto.angle_deg + 90.0).rem_euclid(180.0);
    assert!((lane_dir - 30.0).abs() <= 1.0, "chose {} (lanes at {lane_dir})", auto.angle_deg);
}

/// A lane crossing an edge at a degree or two grazes its staircase for
/// metres; every crumb it left between unsafe samples was a cell of its own
/// (16-100 cells and 21-105 turns for these 9 lanes; 1-5 cells now). Coverage stays where it was (0.95-0.97
/// before, mostly 0.99 now): at 2 deg the outermost lane still crosses the
/// edge and leaves a wedge no straight lane reaches; the crumbs mowed a
/// sliver of it for a dozen extra turns.
#[test]
fn lanes_crossing_an_edge_at_a_shallow_angle_leave_no_crumbs() {
    for long_axis in [17.0, 30.0, 63.0, 141.0] {
        let g = rotated_rect(300, 0.05, 12.0, 6.0, long_axis);
        let sm = SafeMap { grid: g.view(), resolution: 0.05, origin_x: 0.0, origin_y: 0.0 };
        for off in [-2.0, -0.5, 0.5, 2.0] {
            let ang = (long_axis + 90.0 + off).rem_euclid(180.0);
            let p = plan_boustrophedon_rs(g.view(), STRIP, SPACING, 0.05, 0.0, 0.0, Some(ang));
            assert!(p.cells <= 5 && p.turns <= p.lanes + 3, "axis {long_axis}, {ang}: {} cells, {} turns for {} lanes", p.cells, p.turns, p.lanes);
            assert!(validate_path_rs(&p.points, &sm).valid, "axis {long_axis}, {ang}");
            let cov = coverage_ratio_rs(&p.points, &sm, STRIP);
            assert!(cov > 0.93, "axis {long_axis}, {ang}: coverage {cov:.3}");
        }
    }
}

fn turn_at(p: &[(f64, f64)], i: usize) -> f64 {
    let h = |a: (f64, f64), b: (f64, f64)| (b.1 - a.1).atan2(b.0 - a.0);
    let d = h(p[i], p[i + 1]) - h(p[i - 1], p[i]);
    d.sin().atan2(d.cos()).abs()
}

/// Lanes of a narrow strip are joined by an arc too (radius = half the lane
/// gap), not by a square turn: before, any strip under 0.7 m made the mower
/// pivot at the lane end, hop sideways and pivot again. Where lanes end
/// level the path has no corner the navigation server would cut at; along a
/// slanted edge a turn is arcs or, when the lane ends are far out of level,
/// a hop with one pivot. Either way the mower stops once per turn: a
/// FollowPath segment per lane.
#[test]
fn turns_are_arcs_or_one_pivot() {
    for strip in [0.25, 0.3, 0.4, 0.5, 0.65, 0.8] {
        for (res, ang) in [(0.05, 0.0), (0.05, 90.0), (0.1, 0.0), (0.05, 15.0), (0.05, 30.0), (0.05, 45.0), (0.1, 120.0)] {
            let n = (6.0 / res) as usize;
            let g = ones(n, n);
            let sm = SafeMap { grid: g.view(), resolution: res, origin_x: 0.0, origin_y: 0.0 };
            let p = plan_boustrophedon_rs(g.view(), strip, SPACING, res, 0.0, 0.0, Some(ang));
            let what = format!("strip {strip} res {res} angle {ang}");
            assert!(validate_path_rs(&p.points, &sm).valid, "{what}");
            assert!(p.invalid_segments.is_empty(), "{what}");
            assert_eq!(nav_short_segment(&p.points, &p.split_points), None, "{what}");
            // arcs leave the tips of lanes that end in an acute corner unmowed
            assert!(coverage_ratio_rs(&p.points, &sm, strip) > 0.96, "{what}: coverage {}", coverage_ratio_rs(&p.points, &sm, strip));
            let segments = nav_segments(&p.points, &p.split_points);
            let sharp: Vec<usize> = (1..p.points.len() - 1).filter(|&i| turn_at(&p.points, i) >= 0.8).collect();
            assert!(segments.len() <= p.lanes, "{what}: {} segments for {} lanes", segments.len(), p.lanes);
            for i in &sharp {
                assert!(segments.iter().any(|s| s.0 == *i), "{what}: the corner at pose {i} is not where a segment starts");
            }
            if ang == 0.0 || ang == 90.0 {
                // a square: every lane end is level with the next one
                assert!(sharp.is_empty(), "{what}: sharp corners at {sharp:?}");
                assert_eq!(segments.len(), p.lanes, "{what}");
            }
        }
    }
    // lane ends out of level by up to about the lane gap: still arcs
    for (strip, ang) in [(0.4, 45.0), (0.5, 30.0), (0.8, 30.0), (0.8, 45.0)] {
        let g = ones(120, 120);
        let p = plan_boustrophedon_rs(g.view(), strip, SPACING, 0.05, 0.0, 0.0, Some(ang));
        let sharp = (1..p.points.len() - 1).filter(|&i| turn_at(&p.points, i) >= 0.8).count();
        assert_eq!(sharp, 0, "strip {strip} angle {ang}");
    }
}

/// The arc is a half circle: it leaves and joins the lanes tangentially, and
/// every point of it is one radius from the centre.
#[test]
fn the_arc_is_a_half_circle() {
    let g = ones(60, 60);
    let p = plan_boustrophedon_rs(g.view(), 0.3, SPACING, 0.05, 0.0, 0.0, Some(0.0));
    // lanes run along y; the first arc is the first run of steps that do not
    let off_lane: Vec<usize> = (1..p.points.len()).filter(|&i| (p.points[i].0 - p.points[i - 1].0).abs() > 1e-9).collect();
    let first = off_lane[0] - 1;
    let last = off_lane.iter().zip(off_lane.iter().skip(1)).find(|(a, b)| **b != **a + 1).map(|(a, _)| *a).unwrap();
    let arc = &p.points[first..=last];
    let (a, b) = (arc[0], arc[arc.len() - 1]);
    let (centre, radius) = (((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0), (b.0 - a.0).abs() / 2.0);
    assert!((a.1 - b.1).abs() < 1e-9 && radius >= 0.1 && radius <= 0.15 + 1e-9, "radius {radius}");
    for q in arc {
        let d = ((q.0 - centre.0).powi(2) + (q.1 - centre.1).powi(2)).sqrt();
        assert!((d - radius).abs() < 1e-9, "{q:?} is {d} from the centre, radius {radius}");
    }
    for i in first..=last {
        assert!(turn_at(&p.points, i) < 0.55, "corner of {:.2} rad at {i}", turn_at(&p.points, i));
    }
}

/// Gardens with trees at the strip widths the app offers: valid and accepted
/// by the navigation server at every angle.
#[test]
fn navigation_accepts_narrow_strip_garden_plans() {
    let mut refused = Vec::new();
    for seed in 0..12u64 {
        let res = if seed % 2 == 0 { 0.05 } else { 0.1 };
        let strip = [0.3, 0.4, 0.5][(seed % 3) as usize];
        let n = (8.0 / res) as usize;
        let trees: Vec<(f64, f64, f64)> = (0..(seed % 4))
            .map(|k| {
                let f = |m: u64| ((seed * 7919 + k * 104729 + m) % 1000) as f64 / 1000.0;
                (1.5 + 5.0 * f(1), 1.5 + 5.0 * f(2), 0.3 + 0.5 * f(3))
            })
            .collect();
        let g = Array2::from_shape_fn((n, n), |(r, c)| {
            let (x, y) = ((c as f64 + 0.5) * res, (r as f64 + 0.5) * res);
            let inside = if seed % 4 == 0 {
                ((x - 4.0) / 3.2).powi(2) + ((y - 4.0) / 2.4).powi(2) < 1.0
            } else {
                (0.8..7.2).contains(&x) && (1.2..6.6).contains(&y)
            };
            inside && trees.iter().all(|&(tx, ty, tr)| (x - tx).powi(2) + (y - ty).powi(2) > (tr + 0.4).powi(2))
        });
        let (g, _, kept) = filter_safe_components_rs(g.view(), res, 0.05, true);
        if kept.is_empty() {
            continue;
        }
        let sm = SafeMap { grid: g.view(), resolution: res, origin_x: 0.0, origin_y: 0.0 };
        for ang in [Some(0.0), Some(30.0), Some(90.0), Some(135.0)] {
            let p = plan_boustrophedon_rs(g.view(), strip, SPACING, res, 0.0, 0.0, ang);
            assert!(validate_path_rs(&p.points, &sm).valid, "seed {seed} {ang:?}");
            assert!(p.invalid_segments.is_empty(), "seed {seed} {ang:?}");
            if let Some(bad) = nav_short_segment(&p.points, &p.split_points) {
                refused.push((seed, ang, bad));
            }
        }
    }
    assert!(refused.is_empty(), "refused plans: {refused:?}");
}

/// Whatever the lane turns cannot do with arcs (lane ends far out of level
/// along a slanted or curved edge, the way from one cell to the next around
/// a tree) has its corners rounded off afterwards: in an open square no
/// corner is left at any angle, and in a garden with a tree hardly any.
#[test]
fn leftover_corners_are_rounded_off() {
    for strip in [0.3, 0.5, 0.8] {
        for ang in [0.0, 15.0, 30.0, 45.0, 75.0, 120.0] {
            let g = ones(120, 120);
            let sm = SafeMap { grid: g.view(), resolution: 0.05, origin_x: 0.0, origin_y: 0.0 };
            let p = plan_boustrophedon_rs(g.view(), strip, SPACING, 0.05, 0.0, 0.0, Some(ang));
            let what = format!("strip {strip} angle {ang}");
            assert!(validate_path_rs(&p.points, &sm).valid, "{what}");
            assert_eq!(nav_short_segment(&p.points, &p.split_points), None, "{what}");
            let sharp: Vec<usize> = (1..p.points.len() - 1).filter(|&i| turn_at(&p.points, i) >= 0.8).collect();
            assert!(sharp.is_empty(), "{what}: sharp corners at {sharp:?}");
            assert!(nav_segments(&p.points, &p.split_points).len() <= p.lanes, "{what}");
            assert!(coverage_ratio_rs(&p.points, &sm, strip) > 0.95, "{what}: coverage {}", coverage_ratio_rs(&p.points, &sm, strip));
        }
    }
    let res = 0.05;
    let g = Array2::from_shape_fn((100, 100), |(r, c)| {
        let (x, y) = ((c as f64 + 0.5) * res, (r as f64 + 0.5) * res);
        ((x - 2.5) / 2.4).powi(2) + ((y - 2.5) / 1.9).powi(2) < 1.0 && (x - 2.9).powi(2) + (y - 2.4).powi(2) > 0.36
    });
    let sm = SafeMap { grid: g.view(), resolution: res, origin_x: 0.0, origin_y: 0.0 };
    for ang in [0.0, 30.0, 90.0] {
        let p = plan_boustrophedon_rs(g.view(), 0.3, SPACING, res, 0.0, 0.0, Some(ang));
        assert!(validate_path_rs(&p.points, &sm).valid, "{ang}");
        assert_eq!(nav_short_segment(&p.points, &p.split_points), None, "{ang}");
        let sharp = (1..p.points.len() - 1).filter(|&i| turn_at(&p.points, i) >= 0.8).count();
        // the few left are at the ends of the sweep, where a lane is much
        // shorter than the one before it and the way between them hugs the edge
        assert!(sharp * 2 <= p.turns, "angle {ang}: {sharp} sharp corners in {} turns", p.turns);
        assert!(coverage_ratio_rs(&p.points, &sm, 0.3) > 0.93, "{ang}: coverage {}", coverage_ratio_rs(&p.points, &sm, 0.3));
    }
}

fn garden(res: f64) -> Array2<bool> {
    let n = (5.0 / res).round() as usize;
    Array2::from_shape_fn((n, n), |(r, c)| {
        let (x, y) = ((c as f64 + 0.5) * res, (r as f64 + 0.5) * res);
        ((x - 2.5) / 2.4).powi(2) + ((y - 2.5) / 1.9).powi(2) < 1.0 && (x - 2.9).powi(2) + (y - 2.4).powi(2) > 0.36
    })
}

/// With a headland the mower first laps the edge and the tree, and the lanes
/// stay inside those laps: every turn between lanes has room for its arc, so
/// the only corners left are where a lap closes, and more is mowed than
/// without one.
#[test]
fn headland_plans_turn_on_arcs_and_mow_more() {
    for (res, strip) in [(0.05, 0.3), (0.05, 0.5), (0.1, 0.4)] {
        let g = garden(res);
        let sm = SafeMap { grid: g.view(), resolution: res, origin_x: 0.0, origin_y: 0.0 };
        for ang in [Some(0.0), Some(30.0), Some(90.0), Some(135.0), None] {
            let what = format!("res {res} strip {strip} angle {ang:?}");
            let p = plan_boustrophedon_headland_rs(g.view(), strip, SPACING, res, 0.0, 0.0, ang);
            let plain = plan_boustrophedon_rs(g.view(), strip, SPACING, res, 0.0, 0.0, ang.or(Some(p.angle_deg)));
            assert!(validate_path_rs(&p.points, &sm).valid, "{what}");
            assert!(p.invalid_segments.is_empty(), "{what}");
            assert_eq!(nav_short_segment(&p.points, &p.split_points), None, "{what}");
            for &q in &p.points {
                assert!(strictly_inside(q, &g, res, 0.0, 0.0), "{what}: {q:?} outside the safe grid");
            }
            // it is a headland plan, not the fallback: longer by about the laps
            assert!(p.length_m > plain.length_m + 4.0, "{what}: {:.1} m, without {:.1} m", p.length_m, plain.length_m);
            let (cov, cov_plain) = (coverage_ratio_rs(&p.points, &sm, strip), coverage_ratio_rs(&plain.points, &sm, strip));
            // (on a 0.1 m grid the laps' one-cell margins cost about what the turns gain)
            assert!(cov > 0.97 && cov + 0.01 >= cov_plain, "{what}: coverage {cov:.3}, without {cov_plain:.3}");
            if res < 0.1 {
                assert!(cov > cov_plain + 0.01, "{what}: coverage {cov:.3}, without {cov_plain:.3}");
            }
            let sharp = (1..p.points.len() - 1).filter(|&i| turn_at(&p.points, i) >= 0.8).count();
            assert!(sharp <= 4, "{what}: {sharp} sharp corners");
            // a lap is never one FollowPath segment: it would end where it starts
            for (a, b) in nav_segments(&p.points, &p.split_points) {
                let d = ((p.points[a].0 - p.points[b].0).powi(2) + (p.points[a].1 - p.points[b].1).powi(2)).sqrt();
                assert!(d > 0.2, "{what}: segment {a}..{b} ends {d:.2} m from its start");
            }
        }
    }
}

/// Headland plans of the oracle maps (noise, corridors, tiny regions) are
/// valid and accepted by the navigation server, or fall back to the plan
/// without one.
#[test]
fn headland_plans_of_every_map_are_safe() {
    for (name, g, res, ox, oy) in node_maps() {
        let sm = SafeMap { grid: g.view(), resolution: res, origin_x: ox, origin_y: oy };
        for strip in [0.3, STRIP] {
            for ang in [Some(0.0), Some(45.0), Some(90.0)] {
                let p = plan_boustrophedon_headland_rs(g.view(), strip, SPACING, res, ox, oy, ang);
                let plain = plan_boustrophedon_rs(g.view(), strip, SPACING, res, ox, oy, ang);
                let what = format!("{name} strip {strip} {ang:?}");
                assert_eq!(p.points.is_empty(), plain.points.is_empty(), "{what}");
                assert!(p.invalid_segments.len() <= plain.invalid_segments.len(), "{what}: invalid {:?}", p.invalid_segments);
                if p.invalid_segments.is_empty() {
                    assert!(validate_path_rs(&p.points, &sm).valid, "{what}");
                }
                for &q in &p.points {
                    assert!(strictly_inside(q, &g, res, ox, oy), "{what}: {q:?} outside the safe grid");
                }
                assert_eq!(nav_short_segment(&p.points, &p.split_points), None, "{what}");
            }
        }
    }
}
