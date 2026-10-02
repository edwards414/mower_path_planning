//! Boustrophedon coverage planner: straight lanes at any angle, cell
//! decomposition, optimised cell order and optional automatic sweep angle.
//!
//! Replaces the legacy `zigzag::generate_coverage_zigzag_path_rs` for the
//! coverage node. Compared with it:
//! - lanes are straight lines at every angle (the legacy rotated branch
//!   zig-zagged sideways inside each band);
//! - lanes are placed from the safe region's own extent (first and last lane
//!   just inside its outermost cells, spacing <= strip width), not on a fixed
//!   grid stride, so the band along the edges is not skipped;
//! - a lane that grazes the staircase of a raster edge is cleaned up: the
//!   outermost lanes move in until they clear an edge they run along, and
//!   the crumbs a lane leaves where it crosses an edge at a shallow angle
//!   are dropped (each was a cell, a turn and a few jogs of its own);
//! - the lane intervals are grouped into boustrophedon cells (a cell ends
//!   wherever an obstacle splits or merges the free space), each cell is
//!   mowed back and forth on its own, and the cell order and each cell's
//!   entry corner are optimised (DP over entry variants + 2-opt);
//! - with `angle_deg = None` the sweep angle that minimises
//!   `path length + TURN_PENALTY_M * turns` is searched;
//! - unsafe transitions are replaced by A* connectors that are pulled
//!   straight (`simplify_path_rs`), so the returned path is normally valid.
//!
//! Angle convention (same as the legacy generator): `angle_deg = 0` gives
//! lanes parallel to the map y axis, stepping in +x; a positive angle rotates
//! the lanes counter-clockwise.

use crate::connector_planner::{boundary_distance, plan_connector_with_bdist_rs};
use crate::nav_split::coalesce_for_navigation;
use crate::path_validator::{is_point_safe_rs, is_segment_safe_rs};
use crate::types::SafeMap;

/// Smallest U-turn radius the mower can drive (same as the legacy generator).
pub const MIN_U_TURN_RADIUS_M: f64 = 0.35;
/// Cost, in metres of path, charged for every lane-to-lane turn when the
/// sweep angle and the cell order are compared. A differential-drive U-turn
/// slows down, pivots and starts a new FollowPath segment (the navigation
/// server splits the path there): roughly 4-6 s, i.e. 2-3 m at 0.5 m/s.
pub const TURN_PENALTY_M: f64 = 3.0;
/// Estimated detour factor of a transition whose straight line is unsafe.
const UNSAFE_TRANSITION_FACTOR: f64 = 2.0;
/// Uncovered area is charged as `UNCOVERED_WEIGHT * area / strip` metres:
/// the lane length it would take to mow it, weighted so that coverage wins
/// over a shorter path (1 % of the area left costs about 10 % more length).
const UNCOVERED_WEIGHT: f64 = 10.0;
/// Coarse angle search step, then a fine search around the best one.
const COARSE_STEP_DEG: f64 = 5.0;
const FINE_STEP_DEG: f64 = 1.0;
const REFINE_TOP: usize = 6;
/// Above this many cells the order is built greedily instead of by 2-opt.
const MAX_CELLS_FOR_2OPT: usize = 80;
/// A gap of fewer cells between two safe stretches of one lane, next to a
/// wide unsafe region, is a raster artefact (the lane grazes the staircase of
/// that region's edge), never an obstacle: obstacles reach the planner
/// inflated by the robot's footprint.
const GRAZE_GAP_CELLS: f64 = 4.0;
/// Stretches shorter than this next to a graze are crumbs.
const CRUMB_CELLS: f64 = 4.0;
const SAME_POINT_TOL: f64 = 1e-9;

/// Result of [`plan_boustrophedon_rs`].
#[derive(Clone, Debug, Default)]
pub struct BoustrophedonPlan {
    /// Waypoints, world frame.
    pub points: Vec<(f64, f64)>,
    /// Ends of the navigation segments (same meaning as the legacy zigzag).
    pub split_points: Vec<(f64, f64)>,
    /// Segments that are still unsafe (no A* connector found).
    pub invalid_segments: Vec<(usize, usize)>,
    /// A* connectors inserted into `points` (after simplification).
    pub connectors: Vec<Vec<(f64, f64)>>,
    /// Sweep angle actually used.
    pub angle_deg: f64,
    pub lanes: usize,
    pub cells: usize,
    /// Lane-to-lane transitions (U-turns, straight hops and connectors).
    pub turns: usize,
    /// Polyline length of `points`, metres.
    pub length_m: f64,
}

// ── Geometry helpers ─────────────────────────────────────────────────────────

#[derive(Clone, Copy)]
struct Frame {
    u: (f64, f64), // across the lanes
    v: (f64, f64), // along the lanes
}

impl Frame {
    fn new(angle_deg: f64) -> Self {
        let t = angle_deg.to_radians();
        Frame { u: (t.cos(), t.sin()), v: (-t.sin(), t.cos()) }
    }
    fn ab(&self, p: (f64, f64)) -> (f64, f64) {
        (p.0 * self.u.0 + p.1 * self.u.1, p.0 * self.v.0 + p.1 * self.v.1)
    }
    fn xy(&self, a: f64, b: f64) -> (f64, f64) {
        (a * self.u.0 + b * self.v.0, a * self.u.1 + b * self.v.1)
    }
}

fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt()
}

fn same_point(a: (f64, f64), b: (f64, f64)) -> bool {
    (a.0 - b.0).abs() <= SAME_POINT_TOL && (a.1 - b.1).abs() <= SAME_POINT_TOL
}

fn push_unique(pts: &mut Vec<(f64, f64)>, p: (f64, f64)) {
    if pts.last().map_or(true, |&q| !same_point(q, p)) {
        pts.push(p);
    }
}

/// Point check with floor rounding. The shared validator truncates towards
/// zero (Python `int()` parity), so a point up to one cell left of or below
/// the grid maps to row/column 0; the planner never emits such points.
fn point_ok(p: (f64, f64), sm: &SafeMap) -> bool {
    let c = ((p.0 - sm.origin_x) / sm.resolution).floor();
    let r = ((p.1 - sm.origin_y) / sm.resolution).floor();
    c >= 0.0 && r >= 0.0 && is_point_safe_rs(p.0, p.1, sm)
}

/// Every cell the straight segment touches (supercover, Amanatides-Woo),
/// both side cells where it passes exactly through a cell corner, must be
/// safe. Stricter than the shared Bresenham check, which can clip an unsafe
/// corner cell and lets a line through the zero-width gap between two
/// diagonal unsafe cells (A* forbids that corner cut).
fn supercover_ok(p: (f64, f64), q: (f64, f64), sm: &SafeMap) -> bool {
    let res = sm.resolution;
    let (h, w) = (sm.grid.nrows() as i64, sm.grid.ncols() as i64);
    let safe = |r: i64, c: i64| r >= 0 && c >= 0 && r < h && c < w && sm.grid[[r as usize, c as usize]];
    let (x0, y0) = ((p.0 - sm.origin_x) / res, (p.1 - sm.origin_y) / res);
    let (x1, y1) = ((q.0 - sm.origin_x) / res, (q.1 - sm.origin_y) / res);
    let (mut c, mut r) = (x0.floor() as i64, y0.floor() as i64);
    let (c_end, r_end) = (x1.floor() as i64, y1.floor() as i64);
    if !safe(r, c) {
        return false;
    }
    let (dx, dy) = (x1 - x0, y1 - y0);
    let (sc, sr) = (if dx > 0.0 { 1 } else { -1 }, if dy > 0.0 { 1 } else { -1 });
    // parameter t in [0, 1] at which the segment crosses the next cell edge
    let t_for = |v: f64, d: f64| {
        if d > 0.0 {
            (v.floor() + 1.0 - v) / d
        } else if d < 0.0 {
            (v - v.floor()) / -d
        } else {
            f64::INFINITY
        }
    };
    let mut t_max_x = t_for(x0, dx);
    let mut t_max_y = t_for(y0, dy);
    let t_dx = if dx == 0.0 { f64::INFINITY } else { (1.0 / dx).abs() };
    let t_dy = if dy == 0.0 { f64::INFINITY } else { (1.0 / dy).abs() };
    let mut guard = 0;
    while (c, r) != (c_end, r_end) {
        guard += 1;
        if guard > 100_000 {
            return false;
        }
        let tie = (t_max_x - t_max_y).abs() < 1e-12;
        if tie {
            if t_max_x > 1.0 {
                break;
            }
            // through a corner: both side cells count
            if !safe(r, c + sc) || !safe(r + sr, c) {
                return false;
            }
            c += sc;
            r += sr;
            t_max_x += t_dx;
            t_max_y += t_dy;
        } else if t_max_x < t_max_y {
            if t_max_x > 1.0 {
                break;
            }
            c += sc;
            t_max_x += t_dx;
        } else {
            if t_max_y > 1.0 {
                break;
            }
            r += sr;
            t_max_y += t_dy;
        }
        if !safe(r, c) {
            return false;
        }
    }
    safe(r_end, c_end)
}

fn segment_ok(p: (f64, f64), q: (f64, f64), sm: &SafeMap) -> bool {
    point_ok(p, sm) && point_ok(q, sm) && is_segment_safe_rs(p, q, sm) && supercover_ok(p, q, sm)
}

fn path_ok(points: &[(f64, f64)], sm: &SafeMap) -> bool {
    points.iter().all(|&p| point_ok(p, sm)) && points.windows(2).all(|w| segment_ok(w[0], w[1], sm))
}

/// Segments that are unsafe under the shared validator or leave the grid.
fn strict_invalid_segments(points: &[(f64, f64)], sm: &SafeMap) -> Vec<(usize, usize)> {
    (0..points.len().saturating_sub(1))
        .filter(|&i| !segment_ok(points[i], points[i + 1], sm))
        .map(|i| (i, i + 1))
        .collect()
}

/// Length of a polyline.
pub fn path_length_rs(points: &[(f64, f64)]) -> f64 {
    points.windows(2).map(|w| dist(w[0], w[1])).sum()
}

/// Pull a polyline straight: keep a point only when the straight segment
/// from the last kept point to the next one would be unsafe. The endpoints
/// are kept. Used on A* connectors, whose output is one waypoint per cell.
pub fn simplify_path_rs(points: &[(f64, f64)], sm: &SafeMap) -> Vec<(f64, f64)> {
    if points.len() <= 2 {
        return points.to_vec();
    }
    let mut out = vec![points[0]];
    let mut i = 0;
    while i < points.len() - 1 {
        let mut j = points.len() - 1;
        while j > i + 1 && !segment_ok(points[i], points[j], sm) {
            j -= 1;
        }
        out.push(points[j]);
        i = j;
    }
    out
}

/// Fraction of the safe cells whose centre lies within `strip_width_m / 2`
/// of the path (the part of the reachable area the blade passes over).
pub fn coverage_ratio_rs(points: &[(f64, f64)], sm: &SafeMap, strip_width_m: f64) -> f64 {
    let (h, w) = (sm.grid.nrows(), sm.grid.ncols());
    let total = sm.grid.iter().filter(|&&v| v).count();
    if total == 0 {
        return 1.0;
    }
    if points.is_empty() {
        return 0.0;
    }
    let r = strip_width_m / 2.0;
    let res = sm.resolution;
    let mut covered = vec![false; h * w];
    let segs: Vec<((f64, f64), (f64, f64))> = if points.len() == 1 {
        vec![(points[0], points[0])]
    } else {
        points.windows(2).map(|s| (s[0], s[1])).collect()
    };
    for (p, q) in segs {
        let (x0, x1) = (p.0.min(q.0) - r, p.0.max(q.0) + r);
        let (y0, y1) = (p.1.min(q.1) - r, p.1.max(q.1) + r);
        let c0 = (((x0 - sm.origin_x) / res).floor().max(0.0)) as usize;
        let c1 = (((x1 - sm.origin_x) / res).floor().max(-1.0) as i64).min(w as i64 - 1);
        let r0 = (((y0 - sm.origin_y) / res).floor().max(0.0)) as usize;
        let r1 = (((y1 - sm.origin_y) / res).floor().max(-1.0) as i64).min(h as i64 - 1);
        if c1 < 0 || r1 < 0 {
            continue;
        }
        let (dx, dy) = (q.0 - p.0, q.1 - p.1);
        let len2 = dx * dx + dy * dy;
        for row in r0..=(r1 as usize) {
            for col in c0..=(c1 as usize) {
                let k = row * w + col;
                if covered[k] || !sm.grid[[row, col]] {
                    continue;
                }
                let cx = sm.origin_x + (col as f64 + 0.5) * res;
                let cy = sm.origin_y + (row as f64 + 0.5) * res;
                let t = if len2 > 0.0 {
                    (((cx - p.0) * dx + (cy - p.1) * dy) / len2).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let (px, py) = (p.0 + t * dx, p.1 + t * dy);
                if (cx - px).powi(2) + (cy - py).powi(2) <= r * r + 1e-12 {
                    covered[k] = true;
                }
            }
        }
    }
    covered.iter().filter(|&&v| v).count() as f64 / total as f64
}

// ── Lanes and cells ──────────────────────────────────────────────────────────

/// A safe stretch of one lane, in the sweep frame (`b0 < b1`).
#[derive(Clone, Copy, Debug)]
struct Interval {
    lane: usize,
    a: f64,
    b0: f64,
    b1: f64,
}

/// Where the lanes of one angle start along `b` and how many half-cell
/// samples each has.
#[derive(Clone, Copy)]
struct LaneSampling {
    b_start: f64,
    samples: usize,
}

/// Safe stretches of the lane at offset `a`: a sample every half cell, and a
/// new stretch wherever a sample, or the step to it, is unsafe.
fn sample_lane(sm: &SafeMap, frame: Frame, ls: LaneSampling, lane: usize, a: f64) -> Vec<Interval> {
    let step = sm.resolution * 0.5;
    let mut out = Vec::new();
    // current run: (b0, b_last, last point). A single sample is a corner the
    // lane only grazes: still a (zero-length) interval, or that corner is
    // never mowed.
    let mut run: Option<(f64, f64, (f64, f64))> = None;
    for s in 0..ls.samples {
        let b = ls.b_start + s as f64 * step;
        let p = frame.xy(a, b);
        if !point_ok(p, sm) {
            if let Some((b0, b1, _)) = run.take() {
                out.push(Interval { lane, a, b0, b1 });
            }
            continue;
        }
        run = match run {
            Some((b0, _, q)) if segment_ok(q, p, sm) => Some((b0, b, p)),
            other => {
                if let Some((b0, b1, _)) = other {
                    out.push(Interval { lane, a, b0, b1 });
                }
                Some((b, b, p))
            }
        };
    }
    if let Some((b0, b1, _)) = run {
        out.push(Interval { lane, a, b0, b1 });
    }
    out
}

/// Whether the gap between two stretches of the lane at `a` (from `b0` to
/// `b1`) is a graze: short, and with every cell from one to four cells to one
/// side of it unsafe. Then what the lane touches there is the staircase of a
/// wide unsafe region's edge. A speck in the free space leaves a safe cell on
/// both sides: such a gap splits the lane like an obstacle.
fn is_graze(sm: &SafeMap, frame: Frame, a: f64, b0: f64, b1: f64) -> bool {
    let res = sm.resolution;
    if b1 - b0 >= GRAZE_GAP_CELLS * res {
        return false;
    }
    let b = (b0 + b1) / 2.0;
    [1.0, -1.0].iter().any(|side| (1..=4).all(|k| !point_ok(frame.xy(a + side * k as f64 * res, b), sm)))
}

/// Drop the crumbs a lane leaves where it grazes a raster edge: past the
/// outermost long stretch of a group of stretches separated by grazes, the
/// short pieces that alternate with unsafe samples. A lane crossing an edge
/// at a few degrees grazes its staircase for metres; every crumb was a cell
/// of its own (a turn, a connector and a few jogs apiece) for a few
/// centimetres of lane a straight pass cannot join up anyway. Short pieces
/// between two long stretches stay: there the lane rides an edge parallel to
/// it, and only moving the lane helps (see `edge_lane_offset`).
fn drop_crumbs(sm: &SafeMap, frame: Frame, ivs: Vec<Interval>) -> Vec<Interval> {
    let crumb = CRUMB_CELLS * sm.resolution;
    let mut keep = vec![true; ivs.len()];
    let mut i = 0;
    while i < ivs.len() {
        let mut j = i + 1;
        while j < ivs.len() && is_graze(sm, frame, ivs[j].a, ivs[j - 1].b1, ivs[j].b0) {
            j += 1;
        }
        let long: Vec<usize> = (i..j).filter(|&m| ivs[m].b1 - ivs[m].b0 >= crumb).collect();
        if let (Some(&first), Some(&last)) = (long.first(), long.last()) {
            for k in (i..first).chain(last + 1..j) {
                keep[k] = false;
            }
        }
        i = j;
    }
    ivs.into_iter().zip(keep).filter_map(|(iv, k)| k.then_some(iv)).collect()
}

/// Grazes between consecutive stretches of a lane: raster artefacts, each a
/// split of the free space for the cell builder.
fn graze_gaps(sm: &SafeMap, frame: Frame, ivs: &[Interval]) -> usize {
    ivs.windows(2).filter(|p| is_graze(sm, frame, p[1].a, p[0].b1, p[1].b0)).count()
}

/// A lane's stretches, crumbs dropped.
fn lane_at(sm: &SafeMap, frame: Frame, ls: LaneSampling, lane: usize, a: f64) -> Vec<Interval> {
    drop_crumbs(sm, frame, sample_lane(sm, frame, ls, lane, a))
}

/// Offset of an outermost lane: `edge` is the outermost cell centre, `dir`
/// points inwards. The lane starts 0.75 cell inside it and moves in by
/// quarter cells while it grazes the staircase of a raster edge: at an edge
/// parallel to the lanes, or a few tenths of a degree off, 0.75 cell left
/// the lane in a hundred pieces, each a cell with its own turn. Moving in
/// costs no coverage while the outermost cells stay within half a strip, so
/// it may go as far as `max_inset`; it takes the offset with the fewest graze
/// gaps, the outermost one among equals.
fn edge_lane_offset(sm: &SafeMap, frame: Frame, ls: LaneSampling, edge: f64, dir: f64, max_inset: f64) -> f64 {
    let res = sm.resolution;
    let first = (res * 0.75).min(max_inset);
    let mut best: Option<(usize, f64)> = None;
    let mut inset = first;
    while inset <= max_inset + 1e-9 {
        let a = edge + dir * inset;
        let g = graze_gaps(sm, frame, &lane_at(sm, frame, ls, 0, a));
        if g == 0 {
            return a;
        }
        if best.map_or(true, |(bg, _)| g < bg) {
            best = Some((g, a));
        }
        inset += res * 0.25;
    }
    best.map_or(edge + dir * first, |(_, a)| a)
}

/// Lane intervals for one angle; `None` when the map has no safe cell.
fn lane_intervals(sm: &SafeMap, frame: Frame, strip: f64) -> Option<(usize, Vec<Vec<Interval>>)> {
    let res = sm.resolution;
    let (h, w) = (sm.grid.nrows(), sm.grid.ncols());
    let (mut amin, mut amax, mut bmin, mut bmax) =
        (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
    for r in 0..h {
        for c in 0..w {
            if !sm.grid[[r, c]] {
                continue;
            }
            let p = (sm.origin_x + (c as f64 + 0.5) * res, sm.origin_y + (r as f64 + 0.5) * res);
            let (a, b) = frame.ab(p);
            amin = amin.min(a);
            amax = amax.max(a);
            bmin = bmin.min(b);
            bmax = bmax.max(b);
        }
    }
    if !amin.is_finite() {
        return None;
    }
    let step = res * 0.5;
    // offset by a quarter cell so no sample lies on a cell boundary at 0/90 deg
    let b_start = bmin - res + res * 0.25;
    let samples = ((bmax + res - b_start) / step).ceil() as usize + 1;
    let ls = LaneSampling { b_start, samples };
    // the outermost cells stay within half a strip (less two cells) of a lane
    let max_inset = (strip / 2.0 - 2.0 * res).max(res * 0.75).min((amax - amin) / 2.0);
    let lo = edge_lane_offset(sm, frame, ls, amin, 1.0, max_inset);
    let hi = edge_lane_offset(sm, frame, ls, amax, -1.0, max_inset).max(lo);
    let span = hi - lo;
    let n = if span < 1e-9 { 1 } else { (span / strip - 1e-9).ceil() as usize + 1 };

    let lanes = (0..n)
        .map(|k| {
            let a = if n == 1 { (lo + hi) / 2.0 } else { lo + span * k as f64 / (n - 1) as f64 };
            lane_at(sm, frame, ls, k, a)
        })
        .collect();
    Some((n, lanes))
}

/// Boustrophedon cells: runs of intervals on consecutive lanes that overlap
/// one-to-one. A split or a merge of the free space starts new cells.
fn build_cells(lanes: &[Vec<Interval>]) -> Vec<Vec<Interval>> {
    let overlaps = |i: &Interval, j: &Interval| i.b0 <= j.b1 && j.b0 <= i.b1;
    let mut cells: Vec<Vec<Interval>> = Vec::new();
    // cell index of each interval of the previous lane
    let mut prev_cells: Vec<usize> = Vec::new();
    for k in 0..lanes.len() {
        let mut cur_cells = Vec::with_capacity(lanes[k].len());
        for j in &lanes[k] {
            let mut joined = None;
            if k > 0 {
                let above: Vec<usize> = lanes[k - 1]
                    .iter()
                    .enumerate()
                    .filter(|(_, i)| overlaps(i, j))
                    .map(|(idx, _)| idx)
                    .collect();
                if above.len() == 1 {
                    let i = &lanes[k - 1][above[0]];
                    let below = lanes[k].iter().filter(|jj| overlaps(i, jj)).count();
                    let cell = prev_cells[above[0]];
                    let is_tail = cells[cell].last().map_or(false, |t| t.lane == k - 1);
                    if below == 1 && is_tail {
                        joined = Some(cell);
                    }
                }
            }
            let cell = match joined {
                Some(c) => {
                    cells[c].push(*j);
                    c
                }
                None => {
                    cells.push(vec![*j]);
                    cells.len() - 1
                }
            };
            cur_cells.push(cell);
        }
        prev_cells = cur_cells;
    }
    cells
}

// ── Cell traversal ───────────────────────────────────────────────────────────

/// Lane order reversed / first lane driven towards +b.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Variant {
    reversed: bool,
    first_up: bool,
}

const VARIANTS: [Variant; 4] = [
    Variant { reversed: false, first_up: true },
    Variant { reversed: false, first_up: false },
    Variant { reversed: true, first_up: true },
    Variant { reversed: true, first_up: false },
];

/// The lanes of a cell in driving order, each as (start_b, end_b, a).
fn ordered_lanes(cell: &[Interval], v: Variant) -> Vec<(f64, f64, f64)> {
    let mut idx: Vec<usize> = (0..cell.len()).collect();
    if v.reversed {
        idx.reverse();
    }
    idx.iter()
        .enumerate()
        .map(|(n, &i)| {
            let up = v.first_up == (n % 2 == 0);
            let iv = cell[i];
            if up { (iv.b0, iv.b1, iv.a) } else { (iv.b1, iv.b0, iv.a) }
        })
        .collect()
}

fn entry_exit(cell: &[Interval], v: Variant, frame: Frame) -> ((f64, f64), (f64, f64)) {
    let l = ordered_lanes(cell, v);
    let (s, _, a0) = l[0];
    let (_, e, a1) = l[l.len() - 1];
    (frame.xy(a0, s), frame.xy(a1, e))
}

fn transition_cost(p: (f64, f64), q: (f64, f64), sm: &SafeMap) -> f64 {
    let d = dist(p, q);
    if segment_ok(p, q, sm) { d } else { d * UNSAFE_TRANSITION_FACTOR }
}

/// Transition costs from the exit of cell `i` (variant `vi`) to the entry of
/// cell `j` (variant `vj`), computed on first use and cached. Only the pairs
/// the ordering actually looks at are stored (a dense table is 128 B x
/// cells^2, gigabytes on a noisy map).
struct TransitionCosts<'a, 'g> {
    ends: &'a [[((f64, f64), (f64, f64)); 4]],
    sm: &'a SafeMap<'g>,
    tc: std::cell::RefCell<std::collections::HashMap<(usize, usize, usize, usize), f64>>,
}

impl<'a, 'g> TransitionCosts<'a, 'g> {
    fn new(ends: &'a [[((f64, f64), (f64, f64)); 4]], sm: &'a SafeMap<'g>) -> Self {
        TransitionCosts { ends, sm, tc: std::cell::RefCell::new(std::collections::HashMap::new()) }
    }
    fn get(&self, i: usize, vi: usize, j: usize, vj: usize) -> f64 {
        if let Some(&v) = self.tc.borrow().get(&(i, vi, j, vj)) {
            return v;
        }
        let v = transition_cost(self.ends[i][vi].1, self.ends[j][vj].0, self.sm);
        self.tc.borrow_mut().insert((i, vi, j, vj), v);
        v
    }
}

/// Best variant per cell for a fixed cell order (Viterbi over 4 states).
fn best_variants(order: &[usize], tc: &TransitionCosts) -> (f64, Vec<usize>) {
    if order.is_empty() {
        return (0.0, vec![]);
    }
    let mut cost = [0.0f64; 4];
    let mut back: Vec<[usize; 4]> = vec![[0; 4]; order.len()];
    for k in 1..order.len() {
        let (pc, cc) = (order[k - 1], order[k]);
        let mut next = [f64::INFINITY; 4];
        for vi in 0..4 {
            for pi in 0..4 {
                let c = cost[pi] + tc.get(pc, pi, cc, vi);
                if c < next[vi] {
                    next[vi] = c;
                    back[k][vi] = pi;
                }
            }
        }
        cost = next;
    }
    let (mut best, mut bi) = (f64::INFINITY, 0);
    for (i, &c) in cost.iter().enumerate() {
        if c < best {
            best = c;
            bi = i;
        }
    }
    let mut vs = vec![0; order.len()];
    vs[order.len() - 1] = bi;
    for k in (1..order.len()).rev() {
        vs[k - 1] = back[k][vs[k]];
    }
    (best, vs)
}

/// Cell order: sweep order, improved by 2-opt (or nearest-neighbour for very
/// many cells), each candidate scored with its best entry variants.
fn order_cells(cells: &[Vec<Interval>], frame: Frame, sm: &SafeMap, improve: bool) -> (Vec<usize>, Vec<usize>) {
    let ends: Vec<[((f64, f64), (f64, f64)); 4]> = cells
        .iter()
        .map(|c| {
            let mut e = [((0.0, 0.0), (0.0, 0.0)); 4];
            for (i, v) in VARIANTS.iter().enumerate() {
                e[i] = entry_exit(c, *v, frame);
            }
            e
        })
        .collect();
    let mut order: Vec<usize> = (0..cells.len()).collect();
    let tc = TransitionCosts::new(&ends, sm);
    if improve && cells.len() > 2 {
        if cells.len() > MAX_CELLS_FOR_2OPT {
            // nearest neighbour on cell centres, from the first cell
            let centre = |c: &Vec<Interval>| {
                let (a, b) = c.iter().fold((0.0, 0.0), |acc, iv| (acc.0 + iv.a, acc.1 + (iv.b0 + iv.b1) / 2.0));
                frame.xy(a / c.len() as f64, b / c.len() as f64)
            };
            let centres: Vec<(f64, f64)> = cells.iter().map(centre).collect();
            let mut left: Vec<usize> = (1..cells.len()).collect();
            order = vec![0];
            while !left.is_empty() {
                let last = centres[*order.last().unwrap()];
                let (pos, _) = left
                    .iter()
                    .enumerate()
                    .min_by(|x, y| dist(last, centres[*x.1]).partial_cmp(&dist(last, centres[*y.1])).unwrap())
                    .unwrap();
                order.push(left.remove(pos));
            }
        } else {
            let mut best = best_variants(&order, &tc).0;
            let mut improved = true;
            let mut passes = 0;
            while improved && passes < 20 {
                improved = false;
                passes += 1;
                for i in 0..order.len() - 1 {
                    for j in i + 1..order.len() {
                        let mut cand = order.clone();
                        cand[i..=j].reverse();
                        let c = best_variants(&cand, &tc).0;
                        if c + 1e-9 < best {
                            best = c;
                            order = cand;
                            improved = true;
                        }
                    }
                }
            }
        }
    }
    let (_, vs) = best_variants(&order, &tc);
    (order, vs)
}

// ── Assembly ─────────────────────────────────────────────────────────────────

struct Assembly {
    points: Vec<(f64, f64)>,
    split_points: Vec<(f64, f64)>,
    connectors: Vec<Vec<(f64, f64)>>,
    turns: usize,
    /// Straight transitions left unsafe (estimate mode, or no A* path).
    unsafe_transitions: f64,
    /// A*'s boundary-distance cost map, computed on the first connector.
    bdist: Option<ndarray::Array2<f64>>,
}

impl Assembly {
    fn new() -> Self {
        Assembly { points: vec![], split_points: vec![], connectors: vec![], turns: 0, unsafe_transitions: 0.0, bdist: None }
    }
}

fn lane_points(a: f64, b_from: f64, b_to: f64, spacing: f64, frame: Frame) -> Vec<(f64, f64)> {
    let len = (b_to - b_from).abs();
    let n = ((len / spacing) - 1e-9).ceil().max(1.0) as usize;
    (0..=n).map(|i| frame.xy(a, b_from + (b_to - b_from) * i as f64 / n as f64)).collect()
}

/// U-turn between two lanes of a cell: trims both lanes by the turn radius
/// and joins them with a half circle, as the legacy generator did. `None`
/// when the lanes' ends are not level, the gap is too small, or the arc is
/// unsafe.
fn u_turn(
    cur: (f64, f64, f64),
    next: (f64, f64, f64),
    spacing: f64,
    res: f64,
    frame: Frame,
    sm: &SafeMap,
) -> Option<(f64, f64, Vec<(f64, f64)>)> {
    let (s0, e0, a0) = cur;
    let (s1, e1, a1) = next;
    let up = e0 > s0;
    if (e1 > s1) == up || (e0 - s0).abs() < res || (e1 - s1).abs() < res {
        return None;
    }
    if (e0 - s1).abs() > spacing.max(res) * 1.5 {
        return None;
    }
    let radius = (a1 - a0).abs() / 2.0;
    if radius < MIN_U_TURN_RADIUS_M {
        return None;
    }
    let boundary = (e0 + s1) / 2.0;
    let centre_b = if up { boundary - radius } else { boundary + radius };
    // both lanes must still be driven up to the arc's start / from its end
    if up && (centre_b <= s0 || centre_b <= e1) || !up && (centre_b >= s0 || centre_b >= e1) {
        return None;
    }
    let samples = (((std::f64::consts::PI * radius) / spacing.max(res)).ceil() as usize + 1).max(3);
    let mut arc = Vec::with_capacity(samples);
    for i in 0..samples {
        let t = i as f64 / (samples - 1) as f64;
        let a = a0 + (a1 - a0) * t;
        let off = radius * (std::f64::consts::PI * t).sin();
        arc.push(frame.xy(a, if up { centre_b + off } else { centre_b - off }));
    }
    if !path_ok(&arc, sm) {
        return None;
    }
    Some((centre_b, centre_b, arc))
}

/// Join `from` to `to`: straight when safe, else a simplified A* connector
/// (only in `use_astar` mode; the estimate mode leaves it straight).
fn join(out: &mut Assembly, from: (f64, f64), to: (f64, f64), sm: &SafeMap, use_astar: bool) {
    if segment_ok(from, to, sm) {
        push_unique(&mut out.points, to);
        return;
    }
    if use_astar {
        // A* returns cell centres from the start cell to the end cell: put
        // the real endpoints back before checking and pulling it straight.
        let bdist = out.bdist.get_or_insert_with(|| boundary_distance(sm.grid));
        if let Some(c) = plan_connector_with_bdist_rs(from, to, sm, 0.2, bdist) {
            // from -> start-cell centre and end-cell centre -> to stay inside
            // one cell each
            let mut full = vec![from];
            for &p in &c {
                push_unique(&mut full, p);
            }
            push_unique(&mut full, to);
            if path_ok(&full, sm) {
                let full = simplify_path_rs(&full, sm);
                for &p in &full[1..] {
                    push_unique(&mut out.points, p);
                }
                out.connectors.push(full);
                return;
            }
        }
    }
    out.unsafe_transitions += dist(from, to);
    push_unique(&mut out.points, to);
}

fn assemble(
    cells: &[Vec<Interval>],
    order: &[usize],
    variants: &[usize],
    spacing: f64,
    frame: Frame,
    sm: &SafeMap,
    use_astar: bool,
) -> Assembly {
    let res = sm.resolution;
    let mut out = Assembly::new();
    for (k, &ci) in order.iter().enumerate() {
        let lanes = ordered_lanes(&cells[ci], VARIANTS[variants[k]]);
        // start of the first lane may be moved by a U-turn trim; keep as-is here
        let mut start_b = lanes[0].0;
        for (li, &(_, e, a)) in lanes.iter().enumerate() {
            let lane_start = frame.xy(a, start_b);
            if out.points.is_empty() {
                out.points.push(lane_start);
            } else if li == 0 {
                // between cells
                if let Some(&last) = out.points.last() {
                    out.turns += 1;
                    join(&mut out, last, lane_start, sm, use_astar);
                }
            }
            let mut end_b = e;
            let mut arc = None;
            if li + 1 < lanes.len() {
                if let Some((trim_end, next_start, pts)) =
                    u_turn((start_b, e, a), lanes[li + 1], spacing, res, frame, sm)
                {
                    end_b = trim_end;
                    arc = Some((next_start, pts));
                }
            }
            for p in lane_points(a, start_b, end_b, spacing, frame).into_iter().skip(1) {
                push_unique(&mut out.points, p);
            }
            if li + 1 < lanes.len() {
                out.turns += 1;
                match arc {
                    Some((next_start, pts)) => {
                        for p in pts {
                            push_unique(&mut out.points, p);
                        }
                        out.split_points.push(*out.points.last().unwrap());
                        start_b = next_start;
                    }
                    None => {
                        out.split_points.push(*out.points.last().unwrap());
                        let (ns, _, na) = lanes[li + 1];
                        let from = *out.points.last().unwrap();
                        join(&mut out, from, frame.xy(na, ns), sm, use_astar);
                        start_b = ns;
                    }
                }
            } else {
                out.split_points.push(*out.points.last().unwrap());
            }
        }
    }
    out
}

/// Last pass over the assembled path: a lane waypoint between two safe
/// samples can still fall in a corner cell the straight line only grazes
/// (Bresenham skips it), and a trimmed lane end can round across a cell
/// boundary. Drop points that are not safe, join what no longer connects
/// with a simplified A* connector, and move each split point to the nearest
/// point that is left.
fn repair(out: &mut Assembly, sm: &SafeMap) {
    let kept: Vec<(f64, f64)> = out.points.iter().copied().filter(|&p| point_ok(p, sm)).collect();
    if kept.len() == out.points.len() && strict_invalid_segments(&kept, sm).is_empty() {
        return;
    }
    let mut fixed = Assembly::new();
    fixed.bdist = out.bdist.take();
    for &p in &kept {
        match fixed.points.last().copied() {
            None => fixed.points.push(p),
            Some(last) => join(&mut fixed, last, p, sm, true),
        }
    }
    out.connectors.extend(fixed.connectors);
    out.points = fixed.points;
    let pts = &out.points;
    for sp in out.split_points.iter_mut() {
        if let Some(&q) = pts.iter().min_by(|a, b| dist(**a, *sp).partial_cmp(&dist(**b, *sp)).unwrap()) {
            *sp = q;
        }
    }
    out.split_points.dedup_by(|a, b| same_point(*a, *b));
}

struct Candidate {
    cells: Vec<Vec<Interval>>,
    order: Vec<usize>,
    variants: Vec<usize>,
    lanes: usize,
    frame: Frame,
    cost: f64,
}

fn candidate(sm: &SafeMap, strip: f64, spacing: f64, angle_deg: f64, improve: bool) -> Option<Candidate> {
    let frame = Frame::new(angle_deg);
    let (_, lanes) = lane_intervals(sm, frame, strip)?;
    let n_lanes = lanes.iter().filter(|l| !l.is_empty()).count();
    let cells = build_cells(&lanes);
    if cells.is_empty() {
        return None;
    }
    let (order, variants) = order_cells(&cells, frame, sm, improve);
    let est = assemble(&cells, &order, &variants, spacing, frame, sm, false);
    // uncovered safe area, charged as the lane length it would take to mow it
    let safe_area = sm.grid.iter().filter(|&&v| v).count() as f64 * sm.resolution * sm.resolution;
    let uncovered = (1.0 - coverage_ratio_rs(&est.points, sm, strip)) * safe_area;
    let cost = path_length_rs(&est.points)
        + est.unsafe_transitions * (UNSAFE_TRANSITION_FACTOR - 1.0)
        + TURN_PENALTY_M * est.turns as f64
        + UNCOVERED_WEIGHT * uncovered / strip;
    Some(Candidate { cells, order, variants, lanes: n_lanes, frame, cost })
}

/// Angles searched by [`plan_boustrophedon_rs`] with `angle_deg = None`:
/// every `COARSE_STEP_DEG` scored in sweep order, the best `REFINE_TOP` of
/// them re-scored with the optimised cell order, then every `FINE_STEP_DEG`
/// around the winner (also optimised).
fn search_angle(sm: &SafeMap, strip: f64, spacing: f64) -> Option<(f64, Candidate)> {
    let coarse = (180.0 / COARSE_STEP_DEG).round() as usize;
    let mut rough: Vec<(f64, f64)> = (0..coarse)
        .filter_map(|i| {
            let a = i as f64 * COARSE_STEP_DEG;
            candidate(sm, strip, spacing, a, false).map(|c| (a, c.cost))
        })
        .collect();
    rough.sort_by(|x, y| x.1.partial_cmp(&y.1).unwrap().then(x.0.partial_cmp(&y.0).unwrap()));
    let mut best: Option<(f64, Candidate)> = None;
    let consider = |angle: f64, best: &mut Option<(f64, Candidate)>| {
        let a = angle.rem_euclid(180.0);
        if let Some(c) = candidate(sm, strip, spacing, a, true) {
            if best.as_ref().map_or(true, |(_, b)| c.cost + 1e-9 < b.cost) {
                *best = Some((a, c));
            }
        }
    };
    for &(a, _) in rough.iter().take(REFINE_TOP) {
        consider(a, &mut best);
    }
    let centre = best.as_ref()?.0;
    let fine = (COARSE_STEP_DEG / FINE_STEP_DEG).round() as i64;
    for i in -fine + 1..fine {
        if i != 0 {
            consider(centre + i as f64 * FINE_STEP_DEG, &mut best);
        }
    }
    best
}

/// Plan boustrophedon coverage of `safe` (row = y, col = x).
///
/// `angle_deg = Some(a)` sweeps at `a` degrees (0..180, see the module doc);
/// `None` searches the angle with the lowest `length + TURN_PENALTY_M * turns`.
pub fn plan_boustrophedon_rs(
    safe: ndarray::ArrayView2<bool>,
    strip_width_m: f64,
    waypoint_spacing_m: f64,
    res: f64,
    origin_x: f64,
    origin_y: f64,
    angle_deg: Option<f64>,
) -> BoustrophedonPlan {
    let sm = SafeMap { grid: safe, resolution: res, origin_x, origin_y };
    let strip = strip_width_m.max(res);
    let spacing = waypoint_spacing_m.max(res);
    let chosen = match angle_deg {
        Some(a) => candidate(&sm, strip, spacing, a, true).map(|c| (a, c)),
        None => search_angle(&sm, strip, spacing),
    };
    let Some((angle, c)) = chosen else {
        return BoustrophedonPlan { angle_deg: angle_deg.unwrap_or(0.0), ..Default::default() };
    };
    let mut asm = assemble(&c.cells, &c.order, &c.variants, spacing, c.frame, &sm, true);
    repair(&mut asm, &sm);
    // no FollowPath segment the navigation server would refuse
    let (points, split_points) = coalesce_for_navigation(&asm.points, &asm.split_points);
    asm.points = points;
    asm.split_points = split_points;
    let invalid_segments = strict_invalid_segments(&asm.points, &sm);
    BoustrophedonPlan {
        length_m: path_length_rs(&asm.points),
        points: asm.points,
        split_points: asm.split_points,
        invalid_segments,
        connectors: asm.connectors,
        angle_deg: angle,
        lanes: c.lanes,
        cells: c.cells.len(),
        turns: asm.turns,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ndarray::Array2;

    fn open(h: usize, w: usize) -> Array2<bool> {
        Array2::from_elem((h, w), true)
    }

    #[test]
    fn frame_round_trips() {
        for ang in [0.0, 30.0, 90.0, 137.0] {
            let f = Frame::new(ang);
            let (a, b) = f.ab((1.3, -2.7));
            let p = f.xy(a, b);
            assert!((p.0 - 1.3).abs() < 1e-12 && (p.1 + 2.7).abs() < 1e-12);
        }
    }

    #[test]
    fn simplify_keeps_endpoints_and_stays_safe() {
        let g = open(20, 20);
        let sm = SafeMap { grid: g.view(), resolution: 0.1, origin_x: 0.0, origin_y: 0.0 };
        let stair: Vec<(f64, f64)> = (0..10).map(|i| (0.05 + 0.1 * i as f64, 0.05 + 0.1 * (i / 2) as f64)).collect();
        let s = simplify_path_rs(&stair, &sm);
        assert_eq!(s.first(), stair.first());
        assert_eq!(s.last(), stair.last());
        assert_eq!(s.len(), 2);
    }

    #[test]
    fn coverage_ratio_of_nothing_and_everything() {
        let g = open(10, 10);
        let sm = SafeMap { grid: g.view(), resolution: 0.1, origin_x: 0.0, origin_y: 0.0 };
        assert_eq!(coverage_ratio_rs(&[], &sm, 0.4), 0.0);
        assert!((coverage_ratio_rs(&[(0.5, 0.5)], &sm, 10.0) - 1.0).abs() < 1e-12);
    }
}
