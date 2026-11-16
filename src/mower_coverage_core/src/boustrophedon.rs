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
//! - two lanes of a cell are joined without a square turn: by arcs (a half
//!   circle of half the lane gap where the lane ends are level), or, where
//!   the ends are far out of level, by a hop with a single pivot;
//! - [`plan_boustrophedon_headland_rs`] mows a lap along the edge and round
//!   every large obstacle first and keeps the lanes inside those laps, so the
//!   turns are made where a lap has mowed and cost the lanes nothing;
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

/// Smallest radius of the arcs that join two lanes. The mower is a
/// differential drive, so no radius is too tight to drive: under half its
/// 0.35 m track the inner wheel simply runs backwards and the arc becomes a
/// pivot about a point between the wheels. The limit only keeps a turn clear
/// of the 0.15 m segments the navigation server refuses. It was 0.35 m, the
/// legacy generator's value, which left every strip narrower than 0.7 m with
/// a square turn: pivot, hop sideways, pivot again.
pub const MIN_U_TURN_RADIUS_M: f64 = 0.10;
/// Largest change of heading from one chord of an arc to the next. Under the
/// 0.8 rad at which the navigation server cuts a path
/// (`nav_split::NAV_TURN_SPLIT_ANGLE_RAD`), so a turn is driven in one piece.
const MAX_ARC_STEP_RAD: f64 = std::f64::consts::PI / 6.0;
/// Furthest a turn is moved back from the lane ends to clear the edge's
/// raster staircase. Every centimetre is left unmowed at two lane ends, so a
/// turn that needs more keeps its square hop and the lanes their length.
const MAX_TURN_PULL_BACK_M: f64 = 0.15;
/// A corner of at least this much, wherever it is left in the path (a hop
/// between lanes whose ends are far out of level, the way from one cell to
/// the next, an A* connector), is rounded off by [`round_corners`].
const CORNER_MIN_TURN_RAD: f64 = 0.6;
/// A leftover corner may cut this much off its legs, or half the strip width
/// when that is more. A hairpin (a lane that ends in the acute corner of a
/// slanted edge) needs a long run-in for even the smallest radius.
const MAX_CORNER_CUT_M: f64 = 0.4;
/// Smallest radius a leftover corner is rounded with. Below the lane turns'
/// minimum: such a corner often has only a short hop to either side.
const MIN_CORNER_RADIUS_M: f64 = 0.05;
/// A hop between two lanes whose ends are too far out of level for arcs is
/// driven with one pivot when it leaves (or joins) a lane at less than this:
/// the navigation server does not cut there, only at the sharp end.
const HOP_FLOW_ANGLE_RAD: f64 = 0.7;
/// Cost, in metres of path, charged for every lane-to-lane turn when the
/// sweep angle and the cell order are compared. A differential-drive U-turn
/// slows down for the arc (or pivots) and starts a new FollowPath segment
/// (the navigation server splits the path there): roughly 4-6 s, i.e. 2-3 m
/// at 0.5 m/s.
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

/// Turn between two lanes of a cell, without a corner. The straight hop from
/// this lane's end to the next lane's start has both its corners rounded
/// with one radius: lane, arc, what is left of the hop, arc, lane. Level
/// lane ends (the usual case) leave nothing of the hop, and the two arcs are
/// one half circle of half the lane gap. Lane ends out of level, along a
/// slanted edge, get a smaller radius, so that neither lane is cut short by
/// more than half the gap; the radius stops at [`MIN_U_TURN_RADIUS_M`], and
/// from there the sharper corner costs more of its lane.
///
/// Returns where this lane now ends, where the next one starts, and the
/// turn's points from the one to the other. `None` when the lanes are closer
/// than two minimum radii, a lane would lose more than the lane gap or all of
/// its length, or the turn is unsafe even pulled back.
fn u_turn(
    cur: (f64, f64, f64),
    next: (f64, f64, f64),
    spacing: f64,
    res: f64,
    frame: Frame,
    sm: &SafeMap,
    headland: bool,
) -> Option<(f64, f64, Vec<(f64, f64)>)> {
    use std::f64::consts::PI;
    let (s0, e0, a0) = cur;
    let (s1, e1, a1) = next;
    let up = e0 > s0;
    if (e1 > s1) == up || (e0 - s0).abs() < res || (e1 - s1).abs() < res {
        return None;
    }
    let dir = if up { 1.0 } else { -1.0 };
    let side = if a1 > a0 { 1.0 } else { -1.0 };
    let gap = (a1 - a0).abs();
    if gap / 2.0 < MIN_U_TURN_RADIUS_M {
        return None;
    }
    // Lane ends further out of level than the lane gap would need arcs that
    // cut more than the gap off a lane. The lane that reaches further is
    // shortened to a 45 degree hop instead, when MAX_CORNER_CUT_M does it.
    let (mut e0, mut s1) = (e0, s1);
    let over = ((s1 - e0) * dir).abs() - gap;
    if over > 0.0 {
        if over > MAX_CORNER_CUT_M {
            return None;
        }
        if (s1 - e0) * dir > 0.0 {
            s1 -= dir * over;
        } else {
            e0 -= dir * over;
        }
    }
    // the hop, and the turns onto it (theta0) and off it (theta1)
    let ahead = (s1 - e0) * dir;
    let hop = (gap * gap + ahead * ahead).sqrt();
    let theta0 = gap.atan2(ahead);
    let theta1 = PI - theta0;
    let (t0, t1) = ((theta0 / 2.0).tan(), (theta1 / 2.0).tan());
    // the widest radius uses up the whole hop
    let radius = (gap / 2.0 / t0.max(t1)).max(MIN_U_TURN_RADIUS_M).min(hop / (t0 + t1));
    if radius < MIN_U_TURN_RADIUS_M {
        return None;
    }
    let (trim0, trim1) = (radius * t0, radius * t1);
    if trim0.max(trim1) > gap + 1e-9 {
        return None;
    }
    // heading turned by `t` since the lane: (side * sin t, dir * cos t)
    let at = |centre: (f64, f64), t: f64| frame.xy(centre.0 - side * radius * t.cos(), centre.1 + dir * radius * t.sin());
    let chords = |theta: f64| ((theta / MAX_ARC_STEP_RAD - 1e-9).ceil().max((theta * radius / spacing.max(res) - 1e-9).ceil()) as usize).max(1);
    let (n0, n1) = (chords(theta0), chords(theta1));
    // Along a curved or slanted edge both lanes end flush with it, and the
    // raster's staircase reaches in between them: the turn is then pulled
    // back along the lanes, a cell at a time and by MAX_TURN_PULL_BACK_M at
    // most, until it is clear.
    // With a headland the lanes end short of the edge and the turn starts
    // out past their ends, in the headland, so no lane is cut short for it.
    let mut back = if headland { -trim0.min(trim1) } else { 0.0 };
    while back <= MAX_TURN_PULL_BACK_M.min(gap / 2.0) + 1e-9 {
        let (end0, start1) = (e0 - dir * (trim0 + back), s1 - dir * (trim1 + back));
        // both lanes must still be driven up to the turn's start / from its end
        if (end0 - s0) * dir <= 0.0 || (start1 - e1) * dir <= 0.0 {
            return None;
        }
        let mut turn = Vec::new();
        for i in 0..=n0 {
            push_unique(&mut turn, at((a0 + side * radius, end0), theta0 * i as f64 / n0 as f64));
        }
        for i in 0..=n1 {
            push_unique(&mut turn, at((a1 - side * radius, start1), theta0 + theta1 * i as f64 / n1 as f64));
        }
        if path_ok(&turn, sm) {
            return Some((end0, start1, turn));
        }
        back += res;
    }
    None
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
    headland: bool,
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
            let lane_first = out.points.len() - 1;
            let mut end_b = e;
            let mut arc = None;
            if li + 1 < lanes.len() {
                if let Some((trim_end, next_start, pts)) =
                    u_turn((start_b, e, a), lanes[li + 1], spacing, res, frame, sm, headland)
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
                        // The segment ends one waypoint before the arc, so
                        // the arc is driven at the start of the next one,
                        // with the whole lane ahead to settle on. A split
                        // point on the arc would not do: the navigation
                        // server cuts at every pose within 0.1 m of one, and
                        // a tight arc's poses are closer together than that.
                        let split = out.points.len().saturating_sub(2).max(lane_first);
                        out.split_points.push(out.points[split]);
                        for p in pts {
                            push_unique(&mut out.points, p);
                        }
                        start_b = next_start;
                    }
                    None => {
                        let (ns, _, na) = lanes[li + 1];
                        let from = *out.points.last().unwrap();
                        let to = frame.xy(na, ns);
                        // A straight hop that leaves this lane at a shallow
                        // angle (the next lane reaches well past this one)
                        // is driven on without a stop, and the segment ends
                        // at the sharp corner onto the next lane: one pivot.
                        let lane_len = (e - lanes[li].0).abs();
                        let ahead = (ns - e) * (e - lanes[li].0).signum();
                        let flows = lane_len >= res && (na - a).abs().atan2(ahead) < HOP_FLOW_ANGLE_RAD && segment_ok(from, to, sm);
                        if !flows {
                            out.split_points.push(from);
                        }
                        join(&mut out, from, to, sm, use_astar);
                        if flows {
                            out.split_points.push(*out.points.last().unwrap());
                        }
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

fn turn_angle(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
    let (u, v) = ((b.0 - a.0, b.1 - a.1), (c.0 - b.0, c.1 - b.1));
    (u.0 * v.1 - u.1 * v.0).atan2(u.0 * v.0 + u.1 * v.1)
}

/// Round off every corner the lane turns left in the path, so the mower
/// drives through it instead of stopping to pivot: the corner is replaced by
/// an arc tangent to both of its legs. The radius is `radius` at most, less
/// where that would cut more than `max_cut` off a leg (a sharp corner needs
/// a long run-in) or more than the leg has, and never under
/// [`MIN_CORNER_RADIUS_M`]; two corners share the leg between them half and
/// half. An arc cuts the corner
/// on its inside, so the tip of a lane that ends in one is not driven. A
/// corner whose arc is unsafe at every radius stays a corner.
///
/// Split points move with the path: one that sat on a rounded corner goes
/// back to the last pose before the arc that has no neighbour within the
/// navigation server's split tolerance (it cuts at every pose that close to
/// a split point, and an arc's poses are closer together than that).
fn round_corners(out: &mut Assembly, radius: f64, max_cut: f64, spacing: f64, sm: &SafeMap) {
    let pts = std::mem::take(&mut out.points);
    let n = pts.len();
    if n < 3 {
        out.points = pts;
        return;
    }
    let turn: Vec<f64> = (0..n).map(|i| if i == 0 || i + 1 == n { 0.0 } else { turn_angle(pts[i - 1], pts[i], pts[i + 1]) }).collect();
    let corner = |i: usize| turn[i].abs() >= CORNER_MIN_TURN_RAD;
    // length of the straight run from corner `i` backwards / forwards, and
    // whether it ends in another corner
    let leg = |i: usize, step: isize| {
        let (mut j, mut len) = (i, 0.0);
        loop {
            let k = (j as isize + step) as usize;
            len += dist(pts[j], pts[k]);
            j = k;
            if j == 0 || j + 1 == n || turn[j].abs() > 1e-6 {
                return if corner(j) { len / 2.0 } else { len };
            }
        }
    };
    let mut res_pts: Vec<(f64, f64)> = Vec::with_capacity(n);
    // output index of each input pose (of the arc that replaced it)
    let mut map = vec![0usize; n];
    let mut moved = vec![false; n];
    let mut skip_until: Option<((f64, f64), f64)> = None;
    for i in 0..n {
        if let Some((c, d)) = skip_until {
            if !corner(i) && turn[i].abs() <= 1e-6 && i + 1 < n && dist(pts[i], c) < d - 1e-9 {
                map[i] = res_pts.len() - 1;
                moved[i] = true;
                continue;
            }
            skip_until = None;
        }
        let mut arc = None;
        if corner(i) {
            let theta = turn[i].abs();
            let t = (theta / 2.0).tan();
            let room = leg(i, -1).min(leg(i, 1));
            let mut r = radius.min(max_cut / t).min(room / t);
            let (u, v) = (dist(pts[i - 1], pts[i]), dist(pts[i], pts[i + 1]));
            let (ui, vo) = (((pts[i].0 - pts[i - 1].0) / u, (pts[i].1 - pts[i - 1].1) / u), ((pts[i + 1].0 - pts[i].0) / v, (pts[i + 1].1 - pts[i].1) / v));
            let sign = turn[i].signum();
            while r >= MIN_CORNER_RADIUS_M {
                let d = r * t;
                let a = (pts[i].0 - ui.0 * d, pts[i].1 - ui.1 * d);
                let o = (a.0 - sign * ui.1 * r, a.1 + sign * ui.0 * r);
                let chords = ((theta / MAX_ARC_STEP_RAD - 1e-9).ceil().max((theta * r / spacing - 1e-9).ceil()) as usize).max(1);
                let mut cand = Vec::with_capacity(chords + 1);
                for k in 0..=chords {
                    let (sn, cs) = (sign * theta * k as f64 / chords as f64).sin_cos();
                    let (x, y) = (a.0 - o.0, a.1 - o.1);
                    cand.push((o.0 + x * cs - y * sn, o.1 + x * sn + y * cs));
                }
                // the arc ends on the outgoing leg exactly
                *cand.last_mut().unwrap() = (pts[i].0 + vo.0 * d, pts[i].1 + vo.1 * d);
                // What is left of the two legs is checked again: the shared
                // validator's Bresenham line between other end points can
                // clip a cell the whole leg's line missed.
                let keep = res_pts.iter().rposition(|&p| dist(p, pts[i]) >= d - 1e-9).unwrap_or(0);
                let after = (i + 1..n).find(|&m| !(turn[m].abs() <= 1e-6 && m + 1 < n && dist(pts[m], pts[i]) < d - 1e-9)).unwrap();
                let stub_ok = |a: (f64, f64), b: (f64, f64)| same_point(a, b) || segment_ok(a, b, sm);
                if path_ok(&cand, sm) && stub_ok(res_pts[keep], cand[0]) && stub_ok(cand[chords], pts[after]) {
                    arc = Some((cand, d));
                    break;
                }
                r *= 0.7;
            }
        }
        match arc {
            Some((cand, d)) => {
                // the run-in the arc replaces
                while res_pts.len() > 1 && dist(*res_pts.last().unwrap(), pts[i]) < d - 1e-9 {
                    res_pts.pop();
                }
                let first = res_pts.len();
                for p in cand {
                    push_unique(&mut res_pts, p);
                }
                for k in 0..i {
                    if map[k] >= first {
                        map[k] = first.saturating_sub(1);
                        moved[k] = true;
                    }
                }
                map[i] = first.saturating_sub(1);
                moved[i] = true;
                skip_until = Some((pts[i], d));
            }
            None => {
                push_unique(&mut res_pts, pts[i]);
                map[i] = res_pts.len() - 1;
            }
        }
    }
    // split points: follow their pose
    let clear = crate::nav_split::NAV_SPLIT_TOLERANCE_M + 0.02;
    let mut splits = Vec::with_capacity(out.split_points.len());
    for &sp in &out.split_points {
        let Some(i) = (0..n).find(|&i| same_point(pts[i], sp)) else {
            splits.push(sp);
            continue;
        };
        if !moved[i] {
            splits.push(sp);
            continue;
        }
        let mut o = map[i].min(res_pts.len() - 1);
        while o > 0 && (dist(res_pts[o], res_pts[o - 1]) <= clear || o + 1 < res_pts.len() && dist(res_pts[o], res_pts[o + 1]) <= clear) {
            o -= 1;
        }
        if o > 0 {
            splits.push(res_pts[o]);
        }
    }
    splits.dedup_by(|a, b| same_point(*a, *b));
    // never trade a valid path for a rounder one
    if strict_invalid_segments(&res_pts, sm).len() > strict_invalid_segments(&pts, sm).len() {
        out.points = pts;
        return;
    }
    out.points = res_pts;
    out.split_points = splits;
}

struct Candidate {
    cells: Vec<Vec<Interval>>,
    order: Vec<usize>,
    variants: Vec<usize>,
    lanes: usize,
    frame: Frame,
    cost: f64,
}

/// `lanes` is where the lanes go, `sm` where the mower may drive: the same
/// map, or with a headland the region inside it and the whole safe region.
fn candidate(lanes_sm: &SafeMap, sm: &SafeMap, headland: bool, strip: f64, spacing: f64, angle_deg: f64, improve: bool) -> Option<Candidate> {
    let frame = Frame::new(angle_deg);
    let (_, lanes) = lane_intervals(lanes_sm, frame, strip)?;
    let n_lanes = lanes.iter().filter(|l| !l.is_empty()).count();
    let cells = build_cells(&lanes);
    if cells.is_empty() {
        return None;
    }
    let (order, variants) = order_cells(&cells, frame, sm, improve);
    let est = assemble(&cells, &order, &variants, spacing, frame, sm, false, headland);
    // uncovered area, charged as the lane length it would take to mow it
    let area = lanes_sm.grid.iter().filter(|&&v| v).count() as f64 * sm.resolution * sm.resolution;
    let uncovered = (1.0 - coverage_ratio_rs(&est.points, lanes_sm, strip)) * area;
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
fn search_angle(lanes_sm: &SafeMap, sm: &SafeMap, headland: bool, strip: f64, spacing: f64) -> Option<(f64, Candidate)> {
    let coarse = (180.0 / COARSE_STEP_DEG).round() as usize;
    let mut rough: Vec<(f64, f64)> = (0..coarse)
        .filter_map(|i| {
            let a = i as f64 * COARSE_STEP_DEG;
            candidate(lanes_sm, sm, headland, strip, spacing, a, false).map(|c| (a, c.cost))
        })
        .collect();
    rough.sort_by(|x, y| x.1.partial_cmp(&y.1).unwrap().then(x.0.partial_cmp(&y.0).unwrap()));
    let mut best: Option<(f64, Candidate)> = None;
    let consider = |angle: f64, best: &mut Option<(f64, Candidate)>| {
        let a = angle.rem_euclid(180.0);
        if let Some(c) = candidate(lanes_sm, sm, headland, strip, spacing, a, true) {
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

/// A headland ring as a path: the cells' centres, closed, pulled straight
/// where that stays within `tol` of them and safe (a ring of cell centres is
/// a staircase), then cut into steps of `spacing` at most. Corners are left
/// to [`round_corners`].
fn ring_path(ring: &[(usize, usize)], sm: &SafeMap, spacing: f64, tol: f64) -> Option<Vec<(f64, f64)>> {
    let centre = |&(r, c): &(usize, usize)| (sm.origin_x + (c as f64 + 0.5) * sm.resolution, sm.origin_y + (r as f64 + 0.5) * sm.resolution);
    let mut pts: Vec<(f64, f64)> = ring.iter().map(centre).collect();
    pts.push(pts[0]);
    let n = pts.len() - 1;
    if n < 4 || !path_ok(&pts, sm) {
        return None;
    }
    // from each kept point, the longest chord that stays within `tol` of
    // the cells it skips and is safe
    let off = |pa: (f64, f64), pb: (f64, f64), q: (f64, f64)| {
        let len = dist(pa, pb);
        if len <= SAME_POINT_TOL { dist(q, pa) } else { ((pb.0 - pa.0) * (q.1 - pa.1) - (pb.1 - pa.1) * (q.0 - pa.0)).abs() / len }
    };
    let mut kept = vec![pts[0]];
    let mut a = 0;
    while a < n {
        let mut best = a + 1;
        for b in a + 2..=n {
            if (a + 1..b).any(|i| off(pts[a], pts[b], pts[i]) > tol) {
                break;
            }
            if segment_ok(pts[a], pts[b], sm) {
                best = b;
            }
        }
        kept.push(pts[best]);
        a = best;
    }
    let mut out = vec![kept[0]];
    for w in kept.windows(2) {
        let steps = ((dist(w[0], w[1]) / spacing) - 1e-9).ceil().max(1.0) as usize;
        let piece: Vec<(f64, f64)> = (0..=steps)
            .map(|i| if i == steps { w[1] } else { (w[0].0 + (w[1].0 - w[0].0) * i as f64 / steps as f64, w[0].1 + (w[1].1 - w[0].1) * i as f64 / steps as f64) })
            .collect();
        // the shared validator's line between two points of a safe chord
        // can clip a cell the chord's own line missed: such a chord stays whole
        if path_ok(&piece, sm) {
            for &p in &piece[1..] {
                push_unique(&mut out, p);
            }
        } else {
            push_unique(&mut out, w[1]);
        }
    }
    path_ok(&out, sm).then_some(out)
}

/// The headland laps as the start of a path: one ring after the other,
/// joined by the shortest safe way. A lap ends where it began, and the
/// navigation server takes a path whose goal the mower already stands on for
/// done, so each lap is cut in three.
fn headland_laps(rings: &[Vec<(f64, f64)>], sm: &SafeMap) -> Assembly {
    let mut out = Assembly::new();
    for ring in rings {
        match out.points.last().copied() {
            None => out.points.push(ring[0]),
            Some(last) => join(&mut out, last, ring[0], sm, true),
        }
        let first = out.points.len() - 1;
        for &p in &ring[1..] {
            push_unique(&mut out.points, p);
        }
        let n = out.points.len() - 1 - first;
        for cut in [n / 3, 2 * n / 3, n] {
            if cut > 0 {
                out.split_points.push(out.points[first + cut]);
            }
        }
    }
    out
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
    plan(safe, strip_width_m, waypoint_spacing_m, res, origin_x, origin_y, angle_deg, false)
}

/// [`plan_boustrophedon_rs`] with a headland: first one lap along the edge
/// of the safe region and one round every obstacle at least a strip wide,
/// then the lanes, which stay half a strip (and a cell) inside those laps.
/// The turns between lanes are made in the headland, so the lanes are driven
/// to their ends, and what a turn does leave lies where a lap has mowed.
///
/// Falls back to the plan without a headland when the region is too narrow
/// to leave anything inside one, or pinched so that no lap can be driven.
pub fn plan_boustrophedon_headland_rs(
    safe: ndarray::ArrayView2<bool>,
    strip_width_m: f64,
    waypoint_spacing_m: f64,
    res: f64,
    origin_x: f64,
    origin_y: f64,
    angle_deg: Option<f64>,
) -> BoustrophedonPlan {
    plan(safe, strip_width_m, waypoint_spacing_m, res, origin_x, origin_y, angle_deg, true)
}

#[allow(clippy::too_many_arguments)]
fn plan(
    safe: ndarray::ArrayView2<bool>,
    strip_width_m: f64,
    waypoint_spacing_m: f64,
    res: f64,
    origin_x: f64,
    origin_y: f64,
    angle_deg: Option<f64>,
    headland: bool,
) -> BoustrophedonPlan {
    let sm = SafeMap { grid: safe, resolution: res, origin_x, origin_y };
    let strip = strip_width_m.max(res);
    let spacing = waypoint_spacing_m.max(res);
    let without = || plan(safe, strip_width_m, waypoint_spacing_m, res, origin_x, origin_y, angle_deg, false);
    // the laps mow half a strip in from the edge; the lanes start a cell
    // further in, which is the room a turn needs beyond their ends
    let hl = if headland { crate::headland::headland(safe, strip / 2.0 / res + 1.0, (strip / res).ceil() as usize) } else { None };
    let rings: Option<Vec<Vec<(f64, f64)>>> = hl.as_ref().and_then(|h| h.rings.iter().map(|r| ring_path(r, &sm, spacing, res.min(0.1))).collect());
    if headland && rings.is_none() {
        return without();
    }
    let lanes_grid = match &hl {
        Some(h) => h.inner.view(),
        None => safe.view(),
    };
    let lanes_sm = SafeMap { grid: lanes_grid, resolution: res, origin_x, origin_y };
    let chosen = match angle_deg {
        Some(a) => candidate(&lanes_sm, &sm, headland, strip, spacing, a, true).map(|c| (a, c)),
        None => search_angle(&lanes_sm, &sm, headland, strip, spacing),
    };
    let Some((angle, c)) = chosen else {
        if headland {
            return without();
        }
        return BoustrophedonPlan { angle_deg: angle_deg.unwrap_or(0.0), ..Default::default() };
    };
    let mut asm = assemble(&c.cells, &c.order, &c.variants, spacing, c.frame, &sm, true, headland);
    if let Some(rings) = &rings {
        let mut laps = headland_laps(rings, &sm);
        laps.bdist = asm.bdist.take();
        if let (Some(&last), Some(&first)) = (laps.points.last(), asm.points.first()) {
            join(&mut laps, last, first, &sm, true);
        }
        for &p in asm.points.iter().skip(1) {
            push_unique(&mut laps.points, p);
        }
        laps.split_points.extend(asm.split_points);
        laps.connectors.extend(asm.connectors);
        laps.turns += asm.turns;
        laps.unsafe_transitions += asm.unsafe_transitions;
        asm = laps;
    }
    repair(&mut asm, &sm);
    // a quarter of the strip: half of it, as between lanes, would leave too
    // much unmowed where a lane ends in the corner
    round_corners(&mut asm, (strip / 4.0).max(MIN_U_TURN_RADIUS_M), (strip / 2.0).max(MAX_CORNER_CUT_M), spacing, &sm);
    // no FollowPath segment the navigation server would refuse
    let (points, split_points) = coalesce_for_navigation(&asm.points, &asm.split_points);
    asm.points = points;
    asm.split_points = split_points;
    let invalid_segments = strict_invalid_segments(&asm.points, &sm);
    if headland && !invalid_segments.is_empty() {
        let plain = without();
        if plain.invalid_segments.is_empty() {
            return plain;
        }
    }
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
