//! What the navigation server does with a coverage path, and how to plan so
//! it accepts it.
//!
//! `mower_nav` (and the rclpy `nav_action_server`) cut the path it receives
//! into FollowPath segments:
//! 1. at every *internal* pose within `split_tolerance_m` (0.1) of any split
//!    point (`split_path_by_coverage_points`);
//! 2. inside each piece, at every pose where the heading turns by at least
//!    `turn_split_angle_rad` (0.8) once `turn_split_min_segment_length_m`
//!    (0.25) has been driven since the last cut (`split_paths_by_turn_angle`);
//! 3. then it refuses the whole run if any segment is 0.15 m or shorter
//!    (`coverage_segments_block_reason`).
//!
//! [`nav_segments`] mirrors 1 and 2 (defaults of those parameters; the
//! max-length split is off by default), and [`coalesce_for_navigation`]
//! removes the split points, or trims the few centimetres at a path end,
//! that would make 3 refuse the plan. Keep this in step with
//! `src/mower_rs/crates/mower_nav/src/geometry.rs`.

pub const NAV_SPLIT_TOLERANCE_M: f64 = 0.1;
pub const NAV_TURN_SPLIT_ANGLE_RAD: f64 = 0.8;
pub const NAV_TURN_SPLIT_MIN_SEGMENT_M: f64 = 0.25;
/// Segments of this length or shorter make the navigation server refuse.
pub const NAV_MIN_SEGMENT_M: f64 = 0.15;
/// Kept above `NAV_MIN_SEGMENT_M` so rounding on the other side cannot tip it.
const MARGIN_M: f64 = 0.05;

fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt()
}

fn heading(a: (f64, f64), b: (f64, f64)) -> f64 {
    (b.1 - a.1).atan2(b.0 - a.0)
}

fn angle_diff(a: f64, b: f64) -> f64 {
    ((a - b).sin().atan2((a - b).cos())).abs()
}

/// FollowPath segments the navigation server would make, as inclusive
/// `(first, last)` pose indices (consecutive segments share a pose).
pub fn nav_segments(points: &[(f64, f64)], split_points: &[(f64, f64)]) -> Vec<(usize, usize)> {
    let n = points.len();
    if n < 2 {
        return Vec::new();
    }
    let is_split = |p: (f64, f64)| split_points.iter().any(|&s| dist(p, s) <= NAV_SPLIT_TOLERANCE_M);
    let mut bounds = vec![0];
    for (idx, &p) in points.iter().enumerate().take(n - 1).skip(1) {
        if is_split(p) {
            bounds.push(idx);
        }
    }
    bounds.push(n - 1);
    let mut out = Vec::new();
    for w in bounds.windows(2) {
        let (a, b) = (w[0], w[1]);
        if b - a + 1 < 3 {
            out.push((a, b));
            continue;
        }
        let (mut start, mut driven) = (a, 0.0);
        for idx in a + 1..b {
            driven += dist(points[idx - 1], points[idx]);
            let turn = angle_diff(heading(points[idx - 1], points[idx]), heading(points[idx], points[idx + 1]));
            if turn >= NAV_TURN_SPLIT_ANGLE_RAD && driven >= NAV_TURN_SPLIT_MIN_SEGMENT_M {
                out.push((start, idx));
                start = idx;
                driven = 0.0;
            }
        }
        out.push((start, b));
    }
    out
}

fn length(points: &[(f64, f64)], (a, b): (usize, usize)) -> f64 {
    points[a..=b].windows(2).map(|w| dist(w[0], w[1])).sum()
}

/// The first segment [`nav_segments`] would make that the navigation server
/// refuses (0.15 m or shorter, with a small margin).
pub fn nav_short_segment(points: &[(f64, f64)], split_points: &[(f64, f64)]) -> Option<(usize, usize, f64)> {
    nav_segments(points, split_points)
        .into_iter()
        .map(|s| (s.0, s.1, length(points, s)))
        .find(|s| s.2 <= NAV_MIN_SEGMENT_M + MARGIN_M)
}

/// Make `(points, split_points)` acceptable to the navigation server:
/// repeatedly take the first too-short segment and drop the split point that
/// cuts it off (a later one first), or, when only a sharp turn next to a path
/// end cuts it off, trim those last (first) few centimetres of path. Split
/// points that end up on no path pose are dropped, and the path end is kept as
/// a split point. Only ever removes poses at the two ends, so every remaining
/// segment of the path is unchanged (and as safe as before). The path end
/// stays a split point when that does not itself cut off a short segment.
pub fn coalesce_for_navigation(points: &[(f64, f64)], split_points: &[(f64, f64)]) -> (Vec<(f64, f64)>, Vec<(f64, f64)>) {
    let mut pts = points.to_vec();
    let mut sps = split_points.to_vec();
    let near = |pts: &[(f64, f64)], sps: &[(f64, f64)], i: usize| sps.iter().position(|&s| dist(pts[i], s) <= NAV_SPLIT_TOLERANCE_M);
    let budget = pts.len() + sps.len() + 1;
    for _ in 0..budget {
        if pts.len() < 2 {
            break;
        }
        let Some((a, b, _)) = nav_short_segment(&pts, &sps) else { break };
        let last = pts.len() - 1;
        if b != last {
            if let Some(j) = near(&pts, &sps, b) {
                sps.remove(j);
                continue;
            }
        }
        if a != 0 {
            if let Some(j) = near(&pts, &sps, a) {
                if dist(sps[j], pts[last]) > NAV_SPLIT_TOLERANCE_M {
                    sps.remove(j);
                    continue;
                }
            }
        }
        if b == last {
            pts.truncate(a + 1);
        } else if a == 0 {
            pts.drain(0..b);
        } else {
            // a middle segment between two turn cuts is >= 0.25 m by construction
            break;
        }
    }
    // keep split points that are path poses; the path end is one too unless
    // internal poses lie within the tolerance of it (the server ignores the
    // end itself, but would cut at those poses)
    sps.retain(|&s| pts.iter().any(|&p| dist(p, s) <= 1e-9));
    if let Some(&end) = pts.last() {
        if !sps.iter().any(|&s| dist(s, end) <= 1e-9) {
            let mut with_end = sps.clone();
            with_end.push(end);
            if nav_short_segment(&pts, &with_end).is_none() {
                sps = with_end;
            }
        }
    }
    // in path order, no duplicates
    let index = |s: (f64, f64)| pts.iter().position(|&p| dist(p, s) <= 1e-9).unwrap_or(usize::MAX);
    sps.sort_by_key(|&s| index(s));
    sps.dedup_by(|x, y| dist(*x, *y) <= 1e-9);
    (pts, sps)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn splits_at_internal_split_points_and_sharp_turns() {
        let pts = vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (2.0, 1.0), (2.0, 2.0)];
        assert_eq!(nav_segments(&pts, &[(1.0, 0.0)]), vec![(0, 1), (1, 2), (2, 4)]);
        assert_eq!(nav_segments(&pts, &[]), vec![(0, 2), (2, 4)]);
        // the path end is not internal
        assert_eq!(nav_segments(&pts, &[(2.0, 2.0)]), vec![(0, 2), (2, 4)]);
    }

    #[test]
    fn a_split_point_near_the_next_pose_is_coalesced() {
        // lane end at x=1.0 and the next pose 0.08 m further: two cuts 0.08 m apart
        let pts = vec![(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.08, 0.0), (1.6, 0.0), (2.2, 0.0)];
        let sps = vec![(1.0, 0.0), (2.2, 0.0)];
        assert!(nav_short_segment(&pts, &sps).is_some());
        let (p, s) = coalesce_for_navigation(&pts, &sps);
        assert!(nav_short_segment(&p, &s).is_none());
        assert_eq!(p, pts);
        assert_eq!(s.last(), p.last());
    }

    #[test]
    fn a_short_stub_after_a_turn_at_the_end_is_trimmed() {
        let pts = vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (1.1, 1.0)];
        let (p, s) = coalesce_for_navigation(&pts, &[(1.1, 1.0)]);
        assert!(nav_short_segment(&p, &s).is_none());
        assert_eq!(p, vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]);
        assert_eq!(s, vec![(1.0, 1.0)]);
    }
}
