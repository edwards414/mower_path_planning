//! Headland: the lap (or laps, one per large obstacle) along the edge of the
//! safe region that is mowed before the lanes, and the region left for the
//! lanes inside it. The lanes then end short of the edge, so every turn
//! between them has room, and what a turn leaves unmowed lies in the lap.
//!
//! All of it works on cells: the planner turns a ring into a path.

use ndarray::{Array2, ArrayView2};

/// See [`headland`].
pub struct Headland {
    /// Closed loops of `(row, col)` safe cells, each one step (never a
    /// diagonal) from the one before, the last one step from the first: one
    /// along the safe region's outer edge first, then one round every large
    /// hole. A ring runs one cell inside the edge, so that a straight line
    /// between two of its cells has safe cells on both sides.
    pub rings: Vec<Vec<(usize, usize)>>,
    /// The safe cells further than `width_cells` from the outside and from
    /// every large hole.
    pub inner: Array2<bool>,
}

const N8: [(i64, i64); 8] = [(0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1)];

/// 8-connected components of the cells `pick` selects, labelled from 1 in
/// row-major order of their first cell (0 = not picked).
fn label(h: usize, w: usize, pick: impl Fn(usize, usize) -> bool) -> (Array2<u32>, u32) {
    let mut lab = Array2::<u32>::zeros((h, w));
    let mut next = 0;
    let mut stack = Vec::new();
    for r in 0..h {
        for c in 0..w {
            if lab[[r, c]] != 0 || !pick(r, c) {
                continue;
            }
            next += 1;
            lab[[r, c]] = next;
            stack.push((r, c));
            while let Some((y, x)) = stack.pop() {
                for (dy, dx) in N8 {
                    let (ny, nx) = (y as i64 + dy, x as i64 + dx);
                    if ny < 0 || nx < 0 || ny >= h as i64 || nx >= w as i64 {
                        continue;
                    }
                    let (ny, nx) = (ny as usize, nx as usize);
                    if lab[[ny, nx]] == 0 && pick(ny, nx) {
                        lab[[ny, nx]] = next;
                        stack.push((ny, nx));
                    }
                }
            }
        }
    }
    (lab, next)
}

/// Moore-neighbour trace of the outer contour of the cells `inside` selects,
/// from `start`, their first cell in row-major order (so the cell to its
/// west is outside). Clockwise in (row down, column right) terms; a closed
/// loop without its first cell repeated at the end.
fn trace(start: (i64, i64), inside: impl Fn(i64, i64) -> bool, limit: usize) -> Option<Vec<(i64, i64)>> {
    let mut out = vec![start];
    let (mut cur, mut back) = (start, 0usize);
    let mut first_move = None;
    loop {
        let mut found = None;
        for k in 1..=8 {
            let d = (back + k) % 8;
            if inside(cur.0 + N8[d].0, cur.1 + N8[d].1) {
                found = Some(d);
                break;
            }
        }
        let Some(d) = found else { return Some(out) }; // a single cell
        let next = (cur.0 + N8[d].0, cur.1 + N8[d].1);
        if cur == start {
            match first_move {
                None => first_move = Some(next),
                Some(m) if m == next => {
                    out.pop();
                    return Some(out);
                }
                _ => {}
            }
        }
        // the cell scanned just before `next` is outside: the new backtrack
        let prev = (cur.0 + N8[(d + 7) % 8].0, cur.1 + N8[(d + 7) % 8].1);
        back = N8.iter().position(|&n| (next.0 + n.0, next.1 + n.1) == prev)?;
        cur = next;
        out.push(cur);
        if out.len() > limit {
            return None;
        }
    }
}

/// The headland of `safe`: its rings and the region inside them.
///
/// A hole (unsafe cells enclosed by safe ones) is large, and gets a ring and
/// its share of the headland, when its bounding box is at least
/// `min_hole_cells` long or wide; a smaller one is a speck the lanes simply
/// stop at, as without a headland.
///
/// `None` when the safe region is empty, or pinched so that a ring would have
/// to pass diagonally between two unsafe cells.
pub fn headland(safe: ArrayView2<bool>, width_cells: f64, min_hole_cells: usize) -> Option<Headland> {
    let (h, w) = (safe.nrows(), safe.ncols());
    let at = |r: i64, c: i64| r >= 0 && c >= 0 && r < h as i64 && c < w as i64 && safe[[r as usize, c as usize]];
    // the rings' region: the safe cells with no unsafe neighbour
    let ringed = Array2::from_shape_fn((h, w), |(r, c)| safe[[r, c]] && N8.iter().all(|&(dy, dx)| at(r as i64 + dy, c as i64 + dx)));
    let (rings, blocked) = rings_and_reach(ringed.view(), (width_cells - 1.0).max(0.0), min_hole_cells + 2)?;
    let inner = Array2::from_shape_fn((h, w), |(r, c)| safe[[r, c]] && !blocked[[r, c]]);
    Some(Headland { rings, inner })
}

/// The rings of `safe` along its very edge, and the cells within
/// `width_cells` of the outside or of a large hole (those included).
fn rings_and_reach(safe: ArrayView2<bool>, width_cells: f64, min_hole_cells: usize) -> Option<(Vec<Vec<(usize, usize)>>, Array2<bool>)> {
    let (h, w) = (safe.nrows(), safe.ncols());
    // one unsafe cell all round: the outside is one component, at (0, 0)
    let (ph, pw) = (h + 2, w + 2);
    let ok = |r: usize, c: usize| r >= 1 && c >= 1 && r <= h && c <= w && safe[[r - 1, c - 1]];
    let oki = |r: i64, c: i64| r >= 0 && c >= 0 && ok(r as usize, c as usize);
    let (ulab, n_unsafe) = label(ph, pw, |r, c| !ok(r, c));
    let outside = ulab[[0, 0]];
    // bounding boxes of the holes
    let mut bbox = vec![(usize::MAX, usize::MAX, 0usize, 0usize); n_unsafe as usize + 1];
    for r in 0..ph {
        for c in 0..pw {
            let b = &mut bbox[ulab[[r, c]] as usize];
            *b = (b.0.min(r), b.1.min(c), b.2.max(r), b.3.max(c));
        }
    }
    let large: Vec<bool> = (0..=n_unsafe as usize)
        .map(|id| id != 0 && (id as u32 == outside || (bbox[id].2 - bbox[id].0 + 1).max(bbox[id].3 - bbox[id].1 + 1) >= min_hole_cells.max(1)))
        .collect();
    let blocks = |r: usize, c: usize| large[ulab[[r, c]] as usize];

    // inner region: stamp a disc on every blocking cell next to a safe one
    let reach = width_cells.max(0.0);
    let span = reach.floor() as i64;
    let mut blocked = Array2::from_shape_fn((h, w), |(r, c)| blocks(r + 1, c + 1));
    for r in 0..ph {
        for c in 0..pw {
            if !blocks(r, c) || !N8.iter().any(|&(dy, dx)| oki(r as i64 + dy, c as i64 + dx)) {
                continue;
            }
            for dy in -span..=span {
                for dx in -span..=span {
                    if ((dy * dy + dx * dx) as f64).sqrt() > reach {
                        continue;
                    }
                    let (y, x) = (r as i64 + dy - 1, c as i64 + dx - 1);
                    if y >= 0 && x >= 0 && y < h as i64 && x < w as i64 {
                        blocked[[y as usize, x as usize]] = true;
                    }
                }
            }
        }
    }

    // rings: the outer contour of every safe component, then of every large
    // hole together with the safe cells that touch it
    let limit = 4 * ph * pw + 8;
    let mut loops: Vec<Vec<(i64, i64)>> = Vec::new();
    let (slab, n_safe) = label(ph, pw, ok);
    let mut starts = vec![None; n_safe as usize + 1];
    for r in 0..ph {
        for c in 0..pw {
            let s = &mut starts[slab[[r, c]] as usize];
            if s.is_none() {
                *s = Some((r as i64, c as i64));
            }
        }
    }
    for id in 1..=n_safe as usize {
        let inside = |r: i64, c: i64| r >= 0 && c >= 0 && r < ph as i64 && c < pw as i64 && slab[[r as usize, c as usize]] == id as u32;
        loops.push(trace(starts[id]?, inside, limit)?);
    }
    for id in 1..=n_unsafe as usize {
        if !large[id] || id as u32 == outside {
            continue;
        }
        let hole = |r: i64, c: i64| r >= 0 && c >= 0 && r < ph as i64 && c < pw as i64 && ulab[[r as usize, c as usize]] == id as u32;
        let inside = |r: i64, c: i64| hole(r, c) || oki(r, c) && N8.iter().any(|&(dy, dx)| hole(r + dy, c + dx));
        let (r0, c0, r1, c1) = bbox[id];
        let start = (r0 as i64 - 1..=r1 as i64 + 1).flat_map(|r| (c0 as i64 - 1..=c1 as i64 + 1).map(move |c| (r, c))).find(|&(r, c)| inside(r, c))?;
        loops.push(trace(start, inside, limit)?);
    }
    if loops.is_empty() {
        return None;
    }

    // every cell safe, and diagonal steps taken round the corner
    let mut rings = Vec::with_capacity(loops.len());
    for lp in loops {
        if lp.iter().any(|&(r, c)| !oki(r, c)) {
            return None;
        }
        let mut ring: Vec<(usize, usize)> = Vec::with_capacity(lp.len() * 2);
        for (i, &(r, c)) in lp.iter().enumerate() {
            ring.push((r as usize - 1, c as usize - 1));
            let (nr, nc) = lp[(i + 1) % lp.len()];
            if nr != r && nc != c {
                let via = if oki(r, nc) { (r, nc) } else if oki(nr, c) { (nr, c) } else { return None };
                ring.push((via.0 as usize - 1, via.1 as usize - 1));
            }
        }
        if ring.len() >= 4 {
            rings.push(ring);
        }
    }
    if rings.is_empty() {
        return None;
    }
    Some((rings, blocked))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn steps_ok(ring: &[(usize, usize)]) -> bool {
        (0..ring.len()).all(|i| {
            let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
            a.0.abs_diff(b.0) + a.1.abs_diff(b.1) == 1
        })
    }

    #[test]
    fn a_rectangle_has_one_ring_along_its_edge() {
        let g = Array2::from_shape_fn((12, 20), |(r, c)| (1..11).contains(&r) && (2..18).contains(&c));
        let hl = headland(g.view(), 2.0, 3).unwrap();
        assert_eq!(hl.rings.len(), 1);
        let ring = &hl.rings[0];
        // one cell inside the edge
        assert_eq!(ring.len(), 2 * (8 + 14) - 4);
        assert!(steps_ok(ring));
        assert!(ring.iter().all(|&(r, c)| r == 2 || r == 9 || c == 3 || c == 16));
        // two cells in from every side
        for r in 0..12 {
            for c in 0..20 {
                assert_eq!(hl.inner[[r, c]], (3..9).contains(&r) && (4..16).contains(&c), "{r} {c}");
            }
        }
    }

    #[test]
    fn a_large_hole_gets_a_ring_and_a_speck_does_not() {
        let mut g = Array2::from_elem((30, 30), true);
        for r in 10..16 {
            for c in 12..18 {
                g[[r, c]] = false;
            }
        }
        g[[24, 5]] = false;
        let hl = headland(g.view(), 2.0, 4).unwrap();
        assert_eq!(hl.rings.len(), 2);
        assert!(hl.rings.iter().all(|r| steps_ok(r)));
        assert!(hl.rings.iter().flatten().all(|&(r, c)| g[[r, c]]));
        // round the hole, one cell clear of it
        assert!(hl.rings[1].iter().all(|&(r, c)| (8..=17).contains(&r) && (10..=19).contains(&c)));
        assert_eq!(hl.rings[1].len(), 2 * (10 + 10) - 4);
        // headland round the hole, none round the speck
        assert!(!hl.inner[[8, 14]] && hl.inner[[7, 14]]);
        assert!(hl.inner[[24, 6]] && hl.inner[[23, 5]] && !hl.inner[[24, 5]]);
    }

    #[test]
    fn empty_regions_have_none_and_corner_touching_ones_a_ring_each() {
        assert!(headland(Array2::from_elem((5, 5), false).view(), 1.0, 2).is_none());
        // too thin to have a cell with no unsafe neighbour
        assert!(headland(Array2::from_shape_fn((6, 6), |(r, _)| r == 2 || r == 3).view(), 1.0, 2).is_none());
        // two squares that touch only at a corner: the rings do not pass there
        let g = Array2::from_shape_fn((12, 12), |(r, c)| (r < 6) == (c < 6));
        let hl = headland(g.view(), 1.0, 2).unwrap();
        assert_eq!(hl.rings.len(), 2);
        assert!(hl.rings.iter().all(|r| steps_ok(r)));
    }
}
