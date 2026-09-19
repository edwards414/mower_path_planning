//! `cv2.findContours(mask, RETR_EXTERNAL, CHAIN_APPROX_NONE)` and
//! `cv2.contourArea` as OpenCV 4.6.0 computes them (`modules/imgproc/src/
//! contours.cpp`: `cvFindNextContour` + `icvFetchContour`, the Suzuki
//! border following), for the coverage node's boundary ring. Every border
//! point in the same order OpenCV emits it.

const NBD: i8 = 2;
const MARK: i8 = NBD | -128; // (schar)(nbd | -128) == -126

/// Chain-code deltas: 0=E, 1=NE, 2=N, 3=NW, 4=W, 5=SW, 6=S, 7=SE (y down).
const CODE_DELTAS: [(i32, i32); 8] = [(1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1)];

/// External contours of a binary mask (any non-zero = foreground), each as
/// (x, y) pixel coordinates of the unpadded image, in OpenCV's order.
pub fn find_external_contours(mask: &[u8], width: usize, height: usize) -> Vec<Vec<(i32, i32)>> {
    if width == 0 || height == 0 {
        return Vec::new();
    }
    // copyMakeBorder(1, 1, 1, 1, BORDER_CONSTANT 0) then cvThreshold(0, 1)
    let pw = width + 2;
    let ph = height + 2;
    let mut img = vec![0i8; pw * ph];
    for y in 0..height {
        for x in 0..width {
            img[(y + 1) * pw + (x + 1)] = (mask[y * width + x] != 0) as i8;
        }
    }
    let step = pw as i64;
    // scanner: exclude the rightmost column and the bottom row
    let scan_w = pw as i32 - 1;
    let scan_h = ph as i32 - 1;
    let deltas: [i64; 8] = [1, -step + 1, -step, -step - 1, -1, step - 1, step, step + 1];

    let mut contours = Vec::new();
    let (mut x, mut y) = (1i32, 1i32);
    let (mut lnbd_x, mut lnbd_y) = (0i32, 1i32);
    let mut prev: i8 = img[(y * pw as i32 + x - 1) as usize];
    while y < scan_h {
        let row = (y * pw as i32) as usize;
        while x < scan_w {
            let mut p = img[row + x as usize];
            while x < scan_w && {
                p = img[row + x as usize];
                p == prev
            } {
                x += 1;
            }
            if x >= scan_w {
                break;
            }
            let mut resume = false;
            if !(prev == 0 && p == 1) {
                // a hole start or the trailing edge of a border
                resume = true; // mode == RETR_EXTERNAL skips holes either way
            } else if img[(lnbd_y * pw as i32 + lnbd_x) as usize] > 0 {
                // inside an already traced external contour
                resume = true;
            }
            if !resume {
                lnbd_x = x;
                let start = row + x as usize;
                let pts = fetch_contour(&mut img, &deltas, start, (x - 1, y - 1));
                contours.push(pts);
                // scanner->pt.x = x + 1; the next scan reads prev = img[x]
                x += 1;
                prev = img[row + (x - 1) as usize];
                continue;
            }
            prev = p;
            if prev & -2 != 0 {
                lnbd_x = x;
            }
        }
        lnbd_x = 0;
        lnbd_y = y + 1;
        x = 1;
        prev = 0;
        y += 1;
    }
    // the tree is emitted newest first (icvEndProcessContour links each new
    // contour at the head of the frame's child list)
    contours.reverse();
    contours
}

/// `icvFetchContour` with method CV_CHAIN_APPROX_NONE (every border pixel).
fn fetch_contour(img: &mut [i8], deltas: &[i64; 8], i0: usize, origin: (i32, i32)) -> Vec<(i32, i32)> {
    let at = |base: usize, s: usize| (base as i64 + deltas[s & 7]) as usize;
    let mut pt = origin;
    let mut out = Vec::new();
    let s_end0 = 4usize;
    let mut s = s_end0;
    let mut i1;
    loop {
        s = (s.wrapping_sub(1)) & 7;
        i1 = at(i0, s);
        if img[i1] != 0 || s == s_end0 {
            break;
        }
    }
    if s == s_end0 {
        // single pixel domain
        img[i0] = MARK;
        out.push(pt);
        return out;
    }
    let mut i3 = i0;
    loop {
        let s_end = s;
        // s = min(s, MAX_SIZE - 1); while (s < MAX_SIZE - 1) { i4 = i3 + deltas[++s]; if *i4 != 0 break; }
        let mut k = s;
        let mut i4;
        loop {
            k += 1;
            i4 = at(i3, k);
            if img[i4] != 0 {
                break;
            }
            if k >= 15 {
                break;
            }
        }
        s = k & 7;
        // check "right" bound
        if s != 0 && (s - 1) < s_end {
            img[i3] = MARK;
        } else if img[i3] == 1 {
            img[i3] = NBD;
        }
        out.push(pt);
        pt.0 += CODE_DELTAS[s].0;
        pt.1 += CODE_DELTAS[s].1;
        if i4 == i0 && i3 == i1 {
            break;
        }
        i3 = i4;
        s = (s + 4) & 7;
    }
    out
}

/// `cv2.contourArea(contour)` (non-oriented): |Green's formula| / 2.
pub fn contour_area(contour: &[(i32, i32)]) -> f64 {
    if contour.is_empty() {
        return 0.0;
    }
    let mut a00 = 0.0f64;
    let mut prev = contour[contour.len() - 1];
    for &p in contour {
        a00 += prev.0 as f64 * p.1 as f64 - prev.1 as f64 * p.0 as f64;
        prev = p;
    }
    (a00 * 0.5).abs()
}

/// `_outer_boundary_ring`: the largest external contour of the safe region as
/// a closed ring of world points (cell centres), or empty.
pub fn outer_boundary_ring(safe: &[u8], width: usize, height: usize, res: f64, origin_x: f64, origin_y: f64) -> Vec<(f64, f64)> {
    let contours = find_external_contours(safe, width, height);
    if contours.is_empty() {
        return Vec::new();
    }
    // Python's max() keeps the first of equal maxima
    let mut best = 0usize;
    let mut best_area = contour_area(&contours[0]);
    for (i, c) in contours.iter().enumerate().skip(1) {
        let a = contour_area(c);
        if a > best_area {
            best_area = a;
            best = i;
        }
    }
    let mut pts: Vec<(f64, f64)> = contours[best].iter().map(|&(col, row)| (origin_x + (col as f64 + 0.5) * res, origin_y + (row as f64 + 0.5) * res)).collect();
    if pts.len() >= 2 {
        pts.push(pts[0]);
    }
    pts
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    /// `tests/contours_oracle.json` (contours_oracle.py): OpenCV 4.6.0's
    /// external contours, areas and the node's ring on random masks.
    #[test]
    fn matches_opencv_contours() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/contours_oracle.json");
        let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
        let cases: Vec<Value> = serde_json::from_str(&text).unwrap();
        assert!(cases.len() >= 100);
        let mut failures = Vec::new();
        for (n, case) in cases.iter().enumerate() {
            let (w, h) = (case["w"].as_u64().unwrap() as usize, case["h"].as_u64().unwrap() as usize);
            let src: Vec<u8> = case["src"].as_array().unwrap().iter().map(|v| v.as_u64().unwrap() as u8).collect();
            let want: Vec<Vec<(i32, i32)>> = case["contours"].as_array().unwrap().iter().map(|c| c.as_array().unwrap().iter().map(|p| (p[0].as_i64().unwrap() as i32, p[1].as_i64().unwrap() as i32)).collect()).collect();
            let got = find_external_contours(&src, w, h);
            if got != want {
                failures.push(format!("case {n} ({w}x{h}): contours differ: got {} want {}; first got {:?} want {:?}", got.len(), want.len(), got.first().map(|c| &c[..c.len().min(8)]), want.first().map(|c| &c[..c.len().min(8)])));
                continue;
            }
            let areas: Vec<f64> = case["areas"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
            for (c, a) in got.iter().zip(&areas) {
                if (contour_area(c) - a).abs() > 1e-9 {
                    failures.push(format!("case {n}: area {} vs {a}", contour_area(c)));
                }
            }
            let ring = outer_boundary_ring(&src, w, h, case["res"].as_f64().unwrap(), case["ox"].as_f64().unwrap(), case["oy"].as_f64().unwrap());
            let want_ring: Vec<(f64, f64)> = case["ring"].as_array().unwrap().iter().map(|p| (p[0].as_f64().unwrap(), p[1].as_f64().unwrap())).collect();
            if ring.len() != want_ring.len() || ring.iter().zip(&want_ring).any(|(a, b)| (a.0 - b.0).abs() > 1e-9 || (a.1 - b.1).abs() > 1e-9) {
                failures.push(format!("case {n}: ring differs ({} vs {} pts)", ring.len(), want_ring.len()));
            }
        }
        assert!(failures.is_empty(), "{} failures:\n{}", failures.len(), failures.join("\n"));
    }
}
