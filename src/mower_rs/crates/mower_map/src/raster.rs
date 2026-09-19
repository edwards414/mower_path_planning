//! Pixel-exact ports of the OpenCV 4.6.0 drawing and morphology primitives
//! `map_manage_node.py` uses on `uint8` grids (`cv2.fillPoly`, `cv2.polylines`
//! with thickness 1, `cv2.line` with a thickness, `cv2.getStructuringElement(
//! MORPH_ELLIPSE)`, `cv2.erode`, `cv2.dilate`), so the Rust node produces the
//! same occupancy bytes as the Python node. The fixed-point arithmetic,
//! rounding and clipping follow `modules/imgproc/src/drawing.cpp` and
//! `morph.dispatch.cpp` of that release; `tests/raster_oracle.json` holds
//! 720 random cases rendered by that exact OpenCV build.

const XY_SHIFT: i32 = 16;
const XY_ONE: i64 = 1 << XY_SHIFT;

/// Single-channel 8-bit image, row-major.
#[derive(Clone, Debug, PartialEq)]
pub struct Img {
    pub w: i32,
    pub h: i32,
    pub data: Vec<u8>,
}

impl Img {
    pub fn zeros(w: i32, h: i32) -> Img {
        Img { w, h, data: vec![0; (w.max(0) as usize) * (h.max(0) as usize)] }
    }

    #[inline]
    fn put(&mut self, x: i32, y: i32, c: u8) {
        if x >= 0 && x < self.w && y >= 0 && y < self.h {
            self.data[y as usize * self.w as usize + x as usize] = c;
        }
    }

    /// ICV_HLINE: inclusive span on one row, caller guarantees the bounds.
    #[inline]
    fn hline(&mut self, y: i32, xl: i32, xr: i32, c: u8) {
        if y < 0 || y >= self.h || xl > xr {
            return;
        }
        let row = y as usize * self.w as usize;
        for x in xl.max(0)..=xr.min(self.w - 1) {
            self.data[row + x as usize] = c;
        }
    }
}

/// cvRound: round half to even (lrint with the default rounding mode).
#[inline]
fn cv_round(x: f64) -> i64 {
    x.round_ties_even() as i64
}

// ----------------------------------------------------------------- clipLine

/// `clipLine(Size2l, Point2l&, Point2l&)`.
fn clip_line(width: i64, height: i64, p1: &mut (i64, i64), p2: &mut (i64, i64)) -> bool {
    let right = width - 1;
    let bottom = height - 1;
    if width <= 0 || height <= 0 {
        return false;
    }
    let (mut x1, mut y1) = *p1;
    let (mut x2, mut y2) = *p2;
    let code = |x: i64, y: i64| (x < 0) as i32 + (x > right) as i32 * 2 + (y < 0) as i32 * 4 + (y > bottom) as i32 * 8;
    let mut c1 = code(x1, y1);
    let mut c2 = code(x2, y2);
    if (c1 & c2) == 0 && (c1 | c2) != 0 {
        if c1 & 12 != 0 {
            let a = if c1 < 8 { 0 } else { bottom };
            x1 += ((a - y1) as f64 * (x2 - x1) as f64 / (y2 - y1) as f64) as i64;
            y1 = a;
            c1 = (x1 < 0) as i32 + (x1 > right) as i32 * 2;
        }
        if c2 & 12 != 0 {
            let a = if c2 < 8 { 0 } else { bottom };
            x2 += ((a - y2) as f64 * (x2 - x1) as f64 / (y2 - y1) as f64) as i64;
            y2 = a;
            c2 = (x2 < 0) as i32 + (x2 > right) as i32 * 2;
        }
        if (c1 & c2) == 0 && (c1 | c2) != 0 {
            if c1 != 0 {
                let a = if c1 == 1 { 0 } else { right };
                y1 += ((a - x1) as f64 * (y2 - y1) as f64 / (x2 - x1) as f64) as i64;
                x1 = a;
                c1 = 0;
            }
            if c2 != 0 {
                let a = if c2 == 1 { 0 } else { right };
                y2 += ((a - x2) as f64 * (y2 - y1) as f64 / (x2 - x1) as f64) as i64;
                x2 = a;
                c2 = 0;
            }
        }
    }
    *p1 = (x1, y1);
    *p2 = (x2, y2);
    (c1 | c2) == 0
}

// --------------------------------------------------------------------- Line

/// `Line(img, pt1, pt2, color, 8)`: the 8-connected Bresenham line of
/// `LineIterator(img, pt1, pt2, 8, leftToRight=true)`.
fn line8(img: &mut Img, pt1: (i32, i32), pt2: (i32, i32), color: u8) {
    let (mut p1, mut p2) = ((pt1.0 as i64, pt1.1 as i64), (pt2.0 as i64, pt2.1 as i64));
    let outside = |p: (i64, i64)| (p.0 as u32) >= (img.w as u32) || (p.1 as u32) >= (img.h as u32);
    if outside(p1) || outside(p2) {
        if !clip_line(img.w as i64, img.h as i64, &mut p1, &mut p2) {
            return;
        }
    }
    let (mut pt1, mut pt2) = ((p1.0 as i32, p1.1 as i32), (p2.0 as i32, p2.1 as i32));
    let (mut delta_x, mut delta_y) = (1i32, 1i32);
    let mut dx = pt2.0 - pt1.0;
    let mut dy = pt2.1 - pt1.1;
    if dx < 0 {
        // leftToRight
        dx = -dx;
        dy = -dy;
        pt1 = pt2;
    }
    let _ = &mut pt2;
    if dy < 0 {
        dy = -dy;
        delta_y = -1;
    }
    let vert = dy > dx;
    if vert {
        std::mem::swap(&mut dx, &mut dy);
        std::mem::swap(&mut delta_x, &mut delta_y);
    }
    let mut err = dx - (dy + dy);
    let plus_delta = dx + dx;
    let minus_delta = -(dy + dy);
    let mut minus_shift = delta_x;
    let mut plus_shift = 0;
    let mut minus_step = 0;
    let mut plus_step = delta_y;
    let count = dx + 1;
    if vert {
        std::mem::swap(&mut plus_step, &mut plus_shift);
        std::mem::swap(&mut minus_step, &mut minus_shift);
    }
    let (mut x, mut y) = pt1;
    for _ in 0..count {
        img.put(x, y, color);
        let mask = if err < 0 { -1 } else { 0 };
        err += minus_delta + (plus_delta & mask);
        x += minus_shift + (plus_shift & mask);
        y += minus_step + (plus_step & mask);
    }
}

/// `Line2`: fixed-point (XY_SHIFT) line for FillConvexPoly edges with shift != 0.
fn line2(img: &mut Img, mut pt1: (i64, i64), mut pt2: (i64, i64), color: u8) {
    let size_w = (img.w as i64) << XY_SHIFT;
    let size_h = (img.h as i64) << XY_SHIFT;
    if !clip_line(size_w, size_h, &mut pt1, &mut pt2) {
        return;
    }
    let mut dx = pt2.0 - pt1.0;
    let mut dy = pt2.1 - pt1.1;
    let j: i64 = if dx < 0 { -1 } else { 0 };
    let ax = (dx ^ j) - j;
    let i: i64 = if dy < 0 { -1 } else { 0 };
    let ay = (dy ^ i) - i;
    let x_step: i64;
    let y_step: i64;
    let mut ecount: i64;
    if ax > ay {
        dy = (dy ^ j) - j;
        if j != 0 {
            std::mem::swap(&mut pt1, &mut pt2);
        }
        x_step = XY_ONE;
        y_step = dy * (1 << XY_SHIFT) / (ax | 1);
        ecount = (pt2.0 - pt1.0) >> XY_SHIFT;
    } else {
        dx = (dx ^ i) - i;
        if i != 0 {
            std::mem::swap(&mut pt1, &mut pt2);
        }
        x_step = dx * (1 << XY_SHIFT) / (ay | 1);
        y_step = XY_ONE;
        ecount = (pt2.1 - pt1.1) >> XY_SHIFT;
    }
    pt1.0 += XY_ONE >> 1;
    pt1.1 += XY_ONE >> 1;
    img.put(((pt2.0 + (XY_ONE >> 1)) >> XY_SHIFT) as i32, ((pt2.1 + (XY_ONE >> 1)) >> XY_SHIFT) as i32, color);
    if ax > ay {
        pt1.0 >>= XY_SHIFT;
        while ecount >= 0 {
            img.put(pt1.0 as i32, (pt1.1 >> XY_SHIFT) as i32, color);
            pt1.0 += 1;
            pt1.1 += y_step;
            ecount -= 1;
        }
    } else {
        pt1.1 >>= XY_SHIFT;
        while ecount >= 0 {
            img.put((pt1.0 >> XY_SHIFT) as i32, pt1.1 as i32, color);
            pt1.0 += x_step;
            pt1.1 += 1;
            ecount -= 1;
        }
    }
}

// ------------------------------------------------------------ FillConvexPoly

/// `FillConvexPoly(img, v, npts, color, LINE_8, shift)`.
fn fill_convex_poly(img: &mut Img, v: &[(i64, i64)], color: u8, shift: i32) {
    #[derive(Clone, Copy)]
    struct Edge {
        idx: i32,
        di: i32,
        x: i64,
        dx: i64,
        ye: i32,
    }
    let npts = v.len() as i32;
    let delta: i64 = (1i64 << shift) >> 1;
    let delta1 = XY_ONE >> 1;
    let delta2 = XY_ONE >> 1;
    let mut edges = npts;
    let mut p0 = v[(npts - 1) as usize];
    p0.0 <<= XY_SHIFT - shift;
    p0.1 <<= XY_SHIFT - shift;
    let (mut xmin, mut xmax) = (v[0].0, v[0].0);
    let (mut ymin, mut ymax) = (v[0].1, v[0].1);
    let mut imin = 0i32;
    for i in 0..npts {
        let mut p = v[i as usize];
        if p.1 < ymin {
            ymin = p.1;
            imin = i;
        }
        ymax = ymax.max(p.1);
        xmax = xmax.max(p.0);
        xmin = xmin.min(p.0);
        p.0 <<= XY_SHIFT - shift;
        p.1 <<= XY_SHIFT - shift;
        if shift == 0 {
            line8(img, ((p0.0 >> XY_SHIFT) as i32, (p0.1 >> XY_SHIFT) as i32), ((p.0 >> XY_SHIFT) as i32, (p.1 >> XY_SHIFT) as i32), color);
        } else {
            line2(img, p0, p, color);
        }
        p0 = p;
    }
    xmin = (xmin + delta) >> shift;
    xmax = (xmax + delta) >> shift;
    ymin = (ymin + delta) >> shift;
    ymax = (ymax + delta) >> shift;
    if npts < 3 || (xmax as i32) < 0 || (ymax as i32) < 0 || (xmin as i32) >= img.w || (ymin as i32) >= img.h {
        return;
    }
    let ymax = (ymax as i32).min(img.h - 1);
    let mut y = ymin as i32;
    let mut edge = [Edge { idx: imin, di: 1, x: -XY_ONE, dx: 0, ye: y }, Edge { idx: imin, di: npts - 1, x: -XY_ONE, dx: 0, ye: y }];
    loop {
        for e in edge.iter_mut() {
            if y >= e.ye {
                let mut idx0 = e.idx;
                let di = e.di;
                let mut idx = idx0 + di;
                if idx >= npts {
                    idx -= npts;
                }
                loop {
                    let old = edges;
                    edges -= 1;
                    if old <= 0 {
                        break;
                    }
                    let ty = ((v[idx as usize].1 + delta) >> shift) as i32;
                    if ty > y {
                        let mut xs = v[idx0 as usize].0;
                        let mut xe = v[idx as usize].0;
                        if shift != XY_SHIFT {
                            xs <<= XY_SHIFT - shift;
                            xe <<= XY_SHIFT - shift;
                        }
                        e.ye = ty;
                        e.dx = ((xe - xs) * 2 + (ty - y) as i64) / (2 * (ty - y) as i64);
                        e.x = xs;
                        e.idx = idx;
                        break;
                    }
                    idx0 = idx;
                    idx += di;
                    if idx >= npts {
                        idx -= npts;
                    }
                }
            }
        }
        if edges < 0 {
            break;
        }
        if y >= 0 {
            let (left, right) = if edge[0].x > edge[1].x { (1, 0) } else { (0, 1) };
            let mut xx1 = ((edge[left].x + delta1) >> XY_SHIFT) as i32;
            let mut xx2 = ((edge[right].x + delta2) >> XY_SHIFT) as i32;
            if xx2 >= 0 && xx1 < img.w {
                if xx1 < 0 {
                    xx1 = 0;
                }
                if xx2 >= img.w {
                    xx2 = img.w - 1;
                }
                img.hline(y, xx1, xx2, color);
            }
        }
        edge[0].x += edge[0].dx;
        edge[1].x += edge[1].dx;
        y += 1;
        if y > ymax {
            break;
        }
    }
}

// --------------------------------------------------------- arbitrary polygon

#[derive(Clone, Copy, Debug)]
struct PolyEdge {
    y0: i32,
    y1: i32,
    x: i64,
    dx: i64,
    next: i32, // index into the edge vector, -1 = null
}

const NULL: i32 = -1;

/// `CollectPolyEdges(img, v, count, edges, color, LINE_8, shift=0, offset=0)`.
fn collect_poly_edges(img: &mut Img, v: &[(i64, i64)], edges: &mut Vec<PolyEdge>, color: u8) {
    let shift = 0;
    let delta: i64 = (1i64 << shift) >> 1;
    let count = v.len();
    let mut pt0 = v[count - 1];
    pt0.0 <<= XY_SHIFT - shift;
    pt0.1 = (pt0.1 + delta) >> shift;
    edges.reserve(count);
    for i in 0..count {
        let mut pt1 = v[i];
        pt1.0 <<= XY_SHIFT - shift;
        pt1.1 = (pt1.1 + delta) >> shift;
        let t0 = (((pt0.0 + (XY_ONE >> 1)) >> XY_SHIFT) as i32, pt0.1 as i32);
        let t1 = (((pt1.0 + (XY_ONE >> 1)) >> XY_SHIFT) as i32, pt1.1 as i32);
        line8(img, t0, t1, color);
        if pt0.1 != pt1.1 {
            let (y0, y1, x) = if pt0.1 < pt1.1 { (pt0.1 as i32, pt1.1 as i32, pt0.0) } else { (pt1.1 as i32, pt0.1 as i32, pt1.0) };
            let dx = (pt1.0 - pt0.0) / (pt1.1 - pt0.1);
            edges.push(PolyEdge { y0, y1, x, dx, next: NULL });
        }
        pt0 = pt1;
    }
}

/// `FillEdgeCollection(img, edges, color)` with the active-edge linked list
/// kept as indices into `edges` (`tmp` is the sentinel appended at the end).
fn fill_edge_collection(img: &mut Img, edges: &mut Vec<PolyEdge>, color: u8) {
    let total = edges.len();
    if total < 2 {
        return;
    }
    let (mut y_min, mut y_max) = (i32::MAX, i32::MIN);
    let (mut x_min, mut x_max) = (i64::MAX, -1i64);
    for e1 in edges.iter() {
        debug_assert!(e1.y0 < e1.y1);
        let x1 = e1.x + (e1.y1 - e1.y0) as i64 * e1.dx;
        y_min = y_min.min(e1.y0);
        y_max = y_max.max(e1.y1);
        x_min = x_min.min(e1.x).min(x1);
        x_max = x_max.max(e1.x).max(x1);
    }
    if y_max < 0 || y_min >= img.h || x_max < 0 || x_min >= ((img.w as i64) << XY_SHIFT) {
        return;
    }
    edges.sort_by(|a, b| a.y0.cmp(&b.y0).then(a.x.cmp(&b.x)).then(a.dx.cmp(&b.dx)));
    let tmp = total as i32;
    edges.push(PolyEdge { y0: i32::MAX, y1: 0, x: 0, dx: 0, next: NULL });
    let mut i = 0usize;
    let mut e = 0i32;
    let y_max = y_max.min(img.h);
    let mut y = edges[0].y0;
    while y < y_max {
        let mut draw = false;
        let clipline = y < 0;
        let mut prelast = tmp;
        let mut last = edges[tmp as usize].next;
        loop {
            if !(last != NULL || edges[e as usize].y0 == y) {
                break;
            }
            if last != NULL && edges[last as usize].y1 == y {
                // exclude edge if y reaches its lower point
                edges[prelast as usize].next = edges[last as usize].next;
                last = edges[last as usize].next;
                continue;
            }
            let keep_prelast = prelast;
            if last != NULL && (edges[e as usize].y0 > y || edges[last as usize].x < edges[e as usize].x) {
                // go to the next edge in active list
                prelast = last;
                last = edges[last as usize].next;
            } else if i < total {
                // insert new edge into active list if y reaches its upper point
                edges[prelast as usize].next = e;
                edges[e as usize].next = last;
                prelast = e;
                i += 1;
                e = i as i32;
            } else {
                break;
            }
            if draw {
                if !clipline {
                    let (kx, px) = (edges[keep_prelast as usize].x, edges[prelast as usize].x);
                    let (x1, x2) = if kx > px {
                        (((px + XY_ONE - 1) >> XY_SHIFT) as i32, (kx >> XY_SHIFT) as i32)
                    } else {
                        (((kx + XY_ONE - 1) >> XY_SHIFT) as i32, (px >> XY_SHIFT) as i32)
                    };
                    if x1 < img.w && x2 >= 0 {
                        img.hline(y, x1.max(0), x2.min(img.w - 1), color);
                    }
                }
                let kdx = edges[keep_prelast as usize].dx;
                edges[keep_prelast as usize].x += kdx;
                let pdx = edges[prelast as usize].dx;
                edges[prelast as usize].x += pdx;
            }
            draw = !draw;
        }
        // sort edges (using bubble sort)
        let mut keep_prelast = NULL;
        loop {
            let mut prelast = tmp;
            let mut last = edges[tmp as usize].next;
            let mut last_exchange = NULL;
            while last != keep_prelast && last != NULL && edges[last as usize].next != NULL {
                let te = edges[last as usize].next;
                if edges[last as usize].x > edges[te as usize].x {
                    edges[prelast as usize].next = te;
                    edges[last as usize].next = edges[te as usize].next;
                    edges[te as usize].next = last;
                    prelast = te;
                    last_exchange = prelast;
                } else {
                    prelast = last;
                    last = te;
                }
            }
            if last_exchange == NULL {
                break;
            }
            keep_prelast = last_exchange;
            if !(keep_prelast != edges[tmp as usize].next && keep_prelast != tmp) {
                break;
            }
        }
        y += 1;
    }
}

// ------------------------------------------------------------------- Circle

/// `Circle(img, center, radius, color, fill=1)`.
fn circle_filled(img: &mut Img, center: (i32, i32), radius: i32, color: u8) {
    let (w, h) = (img.w, img.h);
    let (mut err, mut dx, mut dy, mut plus, mut minus) = (0i32, radius, 0i32, 1i32, (radius << 1) - 1);
    let inside = center.0 >= radius && center.0 < w - radius && center.1 >= radius && center.1 < h - radius;
    while dx >= dy {
        let (y11, y12, y21, y22) = (center.1 - dy, center.1 + dy, center.1 - dx, center.1 + dx);
        let (mut x11, mut x12, mut x21, mut x22) = (center.0 - dx, center.0 + dx, center.0 - dy, center.0 + dy);
        if inside {
            img.hline(y11, x11, x12, color);
            img.hline(y12, x11, x12, color);
            img.hline(y21, x21, x22, color);
            img.hline(y22, x21, x22, color);
        } else if x11 < w && x12 >= 0 && y21 < h && y22 >= 0 {
            x11 = x11.max(0);
            x12 = x12.min(w - 1);
            if (y11 as u32) < (h as u32) {
                img.hline(y11, x11, x12, color);
            }
            if (y12 as u32) < (h as u32) {
                img.hline(y12, x11, x12, color);
            }
            if x21 < w && x22 >= 0 {
                x21 = x21.max(0);
                x22 = x22.min(w - 1);
                if (y21 as u32) < (h as u32) {
                    img.hline(y21, x21, x22, color);
                }
                if (y22 as u32) < (h as u32) {
                    img.hline(y22, x21, x22, color);
                }
            }
        }
        dy += 1;
        err += plus;
        plus += 2;
        let mask = (err <= 0) as i32 - 1;
        err -= minus & mask;
        dx += mask;
        minus -= mask & 2;
    }
}

// ---------------------------------------------------------------- ThickLine

/// `ThickLine(img, p0, p1, color, thickness, LINE_8, flags, shift=0)`.
fn thick_line(img: &mut Img, p0: (i32, i32), p1: (i32, i32), color: u8, thickness: i32, flags: i32) {
    let mut p0 = ((p0.0 as i64) << XY_SHIFT, (p0.1 as i64) << XY_SHIFT);
    let p1 = ((p1.0 as i64) << XY_SHIFT, (p1.1 as i64) << XY_SHIFT);
    if thickness <= 1 {
        let r = |v: i64| ((v + (XY_ONE >> 1)) >> XY_SHIFT) as i32;
        line8(img, (r(p0.0), r(p0.1)), (r(p1.0), r(p1.1)), color);
        return;
    }
    let inv = 1.0 / XY_ONE as f64;
    let dx = (p0.0 - p1.0) as f64 * inv;
    let dy = (p1.1 - p0.1) as f64 * inv;
    let mut r = dx * dx + dy * dy;
    let odd = (thickness & 1) as i64;
    let thickness = (thickness as i64) << (XY_SHIFT - 1);
    if r.abs() > f64::EPSILON {
        r = (thickness as f64 + odd as f64 * XY_ONE as f64 * 0.5) / r.sqrt();
        let dpx = cv_round(dy * r);
        let dpy = cv_round(dx * r);
        let pt = [(p0.0 + dpx, p0.1 + dpy), (p0.0 - dpx, p0.1 - dpy), (p1.0 - dpx, p1.1 - dpy), (p1.0 + dpx, p1.1 + dpy)];
        fill_convex_poly(img, &pt, color, XY_SHIFT);
    }
    for i in 0..2 {
        if flags & (i + 1) != 0 {
            let center = (((p0.0 + (XY_ONE >> 1)) >> XY_SHIFT) as i32, ((p0.1 + (XY_ONE >> 1)) >> XY_SHIFT) as i32);
            circle_filled(img, center, ((thickness + (XY_ONE >> 1)) >> XY_SHIFT) as i32, color);
        }
        p0 = p1;
    }
}

// --------------------------------------------------------------- public API

/// `cv2.fillPoly(img, [pts], color)` for one contour (LINE_8, shift 0).
pub fn fill_poly(img: &mut Img, pts: &[(i32, i32)], color: u8) {
    if pts.is_empty() {
        return;
    }
    let v: Vec<(i64, i64)> = pts.iter().map(|&(x, y)| (x as i64, y as i64)).collect();
    let mut edges = Vec::with_capacity(v.len() + 1);
    collect_poly_edges(img, &v, &mut edges, color);
    fill_edge_collection(img, &mut edges, color);
}

/// `cv2.polylines(img, [pts], isClosed, color, thickness=1)`.
pub fn polylines(img: &mut Img, pts: &[(i32, i32)], closed: bool, color: u8, thickness: i32) {
    let count = pts.len();
    if count == 0 {
        return;
    }
    let mut i = if closed { count - 1 } else { 0 };
    let mut flags = 2 + (!closed) as i32;
    let mut p0 = pts[i];
    i = (!closed) as usize;
    while i < count {
        let p = pts[i];
        thick_line(img, p0, p, color, thickness, flags);
        p0 = p;
        flags = 2;
        i += 1;
    }
}

/// `cv2.line(img, p1, p2, color, thickness)` (LINE_8, round caps).
pub fn line(img: &mut Img, p1: (i32, i32), p2: (i32, i32), color: u8, thickness: i32) {
    thick_line(img, p1, p2, color, thickness, 3);
}

// --------------------------------------------------------------- morphology

/// `cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2r+1, 2r+1))` as the
/// (dy, dx) offsets of its set cells relative to the centre anchor.
pub fn ellipse_kernel(r: i32) -> Vec<(i32, i32)> {
    let k = 2 * r + 1;
    let (rr, c) = (k / 2, k / 2);
    let inv_r2 = if rr != 0 { 1.0 / (rr as f64 * rr as f64) } else { 0.0 };
    let mut out = Vec::new();
    for i in 0..k {
        let dy = i - rr;
        let (mut j1, mut j2) = (0, 0);
        if dy.abs() <= rr {
            let dx = cv_round(c as f64 * (((rr * rr - dy * dy) as f64) * inv_r2).sqrt()) as i32;
            j1 = (c - dx).max(0);
            j2 = (c + dx + 1).min(k);
        }
        for j in j1..j2 {
            out.push((i - rr, j - c));
        }
    }
    out
}

/// How pixels outside the image enter a min/max window.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Border {
    /// OpenCV's default `morphologyDefaultBorderValue()`: the outside does
    /// not take part (erode: +inf, dilate: -inf).
    Ignore,
    /// `BORDER_CONSTANT` with this value.
    Constant(u8),
}

fn morph(src: &Img, kernel: &[(i32, i32)], border: Border, erode: bool) -> Img {
    let mut out = Img::zeros(src.w, src.h);
    for y in 0..src.h {
        for x in 0..src.w {
            let mut acc: Option<u8> = None;
            for &(dy, dx) in kernel {
                let (sx, sy) = (x + dx, y + dy);
                let v = if sx >= 0 && sx < src.w && sy >= 0 && sy < src.h {
                    Some(src.data[sy as usize * src.w as usize + sx as usize])
                } else {
                    match border {
                        Border::Ignore => None,
                        Border::Constant(c) => Some(c),
                    }
                };
                if let Some(v) = v {
                    acc = Some(match acc {
                        None => v,
                        Some(a) => {
                            if erode {
                                a.min(v)
                            } else {
                                a.max(v)
                            }
                        }
                    });
                }
            }
            out.data[y as usize * src.w as usize + x as usize] = acc.unwrap_or(if erode { 255 } else { 0 });
        }
    }
    out
}

/// `cv2.erode(src, ellipse_kernel)`; `border` = default (`Ignore`) or
/// `BORDER_CONSTANT` with a value.
pub fn erode(src: &Img, kernel: &[(i32, i32)], border: Border) -> Img {
    morph(src, kernel, border, true)
}

/// `cv2.dilate(src, ellipse_kernel)` with the default border.
pub fn dilate(src: &Img, kernel: &[(i32, i32)]) -> Img {
    morph(src, kernel, Border::Ignore, false)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use serde_json::Value;

    fn pts_of(v: &Value) -> Vec<(i32, i32)> {
        v.as_array().unwrap().iter().map(|p| (p[0].as_i64().unwrap() as i32, p[1].as_i64().unwrap() as i32)).collect()
    }

    fn bytes_of(v: &Value) -> Vec<u8> {
        v.as_array().unwrap().iter().map(|x| x.as_i64().unwrap() as u8).collect()
    }

    /// An expected image is either the full byte list or `{fnv, n}` (FNV-1a
    /// 64 of the bytes) for the larger ones, to keep the fixture small.
    pub(crate) fn same(got: &[u8], want: &Value) -> bool {
        match want {
            Value::Array(_) => got == bytes_of(want).as_slice(),
            Value::Object(o) => o["n"].as_u64().unwrap() as usize == got.len() && o["fnv"].as_str().unwrap() == fnv1a(got),
            _ => false,
        }
    }

    pub(crate) fn fnv1a(data: &[u8]) -> String {
        let mut h: u64 = 0xcbf29ce484222325;
        for &b in data {
            h ^= b as u64;
            h = h.wrapping_mul(0x100000001b3);
        }
        format!("{h:016x}")
    }

    /// Every case in `tests/raster_oracle.json` was rendered by OpenCV 4.6.0
    /// (`raster_oracle.py`); the port has to reproduce each byte.
    #[test]
    fn matches_opencv_oracle() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/raster_oracle.json");
        let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("oracle file {path}: {e}"));
        let cases: Vec<Value> = serde_json::from_str(&text).unwrap();
        assert!(cases.len() >= 500);
        let mut failures = Vec::new();
        let mut counts = std::collections::BTreeMap::new();
        for (n, case) in cases.iter().enumerate() {
            let op = case["op"].as_str().unwrap();
            let (w, h) = (case["w"].as_i64().unwrap() as i32, case["h"].as_i64().unwrap() as i32);
            *counts.entry(op.to_string()).or_insert(0) += 1;
            let mut checks: Vec<(&str, Vec<u8>, &Value)> = Vec::new();
            match op {
                "fill_poly" => {
                    let mut img = Img::zeros(w, h);
                    fill_poly(&mut img, &pts_of(&case["pts"]), 1);
                    checks.push(("out", img.data, &case["out"]));
                }
                "polylines1" => {
                    let mut img = Img::zeros(w, h);
                    polylines(&mut img, &pts_of(&case["pts"]), case["closed"].as_bool().unwrap(), 1, 1);
                    checks.push(("out", img.data, &case["out"]));
                }
                "line" => {
                    let mut img = Img::zeros(w, h);
                    let p1 = pts_of(&Value::Array(vec![case["p1"].clone()]))[0];
                    let p2 = pts_of(&Value::Array(vec![case["p2"].clone()]))[0];
                    line(&mut img, p1, p2, 1, case["t"].as_i64().unwrap() as i32);
                    checks.push(("out", img.data, &case["out"]));
                }
                "morph" => {
                    let r = case["r"].as_i64().unwrap() as i32;
                    let src = Img { w, h, data: bytes_of(&case["src"]) };
                    let k = ellipse_kernel(r);
                    let mut kimg = Img::zeros(2 * r + 1, 2 * r + 1);
                    for &(dy, dx) in &k {
                        kimg.put(dx + r, dy + r, 1);
                    }
                    checks.push(("kernel", kimg.data, &case["kernel"]));
                    checks.push(("erode_default", erode(&src, &k, Border::Ignore).data, &case["erode_default"]));
                    checks.push(("erode_const0", erode(&src, &k, Border::Constant(0)).data, &case["erode_const0"]));
                    checks.push(("dilate_default", dilate(&src, &k).data, &case["dilate_default"]));
                }
                other => panic!("unknown op {other}"),
            }
            for (what, got, want) in checks {
                if !same(&got, want) {
                    failures.push(format!("case {n} {op} {what} ({w}x{h}) differs; {}", case.to_string().chars().take(200).collect::<String>()));
                }
            }
        }
        eprintln!("oracle cases: {counts:?}");
        assert!(failures.is_empty(), "{} failures:\n{}", failures.len(), failures.join("\n"));
    }
}
