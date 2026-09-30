//! Shared helpers for the integration tests: grid construction, the recorded
//! Python reference oracle (`tests/data/python_reference_oracle.json`) and
//! the per-section "run the Rust function on the recorded input and compare"
//! checks used by `python_reference_oracle.rs` and `backend_parity.rs`.

#![allow(dead_code)]

use std::fs;
use std::path::Path;
use std::sync::OnceLock;

use ndarray::Array2;
use serde_json::Value;

use mower_coverage_core::cell_decomposition::decompose;
use mower_coverage_core::connector_planner::plan_connector_rs;
use mower_coverage_core::path_validator::validate_path_rs;
use mower_coverage_core::safe_map_filter::filter_safe_components_rs;
use mower_coverage_core::spiral::plan_spiral_coverage_rs;
use mower_coverage_core::types::SafeMap;
use mower_coverage_core::zigzag::generate_coverage_zigzag_path_rs;

/// Float tolerance for recorded coordinates / areas. Integers and bools are
/// compared exactly.
pub const TOL: f64 = 1e-9;

// ── Grids ──────────────────────────────────────────────────────────────────────

/// Resolution and origin used by the Python unit tests (conftest.py).
pub const RES: f64 = 0.1;
pub const OX: f64 = 0.0;
pub const OY: f64 = 0.0;

pub fn ones(h: usize, w: usize) -> Array2<bool> {
    Array2::from_elem((h, w), true)
}

pub fn zeros(h: usize, w: usize) -> Array2<bool> {
    Array2::from_elem((h, w), false)
}

/// numpy `grid[r0:r1, c0:c1] = v` (half-open ranges).
pub fn set(grid: &mut Array2<bool>, rows: std::ops::Range<usize>, cols: std::ops::Range<usize>, v: bool) {
    for r in rows {
        for c in cols.clone() {
            grid[[r, c]] = v;
        }
    }
}

pub fn safe_map(grid: &Array2<bool>) -> SafeMap<'_> {
    SafeMap { grid: grid.view(), resolution: RES, origin_x: OX, origin_y: OY }
}

pub fn count_true(grid: &Array2<bool>) -> usize {
    grid.iter().filter(|&&v| v).count()
}

/// Python `int((x - OX) / RES), int((y - OY) / RES)` as `(row, col)`.
pub fn pt_to_cell(x: f64, y: f64) -> (i64, i64) {
    (((y - OY) / RES) as i64, ((x - OX) / RES) as i64)
}

pub fn in_grid(grid: &Array2<bool>, r: i64, c: i64) -> bool {
    r >= 0 && c >= 0 && (r as usize) < grid.nrows() && (c as usize) < grid.ncols()
}

// ── Oracle loading ─────────────────────────────────────────────────────────────

pub fn oracle() -> &'static Value {
    static ORACLE: OnceLock<Value> = OnceLock::new();
    ORACLE.get_or_init(|| {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests")
            .join("data")
            .join("python_reference_oracle.json");
        let text = fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
        serde_json::from_str(&text).expect("oracle JSON parses")
    })
}

pub fn cases(section: &str) -> &'static [Value] {
    oracle()[section]
        .as_array()
        .unwrap_or_else(|| panic!("oracle has no section {section:?}"))
}

pub fn case(section: &str, id: &str) -> &'static Value {
    cases(section)
        .iter()
        .find(|c| c["id"] == id)
        .unwrap_or_else(|| panic!("oracle section {section:?} has no case {id:?}"))
}

pub struct OracleMap {
    pub name: String,
    pub grid: Array2<bool>,
    pub res: f64,
    pub ox: f64,
    pub oy: f64,
}

impl OracleMap {
    pub fn sm(&self) -> SafeMap<'_> {
        SafeMap { grid: self.grid.view(), resolution: self.res, origin_x: self.ox, origin_y: self.oy }
    }
}

pub fn grid_from_rows(rows: &Value) -> Array2<bool> {
    let rows: Vec<&str> = rows
        .as_array()
        .expect("rows array")
        .iter()
        .map(|r| r.as_str().expect("row string"))
        .collect();
    let h = rows.len();
    let w = rows.first().map_or(0, |r| r.len());
    let mut grid = zeros(h, w);
    for (r, row) in rows.iter().enumerate() {
        assert_eq!(row.len(), w, "ragged grid row {r}");
        for (c, ch) in row.bytes().enumerate() {
            grid[[r, c]] = match ch {
                b'1' => true,
                b'0' => false,
                other => panic!("bad grid char {other:?}"),
            };
        }
    }
    grid
}

pub fn oracle_map(name: &str) -> OracleMap {
    let m = &oracle()["maps"][name];
    assert!(m.is_object(), "oracle has no map {name:?}");
    OracleMap {
        name: name.to_string(),
        grid: grid_from_rows(&m["rows"]),
        res: num(&m["res"]),
        ox: num(&m["origin_x"]),
        oy: num(&m["origin_y"]),
    }
}

pub fn case_map(case: &Value) -> OracleMap {
    oracle_map(case["map"].as_str().expect("case map name"))
}

// ── JSON conversions ───────────────────────────────────────────────────────────

pub fn num(v: &Value) -> f64 {
    v.as_f64().unwrap_or_else(|| panic!("expected number, got {v}"))
}

pub fn uint(v: &Value) -> usize {
    v.as_u64().unwrap_or_else(|| panic!("expected unsigned int, got {v}")) as usize
}

pub fn boolean(v: &Value) -> bool {
    v.as_bool().unwrap_or_else(|| panic!("expected bool, got {v}"))
}

pub fn point(v: &Value) -> (f64, f64) {
    (num(&v[0]), num(&v[1]))
}

pub fn points(v: &Value) -> Vec<(f64, f64)> {
    v.as_array().expect("points array").iter().map(point).collect()
}

pub fn uints(v: &Value) -> Vec<usize> {
    v.as_array().expect("int array").iter().map(uint).collect()
}

pub fn pairs(v: &Value) -> Vec<(usize, usize)> {
    v.as_array().expect("pair array").iter().map(|p| (uint(&p[0]), uint(&p[1]))).collect()
}

// ── Comparisons ────────────────────────────────────────────────────────────────

pub fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= TOL
}

/// Same length and every coordinate within `TOL`; reports the first mismatch.
pub fn cmp_points(ctx: &str, what: &str, actual: &[(f64, f64)], expected: &[(f64, f64)], errs: &mut Vec<String>) {
    if actual.len() != expected.len() {
        errs.push(format!("{ctx}: {what} length rust={} python={}", actual.len(), expected.len()));
        return;
    }
    for (i, (a, e)) in actual.iter().zip(expected).enumerate() {
        if !close(a.0, e.0) || !close(a.1, e.1) {
            errs.push(format!("{ctx}: {what}[{i}] rust={a:?} python={e:?}"));
            return;
        }
    }
}

pub fn cmp_eq<T: PartialEq + std::fmt::Debug>(ctx: &str, what: &str, actual: &T, expected: &T, errs: &mut Vec<String>) {
    if actual != expected {
        errs.push(format!("{ctx}: {what} rust={actual:?} python={expected:?}"));
    }
}

/// Panics listing up to 20 mismatches.
pub fn assert_no_mismatches(section: &str, checked: usize, errs: &[String]) {
    if !errs.is_empty() {
        let shown: Vec<&String> = errs.iter().take(20).collect();
        panic!(
            "{section}: {} mismatch(es) in {checked} case(s) vs the Python reference:\n{}",
            errs.len(),
            shown.iter().map(|s| s.as_str()).collect::<Vec<_>>().join("\n")
        );
    }
}

// ── Per-section checks: run Rust on the recorded input, compare outputs ───────

pub fn check_filter(case: &Value) -> Vec<String> {
    let ctx = case["id"].as_str().unwrap_or("?");
    let m = case_map(case);
    let (filtered, all_sizes, kept_sizes) = filter_safe_components_rs(
        m.grid.view(),
        m.res,
        num(&case["min_area_m2"]),
        boolean(&case["keep_largest_only"]),
    );
    let mut errs = Vec::new();
    let expected = grid_from_rows(&case["filtered"]);
    if filtered != expected {
        let diff = filtered.iter().zip(expected.iter()).filter(|(a, b)| a != b).count();
        errs.push(format!("{ctx}: filtered grid differs in {diff} cell(s)"));
    }
    cmp_eq(ctx, "all_sizes", &all_sizes, &uints(&case["all_sizes"]), &mut errs);
    cmp_eq(ctx, "kept_sizes", &kept_sizes, &uints(&case["kept_sizes"]), &mut errs);
    errs
}

/// Deliberate change after the Python reference: `world_to_grid` rounds down
/// instead of truncating towards zero, so a point in the one-cell band left of
/// or below the grid is outside (unsafe) instead of row/column 0. Cases whose
/// points are all at x, y >= origin must match Python exactly; a case with a
/// point in that band may only add invalid points that lie there and invalid
/// segments with such an endpoint.
pub fn check_validate(case: &Value) -> Vec<String> {
    let ctx = case["id"].as_str().unwrap_or("?");
    let m = case_map(case);
    let pts = points(&case["points"]);
    let r = validate_path_rs(&pts, &m.sm());
    let mut errs = Vec::new();
    let below = |p: (f64, f64)| p.0 < m.ox || p.1 < m.oy;
    if !pts.iter().any(|&p| below(p)) {
        cmp_eq(ctx, "valid", &r.valid, &boolean(&case["valid"]), &mut errs);
        cmp_eq(ctx, "invalid_points", &r.invalid_points, &uints(&case["invalid_points"]), &mut errs);
        cmp_eq(ctx, "invalid_segments", &r.invalid_segments, &pairs(&case["invalid_segments"]), &mut errs);
        cmp_eq(ctx, "message", &r.message.as_str(), &case["message"].as_str().expect("message"), &mut errs);
        return errs;
    }
    let (py_pts, py_segs) = (uints(&case["invalid_points"]), pairs(&case["invalid_segments"]));
    for i in &py_pts {
        if !r.invalid_points.contains(i) {
            errs.push(format!("{ctx}: point {i} unsafe in Python but not in Rust"));
        }
    }
    for s in &py_segs {
        if !r.invalid_segments.contains(s) {
            errs.push(format!("{ctx}: segment {s:?} unsafe in Python but not in Rust"));
        }
    }
    for &i in r.invalid_points.iter().filter(|i| !py_pts.contains(i)) {
        if !below(pts[i]) {
            errs.push(format!("{ctx}: point {i} {:?} newly unsafe but inside the grid", pts[i]));
        }
    }
    for &(a, b) in r.invalid_segments.iter().filter(|s| !py_segs.contains(s)) {
        if !below(pts[a]) && !below(pts[b]) {
            errs.push(format!("{ctx}: segment ({a}, {b}) newly unsafe with both ends inside the grid"));
        }
    }
    errs
}

pub fn check_connector(case: &Value) -> Vec<String> {
    let ctx = case["id"].as_str().unwrap_or("?");
    let m = case_map(case);
    let path = plan_connector_rs(
        point(&case["start"]),
        point(&case["end"]),
        &m.sm(),
        num(&case["boundary_weight"]),
    );
    let mut errs = Vec::new();
    match (&path, case["path"].is_null()) {
        (None, true) => {}
        (Some(p), false) => cmp_points(ctx, "path", p, &points(&case["path"]), &mut errs),
        (p, _) => errs.push(format!(
            "{ctx}: rust path {} but python path {}",
            if p.is_some() { "found" } else { "None" },
            if case["path"].is_null() { "None" } else { "found" },
        )),
    }
    errs
}

pub fn check_zigzag(case: &Value) -> Vec<String> {
    let ctx = case["id"].as_str().unwrap_or("?");
    let m = case_map(case);
    let (h, w) = m.grid.dim();
    let (pts, split, inv) = generate_coverage_zigzag_path_rs(
        m.grid.view(),
        num(&case["strip_width_m"]),
        num(&case["waypoint_spacing_m"]),
        m.res,
        h,
        w,
        m.ox,
        m.oy,
        num(&case["angle_deg"]),
    );
    let mut errs = Vec::new();
    cmp_points(ctx, "points", &pts, &points(&case["points"]), &mut errs);
    cmp_points(ctx, "split_points", &split, &points(&case["split_points"]), &mut errs);
    cmp_eq(ctx, "invalid_segments", &inv, &pairs(&case["invalid_segments"]), &mut errs);
    errs
}

pub fn check_spiral(case: &Value) -> Vec<String> {
    let ctx = case["id"].as_str().unwrap_or("?");
    let m = case_map(case);
    let (h, w) = m.grid.dim();
    let (pts, split, inv) = plan_spiral_coverage_rs(
        m.grid.view(),
        num(&case["strip_width_m"]),
        num(&case["waypoint_spacing_m"]),
        m.res,
        h,
        w,
        m.ox,
        m.oy,
    );
    let mut errs = Vec::new();
    cmp_points(ctx, "points", &pts, &points(&case["points"]), &mut errs);
    cmp_points(ctx, "split_points", &split, &points(&case["split_points"]), &mut errs);
    cmp_eq(ctx, "invalid_segments", &inv, &pairs(&case["invalid_segments"]), &mut errs);
    errs
}

fn mask_from_runs(runs: &Value, h: usize, w: usize) -> Array2<bool> {
    let mut mask = zeros(h, w);
    for run in runs.as_array().expect("mask_runs") {
        let (r, c0, c1) = (uint(&run[0]), uint(&run[1]), uint(&run[2]));
        for c in c0..=c1 {
            mask[[r, c]] = true;
        }
    }
    mask
}

pub fn check_decompose(case: &Value) -> Vec<String> {
    let ctx = case["id"].as_str().unwrap_or("?");
    let m = case_map(case);
    let (h, w) = m.grid.dim();
    let (cells, graph) = decompose(&m.sm());
    let mut errs = Vec::new();
    let expected_cells = case["cells"].as_array().expect("cells");
    if cells.len() != expected_cells.len() {
        errs.push(format!("{ctx}: cell count rust={} python={}", cells.len(), expected_cells.len()));
        return errs;
    }
    for (cell, e) in cells.iter().zip(expected_cells) {
        let cctx = format!("{ctx} cell {}", cell.cell_id);
        cmp_eq(&cctx, "cell_id", &cell.cell_id, &uint(&e["cell_id"]), &mut errs);
        if cell.mask != mask_from_runs(&e["mask_runs"], h, w) {
            errs.push(format!("{cctx}: mask differs"));
        }
        let b = &e["bbox"];
        cmp_eq(&cctx, "bbox", &cell.bbox, &(uint(&b[0]), uint(&b[1]), uint(&b[2]), uint(&b[3])), &mut errs);
        if !close(cell.area_m2, num(&e["area_m2"])) {
            errs.push(format!("{cctx}: area_m2 rust={} python={}", cell.area_m2, num(&e["area_m2"])));
        }
        cmp_points(&cctx, "centroid_xy", &[cell.centroid_xy], &[point(&e["centroid_xy"])], &mut errs);
        cmp_points(&cctx, "entry_candidates", &cell.entry_candidates, &points(&e["entry_candidates"]), &mut errs);
        cmp_points(&cctx, "exit_candidates", &cell.exit_candidates, &points(&e["exit_candidates"]), &mut errs);
        cmp_eq(&cctx, "valid", &cell.valid, &boolean(&e["valid"]), &mut errs);
        cmp_eq(&cctx, "invalid_reason", &cell.invalid_reason.as_str(), &e["invalid_reason"].as_str().expect("reason"), &mut errs);
    }
    let expected_graph: Vec<(usize, Vec<usize>)> = case["graph"]
        .as_array()
        .expect("graph")
        .iter()
        .map(|kv| (uint(&kv[0]), uints(&kv[1])))
        .collect();
    let actual_graph: Vec<(usize, Vec<usize>)> = graph.into_iter().collect();
    cmp_eq(ctx, "graph", &actual_graph, &expected_graph, &mut errs);
    errs
}

/// Run the check for `section` on every recorded case; returns (checked, errors).
pub fn check_section(section: &str, check: fn(&Value) -> Vec<String>) -> (usize, Vec<String>) {
    let all = cases(section);
    let errs = all.iter().flat_map(check).collect();
    (all.len(), errs)
}

/// Assert that one named case (e.g. a ported parity test input) matches.
pub fn assert_case(section: &str, id: &str, check: fn(&Value) -> Vec<String>) {
    let errs = check(case(section, id));
    assert_no_mismatches(section, 1, &errs);
}
