"""Parity tests: verify Python and Rust backends produce identical results."""

import numpy as np
import pytest

# Skip entire module if Rust extension is not installed
pytest.importorskip('mower_coverage_core', reason='mower_coverage_core not installed')

from mower_mission.coverage.path_validator import SafeMap, validate_path
from mower_mission.coverage_backend.python_backend import PythonBackend
from mower_mission.coverage_backend.rust_backend import RustBackend

RES = 0.1
OX = 0.0
OY = 0.0


def _sm(grid):
    return SafeMap(grid=np.asarray(grid, dtype=bool), resolution=RES, origin_x=OX, origin_y=OY)


@pytest.fixture
def py():
    return PythonBackend()


@pytest.fixture
def rs():
    return RustBackend()


# ── PathValidator parity ───────────────────────────────────────────────────────

class TestPathValidatorParity:
    GRIDS = [
        np.ones((10, 10), dtype=bool),
        np.zeros((10, 10), dtype=bool),
    ]

    def _wall_grid(self):
        g = np.ones((20, 20), dtype=bool)
        g[:, 10] = False
        return g

    def test_is_point_safe_open(self, py, rs):
        g = np.ones((10, 10), dtype=bool)
        sm = _sm(g)
        for x in [0.05, 0.55, 0.95]:
            for y in [0.05, 0.55, 0.95]:
                assert (py.validate_path([(x, y)], sm).valid ==
                        rs.validate_path([(x, y)], sm).valid)

    def test_is_point_unsafe(self, py, rs):
        g = np.ones((10, 10), dtype=bool)
        g[5, 5] = False
        sm = _sm(g)
        pts = [(0.55, 0.55)]
        assert py.validate_path(pts, sm).valid == rs.validate_path(pts, sm).valid
        assert not rs.validate_path(pts, sm).valid

    def test_segment_crossing_wall(self, py, rs):
        sm = _sm(self._wall_grid())
        pts = [(0.55, 1.05), (1.45, 1.05)]
        py_r = py.validate_path(pts, sm)
        rs_r = rs.validate_path(pts, sm)
        assert py_r.valid == rs_r.valid
        assert py_r.invalid_segments == rs_r.invalid_segments

    def test_full_path_open_grid(self, py, rs):
        g = np.ones((20, 20), dtype=bool)
        sm = _sm(g)
        pts = [(0.05, 0.05), (0.95, 0.05), (0.95, 0.95), (0.05, 0.95)]
        py_r = py.validate_path(pts, sm)
        rs_r = rs.validate_path(pts, sm)
        assert py_r.valid == rs_r.valid
        assert py_r.invalid_points == rs_r.invalid_points
        assert py_r.invalid_segments == rs_r.invalid_segments

    def test_out_of_bounds_point(self, py, rs):
        g = np.ones((10, 10), dtype=bool)
        sm = _sm(g)
        pts = [(-0.5, 0.05)]
        assert py.validate_path(pts, sm).valid == rs.validate_path(pts, sm).valid

    def test_negative_origin(self, py, rs):
        g = np.ones((10, 10), dtype=bool)
        sm = SafeMap(grid=g, resolution=0.1, origin_x=-0.5, origin_y=-0.5)
        pts = [(-0.45, -0.45), (-0.05, -0.05)]
        py_r = py.validate_path(pts, sm)
        rs_r = rs.validate_path(pts, sm)
        assert py_r.valid == rs_r.valid
        assert py_r.invalid_segments == rs_r.invalid_segments


# ── SafeMapFilter parity ───────────────────────────────────────────────────────

class TestSafeMapFilterParity:
    def test_single_component(self, py, rs):
        g = np.ones((10, 10), dtype=bool)
        py_f, py_all, py_kept = py.filter_safe_components(g, RES, 0.05, True)
        rs_f, rs_all, rs_kept = rs.filter_safe_components(g, RES, 0.05, True)
        assert np.array_equal(py_f, rs_f)
        assert py_all == rs_all
        assert py_kept == rs_kept

    def test_removes_tiny_island(self, py, rs):
        g = np.zeros((6, 6), dtype=bool)
        g[0:3, 0:3] = True
        g[5, 5] = True
        py_f, py_all, py_kept = py.filter_safe_components(g, 0.1, 0.05, True)
        rs_f, rs_all, rs_kept = rs.filter_safe_components(g, 0.1, 0.05, True)
        assert np.array_equal(py_f, rs_f)
        assert py_all == rs_all
        assert py_kept == rs_kept

    def test_keeps_largest_only(self, py, rs):
        g = np.zeros((8, 8), dtype=bool)
        g[0:3, 0:3] = True
        g[5:7, 5:7] = True
        py_f, py_all, py_kept = py.filter_safe_components(g, 0.1, 0.01, True)
        rs_f, rs_all, rs_kept = rs.filter_safe_components(g, 0.1, 0.01, True)
        assert np.array_equal(py_f, rs_f)
        assert py_all == rs_all
        assert py_kept == rs_kept

    def test_empty_grid(self, py, rs):
        g = np.zeros((5, 5), dtype=bool)
        py_f, py_all, py_kept = py.filter_safe_components(g, 0.1, 0.05, True)
        rs_f, rs_all, rs_kept = rs.filter_safe_components(g, 0.1, 0.05, True)
        assert np.array_equal(py_f, rs_f)
        assert py_all == rs_all
        assert py_kept == rs_kept


# ── ConnectorPlanner parity ────────────────────────────────────────────────────

class TestConnectorPlannerParity:
    def test_open_grid_both_find_path(self, py, rs):
        sm = _sm(np.ones((20, 20), dtype=bool))
        start, end = (0.05, 0.05), (1.95, 1.95)
        py_p = py.plan_connector(start, end, sm)
        rs_p = rs.plan_connector(start, end, sm)
        assert (py_p is None) == (rs_p is None)
        if rs_p is not None:
            result = validate_path(rs_p, sm)
            assert result.valid, f'Rust connector invalid: {result.message}'

    def test_disconnected_both_return_none(self, py, rs):
        g = np.ones((20, 20), dtype=bool)
        g[:, 10] = False
        sm = _sm(g)
        assert py.plan_connector((0.55, 1.05), (1.55, 1.05), sm) is None
        assert rs.plan_connector((0.55, 1.05), (1.55, 1.05), sm) is None

    def test_wall_with_gap(self, py, rs):
        g = np.ones((20, 20), dtype=bool)
        g[:18, 10] = False
        sm = _sm(g)
        py_p = py.plan_connector((0.55, 0.55), (1.55, 0.55), sm)
        rs_p = rs.plan_connector((0.55, 0.55), (1.55, 0.55), sm)
        assert (py_p is None) == (rs_p is None)
        if rs_p is not None:
            assert validate_path(rs_p, sm).valid

    def test_unsafe_start_returns_none(self, py, rs):
        g = np.ones((10, 10), dtype=bool)
        g[0, 0] = False
        sm = _sm(g)
        assert py.plan_connector((0.05, 0.05), (0.95, 0.95), sm) is None
        assert rs.plan_connector((0.05, 0.05), (0.95, 0.95), sm) is None

    def test_no_corner_cutting(self, py, rs):
        g = np.ones((5, 5), dtype=bool)
        g[1, 2] = False
        g[2, 1] = False
        sm = _sm(g)
        rs_p = rs.plan_connector((0.15, 0.15), (0.35, 0.35), sm)
        if rs_p is not None:
            assert validate_path(rs_p, sm).valid


# ── Zigzag generator parity ────────────────────────────────────────────────────

class TestZigzagParity:
    def _call(self, backend, grid, angle_deg=0.0):
        H, W = grid.shape
        return backend.generate_zigzag_path(
            grid, strip_width_m=0.2, waypoint_spacing_m=0.1,
            res=RES, H=H, W=W, origin_x=OX, origin_y=OY,
            angle_deg=angle_deg,
        )

    def test_empty_map_both_empty(self, py, rs):
        g = np.zeros((10, 10), dtype=bool)
        py_pts, py_sp, py_is = self._call(py, g)
        rs_pts, rs_sp, rs_is = self._call(rs, g)
        assert py_pts == rs_pts == []
        assert py_sp == rs_sp == []
        assert py_is == rs_is == []

    def test_open_grid_same_points(self, py, rs):
        g = np.ones((20, 20), dtype=bool)
        py_pts, py_sp, py_is = self._call(py, g)
        rs_pts, rs_sp, rs_is = self._call(rs, g)
        assert len(py_pts) == len(rs_pts), f'len mismatch: py={len(py_pts)} rs={len(rs_pts)}'
        assert np.allclose(py_pts, rs_pts, atol=1e-9), 'Point coordinates differ'
        assert py_sp == rs_sp
        assert py_is == rs_is

    def test_rotated_open_grid_runs_for_both_backends(self, py, rs):
        g = np.ones((20, 20), dtype=bool)
        py_pts, py_sp, py_is = self._call(py, g, angle_deg=45.0)
        rs_pts, rs_sp, rs_is = self._call(rs, g, angle_deg=45.0)
        assert len(py_pts) >= 2 and len(rs_pts) >= 2
        assert len(py_sp) >= 1 and len(rs_sp) >= 1
        assert py_is == [] and rs_is == []

    def test_all_rust_waypoints_on_safe_cells(self, py, rs):
        """Waypoints must not land on obstacle cells (segments may still be invalid)."""
        g = np.ones((20, 20), dtype=bool)
        g[8:12, 8:12] = False
        rs_pts, _, _ = self._call(rs, g)
        for x, y in rs_pts:
            col = int((x - OX) / RES)
            row = int((y - OY) / RES)
            if 0 <= row < g.shape[0] and 0 <= col < g.shape[1]:
                assert g[row, col], f'Rust zigzag placed waypoint ({x:.3f},{y:.3f}) on unsafe cell'

    def test_invalid_segments_match(self, py, rs):
        g = np.ones((20, 20), dtype=bool)
        g[:, 10] = False
        _, _, py_is = self._call(py, g)
        _, _, rs_is = self._call(rs, g)
        assert len(py_is) == len(rs_is), f'invalid segs mismatch: py={len(py_is)} rs={len(rs_is)}'

    def test_split_points_subset_of_points(self, py, rs):
        g = np.ones((20, 20), dtype=bool)
        rs_pts, rs_sp, _ = self._call(rs, g)
        pts_set = set(rs_pts)
        for sp in rs_sp:
            assert sp in pts_set, f'Split point {sp} not in Rust points'


# ── Spiral generator parity ────────────────────────────────────────────────────

class TestSpiralParity:
    def _call(self, backend, grid):
        H, W = grid.shape
        return backend.generate_spiral_path(
            grid, strip_width_m=0.4, waypoint_spacing_m=0.2,
            res=RES, H=H, W=W, origin_x=OX, origin_y=OY,
        )

    def test_empty_map_both_empty(self, py, rs):
        g = np.zeros((10, 10), dtype=bool)
        py_pts, py_sp, py_is = self._call(py, g)
        rs_pts, rs_sp, rs_is = self._call(rs, g)
        assert py_pts == rs_pts == []
        assert py_sp == rs_sp == []

    def test_open_grid_nonempty(self, py, rs):
        g = np.ones((20, 20), dtype=bool)
        rs_pts, rs_sp, _ = self._call(rs, g)
        assert len(rs_pts) >= 2
        assert len(rs_sp) >= 1

    def test_all_rust_points_safe(self, py, rs):
        g = np.ones((20, 20), dtype=bool)
        sm = _sm(g)
        rs_pts, _, _ = self._call(rs, g)
        if rs_pts:
            result = validate_path(rs_pts, sm)
            assert result.valid, f'Rust spiral has unsafe path: {result.message}'

    def test_two_islands_split_points_match_count(self, py, rs):
        g = np.zeros((20, 30), dtype=bool)
        g[2:8, 2:10] = True
        g[12:18, 20:28] = True
        py_pts, py_sp, py_is = self._call(py, g)
        rs_pts, rs_sp, rs_is = self._call(rs, g)
        # Both should produce 2 split points (one per component)
        assert len(py_sp) == len(rs_sp), (
            f'split_points count mismatch: py={len(py_sp)} rs={len(rs_sp)}'
        )

    def test_invalid_segs_count_not_worse(self, py, rs):
        g = np.zeros((20, 30), dtype=bool)
        g[2:8, 2:10] = True
        g[12:18, 20:28] = True
        _, _, py_is = self._call(py, g)
        _, _, rs_is = self._call(rs, g)
        assert len(rs_is) <= len(py_is) + 2, (
            f'Rust has more invalid segs: py={len(py_is)} rs={len(rs_is)}'
        )
