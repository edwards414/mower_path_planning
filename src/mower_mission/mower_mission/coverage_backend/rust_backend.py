"""Rust backend — thin wrappers around the mower_coverage_core PyO3 extension."""

from __future__ import annotations

import numpy as np


class RustBackend:
    def __init__(self):
        import mower_coverage_core as _core
        self._core = _core

    def filter_safe_components(self, safe_map, resolution, min_area_m2, keep_largest_only):
        grid = np.asarray(safe_map, dtype=bool)
        filtered, all_sizes, kept_sizes = self._core.py_filter_safe_components(
            grid, resolution, min_area_m2, keep_largest_only,
        )
        return np.asarray(filtered, dtype=bool), list(all_sizes), list(kept_sizes)

    def validate_path(self, points, safe_map):
        grid = np.ascontiguousarray(safe_map.grid, dtype=bool)
        valid, inv_pts, inv_segs, msg = self._core.py_validate_path(
            list(points), grid,
            safe_map.resolution, safe_map.origin_x, safe_map.origin_y,
        )
        from ..coverage.path_validator import ValidationResult
        return ValidationResult(
            valid=valid,
            invalid_points=list(inv_pts),
            invalid_segments=[tuple(s) for s in inv_segs],
            message=msg,
        )

    def plan_connector(self, start, end, safe_map, boundary_weight=0.2):
        grid = np.ascontiguousarray(safe_map.grid, dtype=bool)
        result = self._core.py_plan_connector(
            tuple(start), tuple(end), grid,
            safe_map.resolution, safe_map.origin_x, safe_map.origin_y,
            boundary_weight,
        )
        if result is None:
            return None
        return [tuple(p) for p in result]

    def generate_zigzag_path(
        self, safe_map, strip_width_m,
        res, H, W, origin_x, origin_y,
    ):
        grid = np.ascontiguousarray(np.asarray(safe_map, dtype=bool))
        pts, split_pts, inv_segs = self._core.py_generate_coverage_zigzag_path(
            grid, strip_width_m, strip_width_m, res, H, W,
            origin_x, origin_y, 0.0,
        )
        return (
            [tuple(p) for p in pts],
            [tuple(p) for p in split_pts],
            [tuple(s) for s in inv_segs],
        )

    def generate_spiral_path(
        self, safe_map, strip_width_m,
        res, H, W, origin_x, origin_y,
    ):
        grid = np.ascontiguousarray(np.asarray(safe_map, dtype=bool))
        pts, split_pts, inv_segs = self._core.py_generate_coverage_spiral_path(
            grid, strip_width_m, strip_width_m, res, H, W,
            origin_x, origin_y,
        )
        return (
            [tuple(p) for p in pts],
            [tuple(p) for p in split_pts],
            [tuple(s) for s in inv_segs],
        )
