"""Python backend — thin wrappers around the existing Python implementations."""

from __future__ import annotations

import numpy as np


class PythonBackend:
    def filter_safe_components(self, safe_map, resolution, min_area_m2, keep_largest_only):
        from ..coverage.safe_map_filter import filter_safe_components
        return filter_safe_components(
            safe_map, resolution=resolution,
            min_area_m2=min_area_m2, keep_largest_only=keep_largest_only,
        )

    def validate_path(self, points, safe_map):
        from ..coverage.path_validator import validate_path
        return validate_path(points, safe_map)

    def plan_connector(self, start, end, safe_map, boundary_weight=0.2):
        from ..coverage.connector_planner import plan_connector
        return plan_connector(start, end, safe_map, boundary_weight)

    def generate_zigzag_path(
        self, safe_map, strip_width_m, waypoint_spacing_m,
        res, H, W, origin_x, origin_y, angle_deg=0.0,
    ):
        from ..path_generators.zigzag import _generate_coverage_zigzag_path
        return _generate_coverage_zigzag_path(
            safe_map=safe_map,
            strip_width_m=strip_width_m,
            waypoint_spacing_m=waypoint_spacing_m,
            res=res,
            H=H,
            W=W,
            origin_x=origin_x,
            origin_y=origin_y,
            angle_deg=angle_deg,
        )

    def generate_spiral_path(
        self, safe_map, strip_width_m, waypoint_spacing_m,
        res, H, W, origin_x, origin_y,
    ):
        from ..path_generators.spiral import _generate_coverage_spiral_path
        return _generate_coverage_spiral_path(
            safe_map=safe_map,
            strip_width_m=strip_width_m,
            waypoint_spacing_m=waypoint_spacing_m,
            res=res,
            H=H,
            W=W,
            origin_x=origin_x,
            origin_y=origin_y,
        )
