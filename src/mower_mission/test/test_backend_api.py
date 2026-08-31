"""Pure-Python coverage backend contract tests."""

import numpy as np

from mower_mission.coverage_backend.python_backend import PythonBackend


def _planning_kwargs(grid, *, waypoint_spacing_m=0.1):
    height, width = grid.shape
    return {
        'safe_map': grid,
        'strip_width_m': 0.4,
        'waypoint_spacing_m': waypoint_spacing_m,
        'res': 0.1,
        'H': height,
        'W': width,
        'origin_x': 0.0,
        'origin_y': 0.0,
    }


def test_python_backend_accepts_documented_zigzag_parameters():
    backend = PythonBackend()
    grid = np.ones((30, 30), dtype=bool)

    axis_points, _, _ = backend.generate_zigzag_path(
        **_planning_kwargs(grid), angle_deg=0.0
    )
    rotated_points, _, _ = backend.generate_zigzag_path(
        **_planning_kwargs(grid), angle_deg=45.0
    )

    assert axis_points
    assert rotated_points
    assert axis_points != rotated_points


def test_waypoint_spacing_controls_zigzag_sampling_density():
    backend = PythonBackend()
    grid = np.ones((30, 30), dtype=bool)

    dense, _, _ = backend.generate_zigzag_path(
        **_planning_kwargs(grid, waypoint_spacing_m=0.1)
    )
    sparse, _, _ = backend.generate_zigzag_path(
        **_planning_kwargs(grid, waypoint_spacing_m=0.4)
    )

    assert len(dense) > len(sparse)


def test_waypoint_spacing_controls_spiral_sampling_density():
    backend = PythonBackend()
    grid = np.ones((30, 30), dtype=bool)

    dense, _, _ = backend.generate_spiral_path(
        **_planning_kwargs(grid, waypoint_spacing_m=0.1)
    )
    sparse, _, _ = backend.generate_spiral_path(
        **_planning_kwargs(grid, waypoint_spacing_m=0.4)
    )

    assert len(dense) > len(sparse)
