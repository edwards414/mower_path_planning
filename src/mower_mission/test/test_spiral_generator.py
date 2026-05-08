# Copyright 2024 fxrbindi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Unit tests for path_generators/speiral.py (TDD — written before impl)."""

import numpy as np
import pytest

from mower_mission.path_generators.speiral import (
    _generate_coverage_spiral_path,
    plan_spiral_coverage,
)
from mower_mission.coverage.types import SpiralCoveragePlan, SpiralSegment

RES = 0.1
OX = 0.0
OY = 0.0
STRIP = 0.2
SPACING = 0.1


def _call(grid):
    H, W = grid.shape
    return _generate_coverage_spiral_path(
        safe_map=grid,
        strip_width_m=STRIP,
        waypoint_spacing_m=SPACING,
        res=RES,
        H=H,
        W=W,
        origin_x=OX,
        origin_y=OY,
    )


def _pt_to_cell(x, y):
    col = int((x - OX) / RES)
    row = int((y - OY) / RES)
    return row, col


def _max_segment_distance(points):
    if len(points) < 2:
        return 0.0
    return max(
        float(np.hypot(x1 - x0, y1 - y0))
        for (x0, y0), (x1, y1) in zip(points, points[1:])
    )


# ──────────────────────────────────────────────────────────────────────────────
# Return contract
# ──────────────────────────────────────────────────────────────────────────────

def test_returns_three_tuple():
    grid = np.ones((10, 10), dtype=bool)
    result = _call(grid)
    assert isinstance(result, tuple)
    assert len(result) == 3


def test_empty_map_returns_empty():
    grid = np.zeros((10, 10), dtype=bool)
    pts, split_pts, invalid_segs = _call(grid)
    assert pts == []
    assert split_pts == []
    assert invalid_segs == []


def test_rectangle_returns_nonempty():
    grid = np.ones((20, 20), dtype=bool)
    pts, split_pts, _ = _call(grid)
    assert len(pts) >= 2
    assert len(split_pts) >= 1


def test_accepts_uint8_safe_map():
    grid = np.ones((20, 20), dtype=np.uint8)
    pts, split_pts, invalid_segs = _call(grid)
    assert len(pts) >= 2
    assert len(split_pts) >= 1
    assert invalid_segs == []


def test_spiral_order_is_not_row_major_scan():
    grid = np.ones((20, 20), dtype=bool)
    pts, _, _ = _call(grid)
    cells = [_pt_to_cell(x, y) for x, y in pts[:80]]
    rows = {r for r, _c in cells}
    cols = {c for _r, c in cells}
    assert len(rows) > 1
    assert len(cols) > 1


def test_spiral_uses_strip_centerlines_not_every_cell():
    grid = np.ones((20, 20), dtype=bool)
    pts, _, _ = _call(grid)
    assert len(pts) < int(np.count_nonzero(grid))


# ──────────────────────────────────────────────────────────────────────────────
# Waypoint safety
# ──────────────────────────────────────────────────────────────────────────────

def test_all_waypoints_on_safe_cells():
    grid = np.ones((20, 20), dtype=bool)
    pts, _, _ = _call(grid)
    for x, y in pts:
        r, c = _pt_to_cell(x, y)
        assert 0 <= r < 20 and 0 <= c < 20
        assert grid[r, c], f'Point ({x:.2f},{y:.2f}) → cell ({r},{c}) is unsafe'


def test_center_hole_avoided():
    """No waypoint should land inside the central hole."""
    grid = np.ones((20, 20), dtype=bool)
    grid[8:12, 8:12] = False   # central hole
    pts, _, _ = _call(grid)
    for x, y in pts:
        r, c = _pt_to_cell(x, y)
        if 0 <= r < 20 and 0 <= c < 20:
            assert grid[r, c], f'Point ({x:.2f},{y:.2f}) lands on unsafe cell ({r},{c})'


def test_obstacle_no_unsafe_waypoints():
    """Obstacle anywhere must not appear in the path."""
    grid = np.ones((30, 30), dtype=bool)
    grid[5:10, 5:25] = False   # horizontal obstacle band
    pts, _, _ = _call(grid)
    H, W = grid.shape
    for x, y in pts:
        r, c = _pt_to_cell(x, y)
        if 0 <= r < H and 0 <= c < W:
            assert grid[r, c], f'Point ({x:.2f},{y:.2f}) on unsafe cell ({r},{c})'


# ──────────────────────────────────────────────────────────────────────────────
# Split points
# ──────────────────────────────────────────────────────────────────────────────

def test_split_points_nonempty_for_valid_map():
    grid = np.ones((20, 20), dtype=bool)
    _, split_pts, _ = _call(grid)
    assert len(split_pts) >= 1


def test_split_points_are_in_point_list():
    """Every split point must also appear in the main points list."""
    grid = np.ones((20, 20), dtype=bool)
    pts, split_pts, _ = _call(grid)
    pt_set = set(pts)
    for sp in split_pts:
        assert sp in pt_set, f'Split point {sp} not found in points'


# ──────────────────────────────────────────────────────────────────────────────
# Invalid segment detection
# ──────────────────────────────────────────────────────────────────────────────

def test_connected_obstacle_transitions_are_bridged_safely():
    """A connected safe region should route around an obstacle, not jump it."""
    grid = np.ones((20, 20), dtype=bool)
    grid[5:15, 8:12] = False   # central obstacle with safe corridors around it
    pts, _, invalid_segs = _call(grid)
    assert invalid_segs == []
    assert _max_segment_distance(pts) <= STRIP * 1.75


def test_disconnected_transition_reported():
    """Disconnected safe islands still produce invalid transition segments."""
    grid = np.ones((20, 20), dtype=bool)
    grid[:, 8:12] = False
    pts, _, invalid_segs = _call(grid)
    if len(pts) >= 2:
        assert len(invalid_segs) > 0


def test_concave_region_avoids_long_greedy_jumps():
    """C-shaped maps should not contain long same-layer straight jumps."""
    grid = np.zeros((60, 60), dtype=bool)
    grid[5:55, 5:55] = True
    grid[15:45, 20:55] = False
    pts, _, invalid_segs = _call(grid)
    assert invalid_segs == []
    assert _max_segment_distance(pts) <= STRIP * 1.75


def test_clear_rectangle_no_invalid_segments():
    """A fully open rectangle: every neighbour transition is safe."""
    grid = np.ones((10, 10), dtype=bool)
    _, _, invalid_segs = _call(grid)
    assert invalid_segs == []


# ──────────────────────────────────────────────────────────────────────────────
# plan_spiral_coverage — SpiralCoveragePlan API
# ──────────────────────────────────────────────────────────────────────────────

def _plan(grid):
    H, W = grid.shape
    return plan_spiral_coverage(
        safe_map=grid,
        strip_width_m=STRIP,
        waypoint_spacing_m=SPACING,
        res=RES,
        H=H,
        W=W,
        origin_x=OX,
        origin_y=OY,
    )


def _two_islands():
    """Two 8×8 safe islands separated by a 4-column unsafe gap."""
    grid = np.zeros((10, 22), dtype=bool)
    grid[1:9, 1:9]  = True   # left island
    grid[1:9, 13:21] = True  # right island
    return grid


def test_plan_returns_spiral_coverage_plan():
    plan = _plan(np.ones((10, 10), dtype=bool))
    assert isinstance(plan, SpiralCoveragePlan)


def test_single_component_no_bridge():
    plan = _plan(np.ones((10, 10), dtype=bool))
    types = {s.segment_type for s in plan.segments}
    assert 'bridge' not in types


def test_two_islands_all_safe_cells_covered_by_strip_mask():
    grid = _two_islands()
    plan = _plan(grid)
    uncovered = grid & ~plan.coverage_mask
    assert not np.any(uncovered), f'Uncovered cells: {np.argwhere(uncovered)}'


def test_two_disconnected_islands_both_planned():
    """Disconnected islands: both get a spiral segment even without a safe bridge."""
    plan = _plan(_two_islands())
    spiral_count = sum(1 for s in plan.segments if s.segment_type == 'spiral')
    assert spiral_count == 2, f'Expected 2 spiral segments, got {spiral_count}'
    # Transition is 'invalid' because there is no safe 4-neighbor path across the gap
    transition_types = {s.segment_type for s in plan.segments} - {'spiral'}
    assert transition_types <= {'bridge', 'invalid'}


def test_bridge_waypoints_all_safe():
    grid = _two_islands()
    plan = _plan(grid)
    H, W = grid.shape
    for seg in plan.segments:
        if seg.segment_type == 'bridge':
            for x, y in seg.points:
                r, c = int((y - OY) / RES), int((x - OX) / RES)
                if 0 <= r < H and 0 <= c < W:
                    assert grid[r, c], f'Bridge point ({x},{y}) on unsafe cell ({r},{c})'


def test_coverage_mask_matches_visited_points():
    grid = np.ones((10, 10), dtype=bool)
    plan = _plan(grid)
    for x, y in plan.points:
        r, c = int((y - OY) / RES), int((x - OX) / RES)
        if 0 <= r < 10 and 0 <= c < 10:
            assert plan.coverage_mask[r, c], f'Point ({x},{y}) → ({r},{c}) not in coverage_mask'


def test_split_points_are_ends_of_spiral_segments():
    plan = _plan(_two_islands())
    spiral_ends = [s.points[-1] for s in plan.segments if s.segment_type == 'spiral' and s.points]
    for end in spiral_ends:
        assert end in plan.split_points, f'Spiral-segment end {end} not in split_points'
