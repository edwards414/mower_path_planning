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
"""Unit tests for path_generators/zigzag.py (TDD — written before impl)."""

import numpy as np
import pytest

from mower_mission.path_generators.zigzag import _generate_coverage_zigzag_path

RES = 0.1
OX = 0.0
OY = 0.0
STRIP = 0.2
SPACING = 0.1


def _call(grid):
    H, W = grid.shape
    return _generate_coverage_zigzag_path(
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


def test_risk_hole_no_unsafe_waypoints():
    grid = np.ones((20, 20), dtype=bool)
    grid[8:12, 8:12] = False   # interior obstacle
    pts, _, _ = _call(grid)
    for x, y in pts:
        r, c = _pt_to_cell(x, y)
        if 0 <= r < 20 and 0 <= c < 20:
            assert grid[r, c], f'Point ({x:.2f},{y:.2f}) lands on unsafe cell ({r},{c})'


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

def test_separated_runs_produce_invalid_segments():
    """A full-height wall forces cross-gap transitions → at least one invalid seg."""
    grid = np.ones((20, 20), dtype=bool)
    grid[:, 10] = False   # vertical wall splits the map into left/right halves
    pts, _, invalid_segs = _call(grid)
    if len(pts) >= 2:
        assert len(invalid_segs) > 0, (
            'Expected invalid segments when zigzag must cross a vertical wall'
        )


def test_clear_map_has_no_invalid_segments():
    """A fully open rectangle should produce zero invalid segments."""
    grid = np.ones((20, 20), dtype=bool)
    _, _, invalid_segs = _call(grid)
    assert invalid_segs == []
