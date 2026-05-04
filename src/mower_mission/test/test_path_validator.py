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
"""Unit tests for coverage/path_validator.py."""

import numpy as np
import pytest

from mower_mission.coverage.path_validator import (
    SafeMap,
    ValidationResult,
    is_point_safe,
    is_segment_safe,
    validate_path,
    validate_points,
)

RES = 0.1   # 10 cm per cell
OX = 0.0
OY = 0.0


def _make_safe_map(grid: np.ndarray) -> SafeMap:
    return SafeMap(grid=grid.astype(bool), resolution=RES,
                   origin_x=OX, origin_y=OY)


# ──────────────────────────────────────────────────────────────────────────────
# is_point_safe
# ──────────────────────────────────────────────────────────────────────────────

def test_point_inside_safe_cell():
    grid = np.ones((10, 10), dtype=bool)
    sm = _make_safe_map(grid)
    assert is_point_safe(0.05, 0.05, sm)  # centre of cell (0,0)


def test_point_on_unsafe_cell():
    grid = np.ones((10, 10), dtype=bool)
    grid[2, 3] = False
    sm = _make_safe_map(grid)
    # cell (row=2, col=3) → world x=0.35, y=0.25 (centre)
    assert not is_point_safe(0.35, 0.25, sm)


def test_point_outside_grid_returns_false():
    grid = np.ones((10, 10), dtype=bool)
    sm = _make_safe_map(grid)
    assert not is_point_safe(-0.5, 0.05, sm)
    assert not is_point_safe(0.05, 5.0, sm)


# ──────────────────────────────────────────────────────────────────────────────
# is_segment_safe
# ──────────────────────────────────────────────────────────────────────────────

def test_segment_within_safe_area():
    grid = np.ones((20, 20), dtype=bool)
    sm = _make_safe_map(grid)
    assert is_segment_safe((0.05, 0.05), (1.95, 1.95), sm)


def test_segment_crossing_obstacle():
    """A diagonal segment that crosses a wall of unsafe cells."""
    grid = np.ones((20, 20), dtype=bool)
    # Vertical wall at col=10
    grid[:, 10] = False
    sm = _make_safe_map(grid)
    # p0 is left of wall, p1 is right of wall
    p0 = (0.5, 1.0)
    p1 = (1.5, 1.0)
    assert not is_segment_safe(p0, p1, sm)


def test_segment_along_safe_corridor():
    """Horizontal segment through a narrow safe corridor."""
    grid = np.zeros((10, 20), dtype=bool)
    grid[5, :] = True   # single safe row
    sm = _make_safe_map(grid)
    p0 = (0.05, 0.55)   # row=5, col=0
    p1 = (1.95, 0.55)   # row=5, col=19
    assert is_segment_safe(p0, p1, sm)


# ──────────────────────────────────────────────────────────────────────────────
# validate_path
# ──────────────────────────────────────────────────────────────────────────────

def test_validate_path_all_valid_rectangle():
    grid = np.ones((10, 10), dtype=bool)
    sm = _make_safe_map(grid)
    points = [(0.05, 0.05), (0.05, 0.95), (0.95, 0.95), (0.95, 0.05)]
    result = validate_path(points, sm)
    assert result.valid
    assert result.invalid_points == []
    assert result.invalid_segments == []


def test_validate_path_detects_crossing():
    """Segment that jumps across a wall should appear in invalid_segments."""
    grid = np.ones((20, 20), dtype=bool)
    grid[:, 10] = False   # vertical wall
    sm = _make_safe_map(grid)
    points = [
        (0.55, 1.05),    # left of wall, safe
        (1.45, 1.05),    # right of wall, safe
    ]
    result = validate_path(points, sm)
    assert not result.valid
    assert (0, 1) in result.invalid_segments


def test_validate_path_detects_unsafe_point():
    grid = np.ones((10, 10), dtype=bool)
    grid[5, 5] = False
    sm = _make_safe_map(grid)
    unsafe_world = (0.55, 0.55)   # cell row=5, col=5
    points = [(0.05, 0.05), unsafe_world, (0.95, 0.05)]
    result = validate_path(points, sm)
    assert not result.valid
    assert 1 in result.invalid_points


def test_validate_path_u_shape_no_crossing():
    """U-shape: paths inside each arm should be valid without crossing."""
    H, W = 30, 20
    grid = np.zeros((H, W), dtype=bool)
    # left arm: cols 2-7, rows 0-25
    grid[0:26, 2:8] = True
    # right arm: cols 12-17, rows 0-25
    grid[0:26, 12:18] = True
    sm = _make_safe_map(grid)

    # Path inside left arm only — should be valid
    left_arm_pts = [(0.25 + 0.05, row * RES + 0.05) for row in range(0, 26)]
    result = validate_path(left_arm_pts, sm)
    assert result.valid, result.message


def test_validate_path_u_shape_crossing_detected():
    """Connector between U-shape arms must be flagged as invalid."""
    H, W = 30, 20
    grid = np.zeros((H, W), dtype=bool)
    grid[0:26, 2:8] = True
    grid[0:26, 12:18] = True
    sm = _make_safe_map(grid)

    # One point in left arm, one in right arm — direct connector crosses empty space
    left_pt = (0.5, 1.05)    # inside left arm
    right_pt = (1.45, 1.05)  # inside right arm
    result = validate_path([left_pt, right_pt], sm)
    assert not result.valid
    assert (0, 1) in result.invalid_segments
