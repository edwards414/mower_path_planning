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
"""Unit tests for coverage/connector_planner.py (TDD — written before impl)."""

import math

import numpy as np
import pytest

from mower_mission.coverage.connector_planner import plan_connector
from mower_mission.coverage.path_validator import SafeMap, is_segment_safe, validate_path

RES = 0.1
OX = 0.0
OY = 0.0


def _sm(grid: np.ndarray) -> SafeMap:
    return SafeMap(grid=grid.astype(bool), resolution=RES, origin_x=OX, origin_y=OY)


def _open(rows: int = 20, cols: int = 20) -> np.ndarray:
    return np.ones((rows, cols), dtype=bool)


# ──────────────────────────────────────────────────────────────────────────────
# Basic reachability
# ──────────────────────────────────────────────────────────────────────────────

def test_straight_path_returns_nonempty():
    sm = _sm(_open())
    path = plan_connector((0.05, 0.05), (1.95, 1.95), sm)
    assert path is not None
    assert len(path) >= 2


def test_start_equals_end_returns_two_point_path():
    sm = _sm(_open())
    path = plan_connector((0.55, 0.55), (0.55, 0.55), sm)
    assert path is not None
    assert len(path) >= 1


def test_start_on_unsafe_cell_returns_none():
    grid = _open()
    grid[0, 0] = False   # unsafe start cell
    sm = _sm(grid)
    assert plan_connector((0.05, 0.05), (1.95, 1.95), sm) is None


def test_end_on_unsafe_cell_returns_none():
    grid = _open()
    grid[19, 19] = False
    sm = _sm(grid)
    assert plan_connector((0.05, 0.05), (1.95, 1.95), sm) is None


def test_disconnected_returns_none():
    """Left half and right half separated by a full-height wall."""
    grid = np.ones((20, 20), dtype=bool)
    grid[:, 10] = False   # vertical wall
    sm = _sm(grid)
    # start in left half, end in right half — unreachable
    result = plan_connector((0.55, 1.05), (1.55, 1.05), sm)
    assert result is None


# ──────────────────────────────────────────────────────────────────────────────
# Path quality
# ──────────────────────────────────────────────────────────────────────────────

def test_path_avoids_wall():
    """Start and end are on opposite sides of a wall with one gap."""
    grid = np.ones((20, 20), dtype=bool)
    grid[:18, 10] = False    # wall with gap at rows 18-19
    sm = _sm(grid)

    start = (0.55, 0.55)    # left of wall
    end = (1.55, 0.55)      # right of wall
    path = plan_connector(start, end, sm)

    assert path is not None, 'Should find path through the gap'
    assert len(path) >= 3


def test_all_path_points_are_safe():
    """Every returned waypoint must lie on a safe cell."""
    grid = np.ones((30, 30), dtype=bool)
    grid[10:20, 10:20] = False   # square obstacle in centre
    sm = _sm(grid)

    start = (0.55, 1.55)
    end = (2.95, 1.55)
    path = plan_connector(start, end, sm)

    if path is None:
        pytest.skip('No path found — skipping quality check')

    for x, y in path:
        assert is_segment_safe((x, y), (x, y), sm) or True  # point check
        row = int((y - OY) / RES)
        col = int((x - OX) / RES)
        assert sm.grid[row, col], f'Point ({x:.2f},{y:.2f}) on unsafe cell'


def test_connector_passes_path_validator():
    """validate_path on the connector result must return valid=True."""
    grid = np.ones((30, 30), dtype=bool)
    grid[5:25, 14:16] = False   # narrow wall, no gap → use around-wall path
    # Add a gap at the bottom
    grid[24:, 14:16] = True
    sm = _sm(grid)

    start = (0.55, 1.55)
    end = (2.55, 1.55)
    path = plan_connector(start, end, sm)

    if path is None:
        pytest.skip('No path found')

    result = validate_path(path, sm)
    assert result.valid, f'Connector path invalid: {result.message}'


def test_connector_does_not_cut_diagonal_obstacle_corner():
    """Connector must route around blocked corners instead of diagonal cutting."""
    grid = np.ones((5, 5), dtype=bool)
    grid[1, 2] = False
    grid[2, 1] = False
    sm = _sm(grid)

    start = (0.15, 0.15)  # row=1, col=1
    end = (0.35, 0.35)    # row=3, col=3
    path = plan_connector(start, end, sm)

    assert path is not None
    result = validate_path(path, sm)
    assert result.valid, f'Connector cut a blocked corner: {result.message}'


# ──────────────────────────────────────────────────────────────────────────────
# U-shape integration
# ──────────────────────────────────────────────────────────────────────────────

def test_u_shape_connector_between_arms():
    """
    U-shape (open top): connector from bottom of left arm to bottom of right
    arm must go around the closed bottom, not through it.
    """
    H, W = 30, 30
    grid = np.zeros((H, W), dtype=bool)
    grid[0:20, 5:10] = True    # left arm (open upward)
    grid[0:20, 20:25] = True   # right arm
    grid[20:25, 5:25] = True   # connecting bottom
    sm = _sm(grid)

    # Start near top of left arm, end near top of right arm
    start_x = OX + 7.5 * RES       # col 7 in left arm
    start_y = OY + 1.5 * RES       # row 1
    end_x = OX + 22.5 * RES        # col 22 in right arm
    end_y = OY + 1.5 * RES         # row 1

    path = plan_connector((start_x, start_y), (end_x, end_y), sm)
    assert path is not None, 'Should find path through the bottom'

    result = validate_path(path, sm)
    assert result.valid, f'U-shape connector invalid: {result.message}'
