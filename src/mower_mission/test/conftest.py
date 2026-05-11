"""Shared pytest fixtures for coverage path planning tests."""

import numpy as np
import pytest

from mower_mission.coverage.path_validator import SafeMap

RES = 0.1
OX = 0.0
OY = 0.0


def _make_safe_map(grid: np.ndarray, res: float = RES,
                   ox: float = OX, oy: float = OY) -> SafeMap:
    return SafeMap(grid=grid.astype(bool), resolution=res, origin_x=ox, origin_y=oy)


# ── Grid fixtures ──────────────────────────────────────────────────────────────

@pytest.fixture
def small_open_grid():
    """20×20 fully traversable grid."""
    return np.ones((20, 20), dtype=bool)


@pytest.fixture
def small_open_safe_map(small_open_grid):
    return _make_safe_map(small_open_grid)


@pytest.fixture
def u_shape_grid():
    """30×20 U-shaped region: two vertical arms + connecting bottom.

    Left arm:  cols 2-7, rows 0-25
    Right arm: cols 12-17, rows 0-25
    Bottom:    cols 5-15, rows 26-29
    """
    grid = np.zeros((30, 20), dtype=bool)
    grid[0:26, 2:8] = True
    grid[0:26, 12:18] = True
    grid[26:30, 2:18] = True
    return grid


@pytest.fixture
def u_shape_safe_map(u_shape_grid):
    return _make_safe_map(u_shape_grid)


@pytest.fixture
def two_island_grid():
    """20×30 grid with two disconnected rectangular components."""
    grid = np.zeros((20, 30), dtype=bool)
    grid[2:8, 2:10] = True    # island A
    grid[12:18, 20:28] = True  # island B
    return grid


@pytest.fixture
def two_island_safe_map(two_island_grid):
    return _make_safe_map(two_island_grid)


@pytest.fixture
def rectangle_obstacle_grid():
    """20×20 open grid with a 4×4 obstacle block in the centre."""
    grid = np.ones((20, 20), dtype=bool)
    grid[8:12, 8:12] = False
    return grid


@pytest.fixture
def rectangle_obstacle_safe_map(rectangle_obstacle_grid):
    return _make_safe_map(rectangle_obstacle_grid)


@pytest.fixture
def narrow_passage_grid():
    """20×20 grid split by a vertical wall with a 2-cell gap at the bottom."""
    grid = np.ones((20, 20), dtype=bool)
    grid[:18, 10] = False   # wall with gap at rows 18-19
    return grid


@pytest.fixture
def narrow_passage_safe_map(narrow_passage_grid):
    return _make_safe_map(narrow_passage_grid)
