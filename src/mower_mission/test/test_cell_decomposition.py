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
"""Unit tests for coverage/cell_decomposition.py (TDD — written before impl)."""

import numpy as np
import pytest

from mower_mission.coverage.cell_decomposition import decompose
from mower_mission.coverage.path_validator import SafeMap

RES = 0.1
OX = 0.0
OY = 0.0


def _sm(grid: np.ndarray) -> SafeMap:
    return SafeMap(grid=grid.astype(bool), resolution=RES, origin_x=OX, origin_y=OY)


def _rectangle(rows: int = 20, cols: int = 20) -> np.ndarray:
    return np.ones((rows, cols), dtype=bool)


# ──────────────────────────────────────────────────────────────────────────────
# Basic decomposition
# ──────────────────────────────────────────────────────────────────────────────

def test_rectangle_single_cell():
    """A full rectangle with no obstacles should produce exactly one cell."""
    sm = _sm(_rectangle())
    cells, graph = decompose(sm)
    assert len(cells) == 1


def test_empty_grid_no_cells():
    """An all-False grid should produce no cells and an empty graph."""
    sm = _sm(np.zeros((10, 10), dtype=bool))
    cells, graph = decompose(sm)
    assert len(cells) == 0
    assert len(graph) == 0


def test_cells_cover_safe_area():
    """Union of all cell masks must equal safe_map.grid exactly."""
    grid = np.ones((30, 30), dtype=bool)
    grid[10:20, 10:20] = False   # central hole
    sm = _sm(grid)
    cells, _ = decompose(sm)

    union = np.zeros_like(sm.grid, dtype=bool)
    for cell in cells:
        union |= cell.mask

    np.testing.assert_array_equal(union, sm.grid)


def test_cells_nonempty():
    """Every returned cell must have at least one True pixel and area > 0."""
    sm = _sm(_rectangle())
    cells, _ = decompose(sm)
    for cell in cells:
        assert cell.mask.any(), f'Cell {cell.cell_id} has empty mask'
        assert cell.area_m2 > 0


# ──────────────────────────────────────────────────────────────────────────────
# Critical events
# ──────────────────────────────────────────────────────────────────────────────

def test_obstacle_causes_multiple_cells():
    """An internal obstacle that splits free intervals must produce 3+ cells."""
    grid = np.ones((20, 20), dtype=bool)
    grid[5:15, 8:12] = False   # obstacle: rows 5-14, cols 8-11
    sm = _sm(grid)
    cells, _ = decompose(sm)
    assert len(cells) >= 3


def test_u_shape_has_multiple_cells():
    """Vertical U-shape: row sweep detects two separate arms → 2+ cells."""
    grid = np.zeros((30, 20), dtype=bool)
    grid[0:20, 0:5] = True      # left arm
    grid[0:20, 15:20] = True    # right arm
    grid[20:25, 0:20] = True    # bottom connecting bar
    sm = _sm(grid)
    cells, _ = decompose(sm)
    assert len(cells) >= 2


def test_disconnected_cells_not_adjacent():
    """Two separated blocks must each form their own cell with no edges."""
    grid = np.zeros((10, 20), dtype=bool)
    grid[:, 0:5] = True      # left block
    grid[:, 15:20] = True    # right block (gap cols 5-14)
    sm = _sm(grid)
    cells, graph = decompose(sm)
    assert len(cells) == 2
    for cid, neighbours in graph.items():
        assert neighbours == [], f'Cell {cid} has unexpected neighbours {neighbours}'


def test_obstacle_cells_are_adjacent():
    """Cells produced by a split/merge event must appear in each other's graph."""
    grid = np.ones((20, 20), dtype=bool)
    grid[5:15, 8:12] = False
    sm = _sm(grid)
    cells, graph = decompose(sm)
    total_edges = sum(len(v) for v in graph.values())
    assert total_edges > 0, 'No adjacency edges found after split/merge events'


# ──────────────────────────────────────────────────────────────────────────────
# Cell metadata
# ──────────────────────────────────────────────────────────────────────────────

def test_cell_bbox_contains_all_mask_pixels():
    """Every True pixel in a cell mask must lie within the cell's bbox."""
    grid = np.ones((20, 20), dtype=bool)
    grid[5:15, 8:12] = False
    sm = _sm(grid)
    cells, _ = decompose(sm)
    for cell in cells:
        rows, cols = np.where(cell.mask)
        if len(rows) == 0:
            continue
        rmin, cmin, rmax, cmax = cell.bbox
        assert rows.min() >= rmin, f'Cell {cell.cell_id}: row below bbox'
        assert rows.max() <= rmax, f'Cell {cell.cell_id}: row above bbox'
        assert cols.min() >= cmin, f'Cell {cell.cell_id}: col below bbox'
        assert cols.max() <= cmax, f'Cell {cell.cell_id}: col above bbox'


def test_cell_area_matches_pixel_count():
    """area_m2 must equal pixel_count × resolution²."""
    sm = _sm(_rectangle(10, 10))
    cells, _ = decompose(sm)
    for cell in cells:
        expected = float(cell.mask.sum()) * RES * RES
        assert abs(cell.area_m2 - expected) < 1e-9, (
            f'Cell {cell.cell_id}: area {cell.area_m2:.6f} != expected {expected:.6f}'
        )


def test_cell_centroid_within_world_bbox():
    """Centroid (x, y) must lie within the world-coordinate extent of the cell bbox."""
    grid = np.ones((30, 30), dtype=bool)
    grid[10:20, 10:20] = False
    sm = _sm(grid)
    cells, _ = decompose(sm)
    for cell in cells:
        rmin, cmin, rmax, cmax = cell.bbox
        x_lo = OX + cmin * RES
        x_hi = OX + (cmax + 1) * RES
        y_lo = OY + rmin * RES
        y_hi = OY + (rmax + 1) * RES
        cx, cy = cell.centroid_xy
        assert x_lo <= cx <= x_hi, (
            f'Cell {cell.cell_id} centroid x={cx:.3f} outside [{x_lo:.3f}, {x_hi:.3f}]'
        )
        assert y_lo <= cy <= y_hi, (
            f'Cell {cell.cell_id} centroid y={cy:.3f} outside [{y_lo:.3f}, {y_hi:.3f}]'
        )


def test_single_cell_empty_graph():
    """The cell graph for a single-cell decomposition should be empty."""
    sm = _sm(_rectangle())
    cells, graph = decompose(sm)
    assert len(cells) == 1
    assert graph[cells[0].cell_id] == []
