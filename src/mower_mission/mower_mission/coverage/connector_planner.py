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
"""Grid A* connector planner for safe inter-strip transitions."""

import heapq
import math
from collections import deque

import numpy as np

from .path_validator import SafeMap, _world_to_grid, is_point_safe

# 8-connected neighbours: (Δrow, Δcol, move_cost)
_NEIGHBOURS = [
    (-1,  0, 1.0),
    ( 1,  0, 1.0),
    ( 0, -1, 1.0),
    ( 0,  1, 1.0),
    (-1, -1, math.sqrt(2)),
    (-1,  1, math.sqrt(2)),
    ( 1, -1, math.sqrt(2)),
    ( 1,  1, math.sqrt(2)),
]


def plan_connector(
    start: tuple[float, float],
    end: tuple[float, float],
    safe_map: SafeMap,
    boundary_weight: float = 0.2,
) -> list[tuple[float, float]] | None:
    """Find a safe boundary-aware path from start to end on safe_map.

    Returns a list of world-coordinate (x, y) waypoints, or None when
    no safe path exists (unreachable or start/end on unsafe cell).

    The route is constrained to safe cells. Diagonal corner-cutting is blocked,
    and an extra cost favours cells near the traversable boundary so connector
    motion follows the shape of the safe region instead of jumping across it.
    """
    if not is_point_safe(start[0], start[1], safe_map):
        return None
    if not is_point_safe(end[0], end[1], safe_map):
        return None

    r0, c0 = _world_to_grid(start[0], start[1], safe_map)
    r1, c1 = _world_to_grid(end[0], end[1], safe_map)
    H, W = safe_map.grid.shape

    # Clamp to grid bounds (world_to_grid can be off by 1 at edges)
    r0, c0 = _clamp(r0, c0, H, W)
    r1, c1 = _clamp(r1, c1, H, W)

    if r0 == r1 and c0 == c1:
        return [start, end]

    boundary_dist = _boundary_distance(safe_map.grid)

    def h(r: int, c: int) -> float:
        return math.hypot(r1 - r, c1 - c)

    # (f, g, row, col)
    open_heap: list[tuple[float, float, int, int]] = []
    heapq.heappush(open_heap, (h(r0, c0), 0.0, r0, c0))

    g_score: dict[tuple[int, int], float] = {(r0, c0): 0.0}
    came_from: dict[tuple[int, int], tuple[int, int]] = {}

    while open_heap:
        _f, g, r, c = heapq.heappop(open_heap)

        if (r, c) == (r1, c1):
            return _reconstruct(came_from, r0, c0, r1, c1, safe_map)

        if g > g_score.get((r, c), float('inf')):
            continue

        for dr, dc, cost in _NEIGHBOURS:
            nr, nc = r + dr, c + dc
            if nr < 0 or nr >= H or nc < 0 or nc >= W:
                continue
            if not safe_map.grid[nr, nc]:
                continue
            if _cuts_corner(r, c, nr, nc, safe_map.grid):
                continue

            boundary_cost = boundary_weight * min(boundary_dist[nr, nc], 8.0)
            new_g = g + cost + boundary_cost
            if new_g < g_score.get((nr, nc), float('inf')):
                g_score[(nr, nc)] = new_g
                came_from[(nr, nc)] = (r, c)
                heapq.heappush(open_heap, (new_g + h(nr, nc), new_g, nr, nc))

    return None  # unreachable


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def _clamp(r: int, c: int, H: int, W: int) -> tuple[int, int]:
    return max(0, min(r, H - 1)), max(0, min(c, W - 1))


def _cuts_corner(
    r0: int,
    c0: int,
    r1: int,
    c1: int,
    grid: np.ndarray,
) -> bool:
    """Return True when a diagonal step squeezes through blocked corners."""
    dr = r1 - r0
    dc = c1 - c0
    if abs(dr) != 1 or abs(dc) != 1:
        return False
    return not (grid[r0, c1] and grid[r1, c0])


def _boundary_distance(grid: np.ndarray) -> np.ndarray:
    """Distance in cells from each safe cell to nearest safe-region boundary."""
    H, W = grid.shape
    dist = np.full((H, W), np.inf, dtype=float)
    q: deque[tuple[int, int]] = deque()

    for r in range(H):
        for c in range(W):
            if not grid[r, c]:
                continue
            if _is_boundary_cell(r, c, grid):
                dist[r, c] = 0.0
                q.append((r, c))

    while q:
        r, c = q.popleft()
        for dr, dc, _cost in _NEIGHBOURS[:4]:
            nr, nc = r + dr, c + dc
            if nr < 0 or nr >= H or nc < 0 or nc >= W:
                continue
            if not grid[nr, nc]:
                continue
            nd = dist[r, c] + 1.0
            if nd < dist[nr, nc]:
                dist[nr, nc] = nd
                q.append((nr, nc))

    dist[~grid] = np.inf
    return dist


def _is_boundary_cell(r: int, c: int, grid: np.ndarray) -> bool:
    H, W = grid.shape
    for dr, dc, _cost in _NEIGHBOURS[:4]:
        nr, nc = r + dr, c + dc
        if nr < 0 or nr >= H or nc < 0 or nc >= W:
            return True
        if not grid[nr, nc]:
            return True
    return False


def _reconstruct(
    came_from: dict,
    r0: int, c0: int,
    r1: int, c1: int,
    safe_map: SafeMap,
) -> list[tuple[float, float]]:
    """Walk came_from backwards to rebuild the path as world coordinates."""
    cells: list[tuple[int, int]] = []
    cur = (r1, c1)
    while cur in came_from:
        cells.append(cur)
        cur = came_from[cur]
    cells.append((r0, c0))
    cells.reverse()

    res = safe_map.resolution
    ox = safe_map.origin_x
    oy = safe_map.origin_y
    return [
        (ox + (col + 0.5) * res, oy + (row + 0.5) * res)
        for row, col in cells
    ]
