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
"""Path safety validation for coverage planning."""

from dataclasses import dataclass, field

import numpy as np


@dataclass
class SafeMap:
    """Occupancy grid with spatial metadata."""

    grid: np.ndarray      # bool, True = safe/traversable
    resolution: float     # metres per cell
    origin_x: float       # world X of grid[0, 0] centre
    origin_y: float       # world Y of grid[0, 0] centre
    frame_id: str = 'map'


@dataclass
class ValidationResult:
    """Result of a path validation pass."""

    valid: bool
    invalid_points: list = field(default_factory=list)    # indices into points[]
    invalid_segments: list = field(default_factory=list)  # (i, i+1) index pairs
    message: str = ''


# ──────────────────────────────────────────────────────────────────────────────
# Internal helpers
# ──────────────────────────────────────────────────────────────────────────────

def _world_to_grid(x: float, y: float, safe_map: SafeMap) -> tuple[int, int]:
    """Convert a world (x, y) to grid (row, col)."""
    col = int((x - safe_map.origin_x) / safe_map.resolution)
    row = int((y - safe_map.origin_y) / safe_map.resolution)
    return row, col


def _bresenham(r0: int, c0: int, r1: int, c1: int) -> list[tuple[int, int]]:
    """All integer grid cells on the line from (r0,c0) to (r1,c1)."""
    cells: list[tuple[int, int]] = []
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    r, c = r0, c0
    sr = 1 if r1 > r0 else -1
    sc = 1 if c1 > c0 else -1
    if dc > dr:
        err = dc // 2
        while c != c1:
            cells.append((r, c))
            err -= dr
            if err < 0:
                r += sr
                err += dc
            c += sc
    else:
        err = dr // 2
        while r != r1:
            cells.append((r, c))
            err -= dc
            if err < 0:
                c += sc
                err += dr
            r += sr
    cells.append((r1, c1))
    return cells


# ──────────────────────────────────────────────────────────────────────────────
# Public API
# ──────────────────────────────────────────────────────────────────────────────

def is_point_safe(x: float, y: float, safe_map: SafeMap) -> bool:
    """Return True if world point lies on a safe grid cell."""
    row, col = _world_to_grid(x, y, safe_map)
    H, W = safe_map.grid.shape
    if row < 0 or row >= H or col < 0 or col >= W:
        return False
    return bool(safe_map.grid[row, col])


def is_segment_safe(
    p0: tuple[float, float],
    p1: tuple[float, float],
    safe_map: SafeMap,
) -> bool:
    """Return True if every cell on the straight line p0→p1 is safe.

    Uses Bresenham rasterisation to enumerate all crossed cells.
    """
    r0, c0 = _world_to_grid(p0[0], p0[1], safe_map)
    r1, c1 = _world_to_grid(p1[0], p1[1], safe_map)
    H, W = safe_map.grid.shape
    for r, c in _bresenham(r0, c0, r1, c1):
        if r < 0 or r >= H or c < 0 or c >= W:
            return False
        if not safe_map.grid[r, c]:
            return False
    return True


def validate_points(
    points: list[tuple[float, float]],
    safe_map: SafeMap,
) -> ValidationResult:
    """Check that every waypoint lies on a safe cell."""
    invalid_points = [
        i for i, (x, y) in enumerate(points)
        if not is_point_safe(x, y, safe_map)
    ]
    return ValidationResult(
        valid=len(invalid_points) == 0,
        invalid_points=invalid_points,
        message=f'{len(invalid_points)} unsafe point(s)',
    )


def validate_path(
    points: list[tuple[float, float]],
    safe_map: SafeMap,
) -> ValidationResult:
    """Check every waypoint and every consecutive-pair segment.

    A segment (i, i+1) is unsafe when the Bresenham rasterisation of
    the straight line between them crosses at least one non-safe cell.
    This catches connectors that jump across boundaries or risk zones
    even when both endpoints are individually safe.
    """
    invalid_points = [
        i for i, (x, y) in enumerate(points)
        if not is_point_safe(x, y, safe_map)
    ]
    invalid_segments = [
        (i, i + 1)
        for i in range(len(points) - 1)
        if not is_segment_safe(points[i], points[i + 1], safe_map)
    ]
    valid = not invalid_points and not invalid_segments
    return ValidationResult(
        valid=valid,
        invalid_points=invalid_points,
        invalid_segments=invalid_segments,
        message=(
            f'{len(invalid_points)} unsafe point(s), '
            f'{len(invalid_segments)} unsafe segment(s)'
        ),
    )
