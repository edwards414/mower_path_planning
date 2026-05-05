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
"""Onion-layer spiral coverage path generator with connected-component bridging.

Public API
----------
plan_spiral_coverage(...)  -> SpiralCoveragePlan   (rich output)
_generate_coverage_spiral_path(...)  -> tuple[list, list, list]  (3-tuple wrapper)
"""
from __future__ import annotations

import heapq
from collections import deque

import numpy as np

from ..coverage.path_validator import SafeMap, validate_path
from ..coverage.types import SpiralCoveragePlan, SpiralSegment


# ── Public API ────────────────────────────────────────────────────────────────

def plan_spiral_coverage(
    safe_map: np.ndarray,
    strip_width_m: float,
    waypoint_spacing_m: float,
    res: float,
    H: int,
    W: int,
    origin_x: float,
    origin_y: float,
) -> SpiralCoveragePlan:
    """Onion-layer spiral over each connected component, bridged by A*.

    Returns a SpiralCoveragePlan with typed segments, split_points,
    invalid_segments, coverage_mask, and debug metadata.
    """
    safe_map = safe_map.astype(bool, copy=False)
    empty_plan = SpiralCoveragePlan(
        points=[], segments=[], split_points=[], invalid_segments=[],
        coverage_mask=np.zeros((H, W), dtype=bool),
        debug={'component_count': 0, 'bridge_count': 0},
    )
    if not np.any(safe_map):
        return empty_plan

    components = _connected_components(safe_map, H, W)
    segments: list[SpiralSegment] = []
    coverage_mask = np.zeros((H, W), dtype=bool)
    coverage_radius_cells = max(0, int(np.ceil(strip_width_m / (2.0 * res))))

    for comp_id, comp_mask in enumerate(components):
        pts = _onion_layer_spiral(
            comp_mask, strip_width_m, waypoint_spacing_m, res, H, W, origin_x, origin_y,
        )
        if not pts:
            continue

        previous_coverage_cell: tuple[int, int] | None = None
        for x, y in pts:
            r, c = _world_to_cell(x, y, res, origin_x, origin_y)
            if previous_coverage_cell is None:
                line_cells = [(r, c)]
            else:
                line_cells = _line_cells(previous_coverage_cell, (r, c))

            for lr, lc in line_cells:
                _mark_covered_cells(
                    coverage_mask,
                    safe_map,
                    lr,
                    lc,
                    coverage_radius_cells,
                )
            previous_coverage_cell = (r, c)

        segments.append(SpiralSegment(pts, 'spiral', comp_id, True))

        if comp_id + 1 < len(components):
            last_pt = pts[-1]
            goal_rc = _nearest_entry(last_pt, components[comp_id + 1], res, origin_x, origin_y)
            start_rc = _world_to_cell(last_pt[0], last_pt[1], res, origin_x, origin_y)
            bridge_cells = _astar(start_rc, goal_rc, safe_map, H, W)
            if bridge_cells:
                bridge_pts = [_cell_to_world(r, c, res, origin_x, origin_y) for r, c in bridge_cells]
                segments.append(SpiralSegment(bridge_pts, 'bridge', comp_id, True))
            else:
                goal_pt = _cell_to_world(goal_rc[0], goal_rc[1], res, origin_x, origin_y)
                segments.append(SpiralSegment(
                    [last_pt, goal_pt], 'invalid', comp_id, False, 'no_safe_bridge',
                ))

    all_points = [p for seg in segments for p in seg.points]
    split_points = [
        seg.points[-1] for seg in segments
        if seg.segment_type == 'spiral' and seg.points
    ]
    invalid_segs = _find_invalid_segs(all_points, safe_map, res, origin_x, origin_y)
    bridge_count = sum(1 for s in segments if s.segment_type == 'bridge')

    return SpiralCoveragePlan(
        points=all_points,
        segments=segments,
        split_points=split_points,
        invalid_segments=invalid_segs,
        coverage_mask=coverage_mask,
        debug={'component_count': len(components), 'bridge_count': bridge_count},
    )


def _generate_coverage_spiral_path(
    safe_map: np.ndarray,
    strip_width_m: float,
    waypoint_spacing_m: float,
    res: float,
    H: int,
    W: int,
    origin_x: float,
    origin_y: float,
) -> tuple[list[tuple[float, float]], list[tuple[float, float]], list[tuple[int, int]]]:
    """3-tuple wrapper around plan_spiral_coverage for backward compatibility."""
    plan = plan_spiral_coverage(
        safe_map, strip_width_m, waypoint_spacing_m, res, H, W, origin_x, origin_y,
    )
    return plan.points, plan.split_points, plan.invalid_segments


# ── Per-component spiral ──────────────────────────────────────────────────────

def _onion_layer_spiral(
    comp_mask: np.ndarray,
    strip_width_m: float,
    waypoint_spacing_m: float,
    res: float,
    H: int,
    W: int,
    origin_x: float,
    origin_y: float,
) -> list[tuple[float, float]]:
    """Onion-layer spiral for a single connected-component mask.

    Returns the flat waypoint list only (no split/invalid; those are computed
    at the plan level across all components).
    """
    comp_mask = comp_mask.astype(bool, copy=False)
    if not np.any(comp_mask):
        return []

    dist = _bfs_dist(comp_mask, H, W)
    layer_step = max(1, int(round(strip_width_m / res)))
    # Spiral centerlines are spaced by strip_width_m, so points along one
    # centerline do not need to be denser than the strip width.
    point_spacing_m = max(waypoint_spacing_m, strip_width_m)
    spacing_cells = max(1, int(round(point_spacing_m / res)))
    max_dist = int(dist[comp_mask].max())

    points: list[tuple[float, float]] = []
    previous_cell: tuple[int, int] | None = None

    for contour_mask in _iter_spiral_centerlines(
        comp_mask,
        dist,
        layer_step,
        max_dist,
    ):
        contour_parts = _connected_components(contour_mask, H, W, diagonal=True)
        contour_parts = _sort_layer_parts(contour_parts, previous_cell)

        for part_mask in contour_parts:
            contour_cells = np.argwhere(part_mask)
            ordered_cells = _order_cells_by_continuity(
                contour_cells,
                previous_cell,
            )
            sampled_cells = _sample_cells_by_spacing(
                ordered_cells,
                spacing_cells,
            )

            for r, c in sampled_cells:
                points.append((
                    origin_x + (c + 0.5) * res,
                    origin_y + (r + 0.5) * res,
                ))
                previous_cell = (int(r), int(c))

    return points


# ── Connected components ──────────────────────────────────────────────────────

def _connected_components(
    safe_map: np.ndarray,
    H: int,
    W: int,
    diagonal: bool = False,
) -> list[np.ndarray]:
    """BFS flood-fill. Returns list of bool masks sorted by area descending."""
    visited = np.zeros((H, W), dtype=bool)
    components: list[np.ndarray] = []
    neighbours = (
        (
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1),
        )
        if diagonal
        else ((-1, 0), (1, 0), (0, -1), (0, 1))
    )

    for r in range(H):
        for c in range(W):
            if safe_map[r, c] and not visited[r, c]:
                comp = np.zeros((H, W), dtype=bool)
                dq: deque[tuple[int, int]] = deque([(r, c)])
                visited[r, c] = True
                while dq:
                    cr, cc = dq.popleft()
                    comp[cr, cc] = True
                    for dr, dc in neighbours:
                        nr, nc = cr + dr, cc + dc
                        if 0 <= nr < H and 0 <= nc < W and safe_map[nr, nc] and not visited[nr, nc]:
                            visited[nr, nc] = True
                            dq.append((nr, nc))
                components.append(comp)

    components.sort(key=lambda m: int(m.sum()), reverse=True)
    return components


# ── Layer ordering ────────────────────────────────────────────────────────────

def _iter_spiral_centerlines(
    comp_mask: np.ndarray,
    dist: np.ndarray,
    layer_step: int,
    max_dist: int,
) -> list[np.ndarray]:
    """Return one-cell centerlines for each inward spiral layer.

    Earlier versions traversed every cell in a layer band.  That made RViz show
    a dense block of arrows.  A mower path should follow the strip centerline;
    the strip width accounts for the covered area around that line.
    """
    contour_masks: list[np.ndarray] = []
    used_distances: set[int] = set()

    for layer_start in range(0, max_dist + 1, layer_step):
        layer_end = min(layer_start + layer_step - 1, max_dist)
        target = (layer_start + layer_end) // 2
        band_distances = np.unique(dist[(dist >= layer_start) & (dist <= layer_end)])
        band_distances = band_distances[band_distances >= 0]
        if len(band_distances) == 0:
            continue

        target_dist = int(
            band_distances[np.argmin(np.abs(band_distances - target))]
        )
        if target_dist in used_distances:
            continue
        used_distances.add(target_dist)

        contour_mask = (dist == target_dist) & comp_mask
        if np.any(contour_mask):
            contour_masks.append(contour_mask)

    return contour_masks


def _sort_layer_parts(
    layer_parts: list[np.ndarray],
    previous_cell: tuple[int, int] | None,
) -> list[np.ndarray]:
    """Order disconnected pieces of one layer.

    With no previous cell, use the largest loop first.  Once a layer has an
    exit cell, visit the nearest next loop so inter-loop transitions stay short.
    """
    if previous_cell is None:
        return sorted(layer_parts, key=lambda m: int(m.sum()), reverse=True)

    pr, pc = previous_cell

    def distance_to_previous(mask: np.ndarray) -> int:
        cells = np.argwhere(mask)
        dists = np.abs(cells[:, 0] - pr) + np.abs(cells[:, 1] - pc)
        return int(dists.min())

    return sorted(layer_parts, key=distance_to_previous)


def _order_cells_by_continuity(
    cells: np.ndarray,
    previous_cell: tuple[int, int] | None,
) -> list[tuple[int, int]]:
    """Order contour cells by walking to neighbouring cells first."""
    if len(cells) == 0:
        return []

    remaining: set[tuple[int, int]] = {
        (int(r), int(c)) for r, c in cells
    }
    if previous_cell is None:
        start = min(remaining)
    else:
        start = min(
            remaining,
            key=lambda rc: _cell_dist(rc, previous_cell),
        )

    ordered: list[tuple[int, int]] = [start]
    remaining.remove(start)
    current = start

    while remaining:
        neighbours = [
            (current[0] + dr, current[1] + dc)
            for dr, dc in (
                (-1, 0), (0, 1), (1, 0), (0, -1),
                (-1, 1), (1, 1), (1, -1), (-1, -1),
            )
            if (current[0] + dr, current[1] + dc) in remaining
        ]
        if neighbours:
            next_cell = min(
                neighbours,
                key=lambda rc: (_turn_cost(ordered, rc), rc[0], rc[1]),
            )
        else:
            next_cell = min(
                remaining,
                key=lambda rc: (_cell_dist(rc, current), rc[0], rc[1]),
            )

        ordered.append(next_cell)
        remaining.remove(next_cell)
        current = next_cell

    return ordered


def _cell_dist(a: tuple[int, int], b: tuple[int, int]) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _turn_cost(
    ordered: list[tuple[int, int]],
    candidate: tuple[int, int],
) -> int:
    if len(ordered) < 2:
        return 0
    r0, c0 = ordered[-2]
    r1, c1 = ordered[-1]
    r2, c2 = candidate
    prev = (r1 - r0, c1 - c0)
    nxt = (r2 - r1, c2 - c1)
    return 0 if prev == nxt else 1


def _sample_cells_by_spacing(
    ordered_cells: list[tuple[int, int]],
    spacing_cells: int,
) -> list[tuple[int, int]]:
    """Subsample ordered cells by grid distance while keeping the layer end."""
    if not ordered_cells:
        return []

    sampled = [ordered_cells[0]]
    last_r, last_c = ordered_cells[0]
    for r, c in ordered_cells[1:]:
        dist = abs(r - last_r) + abs(c - last_c)
        if dist >= spacing_cells:
            sampled.append((r, c))
            last_r, last_c = r, c

    if sampled[-1] != ordered_cells[-1]:
        sampled.append(ordered_cells[-1])

    return sampled


def _mark_covered_cells(
    coverage_mask: np.ndarray,
    safe_map: np.ndarray,
    row: int,
    col: int,
    radius_cells: int,
) -> None:
    """Mark cells covered by a mower strip centred on one waypoint."""
    H, W = safe_map.shape
    r0 = max(0, row - radius_cells)
    r1 = min(H - 1, row + radius_cells)
    c0 = max(0, col - radius_cells)
    c1 = min(W - 1, col + radius_cells)
    radius_sq = radius_cells * radius_cells

    for r in range(r0, r1 + 1):
        for c in range(c0, c1 + 1):
            if not safe_map[r, c]:
                continue
            if radius_cells == 0 or (r - row) ** 2 + (c - col) ** 2 <= radius_sq:
                coverage_mask[r, c] = True


def _line_cells(
    start: tuple[int, int],
    end: tuple[int, int],
) -> list[tuple[int, int]]:
    """Integer grid cells touched by a straight segment."""
    r0, c0 = start
    r1, c1 = end
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


# ── A* bridge ────────────────────────────────────────────────────────────────

def _astar(
    start_rc: tuple[int, int],
    goal_rc: tuple[int, int],
    safe_map: np.ndarray,
    H: int,
    W: int,
) -> list[tuple[int, int]] | None:
    """Grid A* on safe_map. Manhattan heuristic. Returns cell path or None."""
    sr, sc = start_rc
    gr, gc = goal_rc

    if (sr, sc) == (gr, gc):
        return [(sr, sc)]

    def h(r: int, c: int) -> int:
        return abs(r - gr) + abs(c - gc)

    open_heap: list = [(h(sr, sc), 0, sr, sc)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score: dict[tuple[int, int], int] = {(sr, sc): 0}

    while open_heap:
        _, cost, r, c = heapq.heappop(open_heap)

        if (r, c) == (gr, gc):
            path: list[tuple[int, int]] = []
            cur: tuple[int, int] = (r, c)
            while cur in came_from:
                path.append(cur)
                cur = came_from[cur]
            path.append(start_rc)
            return list(reversed(path))

        if cost > g_score.get((r, c), float('inf')):
            continue

        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < H and 0 <= nc < W and safe_map[nr, nc]:
                ng = cost + 1
                if ng < g_score.get((nr, nc), float('inf')):
                    g_score[(nr, nc)] = ng
                    came_from[(nr, nc)] = (r, c)
                    heapq.heappush(open_heap, (ng + h(nr, nc), ng, nr, nc))

    return None


# ── Coordinate helpers ────────────────────────────────────────────────────────

def _world_to_cell(
    x: float, y: float, res: float, origin_x: float, origin_y: float,
) -> tuple[int, int]:
    return int((y - origin_y) / res), int((x - origin_x) / res)


def _cell_to_world(
    r: int, c: int, res: float, origin_x: float, origin_y: float,
) -> tuple[float, float]:
    return origin_x + (c + 0.5) * res, origin_y + (r + 0.5) * res


def _nearest_entry(
    last_pt: tuple[float, float],
    comp_mask: np.ndarray,
    res: float,
    origin_x: float,
    origin_y: float,
) -> tuple[int, int]:
    """Nearest cell in comp_mask to last_pt by Manhattan distance."""
    lr, lc = _world_to_cell(last_pt[0], last_pt[1], res, origin_x, origin_y)
    cells = np.argwhere(comp_mask)
    dists = np.abs(cells[:, 0] - lr) + np.abs(cells[:, 1] - lc)
    idx = int(np.argmin(dists))
    return (int(cells[idx, 0]), int(cells[idx, 1]))


# ── BFS distance transform ────────────────────────────────────────────────────

def _bfs_dist(safe_map: np.ndarray, H: int, W: int) -> np.ndarray:
    """BFS distance from each safe cell to the nearest boundary.

    Boundary = unsafe cell or grid edge.  Returns int array; unsafe cells
    keep their initial value of -1.
    """
    dist = np.full((H, W), -1, dtype=int)
    dq: deque[tuple[int, int]] = deque()

    for r in range(H):
        for c in range(W):
            if not safe_map[r, c]:
                continue
            on_boundary = (
                r == 0 or r == H - 1 or c == 0 or c == W - 1
                or not safe_map[r - 1, c]
                or not safe_map[r + 1, c]
                or not safe_map[r, c - 1]
                or not safe_map[r, c + 1]
            )
            if on_boundary:
                dist[r, c] = 0
                dq.append((r, c))

    while dq:
        r, c = dq.popleft()
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < H and 0 <= nc < W and safe_map[nr, nc] and dist[nr, nc] == -1:
                dist[nr, nc] = dist[r, c] + 1
                dq.append((nr, nc))

    return dist


# ── Invalid segment detection ─────────────────────────────────────────────────

def _find_invalid_segs(
    points: list[tuple[float, float]],
    safe_map: np.ndarray,
    res: float,
    origin_x: float,
    origin_y: float,
) -> list[tuple[int, int]]:
    if len(points) < 2:
        return []
    sm = SafeMap(
        grid=safe_map.astype(bool),
        resolution=res,
        origin_x=origin_x,
        origin_y=origin_y,
    )
    return validate_path(points, sm).invalid_segments
