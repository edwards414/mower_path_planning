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
            comp_mask, strip_width_m, waypoint_spacing_m,
            res, H, W, origin_x, origin_y,
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
            # If A* bridge fails (genuinely disconnected regions after filter),
            # emit nothing.  Adding [last_pt, goal_pt] as an invalid segment
            # only creates a known-unresolvable segment that aborts planning.

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
        safe_map, strip_width_m, waypoint_spacing_m,
        res, H, W, origin_x, origin_y,
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
    spacing_cells = max(1, int(round(waypoint_spacing_m / res)))
    max_dist = int(dist[comp_mask].max())

    path_cells: list[tuple[int, int]] = []
    previous_cell: tuple[int, int] | None = None

    for layer_mask in _iter_spiral_layer_masks(
        comp_mask,
        dist,
        layer_step,
        max_dist,
    ):
        boundary_loops = _boundary_cell_loops(layer_mask, H, W)
        boundary_loops = _sort_cell_paths(boundary_loops, previous_cell)

        for loop_cells in boundary_loops:
            ordered_cells = _rotate_path_near_previous(
                loop_cells,
                previous_cell,
            )
            sampled_cells = _sample_cells_by_spacing(
                ordered_cells,
                spacing_cells,
                comp_mask,
            )
            if not sampled_cells:
                continue

            if previous_cell is not None and previous_cell != sampled_cells[0]:
                bridge_cells = _astar(
                    previous_cell, sampled_cells[0], comp_mask, H, W,
                )
                if bridge_cells:
                    _append_cell_path(path_cells, bridge_cells)

            _append_cell_path(path_cells, sampled_cells)
            previous_cell = sampled_cells[-1]

    return [
        (
            origin_x + (c + 0.5) * res,
            origin_y + (r + 0.5) * res,
        )
        for r, c in path_cells
    ]


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

def _iter_spiral_layer_masks(
    comp_mask: np.ndarray,
    dist: np.ndarray,
    layer_step: int,
    max_dist: int,
) -> list[np.ndarray]:
    """Return inward offset masks for each spiral layer.

    The path follows the boundary of each eroded mask, which gives an ordered
    offset contour.  This avoids the long greedy jumps that happen when all
    cells with the same distance value are sorted as an unordered cloud.
    """
    layer_masks: list[np.ndarray] = []
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

        layer_mask = (dist >= target_dist) & comp_mask
        if np.any(layer_mask):
            layer_masks.append(layer_mask)

    return layer_masks


def _boundary_cell_loops(
    mask: np.ndarray,
    H: int,
    W: int,
) -> list[list[tuple[int, int]]]:
    """Trace ordered safe-cell loops around the boundary of a bool mask."""
    edges_by_start: dict[
        tuple[int, int],
        list[tuple[tuple[int, int], tuple[int, int]]],
    ] = {}
    all_edges: list[
        tuple[tuple[int, int], tuple[int, int], tuple[int, int]]
    ] = []

    def is_inside(row: int, col: int) -> bool:
        return 0 <= row < H and 0 <= col < W and bool(mask[row, col])

    def add_edge(
        start: tuple[int, int],
        end: tuple[int, int],
        inside_cell: tuple[int, int],
    ) -> None:
        all_edges.append((start, end, inside_cell))
        edges_by_start.setdefault(start, []).append((end, inside_cell))

    for r, c in np.argwhere(mask):
        r = int(r)
        c = int(c)
        if not is_inside(r - 1, c):
            add_edge((r, c), (r, c + 1), (r, c))
        if not is_inside(r, c + 1):
            add_edge((r, c + 1), (r + 1, c + 1), (r, c))
        if not is_inside(r + 1, c):
            add_edge((r + 1, c + 1), (r + 1, c), (r, c))
        if not is_inside(r, c - 1):
            add_edge((r + 1, c), (r, c), (r, c))

    visited: set[tuple[tuple[int, int], tuple[int, int]]] = set()
    loops: list[list[tuple[int, int]]] = []

    for start, end, inside_cell in all_edges:
        edge_key = (start, end)
        if edge_key in visited:
            continue

        first_start = start
        current_start = start
        current_end = end
        current_cell = inside_cell
        loop_cells: list[tuple[int, int]] = []

        while (current_start, current_end) not in visited:
            visited.add((current_start, current_end))
            loop_cells.append(current_cell)

            candidates = [
                (next_end, next_cell)
                for next_end, next_cell in edges_by_start.get(current_end, [])
                if (current_end, next_end) not in visited
            ]
            if not candidates:
                break

            current_start, previous_end = current_end, current_start
            current_end, current_cell = _choose_next_boundary_edge(
                previous_end,
                current_start,
                candidates,
            )
            if current_start == first_start and current_end == end:
                break

        loop = _dedupe_consecutive_cells(loop_cells)
        if len(loop) > 1 and loop[-1] != loop[0]:
            loop.append(loop[0])
        if loop:
            loops.append(loop)

    return loops


def _choose_next_boundary_edge(
    previous_vertex: tuple[int, int],
    current_vertex: tuple[int, int],
    candidates: list[tuple[tuple[int, int], tuple[int, int]]],
) -> tuple[tuple[int, int], tuple[int, int]]:
    """Pick the smoothest outgoing boundary edge at a shared vertex."""
    in_vec = (
        current_vertex[0] - previous_vertex[0],
        current_vertex[1] - previous_vertex[1],
    )

    def turn_cost(
        candidate: tuple[tuple[int, int], tuple[int, int]],
    ) -> tuple[int, int, tuple[int, int]]:
        next_vertex, _cell = candidate
        out_vec = (
            next_vertex[0] - current_vertex[0],
            next_vertex[1] - current_vertex[1],
        )
        # Prefer continuing straight, then right/left turns, then reversing.
        dot = in_vec[0] * out_vec[0] + in_vec[1] * out_vec[1]
        cross = in_vec[0] * out_vec[1] - in_vec[1] * out_vec[0]
        if dot > 0:
            rank = 0
        elif cross != 0:
            rank = 1
        else:
            rank = 2
        return rank, abs(cross), next_vertex

    return min(candidates, key=turn_cost)


def _dedupe_consecutive_cells(
    cells: list[tuple[int, int]],
) -> list[tuple[int, int]]:
    deduped: list[tuple[int, int]] = []
    for cell in cells:
        if not deduped or deduped[-1] != cell:
            deduped.append(cell)
    return deduped


def _sort_cell_paths(
    paths: list[list[tuple[int, int]]],
    previous_cell: tuple[int, int] | None,
) -> list[list[tuple[int, int]]]:
    if previous_cell is None:
        return sorted(paths, key=len, reverse=True)

    return sorted(
        paths,
        key=lambda path: min(_cell_dist(cell, previous_cell) for cell in path),
    )


def _rotate_path_near_previous(
    path: list[tuple[int, int]],
    previous_cell: tuple[int, int] | None,
) -> list[tuple[int, int]]:
    if not path:
        return []

    closed = len(path) > 1 and path[0] == path[-1]
    base = path[:-1] if closed else list(path)
    if not base:
        return []

    if previous_cell is None:
        start_idx = min(range(len(base)), key=lambda i: base[i])
    else:
        start_idx = min(
            range(len(base)),
            key=lambda i: _cell_dist(base[i], previous_cell),
        )

    rotated = base[start_idx:] + base[:start_idx]
    if closed and rotated:
        rotated.append(rotated[0])
    elif previous_cell is not None and len(rotated) > 1:
        reversed_path = list(reversed(rotated))
        if _cell_dist(reversed_path[0], previous_cell) < _cell_dist(
            rotated[0], previous_cell,
        ):
            rotated = reversed_path
    return rotated


def _append_cell_path(
    target: list[tuple[int, int]],
    cells: list[tuple[int, int]],
) -> None:
    for cell in cells:
        cell = (int(cell[0]), int(cell[1]))
        if target and target[-1] == cell:
            continue
        target.append(cell)


def _cell_dist(a: tuple[int, int], b: tuple[int, int]) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _sample_cells_by_spacing(
    ordered_cells: list[tuple[int, int]],
    spacing_cells: int,
    traversable_mask: np.ndarray | None = None,
) -> list[tuple[int, int]]:
    """Subsample ordered cells by path distance while keeping safe chords."""
    if not ordered_cells:
        return []

    sampled = [ordered_cells[0]]
    chunk = [ordered_cells[0]]
    travelled = 0
    previous = ordered_cells[0]

    for cell in ordered_cells[1:]:
        travelled += _cell_dist(previous, cell)
        chunk.append(cell)
        previous = cell
        if travelled >= spacing_cells:
            _append_sample_chunk(sampled, chunk, traversable_mask)
            chunk = [sampled[-1]]
            travelled = 0

    if chunk[-1] != sampled[-1]:
        _append_sample_chunk(sampled, chunk, traversable_mask)

    return sampled


def _append_sample_chunk(
    sampled: list[tuple[int, int]],
    chunk: list[tuple[int, int]],
    traversable_mask: np.ndarray | None,
) -> None:
    target = chunk[-1]
    if (
        traversable_mask is None
        or _cell_segment_safe(sampled[-1], target, traversable_mask)
    ):
        if sampled[-1] != target:
            sampled.append(target)
        return

    for cell in chunk[1:]:
        if sampled[-1] != cell:
            sampled.append(cell)


def _cell_segment_safe(
    start: tuple[int, int],
    end: tuple[int, int],
    traversable_mask: np.ndarray,
) -> bool:
    H, W = traversable_mask.shape
    for r, c in _line_cells(start, end):
        if r < 0 or r >= H or c < 0 or c >= W:
            return False
        if not traversable_mask[r, c]:
            return False
    return True


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
