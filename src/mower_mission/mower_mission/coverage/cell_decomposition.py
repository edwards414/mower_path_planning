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
"""Boustrophedon Cell Decomposition (row-sweep variant).

Sweeps along rows (y-axis). Each contiguous True run in a row is a free
interval. Critical events — enter, exit, split, merge — define cell boundaries.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .path_validator import SafeMap
from .types import CoverageCell


@dataclass
class _Strip:
    row: int
    col_start: int
    col_end: int


def _free_intervals(row_data: np.ndarray) -> list[tuple[int, int]]:
    """Return (col_start, col_end) inclusive pairs for each contiguous True run."""
    intervals: list[tuple[int, int]] = []
    in_iv = False
    start = 0
    for c, v in enumerate(row_data):
        if v and not in_iv:
            start = c
            in_iv = True
        elif not v and in_iv:
            intervals.append((start, c - 1))
            in_iv = False
    if in_iv:
        intervals.append((start, int(len(row_data)) - 1))
    return intervals


def _overlaps(a: tuple[int, int], b: tuple[int, int]) -> bool:
    return a[0] <= b[1] and b[0] <= a[1]


def decompose(safe_map: SafeMap) -> tuple[list[CoverageCell], dict[int, list[int]]]:
    """Row-sweep BCD: decompose safe_map into topologically simple cells.

    Returns:
        cells: list of CoverageCell
        cell_graph: dict mapping cell_id -> sorted list of adjacent cell_ids.
            Adjacency is established at split and merge critical events.
    """
    grid = safe_map.grid
    H, W = grid.shape

    cell_counter = 0
    cell_strips: dict[int, list[_Strip]] = {}

    def _new_cell() -> int:
        nonlocal cell_counter
        cid = cell_counter
        cell_counter += 1
        cell_strips[cid] = []
        return cid

    adjacency: dict[int, set[int]] = {}

    prev_ivs: list[tuple[int, int]] = []
    prev_active: list[int] = []    # cell_id per prev interval

    for r in range(H):
        curr_ivs = _free_intervals(grid[r, :])

        # Overlap maps for this row transition
        prev_to_curr: dict[int, list[int]] = {pi: [] for pi in range(len(prev_ivs))}
        curr_to_prev: dict[int, list[int]] = {ci: [] for ci in range(len(curr_ivs))}
        for pi, piv in enumerate(prev_ivs):
            for ci, civ in enumerate(curr_ivs):
                if _overlaps(piv, civ):
                    prev_to_curr[pi].append(ci)
                    curr_to_prev[ci].append(pi)

        curr_active: list[int] = []
        for ci, civ in enumerate(curr_ivs):
            prev_list = curr_to_prev.get(ci, [])

            if len(prev_list) == 0:
                cid = _new_cell()                       # ENTER
            elif len(prev_list) == 1:
                pi = prev_list[0]
                if len(prev_to_curr[pi]) == 1:
                    cid = prev_active[pi]               # CONTINUE 1-to-1
                else:
                    cid = _new_cell()                   # SPLIT child
            else:
                cid = _new_cell()                       # MERGE child

            curr_active.append(cid)
            cell_strips[cid].append(_Strip(r, civ[0], civ[1]))

        # Adjacency: SPLIT (1 prev → N curr)
        for pi, curr_list in prev_to_curr.items():
            if len(curr_list) >= 2:
                parent = prev_active[pi]
                children = [curr_active[ci] for ci in curr_list]
                for ch in children:
                    adjacency.setdefault(ch, set()).add(parent)
                    adjacency.setdefault(parent, set()).add(ch)
                for i, ca in enumerate(children):
                    for cb in children[i + 1:]:
                        adjacency.setdefault(ca, set()).add(cb)
                        adjacency.setdefault(cb, set()).add(ca)

        # Adjacency: MERGE (N prev → 1 curr)
        for ci, prev_list in curr_to_prev.items():
            if len(prev_list) >= 2:
                child = curr_active[ci]
                for pi in prev_list:
                    parent = prev_active[pi]
                    adjacency.setdefault(child, set()).add(parent)
                    adjacency.setdefault(parent, set()).add(child)

        prev_ivs = curr_ivs
        prev_active = curr_active

    # ── Build CoverageCell objects ────────────────────────────────────────────
    res = safe_map.resolution
    ox = safe_map.origin_x
    oy = safe_map.origin_y

    cells: list[CoverageCell] = []
    for cid, strips in cell_strips.items():
        if not strips:
            continue

        mask = np.zeros((H, W), dtype=bool)
        for s in strips:
            mask[s.row, s.col_start : s.col_end + 1] = True

        rows_idx, cols_idx = np.where(mask)
        if len(rows_idx) == 0:
            continue

        rmin, rmax = int(rows_idx.min()), int(rows_idx.max())
        cmin, cmax = int(cols_idx.min()), int(cols_idx.max())

        area_m2 = float(mask.sum()) * res * res
        cx = ox + (float(cols_idx.mean()) + 0.5) * res
        cy = oy + (float(rows_idx.mean()) + 0.5) * res

        fs, ls = strips[0], strips[-1]
        entry_xy = (
            ox + ((fs.col_start + fs.col_end) / 2.0 + 0.5) * res,
            oy + (fs.row + 0.5) * res,
        )
        exit_xy = (
            ox + ((ls.col_start + ls.col_end) / 2.0 + 0.5) * res,
            oy + (ls.row + 0.5) * res,
        )

        cells.append(CoverageCell(
            cell_id=cid,
            mask=mask,
            bbox=(rmin, cmin, rmax, cmax),
            area_m2=area_m2,
            centroid_xy=(cx, cy),
            entry_candidates=[entry_xy],
            exit_candidates=[exit_xy],
        ))

    cell_graph: dict[int, list[int]] = {
        cell.cell_id: sorted(adjacency.get(cell.cell_id, set()))
        for cell in cells
    }

    return cells, cell_graph
