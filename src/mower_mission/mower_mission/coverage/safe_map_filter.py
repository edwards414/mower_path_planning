"""Helpers for filtering disconnected safe-map fragments."""

from collections import deque
from math import ceil

import numpy as np

_NEIGHBOURS_8 = [
    (-1, 0),
    (1, 0),
    (0, -1),
    (0, 1),
    (-1, -1),
    (-1, 1),
    (1, -1),
    (1, 1),
]


def filter_safe_components(
    safe_map: np.ndarray,
    resolution: float,
    min_area_m2: float = 0.05,
    keep_largest_only: bool = True,
) -> tuple[np.ndarray, list[int], list[int]]:
    """Remove tiny disconnected safe-map fragments.

    Returns (filtered_safe_map, all_component_sizes, kept_component_sizes).
    Component sizes are expressed in cells.
    """
    safe = safe_map.astype(bool)
    components = _connected_components(safe)
    component_sizes = sorted((len(cells) for cells in components), reverse=True)

    if not components:
        return safe.copy(), [], []

    min_cells = max(1, int(ceil(min_area_m2 / (resolution * resolution))))
    kept = [
        cells
        for cells in components
        if len(cells) >= min_cells
    ]

    if keep_largest_only and kept:
        kept = [max(kept, key=len)]

    filtered = np.zeros_like(safe, dtype=bool)
    for cells in kept:
        rows, cols = zip(*cells)
        filtered[rows, cols] = True

    kept_sizes = sorted((len(cells) for cells in kept), reverse=True)
    return filtered, component_sizes, kept_sizes


def _connected_components(safe: np.ndarray) -> list[list[tuple[int, int]]]:
    H, W = safe.shape
    seen = np.zeros_like(safe, dtype=bool)
    components: list[list[tuple[int, int]]] = []

    for r in range(H):
        for c in range(W):
            if not safe[r, c] or seen[r, c]:
                continue

            cells: list[tuple[int, int]] = []
            q: deque[tuple[int, int]] = deque([(r, c)])
            seen[r, c] = True

            while q:
                rr, cc = q.popleft()
                cells.append((rr, cc))
                for dr, dc in _NEIGHBOURS_8:
                    nr, nc = rr + dr, cc + dc
                    if nr < 0 or nr >= H or nc < 0 or nc >= W:
                        continue
                    if seen[nr, nc] or not safe[nr, nc]:
                        continue
                    seen[nr, nc] = True
                    q.append((nr, nc))

            components.append(cells)

    return components
