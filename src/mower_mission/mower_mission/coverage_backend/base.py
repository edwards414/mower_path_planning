"""Protocol defining the coverage backend interface."""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol

import numpy as np

if TYPE_CHECKING:
    from ..coverage.path_validator import SafeMap, ValidationResult


class CoverageBackend(Protocol):
    def filter_safe_components(
        self,
        safe_map: np.ndarray,
        resolution: float,
        min_area_m2: float,
        keep_largest_only: bool,
    ) -> tuple[np.ndarray, list[int], list[int]]: ...

    def validate_path(
        self,
        points: list[tuple[float, float]],
        safe_map: 'SafeMap',
    ) -> 'ValidationResult': ...

    def plan_connector(
        self,
        start: tuple[float, float],
        end: tuple[float, float],
        safe_map: 'SafeMap',
        boundary_weight: float = 0.2,
    ) -> list[tuple[float, float]] | None: ...

    def generate_zigzag_path(
        self,
        safe_map: np.ndarray,
        strip_width_m: float,
        res: float,
        H: int,
        W: int,
        origin_x: float,
        origin_y: float,
    ) -> tuple[list, list, list]: ...

    def generate_spiral_path(
        self,
        safe_map: np.ndarray,
        strip_width_m: float,
        res: float,
        H: int,
        W: int,
        origin_x: float,
        origin_y: float,
    ) -> tuple[list, list, list]: ...
