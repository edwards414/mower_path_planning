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
"""Shared data types for the coverage planning pipeline."""

from dataclasses import dataclass, field


@dataclass(eq=False)
class CoverageCell:
    """A BCD cell with its occupancy mask and planning metadata."""

    cell_id: int
    mask: object          # np.ndarray (bool), same shape as safe_map.grid
    bbox: tuple           # (row_min, col_min, row_max, col_max)
    area_m2: float
    centroid_xy: tuple    # (x, y) world coordinates
    entry_candidates: list = field(default_factory=list)  # [(x, y), ...]
    exit_candidates: list = field(default_factory=list)   # [(x, y), ...]
    valid: bool = True
    invalid_reason: str = ''

    def __eq__(self, other: object) -> bool:
        return isinstance(other, CoverageCell) and self.cell_id == other.cell_id

    def __hash__(self) -> int:
        return hash(self.cell_id)


@dataclass
class CoverageSegment:
    """A coverage (mowing) path segment inside one cell."""

    cell_id: int
    points: list[tuple[float, float]]
    start_xy: tuple[float, float]
    end_xy: tuple[float, float]
    scan_angle_deg: float
    segment_type: str = 'coverage'


@dataclass
class ConnectorSegment:
    """A transition path segment between two coverage segments."""

    from_cell_id: int
    to_cell_id: int
    points: list[tuple[float, float]]
    length_m: float
    segment_type: str = 'connector'


@dataclass
class RoutePlan:
    """Complete ordered route for one zone."""

    segments: list = field(default_factory=list)  # CoverageSegment | ConnectorSegment
    total_length_m: float = 0.0
    coverage_length_m: float = 0.0
    connector_length_m: float = 0.0
    estimated_turn_count: int = 0
    coverage_ratio: float = 0.0
    valid: bool = False
