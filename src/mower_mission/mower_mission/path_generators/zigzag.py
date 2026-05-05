"""生成 Zigzag 覆蓋路徑."""

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
import numpy as np


def _generate_coverage_zigzag_path(
    safe_map: np.ndarray,
    waypoint_spacing_m: float,
    res: float,
    H: int,
    W: int,
    origin_x: float,
    origin_y: float,
) -> list:
    """
    生成 Zigzag 覆蓋路徑.

    Args
    ----
    safe_map : np.ndarray
        可行區域 2維陣列
    waypoint_spacing_m : float
        路徑點間距
    res : float
        地圖解析度(m)
    H : int
        地圖高
    W : int
        地圖寬
    origin_x : float
        地圖原點X（左下）
    origin_y : float
        地圖原點Y（左下）

    Returns
    -------
    list
        路徑點列表 [(x, y), ...]

    """
    points = []
    valid_cols = [c for c in range(W) if np.any(safe_map[:, c])]
    if not valid_cols:
        return points
    min_col, max_col = valid_cols[0], valid_cols[-1]
    scan_rows = [r for r in range(H) if np.any(safe_map[r, :])]
    if not scan_rows:
        return points
    min_row, max_row = scan_rows[0], scan_rows[-1]

    direction = 1
    for row in range(
        min_row, max_row + 1, max(1, int(round(waypoint_spacing_m / res)))
    ):
        cols = (
            range(min_col, max_col + 1)
            if direction == 1
            else range(max_col, min_col - 1, -1)
        )
        for col in cols:
            if safe_map[row, col]:
                x = origin_x + (col + 0.5) * res
                y = origin_y + (row + 0.5) * res
                points.append((x, y))
        direction *= -1
    if len(points) < 2:
        return points
    densified = []
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        dist = np.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(np.floor(dist / waypoint_spacing_m)))
        for j in range(steps):
            t = j / steps
            xx = x0 + t * (x1 - x0)
            yy = y0 + t * (y1 - y0)
            densified.append((xx, yy))
    densified.append(points[-1])
    return densified
