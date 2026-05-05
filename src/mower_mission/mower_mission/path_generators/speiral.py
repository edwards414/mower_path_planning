"""生成螺旋式覆蓋路徑."""

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


def _generate_coverage_spiral_path(
    safe_map: np.ndarray,
    waypoint_spacing_m: float,
    res: float,
    H: int,
    W: int,
    origin_x: float,
    origin_y: float,
) -> list:
    """
    生成螺旋式覆蓋路徑.

    Args
    ----
    safe_map : np.ndarray
        可行區域 2維陣列
    waypoint_spacing_m : float
        路徑點間距
    res : float
        地圖解析度
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
    visited = np.zeros_like(safe_map, dtype=bool)
    points = []
    free_indices = np.argwhere(safe_map)
    if len(free_indices) == 0:
        return points
    mean_row = int(np.mean(free_indices[:, 0]))
    mean_col = int(np.mean(free_indices[:, 1]))

    dirs = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    d = 0

    r, c = mean_row, mean_col
    if not safe_map[r, c]:
        dists = np.sum((free_indices - np.array([r, c])) ** 2, axis=1)
        nearest_idx = np.argmin(dists)
        r, c = free_indices[nearest_idx]

    visited[r, c] = True
    x = origin_x + (c + 0.5) * res
    y = origin_y + (r + 0.5) * res
    points.append((x, y))

    move_limit = 1
    steps_taken = 0
    changes = 0
    total_points = np.count_nonzero(safe_map)
    cnt = 1

    while cnt < total_points:
        nr = r + dirs[d][0]
        nc = c + dirs[d][1]

        if (0 <= nr < H and 0 <= nc < W and safe_map[nr, nc]
                and not visited[nr, nc]):
            r, c = nr, nc
            visited[r, c] = True
            x = origin_x + (c + 0.5) * res
            y = origin_y + (r + 0.5) * res
            points.append((x, y))
            cnt += 1
            steps_taken += 1
        else:
            d = (d + 1) % 4
            changes += 1
            if changes % 2 == 0:
                move_limit += 1
            steps_taken = 0
            stuck = True
            for dd in range(4):
                tr = r + dirs[dd][0]
                tc = c + dirs[dd][1]
                if (
                    0 <= tr < H
                    and 0 <= tc < W
                    and safe_map[tr, tc]
                    and not visited[tr, tc]
                ):
                    stuck = False
                    break
            if stuck:
                break

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
