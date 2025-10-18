import numpy as np

def _generate_coverage_spiral_path(safe_map: np.ndarray,
                                   waypoint_spacing_m: float,
                                   res: float,
                                   H: int,
                                   W: int,
                                   origin_x: float,
                                   origin_y: float) -> list:
    """
    生成螺旋式覆蓋路徑
    safe_map: 可行區域 2維陣列
    waypoint_spacing_m: 路徑點間距
    res: 地圖解析度
    H, W: 地圖高寬
    origin_x, origin_y: 地圖原點（左下）
    回傳 [(x, y), ...]
    """
    visited = np.zeros_like(safe_map, dtype=bool)
    points = []
    # 找到 safe_map 的中心
    free_indices = np.argwhere(safe_map)
    if len(free_indices) == 0:
        return points
    # 計算重心
    mean_row = int(np.mean(free_indices[:, 0]))
    mean_col = int(np.mean(free_indices[:, 1]))

    dirs = [ (0,1), (1,0), (0,-1), (-1,0) ]  # 右、下、左、上
    d = 0   # 當前方向

    r, c = mean_row, mean_col
    # 若中心不是free，找附近最近的free cell
    if not safe_map[r, c]:
        dists = np.sum((free_indices - np.array([r, c]))**2, axis=1)
        nearest_idx = np.argmin(dists)
        r, c = free_indices[nearest_idx]

    visited[r, c] = True
    x = origin_x + (c + 0.5) * res
    y = origin_y + (r + 0.5) * res
    points.append((x, y))

    move_limit = 1   # 每圈次移動步數，逐圈增加
    steps_taken = 0  # 當前方向已行走步數
    changes = 0      # 方向變換次數
    total_points = np.count_nonzero(safe_map)
    cnt = 1

    while cnt < total_points:
        nr = r + dirs[d][0]
        nc = c + dirs[d][1]

        if 0 <= nr < H and 0 <= nc < W and safe_map[nr, nc] and not visited[nr, nc]:
            r, c = nr, nc
            visited[r, c] = True
            x = origin_x + (c + 0.5) * res
            y = origin_y + (r + 0.5) * res
            points.append((x, y))
            cnt += 1
            steps_taken += 1
        else:
            # 換方向
            d = (d + 1) % 4
            changes += 1
            if changes % 2 == 0:
                move_limit += 1
            steps_taken = 0
            # 若四個方向都走不了則結束
            stuck = True
            for dd in range(4):
                tr = r + dirs[dd][0]
                tc = c + dirs[dd][1]
                if 0 <= tr < H and 0 <= tc < W and safe_map[tr, tc] and not visited[tr, tc]:
                    stuck = False
                    break
            if stuck:
                break

    # 根據 waypoint_spacing_m 做 densify
    if len(points) < 2:
        return points
    densified = []
    for i in range(len(points)-1):
        x0, y0 = points[i]
        x1, y1 = points[i+1]
        dist = np.hypot(x1-x0, y1-y0)
        steps = max(1, int(np.floor(dist/waypoint_spacing_m)))
        for j in range(steps):
            t = j/steps
            xx = x0 + t*(x1-x0)
            yy = y0 + t*(y1-y0)
            densified.append( (xx, yy) )
    densified.append(points[-1])
    return densified
