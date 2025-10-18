import numpy as np

def _generate_coverage_boustrophedon_path(safe_map: np.ndarray,
                                            strip_width_m: float,#割草機有效割幅
                                            waypoint_spacing_m: float,#路徑點間距
                                            res: float,
                                            H: int,#地圖高
                                            W: int,#地圖寬
                                            origin_x: float,#地圖原點X
                                            origin_y: float,#地圖原點Y
                                            angle_deg: float = 0.0)-> list:#條帶相對X軸角度（degree）
    """
    生成直線牛耕式覆蓋路徑，可指定條帶角度（degree, 逆時針，0為Y掃描） 
    angle_deg: 條帶相對X軸角度（degree）
    """
    # 如果角度為0，則維持原行為
    angle_rad = np.deg2rad(angle_deg)
    if abs(angle_rad) < 1e-6:
        # 條帶設定：以 X 方向切直條(沿 Y 掃描)
        strip_w_m = strip_width_m
        strip_cols = max(1, int(round(strip_w_m / res)))
        midcols = list(range(strip_cols // 2, W, strip_cols))
        
        spacing = waypoint_spacing_m
        points = []
        reverse = False

        for mc in midcols:
            segments = []
            start = None
            for i in range(H):
                ok = bool(safe_map[i, mc])
                is_last = (i == H - 1)
                if ok and start is None:
                    start = i
                if (not ok or is_last) and start is not None:
                    end = i if (not ok) else i
                    segments.append((start, end))
                    start = None

            segs = segments[::-1] if reverse else segments
            for (s, e) in segs:
                y0 = origin_y + (s + 0.5) * res
                y1 = origin_y + (e + 0.5) * res
                x  = origin_x + (mc + 0.5) * res
                if y1 >= y0:
                    ys = list(np.arange(y0, y1, max(res, spacing))) + [y1]
                else:
                    ys = list(np.arange(y0, y1, -max(res, spacing))) + [y1]
                ys = ys[::-1] if reverse else ys
                for y in ys:
                    points.append((x, y))
            reverse = not reverse
        return points
    # 否則，根據角度旋轉生成覆蓋路徑
    # Step1: 將safe_map投影至旋轉後座標系
    # step2: 在旋轉後座標系產生牛耕路徑
    # step3: 反旋轉回原座標
    # 先計算所有cell中心的座標
    yy, xx = np.indices((H, W))
    xs = origin_x + (xx + 0.5) * res
    ys = origin_y + (yy + 0.5) * res
    coords = np.stack([xs, ys], axis=-1)     # shape: (H, W, 2)
    # 旋轉中心設為地圖中心
    center_x = origin_x + W * res / 2
    center_y = origin_y + H * res / 2
    c = np.cos(-angle_rad)
    s = np.sin(-angle_rad)
    rotM = np.array([[c, -s],[s, c]])
    coords_rot = coords - np.array([center_x, center_y])
    coords_rot = coords_rot @ rotM.T
    coords_rot = coords_rot + np.array([center_x, center_y])
    # 建立旋轉後的虛擬map
    xs_rot = coords_rot[...,0]
    ys_rot = coords_rot[...,1]
    # 投影到旋轉後的X,Y座標
    safe_map_rot = np.zeros_like(safe_map, dtype=bool)
    # 僅保留原本safe_map的可行點
    idxs = np.argwhere(safe_map)
    for (i, j) in idxs:
        safe_map_rot[i, j] = True
    # 在旋轉後的系下，牛耕路徑、條帶以旋轉後X方向為基準，對於safe_map中每個點，計算其旋轉後的X, Y
    all_rot_points = coords_rot[safe_map]
    # 取得旋轉後地圖的範圍
    minx, maxx = np.min(all_rot_points[:,0]), np.max(all_rot_points[:,0])
    miny, maxy = np.min(all_rot_points[:,1]), np.max(all_rot_points[:,1])
    width_rot = maxx - minx
    height_rot = maxy - miny
    # 條帶中心線
    strip_w_m = strip_width_m
    n_strips = max(1, int(np.floor(width_rot / strip_w_m)))
    if n_strips < 1:
        n_strips = 1
    strip_centers_x = np.linspace(minx + strip_w_m/2, maxx - strip_w_m/2 if n_strips > 1 else maxx - strip_w_m/2, n_strips)
    # 將所有safe點依照離哪個條帶中心x最近分段
    points = []
    reverse = False
    for scx in strip_centers_x:
        # 找在該條帶（中心線strip_w_m/2寬之內）的safe點
        dist_to_strip = np.abs(all_rot_points[:,0] - scx)
        mask = dist_to_strip <= strip_w_m/2
        candidates = all_rot_points[mask]
        if len(candidates) == 0:
            reverse = not reverse
            continue
        # 按Y方向排序（在條帶座標系，Y是覆蓋方向）
        sort_order = np.argsort(candidates[:,1])
        if reverse:
            sort_order = sort_order[::-1]
        ordered = candidates[sort_order]
        # 間隔densify
        prev_pt = None
        for pt in ordered:
            if prev_pt is not None:
                dist = np.linalg.norm(pt - prev_pt)
                if dist < max(res, waypoint_spacing_m)*0.5:
                    continue
            prev_pt = pt
            points.append(tuple(pt))
        reverse = not reverse
    # 將points從旋轉座標轉回原地圖座標
    if angle_deg != 0.0:
        # 反向旋轉
        c_inv = np.cos(angle_rad)
        s_inv = np.sin(angle_rad)
        rotMinv = np.array([[c_inv, -s_inv],[s_inv, c_inv]])
        points_np = np.array(points) - np.array([center_x, center_y])
        points_np = points_np @ rotMinv.T
        points_np = points_np + np.array([center_x, center_y])
        points = [tuple(pt) for pt in points_np]
    return points