# Boustrophedon 覆蓋路徑演算法優化方案

> **2026-09-30 補記：** 本文描述的 Python 實作（`coverage_node.py`、`path_generators/`、`coverage/`）已在 2026-09-30 移除，現在的邏輯在 Rust：
>
> - `coverage/path_validator.py` → `src/mower_coverage_core/src/path_validator.rs`
> - `coverage/safe_map_filter.py` → `src/mower_coverage_core/src/safe_map_filter.rs`
> - `coverage/connector_planner.py` → `src/mower_coverage_core/src/connector_planner.rs`
> - `path_generators/zigzag.py` → `src/mower_coverage_core/src/zigzag.rs`
> - `path_generators/spiral.py`（舊名 `speiral.py`）→ `src/mower_coverage_core/src/spiral.rs`
> - `coverage/cell_decomposition.py` → `src/mower_coverage_core/src/cell_decomposition.rs`（已移植，尚未接進節點）
> - `coverage/types.py`：`CoverageCell` → `src/mower_coverage_core/src/cell_decomposition.rs`；`SpiralCoveragePlan` / `SpiralSegment` / `CoverageSegment` / `ConnectorSegment` / `RoutePlan` 沒有移植（Rust spiral 只回傳 `(points, split_points, invalid_segments)`）；`SafeMap` / `ValidationResult`（原本在 `coverage/path_validator.py`）→ `src/mower_coverage_core/src/types.rs`
> - `coverage_node.py` → `src/mower_rs/crates/mower_coverage/src/lib.rs`（`mower_rs` 的 `mower_coverage`，node 名仍是 `boustrophedon_coverage`，服務與參數不變）
>
> 下面的分析保留原文，檔名、行號與程式片段以當時的 Python 程式碼為準。

## 1. 目前演算法現況

目前 `coverage_node.py` 實際呼叫：

```python
_generate_coverage_boustrophedon_path(
    safe_map=safe_map,
    strip_width_m=strip_width_m,
    waypoint_spacing_m=waypoint_spacing_m,
    res=res,
    H=H,
    W=W,
    origin_x=ox,
    origin_y=oy,
    angle_deg=zigzag_angle_deg,
)
```

`zigzag_angle_deg` 目前由 `/boustrophedon_coverage` 參數指定：

```python
angle_deg = zigzag_angle_deg
```

因此目前覆蓋路徑屬於：

```text
可指定掃描角度的條帶掃描 coverage path
```

還不是完整的 Boustrophedon cellular decomposition。

---

## 2. 目前條帶生成方式

當 `angle_deg == 0.0` 時，演算法沿著地圖 column 做垂直掃描。

```python
strip_cols = max(1, int(round(strip_width_m / res)))
midcols = list(range(strip_cols // 2, W, strip_cols))
```

意義如下：

| 參數 | 意義 |
|---|---|
| `strip_width_m` | 割草機實際割幅 |
| `res` | map resolution，單位 m/cell |
| `strip_cols` | 每條割幅對應多少 grid cells |
| `midcols` | 每條掃描帶的中心 column |

例如：

```text
strip_width_m = 0.2 m
res = 0.05 m/cell
strip_cols = 4 cells
```

代表每隔 4 個 cell 選一條掃描線。

---

## 3. safe segment 生成邏輯

對每個中心 column `mc`，從 row `0` 掃到 `H - 1`。

邏輯如下：

```text
遇到 safe cell:
    如果目前沒有 segment start，記錄 start row

遇到 unsafe cell 或掃描到最後:
    如果目前正在 segment 中，建立一段 safe segment
```

每個 column 可能會有多段 safe segment，因為中間可能被以下區域切開：

```text
risk zone
obstacle
map boundary
unsafe area
```

---

## 4. 目前牛耕方向切換

目前使用 `reverse` 變數交替掃描方向：

```text
第 1 條 strip: 由下到上
第 2 條 strip: 由上到下
第 3 條 strip: 由下到上
...
```

對每個 segment：

```python
y0 = origin_y + (s + 0.5) * res
y1 = origin_y + (e + 0.5) * res
x = origin_x + (mc + 0.5) * res
```

再依照 `waypoint_spacing_m` 產生該 segment 上的 waypoint：

```python
spacing = max(res, waypoint_spacing_m)
```

因此 waypoint 間距不會小於地圖解析度。

---

## 5. 目前 split point 定義

目前每個 safe segment 生成完後，會將該 segment 的最後一個 waypoint 加入：

```python
coverage_split_points
```

目前用途：

```text
coverage_split_points 用於後續 nav action，把覆蓋路徑切成分段導航資訊。
```

但目前 split point 是：

```text
segment 結尾點
```

不是完整 cell decomposition 的分割點。

---

## 6. 目前路徑合法性檢查

路徑生成後會呼叫：

```python
_find_invalid_segments(points, safe_map, res, origin_x, origin_y)
```

內部使用：

```python
validate_path(points, safe_map_struct)
```

驗證邏輯包含：

```text
1. 每個 waypoint 是否落在 safe cell
2. 每一對連續 waypoint 的直線 segment 是否穿越 unsafe cell
```

segment 檢查使用 Bresenham rasterization：

```text
world point -> grid index
line segment -> all crossed grid cells
```

如果任何 cell：

```text
超出地圖
或
不是 safe
```

該 segment 會被判定為 invalid。

這可以抓到一種重要問題：

```text
兩個端點都安全，但中間直線跨越邊界或 risk zone。
```

目前 invalid segment 只會被標記並發布到 RViz，不會自動修補。

---

# 7. 主要問題分析

目前演算法的主要問題如下：

## 7.1 沒有真正做 cell decomposition

目前是直接對整張 `safe_map` 進行條帶掃描。

這在矩形場地可行，但遇到以下形狀會出現問題：

```text
凹形區域
中間有障礙物
risk zone 將區域切開
狹長通道
多個 disconnected safe region
```

目前沒有將相鄰 strip 的 safe segment 分群成 cell。

---

## 7.2 segment 之間可能直線跨越 unsafe

目前 coverage waypoint 生成後，如果兩段 safe segment 之間直接連線，可能發生：

```text
segment end 安全
next segment start 安全
但中間直線穿越 unsafe / risk / boundary
```

雖然目前 validate 可以抓出 invalid segment，但不會修補。

---

## 7.3 掃描方向由參數指定

目前 `angle_deg` 由 `zigzag_angle_deg` 指定。

這代表使用者可以手動為不同地圖設定掃描方向。

問題是不同形狀場地適合不同掃描角度，例如：

```text
長條形場地
斜向邊界
不規則多邊形
有障礙物分布的區域
```

如果掃描方向不佳，會增加：

```text
換行距離
turn count
connector length
invalid segment 數量
```

---

## 7.4 reverse 只適合單一矩形區域

目前的牛耕方向切換：

```text
第 1 條下到上
第 2 條上到下
第 3 條下到上
```

對矩形區域有效。

但如果同一條 strip 有多段 safe segment，單純 reverse 可能造成錯誤連接，例如：

```text
strip i   : 下半段 safe + 上半段 safe
strip i+1 : 下半段 safe + 上半段 safe
```

如果排序不佳，可能從下半段跳到上半段，中間跨過 unsafe。

---

## 7.5 split point 語意不夠清楚

目前 split point 是每段 segment 結尾點。

但在後續導航上，應該區分：

```text
coverage stroke split
cell split
navigation action split
connector split
```

否則 Nav2 action 分段會過細或語意不明確。

---

# 8. 底盤運動限制

目前割草機底盤為：

```text
差速底盤
可原地旋轉
```

因此目前不需要考慮：

```text
最小轉彎半徑
Dubins path
Reeds-Shepp path
車輛曲率限制
```

connector 可以先使用：

```text
grid A*
8-neighbor A*
Theta*
```

轉向只需要作為 cost penalty，不需要當作硬限制。

---

# 9. 優化目標

本次優化目標如下：

```text
1. 提升覆蓋路徑效率
2. 降低換行 connector 長度
3. 避免 segment 直線跨越 unsafe
4. 對不規則形狀做簡易切割
5. 換行或跨 cell 時參考邊界路徑
6. 保留目前 validate_path 檢查機制
7. 讓 path 可分段發布到 RViz 與 Nav2
```

---

# 10. 建議新架構

建議將 coverage path 拆成兩種路徑：

```text
coverage path
connector path
```

也就是：

```text
割草路徑歸割草路徑
換行 / 跨區移動路徑歸 connector path
```

---

## 10.1 PathElement 資料結構

建議新增：

```python
@dataclass
class PathElement:
    type: str
    points: list[tuple[float, float]]
    cell_id: int | None = None
    segment_id: int | None = None
```

其中 `type` 可分為：

```text
coverage
connector
boundary_connector
failed_connector
```

---

## 10.2 CoverageSegment 資料結構

建議將原本直接 extend 到 `coverage_points` 的邏輯，改為先建立 segment。

```python
@dataclass
class CoverageSegment:
    id: int
    cell_id: int | None
    strip_col: int
    start_row: int
    end_row: int
    points: list[tuple[float, float]]
```

原本：

```python
coverage_points.extend(segment_points)
coverage_split_points.append(segment_points[-1])
```

建議改成：

```python
segments.append(
    CoverageSegment(
        id=len(segments),
        cell_id=None,
        strip_col=mc,
        start_row=s,
        end_row=e,
        points=segment_points,
    )
)

coverage_split_points.append(segment_points[-1])
```

---

# 11. 第一階段優化：connector 修補

第一階段先不大改原本 coverage 生成方式。

只新增：

```text
segment end -> next segment start
```

之間的合法 connector。

---

## 11.1 connector 連接流程

流程如下：

```text
prev segment end
        ↓
next segment start
        ↓
先檢查直線是否合法
        ↓
合法：直接接
        ↓
不合法：使用 A* 找安全路徑
        ↓
A* 失敗：標記 failed connector，不硬接危險路徑
```

---

## 11.2 connect_segments()

建議新增：

```python
def connect_segments(prev_seg, next_seg, safe_map_struct):
    p0 = prev_seg.points[-1]
    p1 = next_seg.points[0]

    # 1. 直線合法，直接接
    if validate_path([p0, p1], safe_map_struct):
        return {
            "type": "straight",
            "points": [p0, p1],
            "valid": True,
        }

    # 2. 直線不合法，用 A*
    astar_points = astar_connector(
        start_world=p0,
        goal_world=p1,
        safe_map_struct=safe_map_struct,
    )

    if astar_points:
        return {
            "type": "astar",
            "points": astar_points,
            "valid": True,
        }

    # 3. A* 失敗，標記 failed
    return {
        "type": "failed",
        "points": [p0, p1],
        "valid": False,
    }
```

---

## 11.3 build_connected_coverage_path()

建議新增：

```python
def build_connected_coverage_path(segments, safe_map_struct):
    full_path = []
    connector_paths = []
    invalid_connectors = []

    if not segments:
        return full_path, connector_paths, invalid_connectors

    first = segments[0]
    full_path.extend(first.points)
    current_seg = first

    for next_seg in segments[1:]:
        connector = connect_segments(
            prev_seg=current_seg,
            next_seg=next_seg,
            safe_map_struct=safe_map_struct,
        )

        if connector["valid"]:
            # 避免重複加入目前點
            full_path.extend(connector["points"][1:])
            connector_paths.append(connector)

            # 避免重複加入 next segment 起點
            full_path.extend(next_seg.points[1:])
        else:
            invalid_connectors.append(connector)

            # 不建議硬接危險路徑
            # 可視需求選擇是否跳過此 segment
            full_path.extend(next_seg.points)

        current_seg = next_seg

    return full_path, connector_paths, invalid_connectors
```

---

# 12. A* connector 設計

因為底盤可原地旋轉，建議使用 8-neighbor A*。

允許方向：

```text
↑ ↓ ← → ↖ ↗ ↙ ↘
```

---

## 12.1 A* 基本成本

```python
if diagonal:
    step_cost = 1.414 * res
else:
    step_cost = 1.0 * res
```

---

## 12.2 安全距離成本

建議使用 distance transform：

```python
dist = distance_transform_edt(safe_map)
```

`dist[r, c]` 代表該 cell 離 unsafe 區域的距離。

轉成公尺：

```python
clearance_m = dist[r, c] * res
```

成本設計：

```python
if clearance_m < min_clearance_m:
    extra_cost += 100.0
else:
    extra_cost += 1.0 / max(clearance_m, 0.01)
```

這樣 A* 會偏好走在安全區域中央，不會貼著邊界或 risk zone。

---

## 12.3 turn penalty

雖然差速可以原地旋轉，但仍可加入 turn penalty，讓 connector 不要鋸齒狀。

```python
if prev_dir is not None and new_dir != prev_dir:
    cost += turn_penalty
```

建議初始值：

```python
turn_penalty = 0.05
```

---

# 13. 第二階段優化：segment 方向自動反轉

每條 coverage segment 其實有兩種走法：

```text
start -> end
end -> start
```

目前使用固定 reverse 交替。

建議後續改成：

```text
根據目前位置，選擇下一段 segment 的較近入口
```

---

## 13.1 segment 入口成本

```python
cost_normal = estimate_connector_cost(current_point, seg.points[0])
cost_reverse = estimate_connector_cost(current_point, seg.points[-1])
```

如果反向比較短：

```python
seg.points.reverse()
```

---

## 13.2 注意事項

不建議把全地圖所有 segment 直接丟進 nearest neighbor。

因為可能造成 coverage 順序跳來跳去：

```text
左下 -> 右下 -> 中間 -> 左上 -> 右上
```

較好的策略：

```text
cell 內保持牛耕順序
cell 間再做 nearest neighbor / graph ordering
```

---

# 14. 第三階段優化：掃描角度選擇

目前可由參數指定：

```python
angle_deg = zigzag_angle_deg
```

後續仍可改為測試多個候選角度：

```python
candidate_angles = [0, 15, 30, 45, 60, 75, 90]
```

每個角度生成一次 coverage path，計算分數。

---

## 14.1 評分公式

```python
score = (
    total_path_length
    + 2.0 * connector_length
    + 0.5 * turn_count
    + 100.0 * invalid_segment_count
)
```

其中：

| 項目 | 意義 |
|---|---|
| `total_path_length` | 全路徑長度 |
| `coverage_length` | 實際覆蓋長度 |
| `connector_length` | 非覆蓋移動長度 |
| `turn_count` | 換行或轉向次數 |
| `invalid_segment_count` | 不合法 segment 數量 |

建議優先降低：

```text
connector_length
invalid_segment_count
```

因為 coverage length 基本上由面積決定，差異通常較小。

---

# 15. 第四階段優化：簡易 cell decomposition

目前每條 strip 都可以得到 safe intervals。

例如：

```text
strip 0: [(10, 90)]
strip 1: [(10, 90)]
strip 2: [(10, 40), (60, 90)]
strip 3: [(10, 40), (60, 90)]
strip 4: [(10, 90)]
```

當 interval 數量改變時，代表發生：

```text
分裂
合併
```

這就是 Boustrophedon cellular decomposition 的核心。

---

## 15.1 interval overlap 判斷

可用 overlap ratio 判斷相鄰 strip 的 segment 是否屬於同一 cell。

```python
def overlap_ratio(a, b):
    s1, e1 = a
    s2, e2 = b

    overlap = max(0, min(e1, e2) - max(s1, s2))
    length = min(e1 - s1, e2 - s2)

    return overlap / max(1, length)
```

判斷：

```python
if overlap_ratio(prev_interval, curr_interval) > 0.3:
    same_cell
else:
    new_cell
```

---

## 15.2 CoverageCell 資料結構

建議新增：

```python
@dataclass
class CoverageCell:
    id: int
    strip_indices: list[int]
    intervals_by_strip: dict[int, list[tuple[int, int]]]
    coverage_segments: list[CoverageSegment]
```

---

# 16. 第五階段優化：邊界參考 connector

換行或跨 cell 時，若直線 connector 不合法，可以參考邊界路徑。

建議不要所有 connector 都強制走邊界。

分類如下：

| connector 類型 | 說明 | 是否需要邊界參考 |
|---|---|---|
| `row_change` | 同一 cell 內換行 | 通常不需要 |
| `cell_transfer` | 不同 cell 之間移動 | 建議需要 |
| `recovery` | invalid connector 修補 | 建議需要 |

---

## 16.1 邊界走廊生成方式

可使用 distance transform 產生內縮邊界走廊。

```python
dist = distance_transform_edt(safe_map) * res
```

設定目標距離：

```python
target_dist = robot_radius + safety_margin
```

產生 boundary band：

```python
boundary_band = abs(dist - target_dist) < tolerance
```

這代表一條距離邊界固定安全距離的內縮走廊。

---

## 16.2 boundary preference cost

A* cost 中加入邊界偏好：

```python
cost += k * abs(dist_to_obstacle - target_dist)
```

意義：

```text
希望機器人走在距離邊界 target_dist 的位置
```

不要直接貼邊，也不要離邊界太遠。

---

# 17. split point 重新定義

建議將 split point 分成三種：

```python
coverage_split_points
cell_split_points
navigation_split_points
```

---

## 17.1 coverage_split_points

每條 coverage stroke 結束點。

用途：

```text
debug
RViz 顯示
coverage stroke 統計
```

---

## 17.2 cell_split_points

每個 cell 完成點。

用途：

```text
cell-level 任務管理
cell coverage 進度追蹤
```

---

## 17.3 navigation_split_points

實際送給 Nav2 action 的分段點。

建議切法：

```text
一個 cell 一個 action
或
N 條 strip 一個 action
或
遇到 boundary_connector 時切一段 action
```

不建議每個小 segment 都切 Nav2 action，否則任務會過碎。

---

# 18. RViz debug 建議

建議分別發布以下 marker：

```text
safe_map
coverage_path
connector_path
boundary_connector_path
invalid_segments
coverage_split_points
cell_split_points
cell_id text marker
```

顏色建議：

| 類型 | 顏色 |
|---|---|
| coverage_path | 綠色 |
| connector_path | 藍色 |
| boundary_connector_path | 黃色 |
| invalid_segments | 紅色 |
| split_points | 紫色 |
| cell boundary | 白色 |

---

# 19. 建議程式模組切分

建議將功能拆成以下檔案：

```text
coverage_node.py
│
├── map_preprocess.py
│   ├── shrink_safe_area()
│   ├── expand_risk_area()
│   ├── connected_components()
│   └── distance_transform()
│
├── boustrophedon.py
│   ├── generate_strip_intervals()
│   ├── decompose_cells()
│   ├── generate_cell_coverage_segments()
│   └── generate_waypoints_on_segment()
│
├── connector_router.py
│   ├── validate_straight_connector()
│   ├── astar_connector()
│   ├── boundary_guided_connector()
│   └── smooth_and_validate()
│
├── optimizer.py
│   ├── evaluate_angle()
│   ├── choose_best_angle()
│   ├── order_cells()
│   └── order_segments_in_cell()
│
└── rviz_debug.py
    ├── publish_safe_map()
    ├── publish_coverage_path()
    ├── publish_connector_path()
    ├── publish_invalid_segments()
    └── publish_cell_markers()
```

---

# 20. 建議實作順序

## 第一階段：最小可用改動

目標：

```text
不要讓 segment 之間硬接危險直線
```

實作項目：

```text
1. 將 coverage points 改成 coverage segments
2. 新增 connect_segments()
3. 新增 build_connected_coverage_path()
4. 直線合法直接接
5. 直線不合法使用 A*
6. A* 失敗則標記 invalid connector
```

---

## 第二階段：提升效率

目標：

```text
降低 connector 長度
```

實作項目：

```text
1. segment 可自動反轉
2. 根據目前位置選擇較短入口
3. 計算 connector_length
4. 發布 connector path 到 RViz
```

---

## 第三階段：掃描角度最佳化

目標：

```text
根據場地形狀選擇較佳掃描方向
```

實作項目：

```text
1. 測試多個 candidate_angles
2. 每個 angle 生成 path
3. 計算 score
4. 選擇 score 最低的 angle
```

---

## 第四階段：簡易 cell decomposition

目標：

```text
對不規則形狀做簡易切割
```

實作項目：

```text
1. 每條 strip 生成 safe intervals
2. 使用 overlap_ratio 分群
3. 建立 CoverageCell
4. cell 內牛耕
5. cell 間用 connector router
```

---

## 第五階段：boundary-guided connector

目標：

```text
跨 cell 或 recovery 時參考邊界路徑
```

實作項目：

```text
1. 使用 distance_transform 產生 boundary band
2. A* cost 加入 boundary preference
3. cell_transfer 優先使用 boundary-guided A*
4. RViz 顯示 boundary connector
```

---

# 21. 最終建議結論

目前系統應優先從以下四點開始優化：

```text
1. coverage segment 與 connector segment 分離
2. invalid connector 自動用 A* 修補
3. segment 可根據入口成本自動反轉
4. 掃描角度不再固定 0 度
```

之後再加入：

```text
5. strip interval overlap 的簡易 cell decomposition
6. 跨 cell connector 使用 boundary-guided A*
7. cell-level coverage 任務管理
```

由於目前底盤為差速底盤，且可原地旋轉，因此現階段不需要加入 Dubins 或 Reeds-Shepp 曲率限制。

最適合目前系統的路徑生成策略為：

```text
coverage stroke 照常生成
        +
stroke 之間使用合法 connector 連接
        +
connector 不合法時使用 A* 修補
        +
跨 cell 或 recovery 時參考邊界走廊
```

這樣可以在不大幅重構現有程式的情況下，明顯提升覆蓋路徑的安全性、完整性與效率。
