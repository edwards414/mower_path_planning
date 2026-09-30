# 當前覆蓋式路徑邏輯算法

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

> **2026-09-30 補記二（演算法最佳化）：** `coverage_pattern=zigzag` 改由 `src/mower_coverage_core/src/boustrophedon.rs` 規劃，節點不再呼叫舊的 `zigzag.rs`（它只留給 oracle 測試）。已做：
>
> - 任何角度都是直線 lane（舊的旋轉分支會在條帶內左右來回）；lane 從安全區自己的範圍排（最外兩條貼著最外側的格子，間距 ≤ `strip_width_m`），不再對齊地圖格線，邊緣不會漏割。
> - 在掃描座標系做 BCD：每次 free space 分裂或合併就開新的 cell，每個 cell 各自來回割；cell 順序與每個 cell 的進入角用 DP（4 種進入方式）加 2-opt 最佳化（超過 80 個 cell 改用最近鄰）。模擬退火沒做，2-opt 在實測地圖上已足夠。
> - `zigzag_auto_angle`（預設 false）：每 5° 粗搜、前 4 名用最佳化後的順序重算、再在最佳角度 ±4° 每 1° 細搜，成本 = 路徑長 + 每次轉彎 3 m + 10 × 未覆蓋面積 / 條帶寬。
> - A* connector 會拉直（`simplify_path_rs`），節點的 connector 修補也一樣；規劃器的直線檢查用 supercover（線段碰到的每一格，含剛好穿過格角時的兩側格子），拉直不會穿過 A* 禁止的對角縫隙，路點也用 floor 規則檢查，不會出現在格子外（共用的 validator 用 Bresenham + 截斷，會把格子左/下方一格內的點算成第 0 格）。A* 的邊界距離圖每次規劃只算一次。
> - 導航伺服器會在 split point 0.1 m 內的每個內部點、以及急轉彎處切段，任何一段 ≤ 0.15 m 就拒絕整個任務。`nav_split.rs` 照 `mower_nav/src/geometry.rs` 模擬這個切法，`coalesce_for_navigation` 移除會切出過短段落的 split point（或路徑兩端幾公分），節點對 zigzag 與 spiral 的最終路徑都會跑這一步。
> - 規劃在 `spawn_blocking` 裡跑，不佔 `mower_rsd` 共用的 async 執行緒。
> - 每個 zone 在 log 記錄角度、lane / cell / 轉彎數、長度、覆蓋率與規劃時間；`/coverage_path` 發布實際路徑（以前只發空的）。
>
> 沒做：一個 zone 內不相連的安全區（它們之間隔著膨脹後的障礙，機器人本來就過不去，節點仍只割最大的一塊並記 warning）、各 cell 各自的角度、帶轉彎成本的邊界 connector、分型別的路徑輸出（PathElement）、spiral 的改善。

本文件描述目前專案中實際運作的覆蓋式路徑生成流程。內容以目前程式碼為準，主要來源包含：

- `src/mower_mission/mower_mission/map_manage_node.py`
- `src/mower_mission/mower_mission/coverage_node.py`
- `src/mower_mission/mower_mission/path_generators/boustrophedon.py`
- `src/mower_mission/mower_mission/coverage/path_validator.py`
- `src/mower_mission/mower_mission/utils/path_utils.py`

目前系統實際使用的是固定角度的 Boustrophedon 牛耕式覆蓋路徑。`zigzag.py` 和 `speiral.py` 目前存在，但沒有接入 `coverage_node.py` 的主要服務流程。

## 1. 整體資料流

目前覆蓋路徑流程分成四層：

```text
path_record_node
  -> 記錄工作區域 / 風險區域 polygon

map_manage_node
  -> 根據 polygon 生成 free space / risk map / zone map
  -> 提供 /get_zone_map_list_srv

coverage_node
  -> 取得 zone map
  -> 合成 safe_map
  -> 產生 boustrophedon coverage path
  -> 發布 RViz marker
  -> 儲存每個 zone 的 path 與 split points

nav_robot / NavActionClient
  -> /zone_exec_path 被呼叫後，將指定 zone path 送去導航執行
```

## 2. MapManage 地圖生成

節點：`map_manage_node.py`

主要輸出：

| Topic | 型別 | 用途 |
|---|---|---|
| `/map_grid` | `nav_msgs/OccupancyGrid` | Nav2 global/local costmap 使用的基礎地圖 |
| `/free_space` | `nav_msgs/OccupancyGrid` | 工作區域自由空間 |
| `/free_space_inflated` | `nav_msgs/OccupancyGrid` | 內縮後的自由空間 |
| `/risk_map` | `nav_msgs/OccupancyGrid` | 風險區域 |
| `/risk_map_inflated` | `nav_msgs/OccupancyGrid` | 膨脹後的風險區域 |
| `/chennal_map` | `nav_msgs/OccupancyGrid` | 通道區域 |
| `/chennal_map_inflated` | `nav_msgs/OccupancyGrid` | 內縮後的通道區域 |

主要服務：

| Service | 用途 |
|---|---|
| `/create_free_space` | 根據已記錄的工作區域 polygon 生成 zone map 和自由空間 |
| `/create_risk_map` | 根據已記錄的風險區域 polygon 生成風險地圖 |
| `/create_chennal_map` | 根據通道 path 生成通道地圖 |
| `/get_zone_map_list_srv` | 提供目前的 zone map list 給 coverage planner |

### 2.1 Demo Map

`map_manage_node` 啟動後會建立一張 demo `/map_grid`，目前設定為：

```text
resolution = 0.1 m/cell
width = 400 cells
height = 400 cells
origin = (-20.0, -20.0)
world range ~= x[-20, 20], y[-20, 20]
```

地圖邊界會被設為 `100`，內部為 `0`。

OccupancyGrid 語意：

```text
0   = free
100 = occupied / obstacle
```

### 2.2 Free Space 生成

呼叫 `/create_free_space` 後：

1. `map_manage_node` 呼叫 `/get_record_zone_list` 取得工作區域 polygon。
2. 根據所有 polygon 點計算 bounding box。
3. 使用固定解析度 `0.05 m/cell` 建立局部地圖。
4. 對每個 polygon 使用 `cv2.fillPoly` 填入可行區域。
5. 每個 zone 產生一份 `ZoneMap`：
   - `mask_map`
   - `mask_map_inflated`
6. 全部 zone 合併成整體 free space map。
7. 發布 `/free_space` 與 `/free_space_inflated`。
8. 將 `self.map_msg` 改成新的 free space map，後續每秒發布到 `/map_grid`。

### 2.3 Free Space Inflation

目前自由空間膨脹邏輯其實是對 free mask 做 erosion，效果是讓可走區域向內縮：

```python
free_mask = (free_space_map_data == 0)
eroded_free_mask = cv2.erode(free_mask, kernel)
inflated_data = np.where(eroded_free_mask == 1, 0, 100)
```

使用參數：

```text
inflate_radius_m = 0.75
```

這代表 coverage planner 使用的工作區域會比原始 polygon 內縮，避免路徑太靠邊界。

### 2.4 Risk Map 生成

呼叫 `/create_risk_map` 後：

1. `map_manage_node` 呼叫 `/get_risk_zone_list` 取得風險 polygon。
2. 在 `base_map` 上建立 risk map。
3. polygon 內部設為 `100`。
4. 使用 `cv2.dilate` 對風險區域向外膨脹。
5. 發布：
   - `/risk_map`
   - `/risk_map_inflated`

風險地圖語意：

```text
0   = safe
100 = risk / obstacle
```

## 3. CoveragePlanner 觸發流程

節點：`coverage_node.py`

主要服務：

| Service | 用途 |
|---|---|
| `/generate_coverage_path` | 生成所有 zone 的覆蓋路徑 |
| `/zone_exec_path` | 執行指定 zone 的覆蓋路徑 |
| `/cencel_nav2` | 取消目前 Nav2 任務 |
| `/check_nav_status` | 查詢導航狀態 |

主要參數：

| Parameter | Default | 用途 |
|---|---:|---|
| `strip_width_m` | `0.2` | 牛耕條帶間距，也可視為割幅 |
| `waypoint_spacing_m` | `0.1` | 同一條掃描線上的 waypoint 間距 |
| `unknown_as_obstacle` | `True` | 目前宣告但主流程未使用 |

呼叫 `/generate_coverage_path` 時，流程如下：

```text
check /risk_map_inflated exists
  -> call /get_zone_map_list_srv
  -> for each zone:
       read zone.mask_map / zone.mask_map_inflated
       combine with risk_map_inflated
       build safe_map
       generate boustrophedon path
       validate segments
       convert points to nav_msgs/Path
       convert split points to Pose list
       store path into zone_map
  -> publish /coverage_path_markers
  -> publish /coverage_invalid_segments if needed
```

目前 `/generate_coverage_path` 需要先收到 `/risk_map_inflated`。如果尚未呼叫 `/create_risk_map` 或沒有 risk map，服務會失敗。

## 4. Safe Map 合成邏輯

每個 zone 都會產生自己的 safe map。

來源：

```text
zone.mask_map_inflated
risk_map_inflated
```

程式邏輯：

```python
safe_map = np.logical_and(
    mask_map_inflated_data == 0,
    risk_map_data == 0
).astype(np.uint8)
```

也就是：

```text
safe cell = zone 內縮後仍為 free
            且
            不在膨脹後風險區域內
```

safe map 語意：

```text
1 / True  = 可以覆蓋與通行
0 / False = 不可行區域
```

## 5. Boustrophedon 路徑生成

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

`zigzag_angle_deg` 是 `/boustrophedon_coverage` 參數，允許範圍為 `0.0` 到 `180.0`。

### 5.1 條帶欄位選擇

當 `angle_deg == 0.0` 時，演算法沿著地圖 column 做垂直掃描。

```python
strip_cols = max(1, int(round(strip_width_m / res)))
midcols = list(range(strip_cols // 2, W, strip_cols))
```

意義：

```text
strip_width_m / res -> 每條割幅對應多少 grid cells
midcols              -> 每條掃描帶的中心 column
```

例如：

```text
strip_width_m = 0.2 m
res = 0.05 m/cell
strip_cols = 4 cells
```

所以每隔 4 個 cell 選一條掃描線。

### 5.2 每條掃描線尋找 safe segment

對每個中心 column `mc`，從 row `0` 掃到 `H-1`：

```text
遇到 safe cell:
  如果目前沒有 segment start，記錄 start row

遇到 unsafe cell 或掃描到最後:
  如果目前正在 segment 中，建立一段 (start, end)
```

每個 column 可能有多段 safe segment，因為中間可能被 risk zone 或邊界切開。

### 5.3 牛耕方向切換

演算法使用 `reverse` 變數交替方向：

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

```text
spacing = max(res, waypoint_spacing_m)
```

所以 waypoint 間距不會小於地圖解析度。

### 5.4 Split Points

目前每個 safe segment 生成完後，會將該 segment 最後一個 waypoint 加入 `coverage_split_points`。

用途：

```text
coverage_split_points 用於後續 nav action，把覆蓋路徑切成分段導航資訊。
```

目前 split point 是 segment 結尾點，不是完整 cell decomposition 的分割點。

## 6. 路徑合法性檢查

路徑生成後會呼叫：

```python
_find_invalid_segments(points, safe_map, res, origin_x, origin_y)
```

內部使用：

```python
validate_path(points, safe_map_struct)
```

驗證邏輯包含：

1. 每個 waypoint 是否落在 safe cell。
2. 每一對連續 waypoint 的直線 segment 是否穿越 unsafe cell。

segment 檢查使用 Bresenham rasterization：

```text
world point -> grid index
line segment -> all crossed grid cells
如果任何 cell 超出地圖或不是 safe，該 segment invalid
```

這可以抓到一種重要問題：

```text
兩個端點都安全，但中間直線跨越邊界或 risk zone。
```

目前 invalid segment 只會被標記並發布到 RViz，不會自動修補。

## 7. Path 轉換與發布

coverage points 會被轉成 `nav_msgs/Path`：

```python
_transform_coverage_path_points(points, map_header)
```

轉換規則：

```text
frame_id = map
pose.position.x/y = coverage point
pose.orientation = 根據下一個 waypoint 的方向計算 yaw
```

split points 會被轉成 `geometry_msgs/Pose` list：

```python
_transform_coverage_split_points(points, map_header)
```

生成後會存回每個 `ZoneMap`：

```python
zone_map.path = coverage_path
zone_map.coverage_split_points = coverage_split_points
```

目前 marker 發布：

| Topic | 型別 | 說明 |
|---|---|---|
| `/coverage_path_markers` | `visualization_msgs/MarkerArray` | 每個 zone 的 coverage line 與方向箭頭 |
| `/coverage_invalid_segments` | `visualization_msgs/MarkerArray` | 紅色線段，表示 unsafe connector 或 unsafe segment |

注意：目前 `coverage_node.py` 有建立 `/coverage_path` publisher，但主流程中沒有直接 publish `nav_msgs/Path` 到 `/coverage_path`。目前主要是發布 marker，並把 path 存在 `zone_map_list` 中供 `/zone_exec_path` 使用。

## 8. 路徑執行

呼叫：

```bash
ros2 service call /zone_exec_path mower_interface/srv/ZoneExecPath "{zone_id: <id>}"
```

流程：

```text
收到 zone_id
  -> 在 self.zone_map_list 中尋找對應 zone
  -> 取出 zone.path
  -> 取出 zone.coverage_split_points
  -> NavActionClient.send_goal_split_path(...)
```

也就是 `/generate_coverage_path` 必須先成功執行，`/zone_exec_path` 才有可用的 path。

## 9. 當前服務呼叫順序

典型流程：

```bash
ros2 service call /load_zone_list std_srvs/srv/Trigger "{}"
ros2 service call /create_free_space std_srvs/srv/Trigger "{}"
ros2 service call /create_risk_map std_srvs/srv/Trigger "{}"
ros2 service call /generate_coverage_path std_srvs/srv/Trigger "{}"
ros2 service call /zone_exec_path mower_interface/srv/ZoneExecPath "{zone_id: 0}"
```

在 `system_test.launch.py` 中，目前用 Timer 依序呼叫：

```text
5s   path_record_node
10s  map_manage_node
15s  coverage_node
20s  /load_zone_list
25s  /create_free_space
30s  /create_risk_map
35s  /generate_coverage_path
```

## 10. 目前限制

### 10.1 目前不是真正的 Boustrophedon Cell Decomposition

目前做法是直接對整張 `safe_map` 以固定方向掃描。

尚未做：

- cell decomposition
- obstacle critical point split
- cell adjacency graph
- cell traversal ordering

所以遇到凹形邊界、多障礙物、窄通道時，可能產生不理想的 connector。

### 10.2 Connector 會直接連線

目前所有 waypoint 被直接串成同一條 path。

這代表：

```text
coverage segment 和 connector segment 沒有分開規劃
```

即使 coverage waypoint 本身都安全，兩個 segment 之間的直線連線仍可能跨越 unsafe 區域。

目前系統會偵測 invalid segment，但不會自動避障重規劃。

### 10.3 risk map 與 zone map 尺寸假設相同

目前 `coverage_node.py` 直接將 `risk_map_inflated_map.data` reshape 成 zone map 的 `H, W`：

```python
risk_map_data = np.asarray(...).reshape(H, W)
```

這隱含假設：

```text
risk_map_inflated 和 zone mask map 解析度、寬高、origin 完全一致
```

目前沒有啟用 `_validate_maps_compatibility()` 來檢查。

### 10.4 unknown_as_obstacle 尚未接入主流程

`coverage_node.py` 有宣告：

```python
unknown_as_obstacle = True
```

但目前 safe map 合成只看：

```text
mask_map_inflated_data == 0
risk_map_data == 0
```

沒有額外處理 unknown cell。

### 10.5 angle_deg 支援

`zigzag.py` 有非零角度掃描分支，`coverage_node.py` 會使用：

```python
angle_deg=zigzag_angle_deg
```

目前角度由參數指定，尚未自動選擇最佳掃描角。

### 10.6 `/coverage_path` 目前沒有直接發布

雖然有：

```python
self.path_pub = self.create_publisher(Path, '/coverage_path', 1)
```

但目前 `generate_coverage_path()` 中沒有 publish `coverage_path` 到該 topic。

RViz 目前主要靠 `/coverage_path_markers` 看到路徑。

## 11. 簡化版偽代碼

```text
generate_coverage_path():
  require risk_map_inflated

  zone_maps = call /get_zone_map_list_srv

  for each zone in zone_maps:
    H, W, res, origin = zone.mask_map.info

    mask = reshape(zone.mask_map_inflated.data, H, W)
    risk = reshape(risk_map_inflated.data, H, W)

    safe_map = (mask == 0) AND (risk == 0)

    points, split_points, invalid_segments =
      boustrophedon(safe_map, strip_width, waypoint_spacing)

    if invalid_segments:
      publish red invalid segment markers

    zone.path = convert points to nav_msgs/Path
    zone.coverage_split_points = convert split_points to Pose list

  publish colored path markers
  return success
```

```text
boustrophedon(safe_map):
  strip_cols = round(strip_width / resolution)
  midcols = every strip_cols column

  reverse = false
  points = []
  split_points = []

  for each column in midcols:
    segments = continuous safe row intervals in this column

    if reverse:
      process segments in reverse order

    for each segment:
      create waypoints from segment start to end
      append waypoints to points
      append segment end to split_points

    reverse = not reverse

  invalid_segments = validate all consecutive point pairs
  return points, split_points, invalid_segments
```
