# Coverage Algorithm Improvement Specification

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

## 1. 目的

目前覆蓋式路徑規劃已能根據 zone map 產生基本牛耕式路徑，但在凹形邊界、窄通道、risk zone、zone 被障礙分割時，可能出現路線規劃超出邊界或跨越不可行區域的問題。

本規格書定義下一階段演算法升級方向：

- 修正路徑超出邊界問題
- 加入安全路徑驗證器
- 加入真正的 Boustrophedon Cell Decomposition
- 處理不適切割、不適合進入、過小區域
- 加入 cell 順序與掃描角度最佳化
- 加入模擬退火 Simulated Annealing 作為全域最佳化層

核心原則：

```text
安全合法性先於最佳化。
先保證不出界，再追求路徑短、轉彎少、重複少。
```

## 2. 現況摘要

目前主要流程：

```text
zone polygon
  -> raster OccupancyGrid
  -> free space erode / risk map inflate
  -> safe_map
  -> fixed-direction boustrophedon scan
  -> Path
  -> nav_action_follow_path
```

目前主要檔案：

| 功能 | 檔案 |
| --- | --- |
| coverage 主節點 | `src/boustrophedon_coverage/boustrophedon_coverage/boustrophedon_coverage.py` |
| 牛耕路徑生成 | `src/boustrophedon_coverage/boustrophedon_coverage/path_generators/boustrophedon.py` |
| zigzag 路徑生成 | `src/boustrophedon_coverage/boustrophedon_coverage/path_generators/zigzag.py` |
| spiral 路徑生成 | `src/boustrophedon_coverage/boustrophedon_coverage/path_generators/speiral.py` |
| map / zone / risk map | `src/maphub/maphub/map_manage.py` |
| path 轉 Pose | `src/boustrophedon_coverage/boustrophedon_coverage/utils/path_utils.py` |
| Nav action server | `src/nav_robot/nav_robot/nav_action_server_node.py` |

## 3. 問題定義

### 3.1 路徑點合法但線段不合法

目前路徑生成大多只確認 waypoint 在 `safe_map` 上，但沒有驗證兩個 waypoint 之間的連線是否完全落在 safe area。

常見結果：

```text
point A safe
point B safe
A -> B 的直線穿過邊界或障礙
```

這會在以下情況特別明顯：

- 凹形 polygon
- U 形區域
- 中間有 risk zone 洞
- 多個不連通 free-space island
- strip 內有多段 segment
- strip 與 strip 之間直接連線

### 3.2 coverage segment 和 connector segment 混在一起

目前 coverage path 會把割草路段和轉場路段直接串成同一條 `Path`。

但兩者本質不同：

| 類型 | 目的 | 安全需求 |
| --- | --- | --- |
| coverage segment | 覆蓋割草區域 | 要貼合覆蓋率與割幅 |
| connector segment | 從一段移動到下一段 | 要由 safe map 或 Nav2 規劃，不可直接直線穿越 |

### 3.3 邊界安全半徑不一致

`map_manage_node.py` 目前強制 `inflate_radius_m` 最低/預設為 `0.75m`；部署只能增加，不能在執行期降到安全包絡以下。

這代表 coverage planner 可能認為路徑安全，但 Nav2 costmap 會認為機器人已經貼邊或碰撞。

### 3.4 尚未實作真正 BCD

目前牛耕式路徑是對整張 `safe_map` 進行固定方向掃描。這不是完整 Boustrophedon Cell Decomposition。

完整 BCD 應先將區域切成拓樸簡單的 cells，再對每個 cell 產生 coverage path。

### 3.5 不適切割尚未定義

需要明確定義哪些 cell 不適合切割或不適合執行：

- 面積太小
- 寬度小於割幅或機器人通行寬度
- 細長但入口不可達
- 被侵蝕後消失
- cell 內含過多碎片或洞
- cell entry/exit 無法安全連接

### 3.6 最佳化層尚未存在

目前 cell/zone 順序、掃描角度、entry/exit 方向沒有全域最佳化。後續需要加入模擬退火來降低：

- 總路徑長度
- 非割草移動距離
- 轉彎次數
- 掉頭數
- 重複覆蓋
- 靠邊風險

## 4. 目標架構

建議將 coverage planner 拆成多層：

```text
OccupancyGrid / ZoneMap
  -> SafeMapBuilder
  -> CellDecomposer
  -> CellFilterAndMerger
  -> CoverageGenerator
  -> ConnectorPlanner
  -> PathValidator
  -> RouteOptimizer
  -> PathExporter
```

### 4.1 模組建議

未來若 package 合併成 `mower_mission`，建議目錄：

```text
mower_mission/
  coverage/
    safe_map.py
    cell_decomposition.py
    cell_filter.py
    coverage_generator.py
    connector_planner.py
    path_validator.py
    route_optimizer.py
    simulated_annealing.py
    metrics.py
    types.py
```

第一階段若還沒合併 package，可先放在：

```text
src/boustrophedon_coverage/boustrophedon_coverage/coverage/
```

## 5. 資料結構

### 5.1 SafeMap

```python
@dataclass
class SafeMap:
    grid: np.ndarray          # bool, True = safe
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str = "map"
```

### 5.2 CoverageCell

```python
@dataclass
class CoverageCell:
    cell_id: int
    mask: np.ndarray          # bool, same size as safe_map or cropped mask
    bbox: tuple[int, int, int, int]
    area_m2: float
    centroid_xy: tuple[float, float]
    entry_candidates: list[tuple[float, float]]
    exit_candidates: list[tuple[float, float]]
    valid: bool = True
    invalid_reason: str = ""
```

### 5.3 CoverageSegment

```python
@dataclass
class CoverageSegment:
    cell_id: int
    points: list[tuple[float, float]]
    start_xy: tuple[float, float]
    end_xy: tuple[float, float]
    scan_angle_deg: float
    segment_type: str = "coverage"
```

### 5.4 ConnectorSegment

```python
@dataclass
class ConnectorSegment:
    from_cell_id: int
    to_cell_id: int
    points: list[tuple[float, float]]
    length_m: float
    segment_type: str = "connector"
```

### 5.5 RoutePlan

```python
@dataclass
class RoutePlan:
    segments: list[CoverageSegment | ConnectorSegment]
    total_length_m: float
    coverage_length_m: float
    connector_length_m: float
    estimated_turn_count: int
    coverage_ratio: float
    valid: bool
```

## 6. SafeMapBuilder

### 6.1 輸入

- `zone_map.mask_map_inflated`
- `risk_map_inflated`
- robot radius
- GPS/RTK 誤差
- tracking error margin
- boundary margin

### 6.2 輸出

`safe_map.grid`：

```text
True  = 可行走且可覆蓋
False = 邊界外、risk、障礙、未知、保守安全距離內
```

### 6.3 安全半徑規則

新增參數：

| 參數 | 建議預設 | 說明 |
| --- | --- | --- |
| `robot_radius_m` | `0.5` | 與 Nav2 costmap 對齊 |
| `coverage_clearance_m` | `0.1` | coverage path 額外安全距離 |
| `gps_error_margin_m` | `0.05` to `0.2` | RTK fixed 可小，float 要大 |
| `tracking_error_margin_m` | `0.1` | 控制追蹤誤差 |
| `boundary_margin_m` | auto | 最終邊界收縮距離 |

建議公式：

```text
boundary_margin_m =
  robot_radius_m
  + coverage_clearance_m
  + gps_error_margin_m
  + tracking_error_margin_m
```

### 6.4 驗收

- safe map 不能包含 polygon 外部 cell。
- safe map 不能包含 risk inflated cell。
- safe map 邊界至少收縮 `boundary_margin_m`。

## 7. PathValidator

PathValidator 是第一優先實作，因為它直接解決出界問題。

### 7.1 功能

提供以下驗證：

```python
is_point_safe(x, y, safe_map) -> bool
is_segment_safe(p0, p1, safe_map) -> bool
validate_points(points, safe_map) -> ValidationResult
validate_path(path, safe_map) -> ValidationResult
```

### 7.2 線段驗證

將 world coordinate 轉成 grid index，使用 Bresenham 或 `cv2.line` 取得線段經過的所有 cells。

只要任一 cell 不 safe：

```text
segment invalid
```

### 7.3 ValidationResult

```python
@dataclass
class ValidationResult:
    valid: bool
    invalid_points: list[int]
    invalid_segments: list[tuple[int, int]]
    message: str
```

### 7.4 處理策略

若 coverage segment 不合法：

1. 優先切斷 segment。
2. 將中間連接交給 ConnectorPlanner。
3. 若 connector 也不可達，標記 cell invalid 或需要人工處理。

## 8. ConnectorPlanner

### 8.1 目的

不要讓 coverage generator 直接用直線連接不同 strip/cell。

ConnectorPlanner 負責：

- strip segment 之間的安全連接
- cell 與 cell 之間的安全連接
- zone 與 zone 之間的安全連接

### 8.2 實作選項

第一階段建議用 grid A*：

```text
safe_map grid -> A* -> connector points
```

後續可選：

- Nav2 `getPath()`
- Hybrid A*
- Theta*
- Smoothed A* with validation

### 8.3 Connector 規則

- connector 只允許走 safe_map。
- connector 不計入 coverage ratio。
- connector 若平滑後不合法，要退回未平滑 path。
- connector 需要限制最小轉彎半徑時，第二階段再加入車體運動限制。

## 9. Boustrophedon Cell Decomposition

### 9.1 目的

將複雜區域切成多個拓樸簡單 cell，避免固定掃描方向跨越凹形邊界或洞。

### 9.2 基本流程

```text
safe_map
  -> choose sweep direction
  -> scan columns or rows
  -> detect free intervals
  -> detect interval count changes
  -> create critical events
  -> assign interval continuity
  -> build cells
```

### 9.3 Free Interval

每個 scan line 取得連續 safe cells：

```python
Interval(row_or_col, start_idx, end_idx)
```

### 9.4 Critical Event

當相鄰 scan line 的 free interval 數量或連通關係變化：

- 1 -> 2 split
- 2 -> 1 merge
- 0 -> 1 enter
- 1 -> 0 exit

即為 cell 邊界。

### 9.5 Cell Graph

每個 cell 作為 graph node。相鄰或可連接 cell 建 edge。

```python
cell_graph[cell_a].append(cell_b)
```

edge weight 可先用 centroid distance，後續改用 A* connector path length。

## 10. 不適切割 / Cell Filter

### 10.1 不適合執行條件

新增參數：

| 參數 | 說明 |
| --- | --- |
| `min_cell_area_m2` | 小於此面積的 cell 不單獨執行 |
| `min_cell_width_m` | 小於機器通行寬度的 cell 不進入 |
| `min_strip_count` | 少於指定 strip 數的 cell 可合併或略過 |
| `min_entry_width_m` | entry 太窄視為不可達 |
| `max_isolated_cell_distance_m` | 過遠孤立 cell 需人工確認 |

### 10.2 處理策略

| 狀況 | 策略 |
| --- | --- |
| cell 太小 | merge 到相鄰最大 cell，或標記 ignored |
| cell 太窄 | 不產生 coverage path，只當作 connector 可通過區域或禁用 |
| cell 被 erode 後消失 | 標記 invalid |
| cell 無 entry/exit | 標記 unreachable |
| cell path 出界 | 重新切割或降低掃描角度候選 |

### 10.3 Cell Merge

合併條件：

- 兩 cell 相鄰
- 合併後 mask 仍連通
- 合併後 coverage path valid
- 合併後 connector cost 降低

## 11. CoverageGenerator

### 11.1 每個 cell 單獨產生路徑

不再對整張 safe_map 直接掃描，而是：

```text
for cell in cells:
    generate coverage path inside cell.mask
    validate segment
```

### 11.2 掃描角度候選

先支援離散角度：

```text
0, 15, 30, 45, 60, 75, 90, -15, -30, -45
```

每個角度產生候選 path，計算 cost。

### 11.3 Cost

單一 cell cost：

```text
cell_cost =
  path_length
  + turn_weight * turn_count
  + connector_hint_weight * distance_to_expected_next_cell
  + boundary_penalty
  + invalid_penalty
```

### 11.4 输出

每個 cell 輸出多個候選：

```python
dict[cell_id, list[CoverageSegmentCandidate]]
```

## 12. Simulated Annealing 最佳化

### 12.1 目的

模擬退火不負責修出界。它只在「所有候選都安全」後做全域順序與方向最佳化。

### 12.2 State

```python
SAState:
    cell_order: list[int]
    selected_candidate_by_cell: dict[int, int]
    entry_exit_flip_by_cell: dict[int, bool]
```

候選包含：

- cell 順序
- 每個 cell 的掃描角度
- 每個 cell 的正向/反向執行
- entry/exit 選擇

### 12.3 Neighbor

支援以下變動：

- swap 兩個 cell
- reverse 一段 cell order
- 改某個 cell 掃描角度
- flip 某個 cell path direction
- 將小 cell merge 或 unmerge
- 改 entry/exit candidate

### 12.4 Cost Function

```text
cost =
  w_total_length * total_length
  + w_connector_length * connector_length
  + w_turn_count * turn_count
  + w_overlap * overlap_area
  + w_missed_area * missed_area
  + w_boundary_risk * boundary_risk
  + w_invalid * invalid_segment_count
  + w_unreachable * unreachable_cell_count
```

建議第一版權重：

| 權重 | 建議 |
| --- | --- |
| `w_invalid` | 極大，例如 1e9 |
| `w_unreachable` | 極大，例如 1e8 |
| `w_missed_area` | 大 |
| `w_connector_length` | 中 |
| `w_turn_count` | 中 |
| `w_boundary_risk` | 中 |
| `w_overlap` | 小到中 |

### 12.5 接受準則

```python
if new_cost < current_cost:
    accept
else:
    accept with probability exp(-(new_cost - current_cost) / T)
```

### 12.6 降溫

第一版：

```text
T0 = 1.0
T_min = 1e-3
alpha = 0.995
max_iter = 3000
```

### 12.7 輸出

```python
best_route_plan: RoutePlan
```

## 13. 路徑匯出與 Nav2 執行

### 13.1 Path 分段

最終輸出不要只是一條混合 Path，應保留 segment metadata：

```text
coverage segment 1
connector segment 1
coverage segment 2
connector segment 2
...
```

### 13.2 Nav2 執行策略

建議：

- coverage segment：`followPath`
- connector segment：Nav2 `goToPose` 或 `followPath`
- 每段執行前檢查 TF 與 costmap 是否可用
- 每段執行後確認目前 pose 是否接近下一段起點

### 13.3 Split Points

不要再用 exact float match 判斷 split point。

改成：

```python
distance(current_pose, split_point) < split_tolerance_m
```

建議參數：

```text
split_tolerance_m = 0.05 to 0.15
```

## 14. ROS 參數

建議新增到 coverage node：

```yaml
coverage_planner:
  ros__parameters:
    strip_width_m: 0.2
    waypoint_spacing_m: 0.1
    robot_radius_m: 0.5
    coverage_clearance_m: 0.1
    gps_error_margin_m: 0.05
    tracking_error_margin_m: 0.1
    min_cell_area_m2: 0.25
    min_cell_width_m: 0.6
    min_strip_count: 2
    split_tolerance_m: 0.1
    enable_bcd: true
    enable_connector_planner: true
    enable_simulated_annealing: false
    scan_angle_candidates_deg: [0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0]
    sa_max_iter: 3000
    sa_initial_temperature: 1.0
    sa_min_temperature: 0.001
    sa_cooling_alpha: 0.995
```

## 15. Debug 與視覺化

建議新增 topics：

| Topic | Type | 說明 |
| --- | --- | --- |
| `/coverage_safe_map` | `nav_msgs/OccupancyGrid` | 最終 safe map |
| `/coverage_cells` | `visualization_msgs/MarkerArray` | BCD cells |
| `/coverage_invalid_segments` | `visualization_msgs/MarkerArray` | 出界或不合法線段 |
| `/coverage_connectors` | `visualization_msgs/MarkerArray` | connector path |
| `/coverage_route_plan` | `nav_msgs/Path` | 最終路徑 |
| `/coverage_route_metrics` | optional custom msg / log | 長度、覆蓋率、轉彎數 |

RViz 顏色建議：

| 類型 | 顏色 |
| --- | --- |
| safe map | 淺綠 |
| cell boundary | 藍 |
| coverage path | 綠 |
| connector path | 黃 |
| invalid segment | 紅 |
| ignored cell | 灰 |

## 16. 測試案例

### 16.1 Unit Test

新增測試地圖：

- rectangle
- L shape
- U shape
- donut shape
- narrow corridor
- disconnected islands
- risk zone inside polygon
- tiny cell
- cell narrower than robot

### 16.2 必測項目

PathValidator：

- waypoint outside map 應 invalid
- segment 穿越障礙應 invalid
- segment 沿 safe corridor 應 valid

BCD：

- rectangle 應只有一個或少量 cell
- U shape 應切出多個 cell
- risk zone hole 應不被 coverage segment 穿越

ConnectorPlanner：

- two safe cells 可連通時應產生 connector
- 不連通時應標記 unreachable

SA：

- cost 應隨 iteration 有機會下降
- invalid route 不應成為 best route

## 17. 開發順序

### Phase 1：修出界

目標：任何輸出的 path 都不能越界。

任務：

1. 實作 `PathValidator`
2. 對現有 boustrophedon path 做 point + segment validation
3. invalid segment 視覺化
4. 調整 `inflate_radius_m` / `boundary_margin_m`
5. 修 split point exact float match

驗收：

- L shape、U shape、risk hole 測試不再有直線穿越邊界。

### Phase 2：connector 分離

目標：coverage 和 connector 分開規劃。

任務：

1. 將同一 strip 內不連續 segment 切開
2. 實作 grid A*
3. 使用 connector 連接 segment
4. connector 通過 PathValidator

驗收：

- segment 之間不再直接用危險直線連接。

### Phase 3：BCD

目標：完整 cell decomposition。

任務：

1. 實作 free interval scan
2. 偵測 critical event
3. 建立 cell mask
4. 建立 cell graph
5. 每個 cell 單獨產生 coverage path

驗收：

- 凹形、多洞、多區塊地圖能分 cell 處理。

### Phase 4：不適切割

目標：過小、過窄、不可達 cell 不造成錯誤路徑。

任務：

1. 實作 cell metrics
2. 實作 invalid / ignored / merge
3. 視覺化 ignored cell

驗收：

- 小碎片不會造成奇怪 path。

### Phase 5：模擬退火

目標：在安全合法的前提下降低總成本。

任務：

1. 建立 route state
2. 建立 cost function
3. 建立 neighbor generator
4. 實作 annealing loop
5. 輸出 best route metrics

驗收：

- 對多 cell 地圖，SA 後 connector 長度或 turn count 低於 greedy baseline。

## 18. 驗收條件

### 18.1 安全

- 最終所有 waypoint 都在 safe map 內。
- 最終所有 segment 都通過 `is_segment_safe()`。
- smoothing 後路徑若不合法，必須退回合法版本。

### 18.2 覆蓋

- 覆蓋率達到設定閾值，例如 `>= 95%`。
- 不可進入 cell 需要列出原因。

### 18.3 可執行

- `ros2 service call /generate_coverage_path std_srvs/srv/Trigger` 可成功。
- RViz 可看到 safe map、cells、coverage path、invalid segment。
- `zone_exec_path` 不會送出 invalid path。

### 18.4 最佳化

- SA 啟用時，輸出 cost log。
- best route 必須 valid。
- 若 SA 失敗或 timeout，回退到 greedy route。

## 19. 第一版建議先改的程式點

優先順序：

1. `boustrophedon.py`：新增 segment validation，避免直接跨越 unsafe。
2. `path_utils.py`：轉 Path 前先過濾或切分 invalid segment。
3. `map_manage.py`：將 safety margin 與 Nav2 robot radius 對齊。
4. `nav_action_server_node.py`：split point 改成 tolerance，不用 exact float match。
5. 新增 `coverage/path_validator.py` 與 unit tests。

## 20. 不在第一階段做的事

第一階段不要同時做：

- 大幅重寫 Nav2 action server
- 修改所有 topic/service 名稱
- 同時合併 package
- 直接加入複雜 SA 而沒有 validator
- 將 `Chennal` 全面改名 `Channel`

先讓路徑安全，再讓路徑聰明。
