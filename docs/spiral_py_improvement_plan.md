# `spiral.py` 改善規劃文件

## 1. 目前狀態

目前 `spiral.py` 採用：

```text
中心點 start
  ↓
greedy spiral walk
  ↓
return points
```

也就是從中心點開始，用貪婪式 spiral 方式往外走，最後只回傳 `points`。

---

## 2. 目前主要問題

| 問題 | 說明 | 影響 |
|---|---|---|
| 只回傳 `points` | 沒有 segment、coverage mask、split point、invalid segment | 後續很難 debug，也不好在 RViz 顯示問題 |
| 容易卡住 | greedy 只看附近可走點，走到死路就停止 | 不規則區域容易覆蓋不完整 |
| 不適合洞與凹形區域 | 沒有 obstacle boundary / virtual wall 邏輯 | 洞周圍、凹陷邊界容易漏割 |
| 安全區斷裂處理不足 | 沒有 connected components 分析 | 多塊安全區可能被錯誤硬接 |
| 沒有 split points | 無法標出路徑中斷、backtrack、bridge 起點 | 不利於後續 Nav2 waypoint 分段 |
| 沒有 invalid segments | 無法標記不可達、穿越 unsafe、過窄通道 | 容易產生危險路徑 |
| `move_limit` 邏輯不明確 | 目前比較像 loop 次數限制，沒有真的控制 spiral 半徑 | 不適合用來限制 coverage 範圍 |

---

## 3. 改善目標

`spiral.py` 不應只是一個產生點的函式，而應該升級成一個可以輸出完整規劃結果的 coverage module。

目標：

```text
safe_map
  ↓
coverage planner
  ↓
CoveragePlan
```

輸出內容應包含：

```text
1. 完整 coverage path
2. 每段 segment 的類型
3. split points
4. invalid segments
5. coverage mask
6. debug 資訊
```

---

## 4. 建議資料結構

### 4.1 CoverageSegment

```python
from dataclasses import dataclass
from typing import List, Tuple

GridPoint = Tuple[int, int]
WorldPoint = Tuple[float, float]

@dataclass
class CoverageSegment:
    points: List[WorldPoint]
    segment_type: str
    component_id: int
    valid: bool
    reason: str = ""
```

`segment_type` 建議使用：

```text
spiral      # 正常 spiral coverage
bridge      # 從目前位置接到下一個未覆蓋區
backtrack   # 回溯路徑
invalid     # 不可行路徑
boundary    # 邊界補覆蓋
```

---

### 4.2 CoveragePlan

```python
@dataclass
class CoveragePlan:
    points: List[WorldPoint]
    segments: List[CoverageSegment]
    split_points: List[WorldPoint]
    invalid_segments: List[CoverageSegment]
    coverage_mask: np.ndarray
    debug: dict
```

---

## 5. 建議處理流程

```text
safe_map
  ↓
connected components labeling
  ↓
for each component:
    選擇 start cell
    執行 greedy spiral
    若 spiral 卡住:
        BFS 找最近 unvisited free cell
        A* bridge 接過去
        繼續 spiral
  ↓
合併 segments
  ↓
檢查 invalid segments
  ↓
必要時做 smoothing
  ↓
再次 collision check
  ↓
return CoveragePlan
```

---

## 6. Connected Components 處理

### 6.1 目的

先把安全區分成多個 connected components。

這可以避免安全區已經斷裂時，planner 還硬把路徑連起來。

```python
component_map, num_components = label_connected_components(safe_map)
```

### 6.2 行為規則

| 狀況 | 處理方式 |
|---|---|
| 單一 component | 正常 spiral coverage |
| 多個 component | 每個 component 分別規劃 |
| component 之間有 safe corridor | 用 A* bridge 連接 |
| component 之間無 safe path | 標記 invalid segment |

---

## 7. Greedy Spiral 改善

### 7.1 原本邏輯

```text
目前位置
  ↓
找下一個可走鄰居
  ↓
若沒有可走點就停止
```

### 7.2 改善後邏輯

```text
目前位置
  ↓
找最佳鄰居
  ↓
若有可走點:
      移動並標記 visited
  ↓
若沒有可走點:
      BFS 找最近未覆蓋 free cell
      A* bridge 接過去
      繼續 spiral
```

---

## 8. Neighbor Scoring 建議

不要只用固定方向順序，建議對候選鄰居打分數。

```python
score = 0.0

if is_unvisited:
    score += 100.0

if keeps_same_direction:
    score += 20.0

if follows_virtual_wall:
    score += 10.0

score += 5.0 * distance_from_obstacle

if causes_turn:
    score -= 30.0

if is_unsafe:
    score -= 1000.0
```

### 建議先使用 4-neighbor

割草機是差速底盤，coverage path 應先穩定，不建議一開始就大量使用斜向移動。

```text
4-neighbor:
up, right, down, left
```

後續 smoothing 再處理轉角。

---

## 9. 卡住時的處理：BFS + A*

### 9.1 卡住判斷

```python
if no_unvisited_neighbor:
    stuck = True
```

### 9.2 找下一個未覆蓋區

用 BFS 從目前位置往外找最近的：

```text
safe_map == True
visited_map == False
```

```python
target = find_nearest_unvisited_cell_by_bfs(
    current_cell,
    safe_map,
    visited_map
)
```

### 9.3 用 A* 接過去

```python
bridge = astar(
    start=current_cell,
    goal=target,
    traversable_map=safe_map
)
```

若 A* 成功：

```text
新增 split point
新增 bridge segment
移動到 target
繼續 spiral
```

若 A* 失敗：

```text
新增 invalid segment
標記不可達區域
```

---

## 10. split points 定義

以下情況應加入 split point：

| 觸發條件 | 說明 |
|---|---|
| spiral 卡住 | 準備進入 backtrack / bridge |
| 進入另一個 component | 安全區被切成不同區塊 |
| bridge path 過長 | 代表這段不是正常 coverage，而是轉場 |
| heading change 過大 | 差速底盤可能需要分段控制 |
| smoothing 後碰撞 | 需要切段重規劃 |
| A* path 找不到 | 從此處開始 invalid |

範例：

```python
split_points.append(current_world_point)
```

---

## 11. invalid segments 定義

以下情況應標記 invalid：

| 條件 | 說明 |
|---|---|
| 直線段穿越 unsafe cell | 不能直接連線 |
| A* 找不到 safe path | 兩區域不可達 |
| 通道寬度小於 robot_width + safety_margin | 機器人通不過 |
| smoothing 後曲線碰撞 | 平滑路徑跑出安全區 |
| 曲率半徑小於最小轉彎半徑 | 差速底盤雖可原地旋轉，但高速追蹤不穩 |
| component 面積太小 | 小於有效割幅或機器人尺寸 |

---

## 12. `move_limit` 修正建議

目前 `move_limit` 不應被視為 spiral 半徑控制。

建議拆成不同參數：

```python
max_steps: int
max_radius_m: float | None
max_uncovered_ratio: float
max_bridge_length_m: float
```

### 12.1 若要控制最大半徑

應使用距離判斷：

```python
dist_m = np.linalg.norm(
    np.array(current_cell) - np.array(start_cell)
) * resolution

if max_radius_m is not None and dist_m > max_radius_m:
    stop_or_split()
```

### 12.2 不建議用固定半徑限制 coverage

對割草任務來說，固定半徑容易讓不規則區域被截斷。

比較建議使用：

```text
coverage ratio
unvisited free cell count
component 是否完成
```

---

## 13. 對洞與凹形區域的改善

greedy spiral 對洞與凹形區域不佳，原因是它不知道障礙物邊界也需要被覆蓋。

建議加入：

```python
obstacle_boundary_mask = safe_map & dilate(~safe_map)
covered_boundary_mask = safe_map & dilate(visited_map)
virtual_wall_mask = obstacle_boundary_mask | covered_boundary_mask
```

用途：

```text
1. 沿著 obstacle boundary 補洞周圍
2. 沿著 virtual wall 繼續 coverage
3. 避免過早卡住
```

---

## 14. Spiral 與 Boustrophedon 的分工建議

割草機主 coverage 任務不建議完全依賴 spiral。

比較合理的分工：

```text
大面積規則區域:
    Boustrophedon coverage

洞、邊界、凹形、小區域:
    Spiral / wall-following 補覆蓋

component 之間:
    A* bridge

最後:
    smoothing + collision check
```

### 14.1 為什麼主流程建議 Boustrophedon

| 方法 | 優點 | 缺點 |
|---|---|---|
| Boustrophedon | 覆蓋效率高、割幅容易控制、適合 mower | 對複雜洞與小區域需要補洞 |
| Spiral | 適合局部、不規則小區域 | 全域效率較差，容易卡住 |
| Spiral-STC | coverage 完整性較好 | 實作複雜度較高 |
| Greedy Spiral | 實作簡單 | 不保證完整覆蓋 |

---

## 15. Path Smoothing 建議

不要太早做 smoothing。

建議順序：

```text
raw coverage path
  ↓
split into valid segments
  ↓
simplify waypoints
  ↓
Bezier / spline smoothing
  ↓
collision check
  ↓
if collision:
      reduce smoothing
      or split segment
```

注意：

```text
smoothing 後一定要重新檢查 safe_map
```

因為平滑曲線可能會穿出安全區。

---

## 16. RViz Debug 顯示建議

建議輸出不同 marker：

| 顏色 | 顯示內容 |
|---|---|
| 綠色 | spiral coverage segment |
| 黃色 | bridge / backtrack segment |
| 紅色 | invalid segment |
| 藍色點 | split point |
| 灰色 | visited coverage mask |
| 紫色 | unvisited but reachable area |

---

## 17. 實作優先順序

| 優先級 | 項目 | 目的 |
|---|---|---|
| P0 | `CoveragePlan` 回傳格式 | 讓 planner 可 debug、可視化、可擴充 |
| P0 | `visited_map` / `coverage_mask` | 知道實際覆蓋範圍 |
| P0 | stuck detection | 判斷 greedy spiral 是否卡住 |
| P0 | BFS 找最近 unvisited cell | 避免提前結束 |
| P1 | A* bridge | 安全連接下一個 coverage 區域 |
| P1 | split points | 分段給 Nav2 / RViz 使用 |
| P1 | invalid segments | 避免穿越 unsafe area |
| P1 | connected components | 處理安全區斷裂 |
| P2 | virtual wall / obstacle boundary | 改善洞、凹形區域 |
| P2 | waypoint simplification | 減少不必要 waypoint |
| P2 | smoothing + collision check | 提升差速底盤追蹤穩定性 |
| P3 | Spiral-STC / hybrid CPP | 升級成更完整 coverage planner |

---

## 18. 建議第一版實作範圍

第一版不要做太複雜，先完成：

```text
1. 回傳 CoveragePlan
2. 建立 visited_map
3. spiral 卡住時不要停止
4. BFS 找最近未覆蓋 cell
5. A* bridge 接過去
6. 加入 split_points
7. 加入 invalid_segments
8. RViz 可視化不同 segment
```

第一版完成後，至少可以解決：

```text
- 不規則區域提前停止
- 部分區域漏覆蓋
- 洞附近 coverage 不完整
- 安全區斷裂時硬連線
- 無法 debug 哪裡失敗
```

---

## 19. 建議函式切分

```python
def plan_spiral_coverage(
    safe_map: np.ndarray,
    start_cell: tuple[int, int],
    resolution: float,
    origin_x: float,
    origin_y: float,
    config: SpiralPlannerConfig,
) -> CoveragePlan:
    ...
```

```python
def label_connected_components(safe_map: np.ndarray) -> tuple[np.ndarray, int]:
    ...
```

```python
def run_spiral_on_component(
    component_mask: np.ndarray,
    start_cell: tuple[int, int],
    visited_map: np.ndarray,
    config: SpiralPlannerConfig,
) -> list[CoverageSegment]:
    ...
```

```python
def find_nearest_unvisited_cell_by_bfs(
    current_cell: tuple[int, int],
    safe_map: np.ndarray,
    visited_map: np.ndarray,
) -> tuple[int, int] | None:
    ...
```

```python
def astar_bridge(
    start_cell: tuple[int, int],
    goal_cell: tuple[int, int],
    safe_map: np.ndarray,
) -> list[tuple[int, int]] | None:
    ...
```

```python
def detect_invalid_segments(
    segments: list[CoverageSegment],
    safe_map: np.ndarray,
    config: SpiralPlannerConfig,
) -> list[CoverageSegment]:
    ...
```

---

## 20. 建議 Config

```python
@dataclass
class SpiralPlannerConfig:
    neighbor_mode: str = "4"            # "4" or "8"
    max_steps: int = 100000
    max_radius_m: float | None = None
    max_bridge_length_m: float = 5.0
    min_component_area_m2: float = 0.2
    robot_width_m: float = 0.5
    safety_margin_m: float = 0.1
    enable_astar_bridge: bool = True
    enable_component_split: bool = True
    enable_virtual_wall: bool = False
    enable_smoothing: bool = False
```

---

## 21. 驗收標準

| 項目 | 標準 |
|---|---|
| coverage ratio | 可設定，例如達到 95% 以上 |
| invalid segment | 不可穿越 unsafe cell |
| component split | 多安全區可分段處理 |
| stuck recovery | 卡住後可接到下一個未覆蓋區 |
| RViz debug | 可看出 spiral / bridge / invalid / split point |
| Nav2 可追蹤 | waypoint 數量合理，沒有過密震盪 |
| smoothing 安全性 | smoothing 後仍不穿越 unsafe |

---

## 22. 結論

`spiral.py` 目前不建議只修 greedy spiral。

比較實際的改善方向是：

```text
greedy spiral
  + visited map
  + stuck detection
  + BFS nearest unvisited
  + A* bridge
  + connected components
  + split points
  + invalid segments
  + CoveragePlan output
```

對割草機主流程來說，建議：

```text
Boustrophedon 作為主要 coverage planner
Spiral 作為補洞、小區域、邊界 coverage module
A* bridge 作為區域連接
smoothing 作為最後追蹤優化
```

這樣比單純把 `spiral.py` 改成更大的 greedy spiral 更穩，也比較符合實際 mower coverage path planning 的需求。
