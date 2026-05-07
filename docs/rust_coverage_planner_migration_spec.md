# Rust Coverage Planner Migration Specification

## 1. 目的

目前 coverage path 規劃核心以 Python 實作，主要包含：

- `mower_mission/path_generators/zigzag.py`
- `mower_mission/path_generators/speiral.py`
- `mower_mission/coverage/path_validator.py`
- `mower_mission/coverage/connector_planner.py`
- `mower_mission/coverage/safe_map_filter.py`
- `mower_mission/coverage/cell_decomposition.py`

本規格定義將 coverage 路徑演算法逐步遷移到 Rust 的方案。目標是提升演算法核心的效能、型別安全與長期可維護性，同時避免一次改動 ROS2 node、Nav2 整合與自定義 message/action 帶來過高風險。

核心原則：

```text
先保留現有 ROS2 行為，再替換純演算法核心。
安全驗證先於效能最佳化。
每一階段都必須可回退到 Python backend。
```

## 2. 建議結論

第一階段不建議直接把 `coverage_node` 改成 Rust ROS2 node。

建議採用：

```text
Python ROS2 node
  -> Python facade
  -> Rust coverage core
```

也就是保留目前 `rclpy`、Nav2、RViz marker、service/action 流程，由 Rust 負責下列純演算法：

- safe map component filtering
- path validation
- connector planning
- zigzag path generation
- spiral path generation
- 後續 BCD / route optimizer / simulated annealing

## 3. 現況摘要

目前 `/generate_coverage_path` 的主要流程在 `coverage_node.py`：

```text
zone map + risk_map_inflated
  -> safe_map
  -> filter_safe_components
  -> zigzag 或 spiral generator
  -> validate_path
  -> connector planner 修補 invalid segment
  -> nav_msgs/Path + coverage_split_points
  -> marker / zone path storage
```

目前 generator 對外契約是：

```python
points, split_points, invalid_segments = generator(...)
```

其中：

| 欄位 | 型別 | 說明 |
| --- | --- | --- |
| `points` | `list[tuple[float, float]]` | world coordinate waypoint |
| `split_points` | `list[tuple[float, float]]` | 後續 NavActionServer 用來切段的覆蓋分割點 |
| `invalid_segments` | `list[tuple[int, int]]` | `points` 中不安全的連續 segment index |

## 4. 遷移目標

### 4.1 必須維持

- `/generate_coverage_path` service name 不變。
- `/zone_exec_path` service name 不變。
- `mower_interface/action/Waypoint.action` 不變。
- `ZoneMap.path` 與 `ZoneMap.coverage_split_points` 語意不變。
- RViz marker topic 不變。
- 現有 Python tests 必須可繼續驗證行為。
- 真機執行時，若 Rust backend 發生錯誤，不能發布未驗證路徑。

### 4.2 可以改善

- 演算法執行時間。
- 大地圖下的記憶體配置。
- A* connector 的效率。
- validate_path 的線段檢查速度。
- 後續 BCD / optimizer 的資料模型。

### 4.3 暫不處理

第一輪遷移不處理：

- 將整個 `coverage_node.py` 改寫成 Rust node。
- 將 `nav_action_server.py` 改寫成 Rust。
- 修改 ROS topic/service/action 名稱。
- 修改 map management pipeline。
- 修改 Nav2 controller / behavior tree。

## 5. 目標架構

### 5.1 Package 佈局

建議新增 Rust core：

```text
src/
  mower_coverage_core/
    Cargo.toml
    pyproject.toml
    package.xml
    src/
      lib.rs
      types.rs
      safe_map.rs
      path_validator.rs
      connector_planner.rs
      zigzag.rs
      spiral.rs
      ffi.rs
```

Python 端保留 facade：

```text
src/mower_mission/mower_mission/
  coverage_backend/
    __init__.py
    python_backend.py
    rust_backend.py
```

`coverage_node.py` 只依賴 facade，不直接依賴 Rust module。

### 5.2 Runtime 選擇

新增 ROS parameter：

```text
coverage_backend: "python" | "rust"
```

建議預設：

```text
coverage_backend = "python"
```

等 Rust backend 通過完整測試與真機 dry-run 後，再切成：

```text
coverage_backend = "rust"
```

### 5.3 錯誤策略

建議採用兩種模式：

| 模式 | 行為 |
| --- | --- |
| development | Rust import 或執行失敗時，可 fallback 到 Python backend，並明確 log warning |
| robot / production | Rust backend 失敗時 fail closed，不發布 coverage path |

可用參數控制：

```text
allow_backend_fallback: true | false
```

真機預設建議：

```text
allow_backend_fallback = false
```

## 6. Rust API 契約

### 6.1 SafeMap

Rust 內部建議使用 row-major `Vec<u8>`：

```rust
pub struct SafeMap {
    pub width: usize,
    pub height: usize,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
    pub data: Vec<u8>, // 1 = safe, 0 = unsafe
}
```

注意：

- 不建議使用 `Vec<bool>` 作為跨語言資料，避免 bit packing 與 FFI 語意問題。
- Python `np.ndarray bool/u8` 傳入 Rust 時必須保證 row-major contiguous。
- indexing 必須固定為 `row * width + col`。

### 6.2 Point

```rust
pub struct Point2 {
    pub x: f64,
    pub y: f64,
}
```

### 6.3 Plan Output

```rust
pub struct CoveragePlan {
    pub points: Vec<Point2>,
    pub split_points: Vec<Point2>,
    pub invalid_segments: Vec<(usize, usize)>,
}
```

Python wrapper 對外仍轉成：

```python
list[tuple[float, float]],
list[tuple[float, float]],
list[tuple[int, int]]
```

### 6.4 座標轉換規則

必須完全對齊目前 Python 行為：

```text
col = int((x - origin_x) / resolution)
row = int((y - origin_y) / resolution)
world_x = origin_x + (col + 0.5) * resolution
world_y = origin_y + (row + 0.5) * resolution
```

Rust 需要特別注意：

- Python `int(float)` 對正數等同 truncate，對負數是 toward zero。
- 地圖 origin 可能為負值。
- 邊界 cell 不可因 `floor()` / cast 差異產生 off-by-one。

建議明確寫測試覆蓋：

- origin 為負數。
- point 剛好在 cell 邊界。
- point 剛好超出地圖。
- resolution 為 `0.05`、`0.1`。

## 7. 分階段計畫

### Phase 0：Baseline 與風險盤點

### 範圍

不改 runtime 行為，只建立遷移前基準。

### 工作項目

- 整理目前 Python backend 的輸入/輸出契約。
- 補齊 generator、validator、connector 的 regression tests。
- 建立幾組固定地圖 fixtures：
  - empty map
  - rectangle map
  - U-shaped map
  - obstacle island
  - disconnected components
  - narrow passage
- 記錄目前執行時間與 path 統計。

### 驗收標準

- `pytest src/mower_mission/test` 通過。
- 每個 fixture 都保存：
  - `points` 數量
  - `split_points` 數量
  - `invalid_segments` 數量
  - final `validate_path` 結果
- 文件化目前限制。

### 回退策略

此階段無 runtime 改動，不需要回退。

### Phase 1：Backend Facade 抽象層

### 範圍

仍使用 Python 演算法，但讓 `coverage_node.py` 不直接 import 具體 generator。

### 工作項目

- 新增 `coverage_backend/python_backend.py`。
- 將目前 `zigzag` / `spiral` 呼叫包成統一介面：

```python
generate_coverage_plan(
    pattern: str,
    safe_map,
    strip_width_m: float,
    waypoint_spacing_m: float,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> tuple[list, list, list]
```

- 新增 `coverage_backend/__init__.py` 負責依參數選 backend。
- `coverage_node.py` 改成呼叫 backend facade。

### 驗收標準

- 預設 backend 為 Python 時，行為與現況一致。
- 所有現有 tests 通過。
- `/generate_coverage_path` 仍可正常產生 path。

### 回退策略

保留原本 generator import 一個 commit 內可還原。

### Phase 2：Rust Core Skeleton

### 範圍

建立 Rust crate 與 Python binding，但暫不替換主要演算法。

### 工作項目

- 新增 `mower_coverage_core`。
- 建立 PyO3/maturin 或 setuptools-rust build 流程。
- 實作最小 function：

```text
version()
echo_plan_input_shape()
```

- 新增 `coverage_backend/rust_backend.py`，確認 Python 可 import Rust module。
- CI / colcon build 流程能安裝 Rust extension。

### 驗收標準

- `python -c "import mower_coverage_core"` 成功。
- colcon build 可建出 Python package 與 Rust extension。
- `coverage_backend="rust"` 時至少能進入 Rust backend 並回報尚未支援的 pattern。

### 回退策略

`coverage_backend="python"` 完全不依賴 Rust extension。

### Phase 3：Port PathValidator

### 範圍

先把安全驗證移到 Rust，因為它是後續所有演算法的安全底線。

### 工作項目

- Port：
  - `is_point_safe`
  - `is_segment_safe`
  - `validate_points`
  - `validate_path`
  - Bresenham line rasterization
- Python facade 保留同名 wrapper。
- 針對 Python 與 Rust validator 做同輸入比較。

### 驗收標準

- `test_path_validator.py` 對 Python/Rust backend 都通過。
- 邊界案例結果完全一致。
- Rust validator 不允許 out-of-bounds segment 被視為 safe。

### 回退策略

若 Rust validator 結果與 Python 不一致，維持 Python validator 作為 production backend。

### Phase 4：Port SafeMapFilter

### 範圍

移植 safe component filtering。

### 工作項目

- Port `filter_safe_components`。
- 8-connected component BFS 結果需與 Python 對齊。
- 回傳：
  - filtered safe map
  - all component sizes
  - kept component sizes

### 驗收標準

- `test_safe_map_filter.py` 對 Python/Rust backend 都通過。
- disconnected components、tiny fragments、keep largest only 行為一致。

### 回退策略

`coverage_node.py` 可單獨選擇 validator 使用 Rust、filter 仍使用 Python。

### Phase 5：Port ConnectorPlanner

### 範圍

移植 grid A* connector planner。

### 工作項目

- Port：
  - 8-connected A*
  - diagonal corner-cutting check
  - boundary distance cost
  - world/cell conversion
- 回傳 world-coordinate connector points。
- 保留 Python API：

```python
plan_connector(start, end, safe_map, boundary_weight=0.2)
```

### 驗收標準

- `test_connector_planner.py` 對 Python/Rust backend 都通過。
- connector 每一段必須通過 Rust `validate_path`。
- unreachable case 必須回傳 `None`，不能產生冒險直線。

### 回退策略

若 Rust A* 與 Python path 不完全一致，可接受，但必須滿足：

- path valid
- start/end preserved
- path length 不明顯劣化
- no corner cutting

### Phase 6：Port Zigzag Generator

### 範圍

移植目前實際常用的 zigzag coverage generator。

### 工作項目

- Port `angle_deg == 0.0` 分支。
- 保留 `angle_deg != 0.0` 分支是否 port 的決策：
  - 若主流程仍固定 0 度，可先不支援非 0 度。
  - 若 UI 或後續 optimizer 會使用角度搜尋，需一起 port。
- 回傳 `points`、`split_points`、`invalid_segments`。

### 驗收標準

- `test_zigzag_generator.py` 通過。
- `test_zigzag_segment_boundaries.py` 通過。
- Rust 生成結果的 final `validate_path` 結果不差於 Python。
- `split_points` 能被 `nav_action_server.py` 正確切段。

### 回退策略

可透過 `coverage_backend="python"` 立即回到 Python zigzag。

### Phase 7：Port Spiral Generator

### 範圍

移植 onion-layer spiral generator 與 component bridge 行為。

### 工作項目

- Port：
  - connected components
  - BFS distance transform
  - layer centerline selection
  - contour ordering
  - coverage mask marking
  - component bridge
- 維持 `plan_spiral_coverage` 的 rich output 能力，至少內部保留 debug metadata。

### 驗收標準

- `test_spiral_generator.py` 通過。
- disconnected islands、obstacle gap、coverage mask 相關測試通過。
- `split_points` 仍代表 spiral segment 結束點。

### 回退策略

Rust backend 可先只支援 `zigzag`，`spiral` 繼續走 Python，直到測試全部通過。

### Phase 8：Production Switch

### 範圍

將 Rust backend 設為預設，但 Python backend 仍保留。

### 工作項目

- launch/config 中將：

```text
coverage_backend = "rust"
allow_backend_fallback = false
```

- 增加 runtime log：
  - backend name
  - generator duration
  - validator duration
  - connector count
  - invalid segment count
- 在模擬與真機 dry-run 中比較 Python/Rust 結果。

### 驗收標準

- 模擬流程完整通過：

```text
/create_free_space
/create_risk_map
/generate_coverage_path
/zone_exec_path
```

- Rust backend 不發布 unsafe path。
- RViz marker 與 path execution 行為正常。
- 執行時間相較 Python 有明確改善，或至少沒有明顯退步。

### 回退策略

配置改回：

```text
coverage_backend = "python"
```

即可回到原本 Python 演算法。

### Phase 9：Optional Rust ROS2 Node

### 範圍

這是長期選項，不屬於第一輪遷移。

### 可能工作項目

- 評估 `rclrs` 在目標 ROS2 distribution 的成熟度。
- 評估自定義 `mower_interface` msg/srv/action 的 Rust binding。
- 改寫 `coverage_node` 為 Rust node。
- Python `nav_action_server.py` 可保留，或另行評估 Rust 化。

### 啟動條件

只有在以下條件成立時才建議進入：

- Rust coverage core 已穩定。
- Python/Rust backend 已長期對照測試。
- 需要降低 Python runtime dependency。
- 團隊願意維護 ROS2 Rust build chain。

## 8. 測試策略

### 8.1 單元測試

每個 Rust module 都需要對應 Python 測試：

| 模組 | 測試 |
| --- | --- |
| path validator | point/segment/path safety |
| safe map filter | component filtering |
| connector planner | reachable/unreachable/corner-cutting |
| zigzag | strip generation/split/invalid segments |
| spiral | components/layers/coverage mask |

### 8.2 差異測試

Rust 與 Python 不一定要每個 waypoint 完全相同，但要比較：

- path 是否 valid。
- split point 數量是否合理。
- coverage ratio 是否不低於 Python。
- connector length 是否不明顯增加。
- invalid segment 數量是否不增加。

### 8.3 Integration Test

至少需要覆蓋：

```text
record zone
create free space
create risk map
generate coverage path
execute zone path
```

### 8.4 Performance Test

建議記錄：

- map size
- safe cells count
- generator duration
- validator duration
- connector planner duration
- total `/generate_coverage_path` duration

## 9. 主要風險

### 9.1 座標 off-by-one

這是最高風險。Rust 與 Python 對 float-to-int 的行為若不同，會造成 waypoint 被判定到不同 cell。

處理策略：

- 明確測試負 origin。
- 明確測試 cell boundary。
- Python/Rust validator 必須在 Phase 3 完全對齊。

### 9.2 split point 導航切段失效

`nav_action_server.py` 依靠 split point 與 path waypoint 的距離 tolerance 切段。

處理策略：

- Rust generator 回傳的 split point 必須來自實際 waypoint 或在 tolerance 內。
- Integration test 必須驗證 split path 數量。

### 9.3 Rust build chain 增加部署複雜度

ROS2 workspace 目前以 Python/CMake package 為主，新增 Rust 後會增加工具鏈要求。

處理策略：

- Phase 2 先驗證 colcon build。
- 保留 Python backend。
- 文件化 Rust toolchain version。

### 9.4 A* path 不完全一致

Rust priority queue tie-breaker 可能讓 A* 路徑和 Python 不同。

處理策略：

- 不要求 connector waypoint 完全一致。
- 要求 path valid、無 corner cutting、長度合理。

### 9.5 真機安全

任何 backend 都不能在 validator 失敗後發布 path。

處理策略：

- final validation 必須保留在 `coverage_node.py` 層。
- Rust generator 輸出後仍要走 final `validate_path`。
- production 模式禁止 silent fallback。

## 10. 驗收總表

| 階段 | 可合併條件 |
| --- | --- |
| Phase 0 | baseline tests 與 fixtures 完成 |
| Phase 1 | Python backend facade 不改變現有行為 |
| Phase 2 | Rust extension 可被 Python import，build 流程穩定 |
| Phase 3 | Rust validator 與 Python validator 行為一致 |
| Phase 4 | Rust safe map filter 通過 component tests |
| Phase 5 | Rust connector 產生 valid safe path |
| Phase 6 | Rust zigzag 通過現有測試與 integration smoke test |
| Phase 7 | Rust spiral 通過現有測試 |
| Phase 8 | Rust backend 可設為預設並通過模擬/真機 dry-run |
| Phase 9 | 僅在長期維護需求明確時啟動 |

## 11. 建議實作順序

建議優先順序：

```text
1. Backend facade
2. Rust crate skeleton
3. PathValidator
4. SafeMapFilter
5. ConnectorPlanner
6. Zigzag generator
7. Spiral generator
8. Production switch
9. Optional Rust ROS2 node
```

這個順序讓每一步都有清楚的安全驗收，也讓 Python backend 隨時可作為 fallback。
