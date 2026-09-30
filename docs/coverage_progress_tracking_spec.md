# Coverage Progress Tracking Spec

> **2026-09-30 補記：** 本文寫作時 coverage 節點是 Python 的 `mower_mission/coverage_node.py`，這個檔案已在 2026-09-30 移除。同一個節點（node 名 `boustrophedon_coverage`，服務與 topic 不變）現在是 `mower_rs` 的 `mower_coverage`，程式在 `src/mower_rs/crates/mower_coverage/src/lib.rs`。下文要改 `coverage_node.py` 的項目，現在要改那個檔案。現況描述保留原文。

> **2026-09-30 M1 已完成：** `mower_interface` 新增 `msg/CoverageProgress.msg`（欄位同 5.1）與 `srv/GetCoverageProgress.srv`；`Waypoint.action` goal 加了 `int32 zone_id -1`（`mower_coverage` 填 zone id，通道路線為 -1）。沒有另加 `mission_id` 欄位：每次執行本來就有唯一的 `dispatch_id`，`mission_id` 直接用它。兩個導航伺服器（rclpy `nav_action_server` 與 `mower_rs` 的 `mower_nav`）都發 latched `/coverage_progress` 並提供 `/coverage_progress_status`，計算與狀態轉換照第 6 節；另外三點：段內進度不倒退（Nav2 的剩餘距離會抖動）、回饋驅動的訊息最多每 0.5 s 一筆（狀態轉換立即發）、`current_pose` 取自 `/adapter/robot_pose`。取消途中若 run 以 abort 結束，最終狀態依導航狀態判為 `canceled`。邏輯在 `mower_mission/navigation/coverage_progress.py` 與 `mower_rs/crates/mower_nav/src/progress.rs`（兩份要同步，單元測試同一組情境），`src/mower_rs/tools/nav_progress_check.py` 對兩個伺服器用假 Nav2 做黑箱檢查（CI 會跑）。兩個 topic/service 已加入 app 白名單（rosbridge、ws_bridge）。7.2 的 `/check_nav_status` 改寫與 7.5 的 Qt 按鈕沒做；M2 checkpoint、M3 resume 尚未開始。

本文件規劃如何在目前割草 coverage path 執行時，記錄、發布、查詢目前工作進度。目標是讓前端能顯示割草進度，也讓後續中斷恢復任務有可靠基礎。

## 1. 背景與現況

目前 coverage 流程如下：

```text
/generate_coverage_path
  -> coverage_node.py 產生每個 zone 的 nav_msgs/Path
  -> 暫存在 zone_map_list[i].path 與 zone_map_list[i].coverage_split_points

/zone_exec_path
  -> coverage_node.py 找到指定 zone
  -> NavActionClient.send_goal_split_path(...)
  -> nav_action_server.py 執行 nav_action_follow_path action
  -> nav_action_server.py 將 path 依 split point、轉角、最大長度切成多段
  -> 逐段呼叫 BasicNavigator.followPath(...)
```

目前已存在的資訊：

| 資訊 | 現況 | 限制 |
|---|---|---|
| path 與 split point | 存在 `coverage_node.zone_map_list` | 只在記憶體，節點重啟會消失 |
| 正在執行第幾段 | `nav_action_server.py` log 會印 `執行第 idx/total 段` | 只有 log，前端與其他節點無法穩定讀取 |
| 目前 split path | 發布到 `/split_path` | 只是一段 path 視覺化，不含百分比與任務狀態 |
| `/check_nav_status` | 在 `coverage_node.py` 查 `self.nav.isTaskComplete()` | 實際 coverage follow path 是 `nav_action_server.py` 內的 `BasicNavigator` 在跑，這個 service 不是真正的 coverage progress source |
| `/record_path_status` client | `coverage_node.py` 有建立 client | 目前沒有被 coverage execution 啟用，且這是記錄區域邊界用，不是割草工作進度 |

結論：目前沒有正式的割草工作進度 topic、service、checkpoint 或恢復機制。

## 2. 目標

MVP 需要完成：

1. 割草 path 開始執行時，建立一個 coverage mission progress state。
2. 執行過程中持續發布 `/coverage_progress`。
3. 提供 `/coverage_progress_status` service 給 Qt / Flutter 查詢。
4. 進度需包含 zone、目前段落、總段落、完成百分比、已完成距離、剩餘距離、狀態與訊息。
5. 取消、失敗、成功時要發布最後狀態。

第二階段再做：

1. 將 progress checkpoint 寫入磁碟。
2. 節點重啟後可讀回最後 checkpoint。
3. 同一條 coverage path 可從未完成段落繼續。

本 spec 先規劃完整架構，但實作建議分 M1 / M2 / M3。

## 3. 非目標

第一版不做：

- 不保證割草刀盤實際覆蓋率，只追蹤導航 path 執行進度。
- 不做地圖 cell-level coverage heatmap。
- 不做跨 zone 自動排程。
- 不做低電量自動回充。
- 不做實車斷電後完整 mission resume；這屬於 M3。

## 4. 設計決策

| 項目 | 決策 |
|---|---|
| Progress owner | `nav_action_server.py` |
| 原因 | 只有 nav action server 知道 path 被切成幾個實際 followPath segment，也知道每段是否完成 |
| ROS 介面 | 在 `mower_interface` 新增 typed msg/srv |
| Topic | `/coverage_progress` |
| Service | `/coverage_progress_status` |
| 進度百分比 | 以實際執行 split path 距離計算，不用 waypoint 數量 |
| 路徑起點導航 | `Nav to coverage start` 狀態不算割草進度，overall progress 仍為 0% |
| 目前段落 | UI 使用 1-based，例如第 1/20 段；內部可用 0-based |
| checkpoint | M2 寫 JSON 到參數指定路徑 |

## 5. ROS Interface

### 5.1 新增 `mower_interface/msg/CoverageProgress.msg`

```text
uint8 STATUS_IDLE=0
uint8 STATUS_NAVIGATING_TO_START=1
uint8 STATUS_RUNNING=2
uint8 STATUS_CANCELING=3
uint8 STATUS_CANCELED=4
uint8 STATUS_SUCCEEDED=5
uint8 STATUS_FAILED=6

std_msgs/Header header

uint8 status
string status_text
string mission_id
int32 zone_id

int32 current_segment_index
int32 total_segments
int32 completed_segments

float32 current_segment_progress
float32 overall_progress

float32 current_segment_distance_m
float32 completed_distance_m
float32 total_distance_m
float32 remaining_distance_m

geometry_msgs/PoseStamped current_pose
geometry_msgs/PoseStamped current_segment_start
geometry_msgs/PoseStamped current_segment_goal

bool checkpoint_available
string message
```

欄位約定：

| 欄位 | 約定 |
|---|---|
| `overall_progress` | 0.0 到 1.0 |
| `current_segment_progress` | 0.0 到 1.0 |
| `current_segment_index` | UI 友善的 1-based；沒有 active segment 時為 0 |
| `completed_segments` | 已完成段數 |
| `total_segments` | 實際 split 後段數 |
| `remaining_distance_m` | `max(total_distance_m - completed_distance_m, 0)` |
| `mission_id` | 每次 `/zone_exec_path` 產生一個唯一 ID，方便前端辨識新任務 |
| `checkpoint_available` | M1 固定 false；M2 後依磁碟 checkpoint 判斷 |

### 5.2 新增 `mower_interface/srv/GetCoverageProgress.srv`

```text
---
bool success
string message
mower_interface/CoverageProgress progress
```

### 5.3 修改 `mower_interface/action/Waypoint.action`

目前 Goal 只有 path 與 split points：

```text
nav_msgs/Path path
geometry_msgs/Pose[] coverage_split_points
```

建議新增 `zone_id` 與 `mission_id`：

```text
int32 zone_id
string mission_id
nav_msgs/Path path
geometry_msgs/Pose[] coverage_split_points
---
bool success
---
string feedback
```

原因：

- `nav_action_server.py` 是 progress owner，但目前 action goal 沒有 zone id。
- `coverage_node.py` 在 `/zone_exec_path` 時知道 zone id，應明確傳給 action server。
- `mission_id` 可以由 `coverage_node.py` 產生，也可以由 `nav_action_server.py` 在空字串時補上。

## 6. Progress 計算方式

### 6.1 總距離

`nav_action_server.py` 已有 `_path_distance(path)`。在切完 `split_paths` 後：

```python
segment_distances = [_path_distance(p) for p in split_paths]
total_distance_m = sum(segment_distances)
```

### 6.2 每段進度

Nav2 feedback 可能提供：

- `distance_to_goal`
- `distance_remaining`

現有 `_feedback_distance(feedback)` 已處理這兩種欄位。

目前段落進度：

```python
remaining = feedback_distance
done_in_segment = clamp(segment_distance - remaining, 0.0, segment_distance)
current_segment_progress = done_in_segment / segment_distance
completed_distance_m = completed_before_current + done_in_segment
overall_progress = completed_distance_m / total_distance_m
```

若 Nav2 feedback 沒有距離欄位：

- current segment progress 保持上一筆值。
- 每段完成時直接跳到 1.0。

### 6.3 狀態轉換

```text
IDLE
  -> NAVIGATING_TO_START
  -> RUNNING
  -> SUCCEEDED

RUNNING
  -> CANCELING
  -> CANCELED

NAVIGATING_TO_START or RUNNING
  -> FAILED
```

起點導航階段：

- `status = STATUS_NAVIGATING_TO_START`
- `overall_progress = 0.0`
- `current_segment_index = 0`

每段 followPath 階段：

- `status = STATUS_RUNNING`
- `current_segment_index = idx`
- `total_segments = len(split_paths)`

成功：

- `status = STATUS_SUCCEEDED`
- `completed_segments = total_segments`
- `overall_progress = 1.0`
- `remaining_distance_m = 0.0`

取消：

- `status = STATUS_CANCELED`
- 保留最後一筆完成距離與段落。

失敗：

- `status = STATUS_FAILED`
- 保留最後一筆完成距離與段落。

## 7. Implementation Plan

### M1：即時進度 topic / service

#### 7.1 `mower_interface`

新增：

```text
src/mower_interface/msg/CoverageProgress.msg
src/mower_interface/srv/GetCoverageProgress.srv
```

修改：

```text
src/mower_interface/CMakeLists.txt
src/mower_interface/package.xml
src/mower_interface/action/Waypoint.action
```

`CMakeLists.txt` 需把新 msg/srv 加入 `rosidl_generate_interfaces(...)`。

#### 7.2 `coverage_node.py`

修改 `/zone_exec_path`：

```python
mission_id = f'zone-{zone_id}-{time_ns()}'
self._action_client_split_path.send_goal_split_path(
    zone_id=zone_id,
    mission_id=mission_id,
    path=zone_map.path,
    coverage_split_points=zone_map.coverage_split_points,
)
```

修改 `/check_nav_status`：

- 不再直接用 `self.nav.isTaskComplete()` 當 coverage 狀態。
- 改成呼叫 `/coverage_progress_status`，或在 `coverage_node.py` 訂閱 `/coverage_progress` 保存最後一筆 snapshot。
- 回傳 message 可先用 JSON 字串，維持 `std_srvs/Trigger` 相容 Qt 現有 service client。

#### 7.3 `NavActionClient`

修改：

```python
def send_goal_split_path(self, zone_id, mission_id, path, coverage_split_points):
    goal_msg.zone_id = int(zone_id)
    goal_msg.mission_id = str(mission_id)
    goal_msg.path = path
    goal_msg.coverage_split_points = coverage_split_points
```

建議新增 `feedback_callback`，接收 action feedback log；M1 的主要資料仍以 `/coverage_progress` 為準。

#### 7.4 `nav_action_server.py`

新增 publisher / service：

```python
self.coverage_progress_pub = self.create_publisher(
    CoverageProgress, '/coverage_progress', 10
)
self.create_service(
    GetCoverageProgress,
    '/coverage_progress_status',
    self.coverage_progress_status_srv,
)
```

新增 helper：

```python
self.current_progress = CoverageProgress()
self._publish_progress(...)
self._reset_progress(...)
self._update_progress_from_feedback(...)
```

在 `single_path_execute_callback()` 中插入：

1. 收到 goal 後初始化 progress。
2. 導航到起點前發布 `NAVIGATING_TO_START`。
3. split_paths 計算完成後填入 `total_segments`、`total_distance_m`。
4. 每段開始時發布 `RUNNING`。
5. `_wait_for_nav_task()` loop 內依 feedback 更新距離與百分比。
6. 每段成功後 `completed_segments += 1`。
7. 任務成功、取消、失敗時發布 final state。

建議把 `_wait_for_nav_task()` 改成接收 optional callback：

```python
def _wait_for_nav_task(
    self,
    goal_handle,
    task_name,
    success_distance_m=None,
    feedback_callback=None,
):
    ...
    if feedback and feedback_callback is not None:
        feedback_callback(feedback)
```

#### 7.5 Qt / Flutter

Qt MVP：

- 在 Coverage 群組新增「刷新割草進度」按鈕。
- 呼叫 `/coverage_progress_status`。
- 顯示 `status_text`、`zone_id`、`current_segment_index/total_segments`、`overall_progress * 100`。

Flutter MVP：

- 若已有 ROS adapter，訂閱 `/coverage_progress`。
- UI 顯示進度條、目前 zone、狀態文字、剩餘距離。

## 8. M2：Checkpoint 磁碟紀錄

新增 `nav_action_server.py` 參數：

| Parameter | Default | 用途 |
|---|---|---|
| `progress_checkpoint_enabled` | `true` | 是否寫 checkpoint |
| `progress_checkpoint_path` | `~/.ros/mower_mission/coverage_progress.json` | checkpoint 檔案 |
| `progress_checkpoint_interval_sec` | `1.0` | 最小寫檔間隔 |

checkpoint JSON：

```json
{
  "version": 1,
  "mission_id": "zone-3-1780000000000",
  "zone_id": 3,
  "status": "running",
  "current_segment_index": 5,
  "total_segments": 24,
  "completed_segments": 4,
  "completed_distance_m": 18.42,
  "total_distance_m": 104.9,
  "overall_progress": 0.1756,
  "path_hash": "sha256...",
  "updated_at": "2026-05-07T12:34:56+08:00"
}
```

`path_hash` 計算來源：

- path pose x/y/z
- coverage_split_points x/y
- `zone_id`

用途：

- 防止 path 重新生成後內容不同，卻錯誤從舊 checkpoint resume。
- M2 只紀錄，不自動 resume。

寫檔時機：

- progress 初始化
- 每段完成
- 每秒最多一次
- final state

## 9. M3：Resume 未完成任務

新增 service：

```text
mower_interface/srv/ResumeCoverage.srv
```

建議格式：

```text
int32 zone_id
bool resume_from_checkpoint
---
bool success
string message
```

Resume 流程：

```text
1. 使用者呼叫 /resume_coverage
2. coverage_node 讀 checkpoint
3. 確認 zone_id 相同
4. 確認目前 zone path hash 相同
5. 將 resume_segment_index 傳給 nav_action_server
6. nav_action_server 跳過 completed segments
7. 從 checkpoint 的 current_segment_index 開始執行
```

要修改 `Waypoint.action`：

```text
int32 resume_segment_index
```

M3 注意事項：

- 如果機器人目前離 resume segment 起點太遠，要先 `goToPose(segment_start)`。
- 如果 checkpoint path hash 不一致，必須拒絕 resume，要求重新生成 path。
- resume 只保證從段落開始，不保證從段落中間精準接續。

## 10. Cancel 行為修正

目前 `/cencel_nav2` 在 `coverage_node.py` 中呼叫的是 `self.nav.cancelTask()`，但實際 coverage followPath 是 `nav_action_server.py` 內的 `self.navigator` 在跑。因此取消服務需要修正。

M1 建議：

- `NavActionClient` 保存 active goal handle。
- `/cencel_nav2` 呼叫 action goal cancel。
- `nav_action_server.py` 收到 `goal_handle.is_cancel_requested` 後：
  - 發布 `STATUS_CANCELING`
  - `navigator.cancelTask()`
  - `goal_handle.canceled()`
  - 發布 `STATUS_CANCELED`

也可以另新增 `/cancel_coverage` service 放在 `nav_action_server.py`，但 action cancel 比較符合 ROS action 模型。

## 11. Test Plan

### 11.1 Unit tests

新增：

```text
src/mower_mission/test/test_coverage_progress_tracker.py
```

測試項目：

1. 3 段 path 總距離計算正確。
2. 第 1 段剩餘距離從 10m 到 4m 時，segment progress 為 60%。
3. 完成第 1 段後，completed_segments = 1。
4. overall_progress 使用距離加權，不使用段數平均。
5. 成功時 overall_progress = 1.0。
6. 取消與失敗時保留最後 progress。
7. checkpoint JSON 寫出欄位完整。
8. path_hash 不同時不可 resume。

### 11.2 Interface build

```bash
colcon build --packages-select mower_interface mower_mission
```

### 11.3 Runtime smoke test

Terminal A：

```bash
ros2 launch mower_nav2 navigation.launch.py use_sim_time:=true
```

Terminal B：

```bash
ros2 launch mower_mission mission.launch.py
```

Terminal C：

```bash
ros2 topic echo /coverage_progress
```

操作：

```bash
ros2 service call /generate_coverage_path std_srvs/srv/Trigger {}
ros2 service call /zone_exec_path mower_interface/srv/ZoneExecPath "{zone_id: 0}"
```

預期：

1. `/coverage_progress` 先出現 `NAVIGATING_TO_START`。
2. 開始 follow path 後變成 `RUNNING`。
3. `current_segment_index` 從 1 開始增加。
4. `overall_progress` 單調增加。
5. 完成後 `STATUS_SUCCEEDED` 且 `overall_progress = 1.0`。

取消測試：

```bash
ros2 service call /cencel_nav2 std_srvs/srv/Trigger {}
```

預期：

- `/coverage_progress` 最後狀態為 `STATUS_CANCELED`。

## 12. File Change Checklist

M1 必改：

- `src/mower_interface/msg/CoverageProgress.msg`
- `src/mower_interface/srv/GetCoverageProgress.srv`
- `src/mower_interface/action/Waypoint.action`
- `src/mower_interface/CMakeLists.txt`
- `src/mower_interface/package.xml`
- `src/mower_mission/mower_mission/navigation/nav_action_server.py`
- `src/mower_mission/mower_mission/utils/nav_action_client.py`
- `src/mower_mission/mower_mission/coverage_node.py`
- `src/mower_mission/test/test_coverage_progress_tracker.py`

M1 可選：

- `src/mower_qt/mower_qt/mower_qt.py`
- `src/mower_qt/package.xml`

M2 必改：

- `src/mower_mission/mower_mission/navigation/progress_checkpoint.py`
- `src/mower_mission/test/test_coverage_progress_checkpoint.py`

M3 必改：

- `src/mower_interface/srv/ResumeCoverage.srv`
- `src/mower_interface/action/Waypoint.action`
- `src/mower_mission/mower_mission/coverage_node.py`
- `src/mower_mission/mower_mission/navigation/nav_action_server.py`

## 13. 建議實作順序

1. 先做 `CoverageProgress` msg 與 `/coverage_progress_status` service。
2. 在 `nav_action_server.py` 發布即時 progress。
3. 修正 `/check_nav_status`，讓它回傳 progress snapshot。
4. 修正 `/cencel_nav2`，改用 action cancel。
5. Qt 先加手動刷新進度，不急著做 topic subscription。
6. 確認 M1 穩定後再加 checkpoint。
7. 最後才做 resume。

## 14. Acceptance Criteria

M1 完成條件：

- 執行 `/zone_exec_path` 後，`/coverage_progress` 會立即發布任務狀態。
- 開始割草後，前端或 `ros2 topic echo` 能看到目前第幾段、總段數、百分比。
- 任務成功時進度為 100%。
- 任務取消或失敗時保留最後進度與狀態。
- `/check_nav_status` 不再回傳錯誤來源的 Nav2 狀態。
- 單元測試通過。

M2 完成條件：

- 任務執行時會產生 checkpoint JSON。
- 節點重啟後可查到最後 checkpoint。
- path hash 不一致時明確拒絕 resume。

M3 完成條件：

- 使用同一 zone 與同一路徑時，可從未完成 segment 重新開始。
- resume 前會先導航到 resume segment 起點。
- resume 的 progress 不會從 0% 重算。
