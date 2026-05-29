# Docking Server Integration Spec

本文件規劃如何在目前 mower path planning 專案新增 docking server。目標不是重寫一套底層 docking 控制，而是把 Nav2 Jazzy 已有的 `opennav_docking` 接進現有 `mower_bringup`、`mower_mission`、Flutter adapter 流程。

目前專案已使用 ROS 2 Jazzy、Nav2、`twist_mux`、`mower_mission` 任務節點與 `mower_interface` 自訂介面。Docking 建議獨立成一個任務能力，讓割草 coverage 完成後可以進入「回充 / 離站」流程，也讓 Flutter 可以單獨操作 docking。

## 1. 已確認決策

依目前硬體與產品需求，本版 docking 設計採用下列決策：

| 項目 | 決策 | 對規格的影響 |
|---|---|---|
| Dock 方向 | 倒車進 dock | `dock_direction: "backward"`，final approach 需允許倒車 |
| 充電狀態 | 目前沒有 `sensor_msgs/BatteryState` | MVP 成功只能代表「已停入 dock pose」，不能保證正在充電 |
| 接觸偵測 | 沒有 wheel effort / motor current | 不使用 stall detection，不能靠馬達堵轉判斷 docked |
| Dock 偵測 | Dock 可貼 AprilTag；可用前相機、後相機 | 實車版優先用後相機 AprilTag，避免倒車 final approach 變成盲退 |
| Dock pose | 希望 Flutter 現場錄製 | 需要新增 record / save / reload dock database 流程 |
| 刀盤安全 | 目前應該沒有硬體 interlock | Docking 前必須軟體停止刀盤；無人自動回充需等硬體 interlock 補上 |

## 2. 設計結論

建議採用兩層架構：

| Layer | 責任 | 建議實作 |
|---|---|---|
| Low-level docking | 接收 dock goal、導航到 staging pose、最後慢速靠 dock、undock | Nav2 `opennav_docking` 的 `docking_server` |
| Mission wrapper | 給 mower 任務與 Flutter 使用的高階 API、狀態整理、安全前置檢查 | 新增 `mower_mission.docking_manager_node` |

不要把 docking 邏輯塞進 `coverage_node.py` 或 `nav_action_server.py`。Coverage 是「割草路徑任務」，Nav action server 是「follow path 執行器」，Docking 是另一種長時間任務，應該和 coverage 平行存在。

MVP 建議分兩步做：

- M1：已知 dock 位置 + 低速 blind smoke test，只驗證 action、launch、remap、倒車方向。
- M2：後相機 AprilTag + `detected_dock_pose`，作為實車 docking 的預設方案。
- 目前不依賴電池狀態、不使用 stall detection。
- 使用 pose threshold 判斷 docked；文件與 UI 必須寫成「Docked」，不要寫「Charging confirmed」。
- 速度限制比 coverage 更保守。

第二階段再加：

- `sensor_msgs/BatteryState` 充電狀態。
- `joint_states` effort / velocity stall detection。
- 硬體刀盤 interlock。

## 3. 現有系統接點

目前相關檔案：

| 檔案 | 現況 | Docking 影響 |
|---|---|---|
| `Dockerfile` | `ARG ROS_DISTRO=jazzy` | Jazzy 可用 Nav2 docking package |
| `.devcontainer/apt-dev-packages` | 已安裝 Nav2 bringup、twist mux 等 | 需補 `opennav_docking` 套件 |
| `src/mower_bringup/launch/navigation.launch.py` | 啟動 Nav2 lifecycle nodes | 需新增 `docking_server` Node 並加入 lifecycle |
| `src/mower_bringup/config/nav2_no_map_params.yaml` | Nav2 controller / planner / costmap config | 建議加入 `docking_server` 參數 |
| `src/mower_bringup/config/twist_mux_topics.yaml` | `/nav_cmd_vel`、`/joy_cmd`、`/keyboard_cmd_vel` | Docking server 的 `cmd_vel` 要 remap 到 `/nav_cmd_vel` |
| `src/mower_mission/launch/mission.launch.py` | 啟動 path record / map manage / coverage | 可新增 docking manager |
| `src/mower_interface` | 自訂 msg/srv/action | 若要包裝成 mower API，可新增 docking action/srv |

重要注意：`opennav_docking` 會發布相對 topic `cmd_vel`。本專案目前使用 `twist_mux`，Nav2 controller 經由 velocity smoother 發到 `/nav_cmd_vel`，再由 mux 輸出 `/cmd_vel`。因此 docking server 必須 remap：

```python
remappings=remappings + [('cmd_vel', '/nav_cmd_vel')]
```

否則 docking 可能直接發布 `/cmd_vel`，繞過 mux 的手動介入優先權。

## 4. 使用者流程

### 4.1 完成割草後回 dock

```text
1. Coverage execution completed
2. Mission layer checks:
   - no active coverage action
   - blade command is stopped
   - Nav2 lifecycle active
   - docking_server active
   - selected dock exists
3. Send /dock_robot action goal
4. docking_server navigates to staging pose
5. docking_server performs slow final approach
6. result success -> mower state = docked
7. 目前沒有 BatteryState，因此不宣稱 charging confirmed
8. result fail -> mower state = docking_failed, allow retry / manual
```

### 4.2 從 dock 離站

```text
1. Operator presses Undock
2. Mission layer checks blade off and no active coverage
3. Send /undock_robot action goal
4. docking_server backs out / exits to staging pose
5. result success -> mower state = idle_ready
```

### 4.3 Flutter 操作

Flutter 不建議直接發 `/dock_robot` action。建議透過 ROS adapter 或 `mower_mission.docking_manager_node` 暴露高階 API：

- `Dock Home`
- `Undock`
- `Cancel Docking`
- `Docking Status`
- `Reload Dock Database`

Flutter 畫面需明確顯示 docking 狀態、錯誤碼、retry 次數、目前是否允許手動接管。

## 5. ROS API 規格

### 5.1 Nav2 原生 API

`opennav_docking` 提供：

| API | Type | 用途 |
|---|---|---|
| `/dock_robot` | `nav2_msgs/action/DockRobot` | Dock 到 database 中的 dock，或 goal 直接指定 dock pose |
| `/undock_robot` | `nav2_msgs/action/UndockRobot` | 從 dock 離站 |
| `/docking_server/reload_database` | `nav2_msgs/srv/ReloadDockDatabase` | 重新載入 dock database |

`DockRobot` goal 重點：

```text
bool use_dock_id
string dock_id
geometry_msgs/PoseStamped dock_pose
string dock_type
float32 max_staging_time
bool navigate_to_staging_pose
```

MVP 只使用：

```yaml
use_dock_id: true
dock_id: home_dock
max_staging_time: 120.0
navigate_to_staging_pose: true
```

`DockRobot` result 重點：

```text
bool success
uint16 error_code
uint16 num_retries
```

常見失敗分類：

| error_code | 意義 |
|---:|---|
| `901` | dock id 不在 database |
| `902` | dock pose / type 不合法 |
| `903` | 無法到 staging pose |
| `904` | 無法偵測 dock |
| `905` | final approach 控制失敗 |
| `906` | docking 後未偵測到 charging |
| `999` | unknown |

### 5.2 Mower wrapper API

建議新增 `mower_mission.docking_manager_node`，內部當 `/dock_robot` 與 `/undock_robot` action client，外部提供 mower 專用 API。

MVP 可以先用 service：

| Service | Type | 用途 |
|---|---|---|
| `/mower_dock_home` | `std_srvs/Trigger` | Dock 到預設 `home_dock` |
| `/mower_undock` | `std_srvs/Trigger` | 從目前 dock 離站 |
| `/mower_cancel_docking` | `std_srvs/Trigger` | 取消 docking / undocking goal |
| `/mower_docking_status` | `std_srvs/Trigger` | 回傳目前狀態摘要 |

進階版建議在 `mower_interface` 新增 action：

```text
# Dock.action goal
string dock_id
bool navigate_to_staging_pose
float32 max_staging_time
---
bool success
string message
uint16 error_code
uint16 num_retries
---
string state
builtin_interfaces/Duration docking_time
uint16 num_retries
```

這樣 Flutter 可以取得持續 feedback，而不只是按鈕按下後等待一個 Trigger response。

### 5.3 Docking 狀態模型

| State | 來源 | 說明 |
|---|---|---|
| `idle` | wrapper internal | 沒有 docking goal |
| `nav_to_staging_pose` | `/dock_robot` feedback | 正在去 staging pose |
| `initial_perception` | `/dock_robot` feedback | 正在取得 dock pose |
| `controlling` | `/dock_robot` feedback | final approach |
| `wait_for_charge` | `/dock_robot` feedback | 等待充電確認 |
| `retry` | `/dock_robot` feedback | retry 中 |
| `docked` | result success | 已 docked |
| `undocking` | `/undock_robot` active | 離站中 |
| `failed` | result failure | Dock / undock 失敗 |
| `canceling` | cancel requested | 正在取消 |

## 6. 參數與設定檔

### 6.1 新增 docking config

建議先直接加到 `src/mower_bringup/config/nav2_no_map_params.yaml`，因為目前 `navigation.launch.py` 已經把同一份 params 傳給 Nav2 lifecycle nodes。

實車預設範例：倒車進 dock，後相機 AprilTag 輔助 final approach。

```yaml
docking_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 30.0
    initial_perception_timeout: 3.0
    wait_charge_timeout: 5.0
    dock_approach_timeout: 25.0
    undock_linear_tolerance: 0.05
    undock_angular_tolerance: 0.10
    max_retries: 2
    base_frame: "base_footprint"
    fixed_frame: "odom"
    odom_topic: "/odometry/global"
    odom_duration: 0.3
    dock_prestaging_tolerance: 0.5
    introspection_mode: "disabled"

    dock_plugins: ["mower_reverse_dock"]
    mower_reverse_dock:
      plugin: "opennav_docking::SimpleChargingDock"
      staging_x_offset: -0.8
      # Backward docking without rotate_to_dock should stage facing the
      # opposite direction of the dock pose. Confirm this in sim/field tests.
      staging_yaw_offset: 3.14159
      docking_threshold: 0.08
      use_external_detection_pose: true
      detector_service_name: ""
      detector_service_timeout: 5.0
      subscribe_toggle: false
      external_detection_timeout: 1.0
      external_detection_translation_x: -0.20
      external_detection_translation_y: 0.0
      external_detection_rotation_roll: -1.57
      external_detection_rotation_pitch: 1.57
      external_detection_rotation_yaw: 0.0
      filter_coef: 0.1
      use_battery_status: false
      use_stall_detection: false
      dock_direction: "backward"
      rotate_to_dock: false

    docks: ["home_dock"]
    home_dock:
      type: "mower_reverse_dock"
      frame: "map"
      pose: [0.0, 0.0, 0.0]
      id: "home_dock"

    controller:
      k_phi: 3.0
      k_delta: 2.0
      v_linear_min: 0.04
      v_linear_max: 0.12
      v_angular_max: 0.45
      slowdown_radius: 0.35
      deceleration_max: 1.0
      rotate_to_heading_angular_vel: 0.45
      rotate_to_heading_max_angular_accel: 1.2
      use_collision_detection: true
      costmap_topic: "local_costmap/costmap_raw"
      footprint_topic: "local_costmap/published_footprint"
      transform_tolerance: 0.1
      projection_time: 1.0
      simulation_time_step: 0.1
      dock_collision_threshold: 0.3
```

第一次 wiring smoke test 若還沒有 AprilTag detector，可暫時改成：

```yaml
use_external_detection_pose: false
```

此時只能驗證 launch、action、倒車方向、速度與 cancel，不建議直接靠實體充電座測試。

若 docking station 不是真的充電座，或目前想避免 UI 誤以為「已確認充電」，可把 plugin 換成：

```yaml
plugin: "opennav_docking::SimpleNonChargingDock"
```

目前沒有電池狀態，所以即使用 `SimpleChargingDock` 且 `use_battery_status: false`，action success 也只能視為「抵達 docked pose」，不是「充電已開始」。

### 6.2 Dock database 獨立檔案

若後續 Flutter 要新增/修改 dock pose，建議改成獨立 database：

`src/mower_bringup/config/dock_database.yaml`

```yaml
docks:
  home_dock:
    type: "mower_reverse_dock"
    frame: "map"
    pose: [0.0, 0.0, 0.0]
    id: "home_dock"
```

然後 docking config 改用：

```yaml
dock_database: "/mower_ws/install/mower_bringup/share/mower_bringup/config/dock_database.yaml"
```

注意：install 後的 share path 不適合手動硬編在 source tree。正式實作時應在 launch file 用 `get_package_share_directory('mower_bringup')` 組路徑，或由 launch argument 傳入。

## 7. Launch 規格

### 7.1 修改 `navigation.launch.py`

`lifecycle_nodes` 加入 `docking_server`：

```python
lifecycle_nodes = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'velocity_smoother',
    'bt_navigator',
    'waypoint_follower',
    'docking_server',
]
```

非 composition 模式新增 Node：

```python
Node(
    package='opennav_docking',
    executable='opennav_docking',
    name='docking_server',
    output='screen',
    respawn=use_respawn,
    respawn_delay=2.0,
    parameters=[configured_params],
    arguments=['--ros-args', '--log-level', log_level],
    remappings=remappings + [('cmd_vel', '/nav_cmd_vel')],
)
```

Composition 模式新增：

```python
ComposableNode(
    package='opennav_docking',
    plugin='opennav_docking::DockingServer',
    name='docking_server',
    parameters=[configured_params],
    remappings=remappings + [('cmd_vel', '/nav_cmd_vel')],
)
```

### 7.2 修改 `mission.launch.py`

若新增 `docking_manager_node`，建議加入：

```python
docking_manager_node = Node(
    package='mower_mission',
    executable='docking_manager_node',
    name='docking_manager_node',
    output='screen',
)
```

Mission launch 不負責啟動 `opennav_docking` 本體。Docking server 應跟 Nav2 一起由 `mower_bringup` 啟動，因為它是 Nav2 lifecycle task server。

## 8. Package 變更

### 8.1 Docker / devcontainer

`.devcontainer/apt-dev-packages` 建議新增：

```text
ros-$ROS_DISTRO-opennav-docking
```

若要使用 BT plugin：

```text
ros-$ROS_DISTRO-opennav-docking-bt
```

### 8.2 `mower_bringup/package.xml`

新增：

```xml
<exec_depend>opennav_docking</exec_depend>
```

### 8.3 `mower_mission/package.xml`

若實作 wrapper action client，新增：

```xml
<exec_depend>action_msgs</exec_depend>
<exec_depend>nav2_msgs</exec_depend>
```

### 8.4 `mower_mission/setup.py`

新增 console script：

```python
'docking_manager_node = mower_mission.docking_manager_node:main',
'temp_dock_pose_publisher = mower_mission.temp_dock_pose_publisher:main',
```

## 9. Dock pose 建立方式

需求已確認：Flutter 需要能在現場錄製 dock pose。MVP 可以先人工填 `home_dock.pose` 做 bringup smoke test，但正式操作流程應由 Flutter 觸發後端錄製。

建議現場標定流程：

```text
1. 手動把車停在 docked final pose
2. 確認 TF map -> base_footprint 穩定
3. 讀取 base_footprint 在 map 座標的 x, y, yaw
4. 寫入 dock database:
   home_dock.pose = [x, y, yaw]
5. 手動把車移到 staging pose 附近
6. 呼叫 /dock_robot 測試
```

建議新增 `/record_home_dock_pose` service：

- 讀取 `map -> base_footprint`
- 寫入 `dock_database.yaml`
- 呼叫 reload database service
- Flutter 顯示「已更新 dock pose」

建議再新增 `/get_dock_database` 或由 adapter 提供 dock list DTO，讓 Flutter 地圖可以顯示：

- docked final pose
- staging pose
- AprilTag id / detector source
- dock type：`mower_reverse_dock`

注意：倒車進 dock 時，`home_dock.pose` 應代表車子完成 docking 後的 final docked pose，而不是 staging pose。`staging_x_offset` 與 `staging_yaw_offset` 由 docking plugin 從 final pose 推導。

## 10. 安全需求

Docking 屬於會移動機器的任務，前置條件需比一般 service 嚴格。

必備檢查：

| 條件 | 規格 |
|---|---|
| 刀盤 | Dock / undock 前必須停止 |
| Mission | coverage 不可執行中 |
| Manual override | `/joy_cmd`、`/keyboard_cmd_vel` 在 `twist_mux` priority 高於 docking |
| Velocity | final approach `v_linear_max <= 0.12 m/s` 起步 |
| TF | `map -> base_footprint`、`odom -> base_footprint` 可查 |
| Costmap | local costmap active |
| Rear detection | 實車 docking 預設需後相機 AprilTag 可用；blind 模式只做 smoke test |
| Docked 判斷 | 目前只能用 pose threshold / AprilTag，不可宣稱已確認充電 |
| Stall detection | 目前沒有 wheel effort / current，不啟用 `use_stall_detection` |
| Cancel | Flutter 與後端都能 cancel docking action |
| Timeout | staging、approach 都需 timeout；charging timeout 目前只作相容參數 |
| Blade software lock | 因無硬體 interlock，Dock / undock 前軟體必須停止刀盤並確認命令為 0 |

不建議 MVP 直接做「低電量自動回充」。目前沒有電池狀態與硬體 interlock，因此第一版應限定為有人監看的 Flutter 手動 Dock Home。無人自動回充需等至少補上電池狀態與刀盤硬體 interlock。

## 11. Flutter / Adapter 規格

Docking 頁面建議新增在 Execution 旁，或 Dashboard 增加 Dock quick action。

UI 元件：

| 元件 | 功能 |
|---|---|
| Dock selector | 選 `home_dock` 或其他 dock |
| Dock Home button | 呼叫 `/mower_dock_home` 或 adapter action |
| Undock button | 呼叫 `/mower_undock` |
| Cancel button | cancel docking action |
| Record Dock Pose button | 現場錄製目前 `map -> base_footprint` 為 docked final pose |
| AprilTag status | 顯示後相機 detector 是否有穩定輸出 `detected_dock_pose` |
| Docking status | 顯示 state、elapsed time、retry |
| Error panel | 顯示 error code 與文字 |
| Dock pose marker | 在地圖上顯示 dock pose 與 staging pose |
| Safety banner | 顯示 blade stopped / manual override / Nav2 active / rear detector active |

Adapter DTO 建議：

```json
{
  "DockingState": {
    "state": "controlling",
    "dockId": "home_dock",
    "elapsedSec": 12.4,
    "numRetries": 0,
    "errorCode": 0,
    "message": "approaching dock"
  },
  "DockSummary": {
    "dockId": "home_dock",
    "frame": "map",
    "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
    "type": "mower_reverse_dock",
    "dockDirection": "backward",
    "detector": "rear_apriltag",
    "chargingConfirmed": false
  }
}
```

## 12. 測試計畫

### 12.1 Build 檢查

```bash
colcon build --packages-select mower_bringup mower_mission mower_interface
```

### 12.2 Launch 檢查

```bash
ros2 launch mower_nav2 navigation.launch.py use_sim_time:=true
ros2 lifecycle get /docking_server
ros2 action list | grep dock
```

預期：

```text
/dock_robot
/undock_robot
```

臨時 mission wrapper 與固定 dock pose publisher：

```bash
ros2 launch mower_mission mission.launch.py launch_temp_dock_pose_publisher:=true
ros2 service list | grep mower_dock
ros2 topic echo /mower_docking_state
ros2 topic echo /detected_dock_pose
```

後相機 AprilTag docking detector：

```bash
ros2 launch mower_bringup apriltag_docking.launch.py \
  image_topic:=/back_camera/image_raw \
  camera_info_topic:=/back_camera/camera_info \
  camera_frame:=back_camera_link
```

或用 Makefile：

```bash
make apriltag-docking
```

整套實車 launch 若要一起開 AprilTag：

```bash
ros2 launch mower_bringup mower.launch.py enable_apriltag_docking:=true
```

RViz / topic 檢查：

```text
/rear_apriltag/detections      # apriltag_ros 偵測結果
/tf home_dock_tag              # AprilTag TF
/detected_dock_pose            # 給 Nav2 docking 的 PoseStamped
/back_camera/image_raw         # Gazebo / rear camera image
/back_camera/camera_info       # Camera calibration info
```

### 12.3 Action smoke test

```bash
ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot \
"{use_dock_id: true, dock_id: 'home_dock', navigate_to_staging_pose: true, max_staging_time: 120.0}"
```

### 12.4 Remap 檢查

```bash
ros2 topic info /nav_cmd_vel
ros2 topic info /cmd_vel
```

Docking final approach 時應該看到 docking server 發到 `/nav_cmd_vel`，再由 `twist_mux` 輸出 `/cmd_vel`。

### 12.5 實車測試順序

```text
1. 架空輪子測 /dock_robot，確認倒車方向、速度上限、cancel
2. 地面無 dock 障礙，測 staging pose navigation
3. 啟用後相機 AprilTag，確認 detected_dock_pose 穩定
4. 使用假 dock / 軟障礙低速測 final approach
5. 測 Flutter record dock pose -> reload database -> 地圖 marker 更新
6. 測 undock
7. 測手動 joystick / keyboard override
8. 測完整 Dock -> Undock 20 次
9. 等 BatteryState 與硬體 interlock 補上後，再測自動回充
```

## 13. 分階段里程碑

| Milestone | 內容 | 驗收 |
|---|---|---|
| M0 Spec | 完成此規格、確認倒車 docking、後相機 AprilTag、Flutter 現場錄製 | 文件確認 |
| M1 Nav2 docking MVP | 新增 docking_server params / launch / dependencies | `/dock_robot`、`/undock_robot` 存在且 lifecycle active |
| M2 Reverse smoke test | 使用 known pose / blind 模式完成低速倒車 smoke test | action 可取消，方向與 remap 正確 |
| M3 Rear AprilTag docking | 後相機發布 `detected_dock_pose`，完成低速 dock / undock | 偵測偏移可調，成功率提升 |
| M4 Mission wrapper | 新增 `docking_manager_node` | Flutter/adapter 可呼叫高階 API |
| M5 Flutter dock recording | Flutter 可錄製、保存、reload dock pose | database 更新後地圖 marker 正確 |
| M6 Real robot validation | 實車低速 dock / cancel / undock | 連續 20 次無失控、可取消 |
| M7 Charging safety | 加 `BatteryState` 與硬體 interlock | 可判斷 charging，才允許評估自動回充 |

## 14. 已確認決策摘要

已確認：

- 車子倒車進 dock，參數使用 `dock_direction: "backward"`。
- 目前沒有電池狀態，因此 action success 不能代表 charging confirmed。
- 目前沒有 wheel effort / motor current，因此不使用 stall detection。
- Dock 可貼 AprilTag，且有前/後相機；實車 docking 優先用後相機 AprilTag。
- Flutter 需要現場錄製 dock pose，因此要實作 record / save / reload database。
- 目前沒有硬體 interlock，因此第一版只允許有人監看的手動 Dock Home，不開無人自動回充。

建議：

- 第一版不做自訂 C++ dock plugin，先用 Nav2 `SimpleChargingDock` 或 `SimpleNonChargingDock`。
- 實車版預設後相機 AprilTag；blind docking 僅作 smoke test。
- 一定把 docking cmd_vel 走 `/nav_cmd_vel -> twist_mux -> /cmd_vel`。
- BatteryState 與硬體 interlock 補上前，Flutter 不要顯示「充電已確認」。

## 15. 參考資料

- Nav2 Using Docking Server: https://docs.nav2.org/tutorials/docs/using_docking.html
- Nav2 Docking Server configuration: https://docs.nav2.org/configuration/packages/configuring-docking-server.html
- Nav2 Jazzy navigation launch includes `docking_server` as lifecycle node: https://api.nav2.org/nav2-jazzy/html/navigation__launch_8py_source.html
- Jazzy `nav2_msgs/action/DockRobot`: https://docs.ros.org/en/jazzy/p/nav2_msgs/action/DockRobot.html
