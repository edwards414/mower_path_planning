# Flutter Frontend Function Spec

本文件整理目前 mower path planning 專案已存在的後端功能，給 Flutter 前端設計頁面與資料流使用。內容以目前程式碼為準，主要範圍包含：

- `mower_mission`: 區域記錄、地圖生成、coverage path 生成與執行
- `mower_interface`: service / message / action 定義
- `mower_bringup`: Nav2、twist mux、系統啟動設定
- `mower_teleop`: 鍵盤移動與刀盤 joystick 控制

注意：後端 API 中目前存在 `chennal`、`cencel` 等拼字，Flutter 顯示文案可以修正為 `Channel`、`Cancel`，但呼叫 ROS API 時必須保留原名稱。

## 1. 前端產品目標

Flutter App 主要用途是讓操作員完成一個割草任務：

1. 連線並確認 ROS / Nav2 / Mission 節點狀態。
2. 記錄工作區域、風險區域、通道路徑。
3. 生成 free space、risk map、channel map。
4. 設定 coverage 參數並產生覆蓋路徑。
5. 選擇 zone 執行導航。
6. 監看執行狀態、地圖圖層、路徑、錯誤線段。
7. 必要時取消導航或進入手動控制。

Flutter 建議不要直接執行 shell 指令。建議由一層 ROS adapter 提供 WebSocket / REST API，Flutter 只負責呼叫 adapter、訂閱狀態與渲染資料。

## 2. 建議資訊架構

建議底部或側邊主導覽：

| 頁面 | 目的 | 主要元件 |
|---|---|---|
| Dashboard | 系統總覽 | 節點狀態、任務狀態、最近錯誤、快捷操作 |
| Map | 地圖與圖層 | OccupancyGrid、Polygon、Path、Marker layer toggles |
| Record | 區域記錄 | 工作區、風險區、通道記錄控制 |
| Planning | 地圖生成與 Coverage 規劃 | 參數表單、生成按鈕、規劃結果 |
| Execution | Zone 執行 | Zone selector、開始/取消、分段進度、Nav2 log |
| Manual | 手動控制 | Twist 控制、刀盤控制、安全停止 |
| Settings / Logs | 參數與診斷 | ROS 參數、service log、topic 狀態 |

MVP 可先合併成三頁：`Map + Record`、`Planning`、`Execution + Logs`。

## 3. 共通 UX 規則

- 所有 service call 都是非同步；按下後顯示 loading，不阻塞 UI。
- 同一個 service 在等待回應時，該按鈕 disabled，避免重複送出。
- 成功、失敗、timeout 都寫入操作日誌。
- 任何會移動機器或刀盤的操作，按鈕需有明確安全狀態。
- 地圖圖層需要可開關，避免 coverage、risk、zone 全部堆在一起。
- 後端 Trigger service 有些會立即回應「開始建立」，實際完成要以 topic 是否發布更新判斷。

建議日誌格式：

```text
[HH:mm:ss] [INFO] calling /create_free_space
[HH:mm:ss] [SUCCESS] /create_free_space: 開始創建自由空間...
[HH:mm:ss] [TOPIC] /free_space updated: 320x180, res=0.05
[HH:mm:ss] [ERROR] /generate_coverage_path: 缺少 risk_map_inflated 地圖數據
```

## 4. 任務狀態模型

前端可維護下列狀態，作為按鈕 enable / disable 的依據：

| State | 條件 | 可執行操作 |
|---|---|---|
| `disconnected` | ROS adapter 未連線 | 只顯示重連 |
| `idle` | 已連線，尚未載入 zone | Load / Record |
| `recording_zone` | `/record_zone_start` 成功後 | End Zone |
| `recording_risk` | `/risk_zone_start` 成功後 | End Risk |
| `recording_channel` | `/chennal_record_start` 成功後 | End Channel |
| `zones_loaded` | `/load_zone_list` 或 `/get_record_zone_list` 成功 | Create Free Space |
| `free_space_ready` | 收到 `/free_space` | Create Risk Map / Channel Map |
| `risk_map_ready` | 收到 `/risk_map_inflated` | Generate Coverage |
| `coverage_ready` | `/generate_coverage_path` 成功 | Execute Zone |
| `executing` | `/zone_exec_path` 成功送出 goal | Cancel / Status |
| `failed` | 任務失敗或 Nav2 error | Cancel / Replan / Retry |

前端應禁止同時進入多種 recording mode。後端目前只明確阻擋 zone 與 risk 同時記錄，Flutter 端建議也阻擋 channel 與其他記錄模式並行。

## 5. 頁面規格

### 5.1 Dashboard

用途：讓使用者一眼知道系統是否可以開始任務。

必備資訊：

- ROS adapter 連線狀態
- Mission nodes 狀態：`path_recorder`、`map_manage`、`boustrophedon_coverage`、`nav_action_server`
- Nav2 狀態：idle / running / completed / failed
- 最近一次 service result
- 最近 5 筆 warning / error log

快捷操作：

- Load Saved Zones
- Create Free Space
- Generate Coverage
- Cancel Navigation

狀態來源建議：

- service availability check
- `/rosout`
- `/check_nav_status`
- topic heartbeat：`/map_grid`、`/coverage_path_markers`

### 5.2 Map

用途：顯示目前任務區域、地圖、風險區、coverage path、Nav2 分段路徑。

圖層建議：

| 圖層 | ROS Topic | Type | 顯示 |
|---|---|---|---|
| Nav Base Map | `/map_grid` | `nav_msgs/OccupancyGrid` | 0 free、100 occupied |
| Free Space | `/free_space` | `OccupancyGrid` | 工作區原始 free space |
| Free Space Inflated | `/free_space_inflated` | `OccupancyGrid` | 內縮後工作區 |
| Risk Map | `/risk_map` | `OccupancyGrid` | 風險區 |
| Risk Map Inflated | `/risk_map_inflated` | `OccupancyGrid` | 膨脹後風險區 |
| Channel Map | `/chennal_map` | `OccupancyGrid` | 通道區 |
| Channel Map Inflated | `/chennal_map_inflated` | `OccupancyGrid` | 內縮後通道 |
| Current Zone | `/zone_markers` | `visualization_msgs/Marker` | 正在記錄的工作區 |
| Zone List | `/zone_list` | `MarkerArray` | 已記錄工作區 |
| Current Risk | `/risk_zone_markers` | `Marker` | 正在記錄的風險區 |
| Risk List | `/risk_zone_list` | `MarkerArray` | 已記錄風險區 |
| Current Channel | `/chennal_path` | `nav_msgs/Path` | 正在記錄的通道 |
| Channel List | `/chennal_path_array` | `MarkerArray` | 已記錄通道 |
| Coverage Path | `/coverage_path_markers` | `MarkerArray` | 規劃路徑與方向箭頭 |
| Invalid Segments | `/coverage_invalid_segments` | `MarkerArray` | unsafe segment，紅色 |
| Connectors | `/coverage_connectors` | `MarkerArray` | A* connector，黃色 |
| Split Path | `/split_path` | `nav_msgs/Path` | 正在執行的分段 |
| Split Points | `/coverage_split_points` | `Marker` | 分段點 |

OccupancyGrid 座標轉換：

```text
world_x = origin_x + (col + 0.5) * resolution
world_y = origin_y + (row + 0.5) * resolution
```

地圖渲染需求：

- 支援 pan / zoom / robot follow。
- 支援單層 opacity 調整。
- 支援點擊 marker 顯示 id、點數、類型。
- 支援目前 robot pose 顯示，來源可由 ROS adapter 提供 TF `map -> base_footprint`。

### 5.3 Record

用途：讓使用者沿著機器人軌跡記錄工作區、風險區與通道。

功能區塊：

#### 工作區 Zone

| UI | Service | Type | 說明 |
|---|---|---|---|
| Start Zone | `/record_zone_start` | `std_srvs/Trigger` | 開始記錄工作區 polygon |
| End Zone | `/record_zone_end` | `std_srvs/Trigger` | 結束記錄並加入 `/zone_list` |
| Save All | `/save_zone_list` | `std_srvs/Trigger` | 儲存工作區、風險區、channel |
| Load Zones | `/load_zone_list` | `std_srvs/Trigger` | 載入工作區與風險區 |
| Zone Info | `/get_record_zone_info` | `std_srvs/Trigger` | 目前只回傳測試訊息 |
| Get Zone List | `/get_record_zone_list` | `mower_interface/srv/GetZoneList` | 回傳 MarkerArray |

#### 風險區 Risk Zone

| UI | Service | Type | 說明 |
|---|---|---|---|
| Start Risk | `/risk_zone_start` | `std_srvs/Trigger` | 開始記錄風險區 polygon |
| End Risk | `/risk_zone_end` | `std_srvs/Trigger` | 結束記錄並加入 `/risk_zone_list` |
| Get Risk List | `/get_risk_zone_list` | `mower_interface/srv/GetZoneList` | 回傳 MarkerArray |

注意：`path_record_node.py` 有 `risk_zone_save_srv()` 與 `risk_zone_load_srv()` 函式，但目前沒有 `create_service()` 註冊 `/risk_zone_save` 或 `/risk_zone_load`。Flutter 不要把這兩個當作可用 service。

#### 通道 Channel

| UI | Service | Type | 說明 |
|---|---|---|---|
| Start Channel | `/chennal_record_start` | `std_srvs/Trigger` | 開始記錄 channel path |
| End Channel | `/chennal_record_end` | `std_srvs/Trigger` | 結束並加入 `/chennal_path_array` |
| Get Channel List | `/get_chennal_path_list` | `mower_interface/srv/ChennalPathList` | 回傳 MarkerArray |

注意：`/save_zone_list` 會儲存 channel json；但目前 `/load_zone_list` 只載入工作區與風險區，沒有呼叫 `_load_chennal_path_list()`。前端若需要 channel 載入功能，要先補後端。

### 5.4 Planning

用途：生成 free space / risk map / channel map，設定 coverage 參數並產生 coverage path。

#### 地圖生成

| UI | Service | Type | 前置條件 | 結果 Topic |
|---|---|---|---|---|
| Create Free Space | `/create_free_space` | `std_srvs/Trigger` | 已記錄或載入 zone | `/free_space`, `/free_space_inflated`, `/map_grid` |
| Create Risk Map | `/create_risk_map` | `std_srvs/Trigger` | free space 已建立 | `/risk_map`, `/risk_map_inflated` |
| Create Channel Map | `/create_chennal_map` | `std_srvs/Trigger` | free space 已建立，且有 channel | `/chennal_map`, `/chennal_map_inflated` |

MapManage 參數：

| Parameter | Node | Default | Flutter 控制 |
|---|---|---:|---|
| `inflate_radius_m` | `/map_manage` | `0.55` | double input / slider，0.0-3.0 m |
| `chennal_width_m` | `/map_manage` | `0.6` | double input / slider，0.1-3.0 m |

#### Coverage 參數

| Parameter | Node | Default | Flutter 控制 |
|---|---|---:|---|
| `strip_width_m` | `/boustrophedon_coverage` | `0.8` | double input，0.01-5.0 m |
| `waypoint_spacing_m` | `/boustrophedon_coverage` | `0.2` | double input，0.01-2.0 m |
| `unknown_as_obstacle` | `/boustrophedon_coverage` | `true` | switch |
| `min_safe_component_area_m2` | `/boustrophedon_coverage` | `0.05` | advanced double input |
| `coverage_pattern` | `/boustrophedon_coverage` | `zigzag` | segmented control: `zigzag`, `spiral` |

參數套用方式：

- 呼叫 `/boustrophedon_coverage/set_parameters`
- 呼叫 `/map_manage/set_parameters`
- 全部成功後才呼叫 `/generate_coverage_path`

Coverage 生成：

| UI | Service | Type | 前置條件 | Response |
|---|---|---|---|---|
| Generate Coverage | `/generate_coverage_path` | `std_srvs/Trigger` | 已收到 `/risk_map_inflated`，已有 zone map | `success`, `message` |

生成成功後：

- 後端將 path 存在 `zone_map_list[i].path`
- 後端將 split points 存在 `zone_map_list[i].coverage_split_points`
- 發布 `/coverage_path_markers`
- 若有 unsafe segment，發布 `/coverage_invalid_segments`
- 若 connector planner 修復成功，發布 `/coverage_connectors`

Flutter 顯示建議：

- 顯示 coverage pattern 與參數摘要。
- 顯示 zone 數量、每個 zone 是否有 path。
- 顯示 invalid segment 數量。
- invalid segment > 0 時，地圖自動打開紅色圖層。

### 5.5 Execution

用途：選擇 zone 並送出導航執行，監看狀態與錯誤。

| UI | Service / Action | Type | Request | 說明 |
|---|---|---|---|---|
| Execute Zone | `/zone_exec_path` | `mower_interface/srv/ZoneExecPath` | `zone_id: int32` | 執行指定 zone path |
| Cancel Navigation | `/cencel_nav2` | `std_srvs/Trigger` | 無 | 取消 Nav2 任務 |
| Check Nav Status | `/check_nav_status` | `std_srvs/Trigger` | 無 | 查詢 BasicNavigator 狀態 |
| Follow Path Action | `/nav_action_follow_path` | `mower_interface/action/Waypoint` | `path`, `coverage_split_points` | 由後端內部呼叫 |
| Nav Action | `/nav_action` | `mower_interface/action/Waypoint` | `path`, `coverage_split_points` | 舊版逐段 path action |

`ZoneExecPath` request：

```json
{
  "zone_id": 1
}
```

`Waypoint.action`：

```text
# Goal
nav_msgs/Path path
geometry_msgs/Pose[] coverage_split_points
---
# Result
bool success
---
# Feedback
string feedback
```

`nav_action_server` 目前執行流程：

1. 先導航到 coverage path 起點。
2. 依 coverage split points 切段。
3. 再依大轉角與最大長度切段。
4. 每段 publish `/split_path`。
5. 使用 Nav2 `followPath()` 執行。
6. 失敗時讀取 `/rosout` 中 Nav2 warning/error 摘要。

Execution 頁建議欄位：

- Zone selector：從 `/get_zone_map_list_srv` 或 coverage result 建立。
- Start button：呼叫 `/zone_exec_path`。
- Cancel button：呼叫 `/cencel_nav2`。
- Current segment：由 `/split_path` 更新。
- Distance remaining / speed：若 adapter 能接 Nav2 feedback，顯示。
- Error panel：顯示 `/rosout` 中 `controller_server`、`planner_server`、`bt_navigator`、`behavior_server` 錯誤。

### 5.6 Manual

用途：測試或緊急操作。這頁要比其他頁更保守，避免誤觸。

#### 移動控制

目前 `mower_teleop.teleop_keyboard` 是終端機鍵盤節點：

| Topic | Type | 說明 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | 發布速度命令 |

參數：

| Parameter | Default |
|---|---:|
| `max_linear_speed` | `0.8` |
| `max_angular_speed` | `1.5` |
| `linear_step` | `0.15` |
| `angular_step` | `0.45` |
| `linear_acceleration` | `0.8` |
| `angular_acceleration` | `3.0` |
| `publish_rate` | `20.0` |
| `steering_timeout` | `0.25` |

Flutter 若要做虛擬搖桿，建議透過 adapter 發布 stamped velocity，且加入 deadman switch。專案中 `twist_mux_topics.yaml` 目前列出 `/nav_cmd_vel`、`/joy_cmd`、`/keyboard_cmd_vel`，但 keyboard teleop 現在直接發 `/cmd_vel`，前端實作前應確認最終 mux topic。

#### 刀盤控制

目前 `blade_teleop_joy` 從 joystick `/joy` 讀取輸入，發布刀盤 effort：

| Topic | Type | 說明 |
|---|---|---|
| `/joy` | `sensor_msgs/Joy` | joystick input |
| `/mower_blade_controller/commands` | `std_msgs/Float64MultiArray` | `[blade_command]` |

參數：

| Parameter | Default | 說明 |
|---|---:|---|
| `blade_axis_index` | `5` | 控制刀盤的 joystick axis |
| `axis_released_value` | `1.0` | 放開值 |
| `axis_pressed_value` | `-1.0` | 按到底值 |
| `lock_button_index` | `5` | 鎖定目前刀盤速度 |
| `estop_button_index` | `0` | 刀盤急停 |
| `max_blade_command` | `100.0` | 最大命令 |
| `joy_timeout` | `0.3` | joystick timeout 後歸零 |
| `command_ramp_per_sec` | `200.0` | 命令斜率限制 |

Flutter 若要直接控制刀盤，建議新增專用安全 service，而不是直接發布 `/mower_blade_controller/commands`。MVP 可只顯示 joystick 狀態與刀盤命令，不做手機觸控刀盤控制。

## 6. ROS API 對照表

### 6.1 Services

| Service | Type | Node | 用途 | Flutter 狀態 |
|---|---|---|---|---|
| `/record_zone_start` | `std_srvs/Trigger` | `path_recorder` | 開始工作區記錄 | 使用 |
| `/record_zone_end` | `std_srvs/Trigger` | `path_recorder` | 結束工作區記錄 | 使用 |
| `/risk_zone_start` | `std_srvs/Trigger` | `path_recorder` | 開始風險區記錄 | 使用 |
| `/risk_zone_end` | `std_srvs/Trigger` | `path_recorder` | 結束風險區記錄 | 使用 |
| `/save_zone_list` | `std_srvs/Trigger` | `path_recorder` | 儲存 zone/risk/channel | 使用 |
| `/load_zone_list` | `std_srvs/Trigger` | `path_recorder` | 載入 zone/risk | 使用 |
| `/get_record_zone_info` | `std_srvs/Trigger` | `path_recorder` | 測試資訊 | 可放診斷 |
| `/get_record_zone_list` | `mower_interface/GetZoneList` | `path_recorder` | 取得工作區 MarkerArray | 使用 |
| `/get_risk_zone_list` | `mower_interface/GetZoneList` | `path_recorder` | 取得風險區 MarkerArray | 使用 |
| `/chennal_record_start` | `std_srvs/Trigger` | `path_recorder` | 開始通道記錄 | 使用 |
| `/chennal_record_end` | `std_srvs/Trigger` | `path_recorder` | 結束通道記錄 | 使用 |
| `/get_chennal_path_list` | `mower_interface/ChennalPathList` | `path_recorder` | 取得通道 MarkerArray | 使用 |
| `/create_free_space` | `std_srvs/Trigger` | `map_manage` | 建立 free space | 使用 |
| `/create_risk_map` | `std_srvs/Trigger` | `map_manage` | 建立 risk map | 使用 |
| `/create_chennal_map` | `std_srvs/Trigger` | `map_manage` | 建立 channel map | 使用 |
| `/get_zone_map_list_srv` | `mower_interface/ZoneMapList` | `map_manage` | 取得 ZoneMap[] | 使用 |
| `/generate_coverage_path` | `std_srvs/Trigger` | `boustrophedon_coverage` | 生成 coverage path | 使用 |
| `/zone_exec_path` | `mower_interface/ZoneExecPath` | `boustrophedon_coverage` | 執行 zone path | 使用 |
| `/cencel_nav2` | `std_srvs/Trigger` | `boustrophedon_coverage` | 取消導航 | 使用 |
| `/check_nav_status` | `std_srvs/Trigger` | `boustrophedon_coverage` | 查詢導航狀態 | 使用 |
| `/risk_zone_save` | `std_srvs/Trigger` | 未註冊 | 函式存在但 service 不存在 | 不使用 |
| `/risk_zone_load` | `std_srvs/Trigger` | 未註冊 | 函式存在但 service 不存在 | 不使用 |
| `/record_path_status` | `std_srvs/SetBool` | 未註冊 | coverage_node 有 client / callback，但未建立 service | 不使用 |

### 6.2 Topics

| Topic | Type | Publisher | Flutter 用途 |
|---|---|---|---|
| `/recorded_path` | `nav_msgs/Path` | `path_recorder` | 正在記錄工作區軌跡 |
| `/zone_markers` | `visualization_msgs/Marker` | `path_recorder` | 正在記錄工作區 |
| `/zone_list` | `visualization_msgs/MarkerArray` | `path_recorder` | 已記錄工作區 |
| `/risk_zone_markers` | `visualization_msgs/Marker` | `path_recorder` | 正在記錄風險區 |
| `/risk_zone_list` | `visualization_msgs/MarkerArray` | `path_recorder` | 已記錄風險區 |
| `/chennal_path` | `nav_msgs/Path` | `path_recorder` | 正在記錄通道 |
| `/chennal_path_array` | `visualization_msgs/MarkerArray` | `path_recorder` | 已記錄通道 |
| `/map_grid` | `nav_msgs/OccupancyGrid` | `map_manage` | Nav2 base map / 主地圖 |
| `/free_space` | `nav_msgs/OccupancyGrid` | `map_manage` | 工作區 free space |
| `/free_space_inflated` | `nav_msgs/OccupancyGrid` | `map_manage` | 內縮 free space |
| `/risk_map` | `nav_msgs/OccupancyGrid` | `map_manage` | 風險區 |
| `/risk_map_inflated` | `nav_msgs/OccupancyGrid` | `map_manage` | 膨脹風險區 |
| `/chennal_map` | `nav_msgs/OccupancyGrid` | `map_manage` | 通道地圖 |
| `/chennal_map_inflated` | `nav_msgs/OccupancyGrid` | `map_manage` | 內縮通道地圖 |
| `/coverage_path` | `nav_msgs/Path` | `coverage_node` | publisher 存在，但目前主流程未 publish |
| `/coverage_path_markers` | `visualization_msgs/MarkerArray` | `coverage_node` | coverage 路徑顯示 |
| `/coverage_invalid_segments` | `visualization_msgs/MarkerArray` | `coverage_node` | unsafe segment |
| `/coverage_connectors` | `visualization_msgs/MarkerArray` | `coverage_node` | A* connector |
| `/split_path` | `nav_msgs/Path` | `nav_action_server` | 目前執行中的切段 |
| `/coverage_split_points` | `visualization_msgs/Marker` | `nav_action_server` | 分段點 |
| `/rosout` | `rcl_interfaces/Log` | ROS | 診斷與錯誤 |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | teleop / controller | 移動控制 |
| `/mower_blade_controller/commands` | `std_msgs/Float64MultiArray` | blade teleop | 刀盤命令 |

### 6.3 Interface Schema

`mower_interface/srv/GetZoneList`：

```text
---
bool success
string message
visualization_msgs/MarkerArray zone_list
```

`mower_interface/srv/ZoneMapList`：

```text
---
mower_interface/ZoneMap[] zone_map_list
```

`mower_interface/srv/ZoneExecPath`：

```text
int32 zone_id
---
bool success
string message
```

`mower_interface/srv/ChennalPathList`：

```text
---
bool success
string message
visualization_msgs/MarkerArray chennal_path_array
```

`mower_interface/msg/ZoneMap`：

```text
std_msgs/Header header
int32 zone_id
nav_msgs/OccupancyGrid mask_map
nav_msgs/OccupancyGrid mask_map_inflated
nav_msgs/Path path
geometry_msgs/Pose[] coverage_split_points
```

## 7. Flutter Adapter DTO 建議

前端不要直接綁 ROS message 巢狀結構，建議 adapter 轉成簡潔 DTO。

```json
{
  "ServiceResult": {
    "success": true,
    "message": "覆蓋路徑生成成功"
  },
  "Point2D": {
    "x": 1.23,
    "y": 4.56
  },
  "MapLayer": {
    "name": "risk_map_inflated",
    "type": "occupancy_grid",
    "resolution": 0.05,
    "width": 400,
    "height": 400,
    "origin": {"x": -20.0, "y": -20.0},
    "data": "base64-or-compressed-int8"
  },
  "MarkerLayer": {
    "name": "coverage_invalid_segments",
    "markers": [
      {
        "id": 1,
        "type": "line_strip",
        "color": "#ff0000",
        "points": [{"x": 0.0, "y": 0.0}]
      }
    ]
  },
  "ZoneSummary": {
    "zoneId": 1,
    "pointCount": 52,
    "hasMap": true,
    "hasCoveragePath": true
  },
  "CoverageSettings": {
    "stripWidthM": 0.8,
    "waypointSpacingM": 0.2,
    "inflateRadiusM": 0.55,
    "unknownAsObstacle": true,
    "coveragePattern": "zigzag"
  }
}
```

## 8. 典型使用流程

### 8.1 使用已保存區域

```text
1. /load_zone_list
2. /create_free_space
3. wait /free_space and /free_space_inflated
4. /create_risk_map
5. wait /risk_map_inflated
6. set coverage/map parameters
7. /generate_coverage_path
8. check /coverage_path_markers
9. /zone_exec_path {zone_id}
10. monitor /split_path, /rosout
```

### 8.2 新增工作區

```text
1. /record_zone_start
2. operator drives robot around boundary
3. map page displays /recorded_path and /zone_markers
4. /record_zone_end
5. /save_zone_list
6. /create_free_space
```

### 8.3 新增風險區

```text
1. /risk_zone_start
2. operator drives around risk boundary
3. /risk_zone_end
4. /save_zone_list
5. /create_risk_map
6. /generate_coverage_path
```

### 8.4 執行與失敗恢復

```text
1. /zone_exec_path {zone_id}
2. Execution page enters executing
3. subscribe /split_path and /rosout
4. if Nav2 collision/patience exceeded:
   - show error card
   - offer Cancel
   - offer Re-generate Coverage
   - keep map centered on failed /split_path
```

## 9. 畫面元件清單

建議 Flutter components：

- `RosConnectionBanner`
- `ServiceActionButton`
- `OperationLogPanel`
- `MapCanvas`
- `LayerTogglePanel`
- `ZoneRecordPanel`
- `RiskRecordPanel`
- `ChannelRecordPanel`
- `CoverageSettingsForm`
- `CoverageResultPanel`
- `ZoneSelector`
- `ExecutionStatusPanel`
- `Nav2ErrorCard`
- `ManualTwistPad`
- `BladeStatusPanel`
- `SafetyStopButton`

## 10. 驗收標準

MVP 驗收：

- 可以連線 ROS adapter，顯示 service availability。
- 可以完成 `/load_zone_list -> /create_free_space -> /create_risk_map -> /generate_coverage_path -> /zone_exec_path`。
- 可以顯示 `/map_grid`、`/zone_list`、`/risk_zone_list`、`/coverage_path_markers`、`/coverage_invalid_segments`。
- service call 失敗時，UI 顯示 service 名稱、錯誤訊息、時間。
- coverage 參數可套用到 `/boustrophedon_coverage/set_parameters` 與 `/map_manage/set_parameters`。
- Zone ID 使用整數輸入，不使用字串。
- `/get_chennal_path_list` 使用 `mower_interface/srv/ChennalPathList`，不是 `std_srvs/Trigger`。

進階驗收：

- 地圖支援 layer opacity 與點擊 marker。
- Execution 顯示目前 `/split_path`。
- 讀取 `/rosout` 並顯示 Nav2 最近錯誤。
- 手動控制有 deadman switch，放開即歸零。
- 刀盤控制需二階段確認或專用安全 service。

## 11. 已知限制與待補後端

| 項目 | 現況 | 對 Flutter 的影響 |
|---|---|---|
| Channel load | `_load_chennal_path_list()` 存在但 `/load_zone_list` 未呼叫 | App 載入後 channel 可能不會恢復 |
| Risk save/load service | 函式存在但未註冊 service | 不要做獨立 risk save/load 按鈕 |
| `/coverage_path` | publisher 存在，但主流程未 publish | 前端用 `/coverage_path_markers` 顯示路徑 |
| `/record_path_status` | coverage_node 有 client/callback，但未註冊 service | 不要使用 |
| service progress | Trigger 多半立即回應 | 完成狀態要看 topic 更新 |
| direct manual control | 沒有手機專用安全 service | 建議先只做監看或透過 adapter 加安全層 |
| API 拼字 | `chennal`, `cencel` | UI 文案可修正，API 名稱不可改 |

