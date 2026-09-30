# Mower Package Refactor Specification

> **2026-09-30 補記：** 本文是 package 重構當時的規格。其中的 `mower_mission/coverage_node.py`、`path_generators/` 與 `coverage_node` / `boustrophedon_coverage` 兩個 executable 已隨 Python coverage 一起移除。coverage 規劃節點現在是 `mower_rs` 的 `mower_coverage`（`ros2 run mower_rs mower_coverage`，node 名仍是 `boustrophedon_coverage`，服務不變），`mission.launch.py` 會自動啟動它（`rust_daemon:=true` 時改由 `mower_rsd` 的 `coverage` 模組提供），所以下文的 `ros2 run mower_mission coverage_node` 已不適用。其餘內容保留原文。

## 1. 目的

目前專案中任務、地圖、路徑記錄、bringup、interface 分散在多個 package：

- `boustrophedon_coverage`
- `maphub`
- `path_record`
- `boustrophedon_coverage_interfaces`
- `path_record_interface`
- `nav2_action_interfaces`
- `nav2_gps_waypoint_follower`

本次重構目標是讓 package 邊界更清楚：

- 任務邏輯集中到 `mower_mission`
- 自定義 msg/srv/action 集中到 `mower_interface`
- 啟動、Nav2、定位、模擬/真機 bringup 集中到 `mower_bringup`

第一階段只做結構整理與 import/package 名稱遷移，盡量不改 runtime 行為、topic/service/action 名稱。

## 2. 命名決策

建議目標 package 名稱：

| 類型 | 目前名稱 | 目標名稱 |
| --- | --- | --- |
| 任務邏輯 | `boustrophedon_coverage`, `maphub`, `path_record` | `mower_mission` |
| 自定義介面 | `boustrophedon_coverage_interfaces`, `path_record_interface`, `nav2_action_interfaces` | `mower_interface` |
| Bringup | `nav2_gps_waypoint_follower` | `mower_bringup` |

待確認：

- 使用 `mower_mission` 還是照原文字 `mower_misson`。
- 使用 `mower_interface` 還是 ROS 慣例較常見的 `mower_interfaces`。
- `Chennal` 是否趁這次改成 `Channel`。第一階段建議先不改，避免 API 同時改名。

## 3. 目標檔案結構

```text
src/
  mower_interface/
    CMakeLists.txt
    package.xml
    msg/
      ZoneMap.msg
    srv/
      GetZoneList.srv
      ZoneMapList.srv
      ZoneExecPath.srv
      ChennalPathList.srv
    action/
      Waypoint.action

  mower_mission/
    package.xml
    setup.py
    setup.cfg
    resource/
      mower_mission
    launch/
      mission.launch.py
    mower_mission/
      __init__.py
      coverage_node.py
      map_manage_node.py
      path_record_node.py
      path_generators/
        __init__.py
        boustrophedon.py
        zigzag.py
        speiral.py
      utils/
        __init__.py
        nav_action_client.py
        zone_map_client.py
        map_hub_client.py
        path_record_utils.py

  mower_bringup/
    package.xml
    setup.py
    setup.cfg
    resource/
      mower_bringup
    launch/
      mower.launch.py
      navigation.launch.py
      dual_ekf_navsat.launch.py
      rviz.launch.py
      twist_mux.launch.py
      gps_world.launch.py
      empty_world.launch.py
      spawn_turtlebot3.launch.py
      robot_state_publisher.launch.py
      small_test.launch.py
      system_test.launch.py
    config/
      nav2_no_map_params.yaml
      dual_ekf_navsat_params.yaml
      twist_mux_topics.yaml
      config.rviz
      turtlebot3_burger_cam_gps_bridge.yaml
    models/
    worlds/
    urdf/
```

## 4. Package 職責

### `mower_interface`

只放 ROS interface，不放 node runtime 程式。

從舊 package 搬移：

| 目前 interface | 目標 interface |
| --- | --- |
| `boustrophedon_coverage_interfaces/msg/ZoneMap` | `mower_interface/msg/ZoneMap` |
| `boustrophedon_coverage_interfaces/srv/GetZoneList` | `mower_interface/srv/GetZoneList` |
| `boustrophedon_coverage_interfaces/srv/ZoneMapList` | `mower_interface/srv/ZoneMapList` |
| `boustrophedon_coverage_interfaces/srv/ZoneExecPath` | `mower_interface/srv/ZoneExecPath` |
| `path_record_interface/srv/ChennalPathList` | `mower_interface/srv/ChennalPathList` |
| `nav2_action_interfaces/action/Waypoint` | `mower_interface/action/Waypoint` |

`ZoneMapList.srv` 內部型別要同步更新：

```srv
---
mower_interface/ZoneMap[] zone_map_list
```

### `mower_mission`

負責任務層節點與任務資料處理：

- coverage path 生成與執行
- zone/risk zone/channel path 記錄
- free space/risk map/channel map 建立
- zone map 管理
- coverage path 與 nav action client 串接

從舊 package 搬移：

| 目前 package/file | 目標位置 |
| --- | --- |
| `boustrophedon_coverage/boustrophedon_coverage.py` | `mower_mission/coverage_node.py` |
| `boustrophedon_coverage/path_generators/*` | `mower_mission/path_generators/*` |
| `boustrophedon_coverage/utils/nav_action_client.py` | `mower_mission/utils/nav_action_client.py` |
| `boustrophedon_coverage/utils/zone_map_client.py` | `mower_mission/utils/zone_map_client.py` |
| `maphub/map_manage.py` | `mower_mission/map_manage_node.py` |
| `maphub/utils/map_hub _client.py` | `mower_mission/utils/map_hub_client.py` |
| `path_record/path_record.py` | `mower_mission/path_record_node.py` |
| `path_record/path_record_utils.py` | `mower_mission/utils/path_record_utils.py` |

建議 console scripts：

```text
coverage_node = mower_mission.coverage_node:main
map_manage_node = mower_mission.map_manage_node:main
path_record_node = mower_mission.path_record_node:main
nav_action_client = mower_mission.utils.nav_action_client:main
zone_map_client = mower_mission.utils.zone_map_client:main
```

可選擇保留舊 executable alias 一個版本，降低外部腳本壞掉的機率：

```text
boustrophedon_coverage = mower_mission.coverage_node:main
map_manage = mower_mission.map_manage_node:main
path_record = mower_mission.path_record_node:main
```

### `mower_bringup`

負責啟動真機、Nav2、robot localization、RViz、模擬世界與測試 launch。

從 `nav2_gps_waypoint_follower` 搬移：

- `launch/*`
- `config/*`
- `models/*`
- `worlds/*`
- `urdf/*`
- `MOWER_STARTUP.md`

更新所有 `get_package_share_directory('nav2_gps_waypoint_follower')` 為：

```py
get_package_share_directory('mower_bringup')
```

真機入口：

```bash
ros2 launch mower_bringup mower.launch.py use_sim_time:=false
```

模擬入口可以保留：

```bash
ros2 launch mower_bringup gps_waypoint_follower.launch.py use_sim_time:=true
```

## 5. Runtime API 策略

第一階段建議保持 topic/service/action 名稱不變，只更換 package/import 名稱。

### Services

| 名稱 | 型別 |
| --- | --- |
| `/record_zone_start` | `std_srvs/srv/Trigger` |
| `/record_zone_end` | `std_srvs/srv/Trigger` |
| `/risk_zone_start` | `std_srvs/srv/Trigger` |
| `/risk_zone_end` | `std_srvs/srv/Trigger` |
| `/save_zone_list` | `std_srvs/srv/Trigger` |
| `/load_zone_list` | `std_srvs/srv/Trigger` |
| `/get_record_zone_info` | `std_srvs/srv/Trigger` |
| `/get_record_zone_list` | `mower_interface/srv/GetZoneList` |
| `/get_risk_zone_list` | `mower_interface/srv/GetZoneList` |
| `/chennal_record_start` | `std_srvs/srv/Trigger` |
| `/chennal_record_end` | `std_srvs/srv/Trigger` |
| `/get_chennal_path_list` | `mower_interface/srv/ChennalPathList` |
| `/create_free_space` | `std_srvs/srv/Trigger` |
| `/create_risk_map` | `std_srvs/srv/Trigger` |
| `/create_chennal_map` | `std_srvs/srv/Trigger` |
| `/get_zone_map_list_srv` | `mower_interface/srv/ZoneMapList` |
| `/generate_coverage_path` | `std_srvs/srv/Trigger` |
| `/zone_exec_path` | `mower_interface/srv/ZoneExecPath` |
| `/cencel_nav2` | `std_srvs/srv/Trigger` |
| `/check_nav_status` | `std_srvs/srv/Trigger` |

注意：`/cencel_nav2` 目前拼字是既有 API。第一階段不改，第二階段可新增 `/cancel_nav2` alias。

### Actions

| 名稱 | 型別 |
| --- | --- |
| `/nav_action` | `mower_interface/action/Waypoint` |
| `/nav_action_follow_path` | `mower_interface/action/Waypoint` |

### Topics

| 名稱 | 型別 |
| --- | --- |
| `/recorded_path` | `nav_msgs/msg/Path` |
| `/zone_markers` | `visualization_msgs/msg/Marker` |
| `/zone_list` | `visualization_msgs/msg/MarkerArray` |
| `/risk_zone_markers` | `visualization_msgs/msg/Marker` |
| `/risk_zone_list` | `visualization_msgs/msg/MarkerArray` |
| `/chennal_path` | `nav_msgs/msg/Path` |
| `/chennal_path_array` | `visualization_msgs/msg/MarkerArray` |
| `/free_space` | `nav_msgs/msg/OccupancyGrid` |
| `/free_space_inflated` | `nav_msgs/msg/OccupancyGrid` |
| `/risk_map` | `nav_msgs/msg/OccupancyGrid` |
| `/risk_map_inflated` | `nav_msgs/msg/OccupancyGrid` |
| `/chennal_map` | `nav_msgs/msg/OccupancyGrid` |
| `/chennal_map_inflated` | `nav_msgs/msg/OccupancyGrid` |
| `/map_grid` | `nav_msgs/msg/OccupancyGrid` |
| `/coverage_path` | `nav_msgs/msg/Path` |
| `/split_path` | `nav_msgs/msg/Path` |

## 6. Import 遷移規則

所有程式碼中的 interface import 改成：

```py
from mower_interface.msg import ZoneMap
from mower_interface.srv import GetZoneList, ZoneMapList, ZoneExecPath, ChennalPathList
from mower_interface.action import Waypoint
```

替換來源：

```py
from boustrophedon_coverage_interfaces...
from path_record_interface...
from nav2_action_interfaces...
```

內部 package import 更新範例：

```py
from .utils.nav_action_client import NavActionClient
from .utils.path_record_utils import path_to_marker, simplify_path
```

## 7. 依賴設定

### `mower_interface/package.xml`

需要：

- `ament_cmake`
- `rosidl_default_generators`
- `std_msgs`
- `geometry_msgs`
- `nav_msgs`
- `visualization_msgs`
- `action_msgs`

### `mower_mission/package.xml`

需要：

- `rclpy`
- `std_msgs`
- `std_srvs`
- `geometry_msgs`
- `nav_msgs`
- `visualization_msgs`
- `tf2_ros`
- `mower_interface`
- `python3-opencv`

### `mower_bringup/package.xml`

需要：

- `nav2_bringup`
- `robot_localization`
- `robot_state_publisher`
- `xacro`
- `twist_mux`
- `joy`
- `teleop_twist_joy`
- `mower_controller`
- `mower_description`
- `mower_teleop`
- `wit_ros2_imu`
- `mower_mission`
- `nav_robot`

## 8. Launch 遷移

`mower_bringup/mower.launch.py` 是真機主入口，預設：

```py
DeclareLaunchArgument('use_sim_time', default_value='false')
```

需要確認所有真機節點都吃到 `use_sim_time:=false`：

- `robot_state_publisher`
- `ekf_filter_node_odom`
- `ekf_filter_node_map`
- `navsat_transform`
- Nav2 nodes

已知風險：

- `nav2_no_map_params.yaml` 目前有 `local_costmap.use_sim_time: True`，重構時要改成 `False` 或移除，讓 launch 統一控制。

建議新增 mission launch：

```py
# mower_mission/launch/mission.launch.py
path_record_node
map_manage_node
coverage_node
```

`system_test.launch.py`、`small_test.launch.py` 內的指令要改為：

```bash
ros2 run mower_mission path_record_node
ros2 run mower_mission map_manage_node
ros2 run mower_mission coverage_node
```

若保留 alias，可暫時不改 executable 名稱。

## 9. 外部引用需要同步更新

必改：

- `README.md`
- `Dockerfile`
- `Makefile`
- `docker-compose.yaml`
- `src/mower_qt/*`
- `src/nav_robot/*`
- `src/mower_description/launch/gazebo.launch.py`
- `src/mower_description/launch/sim.launch.py`
- RViz config 中顯示名稱可選擇更新，topic 名稱第一階段保持不變。

Dockerfile 入口要從：

```dockerfile
CMD ["ros2", "launch", "nav2_gps_waypoint_follower", "mower.launch.py"]
```

改成：

```dockerfile
CMD ["ros2", "launch", "mower_bringup", "mower.launch.py"]
```

## 10. 建議遷移順序

1. 新增 `mower_interface`，複製所有 msg/srv/action，更新內部型別名稱。
2. 更新使用 interface 的 package import，先讓舊 runtime package 可依賴 `mower_interface` build 過。
3. 新增 `mower_mission`，搬移 `path_record`、`maphub`、`boustrophedon_coverage` 程式碼。
4. 在 `mower_mission` 中先保留舊 executable alias。
5. 新增 `mower_bringup`，搬移 `nav2_gps_waypoint_follower` 的 launch/config/assets。
6. 更新所有 launch、docs、Dockerfile、Makefile 的 package 名稱。
7. 確認 `colcon build` 通過後，移除舊 package 或保留一版相容殼。

## 11. 驗收條件

Build：

```bash
colcon build --packages-select mower_interface
colcon build --packages-up-to mower_mission
colcon build --packages-up-to mower_bringup
```

Interface：

```bash
ros2 interface show mower_interface/msg/ZoneMap
ros2 interface show mower_interface/srv/ZoneMapList
ros2 interface show mower_interface/action/Waypoint
```

真機 bringup：

```bash
ros2 launch mower_bringup mower.launch.py use_sim_time:=false
```

任務節點：

```bash
ros2 run mower_mission path_record_node
ros2 run mower_mission map_manage_node
ros2 run mower_mission coverage_node
```

核心 service/action 存在：

```bash
ros2 service list | grep record_zone
ros2 service list | grep create_free_space
ros2 service list | grep zone_exec_path
ros2 action list | grep nav_action
```

## 12. 不在第一階段做的事

為了降低風險，第一階段不建議同時做：

- topic/service/action runtime 名稱大改
- `Chennal` 全面改名 `Channel`
- `cencel_nav2` 直接刪除
- coverage/map/path_record 行為重構
- Nav2 controller/planner 參數調整
- GPS/RTK datum 策略調整

這些可以放第二階段，等 package build graph 穩定後再整理。

## 13. 待確認問題

1. `mower_mission` 是否使用正確拼字？還是要照你原先寫的 `mower_misson`？
2. interface package 要叫 `mower_interface` 還是 `mower_interfaces`？
3. `nav_robot` 是否保持獨立 package，只更新 action import？還是也併入 `mower_mission`？
4. 模擬資源 `models/worlds/urdf` 是否留在 `mower_bringup`，或未來拆成 `mower_sim`？
5. 是否要在第一階段保留舊 executable alias，例如 `ros2 run path_record path_record` 這類命令？
