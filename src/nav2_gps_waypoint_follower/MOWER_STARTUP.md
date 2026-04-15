# Mower Bringup Startup Guide

這份文件說明如何使用 `mower.launch.py` 啟動實機底盤、`dual_ekf` 與 Nav2，並在定位穩定後手動啟動導航。

## 啟動前提

- 已完成 workspace build，並 `source install/setup.bash`
- IMU container / node 已可發布 `imu/data`
- GPS container 已可發布 `/gps/fix`
- 如果要規劃路徑，地圖來源需要能發布 `/map_grid`

建議先確認這些 topic 有資料：

```bash
ros2 topic list | grep -E "/gps/fix|/imu/data|/odom|/map_grid|/odometry/global"
```

## 啟動主系統

啟動割草機主 bringup：

```bash
ros2 launch nav2_gps_waypoint_follower mower.launch.py
```

目前 `mower.launch.py` 的預設行為：

- `use_sim_time:=false`
- `enable_localization:=true`
- `enable_navigation:=true`
- `nav_autostart:=false`

這代表：

- 底盤控制、`robot_state_publisher`、`twist_mux`、joystick 會啟動
- `dual_ekf + navsat_transform` 會啟動
- Nav2 節點會啟動
- 但 Nav2 不會自動進入 `active`

## 常用啟動參數

關閉 Nav2：

```bash
ros2 launch nav2_gps_waypoint_follower mower.launch.py enable_navigation:=false
```

關閉 localization：

```bash
ros2 launch nav2_gps_waypoint_follower mower.launch.py enable_localization:=false
```

讓 Nav2 開機後直接自動 activate：

```bash
ros2 launch nav2_gps_waypoint_follower mower.launch.py nav_autostart:=true
```

指定 Nav2 參數檔：

```bash
ros2 launch nav2_gps_waypoint_follower mower.launch.py \
  nav2_params_file:=/absolute/path/to/nav2_params.yaml
```

## 手動啟動 Nav2

當 `/gps/fix`、`/imu/data`、`/odometry/global`、`/map_grid` 都穩定後，再手動 activate Nav2：

```bash
ros2 service call /lifecycle_manager_navigation/manage_nodes \
  nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
```

`command: 0` 代表 `STARTUP`，會依序將 Nav2 lifecycle nodes 做 `configure + activate`。

## 檢查 Nav2 是否已啟動

查看 lifecycle state：

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
```

正常情況應該會看到 `active`。

也可以確認速度輸出鏈：

```bash
ros2 topic echo /nav_cmd_vel
```

Nav2 的速度會先輸出到 `/nav_cmd_vel`，再由 `twist_mux` 仲裁後送到底盤。

## 建議檢查順序

1. 啟動 `mower.launch.py`
2. 確認 `/gps/fix` 有資料
3. 確認 `/imu/data` 有資料
4. 確認 `/odometry/global` 有資料
5. 確認 TF 正常
6. 確認 `/map_grid` 已發布
7. 手動 activate Nav2
8. 再啟動任務節點或送 navigation goal

## TF 檢查

建議至少確認這幾條 TF 存在：

- `map -> odom`
- `odom -> base_footprint`
- `base_footprint -> base_link`

可以用：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

## 常見問題

### 1. Nav2 節點有起來，但不能導航

最常見原因是 Nav2 還沒 activate。請先執行：

```bash
ros2 service call /lifecycle_manager_navigation/manage_nodes \
  nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
```

### 2. EKF 沒有全域定位

請檢查：

- `/gps/fix` 是否存在
- `/imu/data` 是否存在
- `/odom` 是否存在
- `/odometry/global` 是否開始發布

### 3. Planner 或 costmap 一直報錯

請檢查 `/map_grid` 是否已經由地圖模組發布。

### 4. Nav2 有出速度但車不動

請依序檢查：

- `/nav_cmd_vel`
- `/cmd_vel_out`
- `/diff_controller/cmd_vel`

如果 joystick 正在持續發布 `/joy_cmd`，因為 `twist_mux` 優先權較高，Nav2 速度可能會被覆蓋。

## 建議操作模式

實機上建議使用這個流程：

1. 先啟動 `mower.launch.py`
2. 等 GPS、IMU、EKF、地圖穩定
3. 手動 activate Nav2
4. 最後再啟動 coverage / waypoint / mission 節點

這樣比開機直接 `nav_autostart:=true` 更安全，也比較好排查問題。

## 樹莓派開機自動啟動

repo 內已提供這些檔案：

- [`utiles/start-raspi-mower-stack.sh`](/home/mower/Desktop/mower_path_planning/utiles/start-raspi-mower-stack.sh)
- [`utiles/stop-raspi-mower-stack.sh`](/home/mower/Desktop/mower_path_planning/utiles/stop-raspi-mower-stack.sh)
- [`utiles/raspi-mower-stack.service`](/home/mower/Desktop/mower_path_planning/utiles/raspi-mower-stack.service)

這套做法會在開機後：

1. 啟動 `.devcontainer/docker-compose.raspi.dev.yaml`
2. 啟動 `.devcontainer/docker-compose.raspi.zenoh.yaml`
3. 啟動 `mower_bringup` compose service
4. 由 `mower_bringup` 容器直接執行 `ros2 launch nav2_gps_waypoint_follower mower.launch.py`

### 安裝步驟

先確定 image 已經手動 build 過至少一次：

```bash
docker compose \
  -f .devcontainer/docker-compose.raspi.dev.yaml \
  -f .devcontainer/docker-compose.raspi.zenoh.yaml \
  build
```

安裝 service：

```bash
sudo cp utiles/raspi-mower-stack.service /etc/systemd/system/
sudo chmod +x utiles/start-raspi-mower-stack.sh utiles/stop-raspi-mower-stack.sh
sudo systemctl daemon-reload
sudo systemctl enable raspi-mower-stack.service
sudo systemctl start raspi-mower-stack.service
```

### 查看狀態

```bash
sudo systemctl status raspi-mower-stack.service
journalctl -u raspi-mower-stack.service -f
```

查看 `mower.launch.py` log：

```bash
docker compose \
  -f .devcontainer/docker-compose.raspi.dev.yaml \
  -f .devcontainer/docker-compose.raspi.zenoh.yaml \
  logs -f mower_bringup
```

### 停止自動啟動服務

```bash
sudo systemctl stop raspi-mower-stack.service
sudo systemctl disable raspi-mower-stack.service
```

### 注意

- 目前 service 內的 `User=mower` 與 repo 路徑 `/home/mower/Desktop/mower_path_planning` 是依照現在這台機器寫的
- 如果你的樹莓派使用者名稱或 repo 路徑不同，請先修改 [`utiles/raspi-mower-stack.service`](/home/mower/Desktop/mower_path_planning/utiles/raspi-mower-stack.service)
- `mower.launch.py` 現在是 `mower_bringup` service 的主程序，所以可直接用 `docker compose logs -f mower_bringup` 查看
