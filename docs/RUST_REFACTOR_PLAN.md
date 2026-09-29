# CPU 效率分析與 Rust 重構計劃

日期：2026-09-19　量測對象：LubanCat-2（RK3568，4×A55）、映像 `e1cfabc1`（commit ebb386e）、待機狀態（nav2 已啟動、無任務）、2 個 rosbridge 客戶端（Mac 模擬器 + Studio）。

## 1. 量測方法

| 項目 | 方法 |
|---|---|
| 每個程式的 CPU | `/proc/<pid>/stat` utime+stime 差分，30 s 視窗（sum = 388% of one core = 97% of 4 cores；load 8.9、1.4 GHz、83 °C） |
| Python 節點熱點 | py-spy 0.4.2，50 Hz × 15 s，9 個最耗 CPU 的 rclpy 節點 |
| Topic 訊息率 | 149 個 topic 全部 raw 訂閱、每 20 ms 直接 `take_message()` 排空（不經 rclpy executor，否則探針自己就被截到 10 Hz）；全圖共 659 msg/s |
| 每節點入站訊息率 | `get_subscriber_names_and_types_by_node` × 上面的 topic 率 |

## 2. 結果：每個程式的效率

### 2.1 Python（rclpy）節點：合計 298% of one core（佔整機 77%）

| 節點 | CPU % | 事件/s（入站訊息 + timer） | ms CPU / 事件 | 熱點（py-spy） |
|---|---|---|---|---|
| flutter_adapter | 59.1 | /tf 76.6 + timers ≈ 90 | 6.5 | 97% 在 executor；TransformListener 吃整條 /tf；使用者程式碼 ≈ 0 |
| path_record_node | 50.9 | /tf 76.6 | 6.6 | 同上 + MultiThreadedExecutor（58% 在 thread handoff）+ 26 個 service 撐大 wait set |
| rosbridge_websocket | 46.3 | 37 入站 × 2 客戶端 ≈ 74 出站 + service call | ≈ 6.3 / 出站訊息 | 26% tornado write、10% 每次 call_service 都 create_client + 查 graph + destroy |
| nav_action_server | 37.5 | /imu 10 + 20 Hz timer（發 2 topic）≈ 30 | 12.5 | MultiThreadedExecutor（68% 在 worker）+ BasicNavigator 同 process（29 個 sub） |
| telemetry_node | 25.8 | 34.6 入站 + 10 Hz tick ≈ 45 | 5.8 | `_tick` JSON 編碼 22%，其餘 executor |
| velocity_command_guard | 16.7 | /cmd_vel_guard_input 19.7 | 8.5 | 使用者程式碼 8%，其餘 executor |
| manual_velocity_guard | 12.6 | 20 Hz timer，入站 0 | 6.3 | 一個 20 Hz 時鐘就吃 12.6% |
| pid_autotune | 11.4 | /mower_base/telemetry 16.7 | 6.8 | 每筆 json.loads 1.2 KB；沒調參時也一直訂閱 |
| battery_state | 8.4 | 16.7 | 5.0 | 6% 使用者程式碼 |
| wit_ros2_imu | 6.4 | 10 發布 + serial thread | 6.4 | executor + pyserial 輪詢 |
| rosbridge_auth_proxy | 5.6 | 轉發約 80 KB/s | – | 純 byte 轉發用 Python |
| robot_info | 4.2 | 4.8 + 1 Hz | 7.2 | executor |
| heartbeat | 3.8 | 4.8 + 2 Hz | 5.6 | executor |
| coverage_node | 3.6 | 閒置 | – | 保留 |
| map_manage | 2.9 | 閒置 | – | 保留 |
| rosapi | 2.5 | 閒置 | – | Phase 3 併入 |
| mower_agent | 0.2 | – | – | 保留 |

### 2.2 C++（rclcpp）節點：合計約 82%

| 節點 | CPU % | 事件/s | ms / 事件 | 備註 |
|---|---|---|---|---|
| ros2_control_node | 24.4 | 50 Hz 控制迴圈 + 6 個 50 Hz topic | – | 4 個 statistics/introspection topic 50 Hz、323 KB/s，沒有任何訂閱者 |
| ekf_filter_node_map / odom | 10.2 / 9.9 | 60 入 + 30 出 | 1.1 | 含真正的濾波運算 |
| nav2（controller、bt、behavior、planner、smoother、其他） | ≈ 23 | 5 個 tf listener 各吃 /tf 76.6 Hz | – | |
| navsat_transform | 5.2 | 40 | 1.3 | |
| robot_state_publisher | 3.8 | 50 | 0.8 | |
| odom_throttle（topic_tools） | 3.5 | 50 | 0.7 | |
| twist_mux | 2.6 | 39 入 + 20 出 | 0.4 | |

其他：gst-launch（相機）2.5%、mediamtx 2.0%、kernel/tailscale ≈ 2.5%。

### 2.3 結論

- **rclpy 每個事件（訊息或 timer）固定成本 5–12 ms CPU；rclcpp 做真正運算也只要 0.4–1.3 ms。** 差距約 8 倍，跟節點做什麼無關：py-spy 顯示 80–97% 的時間在 `_wait_for_ready_callbacks`（每次 spin 用 Python 重建 wait set，Jazzy 還把每個 pub/sub 的 QoS event handler 也加進去）、`_take_subscription`、Task/coroutine 包裝。
- **MultiThreadedExecutor 讓情況更糟**（path_record、nav_action_server）：每個 handler 都丟給 thread pool，再加上 GIL 交接。
- **一半以上的 Python 負載是「沒必要的工作」**：兩個 tf listener 各吃 77 Hz、pid_autotune 常駐、controller_manager 對空氣廣播 323 KB/s。這些不用 Rust 就能修。
- **剩下的是「必要但用 Python 做太貴」的資料搬運**：guards、telemetry、battery、heartbeat、adapter、rosbridge、auth proxy、IMU driver。這些是 Rust 的目標。
- Rust 透過 rcl FFI 的成本與 rclcpp 同級（估 ≤ 0.3 ms/事件），Rust 本身不會比 C++ 快，好處在於記憶體安全和 tokio 生態（WebSocket、serial、JSON）。

## 3. 目標

| 階段 | 整機 CPU（sum of one-core %） | 附帶效果 |
|---|---|---|
| 現在 | 388%（97%） | 1.1–1.4 GHz 熱降頻、84 °C、load 9–20 |
| Phase 0（純 Python 修正） | ≈ 240%（60%）→ **實測 285%** | 脫離熱降頻 |
| Phase 2（Rust 搬運節點） | ≈ 165%（41%）→ **實測 259%（guards 未切）** | |
| Phase 3（Rust bridge） | ≈ 120%（30%）→ **實測 212%（guards 未切；nav_action_server 45% 未移植）** | 第一筆 heartbeat 425 ms、service call 1–2 ms（bridge 本身）+ 服務端處理 |

驗收指標（同一組腳本再量一次）：sum-of-process CPU、load avg < 3、溫度 < 70 °C 且時脈不降、rosbridge 連線後第一筆 `/robot/online` < 0.5 s、service call p95 < 100 ms、`/adapter/robot_pose` 5 Hz jitter < 50 ms。

## 4. Phase 0：不用 Rust 的立即修正（2–3 天）

| # | 修正 | 預期 | 檔案 |
|---|---|---|---|
| 0.1 | flutter_adapter 拿掉 TransformListener，改訂閱 `/odometry/global_slow`（topic_tools throttle 10 Hz；EKF map 的輸出就是 map→base_link） | 59 → ≈ 6% | `adapters/flutter_adapter_node.py:117`、`mission.launch.py` 加 throttle |
| 0.2 | path_record_node 同樣改法（`lookup_transform` 只在 `:277` 一處），並改回 SingleThreadedExecutor | 51 → ≈ 4% | `path_record_node.py:200,277` |
| 0.3 | nav_action_server：IMU 健康檢查改吃 2 Hz throttle 的 `/imu/data_slow`；20 Hz safety_stop/coordinator_lock 降到 twist_mux timeout 允許的最低頻；評估 SingleThreadedExecutor + callback group | 37 → ≈ 10–15% | `navigation/nav_action_server.py` |
| 0.4 | pid_autotune 只在 `/pid_autotune` 服務開啟 session 時才訂閱 `/mower_base/telemetry` | 11 → ≈ 1% | `pid_autotune_node.py` |
| 0.5 | 關掉 controller_manager 的 introspection/statistics 發布（Jazzy pal_statistics；實作時用 `ros2 param list /controller_manager` 確認參數名） | 24 → ≈ 15% | `mower_hardware/config/mower_controllers.yaml` |
| 0.6 | 確認 `/tf` 的 4 個發布者是否重複（diff_controller 與 ekf_odom 都發 odom→base_link 的話關 `enable_odom_tf`），可再減 nav2 5 個 tf listener 的負載 | 視情況 | `mower_controllers.yaml` |

Phase 0 做完先量一次，作為 Rust 階段的比較基準。

### 4.1 Phase 0 執行紀錄（2026-09-19）

| # | 狀態 | 實際做法 |
|---|---|---|
| 0.1 | 完成 | `flutter_adapter` 改訂閱 `robot_pose_source_topic`（預設 `/odometry/global`，mission.launch.py 指到新的 5 Hz throttle `/odometry/global_slow`），驗證邏輯不變（frame、child_frame、有限值、四元數、時戳年齡），1:1 轉發成 `/adapter/robot_pose`（仍是 5 Hz）。EKF map 的 `base_link_frame` 就是 `base_footprint`，所以 `/odometry/global` 與原本的 `map→base_footprint` tf 查詢相同。 |
| 0.2 | 完成 | `path_record_node` 同上，`get_robot_pos()` 回傳快取的最新位置（`robot_pose_max_age_s` 1.0 s 內），過期回 None。**MultiThreadedExecutor 必須保留**：service callback 會阻塞等 `/mission_operation_lock` 回應（`navigation_guard._wait_for_lock_response`），需要另一條 executor thread。launch test 改餵 Odometry。 |
| 0.3 | 部分 | 兩個 0.05 s timer（health monitor、safety-stop heartbeat，同一 MutuallyExclusive group）合併成一個 `_health_tick`，順序與 20 Hz 節奏不變。**IMU 不能降頻**：`navigation_health_timeout_s` 是 0.30 s 的不可變安全參數，2 Hz throttle 會讓 IMU 永遠被判定 stale。/rosout 訂閱閒置時 0 msg/s，不用動。Executor 結構不動。 |
| 0.4 | 完成 | `pid_autotune` 的 telemetry 訂閱與 5 Hz 進度 timer 只在 session 期間存在（`start` 建立、worker 結束後由 `_tick` 拆掉），`_telemetry()` 對第一筆訊息等最多一個 timeout。7 個節點測試通過。 |
| 0.5 | 不可行 | controller_manager 4.48（Jazzy）沒有任何參數能關掉 pal_statistics introspection（`ros2 param list` 無相關項；上游 `controller_manager.cpp` 無條件 `START_PUBLISH_THREAD`）。要關必須改上游或自建。保留。 |
| 0.6 | 無需修改 | 真機量測 `/tf` = ekf_odom `odom→base_footprint` 30 Hz + ekf_map `map→odom` 30 Hz + robot_state_publisher 兩輪 16.7 Hz；`diff_controller.enable_odom_tf` 執行期已是 false，沒有重複發布。 |

量測（映像 `ba2669e2`，commit 9827538，重啟 7 分鐘後，30 s，仍是 2 個 rosbridge 客戶端）：整機 **388% → 285%**（Python 298 → 197）。flutter_adapter 59.1 → 12.7、path_record 50.9 → 15.7、pid_autotune 11.4 → <0.7、nav_action_server 37.5 → 31.7；其餘不變（rosbridge 51、telemetry 22、guards 17.7+12.6、battery 9.0、ros2_control 23.7）。`/tf` 訂閱者從 7 個降到 5 個（都是 nav2 C++）。

驗證：ROS-free 測試 75 passed（`test_qos_configuration.py` 改為檢查新的 frame gate 並禁止 `from tf2_ros import`）；本機 `mower-jazzy-test` 容器（加裝 nav2_simple_commander、robot_localization、topic_tools）跑 `test_pid_autotune_node.py` 7 passed、`test_nav_action_server_services.py` 33 passed / 6 failed（HEAD 上同樣 6 個失敗，與本次無關）、path_record 錄製情境 2/2 通過（`test_path_record_integration_launch.py` 的類別不是 `unittest.TestCase`，launch_testing 實際跑 0 個測試，已知問題）、flutter_adapter 15/15 轉發、5 種無效樣本全部拒絕。

## 5. Rust 技術選型

| | r2r 0.9.7（2026-09-13） | rclrs 0.7（ros2_rust） |
|---|---|---|
| Jazzy | 支援（0.9.1 起） | 支援，但 2026-01 issue #588 顯示 Jazzy 安裝仍有摩擦 |
| 建置 | 只要 `cargo build` + 已 source 的 ROS 環境；build.rs 從 AMENT_PREFIX_PATH 讀 typesupport 產生綁定，`IDL_PACKAGE_FILTER` 限縮範圍 | 需要 colcon-cargo、colcon-ros-cargo、cargo-ament-build，`vcs import` ros2_rust 進 src，自訂訊息走 rosidl_generator_rs |
| 自訂 `mower_interface` | 自動（package.xml 宣告依賴讓 colcon 先建 mower_interface 即可） | 需 rosidl_generator_rs 重產 |
| Untyped pub/sub/client（serde_json::Value） | 有（bridge 需要） | 無 |
| tf2 | 無（本計劃已把 tf 依賴拿掉） | tf2_rs（cxx 包 tf2::BufferCore，0 star 個人專案） |
| Executor | async/futures，不綁 runtime（用 tokio） | 自家 executor |
| 風險 | API 會變、文件少 | 同樣未到 1.0 |

**決定：r2r**。理由：專案的 builder stage 已經裝了 rustup/cargo（`mower_coverage_core` PyO3）、CI 已快取 cargo registry 與 `CARGO_TARGET_DIR`，r2r 不需要任何 colcon 外掛；bridge 需要 untyped API。rclrs 等 1.0 再評估。

## 6. 建置整合

1. 新增 ament_cmake 套件 `src/mower_rs/`：一個 Cargo workspace，多個 bin（`mower_telemetry`、`velocity_command_guard`、`mower_adapter`、`mower_imu`、`mower_ws_bridge`）。`CMakeLists.txt` 比照 `mower_coverage_core` 在建置時呼叫 `cargo build --release`，把 bin 安裝到 `lib/mower_rs/`，所以 `ros2 run mower_rs <bin>` 和 launch 的 `Node(package='mower_rs', executable=...)` 都照舊。`package.xml` `<depend>mower_interface</depend>` 讓 colcon 先建訊息。
2. Dockerfile builder stage：apt 加 `libclang-dev`（bindgen）；`.cargo/config.toml` 設 `IDL_PACKAGE_FILTER`（std_msgs、geometry_msgs、nav_msgs、sensor_msgs、tf2_msgs、std_srvs、rcl_interfaces、visualization_msgs、builtin_interfaces、action_msgs、unique_identifier_msgs、mower_interface、robot_localization）。首次 arm64 冷建約 5–8 分鐘，之後由 buildkit-cache-dance 快取。
3. Runtime stage 不變：bin 動態連結 /opt/ros/jazzy 的 librcl/rmw/typesupport，都已在映像裡。
4. Launch：每個 process 一個 launch 參數（`rust_status`、`rust_guards`，預設 false，逐一切換），同一映像同時帶 Python 與 Rust 版，回滾只改參數。
5. 開發環境：Mac 上 `cargo test` 跑純邏輯單元測試（不需 ROS）；需要 ROS 的整合測試在 devcontainer / CI arm64 runner。

### 6.1 Phase 1 執行紀錄（2026-09-19）

- `src/mower_rs/`：ament_cmake 套件（`project(mower_rs NONE)`），`CMakeLists.txt` 在建置時跑 `cargo build --release --locked`（target dir 跟著 `CARGO_TARGET_DIR` 進 colcon build tree / CI cache），bin 裝到 `lib/mower_rs/`；`ros2 run mower_rs robot_status`、`ros2 pkg executables mower_rs` 都正常。Cargo workspace：`crates/mower_rs_common`（參數、JSON 四捨五入、state dir 檔案、host.request）+ `crates/robot_status`。
- r2r 0.9.7 在 Jazzy arm64 驗證通過：`IDL_PACKAGE_FILTER` 必須列出 `mower_interface` 的遞移相依（visualization_msgs 等，r2r 不自動解析）；冷建 41 s、暖建 16 s；bin 1.4 MB，只動態連結 /opt/ros/jazzy 與 `mower_interface` 的 typesupport。
- Dockerfile builder 加 `libclang-dev`（bindgen）。
- 切換：每個 process 一個開關。`mission.launch.py` `rust_status`、`twist_mux.launch.py` `rust_guards`，`robot.launch.py` 兩個都收；compose 讀 `.env` 的 `RUST_STATUS` / `RUST_GUARDS`（預設 false）。`rust_status:=true` 時只有 `/robot_status` 起來，false 時是原本三個 Python 節點（本機容器實測）。

### 7.1 `robot_status`（pilot）執行紀錄

實際做法與計劃的差異：一個 process、**一個** node `robot_status`（不是三個保留原名的 node）：launch 的 params 檔以 node 名稱分組，三個節點的 `publish_rate_hz` 會互相打架，所以合成一個 node，會撞名的參數加前綴（`heartbeat_*`、`info_publish_rate_hz`、`telemetry_publish_rate_hz`）。battery_state_node 暫不納入（另一個 session 正在改 battery_estimator / battery_state_node）。

影子比對（本機容器，合成輸入 5 Hz odom / imu / fix / battery / base telemetry，state dir 放 image.json、update_status.json、firmware_sync.json、link_status.json、identity.json）：`/robot/info` 0 個欄位差異；`/robot/telemetry` 70 vs 70 筆、唯一差異是 `robot_id`（Python 版用 hostname，Rust 版用 identity.json 的配對 ID，刻意）；`/robot/online` 相同；`/system/update` 寫出 host.request、`robot_status.json` 內容一致；SIGTERM 正常關閉。單元測試 10 個（`cargo test`）。

真機影子比對（映像 5be6ee5，300 s，remap 到 `/shadow/...`，與正在跑的 Python 節點並行）：`/robot/info` 301 vs 296 筆、0 個欄位差異；`/robot/telemetry` 2997 vs 2953 筆，10 次快照只差 `robot_id`（刻意）與 IMU 加速度的取樣時刻抖動（兩邊各自取到不同筆 IMU）；`/robot/online` 601 vs 591 筆全部 true；Rust 版各區塊的 `age_s` 更小（處理更快）。待辦：`.env` 設 `RUST_STATUS=true` → 24 h 觀察 → 拿掉 Python 版。

### 7.2 `velocity_command_guard` 執行紀錄

`crates/velocity_command_guard`：`core.rs` 是純邏輯（Limits 夾值與有限性檢查、時戳屏障、payload 規則、重播判定、steady-clock watchdog），每條規則一個測試向量，共 7 個測試；`main.rs` 是 r2r 外殼（相對 topic 名 `cmd_vel_in`/`cmd_vel_out`/`command_clock`，啟動與 SIGINT/SIGTERM 時發零，出站時戳用機器人 wall clock，`/dev/urandom` 產生 session id，無 parameter service 所以限制不可在執行期更改）。與 Python 版的差異：不支援 `use_sim_time`（只有 production 用；`robot.launch.py` 本來就強制 `use_sim_time:=false`）；Python 版在 SIGINT 時的最後一筆零其實發不出去（context 已關閉，會丟 `RCLError`），Rust 版在關閉前先發。

差分測試（本機容器，`guard_compare.py`）：兩個實作吃同一串命令（零時戳 ×3、watchdog、帶時戳 ×3、過期、超速、側向、NaN、重播兩次、逾時重播、明確零；session 模式：零時戳、錯誤 id、正確 id ×3），輸出序列與 reject/timeout log **完全一致**。

切換：`twist_mux.launch.py` 新增 `rust_guards`，兩個 guard 各有 Python / Rust 版本互斥；`mower.launch.py`、`robot.launch.py` 轉發（compose `RUST_GUARDS`）。本機容器 `rust_guards:=true` → 2 個 Rust guard、0 個 Python；`false` 反之。**真機切換前必須有人監督**：app 搖桿手動駕駛（含放開搖桿要在 0.2 s 內停）、一次導航任務、`/navigation_safety_stop` 介入；這一步留給現場。`test_controller_configuration.py` 新增 `test_rust_velocity_guard_keeps_the_same_rules_and_wiring`。

### 7.3 `mower_adapter` 執行紀錄

`crates/mower_adapter`：`dto.rs` 是純轉換（OccupancyGrid → base64 map layer、MarkerArray → marker layer（顏色用 round-half-to-even 對齊 Python `round`）、ZoneMap[] → summaries、ParameterValue → JSON、datum bearing），5 個單元測試比對 Python DTO 版面；`main.rs` 保留節點名 `flutter_adapter`（launch 參數不變）、latched relays、pose gate（與 Phase 0 的 Python 版同一套檢查）、三種 service client（get_parameters ×2 每秒、zone map list 2 Hz、toLL 直到鎖定，各 3 s timeout；先等服務出現再開始輪詢）。需要 `geographic_msgs`、`robot_localization` 進 `IDL_PACKAGE_FILTER` 與 package.xml。

影子比對（本機容器，300×200 grid、兩個 marker、fake `/boustrophedon_coverage` / `/map_manage` 參數節點、fake ZoneMapList 與 toLL 服務、10 筆 odometry）：13 個 `/adapter/*` topic 與 `robot_pose` 全部 **IDENTICAL**（map layer JSON 逐位元組相同）。開關 `rust_adapter`（compose `RUST_ADAPTER`）。

備註：ZoneMapList 每 0.5 s 回傳所有 zone 的完整 OccupancyGrid 只為了數 pose、看 data 是否為空，map_manage（Python）那一側每次都要序列化——這是 map_manage 閒置 3% 的來源之一，之後可改成 map_manage 發一個小的 summary topic。

### 7.4 `mower_imu` 執行紀錄

`crates/mower_imu`：`wit.rs` 是 WIT 11-byte frame parser（0x51/0x52/0x53/0x54、checksum、與上游相同的比例係數、Python 版的四元數公式），3 個單元測試；`main.rs` 用 `serialport` crate（不帶 libudev）以 9600 8N1 讀 `/dev/imu_usb`（新增 `port` 參數），保留 fail-closed 規則（200 ms host gap 或 >88 bytes backlog 就清空、姿態 frame 只在 accel/gyro 都新鮮時發、serial 錯誤讓整個 process 以 exit 1 結束）。差分測試（socat pty 兩對、相同 frame 串流 30 週期 + 過期姿態 + 壞 checksum）：`/imu/data` 序列 31 vs 31 筆 **完全相同**，drop 與 checksum 處理一致，covariance 相同。開關 `rust_imu`（compose `RUST_IMU`）。

### 7.5 真機切換紀錄與一個教訓

- 2026-09-19：`RUST_STATUS=true`（映像 197bf89）→ `robot_status` 4.3–4.8%（取代 Python 三節點約 30%）。`RUST_ADAPTER=true`、`RUST_IMU=true`（映像 97d0256）：adapter 正常（robot_pose 4.6 Hz、zone_summaries 1.9 Hz、coverage_settings 1 Hz）、IMU 10.1 Hz、stamp 間隔 100 ± 7 ms。
- **教訓：r2r 的 `spin_once` 在 wait set 為空時立刻返回。** `mower_imu` 沒有訂閱 / timer / service，spin 迴圈變成 busy loop，真機量到 84% CPU；`mower_ws_bridge` 在還沒有客戶端訂閱時也會如此。修法：IMU 不 spin（發布不需要 spin，主執行緒只等訊號）；bridge 用一個私有 `~/wake` topic 讓 wait set 永不為空、命令送達時發一筆喚醒 spin（idle 1.38% → 0.25%），並在關閉時先停掉 node 執行緒再離開（否則 rmw 的解構會跟還在 `rcl_wait` 的執行緒撞在一起，glibc mutex assertion abort）。已修（IMU 在容器裡 10 Hz 輸入下 0.9%），真機的 `RUST_IMU` 先關回 false，等修好的映像上線再開。
- 每個 process 的 CPU 仍以真機 30 s 取樣為準（`/tmp/cpu_sample.sh`）。

### 7.6 Python 端再兩刀（2026-09-19）

- `path_record_node`：10 Hz `publish_path_timer` 改成只在錄製期間存在（三個 start service 重新 arm，callback 發現沒有任何錄製就 cancel）。閒置時每秒省 10 次 rclpy timer 喚醒（MTE 下每次約 8–12 ms）。錄製情境測試 2/2 通過。
- `nav_action_server`：py-spy 顯示 44% 裡 65% 是 `_wait_for_ready_callbacks`（MTE 每個事件都用 Python 重建 wait set），callback 本身 4%。rclpy 7.1.12 有 `rclpy.experimental.EventsExecutor`（C++ 事件迴圈、便宜很多），但它是單執行緒、不理 callback group，而 action execute callback 會阻塞等 nav2，所以不能換。改法：20 Hz health tick 從 executor timer 改成自己的執行緒（決策與發布仍在 `_state_lock` 下、節奏不變、掉拍時重新對齊而不補發），executor 的事件數從 ~35/s 降到 ~15/s。測試集合與 HEAD 相同（33 passed / 同樣的 6 個既有失敗），容器實測 heartbeat 20 Hz、p99 55 ms、鎖定時輸出零速度。

### 7.7 `mower_record` 執行紀錄（path_record_node 移植，使用者要求）

`crates/mower_record`：`geometry.rs`（DP 簡化、點在多邊形、邊距離、shoelace 面積）、`site_store.rs`（場地檔與 `.active_site` manifest，同樣的 tmp+rename+fsync 寫法、同樣的 JSON 版面、同樣的等距投影；`round_ties_even` 對齊 Python `round`）、`guard.rs`（mission mutation lease：unknown 視為 active、本地鎖、1 s 服務等待、3 s 回應等待、逾時 fail-closed 鎖死）、`recorder.rs`（所有 service 本體，ROS-free，透過 `Outputs` trait 發布；app 看得到的中文訊息逐字保留）、`main.rs`（node 名 `path_recorder`、19 個 service、latched 清單、10 Hz 取樣只在錄製時、所有 handler 共用一把 async mutex 等同 rclpy 的 callback group 序列化；Python 版需要 MTE 的原因（service 裡阻塞等 lock）在 async 下自然消失）。

驗證：10 個單元測試；與 Python 節點在同一張圖上跑同一個 32 步情境（錄製 zone/risk/channel、取消、edit_zone 新增/更新/刪除、通道路由正反向、場地 save（無 datum / fallback / navsat）、編輯自動同步場地檔、旋轉 datum 下 load 重投影、rename、delete、load_zone_list、save_zone_list、導航中拒絕錄製）：**所有回覆（含訊息與 id）、三份工作檔、五個發布清單、34 次 lock 呼叫順序完全相同**。開關 `rust_record`（compose `RUST_RECORD`）。

### 7.8 `mower_nav` 執行紀錄（nav_action_server 移植，使用者要求）

`crates/mower_nav`：`geometry.rs`（路徑准入規則、dispatch_id 正規化、依覆蓋分割點 / 轉角 / 最大距離切段）、`state.rs`（整個協調器狀態放在一把 mutex 後：准入順序、感測器健康判定（同樣的協方差特徵值門檻）、手動 hold、mutation lock、`/check_nav_status` JSON、`/rosout` 的 Nav2 log ring）、`nav2.rs`（generation 相關聯的 Nav2 終態證據）、`main.rs`（node 名 `nav_action_server`、兩個 action 名、六個 service、健康與手動訂閱、20 Hz heartbeat task（掉拍時跳過不補發，等同 Python 執行緒的重新對齊）、2 Hz uncertain monitor、執行流程：dispatch 確認 → 有界等 bt_navigator active → NavigateToPose 到覆蓋起點 → 逐段 FollowPath → 取消確認逾時就鎖 `uncertain`）。安全參數只在啟動時讀、沒有 parameter service，執行期無法被削弱。

r2r 的兩個細節：(1) `spin` 與 action server 共用 node 執行緒，goal 的執行在 tokio 上；(2) 接受 action 層取消後 r2r 只剩 `cancel()` 能送出結果、而 rcl 只允許從 CANCELING 進 CANCELED，所以 `Execution::terminate` 依情況選轉移。這同時揭露 Python 版的一個 bug：透過 `/cancel_nav2` 或手動指令取消時 `goal_handle.canceled()` 會丟 rcl 例外（goal 不在 CANCELING），被外層接住變成 `failed` + 一串 rcl 錯誤字串，app 上看到「導航失敗」。已修（`_finish_goal_canceled`：轉移失敗就 abort，狀態仍是 `canceled`），Rust 版同樣行為。

驗證：7 個單元測試；與 Python 節點在同一張圖上對 mock Nav2（假的 `bt_navigator/get_state`、`navigate_to_pose`、`follow_path`：成功 / abort / 聽取消 / 不理取消）跑同一個情境：35 步（壞 dispatch_id、短路徑、未確認逾時、cancel_navigation_dispatch token、確認後完整跑完含切段、`/cancel_nav2` 中途取消、action 層取消、Nav2 失敗、mutation lock 四種情況、手動指令取消、取消逾時鎖 `uncertain` 與相關聯的自動恢復、`/nav_operation_active` 序列）+ 13 步健康閘（四個來源逐一補齊、執行中 GPS 過期取消、待確認時 IMU 過期取消、pose 消失的時序）**全部相同**；唯一保留的差異是 Nav2 錯誤字串：這版 BasicNavigator 沒有 `getTaskError()`，Rust 版回真正的 error code。Heartbeat 容器實測 20 Hz、中位 49.8 ms、p99 55 ms（Python 49.9 / 59）。Python 節點在 0.30 s 的健康期限下在本機容器裡跟不上（rclpy 每個 callback 5–12 ms，40 msg/s），健康情境要放寬到 1.0 s 才能比對；Rust 版 0.30 s 下正常（最後一筆後 0.21 s 判定過期）。開關 `rust_nav`（compose `RUST_NAV`），**真機尚未切換：安全關鍵，先做一次監督下的導航**。

### 7.9 `mower_battery` 執行紀錄（battery_state_node 移植，使用者要求）

`crates/mower_battery`：`estimator.rs` 逐行移植 `battery_estimator.py`（OCV 表、sag 低通、限速回升、只有電壓時的充電斜坡、拔充電器後的重新錨定、庫侖計數與靜置重錨、滿電尾電流判定），14 個 pytest 向量原樣搬成 `cargo test`；`main.rs`（node 名 `battery_state`，同樣參數，16.7 Hz telemetry JSON 以 Python truthiness 判 `valid`/`online`、電表優先 / ADC 備援、充電器在場門檻、有號電流映射、低電量一次性警告、1 Hz `BatteryState` 版面：未量測欄位 NaN、`percentage` 0..1、health 由 cell 電壓決定）。差分測試發現 Python 版遇到 `charger`/`analog` 不是物件的 frame 會 AttributeError 直接死掉，已修（isinstance 檢查），Rust 版忽略該 frame。

驗證：與 Python 節點吃同一條合成 telemetry 串流（無資料、電表 23.89 V、2 s sag、充電器 25.6 V、充電器無電流、拔除、ADC 備援 + AON 3.78 V、垃圾 frame、逾時、低電壓），只有電壓與庫侖計數兩種參數各跑一次：**每一筆 1 Hz 樣本在兩個 timer 相位差內相同**（present / status / health 轉換序列完全一致）。開關 `rust_battery`（compose `RUST_BATTERY`）。

### 7.10 `mower_pid_autotune` 執行紀錄（pid_autotune_node 移植，使用者要求）

`crates/mower_pid_autotune`：`tuning.rs` 逐行移植 `pid_tuning.py`（FOPDT 40×30 粗網格 + 兩輪細化的最小平方擬合、`check_model` 合理性範圍、SIMC PI 與 tau 下限、`step_metrics`、閉迴路模擬器），13 個 pytest 向量搬成 `cargo test`，另加 `tests/pid_tuning_golden.json`：用 Python 模組對 20 組固定樣本（含 6 種錯誤路徑）產生的模型/增益/指標，Rust 在 1e-9 內全部相同，錯誤字串逐字相同。`main.rs`（node 名 `pid_autotune`、同樣 18 個參數與預設、`/pid_autotune` 服務語意、5 Hz latched 狀態 JSON 同鍵序、`/mower_base/pid_command` / `wheel_override` / `led_command` JSON、0x84 確認規則含 flash 重試與 `flash_diag`、review 逾時、每條出口都還原舊增益、mission operation lock）。

驗證：把 `test_pid_autotune_node.py` 的假 STM32 底盤（兩顆一階馬達 + 韌體 PI）改成對子程序跑，Python 節點與 Rust 二進位各跑 8 個情境（完整流程 apply、abort 還原、discard 還原、flash 存檔重試一次、兩次都失敗的 flash_diag 訊息、driver alarm 不擋、無 telemetry、導航中拒絕）：**狀態轉移序列、每個 service 回應、最終狀態/訊息/錯誤、狀態鍵序、marks 鍵、pid_command 序列（persist/closed_loop 旗標與鍵序）、override 訊息集合、LED 訊息、底盤計數器全部相同**，模型/增益/驗證指標在同一假底盤上相差 < 25 %（取樣時刻不同）。唯一差異：Rust 版在 `verify` 指標算完到 `review` 之間兩次發布只隔幾微秒，depth-1 的 latched writer 會把 progress 0.85 那筆併進 0.90（`verify` 欄位仍在後續每筆狀態裡）。開關 `rust_pid_autotune`（compose `RUST_PID_AUTOTUNE`）。

### 7.11 `mower_map` 執行紀錄（map_manage_node 移植，使用者要求）

`crates/mower_map`：`raster.rs` 依 OpenCV 4.6.0 `drawing.cpp` 逐位元重寫 `fillPoly`、`polylines`、粗線 `line`（`FillConvexPoly` + `Circle` 圓端）、`MORPH_ELLIPSE` kernel 與 `erode` / `dilate`（預設與 constant-0 邊界），`tests/raster_oracle.json` 720 個由同版 OpenCV 產生的隨機案例全數逐像素相同；`grid.rs` 移植 `nav_map_fusion.py`（積分影像重取樣）、`map_safety.py`、`image_mask_import.py` 與節點內的風險/區域/通道光柵化（numpy 截斷、floor/ceil、tolerance 語意），pytest 向量搬成 `cargo test`，另有 `tests/map_oracle.json` 用節點自身程式碼在差分測試的幾何上產生的期望值。`main.rs`（node 名 `map_manage`、6 個服務、8 個 latched 地圖、`/map_manage/*` 參數服務含 rclpy 的型別檢查順序與 0.75 m 下限、影像任務備份/還原、每個 mutation 的 guard）。

發現並重現的細節：rclpy 在序列化前把 Python float 留在 float32 `resolution` 欄位裡，所以 Python 節點用 0.05 算格子索引而訂閱者看到 0.0500000007；第一版 Rust 用 float32 算，風險/通道/融合各差一格。`Map` 把 f64 解析度和訊息放在一起後全部相同。

驗證：兩個節點各對同一組假 provider（`/get_record_zone_list` 等三個 service、`/mission_operation_lock`、latched `/nav_operation_active`）跑 47 個步驟（建立自由空間含 2 點退化區域、5 個風險多邊形含單點/兩點/夾邊界、通道含離圖/錯 frame/單點、參數拒絕 0.5/字串/NaN/導航中/鎖被拒、原子設定、影像匯入含風險遮罩/重複匯入/還原/壞編碼/無白色/壞 base64/長度錯/無重疊、空與退化的區域清單、重建，另一個情境為無採集自由空間的純影像匯入/還原/過窄）：**每一筆發布的 OccupancyGrid（frame、幾何、資料 sha1）、每個 service 回應、參數結果與 lock 呼叫序列全部相同**。開關 `rust_map`（compose `RUST_MAP`）。

### 7.12 `mower_coverage` 執行紀錄（coverage_node 移植，使用者要求）

`mower_coverage_core` 改成可不帶 PyO3 建置（feature `python`，wheel 預設開、Rust 節點關），所以 Rust 節點直接連結與 Python 後端同一份規劃器。`crates/mower_coverage`：`contours.rs` 依 OpenCV 4.6.0 `contours.cpp` 移植 `findContours(RETR_EXTERNAL, CHAIN_APPROX_NONE)` 與 `contourArea`（Suzuki 邊界追蹤、輸出順序為最新在前），160 個隨機遮罩逐點相同；`main.rs`（node 名 `boustrophedon_coverage`、4 個服務、`nav_action_follow_path` action client、12 個參數含 rclpy 型別檢查與 startup-only / guarded 規則、marker 版面、風險重取樣、`_send_follow_path` 的 3 s 接受期限與遲到接受取消、`/check_nav_status` 拒絕理由、dispatch 確認、背景/阻塞結果、zone 序列與通道、`_track_and_cancel_navigation_goal` / `_request_nav2_cancel_fallback` 的每 2 s 相關取消重試與單一在途 attempt）。

驗證：兩個節點各對同一組假件（zone map 服務、latched 風險地圖、假 Waypoint action server 含 succeed / fail / reject / hang / 4.5 s 慢接受模式、確認 / 取消 / 狀態 / 通道 / 鎖服務）跑 48 個步驟（無風險、zigzag / spiral / 錯誤 pattern / 邊界環 / 30° / 對齊與重取樣風險 / unknown_as_obstacle / 壞參數 / 無 zone、參數型別與 startup-only、zone 執行成功 / 失敗 / 被拒 / 確認失敗 / 慢接受、導航中拒絕、序列成功 / 執行中拒絕 / hang 後 stop / 通道失敗 / 導航失敗 / 單 zone）：**每個 service 回應、每筆 marker（ns、id、顏色、每個點）與路徑、假件事件序列（goal、confirm、cancel_dispatch、route、lock）全部相同**。開關 `rust_coverage`（compose `RUST_COVERAGE`）。

### 7.13 `mower_agent` 執行紀錄（mower_agent 移植，使用者要求）

`crates/mower_agent`：無 r2r，tokio + tokio-tungstenite（rustls）處理 relay / gate / loopback bridge 三種 WebSocket，ureq（rustls）在 blocking 執行緒做後端與 MediaMTX 的 HTTP（對應 Python 的 `run_in_executor`）。`relay.rs` 移植 `relay_protocol.py`（分塊、重組、pytest 向量）；`main.rs` 保留每一行 log、每個控制訊息、X-Mower-* 簽章與 `mower-agent/<api>` User-Agent、WHEP 路徑規則與 64 KiB 上限、header 過濾、1..60 s 重連退避與被拒後重註冊、30 s 註冊重試、TURN 更新排程、0600 原子寫入 device_key。

驗證：假後端（aiohttp：註冊 / TURN / mrelay1 relay 同一埠）、假 gate（驗 X-Mower-* 的 echo）、假 loopback bridge、假 MediaMTX，各跑 43 個事件的同一劇本（註冊標頭與 body、relay 連線標頭與子協定、心跳含 info/telemetry、open/opened、文字/分塊/二進位/1 MiB 出站分塊、未知 session、WHEP POST/DELETE 轉發與 header 過濾、whip/PUT/壞 headers/超大 body/壞 base64 的 403/413、gate 端結束、hub 端關閉、壞 MAC、gate 不在、MediaMTX 不在的 502、hub 斷線後 1 s 重連、被 401 拒絕後重註冊再 2 s 重連、TURN → MediaMTX PATCH）：**兩者事件相同**（同一秒內的並行啟動順序除外）。開關 `rust_agent`（compose `RUST_AGENT`）。

### 8.1 `mower_ws_bridge` 執行紀錄（Phase 3）

`crates/mower_ws_bridge`：`config.rs`（policy YAML：`topics_sub` / `topics_pub` / `services{name: type}`，fnmatch 風格 `*`）、`auth.rs`（identity.json、base32 secret、HMAC-SHA256、±60 s skew、nonce cache；`compute_mac` 對照 Python 參考值）、`hub.rs`（r2r Node 專用執行緒 spin + 命令通道；每個 topic 一個 ROS 訂閱，QoS 依 publisher 決定（全部 reliable 才 reliable、全部 transient_local 才 latched），一次序列化 fan-out 到所有客戶端，latched topic 對新訂閱者重播最後一筆；publisher / service client 各建一次重用；service 回應在 tokio 上等，不占 node 執行緒）、`client.rs`（rosbridge v2 子集：subscribe/throttle_rate、unsubscribe、advertise、unadvertise、publish、call_service、`/rosapi/topics` 原生回答、status 錯誤）、`main.rs`（tokio-tungstenite 伺服器：`address:port` 走 pairing gate（401），`127.0.0.1:9091` 給 agent 不驗證；64 MB frame；20 s ping）。r2r 沒有 service type 的 graph 查詢，所以 service type 寫在 `mower_bringup/config/ws_bridge.yaml`，測試 `test_ws_bridge_policy_matches_the_rosbridge_allow_lists` 確保與 `rosbridge_params.yaml` 一致。

與 rosbridge_websocket 在同一張圖上比對（本機容器）：String / Bool / BatteryState（NaN → null）/ NavSatFix / Header / PoseStamped 的 `msg` JSON **完全相同**；service 回應相同（含 1.5 s 的慢服務）；publish 到達 ROS 訂閱者；未列入白名單的 subscribe / publish / service 被拒（rosbridge 是靜默不回，Rust 版回 status / result=false）；`/rosapi/topics` 依白名單回答；gate：無標頭 401、重播 nonce 401、壞 MAC 401、正確握手接受並立即收到 latched `/robot/online`。開關 `rust_bridge`（compose `RUST_BRIDGE`），`rosbridge.launch.py` 在 true 時只起 `mower_ws_bridge` + `mower_agent`。

真機影子測試（`--port 9097 --loopback-port none`，從 Mac 走 gate 重放 app 的 21 個訂閱 + 2 個 service）：連線 134 → 14 ms；第一筆訊息 Python 567–1966 ms（逐一處理）、Rust 213–612 ms（並行）；`/robot/online` 第一筆 1386 → 425 ms；`/check_nav_status` 1780 → 507 ms；訊息數與內容相同（`/robot/telemetry` 只差取樣時刻）；`/rosapi/topics` 相同。

**2026-09-19 已切換 `RUST_BRIDGE=true`**：iOS app、Studio、agent 都重新連上；Python rosbridge / rosapi / auth proxy 0 個 process；整機 **388% → 212%**（Python 298 → 106）；溫度 84 → 79 °C。切換後發現兩件事並已修：(1) 剛建立的 service client 在 DDS 配對完成前送出的 request 會被丟掉、永遠等不到回應（app 連線後第一次 `/check_nav_status` 30 s timeout）→ 先等 `is_available`（上限 5 s）再送；(2) Studio 用 `/rosapi/get_time` 量延遲 → 原生回答。並行壓力測試：同一 service 40 筆並行全部回應、1 Hz 輪詢延遲 1–2 ms、不存在的 service 5 s 內回 result=false。未做：`fragment` / `png` / actions（客戶端不用）。

### 8.2 `mower_rsd`（把上面全部合成一個 process，2026-09-23）

所有 crate 拆成 library + 三行 binary，新增 `mower_rsd --modules a,b,c`：同一個 `r2r::Context`（一個 DDS participant），每個模組仍各自 `Node::create` 保留 node 名與參數服務。參數走一份 `--params-file`（以 node 名分節，`mower_bringup/config/mower_rsd.yaml`），remap 走 rcl 的 `-r <node>:<from>:=<to>`。開關 `rust_daemon`（`robot.launch.py`，預設 false，模組集合由既有的 `rust_*` + `enable_gps` 推得）。細節、影子比對與 CPU 數字見 [ROS_FREE_PLAN.md](ROS_FREE_PLAN.md) 的「A5 執行紀錄」。

## 7. Phase 1–2：搬運節點移植（依價值/風險排序）

| 順序 | Rust bin | 取代 | Python 行數 | 現在 → 之後 | 驗證 |
|---|---|---|---|---|---|
| 1（pilot） | `mower_telemetry`（一個 process、四個 node，名稱不變） | telemetry_node、battery_state_node（+battery_estimator）、heartbeat_node、robot_info_node | 422+202+197+100+336 | 42% → ≈ 3% | 影子模式：Rust 版發到 `/shadow/...`，比對 JSON 與 BatteryState 30 分鐘；`/system/update`、`/system/restart` 走 host_request 的 HTTP 端點照舊 |
| 2 | `velocity_command_guard`（同一 bin 兩個實例，參數不變） | mower_bringup velocity_command_guard ×2 | 330 | 29% → ≈ 2% | 安全關鍵：把 mm_test 的契約（stale/invalid 命令 fail-closed、safety zero 一個週期內穿過 rate limiter、只有 mux→controller 一條邊界）寫成 `cargo test` 向量；先在模擬 `system_test.launch.py` 跑，再真機監督測試 |
| 3 | `mower_adapter` | flutter_adapter_node + dto.py | 423+121 | ≈ 6%（Phase 0 後）→ ≈ 1%；214 KB map JSON 編碼從 ~60 ms 降到 ~3 ms | 對每個 `/adapter/*` topic 做 JSON 正規化比對；GetParameters、ZoneMapList、ToLL client 用 r2r 服務 client |
| 4 | `mower_imu` | wit_ros2_imu（第三方 Python） | ~300 | 6% → ≈ 0.5% | `serialport` crate；保留「丟棄積壓、只接受完整新樣本」行為；比對 `/imu/data` 率與數值 |
| 5 | `rosbridge_auth_proxy` | 148 行 Python TCP relay | 148 | 5.6% → 併入 Phase 3（若 Phase 3 延後，先用 tokio 做獨立版） | 現有 HMAC 標頭測試 |

每個 bin 的驗收：影子比對通過 → 對應的 `rust_*` 開關切換 → 真機 24 h 觀察（CPU、`/robot/online` 連續性、app 功能）→ 下一個。

## 8. Phase 3：`mower_ws_bridge`（取代 rosbridge_websocket + rosapi + auth proxy，54% → ≈ 6%）

- 協定：只實作客戶端實際使用的 rosbridge 子集：`subscribe`（含 `throttle_rate`、`queue_length`）、`unsubscribe`、`advertise`、`publish`、`call_service`（timeout）、`status`；`/rosapi/topics` 直接用 graph API 回答（app 的 fleet discovery 用）。App 端 `rosbridge_service.dart` 就是唯一的協定客戶端，mower_agent relay 與 Studio 走同一協定。
- 認證：把 auth proxy 的 HMAC `X-Mower-*` 檢查移到 WebSocket upgrade，直接綁 LAN 位址（少一次 TCP 轉發、少一個 process）。
- 允許清單沿用 `rosbridge_params.yaml` 的 `topics_glob` / `services_glob`。
- ROS 端：r2r `subscribe_untyped` / `publish_untyped` / `create_client_untyped` → `serde_json::Value`；每個 topic 一個 ROS 訂閱、fan-out 給所有客戶端（rosbridge 現在是每客戶端各自序列化）；service client 常駐重用，不再每次 create/destroy。
- 為什麼不直接用 `rosbridge_server_rs`（v0.1.5、0 star、28 commits）：拿來當參考，不當生產依賴。
- 效益不只 CPU：訂閱不再由 Python 串行處理（現在 `/robot/online` 排第 17 個要等 2–3.5 s），service call 從 1.3–1.8 s 降到毫秒級，直接解決 app 開啟時的閃現。
- 驗證：用 `rb_probe.py` 錄下 Python rosbridge 的完整 app 會話（訂閱順序、每個 topic 的 JSON），對 Rust bridge 重放同樣的 op 序列，逐筆比對正規化 JSON（注意 `{sec, nanosec}`、`uint8[]` base64、NaN）；app 真機 e2e；`bridge_impl:=python|rust` 兩版並存一個發布週期。

## 9. 不移植的部分

| 節點 | 理由 |
|---|---|
| ~~nav_action_server（2715 行）~~ | 原判斷「邏輯還在變，移植風險大於收益」；使用者要求後已移植為 `mower_nav`（7.8），真機切換待監督導航 |
| ~~path_record_node（2124 行）~~、~~map_manage（1470）~~、~~coverage_node（1898）~~、auto_coverage、docking | path_record 已移植為 `mower_record`（7.7）、battery_state_node 已移植為 `mower_battery`（7.9）、pid_autotune_node 已移植為 `mower_pid_autotune`（7.10）、map_manage 已移植為 `mower_map`（7.11）、coverage_node 已移植為 `mower_coverage`（7.12）；其餘為幾何/任務邏輯，閒置時 < 4%；coverage 的重運算已經在 `mower_coverage_core`（Rust PyO3） |
| ~~mower_agent~~ | 已移植為 `mower_agent`（7.13） |
| nav2、robot_localization、ros2_control、topic_tools | 已是 C++；只調參數（Phase 0.5、0.6） |

原則：Rust 只做「資料搬運、協定、驅動」；演算法與任務狀態機留在 Python。Phase 3 完成後用同一組量測腳本重評，若上表節點成為新瓶頸再排。

## 10. 風險

| 風險 | 對策 |
|---|---|
| r2r API 變動 | `Cargo.lock` 鎖版；只用 pub/sub/client/parameter 基本面 |
| JSON 格式與 Python 版差異（浮點、時間戳、陣列） | 影子比對 + 錄製重放測試，App 端不改協定 |
| 安全關鍵 guard 移植出錯 | 契約測試向量 + 模擬 + 真機監督；Python 版留在映像裡可一鍵切回 |
| 現有 source-inspection 測試（`test_controller_configuration.py`、`test_qos_configuration.py`）認 Python 原始碼 | 同步改成檢查 launch 參數與 Rust 原始碼 |
| CI 冷建時間 | 首次 +5–8 分鐘；cache-dance 已覆蓋 cargo registry 與 target dir |
| 兩種語言維護 | Rust 限定在 `mower_rs` 一個套件，範圍寫死在本文件 |

## 11. 時程（一人）

| 週 | 內容 |
|---|---|
| 1 | Phase 0 六項修正 + 重量測（2–3 天）；`mower_rs` 套件骨架、Dockerfile、CI 綠燈（2 天） |
| 2 | `mower_telemetry` 影子比對、切換、24 h 觀察 |
| 3 | `velocity_command_guard`（含契約測試、模擬、真機監督）+ `mower_adapter` |
| 4 | `mower_imu`、清理 Python 版（保留一個發布週期） |
| 5–7 | `mower_ws_bridge`：協定子集 + HMAC + rosapi/topics、錄製重放測試、canary、切換 |
| 8 | 重量測、更新本文件、決定第 9 節是否有節點要排入 |
