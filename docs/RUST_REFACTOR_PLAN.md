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
| Phase 0（純 Python 修正） | ≈ 240%（60%） | 脫離熱降頻 |
| Phase 2（Rust 搬運節點） | ≈ 165%（41%） | |
| Phase 3（Rust bridge） | ≈ 120%（30%） | 第一筆 heartbeat < 0.5 s、service call < 100 ms |

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
4. Launch：每個節點加 launch 參數 `rust_nodes:=true|false`（預設 false，逐節點切換），同一映像同時帶 Python 與 Rust 版，回滾只改參數。
5. 開發環境：Mac 上 `cargo test` 跑純邏輯單元測試（不需 ROS）；需要 ROS 的整合測試在 devcontainer / CI arm64 runner。

## 7. Phase 1–2：搬運節點移植（依價值/風險排序）

| 順序 | Rust bin | 取代 | Python 行數 | 現在 → 之後 | 驗證 |
|---|---|---|---|---|---|
| 1（pilot） | `mower_telemetry`（一個 process、四個 node，名稱不變） | telemetry_node、battery_state_node（+battery_estimator）、heartbeat_node、robot_info_node | 422+202+197+100+336 | 42% → ≈ 3% | 影子模式：Rust 版發到 `/shadow/...`，比對 JSON 與 BatteryState 30 分鐘；`/system/update`、`/system/restart` 走 host_request 的 HTTP 端點照舊 |
| 2 | `velocity_command_guard`（同一 bin 兩個實例，參數不變） | mower_bringup velocity_command_guard ×2 | 330 | 29% → ≈ 2% | 安全關鍵：把 mm_test 的契約（stale/invalid 命令 fail-closed、safety zero 一個週期內穿過 rate limiter、只有 mux→controller 一條邊界）寫成 `cargo test` 向量；先在模擬 `system_test.launch.py` 跑，再真機監督測試 |
| 3 | `mower_adapter` | flutter_adapter_node + dto.py | 423+121 | ≈ 6%（Phase 0 後）→ ≈ 1%；214 KB map JSON 編碼從 ~60 ms 降到 ~3 ms | 對每個 `/adapter/*` topic 做 JSON 正規化比對；GetParameters、ZoneMapList、ToLL client 用 r2r 服務 client |
| 4 | `mower_imu` | wit_ros2_imu（第三方 Python） | ~300 | 6% → ≈ 0.5% | `serialport` crate；保留「丟棄積壓、只接受完整新樣本」行為；比對 `/imu/data` 率與數值 |
| 5 | `rosbridge_auth_proxy` | 148 行 Python TCP relay | 148 | 5.6% → 併入 Phase 3（若 Phase 3 延後，先用 tokio 做獨立版） | 現有 HMAC 標頭測試 |

每個 bin 的驗收：影子比對通過 → `rust_nodes` 切換 → 真機 24 h 觀察（CPU、`/robot/online` 連續性、app 功能）→ 下一個。

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
| nav_action_server（2715 行） | 任務執行邏輯、BasicNavigator/nav2 action client；Phase 0 後 ≈ 10%。邏輯還在變，移植風險大於收益 |
| path_record_node（2124 行）、map_manage（1470）、coverage_node（1898）、auto_coverage、docking | 幾何/任務邏輯，閒置時 < 4%；coverage 的重運算已經在 `mower_coverage_core`（Rust PyO3） |
| mower_agent | 0.2% |
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
