# 拿掉 ROS 2：單一 Rust 程序（`mowerd`）計畫

日期：2026-09-19　接續 [RUST_REFACTOR_PLAN.md](RUST_REFACTOR_PLAN.md)（那份把 Python 節點換成 Rust，這份把 ROS 2 中介層本身換掉）。
2026-10-08 補第 8 節（D/E 對照程式碼後的執行計畫）與第 9 節（Phase F：Yocto 自建 OS）。

量測對象：LubanCat-2（RK3568，4×A55 @ 1.99 GHz）、映像 `6b7397`（main `2eea6bf`，RUST_* 全開、RUST_RECORD=false）、待機、nav2 已啟動、2 個 app 連線、62 °C、無降頻。PR #13（EKF 20 Hz、ros2_control 25 Hz）已合併但量測時尚未部署到機器上。

## 0. 結論

| | 單核 % | 整機（4 核）% |
|---|---|---|
| 現況（ROS 圖 + 相機/MediaMTX/tailscale） | 152 | 38 |
| 其中 ROS 圖 | 142 | 36 |
| 其中非 ROS（gst、mediamtx、tailscaled、kworker） | ~10 | 2.5 |
| 留在 ROS、做完 Phase A–C 後（預估） | 55–70 | 14–18 |
| `mowerd` 單體，待機（預估） | 12–20 | 3–5 |
| `mowerd` 單體，導航中（預估） | 25–40 | 6–10 |

拿掉 ROS 可以省掉現在 ROS 用量的 85–90%。**但 CPU 現在已經不是瓶頸**（62% idle、62 °C），戶外的阻礙是 GPS profile、無線、USB hub、避障（見 outdoor readiness 記錄）。所以本計畫的 Phase A–C 無論如何值得做（都在 ROS 裡、低風險、而且是單體的前置零件），Phase D–E 只在第 7 節的條件成立時才做。

## 1. 現況：CPU 去哪了

每執行緒 utime/stime 差分 15 s（腳本見附錄），數字已換算回 `top` 的 10 s 視窗（152% 單核）。

| 群組 | 單核 % | 說明 |
|---|---|---|
| ros2_control_node | 29 | 50 Hz 迴圈；user/sys 各半；pal_statistics 4 個 topic 50 Hz 沒人訂 |
| ekf_node ×2 | 26 | 30 Hz、print_diagnostics 開（PR #13 改 20 Hz/關，待部署） |
| navsat_transform | 6 | 30 Hz timer；health gate 要 0.30 s 內看到 /odometry/gps，所以不能降 |
| nav2 待機 8 個程序 | 26 | controller 6.6、bt 6.0、smoother 4.2、behavior 3.9、planner 3.8、vel_smoother 0.7、waypoint 0.4、lifecycle 0.2；每個都各自訂 /tf 77 Hz |
| 純水管：rsp 4.5、twist_mux 2.4、throttle ×2 6.3 | 13 | 只因為有 ROS 才存在 |
| 11 個 Rust（r2r）節點 | 35 | bridge 6.3、robot_status 6.1、nav 5.2、guards 6.4、imu 2.3、adapter 2.2、battery 1.9、pid 1.9、coverage 1.4、map 1.1、agent 0.1 |
| path_record_node（Python） | 6 | RUST_RECORD 還沒開 |
| 非 ROS | ~10 | gst-launch 3.0、mediamtx 2.4、tailscaled、uvcvideo kworker |

三個證據說這幾乎都是傳輸開銷，不是演算法：

1. **loopback 每秒 1764 個封包**，ROS 圖總量 142% → 每個封包約 0.8 ms CPU。程序內 channel 一跳是個位數 µs。
2. **kernel time 佔 ROS 用量 36%**（`top`：12.1 sy + 1.9 si，busy 38.1）。這是 UDP loopback sendto/recvfrom、futex、context switch。單程序後歸零。
3. **純 DDS 執行緒（recvUC/recv/tev/dq.builtins/gc）佔 19%**。另外 C++ 的 topic_tools throttle 每則訊息 1.3 ms、mower_battery 每秒 18 個事件吃 2.3%、robot_status 每事件 1.2 ms——就算是 C++/Rust 節點，ROS 每個事件的固定成本在這顆 SoC 上也是 1 ms 以上。

真正的工作量（第一原理估，未直接量）：EKF 15 態 20–30 Hz 每個 ≈ 1%；STM32 + IMU 串口 syscall 25–50 Hz ≈ 3–5%（拿不掉）；ws bridge JSON + WebSocket ≈ 2–3%；20 Hz 心跳 ≈ 0.5%；靜態層 costmap ≈ 0。合計 10–15%。導航中另加 RPP 20 Hz、A* 突波、路徑/進度 JSON ≈ +15–20。

## 2. 目標架構

單一二進位 `mowerd`，systemd 直接管（或仍在 compose 裡，但映像不再含 ROS）。

```
                    ┌────────────────────────── mowerd ──────────────────────────┐
 /dev/mower_base ──►│ base      ─┐                                                │
 /dev/imu        ──►│ imu       ─┤   bus (typed broadcast channels,               │
 /dev/gps_rtk    ──►│ gps       ─┼─► 名字沿用現在的 topic 名，讓 ws_bridge JSON 不變)│
                    │ localize  ─┤   ├─ nav (costmap / A* / RPP / behaviors / FSM)│
                    │ map, cover─┤   ├─ guards (inline，在 cmd_vel 進 base 之前)   │
                    │ status    ─┘   ├─ record / sites                            │
                    │                ├─ ws_bridge (rosbridge JSON, HMAC)  ◄──► app │
                    │                └─ agent (wss relay, TURN)        ◄──► backend│
                    └─────────────────────────────────────────────────────────────┘
```

設計原則：

- **每個模組 = 純函式核心 + 薄 IO 層**，延續現在 `core.rs` / `estimator.rs` / `raster.rs` 的做法。核心不碰時間、不碰 IO，測試用向量。
- **bus**：`tokio::sync::broadcast` 或自寫的 SPMC ring，訊息型別是 Rust struct（不再 CDR）。channel 名沿用 ROS topic 名（`/robot/online`、`/map_grid`、`/odometry/global` …），ws_bridge 對 app 的 JSON 一個 byte 都不用改。
- **時間抽象**：所有模組拿 `Clock` trait，replay 時餵錄好的時間。
- **執行緒模型**：base 迴圈、IMU、GPS 各一條專用 thread（可選 SCHED_FIFO）；其餘跑在 tokio runtime。cmd_vel 到 base 這條路徑不經 tokio，避免排程抖動。
- **失效模式**：模組 `catch_unwind` + supervisor thread 重啟該模組；整個程序死掉由 systemd 拉起；韌體 300 ms command timeout 仍是最後一道停車線。
- **可選 `--ros-tap`**：cargo feature，把 bus 上指定幾個 channel 用 r2r 發出去給 rviz / ros2 bag 用。預設不編。

不用動的介面：app 協定（ws_bridge 已原生講 rosbridge JSON + HMAC）、STM32 串口協定（`mower_hardware/mower_protocol.cpp`，1.1k 行 C++ 直接照抄）、backend wss/TURN、identity.json/配對、`~/.mower/sites`。

## 3. 階段

每個階段結束都要有一組數字（同一個量測腳本、同一個狀態：待機 + nav2 起 + 2 個 app 連線）。

### Phase A：留在 ROS 的低風險收斂（2–3 天，預估 152 → 70–85）

| # | 動作 | 預估省下 | 驗收 |
|---|---|---|---|
| A1 | 部署 PR #13（EKF 20 Hz、無 diagnostics、ros2_control 25 Hz） | 20–25 | ekf 各 < 8%，ros2_control < 18%；nav2 無 transform_tolerance 警告；監督試車一次 |
| A2 | `RUST_RECORD=true` | 4 | path_record_node 消失，mower_record 6% 以下 |
| A3 | nav2 `use_composition: true`，8 個 server 進一個 `component_container_isolated`，開 intra-process | 10–15 | nav2 程序數 8 → 2；/tf 訂閱者 -6 |
| A4 | local costmap update 5 → 2 Hz、global 1 Hz 維持 | 1–2 | |
| A5 | mower_rs 11 個 binary 合成一個 `mower_rsd`（一個 r2r Context/Node，模組間 channel，launch 用 `rust_*` 開關決定啟用哪些模組） | 10–15 | Rust 程序 11 → 1；DDS 執行緒 -30 條；每個模組輸出仍與獨立 binary byte 相同（沿用 shadow 比對法） |

A5 就是 `mowerd` 的骨架：bus、模組 trait、supervisor。之後每個 phase 都是往這個骨架裡加模組。

#### A5 執行紀錄（2026-09-23，`feat/rf-a5-mower-rsd`）

`src/mower_rs` 的每個 crate 拆成 library（`pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult`）+ 三行的 `main.rs`，舊的一個一個 binary 照樣建、照樣跑（launch 與 `RUST_*` 開關在整個切換期間都不用動）。新的 `mower_rsd` 把任意子集跑在同一個 `r2r::Context` 上。

- **為什麼一個 Context 就夠**：`r2r::Context::create()` 是 process 級 `OnceLock`，十三個模組共用同一個 context，也就是 **一個 DDS participant**、一套 discovery 執行緒、一個 tokio runtime。每個模組仍各自 `Node::create`，所以 node 名、namespace、topic/service/action、每個 node 的 parameter service 全部不變。
- **參數**：不發明新機制。`--params-file` 由 rcl 解析進共用 context，`Node::create` 只把「寫著自己 node 名」（或 `/**`）的那一節交給該 node——跟以前每個 process 自己吃 `--ros-args` 完全一樣。因此 `mower_bringup/config/mower_rsd.yaml` 一個檔案、一個 node 一節即可；`rust_daemon_params_file:=` 可指向 `~/.mower` 的副本。
- **remap**：也是 rcl 的——`-r <node>:<from>:=<to>` 只作用在該 node，IMU 與兩個 velocity guard 的 launch `remappings=` 就這樣照搬。**絕不能**傳不帶前綴的 `-r __node:=`，那會把 process 裡每個 node 都改名；這也是模組表要自帶正式 node 名的原因（`path_record_node`、`imu`、`gps` 與 binary 內建預設不同）。
- **bridge / agent** 的命令列用 `--module-args <id>=<args>` 傳。
- **監督**：每個模組是包了 `catch_unwind` 的 tokio task。任何模組 panic、回傳 `Err`、或在沒被要求停止時就結束，都會記錄原因，2 s 後只重啟該模組（等同 launch 的 `respawn_delay`），其他模組照跑，而且不重建 DDS participant，所以缺硬體的驅動一直重啟也不會造成 discovery 風暴。只有關機過程中失敗、或模組在 10 s 窗口內停不下來，process 才以非 0 結束讓 launch `respawn`。沒有 `panic = "abort"`。第一版是任一模組失敗就整個 process 退出，2026-09-23 在沒接 IMU/GPS 的機器上實測變成 13 個節點每幾秒全部重建，因此改掉。
- 早期版本的一個 bug：`Shutdown::trigger` 用 `watch::Sender::send`，在當下沒有任何 subscriber 時會失敗而且**不會**更新值，等於丟掉一次 shutdown。改成 `send_replace`，並補了測試。

**影子比對**（`src/mower_rs/tools/shadow_compare.py`，arm64 Jazzy 容器、`ROS_DOMAIN_ID=82`）：一個 rclpy harness 同時扮演假上游（odom / GPS / IMU / base telemetry / 地圖與 marker 圖層 / 搖桿與 mux 速度命令）與假服務（`/boustrophedon_coverage`、`/map_manage` 的 get_parameters、`/get_zone_map_list_srv`、`/toLL`），錄下所有輸出。同一組輸入分別餵給「五個獨立 binary」與「一個 `mower_rsd`」：

23 個 topic 全部 **IDENTICAL**（`/adapter/robot_pose` 139 筆逐筆相同、兩個 guard 的接受/拒絕序列相同、四張 map layer 與六張 marker layer 的 JSON 逐位元組相同、`/robot/info`、`/robot/online`、`/adapter/{coverage_settings,zone_summaries,map_datum}`、`/mower_base/led_command` 相同）。只有兩處給了容差並記錄原因：`/battery_state`、`/aon_battery_state` 1e-3（sag 低通與庫侖積分的 dt 是真實時間），`/robot/telemetry` 2e-2（10 Hz 快照對 10 Hz 輸入，兩邊 timer 相位不同，偶爾抓到前一筆 odom 或落後一筆的 rate 估計）——與 Python→Rust 移植時記錄的「只差取樣時刻」同一類。

**CPU 與執行緒**（同一台 arm64 容器、同樣的假輸入速率、`/proc` 的 utime+stime 30 s 差分）：

| 模組集合 | | process | 單核 CPU % | 執行緒 |
|---|---|---|---|---|
| status, battery, adapter, guards（5 個 node） | 獨立 binary | 5 | 6.76 | 85 |
| | `mower_rsd` | 1 | **5.60** | **23** |
| 再加 map, coverage, nav, record, pid_autotune（11 個 node） | 獨立 binary | 10 | 14.66 | 174 |
| | `mower_rsd` | 1 | **6.36** | **28** |

也就是 11 個 node 的情況下 CPU −57%、執行緒 −84%、process 10 → 1。容器是 Apple Silicon 上的 arm64，每個 process 的 DDS 固定成本比 RK3568 低（計畫裡量到的是每事件 1 ms 以上），真機的絕對收益應該更大；**真機尚未切換**，`rust_daemon` 預設 false。

待辦：真機開 `rust_daemon:=true` 跑一輪（含監督試車，因為 guards 在裡面），並在同一份量測腳本下記錄 LubanCat 的數字。

2026-09-30 補記：Python coverage 已移除（見 [RUST_REFACTOR_PLAN.md](RUST_REFACTOR_PLAN.md) 7.12 補記），`coverage` 模組不再有 `rust_coverage` 開關。`rust_daemon:=true` 時 `robot.launch.py` 永遠把它放進模組集合；`rust_daemon:=false` 時由 `mission.launch.py` 起獨立的 `mower_coverage`。其餘模組仍照各自的 `rust_*` 開關。


### Phase B：Rust base driver 取代 ros2_control（3–4 天，預估 -20）

取代 `controller_manager` + `diff_drive_controller` + `mower_hardware` + `joint_state_broadcaster`（`robot_state_publisher` **留著**，見下方執行紀錄）。

- 移植 `mower_protocol.cpp`（幀格式、CRC、status/command）到 `mower_base` crate；serial 用專用 thread，`VMIN=0` 非阻塞讀法照舊。
- 差速運動學 + odom 積分（照 diff_drive_controller 的公式與 covariance），/odom、/joint_states、tf odom→base_link 由它發。
- 安全減速界限、command timeout 前的重送、safety-zero 行為要和 controllers.yaml 裡的參數一模一樣。
- 驗證：同一根串口不能兩個程序同時開，所以用 **錄放法**：先錄 ros2_control 在真機上 10 分鐘的串口 rx/tx + /odom，離線把 rx 餵給 Rust driver，比 tx 幀與 odom 序列（容差 1e-9）；再監督試車（搖桿、放開即停、一段 nav）。
- 這階段仍是 ROS 節點（在 `mower_rsd` 裡）。pal_statistics 那 323 KB/s 順便消失。

#### B 執行紀錄（2026-09-23，`feat/rf-b-base-node`）

**狀態：程式與驗證完成，`rust_base` 預設 false，真機尚未切換（缺監督試車）。**

新 crate `src/mower_rs/crates/mower_base`（node 名 `mower_base`，也是 `mower_rsd` 的 `base` 模組），
取代 `ros2_control_node`（controller_manager）+ `mower_hardware::MowerSystem`（含它的
`mower_hardware_info` node）+ `diff_drive_controller`（`diff_controller`）+
`joint_state_broadcaster` + 兩個 spawner。**`robot_state_publisher` 沒有取代**——它留著，
也正是 `/joint_states` 必須照樣每個週期發的原因（原計畫寫「取代 robot_state_publisher」是錯的）。

所有計算都用已經對過 C++ 測試向量的 `mower_base_core`（協定、rate limiter、odometry、
控制器週期、telemetry JSON）；這個 crate 只做搬運。

- **參數預設值取自真正啟動的那份檔** `mower_controller/controllers/diff_drive_controller.yaml`
  （`open_loop: true`、`enable_odom_tf: false`、`base_footprint`、0.35/0.09 m、
  `cmd_vel_timeout: 0.25`），**不是** `mower_hardware/config/mower_controllers.yaml`——
  那是 bench 用的，上述每一項都不同。
- **QoS 跟 upstream 要的一樣是 `SystemDefaultsQoS`**（`/odom`、`/tf`、`/joint_states`、
  cmd_vel 訂閱），由各 RMW 自己解讀：機器上的 CycloneDDS 是 reliable + volatile、keep last 1；
  Fast DDS 會變成 transient_local 的 writer、best_effort 的 reader。先前版本把 Fast DDS
  容器裡量到的值寫死，上機後 cmd_vel 會是 best_effort（C++ 是 reliable），已改
  （`fix/rf-base-preflight`），`base_compare.py` 在 CycloneDDS 下逐端點比對。
  `/joint_states` 的 `effort` 是兩個 NaN，`frame_id` 是 `base_link`。
- **比 C++ 多的兩件事**：重啟／回授遺失後的 arm latch（停止邊緣之前輪子保持 0），
  以及控制時鐘用 CLOCK_MONOTONIC（系統時鐘只用在訊息 stamp）。見 `src/mower_rs/README.md`。
- **執行緒**：25 Hz 迴圈是專用 std thread（`VMIN=0` 非阻塞讀 → `tick` → 寫 → 自己發佈；
  r2r 的 publish 就是一次 `rcl_publish`，不需要 executor）。tokio 只負責 spin 六個訂閱，
  每個訂閱把請求寫進 mutex slot，迴圈每週期取走一次；**不用 channel**，因為塞住的 channel
  會把過期的 cmd_vel 遲到送達，而 C++ 的 realtime box 正是要避免這件事。
- **失效**：任何 serial 錯誤 → `BaseCycle::fault()` → 送 stop burst → 回 `Err`，
  launch（或 `mower_rsd` supervisor）2 s 後重啟；在那之前韌體自己的 300 ms command timeout
  已經把輪子停住了。
- **刻意不做**：`/controller_manager/*` 服務與 controller lifecycle（repo 內沒有任何呼叫者，
  只有 spawner 和 `ros2controlcli`）、pal_statistics 的 introspection topic、
  `/dynamic_joint_states`（它帶的十一個 `mower_base/*` 診斷值全都已經在
  `/mower_base/telemetry` JSON 裡，沒有其他消費者）、`~/cmd_vel_out`、`robot_description` 參數。

**差分驗證**（`tools/fake_base.py` + `base_harness.py` + `base_compare.py`，arm64 Jazzy 容器，
`ROS_DOMAIN_ID=86`；同一根 pty 上先後跑兩套，同一份腳本化 `/cmd_vel` 與側通道）：

| | ros2_control | mower_base |
|---|---|---|
| `/odom`、`/joint_states` | 25.001 / 25.002 Hz | 25.004 / 25.004 Hz |
| `/mower_base/telemetry` | 15.07 Hz | 15.37 Hz |
| 六個 topic 的 endpoint 與 QoS | — | 完全相同 |
| telemetry JSON 欄位 | 84 | 84，無多無缺 |
| odometry | — | 走 3.006 m 後差 1.9e-3 m（0.06 %）；平均 \|dx\| 9.8e-4 m |
| 0x01 指令幀 | 25.007 Hz、timeout 300 | 25.029 Hz、timeout 300；40 ms 網格上 median \|d permille\| = 1 |
| 安全歸零時刻 | 13.532 s | 13.520 s（差 12 ms，不到一個週期） |
| wheel_override 爆發 | 56 幀 16.533–18.733 s | 56 幀 16.520–18.720 s（差 13 ms） |
| 0x02/0x03/0x04/0x06/0x07 payload | — | 逐位元組相同 |
| CPU（34 s utime+stime） | 19.16 % 單核 | **6.49 %** 單核（debug build） |

odometry 的殘差是兩次獨立 wall-clock 跑的取樣相位差，不是算術差：`mower_base_core` 的
oracle 向量已經把 limiter、odometry 與 200 週期的控制器對到 1e-12。重跑數次落在
8.6e-4 ~ 1.9e-3 m，方向不固定。容器是 Apple Silicon 上的 arm64，RK3568 上的絕對收益
應該更大（計畫裡量到 ros2_control_node 佔 29 %，PR #13 降到 25 Hz 後約 28 %）。

待辦：真機 `rust_base:=true` 監督試車（搖桿、放開即停、一段 nav、拔掉串口看 fail-closed
與重啟），並照 `crates/mower_base_core/README.md` 的格式錄一段真機串口做離線 replay 比對。

### Phase C：定位（3–4 天，預估 -15~20）

- `mower_localize`：移植 robot_localization 的 EKF（`ekf.cpp` + `filter_base.cpp`，約 1.5k 行；只做這台車用到的 15 態、odom twist + IMU yaw/yaw-rate + GPS pose 三種輸入、兩個實例 odom/map）與 `navsat_transform`（datum、UTM、/odometry/gps）。
- 驗證：用 `make record` 的 bag 餵相同輸入序列，比 /odometry/local、/odometry/global、tf 差 < 1e-6；再看 /adapter/map_datum 與場地庫重投影不變。
- 完成後 A–C 合計預估 55–70% 單核；**第 7 節的 Go/No-Go 在這裡決定**。

#### C 執行紀錄（2026-09-23，`feat/rf-c-localize-node`）

核心（`crates/mower_localize_core`，EKF + preprocessing + navsat/UTM）先前已移植並用容器內的 `librl_lib.so` 產生的向量驗到 1e-15；這一輪補的是 ROS 外殼 `crates/mower_localize`。

- **三個節點、一個模組**：節點名維持 `ekf_filter_node_odom` / `ekf_filter_node_map` / `navsat_transform`，所以 graph、log 與每個 node 的 parameter service 都沒變。在 `mower_rsd` 裡是三個各自被監督的 instance（某個 filter 掛掉只重啟它自己，等同原本三個 launch `Node`），獨立 binary 則把三個角色跑在同一個 process，任一個結束就整個 `Err`。`run()` 用 node 名決定角色。
- **不走 `/tf`**：移植的程式碼只查四個 transform，兩個靜態的（`base_footprint <- imu_link` / `<- gps_link`，來自 `/tf_static`）和兩個動態的（`base_footprint <- odom`、`map <- base_footprint`），而後兩個正是這兩個 filter 自己廣播的。所以它們走 process 內的 `tfbus`，不從 `/tf` 讀回來——那個 topic 還載著 robot_state_publisher 的輪子 joint，77 Hz，在 RK3568 上每則約 1 ms CPU，正好是這個 phase 想省的錢。代價是「別人發的 `odom -> base_footprint` 看不到」，本車沒有（`diff_drive_controller` 的 `enable_odom_tf: false`）。
- **沒有複製**：`/set_pose`、`/enable`、`/toggle`、`/reset`（沒人呼叫，而且同 namespace 的兩個 `ekf_node` 本來就互相蓋掉）、`/setUTMZone`（要 MGRS，核心沒移植）、`/diagnostics`（`print_diagnostics: false`，上游仍會 advertise 但不發）、以及「set parameter 成功但不生效」——這裡直接回絕並附理由。launch 的 `remappings=` 變成 node 範圍的參數，預設值就是生產線路。
- **開關**：`dual_ekf_navsat.launch.py rust_localize:=true`（compose `RUST_LOCALIZE`），預設 false；`rust_daemon:=true` 時改成把 `localize` 放進 `mower_rsd` 的模組集合。兩條路徑互斥有測試把關（兩個 `map -> odom` 發布者會搶 tf，而 nav2 就是靠它導航）。

**差分比對**（`src/mower_rs/tools/localize_compare.py`，arm64 Jazzy 容器、`ROS_DOMAIN_ID=87`）：一個 rclpy harness 扮演整個上游——`/tf_static`（真實的 GPS 天線與 IMU 偏移）、25 Hz 帶 covariance 的 `/odom`、10 Hz `/imu/data`、4 Hz 且落在 UTM 51N 的 `/fix`，並注入過期 0.6 s、亂序與重複時戳——錄下五個 topic 與兩個 tf，最後問 `/toLL`、`/fromLL`、`/datum`。時戳用 `t0 + k*dt` 產生，兩次獨立的 run 因此共用同一條相對時間軸，可以逐筆對時戳比。開頭兩秒靜止：datum 是「odom/IMU/GPS 都到齊時最新的那一筆 fix」，不靜止的話兩次 run 會錨在不同的 fix 上（第一版就是這樣，map frame 差 3 cm）。

必須完全一致的部分都一致：`/toLL` 7.1e-15、`/fromLL` 4.7e-10 m、下 `/datum` 之後的 `/toLL` 1.1e-14、`map -> utm` 靜態 transform 4.7e-10、所有 frame / child_frame / status、以及速率。

filter state 不可能逐位元相同，正確的讀法是跟「同一套實作跑兩次」比。30 s、逐時戳配對後的最大絕對差：

| | C++ vs C++ | Rust vs Rust | C++ vs Rust |
|---|---|---|---|
| `/odometry/local` 位置 | 2.7e-4 m | 2.4e-4 m | 2.0e-4 m |
| `/odometry/local` 姿態 | 1.4e-15 | 8.0e-4 | 1.6e-6 |
| `/odometry/global` 位置 | 2.28e-2 m | 2.11e-2 m | 2.28e-2 m |
| `/odometry/gps` 位置 | 4.1e-3 m | 3.2e-3 m | 3.3e-3 m |
| `/gps/filtered` 經緯度 | 2.3e-7 度 | 2.1e-7 度 | 2.3e-7 度 |
| tf `map -> odom` 位置 | 1.40e-1 m | — | 1.35e-1 m |

也就是 C++ 自己跟自己差多少，Rust 跟 C++ 就差多少（Rust 自己跟自己還更大）。原因是 `periodicUpdate` 跑在牆鐘上：`prepareTwist` / `prepareAcceleration` 的槓桿臂項取自**當下**的 filter state 與 timer 微分出來的角加速度，而且 queue 空掉的那個 tick 會 predict 到現在並重新蓋時戳。差異集中在加速度斜坡開始的那一刻，之後緩慢累積。算術本身是核心的，對真實 library 到 1e-15。

**CPU**（同一個容器、同樣的假輸入、`/proc` utime+stime 30 s 差分；僅供參考，RK3568 每個 process 的固定成本高得多）：

| | process | 執行緒 | 單核 CPU |
|---|---|---|---|
| `ekf_node` ×2 + `navsat_transform_node` | 3 | 48 | 8.2 % |
| `mower_localize`（release） | 1 | 21 | **4.4 %** |

待辦：真機開 `rust_localize:=true` 跑一輪監督試車（nav2 靠 `map -> odom` 導航），並在同一份量測腳本下記錄 LubanCat 的數字與 23 % 的對照。

### Phase D：導航子集（2–3 週，預估 -25，風險最高）

只移植這台車設定檔實際啟用的東西，不做通用 nav2：

| nav2 元件 | 替代 | 估行數 |
|---|---|---|
| costmap_2d：StaticLayer ×2 + inflation | `mower_map` 已有像素精確的光柵 → 加 inflation kernel | 300 |
| NavfnPlanner | 網格 A*（或直接 NavFn 的 Dijkstra 位勢場） | 400 |
| SimpleSmoother | 直接移植 | 200 |
| RotationShim + RegulatedPurePursuit | 直接移植（參數表照 yaml） | 700 |
| SimpleProgressChecker / SimpleGoalChecker | 直接移植 | 100 |
| Spin / BackUp / DriveOnHeading / Wait / AssistedTeleop | 直接移植 | 400 |
| bt_navigator（NavigateToPose / ThroughPoses 的 BT XML） | 狀態機（`mower_nav/state.rs` 已有一半） | 600 |
| velocity_smoother | 併入 guards | 100 |
| lifecycle / bond / waypoint_follower / docking_server | 不要（docking 目前沒啟動；apriltag docking 另議） | 0 |

- 驗證分三層：(1) 純函式核心用 nav2 的測試向量；(2) **shadow 模式**：Rust nav 跑在 bus 上，算出的 cmd_vel 只記錄不送，和 nav2 的 /cmd_vel 逐週期比，跑完整的割草任務錄放；(3) 監督試車：搖桿、中途停、recovery 觸發。
- 從這裡開始 `mower_nav`（action server 包裝）改成直接呼叫內部 nav，不再走 nav2 action。

### Phase E：切換與收尾（1 週）

- ws_bridge、agent、robot_status、guards、record、map、coverage、battery、pid_autotune、imu 從 r2r 訂閱改讀 bus（大多只是把 `r2r::Subscriber` 換成 `bus.subscribe("/topic")`）。
- GPS：ublox UBX 解析（只要 NAV-PVT/NAV-HPPOSLLH + RTCM 透傳）→ `/fix`。這順便把 outdoor 記錄裡「gps profile 從沒啟動過」的問題一起處理。
- 映像：拿掉 ROS 基底（1.93 GB → 預估 < 100 MB）、CI 從 ~40 min 降到幾分鐘、開機到心跳從 ~90 s 降到幾秒。
- `mower_sim`：base driver 加 `--sim` 後端（運動學模型 + 假 IMU/GPS），app 開發用；worlds/urdf 不再需要。
- 錄放：bus tap 寫自家 binary log（`mower_recorder` 已是 opt-in 節點，改寫成 tap）；replay 就是把 log 餵回 bus + 假時鐘。
- 觀測：`mowerctl topic echo/hz`、`mowerctl node` 走 ws_bridge，不會像 `ros2 topic hz` 那樣造成 overrun。
- 拆掉：compose 裡的 `RUST_*` 開關、rosbridge 相關 yaml、launch 檔。

## 4. 不再有的東西（決策前要接受）

- rviz、`ros2 bag`、`ros2 topic`、launch_testing、rqt。全部要自己補（第 3 節 Phase E 的 mowerctl/recorder）。
- nav2 生態：obstacle layer、雷達、SLAM、STVL、MPPI 之類以後要用就得自己寫。若產品路線圖裡有避障感測器（outdoor 記錄：目前沒有任何避障），這是最大的成本。
- nav2 的現場驗證履歷。RPP 與 recovery 自己寫過後就是自己負責。
- 對 Python 節點的差分測試法（那些節點已經沒了，之後基準變成 Rust 自己）。

## 5. 風險與緩解

| 風險 | 緩解 |
|---|---|
| cmd_vel 路徑安全 | guards 核心已是 Rust 且有向量測試；base driver 沿用韌體 300 ms timeout；Phase B/D 都要監督試車，不得無人翻開關 |
| 單程序一個 panic 全倒 | 模組級 catch_unwind + supervisor；systemd Restart=always；`panic = "abort"` 不要用 |
| EKF 數值漂移 | 錄放差分 < 1e-6；保留 robot_localization 當離線 oracle |
| RPP 行為差異（震盪、轉彎切角） | shadow 模式先跑完整任務比 cmd_vel；參數表機械式對照 |
| 即時性（tokio 排程抖動） | base/IMU/GPS 專用 thread；cmd_vel 不經 tokio；量 25 Hz 迴圈的 jitter 直方圖當驗收 |
| 開發期失去可視化 | 先做 `--ros-tap` feature，Phase D 之前都還能開 rviz |
| 同一串口不能雙開，差分只能離線 | 錄放法（Phase B） |

## 6. 時程（一人 + Claude）

| Phase | 天 | 累計省下（單核 %） |
|---|---|---|
| A | 2–3 | 70–85 |
| B | 3–4 | 90–105 |
| C | 3–4 | 105–120 |
| D | 10–15 | 130 |
| E | 5 | 130–140 |

A–C 約兩週，D–E 再三到四週。

## 7. Go/No-Go（Phase C 結束時決定 D–E）

做 D–E 的條件，至少要有兩個成立：

1. 功能集已定型，且路線圖裡沒有需要 nav2 生態的感測器（雷達、深度相機避障）。
2. 需要 ROS-free 映像的好處：開機秒數、映像大小、CI 時間、少一層要跟 Jazzy 版本走的相依。
3. 熱：要拿掉散熱片/風扇或塞進更小的殼，需要待機 < 5% 整機。
4. 要在更便宜的 SoC 上跑同一套（RK3566、雙核 A53 之類）。

只是為了 CPU 數字，不值得做 D–E：A–C 之後整機約 15%，剩下的收益是 10 個百分點的整機 CPU，換三到四週和一份自己維護的導航堆疊。

## 8. D/E 執行計畫（2026-10-08，對照程式碼後）

2026-10-08 起機器已經跑在 `RUST_BASE` / `RUST_LOCALIZE` / `RUST_DAEMON` 上（PR #38 把三個預設翻成 true，機器 `.env` 明寫），
剩下的 ROS 程序只有 `mower_rsd`、nav2 的 component container、`robot_state_publisher`、`twist_mux`。
第 7 節的 Go/No-Go 由「要做 Yocto 自建 OS、需要 ROS-free 映像」（條件 2）決定做 D–E；條件 1（感測器路線圖）仍要自己確認：拿掉 nav2 之後避障要自己寫。

### Phase D 的實際範圍

對過 `crates/mower_nav` 之後，D 比第 3 節寫的小：`mower_nav` 只碰 nav2 的三樣東西，
`navigate_to_pose`（去割草起點，走預設 BT）、`follow_path`（每一段直接丟 controller_server，**不經 BT**）、`bt_navigator/get_state`（就緒檢查）。
NavigateThroughPoses、waypoint_follower、smoother_server、assisted_teleop、docking 沒有任何呼叫者。
兩張 costmap 都只有 `static_layer`，沒有 inflation 層（inflation 在 `mower_map` 就做完了）；
local costmap 只是同一張 `/map_grid` 轉到 odom frame，用 `/odometry/global` 的 map 座標查同一張圖就等價。

| # | 要寫的 | 來源 / 估行數 | 備註 |
|---|---|---|---|
| D1 | `mower_nav_core` 的 costmap 查詢 | ~150 | `/map_grid_global` 的 OccupancyGrid 在 map frame 查格子；lethal / unknown 規則照 StaticLayer（`track_unknown_space: true`） |
| D2 | NavFn | ~400 | `use_astar: false`、`allow_unknown: true`、`tolerance: 0`。格子成本二值，`mower_coverage_core/connector_planner.rs` 的 A* 也能用，但要和 nav2 逐週期比就照移植 |
| D3 | RotationShim + RegulatedPurePursuit + SimpleProgressChecker + SimpleGoalChecker | ~800 | 參數表機械式對照 `nav2_no_map_params.yaml` 的 40 個 key；`use_collision_detection: true` 的弧線碰撞檢查查 D1 |
| D4 | velocity_smoother | ~100 | 0.26 m/s、1.0 rad/s、accel 2.5 / 3.2 的限速器，位於 controller 與 `/nav_cmd_vel` 之間 |
| D5 | NavigateToPose 流程 | ~300 | 預設 BT 是「每秒重規劃，失敗才 recovery」。建議只做重規劃 N 次後停車回報，不移植 Spin / BackUp / Wait：靠靜態圖走的車 recovery 意義不大，`mower_nav` 的失敗路徑本來就通知 app。要移植的話另加 ~400 |
| D6 | `mower_nav` 改接內部呼叫 | 改既有 3.2k 行的 IO 層 | 兩個 action client 換成直接呼叫；對 app 與 coverage node 的 action、service、`/coverage_progress`、20 Hz 心跳不變 |
| D7 | shadow 工具 | ~300（Python） | Rust controller 訂同樣輸入，cmd_vel 發到 shadow topic；錄一整趟割草 bag 逐週期比 `/nav_cmd_vel`。照 `tools/localize_compare.py` 的做法 |

驗收三層照第 3 節：核心用 nav2 的測試向量；shadow 跑完整任務；最後 `rust_nav_core:=true` 監督試車，guards 與 `mower_base` 的 arm latch 是安全網。

**D 之前的兩個前置**：map EKF 的 `initial_estimate_covariance` 根治（handoff「還沒做的 1」）要先決定，否則 shadow 是在比兩個都歪 10–15° 的軌跡；戶外要有 RTK fix，D7 的 bag 必須在真實場地錄。

### Phase E 的實際範圍

| # | 要做的 | 重點 |
|---|---|---|
| E1 | `mower_bus` crate | typed broadcast，channel 名沿用 topic 名；map / marker / `/robot/info` 這類 latched topic 要有 transient-local 語意；`Clock` trait 給 replay |
| E2 | 15 個 crate 的 IO 層從 r2r 換 bus | base、localize、guards 已是「純核心 + 薄 IO」，機械式替換。參數改成 mowerd 讀一份 YAML、按模組名分段，key 不變 |
| E3 | `ws_bridge` 的 schema registry | 現在靠 r2r untyped + ROS introspection 把任意訊息轉 JSON。改成每個 bus 型別用 serde 產出和 ROS 訊息同樣的 JSON 欄位佈局；範圍是 `.cargo/config.toml` 那 17 個套件裡實際用到的 21 個型別。`/rosapi/topics` 與 app 呼叫的兩個參數服務（`/boustrophedon_coverage/set_parameters`、`/map_manage/get_parameters`）在這裡模擬 |
| E4 | 併入 twist_mux、robot_state_publisher | mux 是優先權加 lock 約 100 行；rsp 只剩 imu_link / gps_link 兩個靜態 transform，從 xacro 抄進設定 |
| E5 | recorder 重寫 | app 用六個 `/mower_recorder/*` 端點，加 Bag 頁的改名、刪除、上傳 R2，加 MediaMTX 錄影 API 與尾燈。用 Rust 的 `mcap` crate 寫 MCAP，Foxglove 與 `mower-check-run` 不用重寫。資料收集 profile（gscam 進 MCAP 給 GrassVision）要決定留不留 |
| E6 | host 整合 | `firmware/tools/mower_flash.py`（480 行）移植進 `firmware-sync`，協定核心 `mower_base_core` 已有；link-status、pairing、host.request 收進 mowerd。第一版保留 `~/.mower` 的 JSON 檔契約，一次只換一層 |
| E7 | 打包 | 先不脫離 Docker：`debian:bookworm-slim` + `mowerd` 約 40 MB，`mower-update.sh` 與 OTA 流程不動，每次更新只剩幾 MB，pull 的負載（2026-10-08 讓板子硬重開的那種）也跟著消失。原生 systemd 留給 Phase F |
| E8 | 觀測與模擬 | `mowerctl topic echo/hz` 走 bridge 的 loopback 9091；`--ros-tap` cargo feature 讓 D/E 開發期還能開 rviz；`mower_base --sim` 後端給 app 開發；replay 是把 MCAP 餵回 bus 加假時鐘 |
| E9 | CI 與清理 | cargo 交叉編 aarch64 幾分鐘取代 40 分鐘 QEMU colcon；留一個有 ROS 的 dev container 當離線 oracle；最後拆 launch、`RUST_*` 開關、cyclonedds、sysctl、package.xml |

**時程**：D 10–15 天同第 6 節；E 第 6 節的 5 天太樂觀，recorder 與 bridge registry 各要三四天、host 整合兩三天，抓 2–3 週。
**順序**：D1–D4 核心 → D7 shadow 工具 → 戶外錄 bag 比對 → D5/D6 切換試車 → E1–E3 → E4–E6 → E7 → E9。E7 之後機器上是一個 40 MB 容器跑一個 binary，Phase F 只要把它搬到 systemd 下。

## 9. Phase F：Yocto 自建 OS

目標：映像裡只有 `mowerd` 加五六個第三方 daemon，永遠不碰 meta-ros。所以 F 排在 E7 之後，
E 沒做完之前不要開始 BSP：先在現在的 Debian 12（kernel 6.1.99-rk356x）上證明單一 binary 能跑，Yocto 只搬運。

### 現在 host 上有什麼（要一比一帶過去）

| 現況（`deploy/`） | Yocto 對應 |
|---|---|
| Docker + compose、`pid/ipc/network_mode: host`、`/dev` bind mount、device cgroup、`99-mower-dds.conf` | 全部不要。`mowerd.service` 直接跑，`After=dev-stmcom.device` 不需要：supervisor 本來就會等裝置 |
| `mower.service`、`mower-host-request.path/.service`、`mower-update.*`、`mower-link-status`、`mower-camera`、`mower-lte`、`ntpsec` drop-in | `mowerd.service`、`mower-camera.service`（照抄 `mower-camera.sh` 的 gst 管線）；其餘併進 mowerd 或由下面的元件取代 |
| `udev/99-mower.rules`（stmcom / imu_usb / gps_rtk / lte_at、Genesys hub `power/control=on`） | 原樣安裝到 `/etc/udev/rules.d`；另加 kernel cmdline `usbcore.autosuspend=-1` 一起試 hub reset |
| `mower-pair`（Python，`/etc/machine-id` 推 robot id）、`identity.json`、`r2.env`、`device_key` | `mowerctl pair`；**machine-id 必須跨 A/B 更新不變**：放資料分割（`systemd.machine_id` 指過去），不然每次換 slot 機器人 ID 就變 |
| `mower_flash.py` + firmware-sync | Rust `firmware-sync`（E6），`.bin` 與 manifest 隨 appfs |

### BSP 與 kernel

- 兩個都叫 `meta-rockchip` 的 layer：Rockchip 自家的（BSP kernel 6.1、`rockchip-mpp`、`gstreamer1.0-rockchip`、u-boot 與 Rockchip 分割配置）和 Yocto Project 託管的（偏 mainline，有 Rock 3A = RK3568 可以抄）。**相機決定用哪個**：`mower-camera.sh` 靠 `mppjpegdec` + `mpph264enc` 只吃 2 % 一核，mainline 的硬體 H.264 編碼不成熟，720p25 軟編會吃掉一整核。所以用 Rockchip BSP kernel + MPP。
- LubanCat-2 的 device tree 帶進 kernel recipe，UART3（40-pin pin 8 / 10，`rk356x-lubancat-uart3-m1`）直接寫進 DT，不再走 u-boot 讀 `uEnv.txt` 的 overlay。
- kernel config fragment：`ch341`（IMU）、`cp210x`（bench stmcom）、`cdc-acm`（u-blox）、`qmi_wwan` + `option`（EC25 / SIM7600）、`uvcvideo`（相機）、`tun`（Tailscale）、`dw_wdt`（硬體 watchdog）。
- cpufreq：明確設 governor 與上限。計畫第 1、2 節的量測被 1.4–2.0 GHz 跳動干擾過；OPP 是 408 / 600 / 816 / 1104 / 1416 / 1608 / 1800 / 1992 MHz。

### 系統元件

| 項目 | 選擇 | 理由 |
|---|---|---|
| init | systemd | 現在全是 unit / path / timer。`RuntimeWatchdogSec` 接 `dw_wdt`，mowerd 的 supervisor 做 `sd_notify`，模組 catch_unwind 之上再多一層 |
| 網路 | NetworkManager + ModemManager | 150 行 `mower-lte.sh`、udhcpc、resolvconf 由 MM 取代，4G route metric 用 `ipv4.route-metric` 設 200；訊號改走 mmcli / D-Bus，`/dev/lte_at` 不再自己開 |
| 時間 | systemd-timesyncd | RTC 沒電池；timesyncd 啟動就 step，還存上次時鐘，ntpsec 的 start-limit 問題消失。app 的 HMAC ±60 s 靠它 |
| 遠端 | openssh、Tailscale（抓官方 static tarball 或用 go 建，沒有官方 recipe） | Studio 的 SSH 隧道要 openssh 的 BatchMode 語意 |
| 影像 | mediamtx（抓官方 arm64 release，不要 ffmpeg 變體） | recorder 的 fMP4 錄影走它的 API 9997，照舊 |
| Rust | oe-core 的 `cargo` class，`cargo-update-recipe-crates` 產 crates.inc | 已經 `serialport default-features=false`（無 libudev）、全 rustls（無 OpenSSL），交叉編沒有坑 |
| Python | 零 | 現在散在五個地方（`mower_flash.py`、`mower-pair`、`mower-link-status.py`、兩個 shell 裡的 JSON 解析），E6 全收進 mowerd / mowerctl |
| journald | `Storage=persistent` + `SyncIntervalSec` 縮短 | 2026-10-08 硬重開讓前一個 boot 的 journal 整個截斷（`.journal~`），事後查不到死因 |

### 更新（取代 docker pull）

- RAUC，A/B rootfs 加一個獨立的 `appfs` slot（`/opt/mower`：mowerd、韌體 `.bin` + manifest、設定）。u-boot 用 boot count 自動回退。
- 兩層節奏：rootfs 很少動、手動建；**appfs 由 CI 每次 push 產出**，約 20 MB、幾分鐘。這是 CI 能在 GitHub-hosted runner 上做的那一層。
- `update_status.json` 的狀態機（pulling → restarting → idle / failed）對到 RAUC 的 install 進度，app 與燈效不用改；`robot_status.json` 的 busy gate 只擋**重開**，安裝寫非作用 slot，割草中也能裝。
- 2026-10-08 證實的「pull 負載會讓板子硬重開」在這裡沒有消失：RAUC 寫 eMMC 一樣是負載。`mower-update.sh` 的 CPU cap（PR #39）要帶過來，而且電源軌沒量清楚之前自動更新一律關。
- 唯讀 rootfs、`/etc` overlay、`/var/lib/mower` 資料分割（= 現在的 `~/.mower`：zones、sites、bags、identity、r2.env、calib）。bag 加影片每小時 1 GB 以上，不能和 rootfs 共用。

### CI

GitHub-hosted runner 14 GB 磁碟建不起整個 Yocto；rootfs 在本機或自備 sstate 的 builder 建、只在 tag 時做。CI 交叉編 `mowerd` 與 appfs bundle，跑 `cargo test`、`probe_app.py`、shadow 工具。repo 是公開的，self-hosted runner 不能跑 PR（`build.yml` 已有這條理由）。

### 驗收清單（上機）

開機到 `/robot/online` 幾秒內；`/dev/stmcom` 是 UART3、韌體 sync 成功；IMU / GPS / 相機 / 4G 四個 USB 裝置重插後 udev 名字回來、驅動自己重啟；相機管線走 MPP 且 < 5 % 一核；LTE 掉線由 MM 重撥；RAUC 切 slot 與 boot count 回退各一次；拔 watchdog 餵狗後硬體 watchdog 重開；`probe_app.py` 對 baseline `APP_CONTRACT_OK`；cpufreq 固定；journal 在硬重開後仍可讀。

### 時程與風險

一個人從 BSP 到相機、LTE、GPS、RAUC 全通約 2–3 週，排在 E7 之後。最大的風險不是 Yocto 本身：
(1) 5 V / USB 電源軌，2026-10-07/08 共 22 次硬重開沒有一次正常關機，Yocto 不會修它，`mower_pcb` 的 SOC 板（獨立 5 V buck + 3V3_4G buck）才是解；
(2) MPP 相機綁死 BSP kernel，之後 kernel 升級要跟 Rockchip 走；
(3) 4G 模組與 USB hub 的掉線，換 OS 不會變好，只是 ModemManager 的重撥比自己寫的 shell 穩。

## 附錄：量測方法

- 每執行緒 CPU：機器上 `/tmp/thr.sh N`（對 `/proc/*/task/*/stat` 取 utime/stime 兩次差分，印出 pid tid user sys comm；再用 awk 按 pid 加總）。腳本本身會多出 ~7% 且拉長視窗約 1.27 倍，絕對值以 `top -b -n 2 -d 10` 為準，分佈用腳本。
- loopback 封包率：`/proc/net/dev` 的 `lo` rx packets 差分。
- **不要**在機器上跑 `ros2 topic hz` / `node list`，會造成 controller_manager overrun 與 EKF 更新率警告；探針走 /robot/telemetry 或 ws_bridge。
- 每個 phase 的驗收狀態固定為：待機、nav2 已啟動、2 個 app 連線、60 s 視窗、記錄溫度與頻率。
