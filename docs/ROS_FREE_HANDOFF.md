# ROS-free 工作交接

寫於 2026-09-28。給接手的人（或換模型後的下一個 session）看的現況快照。
計畫本身在 [ROS_FREE_PLAN.md](ROS_FREE_PLAN.md)，這份只講「做到哪、接下來做什麼、怎麼做」。

---

## 一句話現況

**ROS 2 還在，沒有拿掉。** 計畫的 Phase A（減少程序數）已經在真機上量過並還原；
Phase B（Rust 底盤驅動）和 Phase C（Rust EKF/navsat）程式寫完、容器裡測過，
**從來沒在真機上跑過**。真正拿掉 ROS 是 Phase D–E，還沒開始。

## 分支與 PR

| | |
|---|---|
| 工作目錄 | `~/work/mpp-port`（worktree，branch `feat/ros-free`） |
| PR | [#20](https://github.com/edwards414/mower_path_planning/pull/20)，OPEN、mergeable、28 commits |
| CI | **一次都沒跑過**。GitHub 回 `recent account payments have failed or your spending limit needs to be increased`，所有 job 直接 skip。要先處理帳務，否則沒有 CI 映像 |
| main | 完全沒動 |

`docs/ROS_FREE_PLAN.md` 只存在於 `feat/ros-free` 及其子分支。
`~/Desktop/project/mower/mower_path_planning` 那個 checkout 在別的分支上，沒有這份計畫。

## 已完成

### Phase A — 少一點程序（已在真機驗證，已還原）

| commit | 內容 | 開關 |
|---|---|---|
| `0acff53` | nav2 的 8 個 server 併進一個 `component_container_isolated`，開 intra-process；local costmap 5→2 Hz、2→1 Hz；CI smoke test 檢查 component 函式庫沒被映像瘦身砍掉 | `nav_composition` / `NAV_COMPOSITION`，**預設 true** |
| `4ec15a8` `b9fc3cc` `6795313` `ef4bcf2` | 每個 mower_rs crate 改成 library + 薄 binary；`mower_rsd` 用一個 r2r Context（＝一個 DDS participant）跑任意模組子集 | `rust_daemon` / `RUST_DAEMON`，預設 false |

參數走單一 `--params-file`（`mower_bringup/config/mower_rsd.yaml`，按節點名分段）；
remap 走 rcl 的 node-scoped `-r <node>:<from>:=<to>`；**絕不可傳裸的 `-r __node:=`**，會改名整個程序裡的每個節點。

### Phase B — Rust 底盤驅動（未上機）

`ef3eaf5` 是 ROS-free 核心 `mower_base_core`：STM32 協定、diff_drive_controller 4.42.1 +
control_toolbox 4.11.1 的移植（正好是映像裡 apt 版本）、`BaseCycle` 狀態機、串口錄放格式。
後續 7 個 commit（到 `bddd29a`）是 `mower_base` 模組，取代整條 ros2_control 鏈
（controller_manager、mower_hardware、diff_drive_controller、joint_state_broadcaster、兩個 spawner）。
開關 `rust_base` / `RUST_BASE`。

容器對照結果：6 個 topic 的端點與 QoS 完全相同；`/odom`、`/joint_states` 都 25.00 Hz；
跑 3 m 後里程計差 1.9 mm；0x01 命令幀中位數差 0–1‰；telemetry JSON 84 個 key 一致。
CPU 19.2 % → 6.5 %（容器，debug build）。

兩個踩到的事實，寫在這裡免得再查一次：
真機實際載入的控制器設定是 `mower_controller/controllers/diff_drive_controller.yaml`，
**不是** `mower_hardware/config/mower_controllers.yaml`，兩份在 `open_loop`、`enable_odom_tf`、
幾何、`cmd_vel_timeout` 上不一致；`robot_state_publisher` **不在**取代範圍（原計畫寫錯，已改）。

### Phase C — Rust 定位（未上機）

`4118bd8` 是核心 `mower_localize_core`：robot_localization 3.8.3 的 EKF、ros_filter 前處理、
navsat、Krüger UTM，對 `librl_lib.so` 的 oracle 誤差 1e-16。
後續 5 個 commit（到 `cd6b979`）是 `mower_localize` 模組，一個模組裡三個 r2r 節點
（`ekf_filter_node_odom`、`ekf_filter_node_map`、`navsat_transform`，名字照舊），
取代兩個 ekf_node 和 navsat_transform_node。開關 `rust_localize` / `RUST_LOCALIZE`。

容器對照：`/toLL` 差 7e-15、`/fromLL` 4.7e-10 m、`map→utm` 4.7e-10。
濾波器輸出 `/odometry/global` 差 2.3e-2 m、`map→odom` 0.135 m —— 看起來大，但
**兩次 C++ 自己跑的差距一樣大**（2.28e-2、0.140），因為 `periodicUpdate` 是 wall-clock 驅動，
即時跑本來就不可重現；數學本身已由 oracle 釘在 1e-15。
CPU 8.2 %（3 程序 48 執行緒）→ 4.4 %（1 程序 21 執行緒）。

### 跑起來才發現的三個 bug（都已修）

| commit | 問題 |
|---|---|
| `204522c` | nav2 launch 沿用 `PythonExpression(['not ', use_composition])`，傳字串 `'true'` 會 `NameError`，整個 launch 死掉。改 `UnlessCondition` |
| `d9d4233` | `mower_rsd` 原本任一模組失敗就整個程序退出；沒接感測器時變成 13 個節點每幾秒全部重建。改成只重啟該模組、2 秒後、同一個 DDS participant |
| `32ba2aa` | B+C 合併後 `package.xml` 有兩個 `tf2_msgs`，rosdep 拒絕，映像建置在 `rosdep install` 失敗 |

### 驗證工具（`6651331`，在 `src/mower_rs/tools/`）

| 工具 | 用途 |
|---|---|
| `probe_app.py` | **證明 app 沒壞**。用 app 的 HMAC 連 bridge，訂閱 app 會訂的全部 topic、呼叫唯讀服務，跟 `app_contract_baseline.json` 比對，少任何一個就 FAIL |
| `switch.sh` / `measure.sh` | 真機 A/B：切換 `IMAGE_TAG` / `NAV_COMPOSITION` / `RUST_DAEMON` / 任意 `KEY=VAL` 並還原；量每程序 CPU、loopback 封包率、DDS 執行緒 |
| `shadow_compare.py`、`fake_base.py` + `base_harness.py` + `base_compare.py`、`localize_compare.py` | A5 / B / C 各自的差分測試 |

用法在 `src/mower_rs/tools/README.md`。

## 真機量到的數字（2026-09-23，感測器全接、靜止、測完已還原）

| | main | + composition | + RUST_DAEMON |
|---|---|---|---|
| nav2 | 8 程序 ~20 % | 1 程序 12.3 % | 1 程序 11.8 % |
| Rust 節點 | 13 程序 ~34 % | ~34 % | 1 程序 24.8 %、28 執行緒 |
| loopback 封包/s | 1329 | 838 | 581 |
| DDS tev / recv | 6.4 / 3.6 | 5.0 / 5.0 | 1.6 / 0.6 |

composition 省約 8 %、mower_rsd 省約 9 %，合計約 17 %，機器待機從 152 % 降到約 120 % 單核。

**不要看整機 idle 百分比**：RK3568 的頻率在 1.4–2.0 GHz 之間跳，idle 三組都是 68–70 %，看不出差別。
要看每程序 CPU 和封包率。

另外有一組「沒接感測器」的數字（16k 封包/s、DDS 執行緒 165 %）**不可拿來比較** ——
那是 IMU/GPS 驅動每 2 秒 respawn 造成的 discovery 風暴。附帶證明了 `mower_rsd`
的程序內重啟能治這個（同條件只有 720 封包/s）。

## 還沒做的（依優先序）

### 1. `RUST_BASE` / `RUST_LOCALIZE` 上機 —— 最優先，且必須有人在場

兩個都是安全關鍵：`mower_base` 直接驅動輪子，nav2 靠 `mower_localize` 發的 `map→odom` 導航。
第一次開啟一定要監督試車：搖桿、放開即停、一段導航；底盤另外要拔串口線確認 fail-closed + 2 秒重啟。

真機基準是 ros2_control 28 %、定位鏈 23 %，容器裡分別降到 6.5 % 和 4.4 %，
真機能省多少還沒量。

流程（工具 README 有完整指令）：停 `mower-update.timer` → `docker load` 本機建的映像
→ `switch.sh` → 等 90–100 秒 → `measure.sh` → `probe_app.py --baseline` → `switch.sh restore`。

**GPS 導航那一段之前先跑 `datum_check.py`**（`src/mower_rs/tools/`，預設唯讀，ssh 指令在檔頭）。
開 `RUST_BASE` 之後 `/odom` 會比 IMU 先到 map EKF，EKF 的 yaw 從 0 開始、3 秒 delay 結束時還沒收斂，
navsat 鎖的 datum 就歪 10–15 度 —— **C++ 和 Rust 一樣**（容器實測 C++ 14.9°、Rust 15.2°），不是移植的 bug，
但整段 GPS 軌跡會繞原點轉那個角度，車一直偏離條帶。印出 `DATUM_ROTATED` 就加 `--fix`
（`/datum` 設在同一個原點、只拿掉旋轉；容器裡兩個 stack 都修到 0.00°），然後重啟 adapter
（兩個 adapter 都只鎖一次 `/adapter/map_datum`，app 衛星底圖的方位不會自己更新），再載場地 / 開任務。

根治還沒做，要決定：map EKF 加 `initial_estimate_covariance`、yaw（第 6 個）給 1.0、其餘 1e-9，
第一筆 IMU yaw 就會把航向拉到位。容器同一個啟動情境 datum 誤差從 14.9–15.5° 降到 C++ 0.37°、Rust 0.34°，
兩邊讀同一份 yaml 所以仍然一致；但它會改到現在 C++ 路徑的行為，而且 Rust 對這個 key 印
「oracle 沒涵蓋」警告。映像含 `fix/rf-localize-preflight` 之後不用重建就能試：複製 `dual_ekf_navsat_params.yaml` 到 `~/.mower`，
compose command 加 `localize_params_file:=/home/mower/.mower/<檔名>`（兩個 stack 都吃這個參數）。

### 2. Phase D–E 的 Go/No-Go（計畫第 7 節）

**光為了 CPU 不值得做。** A–C 之後整機約 15 %，D–E 再多省約 10 個百分點，
代價是三到四週和一套自己維護的導航堆疊。要四個條件中至少成立兩個：
功能集定型、真的需要 ROS-free 映像、散熱裕度、要上更便宜的 SoC。

### 3. 其他還能省的

- `mower_rsd` 剩下的約 25 % 還沒逐執行緒 profile（懷疑是 bridge 的 JSON、20 Hz 心跳、guard timer）
- 純水管約 11 %（兩個 throttle、robot_state_publisher、twist_mux），Phase C 的模組可以直接發慢速 topic 取代 throttle
- app 離線重連沒有明顯退避：機器離線 3 天半累積約 5 萬行重連日誌，平均每 12 秒一次

## 現在的環境狀態（2026-09-28 23:00）

- **機器**：`cat@192.168.0.114`，開機 2 分鐘，跑 `main` 映像，`ROSBRIDGE_ADDRESS=192.168.0.114`（正確），
  `mower-update.timer` active，沒有殘留的 rosfree 測試映像。9/23 的還原是乾淨的。
- **Mac**：192.168.0.111。
- **本機映像**：`mower_path_planning:ros-free-test`（2.04 GB，從 `32ba2aa` 建的，含
  `mower_base` / `mower_localize` / `mower_rsd`）還在 docker 裡，但匯出的 tarball 已隨 session 暫存目錄消失，
  要用 `docker save ... | gzip -1` 重新匯出。
- **worktree**：`~/work/rf-{a3-nav2-compose,a5-mower-rsd,b-base-core,b-base-node,c-localize-core,c-localize-node}`
  都已合併進 `feat/ros-free`，確認後可以 `git worktree remove`。

## 陷阱清單

1. **機器 IP 會跳**（.113 → .109 → .114），而且 **Mac 可能搶走機器的舊 IP**。ssh 連不上先看 `ipconfig getifaddr en0`。
   `known_hosts` 有別台裝置的舊 key 時用 `ssh -o HostKeyAlias=<舊IP>`，不用改檔案。
2. **`ROSBRIDGE_ADDRESS` 沒跟著 IP 改，bridge 只綁 loopback**，app 走區網完全連不上，而且不會有錯誤訊息指向這裡。
3. **auto mode 分類器**擋掉所有對機器的寫入（scp、`docker load`、`compose up`、改 `.env`），理由 `Production Deploy`；
   唯讀 ssh 可以。改設定檔一律擋（`Self-Modification`），使用者口頭同意也不算，要他自己 `/permissions`。
   已加的規則 `Bash(ssh cat@192.168.0.114 *)` / `Bash(scp * cat@192.168.0.114:*)` **只對「開頭就是 ssh/scp」的指令生效**，
   複合指令（`cp a b && scp ...`）還是會被擋。
4. **熱修補活不過一小時**：`mower-update.timer` 會拉新映像重建容器，曾經 12 分鐘就被換掉。測試前先停 timer。
5. **本機建映像要檢查結果**：build 可能在 `rosdep install`（log 很前面的 `#21 ERROR`）失敗，
   但後面的 `docker save` 步驟仍然成功匯出「上一版」映像。建完 `ls /mower_ws/install/mower_rs/lib/mower_rs/` 確認。
6. **不要在機器上跑 `ros2 topic hz` / `ros2 node list`**，會造成 controller_manager overrun 和 EKF 更新率警告；
   要探測走 `/robot/telemetry` 或 bridge。
7. **子代理會撞用量上限**（發生過四次）。用 `SendMessage` 接回去時，訊息裡要明確寫「已經 commit 了什麼、剩下什麼」，
   否則它會重做已完成的工作。
