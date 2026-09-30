# Robot ⇄ App API 版本契約

三個東西各自有自己的 release 節奏：

| 元件 | 版本來源 | 曝露位置 |
|---|---|---|
| 機器人軟體（這個 repo 的 Docker image） | git tag `vX.Y.Z` → `MOWER_VERSION` build arg → image env | `/robot/info` → `software` |
| STM32 韌體（`firmware/`，同一個 tag、同一個 image 內附） | 同上，經 `firmware/Makefile` 寫進 `0x87` frame | `/robot/info` → `firmware.running` / `firmware.bundled` |
| iOS App（`mower_lawer_app`） | `pubspec.yaml` `version` + build number | App 設定頁 |

它們不需要同一秒更新；需要的是兩條介面都有版本號、且雙方都會檢查：

1. **STM32 ⇄ LubanCat**：UART `protocol_version`（`firmware/UART_OPEN_LOOP_PROTOCOL.md`）。驅動和韌體永遠來自同一個 image，容器啟動時 `firmware-sync` 會把 STM32 燒到 image 內附的版本，所以這條實際上不會不同步；驅動仍會在 `protocol_version` 不同時記 ERROR。
2. **LubanCat ⇄ App**：`api_version`（`src/mower_mission/mower_mission/version.py` 的 `ROBOT_API_VERSION`）。App 內建支援範圍；連上後讀 `/robot/info`，不在範圍就顯示「請更新 App / 機器人」並停用操作。

## 什麼算 API

透過 rosbridge 看得到的東西：`src/mower_bringup/config/rosbridge_params.yaml` 允許的 topics / services、`mower_interface` 的 msg / srv 版面、`/adapter/*` 的 JSON 格式、`/robot/*`、`/system/*`。

## 什麼時候要 bump `ROBOT_API_VERSION`

- 刪除或改名 App 會用到的 topic / service
- 改 srv / msg 欄位（新增欄位在 rosbridge JSON 下通常相容，可以不 bump，但要記在下表）
- 改變既有欄位的語意（單位、座標系、狀態字串）

新增 topic / service 不用 bump；App 用「有沒有」來判斷。

## 版本歷史

| api_version | 機器人版本 | 變更 |
|---|---|---|
| 1 | 0.1.0 | 首版：`/robot/info`、`/system/update`、`/system/restart` 加入；App 開始檢查 `api_version`。之前的 API（`/adapter/*`、zone / site / coverage services、`/robot/online`）視為 1。 |
| 2 | 0.2.0 | **配對**：機器人的 WebSocket 入口改為 `rosbridge_auth_proxy`，每次連線要帶配對 HMAC 標頭（見下方「配對」）；`/robot/info` 多 `name`、`pairing_required`。沒配對的 App 連不上（HTTP 401）。 |

## 配對（pairing）

每台機器人安裝時由 `deploy/host/mower-pair` 產生 `~/.mower/identity.json`：`robot_id`（由 machine-id 導出，如 `MW-7K3Q9P`）、`name`、`secret`（160-bit，base32）。`sudo mower-pair` 印出 QR：

```
https://mower.fxrbindi.com/pair?v=1&id=MW-7K3Q9P&s=<secret>&n=<name>&h=<relay wss url>&l=<lan ip>
```

要貼在車身上的版本：在工作機 `ssh cat@<robot> mower-pair --json --no-qr > pair.json && python3 deploy/mower-pair-sheet.py pair.json --pdf mower_pair_qr.pdf`（A4 四張 80×100 mm 貼紙、EC level H；需 `pip install segno`，用 Chrome 轉 PDF）。換過 secret 要重印。

`h` 是 relay 入口：接上後台時是 `wss://api.mower.fxrbindi.com/v1/relay/app`（App 自己補上 `/<robot_id>`），
舊式固定 tunnel（`wss://control.fxrbindi.com`）也仍接受。後台、註冊、心跳與 relay 協定見 `docs/BACKEND_ARCHITECTURE.md`；
機器人端由 `mower_agent`（`rosbridge.launch.py` 帶起，`MOWER_BACKEND_URL` 空白時閒置）負責。

App 掃碼後把這台存進「我的機器人」，之後**每次 WebSocket 連線**在 HTTP upgrade 帶：

| Header | 值 |
|---|---|
| `X-Mower-Robot` | `robot_id` |
| `X-Mower-Client` | 這支手機 / App 安裝的固定 id |
| `X-Mower-Time` | unix 秒 |
| `X-Mower-Nonce` | 每次連線新的 16–32 bytes hex |
| `X-Mower-Mac` | `hex(HMAC-SHA256(base32decode(secret), "robot_id\nclient\ntime\nnonce"))` |

機器人端 `rosbridge_auth_proxy`（`src/mower_mission/mower_mission/rosbridge_auth_proxy.py`）在 `rosbridge_address:9090` 驗證（時間差 ±60 s、nonce 不可重放、`robot_id` 要對），通過才把 frame 轉給只聽 loopback 的 rosbridge（`127.0.0.1:9091`）。沒有 `identity.json`（模擬、開發）時 proxy 直接放行。App 連上後再比對 `/robot/info.robot_id` 等於掃到的那台。撤銷：`sudo mower-pair --rotate` 換 secret，所有手機重新掃碼。

測試向量（Python `test/test_pairing.py` 與 App `test/pairing_test.dart` 共用）：secret 全 0（`AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA`）、robot `MW-7K3Q9P`、client `iphone-1234`、time `1789600000`、nonce `00112233445566778899aabbccddeeff` → mac `d33d2137cf8c6bb75ba80ff22b9afbf32a09f2346e83617e41e96026aa2cfd42`。

## `/robot/info`（std_msgs/String，JSON，latched + 1 Hz）

```json
{
  "robot_id": "lubancat",
  "api_version": 1,
  "software": {"version": "0.6.0", "git_sha": "…", "build_unix": 1789571662,
               "image": "ghcr.io/edwards414/mower_path_planning:stable",
               "digest": "sha256:…", "tag": "stable", "pulled_at": 1789571700},
  "firmware": {
    "running": {"version": "0.6.0", "semver": [0,6,0], "protocol_version": 1,
                "git_sha": "f22f2465", "git_sha32": 4063175781, "build_unix": 1789571662,
                "dirty": false, "unversioned": false},
    "bundled": {"version": "0.6.0", "semver": [0,6,0], "git_sha": "f22f2465…", "build_unix": 1789571662,
                "dirty": false, "size": 41316, "crc32": "c701e68a"},
    "sync": {"action": "up_to_date", "time": 1789571712, "error": null},
    "up_to_date": true
  },
  "update": {"state": "idle", "message": "", "time": 1789571700},
  "busy": false,
  "uptime_s": 123.4
}
```

`update.state`：`idle` / `pulling` / `restarting` / `up_to_date` / `failed`（由 `deploy/host/mower-host-request.sh` 寫入）。

`update` 另外帶最近一次 registry 查詢（`mower-update-check.timer` 每 5 分鐘、或 `/system/check_update`，都只查 manifest 不下載）：`remote_digest`（頻道目前的 digest）、`checked_at`（unix 秒）、`check_error`（查不到時的原因，否則 null）、`available`（`remote_digest` 與 `software.digest` 不同 = 有新版可更新；`/system/update` 之後自動變回 false）。還沒查過時這四個欄位不存在。

## `/robot/telemetry`（std_msgs/String，JSON，10 Hz）

> `base` 來源 `/mower_base/telemetry` 本身是 20 Hz（與 STM32 `0x85` 同步，PID 自動校正需要這個取樣率），並多了 `t`（ROS time）與 `pid.last_rx_seq`。

給桌面參數儀表板（`mower_sudio_app`）的唯讀資料流，一個 topic 包含儀表板要顯示的全部：`gps`（`/fix` 狀態 / 位置 / 精度，有 u-blox `navpvt` 時附 `pvt` 衛星數與 RTK carrier solution）、`imu`（roll / pitch / yaw、角速度、加速度、更新率）、`odom`、`base`（`/mower_base/telemetry`：左右輪目標 / 實測 RPM、PID 輸出與增益、燈光模式、電源狀態、充電模組 `charger`、抬刀 servo `servo`（脈寬、上下限微動、被限位擋下）、割草馬達 `blade`，來自 STM32 的 `0x81/0x82/0x83/0x84/0x85/0x86/0x88/0x89`）、`battery`（`/battery_state` 攤平：`present / pct / voltage_v / current_a / status`，`aon` 下是小電池同格式）、`link`（LTE `AT+CSQ` RSSI、Wi-Fi RSSI、介面狀態，由 `deploy/host/mower-link-status.py` 寫入 `link_status.json`）、`host`（像 `top` / `free` / `df`：每 2 s 更新的 CPU 總使用率與 us/sy/wa、每核 %、時脈與降頻上限、各溫度區、load 1/5/15、記憶體、磁碟容量與讀寫速率、eMMC 壽命、CPU 前 10 名程序；程序的 % 是單核百分比，4 核滿載 400 %；速率欄位在第一筆是 null；Rust 版在 `robot_status/src/host.rs`，Python 版在 `mower_mission/host_stats.py`）、`info`（最新的 `/robot/info`）。每一塊都有 `valid` 與 `age_s`，欄位說明見 `src/mower_mission/mower_mission/telemetry_node.py`。

## `/battery_state`（sensor_msgs/BatteryState，1 Hz）

真機上由 `battery_state` 節點發（預設 `mower_mission` `battery_state_node`，`rust_battery:=true` 時為 mower_rs `mower_battery`；`src/mower_mission/launch/mission.launch.py`），資料來源是 STM32 `0x89` 的 RS485 電池電表電壓（裝在電池側；`0x8A` 已退役），充電器在不在以電壓 ≥ 25.0 V（`charger_present_min_v`）判斷，SOC 是 6S 鋰電 OCV 查表加濾波（設計與限制見 `docs/BATTERY.md`）。模擬環境仍由 `battery_simulator_node` 發同一個 topic。

| 欄位 | 內容 |
|---|---|
| `present` | STM32 5 s 內有回報有效電壓才 `true`；`false` 時其他欄位是 NaN / UNKNOWN，App 應顯示「--」 |
| `voltage` | 濾波後的電池組電壓（V） |
| `percentage` | `0.0 ~ 1.0`；負載下誤差約 ±10 %，靜置較準 |
| `current` | 電表電流端子未接（`meter_current_wired=false`，預設）時一律 NaN；接上後為電池組電流（A），充電正、放電負 |
| `power_supply_status` | `CHARGING` / `FULL` / `NOT_CHARGING`（接著充電器但沒電流）/ `DISCHARGING` |
| `cell_voltage` | 6 個平均值（沒有逐 cell 量測） |

`/aon_battery_state` 同格式，是維持 STM32 常開的 3.7 V 小電池；目前沒有量測來源（`0x8A` 已退役），`present` 永遠 false，App 不顯示。

## `/pid_autotune`（mower_interface/srv/PidAutotune）、`/pid_autotune/status`（std_msgs/String，JSON，latched）

輪速 PID 自動校正（`src/mower_mission/mower_mission/pid_autotune_node.py`），給 Mower Studio 的「PID 自動校正」按鈕用。
新增於 api 2 之後，App 用「service 有沒有」判斷。**校正時左右輪會以最高 70 % duty 空轉**：呼叫 `start` 之前 App 必須讓操作者確認車輛已架高、兩輪懸空、刀片停止；機器人端無法自行檢查這件事，只會拒絕「導航中 / 手動移動中 / 電源非 RUNNING / 沒有 base telemetry」（`0x81` 的 `DRIVER_ALARM` 是 BTS7960 `IS` 類比電流感測，馬達一有電流就會亮，只記錄不擋）。

Request `op`：

| op | 作用 |
|---|---|
| `start` | 開始。取 `/mission_operation_lock`、燈條琥珀色環繞、切開環 → `0 → 40 % → 70 %` duty 各 2.5 s（兩輪同時，取 `0x85` 的 `pid_output` / `measured_rpm`）→ 擬合一階加延遲模型 → SIMC PI → 用新增益閉環 step 到 50 % 速度驗證（超調 ≤ 25 %、±5 % 安定 ≤ 1.5 s、穩態誤差 ≤ 1.5 rpm）→ `review`。增益此時只在 RAM。 |
| `apply` | 只在 `review` 有效：把新增益寫進 STM32 Flash（`0x04 persist=1`），等 `0x84` 的 `FLASH_VALID + LAST_APPLY_OK`（sector erase 會讓 STM32 停約 1 s，最多等 4 s）→ `done`。沒確認就自動重送一次；兩次都失敗 → `failed`，`error` 帶 `flash_diag=0x....`（`0x84` 的 flash 診斷字，見韌體 `UART_OPEN_LOOP_PROTOCOL.md`）並還原原本的增益。 |
| `discard` | 只在 `review` 有效：還原原本的增益 → `idle`。`review` 超過 5 分鐘沒回應視同 `discard`。 |
| `abort` | 任何階段：停輪、還原原本增益、釋放 lock → `aborted`。 |

Response：`success` / `message`。`start` 回 `success=true` 只代表已開始，過程與結果看 status。

`/pid_autotune/status` JSON：

```
{"state": "idle|precheck|open_loop|fitting|verify|review|saving|done|failed|aborted",
 "progress": 0.0-1.0, "message": "...", "error": null|"...",
 "started_at": unix_s, "updated_at": unix_s,
 "old_gains": {"left": {"kp","ki","kd"}, "right": {...}} | null,
 "new_gains": {...} | null,
 "model":  {"left": {"gain" (rpm/count), "tau" (s), "delay" (s), "y0","y_ss","u0","u1","fit_r2","fit_rmse"}, "right": {...}} | null,
 "verify": {"left": {"target","overshoot_pct","settle_s","ss_error","rise_s"}, "right": {...}} | null,
 "samples": {"open_loop": {"left": [[t, pwm_counts, rpm], ...], "right": [...]},
             "verify":    {"left": [...], "right": [...]}},
 "marks": {"open_loop_low": t, "open_loop_high": t, "verify": t}}
```

`t` 是自 `started_at` 起的秒數。執行中 5 Hz 更新，結束後 latched 留著最後一筆；任何非 `done` / `idle` 的結束都會把原本的增益寫回 RAM（Flash 不動）。

機器人端依賴 mower_hardware 的兩個側通道：`/mower_base/pid_command`（`0x04`）與 `/mower_base/wheel_override`（繞過 `diff_drive_controller` 加速度限制的原始 permille 指令，ttl ≤ 1 s 要一直重送，沒送就回到控制器指令），見 `src/mower_hardware/README.md`。

## `/mower_base/servo_command`、`/mower_base/blade_command`（std_msgs/String，JSON，App → 機器人）

Mower Studio「抬刀 / 割草馬達」卡片用的兩個指令 topic，直接進 `mower_hardware`（見 `src/mower_hardware/README.md`）：

- `servo_command`：`{"pulse_us":1500,"hold_ms":0}`，`pulse_us` 500–2500 是目標，STM32 以 25 µs/10 ms 滑過去，碰到上下限微動就退 50 µs 停住（狀態在 `/robot/telemetry` 的 `base.servo`：`pulse_us / limit_up / limit_down / limit_active`）；`0` 放鬆。
- `blade_command`：`{"permille":300,"ttl_ms":500}`，**dead-man**：ttl（≤ 1 s）內沒再收到就停，App 要每 0.2 s 重送；`permille` 0 立即停。狀態在 `base.blade`。

新增 topic，`api_version` 不 bump；App 用 `base.servo.valid` 判斷機器人是否支援。

## `/system/update`、`/system/restart`（std_srvs/Trigger）

機器人在移動或 `/nav_operation_active` 為 true 時回 `success=false`。成功只代表「已交給 host」，進度看 `/robot/info` 的 `update`。

## `/system/check_update`（std_srvs/Trigger）

立刻做一次 registry 查詢（不下載、不重啟，移動中也可以），結果在幾秒後出現在 `/robot/info` 的 `update.available` / `update.remote_digest`。api 2 新增，App 用「有沒有這個 service」判斷。
