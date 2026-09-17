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

## `/robot/telemetry`（std_msgs/String，JSON，10 Hz）

給桌面參數儀表板（`mower_sudio_app`）的唯讀資料流，一個 topic 包含儀表板要顯示的全部：`gps`（`/fix` 狀態 / 位置 / 精度，有 u-blox `navpvt` 時附 `pvt` 衛星數與 RTK carrier solution）、`imu`（roll / pitch / yaw、角速度、加速度、更新率）、`odom`、`base`（`/mower_base/telemetry`：左右輪目標 / 實測 RPM、PID 輸出與增益、燈光模式、電源狀態，來自 STM32 的 `0x81/0x83/0x84/0x85/0x86`）、`link`（LTE `AT+CSQ` RSSI、Wi-Fi RSSI、介面狀態，由 `deploy/host/mower-link-status.py` 寫入 `link_status.json`）、`host`（load / 記憶體 / CPU 溫度）、`info`（最新的 `/robot/info`）。每一塊都有 `valid` 與 `age_s`，欄位說明見 `src/mower_mission/mower_mission/telemetry_node.py`。

## `/system/update`、`/system/restart`（std_srvs/Trigger）

機器人在移動或 `/nav_operation_active` 為 true 時回 `success=false`。成功只代表「已交給 host」，進度看 `/robot/info` 的 `update`。
