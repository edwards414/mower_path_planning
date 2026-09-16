# 機器人部署（LubanCat 2）

一個 image tag = 一個完整的機器人版本：ROS 軟體 + 同一個 commit 編出來的 STM32 韌體（容器啟動時 `firmware-sync` 自動燒）。機器人只做 **pull**，不需要 GitHub 憑證以外的任何東西。

```
GitHub main / tag v* ──build.yml──► ghcr.io/edwards414/mower_path_planning:{main,v0.6.0,stable}
                                                     │
LubanCat  sudo /opt/mower/host/mower-update.sh ◄─────┘   (或 App「更新機器人」→ /system/update)
          └─ docker compose up -d ─► 容器 entrypoint: firmware-sync ─► ros2 launch robot.launch.py
```

## 第一次安裝

```bash
# 在板子上（deploy/ 這個資料夾即可，不需要整個 repo）
sudo ./install.sh                      # /opt/mower + udev + systemd + ~/.mower
sudo docker login ghcr.io              # image 是 private 時；用只有 read:packages 的 PAT
sudo vi /opt/mower/.env                # IMAGE_TAG、ROSBRIDGE_ADDRESS
sudo /opt/mower/host/mower-update.sh   # pull + 啟動
sudo systemctl status mower
sudo docker compose -f /opt/mower/docker-compose.yaml logs -f lawan_node
```

`install.sh` 可重複執行（更新 compose / 腳本 / udev / 單元）。

## 更新 / 回滾

| 動作 | 指令 |
|---|---|
| 跟著頻道更新 | `sudo /opt/mower/host/mower-update.sh` |
| 換頻道或釘版本 | `sudo /opt/mower/host/mower-update.sh --tag v0.6.0`（`stable` / `main` / `vX.Y.Z`） |
| 回滾 | 同上，指到上一版 tag；韌體跟著 image 一起回去 |
| 從 App | 設定頁「更新機器人」→ `/system/update`（移動中或導航中會拒絕） |
| **自動** | `mower-update.timer` 開機 10 分鐘後、之後每小時檢查一次頻道；機器人移動 / 導航中會跳過。`.env` 設 `MOWER_AUTO_UPDATE=0` 或 `systemctl disable --now mower-update.timer` 關閉 |

進度在 `~/.mower/update_status.json`，App 透過 `/robot/info` 看得到（`pulling` → `restarting` → `idle` / `failed`）。更新進行中（`pulling` / `restarting`）機身燈條會跑**琥珀色環繞光**（`robot_info_node` → `/mower_base/led_command` → STM32 mode `0x06`），結束後淡回常亮白。

## 韌體

- image 內附 `/opt/mower/firmware/mower_robot_firmware.bin` + `.json`（版本、commit、build 時間）。
- 容器每次啟動先跑 `firmware-sync`：讀 STM32 的 `0x87` 身分，不同才透過 bootloader 燒（約 20 s，馬達停止）；結果在 `~/.mower/firmware_sync.json`。
- 手動 / 除錯：`MOWER_FIRMWARE_SYNC=0` 在 `.env` 關掉；板子上直接燒用 `firmware/tools/mower_flash.py -p /dev/stmcom flash …`（先 `sudo systemctl stop mower`）。
- bootloader 本身（`firmware/bootloader/`）不走這條路，要 ST-Link。

## Host 端單元（`host/`）

| 單元 | 作用 |
|---|---|
| `mower.service` | 開機 `docker compose up -d`（不 pull，避免沒網路時卡開機） |
| `mower-host-request.path` + `.service` | 監看 `~/.mower/host.request`；容器寫 `update` / `restart` / `reboot` / `poweroff` 進去，host 執行。STM32 電源鍵長按 3 s → 驅動寫 `poweroff` → `systemctl poweroff` |
| `mower-update.sh` | pull + 比對 digest + `up -d`，寫 `update_status.json`、`image.json`；`--auto` 給 timer 用（閒置才更新） |
| `mower-update.timer` + `.service` | 每小時 `mower-update.sh --auto` |

## 裝置名（`udev/99-mower.rules`）

| 名稱 | 硬體 |
|---|---|
| `/dev/stmcom` | STM32 host UART：LubanCat `ttyS3`（UART3 overlay）或 CP2102 |
| `/dev/imu_usb` | WIT IMU（CH340） |
| `/dev/gps_rtk` | u-blox ZED-F9P（`--profile gps` 服務，設定在 `gps/ublox.yaml`，尚未在機器上驗證） |

## 遠端存取

`server/` 是公網 relay（cloudflared + WireGuard），與本頁獨立；`ROSBRIDGE_ADDRESS` 預設 `10.77.0.2` 是機器人的 WireGuard 位址，桌上測試改成 LAN IP。
