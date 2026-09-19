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
| `/dev/gps_rtk` | u-blox ZED-F9P（CDC-ACM） |

三個裝置都寫在 compose 的 `devices:`，少一個容器就起不來（`install.sh` 最後會列出哪個不在）。

## GPS

u-blox 驅動是 `mower_rs` 的 `mower_gps`（Rust），跟其他節點一樣跑在主容器裡（`mower.launch.py enable_gps:=true`），發布 `/fix` 與 1 Hz 的 `/gps/status`（JSON：`fix_type`、`carrier_solution` none/float/fixed、`num_sv`、`h_acc_m`、`pdop`、`utc`）。
以前是獨立的 `--profile gps` 服務 + `-gps` image 跑 ROS 的 `ublox_gps`，已移除：那個驅動在 USB 斷線後不會退出（無限重讀、吃滿一核、佔著死掉的 tty，重新枚舉的接收器變成別的 minor，容器內靜態的 `/dev/gps_rtk` 就指不到），而這塊板子的 USB hub 每小時會 reset 幾次。
`mower_gps` 遇到序列埠錯誤或 NAV-PVT 停 5 s 就結束、launch 2 s 後 respawn（跟 IMU 驅動一樣），中間導航健康檢查會擋住。`.env` 設 `GPS=false` 可關掉（沒有 `/fix`，導航被健康檢查擋住）。

參數在 image 內的 `src/mower_bringup/config/gps.yaml`（4 Hz、`frame_id: gps_link`、逾時）。驅動每次啟動只在接收器 RAM 開 USB 口的 NAV-PVT + NAV-HPPOSLLH + NAV-EOE，不存 flash、不動其他設定（含 UART1 的 RTCM 改正輸入 / NTRIP，那要另外設）。
要試別的參數不用重建 image：把 yaml 放進 `~/.mower/`，compose 的 command 加 `gps_params_file:=/home/mower/.mower/<檔名>`。

2026-09-19 在機器上驗過：4.00 Hz、室內無星 → `status: -1` / 位置 NaN；模擬拔線（`echo 0 > /sys/bus/usb/devices/1-1.3/authorized`）0.5 s 內退出，重插後回到 `ttyACM0` 重啟正常。RTK 精度（協方差對角線開根號 < 0.015 m 才會開導航閘門）要到戶外接上改正源才驗得到。

驗證：

```bash
sudo docker compose -f /opt/mower/docker-compose.yaml exec lawan_node bash -lc \
  'source /opt/ros/jazzy/setup.bash; ros2 topic hz /fix; ros2 topic echo --once /gps/status'
```

## 遠端存取

前鏡頭：主機端 `mower-camera.service`（`host/mower-camera.sh`）用 GStreamer 從 `/dev/video0` 抓 MJPG，
交給 Rockchip VPU 硬體解 JPEG、硬體編 H.264（約佔一核的 2%），以 RTSP 發布進 mediamtx 容器，App 再以 WebRTC/WHEP 讀。
參數在 `.env` 的 `CAMERA_*`。相機在暗處會因自動曝光掉到約 5 fps，戶外正常。

影像：同一個 Wi-Fi 時 App 直接拉 `http://<robot-ip>:8889/front/whep`；跨網路時 WHEP 訊令經 `../backend/` 轉到機器人上的
mediamtx、媒體走 Cloudflare TURN（agent 每小時從後台拿帳密寫進 mediamtx API），見 `docs/BACKEND_ARCHITECTURE.md` §8。
`server/` 是舊的公網 relay（cloudflared + WireGuard，只對一台機器人），已不需要；控制連線由 `../backend/`
（Cloudflare Worker，`MOWER_BACKEND_URL` / `MOWER_PROVISION_TOKEN` 在 `.env`）中繼。`ROSBRIDGE_ADDRESS` 預設 `10.77.0.2` 是機器人的 WireGuard 位址，桌上測試改成 LAN IP。
