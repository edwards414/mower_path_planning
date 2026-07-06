# TF / GPS 融合定位評估報告

- 日期:2026-07-01
- 對象:割草機器人(ROS 2 Jazzy,跑於 Docker:`mower_bringup` / `ublox_ros2` / `zenoh_bridge`)
- 目的:評估「GPS 融合後在 tf 樹上的位置」是否會亂跳、距離是否相差太遠、能不能當作導航來源

---

## 一、結論(先看這段)

| 問題 | 結論 |
|---|---|
| tf 亂跳嗎? | **本體 tf 完全不跳**(20 秒 0 次跳動)。但**融合後的定位根本還沒上 tf 樹**,所以現在沒有「融合位置」可跳。 |
| 距離會不會相差太遠? | **會,而且很嚴重**。融合設定的 `datum` 設在高雄(22.62, 120.38),但機器人實際在雲林/嘉義(23.70, 120.54),**相距約 120.8 公里**。一旦啟動,map 座標會直接錯 120 公里。 |
| 能不能當導航來源? | **目前不行。** 兩個原因:(1) RTK 沒吃到修正,退化成單點定位(±1.2m);(2) 融合節點沒啟動,且設定有多個接線錯誤,啟動了也拿不到資料。 |

**一句話:tf 本身是健康的,但「GPS→融合→tf」這條鏈現在是斷的,而且設定裡有 3 個會致命的錯誤,必須先修才能用。**

---

## 二、tf 樹健康度(實測)

用自寫的跳動偵測腳本監測 20 秒,結果:

```
transform                         type    Hz     maxΔpos   maxΔang  jumps
base_link -> connector            dyn     16.4   0.0000m   0.0°     0
base_link -> left_wheel_link      dyn     16.4   0.0000m   0.0°     0
base_link -> right_wheel_link     dyn     16.4   0.0000m   0.0°     0
base_footprint -> base_link       static  -      0         0        0
base_link -> {gps_link, imu_link, cameras, blade_hub, ...}  static  全部 0 跳動
```

- 目前 tf 樹只有**機器人本體(URDF)**:`base_footprint → base_link → {輪子 / gps_link / imu_link / 相機 / blade_hub}`
- **樹上沒有 `map`、沒有 `odom`** → 沒有任何世界座標/定位資訊
- `/tf` 發佈者只有兩個:`robot_state_publisher`(本體)、`diff_controller`
- `diff_controller` 的 `enable_odom_tf = False` → **連 odom→base_link 都沒發**

> 判讀:本體 tf 穩定正常。但機器人現在「不知道自己在世界的哪裡」,tf 上沒有定位。

---

## 三、GPS 原始品質(實測 45 秒,靜止)

```
samples=45  rate=1.0 Hz  status=0  position_covariance[0]=1.51  (σ_水平 ≈ 1.23 m)
逐點跳動 (consecutive step): mean=0.038m  max=0.115m  p95=0.088m
離平均位置的漂移 (spread)  : max=0.780m  rms=0.390m
高度漂移                    : 78.13 ~ 79.64 m  (Δ 1.51 m)
```

判讀:
- **不是「每筆狂跳」**(相鄰點只差 ~4cm),但會**在 ±0.8m 範圍內慢慢漂**,更新率只有 **1 Hz**。
- 協方差 1.5 m² → **這是一般單點定位的精度,不是 RTK fixed**(RTK fixed 應為 ~0.0004 m²,σ≈2cm)。
- 用這種資料去做全域 EKF 校正,融合後的 map→base_link 會**每秒被 GPS 往回拉最多約 1 公尺**,在 tf 上就會看到週期性的 ~0.5–1m 跳動。

---

## 四、RTK 診斷 —— 有裝,但現在沒在運作

你有裝 RTK,但實測結果 **RTK 現在沒有生效**:

| 檢查項 | 結果 | 意義 |
|---|---|---|
| `/fix` 協方差 | 1.5 m² (σ≈1.2m) | 單點定位精度,**非 RTK fixed** |
| `/rxmrtcm`(RTCM 接收) | publisher=1,**8 秒內 0 筆** | **接收機根本沒收到 RTCM 修正** |
| `/rtcm` `/ntrip/rtcm` 等 | 不存在 | **沒有 NTRIP / 基站修正流餵進來** |
| `/navpvt` | publisher=1,**0 筆** | ublox 沒發 NAV-PVT,讀不到 carrSoln(載波解狀態) |

> 根因:**沒有 RTCM 修正資料流進 rover**。RTK 要達到公分級,rover 必須持續收到基站的 RTCM3 修正。目前這條流是斷的,所以晶片退回單點定位 ±1.2m。

### 4.1 根本原因(已定位)—— 模組/設定不匹配 + UART1 鮑率

修正來源為**本地基站經電台傳到 F9P 的 UART1 針腳**(硬體序列,不經 ROS)。從 ublox 啟動 log:

```
MOD=ZED-F9P   PROTVER=27.31                         ← 硬體是 ZED-F9P
啟動: ublox_gps_node --params-file c94_m8p_rover.yaml ← 卻套 C94-M8P 設定
[ERROR] received NACK: 0x06 / 0x01                   ← F9P 拒絕 M8P 設定的 CFG-MSG
```

M8P rover 設定的 RTCM 輸入設定:

```yaml
device: /dev/ttyACM0      # ROS 走 USB
uart1: { baudrate: 19200, in: 32 }   # UART1 收 RTCM3,鮑率鎖 19200
dgnss_mode: 3             # Fixed
```

問題:
- ZED-F9P 的 UART1 出廠預設 **38400**,被 M8P 設定改成 **19200**。基站電台送進 UART1 的鮑率若 ≠ 19200 → F9P 解不出 → `/rxmrtcm=0`。**這是 RTCM 收不到的頭號原因。**
- 用 M8P 設定跑 F9P,啟動一直 NACK / cold reset,配置不乾淨。

原因可能性排序:
1. **UART1 鮑率不符**(電台輸出 ≠ F9P UART1 的 19200)← 最可能
2. **設定檔錯用 M8P**(應為 F9P)
3. 電台→UART1 接線(TX/RX/GND)
4. 基站端未 survey-in/fixed 或未發 RTCM3

要修:
1. 改用正確 F9P 設定(或把 `uart1.baudrate` 改成與基站電台一致),`uart1.in` 需含 RTCM3(32)。
2. 兩端鮑率對齊 + 確認接線。
3. 確認基站已 fixed 並發 RTCM3。
4. 重啟 ublox 後,`/rxmrtcm` 應開始有資料、`/fix` 協方差掉到 <0.01 m² = RTK fixed。

---

## 五、融合設定的問題(`nav2_gps_waypoint_follower/config/dual_ekf_navsat_params.yaml` + `dual_ekf_navsat.launch.py`)

這份設定基本是 robot_localization 官方 turtlebot 範例改的,有幾個**會直接致命**的問題:

1. **【致命】datum 座標錯 120 公里**
   `datum: [22.6234688, 120.3800577, 0.0, map, base_footprint]`(高雄),但機器人在 `23.6996953, 120.5397778`(雲林/嘉義)。
   `wait_for_datum: true` + 固定 datum → 機器人會被放在離 map 原點 **~120.8 km** 的地方,costmap/導航直接爆掉。
   → 改法:`wait_for_datum: false`(用第一筆 GPS 自動當原點),或把 datum 改成實際場地座標。

2. **【致命】navsat_transform 收不到 GPS**
   launch 裡 remap 是 `('gps/fix','gps/fix')`(等於沒改),節點會去訂閱 `/gps/fix`,但實際 GPS 在 **`/fix`**。
   → 改法:remap 成 `('gps/fix','/fix')`。

3. **【致命】EKF 收不到輪速里程**
   設定 `odom0: odom`,launch 也沒 remap,節點訂閱 `/odom`,但實際里程在 **`/diff_controller/odom`**。
   → 改法:`odom0: diff_controller/odom`(或在 launch remap)。

4. **【需調】磁偏角沒設**
   `magnetic_declination_radians: 0.0`。台灣磁偏角約 **-4.5°(≈ -0.079 rad,偏西)**。若 IMU 是磁北參考,會造成航向偏差,融合移動時方向會歪。
   → 依實際 IMU 是否輸出磁北航向決定要不要填。

5. **【確認】IMU 已正確接上**
   EKF/navsat 的 `imu` 有 remap 到 `imu/data`(實際存在,✓),base_link_frame=`base_footprint`(tf 上有,✓)。這兩個是對的。

---

## 六、能不能當導航來源?評估

| 用途 | 目前狀態(單點 ±1.2m, 1Hz, 融合未開) | RTK fixed 修好後(±2cm) |
|---|---|---|
| 粗略路點導航(容忍 1–2m) | 勉強,但融合修好前不行 | ✅ 綽綽有餘 |
| 割草直線行列(需 ~10cm) | ❌ 精度不足 | ✅ 可行 |
| 邊界/避障貼邊 | ❌ | ✅ |

**結論:以「割草機」需要的精度,目前的定位不能當導航來源。** 不是因為 tf 會亂跳(tf 很穩),而是因為:
1. RTK 沒吃到修正 → 只有 ±1.2m 單點精度;
2. 融合鏈沒接通(3 個致命設定錯誤)→ 位置根本上不了 tf,且會差 120 公里。

---

## 七、建議修正順序

1. **先修 RTK 修正流**(最關鍵):接 NTRIP client / 基站,確認 `/rxmrtcm` 有資料、`/fix` 協方差 < 0.01 m²、達到 RTK fixed。
2. **修融合接線**:navsat `gps/fix→/fix`、EKF `odom0→diff_controller/odom`。
3. **修 datum**:`wait_for_datum: false` 讓它用第一筆 fix 自動設原點(最省事、最不會差 120 公里)。
4. **設磁偏角**(視 IMU 而定):台灣約 -0.079 rad。
5. **啟動融合後再量跳動**:用附錄的腳本監測 `map → base_footprint`,確認融合後每步位移是否合理(RTK fixed 下應該是公分級、平滑)。

> 注意:目前 Nav2(bt_navigator / costmaps / planner)已在跑,但缺 map→odom→base_link 這條鏈,實際是無法定位導航的。以上修好後這條鏈才會通。

---

## 八、修復實作與結果(2026-07-01 實測)

依上述診斷,實際動手修復並驗證:

1. **換成 F9P rover 設定**:備份原 `c94_m8p_rover.yaml`(→ `.ORIG`),改寫成 F9P rover 設定(`dgnss_mode:3`、`tmode3:0`、`uart1.in: 32` RTCM3)。
2. **掃描 UART1 鮑率**:19200(原值)→ **收不到 RTCM**;改 **38400 → 10 秒收到 45 筆 RTCM,問題解決**。

**根因確認:基站電台是以 38400 baud 送 RTCM,但 M8P 設定把 F9P 的 UART1 鎖在 19200,鮑率對不上,所以之前 `/rxmrtcm=0`、退化單點定位。**

修復後實測:
- `/rxmrtcm` 持續有資料,`flags` 顯示 msgUsed(修正有被使用)。
- 基站送出**完整 RTK 修正組**:`1005`(基站座標)、`1077/1087/1097/1127`(GPS/GLONASS/Galileo/BeiDou MSM7)、`1230`(GLONASS 偏差),1 Hz,正常。
- 每次改設定重啟會觸發 GNSS 重設(cold reset),接收機需時間重新收星;收斂中協方差從 ~1580 → 逐步下降,`num_sv` 從低慢慢回升。RTK float/fixed 需收星恢復後才會進入(觀察 `/navpvt` 的 carrSoln 或 `/navrelposned`)。

**待辦 / 注意:**
- 設定改動在容器 `/opt/...` 層,`docker restart` 保留;若容器被重建(compose recreate)會還原,需把此設定納入正式 image/掛載。
- `uart1.baudrate` 已定為 38400(對齊基站電台)。
- RTK 進 fixed 後,再啟動融合(修 datum/接線,見第五節),即可把公分級定位發上 tf 樹供導航。

---

## 附錄:監測工具(已放進容器)

- `/tmp/tf_jump_monitor.py`  —  監測每個 tf transform 的頻率 / 最大 Δ位移 / Δ角度 / 跳動次數
  用法:`python3 /tmp/tf_jump_monitor.py <位移閾值m> <角度閾值°> <秒數>`
- `/tmp/gps_jitter.py`  —  量 `/fix` 靜止漂移(逐點跳動、離均值漂移、更新率、協方差)
  用法:`python3 /tmp/gps_jitter.py <秒數>`

融合啟動後,把 `tf_jump_monitor.py` 監測到的 `map → base_footprint` 那一列拿來看,就能直接量出「GPS 融合後在 tf 上的位置每步跳幾公尺」。
