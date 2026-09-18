# 電池電量估計（SOC）

主電池：6S 鋰電，滿電 `25.2 V`（4.2 V/cell），截止 `18.0 V`（3.0 V/cell），沒有會回報的 BMS。
小電池：3.7 V 1S，維持 STM32 常開，自帶充電 IC，STM32 只量得到電壓。

## 資料鏈

```
STM32 ADC mux CH2/CH3 ──0x8A ANALOG_STATUS (50 ms, 200 ms 更新)──┐
RS485 充電模組 Vout/Iout/CC/CV ──0x89 CHARGER_STATUS (500 ms 輪詢)─┤
                                                                 ▼
                       mower_hardware  →  /mower_base/telemetry {analog, charger}
                                                                 ▼
                 mower_mission battery_state_node  →  /battery_state, /aon_battery_state
                                                                 ▼
                                         rosbridge → App / mower_recorder bag
```

- 韌體：`firmware/Module/Src/analog_monitor.cpp`（每輪先讀 VREFINT 校正 VDDA）、`firmware/UART_OPEN_LOOP_PROTOCOL.md` `0x8A` / `0x89`
- Host 解碼：`src/mower_hardware/src/mower_protocol.cpp` `decode_analog_status` / `decode_charger_status`
- 估計：`src/mower_mission/mower_mission/battery_estimator.py`（純 Python，`test/test_battery_estimator.py`）
- 節點：`src/mower_mission/mower_mission/battery_state_node.py`，參數見檔頭

## 第一步（已實作）：電壓 OCV 查表

沒有放電電流感測，所以只能用電壓：

1. 電池組電壓過一階低通（`filter_tau_s`，預設 20 s），吃掉馬達啟動的瞬間壓降
2. 每 cell 電壓查 NMC 18650 典型 OCV 表（`DEFAULT_OCV_TABLE`，3.0 → 0 %、3.78 → 50 %、4.2 → 100 %）
3. 放電中：估計值可以自由往下，往上最多 `recovery_rate_pct_per_min`（預設 1 %/min）——長時間割草壓降後電壓回彈，不會一秒跳回去，但也不會永遠卡在低點
4. 充電中（`0x89 CHARGING`）：估計值只准往上，且上限 99 %，因為充電端電壓高於 OCV 會高估
5. 充飽：`CV_PHASE` 且 `Iout <= full_tail_current_a`（0.2 A）持續 `full_hold_s`（60 s）→ 100 %，`FULL`，同時把估計重新對齊
6. 充電器在線但沒電流 → `NOT_CHARGING`；≥ 99.5 % 時報 `FULL`

限制：
- 負載下誤差約 ±10 %，靜置後較準；OCV 表是通用值，沒對這顆電池實測
- 分壓比 `270k/33k` 是圖面值，`main_battery_v` 需拿三用電表校正（差 1 % 電壓在平坦段會差 ~5 % SOC）
- 沒有溫度補償（板溫 NTC 有量但沒用）
- 沒有電量（Ah）、沒有剩餘時間估計

## 第二步（TODO）：庫侖計數 + OCV 校正

要準確就得量主電池電流，然後用電流積分算電量，OCV 只在靜置時用來校正漂移。

### 硬體

| 方案 | 說明 | 取捨 |
|---|---|---|
| **INA226 / INA228（I2C）** | 高側電流 + 電壓 + 功率，內建 16-bit ADC 與平均，shunt 例如 2 mΩ / 20 A | 建議；STM32F411 有空的 I2C（查 `wire.md` 腳位表），不佔 ADC mux；INA228 有內建電荷累加暫存器，STM32 只要定期讀 |
| Hall 電流感測（ACS712/ACS758）→ ADC | 隔離、便宜 | ADC mux 4 通道已用滿，要換 8 通道 4051 或再拉一支 ADC 腳；零點漂移大，低電流不準 |
| 帶 UART/SMBus 的 BMS | 直接給 SOC、cell 電壓、溫度 | 要換電池組；若之後換電池，優先選有通訊的 BMS，第一、二步都可以退場 |

Shunt 放在電池負極（低側）最簡單，但 INA226 高側也可以（最高 36 V，25.2 V OK）。充電電流也會流過同一顆 shunt，所以充電與放電用同一個計數器，`0x89 Iout` 只當交叉檢查。

### 韌體

- 新增 `Module/battery_gauge`：每 10–20 ms 讀電流，`charge_mah += I * dt`；INA228 的話讀它的 `CHARGE` 暫存器即可
- 靜置判定：`|I| < 0.1 A` 持續 ≥ 5 min → 用 OCV 表把 `soc` 對齊到電壓（溫度補償可用板溫 NTC）
- 滿電判定沿用充電器 tail current；此時 `soc = 100 %` 並記下 `capacity_mah`（累積的放電量），做容量學習
- 斷電保存：`soc` 與 `capacity_mah` 寫進 settings flash（`settings_storage`，sector 7 已用於 PID，同一區塊加欄位），開機讀回；AON 電池讓 STM32 不斷電，所以其實只在 AON 也耗盡時才需要
- `0x8A` 加欄位或新開 `0x8B BATTERY_GAUGE`：`current_ca`（有號）、`charge_mah`、`soc_permille`、`capacity_mah`、`flags(RESTING, CALIBRATED, LEARNED)`

### Host

- `battery_state_node` 改成優先用韌體 SOC，`BatteryState.current` 填真實放電電流（負值）、`charge` / `capacity` 填 mAh → Ah
- 剩餘時間：最近 5 min 平均功率 ÷ 剩餘電量，給 App 顯示「還能割 N 分鐘」
- 低電量回充門檻改成用 SOC + 剩餘時間，而不是只看電壓

### 充電曲線

`/mower_base/telemetry` 已進 `mower_recorder` bag，`charger.vout_v / iout_a / cv_phase` 每 500 ms 一點。有實體充電器後：

1. 從 20 % 充到 tail current，`ros2 bag` 匯出 `vout / iout vs t`
2. 對比同一段時間的 `analog.main_battery_v`，量出充電迴路的 I·R（電壓差 ÷ Iout），可以拿來修正「充電中 OCV 高估」
3. 充飽後靜置 30 min 再放電到截止，用第二步的庫侖計數量實際容量，重畫這顆電池的 OCV 表取代 `DEFAULT_OCV_TABLE`

## 校正待辦

- [ ] 三用電表量電池端 vs `analog.main_battery_v`，修 `MAIN_BATTERY_DIVIDER_GAIN`（或在 node 加 `voltage_gain` 參數）
- [ ] 同上，`aon_battery_v`
- [ ] 確認 NTC 型號 / beta，板溫才可信
- [ ] 充電器到貨後跑一次完整充電，看 `CV_PHASE` / tail current 門檻是否合理
