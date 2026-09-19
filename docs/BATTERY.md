# 電池電量估計（SOC）

主電池：6S 鋰電，滿電 `25.2 V`（4.2 V/cell），截止 `18.0 V`（3.0 V/cell），沒有會回報的 BMS。
小電池：3.7 V 1S，維持 STM32 常開，自帶充電 IC；目前沒有量測（STM32 ADC 分壓未裝）。

## 資料鏈

```
RS485 電池電表（電池主線上）電壓 / 電流 / 溫度 ──0x89 CHARGER_STATUS (500 ms 輪詢)──┐
STM32 ADC mux（目前未裝，ANALOG_MONITOR_CHANNEL_MASK=0）──0x8A ANALOG_STATUS──────┤
                                                                                   ▼
                                    mower_hardware  →  /mower_base/telemetry {charger, analog}
                                                                                   ▼
                              mower_mission battery_state_node  →  /battery_state, /aon_battery_state
                                                                                   ▼
                                                      rosbridge → App / mower_recorder bag
```

- 電表：`firmware/wire.md`「RS485 電池電表」——裝在電池側，電壓永遠是電池端電壓，電流端子 **還沒接進主線**（讀 0）
- 韌體：`firmware/Module/charger_rs485`（0x89）、`firmware/Module/analog_monitor`（0x8A，通道由 `hardware_pins.hpp` `ANALOG_MONITOR_CHANNEL_MASK` 決定）
- Host 解碼：`src/mower_hardware/src/mower_protocol.cpp` `decode_charger_status` / `decode_analog_status`
- 估計：`src/mower_mission/mower_mission/battery_estimator.py`（純 Python，`test/test_battery_estimator.py`）
- 節點：`src/mower_mission/mower_mission/battery_state_node.py`，參數見檔頭與 `launch/mission.launch.py`

## 現況（2026-09-19）：電表只有電壓

`battery_state_node` 的輸入：

| 量 | 來源 | 現在 |
|---|---|---|
| 電池電壓 | `charger.voltage_v`（電表在線）→ 否則 `analog.main_battery_v`（ADC 有裝時） | 電表，25.64 V 插充電器 / 24.04 V 靜置 |
| 充電器在不在 | 電池端電壓 `>= charger_present_min_v`（25.0 V）。電表看不到充電器，但充電器插著會把端電壓頂到它的 CV，比任何靜置 OCV（≤ 25.2 V 滿電）都高 | 用電壓判 |
| 電池電流 | `charger.current_a`，`meter_current_wired=true` 才吃 | 未接，NaN |

估算邏輯（`BatteryEstimator`）：

1. 電池組電壓過一階低通（`filter_tau_s`，預設 20 s），吃掉馬達啟動的瞬間壓降
2. **放電中**（沒充電器）：每 cell 電壓查 NMC 18650 典型 OCV 表（`DEFAULT_OCV_TABLE`，3.0 → 0 %、3.78 → 50 %、4.2 → 100 %）。估計值可以自由往下，往上最多 `recovery_rate_pct_per_min`（1 %/min）——壓降回彈不會一秒跳回去
3. **充電中**（電壓 ≥ 25.0 V）：端電壓是充電器的 CV，跟電量無關，**不查 OCV**（查了會瞬間跳 99 %）。估計值從拔線前的值以 `charge_rate_pct_per_min`（預設 0.5 %/min ≈ 3.3 h 充滿）往上爬，上限 99 %。沒電流就沒辦法判充飽，`FULL` 不會出現
4. **拔掉充電器**：端電壓要幾分鐘才鬆下來，先維持原估計 `post_charge_settle_s`（300 s），然後用 OCV 對齊一次
5. `charge_rate_pct_per_min` 請照實際充電器算：`100 / (電池 Ah ÷ 充電器 A) / 60`，例如 20 Ah 電池、2 A 充電器 = 0.17 %/min

限制：
- 負載下誤差約 ±10 %，靜置後較準；OCV 表是通用值，沒對這顆電池實測
- 充電中的百分比是「估計爬升」，不是量到的；充飽判不出來
- 沒有溫度補償、沒有剩餘時間估計

## 下一步：把電表的分流器接進主線 → 庫侖計數

電表本身有電流量測，只是電池負極主線沒有經過它的分流器。接上後 `charger.current_a` 就是電池組電流（充放電都看得到），估算器自動切到庫侖計數，**不用再買 INA226**：

1. 硬體：電池負極 → 電表電流端子（或外掛分流器 / 霍爾環）→ 整車負載，充電器也接在電表的電池側，這樣充電電流也經過它
2. 設定：`launch/mission.launch.py` 的 `battery_state` 參數 `meter_current_wired: true`、`capacity_ah: <電池標稱 Ah>`
3. 方向：Reg1 是 `uint16`，可能沒有正負號。`meter_current_signed: false`（預設）時方向由「充電器在不在」決定：充電器在 = 正（充電）、不在 = 負（放電）；如果實測發現放電時 raw 變 65xxx（有號數回捲），改 `meter_current_signed: true`
4. 估算（已實作，`test_battery_estimator.py` 有測）：
   - `fraction += I × dt / capacity`，充放電都算
   - 靜置（|I| < `rest_current_a` 0.1 A 持續 `rest_hold_s` 5 min）用 OCV 把估計對回去，消積分漂移
   - 充飽：充電器在、電流 ≤ `full_tail_current_a`（0.2 A）、cell 電壓 ≥ `full_min_cell_v`（4.10 V）持續 `full_hold_s`（60 s）→ 100 %、`FULL`
   - 充電器在但電流 ≈ 0 且沒到 CV → `NOT_CHARGING`
5. 之後可加：剩餘時間（最近 5 min 平均電流 ÷ 剩餘 Ah）、容量學習（充飽時記下累積放電量）、板溫補償

再進一步可以考慮給 STM32 一條「充電器在」的感測線（充電器 +24 V 經分壓進 `PA7`），取代用電壓判——電壓判在電池接近滿電、充電器 CV 設得低時會模糊。

## 充電曲線

`/mower_base/telemetry` 已進 `mower_recorder` bag，`charger.voltage_v / current_a` 每 500 ms 一點。電流接上之後：

1. 從 20 % 充到 tail current，`ros2 bag` 匯出 `voltage_v / current_a vs t`
2. 充飽後靜置 30 min 再放電到截止，用庫侖計數量實際容量，重畫這顆電池的 OCV 表取代 `DEFAULT_OCV_TABLE`

## 校正待辦

- [x] RS485 電表對照面板確認 Reg0-2 = 電壓 / 電流 / 溫度（2026-09-19，25.6 V / 0 A / 35 °C）
- [x] 確認電表位置在電池側：拔充電器 25.64 → 24.04 V（2026-09-19）
- [ ] 電表電流端子接進電池主線，確認放電時 `current_a` 不是 0、看 raw 有沒有正負號
- [ ] 填 `capacity_ah`、`charge_rate_pct_per_min`（電池 Ah、充電器 A）
- [ ] 接上充電器跑一次完整充電，看 tail current（0.2 A）與 `full_min_cell_v`（4.10 V）門檻是否合理
- [ ] STM32 ADC mux / 分壓若之後補上，`ANALOG_MONITOR_CHANNEL_MASK` 改回 `0x0F`，並用電表電壓校正 `MAIN_BATTERY_DIVIDER_GAIN`
