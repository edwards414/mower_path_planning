# UART Motor / LED Control Protocol

這份文件是給 `ros2_control` / serial bridge 端對接 STM32 使用。

本版主行走馬達 `0x01 / 0x81 / 0x85`、`lawer_motor` `0x02 / 0x82`、`ws2812` `0x03 / 0x83` 與 PID 設定 `0x04 / 0x84` 都已接上 runtime handler 與 status 回傳。

## 目的

- 這條 UART 負責「左右輪目標速度命令」、「割草馬達開環命令」、「WS2812 命令」、「PID 設定」和「STM32 狀態回傳」。
- 左右輪馬達在 STM32 端以 FT-555 encoder 做 PID 速度閉環；`0x01` 的 `permille` 是目標速度比例，不再是直接 PWM duty。
- UART 上不再輸出文字 log，整條 link 都是 binary frame。

## 基本規則

- Endianness: little-endian
- Protocol version: `0x01`
- SOF0: `0xA5`
- SOF1: `0x5A`
- CRC: CRC-16/CCITT-FALSE
- CRC polynomial: `0x1021`
- CRC init: `0xFFFF`
- CRC xorout: `0x0000`
- CRC 計算範圍: 從 `version` 開始，到 `payload` 結束
- 一個完整 frame 長度: `8 + payload_len`

## Frame Layout

| Offset | Size | Field | Description |
|---|---:|---|---|
| 0 | 1 | `sof0` | 固定 `0xA5` |
| 1 | 1 | `sof1` | 固定 `0x5A` |
| 2 | 1 | `version` | 目前固定 `0x01` |
| 3 | 1 | `type` | frame 類型 |
| 4 | 1 | `seq` | 8-bit sequence number |
| 5 | 1 | `payload_len` | payload byte 數 |
| 6 | N | `payload` | 依 type 而定 |
| 6 + N | 1 | `crc_lo` | CRC 低位元組 |
| 7 + N | 1 | `crc_hi` | CRC 高位元組 |

## Frame Type

| Type | Direction | Meaning |
|---|---|---|
| `0x01` | Host -> STM32 | 左右輪目標速度命令 |
| `0x02` | Host -> STM32 | `lawer_motor` 開環命令 |
| `0x03` | Host -> STM32 | `ws2812` 模式命令 |
| `0x04` | Host -> STM32 | 左右輪 PID 設定 |
| `0x81` | STM32 -> Host | 狀態回傳 |
| `0x82` | STM32 -> Host | `lawer_motor` 狀態回傳格式 |
| `0x83` | STM32 -> Host | `ws2812` 狀態回傳格式 |
| `0x84` | STM32 -> Host | PID 設定狀態回傳格式 |
| `0x85` | STM32 -> Host | 左右輪 PID / encoder 回饋格式 |

## `0x01` Wheel Speed Command

Payload 長度固定 `8` bytes。

| Offset | Type | Field | Description |
|---|---|---|---|
| 0 | `int16_t` | `left_command_permille` | 左輪目標速度，範圍 `-1000 ~ 1000` |
| 2 | `int16_t` | `right_command_permille` | 右輪目標速度，範圍 `-1000 ~ 1000` |
| 4 | `uint16_t` | `command_timeout_ms` | 命令 timeout，`0` 代表 STM32 使用預設值 |
| 6 | `uint16_t` | `reserved` | 目前固定填 `0` |

語意:

- `-1000` = 反向最大目標速度，預設約 `-58 rpm`
- `0` = 停止
- `1000` = 正向最大目標速度，預設約 `58 rpm`
- STM32 會用左右輪 encoder 算 RPM，再用 PID 輸出 BTS7960 PWM

Host 端建議:

- `seq` 每送一包加 1，8-bit overflow 可直接回捲
- 發送頻率建議 `20Hz ~ 50Hz`
- `command_timeout_ms` 建議設成 `150 ~ 300ms`

## `0x02` Lawer Motor Open-Loop Command

Payload 長度固定 `8` bytes。

| Offset | Type | Field | Description |
|---|---|---|---|
| 0 | `int16_t` | `command_permille` | 範圍 `-1000 ~ 1000` |
| 2 | `uint16_t` | `command_timeout_ms` | 命令 timeout，`0` 代表 STM32 使用預設值 |
| 4 | `uint16_t` | `reserved0` | 目前固定填 `0` |
| 6 | `uint16_t` | `reserved1` | 目前固定填 `0` |

語意:

- `-1000` = 全反轉
- `0` = 停止
- `1000` = 全正轉
- 這是 `lawer_motor` 的開環 duty / PWM 命令

## `0x03` WS2812 Command

Payload 長度固定 `8` bytes。

| Offset | Type | Field | Description |
|---|---|---|---|
| 0 | `uint8_t` | `mode` | 燈效模式 |
| 1 | `uint8_t` | `r` | 紅色 |
| 2 | `uint8_t` | `g` | 綠色 |
| 3 | `uint8_t` | `b` | 藍色 |
| 4 | `uint16_t` | `effect_period_ms` | 動畫步進週期 |
| 6 | `uint8_t` | `reserved0` | 目前固定填 `0` |
| 7 | `uint8_t` | `reserved1` | 目前固定填 `0` |

`mode` 定義:

- `0x00`: `CLEAR`
- `0x01`: `ALL_ON`
- `0x02`: `FLOW`
- `0x03`: `TURN_LEFT`
- `0x04`: `TURN_RIGHT`
- `0x05`: `SHOW`

補充:

- `CLEAR` / `SHOW` 可以忽略 `r/g/b/effect_period_ms`
- `ALL_ON` 主要用 `r/g/b`
- `FLOW` 用 `r/g/b + effect_period_ms`
- `TURN_LEFT` / `TURN_RIGHT` 主要用 `effect_period_ms`

## `0x04` PID Config Command

Payload 長度固定 `28` bytes。

| Offset | Type | Field | Description |
|---|---|---|---|
| 0 | `float` | `left_kp` | 左輪 PID Kp |
| 4 | `float` | `left_ki` | 左輪 PID Ki |
| 8 | `float` | `left_kd` | 左輪 PID Kd |
| 12 | `float` | `right_kp` | 右輪 PID Kp |
| 16 | `float` | `right_ki` | 右輪 PID Ki |
| 20 | `float` | `right_kd` | 右輪 PID Kd |
| 24 | `uint8_t` | `persist_to_flash` | `0` 只改 RAM，非 `0` 寫入 STM32 internal Flash |
| 25 | `uint8_t` | `closed_loop_enabled` | `0` 關閉 PID 改用直接 PWM；非 `0` 啟用 PID |
| 26 | `uint16_t` | `reserved` | 目前固定填 `0` |

預設 PID:

| Wheel | Kp | Ki | Kd |
|---|---:|---:|---:|
| Left | `2.0` | `0.6` | `0.0` |
| Right | `2.0` | `0.6` | `0.0` |

內部 Flash 儲存：

- 使用 STM32F411 internal Flash sector 7，位址 `0x08060000`
- linker script 已把程式碼 Flash 限制在前 `384KB`，最後 `128KB` 保留給設定
- 不要高頻率寫入 PID；調參時先用 `persist_to_flash=0`，確認後再寫一次 Flash

## `0x81` Motor Status

STM32 目前每 `50ms` 送一次 status。

Payload 長度固定 `12` bytes。

| Offset | Type | Field | Description |
|---|---|---|---|
| 0 | `int16_t` | `commanded_left_permille` | 最近一次有效命令的左輪目標 |
| 2 | `int16_t` | `commanded_right_permille` | 最近一次有效命令的右輪目標 |
| 4 | `int16_t` | `applied_left_pwm` | 實際套用的左輪 PWM counts |
| 6 | `int16_t` | `applied_right_pwm` | 實際套用的右輪 PWM counts |
| 8 | `uint16_t` | `command_age_ms` | 這份命令距離現在多久 |
| 10 | `uint8_t` | `flags` | 狀態 bit flags |
| 11 | `uint8_t` | `last_rx_seq` | 最近一次成功接收命令的 seq |

補充:

- status frame header 的 `seq` 也會等於 `last_rx_seq`
- `applied_*_pwm` 是 PID 計算後實際輸出的 PWM counts，不是輪速
- 如果 timeout 或 alarm 發生，`applied_*_pwm` 會被拉成 `0`

## `0x82` Lawer Motor Status

Payload 長度固定 `8` bytes。

| Offset | Type | Field | Description |
|---|---|---|---|
| 0 | `int16_t` | `commanded_permille` | 最近一次有效命令 |
| 2 | `int16_t` | `applied_pwm` | 實際套用的 PWM counts |
| 4 | `uint16_t` | `command_age_ms` | 命令已經過多久 |
| 6 | `uint8_t` | `flags` | 狀態 bit flags |
| 7 | `uint8_t` | `last_rx_seq` | 最近一次成功接收命令的 seq |

## `0x83` WS2812 Status

Payload 長度固定 `8` bytes。

| Offset | Type | Field | Description |
|---|---|---|---|
| 0 | `uint8_t` | `mode` | 目前模式 |
| 1 | `uint8_t` | `r` | 目前紅色 |
| 2 | `uint8_t` | `g` | 目前綠色 |
| 3 | `uint8_t` | `b` | 目前藍色 |
| 4 | `uint16_t` | `effect_period_ms` | 目前動畫步進週期 |
| 6 | `uint8_t` | `flags` | 狀態 bit flags |
| 7 | `uint8_t` | `last_rx_seq` | 最近一次成功接收命令的 seq |

## `0x84` PID Config Status

Payload 長度固定 `28` bytes。

| Offset | Type | Field | Description |
|---|---|---|---|
| 0 | `float` | `left_kp` | 目前左輪 Kp |
| 4 | `float` | `left_ki` | 目前左輪 Ki |
| 8 | `float` | `left_kd` | 目前左輪 Kd |
| 12 | `float` | `right_kp` | 目前右輪 Kp |
| 16 | `float` | `right_ki` | 目前右輪 Ki |
| 20 | `float` | `right_kd` | 目前右輪 Kd |
| 24 | `uint8_t` | `flags` | PID/status flags |
| 25 | `uint8_t` | `last_rx_seq` | 最近一次成功接收 PID command 的 seq |
| 26 | `uint16_t` | `reserved` | 目前固定 `0` |

PID status flags:

| Bit | Mask | Meaning |
|---|---|---|
| 0 | `0x01` | `CLOSED_LOOP_ENABLED` |
| 1 | `0x02` | `FLASH_VALID`，開機有讀到有效 Flash 設定 |
| 2 | `0x04` | `LAST_SAVE_OK`，最近一次 Flash 儲存成功 |
| 3 | `0x08` | `LAST_APPLY_OK`，最近一次 PID command 套用成功 |

## `0x85` Wheel Feedback Status

STM32 目前每 `50ms` 送一次 wheel feedback，frame header 的 `seq` 等於最近一次成功接收的左右輪速度命令 seq。

Payload 長度固定 `24` bytes。

| Offset | Type | Field | Description |
|---|---|---|---|
| 0 | `int16_t` | `left_target_rpm_x100` | 左輪目標 RPM x100 |
| 2 | `int16_t` | `left_measured_rpm_x100` | 左輪 encoder 實測 RPM x100 |
| 4 | `int16_t` | `right_target_rpm_x100` | 右輪目標 RPM x100 |
| 6 | `int16_t` | `right_measured_rpm_x100` | 右輪 encoder 實測 RPM x100 |
| 8 | `int16_t` | `left_pid_output` | 左輪 PID 輸出 PWM counts |
| 10 | `int16_t` | `right_pid_output` | 右輪 PID 輸出 PWM counts |
| 12 | `int32_t` | `left_delta_counts` | 左輪最近一個 20ms control tick 的 encoder 增量 |
| 16 | `int32_t` | `right_delta_counts` | 右輪最近一個 20ms control tick 的 encoder 增量 |
| 20 | `uint8_t` | `flags` | wheel controller flags |
| 21 | `uint8_t` | `reserved0` | 目前固定 `0` |
| 22 | `uint8_t` | `reserved1` | 目前固定 `0` |
| 23 | `uint8_t` | `reserved2` | 目前固定 `0` |

Wheel controller flags:

| Bit | Mask | Meaning |
|---|---|---|
| 0 | `0x01` | `OUTPUT_ENABLED`，目前安全條件允許輸出 |
| 1 | `0x02` | `CLOSED_LOOP`，目前使用 PID 閉環 |
| 2 | `0x04` | `FLASH_SETTINGS_VALID`，開機有讀到有效 Flash 設定 |
| 3 | `0x08` | `LAST_SAVE_OK`，最近一次 Flash 儲存成功 |

## Status Flags

| Bit | Mask | Meaning |
|---|---|---|
| 0 | `0x01` | `COMMAND_VALID`，代表至少收過一筆有效命令 |
| 1 | `0x02` | `COMMAND_TIMEOUT`，代表命令超時，輸出已被切成 0 |
| 2 | `0x04` | `DRIVER_ALARM`，代表偵測到 motor driver alarm |

常見組合:

- `0x00`: 尚未收到有效命令
- `0x01`: 有有效命令，且目前未 timeout、未 alarm
- `0x03`: 曾收到有效命令，但目前已 timeout
- `0x05`: 有有效命令，但目前 driver alarm
- `0x07`: 有有效命令，且 timeout + driver alarm 同時成立

## Host 實作注意事項

- 請不要再把這條 UART 當作文字串流解析
- Host 應該把 `last_rx_seq` 視為「最近一次成功被 STM32 接受的命令」
- CRC 錯誤、version 不符、type 不支援的 frame，STM32 會直接丟掉
- 控制命令是 latest-wins，不要依賴 FIFO 語意
- `0x02 / 0x82 / 0x03 / 0x83 / 0x04 / 0x84 / 0x85` 已接上 runtime path，host 可以直接依本文件封包格式對接

## 建議資料流

1. ROS2 端把左右輪速度命令正規化到 `-1000 ~ 1000`
2. 打包成對應的 `0x01 / 0x02 / 0x03 / 0x04` frame 並送到 STM32
3. 讀回對應的 `0x81 / 0x82 / 0x83 / 0x84 / 0x85` status frame
4. 用 `last_rx_seq` 檢查 STM32 是否已接受最新命令
5. 用 `flags`、`command_age_ms`、`applied_*_pwm` 做 safety / health monitoring

## 目前 STM32 端行為

- 控制迴圈: `50Hz`
- status 回傳週期: `50ms`
- 預設 command timeout: `200ms`
- command 超時或 driver alarm 時，左右輪輸出會直接變成 `0`
