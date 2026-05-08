# UART Open-Loop Motor Protocol

這份文件是給 `ros2_control` / serial bridge 端對接 STM32 使用。

本版除了主行走馬達 `0x01 / 0x81` 已接上執行邏輯之外，`mower_motor` 與 `ws2812` 這次先把 binary frame 格式定義好，方便 ROS2 端同步改封包。這兩組 frame 的 runtime handler 目前還沒有接進獨立 task。

## 目的

- 這條 UART 只負責「開環馬達命令」和「STM32 狀態回傳」。
- 這不是速度閉環控制協定，因為目前 STM32 端沒有 encoder。
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
| `0x01` | Host -> STM32 | 開環左右輪命令 |
| `0x02` | Host -> STM32 | `mower_motor` 開環命令 |
| `0x03` | Host -> STM32 | `ws2812` 模式命令 |
| `0x81` | STM32 -> Host | 狀態回傳 |
| `0x82` | STM32 -> Host | `mower_motor` 狀態回傳格式 |
| `0x83` | STM32 -> Host | `ws2812` 狀態回傳格式 |

## `0x01` Motor Open-Loop Command

Payload 長度固定 `8` bytes。

| Offset | Type | Field | Description |
|---|---|---|---|
| 0 | `int16_t` | `left_command_permille` | 左輪命令，範圍 `-1000 ~ 1000` |
| 2 | `int16_t` | `right_command_permille` | 右輪命令，範圍 `-1000 ~ 1000` |
| 4 | `uint16_t` | `command_timeout_ms` | 命令 timeout，`0` 代表 STM32 使用預設值 |
| 6 | `uint16_t` | `reserved` | 目前固定填 `0` |

語意:

- `-1000` = 全反轉
- `0` = 停止
- `1000` = 全正轉
- 這是開環 duty / PWM 命令，不是真實速度

Host 端建議:

- `seq` 每送一包加 1，8-bit overflow 可直接回捲
- 發送頻率建議 `20Hz ~ 50Hz`
- `command_timeout_ms` 建議設成 `150 ~ 300ms`

## `0x02` Mower Motor Open-Loop Command

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
- 這是 `mower_motor` 的開環 duty / PWM 命令

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
- `applied_*_pwm` 是 STM32 實際輸出的 PWM counts，不是輪速
- 如果 timeout 或 alarm 發生，`applied_*_pwm` 會被拉成 `0`

## `0x82` Mower Motor Status

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
- 請不要期待速度或里程回授，這份 status 不是 encoder feedback
- Host 應該把 `last_rx_seq` 視為「最近一次成功被 STM32 接受的命令」
- CRC 錯誤、version 不符、type 不支援的 frame，STM32 會直接丟掉
- 控制命令是 latest-wins，不要依賴 FIFO 語意
- `0x02 / 0x82 / 0x03 / 0x83` 這版先定義格式，方便 host 先同步改封包；實際 handler 與 status producer 還沒接到獨立 runtime path

## 建議資料流

1. ROS2 端把左右輪開環命令正規化到 `-1000 ~ 1000`
2. 打包成對應的 `0x01 / 0x02 / 0x03` frame 並送到 STM32
3. 讀回對應的 `0x81 / 0x82 / 0x83` status frame
4. 用 `last_rx_seq` 檢查 STM32 是否已接受最新命令
5. 用 `flags`、`command_age_ms`、`applied_*_pwm` 做 safety / health monitoring

## 目前 STM32 端行為

- 控制迴圈: `50Hz`
- status 回傳週期: `50ms`
- 預設 command timeout: `200ms`
- command 超時或 driver alarm 時，左右輪輸出會直接變成 `0`
