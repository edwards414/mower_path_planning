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
| `0x05` | Host -> STM32 | 電源 / 關機命令（ack、請求關機、取消、強制斷電） |
| `0x06` | Host -> STM32 | 保留：`INFO_REQUEST`（`mower_path_planning/firmware` 版的 build identity 查詢） |
| `0x07` | Host -> STM32 | MG996 servo 脈寬命令 |
| `0x0F` | Host -> STM32 | 重開進 UART bootloader（見 `BOOTLOADER.md`） |
| `0x81` | STM32 -> Host | 狀態回傳 |
| `0x82` | STM32 -> Host | `lawer_motor` 狀態回傳格式 |
| `0x83` | STM32 -> Host | `ws2812` 狀態回傳格式 |
| `0x84` | STM32 -> Host | PID 設定狀態回傳格式 |
| `0x85` | STM32 -> Host | 左右輪 PID / encoder 回饋格式 |
| `0x86` | STM32 -> Host | 電源狀態（按鈕、關機請求、主電源） |
| `0x87` | STM32 -> Host | 保留：`FIRMWARE_INFO`（`mower_path_planning/firmware` 版的 build identity） |
| `0x88` | STM32 -> Host | MG996 servo 狀態 |
| `0x89` | STM32 -> Host | RS485 充電模組狀態（Vin / Vout / Iout / CC / CV） |
| `0x8A` | STM32 -> Host | 類比監控（主電池 / AON 小電池電壓、板溫、MG996 電流、VDDA） |
| `0x8F` | STM32 -> Host | `0x0F` 的 ack，送完立刻 reset |
| `0x10` ~ `0x14`, `0x90`, `0x91` | Host <-> bootloader | 只有 bootloader 會處理，app 會忽略；定義在 `BOOTLOADER.md` |

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
- 目前刀片馬達為單向：負值會被 STM32 視為 `0`（剎車），`0x82` 的 `applied_pwm` 回報 `0`

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
- linker script 把 app 程式碼放在 sector 2-6（`0x08008000` 起 `352KB`），sector 0-1 是 UART bootloader，最後 `128KB` 保留給設定；bootloader 更新 app 時不會碰 sector 7
- 不要高頻率寫入 PID；調參時先用 `persist_to_flash=0`，確認後再寫一次 Flash
- 寫 Flash 時 sector 7 erase 會讓 MCU 停約 1 秒（這段時間沒有 status frame），host 端等 `0x84` 的 `LAST_APPLY_OK` 要給 3 秒以上；韌體會先清掉 `FLASH_SR` 殘留的 error bits 再 erase（bootloader 跳進 app 不經 reset，殘留 bits 會讓第一次 erase 直接失敗），失敗會自動重試一次，結果在 `flash_diag`
- `0x04` 只帶 Kp/Ki/Kd；integral clamp 固定 ±80 rpm·s，但韌體會再把它縮到 `output_max / Ki`，所以大 Ki（自動校正常見 30–50）不會累積出超過滿輸出的積分

## `0x05` Power Command

Payload 長度固定 `4` bytes。

| Offset | Type | Field | Description |
|---|---|---|---|
| 0 | `uint8_t` | `action` | 見下表 |
| 1 | `uint8_t` | `reserved0` | 填 `0` |
| 2 | `uint16_t` | `reserved1` | 填 `0` |

| `action` | 名稱 | 意義 |
|---|---|---|
| `0x01` | `HOST_SHUTDOWN_ACK` | Host 收到關機請求、已開始 `poweroff`。STM32 收到後再等 `8 s`（grace）就切主電源 |
| `0x02` | `REQUEST_SHUTDOWN` | Host 主動要求關機（例如低電量），流程與長按按鈕相同 |
| `0x03` | `CANCEL_SHUTDOWN` | 取消尚未斷電的關機請求，回到 `RUNNING` |
| `0x04` | `FORCE_POWER_OFF` | 立刻切主電源，不做握手（Host 自己已經 halt 完才用） |

STM32 收到 `0x05` 後會立刻回一筆 `0x86`，`seq` 等於這筆命令的 seq。

## `0x07` Servo Command

MG996 servo（`PB10`，TIM10 中斷計時，50 Hz）。Payload 長度固定 `8` bytes。

| Offset | Type | Field | Description |
|---|---|---|---|
| 0 | `uint16_t` | `pulse_us` | 脈寬 `500 ~ 2500` µs（超出會被夾住）；`0` = 停止送脈波，servo 放鬆 |
| 2 | `uint16_t` | `hold_timeout_ms` | `0` = 一直保持到下一筆命令；非 `0` = 最後一筆命令後過這麼久就停止脈波（host 掛掉時機構放鬆） |
| 4 | `uint16_t` | `reserved0` | 固定 `0` |
| 6 | `uint16_t` | `reserved1` | 固定 `0` |

跟馬達命令一樣，`SHUTDOWN_PENDING / LIGHTS_OFF / LOW_POWER` 期間會被忽略；關機流程會直接 disable servo。ADC 電流限位觸發（`0x88 LIMIT_ACTIVE`）時脈波暫停，限位解除後自動恢復，不用重送命令。

角度換算由 host 做：`pulse_us = 500 + angle_deg / 180 * 2000`（MG996R 的實際端點請實測）。

## `0x0F` Enter Bootloader Command

Payload 長度固定 `8` bytes。

| Offset | Type | Field | Description |
|---|---|---|---|
| 0 | `uint32_t` | `magic` | 固定 `0xB007B007`，不符直接丟掉 |
| 4 | `uint32_t` | `reserved` | 目前固定填 `0` |

STM32 收到後：拉低 `PC13` 刀片剎車、關中斷、blocking 送出 `0x8F` ack、把 magic 寫進 RAM mailbox，然後 `NVIC_SystemReset()`。Reset 後由 bootloader 接手，所有馬達輸出都會停止。Host 收到 `0x8F` 之後（或 0.5 s 內沒收到也一樣）改送 bootloader 的 `0x10 BL_PING`。整個流程 `tools/mower_flash.py` 已經包好。

`0x8F` payload 固定 `4` bytes：`uint8_t status`（`0` = OK）+ 3 bytes reserved。

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
| 26 | `uint16_t` | `flash_diag` | Flash 儲存診斷（舊版固定 `0`）：低 byte = 最近一次失敗儲存的 HAL flash error code（`HAL_FLASH_ERROR_*`，`0` = 上次儲存成功，`0xFF` = unlock 失敗）；高 byte = 儲存前發現已經掛著的 `FLASH_SR` error bits（`OPERR/WRPERR/PGAERR/PGPERR/PGSERR`） |

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
| 12 | `int32_t` | `left_total_counts` | 左輪開機以來累積 encoder 計數（已含方向修正，正 = 前進），8896 counts = 輪子一圈 |
| 16 | `int32_t` | `right_total_counts` | 右輪開機以來累積 encoder 計數，同上 |
| 20 | `uint8_t` | `flags` | wheel controller flags |
| 21 | `uint8_t` | `reserved0` | 目前固定 `0` |
| 22 | `uint8_t` | `reserved1` | 目前固定 `0` |
| 23 | `uint8_t` | `reserved2` | 目前固定 `0` |

里程計用法：Host 端記住上一幀的 `*_total_counts`，用相減得到位移，不會因為 status 週期 (50ms) 比 control tick (20ms) 慢而漏計。int32 溢位約 24 萬圈後回捲，相減時用 32-bit 帶號運算即可正確處理。

位置換算：`wheel_angle_rad = total_counts * 2π / 8896`。

Wheel controller flags:

| Bit | Mask | Meaning |
|---|---|---|
| 0 | `0x01` | `OUTPUT_ENABLED`，目前安全條件允許輸出 |
| 1 | `0x02` | `CLOSED_LOOP`，目前使用 PID 閉環 |
| 2 | `0x04` | `FLASH_SETTINGS_VALID`，開機有讀到有效 Flash 設定 |
| 3 | `0x08` | `LAST_SAVE_OK`，最近一次 Flash 儲存成功 |

## `0x86` Power Status

每 `50ms` 送一次；另外在狀態改變時（長按觸發、收到 ack、切電、喚醒）會**立刻**多送一筆，延遲 ≤ 10 ms。Frame header 的 `seq` 等於最近一次接受的 `0x05` seq。

Payload 長度固定 `8` bytes。

| Offset | Type | Field | Description |
|---|---|---|---|
| 0 | `uint8_t` | `state` | 見下表 |
| 1 | `uint8_t` | `flags` | 見下表 |
| 2 | `uint8_t` | `shutdown_reason` | `0` 無、`1` 按鈕、`2` Host、`3` 強制 |
| 3 | `uint8_t` | `last_rx_seq` | 最近一次接受的 `0x05` seq |
| 4 | `uint16_t` | `press_ms` | 目前按鈕已按住的毫秒數，放開為 `0` |
| 6 | `uint16_t` | `shutdown_elapsed_ms` | 從關機請求起算的毫秒數，沒有請求時為 `0` |

`state`:

| 值 | 名稱 | 意義 |
|---|---|---|
| `0` | `RUNNING` | 正常運作 |
| `1` | `LOW_POWER` | 主電源已切（`PC15` low），STM32 由小電池供電，只等按鈕喚醒 |
| `2` | `WAKE_PULSE` | 剛喚醒，`PC14` 拉 high 1 s 通知 LubanCat 開機 |
| `3` | `SHUTDOWN_PENDING` | 已通知 Host 關機，等 ack / timeout；馬達全部停止、燈光進休眠動畫 |
| `4` | `LIGHTS_OFF` | 最後 0.9 s 燈光淡出，接著切電 |

`flags`:

| Bit | Mask | Meaning |
|---|---|---|
| 0 | `0x01` | `BUTTON_PRESSED` |
| 1 | `0x02` | `MAIN_POWER_ENABLED`，`PC15` 目前為 high |
| 2 | `0x04` | `SHUTDOWN_REQUESTED`，Host 看到這個 bit 就要開始關機並回 `0x05 action=1` |
| 3 | `0x08` | `HOST_ACK_RECEIVED` |
| 4 | `0x10` | `WAKE_ASSERTED`，`PC14` 目前為 high |

### 關機流程

1. 使用者長按電源鍵 `3 s`（或 Host 送 `0x05 action=2`）
2. STM32 立刻：輪子 / 割草馬達 / servo 全停、BTS7960 disable、BLD120A BRK 拉 low，進 `SHUTDOWN_PENDING`，馬上送出 `0x86`（`SHUTDOWN_REQUESTED=1`），燈光開始休眠動畫
3. Host 收到後回 `0x05 action=1`，然後執行 `systemctl poweroff`
4. STM32 收到 ack 後等 `8 s`；若一直沒有 ack，`30 s` 後也會繼續
5. 燈光淡出 `0.9 s`，`PC15` 拉 low 切主電源，進 `LOW_POWER`
6. `LOW_POWER` 時按住 `1 s`：`PC15` 回 high、`PC14` high 1 s、開機動畫重播

在 `SHUTDOWN_PENDING` / `LIGHTS_OFF` / `LOW_POWER` 期間，`0x01 / 0x02` 馬達命令與 `0x03` 燈光命令會被忽略。

`ros2/mower_hardware` 已實作步驟 3：收到 `SHUTDOWN_REQUESTED` 就回 ack 並執行 `shutdown_command` 參數（預設 `systemctl poweroff`，設空字串停用）。ros2_control 通常不是 root，需要 polkit 允許該使用者 `org.freedesktop.login1.power-off`，或把參數改成 `sudo -n systemctl poweroff` 並在 sudoers 放行。

## `0x89` Charger Status

STM32 每 `50ms` 送一次，資料來源是 `USART6`（PA11/PA12）接的 RS485 數控 30V5A 充電模組，STM32 每 `500ms` 用 Modbus RTU（站號 `0x01`、9600 8N1、FC03 讀 Reg0-4）輪詢一次，所以數值每 500 ms 才會更新。Frame header 的 `seq` 固定 `0`。主電源關閉（`0x86 MAIN_POWER_ENABLED=0`）時暫停輪詢。

Payload 長度固定 `16` bytes。

| Offset | Type | Field | Description |
|---|---|---|---|
| 0 | `uint16_t` | `vin_cv` | 充電模組輸入電壓，x0.01 V |
| 2 | `uint16_t` | `vout_cv` | 輸出電壓 = 電池端電壓，x0.01 V |
| 4 | `uint16_t` | `iout_ca` | 充電電流，x0.01 A |
| 6 | `uint16_t` | `set_cc_ca` | 模組設定的 CC 限流，x0.01 A |
| 8 | `uint16_t` | `set_cv_cv` | 模組設定的 CV 電壓，x0.01 V |
| 10 | `uint8_t` | `flags` | 見下表 |
| 11 | `uint8_t` | `comm_error_count` | 開機以來 RS485 timeout / CRC 錯誤次數，8-bit 回捲 |
| 12 | `uint16_t` | `age_ms` | 距離最近一次有效回應的毫秒數，`0xFFFF` = 開機後從未收到 |
| 14 | `uint8_t` | `last_exception_code` | 最近一次 Modbus exception code，`0` 無 |
| 15 | `uint8_t` | `reserved` | 固定 `0` |

`flags`:

| Bit | Mask | Meaning |
|---|---|---|
| 0 | `0x01` | `ONLINE`，最近的輪詢有正確回應（連續 3 次失敗後清除） |
| 1 | `0x02` | `CHARGING`，`iout >= 0.05 A` |
| 2 | `0x04` | `CV_PHASE`，`vout >= set_cv - 0.10 V`，代表進入恆壓／快充飽階段 |
| 3 | `0x08` | `INPUT_PRESENT`，`vin >= 5.00 V` |
| 4 | `0x10` | `EVER_SEEN`，開機後至少收到過一次有效回應 |

`ONLINE=0` 時 `vin/vout/iout/cc/cv` 是最後一次有效值（或全 `0`），Host 應以 `flags` 為準。
`CHARGING / CV_PHASE / INPUT_PRESENT` 只在 `ONLINE=1` 時才會被設定。

## `0x88` Servo Status

每 `50ms` 送一次；frame header 的 `seq` 等於最近一次接受的 `0x07` seq。Payload 長度固定 `8` bytes。

| Offset | Type | Field | Description |
|---|---|---|---|
| 0 | `uint16_t` | `pulse_us` | 目前目標脈寬 |
| 2 | `uint16_t` | `hold_timeout_ms` | 目前的 hold timeout |
| 4 | `uint16_t` | `command_age_ms` | 距離最近一筆 `0x07` 的毫秒數，飽和在 `0xFFFF` |
| 6 | `uint8_t` | `flags` | 見下表 |
| 7 | `uint8_t` | `last_rx_seq` | 最近一次接受的 `0x07` seq |

`flags`:

| Bit | Mask | Meaning |
|---|---|---|
| 0 | `0x01` | `ENABLED`，host 有要求輸出 |
| 1 | `0x02` | `LIMIT_ACTIVE`，電流限位中，脈波暫停 |
| 2 | `0x04` | `OUTPUT_ACTIVE`，脈波實際在輸出（= `ENABLED && !LIMIT_ACTIVE`） |
| 3 | `0x08` | `TIMED_OUT`，`hold_timeout_ms` 到期停掉了 |

## `0x8A` Analog Status

STM32 每 `50ms` 送一次，資料來源是 `PB1/ADC1_IN9` 經 4 通道 analog mux 讀到的四個慢速類比訊號（見 `wire.md`），STM32 每 `200ms` 掃一輪，所以數值每 200 ms 才會更新。每輪掃描前會先讀 `VREFINT` 算出實際 VDDA，所有換算都用這個值而不是假設 3.3 V。Frame header 的 `seq` 固定 `0`。

Payload 長度固定 `12` bytes。

| Offset | Type | Field | Description |
|---|---|---|---|
| 0 | `uint16_t` | `main_battery_cv` | 24 V 主電池電壓（`270k/33k` 分壓還原），x0.01 V；無效時 `0` |
| 2 | `uint16_t` | `aon_battery_cv` | 3.7 V AON 小電池電壓（`100k/300k` 分壓還原），x0.01 V；無效時 `0` |
| 4 | `int16_t` | `board_temp_dc` | 板溫 NTC（10k / beta 3950），x0.1 °C；無效時 `INT16_MIN` (`-32768`) |
| 6 | `uint16_t` | `vdda_mv` | 這輪換算用的 ADC 參考電壓，mV（未校正時 `3300`） |
| 8 | `uint16_t` | `mg996_current_raw` | MG996 電流感測原始 12-bit ADC 值 |
| 10 | `uint8_t` | `flags` | 見下表 |
| 11 | `uint8_t` | `reserved` | 固定 `0` |

`flags`:

| Bit | Mask | Meaning |
|---|---|---|
| 0 | `0x01` | `MAIN_BATTERY_VALID`，CH2 這輪讀取成功 |
| 1 | `0x02` | `AON_BATTERY_VALID`，CH3 這輪讀取成功 |
| 2 | `0x04` | `BOARD_TEMP_VALID`，CH1 讀取成功且 NTC 在合理範圍（開路 / 短路會清掉） |
| 3 | `0x08` | `MG996_CURRENT_VALID`，CH0 這輪讀取成功 |
| 4 | `0x10` | `VDDA_CALIBRATED`，`VREFINT` 讀取成功，`vdda_mv` 是實測值 |
| 5 | `0x20` | `MG996_LIMIT_ACTIVE`，電流超過限位 threshold（與 `0x88 LIMIT_ACTIVE` 同源） |

Host 應以 `flags` 為準：對應 bit 為 `0` 時該欄位沒有意義。分壓電阻比與 NTC 參數尚未實測校正，`main_battery_cv` 目前當作相對值使用。

## Status Flags

| Bit | Mask | Meaning |
|---|---|---|
| 0 | `0x01` | `COMMAND_VALID`，代表至少收過一筆有效命令 |
| 1 | `0x02` | `COMMAND_TIMEOUT`，代表命令超時，輸出已被切成 0 |
| 2 | `0x04` | `DRIVER_ALARM`，代表偵測到 motor driver alarm（目前只回報，不切輸出；BTS7960 `IS` 為類比電流感測，啟動電流大時會短暫拉高） |

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
- `0x02 / 0x82 / 0x03 / 0x83 / 0x04 / 0x84 / 0x85 / 0x05 / 0x86` 已接上 runtime path，host 可以直接依本文件封包格式對接

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
