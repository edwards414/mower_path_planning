# WS2812 LED Command Modes

這份文件是從 STM32 韌體目前的實作整理出來的，給 ROS 端寫 `joy` 控制器直接參考。

程式依據:

- `Module/Inc/uart_interface.hpp`
- `Module/Src/uart_interface.cpp`
- `Module/Inc/ws2812.h`
- `Core/Src/usart.c`

## 快速結論

- LED command frame type: `0x03`
- LED status frame type: `0x83`
- UART: `115200 8N1`
- Protocol version: `0x01`
- Endianness: little-endian
- 兩條燈條都會一起更新
- 每條燈條 LED 數量: `16`
- 規劃保留 LED index `0-2` 作系統狀態燈，但目前尚未實作獨立狀態燈 mode
- LED command 沒有 timeout 機制，最後一次收到的效果會一直維持

完整 UART frame、CRC 與其他 motor command 可另外看 [UART_OPEN_LOOP_PROTOCOL.md](UART_OPEN_LOOP_PROTOCOL.md)。

## `0x03` WS2812 Command Payload

Payload 固定 `8` bytes。

| Offset | Type | Field | 說明 |
|---|---|---|---|
| 0 | `uint8_t` | `mode` | 燈效模式 |
| 1 | `uint8_t` | `r` | 紅色 `0~255` |
| 2 | `uint8_t` | `g` | 綠色 `0~255` |
| 3 | `uint8_t` | `b` | 藍色 `0~255` |
| 4 | `uint16_t` | `effect_period_ms` | 動畫步進週期 |
| 6 | `uint8_t` | `reserved0` | 固定填 `0` |
| 7 | `uint8_t` | `reserved1` | 固定填 `0` |

## Mode 一覽

| Mode | 名稱 | 類型 | ROS 端應帶欄位 | 韌體實際行為 |
|---|---|---|---|---|
| `0x00` | `CLEAR` | 靜態 | `mode` 即可 | 清掉前後燈所有 LED，然後立刻顯示 |
| `0x01` | `ALL_ON` | 靜態 | `mode + r/g/b` | 前後燈全部 `16` 顆都設成同一顏色 |
| `0x02` | `FLOW` | 動畫 | `mode + r/g/b + effect_period_ms` | 每次步進只亮同一個 index 的 1 顆，index `0 -> 15` 循環 |
| `0x03` | `TURN_LEFT` | 動畫 | `mode + effect_period_ms`，可選 `r/g/b` | 每次步進亮 2 顆，主亮點往左轉方向循環，尾巴半亮 |
| `0x04` | `TURN_RIGHT` | 動畫 | `mode + effect_period_ms`，可選 `r/g/b` | 每次步進亮 2 顆，主亮點往右轉方向循環，尾巴半亮 |
| `0x05` | `SHOW` | 靜態 | `mode` 即可 | 只把目前 buffer 再送一次，不會改顏色、不會改 pattern |

## 每個 Mode 的細節

### `0x00` `CLEAR`

- `r/g/b/effect_period_ms` 都可忽略
- 韌體會先清 buffer，再呼叫 `show`
- 適合用在「關燈」或重置視覺狀態

建議 payload:

```text
mode=0x00, r=0, g=0, b=0, effect_period_ms=0
```

### `0x01` `ALL_ON`

- 所有 LED 都會設成同一個 RGB
- `effect_period_ms` 不使用
- 適合常亮狀態，例如待命、已連線、錯誤提示

建議 payload:

```text
mode=0x01, r=255, g=255, b=255, effect_period_ms=0
```

### `0x02` `FLOW`

- 每 `effect_period_ms` 更新一次
- 每次只會亮 1 顆 LED
- 前燈與後燈會亮同一個 index
- index 會從 `0` 跑到 `15`，然後回到 `0`
- 如果 `effect_period_ms = 0`，韌體會自動改成 `100 ms`

建議 payload:

```text
mode=0x02, r=0, g=0, b=255, effect_period_ms=80
```

### `0x03` `TURN_LEFT`

- 每 `effect_period_ms` 更新一次
- 每次會亮 2 顆，主亮點是完整亮度
- 尾巴是主亮點前一顆的半亮
- 前燈與後燈都套用同樣 pattern
- 如果 `r=g=b=0`，韌體會自動改成預設方向燈色 `255,120,0`
- 如果 `effect_period_ms = 0`，韌體會自動改成 `100 ms`

建議 payload:

```text
mode=0x03, r=0, g=0, b=0, effect_period_ms=100
```

### `0x04` `TURN_RIGHT`

- 每 `effect_period_ms` 更新一次
- 每次會亮 2 顆，主亮點是完整亮度
- 尾巴是主亮點後一顆的半亮
- 前燈與後燈都套用同樣 pattern
- 如果 `r=g=b=0`，韌體會自動改成預設方向燈色 `255,120,0`
- 如果 `effect_period_ms = 0`，韌體會自動改成 `100 ms`

建議 payload:

```text
mode=0x04, r=0, g=0, b=0, effect_period_ms=100
```

### `0x05` `SHOW`

- 這個 mode 不會重新建立圖樣
- 它只會把「目前記憶體裡已經存在的 LED buffer」再送一次
- 如果 ROS 端只是用 UART `0x03` 在控燈，通常不需要主動使用 `SHOW`
- 比較像是底層 buffer 操作後的 flush，不是一般 `joy` 模式切換用的 command

## 系統狀態燈設計

這部分是燈號規格，尚未在目前 UART WS2812 command 裡實作。硬體上不新增 GPIO，直接使用既有前後 WS2812 燈條；前燈與後燈同一個 index 同步顯示。

| 狀態燈 | WS2812 index | 顏色/狀態 | 代表意義 |
|---|---|---|---|
| 第一顆 | `0` | 綠燈常亮 | LebanCat 4G 正常連線，且有網路 |
| 第二顆 | `1` | 綠燈常亮 | UART 通訊正常運作 |
| 第三顆 | `2` | 黃燈閃爍 | LebanCat 正在休眠 |
| 第三顆 | `2` | 黃燈常亮 | ROS2 正常運作中 |

第三顆黃燈同時有休眠與 ROS2 運作狀態時，建議以黃燈閃爍優先。
一般燈效或方向燈若仍要使用整條燈條，應避開或在每次更新後重畫 LED index `0-2`，避免覆蓋系統狀態。

建議之後新增一個 mode，例如：

```text
UART_WS2812_MODE_STATUS = 0x06
```

目前 `0x03` WS2812 command payload 只有 `mode + r/g/b + effect_period_ms`，不足以同時表達 4G、UART、LebanCat sleep、ROS2 running 這幾個獨立狀態。若要正式實作，建議把 `reserved0/reserved1` 改成狀態 bitmask，或新增專用 payload/frame。

## ROS `joy` 控制器實作建議

這部分不是韌體硬規定，而是依目前 mode 行為比較適合的用法。

### 1. 靜態模式用 edge trigger

適合:

- `CLEAR`
- `ALL_ON`
- `SHOW`

建議做法:

- 按一下送一次 command 即可
- 每次送都把 `seq` 加 `1`

原因:

- 韌體對靜態模式只會在「第一次套用」或「`seq` 改變」時重新執行
- 如果你連續送 `ALL_ON` 但 `seq` 沒變，顏色更新可能不會重新套用

### 2. 方向燈模式用 hold trigger

適合:

- `TURN_LEFT`
- `TURN_RIGHT`

建議做法:

- 按住時持續送對應 mode
- 放開後送一個 fallback mode，例如 `ALL_ON` 或 `CLEAR`
- 發送頻率其實不用很高，`10~20 Hz` 就夠

原因:

- LED 本身沒有 timeout
- 如果不送 fallback mode，最後一次方向燈效果會一直跑下去

### 3. 動畫速度直接用 `effect_period_ms`

參考值:

- `60~80 ms`: 比較快
- `100 ms`: 中等，最安全
- `120~180 ms`: 比較慢

### 4. `seq` 一律遞增

建議:

- 每送一包 `0x03` 就把 `seq` 加 `1`
- `uint8_t` overflow 直接回到 `0` 即可

這樣最好 debug，也能保證靜態模式一定會被重新套用。

## `0x83` WS2812 Status Payload

Payload 固定 `8` bytes。

| Offset | Type | Field | 說明 |
|---|---|---|---|
| 0 | `uint8_t` | `mode` | STM32 目前保存的 mode |
| 1 | `uint8_t` | `r` | STM32 目前保存的紅色 |
| 2 | `uint8_t` | `g` | STM32 目前保存的綠色 |
| 3 | `uint8_t` | `b` | STM32 目前保存的藍色 |
| 4 | `uint16_t` | `effect_period_ms` | STM32 目前保存的動畫週期 |
| 6 | `uint8_t` | `flags` | 狀態 bit flags |
| 7 | `uint8_t` | `last_rx_seq` | 最近一次成功接收的 seq |

狀態送出週期:

- 每 `50 ms` 一次

`flags` 目前只有一個 bit 真的有用:

| Bit | Mask | 意義 |
|---|---|---|
| 0 | `0x01` | `COMMAND_VALID`，代表至少收過一筆有效 LED command |

對 WS2812 來說，目前不會出現:

- `COMMAND_TIMEOUT`
- `DRIVER_ALARM`

也就是說 LED status 常見值只有:

- `0x00`: 還沒收過有效 LED command
- `0x01`: 已經收過有效 LED command

## ROS 端需要特別注意的坑

### `SHOW` 不是「顯示這次 payload 顏色」

`SHOW` 不會用 `r/g/b` 改 buffer，它只是把舊 buffer 再刷一次。
如果 ROS 端只透過 `0x03` 控燈，通常可以不用這個 mode。

### LED command 不會自己過期

motor command 有 timeout，但 LED command 沒有。
所以 `TURN_LEFT` 或 `TURN_RIGHT` 一旦送出去，如果沒有下一筆新 command 蓋掉，它會一直跑。

### `effect_period_ms = 0` 會被改成 `100`

這個規則對動畫模式很重要。
如果 ROS 端想要明確速度，請直接帶非 0 的值。

### 不支援的 mode 不會報錯，但也不會有燈效

韌體目前沒有對 mode 做白名單 reject。
如果送了未定義數值，STM32 會記住這個 mode 並回在 status 裡，但 LED 不會有對應行為。

## 最推薦給 `joy` 的最小控制集合

如果你要先快速做一版，我會建議只先接這 4 個:

| 用途 | Mode | 推薦參數 |
|---|---|---|
| 關燈 | `CLEAR` | `mode=0x00` |
| 常亮白燈 | `ALL_ON` | `r=255 g=255 b=255` |
| 左方向燈 | `TURN_LEFT` | `r=0 g=0 b=0 effect_period_ms=100` |
| 右方向燈 | `TURN_RIGHT` | `r=0 g=0 b=0 effect_period_ms=100` |

這樣 ROS 端先把 `joy` 邏輯寫起來會最快，之後再加 `FLOW` 或其他常亮顏色就好。
