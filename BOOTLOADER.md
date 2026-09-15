# UART Bootloader

讓 host（LebanCat / ROS2 端或任何接在 `USART1` 上的電腦）不用 ST-Link，直接透過現有的 UART 更新 STM32 應用程式。

- Bootloader 原始碼：`bootloader/`（獨立 Makefile 專案，不在 CubeIDE 的 app build 裡）
- 共用常數：`Module/Inc/boot_shared.h`（flash 分區、RAM mailbox、frame type、payload 格式）
- Host 端工具：`tools/mower_flash.py`（Python 3 + pyserial）
- App 端進入點：`Module/Src/uart_interface.cpp` 的 `0x0F` handler

Bootloader 本身只能用 ST-Link 燒，燒一次之後就不用再碰 ST-Link。

## Flash 分區

STM32F411CE 512 KB internal flash：

| Sector | 位址 | 大小 | 用途 |
| --- | --- | ---: | --- |
| 0 | `0x08000000` | 16 KB | bootloader（目前約 6 KB） |
| 1 | `0x08004000` | 16 KB | bootloader 預留 |
| 2 | `0x08008000` | 16 KB | application 起點（vector table） |
| 3 | `0x0800C000` | 16 KB | application |
| 4 | `0x08010000` | 64 KB | application |
| 5 | `0x08020000` | 128 KB | application |
| 6 | `0x08040000` | 128 KB | application（上限 352 KB） |
| 7 | `0x08060000` | 128 KB | PID 設定（`settings_storage`），bootloader 永遠不碰 |

RAM：`0x20000000` 開頭 32 bytes 是 reset 後不會被清掉的 mailbox，兩邊 linker script 都把它排除在 `RAM` 之外。App 要進 bootloader 時把 `BOOT_REQUEST_MAGIC`（`0xB007B007`）寫進去再 system reset；bootloader 開機看到 magic 就清掉並留在下載模式。

## 開機流程

```
power-on / reset
   │
   ▼
bootloader (0x08000000, HSI 16 MHz, USART1 polling)
   │
   ├─ mailbox == BOOT_REQUEST_MAGIC ──────────────► 留在 bootloader
   ├─ app vector table 不合法（SP/PC 檢查）────────► 留在 bootloader
   └─ 否則等 500 ms（BOOT_GRACE_PERIOD_MS）
        ├─ 收到 BL_PING (0x10) ───────────────────► 留在 bootloader
        └─ 沒收到 ──────────────────────────────► 跳到 app (0x08008000)
```

Bootloader 期間：`PC13`（BLD120A BRK）拉低 = 刀片剎車、板載 LED 亮；`PA4-PA7` BTS7960 EN 拉低。跳 app 前會把 UART、RCC、SysTick、NVIC 全部還原成 reset 狀態，再設 `SCB->VTOR`、MSP 並跳到 app 的 Reset_Handler。`HAL_DeInit()` 會把所有 GPIO 重設成浮接，所以跳轉前會再拉一次 BRK / EN，讓刀片剎車一路保持到 app 的 `MX_GPIO_Init`。

500 ms 的 grace window 是救援用：如果 app 壞到 UART 都跑不起來，host 只要在上電後 0.5 s 內連續送 PING 就能攔在 bootloader。

## Host 更新流程（`tools/mower_flash.py`）

```bash
pip install pyserial          # 一次
python3 tools/mower_flash.py -p /dev/ttyUSB0 flash Debug/mower_robot_firmware.bin
```

工具做的事：

1. 送 `BL_PING`；沒回應代表 app 在跑，改送 `0x0F ENTER_BOOTLOADER`，等 app 回 `0x8F` 後重開，再 PING 直到收到 `0x90 BL_INFO`。
2. `BL_ERASE(image_size)`：只擦 image 會用到的 sector（60 KB 的 app 只擦 sector 2-4，約 1-2 s）。
3. `BL_WRITE(offset, data)`：每次 128 bytes，stop-and-wait，每包等 ACK。**offset 0（vector table）最後才寫**，中途斷線的話 reset vector 還是 `0xFFFFFFFF`，bootloader 開機會判定 app 無效、留在 bootloader，不會跳進半個 image。
4. `BL_VERIFY(image_size, crc32)`：bootloader 直接對 flash 算 CRC-32（zlib 同款），不符就回 `CRC_MISMATCH`。
5. `BL_RUN_APP`：跳到 app。加 `--no-run` 可以留在 bootloader。

其他子命令：`info`（讀 bootloader 狀態）、`enter`（重開進 bootloader 並停住）、`run`（從 bootloader 跳 app）。

注意：

- 要餵 `.bin`，不是 `.hex`/`.elf`。CubeIDE Debug/Release build 都會產生 `mower_robot_firmware.bin`。
- 工具會檢查 image 前 8 bytes（SP/PC）確定是 link 在 `0x08008000` 的 app，舊 layout 的 bin 會被拒絕。
- 更新前先停掉 ROS2 node 或其他佔用 serial port 的程式。
- 60 KB 大約 10 秒（115200 baud、stop-and-wait）。

## UART frame

Framing 跟 `UART_OPEN_LOOP_PROTOCOL.md` 完全一樣（`A5 5A`, version `0x01`, type, seq, len, payload, CRC-16/CCITT-FALSE），所以 host 可以沿用同一套 parser。差別只有 bootloader 端接受的 payload 上限是 `4 + 128` bytes（app 端是 32）。

### App 端

| Type | 方向 | Payload | 說明 |
| --- | --- | --- | --- |
| `0x0F` | Host → app | `u32 magic (0xB007B007)`, `u32 reserved` | 要求重開進 bootloader。magic 不對直接丟掉 |
| `0x8F` | app → Host | `u8 status`, `u8[3] reserved` | 回 ack 後立刻 reset。這包是關中斷 blocking 送的，正常會收到，但 host 不該依賴它 |

### Bootloader 端

| Type | 方向 | Payload | 說明 |
| --- | --- | --- | --- |
| `0x10` `BL_PING` | Host → BL | 無 | 回 `0x90`。也用來在 grace window 裡攔住 bootloader |
| `0x11` `BL_ERASE` | Host → BL | `u32 image_size` | 擦 sector 2 起、足夠放 image 的 sector。回 `0x91` |
| `0x12` `BL_WRITE` | Host → BL | `u32 offset` + `data[4..128]` | offset 相對 `0x08008000`，offset 和長度都要 4 的倍數，必須先 ERASE。寫完會 read-back 驗證。回 `0x91` |
| `0x13` `BL_VERIFY` | Host → BL | `u32 image_size`, `u32 crc32` | 對 flash 算 CRC-32 比對，成功順便重算 app_valid。回 `0x91` |
| `0x14` `BL_RUN_APP` | Host → BL | 無 | app 合法就回 `0x91 OK` 然後跳 app；不合法回 `NO_VALID_APP` |
| `0x90` `BL_INFO` | BL → Host | `u32 app_start`, `u32 app_max_size`, `u16 write_chunk_max`, `u8 bl_version`, `u8 flags`, `u32 reserved` | flags: bit0 `APP_VALID`, bit1 `ENTERED_BY_REQUEST`, bit2 `ERASED` |
| `0x91` `BL_ACK` | BL → Host | `u8 command`, `u8 status`, `u32 value` | `command` = 被 ack 的 type；`value`：WRITE 回 offset、VERIFY 回算出的 crc32、ERASE 回 image_size |

Status 碼：`0 OK`, `1 BAD_ARGUMENT`, `2 FLASH_ERROR`, `3 CRC_MISMATCH`, `4 OUT_OF_RANGE`, `5 NOT_ERASED`, `6 NO_VALID_APP`。

Bootloader 會忽略所有 app 的 frame type（`0x01-0x04`），app 也會忽略 `0x10-0x14`，所以 host 送錯狀態不會出事，只是沒回應。

## 第一次部署（需要 ST-Link）

1. 建 bootloader：

   ```bash
   cd bootloader && make
   ```

   Makefile 預設用 STM32CubeIDE 內附的 `arm-none-eabi-gcc`（macOS 路徑 `/Applications/STM32CubeIDE.app/.../gnu-tools-for-stm32.*/tools/bin`）。不在 mac 上就 `make TOOLCHAIN_BIN=/path/to/bin` 或把工具鏈放進 PATH 後 `make TOOLCHAIN_BIN=`。

2. 燒 bootloader 到 `0x08000000`：

   ```bash
   make flash                     # st-flash --reset write build/bootloader.bin 0x08000000
   ```

   或用 STM32CubeProgrammer 手動燒 `bootloader/build/bootloader.bin` 到 `0x08000000`。

3. 用 CubeIDE 正常 build app（linker script 已經改成 `0x08008000` 起），然後：
   - 第一次可以直接用 CubeIDE Debug 燒（它會依 elf 位址燒到 `0x08008000`，不會蓋掉 bootloader），或
   - 直接用 `tools/mower_flash.py flash`。

之後每次改 app 都走 `mower_flash.py`。

## App 端為了 bootloader 改了什麼

| 檔案 | 改動 |
| --- | --- |
| `STM32F411CEUX_FLASH.ld` | `FLASH` 改為 `0x08008000` 起 352 KB；新增 `BOOTLOADER` 與 `BOOT_SHARED` 區域；`RAM` 從 `0x20000020` 起 |
| `Core/Src/system_stm32f4xx.c` | 打開 `USER_VECT_TAB_ADDRESS`，`VECT_TAB_OFFSET` 預設 `0x8000`，`SystemInit()` 會設 `SCB->VTOR` |
| `Core/Src/main.c` | `USER CODE BEGIN 1` 再設一次 `SCB->VTOR = BOOT_APP_START_ADDRESS`，防 CubeMX 重新產生 `system_stm32f4xx.c` 時失效 |
| `Module/Src/uart_interface.cpp` | 新增 `0x0F` handler：剎車、關中斷、blocking 送 `0x8F`、寫 mailbox、`NVIC_SystemReset()` |
| `Module/Inc/boot_shared.h` | 新增，兩邊共用 |

CubeMX 重新產生程式碼時要檢查：`system_stm32f4xx.c` 的 `USER_VECT_TAB_ADDRESS` 會被還原（`main.c` 那行有補救），linker script 通常不會被動。

## Debug 注意事項

- 用 CubeIDE 對 app 下 debug 時，reset 後會先跑 bootloader 的 500 ms grace window 再進 app；斷點設在 `main()` 前面會看到這段延遲，正常。
- 如果想在 CubeIDE 裡連 bootloader 一起 debug，開 `bootloader/build/bootloader.elf` 作為第二個 symbol file。
- Bootloader 跑在 HSI 16 MHz、沒有 PLL；app 的 `SystemClock_Config()` 從 reset 狀態重新設 HSE + PLL，`HAL_RCC_DeInit()` 在跳轉前已把 RCC 還原。
- 若 app 用 CubeIDE 燒到 `0x08008000` 但 bootloader 沒燒，MCU 從 `0x08000000` 開機會是空的 flash，什麼都不會跑。兩個都要在。
