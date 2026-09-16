# Bench tools

- `mower_uart.py` — send commands / decode every status frame over the host
  UART. `pip install pyserial`, then e.g.
  `./mower_uart.py /dev/cu.usbmodem1234 wheel 400 400 3`.
- `s3_uart_bridge/` — PlatformIO sketch turning an ESP32-S3 devkit into a
  USB-UART bridge (used when the CH340 adapter was dead). Wiring in
  `platformio.ini`.
- `mower_flash.py` — flash the app over the host UART through the bootloader
  (`BOOTLOADER.md`). `app-info` prints the running build's 0x87 identity;
  `sync IMAGE.bin` flashes only when that identity differs from the manifest
  next to the image (what the robot container runs at start-up).
- `fw_manifest.py` — writes that manifest; `make` at the firmware root calls it.
