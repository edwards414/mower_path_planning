# Bench tools

- `mower_uart.py` — send commands / decode every status frame over the host
  UART. `pip install pyserial`, then e.g.
  `./mower_uart.py /dev/cu.usbmodem1234 wheel 400 400 3`.
- `s3_uart_bridge/` — PlatformIO sketch turning an ESP32-S3 devkit into a
  USB-UART bridge (used when the CH340 adapter was dead). Wiring in
  `platformio.ini`.
