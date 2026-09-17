# mower_hardware

ros2_control `SystemInterface` for the mower base: STM32F411 over UART using
the frame protocol in `../../firmware/UART_OPEN_LOOP_PROTOCOL.md`.

Tested against ros2_control on **Humble / Jazzy** API (`on_init(HardwareInfo)`,
`export_*_interfaces()` returning handles). If your distro is newer and the
build complains about deprecated exports, switch to the `on_export_*`
description-based API; the logic in `read()`/`write()` does not change.

## What it exposes

| Interface | Name | Unit | Source |
| --- | --- | --- | --- |
| command | `<joint>/velocity` | rad/s | sent as `0x01` permille (±1000 = ±`max_rpm`) |
| state | `<joint>/position` | rad | integrated from `0x85 total_counts` (wrap-safe) |
| state | `<joint>/velocity` | rad/s | `0x85 measured_rpm` |
| state | `mower_base/wheel_flags` | bits | `0x85 flags` |
| state | `mower_base/motor_flags` | bits | `0x81 flags` (bit 2 = driver alarm) |
| state | `mower_base/command_age_ms` | ms | `0x81` |
| state | `mower_base/crc_errors` | count | parser |
| state | `mower_base/feedback_age_s` | s | time since last `0x85` |
| state | `mower_base/firmware_version` | major·10⁶+minor·10³+patch | `0x87`, 0 until the first frame |
| state | `mower_base/firmware_git_sha32` | number | `0x87` |
| state | `mower_base/firmware_protocol` | number | `0x87` |

The `0x87` build identity is also published latched (transient_local) as JSON
on `/mower_base/firmware_info` (param `firmware_info_topic`, empty disables);
`/robot/info` merges it for the app.

The raw status frames (`0x81` motor, `0x83` lights, `0x84` PID gains, `0x85`
wheel feedback, `0x86` power) are republished together as one JSON message on
`/mower_base/telemetry` at `telemetry_rate_hz` (default 10 Hz, best-effort;
param `telemetry_topic` empty or rate 0 disables). `mower_mission`
`telemetry_node` folds it into `/robot/telemetry` for the parameter dashboard.

Two joints, left then right, in the order they appear in the URDF.

## Parameters (`<hardware><param>`)

| Param | Default | Notes |
| --- | --- | --- |
| `device` | `/dev/ttyS0` | Host UART wired to STM32 `PB6` (TX) / `PA10` (RX), common GND |
| `baud` | `115200` | firmware is fixed at 115200 8N1 |
| `max_rpm` | `58.0` | must equal firmware `wheel_max_rpm` |
| `counts_per_rev` | `8896` | FT-555 16 PPR × 4 × 139:1 |
| `command_timeout_ms` | `300` | firmware stops if no `0x01` within this |
| `feedback_timeout_s` | `0.5` | plugin zeroes velocity and warns if no `0x85` |

## Bring-up

```bash
# this package is part of the mower_path_planning workspace
colcon build --packages-select mower_hardware
source install/setup.bash

# minimal chassis + controllers
ros2 launch mower_hardware mower_base.launch.py device:=/dev/ttyS0

# drive
ros2 topic pub -r 20 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"

# watch
ros2 topic echo /diff_drive_controller/odom
ros2 control list_hardware_interfaces
```

Before trusting odometry, set `wheel_radius` and `wheel_separation` in
`config/mower_controllers.yaml` from the real chassis; the values there are
placeholders.

## Direction conventions

Firmware already handles the mirror-mounted left motor and encoder signs:
positive `0x01` permille = both wheels drive the vehicle forward, and
`total_counts` increase when driving forward. The plugin adds no further
inversion.

## Tests

`test/test_mower_protocol.cpp` covers CRC, frame build (byte-exact against a
frame captured from the bench tool), parser resync, CRC-error skipping, `0x85`
decoding, and the int32 wrap in the odometry difference. Runs under
`colcon test`; it has no ROS dependency.
