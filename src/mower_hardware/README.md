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

The raw status frames (`0x81` motor, `0x82` blade motor, `0x83` lights,
`0x84` PID gains, `0x85` wheel feedback, `0x86` power, `0x88` lift servo,
`0x89` RS485 charger) are republished together as one JSON message on `/mower_base/telemetry`, one
per `0x85` frame (every 50 ms, so the PID auto-tune sees every sample;
`telemetry_rate_hz` only throttles when set below that; best-effort; param
`telemetry_topic` empty or rate 0 disables). Each message carries `t` (ROS
time, s) and `pid.flash_diag`, the `0x84` flash-save diagnostic word (0 unless
a persist failed). `mower_mission` `telemetry_node` folds it into `/robot/telemetry`
for the parameter dashboard; `battery_state_node` turns the `analog` +
`charger` objects into `/battery_state` (see `docs/BATTERY.md`).

`charger` = `{valid, online, current_present, input_present, voltage_v,
current_a, temp_c, reg3, reg4, flags, comm_errors, age_ms}` — the RS485
V/A/temp meter in the battery pack lead (it only measures, no CC/CV
settings; `current_a` is 0 until its shunt is wired in); values are the
last Modbus reply, meaningful only while `online`.
`servo` = `{valid, pulse_us, hold_ms, age_ms, flags, enabled, output,
limit_active, limit_up, limit_down, timed_out}` — the MG996 blade-lift
servo (`0x88`). `pulse_us` is the pulse on the wire, slewing towards the
last target; `limit_active` means the last target was cut short by a limit
microswitch and the servo backed off and stopped there.
`blade` = `{valid, cmd_permille, pwm, age_ms, flags, held}` — the BLD120A
blade motor (`0x82`); `held` is this driver's dead-man state (below).
(`0x8A` analog is retired: the ADC mux was never fitted.)

Five command side channels (JSON on `std_msgs/String`); the first two are
used by the `mower_mission` `pid_autotune_node`, servo / blade by Mower
Studio's 機構 / 割草 card, the rear light by `path_record_node` (app zone recording) and `mower_recorder`:

| Topic | Payload | Effect |
| --- | --- | --- |
| `/mower_base/pid_command` | `{"left":{"kp","ki","kd"},"right":{...},"persist":0\|1,"closed_loop":0\|1}` | one `0x04` per message; result shows up in telemetry `pid.flags` (`LAST_APPLY_OK`, `LAST_SAVE_OK`). `persist=1` writes STM32 flash, do not spam it |
| `/mower_base/wheel_override` | `{"left_permille":400,"right_permille":400,"ttl_ms":300}` | while the ttl (clamped to 1 s) has not expired, `write()` sends these permille instead of the controller's velocity command, so an open-loop / closed-loop step is a real step and not one shaped by `diff_drive_controller`'s acceleration limits. Keep re-publishing to hold it; it falls back to the controller on expiry |
| `/mower_base/servo_command` | `{"pulse_us":1500,"hold_ms":0}` | one `0x07` per message. `pulse_us` 500–2500 is a target the STM32 slews to at 25 µs/10 ms, stopping (and backing off 50 µs) when a limit microswitch trips; `0` releases the servo. `hold_ms` 0 = hold until the next command, else pulses stop that long after the command |
| `/mower_base/blade_command` | `{"permille":300,"ttl_ms":500}` | dead-man: while the ttl (clamped to 1 s) has not expired `write()` re-sends `0x02` every cycle with the STM32 command timeout; on expiry (or `permille` 0) one explicit stop goes out. A dead publisher, a dropped link or a stalled controller manager all stop the blade within ttl + `command_timeout_ms`. Keep re-publishing (e.g. 5 Hz, ttl 500) to hold it |
| `/mower_base/rear_light` | `{"effect":"recording"\|"off","source":"path_record"}` | sets the `0x03` `overlay` byte (`REAR_RECORDING`): a red breath on the back strip only, on top of whatever `/mower_base/led_command` asked for (front untouched). Kept per `source` (`path_record` = zones from the app, `bag` = `mower_recorder`): on while any source is recording, and one source's "off" clears only itself. Each source needs a refresh within `rear_light_timeout_s` or it drops, so a dead publisher cannot leave it on; both recorders re-send every 2 s. Telemetry `led.flags` bit `0x08` confirms the firmware applied it |

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
| `telemetry_rate_hz` | `20.0` | `/mower_base/telemetry` rate, 0 disables |
| `pid_topic` / `override_topic` | `/mower_base/pid_command` / `/mower_base/wheel_override` | empty disables the side channel |
| `rear_light_topic` / `rear_light_timeout_s` | `/mower_base/rear_light` / `6.0` | empty disables; overlay drops this long after the last `recording` |

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
