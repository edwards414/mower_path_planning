# mower_base_core

The ROS-free core of the Rust base driver — Phase B of
[`docs/ROS_FREE_PLAN.md`](../../../../docs/ROS_FREE_PLAN.md). Everything the
driver needs except the transport wrapper: no `r2r`, no ROS headers, so it
builds and tests with a plain `cargo test -p mower_base_core` on any machine.

| module | replaces | ported from |
|---|---|---|
| `protocol` | the STM32 UART frames | `src/mower_hardware/src/mower_protocol.cpp` + `include/mower_hardware/mower_protocol.hpp` |
| `protocol::FrameParser` | the byte-stream framing / resync | the same file, plus the `VMIN=0` read loop in `serial_port.cpp` / `mower_system.cpp::read()` |
| `limiter` | `control_toolbox::RateLimiter<double>` | `control_toolbox/include/control_toolbox/rate_limiter.hpp` (jazzy) — `diff_drive_controller::SpeedLimiter` is a thin wrapper around it |
| `odometry` | `diff_drive_controller::Odometry` + `rcpputils::RollingMeanAccumulator` | `ros2_controllers/diff_drive_controller/src/odometry.cpp` (jazzy) |
| `diff_drive` | one `diff_drive_controller` update cycle | `ros2_controllers/diff_drive_controller/src/diff_drive_controller.cpp` (jazzy), parameterised from `src/mower_hardware/config/mower_controllers.yaml` |
| `cycle::BaseCycle` | `mower_hardware::MowerSystem` read/write | `src/mower_hardware/src/mower_system.cpp` |
| `record` | a `ros2 bag` for the serial link | new |

## The shape of it

```rust
let (mut base, activation_tx) = BaseCycle::new(BaseConfig::default(), monotonic_ns())?;
// ... write activation_tx to the port ...
// the cmd_vel subscription: receive_command(twist, header_stamp, ros_now, monotonic_ns(),
//   cmd_vel_timeout) drops stale messages and keeps the newest one
loop {
    let n = port.read(&mut buf)?;              // VMIN = 0, may return 0
    base.on_rx(&buf[..n], monotonic_ns());
    let (tx, odom, joints) = base.tick(latest_cmd_vel.take(), monotonic_ns(), ros_now());
    if let Some(tx) = tx { port.write_all(&tx.bytes)?; }
    for event in base.take_events() { /* log it */ }
    // publish odom / joints / tf
}
```

`BaseCycle` is pure: it never reads a clock, never touches IO, and holds no
`Instant`. That is what makes the replay test below possible. It is given
two clocks and never mixes them: the monotonic **control clock** for every
period, age, deadline and throttle (controller_manager 4.48 runs ros2_control
on `RCL_STEADY_TIME`), and ROS time only for the header stamps of what it
returns — so a wall-clock step cannot pulse the wheels or stretch a dead-man.

Units and safety behaviour are documented on `cycle` itself; the short version:

* 0x85 `measured_rpm` → `rpm * 2π/60` rad/s; 0x85 `total_counts` → wrapped
  signed 32-bit difference × `2π/counts_per_rev` rad, accumulated.
* commanded rad/s → `round(clamp(rad_s * 60/2π / max_rpm * 1000, ±1000))`
  permille, sent with the firmware's own 300 ms `command_timeout_ms`.
* `cmd_vel_timeout` (0.5 s) zeroes the controller *reference*; the speed
  limiter then ramps the command to zero at `max_acceleration` — the ramp is
  the safety-zero deceleration, the command is never dropped in one step.
* `feedback_timeout_s` (0.5 s) without a 0x85 zeroes the reported joint
  velocities and holds the positions.
* `BaseCycle::fault()` models the `return_type::ERROR` paths: latched, stops
  commanding, emits the `send_stop()` burst every cycle.
* The **arm latch** (`BaseConfig::arm_latch`, on; not in the C++): after
  every activation, when 0x85 feedback that had been arriving stops for
  longer than `feedback_timeout_s` (both leads out), when two fresh 0x81 in
  a row report COMMAND_TIMEOUT once the cycle has been writing a 0x01 every
  tick for 1.5 board timeouts (the LubanCat TX lead alone, also when it was
  already out at activation), and when the board restarted (COMMAND_VALID
  cleared, or both encoder totals back at zero by an impossible jump), the
  cmd_vel reference is held at zero and `wheel_override` is not applied,
  each until its own stream shows a stop edge sustained for
  `latch_release_s` (0.5 s): no non-zero command in that window, explicit
  stops and silence counted together, silence alone for at least
  `cmd_vel_timeout` too; one zero followed by the stick again is not a
  release (the 2026-09-29 guard-zero finding) — and, after one of the three
  losses, the link is back both ways (feedback arriving, and a fresh 0x81
  showing the board receiving again; at activation too, once the board has
  sent a 0x81). The three losses also stop a running
  blade and hold it until its dead-man has been let go for the same window.
  Feedback that was *never*
  seen does not trip it. `BaseCycle::disarmed()` has the details; the C++
  chain simply never came back after an error. With the latch off none of
  it runs, the 0x81 checks and the blade included, which keeps the replay
  byte-identical.
* Transitions worth a log line (the ones `mower_system.cpp` and
  `diff_drive_controller` logged, plus the latch) come out of
  `BaseCycle::take_events()`.

## Tests

Everything here is checked against the original C++ rather than against
hand-written expectations.

```
cargo test -p mower_base_core
```

| test | vectors | checks |
|---|---|---|
| `tests/protocol_vectors.rs` | `vectors/protocol_oracle.json` | 9 CRC, 39 encode (byte-identical), 17 byte-stream parses, 29 decode (field-identical, incl. every wrong-length rejection) |
| `tests/diff_drive_vectors.rs` | `vectors/diff_drive_oracle.json` | rolling mean, 90 × 4 speed-limiter calls, 4 × 60 odometry steps, `updateFromVelocity` / `updateOpenLoop`, and a 200-cycle controller run — all to 1e-12; plus the cmd_vel subscription's stamp rules (zero stamp, stale, ageing from the stamp) and the open-loop odometry seed at activation |
| `tests/replay.rs` | `vectors/replay_synthetic.*` | a 300-cycle recording replayed through `BaseCycle` (latch off: the C++ has none): every tx byte identical, odometry to 1e-12 |
| `tests/safety.rs` | — | what `mower_base` adds, with the production controller parameters: the arm latch (activation under a live command, a stop by zero and by silence, feedback lost after presence vs. never seen, `wheel_override` held by the same rule; against a model of the firmware's command timeout and status batch: the TX-only pull with the stick held, zeros on the re-seat, re-arm only after the board is back *and* a stop edge, the C++ lurch with the latch off, a restart seen in the encoder totals / in COMMAND_VALID / after a real bootloader silence, both leads re-seated together (only the feedback loss, not the board's stale timeout reports; re-armed within three cycles) and only the STM32 TX one (the feedback alone re-arms nothing; pin 8 back 280 ms or 1 s later gets zeros), pin 8 already out at activation (0x03 and 0x02; also a zero, a push and the contact all inside the first 450 ms), nothing at activation, on one or a stale or garbled timeout report or after a stall of the loop, pid_autotune through a flash save, a blade held through a pull not restarting until let go; the release window: the robot's guard zero after a command path loss held and reported once (and, with the window off, the robot's 73 -> 366 permille ramp), a 0.28 s stall of the stream not a release, one zero then silence re-armed 0.52 s later, an override cancel and a blade 0 cut short by the stream), a wall-clock step that changes nothing but the stamps, the log events |

### Regenerating the vectors

`tests/oracle/` holds the harnesses. They link the *original* C++ —
`mower_protocol.cpp` straight out of this repo, `odometry.cpp` out of
ros2_controllers, `rate_limiter.hpp` out of control_toolbox — and stub only
`rclcpp::Time` (an int64 nanosecond count with `seconds() = ns / 1e9`) and
`rcpputils::RollingMeanAccumulator` (copied verbatim from the Jazzy install).
The per-cycle glue in `ddc_oracle.cpp` / `replay_gen.cpp` is transcribed from
`diff_drive_controller.cpp` and `mower_system.cpp`.

```sh
tests/oracle/generate.sh            # clones ros2_controllers + control_toolbox, rebuilds vectors/
```

Pinned upstream commits at the time of writing: ros2_controllers
`516a88be02b658effa34c5a39e8bb971cddab930`, control_toolbox
`71fbf3b6d8b643ef9380e3d4eef6bcb22984ed2b` (both `jazzy`).

## The record/replay format

A length-prefixed binary log, `record::encode` / `record::decode`:

```
magic  "MWRSER01"                8 bytes
record t_ns   i64 little endian  8 bytes   monotonic, same clock the replay feeds BaseCycle
       dir    0 = rx, 1 = tx     1 byte
       len    u32 little endian  4 bytes
       bytes  len bytes
```

`tests/vectors/replay_synthetic.mowerlog` is a synthetic 12 s capture from
`replay_gen`: 20 Hz 0x85 with a closed-loop wheel plant, 0x81/0x86/0x87/0x88/
0x89/0x82, reads split at arbitrary byte boundaries, injected line noise
(including a lone `0xA5` and a false `A5 5A` pair), one corrupted CRC, a 1 s
feedback blackout, a blade dead-man, a light request, a servo command, a raw
wheel override and a PID push, and a 0x86 SHUTDOWN_REQUESTED that must be
acked exactly once. `replay_synthetic.json` carries the tick schedule and the
expected odometry.

### Capturing a real one on the robot

The point of the format is that the same serial port cannot be opened twice,
so the Rust driver is verified offline against what `ros2_control` actually
did. **Neither method below changes ros2_control or the running stack.**

**A. `socat` pty tee (clean bytes, needs a restart of the container).**
Insert a pty pair in front of the real port and point the driver's `device`
parameter at it. Nothing in `mower_hardware` changes — only the device path.

```sh
# on the robot, before starting ros2_control
socat -x -v -d -d \
  PTY,link=/tmp/mower_base_tap,raw,echo=0,mode=666 \
  /dev/mower_base,raw,echo=0,b115200 2>/tmp/mower_base.socat &
# then run ros2_control with device:=/tmp/mower_base_tap
```

`socat -x -v` writes both directions with `> ` / `< ` markers and timestamps;
convert that to a `.mowerlog` with a short script (direction from the marker,
`t_ns` from the timestamp, bytes from the hex dump).

**B. `strace` on the live process (no restart, no reconfiguration).** This is
the one to use on a robot that is already running.

```sh
pgrep -f ros2_control_node                       # -> PID
fd=$(ls -l /proc/$PID/fd | grep -m1 mower_base | awk '{print $9}')   # the port's fd
strace -f -p $PID -e trace=read,write -e read=$fd -e write=$fd \
       -s 4096 -xx -tt -o /tmp/mower_base.strace
```

Each line gives the timestamp, the direction (`read` = rx, `write` = tx) and
the bytes as `\xAB` escapes; keep only the lines whose fd matches. Note that
`strace` slows the traced process down — the 25 Hz loop will report overruns,
so take short captures (a minute or two) and do it supervised, never during a
mowing run.

Either way the replay is the same as `tests/replay.rs`: feed the rx records
into `on_rx`, call `tick` on the recorded cadence, and compare against the tx
records. Real captures do not carry the cmd_vel that produced them, so record
`/cmd_vel` alongside (`ros2 bag record /cmd_vel /odom`) and drive the replay
from that; `/odom` from the same bag is then the odometry reference.
