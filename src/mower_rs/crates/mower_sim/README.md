# mower_sim

The robot's hardware for a development container (E8 design §4): a truth
world driving the STM32 base board, the WIT IMU and the u-blox F9P on ptys,
so the production stack runs against it with no change at all. How to start
it for the app: [`deploy/sim/README.md`](../../../../deploy/sim/README.md).

**Never on a robot.** It is not installed in the robot's image (not in
`MOWER_RS_BINARIES`; deploy/sim mounts a local build), it only creates ptys
and links them where the udev names would be, and it refuses to start when
one of those paths exists as anything but a link to a pty (on the robot they
are the real devices). No launch file, compose service or unit of the robot
starts it (a test checks the robot's files and the install list).

## Why at the device protocol

The drivers open device paths that are parameters already, so nothing in
mower_base, mower_imu or mower_gps changes, and everything from the arm
latch, 0x81 / 0x85, the telemetry JSON, lights, blade, servo, PID autotune
and the battery estimate to the UBX and WIT parsers runs as on the robot.
tools/fake_base.py and e2-verify's fake_sensors.py showed the layer works;
this is them in one process around one world, instead of an IMU and a GNSS
that follow `/odom` (the estimate) through rclpy.

## The world (`world.rs`)

- Wheels: fake_base.py's first-order lag (0.15 s) to `permille / 1000 *
  max_rpm`, integrated into revolutions and wrapping 32-bit counts.
- Body: a differential drive with mower_base's own wheel parameters
  (`geometry::Wheels`: its built-in production values under what
  `mower_rsd.yaml`'s `mower_base` section sets), so `/odom` is the truth;
  `--slip` loses a fraction of the travel on the ground.
- Pack: 6S, 10 Ah, charge drawn by 0.6 A idle + 1.2 A per wheel at full
  speed + 5 A of blade, voltage on mower_battery's OCV curve less 40 mOhm;
  starts resting at 24.3 V (85 %).

## The devices

| | link | what it does |
|---|---|---|
| STM32 (`stm32.rs`) | `/dev/stmcom` | fake_base.py's board on mower_base_core's frame builder and parser: 0x01 wheels with the board's own command timeout, 0x02 blade (with its timeout), 0x03 lights, 0x04 PID (a save stalls the board for the erase), 0x06 info, 0x07 servo; 0x85 + 0x81 every 50 ms, 0x86 / 0x89 / 0x82 / 0x88 / 0x83 / 0x84 / 0x87 every second; 0x87 is the image's bundled firmware (`MOWER_FIRMWARE_MANIFEST`), so `/robot/info` reports it up to date |
| IMU (`imu.rs`) | `/dev/imu_usb` | 0x51..0x54 ten times a second from the truth: heading, yaw rate and the body's acceleration, with the mount read from the URDF (`imu_link` upside down under a `base_link` turned +90 deg: base yaw = IMU yaw + 90 deg, yaw rate = -IMU z, the rule fake_sensors.py found) |
| F9P (`gnss.rs`) | `/dev/gps_rtk` | every CFG acknowledged, CFG-RATE and the NAV CFG-MSG rates followed; per epoch NAV-PVT, NAV-HPPOSLLH, NAV-RELPOSNED, NAV-EOE, NAV-SAT every 4th; the antenna's position (gps_link of the URDF) on WGS84 plus noise; RTK fixed (hAcc 14 mm), float, 3D or no fix |

Faults (`control.rs`), files in `--control-dir`: `base_mute`, `base_deaf`,
`sensors_mute` (a USB hub reset) and `gps_mode`.

## Tests

`cargo test -p mower_sim` (no ROS needed) feeds what the devices send to
the production parsers: mower_base_core's decoders and frame parser,
mower_ubx's message decoders and mower_gps's epoch assembler, and mower_imu's
WIT parser (the last two compiled from their own source files, as their
crates link r2r). It also checks the geometry against the repo's URDF and
mower_base's defaults against its source, the OCV table against
mower_battery's, the ptys end to end, and the refusal of real devices.
