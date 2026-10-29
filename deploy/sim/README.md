# The robot without the robot (`mower_sim`)

For working on the app (or anything that talks to the robot) at a desk: the
robot's production stack runs in its own runtime image, unchanged, against
simulated hardware. `mower_sim` plays the STM32 base board, the WIT IMU and
the u-blox F9P on ptys at `/dev/stmcom`, `/dev/imu_usb` and `/dev/gps_rtk`,
all three driven by one truth world: the wheels turn the encoders, the body
moves, the IMU and the GNSS antenna move with it, the pack drains. Joystick,
missions and nav2 drive a robot that really moves, and everything the app
sees is what the robot would publish: the same modules, the same topics, the
same bridge.

**Development only.** It is not in the robot's image, nothing on the robot
refers to it, the service mounts no host `/dev`, and both `sim-entry.sh` and
`mower_sim` refuse to start where the robot's devices are.

## Start

`mower_sim` is not part of the robot's image: build it once (and after a
change to it), for Linux on the image's platform. It needs no ROS, so any
Rust toolchain in a container does, e.g. from the repository root:

```bash
docker run --rm -v "$PWD/src/mower_rs":/src -w /src rust:1 \
  cargo build --release --locked -p mower_sim
```

(or `cargo build --release -p mower_sim` in `src/mower_rs` in the dev
container). The compose file mounts `src/mower_rs/target/release/mower_sim`;
`MOWER_SIM_BIN` names another build. Then:

```bash
docker compose -f deploy/sim/docker-compose.sim.yaml up
```

The app can connect about 15 s after `up` (the stack starts as on the
robot, nav2 included, with the image already pulled). It pulls
`docker.io/fxrbindi/mower_path_planning:main` if it is not there (log in to
Docker Hub while the repository is private); `IMAGE_TAG=v0.x.y` pins another
build. The image is built for linux/arm64, the robot's platform: it runs
natively on Apple silicon or an arm64 Linux machine.

Stop with Ctrl-C or `docker compose -f deploy/sim/docker-compose.sim.yaml
down`; the zones and sites you made stay in the `sim_state` volume (`down -v`
starts from the sample site again).

## Connect the app

- **iOS simulator on the same Mac**: direct IP `127.0.0.1`.
- **A phone on the LAN**: direct IP of this machine, and start with
  `SIM_HOST_IP=<that IP>` so the camera offers an address the phone can
  reach.

The gate is open (no `identity.json`, as on a development robot), so the
app's direct-IP mode connects with no pairing. `SIM_PAIRED=1` makes a dev
identity once (`MW-SIM001`), turns the gate on and prints the pairing URL
(the QR payload of `mower-pair`) in the log, for testing the pairing flow.

What the app gets:

| | from |
|---|---|
| heartbeat, robot info, telemetry, battery (~85 %, draining with use) | the production modules on the simulated board |
| pose on the map, RTK fix (14 mm), satellite basemap | the simulated F9P at 25.0330 N 121.5654 E (the NLSC basemap covers Taiwan only), the EKFs, navsat's datum |
| joystick | the real chain: manual guard, twist_mux, final guard, mower_base, the simulated board's wheels |
| zones, coverage, missions | a sample site on the first start (two zones around the robot joined by a channel, a risk zone far away); nav2 drives them |
| firmware "up to date" | the board reports the image's bundled firmware |
| front camera | MediaMTX with the repo's `mediamtx.yml`: a `testsrc2` pattern at `:8889/front/whep` |

The recorder page (`/mower_recorder/*`) stays empty: the recorder service is
not part of the sim.

## Faults and conditions

While the sim runs:

```bash
docker compose -f deploy/sim/docker-compose.sim.yaml exec sim touch /tmp/mower_sim/sensors_mute
```

| file in `/tmp/mower_sim` | while it exists |
|---|---|
| `base_mute` | the STM32's UART with both leads pulled: nothing either way (the board stops its wheels after 300 ms) |
| `base_deaf` | only the robot's TX lead out: the board keeps reporting, COMMAND_TIMEOUT |
| `sensors_mute` | a USB hub reset: the IMU and the F9P go silent (their drivers exit and come back) |
| `gps_mode` | its text: `fixed`, `float`, `3d` or `none` (`echo float > /tmp/mower_sim/gps_mode`) |

Start options go in `SIM_ARGS` (`mower_sim --help` in the image), e.g.
`SIM_ARGS="--gps-mode float --heading 30 --battery-v 22.5"`, or `--slip 0.05`
to make /odom drift from the truth. `--truth-log FILE` writes the true pose
at 10 Hz.

`mowerctl` (in images that have it) looks at the running stack:

```bash
docker compose -f deploy/sim/docker-compose.sim.yaml exec sim mowerctl topic hz /fix /robot/telemetry
```

## This tree's mower_rsd

To run a change to `src/mower_rs` against the sim, build in the dev
container, which has the image's platform and glibc, and add the override:

```bash
cd src/mower_rs && cargo build --release -p mower_rsd -p mower_sim
docker compose -f deploy/sim/docker-compose.sim.yaml -f deploy/sim/docker-compose.sim.local.yaml up
```

## When the map comes up turned

The stack's start-up race shows on a busy machine: navsat locks the datum 3 s
after the start, and if robot_state_publisher's `/tf_static` (the IMU's
mount) reaches the localize module later than that, the map EKF has not
taken one IMU heading yet and the map frame comes out turned by up to the
robot's heading (120 deg by default). Missions still run (the zones turn
with it), but the satellite underlay is off. `docker compose restart sim`
starts again. It is the production stack's own race, the same with the
Python fakes; E4, which brings robot_state_publisher into the process,
should remove it.

## What it is not

- Not Gazebo (the ROS package `src/mower_sim`, `make sim-gazebo`): that is
  another stack, without mower_base, mower_imu or mower_gps and with
  `/gps/fix`; this runs the robot's own.
- Not an acceptance test: USB hub resets, the LTE link, GNSS multipath,
  timing under the RK3568's load and the real board's firmware all differ.
  A change that drives the wheels still gets its supervised run on the robot.
- The IMU and GNSS models are those the production EKFs were run against
  closed loop (e2-verify's `fake_sensors.py`): a level IMU, an antenna
  position with noise (5 mm with RTK fixed), no multipath, no magnetometer
  model.
- On macOS, Docker has no host network: the camera's WebRTC media goes
  through the published UDP port 8189, which is what `SIM_HOST_IP` is for.
  If a phone still shows the placeholder, run the sim on a Linux machine.
