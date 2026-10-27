# mower_rs/tools

Verification harnesses. The `*_compare.py` ones run a Python/C++ node and its
Rust replacement against the same fakes and diff every message; the three at
the bottom drive an A/B measurement on the real robot.

| tool | what it does |
|---|---|
| `shadow_compare.py` | separate mower_rs binaries vs one `mower_rsd` process, every topic compared (Phase A5) |
| `fake_base.py`, `base_harness.py`, `base_compare.py`, `base_ab.sh` | a fake STM32 on a pty, then the `ros2_control` chain vs `mower_base` against it (Phase B); `base_ab.sh` runs one side in the runtime image under the robot's CycloneDDS, `base_compare.py` exits 1 on any endpoint QoS difference and on a failed scenario check (mower_base moving after a re-seat or under a pre-restart stream, not following the stick after the release, autotune not saving or not driving after the save, or the C++ side not reproducing the fault), `--scenario pull` / `pullpush` are the cable pull (both leads) with the stick pushed before / during it, `--scenario txpull` the LubanCat TX lead alone (the fake goes deaf but keeps sending, 0x81 COMMAND_TIMEOUT), `--scenario txstart` that lead already out when the driver starts (the fake never reached, 0x81 0x02 throughout; idle, then pushed while still out, held through the re-seat), `--scenario live` a restart under a stream that was already running (run it with `--cpus=1 -e LOAD=8` too: discovery latency is what it tests), `--scenario autotune` `mower_pid_autotune` through a run with a flash save (the fake stalls like the sector erase) and a second run after it (recipe in `../README.md`, mower_base "Verification") |
| `localize_compare.py` | `dual_ekf_navsat.launch.py` vs `mower_localize` on one synthetic sensor stream (Phase C); `--yaw0 --imu-start --launch` replays the robot's start-up (heading far from east, `/odom` before the IMU, stack started with the stream) so navsat's `delay` is exercised |
| `datum_check.py` | **before a GPS run**: asks navsat (`/toLL`, `/fromLL`) whether the `map -> utm` datum is rotated, which it is by 10-15 degrees, in both stacks, when `/odom` reached the map EKF before the IMU (the usual start-up order with `rust_base`); `--fix` re-sets it at the same origin via `/datum` |
| `slow_copy_check.py` | a source topic and its throttled copies (`/odom` -> `/odom_slow`, `/odometry/global` -> `/odometry/global_slow`): rate, spacing, byte-identity with a source message, publisher QoS; run a `topic_tools throttle` on the same source next to the module's copy to compare (`../README.md`, "The slow copies") |
| `coverage_cancel_check.py` | the coverage node's correlated-cancel tracking, black box: the real `mower_coverage` binary against a fake navigation side (action server, `/cancel_navigation_dispatch`, `/confirm_navigation_dispatch`, maps); four timed scenarios (2 s retry sends one request, a late-accepted goal shares the 3 s request, retry only after the 5 s request timeout, nothing after a terminal result or after SIGINT), plus `/resume_coverage` against a fake `/coverage_progress_status` checkpoint. Run by CI's mower_rs tests job: `python3 tools/coverage_cancel_check.py --binary target/debug/mower_coverage` |
| `nav_progress_check.py` | `/coverage_progress` and `/coverage_progress_status` of a navigation server, black box: the server (`mower_nav` or the rclpy `nav_action_server`, given as the command line) against a fake Nav2 (lifecycle state, `navigate_to_pose`, `follow_path` with a falling `distance_to_goal`); a 3-segment run to success, a cancel halfway through segment 2, a restart that restores the checkpoint, rejected resume goals and a resume at segment 2. `--restart-as` restarts into the other server, so the checkpoint crosses implementations; CI's mower_rs tests job runs it both ways: `python3 tools/nav_progress_check.py --restart-as "python3 -m mower_mission.navigation.nav_action_server" -- target/debug/mower_nav` |
| `nav_core_check.py` | `mower_nav`'s in-process navigation (`nav_backend:=nav_core`, `nav_core.pose_source:=tf`, no Nav2), black box: the real binary against both maps (a 10 x 6 m free field with a lethal border), `map -> odom` and `odom -> base_footprint` on `/tf` plus `/odom` at 50 Hz from a kinematic robot that follows `/nav_cmd_vel`. It first checks the in-process stack is on the graph, then: no map, so the mission fails after `nav2_ready_timeout_s` with the readiness message; a 3-segment run to success (progress 3/3, `/plan`, the robot stops at the last pose, `/nav_cmd_vel` stamps never step back); a cancel in segment 1 (CANCELED, `/nav_cmd_vel` zero within 0.5 s); a wall in `/map_grid` only, so the mission fails with `Nav2 task error: code=104` after the tree's recoveries (~35 s). Run by CI's mower_rs tests job: `python3 tools/nav_core_check.py -- target/debug/mower_nav` |
| `autotune_telemetry_check.py` | `mower_pid_autotune` subscribes to `/mower_base/telemetry` only while a session runs, black box: the real binary against a fake 20 Hz telemetry frame; no subscriber while idle, one per session (the frame reaches the precheck), none 1 s after it, and "no /mower_base/telemetry" after ~1.5 s when the base is quiet. Run by CI's mower_rs tests job: `python3 tools/autotune_telemetry_check.py --binary target/debug/mower_pid_autotune` |
| `mux_oracle.py` | **twist_mux** (step E4): the real C++ twist_mux 4.5.0 and `mower_rsd --modules mux` in the runtime image, on the same timed input scripts (S01-S18 of the E4 design and a 10-minute random segment), every forwarded message, its content, its latency and every `/diagnostics` status compared with each other and with a model of twist_mux's rules; `vectors` writes `crates/mower_mux/tests/vectors/` from a C++ run ("The twist_mux oracle" below) |
| `rsp_capture.py` | **robot_state_publisher** (step E4): what rsp 3.3.4 publishes for the real robot's xacro (`/tf_static`, `/robot_description`, and `/tf` for a script of `/joint_states` around its 20 Hz throttle), every float as written, into `crates/mower_static_tf/tests/rsp_capture.json` |
| `probe_app.py` | **the app contract**: connects to `mower_ws_bridge` with the app's HMAC headers, subscribes to everything the app subscribes to and calls the read-only services, then passes or fails against a saved baseline and on content (robot online, base and odometry fresh, pose updating) |
| `switch.sh`, `measure.sh` | put the robot on a test image with `NAV_COMPOSITION` / `RUST_DAEMON` / `RUST_BASE` / `RUST_LOCALIZE` / `RUST_NAV_CORE` / any `KEY=VAL`, check which drivers actually run, and restore; per-process CPU, loopback packet rate and DDS thread split over 10 s |

## Checking the app still works

`probe_app.py` is what proves a `rust_*` switch did not break the phone app,
without a phone. It reads the pairing secret from
`~/.cache/mower-backend/pair-<robot>.json` (override with `MOWER_PAIR_FILE`),
so it authenticates exactly as the app does.

```bash
# on main, right before switching, to record what the app receives (and the rates)
python3 probe_app.py ws://192.168.0.114:9090 20 --save /tmp/baseline-main.json
# after every switch
python3 probe_app.py ws://192.168.0.114:9090 20 --baseline /tmp/baseline-main.json
```

It prints the per-topic first-message latency and rates, the `/adapter/*` set
expanded from `/rosapi/topics`, the service answers and a `content=` line, and
ends in `APP_CONTRACT_OK` or `APP_CONTRACT_FAIL`. It fails when a topic the
baseline delivered is missing, and also when what arrives is wrong: the last
`/robot/online` is not true, the telemetry's `base` block (the STM32's 0x85
feed) or `odom` block is stale or non-finite, `/adapter/robot_pose` stops
updating or is non-finite, or one of the four rated topics drops below 80 % of
the baseline's rate. Arriving alone proves little: with the base dead,
robot_status still publishes `/robot/online` (false), telemetry and battery,
and the EKF still has a pose.

`app_contract_baseline.json` here is the 2026-09-23 recording from the robot
on `main` (18 topics delivered, 22 listed). It has no rates, so the rate check
only applies to a baseline saved with this version. Indoors
`/adapter/map_datum` and the three inflated map layers are legitimately empty,
which is why the check compares against a recording rather than a fixed list.

## The twist_mux oracle

`mux_oracle.py run` starts `--cpp` copies of the image's twist_mux (node `twist_mux_cppN`, output
`/oracle/cppN/cmd_vel_guard_input`) and, with `--rust`, `mower_rsd --modules mux` (node `twist_mux`, output
`/cmd_vel_guard_input`), all on the production twist_mux_topics.yaml, and drives them from one rclpy probe with the
production writer QoS. Every input carries `<source>#<seq>` in its frame_id, so every output names what it forwarded.
Three sessions, each with fresh processes: `main` (S01-S14, S16, S17, the random segment), `startup` (S15: the muxes
start under running inputs) and `generic` (S18: a tie, priority 0, timeout 0 for an input and a lock, a priority-0
lock).

```bash
docker run --rm -u 0 --network none --cap-add NET_ADMIN -e ROS_DOMAIN_ID=71 \
  -v <repo>:/r:ro -v <out>:/o [-v <dir with mower_rsd>:/b:ro] \
  --entrypoint bash mower_path_planning:ros-free-test -c \
  'source /opt/ros/jazzy/setup.bash && source /mower_ws/install/setup.bash && \
   python3 /r/src/mower_rs/tools/mux_oracle.py run --out /o --cpp 2 [--rust /b/mower_rsd]'
python3 mux_oracle.py compare <out> [--latency]   # MUX_ORACLE_OK / MUX_ORACLE_FAIL
python3 mux_oracle.py vectors <out> ../crates/mower_mux/tests/vectors
```

`compare` takes the first C++ copy as the reference. A decision within 3 ms of a timeout, of another handle's
message, or of the same input's next message (a depth-1 reader keeps only the newer one) depends on delivery and is
listed, not compared. S16 compares the counts, S17 the first forwarded sequence number (within one).

**The noise floor**, two C++ copies on the same samples (2026-10-27, Docker Desktop on an M4, 10 cores):

- S01-S15 and S18: the same forwarded sequence as each other and as the model, message for message; S16 200 of 200 on
  both; S17 the same first message.
- The random segment (600 s, 29.5 k inputs, 8.2 k forwarded): 4 decisions away from a boundary differ, after 1610
  ambiguous ones are set aside. The probe stalls now and then for up to 1.2 s (both copies see the same latency then),
  and what arrives bunched after a stall is what differs.
- Latency, send to receipt through a mux: p50 1.2 ms, p99 6.4 ms. One copy against the other, per message: p50
  +0.12 ms, p99 +1.10 ms.
- `/diagnostics`: 2.00 a second, the "Node starting up" status first (when the probe matched the new writer in time),
  none at the end. The two of a second come from two timers on two clocks (twist_mux's wall timer, the updater's on
  the node clock): their order changes with the clock's slew, and the one that comes first carries the lock priority
  of the previous tick (216 of 1367 statuses), its very first one no handles at all.

## Switching and measuring on the robot

**The test image.** Build it from the commit that has the pre-flight
fixes: `feat/ros-free` with `fix/rf-base-preflight`, `fix/rf-localize-preflight`
and `fix/rf-tools-preflight` merged, from a clean tree (`git status
--porcelain` empty), so that the sha the image carries is what is in it. The
`rosfree-test` image built on 2026-09-23 (`MOWER_GIT_SHA` 32ba2aa, ID
`4d6792746f79`) predates all of them. Its `mower_base` has no feedback-loss
latch, and its `robot.launch.py` gives the daemon's recorder a relative
`zone_record`: with the robot's `RUST_RECORD=true` and `RUST_DAEMON=true`,
zones go to `/mower_ws/zone_record` in the container layer, the app lists none
of the saved ones and new ones are lost on restore. `switch.sh` therefore
refuses `RUST_BASE`, `RUST_LOCALIZE` or `RUST_DAEMON` unless
`SWITCH_EXPECT_SHA` names the commit and the image's `MOWER_GIT_SHA` starts
with it. With `RUST_BASE` it also refuses a `mower_base` / `mower_rsd` binary
without the latch's log text. It prints the image's `MOWER_VERSION` /
`MOWER_GIT_SHA` on every switch and `status`.

Compose runs `${IMAGE_REPO}:${IMAGE_TAG}` (the robot's `.env`; default
`docker.io/fxrbindi/mower_path_planning`) with `pull_policy: missing`, so the
loaded image must carry exactly that name, or compose tries a pull and gets
`manifest unknown`. A build can exit 0 and still
leave the previous image under the tag (a `rosdep install` failure far up the
log), so check the sha and the binaries before saving.

```bash
SHA=$(git rev-parse HEAD); test -z "$(git status --porcelain)" || echo DIRTY
docker buildx build --platform linux/arm64 --target runtime -f Dockerfile \
  --build-arg MOWER_VERSION=${SHA:0:7}-rosfree --build-arg MOWER_GIT_SHA=$SHA \
  --build-arg MOWER_BUILD_UNIX=$(git log -1 --format=%ct) \
  --build-arg MOWER_IMAGE=docker.io/fxrbindi/mower_path_planning:rosfree-test \
  --cache-from type=registry,ref=ghcr.io/edwards414/mower_path_planning-buildcache:runtime \
  --load -t docker.io/fxrbindi/mower_path_planning:rosfree-test .
docker image inspect -f '{{range .Config.Env}}{{println .}}{{end}}' \
  docker.io/fxrbindi/mower_path_planning:rosfree-test | grep MOWER_GIT_SHA   # must be $SHA
docker run --rm --entrypoint grep docker.io/fxrbindi/mower_path_planning:rosfree-test \
  -caF 'arm latch: wheel feedback lost' /mower_ws/install/mower_rs/lib/mower_rs/mower_base  # 1, not 0
docker run --rm --entrypoint grep docker.io/fxrbindi/mower_path_planning:rosfree-test \
  -caF 'not receiving our commands' /mower_ws/install/mower_rs/lib/mower_rs/mower_base  # 1: the TX-lead trigger
docker run --rm --entrypoint grep docker.io/fxrbindi/mower_path_planning:rosfree-test \
  -caF 'dead-man is let go' /mower_ws/install/mower_rs/lib/mower_rs/mower_base  # 1: the review fixes (blade, board gate)
docker run --rm --entrypoint grep docker.io/fxrbindi/mower_path_planning:rosfree-test \
  -caF 'non-zero after' /mower_ws/install/mower_rs/lib/mower_rs/mower_base  # 1: the release window (guard zero)
docker save docker.io/fxrbindi/mower_path_planning:rosfree-test | gzip -1 > /tmp/rosfree-test.tar.gz
scp /tmp/rosfree-test.tar.gz cat@<robot>:/tmp/
ssh cat@<robot> 'gunzip -c /tmp/rosfree-test.tar.gz | sudo docker load && rm /tmp/rosfree-test.tar.gz'
# only if it was saved under another name:
#   sudo docker tag mower_path_planning:ros-free-test docker.io/fxrbindi/mower_path_planning:rosfree-test
```

**The scripts** go somewhere that survives a boot (`/tmp` is wiped), together
with the branch compose: main's compose does not pass `RUST_BASE`,
`RUST_LOCALIZE`, `RUST_DAEMON` or `NAV_COMPOSITION` through, and `switch.sh`
refuses to run without one that does.

```bash
ssh cat@<robot> 'mkdir -p ~/rosfree'
scp switch.sh measure.sh ../../../deploy/docker-compose.yaml cat@<robot>:rosfree/
ssh cat@<robot> 'sudo systemctl stop mower-update.timer'            # no auto-update mid-test
ssh cat@<robot> "SWITCH_EXPECT_SHA=$SHA bash ~/rosfree/switch.sh rosfree-test false false ~/rosfree/docker-compose.yaml RUST_BASE=true MOWER_FIRMWARE_SYNC=0"
ssh cat@<robot> 'sleep 90; bash ~/rosfree/measure.sh'
ssh cat@<robot> 'bash ~/rosfree/switch.sh status'                   # any time; changes nothing
ssh cat@<robot> 'bash ~/rosfree/switch.sh restore; sudo systemctl start mower-update.timer'
ssh cat@<robot> 'sudo docker rmi docker.io/fxrbindi/mower_path_planning:rosfree-test'   # when done testing
```

`switch.sh <tag> <nav_composition> <rust_daemon> <compose> [KEY=VAL ...]`:

- Every switch is absolute. `.env` is rebuilt from the pre-test backup and
  `NAV_COMPOSITION`, `RUST_DAEMON`, `RUST_BASE`, `RUST_LOCALIZE` and
  `RUST_NAV_CORE` are always written (the last three `false` unless given), so
  nothing from an earlier call sticks. `RUST_*` / `NAV_*` must be exactly
  `true` or `false`. Edit `.env` during a test and the next switch refuses
  rather than silently dropping the edit: make the same edit in
  `.env.bak-rosfree-test`, which is what `restore` puts back.
- It exits 1 (`SWITCH_FAIL: ...`) if the image is not loaded or is not the
  expected build (above), `compose up` fails, the container's launch
  arguments are not the requested ones, or the expected drivers are not what
  runs: with `RUST_BASE=true` no `ros2_control_node` and a `mower_base` (or
  `mower_rsd --modules ...base...`), with `RUST_LOCALIZE=true` no `ekf_node` /
  `navsat_transform_node` and a `mower_localize` (or the daemon's `localize`
  module), and the reverse for `false`; with `RUST_NAV_CORE=true` also no nav2
  process (neither the component container nor a separate server) and the
  daemon's `nav` module (`false`, nav2 as before, is not checked). It reads
  the process table with `docker top`, never the ros2 CLI (that causes
  controller overruns on the robot), and waits up to `SWITCH_WAIT_S` (180 s)
  for the drivers to appear, then requires the same PIDs for
  `SWITCH_STABLE_S` (30 s) with the container log checked on every poll: a
  driver that dies or is respawned, or a `mower_rsd` base/localize module
  that fails and restarts inside the same PID, fails the switch. `SWITCH_OK`
  otherwise.
- **After a `SWITCH_FAIL`, run `restore`.** A failure from `compose up` on
  stops `lawan_node` (nothing drives the wheels; so does Ctrl-C, a kill or a
  dropped ssh before `SWITCH_OK`). A failure before that leaves either nothing
  changed (it says so) or a partly applied test config (it says `run restore
  now`). Either way the test config is still what the next boot starts.
- `status` shows the build, the launch arguments, the drivers and the
  base/localization restarts in the running container's log so far. Run it
  again right before driving: `SWITCH_OK` only covers its 30 s window.
- For the first `RUST_BASE` / `RUST_LOCALIZE` runs keep `NAV_COMPOSITION=false`
  (the second argument): composed nav2 has never navigated on the robot
  either, and a failure should have one suspect.
- `RUST_NAV_CORE=true` (mower_nav steers with `mower_nav_core` instead of
  nav2, `../README.md`) is refused before anything changes unless the third
  argument is `true` and `RUST_LOCALIZE=true RUST_NAV=true` are given, the
  same rule `robot.launch.py` enforces: `switch.sh <tag> true true <compose>
  RUST_BASE=true RUST_LOCALIZE=true RUST_NAV=true RUST_NAV_CORE=true`. The
  first one on a robot is a supervised outdoor run with RTK.
- **The test config outlives a reboot or power-off.** `mower.service` brings
  up whatever `.env` and the compose say, and `mower-update.sh` cannot pull a
  local-only tag, so after a power-button hold, a brownout or a host poweroff
  the robot comes back on the test drivers until someone runs `restore`. Run
  it before leaving the robot.
- `restore` puts the pre-test `.env` and compose back, recreates the
  container, checks both md5s against the backups, the image, the launch
  arguments and that `ros2_control_node` and the C++ EKFs run again, and only
  then removes the backups (`RESTORE_OK`). With no test active it does nothing
  and exits 0. Chain the timer with `;`, not `&&`, so it restarts even when the
  restore fails. Backups without the `ROSFREE_TEST_ACTIVE` marker (left by
  the old `switch.sh`, which never deleted them) are refused, not restored.

**Every image change re-flashes the STM32.** `utils/firmware-sync` runs at each
container start and flashes unless the board's build id matches the image's
bundled firmware, and the test image's build differs from main's even when the
source is the same: a ~20 s bootloader flash (motors off) on the switch and
another on the restore, and a failed flash leaves the STM32 in its bootloader.
When `git diff <main's sha> <test sha> -- firmware/` is empty, pass
`MOWER_FIRMWARE_SYNC=0` as above; `restore` then brings back the pre-test value
and main finds its own build still on the board.

**Pulling the STM32 cable does not test the restart path.** `/dev/stmcom` is
the LubanCat's native UART (ttyS3), which reports no I/O error when the cable
goes. Pulling it exercises only the firmware's 300 ms command timeout plus
mower_base's arm latch, never the serial-error stop burst and the 2 s
restart. Each way of pulling has its own trigger, so do all three: both
leads (or the STM32 TX one) is the feedback loss (`no wheel feedback for
...`, `arm latch: wheel feedback lost`; `fix/rf-base-preflight` 27bbfaa);
the LubanCat TX -> STM32 RX lead alone (40-pin pin 8 -> PA10), the pull of
the 2026-09-28 run, is the command path loss (`STM32 reports
COMMAND_TIMEOUT: no 0x01 for ... ms`, `arm latch: the STM32 is not
receiving our commands`; `fix/rf-base-cmdtimeout`, the second grep in the
image check above; also with the lead already out when mower_base starts,
which it catches about 0.5 s after the start); an STM32 reset is the
feedback loss, and `STM32 restart: ...` only if the wheels had turned more
than about 1.3 revolutions since the board's previous boot or no 0x01
reached the new boot before its first status (its absence is not a
failure). After the re-seat the log shows `STM32 receiving commands again`
(all three) and the wheels stay put until cmd_vel has been released for
0.5 s (`arm latch: cmd_vel re-armed by ...`); a blade held through the pull
stays off until it has been let go for as long. On this robot pulling pin 8
also resets the USB hub (2026-09-29: IMU and camera re-enumerated), which
stalls the robot side for ~0.3 s: `velocity_command_guard` then logs
`Drivetrain command receipt timeout; forced velocity to zero` and feeds the
driver one zero. The image before `fix/rf-latch-release-window` took that
zero for the release and the still-held stick drove the wheels 0.25 s
later; now the log shows `arm latch: cmd_vel stop not held for 0.50 s
(non-zero after ...), still held` and the wheels stay put. Wheels that start
by themselves after a `re-armed` line are a failure. Read it with `sudo docker logs mower-lawan_node-1 2>&1 | grep
-E 'arm latch|STM32'`. Still re-seat only
with the sticks released and navigation idle: the latch is what is being
tested, not something to rely on yet. To test
the restart, kill the driver by PID (`sudo docker top mower-lawan_node-1 -o
pid,args | grep mower_base`, then `sudo kill -9 <pid>`; not `pkill -f`,
whose pattern can match and kill your own shell). Launch respawns it after
2 s, and the wheels must stay put until a stop edge; `switch.sh status` then
counts that restart. With `RUST_DAEMON=true` that PID is the whole
`mower_rsd`.

Note `measure.sh` reports the box's idle percentage too, but on the RK3568 the
governor moves between 1.4 and 2.0 GHz, so compare per-process CPU and the
loopback packet rate, not the idle figure.
