# mower_rs/tools

Verification harnesses. The `*_compare.py` ones run a Python/C++ node and its
Rust replacement against the same fakes and diff every message; the three at
the bottom drive an A/B measurement on the real robot.

| tool | what it does |
|---|---|
| `shadow_compare.py` | separate mower_rs binaries vs one `mower_rsd` process, every topic compared (Phase A5) |
| `fake_base.py`, `base_harness.py`, `base_compare.py` | a fake STM32 on a socat pty, then the `ros2_control` chain vs `mower_base` against it (Phase B) |
| `localize_compare.py` | `dual_ekf_navsat.launch.py` vs `mower_localize` on one synthetic sensor stream (Phase C) |
| `probe_app.py` | **the app contract**: connects to `mower_ws_bridge` with the app's HMAC headers, subscribes to everything the app subscribes to and calls the read-only services, then passes or fails against a saved baseline and on content (robot online, base and odometry fresh, pose updating) |
| `switch.sh`, `measure.sh` | put the robot on a test image with `NAV_COMPOSITION` / `RUST_DAEMON` / `RUST_BASE` / `RUST_LOCALIZE` / any `KEY=VAL`, check which drivers actually run, and restore; per-process CPU, loopback packet rate and DDS thread split over 10 s |

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

Compose runs `ghcr.io/edwards414/mower_path_planning:${IMAGE_TAG}` with
`pull_policy: missing`, so the loaded image must carry exactly that name, or
compose tries a pull and gets `manifest unknown`. A build can exit 0 and still
leave the previous image under the tag (a `rosdep install` failure far up the
log), so check the sha and the binaries before saving.

```bash
SHA=$(git rev-parse HEAD); test -z "$(git status --porcelain)" || echo DIRTY
docker buildx build --platform linux/arm64 --target runtime -f Dockerfile \
  --build-arg MOWER_VERSION=${SHA:0:7}-rosfree --build-arg MOWER_GIT_SHA=$SHA \
  --build-arg MOWER_BUILD_UNIX=$(git log -1 --format=%ct) \
  --build-arg MOWER_IMAGE=ghcr.io/edwards414/mower_path_planning:rosfree-test \
  --cache-from type=registry,ref=ghcr.io/edwards414/mower_path_planning-buildcache:runtime \
  --load -t ghcr.io/edwards414/mower_path_planning:rosfree-test .
docker image inspect -f '{{range .Config.Env}}{{println .}}{{end}}' \
  ghcr.io/edwards414/mower_path_planning:rosfree-test | grep MOWER_GIT_SHA   # must be $SHA
docker run --rm --entrypoint grep ghcr.io/edwards414/mower_path_planning:rosfree-test \
  -caF 'arm latch: wheel feedback lost' /mower_ws/install/mower_rs/lib/mower_rs/mower_base  # 1, not 0
docker save ghcr.io/edwards414/mower_path_planning:rosfree-test | gzip -1 > /tmp/rosfree-test.tar.gz
scp /tmp/rosfree-test.tar.gz cat@<robot>:/tmp/
ssh cat@<robot> 'gunzip -c /tmp/rosfree-test.tar.gz | sudo docker load && rm /tmp/rosfree-test.tar.gz'
# only if it was saved under another name:
#   sudo docker tag mower_path_planning:ros-free-test ghcr.io/edwards414/mower_path_planning:rosfree-test
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
ssh cat@<robot> 'sudo docker rmi ghcr.io/edwards414/mower_path_planning:rosfree-test'   # when done testing
```

`switch.sh <tag> <nav_composition> <rust_daemon> <compose> [KEY=VAL ...]`:

- Every switch is absolute. `.env` is rebuilt from the pre-test backup and
  `NAV_COMPOSITION`, `RUST_DAEMON`, `RUST_BASE` and `RUST_LOCALIZE` are always
  written (the last two `false` unless given), so nothing from an earlier call
  sticks. `RUST_*` / `NAV_*` must be exactly `true` or `false`. Edit `.env`
  during a test and the next switch refuses rather than silently dropping the
  edit: make the same edit in `.env.bak-rosfree-test`, which is what
  `restore` puts back.
- It exits 1 (`SWITCH_FAIL: ...`) if the image is not loaded or is not the
  expected build (above), `compose up` fails, the container's launch
  arguments are not the requested ones, or the expected drivers are not what
  runs: with `RUST_BASE=true` no `ros2_control_node` and a `mower_base` (or
  `mower_rsd --modules ...base...`), with `RUST_LOCALIZE=true` no `ekf_node` /
  `navsat_transform_node` and a `mower_localize` (or the daemon's `localize`
  module), and the reverse for `false`. It reads the process table with
  `docker top`, never the ros2 CLI (that causes controller overruns on the
  robot), and waits up to `SWITCH_WAIT_S` (180 s) for the drivers to appear,
  then requires the same PIDs for `SWITCH_STABLE_S` (30 s) with the container
  log checked on every poll: a driver that dies or is respawned, or a
  `mower_rsd` base/localize module that fails and restarts inside the same
  PID, fails the switch. `SWITCH_OK` otherwise.
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
mower_base's feedback-loss latch (`fix/rf-base-preflight` 27bbfaa; the image
check above makes sure it is in the binary), never the serial-error stop
burst and the 2 s restart. The log shows `no wheel feedback for ...` and
`arm latch: wheel feedback lost` when the lead goes, and after the re-seat
the wheels stay put until cmd_vel stops (`arm latch: cmd_vel re-armed by
...`); read it with `sudo docker logs mower-lawan_node-1 2>&1 | grep 'arm
latch'`. Still re-seat only with the sticks released and navigation idle:
the latch is what is being tested, not something to rely on yet. To test
the restart, kill the driver by PID (`sudo docker top mower-lawan_node-1 -o
pid,args | grep mower_base`, then `sudo kill -9 <pid>`; not `pkill -f`,
whose pattern can match and kill your own shell). Launch respawns it after
2 s, and the wheels must stay put until a stop edge; `switch.sh status` then
counts that restart. With `RUST_DAEMON=true` that PID is the whole
`mower_rsd`.

Note `measure.sh` reports the box's idle percentage too, but on the RK3568 the
governor moves between 1.4 and 2.0 GHz, so compare per-process CPU and the
loopback packet rate, not the idle figure.
