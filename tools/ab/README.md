# tools/ab: image A/B

Runs two robot images side by side on fake devices and diffs what the app can
see. Every step that removes ROS (docs/ROS_FREE_PLAN.md Phase E, E9 §3.1
O1/O2) proves "no visible difference" with it: A is the image before the
change, B the image after. An empty diff means the app sees no difference.
A non-empty diff lists each difference, one line each.

It replaces `~/work/e2-verify`, the harness that checked E2. That harness
compared hand-built binaries over DDS. This one compares images, through the
bridge, as the app uses them.

## Run it

Locally (Docker, Python 3 with `websockets` 14 or later; the images are
arm64, so an Apple-silicon Mac or an arm64 Linux box):

```sh
python3 tools/ab/ab.py run IMAGE_A IMAGE_B [--out DIR]
python3 tools/ab/ab.py run docker.io/fxrbindi/mower_path_planning@sha256:5c72f8ce10d88b33d46381f3eb85de603dbdd57974b091d79c400000bb70160c \
                           docker.io/fxrbindi/mower_path_planning@sha256:63d7b8f1859312474afe0efa870985373abc47bcb72e506c862263dbdfdc4f45
```

An image that is not local is pulled (about 330 MB compressed, 2 GB
unpacked). One run takes about 3 minutes; 60 s of that is `docker stop`
waiting out compose's grace period (see "Stopping" below). For quicker local
iterations, add `--stop-grace 10 --cpu-seconds 0`.

In CI: Actions, "Image A/B", "Run workflow" (`.github/workflows/ab.yml`), with
two image refs: a Docker Hub digest as above, or a GHCR ref such as
`ghcr.io/edwards414/mower_path_planning:main`. Leave `checks` at all of them
unless a step changes something on purpose (E4 moving `twist_mux` and `rsp`
in-process changes the log, for example). Then list the checks that must stay
the same, or tick `report_only`. The job uploads the whole output directory as
an artifact and puts `report.md` in the job summary.

`ab.py compare DIR` redoes the comparison of an earlier run, for example after
a change to the normalisation.

Exit code: 0 when no gated check differs, 1 when one does (0 with
`--report-only`), 2 when the harness itself failed.

## What one side is

For each image, `ab.py` starts one container with its own network namespace
and its own `ROS_DOMAIN_ID` (A 41, B 42). Both sides use the same fixtures and
run at the same time, so they see the same host load. Each container runs
`side.sh` as root:

1. **Network.** Multicast on `lo` (DDS, as `deploy/host/mower.service` does),
   and the robot's VPN address 10.77.0.2 on `lo`, so `ws_bridge` binds its
   `--address` as it does on the robot.
2. **State directory.** A fresh `/home/mower/.mower` (what compose
   bind-mounts) with `fixtures/identity.json` (pairing gate on, a fake secret)
   and the repo's `zone_record` geometry, owned by uid 1000.
3. **Devices.** `src/mower_rs/tools/fake_base.py` on `/dev/stmcom` (the
   STM32) and `fake_sensors.py` on `/dev/imu_usb` and `/dev/gps_rtk` (a
   stationary WIT IMU and an RTK-fixed F9P), all `root:dialout 0660` like the
   real devices.
4. **Ports.** A TCP relay from published ports to the bridge's loopback
   listeners 9090 (the pairing gate) and 9091 (no gate).
5. **The stack.** `exec` of the image's own entrypoint with compose's
   `lawan_node` command, read from `deploy/docker-compose.yaml`, with the
   robot's `.env` switches (`ROBOT_ENV` in `ab.py`: every `RUST_*` true,
   `RUST_NAV_CORE` false). It runs as uid 1000, as PID 1, and its output is
   the container's log, as under compose. Nothing of the image under test is
   replaced: binaries, configs and launch files all come from the image.

The image has to have `python3` (stdlib only) for the fakes and the helpers,
and `bash` and `setpriv`. The ROS images have them, and so does E7's
`slim-test`.

Where a side differs from the robot, on purpose:

- `MOWER_FIRMWARE_SYNC=0`: the fake STM32 has no bootloader.
- No `MOWER_BACKEND_URL`, so the agent idles.
- The container has its own network, IPC and PID namespaces instead of the
  host's, and no `/dev` bind mount. The probes reach the bridge through
  published ports.

Then `ab.py` probes each side from the host:

| when | what | file |
|---|---|---|
| until it answers (≤ 120 s) | `/rosapi/topics` on 9091 | `bridge_wait.txt` |
| until OK (≤ 120 s, 5 s tries) | `probe_app.py`: the app would show the robot healthy | `ready.txt` |
| 20 s | `probe_app.py --baseline app_contract_baseline.json`, one client | `contract.{txt,json}` |
| 20 s | the same with two clients at once | `contract_2{a,b}.{txt,json}` |
| once | `/rosapi/topics` and the read-only services on 9091 | `services.json` |
| 10 s | every listed topic through the gate | `topics.json` |
| 30 s | CPU per process, no client (informational) | `cpu_idle.json` |

Then the processes, mower_rsd's argv and the files it reads (`procs.txt`,
`rsd_argv.txt`, `rsd_files/`). Then `docker stop --time 60`. Then the
container's log (`stack.log` up to the stop, `stack_stop.log` after it), the
fakes' files (`fake_base.jsonl.gz` is every frame the fake STM32 saw or sent)
and the state directory (`state/`).

## What is compared

`compare.py` turns each side into sorted, normalised lines (`A.summary.txt`,
`B.summary.txt`). `diff.txt` is their unified diff. `report.md` has the diff,
per-check counts, and the numbers that are not compared.

| check | lines | normalised away |
|---|---|---|
| `contract` (O1) | `probe_app.py`'s verdict (`APP_CONTRACT_OK`, `content=`, `services=`, missing topics) for one client and for two at once, and the topics each one got | timings, rates, message counts |
| `lists` | `/rosapi/topics` (names and types); the bridge policy `ws_bridge.yaml` that mower_rsd was given (what the app may subscribe to, publish and call) | comments |
| `services` | the answers of the policy's read-only services on 9091: `/site_op` (list), `/check_nav_status`, `/coverage_progress_status`, `/rosapi/get_time` (result only) | header stamps; floats to 1e-6 |
| `topics` | for every listed topic, through the gate: whether it delivered, the JSON shape of every message (each key path with its JSON type, inside JSON strings too: an int where there was a float breaks the Dart app), and the distinct contents | stamps, floats to 1e-6; see below |
| `frames` (O2) | per STM32 frame type the host sent (`fake_base.py`'s rx), the payloads in order with consecutive repeats collapsed (e2-verify's `base_frames.py`) | counts and rates |
| `log` (O5) | the set of distinct lines the stack printed while it ran | timestamps, pids, launch's process numbers, client ports, temp file names, decimal numbers (measured values) |

Contents that cannot be the same from run to run are not compared:

- `/robot/telemetry`, `/gps/status` and `/adapter/robot_pose` carry live
  values, so only their shape is compared.
- `/robot/info`'s `uptime_s` is masked, and so are its bundled firmware's
  `crc32` and `size`. The firmware embeds the build identity, and whether a
  constant takes one Thumb-2 instruction or two depends on its value: 3827804
  and 16f7356 build the same sources 4 bytes apart. `/pid_autotune/status`'s
  `updated_at` and the random session id in `/manual_command_clock` are
  masked too.
- `/adapter/map_datum`'s `bearing_rad` is masked too. It is the EKF's yaw at
  the moment navsat locks the datum, so it depends on how far the filter has
  converged by then.

Each side's own build identity (its image's `MOWER_GIT_SHA`, `MOWER_VERSION`,
`MOWER_BUILD_UNIX` and `MOWER_IMAGE`) is replaced by a placeholder wherever it
shows. A side that reports its own version therefore compares equal. A side
that reports some other version shows up in the diff.

Some log lines depend on the machine's load, not on the image (`TIMING` in
`compare.py`). These are the lines that say the host stalled a thread
(mower_base's feedback-loss, overrun and arm-latch-gap lines, the IMU's "host
timing gap"), and the lines such a stall leads to: a command that arrived too
late, or `/robot/online -> false` at start, when robot_status's first
heartbeat tick, 0.5 s in, comes before mower_base's first `/odom_slow`. The
report lists them with their counts; the diff leaves them out. A robot that
is offline while the probes run still fails the `contract` check, and one
whose `/robot/online` flips during the `topics` window fails that one. Lines
printed after `docker stop` (`stack_stop.log`) are only counted.

Not compared, in the report only (O6 and friends): how long the bridge and the
app contract took to come up, the rates `probe_app.py` measured, message and
frame counts, CPU per process, whether mower_rsd's argv and the process list
are the same, and how `docker stop` went.

## Stopping

`ros2 launch` has no working SIGTERM handler in these images. As PID 1 (which
is what compose runs) it ignores `docker stop`'s SIGTERM. The stack keeps
running until compose's 60 s grace period runs out, and then everything is
SIGKILLed (exit code 137). The robot's container does the same on every
update. The harness keeps the robot's behaviour. An image whose stack stops
on SIGTERM shows its shorter stop time and its exit code in the report.

## Compared with e2-verify

Kept: `fake_base.py` (unchanged, from `src/mower_rs/tools`), `fake_sensors.py`
(without its rclpy `--follow-odom` mode), `probe_app.py` against
`app_contract_baseline.json` (unchanged), `services_probe.py`'s calls through
the bridge, `base_frames.py`'s frame summary, `log_summary.py`'s normalisation
and `cpu_window.py` (informational). The fixtures are e2-verify's.

Dropped:

- **Hand-built binaries and other worktrees' configs, and the hand-assembled
  argv of `prod_run.sh`.** Everything now comes from the image, started the
  way compose starts it.
- **What needs rclpy or DDS:** `graph_dump`, `graph_diff`, `presence_watch`,
  `dropout_watch`, the DDS half of `services_probe`, `mission_probe`,
  `action_probe` and the shadow and localize comparisons. A ROS-free image has
  no graph to dump, and the app sees none of it. The node-attribution
  differences e2-verify reported for E2 (`/mower_bus_bridge` owning endpoints)
  are invisible through the bridge by design.
- **`probe_compat.py`.** The probes run on the host with a current
  `websockets`, so both sides are probed by the same client.
- **The mutating scenarios** (`mission_*`, `mutation_probe`, `actions_*`,
  `dropout_*`, `startup_*`, `maploop`, `baserestart`). They drove mower_rsd
  over DDS or with rclpy. Only read-only calls go through the bridge here.
  Bringing a scenario back means writing it as a bridge client, as the app
  would drive it.

## Files

| file | runs | what |
|---|---|---|
| `ab.py` | host | runs both sides, probes them, stops them, compares |
| `probe.py` | host | `/rosapi/topics`, read-only services, every topic's shape and contents |
| `compare.py` | host | normalised lines per check, the diff, `report.md` |
| `side.sh` | image | sets a side up, then `exec`s the stack |
| `incontainer.py` | image | lo setup, the port relay, the CPU window, the pre-stop snapshot |
| `fake_sensors.py` | image | fake IMU and F9P |
| `fixtures/` | image | `identity.json`, `pair.json` (its secret, for the probes), `zone_record/` |
