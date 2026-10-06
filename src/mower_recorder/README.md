# mower_recorder

Field "bag system" for analysing every outdoor run. It records the three planes
you need to reconstruct a run — not just topics:

| Plane | What | How |
| --- | --- | --- |
| ① data | topic time-series (odom, tf, cmd_vel, battery, GPS, maps, Nav2, `/rosout`) | rosbag2 → **mcap** + zstd, split by size/time, QoS overrides for latched maps |
| ② graph | node / topic / service state **over time** + node-death events | `graph_snapshot_node` → `/graph_snapshot` (recorded into the bag) |
| ③ metadata | run_id, robot_id, **git sha**, params dump, GPS start | `run_metadata.yaml` next to the bag |

Design rationale and the fleet analysis layer are in the chat design notes.
This package is the on-robot half (record + graph + metadata). Foxglove Studio
opens the resulting `.mcap` for analysis.

## Nodes

- **`recorder_manager_node`** — starts/stops rosbag2, writes metadata, exposes
  services, flushes the snapshot buffer on a fault.
- **`graph_snapshot_node`** — periodically publishes the live graph as JSON on
  `/graph_snapshot`, logs a WARN when any node disappears, and raises a fault on
  `/mower_recorder/fault` if a *watched* node dies (e.g. `map_manage_node`).

## Layout of a run

```
<output_root>/<robot_id>_<timestamp>/
├── bag/            # full-record mcap (split into _0.mcap, _1.mcap …)
├── snapshots/      # heavy topics (costmaps/plan), only written on a fault
├── params/         # ros2 param dump of key nodes at start
└── run_metadata.yaml
```

## Run it

```bash
# standalone (records whatever of the allowlist is currently publishing)
ros2 launch mower_recorder record.launch.py \
    robot_id:=mower-03 \
    output_root:=~/mower_bags \
    git_repo_dir:=~/mower_ws/src/mower_path_planning
```

Or include it from your mission launch so every mission is recorded:

```python
IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('mower_recorder'),
                 'launch', 'record.launch.py')))
```

On the robot this runs as the `recorder` service of
`deploy/docker-compose.yaml` with `autostart:=false`: idle until the app's
錄話題 button (manual-control panel) calls `/mower_recorder/start` / `stop`.
`lawan_node` keeps `record:=false`, so that service is the only owner of
`/mower_recorder/*`; `mower-data-collection.sh` stops it for a
data-collection run and starts it again afterwards.

While a recording is running the robot's **rear light breathes red** (front
light unchanged): `recorder_manager_node` publishes
`{"effect":"recording"|"off","source":"bag"}` on `/mower_base/rear_light` with
every 2 s status tick, and the `mower_hardware` driver turns it into the STM32
`0x03` overlay bit (drops it after 6 s without a refresh). The app's zone
recording (`path_record_node`, source `path_record`) uses the same light; the
driver keeps them apart, so this node's idle "off" does not cut a zone
recording. Param `rear_light_topic` (`''` disables).

### Control services

```bash
ros2 service call /mower_recorder/start    std_srvs/srv/Trigger {}
ros2 service call /mower_recorder/stop     std_srvs/srv/Trigger {}   # finalizes the mcap
ros2 service call /mower_recorder/snapshot std_srvs/srv/Trigger {}   # flush heavy buffer now
```

A fault also flushes the snapshot buffer:

```bash
ros2 topic pub -1 /mower_recorder/fault std_msgs/msg/Bool "{data: true}"
```

## Configure

- **`config/record_topics.yaml`** — the always-on allowlist, the heavy
  `snapshot_topics`, compression and split sizes. **Edit the topic names to match
  your robot** (`ros2 topic list`).
- **`config/qos_overrides.yaml`** — latched (`TRANSIENT_LOCAL`) topics must be
  listed here or the recorder captures nothing for them.

## Notes / to validate on-device

- `autostart` waits 3 s after launch, then begins recording.
- Stop with the service or Ctrl-C — **not `kill -9`** — so the mcap is finalized
  and indexed (SIGINT is handled).
- The recorders are renamed via `--ros-args -r __node:=…`; if your rosbag2 build
  ignores that, the snapshot service path (`/mower_snapshot_recorder/snapshot`)
  changes — adjust `SNAPSHOT_RECORDER_NODE` in `recorder_manager_node.py`.
- Do not add camera/point-cloud topics to the always-on list — record video via
  the WebRTC/MediaMTX pipeline and correlate by timestamp. For camera data use
  the `data_collection` profile below instead.
- Write to eMMC/SSD, not microSD.

## data_collection profile (camera + localization for GrassVision)

A second, opt-in profile for offline mapping data (GrassVision spec
`doc/Robot_Recording_R2_Pipeline_Spec.md` §1–2). The always-on profile above
is unchanged (the app replays it).

```bash
ros2 launch mower_recorder data_collection.launch.py \
    robot_id:=mower-03 output_root:=~/.mower/bags calib_dir:=~/.mower/calib
ros2 service call /mower_recorder/start std_srvs/srv/Trigger {}
ros2 service call /mower_recorder/stop  std_srvs/srv/Trigger {}   # finalizes both mcaps
```

| | |
| --- | --- |
| camera | gscam passes the USB camera's MJPEG through as `/camera/front/image_raw/compressed` (no re-encode), 10 Hz via `videorate drop-only` (keeps capture stamps), `camera_info` from `calib/camera_front.yaml`, frame `camera_front_optical_frame` (`mower_recorder/camera_launch.py`) |
| TF | `base_link → camera_front_link → camera_front_optical_frame` static TF computed from measured extrinsics (`config/camera_front_extrinsics.yaml` template, `camera_extrinsics.py`) |
| `bag/` | telemetry incl. `/gps/status` (mower_gps: carrier solution, satellites) and the three EKF outputs, MCAP `zstd_fast` chunk compression (no per-message zstd) |
| `camera/` | image + camera_info + `/tf_static`, uncompressed, 256 MB splits |
| metadata | spec 1.6 fields in `run_metadata.yaml` (`config/data_collection_session.yaml` + launch values), `calib/` copied into the run |
| upload | `bag_store` uploads every file, then `_manifest.json` (key/size/sha256 + topic counts) last; interrupted uploads resume (`run_manifest.py`) |
| check | `mower-check-run <run_dir>` (pure Python, also runs off-robot) |

`mower-bag upload --all` / `mower-bag verify <run_id>` do the same upload and a
remote check from the command line. Deployment: `deploy/docker-compose.data-collection.yaml`
+ `deploy/host/mower-data-collection.sh`; field procedure (Chinese):
`docs/資料收集錄製程序.md`.

Tests (no ROS needed):

```bash
python -m pytest src/mower_recorder/test -q   # needs pytest pyyaml mcap mcap-ros2-support zstandard moto boto3 pillow
```
