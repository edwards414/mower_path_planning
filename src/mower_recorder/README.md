# mower_recorder

The field "bag system" for analysing every outdoor run. Since E5-7 the recorder itself is **mower_rsd's `bag`
module** (`src/mower_rs/crates/mower_recorder`, switch `rust_recorder` / compose `RUST_RECORDER`, on by default): it
records the three planes you need to reconstruct a run, the same files and topics the Python recorder wrote.

| Plane | What | How |
| --- | --- | --- |
| ① data | topic time-series (odom, tf, cmd_vel, battery, GPS, maps, Nav2, `/rosout`) | rosbag2's **MCAP** + zstd, written by `mower_mcap`; split by size/time, QoS overrides for latched maps |
| ② graph | node / topic / service state **over time** + node-death events | the module's `graph_snapshot` → `/graph_snapshot` (recorded into the bag) |
| ③ metadata | run_id, robot_id, **git sha**, params dump, GPS start | `run_metadata.yaml` next to the bag |

This package keeps what is not the recorder:

- **`config/`**: the profiles the module reads from this package's share directory (`record_topics.yaml`, the
  always-on profile behind the app's 錄話題 button; `data_collection.yaml`; `qos_overrides.yaml`), and the templates
  an operator copies to the robot (`data_collection_session.yaml`, `camera_front_extrinsics.yaml`, `r2.env.example`).
- **`mower-check-run`** (`check_run.py`): checks a run directory. Pure Python (PyYAML; zstandard optional), the
  independent reader of what the module writes (design D9), also run off the robot.
- **`mower-bag`** (`cli.py`, boto3): `ls`, `get`, `rm`, `upload`, `verify` against R2, an off-robot tool. On the
  robot, `mower-data-collection.sh upload` / `verify` run `mower_bag` (Rust, `mower_recorder_core::cli`: the same
  output lines and exit codes). It stays in the image, boto3 with it, while a robot may have the helper from before
  E5-7, which calls `ros2 run mower_recorder mower-bag` (OTA does not update host files).
- `run_manifest.py` (mower-bag's uploader) and `camera_extrinsics.py` (the extrinsics math the module ports; the
  tests' synthetic runs use it).

The three rclpy nodes (`recorder_manager`, `bag_store`, `graph_snapshot`), `record.launch.py`,
`data_collection.launch.py`, `front_camera.launch.py` (gscam), `record_profile.py`, `video_record.py` and
`camera_launch.py` were removed in E5-7; the Rust port cites them by file and line, so read them at bf19e6c. Their
rosbag2 and gscam packages left the image in E5-7c, so `ros2 bag` (`reindex`, `info`, `play`) runs off the robot now.
Roll-back to the Python recorder is an image from before E5-7
(`mower-update.sh --tag <previous>`).

## Layout of a run

```
<output_root>/<robot_id>_<timestamp>/
├── bag/            # full-record mcap (split into _0.mcap, _1.mcap …)
├── snapshots/      # heavy topics (costmaps/plan), only written on a fault
├── params/         # parameter dump of key nodes at start
├── video/          # MediaMTX's fMP4 of the front camera (default profile)
├── camera/, calib/ # data_collection only
└── run_metadata.yaml
```

## Control

The app's Bag page, or the services:

```bash
ros2 service call /mower_recorder/start    std_srvs/srv/Trigger {}
ros2 service call /mower_recorder/stop     std_srvs/srv/Trigger {}   # finalizes the mcap
ros2 service call /mower_recorder/snapshot std_srvs/srv/Trigger {}   # flush the heavy buffer now
```

While a recording runs the robot's **rear light breathes red** (`{"effect":"recording"|"off","source":"bag"}` on
`/mower_base/rear_light` with every 2 s status tick). A run stops by itself when the disk has under `min_free_mb`
(2048) free; the status then says `"stop_reason": "disk_low"`.

## data_collection profile (camera + localization for GrassVision)

`mower-data-collection.sh start` writes the module's request file (`~/.mower/recorder_profile.json`); the module
takes `data_collection.yaml` while idle and runs the front camera itself (its `camera_front` instance, GStreamer).
Field procedure (Chinese): `docs/資料收集錄製程序.md`.

| | |
| --- | --- |
| camera | the USB camera's MJPEG as `/camera/front/image_raw/compressed` (no re-encode), 10 Hz via `videorate drop-only` (keeps capture stamps), `camera_info` from `calib/camera_front.yaml`, frame `camera_front_optical_frame` |
| TF | `base_link → camera_front_link → camera_front_optical_frame` static TF from the measured extrinsics (`config/camera_front_extrinsics.yaml` template) |
| `bag/` | telemetry incl. `/gps/status` and the three EKF outputs, MCAP `zstd_fast` chunk compression (no per-message zstd) |
| `camera/` | image + camera_info + `/tf_static`, uncompressed, 256 MB splits |
| metadata | spec 1.6 fields in `run_metadata.yaml` (`config/data_collection_session.yaml` + the request), `calib/` copied into the run |
| upload | `bag_store` uploads every file, then `_manifest.json` (key/size/sha256 + topic counts) last; interrupted uploads resume |
| check | `mower-check-run <run_dir>` |

## Tests

No ROS needed:

```bash
python -m pytest src/mower_recorder/test -q   # needs pytest pyyaml mcap mcap-ros2-support zstandard moto boto3 pillow
```

CI runs `test/test_camera_extrinsics.py` (the others need mcap, moto and boto3).
