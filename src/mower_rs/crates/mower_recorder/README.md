# mower_recorder

mower_recorder's three rclpy nodes (`src/mower_recorder/mower_recorder/{recorder_manager,bag_store,graph_snapshot}_node.py`)
as the module `bag` of `mower_rsd`: launch switch `rust_recorder`, compose `RUST_RECORDER` (step E5 of
[docs/ROS_FREE_PLAN.md](../../../../docs/ROS_FREE_PLAN.md); not `rust_record` / `RUST_RECORD`, the zone path recorder).

> **E5-3b: it records** the default profile from DDS, as the Python recorder and its `ros2 bag record` children did.
> The switch stays off by default (`robot.launch.py`, `mission.launch.py`, `deploy/docker-compose.yaml`,
> `deploy/.env.example`; `test_controller_configuration.py` keeps it so) until the owner has run the robot checks
> (below) and E5-7 makes it the default. **E5-5:** data_collection too, by the request file
> `~/.mower/recorder_profile.json` that `mower-data-collection.sh start` writes (and `stop` deletes). **E5-6:** the
> front camera too (`camera_front`, GStreamer through gstreamer-rs, the request's `camera.driver: gstreamer`, which
> the helper writes by default); `CAMERA_DRIVER=gscam` keeps E5-5's camera-only gscam service.

## What runs

| Instance | State | Endpoints (on the bus, to DDS through mower_bridge) |
|---|---|---|
| `graph_snapshot` | complete | `/graph_snapshot`, `/graph_events` (R V 10), the fault (`/mower_recorder/fault`, R V 10) |
| `bag_store` | complete | `/mower_recorder/bags` (R TL 1), `/mower_recorder/command_result` (R V 10); reads `/mower_recorder/command` (R V 10) and `/mower_recorder/status` (R TL 1) |
| `camera_front` | E5-6, only while the request file asks for `camera.driver: gstreamer` | `/camera/front/image_raw/compressed`, `/camera/front/camera_info` (R V 1, DDS only); its nodes `camera_front`, `camera_front_link_tf` and `camera_front_optical_tf` (`/tf_static`, R TL 1, one sample each). Nothing on the graph otherwise |
| `recorder_manager` | complete for the default profile (E5-3b) | `/mower_recorder/status` (R TL 1), the rear light `/mower_base/rear_light` (R TL 1); reads `/fix` (R V 1) and the fault; serves `/mower_recorder/start`, `/stop`, `/snapshot` (Trigger). While a run records: the groups' nodes `mower_full_recorder`, `mower_snapshot_recorder` and their DDS subscriptions |

- **graph_snapshot:** every `period_s` (2 s) the DDS graph as rclpy gives it (`get_node_names_and_namespaces`,
  `get_topic_names_and_types` with each topic's writer and reader counts, `get_service_names_and_types`: vendor/r2r
  patch 3), as `mower_recorder_core::graph`'s JSON; the node diff on `/graph_events` and in the log; the watch rule
  as it is (the watched names lack their slash, so the fault never fires: design 1.12 #1).
- **bag_store:** the run list at the start, every 5 s, after a refresh, a rename and a delete, and around each upload;
  the commands and their answers (`mower_recorder_core::store`); the R2 upload every 30 s on a thread of its own
  (`upload_policy`), `upload_now` on another, the R2 delete with a local delete (manifest first). R2 settings: `R2_*`
  overridden by `r2_env_file`. `r2_endpoint_url` (empty: R2's) is for tests.
- **recorder_manager:** `mower_recorder_core::manager::Manager` (E5-3b, its own tests replay F1 and F3 call for call)
  behind the node: start, stop and snapshot with the Python's replies; the status and the rear light (`"recording"`
  while a run goes) at the start and every 2 s, with the disk guard; MediaMTX (`record: false` at the start and its
  retry, `record: true` into `<run>/video/` and the readiness note); `run_metadata.yaml`; the parameter dump of
  `params_dump_nodes` through `ListParameters` / `GetParameters` clients on the node (`ros2 param dump`'s format);
  the fault flushes the snapshot ring; `autostart` (false in production) after 3 s. Each manager call runs on a
  blocking thread, one at a time. A run still going when the instance stops, or steps back for a foreign
  recorder, is stopped and finalized.
- **The sources** (`sources.rs`): a thread per group with the group's node (`mower_full_recorder`,
  `mower_snapshot_recorder`: D12), discovery every 100 ms as rosbag2's, a name subscribed once it has a writer and one
  type, with `reader_qos` (the override, else adapted to the offers), the offers and the type hash taken then; every
  message to the group's `Feed` with rmw's receive and source times (vendor/r2r patch 3, every ready message taken at
  a wakeup); hidden names never; a name in two groups subscribed once. The writer threads (`bag-full`,
  `bag-snapshot`, niced) are `mower_recorder_core::recording`'s.
- **The rear light (design D5)** goes out with path_record_node's through the bridge, to mower_base's r2r reader
  (R TL 8). A source's "recording" holds the overlay 6 s; the 2 s refresh keeps it on while a run records.
- **One owner (design 4.2, R6):** each instance looks 2 s after its start and every 10 s for a DDS server of
  `/mower_recorder/start` that is not a bus bridge's (the Python recorder). While there is one it holds none of its
  endpoints, and says so in an ERROR; when it is gone it serves.
- **Parameters** are what record.launch.py gives the Python nodes: the fixed ones in `mower_rsd.yaml`, the per-robot
  ones (`robot_id`, `output_root`, `r2_env_file`, the profile in mower_recorder's share, `git_repo_dir`) from
  robot.launch.py's arguments. The three r2r nodes have no parameter services (as the other modules); the Python
  nodes' do.

## The switch

| Where | Off (default) | On |
|---|---|---|
| `deploy/docker-compose.yaml` | lawan_node gets `rust_recorder:=false`; the `recorder` service runs `ros2 launch mower_recorder record.launch.py` as before (a `bash -c` in front that `exec`s it) | the `recorder` service sleeps (`sleep infinity`) |
| `robot.launch.py` | `bag` is not in `--modules` | `bag` last in `--modules`, with the recorder's parameter files |
| `mission.launch.py` (`rust_daemon:=false`) | nothing | the `mower_recorder` binary (`mower_bridge::module_main("bag", "bag", ...)`: the three nodes and a bridge, `/mower_bus_bridge/bag`) |
| `deploy/host/mower-data-collection.sh` | as before | `start` writes the request file, `stop` deletes it (E5-5). `CAMERA_DRIVER=gstreamer` (the default, E5-6): nothing else starts, the module runs the camera, and the helper's `ros2` commands run in lawan_node. `CAMERA_DRIVER=gscam`: `start` also starts the camera-only service (`data_collection_camera`, front_camera.launch.py) and `stop` stops it |
| `src/mower_rs/tools/switch.sh` | written false | written true; the processes checked (the module, no Python recorder_manager) |

Roll-back: `RUST_RECORDER=false` in `/opt/mower/.env`, `docker compose up -d`: no new image. It recreates lawan_node,
so do it with the robot idle.

## Tests

- `tests/module.rs` (ROS domain 105, rmw_cyclonedds): the three instances on a manual clock, each of the above, a run
  recorded from DDS (the groups' nodes and subscriptions, a latched sample from before the start, the payloads byte
  for byte, no hidden or unpublished name, the snapshot flush, run_metadata.yaml, the parameter dump), the one-owner
  check against a fake Python server, the rear light read as mower_base reads it and parsed by its parser, the app's
  endpoints through the bridge; every endpoint against mower_bridge's manifest. E5-5: the request file (the
  data_collection status, the camera group from DDS, the camera block and the derived extrinsics of F3's
  calibration and URDF, the default again once the file is gone). `mower_recorder_core/tests/dc_request_port.rs`:
  the request file, the pipeline builder, the placeholder as PyYAML wrote it, F3's camera parameters back from its
  calibration. E5-6 (`--features camera`, needs GStreamer's base and good plugins): `camera_front` on a
  `videotestsrc` pipeline from a request file (its three nodes, frames that are JPEGs, the stamps, F3's CameraInfo,
  the QoS, both transforms; gone with the file). `mower_camera`'s own tests: the stamp formula and the calibration
  file as camera_info_manager reads it.
- `tests/r2r_patch3.rs` (domain 104): vendor/r2r's patch 3 on DDS (VENDORED.md).

## data_collection (E5-5)

- **The request file** (design D7) `~/.mower/recorder_profile.json` (`$MOWER_STATE_DIR`, or the node's
  `profile_request_file`): `mower-data-collection.sh start` writes it (the profile, the camera's launch arguments as
  compose passes them to the camera, `session_file`, `upload_policy`), `stop` deletes it.
  `mower_recorder_core::dc_request` reads it; a key it does not know or another profile is refused (logged once, the
  profile stays).
- **recorder_manager** looks at it when it starts serving and on every 2 s tick, and applies a change while no run
  records: the manager is made again with `data_collection.yaml` (next to `config_file`), the EKF and navsat
  parameter dumps, and the camera parameters `dc_request::resolve` works out as data_collection.launch.py did:
  `build_pipeline` (or `gscam_pipeline`), the calibration's intrinsics or the uncalibrated placeholder (written to the
  temp dir as the launch wrote it), the calibration's extrinsics or the package's placeholder,
  `camera_extrinsics.compute` on robot_state_publisher's latched `/robot_description` (3 s; else the built-in
  geometry, said), the session file or the package's template. The launch's warnings are logged as
  `[data_collection] WARNING: ...`. The status says `"profile": "data_collection"`; the camera group records
  `/camera/front/image_raw/compressed`, `camera_info` and `/tf_static` from DDS, and the run gets spec 1.6's
  `run_metadata.yaml` and `calib/` (`dc_metadata`). The file's going brings the default profile back.
- **bag_store** takes the file's `upload_policy` while it is there (said once), its own otherwise.
- **The camera:** the module's `camera_front` (E5-6, below) when the request says `camera.driver: gstreamer`;
  otherwise gscam, the data-collection compose's `data_collection_camera` (front_camera.launch.py: gscam and the two
  static TFs), in the profile `camera_only`, which the helper starts by name.

## The camera (E5-6)

`camera_front` (`src/camera.rs`, on the crate `mower_camera`) is front_camera.launch.py in the module: gscam
2.0.2 and its two `static_transform_publisher`s, for the data-collection front camera (design 4.7, D6).

- **When:** it looks at the request file every 2 s. A request with `camera.driver: gstreamer` starts it; another
  request, or none, stops it (its nodes leave the graph, `/dev/video0` is free again for mower-camera.service).
  Without a request nothing of it is on the graph; the default profile is as before.
- **What it works out** as recorder_manager does (`dc_request::resolve`): the pipeline (`build_pipeline`, or the
  request's `gscam_pipeline`), the calibration file or the placeholder (written as the launch wrote it), the two
  transforms from the calibration's extrinsics and the URDF. The launch's warnings are logged as
  `[data_collection] WARNING: ...`.
- **The stream** (`mower_camera::stream`, gstreamer-rs 0.25 on the image's GStreamer 1.24): gscam's
  `gst_parse_launch` of the pipeline, an appsink for `image/jpeg` with `sync` off, PAUSED then PLAYING; each
  sample's first memory is the `CompressedImage` (`format: jpeg`, `frame_id: camera_front_optical_frame`), stamped
  `pts + base_time + offset` with the offset (ROS time minus GStreamer's system clock) taken when the stream opens;
  then the `CameraInfo` from the calibration file as camera_info_manager read it (`mower_camera::info`), with the
  same header. Both publishers keep last 1, reliable, volatile (`rclcpp::QoS(1)`). The end of a stream opens it
  again (`reopen_on_eof`). gscam did not do this: an error on the bus ends the stream too and it opens again 3 s later
  (launch respawned a dead gscam after 3 s; a hung one stayed hung).
- **The transforms:** `camera_front_link_tf` (`base_link` to `camera_front_link`) and `camera_front_optical_tf`
  (to `camera_front_optical_frame`), each one sample on `/tf_static` (keep last 1, reliable, transient local),
  stamped when published, with the values static_transform_publisher parsed from `repr(round(v, 9))`.
- **The build:** GStreamer is behind the cargo feature `camera` (`mower_rsd/camera`, `mower_recorder/camera`,
  `mower_camera/gst`). The image builds `mower_rsd` with it (`src/mower_rs/CMakeLists.txt`); its libraries and
  plugins are already in the image (gscam's). Built without it, a request for the camera is refused in the log
  ("this build has no camera"), once per request.
- **The checks (e2-verify):** `camproof`: gscam and `camera_front` on the same deterministic pipeline give the same
  JPEG bytes frame for frame, the same stamp steps, the same CameraInfo and the same publisher QoS. `record W F3` with
  `E5_CAMERA=gstreamer`: the Python recorder and the module record the module's camera identically; against E5-5's
  gscam run the camera bag's QoS, types, CameraInfo and both camera transforms are the same (stamps aside).
- **Roll-back:** `CAMERA_DRIVER=gscam` in `/opt/mower/.env` (E5-5's camera-only service), no new image.

## What comes after E5-6

- **E5-4:** bus taps (`source: auto`) for the names whose writers are all in the process, instead of their DDS faces
  (branch `feat/e5-4-bus-taps`, paused).
- **E5-7:** the default and the removal of the Python recorder (and of gscam's compose service).

## Turning it on (the owner, on the robot)

1. R10: one upload of a Rust-recorded run to the real R2 (`upload_now` from the app with `RUST_RECORDER=true` in a
   test window, or `mower-bag upload` of a copied run), then `mower-bag verify` and the GrassVision worker on it.
2. A supervised recording test with the switch on (`tools/switch.sh ... RUST_RECORDER=true`): start and stop from
   the app, the rear light, the bag list and `mower-check-run` on the run, then `switch.sh restore`.
