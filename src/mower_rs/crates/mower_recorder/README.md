# mower_recorder

mower_recorder's three rclpy nodes (`src/mower_recorder/mower_recorder/{recorder_manager,bag_store,graph_snapshot}_node.py`)
as the module `bag` of `mower_rsd`: launch switch `rust_recorder`, compose `RUST_RECORDER` (step E5 of
[docs/ROS_FREE_PLAN.md](../../../../docs/ROS_FREE_PLAN.md); not `rust_record` / `RUST_RECORD`, the zone path recorder).

> **E5-3a: idle only. Do not turn `RUST_RECORDER` on before E5-3b.** The module does everything the three nodes do
> while nothing records, but it records nothing: `/mower_recorder/start` answers `success: false` with
> `這版的 Rust 錄製器（RUST_RECORDER，E5-3a）還不能錄製，…`. With the switch on, the app's REC button therefore does
> nothing but show that text. The switch is off by default in `robot.launch.py`, `mission.launch.py`,
> `deploy/docker-compose.yaml` and `deploy/.env.example`, and `test_controller_configuration.py` keeps it so.

## What runs

| Instance | E5-3a | Endpoints (on the bus, to DDS through mower_bridge) |
|---|---|---|
| `graph_snapshot` | complete | `/graph_snapshot`, `/graph_events` (R V 10), the fault (`/mower_recorder/fault`, R V 10) |
| `bag_store` | complete | `/mower_recorder/bags` (R TL 1), `/mower_recorder/command_result` (R V 10); reads `/mower_recorder/command` (R V 10) and `/mower_recorder/status` (R TL 1) |
| `recorder_manager` | idle only | `/mower_recorder/status` (R TL 1), the rear light `/mower_base/rear_light` (R TL 1); reads `/fix` (R V 1) and the fault; serves `/mower_recorder/start`, `/stop`, `/snapshot` (Trigger) |

- **graph_snapshot:** every `period_s` (2 s) the DDS graph as rclpy gives it (`get_node_names_and_namespaces`,
  `get_topic_names_and_types` with each topic's writer and reader counts, `get_service_names_and_types`: vendor/r2r
  patch 3), as `mower_recorder_core::graph`'s JSON; the node diff on `/graph_events` and in the log; the watch rule
  as it is (the watched names lack their slash, so the fault never fires: design 1.12 #1).
- **bag_store:** the run list at the start, every 5 s, after a refresh, a rename and a delete, and around each upload;
  the commands and their answers (`mower_recorder_core::store`); the R2 upload every 30 s on a thread of its own
  (`upload_policy`), `upload_now` on another, the R2 delete with a local delete (manifest first). R2 settings: `R2_*`
  overridden by `r2_env_file`. `r2_endpoint_url` (empty: R2's) is for tests.
- **recorder_manager (idle):** the status (`{"recording": false, ..., "num_topics": 32, "profile": "default", ...}`)
  and the rear light (`{"effect": "off", "source": "bag"}`) at the start and every 2 s; MediaMTX's
  `PATCH /v3/config/paths/patch/front {"record": false}` at the start and on each tick until it worked; stop answers
  `目前沒有在錄製` after a status, snapshot `snapshot 觸發失敗`; the fault and `/fix` are read; `autostart` (false in
  production) is refused after 3 s, in the log.
- **The rear light (design D5)** goes out with path_record_node's through the bridge, to mower_base's r2r reader
  (R TL 8). A source's "recording" holds the overlay 6 s, so the 2 s refresh is what will keep it on in E5-3b.
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
| `deploy/host/mower-data-collection.sh` | as before | `start` refuses: data collection in the module is E5-5 |
| `src/mower_rs/tools/switch.sh` | written false | written true; the processes checked (the module, no Python recorder_manager) |

Roll-back: `RUST_RECORDER=false` in `/opt/mower/.env`, `docker compose up -d`: no new image. It recreates lawan_node,
so do it with the robot idle.

## Tests

- `tests/module.rs` (ROS domain 105, rmw_cyclonedds): the three instances on a manual clock, each of the above, the
  one-owner check against a fake Python server, the rear light read as mower_base reads it and parsed by its
  parser, the app's endpoints through the bridge; every endpoint against mower_bridge's manifest.
- `tests/r2r_patch3.rs` (domain 104): vendor/r2r's patch 3 on DDS (VENDORED.md).

## What E5-3b adds

Recording: the groups (`full`, `snapshot`) with `mower_mcap` writers on niced threads, DDS raw sources
(`subscribe_raw_with_info`, patch 3a), `run_metadata.yaml`, the parameter dump over r2r, the disk guard, MediaMTX's
`record: true` and readiness, the snapshot ring, the start/stop texts; `/mower_full_recorder` (and the snapshot one)
as r2r nodes for the length of a run (D12).
