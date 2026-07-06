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
  the WebRTC/MediaMTX pipeline and correlate by timestamp.
- Write to eMMC/SSD, not microSD.
