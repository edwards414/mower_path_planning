# What the Python recorder writes (E5-0 fixtures)

These are runs recorded by today's recorder, `mower_recorder` driving `ros2 bag record` (rosbag2 0.26.11,
libmcap 1.3.1), in the robot image with the production stack running. The Rust recorder (E5) is held to them:
`mower_schema`, `mower_cdr`, `mower_mcap` and `mower_pyfmt` test against these files, and the design note's
G1 compares a rewrite of each run with the run itself.

## How they were made

- **The image:** `mower_path_planning:ros-free-test`, built from f3f4630 (2026-10-24), id `sha256:a399d773…`. Its
  `src/mower_recorder` is identical to main at b2738e4. Each `F*/image_facts.txt` lists the package versions,
  the time zone (UTC) and `ROS_DISTRO`.
- **The run:** `~/work/e2-verify/run.sh record A <run> F1|F2|F2S|F3` (`scripts/record_run.sh`), in a fresh
  container with its own network namespace.
  - It starts the image's own `mower_rsd` with the production modules, `-r` rules and parameter files, and the
    image's `mower.launch.py` peers: nav2, twist_mux and robot_state_publisher.
  - The STM32, IMU and F9P are fakes, the IMU and F9P following `/odom` so nav2 can drive. The zone geometry
    is e2-verify's mission site.
  - The recorder is launched with compose's arguments; MediaMTX is `scripts/mediamtx_stub.py`.
  - The run is driven through `/mower_recorder/start`, `/snapshot` and `/stop`, and copied out.
- **Packing:** `scripts/record_pack.py`. `*.mcap` and the large JSON files are stored as `<name>.zst` (zstd level
  19); the tests decompress them. `MANIFEST.txt` gives the size and sha256 of every original file.
- **Robot id:** `MW-E5FIX` in every run. The run dir name is the start time in UTC (the container's local time).
- **One flaw:** F3's `calib/extrinsics.yaml` still says `source: placeholder`. The harness meant to mark it
  measured but edited the comment line instead, so `mower-check-run` FAILs that one item on this run. The
  bags are not affected.

## The fixtures

| Dir | Profile | What it shows |
|---|---|---|
| `F1` | default (`record_topics.yaml`), 120 s, one `navigate_to_pose` goal at +10 s, one `/mower_recorder/snapshot` at +40 s | per-message zstd; the snapshot group flushed once; `actions/` (below) |
| `F2` | default, `max_bag_size_mb: 1`, `max_bag_duration_s: 10` (`record_topics_split.yaml`), 45 s | five files split by duration; the snapshot group never triggered |
| `F2S` | default, `max_bag_size_mb: 1` only, 100 s | the size split: two files |
| `F3` | `data_collection.launch.py`, 30 s, fake camera (`videotestsrc` 80x60 at 25 fps → `jpegenc` → `videorate` to 10 fps through gscam), a calib dir | `zstd_fast` chunk compression; the uncompressed camera group; gscam's and the static TF publishers' QoS; spec 1.6 metadata |

Next to each run dir:

| File | What it holds |
|---|---|
| `replies.txt` | the start, snapshot and stop replies |
| `qos.json.zst` | every topic's publishers and subscriptions, with node, type hash, GID and full QoS, 20 s into the run (`record_qos.py`) |
| `listen.jsonl.zst` | every `/mower_recorder/{status,bags,command_result}`, `/mower_base/rear_light` and `/graph_*` message, verbatim |
| `mediamtx.jsonl.zst` | the requests MediaMTX got |
| `recorder_warnings.txt` | the recorder's WARN and ERROR lines, and the compression-queue drops counted per topic |
| `zstd_check.json` | (F1, F2, F2S) F6: every per-message zstd payload, decompressed and compressed again with libzstd 1.5.5 at level 1 |
| `actions/` | (F1) `ros2 bag record --include-hidden-topics` of the two `/navigate_to_pose/_action/*` topics during the goal, by a side recorder that is not part of the run |

## What they show

Where the design note's 2.2 had to guess, the captures answer it.

- **File layout.**
  - The header has profile `ros2` and library `libmcap 1.3.1`.
  - The data section holds the header, the open-time `rosbag2` metadata record, the chunks (each followed by its
    message indexes), the final `rosbag2` record, then DataEnd with CRC 0.
  - Chunk CRCs are 0 (rosbag2 sets `noChunkCRC`); the summary CRC is set.
  - A Schema and a Channel record go into the chunk just before a channel's first message, once per file.
  - The summary lists every channel created, with or without messages. Statistics `channel_count` and
    `schema_count` count only those written in the data section.
  - Three record orders follow libstdc++'s `unordered_map` iteration order, not channel order: the message
    indexes after a chunk, Statistics' per-channel counts and a ChunkIndex's offsets. The same holds for the
    topics of the final metadata record, which iterates rosbag2's `unordered_map<std::string, …>`. Given the
    order the topics were created in (the channel ids), an emulation of that hash table reproduces all four
    orders exactly, `metadata.yaml` included (F1: 29 topics, through a rehash).
- **Per-message zstd** (default profile).
  - Chunks stay uncompressed; each payload is one zstd frame (single segment, content size, no checksum).
  - Every payload (F1 14 503, F2 5 555, F2S 11 828) comes out byte-identical when compressed again by libzstd
    1.5.5 at level 1.
  - `compression_format: zstd`, `compression_mode: MESSAGE`; files are still `bag_<n>.mcap`.
  - **The compression queue drops messages.** `ros2 bag record`'s `--compression-queue-size` is 1. When the
    `hardware_concurrency()` compression threads fall behind, the oldest queued message is discarded with a
    WARN. F1 lost 215 messages that way in 120 s on this Mac (10 threads), a latched `/tf_static` sample and
    two `/adapter/map_layers/map_grid` ones among them; runs sharing the CPU with two other stacks lost up to
    488. The RK3568 has 4 threads.
  - The threads write in completion order: 715 of F1's messages have a `log_time` earlier than the one before
    them (by at most 7.6 ms; F2S up to 75 ms).
- **`zstd_fast`** (F3) is chunk compression, zstd at libmcap's `Fastest` (level -5). The chunks are the default
  768 KiB, not 4 MiB (that is `zstd_small`). The camera group's chunks are uncompressed.
- **Splits.**
  - The duration rule compares a message's receive time with the file's first and is deterministic (F2).
  - The size rule compares the bytes libmcap has flushed, so it can act only after a chunk is written. Because
    rosbag2 checks at push time and writes from its cache thread, at least one more message reaches the old
    file: F2S's first file ends with a one-message chunk. Size-split boundaries are therefore timing-dependent.
  - Every new file re-creates every channel. Its open-time record lists all topics in creation order with
    count 0; so does the final record of every file but the last.
- **Snapshot mode.**
  - Never triggered (F2, F2S): `snapshots_0.mcap` has its channels and no messages.
  - Triggered (F1): the ring buffer holds everything since the start, up to 256 MiB, and is written to
    `snapshots_0.mcap`. rosbag2 then splits to `snapshots_1.mcap`, which stays empty: what is buffered after the
    trigger is discarded at stop. In this run only the two costmaps had messages; `/plan` had none.
- **Action topics.**
  - The recorder never records `/navigate_to_pose/_action/feedback` or `/status`, although `record_topics.yaml`
    lists them: rosbag2 skips hidden topics without `--include-hidden-topics` (`recorder_warnings.txt`).
  - When they are recorded (`actions/`), their Schema records have an empty encoding and empty data. rosbag2
    Jazzy has no definition for action types, nor for `action_msgs/msg/GoalStatusArray`. The channels still
    carry the publisher's type hash.
- **QoS.**
  - A channel's `offered_qos_profiles` holds the publishers known when the topic was created: one entry each,
    in rmw's order. A writer that appears later is not added.
  - The recorder's reader is the override if there is one. Otherwise it is reliable if every offer is,
    transient local if every offer is, and `keep_last(max offered depth)` if that depth is above 1, else
    `keep_all` (rosbag2_storage qos.cpp, `adapt_request_to_offers`).
- **`metadata.yaml`** is `rosbag2_bagfile_information:` followed by the final record's YAML indented by 2, byte
  for byte, with no final newline.
- **Payloads** are CDR little endian (`00 01 00 00`), zero-padded to a multiple of 4.
