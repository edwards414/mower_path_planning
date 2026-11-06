# What ROS told us, frozen (E9-06)

ROS leaves the robot in E9. These files keep what it said while it was still
here to ask, so that the later steps can be checked with no rcl on the machine:
E9-22 (types), E9-23 (parameters) and E9-24 (logging).

Do not edit them by hand. `crates/mower_ros_facts` holds each file to `SHA256SUMS`,
and to whatever it can be compared with, with no ROS in the environment. `tools/oracle/facts_no_ros.sh`
is that check: nothing sourced, and `ldd` on every test binary.

## The files

| file | what it is | from |
|---|---|---|
| `resolver.json` | Every name of mower_bridge's manifest as rcl resolved it under production's `-r` rules, and the endpoint's type. Only names rcl proved on its graph are written | `mower_bridge/tests/resolver.rs` |
| `param_overrides.json` | For every module node, what `node.params` holds with production's `--params-file` list, with the type r2r read: rcl's parser and r2r's filter, the only source of the parameters of the nodes with no parameter services | `mower_bridge/tests/param_overrides.rs` |
| `param_rules/`, `param_rules.json` | Two files with every form E9-23's reader has to know, and what the same path makes of them | same |
| `cdr_vectors.txt` | E5's G3 frozen. Ten values of every `mower_schema` type (`mower_cdr/tests/support/random_value.rs`): type, index, rmw_cyclonedds's bytes in hex (unpadded), r2r's JSON of the value | `mower_cdr/tests/rmw_oracle.rs` |
| `json_corpus_cdr.txt` | rmw's bytes for the message cases of E3's JSON corpus (`mower_ws_bridge/tests/golden/`), so its frames can be compared with no r2r. No service cases: `mower_schema` has no service types | `mower_ws_bridge/tests/golden_cdr.rs` |
| `graph.json` | The DDS graph of the production stack from outside: every node, topic and service, each endpoint with its node, type, RIHS01 hash and full QoS | `tools/oracle/freeze_stack.py` |
| `params/<node>.yaml` | `ros2 param dump` of each module node that serves parameters | same |
| `launch/args.json`, `launch/<node>.yaml` | What launch gives mower_rsd: `--modules`, the `-r` rules, the `--params-file` list in order. The repo's files are named by their path; the node-scoped files launch writes for each run are here | same |
| `log/startup.txt` | The distinct lines mower_rsd prints in its first minute, sorted | same |
| `log/rosout_loggers.txt` | Every logger that wrote to `/rosout` in that time | same |
| `SHA256SUMS` | The hash of each file above and of E3's corpus (`tools/oracle/facts_manifest.py`) | |
| `PROVENANCE.txt` | The image and commit the stack facts came from. Not in `SHA256SUMS` | `freeze_stack.py` |

Already in the repo and not repeated here: the corpus's texts
(`mower_ws_bridge/tests/golden/`), the `.msg` files and hashes (`mower_schema`),
and what rosbag2 recorded off the wire (`deploy/test/golden/recorder`, which
`mower_ros_facts/tests/recorded_payloads.rs` replays).

## Regenerating

```sh
docker run --rm -v "$PWD":/repo mower-oracle:jazzy tools/oracle/regen.sh facts    # or tools/oracle/regen_facts.sh in the dev container
python3 tools/oracle/freeze_stack.py docker.io/fxrbindi/mower_path_planning:sha-<commit>
```

`.github/workflows/oracle.yml` runs both, monthly and on demand, and fails if
anything changed. CI's `cargo test --workspace` also holds the first group to rcl
and rmw on every PR: a changed manifest or parameter file fails there, with this
command in the message, until its file is written again. The stack's facts have
no such per-PR check: a PR that changes the graph should run `freeze_stack.py`
on its own image.

The log is the exception. Its lines depend on the commit, so a difference there
is a warning on the oracle run, not a failure.

## What is normalised, and why

The repo is public, and a frozen file must not change from one run to the next.
Three runs on the same image, one of them on 1.5 CPUs, gave byte-identical files
with these and nothing else:

| what | becomes | why |
|---|---|---|
| `/home/mower/.mower` | `<MOWER_STATE_DIR>` | a home path; the state dir is compose's constant |
| `/home/mower` | `<HOME>` | a home path |
| `10.77.0.2` | `<ROSBRIDGE_ADDRESS>` | the robot's VPN address, compose's default |
| the fixture's robot id | `<ROBOT_ID>` | in the log only |
| `transform_listener_impl_<address>` | `transform_listener_impl_*` | nav2's node names hold a pointer; the count stays |
| `launch_ros_<pid>` | `launch_ros_*` | launch's node name holds a pid |
| log stamps, decimal numbers, the image's identity | `[T]`, `N`, `<MOWER_VERSION>` | `tools/ab/compare.py`'s `LOG_SUBS` and `Identity`, used as they are |
| temporary file names | `launch/<node>.yaml` | launch's random suffix |

Left out:

- **`--module-args`** of launch's line: the bridge's address and ports, and the
  agent's URLs, which are the command line of two modules, not parameters.
- **This tool's own nodes** and ros2cli's (`ros_facts_*`, `_ros2cli*`), and
  the log lines they cause.
- **Log lines that depend on the machine** (`compare.TIMING`).
- **Run-time values:** there are none to leave out. No QoS or parameter varies,
  and the graph holds no rates or stamps, so the datum and the like are not in
  it. The datum shows in the log only as `N`.

Where the stack differs from a robot, as `tools/ab/README.md` says: fake devices,
no firmware flash, no fleet backend (the agent idles), no MediaMTX (hence the
`video off failed` line in the log), a fresh state dir. So no saved settings
change a parameter.

`tests/frozen.rs` fails on an address other than loopback, a home path, or the
pairing fixtures' secrets anywhere in here but this file.

## What the facts say that E9's design did not

These were found by freezing them, on main 7fdf614 (`PROVENANCE.txt`). From 32b05d2 to that commit, E8-2 (#124) included, the graph, the parameters and the launch line did not change by a byte; the log gained one line.

- **Parameter services (E9-23).** Only five of the module nodes on the graph serve
  them: `/boustrophedon_coverage`, `/map_manage`, `/ekf_filter_node_odom`,
  `/ekf_filter_node_map`, `/navsat_transform`. All are hand-built on r2r nodes, none is
  on the bus. The other 18 (the bus bridge's node among them) have nothing to dump;
  `param_overrides.json` is their record.
- **`/rosout` (E9-24).** No module writes to it. Its writers are nav2's nodes and
  launch: r2r never calls `rcl_logging_rosout_init_publisher_for_node`, so a module's
  log reaches the console only. `log/rosout_loggers.txt` has no module logger, and
  `graph.json` no module node among `/rosout`'s publishers, though `mower_bridge`'s
  manifest and `docs/BUS_TOPICS.md` say every node has that writer.
- **Which section wins (E9-23).** r2r applies a node's matching sections in the order rcl
  first saw them, and the last one wins, so a `/**` after a node's own section
  overrides it (`node_then_wildcard` in `param_rules.json`; rclcpp's source applies `/**`
  first, not run here). A later file that changes a key's type keeps the old type's value
  (`later_file_type`). `yes`, `on` and `y` are booleans, `~` is a string, and `/ns/x`
  and a bare `x` both reach the node `x` in `/ns`. No production file has `/**`.
- **Types (E9-22).** `mower_schema`'s table has all 18 message types the manifest's
  topics use, and its hashes match the graph's. It has no services or actions: the 24
  service and 3 action types in `resolver.json`, and the messages only they use, are not
  in it.
