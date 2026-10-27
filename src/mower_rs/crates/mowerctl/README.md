# mowerctl

A command-line look at a running robot through ws_bridge, without ROS: no
`ros2` CLI, no DDS participant, no discovery (E8 design §2; on the RK3568 a
`ros2 topic hz` or `node list` has caused overruns, `docs/ROS_FREE_PLAN.md`).
It speaks the rosbridge v2 subset the app speaks, `subscribe` and
`call_service`, so it sees what the listener's policy serves and costs the
robot what one more app client costs.

```text
mowerctl topic list [-v] [--json]
mowerctl topic info NAME [--json]
mowerctl topic echo NAME [-n N] [--throttle MS] [--field A.B] [--no-arr] [--yaml|--raw] [--timeout S]
mowerctl topic hz NAME... [--window N] [--duration S] [--json]
mowerctl service list [--json] [--policy FILE]
mowerctl service call NAME [ARGS] [--yes] [--raw] [--timeout S]
mowerctl ping [-c N] [-i S]
mowerctl help
```

Read-only by default. There is no `topic pub` and no `param set` (design
D2): the four topics the app may publish are actuators (`/app_joy_cmd`, the
blade, the servo, the recorder command), and from a shell they would bypass
the app's dead-man flow. `service call` reaches only what the policy allows,
like the app, and calls a service that is not a known query (anything but
`/rosapi/*`, `/check_nav_status`, `/coverage_progress_status` and parameter
reads) only with `--yes`: `/system/restart`, `/zone_exec_path` and the rest
change the robot's state.

## Connecting

| how | URL | gate |
|---|---|---|
| default | `ws://127.0.0.1:9091`, ws_bridge's loopback listener | none (it is bound to 127.0.0.1 only) |
| `--pair FILE` | `ws://127.0.0.1:9090`, the pairing gate | the app's X-Mower-* HMAC hand-shake with `FILE`'s `robot_id` and `secret` (the robot's own `~/.mower/identity.json` works); the gate wants the two clocks within ±60 s (`mowerctl ping` shows the offset) |
| `--url URL` (or `$MOWERCTL_URL`) | any | as above, by `--pair` |

On the robot, with the image's binary:

```bash
sudo -n docker exec mower-lawan_node-1 mowerctl topic hz /robot/telemetry
```

From another machine, SSH does the authentication:

```bash
ssh -N -L 19091:127.0.0.1:9091 cat@<robot> &
mowerctl --url ws://127.0.0.1:19091 topic list -v
```

A slow link drops messages rather than delaying them: ws_bridge queues 512
frames per client and drops the rest. `--throttle` (done by the bridge)
keeps `echo` within a link's means.

## What it sees today

The loopback listener applies the public listener's policy
(`src/mower_bringup/config/ws_bridge.yaml`), so these are the app's topics
and services:

- `topic list` / `info` ask `/rosapi/topics`, which ws_bridge answers itself:
  the topics that something publishes and the policy allows, with their
  types.
- `topic echo` / `hz` subscribe without a type, so ws_bridge looks the type
  up and refuses a name nothing publishes (exit 3) instead of making a
  reader for a typo.
- `service list` prints what the policy allows. No call of today's bridge
  lists services, so it reads the policy built into the binary (the
  `ws_bridge.yaml` of the tree it was built from, which for the image's
  binary is the image's) or `--policy FILE`; a call shows whether a server
  is up.

E8-2 gives the loopback listener its own policy (read everything that
exists, write what the app may) and the bridge's own answers (`/mowerd/*`):
`topic info` then shows writers, readers, QoS and counters, `hz` reads the
bus's `published` counters by default (no subscription; `--jitter` for
the subscribing mode of today), and `node`, `bridge`, `graph` and
`param list/get` arrive. E6 adds `pair` and `link-status`, which need no
connection.

## Output

- `topic echo`: one JSON line per message, `{"t": <arrival, unix s>,
  "topic": ..., "msg": ...}`. `msg` is what ws_bridge sent the app, byte for
  byte: keys sorted, an `f32` widened to the `f64` it was sent as, NaN as
  `null`. `--raw` prints the frames as received, `--yaml` YAML documents for
  people, `--field A.B` that field alone (array indices are numbers),
  `--no-arr` every array as `"<N items>"` (a `/map_grid` message is about
  0.6 MB of JSON).
- `topic hz`: one line per second, the fields of `ros2 topic hz` (average
  rate, min, max, std dev, window; receive times, intervals over the last
  `--window`); the topic name in front with several topics; `--json` one
  JSON object per report for scripts.
- `topic list` / `info` / `service list`: plain columns; `--json` as
  answered.
- `service call`: the answer's `values`, pretty-printed; `--raw` the whole
  `service_response`.

## Exit codes

| code | meaning |
|---|---|
| 0 | done (also when stdout's reader went away, `| head`) |
| 1 | failed: bad arguments, a call answered with an error, a state-changing service without `--yes` |
| 2 | no connection: refused, closed by the bridge, or rejected by the gate (401) |
| 3 | no such topic or service on this listener (nothing publishes it, no server answers, or the policy does not allow it) |
| 4 | timed out: no answer, fewer than `-n` messages within `--timeout`, or no rate within `hz --duration` |

## Where it lives

- In the runtime image, built and installed with the other mower_rs binaries
  (`MOWER_RS_BINARIES` in `../../CMakeLists.txt`):
  `/mower_ws/install/mower_rs/lib/mower_rs/mowerctl`, and on the `PATH` as
  `/usr/local/bin/mowerctl` (a symlink, `Dockerfile`). No launch file,
  compose service or unit starts it.
- Anywhere else: `cargo build --release -p mowerctl` (no ROS environment
  needed: it has no r2r dependency). Nothing in it links C, so a static
  binary for a host without the image's glibc builds as well (2.5 MB on
  arm64; glibc's linker warns that a host name then resolves only with the
  same glibc's NSS libraries, so give such a binary an IP address):

  ```bash
  RUSTFLAGS='-C target-feature=+crt-static' \
    cargo build --release -p mowerctl --target aarch64-unknown-linux-gnu
  ```

## Tests

`cargo test -p mowerctl` runs the commands against an in-process fake bridge
(`src/fake_bridge.rs`: ws_bridge's frames and error texts, the gate's MAC
check) as well as the units: the MAC against the Python vector, the
`ros2 topic hz` arithmetic, a message's bytes taken from the frame as they
stand (no serde_json feature of its own: those would be unified into the
image's other binaries).
