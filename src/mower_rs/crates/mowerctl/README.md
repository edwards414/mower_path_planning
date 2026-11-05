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

# on the robot's host, no connection: see "The host tools" below
mowerctl pair [--rotate] [--name NAME] [--json] [--no-qr] [--state-dir DIR] [--owner USER] [--env FILE]
mowerctl link-status [STATE_DIR] [--once]
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
`param list/get` arrive.

## The host tools

`pair` and `link-status` need no connection. They are the Python tools of
`deploy/host/`, `mower-pair` and `mower-link-status.py`, for when the host has
no Python (E6 design 3.4, 3.5). **Nothing installs them on the host yet**: E6-8,
if the owner wants it before Phase F, does that with a drop-in; the image has
the binary because it has mowerctl. Only these two write anything: `pair` writes
`identity.json` when it is new, broken, rotated or renamed, and `link-status`
writes `link_status.json` every 5 s.

They must not be told apart from the Python tools. Flags, defaults, files, modes,
owners, stdout and exit codes are held to what Python did in every case of
`deploy/test/golden/host/g5_pair.json` and `g4_link_status.json`, written by
`src/mower_mission/test/test_host_contracts.py`:

- `pair`: argparse's rules included (`--rot`, `--name=x`, exit 2), `quote_plus`'s
  escaping in the URL (`~` stays, `*` does not), `\uXXXX` for non-ASCII in
  `--json` and the file, nothing written without a reason. So also what design
  D6 leaves for the owner to decide: **an `identity.json` that does not parse is
  silently replaced, and its `device_key` is lost**, after which the backend
  answers 409; and ws_bridge sees a new secret only after a restart.
- Both work through the open directory and the open file, as the fixed Python
  tools do (`mower_link::statedir`): no link is followed in the state dir or at
  its name, and an owner or a mode is set on the file, not by name.
- `link-status`: the same AT commands and the same bytes written. `rsrp_dbm` and
  `rsrq_db` stay ints from Quectel's `+QCSQ` and floats from SIMCom's `+CPSI`. The
  first argument not starting with `--` is the state dir. Run one sampler at a
  time, in place of the Python one: two on `/dev/lte_at` read each other's answers.

What is different, and why it does not matter:

- `pair` looks the owner up in `/etc/passwd` rather than NSS, which keeps the
  binary static; a user only LDAP knows counts as unknown, whose chown is skipped.
- `--help` is its own text, and `link-status` has one (Python's treats it as an
  unknown flag and starts sampling). `pair` and `link-status` say why they failed
  in one line, where Python prints a traceback; the exit code is the same.
- A modem that goes away during a read: Python's `at()` now and then dies of a
  negative `select` timeout at its deadline and loses that round (pinned in G4's
  tests). The loop here has no such race, so the round always records the error
  of the write that follows, as Python does the rest of the time.
- A number too long for 64 bits in a modem's answer reads as no answer, and
  digits outside ASCII are not digits. A modem sends neither. In `identity.json`
  such a number makes the file one that does not parse, and a lone surrogate
  escape comes back as U+FFFD (`mower_pyfmt`'s gaps). Neither writer makes those.
- `MOWER_LINK_WIRELESS` replaces `/proc/net/wireless`, for the tests.

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
  needed: it has no r2r dependency, which `tests/host_tools.rs` checks against
  `Cargo.lock`). Nothing in it links C, so a static
  binary for a host without the image's glibc builds as well (3.2 MB on
  arm64; glibc's linker warns that a host name then resolves only with the
  same glibc's NSS libraries, so give such a binary an IP address):

  ```bash
  RUSTFLAGS='-C target-feature=+crt-static' \
    cargo build --release -p mowerctl --target aarch64-unknown-linux-gnu
  ```

  CI's arm64 job builds it this way. `pair` looks users up in `/etc/passwd` for
  the same reason, and the AT port sets its speed without `cfsetispeed`: the libc
  crate binds that to a versioned glibc symbol a static link cannot have.

## Tests

`cargo test -p mowerctl` runs the commands against an in-process fake bridge
(`src/fake_bridge.rs`: ws_bridge's frames and error texts, the gate's MAC
check) as well as the units: the MAC against the Python vector, the
`ros2 topic hz` arithmetic, a message's bytes taken from the frame as they
stand (no serde_json feature of its own: those would be unified into the
image's other binaries).

The host tools run over the goldens: every G5 case through `pair`'s code with
the machine replaced (`src/pair.rs`), and in `tests/host_tools.rs` the real
binary over every G4 case, with the modem on a real pty, and over the G5 cases
the machine cannot change, on a terminal as well as a pipe; side by side with
the Python tools on this machine when there is a python3.
