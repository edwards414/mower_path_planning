# mower_link

`link_status.json` without Python: what `deploy/host/mower-link-status.py` samples
and writes for the robot container (E6 design 3.4). No ROS. `robot_status` reads
the file into `/robot/telemetry.link`, for Studio and the agent's heartbeat.

| module | what |
|---|---|
| `parse` | the modem's `+CSQ`, `+CREG`, `+COPS`, `+QNWINFO`, `+QCSQ`, `+CPSI`, `/proc/net/wireless`, `ip -j -4 addr`: the Python tool's patterns and its rules about what counts as nothing and what raises |
| `at` | the AT port: raw 8N1 at 115200, non-blocking, one command at a time until `OK`, `ERROR` or 1.5 s |
| `py` | the bits of Python's behaviour the output depends on: `str.strip`, `os.path.join`, `round(x, 1)`, `str(OSError)` |
| `statedir` | the state dir as a root tool has to use it: through the open directory and the open file, following no link |
| `lib` | `lte`, `wifi`, `sample`, `write`: the document, the atomic replace, the state dir's owner on the open file |

The entry point is `mowerctl link-status` (`../mowerctl/README.md`, "The host
tools"). Nothing runs it yet: the host keeps the Python tool until Phase F (E6
design D5).

`tests/golden.rs` holds it to what Python did in each case of
`deploy/test/golden/host/g4_link_status.json`: the same bytes in the file and on
stdout, the same commands on a real pty, and the cases where Python raised and
left the old file. The patterns stay close to the Python ones on purpose, so a
modem's quirks stay the same (the bytes `OK` in an operator's name end the read;
a late answer lands in the next command's window).
