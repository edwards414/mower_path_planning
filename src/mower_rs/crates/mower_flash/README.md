# mower_flash

The STM32 firmware flasher, a port of `firmware/tools/mower_flash.py` (E6 of
`docs/ROS_FREE_PLAN.md`). No ROS, no r2r: a library and a binary, in the image
as `/usr/local/bin/mower_flash`. `utils/firmware-sync` runs it instead of the
Python tool at every container start: `MOWER_FIRMWARE_SYNC=rust` (E6-4), and
since E6-5 also `1`, the robot's value, and any other value that names no
flasher. Its log then has one more line, `[firmware-sync] using mower_flash
(rust)`, and its `bundled` line comes from `mower_flash manifest`.
`MOWER_FIRMWARE_SYNC=python` in `/opt/mower/.env` and `docker compose up -d`
bring the Python tool back.

```
mower_flash -p /dev/stmcom sync /opt/mower/firmware/mower_robot_firmware.bin --report ~/.mower/firmware_sync.json
mower_flash -p /dev/stmcom app-info | info | enter | run | flash IMAGE [--no-run]
mower_flash manifest /opt/mower/firmware/mower_robot_firmware.json     # "version sha8"
mower_flash -p /dev/stmcom --trace /tmp/flash.jsonl sync ...          # every frame, one JSON line each
```

## The rule: indistinguishable from mower_flash.py

On the wire and in its results:

- the same frames in the same order with the same seqs; the same timeouts
  (1.5 s / 3 s for 0x87, 0.3 / 0.5 / 0.2 s PINGs, 25 PINGs after 0x0F, 20 s
  ERASE, 2 s WRITE with 3 attempts on a timeout and none on a NAK, 5 s VERIFY,
  2 s RUN_APP), sleeps (0.2 s, 0.3 s) and read cadence: each read waits for 256
  bytes or the end of a `min(remaining, 50 ms)` window, as pyserial's does, so
  a 128-byte chunk still costs about 51 ms (design D2: speed-ups are a later,
  separate change);
- the same flash contents, VERIFY and report: `firmware_sync.json` is
  `json.dump(report, indent=2)` byte for byte, `bundled` holding whatever the
  manifest holds (any JSON type, NaN, big integers, key order);
- the same stdout (`print` and its block buffering on a pipe, so a killed run
  shows what a killed Python run shows) and the same `sys.exit` texts on stderr;
- the same exit codes: sync 0 / 1 / 2, 1 for every error Python catches
  (`error: ...`) and for every one it does not (no report written; stderr
  gets the line its traceback would end with, e.g. `KeyError: 'semver'`,
  `json.decoder.JSONDecodeError: Expecting value: line 1 column 1 (char 0)`,
  `struct.error: unpack requires a buffer of 6 bytes`), 2 for a usage error.

The one deliberate difference: the port is opened exclusively (serialport's
TIOCEXCL and flock, as `mower_base` opens it), so a port someone else holds
fails at once instead of sharing its bytes. A flasher killed by SIGKILL leaves
TIOCEXCL set on a tty that something else still holds open (a pty's other
end); on `/dev/ttyS3` the last close clears it.

Not done the Python way: a lone UTF-16 surrogate in a manifest string
(`"\ud800"`) becomes U+FFFD (a Rust string cannot hold it); `repr()` treats
unassigned code points as printable. Neither occurs in a manifest
`tools/fw_manifest.py` writes.

| module | what |
|---|---|
| `flasher` | `FrameReader` (`mower_base_core::protocol::FrameParser` + a queue), `Bootloader`, `read_app_info`, `do_flash`, `do_sync`, `firmware_matches` |
| `image` | `load_image`, `load_manifest` (UTF-8, universal newlines, CPython's JSON), `os.path.splitext` |
| `pyjson` | `json.loads` with the C scanner's rules and messages, `json.dump(s)`, `str()` / `repr()`, `==` / `list()` / `bool()` / `[:8]` on JSON values |
| `link` | the `Link` and `Clock` traits; `SerialLink`: serialport to open, pyserial's select / read / write semantics on the fd |
| `cli` | argparse's command line, `main()`'s dispatch and exits |

The bootloader protocol itself (constants, payloads, CRC-32) is
`mower_base_core::boot`, checked against `boot_shared.h` and `bl_flash.c`.

## How it is checked

```
cargo test -p mower_flash
```

| test | against |
|---|---|
| `tests/python_oracle.rs` | `tests/vectors/python_oracle.json`, written by `tests/oracle/pyfmt_gen.py` with CPython 3.12 (the image's): 833 float reprs, string reprs, 11 valid and 42 broken JSON texts (dumps, indent=2, str, the exact JSONDecodeError line), `splitext`, 17 `load_image` cases, the **original `do_sync`** on a simulated port and clock for ~100 manifests (wrong types, broken JSON, BOM, CRLF, bad UTF-8; no board / an app / an UNVERSIONED app / `--force`), and on a scripted bootloader for 22 `do_flash` paths the real one never takes (odd `write_chunk_max`, lost / stale / NAKed acks with unknown codes, a short BL_INFO, seq wrapping): the same stdout, report bytes, exit code or exception line, frames and simulated time |
| `tests/pty_e2e.rs` | the binary on a pty against `tools/fake_base.py --bootloader-lib` (the unmodified `bl_protocol.c`), held to the G1 goldens `deploy/test/golden/firmware_sync/` that `mower_flash.py` wrote: needs `python3` and `cc` (skipped without), Linux |
| `tools/flash_check.py` | both tools side by side through every scenario: frames, flash, report, stdout, stderr, exit code, time (CI's mower_rs job; `tools/README.md`, "Checking the firmware flasher") |
