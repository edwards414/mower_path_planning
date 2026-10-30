#!/usr/bin/env python3
"""Vectors for mower_flash's Python-compatibility, from the real Python.

    python3 tests/oracle/pyfmt_gen.py > tests/vectors/python_oracle.json

Run it with the Python the robot image has (3.12; `docker run --rm -v
$PWD:/w -w /w mower_path_planning:ros-free-test python3 ...`, or any 3.12),
TZ=UTC. It writes:

* `float_repr`, `str_repr`: repr() of doubles (edge cases + random bit
  patterns) and of strings;
* `loads`: json.loads of valid texts -> json.dumps(v), json.dumps(v,
  indent=2), str(v); of broken ones -> the JSONDecodeError line;
* `splitext`: os.path.splitext(p)[0];
* `sync`: the original mower_flash.py's do_sync on a simulated port and clock
  for ~100 manifests (odd values, wrong types, broken JSON, BOM, CRLF, bad
  UTF-8), each with no board and with an app that answers 0x06 with 0.6.0
  e6e6e6e6 and ignores the rest (the canonical manifest also with --force and
  with that app UNVERSIONED): stdout, the report bytes, the exit code or the
  exception line, every frame sent;
* `load_image`: load_image() of odd files: the padded length and CRC, or the
  sys.exit text / exception line;
* `flash`: do_sync from the bootloader on a scripted bootloader (BlPort) for
  what the real one never does: odd write_chunk_max, lost / stale / NAKed
  acks with unknown status codes, a short BL_INFO, seq wrapping.

The simulated port is pyserial's read rule: what is pending, at once if it is
`n` bytes or more, otherwise at the end of the timeout. The Rust test
(tests/python_oracle.rs) replays every case on the same simulation.
"""

import contextlib
import hashlib
import importlib.util
import io
import json
import math
import os
import random
import struct
import sys
import tempfile
import time
import time as _time
import traceback
import types
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", "..", ".."))
T0 = 1000.0
WALL = 1700000000.5


def load_mower_flash():
    sys.dont_write_bytecode = True
    stub = types.ModuleType("serial")
    stub.Serial = object
    sys.modules.setdefault("serial", stub)
    spec = importlib.util.spec_from_file_location("mower_flash", os.path.join(REPO, "firmware", "tools", "mower_flash.py"))
    mf = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mf)
    return mf


class Clock:
    def __init__(self):
        self.t = T0

    # the parts of the time module mower_flash.py uses
    def monotonic(self):
        return self.t

    def sleep(self, s):
        self.t += s

    def time(self):
        return WALL

    strftime = staticmethod(_time.strftime)
    localtime = staticmethod(_time.localtime)


APP_INFO = struct.pack("<BBBBIIBBH", 0, 6, 0, 1, 0xE6E6E6E6, 1793000000, 0, 0, 0)


class Port:
    """pyserial's read rule on a simulated board."""

    def __init__(self, clock, board, mf):
        self.clock = clock
        self.board = board
        self.mf = mf
        self.timeout = 0.05
        self.pending = bytearray()
        self.writes = []

    def write(self, data):
        data = bytes(data)
        self.writes.append(data.hex())
        ftype, seq = data[3], data[4]
        if ftype == 0x06 and self.board in ("app", "unversioned"):
            payload = bytearray(APP_INFO)
            if self.board == "unversioned":
                payload[12] = 0x02
            self.pending += self.mf.build_frame(0x87, 0, bytes(payload))
        return len(data)

    def read(self, n):
        if len(self.pending) >= n:
            out = bytes(self.pending[:n])
            del self.pending[:n]
            return out
        out = bytes(self.pending)
        self.pending.clear()
        self.clock.t += self.timeout
        return out


class BlPort(Port):
    """A scripted bootloader (every reply at once) that becomes the app after
    RUN_APP: for do_flash's paths the real one never takes."""

    def __init__(self, clock, mf, cfg):
        super().__init__(clock, "bl", mf)
        self.cfg = cfg
        self.writes_seen = 0
        self.in_app = False

    def reply(self, ftype, seq, payload):
        self.pending += self.mf.build_frame(ftype, seq, payload)

    def ack(self, cmd, seq, status, value):
        self.reply(0x91, seq, struct.pack("<BBI", cmd, status, value))

    def write(self, data):
        data = bytes(data)
        self.writes.append(data.hex())
        ftype, seq, payload = data[3], data[4], data[6:-2]
        cfg = self.cfg
        if self.in_app:
            if ftype == 0x06:
                self.reply(0x87, 0, APP_INFO)
            return len(data)
        if ftype == 0x10:
            info = struct.pack("<IIHBBI", 0x08008000, 0x58000, cfg.get("chunk", 128), 1, 0x01, 0)
            self.reply(0x90, seq, info[: cfg.get("info_len", 16)])
        elif ftype == 0x11:
            self.ack(0x11, seq, cfg.get("erase_status", 0), struct.unpack("<I", payload)[0])
        elif ftype == 0x12:
            self.writes_seen += 1
            n = self.writes_seen
            off = struct.unpack("<I", payload[:4])[0]
            if n in cfg.get("drop", []):
                return len(data)
            if n in cfg.get("stale", []):
                self.ack(0x12, (seq - 1) & 0xFF, 0, off)  # a late ack of an earlier frame
                self.ack(0x11, seq, 0, off)  # right seq, wrong command
            self.ack(0x12, seq, cfg.get("nak_status", 0) if n == cfg.get("nak_at") else 0, off)
        elif ftype == 0x13:
            size, crc = struct.unpack("<II", payload)
            self.ack(0x13, seq, cfg.get("verify_status", 0), cfg.get("verify_value", crc))
        elif ftype == 0x14:
            self.ack(0x14, seq, cfg.get("run_status", 0), 0x08008000)
            self.in_app = not cfg.get("run_status")
        return len(data)


def flash_vectors(mf):
    """do_sync from the bootloader through do_flash, on BlPort."""
    configs = [
        ("default", 300, {}),
        ("chunk 64", 300, {"chunk": 64}),
        ("chunk 6 -> 4", 40, {"chunk": 6}),
        ("chunk 3 -> 0", 40, {"chunk": 3}),
        ("chunk 1000 -> 128", 300, {"chunk": 1000}),
        ("one chunk", 100, {}),
        ("exactly two chunks", 256, {}),
        ("progress every 32", 5000, {}),
        ("seq wraps", 40000, {}),
        ("not a multiple of 4", 301, {}),
        ("ack of write 2 lost once", 300, {"drop": [2]}),
        ("ack of write 2 lost three times", 300, {"drop": [2, 3, 4]}),
        ("ack of the last write lost twice", 300, {"drop": [3, 4]}),
        ("stale acks before write 3's", 600, {"stale": [3]}),
        ("write 2 FLASH_ERROR", 300, {"nak_at": 2, "nak_status": 2}),
        ("write 2 unknown status", 300, {"nak_at": 2, "nak_status": 9}),
        ("erase OUT_OF_RANGE", 300, {"erase_status": 4}),
        ("erase unknown status", 300, {"erase_status": 200}),
        ("verify CRC_MISMATCH", 300, {"verify_status": 3, "verify_value": 0xDEADBEEF}),
        ("verify NO_VALID_APP", 300, {"verify_status": 6}),
        ("run NO_VALID_APP", 300, {"run_status": 6}),
        ("BL_INFO short", 300, {"info_len": 15}),
    ]
    out = []
    with tempfile.TemporaryDirectory() as d:
        mpath = os.path.join(d, "m.json")
        with open(mpath, "w") as f:
            f.write(with_())
        for name, size, cfg in configs:
            image = struct.pack("<II", 0x20020000, 0x080081C1) + bytes((i * 7) & 0xFF for i in range(size - 8))
            img = os.path.join(d, "img.bin")
            with open(img, "wb") as f:
                f.write(image)
            report = os.path.join(d, "r.json")
            if os.path.exists(report):
                os.unlink(report)
            clock = Clock()
            mf.time = clock
            port = BlPort(clock, mf, cfg)
            bl = mf.Bootloader(port)
            buf = io.StringIO()
            rc, error = None, None
            try:
                with contextlib.redirect_stdout(buf):
                    rc = mf.do_sync(bl, img, mpath, report, False)
            except SystemExit as e:
                error = str(e.code)
            except Exception:
                error = exc_line()
            v = {"name": name, "size": size, "cfg": cfg, "rc": rc, "error": error, "stdout": buf.getvalue(),
                 "report": open(report).read() if os.path.exists(report) else None, "writes": port.writes,
                 "elapsed": round(clock.t - T0, 6)}
            if len(port.writes) > 40:  # long ones as a digest
                v["writes"] = {"count": len(port.writes),
                               "sha256": hashlib.sha256("\n".join(port.writes).encode()).hexdigest()}
            out.append(v)
    mf.time = time
    return out


def exc_line():
    return traceback.format_exc().rstrip().splitlines()[-1]


MANIFEST = {
    "file": "mower_robot_firmware.bin", "size": 46440, "crc32": "0178f726",
    "sha256": "bc9fd038a64d545d62275ef30ae1cfe3068d6cd0915969bbc35ab814f11a4921",
    "version": "0.6.0", "semver": [0, 6, 0],
    "git_sha": "e6e6e6e6aa11bb22cc33dd44ee55ff6677889900", "git_sha32": 0xE6E6E6E6,
    "build_unix": 1793000000, "dirty": False,
}


def with_(**kv):
    m = dict(MANIFEST)
    for k, v in kv.items():
        if v is KeyError:
            m.pop(k)
        else:
            m[k] = v
    return json.dumps(m, indent=2) + "\n"


def manifests():
    yield "canonical", with_()
    for key in ("version", "git_sha", "size", "semver", "git_sha32", "build_unix", "dirty", "crc32"):
        yield f"no {key}", with_(**{key: KeyError})
    odd = [
        ("version", 7), ("version", [0, 6, 0]), ("version", None), ("version", 1.5), ("version", True),
        ("version", {"a": 1}), ("version", "0.6.0-é"), ("version", "it's"), ("version", 'say "hi"'),
        ("version", "both ' and \""), ("version", "tab\tnew\nline\\"), ("version", "\u0001\u007f\u0085 x"),
        ("version", "\U0001F600   ﻿"), ("git_sha", 12345), ("git_sha", None), ("git_sha", {"x": 1}),
        ("git_sha", ["a", "b", 3, 4, 5, 6, 7, 8, 9, 10]), ("git_sha", "short"), ("git_sha", 1.25),
        ("size", 46440.0), ("size", "46440"), ("size", None), ("size", -1), ("size", 10 ** 30),
        ("semver", "0.6.0"), ("semver", [0.0, 6, 0]), ("semver", [False, 6, 0]), ("semver", [0, 6]),
        ("semver", {"0": 1}), ("semver", 6), ("semver", None), ("semver", [0, 6.5, 0]), ("semver", [0, 6, 0, 0]),
        ("git_sha32", float(0xE6E6E6E6)), ("git_sha32", str(0xE6E6E6E6)), ("git_sha32", 0xE6E6E6E6 + 2 ** 64),
        ("build_unix", 1793000000.0), ("build_unix", 1793000000.5), ("build_unix", True),
        ("dirty", None), ("dirty", 0), ("dirty", 1), ("dirty", "yes"), ("dirty", ""), ("dirty", []), ("dirty", [0]),
        ("dirty", True), ("crc32", 12345), ("crc32", 1e300), ("crc32", -0.0), ("crc32", 1e-7), ("crc32", 123456789012345678.0),
    ]
    for key, value in odd:
        yield f"{key}={value!r}", with_(**{key: value})
    yield "NaN and Infinity", with_().replace('"dirty": false', '"dirty": NaN, "x": Infinity, "y": -Infinity')
    yield "duplicate keys", '{"version": "a", "git_sha": "b", "version": "c", "size": 1, "size": 2}'
    yield "exponent ints", with_().replace("46440", "4644e1")
    yield "negative zero", with_().replace('"size": 46440', '"size": -0')
    yield "big exponent", with_().replace("46440", "1E400")
    yield "list", "[1, 2]"
    yield "string", '"x"'
    yield "number", "3"
    yield "null", "null"
    yield "empty", ""
    yield "blank", "  \n "
    yield "not json", "not json\n"
    yield "trailing comma object", '{"a": 1,}'
    yield "trailing comma array", '[1, 2,]'
    yield "missing colon", '{"a" 1}'
    yield "missing comma", '{"a": 1 "b": 2}'
    yield "single quotes", "{'a': 1}"
    yield "unterminated string", '{"a": "abc'
    yield "control char", '{"a": "x\ty"}'
    yield "bad escape", '{"a": "\\q"}'
    yield "bad unicode escape", '{"a": "\\u12G4"}'
    yield "short unicode escape", '{"a": "\\u12"}'
    yield "surrogate pair", '{"version": "\\ud83d\\ude00", "git_sha": "x", "size": 1}'
    yield "extra data", '{"a": 1} {"b": 2}'
    yield "unterminated object", '{"a": 1'
    yield "unterminated array", '[1, 2'
    yield "leading zero", '{"a": 01}'
    yield "minus only", '{"a": -}'
    yield "bom", "﻿" + with_()
    yield "crlf error on line 3", '{\r\n  "a": 1,\r\n  "b" 2\r\n}'
    yield "cr only", '{\r"version": "v",\r"git_sha": "abcdefghij",\r"size": 3\r}'
    yield "bad utf-8", b'{"version": "\xff"}'
    yield "truncated utf-8", b'{"version": "\xe2\x82'
    yield "bad continuation", b'{"version": "\xe2(a"}'


def sync_vectors(mf):
    out = []
    image = struct.pack("<II", 0x20020000, 0x080081C1) + bytes(range(56))
    with tempfile.TemporaryDirectory() as d:
        img = os.path.join(d, "img.bin")
        with open(img, "wb") as f:
            f.write(image)
        for name, text in manifests():
            path = os.path.join(d, "m.json")
            with open(path, "wb") as f:
                f.write(text if isinstance(text, bytes) else text.encode())
            # every manifest against no board and against the app; the
            # canonical one also unversioned and --force
            combos = [("silent", False), ("app", False)]
            if name == "canonical":
                combos += [("app", True), ("unversioned", False)]
            for board, force in combos:
                if True:
                    report = os.path.join(d, "r.json")
                    if os.path.exists(report):
                        os.unlink(report)
                    clock = Clock()
                    mf.time = clock
                    port = Port(clock, board, mf)
                    bl = mf.Bootloader(port)
                    buf = io.StringIO()
                    rc, error = None, None
                    try:
                        with contextlib.redirect_stdout(buf):
                            rc = mf.do_sync(bl, img, path, report, force)
                    except SystemExit as e:
                        error = str(e.code)
                    except Exception:
                        error = exc_line()
                    out.append({
                        "manifest": name,
                        "text": text.decode("latin-1") if isinstance(text, bytes) else text,
                        "bytes": isinstance(text, bytes),
                        "board": board,
                        "force": force,
                        "rc": rc,
                        "error": error.replace(d, "DIR") if error else None,
                        "stdout": buf.getvalue(),
                        "report": open(report).read() if os.path.exists(report) else None,
                        "writes": port.writes,
                        "elapsed": round(clock.t - T0, 6),
                    })
    mf.time = time
    return out


def load_image_vectors(mf):
    cases = [
        ("ok.bin", struct.pack("<II", 0x20020000, 0x080081C1) + b"\x01\x02\x03"),
        ("ok4.bin", struct.pack("<II", 0x20000004, 0x0805FFFF) + b"\x01\x02\x03\x04"),
        ("sp.bin", struct.pack("<II", 0x20000000, 0x080081C1)),
        ("sp2.bin", struct.pack("<II", 0x20020004, 0x080081C1)),
        ("pc.bin", struct.pack("<II", 0x20020000, 0x080081C0)),
        ("pc2.bin", struct.pack("<II", 0x20020000, 0x08060001)),
        ("pc3.bin", struct.pack("<II", 0x20020000, 0x08007FFF)),
        ("empty.bin", b""),
        ("short.bin", b"\x00\x01\x02"),
        ("seven.bin", b"\x00" * 7),
        ("big.bin", struct.pack("<II", 0x20020000, 0x080081C1) + b"\x00" * (0x58000 - 7)),
        ("max.bin", struct.pack("<II", 0x20020000, 0x080081C1) + b"\x00" * (0x58000 - 8)),
        ("x.HEX", struct.pack("<II", 0x20020000, 0x080081C1)),
        ("x.elf", b""),
        ("x.Elf", b"\x00"),
        ("nonexistent.bin", None),
        ("dir", "dir"),
    ]
    out = []
    with tempfile.TemporaryDirectory() as d:
        for name, data in cases:
            path = os.path.join(d, name)
            if data == "dir":
                os.mkdir(path)
            elif data is not None:
                with open(path, "wb") as f:
                    f.write(data)
            res = {"name": name, "kind": {None: "missing", "dir": "dir"}.get(data, "file")}
            if isinstance(data, bytes):
                if len(data) <= 64:
                    res["data"] = data.hex()
                else:  # a vector table and zeros
                    assert data[8:] == bytes(len(data) - 8)
                    res["head"], res["zeros"] = data[:8].hex(), len(data) - 8
            try:
                img = mf.load_image(path)
                res["ok"] = {"len": len(img), "crc32": zlib.crc32(img) & 0xFFFFFFFF}
            except SystemExit as e:
                res["exit"] = str(e.code)
            except Exception:
                res["error"] = exc_line().replace(d, "DIR")
            out.append(res)
    return out


def float_cases():
    rng = random.Random(2026)
    xs = [0.0, -0.0, 1.0, -1.0, 0.1, 0.5, 1.5, 2.5, 100.0, 1e15, 1e16, 1e17, 9999999999999998.0, 1e-4, 1e-5,
          0.00012345, 123456789.123, 5e-324, 2.2250738585072014e-308, 1.7976931348623157e308, 1 / 3, 2 / 3,
          math.pi, math.e, 46440.0, 3873892070.0, 1793000000.5, 1e22, 1e23, 1.2345e-300, 4.35, 0.3, 1e300]
    for _ in range(400):
        bits = rng.getrandbits(64)
        x = struct.unpack("<d", struct.pack("<Q", bits))[0]
        if math.isfinite(x):
            xs.append(x)
    for _ in range(200):
        xs.append(rng.uniform(-1e6, 1e6))
        xs.append(round(rng.uniform(0, 1000), rng.randrange(0, 6)))
    return [{"bits": struct.pack("<d", x).hex(), "repr": repr(x)} for x in xs]


def main():
    mf = load_mower_flash()
    strings = ["", "a", "it's", 'say "hi"', "both ' and \"", "back\\slash", "\t\n\r", "\x00\x01\x1f\x7f",
               "\x80\x9f\xa0\xa1\xad\xff", "é ü ñ 中文", "  ​　", "﻿￹", "\U0001F600",
               "", "\U000F0000", "mixed 'a' \"b\" \\ c"]
    valid = ['{"a": 1, "b": [1, 2.5, -0.0, 1e400, -1E-400], "c": {}, "d": [], "e": null, "f": true}',
             '[NaN, Infinity, -Infinity, 0, -0, 12345678901234567890123, 1.0, 10e2, 1E+2, 1e-2]',
             '"\\u00e9\\ud83d\\ude00\\n\\t\\"\\\\\\/\\b\\f"', '{"k": "v", "k": "w", "z": 1}',
             '  {"nested": {"a": [{"b": []}, {}]}}  ', '[1,2 , 3 ]', '{"a":1}',
             '"<\\u007f\\u0080\\u00ff\\u0100\\uffff>"', '[0.1, 0.2, 0.30000000000000004, 1e16, 123e-7]']
    broken = ['', ' ', '{', '[', '{"a"', '{"a":', '{"a":1', '[1', '[1,', '{"a" 1}', '{"a":1 "b":2}', '{1:2}',
              "{'a':1}", '{"a":1,}', '[1,]', '"abc', '"\\', '"\\u', '"\\u12', '"\\u123', '"\\u1234', '"\\uzzzz"',
              '"\\x"', '"a\nb"', '01', '-', '-a', '1.', '1e', '1e+', '.5', 'tru', 'nul', 'NaNx', '[1] x', 'abc',
              '{"a":1}}', '\n\n  {"a"::1}', '[\n1,\n2\n,]', '﻿{}', '"\\ud83d\\u12"', '"\\ud83d\\uzz12"']
    loads_ok = []
    for t in valid:
        v = json.loads(t)
        loads_ok.append({"text": t, "dumps": json.dumps(v), "dump2": json.dumps(v, indent=2), "str": str(v),
                         "repr": repr(v)})
    loads_err = []
    for t in broken:
        try:
            json.loads(t)
            raise AssertionError(f"{t!r} parsed")
        except json.JSONDecodeError:
            loads_err.append({"text": t, "error": exc_line()})
    splits = ["a/b.bin", "b.tar.gz", ".bashrc", "a/.b", "a.b/c", "..x", "..x.bin", "x.", "", ".", "..", "a/",
              "/opt/mower/firmware/mower_robot_firmware.bin", "dir.d/file", "f.json", "a/b/.c.d", "..."]
    json.dump({
        "python": sys.version.split()[0],
        "float_repr": float_cases(),
        "str_repr": [{"s": s, "repr": repr(s)} for s in strings],
        "loads_ok": loads_ok,
        "loads_err": loads_err,
        "splitext": [{"p": p, "root": os.path.splitext(p)[0]} for p in splits],
        "load_image": load_image_vectors(mf),
        "sync": sync_vectors(mf),
        "flash": flash_vectors(mf),
    }, sys.stdout, indent=0, ensure_ascii=True)
    sys.stdout.write("\n")


if __name__ == "__main__":
    os.environ["TZ"] = "UTC"
    time.tzset()
    main()
