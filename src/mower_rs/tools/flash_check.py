#!/usr/bin/env python3
"""The STM32 firmware flasher on a fake board, scenario by scenario (E6).

Runs `firmware/tools/mower_flash.py` -- and any other implementation of its
command line, i.e. the Rust `mower_flash` -- against `fake_base.py
--bootloader-lib`, whose bootloader is the unmodified
`firmware/bootloader/Src/bl_protocol.c` behind a simulated flash
(`bl_sim/sim_flash.c`), and records per scenario and implementation:

  (a) every frame the host sent, every bootloader / app reply to it (0x90,
      0x91, 0x8F) and every flash operation (erase, write, CRC, jump, reset);
  (b) the flash afterwards (all 512 KB, bootloader and settings sectors
      included);
  (c) the report (`--report`, firmware_sync.json), `time` aside;
  (d) the exit code;
  (e) stdout (the `written in X s` number aside) and stderr's last line
      (a Python traceback's exception line);
  (f) how long it took.

    flash_check.py check  --impl python=CMD [--impl rust=CMD] [--quick|--all|--scenarios S01,S02]
                          [--golden DIR] [--work DIR] [--keep]
    flash_check.py run    --impl NAME=CMD --out DIR [...]      one implementation, results to DIR
    flash_check.py compare DIR_A DIR_B                          two `run` outputs, (a)-(f)
    flash_check.py golden --impl python=CMD --golden DIR [...]  (re)write the G1 goldens

`check` also holds every implementation to what the scenario table says
mower_flash.py does (docs: E6_DESIGN.md 1.5 / 4.2), compares each one with
the first, and with `--golden` against the stored transcripts and reports
(deploy/test/golden/firmware_sync/). Two runs of the same implementation
give the same transcript: nothing below depends on timing except (f).

CMD is split like a shell command line and gets `-p PORT ...` appended, e.g.
`python="python3 firmware/tools/mower_flash.py"`, `rust=target/debug/mower_flash`.
The bootloader library is compiled here with `cc` unless `--bl-lib` names a
prebuilt one (the runtime image has no compiler). Linux only (ptys, the
TIOCEXCL check); needs pyserial for mower_flash.py itself.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import random
import re
import shlex
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import time
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
FAKE = os.path.join(HERE, "fake_base.py")

FLASH_SIZE = 512 * 1024
APP = 0x8000  # app region offset in the flash file
APP_MAX = 0x58000
S7 = 0x60000  # sector 7, the PID settings
SECTORS = [(0x0, 16384), (0x4000, 16384), (0x8000, 16384), (0xC000, 16384), (0x10000, 65536),
           (0x20000, 131072), (0x40000, 131072), (0x60000, 131072)]
CHUNK = 128
REPLY_TYPES = {0x90, 0x91, 0x8F}
FLASH_EVENTS = {"erase", "erase_fail", "write", "write_fail", "crc", "flip", "jump", "reset",
                "overrun", "violation", "power_off", "host_killed", "ignored_enter", "state"}
BAD_BOOT_MSG = ("bootloader did not answer. Is the port right, is the bootloader flashed at "
                "0x08000000, and is nothing else using the port?")


# -- fixtures ------------------------------------------------------------------

BUILDS = {
    # name: (seed, size, version, semver, git sha, build_unix, dirty, fake)
    "bundled": (1, None, "0.6.0-e6", "0.6.0", "e6e6e6e6aa11bb22cc33dd44ee55ff6677889900", 1793000000, 0, {}),
    "other": (2, 6000, "0.5.9", "0.5.9", "0123abcd9876543210fedcba0011223344556677", 1790000000, 0, {}),
    "big": (3, 40000, "0.5.8", "0.5.8", "feedface00000000000000000000000000000001", 1789000000, 0, {}),
    "cubeide": (4, 6000, "0.0.0", "0.0.0", "0000000000000000", 0, 0, {"unversioned": True}),
    "old": (5, 6000, "0.4.0", "0.4.0", "0dd0dd0d00000000000000000000000000000002", 1780000000, 0, {"no_0x87": True}),
    "noack": (6, 6000, "0.5.7", "0.5.7", "5ca1ab1e00000000000000000000000000000003", 1788000000, 0, {"no_enter_ack": True}),
    "deafapp": (7, 6000, "0.5.6", "0.5.6", "dea0f0a000000000000000000000000000000004", 1787000000, 0, {"ignore_enter": True}),
    "dirtyrun": (8, 6000, "0.6.0-e6-dirty", "0.6.0", "e6e6e6e6aa11bb22cc33dd44ee55ff6677889900", 1793000000, 1, {}),
}


def make_image(seed: int, size: int, sp: int = 0x20020000, pc: int = 0x080081C1) -> bytes:
    """A build: a valid vector table (unless told otherwise), then noise."""
    body = random.Random(seed).randbytes(max(0, size - 8))
    return (struct.pack("<II", sp, pc) + body)[:size]


def write_manifest(image_path: str, out: str, version: str, semver: str, sha: str, built: int,
                   dirty: int) -> None:
    """With the production tool, so the manifest is what the image ships."""
    subprocess.run([sys.executable, os.path.join(REPO, "firmware", "tools", "fw_manifest.py"),
                    image_path, "--version", version, "--semver", semver, "--git-sha", sha,
                    "--build-unix", str(built), "--dirty", str(dirty), "-o", out],
                   check=True, stdout=subprocess.DEVNULL)


def bootloader_region(path: str) -> bytes:
    if path:
        with open(path, "rb") as f:
            data = f.read()
        return data[:0x8000] + b"\xFF" * (0x8000 - min(len(data), 0x8000))
    out = bytearray()
    h = b"mower bootloader stand-in"
    while len(out) < 0x8000:
        h = hashlib.sha256(h).digest()
        out += h
    return bytes(out[:0x8000])


def settings_region() -> bytes:
    rec = b"PIDSETTINGS-E6" + bytes(range(50))
    return (rec * (131072 // len(rec) + 1))[:131072]


def padded(image: bytes) -> bytes:
    return image + b"\xFF" * (-len(image) % 4)


def erase_sectors(size: int):
    end = 0x8000 + size
    last = 2
    for s in range(2, 7):
        if SECTORS[s][0] < end:
            last = s
    return range(2, last + 1)


def flashed_model(initial: bytes, image: bytes) -> bytes:
    """The flash after a complete BL_ERASE + BL_WRITEs of `image`."""
    img = padded(image)
    out = bytearray(initial)
    for s in erase_sectors(len(img)):
        start, size = SECTORS[s]
        out[start:start + size] = b"\xFF" * size
    out[APP:APP + len(img)] = img
    return bytes(out)


# -- scenarios -----------------------------------------------------------------

class Step:
    def __init__(self, args, restart=False, faults=None):
        self.args = args  # after `-p PORT`; {image} {report} {manifest} filled in
        self.restart = restart  # the fake was powered off: start it again from its flash file
        self.faults = faults  # faults of that restart


class Scenario:
    def __init__(self, name, title, initial="other", catalog=("bundled", "other"), faults=None,
                 steps=None, size=2048, quick=True, image="bundled", manifest="ok", expect=None, real=False,
                 informational=False):
        self.name = name
        self.title = title
        self.initial = initial  # build in the app region; None = erased; "partial:B" = B without its vector table
        self.catalog = catalog  # builds the fake recognises; "bundled_wrong" = bundled bytes, other identity
        self.faults = faults or {}
        self.steps = steps or [Step(["sync", "{image}", "--report", "{report}"])]
        self.size = size  # of the bundled image
        self.quick = quick
        self.image = image  # what IMAGE is: bundled | bad_vector | empty | too_big | hex | short | missing
        self.manifest = manifest  # ok | missing | invalid | no_version | no_semver | list
        self.expect = expect or {}
        self.real = real  # the bundled build is --bundled-bin (the image's own firmware)
        self.informational = informational  # recorded, neither checked nor compared


def sig_flash(writes, enter=True):
    """The host frames of a whole sync: 0x06, PING, (0x0F, PING), ERASE, WRITEs,
    VERIFY, RUN_APP, 0x06. From the bootloader: the PING of sync's own check
    and the one of ensure_bootloader."""
    head = "06 10 0f 10" if enter else "06 10x2"
    return f"{head} 11 12x{writes} 13 14 06"


SYNC = ["sync", "{image}", "--report", "{report}"]
W = 2048 // CHUNK  # writes of the default 2 KB image

SCENARIOS = [
    Scenario("S01", "the app is the bundled build", initial="bundled",
             expect=dict(rc=0, action="up_to_date", sig="06", flash="unchanged",
                         stdout=["firmware is up to date"])),
    Scenario("S02", "the app is another build", expect=dict(
        rc=0, action="flashed", sig=sig_flash(W), flash="bundled",
        stdout=["application is running, asking it to reboot into the bootloader", "  app acknowledged",
                "flashing: different build", "  crc32 0x{crc:08X} ok", "starting application"])),
    Scenario("S02b", "the app is a bigger other build (sectors past the new image keep it)", initial="big",
             catalog=("bundled", "big"), expect=dict(rc=0, action="flashed", sig=sig_flash(W), flash="bundled")),
    Scenario("S03", "the app is an UNVERSIONED (CubeIDE) build", initial="cubeide",
             catalog=("bundled", "cubeide"), expect=dict(
                 rc=0, action="flashed", sig=sig_flash(W), flash="bundled",
                 stdout=["running: v0.0.0 sha=00000000 built=unknown unversioned"])),
    Scenario("S03b", "the app is the bundled commit, but dirty", initial="dirtyrun",
             catalog=("bundled", "dirtyrun"), expect=dict(rc=0, action="flashed", sig=sig_flash(W), flash="bundled")),
    Scenario("S04", "old firmware: status frames, no 0x87", initial="old", catalog=("bundled", "old"),
             expect=dict(rc=0, action="flashed", sig=sig_flash(W), flash="bundled",
                         stdout=["running: unknown (running firmware predates 0x87 (no build identity))"])),
    Scenario("S05", "the board is in the bootloader (app's vector table blank)", initial="partial:bundled",
             expect=dict(rc=0, action="flashed", sig=sig_flash(W, enter=False), flash="bundled",
                         stdout=["running: none (board is in the bootloader (no valid app?))"])),
    Scenario("S06", "no STM32 on the port", faults={"dead": True},
             expect=dict(rc=2, action="no_device", error="no STM32 detected", sig="06 10", flash="unchanged",
                         stdout=["no application status and no bootloader answer on this port"])),
    Scenario("S07", "--force with the bundled build running", initial="bundled",
             steps=[Step(SYNC + ["--force"])],
             expect=dict(rc=0, action="flashed", sig=sig_flash(W), flash="bundled", stdout=["flashing: forced"])),
    Scenario("S08", "the app resets without its 0x8F", initial="noack", catalog=("bundled", "noack"),
             expect=dict(rc=0, action="flashed", sig=sig_flash(W), flash="bundled",
                         stdout=["  no ack from app (continuing anyway)"])),
    Scenario("S09", "the app ignores 0x0F", initial="deafapp", catalog=("bundled", "deafapp"),
             expect=dict(rc=1, action="failed", error=BAD_BOOT_MSG, sig="06 10 0f 10x25", flash="unchanged")),
    Scenario("S10", "BL_ERASE fails", faults={"erase_error": True},
             expect=dict(rc=1, action="failed", error="erase failed: FLASH_ERROR (value=0x00000000)",
                         sig="06 10 0f 10 11", flash="unchanged")),
    Scenario("S11", "the ack of the write @0x400 is lost once", faults={"drop_reply": [{"cmd": 0x12, "offset": 0x400}]},
             expect=dict(rc=0, action="flashed", sig=sig_flash(W + 1), flash="bundled", retries=1,
                         stdout=["  retry write @0x400"])),
    Scenario("S11b", "... twice", faults={"drop_reply": [{"cmd": 0x12, "offset": 0x400, "times": 2}]},
             expect=dict(rc=0, action="flashed", sig=sig_flash(W + 2), flash="bundled", retries=2)),
    Scenario("S11c", "the ack of the write @0x400 comes 2.3 s late: a stale ack after the retry",
             faults={"delay_reply": [{"cmd": 0x12, "offset": 0x400, "s": 2.3}]},
             expect=dict(rc=0, action="flashed", sig=sig_flash(W + 1), flash="bundled", retries=1)),
    Scenario("S12", "... three times", faults={"drop_reply": [{"cmd": 0x12, "offset": 0x400, "times": 3}]},
             expect=dict(rc=1, action="failed", error="no ack for command 0x12", sig="06 10 0f 10 11 12x10",
                         flash="vector_blank", retries=2)),
    Scenario("S13", "the write @0x80 fails (FLASH_ERROR)", faults={"write_error": 0x80},
             expect=dict(rc=1, action="failed", error="write @0x80 failed: FLASH_ERROR (value=0x00000080)",
                         sig="06 10 0f 10 11 12", flash="vector_blank", retries=0)),
    Scenario("S14", "a bit flips after the write @0x100", faults={"flip": [0x100, 3]},
             expect=dict(rc=1, action="failed", error_prefix="verify failed: CRC_MISMATCH (value=0x",
                         sig=f"06 10 0f 10 11 12x{W} 13", flash="flipped")),
    Scenario("S15", "the new app never reports (not a build the board knows)", catalog=("other",),
             expect=dict(rc=1, action="failed", error="application did not report 0x87 after flashing",
                         sig=sig_flash(W), flash="bundled")),
    Scenario("S16", "the new app reports another identity", catalog=("other", "bundled_wrong"),
             expect=dict(rc=1, action="failed", error="running build still differs from the bundled one",
                         sig=sig_flash(W), flash="bundled")),
    Scenario("S17", "the ack of the write @0x200 has a bad CRC", faults={"corrupt_reply": [{"cmd": 0x12, "offset": 0x200}]},
             expect=dict(rc=0, action="flashed", sig=sig_flash(W + 1), flash="bundled", retries=1)),
    Scenario("S17b", "bootloader replies byte by byte", faults={"bytewise": 0.0005},
             expect=dict(rc=0, action="flashed", sig=sig_flash(W), flash="bundled", retries=0)),
    Scenario("S17c", "junk before every bootloader reply", faults={"noise": 7},
             expect=dict(rc=0, action="flashed", sig=sig_flash(W), flash="bundled", retries=0)),
    Scenario("S18", "the flasher is killed at the 6th write; the next sync recovers",
             faults={"kill_host_at_write": 6}, steps=[Step(SYNC), Step(SYNC)],
             expect=dict(rc=[-9, 0], action="flashed", sig=["06 10 0f 10 11 12x6", sig_flash(W, enter=False)],
                         flash="bundled", stdout=["running: none (board is in the bootloader (no valid app?))"])),
    Scenario("S18b", "power lost at the 6th write; after power-up the next sync recovers",
             faults={"power_off_at_write": 6}, steps=[Step(SYNC), Step(SYNC, restart=True, faults={})],
             expect=dict(rc=[1, 0], action="flashed", sig=["06 10 0f 10 11 12x6", sig_flash(W, enter=False)],
                         flash="bundled", stderr_prefix=[
                             # the pty's master is gone: readable, and read() returns 0
                             "serial.serialutil.SerialException: device reports readiness to read but returned "
                             "no data (device disconnected or multiple access on port?)", None])),
    Scenario("S19", "IMAGE has the wrong vector table", image="bad_vector",
             expect=dict(rc=1, action=None, sig="06", flash="unchanged",
                         stderr="this does not look like an app linked at 0x08008000 (SP=0x30000000 PC=0x080081C1); "
                                "rebuild with the updated linker script")),
    Scenario("S19b", "IMAGE is empty", image="empty",
             expect=dict(rc=1, action=None, sig="06", flash="unchanged", stderr="image is empty")),
    Scenario("S19c", "IMAGE is larger than the app region", image="too_big",
             expect=dict(rc=1, action=None, sig="06", flash="unchanged",
                         stderr="image is 360452 bytes, larger than the 360448-byte app region")),
    Scenario("S19d", "IMAGE is a .hex", image="hex",
             expect=dict(rc=1, action=None, sig="06", flash="unchanged",
                         stderr="give the .bin (Debug/mower_robot_firmware.bin), not .hex/.elf")),
    Scenario("S19e", "IMAGE is shorter than its vector table", image="short",
             expect=dict(rc=1, action=None, sig="06", flash="unchanged",
                         stderr="struct.error: unpack requires a buffer of 8 bytes")),
    Scenario("S19f", "IMAGE does not exist (its manifest does)", image="missing",
             expect=dict(rc=1, action=None, sig="06", flash="unchanged",
                         stderr_prefix="FileNotFoundError: [Errno 2] No such file or directory: ")),
    Scenario("S20", "no manifest next to IMAGE", manifest="missing",
             expect=dict(rc=1, action=None, sig="", flash="unchanged",
                         stderr_prefix="FileNotFoundError: [Errno 2] No such file or directory: ")),
    Scenario("S20b", "the manifest is not JSON", manifest="invalid",
             expect=dict(rc=1, action=None, sig="", flash="unchanged",
                         stderr="json.decoder.JSONDecodeError: Expecting value: line 1 column 1 (char 0)")),
    Scenario("S20c", "the manifest has no version", manifest="no_version",
             expect=dict(rc=1, action=None, sig="", flash="unchanged", stderr="KeyError: 'version'")),
    Scenario("S20d", "the manifest has no semver, the app reports 0x87", manifest="no_semver",
             expect=dict(rc=1, action=None, sig="06", flash="unchanged", stderr="KeyError: 'semver'")),
    Scenario("S20e", "the manifest has no semver, the board is in the bootloader", manifest="no_semver",
             initial="partial:bundled",
             expect=dict(rc=1, action=None, sig=sig_flash(W, enter=False), flash="bundled", stderr="KeyError: 'semver'")),
    Scenario("S20f", "the manifest is a JSON list", manifest="list",
             expect=dict(rc=1, action=None, sig="", flash="unchanged",
                         stderr="AttributeError: 'list' object has no attribute 'get'")),
    Scenario("S22", "a BL_ACK one byte short (the write @0x100)", faults={"bad_len_reply": [{"cmd": 0x12, "offset": 0x100}]},
             expect=dict(rc=1, action=None, sig="06 10 0f 10 11 12x2", flash="vector_blank",
                         stderr="struct.error: unpack requires a buffer of 6 bytes")),
    Scenario("S22b", "a BL_INFO one byte short", faults={"bad_len_reply": [{"cmd": 0x10}]},
             expect=dict(rc=1, action=None, sig="06 10 0f 10", flash="unchanged",
                         stderr="struct.error: unpack requires a buffer of 16 bytes")),
    # the other subcommands
    Scenario("C01", "app-info, app running", initial="bundled", steps=[Step(["app-info"])],
             expect=dict(rc=0, sig="06", flash="unchanged")),
    Scenario("C02", "app-info, old firmware", initial="old", catalog=("old",), steps=[Step(["app-info"])],
             expect=dict(rc=1, sig="06", flash="unchanged",
                         stderr="application is running but does not answer 0x06 (firmware predates 0x87)")),
    Scenario("C03", "app-info, board in the bootloader", initial=None, steps=[Step(["app-info"])],
             expect=dict(rc=1, sig="06 10", flash="unchanged", stderr="board is in the bootloader, no application running")),
    Scenario("C04", "app-info, no STM32", faults={"dead": True}, steps=[Step(["app-info"])],
             expect=dict(rc=1, sig="06 10", flash="unchanged", stderr="no STM32 detected on this port")),
    Scenario("C05", "info from the app", steps=[Step(["info"])],
             expect=dict(rc=0, sig="10 0f 10", flash="unchanged", stdout=[
                 "bootloader v1: app @ 0x08008000, max 352 KB, chunk 128 B, app_valid=True by_request=True erased=False"])),
    Scenario("C06", "enter, then run", steps=[Step(["enter"]), Step(["run"])],
             expect=dict(rc=[0, 0], sig=["10 0f 10", "14"], flash="unchanged", stdout=["application started"])),
    Scenario("C07", "run with no valid app", initial=None, steps=[Step(["run"])],
             expect=dict(rc=1, sig="14", flash="unchanged", stderr="error: run failed: NO_VALID_APP (value=0x00000000)")),
    Scenario("C08", "info with an app that ignores 0x0F", initial="deafapp", catalog=("deafapp",),
             steps=[Step(["info"])], expect=dict(rc=1, sig="10 0f 10x25", flash="unchanged", stderr="error: " + BAD_BOOT_MSG)),
    Scenario("C09", "flash --no-run", steps=[Step(["flash", "{image}", "--no-run"])],
             expect=dict(rc=0, sig=f"10 0f 10 11 12x{W} 13", flash="bundled",
                         stdout=["staying in bootloader (use `run` to start the app)"])),
    Scenario("C10", "flash", steps=[Step(["flash", "{image}"])],
             expect=dict(rc=0, sig=f"10 0f 10 11 12x{W} 13 14", flash="bundled")),
    Scenario("C11", "flash, BL_ERASE fails", faults={"erase_error": True}, steps=[Step(["flash", "{image}"])],
             expect=dict(rc=1, sig="10 0f 10 11", flash="unchanged", stderr="error: erase failed: FLASH_ERROR (value=0x00000000)")),
    Scenario("C12", "sync without --report", steps=[Step(["sync", "{image}"])],
             expect=dict(rc=0, action=None, sig=sig_flash(W), flash="bundled")),
    Scenario("C13", "sync --manifest elsewhere", steps=[Step(["sync", "{image}", "--manifest", "{manifest}", "--report", "{report}"])],
             manifest="elsewhere", expect=dict(rc=0, action="flashed", sig=sig_flash(W), flash="bundled")),
    Scenario("C14", "sync --report '' (no report: `if report_path:`)", steps=[Step(["sync", "{image}", "--report", ""])],
             expect=dict(rc=0, action=None, sig=sig_flash(W), flash="bundled")),
    Scenario("C15", "sync --manifest ''", steps=[Step(["sync", "{image}", "--manifest", "", "--report", "{report}"])],
             expect=dict(rc=1, action=None, sig="", flash="unchanged",
                         stderr="FileNotFoundError: [Errno 2] No such file or directory: ''")),
]
# The deliberate difference, made visible: a flasher killed while it holds the
# tty exclusively (TIOCEXCL; the Rust one does, pyserial does not) leaves the
# flag set while something else keeps the tty open -- here the fake's own
# slave fd -- so the next open by a non-root user fails with EBUSY. On the
# robot's /dev/ttyS3 the last close clears the flag. Not checked: run it as
# uid 1000 to see it (e2-verify's flash check does).
SCENARIOS.append(Scenario("S18x", "TIOCEXCL left by a killed flasher on a tty still held open (informational)",
                          faults={"kill_host_at_write": 6, "keep_excl": True}, steps=[Step(SYNC), Step(SYNC)],
                          quick=False, informational=True))

# R: the bundled build is the image's real firmware (--bundled-bin; 46 KB on main, so ~20 s a flash)
REAL = [
    Scenario("R01", "the app is the image's own build", initial="bundled", real=True, quick=False,
             expect=dict(rc=0, action="up_to_date", sig="06", flash="unchanged")),
    Scenario("R02", "another build -> the image's", real=True, quick=False,
             expect=dict(rc=0, action="flashed", flash="bundled")),
    Scenario("R05", "the board is in the bootloader -> the image's", initial="partial:bundled", real=True, quick=False,
             expect=dict(rc=0, action="flashed", flash="bundled")),
    Scenario("R18", "power lost at the 100th write of the image's build, then a sync", real=True, quick=False,
             faults={"power_off_at_write": 100}, steps=[Step(SYNC), Step(SYNC, restart=True, faults={})],
             expect=dict(rc=[1, 0], action="flashed", flash="bundled")),
]
SCENARIOS += REAL

# S21: image sizes; > 32 KB also wraps the frame seq
for _name, _size, _quick in [("S21a", 4097, True), ("S21b", 16380, False), ("S21c", 16384, True),
                             ("S21d", 16388, True), ("S21e", 49152, False), ("S21f", 65540, False),
                             ("S21g", 114688, False), ("S21h", 245760, False), ("S21i", 360448, False)]:
    _w = -(-((_size + 3) // 4 * 4) // CHUNK)
    SCENARIOS.append(Scenario(_name, f"a {_size}-byte image", size=_size, quick=_quick,
                              expect=dict(rc=0, action="flashed", sig=sig_flash(_w), flash="bundled",
                                          erased=len(erase_sectors((_size + 3) // 4 * 4)))))


# -- running one ---------------------------------------------------------------

def build_lib(out_dir: str) -> str:
    out = os.path.join(out_dir, "bl_sim.so")
    fw = os.path.join(REPO, "firmware")
    subprocess.run([os.environ.get("CC", "cc"), "-shared", "-fPIC", "-O1", "-std=c11", "-Wall", "-Wextra",
                    "-I", os.path.join(fw, "bootloader", "Inc"), "-I", os.path.join(fw, "Module", "Inc"),
                    os.path.join(fw, "bootloader", "Src", "bl_protocol.c"),
                    os.path.join(HERE, "bl_sim", "sim_flash.c"), "-o", out], check=True)
    return out


def builds(cache: str, size: int) -> str:
    """Every build of BUILDS as .bin + fw_manifest.py manifest, the bundled one `size` long."""
    d = os.path.join(cache, f"builds-{size}")
    if os.path.exists(os.path.join(d, "done")):
        return d
    os.makedirs(d, exist_ok=True)
    for name, (seed, bsize, version, semver, sha, built, dirty, fake) in BUILDS.items():
        path = os.path.join(d, f"{name}.bin")
        with open(path, "wb") as f:
            f.write(make_image(seed, size if bsize is None else bsize))
        write_manifest(path, os.path.join(d, f"{name}.json"), version, semver, sha, built, dirty)
    open(os.path.join(d, "done"), "w").close()
    return d


def prepare(sc: Scenario, d: str, bootloader_bin: str, cache: str, bundled_bin: str = ""):
    fw = os.path.join(d, "fw")
    cat = os.path.join(d, "catalog")
    shutil.copytree(builds(cache, sc.size), fw)
    os.unlink(os.path.join(fw, "done"))
    if sc.real:
        shutil.copy(bundled_bin, os.path.join(fw, "bundled.bin"))
        shutil.copy(os.path.splitext(bundled_bin)[0] + ".json", os.path.join(fw, "bundled.json"))
    os.makedirs(cat)
    os.makedirs(os.path.join(d, "state"))
    images = {}
    for name in BUILDS:
        with open(os.path.join(fw, f"{name}.bin"), "rb") as f:
            images[name] = f.read()
    for name in sc.catalog:
        src = "bundled" if name == "bundled_wrong" else name
        with open(os.path.join(fw, f"{src}.json")) as f:
            m = json.load(f)
        m["fake"] = BUILDS[src][7]
        if name == "bundled_wrong":
            m["git_sha32"] ^= 0x1
            m["git_sha"] = "%08x" % m["git_sha32"] + m["git_sha"][8:]
        with open(os.path.join(cat, f"{name}.json"), "w") as f:
            json.dump(m, f, indent=2)
    # IMAGE and its manifest as the scenario wants them
    image = os.path.join(fw, "mower_robot_firmware.bin")
    manifest = os.path.splitext(image)[0] + ".json"
    variants = {
        "bundled": lambda: images["bundled"],
        "bad_vector": lambda: make_image(1, sc.size, sp=0x30000000),
        "empty": lambda: b"",
        "too_big": lambda: make_image(1, APP_MAX + 4),
        "hex": lambda: images["bundled"],
        "short": lambda: images["bundled"][:6],
    }
    if sc.image == "hex":
        image = os.path.join(fw, "mower_robot_firmware.hex")
    if sc.image != "missing":
        with open(image, "wb") as f:
            f.write(variants[sc.image]())
    shutil.copy(os.path.join(fw, "bundled.json"), manifest)
    other_manifest = os.path.join(d, "elsewhere.json")
    with open(manifest) as f:
        m = json.load(f)
    if sc.manifest == "missing":
        os.unlink(manifest)
    elif sc.manifest == "invalid":
        with open(manifest, "w") as f:
            f.write("not json\n")
    elif sc.manifest in ("no_version", "no_semver"):
        m.pop(sc.manifest[3:])
        with open(manifest, "w") as f:
            json.dump(m, f, indent=2)
    elif sc.manifest == "list":
        with open(manifest, "w") as f:
            json.dump([m], f)
    elif sc.manifest == "elsewhere":
        os.replace(manifest, other_manifest)
    # the flash: bootloader, the initial app, the settings sector
    flash = bytearray(b"\xFF" * FLASH_SIZE)
    flash[0:0x8000] = bootloader_region(bootloader_bin)
    flash[S7:S7 + 131072] = settings_region()
    if sc.initial:
        partial = sc.initial.startswith("partial:")
        img = padded(images[sc.initial.split(":")[-1]])
        flash[APP:APP + len(img)] = img
        if partial:
            flash[APP:APP + CHUNK] = b"\xFF" * CHUNK
    with open(os.path.join(d, "flash.bin"), "wb") as f:
        f.write(flash)
    with open(os.path.join(d, "flash.initial"), "wb") as f:
        f.write(flash)
    return {"image": image, "manifest": other_manifest if sc.manifest == "elsewhere" else manifest,
            "report": os.path.join(d, "state", "firmware_sync.json"), "bundled": images["bundled"]}


def start_fake(d: str, lib: str, faults: dict, n: int):
    log = os.path.join(d, f"fake.{n}.jsonl")
    link = os.path.join(d, "stmcom")
    proc = subprocess.Popen([sys.executable, FAKE, "--pty-link", link, "--log", log, "--seconds", "3600",
                             "--bootloader-lib", lib, "--flash-file", os.path.join(d, "flash.bin"),
                             "--catalog", os.path.join(d, "catalog"), "--faults", json.dumps(faults),
                             "--host-pid-file", os.path.join(d, "host.pid")],
                            stdout=open(os.path.join(d, f"fake.{n}.out"), "w"), stderr=subprocess.STDOUT)
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            raise RuntimeError(f"fake exited with {proc.returncode}: {open(os.path.join(d, f'fake.{n}.out')).read()}")
        if os.path.exists(log):
            with open(log) as f:
                if '"ev":"ready"' in f.read():
                    return proc, log
        time.sleep(0.02)
    proc.kill()
    raise RuntimeError("fake did not get ready")


def stop_fake(proc) -> int:
    if proc.poll() is None:
        proc.send_signal(signal.SIGTERM)
    try:
        return proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        proc.kill()
        return proc.wait()


def read_log(path: str):
    entries = []
    t0 = None
    with open(path) as f:
        for line in f:
            try:
                e = json.loads(line)
            except ValueError:
                continue  # a power-off can cut the last line
            if e["dir"] == "meta" and "t0_monotonic" in e.get("fields", {}):
                t0 = e["fields"]["t0_monotonic"]
                continue
            e["abs"] = t0 + e["t"]
            entries.append(e)
    return entries


def report_state(path: str):
    if not os.path.exists(path):
        return None
    with open(path, "rb") as f:
        return os.stat(path).st_mtime_ns, f.read()


def run_scenario(sc: Scenario, impl_cmd, d: str, lib: str, bootloader_bin: str, timeout: float, cache: str,
                 bundled_bin: str = ""):
    if os.path.exists(d):
        shutil.rmtree(d)
    os.makedirs(d)
    paths = prepare(sc, d, bootloader_bin, cache, bundled_bin)
    fakes = []
    proc, log = start_fake(d, lib, sc.faults, 1)
    fakes.append(log)
    env = dict(os.environ, TZ="UTC", PYTHONDONTWRITEBYTECODE="1")
    steps = []
    for i, step in enumerate(sc.steps, 1):
        if step.restart:
            stop_fake(proc)
            proc, log = start_fake(d, lib, step.faults or {}, len(fakes) + 1)
            fakes.append(log)
        args = [a.format(**paths) for a in step.args]
        cmd = list(impl_cmd) + ["-p", os.path.join(d, "stmcom")] + args
        before = report_state(paths["report"])
        t_start = time.monotonic()
        wall_start = time.time()
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
        with open(os.path.join(d, "host.pid"), "w") as f:
            f.write(str(p.pid))
        try:
            out, err = p.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            p.kill()
            out, err = p.communicate()
        t_end = time.monotonic()
        wall_end = time.time()
        after = report_state(paths["report"])
        written = after is not None and after != before
        steps.append({
            "args": [a.replace(d, "WORK") for a in args],
            "rc": p.returncode,
            "stdout": out.decode("utf-8", "replace"),
            "stderr": err.decode("utf-8", "replace"),
            "duration": round(t_end - t_start, 3),
            "t_start": t_start,
            "t_end": t_end,
            "wall": [wall_start, wall_end],
            "report": after[1].decode() if written else None,
            "report_written": written,
        })
        if proc.poll() is not None and i < len(sc.steps) and not sc.steps[i].restart:
            raise RuntimeError(f"fake died after step {i}")
    fake_rc = stop_fake(proc)
    entries = []
    for log in fakes:
        entries += read_log(log)
    entries.sort(key=lambda e: e["abs"])
    for s in steps:
        s["host"], s["replies"], s["events"] = [], [], []
    exclusive = set()
    for e in entries:
        idx = None
        for k, s in enumerate(steps):
            if e["abs"] >= s["t_start"]:
                idx = k
        if idx is None:
            continue
        s = steps[idx]
        if e["dir"] == "rx":
            if not s["host"]:
                s["first_frame"] = e["abs"]
            s["host"].append([e["type"], e["seq"], e["payload"]])
        elif e["dir"] == "tx" and e["type"] in REPLY_TYPES:
            s["replies"].append([e["type"], e["seq"], e["payload"], e["fields"].get("fault")])
        elif e["dir"] == "ev":
            f = {k: v for k, v in e["fields"].items() if k not in ("ev", "pid")}
            if e["fields"]["ev"] == "host_exclusive":
                exclusive.add(f.get("exclusive"))
            elif e["fields"]["ev"] in FLASH_EVENTS:
                s["events"].append([e["fields"]["ev"], f])
    for s in steps:
        # (f) without the interpreter's start-up: from the first frame on the wire to the exit
        first = s.pop("first_frame", None)
        s["startup"] = round(first - s["t_start"], 3) if first is not None else None
        s["wire"] = round(s["t_end"] - first, 3) if first is not None else None
        del s["t_start"], s["t_end"]
    with open(os.path.join(d, "flash.bin"), "rb") as f:
        final = f.read()
    with open(os.path.join(d, "flash.initial"), "rb") as f:
        initial = f.read()
    result = {
        "scenario": sc.name,
        "title": sc.title,
        "workdir": d,
        "steps": steps,
        "flash_sha256": hashlib.sha256(final).hexdigest(),
        "flash_check": flash_state(initial, final, paths["bundled"], sc),
        "bundled_crc32": zlib.crc32(padded(paths["bundled"])) & 0xFFFFFFFF,
        "fake_rc": fake_rc,
        "exclusive": sorted(exclusive, key=str),
        "overruns": sum(1 for s in steps for ev in s["events"] if ev[0] == "overrun"),
        "violations": sum(1 for s in steps for ev in s["events"] if ev[0] == "violation"),
    }
    with open(os.path.join(d, "result.json"), "w") as f:
        json.dump(result, f, indent=1)
    return result


def flash_state(initial: bytes, final: bytes, bundled: bytes, sc: Scenario) -> dict:
    out = {
        "bootloader_intact": final[:0x8000] == initial[:0x8000],
        "settings_intact": final[S7:] == initial[S7:],
        "unchanged": final == initial,
        "bundled": final == flashed_model(initial, bundled),
        "vector_blank": final[APP:APP + 8] == b"\xFF" * 8,
    }
    flip = sc.faults.get("flip")
    if flip:
        model = bytearray(flashed_model(initial, bundled))
        model[APP + flip[0] + flip[1] // 8] ^= 1 << (flip[1] % 8)
        out["flipped"] = final == bytes(model)
    return out


# -- what the results must be --------------------------------------------------

def signature(frames) -> str:
    """Host frame types, runs collapsed: '06 10 0f 10 11 12x16 13 14 06'."""
    out = []
    for t, _, _ in frames:
        h = f"{t:02x}"
        if out and out[-1][0] == h:
            out[-1][1] += 1
        else:
            out.append([h, 1])
    return " ".join(h if n == 1 else f"{h}x{n}" for h, n in out)


def norm_stdout(s: str) -> str:
    return re.sub(r"written in \d+\.\d s", "written in X s", s)


def norm_report(text):
    if text is None:
        return None
    return re.sub(r'^\{\n  "time": \d+,', '{\n  "time": 0,', text, count=1)


def last_line(s: str) -> str:
    lines = [ln for ln in s.splitlines() if ln.strip()]
    return lines[-1] if lines else ""


def check_expect(sc: Scenario, r: dict, crc: int) -> list:
    """The scenario table, i.e. what mower_flash.py does (E6_DESIGN.md 1.5)."""
    e = sc.expect
    errs = []
    steps = r["steps"]

    def per_step(key):
        v = e.get(key)
        if v is None:
            return [None] * len(steps)
        return v if isinstance(v, list) else [None] * (len(steps) - 1) + [v]

    for i, (s, rc, sig) in enumerate(zip(steps, per_step("rc"), per_step("sig"))):
        if rc is not None and s["rc"] != rc:
            errs.append(f"step {i + 1}: rc {s['rc']}, want {rc} ({last_line(s['stderr'])})")
        got = signature(s["host"])
        if sig is not None and got != sig:
            errs.append(f"step {i + 1}: host frames {got!r}, want {sig!r}")
    last = steps[-1]
    report = json.loads(last["report"]) if last["report"] else None
    if "action" in e:
        if e["action"] is None:
            if last["report_written"]:
                errs.append(f"a report was written: {report}")
        elif report is None:
            errs.append("no report written")
        else:
            if report["action"] != e["action"]:
                errs.append(f"action {report['action']!r}, want {e['action']!r}")
            if "error" in e and report["error"] != e["error"]:
                errs.append(f"error {report['error']!r}, want {e['error']!r}")
            if "error_prefix" in e and not (report["error"] or "").startswith(e["error_prefix"]):
                errs.append(f"error {report['error']!r}, want {e['error_prefix']!r}...")
            if e["action"] in ("flashed", "up_to_date") and report["error"] is not None:
                errs.append(f"error {report['error']!r} on {e['action']}")
    for line in e.get("stdout", []):
        line = line.format(crc=crc)
        if not any(line in s["stdout"].splitlines() for s in steps):
            errs.append(f"stdout lacks {line!r}")
    if "stderr" in e and last_line(last["stderr"]) != e["stderr"]:
        errs.append(f"stderr {last_line(last['stderr'])!r}, want {e['stderr']!r}")
    for i, (s, want) in enumerate(zip(steps, per_step("stderr_prefix"))):
        if want is not None and not last_line(s["stderr"]).startswith(want):
            errs.append(f"step {i + 1}: stderr {last_line(s['stderr'])!r}, want {want!r}...")
    if "retries" in e:
        n = sum(s["stdout"].count("  retry write @0x") for s in steps)
        if n != e["retries"]:
            errs.append(f"{n} retries, want {e['retries']}")
    if "erased" in e:
        erases = [ev for s in steps for ev in s["events"] if ev[0] == "erase"]
        if [ev[1]["count"] for ev in erases] != [e["erased"]]:
            errs.append(f"erases {erases}, want one of {e['erased']} sectors")
    fc = r["flash_check"]
    want = e.get("flash")
    if want and not fc.get(want):
        errs.append(f"flash is not {want}: {fc}")
    if not fc["bootloader_intact"] or not fc["settings_intact"]:
        errs.append(f"the bootloader or the settings sector changed: {fc}")
    if r["overruns"] or r["violations"]:
        errs.append(f"{r['overruns']} overruns, {r['violations']} violations")
    if r["fake_rc"] not in (0, -15):
        errs.append(f"the fake exited with {r['fake_rc']}")
    return errs


def comparable(r: dict) -> dict:
    """(a)-(e) of one result, for comparing implementations; the work
    directory is WORK wherever a message names a file."""
    def here(text):
        return text.replace(r["workdir"], "WORK")

    return {
        "steps": [{
            "rc": s["rc"],
            "stdout": here(norm_stdout(s["stdout"])),
            "stderr": here(last_line(s["stderr"])),
            "report": norm_report(s["report"]),
            "report_written": s["report_written"],
            "host": s["host"],
            "replies": s["replies"],
            "events": s["events"],
        } for s in r["steps"]],
        "flash_sha256": r["flash_sha256"],
        "overruns": r["overruns"],
    }


def compare(a: dict, b: dict) -> list:
    ca, cb = comparable(a), comparable(b)
    errs = []
    if ca["flash_sha256"] != cb["flash_sha256"]:
        errs.append("(b) final flash differs")
    for i, (sa, sb) in enumerate(zip(ca["steps"], cb["steps"]), 1):
        if sa["host"] != sb["host"]:
            n = next((k for k, (x, y) in enumerate(zip(sa["host"], sb["host"])) if x != y),
                     min(len(sa["host"]), len(sb["host"])))
            errs.append(f"(a) step {i}: host frames differ from #{n + 1} "
                        f"({len(sa['host'])} vs {len(sb['host'])}): "
                        f"{sa['host'][n:n + 1]} vs {sb['host'][n:n + 1]}")
        for key, tag in (("replies", "(a) replies"), ("events", "(a) flash operations"),
                         ("report", "(c) report"), ("report_written", "(c) report written"),
                         ("rc", "(d) exit code"), ("stdout", "(e) stdout"), ("stderr", "(e) stderr")):
            if sa[key] != sb[key]:
                errs.append(f"{tag} step {i}: {sa[key]!r} vs {sb[key]!r}"[:600])
    if len(ca["steps"]) != len(cb["steps"]):
        errs.append("different number of steps")
    for i, (sa, sb) in enumerate(zip(a["steps"], b["steps"]), 1):
        # the protocol's own time: first frame on the wire to exit, +-10 % (50 ms
        # floor for scheduling); a killed run ends when it is killed
        ta, tb = sa.get("wire"), sb.get("wire")
        if ta is not None and tb is not None and sa["rc"] != -9:
            if abs(ta - tb) > max(0.1 * max(ta, tb), 0.05):
                errs.append(f"(f) step {i}: {ta:.3f} s vs {tb:.3f} s on the wire "
                            f"(start-up {sa['startup']} s vs {sb['startup']} s)")
    return errs


# -- goldens (G1) --------------------------------------------------------------

def runs(values):
    """[a, a+k, a+2k, ...] as [first, last, k] runs, so 2816 offsets stay one line."""
    out = []
    for v in values:
        if out and out[-1][2] in (None, v - out[-1][1]) and v != out[-1][1]:
            out[-1][2] = v - out[-1][1]
            out[-1][1] = v
        else:
            out.append([v, v, None])
    return [[f"0x{a:x}", f"0x{b:x}", k or 0] for a, b, k in out]


def stream_sha(frames) -> str:
    h = hashlib.sha256()
    for f in frames:
        h.update(json.dumps(f[:3]).encode())
    return h.hexdigest()


def compact(r: dict) -> dict:
    """A golden transcript: (a)-(e). Frames other than BL_WRITE and its BL_ACK
    in full; the writes as their offsets (in runs); all of it hashed."""
    c = comparable(r)
    for s in c["steps"]:
        s["host_signature"] = signature(s["host"])
        s["host_sha256"] = stream_sha(s["host"])
        s["replies_sha256"] = stream_sha(s["replies"])
        writes = [f for f in s["host"] if f[0] == 0x12]
        s["write_offsets"] = runs(struct.unpack("<I", bytes.fromhex(p[:8]))[0] for _, _, p in writes)
        s["write_data_crc32"] = "%08x" % (zlib.crc32(b"".join(bytes.fromhex(p[8:]) for _, _, p in writes)) & 0xFFFFFFFF)
        s["host"] = [[f"{t:02x}", seq, p] for t, seq, p in s["host"] if t != 0x12]
        ok_acks = [f for f in s["replies"] if f[0] == 0x91 and f[2][:4] == "1200" and f[3] is None]
        s["write_acks_ok"] = len(ok_acks)
        s["replies"] = [[f"{t:02x}", seq, p, fault] for t, seq, p, fault in s["replies"]
                        if not (t == 0x91 and p[:4] == "1200" and fault is None)]
        s["events"] = [[name, f] for name, f in s["events"] if name != "write"]
        s.pop("report")
    c["scenario"] = r["scenario"]
    c["title"] = r["title"]
    return c


def golden_files(golden: str, name: str):
    return (os.path.join(golden, f"{name}.transcript.json"), os.path.join(golden, f"{name}.firmware_sync.json"))


def write_golden(golden: str, r: dict) -> None:
    os.makedirs(golden, exist_ok=True)
    tpath, rpath = golden_files(golden, r["scenario"])
    with open(tpath, "w") as f:
        json.dump(compact(r), f, indent=1)
        f.write("\n")
    report = norm_report(r["steps"][-1]["report"])
    if report is not None:
        with open(rpath, "w") as f:
            f.write(report)  # byte for byte what mower_flash.py wrote, time = 0
    elif os.path.exists(rpath):
        os.unlink(rpath)


def check_golden(golden: str, r: dict) -> list:
    tpath, rpath = golden_files(golden, r["scenario"])
    if not os.path.exists(tpath):
        return [f"no golden {tpath}"]
    with open(tpath) as f:
        want = json.load(f)
    got = json.loads(json.dumps(compact(r)))
    errs = []
    for key in ("flash_sha256", "overruns"):
        if got[key] != want[key]:
            errs.append(f"golden {key}: {got[key]!r} vs {want[key]!r}")
    for i, (sg, sw) in enumerate(zip(got["steps"], want["steps"]), 1):
        for key in sw:
            if sg.get(key) != sw[key]:
                errs.append(f"golden step {i} {key}: {str(sg.get(key))[:300]} vs {str(sw[key])[:300]}")
    if len(got["steps"]) != len(want["steps"]):
        errs.append("golden: different number of steps")
    report = norm_report(r["steps"][-1]["report"])
    stored = open(rpath).read() if os.path.exists(rpath) else None
    if report != stored:
        errs.append(f"golden report: {report!r} vs {stored!r}")
    return errs


# -- command line ----------------------------------------------------------------

def parse_impls(specs):
    impls = []
    for spec in specs:
        name, _, cmd = spec.partition("=")
        if not cmd:
            raise SystemExit(f"--impl wants NAME=CMD, got {spec!r}")
        impls.append((name, shlex.split(cmd)))
    return impls


def select(args):
    if args.scenarios:
        want = args.scenarios.split(",")
        chosen = [sc for sc in SCENARIOS if sc.name in want]
        missing = set(want) - {sc.name for sc in chosen}
        if missing:
            raise SystemExit(f"unknown scenarios: {sorted(missing)}")
    else:
        chosen = [sc for sc in SCENARIOS if (sc.real and args.bundled_bin) or (not sc.real and (sc.quick or args.all))]
    if any(sc.real for sc in chosen) and not args.bundled_bin:
        raise SystemExit("the R scenarios need --bundled-bin")
    return chosen


def run_all(args, impls, out_root):
    lib = args.bl_lib or build_lib(out_root)
    cache = os.path.join(out_root, "fixtures")
    results = {}
    for sc in select(args):
        for name, cmd in impls:
            d = os.path.join(out_root, name, sc.name)
            r = run_scenario(sc, cmd, d, lib, args.bootloader_bin, args.timeout, cache, args.bundled_bin)
            results[(sc.name, name)] = r
            s = r["steps"][-1]
            print(f"{sc.name:5} {name:7} rc={s['rc']:<3} {signature(s['host'])[:46]:46} "
                  f"{sum(x['duration'] for x in r['steps']):7.2f} s  {sc.title}", flush=True)
    return results


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["check", "run", "compare", "golden"])
    ap.add_argument("dirs", nargs="*", help="compare: the two `run` output directories")
    ap.add_argument("--impl", action="append", default=[], help="NAME=CMD (repeatable)")
    ap.add_argument("--scenarios", default="", help="comma-separated names (default: the quick ones)")
    ap.add_argument("--all", action="store_true", help="also the slow ones (images up to 352 KB)")
    ap.add_argument("--golden", default="", help="G1 directory (deploy/test/golden/firmware_sync)")
    ap.add_argument("--out", default="", help="run: where the results go")
    ap.add_argument("--work", default="", help="check: work directory (default: a temporary one)")
    ap.add_argument("--keep", action="store_true", help="check: keep the work directory")
    ap.add_argument("--bl-lib", default="", help="a prebuilt bl_sim.so instead of compiling one")
    ap.add_argument("--bootloader-bin", default="", help="the real bootloader.bin for sectors 0-1 of the fake flash")
    ap.add_argument("--bundled-bin", default="",
                    help="a real firmware .bin (+ its .json next to it) for the R scenarios, e.g. the image's "
                         "/opt/mower/firmware/mower_robot_firmware.bin")
    ap.add_argument("--timeout", type=float, default=900.0, help="per flasher run, seconds")
    args = ap.parse_args()

    if args.cmd == "compare":
        if len(args.dirs) != 2:
            raise SystemExit("compare DIR_A DIR_B")
        bad = 0
        names = sorted(n for n in os.listdir(args.dirs[0]) if os.path.exists(os.path.join(args.dirs[0], n, "result.json")))
        for n in names:
            pb = os.path.join(args.dirs[1], n, "result.json")
            if not os.path.exists(pb):
                print(f"{n}: missing in {args.dirs[1]}")
                bad += 1
                continue
            ra = json.load(open(os.path.join(args.dirs[0], n, "result.json")))
            rb = json.load(open(pb))
            if any(sc.name == n and sc.informational for sc in SCENARIOS):
                for tag, r in (("A", ra), ("B", rb)):
                    print(f"{n:5} INFO {tag}: " + "; ".join(f"step {i}: rc={s['rc']} {last_line(s['stderr'])[:120]!r}"
                                                         for i, s in enumerate(r["steps"], 1)))
                continue
            errs = compare(ra, rb)
            ta = sum(s["duration"] for s in ra["steps"])
            tb = sum(s["duration"] for s in rb["steps"])
            wa = sum(s.get("wire") or 0 for s in ra["steps"])
            wb = sum(s.get("wire") or 0 for s in rb["steps"])
            print(f"{n:5} {'SAME' if not errs else 'DIFF'}  {ta:7.2f} s vs {tb:7.2f} s  (on the wire {wa:7.2f} vs {wb:7.2f})  "
                  f"exclusive {ra['exclusive']} vs {rb['exclusive']}")
            for e in errs:
                print(f"      {e}")
            bad += bool(errs)
        informational = sum(1 for n in names if any(sc.name == n and sc.informational for sc in SCENARIOS))
        print(f"{len(names) - informational - bad}/{len(names) - informational} scenarios identical")
        return 1 if bad else 0

    impls = parse_impls(args.impl)
    if not impls:
        raise SystemExit("at least one --impl NAME=CMD")
    if args.cmd == "run":
        if len(impls) != 1 or not args.out:
            raise SystemExit("run: one --impl and --out DIR")
        os.makedirs(args.out, exist_ok=True)
        lib = args.bl_lib or build_lib(args.out)
        args.bl_lib = lib
        name, cmd = impls[0]
        cache = os.path.join(args.out, "fixtures")
        for sc in select(args):
            r = run_scenario(sc, cmd, os.path.join(args.out, sc.name), lib, args.bootloader_bin, args.timeout, cache,
                             args.bundled_bin)
            s = r["steps"][-1]
            print(f"{sc.name:5} rc={s['rc']:<3} {signature(s['host'])[:46]:46} "
                  f"{sum(x['duration'] for x in r['steps']):7.2f} s  {sc.title}", flush=True)
        return 0

    work = args.work or tempfile.mkdtemp(prefix="flash_check.")
    os.makedirs(work, exist_ok=True)
    results = run_all(args, impls, work)
    bad = 0
    for sc in select(args):
        first = results[(sc.name, impls[0][0])]
        crc = first["bundled_crc32"]
        if sc.informational:
            for name, _ in impls:
                steps = results[(sc.name, name)]["steps"]
                print(f"INFO {sc.name} {name}: " + "; ".join(
                    f"step {i}: rc={s['rc']} {last_line(s['stderr'])[:120]!r}" for i, s in enumerate(steps, 1)))
            continue
        for name, _ in impls:
            r = results[(sc.name, name)]
            errs = check_expect(sc, r, crc)
            if name != impls[0][0]:
                errs += compare(first, r)
            if sc.real:
                pass  # depends on the image's firmware: no golden
            elif args.cmd == "golden" and name == impls[0][0] and not errs:
                write_golden(args.golden, r)
            elif args.golden:
                errs += check_golden(args.golden, r)
            for e in errs:
                print(f"FAIL {sc.name} {name}: {e}")
            bad += bool(errs)
        if len(impls) > 1:
            print(f"{sc.name:5} exclusive: " + ", ".join(f"{n} {results[(sc.name, n)]['exclusive']}" for n, _ in impls))
    total = len(select(args)) * len(impls)
    print(f"{total - bad}/{total} runs as expected" + (f" (work dir {work})" if args.keep or bad else ""))
    if not args.keep and not bad and not args.work:
        shutil.rmtree(work)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
