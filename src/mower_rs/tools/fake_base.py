#!/usr/bin/env python3
"""A fake STM32 base on a pty, for the Phase B differential test.

The real serial port cannot be opened twice, so `ros2_control` +
`mower_hardware` and the Rust `mower_base` node are run one after the other
against *this* instead: it speaks the same UART protocol
(`src/mower_hardware/src/mower_protocol.cpp`, see also
`src/mower_rs/crates/mower_base_core/README.md` for the frame format),
answers with 0x85 wheel feedback at the firmware's 50 ms period from a
deterministic first-order wheel model driven by the received 0x01 commands,
and writes every frame it sees or sends to a log.

    socat -d -d pty,raw,echo=0,link=/tmp/base_host pty,raw,echo=0,link=/tmp/base_stm &
    ./fake_base.py --port /tmp/base_stm --log /tmp/fake_base.jsonl

or, where there is no socat (the runtime image), let it make the pty:

    ./fake_base.py --pty-link /dev/stmcom --log /tmp/fake_base.jsonl

`--mute-file PATH`: while PATH exists the fake is a UART with both leads
pulled -- nothing is sent and everything received is dropped (so its own
300 ms command timeout stops the wheels), which on the robot's native UART
is also no error at all on the host side. Used by `base_harness.py
--scenario pull` / `pullpush`.

`--deaf-file PATH`: while PATH exists only the LubanCat TX -> STM32 RX lead
is out (the 2026-09-28 pull): everything received is dropped, but the fake
keeps sending, and its 0x81 reports COMMAND_TIMEOUT with a growing
`command_age_ms` once its 300 ms timeout has stopped the wheels. Used by
`base_harness.py --scenario txpull`.

0x81 follows `firmware/Module/Src/motor.cpp`: COMMAND_VALID from the first
accepted 0x01 on (only a restart clears it), COMMAND_TIMEOUT when there has
been none yet or the last one is older than its `command_timeout_ms`,
`command_age_ms` saturating at 65535 (65535 before the first).

0x04 is applied at once and answered with a 0x84 (CLOSED_LOOP as
requested, LAST_APPLY_OK, FLASH_VALID / LAST_SAVE_OK once saved). With
`persist_to_flash` the fake stalls like the STM32's sector-7 erase
(`--flash-stall-s`, 1 s): nothing sent, nothing taken, and what the host
wrote meanwhile is lost (the firmware's 256-byte DMA ring has long
wrapped), so the first status batch after it reports COMMAND_TIMEOUT.
Used by `base_harness.py --scenario autotune`.

Log format: one JSON object per line, so the comparison script can diff two
runs frame by frame.

    {"t": 12.3456, "dir": "rx"|"tx", "type": 1, "seq": 7,
     "payload": "0a00f6ff2c01", "fields": {...}}

`t` is seconds since the log was opened (`time.monotonic()` based), so two
runs line up once shifted to their first 0x01.

`--bootloader-lib LIB` makes it the whole board, UART bootloader included,
for the firmware flasher (`firmware/tools/mower_flash.py`, the Rust
`mower_flash`); `tools/flash_check.py` builds LIB and drives the scenarios:

    ./fake_base.py --pty-link /tmp/stmcom --log /tmp/fake.jsonl \\
        --bootloader-lib bl_sim.so --flash-file flash.bin --catalog builds/ \\
        [--faults '{"drop_reply": [{"cmd": 18, "offset": 1024}]}']

LIB is the *unmodified* `firmware/bootloader/Src/bl_protocol.c` compiled
with `tools/bl_sim/sim_flash.c` (the 512 KB flash behind `bl_flash.h`, kept
in `--flash-file` across runs like the chip keeps it across power cycles).
The board boots like `bl_main.c`: the bootloader first, which stays when the
RAM mailbox holds the boot request or the app's vector table is invalid,
otherwise waits 500 ms for a PING and jumps. Received bytes go to
`BlProtocol_FeedByte` one by one; erases, writes and CRCs take the STM32's
time, the reply leaves when they are done, and bytes arriving meanwhile are
lost the way the UART's overrun loses them (logged as `overrun`; a
stop-and-wait host never causes one). The app that runs after the jump is
whatever the flash holds: the build of `--catalog` (fw_manifest.py
manifests, optionally with a `"fake"` object: `no_0x87`, `unversioned`,
`ignore_enter`, `no_enter_ack`) whose size and CRC-32 match the app region;
it streams the status frames, answers 0x06 with that build's 0x87 and
reboots into the bootloader on a 0x0F with the magic (0x8F, mailbox, reset).
No match: a broken app, silent and deaf. Faults (`--faults`, JSON): see
`FakeStm32.__init__`. Events go to the same log with `"dir": "ev"`.

Nothing here is used on the robot; it exists only for the test.
"""

from __future__ import annotations

import argparse
import ctypes
import fcntl
import json
import os
import random
import signal
import struct
import sys
import time
import tty
import zlib

SOF0 = 0xA5
SOF1 = 0x5A
VERSION = 0x01

# host -> STM32
WHEEL_SPEED_COMMAND = 0x01
LAWER_MOTOR_COMMAND = 0x02
WS2812_COMMAND = 0x03
PID_CONFIG_COMMAND = 0x04
POWER_COMMAND = 0x05
INFO_REQUEST = 0x06
SERVO_COMMAND = 0x07
# STM32 -> host
MOTOR_STATUS = 0x81
LAWER_MOTOR_STATUS = 0x82
WS2812_STATUS = 0x83
PID_CONFIG_STATUS = 0x84
WHEEL_FEEDBACK_STATUS = 0x85
POWER_STATUS = 0x86
FIRMWARE_INFO = 0x87
SERVO_STATUS = 0x88
CHARGER_STATUS = 0x89


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build(frame_type: int, seq: int, payload: bytes) -> bytes:
    body = bytes([VERSION, frame_type, seq & 0xFF, len(payload)]) + payload
    crc = crc16_ccitt_false(body)
    return bytes([SOF0, SOF1]) + body + struct.pack("<H", crc)


class Parser:
    """The same resync rules as mower_protocol.cpp's FrameParser."""

    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(bytes([SOF0, SOF1]))
            if i < 0:
                # keep a trailing lone SOF0 in case the pair is split
                if self.buf and self.buf[-1] == SOF0:
                    del self.buf[:-1]
                else:
                    self.buf.clear()
                break
            if i:
                del self.buf[:i]
            if len(self.buf) < 6:
                break
            length = self.buf[5]
            total = 6 + length + 2
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2 : 6 + length])
            crc = struct.unpack_from("<H", self.buf, 6 + length)[0]
            if crc16_ccitt_false(body) == crc and self.buf[2] == VERSION:
                out.append((self.buf[3], self.buf[4], bytes(self.buf[6 : 6 + length])))
                del self.buf[:total]
            else:
                del self.buf[:2]
        return out


class Wheel:
    """First-order lag towards the commanded rpm, then an encoder counter.

    Deterministic: the same command sequence always produces the same
    feedback, which is what lets the two runs be compared.
    """

    TAU_S = 0.15

    def __init__(self, counts_per_rev: float, max_rpm: float) -> None:
        self.counts_per_rev = counts_per_rev
        self.max_rpm = max_rpm
        self.target_rpm = 0.0
        self.measured_rpm = 0.0
        self.total_counts = 0
        self.rev = 0.0

    def command(self, permille: int) -> None:
        self.target_rpm = permille / 1000.0 * self.max_rpm

    def step(self, dt: float) -> None:
        alpha = 1.0 - pow(2.718281828459045, -dt / self.TAU_S)
        self.measured_rpm += (self.target_rpm - self.measured_rpm) * alpha
        self.rev += self.measured_rpm / 60.0 * dt
        self.total_counts = int(round(self.rev * self.counts_per_rev)) & 0xFFFFFFFF
        if self.total_counts >= 1 << 31:
            self.total_counts -= 1 << 32


class FakeBase:
    STATUS_PERIOD_S = 0.050
    COMMAND_TIMEOUT_S = 0.300

    def __init__(self, port: str, log_path: str, max_rpm: float, counts_per_rev: float,
                 pty_link: str = "", mute_file: str = "", deaf_file: str = "",
                 flash_stall_s: float = 1.0) -> None:
        if pty_link:
            # socat's `pty,raw,echo=0,link=...`: the driver's end is the
            # slave, raw from the start so nothing is echoed back, and held
            # open here so a driver restart does not hang the pty up.
            self.fd, self._slave = os.openpty()
            tty.setraw(self._slave)
            os.set_blocking(self.fd, False)
            if os.path.lexists(pty_link):
                os.unlink(pty_link)
            os.symlink(os.ttyname(self._slave), pty_link)
        else:
            self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.mute_file = mute_file
        self.muted = False
        self.deaf_file = deaf_file
        self.deaf = False
        self.flash_stall_s = flash_stall_s
        self.stalled_until = 0.0
        self.log = open(log_path, "w", buffering=1)
        self.t0 = time.monotonic()
        self.log.write(
            json.dumps({"t": 0.0, "dir": "meta", "type": 0, "seq": 0, "payload": "",
                        "fields": {"t0_monotonic": self.t0}}, separators=(",", ":")) + "\n"
        )
        self.parser = Parser()
        self.seq = 0
        self.left = Wheel(counts_per_rev, max_rpm)
        self.right = Wheel(counts_per_rev, max_rpm)
        self.cmd_permille = (0, 0)
        self.last_command_at = None
        self.command_timeout_s = self.COMMAND_TIMEOUT_S
        self.blade_permille = 0
        self.blade_last_at = None
        self.servo_pulse = 0
        self.led = (0, 0, 0, 0, 0)
        self.pid = (2.0, 0.6, 0.0, 2.0, 0.6, 0.0, 0, 1)
        # PID_FLAG_CLOSED_LOOP | PID_FLAG_FLASH_VALID, as a board that booted
        # with saved gains reports it
        self.pid_flags = 0x03
        self.last_rx_seq = {}

    # -- logging ---------------------------------------------------------
    def _log(self, direction: str, frame_type: int, seq: int, payload: bytes, fields=None):
        self.log.write(
            json.dumps(
                {
                    "t": round(time.monotonic() - self.t0, 6),
                    "dir": direction,
                    "type": frame_type,
                    "seq": seq,
                    "payload": payload.hex(),
                    "fields": fields or {},
                },
                separators=(",", ":"),
            )
            + "\n"
        )

    def _next_seq(self) -> int:
        s = self.seq
        self.seq = (self.seq + 1) & 0xFF
        return s

    def _send(self, frame_type: int, payload: bytes, fields=None) -> None:
        if self.muted:
            return
        seq = self._next_seq()
        try:
            os.write(self.fd, build(frame_type, seq, payload))
        except BlockingIOError:
            # --pty-link before the driver has opened its end: the pty
            # buffer is full, and a UART nobody listens to loses the frame.
            return
        self._log("tx", frame_type, seq, payload, fields)

    # -- host -> STM32 ---------------------------------------------------
    def on_frame(self, frame_type: int, seq: int, payload: bytes, now: float) -> None:
        fields = {}
        if frame_type == WHEEL_SPEED_COMMAND and len(payload) == 8:
            left, right, timeout, _ = struct.unpack("<hhHH", payload)
            fields = {"left": left, "right": right, "timeout_ms": timeout}
            self.cmd_permille = (left, right)
            self.left.command(left)
            self.right.command(right)
            self.last_command_at = now
            # MOTOR_DEFAULT_COMMAND_TIMEOUT_MS for 0
            self.command_timeout_s = (timeout or 200) / 1000.0
        elif frame_type == LAWER_MOTOR_COMMAND and len(payload) == 8:
            permille, timeout = struct.unpack("<hH", payload[:4])
            fields = {"permille": permille, "timeout_ms": timeout}
            self.blade_permille = permille
            self.blade_last_at = now
        elif frame_type == WS2812_COMMAND and len(payload) == 8:
            mode, r, g, b, period = struct.unpack("<BBBBH", payload[:6])
            fields = {"mode": mode, "r": r, "g": g, "b": b, "period_ms": period}
            self.led = (mode, r, g, b, period)
        elif frame_type == PID_CONFIG_COMMAND and len(payload) == 28:
            vals = struct.unpack("<ffffffBBH", payload)
            fields = dict(zip(["lkp", "lki", "lkd", "rkp", "rki", "rkd", "persist", "closed"], vals[:8]))
            self.pid = vals[:8]
            persist, closed = vals[6], vals[7]
            # CLOSED_LOOP as asked, LAST_APPLY_OK; a save adds FLASH_VALID
            # and LAST_SAVE_OK, and stalls the board for the sector erase
            flags = (self.pid_flags & 0x06) | 0x08 | (0x01 if closed else 0)
            if persist:
                flags |= 0x06
                self.stalled_until = now + self.flash_stall_s
                fields["flash_stall_s"] = self.flash_stall_s
            self.pid_flags = flags
        elif frame_type == SERVO_COMMAND and len(payload) == 8:
            pulse, hold = struct.unpack("<HH", payload[:4])
            fields = {"pulse_us": pulse, "hold_ms": hold}
            self.servo_pulse = pulse
        elif frame_type == INFO_REQUEST:
            fields = {"info_request": True}
        elif frame_type == POWER_COMMAND and len(payload) == 4:
            fields = {"action": payload[0]}
        self.last_rx_seq[frame_type] = seq
        self._log("rx", frame_type, seq, payload, fields)
        if frame_type == INFO_REQUEST:
            self.send_firmware_info()
        if frame_type == PID_CONFIG_COMMAND and not self.stalled_until > now:
            self.send_pid_status()

    # -- STM32 -> host ---------------------------------------------------
    def send_wheel_feedback(self) -> None:
        # <hhhhhhiiBB>: target/measured rpm x100, pid output, counts, flags, seq
        payload = struct.pack(
            "<hhhhhhiiB3x",
            int(round(self.left.target_rpm * 100)),
            int(round(self.left.measured_rpm * 100)),
            int(round(self.right.target_rpm * 100)),
            int(round(self.right.measured_rpm * 100)),
            self.cmd_permille[0],
            self.cmd_permille[1],
            self.left.total_counts,
            self.right.total_counts,
            0x03,
        )
        self._send(
            WHEEL_FEEDBACK_STATUS,
            payload,
            {
                "left_measured_rpm": round(self.left.measured_rpm, 4),
                "right_measured_rpm": round(self.right.measured_rpm, 4),
                "left_counts": self.left.total_counts,
                "right_counts": self.right.total_counts,
            },
        )

    def send_motor_status(self, now: float) -> None:
        # motor.cpp control_update_50hz: valid from the first 0x01 until a
        # restart; timeout = !valid || age > command_timeout_ms
        valid = self.last_command_at is not None
        age = int((now - self.last_command_at) * 1000) if valid else 0xFFFFFFFF
        timeout = not valid or age > self.command_timeout_s * 1000
        flags = (0x01 if valid else 0) | (0x02 if timeout else 0)
        applied = (0, 0) if timeout else self.cmd_permille
        payload = struct.pack(
            "<hhhhHBB",
            self.cmd_permille[0] if valid else 0,
            self.cmd_permille[1] if valid else 0,
            int(applied[0] * 2),
            int(applied[1] * 2),
            min(age, 65535),
            flags,
            self.last_rx_seq.get(WHEEL_SPEED_COMMAND, 0),
        )
        self._send(MOTOR_STATUS, payload, {"command_age_ms": min(age, 65535), "flags": flags})

    def send_blade_status(self, now: float) -> None:
        age = 0 if self.blade_last_at is None else int((now - self.blade_last_at) * 1000)
        payload = struct.pack(
            "<hhHBB",
            self.blade_permille,
            self.blade_permille * 2,
            min(age, 65535),
            0x01,
            self.last_rx_seq.get(LAWER_MOTOR_COMMAND, 0),
        )
        self._send(LAWER_MOTOR_STATUS, payload)

    def send_servo_status(self) -> None:
        payload = struct.pack(
            "<HHHBB", self.servo_pulse, 0, 10, 0x01 if self.servo_pulse else 0x00,
            self.last_rx_seq.get(SERVO_COMMAND, 0)
        )
        self._send(SERVO_STATUS, payload)

    def send_led_status(self) -> None:
        mode, r, g, b, period = self.led
        payload = struct.pack("<BBBBHBB", mode, r, g, b, period, 0x01,
                              self.last_rx_seq.get(WS2812_COMMAND, 0))
        self._send(WS2812_STATUS, payload)

    def send_pid_status(self) -> None:
        payload = struct.pack(
            "<ffffffBBH", *self.pid[:6], self.pid_flags,
            self.last_rx_seq.get(PID_CONFIG_COMMAND, 0), 0
        )
        self._send(PID_CONFIG_STATUS, payload, {"flags": self.pid_flags})

    def send_power_status(self) -> None:
        payload = struct.pack("<BBBBHH", 0, 0x02, 0, 0, 0, 0)
        self._send(POWER_STATUS, payload)

    def send_charger_status(self) -> None:
        payload = struct.pack("<HHHHHBBHBx", 2537, 125, 28, 1, 2, 0x03, 0, 40, 0)
        self._send(CHARGER_STATUS, payload)

    def send_firmware_info(self) -> None:
        payload = struct.pack("<BBBBIIB3x", 0, 6, 0, VERSION, 0xABCD1234, 1758600000, 0)
        self._send(FIRMWARE_INFO, payload)

    # -- main loop -------------------------------------------------------
    def run(self, duration: float) -> None:
        next_status = time.monotonic()
        next_slow = time.monotonic()
        last_step = time.monotonic()
        end = time.monotonic() + duration
        self.send_firmware_info()
        stalled = False
        while time.monotonic() < end:
            now = time.monotonic()
            if now < self.stalled_until:
                # the sector erase: the CPU runs nothing, UART included
                if not stalled:
                    stalled = True
                    self._log("meta", 0, 0, b"", {"stalled": True})
                time.sleep(0.002)
                continue
            try:
                data = os.read(self.fd, 4096)
            except BlockingIOError:
                data = b""
            except OSError:
                data = b""
            now = time.monotonic()
            if stalled:
                # what came in during the erase overran the DMA ring
                stalled = False
                self._log("meta", 0, 0, b"", {"stalled": False, "dropped": len(data)})
                data = b""
                self.parser = Parser()
                # the save's outcome goes out with the first status batch
                self.send_pid_status()
            muted = bool(self.mute_file) and os.path.exists(self.mute_file)
            if muted != self.muted:
                self.muted = muted
                self._log("meta", 0, 0, b"", {"muted": muted})
            deaf = bool(self.deaf_file) and os.path.exists(self.deaf_file)
            if deaf != self.deaf:
                self.deaf = deaf
                self._log("meta", 0, 0, b"", {"deaf": deaf})
            if muted or deaf:
                data = b""
            for frame_type, seq, payload in self.parser.feed(data):
                self.on_frame(frame_type, seq, payload, now)
                if self.stalled_until > now:
                    break  # the rest of this read is lost with the erase
            if self.stalled_until > now:
                continue
            # the firmware's own command timeout
            if self.last_command_at is not None and now - self.last_command_at > self.command_timeout_s:
                self.left.command(0)
                self.right.command(0)
            if now >= next_status:
                dt = now - last_step
                last_step = now
                self.left.step(dt)
                self.right.step(dt)
                self.send_wheel_feedback()
                self.send_motor_status(now)
                next_status += self.STATUS_PERIOD_S
                if next_status < now:
                    next_status = now + self.STATUS_PERIOD_S
            if now >= next_slow:
                self.send_power_status()
                self.send_charger_status()
                self.send_blade_status(now)
                self.send_servo_status()
                self.send_led_status()
                self.send_pid_status()
                self.send_firmware_info()
                next_slow = now + 1.0
            time.sleep(0.002)
        self.log.close()
        os.close(self.fd)


# -- the UART bootloader (--bootloader-lib) ----------------------------------

ENTER_BOOTLOADER = 0x0F
ENTER_BOOTLOADER_ACK = 0x8F
BL_PING = 0x10
BL_WRITE = 0x12
BL_INFO = 0x90
BL_ACK = 0x91
BOOT_REQUEST_MAGIC = 0xB007B007
FLASH_BASE = 0x08000000
APP_OFFSET = 0x8000  # BOOT_APP_START_ADDRESS - BOOT_FLASH_BASE_ADDRESS
APP_MAX_SIZE = 0x58000
FLASH_SIZE = 512 * 1024
GRACE_S = 0.5  # BOOT_GRACE_PERIOD_MS
SECTOR_SIZES = [16384, 16384, 16384, 16384, 65536, 131072, 131072, 131072]
SIM_EVENTS = {1: "erase", 2: "erase_fail", 3: "write", 4: "write_fail", 5: "crc", 6: "flip",
              7: "violation"}
BYTE_S = 10 / 115200  # one 8N1 character at 115200
TIOCEXCL, TIOCNXCL, TIOCGEXCL = 0x540C, 0x540D, 0x80045440

# How long the STM32F411 is busy (datasheet typical, x32 programming): a
# sector erase by size, one word programmed, one byte of the bit-by-bit CRC
# at the bootloader's 16 MHz.
ERASE_S = {16384: 0.25, 65536: 0.55, 131072: 1.0}
WORD_S = 16e-6
CRC_BYTE_S = 4e-6

_TX_FN = ctypes.CFUNCTYPE(None, ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint16)
_EV_FN = ctypes.CFUNCTYPE(None, ctypes.c_int, ctypes.c_uint32, ctypes.c_uint32)


def load_catalog(directory: str):
    """The builds the fake recognises in its app region, by size and CRC-32."""
    builds = []
    for name in sorted(os.listdir(directory)) if directory else []:
        if name.endswith(".json"):
            with open(os.path.join(directory, name)) as f:
                m = json.load(f)
            m.setdefault("fake", {})
            m["_name"] = name
            builds.append(m)
    return builds


class FakeStm32(FakeBase):
    """The board with its bootloader: see the module docstring."""

    APP_BOOT_S = 0.1  # reset to the app's first frame
    RESET_S = 0.005  # 0x8F sent -> NVIC_SystemReset -> bootloader running

    def __init__(self, port: str, log_path: str, max_rpm: float, counts_per_rev: float,
                 pty_link: str, lib_path: str, flash_file: str, catalog_dir: str,
                 faults: dict, host_pid_file: str = "") -> None:
        """`faults` (all optional):

        - `dead`: no board on the port; nothing sent, everything dropped
        - `erase_error`: BL_ERASE fails (FLASH_ERROR) before erasing anything
        - `write_error`: OFFSET whose BL_WRITE fails (FLASH_ERROR)
        - `flip`: [OFFSET, BIT], a bit that flips right after that write
        - `drop_reply` / `corrupt_reply` / `bad_len_reply` / `delay_reply`:
          lists of `{"cmd": C, "offset": O, "times": N[, "s": S]}`; a reply to
          command C (0x10 for the BL_INFO answering a PING; `offset` matches a
          BL_ACK's value, i.e. a write's offset) is not sent / sent with a
          broken CRC / sent one byte short with a good CRC / sent S seconds
          late, the first N times (1)
        - `bytewise`: S, bootloader replies leave one byte every S seconds
        - `noise`: SEED, junk (never 0xA5) before every bootloader reply
        - `power_off_at_write`: N, the board loses power while the Nth
          BL_WRITE is being acknowledged: the flash file is saved and the
          fake exits with 3 (the pty hangs up)
        - `kill_host_at_write`: N, SIGKILL to the pid in `host_pid_file`
          when the Nth BL_WRITE has been taken (a `docker kill` of the
          flasher); the tty's TIOCEXCL is cleared afterwards, as the last
          close of a real UART does, unless `keep_excl` (this pty's slave
          stays open here, so a flasher that set TIOCEXCL leaves it set)
        """
        super().__init__(port, log_path, max_rpm, counts_per_rev, pty_link)
        lib = ctypes.CDLL(lib_path)
        lib.sim_flash.restype = ctypes.POINTER(ctypes.c_uint8)
        lib.sim_flash_size.restype = ctypes.c_uint32
        lib.sim_session.restype = ctypes.c_int
        lib.sim_boot.argtypes = [ctypes.c_int]
        lib.BlProtocol_FeedByte.argtypes = [ctypes.c_uint8]
        lib.sim_fault_erase.argtypes = [ctypes.c_int]
        lib.sim_fault_write.argtypes = [ctypes.c_uint32]
        lib.sim_fault_flip.argtypes = [ctypes.c_uint32, ctypes.c_uint32]
        assert lib.sim_flash_size() == FLASH_SIZE
        self.lib = lib
        self._tx_cb = _TX_FN(self._bl_tx)
        self._ev_cb = _EV_FN(self._bl_event)
        lib.sim_set_callbacks(self._tx_cb, self._ev_cb)
        self.flash_file = flash_file
        data = b"\xFF" * FLASH_SIZE
        if flash_file and os.path.exists(flash_file):
            with open(flash_file, "rb") as f:
                data = f.read()
            if len(data) != FLASH_SIZE:
                raise SystemExit(f"{flash_file}: {len(data)} bytes, want {FLASH_SIZE}")
        ctypes.memmove(lib.sim_flash(), data, FLASH_SIZE)
        self.catalog = load_catalog(catalog_dir)
        self.faults = faults
        if faults.get("erase_error"):
            lib.sim_fault_erase(1)
        if "write_error" in faults:
            lib.sim_fault_write(int(faults["write_error"]))
        if "flip" in faults:
            lib.sim_fault_flip(int(faults["flip"][0]), int(faults["flip"][1]))
        self.reply_rules = []
        for kind in ("drop_reply", "corrupt_reply", "bad_len_reply", "delay_reply"):
            for rule in faults.get(kind, []):
                self.reply_rules.append(dict(rule, kind=kind, left=int(rule.get("times", 1))))
        self.noise = random.Random(faults["noise"]) if "noise" in faults else None
        self.host_pid_file = host_pid_file
        self.mode = "off"
        self.build = None
        self.mailbox = 0
        self.boot_at = 0.0
        self.due = None  # (when, what): "reset" | "jump" | "app"
        self.rx_log = Parser()
        self.tx_queue: list = []  # (when, bytes)
        self.busy_until = 0.0
        self.busy_s = 0.0  # accumulated during one BlProtocol_FeedByte
        self.busy_bytes = 0
        self.held_byte = None
        self.writes_seen = 0
        self.overruns = 0
        self.after_feed: list = []
        self.feed_now = 0.0
        self.ready_logged = False
        self.host_excl = None
        self.stop = False
        signal.signal(signal.SIGTERM, self._on_signal)
        signal.signal(signal.SIGINT, self._on_signal)

    # -- plumbing ----------------------------------------------------------
    def _on_signal(self, *_):
        self.stop = True

    def _event(self, name: str, **fields) -> None:
        fields["ev"] = name
        self._log("ev", 0, 0, b"", fields)

    def flash_bytes(self) -> bytes:
        return ctypes.string_at(self.lib.sim_flash(), FLASH_SIZE)

    def save_flash(self) -> None:
        if not self.flash_file:
            return
        tmp = self.flash_file + ".tmp"
        with open(tmp, "wb") as f:
            f.write(self.flash_bytes())
        os.replace(tmp, self.flash_file)

    def _write(self, data: bytes) -> None:
        try:
            os.write(self.fd, data)
        except BlockingIOError:
            pass  # nobody has the port open: the bytes are lost, like on a UART

    def _set_mode(self, mode: str, now: float, **fields) -> None:
        self.mode = mode
        self._event("state", state=mode, **fields)
        ready = mode in ("app", "broken", "dead") or (mode == "bl" and self.lib.sim_session() & 0x04)
        if ready and not self.ready_logged:
            self.ready_logged = True
            self._event("ready", state=mode)

    # -- the bootloader ----------------------------------------------------
    def _bl_event(self, kind: int, a: int, b: int) -> None:
        name = SIM_EVENTS.get(kind, str(kind))
        if name == "erase":
            self.busy_s += sum(ERASE_S[SECTOR_SIZES[s]] for s in range(a, a + b))
            self._event("erase", first=a, count=b)
        elif name == "write":
            self.busy_s += WORD_S * (b // 4)
            self._event("write", offset=a, len=b)
        elif name == "crc":
            self.busy_s += CRC_BYTE_S * b
            self._event("crc", address=a, len=b)
        else:
            self._event(name, a=a, b=b)

    def _reply_rule(self, ftype: int, payload: bytes):
        cmd = BL_PING if ftype == BL_INFO else (payload[0] if ftype == BL_ACK and payload else None)
        for rule in self.reply_rules:
            if rule["left"] <= 0 or rule["cmd"] != cmd:
                continue
            if "offset" in rule and (len(payload) < 6 or struct.unpack_from("<I", payload, 2)[0] != rule["offset"]):
                continue
            rule["left"] -= 1
            return rule
        return None

    def _bl_tx(self, data, length: int) -> None:
        frame = bytes(data[:length])
        ftype, seq, payload = frame[3], frame[4], frame[6:-2]
        # the reply leaves once the flash operation and its own transmission are done
        self.busy_s += BYTE_S * length
        when = self.feed_now + self.busy_s
        rule = self._reply_rule(ftype, payload)
        fields = {}
        if rule is not None:
            fields["fault"] = rule["kind"]
            if rule["kind"] == "drop_reply":
                self._log("tx", ftype, seq, payload, {"fault": "drop_reply", "sent": False})
                return
            if rule["kind"] == "corrupt_reply":
                frame = frame[:-1] + bytes([frame[-1] ^ 0xFF])
            elif rule["kind"] == "delay_reply":
                when += float(rule.get("s", 1.0))
            else:
                frame = build(ftype, seq, payload[:-1])
                payload = payload[:-1]
        self._log("tx", ftype, seq, payload, fields)
        if self.noise is not None:
            junk = bytes(self.noise.choice([b for b in range(256) if b != SOF0])
                         for _ in range(self.noise.randrange(1, 9)))
            self._queue(when, junk)
        gap = float(self.faults.get("bytewise", 0.0))
        if gap > 0:
            for i, b in enumerate(frame):
                self._queue(when + i * gap, bytes([b]))
        else:
            self._queue(when, frame)

    def _queue(self, when: float, data: bytes) -> None:
        """In time order, so a delayed reply does not hold the others back."""
        i = len(self.tx_queue)
        while i > 0 and self.tx_queue[i - 1][0] > when:
            i -= 1
        self.tx_queue.insert(i, (when, data))

    def bl_boot(self, now: float) -> None:
        requested = self.mailbox == BOOT_REQUEST_MAGIC  # take_boot_request()
        self.mailbox = 0
        self.lib.sim_boot(1 if requested else 0)
        self.boot_at = now
        self.build = None
        self.parser = Parser()
        flags = self.lib.sim_session()
        self._set_mode("bl", now, entered_by_request=bool(flags & 0x01), app_valid=bool(flags & 0x02),
                       stay=bool(flags & 0x04))

    def _feed(self, byte: int, now: float) -> None:
        self.busy_s = 0.0
        self.feed_now = now
        self.lib.BlProtocol_FeedByte(byte)
        if self.busy_s > 0:
            self.busy_until = now + self.busy_s

    def bl_rx(self, data: bytes, now: float) -> None:
        for b in data:
            if now < self.busy_until:
                # USART1 holds one byte in DR; the next sets ORE, and
                # uart_poll() then reads DR away: two or more are all lost
                self.busy_bytes += 1
                self.held_byte = b if self.busy_bytes == 1 else None
                continue
            self._flush_busy(now)
            self._feed(b, now)
        # power loss / host kill once the bootloader has taken the frame
        actions, self.after_feed = self.after_feed, []
        for action in actions:
            action()

    def _flush_busy(self, now: float) -> None:
        if self.busy_bytes == 1 and self.held_byte is not None:
            held, self.held_byte, self.busy_bytes = self.held_byte, None, 0
            self._feed(held, now)
        elif self.busy_bytes > 1:
            self.overruns += 1
            self._event("overrun", dropped=self.busy_bytes)
            self.busy_bytes, self.held_byte = 0, None

    def bl_tick(self, now: float) -> None:
        if now >= self.busy_until:
            self._flush_busy(now)
        flags = self.lib.sim_session()
        if self.due is not None:
            return
        if flags & 0x08:  # run_app: jump once the RUN_APP ack is out
            self.due = (max([now] + [w for w, _ in self.tx_queue]) + BYTE_S, "jump")
        elif not flags & 0x04 and now - self.boot_at >= GRACE_S:
            self.due = (now, "jump")

    # -- the application ---------------------------------------------------
    def app_start(self, now: float) -> None:
        region = self.flash_bytes()[APP_OFFSET:APP_OFFSET + APP_MAX_SIZE]
        self.build = None
        for b in self.catalog:
            size = int(b["size"])
            if 0 < size <= APP_MAX_SIZE and zlib.crc32(region[:size]) & 0xFFFFFFFF == int(b["crc32"], 16):
                self.build = b
                break
        self.parser = Parser()
        if self.build is None:
            self._set_mode("broken", now)
            return
        self.left = Wheel(self.left.counts_per_rev, self.left.max_rpm)
        self.right = Wheel(self.right.counts_per_rev, self.right.max_rpm)
        self.cmd_permille = (0, 0)
        self.last_command_at = None
        self.next_status = now
        self.next_slow = now
        self.last_step = now
        self._set_mode("app", now, build=self.build["_name"])
        # MotorTask sends the first 0x87 right away (last_info_tick - PERIOD)
        self.send_firmware_info()

    def send_firmware_info(self) -> None:
        if self.mode != "app" or self.build is None or self.build["fake"].get("no_0x87"):
            return
        b = self.build
        major, minor, patch = (int(x) for x in b["semver"])
        flags = (0x01 if b.get("dirty") else 0) | (0x02 if b["fake"].get("unversioned") else 0)
        payload = struct.pack("<BBBBIIBBH", major, minor, patch, VERSION, int(b["git_sha32"]),
                              int(b["build_unix"]), flags, 0, 0)
        self._send(FIRMWARE_INFO, payload, {"build": b["_name"]})

    def app_frame(self, frame_type: int, seq: int, payload: bytes, now: float) -> None:
        if frame_type != ENTER_BOOTLOADER:
            self.on_frame(frame_type, seq, payload, now)
            return
        self._log("rx", frame_type, seq, payload, {"mode": "app"})
        fake = self.build["fake"]
        if len(payload) != 8 or struct.unpack("<I", payload[:4])[0] != BOOT_REQUEST_MAGIC:
            return
        if fake.get("ignore_enter"):
            self._event("ignored_enter")
            return
        if not fake.get("no_enter_ack"):
            # enter_bootloader(rx_seq): the 0x8F carries the 0x0F's seq
            self._send(ENTER_BOOTLOADER_ACK, bytes(4), {"to_seq": seq})
        self.mailbox = BOOT_REQUEST_MAGIC
        self._set_mode("resetting", now)
        self.due = (now + self.RESET_S, "reset")

    def _send(self, frame_type: int, payload: bytes, fields=None) -> None:
        if frame_type == ENTER_BOOTLOADER_ACK and fields and "to_seq" in fields:
            seq = fields["to_seq"]
            self._write(build(frame_type, seq, payload))
            self._log("tx", frame_type, seq, payload, {})
            return
        super()._send(frame_type, payload, fields)

    def app_tick(self, now: float) -> None:
        if self.last_command_at is not None and now - self.last_command_at > self.command_timeout_s:
            self.left.command(0)
            self.right.command(0)
        if now >= self.next_status:
            dt = now - self.last_step
            self.last_step = now
            self.left.step(dt)
            self.right.step(dt)
            self.send_wheel_feedback()
            self.send_motor_status(now)
            self.next_status += self.STATUS_PERIOD_S
            if self.next_status < now:
                self.next_status = now + self.STATUS_PERIOD_S
        if now >= self.next_slow:
            self.send_power_status()
            self.send_charger_status()
            self.send_blade_status(now)
            self.send_servo_status()
            self.send_led_status()
            self.send_pid_status()
            self.send_firmware_info()
            self.next_slow = now + 1.0

    # -- host-side hooks ---------------------------------------------------
    def _host_frame(self, frame_type: int, now: float) -> None:
        if getattr(self, "_slave", None) is not None:
            # whether the host holds the tty exclusively (TIOCEXCL)
            try:
                excl = bool(struct.unpack("i", fcntl.ioctl(self._slave, TIOCGEXCL, b"\0" * 4))[0])
            except OSError:
                excl = None
            if excl != self.host_excl:
                self.host_excl = excl
                self._event("host_exclusive", exclusive=excl)
        if frame_type != BL_WRITE or self.mode != "bl":
            return
        self.writes_seen += 1
        n = self.writes_seen
        if self.faults.get("power_off_at_write") == n:
            self.after_feed.append(self._power_off)
        if self.faults.get("kill_host_at_write") == n:
            self.after_feed.append(self._kill_host)

    def _power_off(self) -> None:
        self._event("power_off", writes=self.writes_seen)
        self.save_flash()
        self.log.close()
        os._exit(3)

    def _kill_host(self) -> None:
        with open(self.host_pid_file) as f:
            pid = int(f.read().strip())
        os.kill(pid, signal.SIGKILL)
        self._event("host_killed", pid=pid, writes=self.writes_seen)
        if getattr(self, "_slave", None) is not None and not self.faults.get("keep_excl"):
            try:
                fcntl.ioctl(self._slave, TIOCNXCL)
            except OSError:
                pass

    # -- main loop ---------------------------------------------------------
    def run(self, duration: float) -> None:
        now = time.monotonic()
        end = now + duration
        if self.faults.get("dead"):
            self._set_mode("dead", now)
        else:
            self.bl_boot(now)  # power-on: the mailbox holds no request
        while not self.stop and time.monotonic() < end:
            try:
                data = os.read(self.fd, 4096)
            except (BlockingIOError, OSError):
                data = b""
            now = time.monotonic()
            frames = self.rx_log.feed(data) if data else []
            for frame_type, seq, payload in frames:
                self._host_frame(frame_type, now)
            if self.mode == "bl":
                for frame_type, seq, payload in frames:
                    self._log("rx", frame_type, seq, payload, {"mode": "bl"})
                self.bl_rx(data, now)
            elif self.mode == "app":
                for frame_type, seq, payload in self.parser.feed(data):
                    if self.mode == "app":
                        self.app_frame(frame_type, seq, payload, now)
                    else:
                        self._log("rx", frame_type, seq, payload, {"mode": self.mode})
            else:
                for frame_type, seq, payload in frames:
                    self._log("rx", frame_type, seq, payload, {"mode": self.mode})
            while self.tx_queue and self.tx_queue[0][0] <= now:
                self._write(self.tx_queue.pop(0)[1])
            if self.due is not None and now >= self.due[0]:
                what = self.due[1]
                self.due = None
                if what == "reset":
                    self._event("reset")
                    self.bl_boot(now)
                elif what == "jump":
                    self._event("jump")
                    self._set_mode("starting", now)
                    self.due = (now + self.APP_BOOT_S, "app")
                elif what == "app":
                    self.app_start(now)
            if self.mode == "bl":
                self.bl_tick(now)
            elif self.mode == "app":
                self.app_tick(now)
            time.sleep(0.001)
        # what the host wrote just before it exited is still in the pty:
        # log it, so the transcript does not depend on how fast it exits
        while True:
            try:
                data = os.read(self.fd, 4096)
            except (BlockingIOError, OSError):
                break
            if not data:
                break
            for frame_type, seq, payload in self.rx_log.feed(data):
                self._log("rx", frame_type, seq, payload, {"mode": self.mode, "at_stop": True})
        self._event("stop", overruns=self.overruns)
        self.save_flash()
        self.log.close()
        os.close(self.fd)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", help="pty the driver is NOT using")
    ap.add_argument("--pty-link", default="",
                    help="make the pty pair here instead and link the driver's end to this path")
    ap.add_argument("--mute-file", default="",
                    help="while this file exists: send nothing, drop everything received")
    ap.add_argument("--deaf-file", default="",
                    help="while this file exists: drop everything received, keep sending")
    ap.add_argument("--flash-stall-s", type=float, default=1.0,
                    help="how long a 0x04 with persist_to_flash stalls the board")
    ap.add_argument("--log", required=True, help="JSONL frame log")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--max-rpm", type=float, default=58.0)
    ap.add_argument("--counts-per-rev", type=float, default=8896.0)
    ap.add_argument("--bootloader-lib", default="",
                    help="the whole board with its UART bootloader: bl_protocol.c + bl_sim/sim_flash.c "
                         "compiled into a shared library (tools/flash_check.py builds it)")
    ap.add_argument("--flash-file", default="",
                    help="with --bootloader-lib: the 512 KB flash, read at start and written at exit")
    ap.add_argument("--catalog", default="",
                    help="with --bootloader-lib: directory of fw_manifest.py manifests the app can be")
    ap.add_argument("--faults", default="{}",
                    help="with --bootloader-lib: JSON object of faults (FakeStm32.__init__)")
    ap.add_argument("--host-pid-file", default="",
                    help="with --bootloader-lib: pid of the flasher, for the kill_host_at_write fault")
    args = ap.parse_args()
    if not args.port and not args.pty_link:
        ap.error("one of --port / --pty-link is required")
    if args.bootloader_lib:
        FakeStm32(args.port, args.log, args.max_rpm, args.counts_per_rev, args.pty_link,
                  args.bootloader_lib, args.flash_file, args.catalog, json.loads(args.faults),
                  args.host_pid_file).run(args.seconds)
        return 0
    FakeBase(args.port, args.log, args.max_rpm, args.counts_per_rev,
             args.pty_link, args.mute_file, args.deaf_file,
             args.flash_stall_s).run(args.seconds)
    return 0


if __name__ == "__main__":
    sys.exit(main())
