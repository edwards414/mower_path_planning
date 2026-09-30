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

Nothing here is used on the robot; it exists only for the test.
"""

from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import time
import tty

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
    args = ap.parse_args()
    if not args.port and not args.pty_link:
        ap.error("one of --port / --pty-link is required")
    FakeBase(args.port, args.log, args.max_rpm, args.counts_per_rev,
             args.pty_link, args.mute_file, args.deaf_file,
             args.flash_stall_s).run(args.seconds)
    return 0


if __name__ == "__main__":
    sys.exit(main())
