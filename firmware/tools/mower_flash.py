#!/usr/bin/env python3
"""Flash the mower STM32F411 application over the host UART.

Talks to the UART bootloader in bootloader/ (see BOOTLOADER.md). The board
can be in either state when you start: if the application is running, the
tool asks it to reboot into the bootloader first (frame 0x0F).

    python3 tools/mower_flash.py -p /dev/tty.usbserial-XXXX flash Debug/mower_robot_firmware.bin
    python3 tools/mower_flash.py -p /dev/ttyUSB0 info
    python3 tools/mower_flash.py -p /dev/ttyUSB0 enter     # reboot into bootloader and stay
    python3 tools/mower_flash.py -p /dev/ttyUSB0 run       # jump to the application

Requires pyserial. Stop anything else using the port (the ROS2 node) first.
"""

import argparse
import struct
import sys
import time
import zlib

try:
    import serial
except ImportError:  # pragma: no cover
    sys.exit("pyserial is required: pip install pyserial")

# --- protocol constants (mirror Module/Inc/boot_shared.h) -------------------
SOF0, SOF1, VERSION = 0xA5, 0x5A, 0x01

T_ENTER_BOOTLOADER = 0x0F
T_ENTER_BOOTLOADER_ACK = 0x8F
T_BL_PING = 0x10
T_BL_ERASE = 0x11
T_BL_WRITE = 0x12
T_BL_VERIFY = 0x13
T_BL_RUN_APP = 0x14
T_BL_INFO = 0x90
T_BL_ACK = 0x91

BOOT_REQUEST_MAGIC = 0xB007B007
APP_START_ADDRESS = 0x08008000
APP_MAX_SIZE = 0x08060000 - APP_START_ADDRESS
WRITE_CHUNK_MAX = 128

STATUS_NAMES = {
    0: "OK",
    1: "BAD_ARGUMENT",
    2: "FLASH_ERROR",
    3: "CRC_MISMATCH",
    4: "OUT_OF_RANGE",
    5: "NOT_ERASED",
    6: "NO_VALID_APP",
}

INFO_FLAG_APP_VALID = 0x01
INFO_FLAG_ENTERED_BY_REQUEST = 0x02
INFO_FLAG_ERASED = 0x04


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_frame(ftype: int, seq: int, payload: bytes = b"") -> bytes:
    header = bytes([VERSION, ftype, seq & 0xFF, len(payload)])
    crc = crc16_ccitt(header + payload)
    return bytes([SOF0, SOF1]) + header + payload + bytes([crc & 0xFF, crc >> 8])


class FrameReader:
    """Incremental parser; returns (type, seq, payload) for CRC-valid frames."""

    def __init__(self, port: serial.Serial):
        self.port = port
        self.buf = bytearray()

    def _pop_frame(self):
        while True:
            start = self.buf.find(bytes([SOF0, SOF1]))
            if start < 0:
                # keep a trailing SOF0 in case SOF1 is still in flight
                if self.buf and self.buf[-1] == SOF0:
                    del self.buf[:-1]
                else:
                    self.buf.clear()
                return None
            if start > 0:
                del self.buf[:start]
            if len(self.buf) < 6:
                return None
            plen = self.buf[5]
            total = 8 + plen
            if len(self.buf) < total:
                return None
            frame = bytes(self.buf[:total])
            crc_rx = frame[6 + plen] | (frame[7 + plen] << 8)
            if crc_rx == crc16_ccitt(frame[2 : 6 + plen]) and frame[2] == VERSION:
                del self.buf[:total]
                return frame[3], frame[4], frame[6 : 6 + plen]
            # bad CRC: skip this SOF and resync
            del self.buf[:1]

    def read(self, want_type, timeout: float):
        """Wait up to `timeout` s for a frame of type `want_type` (int or set)."""
        if isinstance(want_type, int):
            want_type = {want_type}
        deadline = time.monotonic() + timeout
        while True:
            frame = self._pop_frame()
            while frame is not None:
                if frame[0] in want_type:
                    return frame
                frame = self._pop_frame()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self.port.timeout = min(remaining, 0.05)
            chunk = self.port.read(256)
            if chunk:
                self.buf.extend(chunk)


class Bootloader:
    def __init__(self, port: serial.Serial):
        self.port = port
        self.reader = FrameReader(port)
        self.seq = 0

    def _next_seq(self) -> int:
        self.seq = (self.seq + 1) & 0xFF
        return self.seq

    def send(self, ftype: int, payload: bytes = b"") -> int:
        seq = self._next_seq()
        self.port.write(build_frame(ftype, seq, payload))
        return seq

    def ping(self, timeout: float = 0.3):
        seq = self.send(T_BL_PING)
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            frame = self.reader.read(T_BL_INFO, remaining)
            if frame is None:
                return None
            if frame[1] == seq:
                return parse_info(frame[2])

    def command(self, ftype: int, payload: bytes, timeout: float):
        """Send a command and wait for its BL_ACK. Returns (status, value)."""
        seq = self.send(ftype, payload)
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"no ack for command 0x{ftype:02X}")
            frame = self.reader.read(T_BL_ACK, remaining)
            if frame is None:
                raise TimeoutError(f"no ack for command 0x{ftype:02X}")
            cmd, status, value = struct.unpack("<BBI", frame[2])
            if cmd == ftype and frame[1] == seq:
                return status, value

    def request_enter_from_app(self):
        payload = struct.pack("<II", BOOT_REQUEST_MAGIC, 0)
        self.send(T_ENTER_BOOTLOADER, payload)
        frame = self.reader.read(T_ENTER_BOOTLOADER_ACK, 0.5)
        return frame is not None

    def ensure_bootloader(self, attempts: int = 25):
        """Return BL info; reboot the app into the bootloader if needed."""
        info = self.ping()
        if info is not None:
            return info
        print("application is running, asking it to reboot into the bootloader")
        acked = self.request_enter_from_app()
        print("  app acknowledged" if acked else "  no ack from app (continuing anyway)")
        time.sleep(0.2)
        for _ in range(attempts):
            info = self.ping(0.2)
            if info is not None:
                return info
        raise RuntimeError(
            "bootloader did not answer. Is the port right, is the bootloader "
            "flashed at 0x08000000, and is nothing else using the port?"
        )


def parse_info(payload: bytes) -> dict:
    app_start, app_max, chunk, version, flags, _ = struct.unpack("<IIHBBI", payload)
    return {
        "app_start": app_start,
        "app_max_size": app_max,
        "write_chunk_max": chunk,
        "version": version,
        "flags": flags,
        "app_valid": bool(flags & INFO_FLAG_APP_VALID),
        "entered_by_request": bool(flags & INFO_FLAG_ENTERED_BY_REQUEST),
        "erased": bool(flags & INFO_FLAG_ERASED),
    }


def print_info(info: dict) -> None:
    print(
        f"bootloader v{info['version']}: app @ 0x{info['app_start']:08X}, "
        f"max {info['app_max_size'] // 1024} KB, chunk {info['write_chunk_max']} B, "
        f"app_valid={info['app_valid']} by_request={info['entered_by_request']} "
        f"erased={info['erased']}"
    )


def check(status: int, what: str, value: int = 0) -> None:
    if status != 0:
        raise RuntimeError(f"{what} failed: {STATUS_NAMES.get(status, status)} (value=0x{value:08X})")


def load_image(path: str) -> bytes:
    with open(path, "rb") as f:
        image = f.read()
    if path.lower().endswith((".hex", ".elf")):
        sys.exit("give the .bin (Debug/mower_robot_firmware.bin), not .hex/.elf")
    if len(image) == 0:
        sys.exit("image is empty")
    if len(image) > APP_MAX_SIZE:
        sys.exit(f"image is {len(image)} bytes, larger than the {APP_MAX_SIZE}-byte app region")
    sp, pc = struct.unpack("<II", image[:8])
    if not (0x20000000 < sp <= 0x20020000) or not (APP_START_ADDRESS <= pc < 0x08060000 and pc & 1):
        sys.exit(
            f"this does not look like an app linked at 0x{APP_START_ADDRESS:08X} "
            f"(SP=0x{sp:08X} PC=0x{pc:08X}); rebuild with the updated linker script"
        )
    if len(image) % 4:
        image += b"\xFF" * (4 - len(image) % 4)
    return image


def do_flash(bl: Bootloader, path: str, run_after: bool) -> None:
    image = load_image(path)
    crc = zlib.crc32(image) & 0xFFFFFFFF
    info = bl.ensure_bootloader()
    print_info(info)
    chunk_size = min(WRITE_CHUNK_MAX, info["write_chunk_max"]) & ~3

    print(f"erasing for {len(image)} bytes ...")
    status, value = bl.command(T_BL_ERASE, struct.pack("<I", len(image)), timeout=20.0)
    check(status, "erase", value)

    # Write the vector table (offset 0) last: an interrupted update leaves the
    # reset vector blank and the bootloader will refuse to jump into it.
    offsets = list(range(chunk_size, len(image), chunk_size)) + [0]
    t0 = time.monotonic()
    for n, off in enumerate(offsets, 1):
        data = image[off : off + chunk_size]
        payload = struct.pack("<I", off) + data
        for attempt in range(3):
            try:
                status, value = bl.command(T_BL_WRITE, payload, timeout=2.0)
                check(status, f"write @0x{off:X}", value)
                break
            except TimeoutError:
                if attempt == 2:
                    raise
                print(f"  retry write @0x{off:X}")
        if n % 32 == 0 or n == len(offsets):
            done = n * chunk_size
            pct = min(100, 100 * done // len(image))
            print(f"\r  {pct:3d}%  {min(done, len(image))}/{len(image)} bytes", end="", flush=True)
    print(f"\n  written in {time.monotonic() - t0:.1f} s")

    print("verifying ...")
    status, value = bl.command(T_BL_VERIFY, struct.pack("<II", len(image), crc), timeout=5.0)
    check(status, "verify", value)
    print(f"  crc32 0x{crc:08X} ok")

    if run_after:
        print("starting application")
        status, value = bl.command(T_BL_RUN_APP, b"", timeout=2.0)
        check(status, "run", value)
    else:
        print("staying in bootloader (use `run` to start the app)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", required=True, help="serial port, e.g. /dev/ttyUSB0")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("flash", help="write a .bin into the app region and start it")
    f.add_argument("image")
    f.add_argument("--no-run", action="store_true", help="stay in the bootloader afterwards")
    sub.add_parser("info", help="print bootloader info (reboots the app into it)")
    sub.add_parser("enter", help="reboot into the bootloader and stay there")
    sub.add_parser("run", help="jump from the bootloader to the application")
    args = ap.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.05) as port:
        port.reset_input_buffer()
        bl = Bootloader(port)
        try:
            if args.cmd == "flash":
                do_flash(bl, args.image, run_after=not args.no_run)
            elif args.cmd in ("info", "enter"):
                print_info(bl.ensure_bootloader())
            elif args.cmd == "run":
                status, value = bl.command(T_BL_RUN_APP, b"", timeout=2.0)
                check(status, "run", value)
                print("application started")
        except (RuntimeError, TimeoutError) as exc:
            sys.exit(f"error: {exc}")


if __name__ == "__main__":
    main()
