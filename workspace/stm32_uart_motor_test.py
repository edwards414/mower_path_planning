#!/usr/bin/env python3
"""Standalone UART motor command tester for the STM32 open-loop protocol.

Requires:
    pip install pyserial

Example:
    python workspace/stm32_uart_motor_test.py --port COM5 --left 300 --right 300
    python workspace/stm32_uart_motor_test.py --port /dev/ttyACM0 --left 250 --right -250
    python workspace/stm32_uart_motor_test.py --list-ports
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - depends on local environment.
    raise SystemExit(
        "pyserial is required. Install it with: pip install pyserial"
    ) from exc


SOF0 = 0xA5
SOF1 = 0x5A
PROTOCOL_VERSION = 0x01
FRAME_TYPE_MOTOR_COMMAND = 0x01
FRAME_TYPE_MOTOR_STATUS = 0x81
MOTOR_COMMAND_PAYLOAD_SIZE = 8
MOTOR_STATUS_PAYLOAD_SIZE = 12
MIN_FRAME_SIZE = 8
COMMAND_VALID_MASK = 0x01
COMMAND_TIMEOUT_MASK = 0x02
DRIVER_ALARM_MASK = 0x04
DEFAULT_BAUD = 115200
DEFAULT_COMMAND_TIMEOUT_MS = 200


@dataclass
class Frame:
    frame_type: int
    seq: int
    payload: bytes


@dataclass
class MotorStatus:
    seq: int
    commanded_left_permille: int
    commanded_right_permille: int
    applied_left_pwm: int
    applied_right_pwm: int
    command_age_ms: int
    flags: int
    last_rx_seq: int


def clamp_permille(value: int) -> int:
    return max(-1000, min(1000, value))


def compute_crc_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_motor_command_frame(
    seq: int,
    left_permille: int,
    right_permille: int,
    command_timeout_ms: int,
) -> bytes:
    payload = struct.pack(
        "<hhHH",
        clamp_permille(left_permille),
        clamp_permille(right_permille),
        max(0, min(0xFFFF, command_timeout_ms)),
        0,
    )
    if len(payload) != MOTOR_COMMAND_PAYLOAD_SIZE:
        raise ValueError("unexpected motor command payload size")

    frame_wo_crc = bytearray(
        [
            SOF0,
            SOF1,
            PROTOCOL_VERSION,
            FRAME_TYPE_MOTOR_COMMAND,
            seq & 0xFF,
            len(payload),
        ]
    )
    frame_wo_crc.extend(payload)

    crc = compute_crc_ccitt_false(bytes(frame_wo_crc[2:]))
    frame_wo_crc.extend(struct.pack("<H", crc))
    return bytes(frame_wo_crc)


def decode_motor_status(seq: int, payload: bytes) -> MotorStatus:
    if len(payload) != MOTOR_STATUS_PAYLOAD_SIZE:
        raise ValueError(
            f"unexpected 0x81 payload size: {len(payload)} "
            f"(expected {MOTOR_STATUS_PAYLOAD_SIZE})"
        )

    (
        commanded_left_permille,
        commanded_right_permille,
        applied_left_pwm,
        applied_right_pwm,
        command_age_ms,
        flags,
        last_rx_seq,
    ) = struct.unpack("<hhhhHBB", payload)

    return MotorStatus(
        seq=seq,
        commanded_left_permille=commanded_left_permille,
        commanded_right_permille=commanded_right_permille,
        applied_left_pwm=applied_left_pwm,
        applied_right_pwm=applied_right_pwm,
        command_age_ms=command_age_ms,
        flags=flags,
        last_rx_seq=last_rx_seq,
    )


def decode_flags(flags: int) -> List[str]:
    names: List[str] = []
    if flags & COMMAND_VALID_MASK:
        names.append("COMMAND_VALID")
    if flags & COMMAND_TIMEOUT_MASK:
        names.append("COMMAND_TIMEOUT")
    if flags & DRIVER_ALARM_MASK:
        names.append("DRIVER_ALARM")
    if not names:
        names.append("NONE")
    return names


class FrameParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> None:
        self._buffer.extend(data)

    def pop_frames(self) -> List[Frame]:
        frames: List[Frame] = []

        while len(self._buffer) >= MIN_FRAME_SIZE:
            sof_index = self._find_sof()
            if sof_index < 0:
                if self._buffer and self._buffer[-1] == SOF0:
                    self._buffer[:] = self._buffer[-1:]
                else:
                    self._buffer.clear()
                break

            if sof_index > 0:
                del self._buffer[:sof_index]

            if len(self._buffer) < MIN_FRAME_SIZE:
                break

            payload_len = self._buffer[5]
            frame_len = MIN_FRAME_SIZE + payload_len
            if len(self._buffer) < frame_len:
                break

            version = self._buffer[2]
            frame_type = self._buffer[3]
            seq = self._buffer[4]
            payload = bytes(self._buffer[6 : 6 + payload_len])
            received_crc = struct.unpack(
                "<H", self._buffer[6 + payload_len : 8 + payload_len]
            )[0]
            expected_crc = compute_crc_ccitt_false(
                bytes(self._buffer[2 : 6 + payload_len])
            )

            if version != PROTOCOL_VERSION or received_crc != expected_crc:
                del self._buffer[0]
                continue

            frames.append(Frame(frame_type=frame_type, seq=seq, payload=payload))
            del self._buffer[:frame_len]

        return frames

    def _find_sof(self) -> int:
        for index in range(len(self._buffer) - 1):
            if self._buffer[index] == SOF0 and self._buffer[index + 1] == SOF1:
                return index
        return -1


class Stm32MotorTester:
    def __init__(self, port: str, baudrate: int, read_timeout: float) -> None:
        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=read_timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self._parser = FrameParser()
        # Start from a changing sequence value to reduce false positives from
        # stale status frames left over from an earlier test session.
        self._next_seq = int(time.monotonic_ns()) & 0xFF

    def close(self) -> None:
        if self._serial.is_open:
            self._serial.close()

    def reset_buffers(self) -> None:
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

    def send_motor_command(
        self, left_permille: int, right_permille: int, command_timeout_ms: int
    ) -> int:
        seq = self._next_seq
        frame = build_motor_command_frame(
            seq=seq,
            left_permille=left_permille,
            right_permille=right_permille,
            command_timeout_ms=command_timeout_ms,
        )
        self._serial.write(frame)
        self._serial.flush()
        self._next_seq = (self._next_seq + 1) & 0xFF
        return seq

    def read_frames(self) -> List[Frame]:
        waiting = self._serial.in_waiting
        if waiting:
            self._parser.feed(self._serial.read(waiting))
        else:
            chunk = self._serial.read(1)
            if chunk:
                self._parser.feed(chunk)
        return self._parser.pop_frames()


def format_status(status: MotorStatus, latest_seq: Optional[int]) -> str:
    ack = "yes" if latest_seq is not None and status.last_rx_seq == latest_seq else "no"
    flags_text = ",".join(decode_flags(status.flags))
    return (
        f"RX status_seq={status.seq:3d} last_rx_seq={status.last_rx_seq:3d} "
        f"ack_latest={ack} flags={flags_text} age={status.command_age_ms:4d}ms "
        f"cmd=({status.commanded_left_permille:5d},{status.commanded_right_permille:5d}) "
        f"pwm=({status.applied_left_pwm:5d},{status.applied_right_pwm:5d})"
    )


def list_available_ports() -> int:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return 1

    print("Available serial ports:")
    for port in ports:
        description = port.description or "Unknown device"
        hwid = port.hwid or "Unknown HWID"
        print(f"  {port.device}  {description}  [{hwid}]")
    return 0


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be > 0")
    return parsed


def non_negative_float(value: str) -> float:
    parsed = float(value)
    if parsed < 0.0:
        raise argparse.ArgumentTypeError("value must be >= 0")
    return parsed


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Send STM32 open-loop motor commands over UART and check whether "
            "the MCU acknowledges them via 0x81 motor status."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--port", help="Serial port name, for example COM5 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="UART baud rate")
    parser.add_argument("--left", type=int, default=0, help="Left motor command in permille")
    parser.add_argument("--right", type=int, default=0, help="Right motor command in permille")
    parser.add_argument(
        "--duration",
        type=positive_float,
        default=2.0,
        help="How long to keep sending the command",
    )
    parser.add_argument(
        "--rate",
        type=positive_float,
        default=20.0,
        help="Command resend rate in Hz",
    )
    parser.add_argument(
        "--command-timeout-ms",
        type=int,
        default=DEFAULT_COMMAND_TIMEOUT_MS,
        help="Timeout field embedded in each command frame",
    )
    parser.add_argument(
        "--read-timeout",
        type=positive_float,
        default=0.02,
        help="Serial read timeout in seconds",
    )
    parser.add_argument(
        "--post-stop-monitor",
        type=non_negative_float,
        default=0.5,
        help="Monitor status for a little longer after sending stop",
    )
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="Print detected serial ports and exit",
    )
    return parser


def drain_frames(
    tester: Stm32MotorTester,
    active_sequences: Dict[int, float],
    latest_seq: Optional[int],
    accepted_sequences: set[int],
    saw_nonzero_pwm: List[bool],
    saw_timeout: List[bool],
    saw_driver_alarm: List[bool],
    max_wait_s: float = 0.02,
) -> None:
    read_deadline = time.monotonic() + max_wait_s
    while time.monotonic() < read_deadline:
        saw_frame = False
        for frame in tester.read_frames():
            saw_frame = True
            if frame.frame_type != FRAME_TYPE_MOTOR_STATUS:
                continue

            status = decode_motor_status(frame.seq, frame.payload)
            print(format_status(status, latest_seq))

            if status.last_rx_seq in active_sequences:
                accepted_sequences.add(status.last_rx_seq)

            if status.applied_left_pwm != 0 or status.applied_right_pwm != 0:
                saw_nonzero_pwm[0] = True

            if status.flags & COMMAND_TIMEOUT_MASK:
                saw_timeout[0] = True

            if status.flags & DRIVER_ALARM_MASK:
                saw_driver_alarm[0] = True

        if not saw_frame:
            break


def run_test(args: argparse.Namespace) -> int:
    tester = Stm32MotorTester(
        port=args.port,
        baudrate=args.baud,
        read_timeout=args.read_timeout,
    )

    accepted_sequences: set[int] = set()
    active_sequences: Dict[int, float] = {}
    saw_nonzero_pwm = [False]
    saw_timeout = [False]
    saw_driver_alarm = [False]
    latest_seq: Optional[int] = None

    try:
        tester.reset_buffers()
        print(
            f"Opened {args.port} at {args.baud} baud. "
            f"Sending left={clamp_permille(args.left)} right={clamp_permille(args.right)}."
        )

        send_period = 1.0 / args.rate
        next_send_time = time.monotonic()
        stop_time = next_send_time + args.duration

        while time.monotonic() < stop_time:
            now = time.monotonic()
            if now >= next_send_time:
                latest_seq = tester.send_motor_command(
                    left_permille=args.left,
                    right_permille=args.right,
                    command_timeout_ms=args.command_timeout_ms,
                )
                active_sequences[latest_seq] = time.monotonic()
                print(
                    f"TX seq={latest_seq:3d} left={clamp_permille(args.left):5d} "
                    f"right={clamp_permille(args.right):5d} "
                    f"timeout_ms={args.command_timeout_ms}"
                )
                next_send_time += send_period

            drain_frames(
                tester=tester,
                active_sequences=active_sequences,
                latest_seq=latest_seq,
                accepted_sequences=accepted_sequences,
                saw_nonzero_pwm=saw_nonzero_pwm,
                saw_timeout=saw_timeout,
                saw_driver_alarm=saw_driver_alarm,
                max_wait_s=min(send_period, 0.02),
            )

        print("Sending stop command...")
        stop_seq = tester.send_motor_command(
            left_permille=0,
            right_permille=0,
            command_timeout_ms=args.command_timeout_ms,
        )
        active_sequences[stop_seq] = time.monotonic()
        latest_seq = stop_seq

        post_stop_deadline = time.monotonic() + args.post_stop_monitor
        while time.monotonic() < post_stop_deadline:
            drain_frames(
                tester=tester,
                active_sequences=active_sequences,
                latest_seq=latest_seq,
                accepted_sequences=accepted_sequences,
                saw_nonzero_pwm=saw_nonzero_pwm,
                saw_timeout=saw_timeout,
                saw_driver_alarm=saw_driver_alarm,
                max_wait_s=0.02,
            )

    except KeyboardInterrupt:
        print("\nInterrupted by user. Sending emergency stop...")
        try:
            tester.send_motor_command(0, 0, args.command_timeout_ms)
        except Exception:
            pass
    finally:
        tester.close()

    accepted = bool(accepted_sequences)
    print("")
    print("Summary:")
    print(f"  STM32 accepted at least one command: {'YES' if accepted else 'NO'}")
    if latest_seq is not None:
        latest_accepted = latest_seq in accepted_sequences
        print(f"  Latest sequence acknowledged: {'YES' if latest_accepted else 'NO'}")
    print(f"  Non-zero applied PWM observed: {'YES' if saw_nonzero_pwm[0] else 'NO'}")
    print(f"  Timeout flag observed: {'YES' if saw_timeout[0] else 'NO'}")
    print(f"  Driver alarm observed: {'YES' if saw_driver_alarm[0] else 'NO'}")

    if not accepted:
        print("Result: no matching 0x81 status acknowledged the commands sent by this tool.")
        return 1

    print("Result: STM32 reported receiving commands from this tool.")
    return 0


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)

    if args.list_ports:
        return list_available_ports()

    if not args.port:
        parser.error("--port is required unless --list-ports is used")

    try:
        return run_test(args)
    except serial.SerialException as exc:
        print(f"Failed to open or use serial port {args.port}: {exc}")
        return 2


if __name__ == "__main__":
    sys.exit(main())
