#!/usr/bin/env python3
"""PyQt5 monitor for the STM32 UART open-loop status protocol."""

from __future__ import annotations

import queue
import struct
import sys
import time
from collections import Counter
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Optional, Tuple

from PyQt5.QtCore import QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - depends on local environment.
    serial = None
    list_ports = None
    SERIAL_IMPORT_ERROR = exc
else:
    SERIAL_IMPORT_ERROR = None


SOF0 = 0xA5
SOF1 = 0x5A
PROTOCOL_VERSION = 0x01
FRAME_TYPE_MOTOR_COMMAND = 0x01
FRAME_TYPE_MOWER_COMMAND = 0x02
FRAME_TYPE_WS2812_COMMAND = 0x03
FRAME_TYPE_MOTOR_STATUS = 0x81
FRAME_TYPE_MOWER_STATUS = 0x82
FRAME_TYPE_WS2812_STATUS = 0x83
MOTOR_STATUS_PAYLOAD_SIZE = 12
MOWER_STATUS_PAYLOAD_SIZE = 8
WS2812_STATUS_PAYLOAD_SIZE = 8
MIN_FRAME_SIZE = 8
COMMAND_VALID_MASK = 0x01
COMMAND_TIMEOUT_MASK = 0x02
DRIVER_ALARM_MASK = 0x04
DEFAULT_BAUD = 115200
DEFAULT_COMMAND_TIMEOUT_MS = 200
DEFAULT_MOTOR_REPEAT_RATE_HZ = 20
DEFAULT_READ_TIMEOUT_S = 0.05
MAX_LOG_LINES = 300


@dataclass
class Frame:
    frame_type: int
    seq: int
    payload: bytes
    raw_bytes: bytes


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


@dataclass
class MowerMotorStatus:
    seq: int
    commanded_permille: int
    applied_pwm: int
    command_age_ms: int
    flags: int
    last_rx_seq: int


@dataclass
class Ws2812Status:
    seq: int
    mode: int
    red: int
    green: int
    blue: int
    effect_period_ms: int
    flags: int
    last_rx_seq: int


@dataclass
class TxFrame:
    frame_type: int
    seq: int
    raw_bytes: bytes
    description: str


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


def clamp_value(value: int, min_value: int, max_value: int) -> int:
    return max(min_value, min(max_value, value))


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


def flags_text_and_color(flags: int) -> Tuple[str, str]:
    if flags & DRIVER_ALARM_MASK:
        return ("DRIVER_ALARM", "#d32f2f")
    if flags & COMMAND_TIMEOUT_MASK:
        return ("COMMAND_TIMEOUT", "#ef6c00")
    if flags & COMMAND_VALID_MASK:
        return ("COMMAND_VALID", "#2e7d32")
    return ("NONE", "#616161")


def ws2812_mode_name(mode: int) -> str:
    return {
        0x00: "CLEAR",
        0x01: "ALL_ON",
        0x02: "FLOW",
        0x03: "TURN_LEFT",
        0x04: "TURN_RIGHT",
        0x05: "SHOW",
    }.get(mode, f"UNKNOWN(0x{mode:02X})")


def frame_type_name(frame_type: int) -> str:
    return {
        FRAME_TYPE_MOTOR_COMMAND: "0x01 MOTOR_CMD",
        FRAME_TYPE_MOWER_COMMAND: "0x02 MOWER_CMD",
        FRAME_TYPE_WS2812_COMMAND: "0x03 WS2812_CMD",
        FRAME_TYPE_MOTOR_STATUS: "0x81 MOTOR_STATUS",
        FRAME_TYPE_MOWER_STATUS: "0x82 MOWER_STATUS",
        FRAME_TYPE_WS2812_STATUS: "0x83 WS2812_STATUS",
    }.get(frame_type, f"0x{frame_type:02X}")


def bytes_to_hex(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def now_text() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def build_frame(frame_type: int, seq: int, payload: bytes) -> bytes:
    frame_wo_crc = bytearray(
        [
            SOF0,
            SOF1,
            PROTOCOL_VERSION,
            frame_type & 0xFF,
            seq & 0xFF,
            len(payload) & 0xFF,
        ]
    )
    frame_wo_crc.extend(payload)
    crc = compute_crc_ccitt_false(bytes(frame_wo_crc[2:]))
    frame_wo_crc.extend(struct.pack("<H", crc))
    return bytes(frame_wo_crc)


def build_motor_command_frame(
    seq: int,
    left_permille: int,
    right_permille: int,
    command_timeout_ms: int,
) -> bytes:
    payload = struct.pack(
        "<hhHH",
        clamp_value(left_permille, -1000, 1000),
        clamp_value(right_permille, -1000, 1000),
        clamp_value(command_timeout_ms, 0, 0xFFFF),
        0,
    )
    return build_frame(FRAME_TYPE_MOTOR_COMMAND, seq, payload)


def build_mower_command_frame(
    seq: int,
    command_permille: int,
    command_timeout_ms: int,
) -> bytes:
    payload = struct.pack(
        "<hHHH",
        clamp_value(command_permille, -1000, 1000),
        clamp_value(command_timeout_ms, 0, 0xFFFF),
        0,
        0,
    )
    return build_frame(FRAME_TYPE_MOWER_COMMAND, seq, payload)


def build_ws2812_command_frame(
    seq: int,
    mode: int,
    red: int,
    green: int,
    blue: int,
    effect_period_ms: int,
) -> bytes:
    payload = struct.pack(
        "<BBBBHBB",
        mode & 0xFF,
        clamp_value(red, 0, 255),
        clamp_value(green, 0, 255),
        clamp_value(blue, 0, 255),
        clamp_value(effect_period_ms, 0, 0xFFFF),
        0,
        0,
    )
    return build_frame(FRAME_TYPE_WS2812_COMMAND, seq, payload)


def decode_motor_status(seq: int, payload: bytes) -> MotorStatus:
    if len(payload) != MOTOR_STATUS_PAYLOAD_SIZE:
        raise ValueError(
            f"unexpected 0x81 payload size {len(payload)}, "
            f"expected {MOTOR_STATUS_PAYLOAD_SIZE}"
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


def decode_mower_status(seq: int, payload: bytes) -> MowerMotorStatus:
    if len(payload) != MOWER_STATUS_PAYLOAD_SIZE:
        raise ValueError(
            f"unexpected 0x82 payload size {len(payload)}, "
            f"expected {MOWER_STATUS_PAYLOAD_SIZE}"
        )

    (
        commanded_permille,
        applied_pwm,
        command_age_ms,
        flags,
        last_rx_seq,
    ) = struct.unpack("<hhHBB", payload)

    return MowerMotorStatus(
        seq=seq,
        commanded_permille=commanded_permille,
        applied_pwm=applied_pwm,
        command_age_ms=command_age_ms,
        flags=flags,
        last_rx_seq=last_rx_seq,
    )


def decode_ws2812_status(seq: int, payload: bytes) -> Ws2812Status:
    if len(payload) != WS2812_STATUS_PAYLOAD_SIZE:
        raise ValueError(
            f"unexpected 0x83 payload size {len(payload)}, "
            f"expected {WS2812_STATUS_PAYLOAD_SIZE}"
        )

    (
        mode,
        red,
        green,
        blue,
        effect_period_ms,
        flags,
        last_rx_seq,
    ) = struct.unpack("<BBBBHBB", payload)

    return Ws2812Status(
        seq=seq,
        mode=mode,
        red=red,
        green=green,
        blue=blue,
        effect_period_ms=effect_period_ms,
        flags=flags,
        last_rx_seq=last_rx_seq,
    )


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
            raw_frame = bytes(self._buffer[:frame_len])
            received_crc = struct.unpack(
                "<H", self._buffer[6 + payload_len : frame_len]
            )[0]
            expected_crc = compute_crc_ccitt_false(
                bytes(self._buffer[2 : 6 + payload_len])
            )

            if version != PROTOCOL_VERSION or received_crc != expected_crc:
                del self._buffer[0]
                continue

            frames.append(
                Frame(
                    frame_type=frame_type,
                    seq=seq,
                    payload=payload,
                    raw_bytes=raw_frame,
                )
            )
            del self._buffer[:frame_len]

        return frames

    def _find_sof(self) -> int:
        for index in range(len(self._buffer) - 1):
            if self._buffer[index] == SOF0 and self._buffer[index + 1] == SOF1:
                return index
        return -1


class SerialMonitorThread(QThread):
    connection_changed = pyqtSignal(bool, str)
    motor_status_received = pyqtSignal(object)
    mower_status_received = pyqtSignal(object)
    ws2812_status_received = pyqtSignal(object)
    frame_count_changed = pyqtSignal(dict)
    frame_activity_changed = pyqtSignal(str, str, int, str)
    tx_frame_sent = pyqtSignal(str, int, str)
    log_message = pyqtSignal(str, str)

    def __init__(self, port: str, baudrate: int, read_timeout_s: float) -> None:
        super().__init__()
        self._port = port
        self._baudrate = baudrate
        self._read_timeout_s = read_timeout_s
        self._running = True
        self._parser = FrameParser()
        self._counts: Counter[int] = Counter()
        self._tx_queue: "queue.Queue[TxFrame]" = queue.Queue()
        self._next_seq = int(time.monotonic_ns()) & 0xFF

    def stop(self) -> None:
        self._running = False

    def queue_motor_command(
        self,
        left_permille: int,
        right_permille: int,
        command_timeout_ms: int,
    ) -> int:
        seq = self._next_sequence()
        frame = build_motor_command_frame(
            seq=seq,
            left_permille=left_permille,
            right_permille=right_permille,
            command_timeout_ms=command_timeout_ms,
        )
        description = (
            f"left={clamp_value(left_permille, -1000, 1000)} "
            f"right={clamp_value(right_permille, -1000, 1000)} "
            f"timeout_ms={clamp_value(command_timeout_ms, 0, 0xFFFF)}"
        )
        self._tx_queue.put(
            TxFrame(
                frame_type=FRAME_TYPE_MOTOR_COMMAND,
                seq=seq,
                raw_bytes=frame,
                description=description,
            )
        )
        return seq

    def queue_mower_command(
        self,
        command_permille: int,
        command_timeout_ms: int,
    ) -> int:
        seq = self._next_sequence()
        frame = build_mower_command_frame(
            seq=seq,
            command_permille=command_permille,
            command_timeout_ms=command_timeout_ms,
        )
        description = (
            f"command={clamp_value(command_permille, -1000, 1000)} "
            f"timeout_ms={clamp_value(command_timeout_ms, 0, 0xFFFF)}"
        )
        self._tx_queue.put(
            TxFrame(
                frame_type=FRAME_TYPE_MOWER_COMMAND,
                seq=seq,
                raw_bytes=frame,
                description=description,
            )
        )
        return seq

    def queue_ws2812_command(
        self,
        mode: int,
        red: int,
        green: int,
        blue: int,
        effect_period_ms: int,
    ) -> int:
        seq = self._next_sequence()
        frame = build_ws2812_command_frame(
            seq=seq,
            mode=mode,
            red=red,
            green=green,
            blue=blue,
            effect_period_ms=effect_period_ms,
        )
        description = (
            f"mode={ws2812_mode_name(mode)} "
            f"rgb=({clamp_value(red, 0, 255)},{clamp_value(green, 0, 255)},"
            f"{clamp_value(blue, 0, 255)}) "
            f"period_ms={clamp_value(effect_period_ms, 0, 0xFFFF)}"
        )
        self._tx_queue.put(
            TxFrame(
                frame_type=FRAME_TYPE_WS2812_COMMAND,
                seq=seq,
                raw_bytes=frame,
                description=description,
            )
        )
        return seq

    def _next_sequence(self) -> int:
        seq = self._next_seq
        self._next_seq = (self._next_seq + 1) & 0xFF
        return seq

    def run(self) -> None:
        if serial is None:
            self.log_message.emit(
                "ERROR",
                "pyserial is not installed. Run: pip install pyserial",
            )
            self.connection_changed.emit(False, "pyserial missing")
            return

        serial_conn = None
        try:
            serial_conn = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=self._read_timeout_s,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            serial_conn.reset_input_buffer()
            self.connection_changed.emit(True, f"Connected to {self._port}")
            self.log_message.emit(
                "INFO",
                f"Opened {self._port} at {self._baudrate} baud",
            )

            while self._running or not self._tx_queue.empty():
                self._flush_tx_queue(serial_conn)
                if not self._running and self._tx_queue.empty():
                    break

                waiting = serial_conn.in_waiting
                if waiting:
                    data = serial_conn.read(waiting)
                else:
                    data = serial_conn.read(1)

                if not data:
                    continue

                self._parser.feed(data)
                for frame in self._parser.pop_frames():
                    self._handle_frame(frame)

        except Exception as exc:
            self.log_message.emit("ERROR", f"Serial monitor stopped: {exc}")
            self.connection_changed.emit(False, str(exc))
        finally:
            if serial_conn is not None and serial_conn.is_open:
                serial_conn.close()
            self.connection_changed.emit(False, "Disconnected")

    def _flush_tx_queue(self, serial_conn: "serial.Serial") -> None:
        while True:
            try:
                tx_frame = self._tx_queue.get_nowait()
            except queue.Empty:
                break

            serial_conn.write(tx_frame.raw_bytes)
            serial_conn.flush()
            self.tx_frame_sent.emit(
                frame_type_name(tx_frame.frame_type),
                tx_frame.seq,
                tx_frame.description,
            )

    def _handle_frame(self, frame: Frame) -> None:
        self._counts[frame.frame_type] += 1
        counts_dict = {f"0x{frame_type:02X}": count for frame_type, count in self._counts.items()}
        last_time_text = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.frame_count_changed.emit(counts_dict)
        self.frame_activity_changed.emit(
            last_time_text,
            f"0x{frame.frame_type:02X}",
            frame.seq,
            bytes_to_hex(frame.raw_bytes),
        )

        try:
            if frame.frame_type == FRAME_TYPE_MOTOR_STATUS:
                self.motor_status_received.emit(decode_motor_status(frame.seq, frame.payload))
            elif frame.frame_type == FRAME_TYPE_MOWER_STATUS:
                self.mower_status_received.emit(decode_mower_status(frame.seq, frame.payload))
            elif frame.frame_type == FRAME_TYPE_WS2812_STATUS:
                self.ws2812_status_received.emit(decode_ws2812_status(frame.seq, frame.payload))
            else:
                self.log_message.emit(
                    "WARNING",
                    f"Unknown frame type 0x{frame.frame_type:02X}: {bytes_to_hex(frame.raw_bytes)}",
                )
        except ValueError as exc:
            self.log_message.emit("WARNING", f"Frame parse warning: {exc}")


class ValuePanel(QGroupBox):
    def __init__(self, title: str, fields: List[Tuple[str, str]]) -> None:
        super().__init__(title)
        self.setFont(QFont("Arial", 10, QFont.Bold))
        self._labels: Dict[str, QLabel] = {}

        layout = QFormLayout()
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)
        self.setLayout(layout)

        mono_font = QFont("Consolas", 10)
        for key, label_text in fields:
            value_label = QLabel("-")
            value_label.setFont(mono_font)
            value_label.setTextInteractionFlags(value_label.textInteractionFlags())
            layout.addRow(QLabel(label_text), value_label)
            self._labels[key] = value_label

    def set_value(self, key: str, value: str, color: Optional[str] = None) -> None:
        label = self._labels[key]
        label.setText(value)
        if color:
            label.setStyleSheet(f"color: {color}; font-weight: 600;")
        else:
            label.setStyleSheet("")

    def set_all_defaults(self) -> None:
        for label in self._labels.values():
            label.setText("-")
            label.setStyleSheet("")


class Stm32UartMonitorWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("STM32 UART Status Monitor")
        self.setGeometry(120, 120, 1440, 920)

        self._monitor_thread: Optional[SerialMonitorThread] = None
        self._log_lines: List[str] = []
        self.motor_repeat_timer = QTimer(self)
        self.motor_repeat_timer.timeout.connect(self._send_motor_hold_tick)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)

        main_layout.addWidget(self._build_connection_group())
        main_layout.addWidget(self._build_summary_group())
        self.command_group = self._build_command_group()
        main_layout.addWidget(self.command_group)

        status_layout = QGridLayout()
        status_layout.setHorizontalSpacing(12)
        status_layout.setVerticalSpacing(12)
        main_layout.addLayout(status_layout, 1)

        self.motor_panel = ValuePanel(
            "0x81 Motor Status",
            [
                ("last_update", "Last Update"),
                ("status_seq", "Status Seq"),
                ("last_rx_seq", "Last RX Seq"),
                ("flags", "Flags"),
                ("command_age_ms", "Command Age (ms)"),
                ("commanded_left", "Cmd Left"),
                ("commanded_right", "Cmd Right"),
                ("applied_left_pwm", "Applied Left PWM"),
                ("applied_right_pwm", "Applied Right PWM"),
            ],
        )
        self.mower_panel = ValuePanel(
            "0x82 Mower Motor Status",
            [
                ("last_update", "Last Update"),
                ("status_seq", "Status Seq"),
                ("last_rx_seq", "Last RX Seq"),
                ("flags", "Flags"),
                ("command_age_ms", "Command Age (ms)"),
                ("commanded", "Cmd"),
                ("applied_pwm", "Applied PWM"),
            ],
        )
        self.ws2812_panel = ValuePanel(
            "0x83 WS2812 Status",
            [
                ("last_update", "Last Update"),
                ("status_seq", "Status Seq"),
                ("last_rx_seq", "Last RX Seq"),
                ("flags", "Flags"),
                ("mode", "Mode"),
                ("rgb", "RGB"),
                ("effect_period_ms", "Effect Period (ms)"),
            ],
        )
        self.log_panel = self._build_log_group()

        status_layout.addWidget(self.motor_panel, 0, 0, 2, 1)
        status_layout.addWidget(self.mower_panel, 0, 1)
        status_layout.addWidget(self.ws2812_panel, 1, 1)
        status_layout.addWidget(self.log_panel, 0, 2, 2, 1)
        status_layout.setColumnStretch(0, 2)
        status_layout.setColumnStretch(1, 2)
        status_layout.setColumnStretch(2, 3)

        self.refresh_ports()
        self._set_connected_state(False, "Disconnected")
        self._reset_summary()
        self.log_event("INFO", "Ready. Select a serial port and click Connect.")

    def _build_spinbox(
        self,
        min_value: int,
        max_value: int,
        value: int,
        step: int = 1,
    ) -> QSpinBox:
        spinbox = QSpinBox()
        spinbox.setRange(min_value, max_value)
        spinbox.setValue(value)
        spinbox.setSingleStep(step)
        spinbox.setFont(QFont("Consolas", 10))
        spinbox.setMinimumWidth(90)
        return spinbox

    def _build_connection_group(self) -> QGroupBox:
        group = QGroupBox("Connection")
        group.setFont(QFont("Arial", 10, QFont.Bold))

        layout = QHBoxLayout()
        group.setLayout(layout)

        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setMinimumWidth(240)

        self.refresh_button = QPushButton("Refresh Ports")
        self.refresh_button.clicked.connect(self.refresh_ports)

        self.baud_combo = QComboBox()
        self.baud_combo.setEditable(True)
        for baud in ["115200", "230400", "57600", "38400", "19200", "9600"]:
            self.baud_combo.addItem(baud)
        self.baud_combo.setCurrentText(str(DEFAULT_BAUD))
        self.baud_combo.setMinimumWidth(120)

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)

        self.connection_label = QLabel("Disconnected")
        self.connection_label.setFont(QFont("Arial", 10, QFont.Bold))

        layout.addWidget(QLabel("Port"))
        layout.addWidget(self.port_combo)
        layout.addWidget(self.refresh_button)
        layout.addSpacing(10)
        layout.addWidget(QLabel("Baud"))
        layout.addWidget(self.baud_combo)
        layout.addSpacing(10)
        layout.addWidget(self.connect_button)
        layout.addStretch()
        layout.addWidget(self.connection_label)

        return group

    def _build_summary_group(self) -> QGroupBox:
        group = QGroupBox("Frame Summary")
        group.setFont(QFont("Arial", 10, QFont.Bold))

        layout = QGridLayout()
        group.setLayout(layout)

        self.summary_labels: Dict[str, QLabel] = {}
        summary_fields = [
            ("last_frame_time", "Last Frame Time"),
            ("last_frame_type", "Last Frame Type"),
            ("last_frame_seq", "Last Frame Seq"),
            ("last_tx_time", "Last TX Time"),
            ("last_tx_type", "Last TX Type"),
            ("last_tx_seq", "Last TX Seq"),
            ("motor_frames", "0x81 Count"),
            ("mower_frames", "0x82 Count"),
            ("ws2812_frames", "0x83 Count"),
            ("unknown_frames", "Unknown Count"),
            ("last_raw_frame", "Last Raw Frame"),
        ]

        mono_font = QFont("Consolas", 10)
        for row, (key, label_text) in enumerate(summary_fields):
            title_label = QLabel(label_text)
            value_label = QLabel("-")
            value_label.setFont(mono_font)
            value_label.setWordWrap(True)
            if key == "last_raw_frame":
                value_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
            layout.addWidget(title_label, row, 0)
            layout.addWidget(value_label, row, 1)
            self.summary_labels[key] = value_label

        return group

    def _build_command_group(self) -> QGroupBox:
        group = QGroupBox("Command TX")
        group.setFont(QFont("Arial", 10, QFont.Bold))

        layout = QGridLayout()
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(12)
        group.setLayout(layout)

        layout.addWidget(self._build_motor_command_box(), 0, 0)
        layout.addWidget(self._build_mower_command_box(), 0, 1)
        layout.addWidget(self._build_ws2812_command_box(), 0, 2)
        layout.setColumnStretch(0, 2)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 2)

        return group

    def _build_motor_command_box(self) -> QGroupBox:
        group = QGroupBox("0x01 Motor Command")
        group.setFont(QFont("Arial", 10, QFont.Bold))

        layout = QFormLayout()
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)
        group.setLayout(layout)

        self.motor_left_spin = self._build_spinbox(-1000, 1000, 0, 50)
        self.motor_right_spin = self._build_spinbox(-1000, 1000, 0, 50)
        self.motor_timeout_spin = self._build_spinbox(0, 5000, DEFAULT_COMMAND_TIMEOUT_MS, 10)
        self.motor_rate_spin = self._build_spinbox(1, 100, DEFAULT_MOTOR_REPEAT_RATE_HZ, 1)
        self.motor_rate_spin.valueChanged.connect(self._update_motor_hold_timer_interval)
        self.motor_hold_label = QLabel("Idle")
        self.motor_hold_label.setFont(QFont("Arial", 10, QFont.Bold))

        send_once_button = QPushButton("Send Once")
        send_once_button.clicked.connect(self._send_motor_once)
        start_hold_button = QPushButton("Start Hold")
        start_hold_button.clicked.connect(self._start_motor_hold)
        stop_hold_button = QPushButton("Stop Hold")
        stop_hold_button.clicked.connect(self._stop_motor_hold)
        stop_motor_button = QPushButton("Stop Motors")
        stop_motor_button.clicked.connect(self._stop_motors)

        button_row = QWidget()
        button_layout = QHBoxLayout()
        button_layout.setContentsMargins(0, 0, 0, 0)
        button_layout.setSpacing(6)
        button_row.setLayout(button_layout)
        button_layout.addWidget(send_once_button)
        button_layout.addWidget(start_hold_button)
        button_layout.addWidget(stop_hold_button)
        button_layout.addWidget(stop_motor_button)

        layout.addRow("Left", self.motor_left_spin)
        layout.addRow("Right", self.motor_right_spin)
        layout.addRow("Timeout (ms)", self.motor_timeout_spin)
        layout.addRow("Hold Rate (Hz)", self.motor_rate_spin)
        layout.addRow("Hold State", self.motor_hold_label)
        layout.addRow(button_row)

        return group

    def _build_mower_command_box(self) -> QGroupBox:
        group = QGroupBox("0x02 Mower Command")
        group.setFont(QFont("Arial", 10, QFont.Bold))

        layout = QFormLayout()
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)
        group.setLayout(layout)

        self.mower_command_spin = self._build_spinbox(-1000, 1000, 0, 50)
        self.mower_timeout_spin = self._build_spinbox(0, 5000, DEFAULT_COMMAND_TIMEOUT_MS, 10)

        send_button = QPushButton("Send")
        send_button.clicked.connect(self._send_mower_once)
        stop_button = QPushButton("Stop")
        stop_button.clicked.connect(self._stop_mower_motor)

        button_row = QWidget()
        button_layout = QHBoxLayout()
        button_layout.setContentsMargins(0, 0, 0, 0)
        button_layout.setSpacing(6)
        button_row.setLayout(button_layout)
        button_layout.addWidget(send_button)
        button_layout.addWidget(stop_button)

        layout.addRow("Command", self.mower_command_spin)
        layout.addRow("Timeout (ms)", self.mower_timeout_spin)
        layout.addRow(button_row)

        return group

    def _build_ws2812_command_box(self) -> QGroupBox:
        group = QGroupBox("0x03 WS2812 Command")
        group.setFont(QFont("Arial", 10, QFont.Bold))

        layout = QFormLayout()
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)
        group.setLayout(layout)

        self.ws2812_mode_combo = QComboBox()
        self.ws2812_mode_combo.setFont(QFont("Consolas", 10))
        for mode, label in [
            (0x00, "CLEAR"),
            (0x01, "ALL_ON"),
            (0x02, "FLOW"),
            (0x03, "TURN_LEFT"),
            (0x04, "TURN_RIGHT"),
            (0x05, "SHOW"),
        ]:
            self.ws2812_mode_combo.addItem(f"{label} (0x{mode:02X})", mode)

        self.ws2812_red_spin = self._build_spinbox(0, 255, 0, 5)
        self.ws2812_green_spin = self._build_spinbox(0, 255, 255, 5)
        self.ws2812_blue_spin = self._build_spinbox(0, 255, 0, 5)
        self.ws2812_period_spin = self._build_spinbox(0, 5000, 200, 10)

        send_button = QPushButton("Send")
        send_button.clicked.connect(self._send_ws2812_once)
        clear_button = QPushButton("Clear")
        clear_button.clicked.connect(self._clear_ws2812)

        button_row = QWidget()
        button_layout = QHBoxLayout()
        button_layout.setContentsMargins(0, 0, 0, 0)
        button_layout.setSpacing(6)
        button_row.setLayout(button_layout)
        button_layout.addWidget(send_button)
        button_layout.addWidget(clear_button)

        layout.addRow("Mode", self.ws2812_mode_combo)
        layout.addRow("Red", self.ws2812_red_spin)
        layout.addRow("Green", self.ws2812_green_spin)
        layout.addRow("Blue", self.ws2812_blue_spin)
        layout.addRow("Period (ms)", self.ws2812_period_spin)
        layout.addRow(button_row)

        return group

    def _build_log_group(self) -> QGroupBox:
        group = QGroupBox("Event Log")
        group.setFont(QFont("Arial", 10, QFont.Bold))

        layout = QVBoxLayout()
        group.setLayout(layout)

        self.log_text = QPlainTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Consolas", 10))

        clear_button = QPushButton("Clear Log")
        clear_button.clicked.connect(self.clear_log)

        layout.addWidget(self.log_text, 1)
        layout.addWidget(clear_button)

        return group

    def refresh_ports(self) -> None:
        current_text = self.port_combo.currentText().strip()
        self.port_combo.clear()

        if list_ports is None:
            if current_text:
                self.port_combo.addItem(current_text)
            self.log_event("ERROR", "pyserial is not installed, so ports cannot be listed.")
            return

        ports = sorted(list_ports.comports(), key=lambda item: item.device)
        for port in ports:
            description = port.description or "Unknown device"
            self.port_combo.addItem(f"{port.device} | {description}", port.device)

        if current_text:
            self.port_combo.setEditText(current_text)
        elif ports:
            self.port_combo.setCurrentIndex(0)

    def selected_port(self) -> str:
        current_data = self.port_combo.currentData()
        if isinstance(current_data, str) and current_data:
            return current_data

        current_text = self.port_combo.currentText().strip()
        if " | " in current_text:
            return current_text.split(" | ", 1)[0].strip()
        return current_text

    def _connected_monitor(self) -> Optional[SerialMonitorThread]:
        if self._monitor_thread is None or not self._monitor_thread.isRunning():
            QMessageBox.warning(
                self,
                "Not Connected",
                "Please connect to the serial port before sending commands.",
            )
            return None
        return self._monitor_thread

    def _update_motor_hold_label(self, active: bool, details: str = "") -> None:
        if active:
            self.motor_hold_label.setText(f"Active {details}".strip())
            self.motor_hold_label.setStyleSheet("color: #2e7d32; font-weight: 700;")
        else:
            self.motor_hold_label.setText("Idle")
            self.motor_hold_label.setStyleSheet("color: #616161; font-weight: 700;")

    def _queue_motor_command(self, left: int, right: int, timeout_ms: int) -> Optional[int]:
        monitor = self._connected_monitor()
        if monitor is None:
            return None
        return monitor.queue_motor_command(left, right, timeout_ms)

    def _queue_mower_command(self, command: int, timeout_ms: int) -> Optional[int]:
        monitor = self._connected_monitor()
        if monitor is None:
            return None
        return monitor.queue_mower_command(command, timeout_ms)

    def _queue_ws2812_command(
        self,
        mode: int,
        red: int,
        green: int,
        blue: int,
        period_ms: int,
    ) -> Optional[int]:
        monitor = self._connected_monitor()
        if monitor is None:
            return None
        return monitor.queue_ws2812_command(mode, red, green, blue, period_ms)

    def _send_motor_once(self) -> None:
        seq = self._queue_motor_command(
            left=self.motor_left_spin.value(),
            right=self.motor_right_spin.value(),
            timeout_ms=self.motor_timeout_spin.value(),
        )
        if seq is not None:
            self.log_event(
                "INFO",
                f"Queued 0x01 motor command seq={seq} "
                f"left={self.motor_left_spin.value()} "
                f"right={self.motor_right_spin.value()}",
            )

    def _send_motor_hold_tick(self) -> None:
        self._queue_motor_command(
            left=self.motor_left_spin.value(),
            right=self.motor_right_spin.value(),
            timeout_ms=self.motor_timeout_spin.value(),
        )

    def _update_motor_hold_timer_interval(self, _value: int = 0) -> None:
        if self.motor_repeat_timer.isActive():
            self.motor_repeat_timer.setInterval(
                max(10, int(1000 / max(1, self.motor_rate_spin.value())))
            )
            self._update_motor_hold_label(
                True,
                f"@ {self.motor_rate_spin.value()} Hz",
            )

    def _start_motor_hold(self) -> None:
        if self._connected_monitor() is None:
            return
        self.motor_repeat_timer.setInterval(
            max(10, int(1000 / max(1, self.motor_rate_spin.value())))
        )
        self.motor_repeat_timer.start()
        self._update_motor_hold_label(True, f"@ {self.motor_rate_spin.value()} Hz")
        self.log_event(
            "INFO",
            "Motor hold started "
            f"left={self.motor_left_spin.value()} "
            f"right={self.motor_right_spin.value()} "
            f"rate={self.motor_rate_spin.value()}Hz",
        )
        self._send_motor_hold_tick()

    def _stop_motor_hold(self) -> None:
        if self.motor_repeat_timer.isActive():
            self.motor_repeat_timer.stop()
            self.log_event("INFO", "Motor hold stopped")
        self._update_motor_hold_label(False)

    def _stop_motors(self) -> None:
        self._stop_motor_hold()
        self.motor_left_spin.setValue(0)
        self.motor_right_spin.setValue(0)
        seq = self._queue_motor_command(0, 0, self.motor_timeout_spin.value())
        if seq is not None:
            self.log_event("INFO", f"Queued 0x01 motor stop seq={seq}")

    def _send_mower_once(self) -> None:
        seq = self._queue_mower_command(
            command=self.mower_command_spin.value(),
            timeout_ms=self.mower_timeout_spin.value(),
        )
        if seq is not None:
            self.log_event(
                "INFO",
                f"Queued 0x02 mower command seq={seq} "
                f"command={self.mower_command_spin.value()}",
            )

    def _stop_mower_motor(self) -> None:
        self.mower_command_spin.setValue(0)
        seq = self._queue_mower_command(0, self.mower_timeout_spin.value())
        if seq is not None:
            self.log_event("INFO", f"Queued 0x02 mower stop seq={seq}")

    def _send_ws2812_once(self) -> None:
        seq = self._queue_ws2812_command(
            mode=int(self.ws2812_mode_combo.currentData()),
            red=self.ws2812_red_spin.value(),
            green=self.ws2812_green_spin.value(),
            blue=self.ws2812_blue_spin.value(),
            period_ms=self.ws2812_period_spin.value(),
        )
        if seq is not None:
            self.log_event(
                "INFO",
                f"Queued 0x03 ws2812 command seq={seq} "
                f"mode={ws2812_mode_name(int(self.ws2812_mode_combo.currentData()))}",
            )

    def _clear_ws2812(self) -> None:
        self.ws2812_mode_combo.setCurrentIndex(0)
        self.ws2812_red_spin.setValue(0)
        self.ws2812_green_spin.setValue(0)
        self.ws2812_blue_spin.setValue(0)
        self.ws2812_period_spin.setValue(0)
        seq = self._queue_ws2812_command(0x00, 0, 0, 0, 0)
        if seq is not None:
            self.log_event("INFO", f"Queued 0x03 ws2812 clear seq={seq}")

    def toggle_connection(self) -> None:
        if self._monitor_thread is not None and self._monitor_thread.isRunning():
            self.stop_monitor()
            return

        port = self.selected_port()
        if not port:
            QMessageBox.warning(self, "Missing Port", "Please select or enter a serial port.")
            return

        try:
            baudrate = int(self.baud_combo.currentText().strip())
        except ValueError:
            QMessageBox.warning(self, "Invalid Baud", "Baud rate must be an integer.")
            return

        self._clear_status_panels()
        self._reset_summary()
        self.log_event("INFO", f"Starting monitor on {port} at {baudrate} baud")

        self._monitor_thread = SerialMonitorThread(
            port=port,
            baudrate=baudrate,
            read_timeout_s=DEFAULT_READ_TIMEOUT_S,
        )
        self._monitor_thread.connection_changed.connect(self._set_connected_state)
        self._monitor_thread.motor_status_received.connect(self._update_motor_status)
        self._monitor_thread.mower_status_received.connect(self._update_mower_status)
        self._monitor_thread.ws2812_status_received.connect(self._update_ws2812_status)
        self._monitor_thread.frame_count_changed.connect(self._update_frame_counts)
        self._monitor_thread.frame_activity_changed.connect(self._update_frame_activity)
        self._monitor_thread.tx_frame_sent.connect(self._update_tx_activity)
        self._monitor_thread.log_message.connect(self.log_event)
        self._monitor_thread.finished.connect(self._monitor_finished)
        self._monitor_thread.start()

    def stop_monitor(self) -> None:
        if self._monitor_thread is None:
            return

        self._stop_motor_hold()
        self.log_event("INFO", "Stopping serial monitor")
        self._monitor_thread.stop()
        self._monitor_thread.wait(1500)

    def _monitor_finished(self) -> None:
        self._monitor_thread = None
        self._stop_motor_hold()
        self._set_connected_state(False, "Disconnected")

    def _set_connected_state(self, connected: bool, message: str) -> None:
        color = "#2e7d32" if connected else "#c62828"
        self.connection_label.setText(message)
        self.connection_label.setStyleSheet(f"color: {color};")
        self.connect_button.setText("Disconnect" if connected else "Connect")
        self.port_combo.setEnabled(not connected)
        self.baud_combo.setEnabled(not connected)
        self.refresh_button.setEnabled(not connected)
        self.command_group.setEnabled(connected)
        if not connected:
            self._update_motor_hold_label(False)

    def _reset_summary(self) -> None:
        for label in self.summary_labels.values():
            label.setText("-")
        self.summary_labels["motor_frames"].setText("0")
        self.summary_labels["mower_frames"].setText("0")
        self.summary_labels["ws2812_frames"].setText("0")
        self.summary_labels["unknown_frames"].setText("0")
        self.summary_labels["last_tx_time"].setText("-")
        self.summary_labels["last_tx_type"].setText("-")
        self.summary_labels["last_tx_seq"].setText("-")

    def _clear_status_panels(self) -> None:
        self.motor_panel.set_all_defaults()
        self.mower_panel.set_all_defaults()
        self.ws2812_panel.set_all_defaults()

    def _update_frame_counts(self, counts: Dict[str, int]) -> None:
        self.summary_labels["motor_frames"].setText(str(counts.get("0x81", 0)))
        self.summary_labels["mower_frames"].setText(str(counts.get("0x82", 0)))
        self.summary_labels["ws2812_frames"].setText(str(counts.get("0x83", 0)))

        known_total = (
            counts.get("0x81", 0) + counts.get("0x82", 0) + counts.get("0x83", 0)
        )
        total = sum(counts.values())
        self.summary_labels["unknown_frames"].setText(str(total - known_total))

    def _update_frame_activity(
        self, last_time_text: str, frame_type: str, seq: int, raw_frame: str
    ) -> None:
        self.summary_labels["last_frame_time"].setText(last_time_text)
        self.summary_labels["last_frame_type"].setText(frame_type)
        self.summary_labels["last_frame_seq"].setText(str(seq))
        self.summary_labels["last_raw_frame"].setText(raw_frame)

    def _update_tx_activity(self, frame_type: str, seq: int, description: str) -> None:
        tx_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.summary_labels["last_tx_time"].setText(tx_time)
        self.summary_labels["last_tx_type"].setText(frame_type)
        self.summary_labels["last_tx_seq"].setText(str(seq))

    def _update_motor_status(self, status: MotorStatus) -> None:
        flag_text, color = flags_text_and_color(status.flags)
        self.motor_panel.set_value("last_update", now_text())
        self.motor_panel.set_value("status_seq", str(status.seq))
        self.motor_panel.set_value("last_rx_seq", str(status.last_rx_seq))
        self.motor_panel.set_value(
            "flags", f"{flag_text} ({','.join(decode_flags(status.flags))})", color
        )
        self.motor_panel.set_value("command_age_ms", str(status.command_age_ms))
        self.motor_panel.set_value("commanded_left", str(status.commanded_left_permille))
        self.motor_panel.set_value("commanded_right", str(status.commanded_right_permille))
        self.motor_panel.set_value("applied_left_pwm", str(status.applied_left_pwm))
        self.motor_panel.set_value("applied_right_pwm", str(status.applied_right_pwm))

    def _update_mower_status(self, status: MowerMotorStatus) -> None:
        flag_text, color = flags_text_and_color(status.flags)
        self.mower_panel.set_value("last_update", now_text())
        self.mower_panel.set_value("status_seq", str(status.seq))
        self.mower_panel.set_value("last_rx_seq", str(status.last_rx_seq))
        self.mower_panel.set_value(
            "flags", f"{flag_text} ({','.join(decode_flags(status.flags))})", color
        )
        self.mower_panel.set_value("command_age_ms", str(status.command_age_ms))
        self.mower_panel.set_value("commanded", str(status.commanded_permille))
        self.mower_panel.set_value("applied_pwm", str(status.applied_pwm))

    def _update_ws2812_status(self, status: Ws2812Status) -> None:
        flag_text, color = flags_text_and_color(status.flags)
        self.ws2812_panel.set_value("last_update", now_text())
        self.ws2812_panel.set_value("status_seq", str(status.seq))
        self.ws2812_panel.set_value("last_rx_seq", str(status.last_rx_seq))
        self.ws2812_panel.set_value(
            "flags", f"{flag_text} ({','.join(decode_flags(status.flags))})", color
        )
        self.ws2812_panel.set_value(
            "mode", f"{ws2812_mode_name(status.mode)} (0x{status.mode:02X})"
        )
        self.ws2812_panel.set_value(
            "rgb", f"R={status.red} G={status.green} B={status.blue}"
        )
        self.ws2812_panel.set_value("effect_period_ms", str(status.effect_period_ms))

    def log_event(self, level: str, message: str) -> None:
        line = f"[{now_text()}] [{level}] {message}"
        self._log_lines.append(line)
        if len(self._log_lines) > MAX_LOG_LINES:
            self._log_lines = self._log_lines[-MAX_LOG_LINES:]
        self.log_text.setPlainText("\n".join(self._log_lines))
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())

    def clear_log(self) -> None:
        self._log_lines.clear()
        self.log_text.clear()

    def closeEvent(self, event) -> None:  # type: ignore[override]
        self.stop_monitor()
        event.accept()


def main() -> None:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    if SERIAL_IMPORT_ERROR is not None:
        QMessageBox.critical(
            None,
            "Missing Dependency",
            "pyserial is required.\n\nInstall it with:\n\npip install pyserial",
        )
        raise SystemExit(1)

    window = Stm32UartMonitorWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
