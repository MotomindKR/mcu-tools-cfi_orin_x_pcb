from __future__ import annotations

import argparse
from decimal import Decimal
import math
import os
from pathlib import Path
import queue
import re
import struct
import sys
import threading
import time
from dataclasses import dataclass
from typing import Iterable

try:
    import tkinter as tk
    from tkinter import messagebox, ttk
except ImportError:  # Terminal decoding also works on hosts without Tk installed.
    tk = None
    messagebox = ttk = None

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # Keep the decoder importable/testable without pyserial.
    serial = None
    list_ports = None


# Examples (run from the project root):
# python cli_cog/cli_cog.py  # Linux: relaunch with sudo and connect to /dev/ttyTHS1.
# python cli_cog/cli_cog.py --list-ports
# python cli_cog/cli_cog.py --port COM3
# python cli_cog/cli_cog.py --cli --port COM3 --baud 921600
# Binary capture on stdin (cmd.exe):
# python cli_cog/cli_cog.py --cli --stdin < log.bin


DEFAULT_PORT = "/dev/ttyTHS1"

TYPE_NAMES = {
    0x01: "Battery V/I",
    0x02: "GPI",
    0x03: "IWC",
    0x04: "Reserved 0x04",
    0x05: "Temperature / Pressure",
    0x06: "Reserved 0x06",
    0x07: "VMOT V/I",
    0x08: "Tick count",
}
PAYLOAD_SIZES = {0x01: 12, 0x02: 4, 0x03: 12, 0x05: 8, 0x07: 8, 0x08: 4}
MAX_ENCODED_SIZE = 246  # 245 raw bytes + one COBS code byte (delimiter excluded).
MAX_COMMAND_SIZE = 62  # CLI_CMD_BUFFER_SIZE=64, including two reserved bytes.
MAX_LOG_LINES = 2000

# Bit order follows gpio/gpi.c. Bits 13..31 are reserved.
GPI_NAMES = (
    "nEMERGENCY", "nINT_INA226", "nDET_ADAP", "nSYS_RESET_3V3",
    "nMOD_SLEEP_3V3", "nSHUTDOWN_REQ", "PG_3V3_SYS", "nEN_SAFETY",
    "MCU_PWR_BTN", "STM_CAN_STB", "POWER_EN_3V3", "EN_5V0_EXT", "EN_MOT_PWR",
) + tuple(f"RESERVED_{bit}" for bit in range(13, 32))
# Bit order follows iwc/iwc.h.
IWC_NAMES = {
    "Info": (
        "EMERGENCY_EFFECTIVE", "EMERGENCY_HARDWARE", "EMERGENCY_SOFTWARE",
        "CHG_BYPASS_CONNECTED", "BATTERY_CHARGING",
    ),
    "Warning": (
        "OVER_CURRENT_DISCHARGE_VBAT", "OVER_CURRENT_CHARGE_VBAT", "OVER_VOLTAGE_VBAT",
        "UNDER_VOLTAGE_VBAT", "OVER_CURRENT_VMOT", "OVER_VOLTAGE_VMOT",
        "OVER_TEMPERATURE_PCB", "UNDER_TEMPERATURE_PCB",
    ),
    "Critical": (
        "OVER_CURRENT_DISCHARGE_VBAT", "OVER_CURRENT_CHARGE_VBAT", "OVER_VOLTAGE_VBAT",
        "UNDER_VOLTAGE_VBAT", "OVER_CURRENT_VMOT", "OVER_VOLTAGE_VMOT",
        "OVER_TEMPERATURE_PCB", "UNDER_TEMPERATURE_PCB", "POWER_FAULT_3V3",
    ),
}

CLI_COMMAND_TEXT = """UART settings: 921600 baud, 8N1, flow control disabled

SYSTEM
  reboot
  sync

CAMERA SYNC
  camera sync on | off
  camera sync frequency <Hz>

LED (region: 0=all, 1=left, 2=right; color accepts 0xRRGGBB)
  led normal_color <color>
  led <region> off
  led <region> rainbow <step_size>
  led <region> breath <color> <interval_s>
  led <region> blink <color> <interval_s>
  led <region> burst_blink <color> <count> <blink_interval_s> <off_interval_s>
  led <region> static <color>

IWC
  iwc status
  iwc emergency press | release
  iwc emergency stop_category <0|1>
  iwc emergency latch <0|1>
  iwc critical clear
"""


class FrameError(ValueError):
    """A received frame is incomplete or violates the COG protocol."""


@dataclass(frozen=True)
class CogFrame:
    message_type: int
    sequence: int
    payload: bytes


def crc16_ccitt_false(data: bytes) -> int:
    """Return CRC-16/CCITT-FALSE for *data* (poly 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_decode(encoded: bytes) -> bytes:
    """Decode one COBS block, excluding its trailing 0x00 delimiter."""
    if not encoded:
        raise FrameError("empty COBS frame")
    if 0 in encoded:
        raise FrameError("COBS block contains a zero byte")
    decoded = bytearray()
    index = 0
    size = len(encoded)
    while index < size:
        code = encoded[index]
        index += 1
        if code == 0 or index + code - 1 > size:
            raise FrameError("invalid COBS encoding")
        decoded.extend(encoded[index:index + code - 1])
        index += code - 1
        if code != 0xFF and index < size:
            decoded.append(0)
    return bytes(decoded)


def decode_frame(encoded: bytes) -> CogFrame:
    """COBS-decode, length-check, and CRC-check a COG frame."""
    if len(encoded) > MAX_ENCODED_SIZE:
        raise FrameError(f"encoded frame exceeds {MAX_ENCODED_SIZE} bytes")
    raw = cobs_decode(encoded)
    if len(raw) < 5:
        raise FrameError("frame is shorter than header and CRC")
    message_type, sequence, payload_len = raw[:3]
    if message_type not in TYPE_NAMES:
        raise FrameError(f"invalid message type 0x{message_type:02X}")
    expected_size = 3 + payload_len + 2
    if len(raw) != expected_size:
        raise FrameError(f"length mismatch: header says {payload_len}, received {len(raw) - 5}")
    if payload_len > 240:
        raise FrameError("payload exceeds 240 bytes")
    expected_payload = PAYLOAD_SIZES.get(message_type)
    if expected_payload is not None and payload_len != expected_payload:
        raise FrameError(f"invalid payload length for type 0x{message_type:02X}: expected {expected_payload}, received {payload_len}")
    received_crc = int.from_bytes(raw[-2:], "little")
    calculated_crc = crc16_ccitt_false(raw[:-2])
    if received_crc != calculated_crc:
        raise FrameError(f"CRC mismatch: received 0x{received_crc:04X}, expected 0x{calculated_crc:04X}")
    return CogFrame(message_type, sequence, raw[3:-2])


class FrameStream:
    """Bounded, chunk-independent framing; discard an oversized packet through its delimiter."""

    def __init__(self, recover_prefix: bool = False) -> None:
        self.buffer = bytearray()
        self.discarding = False
        self.recover_prefix = recover_prefix

    def feed(self, chunk: bytes) -> Iterable[tuple[str, bytes | str]]:
        for byte in chunk:
            if byte == 0:
                if self.buffer and not self.discarding:
                    encoded = bytes(self.buffer)
                    # A connection/mode change can start in the middle of a frame,
                    # or include the old debug prompt before the first COG frame.
                    if self.recover_prefix:
                        for offset in range(len(encoded)):
                            try:
                                decode_frame(encoded[offset:])
                            except FrameError:
                                continue
                            if offset:
                                yield "notice", "discarded prefix before first valid COG frame"
                            encoded = encoded[offset:]
                            self.recover_prefix = False
                            break
                    yield "frame", encoded
                self.buffer.clear()
                self.discarding = False
            elif not self.discarding:
                self.buffer.append(byte)
                if len(self.buffer) > MAX_ENCODED_SIZE:
                    if self.recover_prefix:
                        # Keep the tail until the first CRC-validated frame boundary.
                        del self.buffer[0]
                    else:
                        self.buffer.clear()
                        self.discarding = True
                        yield "error", f"encoded frame exceeds {MAX_ENCODED_SIZE} bytes"


def encode_command(command: str) -> bytes:
    """Validate a single command against the embedded CLI's actual input capacity."""
    if not command or any(not 32 <= ord(char) <= 126 for char in command):
        raise ValueError("enter one printable ASCII command (no newlines/control characters)")
    if len(command) > MAX_COMMAND_SIZE:
        raise ValueError(f"command exceeds firmware limit of {MAX_COMMAND_SIZE} ASCII characters")
    return command.encode("ascii") + b"\r\n"


def camera_frequency_command(value: str) -> str:
    """Build the firmware command from a 1..240 Hz decimal input."""
    value = value.strip()
    if re.fullmatch(r"[0-9]+(?:\.[0-9]+)?", value) is None:
        raise ValueError("camera sync frequency must be a decimal number from 1 to 240 Hz")
    frequency = Decimal(value)
    if not 1 <= frequency <= 240:
        raise ValueError("camera sync frequency must be a decimal number from 1 to 240 Hz")
    command = f"camera sync frequency {frequency:f}"
    encode_command(command)
    return command


def describe_frame(frame: CogFrame) -> list[str]:
    """Return human-readable, validated COG frame contents for CLI output."""
    title = TYPE_NAMES.get(frame.message_type, f"Unknown 0x{frame.message_type:02X}")
    lines = [f"OK  {title}  seq={frame.sequence}  len={len(frame.payload)}"]
    try:
        if frame.message_type == 0x01:
            voltage, current, soc = struct.unpack("<3f", frame.payload)
            lines.append(f"  Battery: {voltage:.3f} V, {current:.3f} A, SoC {soc:.3f} %")
        elif frame.message_type == 0x05:
            temperature, pressure = struct.unpack("<2f", frame.payload)
            lines.append(f"  Temperature / pressure: {temperature:.3f} degC, {pressure:.3f} Pa")
        elif frame.message_type == 0x07:
            voltage, current = struct.unpack("<2f", frame.payload)
            lines.append(f"  VMOT: {voltage:.3f} V, {current:.3f} A (filtered)")
        elif frame.message_type == 0x08:
            tick_count, = struct.unpack("<I", frame.payload)
            lines.append(f"  Kernel tick count: {tick_count}")
        elif frame.message_type == 0x02:
            bits, = struct.unpack("<I", frame.payload)
            active = [name for bit, name in enumerate(GPI_NAMES) if bits & (1 << bit)]
            lines.append(f"  GPI bits: 0x{bits:08X}  set: {', '.join(active) if active else '(none)'}")
        elif frame.message_type == 0x03:
            for label, bits in zip(("Info", "Warning", "Critical"), struct.unpack("<3I", frame.payload)):
                active = [name for bit, name in enumerate(IWC_NAMES[label]) if bits & (1 << bit)]
                lines.append(f"  IWC {label}: 0x{bits:08X}  set: {', '.join(active) if active else '(none)'}")
    except struct.error:
        lines.append(f"  ERROR: invalid payload length (expected layout for {title})")
    return lines


def process_cli_bytes(chunks: Iterable[bytes]) -> int:
    """Decode zero-delimited COG data from *chunks* and print a validation log."""
    valid = invalid = 0
    stream = FrameStream()
    for chunk in chunks:
        for kind, value in stream.feed(chunk):
            if kind == "frame":
                try:
                    frame = decode_frame(value)
                    valid += 1
                    print(*describe_frame(frame), sep="\n")
                except FrameError as error:
                    invalid += 1
                    print(f"ERR {error}", file=sys.stderr)
            elif kind == "error":
                invalid += 1
                print(f"ERR {value}", file=sys.stderr)
    if stream.buffer:
        invalid += 1
        print("ERR incomplete frame at end of input (missing 0x00 delimiter)", file=sys.stderr)
    print(f"Validation summary: valid={valid}, invalid={invalid}")
    return 0 if invalid == 0 else 1


def run_cli(port: str | None, baudrate: int, use_stdin: bool) -> int:
    """Run the COG monitor without Tk. Ctrl+C cleanly closes a serial port."""
    if use_stdin:
        return process_cli_bytes(iter(lambda: sys.stdin.buffer.read(256), b""))
    if serial is None:
        print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
        return 2
    try:
        with serial.Serial(port, baudrate, timeout=0.2) as serial_port:
            print(f"Reading COG frames from {serial_port.port} @ {serial_port.baudrate}. Press Ctrl+C to stop.")
            def serial_chunks() -> Iterable[bytes]:
                while True:
                    chunk = serial_port.read(256)
                    if chunk:
                        yield chunk
            try:
                return process_cli_bytes(serial_chunks())
            except KeyboardInterrupt:
                print("\nStopped.")
                return 0
    except (ValueError, OSError, serial.SerialException) as error:
        print(f"Serial port error: {error}", file=sys.stderr)
        return 2


class Led(tk.Canvas if tk is not None else object):
    """Raw bit lamp: gray=unknown, green=0, red=1."""

    def __init__(self, parent: tk.Misc, **kwargs: object) -> None:
        super().__init__(parent, width=18, height=18, highlightthickness=0, **kwargs)
        self._lamp = self.create_oval(3, 3, 15, 15, fill="#999999", outline="#666666")

    def set(self, active: bool | None) -> None:
        fill, outline = ("#999999", "#666666") if active is None else (
            ("#d94b45", "#8d2522") if active else ("#2eaf59", "#1e6f38"))
        self.itemconfigure(self._lamp, fill=fill, outline=outline)


class GpioLevel(ttk.Label if ttk is not None else object):
    """Display the raw GPIO level without assigning an active/inactive meaning."""

    def __init__(self, parent: tk.Misc) -> None:
        super().__init__(parent, text="-", width=2, anchor="center",
                         font=("TkDefaultFont", 11, "bold"))

    def set(self, high: bool | None) -> None:
        self.configure(text="-" if high is None else "H" if high else "L")


class CogMonitor(tk.Tk if tk is not None else object):
    def __init__(self, port: str | None = DEFAULT_PORT, baudrate: int = 921600) -> None:
        super().__init__()
        self.title("COG Log Monitor")
        self.geometry("1180x780")
        self.minsize(960, 620)
        self._serial = None
        self._stop_reader = threading.Event()
        self._reader: threading.Thread | None = None
        self._session = 0
        self._events: queue.Queue = queue.Queue(maxsize=1024)
        self._commands: queue.Queue = queue.Queue(maxsize=8)
        self._stream = FrameStream(recover_prefix=True)
        self._history: list[str] = []
        self._history_index = 0
        self._leds: dict[str, list[Led | GpioLevel]] = {}
        self._sensor_values: dict[str, tk.StringVar] = {}
        self._last_frame_times: dict[int, float] = {}
        self._update_intervals = {
            kind: tk.StringVar(value="수신 간격: - ms") for kind in PAYLOAD_SIZES
        }
        self._good_frames = 0
        self._bad_frames = 0
        self._follow_log = True
        self._build_ui()
        self.baud_var.set(str(baudrate))
        if port:
            self.port_var.set(port)
        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.close)
        self._poll_id = self.after(50, self._drain_events)
        if port:
            self.after_idle(self.toggle_connection)

    def _build_ui(self) -> None:
        controls = ttk.Frame(self, padding=(10, 10, 10, 4))
        controls.grid(row=0, column=0, sticky="ew")
        ttk.Label(controls, text="Serial port").grid(row=0, column=0, padx=(0, 5))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(controls, textvariable=self.port_var, width=22)
        self.port_combo.grid(row=0, column=1, padx=(0, 8))
        ttk.Button(controls, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=(0, 14))
        ttk.Label(controls, text="Baudrate").grid(row=0, column=3, padx=(0, 5))
        self.baud_var = tk.StringVar(value="921600")
        ttk.Combobox(controls, textvariable=self.baud_var, width=11,
                     values=("9600", "57600", "115200", "230400", "460800", "921600")).grid(row=0, column=4, padx=(0, 10))
        self.connect_button = ttk.Button(controls, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=5)
        self.connection_var = tk.StringVar(value="Disconnected")
        ttk.Label(controls, textvariable=self.connection_var).grid(row=0, column=6, padx=(14, 0))
        self.tick_var = tk.StringVar(value="Tick: -")
        ttk.Label(controls, textvariable=self.tick_var, font=("TkDefaultFont", 10, "bold")).grid(
            row=0, column=7, padx=(16, 0))
        ttk.Label(controls, textvariable=self._update_intervals[0x08]).grid(
            row=1, column=7, padx=(16, 0), sticky="e")
        controls.columnconfigure(6, weight=1)

        notebook = ttk.Notebook(self)
        notebook.grid(row=1, column=0, sticky="nsew", padx=10, pady=(4, 6))
        self.rowconfigure(1, weight=1)
        self.columnconfigure(0, weight=1)
        sensors = ttk.Frame(notebook, padding=12)
        states = ttk.Frame(notebook, padding=12)
        cli_help = ttk.Frame(notebook, padding=12)
        notebook.add(sensors, text="Sensor data")
        notebook.add(states, text="IWC / GPI")
        notebook.add(cli_help, text="CLI commands")
        self._build_sensors(sensors)
        self._build_states(states)
        self._build_cli_help(cli_help)

        self._build_cli_command_bar()
        log_frame = ttk.LabelFrame(self, text="COG / CLI TX log", padding=8)
        log_frame.grid(row=3, column=0, sticky="ew", padx=10, pady=(0, 10))
        self._build_log(log_frame)

    def _build_cli_command_bar(self) -> None:
        commands = ttk.Frame(self, padding=(10, 0, 10, 6))
        commands.grid(row=2, column=0, sticky="ew")
        ttk.Label(commands, text="CLI command").grid(row=0, column=0, padx=(0, 7))
        self.cli_command_var = tk.StringVar()
        self.cli_command_entry = ttk.Entry(commands, textvariable=self.cli_command_var)
        self.cli_command_entry.grid(row=0, column=1, sticky="ew")
        self.cli_command_entry.bind("<Return>", self.send_cli_command)
        self.cli_command_entry.bind("<Up>", lambda event: self._recall_command(-1))
        self.cli_command_entry.bind("<Down>", lambda event: self._recall_command(1))
        self.cli_send_button = ttk.Button(commands, text="Send", command=self.send_cli_command,
                                          state="disabled")
        self.cli_send_button.grid(row=0, column=2, padx=(7, 0))
        ttk.Label(commands, text="Sends CR/LF to the connected UART").grid(row=0, column=3, padx=(12, 0))
        sync = ttk.Frame(commands)
        sync.grid(row=1, column=0, columnspan=4, sticky="w", pady=(7, 0))
        ttk.Label(sync, text="Camera sync").grid(row=0, column=0, padx=(0, 7))
        self.sync_on_button = ttk.Button(sync, text="ON", state="disabled",
                                         command=lambda: self.send_cli_command(command="camera sync on"))
        self.sync_on_button.grid(row=0, column=1, padx=(0, 4))
        self.sync_off_button = ttk.Button(sync, text="OFF", state="disabled",
                                          command=lambda: self.send_cli_command(command="camera sync off"))
        self.sync_off_button.grid(row=0, column=2, padx=(0, 12))
        ttk.Label(sync, text="Frequency (Hz)").grid(row=0, column=3, padx=(0, 5))
        self.sync_frequency_var = tk.StringVar(value="30")
        self.sync_frequency_entry = ttk.Entry(sync, textvariable=self.sync_frequency_var, width=7)
        self.sync_frequency_entry.grid(row=0, column=4, padx=(0, 5))
        self.sync_frequency_entry.bind("<Return>", self.send_camera_frequency)
        self.sync_frequency_button = ttk.Button(sync, text="Set", state="disabled",
                                                command=self.send_camera_frequency)
        self.sync_frequency_button.grid(row=0, column=5)
        commands.columnconfigure(1, weight=1)

    def send_camera_frequency(self, _event: object | None = None) -> None:
        try:
            command = camera_frequency_command(self.sync_frequency_var.get())
        except ValueError as error:
            self._append_log(f"TX CLI rejected: {error}", "error")
            return
        self.send_cli_command(command=command)

    def _build_cli_help(self, parent: ttk.Frame) -> None:
        """Show the command reference without requiring an active UART connection."""
        scrollbar = ttk.Scrollbar(parent, orient="vertical")
        command_text = tk.Text(parent, wrap="word", state="normal", font=("Consolas", 10),
                               padx=10, pady=8, yscrollcommand=scrollbar.set)
        scrollbar.configure(command=command_text.yview)
        command_text.insert("1.0", CLI_COMMAND_TEXT)
        command_text.configure(state="disabled")
        command_text.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def _build_sensors(self, parent: ttk.Frame) -> None:
        groups = {
            "Battery": (0x01, ("Bus voltage", "Shunt current", "Battery SoC")),
            "Temperature / pressure": (0x05, ("Temperature", "Pressure")),
            "VMOT": (0x07, ("VMOT voltage", "VMOT current")),
        }
        for i, (title, (kind, fields)) in enumerate(groups.items()):
            frame = ttk.LabelFrame(parent, text=title, padding=10)
            frame.grid(row=i // 2, column=i % 2, sticky="nsew", padx=6, pady=6)
            for row, field in enumerate(fields):
                value = tk.StringVar(value="-")
                self._sensor_values[field] = value
                ttk.Label(frame, text=field).grid(row=row, column=0, sticky="w", padx=(0, 20), pady=3)
                ttk.Label(frame, textvariable=value, font=("TkDefaultFont", 11, "bold")).grid(row=row, column=1, sticky="e", pady=3)
            ttk.Label(frame, textvariable=self._update_intervals[kind]).grid(
                row=len(fields), column=0, columnspan=2, sticky="w", pady=(10, 0))
            frame.columnconfigure(1, weight=1)
        parent.columnconfigure((0, 1), weight=1)
        parent.rowconfigure((0, 1), weight=1)

    def _build_states(self, parent: ttk.Frame) -> None:
        canvas = tk.Canvas(parent, highlightthickness=0)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        content = ttk.Frame(canvas)
        content.bind("<Configure>", lambda _event: canvas.configure(scrollregion=canvas.bbox("all")))
        window = canvas.create_window((0, 0), window=content, anchor="nw")
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(
            window, width=max(event.width, content.winfo_reqwidth())))
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")
        parent.rowconfigure(0, weight=1)
        parent.columnconfigure(0, weight=1)
        self._leds["GPI"] = self._make_led_group(
            content, "GPIO (H: HIGH, L: LOW, -: unknown)", GPI_NAMES, 0, gpio_levels=True)
        row = 1
        for category, names in IWC_NAMES.items():
            self._leds[category] = self._make_led_group(content, f"IWC {category} (red: set, green: clear)", names, row)
            row += 1

    def _make_led_group(self, parent: ttk.Frame, title: str, names: Iterable[str], row: int,
                        *, gpio_levels: bool = False) -> list[Led | GpioLevel]:
        frame = ttk.LabelFrame(parent, text=title, padding=8)
        frame.grid(row=row, column=0, sticky="ew", pady=5)
        kind = 0x02 if row == 0 else 0x03  # All IWC groups share one frame.
        ttk.Label(frame, textvariable=self._update_intervals[kind]).grid(
            row=0, column=0, columnspan=2, sticky="w", padx=7, pady=(0, 5))
        leds: list[Led | GpioLevel] = []
        for index, name in enumerate(names):
            item = ttk.Frame(frame)
            item.grid(row=1 + index // 2, column=index % 2, sticky="w", padx=7, pady=2)
            led = GpioLevel(item) if gpio_levels else Led(item)
            led.pack(side="left", padx=(0, 5))
            ttk.Label(item, text=f"{index:02d}  {name}").pack(side="left")
            leds.append(led)
        return leds

    def _build_log(self, parent: ttk.Frame) -> None:
        header = ttk.Frame(parent)
        header.pack(fill="x", pady=(0, 6))
        self.stats_var = tk.StringVar(value="Valid: 0   Invalid: 0")
        ttk.Label(header, textvariable=self.stats_var).pack(side="left")
        ttk.Button(header, text="Clear", command=self._clear_log).pack(side="right")
        self.log_follow_button = ttk.Button(header, text="Follow: ON", command=self._toggle_log_follow)
        self.log_follow_button.pack(side="right", padx=(0, 6))
        log_body = ttk.Frame(parent)
        log_body.pack(fill="both", expand=True)
        self.log_scrollbar = ttk.Scrollbar(log_body, orient="vertical")
        horizontal = ttk.Scrollbar(parent, orient="horizontal")
        self.log_text = tk.Text(log_body, height=11, wrap="none", state="disabled", font=("Consolas", 10),
                                yscrollcommand=self._on_log_scroll, xscrollcommand=horizontal.set)
        horizontal.configure(command=self.log_text.xview)
        horizontal.pack(fill="x")
        self.log_scrollbar.configure(command=self.log_text.yview)
        self.log_text.pack(side="left", fill="both", expand=True)
        self.log_scrollbar.pack(side="right", fill="y")
        self.log_text.tag_configure("error", foreground="#b32121")
        self.log_text.tag_configure("ok", foreground="#176b35")
        self.log_text.tag_configure("cli", foreground="#1d4ed8")

    def _on_log_scroll(self, first: str, last: str) -> None:
        self.log_scrollbar.set(first, last)
        # Inserts also update the scroll position; preserve the explicit Follow setting.

    def _toggle_log_follow(self) -> None:
        self._follow_log = not self._follow_log
        self.log_follow_button.configure(text=f"Follow: {'ON' if self._follow_log else 'OFF'}")
        if self._follow_log:
            self.log_text.see("end")

    def _clear_log(self) -> None:
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    def refresh_ports(self) -> None:
        if list_ports is None:
            self.port_combo["values"] = ()
            return
        ports = {port.device for port in list_ports.comports()}
        if sys.platform.startswith("linux"):
            # pyserial may omit the Tegra UARTs from comports(), although their
            # device nodes are available for direct use.
            for index in range(2):
                port = Path(f"/dev/ttyTHS{index}")
                if port.exists():
                    ports.add(str(port))
        ports = sorted(ports)
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def toggle_connection(self) -> None:
        if self._serial is not None:
            self.disconnect()
            return
        if serial is None:
            messagebox.showerror("pyserial required", "Install it with: python -m pip install pyserial")
            return
        try:
            port = self.port_var.get().strip()
            baud = int(self.baud_var.get())
            if not port or baud <= 0:
                raise ValueError("Enter a serial port and a positive baud rate")
            self._serial = serial.Serial(port, baud, timeout=0.02, write_timeout=0.5,
                                         bytesize=8, parity="N", stopbits=1,
                                         xonxoff=False, rtscts=False, dsrdtr=False)
        except (ValueError, OSError, serial.SerialException) as error:
            messagebox.showerror("Unable to open port", str(error))
            return
        self._session += 1
        self._stop_reader = threading.Event()
        self._commands = queue.Queue(maxsize=8)
        self._reset_display()
        self._reset_decoder()
        self._good_frames = self._bad_frames = 0
        self._update_stats()
        self._reader = threading.Thread(target=self._read_serial,
                                        args=(self._serial, self._stop_reader, self._session, self._commands),
                                        daemon=True)
        self._reader.start()
        self.connect_button.configure(text="Disconnect")
        self.cli_send_button.configure(state="normal")
        for button in (self.sync_on_button, self.sync_off_button, self.sync_frequency_button):
            button.configure(state="normal")
        self.connection_var.set(f"Connected: {self._serial.port} @ {self._serial.baudrate}")
        self._append_log(f"UART connected: {self._serial.port} @ {self._serial.baudrate}", "cli")

    def disconnect(self) -> None:
        self._stop_reader.set()
        self._session += 1  # Reject queued callbacks from this and earlier connections.
        port = self._serial
        self._serial = None
        if port is not None:
            try:
                # Cancel the pending read, then let the worker leave before
                # closing its OS handles (important for pyserial on Windows).
                cancel = getattr(port, "cancel_read", None)
                if cancel is not None:
                    cancel()
            except OSError:
                pass
            if self._reader is not None:
                self._reader.join(timeout=0.75)  # read=20 ms, write timeout=500 ms.
            try:
                port.close()
            except OSError as error:
                self._append_log(f"UART close failed: {error}", "error")
            self._append_log("UART disconnected", "cli")
        self._reset_decoder()
        self._reset_display()
        self.connect_button.configure(text="Connect")
        self.cli_send_button.configure(state="disabled")
        for button in (self.sync_on_button, self.sync_off_button, self.sync_frequency_button):
            button.configure(state="disabled")
        self.connection_var.set("Disconnected")

    def send_cli_command(self, _event: object | None = None, *, command: str | None = None) -> None:
        """Send one newline-terminated CLI command through the connected UART."""
        from_entry = command is None
        source = self.cli_command_var.get() if from_entry else command
        if not source.strip():
            return
        try:
            # Validate before strip so pasted line terminators cannot execute commands.
            encode_command(source)
            command = source.strip()
            encoded = encode_command(command)
        except ValueError as error:
            self._append_log(f"TX CLI rejected: {error}", "error")
            return
        if self._serial is None:
            self._append_log(f"TX CLI rejected (not connected): {command}", "error")
            return
        try:
            self._commands.put_nowait((command, encoded))
        except queue.Full:
            self._append_log("TX CLI rejected: command queue full", "error")
            return
        self._append_log(f"TX queued > {command}", "cli")
        if not self._history or self._history[-1] != command:
            self._history.append(command)
            self._history = self._history[-100:]
        self._history_index = len(self._history)
        if from_entry:
            self.cli_command_var.set("")

    def _recall_command(self, direction: int) -> str:
        self._history_index = max(0, min(len(self._history), self._history_index + direction))
        self.cli_command_var.set(self._history[self._history_index]
                                 if self._history_index < len(self._history) else "")
        self.cli_command_entry.icursor("end")
        return "break"

    def _read_serial(self, port, stop: threading.Event, session: int, commands: queue.Queue) -> None:
        """All UART I/O runs here; only the main thread ever calls Tk."""
        def post(kind: str, value: object) -> None:
            while not stop.is_set():
                try:
                    self._events.put((session, kind, value), timeout=0.05)
                    return
                except queue.Full:
                    continue

        pending = b""
        command = ""
        next_write = 0.0
        try:
            while not stop.is_set():
                if not pending:
                    try:
                        command, pending = commands.get_nowait()
                    except queue.Empty:
                        pass
                if pending and time.monotonic() >= next_write:
                    # Firmware RX ring is only 32 bytes and is serviced every 10 ms.
                    # Pace 16-byte chunks so long LED commands cannot overrun it.
                    part = pending[:16]
                    written = port.write(part)
                    if written != len(part):
                        raise OSError(f"short serial write ({written}/{len(part)} bytes)")
                    pending = pending[len(part):]
                    next_write = time.monotonic() + 0.03
                    if not pending:
                        post("sent", command)
                # Avoid waiting to fill a block; timestamp UART receipt before
                # queueing so the UI polling delay does not affect intervals.
                chunk = port.read(max(1, min(getattr(port, "in_waiting", 0), 256)))
                if chunk:
                    post("data", (chunk, time.monotonic()))
        except (OSError, ValueError) as error:
            if not stop.is_set():
                post("error", str(error))

    def _drain_events(self) -> None:
        for _ in range(200):
            try:
                session, kind, value = self._events.get_nowait()
            except queue.Empty:
                break
            if session != self._session:
                continue
            if kind == "error":
                self._append_log(f"UART error: {value}", "error")
                self.disconnect()
            elif kind == "sent":
                self._append_log(f"TX CLI > {value}", "cli")
            elif kind == "data":
                chunk, received_at = value
                self._receive_bytes(chunk, received_at)
        self._poll_id = self.after(50, self._drain_events)

    def _reset_display(self) -> None:
        self._last_frame_times.clear()
        for interval in self._update_intervals.values():
            interval.set("수신 간격: - ms")
        for value in self._sensor_values.values():
            value.set("-")
        self.tick_var.set("Tick: -")
        for leds in self._leds.values():
            for led in leds:
                led.set(None)

    def _reset_decoder(self) -> None:
        self._stream = FrameStream(recover_prefix=True)

    def _receive_bytes(self, chunk: bytes, received_at: float | None = None) -> None:
        """Receive COG only; CLI commands have no local receive-mode side effects."""
        for kind, value in self._stream.feed(chunk):
            if kind == "frame":
                self.process_encoded_frame(value, received_at)
            elif kind == "error":
                self._record_error(value)
            else:
                self._append_log(value, "cli")

    def process_encoded_frame(self, encoded: bytes, received_at: float | None = None) -> None:
        try:
            frame = decode_frame(encoded)
        except FrameError as error:
            self._record_error(str(error))
            return
        self._good_frames += 1
        if frame.message_type in self._update_intervals:
            now = time.monotonic() if received_at is None else received_at
            previous = self._last_frame_times.get(frame.message_type)
            self._last_frame_times[frame.message_type] = now
            interval = self._update_intervals[frame.message_type]
            interval.set("수신 간격: 측정 중" if previous is None
                         else f"수신 간격: {(now - previous) * 1000:.1f} ms")
        self._append_log("RX COG " + "\n".join(describe_frame(frame)), "ok")
        self._apply_frame(frame)
        self._update_stats()

    def _apply_frame(self, frame: CogFrame) -> None:
        try:
            if frame.message_type == 0x01:
                self._set_values(("Bus voltage", "Shunt current", "Battery SoC"), struct.unpack("<3f", frame.payload), ("V", "A", "%"))
            elif frame.message_type == 0x05:
                self._set_values(("Temperature", "Pressure"), struct.unpack("<2f", frame.payload), ("°C", "Pa"))
            elif frame.message_type == 0x07:
                self._set_values(("VMOT voltage", "VMOT current"), struct.unpack("<2f", frame.payload), ("V", "A"))
            elif frame.message_type == 0x08:
                tick_count, = struct.unpack("<I", frame.payload)
                self.tick_var.set(f"Tick: {tick_count}")
            elif frame.message_type == 0x02:
                self._set_leds("GPI", struct.unpack("<I", frame.payload)[0])
            elif frame.message_type == 0x03:
                for group, bits in zip(("Info", "Warning", "Critical"), struct.unpack("<3I", frame.payload)):
                    self._set_leds(group, bits)
        except struct.error:
            self._record_error(f"invalid payload length for {TYPE_NAMES.get(frame.message_type, 'unknown type')}")

    def _set_values(self, fields: tuple[str, ...], values: tuple[float, ...], units: tuple[str, ...]) -> None:
        for field, value, unit in zip(fields, values, units):
            self._sensor_values[field].set(f"{value:.3f} {unit}" if math.isfinite(value) else f"Unavailable ({unit})")

    def _set_leds(self, group: str, bits: int) -> None:
        for index, led in enumerate(self._leds[group]):
            led.set(bool(bits & (1 << index)))

    def _record_error(self, message: str) -> None:
        self._bad_frames += 1
        self._append_log(f"RX COG ERR {message}", "error")
        self._update_stats()

    def _append_log(self, message: str, tag: str) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"{time.strftime('%H:%M:%S')}  {message}\n", tag)
        line_count = int(self.log_text.index("end-1c").split(".")[0]) - 1
        if line_count > MAX_LOG_LINES:
            self.log_text.delete("1.0", f"{line_count - MAX_LOG_LINES + 1}.0")
        if self._follow_log:
            self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _update_stats(self) -> None:
        self.stats_var.set(f"Valid: {self._good_frames}   Invalid: {self._bad_frames}")

    def close(self) -> None:
        self.after_cancel(self._poll_id)
        self.disconnect()
        self.destroy()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="COBS/CRC COG log monitor")
    parser.add_argument("--cli", action="store_true", help="run the monitor in a terminal instead of the GUI")
    parser.add_argument("--port", help=f"serial port to connect on startup (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=921600, help="serial baud rate (default: 921600)")
    parser.add_argument("--stdin", action="store_true", help="with --cli, read binary COG data from standard input")
    parser.add_argument("--list-ports", action="store_true", help="list available serial ports and exit")
    arguments = parser.parse_args(argv)
    if arguments.baud <= 0:
        parser.error("--baud must be positive")
    if arguments.stdin and not arguments.cli:
        parser.error("--stdin requires --cli")
    if arguments.stdin and arguments.port:
        parser.error("choose either --stdin or --port")

    if sys.platform.startswith("linux") and os.geteuid() != 0:
        try:
            # Keep the interpreter, arguments, and display environment for Tk.
            os.execvp("sudo", ["sudo", "-E", sys.executable, str(Path(__file__).resolve()),
                               *(sys.argv[1:] if argv is None else argv)])
        except OSError as error:
            print(f"Unable to restart with sudo: {error}", file=sys.stderr)
            return 2

    if not arguments.stdin and arguments.port is None:
        arguments.port = DEFAULT_PORT

    if arguments.list_ports:
        if list_ports is None:
            parser.error("pyserial is required: python -m pip install pyserial")
        for available_port in list_ports.comports():
            print(f"{available_port.device}\t{available_port.description}")
    elif arguments.cli:
        return run_cli(arguments.port, arguments.baud, arguments.stdin)
    else:
        if tk is None:
            parser.error("Tk is required for the GUI; install Tk or use --cli")
        CogMonitor(arguments.port, arguments.baud).mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
