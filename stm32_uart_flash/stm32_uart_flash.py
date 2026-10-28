#!/usr/bin/env python3
"""Flash a raw binary through the STM32 ROM USART bootloader (AN3155).

Jetson Orin Nano control signals:
  GPIO12 / PN.01 -> STM32 BOOT
  GPIO13 / PH.00 -> STM32 nRESET (active low)

The default serial port is /dev/ttyTHS1 and the default flash address is
0x08000000. Both libgpiod Python API v1 and v2 are supported.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import struct
import sys
import time
from typing import Optional

try:
    import gpiod
except ImportError:
    print(
        "Error: install the GPIO binding with: "
        "sudo apt update"
        "sudo apt install python3-libgpiod python3-serial",
        file=sys.stderr,
    )
    raise SystemExit(2)

try:
    import serial
except ImportError:
    print(
        "Error: install pyserial with: sudo apt install python3-serial"
        "sudo apt update"
        "sudo apt install python3-libgpiod python3-serial",
        file=sys.stderr,
    )
    raise SystemExit(2)


GPIO_CHIP_LABEL = "tegra234-gpio"
BOOT_OFFSET = 85      # GPIO12 / PN.01 (chip-local line offset)
RESET_OFFSET = 43     # GPIO13 / PH.00 (chip-local line offset)

ACK = 0x79
NACK = 0x1F
CMD_GET = 0x00
CMD_GET_ID = 0x02
CMD_READ_MEMORY = 0x11
CMD_WRITE_MEMORY = 0x31
CMD_ERASE = 0x43
CMD_EXTENDED_ERASE = 0x44
MAX_BLOCK = 256


class FlashError(RuntimeError):
    """Expected STM32 bootloader operation failed."""


def find_gpiochip(label: str) -> str:
    """Find /dev/gpiochipN by sysfs label or device-tree compatible."""
    compatible = f"nvidia,{label}".encode()
    for root in (Path("/sys/bus/gpio/devices"), Path("/sys/class/gpio")):
        for device in sorted(root.glob("gpiochip*")):
            try:
                label_file = device / "label"
                compatible_file = device / "of_node" / "compatible"
                label_matches = (
                    label_file.exists() and label_file.read_text().strip() == label
                )
                compatible_matches = (
                    compatible_file.exists()
                    and compatible in compatible_file.read_bytes().split(b"\0")
                )
                if label_matches or compatible_matches:
                    path = Path("/dev") / device.name
                    if path.exists():
                        return str(path)
            except OSError:
                continue
    raise FlashError(f"GPIO controller '{label}' was not found")


class Stm32ControlPins:
    """Own and drive the BOOT and active-low RESET GPIO lines."""

    def __init__(self, chip_path: str) -> None:
        self.chip_path = chip_path
        self.boot = 0
        self.reset = 1
        self.api_v2 = hasattr(gpiod, "request_lines")
        self.request = None
        self.chip = None
        self.lines = None

    def __enter__(self) -> "Stm32ControlPins":
        if self.api_v2:
            from gpiod.line import Direction, Value

            self._v2_value = Value
            self.request = gpiod.request_lines(
                self.chip_path,
                consumer="stm32-uart-flash",
                config={
                    BOOT_OFFSET: gpiod.LineSettings(
                        direction=Direction.OUTPUT,
                        output_value=Value.INACTIVE,
                    ),
                    RESET_OFFSET: gpiod.LineSettings(
                        direction=Direction.OUTPUT,
                        output_value=Value.ACTIVE,
                    ),
                },
            )
        else:
            self.chip = gpiod.Chip(self.chip_path)
            self.lines = self.chip.get_lines([BOOT_OFFSET, RESET_OFFSET])
            self.lines.request(
                consumer="stm32-uart-flash",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[0, 1],
            )
        return self

    def set(self, *, boot: Optional[int] = None, reset: Optional[int] = None) -> None:
        if boot is not None:
            self.boot = int(bool(boot))
        if reset is not None:
            self.reset = int(bool(reset))

        if self.api_v2:
            value = self._v2_value
            self.request.set_values(
                {
                    BOOT_OFFSET: value.ACTIVE if self.boot else value.INACTIVE,
                    RESET_OFFSET: value.ACTIVE if self.reset else value.INACTIVE,
                }
            )
        else:
            self.lines.set_values([self.boot, self.reset])

    def enter_bootloader(self) -> None:
        """Latch BOOT high by pulsing nRESET low then high."""
        self.set(boot=1, reset=1)
        time.sleep(0.05)
        self.set(reset=0)
        time.sleep(0.10)
        self.set(reset=1)
        time.sleep(0.20)

    def start_application(self) -> None:
        """Latch BOOT low by pulsing nRESET low then high."""
        self.set(boot=0, reset=1)
        time.sleep(0.05)
        self.set(reset=0)
        time.sleep(0.10)
        self.set(reset=1)
        time.sleep(0.20)

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        try:
            # Do not leave the target intentionally held in reset.
            self.set(boot=0, reset=1)
        finally:
            if self.api_v2:
                if self.request is not None:
                    self.request.release()
            else:
                if self.lines is not None:
                    self.lines.release()
                if self.chip is not None:
                    self.chip.close()


class Stm32Bootloader:
    def __init__(self, uart: serial.Serial, timeout: float, erase_timeout: float) -> None:
        self.uart = uart
        self.timeout = timeout
        self.erase_timeout = erase_timeout

    @staticmethod
    def xor(data: bytes) -> int:
        result = 0
        for byte in data:
            result ^= byte
        return result

    def read_exact(self, length: int, timeout: Optional[float] = None) -> bytes:
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        result = bytearray()
        old_timeout = self.uart.timeout
        try:
            while len(result) < length:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self.uart.timeout = remaining
                result.extend(self.uart.read(length - len(result)))
        finally:
            self.uart.timeout = old_timeout
        if len(result) != length:
            raise FlashError(
                f"UART timeout: expected {length} byte(s), received {len(result)}"
            )
        return bytes(result)

    def wait_ack(self, operation: str, timeout: Optional[float] = None) -> None:
        response = self.read_exact(1, timeout)[0]
        if response == ACK:
            return
        if response == NACK:
            raise FlashError(f"STM32 returned NACK during {operation}")
        raise FlashError(
            f"unexpected response 0x{response:02X} during {operation}"
        )

    def send_command(self, command: int, operation: str) -> None:
        self.uart.write(bytes((command, command ^ 0xFF)))
        self.uart.flush()
        self.wait_ack(operation)

    def send_address(self, address: int, operation: str) -> None:
        address_bytes = struct.pack(">I", address)
        self.uart.write(address_bytes + bytes((self.xor(address_bytes),)))
        self.uart.flush()
        self.wait_ack(operation)

    def synchronize(self) -> None:
        self.uart.reset_input_buffer()
        self.uart.write(b"\x7F")
        self.uart.flush()
        self.wait_ack("bootloader synchronization")

    def get_commands(self) -> tuple[int, set[int]]:
        self.send_command(CMD_GET, "Get command")
        count_minus_one = self.read_exact(1)[0]
        payload = self.read_exact(count_minus_one + 1)
        self.wait_ack("Get command completion")
        if not payload:
            raise FlashError("STM32 returned an empty command list")
        return payload[0], set(payload[1:])

    def get_id(self) -> bytes:
        self.send_command(CMD_GET_ID, "Get ID command")
        count_minus_one = self.read_exact(1)[0]
        product_id = self.read_exact(count_minus_one + 1)
        self.wait_ack("Get ID completion")
        return product_id

    def mass_erase(self, commands: set[int]) -> str:
        if CMD_EXTENDED_ERASE in commands:
            self.send_command(CMD_EXTENDED_ERASE, "Extended Erase command")
            # 0xFFFF is mass erase; checksum of FF XOR FF is 00.
            self.uart.write(b"\xFF\xFF\x00")
            self.uart.flush()
            self.wait_ack("mass erase", self.erase_timeout)
            return "extended mass erase (0x44)"
        if CMD_ERASE in commands:
            self.send_command(CMD_ERASE, "Erase command")
            # 0xFF is mass erase; checksum is 00.
            self.uart.write(b"\xFF\x00")
            self.uart.flush()
            self.wait_ack("mass erase", self.erase_timeout)
            return "legacy mass erase (0x43)"
        raise FlashError("this bootloader does not advertise an erase command")

    def write_memory(self, address: int, data: bytes) -> None:
        if not 1 <= len(data) <= MAX_BLOCK or len(data) % 4:
            raise ValueError("write block must be 4..256 bytes and 4-byte aligned")
        self.send_command(CMD_WRITE_MEMORY, "Write Memory command")
        self.send_address(address, "Write Memory address")
        length_minus_one = len(data) - 1
        body = bytes((length_minus_one,)) + data
        self.uart.write(body + bytes((self.xor(body),)))
        self.uart.flush()
        self.wait_ack(f"write at 0x{address:08X}", max(self.timeout, 5.0))

    def read_memory(self, address: int, length: int) -> bytes:
        if not 1 <= length <= MAX_BLOCK:
            raise ValueError("read block must be 1..256 bytes")
        self.send_command(CMD_READ_MEMORY, "Read Memory command")
        self.send_address(address, "Read Memory address")
        length_minus_one = length - 1
        self.uart.write(bytes((length_minus_one, length_minus_one ^ 0xFF)))
        self.uart.flush()
        self.wait_ack(f"read at 0x{address:08X}")
        return self.read_exact(length)


def format_commands(commands: set[int]) -> str:
    return " ".join(f"0x{command:02X}" for command in sorted(commands))


def show_progress(label: str, completed: int, total: int) -> None:
    percent = completed * 100.0 / total
    print(f"\r{label}: {completed}/{total} bytes ({percent:5.1f}%)", end="", flush=True)
    if completed == total:
        print()


def program_firmware(
    bootloader: Stm32Bootloader,
    firmware: bytes,
    address: int,
    commands: set[int],
    *,
    erase: bool,
    verify: bool,
) -> None:
    required = {CMD_WRITE_MEMORY}
    if verify:
        required.add(CMD_READ_MEMORY)
    missing = required - commands
    if missing:
        raise FlashError(
            "required command(s) not supported: " + format_commands(missing)
        )

    if erase:
        print("Erasing STM32 flash...", flush=True)
        method = bootloader.mass_erase(commands)
        print(f"Erase complete using {method}.")
    else:
        print("Erase skipped by request.")

    total = len(firmware)
    for offset in range(0, total, MAX_BLOCK):
        original = firmware[offset : offset + MAX_BLOCK]
        padding = (-len(original)) % 4
        block = original + (b"\xFF" * padding)
        bootloader.write_memory(address + offset, block)
        show_progress("Writing", min(offset + len(original), total), total)

    if verify:
        for offset in range(0, total, MAX_BLOCK):
            expected = firmware[offset : offset + MAX_BLOCK]
            actual = bootloader.read_memory(address + offset, len(expected))
            if actual != expected:
                mismatch = next(
                    index
                    for index, (left, right) in enumerate(zip(expected, actual))
                    if left != right
                )
                bad_address = address + offset + mismatch
                raise FlashError(
                    f"verify failed at 0x{bad_address:08X}: "
                    f"file=0x{expected[mismatch]:02X}, flash=0x{actual[mismatch]:02X}"
                )
            show_progress("Verifying", min(offset + len(expected), total), total)


def integer(value: str) -> int:
    try:
        return int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid integer: {value}") from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Flash a raw .bin file through the STM32 ROM UART bootloader"
    )
    parser.add_argument(
        "firmware",
        nargs="?",
        type=Path,
        default=Path(__file__).resolve().with_name("cfi_orin_x_app.bin"),
        help="raw firmware .bin file (default: cfi_orin_x_app.bin beside this script)",
    )
    parser.add_argument("--port", default="/dev/ttyTHS1", help="UART device")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate")
    parser.add_argument(
        "--address",
        type=integer,
        default=0x08000000,
        help="flash start address (default: 0x08000000)",
    )
    parser.add_argument(
        "--parity",
        choices=("even", "none"),
        default="even",
        help="UART parity (default: even; use none only when required by AN2606)",
    )
    parser.add_argument("--timeout", type=float, default=2.0, help="UART timeout")
    parser.add_argument(
        "--erase-timeout",
        type=float,
        default=120.0,
        help="mass-erase timeout in seconds",
    )
    parser.add_argument(
        "--sync-retries", type=int, default=3, help="boot/reset synchronization attempts"
    )
    parser.add_argument("--skip-erase", action="store_true", help="do not erase flash")
    parser.add_argument("--no-verify", action="store_true", help="skip read-back verify")
    parser.add_argument(
        "--no-run",
        action="store_true",
        help="do not reset into the application after successful programming",
    )
    parser.add_argument("--chip", help="GPIO device override, e.g. /dev/gpiochip1")
    args = parser.parse_args()

    if args.baud <= 0:
        parser.error("--baud must be greater than zero")
    if args.timeout <= 0 or args.erase_timeout <= 0:
        parser.error("timeouts must be greater than zero")
    if args.sync_retries <= 0:
        parser.error("--sync-retries must be greater than zero")
    if not 0 <= args.address <= 0xFFFFFFFF or args.address % 4:
        parser.error("--address must be a 4-byte-aligned 32-bit address")
    if not args.firmware.is_file():
        parser.error(f"firmware file not found: {args.firmware}")
    return args


def main() -> int:
    args = parse_args()
    try:
        if sys.platform.startswith("linux") and os.geteuid() != 0:
            os.execvp(
                "sudo",
                ["sudo", "-E", sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]],
            )

        firmware = args.firmware.read_bytes()
        if not firmware:
            raise FlashError("firmware file is empty")
        if args.address + len(firmware) - 1 > 0xFFFFFFFF:
            raise FlashError("firmware exceeds the 32-bit address space")

        chip_path = args.chip or find_gpiochip(GPIO_CHIP_LABEL)
        parity = serial.PARITY_EVEN if args.parity == "even" else serial.PARITY_NONE
        print(
            f"Firmware : {args.firmware} ({len(firmware)} bytes)\n"
            f"Target   : {args.port} at {args.baud} baud, "
            f"8{args.parity[0].upper()}1\n"
            f"Address  : 0x{args.address:08X}\n"
            f"GPIO     : {chip_path}, BOOT line {BOOT_OFFSET}, "
            f"nRESET line {RESET_OFFSET}",
            flush=True,
        )

        with Stm32ControlPins(chip_path) as pins:
            uart = None
            bootloader = None
            last_error = None

            try:
                for attempt in range(1, args.sync_retries + 1):
                    print(
                        f"Entering system bootloader (attempt {attempt}/"
                        f"{args.sync_retries}); reopening UART...",
                        flush=True,
                    )
                    uart = serial.Serial(
                        port=args.port,
                        baudrate=args.baud,
                        bytesize=serial.EIGHTBITS,
                        parity=parity,
                        stopbits=serial.STOPBITS_ONE,
                        timeout=args.timeout,
                        write_timeout=args.timeout,
                        exclusive=True,
                    )
                    bootloader = Stm32Bootloader(
                        uart, args.timeout, args.erase_timeout
                    )

                    try:
                        # Open and configure the host UART before resetting the target.
                        # This makes any host-side UART startup transient occur before
                        # the STM32 ROM bootloader starts its auto-baud detection.
                        pins.enter_bootloader()
                        bootloader.synchronize()
                        last_error = None
                        break
                    except FlashError as exc:
                        last_error = exc
                        uart.close()
                        uart = None

                if last_error is not None:
                    raise FlashError(
                        f"unable to synchronize after {args.sync_retries} attempts: "
                        f"{last_error}"
                    )
                if uart is None or bootloader is None:
                    raise FlashError("UART synchronization ended without a connection")

                version, commands = bootloader.get_commands()
                product_id = bootloader.get_id()
                print(
                    f"Connected: bootloader v{version >> 4}.{version & 0x0F}, "
                    f"product ID 0x{product_id.hex().upper()}\n"
                    f"Commands : {format_commands(commands)}"
                )

                program_firmware(
                    bootloader,
                    firmware,
                    args.address,
                    commands,
                    erase=not args.skip_erase,
                    verify=not args.no_verify,
                )

                if args.no_run:
                    pins.set(boot=0, reset=1)
                    print("Programming complete; application reset skipped.")
                else:
                    print("Programming complete. Resetting into the application...")
                    pins.start_application()
            finally:
                if uart is not None:
                    uart.close()
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted; BOOT=0 and nRESET=1 restored.", file=sys.stderr)
        return 130
    except (FlashError, OSError, serial.SerialException) as exc:
        print(f"\nError: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
