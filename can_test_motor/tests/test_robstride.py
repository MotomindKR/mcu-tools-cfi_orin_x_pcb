import contextlib
import io
import math
import struct
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from robstride.cli import main
from robstride.controller import Client, MotorError
from robstride.protocol import (PARAMETERS, PROFILES, Frame, decode, mit_request,
                               pack16, parameter, parameter_request, request)
from robstride.transport import CAN_FRAME, EFF, ERR, RTR, Simulator, SocketCAN, Trace


class ProtocolTests(unittest.TestCase):
    def test_pdf_read_loc_kp_vector(self):
        frame = parameter_request(0x7F, 0xFD, parameter("loc_kp"))
        self.assertEqual(frame.arbitration_id, 0x1100FD7F)
        self.assertEqual(frame.data.hex(), "1e70000000000000")
        event = decode(Frame(0x11007FFD, bytes.fromhex("1e7000000000f041")))
        self.assertEqual(event["value"], 30.0)

    def test_parameter_types_and_all_entries(self):
        self.assertEqual(len(PARAMETERS), 26)
        vectors = [("run_mode", 5, "05000000"), ("EPScan_time", 0x1234, "34120000"),
                   ("canTimeout", 0x12345678, "78563412"), ("add_offset", -0.5, "000000bf")]
        for name, value, expected in vectors:
            p = parameter(name)
            self.assertEqual(p.encode(value).hex(), expected)
            self.assertEqual(p.decode(p.encode(value)), value)
            self.assertEqual(parameter(hex(p.index)), p)

    def test_persistent_write_is_index_and_typed_value(self):
        frame = parameter_request(127, 253, parameter("zero_sta"), 1, persistent=True)
        self.assertEqual(frame.arbitration_id, 0x1600FD7F)
        self.assertEqual(frame.data.hex(), "2970000001000000")

    def test_mit_endpoints_and_midpoint(self):
        p = PROFILES["mmmcp"]
        frame = mit_request(1, -p.position, p.velocity, -p.torque, p.kp, p.kd)
        self.assertEqual(frame.arbitration_id, 0x01000001)
        self.assertEqual(frame.data.hex(), "0000ffffffffffff")
        self.assertEqual(pack16(0, -17, 17), 32767)
        self.assertEqual(mit_request(1, 0, 0, 17, 0, 0).arbitration_id, 0x01FFFF01)

    def test_model_ranges(self):
        self.assertEqual(PROFILES["04"].torque, 120)
        self.assertEqual(PROFILES["06"].kd, 100)
        with self.assertRaises(ValueError):
            mit_request(1, 0, 0, 18, 0, 0)
        mit_request(1, 0, 0, 18, 0, 0, PROFILES["03"])

    def test_feedback_and_periodic_are_distinct(self):
        frame_id = 2 << 24 | 2 << 22 | 0x21 << 16 | 127 << 8 | 253
        data = bytes.fromhex("0000ffffffff012c")
        event = decode(Frame(frame_id, data))
        self.assertEqual(event["position"], -4 * math.pi)
        self.assertEqual(event["velocity"], 44)
        self.assertEqual(event["torque"], 17)
        self.assertEqual(event["temperature"], 30)
        self.assertEqual(event["mode_status"], 2)
        self.assertEqual(event["fault_names"], ["undervoltage", "uncalibrated"])
        periodic = decode(Frame((frame_id & 0xFFFFFF) | 0x18 << 24, data))
        self.assertEqual(periodic["kp"], 500)
        self.assertNotIn("temperature", periodic)
        self.assertNotIn("torque", periodic)

    def test_detailed_fault_and_unknown_bits(self):
        event = decode(Frame(0x150001FD, struct.pack("<II", (1 << 16) | 1, 3)))
        self.assertEqual(event["fault_names"], ["motor_overtemperature", "A_phase_overcurrent"])
        self.assertEqual(event["warning_names"], ["motor_overtemperature", "unknown_bit_1"])

    def test_identity_and_version_byte_order(self):
        event = decode(Frame(0x7FFE, bytes.fromhex("0123456789abcdef")))
        self.assertEqual(event["uid"], "0x0123456789ABCDEF")
        event = decode(Frame(0x1A007FFD, bytes.fromhex("00c4560102030400")))
        self.assertTrue(event["valid_signature"])
        self.assertEqual(event["version"], 0x01020304)

    def test_invalid_values_are_rejected(self):
        for name, value in [("run_mode", 4), ("damper", 2), ("EPScan_time", -1),
                            ("canTimeout", 2**32), ("zero_sta", 1.0), ("limit_cur", -1),
                            ("cur_filt_gain", 2), ("iq_ref", float("nan")),
                            ("add_offset", float("inf")), ("iq_ref", 1e100)]:
            with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                parameter(name).encode(value)
        with self.assertRaises(ValueError):
            parameter_request(1, 253, parameter("mechPos"), 1)
        with self.assertRaises(ValueError):
            Frame(1, bytes(7))


class ClientTests(unittest.TestCase):
    def setUp(self):
        self.bus = Simulator()
        self.client = Client(self.bus, timeout=0.005)

    def test_scan_is_read_only_and_discovers_two_motors(self):
        found = self.client.scan(0, 127, timeout=0.001)
        self.assertEqual([m["motor_id"] for m in found], [1, 127])
        self.assertTrue(all(f.command == 0 for f in self.bus.sent))

    def test_diagnose_entire_parameter_table_without_mutations(self):
        report = self.client.diagnose(1)
        self.assertTrue(report["passed"])
        self.assertEqual(len(report["tests"]), 28)
        self.assertEqual({f.command for f in self.bus.sent}, {0, 0x11, 0x1A})

    def test_write_and_readback_both_storage_modes(self):
        for persistent in (False, True):
            self.client.write(1, "add_offset", 0.5, persistent)
            self.assertEqual(self.client.read(1, "add_offset")["value"], 0.5)
        self.assertEqual(self.bus.motors[1]["saved"][0x702B], 0.5)

    def test_id_collision_and_verified_change(self):
        with self.assertRaises(MotorError):
            self.client.set_id(1, 127)
        uid = self.client.identity(1)["uid"]
        self.assertEqual(self.client.set_id(1, 2)["uid"], uid)
        self.assertIn(2, self.bus.motors)
        self.assertNotIn(1, self.bus.motors)

    def test_config_command_wire_payloads(self):
        self.client.baudrate(1, 500000)
        self.assertEqual(next(f.data for f in self.bus.sent if f.command == 0x17).hex(), "0000000000000200")
        self.client.protocol(1, 2)
        self.assertEqual(self.bus.motors[1]["pending_protocol"], 2)
        self.client.periodic(1, True)
        received = []
        self.client.monitor(1, 0.03, received.append)
        self.assertTrue(any(e["kind"] == "periodic" for e in received))
        self.client.periodic(1, False)
        self.assertFalse(self.bus.motors[1]["periodic"])
        self.client.set_zero(1)
        self.assertEqual(self.client.read(1, "mechPos")["value"], 0)

    def test_zero_in_pp_rejected_without_zero_command(self):
        self.bus.motors[1]["values"][0x7005] = 1
        with self.assertRaises(MotorError):
            self.client.set_zero(1)
        self.assertNotIn(6, [f.command for f in self.bus.sent])

    def test_read_failure_and_wrong_parameter_not_accepted(self):
        original = self.bus.send
        def send(frame):
            if frame.command == 0x11:
                self.bus.queue.append(Frame(0x110001FD, bytes.fromhex("0570000000000000")))
                self.bus.queue.append(Frame(0x110101FD, frame.data))
            else:
                original(frame)
        self.bus.send = send
        with self.assertRaises(MotorError):
            self.client.read(1, "loc_kp")

    def test_wrong_source_host_and_stale_frames_are_ignored(self):
        self.bus.queue.append(Frame(0x110001FD, bytes.fromhex("1e7000000000f041")))
        original = self.bus.send
        def send(frame):
            self.bus.queue.append(Frame(0x110002FD, bytes.fromhex("1e7000000000f041")))
            self.bus.queue.append(Frame(0x110001FC, bytes.fromhex("1e7000000000f041")))
            original(frame)
        self.bus.send = send
        self.assertEqual(self.client.read(1, "loc_kp")["value"], 30)

    def test_write_readback_mismatch(self):
        original = self.bus.send
        def send(frame):
            original(frame)
            if frame.command == 0x12:
                self.bus.motors[1]["values"][0x701E] = 999.0
        self.bus.send = send
        with self.assertRaisesRegex(MotorError, "readback"):
            self.client.write(1, "loc_kp", 10)

    def test_all_five_modes_stop_and_initialize_before_enable(self):
        for mode in ("mit", "current", "velocity", "csp", "pp"):
            with self.subTest(mode=mode):
                self.bus.sent.clear()
                result = self.client.motion(1, mode, duration=0.01, position=0.05, relative=True)
                self.assertGreaterEqual(result["cycles"], 1)
                self.assertFalse(self.bus.motors[1]["running"])
                commands = [f.command for f in self.bus.sent]
                self.assertEqual(commands[-1], 4)
                writes = [int.from_bytes(f.data[:2], "little") for f in self.bus.sent[:commands.index(3)]
                          if f.command == 0x12]
                self.assertIn(0x7018, writes)
                self.assertIn(0x700B, writes)
                if mode in ("csp", "pp"):
                    self.assertIn(0x7016, writes)
                else:
                    self.assertNotIn(0x7016, writes)

    def test_velocity_and_current_run_despite_unstable_unused_loc_ref(self):
        original = self.bus.send
        def send(frame):
            original(frame)
            # Reproduce firmware changing loc_ref by more than its readback
            # tolerance while the actual mechanical position stays stable.
            self.bus.motors[1]["values"][0x7016] += 0.002
        self.bus.send = send
        for mode in ("velocity", "current"):
            with self.subTest(mode=mode):
                result = self.client.motion(1, mode, duration=0.01)
                self.assertGreater(result["cycles"], 0)
                self.assertFalse(self.bus.motors[1]["running"])

    def test_position_reference_allows_only_small_absolute_readback_error(self):
        original = self.bus.send
        offset = [0.000043]
        def send(frame):
            original(frame)
            if frame.command == 0x12:
                index = int.from_bytes(frame.data[:2], "little")
                self.bus.motors[1]["values"][index] += offset[0]
        self.bus.send = send
        for target, delta in ((1.46, 0.000043), (7.940566062927246, 0.00012826919555664062),
                              (7.94, 0.0008), (7.94, -0.0008)):
            with self.subTest(target=target, delta=delta):
                offset[0] = delta
                self.client.write(1, "loc_ref", target)
        with self.assertRaises(MotorError):
            self.client.write(1, "limit_cur", 0.3)
        for delta in (0.0012, -0.0012):
            with self.subTest(delta=delta), self.assertRaises(MotorError):
                offset[0] = delta
                self.client.write(1, "loc_ref", 7.94)

    def test_motion_timeout_attempts_stop(self):
        original = self.bus.send
        def send(frame):
            if frame.command == 1 and self.bus.motors[1]["running"]:
                self.bus.sent.append(frame)  # drop the motion ACK
            else:
                original(frame)
        self.bus.send = send
        with self.assertRaises(TimeoutError):
            self.client.motion(1, "mit", duration=0.01)
        self.assertFalse(self.bus.motors[1]["running"])
        self.assertEqual(self.bus.sent[-1].command, 4)

    def test_runtime_fault_and_overtemperature_stop(self):
        for fault in (True, False):
            self.bus = Simulator()
            self.client = Client(self.bus, timeout=0.005)
            original = self.bus.send
            def send(frame):
                original(frame)
                if frame.command == 1 and self.bus.motors[1]["running"]:
                    self.bus.queue.clear()
                    if fault:
                        self.bus.queue.append(Frame(0x150001FD, struct.pack("<II", 1, 0)))
                    else:
                        feedback = self.bus.feedback(1, self.bus.motors[1])
                        self.bus.queue.append(Frame(feedback.arbitration_id, feedback.data[:6] + struct.pack(">H", 700)))
            self.bus.send = send
            with self.subTest(fault=fault), self.assertRaises(MotorError):
                self.client.motion(1, "mit", duration=0.01)
            self.assertFalse(self.bus.motors[1]["running"])

    def test_keyboard_interrupt_in_callback_stops(self):
        def callback(event):
            raise KeyboardInterrupt
        with self.assertRaises(KeyboardInterrupt):
            self.client.motion(1, "mit", duration=0.01, callback=callback)
        self.assertFalse(self.bus.motors[1]["running"])

    def test_continuous_velocity_allows_multiple_turns_and_stops_on_interrupt(self):
        self.bus.motors[1]["values"][0x7019] = 20.0
        received = []
        def callback(event):
            received.append(event)
            if len(received) == 2:
                raise KeyboardInterrupt
        with self.assertRaises(KeyboardInterrupt):
            self.client.motion(1, "velocity", velocity=0.1, duration=0, callback=callback)
        self.assertEqual(len(received), 2)
        self.assertTrue(all(e["mode_status"] == 2 for e in received))
        self.assertFalse(self.bus.motors[1]["running"])
        self.assertEqual(self.bus.sent[-1].command, 4)

    def test_continuous_velocity_timeout_stops(self):
        original = self.bus.send
        def send(frame):
            if (frame.command == 0x12 and frame.data[:2] == b"\x0a\x70"
                    and self.bus.motors[1]["running"]):
                self.bus.sent.append(frame)
            else:
                original(frame)
        self.bus.send = send
        with self.assertRaises(TimeoutError):
            self.client.motion(1, "velocity", velocity=0.1, duration=0)
        self.assertFalse(self.bus.motors[1]["running"])

    def test_zero_duration_rejected_for_other_modes_before_bus_access(self):
        for mode in ("mit", "current", "pp", "csp"):
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                self.client.motion(1, mode, duration=0)
        self.assertEqual(self.bus.sent, [])

    def test_displacement_and_invalid_limits_prevent_enable(self):
        for kwargs in ({"position": 2}, {"current": 2}, {"max_current": float("nan")},
                       {"rate": 0}, {"duration": float("inf")}, {"kp": 21}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.client.motion(1, "mit", **kwargs)
        self.assertNotIn(3, [f.command for f in self.bus.sent])


class SocketTests(unittest.TestCase):
    def test_socketcan_send_native_can_frame(self):
        with patch("robstride.transport.socket.socket") as factory, patch("robstride.transport.select.select", return_value=([], [1], [])):
            sock = factory.return_value
            sock.send.return_value = 16
            bus = SocketCAN("can0")
            frame = request(0x11, 127, 253, bytes.fromhex("1e70000000000000"))
            bus.send(frame)
            self.assertEqual(sock.send.call_args.args[0], CAN_FRAME.pack(0x9100FD7F, 8, frame.data))
            sock.bind.assert_called_once_with(("can0",))
            bus.close()

    def test_non_extended_rtr_and_bad_dlc_filtered(self):
        with patch("robstride.transport.socket.socket") as factory, patch("robstride.transport.select.select", return_value=([1], [], [])):
            data = bytes(8)
            factory.return_value.recv.side_effect = [CAN_FRAME.pack(1, 8, data),
                CAN_FRAME.pack(EFF | RTR | 1, 8, data), CAN_FRAME.pack(EFF | 1, 7, data),
                CAN_FRAME.pack(EFF | 0x7FFE, 8, data)]
            bus = SocketCAN()
            self.assertEqual(bus.recv(0.1), Frame(0x7FFE, data))

    def test_can_error_fails_promptly(self):
        with patch("robstride.transport.socket.socket") as factory, patch("robstride.transport.select.select", return_value=([1], [], [])):
            factory.return_value.recv.return_value = CAN_FRAME.pack(ERR | 0x40, 8, bytes(8))
            with self.assertRaisesRegex(OSError, "CAN error"):
                SocketCAN().recv(0.1)


class CLITests(unittest.TestCase):
    def run_cli(self, args):
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            return main(args)

    def test_mutations_require_yes_before_opening_hardware(self):
        with patch("robstride.cli.SocketCAN") as factory:
            self.assertEqual(self.run_cli(["run", "--motor", "1"]), 1)
            factory.assert_not_called()

    def test_simulated_diagnosis_report_and_log(self):
        with tempfile.TemporaryDirectory() as folder:
            report, log = Path(folder) / "report.json", Path(folder) / "trace.jsonl"
            self.assertEqual(self.run_cli(["--simulate", "--log", str(log), "diagnose", "--motor", "1", "--report", str(report)]), 0)
            import json
            self.assertTrue(json.loads(report.read_text())["passed"])
            entries = [json.loads(line) for line in log.read_text().splitlines()]
            self.assertEqual({e["direction"] for e in entries}, {"tx", "rx"})

    def test_cli_motion_and_readonly_rejection(self):
        self.assertEqual(self.run_cli(["--simulate", "motion", "csp", "--motor", "1", "--position", "0.05", "--relative", "--duration", "0.01", "--yes"]), 0)
        self.assertEqual(self.run_cli(["--simulate", "write", "--motor", "1", "mechPos", "1", "--yes"]), 1)

    def test_menu_keeps_simulator_state(self):
        with patch("builtins.input", side_effect=["write --motor 1 loc_kp 10 --yes", "read --motor 1 loc_kp", "quit"]):
            self.assertEqual(self.run_cli(["--simulate", "menu"]), 0)


if __name__ == "__main__":
    unittest.main()
