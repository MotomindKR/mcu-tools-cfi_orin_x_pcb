"""Linux SocketCAN and deterministic, hardware-free protocol simulator."""
import json
import select
import socket
import struct
import time
from collections import deque

from .protocol import BY_INDEX, Frame, Profile, decode, pack16, unpack16

CAN_FRAME = struct.Struct("=IB3x8s")
EFF = 0x80000000
RTR = 0x40000000
ERR = 0x20000000
# Linux <linux/can/raw.h>; several CPython builds omit this socket constant.
CAN_RAW_ERR_FILTER = getattr(socket, "CAN_RAW_ERR_FILTER", 2)


class SocketCAN:
    def __init__(self, channel="can0"):
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            # Include error frames so bus-off and CAN errors fail requests promptly.
            self.sock.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
                                 struct.pack("=I", 0x1FFFFFFF))
            self.sock.bind((channel,))
            self.sock.setblocking(False)
        except BaseException:
            self.sock.close()
            raise

    def send(self, frame):
        if not select.select([], [self.sock], [], 0.2)[1]:
            raise TimeoutError("CAN 송신 timeout")
        sent = self.sock.send(CAN_FRAME.pack(frame.arbitration_id | EFF, 8, frame.data))
        if sent != CAN_FRAME.size:
            raise OSError("불완전한 CAN frame 송신")

    def recv(self, timeout):
        end = time.monotonic() + max(0, timeout)
        while True:
            if not select.select([self.sock], [], [], max(0, end - time.monotonic()))[0]:
                return None
            try:
                raw = self.sock.recv(CAN_FRAME.size)
            except BlockingIOError:
                continue
            if len(raw) != CAN_FRAME.size:
                continue
            ident, dlc, data = CAN_FRAME.unpack(raw)
            if ident & ERR:
                raise OSError(f"CAN error frame: 0x{ident & 0x1FFFFFFF:08X} {data.hex()}")
            if ident & EFF and not ident & RTR and dlc == 8:
                return Frame(ident & 0x1FFFFFFF, data)
            if time.monotonic() >= end:
                return None

    def close(self):
        self.sock.close()


class Trace:
    """JSONL captures raw and decoded frames; no output file unless requested."""
    def __init__(self, path=None, profile=Profile()):
        self.file = open(path, "a", encoding="utf-8") if path else None
        self.profile = profile

    def record(self, direction, frame):
        if self.file:
            details = decode(frame, self.profile) if direction == "rx" else {
                "command": frame.command, "target_motor": frame.destination,
                "can_id": f"0x{frame.arbitration_id:08X}", "data": frame.data.hex(" ")}
            self.file.write(json.dumps({"time_unix": time.time(), "direction": direction,
                                        **details}, ensure_ascii=False,
                                       allow_nan=False) + "\n")
            self.file.flush()

    def close(self):
        if self.file:
            self.file.close()


class Simulator:
    """Simulates two motors (1, 127). It is not a motor dynamics model."""
    def __init__(self, profile=Profile()):
        self.profile = profile
        self.queue = deque()
        self.motors = {}
        self.sent = []
        for ident in (1, 127):
            values = {index: 0 if p.kind != "float" else 0.0 for index, p in BY_INDEX.items()}
            values.update({0x7019: 0.25, 0x701C: 48.0, 0x701E: 30.0, 0x7026: 10})
            self.motors[ident] = {"uid": 0x524F425300000000 + ident, "values": values,
                                   "running": False, "periodic": False, "host": 0xFD,
                                   "faults": 0, "warnings": 0, "saved": {}}
        self.next_report = time.monotonic()

    def feedback(self, ident, motor, command=2):
        p = self.profile
        values = motor["values"]
        position = max(-p.position, min(p.position, values[0x7019]))
        velocity = values[0x701B] if motor["running"] else 0
        fields = [pack16(position, -p.position, p.position),
                  pack16(max(-p.velocity, min(p.velocity, velocity)), -p.velocity, p.velocity)]
        fields += [0, 0] if command == 0x18 else [pack16(0, -p.torque, p.torque), 250]
        can_id = command << 24 | (2 if motor["running"] else 0) << 22
        can_id |= (1 if motor["faults"] else 0) << 16 | ident << 8 | motor["host"]
        return Frame(can_id, struct.pack(">4H", *fields))

    def identity(self, ident, motor):
        return Frame(ident << 8 | 0xFE, motor["uid"].to_bytes(8, "big"))

    def send(self, frame):
        self.sent.append(frame)
        ident = frame.destination
        motor = self.motors.get(ident)
        if motor is None:
            return
        cmd, data = frame.command, frame.data
        if cmd != 1:
            motor["host"] = frame.source
        if cmd == 0:
            self.queue.append(self.identity(ident, motor))
            return
        if cmd == 7:
            new_id = (frame.arbitration_id >> 16) & 255
            self.motors[new_id] = self.motors.pop(ident)
            self.queue.append(self.identity(new_id, motor))
            return
        if cmd == 0x19:
            motor["pending_protocol"] = data[6]
            self.queue.append(self.identity(ident, motor))
            return
        if cmd == 0x1A:
            status = self.feedback(ident, motor).arbitration_id & 0xFFFFFF
            self.queue.append(Frame(0x1A << 24 | status, b"\x00\xc4\x56\x00\x01\x02\x03\x00"))
            return
        if cmd in (0x11, 0x12, 0x16):
            index = int.from_bytes(data[:2], "little")
            param = BY_INDEX.get(index)
            if cmd == 0x11:
                result = 0 if param else 1
                payload = param.encode(motor["values"][index]) if param else bytes(4)
                self.queue.append(Frame(0x11 << 24 | result << 16 | ident << 8 | motor["host"],
                                        data[:4] + payload))
                return
            if param and param.writable:
                value = param.decode(data[4:])
                motor["values"][index] = value
                if cmd == 0x16:
                    motor["saved"][index] = value
                if index == 0x7016 and motor["running"]:
                    motor["values"][0x7019] = value
                if index == 0x700A:
                    motor["values"][0x701B] = value
        elif cmd == 3:
            motor["running"] = not bool(motor["faults"])
        elif cmd == 4:
            motor["running"] = False
            motor["values"][0x701B] = 0.0
            if data[0] == 1:
                motor["faults"] = 0
                motor["warnings"] = 0
        elif cmd == 6:
            motor["values"][0x7019] = 0.0
        elif cmd == 0x17:
            motor["pending_baudrate"] = data[6]
        elif cmd == 0x18:
            motor["periodic"] = bool(data[6])
            self.queue.append(self.feedback(ident, motor, 0x18))
            return
        elif cmd == 1 and motor["running"]:
            pos = int.from_bytes(data[:2], "big")
            motor["values"][0x7019] = unpack16(pos, -self.profile.position, self.profile.position)
        self.queue.append(self.feedback(ident, motor))

    def recv(self, timeout):
        if self.queue:
            return self.queue.popleft()
        end = time.monotonic() + max(0, timeout)
        while time.monotonic() < end:
            if time.monotonic() >= self.next_report:
                self.next_report = time.monotonic() + 0.01
                for ident, motor in self.motors.items():
                    if motor["periodic"]:
                        self.queue.append(self.feedback(ident, motor, 0x18))
                if self.queue:
                    return self.queue.popleft()
            time.sleep(min(0.001, max(0, end - time.monotonic())))
        return None

    def close(self):
        pass
