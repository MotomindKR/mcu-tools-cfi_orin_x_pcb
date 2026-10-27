"""Correlated requests, diagnostics, and bounded motor tests."""
import math
import sys
import time

from .protocol import (BAUDRATES, MODES, PARAMETERS, Profile, bounded, byte, decode,
                       finite, mit_request, parameter, parameter_request, request)


POSITION_READBACK_TOLERANCE = 0.001  # rad; absolute tolerance, about 0.057 degrees


class MotorError(RuntimeError):
    pass


class Client:
    def __init__(self, transport, host=0xFD, timeout=0.2, profile=Profile(), trace=None):
        self.transport = transport
        self.host = byte(host)
        self.timeout = bounded(timeout, 0.005, 5, "timeout")
        self.profile = profile
        self.trace = trace
        self.guard = None

    def send(self, frame):
        self.transport.send(frame)
        if self.trace:
            self.trace.record("tx", frame)

    def receive(self, timeout):
        frame = self.transport.recv(timeout)
        if frame is None:
            return None
        if self.trace:
            self.trace.record("rx", frame)
        event = decode(frame, self.profile)
        if self.guard and event["motor_id"] == self.guard["motor"] and event["destination"] == self.host:
            if event["kind"] in ("feedback", "periodic", "version", "fault"):
                if event.get("faults") or event.get("warnings"):
                    raise MotorError(f"모터 fault/warning: {event}")
                if event.get("temperature", 0) > self.guard["temperature"]:
                    raise MotorError(f"온도 제한 초과: {event['temperature']} °C")
                if ("position" in event and self.guard["displacement"] is not None
                        and abs(event["position"] - self.guard["initial"]) > self.guard["displacement"]):
                    raise MotorError("위치 이동 제한 초과")
                if "velocity" in event and abs(event["velocity"]) > self.guard["velocity"] + 0.02:
                    raise MotorError("피드백 속도 제한 초과")
                if self.guard["running"] and "mode_status" in event and event["mode_status"] != 2:
                    raise MotorError(f"모터가 run 상태를 벗어났습니다: {event['mode_status']}")
        return event

    def drain(self):
        # Bound draining even if another node floods this bus.
        end = time.monotonic() + 0.01
        for _ in range(1024):
            if self.receive(0) is None:
                return
            if time.monotonic() >= end:
                raise MotorError("수신 버스가 너무 혼잡합니다; 요청을 중단합니다")

    def exchange(self, frame, commands, response_motor=None, index=None, timeout=None):
        self.drain()
        self.send(frame)
        target = frame.destination if response_motor is None else response_motor
        end = time.monotonic() + (self.timeout if timeout is None else timeout)
        while time.monotonic() < end:
            event = self.receive(max(0, end - time.monotonic()))
            if event is None:
                break
            destination = 0xFE if event["command"] == 0 else self.host
            if (event["command"] in commands and event["motor_id"] == target
                    and event["destination"] == destination):
                if index is not None and event.get("index") != f"0x{index:04X}":
                    continue
                return event
        raise TimeoutError(f"motor {target}, command 0x{frame.command:02X}: 응답 timeout")

    def identity(self, motor):
        return self.exchange(request(0, motor, self.host), (0,))

    def scan(self, start=0, end=255, timeout=0.02, progress=None):
        byte(start)
        byte(end)
        bounded(timeout, 0.001, 5, "scan timeout")
        if start > end:
            raise ValueError("scan start는 end보다 작아야 합니다")
        found = {}
        for motor in range(start, end + 1):
            try:
                event = self.exchange(request(0, motor, self.host), (0,), timeout=timeout)
                found[motor] = event
                if progress:
                    progress(event)
            except TimeoutError:
                pass
        return list(found.values())

    def read(self, motor, param):
        p = parameter(param)
        event = self.exchange(parameter_request(motor, self.host, p), (0x11,), index=p.index)
        if event["read_result"]:
            raise MotorError(f"{p.name}: 모터가 읽기 실패를 반환했습니다 ({event['read_result']})")
        finite(event["value"], p.name)
        return event

    def write(self, motor, param, value, persistent=False, verify=True):
        p = parameter(param)
        frame = parameter_request(motor, self.host, p, value, persistent)
        feedback = self.exchange(frame, (2,))
        if verify:
            actual = self.read(motor, p.name)["value"]
            expected = p.decode(p.encode(value))
            # Some firmware updates the stopped position reference from its
            # position estimate. Allow only a small absolute position error;
            # keep exact readback for limits, gains, modes and other registers.
            matches = (math.isclose(actual, expected, rel_tol=0,
                                    abs_tol=POSITION_READBACK_TOLERANCE)
                       if p.name == "loc_ref" else actual == expected)
            if not matches:
                raise MotorError(f"{p.name}: readback 불일치 ({actual} != {expected})")
        return feedback

    def stop(self, motor, clear=False):
        result = self.exchange(request(4, motor, self.host, bytes([int(clear)]) + bytes(7)), (2,))
        if result["mode_status"] != 0:
            raise MotorError(f"정지 후 idle 확인 실패: {result}")
        return result

    def emergency_stop(self, motor):
        # No receive or drain before stop: faults and missing ACK must not prevent TX.
        self.guard = None
        errors = []
        for _ in range(3):
            try:
                self.send(request(4, motor, self.host))
            except Exception as exc:
                errors.append(str(exc))
            time.sleep(0.005)
        if errors:
            print("정지 송신 오류: " + "; ".join(errors), file=sys.stderr)
        return not errors

    def version(self, motor):
        result = self.exchange(request(0x1A, motor, self.host, b"\x00\xc4" + bytes(6)), (0x1A,))
        if not result["valid_signature"]:
            raise MotorError(f"version signature 불일치: {result['data']}")
        return result

    def set_zero(self, motor):
        self.stop(motor)
        mode = self.read(motor, "run_mode")["value"]
        if mode not in (0, 5):
            raise MotorError("영점 설정은 MIT(0), CSP(5) 모드에서만 지원됩니다. run_mode를 먼저 설정하세요")
        return self.exchange(request(6, motor, self.host, b"\x01" + bytes(7)), (2,))

    def set_id(self, motor, new_id):
        byte(new_id)
        if not 1 <= new_id <= 252 or new_id == self.host or new_id == motor:
            raise ValueError("새 ID는 기존 ID/host와 다른 1..252 값을 사용하세요")
        uid = self.identity(motor)["uid"]
        try:
            self.identity(new_id)
        except TimeoutError:
            pass
        else:
            raise MotorError(f"ID {new_id}에 이미 모터가 있습니다")
        self.stop(motor)
        result = self.exchange(request(7, motor, self.host, extra=new_id), (0,), response_motor=new_id)
        if result["uid"] != uid or self.identity(new_id)["uid"] != uid:
            raise MotorError("ID 변경 후 UID가 일치하지 않습니다")
        return result

    def periodic(self, motor, enabled):
        return self.exchange(request(0x18, motor, self.host, bytes(6) + bytes([int(enabled), 0])), (0x18, 2))

    def baudrate(self, motor, bitrate):
        code = BAUDRATES[bitrate]
        self.stop(motor)
        return self.exchange(request(0x17, motor, self.host, bytes(6) + bytes([code, 0])), (2,))

    def protocol(self, motor, code):
        if code not in (0, 1, 2):
            raise ValueError("protocol code는 0, 1, 2입니다")
        self.stop(motor)
        return self.exchange(request(0x19, motor, self.host, bytes(6) + bytes([code, 0])), (0,))

    def monitor(self, motor=None, duration=5, callback=print):
        bounded(duration, 0.01, 86400, "duration")
        end = time.monotonic() + duration
        count = 0
        while time.monotonic() < end:
            event = self.receive(min(0.1, max(0, end - time.monotonic())))
            if event and (motor is None or event["motor_id"] == motor):
                # Identity is broadcast to FE; all other replies are host addressed.
                if event["destination"] == self.host or event["kind"] == "identity":
                    callback(event)
                    count += 1
        return count

    def diagnose(self, motor):
        report = {"motor_id": motor, "tests": [], "scope": "read-only; no enable/stop/write"}
        checks = [("identity", lambda: self.identity(motor)), ("version", lambda: self.version(motor))]
        checks += [(p.name, lambda p=p: self.read(motor, p.name)) for p in PARAMETERS]
        for name, check in checks:
            try:
                report["tests"].append({"test": name, "status": "PASS", "result": check()})
            except (TimeoutError, MotorError) as exc:
                report["tests"].append({"test": name, "status": "FAIL", "error": str(exc)})
        report["passed"] = all(t["status"] == "PASS" for t in report["tests"])
        return report

    def motion(self, motor, mode, *, duration=2, rate=20, position=None, relative=False,
               velocity=0, current=0, torque=0, kp=0, kd=0, speed_limit=0.5,
               acceleration=0.5, current_limit=0.5, torque_limit=0.5,
               max_current=1, max_velocity=1, max_torque=1, max_kp=20, max_kd=1,
               max_displacement=0.25, max_temperature=60, callback=None):
        """Setpoints and limits are initialized while stopped, before enable.

        Every cycle requires fresh command-2 feedback. finally attempts stop even
        when setup, enable, feedback, a signal or the output callback fails.
        """
        if mode not in MODES:
            raise ValueError(f"지원하지 않는 모드: {mode}")
        continuous = duration == 0 and mode == "velocity"
        if not continuous:
            bounded(duration, 0.01, 60, "duration (연속 velocity만 0 허용)")
        bounded(rate, 1, 100, "rate")
        for name, value in {"max_current": max_current, "max_velocity": max_velocity,
                            "max_torque": max_torque, "max_kp": max_kp, "max_kd": max_kd,
                            "max_displacement": max_displacement,
                            "max_temperature": max_temperature}.items():
            finite(value, name)
            if value <= 0:
                raise ValueError(f"{name}은 양수여야 합니다")
        bounded(velocity, -max_velocity, max_velocity, "velocity")
        bounded(current, -max_current, max_current, "current")
        bounded(torque, -max_torque, max_torque, "torque")
        bounded(kp, 0, min(max_kp, self.profile.kp), "kp")
        bounded(kd, 0, min(max_kd, self.profile.kd), "kd")
        bounded(speed_limit, 0.001, min(max_velocity, self.profile.velocity), "speed_limit")
        bounded(current_limit, 0.001, max_current, "current_limit")
        bounded(torque_limit, 0.001, min(max_torque, self.profile.torque), "torque_limit")
        bounded(acceleration, 0.001, 1000, "acceleration")
        if position is not None:
            finite(position, "position")
        self.identity(motor)
        count = 0
        try:
            stopped = self.stop(motor)
            if stopped["mode_status"] != 0 or stopped["faults"]:
                raise MotorError(f"시작 전 idle/무고장 상태 필요: {stopped}")
            initial = self.read(motor, "mechPos")["value"]
            target = initial if position is None else position + (initial if relative else 0)
            if mode in ("mit", "csp", "pp"):
                bounded(target - initial, -max_displacement, max_displacement, "position displacement")
            # MIT feedback is wrapped/scaled by the model range; prevent ambiguous guard values.
            if not continuous:
                bounded(initial, -self.profile.position, self.profile.position, "initial position")
            if mode == "mit":
                mit_request(motor, target, velocity, torque, kp, kd, self.profile)
            self.guard = {"motor": motor, "initial": initial, "temperature": max_temperature,
                          "displacement": None if continuous else max_displacement,
                          "velocity": max_velocity, "running": False}
            self.write(motor, "run_mode", MODES[mode])
            self.write(motor, "limit_torque", torque_limit)
            self.write(motor, "limit_cur", current_limit)
            # Clear current/speed references before run. Only position modes
            # use loc_ref; stopped firmware may update it from a noisy estimate.
            for name, value in (("iq_ref", 0.0), ("spd_ref", 0.0)):
                self.write(motor, name, value)
            if mode in ("csp", "pp"):
                self.write(motor, "loc_ref", initial)
            if mode == "velocity":
                self.write(motor, "acc_rad", acceleration)
            elif mode == "csp":
                self.write(motor, "limit_spd", speed_limit)
            elif mode == "pp":
                self.write(motor, "vel_max", speed_limit)
                self.write(motor, "acc_set", acceleration)
            elif mode == "mit":
                self.exchange(mit_request(motor, initial, 0, 0, 0, 0, self.profile), (2,))
            self.guard["running"] = True
            self.exchange(request(3, motor, self.host), (2,))
            start = time.monotonic()
            end = math.inf if continuous else start + duration
            next_tick = start
            while time.monotonic() < end:
                if mode == "mit":
                    event = self.exchange(mit_request(motor, target, velocity, torque, kp, kd, self.profile), (2,))
                else:
                    name, value = {"current": ("iq_ref", current), "velocity": ("spd_ref", velocity),
                                   "csp": ("loc_ref", target), "pp": ("loc_ref", target)}[mode]
                    event = self.write(motor, name, value, verify=False)
                count += 1
                if callback:
                    callback(event)
                next_tick += 1 / rate
                # Consume unsolicited faults during the inter-command interval.
                until = min(next_tick, end)
                while time.monotonic() < until:
                    self.receive(min(0.02, until - time.monotonic()))
            return {"motor_id": motor, "mode": mode, "cycles": count,
                    "initial_position": initial, "target_position": target, "duration": duration}
        finally:
            if not self.emergency_stop(motor):
                raise MotorError("정지 프레임 송신 실패. 하드웨어 비상 정지를 확인하세요")
            final_status = self.stop(motor)
            if final_status["mode_status"] != 0:
                raise MotorError(f"종료 후 idle 확인 실패: {final_status}")
