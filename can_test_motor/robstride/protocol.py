"""Wire formats from mmmcp.pdf; model ranges from RobStride ROS sample."""
import math
import struct
from dataclasses import dataclass


@dataclass(frozen=True)
class Profile:
    position: float = 4 * math.pi
    velocity: float = 44.0
    torque: float = 17.0
    kp: float = 500.0
    kd: float = 5.0


PROFILES = {"mmmcp": Profile(), "00": Profile(velocity=50),
            "01": Profile(), "02": Profile(),
            "03": Profile(velocity=50, torque=60, kp=5000, kd=100),
            "04": Profile(velocity=15, torque=120, kp=5000, kd=100),
            "05": Profile(velocity=33),
            "06": Profile(velocity=20, torque=36, kp=5000, kd=100)}
MODES = {"mit": 0, "pp": 1, "velocity": 2, "current": 3, "csp": 5}
BAUDRATES = {1000000: 1, 500000: 2, 250000: 3, 125000: 4}
FORMATS = {"float": "f", "uint8": "B", "uint16": "H", "uint32": "I"}


def finite(value, name="value"):
    if not math.isfinite(value):
        raise ValueError(f"{name}: 유한한 값이 필요합니다")
    return value


def bounded(value, low, high, name="value"):
    finite(value, name)
    if not low <= value <= high:
        raise ValueError(f"{name}: {low} .. {high} 범위가 필요합니다 (입력 {value})")
    return value


def byte(value):
    if not isinstance(value, int) or not 0 <= value <= 255:
        raise ValueError("CAN ID는 0..255 정수여야 합니다")
    return value


def pack16(value, low, high):
    return int((bounded(value, low, high) - low) * 65535 / (high - low))


def unpack16(value, low, high):
    return value * (high - low) / 65535 + low


@dataclass(frozen=True)
class Parameter:
    index: int
    name: str
    kind: str = "float"
    writable: bool = True
    description: str = ""

    def encode(self, value):
        finite(value)
        if self.kind != "float" and (not isinstance(value, int)):
            raise ValueError(f"{self.name}: {self.kind} 정수가 필요합니다")
        if self.name == "run_mode" and value not in MODES.values():
            raise ValueError("run_mode는 0, 1, 2, 3, 5만 지원합니다")
        if self.name in ("zero_sta", "damper") and value not in (0, 1):
            raise ValueError(f"{self.name}: 0 또는 1이 필요합니다")
        if self.name in ("cur_filt_gain", "spd_filt_gain"):
            bounded(value, 0, 1, self.name)
        if self.name in ("limit_torque", "limit_spd", "limit_cur", "cur_kp",
                         "cur_ki", "loc_kp", "spd_kp", "spd_ki", "acc_rad",
                         "vel_max", "acc_set") and value < 0:
            raise ValueError(f"{self.name}: 음수는 허용되지 않습니다")
        try:
            return struct.pack("<" + FORMATS[self.kind], value).ljust(4, b"\0")
        except (struct.error, OverflowError) as exc:
            raise ValueError(f"{self.name}: {self.kind} 표현 범위 초과") from exc

    def decode(self, data):
        fmt = "<" + FORMATS[self.kind]
        return struct.unpack(fmt, data[:struct.calcsize(fmt)])[0]


PARAMETERS = [
    Parameter(0x7005, "run_mode", "uint8", description="0 MIT / 1 PP / 2 velocity / 3 current / 5 CSP"),
    Parameter(0x7006, "iq_ref", description="전류 지령 A"),
    Parameter(0x700A, "spd_ref", description="속도 지령 rad/s"),
    Parameter(0x700B, "limit_torque", description="토크 제한 Nm"),
    Parameter(0x7010, "cur_kp", description="전류 루프 Kp"),
    Parameter(0x7011, "cur_ki", description="전류 루프 Ki"),
    Parameter(0x7014, "cur_filt_gain", description="전류 필터 gain"),
    Parameter(0x7016, "loc_ref", description="위치 지령 rad"),
    Parameter(0x7017, "limit_spd", description="CSP 속도 제한 rad/s"),
    Parameter(0x7018, "limit_cur", description="속도/위치 전류 제한 A"),
    Parameter(0x7019, "mechPos", writable=False, description="기계 위치 rad"),
    Parameter(0x701A, "iqf", writable=False, description="필터 전류 A"),
    Parameter(0x701B, "mechVel", writable=False, description="기계 속도 rad/s"),
    Parameter(0x701C, "VBUS", writable=False, description="버스 전압 V"),
    Parameter(0x701E, "loc_kp", description="위치 루프 Kp"),
    Parameter(0x701F, "spd_kp", description="속도 루프 Kp"),
    Parameter(0x7020, "spd_ki", description="속도 루프 Ki"),
    Parameter(0x7021, "spd_filt_gain", description="속도 필터 gain"),
    Parameter(0x7022, "acc_rad", description="속도 모드 가속도 rad/s²"),
    Parameter(0x7024, "vel_max", description="PP 속도 rad/s"),
    Parameter(0x7025, "acc_set", description="PP 가속도 rad/s²"),
    Parameter(0x7026, "EPScan_time", "uint16", description="보고 주기 selector (시간 단위 미명시)"),
    Parameter(0x7028, "canTimeout", "uint32", description="CAN timeout threshold (단위 미명시)"),
    Parameter(0x7029, "zero_sta", "uint8", description="전원 인가 위치 범위 0:0..2π / 1:-π..π"),
    Parameter(0x702A, "damper", "uint8", description="1: 전원 OFF 역구동 댐핑 해제 / 0: 기본"),
    Parameter(0x702B, "add_offset", description="논리 영점 offset rad"),
]
BY_INDEX = {p.index: p for p in PARAMETERS}
BY_NAME = {p.name.lower(): p for p in PARAMETERS}


def parameter(text):
    if str(text).lower() in BY_NAME:
        return BY_NAME[str(text).lower()]
    try:
        return BY_INDEX[int(str(text), 0)]
    except (KeyError, ValueError) as exc:
        raise ValueError(f"알 수 없는 파라미터: {text}") from exc


@dataclass(frozen=True)
class Frame:
    arbitration_id: int
    data: bytes = bytes(8)

    def __post_init__(self):
        if not 0 <= self.arbitration_id <= 0x1FFFFFFF or len(self.data) != 8:
            raise ValueError("MMMCP는 29-bit extended ID, 8-byte payload를 사용합니다")

    @property
    def command(self):
        return self.arbitration_id >> 24

    @property
    def source(self):
        return (self.arbitration_id >> 8) & 255

    @property
    def destination(self):
        return self.arbitration_id & 255


def request(command, motor, host=0xFD, data=bytes(8), extra=0):
    byte(motor)
    byte(host)
    byte(extra)
    if not 0 <= command <= 31:
        raise ValueError("command 범위는 0..31입니다")
    return Frame(command << 24 | extra << 16 | host << 8 | motor, data)


def parameter_request(motor, host, param, value=None, persistent=False):
    prefix = struct.pack("<HH", param.index, 0)
    if value is None:
        return request(0x11, motor, host, prefix + bytes(4))
    if not param.writable:
        raise ValueError(f"{param.name}은 읽기 전용입니다")
    return request(0x16 if persistent else 0x12, motor, host, prefix + param.encode(value))


def mit_request(motor, position, velocity, torque, kp, kd, profile=Profile()):
    packed = [pack16(position, -profile.position, profile.position),
              pack16(velocity, -profile.velocity, profile.velocity),
              pack16(kp, 0, profile.kp), pack16(kd, 0, profile.kd)]
    return Frame(1 << 24 | pack16(torque, -profile.torque, profile.torque) << 8 |
                 byte(motor), struct.pack(">4H", *packed))


STATUS_FAULTS = {0: "undervoltage", 1: "three_phase_overcurrent", 2: "overtemperature",
                 3: "magnetic_encoder", 4: "stall_overload", 5: "uncalibrated"}
DETAIL_FAULTS = {0: "motor_overtemperature", 1: "driver_chip", 2: "undervoltage",
                 3: "overvoltage", 4: "B_phase_overcurrent", 5: "C_phase_overcurrent",
                 7: "encoder_uncalibrated", 8: "hardware_identification",
                 9: "position_initialization", 14: "stall_I2t", 16: "A_phase_overcurrent"}


def bit_names(value, names):
    return [names.get(bit, f"unknown_bit_{bit}") for bit in range(32) if value & (1 << bit)]


def decode(frame, profile=Profile()):
    cmd, data, ident = frame.command, frame.data, frame.arbitration_id
    result = {"command": cmd, "motor_id": frame.source, "destination": frame.destination,
              "can_id": f"0x{ident:08X}", "data": data.hex(" ")}
    if cmd == 0 and frame.destination == 0xFE:
        result.update(kind="identity", uid=f"0x{int.from_bytes(data, 'big'):016X}")
    elif cmd in (2, 0x18, 0x1A):
        flags = (ident >> 16) & 63
        result.update(mode_status=(ident >> 22) & 3, faults=flags,
                      fault_names=bit_names(flags, STATUS_FAULTS))
        if cmd == 0x1A:
            result.update(kind="version", valid_signature=data[:3] == b"\x00\xc4\x56",
                          version=int.from_bytes(data[3:7], "big"))
        else:
            p, v, a, b = struct.unpack(">4H", data)
            result.update(kind="feedback" if cmd == 2 else "periodic",
                          position=unpack16(p, -profile.position, profile.position),
                          velocity=unpack16(v, -profile.velocity, profile.velocity))
            if cmd == 2:
                result.update(torque=unpack16(a, -profile.torque, profile.torque), temperature=b / 10)
            else:
                result.update(kp=unpack16(a, 0, profile.kp), kd=unpack16(b, 0, profile.kd))
    elif cmd == 0x11:
        index = int.from_bytes(data[:2], "little")
        param = BY_INDEX.get(index)
        result.update(kind="parameter", index=f"0x{index:04X}",
                      name=param.name if param else None, read_result=(ident >> 16) & 255,
                      value=param.decode(data[4:]) if param else data[4:].hex())
    elif cmd == 0x15:
        faults, warnings = struct.unpack("<II", data)
        result.update(kind="fault", faults=faults, warnings=warnings,
                      fault_names=bit_names(faults, DETAIL_FAULTS),
                      warning_names=bit_names(warnings, {0: "motor_overtemperature"}))
    else:
        result["kind"] = "unknown"
    return result
