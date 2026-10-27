"""Korean CLI and interactive command menu, with explicit write authorization."""
import argparse
import json
import shlex
import signal
import sys
from pathlib import Path

from .controller import Client, MotorError
from .protocol import BAUDRATES, MODES, PARAMETERS, PROFILES, bounded, byte, parameter
from .transport import Simulator, SocketCAN, Trace


def integer(text):
    return int(text, 0)


def can_id(text):
    try:
        return byte(integer(text))
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def output(value):
    print(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))


def parser():
    p = argparse.ArgumentParser(description="RobStride MMMCP CAN 스캔/시험 도구 (기본 can0, 1 Mbps)")
    p.add_argument("--channel", default="can0", help="SocketCAN 인터페이스")
    p.add_argument("--host-id", type=can_id, default=0xFD)
    p.add_argument("--model", choices=PROFILES, default="mmmcp", help="MIT/피드백 변환 범위, 자동 감지 안 됨")
    p.add_argument("--timeout", type=float, default=0.2, help="응답 timeout 초")
    p.add_argument("--simulate", action="store_true", help="실제 CAN 접근 없이 ID 1,127 모의 시험")
    p.add_argument("--log", metavar="JSONL", help="TX/RX 프레임 기록")
    sub = p.add_subparsers(dest="command", required=True)
    sub.add_parser("menu", help="대화형 메뉴 및 명령 프롬프트")
    sub.add_parser("params", help="PDF 파라미터 전체 목록")
    s = sub.add_parser("scan", help="ID 0 요청만으로 연결 모터 검색")
    s.add_argument("--start", type=can_id, default=0)
    s.add_argument("--end", type=can_id, default=255)
    s.add_argument("--scan-timeout", type=float, default=0.02)
    for name, help_text in (("id", "장치 ID/UID 조회"), ("version", "펌웨어 버전 조회"),
                            ("read", "단일/전체 파라미터 조회"), ("diagnose", "읽기 전용 전체 진단"),
                            ("stop", "모터 정지"), ("clear-fault", "정지 및 오류 초기화"),
                            ("zero", "현재 위치를 기계 영점으로 설정"),
                            ("set-id", "CAN ID 변경 및 UID 검증"),
                            ("write", "파라미터 쓰기 및 readback 검증"),
                            ("baudrate", "재기동 후 적용할 motor bitrate 변경"),
                            ("protocol", "deprecated 프로토콜 변경"),
                            ("periodic", "주기 보고 설정"),
                            ("motion", "MIT/current/velocity/CSP/PP 유한 시간 구동"),
                            ("run", "MIT 영 지령으로 제한 시간 enable 시험")):
        s = sub.add_parser(name, help=help_text)
        s.add_argument("--motor", type=can_id, required=True, help="대상 ID (10진/0x16진)")
        if name in ("clear-fault", "zero", "set-id", "write", "baudrate", "protocol", "periodic", "motion", "run"):
            s.add_argument("--yes", action="store_true", help="표시된 변경/구동 동작 승인")
        if name == "read":
            s.add_argument("parameter", help="이름/0x주소/all")
        elif name == "write":
            s.add_argument("parameter")
            s.add_argument("value")
            s.add_argument("--persistent", action="store_true", help="0x16 저장; 전원 재인가 유지 시험은 별도")
        elif name == "set-id":
            s.add_argument("new_id", type=can_id)
        elif name == "baudrate":
            s.add_argument("bitrate", type=int, choices=BAUDRATES)
        elif name == "protocol":
            s.add_argument("protocol_name", choices=("mmmcp", "canopen", "mit"))
        elif name == "periodic":
            s.add_argument("state", choices=("on", "off"))
        elif name == "diagnose":
            s.add_argument("--report", type=Path, help="진단 JSON 저장")
        elif name in ("motion", "run"):
            if name == "motion":
                s.add_argument("mode", choices=MODES)
            s.add_argument("--duration", type=float, default=2,
                           help="0.01..60초; velocity는 0이면 Ctrl+C까지 연속 회전, 종료 시 정지")
            s.add_argument("--rate", type=float, default=20, help="1..100 Hz (Python 비실시간)")
            if name == "motion":
                s.add_argument("--position", type=float, help="기본 현재 위치 유지, 단위 rad")
                s.add_argument("--relative", action="store_true", help="position을 현재 위치 기준 증분으로 해석")
                for flag in ("velocity", "current", "torque", "kp", "kd"):
                    s.add_argument("--" + flag, type=float, default=0)
                for flag in ("speed-limit", "acceleration", "current-limit", "torque-limit"):
                    s.add_argument("--" + flag, type=float, default=0.5)
            for flag, default in (("max-current", 1), ("max-velocity", 1), ("max-torque", 1),
                                  ("max-kp", 20), ("max-kd", 1), ("max-displacement", 0.25),
                                  ("max-temperature", 60)):
                s.add_argument("--" + flag, type=float, default=default, help=f"소프트웨어 시험 제한, 기본 {default}")
    s = sub.add_parser("monitor", help="수신 전용: 일반/주기/고장 프레임 디코딩")
    s.add_argument("--motor", type=can_id)
    s.add_argument("--duration", type=float, default=5)
    return p


def authorize(args):
    if hasattr(args, "yes") and not args.yes:
        raise ValueError("변경/구동 명령입니다. 대상과 옵션을 확인한 뒤 --yes를 추가하세요")


def dispatch(args, client):
    cmd = args.command
    authorize(args)
    motor = getattr(args, "motor", None)
    if cmd == "scan":
        found = client.scan(args.start, args.end, args.scan_timeout)
        output({"motors": found, "count": len(found)})
        return 0 if found else 1
    if cmd == "id":
        result = client.identity(motor)
    elif cmd == "version":
        result = client.version(motor)
    elif cmd == "read":
        if args.parameter.lower() == "all":
            result = []
            for p in PARAMETERS:
                try:
                    result.append(client.read(motor, p.name))
                except (MotorError, TimeoutError) as exc:
                    result.append({"parameter": p.name, "error": str(exc)})
            output(result)
            return int(any("error" in entry for entry in result))
        result = client.read(motor, args.parameter)
    elif cmd == "diagnose":
        result = client.diagnose(motor)
        if args.report:
            args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        output(result)
        return 0 if result["passed"] else 1
    elif cmd == "stop":
        # Emergency transmission comes first, including on a faulted/flooded bus.
        if not client.emergency_stop(motor):
            raise MotorError("정지 송신 실패")
        result = client.stop(motor)
        if result["mode_status"] != 0:
            raise MotorError(f"idle 확인 실패: {result}")
    elif cmd == "clear-fault":
        result = client.stop(motor, clear=True)
        if result["faults"]:
            raise MotorError(f"고장 원인이 남아 있습니다: {result}")
    elif cmd == "zero":
        result = client.set_zero(motor)
    elif cmd == "set-id":
        result = client.set_id(motor, args.new_id)
    elif cmd == "write":
        p = parameter(args.parameter)
        value = float(args.value) if p.kind == "float" else integer(args.value)
        p.encode(value)
        if not p.writable:
            raise ValueError(f"{p.name}은 읽기 전용입니다")
        client.stop(motor)
        result = client.write(motor, p.name, value, args.persistent)
        result = {"write": p.name, "value": value, "persistent_requested": args.persistent,
                  "readback_verified": True, "feedback": result}
    elif cmd == "baudrate":
        result = {"feedback": client.baudrate(motor, args.bitrate), "pending_bitrate": args.bitrate,
                  "note": "모터 전원 재인가 후 적용. 호스트 can0 bitrate는 별도 변경해야 합니다"}
    elif cmd == "protocol":
        result = {"identity": client.protocol(motor, ("mmmcp", "canopen", "mit").index(args.protocol_name)),
                  "pending_protocol": args.protocol_name,
                  "note": "Deprecated. 재기동 후 적용. CANopen/MIT로 변경하면 이 MMMCP 도구로 통신할 수 없습니다"}
    elif cmd == "periodic":
        result = client.periodic(motor, args.state == "on")
    elif cmd == "monitor":
        count = client.monitor(motor, args.duration, output)
        result = {"received": count, "note": "수신 전용; 프레임을 얻으려면 별도로 periodic on 설정"}
    elif cmd in ("motion", "run"):
        options = {k: v for k, v in vars(args).items() if k in (
            "duration", "rate", "position", "relative", "velocity", "current", "torque", "kp", "kd",
            "speed_limit", "acceleration", "current_limit", "torque_limit", "max_current", "max_velocity",
            "max_torque", "max_kp", "max_kd", "max_displacement", "max_temperature")}
        if cmd == "motion":
            # Reject ignored setpoints rather than silently accepting user mistakes.
            allowed = {"mit": {"position", "velocity", "torque", "kp", "kd"},
                       "current": {"current"}, "velocity": {"velocity"},
                       "csp": {"position"}, "pp": {"position"}}[args.mode]
            for name in ("position", "velocity", "current", "torque", "kp", "kd"):
                if name not in allowed and getattr(args, name) not in (None, 0):
                    raise ValueError(f"{args.mode} 모드에서 --{name}은 사용하지 않습니다")
            if args.relative and args.position is None:
                raise ValueError("--relative에는 --position이 필요합니다")
        result = client.motion(motor, args.mode if cmd == "motion" else "mit", callback=output, **options)
    else:
        raise ValueError(f"알 수 없는 명령: {cmd}")
    output(result)
    return 0


MENU = """
RobStride 시험 메뉴 (명령 뒤 --help로 상세 옵션)
  1 scan                 연결 모터 검색
  2 id --motor ID         장치 ID / MCU UID
  3 diagnose --motor ID   버전 + 파라미터 26개 진단
  4 read --motor ID all   전체 파라미터
  5 monitor              피드백/고장 수신
  6 stop --motor ID       정지
  params                 파라미터 목록
  run, motion            enable / MIT,current,velocity,csp,pp 시험
  write, clear-fault, zero, set-id, baudrate, periodic, protocol
  예: motion csp --motor 1 --position 0.1 --relative --duration 2 --yes
  예: motion velocity --motor 1 --velocity 0.1 --duration 0 --yes
  예: write --motor 1 EPScan_time 10 --yes
  help / quit
변경/구동에는 --yes 필요. 구동 종료 및 Ctrl+C/SIGTERM 시 정지 송신.
"""


def menu(args, client):
    print(MENU)
    aliases = {"1": "scan", "2": "id", "3": "diagnose", "4": "read", "5": "monitor", "6": "stop"}
    while True:
        try:
            line = input("robstride> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return 0
        if line in ("quit", "exit", "q"):
            return 0
        if not line or line == "help":
            print(MENU)
            continue
        try:
            tokens = shlex.split(line)
            if not tokens:
                continue
            tokens[0] = aliases.get(tokens[0], tokens[0])
            base = ["--host-id", str(args.host_id), "--model", args.model, "--timeout", str(args.timeout)]
            subargs = parser().parse_args(base + tokens)
            if subargs.command == "menu":
                continue
            if subargs.command == "params":
                show_parameters()
            else:
                dispatch(subargs, client)
        except SystemExit:
            pass
        except KeyboardInterrupt:
            print("중단됨. 구동 중이었다면 정지 프레임을 송신했습니다.")
        except (ValueError, OSError, MotorError) as exc:
            print(f"오류: {exc}", file=sys.stderr)


def show_parameters():
    for p in PARAMETERS:
        print(f"0x{p.index:04X}  {p.name:15} {p.kind:7} {'RW' if p.writable else 'R ':2}  {p.description}")


def interrupted(signum, frame):
    raise KeyboardInterrupt


def main(argv=None):
    args = parser().parse_args(argv)
    if args.command == "params":
        show_parameters()
        return 0
    transport = trace = None
    previous = signal.signal(signal.SIGTERM, interrupted)
    try:
        # Validate authorization and global options before opening hardware.
        authorize(args)
        bounded(args.timeout, 0.005, 5, "timeout")
        profile = PROFILES[args.model]
        trace = Trace(args.log, profile)
        if args.simulate:
            print("[SIMULATION] 실제 CAN/모터에 접근하지 않습니다", file=sys.stderr)
            transport = Simulator(profile)
        else:
            transport = SocketCAN(args.channel)
        client = Client(transport, args.host_id, args.timeout, profile, trace)
        return menu(args, client) if args.command == "menu" else dispatch(args, client)
    except KeyboardInterrupt:
        print("중단됨. 구동 중이었다면 정지 프레임을 송신했습니다.", file=sys.stderr)
        return 130
    except (ValueError, OSError, MotorError) as exc:
        print(f"오류: {exc}", file=sys.stderr)
        if isinstance(exc, OSError) and not args.simulate:
            print(f"ip -details link show {args.channel} 로 UP/bitrate/bus-off/배선을 확인하세요.", file=sys.stderr)
        return 1
    finally:
        signal.signal(signal.SIGTERM, previous)
        if transport:
            transport.close()
        if trace:
            trace.close()
