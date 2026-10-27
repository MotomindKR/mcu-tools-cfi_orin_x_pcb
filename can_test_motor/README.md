# RobStride CAN 모터 스캔 / MMMCP 기능 시험

Linux `can0`에 연결된 RobStride 모터를 검색하고, 첨부 [mmmcp.pdf](mmmcp.pdf)의 모든 명령과 파라미터를 시험하는 Python 프로그램입니다. Python 3.9 이상과 Linux SocketCAN을 사용하며, ROS와 pip 패키지 설치가 필요 없습니다. CLI와 대화형 메뉴를 제공합니다.

## 시작하기

아래 명령은 이 디렉터리(`can_test_motor`) 기준입니다. 저장소 루트에서는 `python3 can_test_motor/main.py ...`로 실행할 수 있습니다.

```bash
cd can_test_motor
python3 main.py --help
python3 main.py --simulate scan
python3 main.py --simulate menu
```

`--simulate`는 실제 CAN 소켓을 열지 않고 ID `1`, `127`인 모터 두 개를 모의합니다. 시뮬레이터 상태는 한 프로세스 안에서 유지되므로 연속적인 쓰기/읽기 시험에는 `menu`를 사용하세요. 시뮬레이션은 프레임·제어 흐름을 검증하며 실제 모터 역학이나 펌웨어 호환성을 보증하지 않습니다.

실제 장비에서는 먼저 모터 전원, CAN H/L/GND와 종단저항을 확인한 뒤 인터페이스를 설정합니다. 다음 명령은 호스트 네트워크 설정을 변경하므로 장비에서 직접 실행하세요.

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details -statistics link show can0

python3 main.py scan
python3 main.py motion velocity --motor 127 \
  --velocity 1 --max-velocity 1.5 \
  --acceleration 0.2 \
  --current-limit 1 --torque-limit 0.5 \
  --duration 0 --yes
```

프로그램은 인터페이스 UP/DOWN, 호스트 bitrate, 모터 재부팅을 자동으로 변경하지 않습니다. 기본은 `can0`, host ID `0xFD`, 응답 timeout `0.2초`입니다. 전역 옵션은 하위 명령 **앞에** 둡니다.

```bash
python3 main.py --channel can0 --host-id 0xFD --model 02 --timeout 0.3 scan
python3 main.py --model 02 --log session.jsonl read --motor 0x7F all
```

## 검색과 읽기 전용 진단

```bash
python3 main.py scan
python3 main.py scan --start 1 --end 127 --scan-timeout 0.05
python3 main.py id --motor 0x7F
python3 main.py version --motor 0x7F
python3 main.py params
python3 main.py read --motor 0x7F loc_kp
python3 main.py read --motor 0x7F 0x701E
python3 main.py read --motor 0x7F all
python3 main.py diagnose --motor 0x7F --report diagnosis.json
```

`scan`은 broadcast 동작을 가정하지 않고 기본 ID `0..255`에 `0x00` 요청을 하나씩 전송합니다. 모터 ID와 64-bit MCU UID를 출력합니다. 기본 대기 시간은 ID당 20ms이며 응답이 늦으면 `--scan-timeout`을 늘리세요. 동일 ID를 공유하는 모터를 확실하게 분리할 수는 없으므로 ID 변경 작업은 한 대씩 연결해 수행하세요.

`diagnose`는 ID, 버전, 파라미터 26개의 읽기 결과를 PASS/FAIL로 저장합니다. 실행·정지·쓰기 명령을 보내지 않습니다. 미지원 파라미터나 응답 실패를 실패로 보고하며 이후 항목을 계속 검사합니다. 전체가 성공하면 종료 코드 0, 실패 항목이나 검색 결과가 없으면 1입니다. Ctrl+C는 130입니다. 고장/경고 이벤트는 별도 `monitor`로 확인합니다.

## 명령 커버리지

| PDF 명령 | 기능 | 명령/시험 방법 |
|---|---|---|
| `0x00` | ID / MCU UID | `scan`, `id` |
| `0x01` | MIT 지령 | `motion mit` |
| `0x02` | 일반 피드백 | 구동·정지·쓰기 응답 및 `monitor` |
| `0x03` | Run | `run`, 모든 `motion` |
| `0x04` | Stop / fault clear | `stop`, `clear-fault` |
| `0x06` | 기계 영점 | `zero` |
| `0x07` | 모터 ID 변경 | `set-id` |
| `0x11` | 단일 파라미터 읽기 | `read`, `diagnose` |
| `0x12` | 휘발성 파라미터 쓰기 | `write`, 모든 `motion` |
| `0x15` | 상세 fault / warning | `monitor`에서 비트별 해석 |
| `0x16` | 비휘발성 파라미터 쓰기 | `write --persistent` |
| `0x17` | 모터 bitrate 변경 | `baudrate` |
| `0x18` | 주기 보고 | `periodic`, `monitor` |
| `0x19` | 프로토콜 변경 (deprecated) | `protocol` |
| `0x1A` | 펌웨어 버전 | `version`, `diagnose` |

수신 전용 `0x02`와 `0x15`를 요청 명령으로 전송하지 않습니다. 고장을 유발하는 자동 시험도 하지 않습니다. 실제 고장 발생 시 `monitor` 또는 TX/RX 로그로 내용을 검사하세요.

## 파라미터 쓰기와 저장

변경/구동 명령에는 대상과 옵션을 검토했다는 의미로 `--yes`가 필요합니다. 일반 `write`는 먼저 모터를 정지하고 idle 응답을 확인합니다. 기본 `0x12` 쓰기와 `--persistent`의 `0x16` 쓰기 모두 읽어온 값을 비교합니다. 읽기 전용 항목은 쓰기를 거부합니다.

```bash
python3 main.py write --motor 0x7F loc_kp 30 --yes
python3 main.py write --motor 0x7F zero_sta 1 --persistent --yes
python3 main.py read --motor 0x7F zero_sta
```

비휘발성 검증은 저장 명령과 즉시 readback까지입니다. 전원을 껐다 켠 뒤 같은 값을 다시 읽어 실제 유지 여부를 확인하세요. 이 도구는 자동 전원 재인가나 flash 쓰기 반복 시험을 하지 않습니다.

| 파라미터 | 형식 | 접근 / 용도 |
|---|---|---|
| `run_mode` (`0x7005`) | uint8 | RW, MIT=0 / PP=1 / velocity=2 / current=3 / CSP=5 |
| `iq_ref` (`0x7006`) | float32 | RW, 전류 A |
| `spd_ref` (`0x700A`) | float32 | RW, 속도 rad/s |
| `limit_torque` (`0x700B`) | float32 | RW, 토크 제한 Nm |
| `cur_kp`, `cur_ki` (`0x7010`, `0x7011`) | float32 | RW, 전류 루프 |
| `cur_filt_gain` (`0x7014`) | float32 | RW, 전류 필터 |
| `loc_ref` (`0x7016`) | float32 | RW, 위치 rad |
| `limit_spd`, `limit_cur` (`0x7017`, `0x7018`) | float32 | RW, 속도·전류 제한 |
| `mechPos`, `iqf`, `mechVel`, `VBUS` (`0x7019..0x701C`) | float32 | R, 위치·전류·속도·전압 |
| `loc_kp`, `spd_kp`, `spd_ki`, `spd_filt_gain` (`0x701E..0x7021`) | float32 | RW, 위치·속도 루프 |
| `acc_rad` (`0x7022`) | float32 | RW, velocity 가속도 |
| `vel_max`, `acc_set` (`0x7024`, `0x7025`) | float32 | RW, PP 속도·가속도 |
| `EPScan_time` (`0x7026`) | uint16 | RW, 보고 주기 selector |
| `canTimeout` (`0x7028`) | uint32 | RW, CAN timeout threshold |
| `zero_sta` (`0x7029`) | uint8 | RW, 전원 인가 위치 범위 |
| `damper` (`0x702A`) | uint8 | RW, 전원 OFF 역구동 보호 |
| `add_offset` (`0x702B`) | float32 | RW, 논리 영점 offset rad |

파라미터 index와 값은 little-endian입니다. uint8/uint16은 남는 바이트를 0으로 채웁니다. NaN/무한대, 형식 범위를 벗어난 정수, 미정의 run_mode, 음수 제한값을 거부합니다. 일반 `write`의 수치 범위는 해당 형식·파라미터의 기본 의미를 검증하며, 아래 `motion`의 시험 제한값은 적용하지 않습니다. 모터별 전류·튜닝 가능 범위는 해당 하드웨어 규격을 따라 입력하세요.

## 5가지 제어 모드 시험

모터를 고정하고 부하와 주변 간섭을 제거한 상태에서 시작하세요. 구동은 기본 2초, 최대 60초입니다. 모든 구동에서 먼저 Stop → idle/무고장 확인 → 현재 위치 조회 → 모드 및 제한값 설정 → 이전 전류/속도/위치 지령 초기화 → Run을 수행합니다. 제한값과 초기 지령은 enable 전에 씁니다. 이후 각 사이클마다 일반 피드백 응답을 요구합니다.

```bash
# Run(0x03) 시험: MIT 모드, Kp/Kd/토크/속도 0, 2초 후 Stop
python3 main.py --model 02 run --motor 0x7F --duration 2 --yes

# MIT: 현재 위치 유지에 작은 Kp/Kd 적용
python3 main.py --model 02 motion mit --motor 0x7F --kp 1 --kd 0.1 --duration 2 --yes

# Current: 0.1 A
python3 main.py --model 02 motion current --motor 0x7F --current 0.1 --duration 1 --yes

# Velocity: 0.1 rad/s, 가속도 및 전류 제한
python3 main.py --model 02 motion velocity --motor 0x7F --velocity 0.1 --acceleration 0.2 --current-limit 0.5 --duration 1 --yes

# CSP: 시작 위치에서 +0.1 rad
python3 main.py --model 02 motion csp --motor 0x7F --position 0.1 --relative --speed-limit 0.2 --duration 2 --yes

# PP: 시작 위치에서 -0.1 rad, 속도와 가속도 지정
python3 main.py --model 02 motion pp --motor 0x7F --position -0.1 --relative --speed-limit 0.2 --acceleration 0.2 --duration 2 --yes

python3 main.py stop --motor 0x7F
```

`--position`을 생략하면 시작 위치를 유지합니다. `--relative`는 시작 위치에 대한 증분이며 생략하면 절대 위치(rad)입니다. 기본 지령 주기는 20 Hz이고 `--rate 1..100`으로 조정합니다. Python 스케줄링·응답 대기로 실제 주파수는 낮아질 수 있으며 실시간 제어용은 아닙니다. 지정 시간은 구동 루프 기준이며 초기 설정과 마지막 정지 확인 시간이 추가됩니다. 응답 대기 중에는 `--timeout`만큼 종료가 지연될 수 있습니다.

| 소프트웨어 시험 제한 | 기본값 |
|---|---|
| `--max-current` | 1 A |
| `--max-velocity` | 1 rad/s |
| `--max-torque` | 1 Nm |
| `--max-kp`, `--max-kd` | 20, 1 |
| `--max-displacement` | 시작 위치 기준 ±0.25 rad |
| `--max-temperature` | 60 °C |

기본 `--current-limit`, `--torque-limit`, `--speed-limit`, `--acceleration`은 각각 0.5입니다. 더 큰 시험에는 명시적으로 해당 `--max-*`를 조정해야 합니다. 이것은 소프트웨어 제한이며 하드웨어 정격을 판단하지 않습니다. MIT 속도·토크·게인 지령은 선택 모델의 인코딩 범위도 검사합니다. 현재 위치가 모델 피드백 범위(±4π)를 벗어나면 이동량 감시가 모호해지므로 구동 시험을 거부합니다. 이 경우 위치 상태와 영점 설정을 확인하세요.

매 응답에서 고장/경고, 온도, 피드백 속도, 시작 위치 대비 이동량과 run 상태를 검사합니다. 정상 종료·timeout·예외·Ctrl+C·SIGTERM 시에는 응답 대기 없이 Stop을 3회 송신한 뒤 추가 Stop 요청으로 idle을 확인합니다. 정지 송신/확인 실패는 오류로 표시합니다. CAN 단선, bus-off, 프로세스 강제 종료(SIGKILL), 전원 문제에서는 소프트웨어 정지가 보장되지 않으므로 장비의 비상 정지를 사용할 수 있어야 합니다. `canTimeout`의 단위가 PDF에 없기 때문에 프로그램이 임의로 하드웨어 watchdog 값을 설정하지 않습니다.

모드/제한값/지령은 시험 후 자동 복원하지 않습니다. 모터는 정지 상태로 남습니다. 실장비에 연결된 다른 ROS 노드나 제어 프로그램은 함께 구동하지 마세요. 일반 피드백에는 요청 순번이 없어 동시 제어의 응답을 완전히 구분할 수 없습니다.

## 영점과 통신 설정

```bash
# 영점은 MIT(0) 또는 CSP(5)만 허용. PP 등에서는 오류 반환
python3 main.py write --motor 0x7F run_mode 0 --yes
python3 main.py zero --motor 0x7F --yes
python3 main.py read --motor 0x7F mechPos

# 새 ID 존재 여부 검사, 변경 응답 및 새 ID의 UID 대조
python3 main.py set-id --motor 0x7F 2 --yes
python3 main.py id --motor 2

# 모터 전원 재인가 후 적용. 호스트 can0 bitrate도 별도로 맞춰야 함
python3 main.py baudrate --motor 2 500000 --yes

# Deprecated: 전원 재인가 후 MMMCP/CANopen/MIT 프로토콜 선택
python3 main.py protocol --motor 2 mmmcp --yes
```

ID 변경은 새 ID `1..252`를 지원하며 host ID와 중복될 수 없습니다. bitrate는 `1000000`, `500000`, `250000`, `125000`을 지원합니다. `protocol ... canopen`이나 `protocol ... mit`로 바꾼 뒤 재기동하면 이 도구의 MMMCP 요청으로 통신할 수 없으므로 해당 프로토콜을 지원하는 도구로 복구해야 합니다. bitrate/protocol의 출력은 명령 응답 수신을 뜻하며, 전원 재인가 후 적용까지 자동으로 검증하지 않습니다.

## 주기 보고, 고장, PDF의 부가 기능

```bash
python3 main.py write --motor 0x7F EPScan_time 10 --yes
python3 main.py periodic --motor 0x7F on --yes
python3 main.py --log telemetry.jsonl monitor --motor 0x7F --duration 10
python3 main.py periodic --motor 0x7F off --yes

# 원인 제거 후 Stop(Byte0=1)로 fault clear
python3 main.py clear-fault --motor 0x7F --yes

# 전원 인가 위치 범위, 전원 OFF 역구동 보호, 논리 영점 offset
python3 main.py write --motor 0x7F zero_sta 1 --persistent --yes
python3 main.py write --motor 0x7F damper 1 --yes
python3 main.py write --motor 0x7F add_offset 0.1 --yes
```

`monitor`는 수신만 하므로 보고 기능이 꺼져 있으면 아무 프레임도 없을 수 있습니다. `periodic on`은 명시적으로 `off`할 때까지 유지합니다. PDF의 기본 보고 주기는 10ms지만 `EPScan_time` selector의 시간 단위/변환과 `canTimeout`의 단위는 명시되어 있지 않습니다. 입력값을 그대로 전송하며 실제 주기와 timeout은 해당 펌웨어에서 측정하세요.

PDF에 따라 `0x02`의 데이터는 위치/속도/토크/온도, `0x18`은 위치/속도/Kp/Kd로 구분합니다. `0x18`에서 온도를 추정하지 않습니다. `0x15`의 fault/warning은 little-endian uint32로 해석하며 정의되지 않은 비트도 `unknown_bit_N`으로 표시합니다. JSONL에는 타임스탬프, TX/RX 구분, 원시 ID/8바이트와 RX 해석 결과를 남깁니다.

`zero_sta=0`은 전원 인가 위치 범위 0..2π, `1`은 -π..π입니다. `damper=1`은 전원 OFF 상태 역구동 댐핑을 **해제**하며 `0`은 기본 동작입니다. `add_offset`은 rad 단위로 논리 영점을 이동합니다. 이 기능의 물리적 효과, 전원 재인가 유지 여부, 전원 OFF 동작은 별도 장비 시험으로 확인해야 합니다.

## 참고 구현과 규격 우선순위

- 기준: 첨부 `mmmcp.pdf` (MMMCP, CAN 2.0B, 29-bit extended ID, DLC 8).
- [RobStride/robstride_ros_sample](https://github.com/RobStride/robstride_ros_sample/tree/d4751f3f16f3593a4e587738b6e27b949b4999ba): SocketCAN 구조, 모델별 범위, 5가지 모드 흐름 참조.
- [RobStride/robstride_actuator_bridge](https://github.com/RobStride/robstride_actuator_bridge/tree/ed3da42e4d40c2d12a2d11a5224ae4d2ed7e2fe6): extended ID, MIT 패킹과 파라미터 프레임 참조.

두 저장소는 프로토콜 이해를 위해 참고했으며 ROS C++ 코드를 복사하지 않았습니다. 충돌하는 부분은 PDF를 우선합니다. 예를 들어 bridge의 위치 범위 ±12.5 대신 PDF의 ±4π, uint16 매핑 분모 65535, current 지령은 스케일된 정수가 아닌 float32 A, velocity 가속도는 `0x7022`를 사용합니다. PDF에 없는 run_mode 4와 파라미터 `0x701D`는 제공하지 않습니다.

`--model`은 자동 감지가 아니며 MIT 인코딩과 피드백 변환에 쓰입니다. UID로 모델을 추정하지 않습니다.

| model | 위치 ±rad | 속도 ±rad/s | 토크 ±Nm | Kp 최대 | Kd 최대 |
|---|---:|---:|---:|---:|---:|
| `mmmcp` (기본), `01`, `02` | 4π | 44 | 17 | 500 | 5 |
| `00` | 4π | 50 | 17 | 500 | 5 |
| `03` | 4π | 50 | 60 | 5000 | 100 |
| `04` | 4π | 15 | 120 | 5000 | 100 |
| `05` | 4π | 33 | 17 | 500 | 5 |
| `06` | 4π | 20 | 36 | 5000 | 100 |

## 검증과 파일 구성

```bash
python3 -m unittest discover -s tests -v
python3 main.py --simulate diagnose --motor 1 --report /tmp/robstride-diagnosis.json
python3 main.py --simulate motion csp --motor 1 --position 0.05 --relative --duration 0.1 --yes
```

테스트는 PDF의 실제 읽기 예시(0x701E=30.0), 엔디언/자료형, MIT 경계, 두 피드백 형식, 고장 비트, 전체 진단, ID 충돌, 쓰기 검증, 5모드 초기화 순서와 정지, timeout/고장/과열/Ctrl+C 후 정지, SocketCAN 프레임 처리와 CLI를 검사합니다. 실제 CAN 송수신이나 실제 모터 동작을 시험한 것은 아닙니다.

- `main.py`: 실행 진입점
- `robstride/protocol.py`: 파라미터·모델·프레임 인코딩/해석
- `robstride/transport.py`: SocketCAN, JSONL 로그, 시뮬레이터
- `robstride/controller.py`: 요청/응답, 스캔·진단·제한 시간 구동
- `robstride/cli.py`: CLI / 대화형 메뉴
- `tests/test_robstride.py`: 하드웨어 없는 검증

## 문제 해결

- `Network is down`: `ip link show can0` 확인 후 UP/bitrate 설정.
- `No such device`: `--channel`과 CAN 장치 드라이버 확인.
- timeout/검색 0대: 전원, bitrate, 종단, H/L/GND, 실제 프로토콜, 스캔 대기 시간을 확인.
- bus-off/CAN error: `ip -details -statistics link show can0`에서 오류 카운터 확인. 프로그램은 자동 재시작하지 않음.
- 특정 파라미터 실패: 구형 펌웨어의 미지원 가능성. 진단 JSON에서 항목별 결과 확인.
- 영점 실패: `run_mode`를 MIT 또는 CSP로 설정 후 모터가 정지했는지 확인.
