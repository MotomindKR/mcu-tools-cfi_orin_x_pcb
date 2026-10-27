# Info, Warning and Critical System (`cfi_orin_x_pcb`)

## Info

사용자가 참고하면 좋은 상태 정보 또는 이벤트 기록

| 이름 | 설명 | 감지 방법 |
|------|------|-----------|
| `INFO_EMERGENCY_EFFECTIVE` | 시스템이 적용 중인 최종 비상정지 상태 | `INFO_EMERGENCY_HARDWARE` `INFO_EMERGENCY_SOFTWARE`, `emergency_latch` |
| `INFO_EMERGENCY_HARDWARE` | 하드웨어 비상정지 스위치 눌림 | nEMERGENCY 핀 |
| `INFO_EMERGENCY_SOFTWARE` | 소프트웨어 비상정지 요청 | `iwc emergency press/release` |
| `INFO_CHG_BYPASS_CONNECTED` | 충전기 연결됨 | nDET_ADAP 핀 |
| `INFO_BATTERY_CHARGING` | 충전기가 연결되었고 실제 충전 전류가 발생하고 있음 | 진입: 충전기 연결 && x < -0.3 A. 해제: 충전기 미연결 또는 x >= -0.1 A. 중간 구간은 기존 상태 유지, 진입/해제 모두 10회 연속 판정 적용 |


## Warning

향후 문제로 발전할 가능성이 있는 상태

| 이름 | 설명 | 감지 방법 | 기준 | 노란색 LED 깜빡임 횟수|
|------|------|-----------|---|---|
| `WARNING_OVER_CURRENT_DISCHARGE_VBAT` | 배터리 방전 과전류 | INA226 센서값 reading | x >= 12.000 (A) (방전) |1|
| `WARNING_OVER_CURRENT_CHARGE_VBAT` | 배터리 충전 과전류 | INA226 센서값 reading | x <= -5.000 (충전) |2|
| `WARNING_OVER_VOLTAGE_VBAT` | 배터리 과전압 | INA226 센서값 reading | x >= 30.000 (V) (배터리 프로필에 따라 변경 가능) |3|
| `WARNING_UNDER_VOLTAGE_VBAT` | 배터리 저전압 | INA226 센서값 reading | x <= 21.000 (V) |4|
| `WARNING_OVER_CURRENT_VMOT` | 액츄에이터 전원 과전류 | ADC 값 reading | x >= 10.000 (A) |5|
| `WARNING_OVER_VOLTAGE_VMOT` | 액츄에이터 전원 과전압 | ADC 값 reading | x >= 30.000 (V) |6|
| `WARNING_OVER_TEMPERATURE_PCB` | 보드 온도 높음 | bmp388 센서값 reading | x >= 80 (°C) |7|
| `WARNING_UNDER_TEMPERATURE_PCB` | 보드 온도 낮음 | bmp388 센서값 reading | x <= -5 (°C) |8|

## Critical

안전성 또는 핵심 기능에 영향을 주는 이상 상태로, 로봇이 정상 동작을 지속할 수 없는 상태.

| 이름 | 설명 | 감지 방법 | 기준 | 빨간색 LED 깜빡임 횟수|
|------|------|-----------|---|---|
| `CRITICAL_OVER_CURRENT_DISCHARGE_VBAT` | 배터리 방전 과전류 | INA226 센서값 reading | x >= 20.000 (A) (방전) |1|
| `CRITICAL_OVER_CURRENT_CHARGE_VBAT` | 배터리 충전 과전류 | INA226 센서값 reading | x <= -7.000 (충전) |2|
| `CRITICAL_OVER_VOLTAGE_VBAT` | 배터리 과전압 | INA226 센서값 reading | x >= 32.000 (V) (배터리 프로필에 따라 변경 가능) |3|
| `CRITICAL_UNDER_VOLTAGE_VBAT` | 배터리 저전압 | INA226 센서값 reading | x <= 18.000 (V) |4|
| `CRITICAL_OVER_CURRENT_VMOT` | 액츄에이터 전원 과전류 | ADC 값 reading| x >= 12.000 (A) |5|
| `CRITICAL_OVER_VOLTAGE_VMOT` | 액츄에이터 전원 과전압 | ADC 값 reading| x >= 30.000 (V) |6|
| `CRITICAL_OVER_TEMPERATURE_PCB` | 보드 온도 높음 | bmp388 센서값 reading | x >= 100 (°C) |7|
| `CRITICAL_UNDER_TEMPERATURE_PCB` | 보드 온도 낮음 | bmp388 센서값 reading | x <= -20 (°C) |8|
| `CRITICAL_POWER_FAULT_3V3` | 3.3V 전원 문제 있음 | PG_3V3_SYS 핀| LOW |9|

### Critical Latch

Critical은 Warning과 달리 조건이 정상으로 돌아와도 자동 해제되지 않는다. Critical 조건이 debounce를 통과해 발생하면 해당 Critical bit는 latch 상태로 유지된다.

`iwc critical clear` 명령은 latch된 모든 Critical bit의 해제를 시도한다.

| clear 시점의 Critical 조건 | 동작 |
|-----------------------------|------|
| 해제됨 | 해당 Critical bit를 `0`으로 해제 |
| 계속 발생 중 | 해당 bit를 유지하고 clear 요청을 무시 |

따라서 조건이 해제된 뒤 `iwc critical clear`를 실행해야 Critical LED 표시와 상태가 정상 상태로 복귀한다.

## 관련 GPIO

| Name | Mode | Description | Active |
|------|------|------|---|
| `EN_MOT_PWR` | Output | 액츄에이터 전원의 Enable 신호 | High |
| `nEN_SAFETY` | Output | 비상정지 스위치 신호가 모터 전원을 차단하는 하드웨어 신호 라인의 Enable 신호  | Low |
| `nEMERGENCY` | Input | 비상정지 스위치 눌림| Low|
| `nDET_ADAP` | Input |충전기_2 전압 감지 | Low |
| `PG_3V3_SYS` | Input | 3.3V 라인 PG 신호 | High | 

## 관련 센서

| Name | Description | 변수 (단위) |
|------|------|------|
| `BMP388` | 압력 및 온도 센서 | temperature_c = 섭씨(°C), ex. 24.26<br>pressure_pa = 파스칼(Pa), ex. 95287.09 (÷100 하면 hPa) |
| `INA226` | 배터리 전압 및 전류 센서 | BatteryVoltage_V = (V), ex. 25.6 <br> ShuntCurrent_A = (A), ex. 0.234|

## 비상정지 스위치 시나리오

### 상태 및 CLI

- 실제 하드웨어 비상정지 스위치가 눌리면 `nEMERGENCY` (MCU Input)는 `LOW`, 해제되면 `HIGH`가 된다.
- `INFO_EMERGENCY_HARDWARE`는 `nEMERGENCY == LOW`일 때 `1`, `nEMERGENCY == HIGH`일 때 `0`이 된다.
- 소프트웨어 비상정지는 CLI로 설정한다.
  - `iwc emergency press`: `INFO_EMERGENCY_SOFTWARE`를 `1`로 설정한다.
  - `iwc emergency release`: `INFO_EMERGENCY_SOFTWARE`를 `0`으로 설정한다.
- 비상정지 동작 판단에는 `INFO_EMERGENCY_EFFECTIVE`를 사용한다.
  - `INFO_EMERGENCY_HARDWARE == 1` 또는 `INFO_EMERGENCY_SOFTWARE == 1`이면 `INFO_EMERGENCY_EFFECTIVE`는 `1`이 된다.
  - `emergency_latch == 0`이면 `INFO_EMERGENCY_HARDWARE == 0`이고 `INFO_EMERGENCY_SOFTWARE == 0`일 때 `INFO_EMERGENCY_EFFECTIVE`는 자동으로 `0`이 된다.
  - `emergency_latch == 1`에서 `INFO_EMERGENCY_HARDWARE == 1`이 되면 `INFO_EMERGENCY_SOFTWARE`도 자동으로 `1`이 된다.
  - `emergency_latch == 1`에서 전체 비상정지를 해제하려면 하드웨어 비상정지를 먼저 해제한 뒤 `iwc emergency release`로 `INFO_EMERGENCY_SOFTWARE`를 `0`으로 만들어야 한다.
- `iwc emergency stop_category [0/1]`로 `emergency_stop_category`를 변경한다.
- `iwc emergency latch [0/1]`로 `emergency_latch`를 변경한다.
- `sync` 명령으로 `host_valid`를 `1`로 갱신한다. `host_valid == 1`은 5초 동안만 유지되며, 5초 안에 새 `sync` 명령이 들어오지 않으면 자동으로 `0`이 된다.

### `nEN_SAFETY` 제어

`emergency_stop_category` 또는 `host_valid`가 변경되면 MCU는 `nEN_SAFETY`를 아래와 같이 설정한다.

| 조건 | `nEN_SAFETY` | 의미 |
|------|--------------|------|
| `emergency_stop_category == 0` | `LOW` | 하드웨어 비상정지 차단 경로 enable |
| `emergency_stop_category == 1 && host_valid == 0` | `LOW` | 하드웨어 비상정지 차단 경로 enable |
| `emergency_stop_category == 1 && host_valid == 1` | `HIGH` | 하드웨어 비상정지 차단 경로 disable |

### effective emergency pressed 전환

effective emergency 상태가 `released` -> `pressed`로 바뀌면 MCU는 아래와 같이 동작한다.

| 조건 | `EN_MOT_PWR` 동작 |
|------|-------------------|
| `emergency_stop_category == 0` | `LOW`로 설정하여 모터 전원을 차단 |
| `emergency_stop_category == 1` | 변경하지 않음 |

위 전환은 실제 하드웨어 비상정지 스위치가 눌렸을 때뿐 아니라 `iwc emergency press`로 소프트웨어 비상정지가 눌렸을 때도 동일하게 적용된다.

### effective emergency released 전환

effective emergency 상태가 `pressed` -> `released`로 바뀌면 MCU는 `EN_MOT_PWR`를 `HIGH`로 설정하여 모터 전원을 복구한다.

`emergency_latch == 0`이면 하드웨어 비상정지와 소프트웨어 비상정지가 모두 해제되었을 때 자동으로 released 전환이 발생한다.

`emergency_latch == 1`이면 하드웨어 비상정지가 눌릴 때 소프트웨어 비상정지도 자동으로 눌린 상태가 된다. 하드웨어 비상정지를 먼저 해제한 뒤 `iwc emergency release` 명령으로 소프트웨어 비상정지를 해제해야 released 전환이 발생한다.

### 소프트웨어 비상정지 예외 동작

`iwc emergency press` 명령이 들어오면, MCU가 `EN_MOT_PWR`를 직접 `LOW`로 설정한다.

| 조건 | `EN_MOT_PWR` 동작 |
|------|-------------------|
| 항상 | `LOW`로 설정하여 모터 전원을 차단 |

이는 `emergency_stop_category`, `host_valid`, 기존 `INFO_EMERGENCY_EFFECTIVE` 상태와 무관하게, 상위단의 소프트웨어 비상정지 명령만으로도 모터 전원을 명시적으로 차단하기 위한 동작이다.

`emergency_latch == 1`에서 하드웨어 비상정지로 인해 소프트웨어 비상정지가 자동으로 눌린 경우에도, 해제는 일반 소프트웨어 비상정지와 동일하게 `iwc emergency release` 명령으로 수행한다.

| 조건 | `EN_MOT_PWR` 동작 |
|------|-------------------|
| `emergency_latch == 1 && INFO_EMERGENCY_HARDWARE == 0 && INFO_EMERGENCY_SOFTWARE == 0` | `HIGH`로 설정하여 모터 전원을 복구 |

단, 실제 하드웨어 비상정지 스위치가 아직 눌린 상태이면 `iwc emergency release`를 실행해도 latch 동작에 의해 `INFO_EMERGENCY_SOFTWARE`가 다시 `1`이 되므로 모터 전원은 복구되지 않는다.

## LED Profile 적용

IWC 상태가 변경되면 MCU는 아래 우선순위에 따라 LED 상태를 선택한다. 여러 상태가 동시에 발생하면 더 높은 우선순위의 상태 하나만 적용한다.

| 우선순위 | 조건 | LED 동작 |
|---------|------|----------|
| 1 | `INFO_EMERGENCY_EFFECTIVE == 1` | `LED_PROFILE_EMERGENCY_STOP` |
| 2 | Critical bit 중 하나 이상 `1` | 빨간색 burst blink |
| 3 | Warning bit 중 하나 이상 `1` | 노란색 burst blink |
| 4 | `INFO_BATTERY_CHARGING == 1` | `LED_PROFILE_CHARGING` |
| 5 | `charger_connected == 1` | `LED_PROFILE_CHARGER_CONNECTED` |
| 6 | `host_valid == 0` | `LED_PROFILE_NORMAL_HOST_NOT_VALID` |
| 7 | 위 조건에 해당하지 않음 | `LED_PROFILE_NORMAL` |

Critical 또는 Warning은 해당 level에서 enum 순서가 가장 앞선 활성 bit 하나를 선택한다. `CRITICAL`은 `WARNING`보다 우선한다. 선택한 bit의 순서에 따라 `led_post_burst_blink()`으로 다음 동작을 반복한다.

| Level | Color | 점멸 횟수 | 빠른 점멸 간격 | 반복 전 소등 시간 |
|-------|-------|-----------|----------------|------------------|
| Warning | 노란색 (`0xFFFF00`) | enum 값 + 1회 | 0.1 s | 2.0 s |
| Critical | 빨간색 (`0xFF0000`) | enum 값 + 1회 | 0.1 s | 2.0 s |

Warning과 Critical 모두 선택한 bit의 enum 값에 1을 더한 횟수만큼 점멸한다. 선택한 level 또는 bit가 바뀔 때만 새 burst blink 명령을 LED task에 전달하여, 같은 상태에서 애니메이션이 반복 초기화되지 않게 한다.
