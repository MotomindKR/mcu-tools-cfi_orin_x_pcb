# STM32 UART Flash

## 요구 사항

- Python 3
- `python3-libgpiod`
- `python3-serial`(pyserial)
- GPIO 및 UART 장치 파일에 대한 접근 권한


```bash
sudo apt update
sudo apt install python3-libgpiod python3-serial
```

필요한 경우 스크립트에 실행 권한을 부여합니다.

```bash
chmod +x stm32_uart_flash.py
```

## 기본 사용법

```bash
sudo python3 stm32_uart_flash.py
```

펌웨어 경로를 생략하면 현재 작업 디렉터리와 관계없이 스크립트와 같은 폴더의
`cfi_orin_x_app.bin`을 다운로드합니다. 다른 파일을 사용하려면 경로를 지정합니다.

```bash
sudo python3 stm32_uart_flash.py /path/to/firmware.bin
```

기본 설정은 다음과 같습니다.

| 항목 | 기본값 |
| --- | --- |
| UART 장치 | `/dev/ttyTHS1` |
| 전송 속도 | `115200` baud |
| UART 형식 | `8E1`(8 data bits, even parity, 1 stop bit) |
| 기록 시작 주소 | `0x08000000` |
| 일반 UART timeout | 2초 |
| 전체 삭제 timeout | 120초 |
| 동기화 시도 횟수 | 3회 |
| 플래시 삭제 | 실행 |
| 기록 검증 | 실행 |
| 완료 후 애플리케이션 실행 | 실행 |

## 명령행 옵션

```text
usage: stm32_uart_flash.py [-h] [--port PORT] [--baud BAUD]
                           [--address ADDRESS] [--parity {even,none}]
                           [--timeout TIMEOUT]
                           [--erase-timeout ERASE_TIMEOUT]
                           [--sync-retries SYNC_RETRIES] [--skip-erase]
                           [--no-verify] [--no-run] [--chip CHIP]
                           [firmware]
```

| 인자/옵션 | 설명 |
| --- | --- |
| `firmware` | 기록할 raw `.bin` 파일(선택). 기본값: 스크립트와 같은 폴더의 `cfi_orin_x_app.bin` |
| `--port PORT` | UART 장치. 기본값: `/dev/ttyTHS1` |
| `--baud BAUD` | UART 전송 속도. 기본값: `115200` |
| `--address ADDRESS` | 기록 시작 주소. 기본값: `0x08000000` |
| `--parity {even,none}` | UART parity. 기본값: `even` |
| `--timeout TIMEOUT` | 일반 UART 응답 timeout(초). 기본값: `2.0` |
| `--erase-timeout SECONDS` | mass erase timeout(초). 기본값: `120.0` |
| `--sync-retries COUNT` | UART를 다시 열고 GPIO reset 및 부트로더 동기화를 시도하는 횟수. 기본값: `3` |
| `--skip-erase` | 기록 전 전체 플래시 삭제 생략 |
| `--no-verify` | 기록 후 read-back 검증 생략 |
| `--no-run` | 성공 후 애플리케이션으로 reset하지 않음 |
| `--chip PATH` | GPIO 장치 직접 지정(예: `/dev/gpiochip1`) |
