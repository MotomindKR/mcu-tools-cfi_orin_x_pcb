# COG Binary Stream (`cfi_orin_x_pcb`)

COG is a binary telemetry stream from the MCU, sent over the log UART.
This document covers what a receiver needs to decode it.

## Link

| Item          | Value                                   |
|---------------|-----------------------------------------|
| UART          | `/dev/ttyTHS1`, 921600 baud, 8N1, no flow control |
| Framing       | COBS-encoded, terminated by `0x00`      |
| Byte order    | little-endian for all multi-byte fields |
| Float format  | IEEE 754 float32                        |


## Decoding

1. Split the stream on `0x00`. Drop empty chunks.
2. Drop chunks longer than **246 bytes** (245 raw + 1 COBS overhead).
3. COBS-decode the chunk into a raw frame.
4. Validate the raw frame:
   - `len(raw) == 3 + len + 2`
   - `len <= 240`, and equal to the fixed payload size for known types
   - CRC matches (see [CRC](#crc))
5. Dispatch on `type`.

### Raw frame

```text
+------+-----+-----+--------------+------+------+
| type | seq | len | payload[len] | crc  (LE)   |
+------+-----+-----+--------------+------+------+
   1      1     1      0..240          2
```

| Field     | Size | Notes                                                 |
|-----------|------|-------------------------------------------------------|
| `type`    | 1    | message type, see [Message types](#message-types)     |
| `seq`     | 1    | per-type counter, +1 per frame, wraps 255 → 0         |
| `len`     | 1    | payload length                                        |
| `payload` | len  | message body                                          |
| `crc`     | 2    | CRC-16 over `type..payload`, little-endian            |

`seq` is independent for each `type`. A gap within one type means lost frames.
The counter resets to 0 on MCU reset.

### CRC

CRC-16/CCITT-FALSE over `type | seq | len | payload`.

| Parameter  | Value    |
|------------|----------|
| Polynomial | `0x1021` |
| Init       | `0xFFFF` |
| RefIn/Out  | false    |
| XorOut     | `0x0000` |
| Check      | `"123456789"` → `0x29B1` |

```python
def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc
```

Reference decoder: [`cli_cog.py`](cli_cog.py) (`decode_frame`, `FrameStream`).

## Message types

| Type   | Name                            | Len | Payload (`struct`) | Sent when                                  |
|--------|---------------------------------|-----|--------------------|--------------------------------------------|
| `0x01` | `COG_TYPE_BATTERY_V_I`          | 12  | `<3f`              | each INA226 read, while sensor print is on |
| `0x02` | `COG_TYPE_GPI`                  | 4   | `<I`               | any GPI change (debounced), `sync`         |
| `0x03` | `COG_TYPE_IWC`                  | 12  | `<3I`              | any IWC flag change, `sync`                |
| `0x04` | reserved                        | –   | –                  | not sent                                   |
| `0x05` | `COG_TYPE_TEMPERATURE_PRESSURE` | 8   | `<2f`              | each valid BMP390 read, while sensor print is on |
| `0x06` | reserved                        | –   | –                  | not sent                                   |
| `0x07` | `COG_TYPE_VMOT_V_I`             | 8   | `<2f`              | each valid ADC read, while sensor print is on |
| `0x08` | `COG_TYPE_TICK_COUNT`           | 4   | `<I`               | `sync`                                     |

`sync` (CLI command) sends one `0x08`, `0x02` and `0x03` frame. Use it to get the
current state after connecting, since `0x02`/`0x03` are otherwise sent only on change.

Float fields may be `NaN` when a value is unavailable.

### `0x01` Battery V/I

| Offset | Field                | Type    | Unit |
|--------|----------------------|---------|------|
| 0      | `bus_voltage_v`      | float32 | V    |
| 4      | `shunt_current_a`    | float32 | A    |
| 8      | `battery_soc_percent`| float32 | %    |

### `0x02` GPI

| Offset | Field        | Type   |
|--------|--------------|--------|
| 0      | `gpi_bits_1` | uint32 |

Each bit is the raw pin level (`1` = HIGH, `0` = LOW). An `n` prefix means the
signal is active-low; the bit is still the raw level.

| Bit | Signal           | Bit | Signal         |
|-----|------------------|-----|----------------|
| 0   | `nEMERGENCY`     | 7   | `nEN_SAFETY`   |
| 1   | `nINT_INA226`    | 8   | `MCU_PWR_BTN`  |
| 2   | `nDET_ADAP`      | 9   | `STM_CAN_STB`  |
| 3   | `nSYS_RESET_3V3` | 10  | `POWER_EN_3V3` |
| 4   | `nMOD_SLEEP_3V3` | 11  | `EN_5V0_EXT`   |
| 5   | `nSHUTDOWN_REQ`  | 12  | `EN_MOT_PWR`   |
| 6   | `PG_3V3_SYS`     | 13–31 | reserved (`0`) |

### `0x03` IWC (Info / Warning / Critical)

| Offset | Field           | Type   |
|--------|-----------------|--------|
| 0      | `info_bits`     | uint32 |
| 4      | `warning_bits`  | uint32 |
| 8      | `critical_bits` | uint32 |

A set bit means the flag is active. Unlisted bits are reserved (`0`).
Critical flags stay set after the cause clears, until `iwc critical clear` succeeds.

| Bit | Info                   | Warning                       | Critical                      |
|-----|------------------------|-------------------------------|-------------------------------|
| 0   | `EMERGENCY_EFFECTIVE`  | `OVER_CURRENT_DISCHARGE_VBAT` | `OVER_CURRENT_DISCHARGE_VBAT` |
| 1   | `EMERGENCY_HARDWARE`   | `OVER_CURRENT_CHARGE_VBAT`    | `OVER_CURRENT_CHARGE_VBAT`    |
| 2   | `EMERGENCY_SOFTWARE`   | `OVER_VOLTAGE_VBAT`           | `OVER_VOLTAGE_VBAT`           |
| 3   | `CHG_BYPASS_CONNECTED` | `UNDER_VOLTAGE_VBAT`          | `UNDER_VOLTAGE_VBAT`          |
| 4   | `BATTERY_CHARGING`     | `OVER_CURRENT_VMOT`           | `OVER_CURRENT_VMOT`           |
| 5   |                        | `OVER_VOLTAGE_VMOT`           | `OVER_VOLTAGE_VMOT`           |
| 6   |                        | `OVER_TEMPERATURE_PCB`        | `OVER_TEMPERATURE_PCB`        |
| 7   |                        | `UNDER_TEMPERATURE_PCB`       | `UNDER_TEMPERATURE_PCB`       |
| 8   |                        |                               | `POWER_FAULT_3V3`             |

### `0x05` Temperature / Pressure (BMP390)

| Offset | Field           | Type    | Unit |
|--------|-----------------|---------|------|
| 0      | `temperature_c` | float32 | °C   |
| 4      | `pressure_pa`   | float32 | Pa   |

### `0x07` VMOT V/I

| Offset | Field            | Type    | Unit |
|--------|------------------|---------|------|
| 0      | `vmot_voltage_v` | float32 | V    |
| 4      | `vmot_current_a` | float32 | A    |

Both values are filtered and calibrated.

### `0x08` Tick count

| Offset | Field        | Type   | Unit                    |
|--------|--------------|--------|-------------------------|
| 0      | `tick_count` | uint32 | kernel tick (1 ms)      |

