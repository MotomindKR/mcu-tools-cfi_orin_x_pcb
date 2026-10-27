# CLI Command Reference (`cfi_orin_x_pcb`)

| Setting | Value |
|---|---|
| Baud rate | `921600` |
| Data bits | `8` |
| Parity | `None` |
| Stop bits | `1` |
| Flow control | Disabled |
| Command terminator | Enter(\r\n) |

## System commands

| Command | Description |
|---|---|
| `reboot` | Reset the MCU immediately using `NVIC_SystemReset()`. |
| `sync` | Refresh the IWC host-valid timer, request a GPI snapshot, and request an IWC snapshot.|

## Camera sync

| Command | Description |
|---|---|
| `camera sync <on/off>` | Start/Stop trigger pulses. |
| `camera sync frequency <Hz>` | Set a frequency from 1 to 240 Hz, including decimals such as `29.97`; ON/OFF state is preserved. |

## LED commands

`region` selects the target LED area: `0` = both regions, `1` = left, and
`2` = right. Colors are unsigned 24-bit/32-bit RGB values; hexadecimal values
such as `0xFF8C00` are accepted.

| Command | Description |
|---|---|
| `led normal_color <0x000000-0xFFFFFF>` | Set the normal-profile RGB color. |
| `led <region> off` | Turn the selected region off. |
| `led <region> rainbow <step_size>` | Run the rainbow effect. |
| `led <region> breath <color> <interval_s>` | Run a breathing effect. |
| `led <region> blink <color> <interval_s>` | Run a repeating blink effect. |
| `led <region> burst_blink <color> <count> <blink_interval_s> <off_interval_s>` | Run repeated burst blinking. |
| `led <region> static <color>` | Set a fixed RGB color. |

Examples:

```text
led normal_color 0x00BFFF
led 0 static 0xFF8C00
led 1 blink 0xFF0000 0.2
led 2 burst_blink 0xFFFF00 3 0.1 2.0
```

## IWC commands

| Command | Description |
|---|---|
| `iwc emergency press` | Assert the software emergency-stop request. |
| `iwc emergency release` | Release the software emergency-stop request. Hardware emergency and latch conditions can still keep effective emergency active. |
| `iwc emergency stop_category <0/1>` | Set the emergency-stop category. |
| `iwc emergency latch <0/1>` | Enable (`1`) or disable (`0`) the emergency latch. |
| `iwc critical clear` | Request clearing latched Critical states. A Critical bit remains set while its fault condition is still active. |
