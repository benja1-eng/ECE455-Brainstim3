# PicoW DAC — BLE Wave Generator

Standalone HTML frontend + Pico 2 W MicroPython firmware that drive the
`DualDac` engine in `pio_dac.py` over Bluetooth LE.

## Files

| File | Role |
|------|------|
| `pio_dac.py` | Existing PIO + DMA DAC driver (unchanged). |
| `pico_ble_dac.py` | **New firmware.** Runs on the Pico, advertises BLE, maps text commands to `DualDac` calls. |
| `dac_controller.html` | **New frontend.** Standalone HTML — open it in Chrome/Edge and talk to the Pico via Web Bluetooth. |

## Upload to the Pico

1. Copy both `pio_dac.py` and `pico_ble_dac.py` to the Pico root filesystem
   (e.g. using Thonny, `mpremote cp`, or `rshell`).
2. Optionally rename `pico_ble_dac.py` to `main.py` so it auto-starts on
   boot. If you do, remove the `test_temporal_interference()` call at the
   bottom of any other `main.py` that might exist.
3. Soft-reset the board (Ctrl+D in REPL or power-cycle) so the PIO can
   claim the SPI pins.

On boot the Pico advertises as **`PicoW_DAC`** with service `0x1815` and
writable characteristic `0x2A56`.

## Open the frontend

Web Bluetooth needs a secure context. Any of these works:

- Double-click `dac_controller.html` (opens as `file://` — works in
  desktop Chrome, Edge, Opera, and Brave).
- Serve it locally: `python3 -m http.server` in this folder, then open
  `http://localhost:8000/dac_controller.html`.
- Host it on any `https://` URL.

Safari / iOS do not support Web Bluetooth. Use desktop Chromium-family
browsers.

Click **Connect**, pick `PicoW_DAC` from the chooser, and you're live.

## UI modes

- **Temporal Interference** — set `f1` and `f2` directly, one per DAC.
  The beat is displayed as `|f1 − f2|`.
- **Center + Beat** — set the carrier center and the beat; the firmware
  derives `f1 = center − beat/2`, `f2 = center + beat/2`.
- **Single Sine** — drive only DAC 1 or DAC 2.

Shared controls:

- **Amplitude / Center** — volts. Max safe amplitude is `min(center, 3.3 − center)`.
- **Start** — starts (or restarts) the waveform with current params.
- **Update Freqs** — changes only the PIO clock divider on a running
  waveform. No DMA restart, no phase glitch worth mentioning.
- **Stop** — ends DMA and zeroes the outputs.
- **Zero** — explicit zero (also stops any running DMA).
- **Wake** — releases the hardware SHDN line after an emergency stop.
- **EMERGENCY STOP** — pulls SHDN low on both DACs. Outputs are dead
  until you hit **Wake**.

## BLE command reference

All commands are UTF-8 text written to characteristic `0x2A56`, pipe
separated, no trailing newline required. Replies come back as GATT
notifications.

```
TI|f1|f2|amp|center_v              start temporal interference
BEAT|center_f|beat_f|amp|center_v  same, expressed as center + beat
UPDATE|f1|f2                       on-the-fly freq change (no DMA restart)
SINE1|f|amp|center_v               single sine on DAC 1
SINE2|f|amp|center_v               single sine on DAC 2
STOP                               graceful stop, zero outputs
ESTOP                              hardware SHDN emergency stop
WAKE                               release SHDN
ZERO                               zero outputs (also stops DMA)
STATUS                             reply with STATE notification
PING                               reply "PONG"
```

Reply formats:

```
OK|<echo of command>
ERR|<message>
STATE|running=0|f1=0.000|f2=0.000|beat=0.000|amp=0.000|center=0.000
PONG
```

## Safety notes

- A BLE disconnect automatically calls `stop()` on the Pico. A dropped
  link will never leave a stimulation signal running.
- `ESTOP` uses the dedicated SHDN GPIO (20) wired directly to both
  MCP4922s — it works even if the BLE stack is confused.
- The frontend's slider maxes are UI conveniences. The firmware will
  reject (via `ERR|...` notify) any frequency that would push SCK above
  20 MHz for the current LUT size.
