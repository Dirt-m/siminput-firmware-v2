# SIMINPUT Firmware

The firmware that runs the SIMINPUT button box. Configure everything in your browser, flash it once, and the box behaves exactly how you set it up. No code changes, no drivers.

This is CircuitPython firmware for an RP2040 (Raspberry Pi Pico) with a TCA9555 I/O expander. The device presents as a standard USB joystick with 128 buttons and 8 axes. A single JSON config file drives all the behaviour, so the same firmware fits any layout you build.

Part of the open SIMINPUT ecosystem. The firmware, hardware, enclosure CAD, and PCB are all public. Learn more and configure your own box at [siminput.com](https://siminput.com).

## Features

- **Up to 42 physical inputs** depending on board revision (see Hardware below).
- **Rule engine.** MAP, NOR, TOGGLE, PULSE, ENCODER, AXIS_INC, AXIS_DEC.
- **Rotary encoders.** Hardware decoding (rotaryio/PIO) with automatic software fallback.
- **Persistent storage.** Bools and axis values survive across power cycles using NVM.
- **PWM backlight.** Perceptual brightness curve, driven by any axis.
- **Fast loop.** 200 Hz main loop; GPIO encoders poll at sub millisecond rate, expander encoders at roughly 1 ms per sample (I2C bound).
- **Serial protocol.** JSON over USB CDC for configuration and live monitoring.
- **OTA updates.** Firmware files push over serial with power-loss-safe installs and verified transfers.
- **Auto detection.** Probes I2C at boot to pick the correct pin map for the board revision. A missing expander degrades to GPIO-only inputs and is reported over serial instead of disabling the box.

## Hardware

- **MCU:** Raspberry Pi Pico (RP2040)
- **I/O expander:** TCA9555 on I2C (address 0x20)
- **rev1:** 27 inputs. D1 to D14 on the expander, D15 to D24 direct GPIO, A6 to A8. Backlight PWM on GP12.
- **rev2:** 42 inputs. D1 to D22 direct GPIO, A1 to A4, D23 to D38 on the expander. Backlight PWM on GP2.
- All inputs use internal pull-ups, active low (switch closes to GND). A-pins are read as digital inputs like the D-pins, but they are not part of the default passthrough: use them as explicit rule inputs.

## Project layout

```
boot.py              USB descriptor setup (HID gamepad plus CDC serial)
code.py              Main firmware: init, config parsing, rule engine, HID loop
config.json          Default device config (minimal, pure passthrough)
build-package.sh     Builds the firmware update ZIP
configs/
  config-example.json   Example config demonstrating every rule type
lib/
  serial_handler.py     Serial command handler (JSON over CDC)
  community_tca9555.mpy  TCA9555 I2C expander driver
  adafruit_bus_device/   I2C communication helper
  adafruit_register/     Register access helpers
```

## Setup

1. Install [CircuitPython 9.x](https://circuitpython.org/board/raspberry_pi_pico/) on your Pico.
2. Copy all files to the CIRCUITPY drive: `boot.py`, `code.py`, `config.json`, and the `lib/` folder.
3. The device shows up as a USB joystick with 128 buttons and 8 axes.

Note: after the first boot the CIRCUITPY drive is disabled (the box presents as a clean joystick, not a flash drive). From then on, configuration and firmware updates go over the serial protocol via the desktop configurator. To get the drive back for manual file access, reflash CircuitPython over BOOTSEL or use the REPL on the console serial port.

Every pin passes through as a button by default (D1 to B1, D2 to B2, and so on), so nothing needs configuring to get going. Edit `config.json` to add rules, encoders, axes, and toggles.

## Configuration

The device is configured entirely through `config.json`. See `configs/config-example.json` for a full example.

### Device settings

```json
{
    "device": {
        "name": "My Button Box",
        "pid": 61440,
        "inactivity_refresh": 1.0,
        "debounce_ms": 10
    }
}
```

| Field | Description |
|-------|-------------|
| `name` | USB product name (max 32 chars) |
| `pid` | USB Product ID (default 61440 / 0xF000, must not be 0x80F4) |
| `inactivity_refresh` | Seconds between keep alive HID reports (`false` to disable) |
| `debounce_ms` | Debounce filter in milliseconds (default 10) |

### Bools

Named booleans for toggle states. Optionally persisted in NVM.

```json
"bools": [
    { "id": "TOGGLE1", "default": false, "store": true }
]
```

### Axes

Up to 8 HID axis outputs (X, Y, Z, Rx, Ry, Rz, Slider, Dial), plus a backlight only mode. Values are 16 bit unsigned (0 to 65535, center at 32767).

```json
"axes": [
    { "id": "AX1", "output": 1, "default": 32767, "store": true, "backlight": true }
]
```

Set `"output": "BACKLIGHT"` to drive only the backlight PWM with no HID output.

### Rules

Rules run every cycle (200 Hz) in order. Any pin not claimed by a rule automatically passes through (Dn to Bn).

| Type | Description |
|------|-------------|
| **MAP** | Direct input to output mapping, with optional invert |
| **NOR** | Output is true only when all inputs are false (3-way switch middle position) |
| **TOGGLE** | Flips output on each rising edge of input |
| **PULSE** | Fires output for `pulse_ms` after optional `delay_ms` on rising edge |
| **ENCODER** | Reads a quadrature encoder, produces CW/CCW outputs |
| **AXIS_INC / AXIS_DEC** | Adjusts an axis value by `step` on rising edge |

#### Examples

**Inverted button:**
```json
{ "type": "MAP", "input": "D1", "output": "B50", "invert": true }
```

**3-way switch** (D3 is up, D4 is down, NOR is middle):
```json
{ "type": "NOR", "inputs": ["D3", "D4"], "output": "B30" }
```

**Toggle button** with persistent state:
```json
{ "type": "TOGGLE", "input": "D5", "output": "TOGGLE1" },
{ "type": "MAP", "input": "TOGGLE1", "output": "B100" }
```

**Rotary encoder** driving an axis and backlight:
```json
{ "type": "ENCODER", "inputs": ["D17", "D18"], "cw": "B17", "ccw": "B18" },
{ "type": "AXIS_INC", "input": "B17", "axis": "AX1", "step": 2048 },
{ "type": "AXIS_DEC", "input": "B18", "axis": "AX1", "step": 2048 }
```

## Serial protocol

The device exposes a second USB CDC serial port for configuration and monitoring. Commands are line delimited JSON.

| Command | Description |
|---------|-------------|
| `ping` | Connection test and device discovery |
| `get_info` | Hardware and config details |
| `get_config` | Read current config |
| `set_config` | Write new config (validates, saves, reboots) |
| `validate_config` | Validate config without saving |
| `get_state` | Snapshot of all buttons, axes, pins, bools |
| `stream_start` | Begin live state streaming |
| `stream_stop` | Stop live streaming |
| `file_write` | Write a file to the device (for firmware updates) |
| `file_read` | Read a file from the device |
| `reboot` | Soft reboot (`{"hard": true}` for a full chip reset that re-runs boot.py) |
| `bootloader` | Enter UF2 bootloader mode |
| `update_begin` / `update_commit` / `update_abort` | Staged (transactional) firmware updates |

Requests may carry an `"id"` field, echoed on the reply. `get_info` reports the protocol revision, capabilities, transfer limits, and the board's pin list.

Example:
```
-> {"cmd": "ping"}
<- {"ok": true, "product": "SIMINPUT", "version": "2.6.0", "protocol": 2, "name": "My Button Box", "pid": 61440, "board_map": "rev2"}
```

## Firmware packaging

Run `./build-package.sh` to create a firmware update ZIP. The ZIP contains a manifest and all firmware files. It deliberately leaves out `config.json` so installing an update can never overwrite your own configuration.

When bumping versions, update `FW_VERSION` in `lib/serial_handler.py` and `VERSION` in `build-package.sh`.

## License

MIT. See [LICENSE](LICENSE).
