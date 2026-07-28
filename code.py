import board
import busio
import digitalio
import pwmio
import rotaryio
import time
import usb_hid
import json
import os
import microcontroller
import supervisor
from community_tca9555 import TCA9555
from serial_handler import SerialHandler

# Finish or clean up any interrupted OTA update. boot.py already ran this,
# but running it again here is idempotent and covers the transitional case of
# a new code.py under an old boot.py.
try:
    from update_recovery import recover
    recover()
except Exception as e:
    print("update recovery failed:", e)


def _gpio_pin(n):
    """Return the pin object for GP<n>, falling back to microcontroller.pin.GPIO<n>
    when the board module does not expose it (e.g. GP29 on Pico, which is reserved
    for VSYS monitoring at the board level but physically present on the chip)."""
    try:
        return getattr(board, f'GP{n}')
    except AttributeError:
        return getattr(microcontroller.pin, f'GPIO{n}')

# Report layout (32 bytes):
#   Bytes  0-15 : 128 buttons, 1 bit each.
#                 Bit N = button N (0-indexed).  B0=bit0, B1=bit1 … B127=bit127.
#                 B0 is never used by default passthrough, so D1→B1→bit1 shows
#                 as "button 1" in 0-indexed tools / game controllers.
#   Bytes 16-31 : 8 axes, 16-bit unsigned little-endian, 0-65535.
#                 Slot 1=X, 2=Y, 3=Z, 4=Rx, 5=Ry, 6=Rz, 7=Slider, 8=Dial.
REPORT_SIZE = 32
B_MAX       = 127

NVM_MAGIC = 0xA5
# NVM layout: [magic][num_bools][num_axes][bool bytes…][axis lo/hi pairs…]
NVM_HDR   = 3

# Quadrature encoder Gray-code transition table (software fallback for expander pins).
# Key: (prev_a, prev_b, curr_a, curr_b) → +1 CW / -1 CCW / absent = invalid/noise
_ENC_TABLE = {
    (0, 0, 1, 0):  1,  (1, 0, 1, 1):  1,  (1, 1, 0, 1):  1,  (0, 1, 0, 0):  1,
    (0, 0, 0, 1): -1,  (0, 1, 1, 1): -1,  (1, 1, 1, 0): -1,  (1, 0, 0, 0): -1,
}

# ---------------------------------------------------------------------------
# Board revision detection
# ---------------------------------------------------------------------------
# Each revision has a different I2C bus for the TCA9555 IO expander.
# At boot, code.py probes each known bus; the first one that finds a
# TCA9555 at 0x20 determines the board revision.

_I2C_BUSES = [("rev2", 0, 1), ("rev1", 6, 7)]


def _board_map(rev):
    pins = {}
    if rev == "rev1":
        for i in range(8):
            pins["D%d" % (i + 1)] = {"type": "expander", "pin": i}
        for i, p in enumerate(range(15, 9, -1)):
            pins["D%d" % (i + 9)] = {"type": "expander", "pin": p}
        for i in range(10):
            pins["D%d" % (i + 15)] = {"type": "gpio", "pin": i + 14}
        for i in range(3):
            pins["A%d" % (i + 6)] = {"type": "gpio", "pin": i + 27}
        return {"name": "rev1", "backlight": 12, "pins": pins}
    if rev == "rev2":
        for i in range(22):
            pins["D%d" % (i + 1)] = {"type": "gpio", "pin": i + 4}
        for i in range(4):
            pins["A%d" % (i + 1)] = {"type": "gpio", "pin": i + 26}
        for i in range(16):
            pins["D%d" % (i + 23)] = {"type": "expander", "pin": i}
        return {"name": "rev2", "backlight": 2, "pins": pins}
    raise ValueError("Unknown board revision: " + rev)


# ---------------------------------------------------------------------------
# NVM storage
# ---------------------------------------------------------------------------
class NVMStorage:
    """Persists bool and axis values across power cycles using microcontroller NVM.

    Values are cached in RAM and written to flash lazily — at most once every
    _FLUSH_INTERVAL seconds — so that frequent encoder ticks never block the main
    loop with repeated flash erase/program cycles.  Call flush(force=True) before
    any reset to guarantee no data is lost.

    The header (bytes 0-2) stores a magic byte and item counts so that adding or
    removing stored items in the config invalidates the old data automatically.
    Items are silently truncated if the config requests more storage than NVM can hold.
    """

    _FLUSH_INTERVAL = 5.0  # seconds between NVM flushes

    def __init__(self, stored_bools, stored_axes):
        self._bool_offsets = {}
        self._axis_offsets = {}
        self._bool_cache   = {}
        self._axis_cache   = {}
        self._dirty        = False
        self._last_flush   = 0.0

        if not stored_bools and not stored_axes:
            return  # nothing to persist — leave NVM completely untouched

        nvm       = microcontroller.nvm
        available = len(nvm) - NVM_HDR

        # Silently truncate if more storage is requested than NVM can hold.
        # The 255 cap matches the single-byte item counts in the header —
        # without it, byte 1/2 assignment below raises and the whole config
        # gets discarded by the parse fallback.
        max_bools    = min(len(stored_bools), available, 255)
        max_axes     = min(len(stored_axes),  (available - max_bools) // 2, 255)
        stored_bools = stored_bools[:max_bools]
        stored_axes  = stored_axes[:max_axes]

        offset = NVM_HDR
        for bid, _ in stored_bools:
            self._bool_offsets[bid] = offset
            offset += 1
        for aid, _ in stored_axes:
            self._axis_offsets[aid] = offset
            offset += 2

        valid = (nvm[0] == NVM_MAGIC
                 and nvm[1] == len(stored_bools)
                 and nvm[2] == len(stored_axes))

        if valid:
            for bid, _ in stored_bools:
                self._bool_cache[bid] = bool(nvm[self._bool_offsets[bid]])
            for aid, _ in stored_axes:
                off = self._axis_offsets[aid]
                self._axis_cache[aid] = nvm[off] | (nvm[off + 1] << 8)
        else:
            nvm[0] = NVM_MAGIC
            nvm[1] = len(stored_bools)
            nvm[2] = len(stored_axes)
            for bid, default in stored_bools:
                self._bool_cache[bid] = default
                nvm[self._bool_offsets[bid]] = 1 if default else 0
            for aid, default in stored_axes:
                self._axis_cache[aid] = default
                off = self._axis_offsets[aid]
                nvm[off]     = default & 0xFF
                nvm[off + 1] = (default >> 8) & 0xFF

    def read_bool(self, bid):
        return self._bool_cache.get(bid, False)

    def read_axis(self, aid):
        return self._axis_cache.get(aid, 32767)

    def write_bool(self, bid, value):
        if bid in self._bool_offsets and self._bool_cache.get(bid) != value:
            self._bool_cache[bid] = value
            self._dirty = True

    def write_axis(self, aid, value):
        value = max(0, min(65535, value))
        if aid in self._axis_offsets and self._axis_cache.get(aid) != value:
            self._axis_cache[aid] = value
            self._dirty = True

    def flush(self, force=False):
        """Commit dirty cache to NVM flash.

        Called from the main loop every cycle (no-op when clean or too soon) and
        with force=True before any microcontroller reset.
        """
        if not self._dirty:
            return
        now = time.monotonic()
        if not force and now - self._last_flush < self._FLUSH_INTERVAL:
            return
        nvm = microcontroller.nvm
        for bid, off in self._bool_offsets.items():
            nvm[off] = 1 if self._bool_cache.get(bid, False) else 0
        for aid, off in self._axis_offsets.items():
            val = self._axis_cache.get(aid, 0)
            nvm[off]     = val & 0xFF
            nvm[off + 1] = (val >> 8) & 0xFF
        self._dirty      = False
        self._last_flush = now


# ---------------------------------------------------------------------------
# Main button box class
# ---------------------------------------------------------------------------
class ButtonBox:

    def __init__(self):
        self.config = self._load_config()
        self.board_map = self._detect_board()
        self.pin_map = self.board_map.get("pins", {})
        self.pin_names = frozenset(self.pin_map.keys())
        self._init_hardware()
        self.nvm    = None   # set by _parse_config; pre-initialised so fallback is safe
        try:
            self._parse_config()
        except Exception as e:
            print("Config parse failed:", e)
            self.config = {"device": {}, "bools": [], "axes": [], "rules": []}
            self._parse_config()

        self.report            = bytearray(REPORT_SIZE)
        self.gamepad           = None
        self._needs_refresh    = False
        self._last_backlight   = -1       # force first PWM write
        self._last_report_time = 0.0
        self._acquire_hid()
        self._read_all_pins()
        self._init_encoder_states()
        self._init_rule_states()          # seed prev_input → no spurious edges on first cycle
        self._process_rules()             # apply MAP/NOR/etc. so the initial report is accurate
        self._apply_default_passthrough()
        self.report = self._build_report()
        self._send_report()

        # USB reconnect guard — see _check_usb_reconnect().
        self._usb_at_boot      = supervisor.runtime.usb_connected
        self._usb_connect_time = None

        self.serial = SerialHandler(self)

    # ------------------------------------------------------------------
    # Startup
    # ------------------------------------------------------------------

    def _load_config(self):
        try:
            with open("config.json") as f:
                return json.load(f)
        except Exception as e:
            print("Config load failed:", e)
            return {"device": {}, "bools": [], "axes": [], "rules": []}

    def _detect_board(self):
        for rev, sda_n, scl_n in _I2C_BUSES:
            try:
                i2c = busio.I2C(_gpio_pin(scl_n), _gpio_pin(sda_n))
                while not i2c.try_lock():
                    pass
                try:
                    found = i2c.scan()
                finally:
                    i2c.unlock()
                if 0x20 in found:
                    print("Detected board: %s (I2C GP%d/GP%d)" % (rev, sda_n, scl_n))
                    self.i2c = i2c
                    return _board_map(rev)
                i2c.deinit()
            except Exception:
                try:
                    i2c.deinit()
                except Exception:
                    pass
        raise RuntimeError("No TCA9555 found on any known I2C bus")

    def _init_hardware(self):
        self.expander = TCA9555(self.i2c)
        self.expander.configuration_port_0      = 0xFF
        self.expander.configuration_port_1      = 0xFF
        self.expander.polarity_inversion_port_0 = 0xFF
        self.expander.polarity_inversion_port_1 = 0xFF

        self.gpio_pins = {}
        for name, data in self.pin_map.items():
            if data['type'] == 'gpio':
                pin_obj = _gpio_pin(data["pin"])
                pin = digitalio.DigitalInOut(pin_obj)
                pin.direction = digitalio.Direction.INPUT
                pin.pull      = digitalio.Pull.UP
                self.gpio_pins[name] = pin

        bl_n = self.board_map.get("backlight")
        self.backlight_pwm = None
        if bl_n is not None:
            try:
                self.backlight_pwm = pwmio.PWMOut(_gpio_pin(bl_n), frequency=1000, duty_cycle=0)
            except Exception as e:
                print("Backlight PWM init failed:", e)

    def _parse_config(self):
        cfg = self.config
        dev = cfg.get("device", {})

        # Inactivity refresh: push one report after N seconds of no activity so
        # any app that connects after boot sees the current state within N seconds.
        # Set "inactivity_refresh": false to disable.
        _ir = dev.get("inactivity_refresh", 1.0)
        self._inactivity_refresh = None if (_ir is False or _ir is None) else float(_ir)

        # Debounce: a D-pin state change is only committed to pin_cache once the new
        # level has been stable for this many milliseconds.  The 200Hz sample rate
        # already provides 5ms of implicit filtering, so that is the minimum effective
        # value.  10-20ms is good for clicky switches; 50ms matches the old firmware.
        self._debounce_s = max(dev.get("debounce_ms", 10), 0) / 1000.0

        # Cache the active rules list up-front (comment-only entries stripped).
        self.rules = [r for r in cfg.get("rules", []) if r.get("type")]

        # NVM — only initialised once per boot.  If _parse_config is called a
        # second time (config fallback), self.nvm is already set and is left alone
        # so that stored values are not wiped by the fallback empty config.
        if self.nvm is None:
            stored_bools = [(b["id"], b.get("default", False))
                            for b in cfg.get("bools", [])
                            if "id" in b and b.get("store", False)]
            stored_axes  = [(a["id"], a.get("default", 32767))
                            for a in cfg.get("axes",  [])
                            if "id" in a and a.get("store", False)]
            self.nvm             = NVMStorage(stored_bools, stored_axes)
            self.stored_bool_ids = {bid for bid, _ in stored_bools}
            self.stored_axis_ids = {aid for aid, _ in stored_axes}

        # Bool states
        self.bool_states = {}
        for b in cfg.get("bools", []):
            if "id" not in b:
                continue
            bid = b["id"]
            self.bool_states[bid] = (self.nvm.read_bool(bid)
                                     if b.get("store", False)
                                     else b.get("default", False))

        # Axis states
        # axis_output_slot: int 1-8 → HID slot, None → BACKLIGHT-only (no HID).
        # An axis can be both a HID axis and drive the backlight with "backlight": true.
        self.axis_states      = {}
        self.axis_output_slot = {}
        self.backlight_axis   = None
        for a in cfg.get("axes", []):
            if "id" not in a:
                continue
            aid    = a["id"]
            output = a.get("output", 1)
            try:
                default = int(a.get("default", 32767))
            except (TypeError, ValueError):
                default = 32767
            self.axis_states[aid] = (self.nvm.read_axis(aid)
                                     if a.get("store", False)
                                     else default)
            if str(output).upper() == "BACKLIGHT":
                self.axis_output_slot[aid] = None
                self.backlight_axis        = aid
            else:
                slot = int(output)
                self.axis_output_slot[aid] = slot if 1 <= slot <= 8 else None
                if a.get("backlight", False):
                    self.backlight_axis = aid

        # Per-rule runtime state
        self.b_states             = {}
        self.claimed_b            = set()
        self._pin_raw             = {}    # last raw reading per D-pin
        self._pin_change_time     = {}    # time the raw reading last differed from stable
        # Single-cycle encoder outputs (pulse_ms=0) are zeroed at the start of every
        # cycle. Downstream AXIS rules that feed from them also get their prev_input
        # reset, which lets back-to-back ticks each trigger exactly one rising edge.
        self.encoder_b_outputs    = set()    # B-number ints
        self.encoder_bool_outputs = set()    # BOOL id strings
        self.rule_prev_input      = {}
        # Hardware encoder state: rule_idx → (rotaryio.IncrementalEncoder, pulse_s)
        # Software encoder state: rule_idx → (prev_a, prev_b, pulse_s)
        self.hw_encoders          = {}
        self.hw_encoder_positions = {}       # rule_idx → last known position
        self.sw_encoder_states    = {}       # rule_idx → (prev_a, prev_b, pulse_s, accum)
        self.sw_encoder_cfg       = {}       # rule_idx → (name_a, name_b, divisor, invert) — cached for tight polling
        self._sw_enc_pending      = {}       # rule_idx → signed int; accumulated steps between _process_rules drains
        self.pending_delays       = []
        self.pin_cache            = {}
        self._enc_axis_rules      = set()    # AXIS rules fed by single-cycle encoder outputs
        # encoder_axis_links: rule_idx (ENCODER) → [(trigger_dir, axis_id, step, is_inc)]
        # Stores AXIS_INC/DEC rules that are directly driven by an encoder output so
        # multi-step deltas (encoder moved several detents in one cycle) are applied
        # correctly rather than capping at one step per cycle.
        self.encoder_axis_links      = {}
        self.encoder_linked_axis_rules = set()   # these AXIS rules are skipped in the main loop

        # Pass 1 — collect encoder outputs for:
        #   a) single-cycle zero-at-start mechanism (pulse_ms=0)
        #   b) encoder→axis link resolution in pass 2
        # encoder_output_to_enc: output_ref → (enc_rule_idx, trigger_direction +1/-1)
        encoder_output_to_enc = {}
        for enc_idx, rule in enumerate(self.rules):
            if rule.get("type") != "ENCODER":
                continue
            pulse_s = rule.get("pulse_ms", 0) / 1000.0
            for key, trig_dir in (("cw", 1), ("ccw", -1)):
                ref = rule.get(key, "")
                if not ref:
                    continue
                encoder_output_to_enc[ref] = (enc_idx, trig_dir)
                if pulse_s == 0:
                    if ref.startswith("B") and ref[1:].isdigit():
                        self.encoder_b_outputs.add(int(ref[1:]))
                    elif ref in self.bool_states:
                        self.encoder_bool_outputs.add(ref)

        # Initialise hardware (rotaryio) or software encoders.
        # rotaryio on RP2040 uses PIO and requires both pins to be GPIO *and*
        # their GP numbers to be consecutive (checked internally by
        # common_hal_rp2pio_pins_are_sequential).  Normally that mismatch raises
        # RuntimeError, but in some CircuitPython builds it hard-faults the core
        # before the exception is raised (adafruit/circuitpython#10583) — so we
        # gate on sequential numbering ourselves before ever calling rotaryio.
        # Any non-sequential or mixed-expander pair falls back to the software
        # Gray-code path, which reads from the already-claimed DigitalInOut pins.
        # IMPORTANT: _init_hardware() already claimed all GPIO pins as DigitalInOut.
        # Those pins must be deinit'd before rotaryio can take ownership of them.
        for enc_idx, rule in enumerate(self.rules):
            if rule.get("type") != "ENCODER":
                continue
            inputs  = rule.get("inputs", [])
            pulse_s = rule.get("pulse_ms", 0) / 1000.0

            both_gpio = (len(inputs) == 2
                         and self.pin_map.get(inputs[0], {}).get("type") == "gpio"
                         and self.pin_map.get(inputs[1], {}).get("type") == "gpio")
            sequential = (both_gpio
                          and abs(self.pin_map[inputs[0]]["pin"]
                                  - self.pin_map[inputs[1]]["pin"]) == 1)

            if both_gpio and sequential:
                try:
                    # Release DigitalInOut ownership so rotaryio can claim the pins.
                    for inp in inputs:
                        if inp in self.gpio_pins:
                            self.gpio_pins.pop(inp).deinit()
                    pin_a   = _gpio_pin(self.pin_map[inputs[0]]["pin"])
                    pin_b   = _gpio_pin(self.pin_map[inputs[1]]["pin"])
                    divisor = rule.get("divisor", 2)
                    enc     = rotaryio.IncrementalEncoder(pin_a, pin_b, divisor=divisor)
                    self.hw_encoders[enc_idx]          = (enc, pulse_s)
                    self.hw_encoder_positions[enc_idx] = enc.position
                except Exception as e:
                    print("rotaryio encoder init failed:", e)
                    # Pins were deinit'd above — re-claim them as DigitalInOut
                    # so the software encoder path can actually read them.
                    for inp in inputs:
                        if inp not in self.gpio_pins:
                            pin_obj       = _gpio_pin(self.pin_map[inp]["pin"])
                            pin           = digitalio.DigitalInOut(pin_obj)
                            pin.direction = digitalio.Direction.INPUT
                            pin.pull      = digitalio.Pull.UP
                            self.gpio_pins[inp] = pin
                    self.sw_encoder_states[enc_idx] = (0, 0, pulse_s, 0)
            else:
                if both_gpio and not sequential:
                    print("ENCODER", inputs, "GPIOs not sequential — using software path")
                self.sw_encoder_states[enc_idx] = (0, 0, pulse_s, 0)

        # Pass 2 — claimed_b, per-rule state, enc_axis_rules, encoder_axis_links.
        for i, rule in enumerate(self.rules):
            rtype = rule.get("type", "")

            if rtype == "ENCODER":
                for key in ("cw", "ccw"):
                    ref = rule.get(key, "")
                    if ref.startswith("B") and ref[1:].isdigit():
                        self.claimed_b.add(int(ref[1:]))
            else:
                out = rule.get("output", "")
                if out.startswith("B") and out[1:].isdigit():
                    self.claimed_b.add(int(out[1:]))

                if rtype in ("AXIS_INC", "AXIS_DEC"):
                    ref = rule.get("input", "")
                    # Check if this AXIS rule feeds from an encoder output.
                    if ref in encoder_output_to_enc:
                        enc_idx, trig_dir = encoder_output_to_enc[ref]
                        is_inc = (rtype == "AXIS_INC")
                        self.encoder_axis_links.setdefault(enc_idx, []).append(
                            (trig_dir, rule.get("axis", ""), rule.get("step", 1), is_inc)
                        )
                        self.encoder_linked_axis_rules.add(i)
                    # Also track for single-cycle zero-at-start mechanism.
                    if ref.startswith("B") and ref[1:].isdigit():
                        if int(ref[1:]) in self.encoder_b_outputs:
                            self._enc_axis_rules.add(i)
                    elif ref in self.encoder_bool_outputs:
                        self._enc_axis_rules.add(i)

            self.rule_prev_input[i] = False

        # Cache per-rule inputs/divisor/invert for software encoders so the
        # tight-poll inner loop doesn't repeat dict lookups on every sample.
        for enc_idx in self.sw_encoder_states:
            r      = self.rules[enc_idx]
            inputs = r.get("inputs", [])
            self.sw_encoder_cfg[enc_idx] = (
                inputs[0] if len(inputs) >= 1 else "",
                inputs[1] if len(inputs) >= 2 else "",
                r.get("divisor", 2),
                r.get("invert", False),
            )

    def _acquire_hid(self):
        try:
            self.gamepad = usb_hid.devices[0]
        except Exception:
            self.gamepad = None

    def _init_encoder_states(self):
        """Seed software encoder states from actual pin levels.
        Hardware (rotaryio) encoders need no seeding — they track position continuously.
        """
        for i, rule in enumerate(self.rules):
            if rule.get("type") == "ENCODER" and i in self.sw_encoder_states:
                inputs = rule.get("inputs", [])
                if len(inputs) >= 2:
                    a_v = int(self._read_encoder_pin(inputs[0]))
                    b_v = int(self._read_encoder_pin(inputs[1]))
                    _, _, pulse_s, accum = self.sw_encoder_states.get(i, (0, 0, 0, 0))
                    self.sw_encoder_states[i] = (a_v, b_v, pulse_s, accum)

    def _init_rule_states(self):
        """Seed rule_prev_input from actual pin states before the first process pass.

        Without this, any input already active at power-on would look like a rising
        edge on the very first cycle — triggering TOGGLEs, queuing DELAYs, and
        stepping axes even though nothing actually changed.
        """
        for i, rule in enumerate(self.rules):
            if rule.get("type") in ("TOGGLE", "PULSE", "AXIS_INC", "AXIS_DEC"):
                self.rule_prev_input[i] = self._read_input(rule.get("input", ""))

    # ------------------------------------------------------------------
    # Pin reading
    # ------------------------------------------------------------------

    def _read_all_pins(self):
        try:
            port0 = self.expander.input_port_0
            port1 = self.expander.input_port_1
        except Exception:
            port0, port1 = 0, 0

        now = time.monotonic()
        for name, data in self.pin_map.items():
            if data['type'] == 'gpio':
                pin = self.gpio_pins.get(name)
                # Encoder GPIO pins are owned by rotaryio and are absent from gpio_pins.
                raw = (not pin.value) if pin else False
            else:
                pnum = data['pin']
                raw  = bool((port0 if pnum < 8 else port1) & (1 << (pnum if pnum < 8 else pnum - 8)))

            prev_raw = self._pin_raw.get(name)
            if prev_raw is None:
                # First reading — commit immediately, no timer needed.
                self._pin_raw[name]         = raw
                self._pin_change_time[name] = now
                self.pin_cache[name]        = raw
            elif raw != prev_raw:
                # Level changed — restart the debounce timer, don't commit yet.
                self._pin_raw[name]         = raw
                self._pin_change_time[name] = now
            elif now - self._pin_change_time[name] >= self._debounce_s:
                # Stable long enough — commit.
                self.pin_cache[name] = raw

    # ------------------------------------------------------------------
    # Input / output helpers
    # ------------------------------------------------------------------

    def _read_input(self, ref):
        if ref in self.pin_cache:
            return self.pin_cache[ref]
        if ref.startswith("B") and ref[1:].isdigit():
            return self.b_states.get(int(ref[1:]), False)
        return self.bool_states.get(ref, False)

    def _read_encoder_pin(self, name):
        """Read a pin live, bypassing pin_cache's debounce.

        Debouncing would drop legitimate encoder edges that arrive inside the
        debounce window; rotaryio doesn't debounce on the hardware path either.
        Bounce is handled implicitly by the Gray-code table (invalid 4-tuples
        map to direction=0).
        """
        data = self.pin_map.get(name)
        if data is None:
            return False
        if data['type'] == 'gpio':
            pin = self.gpio_pins.get(name)
            return (not pin.value) if pin else False
        pnum = data['pin']
        try:
            port = self.expander.input_port_0 if pnum < 8 else self.expander.input_port_1
        except Exception:
            return False
        bit = pnum if pnum < 8 else pnum - 8
        return bool(port & (1 << bit))

    def _tick_sw_encoders(self):
        """Poll every software-path encoder once, live (no debounce), and
        accumulate completed divisor-scaled steps into self._sw_enc_pending.

        Called many times per main-loop cycle (tight inner loop during what
        used to be time.sleep) so that rapid encoder rotation — where both A
        and B can flip between two 200 Hz cycles, turning a valid Gray
        transition into an invalid (0,0)→(1,1) jump and dropping the tick —
        is captured. Hardware (rotaryio) encoders are interrupt-driven and
        never need this.
        """
        for i, (name_a, name_b, divisor, invert) in self.sw_encoder_cfg.items():
            prev_a, prev_b, pulse_s, accum = self.sw_encoder_states[i]
            curr_a = int(self._read_encoder_pin(name_a))
            curr_b = int(self._read_encoder_pin(name_b))
            raw_dir = _ENC_TABLE.get((prev_a, prev_b, curr_a, curr_b), 0)
            if invert:
                raw_dir = -raw_dir
            accum += raw_dir
            pending = self._sw_enc_pending.get(i, 0)
            while accum >= divisor:
                accum   -= divisor
                pending += 1
            while accum <= -divisor:
                accum   += divisor
                pending -= 1
            if pending:
                self._sw_enc_pending[i] = pending
            self.sw_encoder_states[i] = (curr_a, curr_b, pulse_s, accum)

    def _write_output(self, ref, value):
        if ref.startswith("B") and ref[1:].isdigit():
            self.b_states[int(ref[1:])] = value
        elif ref in self.bool_states:
            if self.bool_states[ref] != value:
                self.bool_states[ref] = value
                if ref in self.stored_bool_ids:
                    self.nvm.write_bool(ref, value)
        elif ref == "REFRESH" and value:
            self._needs_refresh = True

    # ------------------------------------------------------------------
    # Rule engine
    # ------------------------------------------------------------------

    def _apply_encoder_output(self, ref, pulse_s, now):
        """Fire an encoder output (B or BOOL) and schedule its clear if pulse_ms > 0."""
        self._write_output(ref, True)
        if pulse_s > 0:
            self.pending_delays = [(t, o, v) for t, o, v in self.pending_delays if o != ref]
            self.pending_delays.append((now + pulse_s, ref, False))

    def _apply_encoder_axes(self, enc_idx, direction, steps):
        """Apply a multi-step encoder delta directly to all linked axes."""
        for trig_dir, axis_id, step, is_inc in self.encoder_axis_links.get(enc_idx, []):
            if trig_dir != direction:
                continue
            val = self.axis_states.get(axis_id, 32767)
            val += (steps * step) if is_inc else -(steps * step)
            # int(): a float step (possible in a hand-edited config) must not
            # poison axis_states — _build_report's byte math needs ints.
            self.axis_states[axis_id] = int(max(0, min(65535, val)))
            if axis_id in self.stored_axis_ids:
                self.nvm.write_axis(axis_id, self.axis_states[axis_id])

    def _process_rules(self):
        now = time.monotonic()

        # Zero single-cycle encoder outputs so each tick is a fresh rising edge.
        # Direct assignment skips _write_output to avoid NVM writes every cycle.
        for bnum in self.encoder_b_outputs:
            self.b_states[bnum] = False
        for ref in self.encoder_bool_outputs:
            self.bool_states[ref] = False
        for i in self._enc_axis_rules:
            self.rule_prev_input[i] = False

        # Apply expired delays.
        if self.pending_delays:
            remaining = []
            for apply_time, out_ref, value in self.pending_delays:
                if now >= apply_time:
                    self._write_output(out_ref, value)
                else:
                    remaining.append((apply_time, out_ref, value))
            self.pending_delays = remaining

        for i, rule in enumerate(self.rules):
            rtype  = rule.get("type", "")
            invert = rule.get("invert", False)

            if rtype == "MAP":
                val = self._read_input(rule.get("input", ""))
                if invert:
                    val = not val
                self._write_output(rule.get("output", ""), val)

            elif rtype == "NOR":
                result = not any(self._read_input(inp) for inp in rule.get("inputs", []))
                if invert:
                    result = not result
                self._write_output(rule.get("output", ""), result)

            elif rtype == "TOGGLE":
                ref  = rule.get("input",  "")
                out  = rule.get("output", "")
                curr = self._read_input(ref)
                prev = self.rule_prev_input[i]
                triggered = (not curr and prev) if invert else (curr and not prev)
                if triggered:
                    self._write_output(out, not self._read_input(out))
                self.rule_prev_input[i] = curr

            elif rtype == "PULSE":
                ref  = rule.get("input",  "")
                out  = rule.get("output", "")
                curr = self._read_input(ref)
                prev = self.rule_prev_input[i]
                triggered = (not curr and prev) if invert else (curr and not prev)
                if triggered:
                    delay_s = rule.get("delay_ms", 0) / 1000.0
                    pulse_s = rule.get("pulse_ms", 100) / 1000.0
                    self.pending_delays = [(t, o, v) for t, o, v in self.pending_delays
                                           if o != out]
                    if delay_s > 0 and self._read_input(out):
                        # Output is still high from a previous pulse — snap it off
                        # immediately so the retrigger feels responsive.
                        self._write_output(out, False)
                    if delay_s == 0:
                        self._write_output(out, True)
                        self.pending_delays.append((now + pulse_s, out, False))
                    else:
                        self.pending_delays.append((now + delay_s, out, True))
                        self.pending_delays.append((now + delay_s + pulse_s, out, False))
                self.rule_prev_input[i] = curr

            elif rtype == "ENCODER":
                inputs = rule.get("inputs", [])
                if len(inputs) < 2:
                    continue

                if i in self.hw_encoders:
                    # Hardware path — rotaryio interrupt-driven, never misses a step.
                    enc, pulse_s = self.hw_encoders[i]
                    pos   = enc.position
                    delta = pos - self.hw_encoder_positions[i]
                    self.hw_encoder_positions[i] = pos
                    if invert:
                        delta = -delta
                    direction = 1 if delta > 0 else (-1 if delta < 0 else 0)
                    steps = abs(delta)
                else:
                    # Software path — drain steps accumulated by the tight
                    # poll loop (_tick_sw_encoders). All live pin reads and
                    # Gray-code decoding happen there, at sub-millisecond rate.
                    pending   = self._sw_enc_pending.pop(i, 0)
                    _, _, pulse_s, _ = self.sw_encoder_states[i]
                    direction = 1 if pending > 0 else (-1 if pending < 0 else 0)
                    steps     = abs(pending)

                if direction:
                    ref = rule.get("cw" if direction == 1 else "ccw", "")
                    if ref:
                        self._apply_encoder_output(ref, pulse_s, now)
                    # Apply multi-step delta directly to linked axes so fast
                    # spinning never loses steps (each step is always counted).
                    self._apply_encoder_axes(i, direction, steps)

            elif rtype in ("AXIS_INC", "AXIS_DEC"):
                # Encoder-linked AXIS rules are handled inside the ENCODER block above
                # with full multi-step support; skip them here to avoid double-stepping.
                if i in self.encoder_linked_axis_rules:
                    continue
                ref     = rule.get("input", "")
                axis_id = rule.get("axis",  "")
                curr    = self._read_input(ref)
                prev    = self.rule_prev_input[i]
                if curr and not prev:
                    step = rule.get("step", 1)
                    val  = self.axis_states.get(axis_id, 32767)
                    val  = min(65535, val + step) if rtype == "AXIS_INC" else max(0, val - step)
                    self.axis_states[axis_id] = int(val)
                    if axis_id in self.stored_axis_ids:
                        self.nvm.write_axis(axis_id, val)
                self.rule_prev_input[i] = curr

    # ------------------------------------------------------------------
    # Default passthrough  Dn → Bn  for any B not claimed by a rule
    # ------------------------------------------------------------------

    def _apply_default_passthrough(self):
        for name in self.pin_map:
            if not name.startswith("D"):
                continue
            n = int(name[1:])
            if n not in self.claimed_b:
                self.b_states[n] = self.pin_cache.get(name, False)

    # ------------------------------------------------------------------
    # Report building
    # ------------------------------------------------------------------

    def _build_report(self):
        report = bytearray(REPORT_SIZE)

        for bnum, state in self.b_states.items():
            if state and 0 <= bnum <= B_MAX:
                report[bnum >> 3] |= 1 << (bnum & 7)

        slot_values = {}
        for aid, val in self.axis_states.items():
            slot = self.axis_output_slot.get(aid)
            if slot is not None:
                slot_values[slot] = val
        for slot in range(1, 9):
            val = slot_values.get(slot, 32767)
            off = 16 + (slot - 1) * 2
            report[off]     = val & 0xFF
            report[off + 1] = (val >> 8) & 0xFF

        return report

    # ------------------------------------------------------------------
    # Backlight
    # ------------------------------------------------------------------

    def _apply_backlight(self):
        """Write the current backlight level to PWM if it has changed.

        Brightness curve: t² + 0.01 (quadratic, matching human perception).
        The +0.01 offset ensures the first encoder click produces a visible glow
        rather than nothing.  Zero stays zero (off).
        """
        if self.backlight_pwm is None or self.backlight_axis is None:
            return
        value = self.axis_states.get(self.backlight_axis, 0)
        if value == self._last_backlight:
            return
        self._last_backlight = value
        t = value / 65535.0
        if t > 0:
            t = min(t * t + 0.01, 1.0)
        self.backlight_pwm.duty_cycle = int(t * 65535)

    # ------------------------------------------------------------------
    # HID sending — tolerant of USB not being ready
    # ------------------------------------------------------------------

    def _send_report(self):
        if self.gamepad is None:
            self._acquire_hid()
        if self.gamepad:
            try:
                self.gamepad.send_report(self.report)
            except Exception:
                self.gamepad = None

    # ------------------------------------------------------------------
    # USB reconnect guard
    # ------------------------------------------------------------------

    def _check_usb_reconnect(self):
        """Reset if the USB host appears after a no-USB boot.

        When the RP2040 boots without a USB host present (e.g. powered from a
        hub before the PC is ready), the USB enumeration never completes.
        Detecting the host and doing a clean reset lets boot.py re-run with the
        host already up, guaranteeing a successful enumeration.  After that
        reset _usb_at_boot is True and this method becomes a no-op forever.
        """
        if self._usb_at_boot:
            return
        if supervisor.runtime.usb_connected:
            if self._usb_connect_time is None:
                self._usb_connect_time = time.monotonic()
            elif time.monotonic() - self._usb_connect_time >= 1.0:
                if self.nvm is not None:
                    self.nvm.flush(force=True)
                microcontroller.reset()
        else:
            self._usb_connect_time = None

    # ------------------------------------------------------------------
    # Main update
    # ------------------------------------------------------------------

    _CYCLE_S = 0.005   # 200 Hz main-loop cadence

    def update(self):
        cycle_start = time.monotonic()
        self._check_usb_reconnect()
        self.serial.process()
        self._read_all_pins()
        # Catch anything that happened during the previous cycle's tight poll.
        if self.sw_encoder_cfg:
            self._tick_sw_encoders()
        self._process_rules()
        if self.nvm is not None:
            self.nvm.flush()
        self._apply_default_passthrough()
        self._apply_backlight()

        now = time.monotonic()
        if (self._inactivity_refresh is not None
                and now - self._last_report_time >= self._inactivity_refresh):
            self._needs_refresh = True

        new_report = self._build_report()
        if new_report != self.report or self._needs_refresh:
            self.report            = new_report
            self._needs_refresh    = False
            self._last_report_time = now
            self._send_report()
        elif self.gamepad is None:
            self._acquire_hid()
            self._last_report_time = now
            self._send_report()

        self.serial.maybe_send_stream()

        # Wait out the rest of the 5 ms cycle. If there are software-path
        # encoders, spend that time tight-polling them at ~sub-millisecond rate
        # so fast rotations don't slip between 200 Hz samples and get decoded
        # as invalid Gray transitions. Otherwise just sleep.
        deadline = cycle_start + self._CYCLE_S
        if self.sw_encoder_cfg:
            while time.monotonic() < deadline:
                self._tick_sw_encoders()
        else:
            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
# The loop body is guarded: one bad config value or transient hardware error
# must not end code.py, because boot.py disables the USB drive and a dead
# code.py would leave the desktop app with no way to push a fix. After
# repeated consecutive failures the box falls back to a serial-only loop that
# keeps OTA recovery reachable.
button_box = ButtonBox()
_fail_streak = 0
while True:
    try:
        # update() owns the 200 Hz cadence itself — it either sleeps the
        # remainder of the cycle or tight-polls software encoders.
        button_box.update()
        _fail_streak = 0
    except Exception as e:
        _fail_streak += 1
        if _fail_streak <= 3 or _fail_streak % 200 == 0:
            print("update() failed (%d):" % _fail_streak, e)
        if _fail_streak >= 200:
            break
        time.sleep(0.005)

print("main loop abandoned — serial-only recovery mode")
while True:
    try:
        button_box.serial.process()
    except Exception:
        pass
    time.sleep(0.01)
