"""Rule engine, timing, encoder, and NVM tests for code.py.

Headless: CircuitPython modules are stubbed, the real firmware code runs on
desktop Python. Run from the repo root: python3 tests/test_runtime.py
"""
import os
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SANDBOX = tempfile.mkdtemp(prefix="siminput-fw-test-")

import json
import os
import shutil
import sys
import time as _time
import types

WORKTREE = str(REPO)


# ---- CircuitPython stubs ----------------------------------------------------
NVM = bytearray(4096)

mc = types.ModuleType("microcontroller")
mc.nvm = NVM
mc.reset = lambda: None
mc.on_next_reset = lambda *a: None
mc.RunMode = types.SimpleNamespace(BOOTLOADER=None)
mc.pin = types.SimpleNamespace(**{f"GPIO{i}": f"GPIO{i}" for i in range(30)})
sys.modules["microcontroller"] = mc

sv = types.ModuleType("supervisor")
sv.ticks_ms = lambda: int(_time.monotonic() * 1000) % (1 << 29)
sv.reload = lambda: None
sv.runtime = types.SimpleNamespace(usb_connected=True)
sys.modules["supervisor"] = sv

bd = types.ModuleType("board")
for i in range(30):
    setattr(bd, f"GP{i}", f"GP{i}")
sys.modules["board"] = bd

bus = types.ModuleType("busio")


class FailI2C:
    def __init__(self, *a, **k):
        raise RuntimeError("no i2c in test")


bus.I2C = FailI2C
sys.modules["busio"] = bus

PIN_LEVELS: dict[str, bool] = {}  # pin object name -> pressed (True = closed to GND)

dio = types.ModuleType("digitalio")


class _Dir:
    INPUT = "in"


class _Pull:
    UP = "up"


class DigitalInOut:
    def __init__(self, pin_obj):
        self.pin_obj = pin_obj
        self.direction = None
        self.pull = None

    @property
    def value(self):
        # pull-up semantics: True = open (not pressed)
        return not PIN_LEVELS.get(self.pin_obj, False)

    def deinit(self):
        pass


dio.DigitalInOut = DigitalInOut
dio.Direction = _Dir
dio.Pull = _Pull
sys.modules["digitalio"] = dio

ADC_LEVELS: dict[str, int] = {}  # pin object name -> 16-bit sample

an = types.ModuleType("analogio")


class AnalogIn:
    def __init__(self, pin_obj):
        self.pin_obj = pin_obj

    @property
    def value(self):
        return ADC_LEVELS.get(self.pin_obj, 0)


an.AnalogIn = AnalogIn
sys.modules["analogio"] = an

pw = types.ModuleType("pwmio")


class PWMOut:
    def __init__(self, *a, **k):
        self.duty_cycle = 0


pw.PWMOut = PWMOut
sys.modules["pwmio"] = pw

ro = types.ModuleType("rotaryio")


class IncrementalEncoder:
    def __init__(self, *a, **k):
        raise RuntimeError("force software encoders in test")


ro.IncrementalEncoder = IncrementalEncoder
sys.modules["rotaryio"] = ro

uh = types.ModuleType("usb_hid")


class Gamepad:
    def __init__(self):
        self.sent = []

    def send_report(self, r):
        self.sent.append(bytes(r))


GAMEPAD = Gamepad()
uh.devices = [GAMEPAD]
sys.modules["usb_hid"] = uh

tca = types.ModuleType("community_tca9555")
tca.TCA9555 = lambda *a, **k: None
sys.modules["community_tca9555"] = tca

# ---- sandbox ---------------------------------------------------------------
shutil.rmtree(SANDBOX, ignore_errors=True)
os.makedirs(os.path.join(SANDBOX, "lib"))
shutil.copy(os.path.join(WORKTREE, "lib", "serial_handler.py"), os.path.join(SANDBOX, "lib"))
shutil.copy(os.path.join(WORKTREE, "lib", "update_recovery.py"), os.path.join(SANDBOX, "lib"))
os.chdir(SANDBOX)
sys.path.insert(0, os.path.join(SANDBOX, "lib"))
sys.path.insert(0, SANDBOX)

CONFIG = {
    "device": {"name": "RuntimeTest", "pid": 0xF003, "debounce_ms": 0,
               "inactivity_refresh": False},
    "bools": [{"id": "TOGGLE1", "default": False, "store": True},
              {"id": "ARMED", "default": False, "store": True}],
    "axes": [{"id": "AX1", "output": 1, "default": 30000, "store": False},
             {"id": "THR", "output": 2},
             {"id": "STEER", "output": 3},
             {"id": "CURVED", "output": 4},
             {"id": "TABLED", "output": 5}],
    "rules": [
        {"type": "NOR", "inputs": ["D3", "D4"], "output": "B30"},
        {"type": "TOGGLE", "input": "B30", "output": "TOGGLE1"},
        {"type": "MAP", "input": "TOGGLE1", "output": "B100"},
        {"type": "ENCODER", "inputs": ["D5", "D7"], "cw": "B19", "ccw": "B20"},
        {"type": "AXIS_INC", "input": "B19", "axis": "AX1", "step": 1000},
        {"type": "AXIS_DEC", "input": "B20", "axis": "AX1", "step": 1000},
        {"type": "PULSE", "input": "D2", "output": "B50", "pulse_ms": 20},
        # Analog: A1 is a pedal with a narrow swing, no filter so tests are exact.
        {"type": "ANALOG", "input": "A1", "axis": "THR", "min": 10000, "max": 50000,
         "filter": 0, "hysteresis": 100},
        # A2 is a centered stick with a deadzone.
        {"type": "ANALOG", "input": "A2", "axis": "STEER", "min": 0, "max": 60000,
         "center": 30000, "deadzone": 1000, "filter": 0, "invert": True},
        # A3 feeds two axes: exponent curve and a point table (filtered).
        {"type": "ANALOG", "input": "A3", "axis": "CURVED", "curve": 2, "filter": 0},
        {"type": "ANALOG", "input": "A3", "axis": "TABLED", "filter": 3,
         "curve": [[0, 0], [32768, 8192], [65535, 65535]]},
        # THRESHOLD on a raw pin and on a processed axis, chained into a TOGGLE.
        {"type": "THRESHOLD", "input": "A1", "output": "B60", "above": 40000, "hysteresis": 2000},
        {"type": "THRESHOLD", "input": "THR", "output": "B61", "below": 1000},
        {"type": "THRESHOLD", "input": "A4", "output": "B62", "above": 10000},
        {"type": "TOGGLE", "input": "B62", "output": "ARMED"},
    ],
}
with open("config.json", "w") as f:
    json.dump(CONFIG, f)

PASS = 0
FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print("PASS", name)
    else:
        FAIL += 1
        print("FAIL", name, detail)


# Import code.py as a module (it builds ButtonBox at the bottom via the
# guarded loop — chop the entry point off by importing the source up to it).
src = open(os.path.join(WORKTREE, "code.py")).read()
entry = src.index("# ---------------------------------------------------------------------------\n# Entry point")
module = types.ModuleType("fwcode")
module.__dict__["__name__"] = "fwcode"
exec(compile(src[:entry], "code.py", "exec"), module.__dict__)

# D3/D4 open at boot: NOR output true at rest — the classic phantom-toggle trigger.
# A4 above its THRESHOLD at boot: the same trap, via an analog source.
ADC_LEVELS["GP29"] = 20000
box = module.ButtonBox()

check("degraded mode active (no expander)", box.fault == "no_expander", box.fault)
check("boot: NOR steady state seeded", box.b_states.get(30) is True, box.b_states.get(30))
check("boot: TOGGLE did not phantom-fire", box.bool_states["TOGGLE1"] is False,
      box.bool_states["TOGGLE1"])

# A few cycles: still no phantom flip.
for _ in range(5):
    box.update()
check("stable: TOGGLE still off after cycles", box.bool_states["TOGGLE1"] is False)

# Same-cycle passthrough: press D1 (GP-mapped rev2: D1 -> gpio pin 4).
gp = box.pin_map["D1"]["pin"]
PIN_LEVELS[f"GP{gp}"] = True
box.update(); box.update()
check("passthrough same cycle as commit", box.b_states.get(1) is True, box.b_states.get(1))
PIN_LEVELS[f"GP{gp}"] = False
box.update(); box.update()

# Real toggle edge: close D3 (NOR goes false), open again (NOR rises) -> toggle flips.
gp3 = box.pin_map["D3"]["pin"]
PIN_LEVELS[f"GP{gp3}"] = True
box.update(); box.update()
check("NOR responds", box.b_states.get(30) is False)
PIN_LEVELS[f"GP{gp3}"] = False
box.update(); box.update()
check("TOGGLE flips on real edge", box.bool_states["TOGGLE1"] is True)
check("MAP chains from toggle", box.b_states.get(100) is True)

# PULSE: press D2, output high, then clears after pulse_ms.
gp2 = box.pin_map["D2"]["pin"]
PIN_LEVELS[f"GP{gp2}"] = True
box.update(); box.update()
check("PULSE fires", box.b_states.get(50) is True)
_time.sleep(0.03)
box.update()
check("PULSE clears after pulse_ms", box.b_states.get(50) is False)
PIN_LEVELS[f"GP{gp2}"] = False
box.update(); box.update()

# Software encoder on GPIO pins (D5/D7 non-sequential -> software path).
enc_idx = next(i for i, r in enumerate(box.rules) if r.get("type") == "ENCODER")
check("software encoder path chosen", enc_idx in box.sw_encoder_states)
a_pin = f"GP{box.pin_map['D5']['pin']}"
b_pin = f"GP{box.pin_map['D7']['pin']}"

ax_before = box.axis_states["AX1"]
# One full detent CW with divisor default 2: gray sequence 00->10->11.
PIN_LEVELS[a_pin] = True
box._tick_sw_encoders()
PIN_LEVELS[b_pin] = True
box._tick_sw_encoders()
box.update()
ax_after = box.axis_states["AX1"]
check("encoder CW steps axis", ax_after == ax_before + 1000, (ax_before, ax_after))

# Phantom-tick regression: half detent forward then back within drain windows.
PIN_LEVELS[b_pin] = False
box._tick_sw_encoders()
PIN_LEVELS[a_pin] = False
box._tick_sw_encoders()
box.update()  # completes CCW detent: -1000
ax = box.axis_states["AX1"]
check("encoder CCW steps back", ax == ax_before, (ax_before, ax))
# jiggle: one transition forward, then reverse — accumulator returns to 0
PIN_LEVELS[a_pin] = True
box._tick_sw_encoders()
PIN_LEVELS[a_pin] = False
box._tick_sw_encoders()
box.update()
check("jiggle produces no phantom step", box.axis_states["AX1"] == ax,
      (ax, box.axis_states["AX1"]))

# Report content: axis slot 1 carries AX1 value.
box._build_report(box._report_scratch)
lo, hi = box._report_scratch[16], box._report_scratch[17]
check("report axis bytes", lo | (hi << 8) == box.axis_states["AX1"])

# NVM: flip toggle, flush, verify persistence + id-hash invalidation.
box.nvm.flush(force=True)
saved = bytes(NVM[:8])
check("NVM wrote header", NVM[0] == module.NVM_MAGIC and NVM[1] == 2)
nvm2 = module.NVMStorage([("TOGGLE1", False), ("ARMED", False)], [])
check("NVM restores stored bool", nvm2.read_bool("TOGGLE1") is True)
nvm3 = module.NVMStorage([("RENAMED", False)], [])
check("NVM id change resets values", nvm3.read_bool("RENAMED") is False)

# ---- analog inputs ----------------------------------------------------------
A1, A2, A3, A4 = (f"GP{box.pin_map[n]['pin']}" for n in ("A1", "A2", "A3", "A4"))


def cycles(n=2):
    for _ in range(n):
        box.update()


check("analog pins claimed as AnalogIn", sorted(box.analog_ins) == ["A1", "A2", "A3", "A4"],
      sorted(box.analog_ins))
check("analog pins absent from digital scan", "A1" not in box.pin_cache and "A4" not in box.pin_cache)
check("boot: THRESHOLD seeded high", box.b_states.get(62) is True, box.b_states.get(62))
check("boot: TOGGLE off analog threshold did not phantom-fire", box.bool_states["ARMED"] is False)
cycles(3)
check("stable: ARMED still off", box.bool_states["ARMED"] is False)

# Pedal range mapping (min 10000, max 50000, no filter).
ADC_LEVELS[A1] = 5000;  cycles(); check("pedal below min clamps to 0", box.axis_states["THR"] == 0, box.axis_states["THR"])
ADC_LEVELS[A1] = 30000; cycles(); check("pedal mid maps linearly", box.axis_states["THR"] == 32767, box.axis_states["THR"])
ADC_LEVELS[A1] = 60000; cycles(); check("pedal above max clamps to 65535", box.axis_states["THR"] == 65535)
ADC_LEVELS[A1] = 30000; cycles()
ADC_LEVELS[A1] = 30030; cycles(); check("hysteresis holds small wobble", box.axis_states["THR"] == 32767, box.axis_states["THR"])
ADC_LEVELS[A1] = 30100; cycles(); check("hysteresis passes real movement", box.axis_states["THR"] == 32931, box.axis_states["THR"])
box._build_report(box._report_scratch)
check("analog axis lands in HID slot 2",
      box._report_scratch[18] | (box._report_scratch[19] << 8) == 32931)

# THRESHOLD on the raw pin with hysteresis (above 40000, hyst 2000).
ADC_LEVELS[A1] = 39000; cycles(); check("threshold below: off", box.b_states.get(60) is False)
ADC_LEVELS[A1] = 40000; cycles(); check("threshold reached: on", box.b_states.get(60) is True)
ADC_LEVELS[A1] = 38500; cycles(); check("inside hysteresis band: stays on", box.b_states.get(60) is True)
ADC_LEVELS[A1] = 37999; cycles(); check("below band: off", box.b_states.get(60) is False)
# THRESHOLD on a processed axis (below 1000 on THR).
ADC_LEVELS[A1] = 5000;  cycles(); check("axis threshold: on at rest", box.b_states.get(61) is True)
ADC_LEVELS[A1] = 30000; cycles(); check("axis threshold: off when pressed", box.b_states.get(61) is False)

# Centered stick with deadzone, inverted (min 0, center 30000, dz 1000, max 60000).
ADC_LEVELS[A2] = 30000; cycles(); check("center maps to 32767", box.axis_states["STEER"] == 32767, box.axis_states["STEER"])
ADC_LEVELS[A2] = 30800; cycles(); check("inside deadzone stays centered", box.axis_states["STEER"] == 32767, box.axis_states["STEER"])
ADC_LEVELS[A2] = 60000; cycles(); check("max side inverted to 0", box.axis_states["STEER"] == 0, box.axis_states["STEER"])
ADC_LEVELS[A2] = 0;     cycles(); check("min side inverted to 65535", box.axis_states["STEER"] == 65535, box.axis_states["STEER"])
ADC_LEVELS[A2] = 45500; cycles(); check("half deflection past deadzone", box.axis_states["STEER"] == 16384, box.axis_states["STEER"])

# Curves: exponent 2 (unfiltered) and a point table behind a filter.
ADC_LEVELS[A3] = 32768; cycles()
check("exponent curve squares the midpoint", abs(box.axis_states["CURVED"] - 16384) <= 1, box.axis_states["CURVED"])
check("filter lags on the first cycles", 0 < box.axis_states["TABLED"] < 8192, box.axis_states["TABLED"])
cycles(120)
check("filter converges exactly, table applied", box.axis_states["TABLED"] == 8192, box.axis_states["TABLED"])
ADC_LEVELS[A3] = 65535; cycles(120)
check("table endpoint", box.axis_states["TABLED"] == 65535 and box.axis_states["CURVED"] == 65535,
      (box.axis_states["TABLED"], box.axis_states["CURVED"]))

# Serial handler v2 fields over the fake wire.
class FD:
    def __init__(self):
        self.out = b""
        self.write_timeout = None

    def write(self, b):
        self.out += bytes(b)
        return len(b)

    def flush(self):
        pass

    @property
    def in_waiting(self):
        return 0


box.serial._data = FD()
box.serial._buf = bytearray(4096)
box.serial._handle_line(b'{"cmd":"get_info","id":7}')
resp = json.loads(box.serial._data.out.split(b"\n")[0])
check("get_info protocol 2", resp.get("protocol") == 2, resp)
check("get_info pins present", "D1" in resp.get("pins", []), resp.get("pins", [])[:3])
check("get_info fault reported", resp.get("fault") == "no_expander")
check("request id echoed", resp.get("id") == 7, resp)
check("limits advertised", resp.get("limits", {}).get("chunk") == 2048)
check("get_info analog pins", resp.get("analog_pins") == ["A1", "A2", "A3", "A4"], resp.get("analog_pins"))
check("get_info analog active", resp.get("analog_active") == ["A1", "A2", "A3", "A4"], resp.get("analog_active"))
check("get_info analog cap", "analog" in resp.get("caps", []))


def serial(cmd):
    box.serial._data.out = b""
    box.serial._handle_line(json.dumps(cmd).encode())
    return json.loads(box.serial._data.out.split(b"\n")[0])


resp = serial({"cmd": "get_state"})
check("get_state carries raw analog", resp.get("analog", {}).get("A3") == 65535, resp.get("analog"))

serial({"cmd": "stream_start", "interval_ms": 20})
box.serial._stream_last = 0
box.serial._data.out = b""
box.serial.maybe_send_stream()
frame = json.loads(box.serial._data.out.split(b"\n")[0])
check("first stream frame carries analog", frame.get("s", {}).get("an", {}).get("A1") == 30000, frame)
box.serial._stream_last = 0
box.serial._data.out = b""
box.serial.maybe_send_stream()
check("unchanged analog: no frame", box.serial._data.out == b"", box.serial._data.out)
ADC_LEVELS[A4] = 20000 + 10  # under the noise floor: no frame
box._read_analog()
box.serial._stream_last = 0
box.serial.maybe_send_stream()
check("sub-noise analog change: no frame", box.serial._data.out == b"")
ADC_LEVELS[A4] = 20000 + 500
box._read_analog()
box.serial._stream_last = 0
box.serial.maybe_send_stream()
frame = json.loads(box.serial._data.out.split(b"\n")[0])
check("analog change streams", frame.get("s", {}).get("an", {}).get("A4") == 20500, frame)
serial({"cmd": "stream_stop"})


# Validation of analog configs against this board (rev2 map, A1-A4 analog).
def validate(rules, axes=None):
    cfg = {"axes": axes if axes is not None else [{"id": "AX", "output": 1}], "rules": rules}
    return serial({"cmd": "validate_config", "config": cfg})


r = validate([{"type": "ANALOG", "input": "A1", "axis": "AX"}])
check("minimal ANALOG rule valid", r.get("valid") is True, r)
r = validate([{"type": "ANALOG", "input": "A1", "axis": "AX"},
              {"type": "MAP", "input": "A1", "output": "B5"}])
check("analog pin as digital input rejected", "invalid input 'A1'" in r.get("error", ""), r)
r = validate([{"type": "ANALOG", "input": "A1", "axis": "AX"},
              {"type": "ENCODER", "inputs": ["A1", "A2"]}])
check("analog pin in encoder rejected", "A1" in r.get("error", "") and r.get("ok") is False, r)
r = validate([{"type": "ANALOG", "input": "D1", "axis": "AX"}])
check("non-ADC pin rejected", "not analog capable" in r.get("error", ""), r)
r = validate([{"type": "ANALOG", "input": "A1", "axis": "AX"}],
             axes=[{"id": "AX", "output": 1, "store": True}])
check("store on analog axis rejected", "cannot use store" in r.get("error", ""), r)
r = validate([{"type": "AXIS_INC", "input": "D1", "axis": "AX"},
              {"type": "ANALOG", "input": "A1", "axis": "AX"}])
check("AXIS_INC on analog axis rejected (any order)", "driven by an ANALOG" in r.get("error", ""), r)
r = validate([{"type": "ANALOG", "input": "A1", "axis": "AX"},
              {"type": "ANALOG", "input": "A2", "axis": "AX"}])
check("two ANALOG rules on one axis rejected", "already driven" in r.get("error", ""), r)
r = validate([{"type": "ANALOG", "input": "A1", "axis": "AX", "min": 100, "max": 100}])
check("max <= min rejected", "greater than min" in r.get("error", ""), r)
r = validate([{"type": "ANALOG", "input": "A1", "axis": "AX", "center": 1000, "deadzone": 2000}])
check("center/deadzone outside range rejected", "strictly between" in r.get("error", ""), r)
r = validate([{"type": "ANALOG", "input": "A1", "axis": "AX", "deadzone": 10}])
check("deadzone without center rejected", "requires center" in r.get("error", ""), r)
r = validate([{"type": "ANALOG", "input": "A1", "axis": "AX", "curve": [[0, 0], [0, 5]]}])
check("non-increasing curve table rejected", "strictly increasing" in r.get("error", ""), r)
r = validate([{"type": "ANALOG", "input": "A1", "axis": "AX", "curve": 0}])
check("zero exponent rejected", "exponent" in r.get("error", ""), r)
r = validate([{"type": "ANALOG", "input": "A1", "axis": "AX", "filter": 9}])
check("filter out of range rejected", "filter" in r.get("error", ""), r)
r = validate([{"type": "THRESHOLD", "input": "A1", "output": "B3", "above": 100, "below": 5}])
check("threshold with both bounds rejected", "exactly one" in r.get("error", ""), r)
r = validate([{"type": "THRESHOLD", "input": "D1", "output": "B3", "above": 100}])
check("threshold on digital pin rejected", r.get("ok") is False, r)
r = validate([{"type": "THRESHOLD", "input": "AX", "output": "B3", "below": 100, "hysteresis": 50},
              {"type": "THRESHOLD", "input": "A2", "output": "REFRESH", "above": 100}])
check("threshold on axis and pin valid", r.get("valid") is True, r)
r = validate([{"type": "ANALOG", "input": "A1", "axis": "AX", "min": 1000, "max": 60000,
               "center": 30000, "deadzone": 500, "filter": 4, "hysteresis": 32,
               "curve": [[0, 0], [32767, 32767], [65535, 65535]], "invert": True}])
check("full ANALOG rule valid", r.get("valid") is True, r)

print("\n%d passed, %d failed" % (PASS, FAIL))
sys.exit(1 if FAIL else 0)
