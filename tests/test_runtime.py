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
    "bools": [{"id": "TOGGLE1", "default": False, "store": True}],
    "axes": [{"id": "AX1", "output": 1, "default": 30000, "store": False}],
    "rules": [
        {"type": "NOR", "inputs": ["D3", "D4"], "output": "B30"},
        {"type": "TOGGLE", "input": "B30", "output": "TOGGLE1"},
        {"type": "MAP", "input": "TOGGLE1", "output": "B100"},
        {"type": "ENCODER", "inputs": ["D5", "D7"], "cw": "B19", "ccw": "B20"},
        {"type": "AXIS_INC", "input": "B19", "axis": "AX1", "step": 1000},
        {"type": "AXIS_DEC", "input": "B20", "axis": "AX1", "step": 1000},
        {"type": "PULSE", "input": "D2", "output": "B50", "pulse_ms": 20},
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
check("NVM wrote header", NVM[0] == module.NVM_MAGIC and NVM[1] == 1)
nvm2 = module.NVMStorage([("TOGGLE1", False)], [])
check("NVM restores stored bool", nvm2.read_bool("TOGGLE1") is True)
nvm3 = module.NVMStorage([("RENAMED", False)], [])
check("NVM id change resets values", nvm3.read_bool("RENAMED") is False)

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

print("\n%d passed, %d failed" % (PASS, FAIL))
sys.exit(1 if FAIL else 0)
