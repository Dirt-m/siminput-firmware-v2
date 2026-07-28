"""OTA transfer, verification, and recovery tests for lib/serial_handler.py.

Headless: CircuitPython modules are stubbed, the real firmware code runs on
desktop Python. Run from the repo root: python3 tests/test_ota.py
"""
import os
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SANDBOX = tempfile.mkdtemp(prefix="siminput-fw-test-")

import base64
import hashlib
import json
import os
import shutil
import sys
import types

WORKTREE = str(REPO)


# --- stubs before import ---------------------------------------------------
mc = types.ModuleType("microcontroller")
mc.nvm = bytearray(4096)
mc.reset = lambda: None
mc.on_next_reset = lambda *a: None
mc.RunMode = types.SimpleNamespace(BOOTLOADER=None)
sys.modules["microcontroller"] = mc
sv = types.ModuleType("supervisor")
sv.reload = lambda: (_ for _ in ()).throw(RuntimeError("RELOAD"))
sv.ticks_ms = lambda: int(__import__("time").monotonic() * 1000) % (1 << 29)
sys.modules["supervisor"] = sv

sys.path.insert(0, os.path.join(WORKTREE, "lib"))

import serial_handler
import update_recovery


class FakeData:
    def __init__(self):
        self.out = b""
        self.write_timeout = None
        self.short_write = False

    def write(self, b):
        if self.short_write:
            return max(0, len(b) - 3)
        self.out += bytes(b)
        return len(b)

    def flush(self):
        pass

    @property
    def in_waiting(self):
        return 0

    def replies(self):
        lines = [ln for ln in self.out.split(b"\n") if ln]
        self.out = b""
        return [json.loads(ln) for ln in lines]


class FakeBox:
    def __init__(self):
        self.config = {"device": {}, "bools": [], "axes": [], "rules": []}
        self.pin_names = frozenset({"D1", "D2", "D3", "D4"})
        self.board_map = {"name": "rev2"}
        self.rules = []
        self.b_states = {}
        self.axis_states = {}
        self.axis_output_slot = {}
        self.bool_states = {}
        self.pin_cache = {}
        self.nvm = None


def mk_handler():
    box = FakeBox()
    h = serial_handler.SerialHandler(box)
    h._data = FakeData()
    h._buf = bytearray(serial_handler._MAX_LINE)
    # re-run hash detect against desktop hashlib
    h._hash_algo = "sha256"
    return h


def send(h, obj):
    h._handle_line(json.dumps(obj).encode())
    return h._data.replies()


def feed_raw(h, raw):
    """Push raw bytes through _process_inner via a scripted in_waiting/read."""
    state = {"buf": raw}

    class D(FakeData):
        @property
        def in_waiting(self):
            return len(state["buf"])

        def read(self, n):
            out, state["buf"] = state["buf"][:n], state["buf"][n:]
            return out

    old_out = h._data.out
    d = D()
    d.out = old_out
    h._data = d
    while state["buf"]:
        h._process_inner()
    return h._data.replies()


def file_write(h, path, data, corrupt_hash=False, truncate=False, overrun=False):
    sha256 = hashlib.sha256(data).hexdigest()
    sha1 = hashlib.sha1(data).hexdigest()
    if corrupt_hash:
        sha256 = "0" * 64
        sha1 = "0" * 40
    r = send(h, {"cmd": "file_write", "path": path, "size": len(data),
                 "sha256": sha256, "sha1": sha1})
    if not r or not r[0].get("ready"):
        return r
    payload = data + (b"XXXX" if overrun else b"")
    if truncate:
        payload = payload[: max(0, len(payload) - 5)]
    seq = 0
    sent = 0
    replies = []
    while sent < len(payload):
        chunk = payload[sent:sent + 2048]
        replies += send(h, {"chunk": base64.b64encode(chunk).decode(), "seq": seq})
        if replies and not replies[-1].get("ok"):
            return replies
        sent += len(chunk)
        seq += 1
    replies += send(h, {"done": True})
    return replies


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


def setup_sandbox():
    shutil.rmtree(SANDBOX, ignore_errors=True)
    os.makedirs(os.path.join(SANDBOX, "lib"))
    os.chdir(SANDBOX)
    with open("code.py", "w") as f:
        f.write("# old code\n")
    with open("boot.py", "w") as f:
        f.write("# old boot\n")
    with open("lib/serial_handler.py", "w") as f:
        f.write("# old handler\n")
    with open("config.json", "w") as f:
        json.dump({"device": {"name": "Box", "pid": 0xF000}}, f)


setup_sandbox()
h = mk_handler()

# --- basic command surface --------------------------------------------------
r = send(h, {"cmd": "get_info"})
check("get_info advertises hash algo", r and r[0].get("hash") == "sha256", r)
r = send(h, {"cmd": 5})
check("non-string cmd gets error reply", r and r[0].get("ok") is False, r)
r = send(h, {"cmd": "validate_config", "config": {
    "rules": [{"type": "ENCODER", "inputs": ["D1", "D2"]},
              {"type": "ENCODER", "inputs": ["D2", "D3"]}]}})
check("duplicate encoder pin rejected", r and "already used" in r[0].get("error", ""), r)
r = send(h, {"cmd": "validate_config", "config": {"bools": [{}] * 256}})
check("bool count cap", r and "too many bools" in r[0].get("error", ""), r)
r = send(h, {"cmd": "validate_config", "config": {"device": {"pid": 0x80F4}}})
check("pid 0x80F4 rejected", r and r[0].get("ok") is False, r)

# --- file_write happy path --------------------------------------------------
data = os.urandom(5000)
r = file_write(h, "code.py", data)
check("file_write ok", r and r[-1].get("written"), r)
check("file content installed", open("code.py", "rb").read() == data)
check("no .old left", not os.path.exists("code.py.old"))
check("no ._tmp left", not os.path.exists("._tmp"))

# --- verification failures --------------------------------------------------
r = file_write(h, "code.py", os.urandom(3000), corrupt_hash=True)
check("hash mismatch rejected", r and "checksum mismatch" in r[-1].get("error", ""), r)
check("bad write leaves file intact", open("code.py", "rb").read() == data)

r = file_write(h, "code.py", os.urandom(3000), truncate=True)
check("truncated write rejected (size)", r and "size mismatch" in r[-1].get("error", ""), r)

r = file_write(h, "code.py", os.urandom(3000), overrun=True)
check("overrun rejected", r and "exceeds declared size" in r[-1].get("error", ""), r)
check("handler back in command mode", send(h, {"cmd": "ping"})[0].get("ok"))

# hash unavailable → size check still enforced, sha-only host still works
h._hash_algo = ""
r = file_write(h, "code.py", data)
check("no-hash build still writes (size-checked)", r and r[-1].get("written"), r)
h._hash_algo = "sha256"

# --- zero byte file ---------------------------------------------------------
r = file_write(h, "lib/empty.py", b"")
check("zero-byte write ok", r and r[-1].get("written") and r[-1].get("size") == 0, r)

# --- config.json via file_write is validated --------------------------------
bad = json.dumps({"device": {"pid": 0x80F4}}).encode()
r = file_write(h, "config.json", bad)
check("file_write config.json validated", r and "invalid" in r[-1].get("error", ""), r)
good = json.dumps({"device": {"name": "New", "pid": 0xF001}}).encode()
r = file_write(h, "config.json", good)
check("valid config.json accepted", r and r[-1].get("written"), r)

# --- staged update: begin/commit with journal -------------------------------
r = send(h, {"cmd": "update_begin"})
check("update_begin ok", r and r[0].get("update_mode"), r)
new_code = b"# new code v2\n" * 100
new_lib = b"# new lib\n" * 50
file_write(h, "code.py", new_code)
file_write(h, "lib/serial_handler.py", new_lib)
check("staged, not installed", open("code.py", "rb").read() == data)
r = send(h, {"cmd": "update_commit"})
check("commit ok", r and r[0].get("ok") and "code.py" in r[0].get("committed", []), r)
check("commit installed code.py", open("code.py", "rb").read() == new_code)
check("commit installed lib", open("lib/serial_handler.py", "rb").read() == new_lib)
check("staging cleaned", not os.path.exists(".update"))

# --- update_begin discards stale staging ------------------------------------
send(h, {"cmd": "update_begin"})
file_write(h, "code.py", b"stale-stuff")
r = send(h, {"cmd": "update_begin"})  # no commit — simulate dead session
check("re-begin succeeds", r and r[0].get("update_mode"), r)
check("stale staging discarded", not os.path.exists(".update/code.py"))
send(h, {"cmd": "update_abort"})
check("abort clears staging", not os.path.exists(".update"))

# --- interrupted commit: journal + recover() roll forward -------------------
send(h, {"cmd": "update_begin"})
newer = b"# newer code\n" * 80
file_write(h, "code.py", newer)
file_write(h, "boot.py", b"# newer boot\n")
# Interrupt: simulate power loss after journal write but before any rename
staged = h._list_staged_files(".update")
with open(".update/COMMIT", "w") as f:
    for _, rel in staged:
        f.write(rel + "\n")
# also simulate a half-replaced file: code.py renamed aside, staged still present
os.rename("code.py", "code.py.old")
update_recovery.recover()
check("recover installs staged code.py", open("code.py", "rb").read() == newer)
check("recover installs staged boot.py", open("boot.py", "rb").read() == b"# newer boot\n")
check("recover clears staging", not os.path.exists(".update"))
check("recover clears .old", not os.path.exists("code.py.old"))
h._update_mode = False

# --- orphan .old restore (real file vanished mid-replace) -------------------
os.rename("boot.py", "boot.py.old")
update_recovery.recover()
check("orphan .old restored", os.path.exists("boot.py") and not os.path.exists("boot.py.old"))

# --- chunked set_config (memoryview parse + reboot signal) ------------------
cfg = {"device": {"name": "Chunky", "pid": 0xF002},
       "rules": [{"type": "MAP", "input": "D1", "output": "B%d" % (i % 120 + 1)} for i in range(120)]}
raw = json.dumps(cfg).encode()
assert len(raw) > 3072
r = send(h, {"cmd": "set_config", "chunked": True, "size": len(raw)})
check("chunked set_config ready", r and r[0].get("ready"), r)
seq = 0
sent = 0
reboot_hit = False
while sent < len(raw):
    chunk = raw[sent:sent + 2048]
    try:
        r = send(h, {"chunk": base64.b64encode(chunk).decode(), "seq": seq})
    except RuntimeError:
        reboot_hit = True
        break
    sent += len(chunk)
    seq += 1
if not reboot_hit:
    try:
        r = send(h, {"done": True})
    except RuntimeError as e:
        reboot_hit = True
check("chunked set_config wrote and rebooted", reboot_hit)
check("config content on disk", json.load(open("config.json"))["device"]["name"] == "Chunky")
check("no tmp litter", not os.path.exists("config.json.tmp"))

# --- oversized line recovery keeps the next command --------------------------
h2 = mk_handler()
big = b"X" * (serial_handler._MAX_LINE + 500)
raw_stream = big + b"\n" + b'{"cmd":"ping"}\n'
replies = feed_raw(h2, raw_stream)
ping_ok = any(rep.get("product") == "SIMINPUT" for rep in replies)
toolong = any("line too long" in rep.get("error", "") for rep in replies)
check("oversized line errors once", toolong, replies)
check("command after oversized line survives", ping_ok, replies)

# --- MemoryError path leaves command mode -----------------------------------
h3 = mk_handler()
r = send(h3, {"cmd": "set_config", "chunked": True, "size": 40000})
check("oversize chunked config refused", r and "max" in r[0].get("error", ""), r)
check("h3 still in command mode", send(h3, {"cmd": "ping"})[0].get("ok"))

# --- stream: short write disables streaming ---------------------------------
h4 = mk_handler()
send(h4, {"cmd": "stream_start", "interval_ms": 20})
h4._stream_last = 0.0
h4._data.short_write = True
h4._box.b_states = {1: True}
h4.maybe_send_stream()
check("short write stops streaming", h4._streaming is False)

# --- stream suppressed during chunk op --------------------------------------
h5 = mk_handler()
send(h5, {"cmd": "stream_start", "interval_ms": 20})
send(h5, {"cmd": "file_write", "path": "code.py", "size": 10,
          "sha256": "0" * 64})
h5._data.replies()
h5._stream_last = 0.0
h5._box.b_states = {2: True}
h5.maybe_send_stream()
check("no stream frames mid-transfer", h5._data.out == b"", h5._data.out)

print("\n%d passed, %d failed" % (PASS, FAIL))
sys.exit(1 if FAIL else 0)
