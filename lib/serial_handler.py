import json
import time
import os
import gc
import binascii
import sys
import microcontroller
import supervisor

FW_VERSION = "2.6.0"
PROTOCOL = 2

_MAX_LINE = 4096
_CHUNK_SIZE = 2048
_CHUNK_TIMEOUT_MS = 30000
_STREAM_MIN_MS = 20
_MAX_CONFIG = 32768

# supervisor.ticks_ms wraps at 2**29; use ticks_diff for every comparison.
# time.monotonic is unusable for firmware timing: CircuitPython floats are
# single precision, so its resolution decays to worse than a full cycle
# after ~9 hours of uptime.
_TICKS_PERIOD = 1 << 29
_TICKS_HALF = 1 << 28


def ticks_diff(a, b):
    return ((a - b + _TICKS_HALF) % _TICKS_PERIOD) - _TICKS_HALF

_ALLOWED_WRITE_PATHS = {"config.json", "code.py", "boot.py"}
_ALLOWED_WRITE_PREFIXES = ("lib/",)

_VALID_RULE_TYPES = {"MAP", "NOR", "TOGGLE", "PULSE", "ENCODER", "AXIS_INC", "AXIS_DEC"}


# ---------------------------------------------------------------------------
# Config validation
# ---------------------------------------------------------------------------

def validate_config(cfg, pin_names):
    if not isinstance(cfg, dict):
        return False, "config must be a JSON object"

    dev = cfg.get("device", {})
    if not isinstance(dev, dict):
        return False, "device must be an object"

    if "name" in dev:
        name = dev["name"]
        if not isinstance(name, str) or not name:
            return False, "device.name must be a non-empty string"
        if len(name) > 32:
            return False, "device.name must be 32 chars or fewer"

    if "pid" in dev:
        pid = dev["pid"]
        if not isinstance(pid, int) or pid < 1 or pid > 0xFFFF:
            return False, "device.pid must be an integer 1-65535"
        if pid == 0x80F4:
            return False, "device.pid must not be 0x80F4 (CircuitPython default)"

    if "debounce_ms" in dev:
        db = dev["debounce_ms"]
        if not isinstance(db, (int, float)) or db < 0 or db > 100:
            return False, "device.debounce_ms must be 0-100"

    if "inactivity_refresh" in dev:
        ir = dev["inactivity_refresh"]
        if ir is not False and ir is not None:
            if not isinstance(ir, (int, float)) or ir <= 0:
                return False, "device.inactivity_refresh must be a positive number or false"

    bools = cfg.get("bools", [])
    if not isinstance(bools, list):
        return False, "bools must be a list"
    if len(bools) > 255:
        return False, "too many bools (max 255)"

    all_ids = set()
    for i, b in enumerate(bools):
        if not isinstance(b, dict):
            return False, "bools[%d] must be an object" % i
        if "id" not in b:
            continue
        bid = b["id"]
        if not isinstance(bid, str) or not bid or " " in bid:
            return False, "bools[%d].id must be a non-empty string without spaces" % i
        if bid in pin_names:
            return False, "bools[%d].id '%s' conflicts with a pin name" % (i, bid)
        if bid.startswith("B") and bid[1:].isdigit():
            return False, "bools[%d].id '%s' conflicts with button naming" % (i, bid)
        if bid in all_ids:
            return False, "bools[%d].id '%s' is duplicated" % (i, bid)
        all_ids.add(bid)

    axes = cfg.get("axes", [])
    if not isinstance(axes, list):
        return False, "axes must be a list"
    if len(axes) > 255:
        return False, "too many axes (max 255)"

    used_slots = set()
    axis_ids = set()
    for i, a in enumerate(axes):
        if not isinstance(a, dict):
            return False, "axes[%d] must be an object" % i
        if "id" not in a:
            continue
        aid = a["id"]
        if not isinstance(aid, str) or not aid or " " in aid:
            return False, "axes[%d].id must be a non-empty string without spaces" % i
        if aid in pin_names:
            return False, "axes[%d].id '%s' conflicts with a pin name" % (i, aid)
        if aid.startswith("B") and aid[1:].isdigit():
            return False, "axes[%d].id '%s' conflicts with button naming" % (i, aid)
        if aid in all_ids:
            return False, "axes[%d].id '%s' is duplicated" % (i, aid)
        all_ids.add(aid)
        axis_ids.add(aid)

        output = a.get("output", 1)
        if isinstance(output, str):
            if output.upper() != "BACKLIGHT":
                return False, "axes[%d].output must be 1-8 or 'BACKLIGHT'" % i
        elif isinstance(output, int):
            if output < 1 or output > 8:
                return False, "axes[%d].output must be 1-8 or 'BACKLIGHT'" % i
            if output in used_slots:
                return False, "axes[%d].output slot %d is already used" % (i, output)
            used_slots.add(output)
        else:
            return False, "axes[%d].output must be 1-8 or 'BACKLIGHT'" % i

        if "default" in a:
            d = a["default"]
            if not isinstance(d, int) or d < 0 or d > 65535:
                return False, "axes[%d].default must be 0-65535" % i

    rules = cfg.get("rules", [])
    if not isinstance(rules, list):
        return False, "rules must be a list"
    if len(rules) > 256:
        return False, "too many rules (max 256)"

    def _valid_input(ref):
        if ref in pin_names:
            return True
        if isinstance(ref, str) and ref.startswith("B") and ref[1:].isdigit():
            n = int(ref[1:])
            return 1 <= n <= 127
        return ref in all_ids

    def _valid_output(ref):
        if isinstance(ref, str) and ref.startswith("B") and ref[1:].isdigit():
            n = int(ref[1:])
            return 1 <= n <= 127
        if ref == "REFRESH":
            return True
        return ref in all_ids

    encoder_pins = set()
    for i, r in enumerate(rules):
        if not isinstance(r, dict):
            return False, "rules[%d] must be an object" % i
        rtype = r.get("type", "")
        if not rtype:
            continue
        if rtype not in _VALID_RULE_TYPES:
            return False, "rules[%d].type '%s' is not valid" % (i, rtype)

        if rtype == "MAP":
            if not _valid_input(r.get("input", "")):
                return False, "rules[%d] MAP: invalid input '%s'" % (i, r.get("input", ""))
            if not _valid_output(r.get("output", "")):
                return False, "rules[%d] MAP: invalid output '%s'" % (i, r.get("output", ""))

        elif rtype == "NOR":
            inputs = r.get("inputs", [])
            if not isinstance(inputs, list) or len(inputs) == 0:
                return False, "rules[%d] NOR: inputs must be a non-empty list" % i
            for inp in inputs:
                if not _valid_input(inp):
                    return False, "rules[%d] NOR: invalid input '%s'" % (i, inp)
            if not _valid_output(r.get("output", "")):
                return False, "rules[%d] NOR: invalid output '%s'" % (i, r.get("output", ""))

        elif rtype == "TOGGLE":
            if not _valid_input(r.get("input", "")):
                return False, "rules[%d] TOGGLE: invalid input '%s'" % (i, r.get("input", ""))
            if not _valid_output(r.get("output", "")):
                return False, "rules[%d] TOGGLE: invalid output '%s'" % (i, r.get("output", ""))

        elif rtype == "PULSE":
            if not _valid_input(r.get("input", "")):
                return False, "rules[%d] PULSE: invalid input '%s'" % (i, r.get("input", ""))
            if not _valid_output(r.get("output", "")):
                return False, "rules[%d] PULSE: invalid output '%s'" % (i, r.get("output", ""))
            if "pulse_ms" in r:
                if not isinstance(r["pulse_ms"], (int, float)) or r["pulse_ms"] < 0:
                    return False, "rules[%d] PULSE: pulse_ms must be >= 0" % i
            if "delay_ms" in r:
                if not isinstance(r["delay_ms"], (int, float)) or r["delay_ms"] < 0:
                    return False, "rules[%d] PULSE: delay_ms must be >= 0" % i

        elif rtype == "ENCODER":
            inputs = r.get("inputs", [])
            if not isinstance(inputs, list) or len(inputs) != 2:
                return False, "rules[%d] ENCODER: inputs must be exactly 2 pin names" % i
            for inp in inputs:
                if inp not in pin_names:
                    return False, "rules[%d] ENCODER: '%s' is not a valid pin name" % (i, inp)
                if inp in encoder_pins:
                    return False, "rules[%d] ENCODER: pin '%s' is already used by another encoder" % (i, inp)
                encoder_pins.add(inp)
            if inputs[0] == inputs[1]:
                return False, "rules[%d] ENCODER: inputs must be two different pins" % i
            for key in ("cw", "ccw"):
                ref = r.get(key, "")
                if ref and not _valid_output(ref):
                    return False, "rules[%d] ENCODER: invalid %s output '%s'" % (i, key, ref)
            if "divisor" in r and r["divisor"] not in (1, 2, 4):
                return False, "rules[%d] ENCODER: divisor must be 1, 2, or 4" % i
            if "pulse_ms" in r:
                if not isinstance(r["pulse_ms"], (int, float)) or r["pulse_ms"] < 0:
                    return False, "rules[%d] ENCODER: pulse_ms must be >= 0" % i

        elif rtype in ("AXIS_INC", "AXIS_DEC"):
            if not _valid_input(r.get("input", "")):
                return False, "rules[%d] %s: invalid input '%s'" % (i, rtype, r.get("input", ""))
            axis = r.get("axis", "")
            if axis not in axis_ids:
                return False, "rules[%d] %s: axis '%s' is not declared" % (i, rtype, axis)
            if "step" in r:
                s = r["step"]
                if not isinstance(s, int) or s < 1 or s > 65535:
                    return False, "rules[%d] %s: step must be 1-65535" % (i, rtype)

    return True, ""


# ---------------------------------------------------------------------------
# Serial handler
# ---------------------------------------------------------------------------

class SerialHandler:

    def __init__(self, box):
        self._box = box
        self._data = None

        self._buf = None
        self._buf_pos = 0

        self._chunk_op = None
        self._chunk_buf = None
        self._chunk_pos = 0
        self._chunk_expect = 0
        self._chunk_seq = 0
        self._chunk_meta = None
        self._chunk_deadline = 0.0
        self._chunk_file = None

        self._update_mode = False

        self._streaming = False
        self._stream_interval_ms = 50
        self._stream_last = 0
        self._stream_prev_btns = None
        self._stream_prev_axes = None
        self._stream_prev_pins = None
        self._reply_id = None

        try:
            import usb_cdc
            self._data = usb_cdc.data
        except (ImportError, AttributeError):
            pass

        if self._data is not None:
            self._buf = bytearray(_MAX_LINE)
            try:
                # Bounded writes: without this, a host that stops reading fills
                # the CDC TX FIFO and _send blocks the 200 Hz loop forever.
                self._data.write_timeout = 0.5
            except Exception:
                pass

        # Hash support differs per CircuitPython build: 9.x on RP2040 ships
        # sha1 only, sha256 arrives with 10.x. Detect once; transfers verify
        # with the strongest algorithm both sides support and always enforce
        # the byte count.
        self._hash_algo = ""
        try:
            import hashlib
            for _algo in ("sha256", "sha1"):
                try:
                    hashlib.new(_algo)
                    self._hash_algo = _algo
                    break
                except Exception:
                    pass
        except ImportError:
            pass
        self._discarding = False

    # ------------------------------------------------------------------
    # Main entry points (called from ButtonBox.update)
    # ------------------------------------------------------------------

    def process(self):
        if self._data is None:
            return
        try:
            self._process_inner()
        except Exception:
            self._buf_pos = 0

    def _process_inner(self):
        if self._chunk_op is not None:
            if ticks_diff(supervisor.ticks_ms(), self._chunk_deadline) > 0:
                self._abort_chunk("timeout: transfer abandoned after 30s")
                return

        avail = self._data.in_waiting
        if avail == 0:
            return

        raw = self._data.read(min(avail, _MAX_LINE))
        if not raw:
            return

        for b in raw:
            if self._discarding:
                # Skipping the tail of an over-long line. Bytes after its
                # newline belong to the next command and must be kept.
                if b == 0x0A:
                    self._discarding = False
                continue
            if b == 0x0A:
                line = bytes(self._buf[:self._buf_pos])
                self._buf_pos = 0
                self._handle_line(line)
            elif self._buf_pos >= _MAX_LINE:
                self._buf_pos = 0
                self._discarding = True
                self._send({"ok": False, "error": "line too long (max %d bytes)" % _MAX_LINE})
            else:
                self._buf[self._buf_pos] = b
                self._buf_pos += 1

    def maybe_send_stream(self):
        if not self._streaming or self._data is None:
            return
        if self._chunk_op is not None:
            return  # never interleave stream frames with a chunked transfer

        now = supervisor.ticks_ms()
        if ticks_diff(now, self._stream_last) < self._stream_interval_ms:
            return
        # Re-arm before the change check, so an idle box pays the comparison
        # at the stream interval instead of every 5 ms cycle.
        self._stream_last = now

        box = self._box
        active_btns = sorted(bnum for bnum, state in box.b_states.items() if state)
        axes = [32767] * 8
        for aid, val in box.axis_states.items():
            slot = box.axis_output_slot.get(aid)
            if slot is not None and 1 <= slot <= 8:
                axes[slot - 1] = val

        pins_changed = {}
        for name, val in box.pin_cache.items():
            if self._stream_prev_pins is None or self._stream_prev_pins.get(name) != val:
                pins_changed[name] = val

        btns_tuple = tuple(active_btns)
        axes_tuple = tuple(axes)
        changed = (btns_tuple != self._stream_prev_btns
                   or axes_tuple != self._stream_prev_axes
                   or pins_changed)

        if not changed:
            return

        msg = {"s": {"b": active_btns, "a": axes}}
        if pins_changed:
            msg["s"]["p"] = pins_changed
        if not self._send(msg):
            self._streaming = False
            return

        self._stream_prev_btns = btns_tuple
        self._stream_prev_axes = axes_tuple
        if self._stream_prev_pins is None:
            self._stream_prev_pins = dict(box.pin_cache)
        else:
            self._stream_prev_pins.update(pins_changed)

    # ------------------------------------------------------------------
    # Transport
    # ------------------------------------------------------------------

    def _send(self, obj):
        """Write one JSON line. Returns False on a failed or partial write
        (host not reading), so stream callers can stop rather than desync."""
        try:
            if self._reply_id is not None and "id" not in obj:
                obj["id"] = self._reply_id
            data = json.dumps(obj).encode("utf-8") + b"\n"
            n = self._data.write(data)
            if n is not None and n < len(data):
                return False
            self._data.flush()
            return True
        except Exception:
            return False

    def _handle_line(self, line):
        if not line:
            return

        try:
            msg = json.loads(line)
        except (ValueError, Exception):
            self._send({"ok": False, "error": "invalid JSON"})
            return

        if not isinstance(msg, dict):
            self._send({"ok": False, "error": "expected JSON object"})
            return

        # Optional request id: echoed on every reply this line produces, so
        # the host can correlate responses instead of trusting read order.
        rid = msg.get("id")
        if isinstance(rid, (int, str)):
            self._reply_id = rid

        try:
            if self._chunk_op is not None:
                self._handle_chunk_msg(msg)
                return

            cmd = msg.get("cmd")
            if cmd is None:
                self._send({"ok": False, "error": "missing 'cmd' field"})
                return
            if not isinstance(cmd, str):
                self._send({"ok": False, "error": "'cmd' must be a string"})
                return

            handler = getattr(self, "_cmd_" + cmd, None)
            if handler is None:
                self._send({"ok": False, "error": "unknown command: " + str(cmd)})
                return

            try:
                handler(msg)
            except Exception as e:
                self._send({"ok": False, "error": str(e)})
        finally:
            self._reply_id = None

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def _cmd_ping(self, msg):
        dev = self._box.config.get("device", {})
        self._send({
            "ok": True,
            "product": "SIMINPUT",
            "version": FW_VERSION,
            "protocol": PROTOCOL,
            "name": dev.get("name", "SimInput Button Box"),
            "pid": dev.get("pid", 0xF000),
            "board_map": self._box.board_map.get("name", "unknown"),
        })

    def _cmd_get_info(self, msg):
        dev = self._box.config.get("device", {})
        cp_version = ""
        try:
            v = sys.implementation.version
            cp_version = "%d.%d.%d" % (v[0], v[1], v[2])
        except Exception:
            pass

        board_id = ""
        try:
            import board
            board_id = board.board_id
        except Exception:
            pass

        nvm_size = 0
        try:
            nvm_size = len(microcontroller.nvm)
        except Exception:
            pass

        self._send({
            "ok": True,
            "name": dev.get("name", "SimInput Button Box"),
            "pid": dev.get("pid", 0xF000),
            "version": FW_VERSION,
            "protocol": PROTOCOL,
            "circuitpython": cp_version,
            "board": board_id,
            "nvm_size": nvm_size,
            "bools": [b["id"] for b in self._box.config.get("bools", []) if "id" in b],
            "axes": [a["id"] for a in self._box.config.get("axes", []) if "id" in a],
            "rules_count": len(self._box.rules),
            "board_map": self._box.board_map.get("name", "unknown"),
            "hash": self._hash_algo,
            "caps": ["staged_update", "hard_reboot", "stream", "chunked_config", "request_id"],
            "limits": {"max_line": _MAX_LINE, "chunk": _CHUNK_SIZE, "max_config": _MAX_CONFIG},
            "pins": sorted(self._box.pin_names),
            "fault": getattr(self._box, "fault", ""),
        })

    def _cmd_get_config(self, msg):
        try:
            with open("config.json") as f:
                cfg = json.load(f)
        except Exception as e:
            self._send({"ok": False, "error": "cannot read config.json: " + str(e)})
            return
        self._send({"ok": True, "config": cfg})

    def _cmd_set_config(self, msg):
        if msg.get("chunked"):
            size = msg.get("size", 0)
            if not isinstance(size, int) or size < 1 or size > _MAX_CONFIG:
                self._send({"ok": False, "error": "invalid size (max %d)" % _MAX_CONFIG})
                return
            self._start_chunk_receive("set_config", size, msg)
            return

        cfg = msg.get("config")
        if cfg is None:
            self._send({"ok": False, "error": "missing 'config' field"})
            return

        ok, err = validate_config(cfg, self._box.pin_names)
        if not ok:
            self._send({"ok": False, "error": err})
            return
        self._write_config_and_reboot(cfg)

    def _cmd_validate_config(self, msg):
        if msg.get("chunked"):
            size = msg.get("size", 0)
            if not isinstance(size, int) or size < 1 or size > _MAX_CONFIG:
                self._send({"ok": False, "error": "invalid size (max %d)" % _MAX_CONFIG})
                return
            self._start_chunk_receive("validate_config", size, msg)
            return

        cfg = msg.get("config")
        if cfg is None:
            self._send({"ok": False, "error": "missing 'config' field"})
            return

        ok, err = validate_config(cfg, self._box.pin_names)
        if ok:
            self._send({"ok": True, "valid": True})
        else:
            self._send({"ok": False, "error": err})

    def _cmd_get_state(self, msg):
        box = self._box
        active_btns = sorted(bnum for bnum, state in box.b_states.items() if state)
        axes = [32767] * 8
        for aid, val in box.axis_states.items():
            slot = box.axis_output_slot.get(aid)
            if slot is not None and 1 <= slot <= 8:
                axes[slot - 1] = val
        self._send({
            "ok": True,
            "buttons": active_btns,
            "axes": axes,
            "bools": dict(box.bool_states),
            "pins": dict(box.pin_cache),
        })

    def _cmd_stream_start(self, msg):
        interval_ms = msg.get("interval_ms", 50)
        if not isinstance(interval_ms, (int, float)):
            interval_ms = 50
        self._streaming = True
        self._stream_interval_ms = int(max(interval_ms, _STREAM_MIN_MS))
        self._stream_last = 0
        self._stream_prev_btns = None
        self._stream_prev_axes = None
        self._stream_prev_pins = None
        self._send({"ok": True})

    def _cmd_stream_stop(self, msg):
        self._streaming = False
        self._send({"ok": True})

    def _cmd_file_write(self, msg):
        path = msg.get("path", "")
        size = msg.get("size", 0)
        sha256 = msg.get("sha256", "")
        sha1 = msg.get("sha1", "")

        if not self._is_writable_path(path):
            self._send({"ok": False, "error": "path not allowed: " + str(path)})
            return
        if not isinstance(size, int) or size < 0:
            self._send({"ok": False, "error": "invalid size"})
            return
        if not isinstance(sha256, str) or len(sha256) != 64:
            self._send({"ok": False, "error": "sha256 must be a 64-char hex string"})
            return
        if sha1 and (not isinstance(sha1, str) or len(sha1) != 40):
            self._send({"ok": False, "error": "sha1 must be a 40-char hex string"})
            return

        dest = ".update/" + path if self._update_mode else path
        self._start_chunk_receive("file_write", size, {
            "path": dest,
            "real_path": path,
            "sha256": sha256,
            "sha1": sha1,
        })

    def _cmd_update_begin(self, msg):
        # A previous session may have died mid-update (host crash, unplug).
        # Discard any stale staging and start clean, instead of wedging every
        # future update behind an "already in progress" error.
        self._rmtree(".update")
        self._update_mode = False
        try:
            os.mkdir(".update")
            os.mkdir(".update/lib")
        except OSError:
            pass
        try:
            os.stat(".update/lib")
        except OSError as e:
            self._send({"ok": False, "error": "cannot create staging dir: " + str(e)})
            return
        self._update_mode = True
        self._send({"ok": True, "update_mode": True})

    def _cmd_update_commit(self, msg):
        if not self._update_mode:
            self._send({"ok": False, "error": "no update in progress"})
            return
        staged = self._list_staged_files(".update")
        # Commit in dependency order (lib, then boot.py, then code.py) so a
        # partial commit never runs new code.py against old libs.
        def _commit_rank(item):
            path = item[1]
            if path.startswith("lib/"):
                return 0
            if path == "boot.py":
                return 2
            if path == "code.py":
                return 3
            return 1
        staged = sorted(staged, key=_commit_rank)

        # Journal the commit before touching any real file. If power is lost
        # (or an OSError aborts us) mid-way, boot-time recovery reads this
        # journal and rolls the remaining renames forward — every staged file
        # was already size- and hash-verified on upload.
        try:
            with open(".update/COMMIT", "w") as f:
                for _, rel in staged:
                    f.write(rel + "\n")
        except OSError as e:
            self._send({"ok": False, "error": "cannot write commit journal: " + str(e)})
            return

        committed = []
        try:
            for staged_path, real_path in staged:
                self._ensure_parent(real_path)
                self._replace_file(staged_path, real_path)
                committed.append(real_path)
        except OSError as e:
            # Staging and journal stay on flash: the next boot completes the
            # commit, or a fresh update_begin discards it for a clean retry.
            self._update_mode = False
            gc.collect()
            self._send({"ok": False, "error": "commit failed: " + str(e),
                        "committed": committed, "recovery": "pending_on_reboot"})
            return
        try:
            os.remove(".update/COMMIT")
        except OSError:
            pass
        self._rmtree(".update")
        self._update_mode = False
        gc.collect()
        self._send({"ok": True, "committed": committed})

    def _cmd_update_abort(self, msg):
        self._rmtree(".update")
        self._update_mode = False
        gc.collect()
        self._send({"ok": True, "aborted": True})

    def _cmd_file_read(self, msg):
        path = msg.get("path", "")
        if not path:
            self._send({"ok": False, "error": "missing path"})
            return

        try:
            size = os.stat(path)[6]
        except OSError:
            self._send({"ok": False, "error": "file not found: " + str(path)})
            return

        try:
            sha = self._compute_file_hash(path)
        except Exception:
            sha = ""
        algo = self._hash_algo if sha else ""

        if size <= _CHUNK_SIZE:
            with open(path, "rb") as f:
                data = f.read()
            encoded = binascii.b2a_base64(data).decode("utf-8").strip()
            self._send({"ok": True, "size": size, "hash": sha, "hash_algo": algo, "data": encoded})
        else:
            self._send({"ok": True, "size": size, "hash": sha, "hash_algo": algo, "chunked": True})
            with open(path, "rb") as f:
                seq = 0
                while True:
                    chunk = f.read(_CHUNK_SIZE)
                    if not chunk:
                        break
                    encoded = binascii.b2a_base64(chunk).decode("utf-8").strip()
                    self._send({"chunk": encoded, "seq": seq})
                    seq += 1
            self._send({"done": True})

    def _cmd_reboot(self, msg):
        if self._box.nvm is not None:
            self._box.nvm.flush(force=True)
        self._send({"ok": True, "rebooting": True})
        time.sleep(0.1)
        if msg.get("hard"):
            # Full chip reset: re-runs boot.py, so USB identity changes and a
            # freshly flashed boot.py actually take effect.
            microcontroller.reset()
        supervisor.reload()

    def _cmd_bootloader(self, msg):
        if self._box.nvm is not None:
            self._box.nvm.flush(force=True)
        self._send({"ok": True, "entering_bootloader": True})
        time.sleep(0.1)
        microcontroller.on_next_reset(microcontroller.RunMode.BOOTLOADER)
        microcontroller.reset()

    # ------------------------------------------------------------------
    # Chunked transfer
    # ------------------------------------------------------------------

    def _start_chunk_receive(self, op, size, meta):
        # Acquire resources first, set _chunk_op last: a MemoryError or OSError
        # here must leave the handler in command mode, not wedged in a chunk
        # state with no buffer.
        if op == "file_write":
            path = meta["path"]
            temp = self._temp_path(path)
            self._ensure_parent(temp)
            try:
                chunk_file = open(temp, "wb")
            except OSError as e:
                self._send({"ok": False, "error": "cannot create temp file: " + str(e)})
                return
            self._chunk_file = chunk_file
            self._chunk_buf = None
        else:
            # Size is pre-validated against _MAX_CONFIG by the command handlers.
            # The buffer must hold the full declared size: anything smaller makes
            # every transfer past the cap fail with "data exceeds declared size".
            gc.collect()
            try:
                buf = bytearray(size + 256)
            except MemoryError:
                gc.collect()
                self._send({"ok": False, "error": "not enough memory for %d byte transfer" % size})
                return
            self._chunk_buf = buf
            self._chunk_file = None

        self._chunk_pos = 0
        self._chunk_expect = size
        self._chunk_seq = 0
        self._chunk_meta = meta
        self._chunk_deadline = supervisor.ticks_ms() + _CHUNK_TIMEOUT_MS
        self._chunk_op = op
        self._send({"ok": True, "ready": True})

    def _handle_chunk_msg(self, msg):
        if "chunk" in msg:
            seq = msg.get("seq", -1)
            if seq != self._chunk_seq:
                self._abort_chunk("expected seq %d, got %d" % (self._chunk_seq, seq))
                return

            try:
                decoded = binascii.a2b_base64(msg["chunk"])
            except Exception as e:
                self._abort_chunk("base64 decode error: " + str(e))
                return

            if self._chunk_pos + len(decoded) > self._chunk_expect:
                self._abort_chunk("data exceeds declared size")
                return
            if self._chunk_file is not None:
                try:
                    self._chunk_file.write(decoded)
                except OSError as e:
                    self._abort_chunk("write error: " + str(e))
                    return
                self._chunk_pos += len(decoded)
            elif self._chunk_buf is not None:
                end = self._chunk_pos + len(decoded)
                self._chunk_buf[self._chunk_pos:end] = decoded
                self._chunk_pos = end
            else:
                self._abort_chunk("no active transfer buffer")
                return

            self._chunk_seq += 1
            self._chunk_deadline = supervisor.ticks_ms() + _CHUNK_TIMEOUT_MS
            self._send({"ok": True, "seq": seq})

        elif msg.get("done"):
            self._finish_chunk()

        else:
            self._abort_chunk("unexpected message during transfer")

    def _finish_chunk(self):
        op = self._chunk_op
        meta = self._chunk_meta

        if op == "file_write":
            if self._chunk_file is not None:
                self._chunk_file.close()
                self._chunk_file = None

            path = meta["path"]
            temp = self._temp_path(path)

            # 1. Byte count must match the declared size exactly — an early
            # {"done": true} or a dropped chunk is a truncated file.
            if self._chunk_pos != self._chunk_expect:
                self._fail_file_write(temp, "size mismatch: expected %d bytes, got %d"
                                      % (self._chunk_expect, self._chunk_pos))
                return

            # 2. Hash check, fail closed: if this build can hash and the host
            # supplied a digest for that algorithm, a compute failure or a
            # mismatch rejects the file. (Hosts that only sent an algorithm we
            # can't compute still get the size check above.)
            if self._hash_algo:
                expected_hash = meta.get(self._hash_algo, "")
                if expected_hash:
                    try:
                        actual_hash = self._compute_file_hash(temp)
                    except Exception as e:
                        self._fail_file_write(temp, "hash failed: " + str(e))
                        return
                    if actual_hash != expected_hash:
                        self._fail_file_write(temp, "checksum mismatch: expected %s, got %s"
                                              % (expected_hash, actual_hash))
                        return

            # 3. A config.json written this way must pass the same validation
            # as set_config — file_write is not a validation bypass.
            if meta.get("real_path") == "config.json":
                try:
                    with open(temp) as f:
                        cfg = json.load(f)
                except Exception as e:
                    self._fail_file_write(temp, "config.json is not valid JSON: " + str(e))
                    return
                ok, err = validate_config(cfg, self._box.pin_names)
                if not ok:
                    self._fail_file_write(temp, "config.json invalid: " + err)
                    return

            try:
                self._replace_file(temp, path)
            except OSError as e:
                self._chunk_op = None
                gc.collect()
                self._send({"ok": False, "error": "rename failed: " + str(e)})
                return

            file_size = self._chunk_pos
            self._chunk_op = None
            gc.collect()
            self._send({"ok": True, "written": True, "size": file_size})

        elif op in ("set_config", "validate_config"):
            buf = self._chunk_buf
            self._chunk_op = None

            if self._chunk_pos != self._chunk_expect:
                self._chunk_buf = None
                gc.collect()
                self._send({"ok": False, "error": "size mismatch: expected %d bytes, got %d"
                            % (self._chunk_expect, self._chunk_pos)})
                return

            # Parse straight from the receive buffer (no copies on
            # CircuitPython, whose json.loads accepts any buffer object); the
            # buffer must stay referenced until json.loads returns. The bytes
            # fallback keeps this testable on desktop Python.
            try:
                view = memoryview(buf)[:self._chunk_pos]
                try:
                    cfg = json.loads(view)
                except TypeError:
                    cfg = json.loads(bytes(view))
            except (ValueError, Exception) as e:
                self._chunk_buf = None
                gc.collect()
                self._send({"ok": False, "error": "invalid JSON: " + str(e)})
                return
            self._chunk_buf = None
            gc.collect()

            ok, err = validate_config(cfg, self._box.pin_names)
            if op == "validate_config":
                if ok:
                    self._send({"ok": True, "valid": True})
                else:
                    self._send({"ok": False, "error": err})
            else:
                if not ok:
                    self._send({"ok": False, "error": err})
                    return
                self._write_config_and_reboot(cfg)

        else:
            self._chunk_op = None

    def _fail_file_write(self, temp, reason):
        try:
            os.remove(temp)
        except OSError:
            pass
        self._chunk_op = None
        gc.collect()
        self._send({"ok": False, "error": reason})

    def _replace_file(self, src, dst):
        """Install src as dst without a window where dst is absent.

        Rename-aside: dst -> dst.old, src -> dst, delete dst.old. Interrupted
        at any point, boot-time recovery (lib/update_recovery.py) restores a
        consistent state from the .old copy. Raises OSError on failure.
        """
        old = dst + ".old"
        try:
            os.remove(old)
        except OSError:
            pass
        try:
            os.rename(dst, old)
        except OSError:
            pass  # dst may not exist yet
        os.rename(src, dst)
        try:
            os.remove(old)
        except OSError:
            pass

    def _abort_chunk(self, reason):
        if self._chunk_file is not None:
            self._chunk_file.close()
            self._chunk_file = None
        if self._chunk_op == "file_write" and self._chunk_meta:
            temp = self._temp_path(self._chunk_meta.get("path", ""))
            try:
                os.remove(temp)
            except OSError:
                pass
        self._chunk_op = None
        self._chunk_buf = None
        gc.collect()
        self._send({"ok": False, "error": reason})

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _write_config_and_reboot(self, cfg):
        # Write to a temp file and swap it in, so power loss mid-write can
        # never leave a truncated config.json (which would silently reset the
        # device to defaults, including its USB identity).
        try:
            with open("config.json.tmp", "w") as f:
                json.dump(cfg, f)
            self._replace_file("config.json.tmp", "config.json")
        except OSError as e:
            try:
                os.remove("config.json.tmp")
            except OSError:
                pass
            self._send({"ok": False, "error": "write failed: " + str(e)})
            return
        if self._box.nvm is not None:
            self._box.nvm.flush(force=True)
        self._send({"ok": True, "rebooting": True})
        time.sleep(0.1)
        supervisor.reload()

    def _compute_file_hash(self, path):
        """Hex digest of path using the detected algorithm.

        Raises on any failure — callers decide whether that is fatal (it is,
        for OTA verification). Returns "" only when this build has no hash
        support at all.
        """
        if not self._hash_algo:
            return ""
        import hashlib
        h = hashlib.new(self._hash_algo)
        with open(path, "rb") as f:
            while True:
                block = f.read(512)
                if not block:
                    break
                h.update(block)
        return binascii.hexlify(h.digest()).decode("utf-8")

    def _is_writable_path(self, path):
        if not isinstance(path, str) or ".." in path.split("/"):
            return False
        if path in _ALLOWED_WRITE_PATHS:
            return True
        for prefix in _ALLOWED_WRITE_PREFIXES:
            if path.startswith(prefix):
                return True
        return False

    def _temp_path(self, path):
        slash = path.rfind("/")
        if slash >= 0:
            return path[:slash + 1] + "._tmp"
        return "._tmp"

    def _ensure_parent(self, path):
        slash = path.rfind("/")
        if slash <= 0:
            return
        parent = path[:slash]
        try:
            os.stat(parent)
        except OSError:
            try:
                os.mkdir(parent)
            except OSError:
                pass

    def _rmtree(self, path):
        try:
            entries = os.listdir(path)
        except OSError:
            return
        for entry in entries:
            full = path + "/" + entry
            try:
                os.remove(full)
            except OSError:
                self._rmtree(full)
        try:
            os.rmdir(path)
        except OSError:
            pass

    def _list_staged_files(self, base, prefix=""):
        result = []
        try:
            entries = os.listdir(base)
        except OSError:
            return result
        for entry in entries:
            if not prefix and entry == "COMMIT":
                continue  # the commit journal is bookkeeping, not payload
            full = base + "/" + entry
            rel = entry if not prefix else prefix + "/" + entry
            try:
                is_dir = bool(os.stat(full)[0] & 0x4000)
            except OSError:
                continue
            if is_dir:
                result.extend(self._list_staged_files(full, rel))
            else:
                result.append((full, rel))
        return result
