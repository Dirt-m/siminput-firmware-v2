"""Boot-time recovery for interrupted OTA file replacements.

File replacement uses a rename-aside sequence so the real filename is never
absent from flash:

    1. os.rename(path, path + ".old")   (skipped if path does not exist)
    2. os.rename(staged_or_temp, path)
    3. os.remove(path + ".old")

A staged commit additionally writes ".update/COMMIT" (one relative path per
line) before the first rename. recover() runs from boot.py and code.py; it
rolls an interrupted commit forward, restores any orphaned ".old" files, and
clears leftover staging and temp files. Every step is individually guarded:
recovery must never be the reason a device fails to boot.
"""

import os

_UPDATE_DIR = ".update"
_JOURNAL = _UPDATE_DIR + "/COMMIT"

# Directories that OTA writes can touch: the root, lib/, and one level of
# lib/ subdirectories (mirrors _ensure_parent in serial_handler).
def _scan_dirs():
    dirs = ["", "lib"]
    try:
        for entry in os.listdir("lib"):
            full = "lib/" + entry
            try:
                if os.stat(full)[0] & 0x4000:
                    dirs.append(full)
            except OSError:
                pass
    except OSError:
        pass
    return dirs


def _exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False


def _replace(src, dst):
    """Rename-aside replacement of dst by src. Raises OSError on failure."""
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


def _rmtree(path):
    try:
        entries = os.listdir(path)
    except OSError:
        return
    for entry in entries:
        full = path + "/" + entry
        try:
            os.remove(full)
        except OSError:
            _rmtree(full)
    try:
        os.rmdir(path)
    except OSError:
        pass


def recover():
    # 1. Roll an interrupted staged commit forward. The journal lists every
    #    file the commit intended to install; staged copies that survived are
    #    installed now, and half-replaced files are healed from their .old copy.
    paths = None
    try:
        with open(_JOURNAL) as f:
            paths = [ln.strip() for ln in f if ln.strip()]
    except OSError:
        pass
    if paths:
        for rel in paths:
            try:
                staged = _UPDATE_DIR + "/" + rel
                if _exists(staged):
                    _replace(staged, rel)
                elif not _exists(rel) and _exists(rel + ".old"):
                    os.rename(rel + ".old", rel)
            except OSError:
                pass

    # 2. Sweep orphaned aside/temp files from interrupted direct writes.
    for d in _scan_dirs():
        prefix = d + "/" if d else ""
        try:
            entries = os.listdir(d if d else ".")
        except OSError:
            continue
        for entry in entries:
            if entry.endswith(".old"):
                base = prefix + entry[:-4]
                try:
                    if _exists(base):
                        os.remove(prefix + entry)
                    else:
                        os.rename(prefix + entry, base)
                except OSError:
                    pass
            elif entry == "._tmp":
                try:
                    os.remove(prefix + entry)
                except OSError:
                    pass

    # 3. Any remaining staging is stale (upload never committed) — clear it.
    _rmtree(_UPDATE_DIR)
