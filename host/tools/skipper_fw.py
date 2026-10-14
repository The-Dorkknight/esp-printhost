#!/usr/bin/env python3
# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""Skipper Flipper firmware tool: fetch Klipper, overlay the segment queue, build, flash.

    python3 tools/skipper_fw.py list
    python3 tools/skipper_fw.py check [board]
    python3 tools/skipper_fw.py fetch
    python3 tools/skipper_fw.py build skr-pico [--no-segq] [--menuconfig] [--jobs N]
    python3 tools/skipper_fw.py flash skr-pico [--artifact DIR] [--method M]
                                   [--port P] [--mount PATH] [--dry-run] [--yes]
    python3 tools/skipper_fw.py bundle [board ...]      (maintainer: refresh the shipped firmware)
    python3 tools/skipper_fw.py pins skr-pico
    python3 tools/skipper_fw.py overlay --apply|--remove [--klipper-dir DIR]

Everything lives in ~/.skipper-flipper (override with $SKIPPER_FLIPPER_HOME):
    klipper-v0.13.0/   shallow clone of Klipper at the pinned tag
    out/<board>-<stamp>/   build results (firmware + build-info.json)

HONESTY: most presets are only COMPILED; the status field of each preset says which
have run on hardware (see docs/HARDWARE_LOG.md). Flashing
is never implicit: it needs --yes or typing the board id, and --dry-run shows
the exact commands without touching anything.

Standard library only (Python 3.8+). Importable: the GUI uses the pure
functions below; only the slow steps run as a subprocess.
"""
import argparse
import datetime as dt
import glob
import hashlib
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import time

__version__ = "0.1.0-wip"

KLIPPER_URL = "https://github.com/Klipper3d/klipper.git"
KLIPPER_TAG = "v0.13.0"
KLIPPER_COMMIT = "61c0c8d2ef40340781835dd53fb04cc7a454e37a"   # peeled tag, checked with git ls-remote

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(HERE))
FW_DIR = os.path.join(REPO_ROOT, "firmware")
BOARDS_DIR = os.path.join(FW_DIR, "boards")
SKIPPER_SRC_DIR = os.path.join(FW_DIR, "skipper")
OVERLAY_FILES = ("segq.c", "segq_core.c", "segq_core.h")

KCONFIG_BEGIN = "# >>> skipper-flipper segq (managed by skipper_fw.py) >>>"
KCONFIG_END = "# <<< skipper-flipper segq <<<"
KCONFIG_BODY = (
    "config WANT_SEGQ\n"
    "    bool \"Skipper Flipper segment queue (experimental)\"\n"
    "    default y\n"
    "    depends on HAVE_GPIO\n")
MAKEFILE_BODY = "src-$(CONFIG_WANT_SEGQ) += segq.c segq_core.c\n"

FLASH_METHODS = ("uf2-copy", "sdcard-copy", "avrdude", "manual")
ARCHES = ("rp2040", "stm32", "avr")
VALID_STATUS = ("compile-only", "hardware-tested")
# What the tool needs per architecture: (binary, apt package)
ARCH_TOOLS = {
    "rp2040": [("arm-none-eabi-gcc", "gcc-arm-none-eabi"), ("arm-none-eabi-objcopy", "binutils-arm-none-eabi"),
               ("arm-none-eabi-size", "binutils-arm-none-eabi")],
    "stm32": [("arm-none-eabi-gcc", "gcc-arm-none-eabi"), ("arm-none-eabi-objcopy", "binutils-arm-none-eabi"),
              ("arm-none-eabi-size", "binutils-arm-none-eabi")],
    "avr": [("avr-gcc", "gcc-avr"), ("avr-objcopy", "binutils-avr"), ("avr-size", "binutils-avr")],
}
COMMON_TOOLS = [("git", "git"), ("make", "make"), ("gcc", "build-essential"), ("python3", "python3")]
# Libraries the cross compilers need (checked with `gcc -print-file-name=`)
ARCH_LIBS = {
    "rp2040": ("arm-none-eabi-gcc", "libc.a", "libnewlib-arm-none-eabi"),
    "stm32": ("arm-none-eabi-gcc", "libc.a", "libnewlib-arm-none-eabi"),
    "avr": ("avr-gcc", "libc.a", "avr-libc"),
}
SIZE_PREFIX = {"rp2040": "arm-none-eabi-", "stm32": "arm-none-eabi-", "avr": "avr-"}
ARTIFACT_NAMES = ("klipper.uf2", "klipper.bin", "klipper.elf.hex", "klipper.elf", "klipper.dict")
# Flash methods offered per chip family (STM32 boards: SD card; no DFU yet)
ARCH_METHODS = {"rp2040": ("uf2-copy", "manual"), "stm32": ("sdcard-copy", "manual"), "avr": ("avrdude", "manual")}
PRIMARY_EXT = {"uf2-copy": ".uf2", "sdcard-copy": ".bin", "avrdude": ".hex", "manual": None}
REQUIRED_KEYS = ("id", "title", "description", "mcu_arch", "flash_method", "dda_hz", "notes", "status", "sources")


def methods_for(preset):
    return ARCH_METHODS.get(preset["mcu_arch"], FLASH_METHODS)


def can_build_here(platform=None, which_fn=None):
    """(ok, reason). Local building needs Linux (or WSL on Windows); it is untested on macOS."""
    plat = platform or sys.platform
    which_fn = which_fn or shutil.which
    if plat.startswith("win"):
        return False, "Building needs Linux (or WSL on Windows). Use the shipped firmware here."
    if not which_fn("make") or not which_fn("git"):
        return False, "make and git are needed for building (see Check tools)."
    if plat == "darwin":
        return True, "macOS: building is untested; Klipper's Makefile is written for Linux."
    return True, ""


def say(msg=""):
    """Line-based, flushed output (the GUI streams it)."""
    print(msg, flush=True)


def home_dir():
    return os.environ.get("SKIPPER_FLIPPER_HOME") or os.path.expanduser("~/.skipper-flipper")


def klipper_dir():
    return os.path.join(home_dir(), "klipper-" + KLIPPER_TAG)


def out_root():
    return os.path.join(home_dir(), "out")


def prebuilt_root():
    """Firmware built by the author and shipped next to the tools (firmware/prebuilt/<board>/)."""
    return os.path.join(FW_DIR, "prebuilt")


# ------------------------------------------------------------------ presets
class FwError(Exception):
    pass


def validate_preset(meta, config_text=None):
    """Return a list of problems (empty = fine)."""
    p = []
    if not isinstance(meta, dict):
        return ["metadata is not a JSON object"]
    for k in REQUIRED_KEYS:
        if k not in meta:
            p.append(f"missing key '{k}'")
    if p:
        return p
    if not re.fullmatch(r"[a-z0-9][a-z0-9._-]*", str(meta["id"])):
        p.append("id must be lowercase letters, digits, . _ -")
    for k in ("title", "description"):
        if not isinstance(meta[k], str) or not meta[k].strip():
            p.append(f"'{k}' must be a non-empty string")
    if meta["mcu_arch"] not in ARCHES:
        p.append(f"mcu_arch must be one of {ARCHES}")
    if meta["flash_method"] not in FLASH_METHODS:
        p.append(f"flash_method must be one of {FLASH_METHODS}")
    if not isinstance(meta["dda_hz"], int) or isinstance(meta["dda_hz"], bool) or meta["dda_hz"] <= 0:
        p.append("dda_hz must be a positive integer")
    if meta["status"] not in VALID_STATUS:
        p.append(f"status must be one of {VALID_STATUS}")
    if meta["status"] == "hardware-tested" and not meta.get("tested_on"):
        p.append("status 'hardware-tested' needs a 'tested_on' note (who/when/what board)")
    for k in ("notes", "sources"):
        if not isinstance(meta[k], list) or not all(isinstance(x, str) for x in meta[k]):
            p.append(f"'{k}' must be a list of strings")
    if isinstance(meta["sources"], list) and not meta["sources"]:
        p.append("'sources' must say where the config came from")
    if meta["flash_method"] == "avrdude":
        a = meta.get("avrdude")
        if not (isinstance(a, dict) and a.get("part") and a.get("programmer")):
            p.append("flash_method avrdude needs an 'avrdude' object with 'part' and 'programmer'")
        if meta["mcu_arch"] != "avr":
            p.append("avrdude flashing only makes sense for mcu_arch avr")
    if config_text is not None:
        want = {"rp2040": "CONFIG_MACH_RP2040=y", "stm32": "CONFIG_MACH_STM32=y",
                "avr": "CONFIG_MACH_AVR=y"}.get(meta["mcu_arch"])
        lines = [l.strip() for l in config_text.splitlines()]
        if want and want not in lines:
            p.append(f".config must contain {want}")
        if "CONFIG_WANT_SEGQ=y" in lines or "# CONFIG_WANT_SEGQ is not set" in lines:
            p.append("preset .config must not pin CONFIG_WANT_SEGQ (the tool sets it per build)")
    return p


def load_presets(boards_dir=None):
    """Return {id: preset}. Each preset is its JSON dict plus 'config_text' and 'config_path'."""
    boards_dir = boards_dir or BOARDS_DIR
    res = {}
    for jp in sorted(glob.glob(os.path.join(boards_dir, "*.json"))):
        name = os.path.splitext(os.path.basename(jp))[0]
        cp = os.path.join(boards_dir, name + ".config")
        with open(jp, encoding="utf-8") as f:
            meta = json.load(f)
        if not os.path.isfile(cp):
            raise FwError(f"{name}.json has no matching {name}.config")
        with open(cp, encoding="utf-8") as f:
            text = f.read()
        problems = validate_preset(meta, text)
        if meta.get("id") != name:
            problems.append(f"id '{meta.get('id')}' does not match file name '{name}'")
        if problems:
            raise FwError(f"preset {name}: " + "; ".join(problems))
        meta = dict(meta)
        meta["config_text"], meta["config_path"] = text, cp
        res[name] = meta
    return res


def get_preset(board, boards_dir=None):
    presets = load_presets(boards_dir)
    if board not in presets:
        raise FwError(f"unknown board '{board}'. Known: {', '.join(presets) or '(none)'}")
    return presets[board]


def max_step_rate(dda_hz):
    """2*|steps| <= n  =>  at most one step every 2 DDA ticks per axis."""
    return dda_hz // 2


# ------------------------------------------------------------------ overlay
def _strip_block(text, begin, end):
    out, skipping, found = [], False, False
    for line in text.splitlines(keepends=True):
        s = line.strip()
        if s == begin:
            skipping, found = True, True
            continue
        if skipping:
            if s == end:
                skipping = False
            continue
        out.append(line)
    return "".join(out), found


def _add_block(text, begin, end, body):
    text, _ = _strip_block(text, begin, end)
    if text and not text.endswith("\n"):
        text += "\n"
    if text and not text.endswith("\n\n"):
        text += "\n"
    return text + begin + "\n" + body + end + "\n"


def _rw(path, fn):
    with open(path, encoding="utf-8") as f:
        old = f.read()
    new = fn(old)
    if new != old:
        with open(path, "w", encoding="utf-8", newline="") as f:
            f.write(new)
        return True
    return False


def overlay_status(kdir):
    """True if the segq overlay is fully applied to this Klipper tree."""
    try:
        with open(os.path.join(kdir, "src", "Kconfig"), encoding="utf-8") as f:
            k = KCONFIG_BEGIN in f.read()
        with open(os.path.join(kdir, "src", "Makefile"), encoding="utf-8") as f:
            m = KCONFIG_BEGIN in f.read()
    except OSError:
        return False
    files = all(os.path.isfile(os.path.join(kdir, "src", n)) for n in OVERLAY_FILES)
    return k and m and files


def source_hashes(preset, skipper_dir=None):
    """sha256 of every input that shapes a firmware build: the overlay sources and the preset .config.
    Stored as build-info.json "source_sha256" so a shipped build can be checked for staleness."""
    skipper_dir = skipper_dir or SKIPPER_SRC_DIR
    res = {n: sha256_file(os.path.join(skipper_dir, n)) for n in OVERLAY_FILES}
    res[os.path.basename(preset["config_path"])] = sha256_file(preset["config_path"])
    return res


def apply_overlay(kdir, skipper_dir=None):
    """Copy the segq sources into <kdir>/src and wire up Kconfig + Makefile. Idempotent."""
    skipper_dir = skipper_dir or SKIPPER_SRC_DIR
    src = os.path.join(kdir, "src")
    for need in ("Kconfig", "Makefile"):
        if not os.path.isfile(os.path.join(src, need)):
            raise FwError(f"{kdir} does not look like a Klipper tree (no src/{need})")
    changed = []
    for n in OVERLAY_FILES:
        s, d = os.path.join(skipper_dir, n), os.path.join(src, n)
        if not os.path.isfile(s):
            raise FwError(f"missing overlay source {s}")
        with open(s, "rb") as f:
            data = f.read()
        old = None
        if os.path.isfile(d):
            with open(d, "rb") as f:
                old = f.read()
        if old != data:
            with open(d, "wb") as f:
                f.write(data)
            changed.append("src/" + n)
    if _rw(os.path.join(src, "Kconfig"),
           lambda t: _add_block(t, KCONFIG_BEGIN, KCONFIG_END, KCONFIG_BODY)):
        changed.append("src/Kconfig")
    if _rw(os.path.join(src, "Makefile"),
           lambda t: _add_block(t, KCONFIG_BEGIN, KCONFIG_END, MAKEFILE_BODY)):
        changed.append("src/Makefile")
    return changed


def remove_overlay(kdir):
    """Undo apply_overlay exactly (also fine if it was never applied)."""
    src = os.path.join(kdir, "src")
    changed = []
    for n in ("Kconfig", "Makefile"):
        p = os.path.join(src, n)
        if os.path.isfile(p):
            def strip(t):
                new, _ = _strip_block(t, KCONFIG_BEGIN, KCONFIG_END)
                return new.rstrip("\n") + "\n" if new.strip() else new
            # _add_block added one blank separator line; strip() removes it too
            if _rw(p, strip):
                changed.append("src/" + n)
    for n in OVERLAY_FILES:
        p = os.path.join(src, n)
        if os.path.isfile(p):
            os.remove(p)
            changed.append("src/" + n)
    return changed


def set_config_option(text, name, value):
    """Set a boolean CONFIG_<name> in .config text (value True/False)."""
    key = "CONFIG_" + name
    lines = [l for l in text.splitlines()
             if not re.match(rf"^({key}=|# {key} is not set)", l.strip())]
    lines.append(f"{key}=y" if value else f"# {key} is not set")
    return "\n".join(lines) + "\n"


def config_has(text, name):
    return re.search(rf"^CONFIG_{name}=y\s*$", text, re.M) is not None


# ------------------------------------------------------------------ tools check
def which(name):
    return shutil.which(name)


def lib_found(compiler, libname, which_fn=which, run=subprocess.run):
    exe = which_fn(compiler)
    if not exe:
        return False
    try:
        r = run([exe, "-print-file-name=" + libname], capture_output=True, text=True, timeout=20)
    except (OSError, subprocess.SubprocessError):
        return False
    out = r.stdout.strip()
    return bool(out) and out != libname and os.path.exists(out)


def check_tools(arches, flash_methods=(), which_fn=which, libcheck=None):
    """Return (rows, missing_pkgs). rows = [(name, path|None, pkg)]."""
    libcheck = libcheck or (lambda c, l: lib_found(c, l, which_fn))
    rows, pkgs = [], []
    need = list(COMMON_TOOLS)
    for a in arches:
        need += ARCH_TOOLS[a]
    if "avrdude" in flash_methods:
        need.append(("avrdude", "avrdude"))
    seen = set()
    for name, pkg in need:
        if name in seen:
            continue
        seen.add(name)
        path = which_fn(name)
        rows.append((name, path, pkg))
        if not path and pkg not in pkgs:
            pkgs.append(pkg)
    for a in arches:
        comp, lib, pkg = ARCH_LIBS[a]
        if which_fn(comp) and not libcheck(comp, lib):
            rows.append((f"{lib} for {comp}", None, pkg))
            if pkg not in pkgs:
                pkgs.append(pkg)
    return rows, pkgs


def apt_hint(pkgs):
    return "sudo apt install " + " ".join(pkgs) if pkgs else ""


# ------------------------------------------------------------------ run helpers
def run_stream(argv, cwd=None, env=None, check=True):
    """Run a command, echo its output line by line (flushed)."""
    say("$ " + quote_argv(argv))
    try:
        # utf-8 + replace: a Windows console codepage must not kill the stream on odd bytes
        p = subprocess.Popen(argv, cwd=cwd, env=env, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, text=True,
                             encoding="utf-8", errors="replace", bufsize=1)
    except OSError as e:
        raise FwError(f"could not start {argv[0]}: {e}")
    with p.stdout:
        for line in p.stdout:
            say(line.rstrip("\n"))
    code = p.wait()
    if check and code != 0:
        raise FwError(f"command failed with exit code {code}: {quote_argv(argv)}")
    return code


def git_out(kdir, *args):
    r = subprocess.run(["git", "-C", kdir] + list(args), capture_output=True, text=True)
    if r.returncode != 0:
        raise FwError("git " + " ".join(args) + " failed: " + r.stderr.strip())
    return r.stdout.strip()


# ------------------------------------------------------------------ fetch
def verify_checkout(kdir):
    head = git_out(kdir, "rev-parse", "HEAD")
    if head != KLIPPER_COMMIT:
        raise FwError(f"{kdir} is at {head[:12]}, expected {KLIPPER_TAG} ({KLIPPER_COMMIT[:12]}). "
                      "Delete the folder to fetch again, or fix it by hand.")
    return head


def fetch(kdir=None, url=KLIPPER_URL):
    kdir = kdir or klipper_dir()
    if os.path.isdir(os.path.join(kdir, ".git")):
        head = verify_checkout(kdir)
        say(f"Klipper {KLIPPER_TAG} already present at {kdir} ({head[:12]}), skipping.")
        return kdir
    if os.path.exists(kdir):
        raise FwError(f"{kdir} exists but is not a git checkout; move it away first.")
    os.makedirs(os.path.dirname(kdir), exist_ok=True)
    tmp = kdir + ".part"
    shutil.rmtree(tmp, ignore_errors=True)
    say(f"Cloning Klipper {KLIPPER_TAG} (shallow, roughly 20-40 MB) from {url} ...")
    run_stream(["git", "clone", "--depth", "1", "--branch", KLIPPER_TAG, url, tmp])
    try:
        verify_checkout(tmp)
    except FwError:
        shutil.rmtree(tmp, ignore_errors=True)
        raise
    os.rename(tmp, kdir)
    say(f"Klipper {KLIPPER_TAG} ready at {kdir}")
    return kdir


# ------------------------------------------------------------------ build
def utc_stamp(now=None, fmt="%Y%m%d-%H%M"):
    return (now or dt.datetime.now()).strftime(fmt)


def unique_dir(parent, base):
    """parent/base, or parent/base-2, -3 ... whichever does not exist yet."""
    path, n = os.path.join(parent, base), 1
    while os.path.exists(path):
        n += 1
        path = os.path.join(parent, f"{base}-{n}")
    return path


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def parse_size_output(text):
    """Parse Berkeley `size` output (two lines). Returns dict or None."""
    for line in text.splitlines():
        parts = line.split()
        if len(parts) >= 4 and all(p.isdigit() for p in parts[:4]):
            t, d, b = int(parts[0]), int(parts[1]), int(parts[2])
            return {"text": t, "data": d, "bss": b, "flash_bytes": t + d, "ram_bytes": d + b}
    return None


def measure_size(arch, elf):
    exe = SIZE_PREFIX[arch] + "size"
    r = subprocess.run([exe, elf], capture_output=True, text=True)
    return parse_size_output(r.stdout) if r.returncode == 0 else None


def build(*args, **kwargs):
    """Build a board (see _build). Holds the build lock so nothing is flashed mid-build."""
    with _BuildLock():
        return _build(*args, **kwargs)


def _build(board, segq=True, menuconfig=False, jobs=None, boards_dir=None, kdir=None,
           out_base=None, now=None):
    preset = get_preset(board, boards_dir)
    kdir = kdir or fetch()
    verify_checkout(kdir)
    rows, pkgs = check_tools([preset["mcu_arch"]], [])
    if pkgs:
        raise FwError("missing tools: " + ", ".join(r[0] for r in rows if not r[1]) + "\nTry:  " + apt_hint(pkgs))
    say(f"Board: {preset['title']}  (segq {'ON' if segq else 'OFF'}, status: {preset['status']})")
    changed = apply_overlay(kdir)
    say("Overlay applied: " + (", ".join(changed) if changed else "already up to date"))
    cfg = set_config_option(preset["config_text"], "WANT_SEGQ", segq)
    with open(os.path.join(kdir, ".config"), "w", encoding="utf-8") as f:
        f.write(cfg)
    make = ["make", "-C", kdir]
    run_stream(make + ["olddefconfig"])
    if menuconfig:
        say("Opening Klipper menuconfig (needs an interactive terminal). Save and exit to continue.")
        subprocess.call(make + ["menuconfig"], stdin=None)
    with open(os.path.join(kdir, ".config"), encoding="utf-8") as f:
        final_cfg = f.read()
    has_seg = config_has(final_cfg, "WANT_SEGQ")
    if segq and not has_seg:
        raise FwError("CONFIG_WANT_SEGQ ended up off (menuconfig change?). Build aborted so a "
                      "'segq' build is never silently stock.")
    run_stream(make + ["clean"])
    jobs = jobs or os.cpu_count() or 2
    run_stream(make + [f"-j{jobs}"])
    out = os.path.join(kdir, "out")
    found = [n for n in ARTIFACT_NAMES if os.path.isfile(os.path.join(out, n))]
    primary_ext = PRIMARY_EXT.get(preset["flash_method"])
    if primary_ext and not any(n.endswith(primary_ext) for n in found):
        raise FwError(f"build finished but no *{primary_ext} was produced in {out}")
    dest = unique_dir(out_base or out_root(), f"{board}-{utc_stamp(now)}")
    os.makedirs(dest)
    arts = {}
    for n in found:
        shutil.copy2(os.path.join(out, n), os.path.join(dest, n))
        arts[n] = {"bytes": os.path.getsize(os.path.join(dest, n)), "sha256": sha256_file(os.path.join(dest, n))}
    with open(os.path.join(dest, "klipper.config"), "w", encoding="utf-8") as f:
        f.write(final_cfg)
    size = measure_size(preset["mcu_arch"], os.path.join(out, "klipper.elf")) if "klipper.elf" in found else None
    primary = next((n for n in found if primary_ext and n.endswith(primary_ext)), None)
    info = {
        "board": board, "title": preset["title"], "preset": os.path.basename(preset["config_path"]),
        "segq": bool(has_seg), "status": preset["status"],
        "klipper_tag": KLIPPER_TAG, "klipper_commit": git_out(kdir, "rev-parse", "HEAD"),
        "tool_version": __version__, "built": (now or dt.datetime.now()).isoformat(timespec="seconds"),
        "primary_artifact": primary, "size": size, "artifacts": arts,
        "source_sha256": source_hashes(preset),
        "note": "compile-only: this firmware has NOT been verified on hardware",
    }
    with open(os.path.join(dest, "build-info.json"), "w", encoding="utf-8") as f:
        json.dump(info, f, indent=2)
        f.write("\n")
    say("")
    if size:
        say(f"Size: flash {size['flash_bytes']} bytes, RAM (data+bss) {size['ram_bytes']} bytes")
    for n, a in arts.items():
        say(f"  {n}  {a['bytes']} bytes  sha256 {a['sha256'][:16]}...")
    say(f"BUILD OK -> {dest}")
    return dest


# ------------------------------------------------------------------ artifacts
def read_build_info(adir):
    p = os.path.join(adir, "build-info.json")
    try:
        with open(p, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def list_artifact_dirs(board, root=None):
    """Build dirs of a board, newest first (by the timestamp in the name)."""
    root = root or out_root()
    res = []
    for d in glob.glob(os.path.join(root, board + "-*")):
        name = os.path.basename(d)
        if re.fullmatch(re.escape(board) + r"-\d{8}-\d{4}(-\d+)?", name) and read_build_info(d):
            res.append(d)
    def key(d):
        m = re.search(r"-(\d{8}-\d{4})(?:-(\d+))?$", d)
        return (m.group(1), int(m.group(2) or 1))
    return sorted(res, key=key, reverse=True)


def find_latest_artifact(board, root=None):
    dirs = list_artifact_dirs(board, root)
    return dirs[0] if dirs else None


def primary_artifact(adir, method):
    """Path of the file to flash from a build dir for this flash method."""
    info = read_build_info(adir) or {}
    ext = PRIMARY_EXT.get(method)
    name = info.get("primary_artifact")
    if not isinstance(name, str) or os.path.basename(name) != name or name in ("", ".", ".."):
        name = None     # never follow a path out of the build folder
    if name and (ext is None or name.endswith(ext)) and os.path.isfile(os.path.join(adir, name)):
        return os.path.join(adir, name)
    if ext:
        hits = sorted(glob.glob(os.path.join(adir, "*" + ext)))
        hits = [h for h in hits if not h.endswith(".elf.hex") or ext == ".hex"]
        if hits:
            return hits[0]
    elif name:
        return os.path.join(adir, name)
    return None


def pins_from_dict(dict_path):
    """Pin names a firmware accepts, read from its klipper.dict (JSON): {"PA5": 5, ...} -> sorted names."""
    try:
        with open(dict_path, encoding="utf-8") as f:
            d = json.load(f)
    except (OSError, ValueError) as e:
        raise FwError(f"cannot read {dict_path}: {e}")
    enums = d.get("enumerations") if isinstance(d, dict) else None
    pins = enums.get("pin") if isinstance(enums, dict) else None
    if not isinstance(pins, dict):
        return []
    names = set()
    for k, v in pins.items():
        if isinstance(v, int) and not isinstance(v, bool):
            names.add(k)
        elif (isinstance(v, (list, tuple)) and len(v) == 2
              and all(isinstance(x, int) and not isinstance(x, bool) for x in v)):
            # Klipper stores runs as "gpio0": [0, 30]  ->  gpio0 .. gpio29
            m = re.fullmatch(r"(.*?)(\d+)", k)
            if m and 0 <= v[1] <= 4096:
                base = int(m.group(2))
                names.update(f"{m.group(1)}{base + i}" for i in range(v[1]))
            else:
                names.add(k)
    return sorted(names)


def find_prebuilt(board, root=None):
    """The shipped firmware folder for a board, or None."""
    d = os.path.join(root or prebuilt_root(), board)
    return d if read_build_info(d) else None


def bundle(boards=None, segq=True, boards_dir=None, dest_root=None, jobs=None, build_fn=None):
    """Build every board (or the given ones) and store the results as the shipped firmware:
    firmware/prebuilt/<board>/{klipper.uf2|.bin|.hex, klipper.dict, pins.json, build-info.json}.
    The .elf and the full config are not copied (size)."""
    build_fn = build_fn or build
    dest_root = dest_root or prebuilt_root()
    names = boards or sorted(load_presets())
    done = []
    for b in names:
        adir = build_fn(b, segq=segq, jobs=jobs)
        info = read_build_info(adir) or {}
        dst = os.path.join(dest_root, b)
        if os.path.isdir(dst):
            shutil.rmtree(dst)
        os.makedirs(dst)
        keep = [n for n in os.listdir(adir)
                if n in ("klipper.uf2", "klipper.bin", "klipper.elf.hex", "klipper.dict", "build-info.json")]
        for n in keep:
            shutil.copy2(os.path.join(adir, n), os.path.join(dst, n))
        if os.path.isfile(os.path.join(dst, "klipper.dict")):
            pins = pins_from_dict(os.path.join(dst, "klipper.dict"))
            with open(os.path.join(dst, "pins.json"), "w", encoding="utf-8") as f:
                json.dump({"board": b, "count": len(pins), "pins": pins,
                           "note": "pin names this firmware accepts (from klipper.dict); not a board pin map"},
                          f, indent=1)
                f.write("\n")
        meta = dict(info)
        meta["prebuilt"] = True
        meta["artifacts"] = {n: a for n, a in (info.get("artifacts") or {}).items() if n in keep}
        with open(os.path.join(dst, "build-info.json"), "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2)
            f.write("\n")
        say(f"bundled {b} -> {dst}")
        done.append(dst)
    return done


# ------------------------------------------------------------------ flashing: pure logic
def user_name():
    return os.environ.get("USER") or os.environ.get("LOGNAME") or ""


def mount_roots(user=None):
    u = user if user is not None else user_name()
    roots = [f"/media/{u}", f"/run/media/{u}", "/Volumes"] if u else ["/media", "/run/media", "/Volumes"]
    return roots


def windows_removable_drives(ctypes_mod=None):
    """Drive roots such as 'E:\\' that Windows reports as removable (GetDriveTypeW == 2).
    Empty list on other platforms or if the call fails."""
    try:
        ct = ctypes_mod
        if ct is None:
            import ctypes as ct
        k32 = ct.windll.kernel32
        mask = k32.GetLogicalDrives()
        res = []
        for i in range(26):
            if mask & (1 << i):
                root = chr(ord("A") + i) + ":\\"
                if k32.GetDriveTypeW(root) == 2:
                    res.append(root)
        return res
    except (AttributeError, OSError, ImportError):
        return []


def _mount_candidates(roots=None, windows_drives=None):
    """(name, path) of every mounted volume: subfolders of the media roots on Linux/macOS,
    removable drive letters on Windows."""
    res = []
    if roots is None and (windows_drives is not None or os.name == "nt"):
        drives = windows_drives if windows_drives is not None else windows_removable_drives()
        for d in drives:
            res.append((d, d))
        return res
    for r in (roots if roots is not None else mount_roots()):
        try:
            names = sorted(os.listdir(r)) if os.path.isdir(r) else []
        except OSError:
            continue
        for name in names:
            p = os.path.join(r, name)
            # macOS lists the startup disk in /Volumes as a symlink to "/": never a flash target
            if os.path.isdir(p) and os.path.realpath(p) != os.path.realpath(os.sep):
                res.append((name, p))
    return res


def find_uf2_drives(roots=None, label="RPI-RP2", windows_drives=None):
    """Mounted RP2040 boot drives: a volume named like the label, or holding INFO_UF2.TXT."""
    return [p for name, p in _mount_candidates(roots, windows_drives)
            if name.upper().startswith(label) or os.path.isfile(os.path.join(p, "INFO_UF2.TXT"))]


def find_sd_mounts(roots=None, windows_drives=None):
    """Candidate removable-media mounts (anything mounted under the media roots, boot drives excluded)."""
    return [p for name, p in _mount_candidates(roots, windows_drives)
            if not name.upper().startswith("RPI-RP2") and not os.path.isfile(os.path.join(p, "INFO_UF2.TXT"))]


def unique_sd_name(mount, stamp=None, ext=".bin"):
    """skipper-<stamp>.bin that does not exist on the card. The stock Creality bootloader
    ignores a firmware file whose name it has flashed before, so every flash needs a new one."""
    stamp = stamp or utc_stamp(fmt="%Y%m%d-%H%M%S")
    name, n = f"skipper-{stamp}{ext}", 1
    while os.path.exists(os.path.join(mount, name)):
        n += 1
        name = f"skipper-{stamp}-{n}{ext}"
    return name


def normalize_mount(mount):
    """'E:' (a drive with no slash) means 'the current folder on E:' on Windows; make it 'E:\\'."""
    if mount and len(mount) == 2 and mount[1] == ":" and mount[0].isalpha():
        return mount + os.sep
    return mount


def avrdude_argv(preset, hexfile, port, baud=None):
    """Same flags as Klipper's src/avr/Makefile `flash` rule: -D -Uflash:w:<hex>:i,
    plus -b when the preset needs a baud (e.g. Melzi bootloaders use 57600)."""
    a = preset["avrdude"]
    m = re.fullmatch(r"(?i)com(\d+)", port or "")
    if m and int(m.group(1)) > 9:
        port = "\\\\.\\" + port     # Windows needs \\.\COM10 for ports above COM9
    argv = ["avrdude", "-p" + a["part"], "-c" + a["programmer"], "-P" + port]
    b = baud or a.get("baud")
    if b:
        argv += ["-b", str(b)]
    argv += ["-D", f"-Uflash:w:{hexfile}:i"]
    return argv


class Step:
    """One thing the flasher will do: run a command or copy a file."""

    def __init__(self, kind, text, argv=None, src=None, dst=None):
        self.kind, self.text, self.argv, self.src, self.dst = kind, text, argv, src, dst

    def describe(self):
        if self.kind == "run":
            return "run:   " + quote_argv(self.argv)
        if self.kind == "copy":
            return f"copy:  {self.src} -> {self.dst}"
        return "note:  " + self.text


def quote_argv(argv):
    """argv as one display string, quoted for the local shell (cmd.exe style on Windows)."""
    if os.name == "nt":
        return subprocess.list2cmdline([str(a) for a in argv])
    return " ".join(shlex.quote(str(a)) for a in argv)


def plan_flash(preset, adir, method=None, port=None, mount=None, baud=None,
               roots=None, stamp=None):
    """Build the list of steps for flashing. Pure: touches nothing (reads dirs only).
    Raises FwError with a human message if something needed is missing."""
    method = method or preset["flash_method"]
    if method not in FLASH_METHODS:
        raise FwError(f"unknown flash method '{method}'")
    f = primary_artifact(adir, method)
    if method != "manual" and not f:
        raise FwError(f"no firmware file for method '{method}' in {adir}")
    steps = []
    mount = normalize_mount(mount)
    if method == "uf2-copy":
        if not mount:
            drives = find_uf2_drives(roots)
            if not drives:
                raise FwError("RPI-RP2 drive not found. Unplug the board, hold the BOOTSEL (boot) button, "
                              "plug USB in, release it: a drive named RPI-RP2 appears. Then try again "
                              "(or pass --mount PATH).")
            if len(drives) > 1:
                raise FwError("several boot drives found: " + ", ".join(drives) + " - pass --mount PATH")
            mount = drives[0]
        steps.append(Step("copy", "", src=f, dst=os.path.join(mount, os.path.basename(f))))
        steps.append(Step("note", "The board reboots by itself when the copy finishes; the RPI-RP2 drive then disappears. "
                                  "That is normal."))
    elif method == "sdcard-copy":
        if not mount:
            cands = find_sd_mounts(roots)
            if len(cands) != 1:
                raise FwError("pass --mount PATH to the SD card (" +
                              ("found: " + ", ".join(cands) if cands else "no removable media found") + ")")
            mount = cands[0]
        name = unique_sd_name(mount, stamp)
        steps.append(Step("copy", "", src=f, dst=os.path.join(mount, name)))
        if os.name != "nt":
            steps.append(Step("run", "", argv=["sync"]))
        steps.append(Step("note", "Eject the card safely, put it in the printer's SD slot, then power-cycle the printer. "
                                  "The bootloader flashes the file on boot; the screen may stay dark for a while. "
                                  "A new file name is used every time on purpose."))
    elif method == "avrdude":
        if not port:
            raise FwError("avrdude needs --port (e.g. /dev/ttyUSB0 or /dev/ttyACM0, or COM3 on Windows)")
        if port.startswith("-"):
            raise FwError(f"'{port}' is not a serial port name")
        steps.append(Step("run", "", argv=avrdude_argv(preset, f, port, baud)))
    else:   # manual
        steps.append(Step("note", f"Firmware file: {f or '(none yet - build first)'}"))
        steps.append(Step("note", "Flash it with your board's own method (see docs/FIRMWARE.md and the board's "
                                  "documentation). This tool will not touch the board."))
    return steps


def confirm_flash(preset, yes, input_fn=input, isatty=None):
    """Flashing is never implicit. True only with --yes or the board id typed back."""
    if yes:
        return True
    tty = sys.stdin.isatty() if isatty is None else isatty
    if not tty:
        return False
    say(f"Type the board id ({preset['id']}) to confirm flashing, anything else cancels:")
    try:
        return input_fn("> ").strip() == preset["id"]
    except EOFError:
        return False


# ------------------------------------------------------------------ build lock / verification
LOCK_STALE_SECONDS = 12 * 3600


def _lock_path():
    return os.path.join(home_dir(), ".build.lock")


def _pid_alive(pid):
    if os.name == "nt":
        return True     # cannot tell cheaply; the age limit below clears stale locks
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except OSError:
        return True
    return True


def build_in_progress(now=None):
    """True while a build (this tool, any window or terminal) holds the build lock."""
    path = _lock_path()
    try:
        age = (now if now is not None else time.time()) - os.path.getmtime(path)
        with open(path, encoding="utf-8") as f:
            pid = int((f.read().strip() or "0"))
    except (OSError, ValueError):
        return False
    if age > LOCK_STALE_SECONDS or (pid > 0 and not _pid_alive(pid)):
        return False
    return True


class _BuildLock:
    def __enter__(self):
        os.makedirs(home_dir(), exist_ok=True)
        for _ in range(2):
            try:
                fd = os.open(_lock_path(), os.O_CREAT | os.O_EXCL | os.O_WRONLY)
            except FileExistsError:
                if build_in_progress():
                    raise FwError("another build is already running (lock file " + _lock_path() + ")")
                try:
                    os.remove(_lock_path())     # stale
                except OSError:
                    pass
                continue
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                f.write(str(os.getpid()))
            return self
        raise FwError("could not take the build lock " + _lock_path())

    def __exit__(self, *exc):
        try:
            os.remove(_lock_path())
        except OSError:
            pass
        return False


def verify_artifact(adir, path):
    """Check that the file about to be flashed is byte-for-byte what build-info.json recorded.
    Returns the SHA-256; raises FwError on a missing record or a mismatch."""
    info = read_build_info(adir)
    if not isinstance(info, dict):
        raise FwError(f"{adir} has no readable build-info.json: cannot verify the firmware, refusing.")
    name = os.path.basename(path)
    rec = (info.get("artifacts") or {}).get(name)
    want = rec.get("sha256") if isinstance(rec, dict) else None
    if not isinstance(want, str) or not want:
        raise FwError(f"build-info.json has no SHA-256 for {name}: cannot verify the firmware, refusing.")
    try:
        got = sha256_file(path)
    except OSError as e:
        raise FwError(f"cannot read {path}: {e}")
    if got.lower() != want.lower():
        raise FwError(f"SHA-256 MISMATCH for {name}: build-info.json says {want[:16]}..., file is {got[:16]}... "
                      "The file was changed or damaged after it was built. Refusing to flash it.")
    return got


def copy_to_media(src, dst, exclusive=False):
    """Copy src to dst and force it to disk. On failure raises OSError with .complete = True if
    every byte had been written (an RP2040 boot drive may vanish right after the last write)."""
    size = os.path.getsize(src)
    written = 0
    try:
        with open(src, "rb") as fi, open(dst, "xb" if exclusive else "wb") as fo:
            for chunk in iter(lambda: fi.read(1 << 16), b""):
                fo.write(chunk)
                written += len(chunk)
            fo.flush()
            try:
                os.fsync(fo.fileno())
            except OSError:
                pass
    except OSError as e:
        e.complete = (written == size)
        raise


def flash(board, artifact=None, method=None, port=None, mount=None, baud=None,
          dry_run=False, yes=False, boards_dir=None, root=None, roots=None,
          input_fn=input, isatty=None, stamp=None, which_fn=shutil.which):
    preset = get_preset(board, boards_dir)
    adir = artifact or find_latest_artifact(board, root) or find_prebuilt(board)
    if not adir or not os.path.isdir(adir):
        raise FwError(f"no firmware found for '{board}' (no local build, none shipped). "
                      f"Run: skipper_fw.py build {board}")
    method = method or preset["flash_method"]
    if method not in FLASH_METHODS:
        raise FwError(f"unknown flash method '{method}'")
    if method not in methods_for(preset):
        raise FwError(f"method '{method}' does not fit {preset['title']} "
                      f"(a {preset['mcu_arch']} board: {', '.join(methods_for(preset))})")
    info = read_build_info(adir) or {}
    if info.get("board") not in (None, board):
        raise FwError(f"{adir} is a build for '{info.get('board')}', not '{board}'. Refusing.")
    if not dry_run and build_in_progress():
        raise FwError("a firmware build is running; wait for it to finish before flashing.")
    fwfile = primary_artifact(adir, method) if method != "manual" else None
    say(f"Board:    {preset['title']}")
    say(f"Build:    {adir}  (segq {'ON' if info.get('segq') else 'OFF'}, klipper {info.get('klipper_tag', '?')})")
    say(f"Method:   {method}")
    say(f"Status:   {preset['status']}" + ("  (never run on hardware by the author)"
                                          if preset["status"] != "hardware-tested" else ""))
    if fwfile:
        say(f"File:     {fwfile}")
        say(f"SHA-256:  {verify_artifact(adir, fwfile)}  (matches build-info.json)")
    steps = plan_flash(preset, adir, method, port, mount, baud, roots, stamp)
    say("Plan:")
    for s in steps:
        say("  " + s.describe())
    if dry_run:
        say("DRY RUN: nothing was changed.")
        return 0
    for s in steps:
        if s.kind == "copy":
            parent = os.path.dirname(s.dst)
            if not os.path.isdir(parent):
                raise FwError(f"{parent} is not an available drive/folder (card removed or not mounted?).")
            if os.name != "nt" and os.path.realpath(parent) == os.path.realpath(os.sep):
                raise FwError("refusing to write firmware into the filesystem root.")
        elif s.kind == "run" and s.argv and s.argv[0] == "avrdude" and not which_fn("avrdude"):
            raise FwError("avrdude is not installed (Linux: sudo apt install avrdude; macOS: brew install avrdude; "
                          "Windows: install avrdude and put it on PATH).")
    say("WARNING: wrong or broken firmware can leave a board unresponsive until it is re-flashed "
        "through its bootloader. Make sure this is the right board.")
    if not confirm_flash(preset, yes, input_fn, isatty):
        raise FwError("flashing not confirmed (use --yes, or type the board id in a terminal). Nothing changed.")
    if build_in_progress():
        raise FwError("a firmware build started while waiting; wait for it to finish. Nothing changed.")
    if fwfile:
        verify_artifact(adir, fwfile)       # again: the file may have changed while the prompt was open
    for s in steps:
        if s.kind == "copy":
            say(f"Copying to {s.dst} ...")
            sd = method == "sdcard-copy"
            try:
                copy_to_media(s.src, s.dst, exclusive=sd)
            except OSError as e:
                # An RP2040 boot drive vanishes the moment the write completes: not an error.
                if (method == "uf2-copy" and getattr(e, "complete", False)
                        and not os.path.exists(os.path.dirname(s.dst))):
                    say("Drive disappeared after the copy (the board rebooted): OK.")
                    continue
                hint = " (is the card write-protected or full?)" if method == "sdcard-copy" else ""
                raise FwError(f"copy failed: {e}{hint}")
            if sd:
                if sha256_file(s.dst) != sha256_file(s.src):
                    raise FwError(f"the file on the card does not match what was copied ({s.dst}). "
                                  "Delete it and try another card.")
            say("Copy done.")
        elif s.kind == "run":
            run_stream(s.argv)
        else:
            say(s.text)
    say("FLASH STEPS DONE")
    return 0


# ------------------------------------------------------------------ CLI
def cmd_list(args):
    presets = load_presets()
    if args.json:
        say(json.dumps({k: {x: v for x, v in p.items() if x not in ("config_text",)} for k, p in presets.items()}, indent=2))
        return 0
    for k, p in presets.items():
        say(f"{k:18} {p['mcu_arch']:7} {p['flash_method']:12} dda {p['dda_hz']} Hz  [{p['status']}]  {p['title']}")
    return 0


def cmd_check(args):
    presets = load_presets()
    if args.board:
        if args.board not in presets:
            raise FwError(f"unknown board '{args.board}'")
        sel = [presets[args.board]]
    else:
        sel = list(presets.values())
    arches = sorted({p["mcu_arch"] for p in sel})
    rows, pkgs = check_tools(arches, {p["flash_method"] for p in sel})
    for name, path, pkg in rows:
        say(f"  {'ok     ' if path else 'MISSING'} {name:32} {path or '(apt: ' + pkg + ')'}")
    kd = klipper_dir()
    say(f"  {'ok     ' if os.path.isdir(os.path.join(kd, '.git')) else 'MISSING'} Klipper {KLIPPER_TAG:25} "
        f"{kd if os.path.isdir(kd) else '(run: skipper_fw.py fetch)'}")
    if pkgs:
        say("")
        say("Install the missing tools with:")
        say("  " + apt_hint(pkgs))
        return 1
    say("All needed tools are present.")
    return 0


def cmd_overlay(args):
    kd = args.klipper_dir or klipper_dir()
    if args.remove:
        say("removed: " + (", ".join(remove_overlay(kd)) or "nothing to remove"))
    else:
        say("applied: " + (", ".join(apply_overlay(kd)) or "already up to date"))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog="skipper_fw.py", description=__doc__.split("\n")[0])
    ap.add_argument("--version", action="version", version=f"skipper_fw {__version__}")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("list", help="list board presets")
    p.add_argument("--json", action="store_true")
    p = sub.add_parser("check", help="are the toolchains installed?")
    p.add_argument("board", nargs="?")
    sub.add_parser("fetch", help=f"shallow-clone Klipper {KLIPPER_TAG}")
    p = sub.add_parser("build", help="build firmware for a board")
    p.add_argument("board")
    p.add_argument("--no-segq", action="store_true", help="stock Klipper without the segment queue")
    p.add_argument("--menuconfig", action="store_true", help="open Klipper menuconfig before building")
    p.add_argument("--jobs", type=int, default=None)
    p = sub.add_parser("flash", help="flash a built firmware (never implicit)")
    p.add_argument("board")
    p.add_argument("--artifact", help="build folder (default: newest for this board)")
    p.add_argument("--method", choices=FLASH_METHODS)
    p.add_argument("--port")
    p.add_argument("--baud", type=int)
    p.add_argument("--mount", help="RPI-RP2 drive or SD card mount point")
    p.add_argument("--dry-run", action="store_true", help="print the commands, change nothing")
    p.add_argument("--yes", action="store_true", help="skip the typed confirmation")
    p = sub.add_parser("bundle", help="build boards and store them as the shipped firmware (maintainer step)")
    p.add_argument("boards", nargs="*")
    p.add_argument("--no-segq", action="store_true")
    p = sub.add_parser("pins", help="list the pin names a built/shipped firmware accepts")
    p.add_argument("board")
    p = sub.add_parser("overlay", help="apply/remove the segq overlay in a Klipper tree")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--apply", action="store_true")
    g.add_argument("--remove", action="store_true")
    p.add_argument("--klipper-dir")
    args = ap.parse_args(argv)
    try:
        if args.cmd == "list":
            return cmd_list(args)
        if args.cmd == "check":
            return cmd_check(args)
        if args.cmd == "fetch":
            fetch()
            return 0
        if args.cmd == "build":
            build(args.board, segq=not args.no_segq, menuconfig=args.menuconfig, jobs=args.jobs)
            return 0
        if args.cmd == "flash":
            return flash(args.board, args.artifact, args.method, args.port, args.mount, args.baud,
                         args.dry_run, args.yes)
        if args.cmd == "bundle":
            bundle(args.boards or None, segq=not args.no_segq)
            return 0
        if args.cmd == "pins":
            d = find_latest_artifact(args.board) or find_prebuilt(args.board)
            if not d or not os.path.isfile(os.path.join(d, "klipper.dict")):
                raise FwError(f"no firmware dictionary for '{args.board}'")
            for n in pins_from_dict(os.path.join(d, "klipper.dict")):
                say(n)
            return 0
        if args.cmd == "overlay":
            return cmd_overlay(args)
    except FwError as e:
        say(f"error: {e}")
        return 2
    except KeyboardInterrupt:
        say("interrupted")
        return 130
    return 0


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(line_buffering=True, errors="replace")
    except AttributeError:
        pass
    sys.exit(main())
