#!/usr/bin/env python3
# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""Skipper Flipper ESP32-S3 flasher (ESP32-S3 ONLY): write a user-supplied firmware .bin to the ESP32 "master"
(host brain) with Espressif's esptool (python -m esptool). Linux, macOS and Windows.

    python3 tools/skipper_esp.py ports
    python3 tools/skipper_esp.py check
    python3 tools/skipper_esp.py detect --port P
    python3 tools/skipper_esp.py flash FILE --port P [--chip C] [--offset O] [--baud B]
                                  [--erase] [--dry-run] [--yes]

    python3 tools/skipper_esp.py install            (private virtualenv with esptool)
    python3 tools/skipper_esp.py shipped            (the image shipped in firmware/prebuilt-esp32s3/)

HONESTY: the ESP32-S3 firmware in this repository runs a WiFi access point and web pages, a
Klipper link to a printer board over UART, clock sync and printer.cfg [neopixel] LEDs. It does not
move motors or heat anything. See build-info.json for whether this exact build has run on hardware. You
can also flash a file YOU supply; for your own file the offset is your responsibility.

The image offset (own files) is your responsibility. Common ESP-IDF conventions (NOT verified for
this project): a merged image goes at 0x0, an individual app image at 0x10000. Some
chips place the bootloader at different offsets; check your build's flash instructions.

Flashing is never automatic: it needs --yes or typing the port back, and --dry-run
prints the exact commands without touching anything.

esptool is not bundled. Install hint: see install_hint(). On Debian/Mint a plain
pip install may be refused (externally-managed environment); you may need
--break-system-packages, or use pipx / a virtualenv instead.

Standard library only (Python 3.8+).
"""
import argparse
import glob as _glob
import os
import re
import subprocess
import sys

import hashlib
import json

from skipper_fw import FwError, Step, say, run_stream, sha256_file, FW_DIR

__version__ = "0.1.0-wip"

CHIPS = ("esp32s3",)   # this project targets the ESP32-S3 only; esptool refuses a different chip
DEFAULT_CHIP = "esp32s3"
BAUDS = (9600, 57600, 115200, 230400, 460800, 921600, 1500000, 2000000)
MAX_IMAGE = 16 * 1024 * 1024
_OFFSET_RE = re.compile(r"^0x[0-9a-fA-F]+$")


# ------------------------------------------------------------------ shipped ESP32-S3 image
ESP_SRC_DIR = os.path.join(FW_DIR, "esp32s3")
ESP_PREBUILT_DIR = os.path.join(FW_DIR, "prebuilt-esp32s3")
ESP_IMAGE_NAME = "skipper-esp32s3-merged.bin"
_SRC_FILES = ("CMakeLists.txt", "partitions.csv", "sdkconfig.defaults")


def esp_source_hashes(src_dir=None):
    """sha256 of every file that shapes the ESP32-S3 build (for the staleness check)."""
    src_dir = src_dir or ESP_SRC_DIR
    names = list(_SRC_FILES)
    main = os.path.join(src_dir, "main")
    if os.path.isdir(main):
        names += ["main/" + n for n in sorted(os.listdir(main))]
    comp = os.path.join(src_dir, "components")
    if os.path.isdir(comp):
        for root, _dirs, files in sorted(os.walk(comp)):
            for f in sorted(files):
                rel = os.path.relpath(os.path.join(root, f), src_dir).replace(os.sep, "/")
                names.append(rel)
    res = {}
    for n in names:
        path = os.path.join(src_dir, *n.split("/"))
        if os.path.isfile(path):
            res[n] = sha256_file(path)
    # The firmware also compiles the protocol library in host/ (as the "host" component).
    repo = os.path.dirname(FW_DIR)
    for sub in ("host/CMakeLists.txt",):
        p = os.path.join(repo, sub)
        if os.path.isfile(p):
            res[sub] = sha256_file(p)
    for d, exts in (("host/src", (".cpp",)), ("host/include/ph", (".h",)), ("firmware/skipper", (".c", ".h"))):
        full = os.path.join(repo, *d.split("/"))
        if os.path.isdir(full):
            for n in sorted(os.listdir(full)):
                if n.endswith(exts) and (d != "firmware/skipper" or n.startswith("segq_core")):
                    res[d + "/" + n] = sha256_file(os.path.join(full, n))
    return res


def shipped_image(root=None):
    """(path, info) of the shipped merged image, or None."""
    root = root or ESP_PREBUILT_DIR
    try:
        with open(os.path.join(root, "build-info.json"), encoding="utf-8") as f:
            info = json.load(f)
    except (OSError, ValueError):
        return None
    path = os.path.join(root, info.get("image", ESP_IMAGE_NAME))
    return (path, info) if os.path.isfile(path) else None


def verify_image(binfile, root=None):
    """If binfile is the shipped image, its SHA-256 must match build-info.json (refuse otherwise).
    Any other file is the user's own responsibility. Returns the info dict or None."""
    s = shipped_image(root)
    if not s or os.path.abspath(binfile) != os.path.abspath(s[0]):
        return None
    want = (s[1].get("sha256") or "").lower()
    got = sha256_file(binfile)
    if got != want:
        raise FwError(f"the shipped firmware file does not match its recorded SHA-256 "
                      f"(file {got[:12]}..., record {want[:12] or 'none'}...). Refusing to flash a damaged copy.")
    return s[1]


def bundle_image(merged, idf_version, root=None, src_dir=None):
    """Store a freshly built merged image as the shipped one (maintainer step)."""
    root = root or ESP_PREBUILT_DIR
    os.makedirs(root, exist_ok=True)
    dst = os.path.join(root, ESP_IMAGE_NAME)
    with open(merged, "rb") as a, open(dst, "wb") as b:
        b.write(a.read())
    info = {
        "target": "esp32s3", "image": ESP_IMAGE_NAME, "offset": "0x0", "flash_size": "2MB",
        "bytes": os.path.getsize(dst), "sha256": sha256_file(dst), "idf": idf_version,
        "firmware_version": __version__, "status": "compile-only",
        "source_sha256": esp_source_hashes(src_dir),
        "contents": "bootloader@0x0 + partition table@0x8000 + app@0x10000 (merged, flash at 0x0)",
        "note": "WiFi AP and web pages, Klipper link over UART, clock sync, printer.cfg, heaters with Klipper's "
                "safety checks, PID tuning, fans, TMC2209, motor and stallGuard tests, moves, homing, printing "
                "from flash. Earlier builds ran in a WEEDO Tina 2 (mostly working, needs tuning; docs/HARDWARE_LOG.md). "
                "Status resets to compile-only on every rebuild until this exact build is run on hardware",
    }
    with open(os.path.join(root, "build-info.json"), "w", encoding="utf-8") as f:
        json.dump(info, f, indent=2)
        f.write("\n")
    return dst


# ------------------------------------------------------------------ esptool
def venv_dir():
    """Private virtualenv for esptool (~/.skipper-flipper/esp-venv). Needed because Debian/Mint
    refuse `pip install` into the system Python (externally managed environment)."""
    from skipper_fw import home_dir
    return os.path.join(home_dir(), "esp-venv")


def venv_python(vdir=None):
    vdir = vdir or venv_dir()
    return os.path.join(vdir, "Scripts", "python.exe") if os.name == "nt" else os.path.join(vdir, "bin", "python")


def _py(python):
    """Interpreter for esptool: an explicit one, else our private venv if it exists, else this one."""
    if python:
        return python
    vp = venv_python()
    return vp if os.path.isfile(vp) else sys.executable


def esptool_available(python=None, run=subprocess.run):
    """Return (bool, version text or message) by running `python -m esptool version`."""
    py = _py(python)
    try:
        r = run([py, "-m", "esptool", "version"], stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, text=True,
                 encoding="utf-8", errors="replace", timeout=30)
    except (OSError, subprocess.SubprocessError) as e:
        return False, f"could not run {py}: {e}"
    out = (r.stdout or "").strip()
    if r.returncode != 0:
        return False, out.splitlines()[-1] if out else f"esptool not found for {py}"
    lines = [ln for ln in out.splitlines() if ln.strip()]
    return True, lines[-1].strip() if lines else "esptool (version unknown)"


def install_argv(python=None, in_venv=None):
    """pip command as an argv list (no shell splitting: interpreter paths may contain spaces).
    --user is refused by pip inside a virtualenv, so it is left out there."""
    if in_venv is None:
        in_venv = getattr(sys, "base_prefix", sys.prefix) != sys.prefix
    argv = [_py(python), "-m", "pip", "install"]
    if not in_venv:
        argv.append("--user")
    return argv + ["esptool"]


def install_hint(python=None):
    return "Press 'Install esptool' in the Master tab, or run: python3 skipper_esp.py install"


def install_esptool(vdir=None, base_python=None, runner=None, available=None):
    """Create a private virtualenv and pip-install esptool into it. Works on Debian/Mint where
    `pip install --user` is refused. Needs internet (PyPI) and, on Debian/Ubuntu/Mint, the
    python3-venv package. Returns the venv's interpreter path."""
    runner = runner or run_stream
    available = available or esptool_available
    vdir = vdir or venv_dir()
    vpy = venv_python(vdir)
    ok, msg = available(vpy) if os.path.isfile(vpy) else (False, "")
    if ok:
        say(f"esptool already installed in {vdir}: {msg}")
        return vpy
    base = base_python or sys.executable
    if not os.path.isfile(vpy):
        say(f"Creating a private Python environment in {vdir} (nothing system-wide is changed)...")
        try:
            runner([base, "-m", "venv", vdir])
        except FwError as e:
            raise FwError(f"could not create the environment: {e}\n"
                          "On Linux Mint / Ubuntu / Debian install the venv package first:  "
                          "sudo apt install python3-venv   (then press Install again)")
    say("Installing esptool from PyPI (needs internet)...")
    try:
        runner([vpy, "-m", "pip", "install", "--disable-pip-version-check", "esptool"])
    except FwError as e:
        raise FwError(f"pip could not install esptool: {e}. Check your internet connection / proxy.")
    ok, msg = available(vpy)
    if not ok:
        raise FwError(f"esptool installed but does not run: {msg}")
    say(f"esptool OK: {msg}")
    return vpy


# ------------------------------------------------------------------ ports
def list_ports(platform=None, glob_fn=_glob.glob, winreg=None):
    """Candidate serial ports, sorted, without pyserial."""
    plat = platform or sys.platform
    found = set()
    if plat.startswith("linux"):
        for pat in ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/serial/by-id/*"):
            found.update(glob_fn(pat))
    elif plat == "darwin":
        for pat in ("/dev/cu.usbserial*", "/dev/cu.usbmodem*", "/dev/cu.SLAB*", "/dev/cu.wchusbserial*"):
            found.update(glob_fn(pat))
    elif plat.startswith("win") or plat == "cygwin":
        if winreg is None:
            try:
                import winreg as _w
                winreg = _w
            except ImportError:
                return []
        try:
            key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM")
        except OSError:
            return []
        try:
            i = 0
            while True:
                try:
                    _name, value, _t = winreg.EnumValue(key, i)
                except OSError:
                    break
                if value:
                    found.add(str(value))
                i += 1
        finally:
            try:
                winreg.CloseKey(key)
            except Exception:
                pass
    return sorted(found)


ESPRESSIF_VID = "303a"   # USB vendor ID of the ESP32-S3's built-in USB port (and its ROM bootloader)


def windows_espressif_ports(winreg=None):
    """COM ports whose USB device has Espressif's vendor ID (Windows registry), else []."""
    if winreg is None:
        try:
            import winreg as _w
            winreg = _w
        except ImportError:
            return []
    out = []
    try:
        usb = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"SYSTEM\CurrentControlSet\Enum\USB")
    except OSError:
        return []

    def subkeys(k):
        i = 0
        while True:
            try:
                yield winreg.EnumKey(k, i)
            except OSError:
                return
            i += 1
    try:
        for dev in list(subkeys(usb)):
            if not dev.lower().startswith("vid_" + ESPRESSIF_VID):
                continue
            try:
                dk = winreg.OpenKey(usb, dev)
            except OSError:
                continue
            for inst in list(subkeys(dk)):
                try:
                    pk = winreg.OpenKey(dk, inst + r"\Device Parameters")
                    name, _t = winreg.QueryValueEx(pk, "PortName")
                    if name:
                        out.append(str(name))
                except OSError:
                    pass
    finally:
        try:
            winreg.CloseKey(usb)
        except Exception:
            pass
    return sorted(set(out))


def pick_esp_port(ports, espressif=(), realpath=os.path.realpath):
    """Best guess at the ESP32-S3's port among `ports`, or "" when it is not clear.

    1. a port known to be Espressif's USB (Linux /dev/serial/by-id name with "Espressif" or
       VID 303a; on Windows the `espressif` list from the registry), if exactly one;
    2. otherwise the only serial device left after dropping ones named as Klipper boards.
    Boards with a CH340/CP210x USB chip can't be told apart from printer boards using the same
    chips, so with several such ports nothing is picked and the user chooses.
    """
    ports = list(ports)
    byid = [p for p in ports if "/serial/by-id/" in p]
    hits = [p for p in ports if p in set(espressif)] or \
           [p for p in byid if "espressif" in p.lower() or ESPRESSIF_VID in p.lower()]
    if hits:
        devs = {realpath(p) for p in hits}
        return hits[0] if len(devs) == 1 else ""
    klipper = {realpath(p) for p in byid if "klipper" in p.lower()}
    devs = {realpath(p) for p in ports} - klipper
    if len(devs) != 1:
        return ""
    d = devs.pop()
    same = [p for p in ports if realpath(p) == d]
    named = [p for p in same if p in byid]
    return (named or same)[0]


# ------------------------------------------------------------------ planning
def _base(python, chip, port, baud=None):
    argv = [_py(python), "-m", "esptool"]
    if chip != "auto":
        argv += ["--chip", chip]
    argv += ["--port", port]
    if baud is not None:
        argv += ["--baud", str(baud)]
    return argv


def _check_chip_port(chip, port):
    if chip not in CHIPS:
        raise FwError(f"unknown chip '{chip}' (choose from: {', '.join(CHIPS)})")
    if not port or not str(port).strip():
        raise FwError("a serial port is required (see: skipper_esp.py ports)")
    if str(port).strip().startswith("-"):
        raise FwError(f"'{port}' is not a serial port name")


def chip_info_argv(port, chip=DEFAULT_CHIP, python=None, hyphen=True):
    """Read-only detect command: esptool flash-id prints the chip, its features (embedded flash /
    PSRAM), the MAC and the REAL detected flash size. Writes nothing."""
    _check_chip_port(chip, port)
    return _base(python, chip, port) + ["flash-id" if hyphen else "flash_id"]


def plan_flash(binfile, port, chip=DEFAULT_CHIP, offset="0x0", baud=460800, erase=False, python=None,
               hyphen=True):
    """Pure planning; validates and returns a list of Step. Raises FwError on bad input."""
    _check_chip_port(chip, port)
    if not binfile or not os.path.isfile(binfile):
        raise FwError(f"firmware file not found: {binfile}")
    size = os.path.getsize(binfile)
    if size == 0:
        raise FwError(f"firmware file is empty: {binfile}")
    if size > MAX_IMAGE:
        raise FwError(f"firmware file is {size} bytes, over the 16 MiB limit")
    if not isinstance(offset, str) or not _OFFSET_RE.match(offset):
        raise FwError(f"bad offset '{offset}' (expected hex like 0x0 or 0x10000)")
    if baud not in BAUDS:
        raise FwError(f"unsupported baud {baud} (choose from: {', '.join(map(str, BAUDS))})")
    # esptool 5 renamed write_flash/erase_flash to write-flash/erase-flash (old names still work
    # but are deprecated); esptool 4 only knows the underscore names.
    wr, er = ("write-flash", "erase-flash") if hyphen else ("write_flash", "erase_flash")
    steps = []
    if erase:
        steps.append(Step("run", "", argv=_base(python, chip, port, baud) + [er]))
    steps.append(Step("run", "", argv=_base(python, chip, port, baud) + [wr, offset, binfile]))
    return steps


def esptool_major(version_text):
    """Major version from text such as 'esptool v5.5.0' or 'esptool.py v4.8.1'; None if unknown."""
    m = re.search(r"v?(\d+)\.\d+", version_text or "")
    return int(m.group(1)) if m else None


def confirm_flash(port, yes, input_fn=input, isatty=None):
    """True only with --yes or the port typed back in a terminal."""
    if yes:
        return True
    tty = sys.stdin.isatty() if isatty is None else isatty
    if not tty:
        return False
    say(f"Type the port ({port}) to confirm flashing, anything else cancels:")
    try:
        return input_fn("> ").strip() == port
    except EOFError:
        return False


def flash(binfile, port, chip=DEFAULT_CHIP, offset="0x0", baud=460800, erase=False,
          dry_run=False, yes=False, python=None, input_fn=input, isatty=None, runner=run_stream):
    steps = plan_flash(binfile, port, chip, offset, baud, erase, python)
    shipped = verify_image(binfile)
    if shipped and offset != shipped.get("offset", "0x0"):
        raise FwError(f"the shipped image is a merged image and must be flashed at {shipped.get('offset', '0x0')}, "
                      f"not {offset}")
    say(f"Firmware: {binfile}  ({os.path.getsize(binfile)} bytes)")
    if shipped:
        say(f"Shipped:  Skipper Flipper ESP32-S3 {shipped.get('firmware_version')} built with ESP-IDF "
            f"{shipped.get('idf')}, SHA-256 {shipped.get('sha256', '')[:16]}... matches. "
            f"Status: {shipped.get('status')}"
            + (f" (tested on hardware {(shipped.get('hardware_test') or {}).get('date', '')})"
               if shipped.get("status") == "hardware-tested" else " (this exact build has not run on hardware yet)"))
    say(f"Target:   {chip} on {port} at offset {offset}, {baud} baud" + (", full erase first" if erase else ""))
    if not shipped:
        say("Note:     your own file: the offset is your responsibility (merged image 0x0, app-only 0x10000).")
    say("Plan:")
    for s in steps:
        say("  " + s.describe())
    if dry_run:
        say("DRY RUN: nothing was changed.")
        return 0
    ok, msg = esptool_available(python)
    if not ok:
        raise FwError(f"esptool not available ({msg}). {install_hint()}")
    major = esptool_major(msg)
    if major is not None and major < 5:
        steps = plan_flash(binfile, port, chip, offset, baud, erase, python, hyphen=False)
        say(f"esptool {major}.x detected: using its command names instead:")
        for s in steps:
            say("  " + s.describe())
    say("WARNING: flashing the wrong image can leave the board unresponsive until re-flashed.")
    if not confirm_flash(port, yes, input_fn, isatty):
        raise FwError("flashing not confirmed (use --yes, or type the port in a terminal). Nothing changed.")
    for s in steps:
        runner(s.argv)
    say("FLASH STEPS DONE")
    return 0


# ------------------------------------------------------------------ USB log monitor
def monitor(port, seconds=None, reset=False, serial_mod=None, clock=None, out=None):
    """Print the ESP32-S3's USB console (read-only). reset=True pulses RTS first, which restarts
    the chip through its USB serial/JTAG port, so the boot messages (including the reason for the
    last restart) are captured. Needs pyserial (installed together with esptool)."""
    import time
    clock = clock or time.monotonic
    out = out or say
    if serial_mod is None:
        import serial as serial_mod   # ImportError handled by the caller
    if not port or str(port).startswith("-"):
        raise FwError("a serial port is required")
    s = serial_mod.Serial()
    s.port = port
    s.baudrate = 115200   # ignored by the native USB port, needed by USB-UART bridges
    s.timeout = 0.3
    s.dtr = False         # do not hold the chip in reset or boot mode on open
    s.rts = False
    try:
        s.open()
    except Exception as e:   # noqa: BLE001 - pyserial raises its own SerialException
        raise FwError(f"could not open {port}: {e}")
    try:
        if reset:
            out("(restarting the chip to capture its boot messages)")
            s.rts = True
            time.sleep(0.1)
            s.rts = False
        out(f"--- USB log from {port} (read-only; Stop ends it) ---")
        end = clock() + seconds if seconds else None
        pending = b""
        while end is None or clock() < end:
            data = s.read(256)
            if not data:
                continue
            pending += data
            *lines, pending = pending.split(b"\n")
            for ln in lines:
                out(ln.rstrip(b"\r").decode("utf-8", "replace"))
        if pending:
            out(pending.decode("utf-8", "replace"))
    finally:
        s.close()
    return 0


def _monitor_cli(args):
    try:
        import serial  # noqa: F401
    except ImportError:
        vpy = venv_python()
        if os.path.isfile(vpy) and os.path.abspath(vpy) != os.path.abspath(sys.executable):
            argv = [vpy, "-u", os.path.abspath(__file__), "monitor", "--port", args.port]
            if args.seconds:
                argv += ["--seconds", str(args.seconds)]
            if args.reset:
                argv.append("--reset")
            return subprocess.call(argv)
        raise FwError("pyserial is missing: press 'Install esptool' first (it brings pyserial)")
    return monitor(args.port, args.seconds, args.reset)


# ------------------------------------------------------------------ CLI
def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="skipper_esp.py", description=__doc__.split("\n")[0],
        epilog="The shipped image's status (compile-only or hardware-tested) is in firmware/prebuilt-esp32s3/build-info.json; or supply your own .bin. "
               "Offsets (0x0 merged, 0x10000 app) are ESP-IDF conventions, not verified for this project.")
    ap.add_argument("--version", action="version", version=f"skipper_esp {__version__}")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ports", help="list candidate serial ports")
    sub.add_parser("shipped", help="show the shipped ESP32-S3 image (path, offset, SHA-256)")
    p = sub.add_parser("bundle-image", help="maintainer: store a built merged image as the shipped one")
    p.add_argument("file")
    p.add_argument("--idf", required=True, help="ESP-IDF version used, e.g. v5.4.1")
    sub.add_parser("check", help="is esptool installed?")
    p = sub.add_parser("monitor", help="show the ESP32-S3 USB log (read-only)")
    p.add_argument("--port", required=True)
    p.add_argument("--seconds", type=float, default=None, help="stop after this long (default: until stopped)")
    p.add_argument("--reset", action="store_true", help="restart the chip first to capture its boot messages")
    sub.add_parser("install", help="install esptool into a private virtualenv (~/.skipper-flipper/esp-venv)")
    p = sub.add_parser("detect", help="read-only: chip, features, MAC and real flash size (flash-id)")
    p.add_argument("--port", required=True)
    p.add_argument("--chip", choices=CHIPS, default=DEFAULT_CHIP)
    p = sub.add_parser("flash", help="flash a .bin (never implicit)")
    p.add_argument("file")
    p.add_argument("--port", required=True)
    p.add_argument("--chip", choices=CHIPS, default=DEFAULT_CHIP)
    p.add_argument("--offset", default="0x0")
    p.add_argument("--baud", type=int, default=460800)
    p.add_argument("--erase", action="store_true", help="erase the whole flash first")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--yes", action="store_true")
    args = ap.parse_args(argv)
    try:
        if args.cmd == "ports":
            ports = list_ports()
            for pt in ports:
                say(pt)
            if not ports:
                say("no serial ports found")
            return 0
        if args.cmd == "check":
            ok, msg = esptool_available()
            say(("esptool OK: " if ok else "esptool missing: ") + msg)
            if not ok:
                say("Install: " + install_hint())
            return 0 if ok else 1
        if args.cmd == "shipped":
            s = shipped_image()
            if not s:
                raise FwError("no shipped ESP32-S3 image")
            say(s[0])
            say(f"offset {s[1].get('offset')}  sha256 {s[1].get('sha256')}  status {s[1].get('status')}")
            return 0
        if args.cmd == "bundle-image":
            say(f"stored {bundle_image(args.file, args.idf)}")
            return 0
        if args.cmd == "monitor":
            return _monitor_cli(args)
        if args.cmd == "install":
            install_esptool()
            return 0
        if args.cmd == "detect":
            ok, msg = esptool_available()
            if not ok:
                raise FwError(f"esptool not available ({msg}). {install_hint()}")
            major = esptool_major(msg)
            run_stream(chip_info_argv(args.port, args.chip, hyphen=not (major is not None and major < 5)))
            return 0
        if args.cmd == "flash":
            return flash(args.file, args.port, args.chip, args.offset, args.baud, args.erase,
                         args.dry_run, args.yes)
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
