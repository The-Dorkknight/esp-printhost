#!/usr/bin/env python3
# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""esp-printhost — one command does everything.

WORK IN PROGRESS: PROOF OF CONCEPT, MANY FEATURES MAY BE BROKEN  (0.1.0-wip)

    sudo python3 printhost.py            # install deps, fix serial access, build, open GUI
    python3 printhost.py                 # same; asks for your password itself

Options:
    --no-gui        set up + build only
    --clean         rebuild from scratch
    --check         only report what's installed / missing, change nothing
    --yes           answer "yes" to the optional fixes without asking
    --no-firmware   skip the optional firmware toolchains (Firmware tab builds)

What it does (Linux Mint / Ubuntu / Debian):
  1. apt installs only what's missing: build tools, cmake, zlib, python3-tk, acl
  2. serial access for YOU (no logout needed): udev rule + ACL on current ports,
     and adds you to 'dialout' for the long term
  3. tells ModemManager to leave printer boards alone
  4. offers to stop brltty grabbing CH340 USB-serial chips (Ender boards!)
  5. optional: apt installs the firmware toolchains (ARM + AVR gcc, avrdude,
     dfu-util, stm32flash; about 140 MB download) so the GUI's Firmware tab can
     build Klipper-based firmware. Skip with --no-firmware.
  6. builds mcu_probe + runs unit tests, as you (not root)
  7. opens the GUI as you
Everything is logged to ~/esp-printhost-logs/setup-<time>.log
"""
import argparse
import datetime as dt
import glob
import os
import platform
import pwd
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
HOST = os.path.join(ROOT, "host")
TOOLS = os.path.join(HOST, "tools")
BUILD = os.path.join(HOST, "build")
UDEV_RULE = "/etc/udev/rules.d/99-esp-printhost.rules"
BRLTTY_OVERRIDE = "/etc/udev/rules.d/85-brltty.rules"
APT_PKGS = ["build-essential", "cmake", "zlib1g-dev", "python3-tk", "acl"]
# Optional: toolchains for host/tools/skipper_fw.py (build + flash MCU firmware).
# Download is roughly 140 MB, about 1.1 GB installed (Ubuntu 24.04 package sizes).
FW_PKGS = ["git", "gcc-arm-none-eabi", "binutils-arm-none-eabi", "libnewlib-arm-none-eabi",
           "gcc-avr", "avr-libc", "binutils-avr", "dfu-util", "avrdude", "stm32flash"]
FW_SIZE_NOTE = "about 140 MB to download, ~1.1 GB on disk"
RULE_TEXT = """\
# esp-printhost: let the logged-in user use USB-serial printer boards,
# and keep ModemManager from probing them.
SUBSYSTEM=="tty", KERNEL=="ttyUSB[0-9]*", TAG+="uaccess", ENV{ID_MM_DEVICE_IGNORE}="1"
SUBSYSTEM=="tty", KERNEL=="ttyACM[0-9]*", TAG+="uaccess", ENV{ID_MM_DEVICE_IGNORE}="1"
"""

LOG = None
FAILS = []


# ---------------------------------------------------------------- output
def log(msg="", kind="  "):
    line = f"{kind} {msg}" if msg else ""
    print(line, flush=True)
    if LOG:
        LOG.write(line + "\n")
        LOG.flush()


def ok(msg): log(msg, "✔ ")
def warn(msg): log(msg, "! ")
def step(msg): log(); log(msg, "==")


def fail(msg):
    FAILS.append(msg)
    log(msg, "✘ ")


def run(cmd, user=None, env=None, check=False, quiet=False):
    """Run a command, stream + log its output. user= runs it as that user."""
    if user and os.geteuid() == 0 and user.pw_uid != 0:
        cmd = ["sudo", "-u", user.pw_name, "-H", "--preserve-env=PATH"] + cmd
    log("$ " + " ".join(cmd), "  ")
    try:
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             text=True, env=env, bufsize=1)
    except OSError as e:
        log(f"   (skipped: {e.strerror}: {cmd[0]})")
        if check:
            raise RuntimeError(f"cannot run {cmd[0]}")
        return 127
    for line in p.stdout:
        line = line.rstrip("\n")
        if line.startswith("(Reading database") and not line.endswith("installed.)"):
            continue                       # apt progress spam
        if not quiet:
            print("     " + line, flush=True)
        if LOG:
            LOG.write("     " + line + "\n")
    code = p.wait()
    if check and code != 0:
        raise RuntimeError(f"command failed ({code}): {' '.join(cmd)}")
    return code


def out(cmd):
    try:
        return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                              text=True).stdout.strip()
    except OSError:
        return ""


def ask(question, default_yes, auto_yes):
    if auto_yes:
        log(f"{question} -> yes (--yes)", "? ")
        return True
    if not sys.stdin.isatty():
        log(f"{question} -> {'yes' if default_yes else 'no'} (no terminal)", "? ")
        return default_yes
    hint = "[Y/n]" if default_yes else "[y/N]"
    try:
        a = input(f"?  {question} {hint} ").strip().lower()
    except EOFError:
        a = ""
    ans = default_yes if not a else a.startswith("y")
    if LOG:
        LOG.write(f"?  {question} -> {'yes' if ans else 'no'}\n")
    return ans


# ---------------------------------------------------------------- who am I
def target_user():
    """The human who ran sudo (or ourselves if not under sudo)."""
    name = os.environ.get("SUDO_USER") or os.environ.get("PKEXEC_UID")
    if os.environ.get("PKEXEC_UID"):
        return pwd.getpwuid(int(os.environ["PKEXEC_UID"]))
    if name and name != "root":
        return pwd.getpwnam(name)
    return pwd.getpwuid(os.getuid())


def open_log(user):
    global LOG
    d = os.path.join(user.pw_dir, "esp-printhost-logs")
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, dt.datetime.now().strftime("setup-%Y%m%d-%H%M%S.log"))
    LOG = open(path, "w", encoding="utf-8")
    if os.geteuid() == 0:
        for p in (d, path):
            os.chown(p, user.pw_uid, user.pw_gid)
    return path


# ---------------------------------------------------------------- steps
def pkg_installed(pkg):
    return out(["dpkg-query", "-W", "-f=${Status}", pkg]).endswith("install ok installed")


def step_packages(check_only):
    step("1. Packages")
    missing = [p for p in APT_PKGS if not pkg_installed(p)]
    for p in APT_PKGS:
        (ok if p not in missing else warn)(f"{p}: {'installed' if p not in missing else 'MISSING'}")
    if not missing or check_only:
        return
    env = dict(os.environ, DEBIAN_FRONTEND="noninteractive")
    cmd = ["apt-get", "install", "-y", "--no-install-recommends"] + missing
    if run(cmd, env=env) != 0:
        warn("install failed — refreshing package lists and retrying")
        run(["apt-get", "update"], env=env, quiet=True)
        if run(cmd, env=env) != 0:
            fail(f"could not install: {' '.join(missing)}")
            return
    still = [p for p in missing if not pkg_installed(p)]
    if still:
        fail(f"still missing after install: {' '.join(still)}")
    else:
        ok("all packages installed")


def step_serial(user, check_only):
    step("2. Serial port access")
    groups = out(["id", "-nG", user.pw_name]).split()
    have_rule = os.path.exists(UDEV_RULE) and open(UDEV_RULE).read() == RULE_TEXT
    (ok if "dialout" in groups else warn)(
        f"{user.pw_name} in 'dialout' group: {'yes' if 'dialout' in groups else 'no'}")
    (ok if have_rule else warn)(f"udev rule {UDEV_RULE}: {'present' if have_rule else 'missing'}")
    if check_only:
        return
    if "dialout" not in groups:
        if run(["usermod", "-aG", "dialout", user.pw_name]) == 0:
            ok("added to dialout (takes full effect next login — the steps below cover today)")
        else:
            fail("usermod failed")
    if not have_rule:
        os.makedirs(os.path.dirname(UDEV_RULE), exist_ok=True)
        with open(UDEV_RULE, "w") as f:
            f.write(RULE_TEXT)
        ok("udev rule written")
    run(["udevadm", "control", "--reload-rules"], quiet=True)
    run(["udevadm", "trigger", "--subsystem-match=tty", "--action=change"], quiet=True)
    # Immediate access to anything plugged in right now, even without a desktop seat.
    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if ports and shutil.which("setfacl"):
        for p in ports:
            run(["setfacl", "-m", f"u:{user.pw_name}:rw", p], quiet=True)
        ok(f"access granted now on: {', '.join(ports)}")
    elif not ports:
        log("no USB serial ports plugged in right now (that's fine)")


def step_brltty(auto_yes, check_only):
    step("3. brltty (braille driver that hijacks CH340 serial chips)")
    if not pkg_installed("brltty"):
        ok("brltty not installed — nothing to do")
        return
    os.makedirs(os.path.dirname(BRLTTY_OVERRIDE), exist_ok=True)
    if os.path.exists(BRLTTY_OVERRIDE):
        ok("brltty USB auto-grab already disabled")
        return
    warn("brltty is installed. On Mint/Ubuntu it steals CH340 adapters (most Ender/"
         "Creality boards), so /dev/ttyUSB0 vanishes when you plug the printer in.")
    if check_only:
        return
    if ask("Disable brltty's USB auto-start? (only matters if you use a braille display)",
           True, auto_yes):
        with open(BRLTTY_OVERRIDE, "w") as f:
            f.write("# emptied by esp-printhost: stop brltty grabbing USB-serial chips\n")
        run(["udevadm", "control", "--reload-rules"], quiet=True)
        run(["systemctl", "stop", "brltty-udev.service"], quiet=True)
        ok(f"disabled (undo: sudo rm {BRLTTY_OVERRIDE})")
    else:
        warn("left alone — if your board doesn't show up, re-run and say yes")


def step_firmware(auto_yes, check_only, skip):
    step("4. Firmware toolchains (optional)")
    if skip:
        log("skipped (--no-firmware)")
        return
    missing = [p for p in FW_PKGS if not pkg_installed(p)]
    for p in FW_PKGS:
        (ok if p not in missing else warn)(f"{p}: {'installed' if p not in missing else 'not installed'}")
    if not missing:
        return
    log("These are only needed to BUILD/FLASH board firmware from the Firmware tab; "
        "the Probe tab does not need them.")
    if check_only:
        warn(f"not installed: {' '.join(missing)}  ({FW_SIZE_NOTE})")
        return
    if not ask(f"Install firmware toolchains ({FW_SIZE_NOTE})?", True, auto_yes):
        log("skipped — re-run later without --no-firmware to add them")
        return
    env = dict(os.environ, DEBIAN_FRONTEND="noninteractive")
    cmd = ["apt-get", "install", "-y", "--no-install-recommends"] + missing
    if run(cmd, env=env) != 0:
        run(["apt-get", "update"], env=env, quiet=True)
        if run(cmd, env=env) != 0:
            # optional step: warn only, never block the GUI
            warn(f"could not install: {' '.join(missing)} (Firmware tab will say what is missing)")
            return
    still = [p for p in missing if not pkg_installed(p)]
    (warn if still else ok)(f"still missing: {' '.join(still)}" if still else "firmware toolchains installed")


def fix_ownership(user):
    """Earlier root builds leave root-owned files; hand them back."""
    if os.geteuid() != 0 or user.pw_uid == 0 or not os.path.isdir(BUILD):
        return
    for dirpath, dirnames, filenames in os.walk(BUILD):
        for n in [dirpath] + [os.path.join(dirpath, f) for f in filenames]:
            try:
                if os.lstat(n).st_uid != user.pw_uid:
                    os.lchown(n, user.pw_uid, user.pw_gid)
            except OSError:
                pass


def step_build(user, clean, check_only):
    step("5. Build mcu_probe + tests")
    probe = os.path.join(BUILD, "mcu_probe")
    if check_only:
        (ok if os.path.exists(probe) else warn)(
            f"mcu_probe: {'built' if os.path.exists(probe) else 'not built'}")
        return
    fix_ownership(user)
    argv = [sys.executable, "-u", os.path.join(TOOLS, "build.py")]
    if clean:
        argv.append("--clean")
    if run(argv, user=user) == 0:
        ok(f"built {probe}")
    else:
        fail("build failed — see messages above")


def gui_env(user):
    env = {k: v for k, v in os.environ.items()
           if not k.startswith("SUDO_") and k not in ("MAIL", "LOGNAME", "USER", "HOME")}
    env.update(HOME=user.pw_dir, USER=user.pw_name, LOGNAME=user.pw_name)
    env.setdefault("DISPLAY", ":0")
    if "XAUTHORITY" not in env:
        for cand in (os.path.join(user.pw_dir, ".Xauthority"),
                     *glob.glob(f"/run/user/{user.pw_uid}/gdm/Xauthority"),
                     *glob.glob(f"/run/user/{user.pw_uid}/.mutter-Xwaylandauth.*")):
            if os.path.exists(cand):
                env["XAUTHORITY"] = cand
                break
    # If the X cookie isn't readable by the user (e.g. root-owned), hand them a copy.
    xa = env.get("XAUTHORITY")
    if xa and os.path.exists(xa) and os.geteuid() == 0 and user.pw_uid != 0 \
            and os.stat(xa).st_uid != user.pw_uid:
        d = os.path.join(user.pw_dir, ".cache", "esp-printhost")
        os.makedirs(d, exist_ok=True)
        dst = os.path.join(d, "Xauthority")
        shutil.copyfile(xa, dst)
        os.chmod(dst, 0o600)
        for q in (os.path.dirname(d), d, dst):
            os.chown(q, user.pw_uid, user.pw_gid)
        env["XAUTHORITY"] = dst
    rt = f"/run/user/{user.pw_uid}"
    if os.path.isdir(rt):
        env.setdefault("XDG_RUNTIME_DIR", rt)
        env.setdefault("DBUS_SESSION_BUS_ADDRESS", f"unix:path={rt}/bus")
    return env


def step_gui(user):
    step("6. Launch GUI")
    gui = os.path.join(TOOLS, "printhost_gui.py")
    env = gui_env(user)
    cmd = [sys.executable, gui]
    if os.geteuid() == 0 and user.pw_uid != 0:
        # Drop root: the GUI and anything it runs belong to you.
        cmd = ["sudo", "-u", user.pw_name, "-H", "env"] + \
              [f"{k}={v}" for k, v in env.items()] + cmd
        env = None
    errlog = os.path.join(user.pw_dir, "esp-printhost-logs", "gui-output.log")
    log(f"opening GUI as {user.pw_name} on display {os.environ.get('DISPLAY', ':0')}")
    try:
        with open(errlog, "w") as ef:
            if os.geteuid() == 0:
                os.chown(errlog, user.pw_uid, user.pw_gid)
            p = subprocess.Popen(cmd, env=env, stdout=ef, stderr=subprocess.STDOUT,
                                 stdin=subprocess.DEVNULL, start_new_session=True)
    except OSError as e:
        fail(f"could not start GUI: {e}")
        return
    try:
        p.wait(timeout=4)                      # did it die straight away?
        early = open(errlog).read()
        for line in early.splitlines()[-15:]:
            log("     " + line)
        fail("GUI exited immediately (see above). No desktop session? "
             "Run this from a terminal inside your Mint desktop.")
    except subprocess.TimeoutExpired:
        ok(f"GUI is running (its output: {errlog}) — you can close this terminal")


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description="esp-printhost setup + launcher")
    ap.add_argument("--version", action="version",
                    version="esp-printhost 0.1.0-wip (proof of concept, work in progress)")
    ap.add_argument("--no-gui", action="store_true")
    ap.add_argument("--clean", action="store_true")
    ap.add_argument("--check", action="store_true", help="report only, change nothing")
    ap.add_argument("--yes", action="store_true", help="accept optional fixes")
    ap.add_argument("--no-firmware", action="store_true",
                    help="skip the optional firmware toolchains (ARM/AVR gcc, avrdude, ...)")
    a = ap.parse_args()

    if platform.system() != "Linux":
        print("This setup is for Linux Mint/Ubuntu (mcu_probe is Linux-only).\n"
              "On a Mac you can still open the GUI: python3 host/tools/printhost_gui.py")
        return 1
    if not shutil.which("apt-get"):
        print("No apt-get found — this script supports Mint/Ubuntu/Debian.")
        return 1
    if os.geteuid() != 0 and not a.check:
        print("Needs admin rights for installing packages — asking sudo…")
        os.execvp("sudo", ["sudo", sys.executable, os.path.abspath(__file__)] + sys.argv[1:])

    user = target_user()
    path = open_log(user)
    log(f"esp-printhost setup  {dt.datetime.now():%Y-%m-%d %H:%M:%S}")
    log(f"user: {user.pw_name}   distro: {out(['lsb_release', '-ds']) or platform.platform()}"
        f"   python: {platform.python_version()}")
    log(f"project: {ROOT}")
    log(f"log file: {path}")

    try:
        step_packages(a.check)
        step_serial(user, a.check)
        step_brltty(a.yes, a.check)
        step_firmware(a.yes, a.check, a.no_firmware)
        step_build(user, a.clean, a.check)
        if not a.no_gui and not a.check and not FAILS:
            step_gui(user)
    except KeyboardInterrupt:
        fail("interrupted")
    except Exception as e:  # keep going to the summary so the log is useful
        fail(f"unexpected error: {e!r}")

    step("Summary")
    if FAILS:
        for f in FAILS:
            log(f, "✘ ")
        log()
        log(f"Something went wrong. Send this file to Claude: {path}", "!!")
        return 1
    ok("all good")
    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    log(f"serial ports now: {', '.join(ports) if ports else 'none plugged in'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
