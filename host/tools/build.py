#!/usr/bin/env python3
# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""Build mcu_probe + unit tests.

    python3 tools/build.py                 # check deps, build, run tests
    python3 tools/build.py --install-deps  # apt install missing deps first (sudo)
    python3 tools/build.py --clean         # wipe build/ first
    python3 tools/build.py --no-tests

Uses CMake if present; otherwise compiles directly with g++/clang++.
Output: host/build/mcu_probe and host/build/ph_tests
"""
import argparse
import glob
import os
import platform
import shutil
import subprocess
import sys

HOST = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUILD = os.path.join(HOST, "build")
IS_LINUX = platform.system() == "Linux"
APT_PKGS = {"c++": "build-essential", "cmake": "cmake", "zlib": "zlib1g-dev"}


def say(msg):
    print(msg, flush=True)


def run(cmd, **kw):
    say("$ " + " ".join(cmd))
    return subprocess.run(cmd, **kw).returncode


def find_cxx():
    for c in (os.environ.get("CXX"), "g++", "clang++", "c++"):
        if c and shutil.which(c):
            return c
    return None


def have_zlib(cxx):
    """Try compiling a tiny zlib program — the only reliable check."""
    if not cxx:
        return False
    os.makedirs(BUILD, exist_ok=True)
    src = os.path.join(BUILD, "_zcheck.cpp")
    with open(src, "w") as f:
        f.write("#include <zlib.h>\nint main(){return zlibVersion()?0:1;}\n")
    r = subprocess.run([cxx, src, "-lz", "-o", src + ".out"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for p in (src, src + ".out"):
        if os.path.exists(p):
            os.remove(p)
    return r.returncode == 0


def check_deps():
    cxx = find_cxx()
    missing = []
    if not cxx:
        missing.append("c++")
    if not shutil.which("cmake"):
        missing.append("cmake")   # optional: we can fall back
    if IS_LINUX and not have_zlib(cxx):
        missing.append("zlib")
    return cxx, missing


def install_deps(missing):
    pkgs = sorted({APT_PKGS[m] for m in missing})
    if not pkgs:
        return True
    if not shutil.which("apt-get"):
        say(f"Can't auto-install (no apt). Please install: {', '.join(pkgs)}")
        return False
    return run(["sudo", "apt-get", "install", "-y"] + pkgs) == 0


def build_cmake(jobs):
    if run(["cmake", "-S", HOST, "-B", BUILD, "-DCMAKE_BUILD_TYPE=Release"]) != 0:
        return False
    return run(["cmake", "--build", BUILD, "-j", str(jobs)]) == 0


def build_direct(cxx):
    """No CMake: compile everything straight with the compiler (same sources as CMakeLists.txt)."""
    segq = os.path.join(os.path.dirname(HOST), "firmware", "skipper")
    srcs = sorted(glob.glob(os.path.join(HOST, "src", "*.cpp")))
    tests = sorted(glob.glob(os.path.join(HOST, "test", "*.cpp")))
    inc = ["-I", os.path.join(HOST, "include"), "-I", segq]
    flags = ["-std=c++17", "-O2", "-Wall", "-Wextra"]
    os.makedirs(BUILD, exist_ok=True)
    # the segment-queue core is C99: compile it as C, then link it in
    cc = shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")
    segq_obj = os.path.join(BUILD, "segq_core.o")
    ok = run([cc, "-std=c99", "-O2", "-Wall", "-Wextra", "-I", segq, "-c",
              os.path.join(segq, "segq_core.c"), "-o", segq_obj]) == 0 if cc else False
    if not ok:
        return False
    ok = run([cxx] + flags + inc + srcs + tests + [segq_obj, "-o", os.path.join(BUILD, "ph_tests")]) == 0
    if IS_LINUX:
        for tool in ("mcu_probe", "klink_probe"):
            ok &= run([cxx] + flags + ["-Wno-missing-field-initializers"] + inc + srcs + [segq_obj]
                      + [os.path.join(HOST, "tools", tool + ".cpp"), "-lz",
                         "-o", os.path.join(BUILD, tool)]) == 0
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--install-deps", action="store_true", help="apt install missing packages (uses sudo)")
    ap.add_argument("--clean", action="store_true", help="delete build/ first")
    ap.add_argument("--no-tests", action="store_true", help="skip running unit tests")
    ap.add_argument("--no-cmake", action="store_true", help="compile directly even if CMake exists")
    ap.add_argument("-j", "--jobs", type=int, default=os.cpu_count() or 2)
    a = ap.parse_args()

    say(f"host dir: {HOST}")
    if a.clean and os.path.isdir(BUILD):
        say("cleaning build/")
        shutil.rmtree(BUILD)

    cxx, missing = check_deps()
    say(f"compiler: {cxx or 'MISSING'}   cmake: {'yes' if 'cmake' not in missing else 'no'}"
        f"   zlib: {'yes' if 'zlib' not in missing else ('no' if IS_LINUX else 'n/a')}")
    hard = [m for m in missing if m != "cmake"]
    if missing and a.install_deps:
        if not install_deps(missing):
            return 1
        cxx, missing = check_deps()
        hard = [m for m in missing if m != "cmake"]
    if hard:
        pkgs = " ".join(sorted({APT_PKGS[m] for m in hard}))
        say(f"ERROR: missing {', '.join(hard)}.\n  Fix:  sudo apt install {pkgs}\n"
            f"  or:   python3 tools/build.py --install-deps")
        return 1
    if not IS_LINUX:
        say("note: mcu_probe is Linux-only; building tests/library only.")

    use_cmake = "cmake" not in missing and not a.no_cmake
    say("building with " + ("CMake" if use_cmake else f"{cxx} directly"))
    ok = build_cmake(a.jobs) if use_cmake else build_direct(cxx)
    if not ok:
        say("ERROR: build failed (see messages above)")
        return 1

    probe = os.path.join(BUILD, "mcu_probe")
    if IS_LINUX:
        say(f"built: {probe}" if os.path.exists(probe) else "ERROR: mcu_probe was not produced")
        if not os.path.exists(probe):
            return 1

    if not a.no_tests:
        if run([os.path.join(BUILD, "ph_tests")]) != 0:
            say("ERROR: unit tests failed")
            return 1
    say("build OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
