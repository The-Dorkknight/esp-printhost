#!/usr/bin/env python3
# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""Launcher for the Skipper Flipper Flasher window (Linux, macOS, Windows).

    python3 flasher.py

Needs Python 3.8+ with Tk (Linux: sudo apt install python3-tk; Windows/macOS
installers from python.org include it). No other install, no admin rights.
"""
import os
import runpy
import sys

TOOLS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "host", "tools")
sys.path.insert(0, TOOLS)
if __name__ == "__main__":
    if sys.version_info < (3, 8):
        sys.exit("Python 3.8 or newer is needed (this is %d.%d)." % sys.version_info[:2])
    try:
        import tkinter  # noqa: F401
    except ImportError:
        sys.exit("Tk is missing. Linux: sudo apt install python3-tk. Windows/macOS: install Python from python.org.")
    runpy.run_path(os.path.join(TOOLS, "skipper_flasher.py"), run_name="__main__")
