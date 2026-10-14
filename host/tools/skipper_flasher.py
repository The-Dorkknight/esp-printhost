#!/usr/bin/env python3
# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""Skipper Flipper Flasher: a small window to flash firmware. Linux, macOS, Windows.

    python3 host/tools/skipper_flasher.py

Slave tab:  the printer's own board (Klipper-based firmware with the segment queue).
            Uses firmware shipped in firmware/prebuilt/, or your own build (Linux/WSL).
            RP2040: UF2 drive copy. STM32 (e.g. Creality 4.2.x): copy to an SD card.
            AVR: avrdude.
Master tab: the ESP32 host; flashes the shipped ESP32-S3 image or a .bin you supply.

WORK IN PROGRESS: nothing here has been run on hardware. Flashing always asks first.
Needs Tk (Linux: sudo apt install python3-tk). Standard library only.
"""
import datetime as dt
import os
import queue
import re
import sys
import tkinter as tk
from tkinter import messagebox, ttk

import skipper_theme as theme
from skipper_fw import quote_argv
from printhost_gui import Proc, ERR_RE, HINTS, WIP_NOTICE

__version__ = "0.1.0-wip"


class FlasherApp:
    def __init__(self, root):
        import fw_panel
        import master_panel
        self.root = root
        self.q = queue.Queue()
        self.procs = {}
        root.title(f"Skipper Flipper Flasher {__version__}")
        root.geometry("980x720")
        root.minsize(900, 700)   # below this the log pane shrinks to a few lines
        self.fonts = theme.apply(root)
        theme.set_icon(root)
        theme.FilmStrip(root).pack(fill="x")
        self.header = theme.make_header(
            root, self.fonts, "FLASHER", [
                "Slave: the printer's own board. Master: the ESP32-S3 host (ESP32-S3 only).",
                "Firmware for every listed board ships with this app. Every flash asks first.",
                "On real hardware (a WEEDO Tina 2): heaters, PID, motors, stallGuard.",
                "Printing and tuning: not yet. Details: docs/HARDWARE_LOG.md.",
            ], badge=WIP_NOTICE)
        self.header.pack(fill="x")

        # bottom widgets first so the window never clips them
        self.status = ttk.Label(root, text="Ready.", style="Status.TLabel", anchor="w")
        self.status.pack(side="bottom", fill="x", padx=6, pady=(0, 4))
        theme.FilmStrip(root).pack(side="bottom", fill="x")

        self.nb = ttk.Notebook(root)
        self.nb.pack(fill="x", padx=6, pady=4)
        slave = ttk.Frame(self.nb)
        master = ttk.Frame(self.nb)
        self.nb.add(slave, text="Slave: printer board")
        self.nb.add(master, text="Master: ESP32-S3")
        self.slave = fw_panel.FirmwarePanel(slave, self)
        self.slave.pack(fill="x", expand=True)
        self.master = master_panel.MasterPanel(master, self)
        self.master.pack(fill="x", expand=True)

        of = ttk.Frame(root)
        of.pack(fill="both", expand=True, padx=6, pady=(0, 4))
        self.text = tk.Text(of, height=10, wrap="word")
        sb = ttk.Scrollbar(of, command=self.text.yview)
        self.text.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.text.pack(side="left", fill="both", expand=True)
        theme.style_text(self.text, self.fonts)
        self.text.tag_configure("esp", foreground=theme.INK, background="#e2dfd3")
        # read-only log that still allows select-all / copy (Ctrl on Linux/Windows, Command on macOS)
        mod = 0x8 if sys.platform == "darwin" else 0x4
        self.text.bind("<Key>", lambda e: None if (e.state & mod and e.keysym.lower() in ("c", "a")) else "break")
        self.info("Flashing is never automatic: every flash asks first. Hardware results so far: docs/HARDWARE_LOG.md.")
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.after(50, self.poll)

    def _emit(self, tag, msg):
        ts = dt.datetime.now().strftime("%H:%M:%S")
        style = "err" if ERR_RE.search(msg.lower()) else tag
        self.text.insert("end", ts + " ", "ts")
        self.text.insert("end", f"[{tag:4}] ", tag)
        self.text.insert("end", msg + "\n", style)
        for needle, hint in HINTS:
            if needle.lower() in msg.lower():
                self.text.insert("end", " " * 11 + hint + "\n", "hint")
        self.text.see("end")

    def info(self, msg):
        self._emit("app", msg)

    def start(self, tag, argv, cwd=None):
        if tag in self.procs and self.procs[tag].running():
            self.info(f"{tag} already running")
            return False
        self._emit(tag, "$ " + quote_argv(argv))
        try:
            self.procs[tag] = Proc(tag, argv, self.q, cwd)
        except OSError as e:
            self._emit(tag, f"failed to start: {e}")
            return False
        self.status.config(text=f"Running {tag}...")
        return True

    def _flashing(self):
        return [t for t, p in self.procs.items() if p.running() and "flash" in p.argv]

    def stop(self, tag):
        p = self.procs.get(tag)
        if p and p.running():
            if "flash" in p.argv and not messagebox.askyesno(
                    "Stop flashing?", "A flash is in progress. Stopping it halfway can leave the board "
                    "not booting until it is flashed again. Stop anyway?", default="no"):
                return
            p.stop()
            self._emit(tag, "stopped")

    def poll(self):
        try:
            while True:
                tag, line = self.q.get_nowait()
                try:
                    if tag == "_done":
                        pr = self.procs.get(line)
                        if pr is not None and pr.running():
                            continue    # a newer process already took over this tag
                        self.status.config(text="Ready.")
                        (self.slave if line == "fw" else self.master).on_done(line)
                    elif line is None:
                        self._emit(tag, "finished OK")
                    else:
                        self._emit(tag, line)
                except Exception as e:   # noqa: BLE001 - one bad callback must not stop the log
                    self.q_error(e)
        except queue.Empty:
            pass
        finally:
            self.root.after(50, self.poll)

    def q_error(self, e):
        try:
            self.text.insert("end", f"internal error: {e!r}\n", "err")
        except tk.TclError:
            pass

    def on_close(self):
        if self._flashing() and not messagebox.askyesno(
                "Quit while flashing?", "A flash is still running. Closing now stops it halfway and can "
                "leave the board not booting until it is flashed again. Quit anyway?", default="no"):
            return
        for p in self.procs.values():
            p.stop()
        self.root.destroy()


def main():
    if "--version" in sys.argv[1:]:
        print(f"Skipper Flipper Flasher {__version__} - {WIP_NOTICE}")
        return
    root = tk.Tk()
    FlasherApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
