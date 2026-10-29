# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""Firmware tab for the Skipper Flipper tool panel.

Board presets, build and flash of Klipper-derived firmware with the segment
queue. All slow work runs `skipper_fw.py` in a subprocess (tag "fw") through
the host App's Proc/queue machinery, so the window never blocks.

THE FIRMWARE IS COMPILE-TESTED ONLY. The panel says so, and Flash always asks.
"""
import glob
import os
import shlex
import shutil
import subprocess
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import skipper_fw as fw

HERE = os.path.dirname(os.path.abspath(__file__))
FW_SCRIPT = os.path.join(HERE, "skipper_fw.py")
TERMINALS = (  # (binary, args before the command)
    ("x-terminal-emulator", ["-e"]),
    ("gnome-terminal", ["--"]),
    ("xfce4-terminal", ["-x"]),
    ("konsole", ["-e"]),
)


def find_terminal(which=shutil.which):
    for name, pre in TERMINALS:
        path = which(name)
        if path:
            return [path] + pre
    return None


def serial_ports():
    try:
        import skipper_esp
        return skipper_esp.list_ports()
    except Exception:   # noqa: BLE001 - fall back to the Linux globs
        return sorted(set(glob.glob("/dev/serial/by-id/*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")))


def open_path(path):
    """Show a folder in the system file manager (Linux, macOS, Windows)."""
    if sys.platform.startswith("win"):
        os.startfile(path)   # noqa: S606 - Windows only
    else:
        subprocess.Popen(["open" if sys.platform == "darwin" else "xdg-open", path])


UF2_REMINDER = (
    "If the log says the flash finished OK:\n\n"
    "1. Remove the BOOT jumper (or let go of the BOOTSEL button). While it is on, the board starts in its "
    "bootloader (it shows up as the RPI-RP2 drive) instead of running the firmware you just flashed.\n"
    "2. Unplug the board's USB cable and plug it in again.\n"
    "3. A board with the UART firmware (skr-pico-uart) no longer shows up as a USB serial device: that "
    "is normal. Talk to it through the ESP32 instead.\n\n"
    "Flashing never happens automatically; this is only a reminder.")


class FirmwarePanel(ttk.Frame):
    """Needs an App with: start(tag, argv), stop(tag), info(msg), status (ttk.Label),
    procs (dict), root, fonts."""

    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        self.presets = fw.load_presets()
        self.ids = list(self.presets)
        self.titles = [self.presets[i]["title"] for i in self.ids]
        self.board_title = tk.StringVar(value=self.titles[0])
        self.method = tk.StringVar()
        self.port = tk.StringVar()
        self.mount = tk.StringVar()
        self.latest = None
        self.running = False
        self._after_flash = None     # method of a flash in progress: remind about BOOT when it ends
        self.source = tk.StringVar(value="shipped")
        self.can_build, self.build_note = fw.can_build_here()
        self._build()
        self.board_changed()
        if not self.can_build or self.build_note:
            self.after(300, lambda: self.app.info(self.build_note))

    # ------------------------------------------------------------ layout
    def _build(self):
        pad = dict(padx=5, pady=2)
        sm = "Small.TButton"
        top = ttk.Frame(self)
        top.pack(fill="x")
        ttk.Label(top, text="Board:").grid(row=0, column=0, sticky="w", **pad)
        self.board_box = ttk.Combobox(top, textvariable=self.board_title, values=self.titles,
                                      state="readonly", width=42)
        self.board_box.grid(row=0, column=1, sticky="w", **pad)
        self.board_box.bind("<<ComboboxSelected>>", lambda e: self.board_changed())
        self.badge = ttk.Label(top, text=" COMPILE-ONLY / UNVERIFIED ", style="Badge.TLabel")
        self.badge.grid(row=0, column=2, sticky="w", **pad)
        top.columnconfigure(3, weight=1)
        self.desc = ttk.Label(self, text="", style="Muted.TLabel", wraplength=860, justify="left")
        self.desc.pack(fill="x", padx=8)

        sf = ttk.Frame(self)
        sf.pack(fill="x")
        ttk.Label(sf, text="Firmware:").pack(side="left", padx=5)
        ttk.Radiobutton(sf, text="Shipped with this app", value="shipped", variable=self.source,
                        command=self.refresh_latest).pack(side="left", padx=4)
        ttk.Radiobutton(sf, text="My own build", value="mine", variable=self.source,
                        command=self.refresh_latest).pack(side="left", padx=4)

        bf = ttk.Frame(self)
        bf.pack(fill="x", pady=(2, 0))
        self.btn_check = ttk.Button(bf, text="Check tools", style=sm, command=self.check_tools)
        self.btn_fetch = ttk.Button(bf, text="Fetch Klipper", style=sm, command=self.fetch_klipper)
        self.btn_menu = ttk.Button(bf, text="Menuconfig", style=sm, command=self.menuconfig)
        self.btn_build = ttk.Button(bf, text="Build", style=sm, command=self.build)
        for b in (self.btn_check, self.btn_fetch, self.btn_menu):
            b.pack(side="left", **pad)
        self.btn_build.pack(side="left", **pad)
        self.btn_stop = ttk.Button(bf, text="Stop", style=sm, command=lambda: self.app.stop("fw"))
        self.btn_stop.pack(side="left", **pad)

        ff = ttk.LabelFrame(self, text="Flash (never automatic)")
        ff.pack(fill="x", padx=6, pady=(2, 2))
        ttk.Label(ff, text="Method:").grid(row=0, column=0, sticky="w", **pad)
        self.method_box = ttk.Combobox(ff, textvariable=self.method, values=fw.FLASH_METHODS,
                                       state="readonly", width=12)
        self.method_box.grid(row=0, column=1, sticky="w", **pad)
        self.method_box.bind("<<ComboboxSelected>>", lambda e: self.method_changed())
        # port (avrdude) and drive/SD folder (uf2/sdcard) are never needed together: one slot
        self.lbl_target = ttk.Label(ff, text="Target:")
        self.lbl_target.grid(row=0, column=2, sticky="e", **pad)
        self.slot = ttk.Frame(ff)
        self.slot.grid(row=0, column=3, sticky="ew")
        self.port_box = ttk.Combobox(self.slot, textvariable=self.port, width=30)
        self.btn_ports = ttk.Button(self.slot, text="\u21bb", width=3, style=sm, command=self.refresh_ports)
        self.mount_entry = ttk.Entry(self.slot, textvariable=self.mount)
        self.btn_browse = ttk.Button(self.slot, text="Browse\u2026", style=sm, command=self.browse_mount)
        self.btn_flash = ttk.Button(ff, text="Flash\u2026", style=sm, command=self.flash)
        self.btn_flash.grid(row=0, column=4, sticky="ns", padx=8, pady=3)
        ff.columnconfigure(3, weight=1)
        self.hint = ttk.Label(ff, text="", style="Muted.TLabel", wraplength=860, justify="left")
        self.hint.grid(row=1, column=0, columnspan=5, sticky="w", padx=6)

        lf = ttk.Frame(self)
        lf.pack(fill="x")
        self.latest_lbl = ttk.Label(lf, text="", style="Muted.TLabel", wraplength=640, justify="left")
        self.latest_lbl.pack(side="left", fill="x", expand=True, padx=8)
        self.btn_open = ttk.Button(lf, text="Open output folder", style=sm, command=self.open_folder)
        self.btn_copy = ttk.Button(lf, text="Copy path", style=sm, command=self.copy_path)
        self.btn_copy.pack(side="right", **pad)
        self.btn_open.pack(side="right", **pad)

    # ------------------------------------------------------------ state
    @property
    def board(self):
        return self.ids[self.titles.index(self.board_title.get())]

    @property
    def preset(self):
        return self.presets[self.board]

    def board_changed(self):
        p = self.preset
        self.desc.config(text=p['description'])
        self.badge.config(text=f" {p['status'].upper()} / UNVERIFIED " if p["status"] == "compile-only"
                          else f" {p['status'].upper()} ")
        self.method_box["values"] = fw.methods_for(p)
        self.method.set(p["flash_method"])
        self.port.set("")
        self.mount.set("")
        self.refresh_ports()
        self.method_changed()
        self.refresh_latest()

    def method_changed(self):
        m = self.method.get()
        for w in (self.port_box, self.btn_ports, self.mount_entry, self.btn_browse):
            w.pack_forget()
        if m == "avrdude":
            self.lbl_target.config(text="Port:")
            self.port_box.pack(side="left", fill="x", expand=True, padx=2)
            self.btn_ports.pack(side="left", padx=2)
        elif m in ("uf2-copy", "sdcard-copy"):
            self.lbl_target.config(text="Drive / SD:")
            self.mount_entry.pack(side="left", fill="x", expand=True, padx=2)
            self.btn_browse.pack(side="left", padx=2)
        else:
            self.lbl_target.config(text="Target:")
        self.hint.config(text={
            "uf2-copy": "Hold BOOTSEL while plugging in USB first. Leave the drive empty to auto-detect RPI-RP2.",
            "sdcard-copy": "STM32 boards: copies a new skipper-<time>.bin to the SD card. Put the card in this "
                           "computer first, pick it with Browse, flash, eject, insert it in the printer, power-cycle.",
            "avrdude": "Pick the serial port; close anything else using it first.",
            "manual": "This tool flashes nothing; it only shows the file and how.",
        }.get(m, ""))
        self._buttons()

    def refresh_ports(self):
        self.port_box["values"] = serial_ports()

    def refresh_latest(self):
        if self.source.get() == "mine":
            self.latest = fw.find_latest_artifact(self.board)
            kind = "Your latest build"
            none = "No build of your own for this board yet. Press Build, or choose 'Shipped'."
        else:
            self.latest = fw.find_prebuilt(self.board)
            kind = "Shipped firmware"
            none = "No shipped firmware for this board."
        if self.latest:
            info = fw.read_build_info(self.latest) or {}
            size = (info.get("size") or {}).get("flash_bytes")
            txt = (f"{kind}: {os.path.basename(self.latest)}  "
                   f"(plain Klipper {info.get('klipper_tag', '?')}{', old segment queue' if info.get('segq') else ''}"
                   f"{', ' + str(size) + ' bytes' if size else ''})")
        else:
            txt = none
        self.latest_lbl.config(text=txt)
        self._buttons()

    def _buttons(self):
        busy = self.running
        for b in (self.btn_check, self.btn_fetch, self.btn_menu, self.btn_build):
            b.config(state="disabled" if busy or not self.can_build else "normal")
        self.btn_stop.config(state="normal" if busy else "disabled")
        can_flash = (not busy) and bool(self.latest)
        self.btn_flash.config(state="normal" if can_flash else "disabled")
        st = "normal" if self.latest else "disabled"
        self.btn_open.config(state=st)
        self.btn_copy.config(state=st)

    def on_done(self, tag):
        """Called by the App when a 'fw' subprocess ends."""
        self.running = False
        self.refresh_latest()
        method, self._after_flash = self._after_flash, None
        if method == "uf2-copy":
            messagebox.showinfo("Board flashed: remove the BOOT jumper", UF2_REMINDER)

    # ------------------------------------------------------------ actions
    def _run(self, args, cwd=None):
        if self.app.start("fw", [sys.executable, "-u", FW_SCRIPT] + args, cwd):
            self.running = True
            self._buttons()
            return True
        return False

    def _build_args(self):
        return ["build", self.board]

    def check_tools(self):
        self._run(["check", self.board])

    def fetch_klipper(self):
        self._run(["fetch"])

    def build(self):
        self._run(self._build_args())

    def menuconfig(self):
        argv = [sys.executable, FW_SCRIPT] + self._build_args() + ["--menuconfig"]
        cmd = " ".join(shlex.quote(a) for a in argv)
        term = find_terminal()
        if not term:
            self.app.info("No terminal program found. Run this in a terminal yourself:")
            self.app.info("  " + cmd)
            self.app.status.config(text="No terminal found: command shown in Output.")
            return
        shell = f"{cmd}; echo; read -r -p 'Press Enter to close' _"
        try:
            subprocess.Popen(term + ["bash", "-c", shell])
            self.app.info("Menuconfig opened in a terminal. Save, exit, and the build runs there; "
                          "press Refresh/Build here afterwards.")
            self.app.status.config(text="Menuconfig runs in its own terminal window.")
        except OSError as e:
            self.app.info(f"couldn't open a terminal ({e}). Run it yourself:  {cmd}")

    def browse_mount(self):
        start = next((r for r in fw.mount_roots() if os.path.isdir(r)), os.path.expanduser("~"))
        d = filedialog.askdirectory(initialdir=start, mustexist=True)
        if d:
            self.mount.set(d)

    def flash(self):
        if not self.latest or self.running:
            return
        m, p = self.method.get(), self.preset
        if m not in fw.methods_for(p):
            messagebox.showerror("Wrong method", f"Method '{m}' does not fit this board "
                                 f"({', '.join(fw.methods_for(p))}).")
            return
        port, mount = self.port.get().strip(), self.mount.get().strip()
        f = fw.primary_artifact(self.latest, m)
        if m == "uf2-copy" and not mount and not fw.find_uf2_drives():
            messagebox.showinfo("Put the board in BOOTSEL mode",
                                "No RPI-RP2 drive found.\n\nUnplug the board, hold its BOOTSEL (boot) button, "
                                "plug the USB cable in, then release the button. A drive named RPI-RP2 "
                                "appears. Press Flash again.")
            return
        if m == "avrdude" and not port:
            messagebox.showerror("No port", "Pick the board's serial port first.")
            return
        if m == "sdcard-copy" and not mount and len(fw.find_sd_mounts()) != 1:
            messagebox.showerror("No SD card", "Choose the SD card's folder with Browse.")
            return
        if m != "manual" and not f:
            messagebox.showerror("No firmware file", f"This build has no file for method '{m}'.")
            return
        sha = ""
        if f:
            try:
                sha = fw.verify_artifact(self.latest, f)
            except fw.FwError as e:
                messagebox.showerror("Firmware check failed", str(e))
                return
        which = "your own build" if self.source.get() == "mine" else "shipped with this app"
        target = port if m == "avrdude" else (mount or "(auto-detected)") if m != "manual" else "-"
        msg = (f"Board:   {p['title']}\nFile:    {f or '-'}  ({which})\n"
               f"SHA-256: {sha[:16] + '... (matches build-info.json)' if sha else '-'}\n"
               f"Method:  {m}\nTarget:  {target}\n\n"
               f"Status: {p['status']}. This firmware has never been run on hardware.\n\n"
               "Wrong or broken firmware can leave a board unresponsive until it is re-flashed "
               "through its bootloader. Continue?")
        if not messagebox.askokcancel("Flash firmware?", msg, icon="warning", default="cancel"):
            self.app.info("flash cancelled")
            return
        args = ["flash", self.board, "--artifact", self.latest, "--method", m, "--yes"]
        if port and m == "avrdude":
            args += ["--port", port]
        if mount and m in ("uf2-copy", "sdcard-copy"):
            args += ["--mount", mount]
        self._after_flash = m if self._run(args) else None

    def open_folder(self):
        if self.latest:
            try:
                open_path(self.latest)
            except OSError as e:
                self.app.info(f"couldn't open folder: {e}")

    def copy_path(self):
        if self.latest:
            self.app.root.clipboard_clear()
            self.app.root.clipboard_append(self.latest)
            self.app.root.update()
            self.app.status.config(text=f"Copied {self.latest}")
