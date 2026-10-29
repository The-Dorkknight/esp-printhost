# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""Master tab: flash the ESP32-S3 host (ESP32-S3 only) ("master"): the shipped image or a .bin you supply.

"Flash for my board..." is the one-click path: it reads the chip (esptool flash-id, read-only), picks
the shipped build that fits its memory (DECKHAND / CAPTAIN / SKIPPER, skipper_esp.choose_tier) and
flashes it, after one confirmation.

The port box fills itself: it picks the ESP32-S3's own USB port when it can tell (see
skipper_esp.pick_esp_port) and rescans every 2 s while it is empty. The panel runs
`skipper_esp.py` (which uses Espressif's esptool) in a subprocess with tag "esp", through
the host App's start/stop/info machinery. Flash always asks first.
"""
import os
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import skipper_esp as esp

HERE = os.path.dirname(os.path.abspath(__file__))
ESP_SCRIPT = os.path.join(HERE, "skipper_esp.py")


def shipped_status_line():
    """One line from the shipped image's build-info.json (the single place its status is kept)."""
    s = esp.shipped_image()
    if not s:
        return "No shipped ESP32-S3 image in this copy."
    info = s[1]
    ht = info.get("hardware_test") or {}
    if info.get("status") == "hardware-tested" and ht:
        return f"Status: tested on hardware {ht.get('date', '')} (docs/HARDWARE_LOG.md)."
    return ("Status: this exact build has not run on hardware yet; earlier builds have (heaters, motors, "
            "TMC2209, stallGuard: docs/HARDWARE_LOG.md).")


FLASHED_REMINDER = (
    "If the log says the flash finished OK:\n\n"
    "1. Release the BOOT button (or remove any BOOT jumper / wire to GPIO0). While GPIO0 is held low the "
    "chip goes straight back into its bootloader instead of running the firmware.\n"
    "2. Unplug the ESP32's USB cable and plug it in again. Some boards do not restart by themselves "
    "after flashing over their USB port.\n"
    "3. After about 10 seconds the WiFi network SkipperFlipper-XXXX should appear.\n\n"
    "Flashing never happens automatically; this is only a reminder.")


class MasterPanel(ttk.Frame):
    """Needs an App with: start(tag, argv), stop(tag), info(msg), status, root."""

    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        self.binfile = tk.StringVar()
        self.port = tk.StringVar()
        self.chip = tk.StringVar(value=esp.DEFAULT_CHIP)
        self.offset = tk.StringVar(value="0x0")
        self.baud = tk.StringVar(value="460800")
        self.erase = tk.BooleanVar(value=False)
        self.running = False
        self._auto_job = None
        self._after_flash = False    # a flash was started: remind about BOOT when it ends
        self._build()
        self.refresh_ports(quiet=True)   # the app's log box may not exist yet: report once it does
        self.after_idle(self._report_initial_port)
        self.use_shipped(quiet=True)
        self._buttons()

    def _build(self):
        pad = dict(padx=5, pady=2)
        sm = "Small.TButton"
        ttk.Label(self, text=" ESP32-S3 ONLY. THREE BUILDS BY BOARD MEMORY: DECKHAND / CAPTAIN / SKIPPER ",
                  style="Badge.TLabel").pack(anchor="w", padx=6, pady=(4, 2))
        ttk.Label(self, style="Muted.TLabel", wraplength=900, justify="left",
                  text="Shipped image: WiFi SkipperFlipper-XXXX, pages at http://192.168.4.1/, status LED, Klipper "
                       "link to a printer board over UART with printer.cfg LEDs (docs/WIRING_SKR_PICO.md). Own .bin: merged image at 0x0, "
                       "app-only at 0x10000. " + shipped_status_line()).pack(fill="x", padx=8)

        af = ttk.Frame(self)
        af.pack(fill="x", padx=6, pady=(4, 0))
        self.btn_auto = ttk.Button(af, text="Flash for my board…", command=self.flash_auto)
        self.btn_auto.pack(side="left", padx=5, pady=2)
        ttk.Label(af, style="Muted.TLabel", wraplength=700, justify="left",
                  text="One click: reads the chip (read-only), picks the build that fits it and flashes it. "
                       "DECKHAND: any ESP32-S3. CAPTAIN: 4 MB flash + PSRAM (adds WiFi updates, more look-ahead). "
                       "SKIPPER: 8 MB flash + 8 MB PSRAM (everything, plus RGB).").pack(side="left", padx=4)

        tf = ttk.Frame(self)
        tf.pack(fill="x", pady=(2, 0))
        self.btn_check = ttk.Button(tf, text="Check esptool", style=sm, command=self.check)
        self.btn_install = ttk.Button(tf, text="Install esptool", style=sm, command=self.install)
        self.btn_detect = ttk.Button(tf, text="Detect chip", style=sm, command=self.detect)
        self.btn_shipped = ttk.Button(tf, text="Use shipped firmware", style=sm, command=self.use_shipped)
        self.btn_log = ttk.Button(tf, text="USB log", style=sm, command=self.usb_log)
        self.btn_stop = ttk.Button(tf, text="Stop", style=sm, command=lambda: self.app.stop("esp"))
        for b in (self.btn_check, self.btn_install, self.btn_detect, self.btn_shipped, self.btn_log, self.btn_stop):
            b.pack(side="left", **pad)

        ff = ttk.LabelFrame(self, text="Flash (never automatic)")
        ff.pack(fill="x", padx=6, pady=(2, 2))
        ttk.Label(ff, text="Firmware .bin:").grid(row=0, column=0, sticky="w", **pad)
        ttk.Entry(ff, textvariable=self.binfile).grid(row=0, column=1, columnspan=3, sticky="ew", **pad)
        ttk.Button(ff, text="Browse…", style=sm, command=self.browse).grid(row=0, column=4, **pad)
        ttk.Label(ff, text="Port:").grid(row=1, column=0, sticky="w", **pad)
        self.port_box = ttk.Combobox(ff, textvariable=self.port, width=52)
        self.port_box.grid(row=1, column=1, sticky="w", **pad)
        ttk.Button(ff, text="↻", width=3, style=sm, command=self.refresh_ports).grid(row=1, column=2, sticky="w")
        ttk.Label(ff, text="Chip:").grid(row=2, column=0, sticky="w", **pad)
        ttk.Label(ff, text="ESP32-S3 only", style="Badge.TLabel").grid(row=2, column=1, sticky="w", **pad)
        ttk.Label(ff, text="Offset:").grid(row=2, column=2, sticky="e", **pad)
        ttk.Entry(ff, textvariable=self.offset, width=10).grid(row=2, column=3, sticky="w", **pad)
        ttk.Label(ff, text="Baud:").grid(row=3, column=0, sticky="w", **pad)
        ttk.Combobox(ff, textvariable=self.baud, values=[str(b) for b in esp.BAUDS], state="readonly",
                     width=10).grid(row=3, column=1, sticky="w", **pad)
        ttk.Checkbutton(ff, text="Erase the whole flash first", variable=self.erase).grid(
            row=3, column=2, columnspan=2, sticky="w", **pad)
        self.btn_flash = ttk.Button(ff, text="Flash…", style=sm, command=self.flash)
        self.btn_flash.grid(row=1, column=4, rowspan=3, sticky="ns", padx=8, pady=3)
        ff.columnconfigure(3, weight=1)

    # ------------------------------------------------------------ actions
    def refresh_ports(self, quiet=False):
        """Rescan; keep the user's choice if still there, else auto-pick the ESP32-S3's port."""
        ports = esp.list_ports()
        self.port_box["values"] = ports
        cur = self.port.get().strip()
        if cur and cur in ports:
            return cur
        pick = esp.pick_esp_port(ports, esp.windows_espressif_ports() if sys.platform.startswith("win") else ())
        if pick:
            self.port.set(pick)
            if not quiet:
                self.app.info(f"ESP32 port: {pick} (picked automatically; change it in the Port box if wrong)")
        elif cur and cur not in ports and not quiet:
            self.app.info(f"{cur} is gone; pick the ESP32's port again (↻ rescans).")
        elif not quiet and len(ports) > 1:
            self.app.info("Several serial ports and none is clearly the ESP32-S3: pick it in the Port box.")
        self._schedule_rescan()
        return self.port.get().strip()

    def _report_initial_port(self):
        if self.port.get().strip():
            self.app.info(f"ESP32 port: {self.port.get()} (picked automatically; change it in the Port box if wrong)")

    def _schedule_rescan(self):
        """While the box is empty, look again every 2 s, so plugging the ESP32 in fills it."""
        if self._auto_job is None and not self.port.get().strip():
            self._auto_job = self.after(2000, self._auto_rescan)

    def _auto_rescan(self):
        self._auto_job = None
        if not self.port.get().strip() and not self.running:
            self.refresh_ports(quiet=True)
            if self.port.get().strip():
                self.app.info(f"ESP32 port: {self.port.get()} (found when it was plugged in)")
        self._schedule_rescan()

    def browse(self):
        f = filedialog.askopenfilename(title="ESP32 firmware image",
                                       filetypes=[("Firmware", "*.bin"), ("All files", "*")])
        if f:
            self.binfile.set(f)
            kind, ver, want = esp.image_layout(f)   # the right offset, read from the image (2026-10-28)
            if want:
                self.offset.set(want)
                self.app.info(f"{'Full (merged) image' if kind == 'merged' else 'App-only image ' + str(ver)}: "
                              f"offset set to {want}.")

    def use_shipped(self, quiet=False):
        s = esp.shipped_image()
        if not s:
            if not quiet:
                messagebox.showinfo("No shipped firmware", "This copy has no shipped ESP32-S3 image.")
            return
        self.binfile.set(s[0])
        self.offset.set(s[1].get("offset", "0x0"))
        if not quiet:
            self.app.info(f"Using the shipped ESP32-S3 image ({s[1].get('bytes')} bytes, ESP-IDF "
                          f"{s[1].get('idf')}, {s[1].get('status')}); offset {self.offset.get()}.")

    def _buttons(self):
        st = "disabled" if self.running else "normal"
        for b in (self.btn_check, self.btn_install, self.btn_detect, self.btn_shipped, self.btn_log, self.btn_flash,
                  self.btn_auto):
            b.config(state=st)
        self.btn_stop.config(state="normal" if self.running else "disabled")

    def on_done(self, tag):
        self.running = False
        self._buttons()
        if self._after_flash:
            self._after_flash = False
            messagebox.showinfo("ESP32 flashed: now restart it", FLASHED_REMINDER)

    def _run(self, args):
        if self.app.start("esp", [sys.executable, "-u", ESP_SCRIPT] + args):
            self.running = True
            self._buttons()
            return True
        return False

    def check(self):
        self._run(["check"])    # in a subprocess: a hung Python must not freeze the window

    def install(self):
        if not messagebox.askokcancel(
                "Install esptool?",
                "This creates a private Python environment in\n\n  " + esp.venv_dir() + "\n\nand downloads "
                "esptool from PyPI into it (internet needed). Nothing system-wide is changed. On Linux Mint "
                "you may first need:  sudo apt install python3-venv\n\nContinue?", default="cancel"):
            return
        if self.app.start("esp", [sys.executable, "-u", ESP_SCRIPT, "install"]):
            self.running = True
            self._buttons()

    def _need_port(self):
        p = self.port.get().strip() or self.refresh_ports()
        if not p:
            messagebox.showerror("No port", "No port is clearly the ESP32-S3: pick it in the Port box (↻ rescans). "
                                            "Some boards need BOOT held while plugging in.")
        return p

    def usb_log(self):
        """Read-only: restart the chip and stream its USB console here until Stop."""
        p = self._need_port()
        if p:
            self.app.info("USB log: the chip restarts once so its boot messages (and the reason for its last "
                          "restart) are shown. Press Stop to end.")
            self._run(["monitor", "--port", p, "--reset"])

    def detect(self):
        p = self._need_port()
        if p:
            self._run(["detect", "--port", p, "--chip", self.chip.get()])

    def flash(self):
        if self.running:
            return
        p = self._need_port()
        if not p:
            return
        f = self.binfile.get().strip()
        try:
            esp.plan_flash(f, p, self.chip.get(), self.offset.get().strip(), int(self.baud.get()),
                           self.erase.get())
            shipped = esp.verify_image(f)
            if shipped and self.offset.get().strip() != shipped.get("offset", "0x0"):
                raise esp.FwError(f"the shipped image must be flashed at {shipped.get('offset', '0x0')}")
            bad = esp.offset_problem(f, self.offset.get().strip())
            if bad:
                raise esp.FwError(bad)
        except (esp.FwError, ValueError) as e:
            messagebox.showerror("Cannot flash", str(e))
            return
        msg = (f"File:    {f}\nPort:    {p}\nChip:    {self.chip.get()}\nOffset:  {self.offset.get()}\n"
               f"Erase:   {'whole flash first' if self.erase.get() else 'no'}\n\n"
               + (f"Shipped Skipper Flipper image, {shipped.get('status')}, SHA-256 {shipped.get('sha256', '')[:16]}... "
                  "verified. " + (f"Tested on hardware: {(shipped.get('hardware_test') or {}).get('summary', '')}\n\n"
                                  if shipped.get("status") == "hardware-tested" else
                                  "This exact build has not run on hardware yet.\n\n") if shipped else "Your own file.\n\n")
               + "This writes the file to the ESP32. A wrong image or offset can leave the chip not booting "
               "until it is re-flashed. Continue?")
        if not messagebox.askokcancel("Flash ESP32?", msg, icon="warning", default="cancel"):
            self.app.info("flash cancelled")
            return
        a = ["flash", f, "--port", p, "--chip", self.chip.get(), "--offset", self.offset.get().strip(),
             "--baud", self.baud.get(), "--yes"]
        if self.erase.get():
            a.append("--erase")
        self._after_flash = bool(self._run(a))

    def flash_auto(self):
        """One click: detect the chip, pick DECKHAND / CAPTAIN / SKIPPER, flash it (one confirmation)."""
        if self.running:
            return
        p = self._need_port()
        if not p:
            return
        tiers = esp.shipped_tiers()
        if not tiers:
            messagebox.showerror("No shipped firmware", "This copy has no shipped ESP32-S3 images.")
            return
        lines = "\n".join(f"  {info.get('title', t)}: {esp.TIER_NEEDS[t]}" for t, (_p, info) in tiers.items())
        msg = (f"Port: {p}\n\nThis reads the chip (nothing is written), picks the build that fits its memory and "
               f"flashes it at 0x0:\n\n{lines}\n\nSettings (WiFi, pins) and printer.cfg are kept. The log shows "
               "what was found and which build was chosen. Flashing never happens without this confirmation.\n\n"
               "Continue?")
        if not messagebox.askokcancel("Flash for my board?", msg, icon="warning", default="cancel"):
            self.app.info("flash cancelled")
            return
        a = ["flash-auto", "--port", p, "--baud", self.baud.get(), "--yes"]
        if self.erase.get():
            a.append("--erase")
        self._after_flash = bool(self._run(a))
