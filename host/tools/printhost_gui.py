#!/usr/bin/env python3
# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""Skipper Flipper (esp-printhost) tool panel: one window for fake_mcu.py and mcu_probe.

    python3 tools/printhost_gui.py

- Start the fake MCU (no hardware) or pick a real serial port.
- Run mcu_probe (identify, uptime, optional pin blink).
- Everything goes to one output box: select + Ctrl+C, right-click, or
  "Copy all". Optional log file with timestamps.

Needs Tk:  sudo apt install python3-tk   (Mint/Ubuntu)
mcu_probe itself is Linux-only.
"""
import datetime as dt
import glob
import os
import queue
import re
import shlex
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import skipper_theme as theme


__version__ = "0.1.0-wip"
__build_date__ = "2026-10-03 10:00"
WIP_NOTICE = "WORK IN PROGRESS: PROOF OF CONCEPT, MANY FEATURES MAY BE BROKEN"

HERE = os.path.dirname(os.path.abspath(__file__))
HOST_DIR = os.path.dirname(HERE)
BUILD_DIR = os.path.join(HOST_DIR, "build")
PROBE_DEFAULT = os.path.join(BUILD_DIR, "mcu_probe")
FAKE_SCRIPT = os.path.join(HERE, "fake_mcu.py")
LOG_DIR = os.path.expanduser("~/esp-printhost-logs")
ERR_RE = re.compile(r"\b(error|(?<!\b0 )fail(ed|ure)?|denied|no ack|traceback|no such file)\b|exited with code")
BAUDS = ["250000", "115200", "500000", "1000000", "1500000", "2000000", "230400", "57600"]

HINTS = [
    ("tools/build.py --install-deps", "Hint: run that command in a terminal (it asks for your sudo password), then press Build again."),
    ("No such file or directory", "Hint: port not found. Is the board plugged in? Press ↻ to rescan."),
    ("Permission denied", "Hint: add yourself to the dialout group: "
                          "sudo usermod -aG dialout $USER  (then log out/in)"),
    ("Device or resource busy", "Hint: something else has the port open "
                                "(Klipper? Cura? a serial monitor?). Stop it first."),
    ("no identify_response", "Hint: wrong baud, wrong port, board not running "
                             "Klipper firmware, or the board needs a reset."),
    ("already configured", "Hint: power-cycle the board, then run again."),
]


class Proc:
    """A subprocess whose output lines are pushed into a queue with a tag."""

    def __init__(self, tag, argv, outq, cwd=None):
        self.tag, self.argv, self.outq = tag, argv, outq
        env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUTF8="1")
        extra = {"creationflags": subprocess.CREATE_NO_WINDOW} if os.name == "nt" else {}
        # utf-8 + replace: a Windows console codepage must not kill the reader thread
        self.p = subprocess.Popen(argv, cwd=cwd, stdout=subprocess.PIPE, env=env,
                                  stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                  text=True, encoding="utf-8", errors="replace", bufsize=1, **extra)
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        code = -1
        try:
            for line in self.p.stdout:
                self.outq.put((self.tag, line.rstrip("\n")))
            code = self.p.wait()
        except Exception as e:   # noqa: BLE001 - always report "done" so the GUI never stays disabled
            self.outq.put((self.tag, f"output reader failed: {e}"))
        finally:
            self.outq.put((self.tag, None if code == 0 else f"exited with code {code}"))
            self.outq.put(("_done", self.tag))

    def running(self):
        return self.p.poll() is None

    def stop(self):
        if self.running():
            self.p.terminate()
            try:
                self.p.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.p.kill()


class App:
    def __init__(self, root):
        self.root = root
        self.q = queue.Queue()
        self.procs = {}          # tag -> Proc
        self.fake_dev = None
        self.log_fp = None

        root.title(f"Skipper Flipper tools {__version__} (Built {__build_date__})")
        root.geometry("980x780")
        root.minsize(820, 560)
        self.fonts = theme.apply(root)
        theme.set_icon(root)
        self._build_ui()
        self.refresh_ports()
        self.info(f"host dir: {HOST_DIR}")
        if not os.path.exists(PROBE_DEFAULT):
            self.info("mcu_probe not built yet — press 'Build tools'.")
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.after(50, self.poll)

    # ---------------- UI ----------------
    def _build_ui(self):
        pad = dict(padx=6, pady=4)
        theme.FilmStrip(self.root).pack(fill="x")
        self.header = theme.make_header(
            self.root, self.fonts, "TOOLS", [
                "Developer tools: probe a Klipper board over USB, run a simulated one, build and test.",
                "Building and flashing firmware: use flasher.py (Slave and Master tabs).",
            ], badge=WIP_NOTICE)
        self.header.pack(fill="x")
        self.nb = ttk.Notebook(self.root)
        self.nb.pack(fill="x", **pad)
        top = ttk.Frame(self.nb)
        self.nb.add(top, text="Probe")
        # Tools / build
        tf = ttk.LabelFrame(top, text="Tools")
        tf.pack(fill="x", **pad)
        ttk.Label(tf, text="mcu_probe:").grid(row=0, column=0, sticky="w", **pad)
        self.probe_path = tk.StringVar(value=PROBE_DEFAULT)
        ttk.Entry(tf, textvariable=self.probe_path).grid(row=0, column=1, sticky="ew", **pad)
        ttk.Button(tf, text="Browse…", command=self.browse_probe).grid(row=0, column=2, **pad)
        self.build_btn = ttk.Button(tf, text="Build tools", command=self.build)
        self.build_btn.grid(row=0, column=3, **pad)
        self.clean_build = tk.BooleanVar(value=False)
        ttk.Checkbutton(tf, text="clean", variable=self.clean_build).grid(row=0, column=4, **pad)
        tf.columnconfigure(1, weight=1)

        row = ttk.Frame(top)
        row.pack(fill="x")

        # Fake MCU
        ff = ttk.LabelFrame(row, text="Fake MCU (no hardware)")
        ff.pack(side="left", fill="both", **pad)
        ttk.Label(ff, text="start seq:").grid(row=0, column=0, **pad)
        self.fake_seq = tk.StringVar(value="0")
        ttk.Spinbox(ff, from_=0, to=15, width=4, textvariable=self.fake_seq).grid(row=0, column=1, **pad)
        self.fake_btn = ttk.Button(ff, text="Start", command=self.toggle_fake)
        self.fake_btn.grid(row=0, column=2, **pad)
        self.fake_status = ttk.Label(ff, text="stopped", width=16)
        self.fake_status.grid(row=1, column=0, columnspan=3, sticky="w", **pad)

        # Probe
        pf = ttk.LabelFrame(row, text="MCU probe")
        pf.pack(side="left", fill="both", expand=True, **pad)
        ttk.Label(pf, text="Port:").grid(row=0, column=0, sticky="w", **pad)
        self.port = tk.StringVar()
        self.port_box = ttk.Combobox(pf, textvariable=self.port, width=34)
        self.port_box.grid(row=0, column=1, sticky="ew", **pad)
        ttk.Button(pf, text="↻", width=3, command=self.refresh_ports).grid(row=0, column=2, **pad)
        ttk.Label(pf, text="Baud:").grid(row=0, column=3, **pad)
        self.baud = tk.StringVar(value=BAUDS[0])
        ttk.Combobox(pf, textvariable=self.baud, values=BAUDS, width=9).grid(row=0, column=4, **pad)

        self.blink_on = tk.BooleanVar(value=False)
        ttk.Checkbutton(pf, text="Blink pin:", variable=self.blink_on).grid(row=1, column=0, sticky="w", **pad)
        self.blink_pin = tk.StringVar(value="")
        self.pin_box = ttk.Combobox(pf, textvariable=self.blink_pin, width=12)
        self.pin_box.grid(row=1, column=1, sticky="w", **pad)
        ttk.Label(pf, text="(run once to list pins)", style="Muted.TLabel").grid(
            row=2, column=1, columnspan=3, sticky="w", padx=6)
        self.run_btn = ttk.Button(pf, text="Run probe", command=self.run_probe)
        self.run_btn.grid(row=1, column=3, **pad)
        ttk.Button(pf, text="Stop", command=lambda: self.stop("probe")).grid(row=1, column=4, **pad)
        pf.columnconfigure(1, weight=1)

        # Output
        of = ttk.LabelFrame(self.root, text="Output")
        of.pack(fill="both", expand=True, **pad)
        bar = ttk.Frame(of)
        bar.pack(fill="x")
        ttk.Button(bar, style="Small.TButton", text="Copy all", command=self.copy_all).pack(side="left", **pad)
        ttk.Button(bar, style="Small.TButton", text="Copy selection", command=self.copy_sel).pack(side="left", **pad)
        ttk.Button(bar, style="Small.TButton", text="Clear", command=self.clear).pack(side="left", **pad)
        ttk.Button(bar, style="Small.TButton", text="Save as…", command=self.save_as).pack(side="left", **pad)
        self.autoscroll = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text="Auto-scroll", variable=self.autoscroll).pack(side="left", **pad)

        self.log_on = tk.BooleanVar(value=True)
        self.log_path = tk.StringVar(value=self._default_log_path())
        ttk.Checkbutton(bar, text="Log", variable=self.log_on,
                        command=self._log_toggled).pack(side="left", **pad)
        ttk.Button(bar, style="Small.TButton", text="Open log folder", command=self.open_log_dir).pack(side="right", **pad)
        ttk.Button(bar, style="Small.TButton", text="…", width=2, command=self.browse_log).pack(side="right")
        ttk.Entry(bar, textvariable=self.log_path, width=10).pack(side="right", **pad)

        body = ttk.Frame(of)
        body.pack(fill="both", expand=True)
        self.text = tk.Text(body, wrap="none", undo=False)
        ys = ttk.Scrollbar(body, orient="vertical", command=self.text.yview)
        xs = ttk.Scrollbar(of, orient="horizontal", command=self.text.xview)
        self.text.configure(yscrollcommand=ys.set, xscrollcommand=xs.set)
        self.text.pack(side="left", fill="both", expand=True)
        ys.pack(side="right", fill="y")
        xs.pack(side="bottom", fill="x", before=body)
        theme.style_text(self.text, self.fonts)
        self._make_readonly_selectable(self.text)

        strip = theme.FilmStrip(self.root)
        strip.pack(side="bottom", fill="x", before=of)
        sb = ttk.Frame(self.root)
        sb.pack(side="bottom", fill="x", padx=8, pady=(2, 4), before=strip)
        self.status = ttk.Label(sb, text="", anchor="w", style="Status.TLabel")
        self.status.pack(side="left", fill="x", expand=True)
        ttk.Label(sb, text=f"v{__version__} \u2022 {__build_date__} \u2022 WIP / proof of concept",
                  style="Version.TLabel").pack(side="right")

    def _make_readonly_selectable(self, t):
        """Block edits but keep selection, copy and navigation working."""
        allowed = {"Left", "Right", "Up", "Down", "Home", "End", "Prior", "Next",
                   "Shift_L", "Shift_R", "Control_L", "Control_R"}

        def on_key(e):
            ctrl = e.state & 0x4
            if ctrl and e.keysym.lower() in ("c", "a", "insert"):
                if e.keysym.lower() == "a":
                    t.tag_add("sel", "1.0", "end-1c")
                    return "break"
                return None
            return None if e.keysym in allowed else "break"
        t.bind("<Key>", on_key)
        for seq in ("<<Paste>>", "<<Cut>>", "<<Clear>>", "<Button-2>"):
            t.bind(seq, lambda e: "break")

        menu = tk.Menu(t, tearoff=0)
        menu.add_command(label="Copy", command=self.copy_sel)
        menu.add_command(label="Select all", command=lambda: t.tag_add("sel", "1.0", "end-1c"))
        menu.add_command(label="Copy all", command=self.copy_all)
        t.bind("<Button-3>", lambda e: menu.tk_popup(e.x_root, e.y_root))

    # ---------------- output / logging ----------------
    def _default_log_path(self):
        stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        return os.path.join(LOG_DIR, f"session-{stamp}.log")

    def _log_toggled(self):
        if not self.log_on.get() and self.log_fp:
            self.log_fp.close()
            self.log_fp = None

    def _log_write(self, line):
        if not self.log_on.get():
            return
        path = os.path.expanduser(self.log_path.get().strip())
        if self.log_fp and self.log_fp.name != path:
            self.log_fp.close()
            self.log_fp = None
        if not self.log_fp:
            try:
                os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
                self.log_fp = open(path, "a", buffering=1, encoding="utf-8")
            except OSError as e:
                self.log_on.set(False)
                self._emit("err", f"log disabled: {e}")
                return
        self.log_fp.write(line + "\n")

    def _emit(self, tag, msg):
        ts = dt.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        low = msg.lower()
        style = "err" if ERR_RE.search(low) else tag
        self.text.insert("end", ts + " ", "ts")
        self.text.insert("end", f"[{tag:5}] ", tag)
        self.text.insert("end", msg + "\n", style)
        self._log_write(f"{dt.datetime.now().isoformat(timespec='milliseconds')} [{tag}] {msg}")
        for needle, hint in HINTS:
            if needle.lower() in low:
                self.text.insert("end", " " * 13 + hint + "\n", "hint")
                self._log_write(" " * 13 + hint)
        if self.autoscroll.get():
            self.text.see("end")

    def info(self, msg):
        self._emit("app", msg)

    def copy_all(self):
        self._clip(self.text.get("1.0", "end-1c"))

    def copy_sel(self):
        try:
            self._clip(self.text.get("sel.first", "sel.last"))
        except tk.TclError:
            self.status.config(text="Nothing selected.")

    def _clip(self, s):
        self.root.clipboard_clear()
        self.root.clipboard_append(s)
        self.root.update()   # keep clipboard after window loses focus (X11)
        self.status.config(text=f"Copied {len(s.splitlines())} lines to clipboard.")

    def clear(self):
        self.text.delete("1.0", "end")

    def save_as(self):
        path = filedialog.asksaveasfilename(defaultextension=".log",
                                            initialdir=LOG_DIR if os.path.isdir(LOG_DIR) else HOST_DIR,
                                            filetypes=[("Log", "*.log"), ("Text", "*.txt"), ("All", "*")])
        if path:
            with open(path, "w", encoding="utf-8") as f:
                f.write(self.text.get("1.0", "end-1c"))
            self.status.config(text=f"Saved {path}")

    def browse_log(self):
        path = filedialog.asksaveasfilename(defaultextension=".log", initialfile=os.path.basename(self.log_path.get()))
        if path:
            self.log_path.set(path)

    def open_log_dir(self):
        d = os.path.dirname(os.path.expanduser(self.log_path.get())) or LOG_DIR
        os.makedirs(d, exist_ok=True)
        opener = "open" if sys.platform == "darwin" else "xdg-open"
        try:
            subprocess.Popen([opener, d])
        except OSError as e:
            self.info(f"couldn't open folder: {e}")

    # ---------------- processes ----------------
    def start(self, tag, argv, cwd=None):
        if tag in self.procs and self.procs[tag].running():
            self.info(f"{tag} already running")
            return False
        self._emit(tag, "$ " + " ".join(shlex.quote(a) for a in argv))
        try:
            self.procs[tag] = Proc(tag, argv, self.q, cwd)
        except OSError as e:
            self._emit(tag, f"failed to start: {e}")
            return False
        self._update_buttons()
        return True

    def stop(self, tag):
        p = self.procs.get(tag)
        if p and p.running():
            p.stop()
            self._emit(tag, "stopped")

    def poll(self):
        try:
            while True:
                tag, line = self.q.get_nowait()
                if tag == "_done":
                    self._on_done(line)
                elif line is None:
                    self._emit(tag, "finished OK")
                else:
                    if tag == "probe":
                        self._pin_lines(line)
                    if tag == "fake" and self.fake_dev is None and line.startswith("/dev/"):
                        self.fake_dev = line.strip()
                        self.fake_status.config(text=f"on {self.fake_dev}")
                        self.refresh_ports()
                        self.port.set(self.fake_dev)
                        self.baud.set("250000")
                    self._emit(tag, line)
        except queue.Empty:
            pass
        self.root.after(50, self.poll)

    def _on_done(self, tag):
        if tag == "fake":
            self.fake_dev = None
            self.fake_status.config(text="stopped")
            self.refresh_ports()
        if tag == "build" and os.path.exists(PROBE_DEFAULT):
            self.probe_path.set(PROBE_DEFAULT)
        self._update_buttons()

    def _update_buttons(self):
        fake = self.procs.get("fake")
        self.fake_btn.config(text="Stop" if fake and fake.running() else "Start")
        probe = self.procs.get("probe")
        self.run_btn.config(state="disabled" if probe and probe.running() else "normal")
        b = self.procs.get("build")
        self.build_btn.config(state="disabled" if b and b.running() else "normal")

    # ---------------- actions ----------------
    def toggle_fake(self):
        p = self.procs.get("fake")
        if p and p.running():
            self.stop("fake")
            return
        self.fake_dev = None
        self.fake_status.config(text="starting…")
        self.start("fake", [sys.executable, "-u", FAKE_SCRIPT, self.fake_seq.get() or "0"])

    def _pin_lines(self, line):
        """Collect the '--pins' block from probe output into the pin dropdown."""
        if line.startswith("all ") and line.endswith("pin names:"):
            self.collect_pins = []
            return
        if getattr(self, "collect_pins", None) is not None:
            if line.startswith("  "):
                self.collect_pins += line.split()
                return
            pins, self.collect_pins = self.collect_pins, None
            if pins:
                self.pin_box["values"] = pins
                if self.blink_pin.get() not in pins:
                    self.blink_pin.set("")
                self.status.config(text=f"{len(pins)} pins loaded into the Blink pin list.")

    def run_probe(self):
        exe = os.path.expanduser(self.probe_path.get().strip())
        if not os.path.exists(exe):
            messagebox.showerror("mcu_probe missing", f"{exe}\n\nPress 'Build tools' first.")
            return
        port, baud = self.port.get().strip(), self.baud.get().strip()
        if not port:
            messagebox.showerror("No port", "Pick a serial port or start the fake MCU.")
            return
        if not baud.isdigit():
            messagebox.showerror("Bad baud", f"'{baud}' is not a number.")
            return
        argv = [exe, port, baud, "--pins"]
        self.collect_pins = None
        if self.blink_on.get() and self.blink_pin.get().strip():
            argv += ["--blink", self.blink_pin.get().strip()]
        self.start("probe", argv)

    def build(self):
        argv = [sys.executable, "-u", os.path.join(HERE, "build.py")]
        if self.clean_build.get():
            argv.append("--clean")
        self.start("build", argv)

    def browse_probe(self):
        path = filedialog.askopenfilename(initialdir=BUILD_DIR if os.path.isdir(BUILD_DIR) else HOST_DIR)
        if path:
            self.probe_path.set(path)

    def refresh_ports(self):
        ports = sorted(set(glob.glob("/dev/serial/by-id/*") + glob.glob("/dev/ttyUSB*")
                           + glob.glob("/dev/ttyACM*") + glob.glob("/dev/tty.usb*")
                           + glob.glob("/dev/cu.usb*")))
        if self.fake_dev:
            ports.insert(0, self.fake_dev)
        self.port_box["values"] = ports
        if not self.port.get() and ports:
            self.port.set(ports[0])

    def on_close(self):
        for p in self.procs.values():
            p.stop()
        if self.log_fp:
            self.log_fp.close()
        self.root.destroy()


def main():
    if "--version" in sys.argv[1:]:
        print(f"Skipper Flipper tools {__version__} (built {__build_date__}) - {WIP_NOTICE}")
        return
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
