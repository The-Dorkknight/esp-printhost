#!/usr/bin/env python3
# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""Unit tests for skipper_esp.py. No hardware, no network, subprocess mocked.

    python3 host/tools/test_skipper_esp.py
"""
import contextlib
import io
import os
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import skipper_esp as esp
from skipper_fw import FwError


class FakeWinreg:
    HKEY_LOCAL_MACHINE = 1

    def __init__(self, values=None, fail=False):
        self.values, self.fail, self.opened = values or [], fail, None

    def OpenKey(self, root, path):
        if self.fail:
            raise OSError("nokey")
        self.opened = (root, path)
        return object()

    def EnumValue(self, key, i):
        if i >= len(self.values):
            raise OSError("done")
        n, v = self.values[i]
        return n, v, 1

    def CloseKey(self, key):
        pass


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.bin = os.path.join(self.tmp.name, "fw.bin")
        with open(self.bin, "wb") as f:
            f.write(b"\x01" * 100)

    def quiet(self, fn, *a, **k):
        with contextlib.redirect_stdout(io.StringIO()) as o:
            r = fn(*a, **k)
        return r, o.getvalue()


class TestAvail(unittest.TestCase):
    def test_ok(self):
        run = mock.Mock(return_value=mock.Mock(returncode=0, stdout="esptool.py v4\n4.7.0\n"))
        ok, v = esp.esptool_available("py", run=run)
        self.assertTrue(ok)
        self.assertEqual(v, "4.7.0")
        self.assertEqual(run.call_args[0][0], ["py", "-m", "esptool", "version"])

    def test_nonzero(self):
        run = mock.Mock(return_value=mock.Mock(returncode=1, stdout="No module named esptool\n"))
        ok, v = esp.esptool_available("py", run=run)
        self.assertFalse(ok)
        self.assertIn("No module", v)

    def test_oserror(self):
        ok, v = esp.esptool_available("py", run=mock.Mock(side_effect=OSError("boom")))
        self.assertFalse(ok)
        self.assertIn("boom", v)

    def test_timeout(self):
        ok, _ = esp.esptool_available("py", run=mock.Mock(side_effect=subprocess.TimeoutExpired("x", 1)))
        self.assertFalse(ok)

    def test_hint(self):
        self.assertIn("Install esptool", esp.install_hint())
        # --user only outside a virtualenv (pip refuses it inside one); check both explicitly so
        # the result does not depend on which Python runs the tests
        self.assertEqual(esp.install_argv("p", in_venv=False), ["p", "-m", "pip", "install", "--user", "esptool"])
        self.assertEqual(esp.install_argv("p", in_venv=True), ["p", "-m", "pip", "install", "esptool"])

    def test_install_creates_venv_then_pip(self):
        with tempfile.TemporaryDirectory() as d:
            vdir = os.path.join(d, "v")
            vpy = esp.venv_python(vdir)
            calls = []

            def runner(argv):
                calls.append(argv)
                if "venv" in argv:
                    os.makedirs(os.path.dirname(vpy))
                    open(vpy, "w").close()
            got = esp.install_esptool(vdir, "basepy", runner, available=lambda py: (py == vpy and len(calls) == 2, "v5"))
            self.assertEqual(got, vpy)
            self.assertEqual(calls[0], ["basepy", "-m", "venv", vdir])
            self.assertEqual(calls[1][:4], [vpy, "-m", "pip", "install"])
            self.assertEqual(calls[1][-1], "esptool")

    def test_install_noop_when_present(self):
        with tempfile.TemporaryDirectory() as d:
            vpy = esp.venv_python(d)
            os.makedirs(os.path.dirname(vpy))
            open(vpy, "w").close()
            calls = []
            esp.install_esptool(d, "b", lambda a: calls.append(a), available=lambda py: (True, "5"))
            self.assertEqual(calls, [])

    def test_install_venv_failure_mentions_apt(self):
        def runner(argv):
            raise FwError("ensurepip is not available")
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(FwError) as cm:
                esp.install_esptool(os.path.join(d, "v"), "b", runner)
        self.assertIn("python3-venv", str(cm.exception))

    def test_install_pip_failure_mentions_network(self):
        def runner(argv):
            if "pip" in argv:
                raise FwError("boom")
        with tempfile.TemporaryDirectory() as d:
            vdir = os.path.join(d, "v")
            vpy = esp.venv_python(vdir)
            os.makedirs(os.path.dirname(vpy))
            open(vpy, "w").close()
            with self.assertRaises(FwError) as cm:
                esp.install_esptool(vdir, "b", runner, available=lambda py: (False, "x"))
        self.assertIn("internet", str(cm.exception))

    def test_default_python_prefers_private_venv(self):
        with tempfile.TemporaryDirectory() as home, mock.patch.dict(os.environ, {"SKIPPER_FLIPPER_HOME": home}):
            self.assertEqual(esp._py(None), sys.executable)
            vpy = esp.venv_python()
            os.makedirs(os.path.dirname(vpy))
            open(vpy, "w").close()
            self.assertEqual(esp._py(None), vpy)
            self.assertEqual(esp._py("explicit"), "explicit")


class TestPorts(unittest.TestCase):
    def test_linux(self):
        table = {"/dev/ttyUSB*": ["/dev/ttyUSB1", "/dev/ttyUSB0"], "/dev/ttyACM*": ["/dev/ttyACM0"],
                 "/dev/serial/by-id/*": ["/dev/serial/by-id/x"]}
        r = esp.list_ports("linux", glob_fn=lambda p: table.get(p, []))
        self.assertEqual(r, sorted(["/dev/ttyUSB1", "/dev/ttyUSB0", "/dev/ttyACM0", "/dev/serial/by-id/x"]))

    def test_mac(self):
        seen = []

        def g(p):
            seen.append(p)
            return ["/dev/cu.usbmodem1"] if "usbmodem" in p else []
        self.assertEqual(esp.list_ports("darwin", glob_fn=g), ["/dev/cu.usbmodem1"])
        self.assertEqual(len(seen), 4)
        self.assertIn("/dev/cu.SLAB*", seen)
        self.assertIn("/dev/cu.wchusbserial*", seen)

    def test_windows_fake_winreg(self):
        w = FakeWinreg([("\\Device\\Serial0", "COM3"), ("x", "COM10"), ("y", "")])
        self.assertEqual(esp.list_ports("win32", winreg=w), ["COM10", "COM3"])
        self.assertEqual(w.opened, (1, r"HARDWARE\DEVICEMAP\SERIALCOMM"))

    def test_windows_missing_key(self):
        self.assertEqual(esp.list_ports("win32", winreg=FakeWinreg(fail=True)), [])

    def test_windows_no_winreg_on_linux(self):
        self.assertEqual(esp.list_ports("win32"), [])

    def test_unknown_platform(self):
        self.assertEqual(esp.list_ports("freebsd"), [])


class TestPlan(Base):
    def test_argv_exact(self):
        s = esp.plan_flash(self.bin, "/dev/ttyUSB0", python="py")
        self.assertEqual(len(s), 1)
        self.assertEqual(s[0].argv, ["py", "-m", "esptool", "--chip", "esp32s3", "--port", "/dev/ttyUSB0",
                                     "--baud", "460800", "write-flash", "0x0", self.bin])

    def test_only_esp32s3(self):
        for chip in ("auto", "esp32", "esp32s2", "esp32c3"):
            with self.assertRaises(FwError):
                esp.plan_flash(self.bin, "COM3", chip=chip, python="py")
            with self.assertRaises(FwError):
                esp.chip_info_argv("COM3", chip, "py")

    def test_erase_first(self):
        s = esp.plan_flash(self.bin, "P", erase=True, python="py")
        self.assertEqual(len(s), 2)
        self.assertEqual(s[0].argv[-1], "erase-flash")
        self.assertEqual(s[1].argv[-3], "write-flash")

    def test_old_esptool_names(self):
        s = esp.plan_flash(self.bin, "P", erase=True, python="py", hyphen=False)
        self.assertEqual(s[0].argv[-1], "erase_flash")
        self.assertEqual(s[1].argv[-3], "write_flash")

    def test_esptool_major(self):
        self.assertEqual(esp.esptool_major("esptool v5.5.0"), 5)
        self.assertEqual(esp.esptool_major("esptool.py v4.8.1"), 4)
        self.assertEqual(esp.esptool_major("esptool.py v4.9.dev3"), 4)
        self.assertIsNone(esp.esptool_major("nothing"))

    def test_default_python(self):
        self.assertEqual(esp.plan_flash(self.bin, "P")[0].argv[0], sys.executable)

    def test_chip_info(self):
        self.assertEqual(esp.chip_info_argv("P", python="py"),
                         ["py", "-m", "esptool", "--chip", "esp32s3", "--port", "P", "flash-id"])
        self.assertEqual(esp.chip_info_argv("P", python="py", hyphen=False)[-1], "flash_id")
        with self.assertRaises(FwError):
            esp.chip_info_argv("", python="py")

    def test_defaults(self):
        self.assertEqual(esp.DEFAULT_CHIP, "esp32s3")
        self.assertEqual(esp.CHIPS, ("esp32s3",))

    def test_refusals(self):
        bad = [dict(binfile=os.path.join(self.tmp.name, "nope.bin"), port="P"),
               dict(binfile=self.bin, port=""),
               dict(binfile=self.bin, port="  "),
               dict(binfile=self.bin, port="P", chip="esp8266"),
               dict(binfile=self.bin, port="P", offset="10000"),
               dict(binfile=self.bin, port="P", offset="0xZZ"),
               dict(binfile=self.bin, port="P", offset="0x"),
               dict(binfile=self.bin, port="P", baud=12345),
               dict(binfile=self.bin, port="P", baud="460800"),
               dict(binfile=self.tmp.name, port="P")]
        for kw in bad:
            with self.subTest(kw=kw), self.assertRaises(FwError):
                esp.plan_flash(**kw)

    def test_empty_file(self):
        e = os.path.join(self.tmp.name, "e.bin")
        open(e, "wb").close()
        with self.assertRaises(FwError):
            esp.plan_flash(e, "P")

    def test_too_big(self):
        with mock.patch("os.path.getsize", return_value=esp.MAX_IMAGE + 1):
            with self.assertRaises(FwError):
                esp.plan_flash(self.bin, "P")
        with mock.patch("os.path.getsize", return_value=esp.MAX_IMAGE):
            esp.plan_flash(self.bin, "P")


class TestFlash(Base):
    def ok_avail(self):
        return mock.patch.object(esp, "esptool_available", return_value=(True, "4.7"))

    def test_dry_run_runs_nothing(self):
        runner = mock.Mock()
        with mock.patch.object(esp, "esptool_available") as av:
            r, out = self.quiet(esp.flash, self.bin, "P", dry_run=True, runner=runner, python="py")
        self.assertEqual(r, 0)
        runner.assert_not_called()
        av.assert_not_called()
        self.assertIn("DRY RUN", out)
        self.assertIn("write-flash", out)
        self.assertIn("your own file", out)

    def test_refused_without_tty(self):
        runner = mock.Mock()
        with self.ok_avail(), self.assertRaises(FwError):
            self.quiet(esp.flash, self.bin, "P", isatty=False, runner=runner)
        runner.assert_not_called()

    def test_wrong_typed_confirmation(self):
        runner = mock.Mock()
        with self.ok_avail(), self.assertRaises(FwError):
            self.quiet(esp.flash, self.bin, "P", isatty=True, input_fn=lambda _: "nope", runner=runner)
        runner.assert_not_called()

    def test_eof_confirmation(self):
        def eof(_):
            raise EOFError
        self.assertFalse(self.quiet(esp.confirm_flash, "P", False, eof, True)[0])

    def test_typed_confirmation_ok(self):
        runner = mock.Mock()
        with self.ok_avail():
            r, out = self.quiet(esp.flash, self.bin, "P", isatty=True, input_fn=lambda _: "P",
                                runner=runner, python="py")
        self.assertEqual(r, 0)
        self.assertEqual(runner.call_count, 1)
        self.assertIn("FLASH STEPS DONE", out)

    def test_yes_with_erase_runs_both(self):
        runner = mock.Mock()
        with self.ok_avail():
            self.quiet(esp.flash, self.bin, "P", erase=True, yes=True, runner=runner, python="py")
        self.assertEqual(runner.call_count, 2)
        self.assertEqual(runner.call_args_list[0][0][0][-1], "erase_flash")

    def test_missing_esptool(self):
        runner = mock.Mock()
        with mock.patch.object(esp, "esptool_available", return_value=(False, "gone")):
            with self.assertRaises(FwError) as c:
                self.quiet(esp.flash, self.bin, "P", yes=True, runner=runner)
        self.assertIn("Install esptool", str(c.exception))
        runner.assert_not_called()

    def test_invalid_input_before_anything(self):
        with self.assertRaises(FwError):
            self.quiet(esp.flash, self.bin, "P", offset="bad", yes=True, runner=mock.Mock())


class FakeUsbReg:
    """Just enough winreg for windows_espressif_ports: Enum\\USB\\<dev>\\<inst>\\Device Parameters."""
    HKEY_LOCAL_MACHINE = 1

    def __init__(self, tree):
        self.tree = tree   # {dev: {inst: portname}}

    def OpenKey(self, parent, sub):
        if parent == 1:
            if sub != r"SYSTEM\CurrentControlSet\Enum\USB":
                raise OSError
            return ("usb",)
        if parent == ("usb",):
            if sub not in self.tree:
                raise OSError
            return ("dev", sub)
        dev = parent[1]
        inst = sub.split("\\")[0]
        if inst not in self.tree[dev]:
            raise OSError
        return ("params", dev, inst)

    def EnumKey(self, key, i):
        names = list(self.tree) if key == ("usb",) else list(self.tree[key[1]])
        if i >= len(names):
            raise OSError
        return names[i]

    def QueryValueEx(self, key, name):
        return self.tree[key[1]][key[2]], 1

    def CloseKey(self, key):
        pass


class TestPickPort(unittest.TestCase):
    ESP = "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_DC:DA:0C:11:22:33-if00"
    PICO = "/dev/serial/by-id/usb-Klipper_rp2040_E66138935F7B3A2B-if00"
    LINKS = {ESP: "/dev/ttyACM1", PICO: "/dev/ttyACM0"}

    def pick(self, ports, espressif=()):
        return esp.pick_esp_port(ports, espressif, realpath=lambda p: self.LINKS.get(p, p))

    def test_espressif_by_id_wins_over_klipper_board(self):
        ports = sorted(["/dev/ttyACM0", "/dev/ttyACM1", self.ESP, self.PICO])
        self.assertEqual(self.pick(ports), self.ESP)

    def test_vid_in_by_id_name(self):
        p = "/dev/serial/by-id/usb-303a_1001_ABC-if00"
        self.assertEqual(self.pick(["/dev/ttyACM3", "/dev/ttyUSB0", p]), p)

    def test_single_port(self):
        self.assertEqual(self.pick(["/dev/ttyUSB0"]), "/dev/ttyUSB0")
        self.assertEqual(self.pick(["COM5"]), "COM5")

    def test_single_device_prefers_stable_by_id_name(self):
        other = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
        links = {other: "/dev/ttyUSB0"}
        r = esp.pick_esp_port(["/dev/ttyUSB0", other], realpath=lambda p: links.get(p, p))
        self.assertEqual(r, other)

    def test_only_a_klipper_board_is_not_picked(self):
        self.assertEqual(self.pick(["/dev/ttyACM0", self.PICO]), "")

    def test_klipper_board_plus_one_unknown(self):
        self.assertEqual(self.pick(["/dev/ttyACM0", "/dev/ttyUSB0", self.PICO]), "/dev/ttyUSB0")

    def test_ambiguous_picks_nothing(self):
        self.assertEqual(self.pick(["/dev/ttyUSB0", "/dev/ttyUSB1"]), "")
        self.assertEqual(self.pick(["COM3", "COM4"]), "")
        self.assertEqual(self.pick([]), "")

    def test_two_espressif_devices_picks_nothing(self):
        e2 = "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_AA-if00"
        self.assertEqual(self.pick([self.ESP, e2]), "")

    def test_windows_registry_list(self):
        self.assertEqual(self.pick(["COM3", "COM7"], espressif=["COM7"]), "COM7")
        self.assertEqual(self.pick(["COM3", "COM7"], espressif=["COM9"]), "")

    def test_windows_espressif_ports(self):
        reg = FakeUsbReg({"VID_303A&PID_1001": {"9&1": "COM7"},
                          "VID_303A&PID_1001&MI_00": {"x": "COM8"},
                          "VID_2E8A&PID_000A": {"y": "COM3"}})
        self.assertEqual(esp.windows_espressif_ports(reg), ["COM7", "COM8"])

    def test_windows_espressif_ports_no_registry(self):
        self.assertEqual(esp.windows_espressif_ports(FakeWinreg(fail=True)), [])


class TestMasterPanelPorts(unittest.TestCase):
    """The Flasher's Master tab fills the port box itself (needs a display; skipped without one)."""

    def setUp(self):
        try:
            import tkinter as tk
            self.root = tk.Tk()
        except Exception as e:   # no display
            self.skipTest(f"no Tk display: {e}")
        self.root.withdraw()
        import master_panel
        self.mp = master_panel
        self.msgs = []
        app = mock.Mock()
        app.info = self.msgs.append
        app.start = mock.Mock(return_value=True)
        self.app = app

    def tearDown(self):
        self.root.destroy()

    def panel(self, ports):
        self.ports = ports
        with mock.patch.object(esp, "list_ports", side_effect=lambda: list(self.ports)):
            p = self.mp.MasterPanel(self.root, self.app)
        return p

    def test_auto_selects_on_open(self):
        p = self.panel(["/dev/ttyACM0"])
        self.assertEqual(p.port.get(), "/dev/ttyACM0")
        self.assertEqual(self.msgs, [])          # nothing logged while the window is being built
        self.root.update()
        self.assertTrue(any("picked automatically" in m for m in self.msgs))

    def test_real_flasher_window_opens_with_a_port_found(self):
        import skipper_flasher
        with mock.patch.object(esp, "list_ports", return_value=["/dev/ttyACM0"]):
            app = skipper_flasher.FlasherApp(self.root)
            self.root.update()
        self.assertEqual(app.master.port.get(), "/dev/ttyACM0")
        self.assertIn("/dev/ttyACM0", app.text.get("1.0", "end"))

    def test_ambiguous_leaves_box_empty_and_detect_asks(self):
        p = self.panel(["/dev/ttyUSB0", "/dev/ttyUSB1"])
        self.assertEqual(p.port.get(), "")
        with mock.patch.object(esp, "list_ports", return_value=["/dev/ttyUSB0", "/dev/ttyUSB1"]), \
                mock.patch.object(self.mp.messagebox, "showerror") as err:
            p.detect()
        err.assert_called_once()
        self.app.start.assert_not_called()

    def test_detect_fills_empty_box_then_runs(self):
        p = self.panel([])
        self.assertEqual(p.port.get(), "")
        with mock.patch.object(esp, "list_ports", return_value=["/dev/ttyACM0"]):
            p.detect()
        self.assertEqual(p.port.get(), "/dev/ttyACM0")
        argv = self.app.start.call_args[0][1]
        self.assertEqual(argv[argv.index("--port") + 1], "/dev/ttyACM0")

    def test_plugging_in_later_fills_box(self):
        p = self.panel([])
        self.assertIsNotNone(p._auto_job)
        with mock.patch.object(esp, "list_ports", return_value=["/dev/ttyACM0"]):
            p._auto_rescan()
        self.assertEqual(p.port.get(), "/dev/ttyACM0")
        self.assertTrue(any("plugged in" in m for m in self.msgs))

    def test_reminder_after_flash_only(self):
        p = self.panel(["/dev/ttyACM0"])
        with mock.patch.object(self.mp.messagebox, "showinfo") as info:
            p.on_done("esp")                       # e.g. Detect chip ended: no popup
            info.assert_not_called()
            self.app.start.return_value = True
            p._after_flash = bool(p._run(["flash", "x"]))
            p.on_done("esp")
            info.assert_called_once()
            self.assertIn("BOOT", info.call_args[0][1])
            self.assertIn("Unplug", info.call_args[0][1])
            p.on_done("esp")                       # shown once per flash
            info.assert_called_once()

    def test_no_reminder_when_flash_did_not_start(self):
        p = self.panel(["/dev/ttyACM0"])
        self.app.start.return_value = False
        with mock.patch.object(self.mp.messagebox, "showinfo") as info:
            p._after_flash = bool(p._run(["flash", "x"]))
            p.on_done("esp")
            info.assert_not_called()

    def test_slave_panel_reminds_after_uf2_flash_only(self):
        import fw_panel
        with mock.patch.object(fw_panel.messagebox, "showinfo") as info:
            fp = fw_panel.FirmwarePanel.__new__(fw_panel.FirmwarePanel)
            fp.running, fp.refresh_latest = True, lambda: None
            for method, expect in (("uf2-copy", True), ("avrdude", False), (None, False)):
                info.reset_mock()
                fp._after_flash = method
                fp.on_done("fw")
                self.assertEqual(info.called, expect, method)
                if expect:
                    self.assertIn("BOOT jumper", info.call_args[0][1])
                self.assertIsNone(fp._after_flash)

    def test_user_choice_is_kept(self):
        p = self.panel(["/dev/ttyUSB0", "/dev/ttyUSB1"])
        p.port.set("/dev/ttyUSB1")
        with mock.patch.object(esp, "list_ports", return_value=["/dev/ttyUSB0", "/dev/ttyUSB1"]):
            p.refresh_ports()
        self.assertEqual(p.port.get(), "/dev/ttyUSB1")


class TestCLI(Base):
    def test_flash_dry_run(self):
        r, out = self.quiet(esp.main, ["flash", self.bin, "--port", "P", "--dry-run"])
        self.assertEqual(r, 0)
        self.assertIn("DRY RUN", out)

    def test_flash_bad_offset_exit2(self):
        r, out = self.quiet(esp.main, ["flash", self.bin, "--port", "P", "--offset", "zz", "--dry-run"])
        self.assertEqual(r, 2)
        self.assertIn("error:", out)

    def test_ports(self):
        with mock.patch.object(esp, "list_ports", return_value=["/dev/ttyUSB0"]):
            r, out = self.quiet(esp.main, ["ports"])
        self.assertEqual((r, out.strip()), (0, "/dev/ttyUSB0"))

    def test_check_missing(self):
        with mock.patch.object(esp, "esptool_available", return_value=(False, "x")):
            r, out = self.quiet(esp.main, ["check"])
        self.assertEqual(r, 1)
        self.assertIn("Install esptool", out)

    def test_detect_runs_flash_id(self):
        for ver, cmd in (("esptool.py v4.8.1", "flash_id"), ("esptool v5.5.0", "flash-id")):
            with mock.patch.object(esp, "esptool_available", return_value=(True, ver)), \
                    mock.patch.object(esp, "run_stream") as rs:
                r, _ = self.quiet(esp.main, ["detect", "--port", "P"])
            self.assertEqual(r, 0)
            self.assertEqual(rs.call_args[0][0][-1], cmd)

    def test_help_epilog(self):
        with self.assertRaises(SystemExit), contextlib.redirect_stdout(io.StringIO()) as o:
            esp.main(["--help"])
        self.assertIn("compile-only", o.getvalue())


class ImageLayout(unittest.TestCase):
    """2026-10-28: an app-only CAPTAIN image flashed over USB at 0x10000 left a board not booting. The
    Flasher now reads what the image is and refuses a wrong offset."""

    def _img(self, merged=False, version=b"0.1.0-wip-captain"):
        d = bytearray(b"\xff" * (0x9000 if merged else 0x200))
        d[0] = 0xE9
        if merged:
            d[0x8000:0x8002] = b"\xaa\x50"
        else:
            d[0x20:0x24] = b"\x32\x54\xcd\xab"
            d[0x30:0x30 + len(version) + 1] = version + b"\0"
        f = tempfile.NamedTemporaryFile(suffix=".bin", delete=False)
        f.write(d)
        f.close()
        self.addCleanup(os.unlink, f.name)
        return f.name

    def test_layouts(self):
        self.assertEqual(esp.image_layout(self._img(merged=True)), ("merged", None, "0x0"))
        self.assertEqual(esp.image_layout(self._img()), ("app", "0.1.0-wip-captain", "0x20000"))
        self.assertEqual(esp.image_layout(self._img(version=b"0.1.0-wip-skipper"))[2], "0x20000")
        self.assertEqual(esp.image_layout(self._img(version=b"0.1.0-wip-deckhand"))[2], "0x10000")
        self.assertEqual(esp.image_layout(self._img(version=b"someone-else-1.0"))[2], None)

    def test_wrong_offset_refused(self):
        app = self._img()
        self.assertIn("0x20000", esp.offset_problem(app, "0x10000"))
        self.assertIsNone(esp.offset_problem(app, "0x20000"))
        self.assertIn("0x0", esp.offset_problem(self._img(merged=True), "0x20000"))
        with self.assertRaises(FwError):
            esp.flash(app, "/dev/ttyUSB0", offset="0x10000", dry_run=True)
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            esp.flash(app, "/dev/ttyUSB0", offset="0x20000", dry_run=True)
        self.assertIn("app-only image", out.getvalue())


if __name__ == "__main__":
    unittest.main()
