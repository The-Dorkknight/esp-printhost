# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""Flash-safety tests for skipper_fw / skipper_esp (no hardware: temp folders and fakes only)."""
import contextlib
import io
import json
import os
import tempfile
import time
import unittest

import skipper_esp as esp
import skipper_fw as fw


def capture(fn, *a, **k):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        try:
            r = fn(*a, **k)
        except fw.FwError as e:
            r = e
    return r, buf.getvalue()


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.out = os.path.join(self.tmp.name, "out")
        self.old_home = os.environ.get("SKIPPER_FLIPPER_HOME")
        os.environ["SKIPPER_FLIPPER_HOME"] = os.path.join(self.tmp.name, "home")

    def tearDown(self):
        if self.old_home is None:
            os.environ.pop("SKIPPER_FLIPPER_HOME", None)
        else:
            os.environ["SKIPPER_FLIPPER_HOME"] = self.old_home
        self.tmp.cleanup()

    def make_build(self, board, primary, data=b"FW", info_board=None, sha=True):
        d = os.path.join(self.out, f"{board}-20260101-1200")
        os.makedirs(d)
        path = os.path.join(d, primary)
        with open(path, "wb") as f:
            f.write(data)
        info = {"board": info_board or board, "primary_artifact": primary}
        if sha:
            info["artifacts"] = {primary: {"bytes": len(data), "sha256": fw.sha256_file(path)}}
        with open(os.path.join(d, "build-info.json"), "w") as f:
            json.dump(info, f)
        return d

    def flash(self, board, **kw):
        kw.setdefault("root", self.out)
        kw.setdefault("yes", True)
        return capture(fw.flash, board, **kw)


class ShaVerification(Base):
    def test_mismatch_refuses_and_writes_nothing(self):
        d = self.make_build("skr-pico", "klipper.uf2")
        with open(os.path.join(d, "klipper.uf2"), "wb") as f:
            f.write(b"TAMPERED")
        drive = os.path.join(self.tmp.name, "RPI-RP2")
        os.makedirs(drive)
        r, _ = self.flash("skr-pico", mount=drive)
        self.assertIsInstance(r, fw.FwError)
        self.assertIn("MISMATCH", str(r))
        self.assertEqual(os.listdir(drive), [])

    def test_mismatch_refused_even_for_dry_run(self):
        d = self.make_build("skr-pico", "klipper.uf2")
        with open(os.path.join(d, "klipper.uf2"), "wb") as f:
            f.write(b"X")
        r, _ = self.flash("skr-pico", mount="/m", dry_run=True)
        self.assertIsInstance(r, fw.FwError)

    def test_missing_hash_record_refuses(self):
        self.make_build("skr-pico", "klipper.uf2", sha=False)
        r, _ = self.flash("skr-pico", mount=self.tmp.name)
        self.assertIsInstance(r, fw.FwError)
        self.assertIn("SHA-256", str(r))

    def test_file_changed_while_prompt_open_is_caught(self):
        d = self.make_build("creality-4.2.x", "klipper.bin")
        sd = os.path.join(self.tmp.name, "SD")
        os.makedirs(sd)

        def sneaky(prompt):
            with open(os.path.join(d, "klipper.bin"), "wb") as f:
                f.write(b"CHANGED")
            return "creality-4.2.x"
        r, _ = self.flash("creality-4.2.x", mount=sd, yes=False, isatty=True, input_fn=sneaky)
        self.assertIsInstance(r, fw.FwError)
        self.assertEqual(os.listdir(sd), [])

    def test_good_file_prints_sha_and_copies(self):
        self.make_build("skr-pico", "klipper.uf2")
        drive = os.path.join(self.tmp.name, "RPI-RP2")
        os.makedirs(drive)
        r, out = self.flash("skr-pico", mount=drive)
        self.assertEqual(r, 0)
        self.assertIn("matches build-info.json", out)
        with open(os.path.join(drive, "klipper.uf2"), "rb") as f:
            self.assertEqual(f.read(), b"FW")

    def test_primary_name_cannot_escape_build_folder(self):
        d = self.make_build("skr-pico", "klipper.uf2")
        with open(os.path.join(d, "build-info.json")) as f:
            info = json.load(f)
        info["primary_artifact"] = "../../evil.uf2"
        with open(os.path.join(d, "build-info.json"), "w") as f:
            json.dump(info, f)
        self.assertEqual(os.path.basename(fw.primary_artifact(d, "uf2-copy")), "klipper.uf2")


class MethodAndBoard(Base):
    def test_method_not_allowed_for_board(self):
        self.make_build("skr-pico", "klipper.uf2")
        r, _ = self.flash("skr-pico", method="sdcard-copy", mount=self.tmp.name)
        self.assertIsInstance(r, fw.FwError)
        r, _ = self.flash("skr-pico", method="avrdude", port="/dev/x")
        self.assertIsInstance(r, fw.FwError)
        self.assertIn("does not fit", str(r))

    def test_build_of_other_board_refused(self):
        d = self.make_build("skr-pico", "klipper.uf2", info_board="ramps-mega2560")
        r, _ = self.flash("skr-pico", artifact=d, mount=self.tmp.name)
        self.assertIsInstance(r, fw.FwError)
        self.assertIn("ramps-mega2560", str(r))

    def test_manual_never_touches_anything(self):
        self.make_build("skr-pico", "klipper.uf2")
        r, out = self.flash("skr-pico", method="manual")
        self.assertEqual(r, 0)
        self.assertNotIn("Copying", out)

    def test_every_real_flash_needs_confirmation(self):
        self.make_build("skr-pico", "klipper.uf2")
        drive = os.path.join(self.tmp.name, "D")
        os.makedirs(drive)
        r, _ = self.flash("skr-pico", mount=drive, yes=False, isatty=False)
        self.assertIsInstance(r, fw.FwError)
        self.assertEqual(os.listdir(drive), [])


class MediaEdgeCases(Base):
    def test_vanished_drive_before_copy_is_an_error(self):
        self.make_build("skr-pico", "klipper.uf2")
        r, _ = self.flash("skr-pico", mount=os.path.join(self.tmp.name, "gone"))
        self.assertIsInstance(r, fw.FwError)
        self.assertIn("not an available", str(r))

    def test_uf2_vanish_only_ok_when_all_bytes_written(self):
        err = OSError(5, "io")
        err.complete = False
        self.make_build("skr-pico", "klipper.uf2")
        drive = os.path.join(self.tmp.name, "D")
        os.makedirs(drive)
        orig = fw.copy_to_media

        def failing(src, dst, exclusive=False):
            raise err
        fw.copy_to_media = failing
        try:
            r, _ = self.flash("skr-pico", mount=drive)
            self.assertIsInstance(r, fw.FwError)      # incomplete write is never "OK"
            err.complete = True
            os.rmdir(drive)
            r, _ = self.flash("skr-pico", mount=drive)
            self.assertIsInstance(r, fw.FwError)      # dir missing before copy is caught earlier
            os.makedirs(drive)
            shutil_rm = __import__("shutil").rmtree

            def vanish(src, dst, exclusive=False):
                shutil_rm(os.path.dirname(dst))
                raise err
            fw.copy_to_media = vanish
            r, out = self.flash("skr-pico", mount=drive)
            self.assertEqual(r, 0)
            self.assertIn("Drive disappeared", out)
        finally:
            fw.copy_to_media = orig

    def test_readonly_card_gives_clear_error(self):
        if os.name == "nt" or os.geteuid() == 0:
            self.skipTest("permissions are not enforced here")
        self.make_build("creality-4.2.x", "klipper.bin")
        sd = os.path.join(self.tmp.name, "SD")
        os.makedirs(sd)
        os.chmod(sd, 0o500)
        try:
            r, _ = self.flash("creality-4.2.x", mount=sd)
        finally:
            os.chmod(sd, 0o700)
        self.assertIsInstance(r, fw.FwError)
        self.assertIn("copy failed", str(r))

    def test_copy_to_media_exclusive_never_overwrites(self):
        src = os.path.join(self.tmp.name, "s")
        dst = os.path.join(self.tmp.name, "d")
        for p in (src, dst):
            with open(p, "wb") as f:
                f.write(b"abc")
        with self.assertRaises(OSError):
            fw.copy_to_media(src, dst, exclusive=True)
        fw.copy_to_media(src, dst)

    def test_sd_file_is_verified_after_copy(self):
        self.make_build("creality-4.2.x", "klipper.bin", data=b"A" * 1000)
        sd = os.path.join(self.tmp.name, "SD")
        os.makedirs(sd)
        orig = fw.copy_to_media

        def truncating(src, dst, exclusive=False):
            with open(dst, "wb") as f:
                f.write(b"A" * 10)
        fw.copy_to_media = truncating
        try:
            r, _ = self.flash("creality-4.2.x", mount=sd)
        finally:
            fw.copy_to_media = orig
        self.assertIsInstance(r, fw.FwError)
        self.assertIn("does not match", str(r))

    def test_drive_root_without_slash_is_normalised(self):
        self.assertEqual(fw.normalize_mount("E:"), "E:" + os.sep)
        self.assertEqual(fw.normalize_mount("/media/x"), "/media/x")
        self.assertIsNone(fw.normalize_mount(None))

    def test_macos_startup_disk_symlink_is_not_a_candidate(self):
        vol = os.path.join(self.tmp.name, "Volumes")
        os.makedirs(os.path.join(vol, "CARD"))
        try:
            os.symlink(os.sep, os.path.join(vol, "Macintosh HD"))
        except (OSError, NotImplementedError):
            self.skipTest("no symlinks")
        self.assertEqual([os.path.basename(p) for p in fw.find_sd_mounts([vol])], ["CARD"])

    def test_unreadable_root_does_not_crash(self):
        self.assertEqual(fw.find_sd_mounts([os.path.join(self.tmp.name, "nope")]), [])

    def test_avrdude_missing_is_a_clear_error_before_prompt(self):
        self.make_build("ramps-mega2560", "klipper.elf.hex")
        asked = []
        r, _ = self.flash("ramps-mega2560", port="/dev/ttyACM0", yes=False, isatty=True,
                          input_fn=lambda p: asked.append(1) or "ramps-mega2560",
                          which_fn=lambda n: None)
        self.assertIsInstance(r, fw.FwError)
        self.assertIn("avrdude is not installed", str(r))
        self.assertEqual(asked, [])

    def test_run_stream_missing_program_is_fwerror(self):
        r, _ = capture(fw.run_stream, ["definitely-not-a-program-xyz"])
        self.assertIsInstance(r, fw.FwError)

    def test_avrdude_windows_com_port_and_dash_port(self):
        p = fw.get_preset("ramps-mega2560")
        self.assertIn("-P\\\\.\\COM12", fw.avrdude_argv(p, "f.hex", "COM12"))
        self.assertIn("-PCOM3", fw.avrdude_argv(p, "f.hex", "COM3"))
        d = self.make_build("ramps-mega2560", "klipper.elf.hex")
        with self.assertRaises(fw.FwError):
            fw.plan_flash(p, d, port="-evil")

    def test_paths_with_spaces_stay_one_argv_item(self):
        p = fw.get_preset("ramps-mega2560")
        argv = fw.avrdude_argv(p, "C:\\Program Files\\x y\\klipper.elf.hex", "COM3")
        self.assertEqual(sum(1 for a in argv if "Program Files" in a), 1)


class BuildLock(Base):
    def test_flash_refused_while_build_runs(self):
        self.make_build("skr-pico", "klipper.uf2")
        drive = os.path.join(self.tmp.name, "D")
        os.makedirs(drive)
        with fw._BuildLock():
            self.assertTrue(fw.build_in_progress())
            r, _ = self.flash("skr-pico", mount=drive)
            self.assertIsInstance(r, fw.FwError)
            self.assertIn("build is running", str(r))
            with self.assertRaises(fw.FwError):
                fw._BuildLock().__enter__()
            r, _ = self.flash("skr-pico", mount=drive, dry_run=True)    # dry run is harmless
            self.assertEqual(r, 0)
        self.assertFalse(fw.build_in_progress())
        r, _ = self.flash("skr-pico", mount=drive)
        self.assertEqual(r, 0)

    def test_stale_lock_is_ignored_and_replaced(self):
        os.makedirs(fw.home_dir())
        with open(fw._lock_path(), "w") as f:
            f.write("4000000")      # no such pid
        old = time.time() - 5 * 24 * 3600
        self.assertFalse(fw.build_in_progress(now=old + 13 * 3600 + 5 * 24 * 3600))
        if os.name != "nt":
            self.assertFalse(fw.build_in_progress())
        with fw._BuildLock():
            self.assertTrue(fw.build_in_progress())


class PinsRobustness(Base):
    def test_malformed_dicts(self):
        p = os.path.join(self.tmp.name, "d.json")
        for text, want in (("[]", []), ('{"enumerations": 5}', []), ('{"enumerations": {"pin": []}}', []),
                           ('{"enumerations": {"pin": {"gpio0": [0, "x"], "PA1": 1, "B": true}}}', ["PA1"])):
            with open(p, "w") as f:
                f.write(text)
            self.assertEqual(fw.pins_from_dict(p), want)
        with open(p, "w") as f:
            f.write("{not json")
        with self.assertRaises(fw.FwError):
            fw.pins_from_dict(p)


class EspSafety(unittest.TestCase):
    def test_install_argv_keeps_spaces_in_one_item(self):
        a = esp.install_argv("C:\\Program Files\\Python312\\python.exe", in_venv=False)
        self.assertEqual(a[0], "C:\\Program Files\\Python312\\python.exe")
        self.assertIn("--user", a)
        self.assertNotIn("--user", esp.install_argv("/x/python", in_venv=True))
        self.assertEqual(a[-1], "esptool")

    def test_port_starting_with_dash_refused(self):
        with self.assertRaises(fw.FwError):
            esp.chip_info_argv("--baud")

    def test_flash_needs_confirmation_and_esptool(self):
        with tempfile.TemporaryDirectory() as d:
            b = os.path.join(d, "a.bin")
            with open(b, "wb") as f:
                f.write(b"x")
            ran = []
            old = esp.esptool_available
            esp.esptool_available = lambda python=None: (True, "v")
            try:
                r, _ = capture(esp.flash, b, "/dev/ttyUSB0", isatty=False, runner=ran.append)
                self.assertIsInstance(r, fw.FwError)
                self.assertEqual(ran, [])
                esp.esptool_available = lambda python=None: (False, "missing")
                r, _ = capture(esp.flash, b, "/dev/ttyUSB0", yes=True, runner=ran.append)
                self.assertIsInstance(r, fw.FwError)
                self.assertEqual(ran, [])
            finally:
                esp.esptool_available = old


if __name__ == "__main__":
    unittest.main()
