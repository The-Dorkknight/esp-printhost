#!/usr/bin/env python3
# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""Unit tests for skipper_fw.py. No network, no hardware, no toolchains needed.

    python3 tools/test_skipper_fw.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import skipper_fw as fw


def write(path, text="", mode="w"):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, mode) as f:
        f.write(text)


def fake_klipper(root):
    write(os.path.join(root, "src", "Kconfig"), "config FOO\n    bool\n\nendmenu\n")
    write(os.path.join(root, "src", "Makefile"), "src-y += sched.c\n")
    return root


def read(path):
    with open(path) as f:
        return f.read()


def capture(fn, *a, **k):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        try:
            r = fn(*a, **k)
        except fw.FwError as e:
            r = e
    return r, buf.getvalue()


class PresetTests(unittest.TestCase):
    def test_all_shipped_presets_load_and_validate(self):
        presets = fw.load_presets()
        self.assertGreaterEqual(len(presets), 3)
        for need in ("skr-pico", "creality-4.2.x", "ramps-mega2560"):
            self.assertIn(need, presets)
        for k, p in presets.items():
            self.assertEqual(fw.validate_preset({x: v for x, v in p.items()
                                                 if x not in ("config_text", "config_path")}, p["config_text"]), [], k)
            self.assertIn(p["status"], fw.VALID_STATUS, k)
            if p["status"] == "hardware-tested":
                # every hardware claim must also be in the hardware log, by preset id
                log = open(os.path.join(fw.REPO_ROOT, "docs", "HARDWARE_LOG.md"), encoding="utf-8").read()
                self.assertIn(k, log, f"{k} is marked hardware-tested but docs/HARDWARE_LOG.md does not mention it")
            self.assertNotIn("WANT_SEGQ", p["config_text"])
            self.assertFalse([x for x in p if x.startswith("dda") or x == "build_sizes"], k)

    def test_validation_catches_problems(self):
        good = {k: v for k, v in fw.load_presets()["skr-pico"].items() if k not in ("config_text", "config_path")}
        cfg = "CONFIG_MACH_RP2040=y\n"
        self.assertEqual(fw.validate_preset(good, cfg), [])
        for key, bad in (("flash_method", "teleport"), ("mcu_arch", "z80"),
                         ("status", "hardware-verified"), ("id", "Bad Id"), ("sources", [])):
            m = dict(good, **{key: bad})
            self.assertTrue(fw.validate_preset(m, cfg), key)
        m = dict(good)
        del m["notes"]
        self.assertTrue(fw.validate_preset(m, cfg))
        self.assertTrue(fw.validate_preset(good, "CONFIG_MACH_AVR=y\n"))               # arch mismatch
        self.assertTrue(fw.validate_preset(good, cfg + "CONFIG_WANT_SEGQ=y\n"))        # the retired segq
        self.assertTrue(fw.validate_preset(good, cfg + "# CONFIG_WANT_SEGQ is not set\n"))
        self.assertTrue(fw.validate_preset(dict(good, status="hardware-tested"), cfg))  # needs tested_on
        self.assertEqual(fw.validate_preset(dict(good, status="hardware-tested", tested_on="x"), cfg), [])
        av = dict(good, flash_method="avrdude")
        self.assertTrue(fw.validate_preset(av, cfg))

    def test_unknown_board(self):
        with self.assertRaises(fw.FwError):
            fw.get_preset("nope")


OLD_KCONFIG_BLOCK = (fw.KCONFIG_BEGIN + "\nconfig WANT_SEGQ\n    bool \"Skipper Flipper segment queue (experimental)\"\n"
                     "    default y\n    depends on HAVE_GPIO\n" + fw.KCONFIG_END + "\n")
OLD_MAKEFILE_BLOCK = fw.KCONFIG_BEGIN + "\nsrc-$(CONFIG_WANT_SEGQ) += segq.c segq_core.c\n" + fw.KCONFIG_END + "\n"


def add_old_overlay(k):
    """What the tool used to do to a Klipper tree before 2026-10-25."""
    for n, block in (("Kconfig", OLD_KCONFIG_BLOCK), ("Makefile", OLD_MAKEFILE_BLOCK)):
        p = os.path.join(k, "src", n)
        write(p, read(p) + "\n" + block)
    for n in fw.LEGACY_OVERLAY_FILES:
        write(os.path.join(k, "src", n), "// old\n")


class PlainKlipperTests(unittest.TestCase):
    """Builds are plain Klipper: the old segment-queue overlay is removed from existing trees."""
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.k = fake_klipper(os.path.join(self.tmp.name, "k"))
        self.orig = {n: read(os.path.join(self.k, "src", n)) for n in ("Kconfig", "Makefile")}

    def tearDown(self):
        self.tmp.cleanup()

    def test_clean_tree_is_left_alone(self):
        self.assertFalse(fw.legacy_overlay_present(self.k))
        self.assertEqual(fw.remove_legacy_overlay(self.k), [])

    def test_old_overlay_removed_without_git(self):
        add_old_overlay(self.k)
        self.assertTrue(fw.legacy_overlay_present(self.k))
        ch = fw.remove_legacy_overlay(self.k)
        self.assertIn("src/Kconfig", ch)
        self.assertIn("src/segq.c", ch)
        self.assertFalse(fw.legacy_overlay_present(self.k))
        for n, t in self.orig.items():
            self.assertEqual(read(os.path.join(self.k, "src", n)), t)
        self.assertEqual(fw.remove_legacy_overlay(self.k), [])   # twice is fine

    def _git(self, *a):
        subprocess.run(["git", "-C", self.k, "-c", "user.name=t", "-c", "user.email=t@t"] + list(a),
                       check=True, capture_output=True)

    @unittest.skipUnless(shutil.which("git"), "git not installed")
    def test_old_overlay_removed_from_git_checkout_exactly(self):
        self._git("init", "-q")
        self._git("add", "-A")
        self._git("commit", "-qm", "klipper")
        add_old_overlay(self.k)
        with self.assertRaises(fw.FwError):
            fw.check_plain(self.k)
        fw.remove_legacy_overlay(self.k)
        fw.check_plain(self.k)                 # byte-identical to the commit again
        for n, t in self.orig.items():
            self.assertEqual(read(os.path.join(self.k, "src", n)), t)

    @unittest.skipUnless(shutil.which("git"), "git not installed")
    def test_local_edit_refuses_build(self):
        self._git("init", "-q")
        self._git("add", "-A")
        self._git("commit", "-qm", "klipper")
        p = os.path.join(self.k, "src", "Makefile")
        write(p, read(p) + "# my change\n")
        with self.assertRaises(fw.FwError):
            fw.check_plain(self.k)

    def test_set_config_option(self):
        t = fw.set_config_option("CONFIG_A=y\nCONFIG_WANT_SEGQ=y\n", "WANT_SEGQ", False)
        self.assertIn("# CONFIG_WANT_SEGQ is not set", t)
        self.assertNotIn("CONFIG_WANT_SEGQ=y", t)
        self.assertEqual(t.count("WANT_SEGQ"), 2 - 1)
        t = fw.set_config_option(t, "WANT_SEGQ", True)
        self.assertTrue(fw.config_has(t, "WANT_SEGQ"))
        self.assertEqual(t.count("WANT_SEGQ"), 1)


class MountTests(unittest.TestCase):
    def test_uf2_detection(self):
        with tempfile.TemporaryDirectory() as t:
            self.assertEqual(fw.find_uf2_drives([t]), [])
            os.makedirs(os.path.join(t, "RPI-RP2"))
            os.makedirs(os.path.join(t, "USBSTICK"))
            os.makedirs(os.path.join(t, "OTHER"))
            write(os.path.join(t, "OTHER", "INFO_UF2.TXT"), "UF2 Bootloader")
            self.assertEqual(sorted(os.path.basename(x) for x in fw.find_uf2_drives([t, "/nonexistent"])),
                             ["OTHER", "RPI-RP2"])
            self.assertEqual([os.path.basename(x) for x in fw.find_sd_mounts([t])], ["USBSTICK"])

    def test_roots(self):
        self.assertIn("/media/bob", fw.mount_roots("bob"))
        self.assertIn("/run/media/bob", fw.mount_roots("bob"))
        self.assertIn("/Volumes", fw.mount_roots("bob"))


class FlashFixture(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.out = os.path.join(self.tmp.name, "out")
        self.presets = fw.load_presets()

    def tearDown(self):
        self.tmp.cleanup()

    def make_build(self, board, primary, stamp="20260101-1200"):
        d = os.path.join(self.out, f"{board}-{stamp}")
        write(os.path.join(d, primary), "FW")
        write(os.path.join(d, "klipper.elf"), "ELF")
        write(os.path.join(d, "build-info.json"),
              json.dumps({"board": board, "plain_klipper": True, "klipper_tag": "v0.13.0", "primary_artifact": primary,
                          "artifacts": {primary: {"bytes": 2, "sha256": fw.sha256_file(os.path.join(d, primary))}}}))
        return d


class ArtifactTests(FlashFixture):
    def test_discovery_picks_newest(self):
        a = self.make_build("skr-pico", "klipper.uf2", "20260101-1200")
        b = self.make_build("skr-pico", "klipper.uf2", "20260101-1300")
        c = self.make_build("skr-pico", "klipper.uf2", "20260101-1300-2")
        self.make_build("creality-4.2.x", "klipper.bin", "20270101-1200")
        os.makedirs(os.path.join(self.out, "skr-pico-garbage"))
        self.assertEqual(fw.list_artifact_dirs("skr-pico", self.out), [c, b, a])
        self.assertEqual(fw.find_latest_artifact("skr-pico", self.out), c)
        self.assertIsNone(fw.find_latest_artifact("ramps-mega2560", self.out))
        self.assertTrue(fw.primary_artifact(a, "uf2-copy").endswith("klipper.uf2"))
        self.assertIsNone(fw.primary_artifact(a, "sdcard-copy"))

    def test_unique_dir_and_names(self):
        self.assertEqual(os.path.basename(fw.unique_dir(self.tmp.name, "x")), "x")
        os.makedirs(os.path.join(self.tmp.name, "x"))
        os.makedirs(os.path.join(self.tmp.name, "x-2"))
        self.assertEqual(os.path.basename(fw.unique_dir(self.tmp.name, "x")), "x-3")
        self.assertEqual(fw.unique_sd_name(self.tmp.name, "S1"), "skipper-S1.bin")
        write(os.path.join(self.tmp.name, "skipper-S1.bin"))
        self.assertEqual(fw.unique_sd_name(self.tmp.name, "S1"), "skipper-S1-2.bin")
        write(os.path.join(self.tmp.name, "skipper-S1-2.bin"))
        self.assertEqual(fw.unique_sd_name(self.tmp.name, "S1"), "skipper-S1-3.bin")

    def test_parse_size(self):
        s = fw.parse_size_output("   text\t   data\t    bss\t    dec\t    hex\tfilename\n  100\t 20\t 30\t 150\t 96\tx.elf\n")
        self.assertEqual((s["flash_bytes"], s["ram_bytes"]), (120, 50))
        self.assertIsNone(fw.parse_size_output("garbage"))

    def test_sha256(self):
        p = os.path.join(self.tmp.name, "f")
        write(p, "abc")
        self.assertEqual(fw.sha256_file(p), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad")


class FlashPlanTests(FlashFixture):
    def test_uf2_copy(self):
        d = self.make_build("skr-pico", "klipper.uf2")
        roots = os.path.join(self.tmp.name, "media")
        with self.assertRaises(fw.FwError) as cm:
            fw.plan_flash(self.presets["skr-pico"], d, roots=[roots])
        self.assertIn("BOOTSEL", str(cm.exception))
        drive = os.path.join(roots, "RPI-RP2")
        os.makedirs(drive)
        steps = fw.plan_flash(self.presets["skr-pico"], d, roots=[roots])
        self.assertEqual(steps[0].kind, "copy")
        self.assertEqual(steps[0].dst, os.path.join(drive, "klipper.uf2"))
        os.makedirs(os.path.join(roots, "RPI-RP2-2"))
        with self.assertRaises(fw.FwError):
            fw.plan_flash(self.presets["skr-pico"], d, roots=[roots])
        steps = fw.plan_flash(self.presets["skr-pico"], d, mount="/explicit", roots=[roots])
        self.assertEqual(steps[0].dst, "/explicit/klipper.uf2")

    def test_sdcard_copy(self):
        d = self.make_build("creality-4.2.x", "klipper.bin")
        sd = os.path.join(self.tmp.name, "SD")
        os.makedirs(sd)
        steps = fw.plan_flash(self.presets["creality-4.2.x"], d, mount=sd, stamp="T")
        self.assertEqual(steps[0].dst, os.path.join(sd, "skipper-T.bin"))
        self.assertEqual(steps[1].argv, ["sync"])
        self.assertIn("power-cycle", steps[2].text)
        write(os.path.join(sd, "skipper-T.bin"))
        steps = fw.plan_flash(self.presets["creality-4.2.x"], d, mount=sd, stamp="T")
        self.assertTrue(steps[0].dst.endswith("skipper-T-2.bin"))
        with self.assertRaises(fw.FwError):          # no mount, nothing detectable
            fw.plan_flash(self.presets["creality-4.2.x"], d, roots=[os.path.join(self.tmp.name, "none")])

    def test_avrdude(self):
        d = self.make_build("ramps-mega2560", "klipper.elf.hex")
        steps = fw.plan_flash(self.presets["ramps-mega2560"], d, port="/dev/ttyACM0")
        hexf = os.path.join(d, "klipper.elf.hex")
        self.assertEqual(steps[0].argv, ["avrdude", "-patmega2560", "-cwiring", "-P/dev/ttyACM0",
                                         "-D", f"-Uflash:w:{hexf}:i"])
        with self.assertRaises(fw.FwError):
            fw.plan_flash(self.presets["ramps-mega2560"], d)
        if "ender3-1.1.x" in self.presets:
            d2 = self.make_build("ender3-1.1.x", "klipper.elf.hex")
            argv = fw.plan_flash(self.presets["ender3-1.1.x"], d2, port="/dev/ttyUSB0")[0].argv
            self.assertEqual(argv[:6], ["avrdude", "-patmega1284p", "-carduino", "-P/dev/ttyUSB0", "-b", "57600"])
            argv = fw.plan_flash(self.presets["ender3-1.1.x"], d2, port="/dev/ttyUSB0", baud=115200)[0].argv
            self.assertIn("115200", argv)

    def test_manual(self):
        d = self.make_build("skr-pico", "klipper.uf2")
        steps = fw.plan_flash(self.presets["skr-pico"], d, method="manual")
        self.assertTrue(all(s.kind == "note" for s in steps))
        self.assertIn("klipper.uf2", steps[0].text)

    def test_wrong_method_for_build(self):
        d = self.make_build("skr-pico", "klipper.uf2")
        with self.assertRaises(fw.FwError):
            fw.plan_flash(self.presets["skr-pico"], d, method="sdcard-copy", mount="/x")
        with self.assertRaises(fw.FwError):
            fw.plan_flash(self.presets["skr-pico"], d, method="teleport")


class FlashCommandTests(FlashFixture):
    def run_flash(self, board, **kw):
        kw.setdefault("root", self.out)
        return capture(fw.flash, board, **kw)

    def test_refuses_without_confirmation(self):
        self.make_build("creality-4.2.x", "klipper.bin")
        sd = os.path.join(self.tmp.name, "SD")
        os.makedirs(sd)
        r, out = self.run_flash("creality-4.2.x", mount=sd, isatty=False)
        self.assertIsInstance(r, fw.FwError)
        self.assertEqual(os.listdir(sd), [])
        self.assertIn("WARNING", out)
        # typed wrong text
        r, _ = self.run_flash("creality-4.2.x", mount=sd, isatty=True, input_fn=lambda p: "yes")
        self.assertIsInstance(r, fw.FwError)
        self.assertEqual(os.listdir(sd), [])
        # EOF
        def eof(p):
            raise EOFError
        r, _ = self.run_flash("creality-4.2.x", mount=sd, isatty=True, input_fn=eof)
        self.assertIsInstance(r, fw.FwError)
        self.assertEqual(os.listdir(sd), [])

    def test_typed_board_id_confirms(self):
        self.make_build("creality-4.2.x", "klipper.bin")
        sd = os.path.join(self.tmp.name, "SD")
        os.makedirs(sd)
        r, out = self.run_flash("creality-4.2.x", mount=sd, isatty=True,
                                input_fn=lambda p: "creality-4.2.x", stamp="T")
        self.assertEqual(r, 0)
        self.assertEqual(os.listdir(sd), ["skipper-T.bin"])

    def test_dry_run_changes_nothing_and_prints_commands(self):
        d = self.make_build("ramps-mega2560", "klipper.elf.hex")
        r, out = self.run_flash("ramps-mega2560", port="/dev/ttyACM0", dry_run=True)
        self.assertEqual(r, 0)
        self.assertIn("avrdude -patmega2560 -cwiring -P/dev/ttyACM0 -D", out)
        self.assertIn("DRY RUN", out)
        sd = os.path.join(self.tmp.name, "SD")
        os.makedirs(sd)
        self.make_build("creality-4.2.x", "klipper.bin")
        r, out = self.run_flash("creality-4.2.x", mount=sd, dry_run=True, stamp="T")
        self.assertEqual(r, 0)
        self.assertEqual(os.listdir(sd), [])
        self.assertIn(os.path.join(sd, "skipper-T.bin"), out)

    def test_yes_copies_for_uf2_and_sd(self):
        self.make_build("skr-pico", "klipper.uf2")
        drive = os.path.join(self.tmp.name, "RPI-RP2")
        os.makedirs(drive)
        r, out = self.run_flash("skr-pico", mount=drive, yes=True)
        self.assertEqual(r, 0)
        self.assertEqual(read(os.path.join(drive, "klipper.uf2")), "FW")

    def test_no_build_is_an_error(self):
        r, _ = self.run_flash("skr-pico", yes=True)
        self.assertIsInstance(r, fw.FwError)

    def test_explicit_artifact_dir(self):
        d = self.make_build("skr-pico", "klipper.uf2", "20260101-1200")
        self.make_build("skr-pico", "klipper.uf2", "20260202-1200")
        r, out = capture(fw.flash, "skr-pico", artifact=d, mount="/m", dry_run=True, root=self.out)
        self.assertEqual(r, 0)
        self.assertIn("20260101-1200", out)

    def test_main_cli_dry_run_and_refusal(self):
        self.make_build("ramps-mega2560", "klipper.elf.hex")
        old = os.environ.get("SKIPPER_FLIPPER_HOME")
        os.environ["SKIPPER_FLIPPER_HOME"] = self.tmp.name
        try:
            r, out = capture(fw.main, ["flash", "ramps-mega2560", "--port", "/dev/ttyACM0", "--dry-run"])
            self.assertEqual(r, 0)
            self.assertIn("avrdude", out)
            r, out = capture(fw.main, ["flash", "ramps-mega2560", "--port", "/dev/ttyACM0"])
            self.assertEqual(r, 2)      # stdin is not a tty under the test runner
            self.assertIn("not confirmed", out)
        finally:
            if old is None:
                del os.environ["SKIPPER_FLIPPER_HOME"]
            else:
                os.environ["SKIPPER_FLIPPER_HOME"] = old


class ToolCheckTests(unittest.TestCase):
    def test_missing_tools_give_apt_hint(self):
        rows, pkgs = fw.check_tools(["avr", "stm32"], ["avrdude"], which_fn=lambda n: None, libcheck=lambda c, l: True)
        for p in ("gcc-avr", "binutils-avr", "gcc-arm-none-eabi", "avrdude", "git"):
            self.assertIn(p, pkgs)
        self.assertTrue(fw.apt_hint(pkgs).startswith("sudo apt install "))
        rows, pkgs = fw.check_tools(["rp2040"], [], which_fn=lambda n: "/usr/bin/" + n, libcheck=lambda c, l: True)
        self.assertEqual(pkgs, [])
        rows, pkgs = fw.check_tools(["stm32"], [], which_fn=lambda n: "/usr/bin/" + n, libcheck=lambda c, l: False)
        self.assertEqual(pkgs, ["libnewlib-arm-none-eabi"])


if __name__ == "__main__":
    unittest.main(verbosity=1)
