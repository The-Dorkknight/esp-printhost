# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""Tests for the shipped-firmware / cross-platform parts of skipper_fw (no hardware, no network)."""
import io
import contextlib
import json
import os
import tempfile
import unittest

import skipper_fw as fw


def write(path, data=b"x"):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb" if isinstance(data, bytes) else "w") as f:
        f.write(data)


class Pins(unittest.TestCase):
    def test_ranges_and_singles(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "k.dict")
            json.dump({"enumerations": {"pin": {"gpio0": [0, 3], "ADC_TEMPERATURE": 99, "PA0": [100, 2]}}},
                      open(p, "w"))
            self.assertEqual(fw.pins_from_dict(p), ["ADC_TEMPERATURE", "PA0", "PA1", "gpio0", "gpio1", "gpio2"])

    def test_no_pins(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "k.dict")
            json.dump({}, open(p, "w"))
            self.assertEqual(fw.pins_from_dict(p), [])


class Shipped(unittest.TestCase):
    def test_every_preset_has_shipped_firmware_with_right_file_and_hash(self):
        for b, preset in fw.load_presets().items():
            d = fw.find_prebuilt(b)
            self.assertTrue(d, f"no shipped firmware for {b}")
            info = fw.read_build_info(d)
            self.assertTrue(info["segq"])
            self.assertTrue(info["prebuilt"])
            f = fw.primary_artifact(d, preset["flash_method"])
            self.assertTrue(f and os.path.isfile(f), b)
            self.assertEqual(fw.sha256_file(f), info["artifacts"][os.path.basename(f)]["sha256"], b)
            pins = json.load(open(os.path.join(d, "pins.json")))
            self.assertEqual(pins["count"], len(pins["pins"]))
            self.assertGreater(pins["count"], 10)

    def test_shipped_firmware_is_not_stale(self):
        """Fails when firmware/skipper/*.[ch] or a preset .config changed after the prebuilt
        firmware was made. Fix: python3 host/tools/skipper_fw.py bundle"""
        for b, preset in fw.load_presets().items():
            info = fw.read_build_info(fw.find_prebuilt(b))
            want = info.get("source_sha256")
            self.assertIsInstance(want, dict, f"{b}: build-info.json has no source_sha256; re-run bundle")
            self.assertEqual(want, fw.source_hashes(preset),
                             f"{b}: shipped firmware is older than its sources; re-run: "
                             "python3 host/tools/skipper_fw.py bundle")

    def test_source_hashes_cover_overlay_and_preset(self):
        h = fw.source_hashes(fw.get_preset("skr-pico"))
        self.assertEqual(sorted(h), sorted(list(fw.OVERLAY_FILES) + ["skr-pico.config"]))
        with tempfile.TemporaryDirectory() as d:
            for n in fw.OVERLAY_FILES:
                write(os.path.join(d, n), b"a")
            h1 = fw.source_hashes(fw.get_preset("skr-pico"), skipper_dir=d)
            write(os.path.join(d, "segq.c"), b"b")
            h2 = fw.source_hashes(fw.get_preset("skr-pico"), skipper_dir=d)
            self.assertNotEqual(h1["segq.c"], h2["segq.c"])
            self.assertEqual(h1["segq_core.c"], h2["segq_core.c"])

    def test_skr_pico_pin_count_matches_probe(self):
        # mcu_probe --pins read 31 names from the real SKR Pico (older Klipper); same chip family
        self.assertEqual(json.load(open(os.path.join(fw.find_prebuilt("skr-pico"), "pins.json")))["count"], 31)

    def test_find_prebuilt_missing(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertIsNone(fw.find_prebuilt("nope", root=d))

    def test_bundle_copies_only_wanted_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            def fake_build(board, segq=True, jobs=None):
                a = os.path.join(tmp, "out", board)
                write(os.path.join(a, "klipper.uf2"), b"uf2")
                write(os.path.join(a, "klipper.elf"), b"big")
                write(os.path.join(a, "klipper.config"), "cfg")
                json.dump({"board": board, "segq": segq, "primary_artifact": "klipper.uf2",
                           "artifacts": {"klipper.uf2": {"bytes": 3, "sha256": "x"}, "klipper.elf": {"bytes": 3}}},
                          open(os.path.join(a, "build-info.json"), "w"))
                json.dump({"enumerations": {"pin": {"gpio0": [0, 2]}}}, open(os.path.join(a, "klipper.dict"), "w"))
                return a
            dest = os.path.join(tmp, "pre")
            with contextlib.redirect_stdout(io.StringIO()):
                fw.bundle(["skr-pico"], dest_root=dest, build_fn=fake_build)
            got = sorted(os.listdir(os.path.join(dest, "skr-pico")))
            self.assertEqual(got, ["build-info.json", "klipper.dict", "klipper.uf2", "pins.json"])
            info = json.load(open(os.path.join(dest, "skr-pico", "build-info.json")))
            self.assertNotIn("klipper.elf", info["artifacts"])
            self.assertTrue(info["prebuilt"])


class FlashFromShipped(unittest.TestCase):
    def run_flash(self, board, **kw):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            rc = fw.flash(board, root="/nonexistent", yes=True, **kw)
        return rc, out.getvalue()

    def test_uf2_copy_to_fake_drive(self):
        with tempfile.TemporaryDirectory() as drive:
            rc, out = self.run_flash("skr-pico", mount=drive)
            self.assertEqual(rc, 0)
            self.assertTrue(os.path.isfile(os.path.join(drive, "klipper.uf2")))
            self.assertIn("firmware/prebuilt/skr-pico", out.replace("\\", "/"))

    def test_sdcard_copy_new_name_each_time(self):
        with tempfile.TemporaryDirectory() as card:
            self.run_flash("creality-4.2.x", mount=card, stamp="20260101-000000")
            self.run_flash("creality-4.2.x", mount=card, stamp="20260101-000000")
            self.assertEqual(sorted(os.listdir(card)),
                             ["skipper-20260101-000000-2.bin", "skipper-20260101-000000.bin"])

    def test_dry_run_changes_nothing(self):
        with tempfile.TemporaryDirectory() as card:
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                fw.flash("creality-4.2.x", root="/nonexistent", mount=card, dry_run=True)
            self.assertEqual(os.listdir(card), [])

    def test_not_confirmed_refuses(self):
        with tempfile.TemporaryDirectory() as card:
            with contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaises(fw.FwError):
                    fw.flash("creality-4.2.x", root="/nonexistent", mount=card, isatty=False)
            self.assertEqual(os.listdir(card), [])

    def test_sd_plan_has_no_sync_on_windows(self):
        card = tempfile.mkdtemp()
        p = fw.get_preset("creality-4.2.x")
        steps = fw.plan_flash(p, fw.find_prebuilt("creality-4.2.x"), mount=card)
        self.assertEqual([s.kind for s in steps].count("run"), 0 if os.name == "nt" else 1)


class Platform(unittest.TestCase):
    def test_methods_per_chip(self):
        P = fw.load_presets()
        self.assertEqual(fw.methods_for(P["creality-4.2.x"]), ("sdcard-copy", "manual"))
        self.assertEqual(fw.methods_for(P["skr-pico"]), ("uf2-copy", "manual"))
        self.assertEqual(fw.methods_for(P["ramps-mega2560"]), ("avrdude", "manual"))
        for p in P.values():
            self.assertIn(p["flash_method"], fw.methods_for(p))

    def test_can_build_here(self):
        ok, why = fw.can_build_here("win32", which_fn=lambda n: "/x")
        self.assertFalse(ok)
        self.assertIn("shipped", why)
        self.assertTrue(fw.can_build_here("linux", which_fn=lambda n: "/x")[0])
        self.assertFalse(fw.can_build_here("linux", which_fn=lambda n: None)[0])
        ok, why = fw.can_build_here("darwin", which_fn=lambda n: "/x")
        self.assertTrue(ok)
        self.assertIn("untested", why)

    def test_windows_drive_detection(self):
        with tempfile.TemporaryDirectory() as boot, tempfile.TemporaryDirectory() as card:
            write(os.path.join(boot, "INFO_UF2.TXT"), "UF2 Bootloader")
            self.assertEqual(fw.find_uf2_drives(windows_drives=[boot, card]), [boot])
            self.assertEqual(fw.find_sd_mounts(windows_drives=[boot, card]), [card])

    def test_windows_removable_drives_mock(self):
        class K:
            def GetLogicalDrives(self): return 0b11100          # C, D, E
            def GetDriveTypeW(self, root): return 2 if root == "E:\\" else 3
        class W: kernel32 = K()
        class CT: windll = W()
        self.assertEqual(fw.windows_removable_drives(CT()), ["E:\\"])

    def test_windows_removable_drives_not_windows(self):
        class CT: pass
        self.assertEqual(fw.windows_removable_drives(CT()), [])

    def test_linux_roots_scan(self):
        with tempfile.TemporaryDirectory() as root:
            os.makedirs(os.path.join(root, "RPI-RP2"))
            os.makedirs(os.path.join(root, "SDCARD"))
            self.assertEqual(fw.find_uf2_drives([root]), [os.path.join(root, "RPI-RP2")])
            self.assertEqual(fw.find_sd_mounts([root]), [os.path.join(root, "SDCARD")])


if __name__ == "__main__":
    unittest.main()
