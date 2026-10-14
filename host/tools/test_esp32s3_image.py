# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""Tests for the shipped ESP32-S3 image (no hardware, no ESP-IDF needed)."""
import contextlib
import io
import json
import os
import shutil
import tempfile
import unittest

import skipper_esp as esp
from skipper_fw import FwError, sha256_file


class Shipped(unittest.TestCase):
    def test_present_and_hash_matches(self):
        s = esp.shipped_image()
        self.assertIsNotNone(s, "run firmware/esp32s3/build.sh to (re)create the shipped image")
        path, info = s
        self.assertEqual(sha256_file(path), info["sha256"])
        self.assertEqual(os.path.getsize(path), info["bytes"])
        self.assertEqual((info["target"], info["offset"]), ("esp32s3", "0x0"))
        self.assertIn(info["status"], ("compile-only", "hardware-tested"))
        if info["status"] == "hardware-tested":
            self.assertIn("result", info["hardware_test"])
            self.assertIn("not_tested", info["hardware_test"])

    def test_merged_layout(self):
        path, _ = esp.shipped_image()
        with open(path, "rb") as f:
            d = f.read()
        self.assertEqual(d[0], 0xE9)                    # bootloader image magic at 0x0
        self.assertEqual(d[0x8000:0x8002], b"\xaa\x50")  # partition table magic
        self.assertEqual(d[0x10000], 0xE9)              # app image magic at 0x10000
        self.assertLessEqual(len(d), 0x190000)          # inside the 1.5 MiB factory partition + 0x10000

    def test_not_stale(self):
        _, info = esp.shipped_image()
        self.assertEqual(info["source_sha256"], esp.esp_source_hashes(),
                         "ESP32-S3 sources changed: rebuild with firmware/esp32s3/build.sh")

    def test_password_length_guard_in_source(self):
        k = open(os.path.join(esp.ESP_SRC_DIR, "main", "Kconfig.projbuild")).read()
        self.assertIn('default "skipperflipper"', k)
        self.assertGreaterEqual(len("skipperflipper"), 8)

    def test_verify_refuses_tampered_copy(self):
        with tempfile.TemporaryDirectory() as root:
            src, info = esp.shipped_image()
            shutil.copy(src, os.path.join(root, info["image"]))
            shutil.copy(os.path.join(os.path.dirname(src), "build-info.json"), root)
            f = os.path.join(root, info["image"])
            self.assertEqual(esp.verify_image(f, root)["sha256"], info["sha256"])
            with open(f, "r+b") as fh:
                fh.seek(100)
                fh.write(b"\x00\x01\x02")
            with self.assertRaises(FwError):
                esp.verify_image(f, root)

    def test_verify_ignores_other_files(self):
        with tempfile.NamedTemporaryFile(suffix=".bin") as t:
            t.write(b"x")
            t.flush()
            self.assertIsNone(esp.verify_image(t.name))

    def test_dry_run_with_shipped(self):
        path, _ = esp.shipped_image()
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(esp.flash(path, "/dev/ttyACM0", dry_run=True, python="py"), 0)
        text = out.getvalue()
        self.assertIn("matches", text)
        self.assertIn("write-flash 0x0", text)

    def test_old_esptool_flash_uses_old_names(self):
        path, _ = esp.shipped_image()
        calls = []
        with contextlib.redirect_stdout(io.StringIO()):
            from unittest import mock
            with mock.patch.object(esp, "esptool_available", return_value=(True, "esptool.py v4.8.1")):
                esp.flash(path, "/dev/ttyACM0", yes=True, python="py", runner=lambda a: calls.append(a))
        self.assertEqual(calls[0][-3], "write_flash")

    def test_shipped_must_use_offset_zero(self):
        path, _ = esp.shipped_image()
        with contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(FwError):
                esp.flash(path, "/dev/ttyACM0", offset="0x10000", dry_run=True, python="py")

    def test_flash_refuses_tampered_shipped(self):
        with tempfile.TemporaryDirectory() as root:
            src, info = esp.shipped_image()
            f = os.path.join(root, info["image"])
            shutil.copy(src, f)
            shutil.copy(os.path.join(os.path.dirname(src), "build-info.json"), root)
            with open(f, "r+b") as fh:
                fh.write(b"\x00")
            real = esp.ESP_PREBUILT_DIR
            esp.ESP_PREBUILT_DIR = root
            try:
                with contextlib.redirect_stdout(io.StringIO()):
                    with self.assertRaises(FwError):
                        esp.flash(f, "/dev/ttyACM0", dry_run=True, python="py")
            finally:
                esp.ESP_PREBUILT_DIR = real

    def test_bundle_image_roundtrip(self):
        with tempfile.TemporaryDirectory() as d:
            m = os.path.join(d, "m.bin")
            with open(m, "wb") as f:
                f.write(b"\xe9" * 64)
            root = os.path.join(d, "out")
            esp.bundle_image(m, "v9.9", root=root)
            s = esp.shipped_image(root)
            self.assertEqual(s[1]["idf"], "v9.9")
            self.assertEqual(s[1]["sha256"], sha256_file(s[0]))
            self.assertEqual(s[1]["source_sha256"], esp.esp_source_hashes())


if __name__ == "__main__":
    unittest.main()
