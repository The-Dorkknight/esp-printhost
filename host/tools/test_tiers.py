# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""DECKHAND / CAPTAIN / SKIPPER: reading esptool's chip report, choosing the build, the shipped
images and the one-click flash (no hardware, no esptool needed)."""
import contextlib
import io
import json
import os
import re
import unittest

import skipper_esp as esp
from skipper_fw import FwError, sha256_file

# Real esptool wording (v5: "Chip type", "Wi-Fi"; v4: "Chip is", "WiFi").
ESPTOOL5_ZERO = """esptool v5.1.0
Connected to ESP32-S3 on /dev/ttyACM0:
Chip type:          ESP32-S3 (QFN56) (revision v0.2)
Features:           Wi-Fi, BT 5 (LE), Dual Core + LP Core, 240MHz, Embedded Flash 4MB (XMC), Embedded PSRAM 2MB (AP_3v3)
Crystal frequency:  40MHz
MAC:                dc:da:0c:11:22:33
Flash Memory Information:
Manufacturer: 20
Device: 4016
Detected flash size: 4MB
"""
ESPTOOL4_N16R8 = """esptool.py v4.8.1
Serial port /dev/ttyUSB0
Connecting....
Chip is ESP32-S3 (QFN56) (revision v0.2)
Features: WiFi, BLE, Embedded PSRAM 8MB (AP_3v3)
Crystal is 40MHz
MAC: 34:85:18:aa:bb:cc
Manufacturer: c8
Device: 4018
Detected flash size: 16MB
"""
ESPTOOL5_N4 = """Chip type:          ESP32-S3 (QFN56) (revision v0.2)
Features:           Wi-Fi, BT 5 (LE), Dual Core + LP Core, 240MHz
Detected flash size: 4MB
"""
ESPTOOL5_2MB = """Chip type:          ESP32-S3 (QFN56) (revision v0.1)
Features:           Wi-Fi, BT 5 (LE), Dual Core + LP Core, 240MHz
Detected flash size: 2MB
"""
ESPTOOL_C3 = """Chip type:          ESP32-C3 (QFN32) (revision v0.4)
Detected flash size: 4MB
"""


class Choose(unittest.TestCase):
    def test_parse(self):
        i = esp.parse_chip_info(ESPTOOL5_ZERO)
        self.assertEqual((i["chip"], i["flash_mb"], i["embedded_flash_mb"], i["psram_mb"]), ("ESP32-S3", 4, 4, 2))
        i = esp.parse_chip_info(ESPTOOL4_N16R8)
        self.assertEqual((i["chip"], i["flash_mb"], i["psram_mb"]), ("ESP32-S3", 16, 8))
        self.assertEqual(esp.parse_chip_info("")["flash_mb"], None)

    def test_choose(self):
        cases = [(ESPTOOL5_ZERO, "captain"), (ESPTOOL4_N16R8, "skipper"), (ESPTOOL5_N4, "deckhand"),
                 (ESPTOOL5_2MB, "deckhand"), ("garbage", "deckhand")]
        for text, want in cases:
            t, why = esp.choose_tier(esp.parse_chip_info(text))
            self.assertEqual(t, want, why)
            self.assertTrue(why)
        # 8 MB flash with 2 MB (quad) PSRAM: CAPTAIN, SKIPPER needs 8 MB (octal) PSRAM
        self.assertEqual(esp.choose_tier({"chip": "ESP32-S3", "flash_mb": 8, "psram_mb": 2})[0], "captain")
        # 4 MB flash with 8 MB PSRAM: SKIPPER's 8 MB layout would not fit
        self.assertEqual(esp.choose_tier({"chip": "ESP32-S3", "flash_mb": 4, "psram_mb": 8})[0], "captain")
        with self.assertRaises(FwError):
            esp.choose_tier(esp.parse_chip_info(ESPTOOL_C3))

    def test_same_rule_as_firmware(self):
        """main/tier.c decides the /about page's upgrade hint with the same thresholds."""
        src = open(os.path.join(esp.ESP_SRC_DIR, "main", "tier.c")).read()
        self.assertIn('(flash >= (8u << 20) && psram_mb >= 8)   ? "skipper"', src)
        self.assertIn('(flash >= (4u << 20) && psram_mb >= 2) ? "captain"', src)


class Shipped(unittest.TestCase):
    def test_three_tiers_present(self):
        tiers = esp.shipped_tiers()
        self.assertEqual(sorted(tiers), sorted(esp.TIERS), "run firmware/esp32s3/build.sh")
        for t, (path, info) in tiers.items():
            self.assertEqual(sha256_file(path), info["sha256"], t)
            self.assertEqual(info["flash_size"], esp.TIER_FLASH[t])
            with open(path, "rb") as f:
                d = f.read()
            self.assertEqual(d[0], 0xE9)
            self.assertEqual(d[0x8000:0x8002], b"\xaa\x50")
            app_at = 0x10000 if t == "deckhand" else 0x20000   # factory slot vs first OTA slot
            self.assertEqual(d[app_at], 0xE9, t)
            # the app's own description names its tier (the WiFi update checks it)
            ver = re.search(rb"0\.1\.0-wip-(deckhand|captain|skipper)", d[app_at:app_at + 0x400])
            self.assertIsNotNone(ver, t)
            self.assertEqual(ver.group(1).decode(), t)
            if info.get("symbols"):
                self.assertTrue(os.path.isfile(os.path.join(os.path.dirname(path), info["symbols"])), t)
            if t != "deckhand":
                app = os.path.join(os.path.dirname(path), info["app_image"])
                self.assertEqual(sha256_file(app), info["app_sha256"])
                with open(app, "rb") as f:
                    self.assertEqual(f.read(), d[app_at:app_at + info["app_bytes"]])

    def test_partition_tables_fit_and_keep_nvs(self):
        """NVS stays at 0x9000 in every tier (settings survive a tier change); each table fits its flash."""
        rows = {}
        for t, fn in (("deckhand", "partitions.csv"), ("captain", "tiers/partitions_captain.csv"),
                      ("skipper", "tiers/partitions_skipper.csv")):
            parts = []
            with open(os.path.join(esp.ESP_SRC_DIR, fn)) as fh:
                lines = fh.readlines()
            for ln in lines:
                ln = ln.split("#")[0].strip()
                if ln:
                    name, _ty, _sub, off, size = [x.strip() for x in ln.split(",")[:5]]
                    parts.append((name, int(off, 16), int(size, 16)))
            rows[t] = parts
            limit = int(esp.TIER_FLASH[t][:-2]) << 20
            for name, off, size in parts:
                self.assertLessEqual(off + size, limit, f"{t}:{name}")
            nvs = [p for p in parts if p[0] == "nvs"][0]
            self.assertEqual((nvs[1], nvs[2]), (0x9000, 0x6000), t)
            ends = sorted((o, o + s) for _n, o, s in parts)
            for (a0, a1), (b0, _b1) in zip(ends, ends[1:]):
                self.assertLessEqual(a1, b0, f"{t}: overlapping partitions")
        for t in esp.TIERS:   # every build keeps the last crash's core dump (main/crash.c)
            self.assertIn("coredump", [p[0] for p in rows[t]], t)
        for t in ("captain", "skipper"):
            names = [p[0] for p in rows[t]]
            self.assertIn("ota_0", names)
            self.assertIn("ota_1", names)
            self.assertIn("otadata", names)
            self.assertEqual([p for p in rows[t] if p[0] == "ota_0"][0][1], 0x20000)

    def test_verify_covers_every_tier_and_app_images(self):
        for t, (path, info) in esp.shipped_tiers().items():
            self.assertEqual(esp.verify_image(path)["tier"], t)
            if info.get("app_image"):
                v = esp.verify_image(os.path.join(os.path.dirname(path), info["app_image"]))
                self.assertTrue(v["app_only"])
                self.assertEqual(v["offset"], "0x20000")


class FlashAuto(unittest.TestCase):
    def run_auto(self, text, tier=None):
        calls = []
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            esp.flash_auto("/dev/ttyACM0", tier=tier, yes=True, detect=lambda: text,
                           flasher=lambda path, port, chip, off, *a: calls.append((os.path.basename(path), off)))
        return calls, out.getvalue()

    def test_picks_by_chip(self):
        calls, out = self.run_auto(ESPTOOL5_ZERO)
        self.assertEqual(calls, [("skipper-esp32s3-captain-merged.bin", "0x0")])
        self.assertIn("4 MB flash, 2 MB PSRAM: CAPTAIN", out)
        self.assertIn("Embedded PSRAM 2MB", out)   # what esptool said is shown
        calls, _ = self.run_auto(ESPTOOL4_N16R8)
        self.assertEqual(calls[0][0], "skipper-esp32s3-skipper-merged.bin")
        calls, _ = self.run_auto(ESPTOOL5_2MB)
        self.assertEqual(calls[0][0], "skipper-esp32s3-merged.bin")

    def test_forced_tier_skips_detect(self):
        def boom():
            raise AssertionError("detect must not run")
        calls = []
        with contextlib.redirect_stdout(io.StringIO()):
            esp.flash_auto("/dev/ttyACM0", tier="skipper", yes=True, detect=boom,
                           flasher=lambda path, *a: calls.append(os.path.basename(path)))
        self.assertEqual(calls, ["skipper-esp32s3-skipper-merged.bin"])

    def test_dry_run_through_real_flash(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(esp.flash_auto("/dev/ttyACM0", dry_run=True, detect=lambda: ESPTOOL5_ZERO,
                                            python="py"), 0)
        text = out.getvalue()
        self.assertIn("CAPTAIN Edition PRO MAX Turbo Ultra", text)
        self.assertIn("matches", text)
        self.assertIn("DRY RUN", text)

    def test_cli_has_flash_auto(self):
        src = open(esp.__file__).read()
        self.assertIn('"flash-auto"', src)
        info = json.load(open(os.path.join(esp.ESP_PREBUILT_DIR, "build-info.json")))
        self.assertEqual(sorted(info["tiers"]), sorted(esp.TIERS))


if __name__ == "__main__":
    unittest.main()
