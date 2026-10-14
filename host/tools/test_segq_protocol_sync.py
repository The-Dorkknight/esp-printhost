# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""Keeps firmware/skipper/segq.c, docs/PROTOCOL.md and fake_mcu.py describing the same protocol."""
import os
import re
import unittest

import fake_mcu as fm

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def read(*p):
    with open(os.path.join(ROOT, *p), encoding="utf-8") as f:
        return f.read()


def c_strings(expr):
    return "".join(re.findall(r'"((?:[^"\\]|\\.)*)"', expr))


class SegqProtocolSync(unittest.TestCase):
    def setUp(self):
        self.c = read("firmware", "skipper", "segq.c")

    def commands(self):
        return {c_strings(m) for m in re.findall(r"DECL_COMMAND\(\w+,\s*((?:\s*\"[^\"]*\")+)\s*\)", self.c)}

    def responses(self):
        return {c_strings(m) for m in re.findall(r"sendf\(\s*((?:\s*\"[^\"]*\")+)\s*,", self.c)}

    def test_firmware_has_expected_messages(self):
        self.assertEqual(len(self.commands()), 5)
        self.assertEqual(len(self.responses()), 2)

    def test_fake_mcu_matches_firmware(self):
        self.assertEqual({f for f in fm.COMMANDS if "segq" in f}, self.commands())
        self.assertEqual({f for f in fm.RESPONSES if "segq" in f}, self.responses())
        shut = set(re.findall(r'shutdown\("([^"]*)"\)', self.c))
        self.assertTrue(shut)
        for msg in shut:
            self.assertIn(msg, fm.STATIC)

    def test_protocol_md_matches_firmware(self):
        doc = read("docs", "PROTOCOL.md")
        blocks = re.findall(r"```\n(.*?)```", doc, re.S)
        lines = {l.strip() for b in blocks for l in b.splitlines() if l.strip()}
        for f in self.commands() | self.responses():
            self.assertIn(f, lines, f)

    def test_constants_match(self):
        self.assertEqual(int(re.search(r"SEGQ_MAX_DEPTH (\d+)", self.c).group(1)), fm.SEGQ_MAX_DEPTH)
        self.assertEqual(int(re.search(r"SEGQ_PROTOCOL_VERSION (\d+)", self.c).group(1)), 1)


if __name__ == "__main__":
    unittest.main()
