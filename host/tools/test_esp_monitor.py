# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""USB log monitor (skipper_esp.monitor) with a fake pyserial. No hardware."""
import unittest

import skipper_esp as esp
from skipper_fw import FwError


class FakeSerialMod:
    def __init__(self, chunks, fail_open=False):
        self.chunks, self.fail_open, self.inst = list(chunks), fail_open, None
        mod = self

        class Serial:
            def __init__(self):
                self.rts_log, self._rts, self.dtr, self.opened, self.closed = [], None, None, False, False
                mod.inst = self

            @property
            def rts(self):
                return self._rts

            @rts.setter
            def rts(self, v):
                self._rts = v
                if self.opened:
                    self.rts_log.append(v)

            def open(self):
                if mod.fail_open:
                    raise OSError("busy")
                self.opened = True

            def read(self, n):
                return mod.chunks.pop(0) if mod.chunks else b""

            def close(self):
                self.closed = True

        self.Serial = Serial


class Monitor(unittest.TestCase):
    def run_mon(self, chunks, **kw):
        t = [0.0]

        def clock():
            t[0] += 0.1
            return t[0]
        lines = []
        fake = FakeSerialMod(chunks, kw.pop("fail_open", False))
        esp.monitor("/dev/ttyACM0", seconds=2, serial_mod=fake, clock=clock, out=lines.append, **kw)
        return fake, lines

    def test_lines_split_and_decoded(self):
        fake, lines = self.run_mon([b"W (1) skipper: hel", b"lo\r\nsecond\n", b"\xffbad\n", b"tail"])
        self.assertIn("W (1) skipper: hello", lines)
        self.assertIn("second", lines)
        self.assertIn("�bad", lines)
        self.assertEqual(lines[-1], "tail")
        self.assertTrue(fake.inst.closed)

    def test_no_reset_by_default_and_lines_released(self):
        fake, _ = self.run_mon([])
        self.assertFalse(fake.inst.dtr)
        self.assertEqual(fake.inst.rts_log, [])

    def test_reset_pulses_rts(self):
        fake, lines = self.run_mon([], reset=True)
        self.assertEqual(fake.inst.rts_log, [True, False])
        self.assertTrue(any("restarting" in ln for ln in lines))

    def test_open_failure(self):
        with self.assertRaises(FwError):
            self.run_mon([], fail_open=True)

    def test_bad_port(self):
        with self.assertRaises(FwError):
            esp.monitor("--x", seconds=1, serial_mod=FakeSerialMod([]))


if __name__ == "__main__":
    unittest.main()
