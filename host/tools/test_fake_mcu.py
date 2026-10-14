#!/usr/bin/env python3
# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""Talks protocol v1 to tools/fake_mcu.py over its pseudo-terminal. No hardware.

    python3 tools/test_fake_mcu.py
"""
import json
import os
import select
import subprocess
import sys
import unittest
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fake_mcu as fm

HERE = os.path.dirname(os.path.abspath(__file__))


class Client:
    def __init__(self):
        self.p = subprocess.Popen([sys.executable, "-u", os.path.join(HERE, "fake_mcu.py")],
                                  stdout=subprocess.PIPE, text=True)
        self.dev = self.p.stdout.readline().strip()
        self.fd = os.open(self.dev, os.O_RDWR | os.O_NOCTTY)
        import tty
        tty.setraw(self.fd)
        self.seq = 0
        self.buf = b""
        self.dict = None
        self.dict = self.identify()

    def close(self):
        self.p.terminate()
        self.p.wait()
        self.p.stdout.close()
        os.close(self.fd)

    def block(self, content):
        blk = bytes([len(content) + 5, 0x10 | self.seq]) + bytes(content)
        c = fm.crc16(blk)
        os.write(self.fd, blk + bytes([c >> 8, c & 0xFF, 0x7E]))
        self.seq = (self.seq + 1) & 0xF
        return self.read_block()

    def read_block(self):
        while True:
            if len(self.buf) >= 5 and len(self.buf) >= self.buf[0]:
                n = self.buf[0]
                blk, self.buf = self.buf[:n], self.buf[n:]
                return blk[2:-3]
            r, _, _ = select.select([self.fd], [], [], 5)
            assert r, "timeout waiting for fake MCU"
            self.buf += os.read(self.fd, 256)

    def identify(self):
        data = b""
        while True:
            c = self.block(fm.enc(1) + fm.enc(len(data)) + fm.enc(40))
            mid, p = fm.dec(c, 0)
            off, p = fm.dec(c, p)
            ln = c[p]
            data += c[p + 1:p + 1 + ln]
            if ln < 40:
                return json.loads(zlib.decompress(data))

    def cmd(self, fmt, *vals):
        mid = self.dict["commands"][fmt]
        c = fm.enc(mid)
        for v in vals:
            c += fm.enc(v)
        return self.parse(self.block(c))

    def parse(self, data):
        by_id = {i: f for f, i in self.dict["responses"].items()}
        res, p = [], 0
        while p < len(data):
            mid, p = fm.dec(data, p)
            fmt = by_id[mid]
            vals = {}
            for a in fmt.split()[1:]:
                k, t = a.split("=")
                vals[k], p = fm.dec(data, p)
            res.append((fmt.split()[0], vals))
        return res


class FakeMcuSegq(unittest.TestCase):
    def setUp(self):
        self.c = Client()

    def tearDown(self):
        self.c.close()

    def test_dictionary_has_protocol_v1(self):
        d = self.c.dict
        for f in ("config_segq oid=%c period=%u depth=%c start_threshold=%c",
                  "config_segq_axis oid=%c axis=%c step_pin=%u dir_pin=%u invert_step=%c invert_dir=%c",
                  "segq_push oid=%c n=%u x=%i y=%i z=%i e=%i flags=%c tag=%hu",
                  "segq_stop oid=%c", "segq_query oid=%c"):
            self.assertIn(f, d["commands"])
        for f in ("segq_status oid=%c free=%c active=%c tag=%hu underruns=%hu",
                  "segq_position oid=%c x=%i y=%i z=%i e=%i"):
            self.assertIn(f, d["responses"])
        self.assertEqual(d["config"]["SEGQ_PROTOCOL"], 1)

    def test_push_and_query_sums_positions(self):
        c = self.c
        c.cmd("config_segq oid=%c period=%u depth=%c start_threshold=%c", 2, 600, 64, 4)
        c.cmd("config_segq_axis oid=%c axis=%c step_pin=%u dir_pin=%u invert_step=%c invert_dir=%c", 2, 0, 11, 10, 0, 1)
        c.cmd("segq_push oid=%c n=%u x=%i y=%i z=%i e=%i flags=%c tag=%hu", 2, 100, 50, -3, 0, -40, 0, 7)
        c.cmd("segq_push oid=%c n=%u x=%i y=%i z=%i e=%i flags=%c tag=%hu", 2, 1 << 30, 5, 0, 0, 0, 2, 8)
        r = c.cmd("segq_query oid=%c", 2)
        names = [n for n, _ in r]
        self.assertEqual(names, ["segq_status", "segq_position"])
        self.assertEqual(r[0][1], {"oid": 2, "free": 64, "active": 0, "tag": 8, "underruns": 0})
        self.assertEqual(r[1][1], {"oid": 2, "x": 55, "y": -3, "z": 0, "e": -40})

    def test_invalid_segment_shuts_down(self):
        c = self.c
        c.cmd("config_segq oid=%c period=%u depth=%c start_threshold=%c", 0, 600, 8, 1)
        r = c.cmd("segq_push oid=%c n=%u x=%i y=%i z=%i e=%i flags=%c tag=%hu", 0, 10, 6, 0, 0, 0, 0, 1)  # 2*6 > 10
        self.assertEqual(r[0][0], "shutdown")
        r = c.cmd("segq_query oid=%c", 0)
        self.assertEqual(r[0][0], "is_shutdown")

    def test_push_without_config_and_bad_config(self):
        r = self.c.cmd("segq_push oid=%c n=%u x=%i y=%i z=%i e=%i flags=%c tag=%hu", 1, 10, 0, 0, 0, 0, 0, 0)
        self.assertEqual(r[0][0], "shutdown")

    def test_bad_period_and_duplicate_oid(self):
        for period in (0, (1 << 30) + 1, 0x80000000):
            c = Client()
            try:
                r = c.cmd("config_segq oid=%c period=%u depth=%c start_threshold=%c", 0, period, 8, 1)
                self.assertEqual(r[0][0], "shutdown", period)
            finally:
                c.close()
        c = Client()
        try:
            c.cmd("config_segq oid=%c period=%u depth=%c start_threshold=%c", 0, 1 << 30, 8, 1)
            r = c.cmd("config_segq oid=%c period=%u depth=%c start_threshold=%c", 0, 600, 8, 1)
            self.assertEqual(r[0][0], "shutdown")
        finally:
            c.close()

    def test_zero_n_and_too_long(self):
        for n in (0, (1 << 30) + 1):
            c = Client()
            try:
                c.cmd("config_segq oid=%c period=%u depth=%c start_threshold=%c", 0, 600, 8, 1)
                r = c.cmd("segq_push oid=%c n=%u x=%i y=%i z=%i e=%i flags=%c tag=%hu", 0, n, 0, 0, 0, 0, 0, 0)
                self.assertEqual(r[0][0], "shutdown", n)
            finally:
                c.close()


if __name__ == "__main__":
    unittest.main(verbosity=1)
