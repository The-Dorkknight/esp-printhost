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


class FakeMcuPlainKlipper(unittest.TestCase):
    """The simulated board speaks plain Klipper only (the segment queue was removed 2026-10-25)."""
    def setUp(self):
        self.c = Client()

    def tearDown(self):
        self.c.close()

    def test_dictionary_has_no_segment_queue(self):
        d = self.c.dict
        self.assertFalse([f for f in list(d["commands"]) + list(d["responses"]) if "segq" in f])
        self.assertFalse([k for k in d["config"] if k.startswith("SEGQ")])
        self.assertIn("queue_step oid=%c interval=%u count=%hu add=%hi", d["commands"])

    def test_config_round_trip(self):
        c = self.c
        c.cmd("allocate_oids count=%c", 1)
        c.cmd("config_digital_out oid=%c pin=%u value=%c default_value=%c max_duration=%u", 0, 13, 0, 0, 0)
        c.cmd("finalize_config crc=%u", 1234)
        r = c.cmd("get_config")
        self.assertEqual(r[0][0], "config")
        self.assertEqual(r[0][1]["is_config"], 1)
        self.assertEqual(r[0][1]["crc"], 1234)


if __name__ == "__main__":
    unittest.main(verbosity=1)
