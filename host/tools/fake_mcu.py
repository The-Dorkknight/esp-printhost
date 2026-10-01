#!/usr/bin/env python3
# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""Fake Klipper MCU on a pseudo-terminal, for testing host code with no hardware.

    python3 tools/fake_mcu.py            # prints e.g. /dev/pts/7
    ./build/mcu_probe /dev/pts/7 250000 --blink PA5

Implements: identify, get_uptime, get_config, allocate_oids,
config_digital_out, finalize_config, update_digital_out, segq_push (logs).
Sequence numbers and NAKs behave like Klipper's command.c.
"""
import json, os, pty, sys, time, tty, zlib

def crc16(buf):
    crc = 0xFFFF
    for d in buf:
        d ^= crc & 0xFF; d ^= (d & 0x0F) << 4; d &= 0xFF
        crc = (((d << 8) | (crc >> 8)) ^ (d >> 4) ^ (d << 3)) & 0xFFFF
    return crc

def enc(v):
    v &= 0xFFFFFFFF
    if v & 0x80000000: v -= 1 << 32
    o = []
    if v >= 0xC000000 or v < -0x4000000: o.append((v >> 28) & 0x7F | 0x80)
    if v >= 0x180000 or v < -0x80000: o.append((v >> 21) & 0x7F | 0x80)
    if v >= 0x3000 or v < -0x1000: o.append((v >> 14) & 0x7F | 0x80)
    if v >= 0x60 or v < -0x20: o.append((v >> 7) & 0x7F | 0x80)
    o.append(v & 0x7F); return o

def dec(b, p):
    c = b[p]; p += 1; v = c & 0x7F
    if (c & 0x60) == 0x60: v |= -0x20
    while c & 0x80:
        c = b[p]; p += 1; v = (v << 7) | (c & 0x7F)
    return v, p

COMMANDS = {
    "identify offset=%u count=%c": 1,
    "get_uptime": 5, "get_config": 6,
    "allocate_oids count=%c": 7,
    "finalize_config crc=%u": 8,
    "config_digital_out oid=%c pin=%u value=%c default_value=%c max_duration=%u": 12,
    "update_digital_out oid=%c value=%c": 13,
    "segq_push oid=%c ticks=%u x=%i y=%i z=%i e=%i flags=%c tag=%hu": 40,
    "reset": 41,
}
RESPONSES = {
    "identify_response offset=%u data=%.*s": 0,
    "uptime high=%u clock=%u": 81,
    "config is_config=%c crc=%u is_shutdown=%c move_count=%hu": 82,
    "shutdown clock=%u static_string_id=%hu": 83,
    "is_shutdown static_string_id=%hu": 84,
}
STATIC = {"Invalid oid type": 1, "Can not update pin while configured": 2}
DICT = zlib.compress(json.dumps({
    "version": "fake-mcu-1", "build_versions": "python",
    "config": {"MCU": "fake_stm32f103", "CLOCK_FREQ": 72000000},
    "enumerations": {"pin": {"PA0": [0, 16], "PB0": [16, 16], "PC13": 45},
                     "static_string_id": STATIC},
    "commands": COMMANDS, "responses": RESPONSES,
}).encode())
BY_ID = {i: f for f, i in COMMANDS.items()}

def main():
    start_seq = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    master, slave = pty.openpty()
    tty.setraw(slave)
    print(os.ttyname(slave), flush=True)
    next_seq, buf, t0 = start_seq, b"", time.time()
    configured, pins, shutdown_id = False, {}, 0

    def send(content):
        blk = bytes([len(content) + 5, 0x10 | next_seq]) + bytes(content)
        c = crc16(blk)
        os.write(master, blk + bytes([c >> 8, c & 0xFF, 0x7E]))

    while True:
        buf += os.read(master, 256)
        while len(buf) >= 5:
            n = buf[0]
            if n < 5 or n > 64 or len(buf) < n: break
            blk, buf = buf[:n], buf[n:]
            c = crc16(blk[:-3])
            if blk[-3:] != bytes([c >> 8, c & 0xFF, 0x7E]):
                print("bad crc", flush=True); continue
            seq = blk[1] & 0x0F
            if seq != next_seq:
                print(f"NAK got seq {seq}, want {next_seq}", flush=True)
                send([]); continue
            next_seq = (next_seq + 1) & 0x0F
            data, p, out, rebooted = blk[2:-3], 0, [], False
            while p < len(data):
                mid, p = dec(data, p)
                fmt = BY_ID[mid]; name = fmt.split()[0]; args = {}
                for a in fmt.split()[1:]:
                    k, t = a.split("=")
                    if t in ("%s", "%.*s", "%*s"):
                        l = data[p]; args[k] = data[p+1:p+1+l]; p += 1 + l
                    else:
                        args[k], p = dec(data, p)
                if shutdown_id and name not in ("identify", "get_config", "get_uptime", "reset"):
                    out += enc(84) + enc(shutdown_id)
                    continue
                if name == "reset":
                    print("RESET (reboot)", flush=True)
                    configured, pins, shutdown_id = False, {}, 0
                    next_seq, buf, rebooted = 0, b"", True
                    break
                if name == "identify":
                    off, cnt = args["offset"], args["count"]
                    chunk = DICT[off:off+cnt]
                    out += enc(0) + enc(off) + [len(chunk)] + list(chunk)
                elif name == "get_uptime":
                    clk = int((time.time() - t0) * 72e6)
                    out += enc(81) + enc(clk >> 32) + enc(clk & 0xFFFFFFFF)
                elif name == "get_config":
                    out += enc(82) + enc(int(configured)) + enc(0) + enc(int(bool(shutdown_id))) + enc(0)
                elif name == "finalize_config":
                    configured = True
                elif name == "config_digital_out":
                    pins[args["oid"]] = args["pin"]
                    print(f"config oid {args['oid']} -> pin {args['pin']}", flush=True)
                elif name == "update_digital_out":
                    if args["oid"] not in pins:
                        shutdown_id = STATIC["Invalid oid type"]
                        out += enc(83) + enc(0) + enc(shutdown_id)
                        print("SHUTDOWN: Invalid oid type", flush=True)
                        continue
                    print(f"pin {pins.get(args['oid'])} = {args['value']}", flush=True)
                elif name == "segq_push":
                    print("segment", args, flush=True)
            if not rebooted:
                send(out)

if __name__ == "__main__":
    main()
