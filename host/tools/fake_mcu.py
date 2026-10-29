#!/usr/bin/env python3
# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""Fake Klipper MCU on a pseudo-terminal, for testing host code with no hardware.

    python3 tools/fake_mcu.py            # prints e.g. /dev/pts/7
    ./build/mcu_probe /dev/pts/7 250000 --blink PA5

Implements: identify, get_uptime, get_config, allocate_oids,
config_digital_out, finalize_config, update_digital_out (plain Klipper
commands only: the retired segment queue was removed on 2026-10-25).
Sequence numbers and NAKs behave like Klipper's command.c.

Also a simulated printer (for ph::Machine tests), modelled on Klipper's
src/gpiocmds.c, stepper.c, trsync.c, tmcuart.c and sched.c checks:
  * soft PWM outputs (set_digital_out_pwm_cycle, queue_digital_out) with the
    max_duration watchdog ("Missed scheduling of next digital out event",
    "Scheduled digital out event will exceed max_duration") and "Timer too close";
  * a heater on PB8 warms the thermistor on PA3 (simple thermal model; other
    ADC pins read 25 C, PA4 reads open circuit), ADC range checks ("ADC out of
    range");
  * steppers (queue_step, set_next_step_dir, reset_step_clock,
    stepper_get_position) timed against the clock, trsync stop;
  * four TMC2209s on the TMC UART (addresses 0-3), answering register reads
    and writes like the real chip (IFCNT, IOIN version 0x21);
  * emergency_stop.
Options: --tmc-dead ADDR (that driver never answers), --no-heat (the heater
does not warm anything: for the verify_heater test), --no-diag (stallGuard
DIAG never reaches the board: a missing jumper), --ignore-reset (the
"reset" command does nothing, like a board whose reset does not work).

Power (for the connection tests): SIGUSR1 switches the board off / on. While
off it reads as a line held low (a stream of 00 bytes) and answers nothing.
If the host sends bytes to it while it is off, it is "fed through its RX pin"
and comes back HUNG (still only 00 bytes, no answers) until it is switched
off and on again without being fed - a model of the 2026-10-14 SKR Pico
failure, not a measured behaviour. Garbage on the line is handled like
Klipper's src/command.c: bytes are discarded up to the next 0x7e, with one
NAK; leading 0x7e bytes are ignored.

"""
import select
import json, os, pty, signal, sys, time, tty, zlib

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
    "get_uptime": 5, "get_config": 6, "get_clock": 9,
    "allocate_oids count=%c": 7,
    "finalize_config crc=%u": 8,
    "config_digital_out oid=%c pin=%u value=%c default_value=%c max_duration=%u": 12,
    "update_digital_out oid=%c value=%c": 13,
    "reset": 41,
    "config_neopixel oid=%c pin=%u data_size=%hu bit_max_ticks=%u reset_min_ticks=%u": 54,
    "neopixel_update oid=%c pos=%hu data=%*s": 53,
    "neopixel_send oid=%c": 52,
    "config_analog_in oid=%c pin=%u": 32,
    "query_analog_in oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u min_value=%hu max_value=%hu range_check_count=%c": 31,
    "set_digital_out_pwm_cycle oid=%c cycle_ticks=%u": 60,
    "queue_digital_out oid=%c clock=%u on_ticks=%u": 61,
    "config_stepper oid=%c step_pin=%c dir_pin=%c invert_step=%c step_pulse_ticks=%u": 62,
    "queue_step oid=%c interval=%u count=%hu add=%hi": 63,
    "set_next_step_dir oid=%c dir=%c": 64,
    "reset_step_clock oid=%c clock=%u": 65,
    "stepper_get_position oid=%c": 66,
    "stepper_stop_on_trigger oid=%c trsync_oid=%c": 67,
    "config_trsync oid=%c": 68,
    "trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c": 69,
    "trsync_set_timeout oid=%c clock=%u": 70,
    "trsync_trigger oid=%c reason=%c": 71,
    "config_tmcuart oid=%c rx_pin=%u pull_up=%c tx_pin=%u bit_time=%u": 72,
    "tmcuart_send oid=%c write=%*s read=%c": 73,
    "emergency_stop": 74,
    "config_endstop oid=%c pin=%c pull_up=%c": 75,
    "endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c": 76,
    "endstop_query_state oid=%c": 77,
}
RESPONSES = {
    "identify_response offset=%u data=%.*s": 0,
    "uptime high=%u clock=%u": 81,
    "config is_config=%c crc=%u is_shutdown=%c move_count=%hu": 82,
    "shutdown clock=%u static_string_id=%hu": 83,
    "is_shutdown static_string_id=%hu": 84,
    "neopixel_result oid=%c success=%c": 87,
    "clock clock=%u": 88,
    "analog_in_state oid=%c next_clock=%u value=%hu": 89,
    "stepper_position oid=%c pos=%i": 90,
    "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u": 91,
    "tmcuart_response oid=%c read=%*s": 92,
    "endstop_state oid=%c homing=%c next_clock=%u pin_value=%c": 93,
}
STATIC = {"Invalid oid type": 1, "Can not update pin while configured": 2,
          "Can't assign oid": 7,
          "Invalid neopixel data_size": 9, "Invalid neopixel update command": 10,
          "Already finalized": 11, "Missed scheduling of next digital out event": 21,
          "Scheduled digital out event will exceed max_duration": 19, "ADC out of range": 26,
          "Command request": 8, "Can't reset time when stepper active": 22, "Invalid count parameter": 23,
          "Timer too close": 3, "Can not set soft pwm cycle ticks while updates pending": 12}
DICT = zlib.compress(json.dumps({
    "version": "fake-mcu-1", "build_versions": "python",
    "config": {"MCU": "fake_stm32f103", "CLOCK_FREQ": 72000000, "ADC_MAX": 4095},
    "enumerations": {"pin": {"PA0": [0, 16], "PB0": [16, 16], "PC13": 45},
                     "static_string_id": STATIC},
    "commands": COMMANDS, "responses": RESPONSES,
}).encode())
BY_ID = {i: f for f, i in COMMANDS.items()}

import math

FREQ = 72e6
HEATER_PIN, HEATED_ADC_PIN, OPEN_ADC_PIN = 16 + 8, 3, 4    # PB8 heats PA3; PA4 has no sensor


def generic_3950():
    """Klipper thermistor.py Steinhart-Hart fit for 'Generic 3950' (25/100000, 150/1770, 250/230)."""
    pts = [(25., 100000.), (150., 1770.), (250., 230.)]
    inv = [1. / (t + 273.15) for t, r in pts]
    ln = [math.log(r) for t, r in pts]
    ln3 = [v ** 3 for v in ln]
    i12, i13 = inv[0] - inv[1], inv[0] - inv[2]
    l12, l13 = ln[0] - ln[1], ln[0] - ln[2]
    c12, c13 = ln3[0] - ln3[1], ln3[0] - ln3[2]
    c3 = (i12 - i13 * l12 / l13) / (c12 - c13 * l12 / l13)
    c2 = (i12 - c3 * c12) / l12
    c1 = inv[0] - c2 * ln[0] - c3 * ln3[0]
    return c1, c2, c3


C1, C2, C3 = generic_3950()


def adc_fraction(temp_c, pullup=4700.):
    """What a 100k 3950 thermistor on a 4.7k pullup reads at temp_c (0..1)."""
    target = 1. / (temp_c + 273.15)
    lo, hi = -5., 25.                       # ln(R) bounds; 1/T rises with ln(R) (higher R = colder)
    for _ in range(80):
        mid = (lo + hi) / 2
        if C1 + C2 * mid + C3 * mid ** 3 > target:
            hi = mid
        else:
            lo = mid
    r = math.exp((lo + hi) / 2)
    return r / (r + pullup)


def tmc_unbits(raw):
    """Undo Klipper's _add_serial_bits: 10 bits per byte (start, 8 data LSB first, stop)."""
    out = []
    for bit in range(0, len(raw) * 8 - 9, 10):
        b = 0
        for i in range(8):
            p = bit + 1 + i
            if raw[p // 8] & (1 << (p % 8)):
                b |= 1 << i
        out.append(b)
    return out


def tmc_crc8(data):
    crc = 0
    for b in data:
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if ((crc >> 7) ^ (b & 1)) else (crc << 1)
            crc &= 0xff
            b >>= 1
    return crc


def tmc_bits(data):
    out, pos = 0, 0
    for d in data:
        out |= ((d << 1) | 0x200) << pos
        pos += 10
    return bytes((out >> (8 * i)) & 0xff for i in range((pos + 7) // 8))


def extend_clock(c32, clk):
    """A 32-bit clock from the host, as the 64-bit value nearest the current clock (like the MCU's
    wrapping timer_is_before() comparisons)."""
    diff = (c32 - clk) & 0xFFFFFFFF
    if diff >= 1 << 31:
        diff -= 1 << 32
    return clk + diff


class Printer:
    """The simulated printer parts (see the module docstring)."""

    def __init__(self, tmc_dead, no_heat, no_diag=False):
        self.no_diag = no_diag
        self.dout = {}       # oid -> {'pin','default','max_dur','cycle','queue':[(clock,on)],'on':0,'end':None,'duty':0}
        self.steppers = {}   # oid -> {...}
        self.trsync = {}     # oid -> [stepper oids]
        self.tmc = {a: {'regs': {}, 'ifcnt': 0} for a in range(4) if a != tmc_dead}
        self.uarts = set()
        self.temp = 25.0
        self.no_heat = no_heat
        self.last_t = None
        self.endstops = {}   # oid -> {'homing': bool, 'trsync': oid, 'reason': r, 'fire_at': clock}
        self.trsync_timeout = {}   # trsync oid -> expire clock
        self.trsync_events = []    # pending trsync_state messages to send (oid, reason, clock)

    def clock(self, now, t0):
        return int((now - t0) * FREQ)

    def trigger(self, toid, reason, clk):
        for so in self.trsync.get(toid, []):
            s = self.steppers[so]
            s["pos"] = self.stepper_pos(s, clk)
            s["moves"].clear()
            s["need_reset"] = True
            print(f"stepper oid {so} stopped by trsync at position {s['pos']}", flush=True)
        self.trsync[toid] = []
        self.trsync_timeout.pop(toid, None)
        for e in self.endstops.values():
            if e["homing"] and e["trsync"] == toid:
                e["homing"] = False
        self.trsync_events.append((toid, reason, clk))

    def endstop_tick(self, now, t0):
        """Simulated endstops: a homing endstop triggers 0.4 s after homing starts (the axis reached it)."""
        clk = self.clock(now, t0)
        for oid, e in self.endstops.items():
            if e["homing"] and clk >= e["fire_at"]:
                print(f"endstop oid {oid} triggered", flush=True)
                self.trigger(e["trsync"], e["reason"], clk)
        for toid, when in list(self.trsync_timeout.items()):
            if clk >= when:
                print(f"trsync oid {toid} timed out", flush=True)
                self.trigger(toid, 3, clk)

    def heater_duty(self):
        for d in self.dout.values():
            if d["pin"] == HEATER_PIN:
                return d["duty"]
        return 0.0

    def advance(self, now, t0, shut):
        """Apply due PWM events and the max_duration watchdog. Returns a STATIC key on shutdown."""
        clk = self.clock(now, t0)
        for oid, d in self.dout.items():
            while d["queue"] and d["queue"][0][0] <= clk:
                when, on = d["queue"].pop(0)
                d["on"] = on
                d["duty"] = 1.0 if (d["cycle"] == 0 or on >= d["cycle"]) and on else (on / d["cycle"] if d["cycle"] else 0)
                active = (on != 0) != bool(d["default"])
                d["end"] = when + d["max_dur"] if (active and d["max_dur"]) else None
                if d["end"] is not None and d["queue"] and d["queue"][0][0] > d["end"]:
                    return "Scheduled digital out event will exceed max_duration"
                if d["pin"] == HEATER_PIN:
                    print(f"heater pwm {d['duty']:.3f}", flush=True)
            if d["end"] is not None and not d["queue"] and clk > d["end"]:
                return "Missed scheduling of next digital out event"
        if self.last_t is not None and not shut:
            dt = now - self.last_t
            gain = 0.0 if self.no_heat else 4.0
            self.temp += (self.heater_duty() * gain - (self.temp - 25.0) * 0.02) * dt
        self.last_t = now
        return None

    def adc_value(self, pin, count):
        full = count * 4095
        if pin == OPEN_ADC_PIN:
            return full
        t = self.temp if pin == HEATED_ADC_PIN else 25.0
        return int(round(adc_fraction(t) * full))

    def shutdown_outputs(self):
        for d in self.dout.values():
            d["queue"].clear()
            d["duty"] = 0.0
            d["end"] = None
        for s in self.steppers.values():
            s["moves"].clear()

    # ---- stallGuard (simplified: one motor moves at a time; 16 microsteps assumed)
    def moving_interval(self, clk):
        for s in self.steppers.values():
            for m in s["moves"]:
                if m["start"] <= clk < m["start"] + m["interval"] * m["count"]:
                    return m["interval"]
        return None

    def tstep(self, clk):
        iv = self.moving_interval(clk)
        return 0xFFFFF if iv is None else int(12e6 * (iv / FREQ) / 16)

    def sg_result(self, clk):
        """Free-running: around 260; standing still: 0 (no back-EMF)."""
        if self.moving_interval(clk) is None:
            return 0
        return 240 + (clk // 1000) % 41

    def diag(self, clk):
        """DIAG high when SG_RESULT <= 2 * SGTHRS for a driver with SGTHRS set (TMC2209 rule)."""
        if self.no_diag:
            return False
        sg = self.sg_result(clk)
        return any(c["regs"].get(0x40, 0) and sg <= 2 * c["regs"].get(0x40, 0) for c in self.tmc.values())

    # ---- steppers
    def stepper_pos(self, s, clk):
        """Steps done so far: the position the real firmware would report."""
        pos = s["pos"]
        for m in s["moves"]:
            if m["start"] >= clk:
                break
            done = min(m["count"], max(0, (clk - m["start"]) // m["interval"]))
            pos += done if m["dir"] else -done
        return pos

    def settle(self, s, clk):
        """Fold finished moves into the position."""
        while s["moves"] and s["moves"][0]["start"] + s["moves"][0]["interval"] * s["moves"][0]["count"] <= clk:
            m = s["moves"].pop(0)
            s["pos"] += m["count"] if m["dir"] else -m["count"]

    def command(self, name, a, now, t0, out):
        """Handle a printer command. Returns a STATIC key to shut down, else None."""
        clk = self.clock(now, t0)
        if name == "config_digital_out":
            self.dout[a["oid"]] = {"pin": a["pin"], "default": a["default_value"], "max_dur": a["max_duration"],
                                   "cycle": 0, "queue": [], "on": a["value"], "end": None, "duty": 0.0}
            print(f"config oid {a['oid']} -> pin {a['pin']} max_duration {a['max_duration']}", flush=True)
        elif name == "set_digital_out_pwm_cycle":
            d = self.dout.get(a["oid"])
            if d is None:
                return "Invalid oid type"
            if d["queue"]:
                return "Can not set soft pwm cycle ticks while updates pending"
            d["cycle"] = a["cycle_ticks"]
        elif name == "queue_digital_out":
            d = self.dout.get(a["oid"])
            if d is None:
                return "Invalid oid type"
            when = extend_clock(a["clock"], clk)
            if when - clk < -int(FREQ * 0.001) and not d["queue"]:
                print(f"queue_digital_out oid {a['oid']} at {when} is {(clk - when) / FREQ * 1e3:.1f} ms in the past", flush=True)
                return "Timer too close"
            if d["end"] is not None and not d["queue"] and when > d["end"]:
                return "Scheduled digital out event will exceed max_duration"
            d["queue"].append((when, a["on_ticks"]))
        elif name == "config_stepper":
            self.steppers[a["oid"]] = {"pos": 0, "moves": [], "next_dir": 0, "last": None, "need_reset": True}
        elif name == "reset_step_clock":
            s = self.steppers[a["oid"]]
            self.settle(s, clk)
            if s["moves"]:
                return "Can't reset time when stepper active"
            s["last"] = extend_clock(a["clock"], clk)
            s["need_reset"] = False
        elif name == "set_next_step_dir":
            self.steppers[a["oid"]]["next_dir"] = a["dir"]
        elif name == "queue_step":
            s = self.steppers[a["oid"]]
            if not a["count"]:
                return "Invalid count parameter"
            if s["need_reset"]:
                return None                      # firmware drops moves until reset_step_clock
            self.settle(s, clk)
            start = s["last"]
            first = start + a["interval"]
            if not s["moves"] and first < clk - int(FREQ * 0.001):
                print(f"queue_step oid {a['oid']} first step {(clk - first) / FREQ * 1e3:.1f} ms in the past", flush=True)
                return "Timer too close"
            s["moves"].append({"start": start, "interval": a["interval"], "count": a["count"], "dir": s["next_dir"]})
            s["last"] = start + a["interval"] * a["count"]
        elif name == "stepper_get_position":
            s = self.steppers[a["oid"]]
            self.settle(s, clk)
            out += enc(90) + enc(a["oid"]) + enc(self.stepper_pos(s, clk))
        elif name == "config_trsync":
            self.trsync[a["oid"]] = []
        elif name == "trsync_start":
            self.trsync[a["oid"]] = []
            self.trsync_timeout.pop(a["oid"], None)
        elif name == "stepper_stop_on_trigger":
            self.trsync.setdefault(a["trsync_oid"], []).append(a["oid"])
        elif name == "trsync_trigger":
            self.trigger(a["oid"], a["reason"], clk)
            self.trsync_events.pop()   # answered directly below
            out += enc(91) + enc(a["oid"]) + enc(0) + enc(a["reason"]) + enc(clk & 0xFFFFFFFF)
        elif name == "config_endstop":
            self.endstops[a["oid"]] = {"homing": False, "trsync": 0, "reason": 0, "fire_at": 0, "pin": a["pin"]}
        elif name == "endstop_home":
            e = self.endstops.get(a["oid"])
            if e is None:
                return "Invalid oid type"
            if a["sample_count"] == 0:
                e["homing"] = False
            else:
                start = extend_clock(a["clock"], clk)
                e.update(homing=True, trsync=a["trsync_oid"], reason=a["trigger_reason"],
                         fire_at=start + int(0.4 * FREQ))
                print(f"endstop oid {a['oid']} homing", flush=True)
        elif name == "endstop_query_state":
            e = self.endstops.get(a["oid"], {"homing": False})
            out += enc(93) + enc(a["oid"]) + enc(int(e["homing"])) + enc(clk & 0xFFFFFFFF) + enc(int(self.diag(clk)))
        elif name == "config_tmcuart":
            self.uarts.add(a["oid"])
            print(f"config_tmcuart oid {a['oid']} rx {a['rx_pin']} tx {a['tx_pin']} bit_time {a['bit_time']}", flush=True)
        elif name == "tmcuart_send":
            d = tmc_unbits(bytes(a["write"]))
            reply = b""
            if len(d) >= 4 and d[1] in self.tmc and tmc_crc8(d[:-1]) == d[-1]:
                chip = self.tmc[d[1]]
                if len(d) == 8:
                    chip["regs"][d[2] & 0x7f] = (d[3] << 24) | (d[4] << 16) | (d[5] << 8) | d[6]
                    chip["ifcnt"] = (chip["ifcnt"] + 1) & 0xff
                elif len(d) == 4 and a["read"] == 10:
                    reg = d[2]
                    val = {0x06: 0x21000040, 0x02: chip["ifcnt"], 0x6f: 0}.get(reg, chip["regs"].get(reg, 0))
                    if reg == 0x12:   # TSTEP
                        val = self.tstep(clk)
                    elif reg == 0x41:   # SG_RESULT
                        val = self.sg_result(clk)
                    msg = [0x05, 0xff, reg, (val >> 24) & 0xff, (val >> 16) & 0xff, (val >> 8) & 0xff, val & 0xff]
                    reply = tmc_bits(msg + [tmc_crc8(msg)])
            out += enc(92) + enc(a["oid"]) + [len(reply)] + list(reply)
        else:
            return "unknown"
        return None


def main():
    args = sys.argv[1:]
    tmc_dead, no_heat = -1, False
    if "--tmc-dead" in args:
        i = args.index("--tmc-dead")
        tmc_dead = int(args[i + 1])
        del args[i:i + 2]
    if "--no-heat" in args:
        args.remove("--no-heat")
        no_heat = True
    no_diag = "--no-diag" in args
    if no_diag:
        args.remove("--no-diag")
    ignore_reset = "--ignore-reset" in args
    if ignore_reset:
        args.remove("--ignore-reset")
    start_seq = int(args[0]) if args else 0
    printer = Printer(tmc_dead, no_heat, no_diag)
    master, slave = pty.openpty()
    tty.setraw(slave)
    print(os.ttyname(slave), flush=True)
    next_seq, buf, t0 = start_seq, b"", time.time()
    configured, pins, shutdown_id = False, {}, 0
    config_crc = 0
    neo = {}    # oid -> {'pin', 'size', 'buf', 'shown'}
    adc = {}    # oid -> {'pin', 'rest_s', 'count', 'due'}   (query_analog_in running)
    adc_pins = {}   # oid -> pin number from config_analog_in
    PA4 = 4         # simulated: PA4 has nothing connected (reads open circuit)
    power = {"on": True, "fed": False, "hung": False, "toggle": False, "zeros_at": 0.0}
    need_sync = [False]

    def on_usr1(signum, frame):
        power["toggle"] = True
    signal.signal(signal.SIGUSR1, on_usr1)

    def boot():
        nonlocal configured, pins, shutdown_id, neo, config_crc, adc, adc_pins, printer, next_seq, buf, t0
        configured, pins, shutdown_id = False, {}, 0
        neo, config_crc, adc, adc_pins = {}, 0, {}, {}
        printer = Printer(tmc_dead, no_heat, no_diag)
        next_seq, buf, t0 = 0, b"", time.time()
        need_sync[0] = False

    def send(content):
        blk = bytes([len(content) + 5, 0x10 | next_seq]) + bytes(content)
        c = crc16(blk)
        os.write(master, blk + bytes([c >> 8, c & 0xFF, 0x7E]))

    while True:
        if power["toggle"]:
            power["toggle"] = False
            if power["on"]:
                power["on"], power["fed"] = False, False
                print("POWER OFF", flush=True)
            else:
                power["on"] = True
                power["hung"] = power["fed"]
                boot()
                print("POWER ON, HUNG (it was fed through its RX pin while off)" if power["hung"]
                      else "POWER ON (fresh start)", flush=True)
        if not power["on"] or power["hung"]:
            # a line held low: the host's UART reads 00 bytes; anything sent here is lost
            if time.time() >= power["zeros_at"]:
                power["zeros_at"] = time.time() + 0.05
                os.write(master, bytes(16))
            r, _, _ = select.select([master], [], [], 0.02)
            if r:
                got = os.read(master, 256)
                if got and not power["on"] and not power["fed"]:
                    power["fed"] = True
                    print("FED THROUGH RX while off", flush=True)
            continue
        # Simulated ADC reports, sent on the board's own schedule like the real firmware does.
        now = time.time()
        if not shutdown_id:
            why = printer.advance(now, t0, False)
            if why:
                shutdown_id = STATIC[why]
                printer.shutdown_outputs()
                send(enc(83) + enc(printer.clock(now, t0) & 0xFFFFFFFF) + enc(shutdown_id))
                print("SHUTDOWN: " + why, flush=True)
        else:
            printer.advance(now, t0, True)
        if not shutdown_id:
            printer.endstop_tick(now, t0)
            while printer.trsync_events:
                toid, reason, c = printer.trsync_events.pop(0)
                send(enc(91) + enc(toid) + enc(0) + enc(reason) + enc(c & 0xFFFFFFFF))
        for oid, q in adc.items():
            if shutdown_id:
                break
            if now >= q["due"]:
                q["due"] = now + q["rest_s"]
                # PA4 reads open circuit; PA3 follows the simulated heater; others 25 C (100k 3950 on 4.7k)
                value = printer.adc_value(adc_pins.get(oid), q["count"])
                if value < q["min"] or value > q["max"]:
                    q["bad"] += 1
                    if q["bad"] >= q["range_count"]:
                        shutdown_id = STATIC["ADC out of range"]
                        printer.shutdown_outputs()
                        send(enc(83) + enc(0) + enc(shutdown_id))
                        print("SHUTDOWN: ADC out of range", flush=True)
                        break
                else:
                    q["bad"] = 0
                # like the firmware: sent ~8 ms after the samples started; next_clock = next start
                nxt = int((now - t0 - 0.008 + q["rest_s"]) * 72e6) & 0xFFFFFFFF
                send(enc(89) + enc(oid) + enc(nxt) + enc(value))
        ready, _, _ = select.select([master], [], [], 0.05)
        if not ready:
            continue
        buf += os.read(master, 256)
        while buf:
            # Klipper src/command.c command_find_block(): resync on garbage, ignore leading SYNC bytes
            if need_sync[0]:
                i = buf.find(b"\x7e")
                if i < 0:
                    buf = b""
                    break
                buf, need_sync[0] = buf[i + 1:], False
                continue
            if len(buf) < 5:
                break
            n = buf[0]
            if n < 5 or n > 64 or (buf[1] & 0xF0) != 0x10:
                if buf[0] == 0x7E:
                    buf = buf[1:]
                    continue
                print("garbage: resync", flush=True)
                need_sync[0] = True
                send([])
                continue
            if len(buf) < n:
                break
            blk, buf = buf[:n], buf[n:]
            c = crc16(blk[:-3])
            if blk[-3:] != bytes([c >> 8, c & 0xFF, 0x7E]):
                print("bad crc", flush=True)
                need_sync[0] = True
                send([])
                continue
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
                    if ignore_reset:
                        print("RESET IGNORED", flush=True)
                        continue
                    print("RESET (reboot)", flush=True)
                    boot()
                    # a real board is gone for a while: what arrives meanwhile is lost (else the rest
                    # of a blind reset could reset the freshly booted fake a second time)
                    time.sleep(0.05)
                    while select.select([master], [], [], 0)[0]:
                        try:
                            if not os.read(master, 4096):
                                break
                        except OSError:
                            break
                    rebooted = True
                    break
                if name == "identify":
                    off, cnt = args["offset"], args["count"]
                    chunk = DICT[off:off+cnt]
                    out += enc(0) + enc(off) + [len(chunk)] + list(chunk)
                elif name == "get_clock":
                    clk = int((time.time() - t0) * 72e6)
                    out += enc(88) + enc(clk & 0xFFFFFFFF)
                elif name == "get_uptime":
                    clk = int((time.time() - t0) * 72e6)
                    out += enc(81) + enc(clk >> 32) + enc(clk & 0xFFFFFFFF)
                elif name == "get_config":
                    # like Klipper: the move queue exists (and is reported) only once configured
                    out += enc(82) + enc(int(configured)) + enc(config_crc) + enc(int(bool(shutdown_id))) + enc(1024 if configured else 0)
                elif name == "finalize_config":
                    if configured:
                        shutdown_id = STATIC["Already finalized"]
                        out += enc(83) + enc(0) + enc(shutdown_id)
                        print("SHUTDOWN: Already finalized", flush=True)
                        continue
                    configured, config_crc = True, args["crc"] & 0xFFFFFFFF
                    print(f"configured, crc {config_crc}", flush=True)
                elif name == "allocate_oids":
                    print(f"allocate_oids {args['count']}", flush=True)
                elif name == "config_neopixel":
                    if args["data_size"] < 1 or args["data_size"] > 1024:
                        shutdown_id = STATIC["Invalid neopixel data_size"]
                        out += enc(83) + enc(0) + enc(shutdown_id)
                        continue
                    neo[args["oid"]] = {"pin": args["pin"], "size": args["data_size"],
                                        "buf": bytearray(args["data_size"]), "shown": None}
                    print(f"config_neopixel oid {args['oid']} pin {args['pin']} size {args['data_size']} "
                          f"bit_max {args['bit_max_ticks']} reset_min {args['reset_min_ticks']}", flush=True)
                elif name == "neopixel_update":
                    n = neo.get(args["oid"])
                    d = bytes(args["data"])
                    if n is None or args["pos"] + len(d) > n["size"]:
                        shutdown_id = STATIC["Invalid neopixel update command"]
                        out += enc(83) + enc(0) + enc(shutdown_id)
                        print("SHUTDOWN: Invalid neopixel update command", flush=True)
                        continue
                    n["buf"][args["pos"]:args["pos"] + len(d)] = d
                elif name == "neopixel_send":
                    n = neo.get(args["oid"])
                    ok = n is not None
                    if ok:
                        n["shown"] = bytes(n["buf"])
                        print(f"neopixel oid {args['oid']} shows {n['shown'].hex()}", flush=True)
                    out += enc(87) + enc(args["oid"]) + enc(int(ok))
                elif name == "config_analog_in":
                    adc_pins[args["oid"]] = args["pin"]
                    print(f"config_analog_in oid {args['oid']} pin {args['pin']}", flush=True)
                elif name == "query_analog_in":
                    if args["oid"] not in adc_pins:
                        shutdown_id = STATIC["Invalid oid type"]
                        out += enc(83) + enc(0) + enc(shutdown_id)
                        print("SHUTDOWN: Invalid oid type", flush=True)
                        continue
                    adc[args["oid"]] = {"rest_s": args["rest_ticks"] / 72e6, "count": args["sample_count"],
                                        "due": time.time() + 0.05, "min": args["min_value"],
                                        "max": args["max_value"], "range_count": args["range_check_count"] or 1,
                                        "bad": 0}
                    print(f"query_analog_in oid {args['oid']} every {args['rest_ticks'] / 72e6:.3f} s, "
                          f"{args['sample_count']} samples, range {args['min_value']}..{args['max_value']}", flush=True)
                elif name == "emergency_stop":
                    shutdown_id = STATIC["Command request"]
                    printer.shutdown_outputs()
                    out += enc(83) + enc(0) + enc(shutdown_id)
                    print("SHUTDOWN: Command request (emergency_stop)", flush=True)
                    continue
                elif name == "config_digital_out" or name in (
                        "set_digital_out_pwm_cycle", "queue_digital_out", "config_stepper", "queue_step",
                        "set_next_step_dir", "reset_step_clock", "stepper_get_position", "stepper_stop_on_trigger",
                        "config_trsync", "trsync_start", "trsync_set_timeout", "trsync_trigger", "config_tmcuart",
                        "tmcuart_send", "config_endstop", "endstop_home", "endstop_query_state"):
                    if name == "config_digital_out":
                        pins[args["oid"]] = args["pin"]
                    if name == "trsync_set_timeout":
                        printer.trsync_timeout[args["oid"]] = extend_clock(args["clock"], printer.clock(time.time(), t0))
                        continue
                    why = printer.command(name, args, time.time(), t0, out)
                    if why:
                        shutdown_id = STATIC.get(why, STATIC["Invalid oid type"])
                        printer.shutdown_outputs()
                        out += enc(83) + enc(0) + enc(shutdown_id)
                        print("SHUTDOWN: " + why, flush=True)
                        continue
                elif name == "update_digital_out":
                    if args["oid"] not in pins:
                        shutdown_id = STATIC["Invalid oid type"]
                        out += enc(83) + enc(0) + enc(shutdown_id)
                        print("SHUTDOWN: Invalid oid type", flush=True)
                        continue
                    print(f"pin {pins.get(args['oid'])} = {args['value']}", flush=True)
            if not rebooted:
                send(out)

if __name__ == "__main__":
    main()
