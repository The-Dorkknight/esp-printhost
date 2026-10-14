#!/usr/bin/env python3
# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""Research helper: run Klipper's own TMC2209 code (klippy/extras/tmc2209.py, tmc_uart.py) with
mock objects and print the register values and UART datagrams it would send. The C++ tests in
host/test/test_machine.cpp compare ph::tmc2209_registers() and ph::tmc_encode_*() with these.

Needs a Klipper checkout (the Flasher downloads v0.13.0 to ~/.skipper-flipper/klipper-v0.13.0):
    python3 host/tools/klipper_tmc_reference.py [path/to/klipper]
Output from 2026-10-13 (Klipper v0.13.0) is recorded in docs/research/klipper-sources.md.
"""
import sys, types
from unittest import mock
sys.modules['serial'] = types.ModuleType('serial')
import os
KLIPPER = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/.skipper-flipper/klipper-v0.13.0')
sys.path.insert(0, os.path.join(KLIPPER, 'klippy'))
import extras.tmc2209 as t9, extras.tmc_uart as tu

class Cfg:
    def __init__(self, name, vals, sections):
        self.name, self.vals, self.sections = name, vals, sections
        self.printer = mock.MagicMock()
        self.printer.get_start_args.return_value = {}
        pins = mock.MagicMock()
        def lookup_pin(desc, can_pullup=False, share_type=None):
            pull = desc.startswith('^')
            return {'pin': desc.lstrip('^'), 'pullup': int(pull), 'chip': CHIP, 'class': None}
        pins.lookup_pin.side_effect = lookup_pin
        objs = {'pins': pins}
        self.printer.lookup_object.side_effect = lambda n, *a: objs.get(n, mock.MagicMock())
        self.printer.load_object.side_effect = lambda *a: mock.MagicMock()
    def get_printer(self): return self.printer
    def get_name(self): return self.name
    def _g(self, k, d, conv):
        if k in self.vals: return conv(self.vals[k])
        if d is sentinel: raise KeyError(k)
        return d
    def get(self, k, default=None, **kw): return self.vals.get(k, default)
    def getfloat(self, k, default=None, **kw): return float(self.vals[k]) if k in self.vals else default
    def getint(self, k, default=None, **kw): return int(self.vals[k]) if k in self.vals else default
    def getboolean(self, k, default=None, **kw): return bool(int(self.vals[k])) if k in self.vals else default
    def getlist(self, k, default=None, **kw): return default
    def getlists(self, k, default=None, **kw): return default
    def getchoice(self, k, choices, default=None): return choices[int(self.vals[k])] if k in self.vals else default
    def has_section(self, n): return n in self.sections
    def getsection(self, n): return Cfg(n, self.sections[n], self.sections)
    def get_prefix_options(self, p): return []
sentinel = object()
CHIP = mock.MagicMock()

def regs(stepper, sv, tv):
    secs = {stepper: sv}
    c = Cfg('tmc2209 ' + stepper, tv, secs)
    d = t9.TMC2209(c)
    return list(d.fields.registers.items())

x = regs('stepper_x', {'microsteps': '16', 'rotation_distance': '40'},
         {'uart_pin': 'gpio9', 'tx_pin': 'gpio8', 'uart_address': '0', 'run_current': '0.580', 'stealthchop_threshold': '999999'})
e = regs('extruder', {'microsteps': '16', 'rotation_distance': '33.5'},
         {'uart_pin': 'gpio9', 'tx_pin': 'gpio8', 'uart_address': '3', 'run_current': '0.650'})
for name, r in (('stepper_x', x), ('extruder no stealth', e)):
    print(name, ', '.join('%s=0x%08x' % kv for kv in r))
u = tu.MCU_TMC_uart_bitbang.__new__(tu.MCU_TMC_uart_bitbang)
print('read IOIN addr0', u._encode_read(0xf5, 0, 0x06).hex())
print('write GCONF addr3', u._encode_write(0xf5, 3, 0x80, 0x1c0).hex())
print('reply IOIN 0x21000040', u._encode_write(0x05, 0xff, 0x06, 0x21000040).hex())
