# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
"""ph::KLink (the code the ESP32-S3 runs over UART) against fake_mcu.py, via klink_probe. Linux only."""
import os
import subprocess
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
PROBE = os.path.join(os.path.dirname(HERE), "build", "klink_probe")


@unittest.skipUnless(sys.platform.startswith("linux") and os.path.exists(PROBE),
                     "needs Linux and host/build/klink_probe (python3 host/tools/build.py)")
class KLinkAgainstFake(unittest.TestCase):
    def run_probe(self, *fake_args):
        fake = subprocess.Popen([sys.executable, "-u", os.path.join(HERE, "fake_mcu.py")] + list(fake_args),
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        try:
            dev = fake.stdout.readline().strip()
            r = subprocess.run([PROBE, dev, "250000"], capture_output=True, text=True, timeout=60)
            return r.returncode, r.stdout
        finally:
            fake.terminate()
            fake.wait()
            fake.stdout.close()

    def test_identify_and_get_config(self):
        rc, out = self.run_probe()
        self.assertEqual(rc, 0, out)
        self.assertIn("version:    fake-mcu-1", out)
        self.assertIn("MCU:        fake_stm32f103", out)
        self.assertIn("CLOCK_FREQ: 72000000", out)
        self.assertIn("PA0..PA15", out)
        self.assertIn("configured: no  shutdown: no", out)
        self.assertIn("crc errors: 0", out)
        self.assertIn("KLINK OK", out)

    def test_blind_reset_reaches_a_board_whatever_sequence_it_expects(self):
        # the ESP32 uses this when a board stops answering (e.g. after a shutdown)
        fake = subprocess.Popen([sys.executable, "-u", os.path.join(HERE, "fake_mcu.py")],
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        try:
            dev = fake.stdout.readline().strip()
            r = subprocess.run([PROBE, dev, "250000", "--blind-reset"], capture_output=True, text=True, timeout=60)
        finally:
            fake.terminate()
            log = fake.stdout.read()
            fake.wait()
            fake.stdout.close()
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertIn("blind reset: sent", r.stdout)
        self.assertIn("identified again after the reset", r.stdout)
        self.assertEqual(log.count("RESET (reboot)"), 1)

    def test_recovers_from_sequence_mismatch(self):
        # fake starts expecting a different sequence number: KLink must adopt it from the NAK
        rc, out = self.run_probe("7")
        self.assertEqual(rc, 0, out)
        self.assertIn("KLINK OK", out)


@unittest.skipUnless(sys.platform.startswith("linux") and os.path.exists(PROBE),
                     "needs Linux and host/build/klink_probe (python3 host/tools/build.py)")
class KBoardAgainstFake(unittest.TestCase):
    """Configure the (simulated) board like Klipper does and drive its NeoPixel."""

    def setUp(self):
        self.fake = subprocess.Popen([sys.executable, "-u", os.path.join(HERE, "fake_mcu.py")],
                                     stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        self.dev = self.fake.stdout.readline().strip()

    def tearDown(self):
        self.fake.terminate()
        self.fake.wait()
        self.fake.stdout.close()

    def probe(self, *args):
        r = subprocess.run([PROBE, self.dev, "250000"] + list(args), capture_output=True, text=True, timeout=60)
        return r.returncode, r.stdout

    def fake_log(self):
        self.fake.terminate()
        out = self.fake.stdout.read()
        self.fake.wait()
        return out

    def test_configure_then_reuse_then_change(self):
        rc, out = self.probe("--neopixel", "PA5", "GRB", "255,0,16")
        self.assertEqual(rc, 0, out)
        self.assertIn("configure: configured", out)
        self.assertIn("neopixel set 255,0,16", out)
        # same config again: the board keeps it, nothing is re-sent
        rc, out = self.probe("--neopixel", "PA5", "GRB", "0,255,0")
        self.assertEqual(rc, 0, out)
        self.assertIn("already configured", out)
        self.assertIn("configured: yes", out)
        # different pin: Klipper-style reset, identify again, configure
        rc, out = self.probe("--neopixel", "PB2", "GRB", "0,0,255")
        self.assertEqual(rc, 0, out)
        self.assertIn("different config: reset sent", out)
        self.assertIn("configure: configured", out)
        log = self.fake_log()
        # the fake board runs at 72 MHz: 4 us = 288 ticks, 50 us = 3600 ticks
        self.assertIn("config_neopixel oid 0 pin 5 size 3 bit_max 288 reset_min 3600", log)
        self.assertIn("neopixel oid 0 shows 00ff10", log)   # GRB order of 255,0,16
        self.assertIn("neopixel oid 0 shows ff0000", log)   # green
        self.assertIn("RESET (reboot)", log)
        self.assertIn("config_neopixel oid 0 pin 18 size 3", log)   # PB2 = 16 + 2
        self.assertIn("neopixel oid 0 shows 0000ff", log)   # blue
        self.assertNotIn("SHUTDOWN", log)

    def test_clock_sync(self):
        rc, out = self.probe("--clock", "2")
        self.assertEqual(rc, 0, out)
        import re
        m = re.search(r"clock: (\d+) Hz measured, drift (-?[\d.]+) ppm, min rtt ([\d.]+) ms, (\d+) samples", out)
        self.assertTrue(m, out)
        # the fake board's clock is the PC clock * 72 MHz, so the drift is only measurement noise
        self.assertLess(abs(float(m.group(2))), 300.0, out)
        self.assertGreaterEqual(int(m.group(4)), 10, out)

    def test_printer_cfg_and_set_led(self):
        import tempfile
        cfg = ("[mcu]\nserial: /dev/ttyAMA0\n[neopixel board_led]\npin: PA5\ncolor_order: GRB\n"
               "initial_GREEN: 0.2\n[filament_switch_sensor runout]\nswitch_pin: PA6\n")
        with tempfile.NamedTemporaryFile("w", suffix=".cfg", delete=False) as f:
            f.write(cfg)
        try:
            rc, out = self.probe("--config", f.name, "--gcode", "SET_LED LED=board_led RED=1 BLUE=0.5",
                                 "--gcode", "SET_LED LED=nope", "--gcode", "M999",
                                 "--gcode", "SET_LED LED =GREEN", "--gcode", "SET_LED LED=board_led GREEN")
        finally:
            os.unlink(f.name)
        self.assertEqual(rc, 0, out)
        self.assertIn("config note: [filament_switch_sensor] not used yet", out)
        self.assertIn("SET_LED LED=board_led RED=1 BLUE=0.5 -> ok", out)
        self.assertIn("no LED named 'nope'", out)
        self.assertIn("M999 is not supported yet", out)
        self.assertIn("'LED' is not NAME=value (no spaces around '=')", out)   # the phone-typed mistake: space before '='
        self.assertIn("'GREEN' is not NAME=value", out)
        self.assertIn("Example: SET_LED LED=board_led GREEN=1", out)
        log = self.fake_log()
        self.assertIn("neopixel oid 0 shows 330000", log)   # initial_GREEN 0.2, GRB
        self.assertIn("neopixel oid 0 shows 00ff80", log)   # red 1, blue 0.5, GRB

    def test_temperature_sensors_end_to_end(self):
        import tempfile
        cfg = ("[temperature_sensor hotend]\nsensor_type: Generic 3950\nsensor_pin: PA3\nmax_temp: 280\n"
               "[temperature_sensor bed]\nsensor_type: Generic 3950\nsensor_pin: PA4\n"
               "[temperature_sensor mcu_temp]\nsensor_type: temperature_mcu\n")
        with tempfile.NamedTemporaryFile("w", suffix=".cfg", delete=False) as f:
            f.write(cfg)
        try:
            rc, out = self.probe("--config", f.name, "--gcode", "M105")
        finally:
            os.unlink(f.name)
        self.assertEqual(rc, 0, out)
        self.assertIn("temp hotend: 25.0 \u00b0C", out)          # simulated 100k 3950 at 25 C
        self.assertIn("temp bed: no sensor? (reads open circuit)", out)
        self.assertIn("not supported yet (thermistors only)", out)
        self.assertIn("M105 -> ok hotend: 25.0 \u00b0C, bed: no sensor? (reads open circuit)", out)
        log = self.fake_log()
        self.assertIn("config_analog_in oid 0 pin", log)
        self.assertIn("query_analog_in oid 0 every 0.300 s, 8 samples", log)
        self.assertNotIn("SHUTDOWN", log)

    def test_printer_cfg_errors_stop_before_touching_the_board(self):
        import tempfile
        with tempfile.NamedTemporaryFile("w", suffix=".cfg", delete=False) as f:
            f.write("[neopixel x]\npin: gpio24\n")
        try:
            rc, out = self.probe("--config", f.name)
        finally:
            os.unlink(f.name)
        self.assertNotEqual(rc, 0)
        self.assertIn("pin 'gpio24' is not a pin on this board (line 2)", out)
        self.assertNotIn("configure:", out)

    def test_bad_pin_is_an_error_not_a_crash(self):
        rc, out = self.probe("--neopixel", "gpio99", "GRB", "1,2,3")
        self.assertNotEqual(rc, 0)
        self.assertIn("bad value pin=gpio99", out)


MACHINE_CFG = """[mcu]
serial: /dev/null
[extruder]
step_pin: PA1
dir_pin: !PA2
enable_pin: !PA5
microsteps: 16
rotation_distance: 33.5
heater_pin: PB8
sensor_type: Generic 3950
sensor_pin: PA3
control: pid
pid_Kp: 21.527
pid_Ki: 1.063
pid_Kd: 108.982
min_temp: 0
max_temp: 250
[stepper_x]
step_pin: PB0
dir_pin: !PB1
enable_pin: !PB2
microsteps: 16
rotation_distance: 40
position_max: 100
[tmc2209 stepper_x]
uart_pin: PC13
tx_pin: PA15
uart_address: 0
run_current: 0.5
stealthchop_threshold: 999999
[tmc2209 extruder]
uart_pin: PC13
tx_pin: PA15
uart_address: 3
run_current: 0.6
[fan]
pin: PB9
[heater_fan hotend_fan]
pin: PB10
"""


@unittest.skipUnless(sys.platform.startswith("linux") and os.path.exists(PROBE),
                     "needs Linux and host/build/klink_probe (python3 host/tools/build.py)")
class MachineAgainstFake(unittest.TestCase):
    """ph::Machine (heaters, motors, fans, TMC2209, safety) against the simulated printer in fake_mcu.py.
    The fake's PB8 heater warms the PA3 thermistor; its board-side checks are modelled on Klipper's."""

    def run_machine(self, fake_args, probe_args, settle=0.0):
        import tempfile, time
        fake = subprocess.Popen([sys.executable, "-u", os.path.join(HERE, "fake_mcu.py")] + list(fake_args),
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        with tempfile.NamedTemporaryFile("w", suffix=".cfg", delete=False) as f:
            f.write(MACHINE_CFG)
        try:
            dev = fake.stdout.readline().strip()
            r = subprocess.run([PROBE, dev, "250000", "--machine", f.name] + list(probe_args),
                               capture_output=True, text=True, timeout=120)
            time.sleep(settle)          # let the fake run on alone (watchdog test)
        finally:
            os.unlink(f.name)
            fake.terminate()
            log = fake.stdout.read()
            fake.wait()
            fake.stdout.close()
        return r.returncode, r.stdout, log

    def test_heat_motor_fan_tmc(self):
        rc, out, log = self.run_machine([], [
            "--at", "1", "TEST_HEATER HEATER=extruder TARGET=40 HOLD=3",
            "--at", "1", "STEPPER_TEST STEPPER=stepper_x RPM=120 SECONDS=2",
            "--at", "1", "FAN_TEST FAN=fan", "--for", "24"])
        self.assertEqual(rc, 0, out)
        self.assertIn("TMC2209: 2 ok, 0 not answering", out)
        self.assertIn("test heat_extruder: passed reached 40 C", out)
        self.assertIn("test motor_stepper_x: passed", out)
        self.assertIn("back where it started", out)
        self.assertIn("test fan_fan: passed", out)
        self.assertIn('"name":"hotend_fan","speed":', out)
        self.assertIn("shutdown: no", out)
        self.assertNotIn("SHUTDOWN", log)
        self.assertIn("config_tmcuart oid", log)
        self.assertIn("range 1528..32269", log)        # min_temp 0 / max_temp 250 as raw ADC limits

    def test_board_turns_heater_off_when_the_host_dies(self):
        # heater on, then the host exits without a word: the board's max_duration watchdog must act
        rc, out, log = self.run_machine([], ["--at", "1", "M104 S100", "--for", "4", "--no-off"], settle=7.0)
        self.assertEqual(rc, 0, out)
        self.assertIn("M104 S100 -> ok extruder target 100", out)
        self.assertIn("exiting without switching anything off", out)
        self.assertIn("SHUTDOWN: Missed scheduling of next digital out event", log)

    def test_verify_heater_catches_a_heater_that_does_not_heat(self):
        rc, out, log = self.run_machine(["--no-heat"], ["--at", "1", "M104 S200", "--for", "30"])
        self.assertEqual(rc, 0, out)
        self.assertIn("shutdown: Heater extruder not heating at expected rate", out)
        self.assertIn("SHUTDOWN: Command request (emergency_stop)", log)

    def test_missing_driver_abort_and_emergency_stop(self):
        rc, out, log = self.run_machine(["--tmc-dead", "3"], [
            "--at", "0.5", "STEPPER_TEST STEPPER=stepper_x",
            "--at", "2", "TEST_ABORT TEST=motor_stepper_x",
            "--at", "3", "M112", "--at", "3.5", "M104 S50", "--for", "4"])
        self.assertEqual(rc, 0, out)
        self.assertIn("TMC2209: 1 ok, 1 not answering", out)
        self.assertIn("is the 12-24 V motor power on?", out)
        self.assertIn("TEST_ABORT TEST=motor_stepper_x -> ok motor_stepper_x stopped", out)
        self.assertIn("test motor_stepper_x: aborted", out)
        self.assertIn("stopped by trsync", log)
        self.assertIn("M112 -> ok EMERGENCY STOP", out)
        self.assertIn("M104 S50 -> error: the board is shut down", out)
        self.assertIn("SHUTDOWN: Command request (emergency_stop)", log)


PRINTER_CFG = """[mcu]
serial: /dev/null
[printer]
kinematics: cartesian
max_velocity: 100
max_accel: 1000
max_z_velocity: 10
max_z_accel: 250
[stepper_x]
step_pin: PB0
dir_pin: !PB1
enable_pin: !PB2
microsteps: 16
rotation_distance: 32
endstop_pin: tmc2209_stepper_x:virtual_endstop
position_endstop: 85
position_max: 85
homing_speed: 20
[tmc2209 stepper_x]
uart_pin: PC13
tx_pin: PA15
uart_address: 0
run_current: 0.5
stealthchop_threshold: 999999
diag_pin: ^PA8
driver_SGTHRS: 100
[stepper_y]
step_pin: PB3
dir_pin: !PB4
enable_pin: !PB5
microsteps: 16
rotation_distance: 32
endstop_pin: ^PA9
position_endstop: 0
position_max: 105
homing_speed: 20
[stepper_z]
step_pin: PB6
dir_pin: PB7
enable_pin: !PB11
microsteps: 16
rotation_distance: 8
endstop_pin: ^PA10
position_endstop: 80
position_min: -2
position_max: 80
homing_speed: 5
homing_retract_dist: 0
[extruder]
step_pin: PA1
dir_pin: !PA2
enable_pin: !PA5
microsteps: 16
rotation_distance: 33.725
heater_pin: PB8
sensor_type: Generic 3950
sensor_pin: PA3
control: pid
pid_Kp: 21.527
pid_Ki: 1.063
pid_Kd: 108.982
min_temp: 0
max_temp: 250
min_extrude_temp: 50
[probe]
pin: ^PA11
z_offset: 0
[fan]
pin: PB9
"""

PRINT_GCODE = """; test print for the simulator
G28
M109 S60
G90
M83
G1 Z0.3 F600
G1 X20 Y20 F6000
G1 X60 Y20 E2 F1800
G1 X60 Y60 E2
G1 X20 Y60 E2
M106 S200
G1 X20 Y20 E2
G1 Z5
M107
M104 S0
M84
"""


@unittest.skipUnless(sys.platform.startswith("linux") and os.path.exists(PROBE),
                     "needs Linux and host/build/klink_probe (python3 host/tools/build.py)")
class PrintAgainstFake(unittest.TestCase):
    """Homing (sensorless X, physical Y with a second approach, Z), M109, moves with extrusion, fan and M84
    from an uploaded file, over the serial protocol, with the fake board's timing checks (any command
    scheduled in the past shuts it down)."""

    def test_print_a_file(self):
        import tempfile
        fake = subprocess.Popen([sys.executable, "-u", os.path.join(HERE, "fake_mcu.py")],
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        with tempfile.NamedTemporaryFile("w", suffix=".cfg", delete=False) as f:
            f.write(PRINTER_CFG)
        with tempfile.NamedTemporaryFile("w", suffix=".gcode", delete=False) as g:
            g.write(PRINT_GCODE)
        try:
            dev = fake.stdout.readline().strip()
            r = subprocess.run([PROBE, dev, "250000", "--machine", f.name, "--print", g.name,
                                "--at", "0.5", "SDCARD_PRINT_FILE", "--for", "40"],
                               capture_output=True, text=True, timeout=120)
        finally:
            os.unlink(f.name)
            os.unlink(g.name)
            fake.terminate()
            log = fake.stdout.read()
            fake.wait()
            fake.stdout.close()
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertIn("machine: homed XYZ", r.stdout)
        self.assertIn("print done: finished", r.stdout)
        self.assertIn("shutdown: no", r.stdout)
        self.assertNotIn("SHUTDOWN", log)
        self.assertEqual(log.count("triggered"), 4)    # X (sensorless), Y twice (retract + slow approach), Z


if __name__ == "__main__":
    unittest.main()


@unittest.skipUnless(sys.platform.startswith("linux") and os.path.exists(PROBE),
                     "needs Linux and host/build/klink_probe (python3 host/tools/build.py)")
class SessionAgainstFake(unittest.TestCase):
    """ph::BoardSession - the ESP32's whole connection logic - against fake_mcu.py: power cycles,
    a board configured from before, a board that ignores "reset", line noise, FIRMWARE_RESTART.
    Added 2026-10-14 after the SKR Pico could not be reached again once it was power-cycled with the
    ESP32 running. SIGUSR1 switches the fake board off / on (see fake_mcu.py)."""

    def session(self, seconds, events=(), fake_args=(), probe_args=(), preconfigure=False):
        """events: (time_s, action) with action 'power' (toggle) or ('inject', bytes) (noise on the line)."""
        import signal, tempfile, threading, time
        fake = subprocess.Popen([sys.executable, "-u", os.path.join(HERE, "fake_mcu.py")] + list(fake_args),
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        with tempfile.NamedTemporaryFile("w", suffix=".cfg", delete=False) as f:
            f.write(MACHINE_CFG)
        lines = []
        reader = None
        try:
            dev = fake.stdout.readline().strip()
            reader = threading.Thread(target=lambda: lines.extend(iter(fake.stdout.readline, "")), daemon=True)
            reader.start()
            if preconfigure:   # an earlier ESP32 run left the board configured
                subprocess.run([PROBE, dev, "250000", "--machine", f.name, "--for", "1", "--no-off"],
                               capture_output=True, text=True, timeout=60)
            p = subprocess.Popen([PROBE, dev, "250000", "--session", f.name, "--for", str(seconds)] + list(probe_args),
                                 stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
            t0 = time.time()
            for at, action in sorted(events, key=lambda e: e[0]):
                time.sleep(max(0.0, at - (time.time() - t0)))
                if action == "power":
                    fake.send_signal(signal.SIGUSR1)
                else:
                    fd = os.open(dev, os.O_WRONLY | os.O_NOCTTY)
                    os.write(fd, action[1])
                    os.close(fd)
            out, _ = p.communicate(timeout=seconds + 60)
        finally:
            os.unlink(f.name)
            fake.terminate()
            fake.wait()
            if reader:
                reader.join(2)
            fake.stdout.close()
        return out, "".join(lines)

    def assertRunningAtEnd(self, out, connects):
        self.assertIn("final state: running", out)
        self.assertIn("connects: %d\n" % connects, out)
        self.assertIn("writes while TX released: 0", out)
        self.assertRegex(out, r"temp extruder: [0-9.]+ \S*C \([1-9]")

    def test_connects_and_runs(self):
        out, log = self.session(6)
        self.assertRunningAtEnd(out, 1)
        self.assertIn("STATE running (configured)", out)
        self.assertNotIn("RESET", log)
        # 2026-10-28: the move queue size is read again once configured (Klipper reports 0 before)
        self.assertIn("board move queue: 1024 commands", out)

    def test_board_configured_from_before_is_restarted_first(self):
        # the ESP32 restarted (crash, reflash) while the board kept its configuration
        out, log = self.session(8, preconfigure=True)
        self.assertIn("configuration from before: restarting it", out)
        self.assertEqual(log.count("RESET (reboot)"), 1)
        self.assertRunningAtEnd(out, 1)

    def test_board_power_cycled_while_running(self):
        # off for 1 s, on again: the ESP32 must notice, let go of its TX wire, and configure it again
        out, log = self.session(16, events=[(5, "power"), (6, "power")])
        self.assertIn("POWER ON (fresh start)", log)
        self.assertNotIn("FED THROUGH RX", log)
        self.assertNotIn("HUNG", log)
        self.assertIn("TX line went low", out)
        self.assertIn("tx released", out)
        self.assertRunningAtEnd(out, 2)

    def test_board_off_for_a_while(self):
        out, log = self.session(22, events=[(4, "power"), (12, "power")])
        self.assertNotIn("FED THROUGH RX", log)
        self.assertIn("POWER ON (fresh start)", log)
        self.assertIn("the board's TX line is low", out)
        self.assertRunningAtEnd(out, 2)

    def test_hung_board_recovers_after_a_clean_power_cycle(self):
        # Fed through its RX pin while off (as the old firmware did): it comes back hung, sending
        # 00 bytes. The session must say so, never drive TX into it, and connect once it is power-
        # cycled cleanly.
        hang = [(4, "power"), (4.3, ("inject", b"\x7e" * 32)), (5, "power")]
        out, log = self.session(26, events=hang + [(12, "power"), (14, "power")])
        self.assertIn("HUNG", log)
        self.assertIn("the board's TX line is low", out)
        self.assertEqual(log.count("FED THROUGH RX"), 1)   # only the injected bytes, nothing from the session
        self.assertIn("POWER ON (fresh start)", log)
        self.assertRunningAtEnd(out, 2)

    def test_board_that_ignores_reset_is_reported(self):
        out, log = self.session(14, fake_args=["--ignore-reset"], preconfigure=True)
        self.assertIn("RESET IGNORED", log)
        self.assertIn("does not restart when told to", out)
        self.assertIn("final state: searching", out)

    def test_board_that_ignores_reset_runs_after_a_power_cycle(self):
        out, log = self.session(24, fake_args=["--ignore-reset"], preconfigure=True,
                                events=[(10, "power"), (11, "power")])
        self.assertIn("does not restart when told to", out)
        self.assertRunningAtEnd(out, 1)

    def test_line_noise_does_not_drop_the_connection(self):
        out, log = self.session(10, events=[(4, ("inject", bytes(range(1, 40)))), (6, ("inject", b"\x05\x13junk"))])
        self.assertIn("garbage: resync", log)
        self.assertRunningAtEnd(out, 1)

    def test_firmware_restart(self):
        out, log = self.session(12, probe_args=["--at", "4", "FIRMWARE_RESTART"])
        self.assertIn("FIRMWARE_RESTART", out)
        self.assertEqual(log.count("RESET (reboot)"), 1)
        self.assertRunningAtEnd(out, 2)


@unittest.skipUnless(sys.platform.startswith("linux") and os.path.exists(PROBE),
                     "needs Linux and host/build/klink_probe (python3 host/tools/build.py)")
class StallGuardAgainstFake(unittest.TestCase):
    """STALLGUARD_TEST (added 2026-10-14: sensorless homing did not trigger on the Tina 2). The fake's
    TMC2209 model: SG_RESULT about 240-280 while moving, 0 standing; DIAG = SG_RESULT <= 2 * SGTHRS."""

    def run_sg(self, fake_args=(), cmd="STALLGUARD_TEST STEPPER=stepper_x WIRING=1"):
        import tempfile
        fake = subprocess.Popen([sys.executable, "-u", os.path.join(HERE, "fake_mcu.py")] + list(fake_args),
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        with tempfile.NamedTemporaryFile("w", suffix=".cfg", delete=False) as f:
            f.write(PRINTER_CFG)
        try:
            dev = fake.stdout.readline().strip()
            r = subprocess.run([PROBE, dev, "250000", "--machine", f.name,
                                "--at", "0.5", cmd, "--for", "10"],
                               capture_output=True, text=True, timeout=120)
        finally:
            os.unlink(f.name)
            fake.terminate()
            log = fake.stdout.read()
            fake.wait()
            fake.stdout.close()
        return r.stdout, log

    def test_wired(self):
        out, log = self.run_sg()
        self.assertIn("stallGuard test, 10 mm forward and back twice at 20 mm/s", out)
        self.assertIn("test sg_stepper_x: passed", out)
        self.assertRegex(out, r"SG_RESULT while moving freely at 20 mm/s: min 2[4-9]\d")
        self.assertIn("DIAG wiring OK", out)
        self.assertRegex(out, r"with 255 \(must trigger\) high in (\d+) of \1\.")
        self.assertRegex(out, r"with driver_SGTHRS 100 \(triggers at SG_RESULT <= 200\) high in 0 of [1-9]")
        self.assertRegex(out, r"Suggested start: driver_SGTHRS: 7\d ")
        self.assertIn("shutdown: no", out)
        self.assertNotIn("SHUTDOWN", log)

    def test_default_free_run_only(self):
        # 2026-10-20: without WIRING=1 there is no SGTHRS-255 phase (it turned DIAG red on the return
        # stroke and read as a false trigger); the wiring check is the hard-stop test's job now.
        out, log = self.run_sg(cmd="STALLGUARD_TEST STEPPER=stepper_x")
        self.assertIn("test sg_stepper_x: passed", out)
        self.assertNotIn("with 255", out)
        self.assertRegex(out, r"with driver_SGTHRS 100 \(triggers at SG_RESULT <= 200\) high in 0 of [1-9]")
        self.assertNotIn("SHUTDOWN", log)

    def test_missing_jumper(self):
        out, log = self.run_sg(["--no-diag"])
        self.assertIn("test sg_stepper_x: failed", out)
        self.assertIn("DIAG NEVER REACHES PA8", out)
        self.assertNotIn("SHUTDOWN", log)
