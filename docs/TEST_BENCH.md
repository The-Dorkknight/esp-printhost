# Test bench

The ESP32-S3 page `http://192.168.4.1/test` (the **TEST BENCH** button on the status page) runs
every part of the printer one at a time, each with a **Run** and an **Abort** button. It is for
checking the wiring and the board before anything prints.

**Status (2026-10-13): built and tested in simulation only.** Nothing on this page has run on real
hardware yet: see [HARDWARE_LOG.md](HARDWARE_LOG.md) for what has. Stay with the printer.

## Safety, in order of who acts

| What happens | Who stops it | How |
|---|---|---|
| The ESP32-S3 crashes, loses power or the UART wire comes off while a heater is on | **the printer board itself** | Heater pins are configured with `max_duration` 5 s (Klipper's `MAX_HEAT_TIME`). If the ESP32-S3 does not refresh the heater within 5 s, the board shuts down with everything off ("Missed scheduling of next digital out event"). Tested in the simulator by killing the host mid-heat. |
| The thermistor comes off, breaks or shorts | **the printer board itself** | The ADC range check (`query_analog_in` `min_value`/`max_value`, from `min_temp`/`max_temp`) shuts the board down after 4 bad readings ("ADC out of range"). |
| The heater is on but the temperature does not rise (heater off its block, bad wire, wrong pin) | the ESP32-S3 | Klipper's `verify_heater`: it must gain 2 C every 20 s (60 s for the bed) while heating, and stay within 5 C of the target once there. If not: emergency stop. |
| The temperature passes `max_temp` | both | The ESP32-S3 checks every second; the board's range check too. |
| You press **STOP EVERYTHING** (or type `M112`) | the printer board | Klipper's `emergency_stop`: every output to its off state at once. The board then needs **Restart board** (`FIRMWARE_RESTART`). |
| You press **Abort** on one test | the ESP32-S3 | Heater test: target 0. Motor test: a trsync trigger (what Klipper's homing uses) stops the motor at once and clears the board's step queue, then the driver is switched off. Fan / LED test: off / back to the initial colour. |

A heater whose thermistor is not fitted shuts the board down as soon as it is configured (the range
check). That is Klipper's behaviour too: remove the `[heater_bed]` section if no bed thermistor is
fitted.

## The tests

| Test | What it does | Pass means |
|---|---|---|
| **Hotend heater** / **Bed heater** | Heats to the target you type (default 75 C / 50 C), holds it for 60 s, then switches off. Gives up after 5 min (15 min for the bed) if the target is not reached. | It reached the target, and the page shows the temperature range it held and the highest heater power used. |
| **Motor** (one row per stepper) | Enables the driver, then: CW at the rpm you type (default 20) for the seconds you type (default 20), a pause, CCW the same, then a shake (1/8 turn forward and back, 10 times, at 60 rpm), then the driver off. **Shake** runs just the shake. (Klipper's own `STEPPER_BUZZ`, 1 mm back and forth, is still there as a command; on a 40 mm-per-turn axis that is only 9 degrees, which is why it looked weak.) | The board's own step counter is back where it started. That proves the steps were generated and counted, not that the motor turned: watch it. During the CW run the TMC2209 is asked for its status, and over-temperature, short or open-load flags are reported (open load is only a hint in stealthChop). |
| **Fan** (one row per fan) | 100 % for 3 s (after Klipper's 0.1 s kick start), 50 % for 3 s, off. | It ran; there is no tachometer, so check by eye. |
| **LED colours** | Red, green, blue, white, 1 s each, then the initial colour. | It ran; check by eye. |
| **Hotend / Bed output check (MOSFET)** | With the heater below 60 C: 5 s off, then 5 blinks of 1 s at full power and 1 s off (watch the heater's LED on the board: lit when ON, dark when OFF), then 40 s off while the temperature is watched. | It warmed during the blinks and stopped warming soon after. **Fails** if it never warms (not connected, wrong pin, no 12-24 V, output never switches on) or if it is still getting hotter 15-40 s after switching off (output stuck on: a shorted MOSFET). |
| **PID tune hotend** / **PID tune bed** | Klipper's `PID_CALIBRATE`: full power up to the temperature you type (default 170 C / 60 C), off until 5 C below it, about six times; the swings give the PID values (Klipper's Astrom-Hagglund / Ziegler-Nichols maths). | New `pid_Kp/Ki/Kd` shown and used at once; **Save PID to printer.cfg** (`SAVE_CONFIG`) keeps them (the board restarts). |
| **TMC2209 drivers** | Reads each driver's version over the TMC UART, writes Klipper's register values, checks each write was counted (IFCNT), reads the status. | Every driver answered and took its settings. A TMC2209 only answers when the **motor power (12-24 V) is on**. |
| **Run all tests** | TMC check, then LED, fans and every motor in turn, while the hotend heats to 75 C and holds for 60 s. | Every part passed. |

The heater and the TMC check do not run together: a TMC check holds the board link for a few
seconds, which must not happen while a heater relies on being refreshed. Turn heaters off first
(the page says so if you forget).

## When a heater misbehaves

- **It keeps heating with its output off.** Not a Klipper check, added here: if a heater gets 10 C
  hotter after its output has been off for 20 s (60 s for the bed), the ESP32-S3 stops everything and
  the test bench shows **SWITCH OFF THE 12-24 V POWER SUPPLY**. Software cannot switch off a shorted
  MOSFET: only the power switch can. The same warning is added to an "ADC out of range" shutdown when the
  output had been off while the temperature rose. Run the **output check** to test the MOSFET.

- **It overshoots the target.** The default PID values are Klipper's sample values for BTT's reference
  hotend, not yours. Run **PID tune** at the temperature you print at, then **Save PID**. The heat test
  now says when it overshot by more than 3 C.
- **The board shuts down with "ADC out of range".** The board's own check: a raw reading below the range
  means hotter than `max_temp`, above it means colder than `min_temp` or a thermistor that came loose.
  The page shows the last readings and the limits, and **/heater_log.csv** has the last 5 minutes
  (temperature, target, power, raw ADC and the board-clock estimate error), including the session
  before the shutdown. Please send that file if it happens again.
- **The printer board stops answering after a shutdown** (the ESP32-S3 light blinks red fast). The
  ESP32-S3 now sends the board a reset as soon as it stops answering, and again every second failed
  attempt, using every message sequence number so a board that is merely out of step comes back. If
  the board still does not answer, its processor has stopped: power-cycle it, and please note what the
  live log (/console) showed just before.

## The same from the console

Everything also works as typed G-code (status page console, test bench console, or
`/gcode?cmd=...`): `TEST_HEATER HEATER=extruder TARGET=75 HOLD=60`, `STEPPER_TEST STEPPER=stepper_x
RPM=20 SECONDS=20`, `STEPPER_BUZZ STEPPER=stepper_x`, `FAN_TEST FAN=fan`, `LED_TEST`, `TMC_CHECK`,
`DUMP_TMC STEPPER=stepper_x`, `STEPPER_SHAKE STEPPER=stepper_x`, `PID_CALIBRATE HEATER=extruder TARGET=200`, `SAVE_CONFIG`, `HEATER_LOG`, `TEST_ALL`, `TEST_ABORT [TEST=heat_extruder]`, plus `M104`, `M109`,
`M140`, `M190`, `SET_HEATER_TEMPERATURE`, `TURN_OFF_HEATERS`, `M106`, `M107`, `M18`/`M84`, `M112`,
`FIRMWARE_RESTART`, `M105`, `SET_LED`, `HELP`. `M109`/`M190` set the target but do not wait (the
console answers at once). Moves, homing and printing (`G0`, `G1`, `G28`) are not supported yet.

## Temperature boxes

After a shutdown, or when the board or the ESP32-S3 stops answering, every box stops and turns black:
the top line keeps the last reading, the other four say ERROR, and the label says "STOPPED (last
reading)". The board stops sending temperatures when it shuts down, so a heater with a stuck output
can still be getting hotter: check by hand from a distance, and switch the power off.

Each sensor has a box five lines tall. Heating: the reading scrolls up, orange. Cooling: scrolls
down, purple. At temperature or hot: red, still. Cold: blue, still. Settled, the box is filled with
the same reading five times. The state is also written underneath in words (and the target and
heater power), and phones set to reduce motion get no scrolling.

## How it was tested

- `host/test/test_machine.cpp`: the shipped default `printer.cfg` against the shipped SKR Pico
  firmware's real data dictionary (every command must encode with the real formats); TMC2209
  register values and UART datagrams compared with Klipper's own code
  (`host/tools/klipper_tmc_reference.py`); PID heat-and-hold, the 5 s refresh, `verify_heater`,
  `max_temp`, fans with kick start, the motor test's step counts (21333 steps each way at 20 rpm,
  16 microsteps), abort, and "run all", against a mock board.
- `host/tools/test_klink.py` `MachineAgainstFake`: the same code over a pseudo serial port against
  `fake_mcu.py`, whose simulated printer applies the board-side checks modelled on Klipper's
  firmware (`max_duration`, "Timer too close", ADC range check, trsync stop, TMC2209 replies). This
  found two real bugs before shipping (32-bit clock handling in the simulator, and the order of the
  shutdown reason).
