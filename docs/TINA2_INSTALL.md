# Fitting Skipper Flipper to a WEEDO Tina 2 (SKR Pico + ESP32-S3)

**Status (2026-10-13): written before the first Tina 2 installation. Moves, homing, probing and
printing have only run in simulation** (`fake_mcu.py`, unit tests); the heaters, fans, LED, TMC2209
set-up and single-motor tests have run on a bare SKR Pico (see [HARDWARE_LOG.md](HARDWARE_LOG.md)).
Stay with the printer, keep a hand near the power switch and the page's **STOP EVERYTHING** button.

## What you get

- **2026-10-20 (late): the Tina 2 config is a file now, not a preset on the ROM** (the ROM's default is an
  Ender 3). Open [configs/tina2-skr-pico.cfg](configs/tina2-skr-pico.cfg), copy it all into the
  printer.cfg page and Save. It has the tuned values (0.7 A, X flipped) and homes Z down onto the bed,
  then lifts 10 mm (`[homing_override]`; needs firmware from 2026-10-20 late or newer). What follows
  describes the older preset:
- `printer.cfg` preset **WEEDO Tina 2 with an SKR Pico**: Tina 2
  mechanics from the community Tina 2 Klipper config (X/Y `rotation_distance` 32, Z 8, extruder 33.725,
  EPCOS 100K thermistor, PID 41.141 / 2.716 / 155.822, 97 mm/s, 1000 mm/s²), SKR Pico pins, and a
  **smaller build volume for the larger printhead: X 5-85, Y 0-105, Z -2-80 mm** (stock 97 x 120 x 91).
- **Sensorless homing on X, Y and Z** (TMC2209 stallGuard, like Klipper's `tmc2209_stepper_x:virtual_endstop`)
  and a **sensorless Z probe** (`PROBE`: the nozzle is lowered until the Z motor stalls).
- **Printer page** (`/printer`): temperatures, Home all / X / Y / Z, Probe, jog buttons, extrude and
  retract, fan, upload a G-code file, Print / Pause / Resume / Cancel with progress.

## Before you start

1. Flash the new ESP32-S3 image with the Flasher (Master tab, Use shipped firmware). It has a new
   flash layout (a space for the G-code file); settings and the stored `printer.cfg` are kept.
   The SKR Pico firmware does not change.
2. Power: if the ESP32-S3 has its own USB power, switch the printer off and on with care: see
   "Switching the board off with the ESP32 running" in [WIRING_SKR_PICO.md](WIRING_SKR_PICO.md).
   Simplest is to power the ESP32-S3 from the SKR Pico's 5V pin so both switch together.
3. Note the Tina 2's power supply voltage. The SKR Pico takes 12 or 24 V (BTT: "DC 12/24V"). The
   heater cartridge must suit that voltage; if it is rated for a lower one, set `max_power` in
   `[extruder]` to (rated V / supply V)², e.g. 0.36 for a 12 V cartridge on 20 V.

## Wiring (SKR Pico)

| Tina 2 part | SKR Pico | Pin in printer.cfg |
|---|---|---|
| X motor | X driver socket | step gpio11, dir gpio10, enable gpio12 |
| Y motor | Y driver socket | step gpio6, dir gpio5, enable gpio7 |
| Z motor | Z driver socket | step gpio19, dir gpio28, enable gpio2 |
| Extruder motor | E driver socket | step gpio14, dir gpio13, enable gpio15 |
| Hotend heater | HE (heater) output | gpio23 |
| Hotend thermistor | TH0 | gpio27 |
| Part cooling fan | FAN1 | gpio17 (`[fan]`, M106/M107) |
| Hotend (heat-break) fan | FAN2 | gpio18 (`[heater_fan]`: on above 50 °C) |
| ESP32-S3 | UART header, as before | [WIRING_SKR_PICO.md](WIRING_SKR_PICO.md) |

- **Sensorless jumpers:** fit the SKR Pico's sensorless-homing jumper caps for X, Y and Z (BTT: "The
  jump caps are connected to corresponding axis"; "If you select this function, you cannot use
  external ENDSTOP"). Unplug the Tina 2's endstop switches from those ports. The preset expects the
  drivers' DIAG outputs on gpio4 (X), gpio3 (Y) and gpio25 (Z), the pins Klipper's SKR Pico config uses
  for the X/Y/Z endstops. **Not checked against BTT's pinout PDF yet:** `QUERY_ENDSTOPS` (Printer page
  button "Endstops?") shows whether they read.
- The Tina 2's stock connectors may not match the SKR Pico's; check each plug's pin order before
  powering up (motor coils in pairs, thermistor polarity does not matter, fans and heater do).
- Heated bed: the stock Tina 2 has none, but the author's has one (about 120 x 120 mm), so
  `docs/configs/tina2-skr-pico.cfg` has a `[heater_bed]` on the SKR Pico's HB output (gpio21) and
  THB thermistor (gpio26) since 2026-10-27. Without a bed, delete that section; bed commands in G-code
  files are then skipped.

## Step by step

1. **printer.cfg:** open `/config`, click **WEEDO Tina 2 with an SKR Pico**, check it, **Save and apply**.
   The "Last check" box must say "No problems found" (notes about unused sections are fine).
2. **Temperatures:** the hotend should read room temperature on the status page.
3. **Test bench** (`/test`), with the 12/24 V supply on:
   - **TMC2209 drivers**: all four must answer.
   - **Hotend output check (MOSFET)**: watch the board's heater LED blink with the steps shown.
   - **Motor tests**, one axis at a time, at a low speed (e.g. 20 rpm, 2 s). Each axis' **+ (CW)
     direction must move away from 0**: X towards the X home end (X homes to its maximum), Y away from
     its home (Y homes to 0), Z upwards (Z homes up, to the top). If an axis goes the wrong way, add or
     remove the `!` on its `dir_pin` in printer.cfg, save, test again. A wrong direction makes the
     homing move run the wrong way.
   - **Fans**: run each Fan test (fan, hotend_fan, fan3 = gpio17, gpio18, gpio20). Watch the fan and
     the SKR Pico's LED next to each fan port. LED on but fan still: fan wiring or a fan that needs a
     different voltage. LED off: the fan is on another port; run the other fan tests, then swap the
     pins in printer.cfg so the part fan is `[fan]` (M106) and the hotend fan a `[heater_fan]`.
     (If you loaded the Tina 2 preset before 2026-10-14, load it again or add `[heater_fan fan3]`
     `pin: gpio20` by hand.)
   - **LED**.
4. **StallGuard test first** (Test bench, "StallGuard stepper_x" / `_y` / `_z`). Move the axis to the
   middle by hand (motors off), leave the speed box empty (= your `homing_speed`) and press Run. It
   runs the motor 10 mm forward and back twice and reports:
   - `SG_RESULT` while moving freely (high numbers, e.g. 200-400; near 0 means too slow for
     stallGuard: try a higher speed, e.g. 25-30 mm/s, and use that as `homing_speed`);
   - whether the DIAG pin reaches the board: the second half runs with `driver_SGTHRS: 255`, which
     must trigger. "DIAG NEVER REACHES gpio4" = the DIAG jumper for that driver is missing (BTT's own
     SKR Pico config uses `diag_pin: ^gpio4` for X and `^gpio3` for Y, so the pins are right);
   - a suggested `driver_SGTHRS` (30 % of the free-running readings). Put it in printer.cfg, save.
   **Motor current:** the preset's `run_current: 0.500` is a cautious guess (the Tina 2's motor rating
   is not published that I could find). Read the label on each motor and set `run_current` to about
   70 % of its rated current. StallGuard readings change with current: run the test again after.
5. **Sensorless homing**, one axis at a time from the Printer page, hand on STOP:
   - **Home X**. It should run towards its end and stop when it bumps it.
     - It stops early, before reaching the end: lower `driver_SGTHRS` in `[tmc2209 stepper_x]` (try
       steps of 10-20).
     - It bumps or grinds and does not stop (then "No trigger on x after full movement"): raise it.
   - The same for **Home Y** and **Home Z** (Z goes up, slowly, at 4 mm/s).
   - Klipper's own notes on tuning: [Sensorless_Homing.md](https://www.klipper3d.org/Sensorless_Homing.html).
6. **Z height:** Home all. Jog Z down in 1 mm, then 0.1 mm steps until a sheet of paper just drags
   under the nozzle. Read Z on the page (e.g. 6.4). Set `position_endstop: 80 - that number` in
   `[stepper_z]` (e.g. 73.6), and `position_max` to the same value. Save, Home all, check again.
7. **Hotend:** PID tune at your printing temperature (Test bench, PID tune hotend, e.g. 200 °C), then
   **Save PID**. Run the hotend heater test: it must stay within ±5 °C.
8. **Extruder:** heat to 200 °C, Extrude 5 mm a few times (it refuses below 170 °C).
9. **Probe (optional):** `PROBE` lowers the nozzle until the Z motor stalls and reports the height.
   This presses the nozzle onto the bed: try it at a low `driver_SGTHRS` and watch.
10. **First print:** slice for an 80 x 105 mm bed, 0.4 mm nozzle, 1.75 mm filament, with a plain start
   G-code (no `START_PRINT` macro), for example:
   ```
   G28
   M109 S200
   G92 E0
   ```
   Turn **arc fitting off** (G2/G3 stop the print with a message) and use slicer retraction (G10/G11
   are refused too). Upload on the Printer page, press Print, and stay with it.

## Supported G-code while printing

G0, G1, G4, G28, G90, G91, G92, M82, M83, M84/M18, M104, M109, M140, M190 (skipped without a bed),
M106, M107, M114, M117, M204, M220, M221, M400, PROBE. Anything else (M73, M115, M201, M203, M205,
macros, ...) is skipped and listed when the print ends. Not yet: arcs, firmware retraction, bed mesh,
pressure advance (M900 is skipped), filament change (M600), Z offset adjustment during a print.
