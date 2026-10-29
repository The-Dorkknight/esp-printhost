# QIDI X-Smart 3 on a BTT SKR Pico (test bench)

**Status (2026-10-27): written and tested on the PC only** (it parses, and `G28` homes in the
simulation: `test_qidi_xsmart3` in `host/test/test_toolhead.cpp`). Not yet tried on the printer.
Needs a CAPTAIN or SKIPPER build of 2026-10-27 or later.

The config is [`qidi-xsmart3-skr-pico.cfg`](qidi-xsmart3-skr-pico.cfg). The mechanics (CoreXY,
39.96 mm belts, 8 mm Z screw, 9.5:1 extruder), currents, offsets, PIDs and the homing order are taken
from QIDI's own X-Smart 3 printer.cfg (firmware 4.1.13, as kept in
[billkenney/qidi_3series_recovery](https://github.com/billkenney/qidi_3series_recovery)). The pins
are the SKR Pico's.

Why the swap: the stock toolhead board (MKS THR) is dead and the mainboard is doubtful, so all of it
goes to the SKR Pico, which runs plain Klipper v0.13.0 as on the Tina 2.

## Wiring

| What | Was on | SKR Pico | Notes |
|---|---|---|---|
| Motor A (left rear, X+Y) | mainboard X | **X** socket | `stepper_x` |
| Motor B (right rear, X−Y) | mainboard Y | **Y** socket | `stepper_y` |
| Z motor | mainboard Z | **Z1** socket | `stepper_z` (one motor, two screws on a belt) |
| Extruder motor | toolhead board | **E** socket | 9.5:1 gears, `gear_ratio` |
| Hotend heater | toolhead board | **HE** (gpio23) | |
| Hotend sensor | MAX6675 thermocouple on the toolhead | **TH0** (gpio27) | **replace with a thermistor** (below) |
| Bed heater | mainboard | **HB** (gpio21) | check the current (below) |
| Bed thermistor | mainboard | **THB** (gpio26) | NTC 100K MGB18 (QIDI's) |
| Inductive probe | toolhead board | **PROBE** header (gpio22) | check the voltage (below) |
| Part cooling fan | toolhead board | **FAN1** (gpio17) | |
| Both hotend fans | toolhead board | **FAN2** (gpio18) | in parallel; on above 50 °C |
| Electronics fan | mainboard | **FAN3** (gpio20) | on with the hotend |
| Side (aux) fan, chamber fan | mainboard | — | no output left: leave unplugged |
| Chamber thermistor (optional) | — | **ESP32 GPIO7** | 100 kΩ to 3V3: [CHAMBER_SENSOR.md](../CHAMBER_SENSOR.md) |
| Sensorless X/Y homing | — | jumpers on the X and Y **DIAG** pins | as on the Tina 2 |

Motor A and B: on a CoreXY, if X moves when Y is asked for (or the head goes diagonally), swap the
two motor plugs, or change the `!` on a `dir_pin`. Use the Test bench motor test on each motor before
the first `G28`: **motor A forward and motor B forward together** should move the head +X (right);
A forward with B backward should move it +Y.

## Check before the first power-up

1. **Supply voltage.** The X-Smart 3 supply is very likely 24 V (check the label). The SKR Pico takes
   12-24 V; the fans on its outputs get the supply voltage, so they must be 24 V fans.
2. **Bed heater current.** The SKR Pico's HB output is rated about 10 A. A 24 V bed draws
   24 / R amps: measure the bed's resistance with the bed cold. **Under 2.4 Ω (over 10 A) do not
   connect it directly**: use an external MOSFET or SSR driven from HB. If the stock bed runs from
   mains through an SSR, keep the SSR and drive it from HB.
3. **Probe voltage.** QIDI's inductive probe is an open-collector (NPN) sensor, wired with a
   pull-up in the config (`^!gpio22`). Power it from the supply voltage, connect only its signal and
   GND to the PROBE header, and **measure the signal wire with the probe powered but not
   triggered**: it must not be above 3.3 V (the RP2040 pin is not 5 V tolerant). If it is, add a
   diode (cathode to the probe, as on many Ender probe mods) or a divider before connecting it.
   Then hold metal under the probe and check that `QUERY_ENDSTOPS` on the console shows `z:TRIGGERED`
   (Z's endstop is the probe), and `z:open` without the metal. If it is the wrong way round, remove the `!` from `pin: ^!gpio22`.
4. **Hotend thermistor.** The SKR Pico has no thermocouple input, so the thermocouple has to go. Fit
   a 100K cartridge thermistor that fits the heater block (QIDI's blocks take the usual 3 mm
   cartridges); the config is set for an **ATC Semitec 104GT-2** (rated to 300 °C) with
   `max_temp: 290`. For another type, change `sensor_type` and keep `max_temp` below its rating.
   Then run **PID tune hotend** on the Test bench (QIDI's PID values were for the thermocouple).

## First moves

1. Home X alone (`G28 X`) with a hand on the power switch. If it does not stop at the left, lower
   `driver_SGTHRS` (less sensitive) or raise it (more), as on the Tina 2.
2. Then Y alone, then `G28`. The homing override goes: Z up 5 mm, X, back off, Y, back off, the
   probe over the middle of the bed (X 61.5 Y 89, the centre minus the probe offset), Z down onto the
   probe, Z up 10 mm.
3. **Z offset (paper test).** After `G28`, move the nozzle to the middle of the bed and lower it
   step by step until a sheet of paper just drags. The Z it shows is the nozzle's height above where
   the probe triggers: set `z_offset` in `[probe]` to minus that value (Klipper's sign: the probe
   triggers *above* the bed, so a nozzle that reached the paper at Z −1.2 means `z_offset: 1.2`).
   Then `G28` again and check Z 0 touches the paper.
4. Speeds: the config starts at 300 mm/s and 5000 mm/s² (QIDI's own are 600 and 20000). Raise them
   only after long prints show no "Timer too close" and no lost steps; the Tune page's resonance test
   needs an accelerometer (QIDI's was on the dead toolhead board, so an ADXL345 wired as in
   [TUNING.md](../TUNING.md)).

## Differences from QIDI's firmware

- No thermocouple, no filament runout switch, no case light or beeper outputs, no aux / chamber
  fans (no outputs left on the SKR Pico).
- One run current per motor: X and Y start at 0.8 A (QIDI homes at 0.8 / 0.9 A and prints at
  1.07 A). Raise to about 1.0 A once homing works, checking that homing still stops.
- Z homes on the probe in the middle of the bed (as QIDI does); the Z motor current stays at
  QIDI's 0.2 A.
