# Delta printers (kinematics: delta)

*2026-10-31. Works in the PC tests (1,898 checks, both maths): three-tower homing, limits and the head's
path against the real arm geometry (worst 0.04 mm off a straight line, about three steps' worth).
**Not run on a real delta, and the ESP32 firmware was not compiled with the new code in this sandbox.**
The maths is Klipper's `delta.py`.*

## Which build

| Build | Cartesian | CoreXY | Delta |
|---|---|---|---|
| DECKHAND (default) | yes | yes | no |
| DECKHAND delta (`idf.py -B build-delta -DSF_KIN=delta build`) | yes | no | yes |
| CAPTAIN, SKIPPER | yes | yes | yes |

DECKHAND's flash is nearly full, so CoreXY and delta are separate images there (its WiFi-update version
tag ends in `-delta`, so one cannot be flashed over the other by mistake). A printer.cfg that asks for a
kinematics the build lacks is refused with a message. The flash cost of the delta code has **not been
measured** yet.

## printer.cfg

Klipper's options, with `stepper_a`, `stepper_b`, `stepper_c` for the towers (A at 210 degrees, B at 330,
C at 90, looking down from above: Klipper's layout):

```
[printer]
kinematics: delta
max_velocity: 200
max_accel: 2000
max_z_velocity: 100
max_z_accel: 1000
delta_radius: 100        # centre to each carriage's arm joint, mm (calibrate this)
print_radius: 80         # the bed you may print on; the arms may allow less
arm_length: 250          # all three; or arm_length: in each tower
minimum_z_position: 0    # lowest Z allowed (default 0)

[stepper_a]
step_pin: ...  dir_pin: ...  enable_pin: ...  microsteps: 16  rotation_distance: 40
endstop_pin: ^gpio4
position_endstop: 300    # nozzle height above the bed with this carriage at its endstop (mm)
homing_speed: 50
# angle: 210             # tower angle if yours differs
```

`[stepper_b]` and `[stepper_c]` the same with their own pins and `position_endstop`. The three towers need
the same steps per mm. Not used on a delta: `position_min`, `position_max` (the endstop height is the
limit), `homing_positive_dir` (always up).

## What it does

- `G28` homes **all three** towers together (any axis letters are ignored, as in Klipper). Each carriage
  stops on its own endstop; the head then sits where the arms put it (Klipper's `home_position`), backs off
  `homing_retract_dist` (5 mm) and does a slow second approach.
- Moves inside `print_radius` and between `minimum_z_position` and the lowest `position_endstop` are
  accepted; the rest give "Move out of range". Near the edge, where a tower runs several times faster
  than the head, speed is cut to half and a quarter (`delta.py`).
- `PROBE` lowers all three towers together onto the probe.
- Input shaping and pressure advance work as on any printer: they act on the head's X, Y, Z, and the tower
  steps come from the inverse kinematics after them.
- `QUERY_ENDSTOPS` reports `a`, `b`, `c`.

## Not on a delta yet

- Resonance tests (accelerometer) and the one-tower motor tests (a tower alone drags the head sideways
  and can pull the arms off): refused with a message. The stallGuard *hard stop* through G28 is the way.
- **Delta calibration** (`DELTA_CALIBRATE`, bed-radius probing, endstop trim) and a round bed mesh: none yet.
  Measure `delta_radius`, arm length and endstop heights by hand, or calibrate in Klipper and copy the numbers.
- Homing on the probe (`probe:z_virtual_endstop`): only for cartesian `stepper_z`.
- A tower's path is a straight line over each 2-5 ms interval; right beside a tower that is not quite true.
  The PC test sees at most about 0.04 mm; Klipper solves every step exactly.

## First test (bare minimum)

1. First run with the arms **unclipped** from the effector, or the effector lifted clear, so a wrong direction cannot crash anything.
2. `QUERY_ENDSTOPS` with each endstop pressed by hand: `a`, `b`, `c` must each read TRIGGERED.
3. Hand-position the carriages 20 mm below their endstops, effector on, bed clear. `G28`. Watch; hand on power.
4. `M114` after homing should read X0 Y0 Z=your lowest `position_endstop` if all three are equal.
