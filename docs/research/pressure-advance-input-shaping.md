# Pressure advance, input shaping and resonance calibration on an ESP32-S3

Research notes, 2026-10-24. Question asked: *we have the space, but do we have the processing?*
Short answer: **yes on CAPTAIN / SKIPPER at Ender 3 printing speeds, with a design that does not
copy Klipper's per-step maths.** Nothing in this note has run on a printer yet; the numbers come
from the PC, from QEMU with instruction counting, and from the one ESP32 timing measured on
hardware (2026-10-20).

## 1. How Klipper does it

Read from Klipper v0.13.0 itself (`raw.githubusercontent.com/Klipper3d/klipper/v0.13.0/...`):

| File | What it does |
|---|---|
| `klippy/extras/shaper_defs.py` | the six shapers (zv, mzv, zvd, ei, 2hump_ei, 3hump_ei): amplitudes A and times T for a frequency and damping |
| `klippy/chelper/kin_shaper.c` | shaped position x_s(t) = sum A_i x(t + t_i), pulses reversed and shifted so sum A_i t_i = 0 (constant speed passes unchanged); evaluated inside itersolve, i.e. for **every step** |
| `klippy/chelper/kin_extruder.c` | pressure advance: E + K v_E (only for moves with XY travel and positive extrusion), then a triangular average over `pressure_advance_smooth_time`; again per step |
| `klippy/extras/resonance_tester.py` | the test: a vibration sweeping 5 to 135 Hz at 1 Hz/s, acceleration 60 mm/s^2 per Hz, on top of a slow 400 mm/s^2 sweep (period 1.2 s) |
| `klippy/extras/shaper_calibrate.py` | Welch power spectrum (window about 0.5 s rounded up to 2^k, Kaiser beta 6, half overlap), then for each shaper the frequency (0.2 Hz steps up to 150 Hz) that leaves the least vibration with the least smoothing |
| `klippy/extras/adxl345.py`, `mpu9250.py`, `src/sensor_adxl345.c` | chip set-up, 5-byte packed samples over `sensor_bulk_data` |
| `bed_mesh.py`, `screws_tilt_adjust.py`, `gcode_arcs.py`, `firmware_retraction.py`, `exclude_object.py`, `tuning_tower.py` | the other features ported the same day |

Klipper runs on a Raspberry Pi with hardware doubles and solves for every step time iteratively.
The ESP32-S3 has a single-precision FPU only; doubles are software (tens of cycles per operation).

## 2. Where our CPU actually goes

Our planner already cuts motion into constant-speed segments of about 5 ms (it was written for a
segment queue). Step times are spread evenly inside each segment and compressed into Klipper's
`queue_step` (interval, count, add) within 25 us.

Measured, the cost is the **compressor**, not the planning: the 2026-10-20 Tina 2 Z homing move
(49,200 steps) took about 0.4 s on the ESP32-S3 to compress, which is what made "Home all" late.
Two cheap changes (2026-10-24):

- step times in 1/65536-tick fixed point (int64) instead of doubles;
- stop trying further fits once one `queue_step` covers the whole 256-step window.

Same commands out (the unit tests compare), **2.2x faster** in QEMU.

## 3. The design: shaping and advance on a time grid, not per step

`ph::MotionShaper` (host/src/shaping.cpp) sits between the planner and the step queues. It keeps the
plan's positions at segment boundaries (whole steps, exact), and on its own grid of 2 ms works out:

- X and Y: Klipper's convolution, sum A_i x(tau + t_i), with x linear between boundaries;
- E: Klipper's integral, done exactly piece by piece (E is linear between boundaries, so each piece
  of the triangular weight integrates in closed form);
- Z: passed through.

Steps per grid cell = rounded output position at its end minus at its start, spread evenly: no
drift, not a step lost or added (tested). The output starts `lead` early and ends `tail` late (the
shaper's half-length, or half the smoothing time), as Klipper's `gen_steps_pre/post_active`; the
toolhead leaves that gap after each stop. Homing, the stallGuard tests and the resonance test
bypass it (Klipper also turns shaping off for the resonance test).

Cost: per 2 ms, a handful of interpolations per axis, instead of an iterative solve per step.

### Choosing the grid (simulated frame resonance)

A spring-and-mass model of the frame (damping 0.05), a 40 mm move at 150 mm/s and 3000 mm/s^2,
mzv tuned to the resonance; largest swing after the move (`test_shaping.cpp`):

| resonance | no shaping | 5 ms grid | 2 ms grid | 1 ms grid | ideal (continuous) |
|---|---|---|---|---|---|
| 30 Hz | 0.1058 mm | 0.0072 | 0.0051 | 0.0016 | 0.0033 |
| 45 Hz | 0.0293 | 0.0066 | 0.0029 | 0.0031 | 0.0014 |
| 60 Hz | 0.0091 | 0.0029 | 0.0034 | 0.0054 | 0.0004 |
| 80 Hz | 0.0115 | 0.0145 (worse) | 0.0008 | 0.0028 | 0.0005 |

Below about 0.005 mm the numbers are rounding to whole steps (one step is 0.0125 mm here). 5 ms
was fine to 60 Hz and harmful at 80 Hz; **2 ms** keeps most of the benefit to 80 Hz.
`queue_step` traffic on a curvy test path: 320/s plain, 426/s with a 2 ms grid (1 ms: 562/s),
about 5 KB/s of a 25 KB/s UART.

## 4. Processing, measured

`ph::motion_bench_report()` (host/src/bench.cpp) runs a fixed path (circles and zig-zags, extruding,
Ender 3 steps/mm). QEMU 9.0 (Espressif) with `-icount shift=2`: every instruction counts 4 ns,
about the S3's 240 MHz at one instruction per cycle. A QEMU-only build runs it at start-up
(`CONFIG_SKIPPER_BENCH_AT_BOOT`).

| | QEMU | per step |
|---|---|---|
| Z homing move, compressor before 2026-10-24 | 136 ms | 2.76 us |
| Z homing move, 2026-10-24 compressor | 62 ms | 1.27 us |
| print path 150 mm/s, plain | 6 % of one core | 4.84 us |
| print path 150 mm/s, input shaping + pressure advance | 14 % of one core | 11.37 us |

Calibration: the old compressor took about 0.4 s for that homing move on the real chip, QEMU says
0.136 s, so **the real chip is roughly 2.5-3x slower** (flash cache misses, multiply latency; QEMU
counts every instruction as one cycle). Estimates for the real chip:

- plain printing at 150 mm/s: about 15-18 % of one core;
- with input shaping and pressure advance: **about 35-40 % of one core**; at 300 mm/s about twice
  that. The S3 has two cores; WiFi lives on core 0.

The board itself can measure it: Tune page, **Motion benchmark** (`/bench`), or the start-up run in a
QEMU build. Further speed-ups if needed: float positions relative to the current knot (the shaper
still uses doubles), and pinning the link task to core 1.

## 5. Resonance calibration

- **Test motion** (`resonance_test_sequence`, `resonance_motion`): Klipper's sequence exactly; the
  motion is constant-acceleration pieces, and the toolhead generates exact step times (a quadratic
  per step) a fraction of a second ahead while it runs, so memory stays small. Tested: 120,645 steps
  for a path the maths says needs 120,647; it ends within 0.001 mm of where it should. It swings
  about +-18 mm (the slow sweep) and reaches 8,500 mm/s^2 at 135 Hz: small motors may skip; lower
  ACCEL_PER_HZ.
- **Spectrum** (`PsdAccumulator`): Welch built while the samples arrive (three 1024-point FFTs every
  0.32 s at 1600 samples/s), about 30 KB instead of storing two minutes of samples.
- **Fit** (`find_best_shaper`): Klipper's, with float inner loops. Checked against Klipper's own
  `shaper_calibrate.py` on the same synthetic signal (`host/tools/shaper_reference.py` writes the
  reference, `test_resonance.cpp` compares): **every shaper, frequency, vibration, smoothing and
  max_accel identical to the printed precision; best ei at 41.8 Hz in both.** About 6 million
  float exp/sin/cos: estimated 5-10 s on the ESP32, in its own low-priority task.
- **Rates:** ADXL345 at 1600 samples/s (Klipper uses 3200; the FIFO holds 32 samples and the
  firmware tick is 10 ms, so 3200 would overflow), MPU at 1000. The frequency resolution is the
  same (the window scales with the rate) and 1000 samples/s still covers 200 Hz.

## 6. Finding the accelerometer automatically

On spare ESP32 pins (never the UART, LED, SD, USB, start-up or flash/PSRAM pins):

1. the OLED's own two I2C wires first (an ADXL345 or MPU-6050 can share them: different addresses);
2. every ordered pair of spare pins as I2C with the internal pull-ups, addresses 0x53, 0x1D, 0x68,
   0x69, then the ID register (ADXL345 DEVID 0xE5; MPU WHO_AM_I 0x68 / 0x70 / 0x71 / 0x73);
3. SPI for an ADXL345, bit-banged: every (CS, SCK, MOSI) combination driven while **all other spare
   pins are read at once as MISO** (about 3,000 combinations instead of 50,000), accepted only if
   DEVID reads 0xE5 twice and BW_RATE its reset value 0x0A;
4. optionally the USB-C port as a USB host, for a Klipper USB accelerometer.

Under QEMU (no accelerometer) the search finished cleanly in 20 s of emulated time and the firmware
carried on starting; on the chip it should be a few seconds.

### The USB accelerometer (compile-only)

The S3's USB-C port can be a USB host, but it shares its one USB PHY with the USB log / flashing
port, so the log stops until a restart (the web log keeps going). The Klipper USB accelerometers
are RP2040 boards running Klipper (USB id 1d50:614e): the firmware talks to them with the same
`ph::KLink` as to the printer board, configures the five standard RP2040 SPI pin groups at once
(RX, CSn, SCK, TX on GPIO 0-3, 4-7, 8-11, 12-15, 16-19; KUSBA and Klipper's Pico wiring use 0-3,
the BTT ADXL345 V2 8-11) each with an ADXL345 object, and keeps the one whose DEVID reads 0xE5.
Samples are Klipper's 5-byte packing (`adxl345.py _convert_samples`). Needs an OTG adapter and 5 V on
the port. QEMU has no USB: this part has not run at all.

## 7. What is not done (and why)

- Shaping and the advance on a 2 ms grid, not per step: see section 3 for what that costs.
- Changing the shaper frequency during a print applies at the next stop; the advance value can
  change any time (TUNING_TOWER needs that).
- Bed mesh: bilinear on the probed points (Klipper with mesh_pps 0); no fade, no
  zero_reference_position, one saved profile ("default"; printer.cfg was 4 KB then; 16 KB on CAPTAIN / SKIPPER since 2026-10-27).
- Arcs: XY plane only (as most slicers emit).
- DECKHAND (2 MB flash) has none of these: 15 KB left.
