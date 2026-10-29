# Print benchmark, performance counters and the integer test build

**Status (2026-10-29):** **the benchmark runs to the end on hardware** (Tina 2, SKR Pico, drivers
off, INT32/64 test build): 121.7 s and 121.8 s, all 9,617 lines, printer task busy 43.8 % and 44.9 %
of one core, 14.6 % and 14.5 % without sending. The six earlier runs had stopped with a move refused
as too late; the cause was the board's move queue taken as 200 commands instead of the SKR Pico's
1024 (fixed 2026-10-28 evening). Not yet run: the STANDARD build for comparison. CAPTAIN and SKIPPER
builds.

**First real numbers, INT32/64 test build with the SKR Pico:** 1,009,918 steps (8,295/s), 56,292
queue_step (462/s), 57,235 commands, 21,377 command characters/s, 24,231 planner segments; printer
task busy 43.8 % of one core, of which about 29 % is sending (which includes waiting for the board's
acknowledgement) and 14.6 % everything else. The per-stage figures were cut off in that build's log
(fixed: the result is now several lines).

## What it is for

How much of the ESP32-S3's processor does a print take, and where does it go? Answering that decides
the next speed work (see [research/klipper-grade-performance.md](research/klipper-grade-performance.md)).
The benchmark runs a whole print-like file through the real motion pipeline and times each stage. And
there is a second firmware for CAPTAIN that does the motion maths in whole numbers (int32 / int64)
instead of `double`, so the two can be compared on the same board with the same file.

## Running it

Tune page, **Print benchmark**, or on the console: `SF_BENCH` (then `SF_BENCH_RESULT`).

- **Nothing moves or heats.** It refuses to start while a heater is on. The stepper drivers are
  switched off for the whole run (and refused if any motor in printer.cfg has no `enable_pin`, as its
  driver could not be kept off). Afterwards the axes are not homed (the position was made up).
- **With the SKR Pico connected:** everything happens as in a real print, including the board's step
  queues and the UART link; only the drivers stay off.
- **Without a printer board** (unplugged, or not answering): the ESP32 runs it alone. Commands are
  encoded exactly as for the SKR Pico (its data dictionary is built in) and then dropped. This is
  the pure processor cost.
- **The file:** `sf-bench.gcode` on the SD card. The first `SF_BENCH` writes it there if it is
  missing (a few seconds; then send `SF_BENCH` again). A copy is in [bench/sf-bench.gcode](bench/sf-bench.gcode).
  Without a card: `SF_BENCH FILE=builtin` (made line by line in memory; then the file reading is not
  part of the timing). Any other file: `SF_BENCH FILE=name.gcode`; `FILE=flash` for the one uploaded
  to the ESP32's own flash.
- **The file itself:** 12 layers of a 36 mm cylinder: two perimeters in 0.5 mm chords at 100 mm/s,
  300 tiny moves of about 0.25 mm at 50 mm/s (like text or a gyroid), zig-zag infill at 250 mm/s,
  retractions and travel at 300 mm/s, 5000 mm/s². All relative (G91, M83), so it runs from wherever
  the head is: the benchmark puts the made-up position in the middle of X and Y. About 85 s of
  motion when the printer.cfg allows those speeds (the Tina 2's limits make it about 200 s).

## Reading the result

A few lines, each starting with "benchmark", also written to the important-events log (Log page,
"Important events"), for example (PC simulation, 2026-10-29, not real-chip timing):

```
benchmark sf-bench.gcode: done after 198.6 s, 9617 lines; STANDARD build (double motion maths), no printer board ...
benchmark firmware: 0.1.0-wip CAPTAIN ..., built Oct 29 2026 14:20:05, binary 1234abcd
benchmark work: 1249607 steps (6291/s), 107252 queue_step (540/s), 110015 commands, ...
benchmark processor: printer task busy ... % of one core (... % without sending): planner ... %, shaping ... %,
  step times ... %, compression ... %, sending ... %, everything else (G-code, file, heaters) ... %.
benchmark sends: 9808 (101 of them during planning), longest gap between sends 60 ms, longest send 0 ms;
  board move queue: up to 528 commands in it, full 0 times.
```

- **Printer task busy:** the share of one core (100 % = all of one core) the printer task spent
  working during the run. Timed with the CPU's cycle counter.
- **The stages**, each counted once (time in a stage it calls goes to that stage):
  **planner** (look-ahead and speed planning, `planner.cpp`), **shaping** (input shaping and pressure
  advance on the 2 ms grid, `shaping.cpp`; 0 when neither is configured), **step times** (each step's
  board clock), **compression** (into Klipper's `queue_step` commands), **sending** (turning the
  command text into the board's binary messages and, on a real board, sending them and waiting for
  the acknowledgement: so on a real board it includes waiting, not only work).
- **everything else:** reading the file, parsing G-code, heaters and fans, the web status.
- **Sends:** how many, how many during planning (a long planning burst sends what is due every 20 ms),
  the longest gap between two sends (the steps due soonest wait that long: past about 100 ms it gets
  tight), the longest single send, and the board's move queue (most commands in it, times it was full).
- **If it stops early**, "why:" gives the reason; for a move refused as too late it says how long
  nothing had been sent, what the planner was doing, and how full the board's queue was.

The first QEMU runs already point at something: building and encoding command text costs more
than the motion maths. That is what the next speed work should look at, after real-chip numbers.

## The INT32/64 test build (CAPTAIN)

In `firmware/prebuilt-esp32s3/test-int32/`:

| File | What it is | How |
|---|---|---|
| `CAPTAIN-INT32-TEST-full-image-usb.bin` | the test build, full image | Flasher over USB, "own file" (offset 0x0, set by itself) |
| `CAPTAIN-INT32-TEST-wifi-update.bin` | the test build, app only | Update page, over WiFi |
| `CAPTAIN-STANDARD-wifi-update.bin` | the standard CAPTAIN build, app only | Update page, to go back |

The standard full image is the normal CAPTAIN one (`../skipper-esp32s3-captain-merged.bin`, or the
Flasher's "Flash for my board"). Settings and printer.cfg are kept either way. Which build is running:
the first line of "Important events" on the Log page says STANDARD or INT32/64 TEST BUILD.

What is integer in it:

- **input shaping and pressure advance** (`MotionShaperFx`, `host/src/shaping_fx.cpp`): times in
  microseconds, positions in steps with 16 fraction bits, each segment's slope worked out once so a
  position is a multiply and a shift; shaper amplitudes in 16 fraction bits summing to exactly 1. The
  pressure-advance smoothing integral uses hardware single-precision float on small, local numbers.
- **step times**: the board clock of each step and the interval between steps, with a 32-bit divide
  (hardware on the ESP32-S3) for spans up to 5.4 ms, which covers the 2 ms grid and 5 ms segments.

The **motion benchmark** (Tune page, "Run") uses the same shaper as the printer in each build since
2026-10-29 (before that it always used the `double` one, so the two builds gave the same figures).
With input shaping and pressure advance it is the better comparison of the two maths: the Tina 2's
printer.cfg has no input shaper, so the print benchmark's "shaping" is 0 there. Standard build on the
Tina 2: 22.1 us per step, 19 % of one core at 100 mm/s; 15.3 us, 23 % at 200 mm/s.

What is still `double`: the planner (once per 5 ms segment and per move), and turning host time into
board clock (once per segment). The start-up line in the important-events log, the top of
`/report.txt` and every benchmark line say which maths a build uses.

**Checked on the PC:** the whole test suite runs twice, once on each maths (`ph_tests` and
`ph_tests_fixed`, 1,814 checks each); and the integer shaper is compared with the `double` one at
every grid point: never more than one step apart, 0.2 % of grid points differ at all, and the same
total steps.

## The important-events log

The Log page has two tabs: **Scrolling log** (everything, as before) and **Important events**, laid
out in columns (time, time since start-up, event; each clause of a long event on its own line, the
benchmark's figures in a grid): one entry per start-up (with the reason for the last restart), crash, printer board connect or loss,
shutdown, move refused as too late, print end, benchmark result. With a timestamp (the browser's
clock, once a page has been opened) and the time since start-up. **Copy** and **download** buttons;
`GET /report.txt`. Kept in memory (6 KB, oldest lines dropped) and in `skipper/report.log` on the SD
card (256 KB, then `report.1.log`).

## Testing under QEMU (maintainers)

QEMU test builds (`-DSF_EXTRA_DEFAULTS=<file>`) need, besides the console on UART0 and no SD card:
`CONFIG_SKIPPER_HOST_ADC=n` (QEMU has no ADC: the ADC driver's start-up calibration would wait
forever), `CONFIG_FREERTOS_UNICORE=y` (QEMU 9.0 shows the heap region at 0x3FCE9710 as damaged from
core 1, also with older firmware), and `CONFIG_SKIPPER_BENCH_AT_BOOT=y`, which runs the motion
benchmark and then `SF_BENCH FILE=builtin` at start-up and skips WiFi (QEMU has none, nor PSRAM).
