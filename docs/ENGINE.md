# Skipper Flipper motion engine (host side)

Files: `host/include/ph/planner.h`, `host/src/planner.cpp` (look-ahead planner),
`host/include/ph/engine.h`, `host/src/engine.cpp` (G-code interpreter),
`host/test/segq_sim.h` + `host/test/test_engine.cpp` (tests). The planner's segments go to the
toolhead, which turns them into Klipper `queue_step` commands.
Work in progress (0.1.0-wip). Nothing here has run on hardware.

## Pipeline

G-code line -> `Engine::feed_line` (uses `gcode_parse`) -> `Planner::add_move` ->
look-ahead queue -> constant-velocity `ph::Segment`s -> sink callback or
`Engine::drain()`. Output is an ordered stream of `Item`s: a `Segment` or an `Action`.

## Planner parameters (`PlannerConfig`)

| field | default | meaning |
|---|---|---|
| `steps_per_mm[4]` | 80,80,400,93 | X, Y, Z, E |
| `max_velocity` / `max_z_velocity` / `max_e_velocity` | 300 / 20 / 60 mm/s | XY component, Z component, filament |
| `max_accel` / `max_e_accel` | 3000 / 5000 mm/s^2 | path accel; E-only moves and E limit inside mixed moves |
| `junction_deviation` | 0.05 mm | Grbl style corner speed |
| `min_junction_speed` | 0.1 mm/s | floor for corners and reversals (must be > 0) |
| `dda_hz` | 20000 | MCU DDA tick rate |
| `segment_ms` | 5 | target segment duration |
| `step_rate_fraction` | 0.45 | per-axis step-rate cap as a fraction of `dda_hz` (core limit is 0.5) |
| `lookahead` | 256 | max queued blocks (see below) |

With the defaults the per-axis cap is 9000 steps/s, i.e. 112.5 mm/s on a 80 steps/mm axis;
feeds above that are clamped, never rejected. M204 S is clamped to `max_accel`.

## How it works

* **Blocks.** A move is a straight line in (X,Y,Z); length is the XYZ distance. E rides along
  (its distance is part of the move). Z-only moves use their own limits; E-only moves
  (retracts) are a separate block kind, length = |dE|, limited by `max_e_*`.
  Nominal speed is the minimum of feed (F, times M220), the per-axis velocity limits and the
  step-rate cap; accel is limited so E stays under `max_e_accel`.
* **Junctions.** Grbl junction deviation: `v^2 = a*jd*s/(1-s)`, `s = sin(theta/2)`. Collinear
  blocks are limited only by nominal speeds; reversals and XYZ<->E-only changes go to
  `min_junction_speed`. Backward + forward pass over the queue on every added block.
  The oldest block is released when the queue is full or when enough path is queued behind it
  to brake from its nominal speed (a block count alone would cap thousands of tiny moves at
  ~sqrt(2*a*queued length)). `flush()` releases everything with the last exit speed 0.
* **Segments.** Each block is an exact trapezoid/triangle. Its duration is rounded to whole
  ticks with the rounding error carried to the next block; it is cut into
  `round(T/segment_ticks)` near-equal segments. Segment boundaries sample the exact profile;
  per-axis steps are `llround(absolute position * steps_per_mm)` minus a global per-axis
  integer step counter, so there is no cumulative drift. The last segment of a block lands on
  `llround(end * spm)` exactly.
* **Validity.** If rounding would break `2*|steps| <= n` (only possible for segments of a few
  ticks) the segment is *lengthened* to `n = 2*max|steps|` rather than split (splitting a few-tick
  segment cannot reduce its step rate). `stretched_segments()` counts these.
* **SEG_LAST** is set on the final segment after `flush()` / `dwell()`, and only then.
  One segment is held back inside the planner so the last one can be marked.
* **Extruder.** E steps follow the move. With `M900 K != 0` the existing `PressureAdvance`
  runs per segment; on `flush()` its residual is released and a rate-limited correction
  segment forces the total to `llround(E * spm)`. M900 flushes first.
* **Dwell.** `G4 P/S` flushes, then emits zero-step segments, at most 1 s each.
* **Actions** (M104/M106/M107/M140 do not stop motion; M109/M190/M84/M18/G28 flush first)
  are queued as markers in the planner, so they appear after every segment of earlier moves
  and before any segment of later ones.

## Engine G-code semantics

G0/G1 (X Y Z E F, F persists), G4, G21 (accepted), G28 (flush, Action, then the logical
position of the named axes - all XYZ if none - is set to 0; the planner's step counter is NOT
changed, so the host runtime must resync the planner/MCU position itself if homing moves it),
G90/G91, G92 (X Y Z shift the origin, E sets the logical E; no motion, no flush; bare G92 is
a no-op), M82/M83 (E is relative if G91 is active OR M83 is active), M204, M220, M221 (scales
E deltas), M400, M900 K. Unknown commands, G20, tool changes, bad M220/M221/M900 arguments,
parse failures and bad checksums add an entry to `warnings()`; nothing throws.

## Testing

`host/test/test_engine.cpp` runs every program through `segq_core` (a test-only step replay since 2026-10-25, `segq_sim.h`,
queue depth respected) and checks: exact final step position on all axes (square, fuzz,
10,000 x 0.01 mm drift test); motion profile reconstructed from step timestamps only (peak,
~10 ms-window acceleration, cruise, triangle, duration within tolerance); collinear / 90
degree / reversal junction speeds; 40 random programs (absurd feeds, relative/absolute,
retracts, tiny moves, dwells, with and without pressure advance) all accepted by
`segq_core_push`; axis limit clamping; G-code state machine; action ordering and SEG_LAST;
PA total E exact; dwell tick totals; determinism; speed (100k small moves plan in ~0.2 s on
the dev box). Deliberately broken planners (no corner slowdown, reversal not stopping,
per-move rounding, no stretch, no PA correction, doubled accel, higher step cap) fail the tests.

## Not verified / known limits

* No hardware. Real MCU timing, UART/link bandwidth and the ESP32's CPU speed are untested
  (the sim assumes the queue is refilled instantly). Only compiled with `-fno-exceptions
  -fno-rtti` on the PC, never under ESP-IDF.
* Tiny moves give about one segment per move (a 0.1 mm chord at 60 mm/s is ~1.7 ms), which
  may exceed what the link can carry; there is no segment coalescing yet.
* A few-tick segment can run up to ~25% faster/slower than nominal (tick rounding, stretch);
  averages are right. Extruder speed jumps at junctions are only softened by PA smoothing.
* Reconstructed-acceleration tolerances (15%) are loose because of step quantisation.
* The G-code parser stores values as `float`; very long relative sequences of non-dyadic
  values can accumulate sub-step error before the planner's rounding sees them.
* Position is absolute machine mm in a double; G28 resync is left to the caller.
