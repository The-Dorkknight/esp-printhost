# The segment queue ("segq"): what it was and why it was removed

Removed 2026-10-25, at the author's request: **the ESP32-S3 firmware is to be the only bespoke
firmware in this project; the printer board runs plain Klipper.**

## What it was

An experimental add-on to Klipper's MCU firmware, written 2026-10-03 to 10-05, before the host
existed. Instead of Klipper's `queue_step` (the host works out every step time and packs them into
"N steps, starting interval, change per step" commands), the host would send constant-speed
**segments** ("over the next n DDA ticks, X moves 40 steps and Y 12") and a timer on the board,
running at 8 to 40 kHz, would decide on every tick which motors step (a Bresenham DDA). The idea
was less work for the host and less traffic on the wire.

Pieces: `firmware/skipper/segq_core.[ch]` (the DDA algorithm, C99), `firmware/skipper/segq.c` (Klipper
glue: five commands, two responses, one timer), an overlay in `host/tools/skipper_fw.py` that copied
the three files into Klipper's `src/`, added a `WANT_SEGQ` option to `src/Kconfig` and a line to
`src/Makefile`, the Flasher's "Segment queue" box, `firmware/test/` (a PC simulation of the glue
against a mock of Klipper's API, and an AVR check that the core needs no 64-bit helper calls), and
the simulated board in `host/tools/fake_mcu.py`. Cost on the board: +920 bytes flash on RP2040,
+1981 on STM32F103, about +3.5 KB on AVR.

## Why it went unused, then removed

- **The host never used it.** When motion was built (2026-10-13) it sent Klipper's own
  `queue_step`, `trsync` and `endstop_home`, so homing, endstops, the emergency stop and Klipper's
  timing checks ("Timer too close", "Stepper too far in past") all work as Klipper designed them.
  segq v1 had no endstop homing and no synchronised pin actions.
- **Klipper's step packing turned out cheap enough.** Measured on a curvy print path: 320
  `queue_step` per second plain, 426 with input shaping (about 5 KB/s of the 25 KB/s link). The
  2026-10-24 compressor needs 1.27 us per step in QEMU.
- **It capped speed:** at most one step every two DDA ticks per axis (20,000 steps/s at 40 kHz,
  4,000-5,000 on AVR).
- **It was never run on hardware.** The SKR Pico in the Tina 2 ran a build containing it from
  2026-10-10, but nothing ever configured it, so its timer never started (everything in
  HARDWARE_LOG.md from that board was plain Klipper code paths).
- **Plain Klipper is easier to trust and to update**: no "-dirty" version, no overlay to keep in
  step with Klipper releases, and any Klipper build for the chip works.

## What changed on 2026-10-25

- `skipper_fw.py` builds Klipper v0.13.0 unmodified. A Klipper folder that still has the old overlay
  is cleaned automatically (Kconfig and Makefile restored from git, the three files deleted), and a
  build is refused if Klipper's `src/` has any other local change (`git status`). New command
  `skipper_fw.py clean`; `--no-segq` is still accepted and ignored; `overlay` is gone.
- All six shipped printer-board firmwares were rebuilt: they now report `v0.13.0-0-g61c0c8d` (no
  "-dirty"), and a test checks that their dictionaries contain no `segq` command. `build-info.json`
  says `"plain_klipper": true`.
- Board presets lost `dda_hz`, `dda_note` and `build_sizes`; the Flasher lost the "Segment queue" box.
- `firmware/skipper/` and `firmware/test/` deleted; `segq.c` (the Klipper glue) is gone.
- `segq_core.[ch]` moved to `host/test/` as a **test-only** step replay: `test_engine.cpp` still runs
  the planner's segments through it to check speeds and positions without trusting the planner.
  It is not built into any firmware (the ESP32 build no longer compiles it either).
- `fake_mcu.py` speaks plain Klipper only; its segq tests and `test_segq_protocol_sync.py` were removed.
- The old code is in the git history of the 0.1.0-wip releases up to 2026-10-24.

**Your SKR Pico** still runs the 2026-10-10 build with the idle segment queue in it. That is
harmless and nothing needs doing; reflash `skr-pico-uart` (Flasher, Slave tab, hold BOOT) whenever
convenient to get the plain build. The ESP32 configures the board the same way either way.

## A. The protocol as it was (v1, archived from PROTOCOL.md)

Replaces Klipper's `queue_step` / stepcompress for our boards. The host plans
moves and sends constant-velocity **segments**; the MCU runs a DDA that turns
them into step pulses. The MCU side is `firmware/skipper/segq_core.[ch]`
(portable algorithm, unit tested on a PC) plus `firmware/skipper/segq.c`
(Klipper glue). `tools/fake_mcu.py` simulates it for host tests.

The firmware announces `SEGQ_PROTOCOL=1`, `SEGQ_MAX_DEPTH=255` and `SEGQ_AXES=4`
in the data dictionary's `config` section. A host must check `SEGQ_PROTOCOL`.

#### Commands (host -> MCU)

```
config_segq oid=%c period=%u depth=%c start_threshold=%c
config_segq_axis oid=%c axis=%c step_pin=%u dir_pin=%u invert_step=%c invert_dir=%c
segq_push oid=%c n=%u x=%i y=%i z=%i e=%i flags=%c tag=%hu
segq_stop oid=%c
segq_query oid=%c
```

| command | meaning |
|---|---|
| `config_segq` | Create the queue (once, during config, after `allocate_oids`). `period` = length of one DDA tick in MCU clock ticks (`CLOCK_FREQ / dda_hz`), 1..2^30 (larger values break Klipper's wrapping time compares and are rejected). `depth` = queue length in segments, 1..255 (RAM: depth x 24 bytes on ARM, 23 on AVR). `start_threshold` = how many segments must be queued before an idle queue starts (0 or 1 = at once, must be <= depth). A segment with the LAST flag also starts it. |
| `config_segq_axis` | Attach pins to an axis: `axis` 0..3 = X, Y, Z, E. `invert_step` / `invert_dir` = 1 flips the pin level. Axes never configured are simply never stepped (their motion is dropped, but still counted in `segq_position`). Direction pins start at the "positive" level. |
| `segq_push` | Append a segment: move each axis by exactly the signed step count over `n` DDA ticks (sign = direction; E may be negative). |
| `segq_stop` | Drop everything queued and stop now (pause / emergency). The position is kept. |
| `segq_query` | Ask for status and position; answered by `segq_status` **and** `segq_position`. |

`tag` of every segment is echoed in `segq_status` once it starts. `flags` of
`segq_push`: bit0 `SYNC` (0x01) is accepted but **has no effect in v1** (no pin
action); bit1 `LAST` (0x02) marks the
end of a motion burst, so running dry after it is normal (not an underrun).

Segment rules (checked on the MCU; also by `fake_mcu.py`):

- `1 <= n <= 2^30` DDA ticks.
- `2 * |steps[axis]| <= n` for every axis. Max step rate is therefore half the
  DDA rate (`dda_hz / 2` per axis); every step pulse is one DDA tick high and at
  least one tick low.
- A segment moves each axis by exactly its step count (no rounding drift).
- The first tick of a segment never steps, so the direction pin has a full tick
  to settle.

#### Responses (MCU -> host), both sent in answer to `segq_query`

```
segq_status oid=%c free=%c active=%c tag=%hu underruns=%hu
segq_position oid=%c x=%i y=%i z=%i e=%i
```

- `free` free queue slots (capped at 255), `active` 1 while a segment is
  executing or queued or a step pulse is still high, `tag` = tag of the segment
  being executed (last started), `underruns` = times the queue ran dry in the
  middle of a burst (no LAST flag) since config.
- `x..e` absolute executed step position since config.

#### Errors

An invalid segment (rule violation) or a push into a full queue (a host
flow-control bug: check `free` before pushing) makes the MCU call Klipper's
`shutdown()` with a clear message ("segq: invalid segment ...", "segq: queue
overflow ..."). The same applies to bad `config_segq` / `config_segq_axis`
arguments (including a second `config_segq` for an oid in use) and to a DDA timer that cannot keep up ("segq DDA timer too slow").
After a shutdown all pins go to their idle level and the board must be reset.

#### Timing model

One Klipper timer runs every `period` clock ticks **only while there is work**:
it starts when a push arrives while idle and stops when the queue is empty, no
segment is executing and no pulse is high. Each tick first lowers the step pins raised on the previous tick,
then raises the pins of the axes stepping now. If the MCU cannot finish a tick
before the next is due, it shuts down instead of stepping late. The host should
keep roughly 100 ms of segments queued; a segment is a few tens of bytes on the wire, so
200 segments/s is well within 250 kbaud.

#### Not in v1

- **Sync pin actions** (`segq_pin_on_tag`: fan/heater changes lined up with a
  segment): not implemented. `SYNC` only echoes the tag in `segq_status`.
- **Endstop homing in the DDA** (`segq_endstop_arm` / `segq_endstop_hit`): not
  implemented. Use Klipper's stock endstop/trsync commands or a separate move.
- **Host-timeout watchdog**: not implemented in the segq glue (Klipper's own
  clocksync/shutdown behaviour applies).
- Pressure-advance flags (`RESERVED_PA`): reserved for a later version.

#### Status

Compile-tested for RP2040, STM32F103 and AVR (ATmega2560/1284P); **never run on
hardware**. The unit tests cover the algorithm core on a PC and the wire format
via `fake_mcu.py`; timing, pin polarity and ISR load are unverified.
