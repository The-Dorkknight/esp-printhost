# Task cards

Each card is sized for one Gemini (or Claude) chat. Paste the card plus the
files it names — nothing else. Every card must end with `ph_tests` passing.

Status: ✅ done · 🟡 next · ⬜ later · 🧠 needs Claude (architecture/tricky)

## Done (this session)

| # | Task | Files |
|---|---|---|
| ✅ T0 | Wire format: CRC16, VLQ, framing, stream decoder w/ resync | `wire.*` |
| ✅ T1 | Data dictionary JSON + enumerations (pin ranges) | `json_mini.*`, `msgproto.*` |
| ✅ T2 | Text command encoder, response decoder | `msgproto.*` |
| ✅ T3 | `printer.cfg` parser with typed getters + error list + unused-key check | `config_ini.*` |
| ✅ T4 | G-code line parser (comments, checksums, text args) | `gcode.*` |
| ✅ T5a | Drift-free mm→steps accumulator | `motion.*` |
| ✅ T5b | Pseudo pressure advance (smoothed, clamped, extrusion-conserving) | `motion.*` |
| ✅ T6 | PC probe tool + fake MCU (identify, uptime, pin blink, NAK recovery) | `tools/` |

## Next — hardware PoC on your laptop (no ESP32 yet)

**🟡 H1 - Probe a real board.** Flash stock Klipper to an old board
(`make menuconfig` → your MCU, serial or USB). Stop anything else using the
port, then:
```
cd host && cmake -S . -B build && cmake --build build
./build/mcu_probe /dev/ttyUSB0 250000 --pins --blink <pin>
```
Success = MCU name printed and the pin toggles.

Status (2026-10-01): **protocol half proven.** Against a BTT SKR Pico (RP2040,
Klipper v0.10.0-181) the probe identified the board, read its 31 pins, reset
it, configured `gpio24` and sent 10 toggles with 0 CRC errors. The physical
pin change has **not** been confirmed: `gpio24` is, as far as we can tell from
BTT's pin map, the NeoPixel (RGB) header, which a plain on/off will not light.
Next: repeat with a pin that has an LED or a multimeter on it.

## Gemini cards (small, well-specified)

**⬜ G1 — `output` debug messages.** In `MsgProto::load_dictionary`, parse the
`"output"` map (printf-style formats like `"Stepper %c too far"`). Extract the
`%`-specifiers in order, register as responses with name `#output` and
params `arg0..argN`. Add a test. Files: `msgproto.*`, `test_main.cpp`.

**⬜ G2 — Link session class.** Move `Link` out of `tools/mcu_probe.cpp` into
`host/include/ph/link.h` + `src/link.cpp` with an abstract
`struct Transport { virtual int write(const uint8_t*, size_t)=0;
virtual int read(uint8_t*, size_t, int timeout_ms)=0; }`. Keep NAK/ack logic.
Add a unit test using an in-memory loopback transport. Files: `mcu_probe.cpp`.

**⬜ G3 — Heater PID + runaway check.** Pure C++ class, no I/O:
`update(temp_c, dt) -> pwm 0..1`, anti-windup, derivative on measurement.
Runaway: if target set and temp doesn't rise ≥ 2 °C within 20 s of heating,
or deviates > 15 °C from target for > 30 s once reached → `fault()`.
Tests with a simulated first-order hotend. New files `heater.*`.

**⬜ G4 — Thermistor table.** Beta-model (`beta`, `r25`, `pullup`) → ADC
fraction ↔ °C both ways, plus the "Generic 3950" and "EPCOS 100K" presets.
Must match Klipper's `thermistor.py` to 0.5 °C over 20–300 °C. New `thermistor.*`.

**⬜ G5 — Printer config loader.** Using `Config`, build a `PrinterCfg`
struct: 4 steppers (step/dir/enable pins, `rotation_distance`, `microsteps`,
`full_steps_per_rotation`, min/max, endstop pin, homing speed/dir),
extruder + bed heater (pins, sensor type, PID values, min/max temp),
fans, `pressure_advance`. Validate and return all errors at once.
Files: `config_ini.h`, DESIGN.md.

**⬜ G6 — Command builder.** From `PrinterCfg` produce the list of Klipper
text commands (`allocate_oids`, `config_stepper`, `config_digital_out`,
`config_analog_in`, `config_pwm_out`, `finalize_config crc=<crc32 of list>`).

## Needs Claude (🧠)

- **C1 — FluidNC fork plan:** where to cut `Stepper`/`st_prep_buffer`, how
  segments leave the planner, extruder axis handling, E direction per segment.
- **C2 — MCU segment executor:** the `segq_*` commands in Klipper's `src/`
  (new `src/segq.c`), DDA timer ISR, endstop-during-segment, watchdog.
  AVR performance check.
- **C3 — ESP32 transport:** UART + USB-host CDC (CH34x/CP210x) under ESP-IDF,
  miniz inflate, task/core layout.
- **C4 — Web layer:** ESP-IDF HTTP server, config editor with validate +
  last-known-good rollback, upload, status websocket.
- **C5 — Auto-flash:** STK500v2 (AVR) and STM32 ROM bootloader over UART.
