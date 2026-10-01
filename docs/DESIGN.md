# esp-printhost — design

Goal: revive old donated 3D printers. A cheap ESP32-S3 (N16R8) becomes the
"brain" (WiFi, web UI, SD, text config, planning). The printer's original
board (ATmega2560, STM32F1/F4, GD32, LPC, RP2040...) becomes a dumb slave.

## Split

```
 G-code (SD / web upload)
        |
 [ESP32-S3 host]  core 0: WiFi, HTTP/websocket, SD, config, G-code reader
                  core 1: planner (FluidNC/grbl) -> segment prep -> link
        |  serial / USB-CDC, Klipper-style framing
 [MCU slave]      Klipper MCU firmware fork + segment executor (DDA)
        |
 steppers, heaters, fans, endstops, thermistors
```

- Host sends short constant-velocity **segments** (a few ms each), not steps.
- MCU plays segments back to back with a multi-axis DDA.
- No precise clock sync needed: host only watches MCU buffer fill level.
- Heater/fan changes that must line up with motion go in the segment stream
  as sync markers (flag bit).

## Donors

| Part | Source |
|---|---|
| ESP32 foundation, WiFi, web, SD, planner, segment prep | FluidNC (fork) |
| MCU HALs, scheduler, framing, ADC, safety shutdown | Klipper MCU firmware (fork) |
| Protocol reference | Klipper `msgproto`, Gopper `protocol/` |

## v1 scope

In: cartesian, 1 extruder, hotend + bed, 2 fans, Z endstop homing, SD printing,
web upload/edit of `printer.cfg` (INI), PID autotune, thermal runaway.

Out (v2+): input shaping, bed mesh/probes, arcs, macros, multi-MCU, LCDs,
CoreXY (maybe v1.5), full pressure advance smoothing.

Pseudo pressure advance (in `host/src/motion.*`) is written already
and can be switched on once basic printing works (`M900 K`).

## Non-negotiable safety

- Host thermal runaway / heater verification.
- MCU-side min/max temp shutdown (Klipper `config_analog_in` limits).
- MCU shuts heaters off if host goes silent (watchdog on segment/heartbeat).

## Code in this repo so far

`host/` is portable C++17 with no ESP-IDF dependency, unit-tested on PC.
It builds as an ESP-IDF component too (see `host/CMakeLists.txt`; the ESP-IDF
side is **untested**, only the PC build is exercised).

- `wire.*` - Klipper wire format: CRC16, VLQ, block framing, resyncing stream decoder
- `json_mini.*`, `msgproto.*` - data dictionary, enumerations (pins, shutdown strings), command encode / response decode
- `config_ini.*` - `printer.cfg`-style parser with typed getters and an all-errors-at-once report
- `gcode.*` - G-code line parser
- `motion.*` - mm to integer steps with no drift (`StepAccumulator`), pseudo pressure advance (`PressureAdvance`)

PC tools in `host/tools/`: `mcu_probe` (talks to a real Klipper MCU), `fake_mcu.py`
(pretend MCU for testing without hardware), `printhost_gui.py`, `build.py`.
The top-level `printhost.py` sets everything up in one command.

Nothing here drives a stepper or heater yet. The most the PC tools do to a
board is identify it, reset it, and toggle one output pin.
