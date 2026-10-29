# Tasks

Status: ✅ done · 🔬 works on real hardware · 🟡 next · ⬜ later. What has run on hardware is listed in
[HARDWARE_LOG.md](HARDWARE_LOG.md).

**Overall (2026-10-14): mostly working, needs tuning and proper testing.** In a WEEDO Tina 2 with an SKR Pico: heating, PID, safety checks, motor tests, TMC2209, stallGuard triggering and the hotend fan work on hardware. The part-cooling fan and third fan output are not confirmed yet (possibly a broken wire; will update when tested). Moves, homing and printing are simulated only.

## Done

| # | Task | Files |
|---|---|---|
| ✅ T0-T2 | Klipper wire format (CRC16, VLQ, framing, resync), data dictionary, command encode / response decode | `wire.*`, `json_mini.*`, `msgproto.*` |
| ✅ T3-T5 | `printer.cfg` parser, G-code parser, drift-free mm-to-steps, pseudo pressure advance | `config_ini.*`, `gcode.*`, `motion.*` |
| ✅ T6 | PC probe + fake MCU (identify, uptime, pin blink, NAK recovery) | `host/tools/` |
| ✅ T7 | Segment-queue core (C99). **Retired 2026-10-25** from the printer board (never used by the host; the board now runs plain Klipper); kept only as a test-only step replay | `host/test/segq_core.*`, [research/segq-retired.md](research/segq-retired.md) |
| ✅ T8 | Motion planner + G-code engine, verified through the real core | `planner.*`, `engine.*` |
| ✅ T9 | Klipper v0.13.0 glue + overlay + board presets (compile only, except the link test below) | `firmware/` |
| ✅ T10-T11 | Build tool and Flasher (Slave + Master tabs), shipped firmware, SHA-256 checks | `skipper_fw.py`, `skipper_esp.py`, `skipper_flasher.py`, `fw_panel.py`, `master_panel.py` |
| 🔬 T12 | ESP32-S3 firmware: WiFi AP, status page, live console, settings, status LED | `firmware/esp32s3/` |
| 🔬 T13 | Read-only Klipper link over UART: `ph::KLink` (identify, dictionary, `get_config`), `skr-pico-uart` preset, wiring guide, UART diagnostics | `host/src/klink.cpp`, `firmware/esp32s3/main/link.cpp`, `docs/WIRING_SKR_PICO.md` |

## Next: real Klipper behaviour on the ESP32-S3

Each stage is tested on the PC against `fake_mcu.py` first, then on the SKR Pico. The heater code
(K6) only exists together with its safety checks. The test bench (`/test` on the ESP32-S3) runs
every subsystem with an Abort button each: see [TEST_BENCH.md](TEST_BENCH.md).

| # | Stage | What "done" looks like |
|---|---|---|
| 🔬 K3 | **First physical action: the board's NeoPixel** (moved ahead of K1/K2: a bare SKR Pico has no fan). The ESP32-S3 configures the board like Klipper (`allocate_oids`, `config_neopixel`, `finalize_config` with a zlib CRC-32; an already-configured board with the same CRC is left alone, a different one is reset and reconfigured) and sets colours with `neopixel_update` / `neopixel_send` (retried like `neopixel.py`). Buttons on the status page. Tested on the PC against the simulated board and **works on the SKR Pico** (2026-10-11). | The SKR Pico's RGB LED changes colour from the phone. |
| 🔬 K1 | **Clock sync.** `ph::ClockSync` polls the board's clock (`get_uptime` once, then `get_clock` every second), rejects slow round trips and fits frequency and offset with an exponentially weighted regression like Klipper's `clocksync.py`; 32-bit clock wraps are handled. PC test: a simulated 50 ppm drift is measured within 3 ppm across several wraps; against the simulated board it measured 72,000,485 Hz. **On the SKR Pico (2026-10-11): 12,000,464 Hz, +38.7 ppm, stable for about 41 minutes, 2,430 samples, 0 CRC errors.** The status page shows the measured clock, drift, best round trip and sample count. | Status page shows the board clock and its measured drift; stays stable for an hour. |
| 🔬 K2 | **Configure the board from `printer.cfg`.** `printer.cfg` is stored on the ESP32-S3 (NVS, 4 KB max; since 2026-10-27 at the end of the gcode partition, 8 KB DECKHAND / 16 KB CAPTAIN and SKIPPER) and edited at `/config`; saving re-identifies and reconfigures the board. So far only `[neopixel NAME]` sections build board commands (`pin` checked against the board's own pin list, `chain_count`, `color_order`, `initial_RED/GREEN/BLUE/WHITE`); other sections are listed as "not used yet". Errors are shown with line numbers and stop before the board is touched. Default: the SKR Pico LED. `config_digital_out`, `config_analog_in` etc. come with K3b/K4. | Board reports "configured"; a config error is shown on the page with the line number. |
| 🔬 K3b | **G-code console** on the status page and the test bench (`/gcode?cmd=`). On hardware so far: `SET_LED`, `M105`, `HELP`. Built and simulated, not yet run on hardware: `M104/M109/M140/M190`, `SET_HEATER_TEMPERATURE`, `TURN_OFF_HEATERS`, `M106/M107` (with Klipper's kick start), `M18/M84`, `M112`, `FIRMWARE_RESTART`, `STEPPER_BUZZ`, `DUMP_TMC` and the test commands. `M109`/`M190` do not wait in the console. Still to do: `SET_PIN` / `[output_pin]`. | A fan switches from typed G-code on real hardware. |
| 🔬 K4 | **Temperatures, read-only.** Tested on the PC (simulated board) and **working on hardware** (hotend ~24 C, bed ~21 C on an SKR Pico; no reference thermometer check yet). `[temperature_sensor NAME]` (`sensor_type`, `sensor_pin`, `pullup_resistor`, `inline_resistor`, `min_temp`, `max_temp`) and `[thermistor NAME]` in `printer.cfg`; `config_analog_in` goes in the config, `query_analog_in` (8 samples of 1 ms, report every 0.3 s, as Klipper's `adc_temperature.py`) is sent after it like Klipper's init commands, and `analog_in_state` reports become degrees C with Klipper's `thermistor.py` maths (Steinhart-Hart from three points, or beta). Named sensors with Klipper's exact values: Generic 3950, EPCOS 100K B57560G104F, ATC Semitec 104GT-2, NTC 100K MGB18-104F39050L32. Open and short circuits are reported as such. Shown on the status page, `status.json` and `M105`. Other sensor types are listed as not supported yet. | Hotend and bed temperatures read sensibly at room temperature (18 C indoors on 2026-10-11 is the reference). |
| 🟡 K5 | **Motors, moves, homing.** On hardware 2026-10-14: motor tests, TMC2209 and stallGuard triggering (tuning needed). Built 2026-10-13, simulated only: `G0/G1/G4/G28/G90/G91/G92/M82/M83/M114/M204/M220/M221/M400`, `JOG`, `PROBE`, `SET_KINEMATIC_POSITION`, `QUERY_ENDSTOPS` (`ph::Toolhead`): the project's look-ahead planner, steps compressed into Klipper's `queue_step` within 25 us, homing with `endstop_home` + trsync, TMC2209 sensorless homing set up like Klipper's tmc.py, axis limits, `min_extrude_temp`. Earlier: Done (simulated, not yet on hardware): `[stepper_*]` and the extruder's stepper configure the board (`config_stepper`, `config_trsync`, enable pin), `[tmc2209]` drivers are set up over the TMC UART with Klipper's exact register values (checked against Klipper's own code), and a motor test schedules steps directly (`queue_step`): CW, CCW, then Klipper's buzz pattern, stopped at once by a trsync trigger on Abort. On hardware 2026-10-13: the motor test turns the motor. Still to do: `G1` moves from the planner (segment queue or `queue_step`), kinematics, homing (endstops). | One motor turns, the board's step counter returns to zero; then a commanded `G1` on one axis. |
| 🔬 K6 | **Heaters, with Klipper's safety.** Working on hardware 2026-10-14 (PID tune "works quite well"; needs longer testing). First hardware run 2026-10-13: heats, but overshot to an "ADC out of range" shutdown with the untuned sample PID (see HARDWARE_LOG). Added since: `PID_CALIBRATE` + `SAVE_CONFIG`, heater log. Built and simulated: `[extruder]` / `[heater_bed]` with Klipper's PID (or watermark) code, `max_duration` 5 s on the heater pins (the board turns a heater off by itself if the ESP32-S3 stops refreshing it: tested by killing the host in the simulator), the board's own ADC range check from `min_temp`/`max_temp`, `verify_heater` ("not heating at expected rate"), host-side `max_temp`, `M112`; `[heater_fan]` comes on above 50 C. Test: heat to a target, hold, switch off. | The hotend reaches 75 C, holds it within a few degrees for 60 s and switches off; pulling the ESP32-S3's power while heating makes the board shut the heater off within 5 s. |
| 🟡 D1 | **Little OLED** (2026-10-23): SSD1306 / SSD1315 / SH1106 on I2C, from KlipperTFT-Pico; temperatures, progress, HEATING / HOMING / EXTRUDING / LOADING..., NO BOARD with WiFi name and password ([OLED.md](OLED.md)). Built, PC-tested, QEMU start checked. | The screen shows the hotend temperature and HOMING on the Tina 2. |
| 🟡 B1 | **Buttons on the case** (2026-10-26, CAPTAIN+): touch pads or switches, Preheat / Home hold / Unload / E-stop hold ([BUTTONS.md](BUTTONS.md)). Built, PC-tested. | A bolt on the Tina 2's case preheats the hotend. |
| 🟡 Q1 | **QIDI X-Smart 3 test bench** (2026-10-27): CoreXY, Z homing on the probe, `gear_ratio`, chamber thermistor on the ESP32 (CAPTAIN+), config and wiring guide ([configs/qidi-xsmart3-skr-pico.md](configs/qidi-xsmart3-skr-pico.md)). Built, PC-tested, simulated `G28`. | `G28` on the X-Smart 3 with the SKR Pico, then a first layer. |
| ⚪ N1 | **NEPTUNE tier (ESP32-S31)** and speed headroom: research only ([research/klipper-grade-performance.md](research/klipper-grade-performance.md)). Next steps there: measure CPU per print on the chip (SF_BENCH, 2026-10-28: built; [BENCHMARK.md](BENCHMARK.md)), compare the int32/int64 test build, then 500000 baud UART and IRAM placement. First QEMU numbers: encoding the command text costs more than the motion maths. | Wait for stable ESP-IDF support for the S31. |
| 🟡 T1 | **Tuning** (2026-10-24, CAPTAIN+): input shaping and pressure advance on a 2 ms grid, accelerometer auto-search (ADXL345 I2C/SPI, MPU-6050; USB compile-only), Klipper's resonance test and shaper fit (matches Klipper exactly), bed mesh with coloured heightmap, screw tilt, Tune page ([TUNING.md](TUNING.md)). Built, PC-tested, QEMU-checked. | A calibrated `[input_shaper]` saved on the Tina 2, and a heightmap of its bed. |
| 🟡 K7 | **Print a file.** 2026-10-20: three builds by board memory, slicer upload, home WiFi, crash reports ([TIERS.md](TIERS.md); not yet on hardware). 2026-10-19: micro SD card works on hardware (mount, list, upload; [SD_CARD.md](SD_CARD.md)). Built 2026-10-13, simulated only: upload on the Printer page (compressed in the browser, stored in a flash partition, 508 KB now, unpacked line by line with the ROM inflater), `SDCARD_PRINT_FILE` / `PAUSE` / `RESUME` / `CANCEL_PRINT`, M109/M190 waits, idle timeout. Preset and guide for a WEEDO Tina 2: [TINA2_INSTALL.md](TINA2_INSTALL.md). 2026-10-24 (CAPTAIN+, simulated): arcs, firmware retraction, bed mesh, pressure advance, exclude object, tuning tower. | A calibration cube on the Tina 2. |

## Building blocks still to write (small, well-specified)

- ~~**G3 Heater PID + runaway check.**~~ Done in `ph::Machine` (`host/src/machine.cpp`), following Klipper's `heaters.py` and `verify_heater.py`; tested with a simulated hotend (unit tests and `fake_mcu.py`).
- ~~**G4 Thermistor table.**~~ Done: `ph::Thermistor` (`host/src/thermistor.cpp`), the same maths as Klipper's `thermistor.py`; tests use Klipper's own fit points and round-trip 0-300 °C to 0.05 °C.
- ~~**G5/G6**~~ Mostly done in `ph::build_board_plan` (heaters, fans, steppers, TMC2209, sensors, LEDs; typos and pins used twice reported with line numbers). Left: `pressure_advance`, kinematics limits.
- **G5 Printer config loader.** From `Config`, a `PrinterCfg` struct (steppers, heaters, fans,
  `pressure_advance`) with every error reported at once. For K2.
- **G6 Command builder.** From `PrinterCfg`, the list of board config commands and the config CRC. For K2.
- **G1 `output` debug messages** in `MsgProto::load_dictionary` (`#output` responses), for clearer
  board error messages.

## Later

- Web UI proper (beyond the status page): job control, config editor with last-known-good rollback.
- Other boards over UART (Creality 4.2.x, RAMPS, Ender-3 1.1.x: 5 V logic on the AVR boards needs a
  level shifter).
- Auto-flash the printer board from the ESP32-S3 (STM32 / AVR bootloaders over UART).
- USB-host link (parked, see WIRING_SKR_PICO.md "Why UART").
