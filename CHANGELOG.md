# Changelog

Format follows [Keep a Changelog](https://keepachangelog.com/); versions follow [Semantic Versioning](https://semver.org/). Everything before 1.0 is experimental.

## [0.1.0-wip] - 2026-10-11

First public snapshot. **Proof of concept; many features may be broken.** Real-hardware results are in [docs/HARDWARE_LOG.md](docs/HARDWARE_LOG.md).

### Works on real hardware
- Clock sync, `printer.cfg` and the `SET_LED` console ran on an ESP32-S3-Zero type board with a bare SKR Pico for about 41 minutes: board clock measured at 12,000,464 Hz (+38.7 ppm), 2,430 samples, 0 CRC errors
- First real Klipper behaviour: the ESP32-S3 configures the printer board like Klipper's host (`allocate_oids`, `config_neopixel`, `finalize_config` with a zlib CRC-32, reset-and-reconfigure on a mismatch) and drives the SKR Pico's on-board RGB LED from buttons on the status page (`/board_led`). New `ph::KBoard`, binary `hex:` arguments in the encoder, `klink_probe --neopixel`, NeoPixel and config-CRC support in `fake_mcu.py`; tested end to end on the PC.
- ESP32-S3 firmware (`firmware/esp32s3`, ESP-IDF v5.4.1): WiFi access point with captive portal, status page, live web console, settings page stored on the chip, status LED (plain or WS2812, colour order setting), UART diagnostics (byte counters, last bytes, echo detection, loopback self-test)
- Read-only Klipper link over UART: `ph::KLink` (identify, data dictionary unpacked with the ROM inflater, `get_config` alive check) between the ESP32-S3 and a BTT SKR Pico running the `skr-pico-uart` firmware
- Flasher Master tab: esptool in a private virtualenv (works where Linux Mint refuses `pip install --user`), Detect chip (`flash-id`), USB log, shipped image with SHA-256 check

### Works on real hardware (cont.)
- Temperature reading from two thermistors on the SKR Pico (hotend ~24 C, bed ~21 C).

### Fixed (2026-10-14, after the first Tina 2 power-up)
- The ESP32-S3 crashed (`abort()`, out of memory) as soon as it connected to the printer board: the motion planner's 256-move look-ahead needed ~45 KB in one piece. Now 32 moves (~6 KB); the free memory is checked first and a shortage disables moves with a message instead of crashing; the USB log shows the heap before and after. Test: `test_planner_memory`.
- Status page hint for a line that only delivers 00 bytes (the board's TX held low: off, browning out or hung).
- Board connection rewritten as `ph::BoardSession` (shared by the ESP32 firmware and `klink_probe --session`, so the PC tests run the ESP32's own logic). After the SKR Pico could not be reached once it was power-cycled with the ESP32 running:
  - the ESP32 lets go of its TX wire while the board is away and only talks once the board's TX line is back high (a switched-off board is otherwise half-powered through its RX pin; likely cause, not measured);
  - a board that restarts by itself (power cycle, reset button) is noticed within about a second (get_config every second) and configured again; before, it was only noticed when it stopped answering, and the next command for an unknown object shut it down;
  - a board still configured from before (the ESP32 restarted) or shut down is always reset first, for a clean start;
  - "reset" is sent with every sequence number and a leading SYNC byte; a board that ignores it is reported ("power-cycle it") and retried, instead of the ESP32 giving up after three tries and staying stuck;
  - a lost board is detected in about 3 s (was about 15 s), and a line held low at once;
  - retries start with a SYNC byte, so a board that saw line noise resynchronises at once;
  - the console answers while the ESP32 is searching for the board.
- `STALLGUARD_TEST STEPPER=<name> [SPEED=] [DIST=]` and a "StallGuard" row per sensorless axis on the Test bench (sensorless homing did not trigger on the Tina 2): runs the motor forward and back at homing speed in the homing set-up, reads `SG_RESULT` (and `TSTEP`, to use cruising readings only) and the DIAG pin, repeats with `SGTHRS` 255 as a DIAG wiring check, and suggests a `driver_SGTHRS`. Tested against a simplified TMC2209 model in `fake_mcu.py` (`--no-diag` for a missing jumper); not yet on hardware.
- HELP text no longer says moves and homing are missing.
- Setting a temperature (M104, M109, SET_HEATER_TEMPERATURE, the Printer page) while a heater test or PID tune of that heater runs: the reply now says the test was stopped, and the test row says "stopped: you set extruder to 230 C". Before, the heater did go to the new temperature, but the messages ("the target was changed by another command"; "heater off" for a stopped PID tune, while it was heating) read as if the new setting had been overridden.
- Paper: [A Frankenstein Experience](docs/paper/A_FRANKENSTEIN_EXPERIENCE.md) (and PDF, `docs/paper/build_pdf.sh`): how cheap and lean a Klipper brain can get, what works on hardware and what is only simulated, and every embarrassing mistake. Text CC BY-SA 4.0.
- README "what works" brought up to date.
- Status updated everywhere (README, project page, TASKS, firmware notes, build info, paper): **mostly working, needs tuning and proper testing**. Hotend fan confirmed on hardware; the part-cooling fan and third fan output not yet (possibly a broken wire, will update when tested). The project page no longer says nothing moves or heats.
- Paper v1.1: the Elegoo "potato chip" (R528: host on the ARM cores, Klipper's board firmware on the HiFi4 audio DSP, per OpenCentauri), credits (Lost in Tech, Klipper, FluidNC, Anycubic Klipper-go, Elegoo, OpenCentauri, BTT, gistogisto), the nautical naming and the Steamboat Willie public-domain look, cover art, Flasher and phone screenshots, an R528 block diagram, status and disclaimers moved to small print at the end. New `docs/paper/build_pdf.py` (pandoc + Chromium, theme fonts Chewy and Special Elite, Apache-2.0, in `docs/paper/fonts/`).
- Fixed: homing, moves and jogging are refused while a motor or stallGuard test drives a stepper (found while making the paper's screenshots: starting both shut the simulated board down with "Can't reset time when stepper active").
- Test bench "Calibration and stallGuard" section: `rotation_distance` per motor with steps/mm, a "moved / measured -> work out" helper (Klipper's rule: new = old x measured / moved) and Save (`SET_ROTATION_DISTANCE`); `driver_SGTHRS` per sensorless axis with Set (live, written to the driver at once) and Save (`SET_STALLGUARD ... SAVE=1`); a DIAG box (also next to each StallGuard test row) that goes red while the driver reports a stall, read four times a second and held red 3 s so short triggers show.
- Fixed: the ESP32 copied the printer status into a fixed 6 KB buffer and cut it off when longer; with the Tina 2's 16 test rows and long results this could break the Test bench and Printer pages. It now sends the whole status.
- Flasher header and Master status text brought up to date (they still said nothing moves or heats).
- Fans: the Tina 2 preset now has all three SKR Pico fan outputs (gpio17 `[fan]`, gpio18 `[heater_fan hotend_fan]`, gpio20 `[heater_fan fan3]`); the fan test and M106 replies name the pin, and the fan test says to watch the board's fan LED to find which port a fan is on. The commands sent for M106/M107/50 % were checked against Klipper's `src/gpiocmds.c` and are correct; the 2026-10-14 "fans did not run" report is not explained yet.
- `fake_mcu.py`: power off / on (SIGUSR1), a model of the half-powered hang, `--ignore-reset`, Klipper's resync on garbage. 9 new tests (`SessionAgainstFake`); with the listen-first check removed, the simulated board hangs exactly as the real one did.

### Added: moves, homing, printing a file, Tina 2 preset (simulated only, 2026-10-13)
- `ph::Toolhead`: G-code moves through the look-ahead planner, step times compressed into Klipper's `queue_step` (within 25 us; 3000 accelerating steps -> 30 commands), homing and probing with `endstop_home` + trsync, TMC2209 sensorless homing (`tmc2209_<stepper>:virtual_endstop`, `diag_pin`, `driver_SGTHRS`), axis limits, `min_extrude_temp`, `JOG`, `SET_KINEMATIC_POSITION`, `QUERY_ENDSTOPS`, board move-queue flow control.
- Printing: upload on the new Printer page (`/printer`; compressed in the browser, stored in a new 576 KB `gcode` flash partition, unpacked with the ROM inflater), `SDCARD_PRINT_FILE`, `PAUSE`, `RESUME`, `CANCEL_PRINT`, M109/M190 waits; arcs and firmware retraction stop a print with a message; Klipper's 10-minute idle timeout.
- `printer.cfg` preset and install guide for a WEEDO Tina 2 with an SKR Pico (docs/TINA2_INSTALL.md), with a reduced build volume for a larger printhead.
- Heat test passes only within ±5 C of the target. Fixed: commands scheduled after a blocking step (TMC set-up) could reach the board too late; a homing verdict could be overwritten by the host's own trsync reply (both found by the simulator).
- New flash layout (app 1.375 MB + G-code space); settings and printer.cfg are kept.

### Added: test bench, heaters, fans, motors (simulated only, 2026-10-13)
- Heater output check (MOSFET): blinks the output for the LED and checks that heating stops after; stuck-output detector (heating with the output off: emergency stop and "switch off the 12-24 V supply"); on errors the temperature boxes stop, go black, keep the last reading on the top line and show ERROR below. Not yet tried on hardware.
- After the first hardware test (motors moved; hotend overshot, then "ADC out of range", and the board stopped answering until power-cycled): `PID_CALIBRATE` (Klipper's algorithm) with `SAVE_CONFIG` into printer.cfg and a Save button; the heat test reports overshoot; heater log `/heater_log.csv` (temperature, target, power, raw ADC, board-clock error; kept across a board restart); the shutdown message shows the last readings and the allowed range; the ESP32-S3 resets a board that stops answering (once per sequence number) and on Restart; a bigger motor shake (1/8 turn) replaces the 1 mm buzz on the page. Not yet tried on hardware.
- Test bench page `/test`: one row per subsystem with Run and Abort, STOP EVERYTHING (`M112`), Restart board, live temperatures, driver and fan status, console. Run all: TMC check, LED, fans, every motor, hotend to 75 C alongside. See docs/TEST_BENCH.md.
- Heaters `[extruder]`, `[heater_bed]`, `[verify_heater]`: Klipper's PID / watermark control, 5 s `max_duration` on heater pins, the board's ADC range check from min/max_temp, `verify_heater`, host max_temp check, emergency stop; heat-and-hold test.
- Fans `[fan]`, `[heater_fan]` with kick start; `M106`/`M107`; heater fans on above 50 C.
- Steppers `[stepper_*]` and the extruder's stepper (motor tests only): CW / CCW / Klipper's buzz, step counts checked with the board's step counter, Abort via trsync. `STEPPER_TEST`, `STEPPER_BUZZ`, `M18`/`M84`.
- TMC2209 over the TMC UART with Klipper's register values (checked against Klipper's own code), IFCNT write checks, status flags; `TMC_CHECK`, `DUMP_TMC`.
- Console: `M104`/`M109`/`M140`/`M190`, `SET_HEATER_TEMPERATURE`, `TURN_OFF_HEATERS`, `M112`, `FIRMWARE_RESTART`, the test commands.
- Temperature boxes on the status page and test bench: scroll up while heating, down while cooling, colour by state, state in words, no motion for "reduce motion".
- New default `printer.cfg` for the SKR Pico (an unedited earlier default is replaced automatically); "Load the default" link on the `/config` page.
- `fake_mcu.py` simulates a printer (heater warming a thermistor, steppers, trsync, TMC2209s, board-side safety checks); `klink_probe --machine`; research notes in docs/research/klipper-sources.md.

### Fixed
- `/config` page was cut off before the editor (an empty chunk ended the response). `printer.cfg` is now limited to 4 KB so it can always be stored; `/printer.cfg` downloads it.

### Added (simulated or compile-only)
- The Flasher now shows a reminder after a flash: remove the BOOT jumper / release BOOT, then unplug and replug USB (Slave tab after a UF2 flash, Master tab after an ESP32 flash). Shown only when a flash was actually started, never triggers anything
- Temperature sensors (read only): `[temperature_sensor]` and `[thermistor]` in `printer.cfg`, `ph::Thermistor` with Klipper's thermistor maths and four of its named sensors, ADC reports from the board (`config_analog_in` / `query_analog_in` / `analog_in_state`), temperatures on the status page, in `status.json` and via `M105`; `KLink::poll` and a message handler for messages the board sends on its own. The default `printer.cfg` now has hotend (gpio27) and bed (gpio26) sensors for the SKR Pico; a stored `printer.cfg` that was never edited is upgraded to it. Tested against the simulated board; not yet confirmed on hardware.
- Console errors now name the offending word and show an example (`SET_LED LED =GREEN` -> "'LED' is not NAME=value ... Example: SET_LED LED=board_led GREEN=1")
- `printer.cfg` on the ESP32-S3 (edited at `/config`, stored on the chip): `[neopixel]` sections configure the printer board (`ph::build_board_plan`), errors shown with line numbers before anything is sent; replaces the board-LED pin/order settings
- G-code console on the status page (`/gcode`): `SET_LED` and `HELP` so far
- Clock sync (`ph::ClockSync`, after Klipper's `clocksync.py`): measured board clock, drift and round trip on the status page and in `status.json`; `klink_probe --clock`, `--config`, `--gcode`
- Flasher Master tab picks the ESP32-S3's port by itself (Espressif USB ID on Linux by-id names and in the Windows registry, else the only serial port that is not a Klipper board), rescans every 2 s while empty, and **Detect chip** fills an empty port box instead of asking
- Portable C++17 core: Klipper wire format, data dictionary, `printer.cfg` parser, G-code parser, step accumulator, pseudo pressure advance; compiles under ESP-IDF too
- Segment-queue core (`firmware/skipper/segq_core.*`), shared by the firmware and the PC tests
- Motion planner and G-code engine (lookahead, junction deviation, pressure advance) tested through the real core
- Klipper v0.13.0 firmware glue and overlay installer; board presets with prebuilt firmware for SKR Pico (USB and UART), generic RP2040, Creality 4.2.x, RAMPS mega2560, Ender-3 1.1.x; pin tables from each build's dictionary; a test fails when shipped firmware is stale
- Flasher (`flasher.py`, `Flasher.bat`, `Flasher.command`): Slave tab (UF2 drive, STM32 SD card, avrdude; shipped or own build) and Master tab (ESP32-S3 only); SHA-256 check before every flash, refusals for wrong method, flashing during a build, incomplete copies; Windows and macOS drive and port handling (mock-tested)
- `skipper_fw.py` (check tools, fetch Klipper, build, bundle, flash, pins) and `skipper_esp.py` (install, ports, detect, monitor, flash)
- `mcu_probe`, `klink_probe`, `fake_mcu.py`, `printhost.py` one-command Linux setup, Tools window
- 1928 black-and-white cartoon look (visuals only), full title artwork in the GUI header, window icon
- Docs: design, protocol, engine, firmware, wiring guide, hardware log, tasks

### Fixed (found on real hardware)
- ESP32-S3 crashed right after the first successful identify: unpacking the dictionary put the ROM inflater's ~11 KB state on the 12 KB link-task stack. The state is now on the heap, the stack is 16 KB, and a stack-end watchpoint catches any future overflow
- ESP32-S3 restarted when a page was opened: page handlers overflowed the 4 KB web-server stack (now static buffers, 8 KB stack)
- ESP32-S3-Zero status LED is on GPIO21, not GPIO48
- "Install esptool" failed on Linux Mint (externally managed Python)

### Fixed (found in review or tests)
- Segment-queue core: no 64-bit maths in the AVR timer interrupt, no int32 position overflow; `config_segq` rejects periods of 2^31 or more
- GUI clipping (status bar, scrollbar, "Log" checkbox); a test that failed inside a virtualenv

### Removed
- The duplicate Firmware tab in the Tools window (the Flasher's Slave tab is the same panel), the old Firmware-tab screenshot, the separate USB-link note (merged into the wiring guide), unused helper functions
