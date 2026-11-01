# Changelog

Format follows [Keep a Changelog](https://keepachangelog.com/); versions follow [Semantic Versioning](https://semver.org/). Everything before 1.0 is experimental.

## [0.1.0-wip] - 2026-10-11

First public snapshot. **Proof of concept; many features may be broken.** Real-hardware results are in [docs/HARDWARE_LOG.md](docs/HARDWARE_LOG.md).

### Added (2026-10-31): delta kinematics
- `kinematics: delta` (Klipper `delta.py` maths): `stepper_a/b/c`, `delta_radius`, `print_radius`, `arm_length`, `minimum_z_position`; the three towers home together, move limits, edge speed cut, probing on all three towers. The shaper and planner are untouched: tower steps come from the inverse kinematics at the end of the pipeline. See [docs/DELTA.md](docs/DELTA.md). PC-tested (path error 0.04 mm); not run on a real delta; firmware not compiled with ESP-IDF here.
- Separate DECKHAND builds (flash): default DECKHAND = cartesian + CoreXY; `-DSF_KIN=delta` = cartesian + delta. CAPTAIN / SKIPPER have all. Build switches `PH_KIN_COREXY` / `PH_KIN_DELTA`. Flash size of the delta image not measured yet.
- Not yet: delta calibration, round bed mesh, resonance test on a delta.

### Added (2026-10-31): Pasta Inspector dongle (ESP32-CAM) can end failed prints
- Moonraker-style `/websocket` on the ESP32-S3 (CAPTAIN+) so the dongle can `EXCLUDE_OBJECT`, `PAUSE`, `M112` and `RESPOND`, and read print state; `EXCLUDE_OBJECT_DEFINE` now keeps polygons and heights. See [docs/PASTA_DONGLE.md](docs/PASTA_DONGLE.md). Engine PC-tested; **firmware not compiled with ESP-IDF, not tried with a real dongle.**

### Added (2026-10-30): stripboard wiring diagram
- [docs/WIRING_STRIPBOARD.md](docs/WIRING_STRIPBOARD.md): module pinout with wire colours, a stripboard layout (cuts, sockets, buses, resistors) and the four wires to the SKR Pico's UART header. Drawn only, not built yet.

### Works on real hardware
- Clock sync, `printer.cfg` and the `SET_LED` console ran on an ESP32-S3-Zero type board with a bare SKR Pico for about 41 minutes: board clock measured at 12,000,464 Hz (+38.7 ppm), 2,430 samples, 0 CRC errors
- First real Klipper behaviour: the ESP32-S3 configures the printer board like Klipper's host (`allocate_oids`, `config_neopixel`, `finalize_config` with a zlib CRC-32, reset-and-reconfigure on a mismatch) and drives the SKR Pico's on-board RGB LED from buttons on the status page (`/board_led`). New `ph::KBoard`, binary `hex:` arguments in the encoder, `klink_probe --neopixel`, NeoPixel and config-CRC support in `fake_mcu.py`; tested end to end on the PC.
- ESP32-S3 firmware (`firmware/esp32s3`, ESP-IDF v5.4.1): WiFi access point with captive portal, status page, live web console, settings page stored on the chip, status LED (plain or WS2812, colour order setting), UART diagnostics (byte counters, last bytes, echo detection, loopback self-test)
- Read-only Klipper link over UART: `ph::KLink` (identify, data dictionary unpacked with the ROM inflater, `get_config` alive check) between the ESP32-S3 and a BTT SKR Pico running the `skr-pico-uart` firmware
- Flasher Master tab: esptool in a private virtualenv (works where Linux Mint refuses `pip install --user`), Detect chip (`flash-id`), USB log, shipped image with SHA-256 check

### Works on real hardware (cont.)
- Temperature reading from two thermistors on the SKR Pico (hotend ~24 C, bed ~21 C).

### Changed (2026-10-29, night): documentation brought up to date
- **README**: status, the parts table, Known limitations and the Roadmap rewritten for what has now run on hardware (full homing, a print file run in mid-air, WiFi updates, OLED, touch buttons, both benchmarks). The old limitations ("only the link and the LED have run on hardware", a placeholder config CRC, "ESP-IDF build untested") were long out of date.
- **Project page** (`docs/index.html`): the same.
- **The paper, A Frankenstein Experience, version 1.2**: plain Klipper on the board, the PSRAM found, homing and the mid-air print, six new mistakes (18-23), a new section 6.1 with the benchmark measurements, and an updated status. PDF rebuilt.
- **Smaller files**: the paper's PDF went from 8.3 MB to under 1 MB. The film strip's sprocket holes are now drawn as vector shapes (Chromium had turned the repeating background into a full-page picture on every page), the chip diagram is the SVG instead of a 713 KB PNG (removed), and the photos are smaller JPEGs. The screenshots and photos in docs/images were recompressed too (about 375 KB less).

### Fixed (2026-10-29): benchmark result cut off; build time (the benchmark itself now confirmed on hardware)
- **The benchmark ran to the end on the Tina 2** with the 2026-10-28 evening INT32/64 test build, twice (once flashed over WiFi, once over USB: the same binary, e171ff5c): 121.7 s and 121.8 s, printer task busy 43.8 % and 44.9 % of one core. The 1024-command move queue fixed the "too late" refusals.
- **The result line was cut off** after "without sending):", so the per-stage figures and the sends were missing: the important-events log kept lines of up to 480 characters. The result is now five short lines (`benchmark ...`, `benchmark firmware:`, `benchmark work:`, `benchmark processor:`, `benchmark sends:`), and the log keeps lines of up to 1 KB. A test checks that every line fits.
- **The motion benchmark (Tune page) now uses the integer shaper in the INT32/64 test build.** Until now it used the `double` shaper in every build, which is why both builds gave the same "input shaping + pressure advance" figures on the Tina 2 (22.1 us per step, 19 % of one core at 100 mm/s). Its first line says which maths ran, and its lines now also go to the important-events log ("motion benchmark: ..."). The test build no longer carries the `double` shaper at all: 14 KB free instead of 7 KB.
- **Important events laid out in columns** (Log page): time (date under it), time since start-up, then the event, its first clause as a heading in colour (red for shutdowns, losses and refusals, green for start-ups and connects, yellow for the benchmark) and each further clause on its own line; the benchmark's figures in a grid. Lines with the same time are grouped. On a phone the event goes under the time. Copy and download still give the plain text.
- **The build time in the self-identification was wrong** ("built Oct 28 00:44" on a build from the evening): it was the time one file was last compiled. It now comes from a file compiled on every build.

### Fixed (2026-10-28, evening): the board's move queue was taken as 200 instead of 1024 (built and PC-tested; not yet re-run on hardware)
- **The real cause of the "too late" refusals on the Tina 2** (int32/64 test build over USB and over WiFi, four more runs, the board on USB power for two of them; every message said "board move queue … of 200"). The ESP32 read the board's move-queue size before configuring it, when Klipper reports 0, and then fell back to 200 commands. With only 200 commands allowed ahead, the run of tiny moves could not be queued far enough in advance, and a move was due before there was room to send it. It is now read again after configuring (the SKR Pico reports 1024), and the USB log says "board move queue: 1024 commands". PC reproduction with the ESP32's speed simulated (300 times slower): fails with 200, finishes with 1024.
- The board's queue is now counted per command (each command frees its room when its own last step is done).
- The benchmark's "longest gap between sends" no longer counts the time before the run started (one result showed 8,359 ms).
- The benchmark's "why:" gives the message of the move that was refused, not only "stopped".
- The simulated board (`fake_mcu.py`) reports its move queue only once configured, like Klipper; a test checks that the ESP32 then uses 1024.

### Fixed (2026-10-28, afternoon): moves refused as "too late" during dense planning; Flasher offsets; naming (built and PC-tested; not yet re-run on hardware)
- **Moves refused as too late in the benchmark** (Tina 2, int32/64 test build, twice, both in the file's run of 300 tiny moves). Not the integer maths: the same happens with `double` once planning is slow enough. A burst of planning (the look-ahead re-planning many tiny moves, or running dry and planning everything at once) outlasted the 0.1-0.2 s its first steps had been given, because nothing was sent until the loop pass ended. Now: what is due is **sent every 20 ms during planning** as well, the print feeder **stops after about 25 ms** of work per pass, and the board's move queue is counted per motor (a motor's commands free their room when they finish, not when the last step of the whole send does). PC reproduction (the ESP32's speed simulated by slowing the PC's work): failed at 800 times slower before, finishes at 1,500 times (both maths) now. New test: a slow host must finish the benchmark (it fails without the fix).
- **The "too late" message says why**: how long nothing had been sent, how many planner segments were made meanwhile and whether inside one planning burst, the last send's length, the board's move queue against its limit, the longest gap between sends so far.
- **Benchmark result** adds: sends (and how many during planning), the longest gap between sends, the longest send, the most commands in the board's queue and how often it was full, why it stopped, and which firmware exactly ran.
- **Which firmware is running**: the important-events log's start-up line and the top of `/report.txt` now say "STANDARD build (double motion maths)" or "INT32/64 TEST BUILD (whole-number motion maths)", with the build date and time and a short fingerprint of the binary.
- **Test build files renamed**: `firmware/prebuilt-esp32s3/test-int32/` with `CAPTAIN-INT32-TEST-full-image-usb.bin` (USB, 0x0), `CAPTAIN-INT32-TEST-wifi-update.bin` (Update page) and `CAPTAIN-STANDARD-wifi-update.bin` (to go back over WiFi).
- **Flasher: an app-only image flashed over USB as "own file" went to 0x10000 and the ESP32 did not boot** (CAPTAIN's app slot is 0x20000). The Flasher now reads the image: a full image must go at 0x0, a Skipper Flipper app at 0x20000 (CAPTAIN, SKIPPER) or 0x10000 (DECKHAND); picking a file sets the offset, and a wrong one is refused.
- **sf-bench.gcode is refused as a normal print** (START PRINT, SDCARD_PRINT_FILE): it moves relative to wherever the head is and never homes; run it with SF_BENCH.
- DECKHAND: the table of ESP-IDF error names is left out (errors show as a number, e.g. 0x107) to keep room: 8 KB free; CAPTAIN 16 KB, the INT32/64 test build 10 KB.
- Tests: 1,821 C++ checks on each maths.

### Added (2026-10-28): print benchmark, integer motion test build, important-events log (built, PC-tested, QEMU-run; not yet on hardware)
- **`SF_BENCH`** (Tune page: Print benchmark): a 12-layer print-like file (`docs/bench/sf-bench.gcode`, written to the SD card by the first run; `FILE=builtin` needs no card) through the whole motion pipeline with **the stepper drivers and heaters off**, timed stage by stage with the CPU cycle counter: planner, shaping, step times, compression, sending, everything else. With the SKR Pico connected its step queues and the UART are used for real; without a board the ESP32 runs it alone (commands encoded for the SKR Pico, then dropped). Refused while a heater is on, or on a real board if a motor has no `enable_pin`. Guide: [docs/BENCHMARK.md](docs/BENCHMARK.md). CAPTAIN and SKIPPER.
- **int32/int64 test build for CAPTAIN** (`firmware/prebuilt-esp32s3/test/`): input shaping, pressure advance and step times in whole numbers (`MotionShaperFx`); the ESP32-S3 does `double` in software. The whole C++ test suite now also runs on the integer path (`ph_tests_fixed`); the integer shaper matches the `double` one to within one step at every grid point.
- **Important-events log**: a second tab on the Log page (and `/report.txt`, `skipper/report.log` on the SD card) with only start-ups and their reason, crashes, board connects and losses, shutdowns, refused late moves, print ends and benchmark results.
- Fixed: the status LED's colour bytes were on the stack while the LED driver could still be reading them (only after a timeout, which a real WS2812 never hits).
- QEMU test builds: single core, no ADC driver, no WiFi (QEMU 9.0 reports the heap at 0x3FCE9710 as damaged from core 1 even with the earlier firmware; real boards are not affected); see docs/BENCHMARK.md.
- Sizes: DECKHAND 2 KB free (the benchmark is CAPTAIN/SKIPPER only), CAPTAIN 19 KB, the int32/int64 test build 12 KB, SKIPPER 46 %.
- Tests: 1,814 C++ checks, run twice (double and integer maths).

### Changed (2026-10-27): web pages (built; not yet on hardware)
- **The Printer page is now the first page** (`/`; `/printer` still works). The status page moved to `/status` (second in the side bar).
- **A big Print box at the top** of the Printer page: pick a stored file and press START PRINT (a large green button), or upload a new one with "print it when uploaded" ticked. While printing, the box shows the progress bar, layer and Pause / Resume / Cancel.
- Small drawn icons (inline SVG, about 1.5 KB in all, no image files): a house on the Home buttons, a red thermometer on Preheat and Heat, a snowflake on Cool down, a fan, play / pause / cancel.
- New Preheat (hotend and, if there is one, the bed to the temperatures in the boxes) and Cool down (heaters and part fan off) buttons.
- Jog keys: bigger, thick two-tone borders, X red, Y blue, Z black and yellow, and a yellow "home all" key in the middle.
- Sizes after this: DECKHAND 6 KB free (0 %), CAPTAIN 46 KB (3 %), SKIPPER 47 %.

### Added (2026-10-27): QIDI X-Smart 3 test bench, CoreXY, chamber sensor (built and PC-tested; not yet on hardware)
- **[docs/configs/qidi-xsmart3-skr-pico.cfg](docs/configs/qidi-xsmart3-skr-pico.cfg)** with a wiring and first-start guide ([.md](docs/configs/qidi-xsmart3-skr-pico.md)): the X-Smart 3 rebuilt around an SKR Pico (its toolhead board is dead). Mechanics, currents, offsets and PIDs from QIDI's own config; the thermocouple must be replaced by a thermistor; 300 mm/s and 5000 mm/s² to start (QIDI: 600 / 20000). Simulated `G28` ends at X61.5 Y89 Z10.
- **`kinematics: corexy`** (all builds): motor A = `stepper_x` (X+Y), motor B = `stepper_y` (X−Y), as Klipper. Homing an axis drives both motors and stops both on that axis' endstop; the resonance test drives both; free run on X/Y is refused (it would move diagonally); unequal steps/mm on A and B is refused.
- **`endstop_pin: probe:z_virtual_endstop`**: Z homes on the `[probe]`; Z's position at the trigger is the probe's `z_offset` (no `position_endstop`).
- **`gear_ratio`** on steppers (`9.5:1`, `50:17, 80:20`). **Fixed:** it was ignored before, which would have extruded 9.5 times too much on a geared extruder.
- **A thermistor on the ESP32 itself** (`sensor_pin: esp32:gpio7`, e.g. a chamber sensor; GPIO1-10, 100 kohm pull-up by default; never shuts anything down): [docs/CHAMBER_SENSOR.md](docs/CHAMBER_SENSOR.md).
- **Faster ESP32:** 240 MHz (the ESP-IDF default was 160), 32 KB instruction and 64 KB data cache on CAPTAIN and SKIPPER, the motion maths built with -O2. Flash mode left at DIO (QIO can stop some boards booting).
- Research note: [docs/research/klipper-grade-performance.md](docs/research/klipper-grade-performance.md) (how far this can go, the Bambu A1, and the ESP32-S31 as a possible "NEPTUNE" tier).
- Tests: 1,763 C++ checks.

### Fixed (2026-10-27): "Timer too close" during homing on the Tina 2 (fix confirmed on the printer the same evening)
- **The printer link task could be held off by the web server.** It ran at priority 4 on either core; the web server runs at 5 and the TCP/IP task at 18. While the Printer page loaded a G-code preview (a 512 KB read and scan) and WiFi was busy sending, the link task could not run for long enough that steps already planned reached the board after their time. It now has core 1 to itself (priority 10, above the web server); WiFi and the TCP/IP task stay on core 0.
- Moves are timed from the real current time (it was the time the command or tick started, which can be stale after slow work), with at least 100 ms in hand (was 50).
- **Last check before sending steps:** if a stepper's next step is already less than 5 ms away, nothing is sent, motion stops and the axes count as not homed, with a message ("a move was ready N ms too late and was not sent"). The board no longer shuts down for this.
- New tests: moves with a stale "now" and slow board queries (failed before the fix: 4 of 7 late), a frozen ESP32 between planning and sending (refused, not sent), and G28 right after a false-triggered Z hard stop.
- Not the cause: the bigger printer.cfg store (it only writes flash when printer.cfg is saved, and once at the first start).

### Changed (2026-10-27): bigger printer.cfg (built; not yet on hardware)
- **printer.cfg up to 16 KB on CAPTAIN and SKIPPER, 8 KB on DECKHAND** (was 4 KB everywhere). It moved from NVS (24 KB in all, which cannot hold two big copies) to the end of the "gcode" flash partition, in two copies written alternately with a checksum, so a power cut while saving keeps the previous one. No partition-table change, so it works after a WiFi update; the old copy in NVS is moved over at the first start and then deleted. The flash G-code store shrinks by 24 KB (DECKHAND) / 40 KB (CAPTAIN, SKIPPER); an uploaded file that reached into that space is removed once (upload it again).
- **Fixed: a printer.cfg just under 4 KB was refused as too big** when pasted into the page: the browser sends Windows line ends (CR LF), so every line counted one byte extra (the 3,968-byte Tina 2 config arrived as about 4,140). Carriage returns are now dropped before the size check and are not stored.

### Changed (2026-10-27): Tina 2 config (not yet tried on the printer)
- [docs/configs/tina2-skr-pico.cfg](docs/configs/tina2-skr-pico.cfg): **heated bed** added (the author's Tina 2 has one, about 120 x 120 mm): SKR Pico HB output gpio21, THB thermistor gpio26, assumed Generic 3950 thermistor, watermark control until "PID tune bed", max_temp 100.
- **Homing order**: X, X back to the middle (45), Y, Y back to the middle (52), then Z down onto the middle of the bed, then up 10 mm. Before, Z could come down while the bed was off to one side and never touch it. Simulated: ends at X45 Y52 Z10.

### Added (2026-10-26): buttons on the case, CAPTAIN and SKIPPER (built and PC-tested; not yet on hardware)
- **Up to four buttons**, as in KlipperTFT-Pico: capacitive touch pads (a bolt through the case on a wire; ESP32-S3 GPIO1-14) or push switches to GND. Defaults GPIO2 Preheat, GPIO4 Home, GPIO5 Unload, GPIO6 E-stop (Home and E-stop only after a 1 s hold); off until switched on. Two buttons at once count as none (shorted wires, a hand across two bolts). Each can also be Load filament, Cool down, Pause/resume, a custom command or nothing. Guide: [docs/BUTTONS.md](docs/BUTTONS.md).
- New console commands (also usable without buttons): `SF_PREHEAT`, `SF_COOLDOWN`, `SF_LOAD_FILAMENT`, `SF_UNLOAD_FILAMENT` (heat first and wait; a `[gcode_macro LOAD_FILAMENT]` / `UNLOAD_FILAMENT` runs instead if present), `SF_PAUSE_RESUME`. Refused while printing, except pause/resume.
- `/buttons` page (link on Settings): pins, actions, values, live touch readings with the threshold, and a Practice mode where presses show on the page but do nothing. The OLED flashes what was pressed (PREHEAT, NOT NOW, HOLD 1 S...). The accelerometer search leaves the button pins alone; the Settings page refuses pins the buttons use.
- Touch logic (learnt baseline, drift following, hysteresis, stuck-pad recovery) and press timing (debounce, 1 s hold, start-up lockout) in `ph::TouchFilter` / `ph::PressTimer`, unit-tested; the SF_ commands tested against the simulated board (heating, LOADING on the OLED, exact extruder steps, refusals).
- Sizes: CAPTAIN 5 % free (was 7 %), SKIPPER 49 %; DECKHAND unchanged (no room).
- Tests: 1,681 C++ checks.

### Hardware (2026-10-26)
- **Touch buttons work through the case**: Preheat and Home fired from plastic-welder staples melted halfway into the Tina 2's case, sensitivity 10 % ([docs/BUTTONS.md](docs/BUTTONS.md)). The other two pads were not wired yet.
- **The OLED works on a real display** in the Tina 2 ("Dirty Tina", the Klipper test bench): temperatures and the board-shutdown screen ([docs/HARDWARE_LOG.md](docs/HARDWARE_LOG.md)).

### Removed (2026-10-25): the segment queue; the printer board now runs plain Klipper
- **The ESP32-S3 firmware is now the only bespoke firmware.** The printer board gets Klipper v0.13.0 exactly as released, with only Klipper's own menuconfig choices (chip, flash chip, USB or UART). The experimental segment queue (`segq`, three files and a Kconfig option added to Klipper since 2026-10-03) is gone: the host never used it, it had no homing, and Klipper's own `queue_step` is cheap enough. Why and what it was: [docs/research/segq-retired.md](docs/research/segq-retired.md) (its protocol is archived there).
- `skipper_fw.py`: builds plain Klipper; cleans the old overlay out of an existing Klipper folder (Kconfig and Makefile restored from git) and refuses to build if Klipper's sources have any local change. New `clean` command; `overlay` removed; `--no-segq` accepted and ignored. The Flasher's "Segment queue" box is gone.
- All six shipped printer-board firmwares rebuilt (they report `v0.13.0-0-g61c0c8d`, no "-dirty"); `build-info.json` says `plain_klipper: true`; presets lost `dda_hz` and `build_sizes`.
- Deleted `firmware/skipper/` and `firmware/test/`; `segq_core.[ch]` kept in `host/test/` as a test-only step replay for the planner tests (no longer compiled into the ESP32 firmware). The simulated board (`fake_mcu.py`) speaks plain Klipper only.
- Tests: 1,489 C++ checks and 190 Python tests pass (198 before: the segment-queue tests went, plain-Klipper checks came in).
- Docs: FIRMWARE.md rewritten, PROTOCOL.md section 2 now "Klipper's own commands", README, TASKS, THIRD_PARTY, ENGINE, HOW_IT_WORKS.
- **The SKR Pico in the Tina 2** still runs the 2026-10-10 build with the idle segment queue; that is harmless. Reflash `skr-pico-uart` when convenient (Flasher, Slave tab); the plain build has not been flashed yet.

### Fixed (2026-10-24): review of the whole host before the first motion test
- Free run (stallGuard test) now runs away from the homing end, counts its ramps in the distance and is refused when the axis has no room.
- A false trigger or failed hard stop now unhomes the axis (it used to stay "homed" in the wrong place); homing can be aborted cleanly.
- CANCEL_PRINT and a macro abort stop a running macro; a failed macro unhomes axes it had only assumed with `set_position`.
- SAVE_SENSORLESS is refused while the printer is busy; G28 inside a `[gcode_macro]` while a `[homing_override]` exists gives an error instead of recursing.
- Classic G-code in a print needs a digit after the letter (a comment word like "Generated" no longer reads as G-code).
- Hard-stop failures other than a stall now say what happened.
- `[homing_override] set_position_x/y/z` supported (as Klipper).
- printer.cfg editor: replacing a value also removes its old continuation lines.
- Flaky blind-reset Python test (simulated board) fixed.

### Changed (2026-10-24)
- **Tina 2 config:** `[homing_override]` now starts with `set_position_z: 0` and lifts Z 3 mm before homing X and Y, and homes Z at X45 Y52 (the middle of the bed). Not yet tried on the printer.
- **Ender 3 preset:** extruder `rotation_distance` 33.683 (Klipper's value for the stock extruder) and `max_extrude_only_distance` 100.
- **Step compressor 2.2x faster** (fixed-point step times, early exit): the Z homing move that made "Home all" late on the real chip goes from 2.76 to 1.27 us per step in QEMU. Same commands out (tested).

### Added (2026-10-24): Klipper tuning features, CAPTAIN and SKIPPER only (built, PC-tested and run in QEMU; **nothing here has run on a printer**)
- **Input shaping** (`[input_shaper]`, `SET_INPUT_SHAPER`, all six Klipper shapers) and **pressure advance** (`pressure_advance`, `pressure_advance_smooth_time`, `SET_PRESSURE_ADVANCE`), worked out on a 2 ms time grid between the planner and the step queues instead of per step. Simulated frame ringing drops 5-20x. Measured in QEMU at 150 mm/s: 6 % of one core plain, 14 % with both; estimated 35-40 % on the real chip. Details: [docs/research/pressure-advance-input-shaping.md](docs/research/pressure-advance-input-shaping.md).
- **Automatic input shaper calibration** (new **Tune** page, [docs/TUNING.md](docs/TUNING.md)): **Find accelerometer** searches the OLED's wires and every spare pin pair for an ADXL345 (I2C or SPI) or MPU-6050, optionally the USB-C port for a Klipper USB accelerometer (BTT ADXL345 V2, KUSBA, FLY-ADXL345; tries one SPI pin group at a time, then restarts it with only the one found; works with Klipper's old and new SPI bus names; asks for Klipper v0.12+ on it; **compile-only, never run**); then pick X or Y, Klipper's resonance test (5-135 Hz), the spectrum and Klipper's shaper choice, and **Save** into printer.cfg. `TEST_RESONANCES`, `SHAPER_CALIBRATE`. The fit matches Klipper's `shaper_calibrate.py` exactly on the same synthetic data.
- **Bed mesh** (`[bed_mesh]`, `BED_MESH_CALIBRATE`, `_PROFILE`, `_CLEAR`, `_OUTPUT`): coloured text heightmap (high, medium-high, base, medium-low, low), saved as `[bed_mesh default]`.
- **Screw tilt** (`[screws_tilt_adjust]`, `SCREWS_TILT_CALCULATE`): turns per screw in Klipper's CW 00:12 form.
- **Arcs** G2/G3, **firmware retraction** G10/G11 with `SET_RETRACTION`/`GET_RETRACTION`, **EXCLUDE_OBJECT** (with Exclude buttons on the Tune page), **TUNING_TOWER**.
- **Motion benchmark** (`/bench`, Tune page): how much processor the moves take on your board.
- OLED shows SHAKING, MESHING, SCREWS.
- Sizes: DECKHAND 1 % app space free (15.6 KB; it keeps the fixes but none of the new features), CAPTAIN 7 %, SKIPPER 49 %.
- Tests: 1,489 C++ checks (was 1,214), new `test_shaping`, `test_resonance`, `test_pro_features`, `test_review_fixes`, `test_toolhead_shaping`.

### Added (2026-10-23): little OLED display (built and simulated; not yet on a real display)
- **Cheap I2C OLED** (SSD1306 / SSD1315 128x64 and 128x32, SH1106 1.3"), in all three builds, on by default on SDA GPIO8 / SCL GPIO9 ([docs/OLED.md](docs/OLED.md)). Not printing: hotend and bed as big as they fit, with target and heating arrow. Printing: percentage and progress bar (paused alternates with "PAUSED"). HEATING, HOMING, EXTRUDING, RETRACTING, LOADING, UNLOADING flash up 2 s in every 4 s. No board: the reason, the WiFi name, then password and address in turn; printer.cfg problems; board shutdown.
- Drawing code, fonts and I2C driver from KlipperTFT-Pico 0.6.1 (same author), in `firmware/esp32s3/components/ktft_oled/`, GPL-3.0 in this repository (fonts: Bitstream Vera / DejaVu licence). Its layout tests now build with the host tests (`oled_tests`).
- `ph::Machine::display_info()`: what the printer is doing, for the display (unit-tested: homing, extruder-only moves and their duration, LOADING from 20 mm, print state; a print's own retractions do not flash up).
- Settings page: type, SDA, SCL, upside down; pins already used by the UART, LED or SD card are refused. Status page and `status.json` (`oled`) say what was found. The display task looks three times over 10 s after a start, then gives its memory back; if a display stops answering it looks again every 5 s.
- Cost: DECKHAND app space now 2% free (28 KB; was 5%), `.bss` 77.7 KB (was 75 KB). CAPTAIN 16% free, SKIPPER 54%.
- Checked: PC tests; all three builds compile; DECKHAND under QEMU starts the display task and gives up cleanly with no display.

### Housekeeping (2026-10-23)
- Logs brought up to date (this changelog, TASKS, TIERS, README, THIRD_PARTY, research notes).
- Old release zips and build leftovers removed from the work area; the last two release zips kept.

### Changed (2026-10-20, late: after tuning the Tina 2 by hand; none of this has run on hardware yet)
- **Default printer.cfg on the ROM is now a Creality Ender 3 on an SKR Pico** (config page: "Load a default"): Klipper's Ender 3 mechanics (X/Y 40, Z 8, extruder 34.406, 300 mm/s, 3000 mm/s²), SKR Pico pins with switch endstops on X/Y/Z, heated bed. The Tina 2 config moved to [docs/configs/tina2-skr-pico.cfg](docs/configs/tina2-skr-pico.cfg), the user's tuned values plus the new Z homing; [docs/configs/ender3-skr-pico.cfg](docs/configs/ender3-skr-pico.cfg) is the ROM preset as a file (a test keeps the two the same).
- **`[homing_override]` and `[gcode_macro NAME]`** (plain G-code lines; Jinja `{ }` templates are refused with a clear error). The macro waits for each move, homing and M109/M190 before the next line; a failed G28 or PROBE stops it. G28 inside the override does the real homing. Typing the macro's name in the console, or having it in a print, runs it. Unit-tested.
- **Tina 2: Z homes down onto the bed and then lifts 10 mm** (`[homing_override]`: G28 X, G28 Y, G28 Z, G1 Z10), because the top of the frame flexes too much for a stall to register. X `dir_pin` flipped. Z's `driver_SGTHRS` must be found again for this direction.
- **Sensorless homing on the Test bench: three yellow buttons per axis.** **1. Hard stop** (`STALLGUARD_HOME`): the real homing move with the current and sensitivity on screen; passes only when it stops against the end after moving at least 1 mm, then backs off 5 mm (Z 10 mm); "false trigger" when it stops at once, "no stall" when it runs the full length. **2. Free run**: the stallGuard test in the middle of the axis (DIAG must stay quiet). **3. Save** (`SAVE_SENSORLESS`): current and sensitivity into printer.cfg in one write. Replaces Set / Set + test / Save for each value.
- **stallGuard free run no longer turns DIAG red on purpose.** It used to finish with a pass at `SGTHRS 255` to prove the DIAG wire, which showed as "triggered" on the return stroke whatever the setting. That check is now `STALLGUARD_TEST ... WIRING=1`; the hard-stop test proves the wire anyway.
- **Update** button in the left bar (firmware update over WiFi).
- [docs/configs/tina2-run-around-10min.gcode](docs/configs/tina2-run-around-10min.gcode): hand-written (not sliced), heaters off, homes, then runs a 10 mm box in the middle of the Tina 2's bed at 50 and 150 mm/s (capped by its 97 mm/s `max_velocity`) for about 10 minutes. Simulated: 10.5 minutes, stays inside X 40-50, Y 47-57.

### Fixed (2026-10-20, night)
- **"Home all" shut the board down with "Timer too close"** (Tina 2, on hardware). The shutdown report showed the Z homing move's start commands arriving 130 ms after their start time: the whole 49,200-step move was compressed before anything was sent (~0.4 s on the ESP32-S3, against a 0.3 s head start). The homing move is now sent like any other move: the first 0.6 s at once, the rest streamed while it runs; and if the start is still too close when the commands are ready, nothing is sent and it starts again (three tries, then a clear error). Unit-tested: the first batch holds under a second of steps, and with no stall every one of the 49,200 steps still reaches the board, then "No trigger on z".
- Confirmed on hardware: firmware update over WiFi (CAPTAIN).

### Changed (2026-10-20, evening: after the first CAPTAIN run)
- **Left navigation bar on every page** (Status, Printer, Test bench, printer.cfg, Log, Settings, About, and a STOP button), fixed while the page scrolls; narrower on phones. The fixed pages (Printer, Test bench, Log) are cached for 30 s, so switching between them is instant.
- **Test bench: motor current and stallGuard sensitivity** per motor: a current box (`SET_TMC_CURRENT STEPPER= CURRENT= [SAVE=1]`, Klipper's command; unit-tested) and a sensitivity slider for `driver_SGTHRS` with Set, **Set + test** and Save.
- **Homing: the endstop is checked once per step time** (Klipper's `_calc_endstop_rate`), not every 100 us.
- **"Timer too close" shutdowns explain themselves**: the last 48 timed commands are kept, and the shutdown names the ones closest to the moment the board stopped and how far ahead each was sent (status.json `timing`, shown in the shutdown box; unit-tested). New test: "Home all" on the Tina 2 preset with slow TMC replies, 32 and 256 moves of look-ahead, checking every timed command against the board clock when it arrives.
- DECKHAND internal RAM: the Status, Settings and About pages build in heap memory per request (`.bss` 75 KB, was 85 KB before today's extras).

### Added: three builds by board memory, and extras (2026-10-20, compile-only on hardware)
- **DECKHAND / CAPTAIN / SKIPPER** builds from one source ([docs/TIERS.md](docs/TIERS.md)): DECKHAND runs on any ESP32-S3 (as before); CAPTAIN (4 MB flash + PSRAM) adds firmware updates over WiFi with roll-back (`/update`), 256 moves of look-ahead and 64 KB of console history; SKIPPER (8 MB flash + 8 MB octal PSRAM) 512 moves, 256 KB, bigger thumbnails and an RGB salute. The names are a joke; `/about` lists what is on and off and why. PSRAM is "ignore if not found", so a bigger build still starts without it (checked under QEMU).
- **Flasher: Flash for my board...** (Master tab) and `skipper_esp.py flash-auto`: reads the chip (esptool `flash-id`, read-only), picks the build from the detected flash size and the embedded PSRAM, flashes it after one confirmation. Same rule as the firmware's `/about` page.
- In every build: **send from the slicer** (PrusaSlicer / OrcaSlicer as an OctoPrint printer: `GET /api/version`, `POST /api/files/local`, streamed to the SD card, "upload and print"); **join the home WiFi** as well as its own network (Settings); **slicer thumbnails** in the file list and next to the progress bar; **print history** and **event log** on the SD card; **printer.cfg backups** (last 10) on the card; the **clock** taken from the browser; a **memory report** with the lowest free memory since start.
- **Crash reports** in every build: a core dump partition plus the last 2 KB of log kept in RTC memory across a restart; after a crash, brownout or watchdog reset the status page names the panic and the task and links to `/crash` (backtrace, last log lines) and `/coredump.bin`; with a card in it is also saved as `skipper/crash-NNNN.txt`. Shipped `symbols/*.elf` decode the backtrace. Checked under QEMU: a deliberate crash was reported after the restart as "panic in task led".
- Look-ahead size comes from the memory (`ToolheadIO::lookahead`, halved until it fits, never below 32; unit-tested), print-end callback for the history (unit-tested), streaming multipart parser and G-code thumbnail finder (`ph/multipart.h`, unit-tested).
- Flash G-code store (no card) is now 508 KB on DECKHAND and CAPTAIN (was 576 KB; 64 KB went to the core dump), 1.75 MB on SKIPPER.

### Fixed (2026-10-20)
- **Status LED task stack overflow** (found under QEMU): any error the LED driver printed overflowed the LED task's 3 KB stack and crashed the ESP32 ("Stack canary watchpoint triggered (led)"). The stack is now 4 KB. This is the most likely cause of the crash reported on 2026-10-20 (blinking red for 20 minutes, then "last restart: CRASH (panic)"): the red error blink drives the LED ten times a second, so a rare driver time-out is most likely then. Not confirmed: that run's panic message was not captured (the new crash report will catch the next one).
- DECKHAND internal RAM: the new pages take their big buffers from the heap per request instead of static memory (`.bss` 95 KB instead of 136 KB at first; 85 KB before the extras).
- Unknown paths redirect to `/` (relative), so the pages also work over the home WiFi.
- Status page no longer says the ESP32 "cannot move axes, home or print yet".

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
- **Micro SD card support (built and compiled, not yet run on hardware).** SPI card on GPIO10/11/12/13 by default (CS, MOSI, SCK, MISO: SPI2's own pins, edge pins next to TX/RX; first planned as 15-18, but GPIO18 is a small pad under the ESP32-S3-Zero; changeable on the Settings page, with clash checks), FAT32, files in the top folder. Printer page: the card's files with Print / Delete, free space, Remount; uploads go to the card as plain G-code when one is in (written as `.part`, renamed when complete). Commands: `SDCARD_PRINT_FILE FILENAME=`, `M20`, `M23 name` + `M24` (unit-tested). Guide: [docs/SD_CARD.md](docs/SD_CARD.md), including which VCC your module needs.
- Printer page job display (2026-10-19, compile-only on the ESP32; the numbers behind it are unit-tested): a text progress bar, one character per 5 % (`[#######.............]  37%`), time left, and a Z box with the layer number and layer height. Time left starts from the slicer's estimate and moves over to the printer's real pace as the print goes on (fully by 30 %). Layer number, layer count, layer height and the slicer's time come from the slicer's comments (Cura `;LAYER:` `;LAYER_COUNT:` `;TIME:` `;Layer height:`, PrusaSlicer / OrcaSlicer `;LAYER_CHANGE` `;Z:` `; layer_height =` `; total layer number:` `; estimated printing time`); without them the box shows the toolhead's Z only. A "Files" list shows the SD card's files and the file in flash together, each with Print.
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
