> ## WORK IN PROGRESS: MANY FEATURES MAY BE BROKEN
> **`0.1.0-wip`: mostly working, needs tuning and proper testing.** It heats, homes, and has run a whole print file in a real printer (in mid-air: the Z offset was never found), but it has not printed a part yet, and breaking changes are expected. Read [Known limitations](#known-limitations) before you plug anything in.

![Skipper Flipper: an experimental ESP32 pseudo Klipper port. A cartoon dolphin in a captain's hat steers a 3D-printed paddle boat.](docs/images/skipper-flipper.webp)

# Skipper Flipper

*An experimental ESP32 pseudo Klipper port.* (Repository and package name: `esp-printhost`.)

**GPL-3.0** · Linux (Mint / Ubuntu) · C++17 core + Python tools · [Project page](https://The-Dorkknight.github.io/esp-printhost/)

Licensed under the GNU General Public License v3.0, see [LICENSE](LICENSE). **No warranty and no liability:** this is provided as is. You use it at your own risk, on your own hardware, and the author is not responsible for damage to boards, printers or anything else.

## What this is

An idea for reviving old, donated 3D printers: a cheap ESP32-S3 becomes the "brain" (WiFi, web UI, text config, motion planning) and the printer's original board becomes a simple slave running a modified Klipper MCU firmware. The full plan is in [docs/DESIGN.md](docs/DESIGN.md).

**Target hardware: ESP32-S3 only** for the host ("master"). **Status (29 October 2026): mostly working, needs tuning and proper testing.** On real hardware ([docs/HARDWARE_LOG.md](docs/HARDWARE_LOG.md)), installed in a WEEDO Tina 2 with a BTT SKR Pico running plain Klipper v0.13.0:

- the WiFi pages, console and `printer.cfg` editor; firmware updates over WiFi;
- the Klipper link, clock sync, board configuration and recovery from board power cycles;
- temperatures, heating, PID tuning and Klipper's heater safety checks;
- the motor test, TMC2209 set-up, stallGuard and **full homing of X, Y and Z** (2026-10-27);
- **a whole print file, run end to end** (2026-10-27): in mid-air, because the Tina 2's flexible Z gave no reliable stop to set a Z offset from;
- the micro SD card, the little OLED, touch buttons through the case, the hotend fan;
- **the print benchmark** (2026-10-29): 12 layers of a print path (9,617 lines, about a million steps) through the whole motion pipeline and the real board, drivers off, in 121.8 s, with the ESP32's printer task busy about 44 % of one core, of which about 14 % is real work and the rest mostly waiting for the board's acknowledgements ([docs/BENCHMARK.md](docs/BENCHMARK.md));
- **integer motion maths** (INT32/64 test build, 2026-10-29): input shaping with pressure advance costs 10 % of one core at 100 mm/s instead of 19 %; the shaping work itself is about 3.7 times faster.

**Not yet:** a real printed part; input shaping, pressure advance, bed mesh and the other tuning features on a moving printer; the part-cooling fan and the third fan output (possibly a broken wire). **Next test bench:** a QIDI X-Smart 3 (CoreXY) with an SKR Pico; the Tina 2 is retired as the bench (no sharp Z stop, and its tiny Y motor overheats). The whole story, including every mistake: **[A Frankenstein Experience](docs/paper/A_FRANKENSTEIN_EXPERIENCE.md)** ([PDF](docs/paper/A_FRANKENSTEIN_EXPERIENCE.pdf)).

New features by date, and whether each has been on hardware, are in [CHANGELOG.md](CHANGELOG.md).

| Part | State |
|---|---|
| Klipper wire protocol (CRC16, VLQ ints, framing, resync, data dictionary) | Unit-tested; **works on hardware** (ESP32-S3 to SKR Pico) |
| ESP32-S3 firmware (`firmware/esp32s3`): WiFi access point, status page, console, `printer.cfg` editor, test bench, printer page, the Klipper host's jobs (heaters, TMC2209, moves, homing, printing from flash). Shipped ready to flash | **On hardware**: link, heating, PID, motor test, TMC2209, stallGuard, full homing (2026-10-27), a print file run in mid-air (2026-10-27), WiFi updates, the print and motion benchmarks (2026-10-29). Not yet: a real printed part |
| `skr-pico-uart` printer-board firmware (**plain Klipper v0.13.0**, serial on UART0; the ESP32-S3 firmware is the only bespoke part) | **Works on hardware**: link, heater, thermistor, steppers, TMC UART, DIAG pins (with the 2026-10-10 build, which also held the since-removed, never-used segment queue; [why it went](docs/research/segq-retired.md)) |
| Motion planner + G-code engine (lookahead, junction deviation, trapezoids, pressure advance, G-code state) | **On hardware** for homing, a print file in mid-air and the benchmarks; checked in simulation for exact positions, speed and acceleration limits. Never yet used for a real part |
| Delta kinematics (`kinematics: delta`, [docs/DELTA.md](docs/DELTA.md)); DECKHAND needs its own delta build | **PC-tested only** (head path vs the arm geometry, three-endstop homing); never on a real delta; no delta calibration yet |
| Pasta Inspector dongle support (`/websocket`, Moonraker-style: exclude object, pause, M112) | **Engine PC-tested; firmware written but not compiled or tried with a real dongle** ([docs/PASTA_DONGLE.md](docs/PASTA_DONGLE.md)) |
| Other board presets (generic RP2040, Creality 4.2.x, RAMPS mega2560, Ender-3 1.1.x, USB SKR Pico) | **Compile only**; never run on hardware |
| Flasher app (`flasher.py`): Slave tab (printer board) and Master tab (ESP32-S3) | ESP32-S3 flashing **used on hardware** on Linux Mint. Other paths (SD card, avrdude, Windows/macOS) tested with mocks only |
| Firmware build tool (CLI + Slave tab "My own build") | Works on Linux (sandbox and the author's Linux Mint laptop); Windows needs WSL |
| `mcu_probe`, `fake_mcu.py`, one-command setup, Tools window | Working on Linux Mint. Probe tried on an SKR Pico over USB |
| Real printing | A print file **ran end to end on the Tina 2** (2026-10-27) but in mid-air (Z offset not found); upload, print from flash or SD card, pause / resume / cancel are simulated. **No real part printed yet** |
| Micro SD card (`firmware/esp32s3/main/sdcard.c`) | **Works on hardware** (2026-10-19: mount, file list, upload; printing from it not yet tried): SPI card for big G-code files, file list / print / delete on the Printer page, `SDCARD_PRINT_FILE FILENAME=`, M20/M23/M24 (the command side is unit-tested). See [docs/SD_CARD.md](docs/SD_CARD.md) |
| Buttons on the case (`firmware/esp32s3/main/panel.cpp`, `host/src/panel.cpp`) | **Touch works on hardware** (2026-10-26: Preheat and Home from staple pads melted into the Tina 2's case); up to four touch pads (bolts on wires) or switches: Preheat, Home and E-stop (both on a 1 s hold), Unload, and more. CAPTAIN and SKIPPER. See [docs/BUTTONS.md](docs/BUTTONS.md) |
| Tuning (`host/src/shaping.cpp`, `resonance.cpp`, `bedmesh.cpp`, `machine_pro.cpp`; Tune page) | **Built and PC-tested; the shaping and pressure-advance maths timed on the ESP32 (2026-10-29), not yet used on a moving printer** (2026-10-24): input shaping, pressure advance, accelerometer search and shaper calibration (matches Klipper's), bed mesh, screw tilt, arcs, G10/G11, exclude object, tuning tower. USB accelerometer: compile-only. CAPTAIN and SKIPPER only. See [docs/TUNING.md](docs/TUNING.md) |
| Little OLED display (`firmware/esp32s3/main/oled.cpp`, `components/ktft_oled`) | **Works on hardware** (2026-10-26: temperatures and the shutdown screen on a 0.96" OLED in the Tina 2); built 2026-10-23: SSD1306 / SSD1315 / SH1106, 128x64 or 128x32, on GPIO8/9: temperatures, progress bar, HEATING / HOMING / EXTRUDING / LOADING, "no board" with the WiFi name and password. See [docs/OLED.md](docs/OLED.md) |
| Three builds by board memory: DECKHAND / CAPTAIN / SKIPPER ([docs/TIERS.md](docs/TIERS.md)), Flasher "Flash for my board" | **CAPTAIN runs on hardware** (2026-10-20: picked by the Flasher, 2 MB PSRAM found and used); WiFi firmware updates work (2026-10-20); the important-events log works (2026-10-28); not yet tried there: bigger look-ahead with PSRAM, slicer upload (OctoPrint-style), home WiFi, thumbnails, print history, event log, printer.cfg backups, crash reports |
| Print benchmark, motion benchmark, INT32/64 test build ([docs/BENCHMARK.md](docs/BENCHMARK.md)) | **On hardware** (2026-10-28/29): both benchmarks run to the end on the Tina 2 with the SKR Pico and the drivers off; results in the important-events log. The integer build has not moved motors yet |
| Fans | Hotend fan **confirmed on hardware**; part-cooling fan and the third fan output not yet (possibly a broken wire, will update when tested) |

Total checks: 1,825 C++ unit checks (run twice: with double and with integer motion maths), the OLED layout tests and 192 Python tests (including power-cycle, stallGuard and printing runs against a simulated board), all passing in the sandbox. Passing tests mean the maths and bookkeeping are right, not that a printer will behave.

## Screenshots

The GUI building the tools and running the unit tests:

![Building the tools in the Skipper Flipper GUI](docs/images/gui-build.webp)

The GUI running the probe against the built-in fake MCU (simulated, no hardware):

![Running mcu_probe against the fake MCU](docs/images/gui-probe.webp)

Both screenshots use the **simulated** board, not real hardware.

## Flash a board (Linux, macOS, Windows)

```
python3 flasher.py          (Windows: double-click Flasher.bat; macOS: Flasher.command)
```

You need Python 3.8+ with Tk (Linux: `sudo apt install python3-tk`; on Windows and macOS the installer from python.org includes Tk). No toolchain is needed: firmware for every board preset (five boards, plus a serial variant of the SKR Pico) ships in `firmware/prebuilt/`. Pick the **Slave** tab, choose your board, press **Flash...** and confirm. RP2040 boards: hold BOOTSEL while plugging in USB. STM32 boards such as the Creality 4.2.x: put the SD card in this computer, **Browse** to it, flash, then move the card to the printer and power-cycle it. AVR boards need `avrdude` installed. The **Master** tab flashes an ESP32-S3 with esptool (Install esptool, then **Use shipped firmware**, pick the port, Flash). The port box fills itself with the ESP32-S3's port when it can tell which one it is (otherwise pick it). The shipped image starts a WiFi network `SkipperFlipper-XXXX` (WPA2, default password `skipperflipper`, change it before real use) with a page at `http://192.168.4.1/`, a live log at `/console` and `printer.cfg` at `/config`; it talks Klipper to the printer board over UART (identify, clock sync, `printer.cfg`, heaters, TMC2209 drivers, motor tests at `/test`, homing and running a print file from the Printer page: all on hardware with the SKR Pico; a real printed part not yet). Logs appear on the chip's native USB port. **ESP32-S3 only**: other ESP32 chips (original, S2, C3) are not supported, and esptool refuses to write if it finds a different chip. You can also flash your own `.bin`. To connect the ESP32-S3 to an SKR Pico with four jumper wires, see [docs/WIRING_SKR_PICO.md](docs/WIRING_SKR_PICO.md).

![The Flasher, Slave tab with the Creality 4.2.x preset](docs/images/flasher-slave.webp)

**Only the `skr-pico-uart` firmware (SKR Pico) has run on a board**, where it has done everything above; the other presets have never been flashed. Read [docs/FIRMWARE.md](docs/FIRMWARE.md) first, and do not flash a printer you cannot recover.

## Quick start (Linux Mint / Ubuntu / Debian)

```bash
git clone https://github.com/The-Dorkknight/esp-printhost
cd esp-printhost
sudo python3 printhost.py
```

That one command installs what is missing (build tools, cmake, zlib, python3-tk, acl), gives you access to USB serial ports without logging out, stops ModemManager and brltty from grabbing printer boards, builds `mcu_probe`, runs the tests, and opens the GUI as your normal user. It is safe to re-run. [HOW_IT_WORKS.md](HOW_IT_WORKS.md) lists exactly what it changes on your system.

Options: `--check` (report only, change nothing), `--no-gui`, `--clean`, `--yes`, `--version`.

If anything fails it prints the path of a log file in `~/esp-printhost-logs/`. Attach that when you open an issue.

macOS: the C++ library and tests build (`python3 host/tools/build.py`), but `mcu_probe` is Linux-only (it uses Linux's `termios2` for 250000 baud) and the one-command setup is Linux-only.

## Using the GUI

1. **Build tools** compiles everything and runs the unit tests. Output is colour-coded per tool.
2. **Fake MCU: Start** runs a pretend Klipper board (no hardware). Its port appears in the Port box.
3. **Run probe** connects, reads the board's pin list into the **Blink pin** dropdown, and prints uptime and state. Tick **Blink pin** and pick a pin to also toggle it 10 times.
4. The output box is selectable (Ctrl+C, right-click, **Copy all**, **Save as...**). Everything is also logged to `~/esp-printhost-logs/`.

## Building and flashing firmware

Open the **Firmware** tab (or use `python3 host/tools/skipper_fw.py`), pick your board, then **Check tools**, **Fetch Klipper**, **Build**. The tool downloads Klipper v0.13.0 from GitHub into `~/.skipper-flipper/`, builds it unmodified with the board's preset (Klipper's own menuconfig choices only) and saves the result with a SHA-256 list. **Flash...** shows what it is about to do and asks first. Details, per-board warnings and the speed arithmetic are in [docs/FIRMWARE.md](docs/FIRMWARE.md).

**Warning:** wrong or untested firmware can leave a board unresponsive until it is re-flashed through its bootloader. Do not flash a printer you cannot afford to recover. Presets are marked `compile-only / unverified` for a reason.

## Try it without hardware

```bash
python3 host/tools/fake_mcu.py            # prints /dev/pts/N
./host/build/mcu_probe /dev/pts/N 250000 --pins --blink PA5
```

## Try it on a real board

You need a board running **stock Klipper MCU firmware** (the Flasher ships plain Klipper v0.13.0 builds in `firmware/prebuilt/`, or build your own with the Firmware tab). Stop Klipper or anything else using the port, then:

```bash
./host/build/mcu_probe /dev/serial/by-id/<your-board> 250000 --pins --blink <pin>
```

`--pins` lists the pin names the board accepts. The probe validates your pin name before touching the board, and if the board is already configured it **resets it** first (the USB device re-appears after about two seconds). Do not run it on a board that is in the middle of a print.

Tested so far: BTT SKR Pico (RP2040, Klipper v0.10.0-181). The probe identified it, read 31 pin names, reset it, configured `gpio24` and sent 10 toggles with 0 CRC errors.

## Troubleshooting

| Symptom | Likely cause and fix |
|---|---|
| `/dev/ttyUSB0` disappears when an Ender / Creality board is plugged in | brltty grabbing the CH340 chip. Re-run `sudo python3 printhost.py` and say yes to disabling it |
| `Permission denied` on the port | Not in `dialout` yet. `printhost.py` fixes today's session; log out and in for the long term |
| `Device or resource busy` | Klipper, Cura or a serial monitor has the port open. Close it |
| `no identify_response` | Wrong port or baud, board is not running Klipper firmware, or it needs a power cycle |
| `already configured` | The probe resets it automatically; if that fails, power-cycle the board |
| GUI will not open | `sudo apt install python3-tk`, or run `python3 printhost.py --no-gui` |
| Probe says OK but nothing lights up | See [Known limitations](#known-limitations) |

## Known limitations

- **It has not printed a real part yet.** Homing and a whole print file have run on the Tina 2, but in mid-air: first layer, Z offset, extrusion and cooling are untested in practice.
- **Heaters: stay with the printer.** Klipper's heater safety is ported and has shut the board down correctly on hardware ("ADC out of range"), but this is experimental firmware: keep a hand near the power switch and never leave it running unattended.
- **The tuning features have not moved a printer**: input shaping, pressure advance, accelerometer calibration, bed mesh, screw tilt, arcs, firmware retraction, exclude object and tuning tower are built and PC-tested; the shaping maths has been timed on the ESP32, nothing more.
- **The INT32/64 test build** has passed the PC tests (within one step of the `double` maths everywhere) and both benchmarks on the ESP32, but has not driven motors with its integer shaper yet.
- **Sending waits for the board**: about 30 % of one core in the print benchmark is the ESP32 waiting for the SKR Pico's acknowledgements, where Klipper keeps sending. Fine at the Tina 2's speeds; it is the next speed work.
- **Delta is untested on a real printer** and has no calibration (`DELTA_CALIBRATE`); the DECKHAND delta image is a separate build and its flash size is not measured.
- **The Pasta Inspector dongle link is untested**: the websocket server has not been built with ESP-IDF or run against a real ESP32-CAM; CAPTAIN and SKIPPER only.
- **Only the SKR Pico has run.** The other board presets are compile-only, and their bootloader details are assumptions. Small 8-bit boards give Klipper a much smaller move queue than the SKR Pico's 1,024 commands; this host has not been tried with one.
- **Flash is nearly full** on DECKHAND (about 6 KB free) and CAPTAIN (about 14 KB); new features may need something trimmed.
- The part-cooling fan and the third fan output are not confirmed (possibly a broken wire). The cause of one early SKR Pico lock-up after a shutdown (2026-10-13) is still unknown.
- `mcu_probe` is Linux-only.
- The GitHub workflows in `.github/workflows/` have not been checked for this release.
- Presets for boards with clone chips (some Creality 4.2.x) are flagged unverified; check your chip.
- Design documents describe intent, not finished work.

## Project layout

```
printhost.py          one-command setup + launcher
flasher.py            the Flasher window (also Flasher.bat / Flasher.command)
HOW_IT_WORKS.md       what it reads, changes and never does
docs/                 DESIGN, PROTOCOL, ENGINE, FIRMWARE, TUNING, TASKS, project page
firmware/esp32s3/       ESP32-S3 firmware (ESP-IDF project) + build.sh
firmware/prebuilt-esp32s3/  shipped ESP32-S3 images (DECKHAND / CAPTAIN / SKIPPER; test-int32/ for the INT32/64 test build)
firmware/boards/      board presets (.config + .json)
firmware/prebuilt/    shipped plain-Klipper firmware per board (+ pin tables); refresh with skipper_fw.py bundle
host/                 portable C++17 core (the same code runs on the PC and, as an ESP-IDF component, on the ESP32-S3)
  include/ph/         wire, msgproto, json_mini, config_ini, gcode, motion, planner, engine
  src/  test/         implementation + 1,825 unit checks
  tools/              build.py, skipper_fw.py, printhost_gui.py (+ fw_panel, skipper_theme),
                      mcu_probe.cpp, fake_mcu.py
```

## Roadmap

See [docs/TASKS.md](docs/TASKS.md). Next, in order:

1. Make the integer input shaper the standard in every build (keeping the `double` planner), now that it measures about 3.7 times faster on the ESP32.
2. Send without waiting for each acknowledgement, the way Klipper does, to win back most of the 30 % of a core spent waiting.
3. Move to the QIDI X-Smart 3 test bench and print a first real part.
4. Try the tuning features (input shaping, pressure advance, bed mesh) on a moving printer.

The look is a 1928 black-and-white cartoon (visuals only).

## Contributing, security, third-party

Copyright (C) 2026 Matthew Armstrong ([The-Dorkknight](https://github.com/The-Dorkknight)).

[docs/HARDWARE_LOG.md](docs/HARDWARE_LOG.md) (everything tested on real hardware) · [docs/WIRING_SKR_PICO.md](docs/WIRING_SKR_PICO.md) (ESP32-S3 to SKR Pico wiring) · [docs/TEST_BENCH.md](docs/TEST_BENCH.md) (test page and heater safety) · [docs/TUNING.md](docs/TUNING.md) (input shaping, pressure advance, bed mesh, screw tilt) · [docs/BUTTONS.md](docs/BUTTONS.md) (touch pads / buttons on the case) · [docs/TINA2_INSTALL.md](docs/TINA2_INSTALL.md) (WEEDO Tina 2 installation) · [docs/configs/qidi-xsmart3-skr-pico.md](docs/configs/qidi-xsmart3-skr-pico.md) (QIDI X-Smart 3 test bench) · [docs/CHAMBER_SENSOR.md](docs/CHAMBER_SENSOR.md) (chamber thermistor) · [docs/BENCHMARK.md](docs/BENCHMARK.md) (benchmark, integer test build, important events) · [docs/research/klipper-sources.md](docs/research/klipper-sources.md) (research notes; [how fast can it go](docs/research/klipper-grade-performance.md)) · [CONTRIBUTING.md](CONTRIBUTING.md) · [SECURITY.md](SECURITY.md) · [THIRD_PARTY.md](THIRD_PARTY.md) · [CHANGELOG.md](CHANGELOG.md)

## Research paper

**[A Frankenstein Experience: How cheap and lean can a Klipper brain get?](docs/paper/A_FRANKENSTEIN_EXPERIENCE.md)** (version 1.2) · [PDF](docs/paper/A_FRANKENSTEIN_EXPERIENCE.pdf)

The paper's text and figures: [CC BY-SA 4.0](docs/paper/LICENSE.md)

**How to cite:** Unknown (n.d.). *A Frankenstein Experience: How cheap and lean can a Klipper brain get?.* Version 1.2.
