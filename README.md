# esp-printhost

> ## WORK IN PROGRESS: MANY FEATURES MAY BE BROKEN
> **Proof of concept, `0.1.0-wip`. Slow-moving hobby project.** The design is still changing, nothing here runs a printer yet, and breaking changes are expected. Read [Known limitations](#known-limitations) before you plug anything in.

**GPL-3.0** · Linux (Mint / Ubuntu) · C++17 core + Python tools · [Project page](https://The-Dorkknight.github.io/esp-printhost/)

Licensed under the GNU General Public License v3.0, see [LICENSE](LICENSE). **No warranty and no liability:** this is provided as is. You use it at your own risk, on your own hardware, and the author is not responsible for damage to boards, printers or anything else.

## What this is

An idea for reviving old, donated 3D printers: a cheap ESP32-S3 becomes the "brain" (WiFi, web UI, text config, motion planning) and the printer's original board becomes a simple slave running a modified Klipper MCU firmware. The full plan is in [docs/DESIGN.md](docs/DESIGN.md).

**What exists today is only the PC-side groundwork:** a tested C++ library that speaks Klipper's MCU protocol, plus a few tools to prove it against a real board. There is no ESP32 firmware, no firmware fork, no web UI and no motion control yet.

| Part | State |
|---|---|
| Klipper wire protocol (CRC16, VLQ ints, framing, resync, data dictionary) | Working, 122 unit tests pass |
| `printer.cfg` parser, G-code parser, mm-to-steps accumulator, pseudo pressure advance | Working as library code, unit-tested, not used by anything yet |
| `mcu_probe` (identify a real board, list pins, toggle one pin) | Works against a BTT SKR Pico (RP2040); physical pin change not yet confirmed |
| `fake_mcu.py` (pretend board, no hardware needed) | Working |
| One-command setup `printhost.py` and the GUI | Working on Linux Mint |
| Klipper MCU firmware fork with a segment queue (`src/segq.c`) | **Not started** |
| ESP32-S3 firmware, WiFi, web UI | **Not started** |
| Anything that moves a motor or heats a nozzle | **Not started** |

## Screenshots

The GUI building the tools and running the 122 tests:

![Building the tools in the esp-printhost GUI](docs/images/gui-build.webp)

The GUI running the probe against the built-in fake MCU (simulated, no hardware):

![Running mcu_probe against the fake MCU](docs/images/gui-probe.webp)

These two screenshots were taken against the **simulated** board, not real hardware.

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

## Try it without hardware

```bash
python3 host/tools/fake_mcu.py            # prints /dev/pts/N
./host/build/mcu_probe /dev/pts/N 250000 --pins --blink PA5
```

## Try it on a real board

You need a board running **stock Klipper MCU firmware** (build it from Klipper with `make menuconfig`; this project does not ship firmware yet). Stop Klipper or anything else using the port, then:

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

- **Nothing here controls a printer.** The probe identifies a board, resets it and toggles one output pin. Do not rely on it for heaters, steppers or anything safety-related.
- **Pin toggle is not physically confirmed.** On the SKR Pico the test used `gpio24`, which (as far as we can tell from BTT's pin map, not verified) is the NeoPixel/RGB header. A NeoPixel needs a timed data stream, so a plain on/off will not light one. The protocol side worked; try a pin with a visible LED or a multimeter on it.
- `finalize_config` is sent with a fixed placeholder CRC, which is fine for stock firmware but is not a real config checksum yet.
- The ESP-IDF build of the library is untested; only the PC build is exercised.
- `mcu_probe` is Linux-only.
- The GitHub workflows in `.github/workflows/` have not been run yet.
- Design documents describe intent, not finished work. Segment-queue commands (`segq_*`) are a draft and exist only in the simulator.

## Project layout

```
printhost.py          one-command setup + launcher
HOW_IT_WORKS.md       what it reads, changes and never does
docs/                 DESIGN.md, PROTOCOL.md (draft), TASKS.md, project page
host/                 portable C++17 core (also an ESP-IDF component, untested)
  include/ph/         wire, msgproto, json_mini, config_ini, gcode, motion
  src/  test/         implementation + 122 unit tests
  tools/              build.py, printhost_gui.py, mcu_probe.cpp, fake_mcu.py
```

## Roadmap

See [docs/TASKS.md](docs/TASKS.md). Next big step: fork Klipper's MCU firmware and add the segment queue, then the ESP32-S3 side.

## Contributing, security, third-party

[CONTRIBUTING.md](CONTRIBUTING.md) · [SECURITY.md](SECURITY.md) · [THIRD_PARTY.md](THIRD_PARTY.md) · [CHANGELOG.md](CHANGELOG.md)

Copyright (C) 2026 Matthew Armstrong ([The-Dorkknight](https://github.com/The-Dorkknight)).
