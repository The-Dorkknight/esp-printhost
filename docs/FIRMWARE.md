# Firmware: plain Klipper on the printer board, Skipper Flipper on the ESP32-S3

> **The printer board runs plain Klipper v0.13.0**, built from the release with only Klipper's own
> menuconfig choices (chip, flash chip, USB or UART). The ESP32-S3 firmware is the only bespoke part.
> Status per preset is in its `.json`: `skr-pico-uart` has run in a WEEDO Tina 2 (see HARDWARE_LOG.md;
> those runs used the 2026-10-10 build, which also held the since-removed, never-used segment queue);
> the other presets are **compile-only**. Wrong firmware can leave a board unresponsive until it is
> re-flashed through its bootloader.

## What is in the repo

| Path | What |
|---|---|
| `firmware/boards/<id>.config` + `<id>.json` | Board presets: a Klipper `.config` seed (menuconfig choices only) and metadata (description, flash method, notes, sources, status). |
| `host/tools/skipper_fw.py` | CLI + importable library: check tools, fetch Klipper, build, flash. |
| `host/tools/fw_panel.py` | The "Firmware" tab of `printhost_gui.py`. |
| `host/tools/test_skipper_fw.py` | Unit tests of the tooling (no network, no hardware). |

Klipper is not vendored. `fetch` makes a shallow clone of tag **v0.13.0** (commit
`61c0c8d2ef40...`) into `~/.skipper-flipper/klipper-v0.13.0` and verifies the commit. `build`
writes the preset's `.config` seed, runs Klipper's own `make olddefconfig`, `make clean` and `make`,
and nothing else: no file in Klipper is added or changed. Before building it checks with
`git status` that Klipper's `src/`, `lib/`, `scripts/` and `Makefile` are exactly the release and
refuses otherwise. A Klipper folder from before 2026-10-25 that still holds the old segment-queue
overlay is cleaned first (also by hand: `skipper_fw.py clean`). The firmware then reports
`v0.13.0-0-g61c0c8d`, with no "-dirty". Background: [research/segq-retired.md](research/segq-retired.md).

## Shipped firmware and the Flasher

`firmware/prebuilt/<board>/` holds plain Klipper v0.13.0 built for each preset (rebuilt 2026-10-25): the flash file (`klipper.uf2`, `klipper.bin` or
`klipper.elf.hex`), `klipper.dict`, `pins.json` (pin names the firmware accepts, read
from the dictionary; this is the chip's pin naming, not a map of your board's connectors)
and `build-info.json` (SHA-256, sizes, Klipper commit). They were built in the Linux
sandbox by `skipper_fw.py bundle`. `flasher.py` flashes these directly, so Windows and macOS users need no
toolchain. "My own build" in the Slave tab uses what you built yourself in
`~/.skipper-flipper/out/`. Local building needs Linux (or WSL); it is untested on macOS
and the Build buttons are disabled on Windows.

Flash methods per chip: RP2040 `uf2-copy`; STM32 `sdcard-copy` (no DFU yet); AVR
`avrdude`. Windows detects the UF2 drive and the SD card by removable drive letter and
macOS by `/Volumes`; both are tested only against mock folders and a mocked Windows API.

## SKR Pico over UART (`skr-pico-uart`)

Same as `skr-pico` but Klipper talks on UART0 (GPIO0 TX / GPIO1 RX) at 250000 baud instead of USB,
as in BTT's own Raspberry Pi UART instructions. Used for the ESP32-S3 link; wiring in
[WIRING_SKR_PICO.md](WIRING_SKR_PICO.md). Runs in the Tina 2 (link, heaters, steppers, TMC2209). After flashing it the board no longer
answers on USB; flash `skr-pico` to go back.

## ESP32-S3 firmware

`firmware/esp32s3/` is an ESP-IDF v5.4.1 project: a heartbeat line every 5 s on the chip's
native USB serial/JTAG console (optional LED via menuconfig, off by default because many
boards have a colour LED that needs a timed data stream), a WPA2 WiFi access point
`SkipperFlipper-XXXX` (default password `skipperflipper`, set in menuconfig) and a status
page at `http://192.168.4.1/` (`/status.json` too). Everything it logs is also kept in an 8 KB
ring buffer and shown live at `http://192.168.4.1/console` (the page polls `/log?since=N` once a
second), so no USB cable is needed to watch it. Since the UART link build it also talks to a printer board running Klipper serial firmware: it asks it to identify itself (reading and unpacking the data dictionary with the ROM's built-in inflate) and then checks every 5 s that it still answers (`get_config`), using `ph::KLink` from `host/`, the same code tested on the PC (`klink_probe`, `test_klink.py`). Since K1/K2 it also keeps the board's clock in sync (`ph::ClockSync`, like Klipper's `clocksync.py`; measured clock, drift and round trip on the status page) and configures the board from `printer.cfg`, stored on the chip and edited at `/config` (LEDs, thermistors, heaters, fans, TMC2209 drivers and steppers for motor tests; other sections are listed as not used yet). It reads thermistors from `[temperature_sensor]` sections (`config_analog_in` plus `query_analog_in`, converted with `ph::Thermistor`, Klipper's maths; confirmed on hardware with two thermistors). The status page has a G-code console (`/gcode?cmd=`) for `SET_LED`, `M105` and `HELP`. A `/settings` page (stored in NVS) sets the UART pins (default TX GPIO43, RX GPIO44), baud, and the status LED (default: WS2812 colour LED on GPIO21, the Waveshare ESP32-S3-Zero's LED per Waveshare; plain LED also supported). It also answers DNS for every name with its own address and redirects unknown pages to the status page (captive portal).
The shipped image (`firmware/prebuilt-esp32s3/`) is bootloader + partition table + app merged
for 2 MB flash and written at offset 0x0; the app is about 990 KB in a 1.5 MB partition.
Built in the Linux sandbox. Earlier builds ran on one ESP32-S3-Zero type board with an SKR Pico (docs/HARDWARE_LOG.md): status page, Klipper link, and the SKR Pico LED from a phone. Clock sync, `printer.cfg`, the console and thermistor reading have since run on it. The current build adds heaters (with Klipper's safety checks), fans, TMC2209 drivers, motor tests and the test bench (`/test`, docs/TEST_BENCH.md) and has not been flashed yet. The **Detect chip** button runs esptool `flash-id`, which reports the real flash size (the status page shows the size the firmware was built for). `firmware/esp32s3/build.sh` rebuilds and re-ships it
(needs ESP-IDF v5.4 installed). A test fails if the sources change without a rebuild.
Unverified: the heartbeat on the USB console, the console port on other boards, WiFi behaviour and range, the 2 MB flash header on
your module, and everything else that needs a real chip.

The Master tab (`skipper_esp.py`) is ESP32-S3 only and runs Espressif's esptool on a `.bin` you supply. esptool
is not bundled (the tab has an Install button that creates a private virtualenv in
`~/.skipper-flipper/esp-venv` and pip-installs esptool into it after asking; this avoids the
"externally managed environment" refusal on Mint/Ubuntu, and may need `sudo apt install python3-venv`). The port box picks the ESP32-S3's own USB port when it can tell (Espressif's USB ID 303a in Linux `/dev/serial/by-id` names or the Windows registry, else the only serial port that is not a Klipper board) and rescans every 2 s while empty; **Detect chip** uses it. With several unknown ports nothing is picked, because a CH340 or CP210x on an ESP32 board looks the same as one on a printer board. For your own `.bin`, offsets are yours to get right.

Before any flash the tool recomputes the SHA-256 of the file and compares it with
`build-info.json`; a mismatch or missing record refuses. Flashing is also refused while a
build is running or when the method does not fit the board's chip family.

## Using it

GUI: `python3 host/tools/printhost_gui.py`, tab **Firmware**: pick the board,
**Check tools**, **Fetch Klipper**, **Build**, then **Flash...** (it asks first).
**Menuconfig** opens Klipper's menu in a terminal (x-terminal-emulator,
gnome-terminal, xfce4-terminal or konsole; if none is found the command is shown in
the Output box), then builds there.

CLI (same code, same output lines):

```
python3 host/tools/skipper_fw.py list
python3 host/tools/skipper_fw.py check [board]        # prints the exact "sudo apt install ..." line
python3 host/tools/skipper_fw.py fetch
python3 host/tools/skipper_fw.py build skr-pico [--menuconfig] [--jobs N]
python3 host/tools/skipper_fw.py flash skr-pico [--artifact DIR] [--method M] [--port P] [--mount PATH] [--dry-run] [--yes]
python3 host/tools/skipper_fw.py clean [--klipper-dir DIR]     # remove the old segment-queue overlay
```

Toolchains (Linux Mint/Ubuntu): `printhost.py` offers them as an optional step
(`--no-firmware` skips it), or: `sudo apt install git gcc-arm-none-eabi
binutils-arm-none-eabi libnewlib-arm-none-eabi gcc-avr avr-libc binutils-avr
dfu-util avrdude stm32flash` (roughly 140 MB download).

Each build goes to `~/.skipper-flipper/out/<board>-<YYYYmmdd-HHMM>/` with the
firmware file(s), `klipper.elf`, `klipper.dict`, `klipper.config` (the final
resolved config) and `build-info.json` (Klipper tag + commit, preset, `plain_klipper: true`,
tool version, flash/RAM size, sha256 of every artifact). Klipper's own `make
olddefconfig` expands the preset `.config` seed (verified in v0.13.0's Makefile:
the `.config` rule runs `lib/kconfiglib/olddefconfig.py`), then `make clean` and
`make` run, so no stale objects are left from an earlier build.

## Presets and verification status

Sizes are from the 2026-10-25 sandbox build (plain Klipper v0.13.0, Ubuntu 24.04 gcc-arm-none-eabi /
gcc-avr); "RAM" is static data+bss only.

| Board id | Arch | Flash method | Status | Flash bytes | Static RAM bytes |
|---|---|---|---|---|---|
| `creality-4.2.x` | stm32 | sdcard-copy | compile-only | 33363 | 1144 |
| `ender3-1.1.x` | avr | avrdude | compile-only | 35482 | 417 |
| `generic-rp2040` | rp2040 | uf2-copy | compile-only | 39248 | 1544 |
| `ramps-mega2560` | avr | avrdude | compile-only | 36252 | 418 |
| `skr-pico` | rp2040 | uf2-copy | compile-only | 39248 | 1544 |
| `skr-pico-uart` | rp2040 | uf2-copy | hardware-tested (earlier build) | 36708 | 1236 |

- **skr-pico**, **generic-rp2040**: RP2040, USB serial, no bootloader, W25Q080
  flash chip (Klipper defaults; same header comments as Klipper's SKR Pico config).
  Unverified: that your board's flash chip is compatible with the W25Q080 setting.
  `generic-rp2040` is also the firmware for a Klipper USB accelerometer (RP2040 + ADXL345).
- **creality-4.2.x**: STM32F103, 28 KiB bootloader offset, 8 MHz crystal, USART1
  PA10/PA9 at 250000 baud (from the header of Klipper's `generic-creality-v4.2.7.cfg`).
  Unverified: clone chips (GD32F103 etc.) and other bootloaders.
- **ramps-mega2560**: ATmega2560 at 16 MHz, 250000 baud, avrdude protocol `wiring`.
- **ender3-1.1.x**: ATmega1284P at 16 MHz, 250000 baud, avrdude `arduino` at 57600
  baud (from a comment in Klipper's `printer-creality-ender3-2018.cfg`). Boards
  without a bootloader need an ISP programmer and are not covered.

Every option name in the `.config` seeds was checked against Klipper v0.13.0's
`src/*/Kconfig`, and the resolved `.config` of each build was inspected (MCU,
clock, flash offset, serial settings). A test checks that no shipped dictionary has a `segq`
command and that each build is marked plain Klipper.

## Flashing

Flashing is never implicit: `flash` needs `--yes` or the board id typed back in a
terminal; the GUI shows a confirmation dialog (board, file, method, port/drive) and
passes `--yes` only after OK. `--dry-run` prints the exact steps and changes nothing.
Always: wrong or broken firmware can leave a board unresponsive until it is
re-flashed through its bootloader (or an ISP/SWD programmer if there is none).

| Method | What it does | You must |
|---|---|---|
| `uf2-copy` (RP2040) | Copies `klipper.uf2` to the `RPI-RP2` drive (looked up in `/media/$USER`, `/run/media/$USER`, `/Volumes`, or `--mount`). The board reboots by itself when the copy ends. | Hold BOOTSEL while plugging in USB; wait for the drive to appear. |
| `sdcard-copy` (Creality 4.2.x) | Copies `klipper.bin` to the SD card as `skipper-<YYYYmmdd-HHMMSS>.bin` (a new name every time, because the stock Creality bootloader ignores names it has flashed before), then runs `sync`. | Pass `--mount` (or have exactly one removable drive mounted), eject, insert, power-cycle the printer. No visible progress on most boards: connect and look for Klipper's identify afterwards. |
| `avrdude` (AVR) | `avrdude -p<mcu> -c<protocol> -P<port> [-b baud] -D -Uflash:w:<hex>:i`: the flags of Klipper's `src/avr/Makefile` flash rule, run directly instead of through `make flash`. | Pass `--port`; close programs using the port. |
| `manual` | Prints the firmware path and where to read instructions. Touches nothing. | Flash it yourself. |

## Tests without hardware

- `python3 host/tools/test_skipper_fw.py`: the tooling (presets, plain-Klipper check and the
  old-overlay clean-up, mounts, flash plans/refusals, artifacts).
- `python3 host/tools/test_skipper_ship.py`: the shipped firmware (hashes, plain Klipper, pins).
- `python3 host/tools/test_fake_mcu.py`: the simulated board (plain Klipper commands).

## Not done / open items

- Only `skr-pico-uart` has run on hardware, and with the earlier build (see the banner above).
- Bootloader/flash details for individual boards are taken from Klipper's own
  comments; revisions in the wild differ.
- No STM32 USB-DFU, dfu-util or stm32flash flashing method yet (the tools are
  installed by `printhost.py` for later presets).
