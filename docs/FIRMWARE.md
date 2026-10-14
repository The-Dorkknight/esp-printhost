# Firmware (Klipper-based MCU firmware with the segment queue)

> **Honest status: compile-only.** Every preset below was built in a Linux sandbox
> with Klipper v0.13.0 and its toolchain. **No firmware here has ever been flashed
> to or run on a real board.** Pin polarity, timer load, bootloader/flash steps and
> the DDA step rates are design assumptions until someone tests them. Do not use
> it on a printer you cannot afford to re-flash through its bootloader.

## What is in the repo

| Path | What |
|---|---|
| `firmware/skipper/segq_core.[ch]` | Portable segment algorithm (C99), unit tested on a PC. |
| `firmware/skipper/segq.c` | Klipper glue: commands, DDA timer, step/dir pins (protocol v1, see PROTOCOL.md section 2). |
| `firmware/boards/<id>.config` + `<id>.json` | Board presets: a Klipper `.config` seed and metadata (description, flash method, recommended DDA rate, notes, sources, build sizes). |
| `host/tools/skipper_fw.py` | CLI + importable library: check tools, fetch Klipper, build, flash. |
| `host/tools/fw_panel.py` | The "Firmware" tab of `printhost_gui.py`. |
| `host/tools/test_skipper_fw.py` | Unit tests of the tooling (no network, no hardware). |

Klipper is not vendored. `fetch` makes a shallow clone of tag **v0.13.0** (commit
`61c0c8d2ef40...`) into `~/.skipper-flipper/klipper-v0.13.0` and verifies the commit.
The tool then *overlays* our files into that clone (idempotent, removable):

- copies `segq.c`, `segq_core.c`, `segq_core.h` into Klipper's `src/`;
- appends `config WANT_SEGQ` (bool, "Skipper Flipper segment queue (experimental)",
  default y, depends on HAVE_GPIO) to `src/Kconfig`;
- appends `src-$(CONFIG_WANT_SEGQ) += segq.c segq_core.c` to `src/Makefile`.

Both appended blocks are fenced by `skipper-flipper segq` marker comments, so
`skipper_fw.py overlay --remove` restores the stock files byte for byte (checked
with `git status` on a real clone). With the "Segment queue" box off (`--no-segq`)
the overlay stays but `CONFIG_WANT_SEGQ` is off: the build then has exactly the
same section sizes as an untouched Klipper (verified for creality-4.2.x and
ramps-mega2560; the compiled code differs only by the build-time version string).

## Shipped firmware and the Flasher

`firmware/prebuilt/<board>/` holds the firmware built for each preset with the segment
queue on (Klipper v0.13.0): the flash file (`klipper.uf2`, `klipper.bin` or
`klipper.elf.hex`), `klipper.dict`, `pins.json` (pin names the firmware accepts, read
from the dictionary; this is the chip's pin naming, not a map of your board's connectors)
and `build-info.json` (SHA-256, sizes, Klipper commit). They were built in the Linux
sandbox by `skipper_fw.py bundle`, so they carry the same **compile-only** status as the
presets. `flasher.py` flashes these directly, so Windows and macOS users need no
toolchain. "My own build" in the Slave tab uses what you built yourself in
`~/.skipper-flipper/out/`. Local building needs Linux (or WSL); it is untested on macOS
and the Build buttons are disabled on Windows.

Flash methods per chip: RP2040 `uf2-copy`; STM32 `sdcard-copy` (no DFU yet); AVR
`avrdude`. Windows detects the UF2 drive and the SD card by removable drive letter and
macOS by `/Volumes`; both are tested only against mock folders and a mocked Windows API.

## SKR Pico over UART (`skr-pico-uart`)

Same as `skr-pico` but Klipper talks on UART0 (GPIO0 TX / GPIO1 RX) at 250000 baud instead of USB,
as in BTT's own Raspberry Pi UART instructions. Used for the ESP32-S3 link; wiring in
[WIRING_SKR_PICO.md](WIRING_SKR_PICO.md). Compile-only. After flashing it the board no longer
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
python3 host/tools/skipper_fw.py build skr-pico [--no-segq] [--menuconfig] [--jobs N]
python3 host/tools/skipper_fw.py flash skr-pico [--artifact DIR] [--method M] [--port P] [--mount PATH] [--dry-run] [--yes]
python3 host/tools/skipper_fw.py overlay --apply|--remove [--klipper-dir DIR]
```

Toolchains (Linux Mint/Ubuntu): `printhost.py` offers them as an optional step
(`--no-firmware` skips it), or: `sudo apt install git gcc-arm-none-eabi
binutils-arm-none-eabi libnewlib-arm-none-eabi gcc-avr avr-libc binutils-avr
dfu-util avrdude stm32flash` (roughly 140 MB download).

Each build goes to `~/.skipper-flipper/out/<board>-<YYYYmmdd-HHMM>/` with the
firmware file(s), `klipper.elf`, `klipper.dict`, `klipper.config` (the final
resolved config) and `build-info.json` (Klipper tag + commit, preset, segq on/off,
tool version, flash/RAM size, sha256 of every artifact). Klipper's own `make
olddefconfig` expands the preset `.config` seed (verified in v0.13.0's Makefile:
the `.config` rule runs `lib/kconfiglib/olddefconfig.py`), then `make clean` and
`make` run, so toggling segq never leaves stale objects.

## Presets and verification status

All presets: **status compile-only**. Sizes are from the sandbox build (Klipper
v0.13.0, Ubuntu 24.04 gcc-arm-none-eabi / gcc-avr). "RAM" is static data+bss only;
the segment queue is allocated at run time (`depth x 24` bytes ARM, `x 23` AVR).

| Board id | Arch | Flash method | DDA / max steps/s per axis | Flash B, stock | Flash B, segq (delta) | Static RAM B |
|---|---|---|---|---|---|---|
| `creality-4.2.x` | stm32 | sdcard-copy | 25000 Hz / 12500 | 33383 | 35364 (+1981) | 1144 |
| `ender3-1.1.x` | avr | avrdude | 8000 Hz / 4000 | 35500 | 38982 (+3482) | 417 |
| `generic-rp2040` | rp2040 | uf2-copy | 40000 Hz / 20000 | 39264 | 40184 (+920) | 1544 |
| `ramps-mega2560` | avr | avrdude | 10000 Hz / 5000 | 36272 | 39774 (+3502) | 418 |
| `skr-pico` | rp2040 | uf2-copy | 40000 Hz / 20000 | 39264 | 40184 (+920) | 1544 |

- **skr-pico**, **generic-rp2040**: RP2040, USB serial, no bootloader, W25Q080
  flash chip (Klipper defaults; same header comments as Klipper's SKR Pico config).
  Unverified: that your board's flash chip is compatible with the W25Q080 setting.
- **creality-4.2.x**: STM32F103, 28 KiB bootloader offset, 8 MHz crystal, USART1
  PA10/PA9 at 250000 baud (from the header of Klipper's `generic-creality-v4.2.7.cfg`).
  Unverified: clone chips (GD32F103 etc.) and other bootloaders.
- **ramps-mega2560**: ATmega2560 at 16 MHz, 250000 baud, avrdude protocol `wiring`.
- **ender3-1.1.x**: ATmega1284P at 16 MHz, 250000 baud, avrdude `arduino` at 57600
  baud (from a comment in Klipper's `printer-creality-ender3-2018.cfg`). Boards
  without a bootloader need an ISP programmer and are not covered.

Every option name in the `.config` seeds was checked against Klipper v0.13.0's
`src/*/Kconfig`, and the resolved `.config` of each build was inspected (MCU,
clock, flash offset, serial settings, `CONFIG_WANT_SEGQ`). The built
`klipper.dict` of every segq build contains the five commands and two responses.
That only shows the config is accepted and the code compiles, not that it works.

## dda_hz and top speed

`config_segq period` is `CLOCK_FREQ / dda_hz` MCU clock ticks. Because a segment
must satisfy `2*|steps| <= n`, an axis can step at most once every two DDA ticks:

    max step rate per axis = dda_hz / 2
    max speed (mm/s)       = dda_hz / 2 / steps_per_mm

e.g. 40 kHz on an RP2040 (12 MHz Klipper clock, period 300) gives 20,000 steps/s,
250 mm/s at 80 steps/mm; 10 kHz on the AVR gives 5,000 steps/s (62 mm/s). The
recommended values are deliberately conservative guesses (the ISR cost, especially
on 8-bit AVR with 32-bit per-axis accumulators, has not been measured); each preset
`.json` explains its number. If the board shuts down with "segq DDA timer too
slow", lower `dda_hz`.

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

- `python3 host/tools/test_skipper_fw.py`: the tooling (presets, overlay, mounts,
  flash plans/refusals, artifacts).
- `python3 host/tools/test_fake_mcu.py`: protocol v1 against the simulator.
- `firmware/test/run.sh`: compiles the real `segq.c` glue + core on a PC against a
  small mock of the Klipper API and drives the DDA timer in simulated time (pulse
  width, dir pins, inversion, idle stop/restart, underruns, start threshold, stop,
  error shutdowns, late-timer shutdown). It proves the glue logic only; the mock
  is not Klipper and has no real interrupts.

## Not done / open items

- Nothing has run on hardware (see the banner above).
- No sync pin actions and no DDA endstop homing in v1 (PROTOCOL.md).
- DDA ISR cost on AVR is unmeasured; the AVR presets use a conservative 8-10 kHz.
- Bootloader/flash details for individual boards are taken from Klipper's own
  comments; revisions in the wild differ.
- No STM32 USB-DFU, dfu-util or stm32flash flashing method yet (the tools are
  installed by `printhost.py` for later presets).
