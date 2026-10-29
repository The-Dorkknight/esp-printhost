# Three builds: DECKHAND, CAPTAIN, SKIPPER

**Status (2026-10-20): CAPTAIN runs on the ESP32-S3-Zero type board** (the Flasher picked it; 4 MB flash
and 2 MB PSRAM found and in use, 256 moves of look-ahead). DECKHAND and SKIPPER build and boot under QEMU. Each boots under
QEMU (Espressif's ESP32-S3 emulator) up to the point where WiFi starts (QEMU has no WiFi, so it stops
there), including the fall-back when PSRAM is missing. The new host-side code (look-ahead sizing,
slicer upload parsing, thumbnails, layer/time display) is unit-tested on the PC. Everything that
talks to real hardware (PSRAM, WiFi updates, home WiFi, the SD extras) is compile-only until it runs on
a board.

One source, three builds, picked by how much memory the ESP32-S3 has. The names are a joke; what they
do differently is not.

| Build | Needs | What it adds |
|---|---|---|
| **DECKHAND Edition** | any ESP32-S3 (2 MB flash) | everything in the project: Klipper link, heaters, fans, TMC2209, tests, moves, homing, printing from flash or SD card, plus the extras below |
| **CAPTAIN Edition PRO MAX Turbo Ultra** | 4 MB flash **and** PSRAM (e.g. ESP32-S3FH4R2, as on the Waveshare ESP32-S3-Zero and most SuperMini boards) | firmware updates over WiFi with roll-back; 256 moves of look-ahead (was 32); 64 KB of console history |
| **SKIPPER Edition PRO MAX Turbo Ultra Mega Elite Performance Gamer MAXX (RGB)** | 8 MB flash **and** 8 MB octal PSRAM (ESP32-S3R8: WROOM-1 N8R8 / N16R8 modules) | everything CAPTAIN has, 512 moves of look-ahead, 256 KB of console history, bigger slicer thumbnails, and a rainbow on the status LED at start-up |

In every build ("the extras"):
- **Send from the slicer.** PrusaSlicer / OrcaSlicer: add a physical printer of type **OctoPrint**,
  host `http://192.168.4.1` (or the ESP32's home-WiFi address), any API key. "Upload" puts the file on
  the SD card, "Upload and print" also starts it. (The ESP32 answers `GET /api/version` and
  `POST /api/files/local` like OctoPrint; nothing else of OctoPrint is there.)
- **Join your home WiFi** (Settings page), as well as its own network SkipperFlipper-XXXX, which stays
  on. Once joined, its own network moves to the router's channel (one radio).
- **Slicer thumbnails** in the Printer page's file list and next to the progress bar.
- **Print history** (`skipper/history.csv` on the card, "Recent prints" on the Printer page).
- **Event log** on the card (`skipper/events.log`): everything the firmware logs, with the time once a
  browser has opened a page (the ESP32 takes the clock from it).
- **printer.cfg backups**: each Save keeps the version it replaces (`skipper/cfg/`, the last 10),
  listed on the printer.cfg page.
- **Crash reports**: after a crash, brownout or watchdog restart the status page says what happened
  ("Stack canary watchpoint triggered (led) in task led", ...) with a link to `/crash`: the panic
  reason, the crashed task, the backtrace, and the last 2 KB the firmware logged before it. With a card
  in, also saved as `skipper/crash-NNNN.txt`. The raw core dump is at `/coredump.bin`.
- **Little OLED display** (2026-10-23, [OLED.md](OLED.md)): temperatures, print progress and what the
  printer is doing on a cheap I2C OLED. Not yet tried on a real display.
- **Memory report**: `/about` shows the build, the flash and PSRAM found, what is on and off, and
  whether this board could run a bigger build. The heartbeat in the console now also prints the lowest
  free memory since start-up, so a slow leak shows as a number that keeps going down.

## Which one does my board need?

The Flasher works it out: Master tab, **Flash for my board...**. It reads the chip with esptool
(read-only: `flash-id`), looks at the "Detected flash size" and the "Embedded PSRAM" in its Features
line, picks the build and flashes it at 0x0 after one confirmation. From a terminal:

```
python3 host/tools/skipper_esp.py flash-auto --port /dev/ttyACM0 --dry-run     # shows what it would do
python3 host/tools/skipper_esp.py flash-auto --port /dev/ttyACM0               # asks, then flashes
python3 host/tools/skipper_esp.py flash-auto --port /dev/ttyACM0 --tier deckhand   # force a build
```

The rule (the same in the Flasher, `choose_tier()`, and on the ESP32's `/about` page, `main/tier.c`):

- 8 MB flash or more and 8 MB PSRAM or more: SKIPPER (8 MB+ PSRAM on the ESP32-S3 is octal)
- 4 MB flash or more and PSRAM: CAPTAIN
- anything else, or nothing reported: DECKHAND, which boots on every ESP32-S3

Why CAPTAIN wants PSRAM and not just 4 MB of flash: its PSRAM support takes about 8 KB of the chip's
internal RAM even when no PSRAM is found, and a board without PSRAM is better off keeping that RAM
(the first Tina 2 crash was the ESP32 running out of internal RAM).

A build meant for a bigger board still starts on a smaller one as long as the flash is big enough: the
PSRAM is set to "ignore if not found", so CAPTAIN on a 4 MB board without PSRAM runs with DECKHAND's
buffers (QEMU confirms this). A build for more flash than the chip has will not boot; the Flasher's
check is there to stop that.

## Changing build, and what is kept

All three keep the settings (WiFi, pins, SD card) in the same place, so they survive a change of
build, and `printer.cfg` lives there too. The file kept in the ESP32's own flash (the "flash" row of
the Files list) does not survive a change of build: upload it again, or use the SD card.

## Firmware updates over WiFi (CAPTAIN, SKIPPER)

Settings page, **Firmware update** (`/update`). Pick `skipper-esp32s3-<build>-app.bin` from
`firmware/prebuilt-esp32s3/` (the `-app` file, not the merged one). Only an image of the same build
is accepted: changing build changes the partition table and the PSRAM set-up, which only USB flashing
can do. The new firmware goes into the second slot, is checked, and the ESP32 restarts into it. It must
then run for 30 s to be kept; if it crashes before that, the ESP32 goes back to the previous one by
itself. Refused while a print runs.

## Memory, in numbers (2026-10-28 builds, with the OLED, tuning, buttons and the benchmark)

| | DECKHAND | CAPTAIN | SKIPPER |
|---|---|---|---|
| Flash used for the app | 1.37 MB of 1.38 MB (8 KB free: full; no tuning features, buttons or benchmark; error codes shown as numbers) | 1.61 MB of 1.63 MB (16 KB free; the INT32/64 test build 10 KB) | 1.61 MB of 3 MB (46% free) |
| Input shaping, pressure advance, accelerometer, bed mesh, screw tilt, arcs, G10/G11, exclude object, Tune page ([TUNING.md](TUNING.md)) | no | yes | yes |
| Buttons on the case ([BUTTONS.md](BUTTONS.md)) | no | yes | yes |
| Print benchmark `SF_BENCH` ([BENCHMARK.md](BENCHMARK.md)) (2026-10-28) | no | yes | yes |
| Important-events log (Log page) (2026-10-28) | yes | yes | yes |
| CoreXY, probe Z homing, gear_ratio (2026-10-27) | yes | yes | yes |
| Chamber thermistor on the ESP32 ([CHAMBER_SENSOR.md](CHAMBER_SENSOR.md)) (2026-10-27) | no | yes | yes |
| CPU 240 MHz / bigger caches (2026-10-27) | 240 MHz / default caches | 240 MHz / 32 KB I + 64 KB D | 240 MHz / 32 KB I + 64 KB D |
| Flash G-code store (no card) | 484 KB | 468 KB | 1.71 MB |
| Largest printer.cfg (2026-10-27) | 8 KB | 16 KB | 16 KB |
| Core dump partition | 64 KB | 128 KB | 128 KB |
| Look-ahead (with PSRAM) | 32 | 256 | 512 |

DECKHAND's static RAM is 77.7 KB of `.bss` (75 KB before the OLED); the big
buffers of the new pages are taken from the heap only while a request runs, so the printer link keeps
its memory.

## For maintainers

`firmware/esp32s3/build.sh` builds all three (`build/`, `build-captain/`, `build-skipper/`) and stores
them with `skipper_esp.py bundle-tiers`: the merged images, the `-app.bin` files for WiFi updates, and
`symbols/skipper-esp32s3-<build>.elf` (the ELF without debug sections, about 2 MB, for decoding a crash
report's backtrace: `xtensa-esp32s3-elf-addr2line -pfiaC -e symbols/skipper-esp32s3-captain.elf
0x4200f3a2 ...`). By hand: `idf.py -B build-captain -DSF_TIER=captain build`. The tier settings are in
`firmware/esp32s3/tiers/`.

Int32/int64 motion test build (CAPTAIN, 2026-10-28; [BENCHMARK.md](BENCHMARK.md)):
`idf.py -B build-captain-int -DSF_TIER=captain -DSF_FIXED=1 build`; shipped in `firmware/prebuilt-esp32s3/test-int32/` (see BENCHMARK.md for which file is which).

QEMU test build (console on UART0, SD card off; since 2026-10-28 also `CONFIG_SKIPPER_HOST_ADC=n` and
`CONFIG_FREERTOS_UNICORE=y`, see BENCHMARK.md): `-DSF_EXTRA_DEFAULTS=<file>` with
`CONFIG_ESP_CONSOLE_UART_DEFAULT=y` and `CONFIG_SKIPPER_SD_DEFAULT=n`, then
`qemu-system-xtensa -M esp32s3 -nographic -drive file=<merged image padded to the flash size>,if=mtd,format=raw`.
