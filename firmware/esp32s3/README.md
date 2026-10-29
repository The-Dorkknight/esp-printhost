# ESP32-S3 firmware - WORK IN PROGRESS

> **Work in progress: mostly working, needs tuning and proper testing.** Built with ESP-IDF v5.4.1. Earlier builds run in a WEEDO Tina 2 with a BTT SKR Pico (see `docs/HARDWARE_LOG.md`): WiFi pages, Klipper link, heating with PID and safety checks, motor tests, TMC2209, stallGuard and the hotend fan work on hardware. The other fans are not confirmed yet (possibly a broken wire). Moves, homing and printing are simulated only. Micro SD card support (`sdcard.c`, see `docs/SD_CARD.md`) works on hardware (mount, list, upload). The little I2C OLED (`oled.cpp` + `components/ktft_oled`, see `docs/OLED.md`, 2026-10-23) is compiled and PC-tested only. The tuning features (2026-10-24, CAPTAIN and SKIPPER: `pro.cpp`, `accel.cpp`, `usb_accel.cpp`, `web_pro.cpp`, the Tune page; see `docs/TUNING.md`) are compiled, PC-tested and QEMU-checked only; `usb_accel.cpp` has never run.

What it does: starts a WPA2 WiFi access point named `SkipperFlipper-XXXX` (last two bytes of its
MAC; default password `skipperflipper`) with a status page at `http://192.168.4.1/`, `/status.json`,
a live log at `/console`, `/settings` and `/config` (`printer.cfg`). Over UART it talks Klipper to a
printer board (see `docs/WIRING_SKR_PICO.md`): identify, clock sync, configure the board from the
sections of `printer.cfg` (LEDs, thermistors, heaters with Klipper's safety checks, fans, TMC2209
drivers, steppers for motor tests), and has a G-code console and a test bench at `/test`
(see `docs/TEST_BENCH.md`). Heating, motor tests, TMC2209 and stallGuard have run in a WEEDO Tina 2; moves, homing, probing and printing an
uploaded file (Printer page at `/printer`) are **simulated only** so far.

Three builds by board memory (DECKHAND, CAPTAIN, SKIPPER; `tiers/`, `main/tier.c`, docs/TIERS.md).
Flash with the Flasher (Master tab, **Flash for my board...** picks the build), or build them:

```
. ~/esp/esp-idf/export.sh      # ESP-IDF v5.4.x
./build.sh                     # build all three, merge, store in ../prebuilt-esp32s3/
idf.py -B build-captain -DSF_TIER=captain build    # one build by hand
```

Settings (`idf.py menuconfig` > Skipper Flipper): AP password (8 to 63 characters), channel,
LED type and GPIO (default WS2812 on GPIO21, the Waveshare ESP32-S3-Zero). The UART pins, baud and status LED can also be changed at run time on the `/settings` page; the printer board's LEDs are set in `printer.cfg` at `/config`. The console is on the chip's native USB port.
