<!-- Brought into Skipper Flipper on 2026-10-23 from the KlipperTFT OLED workbench (KlipperTFT-Pico 0.6.1, same author). -->
# Design notes: the little OLED (from KlipperTFT-Pico, 2026-10-23)

- **Request.** Cheap 0.9x" OLEDs (about £3): hotend and bed temperatures as big as possible when
  nothing is printing; percentage and a big progress bar while printing; while extruding or
  retracting, the temperatures with the status flashing up for 2 s every 4 s.
- **Which modules.** The user's Amazon link could not be opened (the site blocks automated reads),
  so the common 4-pin I2C modules are all handled: SSD1306/SSD1315 128x64 (0.96", including the
  yellow/blue two-colour ones), SSD1306 128x32 (0.91") and SH1106 128x64 (1.3"). Auto mode probes
  0x3C then 0x3D and assumes SSD1306 128x64; the others are chosen in settings.
- **Controller differences.** Page addressing mode (0x20 0x02) works on both families. SH1106 has a
  132-column RAM with the picture starting at column 2, so its frames are sent from column 2.
  Upside-down mounting is done in software (rotate the framebuffer) because the SH1106 column offset
  makes the hardware flip unreliable across modules.
- **Fonts.** Bitmap fonts rendered from DejaVu (free, Bitstream Vera licence) with Pillow:
  DejaVu Sans Bold 9 px for small text (chosen after comparing seven candidates side by side;
  condensed and mono versions merged "i" and "l"), DejaVu Sans Condensed Bold for 17/27/40 px rows.
  Glyphs are stored column-wise, top bit first, the same order the controllers use.
- **Layouts.** 128x64: two 32 px rows (label and target on the left, number right-aligned with a
  drawn degree ring, heating/cooling arrow); printing: 40 px percentage + 20 px bar; paused alternates
  with "PAUSED"; activity: a full inverted panel with the word, 2 s on / 2 s off. 128x32: hotend and
  bed side by side with a one-letter label; percentage left, bar right.
- **Timing.** 4-5 frames a second. A full frame is ~1 KB, about 25 ms of I2C at 400 kHz. On the
  Pico (single loop) it is sent one 128-byte page per service call so USB is never held up for
  more than ~3 ms; on the ESP32 it runs in its own low-priority task.
- **Pins in KlipperTFT.** Pico GP6/GP7 (I2C1), ESP32 21/22, ESP32-S3 8/9, ESP32-C3 SuperMini 8/9
  (the usual SuperMini OLED pins; both are strapping pins that must be high at boot, which the
  display's pull-ups give; the board's LED status is turned off when a display is set).
- **Verification.** Unit tests on the PC (frame maths, text fitting, idle vs flash timing, progress
  bar fill proportional to progress, 128x32 rendering, text wrapping, init sequence); the screen
  pictures are rendered by the same code; the component builds on its own with ESP-IDF 5.4 for the
  ESP32-S3. Not yet run on a real display.

## Integration into Skipper Flipper (2026-10-23)

- **Licence.** KlipperTFT-Pico is PolyForm Noncommercial; Skipper Flipper is GPL-3.0, and the two
  cannot be mixed in one project. The author owns both and chose GPL-3.0 for the copy in this repository
  (headers say so); KlipperTFT-Pico keeps its own licence for its copy. The fonts keep the
  Bitstream Vera / DejaVu licence (`FONT-LICENSE.txt`).
- **Where it lives.** `firmware/esp32s3/components/ktft_oled/` (component, PC tests, preview and font
  tools moved inside it; their paths adjusted). Unchanged drawing code; headers only.
- **Where the data comes from.** KlipperTFT asks Moonraker; Skipper Flipper *is* the host, so the link
  task publishes a small snapshot (`sf_display_t`) twice a second, taken from a new
  `ph::Machine::display_info()` (heaters, print state, and what is going on: HEATING, HOMING, and
  extruder-only moves outside a print as EXTRUDING / RETRACTING / LOADING / UNLOADING). That part is
  tested on the PC with the simulated board.
- **Screens added for Skipper Flipper** (no new layout code, only the existing "title + three lines"
  screen with Skipper's words): NO BOARD (reason, WiFi name, password / address alternating every
  4 s), CHECK CFG.
- **Pins.** SDA GPIO8, SCL GPIO9, as in KlipperTFT on the S3; checked against the UART (43/44), LED
  (21) and SD card (10-13) defaults; the Settings page refuses clashes.
- **Memory.** DECKHAND app space went from 5% to 2% free (about +45 KB with the I2C driver); `.bss`
  from 75.0 to 77.7 KB. The task only stays alive when a display answers.
- **Checked.** `oled_tests` and `ph_tests` on the PC; the three firmware builds compile; DECKHAND under
  QEMU starts the task, probes an empty bus three times and stops looking (QEMU then stops at WiFi
  start as before). Fonts regenerate byte-identical with `make_oled_fonts.py`. Not run on a real
  display.
