# CAPTAIN test build: INT32/64 (whole-number motion maths)

**Not for everyday printing.** The same as the standard CAPTAIN build, except that input shaping,
pressure advance and step times are worked out in whole numbers (int32 / int64) instead of `double`
(which the ESP32-S3 can only do in software). For comparing the two with the print benchmark
(Tune page: Print benchmark, or `SF_BENCH`; see docs/BENCHMARK.md). Nothing moves or heats during it.

| File | What it is | Use it with |
|---|---|---|
| `CAPTAIN-INT32-TEST-full-image-usb.bin` | the test build, full image | the Flasher over USB, "own file" (it sets offset 0x0 by itself) |
| `CAPTAIN-INT32-TEST-wifi-update.bin` | the test build, app only | the **Update page over WiFi** only |
| `CAPTAIN-STANDARD-wifi-update.bin` | the standard CAPTAIN build, app only | the **Update page over WiFi**, to go back |

Over USB, always use a **full image** (`...full-image-usb.bin`, or `../skipper-esp32s3-captain-merged.bin`
for the standard build). An app-only file over USB at the wrong place leaves the ESP32 not starting
(it happened on 2026-10-28; re-flashing a full image fixes it, and the Flasher now refuses it).

**Which build is running?** Log page, "Important events": the first line says
`STANDARD build (double motion maths)` or `INT32/64 TEST BUILD (whole-number motion maths)`, with the
build date and a short fingerprint of the binary. Settings and printer.cfg are kept when switching.

SHA-256 sums: `SHA256SUMS.txt`.
