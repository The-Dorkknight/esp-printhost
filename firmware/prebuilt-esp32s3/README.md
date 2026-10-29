# Which ESP32-S3 file?

| You want to | Use |
|---|---|
| Flash over **USB** (Flasher), first time or to change build | a **full image**: `skipper-esp32s3-merged.bin` (DECKHAND), `skipper-esp32s3-captain-merged.bin` (CAPTAIN), `skipper-esp32s3-skipper-merged.bin` (SKIPPER). The Flasher's "Flash for my board" picks the right one. |
| Update over **WiFi** (the board's Update page) | the **app-only** file of the same build: `skipper-esp32s3-captain-app.bin` or `skipper-esp32s3-skipper-app.bin` (DECKHAND has no WiFi update) |
| Try the CAPTAIN **INT32/64 test build** | `test-int32/` (its README says which file is which) |

`build-info.json` holds the SHA-256 of every file; the Flasher checks it. `symbols/` is for decoding
crash reports.
