# Pasta Inspector dongle: ending a failed print

*2026-10-31. Engine side tested on the PC (1,862 checks, both maths). The ESP32 websocket server is
**written but not compiled with ESP-IDF and not tried with a real dongle or printer.** CAPTAIN and
SKIPPER only: DECKHAND has no room for it (and no exclude object).*

An ESP32-CAM running **Pasta Inspector** watches the bed. When it sees spaghetti it talks to the
printer the way it talks to Klipper's Moonraker, so Skipper Flipper answers the same way. Skipper
Flipper's address and port go into the dongle's printer settings; no other change is needed.

```
ESP32-CAM dongle ──WiFi──► ws://<skipper-flipper>/websocket ──► Machine (exclude object, pause, M112)
```

## What the dongle can do

| It sends | What happens |
|---|---|
| `EXCLUDE_OBJECT NAME=CUBE_1` | That part's moves are skipped from now on (G-code position still advances); the rest of the plate carries on. Look-ahead is flushed, so it is not instant. |
| `PAUSE` | The print pauses (same look-ahead delay). |
| `M112` / `printer.emergency_stop` | Emergency stop: heaters off, board shut down. Needs a restart to recover. |
| `RESPOND MSG="..."` | Logged to the console (`RESPOND PREFIX=... MSG=...` also works). |

It reads: `print_stats` (state, filename, filament used, layer), `toolhead` (position, homed axes),
`motion_report` (live position), `pause_resume`, `idle_timeout`, `virtual_sdcard` (progress),
`display_status`, `extruder` and `exclude_object` (the part list with centre, polygon and `PI_Z0` /
`PI_Z1`, the excluded parts, the current part). Fields the engine cannot know are left out, not invented.

Websocket methods answered: `server.connection.identify`, `printer.objects.list`,
`printer.objects.subscribe` (then `notify_status_update` about five times a second),
`printer.gcode.script`, `printer.emergency_stop`. Anything else: JSON-RPC error -32601.

## Set-up

1. Flash a CAPTAIN or SKIPPER build (it turns on `CONFIG_HTTPD_WS_SUPPORT`).
2. The slicer must write each part's outline: `EXCLUDE_OBJECT_DEFINE NAME=... POLYGON=[[x,y],...]`
   (PrusaSlicer / OrcaSlicer / SuperSlicer, "Label objects": Klipper). Without outlines the dongle will
   not judge a print. Up to 64 parts and 512 outline numbers per part.
3. Put the dongle on the same WiFi as the printer host, give it Skipper Flipper's IP and the HTTP port
   (80 by default), and enable it. Mounting and lighting: Pasta Inspector's own README.

## Limits to know about

- **Not tested on real hardware.** The same Moonraker engine code runs in SkiPiPir on the PC only.
- Two websocket clients at most; a frame over 3 KB is refused.
- The parts list resets only when a new print starts (as in Klipper), not on cancel or finish.
- An excluded part's moves are skipped; heat and extrusion already queued are not undone.
- A command sent by the dongle waits up to 5 s for the engine's answer on the web server's thread.
- Licences: Pasta Inspector is PolyForm Noncommercial; Skipper Flipper is GPL-3.0. They are separate
  programs that only talk over the network, and this code was written from the protocol, not from the
  dongle's source. Do not copy files from one project into the other.

## Where it lives

`host/src/machine_moon.cpp` (status objects), `host/src/machine_pro.cpp` (EXCLUDE_OBJECT_DEFINE with
polygons), `host/src/machine.cpp` (RESPOND, object-list revision), `host/src/toolhead.cpp` (live
position), `firmware/esp32s3/main/moonraker_ws.cpp` (the `/websocket` server), `main/link.cpp`
(status refresh, only while a dongle is subscribed). Tests: `host/test/test_toolhead.cpp`.
