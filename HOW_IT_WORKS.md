# How it works

> WORK IN PROGRESS: MANY FEATURES MAY BE BROKEN (proof of concept, 0.1.0-wip)

## The idea in plain English

3D printer boards talk to a computer using Klipper's MCU protocol: short framed messages with a checksum. This project re-implements the computer's side of that conversation in portable C++, so the same code can later run on a cheap ESP32 instead of a PC or Raspberry Pi. The tools in this repo prove that the C++ code can talk to a real board.

## What the tools read and change

**`sudo python3 printhost.py`** (only this changes your computer):

- installs missing packages with `apt`: `build-essential`, `cmake`, `zlib1g-dev`, `python3-tk`, `acl`
- adds you to the `dialout` group (so you can use serial ports; fully effective after next login)
- writes `/etc/udev/rules.d/99-esp-printhost.rules` (lets the logged-in user use USB serial ports and tells ModemManager to ignore them), and grants your user access to ports plugged in right now with `setfacl`
- if `brltty` is installed, and you agree, writes an empty `/etc/udev/rules.d/85-brltty.rules` so it stops grabbing CH340 serial chips (undo: `sudo rm /etc/udev/rules.d/85-brltty.rules`)
- builds in `host/build/` as your normal user and writes logs to `~/esp-printhost-logs/`

`python3 printhost.py --check` only reports and changes nothing.

**`mcu_probe`** opens one serial port, asks the board to identify itself, reads its pin list and uptime. With `--blink` it configures one pin as an output and toggles it 10 times. If the board was already configured it first sends Klipper's `reset` command, which reboots it.

**`fake_mcu.py`** is a pretend board on a pseudo-terminal. It touches nothing real.

## What it never does

- never flashes firmware
- never drives steppers or heaters, and never reads temperature sensors
- never sends data over the network
- never deletes your files

## Architecture (short version)

```
G-code (SD card / web upload)
        |
[ESP32-S3 host]   WiFi, web UI, config, planner        <- NOT BUILT YET
        |  serial / USB, Klipper-style framing
[Printer board]   Klipper MCU firmware fork + segment queue   <- NOT BUILT YET
        |
steppers, heaters, fans, endstops
```

The part in this repo is the **protocol and config library** (`host/`) plus PC tools. Details: [docs/DESIGN.md](docs/DESIGN.md). Wire format and the draft segment-queue commands: [docs/PROTOCOL.md](docs/PROTOCOL.md).

## Message framing

```
<len> <seq> <messages...> <crc16 hi> <crc16 lo> <0x7E>
```

Integers inside messages use Klipper's variable-length encoding (small numbers take one byte). On connect the board sends a compressed JSON "data dictionary" listing every command it understands, plus enumerations such as pin names; `mcu_probe` decodes that to build its commands. Protocol knowledge comes from Klipper's open source `msgproto` (see [THIRD_PARTY.md](THIRD_PARTY.md)).
