# Wiring: ESP32-S3 to a BTT SKR Pico over UART (no soldering)

> **Works for the read-only link** (2026-10-11, see [HARDWARE_LOG.md](HARDWARE_LOG.md)): the ESP32-S3
> identifies the SKR Pico and checks it every 5 s. Nothing configures, moves or heats yet.

## Why UART and not USB

A USB link needs the ESP32-S3 to act as a USB **host** and to supply 5 V to the board. Small
ESP32-S3 boards have no power-delivery controller on their USB-C port, and the firmware is not a
USB host (a try with a USB-C PD hub did nothing, as expected). It could be built with ESP-IDF's
USB host library and Espressif's CDC-ACM driver (and a CH34x driver for CH340-based boards), plus a
powered hub, but four jumper wires are simpler and leave the ESP32-S3's USB-C port free for
flashing and its console. The USB route is parked.

## What to flash

1. **SKR Pico:** in the Flasher, Slave tab, board **BTT SKR Pico (RP2040, serial UART0, for the
   ESP32-S3)** (`skr-pico-uart`). This firmware talks on UART0 (GPIO0 TX, GPIO1 RX) at 250000 baud
   and **no longer over USB**. To go back to USB, flash the plain `skr-pico` preset (BOOTSEL + copy).
2. **ESP32-S3:** Master tab, **Use shipped firmware**, Flash.

## Wires (4)

Tested board for the ESP32-S3 side: **Waveshare ESP32-S3-Zero** (and look-alikes). Per Waveshare's
wiki: TX is GPIO43, RX is GPIO44 (the pins marked TX/RX), the colour LED is a WS2812 on GPIO21,
and the board can be powered from its 5V pin (3.7 to 6 V, at least 500 mA).

| SKR Pico (UART / Pi header, see BTT's pinout) | ESP32-S3-Zero |
|---|---|
| TX0 (RP2040 GPIO0) | **RX: GPIO44** (default) |
| RX0 (RP2040 GPIO1) | **TX: GPIO43** (default) |
| GND | GND |
| +5V | 5V pin |

TX always goes to RX. Both chips use 3.3 V logic, so no level shifter is needed. BTT's own guide
powers a Raspberry Pi from the board's +5V on this wiring, so the SKR Pico (on its printer power
supply) can power the ESP32-S3 the same way.

**Do not power the ESP32-S3 from USB and the 5V wire at the same time** unless you have checked
that the board has a diode between them. Unplug the 5V wire while flashing the ESP32-S3 over USB
(GND, TX and RX can stay connected).

GPIO43/44 are the pins usually labelled TX/RX on small ESP32-S3 boards. The ESP32-S3's built-in
boot ROM prints a few lines on GPIO43 at power-up; the SKR Pico's Klipper ignores that noise. Other
pins can be chosen on the **Settings** page (`http://192.168.4.1/settings`).

## What you should see

- Join WiFi `SkipperFlipper-XXXX`, open `http://192.168.4.1/`. The **Printer board** row shows
  `connected: Klipper v0.13.0...` with the MCU `rp2040` once the link works, or the reason it
  does not (`no identify_response` = check the wires, TX/RX swap, and that the SKR Pico has the
  `skr-pico-uart` firmware).
- `http://192.168.4.1/console` shows each attempt and, on success, the board's pin names.
- The status light (WS2812 on GPIO21 by default, right for the ESP32-S3-Zero): slow blink / amber
  = searching, double blink / green = connected, fast blink / red = error, short blue blink every
  2 s = link switched off on the settings page.
- Phones: joining `SkipperFlipper-XXXX` should pop up a "Sign in to network" notice that opens the
  status page (the firmware answers all name lookups itself, like a hotel WiFi login page). There
  is no internet on this network; Android may say so, which is expected.
- If something goes wrong, the Flasher's Master tab **USB log** button restarts the chip and shows
  its messages, including the reason for the previous restart. The status page also shows
  **Last restart** (a "CRASH" there is a bug worth reporting).

After identifying the board, the ESP32-S3 configures it the way Klipper does, from `printer.cfg`
(edit it at `http://192.168.4.1/config`). The default has one object: the SKR Pico's on-board RGB
LED, as in Klipper's own SKR Pico config:

```
[neopixel board_led]
pin: gpio24
chain_count: 1
color_order: GRB
initial_GREEN: 0.15
```

The status page then shows "Board state: configured", **printer.cfg: configured**, an **LED
board_led** row with red / green / blue / white / off buttons, and a **Console** where you can type
`SET_LED LED=board_led RED=1 GREEN=0 BLUE=0.5` (or `HELP`). The **Board clock** row shows the clock
the ESP32-S3 measures for the board, its drift from the nominal value and the best round trip. If
the board was configured differently before, the ESP32-S3 resets it once and configures it again
(you will see it restart). This LED step on its own moves and heats nothing; heaters, fans and motors come next.

**Temperatures.** The default `printer.cfg` also reads the SKR Pico's thermistor inputs, hotend
(`TH0`, `gpio27`) and bed (`TB`, `gpio26`), each with the board's 4.7k pullup, assuming a 100k
3950 thermistor (Klipper's `Generic 3950`). They appear as **Temp hotend** and **Temp bed** rows on
the status page (and in `status.json` and `M105`). A reading of "no sensor? (reads open circuit)"
means nothing is connected to that input (or the pin is wrong); "short circuit?" means the input
reads zero. If your thermistor is a different type the number will be wrong: change `sensor_type`
in `printer.cfg` (`ATC Semitec 104GT-2`, `EPCOS 100K B57560G104F`, `NTC 100K MGB18-104F39050L32`)
or add a `[thermistor NAME]` section with `temperature1`/`resistance1` and `beta`, or three
temperature/resistance pairs, as in Klipper. Saving a new `printer.cfg` makes the board restart once.
Check the pin names against your board's pinout; the pins above are from memory of BTT's own
Klipper config, not yet verified by this project.

## When it says "no identify_response"

First check, because it is easy to miss: **is the SKR Pico still in its bootloader?** If the BOOT
jumper (or the BOOTSEL button) was on when it was last plugged in, the board starts as the
`RPI-RP2` drive and runs no firmware, so nothing at all comes back (UART bytes: received 0).
Remove the jumper and unplug / replug its USB. Also check you flashed `skr-pico-uart`, not
`skr-pico`: the USB version never talks on the UART. The ESP32 has the same trap: if its BOOT
button or GPIO0 wire is still held, it goes back into its bootloader instead of running.

The status page shows what actually crossed the wires:

- **UART bytes: received 0** means nothing at all is coming back. Check the board's TX goes to the
  ESP32 RX pin, GND is shared, the board is powered, and it runs the `skr-pico-uart` firmware (it
  must no longer appear as a USB serial device on a PC). Try swapping the TX and RX wires.
- **Bytes received but no reply** means a signal arrives but not as Klipper replies: check the baud
  (250000) and GND.
- **Only 00 bytes** (or "the board's TX line is low") means the board's TX wire is held low: the
  board is switched off, or it is hung. Switch the printer supply off and unplug the board's USB for
  10 seconds. See "Switching the board off with the ESP32 running" below.
- **Self-test of the ESP32 side:** disconnect the board, then join the ESP32's TX and RX pins
  (GPIO43 and GPIO44 by default) with a single wire. Within about 5 seconds the page should say it
  hears its own transmission. If it does, the ESP32 side is fine and the problem is on the board
  side or in the wiring.
- **"garbled copy of the ESP32's own requests"**: the bytes coming back start with the same
  length and sequence number as the ESP32's own request frames (seen in testing as
  `08 14 00 28 ...`). The ESP32's TX signal is leaking into its RX wire and the board is not
  answering. Check with a multimeter that the TX and RX wires are not shorted, that the ESP32 RX
  goes to the board's **TX0** (not RX0), and replace damaged or salvaged cable.

## Heater, fans and motors (for the test bench)

The default `printer.cfg` uses Klipper's SKR Pico pins: hotend heater on the HE (gpio23) output with
its thermistor on TH0 (gpio27), bed on HB (gpio21) with TB (gpio26), part fan FAN1 (gpio17), and
gpio18 / gpio20 as fans that come on with the hotend above 50 C. Check these against BTT's pin
diagram for your board revision: the LED, thermistor, hotend heater, stepper, TMC2209 UART, DIAG and hotend-fan (gpio18) pins have been used on this
hardware so far.

- The heaters and the TMC2209 drivers need the board's **12-24 V input**: USB alone powers only the
  logic. Without it the TMC2209s do not answer on their UART (the test page says so).
- A heater cartridge must be clamped in its block with the thermistor in place before you heat:
  `verify_heater` will stop a heater that does not warm its thermistor, but a loose cartridge can
  still get very hot.
- If a heater's thermistor is not fitted, the board shuts itself down as soon as it is configured
  (Klipper's range check). Remove that heater's section from `printer.cfg` instead.

## Power

The ESP32-S3 draws short current spikes when its WiFi transmits. If its 5 V comes through thin or
damaged wires, or a weak supply, the voltage dips and the chip resets ("brownout"); the WiFi then
vanishes and reappears in a rhythm. The status page's **Last restart** row says "brownout" when
that happens. The firmware limits WiFi power to 15 dBm to reduce the spikes. While testing, it is
simplest to power the ESP32-S3 from its own USB cable and connect only GND, TX and RX to the board.
The status LED itself draws only a few milliamps at the brightness used.

## Switching the board off with the ESP32 running

When the ESP32-S3 has its own power (USB) and the printer board is switched off, the ESP32's TX
wire (3.3 V when idle) feeds a little current into the board's RX pin and through the chip's pin
protection into the dead board. The board can then sit half-powered and **not restart properly when
its supply comes back**: it answers nothing and its TX stays low (the status page shows only 00
bytes). This is the most likely explanation for the 2026-10-14 failure (the SKR Pico could not be
reached after it was power-cycled with the ESP32 running); it was not measured.

What the firmware does about it (since 2026-10-14): it notices within a fraction of a second that the
board's TX went low, stops driving its own TX wire (lets it float), and only drives it again once the
board's TX is back high, i.e. the board is powered and its firmware is running. For a few seconds
after a board stops answering for other reasons, the ESP32 still drives the wire.

What you can do as well:

- Power the ESP32-S3 from the board's 5V pin (see Wiring): then both switch off together.
- Or wait 10 seconds before switching the board back on.
- Or (hardware, not tried on this project) put a 4.7 kOhm resistor in series in the ESP32 TX ->
  board RX wire. It limits that current to under 1 mA; at 250000 baud the signal is not affected
  with short wires.
