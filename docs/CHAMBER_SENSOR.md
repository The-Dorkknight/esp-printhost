# Chamber thermistor on the ESP32

**Status (2026-10-27): built and tested on the PC; not yet tried on hardware.** CAPTAIN and SKIPPER
builds only (DECKHAND has no room for the ADC driver).

A temperature sensor can sit on a spare pin of the ESP32-S3 itself instead of the printer board,
for example a chamber thermistor. It is shown on the Status and Printer pages and by `M105` like
any other sensor, and it never shuts anything down: unplugged, it just says
"no sensor? (reads open circuit)".

## printer.cfg

```
[temperature_sensor chamber]
sensor_type: Generic 3950        # any thermistor type Klipper knows, or a [thermistor NAME]
sensor_pin: esp32:gpio7          # esp32: = a pin of the ESP32-S3, not the printer board
#pullup_resistor: 100000         # the resistor from 3V3 to the pin (default 100 kohm on esp32: pins)
```

## Wiring

```
3V3 ---[ 100 kohm ]---+--- GPIO7
                      |
                 [ thermistor ]
                      |
GND ------------------+
```

- **Pins:** only **GPIO1 to GPIO10** (ADC1; the ESP32's second ADC does not work while WiFi is on).
  On an ESP32-S3-Zero with the SD card (10-13), the OLED (8, 9) and the default buttons (2, 4, 5, 6)
  that leaves **GPIO1 and GPIO7**. GPIO3 is a start-up pin and is refused; so are pins the UART, LED,
  SD card, OLED or buttons use (the log says why).
- **The resistor:** a 100 kohm resistor for a 100 kohm thermistor (Generic 3950, EPCOS 100K...)
  puts room temperature in the middle of the ADC's range. Klipper boards use 4.7 kohm, but on the
  ESP32 that reads near full scale (the ADC stops at about 3.1 V) at room temperature. If you use a
  different resistor, set `pullup_resistor` to its value.
- A 3-pin sensor module (a DS18B20, LM35 or TMP36) is not supported yet; say if you have one.

## How it works

`firmware/esp32s3/main/hostadc.c` reads the pin once a second (16 samples, Espressif's ADC
calibration) and passes the voltage as a fraction of 3.3 V to the machine, which turns it into a
temperature with the same thermistor maths as every other sensor (`host/src/thermistor.cpp`).
Nothing about it is sent to the printer board. A heater cannot use an ESP32 sensor (its safety
checks must run on the printer board).
