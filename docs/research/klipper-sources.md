# Research notes: where the heater, fan, motor and TMC code comes from

Written 2026-10-13 while building the test bench (K5/K6). Source: Klipper **v0.13.0**, the copy the
Flasher downloads to `~/.skipper-flipper/klipper-v0.13.0`, and the shipped SKR Pico firmware's data
dictionary (`firmware/prebuilt/skr-pico-uart/klipper.dict`).

## Board commands (from the real data dictionary)

```
config_digital_out oid=%c pin=%u value=%c default_value=%c max_duration=%u
set_digital_out_pwm_cycle oid=%c cycle_ticks=%u
queue_digital_out oid=%c clock=%u on_ticks=%u
config_stepper oid=%c step_pin=%c dir_pin=%c invert_step=%c step_pulse_ticks=%u
queue_step oid=%c interval=%u count=%hu add=%hi
set_next_step_dir oid=%c dir=%c
reset_step_clock oid=%c clock=%u
stepper_get_position oid=%c                  -> stepper_position oid=%c pos=%i
config_trsync oid=%c
trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c
stepper_stop_on_trigger oid=%c trsync_oid=%c
trsync_trigger oid=%c reason=%c              -> trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u
config_tmcuart oid=%c rx_pin=%u pull_up=%c tx_pin=%u bit_time=%u
tmcuart_send oid=%c write=%*s read=%c        -> tmcuart_response oid=%c read=%*s
emergency_stop
```

Config values: `CLOCK_FREQ` 12000000, `ADC_MAX` 4095, `MCU` rp2040, `STEPPER_STEP_BOTH_EDGE` 1.
The test `host/test/skr_pico_dict.h` is generated from this dictionary.

## Heaters (klippy/extras/heaters.py, verify_heater.py, mcu.py)

- `MAX_HEAT_TIME = 5.0` s: the heater pin's `max_duration`. `PID_PARAM_BASE = 255` (config PID
  values are divided by it). `pwm_delay` = the ADC report time (0.3 s). `pwm_cycle_time` default
  0.100 s, max = report time.
- Soft PWM (no `hardware_pwm`): `config_digital_out` + `set_digital_out_pwm_cycle`, updates with
  `queue_digital_out on_ticks = value * cycle_ticks`.
- `set_pwm`: skip if the value changed by less than 0.05 and the last update is recent; otherwise
  schedule at `read_time + pwm_delay`, next forced update at `pwm_time + 0.75 * MAX_HEAT_TIME`.
- PID: derivative smoothed over `smooth_time` (1 s), integral clamped to `max_power / Ki`, integral
  frozen while the output is saturated.
- Range check: `setup_minmax(min_temp, max_temp)` -> `min_value = int(min(adc) * max_adc)`,
  `max_value = ceil(max(adc) * max_adc)`, `range_check_count = 4`; `max_adc = 8 * ADC_MAX`.
- verify_heater defaults: `hysteresis 5`, `max_error 120`, `heating_gain 2`, `check_gain_time 20`
  (60 for `heater_bed`), checked once a second.
- Firmware side (`src/gpiocmds.c`): an output away from its default with `max_duration` set must
  receive its next update before `max_duration` runs out, or "Missed scheduling of next digital out
  event"; a queued update later than that: "Scheduled digital out event will exceed max_duration".
  `src/sched.c`: a timer added in the past: "Timer too close".

## Fans (fan.py, heater_fan.py)

`cycle_time 0.010`, `max_duration 0`, `kick_start_time 0.1` (full power first when starting from
off or jumping by more than 0.5), `off_below`. `heater_fan`: on at `fan_speed` when the heater has a
target or is above `heater_temp` (50 C).

## Steppers (stepper.py, stepper.c, trsync.c, force_move.py)

- Default `step_pulse_duration` 2 us when the step-on-both-edges optimisation is not used (this
  project does not use it yet; Klipper with TMC drivers uses 100 ns on both edges).
- Direction: `set_next_step_dir dir = 1` for the positive direction, XOR the `dir_pin` inversion.
- `reset_step_clock` while the stepper is still stepping shuts the board down ("Can't reset time
  when stepper active").
- Abort: `trsync_start` + `stepper_stop_on_trigger`, then `trsync_trigger` clears the stepper's
  queue at once (as homing does).
- `STEPPER_BUZZ`: 1 mm forward and back 10 times at 4 mm/s, with 0.05 s and 0.45 s pauses.

## TMC2209 (tmc2209.py, tmc2208.py, tmc2130.py, tmc.py, tmc_uart.py)

- UART: 40000 baud (`bit_time = CLOCK_FREQ / 40000`), sync byte 0xf5, CRC8-ATM, 10 bits per byte
  (start, data LSB first, stop); reads expect 10 bytes back; writes are checked through IFCNT.
- Current: `cs = int(32 * (Rsense + 0.02) * I * sqrt(2) / Vref + 0.5) - 1`, Vref 0.18 V (vsense) or
  0.32 V; `hold_current` default 2.0 A (MAX_CURRENT), clamped to `run_current`; Rsense 0.110.
- Reference output of `host/tools/klipper_tmc_reference.py` (Klipper's own code, mocked):

```
stepper_x GCONF=0x000001c0, SLAVECONF=0x00000200, CHOPCONF=0x14030053, IHOLD_IRUN=0x00081212,
  TPWMTHRS=0x00000000, TCOOLTHRS=0x00000000, COOLCONF=0x00000000, PWMCONF=0xc80d0e24,
  TPOWERDOWN=0x00000014, SGTHRS=0x00000000
extruder (no stealthchop) GCONF=0x000001c4, ..., IHOLD_IRUN=0x00081414, TPWMTHRS=0x000fffff
read IOIN addr0 ea03c82094
write GCONF addr3 ea1b083080000a08b8ca
reply IOIN 0x21000040 0afacfa090000208a8a7
```

`ph::tmc2209_registers` and `ph::tmc_encode_*` reproduce these exactly (`test_machine.cpp`).

## SKR Pico pins (config/generic-bigtreetech-skr-pico-v1.0.cfg)

X: step gpio11, dir !gpio10, enable !gpio12, endstop ^gpio4. Y: gpio6, !gpio5, !gpio7, ^gpio3.
Z: gpio19, gpio28, !gpio2, ^gpio25. E: gpio14, !gpio13, !gpio15. TMC UART: rx gpio9, tx gpio8,
addresses X 0, Y 2, Z 1, E 3. Heaters: hotend gpio23 (sensor gpio27), bed gpio21 (sensor gpio26).
Fans: gpio17 (`[fan]`), gpio18, gpio20 (`[heater_fan]`s). NeoPixel gpio24.

## Not verified

- These pins are from Klipper's sample; only gpio24 (LED), gpio26 and gpio27 (thermistors) have been
  used on this hardware so far.
- The SKR Pico firmware's move-queue size (`move_count`) was not checked; a motor test queues about
  70 moves.
- `tmcuart_response` for writes is assumed to arrive (Klipper's host waits for it as a query).

## Moves, homing, printing (added 2026-10-13)

- Homing (klippy/homing.py, mcu.py MCU_endstop / MCU_trsync): `endstop_home oid clock sample_ticks
  sample_count rest_ticks pin_value trsync_oid trigger_reason` with sample time 15 us x 4 samples;
  trsync reasons: 1 endstop hit, 2 host request, 3 past end time, 4 comms timeout; a homing move is
  1.5 x the axis length; disarming sends `endstop_home` with zeros, then `trsync_trigger reason=2`.
  `homing_positive_dir` is inferred from where `position_endstop` lies (stepper.py PrinterRail).
- Sensorless (extras/tmc.py TMCVirtualPinHelper): for "stallguard4" drivers (TMC2209) homing needs
  stealthChop: `TPWMTHRS = 0`, `en_spreadcycle = 0` in GCONF, `TCOOLTHRS = 0xfffff` if it was 0;
  restored afterwards. The DIAG pin is set up as an ordinary endstop (with its pull-up).
- Step compression: Klipper's chelper/stepcompress.c fits `interval + k*add` runs within
  `max_error` (25 us). This project's `ph::StepQueue` lets the interval float within the error window
  for a handful of trial adds (float maths: the ESP32-S3 has no double-precision hardware) and keeps the
  longest run. Test: 3000 steps of an accelerate-cruise-decelerate profile give 30 commands, worst
  error 290 ticks of the allowed 300 (12 MHz).
- Square corner velocity to junction deviation as Klipper's toolhead.py: `scv² (√2 - 1) / max_accel`.
- `max_extrude_only_velocity` default: Klipper's `max_velocity * 4 * nozzle² / filament area`.
- WEEDO Tina 2 mechanics: the community config github.com/gistogisto/Tina2Klipper `printer.cfg`
  (fetched 2026-10-13; written for the stock board): X/Y rotation_distance 32, Z 8, extruder 33.725,
  X homes to max (97), Y to 0, Z endstop at max (91.4), EPCOS 100K, PID 41.141/2.716/155.822,
  max_velocity 97, max_accel 1000, max_z_accel 250.
- SKR Pico (BTT wiki, global.bttwiki.com/SKR%20Pico.html, fetched 2026-10-13): power input
  "DC 12/24V"; sensorless homing by jumper caps per axis, then that axis' endstop input cannot be
  used; LED5 is the heater (HE) status light. The DIAG -> gpio4/gpio3/gpio25 mapping is inferred from
  Klipper's SKR Pico endstop pins, not checked against BTT's pinout PDF.
- File reader check: a 12,000-line G-code file compressed by Chromium's `CompressionStream("deflate")`
  (exactly what the Printer page does) was read back line by line by the ESP32's `GcodeReader` code
  compiled on a PC with miniz 3.x's `tinfl` (the same inflater the ESP32-S3 has in ROM): byte-identical,
  and the same for an uncompressed upload.

## ESP32-S3 memory (added 2026-10-14)

- ESP-IDF builds C++ without exceptions here: a failed `new` calls `abort()` (`__wrap___cxa_allocate_exception`,
  `components/cxx/cxx_exception_stubs.cpp`), so every large allocation must be checked before it is made.
- This board has 2 MB flash and no PSRAM (`CONFIG_SPIRAM` not set); about 160 KB heap free after WiFi starts, but
  the largest single free block is smaller and was not enough for a 45 KB planner ring. `heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)`
  is what the firmware now checks.
- Klipper itself plans by time (it flushes its look-ahead after about 2 s or when the queue fills), not by a fixed
  move count; 32 moves is this project's choice, sized so the stop from 97 mm/s at 1000 mm/s^2 (4.7 mm) fits even
  with 0.2 mm segments.

## Board connection (added 2026-10-14)

- Klipper `src/command.c` `command_find_block()`: a block with a wrong length, wrong sequence-byte
  high bits, bad trailer or bad CRC puts the board in "need sync": it discards bytes up to the next
  0x7e and sends one NAK; leading 0x7e bytes are ignored without a NAK. A block with the wrong
  sequence number gets a NAK carrying the expected number (`next_sequence`, 0 after a restart).
- `reset` (`src/generic/armcm_reset.c`, used by the RP2040 build): `NVIC_SystemReset()`, allowed in
  shutdown (`HF_IN_SHUTDOWN`).
- RP2040 serial (`src/rp2040/serial.c`): the RX pin is set up with its pull-up
  (`gpio_peripheral(GPIO_Rx, 2, 1)`), so the line idles high while the board is powered even if the
  ESP32 lets go of its TX wire.
- ESP-IDF v5.4 `uart_set_pin()` does not set a pull on the RX pin; the firmware enables the pull-up.
- The half-powered-board explanation for the 2026-10-14 failure (00 bytes after a power cycle with the
  ESP32 running) is an inference from the symptom (the board's TX held low until both were
  unplugged) and the circuit (a driven 3.3 V TX into an unpowered chip's RX pin). Not measured.

## StallGuard (added 2026-10-14)

- BTT's own SKR Pico Klipper config (github.com/bigtreetech/SKR-Pico, `Klipper/SKR Pico klipper.cfg`,
  fetched 2026-10-14): `diag_pin: ^gpio4` (X, commented out) and `^gpio3` (Y) with `driver_SGTHRS: 100`,
  `homing_speed: 50`, `run_current: 0.8` (X/Y; their sample printer, rotation_distance 40). This confirms
  the DIAG -> gpio4 / gpio3 mapping this project had inferred; Z (gpio25) is still inferred.
- TMC2209 rule used by `STALLGUARD_TEST`: DIAG goes high when SG_RESULT <= 2 x SGTHRS, while
  TCOOLTHRS >= TSTEP > TPWMTHRS (tmc.py sets TCOOLTHRS 0xfffff and TPWMTHRS 0 for homing).
  SG_RESULT is register 0x41 (10 bits), TSTEP 0x12 (20 bits, time between 1/256 microsteps in driver
  clocks, 12 MHz). The suggested SGTHRS (30 % of the 10th-percentile free-running SG_RESULT) is this
  project's starting rule, not Klipper's; Klipper's Sensorless_Homing.md tunes by trial.
- No published current rating for the WEEDO Tina 2 motors was found (2026-10-14 search).

## The SKR Pico firmware is a light fork (noted 2026-10-14)

The shipped `skr-pico-uart` firmware is Klipper v0.13.0 (commit 61c0c8d) plus this project's segment
queue: three files copied into `src/` (`segq.c`, `segq_core.c`, `segq_core.h`) and one Kconfig option
(`WANT_SEGQ`) / Makefile line added by `host/tools/skipper_fw.py`. That is why it reports itself as
`v0.13.0-0-g61c0c8d-dirty`. Nothing else in Klipper is changed. The ESP32-S3 host never sends a
`config_segq` / `segq_*` command (it uses Klipper's own `queue_step`), so on this printer the extra code
sits idle (its timer starts only when a segment queue is configured). Heaters, fans, steppers, endstops,
trsync and the TMC UART all run Klipper's own unmodified code. Fan output checked against
`src/gpiocmds.c` `digital_load_event()`: on_ticks >= cycle_ticks is a steady on, 0 a steady off, values
between toggle with the cycle set by `set_digital_out_pwm_cycle`.

## Elegoo Centauri Carbon "potato chip" (added 2026-10-14, for the paper)

- Mainboard (OpenCentauri hardware docs, docs.opencentauri.cc/hardware/CC1/mainboard/): Allwinner R528-S3,
  128 MB in-chip memory, 8 GB eMMC, TMC2209 drivers for X/Y/Z, a separate bed MCU over RS-232, fan and
  heater outputs "controlled by MCU".
- Software (OpenCentauri software notes, docs.opencentauri.cc/software/): "The DSP (used as a Klipper MCU)
  runs Klipper MCU code that has been extended and modified for DSP use", version
  v0.9.1-616-g28f60f7e-dirty; the host runs a monolithic app (web UI, camera, API, screen UI) with Klippy
  transpiled to C++; Tina Linux, kernel 5.4.61, glibc 2.23.
- R528 blocks (Allwinner R528 brief V1.3): dual Cortex-A7 (32+32 KB L1 each, 256 KB L2), single HiFi4 DSP
  (64 KB I-RAM, 64 KB D-RAM), 64 MB DDR2 (S2) or 128 MB DDR3 (S3) in package, audio codec with 3 ADC / 2 DAC,
  LINEOUT, HPOUT, 3 I2S, 8-ch DMIC; UART x6, SPI x2, TWI x4, PWM 8-ch, GPADC 2-ch, LRADC, TPADC 4-ch; USB 2.0
  DRD + host, gigabit EMAC; H.265/H.264 decode; LFBGA 226, 12 x 12 mm (S3).
- NOT verified: the story (heard via Lost in Tech, the 3D-printing YouTube channel @LostInTech3D) that an internal audio DAC was repurposed to drive
  printer hardware because of a GPIO shortage. No document found that says so; the paper marks it as a story.
- Anycubic Klipper-go (github.com/ANYCUBIC-3D/Klipper-go): "a Golang porting of klippy", for Kobra 3 /
  Kobra S1 models, plus K3-klipper-mcu (the C board side). A port of the host, not bare-metal firmware.
- Steamboat Willie (1928) entered the US public domain on 1 January 2024 (Duke Center for the Study of
  the Public Domain); later versions and Disney's trademarks are not affected.
