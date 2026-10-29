# Can Skipper Flipper reach "Klipper-grade" performance?

*Research note, 2026-10-27. Skipper Flipper project, `docs/research/`. Written with a research agent's help from the sources listed at the end; figures marked [est] or [calc] are ours, not measured.*

**Question.** Skipper Flipper runs the Klipper *host* role (G-code, planner, step generation, web UI) on an ESP32-S3. It drives a stock Klipper v0.13.0 MCU (BTT SKR Pico, RP2040) over UART at 250000 baud, using Klipper's normal `queue_step` protocol. Can it reach what people call "Klipper-grade": 500+ mm/s, 20k+ mm/s², input shaping (IS), pressure advance (PA), high step rates and multiple MCUs? Two side questions: is a Bambu Lab A1 secretly running Klipper, and what would an ESP32-S31 "Neptune" tier add?

**Where we are now (our own measurements, not from sources):**
- Fixed-point step compression.
- IS and PA computed on a 2 ms time grid, not per step.
- In QEMU, at 150 mm/s: about 6 % of one core without IS/PA, about 14 % with IS+PA.
- Real silicon measured about 2.5-3x slower than QEMU, at 160 MHz with 16/32 KB caches. Builds now run at 240 MHz with 32/64 KB caches.
- UART load: about 320-430 `queue_step`/s, roughly 5 KB/s out of about 25 KB/s available.
- Supported setups: one MCU board, cartesian or CoreXY.

Labels used below: **[src]** = taken from a cited source. **[calc]** = our own arithmetic. **[est]** = an engineering estimate that needs to be measured. **[?]** = uncertain or unverified.

---

## 0. Checked against our code, and what was already done (2026-10-27)

- **Already applied** (built for all three tiers; not yet measured on the chip):
  - CPU at 240 MHz on every build (the ESP-IDF default was 160 MHz).
  - 32 KB instruction / 64 KB data cache on CAPTAIN and SKIPPER (DECKHAND keeps the defaults: the bigger caches take internal RAM it does not have).
  - `-O2` for `toolhead.cpp`, `shaping.cpp` and `planner.cpp` on CAPTAIN and SKIPPER (the rest stays size-optimised).
  - The printer link task has core 1 to itself at priority 10; WiFi and the TCP/IP task stay on core 0. This was the "Timer too close" fix, and is option 4 below.
- **Not applied:**
  - **QIO flash:** some boards' flash chips do not start in QIO; a wrong guess means a board that does not boot. To try per board, not by default.
  - **IRAM placement:** DECKHAND has no internal RAM to spare; worth measuring on CAPTAIN/SKIPPER first.
  - **Faster UART:** needs the SKR Pico's Klipper rebuilt for the new baud (still plain Klipper, only a `make menuconfig` choice); 500000 is the easy next step.
- **Float drift (§4 option 6) does not apply as written:** the motion code (`toolhead.cpp`, `planner.cpp`, `shaping.cpp`) works in `double` (about 220 uses) and 64-bit board clocks; `float` appears only in a few places in `toolhead.cpp`. The flip side is that on the ESP32-S3 every `double` operation is done in software, so this is probably where much of the measured time goes. Moving the per-grid-point maths to `float` with move-local times (as the note suggests) is the larger, riskier speed-up, and needs the PC tests to show the same step counts first.
- **Grid error (§4 option 5):** the 2 ms grid is ours; at the QIDI's starting 5000 mm/s² the note's formula gives about 2.5 µm, comfortably inside the 25 µs window. It only matters near 20 000 mm/s².
- **Naming:** "NEPTUNE" is reserved for an ESP32-S31 tier. Nothing is built for it: the chip's ESP-IDF support was still a preview when this was written.

---

## 1. Where Klipper's host CPU time goes

### 1.1 Pipeline (Klipper v0.13-era code)

1. **G-code parsing and lookahead (Python).** `ToolHead` queues moves in `LookAheadQueue`. `flush()` picks start and end speeds for each move, using junction speeds from `square_corner_velocity`. Each move then becomes an accelerate/cruise/decelerate trapezoid and goes into a C "trapq" through `trapq_append()` [src: Code_Overview, Kinematics]. `minimum_cruise_ratio` caps the top speed of short zig-zag moves [src: Kinematics]. This part is mostly per-move cost, not per-step.
2. **Iterative solver (C, `itersolve.c`).** Each stepper has a callback that returns its position at time t. The solver uses the **secant method**, falling back to **bisection**, to find the time of every half-step crossing. It works in `double` and stops at 1e-9 mm or 1e-9 s [src: itersolve.c]. The cost is **per step**, and each step needs several callback calls.
3. **Input shaper (C, `kin_shaper.c`).** IS is a kinematic wrapper: the axis position is a weighted sum of the trapezoid position at 2-5 shifted times (up to 10 pulses are allowed) [src: kin_shaper.c]. So every solver evaluation costs N trapq lookups. In real Klipper, IS multiplies step-generation cost by about the pulse count [calc, from the code].
4. **Pressure advance (C, `kin_extruder.c`).** `pa_position = nominal + K * velocity`. This is smoothed by a triangular-weighted integral over `pressure_advance_smooth_time` (default 40 ms), which can span several moves [src: Kinematics, kin_extruder.c, Config_Reference]. Each extruder position evaluation computes closed-form integrals over every move in the window.
5. **Step compression (C, `stepcompress.c`).** Step times are turned into `queue_step(interval, count, add)` triples, using a bisection search for the longest run that stays inside the error window. The window is 25 µs by default [src: stepcompress.c, Features]. Each command is parsed and queued on the MCU at least 100 ms before its first step [src: Code_Overview].
6. **Serial queue (C thread per MCU, `serialqueue.c`)**, plus `steppersync.c` to order commands across steppers [src: Code_Overview].
7. **Clock sync (Python, `clocksync.py`).** A linear regression of MCU clock against host time, with exponential decay and outlier rejection. This is what allows multiple MCUs [src: clocksync.py, Features].

### 1.2 Typical host CPU on Pi-class hardware

- **There is no official host CPU benchmark.** The Benchmarks page measures MCU step rate and command dispatch, and the host "batch mode" section gives a method but no results [src: Benchmarks].
- The FAQ recommends a Pi Zero 2 W, 3, 4 or 5 as host. It says older boards "don't have enough processing power to run Klipper well", and heavy unrelated host jobs can cause print errors [src: FAQ]. The Pi Zero 2 W has four Cortex-A53 cores at 1 GHz, each with a double-precision FPU.
- Kevin O'Connor wrote that the host has to handle "100s of thousands of steps per second" and that steps must land within 25 µs [src: Patreon post 60563841].
- In the forum example we found, the limit at 375-450 mm/s was the **ATmega MCU**, not the Pi 3 host [src: Klipper Discourse #19368].
- **Takeaway [est]:** a Pi-class host has 10-50x more double-precision compute than an ESP32-S3. Klipper simply spends it: secant solving per step in `double`, times IS pulses. Skipper Flipper's grid approach is the main reason it fits on an MCU at all.

### 1.3 Step rates needed [calc]

| Speed | 80 steps/mm (16x, 20T GT2) | 160 steps/mm (32x) | CoreXY worst motor (45° move, ×√2) at 80 / 160 |
|---|---|---|---|
| 150 mm/s | 12 k/s | 24 k/s | 17 k / 34 k/s |
| 300 mm/s | 24 k/s | 48 k/s | 34 k / 68 k/s |
| 500 mm/s | 40 k/s | 80 k/s | 57 k / 113 k/s |

- **Extruder:** 0.45 × 0.2 mm lines at 500 mm/s is 45 mm³/s, which is about 19 mm/s of 1.75 mm filament. At 400-700 steps/mm that is 7.5-13 k steps/s, plus PA transients.
- **Total for a 500 mm/s CoreXY at 160 steps/mm:** roughly **150-250 k steps/s** at peaks.
- **The RP2040 is not the bottleneck.** Klipper's benchmark reaches 3 ticks per step with one stepper and 14 ticks with three, on a 12 MHz timer. That is well over a million steps/s [src: Benchmarks; calc].

### 1.4 Link bandwidth

- At 250000 baud 8N1, the link carries about 25 KB/s. Klipper's serial command dispatch benchmarks are tens of k commands/s, while USB reaches about **885 k commands/s** on RP2040 [src: Benchmarks].
- Our measured ~5 KB/s at 150 mm/s works out to about 12-15 bytes per command.
- **Scaling to 500 mm/s / 20k accel [est]:**
  - Linear scaling with speed alone gives about 17 KB/s (about 70 % of the link).
  - Higher acceleration, short moves and continuously varying IS/PA velocity profiles break compression into more, shorter `queue_step` runs. Peaks of 2-6x that figure are plausible.
  - **Conclusion: 250000 baud is marginal-to-insufficient for 500 mm/s. Measure command bytes/s on a worst-case file before deciding.**

## 2. Silicon: ESP32-S3 today, ESP32-S31 "Neptune", ESP32-P4

| | ESP32-S3 (current) | ESP32-S31 ("Neptune") | ESP32-P4 |
|---|---|---|---|
| CPU | 2× Xtensa LX7 @ 240 MHz, vector instructions [src] | 2× RISC-V RV32IMAFC @ 320 MHz, MMU; one core has a 128-bit SIMD/AI extension; 40 MHz LP core [src: datasheet v0.5, Espressif PR, components101] | 2× RISC-V @ 400 MHz, AI extensions; 40 MHz LP core [src] |
| FPU | **Single precision only; `double` is software-emulated** [src: Espressif "cores with FPU", ESP-IDF speed guide] | Single precision ("F" extension). Datasheet does not mention double, so assume **no double FPU** [src: datasheet v0.5; ?] | Single precision only [src] |
| SRAM | 512 KB [src] | 512 KB HP + 32 KB LP; 320 KB ROM [src] | 768 KB (usable as cache) + 8 KB TCM [src] |
| PSRAM | Octal SPI, 80 MHz (120 MHz with restrictions) [src: ESP-IDF external RAM] | In-package 8-bit DDR @ 250 MHz, 16/32 MB variants (up to 64 MB per datasheet); flash and PSRAM can be accessed concurrently [src] | Faster PSRAM (speed not stated on product page) [?] |
| USB | USB 2.0 OTG **Full-Speed** + Serial/JTAG; host library supports FS/LS devices, CDC-ACM host driver available [src] | USB 2.0 **High-Speed OTG** + Serial/JTAG [src] | USB 2.0 HS OTG [src] |
| Ethernet | none (SPI PHY add-on) | **1 Gbit EMAC** (PHY integration unconfirmed [?]) [src] | Ethernet MAC [src] |
| Wireless | Wi-Fi 4, BLE 5 | **Wi-Fi 6 (2.4 GHz)**, BT 5.4 Classic + LE, 802.15.4 [src] | **none**; needs a companion C/S chip [src] |
| UARTs | 3 (datasheet: up to 5 Mbps [?, not re-checked]) | 4 + LP UART [src] | – |
| Status | Mature | Mass production announced (article dated 30 Jul 2026) [src: components101]. ESP-IDF support is **preview on master** per Espressif dev portal (page dated Mar 2026; may be outdated) [src; ?]. Datasheet still "Preliminary v0.5". | Mature in ESP-IDF |
| Price | ~US$2-3 chip | **Not published** in sources found [?] | – |

**Implications for Skipper Flipper**
- **No chip in this family gives hardware `double`.** The firmware must stay float/fixed-point by design. Long-print precision is covered in §4.4.
- **S31 vs S3, raw compute [est]:** 320 MHz versus 240 MHz is 1.33x on clock alone. A modern RISC-V core with a concurrent-access DDR PSRAM path and faster PSRAM may add something per clock, but there are no published CoreMark/DMIPS figures for the S31 yet [?]. Plan for **about 1.3-2x** the S3, not 10x.
- **USB HS does not help the MCU link.** The SKR Pico's RP2040 is a Full-Speed device. FS (12 Mbit/s) is already about 50x the current UART.
- **Gigabit Ethernet and Wi-Fi 6** improve the web UI and file transfer, not motion.
- **ESP32-P4 is the faster CPU** (400 MHz, more SRAM) but has no radio. The S31 is the better all-in-one "Neptune" chip. A P4 + C6 pair would be a "Neptune Pro" option.

## 3. Bambu Lab A1 / A1 mini

**Official hardware facts**
- **A1 tech specs:** max toolhead speed 500 mm/s, max acceleration **10 000 mm/s²**, max hot-end flow 28 mm³/s (ABS). The motion controller is listed as **"Dual-Core Cortex M4"**. Wi-Fi is 802.11 b/g/n [src: bambulab.com A1 tech specs].
- **Bambu's open-source wiki:** "There are no General Public License (GPL) components present in the P1P/P1S/A1 mini/A1 system, which utilizes an ESP32 microcontroller", built with the Espressif-provided SDK [src: Bambu wiki].
- **By contrast, the X1/X1C use a Rockchip RV1126 running Linux 4.19**, with u-boot, busybox and mosquitto, and Bambu publishes that GPL source [src: Bambu wiki].
- **Inference:** the A1 series is an ESP32-class application processor plus Cortex-M4 motion control. **There is no Linux SBC and no quad-core host.**
- **Community teardown (P1 series, not A1):** the OpenBL project identifies the MC board MCU as **Spintrol SPC2168APE80** and the toolhead MCU as **Spintrol SPC1168APE48**, with AT8236 drivers. They say the A1 "will be added eventually" [src: OpenBL]. Whether the A1's "dual-core M4" is the same Spintrol part is **not confirmed** [?].
- **A1 mini wiki:** the mainboard holds an **MC board** and an **AP board**. No chip names are given [src: Bambu wiki].

**Speed and quality features (Bambu's claims)**
- "Auto Tune Both X & Y Resonance Before Every Print". Bambu calls this vibration compensation, i.e. input shaping with automatic frequency identification.
- "Nozzle pressure based flow dynamics calibration". This is automatic PA/flow calibration from a pressure sensor.
- Automatic Z-offset, bed levelling and belt-tension monitoring [src: bambulab.com A1 page].
- Some forum users report visible residual ringing on A1/A2L even at 2000 mm/s². No Bambu staff replied [src: Bambu forum #257966].

**Is there hidden Klipper or Marlin code?**
- **No public evidence of Klipper or Marlin code in A1 firmware.** Both are GPLv3, and Bambu's own disclosure states no GPL components in the A1 system [src: Bambu wiki].
- In a 2023 forum thread, one user (chmarr) said the X1 used Klipper, then withdrew it. Another (Thrawn) pointed out that Bambu would have to disclose GPL use [src: Bambu forum #9470].
- **The real licence fight is about the slicer.**
  - Bambu Studio is an AGPLv3 fork of PrusaSlicer/Slic3r [src: BambuStudio README].
  - On 18 May 2026 the Software Freedom Conservancy said it had **confirmed AGPLv3 violations in Bambu Studio**: missing source for `libbambu_networking`, and restrictions on Paweł Jarczak's OrcaSlicer fork.
  - SFC says its investigation also covers device firmware but reports **no confirmed firmware violation** [src: SFC news].
- **Contrast case:** FlashForge's Adventurer 5M line was alleged on the Klipper forum (Jan 2024) to run modified Klipper. FlashForge reportedly replied its firmware "uses no Klipper code", and Klipper's Sineos said such cases are "on our radar" [src: Klipper Discourse #13313].

**What this means for us**
- The A1 is the best existence proof that **an ESP32-class host plus a Cortex-M4 motion MCU can reach 500 mm/s at 10k mm/s² with input shaping and pressure compensation**.
- Its rated acceleration is 10k, not 20k+. "As capable as a quad-core Klipper gantry" holds for bed-slinger use cases but is not literally true on paper.
- **Bambu's likely trick [inference, not sourced]:** step generation and shaping run *on the motion MCU*, not streamed step-by-step from the host. Bambu owns both ends of the link. Skipper Flipper deliberately keeps a **stock** Klipper MCU, so it streams `queue_step`. That is the price of compatibility.

## 4. Options to close the gap, ranked by benefit ÷ effort

| # | Option | Benefit | Effort | Notes |
|---|---|---|---|---|
| 1 | **Hot code in IRAM, data in internal SRAM, `-O2`, flash QIO** | High (likely recovers much of the 2.5-3x QEMU→silicon loss) | Low | Mark solver/compressor/grid loops `IRAM_ATTR`. Keep trapq, grid buffers and stepcompress queues out of PSRAM. ESP-IDF says QIO "nearly doubles" code-from-flash speed versus DIO, and that IRAM code runs "at full speed" [src: ESP-IDF speed guide]. Profile with `esp_cpu_get_cycle_count()` on silicon, not QEMU. |
| 2 | **Faster MCU link: UART 500k → 1M → 2M** | High (4-8x headroom) | Low | Klipper's `[mcu] baud` is configurable (default 250000) [src: Config_Reference]. RP2040 and S3 UARTs handle Mbaud rates. Needs short wires/ground, and the MCU built for the matching baud. Use UART DMA/large RX FIFO thresholds on the S3 [src: ESP-IDF UART]. |
| 3 | **USB CDC to the SKR Pico (S3 as USB host)** | Very high bandwidth (FS 12 Mbit/s; Klipper USB ~885k cmd/s) | Medium | ESP-IDF has a CDC-ACM host driver [src]. Costs USB host stack CPU and RAM; plug/unplug and reset handling needed. Best long-term link. |
| 4 | **Core split** | Medium-high | Low-medium | Pin motion (planner, grid, stepcompress, serial TX) to core 1 at high priority. Wi-Fi/lwIP/web UI go on core 0. Never block motion on the web server; size the queue for at least 100 ms of lead [src: Code_Overview]. |
| 5 | **Grid accuracy and adaptivity** | Medium (quality) | Medium | Linear interpolation between grid nodes has position error ≤ a·h²/8 [calc]: 2 ms grid, 20k mm/s² → **10 µm** (0.8 step at 80 steps/mm, ≈20 µs at 500 mm/s, i.e. close to the 25 µs budget). 1 ms → 2.5 µm; 0.5 ms → 0.6 µm. Fixes: quadratic/cubic interpolation through nodes (exact for piecewise-constant accel away from breakpoints), align nodes to move boundaries and shaper pulse offsets, or scale h with accel. |
| 6 | **Float vs double for long prints** | High (correctness) | Low-medium | float32 ULP at 10 h = 36 000 s is **3.9 ms**; at 24 h, 7.8 ms [calc]. Absolute time in float is unusable. Extruder absolute position at 100 m of filament has ULP **7.8 µm**, larger than one extruder step (~1.4-2.5 µm) [calc]. Rules: 64-bit integer MCU clocks; per-move *local* time in float; rebase E (and XY) per move or segment; never accumulate in float. Use `double` (soft) only in rare per-move setup, never per step. Klipper uses double everywhere [src: itersolve.c]. |
| 7 | **Multi-MCU (e.g. CAN toolhead)** | Feature | Medium-high | Port `clocksync.py`'s regression (DECAY 1/30, outlier reset) [src: clocksync.py]. It is cheap math; the work is in protocol, ID and config handling. S3 has TWAI (classic CAN 2.0), which matches Klipper's CAN bus use. S31 CAN support not checked [?]. Each extra MCU adds its own serial bandwidth. |
| 8 | **Move IS/PA or step generation to the MCU** | Very high | Very high | Requires custom RP2040 firmware (e.g. PIO stepping). Breaks "plain Klipper MCU", the project's main selling point. Only for a separate "Neptune-native" branch. |
| 9 | **Neptune hardware (ESP32-S31)** | Medium (~1.3-2x CPU [est]), better I/O | Medium (new target, preview IDF) | Wait for stable ESP-IDF support and published benchmarks. Keep code portable (no Xtensa-only intrinsics in the hot path). |

### Expected ceiling [est, to be measured]

- **Starting point:** QEMU at 150 mm/s with IS+PA uses 14 % of a core. With the 2.5-3x silicon penalty that is about 35-42 % of a core at 160 MHz. 240 MHz and larger caches should bring it to about **25-30 %**.
- **Scaling to 500 mm/s:** the per-step part scales with step rate (3.3x). The grid part is roughly constant per second. So **about 70-100 % of a core** before optimisation.
- **After options 1, 4 and 5:** a 1.5-2x gain gives **about 40-65 %** of the motion core.
- **ESP32-S3 + 1 Mbaud UART or USB:**
  - 300-400 mm/s at 10k mm/s² with IS+PA at 80 steps/mm should be dependable.
  - **About 500 mm/s at 80 steps/mm is plausible but borderline.**
  - 160 steps/mm at 500 mm/s on CoreXY (>110 k steps/s per motor) is probably beyond the S3 with IS+PA.
- **Neptune (S31):** adds headroom for 500 mm/s at higher microstepping or a second MCU, if the 1.3-2x estimate holds.
- **20k+ mm/s² is mainly a mechanics, bandwidth and short-move problem, not CPU.** Fix the link first.

## 5. Verdict

- **"Klipper-grade" on a Pi-class host is out of reach** if that means matching Klipper's exact per-step double-precision solver, arbitrary multi-MCU setups and 100+ k steps/s per axis with IS.
- **"Bambu A1-grade" is a realistic target:** 400-500 mm/s, about 10k mm/s², IS + PA on a single board. The A1 shows an ESP32 + Cortex-M4 design can do this.
- **Two things decide it:**
  1. A faster MCU link. 250000 baud will run out first.
  2. Getting the hot path into IRAM/internal SRAM on core 1, measured on silicon, not QEMU.
- **Make sure of grid accuracy and long-print float handling before chasing speed.**
- **There is no evidence that Bambu uses Klipper.** Its published disclosure says the A1 firmware has no GPL components.
- **Neptune/ESP32-S31 is a worthwhile next tier for I/O (Wi-Fi 6, GbE, HS USB) and about 1.3-2x CPU.** It is not a different league, and its ESP-IDF support and pricing should be re-checked before committing.

---

## Sources

**Klipper**
- Klipper Benchmarks: https://www.klipper3d.org/Benchmarks.html
- Klipper Code Overview: https://www.klipper3d.org/Code_Overview.html
- Klipper Kinematics: https://www.klipper3d.org/Kinematics.html
- Klipper Features: https://www.klipper3d.org/Features.html
- Klipper FAQ: https://www.klipper3d.org/FAQ.html
- Klipper Config Reference: https://www.klipper3d.org/Config_Reference.html
- Klipper source (master), `klippy/chelper/itersolve.c`, `stepcompress.c`, `kin_shaper.c`, `kin_extruder.c`, `klippy/clocksync.py`: https://github.com/Klipper3d/klipper/tree/master/klippy
- Kevin O'Connor, Klipper update (Patreon): https://www.patreon.com/posts/60563841
- Klipper Discourse, "Finding out max velocity cpu/bandwidth limits": https://klipper.discourse.group/t/finding-out-max-velocity-cpu-bandwith-limits/19368
- Klipper Discourse, "Flashforge Klipper license violation": https://klipper.discourse.group/t/flashforge-klipper-license-violation/13313

**Espressif**
- ESP32-S3 product page: https://www.espressif.com/en/products/socs/esp32-s3
- ESP32-S3 datasheet: https://documentation.espressif.com/esp32-s3_datasheet_en.html
- ESP-IDF v6.0 ESP32-S3 speed optimisation: https://docs.espressif.com/projects/esp-idf/en/v6.0/esp32s3/api-guides/performance/speed.html
- ESP-IDF v6.0 ESP32-S3 external RAM: https://docs.espressif.com/projects/esp-idf/en/v6.0/esp32s3/api-guides/external-ram.html
- ESP-IDF v6.0 ESP32-S3 UART: https://docs.espressif.com/projects/esp-idf/en/v6.0/esp32s3/api-reference/peripherals/uart.html
- ESP-IDF v6.0 ESP32-S3 USB host: https://docs.espressif.com/projects/esp-idf/en/v6.0/esp32s3/api-reference/peripherals/usb_host.html
- Espressif Developer blog, "Cores with FPU": https://developer.espressif.com/blog/2025/10/cores_with_fpu/
- Espressif ESP32-S31 announcement: https://www.espressif.com/news/ESP32_S31_Release
- ESP32-S31 datasheet (v0.5, preliminary): https://documentation.espressif.com/esp32-s31_datasheet_en.html
- Espressif Developer Portal, ESP32-S31 support status: https://developer.espressif.com/hardware/esp32s31/
- Espressif ESP32-P4 product page: https://www.espressif.com/en/products/socs/esp32-p4

**ESP32-S31 coverage elsewhere**
- Components101, "ESP32-S31 enters mass production" (30 Jul 2026): https://components101.com/news/espressif-esp32-s31-risc-v-soc-enters-mass-production
- CNX Software, ESP32-S31 preliminary specs: https://www.cnx-software.com/?p=167703
- Circuit Cellar, ESP32-S31: https://circuitcellar.com/newsletter/espressif-unveils-esp32-s31-a-dual-core-risc-v-soc/
- Mouser, ESP32-S31-WROOM-1: https://www.mouser.ee/new/espressif/espressif-esp32-s31-wroom-1-modules/

**Bambu Lab**
- Bambu Lab A1 tech specs: https://bambulab.com/en/a1/tech-specs
- Bambu Lab A1 product page: https://bambulab.com/en/a1
- Bambu Lab Wiki, open-source software: https://wiki.bambulab.com/en/knowledge-sharing/open-source-software
- Bambu Lab Wiki, A1 mini MC board replacement: https://wiki.bambulab.com/en/a1-mini/maintenance/mc-board-replacement-guide
- OpenBL (community P1 hardware exploration): https://github.com/opensourcemanufacturing/OpenBL
- Bambu forum, "Does Bambu use any open source software in their firmware?": https://forum.bambulab.com/t/does-bambu-use-any-open-source-software-in-their-firmware/9470
- Bambu forum, A-series input shaping thread: https://forum.bambulab.com/t/operation-of-the-input-shaping-algorithm-in-future-firmware-versions/257966
- BambuStudio README: https://github.com/bambulab/BambuStudio
- Software Freedom Conservancy, Bambu Studio AGPL response (18 May 2026): https://sfconservancy.org/news/2026/may/18/bambu-studio-3d-printer-agpl-violation-response/
- Wikipedia, Bambu Lab: https://en.wikipedia.org/wiki/Bambu_Lab
