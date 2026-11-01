// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Shared declarations for the Skipper Flipper ESP32-S3 firmware.
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SF_VERSION "0.1.0-wip"
#define SF_BANNER "WORK IN PROGRESS: MANY FEATURES MAY BE BROKEN"

// The left bar on every page: fixed (it does not scroll with the page), the current page lit,
// STOP at the bottom. No '%' anywhere in it: it goes into printf formats too.
#define SF_NAV                                                                                                  \
    "<style>.sn{position:fixed;left:0;top:0;bottom:0;width:108px;background:#141414;display:flex;"              \
    "flex-direction:column;z-index:20;overflow-y:auto;box-shadow:2px 0 0 #ffd400}"                                \
    ".sn a{color:#e9e6dc;text-decoration:none;font:700 13px/1.15 Impact,Arial Black,sans-serif;letter-spacing:.04em;" \
    "padding:11px 6px;border-bottom:1px solid #2c2c2c;text-align:center}.sn a i{display:block;font:normal 21px/1 "    \
    "Georgia,serif;margin-bottom:3px}.sn a:hover{background:#2c2c2c}.sn a.on{background:#ffd400;color:#141414}"     \
    ".sn button{margin:auto 6px 8px;background:#c4161c;color:#fff;border:3px solid #fff;font:700 15px Impact,Arial " \
    "Black,sans-serif;padding:12px 2px;letter-spacing:.05em;cursor:pointer}body{padding-left:108px}"              \
    "@media(max-width:600px){.sn{width:62px}.sn a{font-size:10px;padding:9px 1px}.sn a i{font-size:18px}"           \
    ".sn button{font-size:12px;padding:10px 1px}body{padding-left:62px}}</style>"                                  \
    "<nav class=\"sn\"><a href=\"/\"><i>&#9654;</i>Printer</a><a href=\"/status\"><i>&#9432;</i>Status</a>"          \
    "<a href=\"/test\"><i>&#9881;</i>Test bench</a><a href=\"/tune\"><i>&#8776;</i>Tune</a>"                    \
    "<a href=\"/config\"><i>&#9776;</i>printer.cfg</a>"                                                             \
    "<a href=\"/console\"><i>&#8801;</i>Log</a><a href=\"/settings\"><i>&#9998;</i>Settings</a>"                    \
    "<a href=\"/update\"><i>&#8679;</i>Update</a><a href=\"/about\"><i>&#9875;</i>About</a><button id=\"snstop\" title=\"Emergency stop (M112)\">STOP</button>" \
    "</nav><script>(function(){var p=location.pathname=='/printer'?'/':location.pathname,l=document.querySelectorAll('.sn a');"                      \
    "for(var i=0;i<l.length;i++)if(l[i].getAttribute('href')==p)l[i].className='on';"                            \
    "document.getElementById('snstop').onclick=function(){fetch('/gcode?cmd=M112',{cache:'no-store'});"           \
    "this.textContent='SENT'}})()</script>"

enum { SF_LED_NONE = 0, SF_LED_PLAIN = 1, SF_LED_WS2812 = 2 };

// Stored in NVS; changed on the /settings page.
typedef struct {
    int uart_tx;      // ESP32-S3 GPIO wired to the printer board's RX
    int uart_rx;      // ESP32-S3 GPIO wired to the printer board's TX
    int baud;
    int led_type;     // SF_LED_*
    int led_gpio;
    int led_rgb;      // 1 = colour LED takes red,green,blue order; 0 = standard WS2812 green,red,blue
    int link_enabled; // 0 = do not touch the UART pins
    int sd_enabled;   // micro SD card on SPI for G-code files
    int sd_cs, sd_sck, sd_mosi, sd_miso;
    char sta_ssid[33];  // home WiFi to join as well (empty: access point only)
    char sta_pass[65];
    int oled_type;      // SF_OLED_*: little I2C OLED (2026-10-23)
    int oled_sda, oled_scl;
    int oled_flip;      // 1 = mounted upside down
} sf_settings_t;
enum { SF_OLED_OFF = 0, SF_OLED_AUTO = 1, SF_OLED_SSD1306_32 = 2, SF_OLED_SH1106 = 3 };

void sf_settings_load(sf_settings_t *s);
bool sf_settings_save(const sf_settings_t *s);
void sf_settings_defaults(sf_settings_t *s);

// Printer-board link state (also drives the LED).
typedef enum { SF_LINK_OFF = 0, SF_LINK_SEARCHING, SF_LINK_OK, SF_LINK_ERROR } sf_link_state_t;

#define SF_MAX_TEMPS 4
typedef struct {
    char name[24];     // "hotend" from [temperature_sensor hotend]
    char text[56];     // "18.2 °C", "no sensor? (reads open circuit)", ...
    float c;           // degrees C, valid when state == 1
    int state;         // 0 waiting for the first reading, 1 ok, 2 open circuit, 3 short circuit
} sf_temp_t;

typedef struct {
    sf_link_state_t state;
    char version[64];
    char mcu[40];
    char pins[120];
    char last_error[120];
    uint32_t clock_freq;
    uint32_t dict_bytes;
    uint32_t crc_errors;
    uint32_t attempts;
    // raw UART diagnostics (updated after every attempt)
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t resyncs;
    char rx_hex[52];   // last up to 16 received bytes, hex
    int loopback;      // 1 = the ESP32 heard its own transmission (TX wired to RX)
    int echo_suspected; // garbled copies of our own frames coming back (TX coupling into RX)
    // printer board configuration from printer.cfg (Klipper allocate_oids / config_* / finalize_config)
    char cfg_status[96]; // "configured", "already configured", "printer.cfg has errors", ...
    int cfg_ok;          // board configured from the current printer.cfg
    char first_led[32];  // name of the first [neopixel] (for the status-page buttons), "" if none
    // board clock (Klipper-style clock sync)
    uint32_t clk_hz;     // measured board clock frequency
    float drift_ppm;     // against the nominal CLOCK_FREQ
    float rtt_ms;        // best round trip seen
    uint32_t clk_samples;
    // temperature sensors from [temperature_sensor] in printer.cfg
    int n_temps;
    sf_temp_t temps[SF_MAX_TEMPS];
    int configured;
    int shutdown;
} sf_link_info_t;

void sf_link_start(const sf_settings_t *s);
void sf_link_get(sf_link_info_t *out);
// printer.cfg, stored on the chip. sf_printer_cfg_copy() copies the current text; set() saves a new
// one and makes the link task reconfigure the board (the board restarts once). report() gives the
// last check's errors and notes as text (one per line).
size_t sf_printer_cfg_copy(char *out, size_t n);
bool sf_printer_cfg_set(const char *text);
void sf_printer_cfg_report(char *out, size_t n);
// Run one console command (e.g. "SET_LED LED=board_led RED=1") on the link task; waits for the reply.
bool sf_console(const char *line, char *reply, size_t n, int timeout_ms);
// Pasta Inspector dongle (Moonraker websocket, moonraker_ws.cpp). Status/object JSON are malloc'd copies (free()).
void sf_moon_want(int on);
char *sf_moon_status_dup(long have_rev);   // {"eventtime","rev","status"}; -1 = with object list
// The default printer.cfg (BTT SKR Pico), for the "load default" button. Returns the length.
size_t sf_printer_cfg_default(char *out, size_t n);
size_t sf_printer_cfg_default_ender3(char *out, size_t n);
// Heaters, fans, motors and tests as a JSON object ("" when the board is not configured).
// The printer status JSON (ph::Machine::status_json), as a malloc'd copy the caller frees; NULL if none.
char *sf_machine_json_dup(void);
// The heater log of the previous board session (before the last shutdown / restart), CSV.
size_t sf_prev_heater_log(char *out, size_t n);

// The uploaded G-code file (storage.c): one file, kept in its own flash partition.
size_t sf_store_capacity(void);
bool sf_store_begin(const char *name, uint32_t data_len, uint32_t orig_len, bool compressed);
bool sf_store_write(const void *data, size_t len);
bool sf_store_end(void);
bool sf_store_info(char *name, size_t n, uint32_t *data_len, uint32_t *orig_len, bool *compressed);
bool sf_store_read(uint32_t offset, void *buf, size_t len);
// printer.cfg at the end of the same partition (2026-10-27): up to CONFIG_SKIPPER_CFG_KB KB
size_t sf_cfgstore_max(void);                          // 0 if there is no room for it
int sf_cfgstore_get(char *out, size_t n);              // length, or -1 if none saved there yet
bool sf_cfgstore_put(const char *text, size_t len);
// True while a print runs (uploads are refused then).
bool sf_printing(void);
const char *sf_link_state_name(sf_link_state_t st);

void sf_led_start(const sf_settings_t *s);

// ---- micro SD card on SPI (sdcard.c), mounted at SF_SD_ROOT
#define SF_SD_ROOT "/sd"
typedef void (*sf_sd_file_cb)(const char *name, uint64_t size, void *arg);
bool sf_sd_mount(const sf_settings_t *s);      // NULL: remount with the last pins
void sf_sd_unmount(void);
bool sf_sd_mounted(void);
const char *sf_sd_error(void);                 // "ok" or why it is not mounted
const char *sf_sd_pin_problem(const sf_settings_t *s);   // NULL if the SD pins are usable
bool sf_sd_space(uint64_t *total, uint64_t *freeb);
int sf_sd_list(sf_sd_file_cb cb, void *arg);   // G-code files in the card's top folder; -1 if no card
bool sf_sd_name_ok(const char *name);          // a plain .gcode/.gco/.g name, no folders
bool sf_sd_path(const char *name, char *out, size_t n);
bool sf_sd_delete(const char *name);

// Serialise SD card access against remounting (sdcard.c); the event log task takes it too.
void sf_sd_lock(void);
void sf_sd_unlock(void);

// ---- build tier and memory (tier.c)
const char *sf_tier_id(void);          // "deckhand", "captain", "skipper"
const char *sf_tier_title(void);       // "CAPTAIN Edition PRO MAX Turbo Ultra", ...
size_t sf_psram_bytes(void);           // PSRAM in use (0: none found, or not in this build)
int sf_lookahead_max(void);            // look-ahead to ask the planner for on this board
int sf_tier_json(char *out, size_t n); // {"id":...} for status.json
void sf_tier_register(void *httpd);    // GET /about

// ---- extras on the SD card (extras.c): event log, print history, printer.cfg backups,
// thumbnails, file access, clock from the browser
void sf_extras_start(void);
void sf_log_tee(const char *s, size_t n);   // from the log hook: copy to the SD event log
// The log of important events (2026-10-27): one line each, also in the scrolling log; kept in memory
// (GET /report.txt) and in skipper/report.log on the SD card.
void sf_report(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
#if CONFIG_SKIPPER_FIXED
#define SF_MATHS "INT32/64 TEST BUILD (whole-number motion maths)"
#else
#define SF_MATHS "STANDARD build (double motion maths)"
#endif
// "STANDARD build ..., built 2026-10-28 17:05, binary 1a2b3c4d" (2026-10-28): which firmware exactly is
// running, for the important-events log and /report.txt
const char *sf_build_id(void);
void sf_history_add(const char *file, const char *state, double seconds, long lines, const char *msg);
void sf_cfg_backup(const char *old_text);   // keep the printer.cfg being replaced (last 10)
void sf_extras_register(void *httpd);       // /history.json /thumb /sdls /sdfile /time
bool sf_clock_set(void);                    // wall clock known (set from a browser)

// ---- slicer "send to printer" (slicer_api.cpp): OctoPrint-style /api/version, /api/files/local
void sf_slicer_api_register(void *httpd);

// ---- firmware update over WiFi (ota.c, CAPTAIN and SKIPPER builds)
void sf_ota_register(void *httpd);
void sf_ota_boot_check(void);   // after a WiFi update: keep the new build once it has run 30 s

// ---- crash reports (crash.c): last log lines kept across a crash + the core dump summary
void sf_crash_boot(void);               // first thing in app_main
void sf_lastwords_add(const char *s, size_t n);
void sf_crash_save(void);               // after the SD card is mounted
const char *sf_crash_short(void);       // "" or e.g. "Stack canary watchpoint triggered (led) in task led"
void sf_crash_register(void *httpd);    // GET /crash, GET /coredump.bin

// ---- what the little OLED shows (link.cpp fills it twice a second; oled.cpp draws it)
typedef struct {
    int have_ext, have_bed;
    float ext_temp, ext_target, bed_temp, bed_target;
    char state[12];      // standby printing paused complete shutdown ("" while the board is not configured)
    float progress;      // 0..1 while printing or paused
    char activity[16];   // HEATING HOMING EXTRUDING RETRACTING LOADING UNLOADING, or ""
} sf_display_t;
void sf_display_get(sf_display_t *out);
void sf_oled_start(const sf_settings_t *s);
void sf_oled_desc(char *out, size_t n);   // "SSD1306 128x64 at 0x3C", "not found on GPIO8/GPIO9", "off"
const char *sf_oled_pin_problem(const sf_settings_t *s);   // NULL if the OLED pins are usable
void *sf_oled_bus(void);               // the OLED's I2C bus handle (shared with an accelerometer), or NULL
void sf_oled_pause(bool pause);        // stop sending frames (a resonance test needs the bus)

// ---- temperature sensors on the ESP32 itself (hostadc.c, 2026-10-27): sensor_pin: esp32:gpioN
void sf_host_adc_settings(const sf_settings_t *s);   // pins in use (UART, LED, SD, OLED) are refused
bool sf_host_adc(int gpio, float *ratio);             // fraction of 3.3 V; false if the pin cannot be used

// ---- buttons on the case (panel.cpp, 2026-10-26): touch pads or switches, /buttons
enum { SF_PANEL_OFF = 0, SF_PANEL_TOUCH = 1, SF_PANEL_SWITCH = 2 };
void sf_panel_start(const sf_settings_t *s, void *httpd);
bool sf_panel_uses_pin(int gpio);                         // the accelerometer search leaves these alone
const char *sf_panel_conflict(const sf_settings_t *s);   // NULL, or why the main settings clash with the buttons
bool sf_panel_msg(char *out, size_t n);                  // a short OLED message after a press ("PREHEAT", "HOLD 1 S")

// ---- CAPTAIN / SKIPPER extras (pro.cpp)
void sf_bench_boot(const sf_settings_t *s);   // QEMU timing builds: benchmark + accelerometer search at start-up
void sf_accel_scan_now(const sf_settings_t *s);   // the search, waiting for it (QEMU start-up run)
// accelerometer (accel.cpp): MachineIO::accel_record, and GET /accel, /accel.json
bool sf_accel_record(int mode, int axis, char *err, size_t errn);
void sf_accel_init(const sf_settings_t *s, void *httpd);
void sf_tune_register(void *httpd);   // GET /tune (web_pro.cpp)
void sf_pro_register(void *httpd);   // GET /bench ...
void sf_moonraker_register(void *httpd);   // /websocket (Pasta Inspector dongle)

// ---- home WiFi (main.c)
const char *sf_ap_ssid(void);   // "SkipperFlipper-XXXX"
const char *sf_sta_ip(void);    // "" if not connected
const char *sf_sta_state(void); // "off", "connecting", "connected", "wrong password?", ...

#ifdef __cplusplus
}
#endif
