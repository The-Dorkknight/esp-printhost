// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Shared declarations for the Skipper Flipper ESP32-S3 firmware.
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SF_VERSION "0.1.0-wip"
#define SF_BANNER "WORK IN PROGRESS: MANY FEATURES MAY BE BROKEN"

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
} sf_settings_t;

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
// The default printer.cfg (BTT SKR Pico), for the "load default" button. Returns the length.
size_t sf_printer_cfg_default(char *out, size_t n);
size_t sf_printer_cfg_default_tina2(char *out, size_t n);
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
// True while a print runs (uploads are refused then).
bool sf_printing(void);
const char *sf_link_state_name(sf_link_state_t st);

void sf_led_start(const sf_settings_t *s);

#ifdef __cplusplus
}
#endif
