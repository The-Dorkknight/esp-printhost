// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// UART link to the printer board running Klipper firmware built for serial.
// Uses ph::KLink from host/ (the same code tested on the PC against fake_mcu.py).
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "miniz.h"
#include "nvs.h"
#include "ph/board_session.h"
#include "ph/boardcfg.h"
#include "ph/clocksync.h"
#include "ph/default_printer_cfg.h"
#include "ph/kboard.h"
#include "ph/klink.h"
#include "ph/machine.h"
#include "sf.h"

static const char *TAG = "link";
static const uart_port_t PORT = UART_NUM_1;
static const uint32_t LINK_STACK = 16384;   // dictionary parsing builds big C++ structures
static SemaphoreHandle_t s_mux;
// printer.cfg text (stored in NVS), the last check's report, and a reload request.
static std::string s_cfg_text, s_cfg_report;
static volatile bool s_cfg_reload;
// One console command at a time, run by the link task (only it talks to the UART).
static SemaphoreHandle_t s_con_slot, s_con_done;
static std::string s_con_cmd, s_con_reply;
static volatile bool s_con_pending;
static const size_t CFG_MAX = 4096;
// Defaults shipped by earlier builds. A stored copy that is still exactly one of these (never edited)
// is replaced by the current default (ph::default_printer_cfg(): heaters, fans, motors, TMC2209).
static const char *OLD_DEFAULT_CFG_V1 =
    "# printer.cfg for Skipper Flipper (Klipper format). Supported so far: [neopixel].\n"
    "# Other sections (steppers, heaters, ...) are accepted but not used yet.\n"
    "[mcu]\n"
    "serial: uart\n"
    "\n"
    "[neopixel board_led]\n"
    "# BTT SKR Pico on-board RGB LED, as in Klipper's own SKR Pico config\n"
    "pin: gpio24\n"
    "chain_count: 1\n"
    "color_order: GRB\n"
    "initial_GREEN: 0.15\n";
static const char *OLD_DEFAULT_CFG_V2 =
    "# printer.cfg for Skipper Flipper (Klipper format).\n"
    "# Used so far: [neopixel], [temperature_sensor] (read only) and [thermistor].\n"
    "# Other sections (steppers, heaters, ...) are accepted but not used yet.\n"
    "[mcu]\n"
    "serial: uart\n"
    "\n"
    "[neopixel board_led]\n"
    "# BTT SKR Pico on-board RGB LED, as in Klipper's own SKR Pico config\n"
    "pin: gpio24\n"
    "chain_count: 1\n"
    "color_order: GRB\n"
    "initial_GREEN: 0.15\n"
    "\n"
    "# SKR Pico thermistor inputs: hotend TH0 = gpio27, bed TB = gpio26 (4.7k pullups on the board).\n"
    "# sensor_type must match your thermistor: Generic 3950, EPCOS 100K B57560G104F,\n"
    "# ATC Semitec 104GT-2, NTC 100K MGB18-104F39050L32, or your own [thermistor NAME] section.\n"
    "[temperature_sensor hotend]\n"
    "sensor_type: Generic 3950\n"
    "sensor_pin: gpio27\n"
    "\n"
    "[temperature_sensor bed]\n"
    "sensor_type: Generic 3950\n"
    "sensor_pin: gpio26\n";
// The machine's status (heaters, fans, motors, tests) as JSON, for /status.json.
static std::string s_machine_json;
static std::string s_prev_heater_log;   // the heater log of the last session (kept across a board restart)
static volatile bool s_printing;

extern "C" bool sf_printing(void)
{
    return s_printing;
}

// Reads the uploaded G-code file (storage.c) line by line, inflating it if the browser compressed it.
struct GcodeReader {
    bool open = false, compressed = false, done = false;
    uint32_t src_len = 0, src_pos = 0;
    tinfl_decompressor *d = nullptr;
    uint8_t *dict = nullptr;
    size_t dict_ofs = 0;
    uint8_t in[1024];
    size_t in_ofs = 0, in_avail = 0;
    std::string buf;
    size_t buf_pos = 0;

    void close()
    {
        free(d);
        free(dict);
        d = nullptr;
        dict = nullptr;
        open = false;
        buf.clear();
        buf_pos = 0;
    }
    bool start(std::string &name, std::string *err)
    {
        close();
        char nm[64];
        uint32_t orig = 0;
        bool z = false;
        if (!sf_store_info(nm, sizeof nm, &src_len, &orig, &z)) {
            if (err) *err = "no G-code file uploaded (Printer page: Upload)";
            return false;
        }
        name = nm;
        compressed = z;
        src_pos = 0;
        in_ofs = in_avail = 0;
        dict_ofs = 0;
        done = false;
        if (compressed) {
            d = (tinfl_decompressor *)malloc(sizeof(tinfl_decompressor));
            dict = (uint8_t *)malloc(TINFL_LZ_DICT_SIZE);
            if (!d || !dict) {
                close();
                if (err) *err = "not enough memory to unpack the file";
                return false;
            }
            tinfl_init(d);
        }
        open = true;
        return true;
    }
    bool more(std::string *err)
    {
        if (!compressed) {
            uint32_t n = src_len - src_pos < sizeof in ? src_len - src_pos : (uint32_t)sizeof in;
            if (n == 0) {
                done = true;
                return true;
            }
            if (!sf_store_read(src_pos, in, n)) {
                if (err) *err = "flash read failed";
                return false;
            }
            src_pos += n;
            buf.append((const char *)in, n);
            return true;
        }
        if (in_avail == 0 && src_pos < src_len) {
            uint32_t n = src_len - src_pos < sizeof in ? src_len - src_pos : (uint32_t)sizeof in;
            if (!sf_store_read(src_pos, in, n)) {
                if (err) *err = "flash read failed";
                return false;
            }
            src_pos += n;
            in_ofs = 0;
            in_avail = n;
        }
        size_t in_bytes = in_avail, out_bytes = TINFL_LZ_DICT_SIZE - dict_ofs;
        mz_uint32 flags = TINFL_FLAG_PARSE_ZLIB_HEADER | (src_pos < src_len ? TINFL_FLAG_HAS_MORE_INPUT : 0);
        tinfl_status st = tinfl_decompress(d, in + in_ofs, &in_bytes, dict, dict + dict_ofs, &out_bytes, flags);
        in_ofs += in_bytes;
        in_avail -= in_bytes;
        buf.append((const char *)dict + dict_ofs, out_bytes);
        dict_ofs = (dict_ofs + out_bytes) & (TINFL_LZ_DICT_SIZE - 1);
        if (st == TINFL_STATUS_DONE) {
            done = true;
        } else if (st < 0 || (st == TINFL_STATUS_NEEDS_MORE_INPUT && src_pos >= src_len && in_avail == 0)) {
            if (err) *err = "the stored file is damaged (could not unpack it)";
            return false;
        }
        return true;
    }
    bool line(std::string &out, bool &eof, std::string *err)
    {
        eof = false;
        if (!open) {
            eof = true;
            return true;
        }
        for (int guard = 0; guard < 200; guard++) {
            size_t nl = buf.find('\n', buf_pos);
            if (nl != std::string::npos || buf.size() - buf_pos > 512) {
                if (nl == std::string::npos) nl = buf_pos + 512;
                out.assign(buf, buf_pos, nl - buf_pos);
                buf_pos = nl + 1;
                if (buf_pos > 4096) {
                    buf.erase(0, buf_pos);
                    buf_pos = 0;
                }
                return true;
            }
            if (done) {
                if (buf_pos < buf.size()) {
                    out.assign(buf, buf_pos, std::string::npos);
                    buf_pos = buf.size();
                    return true;
                }
                eof = true;
                close();
                return true;
            }
            if (!more(err)) {
                close();
                return false;
            }
        }
        out.clear();   // nothing complete yet this time: an empty line, try again next call
        return true;
    }
    double progress() const
    {
        return src_len ? double(src_pos - (compressed ? in_avail : 0)) / double(src_len) : 0.0;
    }
};
static GcodeReader s_reader;
static sf_link_info_t s_info;
static sf_settings_t s_cfg;

const char *sf_link_state_name(sf_link_state_t st)
{
    switch (st) {
    case SF_LINK_OFF: return "off";
    case SF_LINK_SEARCHING: return "searching";
    case SF_LINK_OK: return "connected";
    default: return "error";
    }
}

void sf_link_get(sf_link_info_t *out)
{
    if (!s_mux) {
        memset(out, 0, sizeof *out);
        out->state = SF_LINK_OFF;
        return;
    }
    xSemaphoreTake(s_mux, portMAX_DELAY);
    *out = s_info;
    xSemaphoreGive(s_mux);
}

extern "C" size_t sf_printer_cfg_copy(char *out, size_t n)
{
    xSemaphoreTake(s_mux, portMAX_DELAY);
    size_t len = s_cfg_text.size() < n - 1 ? s_cfg_text.size() : n - 1;
    memcpy(out, s_cfg_text.data(), len);
    out[len] = 0;
    xSemaphoreGive(s_mux);
    return len;
}

extern "C" void sf_printer_cfg_report(char *out, size_t n)
{
    xSemaphoreTake(s_mux, portMAX_DELAY);
    strncpy(out, s_cfg_report.c_str(), n - 1);
    out[n - 1] = 0;
    xSemaphoreGive(s_mux);
}

extern "C" bool sf_printer_cfg_set(const char *text)
{
    size_t len = strlen(text);
    if (len > CFG_MAX) {
        return false;
    }
    nvs_handle_t h;
    if (nvs_open("skipper", NVS_READWRITE, &h) != ESP_OK) {
        return false;
    }
    bool ok = nvs_set_blob(h, "printer_cfg", text, len) == ESP_OK && nvs_commit(h) == ESP_OK;
    nvs_close(h);
    if (ok) {
        xSemaphoreTake(s_mux, portMAX_DELAY);
        s_cfg_text = text;
        s_cfg_reload = true;
        xSemaphoreGive(s_mux);
    }
    return ok;
}

extern "C" size_t sf_printer_cfg_default(char *out, size_t n)
{
    snprintf(out, n, "%s", ph::default_printer_cfg());
    return strlen(out);
}

extern "C" size_t sf_printer_cfg_default_tina2(char *out, size_t n)
{
    snprintf(out, n, "%s", ph::default_printer_cfg_tina2());
    return strlen(out);
}

extern "C" size_t sf_prev_heater_log(char *out, size_t n)
{
    xSemaphoreTake(s_mux, portMAX_DELAY);
    size_t len = s_prev_heater_log.size() < n - 1 ? s_prev_heater_log.size() : n - 1;
    memcpy(out, s_prev_heater_log.data(), len);
    out[len] = 0;
    xSemaphoreGive(s_mux);
    return len;
}

extern "C" char *sf_machine_json_dup(void)
{
    xSemaphoreTake(s_mux, portMAX_DELAY);
    char *out = nullptr;
    if (!s_machine_json.empty()) {
        out = (char *)malloc(s_machine_json.size() + 1);
        if (out) {
            memcpy(out, s_machine_json.c_str(), s_machine_json.size() + 1);
        }
    }
    xSemaphoreGive(s_mux);
    return out;
}

static void cfg_load(void)
{
    s_cfg_text = ph::default_printer_cfg();
    nvs_handle_t h;
    if (nvs_open("skipper", NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    size_t len = 0;
    if (nvs_get_blob(h, "printer_cfg", NULL, &len) == ESP_OK && len > 0 && len <= CFG_MAX) {
        std::string t(len, '\0');
        if (nvs_get_blob(h, "printer_cfg", &t[0], &len) == ESP_OK) {
            s_cfg_text = (t == OLD_DEFAULT_CFG_V1 || t == OLD_DEFAULT_CFG_V2) ? std::string(ph::default_printer_cfg()) : t;
        }
    }
    nvs_close(h);
}

extern "C" bool sf_console(const char *line, char *reply, size_t n, int timeout_ms)
{
    if (!s_con_slot || xSemaphoreTake(s_con_slot, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        snprintf(reply, n, "error: busy");
        return false;
    }
    xSemaphoreTake(s_con_done, 0);   // clear a stale signal
    xSemaphoreTake(s_mux, portMAX_DELAY);
    s_con_cmd = line;
    s_con_pending = true;
    xSemaphoreGive(s_mux);
    bool ok = xSemaphoreTake(s_con_done, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
    xSemaphoreTake(s_mux, portMAX_DELAY);
    if (ok) {
        snprintf(reply, n, "%s", s_con_reply.c_str());
    } else {
        s_con_pending = false;
        snprintf(reply, n, "error: the printer board is not connected (or did not answer in time)");
    }
    xSemaphoreGive(s_mux);
    xSemaphoreGive(s_con_slot);
    return ok;
}

static void update(void (*fn)(sf_link_info_t *, const void *), const void *arg)
{
    xSemaphoreTake(s_mux, portMAX_DELAY);
    fn(&s_info, arg);
    xSemaphoreGive(s_mux);
}

static void copy_str(char *dst, size_t n, const std::string &s)
{
    strncpy(dst, s.c_str(), n - 1);
    dst[n - 1] = 0;
}

// Unpack the zlib data dictionary with the inflater in the chip's ROM. The decompressor state is
// about 11 KB, so it lives on the heap: tinfl_decompress_mem_to_mem() would put it on the task
// stack, and that overflowed the link task's stack and corrupted the UART driver (field report
// 2026-10-11: assert in xTaskRemoveFromEventList from uart_write_bytes right after identify).
static bool inflate_dict(const ph::Bytes &in, std::string &out)
{
    tinfl_decompressor *d = (tinfl_decompressor *)malloc(sizeof(tinfl_decompressor));
    if (!d) {
        return false;
    }
    bool ok = false;
    for (size_t cap = 32 * 1024; cap <= 160 * 1024 && !ok; cap *= 2) {
        uint8_t *buf = (uint8_t *)malloc(cap);
        if (!buf) {
            break;
        }
        tinfl_init(d);
        size_t in_len = in.size(), out_len = cap;
        tinfl_status st = tinfl_decompress(d, in.data(), &in_len, buf, buf, &out_len,
                                           TINFL_FLAG_PARSE_ZLIB_HEADER | TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF);
        if (st == TINFL_STATUS_DONE) {
            out.assign((const char *)buf, out_len);
            ok = true;
        }
        free(buf);
        if (st != TINFL_STATUS_HAS_MORE_OUTPUT) {
            break;   // done, or a real error (a bigger buffer will not help)
        }
    }
    free(d);
    return ok;
}

// Raw byte diagnostics, touched only by the link task; copied into s_info after each attempt.
static uint32_t s_tx, s_rx;
static uint8_t s_tail[16];
static uint32_t s_tail_n;
static uint8_t s_last_frame[64];
static size_t s_last_frame_len;
static uint8_t s_win[160];
static size_t s_win_len;
static bool s_loopback;
static uint32_t s_echo_frames;   // received frames that start like the ones we send
static uint8_t s_prev;

static void note_rx(const uint8_t *p, size_t n)
{
    s_rx += n;
    for (size_t i = 0; i < n; i++) {
        s_tail[s_tail_n++ % sizeof s_tail] = p[i];
        if (s_win_len == sizeof s_win) {
            memmove(s_win, s_win + 1, sizeof s_win - 1);
            s_win_len--;
        }
        s_win[s_win_len++] = p[i];
        // A garbled echo still tends to keep our first two bytes: length, then 0x10|our sequence number.
        if (s_last_frame_len >= 2 && s_prev == s_last_frame[0] && p[i] == s_last_frame[1]) {
            s_echo_frames++;
        }
        s_prev = p[i];
    }
    // TX wired to RX (self-test, or a crossed pair of wires): our own frame comes straight back.
    if (s_last_frame_len && s_win_len >= s_last_frame_len) {
        for (size_t i = 0; i + s_last_frame_len <= s_win_len; i++) {
            if (memcmp(s_win + i, s_last_frame, s_last_frame_len) == 0) {
                s_loopback = true;
                break;
            }
        }
    }
}

static void publish_diag(const ph::KLink &link)
{
    struct { uint32_t tx, rx, res, crc; char hex[52]; int lb, echo; } d = {};
    d.echo = s_echo_frames >= 3;
    d.tx = s_tx;
    d.rx = s_rx;
    d.res = link.resyncs();
    d.crc = link.crc_errors();
    d.lb = s_loopback;
    size_t have = s_tail_n < sizeof s_tail ? s_tail_n : sizeof s_tail;
    char *o = d.hex;
    for (size_t i = 0; i < have; i++) {
        uint8_t b = s_tail[(s_tail_n - have + i) % sizeof s_tail];
        o += snprintf(o, sizeof d.hex - (size_t)(o - d.hex), "%02x ", b);
    }
    update([](sf_link_info_t *i, const void *a) {
        const auto *v = (const decltype(d) *)a;
        i->tx_bytes = v->tx;
        i->rx_bytes = v->rx;
        i->resyncs = v->res;
        i->crc_errors = v->crc;
        i->loopback = v->lb;
        i->echo_suspected = v->echo;
        memcpy(i->rx_hex, v->hex, sizeof i->rx_hex);
    }, &d);
    ESP_LOGW(TAG, "link bytes: sent %u, received %u, CRC errors %u, resyncs %u%s", (unsigned)d.tx, (unsigned)d.rx,
             (unsigned)d.crc, (unsigned)d.res, d.lb ? " - LOOPBACK: hearing our own transmission"
             : d.echo ? " - ECHO: garbled copies of our own requests are coming back" : "");
    if (d.rx) {
        ESP_LOGW(TAG, "last bytes received: %s", d.hex);
    }
}

// Copy the latest sensor readings into the info struct the web pages read.
static void publish_temps(const ph::BoardPlan &plan, bool cfg_ok)
{
    sf_temp_t t[SF_MAX_TEMPS] = {};
    int n = 0;
    if (cfg_ok) {
        for (const auto &s : plan.sensors) {
            if (n >= SF_MAX_TEMPS) {
                break;
            }
            copy_str(t[n].name, sizeof t[n].name, s.name);
            for (char *c = t[n].name; *c; c++) {   // the name goes into HTML and JSON: keep it plain
                if (strchr("<>&\"'\\", *c) || (unsigned char)*c < 0x20) {
                    *c = '_';
                }
            }
            copy_str(t[n].text, sizeof t[n].text, s.describe());
            t[n].c = (float)s.temp_c;
            t[n].state = s.state();
            n++;
        }
    }
    struct { sf_temp_t *t; int n; } a = {t, n};
    update([](sf_link_info_t *i, const void *p) {
        const auto *v = (const decltype(a) *)p;
        i->n_temps = v->n;
        memcpy(i->temps, v->t, sizeof i->temps);
    }, &a);
}

// The TX wire: driven by the UART, or let go (input, no pull) so a printer board whose own supply is
// off is not powered through its RX pin. The SKR Pico's RX has its own pull-up (Klipper
// src/rp2040/serial.c: gpio_peripheral(rx, 2, 1)), so the line still idles high while it is powered.
static void tx_drive(bool on)
{
    if (on) {
        uart_set_pin(PORT, s_cfg.uart_tx, s_cfg.uart_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        ESP_LOGW(TAG, "TX wire driven");
    } else {
        uart_wait_tx_done(PORT, pdMS_TO_TICKS(50));
        gpio_reset_pin((gpio_num_t)s_cfg.uart_tx);   // GPIO input, output off
        gpio_set_pull_mode((gpio_num_t)s_cfg.uart_tx, GPIO_FLOATING);
        ESP_LOGW(TAG, "TX wire released (not driven) while the board is away");
    }
}

// The board's TX line (our RX) while idle: 1 high, 0 held low the whole time (board off or hung).
static int rx_line(void)
{
    int high = 0;
    for (int k = 0; k < 20; k++) {
        high += gpio_get_level((gpio_num_t)s_cfg.uart_rx);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    uart_flush_input(PORT);   // what came in meanwhile (00 bytes from a low line) is not a reply
    return high ? 1 : 0;
}

static void serve_console(ph::BoardSession &bs, ph::KLink &link)
{
    if (!s_con_pending) {
        return;
    }
    std::string cmd;
    xSemaphoreTake(s_mux, portMAX_DELAY);
    cmd = s_con_cmd;
    xSemaphoreGive(s_mux);
    std::string reply = bs.machine() ? bs.machine()->command(cmd, link.host_time_s())
                        : bs.running() ? "error: the board is not configured (" + bs.cfg_msg() + ")"
                                       : "error: no printer board connected (" + bs.last_error() + ")";
    ESP_LOGW(TAG, "console: %s -> %.300s%s", cmd.c_str(), reply.c_str(), reply.size() > 300 ? "..." : "");
    xSemaphoreTake(s_mux, portMAX_DELAY);
    s_con_reply = reply;
    s_con_pending = false;
    xSemaphoreGive(s_mux);
    xSemaphoreGive(s_con_done);
}

static void link_task(void *arg)
{
    ph::KLinkIO io;
    io.write = [](const uint8_t *p, size_t n) {
        s_tx += n;
        if (n <= sizeof s_last_frame) {
            memcpy(s_last_frame, p, n);
            s_last_frame_len = n;
        }
        return uart_write_bytes(PORT, p, n) == (int)n;
    };
    io.read = [](uint8_t *p, size_t n, int ms) -> size_t {
        int got = uart_read_bytes(PORT, p, 1, pdMS_TO_TICKS(ms) ? pdMS_TO_TICKS(ms) : 1);
        if (got <= 0) {
            return 0;
        }
        size_t avail = 0;
        uart_get_buffered_data_len(PORT, &avail);
        if (avail > n - 1) {
            avail = n - 1;
        }
        int more = avail ? uart_read_bytes(PORT, p + 1, avail, 0) : 0;
        size_t total = 1 + (more > 0 ? (size_t)more : 0);
        note_rx(p, total);
        return total;
    };
    io.now_ms = []() { return (uint32_t)(esp_timer_get_time() / 1000); };
    io.inflate = inflate_dict;
    io.log = [](const std::string &s) { ESP_LOGW(TAG, "%s", s.c_str()); };
    io.now_us = []() { return (uint64_t)esp_timer_get_time(); };
    ph::KLink link(io);

    // Connection logic: ph::BoardSession (host/src/board_session.cpp), tested on the PC against
    // fake_mcu.py with power cycles, a hung board, a board that ignores reset, line noise.
    ph::SessionIO sio;
    static ph::BoardSession *s_bs = nullptr;
    sio.sleep_ms = [&link](int ms) {   // keep the console answering while waiting
        int64_t end = esp_timer_get_time() + (int64_t)ms * 1000;
        while (esp_timer_get_time() < end) {
            vTaskDelay(pdMS_TO_TICKS(50));
            if (s_bs && !s_bs->machine()) {
                serve_console(*s_bs, link);
            }
        }
    };
    sio.config_text = []() {
        xSemaphoreTake(s_mux, portMAX_DELAY);
        std::string t = s_cfg_text;
        s_cfg_reload = false;
        xSemaphoreGive(s_mux);
        return t;
    };
    sio.log = [](const std::string &s) { ESP_LOGW(TAG, "%s", s.c_str()); };
    sio.tx_drive = tx_drive;
    sio.rx_line = rx_line;
    sio.flush_input = []() { uart_flush_input(PORT); };

    ph::MachineIO mio;
    mio.log = [](const std::string &s) { ESP_LOGW(TAG, "%s", s.c_str()); };
    mio.config_text = []() {
        xSemaphoreTake(s_mux, portMAX_DELAY);
        std::string t = s_cfg_text;
        xSemaphoreGive(s_mux);
        return t;
    };
    mio.save_config = [](const std::string &t, std::string *e) {
        if (sf_printer_cfg_set(t.c_str())) {
            return true;
        }
        if (e) {
            *e = t.size() > CFG_MAX ? "printer.cfg would be over 4 KB" : "the chip's storage refused it";
        }
        return false;
    };
    mio.print_open = [](std::string &name, std::string *e) { return s_reader.start(name, e); };
    mio.print_read = [](std::string &line, bool &eof, std::string *e) { return s_reader.line(line, eof, e); };
    mio.print_progress = []() { return s_reader.progress(); };
    mio.largest_free = []() { return (size_t)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT); };

    ph::BoardSession bs(link, sio, mio);
    s_bs = &bs;
    bs.on_stop = [](ph::Machine &m) {
        s_printing = false;
        s_reader.close();
        std::string lg = m.heater_log_csv();
        if (lg.find('\n') + 1 < lg.size()) {   // more than the header line: keep it for /heater_log.csv
            xSemaphoreTake(s_mux, portMAX_DELAY);
            s_prev_heater_log.swap(lg);
            xSemaphoreGive(s_mux);
        }
    };
    bs.on_start = [](ph::BoardSession &b) {
        ESP_LOGW(TAG, "memory with the printer running: %u free, %u in one piece", (unsigned)esp_get_free_heap_size(),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        if (b.machine() && b.machine()->toolhead() && !b.machine()->toolhead()->ok()) {
            ESP_LOGW(TAG, "moves disabled: %s", b.machine()->toolhead()->why_not().c_str());
        }
        const ph::KInfo &k = b.info();
        update([](sf_link_info_t *i, const void *a) {
            const ph::KInfo &k = *(const ph::KInfo *)a;
            copy_str(i->version, sizeof i->version, k.version);
            copy_str(i->mcu, sizeof i->mcu, k.mcu);
            copy_str(i->pins, sizeof i->pins, k.pins);
            i->clock_freq = (uint32_t)k.clock_freq;
            i->dict_bytes = (uint32_t)k.dict_compressed;
        }, &k);
        struct { std::string msg; int ok; std::string led; } cs = {
            b.cfg_msg(), b.cfg_ok(), b.plan() && !b.plan()->neopixels.empty() ? b.plan()->neopixels[0].name : ""};
        update([](sf_link_info_t *i, const void *a) {
            const auto *v = (const decltype(cs) *)a;
            copy_str(i->cfg_status, sizeof i->cfg_status, v->msg);
            i->cfg_ok = v->ok;
            copy_str(i->first_led, sizeof i->first_led, v->ok ? v->led : std::string());
        }, &cs);
        // report what the printer.cfg check found
        std::string rep;
        if (b.plan()) {
            for (auto &e : b.plan()->errors) rep += "error: " + e + "\n";
            for (auto &n : b.plan()->notes) rep += "note: " + n + "\n";
        }
        xSemaphoreTake(s_mux, portMAX_DELAY);
        s_cfg_report = rep;
        xSemaphoreGive(s_mux);
    };

    bool was_running = false;
    uint32_t last_pub = 0;
    for (;;) {
        if (s_cfg_reload) {
            bs.reload();   // config_text() clears the flag when the session reads the new text
        }
        bs.step();
        serve_console(bs, link);
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        bool running = bs.running();
        if (running != was_running || (!running) || now - last_pub >= 500) {
            last_pub = now;
            struct { int run; uint32_t att; std::string err; int c, s; uint32_t e, hz, n; float ppm, rtt; } st = {
                running, bs.attempts(), bs.last_error(), bs.board_configured(), bs.board_shutdown(), link.crc_errors(),
                (uint32_t)(bs.clock().frequency() + 0.5), bs.clock().samples(), (float)bs.clock().drift_ppm(),
                (float)(bs.clock().min_rtt() * 1e3)};
            update([](sf_link_info_t *i, const void *a) {
                const auto *v = (const decltype(st) *)a;
                i->attempts = v->att;
                if (v->run) {
                    i->state = SF_LINK_OK;
                    i->last_error[0] = 0;
                    i->configured = v->c;
                    i->shutdown = v->s;
                    i->clk_hz = v->hz;
                    i->clk_samples = v->n;
                    i->drift_ppm = v->ppm;
                    i->rtt_ms = v->rtt;
                } else {
                    i->state = v->att <= 1 ? SF_LINK_SEARCHING : SF_LINK_ERROR;
                    copy_str(i->last_error, sizeof i->last_error, v->err);
                    i->cfg_ok = 0;
                    i->n_temps = 0;
                }
                i->crc_errors = v->e;
            }, &st);
            if (running && bs.plan()) {
                publish_temps(*bs.plan(), bs.cfg_ok());
            }
            std::string js = bs.machine() ? bs.machine()->status_json(link.host_time_s()) : std::string();
            xSemaphoreTake(s_mux, portMAX_DELAY);
            s_machine_json.swap(js);
            xSemaphoreGive(s_mux);
            s_printing = bs.machine() && bs.machine()->printing();
            if (!running) {
                publish_diag(link);   // after each failed attempt: what came over the wire
            }
        }
        if (running != was_running) {
            if (!running) {
                ESP_LOGW(TAG, "printer board lost: %s", bs.last_error().c_str());
            } else {
                ESP_LOGW(TAG, "printer board connected (connection %u)", (unsigned)bs.connects());
                ESP_LOGW(TAG, "link task stack: %u bytes never used (of %u)",
                         (unsigned)uxTaskGetStackHighWaterMark(NULL), (unsigned)LINK_STACK);
            }
            was_running = running;
        }
    }
}

extern "C" void sf_link_start(const sf_settings_t *s)
{
    s_cfg = *s;
    s_mux = xSemaphoreCreateMutex();
    s_con_slot = xSemaphoreCreateMutex();
    s_con_done = xSemaphoreCreateBinary();
    cfg_load();
    memset(&s_info, 0, sizeof s_info);
    s_info.state = SF_LINK_OFF;
    if (!s->link_enabled) {
        ESP_LOGW(TAG, "printer-board link is switched off (settings page)");
        return;
    }
    uart_config_t uc = {};
    uc.baud_rate = s->baud;
    uc.data_bits = UART_DATA_8_BITS;
    uc.parity = UART_PARITY_DISABLE;
    uc.stop_bits = UART_STOP_BITS_1;
    uc.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uc.source_clk = UART_SCLK_DEFAULT;
    if (uart_driver_install(PORT, 2048, 0, 0, NULL, 0) != ESP_OK || uart_param_config(PORT, &uc) != ESP_OK ||
        uart_set_pin(PORT, s->uart_tx, s->uart_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "could not set up the UART (TX GPIO %d, RX GPIO %d)", s->uart_tx, s->uart_rx);
        s_info.state = SF_LINK_ERROR;
        strcpy(s_info.last_error, "UART setup failed (check the GPIO numbers)");
        return;
    }
    // A defined idle level on our RX: a board that is powered drives it high; one that is switched off
    // clamps it low through its pin protection (the ~45 kohm pull-up cannot power it); an unconnected
    // wire or the loopback self-test reads high, so the ESP32 still tries to talk then.
    gpio_pullup_en((gpio_num_t)s->uart_rx);
    s_info.state = SF_LINK_SEARCHING;
    xTaskCreate(link_task, "link", LINK_STACK, NULL, 4, NULL);
}
