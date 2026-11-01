// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// UART link to the printer board running Klipper firmware built for serial.
// Uses ph::KLink from host/ (the same code tested on the PC against fake_mcu.py).
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <sys/stat.h>
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
#include "sdkconfig.h"
#include "sf.h"
#include "esp_cpu.h"
#if CONFIG_SKIPPER_PRO
#include "ph/printbench.h"
#include "../../../host/test/skr_pico_dict.h"   // the SKR Pico's data dictionary, for the board-less benchmark
#endif

static const char *TAG = "link";
static const uart_port_t PORT = UART_NUM_1;
static const uint32_t LINK_STACK = 16384;   // dictionary parsing builds big C++ structures
// The core the printer link (and SF_BENCH) run on: 1, away from WiFi. (QEMU test builds are
// single-core, CONFIG_FREERTOS_UNICORE: under QEMU 9.0 every heap walk from core 1 finds the region
// at 0x3FCE9710 damaged, 2026-10-27, also with the unchanged earlier firmware; real boards are fine.)
static const int LINK_CORE = 1;
static SemaphoreHandle_t s_mux;
// printer.cfg text (stored in NVS), the last check's report, and a reload request.
static std::string s_cfg_text, s_cfg_report;
static volatile bool s_cfg_reload;
// One console command at a time, run by the link task (only it talks to the UART).
static SemaphoreHandle_t s_con_slot, s_con_done;
static std::string s_con_cmd, s_con_reply;
static volatile bool s_con_pending;
static const size_t CFG_MAX_NVS = 4096;   // where older builds kept it
// Largest printer.cfg: CONFIG_SKIPPER_CFG_KB at the end of the gcode partition (storage.c), else 4 KB.
static size_t cfg_max(void)
{
    size_t m = sf_cfgstore_max();
    return m ? m : CFG_MAX_NVS;
}
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
// Moonraker-shaped status for the Pasta Inspector dongle's /websocket (moonraker_ws.cpp). Built on the
// link task, at most every 200 ms, and only while a websocket client has subscribed (sf_moon_want).
static std::string s_moon_json, s_moon_objs;
static unsigned s_moon_rev;
static double s_moon_time;
static volatile int s_moon_want;
extern "C" void sf_moon_want(int on)
{
    s_moon_want = on;
}
static std::string s_prev_heater_log;   // the heater log of the last session (kept across a board restart)
static volatile bool s_printing;

extern "C" bool sf_printing(void)
{
    return s_printing;
}

// {"eventtime":..,"rev":N,"status":{...}}; exclude_object.objects is put in only when the object list
// changed since `have_rev` (-1: always). NULL when there is no printer state yet. free() it.
extern "C" char *sf_moon_status_dup(long have_rev)
{
    if (!s_mux) return NULL;
    char *out = NULL;
    xSemaphoreTake(s_mux, portMAX_DELAY);
    if (!s_moon_json.empty()) {
        std::string st = s_moon_json;
        if (have_rev != (long)s_moon_rev) {
            size_t k = st.rfind("\"exclude_object\":{");
            if (k != std::string::npos) st.insert(k + 18, "\"objects\":" + (s_moon_objs.empty() ? std::string("[]") : s_moon_objs) + ",");
        }
        char head[96];
        snprintf(head, sizeof head, "{\"eventtime\":%.3f,\"rev\":%u,\"status\":", s_moon_time, s_moon_rev);
        std::string all = std::string(head) + st + "}";
        out = (char *)malloc(all.size() + 1);
        if (out) memcpy(out, all.c_str(), all.size() + 1);
    }
    xSemaphoreGive(s_mux);
    return out;
}

// Reads a G-code file line by line: either the one in the flash partition (storage.c, inflated if the
// browser compressed it) or a plain-text file on the SD card (sdcard.c).
struct GcodeReader {
    bool open = false, compressed = false, done = false;
    FILE *fp = nullptr;   // SD card file, or nullptr for the flash partition
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
        if (fp) {
            fclose(fp);
            fp = nullptr;
        }
        d = nullptr;
        dict = nullptr;
        open = false;
        buf.clear();
        buf_pos = 0;
    }
    bool start(std::string &name, std::string *err)
    {
        close();
        if (!name.empty()) {   // a named file: from the SD card
            if (!sf_sd_mounted()) {
                if (err) *err = std::string("no SD card (") + sf_sd_error() + ")";
                return false;
            }
            char path[300];
            struct stat st;
            if (!sf_sd_path(name.c_str(), path, sizeof path) || stat(path, &st) != 0) {
                if (err) *err = "no such file on the SD card: " + name;
                return false;
            }
            fp = fopen(path, "rb");
            if (!fp) {
                if (err) *err = "could not open " + name + " on the SD card";
                return false;
            }
            src_len = (uint32_t)st.st_size;
            src_pos = 0;
            compressed = false;
            done = false;
            in_ofs = in_avail = 0;
            open = true;
            return true;
        }
        char nm[64];
        uint32_t orig = 0;
        bool z = false;
        if (!sf_store_info(nm, sizeof nm, &src_len, &orig, &z)) {
            if (err) *err = sf_sd_mounted() ? "which file? Pick one on the Printer page, or SDCARD_PRINT_FILE FILENAME=name.gcode (M20 lists them)"
                                            : "no G-code file uploaded (Printer page: Upload)";
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
        if (fp) {
            size_t n = fread(in, 1, sizeof in, fp);
            if (n == 0) {
                if (ferror(fp)) {
                    if (err) *err = "SD card read failed (card removed?)";
                    return false;
                }
                done = true;
                return true;
            }
            src_pos += (uint32_t)n;
            buf.append((const char *)in, n);
            return true;
        }
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
static sf_display_t s_disp;   // for the OLED, under s_mux
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

void sf_display_get(sf_display_t *out)
{
    if (!s_mux) {
        memset(out, 0, sizeof *out);
        return;
    }
    xSemaphoreTake(s_mux, portMAX_DELAY);
    *out = s_disp;
    xSemaphoreGive(s_mux);
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

extern "C" bool sf_printer_cfg_set(const char *text_in)
{
    std::string t;   // without carriage returns (a browser sends CRLF line ends)
    t.reserve(strlen(text_in));
    for (const char *c = text_in; *c; c++)
        if (*c != '\r') t += *c;
    const char *text = t.c_str();
    size_t len = t.size();
    if (len > cfg_max()) {
        return false;
    }
    bool ok;
    if (sf_cfgstore_max()) {
        ok = sf_cfgstore_put(text, len);
        if (ok) {   // the old NVS copy is not needed any more (frees NVS space)
            nvs_handle_t h;
            if (nvs_open("skipper", NVS_READWRITE, &h) == ESP_OK) {
                if (nvs_erase_key(h, "printer_cfg") == ESP_OK) nvs_commit(h);
                nvs_close(h);
            }
        }
    } else {
        nvs_handle_t h;
        if (nvs_open("skipper", NVS_READWRITE, &h) != ESP_OK) {
            return false;
        }
        ok = nvs_set_blob(h, "printer_cfg", text, len) == ESP_OK && nvs_commit(h) == ESP_OK;
        nvs_close(h);
    }
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

extern "C" size_t sf_printer_cfg_default_ender3(char *out, size_t n)
{
    snprintf(out, n, "%s", ph::default_printer_cfg_ender3());
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
    size_t m = sf_cfgstore_max();
    if (m) {   // since 2026-10-27: the end of the gcode partition
        std::string t(m + 1, '\0');
        int n = sf_cfgstore_get(&t[0], t.size());
        if (n > 0) {
            t.resize((size_t)n);
            s_cfg_text = t;
            return;
        }
    }
    nvs_handle_t h;   // older builds: NVS (moved over the first time it is read)
    if (nvs_open("skipper", NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    size_t len = 0;
    if (nvs_get_blob(h, "printer_cfg", NULL, &len) == ESP_OK && len > 0 && len <= CFG_MAX_NVS) {
        std::string t(len, '\0');
        if (nvs_get_blob(h, "printer_cfg", &t[0], &len) == ESP_OK) {
            s_cfg_text = (t == OLD_DEFAULT_CFG_V1 || t == OLD_DEFAULT_CFG_V2) ? std::string(ph::default_printer_cfg()) : t;
            if (m && sf_cfgstore_put(s_cfg_text.data(), s_cfg_text.size()))
                ESP_LOGW(TAG, "printer.cfg moved from NVS to the gcode partition (now up to %u KB)", (unsigned)(m / 1024));
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
bool inflate_dict(const ph::Bytes &in, std::string &out)   // also usb_accel.cpp
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

// The CPU cycle counter as a 64-bit count (it is 32 bits and wraps every 18 s at 240 MHz; it is read
// many times a second, from tasks on core 1 only, so a wrap is never missed). For SF_BENCH.
static uint64_t perf_cycles(void)
{
    static uint32_t last;
    static uint64_t high;
    uint32_t c = esp_cpu_get_cycle_count();
    if (c < last) {
        high += (uint64_t)1 << 32;
    }
    last = c;
    return high | c;
}
static const double PERF_HZ = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1e6;

static void report_cb(const std::string &s) { sf_report("%s", s.c_str()); }

#if CONFIG_SKIPPER_PRO
// ---- SF_BENCH with no printer board (2026-10-27): the whole motion pipeline on the ESP32 alone,
// against ph::FakeBoard (commands encoded for the SKR Pico, then dropped), in a task of its own on
// core 1 while the link has no board.
static volatile bool s_fbench_running, s_fbench_making, s_fbench_reported;
static std::string s_fbench_report, s_fbench_file;

// FILE=builtin: the benchmark file made line by line in memory (no SD card needed; the file reading
// is then not part of the timing). Any other name: the normal reader (SD card or flash).
static ph::BenchGcode *s_builtin;
static long s_builtin_n;
static bool file_open(std::string &name, std::string *e)
{
    delete s_builtin;
    s_builtin = nullptr;
    if (name == "builtin") {
        s_builtin = new ph::BenchGcode();
        s_builtin_n = 0;
        return true;
    }
    return s_reader.start(name, e);
}
static bool file_read(std::string &line, bool &eof, std::string *e)
{
    if (!s_builtin) return s_reader.line(line, eof, e);
    eof = !s_builtin->next(line);
    s_builtin_n++;
    return true;
}
static double file_progress()
{
    return s_builtin ? std::min(1.0, s_builtin_n / 9617.0) : s_reader.progress();
}

// The benchmark file on the SD card, if it is not there yet (a few seconds; a task of its own).
static void make_bench_file_task(void *)
{
    char path[64];
    snprintf(path, sizeof path, "%s/sf-bench.gcode", SF_SD_ROOT);
    sf_sd_lock();
    FILE *f = fopen(path, "w");
    bool ok = f != nullptr;
    if (f) {
        ph::BenchGcode g;
        std::string l;
        while (ok && g.next(l)) {
            l += '\n';
            ok = fwrite(l.data(), 1, l.size(), f) == l.size();
        }
        ok = fclose(f) == 0 && ok;
    }
    sf_sd_unlock();
    if (!ok) unlink(path);
    sf_report("benchmark file sf-bench.gcode %s the SD card", ok ? "written to" : "could NOT be written to");
    s_fbench_making = false;
    vTaskDelete(NULL);
}

// "" if the file can be read; else the reply (and it is being made, if it can be)
static std::string bench_file_check(const std::string &cmd)
{
    if (cmd.find("FILE=") != std::string::npos) return "";   // the user's choice
    if (!sf_sd_mounted()) return "";                          // Machine says what is missing
    char path[64];
    struct stat st;
    snprintf(path, sizeof path, "%s/sf-bench.gcode", SF_SD_ROOT);
    if (stat(path, &st) == 0) return "";
    if (!s_fbench_making) {
        s_fbench_making = true;
        xTaskCreate(make_bench_file_task, "benchfile", 6144, NULL, 2, NULL);
    }
    return "ok writing the benchmark file sf-bench.gcode to the SD card (a few seconds): send SF_BENCH again then";
}

static void fake_bench_task(void *)
{
    auto now = []() { return (double)esp_timer_get_time() * 1e-6; };
    std::string cfg;
    xSemaphoreTake(s_mux, portMAX_DELAY);
    cfg = s_cfg_text;
    xSemaphoreGive(s_mux);
    std::string err, report;
#if CONFIG_SKIPPER_BENCH_AT_BOOT
    ESP_LOGW(TAG, "benchmark: heap %s", heap_caps_check_integrity_all(true) ? "intact" : "DAMAGED");
#endif
    ESP_LOGW(TAG, "benchmark: memory before: %u free, %u in one piece, %u internal",
             (unsigned)esp_get_free_heap_size(), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    {
        ph::MsgProto mp;
        if (!mp.load_dictionary(SKR_PICO_DICT, &err)) {
            report = "benchmark: the built-in SKR Pico dictionary did not load: " + err;
        } else {
            ph::BoardPlan plan = ph::build_board_plan(cfg, mp, 12e6);
            plan.move_count = 1024;
            if (!plan.errors.empty()) {
                report = "benchmark: printer.cfg has errors: " + plan.errors[0];
            } else {
                ph::FakeBoard fb(mp);
                ph::MachineIO io = fb.io(now);
                io.log = [](const std::string &s) { ESP_LOGW(TAG, "%s", s.c_str()); };
                io.print_open = file_open;
                io.print_read = file_read;
                io.print_progress = file_progress;
                io.largest_free = []() { return (size_t)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT); };
                io.lookahead = sf_lookahead_max();
                io.perf_now = perf_cycles;
                io.perf_hz = PERF_HZ;
                s_fbench_reported = false;
                io.build_id = sf_build_id();
                io.report = [](const std::string &s) {
                    sf_report("%s", s.c_str());
                    s_fbench_reported = true;
                };
                ph::Machine m(plan, mp, io);
                m.start(now());
                for (int i = 0; i < 20; i++) {   // settle (0.2 s)
                    m.tick(now());
                    vTaskDelay(1);
                }
                ESP_LOGW(TAG, "benchmark: memory with the machine set up: %u free, %u internal",
                         (unsigned)esp_get_free_heap_size(), (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
                std::string r = m.command(s_fbench_file, now());
                if (r.rfind("ok benchmark", 0) != 0) {
                    report = "benchmark (no printer board): " + r;
                } else {
                    while (m.bench_running()) {
                        m.tick(now());
                        vTaskDelay(1);
                    }
                    report = m.bench_report();
                }
                s_reader.close();
            }
        }
    }
    if (!s_fbench_reported) sf_report("%s", report.c_str());
    xSemaphoreTake(s_mux, portMAX_DELAY);
    s_fbench_report = report;
    xSemaphoreGive(s_mux);
    s_fbench_running = false;
    vTaskDelete(NULL);
}

// SF_BENCH / SF_BENCH_RESULT while no board is connected: "" if cmd is neither
static std::string fake_bench_console(const std::string &cmd)
{
    std::string up = cmd.substr(0, cmd.find(' '));
    for (auto &c : up) c = (char)toupper((unsigned char)c);
    if (up == "SF_BENCH_RESULT") {
        if (s_fbench_running) return "ok the benchmark is still running";
        xSemaphoreTake(s_mux, portMAX_DELAY);
        std::string r = s_fbench_report;
        xSemaphoreGive(s_mux);
        return r.empty() ? "ok no benchmark has run yet (SF_BENCH)" : "ok " + r;
    }
    if (up != "SF_BENCH") return "";
    if (s_fbench_running) return "error: the benchmark is already running (SF_BENCH_RESULT)";
    std::string fc = bench_file_check(cmd);
    if (!fc.empty()) return fc;
    s_fbench_file = cmd;
    s_fbench_running = true;
#if CONFIG_FREERTOS_UNICORE
    if (xTaskCreate(fake_bench_task, "bench", 16384, NULL, 10, NULL) != pdPASS) {
#else
    if (xTaskCreatePinnedToCore(fake_bench_task, "bench", 16384, NULL, 10, NULL, LINK_CORE) != pdPASS) {
#endif
        s_fbench_running = false;
        return "error: not enough memory to start the benchmark";
    }
    return "ok benchmark running with no printer board (the ESP32 alone: commands encoded, not sent), " SF_MATHS
           ". About 90 s; SF_BENCH_RESULT, or the important-events log, for the numbers.";
}
#endif

static void serve_console(ph::BoardSession &bs, ph::KLink &link)
{
    if (!s_con_pending) {
        return;
    }
    std::string cmd;
    xSemaphoreTake(s_mux, portMAX_DELAY);
    cmd = s_con_cmd;
    xSemaphoreGive(s_mux);
    std::string reply;
#if CONFIG_SKIPPER_PRO
    if (!bs.machine()) {
        reply = fake_bench_console(cmd);
    } else if (cmd.rfind("SF_BENCH", 0) == 0 || cmd.rfind("sf_bench", 0) == 0) {
        reply = cmd.find("RESULT") == std::string::npos && cmd.find("result") == std::string::npos ? bench_file_check(cmd) : "";
    }
#endif
    if (reply.empty()) {
        reply = bs.machine() ? bs.machine()->command(cmd, link.host_time_s())
                : bs.running() ? "error: the board is not configured (" + bs.cfg_msg() + ")"
                               : "error: no printer board connected (" + bs.last_error() + ")";
    }
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
            *e = t.size() > cfg_max() ? "printer.cfg would be too big (" + std::to_string(cfg_max() / 1024) + " KB max)" : "the chip's storage refused it";
        }
        return false;
    };
#if CONFIG_SKIPPER_PRO
    mio.print_open = file_open;   // also FILE=builtin for SF_BENCH
#else
    mio.print_open = [](std::string &name, std::string *e) { return s_reader.start(name, e); };
#endif
    mio.list_files = [](std::vector<std::pair<std::string, uint64_t>> &files, std::string *e) {
        if (!sf_sd_mounted()) {
            if (e) *e = std::string("no SD card (") + sf_sd_error() + ")";
            return false;
        }
        files.clear();
        sf_sd_list([](const char *n, uint64_t sz, void *a) {
            ((std::vector<std::pair<std::string, uint64_t>> *)a)->push_back({n, sz});
        }, &files);
        return true;
    };
#if CONFIG_SKIPPER_PRO
    mio.print_read = file_read;
    mio.print_progress = file_progress;
#else
    mio.print_read = [](std::string &line, bool &eof, std::string *e) { return s_reader.line(line, eof, e); };
    mio.print_progress = []() { return s_reader.progress(); };
#endif
    mio.largest_free = []() { return (size_t)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT); };
    mio.lookahead = sf_lookahead_max();   // 32, or 256 / 512 on CAPTAIN / SKIPPER with PSRAM
    mio.perf_now = perf_cycles;           // SF_BENCH timing (2026-10-27)
    mio.perf_hz = PERF_HZ;
    mio.report = report_cb;
    mio.build_id = sf_build_id();
#if CONFIG_SKIPPER_PRO
    mio.host_adc = [](int gpio, double &r) {   // [temperature_sensor] on esp32:gpioN (hostadc.c)
        float f = 0;
        if (!sf_host_adc(gpio, &f)) return false;
        r = f;
        return true;
    };
    mio.accel_record = [](int mode, int axis, std::string *e) {
        char b[160] = "";
        bool ok = sf_accel_record(mode, axis, b, sizeof b);
        if (!ok && e) *e = b;
        return ok;
    };
#endif
    mio.print_ended = [](const std::string &file, const std::string &state, double secs, long lines,
                         const std::string &msg) {
        sf_history_add(file.c_str(), state.c_str(), secs, lines, msg.c_str());
        sf_report("print %s: %s after %.0f s, %ld lines%s%s", state.c_str(), file.c_str(), secs, lines,
                  msg.empty() ? "" : ": ", msg.c_str());
    };

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

#if CONFIG_SKIPPER_BENCH_AT_BOOT && CONFIG_SKIPPER_PRO
    // QEMU test builds: the board-less print benchmark from the built-in file, once at start-up
    ESP_LOGW(TAG, "start-up: %s", fake_bench_console("SF_BENCH FILE=builtin").c_str());
#endif
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
            sf_display_t dp = {};
            if (running && bs.machine()) {
                ph::Machine::DisplayInfo di = bs.machine()->display_info(link.host_time_s());
                dp.have_ext = di.have_ext, dp.have_bed = di.have_bed;
                dp.ext_temp = (float)di.ext_temp, dp.ext_target = (float)di.ext_target;
                dp.bed_temp = (float)di.bed_temp, dp.bed_target = (float)di.bed_target;
                strlcpy(dp.state, di.state.c_str(), sizeof dp.state);
                strlcpy(dp.activity, di.activity.c_str(), sizeof dp.activity);
                dp.progress = bs.machine()->printing() ? (float)s_reader.progress() : 0.f;
            }
            std::string js = bs.machine() ? bs.machine()->status_json(link.host_time_s()) : std::string();
            xSemaphoreTake(s_mux, portMAX_DELAY);
            s_machine_json.swap(js);
            s_disp = dp;
            xSemaphoreGive(s_mux);
            s_printing = bs.machine() && bs.machine()->printing();
            if (!running) {
                publish_diag(link);   // after each failed attempt: what came over the wire
            }
        }
#if CONFIG_SKIPPER_PRO
        if (s_moon_want) {
            static uint32_t last_moon;
            uint32_t mnow = (uint32_t)(esp_timer_get_time() / 1000);
            if (mnow - last_moon >= 200) {
                last_moon = mnow;
                std::string mj, mo;
                unsigned rev = s_moon_rev;
                if (running && bs.machine()) {
                    mj = bs.machine()->moonraker_status_json(link.host_time_s());
                    rev = bs.machine()->objects_rev();
                    if (rev != s_moon_rev || s_moon_objs.empty()) mo = bs.machine()->moonraker_objects_json();
                }
                xSemaphoreTake(s_mux, portMAX_DELAY);
                s_moon_json.swap(mj);
                if (!mo.empty()) s_moon_objs.swap(mo), s_moon_rev = rev;
                s_moon_time = link.host_time_s();
                xSemaphoreGive(s_mux);
            }
        }
#endif
        if (running != was_running) {
            if (!running) {
                ESP_LOGW(TAG, "printer board lost: %s", bs.last_error().c_str());
                sf_report("printer board lost: %s", bs.last_error().c_str());
            } else {
                ESP_LOGW(TAG, "printer board connected (connection %u)", (unsigned)bs.connects());
                sf_report("printer board connected (connection %u)%s%s", (unsigned)bs.connects(),
                          bs.cfg_ok() ? "" : "; printer.cfg: ", bs.cfg_ok() ? "" : bs.cfg_msg().c_str());
                ESP_LOGW(TAG, "link task stack: %u bytes never used (of %u)",
                         (unsigned)uxTaskGetStackHighWaterMark(NULL), (unsigned)LINK_STACK);
            }
            was_running = running;
        }
    }
}

extern "C" void sf_link_start(const sf_settings_t *s)
{
    sf_host_adc_settings(s);
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
    // The printer link times every step: it must never wait behind the web server. 2026-10-27, Tina 2:
    // "Timer too close" twice while the Printer page loaded a G-code preview (a 512 KB read and scan
    // on the web server, priority 5, with WiFi busy sending): the link task (then priority 4, either
    // core) was held off long enough for planned steps to go stale. Now: its own core (1; WiFi and
    // the TCP/IP task stay on core 0) and a priority above the web server.
#if CONFIG_FREERTOS_UNICORE
    xTaskCreate(link_task, "link", LINK_STACK, NULL, 10, NULL);
#else
    xTaskCreatePinnedToCore(link_task, "link", LINK_STACK, NULL, 10, NULL, LINK_CORE);
#endif
}
