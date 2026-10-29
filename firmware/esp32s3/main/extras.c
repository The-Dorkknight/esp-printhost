// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Extras that live on the SD card (folder "skipper" on the card), in every build:
//   events.log    everything the firmware logs (board connects, shutdowns, prints, restarts and
//                 their reason), so a problem can be read afterwards; 1 MB, then events.1.log
//   report.log    only the important events (2026-10-27): start-ups and why, crashes, board connects
//                 and losses, shutdowns, refused late moves, prints, benchmark results; the same
//                 lines are kept in memory (GET /report.txt), so they can be read without a card
//   history.csv   one line per finished / cancelled / failed print
//   cfg/          the last 10 printer.cfg versions, saved before each change
// and, over HTTP:
//   GET  /history.json        the last 20 prints
//   GET  /thumb?name=x.gcode  the slicer's preview picture from inside the file (PNG / JPG)
//   GET  /sdls?d=skipper/cfg  a folder listing (only the card's top folder and skipper/...)
//   GET  /sdfile?f=skipper/events.log[&tail=65536]   a file as text
//   POST /time?t=<ms since 1970>   the browser's clock (the ESP32-S3 has no clock of its own)
// Writing to the card happens on a low-priority task every 2 s, never on the printer link task.
#include <dirent.h>
#include <stdarg.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mbedtls/base64.h"
#include "sf.h"

#define DIR_SK SF_SD_ROOT "/skipper"
#define DIR_CFG DIR_SK "/cfg"
#define EVENTS DIR_SK "/events.log"
#define EVENTS_OLD DIR_SK "/events.1.log"
#define HISTORY DIR_SK "/history.csv"
#define REPORT DIR_SK "/report.log"
#define REPORT_OLD DIR_SK "/report.1.log"
#define REPORT_MAX (256 * 1024)
#define EVENTS_MAX (1024 * 1024)
#define CFG_KEEP 10

static const char *TAG = "extras";

// ---------------------------------------------------------------- pending text for the card
#define PEND 2048   // 2 s of log; more than that is counted as "not saved"
static char s_pend[PEND];
static size_t s_pend_n;
static uint32_t s_dropped;
static char s_hist[4][300];
static int s_hist_n;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
// the important events: in memory (oldest lines dropped when full) and waiting for the card
#define REP_RAM 6144
static char s_rep[REP_RAM];
static size_t s_rep_n;
static char s_rep_pend[1536];
static size_t s_rep_pend_n;

bool sf_clock_set(void)
{
    return time(NULL) > 1704067200;   // after 2024-01-01: set from a browser
}

void sf_log_tee(const char *s, size_t n)
{
    if (strstr(s, "alive: uptime")) {   // the 5 s heartbeat would fill the log
        return;
    }
    portENTER_CRITICAL(&s_mux);
    if (s_pend_n + n <= PEND) {
        memcpy(s_pend + s_pend_n, s, n);
        s_pend_n += n;
    } else {
        s_dropped += (uint32_t)n;
    }
    portEXIT_CRITICAL(&s_mux);
}

static void csv_field(char *out, size_t n, const char *s)
{
    size_t k = 0;
    if (n < 3) {
        return;
    }
    out[k++] = '"';
    for (; *s && k + 3 < n; s++) {
        if (*s == '"') {
            out[k++] = '"';
        }
        out[k++] = (*s == '\n' || *s == '\r') ? ' ' : *s;
    }
    out[k++] = '"';
    out[k] = 0;
}

static void now_text(char *out, size_t n)
{
    if (!sf_clock_set()) {
        snprintf(out, n, "-");
        return;
    }
    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);
    strftime(out, n, "%Y-%m-%d %H:%M:%S", &tm);
}

// Called from any task, including app_main (3.5 KB of stack): the buffers are static (a mutex keeps
// callers apart), so it needs little stack itself. 2026-10-27: they were on the stack at first, and
// with vsnprintf and the log hook on top that is a suspect for the heap damage seen under QEMU.
void sf_report(const char *fmt, ...)
{
    static char body[1024], line[1088], when[24];
    static StaticSemaphore_t lock_buf;
    static SemaphoreHandle_t lock;
    if (!lock) {
        lock = xSemaphoreCreateMutexStatic(&lock_buf);   // first call: app_main, before other tasks report
    }
    xSemaphoreTake(lock, portMAX_DELAY);
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(body, sizeof body, fmt, ap);
    va_end(ap);
    for (char *c = body; *c; c++) {
        if (*c == '\n' || *c == '\r') {
            *c = ' ';
        }
    }
    ESP_LOGW("report", "%s", body);   // in the scrolling log too
    uint32_t up = (uint32_t)(esp_log_timestamp() / 1000);
    now_text(when, sizeof when);
    int n = snprintf(line, sizeof line, "[%s | up %uh%02um%02us] %s\n", when, (unsigned)(up / 3600),
                     (unsigned)(up / 60 % 60), (unsigned)(up % 60), body);
    if (n > 0) {
        size_t len = (size_t)n < sizeof line ? (size_t)n : sizeof line - 1;
        portENTER_CRITICAL(&s_mux);
        if (s_rep_n + len > REP_RAM) {   // drop the oldest whole lines
            size_t cut = s_rep_n + len - REP_RAM;
            while (cut < s_rep_n && s_rep[cut - 1] != '\n') {
                cut++;
            }
            if (cut > s_rep_n) {
                cut = s_rep_n;
            }
            memmove(s_rep, s_rep + cut, s_rep_n - cut);
            s_rep_n -= cut;
        }
        memcpy(s_rep + s_rep_n, line, len);
        s_rep_n += len;
        if (s_rep_pend_n + len <= sizeof s_rep_pend) {
            memcpy(s_rep_pend + s_rep_pend_n, line, len);
            s_rep_pend_n += len;
        }
        portEXIT_CRITICAL(&s_mux);
    }
    xSemaphoreGive(lock);
}

void sf_history_add(const char *file, const char *state, double seconds, long lines, const char *msg)
{
    char when[24], f[140], m[120];
    now_text(when, sizeof when);
    csv_field(f, sizeof f, file);
    csv_field(m, sizeof m, msg);
    portENTER_CRITICAL(&s_mux);
    if (s_hist_n < 4) {
        snprintf(s_hist[s_hist_n++], sizeof s_hist[0], "%s,%s,%s,%.0f,%ld,%s\n", when, f, state, seconds, lines, m);
    }
    portEXIT_CRITICAL(&s_mux);
}

static void ensure_dirs(void)
{
    mkdir(DIR_SK, 0775);
    mkdir(DIR_CFG, 0775);
}

static void flush_to_card(void)
{
    static char out[PEND];
    static char hist[4][300];
    static char rep[sizeof s_rep_pend];
    size_t n, rn;
    int hn;
    uint32_t dropped;
    portENTER_CRITICAL(&s_mux);
    rn = s_rep_pend_n;
    memcpy(rep, s_rep_pend, rn);
    s_rep_pend_n = 0;
    n = s_pend_n;
    memcpy(out, s_pend, n);
    s_pend_n = 0;
    hn = s_hist_n;
    memcpy(hist, s_hist, sizeof hist);
    s_hist_n = 0;
    dropped = s_dropped;
    s_dropped = 0;
    portEXIT_CRITICAL(&s_mux);
    if (!n && !hn && !rn) {
        return;
    }
    sf_sd_lock();
    if (sf_sd_mounted()) {
        ensure_dirs();
        if (rn) {
            struct stat st;
            if (stat(REPORT, &st) == 0 && st.st_size > REPORT_MAX) {
                unlink(REPORT_OLD);
                rename(REPORT, REPORT_OLD);
            }
            FILE *f = fopen(REPORT, "a");
            if (f) {
                fwrite(rep, 1, rn, f);
                fclose(f);
            }
        }
        if (n) {
            struct stat st;
            if (stat(EVENTS, &st) == 0 && st.st_size > EVENTS_MAX) {
                unlink(EVENTS_OLD);
                rename(EVENTS, EVENTS_OLD);
            }
            FILE *f = fopen(EVENTS, "a");
            if (f) {
                char when[24];
                now_text(when, sizeof when);
                if (when[0] != '-') {
                    fprintf(f, "--- %s\n", when);
                }
                if (dropped) {
                    fprintf(f, "(%u bytes of log not saved: too much at once)\n", (unsigned)dropped);
                }
                fwrite(out, 1, n, f);
                fclose(f);
            }
        }
        if (hn) {
            struct stat st;
            bool fresh = stat(HISTORY, &st) != 0;
            FILE *f = fopen(HISTORY, "a");
            if (f) {
                if (fresh) {
                    fputs("when,file,result,seconds,lines,message\n", f);
                }
                for (int i = 0; i < hn; i++) {
                    fputs(hist[i], f);
                }
                fclose(f);
            }
        }
    }
    sf_sd_unlock();
}

static void extras_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        flush_to_card();
    }
}

void sf_extras_start(void)
{
    xTaskCreate(extras_task, "sdlog", 4096, NULL, 1, NULL);
}

// ---------------------------------------------------------------- printer.cfg backups

static int cfg_number(const char *name)
{
    int k = 0;
    return sscanf(name, "printer-%d.cfg", &k) == 1 ? k : -1;
}

void sf_cfg_backup(const char *old_text)
{
    if (!old_text || !old_text[0]) {
        return;
    }
    sf_sd_lock();
    if (!sf_sd_mounted()) {
        sf_sd_unlock();
        return;
    }
    ensure_dirs();
    int lo = 1 << 30, hi = 0, count = 0;
    DIR *d = opendir(DIR_CFG);
    if (d) {
        struct dirent *e;
        while ((e = readdir(d)) != NULL) {
            int k = cfg_number(e->d_name);
            if (k >= 0) {
                count++;
                lo = k < lo ? k : lo;
                hi = k > hi ? k : hi;
            }
        }
        closedir(d);
    }
    char path[80];
    snprintf(path, sizeof path, DIR_CFG "/printer-%04d.cfg", hi + 1);
    FILE *f = fopen(path, "w");
    if (f) {
        fputs(old_text, f);
        fclose(f);
        ESP_LOGW(TAG, "previous printer.cfg kept on the SD card as skipper/cfg/printer-%04d.cfg", hi + 1);
    }
    for (int k = lo; count + 1 > CFG_KEEP && k <= hi; k++) {   // keep the newest 10
        snprintf(path, sizeof path, DIR_CFG "/printer-%04d.cfg", k);
        if (unlink(path) == 0) {
            count--;
        }
    }
    sf_sd_unlock();
}

// ---------------------------------------------------------------- HTTP helpers

static bool url_decode_inplace(char *s)
{
    char *o = s;
    for (char *i = s; *i; i++) {
        if (*i == '+') {
            *o++ = ' ';
        } else if (*i == '%' && i[1] && i[2]) {
            char h[3] = {i[1], i[2], 0};
            char *end;
            long v = strtol(h, &end, 16);
            if (*end) {
                return false;
            }
            *o++ = (char)v;
            i += 2;
        } else {
            *o++ = *i;
        }
    }
    *o = 0;
    return true;
}

static bool query(httpd_req_t *req, const char *key, char *out, size_t n)
{
    char q[400];
    if (httpd_req_get_url_query_str(req, q, sizeof q) != ESP_OK ||
        httpd_query_key_value(q, key, out, n) != ESP_OK) {
        out[0] = 0;
        return false;
    }
    return url_decode_inplace(out);
}

// A path on the card that may be read: the card's top folder (G-code files) or skipper/...
// No "..", no backslashes, no hidden files.
static bool safe_rel(const char *rel)
{
    if (!rel[0] || rel[0] == '/' || strstr(rel, "..") || strchr(rel, '\\') || rel[0] == '.') {
        return false;
    }
    const char *slash = strchr(rel, '/');
    if (slash && strncmp(rel, "skipper/", 8) != 0) {
        return false;
    }
    for (const char *c = rel; *c; c++) {
        if ((unsigned char)*c < 0x20) {
            return false;
        }
    }
    return true;
}

static void json_esc(char *out, size_t cap, size_t *pos, const char *s)
{
    if (*pos + 3 >= cap) {
        return;
    }
    out[(*pos)++] = '"';
    for (; *s && *pos + 8 < cap; s++) {
        unsigned char c = (unsigned char)*s;
        if (c == '"' || c == '\\') {
            out[(*pos)++] = '\\';
            out[(*pos)++] = (char)c;
        } else if (c < 0x20) {
            *pos += (size_t)snprintf(out + *pos, cap - *pos, "\\u%04x", c);
        } else {
            out[(*pos)++] = (char)c;
        }
    }
    out[(*pos)++] = '"';
    out[*pos] = 0;
}

// ---------------------------------------------------------------- GET /history.json

// (buffers from the heap per request, not static: the DECKHAND build needs its internal RAM)
#define TAILN 4096
#define OUTN 6144
static esp_err_t history_get(httpd_req_t *req)
{
    char *tail = malloc(TAILN), *out = malloc(OUTN);
    if (!tail || !out) {
        free(tail);
        free(out);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
    }
    size_t pos = (size_t)snprintf(out, OUTN, "{\"mounted\":%s,\"prints\":[", sf_sd_mounted() ? "true" : "false");
    size_t n = 0;
    sf_sd_lock();
    FILE *f = sf_sd_mounted() ? fopen(HISTORY, "r") : NULL;
    if (f) {
        fseek(f, 0, SEEK_END);
        long sz = ftell(f);
        long from = sz > (long)TAILN - 1 ? sz - (long)TAILN + 1 : 0;
        fseek(f, from, SEEK_SET);
        n = fread(tail, 1, TAILN - 1, f);
        fclose(f);
        if (from > 0) {   // drop the partial first line
            char *nl = memchr(tail, '\n', n);
            size_t cut = nl ? (size_t)(nl - tail) + 1 : n;
            memmove(tail, tail + cut, n - cut);
            n -= cut;
        }
    }
    sf_sd_unlock();
    tail[n] = 0;
    // newest first, up to 20, skipping the header line
    int shown = 0;
    char *end = tail + n;
    while (end > tail && shown < 20) {
        char *e = end;
        if (e > tail && e[-1] == '\n') {
            e--;
        }
        char *s = e;
        while (s > tail && s[-1] != '\n') {
            s--;
        }
        size_t len = (size_t)(e - s);
        end = s;
        if (!len || !strncmp(s, "when,", 5)) {
            continue;
        }
        char line[320];
        if (len >= sizeof line) {
            len = sizeof line - 1;
        }
        memcpy(line, s, len);
        line[len] = 0;
        if (pos + 400 > OUTN) {
            break;
        }
        pos += (size_t)snprintf(out + pos, OUTN - pos, "%s", shown ? "," : "");
        json_esc(out, OUTN, &pos, line);
        shown++;
    }
    pos += (size_t)snprintf(out + pos, OUTN - pos, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t r = httpd_resp_send(req, out, (ssize_t)pos);
    free(tail);
    free(out);
    return r;
}

// ---------------------------------------------------------------- GET /thumb?name=

// in C, the C++ finder in the host library (ph::find_gcode_thumbnail), via slicer_api.cpp
bool sf_find_thumbnail(const char *head, size_t n, int max_px, char *fmt, size_t fmt_n, char **b64, size_t *b64_n);

static esp_err_t thumb_get(httpd_req_t *req)
{
    char name[200];
    char path[300];
    if (!query(req, "name", name, sizeof name) || !sf_sd_path(name, path, sizeof path)) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no such file");
    }
    // the pictures sit near the start of the file; read more where PSRAM allows big ones
    // (without PSRAM only 32 KB: the printer link must never run short of memory for a picture)
    size_t want = sf_psram_bytes() ? 512 * 1024 : 32 * 1024;
    char *head = sf_psram_bytes() ? heap_caps_malloc(want + 1, MALLOC_CAP_SPIRAM) : NULL;
    if (!head) {
        want = 32 * 1024;
        head = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) > 96 * 1024 ? malloc(want + 1) : NULL;
    }
    if (!head) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
    }
    size_t n = 0;
    FILE *f = fopen(path, "rb");
    if (f) {
        n = fread(head, 1, want, f);
        fclose(f);
    }
    head[n] = 0;
    char fmt[8] = "";
    char *b64 = NULL;
    size_t b64n = 0;
    bool ok = n && sf_find_thumbnail(head, n, 400, fmt, sizeof fmt, &b64, &b64n);
    free(head);
    if (!ok) {
        httpd_resp_set_hdr(req, "Cache-Control", "max-age=600");
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no preview picture in this file");
    }
    size_t outn = 0;
    unsigned char *img = malloc(b64n * 3 / 4 + 4);
    esp_err_t r;
    if (img && mbedtls_base64_decode(img, b64n * 3 / 4 + 4, &outn, (const unsigned char *)b64, b64n) == 0) {
        httpd_resp_set_type(req, !strcmp(fmt, "jpg") ? "image/jpeg" : "image/png");
        httpd_resp_set_hdr(req, "Cache-Control", "max-age=600");
        r = httpd_resp_send(req, (const char *)img, (ssize_t)outn);
    } else {
        r = httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "bad preview picture");
    }
    free(img);
    free(b64);
    return r;
}

// ---------------------------------------------------------------- GET /sdls?d=  /sdfile?f=

static esp_err_t sdls_get(httpd_req_t *req)
{
    char rel[120];
    query(req, "d", rel, sizeof rel);
    if (strcmp(rel, "skipper") && strcmp(rel, "skipper/cfg")) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "only d=skipper or d=skipper/cfg");
    }
    const size_t cap = 4096;
    char *out = malloc(cap);
    if (!out) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
    }
    size_t pos = (size_t)snprintf(out, cap, "{\"files\":[");
    int k = 0;
    sf_sd_lock();
    char dir[160];
    snprintf(dir, sizeof dir, SF_SD_ROOT "/%s", rel);
    DIR *d = sf_sd_mounted() ? opendir(dir) : NULL;
    if (d) {
        struct dirent *e;
        while ((e = readdir(d)) != NULL && pos + 300 < cap) {
            if (e->d_type == DT_DIR || e->d_name[0] == '.') {
                continue;
            }
            char p[440];
            struct stat st;
            snprintf(p, sizeof p, "%s/%s", dir, e->d_name);
            if (stat(p, &st) != 0) {
                continue;
            }
            pos += (size_t)snprintf(out + pos, cap - pos, "%s{\"name\":", k++ ? "," : "");
            json_esc(out, cap, &pos, e->d_name);
            pos += (size_t)snprintf(out + pos, cap - pos, ",\"size\":%ld}", (long)st.st_size);
        }
        closedir(d);
    }
    sf_sd_unlock();
    pos += (size_t)snprintf(out + pos, cap - pos, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t r = httpd_resp_send(req, out, (ssize_t)pos);
    free(out);
    return r;
}

static esp_err_t sdfile_get(httpd_req_t *req)
{
    char rel[200], v[16];
    query(req, "f", rel, sizeof rel);
    if (!safe_rel(rel)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "not a readable file");
    }
    long tail = query(req, "tail", v, sizeof v) ? atol(v) : 0;
    char path[260];
    snprintf(path, sizeof path, SF_SD_ROOT "/%s", rel);
    char *buf = malloc(2048);
    if (!buf) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
    }
    sf_sd_lock();
    FILE *f = sf_sd_mounted() ? fopen(path, "rb") : NULL;
    if (!f) {
        sf_sd_unlock();
        free(buf);
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no such file on the SD card");
    }
    if (tail > 0) {
        fseek(f, 0, SEEK_END);
        long sz = ftell(f);
        fseek(f, sz > tail ? sz - tail : 0, SEEK_SET);
    }
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    size_t n;
    esp_err_t r = ESP_OK;
    while (r == ESP_OK && (n = fread(buf, 1, 2048, f)) > 0) {
        r = httpd_resp_send_chunk(req, buf, (ssize_t)n);
    }
    fclose(f);
    sf_sd_unlock();
    free(buf);
    return r == ESP_OK ? httpd_resp_send_chunk(req, NULL, 0) : r;
}

// ---------------------------------------------------------------- POST /time?t=

static esp_err_t time_post(httpd_req_t *req)
{
    char v[24];
    if (!query(req, "t", v, sizeof v)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "t=<milliseconds since 1970> expected");
    }
    long long ms = atoll(v);
    if (ms > 1704067200000LL && !sf_clock_set()) {
        struct timeval tv = {.tv_sec = (time_t)(ms / 1000), .tv_usec = (suseconds_t)((ms % 1000) * 1000)};
        settimeofday(&tv, NULL);
        char when[24];
        now_text(when, sizeof when);
        ESP_LOGW(TAG, "clock set from the browser: %s (UTC)", when);
    }
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, sf_clock_set() ? "ok" : "not set");
}

extern const char sf_build_time[];   // "Mmm dd yyyy hh:mm:ss", compiled on every build (main/CMakeLists.txt)

const char *sf_build_id(void)
{
    static char id[160];
    if (!id[0]) {
        char sha[9] = "";
        esp_app_get_elf_sha256(sha, sizeof sha);
        snprintf(id, sizeof id, "%s %s, %s, built %.17s, binary %s", SF_VERSION, sf_tier_title(), SF_MATHS,
                 sf_build_time, sha);
    }
    return id;
}

// GET /report.txt: the important events since start-up, with what is running (to paste into a report)
static esp_err_t report_get(httpd_req_t *req)
{
    static char buf[REP_RAM + 1];
    size_t n;
    portENTER_CRITICAL(&s_mux);
    n = s_rep_n;
    memcpy(buf, s_rep, n);
    portEXIT_CRITICAL(&s_mux);
    buf[n] = 0;
    char head[240];
    snprintf(head, sizeof head, "Skipper Flipper %s\n%s", sf_build_id(), n ? "" : "(nothing important since start-up)\n");
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr_chunk(req, head);
    httpd_resp_send_chunk(req, buf, (ssize_t)n);
    return httpd_resp_sendstr_chunk(req, NULL);
}

void sf_extras_register(void *httpd)
{
    static const httpd_uri_t r = {.uri = "/report.txt", .method = HTTP_GET, .handler = report_get};
    httpd_register_uri_handler((httpd_handle_t)httpd, &r);
    httpd_handle_t s = (httpd_handle_t)httpd;
    static const httpd_uri_t h = {.uri = "/history.json", .method = HTTP_GET, .handler = history_get};
    static const httpd_uri_t t = {.uri = "/thumb", .method = HTTP_GET, .handler = thumb_get};
    static const httpd_uri_t l = {.uri = "/sdls", .method = HTTP_GET, .handler = sdls_get};
    static const httpd_uri_t f = {.uri = "/sdfile", .method = HTTP_GET, .handler = sdfile_get};
    static const httpd_uri_t c = {.uri = "/time", .method = HTTP_POST, .handler = time_post};
    httpd_register_uri_handler(s, &h);
    httpd_register_uri_handler(s, &t);
    httpd_register_uri_handler(s, &l);
    httpd_register_uri_handler(s, &f);
    httpd_register_uri_handler(s, &c);
}
