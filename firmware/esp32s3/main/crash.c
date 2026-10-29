// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// What happened before the last crash, kept across the restart, in every build:
//   - "last words": the last 2 KB the firmware logged, in RTC memory that a panic, watchdog or
//     software restart does not clear;
//   - a core dump in its own flash partition (ESP-IDF), from which the panic reason ("Stack canary
//     watchpoint triggered (led)", ...), the crashed task, its program counter and backtrace are read.
// After a crash the status page says so with a link to GET /crash (all of it as text) and
// GET /coredump.bin (the raw core dump, for `idf.py coredump-info` with the build's ELF from
// firmware/prebuilt-esp32s3/symbols/). With an SD card in, the report is also saved as
// skipper/crash-NNNN.txt.
// 2026-10-20: added after an ESP32-S3 left alone for 20 minutes was found blinking red, then showed
// "last restart: CRASH (panic)"; the USB log that holds the panic message was not connected.
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "esp_attr.h"
#include "esp_core_dump.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "sf.h"

static const char *TAG = "crash";
#define LW_SIZE 2048
#define LW_MAGIC 0x4C415354u   // "LAST"

typedef struct {
    uint32_t magic;
    uint32_t total;   // bytes ever written since the last boot
    char buf[LW_SIZE];
} last_words_t;

static RTC_NOINIT_ATTR last_words_t s_lw;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static char *s_report;          // the report about the previous run's crash, or NULL
static char s_short[200];       // one line for the status page

void sf_lastwords_add(const char *s, size_t n)
{
    if (s_lw.magic != LW_MAGIC) {
        return;   // not started yet (sf_crash_boot runs first thing)
    }
    portENTER_CRITICAL(&s_mux);
    for (size_t i = 0; i < n; i++) {
        s_lw.buf[s_lw.total % LW_SIZE] = s[i];
        s_lw.total++;
    }
    portEXIT_CRITICAL(&s_mux);
}

static bool crashed(esp_reset_reason_t r)
{
    return r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT || r == ESP_RST_WDT ||
           r == ESP_RST_BROWNOUT;
}

static const char *exc_cause_text(uint32_t c)
{
    switch (c) {
    case 0: return "illegal instruction";
    case 2: return "instruction fetch error";
    case 3: return "load/store error";
    case 6: return "integer divide by zero";
    case 9: return "unaligned load/store";
    case 28: return "load from a bad address (NULL pointer?)";
    case 29: return "store to a bad address (NULL pointer?)";
    default: return "";
    }
}

void sf_crash_boot(void)
{
    esp_reset_reason_t r = esp_reset_reason();
    bool have_lw = s_lw.magic == LW_MAGIC && s_lw.total > 0;
    if (crashed(r)) {
        size_t cap = LW_SIZE + 2048;
        s_report = malloc(cap);
        if (s_report) {
            size_t pos = 0;
            pos += (size_t)snprintf(s_report + pos, cap - pos, "Previous run ended with: %s\n",
                                    r == ESP_RST_BROWNOUT ? "BROWNOUT (the 3.3 V supply dipped: power, not software)"
                                    : r == ESP_RST_PANIC  ? "CRASH (panic)"
                                                          : "CRASH (watchdog: something stopped responding)");
            esp_core_dump_summary_t *sum = malloc(sizeof *sum);
            char reason[160] = "";
            if (r != ESP_RST_BROWNOUT && sum && esp_core_dump_image_check() == ESP_OK &&
                esp_core_dump_get_summary(sum) == ESP_OK) {
                esp_core_dump_get_panic_reason(reason, sizeof reason);
                pos += (size_t)snprintf(s_report + pos, cap - pos,
                                        "Panic reason: %s\nCrashed task: %.16s\nProgram counter: 0x%08x\n"
                                        "Exception cause: %u %s, address 0x%08x\nBacktrace%s:",
                                        reason[0] ? reason : "(not recorded)", sum->exc_task, (unsigned)sum->exc_pc,
                                        (unsigned)sum->ex_info.exc_cause, exc_cause_text(sum->ex_info.exc_cause),
                                        (unsigned)sum->ex_info.exc_vaddr, sum->exc_bt_info.corrupted ? " (corrupted)" : "");
                for (uint32_t i = 0; i < sum->exc_bt_info.depth && i < 16 && pos + 12 < cap; i++) {
                    pos += (size_t)snprintf(s_report + pos, cap - pos, " 0x%08x", (unsigned)sum->exc_bt_info.bt[i]);
                }
                pos += (size_t)snprintf(s_report + pos, cap - pos,
                                        "\nBuild (ELF SHA-256): %.16s...\nDecode: xtensa-esp32s3-elf-addr2line -pfiaC -e "
                                        "firmware/prebuilt-esp32s3/symbols/skipper-esp32s3-%s.elf <the addresses above>\n",
                                        (const char *)sum->app_elf_sha256, sf_tier_id());
                snprintf(s_short, sizeof s_short, "%.150s in task %.16s", reason[0] ? reason : "panic", sum->exc_task);
            } else {
                pos += (size_t)snprintf(s_report + pos, cap - pos, "(no core dump readable)\n");
                snprintf(s_short, sizeof s_short, "%s", r == ESP_RST_BROWNOUT ? "brownout" : "crash");
            }
            free(sum);
            if (have_lw) {
                pos += (size_t)snprintf(s_report + pos, cap - pos, "\n--- the last log lines before it ---\n");
                uint32_t from = s_lw.total > LW_SIZE ? s_lw.total - LW_SIZE : 0;
                for (uint32_t i = from; i < s_lw.total && pos + 1 < cap; i++) {
                    s_report[pos++] = s_lw.buf[i % LW_SIZE];
                }
                s_report[pos] = 0;
            }
        }
    }
    // start a fresh record for this run
    s_lw.magic = LW_MAGIC;
    s_lw.total = 0;
}

const char *sf_crash_short(void)
{
    return s_short;
}

// After the SD card is mounted: keep the report on the card too, and say it in the log.
void sf_crash_save(void)
{
    if (!s_report) {
        return;
    }
    sf_report("the previous run crashed: %s (details: /crash)", s_short);
    sf_sd_lock();
    if (sf_sd_mounted()) {
        mkdir(SF_SD_ROOT "/skipper", 0775);
        int hi = 0;
        DIR *d = opendir(SF_SD_ROOT "/skipper");
        if (d) {
            struct dirent *e;
            int k;
            while ((e = readdir(d)) != NULL) {
                if (sscanf(e->d_name, "crash-%d.txt", &k) == 1 && k > hi) {
                    hi = k;
                }
            }
            closedir(d);
        }
        char path[64];
        snprintf(path, sizeof path, SF_SD_ROOT "/skipper/crash-%04d.txt", hi + 1);
        FILE *f = fopen(path, "w");
        if (f) {
            fputs(s_report, f);
            fclose(f);
            ESP_LOGW(TAG, "crash report saved on the SD card: skipper/crash-%04d.txt", hi + 1);
        }
    }
    sf_sd_unlock();
}

static esp_err_t crash_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    if (!s_report) {
        return httpd_resp_sendstr(req, "No crash before this start. (A crash report appears here after a crash, "
                                       "brownout or watchdog restart, until the next restart.)\n");
    }
    return httpd_resp_sendstr(req, s_report);
}

static esp_err_t coredump_get(httpd_req_t *req)
{
    size_t addr = 0, size = 0;
    if (esp_core_dump_image_get(&addr, &size) != ESP_OK || !size) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no core dump stored");
    }
    const esp_partition_t *p =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, NULL);
    if (!p || addr < p->address) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no core dump partition");
    }
    char *buf = malloc(2048);
    if (!buf) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
    }
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"coredump.bin\"");
    esp_err_t r = ESP_OK;
    for (size_t off = 0; off < size && r == ESP_OK; off += 2048) {
        size_t n = size - off < 2048 ? size - off : 2048;
        if (esp_partition_read(p, addr - p->address + off, buf, n) != ESP_OK) {
            break;
        }
        r = httpd_resp_send_chunk(req, buf, (ssize_t)n);
    }
    free(buf);
    return r == ESP_OK ? httpd_resp_send_chunk(req, NULL, 0) : r;
}

void sf_crash_register(void *httpd)
{
    static const httpd_uri_t c = {.uri = "/crash", .method = HTTP_GET, .handler = crash_get};
    static const httpd_uri_t d = {.uri = "/coredump.bin", .method = HTTP_GET, .handler = coredump_get};
    httpd_register_uri_handler((httpd_handle_t)httpd, &c);
    httpd_register_uri_handler((httpd_handle_t)httpd, &d);
}
