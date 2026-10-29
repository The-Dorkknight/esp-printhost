// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Build tiers. One source, three builds, picked by how much memory the ESP32-S3 has:
//   DECKHAND  any ESP32-S3 (2 MB flash, no PSRAM): the low-barrier build
//   CAPTAIN   4 MB flash (PSRAM used if present): + firmware updates over WiFi, bigger buffers
//   SKIPPER   8 MB flash + 8 MB octal PSRAM: + the biggest buffers and an RGB salute
// The names are deliberately ridiculous; what each build does differently is plain (GET /about).
// Every build also reads the chip's own memory (eFuse and the flash chip) and says when the board
// could run a bigger one.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_efuse.h"
#include "esp_efuse_table.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "sdkconfig.h"
#include "sf.h"
#if CONFIG_SPIRAM
#include "esp_psram.h"
#endif

static int tier_rank(const char *id)
{
    return !strcmp(id, "skipper") ? 2 : !strcmp(id, "captain") ? 1 : 0;
}

const char *sf_tier_id(void)
{
    return CONFIG_SKIPPER_TIER;
}

static const char *const DESCRIPTORS[] = {"PRO", "MAX", "Turbo", "Ultra", "Mega", "Elite", "Performance", "Gamer",
                                          "MAXX", "(RGB)"};

// How many of the descriptors each tier has earned.
static int descriptor_count(int rank)
{
    return rank == 2 ? 10 : rank == 1 ? 4 : 0;
}

const char *sf_tier_title(void)
{
    static char t[160];
    if (!t[0]) {
        int r = tier_rank(sf_tier_id());
        snprintf(t, sizeof t, "%s Edition", r == 2 ? "SKIPPER" : r == 1 ? "CAPTAIN" : "DECKHAND");
        for (int i = 0; i < descriptor_count(r); i++) {
            strlcat(t, " ", sizeof t);
            strlcat(t, DESCRIPTORS[i], sizeof t);
        }
    }
    return t;
}

size_t sf_psram_bytes(void)
{
#if CONFIG_SPIRAM
    return esp_psram_is_initialized() ? esp_psram_get_size() : 0;
#else
    return 0;
#endif
}

int sf_lookahead_max(void)
{
    return sf_psram_bytes() ? CONFIG_SKIPPER_LOOKAHEAD : 32;
}

typedef struct {
    uint32_t flash_phys;   // the flash chip's real size
    uint32_t flash_build;  // what this build assumes
    int efuse_flash_mb;    // in-package flash per eFuse (0 = external flash chip / unknown)
    int efuse_psram_mb;    // in-package PSRAM per eFuse (0 = none)
    size_t psram;          // PSRAM in use
    const char *best;      // the biggest build this board can run
} mem_t;

static void mem_probe(mem_t *m)
{
    memset(m, 0, sizeof *m);
    esp_flash_get_physical_size(NULL, &m->flash_phys);
    esp_flash_get_size(NULL, &m->flash_build);
    uint8_t fc = 0, pc = 0;
    esp_efuse_read_field_blob(ESP_EFUSE_FLASH_CAP, &fc, 3);
    esp_efuse_read_field_blob(ESP_EFUSE_PSRAM_CAP, &pc, 3);
    m->efuse_flash_mb = fc == 1 ? 8 : fc == 2 ? 4 : 0;
    m->efuse_psram_mb = pc == 1 ? 8 : pc == 2 ? 2 : pc == 3 ? 16 : pc == 4 ? 4 : 0;
    m->psram = sf_psram_bytes();
    uint32_t flash = m->flash_phys ? m->flash_phys : m->flash_build;
    int psram_mb = m->psram ? (int)(m->psram >> 20) : m->efuse_psram_mb;
    // 8 MB+ PSRAM on the ESP32-S3 is octal (S3R8 / R8V / R16V); 2 / 4 MB is quad. CAPTAIN wants
    // PSRAM too: its PSRAM support costs ~8 KB of internal RAM, which a board without PSRAM would
    // rather keep (the same rule as the Flasher's, host/tools/skipper_esp.py choose_tier()).
    m->best = (flash >= (8u << 20) && psram_mb >= 8)   ? "skipper"
              : (flash >= (4u << 20) && psram_mb >= 2) ? "captain"
                                                       : "deckhand";
}

static bool ota_build(void)
{
#if CONFIG_SKIPPER_OTA
    return true;
#else
    return false;
#endif
}

int sf_tier_json(char *out, size_t n)
{
    mem_t m;
    mem_probe(&m);
    return snprintf(out, n,
                    "{\"id\":\"%s\",\"title\":\"%s\",\"flash_bytes\":%u,\"flash_build_bytes\":%u,\"efuse_flash_mb\":%d,"
                    "\"efuse_psram_mb\":%d,\"psram_bytes\":%u,\"best\":\"%s\",\"upgrade\":%s,\"ota\":%s,"
                    "\"lookahead_max\":%d,\"log_ring_kb\":%d}",
                    sf_tier_id(), sf_tier_title(), (unsigned)m.flash_phys, (unsigned)m.flash_build, m.efuse_flash_mb,
                    m.efuse_psram_mb, (unsigned)m.psram, m.best,
                    tier_rank(m.best) > tier_rank(sf_tier_id()) ? "true" : "false", ota_build() ? "true" : "false",
                    sf_lookahead_max(), m.psram ? CONFIG_SKIPPER_LOG_RING_KB : 8);
}

static const char *yes_no(bool b)
{
    return b ? "<b class=\"y\">on</b>" : "<b class=\"n\">off</b>";
}

// GET /about: which build this is, what this board has, what is on and off, and why.
static esp_err_t about_get(httpd_req_t *req)
{
    mem_t m;
    mem_probe(&m);
    int r = tier_rank(sf_tier_id());
    static char chips[400];
    chips[0] = 0;
    for (int i = 0; i < descriptor_count(r); i++) {
        size_t l = strlen(chips);
        snprintf(chips + l, sizeof chips - l, "<span class=\"c\">%s</span>", DESCRIPTORS[i]);
    }
    static char up[600];
    up[0] = 0;
    if (tier_rank(m.best) > r) {
        snprintf(up, sizeof up,
                 "<p class=\"up\"><b>This board can run the %s build.</b> On the PC: Flasher, Master tab, "
                 "<b>Flash for my board</b>. Settings and printer.cfg are kept.</p>",
                 !strcmp(m.best, "skipper") ? "SKIPPER" : "CAPTAIN");
    } else if (tier_rank(m.best) < r) {
        snprintf(up, sizeof up,
                 "<p class=\"up\">This build is meant for a bigger board than the memory found here; the "
                 "extras that need the missing memory stay off.</p>");
    }
    const esp_partition_t *run = esp_ota_get_running_partition();
    size_t ps = m.psram;
    char psr[80];
    if (ps) {
        snprintf(psr, sizeof psr, "%u MB found and in use", (unsigned)(ps >> 20));
    } else {
        snprintf(psr, sizeof psr, "%s", m.efuse_psram_mb ? "inside the chip, not used by this build" : "none found");
    }
    // from the heap per request (not static): DECKHAND needs its internal RAM for the printer link
    const size_t bufn = 7400;
    char *buf = malloc(bufn);
    if (!buf) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
    }
    snprintf(buf, bufn,
             "<!doctype html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" "
             "content=\"width=device-width,initial-scale=1\"><title>About this build</title><style>"
             "body{background:#e9e6dc;color:#141414;font:16px/1.5 Georgia,serif;margin:0}"
             ".b{background:#141414;color:#e9e6dc;text-align:center;padding:10px;font-weight:700}"
             "main{max-width:680px;margin:0 auto;padding:16px}h1{font:700 2rem Impact,Arial Black,sans-serif;"
             "border-bottom:4px solid #141414;margin-bottom:6px}.c{display:inline-block;background:#141414;"
             "color:#ffd400;font:700 .8rem Impact,Arial Black,sans-serif;letter-spacing:.06em;padding:2px 8px;"
             "margin:2px;transform:rotate(-2deg)}table{border-collapse:collapse;width:100%%;background:#f7f5ee}"
             "td{border:2px solid #141414;padding:5px 8px;vertical-align:top}.y{color:#1a7f37}.n{color:#c4161c}"
             ".up{background:#ffd400;border:3px solid #141414;padding:8px}small{color:#555}</style></head><body>" SF_NAV
             "<div class=\"b\">" SF_BANNER "</div><main><h1>%s</h1><p>%s</p>%s"
             "<h2>This board</h2><table>"
             "<tr><td>Build</td><td>%s (firmware " SF_VERSION ", running from %s)</td></tr>"
             "<tr><td>Flash chip</td><td>%u MB (this build uses %u MB)%s</td></tr>"
             "<tr><td>PSRAM</td><td>%s</td></tr>"
             "<tr><td>Free memory</td><td>%u KB internal, %u KB PSRAM</td></tr></table>"
             "<h2>What runs here</h2><table>"
             "<tr><td>Klipper link, heaters, fans, TMC2209, motor tests, moves, homing, printing</td><td>%s</td></tr>"
             "<tr><td>SD card: big files, thumbnails, print history, event log, printer.cfg backups</td><td>%s</td></tr>"
             "<tr><td>Send from the slicer (PrusaSlicer / OrcaSlicer, as OctoPrint)</td><td>%s</td></tr>"
             "<tr><td>Join your home WiFi (as well as its own network)</td><td>%s</td></tr>"
             "<tr><td>Firmware updates over WiFi, with roll-back (<a href=\"/update\">/update</a>)</td><td>%s"
             "<br><small>CAPTAIN and SKIPPER (4 MB flash or more)</small></td></tr>"
             "<tr><td>Motion look-ahead</td><td>%d moves<br><small>32 without PSRAM; CAPTAIN 256, SKIPPER 512 "
             "with PSRAM. Smoother on curves made of tiny segments.</small></td></tr>"
             "<tr><td>Live console history</td><td>%d KB<br><small>8 KB without PSRAM</small></td></tr>"
             "<tr><td>RGB start-up salute</td><td>%s<br><small>SKIPPER only. Obviously.</small></td></tr>"
             "</table><p><small>Tier names are a joke; the table above is not. The rule: DECKHAND runs on any "
             "ESP32-S3; CAPTAIN needs 4 MB of flash and PSRAM; SKIPPER needs 8 MB of flash and 8 MB of "
             "PSRAM.</small></p>"
             "<p><a href=\"/status\">Status</a> &middot; <a href=\"/printer\">Printer</a> &middot; "
             "<a href=\"/settings\">Settings</a></p></main></body></html>",
             sf_tier_title(), chips, up, sf_tier_id(), run ? run->label : "?", (unsigned)(m.flash_phys >> 20),
             (unsigned)(m.flash_build >> 20), m.efuse_flash_mb ? " (inside the chip)" : "",
             psr,
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024), yes_no(true), yes_no(sf_sd_mounted()),
             yes_no(sf_sd_mounted()), yes_no(true), yes_no(ota_build()), sf_lookahead_max(),
             ps ? CONFIG_SKIPPER_LOG_RING_KB : 8,
#if CONFIG_SKIPPER_RGB_SALUTE
             yes_no(true)
#else
             yes_no(false)
#endif
    );
    httpd_resp_set_type(req, "text/html");
    esp_err_t res = httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    free(buf);
    return res;
}

void sf_tier_register(void *httpd)
{
    static const httpd_uri_t about = {.uri = "/about", .method = HTTP_GET, .handler = about_get};
    httpd_register_uri_handler((httpd_handle_t)httpd, &about);
}
