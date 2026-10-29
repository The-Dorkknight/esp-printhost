// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// One uploaded G-code file in the "gcode" flash partition (partitions.csv). The browser compresses it
// (zlib, CompressionStream "deflate"); the link task inflates it line by line while printing.
// Layout: a 4 KB header sector (magic, sizes, name), then the data. Since 2026-10-27 the last
// 2 x (CONFIG_SKIPPER_CFG_KB + 4) KB of the partition hold printer.cfg instead (two slots written
// alternately: erase, write the text, then its header last, so a power cut while saving leaves the
// previous copy valid). This needs no new partition, so it also works after a WiFi update.
#include <string.h>

#include "esp_log.h"
#include "esp_partition.h"
#include "esp_rom_crc.h"
#include "sdkconfig.h"
#include "sf.h"

static const char *TAG = "storage";
#define HDR_SIZE 4096
#define MAGIC 0x43464753u   // "SGFC"

typedef struct {
    uint32_t magic, data_len, orig_len, compressed;
    char name[64];
} hdr_t;

static const esp_partition_t *part(void)
{
    static const esp_partition_t *p;
    if (!p) {
        p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)0x40, "gcode");
    }
    return p;
}

#define CFG_SLOT ((uint32_t)CONFIG_SKIPPER_CFG_KB * 1024 + 4096)
#define CFG_MAGIC 0x47464353u   // "SCFG"
typedef struct {
    uint32_t magic, seq, len, crc;
} cfg_hdr_t;

// Where the G-code data must stop: the start of the printer.cfg slots.
static uint32_t data_end(const esp_partition_t *p)
{
    return p->size > 2 * CFG_SLOT + 2 * HDR_SIZE ? p->size - 2 * CFG_SLOT : p->size;
}

static hdr_t s_new;
static uint32_t s_written, s_erased;

size_t sf_store_capacity(void)
{
    const esp_partition_t *p = part();
    return p ? data_end(p) - HDR_SIZE : 0;
}

bool sf_store_begin(const char *name, uint32_t data_len, uint32_t orig_len, bool compressed)
{
    const esp_partition_t *p = part();
    if (!p || data_len == 0 || data_len > data_end(p) - HDR_SIZE) {
        return false;
    }
    // invalidate the old file first (a half-written upload must never look valid)
    if (esp_partition_erase_range(p, 0, HDR_SIZE) != ESP_OK) {
        return false;
    }
    memset(&s_new, 0, sizeof s_new);
    s_new.magic = MAGIC;
    s_new.data_len = data_len;
    s_new.orig_len = orig_len;
    s_new.compressed = compressed;
    strncpy(s_new.name, name, sizeof s_new.name - 1);
    s_written = 0;
    s_erased = HDR_SIZE;
    return true;
}

bool sf_store_write(const void *data, size_t len)
{
    const esp_partition_t *p = part();
    if (!p || s_written + len > s_new.data_len) {
        return false;
    }
    uint32_t end = HDR_SIZE + s_written + (uint32_t)len;
    while (s_erased < end) {   // erase 64 KB at a time just ahead of the data
        uint32_t n = 65536 - (s_erased % 65536);
        if (s_erased + n > data_end(p)) {   // never into the printer.cfg slots
            n = data_end(p) - s_erased;
        }
        if (esp_partition_erase_range(p, s_erased, n) != ESP_OK) {
            return false;
        }
        s_erased += n;
    }
    if (esp_partition_write(p, HDR_SIZE + s_written, data, len) != ESP_OK) {
        return false;
    }
    s_written += (uint32_t)len;
    return true;
}

bool sf_store_end(void)
{
    const esp_partition_t *p = part();
    if (!p || s_written != s_new.data_len) {
        return false;
    }
    bool ok = esp_partition_write(p, 0, &s_new, sizeof s_new) == ESP_OK;
    ESP_LOGW(TAG, "stored %s: %u bytes (%u before compression)", s_new.name, (unsigned)s_new.data_len,
             (unsigned)s_new.orig_len);
    return ok;
}

bool sf_store_info(char *name, size_t n, uint32_t *data_len, uint32_t *orig_len, bool *compressed)
{
    const esp_partition_t *p = part();
    hdr_t h;
    if (!p || esp_partition_read(p, 0, &h, sizeof h) != ESP_OK || h.magic != MAGIC || h.data_len == 0 ||
        h.data_len > data_end(p) - HDR_SIZE) {
        return false;
    }
    h.name[sizeof h.name - 1] = 0;
    if (name) {
        strncpy(name, h.name, n - 1);
        name[n - 1] = 0;
    }
    if (data_len) {
        *data_len = h.data_len;
    }
    if (orig_len) {
        *orig_len = h.orig_len;
    }
    if (compressed) {
        *compressed = h.compressed != 0;
    }
    return true;
}

bool sf_store_read(uint32_t offset, void *buf, size_t len)
{
    const esp_partition_t *p = part();
    return p && esp_partition_read(p, HDR_SIZE + offset, buf, len) == ESP_OK;
}

// ---------------------------------------------------------------- printer.cfg slots

size_t sf_cfgstore_max(void)
{
    const esp_partition_t *p = part();
    return p && data_end(p) < p->size ? (size_t)CONFIG_SKIPPER_CFG_KB * 1024 : 0;
}

static bool slot_hdr(const esp_partition_t *p, int k, cfg_hdr_t *h)
{
    uint32_t off = data_end(p) + (uint32_t)k * CFG_SLOT;
    return esp_partition_read(p, off, h, sizeof *h) == ESP_OK && h->magic == CFG_MAGIC && h->len > 0 &&
           h->len <= CFG_SLOT - sizeof *h;
}

// The newest valid copy: its length (text copied into out, NUL-terminated, if it fits), or -1.
int sf_cfgstore_get(char *out, size_t n)
{
    const esp_partition_t *p = part();
    if (!sf_cfgstore_max()) return -1;
    int best = -1;
    cfg_hdr_t hb = {0};
    for (int k = 0; k < 2; k++) {
        cfg_hdr_t h;
        if (!slot_hdr(p, k, &h) || (best >= 0 && h.seq <= hb.seq)) continue;
        if (h.len + 1 > n) continue;
        uint32_t off = data_end(p) + (uint32_t)k * CFG_SLOT + sizeof h;
        if (esp_partition_read(p, off, out, h.len) != ESP_OK) continue;
        if (esp_rom_crc32_le(0, (const uint8_t *)out, h.len) != h.crc) continue;   // torn write: ignore
        best = k, hb = h;
    }
    if (best < 0) return -1;
    uint32_t off = data_end(p) + (uint32_t)best * CFG_SLOT + sizeof hb;   // re-read: `out` may hold the other slot
    if (esp_partition_read(p, off, out, hb.len) != ESP_OK) return -1;
    out[hb.len] = 0;
    return (int)hb.len;
}

bool sf_cfgstore_put(const char *text, size_t len)
{
    const esp_partition_t *p = part();
    if (!sf_cfgstore_max() || len == 0 || len > sf_cfgstore_max()) return false;
    // An uploaded G-code file from an older build may run into the slots: drop it (once).
    hdr_t g;
    if (esp_partition_read(p, 0, &g, sizeof g) == ESP_OK && g.magic == MAGIC && HDR_SIZE + g.data_len > data_end(p)) {
        esp_partition_erase_range(p, 0, HDR_SIZE);
        ESP_LOGW(TAG, "the stored G-code file reached into the printer.cfg area; it was removed (upload it again)");
    }
    cfg_hdr_t h[2];
    bool v[2] = {slot_hdr(p, 0, &h[0]), slot_hdr(p, 1, &h[1])};
    uint32_t seq = 0;
    int target = 0;
    if (v[0] || v[1]) {
        int newest = (v[0] && (!v[1] || h[0].seq >= h[1].seq)) ? 0 : 1;
        seq = h[newest].seq;
        target = 1 - newest;   // overwrite the older copy
    }
    uint32_t off = data_end(p) + (uint32_t)target * CFG_SLOT;
    cfg_hdr_t nh = {CFG_MAGIC, seq + 1, (uint32_t)len, esp_rom_crc32_le(0, (const uint8_t *)text, len)};
    bool ok = esp_partition_erase_range(p, off, CFG_SLOT) == ESP_OK &&
              esp_partition_write(p, off + sizeof nh, text, len) == ESP_OK &&
              esp_partition_write(p, off, &nh, sizeof nh) == ESP_OK;   // header last
    if (ok) ESP_LOGW(TAG, "printer.cfg saved (%u bytes, copy %d)", (unsigned)len, target);
    return ok;
}
