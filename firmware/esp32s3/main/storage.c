// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// One uploaded G-code file in the "gcode" flash partition (partitions.csv). The browser compresses it
// (zlib, CompressionStream "deflate"); the link task inflates it line by line while printing.
// Layout: a 4 KB header sector (magic, sizes, name), then the data.
#include <string.h>

#include "esp_log.h"
#include "esp_partition.h"
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

static hdr_t s_new;
static uint32_t s_written, s_erased;

size_t sf_store_capacity(void)
{
    const esp_partition_t *p = part();
    return p ? p->size - HDR_SIZE : 0;
}

bool sf_store_begin(const char *name, uint32_t data_len, uint32_t orig_len, bool compressed)
{
    const esp_partition_t *p = part();
    if (!p || data_len == 0 || data_len > p->size - HDR_SIZE) {
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
        if (s_erased + n > p->size) {
            n = p->size - s_erased;
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
        h.data_len > p->size - HDR_SIZE) {
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
