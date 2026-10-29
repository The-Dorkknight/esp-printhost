// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// A micro SD card on SPI (FAT32 or exFAT is NOT needed: FAT16/FAT32 as most cards come), mounted at /sd.
// G-code files live in the card's top folder as plain text, so a card written on a PC prints too.
// Default pins (settings page): CS 10, MOSI 11, SCK 12, MISO 13. These are SPI2's native IO_MUX pins
// (the driver then skips the GPIO matrix) and they are edge pins on the ESP32-S3-Zero, next to TX/RX.
// Any other free GPIOs work too, through the GPIO matrix (fine at 20 MHz).
#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdmmc_cmd.h"
#include "sf.h"

static const char *TAG = "sdcard";
#define SD_HOST SPI2_HOST

static sdmmc_card_t *s_card;
static bool s_bus;
static char s_err[120] = "not started";
static sf_settings_t s_pins;
static SemaphoreHandle_t s_lock;

void sf_sd_lock(void)
{
    if (!s_lock) {
        s_lock = xSemaphoreCreateRecursiveMutex();
    }
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
}

void sf_sd_unlock(void)
{
    xSemaphoreGiveRecursive(s_lock);
}

const char *sf_sd_pin_problem(const sf_settings_t *s)
{
    const int p[4] = {s->sd_cs, s->sd_sck, s->sd_mosi, s->sd_miso};
    for (int i = 0; i < 4; i++) {
        int g = p[i];
        if (!GPIO_IS_VALID_OUTPUT_GPIO(g)) {
            return "SD card pins must be valid output GPIOs";
        }
        if (g == 0 || g == 3 || g == 45 || g == 46) {
            return "GPIO0/3/45/46 are boot (strapping) pins: pick others for the SD card";
        }
        if (g == 19 || g == 20) {
            return "GPIO19/20 are the USB port: pick others for the SD card";
        }
        if (g >= 26 && g <= 37) {
            return "GPIO26-37 belong to the flash / PSRAM chips: pick others for the SD card";
        }
        if (g == s->uart_tx || g == s->uart_rx) {
            return "an SD card pin is also a UART pin";
        }
        if (s->led_type != SF_LED_NONE && g == s->led_gpio) {
            return "an SD card pin is also the status LED pin";
        }
        for (int j = 0; j < i; j++) {
            if (p[j] == g) {
                return "the four SD card pins must all be different";
            }
        }
    }
    return NULL;
}

bool sf_sd_mounted(void)
{
    return s_card != NULL;
}

const char *sf_sd_error(void)
{
    return s_err;
}

void sf_sd_unmount(void)
{
    sf_sd_lock();
    if (s_card) {
        esp_vfs_fat_sdcard_unmount(SF_SD_ROOT, s_card);
        s_card = NULL;
    }
    if (s_bus) {
        spi_bus_free(SD_HOST);
        s_bus = false;
    }
    snprintf(s_err, sizeof s_err, "not mounted");
    sf_sd_unlock();
}

static bool mount_locked(const sf_settings_t *s);

bool sf_sd_mount(const sf_settings_t *s)
{
    sf_sd_lock();
    bool ok = mount_locked(s);
    sf_sd_unlock();
    return ok;
}

static bool mount_locked(const sf_settings_t *s)
{
    if (s) {
        s_pins = *s;
    }
    sf_sd_unmount();
    if (!s_pins.sd_enabled) {
        snprintf(s_err, sizeof s_err, "switched off (settings page)");
        return false;
    }
    const char *bad = sf_sd_pin_problem(&s_pins);
    if (bad) {
        snprintf(s_err, sizeof s_err, "%s", bad);
        return false;
    }
    // SD cards want pull-ups on their data lines; the internal ones are enough for short wires.
    gpio_set_pull_mode((gpio_num_t)s_pins.sd_miso, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)s_pins.sd_cs, GPIO_PULLUP_ONLY);
    spi_bus_config_t bus = {
        .mosi_io_num = s_pins.sd_mosi,
        .miso_io_num = s_pins.sd_miso,
        .sclk_io_num = s_pins.sd_sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    esp_err_t e = spi_bus_initialize(SD_HOST, &bus, SDSPI_DEFAULT_DMA);
    if (e != ESP_OK) {
        snprintf(s_err, sizeof s_err, "SPI bus: %s", esp_err_to_name(e));
        return false;
    }
    s_bus = true;
    esp_vfs_fat_sdmmc_mount_config_t mc = {
        .format_if_mount_failed = false,   // never wipe a card on its own
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };
    // 20 MHz first; long wires or a module with a slow level shifter may need the slower retry
    const int speeds[2] = {SDMMC_FREQ_DEFAULT, 4000};
    for (int k = 0; k < 2; k++) {
        sdmmc_host_t host = SDSPI_HOST_DEFAULT();
        host.slot = SD_HOST;
        host.max_freq_khz = speeds[k];
        sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
        slot.gpio_cs = s_pins.sd_cs;
        slot.host_id = SD_HOST;
        e = esp_vfs_fat_sdspi_mount(SF_SD_ROOT, &host, &slot, &mc, &s_card);
        if (e == ESP_OK) {
            break;
        }
        s_card = NULL;
    }
    if (e != ESP_OK) {
        if (e == ESP_FAIL) {
            snprintf(s_err, sizeof s_err, "card found but no FAT file system on it: format it FAT32 on a PC");
        } else if (e == ESP_ERR_TIMEOUT || e == ESP_ERR_INVALID_RESPONSE || e == ESP_ERR_NOT_FOUND) {
            snprintf(s_err, sizeof s_err, "no card answering on CS %d SCK %d MOSI %d MISO %d (%s)", s_pins.sd_cs,
                     s_pins.sd_sck, s_pins.sd_mosi, s_pins.sd_miso, esp_err_to_name(e));
        } else {
            snprintf(s_err, sizeof s_err, "mount failed: %s", esp_err_to_name(e));
        }
        ESP_LOGW(TAG, "SD card: %s", s_err);
        spi_bus_free(SD_HOST);
        s_bus = false;
        return false;
    }
    uint64_t total = 0, freeb = 0;
    sf_sd_space(&total, &freeb);
    snprintf(s_err, sizeof s_err, "ok");
    ESP_LOGW(TAG, "SD card mounted: %s, %llu MB, %llu MB free", s_card->cid.name, (unsigned long long)(total >> 20),
             (unsigned long long)(freeb >> 20));
    return true;
}

bool sf_sd_space(uint64_t *total, uint64_t *freeb)
{
    *total = *freeb = 0;
    if (!s_card) {
        return false;
    }
    return esp_vfs_fat_info(SF_SD_ROOT, total, freeb) == ESP_OK;
}

static bool is_gcode(const char *n)
{
    const char *dot = strrchr(n, '.');
    return n[0] != '.' && dot && (!strcasecmp(dot, ".gcode") || !strcasecmp(dot, ".gco") || !strcasecmp(dot, ".g"));
}

int sf_sd_list(sf_sd_file_cb cb, void *arg)
{
    if (!s_card) {
        return -1;
    }
    DIR *d = opendir(SF_SD_ROOT);
    if (!d) {
        return -1;
    }
    int n = 0;
    struct dirent *e;
    char path[300];
    while ((e = readdir(d)) != NULL) {
        if (e->d_type == DT_DIR || !is_gcode(e->d_name)) {
            continue;
        }
        struct stat st;
        snprintf(path, sizeof path, SF_SD_ROOT "/%s", e->d_name);
        if (stat(path, &st) != 0) {
            continue;
        }
        cb(e->d_name, (uint64_t)st.st_size, arg);
        n++;
    }
    closedir(d);
    return n;
}

bool sf_sd_name_ok(const char *name)
{
    if (!name || !name[0] || strlen(name) > 200 || strchr(name, '/') || strchr(name, '\\') || name[0] == '.') {
        return false;
    }
    return is_gcode(name);
}

bool sf_sd_path(const char *name, char *out, size_t n)
{
    if (!sf_sd_name_ok(name)) {
        return false;
    }
    return snprintf(out, n, SF_SD_ROOT "/%s", name) < (int)n;
}

bool sf_sd_delete(const char *name)
{
    char p[300];
    return s_card && sf_sd_path(name, p, sizeof p) && unlink(p) == 0;
}
