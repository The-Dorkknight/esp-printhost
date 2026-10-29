// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
#include "nvs.h"
#include "sdkconfig.h"
#include "sf.h"

#define NS "skipper"

void sf_settings_defaults(sf_settings_t *s)
{
    s->uart_tx = CONFIG_SKIPPER_UART_TX_GPIO;
    s->uart_rx = CONFIG_SKIPPER_UART_RX_GPIO;
    s->baud = 250000;
    s->led_type = CONFIG_SKIPPER_LED_TYPE;
    s->led_gpio = CONFIG_SKIPPER_LED_GPIO;
    s->led_rgb = 0;
    s->link_enabled = 1;
    // a row of four pins on the ESP32-S3-Zero: a 6-pin SD module (CS SCK MOSI MISO VCC GND) sits on it
#if CONFIG_SKIPPER_SD_DEFAULT
    s->sd_enabled = 1;
#else
    s->sd_enabled = 0;   // e.g. the QEMU test build (no SPI card emulated)
#endif
    s->sd_cs = 10;     // 10-13 are SPI2's own pins (IO_MUX), next to TX/RX on the ESP32-S3-Zero's edge
    s->sd_sck = 12;
    s->sd_mosi = 11;
    s->sd_miso = 13;
    s->sta_ssid[0] = 0;
    s->sta_pass[0] = 0;
    // little I2C OLED: on, looked for at start-up; nothing connected is fine (the settings page says so)
    s->oled_type = SF_OLED_AUTO;
    s->oled_sda = 8;   // as in KlipperTFT on the ESP32-S3; next to the SD card's 10-13 on the S3-Zero
    s->oled_scl = 9;
    s->oled_flip = 0;
}

static int get_i32(nvs_handle_t h, const char *k, int dflt)
{
    int32_t v;
    return nvs_get_i32(h, k, &v) == ESP_OK ? (int)v : dflt;
}

void sf_settings_load(sf_settings_t *s)
{
    sf_settings_defaults(s);
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    s->uart_tx = get_i32(h, "uart_tx", s->uart_tx);
    s->uart_rx = get_i32(h, "uart_rx", s->uart_rx);
    s->baud = get_i32(h, "baud", s->baud);
    s->led_type = get_i32(h, "led_type", s->led_type);
    s->led_gpio = get_i32(h, "led_gpio", s->led_gpio);
    s->led_rgb = get_i32(h, "led_rgb", s->led_rgb);
    s->link_enabled = get_i32(h, "link_on", s->link_enabled);
    s->sd_enabled = get_i32(h, "sd_on", s->sd_enabled);
    s->sd_cs = get_i32(h, "sd_cs", s->sd_cs);
    s->sd_sck = get_i32(h, "sd_sck", s->sd_sck);
    s->sd_mosi = get_i32(h, "sd_mosi", s->sd_mosi);
    s->sd_miso = get_i32(h, "sd_miso", s->sd_miso);
    s->oled_type = get_i32(h, "oled_type", s->oled_type);
    s->oled_sda = get_i32(h, "oled_sda", s->oled_sda);
    s->oled_scl = get_i32(h, "oled_scl", s->oled_scl);
    s->oled_flip = get_i32(h, "oled_flip", s->oled_flip);
    size_t n = sizeof s->sta_ssid;
    if (nvs_get_str(h, "sta_ssid", s->sta_ssid, &n) != ESP_OK) {
        s->sta_ssid[0] = 0;
    }
    n = sizeof s->sta_pass;
    if (nvs_get_str(h, "sta_pass", s->sta_pass, &n) != ESP_OK) {
        s->sta_pass[0] = 0;
    }

    nvs_close(h);
}

bool sf_settings_save(const sf_settings_t *s)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) {
        return false;
    }
    bool ok = nvs_set_i32(h, "uart_tx", s->uart_tx) == ESP_OK && nvs_set_i32(h, "uart_rx", s->uart_rx) == ESP_OK &&
              nvs_set_i32(h, "baud", s->baud) == ESP_OK && nvs_set_i32(h, "led_type", s->led_type) == ESP_OK &&
              nvs_set_i32(h, "led_gpio", s->led_gpio) == ESP_OK && nvs_set_i32(h, "led_rgb", s->led_rgb) == ESP_OK &&
              nvs_set_i32(h, "link_on", s->link_enabled) == ESP_OK && nvs_set_i32(h, "sd_on", s->sd_enabled) == ESP_OK &&
              nvs_set_i32(h, "sd_cs", s->sd_cs) == ESP_OK && nvs_set_i32(h, "sd_sck", s->sd_sck) == ESP_OK &&
              nvs_set_i32(h, "sd_mosi", s->sd_mosi) == ESP_OK && nvs_set_i32(h, "sd_miso", s->sd_miso) == ESP_OK &&
              nvs_set_str(h, "sta_ssid", s->sta_ssid) == ESP_OK && nvs_set_str(h, "sta_pass", s->sta_pass) == ESP_OK &&
              nvs_set_i32(h, "oled_type", s->oled_type) == ESP_OK && nvs_set_i32(h, "oled_sda", s->oled_sda) == ESP_OK &&
              nvs_set_i32(h, "oled_scl", s->oled_scl) == ESP_OK && nvs_set_i32(h, "oled_flip", s->oled_flip) == ESP_OK &&
              nvs_commit(h) == ESP_OK;
    nvs_close(h);
    return ok;
}
