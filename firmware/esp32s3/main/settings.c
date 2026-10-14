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
              nvs_set_i32(h, "link_on", s->link_enabled) == ESP_OK && nvs_commit(h) == ESP_OK;
    nvs_close(h);
    return ok;
}
