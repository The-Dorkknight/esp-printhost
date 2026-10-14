// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Status light. Plain LED: blink pattern. WS2812 colour LED: colour + pulse.
//   searching for the printer board: slow blink / blue
//   printer board answering:         short double blink / green
//   link error:                      fast blink / red
//   link switched off:               one short blink every 2 s / white
// Blue and white look the same whichever colour order the LED uses, so they are safe tells.
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sf.h"

static const char *TAG = "led";
static int s_type, s_gpio, s_rgb;
static rmt_channel_handle_t s_chan;
static rmt_encoder_handle_t s_enc;

static void ws_send(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t px[3] = {g, r, b};   // standard WS2812 order: green, red, blue
    if (s_rgb) {                 // some boards' LEDs take red, green, blue
        px[0] = r;
        px[1] = g;
    }
    rmt_transmit_config_t tc = {.loop_count = 0};
    if (rmt_transmit(s_chan, s_enc, px, sizeof px, &tc) == ESP_OK) {
        rmt_tx_wait_all_done(s_chan, 50);
    }
}

static bool ws_init(int gpio)
{
    rmt_tx_channel_config_t cfg = {
        .gpio_num = gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000,   // 0.1 us per tick
        .mem_block_symbols = 64,
        .trans_queue_depth = 2,
    };
    if (rmt_new_tx_channel(&cfg, &s_chan) != ESP_OK) {
        return false;
    }
    rmt_bytes_encoder_config_t bc = {
        .bit0 = {.level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9},   // 0.3 us high, 0.9 us low
        .bit1 = {.level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3},   // 0.9 us high, 0.3 us low
        .flags.msb_first = 1,
    };
    if (rmt_new_bytes_encoder(&bc, &s_enc) != ESP_OK) {
        return false;
    }
    return rmt_enable(s_chan) == ESP_OK;
}

static void set(bool on, sf_link_state_t st)
{
    if (s_type == SF_LED_PLAIN) {
        gpio_set_level((gpio_num_t)s_gpio, on);
        return;
    }
    if (!on) {
        ws_send(0, 0, 0);
        return;
    }
    switch (st) {   // kept dim: these LEDs are very bright
    case SF_LINK_OK: ws_send(0, 24, 0); break;
    case SF_LINK_ERROR: ws_send(32, 0, 0); break;
    case SF_LINK_OFF: ws_send(12, 12, 12); break;
    default: ws_send(0, 0, 32); break;   // searching: blue
    }
}

static void led_task(void *arg)
{
    for (;;) {
        sf_link_info_t li;
        sf_link_get(&li);
        switch (li.state) {
        case SF_LINK_OK:   // double blink once a second
            set(true, li.state); vTaskDelay(pdMS_TO_TICKS(80));
            set(false, li.state); vTaskDelay(pdMS_TO_TICKS(120));
            set(true, li.state); vTaskDelay(pdMS_TO_TICKS(80));
            set(false, li.state); vTaskDelay(pdMS_TO_TICKS(720));
            break;
        case SF_LINK_ERROR:   // fast blink
            set(true, li.state); vTaskDelay(pdMS_TO_TICKS(100));
            set(false, li.state); vTaskDelay(pdMS_TO_TICKS(100));
            break;
        case SF_LINK_OFF:
            set(true, li.state); vTaskDelay(pdMS_TO_TICKS(60));
            set(false, li.state); vTaskDelay(pdMS_TO_TICKS(1940));
            break;
        default:   // searching: slow blink
            set(true, li.state); vTaskDelay(pdMS_TO_TICKS(500));
            set(false, li.state); vTaskDelay(pdMS_TO_TICKS(500));
            break;
        }
    }
}

void sf_led_start(const sf_settings_t *s)
{
    s_type = s->led_type;
    s_gpio = s->led_gpio;
    s_rgb = s->led_rgb;
    if (s_type == SF_LED_NONE || s_gpio < 0 || !GPIO_IS_VALID_OUTPUT_GPIO(s_gpio)) {
        ESP_LOGW(TAG, "status LED off (type %d, GPIO %d)", s_type, s_gpio);
        return;
    }
    if (s_type == SF_LED_PLAIN) {
        gpio_reset_pin((gpio_num_t)s_gpio);
        gpio_set_direction((gpio_num_t)s_gpio, GPIO_MODE_OUTPUT);
    } else if (!ws_init(s_gpio)) {
        ESP_LOGE(TAG, "could not start the WS2812 driver on GPIO %d", s_gpio);
        return;
    }
    ESP_LOGW(TAG, "status LED: %s on GPIO %d", s_type == SF_LED_PLAIN ? "plain" : "WS2812 colour", s_gpio);
    xTaskCreate(led_task, "led", 3072, NULL, 2, NULL);
}
