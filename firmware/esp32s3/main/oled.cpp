// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// The little I2C OLED (SSD1306 / SSD1315 / SH1106, 128x64 or 128x32), added 2026-10-23.
// Drawing and the screens come from components/ktft_oled (KlipperTFT-Pico 0.6.1); this file feeds
// them from Skipper Flipper's own state: the board link (sf_link_get) and the machine
// (sf_display_get, filled by the link task from ph::Machine::display_info, unit-tested on the PC).
//   no board / link off / printer.cfg problem: the reason, the WiFi name, and (every 4 s in turn)
//     the WiFi password and the page address
//   board running: hotend and bed; while printing the percentage and a bar; HEATING, HOMING,
//     EXTRUDING, RETRACTING, LOADING, UNLOADING flash up 2 s in every 4 s; a shutdown says so.
// Low priority, its own task, 5 frames a second (~25 ms of I2C each). Not yet run on a real display.
#include <stdio.h>
#include <string.h>

#include <string>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "oled_i2c.hpp"
#include "sdkconfig.h"
#include "sf.h"

static const char *TAG = "oled";
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static void *volatile s_bus;
static volatile bool s_paused;

extern "C" void *sf_oled_bus(void)
{
    return s_bus;
}

extern "C" void sf_oled_pause(bool pause)
{
    s_paused = pause;
}
static char s_desc[64] = "off";
static sf_settings_t s_set;

static void set_desc(const char *d)
{
    portENTER_CRITICAL(&s_mux);
    strlcpy(s_desc, d, sizeof s_desc);
    portEXIT_CRITICAL(&s_mux);
}

extern "C" void sf_oled_desc(char *out, size_t n)
{
    portENTER_CRITICAL(&s_mux);
    strlcpy(out, s_desc, n);
    portEXIT_CRITICAL(&s_mux);
}

extern "C" const char *sf_oled_pin_problem(const sf_settings_t *s)
{
    if (s->oled_type == SF_OLED_OFF) {
        return NULL;
    }
    int p[2] = {s->oled_sda, s->oled_scl};
    if (p[0] == p[1]) {
        return "OLED SDA and SCL must be two different pins";
    }
    for (int i = 0; i < 2; i++) {
        int g = p[i];
        if (!GPIO_IS_VALID_OUTPUT_GPIO(g)) return "OLED pins must be valid output GPIOs";
        if (g == 19 || g == 20) return "GPIO19/20 are the USB port; pick other OLED pins";
        if (g == 0 || g == 3 || g == 45 || g == 46) return "GPIO0, 3, 45 and 46 are start-up (strapping) pins; pick other OLED pins";
        if (g >= 26 && g <= 37) return "GPIO26-37 belong to the flash and PSRAM on most ESP32-S3 boards; pick other OLED pins";
        if (s->link_enabled && (g == s->uart_tx || g == s->uart_rx)) return "an OLED pin is one of the printer board UART pins";
        if (s->led_type != SF_LED_NONE && g == s->led_gpio) return "an OLED pin is the status LED pin";
        if (s->sd_enabled && (g == s->sd_cs || g == s->sd_sck || g == s->sd_mosi || g == s->sd_miso))
            return "an OLED pin is one of the SD card pins";
    }
    return NULL;
}

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// No board, link off, or printer.cfg not accepted: what is wrong and how to reach the pages.
static void no_board(ktft::DisplayState &d, const char *title, const char *reason, uint32_t now)
{
    d.link = ktft::DisplayState::Link::NoKlipper;
    d.title = title;
    std::string l2, l3;
    ktft::wrap_small(reason, d.line1, l2, l3);   // the first line of the reason
    d.line2 = sf_ap_ssid();
    const char *ip = sf_sta_ip();
    char b[80];
    if ((now / 4000) % 2 == 0) {
        snprintf(b, sizeof b, "pass %s", CONFIG_SKIPPER_AP_PASSWORD);
    } else {
        snprintf(b, sizeof b, "%s", ip[0] ? ip : "192.168.4.1");
    }
    d.line3 = b;
}

static bool sensor_temp(const sf_link_info_t &li, const char *const *names, float &c)
{
    for (int i = 0; i < li.n_temps && i < SF_MAX_TEMPS; i++) {
        for (int k = 0; names[k]; k++) {
            if (li.temps[i].state == 1 && strcmp(li.temps[i].name, names[k]) == 0) {
                c = li.temps[i].c;
                return true;
            }
        }
    }
    return false;
}

static void fill(ktft::DisplayState &d, uint32_t now)
{
    static sf_link_info_t li;   // large: not on the task's stack
    sf_display_t dp;
    sf_link_get(&li);
    sf_display_get(&dp);
    d = ktft::DisplayState();
    if (!s_set.link_enabled) {
        no_board(d, "NO BOARD", "board link switched off", now);
        return;
    }
    if (li.state != SF_LINK_OK) {
        no_board(d, "NO BOARD", li.last_error[0] ? li.last_error : "looking for the printer board", now);
        return;
    }
    if (!li.cfg_ok) {
        no_board(d, "CHECK CFG", li.cfg_status[0] ? li.cfg_status : "printer.cfg not accepted", now);
        return;
    }
    d.link = ktft::DisplayState::Link::Ok;
    d.st.state = dp.state[0] ? dp.state : "standby";
    d.st.progress = dp.progress;
    d.activity = dp.activity;
    char pm[16];
    if (sf_panel_msg(pm, sizeof pm)) d.activity = pm;   // a button was just pressed
    d.st.ext_temp = dp.ext_temp, d.st.ext_target = dp.ext_target;
    d.st.bed_temp = dp.bed_temp, d.st.bed_target = dp.bed_target;
    // without [extruder] / [heater_bed]: a [temperature_sensor hotend] / [temperature_sensor bed]
    static const char *const EXT[] = {"hotend", "extruder", NULL}, *const BED[] = {"bed", "heater_bed", NULL};
    float c;
    bool ext = dp.have_ext, bed = dp.have_bed;
    if (!ext && sensor_temp(li, EXT, c)) ext = true, d.st.ext_temp = c, d.st.ext_target = 0;
    if (!bed && sensor_temp(li, BED, c)) bed = true, d.st.bed_temp = c, d.st.bed_target = 0;
    d.have_status = ext || bed || d.st.state == "shutdown" || d.st.state == "printing" || d.st.state == "paused";
    if (!d.have_status) {
        d.line1 = "no temperatures yet";
        d.line2 = sf_ap_ssid();
        d.line3 = sf_sta_ip()[0] ? sf_sta_ip() : "192.168.4.1";
    }
}

static ktft::OledKind kind_of(int t)
{
    return t == SF_OLED_SSD1306_32 ? ktft::OledKind::Ssd1306_32
         : t == SF_OLED_SH1106     ? ktft::OledKind::Sh1106
                                   : ktft::OledKind::Ssd1306_64;
}

static void oled_task(void *arg)
{
    static ktft::OledI2c oled;    // static: kept off the task stack
    static ktft::OledFrame frame;
    static ktft::DisplayState st;
    // Look for it three times over the first 10 s (a display powered from the printer board may
    // start later than the ESP32); after that, give the memory back until the next restart.
    int tries = 0;
    esp_log_level_set("i2c.master", ESP_LOG_NONE);   // "probe device timeout" on every try: our own line says it once
    while (!oled.begin(s_set.oled_sda, s_set.oled_scl, kind_of(s_set.oled_type), s_set.oled_flip)) {
        set_desc(oled.description().c_str());
        s_bus = oled.bus();   // kept: an accelerometer may sit on these two wires on its own
        if (++tries >= 3) {
            ESP_LOGW(TAG, "no OLED: %s (Settings page: switch it off, or check the wiring)", oled.description().c_str());
            oled.end();
            esp_log_level_set("i2c.master", ESP_LOG_WARN);
            vTaskDelete(NULL);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    esp_log_level_set("i2c.master", ESP_LOG_WARN);
    s_bus = oled.bus();
    set_desc(oled.description().c_str());
    ESP_LOGW(TAG, "OLED: %s", oled.description().c_str());
    frame.set_height(oled.height());
    int fails = 0;
    bool shown_pause = false;
    for (;;) {
        uint32_t now = now_ms();
        fill(st, now);
        ktft::render_display(frame, st, now);
        if (s_paused) {   // the accelerometer has the bus: one "SHAKING" frame, then nothing until it is done
            if (!shown_pause) {
                st.activity = "SHAKING";
                ktft::render_display(frame, st, 2500);   // 2.5 s: the flashing word is showing
                oled.show(frame);
                shown_pause = true;
            }
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        shown_pause = false;
        if (oled.show(frame)) {
            fails = 0;
        } else if (++fails == 10) {   // two seconds of failed frames: unplugged? look again every 5 s
            set_desc("stopped answering; looking again");
            ESP_LOGW(TAG, "OLED stopped answering");
            while (!oled.begin(s_set.oled_sda, s_set.oled_scl, kind_of(s_set.oled_type), s_set.oled_flip)) {
                vTaskDelay(pdMS_TO_TICKS(5000));
            }
            set_desc(oled.description().c_str());
            fails = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

extern "C" void sf_oled_start(const sf_settings_t *s)
{
    s_set = *s;
    if (s->oled_type == SF_OLED_OFF) {
        set_desc("off");
        return;
    }
    const char *bad = sf_oled_pin_problem(s);
    if (bad) {
        set_desc(bad);
        ESP_LOGW(TAG, "OLED not started: %s", bad);
        return;
    }
    set_desc("looking for it");
    xTaskCreate(oled_task, "oled", 6144, NULL, 1, NULL);   // 4 KB was untested headroom (review 2026-10-24)
}
