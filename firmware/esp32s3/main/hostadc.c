// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Temperature sensors on the ESP32-S3 itself (2026-10-27): [temperature_sensor NAME] with
// sensor_pin: esp32:gpioN, e.g. a chamber thermistor. ADC1 only (GPIO1-10: ADC2 does not work
// while WiFi is on), 12 dB attenuation, Espressif's curve-fitting calibration, 16 samples averaged.
// Wiring: thermistor from the pin to GND, the pull-up resistor (pullup_resistor, default 100 kohm
// for esp32: pins) from 3V3 to the pin. Returns the voltage as a fraction of 3.3 V; at or above the
// ADC's top (about 3.1 V) it says 0.999, which the machine shows as "no sensor? (open circuit)".
#include <string.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "sf.h"

#if CONFIG_SKIPPER_HOST_ADC   // CAPTAIN / SKIPPER: DECKHAND has no room for the ADC driver (2026-10-27)

static const char *TAG = "hostadc";
static adc_oneshot_unit_handle_t s_unit;
static adc_cali_handle_t s_cali;
static uint16_t s_ready;      // bit per channel configured
static uint16_t s_refused;    // bit per GPIO already reported as unusable
static sf_settings_t s_set;
static bool s_have_set;

void sf_host_adc_settings(const sf_settings_t *s)
{
    s_set = *s;
    s_have_set = true;
}

static const char *pin_problem(int g)
{
    if (g < 1 || g > 10) return "not an ADC1 pin (GPIO1-10)";
    if (!s_have_set) return NULL;
    const sf_settings_t *s = &s_set;
    if (s->link_enabled && (g == s->uart_tx || g == s->uart_rx)) return "a printer board UART pin";
    if (s->led_type != SF_LED_NONE && g == s->led_gpio) return "the status LED pin";
    if (s->sd_enabled && (g == s->sd_cs || g == s->sd_sck || g == s->sd_mosi || g == s->sd_miso)) return "an SD card pin";
    if (s->oled_type != SF_OLED_OFF && (g == s->oled_sda || g == s->oled_scl)) return "an OLED pin";
    if (sf_panel_uses_pin(g)) return "a button pin";
    if (g == 3) return "a start-up (strapping) pin";
    return NULL;
}

bool sf_host_adc(int gpio, float *ratio)
{
    const char *bad = pin_problem(gpio);
    if (bad) {
        if (gpio >= 0 && gpio < 16 && !(s_refused & (1u << gpio))) {
            s_refused |= (uint16_t)(1u << gpio);
            ESP_LOGW(TAG, "esp32:gpio%d cannot be a temperature sensor: %s", gpio, bad);
        }
        return false;
    }
    if (!s_unit) {
        adc_oneshot_unit_init_cfg_t u = {.unit_id = ADC_UNIT_1};
        if (adc_oneshot_new_unit(&u, &s_unit) != ESP_OK) return false;
        adc_cali_curve_fitting_config_t cc = {.unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
        if (adc_cali_create_scheme_curve_fitting(&cc, &s_cali) != ESP_OK) s_cali = NULL;
    }
    adc_channel_t ch = (adc_channel_t)(gpio - 1);   // ESP32-S3: GPIO1-10 = ADC1 channels 0-9
    if (!(s_ready & (1u << ch))) {
        adc_oneshot_chan_cfg_t c = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
        if (adc_oneshot_config_channel(s_unit, ch, &c) != ESP_OK) return false;
        s_ready |= (uint16_t)(1u << ch);
    }
    int sum = 0, n = 0;
    for (int k = 0; k < 16; k++) {
        int raw = 0, mv = 0;
        if (adc_oneshot_read(s_unit, ch, &raw) != ESP_OK) continue;
        if (s_cali && adc_cali_raw_to_voltage(s_cali, raw, &mv) == ESP_OK) sum += mv;
        else sum += raw * 3100 / 4095;
        n++;
    }
    if (!n) return false;
    float mv = (float)sum / n;
    *ratio = mv >= 3050 ? 0.999f : mv / 3300.0f;   // the ADC tops out near 3.1 V: treat as open circuit
    return true;
}

#else
void sf_host_adc_settings(const sf_settings_t *s) { (void)s; }
bool sf_host_adc(int gpio, float *ratio) { (void)gpio, (void)ratio; return false; }
#endif
