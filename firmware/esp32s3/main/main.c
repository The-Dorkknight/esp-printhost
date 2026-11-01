// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
//
// Skipper Flipper ESP32-S3 firmware: a WiFi access point (and optionally the home WiFi) with web
// pages, a Klipper link to the printer board over UART, printer.cfg, heaters, fans, motors and
// printing from flash or an SD card. Built in three tiers by board memory (tier.c).
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "esp_chip_info.h"
#include "esp_event.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dns_server.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "sf.h"


// WPA2 needs 8..63 characters; sizeof counts the terminating NUL.
_Static_assert(sizeof(CONFIG_SKIPPER_AP_PASSWORD) >= 9 && sizeof(CONFIG_SKIPPER_AP_PASSWORD) <= 64,
               "CONFIG_SKIPPER_AP_PASSWORD must be 8 to 63 characters");

static const char *TAG = "skipper";
static char s_ssid[33];
static sf_settings_t s_settings;

static uint32_t uptime_s(void) { return (uint32_t)(esp_timer_get_time() / 1000000); }

// ---- live console: everything the firmware logs is also kept in a ring buffer that the web
// page /console reads over WiFi. s_total counts bytes ever written, so a client asks for "since N".
// 8 KB, or CONFIG_SKIPPER_LOG_RING_KB from PSRAM on the CAPTAIN / SKIPPER builds.
static char *s_ring;
static uint32_t s_ring_size;
static uint32_t s_total;
static portMUX_TYPE s_ring_mux = portMUX_INITIALIZER_UNLOCKED;
static vprintf_like_t s_prev_vprintf;

static void ring_append(const char *p, size_t n)
{
    if (!s_ring) {
        return;
    }
    portENTER_CRITICAL(&s_ring_mux);
    for (size_t i = 0; i < n; i++) {
        s_ring[s_total % s_ring_size] = p[i];
        s_total++;
    }
    portEXIT_CRITICAL(&s_ring_mux);
}

static void ring_alloc(void)
{
    uint32_t want = (uint32_t)CONFIG_SKIPPER_LOG_RING_KB * 1024;
    if (want > 8192 && sf_psram_bytes()) {
        s_ring = heap_caps_malloc(want, MALLOC_CAP_SPIRAM);
    }
    if (!s_ring) {
        want = 8192;
        s_ring = malloc(want);
    }
    s_ring_size = s_ring ? want : 0;
}

static int sf_vprintf(const char *fmt, va_list ap)
{
    char buf[256];
    va_list ap2;
    va_copy(ap2, ap);
    int n = vsnprintf(buf, sizeof buf, fmt, ap2);
    va_end(ap2);
    if (n > 0 && !xPortInIsrContext()) {
        size_t k = (size_t)n < sizeof buf ? (size_t)n : sizeof buf - 1;
        ring_append(buf, k);
        sf_log_tee(buf, k);
        sf_lastwords_add(buf, k);
    }
    return s_prev_vprintf ? s_prev_vprintf(fmt, ap) : vprintf(fmt, ap);
}

static void heartbeat_task(void *arg)
{
    uint32_t ticks = 0;
    for (;;) {
        if (++ticks % 5 == 0) {
            // the lowest it has ever been shows a slow leak (it keeps creeping down) at a glance
            ESP_LOGW(TAG, "alive: uptime %us, free heap %u bytes (lowest %u, biggest block %u)", (unsigned)uptime_s(),
                     (unsigned)esp_get_free_heap_size(), (unsigned)esp_get_minimum_free_heap_size(),
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ---- home WiFi (station), next to the ESP32's own network
static char s_sta_ip[16];
static const char *s_sta_state = "off";
static int s_sta_fails;
static esp_timer_handle_t s_sta_retry;

const char *sf_ap_ssid(void)
{
    return s_ssid;
}

const char *sf_sta_ip(void)
{
    return s_sta_ip;
}

const char *sf_sta_state(void)
{
    return s_sta_state;
}

static void sta_retry_cb(void *arg)
{
    esp_wifi_connect();
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        snprintf(s_sta_ip, sizeof s_sta_ip, IPSTR, IP2STR(&e->ip_info.ip));
        s_sta_state = "connected";
        s_sta_fails = 0;
        ESP_LOGW(TAG, "home WiFi \"%s\": connected, pages also at http://%s/", s_settings.sta_ssid, s_sta_ip);
        return;
    }
    if (base != WIFI_EVENT) {
        return;
    }
    if (id == WIFI_EVENT_STA_START) {
        s_sta_state = "connecting";
        esp_wifi_connect();
        return;
    }
    if (id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
        if (s_sta_ip[0]) {
            ESP_LOGW(TAG, "home WiFi lost (reason %d); trying again", e->reason);
        }
        s_sta_ip[0] = 0;
        s_sta_fails++;
        s_sta_state = (e->reason == WIFI_REASON_AUTH_FAIL || e->reason == WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT ||
                       e->reason == WIFI_REASON_HANDSHAKE_TIMEOUT) ? "wrong password?"
                      : e->reason == WIFI_REASON_NO_AP_FOUND ? "network not found (2.4 GHz only)"
                      : "not connected, retrying";
        if (s_sta_fails == 1 || s_sta_fails % 10 == 0) {
            ESP_LOGW(TAG, "home WiFi \"%s\": %s (reason %d)", s_settings.sta_ssid, s_sta_state, e->reason);
        }
        // back off: 2 s, 4 s, ... up to a minute, so the ESP32's own network stays usable
        int delay_s = s_sta_fails < 5 ? 2 * s_sta_fails : 60;
        if (s_sta_retry) {
            esp_timer_stop(s_sta_retry);
            esp_timer_start_once(s_sta_retry, (uint64_t)delay_s * 1000000);
        }
        return;
    }
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
        ESP_LOGW(TAG, "client joined: " MACSTR, MAC2STR(e->mac));
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)data;
        ESP_LOGW(TAG, "client left: " MACSTR, MAC2STR(e->mac));
    }
}

static void wifi_start_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    bool sta = s_settings.sta_ssid[0] != 0;
    if (sta) {
        esp_netif_create_default_wifi_sta();
    }

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP));
    wifi_config_t cfg = {0};
    snprintf(s_ssid, sizeof s_ssid, "SkipperFlipper-%02X%02X", mac[4], mac[5]);
    strlcpy((char *)cfg.ap.ssid, s_ssid, sizeof cfg.ap.ssid);
    cfg.ap.ssid_len = (uint8_t)strlen(s_ssid);
    strlcpy((char *)cfg.ap.password, CONFIG_SKIPPER_AP_PASSWORD, sizeof cfg.ap.password);
    cfg.ap.channel = CONFIG_SKIPPER_AP_CHANNEL;
    cfg.ap.max_connection = 4;
    cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(sta ? WIFI_MODE_APSTA : WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &cfg));
    if (sta) {
        wifi_config_t sc = {0};
        strlcpy((char *)sc.sta.ssid, s_settings.sta_ssid, sizeof sc.sta.ssid);
        strlcpy((char *)sc.sta.password, s_settings.sta_pass, sizeof sc.sta.password);
        sc.sta.threshold.authmode = s_settings.sta_pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
        sc.sta.pmf_cfg.capable = true;
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sc));
        const esp_timer_create_args_t ta = {.callback = sta_retry_cb, .name = "sta_retry"};
        esp_timer_create(&ta, &s_sta_retry);
    }
    ESP_ERROR_CHECK(esp_wifi_start());
    // 15 dBm instead of the ~20 dBm default: plenty for one room, and smaller current spikes on a
    // weak 5 V supply (full-power WiFi bursts are the usual cause of ESP32 brownout resets).
    esp_wifi_set_max_tx_power(60);
    ESP_LOGW(TAG, "WiFi AP \"%s\" (WPA2) on channel %d, page at http://192.168.4.1/", s_ssid,
             CONFIG_SKIPPER_AP_CHANNEL);
    if (sta) {
        // (once joined, the ESP32's own network moves to the home network's channel: one radio)
        ESP_LOGW(TAG, "also joining the home WiFi \"%s\"", s_settings.sta_ssid);
    }
}

static const char *reset_reason_str(void)
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "power on";
    case ESP_RST_SW: return "software restart (settings saved)";
    case ESP_RST_PANIC: return "CRASH (panic) - please report";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT: return "CRASH (watchdog) - please report";
    case ESP_RST_BROWNOUT: return "brownout (power supply dipped)";
    case ESP_RST_USB: return "USB reset (flashing)";
    case ESP_RST_EXT: return "reset button";
    default: return "other";
    }
}

static const char *chip_model(void)
{
    esp_chip_info_t ci;
    esp_chip_info(&ci);
    return ci.model == CHIP_ESP32S3 ? "ESP32-S3" : "NOT an ESP32-S3";
}

static esp_err_t root_get(httpd_req_t *req)
{
    uint32_t flash = 0;
    esp_flash_get_size(NULL, &flash);
    sf_link_info_t li;
    sf_link_get(&li);
    // static: the HTTP server task stack is small, and handlers run one at a time
    static char board[2600];
    if (li.state == SF_LINK_OK) {
        static char led[400];
        static char temps[800];
        led[0] = 0;
        temps[0] = 0;
        for (int i = 0; i < li.n_temps && i < SF_MAX_TEMPS; i++) {
            size_t len = strlen(temps);
            snprintf(temps + len, sizeof temps - len, "<tr><td>Temp %s</td><td>%s</td></tr>", li.temps[i].name,
                     li.temps[i].text);
        }
        if (li.cfg_ok && li.first_led[0]) {
            snprintf(led, sizeof led,
                     "<tr><td>LED %s</td><td>"
                     "<a href=\"/board_led?c=red\">red</a> &middot; <a href=\"/board_led?c=green\">green</a> "
                     "&middot; <a href=\"/board_led?c=blue\">blue</a> &middot; <a href=\"/board_led?c=white\">white"
                     "</a> &middot; <a href=\"/board_led?c=off\">off</a></td></tr>",
                     li.first_led);
        }
        snprintf(board, sizeof board,
                 "<tr><td>Printer board</td><td>connected: Klipper %s</td></tr>"
                 "<tr><td>Board MCU</td><td>%s, %u Hz nominal clock</td></tr>"
                 "<tr><td>Board clock</td><td>%u Hz measured (%+.1f ppm), best round trip %.2f ms, %u samples</td></tr>"
                 "<tr><td>Board state</td><td>%s%s, %u CRC errors</td></tr>"
                 "<tr><td>printer.cfg</td><td>%s (<a href=\"/config\">edit</a>)</td></tr>%s%s",
                 li.version, li.mcu, (unsigned)li.clock_freq, (unsigned)li.clk_hz, (double)li.drift_ppm,
                 (double)li.rtt_ms, (unsigned)li.clk_samples, li.configured ? "configured" : "not configured",
                 li.shutdown ? ", SHUT DOWN" : "", (unsigned)li.crc_errors,
                 li.cfg_status[0] ? li.cfg_status : "waiting", temps, led);
    } else {
        snprintf(board, sizeof board, "<tr><td>Printer board</td><td>%s%s%s (attempt %u)</td></tr>",
                 sf_link_state_name(li.state), li.last_error[0] ? ": " : "", li.last_error, (unsigned)li.attempts);
    }
    // Raw wire diagnostics: what actually went over the UART.
    static char diag[1400];
    bool all_zero = li.rx_bytes > 0 && li.rx_hex[0];
    for (const char *p = li.rx_hex; *p && all_zero; p++) {
        if (*p != '0' && *p != ' ') all_zero = false;
    }
    const char *hint = li.loopback ? "The ESP32 hears its own transmission: its TX is connected to its RX. "
                                     "(If you did that on purpose for the self-test, the ESP32 side works.)"
                       : li.echo_suspected && li.state != SF_LINK_OK
                       ? "What comes back is a garbled copy of the ESP32's own requests, not replies from the board. "
                         "The ESP32's TX signal is leaking into its RX wire: check the two signal wires are not "
                         "touching or shorted (test with a multimeter), that RX goes to the board's TX pin and not "
                         "its RX pin, and avoid damaged cable. The board itself is not answering."
                       : li.rx_bytes == 0 ? "Nothing received at all: check the RX wire (board TX to ESP32 RX), GND, "
                                            "that the board is powered, and that it runs the serial firmware. "
                                            "Try swapping the TX and RX wires."
                       : li.state == SF_LINK_OK ? ""
                       : all_zero ? "Only 00 bytes arrive: the board's TX wire is held low (a UART 'break'). The board "
                                    "is off, browning out, or hung. Switch the printer's 12/24 V supply off and unplug "
                                    "the board's USB for 10 seconds, then on again (the ESP32 lets go of its TX wire "
                                    "while it waits, so it does not keep the board half-powered). If it stays like "
                                    "this, check the board's power and that its TX wire is not shorted to GND."
                       : "Bytes arrive but are not valid Klipper replies: check the baud rate (250000) "
                         "and that both boards share GND.";
    snprintf(diag, sizeof diag,
             "<tr><td>UART bytes</td><td>sent %u, received %u, CRC errors %u</td></tr>"
             "<tr><td>Last bytes in</td><td><code>%s</code></td></tr>%s%s%s",
             (unsigned)li.tx_bytes, (unsigned)li.rx_bytes, (unsigned)li.crc_errors,
             li.rx_hex[0] ? li.rx_hex : "(none)", hint[0] ? "<tr><td>Hint</td><td>" : "", hint,
             hint[0] ? "</td></tr>" : "");
    static char crash_link[260];
    crash_link[0] = 0;
    if (sf_crash_short()[0]) {
        snprintf(crash_link, sizeof crash_link, ": %.160s (<a href=\"/crash\">details</a>)", sf_crash_short());
    }
    static char oled[64];
    sf_oled_desc(oled, sizeof oled);
    static char home[120];
    if (!s_settings.sta_ssid[0]) {
        snprintf(home, sizeof home, "not set (<a href=\"/settings\">Settings</a>)");
    } else if (s_sta_ip[0]) {
        snprintf(home, sizeof home, "%.32s: <a href=\"http://%s/\">http://%s/</a>", s_settings.sta_ssid, s_sta_ip, s_sta_ip);
    } else {
        snprintf(home, sizeof home, "%.32s: %s", s_settings.sta_ssid, s_sta_state);
    }
    // from the heap per request (not static): DECKHAND needs its internal RAM for the printer link
    const size_t bufn = 11000;
    char *buf = malloc(bufn);
    if (!buf) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
    }
    snprintf(buf, bufn,
             "<!doctype html><html><head><meta charset=\"utf-8\">"
             "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
             "<title>Skipper Flipper</title><style>"
             "body{background:#e9e6dc;color:#141414;font:16px/1.5 Georgia,serif;margin:0}"
             ".b{background:#141414;color:#e9e6dc;text-align:center;padding:10px;font-weight:700;"
             "letter-spacing:.06em}main{max-width:640px;margin:0 auto;padding:16px}"
             "h1{font:700 2rem Impact,Arial Black,sans-serif;border-bottom:4px solid #141414}"
             "table{border-collapse:collapse;width:100%%;background:#f7f5ee}"
             "td{border:2px solid #141414;padding:6px 10px}td:first-child{font-weight:700;width:40%%}"
             "input,button{font:inherit;padding:4px;border:2px solid #141414;background:#f7f5ee}"
             "#co{background:#141414;color:#e9e6dc;padding:8px;min-height:3em;white-space:pre-wrap;margin:6px 0}"
             "</style></head><body>" SF_NAV "<div class=\"b\">" SF_BANNER "</div><main>"
             "<h1>Skipper Flipper</h1><p><a href=\"/about\" style=\"color:inherit\"><b>%s</b></a></p>"
             "<p>ESP32-S3 host. It talks Klipper to the printer board and runs, from "
             "printer.cfg: heaters (with Klipper's safety checks), fans, LEDs, TMC2209 drivers and motor tests. "
             "Moves, homing and printing are built and simulation-tested; not yet confirmed on a printer.</p>"
             "<p><a href=\"/test\" style=\"display:inline-block;background:#141414;color:#e9e6dc;padding:8px 16px;"
             "font-weight:700;text-decoration:none\">TEST BENCH &rarr;</a> <a href=\"/printer\" style=\"display:inline-block;"
             "background:#141414;color:#e9e6dc;padding:8px 16px;font-weight:700;text-decoration:none\">PRINTER &rarr;</a></p><div id=\"tp\"></div><table>"
             "<tr><td>Firmware</td><td>" SF_VERSION "</td></tr>"
             "<tr><td>Chip</td><td>%s</td></tr>"
             "<tr><td>Flash</td><td>%u KiB</td></tr>"
             "<tr><td>Uptime</td><td>%u s</td></tr>"
             "<tr><td>Free heap</td><td>%u bytes (lowest since start %u)</td></tr>"
             "<tr><td>WiFi network</td><td>%s (this page: 192.168.4.1)</td></tr>"
             "<tr><td>Home WiFi</td><td>%s</td></tr>"
             "<tr><td>OLED</td><td>%s</td></tr>"
             "<tr><td>ESP-IDF</td><td>%s</td></tr>"
             "<tr><td>Last restart</td><td>%s%s</td></tr>%s%s"
             "</table><h2>Console</h2><form id=\"cf\"><input id=\"ci\" size=\"28\" placeholder=\"SET_LED LED=board_led RED=1\" "
             "autocomplete=\"off\"> <button>Send</button></form><pre id=\"co\">Type HELP for the commands that work.</pre>"
             "<script>document.getElementById('cf').onsubmit=function(e){e.preventDefault();"
             "var i=document.getElementById('ci'),o=document.getElementById('co'),c=i.value;if(!c)return;"
             "o.textContent+='\\n> '+c;fetch('/gcode?cmd='+encodeURIComponent(c),{cache:'no-store'})"
             ".then(function(r){return r.text()}).then(function(t){o.textContent+='\\n'+t})"
             ".catch(function(){o.textContent+='\\n(no answer from the ESP32)'});i.value=''};</script>"
             "<script src=\"/ui.js\"></script><script>function tp(){fetch('/status.json',{cache:'no-store'})"
             ".then(function(r){return r.json()}).then(function(j){if(j.machine)sfTemps(document.getElementById('tp'),"
             "j.machine.sensors,j.machine.shutdown);else sfTempsLost(document.getElementById('tp'))}).catch(function(){sfTempsLost(document.getElementById('tp'))}).then(function(){setTimeout(tp,1500)})}tp();</script>"
             "<p><small>ESP32 self-test: disconnect the board wires and join the ESP32's TX and RX "
             "pins (GPIO43 and GPIO44 by default) with one wire. Within 5 seconds this page should say it "
             "hears its own transmission.</small></p><p><a href=\"/console\">Live console</a> &middot; <a href=\"/settings\">Settings</a> "
             "&middot; <a href=\"/config\">printer.cfg</a> &middot; <a href=\"/test\">Test bench</a> "
             "&middot; <a href=\"/about\">About this build</a> &middot; <a href=\"/update\">Firmware update</a> "
             "&middot; <a href=\"/status.json\">status.json</a></p>"
             "</main></body></html>",
             sf_tier_title(), chip_model(), (unsigned)(flash / 1024), (unsigned)uptime_s(),
             (unsigned)esp_get_free_heap_size(), (unsigned)esp_get_minimum_free_heap_size(), s_ssid, home, oled,
             esp_get_idf_version(), reset_reason_str(), crash_link, board, diag);
    httpd_resp_set_type(req, "text/html");
    esp_err_t r = httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    free(buf);
    return r;
}

static void chunk(httpd_req_t *req, const char *s);
static bool url_decode(char *s);

// Shared page script (/ui.js): the temperature boxes and the command helper.
// Temperature box: five lines tall. Heating: the reading scrolls up; cooling: down; settled: five
// copies standing still. Colour: blue cold, orange heating, red hot / at temperature, purple cooling.
// The state is also written out in words, and phones set to "reduce motion" get no scrolling.
static const char UI_JS[] =
    "(function(){var st=document.createElement('style');st.textContent="
    "'.tc{display:inline-block;margin:6px 12px 6px 0;vertical-align:top;text-align:center}"
    ".tb{height:7.5em;width:7.6em;overflow:hidden;border:3px solid currentColor;background:#f7f5ee;"
    "font:700 1em/1.5em ui-monospace,Menlo,Consolas,monospace;margin:0 auto}.tr span{display:block}"
    ".s-heating .tr{animation:sfup 3s linear infinite}.s-cooling .tr{animation:sfdn 3s linear infinite}"
    "@keyframes sfup{from{transform:translateY(0)}to{transform:translateY(-7.5em)}}"
    "@keyframes sfdn{from{transform:translateY(-7.5em)}to{transform:translateY(0)}}"
    "@media (prefers-reduced-motion:reduce){.tr{animation:none!important}}"
    ".s-cold{color:#1d5fd1}.s-heating{color:#d96a00}.s-holding,.s-hot{color:#c4161c}.s-cooling{color:#7b3fbf}"
    ".s-fault{color:#141414;background:#ffd400}.tl{font-size:.85em;max-width:9.5em;margin:4px auto}"
    ".s-error{color:#ffd400;background:#141414;border-color:#c4161c}.s-error .tr span+span{color:#ff4d4d}"
    ".s-error .tr{animation:none!important}';"
    "document.head.appendChild(st);"
    "var W={cold:'cold',hot:'hot',heating:'heating',holding:'at temperature',cooling:'cooling',fault:'sensor fault'};"
    "window.sfTemps=function(box,list,err){(list||[]).forEach(function(s){var id='t_'+s.name,c=document.getElementById(id);"
    "if(!c){c=document.createElement('div');c.className='tc';c.id=id;"
    "c.innerHTML='<div class=\"tb\"><div class=\"tr\"></div></div><div class=\"tl\"></div>';box.appendChild(c);"
    "for(var k=0;k<10;k++)c.querySelector('.tr').appendChild(document.createElement('span'))}"
    "var bad=err||s.state=='fault',t=s.state=='fault'?'----':s.temp.toFixed(1)+'\\u00b0C',sp=c.querySelectorAll('.tr span');"
    "for(var k=0;k<sp.length;k++)sp[k].textContent=(bad&&k>0)?'ERROR':t;"
    "var cl='tb s-'+(bad?'error':s.state),tb=c.querySelector('.tb');if(tb.className!=cl)tb.className=cl;"
    "var l=s.name+': '+(err?'STOPPED (last reading)':(W[s.state]||s.state));if(!err&&s.target>0)l+=' \\u2192 '+s.target+'\\u00b0C';"
    "if(!err&&s.heater&&s.power>0)l+=', power '+Math.round(s.power*100)+'%';if(s.state=='fault')l+=' ('+s.text+')';"
    "c.querySelector('.tl').textContent=l})};"
    "window.sfTempsLost=function(box){var b=box.querySelectorAll('.tc');for(var i=0;i<b.length;i++){"
    "var tb=b[i].querySelector('.tb'),sp=b[i].querySelectorAll('.tr span');tb.className='tb s-error';"
    "for(var k=1;k<sp.length;k++)sp[k].textContent='ERROR';"
    "b[i].querySelector('.tl').textContent=b[i].id.slice(2)+': no board (last reading)'}};"
    "if(!window.sfT){window.sfT=1;fetch('/time?t='+Date.now(),{method:'POST'}).catch(function(){})}"
    "window.sfCmd=function(cmd,out){return fetch('/gcode?cmd='+encodeURIComponent(cmd),{cache:'no-store'})"
    ".then(function(r){return r.text()}).then(function(t){if(out)out.textContent='> '+cmd+'\\n'+t+'\\n\\n'+"
    "out.textContent.slice(0,6000);return t}).catch(function(){if(out)out.textContent='> '+cmd+"
    "'\\n(no answer from the ESP32)\\n\\n'+out.textContent})}})();";

static esp_err_t ui_js_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/javascript");
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=60");
    return httpd_resp_send(req, UI_JS, HTTPD_RESP_USE_STRLEN);
}

// The test bench: one row per subsystem with Run and Abort, STOP for everything, live temperatures.
static const char TEST_HTML[] =
    "<!doctype html><html><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>Skipper Flipper test bench</title><style>"
    "body{background:#e9e6dc;color:#141414;font:16px/1.5 Georgia,serif;margin:0}"
    ".b{background:#141414;color:#e9e6dc;text-align:center;padding:10px;font-weight:700;letter-spacing:.06em}"
    "main{max-width:760px;margin:0 auto;padding:16px}h1{font:700 2rem Impact,Arial Black,sans-serif;"
    "border-bottom:4px solid #141414;margin-bottom:8px}table{border-collapse:collapse;width:100%;background:#f7f5ee}"
    "td{border:2px solid #141414;padding:6px 8px;vertical-align:top}td.ac{white-space:nowrap;text-align:right}"
    "button,input{font:inherit;border:2px solid #141414;background:#f7f5ee;padding:3px 8px;margin:2px}"
    "button.run{background:#141414;color:#e9e6dc}button.ab{background:#c4161c;color:#fff;border-color:#c4161c}"
    "button.ab:disabled{opacity:.35}input{width:3.6em}"
    ".stop{display:block;width:100%;font:700 1.4rem Impact,Arial Black,sans-serif;background:#c4161c;color:#fff;"
    "border:4px solid #141414;padding:12px;margin:10px 0;letter-spacing:.05em}"
    ".sd{background:#ffd400;border:3px solid #141414;padding:8px;margin:8px 0}"
    ".st{font-weight:700}.running{color:#d96a00}.passed{color:#1a7f37}.failed,.aborted{color:#c4161c}"
    ".dt{font-size:.85em}#out{background:#141414;color:#e9e6dc;padding:8px;min-height:3em;white-space:pre-wrap}"
    "@media(max-width:560px){td.ac{white-space:normal;text-align:left}}"
    ".dg{display:inline-block;min-width:5.5em;text-align:center;padding:1px 6px;border:2px solid #141414;font-weight:700;"
    "background:#ccc}.dg.on{background:#c4161c;color:#fff}.dg.off{background:#2e7d32;color:#fff}"
    ".cal{border-top:2px solid #141414;padding:6px 0}.cal input{width:5.5em}.cal input[type=range]{width:9em;vertical-align:middle}"
    ".sgb{background:#f7f5ee;border:2px solid #141414;padding:6px 8px;margin:6px 0}"
    "button.y{background:#ffd400;color:#141414;font-weight:700;border:2px solid #141414;padding:6px 12px;margin:6px 6px 2px 0}"
    "</style></head><body>" SF_NAV "<div class=\"b\">" SF_BANNER "</div><main><h1>Test bench</h1>"
    "<p>These buttons really heat and move things: stay with the printer. <b>Abort</b> stops one test. "
    "<b>STOP</b> is Klipper's emergency stop: everything off at once, then the board needs a restart.</p>"
    "<button class=\"stop\" id=\"stop\">STOP EVERYTHING</button><div id=\"sd\" class=\"sd\" hidden></div>"
    "<div id=\"nb\" class=\"sd\" hidden></div><h2>Temperatures</h2><div id=\"tp\"></div>"
    "<h2>Tests</h2><table id=\"tt\"></table><div id=\"sv\" class=\"sd\" hidden>New PID values are in use but not saved. "
    "<button data-save=\"1\">Save PID to printer.cfg</button> (the board restarts)</div><p class=\"dt\">Motor test: CW (the axis' + direction) at the rpm "
    "for the seconds, CCW the same, then a shake (1/8 turn back and forth, 10 times; Shake runs just that). Run all: TMC check, LED, "
    "fans, every motor in turn, with the hotend heating to 75 &deg;C alongside. PID tune: Klipper's PID_CALIBRATE "
    "at the temperature you type (it swings 5 &deg;C around it about 6 times); if a heater overshoots, tune it at the "
    "temperature you print at. StallGuard: move the axis to the middle first; it runs the motor forward and back at the speed "
    "(use your homing_speed), reads the driver's SG_RESULT and the DIAG pin, checks the DIAG wiring and suggests a driver_SGTHRS. "
    "<a href=\"/heater_log.csv\">Heater log (CSV, last 5 minutes)</a>.</p>"
    "<h2>Calibration and stallGuard</h2><p class=\"dt\">rotation_distance: mm the axis moves per motor turn "
    "(steps/mm = 200 x microsteps / rotation_distance). To check it: jog a known distance on the Printer page, measure "
    "with a ruler or calipers, type both here, Work out, then Save (the board restarts). Sensorless homing, per axis: "
    "set the motor current and the sensitivity (driver_SGTHRS, 0-255: higher stops more easily), then "
    "<b>1. Hard stop</b> (the real homing move: it must stop at the end and nowhere else; it backs off afterwards), "
    "<b>2. Free run</b> (moves in the middle: DIAG must stay quiet), <b>3. Save</b> (both into printer.cfg, the board "
    "restarts). The DIAG box goes red while the driver says stalled (and stays red 3 s so short triggers show).</p>"
    "<div id=\"cl\"></div>"
    "<h2>Drivers and fans</h2><div id=\"dv\" class=\"dt\"></div>"
    "<h2>Console</h2><form id=\"cf\"><input id=\"ci\" style=\"width:16em\" placeholder=\"M104 S75\" autocomplete=\"off\">"
    " <button>Send</button></form><pre id=\"out\">Replies appear here. Type HELP for every command.</pre>"
    "<p><a href=\"/status\">Status</a> &middot; <a href=\"/config\">printer.cfg</a> &middot; <a href=\"/console\">Live log</a></p>"
    "</main><script src=\"/ui.js\"></script><script>"
    "var out=document.getElementById('out'),built='',calb='';"
    "function E(s){return String(s).replace(/[&<>\"]/g,function(c){return{'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;'}[c]})}"
    "function V(id,d){var e=document.getElementById(id);return e?e.value:d}"
    "function calBuild(st){var h='';st.forEach(function(s){var n=E(s.name);"
    "h+='<div class=\"cal\"><b>'+n+'</b> rotation_distance <input type=\"number\" step=\"0.001\" id=\"rd_'+n+'\" value=\"'+s.rd+'\"> mm = <span id=\"spm_'+n+'\">'+s.spm+'</span> steps/mm <button data-rds=\"'+n+'\">Save</button><br><span class=\"dt\">check: moved <input type=\"number\" id=\"mc_'+n+'\" value=\"50\"> mm, measured <input type=\"number\" step=\"0.01\" id=\"mm_'+n+'\"> mm <button data-rdc=\"'+n+'\">Work out</button></span>';"
    "var cur=s.cur!==undefined?'motor current <input type=\"number\" step=\"0.05\" min=\"0.1\" max=\"2\" id=\"cur_'+n+'\" value=\"'+s.cur.toFixed(2)+'\"> A':'';"
    "if(s.sgthrs!==undefined)h+='<div class=\"sgb\"><b>Sensorless homing</b> &nbsp;DIAG <span class=\"dg\" id=\"dg_'+n+'\">?</span><br>'+cur+"
    "' &nbsp; sensitivity <input type=\"range\" min=\"0\" max=\"255\" id=\"sgr_'+n+'\" data-n=\"'+n+'\" value=\"'+s.sgthrs+'\"> <input type=\"number\" min=\"0\" max=\"255\" id=\"sgv_'+n+'\" data-n=\"'+n+'\" value=\"'+s.sgthrs+'\">'"
    "+'<div><button class=\"y\" data-hs=\"'+n+'\">1. Hard stop</button><button class=\"y\" data-fr=\"'+n+'\">2. Free run</button><button class=\"y\" data-sv=\"'+n+'\">3. Save</button></div>'"
    "+'<div class=\"dt\" id=\"sgres_'+n+'\">Hard stop: the real homing move for this axis with the values above; it must stop at the end of travel, then backs off. Free run: moves in the middle; DIAG must stay quiet. Save: both values into printer.cfg.</div></div>';"
    "else if(cur)h+='<br>'+cur+' <button data-curset=\"'+n+'\">Set</button><button data-cursave=\"'+n+'\">Save</button>';"
    "h+='</div>'});document.getElementById('cl').innerHTML=h}"
    "function dgShow(e,s){if(!e)return;var on=s.diag==1||s.diag_recent;e.className='dg '+(on?'on':s.diag==0?'off':'');e.textContent=on?'TRIGGERED':s.diag==0?'free':'?'}"
    "document.getElementById('cl').oninput=function(e){var n=e.target.dataset.n;if(!n)return;var o=document.getElementById((e.target.type=='range'?'sgv_':'sgr_')+n);if(o)o.value=e.target.value};"
    "document.getElementById('cl').onclick=function(e){var d=e.target.dataset;if(!d)return;"
    "if(d.curset)sfCmd('SET_TMC_CURRENT STEPPER='+d.curset+' CURRENT='+V('cur_'+d.curset,0),out);"
    "if(d.cursave)sfCmd('SET_TMC_CURRENT STEPPER='+d.cursave+' CURRENT='+V('cur_'+d.cursave,0)+' SAVE=1',out);"
    "if(d.hs)sfCmd('STALLGUARD_HOME STEPPER='+d.hs+' SGTHRS='+V('sgv_'+d.hs,0)+' CURRENT='+V('cur_'+d.hs,0.5),out);"
    "if(d.fr)sfCmd('SET_TMC_CURRENT STEPPER='+d.fr+' CURRENT='+V('cur_'+d.fr,0.5),out).then(function(){return sfCmd('SET_STALLGUARD STEPPER='+d.fr+' SGTHRS='+V('sgv_'+d.fr,0),out)}).then(function(){return sfCmd('STALLGUARD_TEST STEPPER='+d.fr,out)});"
    "if(d.sv)sfCmd('SAVE_SENSORLESS STEPPER='+d.sv+' SGTHRS='+V('sgv_'+d.sv,0)+' CURRENT='+V('cur_'+d.sv,0.5),out);"
    "if(d.rdc){var c=parseFloat(V('mc_'+d.rdc,0)),m=parseFloat(V('mm_'+d.rdc,0)),r=document.getElementById('rd_'+d.rdc);"
    "if(c>0&&m>0){r.value=(parseFloat(r.value)*m/c).toFixed(4);out.textContent='new rotation_distance '+r.value+' (old x measured / moved). Save to use it.'}else out.textContent='type the distance moved and the distance measured'}"
    "if(d.rds)sfCmd('SET_ROTATION_DISTANCE STEPPER='+d.rds+' VALUE='+V('rd_'+d.rds,0),out);"
    "if(d.sgset)sfCmd('SET_STALLGUARD STEPPER='+d.sgset+' SGTHRS='+V('sgv_'+d.sgset,0),out);"
    "if(d.sgsave)sfCmd('SET_STALLGUARD STEPPER='+d.sgsave+' SGTHRS='+V('sgv_'+d.sgsave,0)+' SAVE=1',out)};"
    "function cmdFor(id){if(id=='all')return 'TEST_ALL';if(id=='tmc')return 'TMC_CHECK';if(id=='led')return 'LED_TEST';"
    "if(id.indexOf('heat_')==0)return 'TEST_HEATER HEATER='+id.slice(5)+' TARGET='+V('v_'+id,75);"
    "if(id.indexOf('motor_')==0)return 'STEPPER_TEST STEPPER='+id.slice(6)+' RPM='+V('r_'+id,20)+' SECONDS='+V('s_'+id,20);"
    "if(id.indexOf('fan_')==0)return 'FAN_TEST FAN='+id.slice(4);"
    "if(id.indexOf('out_')==0)return 'HEATER_OUTPUT_TEST HEATER='+id.slice(4);"
    "if(id.indexOf('pid_')==0)return 'PID_CALIBRATE HEATER='+id.slice(4)+' TARGET='+V('v_'+id,170);"
    "if(id.indexOf('stop_')==0)return 'STALLGUARD_HOME STEPPER='+id.slice(5);"
    "if(id.indexOf('sg_')==0){var sp=V('r_'+id,'');return 'STALLGUARD_TEST STEPPER='+id.slice(3)+(sp?' SPEED='+sp:'')+' DIST='+V('s_'+id,10)}return ''}"
    "function build(ts){var h='';ts.forEach(function(t){var i=E(t.id),p='';"
    "if(t.id.indexOf('heat_')==0)p='<input type=\"number\" id=\"v_'+i+'\" value=\"'+(t.id=='heat_heater_bed'?50:75)+'\"> &deg;C ';"
    "if(t.id.indexOf('pid_')==0)p='<input type=\"number\" id=\"v_'+i+'\" value=\"'+(t.id=='pid_heater_bed'?60:170)+'\"> &deg;C ';"
    "if(t.id.indexOf('motor_')==0)p='<input type=\"number\" id=\"r_'+i+'\" value=\"20\"> rpm <input type=\"number\" id=\"s_'+i+'\" value=\"20\"> s "
    "<button data-buzz=\"'+i+'\">Shake</button> ';"
    "if(t.id.indexOf('sg_')==0)p='<input type=\"number\" id=\"r_'+i+'\" placeholder=\"homing\" style=\"width:5em\"> mm/s <input type=\"number\" id=\"s_'+i+'\" value=\"10\"> mm ';"
    "h+='<tr><td><b>'+E(t.label)+'</b> '+(t.id.indexOf('sg_')==0?'<span class=\"dg\" id=\"dgt_'+E(t.id.slice(3))+'\">?</span> ':'')+'<span class=\"st\" id=\"st_'+i+'\"></span><div class=\"dt\" id=\"dt_'+i+'\"></div></td>"
    "<td class=\"ac\">'+p+'<button class=\"run\" data-run=\"'+i+'\">Run</button> "
    "<button class=\"ab\" data-abort=\"'+i+'\" id=\"ab_'+i+'\">Abort</button></td></tr>'});"
    "document.getElementById('tt').innerHTML=h}"
    "document.getElementById('tt').onclick=function(e){var d=e.target.dataset;if(!d)return;"
    "if(d.run)sfCmd(cmdFor(d.run),out);if(d.abort)sfCmd('TEST_ABORT TEST='+d.abort,out);"
    "if(d.buzz)sfCmd('STEPPER_SHAKE STEPPER='+d.buzz.slice(6),out);if(d.save)sfCmd('SAVE_CONFIG',out)};"
    "document.getElementById('sv').onclick=function(e){if(e.target.dataset.save)sfCmd('SAVE_CONFIG',out)};"
    "document.getElementById('stop').onclick=function(){sfCmd('M112',out)};"
    "document.getElementById('sd').onclick=function(e){if(e.target.dataset.restart)sfCmd('FIRMWARE_RESTART',out)};"
    "document.getElementById('cf').onsubmit=function(e){e.preventDefault();var i=document.getElementById('ci');"
    "if(i.value)sfCmd(i.value,out);i.value=''};"
    "function poll(){fetch('/status.json',{cache:'no-store'}).then(function(r){return r.json()}).then(function(j){"
    "var m=j.machine,nb=document.getElementById('nb'),sd=document.getElementById('sd');"
    "if(!m){sfTempsLost(document.getElementById('tp'));nb.hidden=false;nb.textContent='Printer board: '+j.board.state+(j.board.cfg_status?' - '+j.board.cfg_status:'')+"
    "'. The tests appear when the board is connected and printer.cfg has no errors.';return}nb.hidden=true;"
    "sd.hidden=!m.shutdown;if(m.shutdown)sd.innerHTML=(m.power_off_now?'<div style=\"background:#c4161c;color:#fff;"
    "font:700 1.3rem Impact,Arial Black,sans-serif;padding:8px;margin-bottom:6px\">SWITCH OFF THE 12-24 V POWER SUPPLY</div>':'')+"
    "'<b>SHUT DOWN:</b> '+E(m.reason)+(m.timing?'<br><span class=\"dt\">'+E(m.timing)+'</span>':'')+' <button data-restart=\"1\">Restart board</button>';"
    "sfTemps(document.getElementById('tp'),m.sensors,m.shutdown);"
    "var ids=m.tests.map(function(t){return t.id}).join(',');if(ids!=built){build(m.tests);built=ids}"
    "var cn=m.steppers.map(function(s){return s.name+s.rd+s.sgthrs+s.cur}).join(',');if(cn!=calb&&!(document.activeElement&&document.getElementById('cl').contains(document.activeElement))){calBuild(m.steppers);calb=cn}"
    "m.steppers.forEach(function(s){if(s.sgthrs===undefined)return;dgShow(document.getElementById('dg_'+s.name),s);"
    "dgShow(document.getElementById('dgt_'+s.name),s);var sp=document.getElementById('spm_'+s.name);if(sp)sp.textContent=s.spm;"
    "var rs=document.getElementById('sgres_'+s.name),tt='';m.tests.forEach(function(t){if((t.id=='stop_'+s.name||t.id=='sg_'+s.name)&&t.state!='idle')"
    "tt+='<div><b>'+(t.id.indexOf('stop_')==0?'Hard stop':'Free run')+': <span class=\"'+t.state+'\">'+t.state+(t.state=='running'?' '+t.elapsed+' s':'')+'</span></b> '+E(t.detail)+'</div>'});"
    "if(rs&&tt)rs.innerHTML=tt});"
    "m.tests.forEach(function(t){var s=document.getElementById('st_'+t.id),d=document.getElementById('dt_'+t.id),"
    "a=document.getElementById('ab_'+t.id);if(!s)return;s.textContent=t.state=='idle'?'':t.state+(t.state=='running'?' '+t.elapsed+' s':'');"
    "s.className='st '+t.state;d.textContent=t.detail;a.disabled=t.state!='running'&&!(t.id.indexOf('motor_')==0)});"
    "var h='';m.steppers.forEach(function(s){h+='<div><b>'+E(s.name)+'</b>: '+E(s.tmc)+(s.enabled?' (driver on)':'')+'</div>'});"
    "m.fans.forEach(function(f){h+='<div><b>fan '+E(f.name)+'</b>: '+Math.round(f.speed*100)+'%'+(f.auto?' (comes on with the heater)':'')+'</div>'});"
    "h+='<div>Board clock estimate error: '+m.clock_err_ms+' ms</div>';document.getElementById('sv').hidden=!m.unsaved_pid;"
    "document.getElementById('dv').innerHTML=h}).catch(function(){sfTempsLost(document.getElementById('tp'))}).then(function(){setTimeout(poll,1000)})}poll();"
    "</script></body></html>";

// The heaters' last 5 minutes, now and before the last board restart, as CSV (open in a spreadsheet).
static esp_err_t heater_log_get(httpd_req_t *req)
{
    static char buf[16384];   // static: small HTTP task stack
    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"heater_log.csv\"");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    if (sf_console("HEATER_LOG", buf, sizeof buf, 3000) && strncmp(buf, "time_s", 6) == 0) {
        chunk(req, "# now\n");
        chunk(req, buf);
    } else {
        chunk(req, "# now: no heater data (board not connected or not configured)\n");
    }
    if (sf_prev_heater_log(buf, sizeof buf)) {
        chunk(req, "# before the last board restart or shutdown\n");
        chunk(req, buf);
    }
    return httpd_resp_sendstr_chunk(req, NULL);
}

// The printer page: homing, jogging, temperatures, upload a G-code file and print it.
static const char PRINTER_HTML[] =
    "<!doctype html><html><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>Skipper Flipper printer</title><style>"
    "body{background:#e9e6dc;color:#141414;font:16px/1.5 Georgia,serif;margin:0}"
    ".b{background:#141414;color:#e9e6dc;text-align:center;padding:10px;font-weight:700;letter-spacing:.06em}"
    "main{max-width:760px;margin:0 auto;padding:16px}h1{font:700 2rem Impact,Arial Black,sans-serif;"
    "border-bottom:4px solid #141414;margin-bottom:8px}h2{margin:18px 0 6px}"
    "button,input,select{font:inherit;border:2px solid #141414;background:#f7f5ee;padding:4px 10px;margin:2px}"
    "button.k{background:#141414;color:#e9e6dc}button.ab{background:#c4161c;color:#fff;border-color:#c4161c}"
    "input[type=number]{width:4.5em}.stop{display:block;width:100%;font:700 1.4rem Impact,Arial Black,sans-serif;"
    "background:#c4161c;color:#fff;border:4px solid #141414;padding:12px;margin:10px 0}"
    ".sd{background:#ffd400;border:3px solid #141414;padding:8px;margin:8px 0}"
    ".pos{font:700 1.15em ui-monospace,Menlo,Consolas,monospace;background:#f7f5ee;border:2px solid #141414;padding:6px 8px}"
    ".jg{display:grid;grid-template-columns:repeat(3,4.2em);gap:4px;margin:6px 0}.jg button{margin:0;padding:8px 0}"
    ".ft{border-collapse:collapse;margin:6px 0;width:100%}.ft td{border-bottom:1px solid #141414;padding:4px}.ft td.r{text-align:right;white-space:nowrap}"
    ".job{margin:6px 0}.job>div{display:inline-block;vertical-align:top;margin:0 8px 6px 0}"
    ".jbar{font:700 1.15em ui-monospace,Menlo,Consolas,monospace;background:#f7f5ee;border:2px solid #141414;padding:6px 8px;white-space:pre}"
    ".zb{font:700 1.15em ui-monospace,Menlo,Consolas,monospace;background:#141414;color:#e9e6dc;padding:6px 10px;min-width:7.5em;text-align:center}"
    ".zb small{display:block;font:400 .7em Georgia,serif}"
    ".th{width:96px;height:96px;object-fit:contain;background:#f7f5ee;border:2px solid #141414}"
    ".ts{width:40px;height:40px;object-fit:contain;vertical-align:middle}.hi td{font-size:.85em}"
    "#out{background:#141414;color:#e9e6dc;padding:8px;min-height:3em;white-space:pre-wrap}.dt{font-size:.85em}"
    "svg.i{width:1.2em;height:1.2em;vertical-align:-.25em;margin-right:4px;fill:currentColor}"
    ".go{border:5px solid #141414;background:#fff8c8;padding:10px 12px;margin:10px 0;box-shadow:6px 6px 0 #141414}"
    ".go h2{margin:0 0 6px;font:700 1.5rem Impact,Arial Black,sans-serif}.go select{width:100%;padding:8px;font-size:1.05em}"
    ".gob{display:block;width:100%;font:700 1.6rem Impact,Arial Black,sans-serif;background:#1e7b34;color:#fff;"
    "border:4px solid #141414;padding:14px;margin:8px 0 4px;letter-spacing:.05em}.gob:disabled{background:#8a8a8a}"
    ".bb{font-weight:700;padding:8px 12px;border-width:3px}"
    ".jg{grid-template-columns:repeat(4,4.6em)}.jg button{font:700 1.2em Impact,Arial Black,sans-serif;"
    "border:4px solid #141414;background:linear-gradient(#fff 50%,#ddd8c8 50%);padding:12px 0}"
    ".jg .x{border-color:#c4161c;color:#c4161c}.jg .y{border-color:#1a5fb4;color:#1a5fb4}"
    ".jg .z{background:linear-gradient(#3a3a3a 50%,#141414 50%);color:#ffd400}.jg .h{background:#ffd400}"
    "</style></head><body>" SF_NAV
    "<svg width=\"0\" height=\"0\" style=\"position:absolute\">"
    "<symbol id=\"ih\" viewBox=\"0 0 24 24\"><path d=\"M2 12 12 3l10 9h-3v9h-5v-6h-4v6H5v-9z\"/></symbol>"
    "<symbol id=\"it\" viewBox=\"0 0 24 24\"><path d=\"M9.5 4.5a2.5 2.5 0 0 1 5 0v8.6a4.6 4.6 0 1 1-5 0z\" fill=\"#fff\" "
    "stroke=\"#141414\" stroke-width=\"1.6\"/><circle cx=\"12\" cy=\"17\" r=\"2.8\" fill=\"#c4161c\"/>"
    "<rect x=\"11\" y=\"7\" width=\"2\" height=\"9\" fill=\"#c4161c\"/></symbol>"
    "<symbol id=\"ic\" viewBox=\"0 0 24 24\"><path d=\"M12 2v20M3.5 7l17 10M3.5 17l17-10\" stroke=\"#1a5fb4\" "
    "stroke-width=\"2.6\" stroke-linecap=\"round\"/></symbol>"
    "<symbol id=\"if\" viewBox=\"0 0 24 24\"><circle cx=\"12\" cy=\"12\" r=\"2.2\"/><path d=\"M12 9.5C11 3 18 2 17.5 6.5 "
    "17 9 14 10 12 9.5zM14.5 12c6.5-1 7.5 6 3 5.5-2.5-.5-3.5-3.5-3-5.5zM9.7 13.3c-4 5-9.5.7-5.8-2 2.1-1.4 4.6.2 5.8 2z\"/></symbol>"
    "<symbol id=\"ip\" viewBox=\"0 0 24 24\"><path d=\"M6 3v18l15-9z\"/></symbol>"
    "<symbol id=\"iz\" viewBox=\"0 0 24 24\"><path d=\"M5 3h5v18H5zM14 3h5v18h-5z\"/></symbol>"
    "<symbol id=\"ix\" viewBox=\"0 0 24 24\"><path d=\"M5 5l14 14M19 5 5 19\" stroke=\"currentColor\" stroke-width=\"4\"/></symbol>"
    "</svg><div class=\"b\">" SF_BANNER "</div><main><h1>Printer</h1>"
    "<button class=\"stop\" id=\"stop\">STOP EVERYTHING</button><div id=\"sd\" class=\"sd\" hidden></div>"
    "<div id=\"nb\" class=\"sd\" hidden></div>"
    "<div class=\"go\"><h2>Print</h2><div id=\"gst\"><select id=\"pfs\"><option value=\"\">(no G-code files yet)</option></select>"
    "<button class=\"gob\" id=\"gob\" disabled><svg class=\"i\"><use href=\"#ip\"/></svg>START PRINT</button>"
    "<div class=\"dt\">or a new file: <input type=\"file\" id=\"fi\" accept=\".gcode,.gco,.g,.txt\"> "
    "<button id=\"upb\">Upload</button> <label><input type=\"checkbox\" id=\"pa\" checked> print it when uploaded</label> "
    "<span id=\"ups\"></span></div></div>"
    "<div class=\"job\"><div class=\"jbar\" id=\"tbar\">[....................]   0%\nidle</div>"
    "<div class=\"zb\" id=\"zb\">Z -<small>layer -</small></div><img id=\"pth\" class=\"th\" alt=\"\" hidden></div>"
    "<div id=\"ps\" class=\"dt\">idle</div>"
    "<div><button class=\"bb\" data-g=\"PAUSE\"><svg class=\"i\"><use href=\"#iz\"/></svg>Pause</button>"
    "<button class=\"bb\" data-g=\"RESUME\"><svg class=\"i\"><use href=\"#ip\"/></svg>Resume</button>"
    "<button class=\"ab bb\" data-g=\"CANCEL_PRINT\"><svg class=\"i\"><use href=\"#ix\"/></svg>Cancel</button></div></div>"
    "<h2>Temperatures</h2><div id=\"tp\"></div><div><button class=\"bb\" data-c=\"ph\"><svg class=\"i\"><use href=\"#it\"/></svg>Preheat</button>"
    "<button class=\"bb\" data-c=\"cd\"><svg class=\"i\"><use href=\"#ic\"/></svg>Cool down</button></div>"
    "<div>Hotend <input type=\"number\" id=\"te\" value=\"200\"> &deg;C "
    "<button data-c=\"he\"><svg class=\"i\"><use href=\"#it\"/></svg>Heat</button> <button data-g=\"M104 S0\">Off</button></div>"
    "<div id=\"bedrow\" hidden>Bed <input type=\"number\" id=\"tb\" value=\"60\"> &deg;C <button data-c=\"hb\">"
    "<svg class=\"i\"><use href=\"#it\"/></svg>Heat</button> <button data-g=\"M140 S0\">Off</button></div>"
    "<div><svg class=\"i\"><use href=\"#if\"/></svg>Fan "
    "<button data-g=\"M106 S255\">100%</button><button data-g=\"M106 S128\">50%</button><button data-g=\"M107\">off</button></div>"
    "<h2>Position</h2><div class=\"pos\" id=\"pos\">-</div><div class=\"dt\" id=\"hm\"></div>"
    "<div><button class=\"k bb\" data-g=\"G28\"><svg class=\"i\"><use href=\"#ih\"/></svg>Home all</button>"
    "<button data-g=\"G28 X\"><svg class=\"i\"><use href=\"#ih\"/></svg>X</button>"
    "<button data-g=\"G28 Y\"><svg class=\"i\"><use href=\"#ih\"/></svg>Y</button>"
    "<button data-g=\"G28 Z\"><svg class=\"i\"><use href=\"#ih\"/></svg>Z</button><button data-g=\"PROBE\">Probe</button>"
    "<button data-g=\"M84\">Motors off</button><button data-g=\"QUERY_ENDSTOPS\">Endstops?</button></div>"
    "<p>Step <select id=\"st\"><option>0.1</option><option selected>1</option><option>10</option><option>50</option></select> mm</p>"
    "<div class=\"jg\"><span></span><button class=\"y\" data-j=\"Y 1\">Y+</button><span></span><button class=\"z\" data-j=\"Z 1\">Z+</button>"
    "<button class=\"x\" data-j=\"X -1\">X-</button><button class=\"h\" data-g=\"G28\" title=\"Home all\"><svg class=\"i\"><use href=\"#ih\"/></svg></button>"
    "<button class=\"x\" data-j=\"X 1\">X+</button><span></span>"
    "<span></span><button class=\"y\" data-j=\"Y -1\">Y-</button><span></span><button class=\"z\" data-j=\"Z -1\">Z-</button></div>"
    "<div>Filament: <button data-e=\"5\">Extrude 5 mm</button><button data-e=\"-5\">Retract 5 mm</button> "
    "<span class=\"dt\">(hotend at least 170 &deg;C)</span></div>"
    "<h2>Files</h2>"
    "<div id=\"sdb\" class=\"dt\">SD card: checking...</div>"
    "<table id=\"sdt\" class=\"ft\"></table>"
    "<div><button data-sdm=\"1\">Remount SD card</button></div>"
    "<p class=\"dt\"><b>Send straight from the slicer</b> (PrusaSlicer, OrcaSlicer): add a physical printer of "
    "type <b>OctoPrint</b>, host <b>http://192.168.4.1</b> (or this ESP32's home WiFi address, see Settings), any "
    "API key. Files go to the SD card; \"Upload and print\" starts them.</p>"
    "<h2>Recent prints</h2><table id=\"hi\" class=\"ft hi\"><tr><td>-</td></tr></table>"
    "<p class=\"dt\"><a href=\"/sdfile?f=skipper/events.log&amp;tail=65536\">Event log</a> (SD card: board connects, "
    "shutdowns, prints, restarts) &middot; <a href=\"/sdfile?f=skipper/history.csv\">history.csv</a></p>"
    "<p class=\"dt\">With an SD card in, uploads go to the card as plain G-code (a card written on a PC works too: files in its top folder). Without one, the file is compressed in the browser (about 5 times smaller) and kept in the ESP32-S3's own flash (508 KB; 1.75 MB on SKIPPER). Plain G-code only: slicer macros (START_PRINT ...) are skipped and listed. No moves or homing while printing.</p>"
    "<h2>Console</h2><form id=\"cf\"><input id=\"ci\" style=\"width:16em\" placeholder=\"G1 X10 F3000\" autocomplete=\"off\">"
    " <button>Send</button></form><pre id=\"out\">Replies appear here.</pre>"
    "<p><a href=\"/status\">Status</a> &middot; <a href=\"/test\">Test bench</a> &middot; <a href=\"/config\">printer.cfg</a></p>"
    "</main><script src=\"/ui.js\"></script><script>"
    "var out=document.getElementById('out'),cap=0;function $(i){return document.getElementById(i)}"
    "function E(s){return String(s).replace(/[&<>\"]/g,function(c){return{'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;'}[c]})}"
    "document.body.onclick=function(e){var b=e.target.closest&&e.target.closest('button'),d=b?b.dataset:e.target.dataset;if(!d)return;"
    "if(d.c=='ph')sfCmd('M104 S'+$('te').value,out).then(function(){if(!$('bedrow').hidden)sfCmd('M140 S'+$('tb').value,out)});"
    "if(d.c=='cd')sfCmd('M104 S0',out).then(function(){return sfCmd('M140 S0',out)}).then(function(){sfCmd('M107',out)});"
    "if(d.g)sfCmd(d.g,out);if(d.c=='he')sfCmd('M104 S'+$('te').value,out);if(d.c=='hb')sfCmd('M140 S'+$('tb').value,out);"
    "if(d.j){var p=d.j.split(' ');sfCmd('JOG AXIS='+p[0]+' DIST='+(parseFloat(p[1])*parseFloat($('st').value)),out)}"
    "if(d.e)sfCmd('JOG AXIS=E DIST='+d.e,out);if(d.restart)sfCmd('FIRMWARE_RESTART',out)};"
    "$('stop').onclick=function(){sfCmd('M112',out)};"
    "$('cf').onsubmit=function(e){e.preventDefault();var i=$('ci');if(i.value)sfCmd(i.value,out);i.value=''};"
    "var sdOn=false,sdFiles=[],flashF=null;"
    "function hm(s){s=Math.round(s/60);return s<1?'<1 min':s<60?s+' min':Math.floor(s/60)+' h '+('0'+s%60).slice(-2)+' min'}"
    "function fsz(n){return n>1073741824?(n/1073741824).toFixed(1)+' GB':n>1048576?(n/1048576).toFixed(1)+' MB':Math.max(1,Math.round(n/1024))+' KB'}"
    "function loadFiles(){fetch('/files.json',{cache:'no-store'}).then(function(r){return r.json()}).then(function(j){sdOn=j.mounted;"
    "$('sdb').textContent=j.mounted?'SD card: '+j.files.length+' G-code file'+(j.files.length==1?'':'s')+', '+fsz(j.free)+' free of '+fsz(j.total):'SD card: '+j.status;"
    "sdFiles=j.mounted?j.files:[];showFiles();$('upb').textContent=j.mounted?'Upload to SD card':'Upload to flash'}).catch(function(){$('sdb').textContent='SD card: no answer'})}"
    "function showFiles(){var h='';sdFiles.slice().sort(function(a,b){return a.name<b.name?-1:1}).forEach(function(f){h+='<tr><td><img class=\"ts\" loading=\"lazy\" alt=\"\" src=\"/thumb?name='+encodeURIComponent(f.name)+'\" onerror=\"this.remove()\"> '+E(f.name)+'</td><td>card</td><td class=\"r\">'+fsz(f.size)+'</td><td class=\"r\"><button data-pf=\"'+E(f.name)+'\">Print</button><button class=\"ab\" data-df=\"'+E(f.name)+'\">Delete</button></td></tr>'});"
    "if(flashF)h+='<tr><td>'+E(flashF.name)+'</td><td>flash</td><td class=\"r\">'+fsz(flashF.orig)+'</td><td class=\"r\"><button data-pfl=\"1\">Print</button></td></tr>';"
    "$('sdt').innerHTML=h||'<tr><td>No G-code files stored yet.</td></tr>';"
    "var o='',ps=$('pfs'),cur=ps.value;sdFiles.slice().sort(function(a,b){return a.name<b.name?-1:1}).forEach(function(f){o+='<option value=\"c:'+E(f.name)+'\">'+E(f.name)+' ('+fsz(f.size)+')</option>'});"
    "if(flashF)o+='<option value=\"f:\">'+E(flashF.name)+' (in flash)</option>';ps.innerHTML=o||'<option value=\"\">(no G-code files yet)</option>';if(cur)ps.value=cur;if(!ps.value&&ps.options.length)ps.selectedIndex=0;$('gob').disabled=!ps.value}"
    "function startF(v){if(!v)return;if(v=='f:')sfCmd('SDCARD_PRINT_FILE',out);else sfCmd('M23 '+v.slice(2),out).then(function(){return sfCmd('M24',out)})}"
    "$('gob').onclick=function(){startF($('pfs').value)};"
    "$('sdt').onclick=function(e){var d=e.target.dataset;if(d.pf){sfCmd('M23 '+d.pf,out).then(function(){return sfCmd('M24',out)})}if(d.pfl)sfCmd('SDCARD_PRINT_FILE',out);"
    "if(d.df){fetch('/sd?do=delete&name='+encodeURIComponent(d.df),{method:'POST'}).then(function(r){return r.text()}).then(function(t){out.textContent=t;loadFiles()})}};"
    "document.querySelector('[data-sdm]').onclick=function(e){e.stopPropagation();fetch('/sd?do=mount',{method:'POST'}).then(function(r){return r.text()}).then(function(t){out.textContent=t;loadFiles()})};"
    "$('upb').onclick=async function(){var f=$('fi').files[0],s=$('ups');if(!f){s.textContent='choose a file first';return}"
    "try{if(sdOn){s.textContent='uploading '+fsz(f.size)+' to the SD card...';"
    "var r=await fetch('/upload?dest=sd&name='+encodeURIComponent(f.name)+'&orig='+f.size+'&z=0',{method:'POST',body:f});s.textContent=await r.text();loadFiles();if(r.ok&&$('pa').checked)startF('c:'+f.name);return}"
    "var body=f,z=0;if(window.CompressionStream){s.textContent='compressing...';"
    "body=await new Response(f.stream().pipeThrough(new CompressionStream('deflate'))).blob();z=1}"
    "if(cap&&body.size>cap){s.textContent='too big: '+Math.round(body.size/1024)+' KB'+(z?' compressed':'')+', the space is '+Math.round(cap/1024)+' KB. Use an SD card for big files.';return}"
    "s.textContent='uploading '+Math.round(body.size/1024)+' KB...';"
    "var r=await fetch('/upload?name='+encodeURIComponent(f.name)+'&orig='+f.size+'&z='+z,{method:'POST',body:body});"
    "s.textContent=await r.text();if(r.ok&&$('pa').checked)startF('f:')}catch(x){s.textContent='upload failed: '+x}};"
    "function csv(l){var o=[],c='',q=false;for(var i=0;i<l.length;i++){var ch=l[i];if(q){if(ch=='\"'){if(l[i+1]=='\"'){c+='\"';i++}else q=false}else c+=ch}"
    "else if(ch=='\"')q=true;else if(ch==','){o.push(c);c=''}else c+=ch}o.push(c);return o}"
    "function loadHist(){fetch('/history.json',{cache:'no-store'}).then(function(r){return r.json()}).then(function(j){"
    "var h='';j.prints.forEach(function(l){var c=csv(l);h+='<tr><td>'+E(c[0])+'</td><td>'+E(c[1])+'</td><td>'+E(c[2])+'</td><td class=\"r\">'+hm(+c[3])+'</td></tr>'});"
    "$('hi').innerHTML=h||'<tr><td>'+(j.mounted?'No prints yet.':'Needs the SD card.')+'</td></tr>'}).catch(function(){})}"
    "loadFiles();loadHist();setInterval(loadFiles,15000);var lastSt='';"
    "function poll(){fetch('/status.json',{cache:'no-store'}).then(function(r){return r.json()}).then(function(j){"
    "var m=j.machine,nb=$('nb'),sd=$('sd');cap=j.file?j.file.cap:0;"
    "var nf=j.file&&j.file.name?j.file:null;if((nf&&nf.name)!==(flashF&&flashF.name)){flashF=nf;showFiles()}"
    "if(!m){sfTempsLost($('tp'));nb.hidden=false;nb.textContent='Printer board: '+j.board.state+(j.board.cfg_status?' - '+j.board.cfg_status:'');return}"
    "nb.hidden=true;sd.hidden=!m.shutdown;if(m.shutdown)sd.innerHTML=(m.power_off_now?'<b style=\"color:#c4161c\">SWITCH OFF THE 12-24 V POWER SUPPLY.</b> ':'')+"
    "'<b>SHUT DOWN:</b> '+E(m.reason)+(m.timing?'<br><span class=\"dt\">'+E(m.timing)+'</span>':'')+' <button data-restart=\"1\">Restart board</button>';"
    "sfTemps($('tp'),m.sensors,m.shutdown);$('bedrow').hidden=!m.sensors.some(function(s){return s.name=='heater_bed'});"
    "var t=m.toolhead;if(t&&t.ok){$('pos').textContent='X '+t.pos[0].toFixed(2)+'  Y '+t.pos[1].toFixed(2)+'  Z '+t.pos[2].toFixed(2)+'  E '+t.pos[3].toFixed(1)"
    "+'   homed: '+(t.homed||'none')+(t.busy?'  (moving)':'');$('hm').textContent=t.msg}"
    "else if(t)$('pos').textContent='Moves are not set up: '+t.why;"
    "var p=m.print,on=p.state=='printing'||p.state=='paused',pc=p.state=='done'?1:p.progress,n=Math.floor(pc*20+1e-9);"
    "if(p.state!=lastSt){if(lastSt)loadHist();lastSt=p.state;var pt=$('pth');if(on&&p.file&&sdFiles.some(function(f){return f.name==p.file})){pt.onerror=function(){pt.hidden=true};pt.src='/thumb?name='+encodeURIComponent(p.file);pt.hidden=false}else pt.hidden=true}"
    "$('gst').hidden=on;$('tbar').textContent='['+'#'.repeat(n)+'.'.repeat(20-n)+'] '+('  '+Math.floor(pc*100)).slice(-3)+'%\\n'+(on?(p.eta>=0?hm(p.eta)+' left':'time left: -'):p.state);"
    "var z=p.layer_z>=0?p.layer_z:(t&&t.ok?t.pos[2]:null);"
    "$('zb').innerHTML='Z '+(z==null?'-':z.toFixed(2))+'<small>'+(p.layer?'layer '+p.layer+(p.layers?' / '+p.layers:''):'layer -')+(p.layer_h>0?' &middot; '+p.layer_h.toFixed(2)+' mm':'')+'</small>';"
    "$('ps').textContent=p.state+(p.file?' '+p.file:'')+(on?', '+hm(p.elapsed)+' so far'+(p.slicer_time>0?' (slicer said '+hm(p.slicer_time)+')':''):'')+(p.msg?' - '+p.msg:'')"
    "}).catch(function(){sfTempsLost($('tp'))}).then(function(){setTimeout(poll,1000)})}poll();"
    "</script></body></html>";

static esp_err_t printer_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=30");   // fixed pages: switching between them is instant
    return httpd_resp_send(req, PRINTER_HTML, HTTPD_RESP_USE_STRLEN);
}

// Upload to the SD card: plain text (the page does not compress for the card), written to name.part
// first and renamed when complete, so a broken upload never looks like a finished file.
static esp_err_t upload_to_sd(httpd_req_t *req, char *name, size_t len, bool z)
{
    static char buf[4096];
    if (!sf_sd_mounted()) {
        snprintf(buf, sizeof buf, "no SD card: %s", sf_sd_error());
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, buf);
    }
    if (z) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "send plain G-code to the SD card (z=0)");
    }
    for (char *c = name; *c; c++) {   // FILENAME= takes no spaces
        if (*c == ' ') {
            *c = '_';
        }
    }
    if (!sf_sd_name_ok(name)) {
        size_t n = strlen(name);
        if (n + 6 < 64) {
            strcat(name, ".gcode");
        }
    }
    char path[300], part[310];
    uint64_t total = 0, freeb = 0;
    sf_sd_space(&total, &freeb);
    if (len == 0 || !sf_sd_path(name, path, sizeof path)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty file or unusable name (use name.gcode)");
    }
    if ((uint64_t)len + 65536 > freeb) {
        snprintf(buf, sizeof buf, "not enough room on the SD card: %u KB needed, %llu KB free", (unsigned)(len / 1024),
                 (unsigned long long)(freeb / 1024));
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, buf);
    }
    snprintf(part, sizeof part, "%s.part", path);
    FILE *f = fopen(part, "wb");
    if (!f) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "could not create the file on the SD card");
    }
    size_t got = 0;
    int timeouts = 0;
    while (got < len) {
        int n = httpd_req_recv(req, buf, len - got < sizeof buf ? len - got : sizeof buf);
        if (n == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts < 5) {
            continue;
        }
        if (n <= 0 || fwrite(buf, 1, (size_t)n, f) != (size_t)n) {
            fclose(f);
            unlink(part);
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "upload interrupted: the file is not stored");
        }
        got += (size_t)n;
    }
    if (fclose(f) != 0) {
        unlink(part);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "could not finish writing to the SD card");
    }
    unlink(path);   // FAT cannot rename over an existing file
    if (rename(part, path) != 0) {
        unlink(part);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "could not rename the finished file");
    }
    snprintf(buf, sizeof buf, "ok stored %s on the SD card (%u KB). Press Print next to it.", name, (unsigned)(len / 1024));
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, buf);
}

static void json_str_append(char *out, size_t cap, size_t *pos, const char *str)
{
    if (*pos + 2 >= cap) {
        return;
    }
    out[(*pos)++] = '"';
    for (const unsigned char *c = (const unsigned char *)str; *c && *pos + 8 < cap; c++) {
        if (*c == '"' || *c == '\\') {
            out[(*pos)++] = '\\';
            out[(*pos)++] = (char)*c;
        } else if (*c < 0x20) {
            *pos += (size_t)snprintf(out + *pos, cap - *pos, "\\u%04x", *c);
        } else {
            out[(*pos)++] = (char)*c;
        }
    }
    out[(*pos)++] = '"';
    out[*pos] = 0;
}

typedef struct { char *out; size_t cap, pos; int n; } files_json_t;

static void files_json_add(const char *name, uint64_t size, void *arg)
{
    files_json_t *j = (files_json_t *)arg;
    if (j->pos + 300 > j->cap) {   // grow
        size_t nc = j->cap * 2;
        char *nb = realloc(j->out, nc);
        if (!nb) {
            return;
        }
        j->out = nb;
        j->cap = nc;
    }
    j->pos += (size_t)snprintf(j->out + j->pos, j->cap - j->pos, "%s{\"name\":", j->n ? "," : "");
    json_str_append(j->out, j->cap, &j->pos, name);
    j->pos += (size_t)snprintf(j->out + j->pos, j->cap - j->pos, ",\"size\":%llu}", (unsigned long long)size);
    j->n++;
}

// GET /files.json: the SD card's G-code files and its space.
static esp_err_t files_get(httpd_req_t *req)
{
    files_json_t j = {malloc(4096), 4096, 0, 0};
    if (!j.out) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
    }
    uint64_t total = 0, freeb = 0;
    bool m = sf_sd_mounted() && sf_sd_space(&total, &freeb);
    j.pos = (size_t)snprintf(j.out, j.cap, "{\"mounted\":%s,\"total\":%llu,\"free\":%llu,\"printing\":%s,\"status\":",
                             m ? "true" : "false", (unsigned long long)total, (unsigned long long)freeb,
                             sf_printing() ? "true" : "false");
    json_str_append(j.out, j.cap, &j.pos, sf_sd_error());
    j.pos += (size_t)snprintf(j.out + j.pos, j.cap - j.pos, ",\"files\":[");
    if (m) {
        sf_sd_list(files_json_add, &j);
    }
    if (j.pos + 4 > j.cap) {
        j.pos = j.cap - 4;
    }
    j.pos += (size_t)snprintf(j.out + j.pos, j.cap - j.pos, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t r = httpd_resp_send(req, j.out, (ssize_t)j.pos);
    free(j.out);
    return r;
}

// POST /sd?do=mount|delete&name=  (remount after swapping cards; delete a file)
static esp_err_t sd_post(httpd_req_t *req)
{
    char q[300], act[16] = "", name[256] = "";
    if (httpd_req_get_url_query_str(req, q, sizeof q) == ESP_OK) {
        httpd_query_key_value(q, "do", act, sizeof act);
        if (httpd_query_key_value(q, "name", name, sizeof name) == ESP_OK) {
            url_decode(name);
        }
    }
    if (sf_printing()) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "a print is running: cancel it first");
    }
    httpd_resp_set_type(req, "text/plain");
    if (!strcmp(act, "mount")) {
        bool ok = sf_sd_mount(NULL);
        static char r[160];
        snprintf(r, sizeof r, ok ? "ok SD card mounted" : "error: %s", sf_sd_error());
        return httpd_resp_sendstr(req, r);
    }
    if (!strcmp(act, "delete")) {
        if (!sf_sd_delete(name)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "could not delete that file");
        }
        return httpd_resp_sendstr(req, "ok deleted");
    }
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "use /sd?do=mount or /sd?do=delete&name=");
}

// POST /upload?name=&orig=&z=1 with the (compressed) file as the body: stored in the gcode partition.
static esp_err_t upload_post(httpd_req_t *req)
{
    static char buf[4096];
    char q[200], v[100], name[64] = "upload.gcode";
    uint32_t orig = 0;
    bool z = false, to_sd = false;
    if (sf_printing()) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "a print is running: cancel it before uploading");
    }
    if (httpd_req_get_url_query_str(req, q, sizeof q) == ESP_OK) {
        if (httpd_query_key_value(q, "name", v, sizeof v) == ESP_OK && url_decode(v) && v[0]) {
            size_t k = 0;
            for (char *c = v; *c && k < sizeof name - 1; c++) {   // a plain name: it goes into HTML and JSON
                name[k++] = (isalnum((unsigned char)*c) || strchr("._- ", *c)) ? *c : '_';
            }
            name[k] = 0;
        }
        if (httpd_query_key_value(q, "orig", v, sizeof v) == ESP_OK) {
            orig = (uint32_t)strtoul(v, NULL, 10);
        }
        if (httpd_query_key_value(q, "z", v, sizeof v) == ESP_OK) {
            z = v[0] == '1';
        }
        if (httpd_query_key_value(q, "dest", v, sizeof v) == ESP_OK) {
            to_sd = !strcmp(v, "sd");
        }
    }
    size_t len = req->content_len;
    if (to_sd) {
        return upload_to_sd(req, name, len, z);
    }
    if (len == 0 || len > sf_store_capacity()) {
        snprintf(buf, sizeof buf, "too big: %u KB, the space is %u KB", (unsigned)(len / 1024),
                 (unsigned)(sf_store_capacity() / 1024));
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, buf);
    }
    if (!sf_store_begin(name, (uint32_t)len, orig ? orig : (uint32_t)len, z)) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "could not prepare the flash");
    }
    size_t got = 0;
    int timeouts = 0;
    while (got < len) {
        int n = httpd_req_recv(req, buf, len - got < sizeof buf ? len - got : sizeof buf);
        if (n == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts < 5) {
            continue;
        }
        if (n <= 0 || !sf_store_write(buf, (size_t)n)) {
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "upload interrupted: the file is not stored");
        }
        got += (size_t)n;
    }
    if (!sf_store_end()) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "could not finish storing the file");
    }
    snprintf(buf, sizeof buf, "ok stored %s (%u KB, %u KB in flash). Press Print.", name, (unsigned)((orig ? orig : len) / 1024),
             (unsigned)(len / 1024));
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, buf);
}

static esp_err_t test_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=30");   // fixed pages: switching between them is instant
    return httpd_resp_send(req, TEST_HTML, HTTPD_RESP_USE_STRLEN);
}

static const char CONSOLE_HTML[] =
    "<!doctype html><html><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>Skipper Flipper console</title><style>"
    "body{background:#141414;color:#e9e6dc;font:14px/1.4 ui-monospace,Menlo,Consolas,monospace;margin:0}"
    ".b{background:#e9e6dc;color:#141414;text-align:center;padding:8px;font:700 14px Georgia,serif;"
    "letter-spacing:.06em}.t{padding:8px;position:sticky;top:0;background:#141414;border-bottom:2px solid #e9e6dc}"
    "button{background:#e9e6dc;color:#141414;border:2px solid #e9e6dc;font:inherit;padding:2px 10px;margin-right:6px}"
    "pre{margin:0;padding:8px;white-space:pre-wrap;word-break:break-all}#s{float:right}.t .on{background:#ffd400;border-color:#ffd400}"
    "#r{color:#ffd400;padding:8px}.h{color:#e9e6dc;border-bottom:1px solid #555;padding-bottom:6px}"
    ".e{display:grid;grid-template-columns:6em 6.5em 1fr;gap:2px 14px;padding:7px 0;border-top:1px solid #333}.e.k{border-top:0;padding-top:0}.u{white-space:nowrap}"
    ".w small{display:block}.w small,.u{color:#9a978e}.c,.g{padding-left:1.2em;color:#e9e6dc}"
    "#r .g{display:grid;grid-template-columns:repeat(auto-fill,minmax(19em,1fr));gap:0 14px}"
    ".bad .c1{color:#ff6b5b}.ok .c1{color:#86d986}.bm .c1{color:#ffd400}"
    "@media(max-width:640px){.e{grid-template-columns:auto 1fr}.m{grid-column:1/-1}}</style></head><body>" SF_NAV
    "<div class=\"b\">" SF_BANNER "</div>"
    "<div class=\"t\"><button id=\"tl\" class=\"on\">Scrolling log</button><button id=\"tr\">Important events</button> "
    "<span id=\"lc\"><button id=\"p\">Pause</button><button id=\"c\">Clear</button></span>"
    "<span id=\"rc\" hidden><button id=\"cp\">Copy</button><a href=\"/report.txt\" download=\"skipper-report.txt\" "
    "style=\"color:#e9e6dc\">download</a></span><span id=\"s\">connecting...</span></div>"
    "<pre id=\"o\"></pre><div id=\"r\" hidden>loading...</div><script>"
    "var R=document.getElementById('r'),O=document.getElementById('o'),rt=0;"
    "function tab(rep){R.hidden=!rep;O.hidden=rep;document.getElementById('rc').hidden=!rep;document.getElementById('lc').hidden=rep;"
    "document.getElementById('tr').className=rep?'on':'';document.getElementById('tl').className=rep?'':'on';"
    "clearInterval(rt);if(rep){lr();rt=setInterval(lr,5000)}}"
    "var RT='';function lr(){fetch('/report.txt',{cache:'no-store'}).then(function(r){return r.text()}).then(function(t){if(t!=RT){RT=t;show(t)}})}"
    // Important events laid out in columns: time (date under it), time since start-up, then the message
    // with its first clause as a heading and each further clause on its own line; lists of figures
    // (the benchmark's stages and counts) as a grid. Copy and download keep the plain text.
    "function el(t,c,x){var e=document.createElement(t);if(c)e.className=c;if(x!=null)e.textContent=x;return e}"
    "function kind(s){return /SHUTDOWN|crash|lost|refused|not sent|error/i.test(s)?'bad':/^(motion )?benchmark/.test(s)?'bm':/connected|started/.test(s)?'ok':''}"
    "function show(t){R.textContent='';var h=el('div','h');R.appendChild(h);var hd=el('div','e k');['time','since start','event'].forEach(function(x){hd.appendChild(el('small','u',x))});R.appendChild(hd);var pv='';t.split('\\n').forEach(function(l){if(!l)return;"
    "var m=l.match(/^\\[([^|]*)\\|\\s*up ([^\\]]*)\\]\\s*(.*)$/);if(!m){h.appendChild(el('div','',l));return}"
    "var w=m[1].trim(),i=w.indexOf(' '),same=l.slice(0,m[0].length-m[3].length)==pv;pv=l.slice(0,m[0].length-m[3].length);"
    "var row=el('div','e '+kind(m[3])+(same?' k':'')),c=el('div','w',same?'':i>0?w.slice(i+1):w),b=el('div','m');"
    "if(i>0&&!same)c.appendChild(el('small','',w.slice(0,i)));row.appendChild(c);row.appendChild(el('div','u',same?'':m[2].trim()));"
    "m[3].split(/(?<=[.;])\\s+(?=[A-Za-z(])/).forEach(function(p,k){var j=p.lastIndexOf(': ');"
    "if(j>0&&(p.indexOf(' %, ')>0||/^benchmark (work|sends):|^motion benchmark: .*: /.test(p))){b.appendChild(el('div',k?'c':'c1',p.slice(0,j+1)));"
    "var g=el('div','g');p.slice(j+2).split(/, (?![^(]*\\))/).forEach(function(x){g.appendChild(el('span','',x))});b.appendChild(g)}"
    "else b.appendChild(el('div',k?'c':'c1',p))});row.appendChild(b);R.appendChild(row)})}"
    "document.getElementById('tl').onclick=function(){tab(false)};document.getElementById('tr').onclick=function(){tab(true)};"
    "document.getElementById('cp').onclick=function(){var b=this;navigator.clipboard.writeText(RT).then(function(){b.textContent='Copied'},"
    "function(){var g=document.createRange();g.selectNodeContents(R);getSelection().removeAllRanges();getSelection().addRange(g);b.textContent='Selected: Ctrl+C'})};"
    "var next=0,paused=false,o=document.getElementById('o'),s=document.getElementById('s');"
    "document.getElementById('p').onclick=function(){paused=!paused;this.textContent=paused?'Resume':'Pause'};"
    "document.getElementById('c').onclick=function(){o.textContent=''};"
    "function tick(){if(paused){setTimeout(tick,500);return}"
    "fetch('/log?since='+next,{cache:'no-store'}).then(function(r){"
    "var n=parseInt(r.headers.get('X-Next'),10);return r.text().then(function(t){"
    "if(n<next){o.textContent=''}"
    "if(t){o.textContent+=t;if(o.textContent.length>100000){o.textContent=o.textContent.slice(-60000)}"
    "window.scrollTo(0,document.body.scrollHeight)}"
    "next=n;s.textContent='live'})}).catch(function(){s.textContent='offline'})"
    ".then(function(){setTimeout(tick,1000)})}tick();"
    "</script></body></html>";

static esp_err_t console_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=30");   // fixed pages: switching between them is instant
    return httpd_resp_send(req, CONSOLE_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t log_get(httpd_req_t *req)
{
    static char out[4096];   // handlers run one at a time on the httpd task
    static char next_hdr[16];
    uint32_t since = 0;
    char q[48], v[16];
    if (httpd_req_get_url_query_str(req, q, sizeof q) == ESP_OK &&
        httpd_query_key_value(q, "since", v, sizeof v) == ESP_OK) {
        since = (uint32_t)strtoul(v, NULL, 10);
    }
    size_t n = 0;
    portENTER_CRITICAL(&s_ring_mux);
    uint32_t total = s_total;
    uint32_t oldest = total > s_ring_size ? total - s_ring_size : 0;
    if (since < oldest || since > total) {   // too old (wrapped) or from before a reboot
        since = oldest;
    }
    n = total - since;
    if (n > sizeof out) {
        n = sizeof out;
    }
    for (size_t i = 0; i < n; i++) {
        out[i] = s_ring[(since + i) % s_ring_size];
    }
    portEXIT_CRITICAL(&s_ring_mux);
    snprintf(next_hdr, sizeof next_hdr, "%u", (unsigned)(since + n));
    httpd_resp_set_hdr(req, "X-Next", next_hdr);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, out, (ssize_t)n);
}

static esp_err_t status_get(httpd_req_t *req)
{
    sf_link_info_t li;
    sf_link_get(&li);
    static char buf[1900], temps[500];   // static: small HTTP task stack
    temps[0] = 0;
    for (int i = 0; i < li.n_temps && i < SF_MAX_TEMPS; i++) {
        size_t len = strlen(temps);
        snprintf(temps + len, sizeof temps - len,
                 "%s{\"name\":\"%s\",\"state\":%d,\"c\":%.1f,\"text\":\"%s\"}", i ? "," : "", li.temps[i].name,
                 li.temps[i].state, (double)li.temps[i].c, li.temps[i].text);
    }
    snprintf(buf, sizeof buf,
             "{\"firmware\":\"" SF_VERSION "\",\"wip\":true,\"chip\":\"%s\",\"uptime_s\":%u,"
             "\"free_heap\":%u,\"min_free_heap\":%u,\"last_restart\":\"%s\",\"ssid\":\"%s\",\"board\":{\"state\":\"%s\",\"version\":\"%s\","
             "\"mcu\":\"%s\",\"clock_freq\":%u,\"configured\":%s,\"shutdown\":%s,\"crc_errors\":%u,"
             "\"attempts\":%u,\"tx_bytes\":%u,\"rx_bytes\":%u,\"loopback\":%s,\"last_rx_hex\":\"%s\","
             "\"cfg_ok\":%s,\"cfg_status\":\"%s\",\"first_led\":\"%s\",\"clock_measured\":%u,"
             "\"drift_ppm\":%.2f,\"rtt_ms\":%.3f,\"clock_samples\":%u,\"temps\":[%s]}",
             chip_model(), (unsigned)uptime_s(), (unsigned)esp_get_free_heap_size(),
             (unsigned)esp_get_minimum_free_heap_size(), reset_reason_str(), s_ssid,
             sf_link_state_name(li.state), li.version, li.mcu, (unsigned)li.clock_freq,
             li.configured ? "true" : "false", li.shutdown ? "true" : "false", (unsigned)li.crc_errors,
             (unsigned)li.attempts, (unsigned)li.tx_bytes, (unsigned)li.rx_bytes, li.loopback ? "true" : "false",
             li.rx_hex, li.cfg_ok ? "true" : "false", li.cfg_status, li.first_led, (unsigned)li.clk_hz,
             (double)li.drift_ppm, (double)li.rtt_ms, (unsigned)li.clk_samples, temps);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    // heaters, fans, motors, tests (ph::Machine::status_json): a full copy (it grew past a fixed 6 KB
    // buffer with the Tina 2's 16 tests, which cut the JSON off and broke the pages)
    char *mj = sf_machine_json_dup();
    httpd_resp_sendstr_chunk(req, buf);
    {
        static char fj[200];
        char nm[64];
        uint32_t dl = 0, ol = 0;
        bool zz = false;
        if (sf_store_info(nm, sizeof nm, &dl, &ol, &zz)) {
            snprintf(fj, sizeof fj, ",\"file\":{\"name\":\"%s\",\"size\":%u,\"orig\":%u,\"cap\":%u}", nm, (unsigned)dl,
                     (unsigned)ol, (unsigned)sf_store_capacity());
        } else {
            snprintf(fj, sizeof fj, ",\"file\":{\"name\":\"\",\"size\":0,\"orig\":0,\"cap\":%u}",
                     (unsigned)sf_store_capacity());
        }
        httpd_resp_sendstr_chunk(req, fj);
    }
    {
        static char tj[600];
        int k = snprintf(tj, sizeof tj, ",\"tier\":");
        sf_tier_json(tj + k, sizeof tj - (size_t)k);
        httpd_resp_sendstr_chunk(req, tj);
        char od[64];
        sf_oled_desc(od, sizeof od);
        snprintf(tj, sizeof tj, ",\"home_wifi\":{\"ssid\":\"%.32s\",\"state\":\"%s\",\"ip\":\"%s\"},\"clock_set\":%s,"
                 "\"oled\":\"%s\"",
                 s_settings.sta_ssid[0] ? "set" : "", s_settings.sta_ssid[0] ? s_sta_state : "off", s_sta_ip,
                 sf_clock_set() ? "true" : "false", od);
        httpd_resp_sendstr_chunk(req, tj);
    }
    httpd_resp_sendstr_chunk(req, ",\"machine\":");
    httpd_resp_sendstr_chunk(req, mj ? mj : "null");
    free(mj);
    httpd_resp_sendstr_chunk(req, "}");
    return httpd_resp_sendstr_chunk(req, NULL);
}

// In-place URL decode ('+' -> space, %XX -> byte). Returns false on a bad escape.
static bool url_decode(char *s)
{
    char *o = s;
    for (char *i = s; *i; i++) {
        if (*i == '+') {
            *o++ = ' ';
        } else if (*i == '%') {
            int hi, lo;
            char a = i[1], b = a ? i[2] : 0;
            hi = (a >= '0' && a <= '9') ? a - '0' : (a >= 'a' && a <= 'f') ? a - 'a' + 10 : (a >= 'A' && a <= 'F') ? a - 'A' + 10 : -1;
            lo = (b >= '0' && b <= '9') ? b - '0' : (b >= 'a' && b <= 'f') ? b - 'a' + 10 : (b >= 'A' && b <= 'F') ? b - 'A' + 10 : -1;
            if (hi < 0 || lo < 0) {
                return false;
            }
            *o++ = (char)(hi * 16 + lo);
            i += 2;
        } else {
            *o++ = *i;
        }
    }
    *o = 0;
    return true;
}

// /gcode?cmd=SET_LED%20LED%3Dboard_led%20RED%3D1 -> the reply, as plain text.
static esp_err_t gcode_get(httpd_req_t *req)
{
    static char q[400], cmd[300], reply[1800];   // static: small HTTP task stack
    if (httpd_req_get_url_query_str(req, q, sizeof q) != ESP_OK ||
        httpd_query_key_value(q, "cmd", cmd, sizeof cmd) != ESP_OK || !url_decode(cmd)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "use /gcode?cmd=COMMAND");
    }
    sf_console(cmd, reply, sizeof reply, 10000);   // TMC_CHECK can take a few seconds; "error: ..." on failure
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, reply, HTTPD_RESP_USE_STRLEN);
}

// The status-page colour links: SET_LED on the first [neopixel] in printer.cfg.
static esp_err_t board_led_get(httpd_req_t *req)
{
    static const struct { const char *name, *rgb; } named[] = {
        {"red", "RED=0.25 GREEN=0 BLUE=0"}, {"green", "RED=0 GREEN=0.25 BLUE=0"},
        {"blue", "RED=0 GREEN=0 BLUE=0.25"}, {"white", "RED=0.19 GREEN=0.19 BLUE=0.19"},
        {"off", "RED=0 GREEN=0 BLUE=0"}};
    char q[32], v[12];
    const char *rgb = NULL;
    if (httpd_req_get_url_query_str(req, q, sizeof q) == ESP_OK &&
        httpd_query_key_value(q, "c", v, sizeof v) == ESP_OK) {
        for (size_t i = 0; i < sizeof named / sizeof named[0]; i++) {
            if (strcmp(v, named[i].name) == 0) {
                rgb = named[i].rgb;
            }
        }
    }
    if (!rgb) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "use ?c=red|green|blue|white|off");
    }
    sf_link_info_t li;
    sf_link_get(&li);
    if (!li.first_led[0]) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "printer.cfg has no [neopixel] section");
    }
    static char line[96], reply[200];
    snprintf(line, sizeof line, "SET_LED LED=%s %s", li.first_led, rgb);
    if (!sf_console(line, reply, sizeof reply, 3000) || strncmp(reply, "error", 5) == 0) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, reply[0] ? reply : "board not ready");
    }
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/status");
    return httpd_resp_send(req, NULL, 0);
}

// HTML-escape src into dst (truncating safely).
static void html_escape(char *dst, size_t n, const char *src)
{
    size_t o = 0;
    for (; *src && o + 7 < n; src++) {
        const char *e = *src == '<' ? "&lt;" : *src == '>' ? "&gt;" : *src == '&' ? "&amp;" : *src == '"' ? "&quot;" : NULL;
        if (e) {
            size_t l = strlen(e);
            memcpy(dst + o, e, l);
            o += l;
        } else {
            dst[o++] = *src;
        }
    }
    dst[o] = 0;
}

// printer.cfg up to CONFIG_SKIPPER_CFG_KB KB (DECKHAND 8, CAPTAIN / SKIPPER 16), kept at the end of the
// gcode partition (storage.c; was 4 KB in NVS before 2026-10-27). +1 for the terminating NUL.
#define CFG_TEXT_MAX ((size_t)CONFIG_SKIPPER_CFG_KB * 1024 + 1)
#define SF_STR2(x) #x
#define SF_STR(x) SF_STR2(x)

// An empty string passed to httpd_resp_sendstr_chunk() ENDS the response, so never send one.
static void chunk(httpd_req_t *req, const char *s)
{
    if (s && *s) {
        httpd_resp_sendstr_chunk(req, s);
    }
}

static esp_err_t printer_cfg_get(httpd_req_t *req)
{
    char *text = malloc(CFG_TEXT_MAX);
    if (!text) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
    }
    sf_printer_cfg_copy(text, CFG_TEXT_MAX);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"printer.cfg\"");
    esp_err_t r = httpd_resp_sendstr(req, text);
    free(text);
    return r;
}

static esp_err_t config_get(httpd_req_t *req)
{
    char *text = malloc(CFG_TEXT_MAX), *esc = malloc(CFG_TEXT_MAX * 2), *rep = malloc(1024);
    if (!text || !esc || !rep) {
        free(text), free(esc), free(rep);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
    }
    char qs[32];
    bool want_default = httpd_req_get_url_query_str(req, qs, sizeof qs) == ESP_OK &&
                        (strstr(qs, "default=1") || strstr(qs, "default=ender3"));
    bool want_ender3 = want_default && strstr(qs, "default=1") == NULL;
    if (want_default && strstr(qs, "default=ender3")) {
        want_default = true;
    }
    if (want_ender3) {
        sf_printer_cfg_default_ender3(text, CFG_TEXT_MAX);
    } else if (want_default) {
        sf_printer_cfg_default(text, CFG_TEXT_MAX);
    } else {
        sf_printer_cfg_copy(text, CFG_TEXT_MAX);
    }
    html_escape(esc, CFG_TEXT_MAX * 2, text);
    sf_printer_cfg_report(text, 1024);
    html_escape(rep, 1024, text);
    if (!rep[0]) {
        snprintf(rep, 1024, "No problems found.");
    }
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr_chunk(req,
        "<!doctype html><html><head><meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<title>Skipper Flipper printer.cfg</title><style>"
        "body{background:#e9e6dc;color:#141414;font:16px/1.5 Georgia,serif;margin:0}"
        ".b{background:#141414;color:#e9e6dc;text-align:center;padding:10px;font-weight:700}"
        "main{max-width:760px;margin:0 auto;padding:16px}"
        "textarea{width:100%;height:24em;font:14px ui-monospace,Menlo,Consolas,monospace;border:2px solid #141414;"
        "background:#f7f5ee;box-sizing:border-box}pre{background:#f7f5ee;border:2px solid #141414;padding:8px;"
        "white-space:pre-wrap}button{font:inherit;background:#141414;color:#e9e6dc;border:0;padding:8px 18px}"
        "</style></head><body>" SF_NAV "<div class=\"b\">" SF_BANNER "</div><main><h1>printer.cfg</h1>"
        "<p>Klipper syntax. Used so far: <code>[extruder]</code> and <code>[heater_bed]</code> (heaters with "
        "Klipper's safety checks), <code>[printer]</code>, <code>[stepper_*]</code>, <code>[probe]</code>, "
        "<code>[tmc2209 ...]</code> (with sensorless homing), <code>[fan]</code>, <code>[heater_fan ...]</code>, <code>[verify_heater ...]</code>, "
        "<code>[neopixel ...]</code>, <code>[temperature_sensor ...]</code>, <code>[thermistor ...]</code>, "
        "<code>[homing_override]</code> and <code>[gcode_macro NAME]</code> (plain G-code lines; Klipper's {templates} not yet). "
        "Other sections are listed below as not used yet. Saving reconfigures the board (it restarts its Klipper "
        "firmware once). Maximum " SF_STR(CONFIG_SKIPPER_CFG_KB) " KB (comments count too). <a href=\"/printer.cfg\">Download</a> &middot; "
        "Load a default into the editor (not saved until you press Save): <a href=\"/config?default=1\">SKR Pico</a> &middot; "
        "<a href=\"/config?default=ender3\">Creality Ender 3 with an SKR Pico (end switches)</a>. Other printers: see docs/configs/ in the project (a tuned WEEDO Tina 2 with sensorless homing is there).</p>"
        "<h2>Last check</h2><pre>");
    if (want_default) {
        chunk(req, want_ender3 ? "The Ender 3 printer.cfg is in the editor below. It is not saved yet: press Save and apply.\n\n"
                              : "The DEFAULT printer.cfg is in the editor below. It is not saved yet: press Save and apply.\n\n");
    }
    chunk(req, rep);
    httpd_resp_sendstr_chunk(req, "</pre><form method=\"post\" action=\"/config\"><textarea name=\"cfg\" "
                                  "spellcheck=\"false\">");
    chunk(req, esc);
    httpd_resp_sendstr_chunk(req, "</textarea><br><button type=\"submit\">Save and apply</button></form>"
                                  "<h2>Earlier versions (SD card)</h2><p id=\"bk\">Each Save keeps the version it "
                                  "replaces on the SD card (skipper/cfg/, the last 10).</p>"
                                  "<script>fetch('/sdls?d=skipper/cfg').then(function(r){return r.json()}).then("
                                  "function(j){if(!j.files.length)return;var h='';j.files.sort(function(a,b){return "
                                  "a.name<b.name?1:-1}).forEach(function(f){h+='<a href=\"/sdfile?f=skipper/cfg/'+"
                                  "encodeURIComponent(f.name)+'\">'+f.name+'</a> '});document.getElementById('bk')"
                                  ".innerHTML+='<br>'+h+'<br><small>Open one, copy the text into the editor above "
                                  "and Save to go back to it.</small>'}).catch(function(){})</script>"
                                  "<p><a href=\"/status\">Back to status</a></p></main></body></html>");
    free(text), free(esc), free(rep);
    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t config_post(httpd_req_t *req)
{
    if (req->content_len > CFG_TEXT_MAX * 3 + 16) {   // url-encoding can triple the size
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "printer.cfg is too big (" SF_STR(CONFIG_SKIPPER_CFG_KB) " KB max)");
    }
    size_t len = req->content_len;
    char *body = malloc(len + 1);
    if (!body) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
    }
    size_t got = 0;
    while (got < len) {
        int r = httpd_req_recv(req, body + got, len - got);
        if (r <= 0) {
            free(body);
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "could not read the form");
        }
        got += (size_t)r;
    }
    body[got] = 0;
    // The form sends exactly one field: cfg=<url-encoded text>.
    if (strncmp(body, "cfg=", 4) != 0 || strchr(body + 4, '&') || !url_decode(body + 4)) {
        free(body);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "expected one form field: cfg");
    }
    char *text = body + 4;
    {   // a browser sends CRLF line ends: they do not count (and are not stored)
        char *w = text;
        for (const char *r = text; *r; r++)
            if (*r != '\r') *w++ = *r;
        *w = 0;
    }
    if (strlen(text) >= CFG_TEXT_MAX) {
        free(body);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "printer.cfg is too big (" SF_STR(CONFIG_SKIPPER_CFG_KB) " KB max)");
    }
    {   // keep the version being replaced on the SD card (skipper/cfg/, last 10)
        char *old = malloc(CFG_TEXT_MAX);
        if (old) {
            sf_printer_cfg_copy(old, CFG_TEXT_MAX);
            if (strcmp(old, text) != 0) {
                sf_cfg_backup(old);
            }
            free(old);
        }
    }
    bool ok = sf_printer_cfg_set(text);
    free(body);
    if (!ok) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "could not save printer.cfg");
    }
    ESP_LOGW(TAG, "printer.cfg saved; reconfiguring the board");
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_sendstr(req, "<!doctype html><meta http-equiv=\"refresh\" content=\"3;url=/config\">"
                                   "<body style=\"font:16px Georgia,serif;background:#e9e6dc\"><p>Saved. The board is "
                                   "being reconfigured; this page shows the result in a few seconds.</p></body>");
}

static int form_int(const char *body, const char *key, int dflt)
{
    char v[12];
    if (httpd_query_key_value(body, key, v, sizeof v) != ESP_OK) {
        return dflt;
    }
    char *end;
    long n = strtol(v, &end, 10);
    return (end != v && *end == 0) ? (int)n : dflt;
}

static esp_err_t settings_get(httpd_req_t *req)
{
    const sf_settings_t *s = &s_settings;
    static char ssid_esc[200];
    html_escape(ssid_esc, sizeof ssid_esc, s->sta_ssid);
    char oled_now[64];
    sf_oled_desc(oled_now, sizeof oled_now);
    // from the heap per request (not static): DECKHAND needs its internal RAM for the printer link
    const size_t bufn = 10500;
    char *buf = malloc(bufn);
    if (!buf) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
    }
    snprintf(buf, bufn,
             "<!doctype html><html><head><meta charset=\"utf-8\">"
             "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
             "<title>Skipper Flipper settings</title><style>"
             "body{background:#e9e6dc;color:#141414;font:16px/1.5 Georgia,serif;margin:0}"
             ".b{background:#141414;color:#e9e6dc;text-align:center;padding:10px;font-weight:700}"
             "main{max-width:640px;margin:0 auto;padding:16px}label{display:block;margin:10px 0 2px;font-weight:700}"
             "input,select{font:inherit;padding:4px;border:2px solid #141414;background:#f7f5ee}"
             "button{font:inherit;background:#141414;color:#e9e6dc;border:0;padding:8px 18px;margin-top:16px}"
             "small{color:#555}</style></head><body>" SF_NAV "<div class=\"b\">" SF_BANNER "</div><main>"
             "<h1>Settings</h1><form method=\"post\" action=\"/settings\">"
             "<label>Printer board link</label><select name=\"link_on\">"
             "<option value=\"1\"%s>on</option><option value=\"0\"%s>off (leave the pins alone)</option></select>"
             "<label>UART TX GPIO (to the board's RX)</label><input name=\"uart_tx\" value=\"%d\" size=\"4\">"
             "<label>UART RX GPIO (from the board's TX)</label><input name=\"uart_rx\" value=\"%d\" size=\"4\">"
             "<label>Baud</label><input name=\"baud\" value=\"%d\" size=\"8\">"
             "<small><br>Klipper's serial default is 250000.</small>"
             "<label>Status LED type</label><select name=\"led_type\">"
             "<option value=\"0\"%s>none</option><option value=\"1\"%s>plain LED</option>"
             "<option value=\"2\"%s>WS2812 colour LED</option></select>"
             "<label>Colour LED order</label><select name=\"led_rgb\">"
             "<option value=\"0\"%s>standard (green, red, blue)</option>"
             "<option value=\"1\"%s>red, green, blue</option></select>"
             "<small><br>An error should blink red. If it blinks green instead, switch this.</small>"
             "<label>Status LED GPIO</label><input name=\"led_gpio\" value=\"%d\" size=\"4\">"
             "<small><br>Default: WS2812 colour LED on GPIO21 (Waveshare ESP32-S3-Zero). Other boards differ: if "
             "nothing lights, try the other type or the LED GPIO from your board's pinout.</small>"
             "<p><small>The printer board's LEDs are set up in <a href=\"/config\">printer.cfg</a>.</small></p>"
             "<h2>SD card (G-code files)</h2><label>SD card</label><select name=\"sd_on\">"
             "<option value=\"1\"%s>on</option><option value=\"0\"%s>off (leave the pins alone)</option></select>"
             "<label>CS GPIO</label><input name=\"sd_cs\" value=\"%d\" size=\"4\">"
             "<label>SCK GPIO</label><input name=\"sd_sck\" value=\"%d\" size=\"4\">"
             "<label>MOSI GPIO (to the card's DI)</label><input name=\"sd_mosi\" value=\"%d\" size=\"4\">"
             "<label>MISO GPIO (from the card's DO)</label><input name=\"sd_miso\" value=\"%d\" size=\"4\">"
             "<small><br>Default CS 10, SCK 12, MOSI 11, MISO 13. Status now: %s.</small>"
             "<h2>Home WiFi</h2><p><small>Optional. The ESP32 also joins your own WiFi (2.4 GHz), so the pages and "
             "slicer uploads work from any device at home; its own network SkipperFlipper-XXXX stays on as well "
             "(once joined, it moves to your router's channel). Status now: %s%s%s.</small></p>"
             "<label>Network name</label><input name=\"sta_ssid\" value=\"%s\" maxlength=\"32\" "
             "autocomplete=\"off\">"
             "<label>Password</label><input name=\"sta_pass\" type=\"password\" maxlength=\"63\" "
             "placeholder=\"%s\" autocomplete=\"new-password\">"
             "<small><br>Leave the name empty to switch this off.</small>"
             "<h2>Little OLED display</h2><p><small>A cheap 4-pin I2C OLED (GND, VCC to 3V3, SDA, SCL): temperatures, "
             "print progress, and what the printer is doing. Status now: %s.</small></p>"
             "<label>Type</label><select name=\"oled_type\">"
             "<option value=\"1\"%s>0.96in 128x64, SSD1306 / SSD1315 (most modules)</option>"
             "<option value=\"2\"%s>0.91in 128x32, SSD1306</option>"
             "<option value=\"3\"%s>1.3in 128x64, SH1106</option>"
             "<option value=\"0\"%s>none (leave the pins alone)</option></select>"
             "<label>SDA GPIO</label><input name=\"oled_sda\" value=\"%d\" size=\"4\">"
             "<label>SCL GPIO</label><input name=\"oled_scl\" value=\"%d\" size=\"4\">"
             "<label>Mounted</label><select name=\"oled_flip\"><option value=\"0\"%s>this way up</option>"
             "<option value=\"1\"%s>upside down</option></select>"
             "<small><br>Default SDA 8, SCL 9. Nothing connected is fine: it looks three times after a start, then stops.</small>"
#if CONFIG_SKIPPER_PRO
             "<h2>Buttons on the case</h2><p><small>Touch pads (a bolt on a wire) or push switches: preheat, load, "
             "unload, E-stop... <a href=\"/buttons\">Set them up</a>.</small></p>"
#endif
             "<button type=\"submit\">Save and restart</button></form>"
             "<p><a href=\"/about\">About this build</a> &middot; <a href=\"/update\">Firmware update</a> &middot; "
             "<a href=\"/status\">Back to status</a></p></main></body></html>",
             s->link_enabled ? " selected" : "", s->link_enabled ? "" : " selected", s->uart_tx, s->uart_rx,
             s->baud, s->led_type == SF_LED_NONE ? " selected" : "", s->led_type == SF_LED_PLAIN ? " selected" : "",
             s->led_type == SF_LED_WS2812 ? " selected" : "", s->led_rgb ? "" : " selected",
             s->led_rgb ? " selected" : "", s->led_gpio, s->sd_enabled ? " selected" : "", s->sd_enabled ? "" : " selected",
             s->sd_cs, s->sd_sck, s->sd_mosi, s->sd_miso, sf_sd_mounted() ? "card mounted" : sf_sd_error(),
             s->sta_ssid[0] ? sf_sta_state() : "off", sf_sta_ip()[0] ? ", http://" : "", sf_sta_ip(), ssid_esc,
             s->sta_pass[0] ? "(saved; leave empty to keep it)" : "", oled_now,
             s->oled_type == SF_OLED_AUTO ? " selected" : "", s->oled_type == SF_OLED_SSD1306_32 ? " selected" : "",
             s->oled_type == SF_OLED_SH1106 ? " selected" : "", s->oled_type == SF_OLED_OFF ? " selected" : "",
             s->oled_sda, s->oled_scl, s->oled_flip ? "" : " selected", s->oled_flip ? " selected" : "");
    httpd_resp_set_type(req, "text/html");
    esp_err_t r = httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    free(buf);
    return r;
}

static void restart_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(800));   // let the reply reach the browser first
    esp_restart();
}

static esp_err_t settings_post(httpd_req_t *req)
{
    static char body[1200];   // static: small HTTP task stack (handlers run one at a time)
    int len = req->content_len < sizeof body - 1 ? (int)req->content_len : (int)sizeof body - 1;
    int got = 0;
    while (got < len) {
        int r = httpd_req_recv(req, body + got, len - got);
        if (r <= 0) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "could not read the form");
        }
        got += r;
    }
    body[got] = 0;
    sf_settings_t n = s_settings;
    n.link_enabled = form_int(body, "link_on", n.link_enabled) ? 1 : 0;
    n.uart_tx = form_int(body, "uart_tx", n.uart_tx);
    n.uart_rx = form_int(body, "uart_rx", n.uart_rx);
    n.baud = form_int(body, "baud", n.baud);
    n.led_type = form_int(body, "led_type", n.led_type);
    n.led_gpio = form_int(body, "led_gpio", n.led_gpio);
    n.led_rgb = form_int(body, "led_rgb", n.led_rgb) ? 1 : 0;
    n.sd_enabled = form_int(body, "sd_on", n.sd_enabled) ? 1 : 0;
    n.sd_cs = form_int(body, "sd_cs", n.sd_cs);
    n.sd_sck = form_int(body, "sd_sck", n.sd_sck);
    n.sd_mosi = form_int(body, "sd_mosi", n.sd_mosi);
    n.sd_miso = form_int(body, "sd_miso", n.sd_miso);
    n.oled_type = form_int(body, "oled_type", n.oled_type);
    n.oled_sda = form_int(body, "oled_sda", n.oled_sda);
    n.oled_scl = form_int(body, "oled_scl", n.oled_scl);
    n.oled_flip = form_int(body, "oled_flip", n.oled_flip) ? 1 : 0;
    {
        char v[200];
        if (httpd_query_key_value(body, "sta_ssid", v, sizeof v) == ESP_OK && url_decode(v)) {
            strlcpy(n.sta_ssid, v, sizeof n.sta_ssid);
        }
        if (httpd_query_key_value(body, "sta_pass", v, sizeof v) == ESP_OK && url_decode(v) && v[0]) {
            strlcpy(n.sta_pass, v, sizeof n.sta_pass);   // empty field: keep the saved password
        }
        if (!n.sta_ssid[0]) {
            n.sta_pass[0] = 0;
        }
    }
    const char *problem = NULL;
    if (!GPIO_IS_VALID_OUTPUT_GPIO(n.uart_tx) || !GPIO_IS_VALID_GPIO(n.uart_rx) || n.uart_tx == n.uart_rx) {
        problem = "UART GPIOs must be two different valid pins";
    } else if (n.uart_tx == 19 || n.uart_tx == 20 || n.uart_rx == 19 || n.uart_rx == 20) {
        problem = "GPIO19/20 are the USB port; pick other pins";
    } else if (n.baud < 9600 || n.baud > 2000000) {
        problem = "baud must be 9600 to 2000000";
    } else if (n.led_type < 0 || n.led_type > 2) {
        problem = "unknown LED type";
    } else if (n.led_type != SF_LED_NONE &&
               (!GPIO_IS_VALID_OUTPUT_GPIO(n.led_gpio) || n.led_gpio == n.uart_tx || n.led_gpio == n.uart_rx ||
                n.led_gpio == 19 || n.led_gpio == 20)) {
        problem = "LED GPIO must be a valid output pin, not a UART or USB pin";
    }
    if (!problem && n.sd_enabled) {
        problem = sf_sd_pin_problem(&n);
    }
    if (!problem && (n.oled_type < SF_OLED_OFF || n.oled_type > SF_OLED_SH1106)) {
        problem = "unknown OLED type";
    }
    if (!problem) {
        problem = sf_oled_pin_problem(&n);
    }
    if (!problem) {
        problem = sf_panel_conflict(&n);
    }
    if (!problem && n.sta_pass[0] && strlen(n.sta_pass) < 8) {
        problem = "a WiFi password has at least 8 characters";
    }
    if (problem) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, problem);
    }
    if (!sf_settings_save(&n)) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "could not save settings");
    }
    ESP_LOGW(TAG, "settings saved; restarting");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req, "<!doctype html><meta http-equiv=\"refresh\" content=\"6;url=/\">"
                            "<body style=\"font:16px Georgia,serif;background:#e9e6dc\"><p>Saved. Restarting; "
                            "this page reloads in a few seconds (reconnect to the WiFi if needed).</p></body>");
    xTaskCreate(restart_task, "restart", 2048, NULL, 1, NULL);
    return ESP_OK;
}

// Unknown paths (phones checking for internet) go to the status page: captive portal.
static esp_err_t not_found(httpd_req_t *req, httpd_err_code_t err)
{
    httpd_resp_set_status(req, "302 Temporary Redirect");
    // relative, so it also works when the page was opened over the home WiFi
    httpd_resp_set_hdr(req, "Location", "/");
    return httpd_resp_send(req, "Redirect to the Skipper Flipper page", HTTPD_RESP_USE_STRLEN);
}

static void http_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8192;
    cfg.max_uri_handlers = 56;
    cfg.lru_purge_enable = true;   // phones open many connections; drop the oldest instead of refusing
    httpd_handle_t srv = NULL;
    ESP_ERROR_CHECK(httpd_start(&srv, &cfg));
    static const httpd_uri_t root = {.uri = "/status", .method = HTTP_GET, .handler = root_get};
    static const httpd_uri_t home = {.uri = "/", .method = HTTP_GET, .handler = printer_get};   // 2026-10-27: the Printer page first
    static const httpd_uri_t stat = {.uri = "/status.json", .method = HTTP_GET, .handler = status_get};
    static const httpd_uri_t cons = {.uri = "/console", .method = HTTP_GET, .handler = console_get};
    static const httpd_uri_t logu = {.uri = "/log", .method = HTTP_GET, .handler = log_get};
    static const httpd_uri_t setg = {.uri = "/settings", .method = HTTP_GET, .handler = settings_get};
    static const httpd_uri_t setp = {.uri = "/settings", .method = HTTP_POST, .handler = settings_post};
    static const httpd_uri_t bled = {.uri = "/board_led", .method = HTTP_GET, .handler = board_led_get};
    static const httpd_uri_t gcod = {.uri = "/gcode", .method = HTTP_GET, .handler = gcode_get};
    static const httpd_uri_t cfgd = {.uri = "/printer.cfg", .method = HTTP_GET, .handler = printer_cfg_get};
    static const httpd_uri_t uijs = {.uri = "/ui.js", .method = HTTP_GET, .handler = ui_js_get};
    static const httpd_uri_t testp = {.uri = "/test", .method = HTTP_GET, .handler = test_get};
    static const httpd_uri_t hlog = {.uri = "/heater_log.csv", .method = HTTP_GET, .handler = heater_log_get};
    static const httpd_uri_t prt = {.uri = "/printer", .method = HTTP_GET, .handler = printer_get};
    static const httpd_uri_t upl = {.uri = "/upload", .method = HTTP_POST, .handler = upload_post};
    static const httpd_uri_t cfgg = {.uri = "/config", .method = HTTP_GET, .handler = config_get};
    static const httpd_uri_t cfgp = {.uri = "/config", .method = HTTP_POST, .handler = config_post};
    static const httpd_uri_t filg = {.uri = "/files.json", .method = HTTP_GET, .handler = files_get};
    static const httpd_uri_t sdp = {.uri = "/sd", .method = HTTP_POST, .handler = sd_post};
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &root));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &home));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &stat));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &cons));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &logu));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &setg));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &setp));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &bled));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &gcod));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &cfgd));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &uijs));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &testp));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &hlog));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &prt));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &upl));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &cfgg));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &cfgp));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &filg));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &sdp));
    sf_tier_register(srv);
    sf_extras_register(srv);
    sf_slicer_api_register(srv);
    sf_ota_register(srv);
    sf_crash_register(srv);
    sf_pro_register(srv);
    sf_moonraker_register(srv);   // /websocket for the Pasta Inspector dongle (CAPTAIN+, needs HTTPD_WS_SUPPORT)
    sf_tune_register(srv);
    sf_accel_init(&s_settings, srv);
    sf_panel_start(&s_settings, srv);
    httpd_register_err_handler(srv, HTTPD_404_NOT_FOUND, not_found);
}

void app_main(void)
{
    sf_crash_boot();   // before anything logs: keeps the previous run's last words
    ring_alloc();
    s_prev_vprintf = esp_log_set_vprintf(sf_vprintf);
    ESP_LOGW(TAG, "==== Skipper Flipper ESP32-S3 " SF_VERSION " - %s - " SF_BANNER, sf_tier_title());
    ESP_LOGW(TAG, "memory: %u KB PSRAM, look-ahead up to %d moves, console history %u KB",
             (unsigned)(sf_psram_bytes() / 1024), sf_lookahead_max(), (unsigned)(s_ring_size / 1024));
    ESP_LOGW(TAG, "chip: %s, IDF %s, last restart: %s", chip_model(), esp_get_idf_version(), reset_reason_str());
    sf_report("started: Skipper Flipper %s; last restart: %s", sf_build_id(), reset_reason_str());
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    sf_settings_load(&s_settings);
    sf_bench_boot(&s_settings);   // only in QEMU timing builds (CONFIG_SKIPPER_BENCH_AT_BOOT)
    sf_sd_mount(&s_settings);   // no card is fine: the Printer page says so, and Remount tries again
    sf_extras_start();
    sf_crash_save();          // the SD event log (this start, with the restart reason, is its first entry)
    sf_link_start(&s_settings);
    sf_led_start(&s_settings);
    sf_oled_start(&s_settings);
    xTaskCreate(heartbeat_task, "heartbeat", 3072, NULL, 3, NULL);
#if CONFIG_SKIPPER_BENCH_AT_BOOT
    // QEMU test builds only: QEMU has no WiFi (and no PSRAM), and the start-up benchmark needs the memory
    ESP_LOGW(TAG, "QEMU test build: no WiFi; app_main stack never used: %u bytes; heap %s",
             (unsigned)uxTaskGetStackHighWaterMark(NULL), heap_caps_check_integrity_all(true) ? "intact" : "DAMAGED");
    return;
#endif
    wifi_start_ap();
    http_start();
    sf_ota_boot_check();
    dns_server_config_t dns = DNS_SERVER_CONFIG_SINGLE("*", "WIFI_AP_DEF");   // every name -> 192.168.4.1
    start_dns_server(&dns);
}
