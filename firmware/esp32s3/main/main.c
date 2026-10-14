// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
//
// Skipper Flipper ESP32-S3 firmware: a WiFi access point and web page, a Klipper link to the
// printer board over UART, clock sync, and a printer.cfg that configures the board's NeoPixels.
// It does NOT move motors or heat anything yet.
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_chip_info.h"
#include "esp_event.h"
#include "esp_flash.h"
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

static uint32_t uptime_s(void) { return (uint32_t)(esp_timer_get_time() / 1000000); }

// ---- live console: everything the firmware logs is also kept in a ring buffer that the web
// page /console reads over WiFi. s_total counts bytes ever written, so a client asks for "since N".
#define LOG_RING 8192
static char s_ring[LOG_RING];
static uint32_t s_total;
static portMUX_TYPE s_ring_mux = portMUX_INITIALIZER_UNLOCKED;
static vprintf_like_t s_prev_vprintf;

static void ring_append(const char *p, size_t n)
{
    portENTER_CRITICAL(&s_ring_mux);
    for (size_t i = 0; i < n; i++) {
        s_ring[s_total % LOG_RING] = p[i];
        s_total++;
    }
    portEXIT_CRITICAL(&s_ring_mux);
}

static int sf_vprintf(const char *fmt, va_list ap)
{
    char buf[256];
    va_list ap2;
    va_copy(ap2, ap);
    int n = vsnprintf(buf, sizeof buf, fmt, ap2);
    va_end(ap2);
    if (n > 0 && !xPortInIsrContext()) {
        ring_append(buf, (size_t)n < sizeof buf ? (size_t)n : sizeof buf - 1);
    }
    return s_prev_vprintf ? s_prev_vprintf(fmt, ap) : vprintf(fmt, ap);
}

static sf_settings_t s_settings;

static void heartbeat_task(void *arg)
{
    uint32_t ticks = 0;
    for (;;) {
        if (++ticks % 5 == 0) {
            ESP_LOGW(TAG, "alive: uptime %us, free heap %u bytes", (unsigned)uptime_s(),
                     (unsigned)esp_get_free_heap_size());
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base != WIFI_EVENT) {
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

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

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

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    // 15 dBm instead of the ~20 dBm default: plenty for one room, and smaller current spikes on a
    // weak 5 V supply (full-power WiFi bursts are the usual cause of ESP32 brownout resets).
    esp_wifi_set_max_tx_power(60);
    ESP_LOGW(TAG, "WiFi AP \"%s\" (WPA2) on channel %d, page at http://192.168.4.1/", s_ssid,
             CONFIG_SKIPPER_AP_CHANNEL);
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
    static char buf[8200];
    snprintf(buf, sizeof buf,
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
             "</style></head><body><div class=\"b\">" SF_BANNER "</div><main>"
             "<h1>Skipper Flipper</h1><p>ESP32-S3 host. It talks Klipper to the printer board and runs, from "
             "printer.cfg: heaters (with Klipper's safety checks), fans, LEDs, TMC2209 drivers and motor tests. "
             "It cannot move axes, home or print yet.</p>"
             "<p><a href=\"/test\" style=\"display:inline-block;background:#141414;color:#e9e6dc;padding:8px 16px;"
             "font-weight:700;text-decoration:none\">TEST BENCH &rarr;</a> <a href=\"/printer\" style=\"display:inline-block;"
             "background:#141414;color:#e9e6dc;padding:8px 16px;font-weight:700;text-decoration:none\">PRINTER &rarr;</a></p><div id=\"tp\"></div><table>"
             "<tr><td>Firmware</td><td>" SF_VERSION "</td></tr>"
             "<tr><td>Chip</td><td>%s</td></tr>"
             "<tr><td>Flash</td><td>%u KiB</td></tr>"
             "<tr><td>Uptime</td><td>%u s</td></tr>"
             "<tr><td>Free heap</td><td>%u bytes</td></tr>"
             "<tr><td>WiFi network</td><td>%s</td></tr>"
             "<tr><td>ESP-IDF</td><td>%s</td></tr>"
             "<tr><td>Last restart</td><td>%s</td></tr>%s%s"
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
             "&middot; <a href=\"/status.json\">status.json</a></p>"
             "</main></body></html>",
             chip_model(), (unsigned)(flash / 1024), (unsigned)uptime_s(),
             (unsigned)esp_get_free_heap_size(), s_ssid, esp_get_idf_version(), reset_reason_str(), board, diag);
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
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
    ".cal{border-top:2px solid #141414;padding:6px 0}.cal input{width:5.5em}"
    "</style></head><body><div class=\"b\">" SF_BANNER "</div><main><h1>Test bench</h1>"
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
    "with a ruler or calipers, type both here, Work out, then Save (the board restarts). stallGuard: Set tries a "
    "driver_SGTHRS at once (higher = stops more easily); Save keeps it in printer.cfg. The box goes red while the "
    "driver's DIAG says stalled (and stays red 3 s so short triggers show).</p><div id=\"cl\"></div>"
    "<h2>Drivers and fans</h2><div id=\"dv\" class=\"dt\"></div>"
    "<h2>Console</h2><form id=\"cf\"><input id=\"ci\" style=\"width:16em\" placeholder=\"M104 S75\" autocomplete=\"off\">"
    " <button>Send</button></form><pre id=\"out\">Replies appear here. Type HELP for every command.</pre>"
    "<p><a href=\"/\">Status</a> &middot; <a href=\"/config\">printer.cfg</a> &middot; <a href=\"/console\">Live log</a></p>"
    "</main><script src=\"/ui.js\"></script><script>"
    "var out=document.getElementById('out'),built='',calb='';"
    "function E(s){return String(s).replace(/[&<>\"]/g,function(c){return{'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;'}[c]})}"
    "function V(id,d){var e=document.getElementById(id);return e?e.value:d}"
    "function calBuild(st){var h='';st.forEach(function(s){var n=E(s.name);"
    "h+='<div class=\"cal\"><b>'+n+'</b> rotation_distance <input type=\"number\" step=\"0.001\" id=\"rd_'+n+'\" value=\"'+s.rd+'\"> mm = <span id=\"spm_'+n+'\">'+s.spm+'</span> steps/mm <button data-rds=\"'+n+'\">Save</button><br><span class=\"dt\">check: moved <input type=\"number\" id=\"mc_'+n+'\" value=\"50\"> mm, measured <input type=\"number\" step=\"0.01\" id=\"mm_'+n+'\"> mm <button data-rdc=\"'+n+'\">Work out</button></span>';"
    "if(s.sgthrs!==undefined)h+='<br>stallGuard driver_SGTHRS <input type=\"number\" min=\"0\" max=\"255\" id=\"sgv_'+n+'\" value=\"'+s.sgthrs+'\"> <button data-sgset=\"'+n+'\">Set</button><button data-sgsave=\"'+n+'\">Save</button> DIAG <span class=\"dg\" id=\"dg_'+n+'\">?</span>';"
    "h+='</div>'});document.getElementById('cl').innerHTML=h}"
    "function dgShow(e,s){if(!e)return;var on=s.diag==1||s.diag_recent;e.className='dg '+(on?'on':s.diag==0?'off':'');e.textContent=on?'TRIGGERED':s.diag==0?'free':'?'}"
    "document.getElementById('cl').onclick=function(e){var d=e.target.dataset;if(!d)return;"
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
    "'<b>SHUT DOWN:</b> '+E(m.reason)+' <button data-restart=\"1\">Restart board</button>';"
    "sfTemps(document.getElementById('tp'),m.sensors,m.shutdown);"
    "var ids=m.tests.map(function(t){return t.id}).join(',');if(ids!=built){build(m.tests);built=ids}"
    "var cn=m.steppers.map(function(s){return s.name+s.rd+s.sgthrs}).join(',');if(cn!=calb){calBuild(m.steppers);calb=cn}"
    "m.steppers.forEach(function(s){if(s.sgthrs===undefined)return;dgShow(document.getElementById('dg_'+s.name),s);"
    "dgShow(document.getElementById('dgt_'+s.name),s);var sp=document.getElementById('spm_'+s.name);if(sp)sp.textContent=s.spm});"
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
    ".bar{height:14px;border:2px solid #141414;background:#f7f5ee}.bar div{height:100%;background:#141414;width:0}"
    "#out{background:#141414;color:#e9e6dc;padding:8px;min-height:3em;white-space:pre-wrap}.dt{font-size:.85em}"
    "</style></head><body><div class=\"b\">" SF_BANNER "</div><main><h1>Printer</h1>"
    "<button class=\"stop\" id=\"stop\">STOP EVERYTHING</button><div id=\"sd\" class=\"sd\" hidden></div>"
    "<div id=\"nb\" class=\"sd\" hidden></div>"
    "<h2>Temperatures</h2><div id=\"tp\"></div><div>Hotend <input type=\"number\" id=\"te\" value=\"200\"> &deg;C "
    "<button data-c=\"he\">Set</button> <button data-g=\"M104 S0\">Off</button> &nbsp; Fan "
    "<button data-g=\"M106 S255\">100%</button><button data-g=\"M106 S128\">50%</button><button data-g=\"M107\">off</button></div>"
    "<div id=\"bedrow\" hidden>Bed <input type=\"number\" id=\"tb\" value=\"60\"> &deg;C <button data-c=\"hb\">Set</button> "
    "<button data-g=\"M140 S0\">Off</button></div>"
    "<h2>Position</h2><div class=\"pos\" id=\"pos\">-</div><div class=\"dt\" id=\"hm\"></div>"
    "<div><button class=\"k\" data-g=\"G28\">Home all</button><button data-g=\"G28 X\">Home X</button>"
    "<button data-g=\"G28 Y\">Home Y</button><button data-g=\"G28 Z\">Home Z</button><button data-g=\"PROBE\">Probe</button>"
    "<button data-g=\"M84\">Motors off</button><button data-g=\"QUERY_ENDSTOPS\">Endstops?</button></div>"
    "<p>Step <select id=\"st\"><option>0.1</option><option selected>1</option><option>10</option><option>50</option></select> mm</p>"
    "<div class=\"jg\"><span></span><button data-j=\"Y 1\">Y+</button><button data-j=\"Z 1\">Z+</button>"
    "<button data-j=\"X -1\">X-</button><button data-j=\"X 0\" disabled>&middot;</button><button data-j=\"X 1\">X+</button>"
    "<span></span><button data-j=\"Y -1\">Y-</button><button data-j=\"Z -1\">Z-</button></div>"
    "<div>Filament: <button data-e=\"5\">Extrude 5 mm</button><button data-e=\"-5\">Retract 5 mm</button> "
    "<span class=\"dt\">(hotend at least 170 &deg;C)</span></div>"
    "<h2>Print</h2><div id=\"fl\">-</div><div><input type=\"file\" id=\"fi\" accept=\".gcode,.gco,.g,.txt\"> "
    "<button id=\"upb\">Upload</button> <span id=\"ups\" class=\"dt\"></span></div>"
    "<div><button class=\"k\" data-g=\"SDCARD_PRINT_FILE\">Print</button><button data-g=\"PAUSE\">Pause</button>"
    "<button data-g=\"RESUME\">Resume</button><button class=\"ab\" data-g=\"CANCEL_PRINT\">Cancel</button></div>"
    "<div class=\"bar\"><div id=\"pb\"></div></div><div id=\"ps\" class=\"dt\"></div>"
    "<p class=\"dt\">The file is compressed in the browser before upload (about 5 times smaller) and unpacked by the "
    "ESP32-S3 while printing. Plain G-code only: slicer macros (START_PRINT ...) are skipped and listed. "
    "No moves or homing while printing.</p>"
    "<h2>Console</h2><form id=\"cf\"><input id=\"ci\" style=\"width:16em\" placeholder=\"G1 X10 F3000\" autocomplete=\"off\">"
    " <button>Send</button></form><pre id=\"out\">Replies appear here.</pre>"
    "<p><a href=\"/\">Status</a> &middot; <a href=\"/test\">Test bench</a> &middot; <a href=\"/config\">printer.cfg</a></p>"
    "</main><script src=\"/ui.js\"></script><script>"
    "var out=document.getElementById('out'),cap=0;function $(i){return document.getElementById(i)}"
    "function E(s){return String(s).replace(/[&<>\"]/g,function(c){return{'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;'}[c]})}"
    "document.body.onclick=function(e){var d=e.target.dataset;if(!d)return;"
    "if(d.g)sfCmd(d.g,out);if(d.c=='he')sfCmd('M104 S'+$('te').value,out);if(d.c=='hb')sfCmd('M140 S'+$('tb').value,out);"
    "if(d.j){var p=d.j.split(' ');sfCmd('JOG AXIS='+p[0]+' DIST='+(parseFloat(p[1])*parseFloat($('st').value)),out)}"
    "if(d.e)sfCmd('JOG AXIS=E DIST='+d.e,out);if(d.restart)sfCmd('FIRMWARE_RESTART',out)};"
    "$('stop').onclick=function(){sfCmd('M112',out)};"
    "$('cf').onsubmit=function(e){e.preventDefault();var i=$('ci');if(i.value)sfCmd(i.value,out);i.value=''};"
    "$('upb').onclick=async function(){var f=$('fi').files[0],s=$('ups');if(!f){s.textContent='choose a file first';return}"
    "try{var body=f,z=0;if(window.CompressionStream){s.textContent='compressing...';"
    "body=await new Response(f.stream().pipeThrough(new CompressionStream('deflate'))).blob();z=1}"
    "if(cap&&body.size>cap){s.textContent='too big: '+Math.round(body.size/1024)+' KB'+(z?' compressed':'')+', the space is '+Math.round(cap/1024)+' KB';return}"
    "s.textContent='uploading '+Math.round(body.size/1024)+' KB...';"
    "var r=await fetch('/upload?name='+encodeURIComponent(f.name)+'&orig='+f.size+'&z='+z,{method:'POST',body:body});"
    "s.textContent=await r.text()}catch(x){s.textContent='upload failed: '+x}};"
    "function poll(){fetch('/status.json',{cache:'no-store'}).then(function(r){return r.json()}).then(function(j){"
    "var m=j.machine,nb=$('nb'),sd=$('sd');cap=j.file?j.file.cap:0;"
    "$('fl').textContent=j.file&&j.file.name?'Stored: '+j.file.name+' ('+Math.round(j.file.orig/1024)+' KB, '+Math.round(j.file.size/1024)+' KB stored)':'No file stored yet.';"
    "if(!m){sfTempsLost($('tp'));nb.hidden=false;nb.textContent='Printer board: '+j.board.state+(j.board.cfg_status?' - '+j.board.cfg_status:'');return}"
    "nb.hidden=true;sd.hidden=!m.shutdown;if(m.shutdown)sd.innerHTML=(m.power_off_now?'<b style=\"color:#c4161c\">SWITCH OFF THE 12-24 V POWER SUPPLY.</b> ':'')+"
    "'<b>SHUT DOWN:</b> '+E(m.reason)+' <button data-restart=\"1\">Restart board</button>';"
    "sfTemps($('tp'),m.sensors,m.shutdown);$('bedrow').hidden=!m.sensors.some(function(s){return s.name=='heater_bed'});"
    "var t=m.toolhead;if(t&&t.ok){$('pos').textContent='X '+t.pos[0].toFixed(2)+'  Y '+t.pos[1].toFixed(2)+'  Z '+t.pos[2].toFixed(2)+'  E '+t.pos[3].toFixed(1)"
    "+'   homed: '+(t.homed||'none')+(t.busy?'  (moving)':'');$('hm').textContent=t.msg}"
    "else if(t)$('pos').textContent='Moves are not set up: '+t.why;"
    "var p=m.print;$('pb').style.width=(p.progress*100).toFixed(1)+'%';"
    "$('ps').textContent=p.state+(p.file?' '+p.file:'')+(p.state=='printing'||p.state=='paused'?' '+(p.progress*100).toFixed(1)+'%, '+Math.round(p.elapsed/60)+' min':'')+(p.msg?' - '+p.msg:'')"
    "}).catch(function(){sfTempsLost($('tp'))}).then(function(){setTimeout(poll,1000)})}poll();"
    "</script></body></html>";

static esp_err_t printer_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, PRINTER_HTML, HTTPD_RESP_USE_STRLEN);
}

// POST /upload?name=&orig=&z=1 with the (compressed) file as the body: stored in the gcode partition.
static esp_err_t upload_post(httpd_req_t *req)
{
    static char buf[4096];
    char q[200], v[100], name[64] = "upload.gcode";
    uint32_t orig = 0;
    bool z = false;
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
    }
    size_t len = req->content_len;
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
    "pre{margin:0;padding:8px;white-space:pre-wrap;word-break:break-all}#s{float:right}</style></head><body>"
    "<div class=\"b\">" SF_BANNER "</div>"
    "<div class=\"t\"><button id=\"p\">Pause</button><button id=\"c\">Clear</button>"
    "<a href=\"/\" style=\"color:#e9e6dc\">status</a><span id=\"s\">connecting...</span></div>"
    "<pre id=\"o\"></pre><script>"
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
    uint32_t oldest = total > LOG_RING ? total - LOG_RING : 0;
    if (since < oldest || since > total) {   // too old (wrapped) or from before a reboot
        since = oldest;
    }
    n = total - since;
    if (n > sizeof out) {
        n = sizeof out;
    }
    for (size_t i = 0; i < n; i++) {
        out[i] = s_ring[(since + i) % LOG_RING];
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
             "\"free_heap\":%u,\"ssid\":\"%s\",\"board\":{\"state\":\"%s\",\"version\":\"%s\","
             "\"mcu\":\"%s\",\"clock_freq\":%u,\"configured\":%s,\"shutdown\":%s,\"crc_errors\":%u,"
             "\"attempts\":%u,\"tx_bytes\":%u,\"rx_bytes\":%u,\"loopback\":%s,\"last_rx_hex\":\"%s\","
             "\"cfg_ok\":%s,\"cfg_status\":\"%s\",\"first_led\":\"%s\",\"clock_measured\":%u,"
             "\"drift_ppm\":%.2f,\"rtt_ms\":%.3f,\"clock_samples\":%u,\"temps\":[%s]}",
             chip_model(), (unsigned)uptime_s(), (unsigned)esp_get_free_heap_size(), s_ssid,
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
    httpd_resp_set_hdr(req, "Location", "/");
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

#define CFG_TEXT_MAX 4096   // keeps an NVS blob replace safe in the 24 KB nvs partition

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
                        (strstr(qs, "default=1") || strstr(qs, "default=tina2"));
    bool want_tina2 = want_default && strstr(qs, "default=1") == NULL;
    if (want_default && strstr(qs, "default=tina2")) {
        want_default = true;
    }
    if (want_tina2) {
        sf_printer_cfg_default_tina2(text, CFG_TEXT_MAX);
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
        "</style></head><body><div class=\"b\">" SF_BANNER "</div><main><h1>printer.cfg</h1>"
        "<p>Klipper syntax. Used so far: <code>[extruder]</code> and <code>[heater_bed]</code> (heaters with "
        "Klipper's safety checks; the extruder's motor for tests), <code>[stepper_*]</code> (motor tests only), "
        "<code>[tmc2209 ...]</code>, <code>[fan]</code>, <code>[heater_fan ...]</code>, <code>[verify_heater ...]</code>, "
        "<code>[neopixel ...]</code>, <code>[temperature_sensor ...]</code> and <code>[thermistor ...]</code>. "
        "Other sections are listed below as not used yet. Saving reconfigures the board (it restarts its Klipper "
        "firmware once). Maximum 4 KB. <a href=\"/printer.cfg\">Download</a> &middot; "
        "Load a default into the editor (not saved until you press Save): <a href=\"/config?default=1\">SKR Pico</a> &middot; "
        "<a href=\"/config?default=tina2\">WEEDO Tina 2 with an SKR Pico (sensorless homing)</a>.</p>"
        "<h2>Last check</h2><pre>");
    if (want_default) {
        chunk(req, want_tina2 ? "The Tina 2 printer.cfg is in the editor below. It is not saved yet: press Save and apply.\n\n"
                              : "The DEFAULT printer.cfg is in the editor below. It is not saved yet: press Save and apply.\n\n");
    }
    chunk(req, rep);
    httpd_resp_sendstr_chunk(req, "</pre><form method=\"post\" action=\"/config\"><textarea name=\"cfg\" "
                                  "spellcheck=\"false\">");
    chunk(req, esc);
    httpd_resp_sendstr_chunk(req, "</textarea><br><button type=\"submit\">Save and apply</button></form>"
                                  "<p><a href=\"/\">Back to status</a></p></main></body></html>");
    free(text), free(esc), free(rep);
    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t config_post(httpd_req_t *req)
{
    if (req->content_len > CFG_TEXT_MAX * 3 + 16) {   // url-encoding can triple the size
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "printer.cfg is too big (4 KB max)");
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
    const char *text = body + 4;
    if (strlen(text) >= CFG_TEXT_MAX) {
        free(body);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "printer.cfg is too big (4 KB max)");
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
    static char buf[3600];   // static: small HTTP task stack
    snprintf(buf, sizeof buf,
             "<!doctype html><html><head><meta charset=\"utf-8\">"
             "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
             "<title>Skipper Flipper settings</title><style>"
             "body{background:#e9e6dc;color:#141414;font:16px/1.5 Georgia,serif;margin:0}"
             ".b{background:#141414;color:#e9e6dc;text-align:center;padding:10px;font-weight:700}"
             "main{max-width:640px;margin:0 auto;padding:16px}label{display:block;margin:10px 0 2px;font-weight:700}"
             "input,select{font:inherit;padding:4px;border:2px solid #141414;background:#f7f5ee}"
             "button{font:inherit;background:#141414;color:#e9e6dc;border:0;padding:8px 18px;margin-top:16px}"
             "small{color:#555}</style></head><body><div class=\"b\">" SF_BANNER "</div><main>"
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
             "<button type=\"submit\">Save and restart</button></form>"
             "<p><a href=\"/\">Back to status</a></p></main></body></html>",
             s->link_enabled ? " selected" : "", s->link_enabled ? "" : " selected", s->uart_tx, s->uart_rx,
             s->baud, s->led_type == SF_LED_NONE ? " selected" : "", s->led_type == SF_LED_PLAIN ? " selected" : "",
             s->led_type == SF_LED_WS2812 ? " selected" : "", s->led_rgb ? "" : " selected",
             s->led_rgb ? " selected" : "", s->led_gpio);
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static void restart_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(800));   // let the reply reach the browser first
    esp_restart();
}

static esp_err_t settings_post(httpd_req_t *req)
{
    char body[256];
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
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    return httpd_resp_send(req, "Redirect to the Skipper Flipper page", HTTPD_RESP_USE_STRLEN);
}

static void http_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8192;
    cfg.max_uri_handlers = 20;
    cfg.lru_purge_enable = true;   // phones open many connections; drop the oldest instead of refusing
    httpd_handle_t srv = NULL;
    ESP_ERROR_CHECK(httpd_start(&srv, &cfg));
    static const httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = root_get};
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
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &root));
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
    httpd_register_err_handler(srv, HTTPD_404_NOT_FOUND, not_found);
}

void app_main(void)
{
    s_prev_vprintf = esp_log_set_vprintf(sf_vprintf);
    ESP_LOGW(TAG, "Skipper Flipper ESP32-S3 " SF_VERSION " - " SF_BANNER);
    ESP_LOGW(TAG, "chip: %s, IDF %s, last restart: %s", chip_model(), esp_get_idf_version(), reset_reason_str());
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    sf_settings_load(&s_settings);
    sf_link_start(&s_settings);
    sf_led_start(&s_settings);
    xTaskCreate(heartbeat_task, "heartbeat", 3072, NULL, 3, NULL);
    wifi_start_ap();
    http_start();
    dns_server_config_t dns = DNS_SERVER_CONFIG_SINGLE("*", "WIFI_AP_DEF");   // every name -> 192.168.4.1
    start_dns_server(&dns);
}
