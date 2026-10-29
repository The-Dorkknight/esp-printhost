// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Buttons on the printer's case (2026-10-26), as in KlipperTFT-Pico: up to four inputs, each either a
// capacitive touch pad (a bolt through the case on a wire; ESP32-S3 GPIO1-14 can sense touch) or a
// push switch from the pin to GND. Defaults: GPIO2 Preheat, GPIO4 Home, GPIO5 Unload, GPIO6 E-stop
// (Home and E-stop only after a 1 s hold; two buttons at once are ignored); off until switched on at /buttons. Each press runs one console
// line through sf_console (the link task), the same as typing it: SF_PREHEAT, SF_LOAD_FILAMENT, ...
// (host/src/machine_panel.cpp). The press logic (baseline, hysteresis, debounce, hold) is
// ph::TouchFilter / ph::PressTimer, unit-tested on the PC (host/src/panel.cpp).
//   GET /buttons, POST /buttons (save), GET /buttons.json[?practice=1] (live readings; practice: the
//   buttons light up on the page but do nothing, for setting the sensitivity safely).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>

#include "driver/gpio.h"
#include "driver/touch_pad.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "ph/panel.h"
#include "sdkconfig.h"
#include "sf.h"

#if CONFIG_SKIPPER_PRO   // CAPTAIN / SKIPPER only: DECKHAND's 2 MB flash is full (2026-10-26)

static const char *TAG = "buttons";
#define NB ph::kPanelButtons

typedef struct {
    int mode;                 // SF_PANEL_*
    int sens;                 // touch sensitivity, percent rise that counts as a touch (1-50)
    int pin[NB];
    int action[NB];           // ph::PanelAction
    char custom[NB][48];      // the command for action "custom"
    ph::PanelPresets pre;
} panel_cfg_t;

static panel_cfg_t s_cfg;          // as saved (web page)
static panel_cfg_t s_run;          // what the task runs with (copied on a change)
static sf_settings_t s_set;          // the main settings, for pin conflicts
static SemaphoreHandle_t s_lock;
static volatile bool s_reconfig;
static int64_t s_practice_until;     // us
static TaskHandle_t s_task;

struct Live {
    uint32_t raw = 0;
    float base = 0, change = 0;
    bool pressed = false;
    char last[64] = "";
};
static Live s_live[NB];
static char s_msg[16];
static int64_t s_msg_until;
static char s_status[96] = "off";

static double now_s(void) { return (double)esp_timer_get_time() / 1e6; }

static void flash(const char *m)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    snprintf(s_msg, sizeof s_msg, "%s", m);
    s_msg_until = esp_timer_get_time() + 2500000;
    xSemaphoreGive(s_lock);
}

extern "C" bool sf_panel_msg(char *out, size_t n)
{
    if (!s_lock) return false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool on = s_msg[0] && esp_timer_get_time() < s_msg_until;
    if (on) snprintf(out, n, "%s", s_msg);
    xSemaphoreGive(s_lock);
    return on;
}

// ---------------------------------------------------------------- settings (NVS "panel")

static void defaults(panel_cfg_t *c)
{
    memset(c, 0, sizeof *c);
    c->mode = SF_PANEL_OFF;
    c->sens = 5;
    static const int P[NB] = {2, 4, 5, 6};   // as KlipperTFT-Pico on the ESP32-S3; all touch-capable
    for (int i = 0; i < NB; i++) {
        c->pin[i] = P[i];
        c->action[i] = (int)ph::panel_default_action(i);
    }
    c->pre = ph::PanelPresets();
}

static void load(panel_cfg_t *c)
{
    defaults(c);
    nvs_handle_t h;
    if (nvs_open("panel", NVS_READONLY, &h) != ESP_OK) return;
    size_t n = sizeof *c;
    panel_cfg_t t;
    int32_t ver = 0;
    if (nvs_get_i32(h, "ver", &ver) == ESP_OK && ver == 1 && nvs_get_blob(h, "cfg", &t, &n) == ESP_OK && n == sizeof t)
        *c = t;
    nvs_close(h);
}

static bool save(const panel_cfg_t *c)
{
    nvs_handle_t h;
    if (nvs_open("panel", NVS_READWRITE, &h) != ESP_OK) return false;
    bool ok = nvs_set_i32(h, "ver", 1) == ESP_OK && nvs_set_blob(h, "cfg", c, sizeof *c) == ESP_OK && nvs_commit(h) == ESP_OK;
    nvs_close(h);
    return ok;
}

static bool active(const panel_cfg_t *c, int i)
{
    return c->mode != SF_PANEL_OFF && c->action[i] != (int)ph::PanelAction::None;
}

// Pins: not the UART, LED, SD card or OLED; not USB, start-up or flash/PSRAM pins; no repeats.
// Touch needs GPIO1-14.
static const char *pin_problem(const panel_cfg_t *c, const sf_settings_t *s)
{
    if (c->mode == SF_PANEL_OFF) return NULL;
    static char why[96];
    for (int i = 0; i < NB; i++) {
        if (!active(c, i)) continue;
        int g = c->pin[i];
        snprintf(why, sizeof why, "button %d: GPIO%d ", i + 1, g);
        size_t l = strlen(why);
        const char *bad = NULL;
        if (!GPIO_IS_VALID_GPIO(g)) bad = "is not a GPIO";
        else if (c->mode == SF_PANEL_TOUCH && (g < 1 || g > 14)) bad = "cannot sense touch (touch pads: GPIO1-14)";
        else if (g == 19 || g == 20) bad = "is the USB port";
        else if (g == 0 || g == 3 || g == 45 || g == 46) bad = "is a start-up (strapping) pin";
        else if (g >= 26 && g <= 37) bad = "belongs to the flash / PSRAM";
        else if (s->link_enabled && (g == s->uart_tx || g == s->uart_rx)) bad = "is a printer board UART pin";
        else if (s->led_type != SF_LED_NONE && g == s->led_gpio) bad = "is the status LED";
        else if (s->sd_enabled && (g == s->sd_cs || g == s->sd_sck || g == s->sd_mosi || g == s->sd_miso)) bad = "is an SD card pin";
        else if (s->oled_type != SF_OLED_OFF && (g == s->oled_sda || g == s->oled_scl)) bad = "is an OLED pin";
        for (int j = 0; !bad && j < i; j++)
            if (active(c, j) && c->pin[j] == g) bad = "is used by two buttons";
        if (bad) {
            snprintf(why + l, sizeof why - l, "%s", bad);
            return why;
        }
    }
    return NULL;
}

extern "C" bool sf_panel_uses_pin(int g)
{
    if (!s_lock) return false;
    for (int i = 0; i < NB; i++)
        if (active(&s_cfg, i) && s_cfg.pin[i] == g) return true;
    return false;
}

// For the main settings page: a pin there that the buttons already use.
extern "C" const char *sf_panel_conflict(const sf_settings_t *s)
{
    if (!s_lock || s_cfg.mode == SF_PANEL_OFF) return NULL;
    const char *p = pin_problem(&s_cfg, s);
    return p ? "a pin is already used by the buttons on the case (see the Buttons page)" : NULL;
}

// ---------------------------------------------------------------- the task

static bool s_touch_on;

static void teardown(void)
{
    if (s_touch_on) {
        touch_pad_fsm_stop();
        touch_pad_deinit();
        s_touch_on = false;
    }
}

static bool setup(const panel_cfg_t *c)
{
    for (int i = 0; i < NB; i++) s_live[i] = Live();
    if (c->mode == SF_PANEL_OFF) {
        snprintf(s_status, sizeof s_status, "off");
        return false;
    }
    const char *bad = pin_problem(c, &s_set);
    if (bad) {
        snprintf(s_status, sizeof s_status, "not started: %s", bad);
        ESP_LOGW(TAG, "%s", s_status);
        return false;
    }
    if (c->mode == SF_PANEL_TOUCH) {
        if (touch_pad_init() != ESP_OK) {
            snprintf(s_status, sizeof s_status, "touch sensor would not start");
            return false;
        }
        for (int i = 0; i < NB; i++)
            if (active(c, i)) touch_pad_config((touch_pad_t)c->pin[i]);
        touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
        touch_pad_fsm_start();
        s_touch_on = true;
        vTaskDelay(pdMS_TO_TICKS(100));
    } else {
        for (int i = 0; i < NB; i++) {
            if (!active(c, i)) continue;
            gpio_reset_pin((gpio_num_t)c->pin[i]);
            gpio_set_direction((gpio_num_t)c->pin[i], GPIO_MODE_INPUT);
            gpio_set_pull_mode((gpio_num_t)c->pin[i], GPIO_PULLUP_ONLY);
        }
    }
    snprintf(s_status, sizeof s_status, "%s, learning the untouched level: keep hands off for 2 s",
             c->mode == SF_PANEL_TOUCH ? "touch pads" : "switches to GND");
    ESP_LOGW(TAG, "buttons on (%s)", c->mode == SF_PANEL_TOUCH ? "touch" : "switches");
    return true;
}

static void run_action(int i, ph::PanelAction a)
{
    const panel_cfg_t &c = s_run;
    std::string line = ph::panel_line(a, c.pre, c.custom[i]);
    if (line.empty()) return;
    if (esp_timer_get_time() < s_practice_until) {   // practice: shown on the page only
        snprintf(s_live[i].last, sizeof s_live[i].last, "practice: would run %.40s", line.c_str());
        return;
    }
    flash(ph::panel_flash(a));
    static char reply[160];
    ESP_LOGW(TAG, "button %d: %s", i + 1, line.c_str());
    if (!sf_console(line.c_str(), reply, sizeof reply, 3000)) snprintf(reply, sizeof reply, "error: no answer");
    if (strncmp(reply, "error", 5) == 0) {
        flash(strstr(reply, "print is running") ? "PRINTING" : "NOT NOW");
        ESP_LOGW(TAG, "button %d: %s", i + 1, reply);
    }
    snprintf(s_live[i].last, sizeof s_live[i].last, "%.60s", reply);
}

static void panel_task(void *arg)
{
    ph::TouchFilter tf[NB];
    ph::PressTimer pt[NB];
    bool on = false;
    for (;;) {
        if (s_reconfig) {
            s_reconfig = false;
            teardown();
            xSemaphoreTake(s_lock, portMAX_DELAY);
            s_run = s_cfg;
            xSemaphoreGive(s_lock);
            on = setup(&s_run);
            for (int i = 0; i < NB; i++) {
                tf[i] = ph::TouchFilter(s_run.sens / 100.0);
                pt[i] = ph::PressTimer(ph::panel_hold_action((ph::PanelAction)s_run.action[i]));
            }
        }
        if (!on) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        double t = now_s();
        bool settled = true;
        uint32_t down = 0;
        for (int i = 0; i < NB; i++) {
            if (!active(&s_run, i)) continue;
            bool pressed;
            if (s_run.mode == SF_PANEL_TOUCH) {
                uint32_t raw = 0;
                touch_pad_read_raw_data((touch_pad_t)s_run.pin[i], &raw);
                pressed = tf[i].update(raw, t);
                settled &= tf[i].ready();
                s_live[i].raw = raw;
                s_live[i].base = (float)tf[i].baseline();
                s_live[i].change = (float)tf[i].change();
            } else {
                pressed = gpio_get_level((gpio_num_t)s_run.pin[i]) == 0;
            }
            s_live[i].pressed = pressed;
            if (pressed) down |= 1u << i;
        }
        // two at once (a short between wires, a hand across two bolts): neither counts
        uint32_t ok = ph::panel_guard(down);
        static bool warned;
        if (down && !ok && !warned) {
            warned = true;
            flash("2 AT ONCE");
            ESP_LOGW(TAG, "two or more buttons pressed at once: ignored (wires touching?)");
        } else if (!down) {
            warned = false;
        }
        for (int i = 0; i < NB; i++) {
            if (!active(&s_run, i)) continue;
            ph::PressTimer::Event e = pt[i].update((ok >> i) & 1, t);
            if (e == ph::PressTimer::Fire) run_action(i, (ph::PanelAction)s_run.action[i]);
            else if (e == ph::PressTimer::TooShort) {
                if (esp_timer_get_time() < s_practice_until)
                    snprintf(s_live[i].last, sizeof s_live[i].last, "practice: too short (hold 1 s)");
                else
                    flash("HOLD 1 S");
            }
        }
        if (settled && strstr(s_status, "learning"))
            snprintf(s_status, sizeof s_status, "%s, ready", s_run.mode == SF_PANEL_TOUCH ? "touch pads" : "switches to GND");
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ---------------------------------------------------------------- web page

static const char PAGE[] =
    R"(<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Skipper Flipper buttons</title><style>
body{background:#e9e6dc;color:#141414;font:16px/1.45 Georgia,serif;margin:0}
.b{background:#141414;color:#e9e6dc;text-align:center;padding:10px;font-weight:700;letter-spacing:.06em}
main{max-width:760px;margin:0 auto;padding:12px}h1{font:700 2rem Impact,Arial Black,sans-serif;border-bottom:4px solid #141414;margin:.3em 0}
h2{font:700 1.3rem Impact,Arial Black,sans-serif;margin:1.1em 0 .3em}.c{background:#f7f5ee;border:2px solid #141414;padding:10px;margin:8px 0}
input,select{font:inherit;padding:3px;border:2px solid #141414;background:#fff;max-width:100%;box-sizing:border-box}input.n{width:4.5em}
td input.n{width:3.2em}td select,td input.w{width:100%}.tw{overflow-x:auto}td.lv{min-width:84px}
button{font:700 15px Georgia,serif;background:#141414;color:#e9e6dc;border:2px solid #141414;padding:7px 12px;cursor:pointer}
table{border-collapse:collapse;width:100%}td,th{border:1px solid #141414;padding:4px 5px;text-align:left;font-size:14px;vertical-align:top}
.on{background:#ffd400}.bar{height:9px;background:#141414}.bw{background:#cfcabb;width:100%;height:9px;position:relative}
.th{position:absolute;top:-3px;width:2px;height:15px;background:#b3141c}small{color:#555}
</style></head><body>)" SF_NAV R"(<div class="b">)" SF_BANNER R"(</div><main><h1>Buttons on the case</h1>
<p><small>New (2026-10-26), not yet tried on a printer. Up to four buttons, as on KlipperTFT-Pico: a bolt through the
case on a wire (touch), or a push switch to GND. Wiring and safety: docs/BUTTONS.md. Home and E-stop only fire after a
1 s hold; two at once do nothing. Load / unload heat the hotend first and are refused while printing.</small></p>
<p>Now: <b id="st">...</b></p>
<form method="post" action="/buttons"><div class="c">
<label>Type <select name="mode"><option value="0">off</option><option value="1">touch pads (bolt on a wire, GPIO1-14)</option>
<option value="2">push switches to GND</option></select></label>
<label> &nbsp;Touch sensitivity <input class="n" name="sens">% <small>(a touch is a rise of at least this much)</small></label>
<div class="tw"><table id="tb"><tr><th>#</th><th>GPIO</th><th>Does</th><th>Live</th></tr></table></div>
<p><label><input type="checkbox" id="pr" style="width:auto"> Practice: the buttons light up here but do nothing (while this page is open)</label></p>
</div><h2>Values</h2><div class="c">
Preheat: hotend <input class="n" name="pe"> C, bed <input class="n" name="pb"> C<br>
Load / unload: heat to at least <input class="n" name="lt"> C; load <input class="n" name="ll"> mm at <input class="n" name="ls"> mm/s;
unload <input class="n" name="ul"> mm at <input class="n" name="us"> mm/s<br>
<small>A <code>[gcode_macro LOAD_FILAMENT]</code> / <code>UNLOAD_FILAMENT</code> in printer.cfg runs instead, if you have one.</small>
</div><p><button>Save</button> <small>Free pins on an ESP32-S3-Zero with the SD card and OLED: 1, 2, 4, 5, 6, 7, 14.</small></p></form>
<script>
var A=[],first=true;function q(n){return document.querySelector('[name='+n+']')}
function row(i,b){var o='<tr id="r'+i+'"><td>'+(i+1)+'</td><td><input class="n" name="p'+i+'" value="'+b.pin+'"></td><td><select name="a'+i+'">';
for(var k=0;k<A.length;k++)o+='<option value="'+k+'"'+(k==b.action?' selected':'')+'>'+A[k]+'</option>';
return o+'</select><br><input class="w" name="c'+i+'" placeholder="custom command" maxlength="47" value="'+b.custom.replace(/"/g,'&quot;')+'"></td><td class="lv" id="l'+i+'"></td></tr>'}
function tick(){fetch('/buttons.json'+(document.getElementById('pr').checked?'?practice=1':''),{cache:'no-store'}).then(function(r){return r.json()}).then(function(j){
document.getElementById('st').textContent=j.status;
if(first){first=false;A=j.actions;q('mode').value=j.mode;q('sens').value=j.sens;var p=j.pre;q('pe').value=p.pe;q('pb').value=p.pb;
q('lt').value=p.lt;q('ll').value=p.ll;q('ls').value=p.ls;q('ul').value=p.ul;q('us').value=p.us;
var t=document.getElementById('tb');for(var i=0;i<j.btn.length;i++)t.insertAdjacentHTML('beforeend',row(i,j.btn[i]))}
for(var i=0;i<j.btn.length;i++){var b=j.btn[i],c=document.getElementById('l'+i),s='';
if(j.mode==1){var pc=Math.max(0,Math.min(100,b.change/(2*j.sens)*100));
s=(b.change>=0?'+':'')+b.change.toFixed(1)+'% <div class="bw"><div class="bar" style="width:'+pc+'%"></div><div class="th" style="left:50%"></div></div><small>raw '+b.raw+'</small>'}
s+=b.pressed?' <b>PRESSED</b>':'';if(b.last)s+='<br><small>'+b.last.replace(/</g,'&lt;')+'</small>';c.innerHTML=s;
document.getElementById('r'+i).className=b.pressed?'on':''}}).catch(function(){})}
tick();setInterval(tick,400);
</script></main></body></html>)";

static esp_err_t page_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, PAGE, HTTPD_RESP_USE_STRLEN);
}

static void json_str(std::string &o, const char *s)
{
    o += '"';
    for (; *s; s++) {
        if (*s == '"' || *s == '\\') o += '\\';
        if ((unsigned char)*s >= 32) o += *s;
    }
    o += '"';
}

static esp_err_t json_get(httpd_req_t *req)
{
    char q[32];
    if (httpd_req_get_url_query_str(req, q, sizeof q) == ESP_OK && strstr(q, "practice=1"))
        s_practice_until = esp_timer_get_time() + 3000000;
    char b[200];
    std::string o = "{\"status\":";
    json_str(o, s_status);
    snprintf(b, sizeof b, ",\"mode\":%d,\"sens\":%d,\"pre\":{\"pe\":%d,\"pb\":%d,\"lt\":%d,\"ll\":%d,\"ls\":%d,\"ul\":%d,\"us\":%d},\"actions\":[",
             s_cfg.mode, s_cfg.sens, s_cfg.pre.preheat_ext, s_cfg.pre.preheat_bed, s_cfg.pre.load_temp, s_cfg.pre.load_len,
             s_cfg.pre.load_speed, s_cfg.pre.unload_len, s_cfg.pre.unload_speed);
    o += b;
    for (int k = 0; k < ph::kPanelActions; k++) {
        if (k) o += ',';
        json_str(o, ph::panel_action_label((ph::PanelAction)k));
    }
    o += "],\"btn\":[";
    for (int i = 0; i < NB; i++) {
        const Live &l = s_live[i];
        snprintf(b, sizeof b, "%s{\"pin\":%d,\"action\":%d,\"raw\":%u,\"base\":%.0f,\"change\":%.2f,\"pressed\":%s,\"custom\":",
                 i ? "," : "", s_cfg.pin[i], s_cfg.action[i], (unsigned)l.raw, l.base, l.change * 100, l.pressed ? "true" : "false");
        o += b;
        json_str(o, s_cfg.custom[i]);
        o += ",\"last\":";
        json_str(o, l.last);
        o += '}';
    }
    o += "]}";
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, o.data(), (ssize_t)o.size());
}

static void url_decode(char *s)
{
    char *w = s;
    for (char *r = s; *r; r++) {
        if (*r == '+') *w++ = ' ';
        else if (*r == '%' && r[1] && r[2]) {
            char h[3] = {r[1], r[2], 0};
            *w++ = (char)strtol(h, NULL, 16);
            r += 2;
        } else *w++ = *r;
    }
    *w = 0;
}

static int fint(const char *body, const char *k, int dflt, int lo, int hi, bool *ok)
{
    char v[16];
    if (httpd_query_key_value(body, k, v, sizeof v) != ESP_OK) return dflt;
    char *e;
    long x = strtol(v, &e, 10);
    if (e == v || *e || x < lo || x > hi) {
        *ok = false;
        return dflt;
    }
    return (int)x;
}

static esp_err_t page_post(httpd_req_t *req)
{
    char body[900];
    int n = req->content_len < (int)sizeof body - 1 ? req->content_len : (int)sizeof body - 1, got = 0;
    while (got < n) {
        int r = httpd_req_recv(req, body + got, n - got);
        if (r <= 0) return ESP_FAIL;
        got += r;
    }
    body[got] = 0;
    panel_cfg_t c = s_cfg;
    bool ok = true;
    c.mode = fint(body, "mode", c.mode, 0, 2, &ok);
    c.sens = fint(body, "sens", c.sens, 1, 50, &ok);
    for (int i = 0; i < NB; i++) {
        char k[4];
        snprintf(k, sizeof k, "p%d", i);
        c.pin[i] = fint(body, k, c.pin[i], 0, 48, &ok);
        snprintf(k, sizeof k, "a%d", i);
        c.action[i] = fint(body, k, c.action[i], 0, ph::kPanelActions - 1, &ok);
        snprintf(k, sizeof k, "c%d", i);
        char v[sizeof c.custom[i] * 3];
        if (httpd_query_key_value(body, k, v, sizeof v) == ESP_OK) {
            url_decode(v);
            snprintf(c.custom[i], sizeof c.custom[i], "%.47s", v);
        }
        if (c.action[i] == (int)ph::PanelAction::Custom && !c.custom[i][0] && c.mode != SF_PANEL_OFF)
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "a custom button needs its command");
    }
    c.pre.preheat_ext = fint(body, "pe", c.pre.preheat_ext, 0, 300, &ok);
    c.pre.preheat_bed = fint(body, "pb", c.pre.preheat_bed, 0, 130, &ok);
    c.pre.load_temp = fint(body, "lt", c.pre.load_temp, 150, 300, &ok);
    c.pre.load_len = fint(body, "ll", c.pre.load_len, 1, 1000, &ok);
    c.pre.load_speed = fint(body, "ls", c.pre.load_speed, 1, 100, &ok);
    c.pre.unload_len = fint(body, "ul", c.pre.unload_len, 1, 1000, &ok);
    c.pre.unload_speed = fint(body, "us", c.pre.unload_speed, 1, 100, &ok);
    if (!ok) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "a number is out of range");
    const char *bad = pin_problem(&c, &s_set);
    if (bad) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, bad);
    if (!save(&c)) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "could not save");
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cfg = c;
    xSemaphoreGive(s_lock);
    s_reconfig = true;
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/buttons");
    return httpd_resp_send(req, NULL, 0);
}

// ---------------------------------------------------------------- start

extern "C" void sf_panel_start(const sf_settings_t *s, void *httpd)
{
    s_set = *s;
    s_lock = xSemaphoreCreateMutex();
    load(&s_cfg);
    s_reconfig = true;
    xTaskCreate(panel_task, "buttons", 4096, NULL, 3, &s_task);
    httpd_handle_t srv = (httpd_handle_t)httpd;
    static const httpd_uri_t g = {.uri = "/buttons", .method = HTTP_GET, .handler = page_get, .user_ctx = NULL};
    static const httpd_uri_t p = {.uri = "/buttons", .method = HTTP_POST, .handler = page_post, .user_ctx = NULL};
    static const httpd_uri_t j = {.uri = "/buttons.json", .method = HTTP_GET, .handler = json_get, .user_ctx = NULL};
    httpd_register_uri_handler(srv, &g);
    httpd_register_uri_handler(srv, &p);
    httpd_register_uri_handler(srv, &j);
}

#else
extern "C" void sf_panel_start(const sf_settings_t *s, void *httpd) {}
extern "C" bool sf_panel_uses_pin(int g) { return false; }
extern "C" const char *sf_panel_conflict(const sf_settings_t *s) { return NULL; }
extern "C" bool sf_panel_msg(char *out, size_t n) { return false; }
#endif
