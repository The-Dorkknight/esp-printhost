// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
//
// Pasta Inspector dongle support (2026-10-31): the part of Moonraker's websocket (ws://<host>/websocket,
// JSON-RPC 2.0) that the spaghetti detector uses, so an ESP32-CAM dongle can pause the print, skip
// the object that failed (EXCLUDE_OBJECT) or stop it (M112). Written from the dongle's protocol, not
// from its source (that is PolyForm Noncommercial; this is GPL-3.0): the two programs only talk over
// the network. Not tested against a real dongle yet.
//
// Methods: server.connection.identify, printer.objects.list, printer.objects.subscribe,
// printer.gcode.script, printer.emergency_stop. Anything else: error -32601. Status goes out as
// notify_status_update about every 200 ms to a subscribed client.
#include "sdkconfig.h"
#include "sf.h"

#if CONFIG_SKIPPER_PRO && CONFIG_HTTPD_WS_SUPPORT

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "moonws";
static const size_t MAX_FRAME = 3072;   // a request from the dongle is a few hundred bytes
enum { MAX_CLIENTS = 2 };

struct Client {
    int fd = -1;
    bool sub = false;
    long rev = -1;   // object-list revision this client has been sent (-1: none)
};
static Client s_cl[MAX_CLIENTS];
static httpd_handle_t s_srv;
static SemaphoreHandle_t s_lock;   // the client table and every send on a socket
static TaskHandle_t s_task;

static Client *find_client(int fd)
{
    for (auto &c : s_cl)
        if (c.fd == fd) return &c;
    return nullptr;
}

static void drop_client(int fd)
{
    if (Client *c = find_client(fd)) *c = Client();
    bool any = false;
    for (auto &c : s_cl) any |= (c.fd >= 0 && c.sub);
    if (!any) sf_moon_want(0);
}

// Caller holds s_lock. A failed send drops the client.
static void send_text(int fd, const std::string &s)
{
    httpd_ws_frame_t f = {};
    f.type = HTTPD_WS_TYPE_TEXT;
    f.payload = (uint8_t *)s.data();
    f.len = s.size();
    if (httpd_ws_send_frame_async(s_srv, fd, &f) != ESP_OK) {
        ESP_LOGW(TAG, "client %d gone", fd);
        drop_client(fd);
    }
}

// ---- the little JSON we need ----------------------------------------------------------------------
// Value after "key": in a flat JSON-RPC request. Strings are returned unescaped, other values raw.
static bool json_field(const std::string &j, const char *key, std::string &out, bool *is_str = nullptr)
{
    std::string k = std::string("\"") + key + "\"";
    size_t p = 0;
    while ((p = j.find(k, p)) != std::string::npos) {
        size_t q = p + k.size();
        while (q < j.size() && (j[q] == ' ' || j[q] == '\t')) q++;
        if (q >= j.size() || j[q] != ':') { p = q; continue; }
        q++;
        while (q < j.size() && (j[q] == ' ' || j[q] == '\t')) q++;
        if (q >= j.size()) return false;
        out.clear();
        if (j[q] == '"') {
            if (is_str) *is_str = true;
            for (q++; q < j.size() && j[q] != '"'; q++) {
                if (j[q] == '\\' && q + 1 < j.size()) {
                    q++;
                    switch (j[q]) {
                        case 'n': out += '\n'; break;
                        case 't': out += '\t'; break;
                        case 'u':   // keep ASCII only; the G-code we accept is ASCII
                            if (q + 4 < j.size()) { out += (char)strtol(j.substr(q + 1, 4).c_str(), nullptr, 16); q += 4; }
                            break;
                        default: out += j[q];
                    }
                } else out += j[q];
            }
            return true;
        }
        if (is_str) *is_str = false;
        while (q < j.size() && j[q] != ',' && j[q] != '}' && j[q] != ' ') out += j[q++];
        return true;
    }
    return false;
}

static std::string jstr(const std::string &s)
{
    std::string o = "\"";
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') { o += '\\'; o += (char)c; }
        else if (c < 0x20) { char b[8]; snprintf(b, sizeof b, "\\u%04x", c); o += b; }
        else o += (char)c;
    }
    return o + "\"";
}

static std::string reply_ok(const std::string &id, const std::string &result)
{
    return "{\"jsonrpc\":\"2.0\",\"result\":" + result + ",\"id\":" + id + "}";
}
static std::string reply_err(const std::string &id, int code, const std::string &msg)
{
    return "{\"jsonrpc\":\"2.0\",\"error\":{\"code\":" + std::to_string(code) + ",\"message\":" + jstr(msg) +
           "},\"id\":" + id + "}";
}

// One command line to the engine; ok -> true. The reply text goes to `text`.
static bool run_gcode(const std::string &line, std::string &text)
{
    char rep[256] = "";
    if (!sf_console(line.c_str(), rep, sizeof rep, 5000)) { text = "printer not ready"; return false; }
    text = rep;
    return strncmp(rep, "error", 5) != 0 && strncmp(rep, "!!", 2) != 0;
}

static void handle_request(int fd, const std::string &req)
{
    std::string method, id = "null", sc;
    bool id_str = false;
    json_field(req, "method", method);
    std::string raw;
    if (json_field(req, "id", raw, &id_str) && !raw.empty()) id = id_str ? jstr(raw) : raw;

    std::string out;
    if (method == "server.connection.identify") {
        out = reply_ok(id, "{\"connection_id\":1}");
    } else if (method == "printer.objects.list") {
        out = reply_ok(id, "{\"objects\":[\"print_stats\",\"toolhead\",\"motion_report\",\"pause_resume\","
                           "\"idle_timeout\",\"virtual_sdcard\",\"display_status\",\"extruder\",\"exclude_object\"]}");
    } else if (method == "printer.objects.subscribe") {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (Client *c = find_client(fd)) {
            c->sub = true;
            c->rev = -1;
            sf_moon_want(1);
        }
        xSemaphoreGive(s_lock);
        char *st = sf_moon_status_dup(-1);   // NULL until the link task has built the first one (<= 200 ms)
        std::string r = st ? st : "{\"eventtime\":0,\"status\":{}}";
        free(st);
        // the reply wants {"eventtime","status"}; ours also carries "rev", which is harmless
        out = reply_ok(id, r);
    } else if (method == "printer.gcode.script") {
        if (!json_field(req, "script", sc) || sc.empty()) {
            out = reply_err(id, -32602, "script missing");
        } else {
            std::string text;
            bool ok = run_gcode(sc, text);
            out = ok ? reply_ok(id, "\"ok\"") : reply_err(id, 400, text);
        }
    } else if (method == "printer.emergency_stop") {
        std::string text;
        bool ok = run_gcode("M112", text);
        out = ok ? reply_ok(id, "\"ok\"") : reply_err(id, 400, text);
    } else {
        out = reply_err(id, -32601, "Method not found");
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    send_text(fd, out);
    xSemaphoreGive(s_lock);
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    int fd = httpd_req_to_sockfd(req);
    if (req->method == HTTP_GET) {   // the upgrade handshake is done
        xSemaphoreTake(s_lock, portMAX_DELAY);
        Client *c = find_client(fd);
        if (!c) {
            for (auto &x : s_cl)
                if (x.fd < 0) { c = &x; break; }
            if (c) { *c = Client(); c->fd = fd; }
        }
        xSemaphoreGive(s_lock);
        if (!c) ESP_LOGW(TAG, "no free websocket slot");
        return c ? ESP_OK : ESP_FAIL;
    }
    httpd_ws_frame_t f = {};
    f.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t e = httpd_ws_recv_frame(req, &f, 0);   // length only
    if (e != ESP_OK) goto gone;
    if (f.len > MAX_FRAME) {
        ESP_LOGW(TAG, "frame of %u bytes refused", (unsigned)f.len);
        goto gone;
    }
    {
        std::string buf(f.len, '\0');
        f.payload = (uint8_t *)&buf[0];
        if (f.len && httpd_ws_recv_frame(req, &f, f.len) != ESP_OK) goto gone;
        if (f.type == HTTPD_WS_TYPE_CLOSE) goto gone;
        if (f.type == HTTPD_WS_TYPE_TEXT) handle_request(fd, buf);
        return ESP_OK;
    }
gone:
    xSemaphoreTake(s_lock, portMAX_DELAY);
    drop_client(fd);
    xSemaphoreGive(s_lock);
    return ESP_FAIL;   // the server closes the socket
}

static void push_task(void *)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(200));
        bool any = false;
        xSemaphoreTake(s_lock, portMAX_DELAY);
        for (auto &c : s_cl) any |= (c.fd >= 0 && c.sub);
        xSemaphoreGive(s_lock);
        if (!any) continue;
        for (int i = 0; i < MAX_CLIENTS; i++) {
            xSemaphoreTake(s_lock, portMAX_DELAY);
            int fd = s_cl[i].fd;
            bool sub = s_cl[i].sub;
            long have = s_cl[i].rev;
            xSemaphoreGive(s_lock);
            if (fd < 0 || !sub) continue;
            char *js = sf_moon_status_dup(have);   // {"eventtime","rev","status"}
            if (!js) continue;
            std::string all(js);
            free(js);
            // notify_status_update params are [status, eventtime]: unpack our wrapper
            size_t a = all.find("\"eventtime\":"), r = all.find("\"rev\":"), s = all.find("\"status\":");
            if (a == std::string::npos || r == std::string::npos || s == std::string::npos) continue;
            std::string et = all.substr(a + 12, all.find(',', a) - (a + 12));
            long rev = strtol(all.c_str() + r + 6, nullptr, 10);
            std::string status = all.substr(s + 9, all.size() - (s + 9) - 1);
            std::string m = "{\"jsonrpc\":\"2.0\",\"method\":\"notify_status_update\",\"params\":[" + status + "," + et + "]}";
            xSemaphoreTake(s_lock, portMAX_DELAY);
            if (find_client(fd)) {
                send_text(fd, m);
                if (Client *c = find_client(fd)) c->rev = rev;
            }
            xSemaphoreGive(s_lock);
        }
    }
}

extern "C" void sf_moonraker_register(void *httpd)
{
    s_srv = (httpd_handle_t)httpd;
    s_lock = xSemaphoreCreateMutex();
    static const httpd_uri_t u = {.uri = "/websocket", .method = HTTP_GET, .handler = ws_handler,
                                  .user_ctx = NULL, .is_websocket = true, .handle_ws_control_frames = false,
                                  .supported_subprotocol = NULL};
    if (httpd_register_uri_handler(s_srv, &u) != ESP_OK) {
        ESP_LOGW(TAG, "could not register /websocket");
        return;
    }
    if (!s_task) xTaskCreate(push_task, "moonws", 4096, NULL, 4, &s_task);
}

#else
extern "C" void sf_moonraker_register(void *) {}
#endif
