// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// "Send to printer" from the slicer. PrusaSlicer and OrcaSlicer can upload to an OctoPrint
// server; this answers the two requests they use, so the ESP32-S3 can be added there as an
// OctoPrint "physical printer":
//   GET  /api/version          the slicer's "Test" button
//   POST /api/files/local      multipart/form-data: file=<G-code>, print=true|false, path=
// The file goes to the SD card (streamed: any size the card has room for); print=true starts it.
// The API key the slicer asks for is not checked: anyone on the ESP32's WiFi can already use
// the printer from the web pages (see SECURITY.md).
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unistd.h>

#include "esp_http_server.h"
#include "esp_log.h"
#include "ph/multipart.h"
#include "sf.h"

static const char *TAG = "slicer";

// for extras.c (C): the preview picture inside a G-code file
extern "C" bool sf_find_thumbnail(const char *head, size_t n, int max_px, char *fmt, size_t fmt_n, char **b64,
                                  size_t *b64_n)
{
    ph::GcodeThumbnail t;
    if (!ph::find_gcode_thumbnail(std::string(head, n), max_px, t)) {
        return false;
    }
    char *p = (char *)malloc(t.base64.size() + 1);
    if (!p) {
        return false;
    }
    memcpy(p, t.base64.c_str(), t.base64.size() + 1);
    snprintf(fmt, fmt_n, "%s", t.format.c_str());
    *b64 = p;
    *b64_n = t.base64.size();
    return true;
}

static esp_err_t version_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    // PrusaSlicer checks that "text" starts with "OctoPrint"
    return httpd_resp_sendstr(req, "{\"api\":\"0.1\",\"server\":\"1.9.0\",\"text\":\"OctoPrint 1.9.0 "
                                   "(Skipper Flipper " SF_VERSION ", OctoPrint-compatible upload only)\"}");
}

static esp_err_t send_json_err(httpd_req_t *req, const char *status, const char *msg)
{
    char b[300];
    snprintf(b, sizeof b, "{\"error\":\"%s\"}", msg);
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    ESP_LOGW(TAG, "slicer upload refused: %s", msg);
    return httpd_resp_sendstr(req, b);
}

static void clean_name(const std::string &in, char *out, size_t n)
{
    // last path component, plain characters, spaces -> '_' (typed commands take no spaces)
    std::string s = in.substr(in.find_last_of("/\\") == std::string::npos ? 0 : in.find_last_of("/\\") + 1);
    size_t k = 0;
    for (char c : s) {
        if (k + 1 >= n) {
            break;
        }
        out[k++] = (isalnum((unsigned char)c) || strchr("._-", c)) ? c : '_';
    }
    out[k] = 0;
    if (!sf_sd_name_ok(out) && k + 7 < n) {
        strcat(out, ".gcode");
    }
}

static esp_err_t files_local_post(httpd_req_t *req)
{
    if (sf_printing()) {
        return send_json_err(req, "409 Conflict", "a print is running: wait for it to finish or cancel it");
    }
    if (!sf_sd_mounted()) {
        return send_json_err(req, "409 Conflict", "no SD card in the ESP32 (slicer uploads go to the card)");
    }
    char ct[200];
    std::string boundary;
    if (httpd_req_get_hdr_value_str(req, "Content-Type", ct, sizeof ct) != ESP_OK ||
        !ph::Multipart::boundary_from_content_type(ct, boundary)) {
        return send_json_err(req, "400 Bad Request", "expected multipart/form-data");
    }
    uint64_t total = 0, freeb = 0;
    sf_sd_space(&total, &freeb);
    if ((uint64_t)req->content_len + 65536 > freeb) {
        return send_json_err(req, "507 Insufficient Storage", "not enough room on the SD card");
    }

    struct Ctx {
        FILE *f = nullptr;
        char name[120] = "";
        char path[300] = "";
        char part[310] = "";
        std::string field, field_name;
        bool in_file = false, have_file = false, print = false, failed = false;
    } c;
    ph::Multipart mp(boundary);
    mp.on_part = [&](const ph::Multipart::Part &p) {
        c.field.clear();
        c.field_name = p.name;
        c.in_file = !p.filename.empty();
        if (!c.in_file) {
            return true;
        }
        if (c.have_file) {
            return false;   // one file per upload
        }
        clean_name(p.filename, c.name, sizeof c.name);
        if (!sf_sd_path(c.name, c.path, sizeof c.path)) {
            return false;
        }
        snprintf(c.part, sizeof c.part, "%s.part", c.path);
        c.f = fopen(c.part, "wb");
        return c.f != nullptr;
    };
    mp.on_data = [&](const char *d, size_t n) {
        if (c.in_file) {
            return fwrite(d, 1, n, c.f) == n;
        }
        if (c.field.size() + n <= 64) {
            c.field.append(d, n);
        }
        return true;
    };
    mp.on_part_end = [&]() {
        if (c.in_file) {
            bool ok = fclose(c.f) == 0;
            c.f = nullptr;
            c.have_file = ok;
            return ok;
        }
        if (c.field_name == "print") {
            c.print = c.field == "true" || c.field == "1";
        }
        return true;
    };

    char *buf = (char *)malloc(4096);   // from the heap: DECKHAND needs its internal RAM free
    if (!buf) {
        if (c.f) {
            fclose(c.f);
        }
        return send_json_err(req, "500 Internal Server Error", "out of memory");
    }
    size_t left = req->content_len;
    int timeouts = 0;
    bool ok = true;
    while (left > 0 && ok) {
        int n = httpd_req_recv(req, buf, left < 4096 ? left : 4096);
        if (n == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts < 5) {
            continue;
        }
        if (n <= 0) {
            ok = false;
            break;
        }
        left -= (size_t)n;
        ok = mp.feed(buf, (size_t)n);
    }
    free(buf);
    if (c.f) {
        fclose(c.f);
    }
    if (!ok || !mp.done() || !c.have_file) {
        if (c.part[0]) {
            unlink(c.part);
        }
        return send_json_err(req, "400 Bad Request",
                             !ok && mp.error() == "stopped" ? "could not write the file to the SD card (name or space)"
                                                             : "upload incomplete or not a G-code file upload");
    }
    unlink(c.path);   // FAT cannot rename over an existing file
    if (rename(c.part, c.path) != 0) {
        unlink(c.part);
        return send_json_err(req, "500 Internal Server Error", "could not rename the finished file");
    }
    ESP_LOGW(TAG, "received %s from the slicer (%u KB)%s", c.name, (unsigned)(req->content_len / 1024),
             c.print ? ", printing it" : "");
    std::string started;
    if (c.print) {
        static char reply[300];
        std::string cmd = std::string("SDCARD_PRINT_FILE FILENAME=") + c.name;
        sf_console(cmd.c_str(), reply, sizeof reply, 3000);
        started = reply;
        for (char &ch : started) {   // keep the reply valid JSON
            if (ch == '"' || ch == '\\' || (unsigned char)ch < 0x20) {
                ch = ' ';
            }
        }
    }
    static char out[600];
    snprintf(out, sizeof out,
             "{\"done\":true,\"files\":{\"local\":{\"name\":\"%s\",\"origin\":\"local\",\"refs\":{\"resource\":"
             "\"/api/files/local/%s\"}}},\"print\":\"%s\"}",
             c.name, c.name, started.substr(0, 200).c_str());
    char loc[200];
    snprintf(loc, sizeof loc, "/api/files/local/%s", c.name);
    httpd_resp_set_status(req, "201 Created");
    httpd_resp_set_hdr(req, "Location", loc);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, out);
}

extern "C" void sf_slicer_api_register(void *httpd)
{
    httpd_handle_t s = (httpd_handle_t)httpd;
    static const httpd_uri_t v = {.uri = "/api/version", .method = HTTP_GET, .handler = version_get,
                                  .user_ctx = nullptr};
    static const httpd_uri_t f = {.uri = "/api/files/local", .method = HTTP_POST, .handler = files_local_post,
                                  .user_ctx = nullptr};
    httpd_register_uri_handler(s, &v);
    httpd_register_uri_handler(s, &f);
}
