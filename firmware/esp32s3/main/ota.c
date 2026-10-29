// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Firmware update over WiFi (CAPTAIN and SKIPPER builds: they have two app slots in flash).
//   GET  /update   a page to pick the new build (skipper-esp32s3-<tier>-app.bin)
//   POST /update   the raw app image; written to the other slot, checked, then the ESP32 restarts
// Only an app image of the SAME tier is accepted (the tiers differ in partition table and PSRAM
// set-up, which only a USB flash can change). Roll-back: the new build must run for 30 s; if it
// crashes or restarts before that, the bootloader goes back to the previous one.
#include <stdio.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "sf.h"

static const char *TAG = "update";

#if CONFIG_SKIPPER_OTA

static void keep_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(30000));
    if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
        ESP_LOGW(TAG, "new firmware has run for 30 s: kept (no roll-back)");
    }
    vTaskDelete(NULL);
}

void sf_ota_boot_check(void)
{
    esp_ota_img_states_t st;
    const esp_partition_t *run = esp_ota_get_running_partition();
    if (run && esp_ota_get_state_partition(run, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_LOGW(TAG, "running a new firmware from %s: it is kept once it has run for 30 s", run->label);
        xTaskCreate(keep_task, "ota_keep", 3072, NULL, 1, NULL);
    }
}

static const char UPDATE_HTML_HEAD[] =
    "<!doctype html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,"
    "initial-scale=1\"><title>Firmware update</title><style>body{background:#e9e6dc;color:#141414;"
    "font:16px/1.5 Georgia,serif;margin:0}.b{background:#141414;color:#e9e6dc;text-align:center;padding:10px;"
    "font-weight:700}main{max-width:640px;margin:0 auto;padding:16px}h1{font:700 2rem Impact,Arial Black,"
    "sans-serif;border-bottom:4px solid #141414}button,input{font:inherit;border:2px solid #141414;"
    "background:#f7f5ee;padding:4px 10px}button{background:#141414;color:#e9e6dc}.bar{height:14px;border:2px solid "
    "#141414;background:#f7f5ee}.bar div{height:100%;background:#141414;width:0}</style></head><body>" SF_NAV
    "<div class=\"b\">" SF_BANNER "</div><main><h1>Firmware update</h1>";

static esp_err_t update_get(httpd_req_t *req)
{
    const esp_app_desc_t *d = esp_app_get_description();
    const esp_partition_t *run = esp_ota_get_running_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    static char mid[1500];
    snprintf(mid, sizeof mid,
             "<p>Running: <b>%s</b> (%s, built %s %s) from slot %s. The update goes to slot %s; the running "
             "build stays as the fall-back.</p><p>Pick <code>skipper-esp32s3-%s-app.bin</code> from "
             "<code>firmware/prebuilt-esp32s3/</code> (the <b>-app</b> file, not the merged one). Only a %s "
             "build is accepted. Not during a print.</p>"
             "<p><input type=\"file\" id=\"f\" accept=\".bin\"> <button id=\"go\">Update</button></p>"
             "<div class=\"bar\"><div id=\"pb\"></div></div><p id=\"s\"></p>"
             "<p>After the restart the new build must run for 30 s to be kept; if it crashes before that, the "
             "ESP32 goes back to this one by itself.</p><p><a href=\"/about\">About this build</a> &middot; "
             "<a href=\"/status\">Status</a></p>",
             sf_tier_title(), d->version, d->date, d->time, run ? run->label : "?", next ? next->label : "?",
             sf_tier_id(), sf_tier_id());
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr_chunk(req, UPDATE_HTML_HEAD);
    httpd_resp_sendstr_chunk(req, mid);
    httpd_resp_sendstr_chunk(
        req, "<script>var $=function(i){return document.getElementById(i)};$('go').onclick=function(){"
             "var f=$('f').files[0];if(!f){$('s').textContent='choose the file first';return}"
             "var x=new XMLHttpRequest();x.open('POST','/update');x.upload.onprogress=function(e){"
             "if(e.lengthComputable)$('pb').style.width=(100*e.loaded/e.total).toFixed(1)+'%'};"
             "x.onload=function(){$('s').textContent=x.responseText;if(x.status==200)setTimeout(function(){"
             "location.href='/about'},15000)};x.onerror=function(){$('s').textContent='upload failed'};"
             "$('s').textContent='uploading '+Math.round(f.size/1024)+' KB...';x.send(f)};</script></main></body></html>");
    return httpd_resp_sendstr_chunk(req, NULL);
}

static void restart_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

static esp_err_t update_post(httpd_req_t *req)
{
    if (sf_printing()) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "a print is running: update afterwards");
    }
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    size_t len = req->content_len;
    if (!next || len < 4096 || len > next->size) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "not an app image of a size that fits");
    }
    char *buf = malloc(4096);
    if (!buf) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
    }
    size_t got = 0;
    int timeouts = 0;
    esp_ota_handle_t h = 0;
    bool begun = false;
    char why[160] = "";
    while (got < len) {
        int n = httpd_req_recv(req, buf, len - got < 4096 ? len - got : 4096);
        if (n == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts < 5) {
            continue;
        }
        if (n <= 0) {
            snprintf(why, sizeof why, "upload interrupted");
            break;
        }
        while (!begun && n < 512 && got + (size_t)n < len) {   // the whole header block first
            int m = httpd_req_recv(req, buf + n, 512 - (size_t)n);
            if (m == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts < 5) {
                continue;
            }
            if (m <= 0) {
                break;
            }
            n += m;
        }
        if (!begun) {
            // the first bytes hold the image header and its description: check before writing anything
            size_t off = sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t);
            const esp_app_desc_t *nd = (const esp_app_desc_t *)(buf + off);
            char want[40];
            snprintf(want, sizeof want, "-%s", sf_tier_id());
            size_t vl = strnlen(nd->version, sizeof nd->version), wl = strlen(want);
            if ((size_t)n < off + sizeof(esp_app_desc_t) || (uint8_t)buf[0] != ESP_IMAGE_HEADER_MAGIC ||
                nd->magic_word != ESP_APP_DESC_MAGIC_WORD) {
                snprintf(why, sizeof why, "not an ESP32 app image (use the -app.bin file, not the merged one)");
                break;
            }
            if (strncmp(nd->project_name, esp_app_get_description()->project_name, sizeof nd->project_name)) {
                snprintf(why, sizeof why, "not a Skipper Flipper build");
                break;
            }
            if (vl < wl || memcmp(nd->version + vl - wl, want, wl)) {
                snprintf(why, sizeof why, "that is a different tier (%.32s); this board runs %s: use the "
                         "Flasher over USB to change tier", nd->version, sf_tier_id());
                break;
            }
            if (esp_ota_begin(next, OTA_WITH_SEQUENTIAL_WRITES, &h) != ESP_OK) {
                snprintf(why, sizeof why, "could not prepare the update slot");
                break;
            }
            begun = true;
            ESP_LOGW(TAG, "updating to %.32s (%u KB) in slot %s", nd->version, (unsigned)(len / 1024), next->label);
        }
        if (esp_ota_write(h, buf, (size_t)n) != ESP_OK) {
            snprintf(why, sizeof why, "writing to flash failed");
            break;
        }
        got += (size_t)n;
    }
    free(buf);
    if (why[0]) {
        if (begun) {
            esp_ota_abort(h);
        }
        ESP_LOGW(TAG, "update refused: %s", why);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, why);
    }
    esp_err_t e = esp_ota_end(h);   // checks the image (and its SHA-256)
    if (e != ESP_OK || esp_ota_set_boot_partition(next) != ESP_OK) {
        ESP_LOGW(TAG, "update failed the image check: %s", esp_err_to_name(e));
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "the image failed its check (damaged file?)");
    }
    ESP_LOGW(TAG, "update written and checked; restarting into it");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "ok: written and checked. Restarting into the new build; this page goes to "
                            "About in 15 s (reconnect to the WiFi if needed).");
    xTaskCreate(restart_task, "restart", 2048, NULL, 1, NULL);
    return ESP_OK;
}

void sf_ota_register(void *httpd)
{
    static const httpd_uri_t g = {.uri = "/update", .method = HTTP_GET, .handler = update_get};
    static const httpd_uri_t p = {.uri = "/update", .method = HTTP_POST, .handler = update_post};
    httpd_register_uri_handler((httpd_handle_t)httpd, &g);
    httpd_register_uri_handler((httpd_handle_t)httpd, &p);
}

#else   // DECKHAND: no second app slot

static esp_err_t update_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_sendstr(req, "<!doctype html><meta name=\"viewport\" content=\"width=device-width\">"
                                   "<body style=\"font:16px Georgia,serif;background:#e9e6dc;padding:16px\">"
                                   "<h1>Firmware update</h1><p>This is the DECKHAND build: it fits boards with 2 MB "
                                   "of flash, which leaves no room for a second copy of the firmware, so updates go "
                                   "over USB with the Flasher. <a href=\"/about\">Can this board run CAPTAIN?</a></p>");
}

void sf_ota_boot_check(void)
{
}

void sf_ota_register(void *httpd)
{
    static const httpd_uri_t g = {.uri = "/update", .method = HTTP_GET, .handler = update_get};
    httpd_register_uri_handler((httpd_handle_t)httpd, &g);
    (void)TAG;
}

#endif
