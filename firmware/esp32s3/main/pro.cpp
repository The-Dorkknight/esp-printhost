// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// The CAPTAIN / SKIPPER extras that need the ESP32 itself (2026-10-24):
//   GET /bench  the motion benchmark (ph::motion_bench_report) in a low-priority task, so the
//               printer link keeps running; the page polls until the numbers are there.
// With CONFIG_SKIPPER_BENCH_AT_BOOT (QEMU timing builds only) the benchmark runs at start-up.
#include <string.h>

#include <string>

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ph/bench.h"
#include "sdkconfig.h"
#include "sf.h"

#if CONFIG_SKIPPER_PRO || CONFIG_SKIPPER_BENCH_AT_BOOT
static const char *TAG = "pro";

static uint64_t now_us(void)
{
    return (uint64_t)esp_timer_get_time();
}
#endif

#if CONFIG_SKIPPER_BENCH_AT_BOOT
static volatile bool s_boot_bench_done;
static void boot_bench_task(void *arg);
#endif

extern "C" void sf_bench_boot(const sf_settings_t *s)
{
#if CONFIG_SKIPPER_BENCH_AT_BOOT
    sf_accel_scan_now(s);
    // its own task: app_main's 3.5 KB stack is too small for the planner (a stack canary panic, QEMU)
    xTaskCreate(boot_bench_task, "bench", 8192, NULL, 5, NULL);
    while (!s_boot_bench_done) vTaskDelay(pdMS_TO_TICKS(100));
#endif
}

#if CONFIG_SKIPPER_BENCH_AT_BOOT
static void boot_bench_task(void *arg)
{
    // one line at a time: QEMU's instruction-counted runs are slow, so each result shows as it lands
    ESP_LOGW(TAG, "motion benchmark (start-up run, short path):");
    ESP_LOGW(TAG, "BENCH %s", ph::homing_bench_legacy(now_us).text("homing move, compressor before 2026-10-24").c_str());
    ESP_LOGW(TAG, "BENCH %s", ph::homing_bench(now_us).text("homing move, 2026-10-24 compressor").c_str());
    ESP_LOGW(TAG, "BENCH %s", ph::motion_bench(150, nullptr, now_us, true).text("print path 150 mm/s, plain").c_str());
    ph::ShapingConfig c;
    std::vector<double> A, T;
    ph::shaper_def("mzv", 45, 0.1, A, T);
    c.x = ph::shaper_pulses(A, T);
    ph::shaper_def("ei", 38, 0.1, A, T);
    c.y = ph::shaper_pulses(A, T);
    c.pa = 0.05;
    c.pa_on = true;
    ESP_LOGW(TAG, "BENCH %s", ph::motion_bench(150, &c, now_us, true).text("print path 150 mm/s, shaping + PA").c_str());
    ESP_LOGW(TAG, "BENCH done");
    s_boot_bench_done = true;
    vTaskDelete(NULL);
}
#endif

#if CONFIG_SKIPPER_PRO
static std::string *s_bench;   // result text ("" while running)
static volatile bool s_bench_running;

static void bench_task(void *arg)
{
    std::string r = ph::motion_bench_report(now_us);
    ESP_LOGW(TAG, "motion benchmark:\n%s", r.c_str());
    // each line into the important-events log too (2026-10-29): the first says which maths this build uses
    for (size_t b = 0, e; b < r.size(); b = e + 1) {
        e = r.find('\n', b);
        if (e == std::string::npos) e = r.size();
        if (e > b) sf_report("motion benchmark: %s", r.substr(b, e - b).c_str());
    }
    *s_bench = r + "(Timed while the printer link and WiFi keep running, so a little high.)\n";
    s_bench_running = false;
    vTaskDelete(NULL);
}

static esp_err_t bench_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    if (!s_bench) s_bench = new std::string();
    char q[16] = "";
    bool start = httpd_req_get_url_query_str(req, q, sizeof q) == ESP_OK && strstr(q, "run=1");
    if (start && !s_bench_running) {
        if (sf_printing()) return httpd_resp_sendstr(req, "Not while printing.\n");
        s_bench->clear();
        s_bench_running = true;
        if (xTaskCreate(bench_task, "bench", 8192, NULL, 1, NULL) != pdPASS) {
            s_bench_running = false;
            return httpd_resp_sendstr(req, "Not enough memory to start the benchmark.\n");
        }
    }
    if (s_bench_running) return httpd_resp_sendstr(req, "running (about 10-30 s)...\n");
    if (s_bench->empty()) return httpd_resp_sendstr(req, "Not run yet: /bench?run=1\n");
    return httpd_resp_sendstr(req, s_bench->c_str());
}
#endif

extern "C" void sf_pro_register(void *httpd)
{
#if CONFIG_SKIPPER_PRO
    static const httpd_uri_t b = {.uri = "/bench", .method = HTTP_GET, .handler = bench_get, .user_ctx = NULL};
    httpd_register_uri_handler((httpd_handle_t)httpd, &b);
#else
    (void)httpd;
#endif
}
