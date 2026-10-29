// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// A Klipper USB accelerometer (RP2040 + ADXL345: BTT ADXL345 V2, KUSBA, a Pico wired as in
// Klipper's docs) on the ESP32-S3's own USB-C port, used as USB host (2026-10-24; CAPTAIN /
// SKIPPER). COMPILE-ONLY: never run, not even in QEMU (it has no USB).
//
//   - The port stops being the USB log / flashing port until the next restart (the S3 has one USB
//     PHY). The web pages, the log page and WiFi updates keep working. Flashing over USB still works
//     after holding BOOT while plugging in.
//   - It needs an OTG adapter (USB-C plug to USB-A socket) and 5 V on the port: on the ESP32-S3-Zero
//     the port's 5 V comes from the board's 5 V pin, so it works when the ESP32 is powered from the
//     printer board's 5 V (not from a computer).
//   - The board runs Klipper v0.12 or newer (older ones are told to update); we talk to it with the
//     same ph::KLink as to the printer board: identify, then the usual RP2040 SPI pin groups (RX, CSn,
//     SCK, TX = GPIO 0-3, 8-11, 12-15, 4-7, 16-19) one at a time until an ADXL345 DEVID reads 0xE5
//     (KUSBA and Klipper's Pico wiring: 0-3; BTT ADXL345 V2: 8-11; Mellow FLY-ADXL345: 12-15); then
//     the board is reset and configured with only that group (2026-10-24, after a review).
//   - Samples arrive as Klipper's sensor_bulk_data, 5 bytes per sample (adxl345.py _convert_samples).
#include <string.h>

#include <string>
#include <vector>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "ph/kboard.h"
#include "ph/klink.h"
#include "sdkconfig.h"
#include "sf.h"
#include "usb/usb_host.h"

bool inflate_dict(const ph::Bytes &in, std::string &out);   // link.cpp

#if CONFIG_SKIPPER_PRO

static const char *TAG = "usbacc";

struct UsbDev {
    usb_host_client_handle_t client = nullptr;
    usb_device_handle_t dev = nullptr;
    uint8_t addr = 0, iface = 0xff, ep_in = 0, ep_out = 0;
    uint16_t mps_in = 64, mps_out = 64;
    usb_transfer_t *in = nullptr, *out = nullptr;
    StreamBufferHandle_t rx = nullptr;
    SemaphoreHandle_t out_done = nullptr;
    volatile bool gone = false;
};
static UsbDev s_u;
static bool s_host_up;
static TaskHandle_t s_lib_task, s_client_task;

static void lib_task(void *arg)
{
    for (;;) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
    }
}

static void client_cb(const usb_host_client_event_msg_t *msg, void *arg)
{
    if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) s_u.addr = msg->new_dev.address;
    else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        s_u.addr = 0;
        s_u.gone = true;
    }
}

static void client_task(void *arg)
{
    for (;;) usb_host_client_handle_events(s_u.client, portMAX_DELAY);
}

static void in_cb(usb_transfer_t *t)
{
    if (t->status == USB_TRANSFER_STATUS_COMPLETED && t->actual_num_bytes > 0)
        xStreamBufferSend(s_u.rx, t->data_buffer, (size_t)t->actual_num_bytes, 0);
    if (!s_u.gone && t->status != USB_TRANSFER_STATUS_NO_DEVICE) usb_host_transfer_submit(t);
}

static void out_cb(usb_transfer_t *t)
{
    xSemaphoreGive(s_u.out_done);
}

static bool host_start(std::string &why)
{
    if (s_host_up) return true;
    usb_host_config_t hc = {};
    hc.intr_flags = 0;
    if (usb_host_install(&hc) != ESP_OK) {   // takes the USB PHY from the USB serial/JTAG log
        why = "USB host would not start";
        return false;
    }
    xTaskCreate(lib_task, "usblib", 4096, NULL, 6, &s_lib_task);
    usb_host_client_config_t cc = {};
    cc.is_synchronous = false;
    cc.max_num_event_msg = 5;
    cc.async.client_event_callback = client_cb;
    cc.async.callback_arg = NULL;
    if (usb_host_client_register(&cc, &s_u.client) != ESP_OK) {
        why = "USB host client failed";
        return false;
    }
    xTaskCreate(client_task, "usbcli", 4096, NULL, 5, &s_client_task);
    s_u.rx = xStreamBufferCreate(4096, 1);
    s_u.out_done = xSemaphoreCreateBinary();
    s_host_up = true;
    ESP_LOGW(TAG, "USB-C port is now a USB host: the USB log stops until the next restart");
    return true;
}

// Open the device, find the CDC data interface (two bulk endpoints), claim it, start reading.
static bool open_cdc(std::string &why)
{
    for (int i = 0; i < 40 && !s_u.addr; i++) vTaskDelay(pdMS_TO_TICKS(100));   // up to 4 s for it to enumerate
    if (!s_u.addr) {
        why = "nothing plugged into the USB-C port (OTG adapter? 5 V on the port?)";
        return false;
    }
    if (usb_host_device_open(s_u.client, s_u.addr, &s_u.dev) != ESP_OK) {
        why = "could not open the USB device";
        return false;
    }
    const usb_device_desc_t *dd = nullptr;
    usb_host_get_device_descriptor(s_u.dev, &dd);
    char b[96];
    snprintf(b, sizeof b, "USB device %04x:%04x", dd ? dd->idVendor : 0, dd ? dd->idProduct : 0);
    if (!dd || dd->idVendor != 0x1d50 || dd->idProduct != 0x614e) {   // Klipper's USB ids
        why = std::string(b) + " is not a Klipper board";
        return false;
    }
    const usb_config_desc_t *cd = nullptr;
    if (usb_host_get_active_config_descriptor(s_u.dev, &cd) != ESP_OK || !cd) {
        why = "no USB configuration";
        return false;
    }
    int off = 0;
    const usb_standard_desc_t *d = (const usb_standard_desc_t *)cd;
    uint8_t cur_if = 0xff, cur_class = 0;
    while ((d = usb_parse_next_descriptor(d, cd->wTotalLength, &off)) != nullptr) {
        if (d->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
            const usb_intf_desc_t *id = (const usb_intf_desc_t *)d;
            cur_if = id->bInterfaceNumber;
            cur_class = id->bInterfaceClass;
        } else if (d->bDescriptorType == USB_B_DESCRIPTOR_TYPE_ENDPOINT && cur_class == 0x0A) {
            const usb_ep_desc_t *ep = (const usb_ep_desc_t *)d;
            if ((ep->bmAttributes & 3) != 2) continue;   // bulk only
            s_u.iface = cur_if;
            if (ep->bEndpointAddress & 0x80) s_u.ep_in = ep->bEndpointAddress, s_u.mps_in = ep->wMaxPacketSize;
            else s_u.ep_out = ep->bEndpointAddress, s_u.mps_out = ep->wMaxPacketSize;
        }
    }
    if (s_u.iface == 0xff || !s_u.ep_in || !s_u.ep_out) {
        why = "no CDC data interface on the Klipper board";
        return false;
    }
    if (usb_host_interface_claim(s_u.client, s_u.dev, s_u.iface, 0) != ESP_OK) {
        why = "could not claim the USB interface";
        return false;
    }
    usb_host_transfer_alloc(s_u.mps_in, 0, &s_u.in);
    usb_host_transfer_alloc(64, 0, &s_u.out);
    s_u.in->device_handle = s_u.dev;
    s_u.in->bEndpointAddress = s_u.ep_in;
    s_u.in->num_bytes = s_u.mps_in;
    s_u.in->callback = in_cb;
    s_u.out->device_handle = s_u.dev;
    s_u.out->bEndpointAddress = s_u.ep_out;
    s_u.out->callback = out_cb;
    usb_host_transfer_submit(s_u.in);
    ESP_LOGW(TAG, "%s: Klipper board, CDC interface %u", b, s_u.iface);
    return true;
}

static ph::KLinkIO usb_io(void)
{
    ph::KLinkIO io;
    io.write = [](const uint8_t *p, size_t n) {
        while (n) {
            size_t k = n > 64 ? 64 : n;
            memcpy(s_u.out->data_buffer, p, k);
            s_u.out->num_bytes = (int)k;
            if (usb_host_transfer_submit(s_u.out) != ESP_OK) return false;
            if (xSemaphoreTake(s_u.out_done, pdMS_TO_TICKS(200)) != pdTRUE) return false;
            p += k, n -= k;
        }
        return true;
    };
    io.read = [](uint8_t *p, size_t n, int ms) -> size_t {
        return xStreamBufferReceive(s_u.rx, p, n, pdMS_TO_TICKS(ms > 0 ? ms : 1));
    };
    io.now_ms = []() { return (uint32_t)(esp_timer_get_time() / 1000); };
    io.now_us = []() { return (uint64_t)esp_timer_get_time(); };
    io.inflate = inflate_dict;
    io.log = [](const std::string &s) { ESP_LOGW(TAG, "%s", s.c_str()); };
    return io;
}

// ---- the Klipper side

static ph::KLink *s_link;
static int s_adxl_oid = -1, s_spi_oid = -1;
static double s_clock = 12e6;
static SemaphoreHandle_t s_buf_lock;
static std::vector<float> s_samples;
static int s_ovf;   // under s_buf_lock
static TaskHandle_t s_poll_task;
static volatile bool s_running;

// The usual RP2040 SPI pin groups (RX, CSn, SCK, TX), most common first: KUSBA and Klipper's Pico
// wiring 0-3, BTT ADXL345 V2 8-11, Mellow FLY-ADXL345 12-15, then 4-7 and 16-19. Klipper's names
// changed in 2024 ("spi0a" -> "spi0_gpio0_gpio3_gpio2"); v0.13 still knows both, so the name is
// taken from the board's own dictionary.
static const char *const BUS_NEW[5] = {"spi0_gpio0_gpio3_gpio2", "spi1_gpio8_gpio11_gpio10", "spi1_gpio12_gpio15_gpio14",
                                       "spi0_gpio4_gpio7_gpio6", "spi0_gpio16_gpio19_gpio18"};
static const char *const BUS_OLD[5] = {"spi0a", "spi1a", "spi1b", "spi0b", "spi0c"};
static const int CS[5] = {1, 9, 13, 5, 17};
static std::string s_bus;

static std::string bus_name(const ph::MsgProto &mp, int k)
{
    int64_t v;
    if (mp.enum_value("spi_bus", BUS_NEW[k], v)) return BUS_NEW[k];
    if (mp.enum_value("spi_bus", BUS_OLD[k], v)) return BUS_OLD[k];
    return "";
}

// The configuration for "ADXL345 on group k" (also what the CRC is worked out from).
static std::vector<std::string> config_lines(const ph::MsgProto &mp, int k)
{
    return {"config_spi oid=0 pin=gpio" + std::to_string(CS[k]) + " cs_active_high=0",
            "spi_set_bus oid=0 spi_bus=" + bus_name(mp, k) + " mode=3 rate=5000000",
            "config_adxl345 oid=1 spi_oid=0"};
}

static uint32_t config_crc(const std::vector<std::string> &cfg)
{
    std::string joined;
    for (auto &l : cfg) joined += l + "\n";
    return ph::crc32_zlib(joined);
}

static bool devid_ok(int oid)
{
    ph::Message m;
    if (!s_link->query("spi_transfer oid=" + std::to_string(oid) + " data=hex:8000", "spi_transfer_response", oid, m, 300))
        return false;
    const ph::Bytes &r = m.params["response"].b;
    return r.size() >= 2 && r[1] == 0xE5;
}

// After "reset" the RP2040 drops off USB and comes back as a new device: let go of the old one,
// open the new one and start a fresh link (new sequence numbers).
static bool reopen_after_reset(std::string &why)
{
    for (int i = 0; i < 30 && !s_u.gone; i++) vTaskDelay(pdMS_TO_TICKS(100));
    if (!s_u.gone) return why = "the accelerometer board did not restart when asked", false;
    vTaskDelay(pdMS_TO_TICKS(50));   // the IN transfer finishes with NO_DEVICE and is not sent again
    usb_host_interface_release(s_u.client, s_u.dev, s_u.iface);
    usb_host_transfer_free(s_u.in);
    usb_host_transfer_free(s_u.out);
    usb_host_device_close(s_u.client, s_u.dev);
    s_u.dev = nullptr, s_u.in = s_u.out = nullptr;
    s_u.iface = 0xff, s_u.ep_in = s_u.ep_out = 0;
    xStreamBufferReset(s_u.rx);
    xSemaphoreTake(s_u.out_done, 0);
    s_u.gone = false;
    if (!open_cdc(why)) return false;
    delete s_link;
    s_link = new ph::KLink(usb_io());
    return true;
}

static bool configure(std::string &why)
{
    ph::KInfo info;
    if (!s_link->identify(info, &why, 5)) return false;
    s_clock = info.clock_freq > 0 ? info.clock_freq : 12e6;
    const ph::MsgProto &mp = s_link->proto();
    for (auto *c : {"config_spi", "spi_set_bus", "spi_transfer", "config_adxl345", "query_adxl345"})
        if (!mp.has_command(c)) {
            why = std::string("the accelerometer's Klipper has no ") + c +
                  ": flash it with the Flasher's generic-rp2040 firmware (Klipper v0.13.0, USB)";
            return false;
        }
    const ph::MsgFormat *q = mp.command("query_adxl345");
    if (!mp.response("sensor_bulk_data") || (q && q->params.size() != 2)) {
        why = "the accelerometer runs Klipper older than v0.12 (2024): flash it with the Flasher's generic-rp2040 "
              "firmware (Klipper v0.13.0, USB; hold BOOT while plugging it into the computer)";
        return false;
    }
    bool configured = false, shutdown = false;
    uint32_t have_crc = 0;
    if (!s_link->get_config(configured, shutdown, &why, &have_crc)) return false;
    if (configured) {
        // Set up by us before (it keeps that until unplugged): find which group from the CRC.
        for (int k = 0; k < 5; k++)
            if (!bus_name(mp, k).empty() && config_crc(config_lines(mp, k)) == have_crc && devid_ok(0)) {
                s_bus = bus_name(mp, k) + ", CS gpio" + std::to_string(CS[k]);
                s_spi_oid = 0, s_adxl_oid = 1;
                return true;
            }
        why = "the accelerometer board is set up for something else: unplug it and plug it in again";
        return false;
    }
    // One group at a time, each with its own SPI oid, before finalize_config (Klipper runs config
    // commands as they arrive), so pins of groups after the one that answers are never touched.
    // Then the board is reset to a clean configuration with only that group.
    std::string err;
    if (!s_link->command("allocate_oids count=5", nullptr, 300, &err)) return why = err, false;
    int found = -1;
    for (int k = 0; k < 5 && found < 0; k++) {
        std::string bus = bus_name(mp, k);
        if (bus.empty()) continue;
        if (!s_link->command("config_spi oid=" + std::to_string(k) + " pin=gpio" + std::to_string(CS[k]) + " cs_active_high=0",
                             nullptr, 300, &err) ||
            !s_link->command("spi_set_bus oid=" + std::to_string(k) + " spi_bus=" + bus + " mode=3 rate=5000000", nullptr, 300, &err))
            return why = err, false;
        if (devid_ok(k)) found = k;
    }
    if (found < 0) {
        why = "no ADXL345 answered on the usual RP2040 SPI pins (0-3, 8-11, 12-15, 4-7, 16-19); unplug it and plug it in "
              "again before the next try";
        return false;
    }
    // Restart the accelerometer board (it comes back on USB) and configure only the group found.
    s_link->command("reset", nullptr, 100, nullptr);
    if (!reopen_after_reset(why)) return false;
    if (!s_link->identify(info, &why, 5)) return false;
    auto cfg = config_lines(s_link->proto(), found);
    if (!s_link->command("allocate_oids count=2", nullptr, 300, &err)) return why = err, false;
    for (auto &l : cfg)
        if (!s_link->command(l, nullptr, 300, &err)) return why = l + ": " + err, false;
    if (!s_link->command("finalize_config crc=" + std::to_string(config_crc(cfg)), nullptr, 300, &err)) return why = err, false;
    if (!devid_ok(0)) return why = "the ADXL345 stopped answering after the restart", false;
    s_bus = bus_name(s_link->proto(), found) + ", CS gpio" + std::to_string(CS[found]);
    s_spi_oid = 0, s_adxl_oid = 1;
    ESP_LOGW(TAG, "ADXL345 found on %s", s_bus.c_str());
    return true;
}

static void on_msg(const ph::Message &m)
{
    if (m.name != "sensor_bulk_data" || int(m.get("oid")) != s_adxl_oid) return;
    auto it = m.params.find("data");
    if (it == m.params.end()) return;
    const ph::Bytes &d = it->second.b;
    const float G = 9806.65f;
    xSemaphoreTake(s_buf_lock, portMAX_DELAY);
    for (size_t i = 0; i + 5 <= d.size(); i += 5) {   // adxl345.py _convert_samples
        int xl = d[i], yl = d[i + 1], zl = d[i + 2], xzh = d[i + 3], yzh = d[i + 4];
        if (yzh & 0x80) {
            s_ovf++;
            continue;
        }
        int rx = (xl | ((xzh & 0x1f) << 8)) - ((xzh & 0x10) << 9);
        int ry = (yl | ((yzh & 0x1f) << 8)) - ((yzh & 0x10) << 9);
        int rz = (zl | ((xzh & 0xe0) << 3) | ((yzh & 0xe0) << 6)) - ((yzh & 0x40) << 7);
        if (s_samples.size() < 3 * 4096) {
            s_samples.push_back(rx * 0.003774f * G);
            s_samples.push_back(ry * 0.003774f * G);
            s_samples.push_back(rz * 0.003906f * G);
        } else {
            s_ovf++;
        }
    }
    xSemaphoreGive(s_buf_lock);
}

static void poll_task(void *arg)
{
    while (s_running) s_link->poll(20);
    s_poll_task = nullptr;
    vTaskDelete(NULL);
}

bool sf_usb_accel_find(std::string &desc)
{
    if (!host_start(desc)) return false;
    if (!s_u.dev && !open_cdc(desc)) return false;
    if (!s_link) s_link = new ph::KLink(usb_io());
    if (s_adxl_oid < 0 && !configure(desc)) return false;
    desc = "Klipper USB accelerometer: ADXL345 on " + s_bus;
    return true;
}

bool sf_usb_accel_start(double &rate, std::string *err)
{
    std::string why;
    if (!sf_usb_accel_find(why)) {
        if (err) *err = why;
        return false;
    }
    if (!s_buf_lock) s_buf_lock = xSemaphoreCreateMutex();
    s_samples.clear();
    s_ovf = 0;
    s_link->set_message_handler(on_msg);
    std::string S = std::to_string(s_spi_oid), A = std::to_string(s_adxl_oid);
    // adxl345.py _start_measurements, at 1600 Hz
    for (const char *w : {"2d00", "310b", "3800", "2c0e", "3890"})
        s_link->command("spi_send oid=" + S + " data=hex:" + w, nullptr, 300, nullptr);
    uint32_t rest = (uint32_t)(s_clock * 4.0 / 1600.0);
    s_link->command("query_adxl345 oid=" + A + " rest_ticks=" + std::to_string(rest), nullptr, 300, nullptr);
    s_link->command("spi_send oid=" + S + " data=hex:2d08", nullptr, 300, nullptr);
    s_running = true;
    xTaskCreate(poll_task, "usbpoll", 6144, NULL, 4, &s_poll_task);
    rate = 1600;
    return true;
}

int sf_usb_accel_read(float *xyz, int max, int *overflow)
{
    if (!s_buf_lock) return -1;
    xSemaphoreTake(s_buf_lock, portMAX_DELAY);
    int n = (int)(s_samples.size() / 3);
    if (n > max) n = max;
    memcpy(xyz, s_samples.data(), sizeof(float) * 3 * (size_t)n);
    s_samples.erase(s_samples.begin(), s_samples.begin() + 3 * n);
    *overflow += s_ovf;
    s_ovf = 0;
    xSemaphoreGive(s_buf_lock);
    return n;
}

void sf_usb_accel_stop(void)
{
    s_running = false;
    for (int i = 0; i < 20 && s_poll_task; i++) vTaskDelay(pdMS_TO_TICKS(20));
    if (!s_link || s_adxl_oid < 0) return;
    s_link->set_message_handler(nullptr);
    s_link->command("spi_send oid=" + std::to_string(s_spi_oid) + " data=hex:2d00", nullptr, 300, nullptr);
    s_link->command("query_adxl345 oid=" + std::to_string(s_adxl_oid) + " rest_ticks=0", nullptr, 300, nullptr);
}

#else
bool sf_usb_accel_find(std::string &desc) { return desc = "CAPTAIN+ only", false; }
bool sf_usb_accel_start(double &rate, std::string *err) { return false; }
int sf_usb_accel_read(float *xyz, int max, int *overflow) { return -1; }
void sf_usb_accel_stop(void) {}
#endif
