// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// The accelerometer for input shaper calibration (2026-10-24; CAPTAIN / SKIPPER). Never run on
// hardware yet.
//
// Find it (GET /accel?scan=1[&usb=1]), looking in this order:
//   1. the OLED's two I2C wires (an accelerometer can share them): ADXL345 at 0x53 / 0x1D,
//      MPU-6050 / 6500 / 9250 at 0x68 / 0x69
//   2. every pair of spare pins as I2C (internal pull-ups), the same addresses
//   3. every combination of spare pins as SPI for an ADXL345 (bit-banged: CS, SCK, MOSI driven, all
//      other spare pins read at once as MISO; DEVID must read 0xE5 twice and BW_RATE its reset value)
//   4. with usb=1: the USB-C port as USB host, for a Klipper USB accelerometer (usb_accel.cpp)
// "Spare" = not the printer board UART, the status LED, the SD card, USB (19/20), the start-up
// pins (0, 3, 45, 46) or the flash / PSRAM pins (26-37). What it finds is kept (NVS, "acc_*").
//
// Record (from the resonance test, MachineIO::accel_record): a task reads the chip's FIFO every
// 10 ms (ADXL345 at 1600 samples/s, MPU at 1000) and feeds ph::PsdAccumulator (the streaming
// Welch spectrum), so no samples are stored. Afterwards ph::find_best_shaper runs in its own
// low-priority task (5-10 s); GET /accel.json has the state, the spectrum and the shapers.
#include <math.h>
#include <string.h>

#include <string>
#include <vector>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "ph/resonance.h"
#include "sdkconfig.h"
#include "sf.h"
#include "soc/gpio_reg.h"

#if CONFIG_SKIPPER_PRO

static const char *TAG = "accel";

enum { ACC_NONE = 0, ACC_ADXL_SPI = 1, ACC_ADXL_I2C = 2, ACC_MPU_I2C = 3, ACC_USB = 4 };
struct Chip {
    int type = ACC_NONE;
    int p[4] = {-1, -1, -1, -1};   // SPI: cs sck mosi miso; I2C: sda scl
    int addr = 0;
    int mpu_id = 0;                // MPU WHO_AM_I
};
static Chip s_chip;
static sf_settings_t s_set;
static SemaphoreHandle_t s_lock;   // the result strings
static std::string s_scan = "Not searched yet.";
static std::string s_state = "idle", s_result = "null", s_spectrum = "", s_error = "";
static volatile bool s_busy;       // a scan, a recording or an analysis

static const char *type_name(int t)
{
    switch (t) {
    case ACC_ADXL_SPI: return "ADXL345 (SPI)";
    case ACC_ADXL_I2C: return "ADXL345 (I2C)";
    case ACC_MPU_I2C: return "MPU-6050 / 6500 (I2C)";
    case ACC_USB: return "Klipper USB accelerometer";
    default: return "none";
    }
}

static std::string chip_text(const Chip &c)
{
    char b[120];
    if (c.type == ACC_ADXL_SPI) snprintf(b, sizeof b, "%s, CS GPIO%d SCK %d MOSI(SDA/SDI) %d MISO(SDO) %d", type_name(c.type), c.p[0], c.p[1], c.p[2], c.p[3]);
    else if (c.type == ACC_ADXL_I2C || c.type == ACC_MPU_I2C)
        snprintf(b, sizeof b, "%s at 0x%02X, SDA GPIO%d SCL GPIO%d%s", type_name(c.type), c.addr, c.p[0], c.p[1],
                 (c.p[0] == s_set.oled_sda && c.p[1] == s_set.oled_scl) ? " (shared with the OLED)" : "");
    else snprintf(b, sizeof b, "%s", type_name(c.type));
    return b;
}

static void set_str(std::string &dst, const std::string &v)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    dst = v;
    xSemaphoreGive(s_lock);
}

static void chip_save(void)
{
    nvs_handle_t h;
    if (nvs_open("skipper", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i32(h, "acc_type", s_chip.type);
    nvs_set_i32(h, "acc_p0", s_chip.p[0]);
    nvs_set_i32(h, "acc_p1", s_chip.p[1]);
    nvs_set_i32(h, "acc_p2", s_chip.p[2]);
    nvs_set_i32(h, "acc_p3", s_chip.p[3]);
    nvs_set_i32(h, "acc_addr", s_chip.addr);
    nvs_commit(h);
    nvs_close(h);
}

static void chip_load(void)
{
    nvs_handle_t h;
    if (nvs_open("skipper", NVS_READONLY, &h) != ESP_OK) return;
    int32_t v;
    if (nvs_get_i32(h, "acc_type", &v) == ESP_OK) s_chip.type = (int)v;
    const char *k[4] = {"acc_p0", "acc_p1", "acc_p2", "acc_p3"};
    for (int i = 0; i < 4; i++)
        if (nvs_get_i32(h, k[i], &v) == ESP_OK) s_chip.p[i] = (int)v;
    if (nvs_get_i32(h, "acc_addr", &v) == ESP_OK) s_chip.addr = (int)v;
    nvs_close(h);
}

// ---------------------------------------------------------------- spare pins

static std::vector<int> spare_pins(void)
{
    static const int CAND[] = {1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 38, 39, 40, 41, 42, 43, 44, 47, 48};
    std::vector<int> out;
    for (int g : CAND) {
        if (!GPIO_IS_VALID_OUTPUT_GPIO(g)) continue;
        if (s_set.link_enabled && (g == s_set.uart_tx || g == s_set.uart_rx)) continue;
        if (s_set.led_type != SF_LED_NONE && g == s_set.led_gpio) continue;
        if (s_set.sd_enabled && (g == s_set.sd_cs || g == s_set.sd_sck || g == s_set.sd_mosi || g == s_set.sd_miso)) continue;
        if (s_set.oled_type != SF_OLED_OFF && (g == s_set.oled_sda || g == s_set.oled_scl)) continue;   // scanned as the OLED bus
        if (sf_panel_uses_pin(g)) continue;   // a button on the case
        out.push_back(g);
    }
    return out;
}

// ---------------------------------------------------------------- I2C

static i2c_master_bus_handle_t s_own_bus;   // our own bus (not the OLED's), while in use
static i2c_master_dev_handle_t s_i2c_dev;

static i2c_master_bus_handle_t bus_for(int sda, int scl)
{
    if (s_set.oled_type != SF_OLED_OFF && sda == s_set.oled_sda && scl == s_set.oled_scl && sf_oled_bus())
        return (i2c_master_bus_handle_t)sf_oled_bus();
    i2c_master_bus_config_t bc = {};
    bc.i2c_port = -1;
    bc.sda_io_num = (gpio_num_t)sda;
    bc.scl_io_num = (gpio_num_t)scl;
    bc.clk_source = I2C_CLK_SRC_DEFAULT;
    bc.glitch_ignore_cnt = 7;
    bc.flags.enable_internal_pullup = true;
    if (i2c_new_master_bus(&bc, &s_own_bus) != ESP_OK) {
        s_own_bus = nullptr;
        return nullptr;
    }
    return s_own_bus;
}

static void i2c_close(void)
{
    if (s_i2c_dev) i2c_master_bus_rm_device(s_i2c_dev);
    s_i2c_dev = nullptr;
    if (s_own_bus) i2c_del_master_bus(s_own_bus);
    s_own_bus = nullptr;
}

static bool i2c_open(int sda, int scl, int addr)
{
    i2c_close();
    i2c_master_bus_handle_t b = bus_for(sda, scl);
    if (!b) return false;
    i2c_device_config_t dc = {};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dc.device_address = (uint16_t)addr;
    dc.scl_speed_hz = 400000;
    if (i2c_master_bus_add_device(b, &dc, &s_i2c_dev) != ESP_OK) {
        s_i2c_dev = nullptr;
        i2c_close();
        return false;
    }
    return true;
}

static bool i2c_read(uint8_t reg, uint8_t *buf, size_t n)
{
    return s_i2c_dev && i2c_master_transmit_receive(s_i2c_dev, &reg, 1, buf, n, 20) == ESP_OK;
}

static bool i2c_write(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = {reg, val};
    return s_i2c_dev && i2c_master_transmit(s_i2c_dev, b, 2, 20) == ESP_OK;
}

// What answers at the accelerometer addresses on this bus: fills c, true if one is found.
static bool i2c_identify(int sda, int scl, Chip &c)
{
    i2c_master_bus_handle_t b = bus_for(sda, scl);
    if (!b) return false;
    bool own = b == s_own_bus;
    static const int ADDRS[4] = {0x53, 0x1D, 0x68, 0x69};
    bool found = false;
    for (int a : ADDRS) {
        if (found || i2c_master_probe(b, (uint16_t)a, 5) != ESP_OK) continue;
        i2c_device_config_t dc = {};
        dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dc.device_address = (uint16_t)a;
        dc.scl_speed_hz = 400000;
        i2c_master_dev_handle_t d = nullptr;
        if (i2c_master_bus_add_device(b, &dc, &d) != ESP_OK) continue;
        uint8_t reg = a >= 0x68 ? 0x75 : 0x00, id = 0;
        if (i2c_master_transmit_receive(d, &reg, 1, &id, 1, 20) == ESP_OK) {
            if (a < 0x68 && id == 0xE5) c.type = ACC_ADXL_I2C, found = true;
            if (a >= 0x68 && (id == 0x68 || id == 0x70 || id == 0x71 || id == 0x73)) c.type = ACC_MPU_I2C, c.mpu_id = id, found = true;
        }
        i2c_master_bus_rm_device(d);
        if (found) c.p[0] = sda, c.p[1] = scl, c.p[2] = c.p[3] = -1, c.addr = a;
    }
    if (own) {
        i2c_del_master_bus(b);
        s_own_bus = nullptr;
    }
    return found;
}

// ---------------------------------------------------------------- SPI (bit-banged for the search)

static inline int pin_in(int g)
{
    return g < 32 ? (REG_READ(GPIO_IN_REG) >> g) & 1 : (REG_READ(GPIO_IN1_REG) >> (g - 32)) & 1;
}

static void bb_release(const std::vector<int> &pins)
{
    for (int g : pins) {
        gpio_reset_pin((gpio_num_t)g);
        gpio_set_direction((gpio_num_t)g, GPIO_MODE_INPUT);
        gpio_set_pull_mode((gpio_num_t)g, GPIO_PULLUP_ONLY);   // an undriven MISO reads 0xFF, never 0xE5
    }
}

// SPI mode 3, ~200 kHz: read register `reg`; the byte seen on every other spare pin comes back in rx.
static void bb_read(int cs, int sck, int mosi, uint8_t reg, const std::vector<int> &pins, uint8_t *rx)
{
    gpio_set_level((gpio_num_t)cs, 0);
    esp_rom_delay_us(2);
    uint8_t cmd = 0x80 | reg;
    for (int i = 7; i >= 0; i--) {
        gpio_set_level((gpio_num_t)sck, 0);
        gpio_set_level((gpio_num_t)mosi, (cmd >> i) & 1);
        esp_rom_delay_us(2);
        gpio_set_level((gpio_num_t)sck, 1);
        esp_rom_delay_us(2);
    }
    for (size_t k = 0; k < pins.size(); k++) rx[k] = 0;
    for (int i = 7; i >= 0; i--) {
        gpio_set_level((gpio_num_t)sck, 0);
        esp_rom_delay_us(2);
        gpio_set_level((gpio_num_t)sck, 1);
        esp_rom_delay_us(1);
        for (size_t k = 0; k < pins.size(); k++) rx[k] |= (uint8_t)(pin_in(pins[k]) << i);
    }
    gpio_set_level((gpio_num_t)cs, 1);
    esp_rom_delay_us(2);
}

static bool spi_search(const std::vector<int> &pins, Chip &c)
{
    bb_release(pins);
    std::vector<uint8_t> rx(pins.size()), rx2(pins.size()), rx3(pins.size());
    for (int cs : pins) {
        vTaskDelay(1);   // let the idle task (and its watchdog) run between rounds
        for (int sck : pins)
            for (int mosi : pins) {
                if (cs == sck || cs == mosi || sck == mosi) continue;
                for (int g : {cs, sck, mosi}) gpio_set_direction((gpio_num_t)g, GPIO_MODE_OUTPUT);
                gpio_set_level((gpio_num_t)cs, 1);
                gpio_set_level((gpio_num_t)sck, 1);
                bb_read(cs, sck, mosi, 0x00, pins, rx.data());
                for (size_t k = 0; k < pins.size(); k++) {
                    int miso = pins[k];
                    if (miso == cs || miso == sck || miso == mosi || rx[k] != 0xE5) continue;
                    bb_read(cs, sck, mosi, 0x00, pins, rx2.data());   // twice, and BW_RATE (0x2C) at its reset 0x0A
                    bb_read(cs, sck, mosi, 0x2C, pins, rx3.data());
                    if (rx2[k] == 0xE5 && (rx3[k] & 0x1F) == 0x0A) {
                        c.type = ACC_ADXL_SPI;
                        c.p[0] = cs, c.p[1] = sck, c.p[2] = mosi, c.p[3] = miso;
                        bb_release(pins);
                        return true;
                    }
                }
                bb_release({cs, sck, mosi});
            }
    }
    bb_release(pins);
    return false;
}

static spi_device_handle_t s_spi_dev;

static bool spi_open(const Chip &c)
{
    spi_bus_config_t bc = {};
    bc.mosi_io_num = c.p[2];
    bc.miso_io_num = c.p[3];
    bc.sclk_io_num = c.p[1];
    bc.quadwp_io_num = bc.quadhd_io_num = -1;
    bc.max_transfer_sz = 64;
    if (spi_bus_initialize(SPI3_HOST, &bc, SPI_DMA_DISABLED) != ESP_OK) return false;
    spi_device_interface_config_t dc = {};
    dc.mode = 3;
    dc.clock_speed_hz = 5000000;   // Klipper's adxl345 default
    dc.spics_io_num = c.p[0];
    dc.queue_size = 2;
    if (spi_bus_add_device(SPI3_HOST, &dc, &s_spi_dev) != ESP_OK) {
        spi_bus_free(SPI3_HOST);
        return false;
    }
    return true;
}

static void spi_close(void)
{
    if (s_spi_dev) {
        spi_bus_remove_device(s_spi_dev);
        spi_bus_free(SPI3_HOST);
    }
    s_spi_dev = nullptr;
}

static bool spi_xfer(uint8_t *buf, size_t n)
{
    spi_transaction_t t = {};
    t.length = n * 8;
    t.tx_buffer = buf;
    t.rx_buffer = buf;
    return s_spi_dev && spi_device_polling_transmit(s_spi_dev, &t) == ESP_OK;
}

// ---------------------------------------------------------------- the chips

static bool reg_read(uint8_t reg, uint8_t *buf, size_t n)
{
    if (s_chip.type == ACC_ADXL_SPI) {
        uint8_t b[8] = {(uint8_t)(0x80 | (n > 1 ? 0x40 : 0) | reg)};
        if (n > 7 || !spi_xfer(b, n + 1)) return false;
        memcpy(buf, b + 1, n);
        return true;
    }
    return i2c_read(reg, buf, n);
}

static bool reg_write(uint8_t reg, uint8_t val)
{
    if (s_chip.type == ACC_ADXL_SPI) {
        uint8_t b[2] = {reg, val};
        return spi_xfer(b, 2);
    }
    return i2c_write(reg, val);
}

static bool chip_open(std::string *err)
{
    bool ok = s_chip.type == ACC_ADXL_SPI ? spi_open(s_chip)
              : (s_chip.type == ACC_ADXL_I2C || s_chip.type == ACC_MPU_I2C) ? i2c_open(s_chip.p[0], s_chip.p[1], s_chip.addr)
                                                                            : false;
    if (!ok && err) *err = "could not open " + chip_text(s_chip);
    return ok;
}

static void chip_close(void)
{
    if (s_chip.type == ACC_ADXL_SPI) {
        reg_write(0x2D, 0x00);   // POWER_CTL: standby
        spi_close();
    } else if (s_chip.type == ACC_ADXL_I2C) {
        reg_write(0x2D, 0x00);
        i2c_close();
    } else if (s_chip.type == ACC_MPU_I2C) {
        reg_write(0x6B, 0x40);   // PWR_MGMT_1: sleep
        i2c_close();
    }
}

static double s_rate_nominal;   // samples per second asked for

// Klipper adxl345.py / mpu9250.py set-up (rates lower than Klipper's: see the file comment)
static bool chip_start(std::string *err)
{
    uint8_t id = 0;
    if (s_chip.type == ACC_ADXL_SPI || s_chip.type == ACC_ADXL_I2C) {
        if (!reg_read(0x00, &id, 1) || id != 0xE5) {
            if (err) *err = "ADXL345 not answering (DEVID " + std::to_string(id) + ", expected 229): check the wiring";
            return false;
        }
        bool ok = reg_write(0x2D, 0x00) && reg_write(0x31, 0x0B) && reg_write(0x38, 0x00) && reg_write(0x2C, 0x0E) &&
                  reg_write(0x38, 0x90) && reg_write(0x2D, 0x08);   // full resolution +-16 g, 1600 Hz, stream FIFO
        s_rate_nominal = 1600;
        if (!ok && err) *err = "ADXL345 set-up failed";
        return ok;
    }
    if (s_chip.type == ACC_MPU_I2C) {
        if (!reg_read(0x75, &id, 1)) {
            if (err) *err = "MPU not answering";
            return false;
        }
        s_chip.mpu_id = id;
        bool ok = reg_write(0x6B, 0x00) && reg_write(0x1A, 0x01) && reg_write(0x19, 0x00) && reg_write(0x1C, 0x10);
        if (id != 0x68) ok = ok && reg_write(0x1D, 0x01);   // MPU-6500/9250: accel at 1 kHz too
        ok = ok && reg_write(0x6C, 0x07) && reg_write(0x6A, 0x04);   // gyro off; FIFO reset
        vTaskDelay(pdMS_TO_TICKS(20));
        ok = ok && reg_write(0x6A, 0x40) && reg_write(0x23, 0x08);   // FIFO on, accelerometer into it
        s_rate_nominal = 1000;
        if (!ok && err) *err = "MPU set-up failed";
        return ok;
    }
    if (err) *err = "no accelerometer set up (Find accelerometer first)";
    return false;
}

// Samples waiting in the chip, in mm/s^2. Returns the count (<= max), -1 on an error.
static int chip_read(float *xyz, int max, int *overflow)
{
    const float G = 9806.65f;
    if (s_chip.type == ACC_ADXL_SPI || s_chip.type == ACC_ADXL_I2C) {
        uint8_t st = 0;
        if (!reg_read(0x39, &st, 1)) return -1;
        int n = st & 0x3F;
        if (n >= 32) (*overflow)++;
        if (n > max) n = max;
        for (int i = 0; i < n; i++) {
            uint8_t b[6];
            if (!reg_read(0x32, b, 6)) return -1;
            int16_t x = (int16_t)(b[0] | b[1] << 8), y = (int16_t)(b[2] | b[3] << 8), z = (int16_t)(b[4] | b[5] << 8);
            xyz[i * 3 + 0] = x * 0.003774f * G;   // Klipper's SCALE_XY / SCALE_Z
            xyz[i * 3 + 1] = y * 0.003774f * G;
            xyz[i * 3 + 2] = z * 0.003906f * G;
        }
        return n;
    }
    if (s_chip.type == ACC_MPU_I2C) {
        uint8_t c[2];
        if (!reg_read(0x72, c, 2)) return -1;
        int bytes = (c[0] << 8) | c[1];
        if (bytes >= 1020) (*overflow)++;
        int n = bytes / 6;
        if (n > max) n = max;
        int done = 0;
        while (done < n) {
            int k = n - done > 20 ? 20 : n - done;
            uint8_t b[120];
            if (!i2c_read(0x74, b, (size_t)(k * 6))) return -1;
            for (int i = 0; i < k; i++) {
                for (int a = 0; a < 3; a++) {
                    int16_t v = (int16_t)(b[i * 6 + a * 2] << 8 | b[i * 6 + a * 2 + 1]);
                    xyz[(done + i) * 3 + a] = v * (G / 4096.0f);   // +-8 g
                }
            }
            done += k;
        }
        return n;
    }
    return -1;
}

// usb_accel.cpp
bool sf_usb_accel_find(std::string &desc);
bool sf_usb_accel_start(double &rate, std::string *err);
int sf_usb_accel_read(float *xyz, int max, int *overflow);
void sf_usb_accel_stop(void);

// ---------------------------------------------------------------- recording and analysis

static ph::PsdAccumulator *s_psd;
static volatile int s_rec;   // 1 recording, 0 stop asked
static TaskHandle_t s_rec_task;
static int s_axis;
static uint64_t s_t_first, s_t_last;
static int s_overflows;
static ph::CalibrationData *s_cal;

static void rec_task(void *arg)
{
    static float buf[64 * 3];
    while (s_rec) {
        int n = s_chip.type == ACC_USB ? sf_usb_accel_read(buf, 64, &s_overflows) : chip_read(buf, 64, &s_overflows);
        uint64_t now = (uint64_t)esp_timer_get_time();
        if (n > 0) {
            if (!s_t_first) s_t_first = now;
            s_t_last = now;
            for (int i = 0; i < n; i++) s_psd->add(buf[i * 3], buf[i * 3 + 1], buf[i * 3 + 2]);
        }
        if (n < 32) vTaskDelay(1);   // 10 ms
    }
    s_rec_task = nullptr;
    vTaskDelete(NULL);
}

static void progress(int i, int n)
{
    char b[64];
    snprintf(b, sizeof b, "working out the shaper (%d of %d)", i, n);
    set_str(s_state, b);
}

static void fit_task(void *arg)
{
    std::vector<ph::ShaperFit> all;
    ph::ShaperFit best = ph::find_best_shaper(*s_cal, 5.0, 1.5 * 135, &all, 0, progress);
    std::string j = "{\"axis\":\"" + std::string(s_axis ? "y" : "x") + "\",\"best\":\"" + best.name + "\",\"freq\":";
    char b[200];
    snprintf(b, sizeof b, "%.1f,\"max_accel\":%.0f,\"shapers\":[", best.freq, round(best.max_accel / 100) * 100);
    j += b;
    for (size_t i = 0; i < all.size(); i++) {
        snprintf(b, sizeof b, "%s{\"name\":\"%s\",\"freq\":%.1f,\"vibrs\":%.4f,\"smoothing\":%.4f,\"max_accel\":%.0f}", i ? "," : "",
                 all[i].name.c_str(), all[i].freq, all[i].vibrs, all[i].smoothing, round(all[i].max_accel / 100) * 100);
        j += b;
    }
    j += "]}";
    ESP_LOGW(TAG, "best shaper for %c: %s at %.1f Hz (suggested max_accel %.0f)", s_axis ? 'Y' : 'X', best.name.c_str(),
             best.freq, round(best.max_accel / 100) * 100);
    set_str(s_result, j);
    set_str(s_state, "done");
    delete s_cal;
    s_cal = nullptr;
    s_busy = false;
    vTaskDelete(NULL);
}

// The spectrum for the page: the peak summed power in 5 Hz bands from 5 to 150 Hz.
static std::string spectrum_json(const ph::CalibrationData &d)
{
    std::string j = "[";
    char b[40];
    for (double f = 5; f < 150; f += 5) {
        double s = 0;
        for (size_t k = 0; k < d.freqs.size(); k++)
            if (d.freqs[k] >= f && d.freqs[k] < f + 5) s = std::max(s, d.psd[3][k]);
        snprintf(b, sizeof b, "%s[%.1f,%.4g]", j.size() > 1 ? "," : "", f, s);
        j += b;
    }
    return j + "]";
}

// MachineIO::accel_record, called from the printer link task: quick, no waiting on the analysis.
extern "C" bool sf_accel_record(int mode, int axis, char *err, size_t errn)
{
    std::string e;
    auto fail = [&](const std::string &why) {
        if (err) snprintf(err, errn, "%s", why.c_str());
        set_str(s_error, why);
        set_str(s_state, "error");
        return false;
    };
    if (mode == 1) {
        if (s_busy) return fail("the accelerometer is busy");
        if (s_chip.type == ACC_NONE) return fail("no accelerometer found yet (Tune page: 1. Find accelerometer)");
        s_busy = true;
        sf_oled_pause(true);
        double rate = 0;
        bool ok = s_chip.type == ACC_USB ? sf_usb_accel_start(rate, &e) : (chip_open(&e) && chip_start(&e));
        if (s_chip.type != ACC_USB) rate = s_rate_nominal;
        if (!ok) {
            if (s_chip.type != ACC_USB) chip_close();
            sf_oled_pause(false);
            s_busy = false;
            return fail(e);
        }
        delete s_psd;
        s_psd = new ph::PsdAccumulator();
        s_psd->start(rate);
        s_axis = axis;
        s_t_first = s_t_last = 0;
        s_overflows = 0;
        s_rec = 1;
        set_str(s_state, "recording");
        set_str(s_result, "null");
        set_str(s_error, "");
        if (xTaskCreate(rec_task, "accel", 6144, NULL, 3, &s_rec_task) != pdPASS) {
            s_rec = 0;
            chip_close();
            sf_oled_pause(false);
            s_busy = false;
            return fail("not enough memory for the accelerometer task");
        }
        ESP_LOGW(TAG, "recording %c with %s, %.0f samples/s, %u bytes for the spectrum", axis ? 'Y' : 'X',
                 chip_text(s_chip).c_str(), rate, (unsigned)s_psd->bytes());
        return true;
    }
    // stop
    s_rec = 0;
    for (int i = 0; i < 50 && s_rec_task; i++) vTaskDelay(pdMS_TO_TICKS(10));
    if (s_chip.type == ACC_USB) sf_usb_accel_stop();
    else chip_close();
    sf_oled_pause(false);
    if (mode < 0) {
        delete s_psd;
        s_psd = nullptr;
        set_str(s_state, "idle");
        s_busy = false;
        return true;
    }
    double secs = (double)(s_t_last - s_t_first) * 1e-6;
    size_t n = s_psd ? s_psd->samples() : 0;
    if (!s_psd || n < 1000 || secs <= 0 || s_psd->windows() == 0) {
        s_busy = false;
        return fail("too few samples (" + std::to_string(n) + "): the accelerometer stopped answering?");
    }
    double rate = (double)n / secs;   // as Klipper: samples / time
    s_cal = new ph::CalibrationData();
    s_psd->finish(rate, s_cal->freqs, s_cal->psd);
    delete s_psd;
    s_psd = nullptr;
    s_cal->normalize_to_frequencies();
    set_str(s_spectrum, spectrum_json(*s_cal));
    ESP_LOGW(TAG, "%u samples in %.1f s (%.0f/s), %d FIFO overflows", (unsigned)n, secs, rate, s_overflows);
    set_str(s_state, "working out the shaper");
    if (xTaskCreate(fit_task, "shaperfit", 8192, NULL, 1, NULL) != pdPASS) {
        delete s_cal;
        s_cal = nullptr;
        s_busy = false;
        return fail("not enough memory to work out the shaper");
    }
    return true;
}

// ---------------------------------------------------------------- the search, and the web side

static bool s_scan_usb;

static void scan_task(void *arg)
{
    std::string log;
    Chip c;
    bool found = false;
    esp_log_level_t old = esp_log_level_get("i2c.master");
    esp_log_level_set("i2c.master", ESP_LOG_NONE);
    sf_oled_pause(true);
    if (s_set.oled_type != SF_OLED_OFF && s_set.oled_sda >= 0) {
        set_str(s_scan, "looking on the OLED's wires...");
        found = i2c_identify(s_set.oled_sda, s_set.oled_scl, c);
        log += found ? "" : "OLED wires (GPIO" + std::to_string(s_set.oled_sda) + "/" + std::to_string(s_set.oled_scl) + "): nothing\n";
    }
    std::vector<int> pins = spare_pins();
    if (!found) {
        set_str(s_scan, "trying every pair of spare pins as I2C...");
        for (size_t i = 0; i < pins.size() && !found; i++)
            for (size_t j = 0; j < pins.size() && !found; j++) {
                if (i != j) found = i2c_identify(pins[i], pins[j], c);
                if ((j & 7) == 0) vTaskDelay(1);
            }
        if (!found) log += "I2C on " + std::to_string(pins.size()) + " spare pins: nothing\n";
    }
    if (!found && pins.size() >= 4) {
        set_str(s_scan, "trying spare pins as SPI (ADXL345)...");
        found = spi_search(pins, c);
        if (!found) log += "SPI (ADXL345) on the spare pins: nothing\n";
    }
    esp_log_level_set("i2c.master", old);
    sf_oled_pause(false);
    if (!found && s_scan_usb) {
        set_str(s_scan, "looking on the USB port...");
        std::string d;
        if (sf_usb_accel_find(d)) {
            c.type = ACC_USB;
            found = true;
            log += d + "\n";
        } else {
            log += "USB: " + d + "\n";
        }
    }
    if (found) {
        s_chip = c;
        chip_save();
        log = "Found: " + chip_text(c) + " (kept for next time)\n" + log;
    } else {
        std::string sp;
        for (int g : pins) sp += " " + std::to_string(g);
        log = "No accelerometer found. Spare pins searched:" + sp + "\n" + log;
    }
    ESP_LOGW(TAG, "%s", log.c_str());
    set_str(s_scan, log);
    s_busy = false;
    vTaskDelete(NULL);
}

static std::string jstr(const std::string &s)
{
    std::string o = "\"";
    for (char ch : s) {
        if (ch == '"' || ch == '\\') o += '\\';
        if (ch == '\n') o += "\\n";
        else if ((unsigned char)ch >= 32) o += ch;
    }
    return o + "\"";
}

static esp_err_t accel_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    char q[64] = "";
    httpd_req_get_url_query_str(req, q, sizeof q);
    if (strstr(q, "scan=1")) {
        if (s_busy) return httpd_resp_sendstr(req, "{\"error\":\"busy\"}");
        if (sf_printing()) return httpd_resp_sendstr(req, "{\"error\":\"not while printing\"}");
        s_busy = true;
        s_scan_usb = strstr(q, "usb=1") != nullptr;
        set_str(s_scan, "starting...");
        if (xTaskCreate(scan_task, "accscan", 6144, NULL, 2, NULL) != pdPASS) s_busy = false;
    } else if (strstr(q, "read=1")) {   // one reading: is it alive, and which way is up
        if (s_busy) return httpd_resp_sendstr(req, "{\"error\":\"busy\"}");
        std::string e;
        float xyz[3 * 40];
        int ov = 0, n = 0;
        if (s_chip.type == ACC_USB) {
            e = "use a resonance test to check a USB accelerometer";
        } else if (chip_open(&e) && chip_start(&e)) {
            vTaskDelay(pdMS_TO_TICKS(60));
            n = chip_read(xyz, 40, &ov);
        }
        chip_close();
        char b[200];
        if (n > 0) {
            float x = 0, y = 0, z = 0;
            for (int i = 0; i < n; i++) x += xyz[i * 3], y += xyz[i * 3 + 1], z += xyz[i * 3 + 2];
            x /= n, y /= n, z /= n;
            snprintf(b, sizeof b, "{\"x\":%.0f,\"y\":%.0f,\"z\":%.0f,\"g\":%.0f,\"n\":%d}", x, y, z, sqrtf(x * x + y * y + z * z), n);
        } else {
            snprintf(b, sizeof b, "{\"error\":%s}", jstr(e.empty() ? "no samples" : e).c_str());
        }
        return httpd_resp_sendstr(req, b);
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    std::string j = "{\"chip\":" + jstr(chip_text(s_chip)) + ",\"found\":" + (s_chip.type ? "true" : "false") +
                    ",\"busy\":" + (s_busy ? "true" : "false") + ",\"scan\":" + jstr(s_scan) + ",\"state\":" + jstr(s_state) +
                    ",\"error\":" + jstr(s_error) + ",\"result\":" + s_result + ",\"spectrum\":" +
                    (s_spectrum.empty() ? "null" : s_spectrum) + "}";
    xSemaphoreGive(s_lock);
    return httpd_resp_sendstr(req, j.c_str());
}

extern "C" void sf_accel_scan_now(const sf_settings_t *s)
{
    s_set = *s;
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    int64_t t0 = esp_timer_get_time();
    s_busy = true;
    s_scan_usb = false;
    xTaskCreate(scan_task, "accscan", 6144, NULL, 2, NULL);
    while (s_busy) vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGW(TAG, "SCAN took %.1f s", (esp_timer_get_time() - t0) * 1e-6);
}

extern "C" void sf_accel_init(const sf_settings_t *s, void *httpd)
{
    s_set = *s;
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    chip_load();
    static const httpd_uri_t u = {.uri = "/accel", .method = HTTP_GET, .handler = accel_get, .user_ctx = NULL};
    static const httpd_uri_t u2 = {.uri = "/accel.json", .method = HTTP_GET, .handler = accel_get, .user_ctx = NULL};
    httpd_register_uri_handler((httpd_handle_t)httpd, &u);
    httpd_register_uri_handler((httpd_handle_t)httpd, &u2);
}

#else

extern "C" bool sf_accel_record(int mode, int axis, char *err, size_t errn)
{
    if (err) snprintf(err, errn, "the resonance test needs the CAPTAIN or SKIPPER build");
    return false;
}
extern "C" void sf_accel_init(const sf_settings_t *s, void *httpd) {}
extern "C" void sf_accel_scan_now(const sf_settings_t *s) {}

#endif
