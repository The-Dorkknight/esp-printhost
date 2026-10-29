// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// From KlipperTFT-Pico 0.6.1 (same author; published there under PolyForm Noncommercial 1.0.0).
#include "oled_i2c.hpp"

#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"

namespace ktft {

OledKind oled_kind_from(const std::string& n) {
    if (n == "ssd1306-32") return OledKind::Ssd1306_32;
    if (n == "sh1106") return OledKind::Sh1106;
    return OledKind::Ssd1306_64;
}

bool OledI2c::send(uint8_t ctrl, const uint8_t* d, size_t n) {
    uint8_t buf[132];
    if (!dev_ || n > sizeof buf - 1) return false;
    buf[0] = ctrl;
    memcpy(buf + 1, d, n);
    return i2c_master_transmit((i2c_master_dev_handle_t)dev_, buf, n + 1, 50) == ESP_OK;
}

void OledI2c::end() {
    if (dev_) i2c_master_bus_rm_device((i2c_master_dev_handle_t)dev_);
    dev_ = nullptr;
    ok_ = false;
}

bool OledI2c::begin(int sda, int scl, OledKind kind, bool flip) {
    end();
    if (bus_ && (sda != sda_ || scl != scl_)) {
        i2c_del_master_bus((i2c_master_bus_handle_t)bus_);
        bus_ = nullptr;
    }
    if (!bus_) {
        i2c_master_bus_config_t bc = {};
        bc.i2c_port = -1;                       // any free controller
        bc.sda_io_num = (gpio_num_t)sda;
        bc.scl_io_num = (gpio_num_t)scl;
        bc.clk_source = I2C_CLK_SRC_DEFAULT;
        bc.glitch_ignore_cnt = 7;
        bc.flags.enable_internal_pullup = true; // modules have their own; these only help
        i2c_master_bus_handle_t b = nullptr;
        if (i2c_new_master_bus(&bc, &b) != ESP_OK) { desc_ = "I2C start failed"; return false; }
        bus_ = b;
        sda_ = sda;
        scl_ = scl;
    }
    uint8_t addr = 0;
    if (i2c_master_probe((i2c_master_bus_handle_t)bus_, 0x3C, 50) == ESP_OK) addr = 0x3C;
    else if (i2c_master_probe((i2c_master_bus_handle_t)bus_, 0x3D, 50) == ESP_OK) addr = 0x3D;
    char b[80];
    if (!addr) {
        snprintf(b, sizeof b, "not found on GPIO%d/GPIO%d", sda, scl);
        desc_ = b;
        return false;
    }
    i2c_device_config_t dc = {};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dc.device_address = addr;
    dc.scl_speed_hz = 400000;
    i2c_master_dev_handle_t d = nullptr;
    if (i2c_master_bus_add_device((i2c_master_bus_handle_t)bus_, &dc, &d) != ESP_OK) { desc_ = "I2C device failed"; return false; }
    dev_ = d;
    kind_ = kind;
    flip_ = flip;
    uint8_t seq[32];
    int n = oled_init_sequence(kind == OledKind::Sh1106, height(), flip, seq, sizeof seq);
    ok_ = send(0x00, seq, (size_t)n);
    snprintf(b, sizeof b, "%s 128x%d at 0x%02X%s", kind == OledKind::Sh1106 ? "SH1106" : "SSD1306", height(), addr,
             ok_ ? "" : " (init failed)");
    desc_ = b;
    return ok_;
}

bool OledI2c::show(OledFrame& f) {
    if (!ok()) return false;
    if (flip_) f.rotate180();
    uint8_t col = kind_ == OledKind::Sh1106 ? 2 : 0;   // SH1106: 132-column RAM, picture starts at 2
    for (int p = 0; p < f.pages(); p++) {
        uint8_t set[3] = {(uint8_t)(0xB0 | p), (uint8_t)(col & 0x0F), (uint8_t)(0x10 | (col >> 4))};
        if (!send(0x00, set, 3) || !send(0x40, f.page(p), 128)) return false;
    }
    return true;
}

}  // namespace ktft
