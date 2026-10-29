// ESP-IDF 5.x driver for the little I2C OLEDs (SSD1306 / SSD1315 / SH1106), on the new
// i2c_master API. Stand-alone: needs only ESP-IDF (esp_driver_i2c) and firmware-core/core/oled.*.
// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// From KlipperTFT-Pico 0.6.1 (same author; published there under PolyForm Noncommercial 1.0.0).
#pragma once
#include <string>

#include "oled.hpp"

namespace ktft {

enum class OledKind { Ssd1306_64, Ssd1306_32, Sh1106 };
OledKind oled_kind_from(const std::string& name);   // "auto"/"ssd1306-32"/"sh1106"

class OledI2c {
public:
    // Finds the display at 0x3C or 0x3D on these pins and initialises it. Safe to call again
    // (e.g. after a settings change). False if nothing answers; description() says why.
    bool begin(int sda_gpio, int scl_gpio, OledKind kind, bool flip);
    void end();
    bool ok() const { return dev_ != nullptr && ok_; }
    int height() const { return kind_ == OledKind::Ssd1306_32 ? 32 : 64; }
    // Sends the frame (8 pages of 128 bytes, ~25 ms at 400 kHz). Rotates first if flip is set.
    bool show(OledFrame& f);
    const std::string& description() const { return desc_; }
    // The I2C bus (i2c_master_bus_handle_t) once begin() has run, else nullptr: another device on the
    // same two wires (an accelerometer, Skipper Flipper 2026-10-24) can be added to it.
    void* bus() const { return bus_; }

private:
    void* bus_ = nullptr;   // i2c_master_bus_handle_t
    void* dev_ = nullptr;   // i2c_master_dev_handle_t
    int sda_ = -1, scl_ = -1;
    OledKind kind_ = OledKind::Ssd1306_64;
    bool flip_ = false, ok_ = false;
    std::string desc_ = "not started";
    bool send(uint8_t ctrl, const uint8_t* d, size_t n);
};

}  // namespace ktft
