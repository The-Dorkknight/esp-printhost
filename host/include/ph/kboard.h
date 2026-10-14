// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Configure a Klipper board the way Klipper's host does (allocate_oids, config commands,
// finalize_config with a CRC), and drive objects on it. First user: the board's NeoPixel.
#pragma once
#include "ph/klink.h"
#include <cstdint>
#include <string>
#include <vector>

namespace ph {

// zlib-compatible CRC-32 (what Klipper's mcu.py uses for the config CRC).
uint32_t crc32_zlib(const std::string& s);

// Klipper's neopixel.py timing: BIT_MAX_TIME 4 us, RESET_MIN_TIME 50 us.
std::string neopixel_config_cmd(int oid, const std::string& pin, int data_size, double clock_freq);

class KBoard {
public:
    enum class Result { Configured, AlreadyConfigured, ResetSent, Error };

    explicit KBoard(KLink& link) : link_(link) {}

    // cmds: the config commands without allocate_oids / finalize_config; oids 0..oid_count-1.
    // A board already configured with the same CRC is left alone. A board configured with a
    // different config, or shut down, is reset ("reset" command): the caller should wait about
    // two seconds, identify again and call configure() again.
    Result configure(const std::vector<std::string>& cmds, int oid_count, std::string* msg = nullptr);
    uint32_t crc() const { return crc_; }

    // Write LED bytes (in the LED's own colour order) and latch them, like neopixel.py:
    // neopixel_update in chunks of up to 16 bytes, then neopixel_send retried until success.
    bool neopixel_write(int oid, const std::vector<uint8_t>& data, std::string* err = nullptr);

private:
    KLink& link_;
    uint32_t crc_ = 0;
};

// Colour (r, g, b 0..255) to the byte order an LED expects ("GRB", "RGB", ...).
std::vector<uint8_t> led_bytes(const std::string& order, uint8_t r, uint8_t g, uint8_t b);

}  // namespace ph
