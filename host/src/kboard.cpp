// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
#include "ph/kboard.h"

#include <cstdio>

namespace ph {

uint32_t crc32_zlib(const std::string& s) {
    uint32_t c = 0xFFFFFFFFu;
    for (unsigned char ch : s) {
        c ^= ch;
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return c ^ 0xFFFFFFFFu;
}

std::string neopixel_config_cmd(int oid, const std::string& pin, int data_size, double clock_freq) {
    // Same as Klipper's mcu.seconds_to_clock(): int(seconds * freq)
    long bit_max = long(0.000004 * clock_freq);
    long reset_min = long(0.000050 * clock_freq);
    char buf[160];
    snprintf(buf, sizeof buf, "config_neopixel oid=%d pin=%s data_size=%d bit_max_ticks=%ld reset_min_ticks=%ld",
             oid, pin.c_str(), data_size, bit_max, reset_min);
    return buf;
}

std::vector<uint8_t> led_bytes(const std::string& order, uint8_t r, uint8_t g, uint8_t b) {
    std::vector<uint8_t> out;
    for (char c : order) {
        if (c == 'R') out.push_back(r);
        else if (c == 'G') out.push_back(g);
        else if (c == 'B') out.push_back(b);
        else if (c == 'W') out.push_back(0);
    }
    return out;
}

KBoard::Result KBoard::configure(const std::vector<std::string>& cmds, int oid_count, std::string* msg) {
    std::vector<std::string> full;
    full.push_back("allocate_oids count=" + std::to_string(oid_count));
    full.insert(full.end(), cmds.begin(), cmds.end());
    std::string joined;
    for (size_t i = 0; i < full.size(); i++) joined += (i ? "\n" : "") + full[i];
    crc_ = crc32_zlib(joined);

    bool configured = false, shutdown = false;
    uint32_t board_crc = 0;
    std::string err;
    if (!link_.get_config(configured, shutdown, &err, &board_crc)) {
        if (msg) *msg = "get_config: " + err;
        return Result::Error;
    }
    if (!shutdown && configured && board_crc == crc_) {
        if (msg) *msg = "already configured with this config";
        return Result::AlreadyConfigured;
    }
    if (shutdown || configured) {
        // Klipper's firmware_restart: the board can only take a new config after a reboot.
        // Sent with every sequence number: works even if the board lost count of our messages.
        if (!link_.blind_reset()) {
            if (msg) *msg = "reset: could not send";
            return Result::Error;
        }
        if (msg) *msg = shutdown ? "board was shut down: reset sent" : "board had a different config: reset sent";
        return Result::ResetSent;
    }
    for (auto& c : full) {
        if (!link_.command(c, nullptr, 300, &err)) {
            if (msg) *msg = err;
            return Result::Error;
        }
    }
    if (!link_.command("finalize_config crc=" + std::to_string(crc_), nullptr, 300, &err)) {
        if (msg) *msg = err;
        return Result::Error;
    }
    if (!link_.get_config(configured, shutdown, &err, &board_crc) || !configured || shutdown || board_crc != crc_) {
        if (msg) *msg = shutdown ? "board shut down while configuring (check the pin names)"
                                 : "board did not accept the config";
        return Result::Error;
    }
    if (msg) *msg = "configured";
    return Result::Configured;
}

bool KBoard::neopixel_write(int oid, const std::vector<uint8_t>& data, std::string* err) {
    static const char* hexd = "0123456789abcdef";
    for (size_t pos = 0; pos < data.size(); pos += 16) {
        size_t n = data.size() - pos < 16 ? data.size() - pos : 16;
        std::string hx = "hex:";
        for (size_t i = 0; i < n; i++) {
            hx += hexd[data[pos + i] >> 4];
            hx += hexd[data[pos + i] & 15];
        }
        if (!link_.command("neopixel_update oid=" + std::to_string(oid) + " pos=" + std::to_string(pos) +
                               " data=" + hx, nullptr, 300, err))
            return false;
    }
    for (int attempt = 0; attempt < 8; attempt++) {   // neopixel.py retries up to 8 times
        std::vector<Message> msgs;
        if (!link_.command("neopixel_send oid=" + std::to_string(oid), &msgs, 300, err)) return false;
        for (auto& m : msgs)
            if (m.name == "neopixel_result" && m.get("oid") == oid && m.get("success")) return true;
    }
    if (err) *err = "neopixel_send did not succeed after 8 tries";
    return false;
}

}  // namespace ph
