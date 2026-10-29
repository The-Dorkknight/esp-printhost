// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// See printbench.h.
#include "ph/printbench.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace ph {

// ---------------------------------------------------------------- the benchmark file

namespace {
const double PI = 3.14159265358979323846;
const double E_LINE = 0.4 * 0.2 / (PI * 0.875 * 0.875);   // mm of 1.75 mm filament per mm of 0.4 x 0.2 line

std::string arg(const std::string& cmd, const char* key) {
    std::string k = std::string(" ") + key + "=";
    size_t p = cmd.find(k);
    if (p == std::string::npos) return "";
    p += k.size();
    size_t e = cmd.find(' ', p);
    return cmd.substr(p, e == std::string::npos ? std::string::npos : e - p);
}
}  // namespace

// A move to (x, y), relative to the start of the file, written as a relative move rounded to 0.001 mm;
// the rounded position is kept, so nothing drifts over thousands of moves.
void BenchGcode::go(double x, double y, double e_per_mm, int feed) {
    long dx = std::lround((x - x_) * 1000), dy = std::lround((y - y_) * 1000);
    if (!dx && !dy) return;
    x_ += dx / 1000.0;
    y_ += dy / 1000.0;
    double len = std::sqrt(double(dx) * dx + double(dy) * dy) / 1000.0;
    char b[96];
    if (e_per_mm > 0)
        snprintf(b, sizeof b, "G1 X%.3f Y%.3f E%.5f F%d", dx / 1000.0, dy / 1000.0, len * e_per_mm, feed);
    else
        snprintf(b, sizeof b, "G0 X%.3f Y%.3f F%d", dx / 1000.0, dy / 1000.0, feed);
    q_.push_back(b);
}

void BenchGcode::layer(int n) {
    char b[64];
    snprintf(b, sizeof b, ";LAYER:%d", n);
    q_.push_back(b);
    q_.push_back("G1 Z0.200 F600");
    // two perimeters, 0.5 mm chords, 100 mm/s
    for (double r : {18.0, 17.6}) {
        int k = int(2 * PI * r / 0.5);
        go(r, 0, 0, 18000);
        q_.push_back("G1 E0.80000 F2400");
        for (int i = 1; i <= k; i++) go(r * std::cos(2 * PI * i / k), r * std::sin(2 * PI * i / k), E_LINE, 6000);
        q_.push_back("G1 E-0.80000 F2400");
    }
    // a fine wiggle: 300 moves of about 0.25 mm at 50 mm/s (small detail: text, a gyroid, a logo)
    go(-12, -15, 0, 18000);
    q_.push_back("G1 E0.80000 F2400");
    for (int i = 1; i <= 300; i++) go(-12 + i * 0.08, -15 + 1.2 * std::sin(i * 0.35), E_LINE, 3000);
    q_.push_back("G1 E-0.80000 F2400");
    // zig-zag infill inside the circle, 1.2 mm apart, 250 mm/s, crossing direction each layer
    const double h = 12.0;
    bool along_x = n % 2 == 0;
    auto pt = [&](double u, double v) { return along_x ? std::make_pair(u, v) : std::make_pair(v, u); };
    auto p0 = pt(-h, -h);
    go(p0.first, p0.second, 0, 18000);
    q_.push_back("G1 E0.80000 F2400");
    int lines = int(2 * h / 1.2);
    for (int i = 0; i <= lines; i++) {
        double v = -h + i * 1.2, u0 = (i % 2) ? h : -h, u1 = -u0;
        auto a = pt(u0, v), c = pt(u1, v);
        go(a.first, a.second, E_LINE, 15000);
        go(c.first, c.second, E_LINE, 15000);
    }
    q_.push_back("G1 E-0.80000 F2400");
}

bool BenchGcode::next(std::string& line) {
    while (q_.empty()) {
        if (done_) return false;
        if (n_ < 0) {
            q_.push_back("; Skipper Flipper benchmark (SF_BENCH, 2026-10-27): a print-like run through the motion");
            q_.push_back("; pipeline. Relative moves from wherever the head is; no heating, no homing.");
            q_.push_back("; Made by BenchGcode (host/src/printbench.cpp).");
            char b[48];
            snprintf(b, sizeof b, ";LAYER_COUNT:%d", layers_);
            q_.push_back(b);
            q_.push_back("M83");
            q_.push_back("G91");
            q_.push_back("M204 S5000");
            n_ = 0;
        } else if (n_ < layers_) {
            layer(n_++);
        } else {
            go(0, 0, 0, 18000);
            q_.push_back("G90");
            done_ = true;
        }
    }
    line = q_.front();
    q_.pop_front();
    return true;
}

// ---------------------------------------------------------------- a board that is not there

MachineIO FakeBoard::io(std::function<double()> now) {
    MachineIO io;
    io.send = [this](const std::vector<std::string>& c, std::string* err) {
        for (auto& s : c) {   // the same encoding work as for the real board, then dropped
            Bytes b;
            if (!mp_.encode(s, b, err)) return false;
            bytes_ += b.size();
        }
        return true;
    };
    io.query = [this](const std::string& cmd, const std::string& resp, int oid, Message& out, int) {
        out = Message();
        out.name = resp;
        out.params["oid"].i = oid;
        if (resp == "stepper_position" || resp == "endstop_state") return true;
        if (resp != "tmcuart_response") return false;
        // a TMC2209 that keeps what is written to it (IFCNT counts the writes)
        std::string hx = arg(cmd, "write");
        if (hx.rfind("hex:", 0) == 0) hx = hx.substr(4);
        std::vector<uint8_t> raw, d;
        for (size_t i = 0; i + 1 < hx.size(); i += 2) raw.push_back(uint8_t(strtol(hx.substr(i, 2).c_str(), nullptr, 16)));
        for (size_t bit = 0; bit + 10 <= raw.size() * 8; bit += 10) {
            uint8_t v = 0;
            for (int i = 0; i < 8; i++)
                if (raw[(bit + 1 + i) / 8] & (1u << ((bit + 1 + i) % 8))) v |= uint8_t(1u << i);
            d.push_back(v);
        }
        if (d.size() < 3) return false;
        int addr = d[1];
        auto& p = out.params["read"];
        p.is_bytes = true;
        if (d.size() == 8) {
            regs_[addr][d[2] & 0x7f] = (uint32_t(d[3]) << 24) | (uint32_t(d[4]) << 16) | (uint32_t(d[5]) << 8) | d[6];
            ifcnt_[addr]++;
            return true;
        }
        int reg = d[2];
        uint32_t v = reg == 0x06 ? 0x21000040u : reg == 0x02 ? ifcnt_[addr] : regs_[addr][reg];
        auto e = tmc_encode_write(0x05, 0xff, uint8_t(reg), v);
        p.b.assign(e.begin(), e.end());
        return true;
    };
    io.clock_at = [](double t) { return uint64_t(t * 12e6); };
    io.neopixel_write = [](int, const std::vector<uint8_t>&, std::string*) { return true; };
    io.now = now;
    io.board_kind = "no printer board (ESP32 only: commands encoded, not sent)";
    io.fake_board = true;
    return io;
}

}  // namespace ph
