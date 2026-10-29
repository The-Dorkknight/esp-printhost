// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// The print benchmark (2026-10-27, SF_BENCH):
//  - BenchGcode writes the benchmark file (docs/bench/sf-bench.gcode is its output): a print-like
//    cylinder, layer after layer: two perimeters in 0.5 mm chords, a fine wiggle of 0.25 mm moves
//    (like text or a gyroid), zig-zag infill at 250 mm/s, retractions and travel. Everything relative
//    (G91, M83), so it runs from wherever the head is; no heaters, no homing. About 85 s of motion.
//  - FakeBoard: a MachineIO with no printer board behind it, for the "ESP32 only" benchmark: commands
//    are encoded as for the real board (so that work is counted) and dropped; the TMC2209 answers
//    like a real one; the clock is the ESP32's own at 12 MHz.
#pragma once
#include "ph/machine.h"
#include "ph/msgproto.h"

#include <cstdint>
#include <deque>
#include <map>
#include <string>

namespace ph {

class BenchGcode {
public:
    explicit BenchGcode(int layers = 12) : layers_(layers) {}
    // The next line (without the newline); false at the end.
    bool next(std::string& line);
private:
    void layer(int n);
    void go(double x, double y, double e_per_mm, int feed);
    int layers_, n_ = -1;
    bool done_ = false;
    std::deque<std::string> q_;
    double x_ = 0, y_ = 0;   // where the generated moves have taken the head (relative to the start)
};

class FakeBoard {
public:
    explicit FakeBoard(const MsgProto& mp) : mp_(mp) {}
    // The IO for Machine. now(): host seconds (the caller's clock).
    MachineIO io(std::function<double()> now);
    uint64_t bytes() const { return bytes_; }   // encoded command bytes "sent"
private:
    const MsgProto& mp_;
    uint64_t bytes_ = 0;
    std::map<int, std::map<int, uint32_t>> regs_;
    std::map<int, uint32_t> ifcnt_;
};

}  // namespace ph
