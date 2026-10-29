// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// How much processor the motion pipeline needs (2026-10-24): planner -> (input shaping / pressure
// advance) -> step times -> queue_step compression, on a fixed print-like path, timed with the
// caller's clock. Runs on the PC (tests), under QEMU at start-up (instruction-counted) and on the
// ESP32 itself (Test bench, "Motion benchmark"), so the three can be compared.
#pragma once
#include "ph/shaping.h"

#include <cstdint>
#include <functional>
#include <string>

namespace ph {

struct BenchResult {
    double sim_s = 0;         // seconds of printer motion the path takes
    uint64_t steps = 0;       // steps generated (all axes)
    uint64_t commands = 0;    // queue_step commands they compressed into
    double plan_us = 0;       // planner + shaping + step times
    double compress_us = 0;   // compression into queue_step
    double us_per_step() const { return steps ? (plan_us + compress_us) / double(steps) : 0; }
    double core_load() const { return sim_s > 0 ? (plan_us + compress_us) * 1e-6 / sim_s : 0; }   // 1.0 = a whole core
    std::string text(const char* label) const;
};

// The path: three circles (r 20 mm, 2 degree chords) and 40 zig-zag lines, extruding, at `speed`
// mm/s and 3000 mm/s^2, Ender 3 steps/mm (80 / 80 / 400 / 93). `shaping` may be null (none).
// short: one circle and 10 lines (about a quarter: for QEMU's slow instruction-counted runs).
BenchResult motion_bench(double speed, const ShapingConfig* shaping, const std::function<uint64_t()>& now_us,
                         bool short_path = false);
// The Tina 2 Z homing move of 2026-10-20 (49,200 steps at 4 mm/s, compressed in one go): on the
// ESP32-S3 it took about 0.4 s, the one timing measured on hardware so far.
BenchResult homing_bench(const std::function<uint64_t()>& now_us);
BenchResult homing_bench_legacy(const std::function<uint64_t()>& now_us);   // the old compressor (QEMU / PC only)
// Both, with and without shaping, as text lines.
std::string motion_bench_report(const std::function<uint64_t()>& now_us, bool short_path = false);

}  // namespace ph
