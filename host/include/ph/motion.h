// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Motion helpers between the planner (FluidNC/grbl segment prep) and the link.
#pragma once
#include <cstdint>
#include <deque>

namespace ph {

enum Axis { AX_X = 0, AX_Y, AX_Z, AX_E, NUM_AXES };

// A constant-speed piece of motion from the planner; the toolhead turns it into step times for
// Klipper's queue_step. Rules: 1 <= n <= 2^30 and 2*|steps[axis]| <= n.
struct Segment {
    uint32_t n = 0;                 // duration in planner ticks (1/40000 s by default)
    int32_t  steps[NUM_AXES] = {};  // signed step counts
    uint8_t  flags = 0;             // SEG_SYNC | SEG_LAST
    uint16_t tag = 0;
};
enum : uint8_t { SEG_SYNC = 1, SEG_LAST = 2, SEG_RESERVED_PA = 4 };

// Converts mm moves into integer steps with no cumulative rounding drift.
class StepAccumulator {
public:
    void set_steps_per_mm(Axis a, double spm) { spm_[a] = spm; }
    // Move axis by delta mm; returns the whole steps to emit now.
    int32_t add(Axis a, double delta_mm);
    // Set logical position (G92 / after homing) without emitting steps.
    void set_position(Axis a, double mm);
    double position_mm(Axis a) const { return target_[a] / spm_[a]; }
    int64_t committed_steps(Axis a) const { return committed_[a]; }
private:
    double spm_[NUM_AXES] = {80, 80, 400, 93};
    double target_[NUM_AXES] = {};     // exact position, in steps
    int64_t committed_[NUM_AXES] = {}; // steps already emitted
};

// "Pseudo" pressure advance, Klipper-style: extra filament = K * e_ratio * v.
// Emitting only the *change* of that term each segment means it sums to zero
// over any move that starts and ends at rest, so total extrusion is exact.
class PressureAdvance {
public:
    struct In {
        double xyz_mm;     // XYZ distance of this segment
        double e_mm;       // planner's (un-advanced) E distance for this segment
        double v_end;      // XYZ speed at end of segment, mm/s
        double dt;         // segment duration, s
    };
    void configure(double k, int smooth_segments = 4, double max_e_speed = 60.0);
    double k() const { return k_; }
    void set_k(double k) { k_ = k; }   // M900 K
    // Returns E mm to actually emit for this segment (may be negative).
    double process(const In& in);
    // Drop the current advance (e.g. at print end). Returns E mm to emit.
    double flush();
    double pending_mm() const { return debt_; }
private:
    double k_ = 0, max_e_speed_ = 60;
    int smooth_n_ = 4;
    std::deque<double> hist_;
    double hist_sum_ = 0, applied_ = 0, debt_ = 0;
};

}  // namespace ph
