// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
#include "ph/motion.h"
#include <algorithm>
#include <cmath>

namespace ph {

int32_t StepAccumulator::add(Axis a, double delta_mm) {
    target_[a] += delta_mm * spm_[a];
    int64_t want = int64_t(std::llround(target_[a]));
    int32_t out = int32_t(want - committed_[a]);
    committed_[a] = want;
    return out;
}

void StepAccumulator::set_position(Axis a, double mm) {
    target_[a] = mm * spm_[a];
    committed_[a] = int64_t(std::llround(target_[a]));
}

void PressureAdvance::configure(double k, int smooth, double max_e_speed) {
    k_ = k;
    smooth_n_ = std::max(1, smooth);
    max_e_speed_ = max_e_speed;
    hist_.clear();
    hist_sum_ = applied_ = debt_ = 0;
}

double PressureAdvance::process(const In& in) {
    // Only extruding moves with XY/Z travel get advance.
    bool printing = in.xyz_mm > 1e-9 && in.e_mm > 0;
    double target = printing ? k_ * (in.e_mm / in.xyz_mm) * in.v_end : 0.0;

    // Moving average of the advance position -> smooths junction steps.
    hist_.push_back(target);
    hist_sum_ += target;
    if (int(hist_.size()) > smooth_n_) { hist_sum_ -= hist_.front(); hist_.pop_front(); }
    double smoothed = hist_sum_ / double(hist_.size());

    double e = in.e_mm + (smoothed - applied_) + debt_;
    applied_ = smoothed;
    debt_ = 0;

    // Clamp extruder speed; carry the remainder so nothing is lost.
    if (in.dt > 0 && max_e_speed_ > 0) {
        double lim = max_e_speed_ * in.dt;
        if (e > lim) { debt_ = e - lim; e = lim; }
        else if (e < -lim) { debt_ = e + lim; e = -lim; }
    }
    return e;
}

double PressureAdvance::flush() {
    double e = -applied_ + debt_;
    hist_.clear();
    hist_sum_ = applied_ = debt_ = 0;
    return e;
}

}  // namespace ph
