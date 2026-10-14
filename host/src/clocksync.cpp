// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
#include "ph/clocksync.h"

namespace ph {

static const double DECAY = 1.0 / 30.0;   // same weighting as Klipper's clocksync.py
static const double RTT_AGE = 0.000010 / (60.0 * 60.0);   // min RTT may grow 10 us per hour

void ClockSync::reset(double nominal_hz, uint64_t clock64, double host_time_s) {
    nominal_ = nominal_hz;
    freq_ = nominal_hz;
    last_clock_ = clock64;
    time_avg_ = host_time_s;
    clock_avg_ = double(clock64);
    time_var_ = clock_cov_ = 0;
    min_rtt_ = 1e9;
    rtt_age_ = 0;
    samples_ = rejected_ = 0;
    started_ = true;
}

uint64_t ClockSync::extend(uint32_t clock32) const {
    // the nearest 64-bit value to the last clock (board clocks wrap 32 bits every few minutes)
    return last_clock_ + uint64_t(int64_t(int32_t(clock32 - uint32_t(last_clock_))));
}

bool ClockSync::add_sample(double sent_s, double recv_s, uint32_t clock32) {
    if (!started_) return false;
    uint64_t clock = extend(clock32);
    last_clock_ = clock;
    double rtt = recv_s - sent_s;
    // Klipper keeps a slowly ageing minimum round trip and only trusts samples close to it.
    double aged_min = min_rtt_ + (sent_s - rtt_age_) * RTT_AGE;
    if (rtt < aged_min || min_rtt_ >= 1e8) {
        min_rtt_ = rtt;
        rtt_age_ = sent_s;
    }
    if (samples_ >= 3 && rtt > min_rtt_ * 2.0 + 0.002) {
        rejected_++;
        return false;
    }
    // The board read its clock somewhere inside the round trip: use the midpoint.
    double t = sent_s + rtt / 2.0;
    double diff_t = t - time_avg_;
    double diff_c = double(clock) - clock_avg_;
    time_avg_ += DECAY * diff_t;
    time_var_ = (1.0 - DECAY) * (time_var_ + diff_t * diff_t * DECAY);
    clock_avg_ += DECAY * diff_c;
    clock_cov_ = (1.0 - DECAY) * (clock_cov_ + diff_t * diff_c * DECAY);
    samples_++;
    if (time_var_ > 0 && samples_ >= 3) freq_ = clock_cov_ / time_var_;
    return true;
}

uint64_t ClockSync::clock_at(double host_time_s) const {
    double c = clock_avg_ + (host_time_s - time_avg_) * freq_;
    return c < 0 ? 0 : uint64_t(c);
}

double ClockSync::time_of(uint64_t clock) const {
    return freq_ > 0 ? time_avg_ + (double(clock) - clock_avg_) / freq_ : 0;
}

}  // namespace ph
