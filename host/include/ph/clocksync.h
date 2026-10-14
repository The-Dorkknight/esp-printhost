// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Board clock tracking, after Klipper's klippy/clocksync.py: periodic get_clock samples, a filter
// on round-trip time, and an exponentially weighted linear regression of board clock against
// host time. Gives the board's real clock frequency and the board clock at any host time.
#pragma once
#include <cstdint>

namespace ph {

class ClockSync {
public:
    explicit ClockSync(double nominal_hz = 0) : nominal_(nominal_hz) {}

    // Start from a full 64-bit board clock (get_uptime: high << 32 | clock) seen at host_time_s.
    void reset(double nominal_hz, uint64_t clock64, double host_time_s);
    // One get_clock round trip: host time just before sending, just after the reply, and the
    // reply's 32-bit clock. Returns false if the sample was rejected (slow round trip).
    bool add_sample(double sent_s, double recv_s, uint32_t clock32);

    bool ready() const { return samples_ >= 3; }
    double frequency() const { return freq_; }               // measured board ticks per second
    double drift_ppm() const { return nominal_ > 0 ? (freq_ / nominal_ - 1.0) * 1e6 : 0; }
    double min_rtt() const { return min_rtt_; }
    uint32_t samples() const { return samples_; }
    uint32_t rejected() const { return rejected_; }
    uint64_t last_clock() const { return last_clock_; }
    // Board clock (64-bit) expected at a host time, and the reverse.
    uint64_t clock_at(double host_time_s) const;
    double time_of(uint64_t clock) const;
    // Extend a 32-bit clock seen near the last sample to 64 bits.
    uint64_t extend(uint32_t clock32) const;

private:
    double nominal_ = 0, freq_ = 0;
    double time_avg_ = 0, time_var_ = 0, clock_avg_ = 0, clock_cov_ = 0;
    double min_rtt_ = 1e9, rtt_age_ = 0;
    uint64_t last_clock_ = 0;
    uint32_t samples_ = 0, rejected_ = 0;
    bool started_ = false;
};

}  // namespace ph
