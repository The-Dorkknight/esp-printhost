// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Resonance test and input shaper calibration (2026-10-24), ported from Klipper v0.13.0:
//   klippy/extras/resonance_tester.py   the test: a vibration whose frequency sweeps 5 -> 135 Hz at
//                                       1 Hz/s (accel 60 mm/s^2 per Hz) on top of a slow sweep
//   klippy/extras/shaper_calibrate.py   Welch power spectrum of the accelerometer, then the shaper
//                                       and frequency that leave the least vibration with the least
//                                       smoothing (find_best_shaper)
// The spectrum is built while the samples arrive (streaming Welch), so no recording is kept: a few
// tens of KB instead of megabytes. Checked against Klipper's own shaper_calibrate.py on the same
// synthetic signal (host/tools/shaper_reference.py, test_resonance.cpp).
#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ph {

struct ResonanceParams {   // [resonance_tester] defaults
    double min_freq = 5, max_freq = 135, accel_per_hz = 60, hz_per_sec = 1;
    double sweeping_accel = 400, sweeping_period = 1.2;
};
struct TestPoint {
    double t, accel, freq;   // the acceleration holds from the previous point's time until t
};
std::vector<TestPoint> resonance_test_sequence(const ResonanceParams& p);

// The test's motion along one axis (resonance_tester.py run_test): constant acceleration pieces,
// then a stop at `end_accel`. Positions relative to the start (mm).
struct ResonanceMotion {
    struct Piece { double t0, x0, v0, a; };
    std::vector<Piece> pieces;
    double end_t = 0, end_x = 0, min_x = 0, max_x = 0, max_v = 0, max_a = 0;
    double pos(double t, double* vel = nullptr) const;
    size_t piece_at(double t) const;
};
ResonanceMotion resonance_motion(const std::vector<TestPoint>& seq, double end_accel);

// Welch power spectral density of three axes, as shaper_calibrate.py _psd(): window 2^k samples of
// about 0.5 s, Kaiser (beta 6), half overlap, mean removed per window.
class PsdAccumulator {
public:
    void start(double nominal_rate);   // picks the window size
    void add(float x, float y, float z);
    size_t samples() const { return n_; }
    size_t windows() const { return windows_; }
    int window_size() const { return m_; }
    // measured_rate: samples / (last time - first time), as Klipper computes it
    bool finish(double measured_rate, std::vector<double>& freqs, std::vector<double> psd[4]);   // psd[3] = sum
    size_t bytes() const;

private:
    void window();
    int m_ = 0;
    std::vector<float> ring_[3], win_, re_, im_, cos_, sin_;
    std::vector<double> acc_[3];
    size_t n_ = 0, since_ = 0, windows_ = 0;
    double wscale_ = 0;
};

struct CalibrationData {
    std::vector<double> freqs, psd[4];   // x, y, z, sum
    void normalize_to_frequencies();     // shaper_calibrate.py, called before the fit
};

struct ShaperFit {
    std::string name;
    double freq = 0, vibrs = 0, smoothing = 0, score = 0, max_accel = 0;
};
// shaper_calibrate.py find_best_shaper (shapers zv mzv ei 2hump_ei 3hump_ei; scv = square corner
// velocity; max_freq 1.5 x the test's top frequency). progress(i, n) is called between shapers.
ShaperFit find_best_shaper(const CalibrationData& d, double scv, double max_freq, std::vector<ShaperFit>* all = nullptr,
                           double max_smoothing = 0, void (*progress)(int, int) = nullptr);
double shaper_smoothing(const std::vector<double>& A, const std::vector<double>& T, double accel, double scv);
double shaper_max_accel(const std::vector<double>& A, const std::vector<double>& T, double scv);

}  // namespace ph
