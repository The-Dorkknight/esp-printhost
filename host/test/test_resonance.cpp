// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Resonance test motion and shaper calibration (resonance.h), against Klipper's own code.
#include "check.h"
#include "ph/resonance.h"
#include "shaper_ref.h"

#include <cmath>
#include <cstring>
#include <vector>

using namespace ph;

// The synthetic accelerometer signal of host/tools/shaper_reference.py, sample for sample.
static std::vector<double> lcg_seq(size_t n, uint64_t seed) {
    std::vector<double> out(n);
    uint64_t s = seed;
    for (size_t i = 0; i < n; i++) {
        s = (s * 1103515245ULL + 12345ULL) % 2147483648ULL;
        out[i] = double(s) / 2147483648.0;
    }
    return out;
}

static void synth(std::vector<double> ax[3], double& fs_measured) {
    const double FS = 3200, SECONDS = 12;
    size_t n = size_t(FS * SECONDS);
    std::vector<double> freqs(781);
    for (size_t k = 0; k < freqs.size(); k++) freqs[k] = 5.0 + 0.25 * double(k);
    struct P { double f0, zeta, gain; uint64_t seed; } ps[3] = {{47.3, 0.08, 3000, 11}, {38.1, 0.1, 1500, 23}, {90, 0.2, 200, 37}};
    for (int a = 0; a < 3; a++) {
        std::vector<double> ph = lcg_seq(freqs.size(), ps[a].seed), noise = lcg_seq(n, ps[a].seed + 1000);
        std::vector<double> amp(freqs.size());
        for (size_t k = 0; k < freqs.size(); k++) {
            double r = freqs[k] / ps[a].f0;
            amp[k] = ps[a].gain / std::sqrt((1 - r * r) * (1 - r * r) + (2 * ps[a].zeta * r) * (2 * ps[a].zeta * r)) /
                     std::sqrt(double(freqs.size()));
        }
        ax[a].assign(n, 0.0);
        for (size_t k = 0; k < freqs.size(); k++) {
            double w = 2 * M_PI * freqs[k], p0 = 2 * M_PI * ph[k];
            for (size_t i = 0; i < n; i++) ax[a][i] += amp[k] * std::sin(w * (double(i) / FS) + p0);
        }
        for (size_t i = 0; i < n; i++) ax[a][i] += 20.0 * (noise[i] - 0.5) + (a == 2 ? 9806.65 : 0.0);
    }
    fs_measured = double(n) / (double(n - 1) / FS);   // N / (t_last - t_first)
}

static void test_shaper_calibration_vs_klipper() {
    std::vector<double> ax[3];
    double fs = 0;
    synth(ax, fs);
    PsdAccumulator acc;
    acc.start(3200);
    CHECK(acc.window_size() == 2048);
    for (size_t i = 0; i < ax[0].size(); i++) acc.add(float(ax[0][i]), float(ax[1][i]), float(ax[2][i]));
    CalibrationData d;
    CHECK(acc.finish(fs, d.freqs, d.psd));
    d.normalize_to_frequencies();
    // the spectrum itself
    for (auto& r : PSD_REF) {
        size_t k = size_t(std::lround(r[0] / (fs / 2048)));
        NEAR(d.freqs[k], r[0], 1e-4);
        CHECK(std::fabs(d.psd[3][k] - r[1]) <= 1e-3 * r[1] + 1e-6);
    }
    std::vector<ShaperFit> all;
    ShaperFit best = find_best_shaper(d, 5.0, 1.5 * 135, &all);
    printf("  shaper calibration on the synthetic signal (ours / Klipper's):\n");
    CHECK(all.size() == sizeof SHAPER_REF / sizeof SHAPER_REF[0]);
    for (size_t i = 0; i < all.size() && i < sizeof SHAPER_REF / sizeof SHAPER_REF[0]; i++) {
        const ShaperRef& r = SHAPER_REF[i];
        printf("    %-8s %5.1f / %5.1f Hz  vibrations %.4f / %.4f  smoothing %.4f / %.4f  max_accel %.0f / %.0f\n",
               all[i].name.c_str(), all[i].freq, r.freq, all[i].vibrs, r.vibrs, all[i].smoothing, r.smoothing,
               all[i].max_accel, r.max_accel);
        CHECK(all[i].name == r.name);
        NEAR(all[i].freq, r.freq, 0.41);   // float vs numpy's double: at most two 0.2 Hz steps
        NEAR(all[i].vibrs, r.vibrs, 0.003);
        NEAR(all[i].smoothing, r.smoothing, 0.01 * r.smoothing);
    }
    printf("    best: %s %.1f Hz (Klipper: %s)\n", best.name.c_str(), best.freq, SHAPER_REF_BEST);
    CHECK(best.name == SHAPER_REF_BEST);
}

static void test_resonance_motion() {
    ResonanceParams p;
    auto seq = resonance_test_sequence(p);
    CHECK(seq.size() > 1000);
    NEAR(seq.front().freq, 5.0, 1e-9);
    CHECK(seq.back().freq <= 135.000001 && seq.back().freq > 134);
    ResonanceMotion m = resonance_motion(seq, 3000);
    printf("  resonance test: %.1f s, travel %.2f .. %.2f mm, top speed %.0f mm/s, top accel %.0f mm/s^2\n", m.end_t, m.min_x,
           m.max_x, m.max_v, m.max_a);
    CHECK(m.end_t > 120 && m.end_t < 140);
    CHECK(m.max_x - m.min_x < 40);   // the slow 400 mm/s^2 sweep: about +-18 mm around the test point
    CHECK(m.max_a > 8000 && m.max_a < 8600);   // 60 mm/s^2 per Hz at 135 Hz, plus the 400 sweep
    // continuity: position and speed carry over between pieces
    double worst = 0;
    for (size_t i = 1; i < m.pieces.size(); i++) {
        const auto& a = m.pieces[i - 1];
        double dt = m.pieces[i].t0 - a.t0;
        worst = std::max(worst, std::fabs(a.x0 + a.v0 * dt + 0.5 * a.a * dt * dt - m.pieces[i].x0));
    }
    CHECK(worst < 1e-9);
    NEAR(m.pos(m.end_t + 1), m.end_x, 1e-12);
}

void test_resonance() {
    test_resonance_motion();
    test_shaper_calibration_vs_klipper();
}
