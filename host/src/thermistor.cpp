// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Derived from Klipper's klippy/extras/thermistor.py (GPL-3.0).
#include "ph/thermistor.h"
#include <algorithm>
#include <cmath>

namespace ph {

static const double K = 273.15;

bool Thermistor::setup_beta(double t1, double r1, double beta) {
    if (r1 <= 0 || beta <= 0 || t1 <= -K) return false;
    c1 = 1.0 / (t1 + K) - std::log(r1) / beta;
    c2 = 1.0 / beta;
    c3 = 0.0;
    return true;
}

bool Thermistor::setup_points(double t1, double r1, double t2, double r2, double t3, double r3) {
    if (r1 <= 0 || r2 <= 0 || r3 <= 0 || t1 <= -K || t2 <= -K || t3 <= -K) return false;
    double inv_t1 = 1.0 / (t1 + K), inv_t2 = 1.0 / (t2 + K), inv_t3 = 1.0 / (t3 + K);
    double ln_r1 = std::log(r1), ln_r2 = std::log(r2), ln_r3 = std::log(r3);
    double ln3_r1 = ln_r1 * ln_r1 * ln_r1, ln3_r2 = ln_r2 * ln_r2 * ln_r2, ln3_r3 = ln_r3 * ln_r3 * ln_r3;
    double inv_t12 = inv_t1 - inv_t2, inv_t13 = inv_t1 - inv_t3;
    double ln_r12 = ln_r1 - ln_r2, ln_r13 = ln_r1 - ln_r3;
    double ln3_r12 = ln3_r1 - ln3_r2, ln3_r13 = ln3_r1 - ln3_r3;
    if (ln_r13 == 0 || ln_r12 == 0) return false;
    double den = ln3_r12 - ln3_r13 * ln_r12 / ln_r13;
    if (den == 0) return false;
    c3 = (inv_t12 - inv_t13 * ln_r12 / ln_r13) / den;
    if (c3 <= 0.0) {   // not a physical fit: use a beta model through the first two points
        return setup_beta(t1, r1, ln_r12 / inv_t12);
    }
    c2 = (inv_t12 - c3 * ln3_r12) / ln_r12;
    c1 = inv_t1 - c2 * ln_r1 - c3 * ln3_r1;
    return true;
}

double Thermistor::calc_temp(double adc) const {
    adc = std::max(0.00001, std::min(0.99999, adc));
    double r = pullup * adc / (1.0 - adc);
    double rr = r - inline_resistor;
    if (rr <= 0) rr = 1e-6;
    double ln_r = std::log(rr);
    double inv_t = c1 + c2 * ln_r + c3 * ln_r * ln_r * ln_r;
    return 1.0 / inv_t - K;
}

double Thermistor::calc_adc(double temp_c) const {
    if (temp_c <= -K) return 0.0;
    double inv_t = 1.0 / (temp_c + K);
    double ln_r;
    if (c3 != 0.0) {
        double y = (c1 - inv_t) / (2.0 * c3);
        double x = std::sqrt(std::pow(c2 / (3.0 * c3), 3) + y * y);
        ln_r = std::cbrt(x - y) - std::cbrt(x + y);
    } else {
        ln_r = (inv_t - c1) / c2;
    }
    double r = std::exp(ln_r) + inline_resistor;
    return r / (pullup + r);
}

bool thermistor_preset(const std::string& name, Thermistor& t) {
    Thermistor n;   // default pullup 4700
    bool ok;
    if (name == "EPCOS 100K B57560G104F") ok = n.setup_points(25, 100000, 150, 1641.9, 250, 226.15);
    else if (name == "ATC Semitec 104GT-2") ok = n.setup_points(20, 126800, 150, 1360, 300, 80.65);
    else if (name == "Generic 3950") ok = n.setup_points(25, 100000, 150, 1770, 250, 230);
    else if (name == "NTC 100K MGB18-104F39050L32") ok = n.setup_beta(25, 100000, 4100);
    else return false;
    if (!ok) return false;
    t = n;
    return true;
}

}  // namespace ph
