// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// See shaping.h. The shaper formulas are Klipper's (shaper_defs.py, Dmitry Butyugin, GPLv3).
#include "ph/shaping.h"

#include <algorithm>
#include <cmath>

namespace ph {

const char* const SHAPER_TYPES[6] = {"zv", "mzv", "zvd", "ei", "2hump_ei", "3hump_ei"};

double shaper_min_freq(const std::string& t) {
    if (t == "zv") return 21;
    if (t == "mzv") return 23;
    if (t == "zvd" || t == "ei") return 29;
    if (t == "2hump_ei") return 39;
    if (t == "3hump_ei") return 48;
    return 0;
}

bool shaper_def(const std::string& type, double f, double zeta, std::vector<double>& A, std::vector<double>& T) {
    A.clear(), T.clear();
    if (!(f > 0) || zeta < 0 || zeta >= 1) return false;
    const double v_tol = 1.0 / SHAPER_VIBRATION_REDUCTION;
    const double df = std::sqrt(1.0 - zeta * zeta);
    const double K = std::exp(-zeta * M_PI / df);
    const double t_d = 1.0 / (f * df);
    if (type == "zv") {
        A = {1.0, K};
        T = {0.0, 0.5 * t_d};
    } else if (type == "zvd") {
        A = {1.0, 2.0 * K, K * K};
        T = {0.0, 0.5 * t_d, t_d};
    } else if (type == "mzv") {
        double Km = std::exp(-0.75 * zeta * M_PI / df);
        double a1 = 1.0 - 1.0 / std::sqrt(2.0), a2 = (std::sqrt(2.0) - 1.0) * Km, a3 = a1 * Km * Km;
        A = {a1, a2, a3};
        T = {0.0, 0.375 * t_d, 0.75 * t_d};
    } else if (type == "ei") {
        double a1 = 0.25 * (1.0 + v_tol), a2 = 0.5 * (1.0 - v_tol) * K, a3 = a1 * K * K;
        A = {a1, a2, a3};
        T = {0.0, 0.5 * t_d, t_d};
    } else if (type == "2hump_ei") {
        double V2 = v_tol * v_tol;
        double X = std::pow(V2 * (std::sqrt(1.0 - V2) + 1.0), 1.0 / 3.0);
        double a1 = (3.0 * X * X + 2.0 * X + 3.0 * V2) / (16.0 * X);
        double a2 = (0.5 - a1) * K, a3 = a2 * K, a4 = a1 * K * K * K;
        A = {a1, a2, a3, a4};
        T = {0.0, 0.5 * t_d, t_d, 1.5 * t_d};
    } else if (type == "3hump_ei") {
        double K2 = K * K;
        double a1 = 0.0625 * (1.0 + 3.0 * v_tol + 2.0 * std::sqrt(2.0 * (v_tol + 1.0) * v_tol));
        double a2 = 0.25 * (1.0 - v_tol) * K;
        double a3 = (0.5 * (1.0 + v_tol) - 2.0 * a1) * K2;
        double a4 = a2 * K2, a5 = a1 * K2 * K2;
        A = {a1, a2, a3, a4, a5};
        T = {0.0, 0.5 * t_d, t_d, 1.5 * t_d, 2.0 * t_d};
    } else {
        return false;
    }
    return true;
}

ShaperPulses shaper_pulses(const std::vector<double>& A, const std::vector<double>& T) {
    ShaperPulses sp;
    int n = int(std::min<size_t>(A.size(), 5));
    if (n == 0 || T.size() < size_t(n)) return sp;
    double sum = 0;
    for (int i = 0; i < n; i++) sum += A[size_t(i)];
    for (int i = 0; i < n; i++) {   // kin_shaper.c init_shaper: reversed, normalised
        sp.a[n - i - 1] = A[size_t(i)] / sum;
        sp.t[n - i - 1] = -T[size_t(i)];
    }
    sp.n = n;
    double ts = 0;   // shift_pulses: centre them
    for (int i = 0; i < n; i++) ts += sp.a[i] * sp.t[i];
    for (int i = 0; i < n; i++) sp.t[i] -= ts;
    return sp;
}

bool ShapingConfig::same_except_pa(const ShapingConfig& o) const {
    auto eq = [](const ShaperPulses& a, const ShaperPulses& b) {
        if (a.n != b.n) return false;
        for (int i = 0; i < a.n; i++)
            if (a.a[i] != b.a[i] || a.t[i] != b.t[i]) return false;
        return true;
    };
    return eq(x, o.x) && eq(y, o.y) && pa_on == o.pa_on && pa_smooth == o.pa_smooth;
}

void MotionShaper::configure(const ShapingConfig& c) {
    cfg_ = c;
    lead_ = tail_ = 0;
    for (const ShaperPulses* s : {&cfg_.x, &cfg_.y})
        for (int i = 0; i < s->n; i++) {
            lead_ = std::max(lead_, s->t[i]);
            tail_ = std::max(tail_, -s->t[i]);
        }
    if (cfg_.pa_on && cfg_.pa_smooth > 0) {
        lead_ = std::max(lead_, 0.5 * cfg_.pa_smooth);
        tail_ = std::max(tail_, 0.5 * cfg_.pa_smooth);
    }
}

void MotionShaper::reset() {
    knots_.clear();
    for (int a = 0; a < NUM_AXES; a++) emitted_[a] = cum_[a];
    out_end_ = 0;
}

double MotionShaper::pos(int axis, double t) const {
    if (knots_.empty()) return double(cum_[axis]);
    if (t <= knots_.front().t) return knots_.front().p[axis];
    if (t >= knots_.back().t) return knots_.back().p[axis];
    size_t lo = 0, hi = knots_.size() - 1;   // knots_[lo].t <= t < knots_[hi].t
    while (hi - lo > 1) {
        size_t mid = (lo + hi) / 2;
        if (knots_[mid].t <= t) lo = mid;
        else hi = mid;
    }
    const Knot &k0 = knots_[lo], &k1 = knots_[hi];
    double dt = k1.t - k0.t;
    if (dt <= 0) return k1.p[axis];
    return k0.p[axis] + (k1.p[axis] - k0.p[axis]) * (t - k0.t) / dt;
}

// kin_extruder.c: smooth(tau) = 1/h^2 * integral over [tau-h, tau+h] of (E(x) + K * v(x)) * (h - |tau - x|),
// with K only where the move may use pressure advance. Here E is linear between knots, so each
// piece integrates exactly; u = x - tau keeps the numbers small.
double MotionShaper::pa_smooth_pos(double tau) const {
    const double h = 0.5 * cfg_.pa_smooth;
    const int E = AX_E;
    if (h <= 0) return pos(E, tau);
    auto piece = [&](double u0, double u1, double q0, double v) {   // q0 = Q at u = 0 (extended line)
        double res = 0;
        if (u0 < 0) {   // left part: weight h + u
            double a = u0, b = std::min(u1, 0.0);
            res += q0 * h * (b - a) + (q0 + v * h) * (b * b - a * a) / 2 + v * (b * b * b - a * a * a) / 3;
        }
        if (u1 > 0) {   // right part: weight h - u
            double a = std::max(u0, 0.0), b = u1;
            res += q0 * h * (b - a) + (-q0 + v * h) * (b * b - a * a) / 2 - v * (b * b * b - a * a * a) / 3;
        }
        return res;
    };
    double lo = tau - h, hi = tau + h, area = 0;
    if (knots_.empty()) return pos(E, tau);
    // held before the first knot / after the last (at rest there)
    const Knot &f = knots_.front(), &l = knots_.back();
    if (lo < f.t) area += piece(lo - tau, std::min(hi, f.t) - tau, f.p[E], 0.0);
    if (hi > l.t) area += piece(std::max(lo, l.t) - tau, hi - tau, l.p[E], 0.0);
    // first knot whose interval can overlap [lo, hi]
    size_t i = 0, n = knots_.size();
    {
        size_t a = 0, b = n - 1;
        if (knots_[0].t < lo) {
            while (b - a > 1) {
                size_t m = (a + b) / 2;
                if (knots_[m].t <= lo) a = m;
                else b = m;
            }
            i = a;
        }
    }
    for (; i + 1 < n && knots_[i].t < hi; i++) {
        const Knot &k0 = knots_[i], &k1 = knots_[i + 1];
        double dt = k1.t - k0.t;
        if (dt <= 0) continue;
        double s = std::max(k0.t, lo), e = std::min(k1.t, hi);
        if (e <= s) continue;
        double v = (k1.p[E] - k0.p[E]) / dt;
        double q0 = k0.p[E] + v * (tau - k0.t) + k0.k * v;
        area += piece(s - tau, e - tau, q0, v);
    }
    return area / (h * h);
}

double MotionShaper::out_pos(int axis, double tau) const {
    const ShaperPulses* sp = axis == 0 ? &cfg_.x : axis == 1 ? &cfg_.y : nullptr;
    if (sp && sp->n) {
        double r = 0;
        for (int i = 0; i < sp->n; i++) r += sp->a[i] * pos(axis, tau + sp->t[i]);
        return r;
    }
    if (axis == AX_E && cfg_.pa_on) return pa_smooth_pos(tau);
    return pos(axis, tau);
}

void MotionShaper::emit_until(double limit) {
    while (out_t_ + grid_ <= limit + 1e-12) {
        double t1 = out_t_ + grid_;
        int32_t st[NUM_AXES];
        bool any = false;
        for (int a = 0; a < NUM_AXES; a++) {
            int64_t target = int64_t(std::llround(out_pos(a, t1)));
            st[a] = int32_t(target - emitted_[a]);
            emitted_[a] = target;
            any |= st[a] != 0;
        }
        if (sink_ && any) sink_(out_t_, grid_, st);
        out_count_++;
        out_t_ = t1;
        out_end_ = t1;
    }
}

void MotionShaper::trim() {
    // keep the knots the oldest pending output can still reach (tail, plus the PA window)
    double keep_from = out_t_ - tail_ - 1e-6;
    while (knots_.size() > 2 && knots_[1].t < keep_from) knots_.pop_front();
}

void MotionShaper::push(double t0, double dur, const int32_t steps[NUM_AXES], bool last) {
    if (knots_.empty() || t0 > knots_.back().t + 1e-9) {
        // starting from rest (or after a gap the planner left): the position holds until t0
        // (at rest emitted_ == cum_; the toolhead leaves lead + tail between motions, so out_end_
        // never reaches past t0 - lead: if it ever did, the steps come late rather than get lost)
        if (knots_.empty()) out_t_ = std::max(t0 - lead_, out_end_);
        Knot k;
        k.t = t0;
        for (int a = 0; a < NUM_AXES; a++) k.p[a] = double(cum_[a]);
        k.k = 0;
        knots_.push_back(k);
    }
    // the interval starting at the last knot: pressure advance for XY moves that extrude (Klipper:
    // extruder.py, "can_pressure_advance" = XY travel and positive extrusion)
    knots_.back().k = ((steps[0] || steps[1]) && steps[AX_E] > 0) ? cfg_.pa : 0.0;
    Knot k;
    k.t = t0 + dur;
    for (int a = 0; a < NUM_AXES; a++) {
        cum_[a] += steps[a];
        k.p[a] = double(cum_[a]);
    }
    k.k = 0;
    knots_.push_back(k);
    if (last) {
        emit_until(k.t + tail_);
        if (out_t_ < k.t + tail_) emit_until(out_t_ + grid_);   // the grid may not land exactly
        knots_.clear();
        return;
    }
    emit_until(k.t - lead_);
    trim();
}

}  // namespace ph
