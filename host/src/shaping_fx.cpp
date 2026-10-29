// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// MotionShaperFx (2026-10-27): MotionShaper (shaping.cpp) in whole numbers for the int32/int64 test
// build. Same knots, same pulses, same pressure-advance integral; see shaping.h for the formats.
// test_shaping.cpp checks it against MotionShaper step for step.
#include "ph/shaping.h"

#include <algorithm>
#include <cmath>

namespace ph {

namespace {
// int64 -> float without the software 64-bit conversion: the callers only pass small values
inline float f32(int64_t v) { return float(int32_t(v)); }
}  // namespace

void MotionShaperFx::configure(const ShapingConfig& c) {
    cfg_ = c;
    lead_us_ = tail_us_ = 0;
    auto quant = [&](const ShaperPulses& s, Pulses& q) {
        q = Pulses();
        q.n = s.n;
        int32_t sum = 0;
        int big = 0;
        for (int i = 0; i < s.n; i++) {
            q.a[i] = int32_t(std::lround(s.a[i] * 65536.0));
            q.t[i] = int32_t(to_us(s.t[i]));
            sum += q.a[i];
            if (q.a[i] > q.a[big]) big = i;
            lead_us_ = std::max<int64_t>(lead_us_, q.t[i]);
            tail_us_ = std::max<int64_t>(tail_us_, -q.t[i]);
        }
        if (s.n) q.a[big] += 65536 - sum;   // exactly 1 in all: a constant position passes unchanged
    };
    quant(cfg_.x, px_);
    quant(cfg_.y, py_);
    pa_h_ = 0;
    if (cfg_.pa_on && cfg_.pa_smooth > 0) {
        int64_t h = to_us(0.5 * cfg_.pa_smooth);
        pa_h_ = float(h) * 1e-6f;
        lead_us_ = std::max(lead_us_, h);
        tail_us_ = std::max(tail_us_, h);
    }
}

void MotionShaperFx::reset() {
    knots_.clear();
    for (int a = 0; a < NUM_AXES; a++) emitted_[a] = cum_[a];
    out_end_us_ = 0;
}

void MotionShaperFx::add_knot(int64_t t) {
    Knot k;
    k.t = t;
    for (int a = 0; a < NUM_AXES; a++) {
        k.p[a] = cum_[a];
        k.slope[a] = 0;
    }
    k.ve = 0;
    k.k = 0;
    if (!knots_.empty()) {
        Knot& b = knots_.back();
        int64_t dt = t - b.t;
        if (dt > 0) {
            for (int a = 0; a < NUM_AXES; a++) b.slope[a] = ((k.p[a] - b.p[a]) * (int64_t(1) << 32)) / dt;
            b.ve = f32(k.p[AX_E] - b.p[AX_E]) / (f32(dt) * 1e-6f);
        }
    }
    knots_.push_back(k);
}

int64_t MotionShaperFx::pos_q16(int axis, int64_t t) const {
    if (knots_.empty()) return cum_[axis] * 65536;
    if (t <= knots_.front().t) return knots_.front().p[axis] * 65536;
    if (t >= knots_.back().t) return knots_.back().p[axis] * 65536;
    size_t lo = 0, hi = knots_.size() - 1;   // knots_[lo].t <= t < knots_[hi].t
    while (hi - lo > 1) {
        size_t mid = (lo + hi) / 2;
        if (knots_[mid].t <= t) lo = mid;
        else hi = mid;
    }
    const Knot& k0 = knots_[lo];
    return k0.p[axis] * 65536 + ((k0.slope[axis] * (t - k0.t)) >> 16);
}

// The same integral as MotionShaper::pa_smooth_pos, in float, on positions relative to `base` (the
// position at tau) and times relative to tau: the numbers stay small, so float keeps the precision.
int64_t MotionShaperFx::pa_smooth_q16(int64_t tau) const {
    const float h = pa_h_;
    const int E = AX_E;
    if (h <= 0 || knots_.empty()) return pos_q16(E, tau);
    const int64_t h_us = to_us(double(h));
    const int64_t base = pos_q16(E, tau) >> 16;
    auto piece = [&](float u0, float u1, float q0, float v) {
        float res = 0;
        if (u0 < 0) {
            float a = u0, b = std::min(u1, 0.0f);
            res += q0 * h * (b - a) + (q0 + v * h) * (b * b - a * a) * 0.5f + v * (b * b * b - a * a * a) * (1.0f / 3);
        }
        if (u1 > 0) {
            float a = std::max(u0, 0.0f), b = u1;
            res += q0 * h * (b - a) + (-q0 + v * h) * (b * b - a * a) * 0.5f - v * (b * b * b - a * a * a) * (1.0f / 3);
        }
        return res;
    };
    auto sec = [&](int64_t us) { return f32(us - tau) * 1e-6f; };
    const int64_t lo = tau - h_us, hi = tau + h_us;
    float area = 0;
    const Knot &f = knots_.front(), &l = knots_.back();
    if (lo < f.t) area += piece(sec(lo), sec(std::min(hi, f.t)), f32(f.p[E] - base), 0.0f);
    if (hi > l.t) area += piece(sec(std::max(lo, l.t)), sec(hi), f32(l.p[E] - base), 0.0f);
    size_t i = 0, n = knots_.size();
    if (knots_[0].t < lo) {
        size_t a = 0, b = n - 1;
        while (b - a > 1) {
            size_t m = (a + b) / 2;
            if (knots_[m].t <= lo) a = m;
            else b = m;
        }
        i = a;
    }
    for (; i + 1 < n && knots_[i].t < hi; i++) {
        const Knot &k0 = knots_[i], &k1 = knots_[i + 1];
        if (k1.t <= k0.t) continue;
        int64_t s = std::max(k0.t, lo), e = std::min(k1.t, hi);
        if (e <= s) continue;
        float v = k0.ve;
        float q0 = f32(k0.p[E] - base) + v * (f32(tau - k0.t) * 1e-6f) + k0.k * v;
        area += piece(sec(s), sec(e), q0, v);
    }
    float r = area / (h * h);
    return base * 65536 + int64_t(std::lrint(r * 65536.0f));
}

int64_t MotionShaperFx::out_q16(int axis, int64_t tau) const {
    const Pulses* sp = axis == 0 ? &px_ : axis == 1 ? &py_ : nullptr;
    if (sp && sp->n) {
        int64_t r = 0;   // 32 fraction bits
        for (int i = 0; i < sp->n; i++) r += int64_t(sp->a[i]) * pos_q16(axis, tau + sp->t[i]);
        return (r + 32768) >> 16;
    }
    if (axis == AX_E && cfg_.pa_on) return pa_smooth_q16(tau);
    return pos_q16(axis, tau);
}

void MotionShaperFx::emit_until(int64_t limit) {
    while (out_t_ + grid_us_ <= limit) {
        int64_t t1 = out_t_ + grid_us_;
        int32_t st[NUM_AXES];
        bool any = false;
        for (int a = 0; a < NUM_AXES; a++) {
            int64_t target = (out_q16(a, t1) + 32768) >> 16;
            st[a] = int32_t(target - emitted_[a]);
            emitted_[a] = target;
            any |= st[a] != 0;
        }
        if (sink_ && any) sink_(out_t_, t1, st);
        out_count_++;
        out_t_ = t1;
        out_end_us_ = t1;
    }
}

void MotionShaperFx::trim() {
    int64_t keep_from = out_t_ - tail_us_ - 1;
    while (knots_.size() > 2 && knots_[1].t < keep_from) knots_.pop_front();
}

void MotionShaperFx::push(double t0d, double dur, const int32_t steps[NUM_AXES], bool last) {
    const int64_t t0 = to_us(t0d), t1 = to_us(t0d + dur);
    if (knots_.empty() || t0 > knots_.back().t) {
        if (knots_.empty()) out_t_ = std::max(t0 - lead_us_, out_end_us_);
        add_knot(t0);
    }
    knots_.back().k = ((steps[0] || steps[1]) && steps[AX_E] > 0) ? float(cfg_.pa) : 0.0f;
    for (int a = 0; a < NUM_AXES; a++) cum_[a] += steps[a];
    add_knot(t1);
    if (last) {
        emit_until(t1 + tail_us_);
        if (out_t_ < t1 + tail_us_) emit_until(out_t_ + grid_us_);
        knots_.clear();
        return;
    }
    emit_until(t1 - lead_us_);
    trim();
}

}  // namespace ph
