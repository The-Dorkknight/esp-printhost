// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
#include "ph/planner.h"
#include <algorithm>
#include <cmath>

namespace ph {
namespace {

inline double sq(double x) { return x * x; }
inline int64_t rnd(double x) { return int64_t(std::llround(x)); }
const double kInf = 1e300;
const uint64_t kMaxN = 1ull << 30;

// Exact trapezoid of one block (distance / speed as functions of time).
struct Profile {
    double v0, vp, v1, a, L, da, cruise;
    double ta, tc, td, dur;

    Profile(double v0_, double v1_, double vnom, double a_, double L_)
        : v0(v0_), v1(v1_), a(a_), L(L_) {
        double vmax_end = std::max(v0, v1);
        double vp2 = std::max((2 * a * L + sq(v0) + sq(v1)) / 2, sq(vmax_end));
        vp = std::min(vnom, std::sqrt(vp2));
        vp = std::max(vp, vmax_end);
        if (vp < 1e-9) vp = 1e-9;
        da = std::max(0.0, (sq(vp) - sq(v0)) / (2 * a));
        double dd = std::max(0.0, (sq(vp) - sq(v1)) / (2 * a));
        cruise = std::max(0.0, L - da - dd);
        ta = std::max(0.0, (vp - v0) / a);
        tc = cruise / vp;
        td = std::max(0.0, (vp - v1) / a);
        dur = ta + tc + td;
    }
    double dist(double t) const {
        double s;
        if (t <= ta) s = v0 * t + 0.5 * a * t * t;
        else if (t <= ta + tc) s = da + vp * (t - ta);
        else {
            double tau = std::min(t - ta - tc, td);
            s = da + cruise + vp * tau - 0.5 * a * tau * tau;
        }
        return std::min(std::max(s, 0.0), L);
    }
    double vel(double t) const {
        double v;
        if (t <= ta) v = v0 + a * t;
        else if (t <= ta + tc) v = vp;
        else v = vp - a * std::min(t - ta - tc, td);
        return std::max(v, 0.0);
    }
};

}  // namespace

Planner::Planner(const PlannerConfig& cfg) : cfg_(cfg), accel_(cfg.max_accel) {
    if (cfg_.lookahead < 1) cfg_.lookahead = 1;
    if (cfg_.dda_hz < 1) cfg_.dda_hz = 1;
    ring_.resize(size_t(cfg_.lookahead) + 2);
    pa_.configure(0.0, 4, cfg_.step_rate_fraction * cfg_.dda_hz / cfg_.steps_per_mm[AX_E]);
}

void Planner::set_accel(double a) {
    if (!(a > 0)) return;
    accel_ = std::min(a, cfg_.max_accel);
}

void Planner::set_pressure_advance(double k) {
    // Reconfigure: drops PA history (queue is expected to be empty).
    pa_.configure(k, 4, cfg_.step_rate_fraction * cfg_.dda_hz / cfg_.steps_per_mm[AX_E]);
    e_acc_ = pos_[AX_E];
}

void Planner::set_position(const double mm[NUM_AXES]) {
    flush();
    for (int a = 0; a < NUM_AXES; a++) {
        pos_[a] = mm[a];
        emitted_[a] = rnd(mm[a] * cfg_.steps_per_mm[a]);
    }
    e_acc_ = pos_[AX_E];
}

void Planner::out(const Segment& s) {
    seg_count_++;
    if (seg_sink_) seg_sink_(s);
}

void Planner::release_held(bool last) {
    if (!has_held_) return;
    if (last) held_.flags |= SEG_LAST;
    out(held_);
    has_held_ = false;
}

void Planner::emit_segment(uint64_t n, const int32_t steps[NUM_AXES]) {
    uint64_t need = 0;
    for (int a = 0; a < NUM_AXES; a++) {
        uint64_t m = steps[a] < 0 ? uint64_t(-int64_t(steps[a])) : uint64_t(steps[a]);
        need = std::max(need, 2 * m);
    }
    if (n < need) {          // would violate 2*|steps| <= n: lengthen (tiny timing change)
        n = need;
        stretched_++;
    }
    if (n < 1) n = 1;
    if (n > kMaxN) n = kMaxN;   // callers never ask for more (dwell is chunked)
    Segment s;
    s.n = uint32_t(n);
    for (int a = 0; a < NUM_AXES; a++) s.steps[a] = steps[a];
    s.flags = 0;
    s.tag = tag_++;
    release_held(false);
    held_ = s;
    has_held_ = true;
}

void Planner::emit_markers_upto(uint64_t block_index) {
    while (!markers_.empty() && markers_.front().seq <= block_index) {
        uint32_t id = markers_.front().id;
        markers_.pop_front();
        release_held(false);
        if (marker_sink_) marker_sink_(id);
    }
}

void Planner::add_marker(uint32_t id) {
    if (count_ == 0) {
        release_held(false);
        if (marker_sink_) marker_sink_(id);
        return;
    }
    markers_.push_back({id, blocks_pushed_});
}

void Planner::add_move(const double tgt[NUM_AXES], double feed) {
    const double* spm = cfg_.steps_per_mm;
    double d[NUM_AXES];
    for (int a = 0; a < NUM_AXES; a++) d[a] = tgt[a] - pos_[a];

    Block b;
    for (int a = 0; a < NUM_AXES; a++) { b.start[a] = pos_[a]; b.end[a] = tgt[a]; b.delta[a] = d[a]; }
    for (int a = 0; a < NUM_AXES; a++) pos_[a] = tgt[a];

    const double cap = cfg_.step_rate_fraction * cfg_.dda_hz;   // steps/s per axis
    double v = feed > 0 ? feed : cfg_.max_velocity;
    double acc = accel_;
    double xyz = std::sqrt(sq(d[0]) + sq(d[1]) + sq(d[2]));
    double ea = std::fabs(d[AX_E]);
    b.u[0] = b.u[1] = b.u[2] = 0;
    if (xyz > 1e-9) {
        b.kind = 0;
        b.L = xyz;
        for (int i = 0; i < 3; i++) b.u[i] = d[i] / xyz;
        double xy = std::sqrt(sq(d[0]) + sq(d[1]));
        if (xy > 1e-12) v = std::min(v, cfg_.max_velocity * xyz / xy);
        if (std::fabs(d[2]) > 1e-12) v = std::min(v, cfg_.max_z_velocity * xyz / std::fabs(d[2]));
        if (ea > 1e-12) {
            v = std::min(v, cfg_.max_e_velocity * xyz / ea);
            acc = std::min(acc, cfg_.max_e_accel * xyz / ea);
        }
        for (int a = 0; a < NUM_AXES; a++) {
            double m = std::fabs(d[a]);
            if (m > 1e-12) v = std::min(v, cap * xyz / (spm[a] * m));
        }
    } else if (ea > 1e-12) {
        b.kind = 1;
        b.L = ea;
        b.u[0] = d[AX_E] > 0 ? 1.0 : -1.0;
        v = std::min(v, std::min(cfg_.max_e_velocity, cap / spm[AX_E]));
        acc = std::min(accel_, cfg_.max_e_accel);
    } else {
        return;
    }
    v = std::max(v, 1e-3);
    acc = std::max(acc, 1e-3);
    b.nominal = v;
    b.nominal_v2 = sq(v);
    b.accel = acc;

    // Junction speed from the previous queued block.
    if (count_ == 0) {
        b.max_entry_v2 = 0;
        pin_v2_ = 0;
    } else {
        const Block& p = at(count_ - 1);
        double minj2 = sq(cfg_.min_junction_speed);
        double vj2;
        if (p.kind != b.kind) {
            vj2 = minj2;
        } else {
            double cos_t = -(p.u[0] * b.u[0] + p.u[1] * b.u[1] + p.u[2] * b.u[2]);
            if (cos_t > 0.999999) vj2 = minj2;                 // reversal
            else if (cos_t < -0.999999) vj2 = kInf;            // collinear
            else {
                double s = std::sqrt(0.5 * (1.0 - cos_t));
                double a = std::min(p.accel, b.accel);
                vj2 = a * cfg_.junction_deviation * s / (1.0 - s);
            }
            vj2 = std::max(vj2, minj2);
        }
        b.max_entry_v2 = std::min(vj2, std::min(b.nominal_v2, p.nominal_v2));
    }
    b.entry_v2 = 0;
    ring_[(head_ + count_) % ring_.size()] = b;
    count_++;
    blocks_pushed_++;
    replan();
    q_len_ += b.L;
    // Release the oldest block once (a) the queue is full, or (b) enough path is
    // queued behind it to brake from its nominal speed. (b) is what lets
    // thousands of tiny moves reach full speed: a block count alone would cap
    // the speed at sqrt(2*a*queued_length).
    while (count_ > size_t(cfg_.lookahead) ||
           (count_ >= 2 && q_len_ - at(0).L >= at(0).nominal_v2 / (2 * at(0).accel)))
        emit_head();
}

void Planner::replan() {
    size_t n = count_;
    if (!n) return;
    for (size_t i = n; i-- > 0;) {
        Block& b = at(i);
        if (i == 0) {
            b.entry_v2 = pin_v2_;
        } else {
            double ex2 = (i == n - 1) ? 0.0 : at(i + 1).entry_v2;
            b.entry_v2 = std::min(b.max_entry_v2, ex2 + 2 * b.accel * b.L);
        }
    }
    for (size_t i = 0; i + 1 < n; i++) {
        const Block& b = at(i);
        double reach = b.entry_v2 + 2 * b.accel * b.L;
        Block& nx = at(i + 1);
        if (nx.entry_v2 > reach) nx.entry_v2 = reach;
    }
}

void Planner::emit_head() {
    if (!count_) return;
    Block b = at(0);
    q_len_ -= b.L;
    if (count_ == 1) q_len_ = 0;
    double exit2 = (count_ > 1) ? at(1).entry_v2 : 0.0;
    emit_block(b, std::sqrt(b.entry_v2), std::sqrt(exit2));
    head_ = (head_ + 1) % ring_.size();
    count_--;
    pin_v2_ = count_ ? exit2 : 0.0;
}

void Planner::emit_block(const Block& b, double v0, double v1) {
    emit_markers_upto(blocks_emitted_);
    blocks_emitted_++;
    const double* spm = cfg_.steps_per_mm;
    const double hz = cfg_.dda_hz;
    const double L = b.L;
    Profile pr(v0, v1, b.nominal, b.accel, L);

    // Whole ticks for this block; the rounding error is carried into the next
    // block so thousands of tiny moves keep the programmed speed.
    double want = pr.dur * hz + tick_carry_;
    uint64_t T = uint64_t(std::max(1.0, std::floor(want + 0.5)));
    tick_carry_ = std::max(-2.0, std::min(2.0, want - double(T)));
    double seg_ticks = std::max(1.0, std::floor(cfg_.segment_ms * hz / 1000.0 + 0.5));
    uint64_t cnt = uint64_t(std::max(1.0, std::floor(double(T) / seg_ticks + 0.5)));
    if (cnt > T) cnt = T;

    const bool use_pa = pa_.k() != 0.0;
    double s_prev = 0;
    uint64_t k_prev = 0;
    for (uint64_t i = 1; i <= cnt; i++) {
        uint64_t k = (i * T) / cnt;
        bool last = (i == cnt);
        double s, vend;
        if (last) { s = L; vend = v1; }
        else {
            double t = pr.dur * double(k) / double(T);
            s = pr.dist(t);
            vend = pr.vel(t);
        }
        double f = s / L;
        int64_t tgt[NUM_AXES];
        for (int a = 0; a < NUM_AXES; a++) {
            if (a == AX_E && use_pa) continue;
            tgt[a] = last ? rnd(b.end[a] * spm[a]) : rnd((b.start[a] + b.delta[a] * f) * spm[a]);
        }
        if (use_pa) {
            double ds = s - s_prev;
            PressureAdvance::In in{b.kind == 0 ? ds : 0.0, b.delta[AX_E] * ds / L, vend,
                                   double(k - k_prev) / hz};
            e_acc_ += pa_.process(in);
            tgt[AX_E] = rnd(e_acc_ * spm[AX_E]);
        }
        int32_t st[NUM_AXES];
        for (int a = 0; a < NUM_AXES; a++) {
            st[a] = int32_t(tgt[a] - emitted_[a]);
            emitted_[a] = tgt[a];
        }
        emit_segment(k - k_prev, st);
        s_prev = s;
        k_prev = k;
    }
}

// After a drain: make the driven position exactly round(pos*spm) on every axis
// (needed for E when pressure advance is on). Rate-limited, chunked.
void Planner::correct_positions() {
    const double* spm = cfg_.steps_per_mm;
    const double cap = cfg_.step_rate_fraction * cfg_.dda_hz;
    for (;;) {
        int64_t diff[NUM_AXES];
        int64_t big = 0;
        for (int a = 0; a < NUM_AXES; a++) {
            diff[a] = rnd(pos_[a] * spm[a]) - emitted_[a];
            big = std::max<int64_t>(big, diff[a] < 0 ? -diff[a] : diff[a]);
        }
        if (!big) break;
        int32_t st[NUM_AXES];
        for (int a = 0; a < NUM_AXES; a++) {
            int64_t c = std::max<int64_t>(-100000, std::min<int64_t>(100000, diff[a]));
            st[a] = int32_t(c);
            emitted_[a] += c;
        }
        int64_t m = std::min<int64_t>(big, 100000);
        uint64_t n = uint64_t(std::ceil(double(m) * cfg_.dda_hz / cap)) + 1;
        emit_segment(n, st);
    }
}

void Planner::flush() {
    while (count_ > 0) emit_head();
    if (pa_.k() != 0.0) {
        e_acc_ += pa_.flush();      // release the advance still applied
        correct_positions();
        e_acc_ = pos_[AX_E];
    } else {
        correct_positions();
    }
    release_held(true);
    tick_carry_ = 0;
    emit_markers_upto(~uint64_t(0));
}

void Planner::dwell(uint64_t ticks) {
    flush();
    int32_t zero[NUM_AXES] = {0, 0, 0, 0};
    uint64_t chunk = uint64_t(std::max(1.0, std::min(cfg_.dda_hz, double(kMaxN))));
    while (ticks > 0) {
        uint64_t n = std::min(ticks, chunk);
        ticks -= n;
        emit_segment(n, zero);
    }
    release_held(true);
}

}  // namespace ph
