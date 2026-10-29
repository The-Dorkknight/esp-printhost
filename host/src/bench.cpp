// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
#include "ph/bench.h"

#include "ph/planner.h"
#include "ph/toolhead.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <cstdio>
#include <memory>
#include <vector>

namespace ph {

std::string BenchResult::text(const char* label) const {
    char b[240];
    snprintf(b, sizeof b, "%s: %.1f s of motion, %llu steps, %llu queue_step, %.2f us per step, %.0f %% of one core",
             label, sim_s, (unsigned long long)steps, (unsigned long long)commands, us_per_step(), core_load() * 100);
    return b;
}

namespace {
const double CLOCK = 12e6;   // the SKR Pico's step clock

// The shaper the printer itself uses in this build: the integer one in the INT32/64 test build
// (until 2026-10-29 this benchmark used the double one in every build, so it could not compare them).
#if PH_FIXED
using BenchShaper = MotionShaperFx;
#else
using BenchShaper = MotionShaper;
#endif

struct Pipe {
    StepQueue q[NUM_AXES];
    std::unique_ptr<BenchShaper> shaper;
    double t = 1.0;
    BenchResult r;
    std::vector<std::string> out;
    void steps(double t0, double dur, const int32_t* st) {
        double c0 = t0 * CLOCK, c1 = (t0 + dur) * CLOCK;
        for (int a = 0; a < NUM_AXES; a++) {
            if (!st[a]) continue;
            uint32_t k = uint32_t(st[a] < 0 ? -st[a] : st[a]);
            double iv = (c1 - c0) / k;
            q[a].push_run(st[a] > 0, c0 + 0.5 * iv, iv, k);
            r.steps += k;
        }
    }
};
}  // namespace

BenchResult motion_bench(double speed, const ShapingConfig* shaping, const std::function<uint64_t()>& now_us,
                         bool short_path) {
    PlannerConfig pc;
    const double spm[NUM_AXES] = {80, 80, 400, 93};
    for (int a = 0; a < NUM_AXES; a++) pc.steps_per_mm[a] = spm[a];
    pc.dda_hz = 1e6;
    pc.segment_ms = 5;
    pc.max_velocity = 300;
    pc.max_accel = 3000;
    pc.lookahead = 32;
    pc.junction_deviation = 5.0 * 5.0 * (std::sqrt(2.0) - 1.0) / 3000;
    Pipe p;
    if (shaping) {
        p.shaper.reset(new BenchShaper());
        p.shaper->configure(*shaping);
#if PH_FIXED
        p.shaper->set_sink([&p](int64_t u0, int64_t u1, const int32_t* st) { p.steps(u0 * 1e-6, (u1 - u0) * 1e-6, st); });
#else
        p.shaper->set_sink([&p](double t0, double d, const int32_t* st) { p.steps(t0, d, st); });
#endif
    }
    Planner pl(pc);
    pl.set_sinks([&p](const Segment& s) {
        double T = s.n / 1e6;
        if (p.shaper) p.shaper->push(p.t, T, s.steps, (s.flags & SEG_LAST) != 0);
        else p.steps(p.t, T, s.steps);
        p.t += T;
        p.r.sim_s += T;
    });
    // compress every 0.1 s of motion, as the toolhead's tick does
    double next_flush = p.t + 0.1;
    auto maybe_flush = [&](bool all) {
        if (!all && p.t < next_flush) return;
        uint64_t c0 = now_us();
        for (auto& q : p.q) {
            p.out.clear();
            q.flush(all ? UINT64_MAX : uint64_t((p.t - 0.05) * CLOCK), p.out);
            p.r.commands += p.out.size();
        }
        p.r.compress_us += double(now_us() - c0);
        next_flush = p.t + 0.1;
    };
    double e = 0;
    uint64_t t0 = now_us();
    double plan_extra = 0;
    for (int k = 0; k <= (short_path ? 360 : 360 * 3); k += 2) {
        double a = k * M_PI / 180;
        double tg[NUM_AXES] = {60 + 20 * std::cos(a), 60 + 20 * std::sin(a), 0.2, e += 0.023};
        pl.add_move(tg, speed);
        uint64_t f0 = now_us();
        maybe_flush(false);
        plan_extra += double(now_us() - f0);
    }
    for (int k = 0; k < (short_path ? 10 : 40); k++) {
        double tg[NUM_AXES] = {double(20 + (k % 2) * 60), 20 + k * 1.5, 0.2, e += 2.0};
        pl.add_move(tg, speed);
        uint64_t f0 = now_us();
        maybe_flush(false);
        plan_extra += double(now_us() - f0);
    }
    pl.flush();
    p.r.plan_us = double(now_us() - t0) - plan_extra;
    maybe_flush(true);
    return p.r;
}

BenchResult homing_bench(const std::function<uint64_t()>& now_us) {
    // as Toolhead::schedule_homing_move: 123 mm at 4 mm/s (Z, 400 steps/mm), 100 mm/s^2
    StepQueue q;
    q.init(1, false, uint32_t(CLOCK * 25e-6), uint32_t(CLOCK * 2e-6));
    BenchResult r;
    uint64_t t0 = now_us();
    double spm = 400, speed = 4, accel = 100, dist = 123;
    uint64_t n = uint64_t(std::llround(dist * spm));
    double da = speed * speed / (2 * accel);
    uint64_t n_acc = std::min<uint64_t>(n, uint64_t(da * spm));
    double c0 = CLOCK;
    for (uint64_t k = 1; k <= n_acc; k++) q.push_run(1, c0 + std::sqrt(2 * ((double(k) - 0.5) / spm) / accel) * CLOCK, 1, 1);
    double x = (double(n_acc) + 0.5) / spm;
    q.push_run(1, c0 + (speed / accel + (x - da) / speed) * CLOCK, CLOCK / (speed * spm), uint32_t(n - n_acc));
    r.plan_us = double(now_us() - t0);
    std::vector<std::string> out;
    uint64_t t1 = now_us();
    q.flush(UINT64_MAX, out);
    r.compress_us = double(now_us() - t1);
    r.commands = out.size();
    r.steps = n;
    r.sim_s = dist / speed;
    return r;
}

#if PH_BENCH_LEGACY || !defined(ESP_PLATFORM)
// The step compressor as it was until 2026-10-23 (double-precision step times, every candidate
// tried), kept only to measure the difference under QEMU and on the PC.
namespace {
class LegacyStepQueue {
public:
    void init(int oid, bool invert_dir, uint32_t max_error_ticks, uint32_t min_interval_ticks) {
        oid_ = oid, invert_ = invert_dir, max_err_ = max_error_ticks, min_iv_ = min_interval_ticks;
    }
    void push_run(int dir, double first, double interval, uint32_t count) {
        if (!count) return;
        runs_.push_back({first, interval, count, 0, int8_t(dir)});
        pending_steps_ += count;
    }
    int flush(uint64_t upto, std::vector<std::string>& out) {
        int moves = 0;
        std::vector<int64_t> win;
        const std::string O = std::to_string(oid_);
        while (!runs_.empty()) {
            // the next steps in one direction, at or before `upto` (at most 256 at a time: the ESP32-S3
            // has no double-precision hardware, so the fit works on short windows in float)
            win.clear();
            int dir = runs_.front().dir;
            for (size_t ri = 0; ri < runs_.size() && win.size() < 256; ri++) {
                const Run& r = runs_[ri];
                if (r.dir != dir) break;
                bool stop = false;
                for (uint32_t j = r.done; j < r.count && win.size() < 256; j++) {
                    double c = r.first + r.interval * j;
                    if (c > double(upto)) {
                        stop = true;
                        break;
                    }
                    win.push_back(int64_t(std::llround(c)));
                }
                if (stop) break;
            }
            if (win.empty()) break;
            if (need_reset_ || win[0] - int64_t(last_clock_) > 0x40000000LL) {
                // start from a known time: just before the first step
                uint64_t r = uint64_t(win[0] - 2 * int64_t(min_iv_) - 1000);
                out.push_back("reset_step_clock oid=" + O + " clock=" + std::to_string(uint32_t(r)));
                last_clock_ = r;
                need_reset_ = false;
                cur_dir_ = -1;
            }
            if (dir != cur_dir_) {
                out.push_back("set_next_step_dir oid=" + O + " dir=" + std::to_string(dir ^ (invert_ ? 1 : 0)));
                cur_dir_ = dir;
            }
            // Fit: for a trial `add`, every step k must satisfy |last + (k+1)*I + add*k(k+1)/2 - t_k| <= max_err,
            // which bounds I to a range; extend k while the ranges still overlap (Klipper's stepcompress
            // does a finer search; this tries a handful of adds and keeps the longest run).
            const int64_t last = int64_t(last_clock_);
            const double E = double(max_err_);
            auto span = [&](float A, float& lo, float& hi) {
                lo = -3e38f, hi = 3e38f;
                const float e = float(E);
                size_t k = 0;
                for (; k < win.size(); k++) {
                    float kf = float(k);
                    float base = float(win[k] - last) - A * kf * (kf + 1.0f) * 0.5f;
                    float inv = 1.0f / (kf + 1.0f);
                    float nlo = std::max(lo, (base - e) * inv), nhi = std::min(hi, (base + e) * inv);
                    if (std::ceil(nlo) > std::floor(nhi)) break;
                    if (k > 0 && nlo + std::min(0.0f, A * kf) < float(min_iv_)) break;   // intervals stay >= minimum
                    lo = nlo, hi = nhi;
                }
                return k;
            };
            auto exact = [&](int64_t I, int64_t A, int64_t& end_clock) {   // the count with integer I, A
                int64_t pred = last, iv = I;
                size_t k = 0;
                for (; k < win.size() && k < 65535; k++) {
                    if (iv < int64_t(min_iv_)) break;
                    int64_t pp = pred + iv;
                    if (std::llabs(pp - win[k]) > int64_t(max_err_)) break;
                    pred = pp;
                    iv += A;
                }
                end_clock = pred;
                return k;
            };
            size_t best = 0;
            int64_t best_add = 0, best_end = 0, best_iv = 0;
            float cand[12];
            int nc = 0;
            cand[nc++] = 0.0f;
            int64_t I0e = win[0] - last;
            for (size_t j = 2; j < win.size() && nc < 11; j *= 2)
                cand[nc++] = 2.0f * (float(win[j] - last) - float(j + 1) * float(I0e)) / (float(j) * float(j + 1));
            if (win.size() > 2) {
                size_t j = win.size() - 1;
                cand[nc++] = 2.0f * (float(win[j] - last) - float(j + 1) * float(I0e)) / (float(j) * float(j + 1));
            }
            for (int ci = 0; ci < nc; ci++) {
                float Ar = std::round(cand[ci]);
                if (Ar < -32768.0f || Ar > 32767.0f) continue;
                float lo, hi;
                if (span(Ar, lo, hi) <= best) continue;
                int64_t I = int64_t(std::llround((double(lo) + double(hi)) / 2));
                int64_t endc = 0;
                size_t n = exact(I, int64_t(Ar), endc);
                if (n > best) best = n, best_add = int64_t(Ar), best_end = endc, best_iv = I;
            }
            int64_t I0 = best_iv;
            if (best == 0) {   // a step closer than the board can do: send it alone, as soon as allowed
                I0 = std::max<int64_t>(win[0] - last, int64_t(min_iv_));
                best = 1, best_add = 0, best_end = last + I0;
            }
            out.push_back("queue_step oid=" + O + " interval=" + std::to_string(I0) + " count=" + std::to_string(best) +
                          " add=" + std::to_string(best_add));
            moves++;
            last_clock_ = uint64_t(best_end);
            // consume `best` steps from the runs
            uint64_t left = best;
            pending_steps_ -= best;
            while (left) {
                Run& r = runs_.front();
                uint32_t avail = r.count - r.done;
                if (avail > left) {
                    r.done += uint32_t(left);
                    left = 0;
                } else {
                    left -= avail;
                    runs_.pop_front();
                }
            }
        }
        return moves;
    }

private:
    struct Run { double first, interval; uint32_t count, done; int8_t dir; };
    int oid_ = -1;
    bool invert_ = false;
    uint32_t max_err_ = 300, min_iv_ = 50;
    std::deque<Run> runs_;
    uint64_t pending_steps_ = 0;
    uint64_t last_clock_ = 0;
    bool need_reset_ = true;
    int cur_dir_ = -1;
};
}  // namespace

BenchResult homing_bench_legacy(const std::function<uint64_t()>& now_us) {
    LegacyStepQueue q;
    q.init(1, false, uint32_t(CLOCK * 25e-6), uint32_t(CLOCK * 2e-6));
    BenchResult r;
    uint64_t t0 = now_us();
    double spm = 400, speed = 4, accel = 100, dist = 123;
    uint64_t n = uint64_t(std::llround(dist * spm));
    double da = speed * speed / (2 * accel);
    uint64_t n_acc = std::min<uint64_t>(n, uint64_t(da * spm));
    double c0 = CLOCK;
    for (uint64_t k = 1; k <= n_acc; k++) q.push_run(1, c0 + std::sqrt(2 * ((double(k) - 0.5) / spm) / accel) * CLOCK, 1, 1);
    double x = (double(n_acc) + 0.5) / spm;
    q.push_run(1, c0 + (speed / accel + (x - da) / speed) * CLOCK, CLOCK / (speed * spm), uint32_t(n - n_acc));
    r.plan_us = double(now_us() - t0);
    std::vector<std::string> out;
    uint64_t t1 = now_us();
    q.flush(UINT64_MAX, out);
    r.compress_us = double(now_us() - t1);
    r.commands = out.size();
    r.steps = n;
    r.sim_s = dist / speed;
    return r;
}
#endif

std::string motion_bench_report(const std::function<uint64_t()>& now_us, bool short_path) {
    std::string s = std::string(Toolhead::MATHS) + "\n";
#if PH_BENCH_LEGACY || !defined(ESP_PLATFORM)
    s += homing_bench_legacy(now_us).text("homing move, compressor before 2026-10-24") + "\n";
#endif
    s += homing_bench(now_us).text("homing move (Tina 2 Z, 49,200 steps)") + "\n";
    for (double v : {100.0, 200.0}) {
        char l[64];
        snprintf(l, sizeof l, "print path at %.0f mm/s, plain", v);
        s += motion_bench(v, nullptr, now_us, short_path).text(l) + "\n";
#if PH_PRO
        ShapingConfig c;
        std::vector<double> A, T;
        shaper_def("mzv", 45, 0.1, A, T);
        c.x = shaper_pulses(A, T);
        shaper_def("ei", 38, 0.1, A, T);
        c.y = shaper_pulses(A, T);
        c.pa = 0.05;
        c.pa_on = true;
        snprintf(l, sizeof l, "print path at %.0f mm/s, input shaping + pressure advance", v);
        s += motion_bench(v, &c, now_us, short_path).text(l) + "\n";
#endif
    }
    return s;
}

}  // namespace ph
