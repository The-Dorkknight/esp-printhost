// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Input shaping and pressure advance (shaping.h), 2026-10-24.
#include <array>
#include "check.h"
#include "ph/bench.h"
#include "ph/planner.h"
#include <chrono>
#include "ph/shaping.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

using namespace ph;

namespace {

struct Knots {   // a position over time, linear in between
    std::vector<double> t, p;
    double at(double x) const {
        if (t.empty()) return 0;
        if (x <= t.front()) return p.front();
        if (x >= t.back()) return p.back();
        size_t i = size_t(std::upper_bound(t.begin(), t.end(), x) - t.begin());
        double f = (x - t[i - 1]) / (t[i] - t[i - 1]);
        return p[i - 1] + (p[i] - p[i - 1]) * f;
    }
};

// Planner segments for one or more moves, from rest to rest. Returns nominal knots for each axis.
struct Plan {
    std::vector<Segment> segs;
    double spm[NUM_AXES] = {80, 80, 400, 100};
    void run(const std::vector<std::vector<double>>& targets, double feed, double accel) {
        PlannerConfig pc;
        for (int a = 0; a < NUM_AXES; a++) pc.steps_per_mm[a] = spm[a];
        pc.dda_hz = 1e6;
        pc.segment_ms = 5;
        pc.max_velocity = 300;
        pc.max_accel = accel;
        pc.max_e_velocity = 100;
        pc.max_e_accel = 10000;
        pc.lookahead = 64;
        Planner p(pc);
        p.set_sinks([this](const Segment& s) { segs.push_back(s); });
        for (auto& t : targets) {
            double tg[NUM_AXES] = {t[0], t[1], t[2], t[3]};
            p.add_move(tg, feed);
        }
        p.flush();
    }
    Knots nominal(int axis, double t0) const {
        Knots k;
        double t = t0, pos = 0;
        k.t.push_back(t), k.p.push_back(0);
        for (auto& s : segs) {
            t += s.n / 1e6;
            pos += s.steps[axis];
            k.t.push_back(t), k.p.push_back(pos);
        }
        return k;
    }
};

// Feed segments through a MotionShaper; returns the output knots per axis (steps).
struct Shaped {
    Knots k[NUM_AXES];
    int64_t total[NUM_AXES] = {};
    void run(const Plan& pl, const ShapingConfig& c, double grid, double t0) {
        MotionShaper ms(grid);
        ms.configure(c);
        bool first = true;
        ms.set_sink([&](double t, double dur, const int32_t* st) {
            for (int a = 0; a < NUM_AXES; a++) {
                if (first || k[a].t.empty() || std::fabs(k[a].t.back() - t) > 1e-9) {   // a gap: hold
                    k[a].t.push_back(t), k[a].p.push_back(double(total[a]));
                }
                total[a] += st[a];
                k[a].t.push_back(t + dur), k[a].p.push_back(double(total[a]));
            }
            first = false;
        });
        double t = t0;
        for (size_t i = 0; i < pl.segs.size(); i++) {
            const Segment& s = pl.segs[i];
            ms.push(t, s.n / 1e6, s.steps, i + 1 == pl.segs.size());
            t += s.n / 1e6;
        }
        CHECK(ms.idle());
    }
};

// A printer frame resonance: the nozzle follows the motors through a spring with natural frequency
// f and damping zeta. Returns the largest swing (mm) once the commanded motion has ended.
template <class U>
double residual(U u, double t_end, double f, double zeta) {
    const double w = 2 * M_PI * f, dt = 2e-6;
    double x = u(0), v = 0, prev_u = u(0);
    double worst = 0, final_u = u(t_end + 1);
    for (double t = 0; t < t_end + 0.3; t += dt) {
        double uu = u(t), ud = (uu - prev_u) / dt;
        prev_u = uu;
        double acc = -w * w * (x - uu) - 2 * zeta * w * (v - ud);
        v += acc * dt;
        x += v * dt;
        if (t > t_end + 0.005) worst = std::max(worst, std::fabs(x - final_u));
    }
    return worst;
}

}  // namespace

static void test_shaper_defs() {
    // Klipper v0.13.0 shaper_defs.py at 43.7 Hz, damping 0.1 (printed by Klipper's own file)
    struct Ref { const char* name; std::vector<double> A, T; } refs[] = {
        {"zv", {1, 0.729247614288}, {0, 0.0114992885041}},
        {"mzv", {0.292893218813, 0.32687415041, 0.182398743541}, {0, 0.00862446637808, 0.0172489327562}},
        {"zvd", {1, 1.45849522858, 0.531802082944}, {0, 0.0114992885041, 0.0229985770082}},
        {"ei", {0.2625, 0.346392616787, 0.139598046773}, {0, 0.0114992885041, 0.0229985770082}},
        {"2hump_ei", {0.159797202155, 0.248092078702, 0.180920556517, 0.0619718159144},
         {0, 0.0114992885041, 0.0229985770082, 0.0344978655123}},
        {"3hump_ei", {0.112379629365, 0.173196308393, 0.159668651592, 0.0921061575618, 0.0317824713},
         {0, 0.0114992885041, 0.0229985770082, 0.0344978655123, 0.0459971540164}},
    };
    for (auto& r : refs) {
        std::vector<double> A, T;
        CHECK(shaper_def(r.name, 43.7, 0.1, A, T));
        CHECK(A.size() == r.A.size() && T.size() == r.T.size());
        for (size_t i = 0; i < A.size() && i < r.A.size(); i++) {
            NEAR(A[i], r.A[i], 1e-9);
            NEAR(T[i], r.T[i], 1e-11);
        }
        ShaperPulses sp = shaper_pulses(A, T);
        double sa = 0, sat = 0;
        for (int i = 0; i < sp.n; i++) sa += sp.a[i], sat += sp.a[i] * sp.t[i];
        NEAR(sa, 1.0, 1e-12);
        NEAR(sat, 0.0, 1e-12);
    }
    std::vector<double> A, T;
    CHECK(!shaper_def("bogus", 40, 0.1, A, T));
    CHECK(shaper_min_freq("mzv") == 23 && shaper_min_freq("3hump_ei") == 48);
}

// Shaping must not lose or add a single step, and must leave constant speed alone.
static void test_shaper_conserves_steps() {
    Plan pl;
    pl.run({{40, 0, 0, 2}, {40, 30, 0, 3.5}, {0, 30, 0, 5}, {10, 5, 0, 6}}, 120, 3000);
    ShapingConfig c;
    std::vector<double> A, T;
    shaper_def("mzv", 45, 0.1, A, T);
    c.x = shaper_pulses(A, T);
    shaper_def("ei", 38, 0.1, A, T);
    c.y = shaper_pulses(A, T);
    c.pa = 0.05;
    c.pa_on = true;
    Shaped sh;
    sh.run(pl, c, 0.005, 1.0);
    int64_t nominal[NUM_AXES] = {};
    for (auto& s : pl.segs)
        for (int a = 0; a < NUM_AXES; a++) nominal[a] += s.steps[a];
    for (int a = 0; a < NUM_AXES; a++) CHECK(sh.total[a] == nominal[a]);
    // in the middle of a long cruise, shaped X equals the plain plan (to rounding)
    Plan cr;
    cr.run({{200, 0, 0, 0}}, 100, 3000);
    Knots nom = cr.nominal(0, 1.0);
    Shaped s2;
    ShapingConfig c2;
    c2.x = c.x;
    s2.run(cr, c2, 0.005, 1.0);
    double tmid = 1.0 + (nom.t.back() - 1.0) / 2;
    NEAR(s2.k[0].at(tmid), nom.at(tmid), 1.0);
}

// The point of it all: how much ringing is left after a move.
static void test_shaper_reduces_vibration() {
    Plan pl;
    pl.run({{40, 0, 0, 0}}, 150, 3000);   // Ender 3 class: 40 mm at 150 mm/s, 3000 mm/s^2
    const double t0 = 0.0, spm = pl.spm[0];
    Knots nom = pl.nominal(0, t0);
    double t_end = nom.t.back();
    auto plain = [&](double t) { return nom.at(t) / spm; };
    printf("  input shaping, residual ringing after a 40 mm move (mm, mzv tuned to the resonance):\n");
    printf("    resonance  plain   5ms grid  2ms grid  1ms grid  ideal (continuous)\n");
    for (double f : {30.0, 45.0, 60.0, 80.0}) {
        std::vector<double> A, T;
        shaper_def("mzv", f, 0.1, A, T);
        ShapingConfig c;
        c.x = shaper_pulses(A, T);
        Shaped g5, g25, g1;
        g5.run(pl, c, 0.005, t0);
        g25.run(pl, c, 0.002, t0);
        g1.run(pl, c, 0.001, t0);
        auto s5 = [&](double t) { return g5.k[0].at(t) / spm; };
        auto s25 = [&](double t) { return g25.k[0].at(t) / spm; };
        auto s1 = [&](double t) { return g1.k[0].at(t) / spm; };
        auto ideal = [&](double t) {
            double r = 0;
            for (int i = 0; i < c.x.n; i++) r += c.x.a[i] * nom.at(t + c.x.t[i]);
            return r / spm;
        };
        double tail = 0;
        for (int i = 0; i < c.x.n; i++) tail = std::max(tail, -c.x.t[i]);
        double rp = residual(plain, t_end, f, 0.05), r5 = residual(s5, t_end + tail, f, 0.05);
        double r25 = residual(s25, t_end + tail, f, 0.05), ri = residual(ideal, t_end + tail, f, 0.05);
        double r1 = residual(s1, t_end + tail, f, 0.05);
        printf("    %4.0f Hz   %6.4f   %6.4f    %6.4f    %6.4f    %6.4f\n", f, rp, r5, r25, r1, ri);
        CHECK(ri < rp * 0.2);              // the shaper itself works
        if (f <= 60) CHECK(r5 < rp * 0.35);   // a 5 ms grid only works up to about 60 Hz
        // the 2 ms grid the toolhead uses: at least 60 % less, or below a third of a step (0.0125 mm
        // here: below that the rounding to whole steps is all that is left)
        CHECK(r25 < rp * 0.4 || r25 < 0.004);
    }
}

// Pressure advance: E runs ahead while speeding up, falls back while slowing down, ends exactly
// where it should; E-only moves (retractions) get none.
static void test_pressure_advance() {
    Plan pl;
    pl.run({{60, 0, 0, 3}}, 100, 2000);   // 60 mm at 100 mm/s with 3 mm of filament
    ShapingConfig c;
    c.pa = 0.08;
    c.pa_on = true;
    c.pa_smooth = 0.04;
    Shaped sh;
    sh.run(pl, c, 0.005, 0.0);
    Knots nom = pl.nominal(AX_E, 0.0);
    double accel_end = 100.0 / 2000.0;   // 0.05 s
    double espm = pl.spm[AX_E];
    double ahead = (sh.k[AX_E].at(accel_end) - nom.at(accel_end)) / espm;
    // v_E at cruise = 3 mm / 60 mm * 100 mm/s = 5 mm/s: advance K * v = 0.4 mm (smoothed: a bit less)
    CHECK(ahead > 0.25 && ahead < 0.45);
    double tmid = nom.t.back() / 2;
    NEAR((sh.k[AX_E].at(tmid) - nom.at(tmid)) / espm, 0.4, 0.02);
    CHECK(sh.total[AX_E] == int64_t(std::llround(3 * espm)));
    // a retraction alone: no advance
    Plan rt;
    rt.run({{0, 0, 0, -2}}, 30, 2000);
    Shaped s2;
    s2.run(rt, c, 0.005, 0.0);
    Knots rn = rt.nominal(AX_E, 0.0);
    double worst = 0;
    for (double t = 0; t < rn.t.back(); t += 0.001) worst = std::max(worst, std::fabs(s2.k[AX_E].at(t) - rn.at(t)));
    // Klipper smooths all extruder motion while pressure advance is on (kin_extruder.c), so a fast
    // retraction lags a little; it must not be advanced (no bias either way on average)
    double bias = 0;
    int nb = 0;
    for (double t = 0; t < rn.t.back(); t += 0.001) bias += s2.k[AX_E].at(t + 0.0) - rn.at(t), nb++;
    CHECK(worst < 0.35 * espm);
    CHECK(std::fabs(bias / nb) < 0.05 * espm);
    printf("  pressure advance 0.08: E %.3f mm ahead at the end of the speed-up, %.3f mm at cruise\n", ahead,
           (sh.k[AX_E].at(tmid) - nom.at(tmid)) / espm);
}

static void test_bench() {
    auto now = []() {
        return uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now().time_since_epoch()).count());
    };
    BenchResult h = homing_bench(now);
    CHECK(h.steps == 49200 && h.commands > 0);
    BenchResult a = motion_bench(150, nullptr, now);
    ShapingConfig c;
    std::vector<double> A, T;
    shaper_def("mzv", 45, 0.1, A, T);
    c.x = c.y = shaper_pulses(A, T);
    c.pa = 0.05;
    c.pa_on = true;
    BenchResult b = motion_bench(150, &c, now);
    CHECK(a.sim_s > 10 && std::fabs(a.sim_s - b.sim_s) < 0.2);
    CHECK(a.steps > 100000 && b.steps > 100000);
    printf("  PC benchmark:\n%s", motion_bench_report(now).c_str());
}

// 2026-10-27: the integer shaper (int32/int64 test build) against the double one: at every grid point
// the same position to within one step, and exactly the same steps in all at the end.
static void test_shaper_fx_matches() {
    Plan pl;
    std::vector<std::vector<double>> zz;
    double e = 0;
    for (int i = 0; i < 60; i++) {   // zig-zags, short wiggles and long lines, extruding
        double x = (i % 2) ? 40 : 2, y = 1 + i * 0.6;
        e += 0.8;
        zz.push_back({x, y, 0, e});
        for (int j = 0; j < 6; j++) zz.push_back({x + ((i % 2) ? -0.3 : 0.3) * (j + 1), y + 0.2 * (j % 2), 0, e += 0.01});
    }
    zz.push_back({0, 0, 0, e - 1.5});   // a retraction with travel
    pl.run(zz, 150, 5000);
    ShapingConfig c;
    std::vector<double> A, T;
    shaper_def("mzv", 45, 0.1, A, T);
    c.x = shaper_pulses(A, T);
    shaper_def("ei", 38, 0.1, A, T);
    c.y = shaper_pulses(A, T);
    c.pa = 0.04;
    c.pa_on = true;
    for (int variant = 0; variant < 3; variant++) {
        ShapingConfig cv = c;
        if (variant == 1) cv.pa_on = false, cv.pa = 0;   // shaping only
        if (variant == 2) cv.x = cv.y = ShaperPulses();   // pressure advance only
        const double t0 = 1234.567891;                    // a host time with an awkward fraction
        struct Out { int64_t us; int64_t cum[NUM_AXES]; };
        std::vector<Out> od, of;
        int64_t cd[NUM_AXES] = {}, cf[NUM_AXES] = {};
        MotionShaper md(0.002);
        md.configure(cv);
        md.set_sink([&](double t, double dur, const int32_t* st) {
            Out o;
            o.us = MotionShaperFx::to_us(t + dur);
            for (int a = 0; a < NUM_AXES; a++) o.cum[a] = cd[a] += st[a];
            od.push_back(o);
        });
        MotionShaperFx mf(0.002);
        mf.configure(cv);
        mf.set_sink([&](int64_t, int64_t us1, const int32_t* st) {
            Out o;
            o.us = us1;
            for (int a = 0; a < NUM_AXES; a++) o.cum[a] = cf[a] += st[a];
            of.push_back(o);
        });
        double t = t0;
        for (size_t i = 0; i < pl.segs.size(); i++) {
            const Segment& s = pl.segs[i];
            bool last = i + 1 == pl.segs.size();
            md.push(t, s.n / 1e6, s.steps, last);
            mf.push(t, s.n / 1e6, s.steps, last);
            t += s.n / 1e6;
        }
        CHECK(md.idle() && mf.idle());
        CHECK(md.grid_segments() == mf.grid_segments());
        for (int a = 0; a < NUM_AXES; a++) CHECK(cd[a] == cf[a]);
        // position at each fx grid point vs the double output at the same moment (a step function)
        size_t j = 0;
        int64_t worst = 0, differ = 0;
        int64_t pos[NUM_AXES] = {};
        for (auto& o : of) {
            while (j < od.size() && od[j].us <= o.us + 1) {
                for (int a = 0; a < NUM_AXES; a++) pos[a] = od[j].cum[a];
                j++;
            }
            bool any = false;
            for (int a = 0; a < NUM_AXES; a++) {
                int64_t d = std::llabs(o.cum[a] - pos[a]);
                worst = std::max(worst, d);
                any |= d != 0;
            }
            differ += any;
        }
        CHECK(worst <= 1);
        CHECK(differ * 50 <= int64_t(of.size()));   // under 2 % of the grid points differ at all
        printf("  integer shaper (variant %d): %zu grid points, %lld differ by a step, none by more\n", variant,
               of.size(), (long long)differ);
    }
}

void test_shaping() {
    test_bench();
    test_shaper_defs();
    test_shaper_conserves_steps();
    test_shaper_reduces_vibration();
    test_pressure_advance();
    test_shaper_fx_matches();
}
