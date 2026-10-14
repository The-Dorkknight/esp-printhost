// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Tests for ph::Planner / ph::Engine. Every motion check goes through the REAL
// segq_core (test/segq_sim.h) and, where it matters, is reconstructed from the
// recorded step timestamps only, independent of the planner's own bookkeeping.
#include "check.h"
#include "ph/engine.h"
#include "ph/planner.h"
#include "segq_sim.h"
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

using namespace ph;

namespace {

const double HZ = 20000;
const double SPM[4] = {80, 80, 400, 93};

struct Out {
    std::vector<Segment> segs;
    std::vector<Item> items;
};

Out run_prog(Engine& e, const std::vector<std::string>& prog, bool flush = true) {
    for (auto& l : prog) e.feed_line(l);
    if (flush) e.flush();
    Out o;
    o.items = e.drain();
    for (auto& it : o.items)
        if (it.type == Item::SEGMENT) o.segs.push_back(it.seg);
    return o;
}

// XY path speed (mm/s) over ticks [t0, t1)
double xy_speed(const SegqSim& s, uint64_t t0, uint64_t t1) {
    double dx = double(s.pos_at(0, t1) - s.pos_at(0, t0)) / SPM[0];
    double dy = double(s.pos_at(1, t1) - s.pos_at(1, t0)) / SPM[1];
    return std::sqrt(dx * dx + dy * dy) / (double(t1 - t0) / HZ);
}
double axis_speed(const SegqSim& s, int a, uint64_t t0, uint64_t t1) {
    return std::fabs(double(s.pos_at(a, t1) - s.pos_at(a, t0))) / SPM[a] / (double(t1 - t0) / HZ);
}
std::vector<double> windows(const SegqSim& s, uint64_t w) {
    std::vector<double> v;
    for (uint64_t t = 0; t + w <= s.tick; t += w) v.push_back(xy_speed(s, t, t + w));
    return v;
}

// ---------------------------------------------------------------- tests

void t_square() {
    Engine e;
    Out o = run_prog(e, {"G90", "M83", "G1 X20 E0.7 F6000", "G1 Y20 E0.7", "G1 X0 E0.7",
                         "G1 Y0 E0.7", "G1 Z0.4 F600"});
    SegqSim s;
    s.run(o.segs);
    CHECK(s.invalid == 0);
    CHECK(s.pos(0) == 0 && s.pos(1) == 0);
    CHECK(s.pos(2) == 160);           // 0.4 * 400
    CHECK(s.pos(3) == 260);           // 2.8 * 93 = 260.4
    CHECK(s.c.underruns == 0);
    int64_t mx = 0, my = 0;
    for (auto v : s.cum[0]) mx = std::max(mx, v);
    for (auto v : s.cum[1]) my = std::max(my, v);
    CHECK(mx == 1600 && my == 1600);  // 20 mm * 80
    bool seg_ok = true, last_ok = true;
    for (size_t i = 0; i < o.segs.size(); i++) {
        if (o.segs[i].n > 160 || o.segs[i].n < 1) seg_ok = false;
        if (((o.segs[i].flags & SEG_LAST) != 0) != (i + 1 == o.segs.size())) last_ok = false;
    }
    CHECK(seg_ok);
    CHECK(last_ok);                   // SEG_LAST only on the very last segment
}

void t_profile() {
    // long move: trapezoid
    {
        Engine e;
        Out o = run_prog(e, {"G90", "G1 X100 F6000"});
        SegqSim s;
        s.run(o.segs);
        CHECK(s.invalid == 0 && s.pos(0) == 8000);
        auto w20 = windows(s, 400);
        double peak = 0; int cruise = 0;
        for (double v : w20) { peak = std::max(peak, v); if (v >= 97.0) cruise++; }
        CHECK(peak <= 100 * 1.02);
        CHECK(cruise >= 40);              // ~1 s of cruise = 50 windows
        auto w10 = windows(s, 200);
        double amax = 0;
        for (size_t i = 0; i + 1 < w10.size(); i++)
            amax = std::max(amax, std::fabs(w10[i + 1] - w10[i]) / 0.01);
        CHECK(amax <= 1.15 * 3000);
        CHECK(amax >= 0.8 * 3000);        // it really does accelerate at ~max_accel
        double T = double(s.stamp[0].back()) / HZ, Tan = 100.0 / 100.0 + 100.0 / 3000.0;
        CHECK(std::fabs(T - Tan) <= 0.02 * Tan);
    }
    // short move: triangle (peak sqrt(a*L) = 94.9 mm/s < 100)
    {
        Engine e;
        Out o = run_prog(e, {"G90", "G1 X3 F6000"});
        SegqSim s;
        s.run(o.segs);
        CHECK(s.invalid == 0 && s.pos(0) == 240);
        double peak = 0;
        for (double v : windows(s, 80)) peak = std::max(peak, v);
        double pan = std::sqrt(3000.0 * 3.0), Tan = 2 * std::sqrt(3.0 / 3000.0);
        CHECK(peak <= pan * 1.05 && peak >= pan * 0.85);
        CHECK(peak < 97);                 // never reaches the 100 mm/s nominal
        double T = double(s.stamp[0].back()) / HZ;
        CHECK(std::fabs(T - Tan) <= 0.08 * Tan);
    }
}

void t_junctions() {
    const uint64_t W = 80;   // 4 ms
    {   // collinear: no slow-down
        Engine e;
        Out o = run_prog(e, {"G90", "G1 X50 F6000", "G1 X100"});
        SegqSim s; s.run(o.segs);
        CHECK(s.invalid == 0 && s.pos(0) == 8000);
        uint64_t t = s.stamp[0][3999];
        CHECK(xy_speed(s, t - W, t) >= 90 && xy_speed(s, t, t + W) >= 90);
        CHECK(o.segs.size() > 0);
        bool no_last_mid = true;
        for (size_t i = 0; i + 1 < o.segs.size(); i++) if (o.segs[i].flags & SEG_LAST) no_last_mid = false;
        CHECK(no_last_mid);
    }
    double v90a, v90b;
    {   // 90 degrees: v_j = sqrt(a*jd*s/(1-s)), s = sin 45 -> 19 mm/s
        Engine e;
        Out o = run_prog(e, {"G90", "G1 X50 F6000", "G1 Y50"});
        SegqSim s; s.run(o.segs);
        CHECK(s.invalid == 0 && s.pos(0) == 4000 && s.pos(1) == 4000);
        uint64_t t = s.stamp[0][3999];
        v90a = axis_speed(s, 0, t - W, t);
        v90b = axis_speed(s, 1, t, t + W);
        CHECK(v90a >= 12 && v90a <= 45);
        CHECK(v90b >= 12 && v90b <= 45);
    }
    {   // 180 degree reversal: stops
        Engine e;
        Out o = run_prog(e, {"G90", "G1 X50 F6000", "G1 X0"});
        SegqSim s; s.run(o.segs);
        CHECK(s.invalid == 0 && s.pos(0) == 0);
        uint64_t t = s.stamp[0][3999];
        double a = axis_speed(s, 0, t - W, t), b = axis_speed(s, 0, t, t + W);
        CHECK(a <= 12 && b <= 12);
        CHECK(a < v90a * 0.6 && b < v90b * 0.6);
    }
    {   // lookahead: a slow move after a fast one forces the fast one to brake early
        Engine e;
        Out o = run_prog(e, {"G90", "G1 X100 F6000", "G1 X110 F300"});
        SegqSim s; s.run(o.segs);
        uint64_t t = s.stamp[0][7999];
        double v = xy_speed(s, t - 40, t + 40);     // 4 ms around the junction
        CHECK(v <= 5.0 * 1.3 + 3 && v >= 3);        // ~5 mm/s (F300)
    }
}

// ---- fuzz
struct Rng {
    uint64_t s;
    uint32_t next() { s = s * 6364136223846793005ULL + 1442695040888963407ULL; return uint32_t(s >> 33); }
    int range(int lo, int hi) { return lo + int(next() % uint32_t(hi - lo + 1)); }
    double u() { return next() / 2147483648.0; }
};
int64_t fdiv(int64_t a, int64_t b) { int64_t q = a / b; if ((a % b != 0) && ((a < 0) != (b < 0))) q--; return q; }
// round-half-up of units/64 * spm
int64_t exp_steps(int64_t units, int axis) { return fdiv(2 * units * int64_t(SPM[axis]) + 64, 128); }
bool is_tie(int64_t units, int axis) {
    int64_t m = (2 * units * int64_t(SPM[axis])) % 128;
    if (m < 0) m += 128;
    return m == 64;
}
std::string fmt(int64_t units) {
    char b[64];
    snprintf(b, sizeof b, "%.6f", double(units) / 64.0);
    return b;
}

std::vector<std::string> gen_program(uint32_t seed, int lines, int64_t exp_abs[4], bool pa) {
    Rng r{seed * 7919u + 13};
    std::vector<std::string> prog;
    int64_t pos[4] = {0, 0, 0, 0};   // XYZ logical == absolute, E absolute (units of 1/64 mm)
    int64_t elog = 0;
    bool rel = false, erel = false;
    prog.push_back("G90");
    prog.push_back("M82");
    if (pa) prog.push_back("M900 K" + std::to_string(0.01 * (1 + seed % 6)));
    for (int n = 0; n < lines; n++) {
        int k = r.range(0, 99);
        if (k < 6) { rel = !rel; prog.push_back(rel ? "G91" : "G90"); continue; }
        if (k < 11) { erel = !erel; prog.push_back(erel ? "M83" : "M82"); continue; }
        if (k < 14) { prog.push_back("G92 E0"); elog = 0; continue; }
        if (k < 18) { prog.push_back("G4 P" + std::to_string(r.range(0, 30))); continue; }
        if (pa && k < 20) { prog.push_back("M900 K" + std::to_string(0.01 * r.range(0, 8))); continue; }
        std::string line = (r.range(0, 9) == 0) ? "G0" : "G1";
        bool any = false;
        const char L[3] = {'X', 'Y', 'Z'};
        int64_t np[4] = {pos[0], pos[1], pos[2], pos[3]};
        for (int a = 0; a < 3; a++) {
            int prob = a < 2 ? 80 : 12;
            if (r.range(0, 99) >= prob) continue;
            for (int tries = 0; tries < 8; tries++) {
                int64_t v;
                int64_t lim = a < 2 ? 200 * 64 : 20 * 64;
                if (rel) {
                    int64_t d = (r.range(0, 4) == 0) ? r.range(1, 8) * (r.range(0, 1) ? 1 : -1)
                                                     : r.range(-40 * 64, 40 * 64);
                    if (a == 2) d = r.range(-4 * 64, 4 * 64);
                    if (pos[a] + d > lim * 2 || pos[a] + d < -lim) d = -d;
                    v = d;
                    if (is_tie(pos[a] + v, a)) continue;
                    np[a] = pos[a] + v;
                } else {
                    v = r.range(0, int(lim));
                    if (is_tie(v, a)) continue;
                    np[a] = v;
                }
                line += std::string(" ") + L[a] + fmt(v);
                any = true;
                break;
            }
        }
        if (r.range(0, 99) < 55) {
            for (int tries = 0; tries < 8; tries++) {
                int64_t d = r.range(-6 * 64, 6 * 64);
                int64_t newe = pos[3] + d;
                if (is_tie(newe, 3)) continue;
                int64_t v = (rel || erel) ? d : elog + d;
                line += " E" + fmt(v);
                np[3] = newe;
                elog += d;
                any = true;
                break;
            }
        }
        if (!any) continue;
        if (r.range(0, 99) < 70) {
            int c = r.range(0, 99);
            double f = c < 70 ? 3000 + r.u() * 17000 : c < 90 ? 20000 + r.u() * 1e8 : 600 + r.u() * 2400;
            line += " F" + std::to_string(int64_t(f));
        }
        for (int a = 0; a < 4; a++) pos[a] = np[a];
        prog.push_back(line);
    }
    for (int a = 0; a < 4; a++) exp_abs[a] = exp_steps(pos[a], a);
    return prog;
}

void t_fuzz() {
    int bad = 0, total_segs = 0;
    for (uint32_t seed = 1; seed <= 40; seed++) {
        int64_t want[4];
        auto prog = gen_program(seed, 250, want, seed % 2 == 0);
        Engine e;
        Out o = run_prog(e, prog);
        SegqSim s(16, 1);
        s.record = false;
        s.run(o.segs);
        total_segs += int(o.segs.size());
        bool ok = s.invalid == 0 && s.c.underruns == 0 && !o.segs.empty();
        for (int a = 0; a < 4; a++) ok = ok && s.pos(a) == want[a];
        for (auto& sg : o.segs) {
            ok = ok && sg.n >= 1 && sg.n <= SEGQ_MAX_N;
            for (int a = 0; a < 4; a++) ok = ok && 2 * int64_t(std::llabs(sg.steps[a])) <= int64_t(sg.n);
        }
        if (!e.warnings().empty()) ok = false;
        if (!ok) {
            bad++;
            printf("fuzz seed %u failed: invalid=%d pos=%lld,%lld,%lld,%lld want=%lld,%lld,%lld,%lld\n", seed,
                   s.invalid, (long long)s.pos(0), (long long)s.pos(1), (long long)s.pos(2),
                   (long long)s.pos(3), (long long)want[0], (long long)want[1], (long long)want[2],
                   (long long)want[3]);
        }
    }
    CHECK(bad == 0);
    CHECK(total_segs > 5000);
}

void t_limits() {
    // absurd feeds must be clamped to the step-rate / axis limits, never rejected
    {
        Engine e;
        Out o = run_prog(e, {"G90", "G1 X200 Y200 E40 F100000000"});
        SegqSim s; s.run(o.segs);
        CHECK(s.invalid == 0 && s.pos(0) == 16000 && s.pos(1) == 16000 && s.pos(3) == 3720);
        double peak = 0;
        for (double v : windows(s, 400)) peak = std::max(peak, v);
        // each axis <= 0.45 * 20000 / 80 = 112.5 mm/s -> 159 mm/s along the diagonal
        CHECK(peak <= 112.5 * std::sqrt(2.0) * 1.02 && peak >= 140);
    }
    {
        Engine e;
        Out o = run_prog(e, {"G90", "G1 Z20 F100000000"});
        SegqSim s; s.run(o.segs);
        CHECK(s.invalid == 0 && s.pos(2) == 8000);
        double peak = 0;
        for (uint64_t t = 0; t + 400 <= s.tick; t += 400) peak = std::max(peak, axis_speed(s, 2, t, t + 400));
        CHECK(peak <= 20 * 1.02 && peak >= 18);
    }
    {
        Engine e;
        Out o = run_prog(e, {"M83", "G1 E-50 F100000000", "G1 E50"});
        SegqSim s; s.run(o.segs);
        CHECK(s.invalid == 0 && s.pos(3) == 0);
        double peak = 0;
        for (uint64_t t = 0; t + 400 <= s.tick; t += 400) peak = std::max(peak, axis_speed(s, 3, t, t + 400));
        CHECK(peak <= 60 * 1.02 && peak >= 55);
    }
}

void t_drift() {
    Engine e;
    std::vector<std::string> prog = {"G91", "M83"};
    for (int i = 0; i < 10000; i++) prog.push_back("G1 X0.01 E0.0005 F6000");
    Out o = run_prog(e, prog);
    SegqSim s; s.run(o.segs);
    CHECK(s.invalid == 0);
    CHECK(s.pos(0) == 8000);      // 100 mm exactly
    CHECK(s.pos(3) == 465);       // 5 mm * 93
    double T = double(s.tick) / HZ, Tan = 1.0 + 1.0 / 30.0;
    CHECK(std::fabs(T - Tan) <= 0.04 * Tan);   // tiny collinear moves must still run at speed
    // and the opposite direction lands back on zero
    prog.clear();
    for (int i = 0; i < 10000; i++) prog.push_back("G1 X-0.01 E-0.0005");
    Out o2 = run_prog(e, prog);
    SegqSim s2; s2.run(o.segs); s2.run(o2.segs);
    CHECK(s2.invalid == 0 && s2.pos(0) == 0 && s2.pos(3) == 0);
}

void t_tiny_pa() {
    // sub-step moves + pressure advance exercise the "stretch" path of the planner
    Engine e;
    std::vector<std::string> prog = {"G91", "M83", "M900 K0.05"};
    for (int i = 0; i < 10000; i++) prog.push_back("G1 X0.01 E0.0005 F6000");
    for (int i = 0; i < 2000; i++) prog.push_back("G1 X0.003 Y0.002 E0.0001 F9000");
    prog.push_back("G1 E-5 F20000");
    prog.push_back("G1 E5 F20000");
    Out o = run_prog(e, prog);
    SegqSim s; s.run(o.segs);
    CHECK(s.invalid == 0 && s.c.underruns == 0);
    CHECK(s.pos(0) == 8480 && s.pos(1) == 320 && s.pos(3) == 484);
    CHECK(e.planner().stretched_segments() > 0);
}

void t_state() {
    Engine e;
    e.feed_line("G90"); e.feed_line("G21");
    e.feed_line("G1 X10 Y10 F3000");
    NEAR(e.position(0), 10, 1e-9);
    CHECK(e.feed_mm_min() == 3000);
    e.feed_line("G91"); e.feed_line("G1 X5");
    NEAR(e.position(0), 15, 1e-9);
    CHECK(e.feed_mm_min() == 3000 && e.xyz_relative());
    e.feed_line("G90"); e.feed_line("G92 X0");
    NEAR(e.position(0), 0, 1e-9);
    NEAR(e.planner().position(0), 15, 1e-9);
    e.feed_line("G1 X5");
    NEAR(e.planner().position(0), 20, 1e-9);
    NEAR(e.planner().position(1), 10, 1e-9);
    // E modes
    e.feed_line("G92 E0");
    e.feed_line("M83"); e.feed_line("G1 E1"); e.feed_line("G1 E1");
    NEAR(e.position(3), 2, 1e-9);
    NEAR(e.planner().position(3), 2, 1e-9);
    e.feed_line("M82"); e.feed_line("G1 E5");
    NEAR(e.planner().position(3), 5, 1e-9);
    e.feed_line("G92 E0"); e.feed_line("G1 E1");
    NEAR(e.planner().position(3), 6, 1e-9);        // absolute again: 0 -> 1 is +1
    e.feed_line("G91");
    CHECK(e.e_relative());                         // G91 also makes E relative
    e.feed_line("G1 E1");
    NEAR(e.planner().position(3), 7, 1e-9);
    e.feed_line("G90");
    CHECK(!e.e_relative());
    // M221 / M220
    e.feed_line("M221 S50"); e.feed_line("M83"); e.feed_line("G1 X6 E2");
    NEAR(e.planner().position(3), 8, 1e-9);        // flow 50%: +1
    NEAR(e.position(3), 4.0, 1e-9);                // logical E sees the full 2 mm (2 -> 4)
    CHECK(e.flow_factor() == 50);
    e.feed_line("M220 S150");
    CHECK(e.speed_factor() == 150);
    e.feed_line("M900 K0.05");
    NEAR(e.planner().pressure_advance(), 0.05, 1e-6);
    e.feed_line("M900 K0");
    NEAR(e.planner().pressure_advance(), 0.0, 1e-12);
    e.feed_line("M204 S1000");
    NEAR(e.planner().accel(), 1000, 1e-9);
    e.feed_line("M204 S99999");
    NEAR(e.planner().accel(), 3000, 1e-9);         // clamped to max_accel
    CHECK(e.warnings().empty());
    // warnings, never a crash
    CHECK(e.feed_line("G29"));
    CHECK(e.feed_line("T1"));
    CHECK(e.feed_line("M999 S1"));
    CHECK(e.feed_line("G20"));
    CHECK(e.warnings().size() == 4);
    CHECK(!e.feed_line("G1 X1.2.3"));
    CHECK(e.warnings().size() == 5);
    CHECK(e.feed_line("; comment only") && e.feed_line(""));
    CHECK(e.warnings().size() == 5);
    CHECK(e.feed_line("M220 S0") && e.warnings().size() == 6);
    e.flush();
    auto items = e.drain();
    CHECK(!items.empty());

    // timing: M220 doubles the speed (checked through the simulator)
    auto dur = [](const char* extra) {
        Engine g;
        std::vector<std::string> p = {"G90"};
        if (extra) p.push_back(extra);
        p.push_back("G1 X100 F3000");
        Out o = run_prog(g, p);
        SegqSim s; s.run(o.segs);
        return double(s.tick) / HZ;
    };
    double d1 = dur(nullptr), d2 = dur("M220 S200");
    NEAR(d1, 100.0 / 50 + 50.0 / 3000, 0.04 * d1);
    NEAR(d2, 100.0 / 100 + 100.0 / 3000, 0.04 * d2);
    // M221 changes E steps
    Engine g;
    Out o = run_prog(g, {"M83", "M221 S50", "G1 X10 E2 F3000"});
    SegqSim s; s.run(o.segs);
    CHECK(s.pos(3) == 93);
    // G92 shifts XYZ but the machine still goes to the right place
    Engine h;
    Out o2 = run_prog(h, {"G90", "G1 X10 F6000", "G92 X0", "G1 X5", "G92 E0"});
    SegqSim s2; s2.run(o2.segs);
    CHECK(s2.pos(0) == 1200);
}

void t_actions() {
    Engine e;
    Out o = run_prog(e, {"G90", "G1 X10 F6000", "M106 S255", "G1 X20", "M104 S210 T0", "M109 S200",
                         "G1 X30", "G28 X Y", "M84", "G1 X5"});
    std::vector<std::string> kinds;
    int64_t xsum = 0;
    std::vector<int64_t> at;
    bool last_before_sync = true;
    for (size_t i = 0; i < o.items.size(); i++) {
        const Item& it = o.items[i];
        if (it.type == Item::SEGMENT) { xsum += it.seg.steps[0]; continue; }
        kinds.push_back(it.action.kind);
        at.push_back(xsum);
        if (it.action.kind == "M109" || it.action.kind == "G28" || it.action.kind == "M84") {
            // previous item must be a segment carrying SEG_LAST
            size_t j = i;
            while (j > 0 && o.items[j - 1].type != Item::SEGMENT) j--;   // skip adjacent actions
            if (j == 0 || !(o.items[j - 1].seg.flags & SEG_LAST)) last_before_sync = false;
        }
        if (it.action.kind == "M106") {
            CHECK(it.action.get('S') == 255);
            CHECK(!it.action.has('T'));
        }
        if (it.action.kind == "M104") CHECK(it.action.get('S') == 210 && it.action.get('T') == 0);
        if (it.action.kind == "G28") CHECK(it.action.has('X') && it.action.has('Y') && !it.action.has('Z'));
    }
    CHECK(kinds.size() == 5);
    if (kinds.size() == 5) {
        CHECK(kinds[0] == "M106" && kinds[1] == "M104" && kinds[2] == "M109" && kinds[3] == "G28" &&
              kinds[4] == "M84");
        CHECK(at[0] == 800);     // M106 sits exactly between the two moves
        CHECK(at[1] == 1600 && at[2] == 1600);
        CHECK(at[3] == 2400 && at[4] == 2400);
    }
    CHECK(last_before_sync);
    // M106 did not break the motion: collinear moves around it ran at speed
    SegqSim s; s.run(o.segs);
    CHECK(s.invalid == 0);
    // after G28 X Y the logical X is 0, so "G1 X5" goes to abs 30 + 5
    CHECK(s.pos(0) == (30 + 5) * 80);
    NEAR(e.position(0), 5, 1e-9);
    // sink mode
    Engine f;
    int nseg = 0, nact = 0;
    f.set_sink([&](const Item& it) { (it.type == Item::SEGMENT ? nseg : nact)++; });
    f.feed_line("G1 X10 F6000"); f.feed_line("M107"); f.flush();
    CHECK(nseg > 0 && nact == 1 && f.drain().empty());
}

void t_pa() {
    std::vector<std::string> prog = {"G90", "M83", "G1 Z0.2 F600"};
    for (int i = 0; i < 6; i++) {
        prog.push_back("G1 X40 E0.25 F4800");
        prog.push_back("G1 Y10 E0.25");
        prog.push_back("G1 E-0.75 F2400");
        prog.push_back("G1 X0 Y0 F9000");
        prog.push_back("G1 E0.75 F2400");
        prog.push_back("G1 X20 E1.5 F3600");
    }
    // total E = 6*(0.25+0.25-0.75+0.75+1.5) = 12.0 mm = 1116 steps
    SegqSim base;
    {
        Engine e; Out o = run_prog(e, prog); base.run(o.segs);
        CHECK(base.invalid == 0 && base.pos(3) == 1116);
    }
    for (double k : {0.02, 0.05, 0.12}) {
        std::vector<std::string> p2 = prog;
        p2.insert(p2.begin() + 1, "M900 K" + std::to_string(k));
        Engine e; Out o = run_prog(e, p2);
        SegqSim s; s.run(o.segs);
        CHECK(s.invalid == 0 && s.c.underruns == 0);
        CHECK(s.pos(3) == 1116);                          // exact after flush
        CHECK(s.pos(0) == base.pos(0) && s.pos(1) == base.pos(1) && s.pos(2) == base.pos(2));
        // and the advance is really applied: E position differs mid-print
        int diff = 0;
        uint64_t T = std::min(s.tick, base.tick);
        for (uint64_t t = 0; t < T; t += 100)
            if (s.pos_at(3, t) != base.pos_at(3, t)) diff++;
        CHECK(diff > 50);
        CHECK(o.segs.back().flags & SEG_LAST);
    }
}

void t_dwell() {
    {
        Engine e;
        Out o = run_prog(e, {"G4 P500"});
        uint64_t sum = 0;
        for (auto& s : o.segs) { sum += s.n; for (int a = 0; a < 4; a++) CHECK(s.steps[a] == 0); }
        CHECK(sum == 10000);
        CHECK(!o.segs.empty() && (o.segs.back().flags & SEG_LAST));
    }
    {
        Engine e;
        Out o = run_prog(e, {"G4 S10 P250"});
        uint64_t sum = 0, mx = 0;
        for (auto& s : o.segs) { sum += s.n; mx = std::max<uint64_t>(mx, s.n); }
        CHECK(sum == 205000);
        CHECK(mx <= 20000 && o.segs.size() >= 11);
        SegqSim s; s.run(o.segs);
        CHECK(s.invalid == 0 && s.tick == 205000);
    }
    {   // move, dwell, move: dwell sits between them and flushes
        Engine e;
        Out o = run_prog(e, {"G90", "G1 X10 F6000", "G4 P100", "G1 X20"});
        uint64_t dw = 0;
        int64_t x = 0;
        bool order_ok = true, seen_dwell = false;
        for (size_t i = 0; i < o.segs.size(); i++) {
            const Segment& s = o.segs[i];
            bool zero = !s.steps[0] && !s.steps[1] && !s.steps[2] && !s.steps[3];
            if (zero) { dw += s.n; seen_dwell = true; if (x != 800) order_ok = false; }
            x += s.steps[0];
            if (!zero && seen_dwell && x <= 800) order_ok = false;
        }
        CHECK(dw == 2000 && order_ok && x == 1600);
        SegqSim s; s.run(o.segs);
        CHECK(s.invalid == 0 && s.pos(0) == 1600);
    }
}

void t_determinism_perf() {
    int64_t want[4];
    auto prog = gen_program(77, 400, want, true);
    Engine a, b;
    Out oa = run_prog(a, prog), ob = run_prog(b, prog);
    bool same = oa.segs.size() == ob.segs.size();
    for (size_t i = 0; same && i < oa.segs.size(); i++)
        same = oa.segs[i].n == ob.segs[i].n && oa.segs[i].flags == ob.segs[i].flags &&
               oa.segs[i].tag == ob.segs[i].tag &&
               std::memcmp(oa.segs[i].steps, ob.segs[i].steps, sizeof oa.segs[i].steps) == 0;
    CHECK(same && !oa.segs.empty());

    // 100k small moves (a circle of 0.1 mm chords, with E and pressure advance)
    auto t0 = std::chrono::steady_clock::now();
    Engine e;
    uint64_t nseg = 0;
    e.set_sink([&](const Item& it) { if (it.type == Item::SEGMENT) nseg++; });
    e.feed_line("G90"); e.feed_line("M83"); e.feed_line("M900 K0.04");
    char buf[96];
    for (int i = 0; i < 100000; i++) {
        double ang = i * 0.005;
        snprintf(buf, sizeof buf, "G1 X%.4f Y%.4f E0.004 F6000", 100 + 20 * std::cos(ang), 100 + 20 * std::sin(ang));
        e.feed_line(buf);
    }
    e.flush();
    double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    printf("engine perf: 100k moves -> %llu segments in %.2f s\n", (unsigned long long)nseg, secs);
    CHECK(secs < 5.0);
    CHECK(nseg > 90000);
}

}  // namespace

void test_engine() {
    t_square();
    t_profile();
    t_junctions();
    t_fuzz();
    t_limits();
    t_drift();
    t_tiny_pa();
    t_state();
    t_actions();
    t_pa();
    t_dwell();
    t_determinism_perf();
}
