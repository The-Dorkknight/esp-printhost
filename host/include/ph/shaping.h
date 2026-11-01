// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Input shaping and pressure advance (2026-10-24), the maths of Klipper v0.13.0:
//   klippy/extras/shaper_defs.py      the shapers (zv, mzv, zvd, ei, 2hump_ei, 3hump_ei)
//   klippy/chelper/kin_shaper.c       shaped position = sum of A_i * x(t + t_i), pulses centred
//   klippy/chelper/kin_extruder.c     pressure advance: E + K * v, then a triangular average over
//                                     pressure_advance_smooth_time
// Klipper evaluates these at every step (itersolve). Here they are evaluated on a 2 ms time grid
// between the planner (which already cuts motion into constant-speed segments) and the step
// queues: the shaped / advanced position at each grid point, steps spread evenly in between. That
// costs a few dozen multiplications per 2 ms instead of per step, which is what makes it affordable
// on an ESP32-S3; the price is that the shaped motion is followed in 2 ms straight pieces (see
// docs/research/pressure-advance-input-shaping.md for how much that costs in vibration reduction).
#pragma once
#include "ph/motion.h"

#include <cstdint>
#include <deque>
#include <functional>
#include <string>
#include <vector>

#ifndef PH_PRO
#define PH_PRO 1   // the ESP32 DECKHAND build sets 0: no shaping, calibration, mesh... (flash space)
#endif
// Kinematics compiled in (2026-10-31). Cartesian is always there. DECKHAND has no room for everything:
// its default is CoreXY, and a separate DECKHAND build (-DPH_KIN_DELTA=1 -DPH_KIN_COREXY=0) has delta
// instead. CAPTAIN / SKIPPER and the PC build have both.
#ifndef PH_KIN_COREXY
#define PH_KIN_COREXY 1
#endif
#ifndef PH_KIN_DELTA
#define PH_KIN_DELTA PH_PRO
#endif
#ifndef PH_FIXED
#define PH_FIXED 0   // 1: the integer motion path (MotionShaperFx, integer step times); test builds, 2026-10-27
#endif

namespace ph {

// Klipper shaper_defs.py: A (amplitudes) and T (times, s) in their traditional order. False if the
// type is unknown. Types: zv mzv zvd ei 2hump_ei 3hump_ei.
bool shaper_def(const std::string& type, double freq, double damping, std::vector<double>& A, std::vector<double>& T);
double shaper_min_freq(const std::string& type);   // Klipper's min_freq for the type; 0 if unknown
extern const char* const SHAPER_TYPES[6];
const double SHAPER_VIBRATION_REDUCTION = 20.0;
const double DEFAULT_DAMPING_RATIO = 0.1;

// The pulses as kin_shaper.c uses them: reversed, normalised and shifted so that sum(a * t) = 0
// (constant speed passes unchanged). t may be negative (the past) or positive (the future).
struct ShaperPulses {
    int n = 0;
    double a[5] = {}, t[5] = {};
};
ShaperPulses shaper_pulses(const std::vector<double>& A, const std::vector<double>& T);

struct ShapingConfig {
    ShaperPulses x, y;             // n = 0: no shaping on that axis
    std::string x_type, y_type;    // for status / reports
    double x_freq = 0, y_freq = 0, x_damping = DEFAULT_DAMPING_RATIO, y_damping = DEFAULT_DAMPING_RATIO;
    double pa = 0;                 // pressure advance (s)
    double pa_smooth = 0.040;      // pressure_advance_smooth_time (s)
    bool pa_on = false;            // E goes through the advance / smoothing (pa > 0, or kept on for TUNING_TOWER)
    bool any() const { return x.n || y.n || pa_on; }
    // Only the advance value differs: that can change while moving (Klipper keeps a time-ordered
    // list of values; here each new segment takes the value current when it is planned).
    bool same_except_pa(const ShapingConfig& o) const;
};

// Between the planner and the step queues. Input: the planner's segments (start time, duration,
// whole steps per axis). Output: segments on its own grid of `grid` seconds, with the shaped X / Y
// and the advanced E; Z (and an axis with nothing to do) passes through, resampled on the grid.
// Positions are kept as whole steps, so nothing drifts.
class MotionShaper {
public:
    using Sink = std::function<void(double t0, double dur, const int32_t steps[NUM_AXES])>;
    // grid: 2 ms keeps 60-90 % of Klipper's vibration reduction from 30 to 80 Hz in the PC simulation
    // (test_shaping.cpp); 5 ms was fine to 60 Hz but made 80 Hz worse.
    explicit MotionShaper(double grid = 0.002) : grid_(grid) {}
    void set_sink(Sink s) { sink_ = std::move(s); }
    // Only while idle(): changes during motion would make the positions jump.
    void configure(const ShapingConfig& c);
    void set_pa(double k) { cfg_.pa = k; }   // any time: segments pushed from now on
    const ShapingConfig& config() const { return cfg_; }
    bool active() const { return cfg_.any(); }
    // How far into the future / past the output looks (s): the motion starts `lead` early and
    // ends `tail` late compared with the plain plan (Klipper: gen_steps_pre/post_active).
    double lead() const { return lead_; }
    double tail() const { return tail_; }
    // One planner segment. `last`: the motion comes to rest at its end (SEG_LAST): everything is
    // emitted up to the end of the tail. The next segment must not start before
    // t0 + dur + lead() + tail() (the toolhead leaves that gap).
    void push(double t0, double dur, const int32_t steps[NUM_AXES], bool last);
    bool idle() const { return knots_.empty(); }
    double out_end() const { return out_end_; }   // end time of the last output segment
    void reset();                                  // emergency stop: drop everything
    uint64_t grid_segments() const { return out_count_; }

private:
    struct Knot {
        double t;
        double p[NUM_AXES];   // whole-step position (as double)
        double k;             // pressure advance of the interval starting here (0: none)
    };
    double pos(int axis, double t) const;            // nominal, linear between knots, held outside
    double pa_smooth_pos(double tau) const;          // E with advance, triangular average
    double out_pos(int axis, double tau) const;
    void emit_until(double limit);                   // output segments ending at or before `limit`
    void trim();

    double grid_;
    ShapingConfig cfg_;
    Sink sink_;
    std::deque<Knot> knots_;
    int64_t cum_[NUM_AXES] = {};     // nominal position after everything pushed (steps)
    int64_t emitted_[NUM_AXES] = {}; // output position after everything emitted (steps)
    double out_t_ = 0, out_end_ = 0, lead_ = 0, tail_ = 0;
    uint64_t out_count_ = 0;
};

// The same as MotionShaper in whole numbers (2026-10-27, the int32/int64 test build): the ESP32-S3
// has no double-precision hardware, so every double operation above is done in software.
//  - times in microseconds (int64), positions in whole steps (int64), and between knots in steps
//    with 16 fraction bits; each knot keeps its slope (steps per us, 32 fraction bits), worked out
//    once per planner segment, so a position needs one multiply and a shift, no divide;
//  - shaper pulses: amplitudes with 16 fraction bits (summing to exactly 1), times in whole us;
//  - pressure advance smoothing: hardware single-precision float, on numbers relative to the
//    position at that moment (small, so float is precise to well under 0.001 step).
// Output: (start us, end us, steps per axis) on a grid of whole microseconds.
class MotionShaperFx {
public:
    using Sink = std::function<void(int64_t us0, int64_t us1, const int32_t steps[NUM_AXES])>;
    explicit MotionShaperFx(double grid = 0.002) : grid_us_(int64_t(grid * 1e6 + 0.5)) {}
    void set_sink(Sink s) { sink_ = std::move(s); }
    void configure(const ShapingConfig& c);
    void set_pa(double k) { cfg_.pa = k; }
    const ShapingConfig& config() const { return cfg_; }
    bool active() const { return cfg_.any(); }
    double lead() const { return double(lead_us_) * 1e-6; }
    double tail() const { return double(tail_us_) * 1e-6; }
    void push(double t0, double dur, const int32_t steps[NUM_AXES], bool last);
    bool idle() const { return knots_.empty(); }
    double out_end() const { return double(out_end_us_) * 1e-6; }
    void reset();
    uint64_t grid_segments() const { return out_count_; }
    static int64_t to_us(double t) { return int64_t(t * 1e6 + (t >= 0 ? 0.5 : -0.5)); }

private:
    struct Pulses {
        int n = 0;
        int32_t a[5] = {};    // amplitude, 1.0 = 65536
        int32_t t[5] = {};    // us
    };
    struct Knot {
        int64_t t;                 // us
        int64_t p[NUM_AXES];       // whole steps
        int64_t slope[NUM_AXES];   // to the next knot: steps per us << 32 (0 for the last knot)
        float ve;                  // E speed to the next knot (steps / s), for the advance
        float k;                   // pressure advance of the interval starting here (s)
    };
    int64_t pos_q16(int axis, int64_t t) const;
    int64_t pa_smooth_q16(int64_t tau) const;
    int64_t out_q16(int axis, int64_t tau) const;
    void emit_until(int64_t limit);
    void trim();
    void add_knot(int64_t t);

    int64_t grid_us_;
    ShapingConfig cfg_;
    Pulses px_, py_;
    float pa_h_ = 0;                 // half the smoothing time (s)
    Sink sink_;
    std::deque<Knot> knots_;
    int64_t cum_[NUM_AXES] = {};
    int64_t emitted_[NUM_AXES] = {};
    int64_t out_t_ = 0, out_end_us_ = 0, lead_us_ = 0, tail_us_ = 0;
    uint64_t out_count_ = 0;
};

}  // namespace ph
