// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// See toolhead.h. Klipper references (v0.13.0): klippy/toolhead.py, gcode_move.py, homing.py,
// stepper.py (PrinterRail), extras/probe.py, extras/tmc.py (sensorless), chelper/stepcompress.c
// (the idea of fitting interval/count/add within max_error; this is a simpler greedy fit).
#include "ph/toolhead.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace ph {

static std::string f3(double v) {
    char b[32];
    snprintf(b, sizeof b, "%.3f", v);
    return b;
}

// ---------------------------------------------------------------- StepQueue

void StepQueue::init(int oid, bool invert_dir, uint32_t max_error_ticks, uint32_t min_interval_ticks) {
    oid_ = oid;
    invert_ = invert_dir;
    max_err_ = max_error_ticks;
    min_iv_ = min_interval_ticks;
}

void StepQueue::push_run(int dir, double first, double interval, uint32_t count) {
    // Step times in 1/65536 tick fixed point: the ESP32-S3 has no double-precision hardware, so the
    // per-step work below stays in integers (2026-10-24; the error is under a tick per 60,000 steps).
    push_run_fx(dir, int64_t(std::llround(first * 65536.0)), int64_t(std::llround(interval * 65536.0)), count);
}

void StepQueue::push_run_fx(int dir, int64_t first_fx, int64_t interval_fx, uint32_t count) {
    if (!count) return;
    Run r;
    r.count = count;
    r.done = 0;
    r.dir = int8_t(dir);
    r.first_fx = first_fx;
    r.iv_fx = interval_fx;
    runs_.push_back(r);
    pending_steps_ += count;
}

double StepQueue::last_pushed() const {
    if (runs_.empty()) return double(last_clock_);
    const Run& r = runs_.back();
    return double(r.first_fx + r.iv_fx * int64_t(r.count - 1)) / 65536.0;
}

const char* MotionPerf::name(int i) {
    static const char* const N[] = {"planner", "shaping", "step times", "compression", "sending"};
    return i >= 0 && i < MotionPerf::N ? N[i] : "?";
}

void StepQueue::reset() {
    need_reset_ = true;
    cur_dir_ = -1;
}

int StepQueue::flush(uint64_t upto, std::vector<std::string>& out, std::vector<uint64_t>* ends) {
    int moves = 0;
    std::vector<int64_t> win;
    const std::string O = std::to_string(oid_);
    while (!runs_.empty()) {
        // the next steps in one direction, at or before `upto` (at most 256 at a time: the ESP32-S3
        // has no double-precision hardware, so the fit works on short windows in float)
        win.clear();
        int dir = runs_.front().dir;
        const int64_t upto_fx = upto >= (uint64_t(1) << 46) ? INT64_MAX : int64_t(upto) << 16;
        for (size_t ri = 0; ri < runs_.size() && win.size() < 256; ri++) {
            const Run& r = runs_[ri];
            if (r.dir != dir) break;
            bool stop = false;
            int64_t c = r.first_fx + r.iv_fx * int64_t(r.done);
            for (uint32_t j = r.done; j < r.count && win.size() < 256; j++, c += r.iv_fx) {
                if (c > upto_fx) {
                    stop = true;
                    break;
                }
                win.push_back((c + 32768) >> 16);
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
            if (best == win.size()) break;   // the whole window in one command: no candidate can beat it
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
        if (ends) ends->push_back(uint64_t(best_end));
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

// ---------------------------------------------------------------- Toolhead

static const double START_DELAY = 0.25;    // a move starts this long after it is planned (s)
static const double SEND_AHEAD = 0.6;      // steps are sent this far ahead of their time (s)
static const char AXL[4] = {'X', 'Y', 'Z', 'E'};

Toolhead::Toolhead(BoardPlan& plan, ToolheadIO io) : plan_(plan), io_(std::move(io)) {
    const char* names[NUM_AXES] = {"stepper_x", "stepper_y", "stepper_z", "extruder"};
    for (int a = 0; a < NUM_AXES; a++) {
        StepperObj* s = plan_.stepper(names[a]);
        if (s) si_[a] = int(s - plan_.steppers.data());
    }
    sq_.resize(plan_.steppers.size());
    for (size_t i = 0; i < plan_.steppers.size(); i++) {
        const StepperObj& s = plan_.steppers[i];
        uint32_t min_iv = std::max<uint32_t>(2 * s.pulse_ticks + 10, uint32_t(plan_.clock_freq * 2e-6));
        sq_[i].init(s.oid, s.invert_dir, uint32_t(plan_.clock_freq * 0.000025), min_iv);   // Klipper max_error 25 us
    }
    for (int a = 0; a < 3; a++)
        if (si_[a] < 0 || !plan_.steppers[size_t(si_[a])].has_rail) why_not_ += std::string(why_not_.empty() ? "" : ", ") + "[" + names[a] + "]";
    if (!why_not_.empty()) why_not_ = "no " + why_not_ + " in printer.cfg";
    else if (!plan_.printer.present || plan_.printer.max_velocity <= 0) why_not_ = "no [printer] section with max_velocity / max_accel";
    if (why_not_.empty() && plan_.printer.corexy) {
        double a = plan_.steppers[size_t(si_[0])].steps_per_mm(), b = plan_.steppers[size_t(si_[1])].steps_per_mm();
        if (std::fabs(a - b) > 1e-9 * a)
            why_not_ = "corexy needs the same steps per mm on stepper_x and stepper_y (rotation_distance, microsteps)";
        corexy_ = true;
    }
    ok_ = why_not_.empty();
    if (!ok_) return;
    const PrinterLimits& pr = plan_.printer;
    for (int a = 0; a < NUM_AXES; a++)
        pcfg_.steps_per_mm[a] = si_[a] >= 0 ? plan_.steppers[size_t(si_[a])].steps_per_mm() : 1.0;
    pcfg_.max_velocity = pr.max_velocity;
    pcfg_.max_z_velocity = pr.max_z_velocity > 0 ? pr.max_z_velocity : pr.max_velocity;
    pcfg_.max_accel = pr.max_accel;
    // Klipper's defaults for a 0.4 mm nozzle and 1.75 mm filament: max cross section 4 * 0.4^2
    double ratio = 4 * 0.4 * 0.4 / (M_PI / 4 * 1.75 * 1.75);
    pcfg_.max_e_velocity = pr.max_extrude_only_velocity > 0 ? pr.max_extrude_only_velocity : pr.max_velocity * ratio;
    pcfg_.max_e_accel = pr.max_accel * ratio;
    pcfg_.junction_deviation = pr.square_corner_velocity * pr.square_corner_velocity * (std::sqrt(2.0) - 1.0) / pr.max_accel;
    if (pcfg_.junction_deviation <= 0) pcfg_.junction_deviation = 0.001;
    pcfg_.dda_hz = 1e6;          // segment time unit: 1 us
    pcfg_.segment_ms = 5;
    // 32 moves of look-ahead: ~6 KB. (The planner's default 256 needed ~45 KB in one piece and aborted
    // the ESP32-S3 on the first Tina 2 power-up, 2026-10-14.) Enough to plan the stop from full speed
    // even with 0.2 mm G-code segments: 97 mm/s at 1000 mm/s^2 needs 4.7 mm, 32 x 0.2 mm = 6.4 mm.
    pcfg_.lookahead = io_.lookahead > 32 ? io_.lookahead : 32;
    // More look-ahead (boards with PSRAM): halve until it fits with room to spare, down to 32.
    while (pcfg_.lookahead > 32 && io_.largest_free &&
           io_.largest_free() < Planner::ring_bytes(pcfg_.lookahead) + 65536)
        pcfg_.lookahead /= 2;
    if (pcfg_.lookahead < 32) pcfg_.lookahead = 32;
    if (io_.largest_free) {
        size_t need = Planner::ring_bytes(pcfg_.lookahead) + 8192, have = io_.largest_free();
        if (have < need) {
            ok_ = false;
            why_not_ = "not enough memory for the motion planner (" + std::to_string(have) +
                       " bytes free in one piece, needs " + std::to_string(need) + ")";
            return;
        }
    }
    planner_.reset(new Planner(pcfg_));
    planner_->set_sinks([this](const Segment& s) { on_segment(s); });
#if PH_FIXED
    shaper_.set_sink([this](int64_t us0, int64_t us1, const int32_t* st) { emit_steps_us(us0, us1, st); });
#else
    shaper_.set_sink([this](double t0, double dur, const int32_t* st) { emit_steps(t0, dur, st); });
#endif
#if PH_PRO
    shaper_.configure(plan_.shaping);
    retract_ = plan_.retract;
    if (plan_.mesh_default.valid()) mesh_ = plan_.mesh_default;   // Klipper loads the "default" profile at start
#endif
}

std::vector<std::pair<size_t, int>> Toolhead::axis_motors(int axis) const {
    if (corexy_ && axis == 0) return {{size_t(si_[0]), 1}, {size_t(si_[1]), 1}};    // +X: A+, B+
    if (corexy_ && axis == 1) return {{size_t(si_[0]), 1}, {size_t(si_[1]), -1}};   // +Y: A+, B-
    return {{size_t(si_[axis]), 1}};
}

void Toolhead::emit_steps(double t0, double T, const int32_t steps_in[NUM_AXES]) {
    PerfScope ps(this, MotionPerf::STEPS);
    double c0 = double(io_.clock_at(t0)), c1 = double(io_.clock_at(t0 + T));
    int32_t steps[NUM_AXES] = {steps_in[0], steps_in[1], steps_in[2], steps_in[3]};
    if (corexy_) {   // X and Y steps -> motor A and B steps (same steps/mm, so exact integers)
        steps[0] = steps_in[0] + steps_in[1];
        steps[1] = steps_in[0] - steps_in[1];
    }
    for (int a = 0; a < NUM_AXES; a++) {
        int32_t st = steps[a];
        if (!st || si_[a] < 0) continue;
        uint32_t k = uint32_t(st < 0 ? -st : st);
        double iv = (c1 - c0) / k;
        sq_[size_t(si_[a])].push_run(st > 0 ? 1 : 0, c0 + 0.5 * iv, iv, k);
        perf_.steps += k;
    }
}

// The same with the board clocks already known: no floating point at all (PH_FIXED). The interval in
// 1/65536 ticks is span * 65536 / count; for spans under 65536 ticks (5.4 ms at 12 MHz, which covers
// the 2 ms grid and the 5 ms planner segments) that is a 32-bit divide, which the ESP32-S3 does in
// hardware (a 64-bit divide is a software routine).
void Toolhead::emit_steps_clk(uint64_t c0, uint64_t c1, const int32_t steps_in[NUM_AXES]) {
    PerfScope ps(this, MotionPerf::STEPS);
    int32_t steps[NUM_AXES] = {steps_in[0], steps_in[1], steps_in[2], steps_in[3]};
    if (corexy_) {
        steps[0] = steps_in[0] + steps_in[1];
        steps[1] = steps_in[0] - steps_in[1];
    }
    const uint64_t span = c1 > c0 ? c1 - c0 : 0;
    for (int a = 0; a < NUM_AXES; a++) {
        int32_t st = steps[a];
        if (!st || si_[a] < 0) continue;
        uint32_t k = uint32_t(st < 0 ? -st : st);
        int64_t iv_fx = span < 65536 ? int64_t(uint32_t(span << 16) / k) : int64_t((span << 16) / k);
        sq_[size_t(si_[a])].push_run_fx(st > 0 ? 1 : 0, int64_t(c0 << 16) + iv_fx / 2, iv_fx, k);
        perf_.steps += k;
    }
}

#if PH_FIXED
// Host time -> board clock as a straight line through clock_at(t) with the slope over the next
// second, refreshed for every planner segment (clock_at itself follows the clock sync).
void Toolhead::map_refresh(double t) {
    map_us_ = MotionShaperFx::to_us(t);
    double tt = double(map_us_) * 1e-6;
    map_clk_ = io_.clock_at(tt);
    uint64_t c2 = io_.clock_at(tt + 1.0);
    map_q32_ = int64_t(double(c2 - map_clk_) * 1e-6 * 4294967296.0);
}

uint64_t Toolhead::clock_us(int64_t us) const {
    int64_t d = us - map_us_;   // within a fraction of a second: fits easily
    return uint64_t(int64_t(map_clk_) + ((d * map_q32_ + (int64_t(1) << 31)) >> 32));
}

void Toolhead::emit_steps_us(int64_t us0, int64_t us1, const int32_t steps[NUM_AXES]) {
    emit_steps_clk(clock_us(us0), clock_us(us1), steps);
}
#endif

void Toolhead::on_segment_plan(const Segment& s) {
    // Planned late (held for look-ahead, or `now` was taken before something slow): never schedule in
    // the past. The time is read fresh here: 2026-10-27 on the Tina 2, a G1 Z3 right after a board
    // restart was timed from a stale "now" and its steps reached the board 44 ms late ("Timer too close").
    double f = io_.now ? std::max(now_, io_.now()) : now_;
    if (t_ < f + 0.1) t_ = f + 0.2;
    double T = double(s.n) / pcfg_.dda_hz;
    perf_.segments++;
    segs_since_send_++;
#if PH_FIXED
    map_refresh(t_);
#endif
#if PH_PRO
    if (shaper_.active()) {
        bool last = (s.flags & SEG_LAST) != 0;
        {
            PerfScope ps(this, MotionPerf::SHAPE);
            shaper_.push(t_, T, s.steps, last);
        }
        t_ += T;
        if (last) {   // the shaped motion ends `tail` later; the next one starts `lead` early
            t_ = std::max(t_, shaper_.out_end()) + shaper_.lead() + 1e-6;
            if (shaping_pending_) {
                shaper_.configure(shaping_next_);
                shaping_pending_ = false;
            }
        }
        t_end_ = std::max(t_end_, shaper_.out_end());
        return;
    }
#endif
#if PH_FIXED
    emit_steps_clk(io_.clock_at(t_), io_.clock_at(t_ + T), s.steps);
#else
    emit_steps(t_, T, s.steps);
#endif
    t_ += T;
    t_end_ = t_;
}

// Every planner segment (2026-10-28): plan it, then, if nothing has been sent for 20 ms, send what is
// due now. A burst of planning (look-ahead running dry, or many tiny moves, e.g. the benchmark's
// "wiggle") can otherwise take longer on the ESP32-S3 than the 0.1-0.2 s its first steps were given,
// and they reach the send already late (Tina 2, SF_BENCH, 2026-10-28: refused 0.65 and 18.8 ms late).
void Toolhead::on_segment(const Segment& s) {
    on_segment_plan(s);
    send_soon();
}

std::string Toolhead::set_shaping(const ShapingConfig& c, double now) {
#if PH_PRO
    if (!ok_) return "moves are not set up: " + why_not_;
    if (shaper_.idle() && !planner_->queued() && (!shaping_pending_ || idle(now))) {
        shaper_.configure(c);
        shaping_pending_ = false;
        return "";
    }
    if (!shaping_pending_ && shaper_.config().same_except_pa(c)) {   // only the advance value: at once
        shaper_.set_pa(c.pa);
        return "";
    }
    shaping_next_ = c;   // in motion: from the next stop
    shaping_pending_ = true;
    return "";
#else
    (void)c, (void)now;
    return "input shaping and pressure advance need the CAPTAIN or SKIPPER build";
#endif
}

void Toolhead::ensure_time(double now) {
    if (io_.now) now = std::max(now, io_.now());
    if (t_ < now + 0.1) t_ = now + START_DELAY;
}

void Toolhead::enable_all(double now) {
    if (enabled_ || bench_) return;   // benchmark: the drivers stay off
    for (int a = 0; a < NUM_AXES; a++)
        if (si_[a] >= 0 && io_.enable) io_.enable(size_t(si_[a]), true, now);
    enabled_ = true;
}

void Toolhead::flush_lookahead() {
    PerfScope ps(this, MotionPerf::PLAN);
    if (planner_ && planner_->queued()) planner_->flush();
}

double Toolhead::buffered(double now) const {
    double b = t_end_ > now ? t_end_ - now : 0;
    if (planner_ && planner_->queued()) b += 0.25;
    return b;
}

bool Toolhead::idle(double now) const {
    if (!ok_) return true;
    if (hstate_ != H_IDLE || planner_->queued() || !shaper_.idle() || rs_.active) return false;
    for (auto& q : sq_)
        if (q.pending()) return false;
    return now > t_end_ + 0.05;
}

bool Toolhead::stepper_in_use(size_t stepper, double now) const {
    if (!ok_) return false;
    for (int a = 0; a < NUM_AXES; a++)
        if (si_[a] == int(stepper)) return !idle(now);
    return false;
}

void Toolhead::stepper_used_elsewhere(size_t stepper) {
    if (stepper < sq_.size()) sq_[stepper].reset();
}

void Toolhead::stop(double now) {
    if (!ok_) return;
    double pos[NUM_AXES];
    for (int a = 0; a < NUM_AXES; a++) pos[a] = planner_->position(a);
    planner_.reset(new Planner(pcfg_));
    planner_->set_sinks([this](const Segment& s) { on_segment(s); });
    planner_->set_position(pos);   // where it was planned to be: not reliable after a stop
    for (auto& q : sq_) q.clear_pending(), q.reset();
    shaper_.reset();
    if (shaping_pending_) shaper_.configure(shaping_next_), shaping_pending_ = false;
    rs_.active = false;
    for (bool& h : homed_) h = false;   // positions are no longer known
    hstate_ = H_IDLE;
    t_ = t_end_ = now;
    inflight_ = decltype(inflight_)();
    inflight_count_ = 0;
    late_.clear();
}

void Toolhead::motors_off(double now) {
    if (!ok_) return;
    stop(now);
    for (int a = 0; a < NUM_AXES; a++)
        if (si_[a] >= 0 && io_.enable) io_.enable(size_t(si_[a]), false, now);
    enabled_ = false;
}

std::string Toolhead::check_limits(const double tgt[NUM_AXES]) const {
    for (int a = 0; a < 3; a++) {
        const StepperObj& s = plan_.steppers[size_t(si_[a])];
        if (std::fabs(tgt[a] - planner_->position(a)) < 1e-9) continue;
        if (!homed_[a]) return std::string("Must home axis first: ") + AXL[a] + " (G28, or SET_KINEMATIC_POSITION to test)";
        if (tgt[a] < s.position_min - 1e-6 || tgt[a] > s.position_max + 1e-6)
            return "Move out of range: " + f3(tgt[0]) + " " + f3(tgt[1]) + " " + f3(tgt[2]) + " (" + AXL[a] + " limits " +
                   f3(s.position_min) + ".." + f3(s.position_max) + ")";
    }
    return "";
}

void Toolhead::add_planned_move(const double tgt[NUM_AXES], double feed, double now) {
    {
        PerfScope ps(this, MotionPerf::PLAN);
        ensure_time(now);
        enable_all(now);
        planner_->add_move(tgt, feed);
        last_add_ = now;
    }
    // the look-ahead re-planning itself can take a while with many moves queued: send what is due
    send_soon();
}

// Send what is due if nothing has been sent for 20 ms (2026-10-28), from inside planning.
void Toolhead::send_soon() {
    if (!io_.now || in_send_ || !late_.empty() || hstate_ != H_IDLE || rs_.active) return;
    double f = io_.now();
    if (f - last_send_t_ > 0.02) send_due(f, false);
}

std::string Toolhead::move(const GLine& g, bool immediate, double now) {
    if (g.has('F')) {
        double f = g.get('F');
        if (f <= 0) return "error: F must be above 0";
        feed_ = f / 60.0;
    }
    double tgt[NUM_AXES], npos[NUM_AXES];
    for (int a = 0; a < 3; a++) {
        npos[a] = gpos_[a];
        if (g.has(AXL[a])) npos[a] = rel_ ? gpos_[a] + g.get(AXL[a]) : double(g.get(AXL[a]));
        tgt[a] = npos[a] + off_[a];
    }
    double de = 0;
    npos[AX_E] = gpos_[AX_E];
    if (g.has('E')) {
        de = (rel_ || e_rel_) ? double(g.get('E')) : g.get('E') - gpos_[AX_E];
        npos[AX_E] = gpos_[AX_E] + de;
    }
    if (de != 0 && si_[AX_E] < 0) return "error: no [extruder] stepper in printer.cfg";
    if (de != 0 && !bench_ && io_.extruder_temp && io_.extruder_temp() < plan_.printer.min_extrude_temp)
        return "error: Extrude below minimum temp (" + f3(plan_.printer.min_extrude_temp) + " C, min_extrude_temp)";
    tgt[AX_E] = planner_->position(AX_E) + de * extrude_factor_;
    double flat_z = tgt[2];
#if PH_PRO
    if (mesh_.valid()) tgt[2] += mesh_.at(tgt[0], tgt[1]);
#endif
    std::string lim = check_limits(tgt);
    if (!lim.empty()) return "error: " + lim;
    for (int a = 0; a < NUM_AXES; a++) gpos_[a] = npos[a];
    bool moves = false;
    for (int a = 0; a < NUM_AXES; a++) moves |= std::fabs(tgt[a] - planner_->position(a)) > 1e-9;
    if (!moves) return "ok";
#if PH_PRO
    double dxy = std::hypot(tgt[0] - planner_->position(0), tgt[1] - planner_->position(1));
    if (mesh_.valid() && dxy > 5.0) {   // follow the mesh: 5 mm pieces
        double from[NUM_AXES];
        for (int a = 0; a < NUM_AXES; a++) from[a] = planner_->position(a);
        double from_flat_z = from[2] - mesh_.at(from[0], from[1]);
        int n = int(std::ceil(dxy / 5.0));
        for (int k = 1; k <= n; k++) {
            double f = double(k) / n, p[NUM_AXES];
            for (int a = 0; a < NUM_AXES; a++) p[a] = from[a] + (tgt[a] - from[a]) * f;
            p[2] = from_flat_z + (flat_z - from_flat_z) * f + mesh_.at(p[0], p[1]);
            if (k == n) p[2] = tgt[2];
            add_planned_move(p, feed_ * speed_factor_, now);
        }
    } else {
        add_planned_move(tgt, feed_ * speed_factor_, now);
    }
#else
    (void)flat_z;
    add_planned_move(tgt, feed_ * speed_factor_, now);
#endif
    if (immediate) flush_lookahead();
    return "ok";
}

#if PH_PRO
std::string Toolhead::move_to(const double xyz[3], double speed, double now) {
    if (!ok_) return "error: moves are not set up: " + why_not_;
    if (hstate_ != H_IDLE) return "error: busy homing / probing";
    double tgt[NUM_AXES];
    for (int a = 0; a < NUM_AXES; a++) tgt[a] = planner_->position(a);
    double flat_z = tgt[2] - mesh_z(tgt[0], tgt[1]);
    for (int a = 0; a < 3; a++)
        if (!std::isnan(xyz[a])) tgt[a] = xyz[a];
    if (!std::isnan(xyz[2])) flat_z = xyz[2];
    tgt[2] = flat_z + mesh_z(tgt[0], tgt[1]);
    std::string lim = check_limits(tgt);
    if (!lim.empty()) return "error: " + lim;
    for (int a = 0; a < 3; a++) gpos_[a] = (a == 2 ? flat_z : tgt[a]) - off_[a];
    bool moves = false;
    for (int a = 0; a < 3; a++) moves |= std::fabs(tgt[a] - planner_->position(a)) > 1e-9;
    if (!moves) return "ok";
    add_planned_move(tgt, speed, now);
    flush_lookahead();
    return "ok";
}

void Toolhead::skip_move(const GLine& g) {
    for (int a = 0; a < 3; a++)
        if (g.has(AXL[a])) gpos_[a] = rel_ ? gpos_[a] + g.get(AXL[a]) : double(g.get(AXL[a]));
    if (g.has('E')) gpos_[AX_E] = (rel_ || e_rel_) ? gpos_[AX_E] + g.get('E') : double(g.get('E'));
    if (g.has('F') && g.get('F') > 0) feed_ = g.get('F') / 60.0;
}

// G2 / G3 (Klipper gcode_arcs.py planArc, from Marlin): the XY plane, I / J centre offsets, the arc
// in pieces of [gcode_arcs] resolution (1 mm), helical Z and E spread evenly.
std::string Toolhead::arc(const GLine& g, bool clockwise, bool immediate, double now) {
    if (rel_) return "error: G2/G3 does not support relative move mode";
    if (g.has('R')) return "error: G2/G3 does not support R moves";
    double I = g.get('I', 0), J = g.get('J', 0);
    if (I == 0 && J == 0) return "error: G2/G3 requires IJ parameters";
    const double cur[4] = {gpos_[0], gpos_[1], gpos_[2], gpos_[3]};
    double tgt[3] = {g.has('X') ? double(g.get('X')) : cur[0], g.has('Y') ? double(g.get('Y')) : cur[1],
                     g.has('Z') ? double(g.get('Z')) : cur[2]};
    double rP = -I, rQ = -J;
    double cP = cur[0] - rP, cQ = cur[1] - rQ;
    double rtA = tgt[0] - cP, rtB = tgt[1] - cQ;
    double ang = std::atan2(rP * rtB - rQ * rtA, rP * rtA + rQ * rtB);
    if (ang < 0) ang += 2 * M_PI;
    if (clockwise) ang -= 2 * M_PI;
    if (ang == 0 && cur[0] == tgt[0] && cur[1] == tgt[1]) ang = 2 * M_PI;
    double lin = tgt[2] - cur[2], radius = std::hypot(rP, rQ), flat = radius * ang;
    double travel = lin != 0 ? std::hypot(flat, lin) : std::fabs(flat);
    double res = plan_.arc_resolution > 0 ? plan_.arc_resolution : 1.0;
    double segs = std::max(1.0, std::floor(travel / res));
    if (segs > 20000) return "error: G2/G3 arc too long";
    double dth = ang / segs, dz = lin / segs, e_per = 0, e_base = 0;
    bool has_e = g.has('E');
    if (has_e) {
        if (!e_rel_) e_base = cur[3];
        e_per = (g.get('E') - e_base) / segs;
    }
    int n = int(segs);
    for (int i = 1; i <= n; i++) {
        double c = i * dth, ct = std::cos(c), st = std::sin(c);
        double p[3] = {cP + (-I * ct + J * st), cQ + (-I * st - J * ct), cur[2] + i * dz};
        if (i == n) p[0] = tgt[0], p[1] = tgt[1], p[2] = tgt[2];
        char b[160];
        int k = snprintf(b, sizeof b, "G1 X%.6f Y%.6f Z%.6f", p[0], p[1], p[2]);
        if (has_e && e_per != 0) {
            k += snprintf(b + k, sizeof b - size_t(k), " E%.6f", e_base + e_per);
            if (!e_rel_) e_base += e_per;
        }
        if (g.has('F')) snprintf(b + k, sizeof b - size_t(k), " F%.3f", double(g.get('F')));
        GLine s;
        gcode_parse(b, s);
        std::string r = move(s, false, now);
        if (r.rfind("ok", 0) != 0) return r;
    }
    if (immediate) flush_lookahead();
    return "ok";
}

#else
std::string Toolhead::move_to(const double*, double, double) { return "error: CAPTAIN+"; }
void Toolhead::skip_move(const GLine&) {}
#endif

std::string Toolhead::jog(int axis, double dist, double speed, double now) {
    if (!ok_) return "error: moves are not set up: " + why_not_;
    if (hstate_ != H_IDLE) return "error: busy homing / probing";
    if (axis < 0 || axis >= NUM_AXES || speed <= 0) return "error: bad jog";
    now_ = now;
    double tgt[NUM_AXES];
    for (int a = 0; a < NUM_AXES; a++) tgt[a] = planner_->position(a);
    if (axis == AX_E) {
        if (si_[AX_E] < 0) return "error: no [extruder] stepper in printer.cfg";
        if (!bench_ && io_.extruder_temp && io_.extruder_temp() < plan_.printer.min_extrude_temp)
            return "error: Extrude below minimum temp (" + f3(plan_.printer.min_extrude_temp) + " C, min_extrude_temp)";
    }
    tgt[axis] += dist;
    if (axis < 2 && mesh_.valid()) tgt[2] += mesh_z(tgt[0], tgt[1]) - mesh_z(planner_->position(0), planner_->position(1));
    std::string lim = check_limits(tgt);
    if (!lim.empty()) return "error: " + lim;
    gpos_[axis] += dist;
    add_planned_move(tgt, speed, now);
    flush_lookahead();
    return "ok";
}

int64_t Toolhead::mcu_position(size_t i, bool* okp) {
    Message m;
    bool ok = io_.query("stepper_get_position oid=" + std::to_string(plan_.steppers[i].oid), "stepper_position",
                        plan_.steppers[i].oid, m, 200);
    if (okp) *okp = ok;
    return ok ? int64_t(int32_t(uint32_t(m.get("pos")))) : 0;
}

// ---- G-code

std::string Toolhead::gcode(const std::string& name, const GLine* g, const std::map<std::string, std::string>& p,
                            bool immediate, double now) {
    static const char* MOTION[] = {"G0", "G1", "G4", "G28", "G90", "G91", "G92", "M82", "M83", "M114", "M204", "M220",
                                   "M221", "M400", "PROBE", "SET_KINEMATIC_POSITION", "QUERY_ENDSTOPS", "GET_POSITION",
                                   "G2", "G3", "G10", "G11", "G17", "G18", "G19", "SET_RETRACTION", "GET_RETRACTION"};
    bool is_motion = false;
    for (auto* m : MOTION) is_motion |= name == m;
    if (!is_motion) return "";
    if (!ok_) return "error: moves are not set up: " + why_not_;
    now_ = now;
    if (name == "G90") return rel_ = false, "ok";
    if (name == "G91") return rel_ = true, "ok";
    if (name == "M82") return e_rel_ = false, "ok";
    if (name == "M83") return e_rel_ = true, "ok";
    if (name == "M114" || name == "GET_POSITION")
        return "ok X:" + f3(gpos_[0]) + " Y:" + f3(gpos_[1]) + " Z:" + f3(gpos_[2]) + " E:" + f3(gpos_[3]);
    if (name == "M400") return "ok";
    if (hstate_ != H_IDLE && name != "QUERY_ENDSTOPS") return "error: busy homing / probing";
    if ((name == "G0" || name == "G1") && g) return move(*g, immediate, now);
#if PH_PRO
    if ((name == "G2" || name == "G3") && g) return arc(*g, name == "G2", immediate, now);
    if (name == "G17") return "ok";
    if (name == "G18" || name == "G19") return "error: only the XY plane (G17) is supported for arcs";
    if (name == "SET_RETRACTION") {
        auto get = [&](const char* k, double& v) {
            auto it = p.find(k);
            if (it != p.end()) v = std::max(0.0, atof(it->second.c_str()));
        };
        get("RETRACT_LENGTH", retract_.length);
        get("RETRACT_SPEED", retract_.speed);
        get("UNRETRACT_EXTRA_LENGTH", retract_.unretract_extra);
        get("UNRETRACT_SPEED", retract_.unretract_speed);
        if (retract_.speed <= 0) retract_.speed = 20;
        if (retract_.unretract_speed <= 0) retract_.unretract_speed = 10;
    }
    if (name == "SET_RETRACTION" || name == "GET_RETRACTION")
        return "ok RETRACT_LENGTH=" + f3(retract_.length) + " RETRACT_SPEED=" + f3(retract_.speed) +
               " UNRETRACT_EXTRA_LENGTH=" + f3(retract_.unretract_extra) + " UNRETRACT_SPEED=" + f3(retract_.unretract_speed);
    if (name == "G10" || name == "G11") {   // firmware_retraction.py: E only, the G-code E position unchanged
        bool g10 = name == "G10";
        if (retract_.length <= 0 || retracted_ == g10) return "ok";
        if (si_[AX_E] < 0) return "error: no [extruder] stepper in printer.cfg";
        if (!bench_ && io_.extruder_temp && io_.extruder_temp() < plan_.printer.min_extrude_temp)
            return "error: Extrude below minimum temp (" + f3(plan_.printer.min_extrude_temp) + " C, min_extrude_temp)";
        double t[NUM_AXES];
        for (int a = 0; a < NUM_AXES; a++) t[a] = planner_->position(a);
        t[AX_E] += g10 ? -retract_.length : retract_.length + retract_.unretract_extra;
        add_planned_move(t, g10 ? retract_.speed : retract_.unretract_speed, now);
        retracted_ = g10;
        if (immediate) flush_lookahead();
        return "ok";
    }
#else
    if (name == "G2" || name == "G3" || name == "G10" || name == "G11" || name == "SET_RETRACTION" || name == "GET_RETRACTION")
        return "error: " + name + " needs the CAPTAIN or SKIPPER build";
    if (name == "G17" || name == "G18" || name == "G19") return "ok";
#endif
    if (name == "G4" && g) {
        double sec = (g->has('P') ? g->get('P') / 1000.0 : 0.0) + (g->has('S') ? double(g->get('S')) : 0.0);
        if (sec < 0 || sec > 3600) return "error: G4 dwell must be 0 to 3600 s";
        ensure_time(now);
        planner_->dwell(uint64_t(sec * pcfg_.dda_hz));
        return "ok";
    }
    if (name == "G92" && g) {
        for (int a = 0; a < 3; a++)
            if (g->has(AXL[a])) {
                gpos_[a] = g->get(AXL[a]);
                off_[a] = planner_->position(a) - gpos_[a];
                if (a == 2) off_[a] -= mesh_z(planner_->position(0), planner_->position(1));
            }
        if (g->has('E')) gpos_[AX_E] = g->get('E');
        return "ok";
    }
    if (name == "M204" && g) {
        double a = g->has('S') ? g->get('S') : g->get('P', 0);
        if (a <= 0) return "error: M204 needs S";
        planner_->set_accel(a);
        return "ok";
    }
    if (name == "M220" && g) {
        if (!(g->get('S', 0) > 0)) return "error: M220 needs S above 0";
        speed_factor_ = g->get('S') / 100.0;
        return "ok";
    }
    if (name == "M221" && g) {
        if (!(g->get('S', 0) > 0)) return "error: M221 needs S above 0";
        extrude_factor_ = g->get('S') / 100.0;
        return "ok";
    }
    if (name == "SET_KINEMATIC_POSITION") {
        flush_lookahead();
        double pos[NUM_AXES];
        for (int a = 0; a < NUM_AXES; a++) pos[a] = planner_->position(a);
        std::string done;
        for (int a = 0; a < 3; a++) {
            auto it = p.find(std::string(1, AXL[a]));
            if (it == p.end()) continue;
            pos[a] = atof(it->second.c_str());
            homed_[a] = true;
            off_[a] = 0;
            gpos_[a] = pos[a];
            done += AXL[a];
        }
        if (done.empty()) {   // like Klipper: no axis given = all axes at the current position
            for (int a = 0; a < 3; a++) homed_[a] = true, gpos_[a] = pos[a] - off_[a];
            done = "XYZ";
        }
        planner_->set_position(pos);
        return "ok " + done + " marked as homed (no homing move was made)";
    }
    if (name == "QUERY_ENDSTOPS") {
        std::string r = "ok";
        for (int a = 0; a < 3; a++) {
            const StepperObj& s = plan_.steppers[size_t(si_[a])];
            if (s.endstop < 0) continue;
            const EndstopObj& e = plan_.endstops[size_t(s.endstop)];
            Message m;
            if (!io_.query("endstop_query_state oid=" + std::to_string(e.oid), "endstop_state", e.oid, m, 200)) {
                r += std::string(" ") + char(tolower(AXL[a])) + ":no answer";
                continue;
            }
            bool trig = (m.get("pin_value") != 0) != e.invert;
            r += std::string(" ") + char(tolower(AXL[a])) + ":" + (trig ? "TRIGGERED" : "open") + (e.tmc >= 0 ? "(stallguard)" : "");
        }
        return r;
    }
    if (name == "G28") {
        std::vector<int> axes;
        for (int a = 0; a < 3; a++)
            if (g && g->has(AXL[a])) axes.push_back(a);
        if (axes.empty()) axes = {0, 1, 2};
        return start_home(axes, false, now);
    }
    if (name == "PROBE") {
        if (!plan_.probe.present) return "error: no [probe] in printer.cfg";
        return start_home({2}, true, now);
    }
    return "error: " + name + " is not supported";
}

// ---- homing and probing (homing.py, probe.py; sensorless: tmc.py)

std::string Toolhead::start_home(const std::vector<int>& axes, bool probe, double now) {
    for (int a : axes) {
        const StepperObj& s = plan_.steppers[size_t(si_[a])];
        int e = probe ? plan_.probe.endstop : s.endstop;
        if (e < 0) return std::string("error: no endstop for ") + AXL[a] + " (endstop_pin in [" + s.name + "])";
    }
    if (probe && !homed_[2]) return "error: home Z before probing (G28 Z)";
    hj_ = HomeJob();
    last_error_.clear();   // a new homing: an old failure must not count against it
    for (int a : axes) last_moved_[a] = NAN;
    hj_.axes = axes;
    hj_.probe = probe;
    hj_.started = now;
    hstate_ = H_WAIT_IDLE;
    flush_lookahead();
    have_probe_result_ = false;
    std::string list;
    for (int a : axes) list += AXL[a];
    home_msg_ = (probe ? "probing" : "homing ") + (probe ? std::string() : list);
    return "ok " + home_msg_ + " (watch the status)";
}

void Toolhead::tmc_sensorless(int e, bool on) {
    if (e < 0 || !io_.tmc_write) return;
    const EndstopObj& es = plan_.endstops[size_t(e)];
    if (es.tmc < 0) return;
    const TmcObj& t = plan_.tmcs[size_t(es.tmc)];
    if (on) {   // stallGuard4 needs stealthChop: TPWMTHRS 0, en_spreadcycle 0, TCOOLTHRS max
        io_.tmc_write(es.tmc, 0x13, 0);
        io_.tmc_write(es.tmc, 0x00, t.gconf & ~(1u << 2));
        if (t.tcoolthrs == 0) io_.tmc_write(es.tmc, 0x14, 0xfffff);
    } else {
        io_.tmc_write(es.tmc, 0x13, t.tpwmthrs);
        io_.tmc_write(es.tmc, 0x00, t.gconf);
        io_.tmc_write(es.tmc, 0x14, t.tcoolthrs);
    }
}

void Toolhead::schedule_homing_move(size_t i, int dir, double dist, double speed, double accel, double t0) {
    const StepperObj& s = plan_.steppers[i];
    double spm = s.steps_per_mm();
    uint64_t n = uint64_t(std::llround(dist * spm));
    double da = speed * speed / (2 * accel);    // distance to reach speed
    uint64_t n_acc = std::min<uint64_t>(n, uint64_t(da * spm));
    double c0 = double(io_.clock_at(t0)), f = double(io_.clock_at(t0 + 1.0)) - c0;
    for (uint64_t k = 1; k <= n_acc; k++) {   // speeding up: one step per run
        double x = (double(k) - 0.5) / spm;
        sq_[i].push_run(dir, c0 + std::sqrt(2 * x / accel) * f, 1, 1);
    }
    if (n > n_acc) {   // cruise: evenly spaced
        double x = (double(n_acc) + 0.5) / spm;
        double t = speed / accel + (x - da) / speed;
        if (n_acc == 0) t = x / speed;
        sq_[i].push_run(dir, c0 + t * f, f / (speed * spm), uint32_t(std::min<uint64_t>(n - n_acc, 0xffffffffu)));
    }
}

bool Toolhead::home_begin_axis(double now) {
    int a = hj_.axes[hj_.idx];
    size_t i = size_t(si_[a]);
    const StepperObj& s = plan_.steppers[i];
    int e = hj_.probe ? plan_.probe.endstop : s.endstop;
    const EndstopObj& es = plan_.endstops[size_t(e)];
    double speed, dist;
    int dir;
    if (hj_.probe) {
        speed = plan_.probe.speed;
        dir = 0;
        dist = planner_->position(2) - s.position_min;
        if (dist <= 0.01) {
            home_fail("Z is already at position_min: nothing to probe", now);
            return false;
        }
    } else {
        speed = hj_.pass == 0 ? s.homing_speed : s.second_homing_speed;
        dir = s.homing_positive_dir ? 1 : 0;
        dist = (s.position_max - s.position_min) * 1.5;
        if (hj_.pass == 1) dist = s.homing_retract_dist * 2;
    }
    double accel = a == 2 ? plan_.printer.max_z_accel : plan_.printer.max_accel;
    if (accel <= 0) accel = 500;
    tmc_sensorless(e, true);
    enable_all(now);
    bool ok = true;
    hj_.pos_before = mcu_position(i, &ok);
    if (ok && corexy_ && !hj_.probe && a < 2) hj_.pos_before_b = mcu_position(size_t(si_[1 - a]), &ok);
    if (!ok) {
        home_fail("the board did not report the stepper position", now);
        return false;
    }
    if (io_.now) now = std::max(now, io_.now());   // the TMC set-up and the position query took time
    double t0 = now + 0.3;
    double ta = speed / accel, da = speed * speed / (2 * accel);
    double move_t = dist <= da ? std::sqrt(2 * dist / accel) : ta + (dist - da) / speed;
    double f = double(io_.clock_at(now + 1.0)) - double(io_.clock_at(now));
    // Klipper (homing.py _calc_endstop_rate): check the endstop once per step time, not faster
    // (was a fixed 100 us: up to six times the board load Klipper itself puts on it while homing)
    double rest = 1.0 / std::max(1.0, speed * s.steps_per_mm());
    // CoreXY: X and Y homing moves turn both motors; both stop on the axis' endstop (Klipper adds
    // every stepper that moves the axis to its endstop).
    auto motors = hj_.probe ? std::vector<std::pair<size_t, int>>{{i, 1}} : axis_motors(a);
    std::string T = std::to_string(s.trsync_oid), E = std::to_string(es.oid);
    std::vector<std::string> cmds = {
        "trsync_start oid=" + T + " report_clock=" + std::to_string(uint32_t(io_.clock_at(t0))) +
        " report_ticks=" + std::to_string(uint32_t(0.05 * f)) + " expire_reason=3"};
    for (auto& mo : motors) cmds.push_back("stepper_stop_on_trigger oid=" + std::to_string(plan_.steppers[mo.first].oid) + " trsync_oid=" + T);
    cmds.push_back("trsync_set_timeout oid=" + T + " clock=" + std::to_string(uint32_t(io_.clock_at(t0 + move_t + 0.2))));
    cmds.push_back("endstop_home oid=" + E + " clock=" + std::to_string(uint32_t(io_.clock_at(t0))) +
                   " sample_ticks=" + std::to_string(uint32_t(0.000015 * f)) + " sample_count=4 rest_ticks=" +
                   std::to_string(uint32_t(rest * f)) + " pin_value=" + (es.invert ? "0" : "1") + " trsync_oid=" + T +
                   " trigger_reason=1");
    for (auto& mo : motors) {
        sq_[mo.first].reset();
        schedule_homing_move(mo.first, mo.second > 0 ? dir : 1 - dir, dist, speed, accel, t0);
    }
    // Only the first SEND_AHEAD seconds of steps now; tick() streams the rest like any move.
    // (2026-10-20 Tina 2: compressing all 49,200 steps of the Z homing move up front took about
    // 0.4 s on the ESP32-S3, so endstop_home reached the board 130 ms after its own start time:
    // "Timer too close".)
    int first_moves = 0;
    for (auto& mo : motors) first_moves += sq_[mo.first].flush(io_.clock_at(t0 + SEND_AHEAD), cmds);
    if (io_.now && io_.now() > t0 - 0.05) {   // still too slow: do not send anything late, start again
        for (auto& mo : motors) sq_[mo.first].clear_pending(), sq_[mo.first].reset();
        tmc_sensorless(e, false);
        if (++hj_.late_tries >= 3) {
            home_fail("the ESP32 could not prepare the homing move in time (3 tries)", now);
            return false;
        }
        if (io_.log) io_.log("homing: move prepared too late, trying again");
        hstate_ = H_WAIT_IDLE;
        return false;
    }
    std::string err;
    if (!io_.send(cmds, &err)) {
        tmc_sensorless(e, false);
        home_fail("could not send the homing move: " + err, now);
        return false;
    }
    if (first_moves) {
        uint64_t lc = 0;
        for (auto& mo : motors) lc = std::max(lc, sq_[mo.first].last_clock());
        inflight_.push({lc, first_moves});
        inflight_count_ += first_moves;
    }
    hj_.t_end = t0 + move_t;
    hj_.triggered = hj_.finished = false;
    hj_.reason = 0;
    t_end_ = std::max(t_end_, hj_.t_end);
    home_msg_ = std::string(hj_.probe ? "probing Z" : "homing ") + (hj_.probe ? "" : std::string(1, AXL[a])) +
                (hj_.pass ? " (second, slower approach)" : "");
    return true;
}

#if PH_PRO
// ---- the resonance test motion

std::string Toolhead::start_resonance(int axis, const ResonanceMotion& m, double now) {
    if (!ok_) return "moves are not set up: " + why_not_;
    if (axis < 0 || axis > 1) return "the resonance test runs along X or Y";
    if (!homed_[axis]) return std::string("home ") + AXL[axis] + " first";
    if (rs_.active || hstate_ != H_IDLE || !idle(now)) return "wait until nothing moves";
    const StepperObj& s = plan_.steppers[size_t(si_[axis])];
    double pos = planner_->position(axis);
    if (pos + m.min_x < s.position_min - 1e-6 || pos + m.max_x > s.position_max + 1e-6)
        return "the test swings from " + f3(m.min_x) + " to +" + f3(m.max_x) + " mm around the head; at " + f3(pos) +
               " that leaves " + AXL[axis] + " limits " + f3(s.position_min) + ".." + f3(s.position_max) +
               ". Move the head towards the middle.";
    rs_ = ResRun();
    rs_.active = true;
    rs_.axis = axis;
    rs_.m = m;
    rs_.spm = s.steps_per_mm();
    if (io_.now) now = std::max(now, io_.now());
    rs_.t0 = now + START_DELAY;
    enable_all(now);
    t_end_ = std::max(t_end_, rs_.t0 + m.end_t);
    return "";
}

// Exact step times: a step up when the position passes the next half step, down likewise; each
// piece is a parabola, split where the speed passes zero.
void Toolhead::resonance_generate(double upto) {
    if (upto > rs_.m.end_t) upto = rs_.m.end_t;
    auto mot = axis_motors(rs_.axis);   // CoreXY: both motors, one step each per axis step
    const double f = double(io_.clock_at(rs_.t0 + 1.0)) - double(io_.clock_at(rs_.t0)), c0 = double(io_.clock_at(rs_.t0));
    const auto& P = rs_.m.pieces;
    while (rs_.gen < upto && rs_.piece < P.size()) {
        const ResonanceMotion::Piece& p = P[rs_.piece];
        double pend = rs_.piece + 1 < P.size() ? P[rs_.piece + 1].t0 : rs_.m.end_t;
        double u0 = rs_.gen - p.t0, u1 = std::min(upto, pend) - p.t0;
        // split at the turning point
        double tz = p.a != 0 ? -p.v0 / p.a : -1;
        double cuts[3] = {u0, (tz > u0 && tz < u1) ? tz : u1, u1};
        for (int part = 0; part < 2; part++) {
            double a0 = cuts[part], a1 = cuts[part + 1];
            if (a1 <= a0) continue;
            double vm = p.v0 + p.a * 0.5 * (a0 + a1);
            if (vm == 0) continue;
            int dir = vm > 0 ? 1 : -1;
            double xe = (p.x0 + p.v0 * a1 + 0.5 * p.a * a1 * a1) * rs_.spm;   // in steps
            for (int guard = 0; guard < 1000000; guard++) {
                double target = double(rs_.s) + 0.5 * dir;
                if ((dir > 0 && xe < target) || (dir < 0 && xe > target)) break;
                // 0.5 a u^2 + v0 u + (x0 - target / spm) = 0, the root in [a0, a1]
                double A = 0.5 * p.a * rs_.spm, B = p.v0 * rs_.spm, C = p.x0 * rs_.spm - target, u;
                if (std::fabs(A) < 1e-12) {
                    u = -C / B;
                } else {
                    double disc = std::max(0.0, B * B - 4 * A * C), sq = std::sqrt(disc);
                    double qq = -0.5 * (B + (B >= 0 ? sq : -sq));
                    double r1 = qq / A, r2 = qq != 0 ? C / qq : r1;
                    if (r1 > r2) std::swap(r1, r2);
                    u = (r1 >= a0 - 1e-9 && r1 <= a1 + 1e-9) ? r1 : r2;
                }
                u = std::min(std::max(u, a0), a1);
                for (auto& mo : mot) sq_[mo.first].push_run(dir * mo.second > 0 ? 1 : 0, c0 + (p.t0 + u) * f, 1, 1);
                rs_.s += dir;
            }
        }
        rs_.gen = std::min(upto, pend);
        if (rs_.gen >= pend) rs_.piece++;
    }
}

void Toolhead::resonance_finish(double now) {
    rs_.active = false;
    double pos[NUM_AXES];
    for (int k = 0; k < NUM_AXES; k++) pos[k] = planner_->position(k);
    pos[rs_.axis] += double(rs_.s) / rs_.spm;   // where the steps left it (exact)
    planner_->set_position(pos);
    gpos_[rs_.axis] = pos[rs_.axis] - off_[rs_.axis];
    t_end_ = std::max(t_end_, now);
}

void Toolhead::abort_resonance(double now) {
    if (!rs_.active) return;
    for (auto& mo : axis_motors(rs_.axis)) sq_[mo.first].clear_pending(), sq_[mo.first].reset();
    rs_.active = false;
    homed_[rs_.axis] = false;   // the board may still run what was sent: the position is not known
    t_end_ = now + SEND_AHEAD;
}

#else
std::string Toolhead::start_resonance(int, const ResonanceMotion&, double) { return "needs the CAPTAIN or SKIPPER build"; }
void Toolhead::resonance_generate(double) {}
void Toolhead::resonance_finish(double) {}
void Toolhead::abort_resonance(double) {}
#endif

void Toolhead::abort_homing(double now) {
    if (hstate_ == H_IDLE) return;
    int a = hj_.axes.empty() ? -1 : hj_.axes[hj_.idx < hj_.axes.size() ? hj_.idx : 0];
    if (hstate_ == H_MOVING && a >= 0) {   // the move is out: the board stops it, home_finish_axis follows
        const StepperObj& s = plan_.steppers[size_t(si_[a])];
        io_.send({"trsync_trigger oid=" + std::to_string(s.trsync_oid) + " reason=2"}, nullptr);
        hj_.reason = 2;
        hj_.finished = true;
        return;
    }
    // not sent yet (waiting for idle, or between late-guard retries): send nothing at all
    if (a >= 0) homed_[a] = false;
    home_fail("aborted", now);
}

void Toolhead::home_fail(const std::string& why, double now) {
    (void)now;
    last_error_ = why;
    home_msg_ = "FAILED: " + why;
    if (io_.log) io_.log((hj_.probe ? "probe: " : "homing: ") + why);
    hstate_ = H_IDLE;
}

void Toolhead::home_finish_axis(double now) {
    int a = hj_.axes[hj_.idx];
    size_t i = size_t(si_[a]);
    const StepperObj& s = plan_.steppers[i];
    int e = hj_.probe ? plan_.probe.endstop : s.endstop;
    const EndstopObj& es = plan_.endstops[size_t(e)];
    io_.send({"endstop_home oid=" + std::to_string(es.oid) +
                  " clock=0 sample_ticks=0 sample_count=0 rest_ticks=0 pin_value=0 trsync_oid=0 trigger_reason=0",
              "trsync_trigger oid=" + std::to_string(s.trsync_oid) + " reason=2"},
             nullptr);
    bool cxy = corexy_ && !hj_.probe && a < 2;
    for (auto& mo : (hj_.probe ? std::vector<std::pair<size_t, int>>{{i, 1}} : axis_motors(a)))
        sq_[mo.first].clear_pending(), sq_[mo.first].reset();
    tmc_sensorless(e, false);
    bool ok = true;
    int64_t after = mcu_position(i, &ok);
    double moved = double(after - hj_.pos_before) / s.steps_per_mm() * (s.invert_dir ? -1.0 : 1.0);
    if (cxy && ok) {   // the axis moved by half the sum (X) or half the difference (Y) of the two motors
        const StepperObj& o = plan_.steppers[size_t(si_[1 - a])];
        int64_t after_o = mcu_position(size_t(si_[1 - a]), &ok);
        double mo = double(after_o - hj_.pos_before_b) / o.steps_per_mm() * (o.invert_dir ? -1.0 : 1.0);
        double mA = a == 0 ? moved : mo, mB = a == 0 ? mo : moved;
        moved = a == 0 ? 0.5 * (mA + mB) : 0.5 * (mA - mB);
    }
    if (ok) last_moved_[a] = std::fabs(moved);
    t_end_ = now;
    if (hj_.reason != 1) {
        home_fail(hj_.probe ? "the probe did not trigger before position_min"
                            : std::string("No trigger on ") + char(tolower(AXL[a])) +
                                  " after full movement (sensorless: raise driver_SGTHRS / check the DIAG jumper)",
                  now);
        double pos[NUM_AXES];
        for (int k = 0; k < NUM_AXES; k++) pos[k] = planner_->position(k);
        if (ok) pos[a] += moved;
        homed_[a] = false;
        planner_->set_position(pos);
        return;
    }
    double pos[NUM_AXES];
    for (int k = 0; k < NUM_AXES; k++) pos[k] = planner_->position(k);
    if (hj_.probe) {
        pos[2] += moved;   // where it stopped
        planner_->set_position(pos);
        gpos_[2] = pos[2] - off_[2];
        probe_result_ = pos[2];
        have_probe_result_ = true;
        home_msg_ = "probe at " + f3(pos[0] + plan_.probe.x_offset) + "," + f3(pos[1] + plan_.probe.y_offset) +
                    " is z=" + f3(pos[2]) + " (z_offset " + f3(plan_.probe.z_offset) + ")";
        if (io_.log) io_.log(home_msg_);
        hstate_ = H_IDLE;
        // lift 2 mm (Klipper sample_retract_dist)
        double up[NUM_AXES];
        for (int k = 0; k < NUM_AXES; k++) up[k] = pos[k];
        up[2] = std::min(pos[2] + 2.0, s.position_max);
        add_planned_move(up, plan_.probe.lift_speed, now);
        flush_lookahead();
        gpos_[2] = up[2] - off_[2];
        return;
    }
    pos[a] = s.position_endstop;
    planner_->set_position(pos);
    off_[a] = 0;
    gpos_[a] = pos[a];
    if (hj_.pass == 0 && s.homing_retract_dist > 0) {   // back off, then a second slower approach
        double away[NUM_AXES];
        for (int k = 0; k < NUM_AXES; k++) away[k] = pos[k];
        away[a] += s.homing_positive_dir ? -s.homing_retract_dist : s.homing_retract_dist;
        homed_[a] = true;
        add_planned_move(away, s.homing_speed, now);
        flush_lookahead();
        hj_.pass = 1;
        hstate_ = H_WAIT_IDLE;
        return;
    }
    homed_[a] = true;
    hj_.pass = 0;
    hj_.idx++;
    if (hj_.idx >= hj_.axes.size()) {
        std::string l;
        for (int x : hj_.axes) l += AXL[x];
        home_msg_ = "homed " + l;
        if (io_.log) io_.log(home_msg_);
        hstate_ = H_IDLE;
        return;
    }
    hstate_ = H_WAIT_IDLE;
}

void Toolhead::home_step(double now) {
    switch (hstate_) {
    case H_IDLE:
        return;
    case H_WAIT_IDLE: {
        if (planner_->queued()) flush_lookahead();
        bool steps = false;
        for (auto& q : sq_) steps |= q.pending() > 0;
        if (steps || now < t_end_ + 0.15) return;
        hstate_ = H_START;
    }
        [[fallthrough]];
    case H_START:
        if (home_begin_axis(now)) hstate_ = H_MOVING;
        return;
    case H_MOVING:
        if (hj_.finished || now > hj_.t_end + 1.0) {
            if (!hj_.finished) hj_.reason = 3;
            home_finish_axis(now);
        }
        return;
    default:
        return;
    }
}

void Toolhead::on_message(const Message& m, double now) {
    (void)now;
    if (!ok_ || hstate_ != H_MOVING || m.name != "trsync_state" || hj_.finished) return;   // the first verdict counts

    int a = hj_.axes[hj_.idx];
    const StepperObj& s = plan_.steppers[size_t(si_[a])];
    if (m.get("oid") != s.trsync_oid || m.get("can_trigger")) return;
    hj_.finished = true;
    hj_.reason = int(m.get("trigger_reason"));
}

// ---- periodic: plan, compress, send

void Toolhead::tick(double now) {
    if (!ok_) return;
    if (io_.now) now = std::max(now, io_.now());   // the caller's time may be old (slow work before)
    now_ = now;
    home_step(now);
    // nothing new for a moment, or about to run dry: plan out what is queued
    if (planner_->queued() && (now - last_add_ > 0.3 || t_ - now < 0.3)) flush_lookahead();
    if (!late_.empty()) {   // a send in the middle of that found a step too late: stop now
        std::string why = late_;
        late_.clear();
        double fresh = io_.now ? std::max(now, io_.now()) : now;
        if (io_.log) io_.log("moves: " + why);
        if (io_.report) io_.report("moves: " + why);
        last_error_ = why;
        stop(fresh);
        home_msg_ = "FAILED: " + why;
        return;
    }
    if (rs_.active) {   // only as far ahead as gets sent now: the step queue stays small
        resonance_generate(now + SEND_AHEAD - rs_.t0);
        if (rs_.gen >= rs_.m.end_t && now > rs_.t0 + rs_.m.end_t + 0.05) resonance_finish(now);
    }
    send_due(now, true);
}

bool Toolhead::send_due(double now, bool may_stop) {
    if (in_send_) return true;
    double fresh = io_.now ? std::max(now, io_.now()) : now;
    // make room in the board's move queue
    uint64_t nowc = io_.clock_at(fresh);
    while (!inflight_.empty() && inflight_.top().first <= nowc) {
        inflight_count_ -= inflight_.top().second;
        inflight_.pop();
    }
    int limit = plan_.move_count > 100 ? plan_.move_count - 60 : 200;
    if (inflight_count_ >= limit) {
        perf_.queue_full++;
        return true;
    }
    uint64_t upto = io_.clock_at(fresh + SEND_AHEAD);
    // Last check before sending: a step whose time has (nearly) gone by would shut the board down with
    // "Timer too close" (2026-10-27, Tina 2). Refuse the move instead: nothing is sent, everything
    // stops, and the axes count as not homed (the position is no longer known).
    {
        uint64_t guard = io_.clock_at(fresh + 0.005) - nowc;
        for (auto& q : sq_) {
            uint64_t nc = q.pending() ? q.next_clock() : 0;
            if (!nc || int64_t(nc - nowc) >= int64_t(guard)) continue;
            double ms = double(int64_t(nowc - nc)) / double(std::max<uint64_t>(1, guard)) * 5.0;
            // what led to it (2026-10-28): how long nothing was sent, the planning since, the board's queue
            char why[400];
            snprintf(why, sizeof why,
                     "a move was due in %.3f ms and was not sent (the board would have stopped with Timer too "
                     "close); home again. Nothing sent for %.0f ms (%llu planner segments since; %s), last send "
                     "%.0f ms for %u commands, board move queue %d of %d, longest gap between sends so far %.0f ms",
                     -ms, (fresh - last_send_t_) * 1e3, (unsigned long long)segs_since_send_,
                     may_stop ? "between loop passes" : "inside one planning burst", last_send_s_ * 1e3,
                     unsigned(last_send_cmds_), inflight_count_, limit, perf_.max_send_gap * 1e3);
            if (!may_stop) {
                late_ = why;
                return false;
            }
            last_error_ = why;
            if (io_.log) io_.log("moves: " + last_error_);
            if (io_.report) io_.report("moves: " + last_error_);
            stop(fresh);
            home_msg_ = "FAILED: " + last_error_;
            return false;
        }
    }
    std::vector<std::string> cmds;
    std::vector<uint64_t> ends;   // each queue_step's last step: it leaves the board's queue then
    int moves = 0;
    {
        PerfScope ps(this, MotionPerf::COMPRESS);
        for (auto& q : sq_) {
            if (!q.pending()) continue;
            moves += q.flush(upto, cmds, &ends);
        }
    }
    if (cmds.empty()) return true;
    perf_.queue_steps += uint64_t(moves);
    perf_.commands += cmds.size();
    for (auto& c : cmds) perf_.command_chars += c.size();
    std::string err;
    bool sent;
    in_send_ = true;
    double t_before = io_.now ? io_.now() : fresh;
    {
        PerfScope ps(this, MotionPerf::SEND);
        sent = io_.send(cmds, &err);
    }
    double t_after = io_.now ? io_.now() : t_before;
    in_send_ = false;
    if (last_send_t_ > 0 && fresh - last_send_t_ > perf_.max_send_gap) perf_.max_send_gap = fresh - last_send_t_;
    last_send_t_ = t_after;
    last_send_s_ = t_after - t_before;
    last_send_cmds_ = cmds.size();
    perf_.max_send_s = std::max(perf_.max_send_s, last_send_s_);
    perf_.sends++;
    if (!may_stop) perf_.burst_sends++;
    segs_since_send_ = 0;
    if (!sent && io_.log) io_.log("moves: " + err);
    for (uint64_t e : ends) inflight_.push({e, 1});
    inflight_count_ += moves;
    perf_.max_inflight = std::max(perf_.max_inflight, inflight_count_);
    return true;
}

std::string Toolhead::status_json(double now) const {
    if (!ok_) return "{\"ok\":false,\"why\":\"" + why_not_ + "\"}";
    std::string h;
    for (int a = 0; a < 3; a++)
        if (homed_[a]) h += char(tolower(AXL[a]));
    char b[600];
    const ShapingConfig& sc = shaping();
    snprintf(b, sizeof b,
             "{\"ok\":true,\"pos\":[%.3f,%.3f,%.3f,%.3f],\"homed\":\"%s\",\"busy\":%s,\"homing\":%s,\"msg\":\"%s\","
             "\"min\":[%.1f,%.1f,%.1f],\"max\":[%.1f,%.1f,%.1f],\"lookahead\":%d,\"shaper_x\":\"%s\",\"shaper_y\":\"%s\","
             "\"pa\":%.4f,\"pa_smooth\":%.3f}",
             gpos_[0], gpos_[1], gpos_[2], gpos_[3], h.c_str(), idle(now) ? "false" : "true",
             hstate_ != H_IDLE ? "true" : "false", home_msg_.c_str(),
             plan_.steppers[size_t(si_[0])].position_min, plan_.steppers[size_t(si_[1])].position_min,
             plan_.steppers[size_t(si_[2])].position_min, plan_.steppers[size_t(si_[0])].position_max,
             plan_.steppers[size_t(si_[1])].position_max, plan_.steppers[size_t(si_[2])].position_max,
             pcfg_.lookahead, sc.x.n ? (sc.x_type + " " + f3(sc.x_freq)).c_str() : "",
             sc.y.n ? (sc.y_type + " " + f3(sc.y_freq)).c_str() : "", sc.pa, sc.pa_smooth);
    return b;
}

}  // namespace ph
