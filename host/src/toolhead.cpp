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
    if (!count) return;
    runs_.push_back({first, interval, count, 0, int8_t(dir)});
    pending_steps_ += count;
}

double StepQueue::last_pushed() const {
    if (runs_.empty()) return double(last_clock_);
    const Run& r = runs_.back();
    return r.first + r.interval * (r.count - 1);
}

void StepQueue::reset() {
    need_reset_ = true;
    cur_dir_ = -1;
}

int StepQueue::flush(uint64_t upto, std::vector<std::string>& out) {
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
    pcfg_.lookahead = 32;
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
}

void Toolhead::on_segment(const Segment& s) {
    if (t_ < now_ + 0.05) t_ = now_ + 0.15;   // planned late (held for look-ahead): never schedule in the past
    double T = double(s.n) / pcfg_.dda_hz;
    double c0 = double(io_.clock_at(t_)), c1 = double(io_.clock_at(t_ + T));
    for (int a = 0; a < NUM_AXES; a++) {
        int32_t st = s.steps[a];
        if (!st || si_[a] < 0) continue;
        uint32_t k = uint32_t(st < 0 ? -st : st);
        double iv = (c1 - c0) / k;
        sq_[size_t(si_[a])].push_run(st > 0 ? 1 : 0, c0 + 0.5 * iv, iv, k);
    }
    t_ += T;
    t_end_ = t_;
}

void Toolhead::ensure_time(double now) {
    if (io_.now) now = std::max(now, io_.now());
    if (t_ < now + 0.1) t_ = now + START_DELAY;
}

void Toolhead::enable_all(double now) {
    if (enabled_) return;
    for (int a = 0; a < NUM_AXES; a++)
        if (si_[a] >= 0 && io_.enable) io_.enable(size_t(si_[a]), true, now);
    enabled_ = true;
}

void Toolhead::flush_lookahead() {
    if (planner_ && planner_->queued()) planner_->flush();
}

double Toolhead::buffered(double now) const {
    double b = t_end_ > now ? t_end_ - now : 0;
    if (planner_ && planner_->queued()) b += 0.25;
    return b;
}

bool Toolhead::idle(double now) const {
    if (!ok_) return true;
    if (hstate_ != H_IDLE || planner_->queued()) return false;
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
    for (bool& h : homed_) h = false;   // positions are no longer known
    hstate_ = H_IDLE;
    t_ = t_end_ = now;
    inflight_.clear();
    inflight_count_ = 0;
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
    ensure_time(now);
    enable_all(now);
    planner_->add_move(tgt, feed);
    last_add_ = now;
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
    if (de != 0 && io_.extruder_temp && io_.extruder_temp() < plan_.printer.min_extrude_temp)
        return "error: Extrude below minimum temp (" + f3(plan_.printer.min_extrude_temp) + " C, min_extrude_temp)";
    tgt[AX_E] = planner_->position(AX_E) + de * extrude_factor_;
    std::string lim = check_limits(tgt);
    if (!lim.empty()) return "error: " + lim;
    for (int a = 0; a < NUM_AXES; a++) gpos_[a] = npos[a];
    bool moves = false;
    for (int a = 0; a < NUM_AXES; a++) moves |= std::fabs(tgt[a] - planner_->position(a)) > 1e-9;
    if (!moves) return "ok";
    add_planned_move(tgt, feed_ * speed_factor_, now);
    if (immediate) flush_lookahead();
    return "ok";
}

std::string Toolhead::jog(int axis, double dist, double speed, double now) {
    if (!ok_) return "error: moves are not set up: " + why_not_;
    if (hstate_ != H_IDLE) return "error: busy homing / probing";
    if (axis < 0 || axis >= NUM_AXES || speed <= 0) return "error: bad jog";
    now_ = now;
    double tgt[NUM_AXES];
    for (int a = 0; a < NUM_AXES; a++) tgt[a] = planner_->position(a);
    if (axis == AX_E) {
        if (si_[AX_E] < 0) return "error: no [extruder] stepper in printer.cfg";
        if (io_.extruder_temp && io_.extruder_temp() < plan_.printer.min_extrude_temp)
            return "error: Extrude below minimum temp (" + f3(plan_.printer.min_extrude_temp) + " C, min_extrude_temp)";
    }
    tgt[axis] += dist;
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
                                   "M221", "M400", "PROBE", "SET_KINEMATIC_POSITION", "QUERY_ENDSTOPS", "GET_POSITION"};
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
    if (!ok) {
        home_fail("the board did not report the stepper position", now);
        return false;
    }
    if (io_.now) now = std::max(now, io_.now());   // the TMC set-up and the position query took time
    double t0 = now + 0.3;
    double ta = speed / accel, da = speed * speed / (2 * accel);
    double move_t = dist <= da ? std::sqrt(2 * dist / accel) : ta + (dist - da) / speed;
    double f = double(io_.clock_at(now + 1.0)) - double(io_.clock_at(now));
    std::string T = std::to_string(s.trsync_oid), S = std::to_string(s.oid), E = std::to_string(es.oid);
    std::vector<std::string> cmds = {
        "trsync_start oid=" + T + " report_clock=" + std::to_string(uint32_t(io_.clock_at(t0))) +
            " report_ticks=" + std::to_string(uint32_t(0.05 * f)) + " expire_reason=3",
        "stepper_stop_on_trigger oid=" + S + " trsync_oid=" + T,
        "trsync_set_timeout oid=" + T + " clock=" + std::to_string(uint32_t(io_.clock_at(t0 + move_t + 0.2))),
        "endstop_home oid=" + E + " clock=" + std::to_string(uint32_t(io_.clock_at(t0))) +
            " sample_ticks=" + std::to_string(uint32_t(0.000015 * f)) + " sample_count=4 rest_ticks=" +
            std::to_string(uint32_t(0.0001 * f)) + " pin_value=" + (es.invert ? "0" : "1") + " trsync_oid=" + T +
            " trigger_reason=1"};
    sq_[i].reset();
    schedule_homing_move(i, dir, dist, speed, accel, t0);
    sq_[i].flush(UINT64_MAX, cmds);
    std::string err;
    if (!io_.send(cmds, &err)) {
        tmc_sensorless(e, false);
        home_fail("could not send the homing move: " + err, now);
        return false;
    }
    hj_.t_end = t0 + move_t;
    hj_.triggered = hj_.finished = false;
    hj_.reason = 0;
    t_end_ = std::max(t_end_, hj_.t_end);
    home_msg_ = std::string(hj_.probe ? "probing Z" : "homing ") + (hj_.probe ? "" : std::string(1, AXL[a])) +
                (hj_.pass ? " (second, slower approach)" : "");
    return true;
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
    sq_[i].clear_pending();
    sq_[i].reset();
    tmc_sensorless(e, false);
    bool ok = true;
    int64_t after = mcu_position(i, &ok);
    double moved = double(after - hj_.pos_before) / s.steps_per_mm() * (s.invert_dir ? -1.0 : 1.0);
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
    now_ = now;
    home_step(now);
    // nothing new for a moment, or about to run dry: plan out what is queued
    if (planner_->queued() && (now - last_add_ > 0.3 || t_ - now < 0.3)) flush_lookahead();
    // make room in the board's move queue
    uint64_t nowc = io_.clock_at(now);
    while (!inflight_.empty() && inflight_.front().first <= nowc) {
        inflight_count_ -= inflight_.front().second;
        inflight_.pop_front();
    }
    int limit = plan_.move_count > 100 ? plan_.move_count - 60 : 200;
    if (inflight_count_ >= limit) return;
    uint64_t upto = io_.clock_at(now + SEND_AHEAD);
    std::vector<std::string> cmds;
    int moves = 0;
    uint64_t last = 0;
    for (auto& q : sq_) {
        if (!q.pending()) continue;
        moves += q.flush(upto, cmds);
        last = std::max(last, q.last_clock());
    }
    if (cmds.empty()) return;
    std::string err;
    if (!io_.send(cmds, &err) && io_.log) io_.log("moves: " + err);
    if (moves) {
        inflight_.push_back({last, moves});
        inflight_count_ += moves;
    }
}

std::string Toolhead::status_json(double now) const {
    if (!ok_) return "{\"ok\":false,\"why\":\"" + why_not_ + "\"}";
    std::string h;
    for (int a = 0; a < 3; a++)
        if (homed_[a]) h += char(tolower(AXL[a]));
    char b[400];
    snprintf(b, sizeof b,
             "{\"ok\":true,\"pos\":[%.3f,%.3f,%.3f,%.3f],\"homed\":\"%s\",\"busy\":%s,\"homing\":%s,\"msg\":\"%s\","
             "\"min\":[%.1f,%.1f,%.1f],\"max\":[%.1f,%.1f,%.1f]}",
             gpos_[0], gpos_[1], gpos_[2], gpos_[3], h.c_str(), idle(now) ? "false" : "true",
             hstate_ != H_IDLE ? "true" : "false", home_msg_.c_str(),
             plan_.steppers[size_t(si_[0])].position_min, plan_.steppers[size_t(si_[1])].position_min,
             plan_.steppers[size_t(si_[2])].position_min, plan_.steppers[size_t(si_[0])].position_max,
             plan_.steppers[size_t(si_[1])].position_max, plan_.steppers[size_t(si_[2])].position_max);
    return b;
}

}  // namespace ph
