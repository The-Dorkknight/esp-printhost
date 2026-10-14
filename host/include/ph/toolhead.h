// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Moves, homing and probing on a cartesian printer, using the board's own Klipper stepper commands.
//
//  G-code -> ph::Planner (look-ahead, trapezoids; the project's planner, docs/ENGINE.md) -> constant-
//  speed segments -> exact step times per stepper -> StepQueue compresses them into Klipper's
//  queue_step (interval, count, add) within 25 us, like Klipper's stepcompress -> the board steps.
//  Homing and probing use Klipper's endstop_home + trsync (the board stops the stepper itself when
//  the endstop triggers); sensorless homing sets up the TMC2209 like Klipper's tmc.py.
#pragma once
#include "ph/boardcfg.h"
#include "ph/gcode.h"
#include "ph/msgproto.h"
#include "ph/planner.h"

#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ph {

// One stepper's step times -> queue_step / set_next_step_dir / reset_step_clock commands.
class StepQueue {
public:
    StepQueue() = default;
    void init(int oid, bool invert_dir, uint32_t max_error_ticks, uint32_t min_interval_ticks);
    // `count` evenly spaced steps in direction dir (1 = positive), the first at board clock `first`
    // (64-bit, fractional), then every `interval` ticks. Runs must come in time order.
    void push_run(int dir, double first, double interval, uint32_t count);
    // Compress and append commands for steps at or before `upto`. Returns the number of queue_step
    // commands (each takes a slot in the board's move queue).
    int flush(uint64_t upto, std::vector<std::string>& out);
    // The board stopped this stepper (trsync) or something else drove it: start afresh next time.
    void reset();
    void clear_pending() { runs_.clear(); pending_steps_ = 0; }
    uint64_t pending() const { return pending_steps_; }
    uint64_t last_clock() const { return last_clock_; }       // last step sent
    double last_pushed() const;                                 // clock of the last step queued here

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

struct ToolheadIO {
    std::function<bool(const std::vector<std::string>&, std::string*)> send;
    std::function<bool(const std::string&, const std::string&, int, Message&, int)> query;
    std::function<uint64_t(double)> clock_at;
    std::function<void(size_t stepper, bool on, double now)> enable;     // stepper enable pins
    std::function<bool(int tmc, int reg, uint32_t val)> tmc_write;       // for sensorless homing
    std::function<double()> extruder_temp;                               // min_extrude_temp check
    std::function<void(const std::string&)> log;
    std::function<double()> now;   // optional: fresh host time after blocking steps
    std::function<size_t()> largest_free;   // optional: biggest free heap block (ESP32: no exceptions, a failed new aborts)
};

class Toolhead {
public:
    Toolhead(BoardPlan& plan, ToolheadIO io);
    // True when [stepper_x], [stepper_y], [stepper_z] and [printer] are present and usable.
    bool ok() const { return ok_; }
    const std::string& why_not() const { return why_not_; }

    // Motion G-code: G0 G1 G4 G28 G90 G91 G92 M82 M83 M84 M18 M114 M204 M220 M221 M400, and the
    // extended PROBE, SET_KINEMATIC_POSITION, QUERY_ENDSTOPS. `immediate` (console, jog): run the
    // move now rather than waiting for more moves to plan with. Returns "" if the command is not a
    // motion command, else the reply ("ok ..." or "error: ...").
    std::string gcode(const std::string& name, const GLine* g, const std::map<std::string, std::string>& p,
                      bool immediate, double now);
    // A relative move of one axis (0..3 = X, Y, Z, E) at `speed` mm/s, run at once (jog buttons).
    std::string jog(int axis, double dist, double speed, double now);
    void on_message(const Message& m, double now);
    void tick(double now);

    // Seconds of motion already planned ahead of now (0 when idle).
    double buffered(double now) const;
    bool idle(double now) const;                 // no motion queued or running, not homing / probing
    bool homing() const { return hstate_ != H_IDLE; }
    void flush_lookahead();                       // plan out everything queued (ends at rest)
    void motors_off(double now);                  // M84: drivers off, axes no longer homed
    void stop(double now);                        // emergency: drop all queued motion (board shut down)
    void stepper_used_elsewhere(size_t stepper);  // a motor test drove it: resync before the next move
    bool stepper_in_use(size_t stepper, double now) const;
    std::string status_json(double now) const;
    std::string last_error() const { return last_error_; }
    double position(int axis) const { return gpos_[axis]; }   // G-code position (after G92 offsets)
    bool homed(int axis) const { return homed_[axis]; }

private:
    enum HState { H_IDLE, H_WAIT_IDLE, H_START, H_MOVING, H_RETRACT, H_DONE_AXIS };
    struct HomeJob {
        std::vector<int> axes;    // in order
        size_t idx = 0;
        int pass = 0;             // 0 first approach, 1 second (slow) approach
        bool probe = false;
        double t_end = 0, started = 0;
        int64_t pos_before = 0;
        bool triggered = false, finished = false;
        int reason = 0;
    };

    void on_segment(const Segment& s);
    std::string move(const GLine& g, bool immediate, double now);
    std::string check_limits(const double tgt[NUM_AXES]) const;
    void add_planned_move(const double tgt[NUM_AXES], double feed_mm_s, double now);
    void ensure_time(double now);
    void enable_all(double now);
    // homing / probing
    std::string start_home(const std::vector<int>& axes, bool probe, double now);
    void home_step(double now);
    bool home_begin_axis(double now);
    void home_finish_axis(double now);
    void home_fail(const std::string& why, double now);
    void tmc_sensorless(int endstop, bool on);
    int64_t mcu_position(size_t stepper, bool* ok);
    void schedule_homing_move(size_t stepper, int dir, double dist, double speed, double accel, double t0);

    BoardPlan& plan_;
    ToolheadIO io_;
    bool ok_ = false;
    std::string why_not_, last_error_;
    int si_[NUM_AXES] = {-1, -1, -1, -1};    // stepper index per axis
    std::vector<StepQueue> sq_;              // per stepper
    std::unique_ptr<Planner> planner_;
    PlannerConfig pcfg_;
    double t_ = 0;                           // host time where the next planned segment starts
    double t_end_ = 0;                       // host time the last queued step happens
    double last_add_ = 0;
    double now_ = 0;                    // host time of the last move added (for the lookahead flush)
    double gpos_[NUM_AXES] = {};             // G-code position (logical)
    double off_[NUM_AXES] = {};              // machine = logical + off
    double feed_ = 25;                       // mm/s
    double speed_factor_ = 1, extrude_factor_ = 1;
    bool rel_ = false, e_rel_ = false;
    bool homed_[3] = {false, false, false};
    bool enabled_ = false;
    std::deque<std::pair<uint64_t, int>> inflight_;   // (end clock, queue_step count) for move-queue room
    int inflight_count_ = 0;
    // homing
    HState hstate_ = H_IDLE;
    HomeJob hj_;
    std::string home_msg_;
    double probe_result_ = 0;
    bool have_probe_result_ = false;
};

}  // namespace ph
