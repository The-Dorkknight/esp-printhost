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
#include "ph/resonance.h"
#include "ph/shaping.h"

#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <cmath>
#include <memory>
#include <queue>
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
    // The same with the times already in 1/65536 board ticks (the integer motion path, PH_FIXED).
    void push_run_fx(int dir, int64_t first_fx, int64_t interval_fx, uint32_t count);
    // Compress and append commands for steps at or before `upto`. Returns the number of queue_step
    // commands (each takes a slot in the board's move queue).
    // ends (optional): the board clock of each queue_step's last step, so the board's move queue can be
    // counted exactly (2026-10-28)
    int flush(uint64_t upto, std::vector<std::string>& out, std::vector<uint64_t>* ends = nullptr);
    // The board stopped this stepper (trsync) or something else drove it: start afresh next time.
    void reset();
    void clear_pending() { runs_.clear(); pending_steps_ = 0; }
    uint64_t pending() const { return pending_steps_; }
    uint64_t last_clock() const { return last_clock_; }       // last step sent
    // Clock of the next step not yet sent (0 if none): checked against "now" just before sending.
    uint64_t next_clock() const {
        if (runs_.empty()) return 0;
        const Run& r = runs_.front();
        return uint64_t((r.first_fx + r.iv_fx * int64_t(r.done) + 32768) >> 16);
    }
    double last_pushed() const;                                 // clock of the last step queued here

private:
    struct Run {
        uint32_t count, done;
        int8_t dir;
        int64_t first_fx, iv_fx;   // first step and interval in 1/65536 ticks
    };
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
    // Most moves of look-ahead wanted (0 = the default 32). Halved until it fits in largest_free,
    // never below 32: boards with PSRAM (Captain / Skipper builds) ask for 256 or 512.
    int lookahead = 0;
    // Optional (2026-10-27): a fast counter for the motion performance counters (ESP32: the CPU
    // cycle counter; PC: nanoseconds) and its rate per second. Without it nothing is timed.
    std::function<uint64_t()> perf_now;
    double perf_hz = 0;
    std::function<void(const std::string&)> report;   // optional: the log of important events
};

// Where the motion time goes (2026-10-27), each stage counted once (time spent in a stage it calls
// is counted in that one): G-code -> planner -> shaper (input shaping / pressure advance) -> step
// times -> compression into queue_step -> sending (on a real board this includes waiting for its
// acknowledgement). Counter units: ToolheadIO::perf_hz per second.
struct MotionPerf {
    enum { PLAN, SHAPE, STEPS, COMPRESS, SEND, N };
    uint64_t t[N] = {};
    uint64_t calls[N] = {};
    uint64_t steps = 0, queue_steps = 0, commands = 0, command_chars = 0, grid_points = 0, segments = 0;
    // Timing of the sends (host seconds; 2026-10-28): how many, the longest gap between two (the
    // steps due soonest wait that long), the longest single send, sends made in the middle of a
    // long planning burst, and how often the board's move queue was full.
    uint64_t sends = 0, burst_sends = 0, queue_full = 0;
    double max_send_gap = 0, max_send_s = 0;
    int max_inflight = 0;
    static const char* name(int i);
};

class Toolhead {
public:
    Toolhead(BoardPlan& plan, ToolheadIO io);
    // True when [stepper_x], [stepper_y], [stepper_z] and [printer] are present and usable.
    bool ok() const { return ok_; }
    const std::string& why_not() const { return why_not_; }
    int lookahead() const { return pcfg_.lookahead; }   // moves of look-ahead in use

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
    bool homed_all() const { return homed_[0] && homed_[1] && homed_[2] && hstate_ == H_IDLE; }
    void flush_lookahead();                       // plan out everything queued (ends at rest)
    void motors_off(double now);                  // M84: drivers off, axes no longer homed
    void stop(double now);                        // emergency: drop all queued motion (board shut down)
    void stepper_used_elsewhere(size_t stepper);  // a motor test drove it: resync before the next move
    bool stepper_in_use(size_t stepper, double now) const;
    std::string status_json(double now) const;
    std::string last_error() const { return last_error_; }
    double position(int axis) const { return gpos_[axis]; }   // G-code position (after G92 offsets)
    bool homed(int axis) const { return homed_[axis]; }
    // How far the last homing move of this axis went before it stopped (mm; NAN if not known).
    double last_home_moved(int axis) const { return last_moved_[axis]; }
    // Forget that an axis is homed (a test moved it, or a hard stop gave a false trigger): the next
    // move of that axis is refused until it is homed again.
    void unhome(int axis) { if (axis >= 0 && axis < 3) homed_[axis] = false; }
    // Stop homing / probing: before the move is sent, nothing is sent; while it runs, the board is
    // told to stop the stepper (trsync). Either way the axis ends up not homed.
    void abort_homing(double now);
    // The axis (0..3) a stepper drives, or -1; its machine position (mm) at the end of what is queued.
    int axis_of(size_t stepper) const {
        for (int a = 0; a < NUM_AXES; a++)
            if (si_[a] == int(stepper)) return a;
        return -1;
    }
    double machine_position(int axis) const { return planner_ ? planner_->position(axis) : 0; }
    // Bed mesh compensation (bedmesh.h): Z follows the mesh under X / Y moves, long moves split into
    // 5 mm pieces (Klipper's move_check_distance). nullptr clears it.
    void set_mesh(const BedMesh* m) { mesh_ = m ? *m : BedMesh(); }
    const BedMesh& mesh() const { return mesh_; }
#if PH_PRO
    double mesh_z(double x, double y) const { return mesh_.valid() ? mesh_.at(x, y) : 0.0; }
#else
    double mesh_z(double, double) const { return 0.0; }
#endif
    // Firmware retraction (G10 / G11) settings, from [firmware_retraction] or SET_RETRACTION.
    RetractConfig& retraction() { return retract_; }
    // A move of an excluded object (EXCLUDE_OBJECT): the G-code position follows it, nothing moves,
    // so the next move goes straight to where the G-code is and no filament is pushed for it.
    void skip_move(const GLine& g);
    bool relative() const { return rel_; }
    // An absolute move to machine X / Y / Z (NAN: stays) at `speed` mm/s, mesh applied; the G-code
    // position follows. For the probing sequences (bed mesh, screws) and the resonance test.
    std::string move_to(const double xyz[3], double speed, double now);
    // The last PROBE's z (machine coordinates), if one finished since the last start_home.
    bool probe_result(double& z) const { if (have_probe_result_) z = probe_result_; return have_probe_result_; }

    // Input shaping and pressure advance (shaping.h; CAPTAIN / SKIPPER builds). Takes effect at once
    // when nothing is moving, else when the motion next comes to rest. Returns "" or why not.
    std::string set_shaping(const ShapingConfig& c, double now);
    const ShapingConfig& shaping() const { return shaping_pending_ ? shaping_next_ : shaper_.config(); }
    uint64_t shaper_grid_segments() const { return shaper_.grid_segments(); }

    // Performance counters (needs ToolheadIO::perf_now).
    const MotionPerf& perf() const { return perf_; }
    void perf_reset() { perf_ = MotionPerf(); last_send_t_ = 0; }   // (idle time before is not a "gap")
    double perf_hz() const { return io_.perf_hz; }
    // Benchmark mode (SF_BENCH, 2026-10-27): the stepper drivers are never switched on and
    // min_extrude_temp is not checked, so a print runs through the whole motion pipeline (and, on a
    // real board, its step queues) without anything moving or heating.
    void set_bench(bool on) { bench_ = on; }
    bool bench() const { return bench_; }
    // Mark all axes homed at the current position (SF_BENCH; SET_KINEMATIC_POSITION does the same).
    void assume_homed() { homed_[0] = homed_[1] = homed_[2] = true; }
#if PH_FIXED
    static constexpr const char* MATHS = "INT32/64 TEST BUILD (whole-number motion maths)";
#else
    static constexpr const char* MATHS = "STANDARD build (double motion maths)";
#endif

    // The resonance test motion along X (0) or Y (1), from where the head is now, with exact step
    // times generated a fraction of a second ahead while it runs (resonance_tester.py run_test).
    // Needs the axis homed and the room for the whole motion inside the axis limits.
    std::string start_resonance(int axis, const ResonanceMotion& m, double now);
    bool resonance_running() const { return rs_.active; }
    // seconds into the test (or -1), and the test frequency there
    double resonance_elapsed(double now) const { return rs_.active ? now - rs_.t0 : -1; }
    void abort_resonance(double now);

private:
    enum HState { H_IDLE, H_WAIT_IDLE, H_START, H_MOVING, H_RETRACT, H_DONE_AXIS };
    struct HomeJob {
        std::vector<int> axes;    // in order
        size_t idx = 0;
        int pass = 0;             // 0 first approach, 1 second (slow) approach
        bool probe = false;
        double t_end = 0, started = 0;
        int64_t pos_before = 0, pos_before_b = 0;   // _b: CoreXY, the other motor
        bool triggered = false, finished = false;
        int reason = 0;
        int late_tries = 0;       // homing move prepared too late to send (see home_begin_axis)
    };

    void on_segment(const Segment& s);
    void on_segment_plan(const Segment& s);
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
    double last_moved_[3] = {NAN, NAN, NAN};
    bool enabled_ = false;
    // (end clock, queue_step count) of what is in the board's move queue, soonest first: one entry per
    // queue_step, freed when its last step has run (2026-10-28; was one entry per send with the
    // latest end of all, which held every slot up to 0.6 s)
    std::priority_queue<std::pair<uint64_t, int>, std::vector<std::pair<uint64_t, int>>,
                        std::greater<std::pair<uint64_t, int>>> inflight_;
    int inflight_count_ = 0;
    // Send what is due (compress + send), also in the middle of a long planning burst. may_stop:
    // a step found too late stops everything at once; otherwise (inside the planner) late_ is set
    // and tick() stops when the burst is over. False if a step was too late.
    bool send_due(double now, bool may_stop);
    void send_soon();
    double last_send_t_ = 0, last_send_s_ = 0;
    size_t last_send_cmds_ = 0;
    uint64_t segs_since_send_ = 0;
    bool in_send_ = false;
    std::string late_;
    // homing
    HState hstate_ = H_IDLE;
    HomeJob hj_;
    std::string home_msg_;
    double probe_result_ = 0;
    bool have_probe_result_ = false;
    // input shaping / pressure advance between the planner and the step queues
#if PH_FIXED
    MotionShaperFx shaper_;
    // host time (us) -> board clock, a straight line refreshed every planner segment
    int64_t map_us_ = 0;
    uint64_t map_clk_ = 0;
    int64_t map_q32_ = 0;   // board ticks per us, 32 fraction bits
    void map_refresh(double t);
    uint64_t clock_us(int64_t us) const;
    void emit_steps_us(int64_t us0, int64_t us1, const int32_t steps[NUM_AXES]);
#else
    MotionShaper shaper_;
#endif
    ShapingConfig shaping_next_;
    bool shaping_pending_ = false;
    void emit_steps(double t0, double dur, const int32_t steps[NUM_AXES]);
    void emit_steps_clk(uint64_t c0, uint64_t c1, const int32_t steps[NUM_AXES]);
    // performance counters
    MotionPerf perf_;
    uint64_t perf_child_ = 0;
    bool bench_ = false;
    struct PerfScope {
        Toolhead* th;
        int b;
        uint64_t t0 = 0, saved = 0;
        PerfScope(Toolhead* t, int bucket) : th(t->io_.perf_now ? t : nullptr), b(bucket) {
            if (!th) return;
            saved = th->perf_child_;
            th->perf_child_ = 0;
            t0 = th->io_.perf_now();
        }
        ~PerfScope() {
            if (!th) return;
            uint64_t el = th->io_.perf_now() - t0;
            th->perf_.t[b] += el > th->perf_child_ ? el - th->perf_child_ : 0;
            th->perf_.calls[b]++;
            th->perf_child_ = saved + el;
        }
    };
    BedMesh mesh_;
    RetractConfig retract_;
    bool retracted_ = false;
    std::string arc(const GLine& g, bool clockwise, bool immediate, double now);
    struct ResRun {
        bool active = false;
        int axis = 0;
        ResonanceMotion m;
        double t0 = 0, gen = 0;   // host time of the start; generated up to (s from the start)
        size_t piece = 0;
        int64_t s = 0;            // steps from the start
        double spm = 1;
    } rs_;
    // CoreXY (Klipper corexy.py): motor A (stepper_x) = X + Y, motor B (stepper_y) = X - Y, in steps.
    bool corexy_ = false;
    // The motors an axis moves, and the direction of each for a positive move: (stepper, +1/-1).
    std::vector<std::pair<size_t, int>> axis_motors(int axis) const;
    void resonance_generate(double upto);   // seconds from the start
    void resonance_finish(double now);
};

}  // namespace ph
