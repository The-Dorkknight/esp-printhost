// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Cartesian look-ahead motion planner producing constant-velocity DDA segments
// for the toolhead (constant-speed pieces, turned into queue_step). See docs/ENGINE.md.
#pragma once
#include "ph/motion.h"
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <vector>

namespace ph {

struct PlannerConfig {
    double steps_per_mm[NUM_AXES] = {80, 80, 400, 93};   // X, Y, Z, E
    double max_velocity = 300;        // mm/s, XY component
    double max_z_velocity = 20;       // mm/s, Z component
    double max_e_velocity = 60;       // mm/s, filament
    double max_accel = 3000;          // mm/s^2 (XYZ path)
    double max_e_accel = 5000;        // mm/s^2 (E-only moves, and E limit in mixed moves)
    double junction_deviation = 0.05; // mm, Grbl style
    double min_junction_speed = 0.1;  // mm/s, floor for corner / reversal speed (>0)
    double dda_hz = 20000;            // MCU DDA tick rate
    double segment_ms = 5;            // target segment duration
    double step_rate_fraction = 0.45; // max per-axis step rate as a fraction of dda_hz (core limit is 0.5)
    int    lookahead = 256;           // max blocks queued for junction planning (>= 1); fewer are
                                      // used once enough path is queued to brake from full speed
};

class Planner {
public:
    using SegmentSink = std::function<void(const Segment&)>;
    using MarkerSink = std::function<void(uint32_t)>;

    explicit Planner(const PlannerConfig& cfg = PlannerConfig());

    void set_sinks(SegmentSink seg, MarkerSink marker = nullptr) {
        seg_sink_ = std::move(seg);
        marker_sink_ = std::move(marker);
    }
    const PlannerConfig& config() const { return cfg_; }

    // Acceleration for moves added from now on (clamped to (0, max_accel]).
    void set_accel(double a);
    double accel() const { return accel_; }
    // Call only on an empty queue (flush() first); the Engine does this for M900.
    void set_pressure_advance(double k);
    double pressure_advance() const { return pa_.k(); }
    // Bytes the look-ahead ring takes (allocated in one piece by the constructor).
    static size_t ring_bytes(int lookahead) { return (size_t(lookahead < 1 ? 1 : lookahead) + 2) * sizeof(Block); }

    // Machine-absolute position in mm (end of everything queued so far).
    double position(int axis) const { return pos_[axis]; }
    // Steps already handed to the sink (or held for the next call) per axis.
    int64_t step_position(int axis) const { return emitted_[axis]; }
    // Flushes, then declares the machine to be at `mm` (no motion).
    void set_position(const double mm[NUM_AXES]);

    // Queue a straight move to the absolute target (mm). feed_mm_s <= 0 means max_velocity.
    void add_move(const double target[NUM_AXES], double feed_mm_s);
    // Opaque marker delivered through the marker sink after every segment of the
    // moves queued before it and before any segment of moves queued after it.
    void add_marker(uint32_t id);
    // Drain the queue so the last move ends at rest; the final segment gets SEG_LAST.
    void flush();
    // Flush, then emit `ticks` DDA ticks of zero-step segments (last one SEG_LAST).
    void dwell(uint64_t ticks);

    size_t queued() const { return count_; }
    uint64_t segments_emitted() const { return seg_count_; }
    // Segments whose n had to be lengthened to satisfy 2*|steps| <= n.
    uint64_t stretched_segments() const { return stretched_; }

private:
    struct Block {
        double start[NUM_AXES], end[NUM_AXES], delta[NUM_AXES];
        double u[3];
        double L, nominal, nominal_v2, accel, max_entry_v2, entry_v2;
        int kind;   // 0 = XYZ path, 1 = E only
    };
    struct Marker { uint32_t id; uint64_t seq; };

    Block& at(size_t i) { return ring_[(head_ + i) % ring_.size()]; }
    void replan();
    void emit_head();
    void emit_block(const Block& b, double v0, double v1);
    void emit_segment(uint64_t n, const int32_t steps[NUM_AXES]);
    void emit_markers_upto(uint64_t block_index);
    void release_held(bool last);
    void out(const Segment& s);
    void correct_positions();

    PlannerConfig cfg_;
    double accel_;
    PressureAdvance pa_;
    SegmentSink seg_sink_;
    MarkerSink marker_sink_;
    std::vector<Block> ring_;
    size_t head_ = 0, count_ = 0;
    double pin_v2_ = 0;                   // squared entry speed of ring_[head_]
    double pos_[NUM_AXES] = {};
    int64_t emitted_[NUM_AXES] = {};
    double e_acc_ = 0;                    // E position (mm) as actually driven when PA is on
    std::deque<Marker> markers_;
    double q_len_ = 0;                    // total path length (mm) of queued blocks
    double tick_carry_ = 0;               // fractional-tick rounding carried between blocks
    uint64_t blocks_pushed_ = 0, blocks_emitted_ = 0;
    Segment held_;
    bool has_held_ = false;
    uint16_t tag_ = 0;
    uint64_t seg_count_ = 0, stretched_ = 0;
};

}  // namespace ph
