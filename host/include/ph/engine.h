// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// G-code interpreter on top of ph::Planner. See docs/ENGINE.md.
#pragma once
#include "ph/gcode.h"
#include "ph/motion.h"
#include "ph/planner.h"
#include <deque>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace ph {

// A command the engine does not execute itself (heaters, fan, homing, ...).
struct Action {
    std::string kind;                            // "M104", "G28", ...
    long line_no = -1;                           // N word, if any
    std::vector<std::pair<char, double>> args;   // every letter word seen, in A..Z order
    bool has(char c) const {
        for (auto& a : args) if (a.first == c) return true;
        return false;
    }
    double get(char c, double dflt = 0) const {
        for (auto& a : args) if (a.first == c) return a.second;
        return dflt;
    }
};

struct Item {
    enum Type { SEGMENT, ACTION } type = SEGMENT;
    Segment seg;       // valid when type == SEGMENT
    Action action;     // valid when type == ACTION
};

class Engine {
public:
    using Sink = std::function<void(const Item&)>;

    explicit Engine(const PlannerConfig& cfg = PlannerConfig());
    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;

    // With a sink, items go to it as they are produced; otherwise they collect
    // in an internal vector retrievable with drain().
    void set_sink(Sink s) { sink_ = std::move(s); }
    std::vector<Item> drain();

    // Interpret one line. Returns false only if the line could not be parsed
    // (a warning is recorded, state is unchanged). Unknown commands return true
    // and add a warning.
    bool feed_line(const std::string& line);
    // End of program: drain the planner (last segment gets SEG_LAST).
    void flush() { planner_.flush(); }

    const std::vector<std::string>& warnings() const { return warnings_; }
    void clear_warnings() { warnings_.clear(); }

    // Logical (G92-shifted) position in mm; E is the logical G-code E value.
    double position(int axis) const { return pos_[axis]; }
    double feed_mm_min() const { return feed_; }
    double speed_factor() const { return speed_pct_; }
    double flow_factor() const { return flow_pct_; }
    bool xyz_relative() const { return rel_xyz_; }
    bool e_relative() const { return rel_xyz_ || e_rel_; }
    Planner& planner() { return planner_; }

private:
    void emit_item(const Item& it);
    void queue_action(const GLine& g, bool sync);
    void warn(const std::string& s) { warnings_.push_back(s); }
    void do_move(const GLine& g);

    Planner planner_;
    Sink sink_;
    std::vector<Item> items_;
    std::vector<std::string> warnings_;
    std::deque<Action> pending_;
    double pos_[NUM_AXES] = {};      // logical position
    double off_[NUM_AXES] = {};      // abs = logical + off (X,Y,Z); E uses deltas only
    double feed_ = 1500;             // mm/min (F persists)
    double speed_pct_ = 100, flow_pct_ = 100;
    bool rel_xyz_ = false, e_rel_ = false;
};

}  // namespace ph
