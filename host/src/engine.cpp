// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
#include "ph/engine.h"
#include "ph/gcode.h"
#include <cmath>

namespace ph {

Engine::Engine(const PlannerConfig& cfg) : planner_(cfg) {
    planner_.set_sinks(
        [this](const Segment& s) {
            Item it;
            it.type = Item::SEGMENT;
            it.seg = s;
            emit_item(it);
        },
        [this](uint32_t) {
            if (pending_.empty()) return;
            Item it;
            it.type = Item::ACTION;
            it.action = std::move(pending_.front());
            pending_.pop_front();
            emit_item(it);
        });
}

void Engine::emit_item(const Item& it) {
    if (sink_) sink_(it);
    else items_.push_back(it);
}

std::vector<Item> Engine::drain() {
    std::vector<Item> o;
    o.swap(items_);
    return o;
}

void Engine::queue_action(const GLine& g, bool sync) {
    Action a;
    a.kind = std::string(1, g.cmd_letter) + std::to_string(g.cmd_num);
    a.line_no = g.line_no;
    for (char c = 'A'; c <= 'Z'; c++)
        if (g.has(c)) a.args.push_back({c, double(g.get(c))});
    pending_.push_back(std::move(a));
    planner_.add_marker(0);          // ids are positional: pending_ is FIFO
    if (sync) planner_.flush();      // action may not overtake motion, and motion stops here
}

void Engine::do_move(const GLine& g) {
    if (g.has('F')) {
        double f = g.get('F');
        if (f > 0) feed_ = f;
        else warn("ignored non-positive F");
    }
    const bool erel = rel_xyz_ || e_rel_;
    double tgt[NUM_AXES];
    double npos[NUM_AXES];
    static const char L[3] = {'X', 'Y', 'Z'};
    for (int a = 0; a < 3; a++) {
        npos[a] = pos_[a];
        if (g.has(L[a])) npos[a] = rel_xyz_ ? pos_[a] + g.get(L[a]) : double(g.get(L[a]));
        tgt[a] = npos[a] + off_[a];
    }
    double de = 0;
    npos[AX_E] = pos_[AX_E];
    if (g.has('E')) {
        de = erel ? double(g.get('E')) : g.get('E') - pos_[AX_E];
        npos[AX_E] = pos_[AX_E] + de;
    }
    tgt[AX_E] = planner_.position(AX_E) + de * flow_pct_ / 100.0;
    for (int a = 0; a < NUM_AXES; a++) pos_[a] = npos[a];
    planner_.add_move(tgt, feed_ / 60.0 * speed_pct_ / 100.0);
}

bool Engine::feed_line(const std::string& line) {
    GLine g;
    if (!gcode_parse(line, g)) {
        warn("parse error: " + line);
        return false;
    }
    if (!g.checksum_ok) {
        warn("bad checksum: " + line);
        return false;
    }
    if (g.cmd_letter == 0) {
        for (char c = 'A'; c <= 'Z'; c++)
            if (g.has(c)) { warn("word without command: " + line); break; }
        return true;
    }
    if (g.cmd_sub >= 0) {
        warn("unsupported command " + std::string(1, g.cmd_letter) + std::to_string(g.cmd_num) +
             "." + std::to_string(g.cmd_sub));
        return true;
    }
    if (g.cmd_letter == 'G') {
        switch (g.cmd_num) {
        case 0: case 1: do_move(g); return true;
        case 4: {
            double sec = (g.has('P') ? g.get('P') / 1000.0 : 0.0) + (g.has('S') ? double(g.get('S')) : 0.0);
            if (!(sec >= 0)) { warn("negative dwell ignored"); return true; }
            planner_.dwell(uint64_t(std::llround(sec * planner_.config().dda_hz)));
            return true;
        }
        case 21: return true;
        case 28: {
            queue_action(g, true);
            bool any = g.has('X') || g.has('Y') || g.has('Z');
            static const char L[3] = {'X', 'Y', 'Z'};
            for (int a = 0; a < 3; a++)
                if (!any || g.has(L[a])) { pos_[a] = 0; off_[a] = planner_.position(a); }
            return true;
        }
        case 90: rel_xyz_ = false; return true;
        case 91: rel_xyz_ = true; return true;
        case 92: {
            static const char L[3] = {'X', 'Y', 'Z'};
            for (int a = 0; a < 3; a++)
                if (g.has(L[a])) { pos_[a] = g.get(L[a]); off_[a] = planner_.position(a) - pos_[a]; }
            if (g.has('E')) pos_[AX_E] = g.get('E');
            return true;
        }
        default: break;
        }
    } else if (g.cmd_letter == 'M') {
        switch (g.cmd_num) {
        case 82: e_rel_ = false; return true;
        case 83: e_rel_ = true; return true;
        case 104: case 140: case 106: case 107: queue_action(g, false); return true;
        case 109: case 190: case 84: case 18: queue_action(g, true); return true;
        case 204: {
            if (g.has('S')) planner_.set_accel(g.get('S'));
            else if (g.has('P')) planner_.set_accel(g.get('P'));
            else warn("M204 without S/P");
            return true;
        }
        case 220:
            if (g.has('S') && g.get('S') > 0) speed_pct_ = g.get('S');
            else warn("M220 needs S > 0");
            return true;
        case 221:
            if (g.has('S') && g.get('S') > 0) flow_pct_ = g.get('S');
            else warn("M221 needs S > 0");
            return true;
        case 400: planner_.flush(); return true;
        case 900:
            if (g.has('K') && g.get('K') >= 0) {
                planner_.flush();
                planner_.set_pressure_advance(g.get('K'));
            } else {
                warn("M900 needs K >= 0");
            }
            return true;
        default: break;
        }
    }
    warn("unknown command " + std::string(1, g.cmd_letter) + std::to_string(g.cmd_num));
    return true;
}

}  // namespace ph
