// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Test-only MCU simulator: drives the REAL segq_core with host segments, never
// overfilling the queue, and records every step as (tick, running position) so
// tests can reconstruct motion without trusting the planner.
#pragma once
#include "ph/motion.h"
#include "segq_core.h"
#include <algorithm>
#include <cstdint>
#include <vector>

static_assert(ph::SEG_LAST == SEGQ_FLAG_LAST && ph::SEG_SYNC == SEGQ_FLAG_SYNC, "flag mismatch");

struct SegqSim {
    std::vector<segq_seg> store;
    segq_core c;
    uint64_t tick = 0;                       // DDA ticks executed so far
    std::vector<uint64_t> stamp[SEGQ_AXES];  // tick index of each step pulse
    std::vector<int64_t> cum[SEGQ_AXES];     // executed position right after that pulse
    int invalid = 0;                         // segments rejected by segq_core_push
    bool record = true;

    explicit SegqSim(uint16_t depth = 64, uint16_t thr = 1) : store(depth) {
        segq_core_init(&c, store.data(), depth, thr);
    }

    void run(const std::vector<ph::Segment>& segs, uint64_t max_ticks = 4000000000ull) {
        size_t i = 0;
        const uint64_t limit = tick + max_ticks;
        for (;;) {
            while (i < segs.size() && segq_core_free(&c) > 0) {
                segq_seg s{};
                s.n = segs[i].n;
                for (int a = 0; a < SEGQ_AXES; a++) s.steps[a] = segs[i].steps[a];
                s.flags = segs[i].flags;
                s.tag = segs[i].tag;
                if (segq_core_push(&c, &s) != SEGQ_OK) invalid++;
                i++;
            }
            if (!segq_core_active(&c)) {
                if (i >= segs.size()) break;
                continue;
            }
            if (tick >= limit) break;
            segq_tick t;
            segq_core_tick(&c, &t);
            if (record && t.step_mask) {
                for (int a = 0; a < SEGQ_AXES; a++)
                    if (t.step_mask & (1u << a)) {
                        stamp[a].push_back(tick);
                        cum[a].push_back(c.pos[a]);
                    }
            }
            tick++;
        }
    }

    int64_t pos(int a) const { return c.pos[a]; }
    // executed position after `t` ticks have run (needs record == true)
    int64_t pos_at(int a, uint64_t t) const {
        size_t idx = size_t(std::lower_bound(stamp[a].begin(), stamp[a].end(), t) - stamp[a].begin());
        return idx == 0 ? 0 : cum[a][idx - 1];
    }
};
