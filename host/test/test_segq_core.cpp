// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Tests for the segment-queue core that is shared with the MCU firmware.
#include "check.h"
#include "segq_core.h"
#include <cstdint>
#include <cstdlib>
#include <vector>

namespace {

segq_seg mk(uint32_t n, int32_t x, int32_t y = 0, int32_t z = 0, int32_t e = 0,
            uint16_t tag = 0, uint8_t flags = 0) {
    segq_seg s{};
    s.n = n; s.steps[0] = x; s.steps[1] = y; s.steps[2] = z; s.steps[3] = e;
    s.tag = tag; s.flags = flags;
    return s;
}

struct Rig {
    std::vector<segq_seg> store;
    segq_core c;
    uint64_t tick = 0;
    explicit Rig(uint16_t depth = 64, uint16_t thr = 1) : store(depth) {
        segq_core_init(&c, store.data(), depth, thr);
    }
    // one DDA tick
    segq_tick step() { segq_tick t; segq_core_tick(&c, &t); tick++; return t; }
};

uint32_t rnd(uint32_t& s) { s = s * 1664525u + 1013904223u; return s >> 8; }

void t_exact_and_even() {
    uint32_t seed = 12345;
    Rig r(32, 1);
    int64_t want[SEGQ_AXES] = {};
    // per-axis step tick history, to check spacing
    std::vector<uint64_t> hist[SEGQ_AXES];
    int seg_count = 0;
    for (int round = 0; round < 40; round++) {
        // fill a burst of random valid segments
        int pushed = 0;
        while (pushed < 20) {
            uint32_t n = 1 + rnd(seed) % 3000;
            segq_seg s = mk(n, 0);
            for (int a = 0; a < SEGQ_AXES; a++) {
                int32_t m = (int32_t)(rnd(seed) % (n / 2 + 1));
                s.steps[a] = (rnd(seed) & 1) ? -m : m;
            }
            s.flags = (pushed == 19) ? SEGQ_FLAG_LAST : 0;
            if (segq_core_push(&r.c, &s) != SEGQ_OK) break;
            for (int a = 0; a < SEGQ_AXES; a++) want[a] += s.steps[a];
            pushed++; seg_count++;
        }
        int guard = 0;
        while (segq_core_active(&r.c) && guard++ < 10000000) {
            segq_tick t = r.step();
            for (int a = 0; a < SEGQ_AXES; a++)
                if (t.step_mask & (1u << a)) hist[a].push_back(r.tick);
        }
        CHECK(guard < 10000000);
    }
    for (int a = 0; a < SEGQ_AXES; a++) {
        CHECK(r.c.pos[a] == want[a]);
        // every step is at least 2 ticks after the previous one on the same axis
        bool spaced = true;
        for (size_t i = 1; i < hist[a].size(); i++)
            if (hist[a][i] - hist[a][i - 1] < 2) { spaced = false; break; }
        CHECK(spaced);
    }
    CHECK(r.c.underruns == 0);   // bursts ended with LAST
    CHECK(seg_count > 100);
}

void t_per_segment_exact_and_even() {
    // one axis, 7 steps over 100 ticks: exactly 7 steps, gaps within 1 of each other
    Rig r(8, 1);
    segq_seg s = mk(100, 7, 0, 0, 0, 1, SEGQ_FLAG_LAST);
    CHECK(segq_core_push(&r.c, &s) == SEGQ_OK);
    std::vector<int> at;
    for (int i = 0; i < 100; i++) {
        segq_tick t = r.step();
        if (t.step_mask & 1) at.push_back(i);
        if (i == 0) CHECK(!(t.step_mask & 1));   // first tick never steps
    }
    CHECK(at.size() == 7);
    int lo = 1000, hi = 0;
    for (size_t i = 1; i < at.size(); i++) { int g = at[i] - at[i - 1]; lo = g < lo ? g : lo; hi = g > hi ? g : hi; }
    CHECK(hi - lo <= 1);
    CHECK(r.c.pos[0] == 7);
}

void t_validation() {
    Rig r(8, 1);
    segq_seg s = mk(0, 0);
    CHECK(segq_core_push(&r.c, &s) == SEGQ_ERR_INVALID);          // n == 0
    s = mk(10, 5);
    CHECK(segq_core_push(&r.c, &s) == SEGQ_OK);                    // exactly half rate
    s = mk(10, 6);
    CHECK(segq_core_push(&r.c, &s) == SEGQ_ERR_INVALID);          // over half rate
    s = mk(10, 0, -6);
    CHECK(segq_core_push(&r.c, &s) == SEGQ_ERR_INVALID);          // negative too
    s = mk(10, 0, 0, 0, INT32_MIN);
    CHECK(segq_core_push(&r.c, &s) == SEGQ_ERR_INVALID);          // no overflow on abs()
    s = mk((uint32_t)SEGQ_MAX_N + 1u, 0);
    CHECK(segq_core_push(&r.c, &s) == SEGQ_ERR_INVALID);          // too long
    s = mk((uint32_t)SEGQ_MAX_N, 0);
    CHECK(segq_core_push(&r.c, &s) == SEGQ_OK);                    // longest allowed
    CHECK(segq_core_free(&r.c) == 6);
}

void t_ring_and_full() {
    Rig r(4, 1);
    for (int i = 0; i < 4; i++) CHECK(segq_core_push(&r.c, &(const segq_seg&)mk(2, 1, 0, 0, 0, (uint16_t)i)) == SEGQ_OK);
    segq_seg extra = mk(2, 1);
    CHECK(segq_core_push(&r.c, &extra) == SEGQ_ERR_FULL);
    // drain two segments (2 ticks each), then the ring must wrap on the next pushes
    std::vector<uint16_t> tags;
    for (int i = 0; i < 4; i++) { segq_tick t = r.step(); if (t.seg_started) tags.push_back(t.seg_tag); }
    CHECK(tags.size() == 2 && tags[0] == 0 && tags[1] == 1);
    for (int i = 4; i < 6; i++) CHECK(segq_core_push(&r.c, &(const segq_seg&)mk(2, 1, 0, 0, 0, (uint16_t)i)) == SEGQ_OK);
    CHECK(segq_core_free(&r.c) == 0);
    for (int i = 0; i < 12; i++) { segq_tick t = r.step(); if (t.seg_started) tags.push_back(t.seg_tag); }
    CHECK(tags.size() == 6);
    for (size_t i = 0; i < tags.size(); i++) CHECK(tags[i] == i);   // strict FIFO across the wrap
}

void t_start_threshold() {
    Rig r(16, 3);
    CHECK(segq_core_push(&r.c, &(const segq_seg&)mk(4, 1, 0, 0, 0, 1)) == SEGQ_OK);
    CHECK(segq_core_push(&r.c, &(const segq_seg&)mk(4, 1, 0, 0, 0, 2)) == SEGQ_OK);
    for (int i = 0; i < 20; i++) CHECK(!r.step().seg_started);   // 2 < threshold: holds
    CHECK(r.c.pos[0] == 0);
    CHECK(segq_core_push(&r.c, &(const segq_seg&)mk(4, 1, 0, 0, 0, 3)) == SEGQ_OK);
    CHECK(r.step().seg_started);                                  // 3 queued: go
    for (int i = 0; i < 20; i++) r.step();
    CHECK(r.c.pos[0] == 3);
    CHECK(r.c.underruns == 1);                                    // no LAST flag: counted
    // a LAST segment starts immediately even below the threshold, and is not an underrun
    CHECK(segq_core_push(&r.c, &(const segq_seg&)mk(4, 1, 0, 0, 0, 9, SEGQ_FLAG_LAST)) == SEGQ_OK);
    segq_tick t = r.step();
    CHECK(t.seg_started && t.seg_tag == 9 && (t.seg_flags & SEGQ_FLAG_LAST));
    for (int i = 0; i < 20; i++) r.step();
    CHECK(r.c.pos[0] == 4);
    CHECK(r.c.underruns == 1);
}

void t_underrun_and_rearm() {
    Rig r(16, 2);
    segq_core_push(&r.c, &(const segq_seg&)mk(4, 2));
    segq_core_push(&r.c, &(const segq_seg&)mk(4, 2));
    for (int i = 0; i < 30; i++) r.step();
    CHECK(r.c.underruns == 1);
    CHECK(!segq_core_active(&r.c));
    // after an underrun it waits for the threshold again (no stutter-start)
    segq_core_push(&r.c, &(const segq_seg&)mk(4, 2));
    for (int i = 0; i < 10; i++) r.step();
    CHECK(r.c.pos[0] == 4);
    segq_core_push(&r.c, &(const segq_seg&)mk(4, 2));
    for (int i = 0; i < 30; i++) r.step();
    CHECK(r.c.pos[0] == 8);
}

void t_stop() {
    Rig r(16, 1);
    segq_core_push(&r.c, &(const segq_seg&)mk(1000, 400));
    segq_core_push(&r.c, &(const segq_seg&)mk(1000, 400, 0, 0, 0, 0, SEGQ_FLAG_LAST));
    for (int i = 0; i < 500; i++) r.step();
    int32_t at = r.c.pos[0];
    CHECK(at > 150 && at < 250);          // about half way through the first segment
    segq_core_stop(&r.c);
    CHECK(!segq_core_active(&r.c));
    CHECK(r.c.last_queued == 0);
    for (int i = 0; i < 100; i++) CHECK(r.step().step_mask == 0);
    CHECK(r.c.pos[0] == at);              // position kept
    CHECK(r.c.underruns == 0);            // a stop is not an underrun
    segq_core_push(&r.c, &(const segq_seg&)mk(10, 5, 0, 0, 0, 0, SEGQ_FLAG_LAST));   // restarts cleanly
    for (int i = 0; i < 20; i++) r.step();
    CHECK(r.c.pos[0] == at + 5);
}

void t_direction() {
    Rig r(8, 1);
    segq_core_push(&r.c, &(const segq_seg&)mk(20, 5, -4));
    segq_core_push(&r.c, &(const segq_seg&)mk(20, -5, -4));
    segq_core_push(&r.c, &(const segq_seg&)mk(20, -5, 4, 0, 0, 0, SEGQ_FLAG_LAST));
    uint8_t changed[3] = {0, 0, 0}, dirs[3] = {0, 0, 0};
    int seg = -1;
    for (int i = 0; i < 70; i++) {
        segq_tick t = r.step();
        if (t.seg_started) { seg++; changed[seg] = t.dir_changed; dirs[seg] = t.dir_mask; }
        if (t.seg_started) CHECK(t.step_mask == 0);      // direction settles for a tick
    }
    CHECK(seg == 2);
    CHECK(dirs[0] == 0x2 && changed[0] == 0x2);          // Y negative from rest
    CHECK(dirs[1] == 0x3 && changed[1] == 0x1);          // X flips
    CHECK(dirs[2] == 0x1 && changed[2] == 0x2);          // Y flips back
    CHECK(r.c.pos[0] == -5 && r.c.pos[1] == -4);
}

void t_dwell() {
    Rig r(4, 1);
    segq_core_push(&r.c, &(const segq_seg&)mk(50, 0, 0, 0, 0, 5, SEGQ_FLAG_LAST));
    int started = 0, steps = 0;
    for (int i = 0; i < 50; i++) { segq_tick t = r.step(); started += t.seg_started; steps += t.step_mask != 0; }
    CHECK(started == 1 && steps == 0);
    CHECK(!segq_core_active(&r.c));
}

}  // namespace

void test_segq_core() {
    t_exact_and_even();
    t_per_segment_exact_and_even();
    t_validation();
    t_ring_and_full();
    t_start_threshold();
    t_underrun_and_rearm();
    t_stop();
    t_direction();
    t_dwell();
}
