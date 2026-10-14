// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
//
// Skipper Flipper segment-queue core.
//
// Pure C99, no dependencies, no hardware access. The same file is
//   - compiled into the Klipper-derived MCU firmware (via segq.c glue), and
//   - compiled on the PC, where host/test/ drives it tick by tick.
//
// The host sends constant-velocity SEGMENTS: "move these signed step counts
// over n DDA ticks". Every DDA tick the firmware calls segq_core_tick(), which
// runs a per-axis Bresenham accumulator and says which axes step now.
//
// Rules (checked by segq_core_push):
//   * 1 <= n <= SEGQ_MAX_N
//   * 2 * |steps[axis]| <= n   (max step rate = half the DDA rate, so every
//     step pulse is high for one tick and low for at least one tick)
// A segment moves each axis by EXACTLY steps[axis] (no rounding error).
// The first tick of a segment never steps, so the direction pin has one full
// tick to settle.
#ifndef SKIPPER_SEGQ_CORE_H
#define SKIPPER_SEGQ_CORE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SEGQ_AXES 4              // X, Y, Z, E
#define SEGQ_MAX_N (1UL << 30)   // longest segment, in DDA ticks

#define SEGQ_FLAG_SYNC 0x01      // reported when the segment starts (host sync marker)
#define SEGQ_FLAG_LAST 0x02      // end of a motion burst: going idle afterwards is normal

enum segq_result {
    SEGQ_OK = 0,
    SEGQ_ERR_FULL = 1,           // queue full: host must wait for free space
    SEGQ_ERR_INVALID = 2         // n == 0, too long, or step rate above half the DDA rate
};

struct segq_seg {
    uint32_t n;                  // duration in DDA ticks
    int32_t  steps[SEGQ_AXES];   // signed step counts for this segment
    uint16_t tag;                // host-chosen id, echoed in status
    uint8_t  flags;
};

struct segq_core {
    // ring buffer (storage is supplied by the caller, so firmware can size it)
    struct segq_seg *q;
    uint16_t depth, head, count, last_queued;
    uint16_t start_threshold;    // idle -> running once this many segments are queued

    uint8_t  waiting;            // 1 = idle, waiting for start_threshold or a LAST segment
    uint8_t  dir_mask;           // current direction bits (bit set = negative)

    // segment being executed
    uint32_t n_total, n_left;
    uint32_t mag[SEGQ_AXES], err[SEGQ_AXES];
    uint16_t cur_tag;
    uint8_t  cur_flags;

    int32_t  pos[SEGQ_AXES];     // executed step position (signed)
    uint16_t underruns;          // queue ran dry mid-burst (no LAST flag) since init
};

struct segq_tick {
    uint8_t step_mask;           // bit per axis: emit a step pulse now
    uint8_t dir_mask;            // direction bit per axis (1 = negative)
    uint8_t dir_changed;         // axes whose direction changed (only set on a segment's first tick)
    uint8_t seg_started;         // 1 on the first tick of a segment
    uint8_t seg_flags;           // flags of the segment that just started
    uint16_t seg_tag;            // tag of the segment that just started
};

// storage must hold `depth` entries. start_threshold of 0 or 1 starts at once.
void segq_core_init(struct segq_core *c, struct segq_seg *storage, uint16_t depth,
                    uint16_t start_threshold);

enum segq_result segq_core_push(struct segq_core *c, const struct segq_seg *s);

// Call once per DDA tick. Fills *out.
void segq_core_tick(struct segq_core *c, struct segq_tick *out);

// Drop everything queued and stop now (pause / emergency). Position is kept.
void segq_core_stop(struct segq_core *c);

static inline uint16_t segq_core_free(const struct segq_core *c) {
    return (uint16_t)(c->depth - c->count);
}
// 1 while a segment is executing or queued
static inline int segq_core_active(const struct segq_core *c) {
    return c->n_left != 0 || c->count != 0;
}

#ifdef __cplusplus
}
#endif
#endif
