// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Skipper Flipper segment-queue core. See segq_core.h.
#include "segq_core.h"

void segq_core_init(struct segq_core *c, struct segq_seg *storage, uint16_t depth,
                    uint16_t start_threshold)
{
    uint8_t i;
    c->q = storage;
    c->depth = depth;
    c->head = c->count = c->last_queued = 0;
    c->start_threshold = start_threshold;
    c->waiting = 1;
    c->dir_mask = 0;
    c->n_total = c->n_left = 0;
    c->cur_tag = 0;
    c->cur_flags = 0;
    for (i = 0; i < SEGQ_AXES; i++) {
        c->mag[i] = c->err[i] = 0;
        c->pos[i] = 0;
    }
    c->underruns = 0;
}

enum segq_result segq_core_push(struct segq_core *c, const struct segq_seg *s)
{
    uint8_t i;
    uint16_t idx;
    if (s->n == 0 || s->n > SEGQ_MAX_N)
        return SEGQ_ERR_INVALID;
    for (i = 0; i < SEGQ_AXES; i++) {
        // 32-bit only: this runs with interrupts off, and 64-bit helpers are
        // slow on AVR. |INT32_MIN| = 2^31 fits uint32; 2*|v| <= n <=> |v| <= n/2.
        int32_t v = s->steps[i];
        uint32_t mag = v < 0 ? 0u - (uint32_t)v : (uint32_t)v;
        if (mag > (s->n >> 1))
            return SEGQ_ERR_INVALID;
    }
    if (c->count >= c->depth)
        return SEGQ_ERR_FULL;
    idx = (uint16_t)(c->head + c->count);
    if (idx >= c->depth)
        idx = (uint16_t)(idx - c->depth);
    c->q[idx] = *s;
    c->count++;
    if (s->flags & SEGQ_FLAG_LAST)
        c->last_queued++;
    return SEGQ_OK;
}

// Load the next queued segment as the current one.
static void load_next(struct segq_core *c, struct segq_tick *out)
{
    uint8_t i, neg = 0;
    const struct segq_seg *s = &c->q[c->head];
    for (i = 0; i < SEGQ_AXES; i++) {
        int32_t v = s->steps[i];
        if (v < 0) {
            neg |= (uint8_t)(1u << i);
            c->mag[i] = 0u - (uint32_t)v;
        } else {
            c->mag[i] = (uint32_t)v;
        }
        c->err[i] = 0;
    }
    c->n_total = c->n_left = s->n;
    c->cur_tag = s->tag;
    c->cur_flags = s->flags;
    if (s->flags & SEGQ_FLAG_LAST)
        c->last_queued--;
    c->head++;
    if (c->head >= c->depth)
        c->head = 0;
    c->count--;
    out->dir_changed = (uint8_t)(c->dir_mask ^ neg);
    c->dir_mask = neg;
    out->seg_started = 1;
    out->seg_flags = c->cur_flags;
    out->seg_tag = c->cur_tag;
}

void segq_core_tick(struct segq_core *c, struct segq_tick *out)
{
    uint8_t i;
    out->step_mask = 0;
    out->dir_changed = 0;
    out->seg_started = 0;
    out->seg_flags = 0;
    out->seg_tag = 0;

    if (c->n_left == 0) {
        if (c->waiting) {
            if (c->count == 0 ||
                (c->count < c->start_threshold && c->last_queued == 0)) {
                out->dir_mask = c->dir_mask;
                return;
            }
            c->waiting = 0;
        } else if (c->count == 0) {
            // ran dry. Normal after a LAST segment, an underrun otherwise.
            if (!(c->cur_flags & SEGQ_FLAG_LAST) && c->underruns != 0xFFFF)
                c->underruns++;
            c->waiting = 1;
            out->dir_mask = c->dir_mask;
            return;
        }
        load_next(c, out);
    }

    for (i = 0; i < SEGQ_AXES; i++) {
        c->err[i] += c->mag[i];
        if (c->err[i] >= c->n_total) {
            c->err[i] -= c->n_total;
            out->step_mask |= (uint8_t)(1u << i);
            // wrap on overflow (int32 overflow would be undefined behaviour)
            if (c->dir_mask & (1u << i))
                c->pos[i] = (int32_t)((uint32_t)c->pos[i] - 1u);
            else
                c->pos[i] = (int32_t)((uint32_t)c->pos[i] + 1u);
        }
    }
    out->dir_mask = c->dir_mask;
    c->n_left--;
    if (c->n_left == 0 && (c->cur_flags & SEGQ_FLAG_LAST) && c->count == 0)
        c->waiting = 1;   // clean end of a burst
}

void segq_core_stop(struct segq_core *c)
{
    c->head = c->count = c->last_queued = 0;
    c->n_left = c->n_total = 0;
    c->cur_flags = 0;
    c->waiting = 1;
}
