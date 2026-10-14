// Edge-case checks for segq_core.c that the PC C++ tests do not hit; built with
// ASan+UBSan by run.sh so signed overflow is a hard failure.
// SPDX-License-Identifier: GPL-3.0-only
#include <stdint.h>
#include <stdio.h>
#include "../skipper/segq_core.h"

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

static struct segq_seg store[4];

int main(void) {
    struct segq_core c; struct segq_tick t; struct segq_seg s = {0};
    segq_core_init(&c, store, 4, 0);

    // position counters must wrap, not overflow (signed overflow is UB)
    c.pos[0] = INT32_MAX; c.pos[1] = INT32_MIN;
    s.n = 2; s.steps[0] = 1; s.steps[1] = -1; s.flags = SEGQ_FLAG_LAST;
    CHECK(segq_core_push(&c, &s) == SEGQ_OK);
    for (int i = 0; i < 4; i++) segq_core_tick(&c, &t);
    CHECK(c.pos[0] == INT32_MIN);       // wrapped
    CHECK(c.pos[1] == INT32_MAX);

    // extreme magnitudes are rejected, boundary accepted
    struct segq_seg b = {0};
    b.n = SEGQ_MAX_N; b.steps[0] = INT32_MIN;      CHECK(segq_core_push(&c, &b) == SEGQ_ERR_INVALID);
    b.steps[0] = INT32_MAX;                        CHECK(segq_core_push(&c, &b) == SEGQ_ERR_INVALID);
    b.steps[0] = (int32_t)(SEGQ_MAX_N / 2);        CHECK(segq_core_push(&c, &b) == SEGQ_OK);
    b.steps[0] = -(int32_t)(SEGQ_MAX_N / 2);       CHECK(segq_core_push(&c, &b) == SEGQ_OK);
    b.steps[0] = (int32_t)(SEGQ_MAX_N / 2) + 1;    CHECK(segq_core_push(&c, &b) == SEGQ_ERR_INVALID);
    b.n = SEGQ_MAX_N + 1; b.steps[0] = 0;          CHECK(segq_core_push(&c, &b) == SEGQ_ERR_INVALID);
    b.n = 3; b.steps[0] = 1;                       CHECK(segq_core_push(&c, &b) == SEGQ_OK);   // 2*1 <= 3
    b.n = 3; b.steps[0] = 2;                       CHECK(segq_core_push(&c, &b) == SEGQ_ERR_INVALID);
    b.n = 3; b.steps[0] = -2;                      CHECK(segq_core_push(&c, &b) == SEGQ_ERR_INVALID);

    // a maximum-length negative segment loads with the right magnitude/direction
    segq_core_stop(&c);
    b.n = SEGQ_MAX_N; b.steps[0] = -(int32_t)(SEGQ_MAX_N / 2); b.steps[1] = 0; b.flags = 0;
    CHECK(segq_core_push(&c, &b) == SEGQ_OK);
    segq_core_tick(&c, &t);
    CHECK(t.seg_started && (t.dir_mask & 1) && c.mag[0] == SEGQ_MAX_N / 2);

    printf(fails ? "segq_core_edge: %d FAILED\n" : "segq_core_edge: all passed\n", fails);
    return fails != 0;
}
