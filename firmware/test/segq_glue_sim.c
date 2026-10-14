// Runs firmware/skipper/segq.c (the real Klipper glue) on a PC against a mock of
// the Klipper API, driving the DDA timer in simulated time. Checks pulse shape,
// direction pins, inversion, idle stop/restart, underruns, errors, positions.
// This proves the glue LOGIC only: no real timers, interrupts or pins are involved.
// SPDX-License-Identifier: GPL-3.0-only
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "klipper_mock.h"
#include "../skipper/segq_core.h"

void command_config_segq(uint32_t *args);
void command_config_segq_axis(uint32_t *args);
void command_segq_push(uint32_t *args);
void command_segq_stop(uint32_t *args);
void command_segq_query(uint32_t *args);
void segq_shutdown(void);

#define PERIOD 100
static uint32_t now;
static struct timer *armed;       // the one timer segq uses
static int timer_adds;
static struct { void *type; void *data; } oids[8];
static uint8_t pin_level[32];
static uint32_t edge_tick[32][4096]; static int edge_val[32][4096]; static int edge_n[32];
static char last_sendf[512], all_sendf[4096];
static jmp_buf jb; static const char *shut_msg;
static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

void sched_add_timer(struct timer *t) { armed = t; timer_adds++; }
struct gpio_out gpio_out_setup(uint32_t pin, uint8_t val) { pin_level[pin] = val; edge_n[pin] = 0; return (struct gpio_out){(uint8_t)pin}; }
void gpio_out_write(struct gpio_out g, uint8_t v) {
    pin_level[g.id] = v;
    int n = edge_n[g.id]++; edge_tick[g.id][n] = now; edge_val[g.id][n] = v;
}
static int irq_depth;
void irq_disable(void) { irq_depth++; }
void irq_enable(void) { irq_depth--; }
uint32_t timer_read_time(void) { return now; }
uint32_t timer_from_us(uint32_t us) { return us * 12; }
uint8_t timer_is_before(uint32_t a, uint32_t b) { return (int32_t)(a - b) < 0; }
void *alloc_chunk(size_t s) { return calloc(1, s); }
void *oid_alloc(uint8_t oid, void *type, uint16_t size) { oids[oid].type = type; oids[oid].data = calloc(1, size); return oids[oid].data; }
void *oid_lookup(uint8_t oid, void *type) { if (oids[oid].type != type) mock_shutdown("Invalid oid type"); return oids[oid].data; }
void *mock_oid_next(uint8_t *pos, void *type) { for (int i = (uint8_t)(*pos + 1); i < 8; i++) if (oids[i].type == type) { *pos = (uint8_t)i; return oids[i].data; } return NULL; }
void mock_shutdown(const char *msg) { shut_msg = msg; longjmp(jb, 1); }
void mock_sendf(const char *fmt, ...) {
    va_list ap; va_start(ap, fmt);
    char *o = last_sendf; o += sprintf(o, "%s:", fmt);
    for (const char *p = fmt; *p; p++) if (*p == '%') { p++; if (*p == 'h') p++; (void)0;
        if (*p == 'u' || *p == 'c') o += sprintf(o, " %u", va_arg(ap, unsigned)); else o += sprintf(o, " %d", va_arg(ap, int)); }
    strcat(all_sendf, last_sendf); strcat(all_sendf, "\n");
    va_end(ap);
}

// Run the timer for `ticks` DDA periods of simulated time.
static void run_for(uint32_t ticks) {
    uint32_t end = now + ticks * PERIOD;
    while (armed && !timer_is_before(end, armed->waketime)) {
        now = armed->waketime;
        struct timer *t = armed;
        if (t->func(t) == SF_DONE) armed = NULL;
    }
    now = end;
}
#define CMD(fn, ...) do { uint32_t a[] = {__VA_ARGS__}; if (setjmp(jb) == 0) fn(a); } while (0)
static int shut(void) { int s = shut_msg != NULL; return s; }

static uint32_t cfg_args_depth = 8;
static void setup(uint32_t thr, uint32_t depth, int inv_step, int inv_dir) {
    memset(oids, 0, sizeof oids); armed = NULL; timer_adds = 0; now = 1000; shut_msg = NULL; all_sendf[0] = 0;
    memset(pin_level, 0, sizeof pin_level);
    CMD(command_config_segq, 0, PERIOD, depth, thr);
    CMD(command_config_segq_axis, 0, 0, 10, 11, inv_step, inv_dir);   // X: step pin 10, dir pin 11
    CMD(command_config_segq_axis, 0, 3, 12, 13, 0, 0);                // E: step 12, dir 13
}
static void push(uint32_t n, int x, int e, uint32_t flags, uint32_t tag) {
    CMD(command_segq_push, 0, n, (uint32_t)x, 0, 0, (uint32_t)e, flags, tag);
}
static int count_rising(int pin, int active) { int c = 0; for (int i = 0; i < edge_n[pin]; i++) if (edge_val[pin][i] == active) c++; return c; }
static int pulses_ok(int pin, int active) {   // every active edge followed by inactive exactly one PERIOD later
    for (int i = 0; i < edge_n[pin]; i++) if (edge_val[pin][i] == active) {
        if (i + 1 >= edge_n[pin] || edge_val[pin][i + 1] == active) return 0;
        if (edge_tick[pin][i + 1] - edge_tick[pin][i] != PERIOD) return 0;
    }
    return 1;
}
static int32_t status_pos(int axis) { (void)axis; return 0; }

int main(void) {
    (void)status_pos; (void)cfg_args_depth;
    // 1. basic move, positive: 10 steps over 40 ticks on X, -5 on E over 20 ticks
    setup(1, 8, 0, 0);
    CHECK(pin_level[11] == 0 && pin_level[10] == 0);          // dir "positive", step idle
    push(40, 10, -5, SEGQ_FLAG_LAST, 7);
    CHECK(armed != NULL && timer_adds == 1);
    CHECK(armed->waketime > now);                             // timer armed in the future
    run_for(60);
    CHECK(count_rising(10, 1) == 10 && pulses_ok(10, 1));     // exactly 10 one-period pulses
    CHECK(count_rising(12, 1) == 5 && pulses_ok(12, 1));
    CHECK(pin_level[13] == 1);                                // E negative -> dir pin high
    CHECK(pin_level[11] == 0);
    CHECK(armed == NULL);                                     // timer stopped when idle
    CMD(command_segq_query, 0);
    CHECK(strstr(all_sendf, "segq_position oid=%c x=%i y=%i z=%i e=%i: 0 10 0 0 -5"));
    CHECK(strstr(all_sendf, "free=%c active=%c tag=%hu underruns=%hu: 0 8 0 7 0"));
    // 2. restart after idle, direction flips, tag changes
    push(20, -4, 0, SEGQ_FLAG_LAST, 9);
    CHECK(armed != NULL && timer_adds == 2);
    run_for(30);
    CHECK(pin_level[11] == 1 && pin_level[13] == 0);          // X now negative, E back to positive
    CMD(command_segq_query, 0);
    CHECK(strstr(all_sendf, "0 6 0 0 -5"));
    CHECK(armed == NULL && !shut());
    // 3. inversion: step active-low, dir inverted
    setup(1, 8, 1, 1);
    CHECK(pin_level[10] == 1 && pin_level[11] == 1);          // idle step = high; "positive" dir = high when inverted
    push(20, 3, 0, SEGQ_FLAG_LAST, 1);
    run_for(30);
    CHECK(count_rising(10, 0) == 3 && pulses_ok(10, 0) && pin_level[10] == 1);
    push(20, -3, 0, SEGQ_FLAG_LAST, 2);
    run_for(30);
    CHECK(pin_level[11] == 0);
    // 4. underrun: no LAST flag, queue runs dry -> counted exactly once, timer stops
    setup(1, 8, 0, 0);
    push(10, 2, 0, 0, 3);
    run_for(40);
    CHECK(armed == NULL);
    CMD(command_segq_query, 0);
    CHECK(strstr(all_sendf, "tag=%hu underruns=%hu: 0 8 0 3 1"));
    // 5. start_threshold holds back the start until enough segments are queued
    setup(3, 8, 0, 0);
    push(10, 2, 0, 0, 1); push(10, 2, 0, 0, 2);
    run_for(50);
    CHECK(count_rising(10, 1) == 0);
    push(10, 2, 0, 0, 3);
    run_for(60);
    CHECK(count_rising(10, 1) == 6);
    // 6. stop clears queue and lowers a high step pin on the next tick
    setup(1, 8, 0, 0);
    push(1000, 400, 0, 0, 1);
    run_for(12);
    CMD(command_segq_stop, 0);
    run_for(5);
    CHECK(pin_level[10] == 0 && armed == NULL);
    CMD(command_segq_query, 0);
    CHECK(strstr(all_sendf, "free=%c active=%c tag=%hu underruns=%hu: 0 8 0 1"));   // free 8, inactive
    // 7. errors -> shutdown with clear messages
    setup(1, 2, 0, 0);
    push(10, 6, 0, 0, 1);                                      // 2*6 > 10
    CHECK(shut() && strstr(shut_msg, "invalid segment"));
    setup(2, 2, 0, 0);
    push(100, 1, 0, 0, 1); push(100, 1, 0, 0, 2);
    CHECK(!shut());
    push(100, 1, 0, 0, 3);
    CHECK(shut() && strstr(shut_msg, "overflow"));
    setup(1, 8, 0, 0);
    setup(1, 8, 0, 0);
    shut_msg = NULL; { uint32_t a[] = {1, 0, 8, 1}; if (setjmp(jb) == 0) command_config_segq(a); }
    CHECK(shut() && strstr(shut_msg, "bad config"));
    shut_msg = NULL; { uint32_t a[] = {1, 100, 8, 9}; if (setjmp(jb) == 0) command_config_segq(a); }   // threshold > depth
    CHECK(shut());
    shut_msg = NULL; { uint32_t a[] = {1, 0x80000000u, 8, 1}; if (setjmp(jb) == 0) command_config_segq(a); }   // period >= 2^31 breaks wrapping time compares
    CHECK(shut() && strstr(shut_msg, "bad config"));
    shut_msg = NULL; { uint32_t a[] = {1, 0x40000000u, 8, 1}; if (setjmp(jb) == 0) command_config_segq(a); }   // 2^30 is the maximum
    CHECK(!shut());
    shut_msg = NULL; { uint32_t a[] = {0, 4, 1, 2, 0, 0}; if (setjmp(jb) == 0) command_config_segq_axis(a); }
    CHECK(shut() && strstr(shut_msg, "axis"));
    // 8. lateness: simulate the ISR running far behind -> shutdown message
    setup(1, 8, 0, 0);
    push(1000, 10, 0, 0, 1);
    now = armed->waketime + 5 * PERIOD;                         // the ISR is 5 periods late
    shut_msg = NULL;
    if (setjmp(jb) == 0) armed->func(armed);
    CHECK(shut() && strstr(shut_msg, "too slow"));
    // 9. emergency shutdown hook drops pins and queue
    setup(1, 8, 0, 0);
    push(1000, 400, 0, 0, 1);
    run_for(8);
    segq_shutdown();
    CHECK(pin_level[10] == 0);
    CHECK(irq_depth == 0);                                      // irq_disable/enable balanced (except shutdown paths)
    printf(fails ? "SEGQ GLUE SIM: %d FAILURES\n" : "SEGQ GLUE SIM: all checks passed\n", fails);
    return fails != 0;
}
