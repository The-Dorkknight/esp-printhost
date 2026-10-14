// Skipper Flipper: Klipper glue for the segment-queue core (protocol v1).
//
// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// This file may be distributed under the terms of the GNU GPLv3 license.
// No warranty. Work in progress (0.1.0-wip).
//
// Written against the Klipper v0.13.0 MCU API (src/basecmd.h, src/sched.h,
// src/command.h, board/gpio.h, board/irq.h, board/misc.h).
//
// HONESTY NOTE: this file is only COMPILE-tested (for RP2040, STM32F103 and
// AVR targets). It has never run on a real board. Timing statements in the
// comments below are design assumptions, not measurements.
//
// How it works
//   * The host sends constant-velocity segments (segq_push). segq_core.c turns
//     them into one "which axes step now" answer per DDA tick.
//   * ONE Klipper timer fires every `period` MCU clock ticks while there is
//     work. It first clears the step pins raised on the previous tick, then
//     raises the pins for the axes stepping now. A step pulse is therefore
//     exactly one DDA period long; the core guarantees at least one idle tick
//     between two steps of the same axis (2*|steps| <= n).
//   * Direction pins are written on a segment's first tick, which the core
//     guarantees never steps, so the driver sees one full period of dir setup.
//   * The timer stops itself when the queue is idle and is restarted by the
//     next segq_push, so an idle board does not burn CPU.
//   * The core is NOT reentrant: every call from task context (push, stop,
//     query) is wrapped in irq_disable/irq_enable (irq_save/restore where the
//     caller's state is unknown); the timer ISR owns it otherwise.
//   * v1 has no sync pin actions (segq_pin_on_tag) and no endstop homing.
#include <stdint.h>
#include "autoconf.h"       // CONFIG_*
#include "basecmd.h"        // oid_alloc
#include "board/gpio.h"     // gpio_out_write
#include "board/irq.h"      // irq_disable
#include "board/misc.h"     // timer_read_time
#include "command.h"        // DECL_COMMAND
#include "sched.h"          // struct timer
#include "segq_core.h"

#define SEGQ_PROTOCOL_VERSION 1
#define SEGQ_MAX_DEPTH 255  // depth is sent as %c (one byte)
#define SEGQ_MAX_PERIOD (1UL << 30)  // Klipper's wrapping time compares need deltas < 2^31

DECL_CONSTANT("SEGQ_PROTOCOL", SEGQ_PROTOCOL_VERSION);
DECL_CONSTANT("SEGQ_MAX_DEPTH", SEGQ_MAX_DEPTH);
DECL_CONSTANT("SEGQ_AXES", SEGQ_AXES);

struct segq_axis {
    struct gpio_out step, dir;
    uint8_t configured;
    uint8_t step_on, step_off;   // pin levels for "pulse high" / "idle"
    uint8_t dir_inv;             // 1 = dir pin level is inverted
};

struct segq {
    struct timer timer;          // first member: container_of is trivial
    struct segq_core core;
    struct segq_axis ax[SEGQ_AXES];
    uint32_t period;             // DDA tick length in MCU clock ticks
    uint32_t start_lead;         // delay before the first tick after idle
    uint8_t pulse_mask;          // step pins currently raised
    uint8_t running;             // timer is armed
};

static uint_fast8_t segq_event(struct timer *t);

// Clear every raised step pin. Caller has interrupts disabled (ISR/shutdown).
static void
segq_clear_pulses(struct segq *s)
{
    uint8_t m = s->pulse_mask, a;
    for (a = 0; m; a++, m >>= 1)
        if (m & 1)
            gpio_out_write(s->ax[a].step, s->ax[a].step_off);
    s->pulse_mask = 0;
}

// Timer ISR: one DDA tick. Keep this small, it is the hot path (AVR!).
static uint_fast8_t
segq_event(struct timer *t)
{
    struct segq *s = container_of(t, struct segq, timer);
    struct segq_tick tk;
    uint8_t a, m;

    // 1. end the pulses started on the previous tick
    if (s->pulse_mask)
        segq_clear_pulses(s);

    // 2. advance the core
    segq_core_tick(&s->core, &tk);

    // 3. direction first (only on a segment's first tick, which never steps)
    if (tk.seg_started && tk.dir_changed) {
        m = tk.dir_changed;
        for (a = 0; m; a++, m >>= 1) {
            if ((m & 1) && s->ax[a].configured)
                gpio_out_write(s->ax[a].dir,
                               ((tk.dir_mask >> a) & 1) ^ s->ax[a].dir_inv);
        }
    }

    // 4. raise step pins. Axes that are not configured are never stepped
    // (their motion is dropped but still counted in the core's position).
    m = tk.step_mask;
    if (m) {
        for (a = 0; m; a++, m >>= 1) {
            if ((m & 1) && s->ax[a].configured) {
                gpio_out_write(s->ax[a].step, s->ax[a].step_on);
                s->pulse_mask |= (uint8_t)(1u << a);
            }
        }
    }

    // 5. go idle when the core has nothing left and no pulse is high.
    // core.waiting is set on a clean end (LAST flag) and one tick after the
    // queue ran dry mid-burst (that tick is what counts the underrun), so
    // testing it, rather than segq_core_active(), keeps underruns accurate.
    // If segments are queued but below start_threshold the timer keeps
    // ticking (cheap: the core returns at once) so the start is not missed.
    if (s->core.waiting && s->core.count == 0 && !s->pulse_mask) {
        s->running = 0;
        return SF_DONE;
    }

    s->timer.waketime += s->period;
    // Lateness: if the next tick is already in the past the ISR cannot keep
    // up with this period (or something blocked interrupts too long). Stepping
    // late would silently corrupt the move, so stop loudly. Klipper's port
    // code has a similar "Rescheduled timer in the past" check; this one
    // gives a clearer message.
    if (timer_is_before(s->timer.waketime, timer_read_time()))
        shutdown("segq DDA timer too slow: raise period / lower dda_hz");
    return SF_RESCHEDULE;
}

void
command_config_segq(uint32_t *args)
{
    uint16_t depth = args[2], thr = args[3];
    struct segq *s = oid_alloc(args[0], command_config_segq, sizeof(*s));
    if (!args[1] || args[1] > SEGQ_MAX_PERIOD || !depth || depth > SEGQ_MAX_DEPTH || thr > depth)
        shutdown("segq: bad config_segq arguments");
    s->timer.func = segq_event;
    s->period = args[1];
    // Wait at least one full period before the first tick after idle, so the
    // timer is safely in the future even with some interrupt latency. Not
    // tuned on hardware.
    s->start_lead = s->period + timer_from_us(20);
    segq_core_init(&s->core,
                   alloc_chunk(sizeof(struct segq_seg) * depth), depth, thr);
}
DECL_COMMAND(command_config_segq,
             "config_segq oid=%c period=%u depth=%c start_threshold=%c");

void
command_config_segq_axis(uint32_t *args)
{
    struct segq *s = oid_lookup(args[0], command_config_segq);
    uint8_t axis = args[1];
    if (axis >= SEGQ_AXES)
        shutdown("segq: axis out of range");
    struct segq_axis *ax = &s->ax[axis];
    ax->step_on = args[4] ? 0 : 1;
    ax->step_off = !ax->step_on;
    ax->dir_inv = args[5] ? 1 : 0;
    // Initial levels: step idle, direction "positive".
    ax->step = gpio_out_setup(args[2], ax->step_off);
    ax->dir = gpio_out_setup(args[3], ax->dir_inv);
    ax->configured = 1;
}
DECL_COMMAND(command_config_segq_axis,
             "config_segq_axis oid=%c axis=%c step_pin=%u dir_pin=%u"
             " invert_step=%c invert_dir=%c");

void
command_segq_push(uint32_t *args)
{
    struct segq *s = oid_lookup(args[0], command_config_segq);
    struct segq_seg seg;
    enum segq_result r;
    uint8_t a;
    seg.n = args[1];
    for (a = 0; a < SEGQ_AXES; a++)
        seg.steps[a] = (int32_t)args[2 + a];
    seg.flags = args[6];
    seg.tag = args[7];

    irq_disable();
    r = segq_core_push(&s->core, &seg);
    if (r == SEGQ_OK && !s->running) {
        // Idle -> work: arm the DDA timer. The ISR cannot be running now
        // (running==0 and interrupts are off), so the timer struct is ours.
        s->timer.waketime = timer_read_time() + s->start_lead;
        s->running = 1;
        sched_add_timer(&s->timer);
    }
    irq_enable();

    if (r == SEGQ_ERR_INVALID)
        shutdown("segq: invalid segment (n==0, too long, or steps > n/2)");
    if (r == SEGQ_ERR_FULL)
        shutdown("segq: queue overflow (host flow-control bug)");
}
DECL_COMMAND(command_segq_push,
             "segq_push oid=%c n=%u x=%i y=%i z=%i e=%i flags=%c tag=%hu");

void
command_segq_stop(uint32_t *args)
{
    struct segq *s = oid_lookup(args[0], command_config_segq);
    irq_disable();
    // Drop the queue now. If the timer is armed it stays armed for one more
    // tick, which also lowers any step pin that is currently high.
    segq_core_stop(&s->core);
    irq_enable();
}
DECL_COMMAND(command_segq_stop, "segq_stop oid=%c");

void
command_segq_query(uint32_t *args)
{
    uint8_t oid = args[0];
    struct segq *s = oid_lookup(oid, command_config_segq);
    int32_t pos[SEGQ_AXES];
    uint16_t tag, under, fr;
    uint8_t act, a;

    irq_disable();
    for (a = 0; a < SEGQ_AXES; a++)
        pos[a] = s->core.pos[a];
    tag = s->core.cur_tag;
    under = s->core.underruns;
    fr = segq_core_free(&s->core);
    act = segq_core_active(&s->core) || s->pulse_mask;
    irq_enable();

    sendf("segq_status oid=%c free=%c active=%c tag=%hu underruns=%hu",
          oid, (uint8_t)(fr > 255 ? 255 : fr), act, tag, under);
    sendf("segq_position oid=%c x=%i y=%i z=%i e=%i",
          oid, pos[0], pos[1], pos[2], pos[3]);
}
DECL_COMMAND(command_segq_query, "segq_query oid=%c");

// Emergency stop: runs with interrupts off. Pins go idle and the queue is
// dropped; Klipper's scheduler discards the timer itself.
void
segq_shutdown(void)
{
    uint8_t i;
    struct segq *s;
    foreach_oid(i, s, command_config_segq) {
        segq_core_stop(&s->core);
        segq_clear_pulses(s);
    }
}
DECL_SHUTDOWN(segq_shutdown);
