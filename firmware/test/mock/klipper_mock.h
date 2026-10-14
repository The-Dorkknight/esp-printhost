// Minimal stand-in for the Klipper v0.13.0 MCU API used by firmware/skipper/segq.c,
// so the glue can run on a PC. Only what segq.c uses. NOT Klipper.
// SPDX-License-Identifier: GPL-3.0-only
#ifndef KLIPPER_MOCK_H
#define KLIPPER_MOCK_H
#include <setjmp.h>
#include <stddef.h>
#include <stdint.h>

#define container_of(ptr, type, member) ((type *)((char *)(ptr) - offsetof(type, member)))
#define DECL_COMMAND(FUNC, MSG)
#define DECL_CONSTANT(NAME, VALUE)
#define DECL_SHUTDOWN(FUNC)

struct timer { struct timer *next; uint_fast8_t (*func)(struct timer *); uint32_t waketime; };
enum { SF_DONE = 0, SF_RESCHEDULE = 1 };
struct gpio_out { uint8_t id; };

// mock control (implemented in segq_glue_sim.c)
void sched_add_timer(struct timer *t);
struct gpio_out gpio_out_setup(uint32_t pin, uint8_t val);
void gpio_out_write(struct gpio_out g, uint8_t val);
void irq_disable(void);
void irq_enable(void);
uint32_t timer_read_time(void);
uint32_t timer_from_us(uint32_t us);
uint8_t timer_is_before(uint32_t a, uint32_t b);
void *alloc_chunk(size_t size);
void *oid_alloc(uint8_t oid, void *type, uint16_t size);
void *oid_lookup(uint8_t oid, void *type);
void *mock_oid_next(uint8_t *pos, void *type);
void mock_shutdown(const char *msg) __attribute__((noreturn));
void mock_sendf(const char *fmt, ...);
#define shutdown(msg) mock_shutdown(msg)
#define sendf(fmt, ...) mock_sendf(fmt, __VA_ARGS__)
#define foreach_oid(pos, data, oidtype) for (pos = -1; (data = mock_oid_next(&pos, oidtype)); )
#endif
