/* Stand-in for the Pico SDK header, so that the sensor drivers compile on a PC against
 * the register-level device models in device_models.c. Time is simulated: it advances
 * only when the code under test sleeps, waits or talks on the bus. */
#ifndef FAKE_PICO_STDLIB_H
#define FAKE_PICO_STDLIB_H

#include <stdbool.h>
#include <stdint.h>

typedef uint64_t absolute_time_t;

extern uint64_t fake_time_us;

static inline uint64_t time_us_64(void) { return fake_time_us; }
static inline void sleep_ms(uint32_t ms) { fake_time_us += 1000ull * ms; }
static inline void busy_wait_us_32(uint32_t us) { fake_time_us += us; }
static inline absolute_time_t make_timeout_time_ms(uint32_t ms) { return fake_time_us + 1000ull * ms; }
static inline bool time_reached(absolute_time_t t) { return fake_time_us >= t; }

#endif
