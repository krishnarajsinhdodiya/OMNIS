/**
 * @file    tick.h
 * @brief   500 Hz hardware-timer tick — the timing spine of the superloop.
 *
 * THE CENTRAL DESIGN RULE OF THIS BUILD:
 *
 *   The ISR sets a flag. The superloop does the work.
 *
 * The ISR does exactly two things — set `g_tick_500hz`, and count the case
 * where it was already set. Nothing else. No I2C (omnis-info.md is explicit
 * that I2C must never happen in an ISR), no floating point, no logging, no
 * peripheral access, no allocation, nothing that can block.
 *
 * NO RTOS PRIMITIVES. The flag is a plain `volatile bool`, not a semaphore or
 * a task notification. That is sufficient here and not merely expedient:
 * app_main is pinned to CPU1 (sdkconfig.defaults) and the GPTimer interrupt is
 * allocated on whichever core registers it — CPU1, because tick_init() runs
 * from app_main. Producer and consumer are therefore the same core, so there is
 * no cross-core visibility problem and no barrier is required. If app_main is
 * ever un-pinned, this reasoning breaks and the flag needs to become atomic.
 *
 * Rate: 500 Hz, matching omnis-info.md §11d's EKF/balance loop target.
 */

#ifndef TICK_H
#define TICK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TICK_RATE_HZ        500u
#define TICK_PERIOD_US      (1000000u / TICK_RATE_HZ)   /* 2000 us */
#define TICK_PERIOD_S       (1.0f / (float)TICK_RATE_HZ) /* 0.002 s */

/**
 * Set by the timer ISR, cleared by the superloop.
 *
 * `volatile` is load-bearing: without it the compiler is entitled to hoist the
 * read out of the `while (!g_tick_500hz)` spin and produce an infinite loop.
 */
extern volatile bool g_tick_500hz;

/**
 * Incremented by the ISR whenever it fires while the previous tick is still
 * pending — i.e. the superloop did not finish its work inside 2 ms.
 *
 * This is the single most important health metric in the build. A non-zero
 * value means the control loop is not actually running at 500 Hz, which
 * invalidates the EKF's dt and the PID's timing. Watch it.
 */
extern volatile uint32_t g_tick_overruns;

/** Total ticks generated since boot. */
extern volatile uint32_t g_tick_count;

/**
 * @brief Create and start the 500 Hz timer.
 *
 * Call from app_main, once, after GPIO is safe. The interrupt lands on the
 * calling core — see the note above about why that matters.
 *
 * @return true on success
 */
bool tick_init(void);

/**
 * @brief Block until the next tick, then consume it.
 *
 * A busy-wait, deliberately: this build has no RTOS primitive to block on and
 * CPU1 has nothing else to run (its idle task is un-watchdogged in
 * sdkconfig.defaults precisely because of this).
 *
 * @return microseconds spent waiting — headroom. Falling toward zero means the
 *         loop is running out of time before the overrun counter says so, which
 *         makes it a leading indicator rather than a lagging one.
 */
uint32_t tick_wait(void);

/** Zero the overrun and count statistics. Does not touch the pending flag. */
void tick_reset_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* TICK_H */
