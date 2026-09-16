/**
 * @file    omnis_time.h
 * @brief   Time helpers for a build with no RTOS delay primitives.
 *
 * BUILD-LOG.md Planning 1a: no vTaskDelay anywhere in OMNIS-authored code. The superloop is
 * paced by the GPTimer tick; the only delays that exist are short boot-time
 * waits (sensor reset, filter settling), and those are busy-waits on the ROM's
 * microsecond delay.
 *
 * That is legal here because app_main is pinned to CPU1 with the CPU1 idle-task
 * watchdog disabled (sdkconfig.defaults). Never call omnis_delay_ms() from the
 * superloop body — it would stall the control loop for its whole duration.
 */

#ifndef OMNIS_TIME_H
#define OMNIS_TIME_H

#include <stdint.h>

#include "esp_rom_sys.h"
#include "esp_timer.h"

/** Microseconds since boot. Monotonic, 64-bit, safe from the superloop. */
static inline int64_t omnis_now_us(void)
{
    return esp_timer_get_time();
}

/** Milliseconds since boot, truncated to 32 bits (wraps after ~49 days). */
static inline uint32_t omnis_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/**
 * @brief Busy-wait. BOOT-TIME ONLY.
 *
 * Deliberately a loop of 1 ms ROM delays rather than one long one, so a
 * pathological value cannot overflow the ROM function's microsecond argument.
 */
static inline void omnis_delay_ms(uint32_t ms)
{
    while (ms-- > 0u) {
        esp_rom_delay_us(1000u);
    }
}

#endif /* OMNIS_TIME_H */
