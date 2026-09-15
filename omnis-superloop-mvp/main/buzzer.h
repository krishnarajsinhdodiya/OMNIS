/**
 * @file    buzzer.h
 * @brief   Buzzer GPIO glue over the pure pattern player (buzzer_pattern.c).
 *
 * Called once per tick. Writes GPIO16 only when the level changes, so it costs
 * nothing on the ticks that matter.
 */

#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

#include "buzzer_pattern.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Buzzer off and player reset. GPIO16 is configured by gpio_safe_state(). */
void buzzer_init(void);

/** Start an event pattern (one-shot, or BUZZ_CAL_HOLD_STILL until cancelled). */
void buzzer_event(buzz_pattern_t p, uint32_t now_ms);

/** Update the pin from events and active faults. Once per tick. */
void buzzer_service(uint32_t active_faults, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* BUZZER_H */
