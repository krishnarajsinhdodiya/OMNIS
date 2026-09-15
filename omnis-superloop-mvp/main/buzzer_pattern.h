/**
 * @file    buzzer_pattern.h
 * @brief   Pure buzzer patterns and the priority rules that choose between them.
 *
 * No GPIO, no clock: given a time, return on or off. buzzer.c writes the pin.
 * Host-tested in test/test_supervisor.c.
 *
 * The buzzer module has its own driver IC and no pitch control (omnis_pins.h),
 * so every pattern is on/off timing in 50 ms slots. Each is chosen to be told
 * apart by ear without looking at the robot:
 *
 *   boot-ok          one short blip
 *   hold-still       a soft tick every second, while gyro calibration runs
 *   armed            two short
 *   disarmed         one long
 *   arm-rejected     three fast
 *   balance-engaged  short then long
 *
 *   fault-imu        CONTINUOUS — IMU disagreement, comm, init, calibration
 *   fault-rc-link    fast double chirp every 500 ms — radio link lost
 *   fault-tilt       slow half-second beeps — fell over / flipped while armed
 *   fault-config     long-short-short — parameters or step generator failed
 *   fault-overrun    triple chirp every second — control loop too slow
 *
 * PRIORITY: a one-shot event plays to completion over any fault pattern, so an
 * "arm rejected" is always heard; then the most severe active fault's pattern
 * resumes from its start. With no events and no faults, the buzzer is silent.
 */

#ifndef BUZZER_PATTERN_H
#define BUZZER_PATTERN_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BUZZ_SLOT_MS  50u

typedef enum {
    BUZZ_SILENT = 0,
    BUZZ_BOOT_OK,
    BUZZ_CAL_HOLD_STILL,
    BUZZ_ARMED,
    BUZZ_DISARMED,
    BUZZ_ARM_REJECTED,
    BUZZ_BALANCE_ENGAGED,
    BUZZ_FAULT_IMU,
    BUZZ_FAULT_RC_LINK,
    BUZZ_FAULT_TILT,
    BUZZ_FAULT_CONFIG,
    BUZZ_FAULT_OVERRUN,
    BUZZ_PATTERN_COUNT
} buzz_pattern_t;

/** Is the buzzer on, t ms after the pattern started? */
bool buzz_pattern_level(buzz_pattern_t p, uint32_t t_ms);

/** Has a one-shot pattern finished? Repeating patterns never finish. */
bool buzz_pattern_done(buzz_pattern_t p, uint32_t t_ms);

/** Length of one cycle [ms]. */
uint32_t buzz_pattern_length_ms(buzz_pattern_t p);

bool buzz_pattern_repeats(buzz_pattern_t p);

/** The repeating pattern for the most severe active fault, or BUZZ_SILENT. */
buzz_pattern_t buzz_pattern_for_faults(uint32_t active_faults);

const char *buzz_pattern_name(buzz_pattern_t p);

/** Event-over-fault arbitration. */
typedef struct {
    buzz_pattern_t event;
    uint32_t       event_start_ms;
    bool           event_active;
    buzz_pattern_t fault;
    uint32_t       fault_start_ms;
} buzz_player_t;

void buzz_player_init(buzz_player_t *pl);

/** Start a one-shot (or, for BUZZ_CAL_HOLD_STILL, a repeating) event.
 *  BUZZ_SILENT cancels any event in progress. */
void buzz_player_event(buzz_player_t *pl, buzz_pattern_t p, uint32_t now_ms);

/** Decide the pin level now. */
bool buzz_player_level(buzz_player_t *pl, uint32_t active_faults, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* BUZZER_PATTERN_H */
