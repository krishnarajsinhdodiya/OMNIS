/**
 * @file    fault.h
 * @brief   Fault latch: what is wrong, how long it stays wrong, and how it clears.
 *
 * Pure: no IDF, no clock. Host-tested in test/test_supervisor.c.
 *
 * THREE CLEARING CLASSES, chosen by what the fault implies:
 *
 *   FOREVER       Cleared only by a reboot. The robot's own sensing or setup
 *                 cannot be trusted: parameters, IMU bring-up, calibration, IMU
 *                 communication, IMU disagreement (omnis-info.md §11c: "a bad
 *                 mount, sensor, or cable, not something to average through"),
 *                 step-generator bring-up. Nothing the operator does from the
 *                 radio can make these safe again.
 *
 *   UNTIL DISARM  Tilt (fell over / flipped / picked up while armed) and loop
 *                 overrun. The hardware is fine; the situation was not. Cleared
 *                 when the robot is disarmed and the arm switch is LOW, so
 *                 re-arming is a deliberate act.
 *
 *   LIVE          RC link loss. Present while the link is down, gone when it
 *                 returns. It still forces a disarm, and re-arming still needs
 *                 the arm switch cycled — the supervisor enforces that, not this
 *                 latch.
 *
 * Any active fault, of any class, blocks arming and forces a disarm.
 */

#ifndef FAULT_H
#define FAULT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FAULT_PARAMS            (1u << 0)
#define FAULT_IMU_INIT          (1u << 1)
#define FAULT_IMU_CAL           (1u << 2)
#define FAULT_IMU_COMM          (1u << 3)
#define FAULT_IMU_DISAGREE      (1u << 4)
#define FAULT_STEP_INIT         (1u << 5)
#define FAULT_RC_LINK           (1u << 6)
#define FAULT_TILT              (1u << 7)
#define FAULT_LOOP_OVERRUN      (1u << 8)

#define FAULT_MASK_FOREVER      (FAULT_PARAMS | FAULT_IMU_INIT | FAULT_IMU_CAL | \
                                 FAULT_IMU_COMM | FAULT_IMU_DISAGREE | FAULT_STEP_INIT)
#define FAULT_MASK_UNTIL_DISARM (FAULT_TILT | FAULT_LOOP_OVERRUN)
#define FAULT_MASK_LIVE         (FAULT_RC_LINK)
#define FAULT_MASK_ALL          (FAULT_MASK_FOREVER | FAULT_MASK_UNTIL_DISARM | FAULT_MASK_LIVE)

typedef struct {
    uint32_t active;        /**< faults in force now                   */
    uint32_t ever;          /**< every fault seen since boot           */
    uint32_t raise_count;   /**< how many times a new fault appeared   */
} fault_state_t;

void fault_init(fault_state_t *f);

/** Latch fault bits. @return true if any bit was not already active. */
bool fault_raise(fault_state_t *f, uint32_t bits);

/** Set or clear a LIVE condition. Bits outside FAULT_MASK_LIVE are ignored. */
void fault_set_condition(fault_state_t *f, uint32_t bits, bool present);

/** Clear the UNTIL-DISARM class. Call only while disarmed with the switch LOW. */
void fault_clear_on_disarm(fault_state_t *f);

static inline bool fault_any(const fault_state_t *f) { return f->active != 0u; }

/** The single most severe active bit (0 if none). Order is in fault.c. */
uint32_t fault_most_severe(uint32_t active);

/** Name of one fault bit. */
const char *fault_name(uint32_t bit);

/** "IMU_COMM|RC_LINK", most severe first, or "none". @return length written. */
size_t fault_describe(uint32_t active, char *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* FAULT_H */
