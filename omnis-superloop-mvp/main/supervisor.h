/**
 * @file    supervisor.h
 * @brief   Arming state machine: when the motors may move, and in which mode.
 *
 * Pure: no IDF, no clock of its own (time comes in as an input). Host-tested in
 * test/test_supervisor.c. This is the single place that decides whether STEP
 * pulses may exist — which on board Rev 2.0 is the whole safety story, since
 * the drivers have no enable line.
 *
 *   DISARMED ──arm edge, all checks pass, flat──▶ ARMED_FLAT
 *      ▲  ▲                                           │
 *      │  └──arm edge, checks pass, balance──▶ ARMED_BALANCE_SETTLE
 *      │                                              │ settle_ms
 *      │                                              ▼
 *      └──────────── any disarm condition ◀── ARMED_BALANCE
 *
 * ARMING requires a LOW -> HIGH transition of the arm switch, and every one of:
 *   no active fault · radio link OK · IMUs calibrated and valid ·
 *   all four sticks centred · the robot's pose matches the requested mode
 *   (flat: not tipped; balance: tipped onto a pair) · balance gains set.
 *
 * The edge is CONSUMED by the attempt, pass or fail. A rejected arm needs the
 * switch cycled again, and a switch left HIGH at power-on can never arm the
 * robot. That is what stops a robot re-arming itself the moment a fault clears
 * or the link comes back with the switch still up.
 *
 * DISARMING happens on any of: arm switch LOW · any active fault · link lost ·
 * IMU not valid. Also latched when a disarm happens with the switch still HIGH,
 * the switch must go LOW before the next arm.
 *
 * MODE is latched at arming. Moving the mode switch while armed does nothing
 * until the next arm — changing between flat and balance mid-motion is a crash.
 * The AUTO position chooses from the pose: tipped past upright_min_rad means
 * balance on whichever pair is down, otherwise flat.
 *
 * BALANCE SETTLE: after arming balance, the step generator holds the wheels
 * still for settle_ms while the filters, freshly re-seeded in the balance frame,
 * converge (tau ~0.2 s). Only then does the balance loop drive.
 */

#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>

#include "omnis_imu_mounting.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SUP_DISARMED = 0,
    SUP_ARMED_FLAT,
    SUP_ARMED_BALANCE_SETTLE,
    SUP_ARMED_BALANCE,
} sup_state_t;

/** Mode-switch request; values match rc_switch_pos_t LOW / MID / HIGH. */
typedef enum {
    SUP_MODE_FLAT    = 0,
    SUP_MODE_BALANCE = 1,
    SUP_MODE_AUTO    = 2,
} sup_mode_req_t;

typedef enum {
    SUP_ARM_OK = 0,
    SUP_REJECT_FAULT,
    SUP_REJECT_NO_LINK,
    SUP_REJECT_IMU_NOT_READY,
    SUP_REJECT_STICKS_NOT_CENTRED,
    SUP_REJECT_NOT_UPRIGHT,         /**< balance requested, robot not tipped up    */
    SUP_REJECT_NOT_FLAT,            /**< flat requested, robot tipped              */
    SUP_REJECT_NO_BALANCE_GAINS,    /**< balance requested with zero gains (§13d)  */
} sup_arm_result_t;

typedef struct {
    uint32_t settle_ms;          /**< balance: hold still this long after arming   */
    float    upright_min_rad;    /**< |flat pitch| past this = standing on a pair  */
    float    flat_max_tilt_rad;  /**< |flat roll| and |pitch| under this to arm flat */
    bool     balance_gains_set;  /**< kp > 0                                        */
} sup_config_t;

typedef struct {
    uint32_t       now_ms;
    bool           link_ok;
    bool           arm_switch;
    bool           sticks_centered;
    sup_mode_req_t mode_request;
    bool           imu_ready;
    float          flat_roll_rad;    /**< flat-frame attitude, meaningful while disarmed */
    float          flat_pitch_rad;
    uint32_t       active_faults;
} sup_inputs_t;

typedef struct {
    sup_state_t          state;
    omnis_balance_pair_t pair;
    uint32_t             state_since_ms;
    bool                 switch_seen_low;
    uint32_t             arm_count;
    sup_arm_result_t     last_result;

    /* Events from the most recent update — each true for exactly one update. */
    bool                 ev_armed;
    bool                 ev_disarmed;
    bool                 ev_balance_engaged;
    bool                 ev_rejected;
} supervisor_t;

void supervisor_init(supervisor_t *s);

void supervisor_update(supervisor_t *s, const sup_config_t *cfg, const sup_inputs_t *in);

/** STEP pulses may be generated from the drive or balance command. */
bool supervisor_motors_live(const supervisor_t *s);

/** Any armed state, including the balance settle. */
bool supervisor_is_armed(const supervisor_t *s);

/** Attitude should be estimated in the balance frame. */
bool supervisor_wants_balance_frame(const supervisor_t *s);

const char *sup_state_name(sup_state_t st);
const char *sup_result_name(sup_arm_result_t r);

#ifdef __cplusplus
}
#endif

#endif /* SUPERVISOR_H */
