/**
 * @file    supervisor.c
 * @brief   Arming state machine — implementation. See supervisor.h.
 */

#include <math.h>
#include <string.h>

#include "supervisor.h"

void supervisor_init(supervisor_t *s)
{
    memset(s, 0, sizeof *s);
    s->state           = SUP_DISARMED;
    s->pair            = OMNIS_BALANCE_ON_FRONT_PAIR;
    /* FALSE at boot: a switch left HIGH at power-on must be cycled before the
     * robot can arm. */
    s->switch_seen_low = false;
    s->last_result     = SUP_ARM_OK;
}

static void go(supervisor_t *s, sup_state_t st, uint32_t now_ms)
{
    s->state          = st;
    s->state_since_ms = now_ms;
}

static sup_arm_result_t evaluate_arm(supervisor_t *s, const sup_config_t *cfg,
                                     const sup_inputs_t *in, sup_state_t *target)
{
    /* Order matters only for what gets reported: the first failing check is the
     * one the operator is told about, so it runs from "hardware is broken" to
     * "you asked for the wrong thing". */
    if (in->active_faults != 0u) return SUP_REJECT_FAULT;
    if (!in->link_ok)            return SUP_REJECT_NO_LINK;
    if (!in->imu_ready)          return SUP_REJECT_IMU_NOT_READY;
    if (!in->sticks_centered)    return SUP_REJECT_STICKS_NOT_CENTRED;

    const bool tipped = fabsf(in->flat_pitch_rad) >= cfg->upright_min_rad;

    sup_mode_req_t mode = in->mode_request;
    if (mode == SUP_MODE_AUTO) {
        mode = tipped ? SUP_MODE_BALANCE : SUP_MODE_FLAT;
    }

    if (mode == SUP_MODE_BALANCE) {
        if (!tipped)                 return SUP_REJECT_NOT_UPRIGHT;
        if (!cfg->balance_gains_set) return SUP_REJECT_NO_BALANCE_GAINS;
        /* The flat-frame pitch folds near 90 deg but keeps its sign, which is
         * exactly what picking the pair needs (omnis_imu_mounting.h). */
        s->pair = omnis_balance_pair_from_pitch(in->flat_pitch_rad);
        *target = SUP_ARMED_BALANCE_SETTLE;
        return SUP_ARM_OK;
    }

    if (fabsf(in->flat_pitch_rad) > cfg->flat_max_tilt_rad ||
        fabsf(in->flat_roll_rad)  > cfg->flat_max_tilt_rad) {
        return SUP_REJECT_NOT_FLAT;
    }
    *target = SUP_ARMED_FLAT;
    return SUP_ARM_OK;
}

void supervisor_update(supervisor_t *s, const sup_config_t *cfg, const sup_inputs_t *in)
{
    s->ev_armed = s->ev_disarmed = s->ev_balance_engaged = s->ev_rejected = false;

    if (!in->arm_switch) {
        s->switch_seen_low = true;
    }

    if (s->state != SUP_DISARMED) {
        const bool must_disarm = !in->arm_switch || in->active_faults != 0u ||
                                 !in->link_ok || !in->imu_ready;
        if (must_disarm) {
            go(s, SUP_DISARMED, in->now_ms);
            s->ev_disarmed = true;
            /* Disarmed by a fault or link loss with the switch still HIGH: the
             * switch must go LOW before the next arm. */
            s->switch_seen_low = !in->arm_switch;
            return;
        }
        if (s->state == SUP_ARMED_BALANCE_SETTLE &&
            in->now_ms - s->state_since_ms >= cfg->settle_ms) {
            go(s, SUP_ARMED_BALANCE, in->now_ms);
            s->ev_balance_engaged = true;
        }
        return;
    }

    if (in->arm_switch && s->switch_seen_low) {
        s->switch_seen_low = false;   /* the edge is consumed, pass or fail */

        sup_state_t target = SUP_DISARMED;
        const sup_arm_result_t r = evaluate_arm(s, cfg, in, &target);
        s->last_result = r;
        if (r == SUP_ARM_OK) {
            go(s, target, in->now_ms);
            ++s->arm_count;
            s->ev_armed = true;
        } else {
            s->ev_rejected = true;
        }
    }
}

bool supervisor_motors_live(const supervisor_t *s)
{
    return s->state == SUP_ARMED_FLAT || s->state == SUP_ARMED_BALANCE;
}

bool supervisor_is_armed(const supervisor_t *s)
{
    return s->state != SUP_DISARMED;
}

bool supervisor_wants_balance_frame(const supervisor_t *s)
{
    return s->state == SUP_ARMED_BALANCE_SETTLE || s->state == SUP_ARMED_BALANCE;
}

const char *sup_state_name(sup_state_t st)
{
    switch (st) {
        case SUP_ARMED_FLAT:           return "ARMED-FLAT";
        case SUP_ARMED_BALANCE_SETTLE: return "ARMED-BAL-SETTLE";
        case SUP_ARMED_BALANCE:        return "ARMED-BALANCE";
        case SUP_DISARMED:
        default:                       return "DISARMED";
    }
}

const char *sup_result_name(sup_arm_result_t r)
{
    switch (r) {
        case SUP_ARM_OK:                    return "ok";
        case SUP_REJECT_FAULT:              return "a fault is active";
        case SUP_REJECT_NO_LINK:            return "no radio link";
        case SUP_REJECT_IMU_NOT_READY:      return "IMUs not ready";
        case SUP_REJECT_STICKS_NOT_CENTRED: return "sticks not centred";
        case SUP_REJECT_NOT_UPRIGHT:        return "balance requested but robot not tipped up";
        case SUP_REJECT_NOT_FLAT:           return "flat requested but robot is tipped";
        case SUP_REJECT_NO_BALANCE_GAINS:   return "balance gains are zero (see TESTING.md)";
        default:                            return "?";
    }
}
