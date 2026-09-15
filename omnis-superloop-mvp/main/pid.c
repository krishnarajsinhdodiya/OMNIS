/**
 * @file    pid.c
 * @brief   Discrete PID and outer velocity-bias loop — implementation.
 *
 * See pid.h for the architectural rationale and pid-reference.md for the
 * tuning procedure. Pure arithmetic; no allocation, no statics, no blocking.
 */

#include <stddef.h>
#include <math.h>

#include "pid.h"

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void pid_config_defaults(pid_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    /* Zero gains on purpose. omnis-info.md §13d: gains depend on mass, CG
     * height and wheel radius, which this code does not know. A zero-gain
     * controller does nothing, which is visible and safe; a guessed-gain
     * controller looks like it works and falls over. */
    cfg->kp = 0.0f;
    cfg->ki = 0.0f;
    cfg->kd = 0.0f;
    cfg->out_min      = -1.0f;
    cfg->out_max      =  1.0f;
    cfg->integral_max =  1.0f;
}

void pid_init(pid_ctrl_t *pid, const pid_config_t *cfg)
{
    if (pid == NULL) {
        return;
    }
    if (cfg != NULL) {
        pid->cfg = *cfg;
    } else {
        pid_config_defaults(&pid->cfg);
    }
    pid_reset(pid);
}

void pid_reset(pid_ctrl_t *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->integral   = 0.0f;
    pid->last_p     = 0.0f;
    pid->last_d     = 0.0f;
    pid->last_error = 0.0f;
    pid->saturated  = false;
}

/* ------------------------------------------------------------------------
 * One PID step.
 *
 * Order of operations matters here, and the reason is the anti-windup:
 *
 *   1. Compute a CANDIDATE integral, do not commit it yet.
 *   2. Form the tentative output from that candidate.
 *   3. If the output saturates AND the candidate moved the integral further in
 *      the saturating direction, throw the candidate away and keep the old
 *      integral. Otherwise commit.
 *   4. Recompute the output from the committed integral and clamp.
 *
 * Step 3 is "conditional integration". The alternative — clamping the integral
 * to a fixed range only — still lets it wind all the way to the clamp while the
 * actuator is pinned, and on a balancer that stored energy comes back out as an
 * overshoot at exactly the moment the robot is trying to recover. Freezing
 * instead means the integrator holds whatever value it had when the actuator
 * ran out of authority, and resumes the moment there is authority again.
 * ------------------------------------------------------------------------ */
float pid_update(pid_ctrl_t *pid, float setpoint, float measurement,
                 float measurement_rate, float dt)
{
    if (pid == NULL || dt <= 0.0f) {
        return 0.0f;
    }

    const float error = setpoint - measurement;

    const float p_term = pid->cfg.kp * error;

    /* Derivative on measurement, taken from the gyro rather than differenced.
     * d(error)/dt = d(setpoint)/dt - d(measurement)/dt, and the setpoint moves
     * slowly compared to the 500 Hz loop, so the first term is dropped. The
     * minus sign is that dropped derivation, not a sign error. */
    const float d_term = -pid->cfg.kd * measurement_rate;

    /* 1. candidate integral, hard-capped */
    float cand = pid->integral + pid->cfg.ki * error * dt;
    cand = clampf(cand, -pid->cfg.integral_max, pid->cfg.integral_max);

    /* 2. tentative output */
    const float tentative = p_term + cand + d_term;

    /* 3. commit unless it deepens saturation */
    bool blocked = false;
    if (tentative > pid->cfg.out_max) {
        blocked = (cand > pid->integral);      /* pushing further past the ceiling */
    } else if (tentative < pid->cfg.out_min) {
        blocked = (cand < pid->integral);      /* pushing further past the floor   */
    }
    if (!blocked) {
        pid->integral = cand;
    }

    /* 4. final output from the committed integral */
    const float raw = p_term + pid->integral + d_term;
    const float out = clampf(raw, pid->cfg.out_min, pid->cfg.out_max);

    pid->last_p     = p_term;
    pid->last_d     = d_term;
    pid->last_error = error;
    pid->saturated  = (raw != out);

    return out;
}

/* ======================================================================== */

void vel_bias_config_defaults(vel_bias_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    cfg->gain           = 0.0f;        /* must be tuned on hardware */
    cfg->lean_limit_rad = 0.10472f;    /* 6 deg — see pid-reference.md §4 */
    cfg->trim_rad       = 0.0f;
    cfg->leak           = 0.0f;        /* no leak by default */
    cfg->integral_max   = 1.0e7f;
}

void vel_bias_init(vel_bias_t *vb, const vel_bias_config_t *cfg)
{
    if (vb == NULL) {
        return;
    }
    if (cfg != NULL) {
        vb->cfg = *cfg;
    } else {
        vel_bias_config_defaults(&vb->cfg);
    }
    vb->integral = 0.0f;
}

void vel_bias_reset(vel_bias_t *vb)
{
    if (vb != NULL) {
        vb->integral = 0.0f;
    }
}

/* ------------------------------------------------------------------------
 * Outer loop.
 *
 * The integral of commanded step rate is a DISTANCE proxy, not a velocity one —
 * integrating steps/s over seconds gives steps. That is deliberate: what we
 * want to null out is accumulated displacement effort, so the robot returns
 * toward where it started rather than merely stopping wherever it drifted to.
 *
 * The optional leak makes it forget old effort exponentially. With leak = 0 the
 * loop is a pure integrator and will eventually saturate the lean clamp under
 * any sustained command, which is the correct behaviour for stick-driven
 * driving (the rider is asking for sustained motion). Enable a small leak only
 * if position-hold drift becomes the dominant complaint.
 * ------------------------------------------------------------------------ */
float vel_bias_update(vel_bias_t *vb, float commanded_step_rate, float dt)
{
    if (vb == NULL || dt <= 0.0f) {
        return 0.0f;
    }

    if (vb->cfg.leak > 0.0f) {
        vb->integral -= vb->integral * vb->cfg.leak * dt;
    }

    vb->integral += commanded_step_rate * dt;
    vb->integral = clampf(vb->integral,
                          -vb->cfg.integral_max, vb->cfg.integral_max);

    /* Negative sign: forward drive effort commands a BACKWARD lean. Leaning
     * back is how a balancing robot decelerates. Getting this sign wrong gives
     * a robot that accelerates until it falls over, and it is the single most
     * common bug in this loop. */
    const float target = -vb->cfg.gain * vb->integral + vb->cfg.trim_rad;

    return clampf(target, -vb->cfg.lean_limit_rad, vb->cfg.lean_limit_rad);
}
