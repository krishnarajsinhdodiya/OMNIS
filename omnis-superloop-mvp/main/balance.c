/**
 * @file    balance.c
 * @brief   Two-wheel balance controller — implementation. See balance.h,
 *          which carries the wheel-sign derivation.
 */

#include <math.h>
#include <stddef.h>

#include "balance.h"

static float clampf(float v, float lo, float hi)
{
    if (!isfinite(v)) return 0.0f;
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void balance_init(balance_ctrl_t *b, const balance_cfg_t *cfg)
{
    pid_config_t pc;
    pid_config_defaults(&pc);
    pc.kp           = cfg->kp;
    pc.ki           = cfg->ki;
    pc.kd           = cfg->kd;
    pc.out_min      = -cfg->max_wheel_accel;
    pc.out_max      =  cfg->max_wheel_accel;
    pc.integral_max =  cfg->integral_max;
    pid_init(&b->pid, &pc);

    vel_bias_config_t vc;
    vel_bias_config_defaults(&vc);
    vc.gain           = cfg->vel_bias_gain;
    vc.lean_limit_rad = cfg->lean_limit_rad;
    vc.trim_rad       = 0.0f;   /* trim is added once, below, not twice */
    vel_bias_init(&b->vb, &vc);

    balance_reset(b);
}

void balance_reset(balance_ctrl_t *b)
{
    pid_reset(&b->pid);
    vel_bias_reset(&b->vb);
    b->fwd_speed     = 0.0f;
    b->outer_accum_s = 0.0f;
    b->outer_target  = 0.0f;
    b->last_target   = 0.0f;
    b->last_accel    = 0.0f;
    b->saturated     = false;
}

void balance_set_kp(balance_ctrl_t *b, float kp)
{
    b->pid.cfg.kp = isfinite(kp) && kp >= 0.0f ? kp : 0.0f;
}

float balance_pair_sign(omnis_balance_pair_t pair)
{
    return (pair == OMNIS_BALANCE_ON_FRONT_PAIR) ? 1.0f : -1.0f;
}

void balance_step(balance_ctrl_t *b, const balance_cfg_t *cfg,
                  omnis_balance_pair_t pair, float lean_rad, float lean_rate,
                  float stick_pitch, float stick_roll, float dt,
                  wheel_rates_t *out)
{
    out->fl = out->fr = out->rl = out->rr = 0.0f;
    if (!(dt > 0.0f) || !isfinite(lean_rad) || !isfinite(lean_rate)) {
        return;   /* nothing sensible to do; the supervisor's tilt/IMU faults act */
    }

    /* --- Outer loop, 20 Hz: lean back to bleed off sustained forward effort.
     *     gain 0 (the default) makes this a no-op. */
    b->outer_accum_s += dt;
    if (b->outer_accum_s >= cfg->outer_period_s) {
        b->outer_target  = vel_bias_update(&b->vb, b->fwd_speed, b->outer_accum_s);
        b->outer_accum_s = 0.0f;
    }

    /* --- Lean target: trim + outer loop + pitch stick ("controlled fall") --- */
    float target = cfg->trim_rad + b->outer_target
                 + clampf(stick_pitch, -1.0f, 1.0f) * cfg->stick_lean_max_rad;
    target = clampf(target, -cfg->lean_limit_rad, cfg->lean_limit_rad);
    b->last_target = target;

    /* --- Inner PID. Falling forward past the target gives u < 0. --- */
    float u = pid_update(&b->pid, target, lean_rad, lean_rate, dt);
    if (cfg->output_invert) {
        u = -u;
    }

    /* Catching a forward fall needs FORWARD acceleration, so the forward
     * acceleration command is -u. */
    const float fwd_accel = -u;
    b->last_accel = fwd_accel;

    b->fwd_speed += fwd_accel * dt;
    const float limited = clampf(b->fwd_speed, -cfg->max_wheel_rate, cfg->max_wheel_rate);
    b->saturated = (limited != b->fwd_speed);
    b->fwd_speed = limited;

    /* --- Turn: CCW means left wheel back, right wheel forward --- */
    const float turn      = clampf(stick_roll, -1.0f, 1.0f) * cfg->turn_max_steps;
    const float left_fwd  = b->fwd_speed - turn;
    const float right_fwd = b->fwd_speed + turn;

    /* --- Forward speeds -> wheel omega. The ONE place the pair sign enters. --- */
    const float sigma = balance_pair_sign(pair);
    if (pair == OMNIS_BALANCE_ON_FRONT_PAIR) {
        out->fl = sigma * left_fwd;     /* left  = FL */
        out->fr = sigma * right_fwd;    /* right = FR */
    } else {
        out->rr = sigma * left_fwd;     /* left  = RR (Y' flips on the rear pair) */
        out->rl = sigma * right_fwd;    /* right = RL */
    }
}
