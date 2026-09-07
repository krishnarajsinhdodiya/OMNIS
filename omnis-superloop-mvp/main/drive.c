/**
 * @file    drive.c
 * @brief   RC -> body -> wheel pipeline. See drive.h for the ordering rationale.
 */

#include <stddef.h>
#include <math.h>

#include "drive.h"

static float clamp_unit(float v)
{
    if (v >  1.0f) return  1.0f;
    if (v < -1.0f) return -1.0f;
    /* NaN fails both comparisons and would propagate all the way to a step
     * rate. A dead stick reads as centred, which is the safe interpretation. */
    if (!isfinite(v)) return 0.0f;
    return v;
}

static float apply_polarity(float v, bool invert)
{
    return invert ? -v : v;
}

body_vel_t drive_rc_to_body(const rc_sticks_t *s, const omnis_rc_scale_t *scale)
{
    body_vel_t v = { 0.0f, 0.0f, 0.0f };

    if (s == NULL || scale == NULL) {
        return v;                      /* centred sticks: stopped */
    }

    const float throttle = apply_polarity(clamp_unit(s->throttle), scale->invert_throttle);
    const float pitch    = apply_polarity(clamp_unit(s->pitch),    scale->invert_pitch);
    const float roll     = apply_polarity(clamp_unit(s->roll),     scale->invert_roll);
    const float yaw      = apply_polarity(clamp_unit(s->yaw),      scale->invert_yaw);

    /* Throttle and pitch are additive into vx. Not clamped here — see drive.h. */
    v.vx = throttle * scale->vx_max_mmps + pitch * scale->vx_secondary_mmps;
    v.vy = roll     * scale->vy_max_mmps;
    v.w  = yaw      * scale->w_max_radps;

    return v;
}

void drive_solution_zero(drive_solution_t *out)
{
    if (out == NULL) {
        return;
    }
    out->commanded.vx = out->commanded.vy = out->commanded.w = 0.0f;
    out->achieved.vx  = out->achieved.vy  = out->achieved.w  = 0.0f;
    out->rates.fl = out->rates.fr = out->rates.rl = out->rates.rr = 0.0f;
    out->null_space = 0.0f;
    out->peak_rate  = 0.0f;
    out->clamped    = false;
    out->deadbanded = 0;
}

static float peak_of(const wheel_rates_t *r)
{
    float p = fabsf(r->fl);
    if (fabsf(r->fr) > p) p = fabsf(r->fr);
    if (fabsf(r->rl) > p) p = fabsf(r->rl);
    if (fabsf(r->rr) > p) p = fabsf(r->rr);
    return p;
}

static uint8_t count_zeroed(const wheel_rates_t *before, const wheel_rates_t *after)
{
    uint8_t n = 0;
    if (before->fl != 0.0f && after->fl == 0.0f) ++n;
    if (before->fr != 0.0f && after->fr == 0.0f) ++n;
    if (before->rl != 0.0f && after->rl == 0.0f) ++n;
    if (before->rr != 0.0f && after->rr == 0.0f) ++n;
    return n;
}

void drive_solve(const body_vel_t *v,
                 const omnis_geometry_t *geom,
                 const omnis_step_limits_t *limits,
                 drive_solution_t *out)
{
    if (out == NULL) {
        return;
    }
    if (v == NULL || geom == NULL || limits == NULL) {
        drive_solution_zero(out);
        return;
    }

    out->commanded = *v;

    /* 1. Inverse kinematics — the verified matrix, untouched. */
    wheel_rates_t r = mecanum_inverse(v->vx, v->vy, v->w,
                                      geom->wheel_radius_mm,
                                      geom->wheelbase_mm,
                                      geom->track_width_mm);

    out->peak_rate = peak_of(&r);
    out->clamped   = (out->peak_rate > limits->max_step_rate);

    /* 2. Saturate by COMMON scale factor. Per-wheel clipping would turn a
     *    commanded strafe into an arc; scaling keeps the direction. */
    mecanum_clamp(&r, limits->max_step_rate);

    /* 3. Health check, taken here: post-clamp, pre-deadband. mecanum_clamp
     *    preserves the null space (it scales all four wheels equally), so a
     *    non-zero value at this point means something genuinely corrupted the
     *    command. The deadband below would mask that by adding null-space
     *    energy of its own. */
    out->null_space = mecanum_null_space(&r);

    /* 4. Deadband. Below ~20 steps/s a wheel chatters across the direction
     *    reversal, because the A4988 needs DIR stable >=200 ns before a STEP
     *    edge. Hold it stopped instead. */
    const wheel_rates_t pre_deadband = r;
    mecanum_deadband(&r, limits->deadband_steps);
    out->deadbanded = count_zeroed(&pre_deadband, &r);

    out->rates = r;

    /* 5. Forward kinematics on the FINAL rates — what the hardware was actually
     *    asked to do, deadband included. */
    out->achieved = mecanum_forward(&r,
                                    geom->wheel_radius_mm,
                                    geom->wheelbase_mm,
                                    geom->track_width_mm);
}

void drive_from_sticks(const rc_sticks_t *s,
                       const omnis_params_t *p,
                       drive_solution_t *out)
{
    if (out == NULL) {
        return;
    }
    if (p == NULL) {
        drive_solution_zero(out);
        return;
    }

    const body_vel_t v = drive_rc_to_body(s, &p->rc_scale);
    drive_solve(&v, &p->geometry, &p->step, out);
}
