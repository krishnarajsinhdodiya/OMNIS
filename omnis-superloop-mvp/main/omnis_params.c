/**
 * @file    omnis_params.c
 * @brief   Compile-time parameter defaults for the OMNIS superloop MVP.
 *
 * Stands in for §9f's params.json, which needs the SD card this build excludes.
 */

#include <stddef.h>
#include <math.h>

#include "omnis_params.h"

static omnis_params_t s_params;

void omnis_params_defaults(omnis_params_t *p)
{
    if (p == NULL) {
        return;
    }

    p->schema_version = OMNIS_PARAMS_SCHEMA_VERSION;

    /* --- Geometry: as-built, measured, not negotiable ------------------- */
    p->geometry.wheel_radius_mm = 30.0f;
    p->geometry.wheelbase_mm    = 223.0f;
    p->geometry.track_width_mm  = 230.0f;

    /* --- Step limits ----------------------------------------------------
     * max_step_rate comes from the kinematics reference §7 Case D: FR at
     * 7016 microsteps/s is 2.19 rev/s ~ 131 RPM, described there as "near the
     * practical ceiling of this build" because a NEMA17 on 12 V is well off its
     * torque curve by then. 7000 is that ceiling, rounded down.
     *
     * deadband is the reference §6 figure: below ~20 steps/s a wheel dithers
     * across the direction reversal and chatters, because the A4988 needs DIR
     * stable >=200 ns before a STEP edge.
     *
     * max_accel is a placeholder pending bench measurement — it is the §13c
     * lost-step defence, and the correct value depends on the Vref current
     * limit that has not been set yet (§3f). Deliberately conservative.
     * ------------------------------------------------------------------- */
    p->step.max_step_rate      = 7000.0f;
    p->step.deadband_steps     = 20.0f;
    p->step.max_accel_steps_s2 = 20000.0f;

    /* --- RC scaling -----------------------------------------------------
     * Derived so that a full-deflection single-axis command lands inside
     * max_step_rate rather than being clamped:
     *
     *   pure vx:  f = vx * STEPS_PER_RAD / r = vx * 509.2958 / 30 = 16.98 * vx
     *             7000 / 16.98 = 412 mm/s, so 300 leaves ~27% headroom for
     *             combined commands.
     *   pure w:   f = k*w * 509.2958 / r = 7.55 * 509.2958 * w = 3845 * w
     *             7000 / 3845 = 1.82 rad/s, so 1.5 leaves headroom.
     *
     * Unverified on hardware — see PLAN.md open question 6.
     * ------------------------------------------------------------------- */
    p->rc_scale.vx_max_mmps       = 300.0f;
    p->rc_scale.vx_secondary_mmps = 150.0f;
    p->rc_scale.vy_max_mmps       = 300.0f;
    p->rc_scale.w_max_radps       = 1.5f;

    /* --- CRSF channel map: §9f rc.channel_map, 1-based ------------------ */
    p->channel_map.throttle    = 1;
    p->channel_map.pitch       = 2;
    p->channel_map.roll        = 3;
    p->channel_map.yaw         = 4;
    p->channel_map.kill_switch = 6;

    /* --- Balance ---------------------------------------------------------
     * ZERO GAINS ON PURPOSE. omnis-info.md §13d declines to guess PID gains
     * without mass, CG height and wheel radius, and that refusal is correct: a
     * zero-gain controller does nothing, which is visible and safe, whereas a
     * guessed-gain controller looks like it works and falls over. The tuning
     * procedure is in assets/control/pid-reference.md §5.
     * ------------------------------------------------------------------- */
    p->balance.kp             = 0.0f;
    p->balance.ki             = 0.0f;
    p->balance.kd             = 0.0f;
    p->balance.integral_max   = 0.0f;
    p->balance.trim_rad       = 0.0f;
    p->balance.lean_limit_rad = 0.10472f;   /* 6 degrees */
    p->balance.vel_bias_gain  = 0.0f;

    /* --- RC link loss (§7f) ---------------------------------------------
     * 250 ms is comfortably longer than the slowest ExpressLRS packet interval
     * (50 Hz = 20 ms), so a few dropped frames do not fire it, but short enough
     * that a runaway is caught fast. §7f is explicit that hold-last-value is the
     * wrong behaviour here.
     * ------------------------------------------------------------------- */
    p->rc_timeout_us = 250000u;
}

bool omnis_params_valid(const omnis_params_t *p)
{
    if (p == NULL) {
        return false;
    }
    if (p->schema_version != OMNIS_PARAMS_SCHEMA_VERSION) {
        return false;
    }

    /* Geometry divides in the kinematics — a zero here is a divide-by-zero
     * that would surface as inf step rates rather than an obvious fault. */
    if (!(p->geometry.wheel_radius_mm > 0.0f) ||
        !(p->geometry.wheelbase_mm    > 0.0f) ||
        !(p->geometry.track_width_mm  > 0.0f)) {
        return false;
    }
    if (!isfinite(p->geometry.wheel_radius_mm) ||
        !isfinite(p->geometry.wheelbase_mm) ||
        !isfinite(p->geometry.track_width_mm)) {
        return false;
    }

    if (!(p->step.max_step_rate > 0.0f)) {
        return false;
    }
    if (p->rc_timeout_us == 0u) {
        return false;
    }

    /* Channel numbers are 1-based into a 16-channel CRSF frame. */
    const uint8_t ch[] = { p->channel_map.throttle, p->channel_map.pitch,
                           p->channel_map.roll,     p->channel_map.yaw,
                           p->channel_map.kill_switch };
    for (size_t i = 0; i < sizeof(ch) / sizeof(ch[0]); ++i) {
        if (ch[i] < 1u || ch[i] > 16u) {
            return false;
        }
    }

    return true;
}

void omnis_params_init(void)
{
    omnis_params_defaults(&s_params);
}

const omnis_params_t *omnis_params(void)
{
    return &s_params;
}
