/**
 * @file    mecanum_kinematics.c
 * @brief   Inverse/forward kinematics for the OMNIS 4-wheel mecanum drivetrain.
 *
 * Fully symbolic in wheel_radius_mm, wheelbase_mm and track_width_mm — those
 * arrive as arguments from params.json. No numeric geometry constant appears
 * anywhere in this file, so resizing the frame or swapping wheels is a config
 * edit with no code change.
 *
 * Derivation: assets/kinematics/mecanum-kinematics-derivation.md
 * Reference:  assets/kinematics/mecanum-kinematics-reference.md
 */

#include <stddef.h>   /* NULL */

#include "mecanum_kinematics.h"

/* ------------------------------------------------------------------------
 * Inverse kinematics
 *
 *   k = (wheelbase_mm + track_width_mm) / 2          "yaw lever arm"
 *
 *   w_FL = ( vx - vy - k*w ) / wheel_radius_mm
 *   w_FR = ( vx + vy + k*w ) / wheel_radius_mm
 *   w_RL = ( vx + vy - k*w ) / wheel_radius_mm
 *   w_RR = ( vx - vy + k*w ) / wheel_radius_mm
 *
 * Sign structure: strafe splits along DIAGONALS (FL/RR vs FR/RL), rotation
 * splits along SIDES (FL/RL vs FR/RR).
 * ------------------------------------------------------------------------ */
wheel_rates_t mecanum_inverse(float vx, float vy, float w,
                              float wheel_radius_mm,
                              float wheelbase_mm,
                              float track_width_mm)
{
    /* Yaw lever arm: half the SUM of the two frame dimensions. Depends on the
     * sum only — neither dimension appears alone. */
    const float k = 0.5f * (wheelbase_mm + track_width_mm);

    /* Yaw contribution as a linear velocity, so it adds directly to vx/vy.
     * [mm] * [rad/s] = [mm/s]. */
    const float yaw_term = k * w;

    /* One divide instead of four. */
    const float inv_r = 1.0f / wheel_radius_mm;

    /* Roller handedness (delta) baked into the vy signs:
     *   FL, RR -> delta = -1  (roller axis on the forward-right diagonal)
     *   FR, RL -> delta = +1  (roller axis on the forward-left  diagonal)
     * The w signs follow from the same delta and the wheel positions — they
     * are NOT independent. Mirrored wheels flip both columns together. */
    const float w_fl = (vx - vy - yaw_term) * inv_r;   /* [rad/s] */
    const float w_fr = (vx + vy + yaw_term) * inv_r;
    const float w_rl = (vx + vy - yaw_term) * inv_r;
    const float w_rr = (vx - vy + yaw_term) * inv_r;

    /* rad/s -> microsteps/s. Sign is preserved and carries the DIR pin. */
    wheel_rates_t out;
    out.fl = w_fl * STEPS_PER_RAD;
    out.fr = w_fr * STEPS_PER_RAD;
    out.rl = w_rl * STEPS_PER_RAD;
    out.rr = w_rr * STEPS_PER_RAD;
    return out;
}

/* ------------------------------------------------------------------------
 * Forward kinematics (left pseudoinverse of the 4x3 mixing matrix)
 *
 *   vx = (r/4)             * (  w_FL + w_FR + w_RL + w_RR )
 *   vy = (r/4)             * ( -w_FL + w_FR + w_RL - w_RR )
 *   w  = (r / (2*(L + W))) * ( -w_FL + w_FR - w_RL + w_RR )
 *
 * The vy row carries the strafe sign pattern and the w row carries the
 * rotation pattern — the columns of the IK matrix, transposed. That symmetry
 * is a quick way to eyeball the implementation for sign errors.
 * ------------------------------------------------------------------------ */
body_vel_t mecanum_forward(const wheel_rates_t *f,
                           float wheel_radius_mm,
                           float wheelbase_mm,
                           float track_width_mm)
{
    /* microsteps/s -> rad/s: exact inverse of the IK conversion. */
    const float inv_spr = 1.0f / STEPS_PER_RAD;

    const float w_fl = f->fl * inv_spr;   /* [rad/s] */
    const float w_fr = f->fr * inv_spr;
    const float w_rl = f->rl * inv_spr;
    const float w_rr = f->rr * inv_spr;

    const float r_over_4 = wheel_radius_mm * 0.25f;

    body_vel_t v;
    v.vx = r_over_4 * ( w_fl + w_fr + w_rl + w_rr);
    v.vy = r_over_4 * (-w_fl + w_fr + w_rl - w_rr);
    v.w  = (wheel_radius_mm / (2.0f * (wheelbase_mm + track_width_mm)))
                   * (-w_fl + w_fr - w_rl + w_rr);
    return v;
}

/* ------------------------------------------------------------------------
 * Saturation by common scale factor.
 * Per-wheel clipping would warp the motion vector; scaling preserves it.
 * ------------------------------------------------------------------------ */
void mecanum_clamp(wheel_rates_t *r, float max_steps_per_sec)
{
    if (r == NULL || max_steps_per_sec <= 0.0f) {
        return;
    }

    float peak = fabsf(r->fl);
    if (fabsf(r->fr) > peak) peak = fabsf(r->fr);
    if (fabsf(r->rl) > peak) peak = fabsf(r->rl);
    if (fabsf(r->rr) > peak) peak = fabsf(r->rr);

    if (peak > max_steps_per_sec) {
        /* One factor for all four wheels: direction preserved, magnitude cut. */
        const float s = max_steps_per_sec / peak;
        r->fl *= s;
        r->fr *= s;
        r->rl *= s;
        r->rr *= s;
    }
}

/* ------------------------------------------------------------------------
 * Direction-reversal deadband.
 * A4988 needs DIR stable >=200 ns before a STEP edge; a wheel dithering around
 * zero would chatter across the reversal. Hold it stopped instead.
 * ------------------------------------------------------------------------ */
void mecanum_deadband(wheel_rates_t *r, float deadband_steps_per_sec)
{
    if (r == NULL || deadband_steps_per_sec <= 0.0f) {
        return;
    }

    if (fabsf(r->fl) < deadband_steps_per_sec) r->fl = 0.0f;
    if (fabsf(r->fr) < deadband_steps_per_sec) r->fr = 0.0f;
    if (fabsf(r->rl) < deadband_steps_per_sec) r->rl = 0.0f;
    if (fabsf(r->rr) < deadband_steps_per_sec) r->rr = 0.0f;
}

/* ------------------------------------------------------------------------
 * Null-space (slip) metric. Expected ~0 for any command from mecanum_inverse.
 * ------------------------------------------------------------------------ */
float mecanum_null_space(const wheel_rates_t *f)
{
    if (f == NULL) {
        return 0.0f;
    }
    return (f->fl + f->fr - f->rl - f->rr);
}
