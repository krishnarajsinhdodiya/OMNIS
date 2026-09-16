/**
 * @file    mecanum_kinematics.c
 * @brief   Inverse/forward kinematics for the OMNIS 4-wheel mecanum drivetrain.
 *
 * LATERAL PARALLEL ROLLER LAYOUT — both left wheels share a roller tilt, both
 * right wheels share the mirror. See mecanum_kinematics.h for the handedness
 * constants, which are the only place the layout is stated.
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
 * Yaw lever arms
 *
 *   kappa_i = delta_i * x_i - y_i          [mm]           derivation (5.3)
 *
 * with x_i = +/- wheelbase/2 (front positive) and y_i = +/- track/2 (left
 * positive). Under parallel rollers the two axles land on DIFFERENT magnitudes:
 * the front pair on (L+W)/2 and the rear pair on (L-W)/2. That asymmetry is the
 * defining consequence of the layout — under an X-drive all four were (L+W)/2.
 * ------------------------------------------------------------------------ */
mecanum_levers_t mecanum_yaw_levers(float wheelbase_mm, float track_width_mm)
{
    const float half_l = 0.5f * wheelbase_mm;
    const float half_w = 0.5f * track_width_mm;

    mecanum_levers_t k;
    k.fl = (float)MECANUM_DELTA_FL * ( half_l) - ( half_w);
    k.fr = (float)MECANUM_DELTA_FR * ( half_l) - (-half_w);
    k.rl = (float)MECANUM_DELTA_RL * (-half_l) - ( half_w);
    k.rr = (float)MECANUM_DELTA_RR * (-half_l) - (-half_w);
    return k;
}

/* ------------------------------------------------------------------------
 * Inverse kinematics
 *
 *   r * omega_i = vx + delta_i*vy + w*kappa_i                derivation (5.3)
 *
 * With the as-built handedness (left pair -1, right pair +1) that expands to:
 *
 *   w_FL = ( vx - vy - k*w ) / r          k = (L + W)/2   front lever
 *   w_FR = ( vx + vy + k*w ) / r
 *   w_RL = ( vx - vy + m*w ) / r          m = (L - W)/2   rear lever
 *   w_RR = ( vx + vy - m*w ) / r
 *
 * Sign structure: strafe splits along SIDES (FL/RL vs FR/RR); yaw ALSO splits
 * along sides but with wildly unequal magnitudes (|k| vs |m|). The two motions
 * are no longer orthogonal wheel patterns — that is the price of the layout,
 * and it is why the FK below is not a simple transpose.
 * ------------------------------------------------------------------------ */
wheel_rates_t mecanum_inverse(float vx, float vy, float w,
                              float wheel_radius_mm,
                              float wheelbase_mm,
                              float track_width_mm)
{
    /* Per-wheel yaw lever arms. Folded to constants at -O2: the deltas are
     * compile-time, so each of these is one add of two halved parameters. */
    const mecanum_levers_t k = mecanum_yaw_levers(wheelbase_mm, track_width_mm);

    /* Yaw contribution as a linear velocity, so it adds directly to vx/vy.
     * [mm] * [rad/s] = [mm/s]. */
    const float inv_r = 1.0f / wheel_radius_mm;   /* one divide, not four */

    const float w_fl = (vx + (float)MECANUM_DELTA_FL * vy + k.fl * w) * inv_r;  /* [rad/s] */
    const float w_fr = (vx + (float)MECANUM_DELTA_FR * vy + k.fr * w) * inv_r;
    const float w_rl = (vx + (float)MECANUM_DELTA_RL * vy + k.rl * w) * inv_r;
    const float w_rr = (vx + (float)MECANUM_DELTA_RR * vy + k.rr * w) * inv_r;

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
 * Under parallel rollers M^T M is NO LONGER DIAGONAL: the vy and w columns
 * have dot product -2*s*W. Inverting the resulting 2x2 block gives a clean
 * closed form once it is written in terms of the two AXLE DIFFERENTIALS:
 *
 *   dF = w_FL - w_FR        front differential
 *   dR = w_RL - w_RR        rear  differential
 *
 *   vx = (r/4)        * ( w_FL + w_FR + w_RL + w_RR )
 *   vy = (r*s/(4*L))  * ( (L + s*W)*dF + (L - s*W)*dR )
 *   w  = (r*s/(2*L))  * ( dF - dR )
 *
 * with s = MECANUM_DELTA_LEFT. Read that aloud: YAW is the DIFFERENCE of the
 * two axle differentials, STRAFE is their weighted SUM. Under the old X-drive
 * it was the other way round, which is the single most likely thing to get
 * backwards when porting old code — the sign patterns look deceptively alike.
 *
 * The denominator is the wheelbase ALONE (det(M^T M) works out to 4*L^2), so
 * unlike the X-drive the track width does not appear in the yaw scale factor.
 * Verified against an exact rational pseudoinverse for both handedness signs
 * and four geometries; see derivation §8.
 * ------------------------------------------------------------------------ */
body_vel_t mecanum_forward(const wheel_rates_t *f,
                           float wheel_radius_mm,
                           float wheelbase_mm,
                           float track_width_mm)
{
    body_vel_t v = { 0.0f, 0.0f, 0.0f };

    /* The wheelbase is now a lone divisor rather than half of (L+W), so a zero
     * or negative value would produce infinities instead of a large number.
     * Fail stopped: a zero body-velocity estimate is safe, a NaN is not. */
    if (f == NULL || !(wheelbase_mm > 0.0f)) {
        return v;
    }

    /* microsteps/s -> rad/s: exact inverse of the IK conversion. */
    const float inv_spr = 1.0f / STEPS_PER_RAD;

    const float w_fl = f->fl * inv_spr;   /* [rad/s] */
    const float w_fr = f->fr * inv_spr;
    const float w_rl = f->rl * inv_spr;
    const float w_rr = f->rr * inv_spr;

    const float s  = (float)MECANUM_DELTA_LEFT;
    const float d_front = w_fl - w_fr;
    const float d_rear  = w_rl - w_rr;

    v.vx = 0.25f * wheel_radius_mm * (w_fl + w_fr + w_rl + w_rr);

    v.vy = (wheel_radius_mm * s / (4.0f * wheelbase_mm))
         * ((wheelbase_mm + s * track_width_mm) * d_front
          + (wheelbase_mm - s * track_width_mm) * d_rear);

    v.w  = (wheel_radius_mm * s / (2.0f * wheelbase_mm)) * (d_front - d_rear);

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
 *
 * Worth knowing under this layout: a pure spin puts the REAR wheels at only
 * |m/k| = 1.5% of the front wheels' rate, so on the as-built frame a slow yaw
 * command deadbands the rear pair long before the front. That is correct
 * behaviour, not a fault — the rear wheels genuinely have almost nothing to do
 * during a spin — but it does mean the null-space metric is taken BEFORE the
 * deadband (drive.h) or it would fire on every gentle turn.
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
 * Unchanged by the layout change: (+1,+1,-1,-1) annihilates all three motion
 * columns for either handedness and any geometry (derivation §9.2).
 * ------------------------------------------------------------------------ */
float mecanum_null_space(const wheel_rates_t *f)
{
    if (f == NULL) {
        return 0.0f;
    }
    return (f->fl + f->fr - f->rl - f->rr);
}
