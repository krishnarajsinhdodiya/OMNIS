/**
 * @file    attitude_ekf.c
 * @brief   Per-IMU attitude EKF — implementation.
 *
 * Structured as two independent 2-state filters (roll, pitch), which is the
 * exact block decomposition of the 4-state filter specified in
 * omnis-info.md §11b. See attitude-ekf-derivation.md §4 for why that is an
 * identity rather than an approximation.
 *
 * Every operation here is scalar float arithmetic on a caller-owned struct:
 * no allocation, no statics, no matrix inversion, no library calls beyond
 * <math.h>. Cost is roughly 40 flops + 2 atan2 per IMU per tick.
 */

#include <stddef.h>   /* NULL */

#include "attitude_ekf.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ------------------------------------------------------------------------
 * Angle wrapping.
 *
 * The innovation y = z - x is an angle difference. Without wrapping, a
 * measurement at +179 deg against a state at -179 deg produces an innovation
 * of 358 deg instead of 2 deg, and the filter slams the estimate across the
 * whole circle. atan2(sin d, cos d) is the branch-free way to do this and
 * costs less than the conditional-subtract loop it replaces.
 * ------------------------------------------------------------------------ */
float ekf_wrap_pi(float a)
{
    return atan2f(sinf(a), cosf(a));
}

void attitude_ekf_config_defaults(attitude_ekf_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    cfg->q_angle_psd     = EKF_Q_ANGLE_PSD_DEFAULT;
    cfg->q_bias_psd      = EKF_Q_BIAS_PSD_DEFAULT;
    cfg->r_accel         = EKF_R_ACCEL_DEFAULT;
    cfg->accel_dev_thresh = EKF_ACCEL_DEV_THRESH_G;
    cfg->accel_reject    = EKF_ACCEL_REJECT_THRESH_G;
    cfg->adapt_r_gain    = EKF_ADAPT_R_GAIN;
}

static void axis_init(ekf_axis_t *ax)
{
    ax->x[0] = 0.0f;
    ax->x[1] = 0.0f;
    /* Wide initial covariance: "the angle could be anything, the bias is
     * probably small". 1.0 rad^2 is ~57 deg of 1-sigma uncertainty, which lets
     * the first accel update dominate almost completely — exactly what we want
     * on a cold start. */
    ax->P[0][0] = 1.0f;   ax->P[0][1] = 0.0f;
    ax->P[1][0] = 0.0f;   ax->P[1][1] = 0.01f;
}

void attitude_ekf_init(attitude_ekf_t *ekf, const attitude_ekf_config_t *cfg)
{
    if (ekf == NULL) {
        return;
    }
    axis_init(&ekf->roll);
    axis_init(&ekf->pitch);

    if (cfg != NULL) {
        ekf->cfg = *cfg;
    } else {
        attitude_ekf_config_defaults(&ekf->cfg);
    }

    ekf->last_accel_mag_g     = 1.0f;
    ekf->last_r_used          = ekf->cfg.r_accel;
    ekf->last_update_rejected = false;
}

/* ------------------------------------------------------------------------
 * Accelerometer -> tilt angles (omnis-info.md §11b measurement model).
 *
 *   roll  = atan2( ay, az )
 *   pitch = atan2( -ax, sqrt(ay^2 + az^2) )
 *
 * Note the asymmetry: pitch is well-defined and continuous through +-90 deg
 * (at pitch = 90, ax = -1g and the hypot term goes to zero, which atan2 handles
 * exactly), but roll becomes undefined there because ay and az both vanish.
 * That is textbook Euler gimbal lock, and it matters for OMNIS specifically:
 * balance mode sits at pitch ~ +-90 deg, where the roll estimate is garbage.
 * The balance controller uses pitch as the lean angle and ignores roll, and the
 * fusion layer compares gravity VECTORS rather than Euler angles for exactly
 * this reason. See attitude-ekf-derivation.md §7.
 * ------------------------------------------------------------------------ */
static void accel_to_angles(float ax, float ay, float az,
                            float *roll, float *pitch)
{
    *roll  = atan2f(ay, az);
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
}

void attitude_ekf_seed_from_accel(attitude_ekf_t *ekf,
                                  float ax, float ay, float az)
{
    if (ekf == NULL) {
        return;
    }

    float roll, pitch;
    accel_to_angles(ax, ay, az, &roll, &pitch);

    ekf->roll.x[0]  = roll;
    ekf->pitch.x[0] = pitch;

    /* The angle is now known exactly as well as one accel sample knows it —
     * no better, no worse. Claiming less uncertainty than that would make the
     * filter ignore the next few measurements. */
    ekf->roll.P[0][0]  = ekf->cfg.r_accel;
    ekf->pitch.P[0][0] = ekf->cfg.r_accel;
    ekf->roll.P[0][1]  = ekf->roll.P[1][0]  = 0.0f;
    ekf->pitch.P[0][1] = ekf->pitch.P[1][0] = 0.0f;
}

void attitude_ekf_set_gyro_bias(attitude_ekf_t *ekf,
                                float bias_roll, float bias_pitch)
{
    if (ekf == NULL) {
        return;
    }
    ekf->roll.x[1]  = bias_roll;
    ekf->pitch.x[1] = bias_pitch;

    /* A measured bias deserves a tighter prior than the cold-start 0.01. This
     * is roughly (0.6 deg/s)^2 — about what a 5-second average leaves behind. */
    ekf->roll.P[1][1]  = 1.0e-4f;
    ekf->pitch.P[1][1] = 1.0e-4f;
}

/* ------------------------------------------------------------------------
 * Time update for one axis.
 *
 *   angle_k = angle_{k-1} + (rate - bias) * dt
 *   bias_k  = bias_{k-1}
 *
 *   F = [ 1  -dt ]        Q = [ q_angle*dt      0      ]
 *       [ 0   1  ]            [     0      q_bias*dt   ]
 *
 * P' = F P F^T + Q, expanded by hand:
 *
 *   P00' = P00 - dt*(P01 + P10) + dt^2*P11 + q_angle*dt
 *   P01' = P01 - dt*P11
 *   P10' = P10 - dt*P11
 *   P11' = P11 + q_bias*dt
 *
 * The dt^2*P11 term is what couples bias uncertainty into angle uncertainty:
 * the longer the bias is unknown, the faster the integrated angle decays in
 * confidence. Dropping it (as some published tilt filters do) makes the filter
 * over-confident and slowly stops it trusting the accelerometer at all.
 * ------------------------------------------------------------------------ */
static void axis_predict(ekf_axis_t *a, float rate, float dt,
                         float q_angle_psd, float q_bias_psd)
{
    a->x[0] += (rate - a->x[1]) * dt;

    const float p00 = a->P[0][0];
    const float p01 = a->P[0][1];
    const float p10 = a->P[1][0];
    const float p11 = a->P[1][1];

    a->P[0][0] = p00 - dt * (p01 + p10) + dt * dt * p11 + q_angle_psd * dt;
    a->P[0][1] = p01 - dt * p11;
    a->P[1][0] = p10 - dt * p11;
    a->P[1][1] = p11 + q_bias_psd * dt;
}

void attitude_ekf_predict(attitude_ekf_t *ekf, float gx, float gy, float dt)
{
    if (ekf == NULL || dt <= 0.0f) {
        return;
    }
    axis_predict(&ekf->roll,  gx, dt, ekf->cfg.q_angle_psd, ekf->cfg.q_bias_psd);
    axis_predict(&ekf->pitch, gy, dt, ekf->cfg.q_angle_psd, ekf->cfg.q_bias_psd);
}

/* ------------------------------------------------------------------------
 * Scalar measurement update for one axis. H = [1 0], so no matrix inverse:
 *
 *   S  = P00 + R
 *   K  = [ P00/S , P10/S ]
 *   x += K * wrap(z - angle)
 *   P -= K * H * P
 *
 * The second row of the covariance update must use the PRE-update P00 and P01,
 * hence the saved copies. Overwriting row 0 first and then reading it back is
 * the classic silent bug here — it makes the bias covariance shrink too fast
 * and the filter quietly stops adapting.
 * ------------------------------------------------------------------------ */
static void axis_update(ekf_axis_t *a, float z, float R)
{
    const float S = a->P[0][0] + R;
    if (S <= 0.0f) {
        return;                      /* cannot happen with R > 0; cheap guard */
    }

    const float k0 = a->P[0][0] / S;
    const float k1 = a->P[1][0] / S;

    const float y = ekf_wrap_pi(z - a->x[0]);

    a->x[0] += k0 * y;
    a->x[1] += k1 * y;

    const float p00 = a->P[0][0];
    const float p01 = a->P[0][1];

    a->P[0][0] -= k0 * p00;
    a->P[0][1] -= k0 * p01;
    a->P[1][0] -= k1 * p00;
    a->P[1][1] -= k1 * p01;
}

/* ------------------------------------------------------------------------
 * Adaptive measurement noise (omnis-info.md §11b).
 *
 * The accelerometer measures specific force, not gravity. Whenever the platform
 * accelerates linearly, the accel-derived angle is wrong by roughly
 * atan(a_linear / g) — and on a balancing robot the linear acceleration is
 * strongly correlated with the very lean angle we are trying to measure, so it
 * is a systematic error, not noise that averages out.
 *
 * The only cheap, reliable detector is |a| deviating from 1 g. Below the
 * threshold, trust the accel normally. Above it, inflate R quadratically:
 *
 *   R_eff = R * (1 + gain * (dev - thresh))^2
 *
 * Quadratic-in-the-excess (rather than a hard switch) keeps R continuous at the
 * threshold. The square also means R grows fast enough to matter: at 0.4 g of
 * deviation the accel is already trusted 9x less.
 *
 * @return R to use, or a negative value meaning "reject this measurement".
 * ------------------------------------------------------------------------ */
static float adaptive_r(const attitude_ekf_config_t *cfg, float accel_mag_g)
{
    const float dev = fabsf(accel_mag_g - 1.0f);

    if (dev >= cfg->accel_reject) {
        return -1.0f;                /* no usable attitude information */
    }
    if (dev <= cfg->accel_dev_thresh) {
        return cfg->r_accel;
    }

    const float excess = dev - cfg->accel_dev_thresh;
    const float factor = 1.0f + cfg->adapt_r_gain * excess;
    return cfg->r_accel * factor * factor;
}

bool attitude_ekf_update_accel(attitude_ekf_t *ekf,
                               float ax, float ay, float az)
{
    if (ekf == NULL) {
        return false;
    }

    const float mag = sqrtf(ax * ax + ay * ay + az * az);
    ekf->last_accel_mag_g = mag;

    /* A zero-magnitude reading means a dead or unplugged sensor, not free-fall
     * worth modelling. Reject rather than feed atan2(0,0) into the filter. */
    if (mag < 1.0e-3f) {
        ekf->last_update_rejected = true;
        return false;
    }

    const float R = adaptive_r(&ekf->cfg, mag);
    if (R < 0.0f) {
        ekf->last_update_rejected = true;
        ekf->last_r_used = 0.0f;
        return false;               /* gyro-only propagation this tick */
    }

    float roll_meas, pitch_meas;
    accel_to_angles(ax, ay, az, &roll_meas, &pitch_meas);

    /* Two sequential scalar updates. Exactly equivalent to one joint 2x2 update
     * because R is diagonal — sequential processing of independent measurements
     * is an identity, not an approximation. */
    axis_update(&ekf->roll,  roll_meas,  R);
    axis_update(&ekf->pitch, pitch_meas, R);

    ekf->last_r_used          = R;
    ekf->last_update_rejected = false;
    return true;
}

bool attitude_ekf_step(attitude_ekf_t *ekf,
                       float gx, float gy,
                       float ax, float ay, float az,
                       float dt)
{
    attitude_ekf_predict(ekf, gx, gy, dt);
    return attitude_ekf_update_accel(ekf, ax, ay, az);
}
