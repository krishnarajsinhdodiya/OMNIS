/**
 * @file    imu_fusion.c
 * @brief   Dual-IMU fusion, disagreement fault, and side detection.
 *
 * Implementation of omnis-info.md §11c. Pure arithmetic; no allocation, no
 * statics, no peripheral access.
 */

#include <stddef.h>
#include <math.h>

#include "imu_fusion.h"

void imu_gravity_unit_vector(float roll, float pitch, float out[3])
{
    if (out == NULL) {
        return;
    }

    const float sr = sinf(roll),  cr = cosf(roll);
    const float sp = sinf(pitch), cp = cosf(pitch);

    /* Exact inverse of the measurement model:
     *   roll  = atan2(ay, az)              -> ay = sin(r)cos(p), az = cos(r)cos(p)
     *   pitch = atan2(-ax, hypot(ay,az))   -> ax = -sin(p)
     * Already unit length by construction, so no normalisation needed. */
    out[0] = -sp;
    out[1] =  sr * cp;
    out[2] =  cr * cp;
}

float imu_disagreement_rad(float roll_a, float pitch_a,
                           float roll_b, float pitch_b)
{
    float ua[3], ub[3];
    imu_gravity_unit_vector(roll_a, pitch_a, ua);
    imu_gravity_unit_vector(roll_b, pitch_b, ub);

    float dot = ua[0] * ub[0] + ua[1] * ub[1] + ua[2] * ub[2];

    /* Both are unit vectors, so |dot| <= 1 mathematically — but float rounding
     * can push it to 1.0000001 and acosf() then returns NaN, which would
     * propagate straight into the fault comparison and silently evaluate false.
     * A NaN that disables the IMU fault check is exactly the failure this
     * clamp exists to prevent. */
    if (dot >  1.0f) dot =  1.0f;
    if (dot < -1.0f) dot = -1.0f;

    return acosf(dot);
}

void imu_fusion_combine(const attitude_ekf_t *a,
                        const attitude_ekf_t *b,
                        float disagree_thresh_rad,
                        imu_fusion_result_t *out)
{
    if (out == NULL) {
        return;
    }

    /* Degrade gracefully if one filter is missing: a single-IMU estimate beats
     * no estimate, and the caller's fault path is what decides whether to keep
     * driving. */
    if (a == NULL && b == NULL) {
        out->roll = out->pitch = 0.0f;
        out->var_roll = out->var_pitch = 1.0f;
        out->disagreement = 0.0f;
        out->fault = true;
        return;
    }
    if (a == NULL || b == NULL) {
        const attitude_ekf_t *o = (a != NULL) ? a : b;
        out->roll         = attitude_ekf_roll(o);
        out->pitch        = attitude_ekf_pitch(o);
        out->var_roll     = attitude_ekf_var_roll(o);
        out->var_pitch    = attitude_ekf_var_pitch(o);
        out->disagreement = 0.0f;
        out->fault        = false;   /* nothing to disagree with */
        return;
    }

    const float ra = attitude_ekf_roll(a),  pa = attitude_ekf_pitch(a);
    const float rb = attitude_ekf_roll(b),  pb = attitude_ekf_pitch(b);

    /* --- Fault check first ------------------------------------------------
     * Computed before the fuse so the caller always has a disagreement number
     * even when it decides to throw the fused value away. */
    out->disagreement = imu_disagreement_rad(ra, pa, rb, pb);
    out->fault        = (out->disagreement > disagree_thresh_rad);

    /* --- Inverse-covariance weighting ------------------------------------
     * Guard the reciprocals: a variance of zero would mean "this estimate is
     * perfect", which no filter should ever claim, but a corrupted struct or a
     * covariance that has gone non-positive-definite through accumulated float
     * error could produce one. Falling back to a plain average is a safe,
     * obviously-wrong-in-the-right-direction answer. */
    const float va_r = attitude_ekf_var_roll(a),  vb_r = attitude_ekf_var_roll(b);
    const float va_p = attitude_ekf_var_pitch(a), vb_p = attitude_ekf_var_pitch(b);

    if (va_r > 0.0f && vb_r > 0.0f) {
        const float wa = 1.0f / va_r, wb = 1.0f / vb_r;
        const float wsum = wa + wb;
        /* Roll is wrapped-averaged relative to A: averaging +179 and -179
         * naively gives 0, which is 180 deg wrong. */
        out->roll     = ra + ekf_wrap_pi(rb - ra) * (wb / wsum);
        out->var_roll = 1.0f / wsum;
    } else {
        out->roll     = ra + ekf_wrap_pi(rb - ra) * 0.5f;
        out->var_roll = (va_r + vb_r) * 0.25f;
    }

    if (va_p > 0.0f && vb_p > 0.0f) {
        const float wa = 1.0f / va_p, wb = 1.0f / vb_p;
        const float wsum = wa + wb;
        out->pitch     = pa + ekf_wrap_pi(pb - pa) * (wb / wsum);
        out->var_pitch = 1.0f / wsum;
    } else {
        out->pitch     = pa + ekf_wrap_pi(pb - pa) * 0.5f;
        out->var_pitch = (va_p + vb_p) * 0.25f;
    }
}

imu_side_t imu_detect_side(float ax, float ay, float az,
                           float gyro_mag_radps)
{
    /* Rest gate. §11c: evaluate only at rest, never continuously. Both
     * conditions matter — a robot in steady free-fall has low gyro but no
     * usable gravity vector, and one rotating at constant |a| would otherwise
     * report a confident wrong answer. */
    if (gyro_mag_radps > IMU_REST_GYRO_MAX_RADPS) {
        return IMU_SIDE_UNKNOWN;
    }

    const float mag = sqrtf(ax * ax + ay * ay + az * az);
    if (fabsf(mag - 1.0f) > IMU_REST_ACCEL_TOL_G) {
        return IMU_SIDE_UNKNOWN;
    }

    const float fx = fabsf(ax), fy = fabsf(ay), fz = fabsf(az);

    /* Dominant axis wins. Require it to be clearly dominant — on a 45 deg
     * corner two axes read ~0.7 g each and there is no right answer, so
     * returning UNKNOWN is better than picking one and having the balance
     * controller drive the wrong wheel pair. 0.8 g corresponds to being within
     * ~37 deg of an axis. */
    const float DOMINANT_MIN_G = 0.80f;

    if (fz >= fx && fz >= fy && fz >= DOMINANT_MIN_G) {
        return (az > 0.0f) ? IMU_SIDE_Z_UP : IMU_SIDE_Z_DOWN;
    }
    if (fx >= fy && fx >= fz && fx >= DOMINANT_MIN_G) {
        return (ax > 0.0f) ? IMU_SIDE_X_UP : IMU_SIDE_X_DOWN;
    }
    if (fy >= fx && fy >= fz && fy >= DOMINANT_MIN_G) {
        return (ay > 0.0f) ? IMU_SIDE_Y_UP : IMU_SIDE_Y_DOWN;
    }

    return IMU_SIDE_UNKNOWN;
}

const char *imu_side_name(imu_side_t s)
{
    switch (s) {
        case IMU_SIDE_Z_UP:   return "Z_UP";
        case IMU_SIDE_Z_DOWN: return "Z_DOWN";
        case IMU_SIDE_X_UP:   return "X_UP";
        case IMU_SIDE_X_DOWN: return "X_DOWN";
        case IMU_SIDE_Y_UP:   return "Y_UP";
        case IMU_SIDE_Y_DOWN: return "Y_DOWN";
        case IMU_SIDE_UNKNOWN:
        default:              return "UNKNOWN";
    }
}

/* ------------------------------------------------------------------------
 * Sensor-frame -> body-frame axis remap.
 *
 * Signed permutation only: both MPU6050s lie flat on the same board, so every
 * physically realistic mounting differs by a multiple of 90 degrees. Doing this
 * as a swap-and-negate rather than a rotation matrix keeps it bit-exact and
 * costs three loads instead of nine multiply-accumulates.
 * ------------------------------------------------------------------------ */
void imu_apply_mount(const imu_mount_t *m, const float in[3], float out[3])
{
    if (m == NULL || in == NULL || out == NULL) {
        return;
    }

    for (int i = 0; i < 3; ++i) {
        const int   code = m->map[i];
        const int   idx  = (code < 0 ? -code : code) - 1;   /* 1-based -> 0-based */

        /* An out-of-range entry means a malformed descriptor. Emitting zero is
         * safer than reading past the array: a zeroed axis shows up immediately
         * as a stuck attitude, whereas an out-of-bounds read is undefined and
         * may look fine on the bench and fail in flight. */
        if (idx < 0 || idx > 2) {
            out[i] = 0.0f;
            continue;
        }

        out[i] = (code < 0) ? -in[idx] : in[idx];
    }
}

bool imu_mount_is_valid(const imu_mount_t *m)
{
    if (m == NULL) {
        return false;
    }

    bool seen[3] = { false, false, false };

    for (int i = 0; i < 3; ++i) {
        const int code = m->map[i];
        const int idx  = (code < 0 ? -code : code) - 1;

        if (idx < 0 || idx > 2) {
            return false;          /* out of range */
        }
        if (seen[idx]) {
            return false;          /* same sensor axis used twice */
        }
        seen[idx] = true;
    }
    return true;
}

bool imu_mount_is_right_handed(const imu_mount_t *m)
{
    if (!imu_mount_is_valid(m)) {
        return false;
    }

    /* Determinant of a signed permutation matrix = (sign product) x
     * (permutation parity). Computing it directly from the 3x3 is clearer than
     * reasoning about parity, and this runs once at boot. */
    float M[3][3] = { { 0 } };
    for (int i = 0; i < 3; ++i) {
        const int code = m->map[i];
        const int idx  = (code < 0 ? -code : code) - 1;
        M[i][idx] = (code < 0) ? -1.0f : 1.0f;
    }

    const float det =
          M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
        - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
        + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);

    return (det > 0.5f);           /* det is exactly +1 or -1 */
}
