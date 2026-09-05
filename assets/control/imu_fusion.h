/**
 * @file    imu_fusion.h
 * @brief   Dual-IMU fusion, disagreement fault, and down-side detection.
 *
 * Implements omnis-info.md §11c. Two MPU6050s at diagonally opposite corners
 * each run their own attitude_ekf_t; this layer combines them, cross-checks
 * them, and answers "which side is currently down".
 *
 * Three jobs, deliberately kept separate:
 *
 *   1. FUSE   — inverse-covariance-weighted average of the two estimates.
 *   2. FAULT  — angle between the two estimated gravity directions. Past a
 *               threshold (~15 deg) this is a bad mount, sensor or cable, and
 *               averaging through it is the wrong answer.
 *   3. SIDE   — which body axis is aligned with gravity, evaluated only at rest.
 *
 * WHY GRAVITY VECTORS AND NOT EULER ANGLES for the fault check: at pitch near
 * +-90 deg — which is exactly where balance mode lives — roll is gimbal-locked
 * and meaningless. A naive |roll_A - roll_B| > 15 deg test false-faults
 * constantly there. Verified case: two IMUs at (roll 30, pitch 88) and
 * (roll -30, pitch 92) differ by 60 deg of roll but only 3.46 deg of actual
 * gravity direction. The dot-product metric is singularity-free everywhere.
 *
 * UNITS: radians throughout. Accelerometer in g.
 *
 * NO RTOS: pure arithmetic on caller-owned structs.
 *
 * Reference: assets/control/sensor-fusion-reference.md
 */

#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include <stdbool.h>

#include "attitude_ekf.h"

#ifdef __cplusplus
extern "C" {
#endif

/** omnis-info.md §11c: disagree beyond ~15 deg and it is a fault, not noise. */
#define IMU_DISAGREE_THRESH_RAD   0.2617993878f   /* 15.0 deg */

/** Rest gate for side detection: below these, the platform is not manoeuvring. */
#define IMU_REST_GYRO_MAX_RADPS   0.0523598776f   /* 3 deg/s          */
#define IMU_REST_ACCEL_TOL_G      0.10f           /* | |a| - 1g | max */

/** Which body axis is pointing up. Maps to the grounded wheel pair (§1). */
typedef enum {
    IMU_SIDE_UNKNOWN = 0,  /**< moving, or ambiguous — do not act on this */
    IMU_SIDE_Z_UP,         /**< chassis flat, normal 4-wheel mode         */
    IMU_SIDE_Z_DOWN,       /**< inverted                                  */
    IMU_SIDE_X_UP,         /**< tipped onto one end                       */
    IMU_SIDE_X_DOWN,
    IMU_SIDE_Y_UP,         /**< tipped onto one side                      */
    IMU_SIDE_Y_DOWN,
} imu_side_t;

/* --- Sensor-frame to body-frame mounting ---------------------------------
 * The two MPU6050s sit at diagonally opposite corners of the chassis. There is
 * no guarantee their packages point the same way — on a hand-routed board, one
 * is very often rotated 180 deg about Z relative to the other because that is
 * what makes the I2C run reach.
 *
 * If that rotation is not undone before the EKF sees the data, the two filters
 * converge to attitudes that differ by the mounting rotation, and
 * imu_disagreement_rad() reports ~180 deg on two perfectly healthy sensors.
 * The 15 deg fault then fires at boot, every boot. This is the single most
 * likely reason a correct dual-IMU implementation refuses to arm.
 *
 * A remap, not a rotation matrix: both modules lie flat on the same board, so
 * every realistic mounting differs by a multiple of 90 deg. That is exactly a
 * signed axis permutation — a swap and a sign flip, no trig, no matrix, and
 * bit-exact rather than accumulating float error.
 *
 * ENCODING: map[i] selects which sensor axis feeds body axis i, 1-based so the
 * sign is usable. +1/-1 = sensor X, +2/-2 = sensor Y, +3/-3 = sensor Z.
 *
 *   body_x = sign(map[0]) * sensor[|map[0]| - 1]
 *
 * BODY FRAME (matches the kinematics convention exactly — see
 * mecanum-kinematics-reference.md §1): +X forward, +Y left, +Z up.
 *
 * The same remap applies to the accelerometer and the gyro. Both are vectors in
 * the sensor frame; a frame change does not care what they measure.
 * ------------------------------------------------------------------------ */
typedef struct {
    signed char map[3];   /**< see encoding above; e.g. {+1,+2,+3} = identity */
} imu_mount_t;

/** Sensor axes already match the body frame. */
#define IMU_MOUNT_IDENTITY    ((imu_mount_t){ { +1, +2, +3 } })
/** Rotated 90 deg CCW about Z (viewed from above). */
#define IMU_MOUNT_ROT_Z_90    ((imu_mount_t){ { +2, -1, +3 } })
/** Rotated 180 deg about Z — the common opposite-corner case. */
#define IMU_MOUNT_ROT_Z_180   ((imu_mount_t){ { -1, -2, +3 } })
/** Rotated 270 deg CCW about Z. */
#define IMU_MOUNT_ROT_Z_270   ((imu_mount_t){ { -2, +1, +3 } })
/** Mounted on the underside of the board (180 deg about X). */
#define IMU_MOUNT_FLIP_X      ((imu_mount_t){ { +1, -2, -3 } })
/** Underside AND rotated 180 about Z. */
#define IMU_MOUNT_FLIP_X_ROT_Z_180 ((imu_mount_t){ { -1, +2, -3 } })

/**
 * @brief Rotate a sensor-frame vector into the body frame.
 *
 * Apply to BOTH the accelerometer and the gyro sample, for each IMU, BEFORE
 * feeding attitude_ekf_step(). In-place is not supported; out must not alias in.
 *
 * @param m    mounting descriptor
 * @param in   sensor-frame vector [x,y,z]
 * @param out  body-frame vector, filled
 */
void imu_apply_mount(const imu_mount_t *m, const float in[3], float out[3]);

/**
 * @brief Sanity-check a mounting descriptor.
 *
 * Valid means: every entry is +-1..+-3, and all three reference distinct axes.
 * A typo like {+1,+1,+3} silently collapses two body axes onto one sensor axis
 * and produces an attitude that looks plausible but is wrong in a way that is
 * very hard to see on a plot.
 *
 * Does NOT check handedness — {+2,+1,+3} is a valid permutation but mirrors the
 * frame, turning a left-handed reading into a right-handed one. Use
 * imu_mount_is_right_handed() for that.
 */
bool imu_mount_is_valid(const imu_mount_t *m);

/**
 * @brief True if the mapping preserves handedness (determinant = +1).
 *
 * A mirrored frame (determinant -1) inverts the sign of every rotation, so the
 * gyro and the accel-derived angle disagree permanently and the EKF fights
 * itself. Worth asserting once at boot.
 */
bool imu_mount_is_right_handed(const imu_mount_t *m);

/** Result of combining the two per-IMU filters. */
typedef struct {
    float roll;             /**< fused roll  [rad] */
    float pitch;            /**< fused pitch [rad] */
    float var_roll;         /**< fused variance [rad^2] */
    float var_pitch;        /**< fused variance [rad^2] */
    float disagreement;     /**< angle between the two gravity estimates [rad] */
    bool  fault;            /**< disagreement exceeded the threshold */
} imu_fusion_result_t;

/**
 * @brief Unit vector pointing along gravity in the body frame, from Euler angles.
 *
 * Exact inverse of the atan2 measurement model in attitude_ekf.c:
 *
 *     u = ( -sin(pitch),
 *            sin(roll) * cos(pitch),
 *            cos(roll) * cos(pitch) )
 *
 * @param out  3-element array, filled with a unit vector
 */
void imu_gravity_unit_vector(float roll, float pitch, float out[3]);

/**
 * @brief Angle between two attitude estimates, measured as gravity directions.
 *
 * Singularity-free: valid at any attitude including pitch = +-90 deg. This is
 * the metric §11c's 15 deg threshold should be applied to.
 *
 * @return angle in [0, pi] [rad]
 */
float imu_disagreement_rad(float roll_a, float pitch_a,
                           float roll_b, float pitch_b);

/**
 * @brief Inverse-covariance-weighted fusion of two independent estimates.
 *
 *     x_f = (x_a/P_a + x_b/P_b) / (1/P_a + 1/P_b)
 *     P_f = 1 / (1/P_a + 1/P_b)
 *
 * This is the maximum-likelihood combination for two INDEPENDENT Gaussian
 * estimates. The two IMUs here are not truly independent — they observe the
 * same physical motion and share a vibration environment — so P_f is optimistic
 * (too small). That is acceptable for a fault-checked average whose covariance
 * is not fed back into anything, but do not treat P_f as a calibrated
 * uncertainty. See sensor-fusion-reference.md §4.
 *
 * If the disagreement exceeds disagree_thresh_rad, out->fault is set. The fused
 * values are still computed — the caller decides what to do, and having the
 * numbers available makes the fault diagnosable.
 *
 * @param a,b                  the two per-IMU filters
 * @param disagree_thresh_rad  fault threshold; IMU_DISAGREE_THRESH_RAD is §11c's
 * @param out                  result, must not be NULL
 */
void imu_fusion_combine(const attitude_ekf_t *a,
                        const attitude_ekf_t *b,
                        float disagree_thresh_rad,
                        imu_fusion_result_t *out);

/**
 * @brief Determine which body axis is up, from a single accel sample at rest.
 *
 * omnis-info.md §11c is explicit that this must be evaluated only at mode-entry
 * or after a detected flip, never continuously — a live side-detector fights
 * the balance controller mid-balance. This function enforces the rest gate and
 * returns IMU_SIDE_UNKNOWN whenever the platform is moving, so a caller that
 * ignores the advice still cannot get a bad answer while manoeuvring.
 *
 * @param ax,ay,az        accelerometer [g]
 * @param gyro_mag_radps  magnitude of the gyro vector [rad/s]
 * @return the up-axis, or IMU_SIDE_UNKNOWN if not confidently at rest
 */
imu_side_t imu_detect_side(float ax, float ay, float az,
                           float gyro_mag_radps);

/** Human-readable name, for logging and the buzzer/fault report. */
const char *imu_side_name(imu_side_t s);

#ifdef __cplusplus
}
#endif

#endif /* IMU_FUSION_H */
