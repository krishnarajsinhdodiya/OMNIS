/**
 * @file    mecanum_kinematics.h
 * @brief   Inverse/forward kinematics for the OMNIS 4-wheel mecanum drivetrain.
 *
 * Body frame (right-handed, ROS REP-103):
 *   +vx = forward          [mm/s]
 *   +vy = left  (strafe)   [mm/s]
 *   +w  = counter-clockwise viewed from above  [rad/s]
 *
 * Geometry is passed in as arguments (fields of params.json), never baked in,
 * so wheel size and frame dimensions can change without touching this code.
 *
 * UNITS: geometry is in millimetres, so vx/vy MUST be mm/s. Passing m/s makes
 * the yaw term wrong by 1000x while forward motion still looks correct.
 *
 * Derivation: assets/kinematics/mecanum-kinematics-derivation.md
 * Reference:  assets/kinematics/mecanum-kinematics-reference.md
 */

#ifndef MECANUM_KINEMATICS_H
#define MECANUM_KINEMATICS_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- Fixed drivetrain constants -----------------------------------------
 * Hard-wired properties of the hardware, not runtime parameters.
 * A4988 MS1/MS2/MS3 are tied HIGH -> fixed 1/16 microstepping.
 * ------------------------------------------------------------------------ */
#define MOTOR_FULL_STEPS_PER_REV   200.0f   /* 1.8 deg/step NEMA17-class    */
#define A4988_MICROSTEPS            16.0f   /* MS1/MS2/MS3 = HIGH           */
#define MICROSTEPS_PER_REV   (MOTOR_FULL_STEPS_PER_REV * A4988_MICROSTEPS)

/* rad/s -> microsteps/s. Exact given the two constants above: 3200/(2*pi). */
#define STEPS_PER_RAD        (MICROSTEPS_PER_REV / (2.0f * (float)M_PI))

/** Signed step-pulse rates, one per wheel. Sign -> DIR pin, |value| -> STEP. */
typedef struct {
    float fl;   /**< front-left   [microsteps/s] */
    float fr;   /**< front-right  [microsteps/s] */
    float rl;   /**< rear-left    [microsteps/s] */
    float rr;   /**< rear-right   [microsteps/s] */
} wheel_rates_t;

/** Body-frame velocity triple. */
typedef struct {
    float vx;   /**< +forward             [mm/s]  */
    float vy;   /**< +left                [mm/s]  */
    float w;    /**< +counter-clockwise   [rad/s] */
} body_vel_t;

/**
 * @brief Inverse kinematics: body velocity -> per-wheel step-pulse rates.
 *
 * Roller handedness: FL and RR share one tilt (negative on vy); FR and RL
 * share the mirrored tilt (positive on vy). Standard X-drive pairing.
 *
 * @param vx                forward velocity            [mm/s]
 * @param vy                left (strafe) velocity      [mm/s]
 * @param w                 counter-clockwise yaw rate  [rad/s]
 * @param wheel_radius_mm   wheel radius                [mm]
 * @param wheelbase_mm      front-to-back wheel-centre distance  [mm]
 * @param track_width_mm    left-to-right wheel-centre distance  [mm]
 * @return signed step rates, one per wheel [microsteps/s]
 */
wheel_rates_t mecanum_inverse(float vx, float vy, float w,
                              float wheel_radius_mm,
                              float wheelbase_mm,
                              float track_width_mm);

/**
 * @brief Forward kinematics: step-pulse rates -> body velocity.
 *
 * Open-loop, no encoders: this is a COMMANDED-velocity estimate, not a
 * measurement. Always pass POST-clamp rates, or dead reckoning will believe a
 * velocity the hardware never produced.
 *
 * @param f                 signed step rates, one per wheel [microsteps/s]
 * @param wheel_radius_mm   wheel radius                [mm]
 * @param wheelbase_mm      front-to-back wheel-centre distance  [mm]
 * @param track_width_mm    left-to-right wheel-centre distance  [mm]
 * @return estimated body velocity
 */
body_vel_t mecanum_forward(const wheel_rates_t *f,
                           float wheel_radius_mm,
                           float wheelbase_mm,
                           float track_width_mm);

/**
 * @brief Scale all four wheels by one common factor to respect a speed limit.
 *
 * Uses a common factor rather than per-wheel clipping: clipping wheels
 * independently warps the motion vector (a commanded strafe becomes an arc).
 * Scaling preserves direction and only reduces magnitude.
 *
 * @param r                  rates to scale, modified in place
 * @param max_steps_per_sec  per-wheel magnitude limit [microsteps/s]
 */
void mecanum_clamp(wheel_rates_t *r, float max_steps_per_sec);

/**
 * @brief Zero any wheel whose commanded rate is too small to drive cleanly.
 *
 * The A4988 needs DIR stable >=200 ns before a STEP edge. A wheel whose command
 * dithers around zero will chatter across the direction reversal, so hold it
 * stopped instead. Suggested threshold: ~20 steps/s.
 *
 * @param r                     rates to filter, modified in place
 * @param deadband_steps_per_sec  magnitude below which a wheel is held stopped
 */
void mecanum_deadband(wheel_rates_t *r, float deadband_steps_per_sec);

/**
 * @brief Slip / health metric: the wheel-speed combination that moves nothing.
 *
 * The 4x3 mixing matrix has a one-dimensional left null space, spanned by
 * (+FL +FR -RL -RR) — the front pair opposing the rear pair. Energy in this
 * combination produces no body motion and goes entirely into roller scrub.
 * mecanum_inverse() never generates a non-zero value, so a non-zero result
 * means something downstream corrupted the command (per-wheel clipping, a
 * wrong invert flag).
 *
 * @param f  step rates to test [microsteps/s]
 * @return   null-space component; expected ~0.0f
 */
float mecanum_null_space(const wheel_rates_t *f);

#ifdef __cplusplus
}
#endif

#endif /* MECANUM_KINEMATICS_H */
