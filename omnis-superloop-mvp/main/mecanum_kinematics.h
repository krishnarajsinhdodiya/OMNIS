/**
 * @file    mecanum_kinematics.h
 * @brief   Inverse/forward kinematics for the OMNIS 4-wheel mecanum drivetrain.
 *
 * LAYOUT: LATERAL PARALLEL ROLLERS, not the textbook X-drive.
 * Mounting-shaft and motor constraints put the SAME roller tilt on both left
 * wheels and the mirrored tilt on both right wheels:
 *
 *       FL /        \ FR          left pair  FL, RL  ->  delta = -1
 *       RL /        \ RR          right pair FR, RR  ->  delta = +1
 *
 * This is still fully holonomic (the 4x3 mixing matrix stays rank 3 for any
 * non-zero wheelbase), but it is NOT the standard matrix and the two axles are
 * no longer interchangeable — see the yaw-lever note below.
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

/* --- Roller handedness: the ONE place the wheel layout is stated ---------
 * delta_i is the sign that distinguishes the two mirrored roller tilts, as
 * defined in derivation §5: the roller axis of wheel i, projected into the
 * ground plane, is  a_i = (1, delta_i)/sqrt(2)  in body coordinates. So
 *
 *    delta = +1  ->  axis on the forward-LEFT  diagonal, drawn "\" from above
 *    delta = -1  ->  axis on the forward-RIGHT diagonal, drawn "/" from above
 *
 * (drawn with the robot facing up the page, body +y to the page-left).
 *
 * IF THE ROBOT STRAFES THE WRONG WAY, negate all four of these together and
 * rebuild. That is the complete fix — every vy sign and every yaw lever is
 * derived from them below, so nothing else needs touching, and nothing else
 * SHOULD be touched (derivation §5.3). Do not "fix" a strafe by flipping a
 * dir_invert flag: that would break forward motion and the FK round trip.
 *
 * Integers, not floats, so the layout assertions below are valid integer
 * constant expressions.
 * ------------------------------------------------------------------------ */
#define MECANUM_DELTA_FL   (-1)
#define MECANUM_DELTA_FR   (+1)
#define MECANUM_DELTA_RL   (-1)
#define MECANUM_DELTA_RR   (+1)

/** Shorthand for the left-pair handedness — "s" in the derivation. */
#define MECANUM_DELTA_LEFT  MECANUM_DELTA_FL

/* The closed-form pseudoinverse in mecanum_forward() is derived specifically
 * for the lateral-parallel family (both left wheels alike, both right wheels
 * mirrored). Reverting to an X-drive layout invalidates it, so fail loudly at
 * compile time rather than silently returning a wrong body velocity. */
_Static_assert(MECANUM_DELTA_FL == MECANUM_DELTA_RL,
               "left pair must share a roller tilt; see derivation §5.3");
_Static_assert(MECANUM_DELTA_FR == MECANUM_DELTA_RR,
               "right pair must share a roller tilt; see derivation §5.3");
_Static_assert(MECANUM_DELTA_FR == -MECANUM_DELTA_FL,
               "the two sides must be mirrored; see derivation §5.3");
_Static_assert(MECANUM_DELTA_FL == 1 || MECANUM_DELTA_FL == -1,
               "handedness is +1 or -1");

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

/** Per-wheel yaw lever arms, kappa_i = delta_i*x_i - y_i  [mm]. */
typedef struct {
    float fl;
    float fr;
    float rl;
    float rr;
} mecanum_levers_t;

/**
 * @brief The per-wheel yaw lever arms, in millimetres.
 *
 * These replace the single `k` of the X-drive layout, which no longer exists:
 * under parallel rollers the two axles get DIFFERENT lever arms.
 *
 *   front pair:  -/+ (wheelbase + track_width)/2     large
 *   rear  pair:  +/- (wheelbase - track_width)/2     small, and sign-fragile
 *
 * For the as-built OMNIS (223 x 230 mm) that is 226.5 mm against -3.5 mm: the
 * front axle does 98.5% of the yaw work and the rear axle's contribution
 * CHANGES SIGN if the frame is ever made longer than it is wide. Nothing in
 * the maths breaks when it does — the term simply passes through zero — but
 * do not build an intuition on the rear pair's direction during a spin.
 *
 * @param wheelbase_mm    front-to-back wheel-centre distance [mm]
 * @param track_width_mm  left-to-right wheel-centre distance [mm]
 */
mecanum_levers_t mecanum_yaw_levers(float wheelbase_mm, float track_width_mm);

/**
 * @brief Inverse kinematics: body velocity -> per-wheel step-pulse rates.
 *
 * Implements, per wheel:  r*omega_i = vx + delta_i*vy + w*(delta_i*x_i - y_i)
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
 * NOTE ON PRECISION. Under parallel rollers the vy and w columns of the mixing
 * matrix are no longer orthogonal (they meet at 44 deg for the as-built frame),
 * so this estimate is measurably noisier than the X-drive's was for the same
 * per-wheel error: about 1.4x on vy and 2.0x on w. Derivation §9.3 has the
 * numbers. It is exact for any command the IK produced; the cost is only in
 * how it amplifies wheel-speed ERROR.
 *
 * @param f                 signed step rates, one per wheel [microsteps/s]
 * @param wheel_radius_mm   wheel radius                [mm]
 * @param wheelbase_mm      front-to-back wheel-centre distance  [mm]
 * @param track_width_mm    left-to-right wheel-centre distance  [mm]
 * @return estimated body velocity; all zero if the geometry is degenerate
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
 * Unchanged by the move to parallel rollers: this direction annihilates all
 * three motion columns for EITHER handedness and for ANY geometry — one of the
 * few things the layout change left alone (derivation §9.2).
 *
 * @param f  step rates to test [microsteps/s]
 * @return   null-space component; expected ~0.0f
 */
float mecanum_null_space(const wheel_rates_t *f);

#ifdef __cplusplus
}
#endif

#endif /* MECANUM_KINEMATICS_H */
