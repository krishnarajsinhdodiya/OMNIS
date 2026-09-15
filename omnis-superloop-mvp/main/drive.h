/**
 * @file    drive.h
 * @brief   RC sticks -> body velocity -> per-wheel step rates.
 *
 * The layer between the radio and the kinematics, plus the one function that
 * runs the whole drive-command pipeline in the ORDER the kinematics reference
 * §6 requires. Getting that order wrong is the classic open-loop drift bug, so
 * it is encoded here once rather than trusted to each call site.
 *
 * PIPELINE, and why each step sits where it does:
 *
 *   sticks [-1,+1]
 *      |  drive_rc_to_body()     scale, and apply stick polarity HERE ONLY
 *      v
 *   body velocity (vx, vy, w)
 *      |  mecanum_inverse()      the verified mixing matrix, untouched
 *      v
 *   raw wheel rates
 *      |  mecanum_clamp()        COMMON factor, never per-wheel clipping
 *      v
 *   clamped rates ------------> null-space health check taken HERE
 *      |  mecanum_deadband()     per-wheel, kills direction chatter
 *      v
 *   final rates --------------> forward kinematics taken HERE
 *
 * WHY THE NULL-SPACE CHECK IS TAKEN BEFORE THE DEADBAND: the deadband is
 * deliberately a per-wheel operation, so it legitimately puts energy into the
 * null space. Measuring after it would make the health metric fire constantly
 * on healthy commands and be ignored — the exact fate of a noisy alarm.
 *
 * WHY FORWARD KINEMATICS IS TAKEN AFTER THE DEADBAND: FK is a commanded-velocity
 * estimate, and a deadbanded wheel genuinely is not going to turn. Feeding
 * pre-deadband rates to FK would have the estimator believe motion the hardware
 * was never asked to produce.
 *
 * NO RTOS, no allocation, no statics, no peripheral access. Host-testable.
 */

#ifndef DRIVE_H
#define DRIVE_H

#include <stdbool.h>

#include "mecanum_kinematics.h"
#include "omnis_params.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Normalised stick positions, each in [-1, +1]. */
typedef struct {
    float throttle;   /**< + = forward                */
    float pitch;      /**< + = forward (secondary vx) */
    float roll;       /**< + = left   (vy)            */
    float yaw;        /**< + = left / CCW (w)         */
} rc_sticks_t;

/** Everything the pipeline produced, including its own health metrics. */
typedef struct {
    body_vel_t    commanded;    /**< what the sticks asked for, pre-saturation */
    wheel_rates_t rates;        /**< final, post-clamp, post-deadband [usteps/s] */
    body_vel_t    achieved;     /**< FK of `rates` — what was actually commanded */
    float         null_space;   /**< health metric, post-clamp pre-deadband     */
    float         peak_rate;    /**< largest |wheel rate| before clamping       */
    bool          clamped;      /**< the common scale factor was applied        */
    uint8_t       deadbanded;   /**< how many wheels were held stopped (0..4)   */
} drive_solution_t;

/**
 * @brief Map normalised sticks to a body-frame velocity.
 *
 *   vx = throttle * VX_MAX  +  pitch * VX_SECONDARY
 *   vy = roll     * VY_MAX
 *   w  = yaw      * W_MAX
 *
 * Throttle and pitch both feed vx additively, so their sum can exceed VX_MAX.
 * Reference §5 says to handle that EITHER by clamping the combined vx here OR
 * by relying on the common-factor wheel clamp — "but not both, or the response
 * becomes non-linear near full stick."
 *
 * THIS BUILD RELIES ON THE WHEEL CLAMP and deliberately does not clamp vx.
 * Reason: clamping vx alone while leaving vy and w untouched would rotate the
 * commanded motion vector — a full-throttle diagonal would quietly become a
 * different diagonal. The common factor in mecanum_clamp() scales all three
 * components together, so the direction survives and only the magnitude drops.
 * That is exactly the distinction §6's first checklist rule is about.
 *
 * @param s      stick positions, each clamped internally to [-1, +1]
 * @param scale  full-deflection scaling and stick polarity
 */
body_vel_t drive_rc_to_body(const rc_sticks_t *s, const omnis_rc_scale_t *scale);

/**
 * @brief Run the full pipeline: body velocity -> ready-to-drive wheel rates.
 *
 * @param v         desired body velocity [mm/s, mm/s, rad/s]
 * @param geom      wheel radius and frame dimensions
 * @param limits    saturation and deadband
 * @param out       filled in; must not be NULL
 */
void drive_solve(const body_vel_t *v,
                 const omnis_geometry_t *geom,
                 const omnis_step_limits_t *limits,
                 drive_solution_t *out);

/** Convenience: sticks straight through to wheel rates. */
void drive_from_sticks(const rc_sticks_t *s,
                       const omnis_params_t *p,
                       drive_solution_t *out);

/**
 * @brief Limit how fast wheel rates change, by a COMMON factor.
 *
 * Moves `current` toward `target` so that no wheel's rate changes by more than
 * max_delta_steps this tick, scaling ALL FOUR changes by the same factor.
 *
 * Why common: slewing each wheel independently lets a wheel with a small change
 * arrive early and a wheel with a large change arrive late, and during that
 * transition the motion vector is warped exactly the way per-wheel clipping
 * warps it (kinematics reference §6) — a commanded strafe would briefly arc. With
 * a common factor every intermediate command lies on the straight line between
 * the old motion and the new, so the robot changes speed without changing
 * direction, and the null space stays empty.
 *
 * This is the §13c defence against lost steps: a stepper asked to accelerate
 * faster than its torque allows slips silently, and there are no encoders to
 * notice.
 *
 * @param current          last tick's commanded rates, updated in place
 * @param target           what the drive pipeline wants now
 * @param max_delta_steps  max acceleration [steps/s^2] x dt [s]. Non-positive
 *                         freezes `current`: a mis-set limit must not mean
 *                         "unlimited".
 * @return true if the limit was active this tick
 */
bool drive_slew_rates(wheel_rates_t *current, const wheel_rates_t *target,
                      float max_delta_steps);

/** All wheels stopped, all metrics zeroed. The failsafe's drive command. */
void drive_solution_zero(drive_solution_t *out);

#ifdef __cplusplus
}
#endif

#endif /* DRIVE_H */
