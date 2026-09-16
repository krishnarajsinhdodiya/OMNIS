/**
 * @file    balance.h
 * @brief   Two-wheel balance controller: lean in, wheel rates out.
 *
 * Pure: no IDF, no clock. Host-tested in test/test_balance.c, including a
 * closed-loop inverted-pendulum simulation on both wheel pairs.
 *
 * Implements omnis-info.md §13b as scoped by PLAN.md decision 1c:
 *
 *   INNER (every tick)  pid_ctrl_t on the balance-frame lean angle, derivative
 *                       taken from the raw gyro. Output = commanded wheel
 *                       ACCELERATION, integrated into a forward speed.
 *   OUTER (20 Hz)       vel_bias_t, wired but OFF by default (gain 0) per
 *                       decision 1c. Enable by setting params.balance.vel_bias_gain.
 *
 * INPUTS ARE IN THE BALANCE FRAME (omnis_imu_mounting.h): +X' is the chassis top
 * face, +Z' up. Positive lean means the upper end is falling toward the top face
 * — "falling forward" — on EITHER pair. That frame exists because the flat-frame
 * pitch folds at 90 deg (assets/control/attitude-ekf-derivation.md §7).
 *
 * WHEEL SIGN — DIFFERS BETWEEN PAIRS. The kinematics' +omega is the wheel sense
 * that drives the robot forward when flat, i.e. positive rotation about body +Y.
 * For a wheel rolling without slip, the centre moves opposite to the tangential
 * velocity of the contact point, omega x r, with r pointing from the axle to the
 * ground:
 *
 *   flat          ground at body -Z   omega x r = (-wR, 0, 0)  -> centre +X      (forward, by definition)
 *   front pair    ground at body +X   omega x r = (0, 0, -wR)  -> centre +Z = +X' (toward the top face)
 *   rear pair     ground at body -X   omega x r = (0, 0, +wR)  -> centre -Z = -X' (away from the top face)
 *
 * So +omega moves the robot FORWARD (toward the top face) on the front pair and
 * BACKWARD on the rear pair: sigma = +1 front, -1 rear. That one factor is the
 * whole difference between the two poses, and it is applied in exactly one
 * place. If the bench check in TESTING.md shows the wheels chasing the fall the
 * wrong way on both pairs, params.balance.output_invert flips the controller;
 * if only one pair is wrong, this derivation is wrong — do not paper over that
 * with the flag.
 *
 * LEFT/RIGHT. With X' on the top face and Z' up, the robot's left is +Y'. On the
 * front pair Y' = +Y, so left = FL, right = FR. On the rear pair Y' = -Y, so
 * left = RR, right = RL.
 *
 * The airborne pair is commanded zero: it stays energised (EN# is hardwired) and
 * holds, ready if the robot is flipped onto it — omnis-info.md §1.
 *
 * KNOWN LIMITATION: two grounded mecanum wheels driven differentially produce yaw
 * AND some lateral force (the rollers are at 45 deg). Turning while balancing will
 * crab sideways a little. Acceptable for the MVP; noted in BUILD-LOG.
 */

#ifndef BALANCE_H
#define BALANCE_H

#include <stdbool.h>

#include "mecanum_kinematics.h"
#include "omnis_imu_mounting.h"
#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;                  /**< (steps/s^2) per rad of lean error        */
    float ki;                  /**< (steps/s^2) per (rad s)                  */
    float kd;                  /**< (steps/s^2) per (rad/s) of lean rate     */
    float integral_max;        /**< |I term| cap [steps/s^2]                 */
    float max_wheel_accel;     /**< PID output clamp [steps/s^2]             */
    float max_wheel_rate;      /**< forward-speed clamp [steps/s]            */
    float trim_rad;            /**< static lean offset: §7d's trim pot       */
    float stick_lean_max_rad;  /**< full pitch stick -> this much lean       */
    float turn_max_steps;      /**< full roll stick -> this differential     */
    float lean_limit_rad;      /**< clamp on the total lean target           */
    float vel_bias_gain;       /**< outer loop, 0 = off (decision 1c)        */
    float outer_period_s;      /**< outer loop period, 0.05 s = 20 Hz        */
    bool  output_invert;       /**< flips the controller; see the header     */
} balance_cfg_t;

typedef struct {
    pid_ctrl_t pid;
    vel_bias_t vb;
    float      fwd_speed;       /**< integrated forward speed [steps/s], + = toward the top face */
    float      outer_accum_s;
    float      outer_target;    /**< lean target from the outer loop [rad]     */
    float      last_target;     /**< total lean target this tick [rad]         */
    float      last_accel;      /**< forward acceleration command [steps/s^2]  */
    bool       saturated;       /**< forward speed hit max_wheel_rate          */
} balance_ctrl_t;

/** Build the PID and outer loop from cfg and clear all state. */
void balance_init(balance_ctrl_t *b, const balance_cfg_t *cfg);

/** Clear integrators and speed, keep gains. Call on every entry to balance mode. */
void balance_reset(balance_ctrl_t *b);

/**
 * @brief Change the proportional gain WITHOUT resetting state.
 *
 * For live tuning from the S1 slider (OMNIS_TUNE_KP_FROM_POT). balance_init()
 * would clear the integrators and forward speed mid-balance, which is a fall.
 */
void balance_set_kp(balance_ctrl_t *b, float kp);

/** +1 if +omega moves the robot toward the top face on this pair, -1 if away. */
float balance_pair_sign(omnis_balance_pair_t pair);

/**
 * @brief One control step.
 *
 * @param lean_rad     balance-frame pitch [rad], + = falling forward
 * @param lean_rate    balance-frame gyro_y' [rad/s]
 * @param stick_pitch  [-1, +1], + = drive toward the top face
 * @param stick_roll   [-1, +1], + = turn left (CCW seen from above)
 * @param dt           [s]
 * @param out          signed wheel rates [steps/s]; airborne pair = 0
 */
void balance_step(balance_ctrl_t *b, const balance_cfg_t *cfg,
                  omnis_balance_pair_t pair, float lean_rad, float lean_rate,
                  float stick_pitch, float stick_roll, float dt,
                  wheel_rates_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BALANCE_H */
