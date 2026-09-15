/**
 * @file    pid.h
 * @brief   Discrete PID and the outer velocity-bias loop for OMNIS balance.
 *
 * Implements omnis-info.md §13b's cascaded architecture:
 *
 *   INNER (500 Hz)  pid_ctrl_t         — lean-angle PID. Output is a commanded
 *                                   wheel ACCELERATION, per §13b.
 *   OUTER (~20 Hz)  vel_bias_t    — integrates commanded step-rate as a
 *                                   velocity proxy and biases the lean target
 *                                   to pull it back toward zero. This is what
 *                                   stops "balanced but drifting across the
 *                                   room", and is where §7d's trim pot lands.
 *
 * TWO DELIBERATE DEPARTURES from a textbook PID, both required here:
 *
 *  1. DERIVATIVE ON MEASUREMENT, from the gyro directly. §13b is explicit:
 *     "angle_rate taken directly from gyro rather than differentiated from the
 *     EKF output, for lower latency". So pid_update() takes the rate as an
 *     argument instead of differencing the error. This also removes derivative
 *     kick — a step change in setpoint no longer produces an impulse in D.
 *
 *  2. CONDITIONAL-INTEGRATION ANTI-WINDUP. On an open-loop stepper balancer the
 *     output saturates often (§13c), and a wound-up integrator on a balancing
 *     robot is a fall. The integrator is frozen whenever it would push further
 *     into saturation, and released the instant the error reverses.
 *
 * NO NUMERIC GAINS ARE PROVIDED. omnis-info.md §13d declines to guess them
 * without mass, CG height and wheel radius, and that refusal is correct — a
 * number offered without those is a guess dressed up as a spec. pid-reference.md
 * §5 gives the empirical tuning procedure instead.
 *
 * UNITS: the PID is unit-agnostic; the gains carry the units. See
 * pid-reference.md §3 for the dimensional analysis of this specific loop.
 *
 * NO RTOS: pure arithmetic on caller-owned structs. No allocation, no statics.
 *
 * Reference: assets/control/pid-reference.md
 */

#ifndef PID_H
#define PID_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Inner-loop gains and limits. */
typedef struct {
    float kp;
    float ki;
    float kd;
    float out_min;        /**< output clamp, low          */
    float out_max;        /**< output clamp, high         */
    float integral_max;   /**< |integral term| hard cap   */
} pid_config_t;

/** Inner-loop state. Zero-initialise then pid_init(). */
typedef struct {
    pid_config_t cfg;
    float integral;       /**< accumulated ki * err * dt          */
    float last_p;         /**< diagnostics: proportional term     */
    float last_d;         /**< diagnostics: derivative term       */
    float last_error;     /**< diagnostics                        */
    bool  saturated;      /**< output hit a clamp on the last call */
} pid_ctrl_t;

/** Fill cfg with zero gains and symmetric unit limits. Gains MUST be set by
 *  the caller — see §13d, there are no sensible defaults for this plant. */
void pid_config_defaults(pid_config_t *cfg);

/** Initialise state and copy the config. Clears the integrator. */
void pid_init(pid_ctrl_t *pid, const pid_config_t *cfg);

/**
 * @brief Clear the integrator and diagnostics, keeping the gains.
 *
 * Call on every transition into balance mode. Carrying an integrator across a
 * mode change applies correction earned under completely different dynamics.
 */
void pid_reset(pid_ctrl_t *pid);

/**
 * @brief One control step.
 *
 *   error = setpoint - measurement
 *   P     = kp * error
 *   I     = clamp(I + ki * error * dt), frozen if it worsens saturation
 *   D     = -kd * measurement_rate          (derivative on measurement)
 *   out   = clamp(P + I + D)
 *
 * @param setpoint          target lean angle [rad]
 * @param measurement       fused lean angle  [rad]
 * @param measurement_rate  gyro rate about the lean axis [rad/s] — RAW gyro,
 *                          not a differentiated estimate
 * @param dt                timestep [s]
 * @return clamped control output
 */
float pid_update(pid_ctrl_t *pid, float setpoint, float measurement,
                 float measurement_rate, float dt);

static inline float pid_integral(const pid_ctrl_t *p)  { return p->integral;  }
static inline float pid_last_p(const pid_ctrl_t *p)    { return p->last_p;    }
static inline float pid_last_d(const pid_ctrl_t *p)    { return p->last_d;    }
static inline bool  pid_saturated(const pid_ctrl_t *p) { return p->saturated; }

/* ======================================================================== */

/**
 * Outer velocity-bias loop (omnis-info.md §13b).
 *
 * With no encoders there is no true velocity feedback, so the INTEGRAL OF THE
 * COMMANDED STEP RATE stands in as a proxy for sustained drive effort. If the
 * robot has been asked to drive forward for a while, that integral grows, and
 * the loop leans the target BACKWARD to bleed it off — which is exactly how a
 * balancing robot slows down.
 *
 * Sign: a positive (forward) drive integral produces a NEGATIVE lean target.
 * The controller must lean back to stop going forward.
 *
 * This is the weakest link in the whole stack and §13a says so plainly: the
 * proxy believes every commanded step actually happened. A slipped or lost step
 * is invisible here and shows up as slow drift. Encoders are the fix; the
 * mitigation without them is generous current margin and acceleration limits.
 */
typedef struct {
    float gain;             /**< integral -> lean target [rad per (step/s * s)] */
    float lean_limit_rad;   /**< clamp on the produced target [rad]             */
    float trim_rad;         /**< static offset — §7d's IMU trim pot lives here  */
    float leak;             /**< per-second fractional decay of the integral    */
    float integral_max;     /**< hard cap on the raw integral                   */
} vel_bias_config_t;

typedef struct {
    vel_bias_config_t cfg;
    float integral;         /**< accumulated commanded step-rate [steps]        */
} vel_bias_t;

/** Zero gain, 6 deg limit, no trim, gentle leak. Gain needs bench tuning. */
void vel_bias_config_defaults(vel_bias_config_t *cfg);

void vel_bias_init(vel_bias_t *vb, const vel_bias_config_t *cfg);

/** Clear the accumulated drive-effort integral, keeping the config. */
void vel_bias_reset(vel_bias_t *vb);

/**
 * @brief Advance the outer loop and produce a lean-angle target.
 *
 * @param commanded_step_rate  signed fore-aft step-rate command [microsteps/s]
 * @param dt                   outer-loop timestep [s] (~0.05 s at 20 Hz)
 * @return lean target [rad], clamped, trim included
 */
float vel_bias_update(vel_bias_t *vb, float commanded_step_rate, float dt);

static inline float vel_bias_integral(const vel_bias_t *v) { return v->integral; }

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
