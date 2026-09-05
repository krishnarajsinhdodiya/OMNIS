/**
 * @file    attitude_ekf.h
 * @brief   Per-IMU attitude EKF for the OMNIS balance/estimation stack.
 *
 * Implements omnis-info.md §11b: a 4-state filter estimating lean angle and
 * gyro bias on two axes, from one MPU6050.
 *
 *     x = [ roll, pitch, bias_roll, bias_pitch ]
 *
 * Because the process model couples roll only to bias_roll and pitch only to
 * bias_pitch, and the measurement noise matrix is diagonal, the 4x4 filter
 * decomposes EXACTLY into two independent 2-state filters. That is not an
 * approximation — see attitude-ekf-derivation.md §4. This file stores it that
 * way: two ekf_axis_t, 2x2 covariance each, no matrix inversion anywhere.
 *
 * UNITS: radians and rad/s throughout, seconds for dt. Accelerometer input is
 * in g (not m/s^2) because the adaptive-R gate keys off |a| deviating from 1.0.
 * Convert once at the driver boundary; never inside this file.
 *
 * NO RTOS: pure arithmetic on caller-owned structs. No allocation, no statics,
 * no blocking, no peripheral access. Safe to call from a superloop tick. It is
 * technically ISR-safe too, but omnis-info.md's rule stands — the I2C read that
 * feeds it must never happen in an ISR, so neither should this.
 *
 * Reference:  assets/control/attitude-ekf-reference.md
 * Derivation: assets/control/attitude-ekf-derivation.md
 */

#ifndef ATTITUDE_EKF_H
#define ATTITUDE_EKF_H

#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- Default tuning -------------------------------------------------------
 * Q values are continuous-time power spectral densities; the filter multiplies
 * them by dt internally, so the tuning does not change if the loop rate does.
 *
 * These are STARTING POINTS derived in attitude-ekf-reference.md §5, not
 * hardware-verified gains. At 500 Hz they give an accel/gyro crossover of
 * 0.80 Hz (tau = 0.198 s) and a steady-state angle sigma of 0.57 deg.
 * ------------------------------------------------------------------------- */
#define EKF_Q_ANGLE_PSD_DEFAULT   5.0e-4f   /* [rad^2/s]    angle process noise */
#define EKF_Q_BIAS_PSD_DEFAULT    1.0e-7f   /* [rad^2/s^3]  bias random walk    */
#define EKF_R_ACCEL_DEFAULT       1.0e-2f   /* [rad^2]      accel angle noise   */

/* Adaptive measurement trust (omnis-info.md §11b).
 * Below DEV_THRESH the accelerometer is trusted at R_accel. Above it, R grows
 * quadratically; past REJECT_THRESH the update is skipped entirely. The ramp is
 * continuous at DEV_THRESH — a step change in R produces a visible kick in the
 * estimate, which is exactly what you do not want mid-balance. */
#define EKF_ACCEL_DEV_THRESH_G    0.20f     /* start distrusting past this      */
#define EKF_ACCEL_REJECT_THRESH_G 0.50f     /* skip the update entirely past it */
#define EKF_ADAPT_R_GAIN          10.0f     /* per g of excess deviation        */

/** One axis: state [angle, bias] with its 2x2 covariance. */
typedef struct {
    float x[2];      /**< [0] = angle [rad], [1] = gyro bias [rad/s] */
    float P[2][2];   /**< estimate covariance                        */
} ekf_axis_t;

/** Filter tuning. Zero-initialise then call attitude_ekf_config_defaults(). */
typedef struct {
    float q_angle_psd;      /**< [rad^2/s]   */
    float q_bias_psd;       /**< [rad^2/s^3] */
    float r_accel;          /**< [rad^2]     */
    float accel_dev_thresh; /**< [g]         */
    float accel_reject;     /**< [g]         */
    float adapt_r_gain;     /**< [1/g]       */
} attitude_ekf_config_t;

/** One IMU's complete attitude filter. */
typedef struct {
    ekf_axis_t            roll;    /**< about +x, from atan2(ay, az)            */
    ekf_axis_t            pitch;   /**< about +y, from atan2(-ax, hypot(ay,az)) */
    attitude_ekf_config_t cfg;
    float                 last_accel_mag_g;  /**< diagnostics */
    float                 last_r_used;       /**< diagnostics: post-adaptation R */
    bool                  last_update_rejected;
} attitude_ekf_t;

/** Fill cfg with the EKF_*_DEFAULT values above. */
void attitude_ekf_config_defaults(attitude_ekf_config_t *cfg);

/**
 * @brief Reset the filter to zero attitude with a wide initial covariance.
 *
 * Prefer attitude_ekf_seed_from_accel() at boot — starting at zero when the
 * robot is not level costs a full settling time (~1 s) of wrong estimate.
 */
void attitude_ekf_init(attitude_ekf_t *ekf, const attitude_ekf_config_t *cfg);

/**
 * @brief Initialise the angle states directly from one accelerometer sample.
 *
 * Sets covariance to the measurement noise, since that is genuinely how well
 * the angle is known from a single accel reading. Call once at boot, with the
 * robot stationary, after gyro-bias calibration.
 *
 * @param ax,ay,az  accelerometer reading [g]
 */
void attitude_ekf_seed_from_accel(attitude_ekf_t *ekf,
                                  float ax, float ay, float az);

/**
 * @brief Preload the gyro bias states from the boot-time calibration (§12.1).
 *
 * The filter will refine these, but starting from a measured value rather than
 * zero removes the slow bias-convergence transient (Case 3 in the reference:
 * ~92% of a 0.02 rad/s bias is only recovered after a full second).
 */
void attitude_ekf_set_gyro_bias(attitude_ekf_t *ekf,
                                float bias_roll, float bias_pitch);

/**
 * @brief Time update. Integrates bias-corrected gyro rates, grows covariance.
 *
 * @param gx,gy  raw gyro rates about body x and y [rad/s]
 * @param dt     timestep [s]
 */
void attitude_ekf_predict(attitude_ekf_t *ekf, float gx, float gy, float dt);

/**
 * @brief Measurement update from the accelerometer, with adaptive R.
 *
 * Computes both tilt angles by atan2, inflates R according to how far the
 * measured specific force is from 1 g, and applies two sequential scalar
 * updates. Sequential scalar updates are exactly equivalent to one joint 2x2
 * update when R is diagonal — and need no matrix inverse.
 *
 * If |a| deviates by more than cfg.accel_reject, no update is applied and
 * last_update_rejected is set: the platform is accelerating hard enough that
 * the accel-derived angle carries no usable attitude information.
 *
 * @param ax,ay,az  accelerometer reading [g]
 * @return true if an update was applied, false if rejected
 */
bool attitude_ekf_update_accel(attitude_ekf_t *ekf,
                               float ax, float ay, float az);

/** Convenience: predict() then update_accel(). */
bool attitude_ekf_step(attitude_ekf_t *ekf,
                       float gx, float gy,
                       float ax, float ay, float az,
                       float dt);

/* --- Accessors ---------------------------------------------------------- */
static inline float attitude_ekf_roll(const attitude_ekf_t *e)      { return e->roll.x[0];    }
static inline float attitude_ekf_pitch(const attitude_ekf_t *e)     { return e->pitch.x[0];   }
static inline float attitude_ekf_bias_roll(const attitude_ekf_t *e) { return e->roll.x[1];    }
static inline float attitude_ekf_bias_pitch(const attitude_ekf_t *e){ return e->pitch.x[1];   }
static inline float attitude_ekf_var_roll(const attitude_ekf_t *e)  { return e->roll.P[0][0]; }
static inline float attitude_ekf_var_pitch(const attitude_ekf_t *e) { return e->pitch.P[0][0];}

/** Wrap an angle difference to [-pi, +pi]. Exposed because the fusion layer
 *  and any consumer comparing two angles needs exactly this. */
float ekf_wrap_pi(float a);

#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_EKF_H */
