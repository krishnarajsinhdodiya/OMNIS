/**
 * @file    omnis_params.h
 * @brief   Runtime-tunable parameters for the OMNIS superloop MVP.
 *
 * Mirrors the `params.json` schema from omnis-info.md §9f, but populated from
 * compile-time defaults because this build has no SD card (PLAN.md: microSD and
 * FATFS are excluded, deferred not abandoned).
 *
 * WHY A STRUCT AND NOT #defines: §9f's whole point is that these become a file
 * on a card later. Keeping them as fields of one struct passed by pointer means
 * the JSON loader drops in without touching a single call site. Bake them into
 * macros now and that port becomes a rewrite.
 *
 * GEOMETRY IS NEVER INLINE IN A FORMULA. mecanum_kinematics.c takes
 * wheel_radius_mm / wheelbase_mm / track_width_mm as arguments precisely so a
 * frame change is a config edit.
 */

#ifndef OMNIS_PARAMS_H
#define OMNIS_PARAMS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Bumped when the shape of this struct changes. Mirrors §9f schema_version. */
#define OMNIS_PARAMS_SCHEMA_VERSION   1

/** §9f "geometry". As-built, confirmed. */
typedef struct {
    float wheel_radius_mm;   /**< 30  */
    float wheelbase_mm;      /**< 223 — front-to-back wheel-centre distance */
    float track_width_mm;    /**< 230 — left-to-right wheel-centre distance */
} omnis_geometry_t;

/** RC stick -> body velocity scaling (kinematics reference §5). */
typedef struct {
    float vx_max_mmps;        /**< throttle at full deflection      */
    float vx_secondary_mmps;  /**< pitch stick's additive contribution */
    float vy_max_mmps;        /**< roll at full deflection (+ = left) */
    float w_max_radps;        /**< yaw at full deflection (+ = CCW)   */

    /* Stick polarity. The reference is emphatic that a transmitter which is
     * right-positive on roll or yaw must be corrected HERE and nowhere else:
     * "Never negate inside the kinematics." Flipping a matrix sign to fix a
     * backwards stick breaks the IK->FK round trip and every other motion. */
    bool invert_throttle;
    bool invert_pitch;
    bool invert_roll;         /**< set if stick-left gives negative */
    bool invert_yaw;          /**< set if stick-left gives negative */
} omnis_rc_scale_t;

/** §9f "rc.channel_map". 1-based CRSF channel numbers. */
typedef struct {
    uint8_t throttle;
    uint8_t pitch;
    uint8_t roll;
    uint8_t yaw;
    /* ARM switch (§7a's "emergency kill switch"). HIGH = motors permitted; LOW
     * or MID = never. The field keeps §9f's name so params.json stays aligned. */
    uint8_t kill_switch;
    uint8_t drive_mode;          /**< 3-pos: LOW flat, MID balance, HIGH auto */
    uint8_t speed_limiter;       /**< 3-pos: LOW / MID / HIGH                  */
    uint8_t tune_pot;            /**< S1 slider — optional live kp tuning      */
    bool    arm_switch_invert;   /**< true if the switch reads LOW when "up"   */
} omnis_channel_map_t;

/** Step-generation limits (§13c: the only defence against lost steps). */
typedef struct {
    float max_step_rate;       /**< per-wheel magnitude cap [microsteps/s]   */
    float deadband_steps;      /**< below this, hold the wheel stopped       */
    float max_accel_steps_s2;  /**< slew limit on commanded rate             */
} omnis_step_limits_t;

/** §9f "balance". Gains are ZERO by default — see §13d and pid.h. */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_max;
    float trim_rad;            /**< §7d's trim pot: static lean-target offset */
    float lean_limit_rad;      /**< outer loop's clamp                       */
    float vel_bias_gain;
} omnis_balance_t;

/** IMU health, calibration and attitude-fault limits (omnis-info.md §11c, §12). */
typedef struct {
    float    disagree_thresh_rad;     /**< §11c: IMUs disagreeing past this = fault  */
    uint16_t comm_fail_reads;         /**< consecutive failed reads -> comm fault    */
    uint16_t frozen_reads;            /**< consecutive identical bursts -> comm fault */
    uint16_t cal_samples;             /**< boot gyro-bias samples (one per tick)     */
    uint8_t  cal_attempts;            /**< retries if the robot is moved             */
    float    cal_gyro_std_max_radps;  /**< stillness gate: gyro std-dev per axis     */
    float    cal_accel_std_max_g;     /**< stillness gate: |a| std-dev               */
    float    cal_accel_tol_g;         /**< |mean |a| - 1 g| must be under this        */
    float    flat_tilt_fault_rad;     /**< armed in flat mode and tipped past this   */
} omnis_imu_params_t;

/** Radio link settings. */
typedef struct {
    uint32_t crsf_baud;          /**< ExpressLRS default for CRSF: 420000      */
    float    stick_deadzone;     /**< rescaled deadzone on the four sticks     */
    float    center_tolerance;   /**< arming requires every stick within this  */
    uint32_t pin_probe_ms;       /**< no frames -> try the other RX pin        */
    uint8_t  min_link_quality;   /**< uplink LQ floor [%]; 0 disables          */
} omnis_rc_params_t;

/** §9f "control": speed-limiter switch scale factors. */
typedef struct {
    float speed_low;
    float speed_med;
    float speed_high;
} omnis_control_t;

/** A4988 STEP/DIR electrical settings (Stage 5). */
typedef struct {
    /* Per-wheel DIR polarity, ordered [FL, FR, RL, RR]. The kinematics' +omega is
     * "the sense that drives the robot forward", and the left and right motors
     * are mirror images across the chassis, so the same wheel direction needs
     * opposite shaft rotation on each side. Fix a backwards wheel HERE — never by
     * flipping a sign in the IK (kinematics reference §6). */
    bool     dir_invert[4];
    uint16_t step_pulse_us;      /**< STEP high time. A4988 minimum is 1 us.  */
    uint16_t step_min_low_us;    /**< STEP low-time floor between pulses.     */
} omnis_motor_t;

/** Everything, in one place. */
typedef struct {
    uint32_t             schema_version;
    omnis_geometry_t     geometry;
    omnis_rc_scale_t     rc_scale;
    omnis_channel_map_t  channel_map;
    omnis_step_limits_t  step;
    omnis_balance_t      balance;
    omnis_imu_params_t   imu;
    omnis_rc_params_t    rc;
    omnis_control_t      control;
    omnis_motor_t        motor;
    uint32_t             rc_timeout_us;   /**< link-loss threshold (§7f) */
} omnis_params_t;

/**
 * @brief Populate with the compile-time defaults.
 *
 * Every value is either as-built hardware truth (geometry), derived from a
 * verified figure (the step-rate cap, from Case D's practical ceiling), or
 * deliberately zero because guessing it would be worse than not running
 * (the PID gains, per §13d).
 */
void omnis_params_defaults(omnis_params_t *p);

/**
 * @brief Cheap consistency check. Catches a zeroed or half-initialised struct
 *        before it reaches the kinematics and produces a divide-by-zero.
 *
 * @return true if the parameters are usable
 */
bool omnis_params_valid(const omnis_params_t *p);

/** Read-only handle to the process-wide parameter set. */
const omnis_params_t *omnis_params(void);

/** Initialise the process-wide set from defaults. Call once, early. */
void omnis_params_init(void);

#ifdef __cplusplus
}
#endif

#endif /* OMNIS_PARAMS_H */
