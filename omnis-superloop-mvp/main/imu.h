/**
 * @file    imu.h
 * @brief   Both IMUs: I2C bring-up, boot calibration, per-tick fusion, frames.
 *
 * The orchestration layer over mpu6050 (transport), attitude_ekf (per-IMU
 * filter) and imu_fusion (combine + fault). The pure maths in those modules is
 * byte-identical to assets/control/ and host-tested there; this file is the
 * part that can only be exercised on the board.
 *
 * PER-TICK DATA PATH, per IMU:
 *
 *   14-byte burst -> decode (sensor frame)
 *     -> imu_apply_mount(OMNIS_IMU_x_MOUNT)       sensor -> body (flat) frame
 *     -> subtract boot gyro bias                   (body frame)
 *     -> imu_apply_mount(balance frame)            ONLY while balancing
 *     -> attitude_ekf_step(measured dt)
 *   then imu_fusion_combine() -> fused angle + 15 deg disagreement fault.
 *
 * Why the order is fixed: the mount must come before the EKF or two healthy
 * antiparallel IMUs read 20 deg apart; the bias is subtracted in the body frame
 * so a later frame change never has to rotate a bias state; and the balance
 * frame exists because the flat-frame pitch folds at 90 deg and loses the lean
 * direction (assets/control/attitude-ekf-derivation.md §7).
 *
 * NO RTOS PRIMITIVES. Called only from the superloop. I2C never runs in an ISR.
 */

#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

#include "omnis_imu_mounting.h"
#include "omnis_params.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Which frame the filters estimate in. */
typedef enum {
    IMU_FRAME_FLAT = 0,        /**< body frame: 4-wheel mode, and whenever disarmed */
    IMU_FRAME_BALANCE_FRONT,   /**< balancing on FL+FR                              */
    IMU_FRAME_BALANCE_REAR,    /**< balancing on RL+RR                              */
} imu_frame_t;

/** One tick's attitude estimate and health. */
typedef struct {
    bool        valid;          /**< both filters running and no comm fault      */
    imu_frame_t frame;          /**< the frame the angles below are expressed in */
    float       roll;           /**< fused [rad]                                  */
    float       pitch;          /**< fused [rad]; in a balance frame: + = falling forward */
    float       var_roll;       /**< fused variance [rad^2] — optimistic, see §4 of the fusion ref */
    float       var_pitch;
    float       gyro[3];        /**< mean bias-corrected rate, in `frame` [rad/s] */
    float       accel[3];       /**< mean specific force, in `frame` [g]          */
    float       disagreement;   /**< angle between the two gravity estimates [rad] */
    bool        fault_disagree; /**< disagreement past the threshold this tick    */
    bool        fault_comm;     /**< an IMU failed too many reads, or froze       */
    uint32_t    read_us;        /**< I2C time spent this tick [us]                */
} imu_state_t;

/** Diagnostics for telemetry and the boot report. */
typedef struct {
    uint8_t  who_am_i[2];
    uint32_t read_errors[2];
    float    bias[2][3];        /**< boot gyro bias, body frame [rad/s] */
    float    temp_c[2];
} imu_diag_t;

/**
 * @brief Create the I2C bus, probe and configure both MPU6050s. BOOT-TIME ONLY.
 *
 * Busy-waits ~400 ms for sensor resets. Returns false if either IMU is missing
 * or fails configuration — both are required, since redundancy and the
 * disagreement check are the point of having two.
 */
bool imu_init(const omnis_params_t *p);

/**
 * @brief Measure each IMU's gyro bias with the robot still. BOOT-TIME ONLY.
 *
 * Paced by the tick (which must be running). Retries if the robot is moved,
 * up to params.imu.cal_attempts. On success the filters are seeded from the
 * averaged accelerometer and imu_update() may be called.
 *
 * @param on_tick  optional hook run once per tick during calibration, so the
 *                 buzzer and radio keep being serviced; may be NULL
 */
bool imu_calibrate(const omnis_params_t *p, void (*on_tick)(void));

/** Read, filter and fuse. Exactly once per tick, from the superloop. */
void imu_update(int64_t now_us, imu_state_t *out);

/**
 * @brief Switch estimation frame and re-seed both filters from the accelerometer.
 *
 * The EKF angle states mean nothing in a different frame, so they are rebuilt
 * from the last accelerometer sample. Bias was already removed in the body
 * frame, so nothing else needs rotating.
 */
void imu_set_frame(imu_frame_t frame);

/** The balance frame for a grounded pair. */
imu_frame_t imu_frame_for_pair(omnis_balance_pair_t pair);

/** Human-readable frame name. */
const char *imu_frame_name(imu_frame_t frame);

void imu_get_diag(imu_diag_t *out);

/**
 * @brief Interactive mount-derivation procedure. Never returns.
 *
 * Enabled by OMNIS_RUN_MOUNT_WIZARD. Requires imu_init() to have succeeded.
 * Walks through the two poses on the serial console, runs imu_mount_resolve()
 * for each IMU on raw sensor-frame data, and prints whether each result matches
 * omnis_imu_mounting.h.
 */
void imu_run_mount_wizard(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */
