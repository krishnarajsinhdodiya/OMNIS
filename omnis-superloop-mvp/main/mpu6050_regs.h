/**
 * @file    mpu6050_regs.h
 * @brief   MPU6050 register map, configuration values and the pure burst parser.
 *
 * Header-only and free of ESP-IDF includes, so the byte-level decoding can be
 * unit-tested on the host (test/test_mpu6050.c). The I2C transport lives in
 * mpu6050.c.
 *
 * Configuration chosen for OMNIS, and why:
 *
 *   Gyro  +-500 deg/s   65.5 LSB/(deg/s). A balancing robot recovering from a
 *                       shove turns fast; +-250 clips, +-2000 wastes resolution.
 *   Accel +-4 g         8192 LSB/g. Stepper vibration spikes well past 2 g, and
 *                       a clipped accel reading is a silently wrong angle.
 *   DLPF  cfg 2         accel 94 Hz / 3.0 ms, gyro 98 Hz / 2.8 ms. Removes
 *                       stepper chatter while adding only ~3 ms of lag to a
 *                       balance loop. cfg 3 (44 Hz / ~5 ms) is the next step if
 *                       vibration dominates on the real chassis.
 *   Rate  1 kHz / (1+1) 500 Hz internal sample rate = the tick rate, so every
 *                       poll sees a fresh sample and none is wasted.
 *   INT   open-drain,   Both INT pins share GPIO7 with one 10 k pull-up on board
 *         active-low    Rev 2.0. Push-pull outputs wired together fight, so this
 *                       is a hardware-protection setting, not a preference. The
 *                       interrupt itself stays disabled; the tick polls.
 */

#ifndef MPU6050_REGS_H
#define MPU6050_REGS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* --- Registers ----------------------------------------------------------- */
#define MPU6050_REG_SMPLRT_DIV          0x19
#define MPU6050_REG_CONFIG              0x1A
#define MPU6050_REG_GYRO_CONFIG         0x1B
#define MPU6050_REG_ACCEL_CONFIG        0x1C
#define MPU6050_REG_INT_PIN_CFG         0x37
#define MPU6050_REG_INT_ENABLE          0x38
#define MPU6050_REG_ACCEL_XOUT_H        0x3B   /* first of the 14-byte burst */
#define MPU6050_REG_PWR_MGMT_1          0x6B
#define MPU6050_REG_WHO_AM_I            0x75

/* --- Values -------------------------------------------------------------- */
#define MPU6050_WHO_AM_I_GENUINE        0x68
#define MPU6050_BURST_LEN               14u    /* AX AY AZ TEMP GX GY GZ, big-endian */

#define MPU6050_PWR_DEVICE_RESET        0x80
#define MPU6050_PWR_CLKSEL_PLL_XGYRO    0x01   /* wake, PLL on X gyro: better than the RC osc */
#define MPU6050_CONFIG_DLPF_94HZ        0x02
#define MPU6050_SMPLRT_DIV_500HZ        0x01
#define MPU6050_GYRO_FS_500DPS          0x08
#define MPU6050_ACCEL_FS_4G             0x08
#define MPU6050_INT_PIN_OPEN_DRAIN_LOW  0xC0   /* INT_LEVEL=1 (active low) | INT_OPEN=1 */
#define MPU6050_INT_ENABLE_NONE         0x00

/* --- Scaling ------------------------------------------------------------- */
#define MPU6050_GYRO_LSB_PER_DPS        65.5f
#define MPU6050_ACCEL_LSB_PER_G         8192.0f
#define MPU6050_DEG_TO_RAD              0.017453292519943295f

/** One decoded sample, in the SENSOR frame (before any mounting remap). */
typedef struct {
    float accel_g[3];      /**< specific force [g], reads +1 along "up" at rest */
    float gyro_radps[3];   /**< angular rate [rad/s], right-handed             */
    float temp_c;          /**< die temperature [degC] — diagnostic only        */
} mpu6050_sample_t;

/**
 * @brief Is this WHO_AM_I value a register-compatible part?
 *
 * GY-521 boards are very frequently populated with an MPU6500-family chip
 * rather than a genuine MPU6050. Those answer 0x70/0x71/0x72/0x73 (or 0x98 on
 * some clones) instead of 0x68, but every register this driver touches —
 * PWR_MGMT_1, CONFIG, SMPLRT_DIV, GYRO/ACCEL_CONFIG, INT_PIN_CFG and the
 * 0x3B data burst — has the same address and meaning. Refusing to run on them
 * would reject most of the parts actually sold. Only the temperature formula
 * differs, and temperature is diagnostic only.
 *
 * 0x00 and 0xFF are rejected: those are a dead bus or a device that did not
 * drive the line, never a real identity.
 */
typedef enum {
    MPU_DEVICE_UNKNOWN = 0,
    MPU_DEVICE_6050,
    MPU_DEVICE_6500,
    MPU_DEVICE_6515,
} mpu_device_type_t;

static inline mpu_device_type_t
mpu6050_device_type_from_who_am_i(uint8_t who)
{
    switch (who) {
        case 0x68:
            return MPU_DEVICE_6050;

        case 0x70:
            return MPU_DEVICE_6500;

        case 0x74:
            return MPU_DEVICE_6515;

        default:
            return MPU_DEVICE_UNKNOWN;
    }
}

static inline const char *
mpu6050_device_type_name(mpu_device_type_t type)
{
    switch (type) {
        case MPU_DEVICE_6050:
            return "MPU6050";

        case MPU_DEVICE_6500:
            return "MPU6500";

        case MPU_DEVICE_6515:
            return "MPU6515";

        default:
            return "UNKNOWN";
    }
}

 static inline bool mpu6050_who_am_i_compatible(uint8_t who)
{
    switch (who) {
        case 0x68:  /* MPU6050 */
        case 0x70:  /* MPU6500 */
        case 0x71:  /* MPU9250 */
        case 0x72:
        case 0x73:
        case 0x74:  /* MPU6515 */
        case 0x98:
            return true;

        default:
            return false;
    }
}

/** Big-endian two's-complement 16-bit field. */
static inline int16_t mpu6050_be16(const uint8_t *p)
{
    return (int16_t)(uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/**
 * @brief Decode the 14-byte burst read from ACCEL_XOUT_H.
 *
 * Layout: [0..5] accel X/Y/Z, [6..7] temperature, [8..13] gyro X/Y/Z. Units
 * follow the full-scale settings above; change those and these divisors must
 * change with them.
 */
static inline void mpu6050_parse_burst(
    const uint8_t b[MPU6050_BURST_LEN],
    mpu_device_type_t type,
    mpu6050_sample_t *s)
{
    for (int i = 0; i < 3; ++i) {
        s->accel_g[i]    = (float)mpu6050_be16(&b[2 * i]) / MPU6050_ACCEL_LSB_PER_G;
        s->gyro_radps[i] = (float)mpu6050_be16(&b[8 + 2 * i])
                           / MPU6050_GYRO_LSB_PER_DPS * MPU6050_DEG_TO_RAD;
    }
    const int16_t temp_raw = mpu6050_be16(&b[6]);
    switch (type) {
        case MPU_DEVICE_6050:
            s->temp_c = (float)temp_raw / 340.0f + 36.53f;
            break;

        case MPU_DEVICE_6500:
        case MPU_DEVICE_6515:
            s->temp_c = (float)temp_raw / 333.87f + 21.0f;
            break;

        default:
            s->temp_c = 0.0f;
            break;
    }
}

/**
 * @brief Reject bursts that cannot be real data.
 *
 * All-0x00 or all-0xFF is a stuck bus, a device held in reset, or one that
 * never configured. Real data can never be all zeros — gravity alone guarantees
 * a non-zero accelerometer. Feeding either to the EKF would inject a confident,
 * wrong attitude.
 */
static inline bool mpu6050_burst_plausible(const uint8_t b[MPU6050_BURST_LEN])
{
    bool all_00 = true;
    bool all_ff = true;
    for (size_t i = 0; i < MPU6050_BURST_LEN; ++i) {
        if (b[i] != 0x00) all_00 = false;
        if (b[i] != 0xFF) all_ff = false;
    }
    return !(all_00 || all_ff);
}

#endif /* MPU6050_REGS_H */
