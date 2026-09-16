/**
 * @file    mpu6050.h
 * @brief   Register-level MPU6050 driver on the ESP-IDF v6 i2c_master API.
 *
 * NO RTOS PRIMITIVES in this code. The i2c_master driver blocks internally on
 * its own completion event while a transfer is on the bus — that is an IDF
 * driver internal, treated as hardware under BUILD-LOG.md Planning 1a — and is only ever
 * called from the superloop, never from an ISR.
 *
 * Health tracking lives here because this is the only layer that can see it.
 * On board Rev 2.0 the two INT pins share one line, so a dead IMU cannot be
 * detected from its interrupt; it has to be detected from I2C errors and from
 * data that stops changing.
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#include "mpu6050_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_master_dev_handle_t dev;
    uint8_t  addr;
    uint8_t  who_am_i;                       /**< identity read at configure   */
    bool     configured;
    uint8_t  last_burst[MPU6050_BURST_LEN];  /**< for frozen-data detection    */
    uint32_t read_errors;                    /**< total, since boot            */
    uint32_t consecutive_errors;             /**< reset by any good read       */
    uint32_t identical_bursts;               /**< consecutive byte-identical   */
} mpu6050_t;

/**
 * @brief Probe the address and register the device on the bus.
 * @return ESP_OK, or the probe error if nothing ACKs at that address.
 */
esp_err_t mpu6050_attach(i2c_master_bus_handle_t bus, uint8_t addr,
                         uint32_t scl_speed_hz, mpu6050_t *m);

/**
 * @brief Reset, identify, and configure the device. BOOT-TIME ONLY.
 *
 * Busy-waits ~200 ms in total for reset and filter settling. Every register
 * written is read back and compared, which catches a failed write, a wrong
 * address, or a second device answering in place of this one.
 */
esp_err_t mpu6050_configure(mpu6050_t *m);

/**
 * @brief Read one 14-byte sample burst. Called from the superloop every tick.
 *
 * Fails (and counts an error) on an I2C error or an implausible burst. Updates
 * identical_bursts; the caller decides how many identical bursts mean frozen.
 */
esp_err_t mpu6050_read(mpu6050_t *m, mpu6050_sample_t *out);

/** Raw burst read without decoding — used by the mount wizard. */
esp_err_t mpu6050_read_raw(mpu6050_t *m, uint8_t burst[MPU6050_BURST_LEN]);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
