/**
 * @file    omnis_config.h
 * @brief   Build-time switches. Edit, rebuild, reflash.
 *
 * Runtime-tunable numbers (gains, limits, geometry) live in omnis_params.c,
 * because they will become params.json on the SD card. The switches here change
 * what the firmware IS, not how it is tuned, so they stay compile-time.
 */

#ifndef OMNIS_CONFIG_H
#define OMNIS_CONFIG_H

/* --- Loop rate -----------------------------------------------------------
 * 500 Hz per omnis-info.md §11d, verified on hardware in Stage 1 with zero
 * overruns and no measurable jitter. If the I2C reads in Stage 3 push the loop
 * body past ~1.6 ms (telemetry "body" field), either set
 * OMNIS_IMU_READ_ALTERNATE below or drop this to 250. Every consumer uses
 * measured dt, so nothing else needs to change.
 * ------------------------------------------------------------------------ */
#define OMNIS_TICK_RATE_HZ              500u

/* --- I2C -----------------------------------------------------------------
 * 400 kHz fast mode. The GY-521 modules carry their own pull-ups; drop to
 * 100000 if the bus proves marginal on the finished harness.
 * ------------------------------------------------------------------------ */
#define OMNIS_I2C_SCL_HZ                400000u

/* --- IMU read pattern ----------------------------------------------------
 * 0: read both IMUs every tick (each at the full tick rate).
 * 1: read IMU A on even ticks and IMU B on odd ticks. Halves the I2C time per
 *    tick; each filter then runs at half the tick rate using its own measured
 *    dt. Use this if the loop overruns with both reads in one tick.
 * ------------------------------------------------------------------------ */
#define OMNIS_IMU_READ_ALTERNATE        0

/* --- IMU mount wizard ----------------------------------------------------
 * 1: boot into an interactive two-pose procedure that derives each IMU's
 *    mounting descriptor with imu_mount_resolve() and prints the constants to
 *    paste into omnis_imu_mounting.h. The robot never arms in this mode.
 *    See TESTING.md, Stage 3.
 * ------------------------------------------------------------------------ */
#define OMNIS_RUN_MOUNT_WIZARD          0

#endif /* OMNIS_CONFIG_H */
