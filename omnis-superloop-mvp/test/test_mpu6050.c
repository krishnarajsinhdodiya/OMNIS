/**
 * @file    test_mpu6050.c
 * @brief   Host-side verification of the MPU6050 burst decoder.
 *
 * The I2C transport cannot run on a Mac, but the byte-level decoding can — and
 * it is where a sign or scaling error would silently poison every angle
 * downstream. Expected values are computed from the datasheet scale factors.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "mpu6050_regs.h"

static int g_fail = 0, g_pass = 0;

static void chk(const char *name, double got, double want, double tol)
{
    if (fabs(got - want) <= tol) { ++g_pass; printf("  ok    %-46s %12.6f\n", name, got); }
    else { ++g_fail; printf("  FAIL  %-46s %12.6f (want %.6f)\n", name, got, want); }
}

static void chk_true(const char *name, int cond)
{
    if (cond) { ++g_pass; printf("  ok    %-46s\n", name); }
    else      { ++g_fail; printf("  FAIL  %-46s\n", name); }
}

static void put16(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)((uint16_t)v >> 8);
    p[1] = (uint8_t)((uint16_t)v & 0xFF);
}

int main(void)
{
    puts("OMNIS MPU6050 decoder verification");

    puts("\nBig-endian two's complement");
    const uint8_t pos[2] = { 0x20, 0x00 }, neg[2] = { 0xE0, 0x00 },
                  min[2] = { 0x80, 0x00 }, max[2] = { 0x7F, 0xFF };
    chk("0x2000 ->  8192", mpu6050_be16(pos),  8192.0, 0);
    chk("0xE000 -> -8192", mpu6050_be16(neg), -8192.0, 0);
    chk("0x8000 -> -32768", mpu6050_be16(min), -32768.0, 0);
    chk("0x7FFF ->  32767", mpu6050_be16(max),  32767.0, 0);

    puts("\nFull burst at +-4 g / +-500 deg/s");
    uint8_t b[MPU6050_BURST_LEN];
    put16(&b[0],   8192);    /* AX = +1 g                              */
    put16(&b[2],  -4096);    /* AY = -0.5 g                            */
    put16(&b[4],      0);    /* AZ =  0 g                              */
    put16(&b[6],   -521);    /* TEMP = -521/340 + 36.53 = 34.9976 degC */
    put16(&b[8],    655);    /* GX = 10 deg/s                          */
    put16(&b[10], -1310);    /* GY = -20 deg/s                         */
    put16(&b[12],     0);    /* GZ = 0                                 */

    mpu6050_sample_t s;
    mpu6050_parse_burst(b, MPU_DEVICE_6050, &s);
    chk("accel X  [g]", s.accel_g[0],  1.0, 1e-6);
    chk("accel Y  [g]", s.accel_g[1], -0.5, 1e-6);
    chk("accel Z  [g]", s.accel_g[2],  0.0, 1e-6);
    chk("gyro X   [rad/s] (10 deg/s)",  s.gyro_radps[0],  10.0 * M_PI / 180.0, 1e-5);
    chk("gyro Y   [rad/s] (-20 deg/s)", s.gyro_radps[1], -20.0 * M_PI / 180.0, 1e-5);
    chk("gyro Z   [rad/s]", s.gyro_radps[2], 0.0, 1e-6);
    chk("temp     [degC]", s.temp_c, 34.9976, 1e-3);

    puts("\nFull-scale limits");
    put16(&b[0], 32767);
    put16(&b[8], -32768);
    mpu6050_parse_burst(b, MPU_DEVICE_6050, &s);
    chk("accel +full scale ~ +4 g", s.accel_g[0], 32767.0 / 8192.0, 1e-5);
    chk("gyro  -full scale ~ -500 deg/s [deg/s]",
        s.gyro_radps[0] * 180.0 / M_PI, -32768.0 / 65.5, 1e-2);

    puts("\nPlausibility");
    uint8_t z[MPU6050_BURST_LEN], f[MPU6050_BURST_LEN];
    memset(z, 0x00, sizeof z);
    memset(f, 0xFF, sizeof f);
    chk_true("all-0x00 burst rejected (stuck bus / unconfigured)", !mpu6050_burst_plausible(z));
    chk_true("all-0xFF burst rejected (nothing driving SDA)",       !mpu6050_burst_plausible(f));
    chk_true("real burst accepted", mpu6050_burst_plausible(b));
    z[4] = 0x01;
    chk_true("a single non-zero byte is accepted", mpu6050_burst_plausible(z));

    puts("\nWHO_AM_I compatibility");
    chk_true("0x68 genuine MPU6050",  mpu6050_who_am_i_compatible(0x68));
    chk_true("0x70 MPU6500 (GY-521 clone)", mpu6050_who_am_i_compatible(0x70));
    chk_true("0x71 MPU9250", mpu6050_who_am_i_compatible(0x71));
    chk_true("0x00 rejected", !mpu6050_who_am_i_compatible(0x00));
    chk_true("0xFF rejected", !mpu6050_who_am_i_compatible(0xFF));
    chk_true("0x69 rejected (that is an ADDRESS, not an identity)",
             !mpu6050_who_am_i_compatible(0x69));

    puts("\nConfiguration constants match the documented intent");
    chk("SMPLRT_DIV gives 500 Hz from 1 kHz", 1000.0 / (1 + MPU6050_SMPLRT_DIV_500HZ), 500.0, 0);
    chk_true("GYRO_CONFIG FS_SEL = 1 (+-500)",  ((MPU6050_GYRO_FS_500DPS >> 3) & 3) == 1);
    chk_true("ACCEL_CONFIG AFS_SEL = 1 (+-4 g)", ((MPU6050_ACCEL_FS_4G >> 3) & 3) == 1);
    chk_true("INT_PIN_CFG: INT_LEVEL=1 active-low", (MPU6050_INT_PIN_OPEN_DRAIN_LOW & 0x80) != 0);
    chk_true("INT_PIN_CFG: INT_OPEN=1 open-drain",  (MPU6050_INT_PIN_OPEN_DRAIN_LOW & 0x40) != 0);

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
