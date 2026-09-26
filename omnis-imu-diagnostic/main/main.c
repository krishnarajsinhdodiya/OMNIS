#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"
#include "esp_err.h"

/* ============================================================
 * OMNIS REV 2.0
 * MPU6050 / MPU6500 / MPU6515 IDENTITY TEST
 *
 * SDA  = GPIO8
 * SCL  = GPIO9
 *
 * IMU-A = 0x68
 * IMU-B = 0x69
 * ============================================================ */

#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_GPIO    GPIO_NUM_8
#define I2C_SCL_GPIO    GPIO_NUM_9

#define I2C_FREQ_HZ     400000
#define I2C_TIMEOUT_MS  100

#define IMU_A_ADDR      0x68
#define IMU_B_ADDR      0x69

/* ============================================================
 * WHO_AM_I values
 * ============================================================ */

#define WHOAMI_MPU6050  0x68
#define WHOAMI_MPU6500  0x70
#define WHOAMI_MPU6515  0x74

/* ============================================================
 * Common MPU registers
 * ============================================================ */

#define REG_SMPLRT_DIV          0x19
#define REG_CONFIG              0x1A
#define REG_GYRO_CONFIG         0x1B
#define REG_ACCEL_CONFIG        0x1C
#define REG_ACCEL_CONFIG2       0x1D

#define REG_FIFO_EN             0x23

#define REG_INT_PIN_CFG         0x37
#define REG_INT_ENABLE          0x38
#define REG_INT_STATUS          0x3A

#define REG_ACCEL_XOUT_H        0x3B
#define REG_TEMP_OUT_H          0x41
#define REG_GYRO_XOUT_H         0x43

#define REG_EXT_SENS_DATA_00    0x49

#define REG_ACCEL_INTEL_CTRL    0x69

#define REG_USER_CTRL           0x6A
#define REG_PWR_MGMT_1          0x6B
#define REG_PWR_MGMT_2          0x6C

#define REG_FIFO_COUNTH         0x72
#define REG_FIFO_COUNTL         0x73
#define REG_FIFO_R_W            0x74

#define REG_WHO_AM_I            0x75

/* MPU6500 / MPU6515 extended registers */

#define REG_XG_OFFS_TC          0x00
#define REG_YG_OFFS_TC          0x01
#define REG_ZG_OFFS_TC          0x02

#define REG_X_FINE_GAIN         0x03
#define REG_Y_FINE_GAIN         0x04
#define REG_Z_FINE_GAIN         0x05

#define REG_XA_OFFS_H           0x06
#define REG_XA_OFFS_L           0x07
#define REG_YA_OFFS_H           0x08
#define REG_YA_OFFS_L           0x09
#define REG_ZA_OFFS_H           0x0A
#define REG_ZA_OFFS_L           0x0B

#define REG_PRODUCT_ID          0x0C

#define REG_SELF_TEST_X_GYRO   0x00
#define REG_SELF_TEST_Y_GYRO   0x01
#define REG_SELF_TEST_Z_GYRO   0x02

#define REG_SELF_TEST_X_ACCEL   0x0D
#define REG_SELF_TEST_Y_ACCEL   0x0E
#define REG_SELF_TEST_Z_ACCEL   0x0F

#define REG_XG_OFFS_USRH        0x13
#define REG_XG_OFFS_USRL        0x14
#define REG_YG_OFFS_USRH        0x15
#define REG_YG_OFFS_USRL        0x16
#define REG_ZG_OFFS_USRH        0x17
#define REG_ZG_OFFS_USRL        0x18

#define REG_ACCEL_XOFFS_H       0x77
#define REG_ACCEL_YOFFS_H       0x7A
#define REG_ACCEL_ZOFFS_H       0x7D

/* ============================================================
 * I2C helpers
 * ============================================================ */

static esp_err_t read_reg(
    i2c_master_dev_handle_t dev,
    uint8_t reg,
    uint8_t *value)
{
    return i2c_master_transmit_receive(
        dev,
        &reg,
        1,
        value,
        1,
        I2C_TIMEOUT_MS
    );
}

static esp_err_t write_reg(
    i2c_master_dev_handle_t dev,
    uint8_t reg,
    uint8_t value)
{
    uint8_t data[2] = {
        reg,
        value
    };

    return i2c_master_transmit(
        dev,
        data,
        2,
        I2C_TIMEOUT_MS
    );
}

static uint8_t read8(
    i2c_master_dev_handle_t dev,
    uint8_t reg)
{
    uint8_t value = 0xFF;

    if (read_reg(dev, reg, &value) != ESP_OK) {
        return 0xFF;
    }

    return value;
}

/* ============================================================
 * Print register
 * ============================================================ */

static void show_reg(
    i2c_master_dev_handle_t dev,
    uint8_t reg,
    const char *name)
{
    uint8_t value = 0;

    esp_err_t err =
        read_reg(
            dev,
            reg,
            &value
        );

    if (err == ESP_OK) {

        printf(
            "  0x%02X %-24s = 0x%02X\n",
            reg,
            name,
            value
        );

    } else {

        printf(
            "  0x%02X %-24s = ERROR %s\n",
            reg,
            name,
            esp_err_to_name(err)
        );
    }
}

/* ============================================================
 * Identify device
 * ============================================================ */

static const char *identify(
    uint8_t who)
{
    switch (who) {

        case WHOAMI_MPU6050:
            return "MPU6050";

        case WHOAMI_MPU6500:
            return "MPU6500";

        case WHOAMI_MPU6515:
            return "MPU6515";

        default:
            return "UNKNOWN";
    }
}

/* ============================================================
 * Raw sensor data
 * ============================================================ */

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t temp;

    int16_t gx;
    int16_t gy;
    int16_t gz;

} imu_raw_t;

static esp_err_t read_raw(
    i2c_master_dev_handle_t dev,
    imu_raw_t *raw)
{
    uint8_t reg = REG_ACCEL_XOUT_H;
    uint8_t data[14];

    esp_err_t err =
        i2c_master_transmit_receive(
            dev,
            &reg,
            1,
            data,
            sizeof(data),
            I2C_TIMEOUT_MS
        );

    if (err != ESP_OK) {
        return err;
    }

    raw->ax =
        (int16_t)((data[0] << 8) | data[1]);

    raw->ay =
        (int16_t)((data[2] << 8) | data[3]);

    raw->az =
        (int16_t)((data[4] << 8) | data[5]);

    raw->temp =
        (int16_t)((data[6] << 8) | data[7]);

    raw->gx =
        (int16_t)((data[8] << 8) | data[9]);

    raw->gy =
        (int16_t)((data[10] << 8) | data[11]);

    raw->gz =
        (int16_t)((data[12] << 8) | data[13]);

    return ESP_OK;
}

static void print_raw(
    imu_raw_t *r)
{
    printf(
        "  ACC  raw: X=%6d Y=%6d Z=%6d\n",
        r->ax,
        r->ay,
        r->az
    );

    printf(
        "  TEMP raw: %6d\n",
        r->temp
    );

    printf(
        "  GYRO raw: X=%6d Y=%6d Z=%6d\n",
        r->gx,
        r->gy,
        r->gz
    );
}

/* ============================================================
 * Basic configuration
 * ============================================================ */

static void configure_common(
    i2c_master_dev_handle_t dev)
{
    printf(
        "\nWriting common MPU configuration...\n"
    );

    /*
     * PLL X gyro
     */
    write_reg(
        dev,
        REG_PWR_MGMT_1,
        0x01
    );

    /*
     * All accel + gyro axes enabled
     */
    write_reg(
        dev,
        REG_PWR_MGMT_2,
        0x00
    );

    /*
     * DLPF
     */
    write_reg(
        dev,
        REG_CONFIG,
        0x02
    );

    /*
     * 500 Hz sample rate
     */
    write_reg(
        dev,
        REG_SMPLRT_DIV,
        0x01
    );

    /*
     * Gyro ±500 dps
     */
    write_reg(
        dev,
        REG_GYRO_CONFIG,
        0x08
    );

    /*
     * Accel ±4 g
     */
    write_reg(
        dev,
        REG_ACCEL_CONFIG,
        0x08
    );

    /*
     * MPU6500/6515 has ACCEL_CONFIG2.
     *
     * 0x00 selects normal accelerometer filtering.
     */
    write_reg(
        dev,
        REG_ACCEL_CONFIG2,
        0x00
    );

    /*
     * I2C bypass
     */
    write_reg(
        dev,
        REG_INT_PIN_CFG,
        0x40
    );

    /*
     * No interrupts
     */
    write_reg(
        dev,
        REG_INT_ENABLE,
        0x00
    );

    vTaskDelay(
        pdMS_TO_TICKS(20)
    );
}

/* ============================================================
 * Configuration readback
 * ============================================================ */

static void configuration_dump(
    i2c_master_dev_handle_t dev)
{
    printf(
        "\nCommon configuration readback:\n"
    );

    show_reg(
        dev,
        REG_PWR_MGMT_1,
        "PWR_MGMT_1"
    );

    show_reg(
        dev,
        REG_PWR_MGMT_2,
        "PWR_MGMT_2"
    );

    show_reg(
        dev,
        REG_CONFIG,
        "CONFIG"
    );

    show_reg(
        dev,
        REG_SMPLRT_DIV,
        "SMPLRT_DIV"
    );

    show_reg(
        dev,
        REG_GYRO_CONFIG,
        "GYRO_CONFIG"
    );

    show_reg(
        dev,
        REG_ACCEL_CONFIG,
        "ACCEL_CONFIG"
    );

    show_reg(
        dev,
        REG_ACCEL_CONFIG2,
        "ACCEL_CONFIG2"
    );

    show_reg(
        dev,
        REG_INT_PIN_CFG,
        "INT_PIN_CFG"
    );

    show_reg(
        dev,
        REG_INT_ENABLE,
        "INT_ENABLE"
    );
}

/* ============================================================
 * MPU6515-specific fingerprint
 * ============================================================ */

static void mpu6515_fingerprint(
    i2c_master_dev_handle_t dev)
{
    printf("\n");
    printf("============================================================\n");
    printf("MPU6500 / MPU6515 EXTENDED FINGERPRINT\n");
    printf("============================================================\n");

    printf(
        "\nSelf-test / factory-related registers:\n"
    );

    show_reg(
        dev,
        REG_SELF_TEST_X_GYRO,
        "XG_ST_DATA"
    );

    show_reg(
        dev,
        REG_SELF_TEST_Y_GYRO,
        "YG_ST_DATA"
    );

    show_reg(
        dev,
        REG_SELF_TEST_Z_GYRO,
        "ZG_ST_DATA"
    );

    show_reg(
        dev,
        REG_SELF_TEST_X_ACCEL,
        "XA_ST_DATA"
    );

    show_reg(
        dev,
        REG_SELF_TEST_Y_ACCEL,
        "YA_ST_DATA"
    );

    show_reg(
        dev,
        REG_SELF_TEST_Z_ACCEL,
        "ZA_ST_DATA"
    );

    show_reg(
        dev,
        REG_PRODUCT_ID,
        "PRODUCT_ID"
    );

    printf(
        "\nFactory offset / extended registers:\n"
    );

    show_reg(
        dev,
        REG_XG_OFFS_USRH,
        "XG_OFFS_USRH"
    );

    show_reg(
        dev,
        REG_XG_OFFS_USRL,
        "XG_OFFS_USRL"
    );

    show_reg(
        dev,
        REG_YG_OFFS_USRH,
        "YG_OFFS_USRH"
    );

    show_reg(
        dev,
        REG_YG_OFFS_USRL,
        "YG_OFFS_USRL"
    );

    show_reg(
        dev,
        REG_ZG_OFFS_USRH,
        "ZG_OFFS_USRH"
    );

    show_reg(
        dev,
        REG_ZG_OFFS_USRL,
        "ZG_OFFS_USRL"
    );

    printf(
        "\nAccelerometer offset registers:\n"
    );

    show_reg(
        dev,
        REG_ACCEL_XOFFS_H,
        "XA_OFFS_H"
    );

    show_reg(
        dev,
        REG_ACCEL_YOFFS_H,
        "YA_OFFS_H"
    );

    show_reg(
        dev,
        REG_ACCEL_ZOFFS_H,
        "ZA_OFFS_H"
    );
}

/* ============================================================
 * Reset + identity
 * ============================================================ */

static void reset_identity_test(
    i2c_master_dev_handle_t dev)
{
    printf("\n");
    printf("============================================================\n");
    printf("RESET + WHO_AM_I TEST\n");
    printf("============================================================\n");

    uint8_t before =
        read8(
            dev,
            REG_WHO_AM_I
        );

    printf(
        "WHO_AM_I before reset = 0x%02X (%s)\n",
        before,
        identify(before)
    );

    printf(
        "Writing PWR_MGMT_1 = 0x80...\n"
    );

    esp_err_t err =
        write_reg(
            dev,
            REG_PWR_MGMT_1,
            0x80
        );

    if (err != ESP_OK) {

        printf(
            "Reset write failed: %s\n",
            esp_err_to_name(err)
        );

        return;
    }

    vTaskDelay(
        pdMS_TO_TICKS(100)
    );

    uint8_t pwr =
        read8(
            dev,
            REG_PWR_MGMT_1
        );

    uint8_t after =
        read8(
            dev,
            REG_WHO_AM_I
        );

    printf(
        "PWR_MGMT_1 after reset = 0x%02X\n",
        pwr
    );

    printf(
        "WHO_AM_I after reset   = 0x%02X (%s)\n",
        after,
        identify(after)
    );

    if (after == before) {

        printf(
            "WHO_AM_I stability: PASS\n"
        );

    } else {

        printf(
            "WHO_AM_I stability: FAIL\n"
        );
    }
}

/* ============================================================
 * Data test
 * ============================================================ */

static void data_test(
    i2c_master_dev_handle_t dev)
{
    printf("\n");
    printf("============================================================\n");
    printf("RAW SENSOR DATA TEST\n");
    printf("============================================================\n");

    for (int i = 0; i < 10; i++) {

        imu_raw_t raw;

        esp_err_t err =
            read_raw(
                dev,
                &raw
            );

        if (err != ESP_OK) {

            printf(
                "Sample %d ERROR: %s\n",
                i + 1,
                esp_err_to_name(err)
            );

        } else {

            printf(
                "Sample %02d: "
                "A[%6d %6d %6d] "
                "G[%6d %6d %6d] "
                "T[%6d]\n",

                i + 1,

                raw.ax,
                raw.ay,
                raw.az,

                raw.gx,
                raw.gy,
                raw.gz,

                raw.temp
            );
        }

        vTaskDelay(
            pdMS_TO_TICKS(100)
        );
    }
}

/* ============================================================
 * Device test
 * ============================================================ */

static void test_device(
    i2c_master_dev_handle_t dev,
    uint8_t addr)
{
    printf("\n\n");
    printf(
        "################################################################\n"
    );

    printf(
        "### DEVICE TEST — 0x%02X\n",
        addr
    );

    printf(
        "################################################################\n"
    );

    uint8_t who =
        read8(
            dev,
            REG_WHO_AM_I
        );

    printf(
        "\nWHO_AM_I = 0x%02X\n",
        who
    );

    printf(
        "Detected identity = %s\n",
        identify(who)
    );

    if (who == WHOAMI_MPU6515) {

        printf(
            "\n*** MPU6515 ID DETECTED ***\n"
        );

        printf(
            "This device matches the MPU6515 WHO_AM_I value.\n"
        );
    }

    reset_identity_test(
        dev
    );

    configure_common(
        dev
    );

    configuration_dump(
        dev
    );

    mpu6515_fingerprint(
        dev
    );

    data_test(
        dev
    );

    printf("\n");
    printf(
        "################################################################\n"
    );

    printf(
        "### END DEVICE TEST — 0x%02X\n",
        addr
    );

    printf(
        "################################################################\n"
    );
}

/* ============================================================
 * Main
 * ============================================================ */

void app_main(void)
{
    printf("\n");
    printf(
        "============================================================\n"
    );

    printf(
        "OMNIS REV 2.0\n"
        "MPU6050 / MPU6500 / MPU6515 CONFIRMATION TEST\n"
    );

    printf(
        "============================================================\n"
    );

    printf(
        "SDA       = GPIO8\n"
        "SCL       = GPIO9\n"
        "I2C       = 400 kHz\n"
        "IMU-A     = 0x68\n"
        "IMU-B     = 0x69\n"
    );

    printf(
        "============================================================\n"
    );

    /* --------------------------------------------------------
     * I2C bus
     * -------------------------------------------------------- */

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,

        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,

        .clk_source = I2C_CLK_SRC_RC_FAST,

        .flags = {
            .enable_internal_pullup = false,
            .allow_pd = false,
        },
    };

    i2c_master_bus_handle_t bus = NULL;

    esp_err_t err =
        i2c_new_master_bus(
            &bus_cfg,
            &bus
        );

    if (err != ESP_OK) {

        printf(
            "I2C bus creation failed: %s\n",
            esp_err_to_name(err)
        );

        return;
    }

    printf(
        "I2C bus created successfully\n"
    );

    /* --------------------------------------------------------
     * IMU A
     * -------------------------------------------------------- */

    i2c_device_config_t dev_cfg_a = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_A_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    i2c_master_dev_handle_t imu_a = NULL;

    err =
        i2c_master_bus_add_device(
            bus,
            &dev_cfg_a,
            &imu_a
        );

    if (err != ESP_OK) {

        printf(
            "0x68 device creation failed: %s\n",
            esp_err_to_name(err)
        );

        return;
    }

    /* --------------------------------------------------------
     * IMU B
     * -------------------------------------------------------- */

    i2c_device_config_t dev_cfg_b = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_B_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    i2c_master_dev_handle_t imu_b = NULL;

    err =
        i2c_master_bus_add_device(
            bus,
            &dev_cfg_b,
            &imu_b
        );

    if (err != ESP_OK) {

        printf(
            "0x69 device creation failed: %s\n",
            esp_err_to_name(err)
        );

        return;
    }

    /* --------------------------------------------------------
     * Probe
     * -------------------------------------------------------- */

    printf("\n");
    printf(
        "I2C ADDRESS PROBE\n"
    );

    err =
        i2c_master_probe(
            bus,
            IMU_A_ADDR,
            I2C_TIMEOUT_MS
        );

    printf(
        "  0x68 = %s\n",
        err == ESP_OK ?
            "ACK" :
            esp_err_to_name(err)
    );

    err =
        i2c_master_probe(
            bus,
            IMU_B_ADDR,
            I2C_TIMEOUT_MS
        );

    printf(
        "  0x69 = %s\n",
        err == ESP_OK ?
            "ACK" :
            esp_err_to_name(err)
    );

    /* --------------------------------------------------------
     * 0x68 first
     * -------------------------------------------------------- */

    test_device(
        imu_a,
        IMU_A_ADDR
    );

    /* --------------------------------------------------------
     * 0x69 reference
     * -------------------------------------------------------- */

    test_device(
        imu_b,
        IMU_B_ADDR
    );

    /* --------------------------------------------------------
     * Final
     * -------------------------------------------------------- */

    printf("\n");
    printf(
        "============================================================\n"
    );

    printf(
        "MPU FAMILY CONFIRMATION TEST COMPLETE\n"
    );

    printf(
        "============================================================\n"
    );

    printf(
        "Production OMNIS firmware was not modified.\n"
    );

    while (1) {

        vTaskDelay(
            pdMS_TO_TICKS(1000)
        );
    }
}