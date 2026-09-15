/**
 * @file    mpu6050.c
 * @brief   MPU6050 driver — implementation. See mpu6050.h and mpu6050_regs.h.
 */

#include <string.h>

#include "esp_log.h"

#include "mpu6050.h"
#include "omnis_time.h"

static const char *TAG = "mpu6050";

/* Boot-time transfers can afford to wait. Per-tick reads cannot: at 400 kHz a
 * 1+14 byte transaction is well under 1 ms, so a 5 ms timeout only fires on a
 * genuinely broken bus — and when it does, it costs a couple of ticks rather
 * than stalling the control loop indefinitely. */
#define BOOT_TIMEOUT_MS     50
#define READ_TIMEOUT_MS     5

static esp_err_t write_reg(mpu6050_t *m, uint8_t reg, uint8_t val, int timeout_ms)
{
    const uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(m->dev, buf, sizeof buf, timeout_ms);
}

static esp_err_t read_reg(mpu6050_t *m, uint8_t reg, uint8_t *out, size_t n,
                          int timeout_ms)
{
    return i2c_master_transmit_receive(m->dev, &reg, 1, out, n, timeout_ms);
}

esp_err_t mpu6050_attach(i2c_master_bus_handle_t bus, uint8_t addr,
                         uint32_t scl_speed_hz, mpu6050_t *m)
{
    memset(m, 0, sizeof *m);
    m->addr = addr;

    esp_err_t err = i2c_master_probe(bus, addr, BOOT_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "0x%02X: no ACK (%s) - check wiring, AD0 level and power",
                 addr, esp_err_to_name(err));
        return err;
    }

    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = scl_speed_hz,
    };
    return i2c_master_bus_add_device(bus, &cfg, &m->dev);
}

typedef struct {
    uint8_t     reg;
    uint8_t     val;
    const char *name;
} reg_write_t;

esp_err_t mpu6050_configure(mpu6050_t *m)
{
    esp_err_t err;

    /* 1. Reset everything to power-on defaults, so a warm reboot of the ESP32
     *    does not inherit whatever a previous firmware left configured. */
    err = write_reg(m, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_DEVICE_RESET, BOOT_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "0x%02X: reset write failed: %s", m->addr, esp_err_to_name(err));
        return err;
    }
    omnis_delay_ms(100);   /* datasheet: reset takes up to 100 ms */

    /* 2. Identity. */
    err = read_reg(m, MPU6050_REG_WHO_AM_I, &m->who_am_i, 1, BOOT_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "0x%02X: WHO_AM_I read failed: %s", m->addr, esp_err_to_name(err));
        return err;
    }
    if (!mpu6050_who_am_i_compatible(m->who_am_i)) {
        ESP_LOGE(TAG, "0x%02X: WHO_AM_I = 0x%02X is not an MPU6050-compatible part",
                 m->addr, m->who_am_i);
        return ESP_ERR_NOT_FOUND;
    }
    if (m->who_am_i != MPU6050_WHO_AM_I_GENUINE) {
        ESP_LOGW(TAG, "0x%02X: WHO_AM_I = 0x%02X - not a genuine MPU6050 (common "
                      "on GY-521 clones). Register-compatible for everything used "
                      "here; continuing.", m->addr, m->who_am_i);
    }

    /* 3. Wake, clocked from the X-gyro PLL. The PLL needs tens of ms to lock. */
    err = write_reg(m, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_CLKSEL_PLL_XGYRO, BOOT_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    omnis_delay_ms(50);

    /* 4. Configuration, each write verified by read-back.
     *
     * INT_PIN_CFG goes FIRST. On board Rev 2.0 both INT pins share GPIO7, and
     * the chip powers up push-pull. Making it open-drain before anything else
     * shortens the window in which two push-pull outputs share a wire. (After
     * reset the interrupt is disabled and both idle at the same level, so the
     * window is benign — but there is no reason to leave it open longer.) */
    const reg_write_t seq[] = {
        { MPU6050_REG_INT_PIN_CFG,  MPU6050_INT_PIN_OPEN_DRAIN_LOW, "INT_PIN_CFG"  },
        { MPU6050_REG_INT_ENABLE,   MPU6050_INT_ENABLE_NONE,        "INT_ENABLE"   },
        { MPU6050_REG_CONFIG,       MPU6050_CONFIG_DLPF_94HZ,       "CONFIG"       },
        { MPU6050_REG_SMPLRT_DIV,   MPU6050_SMPLRT_DIV_500HZ,       "SMPLRT_DIV"   },
        { MPU6050_REG_GYRO_CONFIG,  MPU6050_GYRO_FS_500DPS,         "GYRO_CONFIG"  },
        { MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_4G,            "ACCEL_CONFIG" },
    };

    for (size_t i = 0; i < sizeof seq / sizeof seq[0]; ++i) {
        err = write_reg(m, seq[i].reg, seq[i].val, BOOT_TIMEOUT_MS);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "0x%02X: write %s failed: %s", m->addr, seq[i].name,
                     esp_err_to_name(err));
            return err;
        }
        uint8_t back = 0;
        err = read_reg(m, seq[i].reg, &back, 1, BOOT_TIMEOUT_MS);
        if (err != ESP_OK) {
            return err;
        }
        if (back != seq[i].val) {
            ESP_LOGE(TAG, "0x%02X: %s read back 0x%02X, wrote 0x%02X", m->addr,
                     seq[i].name, back, seq[i].val);
            return ESP_ERR_INVALID_RESPONSE;
        }
    }

    /* 5. Let the new DLPF setting settle before anyone trusts a sample. */
    omnis_delay_ms(50);

    m->configured         = true;
    m->consecutive_errors = 0;
    ESP_LOGI(TAG, "0x%02X: configured (WHO_AM_I 0x%02X, +-500 dps, +-4 g, "
                  "DLPF 94 Hz, 500 Hz, INT open-drain)", m->addr, m->who_am_i);
    return ESP_OK;
}

esp_err_t mpu6050_read_raw(mpu6050_t *m, uint8_t burst[MPU6050_BURST_LEN])
{
    return read_reg(m, MPU6050_REG_ACCEL_XOUT_H, burst, MPU6050_BURST_LEN,
                    READ_TIMEOUT_MS);
}

esp_err_t mpu6050_read(mpu6050_t *m, mpu6050_sample_t *out)
{
    uint8_t burst[MPU6050_BURST_LEN];

    esp_err_t err = mpu6050_read_raw(m, burst);
    if (err == ESP_OK && !mpu6050_burst_plausible(burst)) {
        err = ESP_ERR_INVALID_RESPONSE;
    }
    if (err != ESP_OK) {
        ++m->read_errors;
        ++m->consecutive_errors;
        return err;
    }

    /* Frozen-data detection. A live MPU6050's noise floor guarantees the 14
     * bytes change every sample; a latched or reset-looping part repeats them.
     * With the shared INT line this is one of only two ways to notice a dead
     * IMU at all. */
    if (memcmp(burst, m->last_burst, MPU6050_BURST_LEN) == 0) {
        ++m->identical_bursts;
    } else {
        m->identical_bursts = 0;
        memcpy(m->last_burst, burst, MPU6050_BURST_LEN);
    }

    m->consecutive_errors = 0;
    mpu6050_parse_burst(burst, out);
    return ESP_OK;
}
