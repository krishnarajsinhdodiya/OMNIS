/**
 * @file    imu.c
 * @brief   Both IMUs: bring-up, calibration, per-tick fusion, frames.
 *
 * See imu.h for the data path and why its order is fixed.
 */

#include <math.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "imu.h"
#include "mpu6050.h"
#include "omnis_config.h"
#include "omnis_pins.h"
#include "omnis_time.h"
#include "tick.h"

static const char *TAG = "imu";

#define IMU_COUNT   2
#define RAD2DEG     57.29577951f

typedef struct {
    mpu6050_t      dev;
    imu_mount_t    mount;          /**< sensor -> body                          */
    attitude_ekf_t ekf;
    float          bias[3];        /**< body-frame gyro bias from calibration   */
    float          accel_body[3];  /**< last accel, body frame                  */
    float          gyro_body[3];   /**< last bias-corrected gyro, body frame     */
    float          temp_c;
    int64_t        last_us;        /**< time of this unit's last filter step    */
    bool           have_sample;    /**< filter seeded and running               */
} imu_unit_t;

static i2c_master_bus_handle_t s_bus    = NULL;
static imu_unit_t              s_unit[IMU_COUNT];
static imu_frame_t             s_frame  = IMU_FRAME_FLAT;
static const omnis_params_t   *s_params = NULL;
static uint32_t                s_update_count = 0;

static const uint8_t     k_addr[IMU_COUNT] = { I2C_ADDR_IMU_A, I2C_ADDR_IMU_B };
static const char *const k_name[IMU_COUNT] = { "A", "B" };

/* ------------------------------------------------------------------------
 * Frames
 * ------------------------------------------------------------------------ */
const char *imu_frame_name(imu_frame_t frame)
{
    switch (frame) {
        case IMU_FRAME_BALANCE_FRONT: return "balance-front";
        case IMU_FRAME_BALANCE_REAR:  return "balance-rear";
        case IMU_FRAME_FLAT:
        default:                      return "flat";
    }
}

imu_frame_t imu_frame_for_pair(omnis_balance_pair_t pair)
{
    return (pair == OMNIS_BALANCE_ON_FRONT_PAIR) ? IMU_FRAME_BALANCE_FRONT
                                                 : IMU_FRAME_BALANCE_REAR;
}

/* Body frame -> the active estimation frame. */
static void to_frame(const float v_body[3], float out[3])
{
    if (s_frame == IMU_FRAME_FLAT) {
        out[0] = v_body[0];
        out[1] = v_body[1];
        out[2] = v_body[2];
        return;
    }
    const imu_mount_t m = omnis_balance_frame_for_pair(
        s_frame == IMU_FRAME_BALANCE_FRONT ? OMNIS_BALANCE_ON_FRONT_PAIR
                                           : OMNIS_BALANCE_ON_REAR_PAIR);
    imu_apply_mount(&m, v_body, out);
}

/* ------------------------------------------------------------------------
 * Bring-up
 * ------------------------------------------------------------------------ */
bool imu_init(const omnis_params_t *p)
{
    s_params = p;
    memset(s_unit, 0, sizeof s_unit);

    if (!omnis_imu_mounting_selfcheck()) {
        /* A mis-typed descriptor produces a plausible-looking but wrong attitude,
         * which is far harder to find later than a refusal to boot now. */
        ESP_LOGE(TAG, "omnis_imu_mounting.h holds an invalid or mirrored descriptor");
        return false;
    }
    s_unit[0].mount = OMNIS_IMU_A_MOUNT;
    s_unit[1].mount = OMNIS_IMU_B_MOUNT;

    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = -1,
        .sda_io_num        = PIN_I2C_SDA,
        .scl_io_num        = PIN_I2C_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        /* The GY-521 modules carry 4.7 k pull-ups; the ~45 k internal ones in
         * parallel change nothing electrically but keep the bus defined if the
         * module pull-ups are lifted as the Rev 2.0 plan suggests. */
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return false;
    }

    bool ok = true;
    for (int i = 0; i < IMU_COUNT; ++i) {
        err = mpu6050_attach(s_bus, k_addr[i], OMNIS_I2C_SCL_HZ, &s_unit[i].dev);
        if (err == ESP_OK) {
            err = mpu6050_configure(&s_unit[i].dev);
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "IMU %s (0x%02X) unavailable: %s", k_name[i], k_addr[i],
                     esp_err_to_name(err));
            ok = false;
            continue;
        }
        attitude_ekf_init(&s_unit[i].ekf, NULL);
        ESP_LOGI(TAG, "IMU %s (0x%02X) ready, mount {%+d,%+d,%+d}", k_name[i], k_addr[i],
                 s_unit[i].mount.map[0], s_unit[i].mount.map[1], s_unit[i].mount.map[2]);
    }
    return ok;
}

/* ------------------------------------------------------------------------
 * Calibration (omnis-info.md §12 step 1)
 * ------------------------------------------------------------------------ */
typedef struct {
    double   g_sum[3];
    double   g_sq[3];
    double   a_sum[3];
    double   mag_sum;
    double   mag_sq;
    uint32_t n;
    uint32_t errors;
} cal_acc_t;

static float std_of(double sum, double sq, uint32_t n)
{
    if (n == 0u) {
        return 0.0f;
    }
    const double mean = sum / (double)n;
    const double var  = sq / (double)n - mean * mean;
    return var > 0.0 ? (float)sqrt(var) : 0.0f;   /* rounding can go slightly negative */
}

bool imu_calibrate(const omnis_params_t *p, void (*on_tick)(void))
{
    const omnis_imu_params_t *ip = &p->imu;

    for (uint8_t attempt = 1; attempt <= ip->cal_attempts; ++attempt) {
        ESP_LOGI(TAG, "gyro calibration %u/%u: keep the robot STILL for %.1f s",
                 attempt, ip->cal_attempts, (double)ip->cal_samples / (double)TICK_RATE_HZ);

        cal_acc_t acc[IMU_COUNT];
        memset(acc, 0, sizeof acc);

        for (uint32_t k = 0; k < ip->cal_samples; ++k) {
            (void)tick_wait();
            if (on_tick != NULL) {
                on_tick();
            }
            for (int i = 0; i < IMU_COUNT; ++i) {
                mpu6050_sample_t s;
                if (mpu6050_read(&s_unit[i].dev, &s) != ESP_OK) {
                    ++acc[i].errors;
                    continue;
                }
                float a[3], g[3];
                imu_apply_mount(&s_unit[i].mount, s.accel_g, a);
                imu_apply_mount(&s_unit[i].mount, s.gyro_radps, g);

                cal_acc_t *c = &acc[i];
                double mag = 0.0;
                for (int ax = 0; ax < 3; ++ax) {
                    c->g_sum[ax] += g[ax];
                    c->g_sq[ax]  += (double)g[ax] * g[ax];
                    c->a_sum[ax] += a[ax];
                    mag          += (double)a[ax] * a[ax];
                }
                mag = sqrt(mag);
                c->mag_sum += mag;
                c->mag_sq  += mag * mag;
                ++c->n;
                s_unit[i].temp_c = s.temp_c;
            }
        }

        bool ok = true;
        for (int i = 0; i < IMU_COUNT; ++i) {
            const cal_acc_t *c = &acc[i];
            if (c->n < (ip->cal_samples * 9u) / 10u) {
                ESP_LOGW(TAG, "  IMU %s: only %u of %u reads succeeded (%u errors)",
                         k_name[i], (unsigned)c->n, (unsigned)ip->cal_samples,
                         (unsigned)c->errors);
                ok = false;
                continue;
            }
            float gyro_sd_max = 0.0f;
            for (int ax = 0; ax < 3; ++ax) {
                const float sd = std_of(c->g_sum[ax], c->g_sq[ax], c->n);
                if (sd > gyro_sd_max) gyro_sd_max = sd;
            }
            const float mag_mean = (float)(c->mag_sum / c->n);
            const float mag_sd   = std_of(c->mag_sum, c->mag_sq, c->n);

            ESP_LOGI(TAG, "  IMU %s: bias (%+.4f %+.4f %+.4f) rad/s | gyro sd %.4f | "
                          "|a| %.3f g sd %.4f | %.1f C", k_name[i],
                     c->g_sum[0] / c->n, c->g_sum[1] / c->n, c->g_sum[2] / c->n,
                     (double)gyro_sd_max, (double)mag_mean, (double)mag_sd,
                     (double)s_unit[i].temp_c);

            if (gyro_sd_max > ip->cal_gyro_std_max_radps) {
                ESP_LOGW(TAG, "  IMU %s: gyro not still (sd %.4f > %.4f rad/s)", k_name[i],
                         (double)gyro_sd_max, (double)ip->cal_gyro_std_max_radps);
                ok = false;
            }
            if (fabsf(mag_mean - 1.0f) > ip->cal_accel_tol_g) {
                /* Not a motion problem: a stationary accelerometer must read 1 g.
                 * This is a wrong full-scale setting or a failing sensor. */
                ESP_LOGW(TAG, "  IMU %s: |a| = %.3f g at rest, expected 1.000", k_name[i],
                         (double)mag_mean);
                ok = false;
            }
            if (mag_sd > ip->cal_accel_std_max_g) {
                ESP_LOGW(TAG, "  IMU %s: accelerometer not still (sd %.4f g)", k_name[i],
                         (double)mag_sd);
                ok = false;
            }
        }

        if (!ok) {
            continue;
        }

        const int64_t now = esp_timer_get_time();
        s_frame = IMU_FRAME_FLAT;
        for (int i = 0; i < IMU_COUNT; ++i) {
            imu_unit_t *u = &s_unit[i];
            for (int ax = 0; ax < 3; ++ax) {
                u->bias[ax]       = (float)(acc[i].g_sum[ax] / acc[i].n);
                u->accel_body[ax] = (float)(acc[i].a_sum[ax] / acc[i].n);
                u->gyro_body[ax]  = 0.0f;
            }
            /* Bias is subtracted before the filter, so the EKF's own bias
             * states start at zero and only track residual drift. */
            attitude_ekf_init(&u->ekf, NULL);
            attitude_ekf_seed_from_accel(&u->ekf, u->accel_body[0], u->accel_body[1],
                                         u->accel_body[2]);
            u->last_us     = now;
            u->have_sample = true;
        }
        ESP_LOGI(TAG, "calibration done");
        return true;
    }

    ESP_LOGE(TAG, "calibration failed after %u attempts", (unsigned)ip->cal_attempts);
    return false;
}

/* ------------------------------------------------------------------------
 * Per tick
 * ------------------------------------------------------------------------ */
static bool read_unit(int i, int64_t now_us)
{
    imu_unit_t *u = &s_unit[i];
    mpu6050_sample_t s;

    if (mpu6050_read(&u->dev, &s) != ESP_OK) {
        /* No step this tick. The next good read uses the full elapsed dt, so
         * the gyro integration spans the gap rather than losing it. */
        return false;
    }

    imu_apply_mount(&u->mount, s.accel_g, u->accel_body);
    float g_body[3];
    imu_apply_mount(&u->mount, s.gyro_radps, g_body);
    for (int ax = 0; ax < 3; ++ax) {
        u->gyro_body[ax] = g_body[ax] - u->bias[ax];
    }
    u->temp_c = s.temp_c;

    float a[3], g[3];
    to_frame(u->accel_body, a);
    to_frame(u->gyro_body, g);

    if (!u->have_sample) {
        attitude_ekf_seed_from_accel(&u->ekf, a[0], a[1], a[2]);
        u->have_sample = true;
        u->last_us     = now_us;
        return true;
    }

    /* Measured dt, not the nominal tick period: correct after an overrun, and
     * correct when OMNIS_IMU_READ_ALTERNATE halves this unit's rate. Clamped so
     * a long stall cannot integrate a wild angle in one step. */
    float dt = (float)(now_us - u->last_us) * 1e-6f;
    if (dt < 1e-4f) dt = 1e-4f;
    if (dt > 0.05f) dt = 0.05f;

    attitude_ekf_step(&u->ekf, g[0], g[1], a[0], a[1], a[2], dt);
    u->last_us = now_us;
    return true;
}

static bool unit_comm_failed(const imu_unit_t *u, const omnis_imu_params_t *ip)
{
    return u->dev.consecutive_errors >= ip->comm_fail_reads
        || u->dev.identical_bursts   >= ip->frozen_reads;
}

void imu_update(int64_t now_us, imu_state_t *out)
{
    const omnis_imu_params_t *ip = &s_params->imu;
    const int64_t t0 = esp_timer_get_time();

#if OMNIS_IMU_READ_ALTERNATE
    const int which = (int)(s_update_count & 1u);
    (void)read_unit(which, now_us);
#else
    (void)read_unit(0, now_us);
    (void)read_unit(1, now_us);
#endif
    ++s_update_count;

    out->read_us = (uint32_t)(esp_timer_get_time() - t0);
    out->frame   = s_frame;

    out->fault_comm = unit_comm_failed(&s_unit[0], ip) || unit_comm_failed(&s_unit[1], ip);

    const bool both = s_unit[0].have_sample && s_unit[1].have_sample;
    imu_fusion_result_t r;
    imu_fusion_combine(s_unit[0].have_sample ? &s_unit[0].ekf : NULL,
                       s_unit[1].have_sample ? &s_unit[1].ekf : NULL,
                       ip->disagree_thresh_rad, &r);

    out->roll         = r.roll;
    out->pitch        = r.pitch;
    out->var_roll     = r.var_roll;
    out->var_pitch    = r.var_pitch;
    out->disagreement = r.disagreement;
    out->fault_disagree = both && r.fault;
    out->valid        = both && !out->fault_comm;

    float g0[3], g1[3], a0[3], a1[3];
    to_frame(s_unit[0].gyro_body, g0);
    to_frame(s_unit[1].gyro_body, g1);
    to_frame(s_unit[0].accel_body, a0);
    to_frame(s_unit[1].accel_body, a1);
    for (int ax = 0; ax < 3; ++ax) {
        out->gyro[ax]  = 0.5f * (g0[ax] + g1[ax]);
        out->accel[ax] = 0.5f * (a0[ax] + a1[ax]);
    }
}

void imu_set_frame(imu_frame_t frame)
{
    if (frame == s_frame) {
        return;
    }
    s_frame = frame;

    for (int i = 0; i < IMU_COUNT; ++i) {
        imu_unit_t *u = &s_unit[i];
        if (!u->have_sample) {
            continue;
        }
        float a[3];
        to_frame(u->accel_body, a);
        attitude_ekf_init(&u->ekf, NULL);
        attitude_ekf_seed_from_accel(&u->ekf, a[0], a[1], a[2]);
    }
    ESP_LOGI(TAG, "estimation frame -> %s (filters re-seeded)", imu_frame_name(frame));
}

void imu_get_diag(imu_diag_t *out)
{
    for (int i = 0; i < IMU_COUNT; ++i) {
        out->who_am_i[i]    = s_unit[i].dev.who_am_i;
        out->read_errors[i] = s_unit[i].dev.read_errors;
        out->temp_c[i]      = s_unit[i].temp_c;
        for (int ax = 0; ax < 3; ++ax) {
            out->bias[i][ax] = s_unit[i].bias[ax];
        }
    }
}

/* ------------------------------------------------------------------------
 * Mount wizard (OMNIS_RUN_MOUNT_WIZARD)
 * ------------------------------------------------------------------------ */
static bool average_raw_accel(int samples, float out[IMU_COUNT][3])
{
    double sum[IMU_COUNT][3] = { { 0 } };
    int    got[IMU_COUNT]    = { 0 };

    for (int k = 0; k < samples; ++k) {
        for (int i = 0; i < IMU_COUNT; ++i) {
            mpu6050_sample_t s;
            if (mpu6050_read(&s_unit[i].dev, &s) == ESP_OK) {
                for (int ax = 0; ax < 3; ++ax) sum[i][ax] += s.accel_g[ax];
                ++got[i];
            }
        }
        omnis_delay_ms(2);
    }
    for (int i = 0; i < IMU_COUNT; ++i) {
        if (got[i] < samples / 2) {
            ESP_LOGE(TAG, "IMU %s: too many read errors while sampling", k_name[i]);
            return false;
        }
        for (int ax = 0; ax < 3; ++ax) out[i][ax] = (float)(sum[i][ax] / got[i]);
    }
    return true;
}

static void countdown(const char *what, int seconds)
{
    ESP_LOGW(TAG, "%s", what);
    for (int s = seconds; s > 0; --s) {
        ESP_LOGW(TAG, "  sampling in %d ...", s);
        omnis_delay_ms(1000);
    }
    ESP_LOGW(TAG, "  sampling - HOLD STILL");
}

void imu_run_mount_wizard(void)
{
    ESP_LOGW(TAG, "==============================================================");
    ESP_LOGW(TAG, " IMU MOUNT WIZARD (OMNIS_RUN_MOUNT_WIZARD = 1)");
    ESP_LOGW(TAG, " Derives each IMU's mounting from two poses. No arrow-reading.");
    ESP_LOGW(TAG, " The robot will not drive in this mode.");
    ESP_LOGW(TAG, "==============================================================");

    float level[IMU_COUNT][3], nose[IMU_COUNT][3];

    countdown("POSE 1: chassis FLAT and LEVEL on the bench (OLED side up).", 8);
    if (!average_raw_accel(400, level)) goto halt;

    countdown("POSE 2: tip the chassis NOSE-DOWN (front edge lowered) 30-60 deg and hold.", 8);
    if (!average_raw_accel(400, nose)) goto halt;

    ESP_LOGW(TAG, "RESULTS (paste into omnis_imu_mounting.h if they differ):");
    const imu_mount_t expected[IMU_COUNT] = { OMNIS_IMU_A_MOUNT, OMNIS_IMU_B_MOUNT };
    for (int i = 0; i < IMU_COUNT; ++i) {
        imu_mount_t m;
        ESP_LOGW(TAG, "  IMU %s raw level (%+.2f %+.2f %+.2f) nose-down (%+.2f %+.2f %+.2f)",
                 k_name[i], (double)level[i][0], (double)level[i][1], (double)level[i][2],
                 (double)nose[i][0], (double)nose[i][1], (double)nose[i][2]);
        if (!imu_mount_resolve(level[i], nose[i], &m)) {
            ESP_LOGE(TAG, "  IMU %s (0x%02X): could not resolve - was pose 1 level, and "
                          "pose 2 tipped at least ~15 deg?", k_name[i], k_addr[i]);
            continue;
        }
        const bool same = m.map[0] == expected[i].map[0] && m.map[1] == expected[i].map[1]
                       && m.map[2] == expected[i].map[2];
        ESP_LOGW(TAG, "  #define OMNIS_IMU_%s_MOUNT ((imu_mount_t){ { %+d, %+d, %+d } })   %s",
                 k_name[i], m.map[0], m.map[1], m.map[2],
                 same ? "<- matches the current header" : "<- DIFFERS: update the header");
    }

halt:
    ESP_LOGW(TAG, "wizard finished. Set OMNIS_RUN_MOUNT_WIZARD back to 0 and reflash.");
    for (;;) {
        omnis_delay_ms(1000);
    }
}
