/**
 * @file    main.c
 * @brief   OMNIS superloop MVP — boot sequence and the superloop.
 *
 * ARCHITECTURE (BUILD-LOG.md Planning 1a). One while(1) inside app_main, paced by a 500 Hz
 * GPTimer ISR that does nothing but set a flag. No xTaskCreate, queues,
 * semaphores, mutexes or vTaskDelay anywhere in OMNIS-authored code; ESP-IDF
 * driver internals are treated as part of the hardware. app_main is pinned to
 * CPU1 with CPU1's idle-task watchdog disabled (sdkconfig.defaults), which is
 * what makes a never-yielding loop legal on this chip.
 *
 * BOOT NEVER HALTS ON A PERIPHERAL PROBLEM. A missing IMU, a failed calibration
 * or an RMT failure each become a latched fault (fault.h): the superloop still
 * runs, the buzzer says what is wrong, telemetry prints it, and the supervisor
 * refuses to arm. Only untrustworthy parameters — which everything downstream
 * divides by — or a tick that will not start drop into fault_only_loop().
 *
 * EACH TICK, in this order:
 *   1. sense    IMUs (read, filter, fuse) and radio
 *   2. judge    faults (IMU, link, tilt, overrun), then the supervisor
 *   3. act      flat:    drive pipeline -> acceleration limit -> STEP pulses
 *               balance: lean PID in the balance frame -> STEP pulses
 *               otherwise stop
 *   4. report   buzzer every tick, telemetry once a second
 *
 * Every fault check runs before the motor command on every tick.
 */

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "balance.h"
#include "buzzer.h"
#include "crsf.h"
#include "drive.h"
#include "fault.h"
#include "imu.h"
#include "mecanum_kinematics.h"
#include "omnis_config.h"
#include "omnis_params.h"
#include "omnis_pins.h"
#include "rc_input.h"
#include "step_gen.h"
#include "supervisor.h"
#include "tick.h"

static const char *TAG = "omnis";

#define RAD2DEG  57.29578f

static const omnis_params_t *s_params  = NULL;
static fault_state_t          s_faults;
static supervisor_t           s_sup;
static bool                   s_step_ok = false;   /* RMT channels came up          */
static bool                   s_imu_ok  = false;   /* IMUs initialised + calibrated */
static bool                   s_crsf_ok = false;   /* UART installed                */

/* Rates actually handed to the step generator: the drive pipeline's output after
 * the acceleration limit. Kept across ticks because the slew limiter works from
 * where the wheels ARE, not from where they were last asked to be. */
static wheel_rates_t s_cmd_rates = { 0.0f, 0.0f, 0.0f, 0.0f };

/* Balance controller, plus the per-arming summary printed on disarm (telemetry
 * is silent while balancing — see OMNIS_TELEMETRY_IN_BALANCE). */
static balance_cfg_t  s_bal_cfg;
static balance_ctrl_t s_bal;
static float          s_bal_peak_lean = 0.0f;
static uint32_t       s_bal_ticks     = 0u;

/* ==========================================================================
 * Boot helpers
 * ========================================================================== */

/* Board Rev 2.0 has no stepper enable line: the A4988s are live whenever 12 V is
 * present, from before this code runs. The only thing firmware controls is
 * whether STEP pulses exist, so the STEP lines are pinned low before anything
 * else, and stay low until the RMT generator takes them over. */
static void gpio_safe_state(void)
{
    const gpio_config_t steppers = {
        .pin_bit_mask = (1ULL << PIN_FL_STEP) | (1ULL << PIN_FL_DIR)
                      | (1ULL << PIN_FR_STEP) | (1ULL << PIN_FR_DIR)
                      | (1ULL << PIN_RL_STEP) | (1ULL << PIN_RL_DIR)
                      | (1ULL << PIN_RR_STEP) | (1ULL << PIN_RR_DIR),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&steppers));
    gpio_set_level(PIN_FL_STEP, 0);  gpio_set_level(PIN_FL_DIR, 0);
    gpio_set_level(PIN_FR_STEP, 0);  gpio_set_level(PIN_FR_DIR, 0);
    gpio_set_level(PIN_RL_STEP, 0);  gpio_set_level(PIN_RL_DIR, 0);
    gpio_set_level(PIN_RR_STEP, 0);  gpio_set_level(PIN_RR_DIR, 0);

    const gpio_config_t buzzer = {
        .pin_bit_mask = (1ULL << PIN_BUZZER),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&buzzer));
    gpio_set_level(PIN_BUZZER, BUZZER_OFF);

    ESP_LOGI(TAG, "STEP/DIR held low, buzzer off. NOTE: EN# is hardwired - the "
                  "motors are energised whenever 12 V is present");
}

static void log_params(const omnis_params_t *p)
{
    /* Parallel rollers give the two axles different yaw levers, so print both.
      * A rear lever near zero is EXPECTED on a near-square frame, not a fault. */
    const mecanum_levers_t k = mecanum_yaw_levers(p->geometry.wheelbase_mm,
                                                  p->geometry.track_width_mm);
    ESP_LOGI(TAG, "params v%" PRIu32 ": r=%.1f L=%.1f W=%.1f mm | yaw levers front "
                  "%.1f rear %.1f mm (%.1f:1) | max %.0f st/s, accel %.0f st/s^2, "
                  "deadband %.0f st/s",
             p->schema_version, (double)p->geometry.wheel_radius_mm,
             (double)p->geometry.wheelbase_mm, (double)p->geometry.track_width_mm,
             (double)fabsf(k.fl), (double)fabsf(k.rl),
             (double)(k.rl != 0.0f ? fabsf(k.fl / k.rl) : INFINITY),
             (double)p->step.max_step_rate,
             (double)p->step.max_accel_steps_s2, (double)p->step.deadband_steps);
    const omnis_channel_map_t *m = &p->channel_map;
    ESP_LOGI(TAG, "channels: thr %u pitch %u roll %u yaw %u | arm %u%s mode %u speed %u pot %u",
             m->throttle, m->pitch, m->roll, m->yaw, m->kill_switch,
             m->arm_switch_invert ? " (inverted)" : "", m->drive_mode,
             m->speed_limiter, m->tune_pot);
    ESP_LOGI(TAG, "DIR invert FL %d FR %d RL %d RR %d | pulse %u us",
             p->motor.dir_invert[0], p->motor.dir_invert[1],
             p->motor.dir_invert[2], p->motor.dir_invert[3],
             (unsigned)p->motor.step_pulse_us);
}

/* The verified kinematics cases, re-checked on the Xtensa FPU at boot against
 * the REFERENCE geometry (30 / 223 / 230 mm), not the parameter set — so this
 * checks the code and the FPU, and editing the geometry cannot fake a fault. */
static bool near(float got, float want, float tol)
{
    return fabsf(got - want) <= tol;
}

static bool kinematics_selftest(void)
{
    const float R = 30.0f, L = 223.0f, W = 230.0f, T = 0.01f;
    bool ok = true;

    const wheel_rates_t a = mecanum_inverse(200.0f, 0.0f, 0.0f, R, L, W);
    ok = ok && near(a.fl, 3395.305f, T) && near(a.fr, 3395.305f, T)
            && near(a.rl, 3395.305f, T) && near(a.rr, 3395.305f, T);
    const wheel_rates_t b = mecanum_inverse(0.0f, 200.0f, 0.0f, R, L, W);
    ok = ok && near(b.fl, -3395.305f, T) && near(b.fr, 3395.305f, T)
            && near(b.rl, -3395.305f, T) && near(b.rr, 3395.305f, T);
    /* Case C is the layout's signature: under parallel rollers a pure spin
     * barely turns the rear pair (59 steps/s against 3845). An X-drive build
     * would put 3845 on all four, so this line alone catches a stale port. */
    const wheel_rates_t c = mecanum_inverse(0.0f, 0.0f, 1.0f, R, L, W);
    ok = ok && near(c.fl, -3845.183f, T) && near(c.fr, 3845.183f, T)
            && near(c.rl, -59.418f, T) && near(c.rr, 59.418f, T);
    const wheel_rates_t d = mecanum_inverse(200.0f, 100.0f, 0.5f, R, L, W);
    ok = ok && near(d.fl, -224.939f, T) && near(d.fr, 7015.550f, T)
            && near(d.rl, 1667.944f, T) && near(d.rr, 5122.667f, T);
    const body_vel_t fk = mecanum_forward(&d, R, L, W);
    ok = ok && near(fk.vx, 200.0f, 0.01f) && near(fk.vy, 100.0f, 0.01f)
            && near(fk.w, 0.5f, 1e-4f) && near(mecanum_null_space(&d), 0.0f, 0.01f);
    /* vx = vy idles the LEFT SIDE (it idled the FL/RR diagonal under X-drive),
     * and vx = -vy must idle the right. Checking both directions is what
     * distinguishes "handedness correct" from "handedness mirrored". */
    const wheel_rates_t diag = mecanum_inverse(200.0f, 200.0f, 0.0f, R, L, W);
    ok = ok && diag.fl == 0.0f && diag.rl == 0.0f;
    const wheel_rates_t anti = mecanum_inverse(200.0f, -200.0f, 0.0f, R, L, W);
    ok = ok && anti.fr == 0.0f && anti.rr == 0.0f;

    ESP_LOGI(TAG, "kinematics self-check (cases A-D, round trip, null space, side idle): %s",
             ok ? "PASS" : "*** FAIL ***");
    return ok;
}

/* Parameters or the tick are untrustworthy, so nothing that depends on them may
 * run. Keeps the buzzer going and explains itself every 2 s. Paced by the wall
 * clock rather than the tick, because the tick may be the thing that failed. */
static void fault_only_loop(const char *why)
{
    char desc[96];
    int64_t next_log = 0;
    for (;;) {
        const int64_t now = esp_timer_get_time();
        buzzer_service(s_faults.active, (uint32_t)(now / 1000));
        if (now >= next_log) {
            fault_describe(s_faults.active, desc, sizeof desc);
            ESP_LOGE(TAG, "HALTED: %s. Faults: %s. Motors cannot be armed; fix and reboot.",
                     why, desc);
            next_log = now + 2000000;
        }
    }
}

/* Serviced once per tick while the boot-time IMU calibration holds the loop, so
 * the radio is already locked, and the buzzer already talking, when it ends. */
static void boot_tick_hook(void)
{
    const int64_t now = esp_timer_get_time();
    if (s_crsf_ok) {
        crsf_poll(now);
    }
    buzzer_service(s_faults.active, (uint32_t)(now / 1000));
}

static sup_config_t sup_config_from(const omnis_params_t *p)
{
    const sup_config_t c = {
        .settle_ms         = p->safety.balance_settle_ms,
        .upright_min_rad   = p->safety.upright_min_rad,
        .flat_max_tilt_rad = p->safety.arm_flat_max_tilt_rad,
        /* Zero gains cannot balance anything (§13d; test_balance.c shows a
         * zero-gain robot simply falls), so balance arming is refused until kp is
         * set — or, in live-tuning mode, until the slider has a range to cover. */
#if OMNIS_TUNE_KP_FROM_POT
        .balance_gains_set = p->balance.tune_kp_max > 0.0f,
#else
        .balance_gains_set = p->balance.kp > 0.0f,
#endif
    };
    return c;
}

static balance_cfg_t balance_cfg_from(const omnis_params_t *p)
{
    const balance_cfg_t c = {
        .kp                 = p->balance.kp,
        .ki                 = p->balance.ki,
        .kd                 = p->balance.kd,
        .integral_max       = p->balance.integral_max,
        .max_wheel_accel    = p->balance.max_wheel_accel,
        .max_wheel_rate     = p->step.max_step_rate,
        .trim_rad           = p->balance.trim_rad,
        .stick_lean_max_rad = p->balance.stick_lean_max_rad,
        .turn_max_steps     = p->balance.turn_max_steps,
        .lean_limit_rad     = p->balance.lean_limit_rad,
        .vel_bias_gain      = p->balance.vel_bias_gain,
        .outer_period_s     = p->balance.outer_period_s,
        .output_invert      = p->balance.output_invert,
    };
    return c;
}

#if OMNIS_BENCH_STEP_TEST
/* Stage 5 bench pattern. WHEELS OFF THE GROUND — see omnis_config.h. Each segment
 * runs a body velocity through the real pipeline (IK, clamp, deadband). The
 * expected wheel behaviour is in the segment name and in TESTING.md. */
static void bench_pattern(int64_t now_us, const omnis_params_t *p, drive_solution_t *sol)
{
    typedef struct { const char *name; float vx, vy, w; uint32_t ms; } bench_seg_t;
    static const bench_seg_t segs[] = {
        { "stop (settle)",                                          0.0f,   0.0f, 0.0f, 3000u },
        { "FORWARD 200 mm/s - all four wheels forward, ~3395 Hz",  200.0f,  0.0f, 0.0f, 3000u },
        { "stop",                                                   0.0f,   0.0f, 0.0f, 1500u },
        { "BACKWARD 200 mm/s - all four wheels backward",         -200.0f,  0.0f, 0.0f, 3000u },
        { "stop",                                                   0.0f,   0.0f, 0.0f, 1500u },
        { "STRAFE LEFT 200 mm/s - LEFT pair backward; RIGHT pair forward",
                                                                    0.0f, 200.0f, 0.0f, 3000u },
        { "stop",                                                   0.0f,   0.0f, 0.0f, 1500u },
        /* Rear pair at 59 steps/s is a visible crawl, not a stall: parallel
         * rollers absorb a spin almost entirely at the rear. See TESTING.md. */
        { "ROTATE CCW 1 rad/s - FL,FR fast (3845); RL,RR CRAWL (59)",
                                                                    0.0f,   0.0f, 1.0f, 3000u },
        { "stop",                                                   0.0f,   0.0f, 0.0f, 1500u },
        { "DIAGONAL vx=vy=150 - FR,RR forward; FL,RL MUST NOT TURN", 150.0f, 150.0f, 0.0f, 3000u },
        { "stop",                                                   0.0f,   0.0f, 0.0f, 1500u },
        { "ANTI-DIAGONAL vx=-vy=150 - FL,RL forward; FR,RR MUST NOT TURN",
                                                                  150.0f, -150.0f, 0.0f, 3000u },
    };
    static int     idx       = -1;
    static int64_t seg_start = 0;
    const int      count     = (int)(sizeof segs / sizeof segs[0]);

    if (idx < 0 || now_us - seg_start >= (int64_t)segs[idx].ms * 1000) {
        idx       = (idx + 1) % count;
        seg_start = now_us;
        ESP_LOGW(TAG, "BENCH %d/%d: %s", idx + 1, count, segs[idx].name);
    }
    const body_vel_t v = { segs[idx].vx, segs[idx].vy, segs[idx].w };
    drive_solve(&v, &p->geometry, &p->step, sol);
}
#endif

/* ==========================================================================
 * Telemetry
 * ========================================================================== */
typedef struct {
    uint32_t ticks;
    int64_t  start_us;
    uint32_t min_headroom_us;
    uint32_t overruns;
    uint32_t i2c_max_us;
    uint32_t imu_invalid;
    float    worst_disagree;
} window_t;

static void window_reset(window_t *w)
{
    memset(w, 0, sizeof *w);
    w->min_headroom_us = UINT32_MAX;
}

#if OMNIS_TELEMETRY_ENABLED
static void telemetry_print(const window_t *w, int64_t now_us, const imu_state_t *imu,
                            bool link_ok, const rc_command_t *rc)
{
    /* N tick edges span N - 1 intervals, and the window starts on a tick edge
     * (the Stage 1 bench test found both off-by-ones). */
    const double hz = (w->ticks > 1u && now_us > w->start_us)
                    ? (double)(w->ticks - 1u) * 1e6 / (double)(now_us - w->start_us) : 0.0;
    const uint32_t body = (w->min_headroom_us == UINT32_MAX)
                        ? 0u : (uint32_t)TICK_PERIOD_US - w->min_headroom_us;
    char desc[96];
    fault_describe(s_faults.active, desc, sizeof desc);
    crsf_status_t cs;
    crsf_get_status(now_us, &cs);

    ESP_LOGI(TAG,
             "%.1fHz body %" PRIu32 "us ovr %" PRIu32 " | %s | faults %s"
             " | imu r%+.1f p%+.1f dis %.1f i2c %" PRIu32 "us bad %" PRIu32
             " | rc %s LQ%u thr%+.2f pit%+.2f rol%+.2f yaw%+.2f arm%d mode %s spd %s"
             " | whl %+.0f %+.0f %+.0f %+.0f",
             hz, body, w->overruns, sup_state_name(s_sup.state), desc,
             (double)(imu->roll * RAD2DEG), (double)(imu->pitch * RAD2DEG),
             (double)(w->worst_disagree * RAD2DEG), w->i2c_max_us, w->imu_invalid,
             !cs.locked ? "NONE" : (link_ok ? "OK" : "LOST"), (unsigned)cs.uplink_lq,
             (double)rc->sticks.throttle, (double)rc->sticks.pitch,
             (double)rc->sticks.roll, (double)rc->sticks.yaw,
             rc->arm_request ? 1 : 0, rc_switch_name(rc->drive_mode),
             rc_switch_name(rc->speed), (double)s_cmd_rates.fl, (double)s_cmd_rates.fr,
             (double)s_cmd_rates.rl, (double)s_cmd_rates.rr);

#if OMNIS_TUNE_KP_FROM_POT
    ESP_LOGI(TAG, "tune: slider %.2f -> kp %.0f (range 0..%.0f)", (double)rc->tune_pot,
             (double)(rc->tune_pot * s_params->balance.tune_kp_max),
             (double)s_params->balance.tune_kp_max);
#endif
#if OMNIS_BENCH_STEP_TEST
    uint32_t und = 0, fail = 0, rev = 0;
    uint64_t pulses = 0;
    for (int wh = 0; wh < STEP_WHEELS; ++wh) {
        step_gen_stats_t st;
        step_gen_get_stats(wh, &st);
        und += st.underruns; fail += st.submit_failures; rev += st.reversals; pulses += st.pulses;
    }
    ESP_LOGI(TAG, "step: pulses %llu underruns %" PRIu32 " submit-fail %" PRIu32
                  " reversals %" PRIu32, (unsigned long long)pulses, und, fail, rev);
#endif
}
#endif

/* ==========================================================================
 * app_main
 * ========================================================================== */
void app_main(void)
{
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "OMNIS superloop MVP - board Rev 2.0 - all stages");
    ESP_LOGI(TAG, "==================================================");

    /* 1. Hardware safe before anything else can move. */
    gpio_safe_state();
    fault_init(&s_faults);
    buzzer_init();
    supervisor_init(&s_sup);

    /* 2. Parameters, and the kinematics they feed. */
    omnis_params_init();
    s_params = omnis_params();
    if (!omnis_params_valid(s_params)) {
        fault_raise(&s_faults, FAULT_PARAMS);
        fault_only_loop("parameter set failed validation (omnis_params.c)");
    }
    log_params(s_params);
    if (!kinematics_selftest()) {
        fault_raise(&s_faults, FAULT_PARAMS);
        fault_only_loop("kinematics self-check failed on this FPU");
    }

    /* 3. Step generation — right after parameters, so RMT owns the STEP lines as
     *    early as possible on drivers that are already live. */
    s_step_ok = step_gen_init(s_params);
    if (!s_step_ok) {
        fault_raise(&s_faults, FAULT_STEP_INIT);
    }
#if OMNIS_BENCH_STEP_TEST
    ESP_LOGW(TAG, "*** OMNIS_BENCH_STEP_TEST = 1: WHEELS WILL TURN ~3 s AFTER BOOT, "
                  "IGNORING THE RADIO. WHEELS OFF THE GROUND. ***");
#endif

    /* 4. The tick. Registered from app_main, so its ISR lands on CPU1 with the
     *    superloop (tick.h explains why that makes a plain volatile flag enough). */
    if (!tick_init()) {
        fault_raise(&s_faults, FAULT_PARAMS);
        fault_only_loop("500 Hz tick failed to start");
    }

    /* 5. Radio first, so it is found and locked during the IMU calibration. */
    s_crsf_ok = crsf_init(s_params);
    if (!s_crsf_ok) {
        ESP_LOGE(TAG, "CRSF UART failed: no radio, so the robot can never arm");
    }

    /* 6. IMUs. */
    if (!imu_init(s_params)) {
        fault_raise(&s_faults, FAULT_IMU_INIT);
    } else {
#if OMNIS_RUN_MOUNT_WIZARD
        imu_run_mount_wizard();   /* never returns */
#endif
        buzzer_event(BUZZ_CAL_HOLD_STILL, (uint32_t)(esp_timer_get_time() / 1000));
        if (imu_calibrate(s_params, boot_tick_hook)) {
            s_imu_ok = true;
        } else {
            fault_raise(&s_faults, FAULT_IMU_CAL);
        }
        buzzer_event(BUZZ_SILENT, (uint32_t)(esp_timer_get_time() / 1000));
    }

    const sup_config_t sup_cfg = sup_config_from(s_params);
    s_bal_cfg = balance_cfg_from(s_params);
    balance_init(&s_bal, &s_bal_cfg);
    {
        char desc[96];
        fault_describe(s_faults.active, desc, sizeof desc);
        if (s_faults.active == 0u) {
            buzzer_event(BUZZ_BOOT_OK, (uint32_t)(esp_timer_get_time() / 1000));
            ESP_LOGI(TAG, "boot OK - flip the arm switch LOW then HIGH to arm");
        } else {
            ESP_LOGE(TAG, "boot finished WITH FAULTS: %s - arming is blocked", desc);
        }
    }

    /* ---------------------------------------------------------------- loop */
    imu_state_t imu;
    memset(&imu, 0, sizeof imu);
    rc_command_t     rc;
    drive_solution_t sol;
    drive_solution_zero(&sol);
    rc_command_neutral(&rc);

    window_t win;
    window_reset(&win);
    uint32_t ovr_seen          = g_tick_overruns;
    uint32_t ovr_block_count   = 0;
    uint32_t ovr_block_ticks   = 0;
    uint32_t ignore_ovr_ticks  = 0;
    uint32_t faults_logged     = s_faults.raise_count;
    int64_t  last_tick_us      = esp_timer_get_time();

    for (;;) {
        const uint32_t headroom_us = tick_wait();
        const int64_t  now        = esp_timer_get_time();
        const uint32_t now_ms     = (uint32_t)(now / 1000);
        float dt = (float)(now - last_tick_us) * 1e-6f;
        last_tick_us = now;
        if (dt > 0.02f) dt = 0.02f;           /* a stall must not permit a huge step */
        if (dt < 1e-4f) dt = 1e-4f;

        /* ---- overrun accounting (the telemetry line's own cost excluded) ---- */
        const uint32_t ovr_now   = g_tick_overruns;
        const uint32_t ovr_delta = ovr_now - ovr_seen;
        ovr_seen = ovr_now;
        if (ignore_ovr_ticks > 0u) {
            --ignore_ovr_ticks;
        } else {
            win.overruns    += ovr_delta;
            ovr_block_count += ovr_delta;
        }

        /* ===== 1. SENSE =================================================== */
        if (s_imu_ok) {
            imu_update(now, &imu);
            if (imu.fault_comm)     fault_raise(&s_faults, FAULT_IMU_COMM);
            if (imu.fault_disagree) fault_raise(&s_faults, FAULT_IMU_DISAGREE);
        } else {
            imu.valid = false;
        }

        bool link_ok = false;
        if (s_crsf_ok) {
            crsf_poll(now);
            link_ok = crsf_link_ok(now);
        }
        if (link_ok) {
            rc_input_decode(crsf_channels(), &s_params->channel_map, &s_params->rc, &rc);
        } else {
            rc_command_neutral(&rc);   /* §7f: never hold the last value */
        }

        /* ===== 2. JUDGE =================================================== */
        /* Link loss is a fault only once there has been a link to lose. */
        fault_set_condition(&s_faults, FAULT_RC_LINK, s_crsf_ok && crsf_ever_locked() && !link_ok);

        if (++ovr_block_ticks >= s_params->safety.overrun_window_ticks) {
            if (supervisor_is_armed(&s_sup) &&
                ovr_block_count >= s_params->safety.overrun_fault_count) {
                fault_raise(&s_faults, FAULT_LOOP_OVERRUN);
            }
            ovr_block_ticks = 0u;
            ovr_block_count = 0u;
        }

        if (s_sup.state == SUP_ARMED_FLAT && imu.valid &&
            (fabsf(imu.roll)  > s_params->imu.flat_tilt_fault_rad ||
             fabsf(imu.pitch) > s_params->imu.flat_tilt_fault_rad)) {
            fault_raise(&s_faults, FAULT_TILT);
        }

        /* While balancing, the filters run in the balance frame, so these are lean
         * angles from upright on the grounded pair. The frame check matters: on
         * the arming tick itself the estimate is still flat-frame (pitch ~85 deg)
         * and would otherwise trip this instantly. Past the limit the robot has
         * fallen or been grabbed. */
        if (supervisor_wants_balance_frame(&s_sup) && imu.valid &&
            imu.frame != IMU_FRAME_FLAT &&
            (fabsf(imu.pitch) > s_params->balance.tilt_fault_rad ||
             fabsf(imu.roll)  > s_params->balance.tilt_fault_rad)) {
            fault_raise(&s_faults, FAULT_TILT);
        }

        if (!supervisor_is_armed(&s_sup) && !rc.arm_request) {
            fault_clear_on_disarm(&s_faults);
        }

        const sup_inputs_t sin = {
            .now_ms          = now_ms,
            .link_ok         = link_ok,
            .arm_switch      = rc.arm_request,
            .sticks_centered = rc.sticks_centered,
            .mode_request    = (sup_mode_req_t)rc.drive_mode,
            .imu_ready       = s_imu_ok && imu.valid,
            .flat_roll_rad   = imu.roll,
            .flat_pitch_rad  = imu.pitch,
            .active_faults   = s_faults.active,
        };
        supervisor_update(&s_sup, &sup_cfg, &sin);

        if (s_sup.ev_disarmed) {
            s_cmd_rates.fl = s_cmd_rates.fr = s_cmd_rates.rl = s_cmd_rates.rr = 0.0f;
            step_gen_stop();
            buzzer_event(BUZZ_DISARMED, now_ms);
            if (s_imu_ok) {
                imu_set_frame(IMU_FRAME_FLAT);
            }
            char desc[96];
            fault_describe(s_faults.active, desc, sizeof desc);
            if (s_bal_ticks > 0u) {
                ESP_LOGW(TAG, "DISARMED after %.1f s balancing | peak lean %.1f deg | "
                              "final speed %.0f st/s (faults: %s)",
                         (double)s_bal_ticks / (double)TICK_RATE_HZ,
                         (double)(s_bal_peak_lean * RAD2DEG), (double)s_bal.fwd_speed, desc);
            } else {
                ESP_LOGW(TAG, "DISARMED (faults: %s)", desc);
            }
            s_bal_ticks     = 0u;
            s_bal_peak_lean = 0.0f;
            ignore_ovr_ticks = 3u;
        }
        if (s_sup.ev_armed) {
            buzzer_event(BUZZ_ARMED, now_ms);
            if (supervisor_wants_balance_frame(&s_sup) && s_imu_ok) {
                /* Estimate in the frame where upright reads level — the flat pitch
                 * folds at 90 deg — and start the controller from rest. */
                imu_set_frame(imu_frame_for_pair(s_sup.pair));
                balance_reset(&s_bal);
                ESP_LOGW(TAG, "ARMED -> %s on the %s pair: hold it upright for %u ms",
                         sup_state_name(s_sup.state),
                         s_sup.pair == OMNIS_BALANCE_ON_FRONT_PAIR ? "FRONT" : "REAR",
                         (unsigned)s_params->safety.balance_settle_ms);
            } else {
                ESP_LOGW(TAG, "ARMED -> %s", sup_state_name(s_sup.state));
            }
            ignore_ovr_ticks = 3u;
        }
        if (s_sup.ev_balance_engaged) {
            /* Buzzer only: a log line here would disturb the loop at the exact
             * moment it starts balancing. */
            buzzer_event(BUZZ_BALANCE_ENGAGED, now_ms);
            balance_reset(&s_bal);
        }
        if (s_sup.ev_rejected) {
            buzzer_event(BUZZ_ARM_REJECTED, now_ms);
            ESP_LOGW(TAG, "ARM REJECTED: %s", sup_result_name(s_sup.last_result));
            ignore_ovr_ticks = 3u;
        }
        if (s_faults.raise_count != faults_logged) {
            faults_logged = s_faults.raise_count;
            char desc[96];
            fault_describe(s_faults.active, desc, sizeof desc);
            ESP_LOGE(TAG, "FAULT raised - active: %s", desc);
            ignore_ovr_ticks = 3u;
        }

        /* ===== 3. ACT ===================================================== */
        bool          drive_live   = false;
        bool          balance_live = false;
        wheel_rates_t target       = { 0.0f, 0.0f, 0.0f, 0.0f };

#if OMNIS_BENCH_STEP_TEST
        if (s_step_ok) {
            bench_pattern(now, s_params, &sol);
            target     = sol.rates;
            drive_live = true;
        }
#else
        if (s_step_ok && s_sup.state == SUP_ARMED_FLAT) {
            /* The speed limiter scales the sticks before the kinematics, so LOW
             * limits every motion equally rather than clipping some axes. */
            const float speed = rc_speed_scale(rc.speed, &s_params->control);
            const rc_sticks_t sticks = {
                .throttle = rc.sticks.throttle * speed,
                .pitch    = rc.sticks.pitch    * speed,
                .roll     = rc.sticks.roll     * speed,
                .yaw      = rc.sticks.yaw      * speed,
            };
            drive_from_sticks(&sticks, s_params, &sol);
            target     = sol.rates;
            drive_live = true;
        } else if (s_step_ok && s_sup.state == SUP_ARMED_BALANCE && imu.valid &&
                   imu.frame != IMU_FRAME_FLAT) {
#if OMNIS_TUNE_KP_FROM_POT
            balance_set_kp(&s_bal, rc.tune_pot * s_params->balance.tune_kp_max);
#endif
            /* imu.pitch is the balance-frame lean (+ = falling toward the top
             * face) and imu.gyro[1] its rate. Pitch stick leans the target;
             * roll stick turns. The throttle is unused in balance mode. */
            balance_step(&s_bal, &s_bal_cfg, s_sup.pair, imu.pitch, imu.gyro[1],
                         rc.sticks.pitch, rc.sticks.roll, dt, &target);
            /* Hold a wheel near zero still rather than dithering its DIR line
             * (kinematics reference §6). */
            mecanum_deadband(&target, s_params->step.deadband_steps);
            balance_live = true;
            ++s_bal_ticks;
            if (fabsf(imu.pitch) > s_bal_peak_lean) s_bal_peak_lean = fabsf(imu.pitch);
        }
#endif

        if (drive_live) {
            (void)drive_slew_rates(&s_cmd_rates, &target,
                                   s_params->step.max_accel_steps_s2 * dt);
            step_gen_update(&s_cmd_rates);
        } else if (balance_live) {
            /* No extra slew here: the balance controller already limits
             * acceleration through its PID output clamp, and an added rate limit
             * is added lag — which destabilises a balancer. */
            s_cmd_rates = target;
            step_gen_update(&s_cmd_rates);
        } else {
            s_cmd_rates.fl = s_cmd_rates.fr = s_cmd_rates.rl = s_cmd_rates.rr = 0.0f;
            step_gen_stop();
        }

        /* ===== 4. REPORT ================================================== */
        buzzer_service(s_faults.active, now_ms);

        if (win.ticks == 0u) {
            win.start_us = now;   /* anchor on a tick edge */
        } else {
            if (headroom_us < win.min_headroom_us) win.min_headroom_us = headroom_us;
        }
        ++win.ticks;
        if (imu.read_us > win.i2c_max_us) win.i2c_max_us = imu.read_us;
        if (!imu.valid)                   ++win.imu_invalid;
        if (imu.disagreement > win.worst_disagree) win.worst_disagree = imu.disagreement;

        if (win.ticks >= TICK_RATE_HZ) {
#if OMNIS_TELEMETRY_ENABLED
            if (OMNIS_TELEMETRY_IN_BALANCE || !supervisor_wants_balance_frame(&s_sup)) {
                telemetry_print(&win, now, &imu, link_ok, &rc);
                ignore_ovr_ticks = 3u;   /* the line itself blocks for a few ms */
            }
#endif
            window_reset(&win);
        }
    }
}
