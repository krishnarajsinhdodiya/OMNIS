/**
 * @file    main.c
 * @brief   OMNIS superloop MVP — entry point and the superloop itself.
 *
 * STAGE 1: timing spine only. GPIO is brought to a safe state, parameters are
 * loaded, the 500 Hz tick runs, and the loop measures itself. No motors, no
 * IMUs, no radio yet — those arrive in later stages.
 *
 * ARCHITECTURE (PLAN.md §1a): a plain while(1) with a hardware-timer ISR.
 * No xTaskCreate, no queues, no semaphores, no mutexes, no vTaskDelay anywhere
 * in OMNIS-authored code. ESP-IDF driver internals are treated as part of the
 * hardware. app_main is pinned to CPU1 and CPU1's idle-task watchdog is
 * disabled (sdkconfig.defaults), which is what makes a never-yielding loop
 * legal on this chip.
 */

#include <stdio.h>
#include <inttypes.h>
#include <math.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "crsf.h"
#include "drive.h"
#include "imu.h"
#include "mecanum_kinematics.h"
#include "omnis_config.h"
#include "omnis_pins.h"
#include "omnis_params.h"
#include "rc_input.h"
#include "step_gen.h"
#include "tick.h"

static const char *TAG = "omnis";

/* The rates actually handed to the step generator: the drive pipeline's output
 * after the acceleration limit. Kept across ticks because the slew limiter works
 * from where the wheels ARE, not from where they were asked to be. */
static wheel_rates_t s_cmd_rates = { 0.0f, 0.0f, 0.0f, 0.0f };

/* ------------------------------------------------------------------------
 * Bring every output to a safe, known state.
 *
 * BOARD REV 2.0 HAS NO STEPPER ENABLE LINE. All four A4988 EN# pins are
 * hardwired to GND (assets/pcb/rev2-pin-assignment.md §3), so the drivers are
 * live — energised and holding — whenever 12 V is present, from before this code
 * runs until the battery is unplugged. The only thing firmware controls is
 * whether STEP pulses exist.
 *
 * That makes the STEP lines the first thing to pin down: a floating STEP input
 * on a powered driver can pick up an edge and move a wheel. They are driven low
 * here, before anything else, and stay low until the RMT step generator takes
 * them over.
 * ------------------------------------------------------------------------ */
static void gpio_safe_state(void)
{
    /* --- STEP and DIR: outputs, all low ---
     * RMT takes these over in Stage 5. Until then they are plain outputs held
     * low so nothing floats into a driver input. */
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

    /* --- Buzzer ---
     * The Stage 1 tick heartbeat on GPIO35 is gone: GPIO35 is an octal-PSRAM
     * line on the N16R8 module, so toggling it was a live bug even though the
     * PSRAM is not enabled in this build (flagged in
     * assets/pcb/schematic-review-rev1.md, P4). The tick is verified. */
    const gpio_config_t misc = {
        .pin_bit_mask = (1ULL << PIN_BUZZER),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&misc));
    gpio_set_level(PIN_BUZZER, BUZZER_OFF);

    ESP_LOGI(TAG, "STEP/DIR held low, buzzer off. NOTE: EN# is hardwired - "
                  "motors are energised whenever 12 V is present");
}

static void log_params(const omnis_params_t *p)
{
    ESP_LOGI(TAG, "params schema v%" PRIu32 "  geometry r=%.1fmm L=%.1fmm W=%.1fmm",
             p->schema_version,
             (double)p->geometry.wheel_radius_mm,
             (double)p->geometry.wheelbase_mm,
             (double)p->geometry.track_width_mm);

    /* The derived quantity every kinematics equation depends on. Printing it at
     * boot means a wrong geometry is visible immediately rather than as a
     * subtly wrong turning radius later. */
    const float k = 0.5f * (p->geometry.wheelbase_mm + p->geometry.track_width_mm);
    ESP_LOGI(TAG, "  yaw lever arm k = %.1f mm, k/r = %.3f  (expect 226.5, 7.550)",
             (double)k, (double)(k / p->geometry.wheel_radius_mm));

    ESP_LOGI(TAG, "  step: max %.0f/s, deadband %.0f/s, accel %.0f/s^2",
             (double)p->step.max_step_rate,
             (double)p->step.deadband_steps,
             (double)p->step.max_accel_steps_s2);
}


/* ------------------------------------------------------------------------
 * Boot-time kinematics self-check.
 *
 * The same four cases the host test runs, re-checked on the Xtensa FPU. IEEE-754
 * float32 says these must agree bit-for-bit with the Mac, but "must" and "does"
 * are different claims and this costs microseconds to settle. A silent
 * divergence here would surface later as a robot that drives subtly wrong.
 * ------------------------------------------------------------------------ */
static bool near_enough(float got, float want, float tol)
{
    return fabsf(got - want) <= tol;
}

static bool kinematics_selftest(const omnis_geometry_t *g)
{
    const float R = g->wheel_radius_mm;
    const float L = g->wheelbase_mm;
    const float W = g->track_width_mm;
    const float TOL = 0.01f;
    bool ok = true;

    /* Case A - pure forward */
    wheel_rates_t a = mecanum_inverse(200.0f, 0.0f, 0.0f, R, L, W);
    ok &= near_enough(a.fl, 3395.305f, TOL) && near_enough(a.fr, 3395.305f, TOL)
       && near_enough(a.rl, 3395.305f, TOL) && near_enough(a.rr, 3395.305f, TOL);

    /* Case B - pure strafe left, splits along diagonals */
    wheel_rates_t b = mecanum_inverse(0.0f, 200.0f, 0.0f, R, L, W);
    ok &= near_enough(b.fl, -3395.305f, TOL) && near_enough(b.fr, 3395.305f, TOL)
       && near_enough(b.rl,  3395.305f, TOL) && near_enough(b.rr, -3395.305f, TOL);

    /* Case C - pure rotation CCW, splits along sides */
    wheel_rates_t c = mecanum_inverse(0.0f, 0.0f, 1.0f, R, L, W);
    ok &= near_enough(c.fl, -3845.183f, TOL) && near_enough(c.fr, 3845.183f, TOL)
       && near_enough(c.rl, -3845.183f, TOL) && near_enough(c.rr, 3845.183f, TOL);

    /* Case D - combined, and the IK->FK round trip that catches sign errors */
    wheel_rates_t d = mecanum_inverse(200.0f, 100.0f, 0.5f, R, L, W);
    ok &= near_enough(d.fl,  -224.939f, TOL) && near_enough(d.fr, 7015.550f, TOL)
       && near_enough(d.rl,  3170.366f, TOL) && near_enough(d.rr, 3620.244f, TOL);

    body_vel_t fk = mecanum_forward(&d, R, L, W);
    ok &= near_enough(fk.vx, 200.0f, 0.01f)
       && near_enough(fk.vy, 100.0f, 0.01f)
       && near_enough(fk.w,    0.5f, 1e-4f);

    ok &= near_enough(mecanum_null_space(&d), 0.0f, 0.01f);

    /* Reference §4's bench test: a vx = vy diagonal must leave FL and RR dead
     * still. This is the one that catches a swapped roller handedness. */
    wheel_rates_t diag = mecanum_inverse(200.0f, 200.0f, 0.0f, R, L, W);
    ok &= (diag.fl == 0.0f) && (diag.rr == 0.0f);

    ESP_LOGI(TAG, "kinematics self-check: %s", ok ? "PASS (cases A-D + round trip"
                                                    " + null space + diagonal)"
                                                  : "*** FAIL ***");
    if (ok) {
        ESP_LOGI(TAG, "  A fwd  %.3f | B strafe %.3f | C rot %.3f | D FR %.3f",
                 (double)a.fl, (double)b.fl, (double)c.fl, (double)d.fr);
    }
    return ok;
}

/* Measure drive_solve() properly: one call is only a few microseconds, and
 * esp_timer_get_time() around a single call would be mostly measuring the
 * timer. Average over many iterations instead. */
static void benchmark_drive(const omnis_params_t *p)
{
    const int N = 5000;
    drive_solution_t sol;
    body_vel_t v = { 200.0f, 100.0f, 0.5f };

    const int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < N; ++i) {
        /* Vary the input so the optimiser cannot hoist the call out. */
        v.vx = 200.0f + (float)(i & 7);
        drive_solve(&v, &p->geometry, &p->step, &sol);
    }
    const int64_t t1 = esp_timer_get_time();

    const double ns = (double)(t1 - t0) * 1000.0 / (double)N;
    ESP_LOGI(TAG, "drive_solve: %.0f ns/call (%.2f%% of the 2000 us tick)",
             ns, ns / 10.0 / 2000.0);
}

#if OMNIS_BENCH_STEP_TEST
/* ------------------------------------------------------------------------
 * Stage 5 bench pattern. WHEELS OFF THE GROUND — see omnis_config.h.
 *
 * Each segment runs a body velocity through the real pipeline (IK, clamp,
 * deadband) so the bench exercises exactly what driving will. The expected
 * wheel behaviour is in the segment name and in TESTING.md.
 * ------------------------------------------------------------------------ */
static void bench_pattern(int64_t now_us, const omnis_params_t *p,
                          drive_solution_t *sol)
{
    typedef struct {
        const char *name;
        float       vx, vy, w;
        uint32_t    ms;
    } bench_seg_t;

    static const bench_seg_t segs[] = {
        { "stop (settle)",                                          0.0f,   0.0f, 0.0f, 3000u },
        { "FORWARD 200 mm/s - all four wheels forward, ~3395 Hz",  200.0f,  0.0f, 0.0f, 3000u },
        { "stop",                                                   0.0f,   0.0f, 0.0f, 1500u },
        { "BACKWARD 200 mm/s - all four wheels backward",         -200.0f,  0.0f, 0.0f, 3000u },
        { "stop",                                                   0.0f,   0.0f, 0.0f, 1500u },
        { "STRAFE LEFT 200 mm/s - FL,RR backward; FR,RL forward",    0.0f, 200.0f, 0.0f, 3000u },
        { "stop",                                                   0.0f,   0.0f, 0.0f, 1500u },
        { "ROTATE CCW 1 rad/s - FL,RL backward; FR,RR forward",      0.0f,   0.0f, 1.0f, 3000u },
        { "stop",                                                   0.0f,   0.0f, 0.0f, 1500u },
        { "DIAGONAL vx=vy=150 - FR,RL forward; FL,RR MUST NOT TURN", 150.0f, 150.0f, 0.0f, 3000u },
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

/* Serviced once per tick while the boot-time IMU calibration holds the loop,
 * so the radio link is already found and locked by the time it finishes. */
static void boot_tick_hook(void)
{
    crsf_poll(esp_timer_get_time());
}

void app_main(void)
{
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "OMNIS superloop MVP - board Rev 2.0");
    ESP_LOGI(TAG, "==================================================");

    /* 1. Hardware safe FIRST, before anything can move. */
    gpio_safe_state();

    /* 2. Parameters. */
    omnis_params_init();
    const omnis_params_t *params = omnis_params();
    if (!omnis_params_valid(params)) {
        /* Cannot continue: the kinematics would divide by zero. In a later
         * stage this raises a buzzer fault; for now, refuse loudly and stop.
         * Note the motors are already disabled, so stopping here is safe. */
        ESP_LOGE(TAG, "PARAMETERS INVALID - refusing to run");
        while (1) { /* halt, motors off */ }
    }
    log_params(params);

    /* 2a. Step generation. Right after parameters, because board Rev 2.0's
     *     drivers are live whenever 12 V is present: the sooner RMT owns the STEP
     *     lines (initialised low), the shorter the window in which anything else
     *     could drive them. */
    if (!step_gen_init(params)) {
        ESP_LOGE(TAG, "STEP GENERATION INIT FAILED - refusing to run");
        while (1) { /* halt, STEP lines low */ }
    }
#if OMNIS_BENCH_STEP_TEST
    ESP_LOGW(TAG, "*** OMNIS_BENCH_STEP_TEST = 1: WHEELS WILL TURN ~3 s AFTER BOOT. "
                  "WHEELS OFF THE GROUND. ***");
#endif

    /* 2b. Kinematics: prove the ported module behaves on this FPU before the
     *     loop starts depending on it. */
    if (!kinematics_selftest(&params->geometry)) {
        ESP_LOGE(TAG, "KINEMATICS SELF-CHECK FAILED - refusing to run");
        while (1) { /* halt, motors off */ }
    }
    benchmark_drive(params);

    /* 3. The tick. Registered from app_main, so the ISR lands on CPU1 —
     *    the same core as this loop. See tick.h. */
    if (!tick_init()) {
        ESP_LOGE(TAG, "TICK INIT FAILED - refusing to run");
        while (1) { /* halt, motors off */ }
    }

    /* 3b. Radio. Started before the IMUs so it is already listening — and
     *     probing the RX pin if needed — during the 2 s gyro calibration. */
    if (!crsf_init(params)) {
        ESP_LOGE(TAG, "CRSF UART INIT FAILED - refusing to run");
        while (1) { /* halt, STEP lines low */ }
    }

    /* 4. Both IMUs. Stage 3 halts on failure; Stage 6 turns this into a latched
     *    fault that blocks arming instead, so the radio and buzzer keep working
     *    and the reason is reported. */
    if (!imu_init(params)) {
        ESP_LOGE(TAG, "IMU INIT FAILED - check I2C wiring and both 0x68/0x69");
        while (1) { /* halt, STEP lines low */ }
    }

#if OMNIS_RUN_MOUNT_WIZARD
    imu_run_mount_wizard();   /* never returns */
#endif

    if (!imu_calibrate(params, boot_tick_hook)) {
        ESP_LOGE(TAG, "IMU CALIBRATION FAILED - keep the robot still at power-on");
        while (1) { /* halt, STEP lines low */ }
    }

    ESP_LOGI(TAG, "entering superloop");

    /* --- Loop-health statistics ------------------------------------------
     * Measured over a one-second window, then reported and reset. The report
     * itself is deliberately OUTSIDE the measurement window: a log line takes
     * far longer than one 2 ms tick even at 921600 baud, so counting it would
     * guarantee an overrun every second and drown the signal we actually care
     * about. Stats are reset after printing so its cost is not attributed to
     * the loop. The report goes away entirely once the loop has real work.
     * ------------------------------------------------------------------- */
    uint32_t window_ticks   = 0;
    int64_t  window_start   = esp_timer_get_time();
    int64_t  last_tick_us   = window_start;
    uint32_t min_period_us  = UINT32_MAX;
    uint32_t max_period_us  = 0;
    uint32_t min_headroom   = UINT32_MAX;

    /* Stage 2 drive-pipeline health, accumulated over the window. */
    drive_solution_t sol;
    uint32_t window_clamped    = 0;
    uint32_t window_deadbanded = 0;
    float    worst_null        = 0.0f;

    /* Stage 3 IMU health, accumulated over the window. */
    imu_state_t imu;
    uint32_t    window_i2c_max_us = 0;
    uint32_t    window_imu_invalid = 0;
    float       worst_disagree    = 0.0f;

    drive_solution_zero(&sol);

    while (1) {
        /* Wait for the ISR's flag and consume it. Returns the microseconds
         * spent waiting, which is this cycle's spare time. */
        const uint32_t headroom_us = tick_wait();

        const int64_t now = esp_timer_get_time();
        const uint32_t period_us = (uint32_t)(now - last_tick_us);
        last_tick_us = now;

        /* ---- Stage 3: read, filter and fuse both IMUs ---- */
        imu_update(now, &imu);
        if (imu.read_us > window_i2c_max_us) window_i2c_max_us = imu.read_us;
        if (!imu.valid)                      ++window_imu_invalid;
        if (imu.disagreement > worst_disagree) worst_disagree = imu.disagreement;

        /* ---- Stage 4: radio ----
         * Whenever the link is not OK the command is NEUTRAL: centred sticks and
         * no arm request. §7f is explicit that holding the last value on link loss
         * is the wrong behaviour. */
        crsf_poll(now);
        const bool   link_ok = crsf_link_ok(now);
        rc_command_t rc;
        if (link_ok) {
            rc_input_decode(crsf_channels(), &params->channel_map, &params->rc, &rc);
        } else {
            rc_command_neutral(&rc);
        }

        /* ---- Drive pipeline, now fed by the radio ----
         * The speed-limiter switch scales the sticks before kinematics, so a LOW
         * setting limits every motion equally rather than clipping some axes.
         * Stage 5 hands sol.rates to the step generator; until then they are
         * computed and measured but drive nothing. */
        const float speed = rc_speed_scale(rc.speed, &params->control);
        const rc_sticks_t sticks = {
            .throttle = rc.sticks.throttle * speed,
            .pitch    = rc.sticks.pitch    * speed,
            .roll     = rc.sticks.roll     * speed,
            .yaw      = rc.sticks.yaw      * speed,
        };
        drive_from_sticks(&sticks, params, &sol);

        /* ---- Stage 5: acceleration limit, then STEP pulses ----
         * Motors step only when something has explicitly enabled them. Until the
         * supervisor exists (Stage 6/7) that is the bench pattern alone; in a
         * normal build the step generator is held stopped. */
#if OMNIS_BENCH_STEP_TEST
        bench_pattern(now, params, &sol);
        const bool motors_enabled = true;
#else
        const bool motors_enabled = false;
#endif
        float dt_s = (float)period_us * 1e-6f;
        if (dt_s > 0.02f) dt_s = 0.02f;   /* a stall must not permit a huge jump */

        if (motors_enabled) {
            (void)drive_slew_rates(&s_cmd_rates, &sol.rates,
                                   params->step.max_accel_steps_s2 * dt_s);
            step_gen_update(&s_cmd_rates);
        } else {
            s_cmd_rates.fl = s_cmd_rates.fr = s_cmd_rates.rl = s_cmd_rates.rr = 0.0f;
            step_gen_stop();
        }

        if (sol.clamped)         ++window_clamped;
        if (sol.deadbanded > 0)  ++window_deadbanded;
        /* The null-space metric is taken post-clamp, pre-deadband, so on a
         * healthy command it stays at zero. Anything above float noise means
         * the drivetrain would be fighting itself. */
        if (fabsf(sol.null_space) > worst_null) worst_null = fabsf(sol.null_space);

        if (window_ticks == 0) {
            /* Anchor the measurement window on a TICK EDGE, not on the moment
             * the previous report finished.
             *
             * Anchoring it on the report instead made the window span 499 whole
             * periods plus whatever fraction remained of the one in progress,
             * which biased the reported rate high by exactly that shortfall. On
             * hardware it read a stubborn 500.13 Hz while the period itself
             * measured an exact 2000-2000 us — the bias was in this arithmetic,
             * not in the timer. */
            window_start = now;
        } else {
            /* The first period of a window spans the report gap, so it is
             * measured but not counted. */
            if (period_us < min_period_us) min_period_us = period_us;
            if (period_us > max_period_us) max_period_us = period_us;
            if (headroom_us < min_headroom) min_headroom = headroom_us;
        }
        ++window_ticks;

        if (window_ticks >= TICK_RATE_HZ) {
            const int64_t  elapsed_us = now - window_start;
            const uint32_t overruns   = g_tick_overruns;

            /* Measured rate from the wall clock, not from the tick count —
             * counting ticks and dividing by the period they were configured
             * with would just report the configuration back.
             *
             * N tick edges span N-1 intervals. Dividing by N is the off-by-one
             * that produced the 500.13 Hz above. */
            const double measured_hz =
                (double)(window_ticks - 1u) * 1e6 / (double)elapsed_us;

            /* One line, deliberately: every log line costs milliseconds, and
             * this report sits outside the measurement window for that reason. */
            crsf_status_t cs;
            crsf_get_status(now, &cs);
            ESP_LOGI(TAG,
                     "%.2f Hz | body %" PRIu32 " us ovr %" PRIu32
                     " | imu r %+.1f p %+.1f dis %.2f i2c %" PRIu32 " us bad %" PRIu32 "%s%s"
                     " | rc %s gpio%d LQ %u crc %" PRIu32
                     " arm %d mode %s spd %s thr %+.2f rol %+.2f yaw %+.2f | up %" PRId64 " s",
                     measured_hz, (uint32_t)(TICK_PERIOD_US - min_headroom), overruns,
                     (double)(imu.roll * 57.29578f), (double)(imu.pitch * 57.29578f),
                     (double)(worst_disagree * 57.29578f), window_i2c_max_us,
                     window_imu_invalid,
                     imu.fault_comm ? " COMM-FAULT" : "",
                     imu.fault_disagree ? " DISAGREE-FAULT" : "",
                     !cs.locked ? "NONE" : (link_ok ? "OK" : "LOST"), cs.rx_pin,
                     (unsigned)cs.uplink_lq, cs.crc_errors,
                     rc.arm_request ? 1 : 0, rc_switch_name(rc.drive_mode),
                     rc_switch_name(rc.speed), (double)rc.sticks.throttle,
                     (double)rc.sticks.roll, (double)rc.sticks.yaw, now / 1000000);
            (void)min_period_us;
            (void)max_period_us;
            (void)window_deadbanded;
            (void)window_clamped;
            (void)worst_null;
#if OMNIS_BENCH_STEP_TEST
            {
                uint32_t und = 0, fail = 0, rev = 0;
                uint64_t pulses = 0;
                for (int wh = 0; wh < STEP_WHEELS; ++wh) {
                    step_gen_stats_t st;
                    step_gen_get_stats(wh, &st);
                    und += st.underruns; fail += st.submit_failures;
                    rev += st.reversals; pulses += st.pulses;
                }
                ESP_LOGI(TAG, "step: FL %+6.0f FR %+6.0f RL %+6.0f RR %+6.0f st/s | "
                              "pulses %llu underruns %" PRIu32 " submit-fail %" PRIu32
                              " reversals %" PRIu32,
                         (double)s_cmd_rates.fl, (double)s_cmd_rates.fr,
                         (double)s_cmd_rates.rl, (double)s_cmd_rates.rr,
                         (unsigned long long)pulses, und, fail, rev);
            }
#endif

            window_ticks  = 0;
            /* window_start is re-anchored on the next tick edge, above.
             * last_tick_us is reset here so the report's own duration is not
             * charged to the next window's first period. */
            last_tick_us  = esp_timer_get_time();
            min_period_us     = UINT32_MAX;
            max_period_us     = 0;
            min_headroom      = UINT32_MAX;
            window_clamped    = 0;
            window_deadbanded = 0;
            worst_null        = 0.0f;
            window_i2c_max_us = 0;
            window_imu_invalid = 0;
            worst_disagree    = 0.0f;
            g_tick_overruns   = 0;
        }
    }
}
