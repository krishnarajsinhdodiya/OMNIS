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

#include "drive.h"
#include "mecanum_kinematics.h"
#include "omnis_pins.h"
#include "omnis_params.h"
#include "tick.h"

static const char *TAG = "omnis";

/* ------------------------------------------------------------------------
 * Bring every output to a safe, known state.
 *
 * ORDER MATTERS AND COM_ENA IS FIRST. omnis-info.md §7g point 5: the failsafe
 * is "a single GPIO write on the shared EN# line and should be one of the
 * earliest-tested pieces of the whole firmware". Until this runs, the A4988
 * enable line is floating and the drivers' state is undefined.
 *
 * Note gpio_set_level() is called BEFORE gpio_config() for COM_ENA. That looks
 * backwards and is deliberate: it primes the output latch so that the instant
 * gpio_config() switches the pin to an output, it drives the disabled level
 * rather than briefly driving whatever was in the register.
 * ------------------------------------------------------------------------ */
static void gpio_safe_state(void)
{
    /* --- The kill line, first --- */
    gpio_set_level(PIN_COM_ENA, COM_ENA_DISABLED);

    gpio_config_t ena = {
        .pin_bit_mask = (1ULL << PIN_COM_ENA),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&ena));
    gpio_set_level(PIN_COM_ENA, COM_ENA_DISABLED);

    ESP_LOGI(TAG, "COM_ENA driven DISABLED (GPIO%d = %d) - motors are off",
             (int)PIN_COM_ENA, COM_ENA_DISABLED);

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

    /* --- Buzzer and the bring-up heartbeat --- */
    const gpio_config_t misc = {
        .pin_bit_mask = (1ULL << PIN_BUZZER) | (1ULL << PIN_TICK_HEARTBEAT),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&misc));
    gpio_set_level(PIN_BUZZER, BUZZER_OFF);
    gpio_set_level(PIN_TICK_HEARTBEAT, 0);

    ESP_LOGI(TAG, "STEP/DIR low, buzzer off, heartbeat on GPIO%d",
             (int)PIN_TICK_HEARTBEAT);
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

void app_main(void)
{
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "OMNIS superloop MVP - Stage 1 (timing spine)");
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
    bool     heartbeat      = false;

    /* Stage 2 drive-pipeline health, accumulated over the window. */
    drive_solution_t sol;
    uint32_t window_clamped    = 0;
    uint32_t window_deadbanded = 0;
    float    worst_null        = 0.0f;

    drive_solution_zero(&sol);

    while (1) {
        /* Wait for the ISR's flag and consume it. Returns the microseconds
         * spent waiting, which is this cycle's spare time. */
        const uint32_t headroom_us = tick_wait();

        const int64_t now = esp_timer_get_time();
        const uint32_t period_us = (uint32_t)(now - last_tick_us);
        last_tick_us = now;

        /* Square wave at TICK_RATE/2 = 250 Hz for scope verification. */
        heartbeat = !heartbeat;
        gpio_set_level(PIN_TICK_HEARTBEAT, heartbeat ? 1 : 0);

        /* ---- Stage 2: exercise the full drive pipeline every tick ----
         * Synthetic stick input tracing a slow circle in (vx, vy) with a bit of
         * yaw, so all four wheels see varying rates and both the clamp and the
         * deadband get hit during a run. The CRSF parser replaces this in
         * Stage 4; the RMT step generator consumes sol.rates in Stage 5.
         *
         * The two trig calls here are part of the synthetic input, NOT of the
         * kinematics — see the drive_solve benchmark printed at boot for the
         * real per-call cost. */
        const float phase = (float)(g_tick_count % 1000u) * (6.2831853f / 1000.0f);
        const rc_sticks_t sticks = {
            .throttle = 0.60f * cosf(phase),
            .pitch    = 0.0f,
            .roll     = 0.60f * sinf(phase),
            .yaw      = 0.25f * sinf(phase * 0.5f),
        };

        drive_from_sticks(&sticks, params, &sol);

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

            ESP_LOGI(TAG,
                     "%.2f Hz | period %" PRIu32 "-%" PRIu32 " us "
                     "| body %" PRIu32 " us | overruns %" PRIu32
                     " | clamp %" PRIu32 " dead %" PRIu32 " null %.4f"
                     " | up %" PRId64 " s",
                     measured_hz, min_period_us, max_period_us,
                     (uint32_t)(TICK_PERIOD_US - min_headroom), overruns,
                     window_clamped, window_deadbanded, (double)worst_null,
                     now / 1000000);

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
            g_tick_overruns   = 0;
        }
    }
}
