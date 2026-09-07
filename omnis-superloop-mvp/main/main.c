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

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

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

        /* ---- Stage 2+ work goes here ---- */

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
                     "| headroom min %" PRIu32 " us | overruns %" PRIu32
                     " | uptime %" PRId64 " s",
                     measured_hz, min_period_us, max_period_us,
                     min_headroom, overruns, now / 1000000);

            window_ticks  = 0;
            /* window_start is re-anchored on the next tick edge, above.
             * last_tick_us is reset here so the report's own duration is not
             * charged to the next window's first period. */
            last_tick_us  = esp_timer_get_time();
            min_period_us = UINT32_MAX;
            max_period_us = 0;
            min_headroom  = UINT32_MAX;
            g_tick_overruns = 0;
        }
    }
}
