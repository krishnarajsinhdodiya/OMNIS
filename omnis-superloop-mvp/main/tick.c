/**
 * @file    tick.c
 * @brief   500 Hz GPTimer tick — implementation.
 *
 * See tick.h for the design rule. The whole point of this file is that the ISR
 * below stays four lines long forever.
 */

#include "tick.h"

#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "tick";

volatile bool     g_tick_500hz    = false;
volatile uint32_t g_tick_overruns = 0;
volatile uint32_t g_tick_count    = 0;

static gptimer_handle_t s_timer = NULL;

/* ------------------------------------------------------------------------
 * The ISR.
 *
 * IRAM_ATTR because CONFIG_GPTIMER_ISR_HANDLER_IN_IRAM=y: the handler must not
 * live in flash, or its latency would depend on cache state and the tick would
 * jitter whenever something else touched flash.
 *
 * Everything it touches (the three volatiles) is in DRAM, so this is safe even
 * with the flash cache disabled.
 *
 * Returns false: no task was woken, so no context switch is requested. There is
 * no task to wake — that is the entire architecture.
 * ------------------------------------------------------------------------ */
static bool IRAM_ATTR tick_isr(gptimer_handle_t timer,
                               const gptimer_alarm_event_data_t *edata,
                               void *user_ctx)
{
    (void)timer;
    (void)edata;
    (void)user_ctx;

    if (g_tick_500hz) {
        /* The superloop has not consumed the previous tick yet — it overran its
         * 2 ms budget. Count it and carry on; the loop will simply see one tick
         * where two were due. Silently dropping it is the right call: trying to
         * "catch up" would make dt wrong in the EKF, which is worse than a
         * skipped cycle. */
        ++g_tick_overruns;
    }

    g_tick_500hz = true;
    ++g_tick_count;

    return false;
}

bool tick_init(void)
{
    const gptimer_config_t cfg = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        /* 1 MHz -> one count per microsecond, so the alarm value below reads
         * directly as a period in microseconds. */
        .resolution_hz = 1000000,
        .intr_priority = 0,
    };

    esp_err_t err = gptimer_new_timer(&cfg, &s_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gptimer_new_timer failed: %s", esp_err_to_name(err));
        return false;
    }

    const gptimer_event_callbacks_t cbs = { .on_alarm = tick_isr };
    err = gptimer_register_event_callbacks(s_timer, &cbs, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register callbacks failed: %s", esp_err_to_name(err));
        return false;
    }

    /* Hardware auto-reload. The period is maintained by the timer peripheral,
     * not by the ISR rewriting the alarm — so a late ISR does not accumulate
     * drift, it just delivers a late tick. */
    const gptimer_alarm_config_t alarm = {
        .alarm_count  = TICK_PERIOD_US,
        .reload_count = 0,
        .flags = { .auto_reload_on_alarm = true },
    };
    err = gptimer_set_alarm_action(s_timer, &alarm);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_alarm_action failed: %s", esp_err_to_name(err));
        return false;
    }

    err = gptimer_enable(s_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gptimer_enable failed: %s", esp_err_to_name(err));
        return false;
    }

    err = gptimer_start(s_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gptimer_start failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "%u Hz tick running (period %u us)",
             (unsigned)TICK_RATE_HZ, (unsigned)TICK_PERIOD_US);
    return true;
}

uint32_t tick_wait(void)
{
    const int64_t entered = esp_timer_get_time();

    /* Busy-wait. There is deliberately nothing else for this core to do. */
    while (!g_tick_500hz) {
        /* empty */
    }
    g_tick_500hz = false;

    const int64_t now = esp_timer_get_time();
    return (uint32_t)(now - entered);
}

void tick_reset_stats(void)
{
    g_tick_overruns = 0;
    g_tick_count    = 0;
}
