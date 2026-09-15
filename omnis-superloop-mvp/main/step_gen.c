/**
 * @file    step_gen.c
 * @brief   RMT STEP generation for four A4988s — implementation. See step_gen.h.
 */

#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_attr.h"
#include "esp_log.h"

#include "omnis_pins.h"
#include "step_gen.h"
#include "step_wave.h"
#include "tick.h"

static const char *TAG = "step";

typedef enum {
    WHEEL_RUN = 0,   /* queueing normally                                  */
    WHEEL_DRAIN,     /* reversal requested: wait for the queue to empty    */
    WHEEL_SETTLE,    /* DIR just written: hold one tick before stepping     */
} wheel_state_t;

typedef struct {
    rmt_channel_handle_t chan;
    rmt_encoder_handle_t enc;
    gpio_num_t           step_pin;
    gpio_num_t           dir_pin;
    bool                 dir_invert;

    wheel_state_t        state;
    int8_t               dir_sign;       /* +1 / -1 currently on DIR */
    int8_t               next_sign;      /* sign to apply after a drain */
    bool                 was_running;    /* for underrun detection      */
    step_wave_t          wave;

    /* Payload buffers must outlive their transaction: the copy encoder reads
     * them from the ISR while the transaction plays. A ring larger than the
     * driver's queue guarantees a slot is never rewritten while queued. */
    rmt_symbol_word_t    ring[STEP_RING_SLOTS][STEP_MAX_SYMBOLS];
    uint16_t             ring_len[STEP_RING_SLOTS];
    uint8_t              slot;

    uint32_t             submitted;
    volatile uint32_t    completed;      /* ISR-written */

    uint32_t             submit_failures;
    uint32_t             underruns;
    uint32_t             reversals;
    uint64_t             pulses;
} wheel_t;

static wheel_t          s_wheel[STEP_WHEELS];
static step_wave_cfg_t  s_cfg;
static bool             s_ready   = false;
static bool             s_stopped = true;

static const rmt_transmit_config_t k_tx_cfg = {
    .loop_count = 0,
    .flags = {
        .eot_level         = 0,   /* idle low between transactions */
        .queue_nonblocking = 1,   /* never block the superloop     */
    },
};

/* ------------------------------------------------------------------------
 * The only ISR in this module. RMT done-callback: count, nothing else.
 * ------------------------------------------------------------------------ */
static bool IRAM_ATTR on_trans_done(rmt_channel_handle_t chan,
                                    const rmt_tx_done_event_data_t *edata,
                                    void *user_ctx)
{
    (void)chan;
    (void)edata;
    wheel_t *w = (wheel_t *)user_ctx;
    ++w->completed;
    return false;   /* no task to wake — there are no tasks */
}

static void write_dir(wheel_t *w, int8_t sign)
{
    const int level = ((sign > 0) ? 1 : 0) ^ (w->dir_invert ? 1 : 0);
    gpio_set_level(w->dir_pin, level);
}

bool step_gen_init(const omnis_params_t *p)
{
    static const gpio_num_t step_pins[STEP_WHEELS] = {
        PIN_FL_STEP, PIN_FR_STEP, PIN_RL_STEP, PIN_RR_STEP };
    static const gpio_num_t dir_pins[STEP_WHEELS] = {
        PIN_FL_DIR, PIN_FR_DIR, PIN_RL_DIR, PIN_RR_DIR };
    static const char *const names[STEP_WHEELS] = { "FL", "FR", "RL", "RR" };

    memset(s_wheel, 0, sizeof s_wheel);

    s_cfg.chunk_us   = TICK_PERIOD_US;
    s_cfg.pulse_us   = p->motor.step_pulse_us;
    s_cfg.min_low_us = p->motor.step_min_low_us;

    /* Refuse any rate at which a chunk could end early. Short chunks keep every
     * pulse guarantee but cover less than a tick, which quietly erodes the queue
     * lead the generator depends on. Found by test_step_wave.c. */
    const float full_rate = step_wave_max_full_chunk_rate(&s_cfg, STEP_MAX_SYMBOLS);
    if (p->step.max_step_rate > full_rate) {
        ESP_LOGE(TAG, "max_step_rate %.0f exceeds %.0f steps/s, the most a %u-symbol "
                      "chunk can hold at full length", (double)p->step.max_step_rate,
                 (double)full_rate, (unsigned)STEP_MAX_SYMBOLS);
        return false;
    }

    for (int i = 0; i < STEP_WHEELS; ++i) {
        wheel_t *w     = &s_wheel[i];
        w->step_pin    = step_pins[i];
        w->dir_pin     = dir_pins[i];
        w->dir_invert  = p->motor.dir_invert[i];
        w->state       = WHEEL_RUN;
        w->dir_sign    = +1;
        step_wave_init(&w->wave);

        const gpio_config_t dir_cfg = {
            .pin_bit_mask = 1ULL << w->dir_pin,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        if (gpio_config(&dir_cfg) != ESP_OK) {
            return false;
        }
        write_dir(w, w->dir_sign);

        const rmt_tx_channel_config_t tx = {
            .gpio_num          = w->step_pin,
            .clk_src           = RMT_CLK_SRC_DEFAULT,
            .resolution_hz     = 1000000,               /* 1 us per tick */
            .mem_block_symbols = 48,                    /* S3 minimum, one block */
            .trans_queue_depth = STEP_TRANS_QUEUE_DEPTH,
            .flags = { .init_level = 0 },
        };
        esp_err_t err = rmt_new_tx_channel(&tx, &w->chan);
        if (err == ESP_OK) {
            /* One encoder per channel: an encoder holds per-transaction state,
             * so sharing one across channels would corrupt concurrent bursts. */
            const rmt_copy_encoder_config_t enc_cfg = {};
            err = rmt_new_copy_encoder(&enc_cfg, &w->enc);
        }
        if (err == ESP_OK) {
            const rmt_tx_event_callbacks_t cbs = { .on_trans_done = on_trans_done };
            err = rmt_tx_register_event_callbacks(w->chan, &cbs, w);
        }
        if (err == ESP_OK) {
            err = rmt_enable(w->chan);
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s: RMT setup on GPIO%d failed: %s", names[i],
                     (int)w->step_pin, esp_err_to_name(err));
            return false;
        }
        ESP_LOGI(TAG, "%s: STEP GPIO%d, DIR GPIO%d%s", names[i], (int)w->step_pin,
                 (int)w->dir_pin, w->dir_invert ? " (inverted)" : "");
    }

    s_ready   = true;
    s_stopped = true;
    ESP_LOGI(TAG, "RMT step generation ready: %u us chunks, %u us pulses, max %.0f steps/s",
             (unsigned)s_cfg.chunk_us, (unsigned)s_cfg.pulse_us,
             (double)step_wave_max_rate(&s_cfg));
    return true;
}

static uint32_t pending_of(const wheel_t *w)
{
    return w->submitted - w->completed;   /* 32-bit read of an ISR counter: atomic */
}

static void wheel_service(wheel_t *w, float rate)
{
    const uint32_t pending = pending_of(w);
    const int8_t   want    = (rate > 0.0f) ? 1 : ((rate < 0.0f) ? -1 : 0);

    switch (w->state) {
        case WHEEL_DRAIN:
            if (pending == 0u) {
                write_dir(w, w->next_sign);
                w->dir_sign = w->next_sign;
                w->state    = WHEEL_SETTLE;
            }
            return;
        case WHEEL_SETTLE:
            /* DIR has been stable for a whole tick: 2 ms against a 200 ns
             * requirement. Resume next tick. */
            w->state = WHEEL_RUN;
            return;
        case WHEEL_RUN:
        default:
            break;
    }

    if (want != 0 && want != w->dir_sign) {
        ++w->reversals;
        w->was_running = false;
        w->next_sign   = want;
        step_wave_init(&w->wave);
        if (pending == 0u) {
            write_dir(w, want);
            w->dir_sign = want;
            w->state    = WHEEL_SETTLE;
        } else {
            w->state = WHEEL_DRAIN;
        }
        return;
    }

    if (want == 0) {
        /* Queue nothing; what is queued finishes and the output idles low. */
        w->wave.phase  = 1.0f;
        w->was_running = false;
        return;
    }

    if (pending == 0u && w->was_running) {
        ++w->underruns;   /* the queue ran dry while moving: a gap occurred */
    }

    const float mag = fabsf(rate);
    uint32_t    p   = pending;
    for (int k = 0; k < STEP_MAX_SUBMITS_PER_TICK && p < STEP_TARGET_PENDING; ++k) {
        step_symbol_t sym[STEP_MAX_SYMBOLS];
        step_wave_t   trial = w->wave;      /* commit phase only if queued */
        uint32_t      dur   = 0u;
        uint32_t      np    = 0u;

        const size_t n = step_wave_fill(&trial, &s_cfg, mag, sym, STEP_MAX_SYMBOLS,
                                        &dur, &np);
        if (n == 0u) {
            break;
        }

        rmt_symbol_word_t *buf = w->ring[w->slot];
        for (size_t j = 0; j < n; ++j) {
            buf[j].duration0 = sym[j].dur0;
            buf[j].level0    = sym[j].lvl0;
            buf[j].duration1 = sym[j].dur1;
            buf[j].level1    = sym[j].lvl1;
        }
        w->ring_len[w->slot] = (uint16_t)n;

        const esp_err_t err = rmt_transmit(w->chan, w->enc, buf,
                                           n * sizeof(rmt_symbol_word_t), &k_tx_cfg);
        if (err != ESP_OK) {
            ++w->submit_failures;
            break;   /* phase not committed: this chunk is regenerated next tick */
        }

        w->wave = trial;
        w->slot = (uint8_t)((w->slot + 1u) % STEP_RING_SLOTS);
        ++w->submitted;
        ++p;
        w->pulses += np;
    }
    w->was_running = true;
}

void step_gen_update(const wheel_rates_t *rates)
{
    if (!s_ready || rates == NULL) {
        return;
    }
    s_stopped = false;

    const float r[STEP_WHEELS] = { rates->fl, rates->fr, rates->rl, rates->rr };
    for (int i = 0; i < STEP_WHEELS; ++i) {
        wheel_service(&s_wheel[i], isfinite(r[i]) ? r[i] : 0.0f);
    }
}

void step_gen_stop(void)
{
    if (!s_ready || s_stopped) {
        return;
    }
    s_stopped = true;

    for (int i = 0; i < STEP_WHEELS; ++i) {
        wheel_t *w = &s_wheel[i];
        /* Silence every queued burst in place. The ISR only ever READS these
         * words, so a read-modify-write here cannot lose an update. */
        for (int s = 0; s < STEP_RING_SLOTS; ++s) {
            for (uint16_t j = 0; j < w->ring_len[s]; ++j) {
                w->ring[s][j].level0 = 0;
                w->ring[s][j].level1 = 0;
            }
        }
        w->wave.phase  = 1.0f;
        w->was_running = false;
        if (w->state != WHEEL_RUN) {
            w->state = WHEEL_RUN;   /* DIR already written; nothing more to wait for */
        }
    }
}

bool step_gen_idle(void)
{
    for (int i = 0; i < STEP_WHEELS; ++i) {
        if (pending_of(&s_wheel[i]) != 0u) {
            return false;
        }
    }
    return true;
}

void step_gen_get_stats(int wheel, step_gen_stats_t *out)
{
    if (wheel < 0 || wheel >= STEP_WHEELS || out == NULL) {
        return;
    }
    const wheel_t *w = &s_wheel[wheel];
    out->submitted       = w->submitted;
    out->completed       = w->completed;
    out->pending         = pending_of(w);
    out->submit_failures = w->submit_failures;
    out->underruns       = w->underruns;
    out->reversals       = w->reversals;
    out->pulses          = w->pulses;
    out->dir_sign        = w->dir_sign;
}
