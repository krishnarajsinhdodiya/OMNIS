/**
 * @file    step_wave.c
 * @brief   Pure STEP waveform generator — implementation. See step_wave.h.
 */

#include <math.h>
#include <stddef.h>
#include <stdbool.h>

#include "step_wave.h"

/* ------------------------------------------------------------------------
 * Symbol emitter.
 *
 * The timeline is built as a sequence of (level, duration) HALVES; RMT symbols
 * hold two halves each, so halves are paired as they arrive. A half left
 * unpaired at the end of the chunk is completed in finish().
 * ------------------------------------------------------------------------ */
typedef struct {
    step_symbol_t *out;
    size_t         max;
    size_t         count;          /* complete symbols written            */
    bool           have_pending;   /* a first half waiting for its partner */
    uint8_t        pend_lvl;
    uint16_t       pend_dur;
    uint8_t        last_lvl;       /* level of the most recent half        */
} emitter_t;

static void put_half(emitter_t *e, uint8_t lvl, uint16_t dur)
{
    e->last_lvl = lvl;
    if (!e->have_pending) {
        e->have_pending = true;
        e->pend_lvl     = lvl;
        e->pend_dur     = dur;
        return;
    }
    if (e->count < e->max) {
        e->out[e->count].lvl0 = e->pend_lvl;
        e->out[e->count].dur0 = e->pend_dur;
        e->out[e->count].lvl1 = lvl;
        e->out[e->count].dur1 = dur;
        ++e->count;
    }
    e->have_pending = false;
}

/* Emit a segment, split into halves no longer than the 15-bit limit. */
static void emit(emitter_t *e, uint8_t lvl, uint32_t dur)
{
    while (dur > 0u) {
        const uint16_t part = (uint16_t)(dur > STEP_SYMBOL_MAX_DURATION
                                         ? STEP_SYMBOL_MAX_DURATION : dur);
        put_half(e, lvl, part);
        dur -= part;
    }
}

/* Pair any leftover half. Returns microseconds of padding added (0 or 1). */
static uint32_t finish(emitter_t *e)
{
    if (!e->have_pending) {
        return 0u;
    }
    if (e->pend_lvl == 0u && e->pend_dur >= 2u) {
        /* Split the trailing low into two halves: same waveform, now paired. */
        const uint16_t a = (uint16_t)(e->pend_dur / 2u);
        const uint16_t b = (uint16_t)(e->pend_dur - a);
        e->have_pending = false;
        if (e->count < e->max) {
            e->out[e->count].lvl0 = 0u;  e->out[e->count].dur0 = a;
            e->out[e->count].lvl1 = 0u;  e->out[e->count].dur1 = b;
            ++e->count;
        }
        return 0u;
    }
    /* A 1 us low (or, defensively, a high) cannot be split: pair it with a 1 us
     * low. Never zero — zero duration means end-of-transmission to RMT. */
    put_half(e, 0u, 1u);
    return 1u;
}

/* ------------------------------------------------------------------------ */

void step_wave_init(step_wave_t *w)
{
    w->phase  = 1.0f;
    w->pulses = 0u;
}

static uint16_t effective_min_low(const step_wave_cfg_t *c)
{
    /* >= 2 us: an edge is placed by rounding, which can move it by up to
     * 0.5 us. Two microseconds of guaranteed low therefore survive as at least
     * one after rounding, so a low can never collapse to zero. */
    return (c->min_low_us < 2u) ? 2u : c->min_low_us;
}

float step_wave_max_rate(const step_wave_cfg_t *c)
{
    return 1.0e6f / (float)((uint32_t)c->pulse_us + effective_min_low(c));
}

float step_wave_max_full_chunk_rate(const step_wave_cfg_t *c, size_t max_symbols)
{
    if (c == NULL || c->chunk_us == 0u || max_symbols < 5u) {
        return 0.0f;
    }
    const float by_buffer = (float)(max_symbols - 4u) * 1.0e6f / (float)c->chunk_us;
    const float by_timing = step_wave_max_rate(c);
    return (by_buffer < by_timing) ? by_buffer : by_timing;
}

size_t step_wave_fill(step_wave_t *w, const step_wave_cfg_t *c, float rate_abs,
                      step_symbol_t *out, size_t max_symbols,
                      uint32_t *duration_us, uint32_t *pulses)
{
    if (duration_us != NULL) *duration_us = 0u;
    if (pulses != NULL)      *pulses = 0u;
    if (w == NULL || c == NULL || out == NULL || duration_us == NULL ||
        pulses == NULL || max_symbols < 2u || c->chunk_us == 0u ||
        c->pulse_us == 0u) {
        return 0u;
    }

    const uint16_t min_low = effective_min_low(c);
    emitter_t e = { .out = out, .max = max_symbols, .count = 0u,
                    .have_pending = false, .last_lvl = 0u };

    float rate = (isfinite(rate_abs) && rate_abs > 0.0f) ? rate_abs : 0.0f;
    const float rmax = step_wave_max_rate(c);
    if (rate > rmax) {
        rate = rmax;
    }

    uint32_t t = 0u;      /* microseconds into this chunk */
    uint32_t n = 0u;      /* pulses in this chunk         */

    if (rate <= 0.0f) {
        /* Idle: low for the whole chunk, and arm the generator so the first
         * pulse after a restart fires immediately instead of waiting out a
         * period that belongs to a speed the wheel is no longer doing. */
        emit(&e, 0u, c->chunk_us);
        t = c->chunk_us;
        w->phase = 1.0f;
    } else {
        const float period = 1.0e6f / rate;

        for (;;) {
            float wait = (1.0f - w->phase) * period;
            if (wait < 0.0f) {
                wait = 0.0f;
            }
            const float exact = (float)t + wait;
            uint32_t edge = (uint32_t)(exact + 0.5f);
            if (edge < t) {
                edge = t;
            }

            if (edge >= c->chunk_us) {
                /* No further edge inside this chunk: low to the end, and carry
                 * the elapsed fraction of the period into the next chunk. */
                if (c->chunk_us > t) {
                    emit(&e, 0u, c->chunk_us - t);
                    w->phase += (float)(c->chunk_us - t) / period;
                    t = c->chunk_us;
                }
                break;
            }

            /* Room for a low half, a high half, the trailing low and a pad.
             * If not, end the chunk here; the phase already reflects time t. */
            if (e.count + 3u > e.max) {
                break;
            }

            if (edge > t) {
                emit(&e, 0u, edge - t);
            }
            emit(&e, 1u, c->pulse_us);
            ++n;

            /* Carry the rounding residue so the average rate is exact. At time
             * t (end of the pulse), the time since the TRUE edge is
             * pulse_us + (edge - exact). */
            const float residue = (float)edge - exact;
            t = edge + c->pulse_us;
            w->phase = ((float)c->pulse_us + residue) / period;

            if (t >= c->chunk_us) {
                break;   /* the chunk may run up to one pulse past nominal */
            }
        }

        /* Guarantee 1: never end on a HIGH. Append the minimum low, counted in
         * the phase, so a pulse at the very start of the next chunk is still
         * separated from this one. */
        if (e.last_lvl == 1u) {
            emit(&e, 0u, min_low);
            t += min_low;
            w->phase += (float)min_low / period;
        }
    }

    const uint32_t pad = finish(&e);
    if (pad > 0u) {
        t += pad;
        if (rate > 0.0f) {
            w->phase += (float)pad * rate / 1.0e6f;
        }
    }

    *duration_us = t;
    *pulses      = n;
    w->pulses   += n;
    return e.count;
}
