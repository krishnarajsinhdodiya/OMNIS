/**
 * @file    step_wave.h
 * @brief   Pure STEP-pulse waveform generator: rate in, RMT-style symbols out.
 *
 * No ESP-IDF, no RMT, no clock. Given a step rate, fill one "chunk" of waveform
 * — nominally one tick long — as (level, duration) pairs, carrying phase across
 * chunks so the pulse train is continuous. step_gen.c converts the symbols to
 * rmt_symbol_word_t and queues them; test/test_step_wave.c proves the waveform
 * properties on the host by expanding the symbols back into a timeline.
 *
 * WHY CHUNKS (BUILD-LOG.md Planning 1b, Option B): the RMT peripheral cannot change the rate
 * of a looping transmission without disable/enable/transmit on every change. So
 * each tick queues a fresh ~2 ms burst at the current rate, and the driver's own
 * ISR chains the bursts back-to-back.
 *
 * GUARANTEES (each one tested):
 *
 *   1. A HIGH pulse is never split across chunks, and every chunk ENDS LOW. The
 *      output idles low between RMT transactions, so a pulse cut at a chunk
 *      boundary would become two pulses — an extra step — and a chunk ending high
 *      followed by one starting high would merge two pulses into one — a lost step.
 *   2. Every pulse is exactly pulse_us high, and every low is at least
 *      min_low_us, including across chunk boundaries.
 *   3. No symbol half has zero duration. RMT treats a zero duration as
 *      end-of-transmission; a zero in the middle of a chunk would silently
 *      truncate it.
 *   4. The AVERAGE rate is exact. Each edge is rounded to 1 us, but the rounding
 *      residue is carried into the phase, so the error never accumulates.
 *   5. Chunk duration is chunk_us, extended by at most one pulse plus one
 *      minimum low (plus a 1 us pairing pad) — provided the rate is at or below
 *      step_wave_max_full_chunk_rate(). Above it the symbol buffer fills before
 *      the chunk does and the chunk ENDS EARLY; guarantees 1-4 still hold and the
 *      average rate stays exact, but each chunk covers less than a tick.
 *      step_gen refuses a max_step_rate above that limit, so it never happens
 *      in the firmware. (Found by test_step_wave.c: at 50 000 steps/s a
 *      64-symbol chunk lasted ~1.26 ms instead of 2 ms.)
 *   6. Rate changes take effect without a stall: phase is a FRACTION of the
 *      period, so speeding up from a slow crawl does not wait out the old period.
 */

#ifndef STEP_WAVE_H
#define STEP_WAVE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** RMT symbol half durations are 15-bit. */
#define STEP_SYMBOL_MAX_DURATION  32767u

/** One RMT-style symbol: two (level, duration) halves. Durations in microseconds. */
typedef struct {
    uint16_t dur0;
    uint8_t  lvl0;
    uint16_t dur1;
    uint8_t  lvl1;
} step_symbol_t;

typedef struct {
    uint32_t chunk_us;     /**< nominal chunk length — the tick period            */
    uint16_t pulse_us;     /**< STEP high time. A4988 minimum is 1 us.            */
    uint16_t min_low_us;   /**< minimum STEP low time; clamped to >= 2 internally */
} step_wave_cfg_t;

typedef struct {
    /* Fraction of the current step period elapsed since the last rising edge.
     * >= 1.0 means an edge is due immediately. Carried across chunks. */
    float    phase;
    uint64_t pulses;       /**< total pulses emitted by this generator */
} step_wave_t;

/** Phase set to "ready": the first pulse after init or a stop fires at once. */
void step_wave_init(step_wave_t *w);

/** Highest rate the config can produce with the pulse/low guarantees intact. */
float step_wave_max_rate(const step_wave_cfg_t *c);

/**
 * @brief Highest rate at which every chunk is guaranteed its full nominal length.
 *
 * Each pulse consumes about one symbol and the generator holds three in reserve
 * (trailing low, pairing pad, one of slack), so max_symbols fits roughly
 * (max_symbols - 4) pulses per chunk. With 64 symbols and 2 ms chunks: 30 000
 * steps/s. Returns 0 if max_symbols is too small to guarantee anything.
 */
float step_wave_max_full_chunk_rate(const step_wave_cfg_t *c, size_t max_symbols);

/**
 * @brief Generate one chunk of STEP waveform.
 *
 * @param w            generator state, advanced only by the chunk produced
 * @param c            timing config
 * @param rate_abs     step rate magnitude [steps/s]; <= 0, NaN or inf -> idle
 *                     low; above step_wave_max_rate() -> capped
 * @param out          symbol buffer
 * @param max_symbols  capacity of out; must be >= 2
 * @param duration_us  [out] exact duration of the symbols written
 * @param pulses       [out] rising edges in this chunk
 * @return symbols written, or 0 on invalid arguments
 */
size_t step_wave_fill(step_wave_t *w, const step_wave_cfg_t *c, float rate_abs,
                      step_symbol_t *out, size_t max_symbols,
                      uint32_t *duration_us, uint32_t *pulses);

#ifdef __cplusplus
}
#endif

#endif /* STEP_WAVE_H */
