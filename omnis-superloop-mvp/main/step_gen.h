/**
 * @file    step_gen.h
 * @brief   STEP/DIR generation for all four A4988s on the RMT peripheral.
 *
 * BUILD-LOG.md Planning 1b Option B, "burst re-arm": every tick, each wheel queues short
 * bursts of STEP pulses at its current rate, and the RMT driver's own ISR chains
 * them back-to-back. Nothing of ours runs in the pulse hot path — no task, no
 * timer ISR toggling pins, no loop.
 *
 * WHY NOT LOOP MODE: a looping RMT transmission can only change rate through
 * rmt_disable() / rmt_enable() / rmt_transmit(), on four channels, every tick,
 * with a glitch each time. ESP32-S3 has exactly four RMT TX channels, so there
 * is no spare to hide the glitch on.
 *
 * QUEUE REGULATION. Each wheel keeps STEP_TARGET_PENDING chunks (~6 ms) queued.
 * "Pending" is counted exactly — submitted in the superloop minus completed in
 * the RMT done-callback — so the generator never over-fills the driver's queue
 * and never guesses at playback timing. A late tick is absorbed by the lead; the
 * next tick tops the queue back up.
 *
 * DIRECTION REVERSAL. The A4988 needs DIR stable >= 200 ns before a STEP edge,
 * and chunks already queued were generated for the OLD direction. Flipping DIR
 * while they play would send those steps the wrong way. So a reversal stops
 * queueing, lets the queue drain, sets DIR, waits one full tick, then resumes.
 * Cost: a pause of a few ms at every reversal — invisible, and the deadband
 * already holds a wheel near zero before it reverses.
 *
 * STOPPING. Board Rev 2.0 has no driver enable (EN# is hardwired), so stopping
 * means "no more pulses". step_gen_stop() overwrites the level bits of every
 * queued chunk in place: the copy encoder reads payloads lazily, so chunks not
 * yet encoded play as silence, and the done-callback counters stay consistent.
 * At most the chunk already inside RMT memory (one tick, ~2 ms) still plays.
 * rmt_disable() was rejected for this: it recycles only the transaction in
 * progress, without a done callback, and leaves the rest queued to play after
 * the next enable.
 *
 * NO RTOS PRIMITIVES. The only ISR is the RMT done-callback, which increments
 * a counter. Both producer and consumer run on CPU1.
 */

#ifndef STEP_GEN_H
#define STEP_GEN_H

#include <stdbool.h>
#include <stdint.h>

#include "mecanum_kinematics.h"
#include "omnis_params.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STEP_WHEELS                 4       /* FL, FR, RL, RR */
#define STEP_TRANS_QUEUE_DEPTH      6
#define STEP_TARGET_PENDING         3       /* chunks queued ahead, ~6 ms at 500 Hz */
#define STEP_RING_SLOTS             8       /* > queue depth + 1, so no slot in use is reused */
#define STEP_MAX_SYMBOLS            64
#define STEP_MAX_SUBMITS_PER_TICK   4

typedef struct {
    uint32_t submitted;
    uint32_t completed;
    uint32_t pending;
    uint32_t submit_failures;   /**< rmt_transmit refused (queue full) */
    uint32_t underruns;         /**< queue ran dry while the wheel was moving */
    uint32_t reversals;
    uint64_t pulses;            /**< STEP pulses queued since boot */
    int8_t   dir_sign;          /**< +1 / -1: what DIR currently encodes */
} step_gen_stats_t;

/**
 * @brief Configure DIR pins and the four RMT channels. Call after
 *        gpio_safe_state() has driven STEP low.
 */
bool step_gen_init(const omnis_params_t *p);

/**
 * @brief Top up each wheel's queue at the given signed rates. Once per tick.
 * @param rates  signed step rates [microsteps/s], already clamped, deadbanded
 *               and slew-limited by the drive layer
 */
void step_gen_update(const wheel_rates_t *rates);

/** Stop all stepping as fast as the hardware allows. Idempotent; cheap to repeat. */
void step_gen_stop(void);

/** True when no wheel has anything queued. */
bool step_gen_idle(void);

void step_gen_get_stats(int wheel, step_gen_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* STEP_GEN_H */
