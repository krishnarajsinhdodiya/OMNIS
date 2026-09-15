/**
 * @file    crsf.h
 * @brief   CRSF receiver on UART1: RX-only, non-blocking, with pin auto-detect.
 *
 * Glue between the UART and the pure parser (crsf_parser.c). Polled once per
 * tick from the superloop with zero-timeout reads — no task, no queue, no event
 * callback of ours (PLAN.md §1a).
 *
 * RX-ONLY, ON PURPOSE. The MVP sends nothing to the receiver (no telemetry), so
 * the UART TX signal is never routed to a pin. That makes the pin auto-detect
 * below safe: if no valid frame arrives on GPIO18 (net "RX"), the receive signal
 * is moved to GPIO17 (net "TX") and back, every params.rc.pin_probe_ms, until
 * frames appear. Both pins remain inputs throughout, so a crossed harness can
 * never put the ESP32's output against the receiver's.
 *
 * LINK LOSS (omnis-info.md §7f). The link is "ok" only while RC frames keep
 * arriving within params.rc_timeout_us, and — if params.rc.min_link_quality is
 * non-zero — while the receiver's own uplink LQ is above it. §7f is explicit
 * that holding the last value on link loss is the wrong behaviour; the caller
 * treats !crsf_link_ok() as "stop".
 */

#ifndef CRSF_H
#define CRSF_H

#include <stdbool.h>
#include <stdint.h>

#include "crsf_parser.h"
#include "omnis_params.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool     locked;            /**< a valid RC frame has been seen on rx_pin  */
    int      rx_pin;            /**< GPIO currently carrying the receive signal */
    uint8_t  uplink_lq;         /**< [%] from the last link-statistics frame   */
    int      uplink_rssi_dbm;   /**< negative dBm                              */
    uint32_t rc_frames;
    uint32_t crc_errors;
    uint32_t bytes;
    int64_t  ms_since_rc;       /**< -1 if never                               */
} crsf_status_t;

/** Configure UART1 and start listening on GPIO18. */
bool crsf_init(const omnis_params_t *p);

/** Drain the UART and parse. Once per tick. Never blocks. */
void crsf_poll(int64_t now_us);

/** Link alive: fresh RC frames, and link quality above the configured floor. */
bool crsf_link_ok(int64_t now_us);

/** A valid RC frame has ever been received. Distinguishes "lost" from "never had". */
bool crsf_ever_locked(void);

/** Last good channel values (all 992 until the first frame). */
const uint16_t *crsf_channels(void);

void crsf_get_status(int64_t now_us, crsf_status_t *out);

#ifdef __cplusplus
}
#endif

#endif /* CRSF_H */
