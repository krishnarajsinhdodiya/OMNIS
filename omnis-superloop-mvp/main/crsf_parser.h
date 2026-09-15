/**
 * @file    crsf_parser.h
 * @brief   Pure CRSF (Crossfire / ExpressLRS) byte-stream parser.
 *
 * No ESP-IDF, no UART, no clock: bytes in, decoded frames out. The UART glue is
 * crsf.c. Keeping them apart is what lets every framing edge case be tested on a
 * Mac (test/test_crsf.c) instead of discovered with a receiver on the bench.
 *
 * FRAME FORMAT
 *
 *   [sync][len][type][payload ... ][crc]
 *     1     1    1    len - 2       1
 *
 *   len  counts type + payload + crc, so a whole frame is len + 2 bytes.
 *        Valid range 2..62 (frames are at most 64 bytes).
 *   crc  CRC-8/DVB-S2 (poly 0xD5, init 0) over type + payload.
 *
 * Only two frame types matter here:
 *
 *   0x16 RC_CHANNELS_PACKED  22 bytes = 16 channels x 11 bits, LSB-first.
 *                            Values 172..1811 map to 988..2012 us; 992 = centre.
 *   0x14 LINK_STATISTICS     10 bytes; uplink link quality drives the failsafe.
 *
 * RESYNC STRATEGY. On any inconsistency — a non-sync first byte, an impossible
 * length, a CRC failure — the parser drops exactly ONE byte and rescans what it
 * already holds, rather than discarding the whole buffer. That matters after a
 * truncated frame: the bytes of the next, perfectly good frame are already in the
 * buffer, and throwing them away would cost a whole frame of latency every time.
 */

#ifndef CRSF_PARSER_H
#define CRSF_PARSER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CRSF_MAX_FRAME_LEN            64u
#define CRSF_LEN_FIELD_MIN            2u
#define CRSF_LEN_FIELD_MAX            62u

/* Address bytes seen as the first byte of a frame. A receiver talking to a
 * flight controller uses 0xC8; the others appear on some firmware and in
 * handset/module traffic. The CRC check makes accepting all of them safe. */
#define CRSF_SYNC_FLIGHT_CONTROLLER   0xC8
#define CRSF_SYNC_RADIO_TRANSMITTER   0xEA
#define CRSF_SYNC_RECEIVER            0xEC
#define CRSF_SYNC_TRANSMITTER_MODULE  0xEE

#define CRSF_TYPE_LINK_STATISTICS     0x14
#define CRSF_TYPE_RC_CHANNELS_PACKED  0x16

#define CRSF_RC_PAYLOAD_LEN           22u
#define CRSF_LINK_STATS_PAYLOAD_LEN   10u
#define CRSF_NUM_CHANNELS             16u

#define CRSF_CHANNEL_VALUE_MIN        172u
#define CRSF_CHANNEL_VALUE_MID        992u
#define CRSF_CHANNEL_VALUE_MAX        1811u

/** Events returned by crsf_parser_feed(), OR-ed together. */
#define CRSF_EVT_NONE                 0u
#define CRSF_EVT_CHANNELS             (1u << 0)
#define CRSF_EVT_LINK_STATS           (1u << 1)

typedef struct {
    uint8_t uplink_rssi_1;     /**< dBm, sign-inverted: 65 means -65 dBm */
    uint8_t uplink_rssi_2;
    uint8_t uplink_lq;         /**< uplink link quality [%]              */
    int8_t  uplink_snr;        /**< [dB]                                 */
    uint8_t active_antenna;
    uint8_t rf_mode;
    uint8_t uplink_tx_power;
    uint8_t downlink_rssi;
    uint8_t downlink_lq;
    int8_t  downlink_snr;
} crsf_link_stats_t;

typedef struct {
    uint8_t           buf[CRSF_MAX_FRAME_LEN];
    uint8_t           len;

    uint16_t          channels[CRSF_NUM_CHANNELS];   /**< last good RC frame */
    crsf_link_stats_t link;                          /**< last good stats    */

    uint32_t          bytes;            /**< fed in, total                         */
    uint32_t          frames_ok;        /**< CRC-valid frames of any type          */
    uint32_t          rc_frames;
    uint32_t          link_frames;
    uint32_t          other_frames;     /**< valid CRC, type not used here         */
    uint32_t          bad_payload;      /**< known type with the wrong length      */
    uint32_t          crc_errors;
    uint32_t          bytes_discarded;  /**< dropped while hunting for sync        */
} crsf_parser_t;

/** CRC-8/DVB-S2: poly 0xD5, init 0x00, no reflection. Check("123456789") = 0xBC. */
uint8_t crsf_crc8(const uint8_t *data, size_t len);

void crsf_parser_init(crsf_parser_t *p);

/**
 * @brief Feed received bytes. Any number, split anywhere.
 * @return CRSF_EVT_* bits for frames completed by this call
 */
unsigned crsf_parser_feed(crsf_parser_t *p, const uint8_t *data, size_t n);

bool crsf_is_sync_byte(uint8_t b);

/** 22 payload bytes -> 16 channels of 11 bits, LSB-first. */
void crsf_unpack_channels(const uint8_t payload[CRSF_RC_PAYLOAD_LEN],
                          uint16_t out[CRSF_NUM_CHANNELS]);

/** Inverse of crsf_unpack_channels(). Values are masked to 11 bits. */
void crsf_pack_channels(const uint16_t ch[CRSF_NUM_CHANNELS],
                        uint8_t payload[CRSF_RC_PAYLOAD_LEN]);

/**
 * @brief Build a complete RC_CHANNELS_PACKED frame (26 bytes).
 *
 * The firmware only receives, but a frame builder is what makes the parser
 * testable, and it is the obvious tool for a future bench simulator.
 *
 * @return bytes written (always 26)
 */
size_t crsf_build_rc_frame(const uint16_t ch[CRSF_NUM_CHANNELS],
                           uint8_t out[CRSF_MAX_FRAME_LEN]);

/**
 * @brief Channel value -> [-1, +1], centred on 992, clamped.
 *
 * 172 -> -1, 992 -> 0, 1811 -> +1. Values outside the nominal range clamp rather
 * than extrapolate: an out-of-range channel is a radio configuration problem, and
 * it must not become more than full stick.
 */
float crsf_channel_to_unit(uint16_t value);

#ifdef __cplusplus
}
#endif

#endif /* CRSF_PARSER_H */
