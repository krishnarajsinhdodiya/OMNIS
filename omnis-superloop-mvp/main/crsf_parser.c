/**
 * @file    crsf_parser.c
 * @brief   Pure CRSF parser — implementation. See crsf_parser.h.
 */

#include <string.h>

#include "crsf_parser.h"

uint8_t crsf_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0xD5u) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

void crsf_parser_init(crsf_parser_t *p)
{
    memset(p, 0, sizeof *p);
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        p->channels[i] = CRSF_CHANNEL_VALUE_MID;
    }
}

bool crsf_is_sync_byte(uint8_t b)
{
    return b == CRSF_SYNC_FLIGHT_CONTROLLER || b == CRSF_SYNC_RADIO_TRANSMITTER
        || b == CRSF_SYNC_RECEIVER          || b == CRSF_SYNC_TRANSMITTER_MODULE;
}

void crsf_unpack_channels(const uint8_t payload[CRSF_RC_PAYLOAD_LEN],
                          uint16_t out[CRSF_NUM_CHANNELS])
{
    uint32_t acc   = 0;   /* bit accumulator, LSB-first */
    unsigned nbits = 0;
    size_t   ch    = 0;

    for (size_t i = 0; i < CRSF_RC_PAYLOAD_LEN && ch < CRSF_NUM_CHANNELS; ++i) {
        acc |= (uint32_t)payload[i] << nbits;
        nbits += 8u;
        while (nbits >= 11u && ch < CRSF_NUM_CHANNELS) {
            out[ch++] = (uint16_t)(acc & 0x7FFu);
            acc >>= 11;
            nbits -= 11u;
        }
    }
}

void crsf_pack_channels(const uint16_t ch[CRSF_NUM_CHANNELS],
                        uint8_t payload[CRSF_RC_PAYLOAD_LEN])
{
    uint32_t acc   = 0;
    unsigned nbits = 0;
    size_t   out   = 0;

    memset(payload, 0, CRSF_RC_PAYLOAD_LEN);
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        acc |= (uint32_t)(ch[i] & 0x7FFu) << nbits;
        nbits += 11u;
        while (nbits >= 8u && out < CRSF_RC_PAYLOAD_LEN) {
            payload[out++] = (uint8_t)(acc & 0xFFu);
            acc >>= 8;
            nbits -= 8u;
        }
    }
    /* 16 x 11 = 176 bits = exactly 22 bytes, so nothing is left over. */
}

size_t crsf_build_rc_frame(const uint16_t ch[CRSF_NUM_CHANNELS],
                           uint8_t out[CRSF_MAX_FRAME_LEN])
{
    out[0] = CRSF_SYNC_FLIGHT_CONTROLLER;
    out[1] = (uint8_t)(1u + CRSF_RC_PAYLOAD_LEN + 1u);     /* type + payload + crc */
    out[2] = CRSF_TYPE_RC_CHANNELS_PACKED;
    crsf_pack_channels(ch, &out[3]);
    out[3 + CRSF_RC_PAYLOAD_LEN] = crsf_crc8(&out[2], 1u + CRSF_RC_PAYLOAD_LEN);
    return 4u + CRSF_RC_PAYLOAD_LEN;
}

float crsf_channel_to_unit(uint16_t value)
{
    /* (1811 - 992) = 819, so full deflection is exactly +1.0. The low end
     * (172 - 992 = -820) overshoots by one count and is clamped to -1.0. */
    float u = ((float)value - (float)CRSF_CHANNEL_VALUE_MID) / 819.0f;
    if (u >  1.0f) u =  1.0f;
    if (u < -1.0f) u = -1.0f;
    return u;
}

static void drop_front(crsf_parser_t *p, size_t n)
{
    if (n >= p->len) {
        p->len = 0;
        return;
    }
    memmove(p->buf, &p->buf[n], (size_t)p->len - n);
    p->len = (uint8_t)(p->len - n);
}

static unsigned handle_frame(crsf_parser_t *p, uint8_t type,
                             const uint8_t *payload, size_t payload_len)
{
    ++p->frames_ok;

    switch (type) {
        case CRSF_TYPE_RC_CHANNELS_PACKED:
            if (payload_len != CRSF_RC_PAYLOAD_LEN) {
                ++p->bad_payload;
                return CRSF_EVT_NONE;
            }
            crsf_unpack_channels(payload, p->channels);
            ++p->rc_frames;
            return CRSF_EVT_CHANNELS;

        case CRSF_TYPE_LINK_STATISTICS:
            if (payload_len != CRSF_LINK_STATS_PAYLOAD_LEN) {
                ++p->bad_payload;
                return CRSF_EVT_NONE;
            }
            p->link.uplink_rssi_1   = payload[0];
            p->link.uplink_rssi_2   = payload[1];
            p->link.uplink_lq       = payload[2];
            p->link.uplink_snr      = (int8_t)payload[3];
            p->link.active_antenna  = payload[4];
            p->link.rf_mode         = payload[5];
            p->link.uplink_tx_power = payload[6];
            p->link.downlink_rssi   = payload[7];
            p->link.downlink_lq     = payload[8];
            p->link.downlink_snr    = (int8_t)payload[9];
            ++p->link_frames;
            return CRSF_EVT_LINK_STATS;

        default:
            ++p->other_frames;
            return CRSF_EVT_NONE;
    }
}

/* Consume every complete frame currently in the buffer. */
static unsigned process(crsf_parser_t *p)
{
    unsigned events = CRSF_EVT_NONE;

    for (;;) {
        if (p->len == 0u) {
            return events;
        }
        if (!crsf_is_sync_byte(p->buf[0])) {
            drop_front(p, 1);
            ++p->bytes_discarded;
            continue;
        }
        if (p->len < 2u) {
            return events;
        }
        const uint8_t flen = p->buf[1];
        if (flen < CRSF_LEN_FIELD_MIN || flen > CRSF_LEN_FIELD_MAX) {
            /* That "sync" byte was data. Drop it and look again. */
            drop_front(p, 1);
            ++p->bytes_discarded;
            continue;
        }
        const size_t total = (size_t)flen + 2u;
        if (p->len < total) {
            return events;   /* wait for the rest */
        }
        const uint8_t crc = crsf_crc8(&p->buf[2], (size_t)flen - 1u);
        if (crc != p->buf[total - 1u]) {
            /* Drop ONE byte, not the frame: a real frame may already be sitting
             * inside what was mistaken for this one. */
            ++p->crc_errors;
            drop_front(p, 1);
            continue;
        }
        events |= handle_frame(p, p->buf[2], &p->buf[3], (size_t)flen - 2u);
        drop_front(p, total);
    }
}

unsigned crsf_parser_feed(crsf_parser_t *p, const uint8_t *data, size_t n)
{
    unsigned events = CRSF_EVT_NONE;

    for (size_t i = 0; i < n; ++i) {
        ++p->bytes;
        if (p->len >= CRSF_MAX_FRAME_LEN) {
            /* Unreachable in practice — process() never leaves a full buffer
             * waiting, since the largest valid frame fits exactly. Kept so a
             * logic error degrades to a resync instead of an overflow. */
            drop_front(p, 1);
            ++p->bytes_discarded;
        }
        p->buf[p->len++] = data[i];
        events |= process(p);
    }
    return events;
}
