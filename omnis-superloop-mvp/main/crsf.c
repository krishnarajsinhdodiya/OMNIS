/**
 * @file    crsf.c
 * @brief   CRSF receiver glue: UART1, RX-only, pin auto-detect. See crsf.h.
 */

#include "driver/uart.h"
#include "esp_log.h"

#include "crsf.h"
#include "omnis_pins.h"

static const char *TAG = "crsf";

#define CRSF_UART          UART_NUM_1
#define CRSF_RX_BUF_BYTES  1024          /* ~40 frames: a long stall cannot overflow it */
#define CRSF_READ_CHUNK    128u
#define CRSF_MAX_CHUNKS    4             /* bound the parsing work done in one tick */

static const omnis_params_t *s_params = NULL;
static crsf_parser_t         s_parser;
static int                   s_rx_pin       = PIN_CRSF_ESP_RX;
static bool                  s_locked       = false;
static int64_t               s_probe_since  = 0;
static bool                  s_have_rc      = false;
static int64_t               s_last_rc_us   = 0;
static bool                  s_have_link    = false;
static int64_t               s_last_link_us = 0;

bool crsf_init(const omnis_params_t *p)
{
    s_params = p;
    crsf_parser_init(&s_parser);

    const uart_config_t cfg = {
        .baud_rate  = (int)p->rc.crsf_baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(CRSF_UART, &cfg);
    if (err == ESP_OK) {
        /* RX only: TX stays unrouted, see crsf.h. */
        err = uart_set_pin(CRSF_UART, UART_PIN_NO_CHANGE, s_rx_pin,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err == ESP_OK) {
        /* No TX buffer, no event queue: nothing of ours sits between the ISR's
         * ring buffer and the zero-timeout read in crsf_poll(). */
        err = uart_driver_install(CRSF_UART, CRSF_RX_BUF_BYTES, 0, 0, NULL, 0);
    }
    if (err == ESP_OK) {
        /* Hand bytes to the ring buffer after ~3 idle symbols rather than the
         * default, so a 26-byte frame reaches the parser in well under a tick. */
        err = uart_set_rx_timeout(CRSF_UART, 3);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART1 setup failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "listening at %u baud on GPIO%d (RX-only); will also probe GPIO%d",
             (unsigned)p->rc.crsf_baud, s_rx_pin, (int)PIN_CRSF_ESP_TX);
    return true;
}

static void swap_rx_pin(int64_t now_us)
{
    s_rx_pin = (s_rx_pin == PIN_CRSF_ESP_RX) ? PIN_CRSF_ESP_TX : PIN_CRSF_ESP_RX;
    (void)uart_set_pin(CRSF_UART, UART_PIN_NO_CHANGE, s_rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    (void)uart_flush_input(CRSF_UART);
    s_parser.len  = 0;           /* half a frame from the other pin is garbage */
    s_probe_since = now_us;
    ESP_LOGW(TAG, "no CRSF frames yet - now listening on GPIO%d", s_rx_pin);
}

void crsf_poll(int64_t now_us)
{
    if (s_params == NULL) {
        return;
    }

    uint8_t chunk[CRSF_READ_CHUNK];
    for (int i = 0; i < CRSF_MAX_CHUNKS; ++i) {
        const int n = uart_read_bytes(CRSF_UART, chunk, sizeof chunk, 0);
        if (n <= 0) {
            break;
        }
        const unsigned ev = crsf_parser_feed(&s_parser, chunk, (size_t)n);
        if (ev & CRSF_EVT_CHANNELS) {
            s_have_rc    = true;
            s_last_rc_us = now_us;
            if (!s_locked) {
                s_locked = true;
                ESP_LOGI(TAG, "CRSF locked on GPIO%d", s_rx_pin);
            }
        }
        if (ev & CRSF_EVT_LINK_STATS) {
            s_have_link    = true;
            s_last_link_us = now_us;
        }
        if ((size_t)n < sizeof chunk) {
            break;   /* drained */
        }
    }

    if (!s_locked) {
        if (s_probe_since == 0) {
            s_probe_since = now_us;
        } else if (now_us - s_probe_since > (int64_t)s_params->rc.pin_probe_ms * 1000) {
            swap_rx_pin(now_us);
        }
    }
}

bool crsf_link_ok(int64_t now_us)
{
    if (s_params == NULL || !s_have_rc) {
        return false;
    }
    if (now_us - s_last_rc_us > (int64_t)s_params->rc_timeout_us) {
        return false;
    }
    /* Optional LQ floor, trusted only while the statistics are recent. */
    const uint8_t floor_lq = s_params->rc.min_link_quality;
    if (floor_lq > 0u && s_have_link && (now_us - s_last_link_us) < 1000000
        && s_parser.link.uplink_lq < floor_lq) {
        return false;
    }
    return true;
}

bool crsf_ever_locked(void)
{
    return s_locked;
}

const uint16_t *crsf_channels(void)
{
    return s_parser.channels;
}

void crsf_get_status(int64_t now_us, crsf_status_t *out)
{
    out->locked          = s_locked;
    out->rx_pin          = s_rx_pin;
    out->uplink_lq       = s_parser.link.uplink_lq;
    out->uplink_rssi_dbm = -(int)s_parser.link.uplink_rssi_1;
    out->rc_frames       = s_parser.rc_frames;
    out->crc_errors      = s_parser.crc_errors;
    out->bytes           = s_parser.bytes;
    out->ms_since_rc     = s_have_rc ? (now_us - s_last_rc_us) / 1000 : -1;
}
