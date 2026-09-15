/**
 * @file    test_crsf.c
 * @brief   Host-side verification of the CRSF parser and RC input decoder.
 *
 * Golden frames in crsf_golden.h come from an independent Python
 * implementation, so a packing or CRC bug cannot pass by agreeing with itself.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "crsf_parser.h"
#include "rc_input.h"
#include "omnis_params.h"
#include "crsf_golden.h"

static int g_fail = 0, g_pass = 0;

static void chk(const char *name, double got, double want, double tol)
{
    if (fabs(got - want) <= tol) { ++g_pass; printf("  ok    %-50s %10.4f\n", name, got); }
    else { ++g_fail; printf("  FAIL  %-50s %10.4f (want %.4f)\n", name, got, want); }
}

static void chk_true(const char *name, int cond)
{
    if (cond) { ++g_pass; printf("  ok    %-50s\n", name); }
    else      { ++g_fail; printf("  FAIL  %-50s\n", name); }
}

static void set_all(uint16_t ch[16], uint16_t v) { for (int i = 0; i < 16; ++i) ch[i] = v; }

static void case_crc_and_packing(void)
{
    puts("\nCRC-8/DVB-S2 and channel packing");
    chk("check value crc8(\"123456789\") (published: 0xBC)",
        crsf_crc8((const uint8_t *)"123456789", 9), 0xBC, 0);

    uint16_t centre[16]; set_all(centre, 992);
    uint8_t f[CRSF_MAX_FRAME_LEN];
    size_t n = crsf_build_rc_frame(centre, f);
    chk("RC frame length", (double)n, 26, 0);
    chk_true("centre frame == independent golden bytes", n == sizeof FRAME_CENTRE &&
             memcmp(f, FRAME_CENTRE, n) == 0);

    n = crsf_build_rc_frame(PATTERN, f);
    chk_true("pattern frame == independent golden bytes (incl. 0, 2047)",
             n == sizeof FRAME_PATTERN && memcmp(f, FRAME_PATTERN, n) == 0);

    uint16_t back[16];
    crsf_unpack_channels(&FRAME_PATTERN[3], back);
    int same = 1;
    for (int i = 0; i < 16; ++i) same &= (back[i] == PATTERN[i]);
    chk_true("unpack(golden pattern) recovers all 16 channels", same);
}

static void case_parser_framing(void)
{
    puts("\nParser framing and resync");
    crsf_parser_t p;

    crsf_parser_init(&p);
    unsigned ev = crsf_parser_feed(&p, FRAME_PATTERN, sizeof FRAME_PATTERN);
    chk_true("whole frame -> CHANNELS event", ev & CRSF_EVT_CHANNELS);
    chk_true("  channels match", memcmp(p.channels, PATTERN, sizeof PATTERN) == 0);

    crsf_parser_init(&p);
    ev = 0;
    for (size_t i = 0; i < sizeof FRAME_PATTERN; ++i) ev |= crsf_parser_feed(&p, &FRAME_PATTERN[i], 1);
    chk_true("byte-at-a-time -> CHANNELS event", ev & CRSF_EVT_CHANNELS);
    chk("  exactly one RC frame", p.rc_frames, 1, 0);

    crsf_parser_init(&p);
    uint8_t two[52];
    memcpy(two, FRAME_CENTRE, 26); memcpy(two + 26, FRAME_PATTERN, 26);
    crsf_parser_feed(&p, two, sizeof two);
    chk("two back-to-back frames in one feed -> 2 frames", p.rc_frames, 2, 0);
    chk_true("  last one wins", memcmp(p.channels, PATTERN, sizeof PATTERN) == 0);

    /* Garbage, including fake sync bytes and an impossible length, then a frame. */
    crsf_parser_init(&p);
    const uint8_t junk[] = { 0x00, 0xC8, 0x01, 0xFF, 0xC8, 0x7F, 0x13, 0xEE, 0x00 };
    crsf_parser_feed(&p, junk, sizeof junk);
    ev = crsf_parser_feed(&p, FRAME_PATTERN, sizeof FRAME_PATTERN);
    chk_true("garbage then frame -> frame still parsed", ev & CRSF_EVT_CHANNELS);
    chk_true("  garbage bytes were counted", p.bytes_discarded > 0);

    /* Corrupted CRC: rejected, and the NEXT frame must still parse. */
    crsf_parser_init(&p);
    uint8_t bad[26]; memcpy(bad, FRAME_CENTRE, 26); bad[25] ^= 0x5A;
    ev = crsf_parser_feed(&p, bad, sizeof bad);
    chk_true("bad CRC -> no event", ev == CRSF_EVT_NONE);
    chk("  crc_errors counted", p.crc_errors >= 1 ? 1 : 0, 1, 0);
    ev = crsf_parser_feed(&p, FRAME_PATTERN, sizeof FRAME_PATTERN);
    chk_true("  frame after bad CRC still parses", ev & CRSF_EVT_CHANNELS);

    /* The case single-byte resync exists for: a TRUNCATED frame followed by a
     * good one. The truncated header claims 24 more bytes, so the good frame is
     * swallowed into it, fails CRC, and must be recovered from the buffer. */
    crsf_parser_init(&p);
    uint8_t stream[36];
    memcpy(stream, FRAME_CENTRE, 10);          /* truncated: only 10 of 26 bytes */
    memcpy(stream + 10, FRAME_PATTERN, 26);
    ev = crsf_parser_feed(&p, stream, sizeof stream);
    chk_true("truncated frame then good frame -> good frame recovered",
             (ev & CRSF_EVT_CHANNELS) && memcmp(p.channels, PATTERN, sizeof PATTERN) == 0);

    /* Link statistics. */
    crsf_parser_init(&p);
    ev = crsf_parser_feed(&p, FRAME_LINK, sizeof FRAME_LINK);
    chk_true("link statistics frame -> LINK event", ev & CRSF_EVT_LINK_STATS);
    chk("  uplink LQ %", p.link.uplink_lq, 87, 0);
    chk("  uplink RSSI (stored inverted)", p.link.uplink_rssi_1, 65, 0);
    chk("  uplink SNR is signed", p.link.uplink_snr, -3, 0);
    chk("  downlink SNR is signed", p.link.downlink_snr, -8, 0);
}

static void case_units(void)
{
    puts("\nChannel value -> unit");
    chk("172  -> -1", crsf_channel_to_unit(172),  -1.0, 1e-6);
    chk("992  ->  0", crsf_channel_to_unit(992),   0.0, 1e-6);
    chk("1811 -> +1", crsf_channel_to_unit(1811),  1.0, 1e-6);
    chk("2047 clamps to +1", crsf_channel_to_unit(2047), 1.0, 1e-6);
    chk("0    clamps to -1", crsf_channel_to_unit(0),   -1.0, 1e-6);
    chk("1401.5 is half stick", crsf_channel_to_unit(1402), (1402.0 - 992.0) / 819.0, 1e-5);

    puts("\nDeadzone and switches");
    chk("deadzone: 0.03 inside 0.04 -> 0", rc_deadzone(0.03f, 0.04f), 0.0, 1e-6);
    chk("deadzone: 0.52 -> rescaled 0.5", rc_deadzone(0.52f, 0.04f), 0.5, 1e-5);
    chk("deadzone: -1 -> -1 (full scale preserved)", rc_deadzone(-1.0f, 0.04f), -1.0, 1e-6);
    chk("deadzone: just outside is ~0 (no step)", rc_deadzone(0.0401f, 0.04f), 0.0, 1e-3);
    chk_true("deadzone: NaN -> 0", rc_deadzone(NAN, 0.04f) == 0.0f);
    chk_true("3-pos -1 -> LOW",  rc_three_pos(-1.0f) == RC_SW_LOW);
    chk_true("3-pos  0 -> MID",  rc_three_pos(0.0f)  == RC_SW_MID);
    chk_true("3-pos +1 -> HIGH", rc_three_pos(1.0f)  == RC_SW_HIGH);
    chk_true("3-pos NaN -> LOW", rc_three_pos(NAN)   == RC_SW_LOW);
}

static void case_decode(void)
{
    puts("\nrc_input_decode with the default channel map");
    omnis_params_t prm;
    omnis_params_defaults(&prm);
    const omnis_channel_map_t *m = &prm.channel_map;
    uint16_t ch[16];
    rc_command_t c;

    set_all(ch, 992);
    ch[m->throttle - 1]      = 1811;
    ch[m->roll - 1]          = 172;
    ch[m->kill_switch - 1]   = 1811;
    ch[m->drive_mode - 1]    = 992;
    ch[m->speed_limiter - 1] = 172;
    ch[m->tune_pot - 1]      = 1811;
    rc_input_decode(ch, m, &prm.rc, &c);
    chk("throttle full forward", c.sticks.throttle, 1.0, 1e-5);
    chk("roll full (raw -1, polarity applied later)", c.sticks.roll, -1.0, 1e-5);
    chk("pitch centred", c.sticks.pitch, 0.0, 1e-6);
    chk_true("arm switch HIGH -> arm requested", c.arm_request);
    chk_true("drive mode MID", c.drive_mode == RC_SW_MID);
    chk_true("speed LOW", c.speed == RC_SW_LOW);
    chk("tune pot full", c.tune_pot, 1.0, 1e-5);
    chk_true("sticks NOT centred", !c.sticks_centered);

    set_all(ch, 992);
    ch[m->kill_switch - 1] = 172;
    rc_input_decode(ch, m, &prm.rc, &c);
    chk_true("all centred -> centred", c.sticks_centered);
    chk_true("arm switch LOW -> not armed", !c.arm_request);

    ch[m->kill_switch - 1] = 992;
    rc_input_decode(ch, m, &prm.rc, &c);
    chk_true("arm switch MID -> NOT armed (3-pos switch safety)", !c.arm_request);
    prm.channel_map.arm_switch_invert = true;
    rc_input_decode(ch, m, &prm.rc, &c);
    chk_true("arm switch MID, inverted -> still NOT armed", !c.arm_request);
    ch[m->kill_switch - 1] = 172;
    rc_input_decode(ch, m, &prm.rc, &c);
    chk_true("arm switch LOW, inverted -> armed", c.arm_request);

    chk("speed scale LOW",  rc_speed_scale(RC_SW_LOW,  &prm.control), 0.3, 1e-6);
    chk("speed scale MID",  rc_speed_scale(RC_SW_MID,  &prm.control), 0.6, 1e-6);
    chk("speed scale HIGH", rc_speed_scale(RC_SW_HIGH, &prm.control), 1.0, 1e-6);

    rc_command_neutral(&c);
    chk_true("neutral command: not armed, centred, flat, slow",
             !c.arm_request && c.sticks_centered && c.drive_mode == RC_SW_LOW &&
             c.speed == RC_SW_LOW && c.sticks.throttle == 0.0f);

    /* An impossible channel number reads centred rather than reading off the end. */
    prm.channel_map.throttle = 0;
    set_all(ch, 1811);
    rc_input_decode(ch, &prm.channel_map, &prm.rc, &c);
    chk("channel number 0 reads centred", c.sticks.throttle, 0.0, 1e-6);
}

int main(void)
{
    puts("OMNIS CRSF parser + RC input verification");
    case_crc_and_packing();
    case_parser_framing();
    case_units();
    case_decode();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
