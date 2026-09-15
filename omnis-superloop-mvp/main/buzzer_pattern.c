/**
 * @file    buzzer_pattern.c
 * @brief   Buzzer patterns — implementation. See buzzer_pattern.h.
 */

#include <string.h>

#include "buzzer_pattern.h"
#include "fault.h"

typedef struct {
    uint32_t    bits;     /* bit i = slot i on; slot 0 is the first 50 ms */
    uint8_t     slots;
    bool        repeat;
    const char *name;
} buzz_def_t;

static const buzz_def_t k_defs[BUZZ_PATTERN_COUNT] = {
    [BUZZ_SILENT]          = { 0x000u,  1u, false, "silent"          },
    [BUZZ_BOOT_OK]         = { 0x003u,  2u, false, "boot-ok"         }, /* 100 on               */
    [BUZZ_CAL_HOLD_STILL]  = { 0x001u, 20u, true,  "hold-still"      }, /* 50 on, 950 off       */
    [BUZZ_ARMED]           = { 0x033u,  6u, false, "armed"           }, /* 100 on 100 off 100 on */
    [BUZZ_DISARMED]        = { 0x0FFu,  8u, false, "disarmed"        }, /* 400 on               */
    [BUZZ_ARM_REJECTED]    = { 0x015u,  5u, false, "arm-rejected"    }, /* 50/50 x3             */
    [BUZZ_BALANCE_ENGAGED] = { 0x01Du,  5u, false, "balance-engaged" }, /* 50 on 50 off 150 on  */
    [BUZZ_FAULT_IMU]       = { 0x001u,  1u, true,  "fault-imu"       }, /* continuous           */
    [BUZZ_FAULT_RC_LINK]   = { 0x005u, 10u, true,  "fault-rc-link"   }, /* double chirp / 500ms */
    [BUZZ_FAULT_TILT]      = { 0x3FFu, 20u, true,  "fault-tilt"      }, /* 500 on 500 off       */
    [BUZZ_FAULT_CONFIG]    = { 0x53Fu, 20u, true,  "fault-config"    }, /* long-short-short     */
    [BUZZ_FAULT_OVERRUN]   = { 0x015u, 20u, true,  "fault-overrun"   }, /* triple chirp / s     */
};

static const buzz_def_t *def_of(buzz_pattern_t p)
{
    return ((unsigned)p < (unsigned)BUZZ_PATTERN_COUNT) ? &k_defs[p] : &k_defs[BUZZ_SILENT];
}

bool buzz_pattern_level(buzz_pattern_t p, uint32_t t_ms)
{
    const buzz_def_t *d = def_of(p);
    uint32_t slot = t_ms / BUZZ_SLOT_MS;
    if (d->repeat) {
        slot %= d->slots;
    } else if (slot >= d->slots) {
        return false;
    }
    return ((d->bits >> slot) & 1u) != 0u;
}

bool buzz_pattern_done(buzz_pattern_t p, uint32_t t_ms)
{
    const buzz_def_t *d = def_of(p);
    return !d->repeat && t_ms >= (uint32_t)d->slots * BUZZ_SLOT_MS;
}

uint32_t buzz_pattern_length_ms(buzz_pattern_t p)
{
    return (uint32_t)def_of(p)->slots * BUZZ_SLOT_MS;
}

bool buzz_pattern_repeats(buzz_pattern_t p)
{
    return def_of(p)->repeat;
}

const char *buzz_pattern_name(buzz_pattern_t p)
{
    return def_of(p)->name;
}

buzz_pattern_t buzz_pattern_for_faults(uint32_t active)
{
    if (active & (FAULT_IMU_DISAGREE | FAULT_IMU_COMM | FAULT_IMU_INIT | FAULT_IMU_CAL)) {
        return BUZZ_FAULT_IMU;
    }
    if (active & (FAULT_PARAMS | FAULT_STEP_INIT)) {
        return BUZZ_FAULT_CONFIG;
    }
    if (active & FAULT_TILT) {
        return BUZZ_FAULT_TILT;
    }
    if (active & FAULT_LOOP_OVERRUN) {
        return BUZZ_FAULT_OVERRUN;
    }
    if (active & FAULT_RC_LINK) {
        return BUZZ_FAULT_RC_LINK;
    }
    return BUZZ_SILENT;
}

void buzz_player_init(buzz_player_t *pl)
{
    memset(pl, 0, sizeof *pl);
    pl->event = BUZZ_SILENT;
    pl->fault = BUZZ_SILENT;
}

void buzz_player_event(buzz_player_t *pl, buzz_pattern_t p, uint32_t now_ms)
{
    pl->event          = p;
    pl->event_start_ms = now_ms;
    pl->event_active   = (p != BUZZ_SILENT);
    if (!pl->event_active) {
        pl->fault_start_ms = now_ms;   /* a cancelled event hands straight back */
    }
}

bool buzz_player_level(buzz_player_t *pl, uint32_t active_faults, uint32_t now_ms)
{
    if (pl->event_active) {
        const uint32_t t = now_ms - pl->event_start_ms;
        if (!buzz_pattern_done(pl->event, t)) {
            return buzz_pattern_level(pl->event, t);
        }
        pl->event_active   = false;
        pl->fault_start_ms = now_ms;   /* resume the fault pattern from its start */
    }

    const buzz_pattern_t want = buzz_pattern_for_faults(active_faults);
    if (want != pl->fault) {
        pl->fault          = want;
        pl->fault_start_ms = now_ms;
    }
    return buzz_pattern_level(pl->fault, now_ms - pl->fault_start_ms);
}
