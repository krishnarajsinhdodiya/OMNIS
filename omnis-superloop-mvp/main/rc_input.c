/**
 * @file    rc_input.c
 * @brief   CRSF channels -> robot command. See rc_input.h.
 */

#include <math.h>

#include "rc_input.h"

float rc_deadzone(float v, float dz)
{
    if (!isfinite(v)) {
        return 0.0f;
    }
    if (dz <= 0.0f) {
        return v;
    }
    if (dz >= 1.0f) {
        return 0.0f;
    }
    const float a = fabsf(v);
    if (a <= dz) {
        return 0.0f;
    }
    float out = (a - dz) / (1.0f - dz);
    if (out > 1.0f) {
        out = 1.0f;
    }
    return (v < 0.0f) ? -out : out;
}

rc_switch_pos_t rc_three_pos(float unit)
{
    if (!isfinite(unit) || unit < -0.33f) {
        return RC_SW_LOW;
    }
    if (unit > 0.33f) {
        return RC_SW_HIGH;
    }
    return RC_SW_MID;
}

float rc_speed_scale(rc_switch_pos_t pos, const omnis_control_t *c)
{
    switch (pos) {
        case RC_SW_HIGH: return c->speed_high;
        case RC_SW_MID:  return c->speed_med;
        case RC_SW_LOW:
        default:         return c->speed_low;
    }
}

const char *rc_switch_name(rc_switch_pos_t pos)
{
    switch (pos) {
        case RC_SW_HIGH: return "HIGH";
        case RC_SW_MID:  return "MID";
        case RC_SW_LOW:
        default:         return "LOW";
    }
}

/* 1-based channel number -> [-1, +1]. An invalid channel number reads centred,
 * which for every consumer here is the safe value. */
static float channel_unit(const uint16_t ch[CRSF_NUM_CHANNELS], uint8_t one_based)
{
    if (one_based < 1u || one_based > CRSF_NUM_CHANNELS) {
        return 0.0f;
    }
    return crsf_channel_to_unit(ch[one_based - 1u]);
}

void rc_command_neutral(rc_command_t *out)
{
    out->sticks.throttle = 0.0f;
    out->sticks.pitch    = 0.0f;
    out->sticks.roll     = 0.0f;
    out->sticks.yaw      = 0.0f;
    out->arm_request     = false;
    out->drive_mode      = RC_SW_LOW;
    out->speed           = RC_SW_LOW;
    out->tune_pot        = 0.0f;
    out->sticks_centered = true;
}

void rc_input_decode(const uint16_t channels[CRSF_NUM_CHANNELS],
                     const omnis_channel_map_t *map,
                     const omnis_rc_params_t *rc,
                     rc_command_t *out)
{
    const float dz = rc->stick_deadzone;

    out->sticks.throttle = rc_deadzone(channel_unit(channels, map->throttle), dz);
    out->sticks.pitch    = rc_deadzone(channel_unit(channels, map->pitch),    dz);
    out->sticks.roll     = rc_deadzone(channel_unit(channels, map->roll),     dz);
    out->sticks.yaw      = rc_deadzone(channel_unit(channels, map->yaw),      dz);

    /* Arm switch: requires a DECISIVE position. The middle band (-0.5..+0.5)
     * is never "armed", so a 3-position switch on this channel cannot arm the
     * robot from its centre detent, whichever way the invert flag is set. */
    const float arm = channel_unit(channels, map->kill_switch);
    out->arm_request = map->arm_switch_invert ? (arm < -0.5f) : (arm > 0.5f);

    out->drive_mode = rc_three_pos(channel_unit(channels, map->drive_mode));
    out->speed      = rc_three_pos(channel_unit(channels, map->speed_limiter));

    float pot = 0.5f * (channel_unit(channels, map->tune_pot) + 1.0f);
    if (pot < 0.0f) pot = 0.0f;
    if (pot > 1.0f) pot = 1.0f;
    out->tune_pot = pot;

    const float tol = rc->center_tolerance;
    out->sticks_centered = fabsf(out->sticks.throttle) <= tol
                        && fabsf(out->sticks.pitch)    <= tol
                        && fabsf(out->sticks.roll)     <= tol
                        && fabsf(out->sticks.yaw)      <= tol;
}
