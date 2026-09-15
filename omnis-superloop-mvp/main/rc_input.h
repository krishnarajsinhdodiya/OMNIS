/**
 * @file    rc_input.h
 * @brief   CRSF channels -> what the robot was asked to do.
 *
 * Pure: no UART, no clock, no IDF. Turns 16 raw channel values into sticks,
 * switches and the arming request, using the channel map in omnis_params.
 * Host-tested in test/test_crsf.c.
 *
 * WHAT IS DECODED (omnis-info.md §7a):
 *
 *   throttle, pitch, roll, yaw   sticks, [-1, +1], after a rescaled deadzone
 *   arm switch (ch 6)            HIGH = motors permitted. LOW or MID = not.
 *   drive mode (ch 8, 3-pos)     LOW flat, MID balance, HIGH auto-detect
 *   speed limiter (ch 9, 3-pos)  LOW/MID/HIGH -> params.control scale
 *   S1 pot (ch 10)               [0, 1], for the optional kp tuning knob
 *
 * Stick POLARITY is not applied here — it lives in drive_rc_to_body(), which is
 * the one place the kinematics reference allows it.
 *
 * SAFETY PROPERTIES this layer guarantees, all tested:
 *   - A switch in its middle position never reads as "armed", so a 3-position
 *     switch mapped to the arm channel by mistake cannot arm the robot.
 *   - A NaN or out-of-range channel reads as centred / not armed.
 *   - rc_command_neutral() is the command used whenever the link is down.
 */

#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <stdbool.h>
#include <stdint.h>

#include "crsf_parser.h"
#include "drive.h"
#include "omnis_params.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RC_SW_LOW  = 0,
    RC_SW_MID  = 1,
    RC_SW_HIGH = 2,
} rc_switch_pos_t;

typedef struct {
    rc_sticks_t     sticks;            /**< deadzoned, NOT polarity-corrected     */
    bool            arm_request;       /**< arm switch in the "run" position      */
    rc_switch_pos_t drive_mode;        /**< LOW flat, MID balance, HIGH auto      */
    rc_switch_pos_t speed;             /**< speed-limiter position                */
    float           tune_pot;          /**< [0, 1]                                */
    bool            sticks_centered;   /**< all four within params center tolerance */
} rc_command_t;

/**
 * @brief Rescaled deadzone.
 *
 * Inside +-dz the output is exactly 0; outside it ramps linearly from 0 to +-1,
 * so there is no step at the edge of the deadzone (a plain "zero it if small"
 * would make the robot lurch the instant the stick leaves the band).
 */
float rc_deadzone(float v, float dz);

/** [-1, +1] -> LOW (< -1/3), MID, HIGH (> +1/3). NaN reads LOW. */
rc_switch_pos_t rc_three_pos(float unit);

/** Speed-limiter position -> scale factor from params.control. */
float rc_speed_scale(rc_switch_pos_t pos, const omnis_control_t *c);

/** Decode one set of 16 channels. */
void rc_input_decode(const uint16_t channels[CRSF_NUM_CHANNELS],
                     const omnis_channel_map_t *map,
                     const omnis_rc_params_t *rc,
                     rc_command_t *out);

/** Centred sticks, arm not requested, flat mode, lowest speed. */
void rc_command_neutral(rc_command_t *out);

const char *rc_switch_name(rc_switch_pos_t pos);

#ifdef __cplusplus
}
#endif

#endif /* RC_INPUT_H */
