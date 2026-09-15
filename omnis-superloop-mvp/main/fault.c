/**
 * @file    fault.c
 * @brief   Fault latch — implementation. See fault.h.
 */

#include <stdio.h>
#include <string.h>

#include "fault.h"

/* Most severe first. Sensor-integrity faults outrank everything: a robot that
 * cannot trust its own attitude must not be told anything else is the problem.
 * Link loss ranks last because it is the one fault the operator fixes by
 * walking closer. */
static const struct {
    uint32_t    bit;
    const char *name;
} k_order[] = {
    { FAULT_IMU_DISAGREE, "IMU_DISAGREE" },
    { FAULT_IMU_COMM,     "IMU_COMM"     },
    { FAULT_IMU_INIT,     "IMU_INIT"     },
    { FAULT_IMU_CAL,      "IMU_CAL"      },
    { FAULT_STEP_INIT,    "STEP_INIT"    },
    { FAULT_PARAMS,       "PARAMS"       },
    { FAULT_TILT,         "TILT"         },
    { FAULT_LOOP_OVERRUN, "LOOP_OVERRUN" },
    { FAULT_RC_LINK,      "RC_LINK"      },
};
#define K_COUNT (sizeof k_order / sizeof k_order[0])

void fault_init(fault_state_t *f)
{
    memset(f, 0, sizeof *f);
}

bool fault_raise(fault_state_t *f, uint32_t bits)
{
    bits &= FAULT_MASK_ALL;
    const uint32_t fresh = bits & ~f->active;
    f->active |= bits;
    f->ever   |= bits;
    if (fresh != 0u) {
        ++f->raise_count;
    }
    return fresh != 0u;
}

void fault_set_condition(fault_state_t *f, uint32_t bits, bool present)
{
    bits &= FAULT_MASK_LIVE;
    if (bits == 0u) {
        return;
    }
    if (present) {
        (void)fault_raise(f, bits);
    } else {
        f->active &= ~bits;
    }
}

void fault_clear_on_disarm(fault_state_t *f)
{
    f->active &= ~FAULT_MASK_UNTIL_DISARM;
}

uint32_t fault_most_severe(uint32_t active)
{
    for (size_t i = 0; i < K_COUNT; ++i) {
        if (active & k_order[i].bit) {
            return k_order[i].bit;
        }
    }
    return 0u;
}

const char *fault_name(uint32_t bit)
{
    for (size_t i = 0; i < K_COUNT; ++i) {
        if (bit == k_order[i].bit) {
            return k_order[i].name;
        }
    }
    return bit ? "UNKNOWN" : "NONE";
}

size_t fault_describe(uint32_t active, char *buf, size_t len)
{
    if (buf == NULL || len == 0u) {
        return 0u;
    }
    if ((active & FAULT_MASK_ALL) == 0u) {
        const int n = snprintf(buf, len, "none");
        return (n < 0) ? 0u : strlen(buf);
    }
    size_t used = 0u;
    buf[0] = '\0';
    for (size_t i = 0; i < K_COUNT && used + 1u < len; ++i) {
        if (active & k_order[i].bit) {
            const int n = snprintf(buf + used, len - used, "%s%s",
                                   used ? "|" : "", k_order[i].name);
            if (n < 0) {
                break;
            }
            used += (size_t)n;
            if (used >= len) {
                used = len - 1u;   /* truncated; snprintf already terminated it */
            }
        }
    }
    return used;
}
