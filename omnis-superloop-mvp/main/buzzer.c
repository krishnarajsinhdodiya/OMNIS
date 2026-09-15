/**
 * @file    buzzer.c
 * @brief   Buzzer GPIO glue. See buzzer.h.
 */

#include "driver/gpio.h"

#include "buzzer.h"
#include "omnis_pins.h"

static buzz_player_t s_player;
static int           s_level = -1;

void buzzer_init(void)
{
    buzz_player_init(&s_player);
    gpio_set_level(PIN_BUZZER, BUZZER_OFF);
    s_level = BUZZER_OFF;
}

void buzzer_event(buzz_pattern_t p, uint32_t now_ms)
{
    buzz_player_event(&s_player, p, now_ms);
}

void buzzer_service(uint32_t active_faults, uint32_t now_ms)
{
    const int level = buzz_player_level(&s_player, active_faults, now_ms)
                    ? BUZZER_ON : BUZZER_OFF;
    if (level != s_level) {
        gpio_set_level(PIN_BUZZER, level);
        s_level = level;
    }
}
