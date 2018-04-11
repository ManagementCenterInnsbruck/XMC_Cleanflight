/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "config/parameter_group.h"
#include "drivers/io_types.h"

#define LED_NUMBER 3

typedef struct statusLedConfig_s {
    ioTag_t ledTags[LED_NUMBER];
    uint8_t polarity;
} statusLedConfig_t;

PG_DECLARE(statusLedConfig_t, statusLedConfig);

// Helpful macros
#ifdef LED0
# define LED0_TOGGLE              ledToggle(0)
# define LED0_OFF                 ledSet(0, false)
# define LED0_ON                  ledSet(0, true)
#else
# define LED0_TOGGLE              do {} while(0)
# define LED0_OFF                 do {} while(0)
# define LED0_ON                  do {} while(0)
#endif

#ifdef LED1
# define LED1_TOGGLE              ledToggle(1)
# define LED1_OFF                 ledSet(1, false)
# define LED1_ON                  ledSet(1, true)
#else
# define LED1_TOGGLE              do {} while(0)
# define LED1_OFF                 do {} while(0)
# define LED1_ON                  do {} while(0)
#endif

#ifdef LED2
# define LED2_TOGGLE              ledToggle(2)
# define LED2_OFF                 ledSet(2, false)
# define LED2_ON                  ledSet(2, true)
#else
# define LED2_TOGGLE              do {} while(0)
# define LED2_OFF                 do {} while(0)
# define LED2_ON                  do {} while(0)
#endif

void ledInit(const statusLedConfig_t *statusLedConfig);
void ledToggle(int led);
void ledSet(int led, bool state);
