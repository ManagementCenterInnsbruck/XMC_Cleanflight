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

#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"

#include "drivers/timer.h"
#include "drivers/timer_def.h"
#include "drivers/dma.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
	{ .tim = &XMCTimer[0], .tag = IO_TAG(P13), .usageFlags = TIM_USE_MOTOR, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
	{ .tim = &XMCTimer[1], .tag = IO_TAG(P12), .usageFlags = TIM_USE_MOTOR, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
	{ .tim = &XMCTimer[3], .tag = IO_TAG(P10), .usageFlags = TIM_USE_MOTOR, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
	{ .tim = &XMCTimer[2], .tag = IO_TAG(P11), .usageFlags = TIM_USE_MOTOR, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
};

