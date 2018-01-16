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
		// Motor 1
		{ .tim = &XMCTimer[16], .tag = IO_TAG(P05),  .usageFlags = TIM_USE_MOTOR, .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[16], .tag = IO_TAG(P010), .usageFlags = TIM_USE_MOTOR, .channel=1, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[17], .tag = IO_TAG(P04),  .usageFlags = TIM_USE_MOTOR, .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[17], .tag = IO_TAG(P09),  .usageFlags = TIM_USE_MOTOR, .channel=1, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[18], .tag = IO_TAG(P03),  .usageFlags = TIM_USE_MOTOR, .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[18], .tag = IO_TAG(P29),  .usageFlags = TIM_USE_MOTOR, .channel=1, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[19], .tag = IO_TAG(P06),  .usageFlags = 0,             .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[19], .tag = IO_TAG(P28),  .usageFlags = 0,             .channel=1, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		// Motor 2
		{ .tim = &XMCTimer[20], .tag = IO_TAG(P115), .usageFlags = TIM_USE_MOTOR, .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[20], .tag = IO_TAG(P57),  .usageFlags = TIM_USE_MOTOR, .channel=1, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[21], .tag = IO_TAG(P114), .usageFlags = TIM_USE_MOTOR, .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[21], .tag = IO_TAG(P55),  .usageFlags = TIM_USE_MOTOR, .channel=1, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[22], .tag = IO_TAG(P113), .usageFlags = TIM_USE_MOTOR, .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[22], .tag = IO_TAG(P53),  .usageFlags = TIM_USE_MOTOR, .channel=1, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[23], .tag = IO_TAG(P61),  .usageFlags = 0,             .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[23], .tag = IO_TAG(P51),  .usageFlags = 0,             .channel=1, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		// Motor 3
		{ .tim = &XMCTimer[0], .tag = IO_TAG(P13), .usageFlags = TIM_USE_MOTOR, .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[1], .tag = IO_TAG(P12), .usageFlags = TIM_USE_MOTOR, .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[2], .tag = IO_TAG(P11), .usageFlags = TIM_USE_MOTOR, .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[3], .tag = IO_TAG(P10), .usageFlags = TIM_USE_MOTOR, .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[4], .tag = IO_TAG(P25), .usageFlags = TIM_USE_MOTOR, .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[5], .tag = IO_TAG(P24), .usageFlags = TIM_USE_MOTOR, .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[6], .tag = IO_TAG(P23), .usageFlags = 0,             .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[7], .tag = IO_TAG(P22), .usageFlags = 0,             .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		// Motor 4
		{ .tim = &XMCTimer[8],  .tag = IO_TAG(P36), .usageFlags = TIM_USE_MOTOR, .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[9],  .tag = IO_TAG(P35), .usageFlags = TIM_USE_MOTOR, .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[10], .tag = IO_TAG(P34), .usageFlags = TIM_USE_MOTOR, .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[11], .tag = IO_TAG(P33), .usageFlags = TIM_USE_MOTOR, .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[12], .tag = IO_TAG(P46), .usageFlags = TIM_USE_MOTOR, .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[13], .tag = IO_TAG(P45), .usageFlags = TIM_USE_MOTOR, .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[14], .tag = IO_TAG(P44), .usageFlags = 0,             .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
		{ .tim = &XMCTimer[15], .tag = IO_TAG(P43), .usageFlags = 0,             .channel=0, .alternateFunction = XMC_GPIO_MODE_OUTPUT_ALT3 },
};

