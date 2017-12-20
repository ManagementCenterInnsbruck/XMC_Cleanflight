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
		//DEF_TIM(tim, chan, global, ccu8, pin, flags, out)
	    DEF_TIM(CCU40_CC40, CH1, CCU40, 0, P13, TIM_USE_MOTOR, 0),
		DEF_TIM(CCU40_CC41, CH1, CCU40, 0, P12, TIM_USE_MOTOR, 0),
		DEF_TIM(CCU40_CC43, CH1, CCU40, 0, P10, TIM_USE_MOTOR, 0),
		DEF_TIM(CCU40_CC42, CH1, CCU40, 0, P11, TIM_USE_MOTOR, 0),
	    DEF_TIM(CCU41_CC40, CH1, CCU41, 0, P25, 0, 0),
		DEF_TIM(CCU41_CC41, CH1, CCU41, 0, P24, 0, 0),
		DEF_TIM(CCU41_CC42, CH1, CCU41, 0, P23, 0, 0),
		DEF_TIM(CCU41_CC43, CH1, CCU41, 0, P22, 0, 0),
};

