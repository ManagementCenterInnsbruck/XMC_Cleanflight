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
		DEF_TIM(CCU40_CC41, CH2, CCU40, 0, P12, TIM_USE_MOTOR, 0),
		DEF_TIM(CCU40_CC42, CH3, CCU40, 0, P11, TIM_USE_MOTOR, 0),
		DEF_TIM(CCU40_CC43, CH4, CCU40, 0, P10, TIM_USE_MOTOR, 0),
	    DEF_TIM(CCU41_CC40, CH1, CCU41, 0, P25, 0, 0),
		DEF_TIM(CCU41_CC41, CH2, CCU41, 0, P24, 0, 0),
		DEF_TIM(CCU41_CC42, CH3, CCU41, 0, P23, 0, 0),
		DEF_TIM(CCU41_CC43, CH4, CCU41, 0, P22, 0, 0),
//	DEF_TIM(0,CH1,0,P00,0,0),
//    DEF_TIM(CCU40_CC41, (uint32_t)CCU40, P014, 3),
//    DEF_TIM(CCU40_CC42, (uint32_t)CCU40, P013, 3),
//    DEF_TIM(CCU40_CC43, (uint32_t)CCU40, P012, 3),
//    DEF_TIM(CCU41_CC40, (uint32_t)CCU41, P25,  3),
//    DEF_TIM(CCU41_CC41, (uint32_t)CCU41, P24,  3),
//    DEF_TIM(CCU41_CC42, (uint32_t)CCU41, P23,  3),
//    DEF_TIM(CCU41_CC43, (uint32_t)CCU41, P22,  3),
//    DEF_TIM(CCU42_CC40, (uint32_t)CCU42, P36,  3),
//    DEF_TIM(CCU42_CC41, (uint32_t)CCU42, P35,  3),
//    DEF_TIM(CCU42_CC42, (uint32_t)CCU42, P34,  3),
//    DEF_TIM(CCU42_CC43, (uint32_t)CCU42, P33,  3),
//    DEF_TIM(CCU43_CC40, (uint32_t)CCU43, P46,  3),
//    DEF_TIM(CCU43_CC41, (uint32_t)CCU43, P45,  3),
//	DEF_TIM(CCU43_CC42, (uint32_t)CCU43, P44,  3),
//	DEF_TIM(CCU43_CC43, (uint32_t)CCU43, P43,  3),
};

