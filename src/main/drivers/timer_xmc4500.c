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

#include "platform.h"

#include "common/utils.h"

#include "timer.h"

TIM_TypeDef XMCTimer[HARDWARE_TIMER_DEFINITION_COUNT] =
{
	{ .slice_ptr = (uint32_t*)CCU40_CC40, .slice_no = 0, .module = (uint32_t*)CCU40, .isCCU8 = 0 },
	{ .slice_ptr = (uint32_t*)CCU40_CC41, .slice_no = 1, .module = (uint32_t*)CCU40, .isCCU8 = 0 },
	{ .slice_ptr = (uint32_t*)CCU40_CC42, .slice_no = 2, .module = (uint32_t*)CCU40, .isCCU8 = 0 },
	{ .slice_ptr = (uint32_t*)CCU40_CC43, .slice_no = 3, .module = (uint32_t*)CCU40, .isCCU8 = 0 },
	{ .slice_ptr = (uint32_t*)CCU41_CC40, .slice_no = 0, .module = (uint32_t*)CCU41, .isCCU8 = 0 },
	{ .slice_ptr = (uint32_t*)CCU41_CC41, .slice_no = 1, .module = (uint32_t*)CCU41, .isCCU8 = 0 },
	{ .slice_ptr = (uint32_t*)CCU41_CC42, .slice_no = 2, .module = (uint32_t*)CCU41, .isCCU8 = 0 },
	{ .slice_ptr = (uint32_t*)CCU41_CC43, .slice_no = 3, .module = (uint32_t*)CCU41, .isCCU8 = 0 },
	{ .slice_ptr = (uint32_t*)CCU42_CC40, .slice_no = 0, .module = (uint32_t*)CCU42, .isCCU8 = 0 },
	{ .slice_ptr = (uint32_t*)CCU42_CC41, .slice_no = 1, .module = (uint32_t*)CCU42, .isCCU8 = 0 },
	{ .slice_ptr = (uint32_t*)CCU42_CC42, .slice_no = 2, .module = (uint32_t*)CCU42, .isCCU8 = 0 },
	{ .slice_ptr = (uint32_t*)CCU42_CC43, .slice_no = 3, .module = (uint32_t*)CCU42, .isCCU8 = 0 },
	{ .slice_ptr = (uint32_t*)CCU43_CC40, .slice_no = 0, .module = (uint32_t*)CCU43, .isCCU8 = 0 },
	{ .slice_ptr = (uint32_t*)CCU43_CC41, .slice_no = 1, .module = (uint32_t*)CCU43, .isCCU8 = 0 },
	{ .slice_ptr = (uint32_t*)CCU43_CC42, .slice_no = 2, .module = (uint32_t*)CCU43, .isCCU8 = 0 },
	{ .slice_ptr = (uint32_t*)CCU43_CC43, .slice_no = 3, .module = (uint32_t*)CCU43, .isCCU8 = 0 },
	{ .slice_ptr = (uint32_t*)CCU80_CC80, .slice_no = 0, .module = (uint32_t*)CCU80, .isCCU8 = 1 },
	{ .slice_ptr = (uint32_t*)CCU80_CC81, .slice_no = 1, .module = (uint32_t*)CCU80, .isCCU8 = 1 },
	{ .slice_ptr = (uint32_t*)CCU80_CC82, .slice_no = 2, .module = (uint32_t*)CCU80, .isCCU8 = 1 },
	{ .slice_ptr = (uint32_t*)CCU80_CC83, .slice_no = 3, .module = (uint32_t*)CCU80, .isCCU8 = 1 },
	{ .slice_ptr = (uint32_t*)CCU81_CC80, .slice_no = 0, .module = (uint32_t*)CCU81, .isCCU8 = 1 },
	{ .slice_ptr = (uint32_t*)CCU81_CC81, .slice_no = 1, .module = (uint32_t*)CCU81, .isCCU8 = 1 },
	{ .slice_ptr = (uint32_t*)CCU81_CC82, .slice_no = 2, .module = (uint32_t*)CCU81, .isCCU8 = 1 },
	{ .slice_ptr = (uint32_t*)CCU81_CC83, .slice_no = 3, .module = (uint32_t*)CCU81, .isCCU8 = 1 },
};

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
		{	.TIMx = &XMCTimer[0],  .inputIrq = 0	},
		{	.TIMx = &XMCTimer[1],  .inputIrq = 0	},
		{	.TIMx = &XMCTimer[2],  .inputIrq = 0	},
		{	.TIMx = &XMCTimer[3],  .inputIrq = 0	},
		{	.TIMx = &XMCTimer[4],  .inputIrq = 0	},
		{	.TIMx = &XMCTimer[5],  .inputIrq = 0	},
		{	.TIMx = &XMCTimer[6],  .inputIrq = 0	},
		{	.TIMx = &XMCTimer[7],  .inputIrq = 0	},
		{	.TIMx = &XMCTimer[8],  .inputIrq = 0	},
		{	.TIMx = &XMCTimer[9],  .inputIrq = 0	},
		{	.TIMx = &XMCTimer[10], .inputIrq = 0	},
		{	.TIMx = &XMCTimer[11], .inputIrq = 0	},
		{	.TIMx = &XMCTimer[12], .inputIrq = 0	},
		{	.TIMx = &XMCTimer[13], .inputIrq = 0	},
		{	.TIMx = &XMCTimer[14], .inputIrq = 0	},
		{	.TIMx = &XMCTimer[15], .inputIrq = 0	},
		{	.TIMx = &XMCTimer[16], .inputIrq = 0	},
		{	.TIMx = &XMCTimer[17], .inputIrq = 0	},
		{	.TIMx = &XMCTimer[18], .inputIrq = 0	},
		{	.TIMx = &XMCTimer[19], .inputIrq = 0	},
		{	.TIMx = &XMCTimer[20], .inputIrq = 0	},
		{	.TIMx = &XMCTimer[21], .inputIrq = 0	},
		{	.TIMx = &XMCTimer[22], .inputIrq = 0	},
		{	.TIMx = &XMCTimer[23], .inputIrq = 0	},
};

uint8_t timerClockDivisor(TIM_TypeDef *tim)
{
    UNUSED(tim);
    return 1;
}

uint32_t timerClock(TIM_TypeDef *tim)
{
    UNUSED(tim);
    return SystemCoreClock;
}
