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

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
	  {   .TIMx = (TIM_TypeDef*)CCU40_CC40,   .ccu_global = (uint32_t*)CCU40,	.isCCU8 = 0,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU40_CC41,   .ccu_global = (uint32_t*)CCU40,	.isCCU8 = 0,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU40_CC42,   .ccu_global = (uint32_t*)CCU40,	.isCCU8 = 0,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU40_CC43,   .ccu_global = (uint32_t*)CCU40,	.isCCU8 = 0,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU41_CC40,   .ccu_global = (uint32_t*)CCU41,	.isCCU8 = 0,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU41_CC41,   .ccu_global = (uint32_t*)CCU41,	.isCCU8 = 0,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU41_CC42,   .ccu_global = (uint32_t*)CCU41,	.isCCU8 = 0,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU41_CC43,   .ccu_global = (uint32_t*)CCU41,	.isCCU8 = 0,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU42_CC40,   .ccu_global = (uint32_t*)CCU42,	.isCCU8 = 0,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU42_CC41,   .ccu_global = (uint32_t*)CCU42,	.isCCU8 = 0,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU42_CC42,   .ccu_global = (uint32_t*)CCU42,	.isCCU8 = 0,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU42_CC43,   .ccu_global = (uint32_t*)CCU42,	.isCCU8 = 0,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU43_CC40,   .ccu_global = (uint32_t*)CCU43,	.isCCU8 = 0,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU43_CC41,   .ccu_global = (uint32_t*)CCU43,	.isCCU8 = 0,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU43_CC42,   .ccu_global = (uint32_t*)CCU43,	.isCCU8 = 0,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU43_CC43,   .ccu_global = (uint32_t*)CCU43,	.isCCU8 = 0,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU80_CC80,   .ccu_global = (uint32_t*)CCU80,	.isCCU8 = 1,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU80_CC81,   .ccu_global = (uint32_t*)CCU80,	.isCCU8 = 1,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU80_CC82,   .ccu_global = (uint32_t*)CCU80,	.isCCU8 = 1,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU80_CC83,   .ccu_global = (uint32_t*)CCU80,	.isCCU8 = 1,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU81_CC80,   .ccu_global = (uint32_t*)CCU81,	.isCCU8 = 1,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU81_CC81,   .ccu_global = (uint32_t*)CCU81,	.isCCU8 = 1,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU81_CC82,   .ccu_global = (uint32_t*)CCU81,	.isCCU8 = 1,   .inputIrq = 0},
	  {   .TIMx = (TIM_TypeDef*)CCU81_CC83,   .ccu_global = (uint32_t*)CCU81,	.isCCU8 = 1,   .inputIrq = 0},
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
