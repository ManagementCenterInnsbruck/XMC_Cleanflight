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

#if !defined(UNIT_TEST) && !defined(SITL) && !(USBD_DEBUG_LEVEL > 0)
#pragma GCC poison sprintf snprintf
#endif

#if defined(STM32F745xx) || defined(STM32F746xx) || defined(STM32F722xx)
#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

// Chip Unique ID on F7
#if defined(STM32F722xx)
#define U_ID_0 (*(uint32_t*)0x1ff07a10)
#define U_ID_1 (*(uint32_t*)0x1ff07a14)
#define U_ID_2 (*(uint32_t*)0x1ff07a18)
#else
#define U_ID_0 (*(uint32_t*)0x1ff0f420)
#define U_ID_1 (*(uint32_t*)0x1ff0f424)
#define U_ID_2 (*(uint32_t*)0x1ff0f428)
#endif

#define STM32F7

#elif defined(STM32F40_41xxx) || defined (STM32F411xE) || defined (STM32F446xx)

#include "stm32f4xx_conf.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "core_cm4.h"

// Chip Unique ID on F405
#define U_ID_0 (*(uint32_t*)0x1fff7a10)
#define U_ID_1 (*(uint32_t*)0x1fff7a14)
#define U_ID_2 (*(uint32_t*)0x1fff7a18)

#define STM32F4

#elif defined(STM32F303xC)
#include <xmc4500_conf.h>
#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "core_cm4.h"

// Chip Unique ID on F303
#define U_ID_0 (*(uint32_t*)0x1FFFF7AC)
#define U_ID_1 (*(uint32_t*)0x1FFFF7B0)
#define U_ID_2 (*(uint32_t*)0x1FFFF7B4)

#define STM32F3

#elif defined(STM32F10X)

#include "stm32f10x_conf.h"
#include "stm32f10x_gpio.h"
#include "core_cm3.h"

// Chip Unique ID on F103
#define U_ID_0 (*(uint32_t*)0x1FFFF7E8)
#define U_ID_1 (*(uint32_t*)0x1FFFF7EC)
#define U_ID_2 (*(uint32_t*)0x1FFFF7F0)

#define STM32F1

#elif defined(SITL)

// Nop

#elif defined(XMC4500_F100x1024)

#define U_ID_0 (*(uint32_t*)0x50004000)
#define U_ID_1 (*(uint32_t*)0x50004004)
#define U_ID_2 (*(uint32_t*)0x50004008)

#include "XMC4500.h"
#include "xmc4500_conf.h"

#else // STM32F10X
#error "Invalid chipset specified. Update platform.h"
#endif

#ifdef USE_OSD_SLAVE
#include "target/common_osd_slave.h"
#include "target.h"
#else
#include "target/common_fc_pre.h"
#include "target.h"
#include "target/common_fc_post.h"
#endif
