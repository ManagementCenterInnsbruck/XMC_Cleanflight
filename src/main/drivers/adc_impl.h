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

#include "drivers/io_types.h"
#include "rcc_types.h"

#if defined(STM32F4) || defined(STM32F7)
#define ADC_TAG_MAP_COUNT 16
#elif defined(STM32F3)
#define ADC_TAG_MAP_COUNT 39
#elif defined(XMC4500_F100x1024)
#define ADC_TAG_MAP_COUNT 32
#else
#define ADC_TAG_MAP_COUNT 10
#endif

#ifdef XMC4500_F100x1024
#include "dma.h"
typedef VADC_GLOBAL_TypeDef ADC_TypeDef;
#endif

typedef enum ADCDevice {
    ADCINVALID = -1,
    ADCDEV_1   = 0,
#if defined(STM32F3) || defined(STM32F4) || defined(STM32F7)
    ADCDEV_2,
    ADCDEV_3
#endif
} ADCDevice;

typedef struct adcTagMap_s {
    ioTag_t tag;
    uint8_t channel;
#ifdef XMC4500_F100x1024
    VADC_G_TypeDef* group;
#endif
} adcTagMap_t;

typedef struct adcDevice_s {
    ADC_TypeDef* ADCx;
#ifndef XMC4500_F100x1024
    rccPeriphTag_t rccADC;
#if defined(STM32F4) || defined(STM32F7)
    DMA_Stream_TypeDef* DMAy_Streamx;
    uint32_t channel;
#else
    DMA_Channel_TypeDef* DMAy_Channelx;
#endif
#if defined(STM32F7)
    ADC_HandleTypeDef ADCHandle;
    DMA_HandleTypeDef DmaHandle;
#endif
#endif
} adcDevice_t;

#ifdef USE_ONBOARD_ESC
typedef struct adcInverter_s
{
	VADC_G_TypeDef* group;
	uint8_t			channels[3];
	uint8_t			trigger_input;
	uint32_t		irq;
}adcInverter_t;
#endif

extern const adcDevice_t adcHardware[];
extern const adcTagMap_t adcTagMap[ADC_TAG_MAP_COUNT];
extern adcOperatingConfig_t adcOperatingConfig[ADC_CHANNEL_COUNT];
extern volatile uint16_t adcValues[ADC_CHANNEL_COUNT];

uint8_t adcChannelByTag(ioTag_t ioTag);
