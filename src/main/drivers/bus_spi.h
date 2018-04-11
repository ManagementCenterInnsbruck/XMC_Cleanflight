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
#include "nvic.h"

#if defined(USE_SPIS1)
#ifndef USE_SPIS
#define USE_SPIS
#endif
#endif

#if defined(STM32F4) || defined(STM32F3)
#define SPI_IO_AF_CFG      IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define SPI_IO_AF_SCK_CFG  IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_DOWN)
#define SPI_IO_AF_MISO_CFG IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP)
#define SPI_IO_CS_CFG      IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#elif defined(STM32F7)
#define SPI_IO_AF_CFG           IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)
#define SPI_IO_AF_SCK_CFG_HIGH  IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#define SPI_IO_AF_SCK_CFG_LOW   IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN)
#define SPI_IO_AF_MISO_CFG      IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)
#elif defined(STM32F1)
#define SPI_IO_AF_SCK_CFG     IO_CONFIG(GPIO_Mode_AF_PP,       GPIO_Speed_50MHz)
#define SPI_IO_AF_MOSI_CFG    IO_CONFIG(GPIO_Mode_AF_PP,       GPIO_Speed_50MHz)
#define SPI_IO_AF_MISO_CFG    IO_CONFIG(GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz)
#define SPI_IO_CS_CFG         IO_CONFIG(GPIO_Mode_Out_PP,      GPIO_Speed_50MHz)
#elif defined(XMC4500_F100x1024)
typedef USIC_CH_TypeDef SPI_TypeDef;
#endif

#ifdef USE_SPIS
#include "drivers/serial.h"

#define SPIS_BUFFER_SIZE 512
#endif

/*
  Flash M25p16 tolerates 20mhz, SPI_CLOCK_FAST should sit around 20 or less.
*/
typedef enum {
    SPI_CLOCK_INITIALIZATON = 256,
#if defined(STM32F4)
    SPI_CLOCK_SLOW          = 128, //00.65625 MHz
    SPI_CLOCK_STANDARD      = 8,   //10.50000 MHz
    SPI_CLOCK_FAST          = 4,   //21.00000 MHz
    SPI_CLOCK_ULTRAFAST     = 2    //42.00000 MHz
#elif defined(STM32F7)
    SPI_CLOCK_SLOW          = 256, //00.42188 MHz
    SPI_CLOCK_STANDARD      = 16,  //06.57500 MHz
    SPI_CLOCK_FAST          = 4,   //27.00000 MHz
    SPI_CLOCK_ULTRAFAST     = 2    //54.00000 MHz
#elif defined(XMC4500_F100x1024)
    SPI_CLOCK_SLOW          = 240, //00.50000 MHz
    SPI_CLOCK_STANDARD      = 20,  //06.00000 MHz
    SPI_CLOCK_FAST          = 4,   //30.00000 MHz
    SPI_CLOCK_ULTRAFAST     = 2    //60.00000 MHz
#else
    SPI_CLOCK_SLOW          = 128, //00.56250 MHz
    SPI_CLOCK_STANDARD      = 4,   //09.00000 MHz
    SPI_CLOCK_FAST          = 2,   //18.00000 MHz
    SPI_CLOCK_ULTRAFAST     = 2    //18.00000 MHz
#endif
} SPIClockDivider_e;

typedef enum SPIDevice {
    SPIINVALID = -1,
    SPIDEV_1   = 0,
    SPIDEV_2,
    SPIDEV_3,
    SPIDEV_4
} SPIDevice;

typedef struct SPIDevice_s {
#ifdef USE_SPIS
    serialPort_t port;
#endif
    SPI_TypeDef *dev;
    ioTag_t nss;
    ioTag_t sck;
    ioTag_t mosi;
    ioTag_t miso;
#ifndef XMC4500_F100x1024
    rccPeriphTag_t rcc;
    uint8_t af;
#else
    uint32_t af_source_clk;
    uint32_t af_source_mosi;
    uint32_t af_source_nss;
    uint32_t af_source_miso;
    XMC_SPI_CH_SLAVE_SELECT_t en_nss;

    uint8_t isSlave;

#ifdef USE_SPIS
    uint8_t irqn_rx;
    uint8_t irqn_tx;

    uint8_t txPriority;
    uint8_t rxPriority;

    volatile uint8_t rxBuffer[SPIS_BUFFER_SIZE];
    volatile uint8_t txBuffer[SPIS_BUFFER_SIZE];
#endif
#endif
    volatile uint16_t errorCount;
    bool leadingEdge;
#if defined(STM32F7)
    SPI_HandleTypeDef hspi;
    DMA_HandleTypeDef hdma;
    uint8_t dmaIrqHandler;
#endif
} spiDevice_t;

bool spiInit(SPIDevice device);
void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor);
uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t in);
bool spiIsBusBusy(SPI_TypeDef *instance);

bool spiTransfer(SPI_TypeDef *instance, uint8_t *out, const uint8_t *in, int len);

uint16_t spiGetErrorCounter(SPI_TypeDef *instance);
void spiResetErrorCounter(SPI_TypeDef *instance);
SPIDevice spiDeviceByInstance(SPI_TypeDef *instance);

#if defined(USE_HAL_DRIVER)
SPI_HandleTypeDef* spiHandleByInstance(SPI_TypeDef *instance);
DMA_HandleTypeDef* spiSetDMATransmit(DMA_Stream_TypeDef *Stream, uint32_t Channel, SPI_TypeDef *Instance, uint8_t *pData, uint16_t Size);
#endif

#ifdef USE_SPIS
serialPort_t *spisOpen(SPIDevice device, serialReceiveCallbackPtr rxCallback, uint32_t baudRate, portMode_t mode, portOptions_t options);
#endif
