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

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "io_impl.h"
#include "rcc.h"

/* for F30x processors */
#ifndef XMC4500_F100x1024
#if defined(STM32F303xC)
#ifndef GPIO_AF_SPI1
#define GPIO_AF_SPI1    GPIO_AF_5
#endif
#ifndef GPIO_AF_SPI2
#define GPIO_AF_SPI2    GPIO_AF_5
#endif
#ifndef GPIO_AF_SPI3
#define GPIO_AF_SPI3    GPIO_AF_6
#endif
#endif

#ifndef SPI1_SCK_PIN
#define SPI1_NSS_PIN    PA4
#define SPI1_SCK_PIN    PA5
#define SPI1_MISO_PIN   PA6
#define SPI1_MOSI_PIN   PA7
#endif

#ifndef SPI2_SCK_PIN
#define SPI2_NSS_PIN    PB12
#define SPI2_SCK_PIN    PB13
#define SPI2_MISO_PIN   PB14
#define SPI2_MOSI_PIN   PB15
#endif

#ifndef SPI3_SCK_PIN
#define SPI3_NSS_PIN    PA15
#define SPI3_SCK_PIN    PB3
#define SPI3_MISO_PIN   PB4
#define SPI3_MOSI_PIN   PB5
#endif

#ifndef SPI1_NSS_PIN
#define SPI1_NSS_PIN NONE
#endif
#ifndef SPI2_NSS_PIN
#define SPI2_NSS_PIN NONE
#endif
#ifndef SPI3_NSS_PIN
#define SPI3_NSS_PIN NONE
#endif

#else

#ifndef GPIO_AF_SPI1_CLK
#define GPIO_AF_SPI1_CLK    XMC_GPIO_MODE_OUTPUT_ALT2
#endif
#ifndef GPIO_AF_SPI1_MOSI
#define GPIO_AF_SPI1_MOSI    XMC_GPIO_MODE_OUTPUT_ALT2
#endif
#ifndef GPIO_AF_SPI1_NSS
#define GPIO_AF_SPI1_NSS    XMC_GPIO_MODE_OUTPUT_ALT2
#endif


#ifndef SPI1_SCK_PIN
#define SPI1_NSS_PIN    P06
#define SPI1_SCK_PIN    P010
#define SPI1_MISO_PIN   P00
#define SPI1_MOSI_PIN   P01
#endif

#endif

static spiDevice_t spiHardwareMap[] = {
#ifndef XMC4500_F100x1024
#if defined(STM32F1)
    { .dev = SPI1, .nss = IO_TAG(SPI1_NSS_PIN), .sck = IO_TAG(SPI1_SCK_PIN), .miso = IO_TAG(SPI1_MISO_PIN), .mosi = IO_TAG(SPI1_MOSI_PIN), .rcc = RCC_APB2(SPI1), .af = 0, false },
    { .dev = SPI2, .nss = IO_TAG(SPI2_NSS_PIN), .sck = IO_TAG(SPI2_SCK_PIN), .miso = IO_TAG(SPI2_MISO_PIN), .mosi = IO_TAG(SPI2_MOSI_PIN), .rcc = RCC_APB1(SPI2), .af = 0, false },
#else
    { .dev = SPI1, .nss = IO_TAG(SPI1_NSS_PIN), .sck = IO_TAG(SPI1_SCK_PIN), .miso = IO_TAG(SPI1_MISO_PIN), .mosi = IO_TAG(SPI1_MOSI_PIN), .rcc = RCC_APB2(SPI1), .af = GPIO_AF_SPI1, false },
    { .dev = SPI2, .nss = IO_TAG(SPI2_NSS_PIN), .sck = IO_TAG(SPI2_SCK_PIN), .miso = IO_TAG(SPI2_MISO_PIN), .mosi = IO_TAG(SPI2_MOSI_PIN), .rcc = RCC_APB1(SPI2), .af = GPIO_AF_SPI2, false },
#endif
#if defined(STM32F3) || defined(STM32F4)
    { .dev = SPI3, .nss = IO_TAG(SPI3_NSS_PIN), .sck = IO_TAG(SPI3_SCK_PIN), .miso = IO_TAG(SPI3_MISO_PIN), .mosi = IO_TAG(SPI3_MOSI_PIN), .rcc = RCC_APB1(SPI3), .af = GPIO_AF_SPI3, false }
#endif
#else
    { .dev = USIC1_CH1, .nss = IO_TAG(SPI1_NSS_PIN), .sck = IO_TAG(SPI1_SCK_PIN), .miso = IO_TAG(SPI1_MISO_PIN), .mosi = IO_TAG(SPI1_MOSI_PIN), .af_clk = GPIO_AF_SPI1_CLK, .af_mosi = GPIO_AF_SPI1_MOSI, .af_nss = GPIO_AF_SPI1_NSS, .miso_source = 3, .en_nss = XMC_SPI_CH_SLAVE_SELECT_1, false },
#endif
};

SPIDevice spiDeviceByInstance(SPI_TypeDef *instance)
{
#ifndef XMC4500_F100x1024
    if (instance == SPI1)
        return SPIDEV_1;

    if (instance == SPI2)
        return SPIDEV_2;

    if (instance == SPI3)
        return SPIDEV_3;
#else
    if (instance == USIC1_CH1)
    	return SPIDEV_1;
#endif
    return SPIINVALID;

}

void spiInitDevice(SPIDevice device)
{
    spiDevice_t *spi = &(spiHardwareMap[device]);

#ifdef SDCARD_SPI_INSTANCE
    if (spi->dev == SDCARD_SPI_INSTANCE) {
        spi->leadingEdge = true;
    }
#endif
#ifdef RX_SPI_INSTANCE
    if (spi->dev == RX_SPI_INSTANCE) {
        spi->leadingEdge = true;
    }
#endif

#ifndef XMC4500_F100x1024
    // Enable SPI clock
    RCC_ClockCmd(spi->rcc, ENABLE);
    RCC_ResetCmd(spi->rcc, ENABLE);
#endif

    IOInit(IOGetByTag(spi->sck),  OWNER_SPI_SCK,  RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->miso), OWNER_SPI_MISO, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI_MOSI, RESOURCE_INDEX(device));

#if defined(STM32F3) || defined(STM32F4)
    IOConfigGPIOAF(IOGetByTag(spi->sck),  SPI_IO_AF_CFG, spi->af);
    IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_CFG, spi->af);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->af);

    if (spi->nss) {
        IOInit(IOGetByTag(spi->nss), OWNER_SPI_CS, RESOURCE_INDEX(device));
        IOConfigGPIOAF(IOGetByTag(spi->nss), SPI_IO_CS_CFG, spi->af);
    }
#endif
#if defined(STM32F10X)
    IOConfigGPIO(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG);
    IOConfigGPIO(IOGetByTag(spi->miso), SPI_IO_AF_MISO_CFG);
    IOConfigGPIO(IOGetByTag(spi->mosi), SPI_IO_AF_MOSI_CFG);

    if (spi->nss) {
        IOInit(IOGetByTag(spi->nss), OWNER_SPI_CS, RESOURCE_INDEX(device));
        IOConfigGPIO(IOGetByTag(spi->nss), SPI_IO_CS_CFG);
    }
#endif

#ifdef XMC4500_F100x1024
    IOConfigGPIOAF(IOGetByTag(spi->sck), XMC_GPIO_MODE_OUTPUT_PUSH_PULL, spi->af_clk);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), XMC_GPIO_MODE_OUTPUT_PUSH_PULL, spi->af_mosi);
    IOConfigGPIO(IOGetByTag(spi->miso), XMC_GPIO_MODE_INPUT_TRISTATE);

    if(spi->nss)
    {
    	IOInit(IOGetByTag(spi->nss), OWNER_SPI_CS, RESOURCE_INDEX(device));
    	IOConfigGPIOAF(IOGetByTag(spi->nss), XMC_GPIO_MODE_OUTPUT_PUSH_PULL, spi->af_nss);
    }
#endif

    // Init SPI hardware
#ifndef XMC4500_F100x1024
    SPI_I2S_DeInit(spi->dev);

    SPI_InitTypeDef spiInit;
    spiInit.SPI_Mode = SPI_Mode_Master;
    spiInit.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spiInit.SPI_DataSize = SPI_DataSize_8b;
    spiInit.SPI_NSS = SPI_NSS_Soft;
    spiInit.SPI_FirstBit = SPI_FirstBit_MSB;
    spiInit.SPI_CRCPolynomial = 7;
    spiInit.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;

    if (spi->leadingEdge) {
        spiInit.SPI_CPOL = SPI_CPOL_Low;
        spiInit.SPI_CPHA = SPI_CPHA_1Edge;
    } else {
        spiInit.SPI_CPOL = SPI_CPOL_High;
        spiInit.SPI_CPHA = SPI_CPHA_2Edge;
    }

#ifdef STM32F303xC
    // Configure for 8-bit reads.
    SPI_RxFIFOThresholdConfig(spi->dev, SPI_RxFIFOThreshold_QF);
#endif

    SPI_Init(spi->dev, &spiInit);
    SPI_Cmd(spi->dev, ENABLE);

    if (spi->nss) {
        // Drive NSS high to disable connected SPI device.
        IOHi(IOGetByTag(spi->nss));
    }
#else
    XMC_SPI_CH_CONFIG_t spi_config =
    {
    	.baudrate = 1000000,
		.bus_mode = XMC_SPI_CH_BUS_MODE_MASTER,
		.selo_inversion = XMC_SPI_CH_SLAVE_SEL_INV_TO_MSLS,
		.parity_mode = XMC_USIC_CH_PARITY_MODE_NONE
    };

    XMC_SPI_CH_Init((XMC_USIC_CH_t*)spi->dev, &spi_config);
    XMC_SPI_CH_DisableFEM((XMC_USIC_CH_t*)spi->dev);
    XMC_SPI_CH_SetBitOrderMsbFirst((XMC_USIC_CH_t*)spi->dev);
    XMC_SPI_CH_SetWordLength((XMC_USIC_CH_t*)spi->dev, 8);
    XMC_SPI_CH_SetFrameLength((XMC_USIC_CH_t*)spi->dev,64);

    XMC_SPI_CH_ConfigureShiftClockOutput((XMC_USIC_CH_t*)spi->dev, XMC_SPI_CH_BRG_SHIFT_CLOCK_PASSIVE_LEVEL_0_DELAY_DISABLED, XMC_SPI_CH_BRG_SHIFT_CLOCK_OUTPUT_SCLK);
    XMC_SPI_CH_SetSlaveSelectDelay((XMC_USIC_CH_t*)spi->dev, 2);

    XMC_SPI_CH_SetInputSource((XMC_USIC_CH_t*)spi->dev, XMC_SPI_CH_INPUT_DIN0, spi->miso_source);
    XMC_SPI_CH_Start((XMC_USIC_CH_t*)spi->dev);

    XMC_SPI_CH_EnableSlaveSelect((XMC_USIC_CH_t*)spi->dev, spi->en_nss);

    switch((uint32_t)spi->dev)
    {
		case (uint32_t)USIC0_CH0:
		case (uint32_t)USIC1_CH0:
		case (uint32_t)USIC2_CH0:
		    XMC_USIC_CH_TXFIFO_Configure((XMC_USIC_CH_t*)spi->dev, 0, XMC_USIC_CH_FIFO_SIZE_16WORDS, 1);
		    XMC_USIC_CH_RXFIFO_Configure((XMC_USIC_CH_t*)spi->dev, 16, XMC_USIC_CH_FIFO_SIZE_16WORDS, 0);
			break;
		case (uint32_t)USIC0_CH1:
		case (uint32_t)USIC1_CH1:
		case (uint32_t)USIC2_CH1:
		    XMC_USIC_CH_TXFIFO_Configure((XMC_USIC_CH_t*)spi->dev, 32, XMC_USIC_CH_FIFO_SIZE_16WORDS, 1);
		    XMC_USIC_CH_RXFIFO_Configure((XMC_USIC_CH_t*)spi->dev, 48, XMC_USIC_CH_FIFO_SIZE_16WORDS, 0);
			break;
    }
#endif
}

bool spiInit(SPIDevice device)
{
    switch (device)
    {
    case SPIINVALID:
        return false;
    case SPIDEV_1:
#ifdef USE_SPI_DEVICE_1
        spiInitDevice(device);
        return true;
#else
        break;
#endif
    case SPIDEV_2:
#ifdef USE_SPI_DEVICE_2
        spiInitDevice(device);
        return true;
#else
        break;
#endif
    case SPIDEV_3:
#if defined(USE_SPI_DEVICE_3) && (defined(STM32F303xC) || defined(STM32F4))
        spiInitDevice(device);
        return true;
#else
        break;
#endif
    case SPIDEV_4:
#if defined(USE_SPI_DEVICE_4)
        spiInitDevice(device);
        return true;
#else
        break;
#endif
    }
    return false;
}

uint32_t spiTimeoutUserCallback(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device == SPIINVALID)
        return -1;
    spiHardwareMap[device].errorCount++;
    return spiHardwareMap[device].errorCount;
}

// return uint8_t value or -1 when failure
uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t data)
{
    uint16_t spiTimeout = 1000;

#ifndef XMC4500_F100x1024

    while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET)
        if ((spiTimeout--) == 0)
            return spiTimeoutUserCallback(instance);

#ifdef STM32F303xC
    SPI_SendData8(instance, data);
#else
    SPI_I2S_SendData(instance, data);
#endif
    spiTimeout = 1000;
    while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET)
        if ((spiTimeout--) == 0)
            return spiTimeoutUserCallback(instance);

#ifdef STM32F303xC
    return ((uint8_t)SPI_ReceiveData8(instance));
#else
    return ((uint8_t)SPI_I2S_ReceiveData(instance));
#endif

#else

    XMC_USIC_CH_RXFIFO_Flush((XMC_USIC_CH_t*)instance);
    XMC_USIC_CH_TXFIFO_Flush((XMC_USIC_CH_t*)instance);
    XMC_SPI_CH_Transmit((XMC_USIC_CH_t*)instance, data, XMC_SPI_CH_MODE_STANDARD);
    XMC_SPI_CH_Receive((XMC_USIC_CH_t*)instance, XMC_SPI_CH_MODE_STANDARD);
    while (XMC_USIC_CH_RXFIFO_GetLevel((XMC_USIC_CH_t*)instance) < 2)
    {
    	if(spiTimeout-- == 0)
    		return spiTimeoutUserCallback(instance);
	}

    return XMC_SPI_CH_GetReceivedData((XMC_USIC_CH_t*)instance);
#endif
}

#ifndef XMC4500_F100x1024
/**
 * Return true if the bus is currently in the middle of a transmission.
 */
bool spiIsBusBusy(SPI_TypeDef *instance)
{
#ifdef STM32F303xC
    return SPI_GetTransmissionFIFOStatus(instance) != SPI_TransmissionFIFOStatus_Empty || SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_BSY) == SET;
#else
    return SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET || SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_BSY) == SET;
#endif

}
#endif

bool spiTransfer(SPI_TypeDef *instance, uint8_t *out, const uint8_t *in, int len)
{
    uint16_t spiTimeout = 1000;

#ifndef XMC4500_F100x1024

    uint8_t b;
    instance->DR;
    while (len--) {
        b = in ? *(in++) : 0xFF;
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET) {
            if ((spiTimeout--) == 0)
                return spiTimeoutUserCallback(instance);
        }
#ifdef STM32F303xC
        SPI_SendData8(instance, b);
#else
        SPI_I2S_SendData(instance, b);
#endif
        spiTimeout = 1000;
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET) {
            if ((spiTimeout--) == 0)
                return spiTimeoutUserCallback(instance);
        }
#ifdef STM32F303xC
        b = SPI_ReceiveData8(instance);
#else
        b = SPI_I2S_ReceiveData(instance);
#endif
        if (out)
            *(out++) = b;
    }

#else

    XMC_USIC_CH_RXFIFO_Flush((XMC_USIC_CH_t*)instance);
    XMC_USIC_CH_TXFIFO_Flush((XMC_USIC_CH_t*)instance);

    for (uint16_t i=0; i<len; i++)
    {
    	while (XMC_USIC_CH_TXFIFO_IsFull((XMC_USIC_CH_t*)instance))
    	{
			if(spiTimeout-- == 0)
				return spiTimeoutUserCallback(instance);
    	}

    	XMC_SPI_CH_Transmit((XMC_USIC_CH_t*)instance, in[i], XMC_SPI_CH_MODE_STANDARD);

    	while (XMC_USIC_CH_RXFIFO_IsEmpty((XMC_USIC_CH_t*)instance))
    	{
			if(spiTimeout-- == 0)
				return spiTimeoutUserCallback(instance);
    	}

    	out[i] = XMC_SPI_CH_GetReceivedData((XMC_USIC_CH_t*)instance);
    }

#endif

    return true;
}

void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor)
{
#ifndef XMC4500_F100x1024

#define BR_CLEAR_MASK 0xFFC7

    uint16_t tempRegister;

    SPI_Cmd(instance, DISABLE);

    tempRegister = instance->CR1;

    switch (divisor) {
    case 2:
        tempRegister &= BR_CLEAR_MASK;
        tempRegister |= SPI_BaudRatePrescaler_2;
        break;

    case 4:
        tempRegister &= BR_CLEAR_MASK;
        tempRegister |= SPI_BaudRatePrescaler_4;
        break;

    case 8:
        tempRegister &= BR_CLEAR_MASK;
        tempRegister |= SPI_BaudRatePrescaler_8;
        break;

    case 16:
        tempRegister &= BR_CLEAR_MASK;
        tempRegister |= SPI_BaudRatePrescaler_16;
        break;

    case 32:
        tempRegister &= BR_CLEAR_MASK;
        tempRegister |= SPI_BaudRatePrescaler_32;
        break;

    case 64:
        tempRegister &= BR_CLEAR_MASK;
        tempRegister |= SPI_BaudRatePrescaler_64;
        break;

    case 128:
        tempRegister &= BR_CLEAR_MASK;
        tempRegister |= SPI_BaudRatePrescaler_128;
        break;

    case 256:
        tempRegister &= BR_CLEAR_MASK;
        tempRegister |= SPI_BaudRatePrescaler_256;
        break;
    }

    instance->CR1 = tempRegister;

    SPI_Cmd(instance, ENABLE);

#else

    XMC_SPI_CH_Stop((XMC_USIC_CH_t*)instance);
    XMC_SPI_CH_SetBaudrate((XMC_USIC_CH_t*)instance, SystemCoreClock / divisor);
    XMC_SPI_CH_Start((XMC_USIC_CH_t*)instance);

#endif
}

uint16_t spiGetErrorCounter(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device == SPIINVALID)
        return 0;
    return spiHardwareMap[device].errorCount;
}

void spiResetErrorCounter(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device != SPIINVALID)
        spiHardwareMap[device].errorCount = 0;
}
