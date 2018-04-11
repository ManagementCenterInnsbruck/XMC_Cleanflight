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

#ifndef SPI1_SCK_PIN
#define SPI1_NSS_PIN    P31
#define SPI1_SCK_PIN    P30
#define SPI1_MISO_PIN   P25
#define SPI1_MOSI_PIN   P40
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
#ifdef USE_SPIS1
    { .dev = USIC0_CH1, .nss = IO_TAG(SPI1_NSS_PIN), .sck = IO_TAG(SPI1_SCK_PIN), .miso = IO_TAG(SPI1_MISO_PIN), .mosi = IO_TAG(SPI1_MOSI_PIN), .af_source_clk = 1, .af_source_mosi = 4, .af_source_nss = 1, .af_source_miso = XMC_GPIO_MODE_OUTPUT_ALT2, .en_nss = XMC_SPI_CH_SLAVE_SELECT_1, .isSlave = 1, .irqn_tx = USIC0_2_IRQn, .irqn_rx = USIC0_3_IRQn, .txPriority = NVIC_PRIO_SPI_TXDMA, .rxPriority = NVIC_PRIO_SPI_RXDMA},
#else
    { .dev = USIC1_CH1, .nss = IO_TAG(SPI1_NSS_PIN), .sck = IO_TAG(SPI1_SCK_PIN), .miso = IO_TAG(SPI1_MISO_PIN), .mosi = IO_TAG(SPI1_MOSI_PIN), .af_source_clk = XMC_GPIO_MODE_OUTPUT_ALT2, .af_source_mosi = XMC_GPIO_MODE_OUTPUT_ALT2, .af_source_nss = XMC_GPIO_MODE_OUTPUT_ALT2, .af_source_miso = 3, .en_nss = XMC_SPI_CH_SLAVE_SELECT_1, .isSlave = 0},
#endif
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
#ifdef USE_SPIS1
    if (instance == USIC0_CH1)
#else
    if (instance == USIC1_CH1)
#endif
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
    if (spi->isSlave)
    {
    	IOConfigGPIO(IOGetByTag(spi->sck), XMC_GPIO_MODE_INPUT_TRISTATE);
		IOConfigGPIO(IOGetByTag(spi->mosi), XMC_GPIO_MODE_INPUT_TRISTATE);
		IOConfigGPIOAF(IOGetByTag(spi->miso), XMC_GPIO_MODE_OUTPUT_PUSH_PULL, spi->af_source_miso);

		if(spi->nss)
		{
			IOInit(IOGetByTag(spi->nss), OWNER_SPI_CS, RESOURCE_INDEX(device));
			IOConfigGPIO(IOGetByTag(spi->nss), XMC_GPIO_MODE_INPUT_TRISTATE);
		}
    }
    else
    {
		IOConfigGPIOAF(IOGetByTag(spi->sck), XMC_GPIO_MODE_OUTPUT_PUSH_PULL, spi->af_source_clk);
		IOConfigGPIOAF(IOGetByTag(spi->mosi), XMC_GPIO_MODE_OUTPUT_PUSH_PULL, spi->af_source_mosi);
		IOConfigGPIO(IOGetByTag(spi->miso), XMC_GPIO_MODE_INPUT_TRISTATE);

		if(spi->nss)
		{
			IOInit(IOGetByTag(spi->nss), OWNER_SPI_CS, RESOURCE_INDEX(device));
			IOConfigGPIOAF(IOGetByTag(spi->nss), XMC_GPIO_MODE_OUTPUT_PUSH_PULL, spi->af_source_nss);
		}
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
    XMC_SPI_CH_CONFIG_t spi_config;

	if (spi->isSlave)
	{
		spi_config.bus_mode = XMC_SPI_CH_BUS_MODE_SLAVE;
	}
	else
	{
		spi_config.baudrate = 1000000;
		spi_config.bus_mode = XMC_SPI_CH_BUS_MODE_MASTER;
		spi_config.selo_inversion = XMC_SPI_CH_SLAVE_SEL_INV_TO_MSLS;
		spi_config.parity_mode = XMC_USIC_CH_PARITY_MODE_NONE;
	}
	spi_config.parity_mode = XMC_USIC_CH_PARITY_MODE_NONE;

	XMC_SPI_CH_Init((XMC_USIC_CH_t*)spi->dev, &spi_config);

	XMC_SPI_CH_SetBitOrderMsbFirst((XMC_USIC_CH_t*)spi->dev);
	XMC_SPI_CH_SetWordLength((XMC_USIC_CH_t*)spi->dev, 8);
	XMC_SPI_CH_SetFrameLength((XMC_USIC_CH_t*)spi->dev,64);

	if (spi->isSlave)
	{
		XMC_SPI_CH_SetInputSource((XMC_USIC_CH_t*)spi->dev, XMC_SPI_CH_INPUT_DIN0, spi->af_source_mosi);
		XMC_SPI_CH_SetInputSource((XMC_USIC_CH_t*)spi->dev, XMC_SPI_CH_INPUT_SLAVE_SCLKIN, spi->af_source_clk);
		XMC_SPI_CH_SetInputSource((XMC_USIC_CH_t*)spi->dev, XMC_SPI_CH_INPUT_SLAVE_SELIN, spi->af_source_nss);
		XMC_SPI_CH_EnableInputInversion((XMC_USIC_CH_t*)spi->dev, XMC_SPI_CH_INPUT_SLAVE_SELIN);
	}
	else
	{
		XMC_SPI_CH_DisableFEM((XMC_USIC_CH_t*)spi->dev);
		XMC_SPI_CH_ConfigureShiftClockOutput((XMC_USIC_CH_t*)spi->dev, XMC_SPI_CH_BRG_SHIFT_CLOCK_PASSIVE_LEVEL_0_DELAY_DISABLED, XMC_SPI_CH_BRG_SHIFT_CLOCK_OUTPUT_SCLK);
		XMC_SPI_CH_SetSlaveSelectDelay((XMC_USIC_CH_t*)spi->dev, 2);

		XMC_SPI_CH_SetInputSource((XMC_USIC_CH_t*)spi->dev, XMC_SPI_CH_INPUT_DIN0, spi->af_source_miso);
		XMC_SPI_CH_Start((XMC_USIC_CH_t*)spi->dev);
		XMC_SPI_CH_EnableSlaveSelect((XMC_USIC_CH_t*)spi->dev, spi->en_nss);
	}

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



#ifdef USE_SPIS
// *****************************************************************
// ---------------------- SPI SLAVE --------------------------------
//
// *****************************************************************

uint32_t spiSlaveTotalRxBytesWaiting(const serialPort_t *instance)
{
    const spiDevice_t *s = (const spiDevice_t*)instance;

    if (s->port.rxBufferHead >= s->port.rxBufferTail) {
        return s->port.rxBufferHead - s->port.rxBufferTail;
    } else {
        return s->port.rxBufferSize + s->port.rxBufferHead - s->port.rxBufferTail;
    }
}

uint32_t spiSlaveTotalTxBytesFree(const serialPort_t *instance)
{
    const spiDevice_t *s = (const spiDevice_t*)instance;

    uint32_t bytesUsed;

    if (s->port.txBufferHead >= s->port.txBufferTail) {
        bytesUsed = s->port.txBufferHead - s->port.txBufferTail;
    } else {
        bytesUsed = s->port.txBufferSize + s->port.txBufferHead - s->port.txBufferTail;
    }

    return (s->port.txBufferSize - 1) - bytesUsed;
}

bool isSpiSlaveTransmitBufferEmpty(const serialPort_t *instance)
{
    const spiDevice_t *s = (const spiDevice_t *)instance;

    return s->port.txBufferTail == s->port.txBufferHead;
}

uint8_t spiSlaveRead(serialPort_t *instance)
{
    uint8_t ch;
    spiDevice_t *s = (spiDevice_t *)instance;

	ch = s->port.rxBuffer[s->port.rxBufferTail];
	if (s->port.rxBufferTail + 1 >= s->port.rxBufferSize) {
		s->port.rxBufferTail = 0;
	} else {
		s->port.rxBufferTail++;
	}

    return ch;
}

void spiSlaveWriteBuf(serialPort_t *instance, const void *data, int count)
{
	spiDevice_t *spi = (spiDevice_t*)instance;

	for (int i=0; i<count; i++)
	{
		spi->port.txBuffer[spi->port.txBufferHead] = ((uint8_t*)data)[i];
		if (spi->port.txBufferHead + 1 >= spi->port.txBufferSize) {
			spi->port.txBufferHead = 0;
		} else {
			spi->port.txBufferHead++;
		}
	}
}

void spiSlaveEndWrite(serialPort_t *instance)
{
	spiDevice_t *spi = (spiDevice_t*)instance;

	switch (spi->irqn_tx)
	{
		case USIC0_0_IRQn:
		case USIC1_0_IRQn:
		case USIC2_0_IRQn:
			XMC_USIC_CH_TriggerServiceRequest((XMC_USIC_CH_t*)spi->dev, 0);
			break;
		case USIC0_1_IRQn:
		case USIC1_1_IRQn:
		case USIC2_1_IRQn:
			XMC_USIC_CH_TriggerServiceRequest((XMC_USIC_CH_t*)spi->dev, 1);
			break;
		case USIC0_2_IRQn:
		case USIC1_2_IRQn:
		case USIC2_2_IRQn:
			XMC_USIC_CH_TriggerServiceRequest((XMC_USIC_CH_t*)spi->dev, 2);
			break;
		case USIC0_3_IRQn:
		case USIC1_3_IRQn:
		case USIC2_3_IRQn:
			XMC_USIC_CH_TriggerServiceRequest((XMC_USIC_CH_t*)spi->dev, 3);
			break;
		case USIC0_4_IRQn:
		case USIC1_4_IRQn:
		case USIC2_4_IRQn:
			XMC_USIC_CH_TriggerServiceRequest((XMC_USIC_CH_t*)spi->dev, 4);
			break;
		case USIC0_5_IRQn:
		case USIC1_5_IRQn:
		case USIC2_5_IRQn:
			XMC_USIC_CH_TriggerServiceRequest((XMC_USIC_CH_t*)spi->dev, 5);
			break;
	}
}

void spiSlaveWrite(serialPort_t *instance, uint8_t ch)
{
	spiDevice_t *spi = (spiDevice_t*)instance;
	spi->port.txBuffer[spi->port.txBufferHead] = ch;
    if (spi->port.txBufferHead + 1 >= spi->port.txBufferSize) {
    	spi->port.txBufferHead = 0;
    } else {
    	spi->port.txBufferHead++;
    }

	switch (spi->irqn_tx)
	{
		case USIC0_0_IRQn:
		case USIC1_0_IRQn:
		case USIC2_0_IRQn:
			XMC_USIC_CH_TriggerServiceRequest((XMC_USIC_CH_t*)spi->dev, 0);
			break;
		case USIC0_1_IRQn:
		case USIC1_1_IRQn:
		case USIC2_1_IRQn:
			XMC_USIC_CH_TriggerServiceRequest((XMC_USIC_CH_t*)spi->dev, 1);
			break;
		case USIC0_2_IRQn:
		case USIC1_2_IRQn:
		case USIC2_2_IRQn:
			XMC_USIC_CH_TriggerServiceRequest((XMC_USIC_CH_t*)spi->dev, 2);
			break;
		case USIC0_3_IRQn:
		case USIC1_3_IRQn:
		case USIC2_3_IRQn:
			XMC_USIC_CH_TriggerServiceRequest((XMC_USIC_CH_t*)spi->dev, 3);
			break;
		case USIC0_4_IRQn:
		case USIC1_4_IRQn:
		case USIC2_4_IRQn:
			XMC_USIC_CH_TriggerServiceRequest((XMC_USIC_CH_t*)spi->dev, 4);
			break;
		case USIC0_5_IRQn:
		case USIC1_5_IRQn:
		case USIC2_5_IRQn:
			XMC_USIC_CH_TriggerServiceRequest((XMC_USIC_CH_t*)spi->dev, 5);
			break;
	}
}

const struct serialPortVTable spisVTable[] = {
    {
        .serialWrite = spiSlaveWrite,
        .serialTotalRxWaiting = spiSlaveTotalRxBytesWaiting,
        .serialTotalTxFree = spiSlaveTotalTxBytesFree,
        .serialRead = spiSlaveRead,
        .serialSetBaudRate = NULL,
        .isSerialTransmitBufferEmpty = isSpiSlaveTransmitBufferEmpty,
        .setMode = NULL,
        .writeBuf = spiSlaveWriteBuf,
        .beginWrite = NULL,
        .endWrite = spiSlaveEndWrite,
    }
};

serialPort_t *spisOpen(SPIDevice device, serialReceiveCallbackPtr rxCallback, uint32_t baudRate, portMode_t mode, portOptions_t options)
{
	serialPort_t *s;

	spiDevice_t *spiDev = &(spiHardwareMap[device]);

	if (!spiDev)
		return NULL;

	s = &(spiDev->port);

	s->vTable = spisVTable;

	s->rxCallback = rxCallback;
	s->baudRate = baudRate;
	s->mode = mode;
	s->options = options;

	s->rxBuffer = spiDev->rxBuffer;
	s->txBuffer = spiDev->txBuffer;
	s->rxBufferSize = sizeof(spiDev->rxBuffer);
	s->txBufferSize = sizeof(spiDev->txBuffer);

	s->rxBufferHead = s->rxBufferTail = 0;
	s->txBufferHead = s->txBufferTail = 0;

	spiInitDevice(device);

	switch(spiDev->irqn_tx)
	{
		case USIC0_0_IRQn:
		case USIC1_0_IRQn:
		case USIC2_0_IRQn:
			XMC_USIC_CH_TXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)spiDev->dev, XMC_USIC_CH_TXFIFO_INTERRUPT_NODE_POINTER_STANDARD, 0);
			break;
		case USIC0_1_IRQn:
		case USIC1_1_IRQn:
		case USIC2_1_IRQn:
			XMC_USIC_CH_TXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)spiDev->dev, XMC_USIC_CH_TXFIFO_INTERRUPT_NODE_POINTER_STANDARD, 1);
			break;
		case USIC0_2_IRQn:
		case USIC1_2_IRQn:
		case USIC2_2_IRQn:
			XMC_USIC_CH_TXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)spiDev->dev, XMC_USIC_CH_TXFIFO_INTERRUPT_NODE_POINTER_STANDARD, 2);
			break;
		case USIC0_3_IRQn:
		case USIC1_3_IRQn:
		case USIC2_3_IRQn:
			XMC_USIC_CH_TXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)spiDev->dev, XMC_USIC_CH_TXFIFO_INTERRUPT_NODE_POINTER_STANDARD, 3);
			break;
		case USIC0_4_IRQn:
		case USIC1_4_IRQn:
		case USIC2_4_IRQn:
			XMC_USIC_CH_TXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)spiDev->dev, XMC_USIC_CH_TXFIFO_INTERRUPT_NODE_POINTER_STANDARD, 4);
			break;
		case USIC0_5_IRQn:
		case USIC1_5_IRQn:
		case USIC2_5_IRQn:
			XMC_USIC_CH_TXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)spiDev->dev, XMC_USIC_CH_TXFIFO_INTERRUPT_NODE_POINTER_STANDARD, 5);
			break;
	}

	switch(spiDev->irqn_rx)
	{
		case USIC0_0_IRQn:
		case USIC1_0_IRQn:
		case USIC2_0_IRQn:
			XMC_USIC_CH_RXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)spiDev->dev, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_STANDARD, 0);
			XMC_USIC_CH_RXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)spiDev->dev, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_ALTERNATE, 0);
			break;
		case USIC0_1_IRQn:
		case USIC1_1_IRQn:
		case USIC2_1_IRQn:
			XMC_USIC_CH_RXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)spiDev->dev, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_STANDARD, 1);
			XMC_USIC_CH_RXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)spiDev->dev, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_ALTERNATE, 1);
			break;
		case USIC0_2_IRQn:
		case USIC1_2_IRQn:
		case USIC2_2_IRQn:
			XMC_USIC_CH_RXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)spiDev->dev, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_STANDARD, 2);
			XMC_USIC_CH_RXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)spiDev->dev, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_ALTERNATE, 2);
			break;
		case USIC0_3_IRQn:
		case USIC1_3_IRQn:
		case USIC2_3_IRQn:
			XMC_USIC_CH_RXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)spiDev->dev, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_STANDARD, 3);
			XMC_USIC_CH_RXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)spiDev->dev, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_ALTERNATE, 3);
			break;
		case USIC0_4_IRQn:
		case USIC1_4_IRQn:
		case USIC2_4_IRQn:
			XMC_USIC_CH_RXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)spiDev->dev, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_STANDARD, 4);
			XMC_USIC_CH_RXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)spiDev->dev, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_ALTERNATE, 4);
			break;
		case USIC0_5_IRQn:
		case USIC1_5_IRQn:
		case USIC2_5_IRQn:
			XMC_USIC_CH_RXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)spiDev->dev, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_STANDARD, 5);
			XMC_USIC_CH_RXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)spiDev->dev, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_ALTERNATE, 5);
			break;
	}

	XMC_USIC_CH_TXFIFO_EnableEvent((XMC_USIC_CH_t*)spiDev->dev, XMC_USIC_CH_TXFIFO_EVENT_CONF_STANDARD);
	XMC_USIC_CH_RXFIFO_EnableEvent((XMC_USIC_CH_t*)spiDev->dev, XMC_USIC_CH_RXFIFO_EVENT_CONF_STANDARD | XMC_USIC_CH_RXFIFO_EVENT_CONF_ALTERNATE);

	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = spiDev->irqn_rx;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(spiDev->rxPriority);
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(spiDev->rxPriority);
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = spiDev->irqn_tx;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(spiDev->txPriority);
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(spiDev->txPriority);
	NVIC_Init(&NVIC_InitStructure);

	XMC_SPI_CH_Start((XMC_USIC_CH_t*)spiDev->dev);

	return s;
}

void spiSlaveTxIrqHandler(spiDevice_t *spi)
{
	while(spi->port.txBufferTail != spi->port.txBufferHead &&
		 !XMC_USIC_CH_TXFIFO_IsFull((XMC_USIC_CH_t*)spi->dev))
	{
		XMC_SPI_CH_Transmit((XMC_USIC_CH_t*)spi->dev, spi->port.txBuffer[spi->port.txBufferTail++], XMC_SPI_CH_MODE_STANDARD);
		if (spi->port.txBufferTail >= spi->port.txBufferSize)
			spi->port.txBufferTail = 0;
	}
}

void spiSlaveRxIrqHandler(spiDevice_t *spi)
{
	while (!XMC_USIC_CH_RXFIFO_IsEmpty((XMC_USIC_CH_t*)spi->dev))
	{
		spi->port.rxBuffer[spi->port.rxBufferHead++] = XMC_SPI_CH_GetReceivedData((XMC_USIC_CH_t*)spi->dev);
		if (spi->port.rxBufferHead >= spi->port.rxBufferSize)
		{
			spi->port.rxBufferHead = 0;
		}
	}
}

void USIC0_2_IRQHandler()
{
	spiDevice_t *spi = &(spiHardwareMap[SPIDEV_1]);
	spiSlaveTxIrqHandler(spi);
}

void USIC0_3_IRQHandler()
{
	spiDevice_t *spi = &(spiHardwareMap[SPIDEV_1]);
	spiSlaveRxIrqHandler(spi);
}
#endif
