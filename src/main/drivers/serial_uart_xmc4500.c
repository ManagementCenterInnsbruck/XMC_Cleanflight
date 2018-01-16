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

/*
 * Authors:
 * jflyper - Refactoring, cleanup and made pin-configurable
 * Dominic Clifton - Port baseflight STM32F10x to STM32F30x for cleanflight
 * J. Ihlein - Code from FocusFlight32
 * Bill Nesbitt - Code from AutoQuad
 * Hamasaki/Timecop - Initial baseflight code
*/

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "drivers/system.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/dma.h"
#ifndef XMC4500_F100x1024
#include "drivers/rcc.h"
#endif

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

#ifdef USE_UART

// XXX Will DMA eventually be configurable?
// XXX Do these belong here?

#ifdef USE_UART1_RX_DMA
# define UART1_RX_DMA DMA1_Channel5
#else
# define UART1_RX_DMA 0
#endif

#ifdef USE_UART1_TX_DMA
# define UART1_TX_DMA DMA1_Channel4
#else
# define UART1_TX_DMA 0
#endif

#ifdef USE_UART2_RX_DMA
# define UART2_RX_DMA DMA1_Channel6
#else
# define UART2_RX_DMA 0
#endif

#ifdef USE_UART2_TX_DMA
# define UART2_TX_DMA DMA1_Channel7
#else
# define UART2_TX_DMA 0
#endif

#ifdef USE_UART3_RX_DMA
# define UART3_RX_DMA DMA1_Channel3
#else
# define UART3_RX_DMA 0
#endif

#ifdef USE_UART3_TX_DMA
# define UART3_TX_DMA DMA1_Channel2
#else
# define UART3_TX_DMA 0
#endif

const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART1
    {
        .device = UARTDEV_1,
        .rxDMAChannel = UART1_RX_DMA,
        .txDMAChannel = UART1_TX_DMA,
        .txPriority = NVIC_PRIO_SERIALUART1_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART1_RXDMA,
#if defined(UART1_USIC) && UART1_USIC == U1C1
        .reg = USIC1_CH1,
        .rxPins = { IO_TAG_NONE, IO_TAG_NONE, IO_TAG_NONE, DEFIO_TAG_E(P00) },
        .txPins = { DEFIO_TAG_E(P01) },
        .txAf = { XMC_GPIO_MODE_OUTPUT_ALT2 },
        .irqn_tx = USIC1_2_IRQn,
		.irqn_rx = USIC1_3_IRQn,
#else
        .reg = USIC0_CH0,
        .rxPins = { DEFIO_TAG_E(P15), DEFIO_TAG_E(P14), IO_TAG_NONE, DEFIO_TAG_E(P50) },//DXx channels on XMC depends on placement here
        .txPins = { DEFIO_TAG_E(P15), DEFIO_TAG_E(P17), DEFIO_TAG_E(P51) },
        .txAf = { XMC_GPIO_MODE_OUTPUT_ALT2, XMC_GPIO_MODE_OUTPUT_ALT2, XMC_GPIO_MODE_OUTPUT_ALT1, 0 },
        .irqn_tx = USIC0_0_IRQn,
		.irqn_rx = USIC0_1_IRQn,
#endif
    },
#endif

#ifdef USE_UART2
    {
        .device = UARTDEV_2,
        .reg = USIC1_CH0,
        .rxDMAChannel = UART2_RX_DMA,
        .txDMAChannel = UART2_TX_DMA,
		.rxPins = { DEFIO_TAG_E(P04), DEFIO_TAG_E(P05), DEFIO_TAG_E(P215), DEFIO_TAG_E(P214) },
        .txPins = { DEFIO_TAG_E(P05), DEFIO_TAG_E(P214), IO_TAG_NONE, IO_TAG_NONE },
        .txAf = { XMC_GPIO_MODE_OUTPUT_ALT2, XMC_GPIO_MODE_OUTPUT_ALT2, 0, 0 },
        .irqn_tx = USIC1_0_IRQn,
		.irqn_rx = USIC1_1_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART2_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART2_RXDMA,
    },
#endif

#ifdef USE_UART3
    {
        .device = UARTDEV_3,
        .rxDMAChannel = UART3_RX_DMA,
        .txDMAChannel = UART3_TX_DMA,
        .txPriority = NVIC_PRIO_SERIALUART3_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART3_RXDMA,
#if UART3_USIC == U0C0
		.reg = USIC0_CH0,
        .rxPins = { DEFIO_TAG_E(P15), DEFIO_TAG_E(P14), IO_TAG_NONE, DEFIO_TAG_E(P50) },
        .txPins = { DEFIO_TAG_E(P15), DEFIO_TAG_E(P17), DEFIO_TAG_E(P51), IO_TAG_NONE },
        .txAf = { XMC_GPIO_MODE_OUTPUT_ALT2, XMC_GPIO_MODE_OUTPUT_ALT2, XMC_GPIO_MODE_OUTPUT_ALT1, 0 },
        .irqn_tx = USIC0_0_IRQn,
		.irqn_rx = USIC0_1_IRQn,
#else
		.reg = USIC2_CH1,
        .rxPins = { IO_TAG_NONE, DEFIO_TAG_E(P34), DEFIO_TAG_E(P40), IO_TAG_NONE },
        .txPins = { DEFIO_TAG_E(P35), IO_TAG_NONE, IO_TAG_NONE, IO_TAG_NONE },
        .txAf = { XMC_GPIO_MODE_OUTPUT_ALT1, 0, 0, 0 },
        .irqn_tx = USIC2_2_IRQn,
		.irqn_rx = USIC2_3_IRQn,
#endif
    },
#endif

#ifdef USE_UART4
    // UART4 XXX Not tested (yet!?) Need 303RC, e.g. LUX for testing
    {
        .device = UARTDEV_4,
        .reg = UART4,
        .rxDMAChannel = 0, // XXX UART4_RX_DMA !?
        .txDMAChannel = 0, // XXX UART4_TX_DMA !?
        .rxPins = { DEFIO_TAG_E(PC11), IO_TAG_NONE, IO_TAG_NONE, IO_TAG_NONE },
        .txPins = { DEFIO_TAG_E(PC10), IO_TAG_NONE, IO_TAG_NONE, IO_TAG_NONE },
        .rcc = RCC_APB1(UART4),
        .af = GPIO_AF_5,
        .irqn = UART4_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART4_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART4_RXDMA,
    },
#endif

#ifdef USE_UART5
    // UART5 XXX Not tested (yet!?) Need 303RC; e.g. LUX for testing
    {
        .device = UARTDEV_5,
        .reg = UART5,
        .rxDMAChannel = 0,
        .txDMAChannel = 0,
        .rxPins = { DEFIO_TAG_E(PD2), IO_TAG_NONE, IO_TAG_NONE, IO_TAG_NONE },
        .txPins = { DEFIO_TAG_E(PC12), IO_TAG_NONE, IO_TAG_NONE, IO_TAG_NONE },
        .rcc = RCC_APB1(UART5),
        .af = GPIO_AF_5,
        .irqn = UART5_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART5,
        .rxPriority = NVIC_PRIO_SERIALUART5,
    },
#endif
};

static void handleUsartTxDma(dmaChannelDescriptor_t* descriptor)
{
//    uartPort_t *s = (uartPort_t*)(descriptor->userParam);
//    DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
//    DMA_Cmd(descriptor->ref, DISABLE);

//    if (s->port.txBufferHead != s->port.txBufferTail)
//        uartStartTxDMA(s);
//    else
//        s->txDMAEmpty = true;
}

void serialUARTInitIO(IO_t txIO, IO_t rxIO, portMode_t mode, portOptions_t options, uint8_t af, uint8_t index)
{
    if ((options & SERIAL_BIDIR) && txIO) {
        ioConfig_t ioCfg = 0x00;

        IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(index));
        IOConfigGPIOAF(txIO, ioCfg, af);

        if (!(options & SERIAL_INVERTED))
            IOLo(txIO);   // OpenDrain output should be inactive
    } else {
        ioConfig_t ioCfg = XMC_GPIO_MODE_OUTPUT_PUSH_PULL | 0x01;
        if ((mode & MODE_TX) && txIO) {
            IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(index));
            IOConfigGPIOAF(txIO, ioCfg, af);
        }
        ioCfg = XMC_GPIO_MODE_INPUT_TRISTATE | 0x01;
        if ((mode & MODE_RX) && rxIO) {
            IOInit(rxIO, OWNER_SERIAL_RX, RESOURCE_INDEX(index));
            IOConfigGPIO(rxIO, ioCfg);
        }
    }
}

// XXX Should serialUART be consolidated?

uartPort_t *serialUART(UARTDevice device, uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s;

    uartDevice_t *uartDev = uartDevmap[device];
    if (!uartDev) {
        return NULL;
    }

    s = &(uartDev->port);
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = uartDev->rxBuffer;
    s->port.txBuffer = uartDev->txBuffer;
    s->port.rxBufferSize = sizeof(uartDev->rxBuffer);
    s->port.txBufferSize = sizeof(uartDev->txBuffer);

    const uartHardware_t *hardware = uartDev->hardware;

    s->USARTx = hardware->reg;

    if (hardware->rxDMAChannel) {
	   dmaInit(dmaGetIdentifier(hardware->rxDMAChannel), OWNER_SERIAL_RX, RESOURCE_INDEX(device));
	   s->rxDMAChannel = hardware->rxDMAChannel;
	   //s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->RDR;
   }

   if (hardware->txDMAChannel) {
	   const dmaIdentifier_e identifier = dmaGetIdentifier(hardware->txDMAChannel);
	   dmaInit(identifier, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
	   dmaSetHandler(identifier, handleUsartTxDma, hardware->txPriority, (uint32_t)s);
	   s->txDMAChannel = hardware->txDMAChannel;
	   //s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->TDR;
   }

   serialUARTInitIO(IOGetByTag(uartDev->tx), IOGetByTag(uartDev->rx), mode, options, uartDev->port.af, device);

   if (!s->rxDMAChannel || !s->txDMAChannel) {
	   NVIC_InitTypeDef NVIC_InitStructure;

	   NVIC_InitStructure.NVIC_IRQChannel = hardware->irqn_rx;
	   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(hardware->rxPriority);
	   NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(hardware->rxPriority);
	   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	   NVIC_Init(&NVIC_InitStructure);

	   NVIC_InitStructure.NVIC_IRQChannel = hardware->irqn_tx;
	   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(hardware->txPriority);
	   NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(hardware->txPriority);
	   NVIC_Init(&NVIC_InitStructure);
   }

    return s;
}

void uartTxIrqHandler(uartPort_t *s)
{
	if (!s->txDMAChannel)
	{
		if (s->port.txBufferTail != s->port.txBufferHead)
		{
			while(XMC_USIC_CH_TXFIFO_IsFull((XMC_USIC_CH_t*)s->USARTx));
			XMC_UART_CH_Transmit((XMC_USIC_CH_t*)s->USARTx, s->port.txBuffer[s->port.txBufferTail++]);
			if (s->port.txBufferTail >= s->port.txBufferSize)
				s->port.txBufferTail = 0;
		}
	}
}

void uartRxIrqHandler(uartPort_t *s)
{
	if(!s->rxDMAChannel)
	{
		while (!XMC_USIC_CH_RXFIFO_IsEmpty((XMC_USIC_CH_t*)s->USARTx))
		{
			if (s->port.rxCallback)
				s->port.rxCallback(XMC_UART_CH_GetReceivedData((XMC_USIC_CH_t*)s->USARTx));
			else
			{
				s->port.rxBuffer[s->port.rxBufferHead++] = XMC_UART_CH_GetReceivedData((XMC_USIC_CH_t*)s->USARTx);
				if (s->port.rxBufferHead >= s->port.rxBufferSize)
				{
					s->port.rxBufferHead = 0;
				}
			}
		}
	}
}
#endif // USE_UART
