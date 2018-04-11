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

#include "platform.h"

#include "drivers/gpio.h"
#include "drivers/nvic.h"
#include "drivers/system.h"

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

void systemReset(void)
{
    // Generate system reset
	NVIC_SystemReset();
}

void systemResetToBootloader(void)
{
    // 1FFFF000 -> 20000200 -> SP
    // 1FFFF004 -> 1FFFF021 -> PC

//    *((uint32_t *)0x20009FFC) = 0xDEADBEEF; // 40KB SRAM STM32F30X
//
//    systemReset();
}


void enableGPIOPowerUsageAndNoiseReductions(void)
{
#ifndef XMC4500_F100x1024
    RCC_AHBPeriphClockCmd(
        RCC_AHBPeriph_GPIOA |
        RCC_AHBPeriph_GPIOB |
        RCC_AHBPeriph_GPIOC |
        RCC_AHBPeriph_GPIOD |
        RCC_AHBPeriph_GPIOE |
        RCC_AHBPeriph_GPIOF,
        ENABLE
    );

    gpio_config_t gpio;

    gpio.mode = Mode_AIN;

    gpio.pin = Pin_All & ~(Pin_13 | Pin_14 | Pin_15);  // Leave JTAG pins alone
    gpioInit(GPIOA, &gpio);

    gpio.pin = Pin_All;
    gpioInit(GPIOB, &gpio);
    gpioInit(GPIOC, &gpio);
    gpioInit(GPIOD, &gpio);
    gpioInit(GPIOE, &gpio);
    gpioInit(GPIOF, &gpio);
#endif
}

//bool isMPUSoftReset(void)
//{
//    if (cachedRccCsrValue & RCC_CSR_SFTRSTF)
//        return true;
//    else
//        return false;
//}

void SystemCoreClockSetup(void)
{
	  /* Local data structure for initializing the clock functional block */
	  const XMC_SCU_CLOCK_CONFIG_t CLOCK_XMC4_0_CONFIG =
	  {
	    /* N-Divider Value */
	    .syspll_config.n_div = 80U,
	    /* P-Divider Value */
	    .syspll_config.p_div = 2U,
	    /* K2-Divider Value */
	    .syspll_config.k_div = 4U,
	    /* PLL Operating Mode */
	    .syspll_config.mode = XMC_SCU_CLOCK_SYSPLL_MODE_NORMAL,
	    /* PLL Clock Source */
	    .syspll_config.clksrc = XMC_SCU_CLOCK_SYSPLLCLKSRC_OSCHP,
	    /* High Precision Oscillator Operating Mode */
	    .enable_oschp = true,
	    /* Ultra Low Power Oscillator Setting */
	    .enable_osculp = false,
	    /* Calibration Mode */
	    .calibration_mode = XMC_SCU_CLOCK_FOFI_CALIBRATION_MODE_FACTORY,
	    /* Standby Clock Source */
	    .fstdby_clksrc = XMC_SCU_HIB_STDBYCLKSRC_OSI,
	    /* System Clock Source */
	    .fsys_clksrc = XMC_SCU_CLOCK_SYSCLKSRC_PLL,
	    /* System Clock Divider Value */
	    .fsys_clkdiv = 1U,
	    /* CPU Clock Divider Value */
	    .fcpu_clkdiv = 1U,
	    /* CCU Clock Divider Value */
	    .fccu_clkdiv = 1U,
	    /* Peripheral Clock Divider Value */
	    .fperipheral_clkdiv = 1U
	  };
	  /* Initialize the SCU clock */
	  XMC_SCU_CLOCK_Init(&CLOCK_XMC4_0_CONFIG);
	  /* RTC source clock */
	  XMC_SCU_HIB_SetRtcClockSource(XMC_SCU_HIB_RTCCLKSRC_OSI);
	  /* USB/SDMMC source clock */
	  XMC_SCU_CLOCK_SetUsbClockSource(XMC_SCU_CLOCK_USBCLKSRC_USBPLL);
	  /* USB/SDMMC divider setting */
	  XMC_SCU_CLOCK_SetUsbClockDivider(4U);
	  /* Start USB PLL */
	  XMC_SCU_CLOCK_StartUsbPll(1U, 32U);
	  /* WDT source clock */
	  XMC_SCU_CLOCK_SetWdtClockSource(XMC_SCU_CLOCK_WDTCLKSRC_OFI);
	  /* WDT divider setting */
	  XMC_SCU_CLOCK_SetWdtClockDivider(1U);
	  /* EBU divider setting */
	  XMC_SCU_CLOCK_SetEbuClockDivider(1U);
}


void systemInit(void)
{
    checkForBootLoaderRequest();

//    // Enable FPU
//    SCB->CPACR = (0x3 << (10 * 2)) | (0x3 << (11 * 2));
//    SetSysClock();
//
    // Configure NVIC preempt/priority groups
    NVIC_PriorityGroupConfig(NVIC_PRIORITY_GROUPING);
//
//    // cache RCC->CSR value to use it in isMPUSoftreset() and others
//    cachedRccCsrValue = RCC->CSR;
//    RCC_ClearFlag();

    enableGPIOPowerUsageAndNoiseReductions();

    // Init cycle counter
    cycleCounterInit();

    // SysTick
    SysTick_Config(SystemCoreClock / 1000);
}

void checkForBootLoaderRequest(void)
{
//    void(*bootJump)(void);
//
//    if (*((uint32_t *)0x20009FFC) == 0xDEADBEEF) {
//
//        *((uint32_t *)0x20009FFC) = 0x0;
//
//        __enable_irq();
//        __set_MSP(*((uint32_t *)0x1FFFD800));
//
//        bootJump = (void(*)(void))(*((uint32_t *) 0x1FFFD804));
//        bootJump();
//        while (1);
//    }
}
