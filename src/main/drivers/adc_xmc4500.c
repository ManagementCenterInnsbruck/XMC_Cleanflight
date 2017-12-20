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
#include <string.h>

#include "platform.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/gpio.h"
#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/time.h"
#include "drivers/pwm_output.h"

#include "flight/mixer.h"

#include "fc/rc_controls.h"

#include "adc.h"
#include "adc_impl.h"
#include "rcc.h"
#include "dma.h"

#include "common/utils.h"

#ifndef ADC_INSTANCE
#define ADC_INSTANCE                VADC
#endif

const adcDevice_t adcHardware[] = {
    { .ADCx = VADC },
};

#ifdef USE_ONBOARD_ESC
const adcInverter_t inverterHardware[] = {
	{ .group = VADC_G0, .channels = {3, 2, 1}, .trigger_input = XMC_VADC_REQ_TR_J, .irq = VADC0_G0_0_IRQn},
	{ .group = VADC_G1, .channels = {4, 5, 6}, .trigger_input = XMC_VADC_REQ_TR_L, .irq = VADC0_G1_0_IRQn},
	{ .group = VADC_G2, .channels = {3, 4, 5}, .trigger_input = XMC_VADC_REQ_TR_B, .irq = VADC0_G2_0_IRQn},
	{ .group = VADC_G3, .channels = {4, 5, 6}, .trigger_input = XMC_VADC_REQ_TR_E, .irq = VADC0_G3_0_IRQn},
};
#endif

const adcTagMap_t adcTagMap[] = {
    { DEFIO_TAG_E__P140,   0  , VADC_G0}, // G0
    { DEFIO_TAG_E__P141,   1  , VADC_G0}, // G0
    { DEFIO_TAG_E__P142,   2  , VADC_G0}, // G0
    { DEFIO_TAG_E__P143,   3  , VADC_G0}, // G0
    { DEFIO_TAG_E__P144,   4  , VADC_G0}, // G0
    { DEFIO_TAG_E__P145,   5  , VADC_G0}, // G0
    { DEFIO_TAG_E__P146,   6  , VADC_G0}, // G0
    { DEFIO_TAG_E__P147,   7  , VADC_G0}, // G0
    { DEFIO_TAG_E__P148,   0  , VADC_G1}, // G1
    { DEFIO_TAG_E__P149,   1  , VADC_G1}, // G1
	{ DEFIO_TAG_E__P142,   2  , VADC_G1}, // G1
	{ DEFIO_TAG_E__P143,   3  , VADC_G1}, // G1
    { DEFIO_TAG_E__P1412,  4  , VADC_G1}, // G1
    { DEFIO_TAG_E__P1413,  5  , VADC_G1}, // G1
    { DEFIO_TAG_E__P1414,  6  , VADC_G1}, // G1
    { DEFIO_TAG_E__P1415,  7  , VADC_G1}, // G1
	{ DEFIO_TAG_E__P144,   0  , VADC_G2}, // G2
	{ DEFIO_TAG_E__P145,   1  , VADC_G2}, // G2
    { DEFIO_TAG_E__P152,   2  , VADC_G2}, // G2
    { DEFIO_TAG_E__P153,   3  , VADC_G2}, // G2
    { DEFIO_TAG_E__P154,   4  , VADC_G2}, // G2
    { DEFIO_TAG_E__P155,   5  , VADC_G2}, // G2
    { DEFIO_TAG_E__P156,   6  , VADC_G2}, // G2
    { DEFIO_TAG_E__P157,   7  , VADC_G2}, // G2
    { DEFIO_TAG_E__P158,   0  , VADC_G3}, // G3
    { DEFIO_TAG_E__P159,   1  , VADC_G3}, // G3
	{ DEFIO_TAG_E__P148,   2  , VADC_G3}, // G3
	{ DEFIO_TAG_E__P149,   3  , VADC_G3}, // G3
    { DEFIO_TAG_E__P1512,  4  , VADC_G3}, // G3
    { DEFIO_TAG_E__P1513,  5  , VADC_G3}, // G3
    { DEFIO_TAG_E__P1514,  6  , VADC_G3}, // G3
    { DEFIO_TAG_E__P1515,  7  , VADC_G3}, // G3
};

pwmOutputPort_t* motors;

ADCDevice adcDeviceByInstance(ADC_TypeDef *instance)
{
    if (instance == VADC)
        return ADCDEV_1;

    return ADCINVALID;
}

VADC_G_TypeDef* adcGroupByTag(ioTag_t ioTag)
{
    for (uint8_t i = 0; i < ARRAYLEN(adcTagMap); i++) {
        if (ioTag == adcTagMap[i].tag)
            return adcTagMap[i].group;
    }
    return 0;
}

void adcInit(const adcConfig_t *config)
{
    memset(&adcOperatingConfig, 0, sizeof(adcOperatingConfig));

    if (config->vbat.enabled) {
        adcOperatingConfig[ADC_BATTERY].tag = config->vbat.ioTag;
    }

    if (config->rssi.enabled) {
        adcOperatingConfig[ADC_RSSI].tag = config->rssi.ioTag;  //RSSI_ADC_CHANNEL;
    }

    if (config->external1.enabled) {
        adcOperatingConfig[ADC_EXTERNAL1].tag = config->external1.ioTag; //EXTERNAL1_ADC_CHANNEL;
    }

    if (config->current.enabled) {
        adcOperatingConfig[ADC_CURRENT].tag = config->current.ioTag;  //CURRENT_METER_ADC_CHANNEL;
    }

    ADCDevice device = adcDeviceByInstance(ADC_INSTANCE);
    if (device == ADCINVALID)
        return;

    adcDevice_t adc = adcHardware[device];

    bool adcActive = false;
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (!adcOperatingConfig[i].tag)
            continue;

        adcActive = true;
        IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
        adcOperatingConfig[i].adcChannel = adcChannelByTag(adcOperatingConfig[i].tag);
        adcOperatingConfig[i].group = adcGroupByTag(adcOperatingConfig[i].tag);
        adcOperatingConfig[i].enabled = true;
    }

    if (!adcActive) {
        return;
    }

	XMC_VADC_GLOBAL_CONFIG_t global_config;
	memset(&global_config, 0, sizeof(global_config));
	XMC_VADC_GLOBAL_Init(adc.ADCx, &global_config);

	XMC_VADC_GROUP_CONFIG_t group_config;
	memset(&group_config, 0, sizeof(group_config));

	XMC_VADC_GROUP_Init(VADC_G0, &group_config);
	XMC_VADC_GROUP_Init(VADC_G1, &group_config);
	XMC_VADC_GROUP_Init(VADC_G2, &group_config);
	XMC_VADC_GROUP_Init(VADC_G3, &group_config);

	XMC_VADC_GROUP_SetPowerMode(VADC_G0, XMC_VADC_GROUP_POWERMODE_NORMAL);
	XMC_VADC_GROUP_SetPowerMode(VADC_G1, XMC_VADC_GROUP_POWERMODE_NORMAL);
	XMC_VADC_GROUP_SetPowerMode(VADC_G2, XMC_VADC_GROUP_POWERMODE_NORMAL);
	XMC_VADC_GROUP_SetPowerMode(VADC_G3, XMC_VADC_GROUP_POWERMODE_NORMAL);

	XMC_VADC_GLOBAL_StartupCalibration(adc.ADCx);

	XMC_VADC_GLOBAL_CLASS_t global_iclass_config =
	{
		.conversion_mode_standard = XMC_VADC_CONVMODE_12BIT,
		.sample_time_std_conv     = 0,
	};
	XMC_VADC_GLOBAL_InputClassInit(adc.ADCx, global_iclass_config, XMC_VADC_GROUP_CONV_STD, 0);

	const XMC_VADC_BACKGROUND_CONFIG_t backgnd_config =
	{
		.conv_start_mode   = (uint32_t) XMC_VADC_STARTMODE_CIR,
		.req_src_priority  = (uint32_t) XMC_VADC_GROUP_RS_PRIORITY_1,
		.trigger_signal    = (uint32_t) XMC_VADC_REQ_TR_A,
		.trigger_edge      = (uint32_t) XMC_VADC_TRIGGER_EDGE_NONE,
		.gate_signal       = (uint32_t) XMC_VADC_REQ_GT_A,
		.timer_mode        = (uint32_t) 0,
		.external_trigger  = (uint32_t) 0,
		.req_src_interrupt = (uint32_t) 0,
		.enable_auto_scan  = (uint32_t) 1,
		.load_mode         = (uint32_t) XMC_VADC_SCAN_LOAD_OVERWRITE
	};
	XMC_VADC_GLOBAL_BackgroundInit(adc.ADCx, &backgnd_config);

    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (!adcOperatingConfig[i].enabled) {
            continue;
        }

        XMC_VADC_CHANNEL_CONFIG_t ch_config =
        {
			.input_class                = XMC_VADC_CHANNEL_CONV_GLOBAL_CLASS0,
			.lower_boundary_select 	    = XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
			.upper_boundary_select 	    = XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
			.event_gen_criteria         = XMC_VADC_CHANNEL_EVGEN_NEVER,
			.sync_conversion  		    = 0,
			.alternate_reference        = XMC_VADC_CHANNEL_REF_INTREF,
			.result_reg_number          = adcOperatingConfig[i].adcChannel,
			.use_global_result          = 0,
			.result_alignment           = XMC_VADC_RESULT_ALIGN_RIGHT,
			.broken_wire_detect_channel = XMC_VADC_CHANNEL_BWDCH_VAGND,
			.broken_wire_detect		    = 0,
			.bfl                        = 0,
			.channel_priority           = 0,
			.alias_channel              = -1,
        };
        XMC_VADC_GROUP_ChannelInit(adcOperatingConfig[i].group, adcOperatingConfig[i].adcChannel, &ch_config);

        XMC_VADC_RESULT_CONFIG_t result_config =
        {
			.data_reduction_control  = 0,
			.post_processing_mode    = XMC_VADC_DMM_REDUCTION_MODE,
			.wait_for_read_mode  	 = 0,
			.part_of_fifo       	 = 0,
			.event_gen_enable   	 = 0
        };
        XMC_VADC_GROUP_ResultInit(adcOperatingConfig[i].group, adcOperatingConfig[i].adcChannel, &result_config);

        uint8_t group_index = 0;

        if (adcOperatingConfig[i].group == VADC_G0)
        	group_index = 0;
        else if (adcOperatingConfig[i].group == VADC_G1)
        	group_index = 1;
        else if (adcOperatingConfig[i].group == VADC_G2)
        	group_index = 2;
        else if (adcOperatingConfig[i].group == VADC_G3)
        	group_index = 3;

        XMC_VADC_GLOBAL_BackgroundAddChannelToSequence(VADC, group_index, adcOperatingConfig[i].adcChannel);
    }

    XMC_VADC_GLOBAL_BackgroundTriggerConversion(VADC);

#ifdef USE_ONBOARD_ESC

    motors = pwmGetMotors();

    for (uint8_t i=0; i<getMotorCount(); i++)
    {
    	motors[i].inverter.min_ccr = (motorConfig()->minthrottle - 1000) * motors[0].period / 1000;
    	motors[i].inverter.adc_group = inverterHardware[i].group;

        XMC_VADC_GROUP_CLASS_t group_iclass_config;
        memset(&group_iclass_config, 0, sizeof(group_iclass_config));
        XMC_VADC_GROUP_InputClassInit(inverterHardware[i].group, group_iclass_config, XMC_VADC_GROUP_CONV_STD, 0);

        XMC_VADC_SCAN_CONFIG_t scan_config =
        {
    		.conv_start_mode   	= XMC_VADC_STARTMODE_WFS,
    		.req_src_priority  	= XMC_VADC_GROUP_RS_PRIORITY_3,
    		.trigger_signal    	= inverterHardware[i].trigger_input,
    		.trigger_edge      	= XMC_VADC_TRIGGER_EDGE_RISING,
    		.gate_signal       	= XMC_VADC_REQ_GT_A,
    		.timer_mode		 	= 0,
    		.external_trigger	= 1,
    		.req_src_interrupt 	= 1,
    		.enable_auto_scan  	= 0,
    		.load_mode	     	= XMC_VADC_SCAN_LOAD_COMBINE,
        };
        XMC_VADC_GROUP_ScanInit(inverterHardware[i].group, &scan_config);

        for (uint8_t phase=0; phase<3; phase++)
		{
			motors[i].inverter.phase_channel[phase] = inverterHardware[i].channels[phase];
			XMC_VADC_GROUP_ScanAddChannelToSequence(inverterHardware[i].group, inverterHardware[i].channels[phase]);

			XMC_VADC_CHANNEL_CONFIG_t channel_config;
			memset(&channel_config, 0, sizeof(channel_config));
			channel_config.result_reg_number 	= inverterHardware[i].channels[phase];
			channel_config.result_alignment 	= XMC_VADC_RESULT_ALIGN_RIGHT;
			channel_config.channel_priority 	= true;
			channel_config.alias_channel 		= XMC_VADC_CHANNEL_ALIAS_DISABLED;

			XMC_VADC_GROUP_ChannelInit(inverterHardware[i].group, inverterHardware[i].channels[phase], &channel_config);

			XMC_VADC_RESULT_CONFIG_t result_config;
			memset(&result_config, 0, sizeof(result_config));
			XMC_VADC_GROUP_ResultInit(inverterHardware[i].group, inverterHardware[i].channels[phase], &result_config);
		}

    	XMC_VADC_GROUP_ScanSetReqSrcEventInterruptNode(inverterHardware[i].group, XMC_VADC_SR_GROUP_SR0);
    	NVIC_EnableIRQ(inverterHardware[i].irq);
    }

#endif
}

#ifdef USE_ONBOARD_ESC

void MotorCommutationCCU8(uint8_t motorIndex)
{
	switch (motors[motorIndex].inverter.pattern)
	{
		case 0:
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel1((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[0], motors[motorIndex].CCR_dummy);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel2((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[1], motors[motorIndex].CCR_dummy);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel1((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[2], 0);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel2((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[3], 0);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel1((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[4], 0);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel2((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[5], motors[motorIndex].period);
			break;
		case 1:
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel1((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[0], motors[motorIndex].CCR_dummy);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel2((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[1], motors[motorIndex].CCR_dummy);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel1((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[2], 0);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel2((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[3], motors[motorIndex].period);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel1((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[4], 0);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel2((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[5], 0);
			break;
		case 2:
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel1((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[0], 0);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel2((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[1], motors[motorIndex].period);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel1((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[2], motors[motorIndex].CCR_dummy);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel2((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[3], motors[motorIndex].CCR_dummy);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel1((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[4], 0);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel2((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[5], 0);
			break;
		case 3:
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel1((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[0], 0);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel2((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[1], 0);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel1((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[2], motors[motorIndex].CCR_dummy);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel2((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[3], motors[motorIndex].CCR_dummy);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel1((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[4], 0);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel2((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[5], motors[motorIndex].period);
			break;
		case 4:
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel1((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[0], 0);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel2((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[1], 0);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel1((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[2], 0);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel2((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[3], motors[motorIndex].period);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel1((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[4], motors[motorIndex].CCR_dummy);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel2((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[5], motors[motorIndex].CCR_dummy);
			break;
		case 5:
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel1((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[0], 0);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel2((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[1], motors[motorIndex].period);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel1((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[2], 0);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel2((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[3], 0);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel1((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[4], motors[motorIndex].CCR_dummy);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel2((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[5], motors[motorIndex].CCR_dummy);
			break;
		default:
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel1((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[0], 0);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel2((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[1], 0);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel1((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[2], 0);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel2((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[3], 0);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel1((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[4], 0);
			XMC_CCU8_SLICE_SetTimerCompareMatchChannel2((XMC_CCU8_SLICE_t*)motors[motorIndex].inverter.tim[5], 0);
			break;
	}

	XMC_CCU8_EnableShadowTransfer(CCU80, 0xFFFF);
	XMC_CCU8_EnableShadowTransfer(CCU81, 0xFFFF);
}

void MotorCommutationCCU4(uint8_t motorIndex)
{
	switch (motors[motorIndex].inverter.pattern)
	{
		case 0:
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[0], motors[motorIndex].CCR_dummy);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[1], motors[motorIndex].CCR_dummy + motors[motorIndex].inverter.deadtime);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[2], 0);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[3], 0);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[4], 0);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[5], motors[motorIndex].period);
			break;
		case 1:
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[0], motors[motorIndex].CCR_dummy);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[1], motors[motorIndex].CCR_dummy + motors[motorIndex].inverter.deadtime);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[2], 0);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[3], motors[motorIndex].period);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[4], 0);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[5], 0);
			break;
		case 2:
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[0], 0);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[1], motors[motorIndex].period);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[2], motors[motorIndex].CCR_dummy);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[3], motors[motorIndex].CCR_dummy + motors[motorIndex].inverter.deadtime);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[4], 0);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[5], 0);
			break;
		case 3:
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[0], 0);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[1], 0);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[2], motors[motorIndex].CCR_dummy);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[3], motors[motorIndex].CCR_dummy + motors[motorIndex].inverter.deadtime);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[4], 0);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[5], motors[motorIndex].period);
			break;
		case 4:
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[0], 0);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[1], 0);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[2], 0);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[3], motors[motorIndex].period);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[4], motors[motorIndex].CCR_dummy);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[5], motors[motorIndex].CCR_dummy + motors[motorIndex].inverter.deadtime);
			break;
		case 5:
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[0], 0);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[1], motors[motorIndex].period);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[2], 0);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[3], 0);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[4], motors[motorIndex].CCR_dummy);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[5], motors[motorIndex].CCR_dummy + motors[motorIndex].inverter.deadtime);
			break;
		default:
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[0], 0);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[1], 0);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[2], 0);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[3], 0);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[4], 0);
			XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)motors[motorIndex].inverter.tim[5], 0);
			break;
	}

	XMC_CCU4_EnableShadowTransfer(CCU40, 0xFFFF);
	XMC_CCU4_EnableShadowTransfer(CCU41, 0xFFFF);
	XMC_CCU4_EnableShadowTransfer(CCU42, 0xFFFF);
	XMC_CCU4_EnableShadowTransfer(CCU43, 0xFFFF);
}

void ZerocrossingDetection(uint8_t motorIndex)
{
	uint16_t reference = adcGetChannel(0);

	uint16_t result_u = XMC_VADC_GROUP_GetResult(motors[motorIndex].inverter.adc_group, motors[motorIndex].inverter.phase_channel[0]);
	uint16_t result_v = XMC_VADC_GROUP_GetResult(motors[motorIndex].inverter.adc_group, motors[motorIndex].inverter.phase_channel[1]);
	uint16_t result_w = XMC_VADC_GROUP_GetResult(motors[motorIndex].inverter.adc_group, motors[motorIndex].inverter.phase_channel[2]);

	if (motors[motorIndex].CCR_dummy < motors[motorIndex].inverter.min_ccr)
	{
		motors[motorIndex].inverter.pattern = 0xff;
		motors[motorIndex].inverter.avg_delay = 300;
		motors[motorIndex].inverter.emergency_stop = 0;
		motors[motorIndex].inverter.emergency_stop_cnt=0;
		motors[motorIndex].inverter.startup=1;
	}
	else
	{
		if (motors[motorIndex].inverter.disable_cnt > 0)
			motors[motorIndex].inverter.disable_cnt--;
		else
		{
			if (motors[motorIndex].inverter.emergency_stop)
			{
				motors[motorIndex].inverter.pattern = 0xff;

				if (calculateThrottleStatus() == THROTTLE_LOW && motors[motorIndex].inverter.emergency_stop_cnt-- == 0)
				{
					motors[motorIndex].inverter.avg_delay = 300;
					motors[motorIndex].inverter.emergency_stop = 0;
					motors[motorIndex].inverter.startup=1;
				}
			}
			else
			{
				motors[motorIndex].inverter.emergency_stop_cnt++;

				if (motors[motorIndex].inverter.startup && motors[motorIndex].inverter.emergency_stop_cnt > 25000)
				{
					motors[motorIndex].inverter.startup = 0;
					motors[motorIndex].inverter.emergency_stop_cnt = 0;
				}
				else if(!motors[motorIndex].inverter.startup)
				{
					if (motors[motorIndex].inverter.emergency_stop_cnt > 100)
						motors[motorIndex].inverter.emergency_stop = 1;
				}

				if (!motors[motorIndex].inverter.crossing_detected)
				{
					switch (motors[motorIndex].inverter.pattern)
					{
						case 0:	if (result_w < reference) motors[motorIndex].inverter.crossing_detected = 1; break;
						case 1:	if (result_v > reference) motors[motorIndex].inverter.crossing_detected = 1; break;
						case 2:	if (result_u < reference) motors[motorIndex].inverter.crossing_detected = 1; break;
						case 3:	if (result_w > reference) motors[motorIndex].inverter.crossing_detected = 1; break;
						case 4: if (result_v < reference) motors[motorIndex].inverter.crossing_detected = 1; break;
						case 5:	if (result_u > reference) motors[motorIndex].inverter.crossing_detected = 1; break;
						default: motors[motorIndex].inverter.crossing_detected = 1;	break;
					}

					if (motors[motorIndex].inverter.pattern != 0xff)
					{
						if (motors[motorIndex].inverter.crossing_detected)
							motors[motorIndex].inverter.avg_delay += (motors[motorIndex].inverter.delay_cnt - motors[motorIndex].inverter.avg_delay)*0.05;
						else
							motors[motorIndex].inverter.delay_cnt++;
					}
				}
				else
				{
					if (motors[motorIndex].inverter.pattern == 0xff)
					{
						motors[motorIndex].inverter.pattern=0;
						motors[motorIndex].inverter.disable_cnt = 50000;		//brake before start
					}
					else
					{
						if (++motors[motorIndex].inverter.com_cnt > motors[motorIndex].inverter.avg_delay)
						{
							motors[motorIndex].inverter.com_cnt = 0;
							motors[motorIndex].inverter.delay_cnt=0;
							motors[motorIndex].inverter.crossing_detected=0;

							motors[motorIndex].inverter.pattern++;
							motors[motorIndex].inverter.disable_cnt = 1;

							if (motors[motorIndex].inverter.pattern > 5)
								motors[motorIndex].inverter.pattern = 0;

							if (!motors[motorIndex].inverter.startup)
								motors[motorIndex].inverter.emergency_stop_cnt=0;
						}
					}
				}
			}
		}
	}
}

void VADC0_G0_0_IRQHandler()
{
	ZerocrossingDetection(0);
	MotorCommutationCCU8(0);
}

void VADC0_G1_0_IRQHandler()
{
	ZerocrossingDetection(1);
	MotorCommutationCCU8(1);
}

void VADC0_G2_0_IRQHandler()
{
	ZerocrossingDetection(2);
	MotorCommutationCCU4(2);
}

void VADC0_G3_0_IRQHandler()
{
	ZerocrossingDetection(3);
	MotorCommutationCCU4(3);
}
#endif
