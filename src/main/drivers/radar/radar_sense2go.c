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

#include "radar_sense2go.h"

static int32_t radarSense2GoGetDistance(volatile uint8_t *radarFrame);
static int32_t radarSense2GoGetVelocity(volatile uint8_t *radarFrame);
static bool radarSense2GoIsDataValid(volatile uint8_t *radarFrame);

XMC_FCE_t FCE_config =
{
  .kernel_ptr 	 = XMC_FCE_CRC8,    /**< FCE Kernel Pointer */
  .fce_cfg_update.config_refin = XMC_FCE_REFIN_RESET,
  .fce_cfg_update.config_refout = XMC_FCE_REFOUT_RESET,
  .fce_cfg_update.config_xsel = XMC_FCE_INVSEL_SET,
  .seedvalue	 = 0xff/*initial value*/
};

bool radarSense2GoInit(radar_t *radar) {
	radar->dev.baudRate = RADAR_SENSE2GO_BAUDRATE;
	radar->dev.frameSize = RADAR_SENSE2GO_FRAMESIZE;
	radar->dev.getDistance = radarSense2GoGetDistance;
	radar->dev.getVelocity = radarSense2GoGetVelocity;
	radar->dev.isDataValid = radarSense2GoIsDataValid;
	radar->radarDetectionConeDecidegrees = RADAR_SENSE2GO_DETECTION_CONE_DECIDEGREES;
	radar->radarMaxRangeCm = RADAR_SENSE2GO_MAX_RANGE_CM;

	/* Enable FCE module for CRC Check*/
	XMC_FCE_Enable();
	XMC_FCE_Init(&FCE_config);

	return true;
}

static int32_t radarSense2GoGetDistance(volatile uint8_t *radarFrame) {
	return (int32_t) radarFrame[1] << 8 | radarFrame[2];
}

static int32_t radarSense2GoGetVelocity(volatile uint8_t *radarFrame) {
	return (int32_t) radarFrame[3] << 8 | radarFrame[4];
}

static bool radarSense2GoIsDataValid(volatile uint8_t *radarFrame){
	uint8_t CRCResult = 0;
	const uint8_t *framePointer = (const uint8_t*)radarFrame;

	//check STOP Byte
	if (radarFrame[RADAR_SENSE2GO_FRAMESIZE-1] != RADAR_FRAME_STOP_BYTE)
	{
		return false;
	}

	//CRC Check
	XMC_FCE_InitializeSeedValue(&FCE_config, 0xff);
	XMC_FCE_CalculateCRC8(&FCE_config, framePointer, RADAR_SENSE2GO_FRAMESIZE-2, &CRCResult);
	XMC_FCE_GetCRCResult(&FCE_config, (uint32_t*)&CRCResult);

	if (CRCResult == radarFrame[RADAR_SENSE2GO_FRAMESIZE-2]) {
		return true;
	}
	else {
		return false;
	}
}
