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

#include "radar_distance2go.h"

static int32_t radarDistance2GoGetDistance(volatile uint8_t *radarFrame);
static int32_t radarDistance2GoGetVelocity(volatile uint8_t *radarFrame);
static bool radarDistance2GoIsDataValid(volatile uint8_t *radarFrame);

bool radarDistance2GoInit(radar_t *radar) {
	radar->dev.baudRate = RADAR_DISTANCE2GO_BAUDRATE;
	radar->dev.frameSize = RADAR_DISTANCE2GO_FRAMESIZE;
	radar->dev.getDistance = radarDistance2GoGetDistance;
	radar->dev.getVelocity = radarDistance2GoGetVelocity;
	radar->dev.isDataValid = radarDistance2GoIsDataValid;
	radar->radarDetectionConeDecidegrees = RADAR_DISTANCE2GO_DETECTION_CONE_DECIDEGREES;
	radar->radarMaxRangeCm = RADAR_DISTANCE2GO_MAX_RANGE_CM;

	return true;
}

static int32_t radarDistance2GoGetDistance(volatile uint8_t *radarFrame) {
	return (int32_t) radarFrame[1] << 8 | radarFrame[2];
}

static int32_t radarDistance2GoGetVelocity(volatile uint8_t *radarFrame) {
	return 0;
}

static bool radarDistance2GoIsDataValid(volatile uint8_t *radarFrame) {

	if (radarFrame[RADAR_DISTANCE2GO_FRAMESIZE-1] != RADAR_FRAME_STOP_BYTE)
	{
		return false;
	}

	return true;
}

