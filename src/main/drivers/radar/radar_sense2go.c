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

bool radarSense2GoInit(radar_t *radar) {
	radar->dev.baudRate = RADAR_SENSE2GO_BAUDRATE;
	radar->dev.frameSize = RADAR_SENSE2GO_FRAMSIZE;
	radar->dev.getDistance = radarSense2GoGetDistance;
	radar->dev.getVelocity = radarSense2GoGetVelocity;
	radar->radarDetectionConeDecidegrees = RADAR_SENSE2GO_DETECTION_CONE_DECIDEGREES;
	radar->radarMaxRangeCm = RADAR_SENSE2GO_MAX_RANGE_CM;

	return true;
}

static int32_t radarSense2GoGetDistance(volatile uint8_t *radarFrame) {
	return (int32_t) radarFrame[1] << 8 | radarFrame[2];
}

static int32_t radarSense2GoGetVelocity(volatile uint8_t *radarFrame) {
	return 0;
}
