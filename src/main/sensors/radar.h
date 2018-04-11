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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "string.h"
#include "platform.h"
#include "common/maths.h"


typedef int32_t (*radarOpFuncPtr)(volatile uint8_t *radarFrame);
typedef bool (*radarCheckFuncPtr)(volatile uint8_t *radarFrame);

typedef struct radarDev_s {
	uint8_t frameSize;
	uint32_t baudRate;
	radarOpFuncPtr getDistance;
	radarOpFuncPtr getVelocity;
	radarCheckFuncPtr isDataValid;
}radarDev_t;

typedef struct radar_s {
	radarDev_t dev;
	uint32_t radarDistance;
	int32_t radarMaxRangeCm;
	int32_t radarVelocity;
	uint16_t radarDetectionConeDecidegrees;
}radar_t;


typedef enum {
    RADAR_DEFAULT = 0,
    RADAR_NONE = 1,
    RADAR_DISTANCE2GO = 2,
    RADAR_SENSE2GO = 3,
} radarSensor_e;

#include "drivers/serial.h"
#include "drivers/time.h"
#include "io/serial.h"
#include "fc/runtime_config.h"
#include "build/debug.h"

#include "drivers/radar/radar_distance2go.h"
#include "drivers/radar/radar_sense2go.h"
#include "sensors/sensors.h"

#define RADAR_FRAME_SIZE_MAX (20+2)  //20 databytes + 2 Headerbytes
#define RADAR_FRAME_BEGIN_BYTE 0xAA
#define RADAR_FRAME_STOP_BYTE  0xBB

bool radarDetect(void);
void radarUpdate(timeUs_t currentTimeUs);


