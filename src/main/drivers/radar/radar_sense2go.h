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
#include "sensors/sensors.h"
#include "sensors/radar.h"

#define RADAR_SENSE2GO_FRAMSIZE 						4
#define RADAR_SENSE2GO_BAUDRATE 						115200
#define RADAR_SENSE2GO_MAX_RANGE_CM 					500
#define RADAR_SENSE2GO_DETECTION_CONE_DECIDEGREES 		300


bool radarSense2GoInit(radar_t *radar);
