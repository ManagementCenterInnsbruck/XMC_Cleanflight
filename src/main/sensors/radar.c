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

#include "radar.h"

radar_t radar;



static volatile bool radarFrameComplete = false;
static volatile uint8_t radarFrame[RADAR_FRAME_SIZE_MAX];


static serialPort_t *radarSerialPort;

void radarUpdate(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);

    if (radarFrameComplete) {
    	radar.radarDistance = radar.dev.getDistance(&radarFrame[0]);
    	radar.radarVelocity = radar.dev.getVelocity(&radarFrame[0]);
    	debug[0] = (int16_t) radar.radarDistance;
    	radarFrameComplete = false;
    }
}

static void radarDataReceive(uint16_t c)
{
    uint32_t radarTime, radarTimeInterval;
    static uint32_t radarTimeLast = 0;
    static uint8_t radarFramePosition = 0;

    radarTime = micros();
    radarTimeInterval = radarTime - radarTimeLast;
    radarTimeLast = radarTime;

    if (radarTimeInterval > RADAR_TIMEOUT) {
        radarFramePosition = 0;
    }

    if (radarFramePosition < radar.dev.frameSize) {
    	radarFrame[radarFramePosition++] = (uint8_t)c;
        if (radarFramePosition < radar.dev.frameSize) {
        	radarFrameComplete = false;
        } else {
        	radarFrameComplete = true;
        }
    }
}

bool radarDetect(void) {
	bool portShared = false;
	serialPortConfig_t portConfig;
	serialPortConfig_t *portConfigPtr = &portConfig ;
	portConfigPtr->functionMask = FUNCTION_RADAR_RX ;
	portConfigPtr->identifier = SERIAL_PORT_USART3;

	radarSensor_e radarHardware = RADAR_NONE;

	#ifdef USE_RADAR_DISTANCE2GO
	        if (radarDistance2GoInit(&radar)) {
	            radarHardware = RADAR_DISTANCE2GO;
	        }
	#endif

	#ifdef USE_RADAR_SENSE2GO
	        if (radarSense2GoInit(&radar)) {
	            radarHardware = RADAR_SENSE2GO;
	            break;
	        }
	#endif

	    if (radarHardware == RADAR_NONE) {
	        return false;
	    }

	    detectedSensors[SENSOR_INDEX_RADAR] = radarHardware;
	    sensorsSet(SENSOR_RADAR);


	radarSerialPort = openSerialPort(portConfigPtr->identifier,
			FUNCTION_RADAR_RX,
			radarDataReceive,
			radar.dev.baudRate,
			portShared ||  MODE_RX,
			SERIAL_NOT_INVERTED
			);
	return true;
}
