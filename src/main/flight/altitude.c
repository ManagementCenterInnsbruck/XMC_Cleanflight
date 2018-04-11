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
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/altitude.h"
#include "flight/imu.h"
#include "flight/pid.h"

#include "rx/rx.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"
#include "sensors/sonar.h"


int32_t AltHold;
static int32_t estimatedVario = 0;                      // variometer in cm/s
static int32_t estimatedAltitude = 0;                // in cm


#if defined(BARO) || defined(SONAR)

enum {
    DEBUG_ALTITUDE_ACC,
    DEBUG_ALTITUDE_VEL,
    DEBUG_ALTITUDE_HEIGHT
};

PG_REGISTER_WITH_RESET_TEMPLATE(airplaneConfig_t, airplaneConfig, PG_AIRPLANE_CONFIG, 0);

PG_RESET_TEMPLATE(airplaneConfig_t, airplaneConfig,
    .fixedwing_althold_reversed = false
);

static int32_t setVelocity = 0;
static uint8_t velocityControl = 0;
static int32_t errorVelocityI = 0;
static int32_t altHoldThrottleAdjustment = 0;
static int16_t initialThrottleHold;

// 40hz update rate (20hz LPF on acc)
#define BARO_UPDATE_FREQUENCY_40HZ (1000 * 25)

#define DEGREES_80_IN_DECIDEGREES 800

static void applyMultirotorAltHold(void)
{
    static uint8_t isAltHoldChanged = 0;
    static int16_t AltHold_deltaThrottle = 0.0f;
    static float AltHold_scaleClimbingVel = 4.0f;

    // multirotor alt hold
    if (rcControlsConfig()->alt_hold_fast_change)
    {
        // rapid alt changes
        if (ABS(rcData[THROTTLE] - initialThrottleHold) > rcControlsConfig()->alt_hold_deadband)
        {
            errorVelocityI = 0;
            isAltHoldChanged = 1;
            rcCommand[THROTTLE] += (rcData[THROTTLE] > initialThrottleHold) ? -rcControlsConfig()->alt_hold_deadband : rcControlsConfig()->alt_hold_deadband;
        }
        else
        {
            if (isAltHoldChanged)
            {
                AltHold = estimatedAltitude;
                isAltHoldChanged = 0;
            }
            rcCommand[THROTTLE] = constrain(initialThrottleHold + altHoldThrottleAdjustment, PWM_RANGE_MIN, PWM_RANGE_MAX);
        }
    }
    else
    {
    	AltHold_deltaThrottle = rcData[THROTTLE] - initialThrottleHold;
        // slow alt changes, mostly used for aerial photography
        if (ABS(AltHold_deltaThrottle) > rcControlsConfig()->alt_hold_deadband)
        {
        	AltHold_deltaThrottle = applyDeadband(AltHold_deltaThrottle,rcControlsConfig()->alt_hold_deadband);

            // set velocity proportional to stick movement
            setVelocity = (int32_t)(AltHold_deltaThrottle) / AltHold_scaleClimbingVel;
            velocityControl = 1;
            isAltHoldChanged = 1;
        }
        else if (isAltHoldChanged)
        {
            AltHold = estimatedAltitude;
            velocityControl = 0;
            isAltHoldChanged = 0;
        }
        rcCommand[THROTTLE] = constrain(initialThrottleHold + altHoldThrottleAdjustment, PWM_RANGE_MIN, PWM_RANGE_MAX);
    }
}

static void applyFixedWingAltHold(void)
{
    // handle fixedwing-related althold. UNTESTED! and probably wrong
    // most likely need to check changes on pitch channel and 'reset' althold similar to
    // how throttle does it on multirotor

    rcCommand[PITCH] += altHoldThrottleAdjustment * GET_DIRECTION(airplaneConfig()->fixedwing_althold_reversed);
}

void applyAltHold(void)
{
    if (STATE(FIXED_WING))
    {
        applyFixedWingAltHold();
    }
    else
    {
        applyMultirotorAltHold();
    }
}

void updateAltHoldState(void)
{
    // Baro alt hold activate
    if (!IS_RC_MODE_ACTIVE(BOXBARO))
    {
        DISABLE_FLIGHT_MODE(BARO_MODE);
        return;
    }

    if (!FLIGHT_MODE(BARO_MODE))
    {
        ENABLE_FLIGHT_MODE(BARO_MODE);
        AltHold = estimatedAltitude;
        initialThrottleHold = rcData[THROTTLE];
        errorVelocityI = 0;
        altHoldThrottleAdjustment = 0;
    }
}

void updateSonarAltHoldState(void)
{
    // Sonar alt hold activate
    if (!IS_RC_MODE_ACTIVE(BOXSONAR))
    {
        DISABLE_FLIGHT_MODE(SONAR_MODE);
        return;
    }

    if (!FLIGHT_MODE(SONAR_MODE))
    {
        ENABLE_FLIGHT_MODE(SONAR_MODE);
        AltHold = estimatedAltitude;
        initialThrottleHold = rcData[THROTTLE];
        errorVelocityI = 0;
        altHoldThrottleAdjustment = 0;
    }
}

bool isThrustFacingDownwards(attitudeEulerAngles_t *attitude)
{
    return ABS(attitude->values.roll) < DEGREES_80_IN_DECIDEGREES && ABS(attitude->values.pitch) < DEGREES_80_IN_DECIDEGREES;
}

int32_t calculateAltHoldThrottleAdjustment(int32_t vel_tmp, float accZ_tmp, float accZ_old)
{
    int32_t result = 0;
    int32_t error;
    int32_t setVel;

    if (!isThrustFacingDownwards(&attitude))
    {
        return result;
    }

    // Altitude P-Controller

    if (!velocityControl)
    {
        error = constrain(AltHold - estimatedAltitude, -500, 500);
        error = applyDeadband(error, 5); // remove small P parameter to reduce noise near zero position
        setVel = constrain((currentPidProfile->pid[PID_ALT].P * error / 128), -100, +100); // limit velocity to +/- 1 m/s

    }
    else
    {
        setVel = setVelocity;
    }
    // Velocity PID-Controller

    // P
    error = setVel - vel_tmp;
    result = constrain((currentPidProfile->pid[PID_VEL].P * error / 32), -300, +300);

    // I
    errorVelocityI += (currentPidProfile->pid[PID_VEL].I * error);
    errorVelocityI = constrain(errorVelocityI, -(8192 * 200), (8192 * 200));
    result += (int32_t)errorVelocityI / 8192;     // I in range +/-200

    // D
    result -= constrain(currentPidProfile->pid[PID_VEL].D * (accZ_tmp + accZ_old) / 512, -150, 150);

    return result;
}

void calculateEstimatedAltitude(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimeUs = 0;
    const uint32_t dTime = currentTimeUs - previousTimeUs;
    if (dTime < BARO_UPDATE_FREQUENCY_40HZ)
    {
        return;
    }
    previousTimeUs = currentTimeUs;

    static float vel = 0.0f;
    static float accAlt = 0.0f;

    int32_t baroAlt = 0;
#ifdef BARO
    if (sensors(SENSOR_BARO))
    {
        if (!isBaroCalibrationComplete())
        {
            performBaroCalibrationCycle();
            vel = 0;
            accAlt = 0;
        }
        else
        {
            baroAlt = baroCalculateAltitude();
            estimatedAltitude = baroAlt;
        }
    }
#endif

#ifdef SONAR
    if (sensors(SENSOR_SONAR))
    {
        int32_t sonarAlt = sonarCalculateAltitude(sonarRead(), getCosTiltAngle());
        if (sonarAlt > 0 && sonarAlt >= sonarCfAltCm && sonarAlt <= sonarMaxAltWithTiltCm)
        {
            // SONAR in range, so use complementary filter
            float sonarTransition = (float)(sonarMaxAltWithTiltCm - sonarAlt) / (sonarMaxAltWithTiltCm - sonarCfAltCm);
            sonarAlt = (float)sonarAlt * sonarTransition + baroAlt * (1.0f - sonarTransition);
            estimatedAltitude = sonarAlt;
        }
    }
#endif

    float accZ_tmp = 0;
#ifdef ACC
    if (sensors(SENSOR_ACC))
    {
        const float dt = accTimeSum * 1e-6f; // delta acc reading time in seconds

        // Integrator - velocity, cm/sec
        if (accSumCount)
        {
            accZ_tmp = (float)accSum[2] / accSumCount;
        }
        const float vel_acc = accZ_tmp * accVelScale * (float)accTimeSum;

        // Integrator - Altitude in cm
        accAlt += (vel_acc * 0.5f) * dt + vel * dt;  // integrate velocity to get distance (x= a/2 * t^2)
        accAlt = accAlt * CONVERT_PARAMETER_TO_FLOAT(barometerConfig()->baro_cf_alt) + (float)baro.BaroAlt * (1.0f - CONVERT_PARAMETER_TO_FLOAT(barometerConfig()->baro_cf_alt));    // complementary filter for altitude estimation (baro & acc)
        vel += vel_acc;
        estimatedAltitude = accAlt;
    }
#endif

    DEBUG_SET(DEBUG_ALTITUDE, DEBUG_ALTITUDE_ACC, accSum[2] / accSumCount);
    DEBUG_SET(DEBUG_ALTITUDE, DEBUG_ALTITUDE_VEL, vel);
    DEBUG_SET(DEBUG_ALTITUDE, DEBUG_ALTITUDE_HEIGHT, accAlt);

    imuResetAccelerationSum();

    int32_t baroVel = 0;
#ifdef BARO
    if (sensors(SENSOR_BARO))
    {
        if (!isBaroCalibrationComplete())
        {
            return;
        }

        static int32_t lastBaroAlt = 0;
        baroVel = (baroAlt - lastBaroAlt) * 1000000.0f / dTime;
        lastBaroAlt = baroAlt;

        baroVel = constrain(baroVel, -150, 150);  // constrain baro velocity +/- 150cm/s
        baroVel = applyDeadband(baroVel, 10);       // to reduce noise near zero
    }
#endif // BARO

    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    vel = vel * CONVERT_PARAMETER_TO_FLOAT(barometerConfig()->baro_cf_vel) + baroVel * (1.0f - CONVERT_PARAMETER_TO_FLOAT(barometerConfig()->baro_cf_vel));
    int32_t vel_tmp = lrintf(vel);
    vel_tmp = constrain(vel_tmp, -1500, 1500);

    // set vario
    estimatedVario = applyDeadband(vel_tmp, 5);

    static float accZ_old = 0.0f;
    altHoldThrottleAdjustment = calculateAltHoldThrottleAdjustment(vel_tmp, accZ_tmp, accZ_old);
    accZ_old = accZ_tmp;
}
#endif // defined(BARO) || defined(SONAR)

int32_t getEstimatedAltitude(void)
{
    return estimatedAltitude;
}

int32_t getEstimatedVario(void)
{
    return estimatedVario;
}
