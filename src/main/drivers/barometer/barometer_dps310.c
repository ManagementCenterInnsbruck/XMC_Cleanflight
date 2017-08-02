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

#include <platform.h>

#include "build/build_config.h"

#include "barometer.h"

#include "drivers/bus_i2c.h"
#include "drivers/exti.h"
#include "drivers/gpio.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/time.h"

#include "barometer_dps310.h"

#ifdef BARO

typedef struct {
	// coefficients to calculate physical values
	int32_t c00; /**< coefficients to calculate physical values */
	int32_t c10; /**< coefficients to calculate physical values */
	int32_t c20; /**< coefficients to calculate physical values */
	int32_t c30; /**< coefficients to calculate physical values */
	int32_t c01; /**< coefficients to calculate physical values */
	int32_t c11; /**< coefficients to calculate physical values */
	int32_t c21; /**< coefficients to calculate physical values */
	int32_t c0; /**< coefficients to calculate physical values */
	int32_t c1; /**< coefficients to calculate physical values */
	// compensation scale factors (data sheet page 15, table 9)
	uint32_t kP; /**< compensation scale factor for pressure measurement (data sheet page 15, table 9) */
	uint32_t kT; /**< compensation scale factor for temperature measurement (data sheet page 15, table 9) */
} dps310_calibration_param_t;

typedef struct {
	dps310_calibration_param_t cal_param;
	uint8_t dev_addr;
	int16_t oversampling_setting;
} dps310_t;

#define DPS310_Address 0x77

#define PRS_CFG 0x06 ///pressure config register
#define TMP_CFG 0x07 ///temperature config register
#define MEAS_CFG 0x08 ///Sensor Operating mode register
#define CFG_REG 0x09///Interrupt and FIFO-config
#define INT_STS 0x0A///Interrupt status register
#define RESET_REGISTER 0x0C///reset register of sensor
#define COEF_SRC 0x28///Temperature coefficient source

//reset
#define SOFT_RESET 0x09///sequence to perform soft-reset
#define FIFO_FLUSH 0x80///flush FIFO-Buffer

//sensor interrupt and fifo configuration
#define GENERATE_IR_T_SHIFT_P_SHIFT 0x2C///Generate interrupt if new pressure-value is available + shift values right(see datasheet for info)
#define GENERATE_IR_PR 0x20 ///Generate Interrupt if new pressure-value is available, without T_SHIFT and P_SHIFT

//sensor operating mode and status register-config
#define MEAS_CTRL_CONTINUES_TEMP_PRESSURE 0x07
#define MEAS_CTRL_CONTINUES_TEMP 0x06
#define MEAS_CTRL_CONTINUES_PRESSURE 0x05

//pressure-config
#define PM_RATE_128 0x70 ///128 messungen/sec
#define PM_PRC_128 0x07 ///pressure oversampling rate -128 times

#define PM_RATE_64 0x60 ///64 messungen/sec
#define PM_PRC_64 0x06 ///pressure oversampling rate -64 times

#define PM_RATE_32 0x50 ///32 messungen/sec
#define PM_PRC_32 0x05 ///pressure oversampling rate -32 times

#define PM_RATE_16 0x40 ///16 messungen/sec
#define PM_PRC_16 0x04 ///pressure oversampling rate -16 times

#define PM_RATE_8 0x30 ///8 messungen/sec
#define PM_PRC_8 0x03 ///pressure oversampling rate -8 times

#define PM_RATE_4 0x20 ///4 messungen/sec
#define PM_PRC_4 0x02 ///pressure oversampling rate -4 times

#define PM_RATE_2 0x10 ///2 messungen/sec
#define PM_PRC_2 0x01 ///pressure oversampling rate -2 times

#define PM_RATE_1 0x00 ///1 messungen/sec
#define PM_PRC_1 0x00 ///pressure oversampling rate -1 time

//temp-config
#define TMP_RATE_128 0x70 ///128 messungen/sec
#define TMP_PRC_128 0x07 ///temp oversampling rate -128 times

#define TMP_RATE_64 0x60 ///64 messungen/sec
#define TMP_PRC_64 0x06 ///temp oversampling rate -64 times

#define TMP_RATE_32 0x50 ///32 messungen/sec
#define TMP_PRC_32 0x05 ///temp oversampling rate -32 times

#define TMP_RATE_16 0x40 ///16 messungen/sec
#define TMP_PRC_16 0x04 ///temp oversampling rate -16 times

#define TMP_RATE_8 0x30 ///8 messungen/sec
#define TMP_PRC_8 0x03 ///temp oversampling rate -8 times

#define TMP_RATE_4 0x20 ///4 messungen/sec
#define TMP_PRC_4 0x02 ///temp oversampling rate -4 times

#define TMP_RATE_2 0x10 ///2 messungen/sec
#define TMP_PRC_2 0x01 ///temp oversampling rate -2 times

#define TMP_RATE_1 0x00 ///1 messungen/sec
#define TMP_PRC_1 0x00 ///temp oversampling rate -1 time

STATIC_UNIT_TESTED dps310_t dps310;

#define UT_DELAY    6000        // 1.5ms margin according to the spec (4.5ms T conversion time)
#define UP_DELAY    27000       // 6000+21000=27000 1.5ms margin according to the spec (25.5ms P conversion time with OSS=3)

static bool dps310InitDone = false;
STATIC_UNIT_TESTED uint16_t bmp085_ut; // static result of temperature measurement
STATIC_UNIT_TESTED uint32_t bmp085_up;  // static result of pressure measurement

static void bmp085_get_cal_param(void);
static void bmp085_start_ut(void);
static void bmp085_get_ut(void);
static void bmp085_start_up(void);
static void bmp085_get_up(void);
static int32_t bmp085_get_temperature(uint32_t ut);
static int32_t bmp085_get_pressure(uint32_t up);
STATIC_UNIT_TESTED void bmp085_calculate(int32_t *pressure,
		int32_t *temperature);

bool dps310Detect(baroDev_t *baro) {
//	uint8_t data;
//	bool ack;
//
//	if (dps310InitDone)
//		return true;
//
//	delay(20); // datasheet says 10ms, we'll be careful and do 20.
//
//	ack = i2cRead(BARO_I2C_INSTANCE, BMP085_I2C_ADDR, BMP085_CHIP_ID__REG, 1,
//			&data); /* read Chip Id */
//	if (ack) {
//		bmp085.chip_id = BMP085_GET_BITSLICE(data, BMP085_CHIP_ID);
//		bmp085.oversampling_setting = 3;
//
//		if (bmp085.chip_id == BMP085_CHIP_ID) { /* get bitslice */
//			i2cRead(BARO_I2C_INSTANCE, BMP085_I2C_ADDR, BMP085_VERSION_REG, 1,
//					&data); /* read Version reg */
//			bmp085.ml_version = BMP085_GET_BITSLICE(data, BMP085_ML_VERSION); /* get ML Version */
//			bmp085.al_version = BMP085_GET_BITSLICE(data, BMP085_AL_VERSION); /* get AL Version */
//			bmp085_get_cal_param(); /* readout bmp085 calibparam structure */
//			baro->ut_delay = UT_DELAY;
//			baro->up_delay = UP_DELAY;
//			baro->start_ut = bmp085_start_ut;
//			baro->get_ut = bmp085_get_ut;
//			baro->start_up = bmp085_start_up;
//			baro->get_up = bmp085_get_up;
//			baro->calculate = bmp085_calculate;
//
//			dps310InitDone = true;
//			return true;
//		}
//	}
//
//#if defined(BARO_EOC_GPIO)
//	if (eocIO)
//	EXTIRelease(eocIO);
//#endif


	return false;
}

static int32_t bmp085_get_temperature(uint32_t ut) {
	int32_t temperature;

	return temperature;
}

static int32_t bmp085_get_pressure(uint32_t up) {
	int32_t pressure;

	return pressure;
}

static void bmp085_start_ut(void) {

}

static void bmp085_get_ut(void) {

}

static void bmp085_start_up(void) {

}

/** read out up for pressure conversion
 depending on the oversampling ratio setting up can be 16 to 19 bit
 \return up parameter that represents the uncompensated pressure value
 */
static void bmp085_get_up(void) {
	uint8_t data[3];

}

STATIC_UNIT_TESTED void bmp085_calculate(int32_t *pressure,
		int32_t *temperature) {
	int32_t temp, press;

	temp = bmp085_get_temperature(bmp085_ut);
	press = bmp085_get_pressure(bmp085_up);
	if (pressure)
		*pressure = press;
	if (temperature)
		*temperature = temp;
}

static void bmp085_get_cal_param(void) {

}

#endif /* BARO */
