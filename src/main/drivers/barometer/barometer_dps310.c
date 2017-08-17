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
#include "math.h"

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
#define DPS310_PROM_DATA__LEN1 18
#define DPS310_PROM_DATA__LEN2 6

#define PSR_B2 0x00
#define PSR_B1 0x01
#define PSR_B0 0x02
#define TMP_B2 0x03
#define TMP_B1 0x04
#define TMP_B0 0x05
#define PRS_CFG 0x06 ///pressure config register
#define TMP_CFG 0x07 ///temperature config register
#define MEAS_CFG 0x08 ///Sensor Operating mode register
#define CFG_REG 0x09///Interrupt and FIFO-config
#define INT_STS 0x0A///Interrupt status register
#define RESET_REGISTER 0x0C///reset register of sensor
#define COEF_SRC 0x28///Temperature coefficient source
#define CAL_COEF_START_ADDR1 0x10
#define CAL_COEF_START_ADDR2 0x1A

STATIC_UNIT_TESTED uint32_t dps310_ut;  // static result of temperature measurement
STATIC_UNIT_TESTED int32_t dps310_up;  // static result of pressure measurement

float up_sc = 0.0f;
float ut_sc = 0.0f;

//reset
#define SOFT_RESET 0x09///sequence to perform soft-reset
#define FIFO_FLUSH 0x80///flush FIFO-Buffer

//sensor operating mode and status register-config
#define MEAS_CTRL_TEMP_SINGLE 0x02
#define MEAS_CTRL_PRESSURE_SINGLE 0x01
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

//#define UT_DELAY    6000        // 1.5ms margin according to the spec (4.5ms T conversion time)
//#define UP_DELAY    27000       // 6000+21000=27000 1.5ms margin according to the spec (25.5ms P conversion time with OSS=3)
#define UT_DELAY    60000        // 1.5ms margin according to the spec (4.5ms T conversion time)
#define UP_DELAY    50000       // 6000+21000=27000 1.5ms margin according to the spec (25.5ms P conversion time with OSS=3)

static bool dps310InitDone = false;
bool dps310Detect(baroDev_t *baro);
static void dps310_get_cal_param(void);
static void dps310_start_ut(void);
static void dps310_get_ut(void);
static void dps310_start_up(void);
static void dps310_get_up(void);
static int32_t dps310_get_temperature(uint32_t ut);
static int32_t dps310_get_pressure(int32_t up);
STATIC_UNIT_TESTED void dps310_calculate(int32_t *pressure,
		int32_t *temperature);

bool dps310Detect(baroDev_t *baro) {

	if (dps310InitDone)
	return true;

	delay(12);		/* wait 12ms for sensor to be ready*/

	//*****************************************************************
	//		Attention: Measurement Time of the DPS310 must be <1s!
	//	  Measurement Time = PM_Rate*Time_Meas + TMP_RATE*Time_Meas
	//*****************************************************************
	//	Table: Time for 1 Measurement with Oversampling (PRC)
	//	_____________________________________________________________________________
	// |             |       |       |       |       |       |       |       |       |
	// |     PRC     |   1   |   2   |   4   |   8   |   16  |   32  |   64  |  128  |
	// |_____________|_______|_______|_______|_______|_______|_______|_______|_______|
	// |             |       |       |       |       |       |       |       |       |
	// |  Time_Meas  |  3.6  |  5.2  |  8.4  | 14.8  | 27.6  | 53.2  | 104.4 | 206.8 |
	// |     [ms]    |       |       |       |       |       |       |       |       |
	// |_____________|_______|_______|_______|_______|_______|_______|_______|_______|
	//
	// --------------------------Measurement Modes (MEAS_config)--------------------------
	//		Standby Mode:		0x00	Idle/Stop background measurement
	//		Command Mode:		0x01	Single Pressure Measurement
	//							0x02	Single Temperature Measurement
	//		Background Mode:	0x05	Continuous Pressure Measurement (change in updateValues, if (x08>>4 == 13))
	//							0x06	Continuous Temperature Measurement
	//							0x07 	Continuous Pressure and Temperature Measurement (change in updateValues, if (x08>>4 == 15))
	//
	//--------------------Interrupt an FIFO Configuration(CFG_config)--------------------
	//						(check DPS310 Datasheet for Details)
	//					0x01	No Interrupts, FIFO disabled, 3-Wire Interface
	//					0x11	Pressure Interrupt, FIFO disabled, 3-Wire Interface
	//					0x21	Temperature Interrupt, FIFO disabled, 3-Wire Interface
	//
	//------------------------------configure sensor start------------------------------
	/*values per seconds only appricable for measurements in background mode*/
		uint8_t P_config =  PM_PRC_4;	// values/sec | oversamplingrate
		uint8_t T_config =  TMP_PRC_4;	// values/sec | oversampling rate
	//-------------------------------configure sensor end-------------------------------
		i2cWrite(BARO_I2C_INSTANCE, DPS310_Address, RESET_REGISTER, SOFT_RESET | FIFO_FLUSH);	// Software Reset, FIFO-Flush
		delay(40);

		/* check which temperature sensor is used (bit7 == 0 => ASIC, bit7 == 1 => MEMS) */
		uint8_t x28;
		i2cRead(BARO_I2C_INSTANCE, DPS310_Address, COEF_SRC, 1, &x28);
		x28 = x28>>7;
		if (x28 == 1) i2cWrite(BARO_I2C_INSTANCE, DPS310_Address, TMP_CFG , (1<<7) | T_config);  // use external sensor (MEMS)
		else  i2cWrite(BARO_I2C_INSTANCE, DPS310_Address,TMP_CFG , T_config);  				  // use internal sensor (ASIC)

		i2cWrite(BARO_I2C_INSTANCE, DPS310_Address, PRS_CFG, P_config);
		i2cWrite(BARO_I2C_INSTANCE, DPS310_Address, CFG_REG, 0x00);

		dps310_get_cal_param(); /* readout bmp085 calibparam structure */
		baro->ut_delay = UT_DELAY;
		baro->up_delay = UP_DELAY;
		baro->start_ut = dps310_start_ut;
		baro->get_ut = dps310_get_ut;
		baro->start_up = dps310_start_up;
		baro->get_up = dps310_get_up;
		baro->calculate = dps310_calculate;

		dps310InitDone = true;

	return true;
}

static int32_t dps310_get_temperature(uint32_t ut) {

	int32_t temperature;

	// calculate physical temperature Tcomp [°C]
	ut_sc = (float) ut / dps310.cal_param.kT;
	temperature = (dps310.cal_param.c0 * 0.5 + dps310.cal_param.c1 * ut_sc) ;//* 100; //make 0.01 °C

	return temperature;
}

static int32_t dps310_get_pressure(int32_t up) {

	float pressure;
	float T_raw_sc;
	float P_raw_sc;

	float offset = 0.0f;
	float firstTerm = 0.0f;
	float secondTerm = 0.0f;
	float thirdTerm = 0.0f;

	// calculate physical pressure Pcomp [Pa]
	up_sc = (float) up / dps310.cal_param.kP;

	T_raw_sc = ut_sc;
	P_raw_sc = up_sc;

	offset = dps310.cal_param.c00;
	firstTerm  = P_raw_sc * ( (float) dps310.cal_param.c10 + P_raw_sc * ( (float)dps310.cal_param.c20 + P_raw_sc * (float) dps310.cal_param.c30));
	secondTerm = T_raw_sc * (float) dps310.cal_param.c01;
	thirdTerm  = T_raw_sc * P_raw_sc * ( (float) dps310.cal_param.c11 + P_raw_sc * (float) dps310.cal_param.c21);

	pressure = offset + firstTerm + secondTerm + thirdTerm;

//	pressure = dps310.cal_param.c00 + up_sc * (dps310.cal_param.c10 + up_sc * (dps310.cal_param.c20 + up_sc * dps310.cal_param.c30))
//			+ ut_sc * dps310.cal_param.c01 + ut_sc * up_sc * (dps310.cal_param.c11 + up_sc * dps310.cal_param.c21);
	return pressure;
}

static void dps310_start_ut(void) {

	i2cWrite(BARO_I2C_INSTANCE, DPS310_Address, MEAS_CFG, 0xc0 | MEAS_CTRL_TEMP_SINGLE);
}

static void dps310_get_ut(void) {

	uint8_t data[3];
	uint32_t ut_temp = 0U;
	uint8_t x08;

	i2cRead(BARO_I2C_INSTANCE, DPS310_Address, MEAS_CFG, 1, &x08);
	/*read temperature register only if new values are ready*/

	uint8_t temp_ready = (x08 >> 4) & 0b10;

	if (temp_ready)
	{
		i2cRead(BARO_I2C_INSTANCE, DPS310_Address, TMP_B2, 3, data);
		ut_temp = (uint32_t) (data[0] << 16 | data[1] << 8 | data[2]);
		/* generate ut
		 24bit two's complement value out of reg 3-5*/
		if (ut_temp > 8388607u)	// convert to signed int (ut > (ut(2, 23) - 1))
		{
			ut_temp = ut_temp - 16777216u; /*ut - pow(2, 24)*/
		}
		dps310_ut = ut_temp;
	}
}

static void dps310_start_up(void) {

	i2cWrite(BARO_I2C_INSTANCE, DPS310_Address, MEAS_CFG, 0xc0 | MEAS_CTRL_PRESSURE_SINGLE);
}

static void dps310_get_up(void) {

	uint8_t data[3];
	int32_t up_temp = 0U;
	uint8_t x08;
	i2cRead(BARO_I2C_INSTANCE, DPS310_Address, MEAS_CFG, 1, &x08);

	uint8_t pressure_ready = (x08 >> 4) & 0b1;

	if (pressure_ready)
	{
		i2cRead(BARO_I2C_INSTANCE, DPS310_Address, PSR_B2, 3, &data[0]);
		i2cRead(BARO_I2C_INSTANCE, DPS310_Address, PSR_B1, 3, &data[1]);
		i2cRead(BARO_I2C_INSTANCE, DPS310_Address, PSR_B0, 3, &data[2]);

		up_temp = (int32_t) (data[0] << 16 | data[1] << 8 | data[2]);
		/* generate Praw */
		if (up_temp > 8388607u)	/*convert to signed int (Praw > (pow(2, 23) - 1))*/
		{
			up_temp = up_temp - 16777216u;  /*Praw - pow(2, 24)*/
		}
		dps310_up = up_temp;
	}
}

STATIC_UNIT_TESTED void dps310_calculate(int32_t *pressure,int32_t *temperature)
{
	int32_t temp, press;

	temp = dps310_get_temperature(dps310_ut);
	press = dps310_get_pressure(dps310_up);
	if (pressure)
		*pressure = press;
	if (temperature)
		*temperature = temp;
}

static void dps310_get_cal_param(void) {

	uint8_t data[DPS310_PROM_DATA__LEN1];
	i2cRead(BARO_I2C_INSTANCE, DPS310_Address, CAL_COEF_START_ADDR1,
	DPS310_PROM_DATA__LEN1, data);

	/* get fused coefficients from a calibrated sensor */
	dps310.cal_param.c0 = (data[0] << 4) | (data[1] >> 4);
	if (dps310.cal_param.c0 > (powf(2.0f, 11.0f) - 1.0f)) {
		dps310.cal_param.c0 = dps310.cal_param.c0 - powf(2.0f, 12.0f);
	}

	uint8_t c1 = (data[1] << 4); /* "delete" bit7 - bit4 */
	dps310.cal_param.c1 = (c1 << 4) | (data[2]);
	if (dps310.cal_param.c1 > (powf(2.0f, 11.0f) - 1.0f)) {
		dps310.cal_param.c1 = dps310.cal_param.c1 - powf(2.0f, 12.0f);
	}

	dps310.cal_param.c00 = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
	if (dps310.cal_param.c00 > (powf(2.0f, 19.0f) - 1.0f)) {
		dps310.cal_param.c00 = dps310.cal_param.c00 - powf(2.0f, 20.0f);
	}

	uint8_t c10_ = (data[5] << 4);		/* "delete" bit7 - bit4*/
	dps310.cal_param.c10 = (c10_ << 12) | (data[6] << 8) | data[7];
	if (dps310.cal_param.c10 > (powf(2.0f, 19.0f) - 1.0f)) {
		dps310.cal_param.c10 = dps310.cal_param.c10 - powf(2.0f, 20.0f);
	}

	dps310.cal_param.c01 = (data[8] << 8) | data[9];
	if (dps310.cal_param.c01 > (powf(2.0f, 15.0f) - 1.0f)) {
		dps310.cal_param.c01 = dps310.cal_param.c01 - powf(2.0f, 16.0f);
	}


	dps310.cal_param.c11 = (data[10] << 8) | data[11];
	if (dps310.cal_param.c11 > (powf(2.0f, 15.0f) - 1.0f)) {
		dps310.cal_param.c11 = dps310.cal_param.c11 - powf(2.0f, 16.0f);
	}

	dps310.cal_param.c20 = (data[12] << 8) | data[13];
	if (dps310.cal_param.c20 > (powf(2.0f, 15.0f) - 1.0f)) {
		dps310.cal_param.c20 = dps310.cal_param.c20 - powf(2.0f, 16.0f);
	}

	dps310.cal_param.c21 = (data[14] << 8) | data[15];
	if (dps310.cal_param.c21 > (powf(2.0f, 15.0f) - 1.0f)) {
		dps310.cal_param.c21 = dps310.cal_param.c21 - powf(2.0f, 16.0f);
	}

	dps310.cal_param.c30 = (data[16] << 8) | data[17];
	if (dps310.cal_param.c30 > (powf(2.0f, 15.0f) - 1.0f)) {
		dps310.cal_param.c30 = dps310.cal_param.c30 - powf(2.0f, 16.0f);
	}


	/*use some default values if coefficient registers are "empty"*/
	if (dps310.cal_param.c0 == 0 || dps310.cal_param.c1 == 0 || dps310.cal_param.c00 == 0
			|| dps310.cal_param.c10 == 0 || dps310.cal_param.c01 == 0 || dps310.cal_param.c11 == 0
			|| dps310.cal_param.c20 == 0 || dps310.cal_param.c21 == 0 || dps310.cal_param.c30 == 0)
	{
		dps310.cal_param.c00 = 81507;
		dps310.cal_param.c10 = -66011;
		dps310.cal_param.c20 = -13579;
		dps310.cal_param.c30 = -2154;
		dps310.cal_param.c01 = -2115;
		dps310.cal_param.c11 = 1585;
		dps310.cal_param.c21 = 117;
		dps310.cal_param.c0 = 205;
		dps310.cal_param.c1 = -258;
	}

	/* select compensation scale factors*/
	uint8_t P_sampling_buffer;
	i2cRead(BARO_I2C_INSTANCE, DPS310_Address, PRS_CFG,1, &P_sampling_buffer);
	uint8_t P_sampling = P_sampling_buffer << 4;

	switch (P_sampling >> 4)
	{
		case 0: dps310.cal_param.kP = 524288;
		break;
		case 1: dps310.cal_param.kP = 1572864;
		break;
		case 2: dps310.cal_param.kP = 3670016;
		break;
		case 3: dps310.cal_param.kP = 7864320;
		break;
		case 4: dps310.cal_param.kP = 253952;
		break;
		case 5: dps310.cal_param.kP = 516096;
		break;
		case 6: dps310.cal_param.kP = 1040384;
		break;
		case 7: dps310.cal_param.kP = 2088960;
		break;
	}

	uint8_t T_sampling_buffer;
	i2cRead(BARO_I2C_INSTANCE, DPS310_Address, TMP_CFG,1, &T_sampling_buffer);
	uint8_t T_sampling = T_sampling_buffer << 4;
	switch (T_sampling >> 4)
	{
		case 0: dps310.cal_param.kT = 524288;
		break;
		case 1: dps310.cal_param.kT = 1572864;
		break;
		case 2: dps310.cal_param.kT = 3670016;
		break;
		case 3: dps310.cal_param.kT = 7864320;
		break;
		case 4: dps310.cal_param.kT = 253952;
		break;
		case 5: dps310.cal_param.kT = 516096;
		break;
		case 6: dps310.cal_param.kT = 1040384;
		break;
		case 7: dps310.cal_param.kT = 2088960;
		break;
	}

}

#endif /* BARO */
