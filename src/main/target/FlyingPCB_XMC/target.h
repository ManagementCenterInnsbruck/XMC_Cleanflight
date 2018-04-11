#pragma once

#define TARGET_BOARD_IDENTIFIER "FPCB"

#define LED0					P511
#define LED0_INVERTED
#define LED1					P65
#define LED1_INVERTED
#define LED2					P012
#define LED2_INVERTED

#define SERIAL_PORT_COUNT       1

#define USE_UART1

#define USE_RX_MSP

#define USE_I2C
#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define I2C_DEVICE 				(I2CDEV_1)
#define BARO_I2C_INSTANCE 		(I2CDEV_2)

#define I2C2_SCL P30
#define I2C2_SDA P313

#define GYRO
#define USE_GYRO_MPU6500

#define ACC
#define USE_ACC_MPU6500

#define MAG
#define USE_MPU9250_MAG
#define USE_MAG_AK8963

#define BARO
#define USE_BARO_DPS310
#define USE_BARO_DPS310_BACKGROUND

#define USE_SERIALRX_JETIEXBUS
#define USE_RX_MSP

#define USE_ADC
#define VBAT_ADC_PIN            P157
#define VBAT_SCALE_DEFAULT 		80

#define USE_ONBOARD_ESC

#define TARGET_IO_PORT0         0xffff
#define TARGET_IO_PORT1         0xffff
#define TARGET_IO_PORT2         0xffff
#define TARGET_IO_PORT3         0xffff
#define TARGET_IO_PORT4         0xffff
#define TARGET_IO_PORT5         0xffff
#define TARGET_IO_PORT6         0xffff
#define TARGET_IO_PORT14        0xffff
#define TARGET_IO_PORT15        0xffff

#define USABLE_TIMER_CHANNEL_COUNT 32
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4))
