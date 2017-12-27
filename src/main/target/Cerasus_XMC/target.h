#pragma once

#define TARGET_BOARD_IDENTIFIER "CER" // MCI Cerasus

#define LED0					P110
#define LED0_INVERTED
#define LED1					P111
#define LED1_INVERTED
#define LED2					P112
#define LED2_INVERTED

#define SERIAL_PORT_COUNT       3

#define USE_VCP
#define USE_UART1
#define USE_UART2

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE 				(I2CDEV_1)

#define USE_SPI
#define USE_SPI_DEVICE_1

#define GYRO
#define USE_GYRO_MPU6500

#define ACC
#define USE_ACC_MPU6500

#define MAG
#define USE_MPU9250_MAG
#define USE_MAG_AK8963

#define USE_SERIALRX_JETIEXBUS
#define USE_RX_MSP

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORT0         0xffff
#define TARGET_IO_PORT1         0xffff
#define TARGET_IO_PORT2         0xffff
#define TARGET_IO_PORT3         0xffff
#define TARGET_IO_PORT4         0xffff
#define TARGET_IO_PORT5         0xffff
#define TARGET_IO_PORT6         0xffff
#define TARGET_IO_PORT14        0xffff
#define TARGET_IO_PORT15        0xffff

#define USABLE_TIMER_CHANNEL_COUNT 8
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4))
