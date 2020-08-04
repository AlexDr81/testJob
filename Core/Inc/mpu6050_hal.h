#ifndef __MPU6050_HAL_H
#define __MPU6050_HAL_H

#include <math.h>
#include "stm32f1xx_hal.h"

#define PI (3.14159265F)

#define MPU6050_GYRO_CONFIG           0x1B
#define MPU6050_ACCEL_CONFIG          0x1C
#define MPU6050_ACCEL_XOUT_H          0x3B
#define MPU6050_TEMP_OUT_H            0x41
#define MPU6050_GYRO_XOUT_H           0x43
#define MPU6050_I2C_ADDRESS           0x68
#define MPU6050_PWR_MGMT_1            0x6B

#define MPU6050_GYRO_CFG_FS_SEL250    0x00
#define MPU6050_GYRO_CFG_FS_SEL500    0x08
#define MPU6050_GYRO_CFG_FS_SEL1000   0x10
#define MPU6050_GYRO_CFG_FS_SEL2000   0x18

#define MPU6050_ACCEL_CFG_AFS_SEL2    0x00
#define MPU6050_ACCEL_CFG_AFS_SEL4    0x08
#define MPU6050_ACCEL_CFG_AFS_SEL8    0x10
#define MPU6050_ACCEL_CFG_AFS_SEL16   0x18

#define MPU6050_CALIB_CYCLES          200

#define MPU6050_ACCEL_2G_SCALE        16384
#define MPU6050_ACCEL_4G_SCALE        8192
#define MPU6050_ACCEL_8G_SCALE        4096
#define MPU6050_ACCEL_16G_SCALE       2048

#define MPU6050_GYRO_250_SCALE        131
#define MPU6050_GYRO_500_SCALE        65.5
#define MPU6050_GYRO_1000_SCALE       32.8
#define MPU6050_GYRO_2000_SCALE       16.4


typedef struct{
	GPIO_TypeDef *GPIO_ErrorLedPort;
	uint16_t     GPIO_ErrorLedPin;
} MPU6050_HAL_ERRORLED_t;

typedef struct{
	float X;
	float Y;
	float Z;
} MPU6050_HAL_ACCEL_t;

typedef struct{
	float X;
	float Y;
	float Z;
} MPU6050_HAL_GYRO_t;

void I2C_ReadBuffer(uint8_t i2c_address, uint8_t regAddr, uint8_t *aRxBuffer, uint8_t rxBufferSize);
void I2C_WriteBuffer(uint8_t i2c_address, uint8_t *aTxBuffer, uint8_t txBufferSize);

void mpu6050_hal_init(MPU6050_HAL_ERRORLED_t *led,
						I2C_HandleTypeDef *phi2c1,
						float aScale,
						float gScale);

void errorLedOn(void);
void errorLedOff(void);

void getAngleAcceleration(MPU6050_HAL_ACCEL_t *accel);
void getAngleGyroscope(MPU6050_HAL_GYRO_t *gyro);

void calibrationAccel(void);
void calibrationGyro(void);

#endif /*__MPU6050_HAL_H*/

