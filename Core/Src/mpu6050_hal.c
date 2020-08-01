
#include "mpu6050_hal.h"

mpu6050_hal_ErrorLed errorLed;
I2C_HandleTypeDef hi2c1;
uint8_t buffer[6] = {0};
accelType accelCalibData;
gyroType gyroCalibData;
float accelScale;
float gyroScale;

void I2C_WriteBuffer(uint8_t i2c_address, uint8_t *aTxBuffer, uint8_t txBufferSize){
	errorLedOff();
	  while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) i2c_address<<1, aTxBuffer, (uint16_t)txBufferSize, (uint32_t)1000)){
		  if(HAL_I2C_GetState(&hi2c1) != HAL_I2C_ERROR_AF){
			  errorLedOn();
		  }
	  }
	  while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}
  }

void I2C_ReadBuffer(uint8_t i2c_address, uint8_t regAddr, uint8_t *aRxBuffer, uint8_t rxBufferSize){
	errorLedOff();
  	  I2C_WriteBuffer(i2c_address, &regAddr, 1);
  	  while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t) i2c_address<<1, aRxBuffer, (uint16_t)rxBufferSize, (uint32_t)1000)){
  	  		  if(HAL_I2C_GetState(&hi2c1) != HAL_I2C_ERROR_AF){
  	  			  errorLedOn();
  	  		  }
  	  	  }
    }

void mpu6050_hal_init(mpu6050_hal_ErrorLed eLed,
						I2C_HandleTypeDef ehi2c1,
						float aScale,
						float gScale){
	HAL_Delay(100);
	errorLed = eLed;
	hi2c1 = ehi2c1;
	accelScale = aScale;
	gyroScale = gScale;

	buffer[0] = MPU6050_PWR_MGMT_1;
	buffer[1] = 0x00;
	I2C_WriteBuffer(MPU6050_I2C_ADDRESS, buffer, 2);

	//Init scale!!!
	buffer[0] = MPU6050_ACCEL_CONFIG;
	if (aScale == MPU6050_ACCEL_2G_SCALE) buffer[1] = MPU6050_ACCEL_CFG_AFS_SEL2;
	if (aScale == MPU6050_ACCEL_4G_SCALE) buffer[1] = MPU6050_ACCEL_CFG_AFS_SEL4;
	if (aScale == MPU6050_ACCEL_8G_SCALE) buffer[1] = MPU6050_ACCEL_CFG_AFS_SEL8;
	if (aScale == MPU6050_ACCEL_16G_SCALE) buffer[1] = MPU6050_ACCEL_CFG_AFS_SEL16;
	I2C_WriteBuffer(MPU6050_I2C_ADDRESS, buffer, 2);

	buffer[0] = MPU6050_GYRO_CONFIG;
	if (aScale == MPU6050_GYRO_250_SCALE) buffer[1] = MPU6050_GYRO_CFG_FS_SEL250;
	if (aScale == MPU6050_GYRO_500_SCALE) buffer[1] = MPU6050_GYRO_CFG_FS_SEL500;
	if (aScale == MPU6050_GYRO_1000_SCALE) buffer[1] = MPU6050_GYRO_CFG_FS_SEL1000;
	if (aScale == MPU6050_GYRO_2000_SCALE) buffer[1] = MPU6050_GYRO_CFG_FS_SEL2000;
	I2C_WriteBuffer(MPU6050_I2C_ADDRESS, buffer, 2);
}

void errorLedOn(void){
	HAL_GPIO_WritePin(errorLed.GPIO_ErrorLedPort, errorLed.GPIO_ErrorLedPin, GPIO_PIN_RESET);
}

void errorLedOff(void){
	HAL_GPIO_WritePin(errorLed.GPIO_ErrorLedPort, errorLed.GPIO_ErrorLedPin, GPIO_PIN_SET);
}


void getAngleAcceleration(accelType *accel){
	I2C_ReadBuffer(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H, buffer, 6);
	float ax = (((int16_t)(buffer[0] << 8)|buffer[1])-accelCalibData.X) / accelScale;
	float ay = (((int16_t)(buffer[2] << 8)|buffer[3])-accelCalibData.Y) / accelScale;
	float az = (((int16_t)(buffer[4] << 8)|buffer[5])-accelCalibData.Z) / accelScale;

	accel->X = (float)(atan(ay/sqrt(ax*ax + az*az))) * 180 / PI;
	accel->Y = (float)(atan(ax/sqrt(ay*ay + az*az))) * 180 / PI;
	accel->Z = 0;
}

void getAngleGyroscope(gyroType *gyro){
	I2C_ReadBuffer(MPU6050_I2C_ADDRESS, MPU6050_GYRO_XOUT_H, buffer, 6);

	gyro->X = (((int16_t)(buffer[0] << 8)|buffer[1]) - gyroCalibData.X) / gyroScale;
	gyro->Y = (((int16_t)(buffer[2] << 8)|buffer[3]) - gyroCalibData.Y) / gyroScale;
	gyro->Z = (((int16_t)(buffer[4] << 8)|buffer[5]) - gyroCalibData.Z) / gyroScale;
}

void calibrationAccel(void){

	uint32_t cycle = 0;
	while (cycle < MPU6050_CALIB_CYCLES){
		I2C_ReadBuffer(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H, buffer, 6);
		accelCalibData.X += (float)((int16_t)(buffer[0] << 8)|buffer[1]);
		accelCalibData.Y += (float)((int16_t)(buffer[2] << 8)|buffer[3]);
		accelCalibData.Z += (float)((int16_t)(buffer[4] << 8)|buffer[5]);
		HAL_Delay(1);
		cycle++;
	}

	accelCalibData.X = accelCalibData.X / MPU6050_CALIB_CYCLES;
	accelCalibData.Y = accelCalibData.Y / MPU6050_CALIB_CYCLES;
	accelCalibData.Z = accelCalibData.Z / MPU6050_CALIB_CYCLES;
	if (accelCalibData.Z < 0) {
		accelCalibData.Z += accelScale;
	} else {
		accelCalibData.Z -= accelScale;
	}
}

void calibrationGyro(void){
	uint32_t cycle = 0;
	while (cycle < MPU6050_CALIB_CYCLES){
		I2C_ReadBuffer(MPU6050_I2C_ADDRESS, MPU6050_GYRO_XOUT_H, buffer, 6);
		gyroCalibData.X += (float)((int16_t)(buffer[0] << 8)|buffer[1]);
		gyroCalibData.Y += (float)((int16_t)(buffer[2] << 8)|buffer[3]);
		gyroCalibData.Z += (float)((int16_t)(buffer[4] << 8)|buffer[5]);
		HAL_Delay(1);
		cycle++;
	}

	gyroCalibData.X = gyroCalibData.X / MPU6050_CALIB_CYCLES;
	gyroCalibData.Y = gyroCalibData.Y / MPU6050_CALIB_CYCLES;
	gyroCalibData.Z = gyroCalibData.Z / MPU6050_CALIB_CYCLES;
}





















