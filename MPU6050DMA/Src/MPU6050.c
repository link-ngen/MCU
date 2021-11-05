/*
 * MPU6050.c
 *
 *  Created on: 15.06.2020
 *      Author: nguyen
 */

#include <math.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_dma.h"
#include "MPU6050.h"

MPU6050_ErrorOffset_t _error_offset;
//uint32_t _last_time;

MPU6050_StateTypeDef MPU6050_Init(MPU6050_t *data_struct,
		MPU6050_InitAccScale accSensitivity,
		MPU6050_InitGyrosScale gyroSensitivity)
{
	// Check if device is connected
	if (MPU6050_get_device_address() != MPU6050_I_AM)
	{
		return MPU6050_DEV_INVALID;
	}
	data_struct->DevAddress = MPU6050_ADDR;

	// reset the whole module first
	uint8_t cmd = PWR1_DEVICE_RESET_BIT;
	HAL_I2C_Mem_Write(&MPU6050_I2C_PORT, MPU6050_ADDR, REG_PWR_MGMT_1,
	I2C_MEMADD_SIZE_8BIT, &cmd, 1, I2C_TIMEOUT);
	HAL_Delay(100);	//wait for 100ms for the gyro to stable

	// Wakeup MPU6050
	cmd = ZEROS;
	HAL_I2C_Mem_Write(&MPU6050_I2C_PORT, MPU6050_ADDR, REG_PWR_MGMT_1,
	I2C_MEMADD_SIZE_8BIT, &cmd, 1, I2C_TIMEOUT);

	//DLPF_CFG = 1: Fs=1khz; bandwidth=42hz
	cmd = 0x01;
	HAL_I2C_Mem_Write(&MPU6050_I2C_PORT, MPU6050_ADDR, REG_CONFIG,
	I2C_MEMADD_SIZE_8BIT, &cmd, 1, I2C_TIMEOUT);
	HAL_I2C_Mem_Write(&MPU6050_I2C_PORT, MPU6050_ADDR, REG_SMPRT_DIV,
	I2C_MEMADD_SIZE_8BIT, &cmd, 1, I2C_TIMEOUT);

	// Config accelerometer
	HAL_I2C_Mem_Read(&MPU6050_I2C_PORT, MPU6050_ADDR, REG_ACCEL_CONFIG,
	I2C_MEMADD_SIZE_8BIT, &cmd, 1, I2C_TIMEOUT);
	cmd = (cmd & 0xE7) | ((uint8_t) accSensitivity << 3);
	HAL_I2C_Mem_Write(&MPU6050_I2C_PORT, MPU6050_ADDR, REG_ACCEL_CONFIG,
	I2C_MEMADD_SIZE_8BIT, &cmd, 1, I2C_TIMEOUT);

	// Config gyroscope
	HAL_I2C_Mem_Read(&MPU6050_I2C_PORT, MPU6050_ADDR, REG_GYRO_CONFIG,
	I2C_MEMADD_SIZE_8BIT, &cmd, 1, I2C_TIMEOUT);
	cmd = (cmd & 0xE7) | ((uint8_t) gyroSensitivity << 3);
	HAL_I2C_Mem_Write(&MPU6050_I2C_PORT, MPU6050_ADDR, REG_GYRO_CONFIG,
	I2C_MEMADD_SIZE_8BIT, &cmd, 1, I2C_TIMEOUT);

	// Set sensitivities for multiplying gyro and accelerometer data
	switch (accSensitivity)
	{
	case MPU6050_AFS_SEL_2G:
		data_struct->AcceScale = 1.0f / MPU6050_ACCE_SENS_2G;
		break;
	case MPU6050_AFS_SEL_4G:
		data_struct->AcceScale = 1.0f / MPU6050_ACCE_SENS_4G;
		break;
	case MPU6050_AFS_SEL_8G:
		data_struct->AcceScale = 1.0f / MPU6050_ACCE_SENS_8G;
		break;
	case MPU6050_AFS_SEL_16G:
		data_struct->AcceScale = 1.0f / MPU6050_ACCE_SENS_16G;
		break;
	default:
		break;
	}

	switch (gyroSensitivity)
	{
	case MPU6050_FS_SEL_250:
		data_struct->GyroScale = 1.0f / MPU6050_GYRO_SENS_250;
		break;
	case MPU6050_FS_SEL_500:
		data_struct->GyroScale = 1.0f / MPU6050_GYRO_SENS_500;
		break;
	case MPU6050_FS_SEL_1000:
		data_struct->GyroScale = 1.0f / MPU6050_GYRO_SENS_1000;
		break;
	case MPU6050_FS_SEL_2000:
		data_struct->GyroScale = 1.0f / MPU6050_GYRO_SENS_2000;
		break;
	default:
		break;
	}

	// reset gyro and accel sensor
	cmd = 0x07;
	HAL_I2C_Mem_Write(&MPU6050_I2C_PORT, MPU6050_ADDR, REG_SIGNAL_PATH_RESET,
	I2C_MEMADD_SIZE_8BIT, &cmd, 1, I2C_TIMEOUT);

	MPU6050_calculate_error_offset(data_struct);

	return MPU6050_OK;
}

void MPU6050_enableInterrupts(void)
{
	/* Enable interrupts for data ready and motion detect */
	uint8_t cmd = 0x21;
	HAL_I2C_Mem_Write(&MPU6050_I2C_PORT, MPU6050_ADDR, REG_INT_ENABLE,
	I2C_MEMADD_SIZE_8BIT, &cmd, 1, I2C_TIMEOUT);

	/* Clear IRQ flag on any read operation */
	HAL_I2C_Mem_Read(&MPU6050_I2C_PORT, MPU6050_ADDR, REG_INT_PIN_CFG,
	I2C_MEMADD_SIZE_8BIT, &cmd, 1, I2C_TIMEOUT);
	cmd |= (1 << MPU6050_INTCFG_INT_RD_CLEAR_BIT);
	HAL_I2C_Mem_Write(&MPU6050_I2C_PORT, MPU6050_ADDR, REG_INT_PIN_CFG,
	I2C_MEMADD_SIZE_8BIT, &cmd, 1, I2C_TIMEOUT);
}

uint8_t MPU6050_get_device_address(void)
{
	uint8_t data;
	HAL_I2C_Mem_Read(&MPU6050_I2C_PORT, MPU6050_ADDR, REG_WHO_AM_I,
	I2C_MEMADD_SIZE_8BIT, &data, 1, I2C_TIMEOUT);
	return data;
}

void MPU6050_read_temperature(MPU6050_t *data_struct)
{
	uint8_t data[2];
	HAL_I2C_Mem_Read(&MPU6050_I2C_PORT, MPU6050_ADDR, REG_TEMP_OUT_H,
	I2C_MEMADD_SIZE_8BIT, data, 2, I2C_TIMEOUT);
	int16_t res = (((int16_t) data[0]) << 8) | data[1];
	data_struct->Temperature = (float) res / 340.0f + 36.53f;
}

void MPU6050_readRAW_acceleration(MPU6050_t *data_struct)
{
	uint8_t data[6];
	HAL_I2C_Mem_Read(&MPU6050_I2C_PORT, MPU6050_ADDR, REG_ACCEL_XOUT_H,
	I2C_MEMADD_SIZE_8BIT, data, 6, I2C_TIMEOUT);
	data_struct->RawAccX = (((int16_t) data[0]) << 8) | data[1];
	data_struct->RawAccY = (((int16_t) data[2]) << 8) | data[3];
	data_struct->RawAccZ = (((int16_t) data[4]) << 8) | data[5];
}

void MPU6050_read_scaled_acceleration(MPU6050_t *data_struct)
{
	MPU6050_readRAW_acceleration(data_struct);
	data_struct->RawAccX = (float) data_struct->RawAccX
			* data_struct->AcceScale;
	data_struct->RawAccY = (float) data_struct->RawAccY
			* data_struct->AcceScale;
	data_struct->RawAccZ = (float) data_struct->RawAccZ
			* data_struct->AcceScale;
}

void MPU6050_readRAW_rotation(MPU6050_t *data_struct)
{
	uint8_t data[6];
	HAL_I2C_Mem_Read(&MPU6050_I2C_PORT, MPU6050_ADDR, REG_GYRO_XOUT_H,
	I2C_MEMADD_SIZE_8BIT, data, 6, I2C_TIMEOUT);
	data_struct->RawGyroX = (((int16_t) data[0]) << 8) | data[1];
	data_struct->RawGyroY = (((int16_t) data[2]) << 8) | data[3];
	data_struct->RawGyroZ = (((int16_t) data[4]) << 8) | data[5];
}

void MPU6050_read_scaled_rotation(MPU6050_t *data_struct)
{
	MPU6050_readRAW_rotation(data_struct);
	data_struct->RawGyroX = (float) data_struct->RawGyroX
			* data_struct->GyroScale;
	data_struct->RawGyroY = (float) data_struct->RawGyroY
			* data_struct->GyroScale;
	data_struct->RawGyroZ = (float) data_struct->RawGyroZ
			* data_struct->GyroScale;
}

void MPU6050_read_raw_data(MPU6050_t *data_struct)
{
	uint8_t data[14];
	int16_t temp;

	HAL_I2C_Mem_Read(&MPU6050_I2C_PORT, MPU6050_ADDR, REG_ACCEL_XOUT_H,
	I2C_MEMADD_SIZE_8BIT, data, 14, I2C_TIMEOUT);
	data_struct->RawAccX = (((int16_t) data[0]) << 8) | data[1];
	data_struct->RawAccY = (((int16_t) data[2]) << 8) | data[3];
	data_struct->RawAccZ = (((int16_t) data[4]) << 8) | data[5];

	temp = (((int16_t) data[6]) << 8) | data[7];
	data_struct->Temperature = (float) temp / 340.0f + 36.53f;

	data_struct->RawGyroX = (((int16_t) data[8]) << 8) | data[9];
	data_struct->RawGyroY = (((int16_t) data[10]) << 8) | data[11];
	data_struct->RawGyroZ = (((int16_t) data[12]) << 8) | data[13];
}

void MPU6050_readAll_DMA(MPU6050_t *data_struct, uint8_t *buffer)
{
	int16_t temp;
	HAL_I2C_Mem_Read_DMA(&hi2c2, MPU6050_ADDR, REG_ACCEL_XOUT_H,
	I2C_MEMADD_SIZE_8BIT, buffer, 14);

	data_struct->RawAccX = (((int16_t) buffer[0]) << 8) | buffer[1];
	data_struct->RawAccY = (((int16_t) buffer[2]) << 8) | buffer[3];
	data_struct->RawAccZ = (((int16_t) buffer[4]) << 8) | buffer[5];

	temp = (((int16_t) buffer[6]) << 8) | buffer[7];
	data_struct->Temperature = (float) temp / 340.0f + 36.53f;

	data_struct->RawGyroX = (((int16_t) buffer[8]) << 8) | buffer[9];
	data_struct->RawGyroY = (((int16_t) buffer[10]) << 8) | buffer[11];
	data_struct->RawGyroZ = (((int16_t) buffer[12]) << 8) | buffer[13];

	data_struct->AccX = (float) data_struct->RawAccX * data_struct->AcceScale;
	data_struct->AccY = (float) data_struct->RawAccY * data_struct->AcceScale;
	data_struct->AccZ = (float) data_struct->RawAccZ * data_struct->AcceScale;

	data_struct->Roll = (atan2f(data_struct->AccY, data_struct->AccZ)
			* RAD_TO_DEG) - _error_offset.AccErrorOffsetX;
	data_struct->Pitch = (atan2f(-1 * data_struct->AccX,
			sqrtf(data_struct->AccY * data_struct->AccY
							+ data_struct->AccZ * data_struct->AccZ))
							* RAD_TO_DEG) - _error_offset.AccErrorOffsetY;
}

void MPU6050_read_scaled_data(MPU6050_t *data_struct)
{
	MPU6050_read_raw_data(data_struct);

	data_struct->AccX = (float) data_struct->RawAccX * data_struct->AcceScale;
	data_struct->AccY = (float) data_struct->RawAccY * data_struct->AcceScale;
	data_struct->AccZ = (float) data_struct->RawAccZ * data_struct->AcceScale;

	data_struct->GyroX =
			((float) data_struct->RawGyroX * data_struct->GyroScale)
					- _error_offset.GyroErrorOffsetX;
	data_struct->GyroY =
			((float) data_struct->RawGyroY * data_struct->GyroScale)
					- _error_offset.GyroErrorOffsetY;
	data_struct->GyroZ =
			((float) data_struct->RawGyroZ * data_struct->GyroScale)
					- _error_offset.GyroErrorOffsetZ;
}

void MPU6050_read_rpy(MPU6050_t *data_struct)
{
	MPU6050_read_scaled_data(data_struct);
	data_struct->Roll = (atan2f(data_struct->AccY, data_struct->AccZ)
			* RAD_TO_DEG) - _error_offset.AccErrorOffsetX;
	data_struct->Pitch = (atan2f(-1 * data_struct->AccX,
			sqrtf(
					data_struct->AccY * data_struct->AccY
							+ data_struct->AccZ * data_struct->AccZ))
			* RAD_TO_DEG) - _error_offset.AccErrorOffsetY;

	//float elapsed_time = (float)(HAL_GetTick() - _last_time) / 1000.0f;	// in second
	//_last_time = HAL_GetTick();
}

uint8_t MPU6050_getIntStatusRegister(void)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(&MPU6050_I2C_PORT, MPU6050_ADDR, MPU6050_RA_INT_STATUS,
	I2C_MEMADD_SIZE_8BIT, &tmp, 2, I2C_TIMEOUT);
	return tmp;
}

static void MPU6050_calculate_error_offset(MPU6050_t *data_struct)
{
	const uint8_t N = 200;
	for (uint8_t i = 0; i < N; ++i)
	{
		MPU6050_read_scaled_data(data_struct);

		// sum all readings
		_error_offset.AccErrorOffsetX += atan2f(data_struct->AccY,
				data_struct->AccZ) * RAD_TO_DEG;
		_error_offset.AccErrorOffsetY += atan2f(-1 * data_struct->AccX,
				sqrtf(
						data_struct->AccY * data_struct->AccY
								+ data_struct->AccZ
										* data_struct->AccZ)) * RAD_TO_DEG;

		_error_offset.GyroErrorOffsetX += data_struct->GyroX;
		_error_offset.GyroErrorOffsetY += data_struct->GyroY;
		_error_offset.GyroErrorOffsetZ += data_struct->GyroZ;
	}

	// divide the sum by 200 to get the error offset value
	_error_offset.AccErrorOffsetX /= N;
	_error_offset.AccErrorOffsetY /= N;
	_error_offset.GyroErrorOffsetX /= N;
	_error_offset.GyroErrorOffsetY /= N;
	_error_offset.GyroErrorOffsetZ /= N;
}
