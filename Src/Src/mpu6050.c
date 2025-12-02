#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "mpu6050.h"
#include "cmsis_os.h"

void mpu_init(void) {
	uint8_t check;
	uint8_t data;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 50);

	data = 0;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 50);

	data = 0x07;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 50);

	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 50);

	data = 0x10;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &data, 1, 50);
}

float mpu_accel_read(int ret) {
	uint8_t Rec_Data[6];

	int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;

	float Ax, Ay, Az;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6,
			50);

	Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

	Ax = (float) Accel_X_RAW / 16384.0f;
	Ay = (float) Accel_Y_RAW / 16384.0f;
	Az = (float) Accel_Z_RAW / 16384.0f;

	float Roll = atan2f(Ay, sqrtf(Ax * Ax + Az * Az)) * 180.0f / 3.14159f;
	float Pitch = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az)) * 180.0f / 3.14159f;

	if (ret == 0) {
		return Roll;
	} else if (ret == 1) {
		return -Pitch;
	} else {
		return 0.0;
	}
}

float mpu_roll_pitch_calibration(int rp) {

	int i;
	float roll_calib = 0;
	float pitch_calib = 0;
	float roll_calib_addition = 0;
	float pitch_calib_addition = 0;

	for (i = 0; i <= 1000; i++) {
		roll_calib_addition += mpu_accel_read(0);
		pitch_calib_addition += mpu_accel_read(1);
		osDelay(1);
	}

	roll_calib = roll_calib_addition / 2000;
	pitch_calib = pitch_calib_addition / 2000;


	if (rp == 0)
	{
		return roll_calib;
	}
	else if (rp == 1)
	{
		return pitch_calib;
	}
	else
	{
		return 0.0f;
	}

}


float mpu_gyro_calibration(int axis)
{
    int i;
    float gx_sum = 0;
    float gy_sum = 0;
    float gz_sum = 0;

    for (i = 0; i < 1000; i++)
    {
        gx_sum += mpu_gyro_read(0);
        gy_sum += mpu_gyro_read(1);
        gz_sum += mpu_gyro_read(2);
        osDelay(1);
    }

    float gx_offset = gx_sum / 1000.0f;
    float gy_offset = gy_sum / 1000.0f;
    float gz_offset = gz_sum / 1000.0f;

    if (axis == 0)
        return gx_offset;
    else if (axis == 1)
        return gy_offset;
    else if (axis == 2)
        return gz_offset;
    else
        return 0.0f;
}



float mpu_gyro_read(int ret) {
	uint8_t Rec_Data[6];

	int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;

	float Gx, Gy, Gz;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 50);

	Gyro_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	Gyro_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	Gyro_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

	Gx = (float) Gyro_X_RAW / 131.0f;
	Gy = (float) Gyro_Y_RAW / 131.0f;
	Gz = (float) Gyro_Z_RAW / 131.0f;

	if (ret == 0)
	{
		return Gx;
	}
	else if (ret == 1)
	{
		return Gy;
	}
	else if (ret ==2)
	{
		return Gz;
	}
	else
	{
		return 0.0f;
	}
}
