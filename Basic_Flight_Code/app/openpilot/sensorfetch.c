/**
 ******************************************************************************
 * @addtogroup TauLabsModules Tau Labs Modules
 * @{
 * @addtogroup CCState Copter Control State Estimation
 * @{
 *
 * @file       sensorfetch.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013
 * @brief      Fetch the sensor data
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 ******************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include "sensorfetch.h"

// Private constants
#define SENSOR_PERIOD     4
#define LOOP_RATE_MS      25.0f
#define GYRO_NEUTRAL_BIAS 1665
#define ACCEL_SCALE  (GRAVITY * 0.004f)

void getattdata()
{
      sensorData.M_X=ak8975_soft_getdata(AK8975_SOFT_X); 
      sensorData.M_Y=ak8975_soft_getdata(AK8975_SOFT_Y); 
      sensorData.M_Z=ak8975_soft_getdata(AK8975_SOFT_Z);
      
      
      ak8975_soft_write_reg(AK8975_SOFT_CNTL, 0x01);   
      
      sensorData.G_X=mpu6050_soft_getdata(MPU6050_SOFT_G_X)/32768.0;
      sensorData.G_Y=mpu6050_soft_getdata(MPU6050_SOFT_G_Y)/32768.0;
      sensorData.G_Z=mpu6050_soft_getdata(MPU6050_SOFT_G_Z)/32768.0;
      
      sensorData.A_X=-mpu6050_soft_getdata(MPU6050_SOFT_A_X); 
      sensorData.A_Y=-mpu6050_soft_getdata(MPU6050_SOFT_A_Y); 
      sensorData.A_Z=-mpu6050_soft_getdata(MPU6050_SOFT_A_Z);
      sensorData.T=mpu6050_soft_getdata(MPU6050_SOFT_T);
}
/**
 * Get an update from the sensors
 * @param[in] attitudeRaw Populate the UAVO instead of saving right here
 * @return 0 if successfull, -1 if not
 */
int8 senfetch(float *prelim_accels, float *prelim_gyros, GlobalAttitudeVariables *glblAtt, GyrosBiasData *gyrosBias, SensorSettingsData *inertialSensorSettings)
{
	//Rotated data from internal gryoscope sensor frame into board sensor frame
	prelim_gyros[0] = sensorData.G_X * inertialSensorSettings->GyroScale[SENSORSETTINGS_GYROSCALE_X];//!!@@
	prelim_gyros[1] = sensorData.G_Y * inertialSensorSettings->GyroScale[SENSORSETTINGS_GYROSCALE_Y];
	prelim_gyros[2] = sensorData.G_Z * inertialSensorSettings->GyroScale[SENSORSETTINGS_GYROSCALE_Z];
	
	// When this is enabled remove estimate of bias
	if (glblAtt->bias_correct_gyro) {
		prelim_gyros[0] -= gyrosBias->x;
		prelim_gyros[1] -= gyrosBias->y;
		prelim_gyros[2] -= gyrosBias->z;
	}
	
	//Rotated data from internal accelerometer sensor frame into board sensor frame
	//Apply scaling and bias correction in sensor frame
	prelim_accels[0] = sensorData.A_X * inertialSensorSettings->AccelScale[0] - inertialSensorSettings->AccelBias[0];//!!@@
	prelim_accels[1] = sensorData.A_Y * inertialSensorSettings->AccelScale[1] - inertialSensorSettings->AccelBias[1];
	prelim_accels[2] = sensorData.A_Z * inertialSensorSettings->AccelScale[2] - inertialSensorSettings->AccelBias[2];

	prelim_gyros[3] = 35.0f + ((float)sensorData.T + 512.0f) / 340.0f;	//Temperature sensor has a 35deg bias. //WHY? AS PER DOCS?
	prelim_accels[3] = 35.0f + ((float)sensorData.T + 512.0f) / 340.0f;


	return 0;
}

/**
 * @}
 * @}
 */
