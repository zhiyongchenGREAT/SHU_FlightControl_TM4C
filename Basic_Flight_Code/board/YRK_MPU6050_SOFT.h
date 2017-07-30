#ifndef __YRK_MPU6050_SOFT_H__
#define __YRK_MPU6050_SOFT_H__

#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>

#define     MPU6050_SOFT_DEVICE          I2C0        //定义MPU6050_SOFT 所用的接口 为 I2C0

#define     MPU6050_SOFT_ADRESS          (0xd0)      /*MPU6050_SOFT_Device Address*/

// MPU6050_SOFT寄存器地址
//****************************************
#define	MPU6050_SOFT_SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	MPU6050_SOFT_CONFIG		   	0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	MPU6050_SOFT_GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	MPU6050_SOFT_ACCEL_CONFIG            0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define MPU6050_SOFT_I2C_MST_CTRL            0x24
#define MPU6050_SOFT_I2C_SLV0_ADDR           0x25
#define MPU6050_SOFT_I2C_SLV0_REG            0x26
#define MPU6050_SOFT_I2C_SLV0_CTRL           0x27
#define MPU6050_SOFT_INT_PIN_CFG             0x37
#define	MPU6050_SOFT_ACCEL_XOUT_H	        0x3B
#define	MPU6050_SOFT_ACCEL_XOUT_L	        0x3C
#define	MPU6050_SOFT_ACCEL_YOUT_H	        0x3D
#define	MPU6050_SOFT_ACCEL_YOUT_L	        0x3E
#define	MPU6050_SOFT_ACCEL_ZOUT_H	        0x3F
#define	MPU6050_SOFT_ACCEL_ZOUT_L	        0x40
#define	MPU6050_SOFT_TEMP_OUT_H		0x41
#define	MPU6050_SOFT_TEMP_OUT_L		0x42
#define	MPU6050_SOFT_GYRO_XOUT_H		0x43
#define	MPU6050_SOFT_GYRO_XOUT_L		0x44	
#define	MPU6050_SOFT_GYRO_YOUT_H		0x45
#define	MPU6050_SOFT_GYRO_YOUT_L		0x46
#define	MPU6050_SOFT_GYRO_ZOUT_H		0x47
#define	MPU6050_SOFT_GYRO_ZOUT_L		0x48
#define MPU6050_SOFT_USER_CTRL 		0x6A
#define	MPU6050_SOFT_PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	MPU6050_SOFT_WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68，只读)
#define	MPU6050_SOFT_MYSELF		0x68	//IIC地址寄存器(默认数值0x68，只读)
#define	MPU6050_SOFT_A_X	0x3B
#define	MPU6050_SOFT_A_Y	0x3D
#define	MPU6050_SOFT_A_Z	0x3F
#define	MPU6050_SOFT_T	0x41
#define	MPU6050_SOFT_G_X	0x43	
#define	MPU6050_SOFT_G_Y	0x45
#define	MPU6050_SOFT_G_Z	0x47

//函数声明
extern void  mpu6050_soft_init(void);                        //初始化MPU6050_SOFT
extern void  mpu6050_soft_write_reg(uint8 reg, uint8 Data);  //写MPU6050_SOFT寄存器
extern uint8 mpu6050_soft_read_reg(uint8 reg);               //读MPU6050_SOFT寄存器
extern int16 mpu6050_soft_getdata(uint8 REG_Address);
#endif  //__FIRE_MPU6050_SOFT_H__