#ifndef _KS103_H
#define _KS103_H

#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>

//#define PI 3.1416
#define     KS103_SOFT_DEVICE          I2C3        //定义ks103 所用的接口 为 I2C3

#define     KS103_SOFT_ADRESS          (0xE8)      /*KS103Device Default Address*/

extern float ks103_distance,ks103_delta_distance;//KS103测得的距离，单位为mm
extern float fused_height,last_fused_height,delta_fused_height;

// ks103寄存器地址
//****************************************


//函数声明
extern void  KS103_init(void);                        //初始化MPU6050_SOFT
extern void  KS103_soft_write_reg(uint8 reg, uint8 Data);  //写MPU6050_SOFT寄存器
extern uint8 KS103_soft_read_reg(uint8 reg);               //读MPU6050_SOFT寄存器
extern int16 KS103_soft_getdata(uint8 REG_Address);//读两字节数据
extern void KS103_get_distance(void);//获取高度
void ks103_handler(void);



#endif