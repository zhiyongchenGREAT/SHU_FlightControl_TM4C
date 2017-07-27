#ifndef __YRK_AK8975_SOFT_H__
#define __YRK_AK8975_SOFT_H__

#define     AK8975_SOFT_DEVICE          I2C0        //定义AK8975_SOFT 所用的接口 为 I2C0

#define     AK8975_SOFT_ADRESS          (0x18)      /*AK8975_SOFT_Device Address*/

// AK8975_SOFT寄存器地址
//****************************************
#define AK8975_SOFT_WHO_AM_I  0x00
#define AK8975_SOFT_MYSELF    0x48
#define AK8975_SOFT_HXL  0x03
#define AK8975_SOFT_HXH  0x04
#define AK8975_SOFT_HYL  0x05
#define AK8975_SOFT_HYH  0x06
#define AK8975_SOFT_HZL  0x07
#define AK8975_SOFT_HZH  0x08
#define AK8975_SOFT_CNTL 0x0A
#define AK8975_SOFT_ASAX 0x10
#define AK8975_SOFT_ASAY 0x11
#define AK8975_SOFT_ASAZ 0x12
#define	AK8975_SOFT_X	0x04
#define	AK8975_SOFT_Y	0x06
#define	AK8975_SOFT_Z	0x08

//函数声明
extern void  ak8975_soft_init(void);                        //初始化AK8975_SOFT
extern void  ak8975_soft_write_reg(uint8 reg, uint8 Data);  //写AK8975_SOFT寄存器
extern uint8 ak8975_soft_read_reg(uint8 reg);               //读AK8975_SOFT寄存器
extern int16 ak8975_soft_getdata(uint8 REG_Address);
#endif  //__FIRE_AK8975_SOFT_H__