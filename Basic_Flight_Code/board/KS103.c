#include "KS103.h"



#include "common.h"
#include "data_common.h"
#include <math.h>


#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "stdlib.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
//#include "inc/hw_uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"

#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
//#include "driverlib/uart.h"
//#include "driverlib/udma.h"
//#include "driverlib/pwm.h"
//#include "driverlib/ssi.h"
#include "driverlib/systick.h"


//#include "utils/uartstdio.c"
#include <string.h>


#define    KS103_SDA                  GPIO_PIN_1
#define    KS103_SCL                  GPIO_PIN_0
#define    KS103_SDA_PORT            GPIO_PORTD_BASE
#define    KS103_SCL_PORT            GPIO_PORTD_BASE


#define    KS103_SDA_SET()            HWREG(KS103_SDA_PORT + (GPIO_O_DATA + (KS103_SDA << 2))) = KS103_SDA			//IIC数据引脚定义  
#define    KS103_SDA_RST()            HWREG(KS103_SDA_PORT + (GPIO_O_DATA + (KS103_SDA << 2))) = 0x00			//IIC数据引脚定义  
#define    KS103_SDA_IN()             HWREG(KS103_SDA_PORT + (GPIO_O_DATA + (KS103_SDA << 2)))			//IIC数据引脚定义 
#define    KS103_SCL_SET()            HWREG(KS103_SCL_PORT + (GPIO_O_DATA + (KS103_SCL << 2))) = KS103_SCL			//IIC时钟引脚定义
#define    KS103_SCL_RST()            HWREG(KS103_SCL_PORT + (GPIO_O_DATA + (KS103_SCL << 2))) = 0x00			//IIC时钟引脚定义
#define    KS103_SCL_HIGH()           HWREG(KS103_SCL_PORT + (GPIO_O_DATA + (KS103_SCL << 2)))	//IIC时钟引脚高电平
#define    KS103_Data_O()             HWREG(KS103_SDA_PORT + GPIO_O_DIR) = (HWREG(KS103_SDA_PORT + GPIO_O_DIR) |(KS103_SDA));
#define    KS103_Data_I()             HWREG(KS103_SDA_PORT + GPIO_O_DIR) = (HWREG(KS103_SDA_PORT + GPIO_O_DIR) & ~(KS103_SDA));


float ks103_distance;//KS103测得的距离，单位为mm
float ks103_last_distance,ks103_delta_distance;
float fused_height,last_fused_height,delta_fused_height;
float yyy;

#pragma optimize=none 
void KS103_soft_delay()
{
  DELAY_US(8);
  asm("NOP");asm("NOP");//asm("NOP");asm("NOP");
  
  
}
//**************************************
//KS103_I2C起始信号
//**************************************
#pragma optimize=none 
void KS103_soft_OPEN()
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  // Enable pin PD1 PD0 for GPIOOutput
  
  //
  ROM_GPIOPinTypeGPIOOutputOD(GPIO_PORTD_BASE, GPIO_PIN_1|GPIO_PIN_0);
  
  GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_1|GPIO_PIN_0, GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD_WPU);//GPIO引脚设置为弱上拉模式
  GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_0|GPIO_PIN_1,GPIO_PIN_1|GPIO_PIN_0);
}
#pragma optimize=none 
void KS103_I2C_Start() 
{ 
  KS103_SDA_SET() ;                    //拉高数据线     
  KS103_SCL_SET() ;                    //拉高时钟线     
  KS103_soft_delay();              //延时
  
  KS103_SDA_RST();                    //产生下降沿     
  KS103_soft_delay();                 //延时 
  
  KS103_SCL_RST();                    //拉低时钟线 
} 
//**************************************
//KS103_I2C停止信号
//**************************************
#pragma optimize=none 
void KS103_I2C_Stop()
{
  KS103_SDA_RST();                    //拉低数据线
  KS103_SCL_SET() ;                    //拉高时钟线
  KS103_soft_delay();                 //延时
  
  KS103_SDA_SET() ;                    //产生上升沿
  KS103_soft_delay();                 //延时
  
}
//**************************************
//KS103_I2C发送应答信号
//入口参数:ack (0:ACK 1:NAK)
//**************************************
#pragma optimize=none 
void KS103_I2C_SendACK(_Bool ack)
{
  if(ack)
    KS103_SDA_SET() ;
  else
    KS103_SDA_RST();//写应答信号
  KS103_SCL_SET() ;                    //拉高时钟线
  
  KS103_soft_delay();                 //延时
  KS103_SCL_RST();                    //拉低时钟线
  KS103_soft_delay();                 //延时
  
}
//**************************************
//KS103_I2C接收应答信号
//**************************************
#pragma optimize=none 
uint8 KS103_I2C_RecvACK()
{
  uint8 RF;
  KS103_Data_I();
  KS103_SCL_SET() ;
  
  KS103_Data_I();                    //拉高时钟线
  KS103_soft_delay(); 
  
  //延时
  RF = KS103_SDA_IN();                   //读应答信号
  KS103_SCL_RST();                    //拉低时钟线
  KS103_soft_delay();
  
  KS103_Data_O();                 //延时
  return RF;
}
//**************************************
//向KS103_I2C总线发送一个字节数据
//**************************************
#pragma optimize=none 
void KS103_I2C_SendByte(uint8 dat)
{
  uint8 i;
  //DDRE=0b00001100;
  for (i=0; i<8; i++)         //8位计数器
  {
    if(dat&0x80)
      KS103_SDA_SET() ;
    else
      KS103_SDA_RST();
    dat <<= 1;              //移出数据的最高位
    //送数据口
    KS103_SCL_SET() ;                //拉高时钟线
    KS103_soft_delay();             //延时
    
    KS103_SCL_RST();                //拉低时钟线
    KS103_soft_delay();             //延时
    
  }
  KS103_I2C_RecvACK();
}
//**************************************
//从KS103_I2C总线接收一个字节数据
//**************************************
#pragma optimize=none 
uint8 KS103_I2C_RecvByte()
{
  uint8 i;
  uint8 dat = 0,cy;
  // DDRE=0b00001100;
  KS103_SDA_SET() ;  
  KS103_Data_I();                  //使能内部上拉,准备读取数据,
  for (i=0; i<8; i++)         //8位计数器
  {
    dat <<= 1;
    KS103_SCL_SET() ;
    //DDRE=0b00000100;                 //拉高时钟线
    KS103_soft_delay();             //延时
    
    if(KS103_SDA_IN())
      cy=1;
    else
      cy=0;
    
    dat |= cy;             //读数据  
    
    KS103_SCL_RST();                //拉低时钟线
    KS103_soft_delay();             //延时
    
    
  }
  KS103_Data_O();
  return dat;
}




/*!
*  @brief      KS103_SOFT写寄存器
*  @param      reg         寄存器
*  @param      dat         需要写入的数据的寄存器地址
*  @since      v5.0
*  Sample usage:       KS103_soft_write_reg(KS103_SOFT_XOFFL,0);   // 写寄存器 KS103_SOFT_XOFFL 为 0
*/
void KS103_soft_write_reg(uint8 reg, uint8 Data)
{
  KS103_I2C_Start();                  //起始信号
  KS103_I2C_SendByte(KS103_SOFT_ADRESS);   //发送设备地址+写信号
  KS103_I2C_SendByte(reg);    //内部寄存器地址，
  KS103_I2C_SendByte(Data);       //内部寄存器数据，
  KS103_I2C_Stop();                   //发送停止信号
}

/*!
*  @brief      KS103_SOFT读寄存器
*  @param      reg         寄存器
*  @param      dat         需要读取数据的寄存器地址
*  @since      v5.0
*  Sample usage:       uint8 data = KS103_soft_read_reg(KS103_SOFT_XOFFL);    // 读寄存器 KS103_SOFT_XOFFL
*/
uint8 KS103_soft_read_reg(uint8 reg)
{
  uint8 REG_data;
  KS103_I2C_Start();                   //起始信号
  KS103_I2C_SendByte(KS103_SOFT_ADRESS);    //发送设备地址+写信号
  KS103_I2C_SendByte(reg);     //发送存储单元地址，从0开始	
  KS103_I2C_Start();                   //起始信号
  KS103_I2C_SendByte(KS103_SOFT_ADRESS+1);  //发送设备地址+读信号
  DELAY_US(90);						//根据通信协议必须加，短了数据会错误
  REG_data=KS103_I2C_RecvByte();       //读出寄存器数据
  KS103_I2C_SendACK(1);                //接收应答信号
  KS103_I2C_Stop();                    //停止信号
  return REG_data;
}
int16 KS103_soft_getdata(uint8 REG_Address)
{
  uint8 H,L;
  H=KS103_soft_read_reg(REG_Address);
  L=KS103_soft_read_reg(REG_Address+1);
  return (H<<8)+L;   //合成数据
}


void ks103_handler(void)
{
//  float a,b;
  uint16 temp;
  
  //	GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_0);//清除中断标志位
  //	GPIOIntDisable(GPIO_PORTD_BASE, GPIO_PIN_0);//关中断
  temp=KS103_soft_read_reg(2);
  temp=temp<<8;
  temp=temp+KS103_soft_read_reg(3);
  
  ks103_distance=temp;
/////////////////////////////////////////////
  if(ks103_distance == 0)
  {
    ks103_distance = ks103_last_distance;
  }
/////////////////////////////////////////////  
  ks103_delta_distance = ks103_distance - ks103_last_distance;

  
  ks103_last_distance = ks103_distance;
}

/*!
*  @brief      KS103_SOFT初始化，进入 2g 量程测试模式
*  @since      v5.0
*  Sample usage:            KS103_soft_init();    //初始化 KS103_SOFT
*/

void KS103_init(void)
{
  KS103_soft_OPEN();        //初始化KS103_soft接口
  
  //KS103_soft_write_reg(2,0xC2);
  //                KS103_soft_write_reg(2,0x9C);
  //                KS103_soft_write_reg(2,0x95);
  //                KS103_soft_write_reg(2,0x98);
  //                KS103_soft_write_reg(2,0x73);
  
  /*      MY KS103 Init 2017-5-12*/
//#define KS103_INIT_CONFIGURATION
//#ifdef  KS103_INIT_CONFIGURATION      
//  KS103_soft_write_reg(2,0xC0);
//  DELAY_MS(2000);
//  KS103_soft_write_reg(2,0xC2);
//  DELAY_MS(2000);
//  KS103_soft_write_reg(2,0x75);
//  DELAY_MS(2000);
//#endif
#define KS103_INIT_CONFIGURATION
#ifdef  KS103_INIT_CONFIGURATION      
  KS103_soft_write_reg(2,0x9C);
  KS103_soft_write_reg(2,0x95);
  KS103_soft_write_reg(2,0x98);  
  KS103_soft_write_reg(2,0x74);
  DELAY_MS(5000);
#endif
  ////////////////////////////////////////////////////////////////////////////////
  
  
  
//  DELAY_MS(2000);
  
}

void KS103_get_distance(void)//获取高度
{
#define KS103_GET_DISTANCE_COMMAND 0xb8 //11 meters range
//#define KS103_GET_DISTANCE_COMMAND 0xb0 //5 meters range
  
  KS103_soft_write_reg(2,KS103_GET_DISTANCE_COMMAND);
  
}