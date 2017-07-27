#include "common.h"
 
#include "YRK_AK8975_SOFT.h"

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"


#define    AK8975_SDA                  GPIO_PIN_7
#define    AK8975_SCL                  GPIO_PIN_6
#define    AK8975_SDA_POART            GPIO_PORTA_BASE
#define    AK8975_SCL_POART            GPIO_PORTA_BASE


#define    AK8975_SDA_SET()            HWREG(AK8975_SDA_POART + (GPIO_O_DATA + (AK8975_SDA << 2))) = AK8975_SDA			//IIC数据引脚定义  
#define    AK8975_SDA_RST()            HWREG(AK8975_SDA_POART + (GPIO_O_DATA + (AK8975_SDA << 2))) = 0x00			//IIC数据引脚定义  
#define    AK8975_SDA_IN()             HWREG(AK8975_SDA_POART + (GPIO_O_DATA + (AK8975_SDA << 2)))			//IIC数据引脚定义 
#define    AK8975_SCL_SET()            HWREG(AK8975_SCL_POART + (GPIO_O_DATA + (AK8975_SCL << 2))) = AK8975_SCL			//IIC时钟引脚定义
#define    AK8975_SCL_RST()            HWREG(AK8975_SCL_POART + (GPIO_O_DATA + (AK8975_SCL << 2))) = 0x00			//IIC时钟引脚定义
#define    AK8975_Data_O()             HWREG(AK8975_SDA_POART + GPIO_O_DIR) = (HWREG(AK8975_SDA_POART + GPIO_O_DIR) |(AK8975_SDA));
#define    AK8975_Data_I()             HWREG(AK8975_SDA_POART + GPIO_O_DIR) = (HWREG(AK8975_SDA_POART + GPIO_O_DIR) & ~(AK8975_SDA));

#pragma optimize=none
void ak8975_soft_delay()
{
asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
asm("NOP");asm("NOP");

//asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
//asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");

//AK8975_delay();
}
//**************************************
//ak8975_soft_I2C起始信号
//**************************************
#pragma optimize=none
void ak8975_soft_OPEN()
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    // Enable pin PA6 for GPIOOutput

    //
    ROM_GPIOPinTypeGPIOOutputOD(GPIO_PORTA_BASE, GPIO_PIN_6|GPIO_PIN_7);
    
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6|GPIO_PIN_7, GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD_WPU);//GPIO引脚设置为弱上拉模式
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6|GPIO_PIN_7,GPIO_PIN_6|GPIO_PIN_7);
}
#pragma optimize=none
void ak8975_soft_I2C_Start() 
{ 
    AK8975_SDA_SET() ;                    //拉高数据线     
    AK8975_SCL_SET() ;                    //拉高时钟线     
    ak8975_soft_delay();              //延时
     
    AK8975_SDA_RST();                    //产生下降沿     
    ak8975_soft_delay();                 //延时 
      
    AK8975_SCL_RST();                    //拉低时钟线 
} 
//**************************************
//ak8975_soft_I2C停止信号
//**************************************
#pragma optimize=none
void ak8975_soft_I2C_Stop()
{
    AK8975_SDA_RST();                    //拉低数据线
    AK8975_SCL_SET() ;                    //拉高时钟线
    ak8975_soft_delay();                 //延时
      
    AK8975_SDA_SET() ;                    //产生上升沿
    ak8975_soft_delay();                 //延时
      
}
//**************************************
//ak8975_soft_I2C发送应答信号
//入口参数:ack (0:ACK 1:NAK)
//**************************************
#pragma optimize=none
void ak8975_soft_I2C_SendACK(_Bool ack)
{
    if(ack)
    AK8975_SDA_SET() ;
    else
    AK8975_SDA_RST();//写应答信号
    AK8975_SCL_SET() ;                    //拉高时钟线
      
    ak8975_soft_delay();                 //延时
    AK8975_SCL_RST();                    //拉低时钟线
    ak8975_soft_delay();                 //延时
      
}
//**************************************
//ak8975_soft_I2C接收应答信号
//**************************************
#pragma optimize=none
uint8 ak8975_soft_I2C_RecvACK()
{
uint8 RF;
    AK8975_Data_I();
    AK8975_SCL_SET() ;
    
    AK8975_Data_I();                    //拉高时钟线
    ak8975_soft_delay(); 
      
                    //延时
    RF = AK8975_SDA_IN();                   //读应答信号
    AK8975_SCL_RST();                    //拉低时钟线
    asm("NOP");asm("NOP");
      
    AK8975_Data_O();                 //延时
    return RF;
}
//**************************************
//向ak8975_soft_I2C总线发送一个字节数据
//**************************************
#pragma optimize=none
void ak8975_soft_I2C_SendByte(uint8 dat)
{
    uint8 i;
    //DDRE=0b00001100;
    for (i=0; i<8; i++)         //8位计数器
    {
        if(dat&0x80)
        AK8975_SDA_SET() ;
        else
        AK8975_SDA_RST();
        dat <<= 1;              //移出数据的最高位
                       //送数据口
        AK8975_SCL_SET() ;                //拉高时钟线
        ak8975_soft_delay();             //延时
          
        AK8975_SCL_RST();                //拉低时钟线
        ak8975_soft_delay();             //延时
          
    }
    ak8975_soft_I2C_RecvACK();
}
//**************************************
//从ak8975_soft_I2C总线接收一个字节数据
//**************************************
#pragma optimize=none
uint8 ak8975_soft_I2C_RecvByte()
{
    uint8 i;
    uint8 dat = 0,cy;
   // DDRE=0b00001100;
    AK8975_SDA_SET() ;  
    AK8975_Data_I();                  //使能内部上拉,准备读取数据,
    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;
        AK8975_SCL_SET() ;
        //DDRE=0b00000100;                 //拉高时钟线
        ak8975_soft_delay();             //延时
          
        if(AK8975_SDA_IN())
        cy=1;
        else
        cy=0;
        
        dat |= cy;             //读数据  
                
        AK8975_SCL_RST();                //拉低时钟线
        ak8975_soft_delay();             //延时
          
         
    }
    AK8975_Data_O();
    return dat;
}



/*!
 *  @brief      AK8975_SOFT写寄存器
 *  @param      reg         寄存器
 *  @param      dat         需要写入的数据的寄存器地址
 *  @since      v5.0
 *  Sample usage:       ak8975_soft_write_reg(AK8975_SOFT_XOFFL,0);   // 写寄存器 AK8975_SOFT_XOFFL 为 0
 */
void ak8975_soft_write_reg(uint8 reg, uint8 Data)
{
    ak8975_soft_I2C_Start();                  //起始信号
    ak8975_soft_I2C_SendByte(AK8975_SOFT_ADRESS);   //发送设备地址+写信号
    ak8975_soft_I2C_SendByte(reg);    //内部寄存器地址，
    ak8975_soft_I2C_SendByte(Data);       //内部寄存器数据，
    ak8975_soft_I2C_Stop();                   //发送停止信号
}

/*!
 *  @brief      AK8975_SOFT读寄存器
 *  @param      reg         寄存器
 *  @param      dat         需要读取数据的寄存器地址
 *  @since      v5.0
 *  Sample usage:       uint8 data = ak8975_soft_read_reg(AK8975_SOFT_XOFFL);    // 读寄存器 AK8975_SOFT_XOFFL
 */
uint8 ak8975_soft_read_reg(uint8 reg)
{
        uint8 REG_data;
	ak8975_soft_I2C_Start();                   //起始信号
	ak8975_soft_I2C_SendByte(AK8975_SOFT_ADRESS);    //发送设备地址+写信号
	ak8975_soft_I2C_SendByte(reg);     //发送存储单元地址，从0开始	
	ak8975_soft_I2C_Start();                   //起始信号
	ak8975_soft_I2C_SendByte(AK8975_SOFT_ADRESS+1);  //发送设备地址+读信号
	REG_data=ak8975_soft_I2C_RecvByte();       //读出寄存器数据
	ak8975_soft_I2C_SendACK(1);                //接收应答信号
	ak8975_soft_I2C_Stop();                    //停止信号
	return REG_data;
}
int16 ak8975_soft_getdata(uint8 REG_Address)
{
	uint8 H,L;
	H=ak8975_soft_read_reg(REG_Address);
	L=ak8975_soft_read_reg(REG_Address-1);
	return (H<<8)+L;   //合成数据
}
/*!
 *  @brief      AK8975_SOFT初始化，进入 2g 量程测试模式
 *  @since      v5.0
 *  Sample usage:            ak8975_soft_init();    //初始化 AK8975_SOFT
 */

void ak8975_soft_init(void)
{
        ak8975_soft_OPEN();        //初始化ak8975_soft接口
	 
	ak8975_soft_write_reg(AK8975_SOFT_CNTL, 0x0f);    //Range 2g
	 
        DELAY_MS(500);
}