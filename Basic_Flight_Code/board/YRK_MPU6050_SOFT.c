#include "common.h"

#include "YRK_MPU6050_SOFT.h"


#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"


#define    MPU6050_SDA                  GPIO_PIN_7
#define    MPU6050_SCL                  GPIO_PIN_6
#define    MPU6050_SDA_PORT            GPIO_PORTA_BASE
#define    MPU6050_SCL_PORT            GPIO_PORTA_BASE


#define    MPU6050_SDA_SET()            HWREG(MPU6050_SDA_PORT + (GPIO_O_DATA + (MPU6050_SDA << 2))) = MPU6050_SDA			//IIC数据引脚定义  
#define    MPU6050_SDA_RST()            HWREG(MPU6050_SDA_PORT + (GPIO_O_DATA + (MPU6050_SDA << 2))) = 0x00			//IIC数据引脚定义  
#define    MPU6050_SDA_IN()             HWREG(MPU6050_SDA_PORT + (GPIO_O_DATA + (MPU6050_SDA << 2)))			//IIC数据引脚定义 
#define    MPU6050_SCL_SET()            HWREG(MPU6050_SCL_PORT + (GPIO_O_DATA + (MPU6050_SCL << 2))) = MPU6050_SCL			//IIC时钟引脚定义
#define    MPU6050_SCL_RST()            HWREG(MPU6050_SCL_PORT + (GPIO_O_DATA + (MPU6050_SCL << 2))) = 0x00			//IIC时钟引脚定义
#define    MPU6050_Data_O()             HWREG(MPU6050_SDA_PORT + GPIO_O_DIR) = (HWREG(MPU6050_SDA_PORT + GPIO_O_DIR) |(MPU6050_SDA));
#define    MPU6050_Data_I()             HWREG(MPU6050_SDA_PORT + GPIO_O_DIR) = (HWREG(MPU6050_SDA_PORT + GPIO_O_DIR) & ~(MPU6050_SDA));

#pragma optimize=none 
void mpu6050_soft_delay()
{
 // DELAY_US(1);
  //asm("NOP");asm("NOP");//asm("NOP");asm("NOP");

//mpu6050_soft_delay();
}
//**************************************
//mpu6050_I2C起始信号
//**************************************
#pragma optimize=none 
void mpu6050_soft_OPEN()
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    // Enable pin PA6 for GPIOOutput

    //
    ROM_GPIOPinTypeGPIOOutputOD(GPIO_PORTA_BASE, GPIO_PIN_6|GPIO_PIN_7);
    
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6|GPIO_PIN_7, GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD_WPU);//GPIO引脚设置为弱上拉模式
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6|GPIO_PIN_7,GPIO_PIN_6|GPIO_PIN_7);
}
#pragma optimize=none 
void mpu6050_I2C_Start() 
{ 
    MPU6050_SDA_SET() ;                    //拉高数据线     
    MPU6050_SCL_SET() ;                    //拉高时钟线     
    mpu6050_soft_delay();              //延时
     
    MPU6050_SDA_RST();                    //产生下降沿     
    mpu6050_soft_delay();                 //延时 
      
    MPU6050_SCL_RST();                    //拉低时钟线 
} 
//**************************************
//mpu6050_I2C停止信号
//**************************************
#pragma optimize=none 
void mpu6050_I2C_Stop()
{
    MPU6050_SDA_RST();                    //拉低数据线
    MPU6050_SCL_SET() ;                    //拉高时钟线
   mpu6050_soft_delay();                 //延时
      
    MPU6050_SDA_SET() ;                    //产生上升沿
    mpu6050_soft_delay();                 //延时
      
}
//**************************************
//mpu6050_I2C发送应答信号
//入口参数:ack (0:ACK 1:NAK)
//**************************************
#pragma optimize=none 
void mpu6050_I2C_SendACK(_Bool ack)
{
    if(ack)
    MPU6050_SDA_SET() ;
    else
    MPU6050_SDA_RST();//写应答信号
    MPU6050_SCL_SET() ;                    //拉高时钟线
      
    mpu6050_soft_delay();                 //延时
    MPU6050_SCL_RST();                    //拉低时钟线
    mpu6050_soft_delay();                 //延时
      
}
//**************************************
//mpu6050_I2C接收应答信号
//**************************************
#pragma optimize=none 
uint8 mpu6050_I2C_RecvACK()
{
uint8 RF;
    MPU6050_Data_I();
    MPU6050_SCL_SET() ;
    
    MPU6050_Data_I();                    //拉高时钟线
   mpu6050_soft_delay(); 
      
                    //延时
    RF = MPU6050_SDA_IN();                   //读应答信号
    MPU6050_SCL_RST();                    //拉低时钟线
    mpu6050_soft_delay();
      
    MPU6050_Data_O();                 //延时
    return RF;
}
//**************************************
//向mpu6050_I2C总线发送一个字节数据
//**************************************
#pragma optimize=none 
void mpu6050_I2C_SendByte(uint8 dat)
{
    uint8 i;
    //DDRE=0b00001100;
    for (i=0; i<8; i++)         //8位计数器
    {
        if(dat&0x80)
        MPU6050_SDA_SET() ;
        else
        MPU6050_SDA_RST();
        dat <<= 1;              //移出数据的最高位
                       //送数据口
        MPU6050_SCL_SET() ;                //拉高时钟线
        mpu6050_soft_delay();             //延时
          
        MPU6050_SCL_RST();                //拉低时钟线
        mpu6050_soft_delay();             //延时
          
    }
    mpu6050_I2C_RecvACK();
}
//**************************************
//从mpu6050_I2C总线接收一个字节数据
//**************************************
#pragma optimize=none 
uint8 mpu6050_I2C_RecvByte()
{
    uint8 i;
    uint8 dat = 0,cy;
   // DDRE=0b00001100;
    MPU6050_SDA_SET() ;  
    MPU6050_Data_I();                  //使能内部上拉,准备读取数据,
    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;
        MPU6050_SCL_SET() ;
        //DDRE=0b00000100;                 //拉高时钟线
        mpu6050_soft_delay();             //延时
          
        if(MPU6050_SDA_IN())
        cy=1;
        else
        cy=0;
        
        dat |= cy;             //读数据  
                
        MPU6050_SCL_RST();                //拉低时钟线
       mpu6050_soft_delay();             //延时
          
         
    }
    MPU6050_Data_O();
    return dat;
}




/*!
 *  @brief      MPU6050_SOFT写寄存器
 *  @param      reg         寄存器
 *  @param      dat         需要写入的数据的寄存器地址
 *  @since      v5.0
 *  Sample usage:       mpu6050_soft_write_reg(MPU6050_SOFT_XOFFL,0);   // 写寄存器 MPU6050_SOFT_XOFFL 为 0
 */
void mpu6050_soft_write_reg(uint8 reg, uint8 Data)
{
    mpu6050_I2C_Start();                  //起始信号
    mpu6050_I2C_SendByte(MPU6050_SOFT_ADRESS);   //发送设备地址+写信号
    mpu6050_I2C_SendByte(reg);    //内部寄存器地址，
    mpu6050_I2C_SendByte(Data);       //内部寄存器数据，
    mpu6050_I2C_Stop();                   //发送停止信号
}

/*!
 *  @brief      MPU6050_SOFT读寄存器
 *  @param      reg         寄存器
 *  @param      dat         需要读取数据的寄存器地址
 *  @since      v5.0
 *  Sample usage:       uint8 data = mpu6050_soft_read_reg(MPU6050_SOFT_XOFFL);    // 读寄存器 MPU6050_SOFT_XOFFL
 */
uint8 mpu6050_soft_read_reg(uint8 reg)
{
        uint8 REG_data;
	mpu6050_I2C_Start();                   //起始信号
	mpu6050_I2C_SendByte(MPU6050_SOFT_ADRESS);    //发送设备地址+写信号
	mpu6050_I2C_SendByte(reg);     //发送存储单元地址，从0开始	
	mpu6050_I2C_Start();                   //起始信号
	mpu6050_I2C_SendByte(MPU6050_SOFT_ADRESS+1);  //发送设备地址+读信号
	REG_data=mpu6050_I2C_RecvByte();       //读出寄存器数据
	mpu6050_I2C_SendACK(1);                //接收应答信号
	mpu6050_I2C_Stop();                    //停止信号
	return REG_data;
}
int16 mpu6050_soft_getdata(uint8 REG_Address)
{
	uint8 H,L;
	H=mpu6050_soft_read_reg(REG_Address);
	L=mpu6050_soft_read_reg(REG_Address+1);
	return (H<<8)+L;   //合成数据
}
/*!
 *  @brief      MPU6050_SOFT初始化，进入 2g 量程测试模式
 *  @since      v5.0
 *  Sample usage:            mpu6050_soft_init();    //初始化 MPU6050_SOFT
 */

void mpu6050_soft_init(void)
{
        mpu6050_soft_OPEN();        //初始化mpu6050_soft接口
        mpu6050_soft_write_reg(MPU6050_SOFT_PWR_MGMT_1, 0x80);	//解除休眠状态
        DELAY_MS(100);

	mpu6050_soft_write_reg(MPU6050_SOFT_PWR_MGMT_1, 0x00);	//解除休眠状态
	DELAY_MS(1);
	 
	mpu6050_soft_write_reg(MPU6050_SOFT_SMPLRT_DIV, 0x01);
	DELAY_MS(1);
	 
	mpu6050_soft_write_reg(MPU6050_SOFT_CONFIG, 0x03);
	DELAY_MS(1);
	 
        mpu6050_soft_write_reg(MPU6050_SOFT_USER_CTRL, 0x00);
	DELAY_MS(1);
	 
        mpu6050_soft_write_reg(MPU6050_SOFT_INT_PIN_CFG, 0x02);	
	DELAY_MS(1);
	 
	mpu6050_soft_write_reg(MPU6050_SOFT_GYRO_CONFIG, 0x10);     //Range 2000d/s
	DELAY_MS(1);
	 
	mpu6050_soft_write_reg(MPU6050_SOFT_ACCEL_CONFIG, 0x01);    //Range 2g
	 
        DELAY_MS(500);
}