#include <stdbool.h>
#include <stdint.h>
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "Simulation.h"
#include "math.h"
#include "common.h"
#include "flight_routine.h"
#include "data_common.h"
#include "YRK_SONAR.h"

#define pos_x_i_max 5000
#define pos_mov_i_max 800
#define angel_out_max 10.0f

float last_roll = 0;
float last_pitch = 0;
float change_x, change_y;
float radians_to_pixels_x = 2.5435f,radians_to_pixels_y = 2.5435f;
float conv_factor = 0.008f; 
float High_Now;


int8_t  x=0;
int8_t  y=0;
//float  SumX;
//float SumY;
//float  SumX1;
//float SumY1;
 
typedef enum {
	Mode_One,					 //CPOL=0,CPHA=0
	Mode_Two,					//CPOL=0,CPHA=1
	Mode_Three,				        //CPOL=1,CPHA=0
	Mode_Four				        //CPOL=1,CPHA=1
}SPI_Mode;	

SPI_Mode SPI_Ope_Mode;					//SPI Mode

void Sim_SPI_GPIO_Init(uint8 CPOL,uint8 CPHA,uint32 Speed)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);  
    SysCtlDelay(3);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6|GPIO_PIN_7);
     GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_6|GPIO_PIN_7, GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD_WPU);//GPIO引脚设置为弱上拉模式
    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);  
    SysCtlDelay(3);
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_5);
    
    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); 
    SysCtlDelay(3);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);
     GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD_WPU);//GPIO引脚设置为弱上拉模式
    
    if(CPOL == 0)
	{
		if(CPHA == 0)
			SPI_Ope_Mode = Mode_One;
		else
			SPI_Ope_Mode = Mode_Two;
	}
	else
	{
		if(CPHA == 0)
			SPI_Ope_Mode = Mode_Three;
		else
			SPI_Ope_Mode = Mode_Four;
	}

}

void ADNS3080_Init(void)
{
  uint8 temp_spi;
  CLR_SPI_NCS;
 // SysCtlDelay(SysCtlClockGet()/150);                    //实际上测出为20ms
  delay(67000);
  // set frame rate to manual
  temp_spi = SPI_Read(ADNS3080_EXTENDED_CONFIG);
  temp_spi = (temp_spi & ~0x01) | 0x01;
  delay(67000);                                           //实际上测出为20ms
  SPI_Write(ADNS3080_EXTENDED_CONFIG,temp_spi);
  delay(67000);                                     //实际上测出为20ms
  temp_spi=SPI_Read(ADNS3080_EXTENDED_CONFIG);
  // set frame period to 12000 (0x2EE0)	
  SPI_Write(ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER,0xE0);
  delay(67000);                                           //实际上测出为20ms
  SPI_Write(ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER,0x2E);
  delay(67000);                                         //实际上测出为20ms
  // set 1600 resolution bit
  temp_spi = SPI_Read(ADNS3080_CONFIGURATION_BITS);
  temp_spi |= 0x10;
 delay(67000);                                          //实际上测出为20ms
  SPI_Write(ADNS3080_CONFIGURATION_BITS,temp_spi);	
  delay(67000);                                     //实际上测出为20ms
  SET_SPI_NCS;
  
}

uint8 SPI_Basic_RW(uint8 data_write)
{
  uint8 count_spi;
  uint8 data_read;
  switch(SPI_Ope_Mode)
  {
  case Mode_One:											//???Dμ????aμíμ???
    if(data_write & 0x80)							//CSNò?à-μíá????í?aê?D′3?êy?Y￡?è?oó?úμúò???é?éy??á?±??aê?2é?ùêy?Y￡??ú
      SET_SPI_MOSI;
    else
      CLR_SPI_MOSI;
    for(count_spi = 0;count_spi < 8;count_spi ++)
    {
      delay(28);
      SET_SPI_SCL;										//é?éy????oó?íé??áè?êy?Y
      if(READ_MISO_DATA)						                 
        data_read |= 0x01;
      if(count_spi<7)											//??Dèòa×óò?7′?
      {
        data_write = data_write << 1; 
        data_read = data_read << 1; 
      }
      delay(28);
      
      CLR_SPI_SCL;										//???μ????oó?íé?D′3?êy?Y
      
      if(data_write & 0x80)						//D′3?μ?μú????ê??TD§μ?￡?′óéè±?ò2??óDé?éy??à′?áè?￡?CSNá￠?í±?à-??￡?ò????ù±??áD′2ù×÷ò??-íê3é
       SET_SPI_MOSI;
      else
        CLR_SPI_MOSI;
    }
    break;
  case Mode_Two:											//???Dμ????aμíμ???
    for(count_spi = 0;count_spi < 8;count_spi ++)
    {
      SET_SPI_SCL;										//é?éy????oóê?3?êy?Y
      
      if(data_write & 0x80)
        SET_SPI_MOSI;
      else
        CLR_SPI_MOSI;				
      delay(28);
      CLR_SPI_SCL;										//???μ????oó?áè?êy?Y				
      if(READ_MISO_DATA)						                
        data_read |= 0x01;
      if(count_spi<7)											//??Dèòa×óò?7′?
      {
        data_write = data_write << 1; 
        data_read = data_read << 1;
	
      }
      delay(28);
    }
    break;	
  case Mode_Three:										//???Dμ????a??μ???
    if(data_write & 0x80)
      SET_SPI_MOSI;
    else
      CLR_SPI_MOSI;	
    for(count_spi = 0;count_spi < 8;count_spi ++)
    {
      delay(28);
      CLR_SPI_SCL;										//???μ???áêy?Y	
				
      if(READ_MISO_DATA)						                 
        data_read |= 0x01;				
      if(count_spi<7)											//??Dèòa×óò?7′?
      {
        data_write = data_write << 1; 
        data_read = data_read << 1; 
				
      }
      delay(28);
      SET_SPI_SCL;										//é?éy??D′êy?Y				
      if(data_write & 0x80)						//bit9D′è?μ?êy?Y?TD§￡?ò2??óD???μ??à′è????áè?￡?CSN±?à-??￡?ò????ù±??áD′2ù×÷ò??-íê3é
        SET_SPI_MOSI;
      else
        CLR_SPI_MOSI;								
    }
    break;
  case Mode_Four:											//???Dμ????a??μ???
    for(count_spi = 0;count_spi < 8;count_spi ++)
    {
      CLR_SPI_SCL;	
      if(data_write & 0x80)						//???μ??D′êy?Y
        SET_SPI_MOSI;
      else
        CLR_SPI_MOSI;
      delay(28);
      SET_SPI_SCL;										//é?éy???áè?êy?Y									
      if(READ_MISO_DATA)						               
        data_read |= 0x01;
      if(count_spi<7)											//??Dèòa×óò?7′?
      {
        data_write = data_write << 1; 
        data_read = data_read << 1;
      }
		
      delay(28);

    }

    break;
  default:
    break;
  }
  return(data_read); 
}

uint8 SPI_Read(uint8 reg)
{
	uint8 reg_val;
//	uint16 count_spi;
	CLR_SPI_NCS;                
	SPI_Basic_RW(ADNS3080_READ_REG + reg);       
	// SysCtlDelay(SysCtlClockGet()/1000);                  //实际上测出为3ms
        delay(9900);
	reg_val = SPI_Basic_RW(0x00);   
	SET_SPI_NCS;               
	return(reg_val);       
}

void SPI_Write(uint8 reg,uint8 value)
{
//	uint16 count_spi;
	CLR_SPI_NCS; 
	SPI_Basic_RW(ADNS3080_WRITE_REG + reg);
	// SysCtlDelay(SysCtlClockGet()/1000);              //实际上测出为3ms
        delay(9900);
	SPI_Basic_RW(value);
	SET_SPI_NCS;
}

void motion_read(uint8 * data_buffer)
{
//       int32 i=8000;
	uint8 count_spi;
	CLR_SPI_NCS;                
	SPI_Basic_RW(ADNS3080_READ_REG + ADNS3080_MOTION_BURST);
	// SysCtlDelay(SysCtlClockGet()/30000);                //实际上测出s为100u
        delay(320);
	for(count_spi = 0;count_spi < 7;count_spi++)
	{
		*(data_buffer ++) = SPI_Basic_RW(0x00);
	}
	
	SET_SPI_NCS;
        
}

void data_fix(void)
{
  float diff_roll     = attitudeActual.Roll - last_roll;	//×a???a???è￡??ò×ó×a?ˉê±￡?diff_roll?a?o?μ￡??òóò×a?ˉê±?a?y?μ
  float diff_pitch    = attitudeActual.Pitch - last_pitch;
  last_roll   = attitudeActual.Roll;
  last_pitch  = attitudeActual.Pitch;
  float x_mm,y_mm;

  //float sum_x,sum_y;
   x=0;
  y=0;
  float High_Now;
  unsigned char move=0;
   move=ADNS3080_Data_Buffer[0];
   x=ADNS3080_Data_Buffer[1];
   y=ADNS3080_Data_Buffer[2];
   	if(x&0x80)
	  {
	  //xμ??t21??×a??	
	  x -= 1;
	  x = ~x;	
	  x=(-1)*x;
	  x-=256;
 	  }
	if(y&0x80)
	  {
	  //yμ??t21??×a??	
	  y -= 1;
	  y = ~y;	
	  y=(-1)*y;
	  y-=256;
	  } 
   
   if(move&0x10!=1)
     if(move&0x80)
     {
       // printf("%d,%d\n",sum_x,sum_y);
     }
     else
     {
       x=0;
       y=0;
     }
     x=x^y;
     y=x^y;
     x=x^y;
     
    x_mm =(float)x + (diff_pitch * radians_to_pixels_x);   //diff_pitch x_mm
    y_mm = (float)y + (diff_roll * radians_to_pixels_y); 
      High_Now= sonar_distance[0]*1000;
    // x=(25.4*(float)x *High_Now)/(12*1600);//?àà?=d_x*(25.4/1600)*n   ???Dn=????:????=8oá?×:??3¤
    //y=(25.4*(float)y *High_Now)/(12*1600);   
      x_mm=x_mm*High_Now * conv_factor;
      y_mm=y_mm*High_Now * conv_factor;
      if(sonar_distance[0]*100>=20)
      {
    SumX=SumX+x_mm;             //à??óX?áè?μ?ò??ˉêy?Y
     SumY=SumY+y_mm;			 //à??óY?áè?μ?ò??ˉêy?Y
      }
      else if (sonar_distance[0]*100<=15)
      {
        SumX=0;
        SumY=0;
      }
    x=0;
    y=0;
}

void delay(int32 count_delay)
{
  int32 i=count_delay;   // 28  10us
               //320  100us     9900  3ms  67000  20ms
   //SysCtlDelay(SysCtlClockGet()/600000);
  //  SysCtlDelay(SysCtlClockGet()/6000000);
   while(i>=0)
          i--;
}



