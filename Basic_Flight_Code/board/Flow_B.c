/*
************************************************************************************************************************
*                                                TI-TM4C Flight Control
*                                               SCIE/Shanghai University
*                                              
* File    : Flow_B.c
* By      : Bicbrv
* Note    : Flow improved version
*
* TERMS OF USE:
* ---------------
*           We provide ALL the source code for your convenience and to help you 
*           keep developing our flight control firmware.  
*
*           Please help us continue to provide our project with the finest software available.
*           Your dedicated work is greatly appreciated. Feel free to ameliorate any 
*           part of our code without any restriction to pursue maximum performance.
*
************************************************************************************************************************
*/
#include "Flow_B.h"

static float px4_sumx=0,px4_sumy=0;
//FLOW flow;
static FLOW_RAD flow_rad;
//FLOW_FIX flow_fix;
static uint8 FLOW_STATE[4];
static volatile uint8 flow_buf[27];
static uint8 flow_buf_rad[45];

//static float last_roll=0;
//static float last_pitch=0;
static float SumX=0;
static float SumY=0;

float SumX_amend=0;
float SumY_amend=0;

float flow_distance=0, flow_delta_distance=0, flow_last_distance=0;
/* test              */
float Xmm_Send=0;
float Ymm_Send=0;
static float last_roll=0;
static float last_pitch=0;
//const float radians_to_pixels_x = 10.9, radians_to_pixels_y = 10.9;

/* stable para: 6.95 6.95              */

static const float radians_to_pixels_x = 6.95*1.2, radians_to_pixels_y = 6.95*1.2;
//static const float radians_to_pixels_x = 4.63*0.75, radians_to_pixels_y = 4.63*0.75;
/* test              */

//static const float conv_factor =  0.0010f;
//static const float conv_factor =  0.00085f;
//static const float conv_factor =  0.00085f;
//static const float conv_factor =  0.001275f;
static const float conv_factor =  0.00153*0.85;


static float ByteToFloat(char *byteArry)
{
  return *((float*)byteArry);
}

void PX4Flow_uart_init(uint32 band,void (*pfnHandler)(void))
{
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
  
  GPIOPinConfigure(GPIO_PD4_U6RX);                   
  GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4);
  
  GPIOPinConfigure(GPIO_PD5_U6TX);                     
  GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_5);
  
  IntRegister(INT_UART6,pfnHandler);  
  IntEnable(INT_UART6); //enable the UART interrupt
  UARTIntEnable(UART6_BASE, UART_INT_RX); //only enable RX and TX interrupts
  UARTClockSourceSet(UART6_BASE, UART_CLOCK_SYSTEM);
  UARTStdioConfig(6, band, ROM_SysCtlClockGet());

//  UARTConfigSetExpClk(UART6_BASE,
//                      SysCtlClockGet(),
//                      band,
//                      (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE |
//                       UART_CONFIG_WLEN_8));
//
//  IntRegister(INT_UART6,pfnHandler);
//  
//  UARTFIFODisable(UART6_BASE);  
//  
//  IntEnable(INT_UART6);                                                       //enable the UART interrupt
//  UARTIntEnable(UART6_BASE, UART_INT_RX);  
}

void FLOW_MAVLINK(unsigned char data)
{
  static uint8 s_flow=0,data_cnt=0;
      
  uint8 get_one_fame=0;
  
  char floattobyte[4];
  
  switch(s_flow)
  {
  case 0: 
    if(data==0xFE)
      s_flow=1;
    break;
    
  case 1: 
    if(data==0x1A||data==0x2C)
      s_flow=2;
    else
      s_flow=0;
    break;
    
  case 2:
    if(data_cnt<4)
    {
      s_flow=2; 
      FLOW_STATE[data_cnt++]=data;
    }
    else
    {
      data_cnt=0;
      s_flow=3;
      flow_buf[data_cnt]=data;
      flow_buf_rad[data_cnt]=data;
      data_cnt++;
    }
    break;
    
  case 3:
    if(FLOW_STATE[3]==100)
    {
      if(data_cnt<26)
      {
        s_flow=3; 
        flow_buf[data_cnt++]=data;
      }
      else
      {
        data_cnt=0;
        s_flow=4;
      }
    }
    else if(FLOW_STATE[3]==106)
    {
      if(data_cnt<44)
      {
        s_flow=3; 
        flow_buf_rad[data_cnt++]=data;
      }
      else
      {
        data_cnt=0;
        s_flow=4;
      }
    }
    else
    {
      data_cnt=0;
      s_flow=0;
    }
    break;
    
  case 4:
    get_one_fame=1;
    s_flow=0;
    data_cnt=0;
    break;
    
  default:
    s_flow=0;
    data_cnt=0;
    break;
  }//--end of  switch
  
  if(get_one_fame)
  {
/* Note: TM4C model mcu use little endian mode so the lower bits of an int is stored before the higher part            */
 
    if(FLOW_STATE[3]==106)
    {
      flow_rad.time_usec = (uint64_t)(((uint64_t)flow_buf_rad[7]<<56)
                                      |((uint64_t)flow_buf_rad[6]<<48)
                                        |((uint64_t)flow_buf_rad[5]<<40)
                                          |((uint64_t)flow_buf_rad[4]<<32)
                                            |((uint64_t)flow_buf_rad[3]<<24)
                                              |((uint64_t)flow_buf_rad[2]<<16)
                                                |((uint64_t)flow_buf_rad[1]<<8)
                                                  |((uint64_t)flow_buf_rad[0]));
      
      flow_rad.integration_time_us = (uint32_t)((flow_buf_rad[11]<<24)
                                                |(flow_buf_rad[10]<<16)
                                                  |(flow_buf_rad[9]<<8)
                                                    |(flow_buf_rad[8]));
      floattobyte[0]=flow_buf_rad[12];
      floattobyte[1]=flow_buf_rad[13];
      floattobyte[2]=flow_buf_rad[14];
      floattobyte[3]=flow_buf_rad[15];
      flow_rad.integrated_x=ByteToFloat(floattobyte);
      floattobyte[0]=flow_buf_rad[16];
      floattobyte[1]=flow_buf_rad[17];
      floattobyte[2]=flow_buf_rad[18];
      floattobyte[3]=flow_buf_rad[19];
      flow_rad.integrated_y=ByteToFloat(floattobyte);
      floattobyte[0]=flow_buf_rad[20];
      floattobyte[1]=flow_buf_rad[21];
      floattobyte[2]=flow_buf_rad[22];
      floattobyte[3]=flow_buf_rad[23];
      flow_rad.integrated_xgyro=ByteToFloat(floattobyte);
      floattobyte[0]=flow_buf_rad[24];
      floattobyte[1]=flow_buf_rad[25];
      floattobyte[2]=flow_buf_rad[26];
      floattobyte[3]=flow_buf_rad[27];
      flow_rad.integrated_ygyro=ByteToFloat(floattobyte);
      floattobyte[0]=flow_buf_rad[28];
      floattobyte[1]=flow_buf_rad[29];
      floattobyte[2]=flow_buf_rad[30];
      floattobyte[3]=flow_buf_rad[31];
      flow_rad.integrated_zgyro=ByteToFloat(floattobyte);
      
      flow_rad.time_delta_distance_us=(uint32_t)((flow_buf_rad[35]<<24)
                                                 |(flow_buf_rad[34]<<16)
                                                   |(flow_buf_rad[33]<<8)
                                                     |(flow_buf_rad[32]));
      floattobyte[0]=flow_buf_rad[36];
      floattobyte[1]=flow_buf_rad[37];
      floattobyte[2]=flow_buf_rad[38];
      floattobyte[3]=flow_buf_rad[39];
      flow_rad.distance=ByteToFloat(floattobyte);
      flow_rad.temperature=(int16_t)(flow_buf_rad[41]<<8)|(flow_buf_rad[40]);
      flow_rad.sensor_id=(flow_buf_rad[42]);
      flow_rad.quality=(flow_buf_rad[43]);
      
      px4_sumx+=flow_rad.integrated_x*1000;
      px4_sumy+=flow_rad.integrated_y*1000;
      
      flow_distance = flow_rad.distance * 1000;
      flow_delta_distance = flow_distance - flow_last_distance;
      flow_last_distance = flow_distance;

//      px4_sumx += (flow_rad.integrated_x - flow_rad.integrated_xgyro) * 1000;
//      px4_sumy += (flow_rad.integrated_y - flow_rad.integrated_ygyro) * 1000; 
    }
  }
}


void px4_data_fix(void)
{
  float diff_roll     = attitudeActual.Roll - last_roll;	                
  float diff_pitch    = attitudeActual.Pitch - last_pitch;                     
  last_roll   = attitudeActual.Roll;
  last_pitch  = attitudeActual.Pitch;

  float x_mm,y_mm;
  static float High_Now_before=0;
  
  //float sum_x,sum_y;
  float High_Now;;
  
  y_mm =(float)px4_sumy - (diff_roll * radians_to_pixels_y);
  x_mm =(float)px4_sumx - (diff_pitch * radians_to_pixels_x);
  
  Xmm_Send += x_mm;
  Ymm_Send += y_mm;

//  y_mm =(float)px4_sumy;
//  x_mm =(float)px4_sumx;
  
  High_Now = ks103_distance;   //单位是毫米
  if(High_Now-High_Now_before>1000 || High_Now_before-High_Now<-1000)
  {
    High_Now=High_Now_before;
  }
  if(High_Now<200)
  {
    High_Now=0;
    Xmm_Send = 0;
    Ymm_Send = 0;
  }
  
  y_mm=y_mm*High_Now * conv_factor;
  x_mm=x_mm*High_Now * conv_factor;  
  
  if(High_Now/10>=20)
  {
    SumX=SumX+x_mm;      
    SumY=SumY+y_mm;	
  }
  
  else if (High_Now/10<=20)
  {
    SumX=0; 
    SumY=0;
  }
  
  SumX_amend=SumX/10;
  SumY_amend=SumY/10;
  
  px4_sumx=0;                                                                   //::note::usage of px4_sumx/y, and why clear here?   
  px4_sumy=0;
  High_Now_before=High_Now;
  
}