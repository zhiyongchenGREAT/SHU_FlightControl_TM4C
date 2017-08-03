/*
************************************************************************************************************************
*                                               UART should eliminated using uartstdio.h
************************************************************************************************************************
*/
#include "PX4Flow.h"

float x1,y1;
float px4_sumx=0,px4_sumy=0;
FLOW flow;
FLOW_RAD flow_rad;
FLOW_FIX flow_fix;
uint8 FLOW_STATE[4];
uint8 flow_buf[27];
uint64 flow_buf_rad[45];
//uint8 shape_state=0;

float last_roll=0;
float last_pitch=0;
//光流输出给PID调节的值
float SumX=0;
float SumY=0;
//飞机当前坐标
float global_x=0,global_y=0;


//float High_Now;
float SumX_amend=0;
float SumY_amend=0;

//float radians_to_pixels_x = 2.5435f,radians_to_pixels_y = 2.5435f;
//float radians_to_pixels_x = 15.6,radians_to_pixels_y = 15.6;
float radians_to_pixels_x = 20,radians_to_pixels_y = 20;
float conv_factor =  0.0010f; 


//int8_t  x=0;
//int8_t  y=0;

float ByteToFloat(char *byteArry)
{
  return *((float*)byteArry);
}

void PX4Flow_uart_init(uint32 band,void (*pfnHandler)(void))
{
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
  
  GPIOPinConfigure(GPIO_PD4_U6RX);                    //配置输出引脚    PB1=usb1Rx
  GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4);
  
  GPIOPinConfigure(GPIO_PD5_U6TX);                     //配置输出引脚    PB1=usb1Rx ,PB2=usb1TX
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
  // uint8 cnt_offset=0;        
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
      flow_buf[data_cnt++]=data;
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
    //  USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    
    if(FLOW_STATE[3]==106)
    {
      flow_rad.time_usec=(flow_buf_rad[7]<<63)|(flow_buf_rad[6]<<56)|(flow_buf_rad[5]<<48)|(flow_buf_rad[4]<<40)
        |(flow_buf_rad[3]<<32)|(flow_buf_rad[2]<<16)|(flow_buf_rad[1]<<8)|(flow_buf_rad[0]);
      flow_rad.integration_time_us=(flow_buf_rad[11]<<32)|(flow_buf_rad[10]<<16)|(flow_buf_rad[9]<<8)|(flow_buf_rad[8]);
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
      flow_rad.time_delta_distance_us=(flow_buf_rad[35]<<32)|(flow_buf_rad[34]<<16)|(flow_buf_rad[33]<<8)|(flow_buf_rad[32]);
      floattobyte[0]=flow_buf_rad[36];
      floattobyte[1]=flow_buf_rad[37];
      floattobyte[2]=flow_buf_rad[38];
      floattobyte[3]=flow_buf_rad[39];
      flow_rad.distance=ByteToFloat(floattobyte);
      flow_rad.temperature=(flow_buf_rad[41]<<8)|(flow_buf_rad[40]);
      flow_rad.sensor_id=(flow_buf_rad[42]);
      flow_rad.quality=(flow_buf_rad[43]);
      
      //flow_fix.flow_x.origin =flow.flow_x.origin + flow_rad.integrated_ygyro*flow_fix.scale_rad_fix;
      //flow_fix.flow_y.origin =flow.flow_y.origin - flow_rad.integrated_xgyro*flow_fix.scale_rad_fix;
      //flow_fix.flow_comp_x.originf =flow.flow_comp_x.originf - flow_rad.integrated_ygyro*flow_fix.scale_rad_fix_comp;
      //flow_fix.flow_comp_y.originf =flow.flow_comp_y.originf + flow_rad.integrated_xgyro*flow_fix.scale_rad_fix_comp;
      //x=flow_fix.flow_comp_x.originf;
      //y= flow_fix.flow_comp_y.originf;
      
      x1 = flow_rad.integrated_x;// - flow_rad.integrated_xgyro;   //角度修正
      y1 = flow_rad.integrated_y;// - flow_rad.integrated_ygyro;
      px4_sumx+=flow_rad.integrated_x*1000;
      px4_sumy+=flow_rad.integrated_y*1000; 
    }        
  }
}


void px4_data_fix(void)
{
//  float sum_trf=0;
  float diff_roll     = attitudeActual.Roll - last_roll;	                //float last_roll=0 init
  float diff_pitch    = attitudeActual.Pitch - last_pitch;                      //float last_pitch=0 init
  last_roll   = attitudeActual.Roll;
  last_pitch  = attitudeActual.Pitch;
  float x_mm,y_mm;
  static float High_Now_before=0;
  
  //float sum_x,sum_y;
  float High_Now;
  //unsigned char move=0;
  //move=ADNS3080_Data_Buffer[0];
  
  y_mm =(float)px4_sumy - (diff_roll * radians_to_pixels_y);   //单位是像素      //float radians_to_pixels_x = 15.6; radians_to_pixels_y = 15.6;
  x_mm =(float)px4_sumx - (diff_pitch * radians_to_pixels_x);                    //float px4_sumx=0,px4_sumy=0;
  High_Now = ks103_distance;   //单位是毫米
  if(High_Now-High_Now_before>1000 || High_Now_before-High_Now<-1000)           //::note::large delta height
  {
    High_Now=High_Now_before;
  }
  if(High_Now<200)
  {
    High_Now=0; 
  }
  
  y_mm=y_mm*High_Now * conv_factor;// * 6;   //单位是mm  //最佳高度是50-60cm           //float conv_factor =  0.0010f init
  x_mm=x_mm*High_Now * conv_factor;// * 6;
  
////光流输出给PID调节的值
//float SumX=0;
//float SumY=0;
////飞机当前坐标
//float global_x=0,global_y=0;  
  
  if(High_Now/10>=20)
  {
    SumX=SumX+x_mm;      
    SumY=SumY+y_mm;	
    
    global_x+=x_mm;
    global_y+=y_mm;
  }
  else if (High_Now/10<=20)
  {
    SumX=0; 
    SumY=0;
    //	shape_state=0;
  }
  SumX_amend=SumX/10;
  SumY_amend=SumY/10;
  
  px4_sumx=0;                                                                   //::note::usage of px4_sumx/y, and why clear here?   
  px4_sumy=0;
  High_Now_before=High_Now;
  
}