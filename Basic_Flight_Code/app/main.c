#include "common.h"
#include "include.h"
#include "IRQ_handler.h"
//#include "YRK_DATA_COMMON.h"
#include "main.h"

#include "data_common.h"
#include "param_common.h"
#include "eeprom.h"
#include "attitudesolving.h"
#include "stabilization.h"
#include "motor_control.h"
#include "sensorfetch.h"
#include "PX4Flow.h"
#include "KS103.h"



#define SHOW_RPY



int main()
{
  
  //int i=0;
  IntMasterDisable();
  lowlevel_init();
  DELAY_MS(100);
  PX4Flow_uart_init(115200,UART6_IRQHandler);
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
  EEPROMInit();
  
  KS103_init();
  
  mpu6050_soft_init();
  //ak8975_soft_init();
  
  tim1_init(PIT_IRQHandler);
  led_init();
  
  IntPriorityGroupingSet(3);
  
  ROM_IntPrioritySet(INT_UART6, 0x00);//光流传感器中断	
  ROM_IntPrioritySet(INT_TIMER1A, 0x02<<6);
  ROM_IntPrioritySet(INT_GPIOC, 0x01<<6);
  
  //ROM_IntPrioritySet(INT_WTIMER3A, 0x40);
  
  data_common_init();//数据常量初始化
  param_common_init();//参数常量初始化
  motorcontrol_init();
  
  //motor_reset();//                                                            //::note::Why the motors don't rotate if not in debug mode?              
  DELAY_MS(5000);
  AttitudeInitialize();
  StabilizationInitialize();
  
  while(!nrf_init(PORTC_IRQHandler)); 
  //   sonar_init();
  
  IntMasterEnable();
  
  UARTprintf("SYSTEM_BOOT\n");
  
  while(1);
  
}