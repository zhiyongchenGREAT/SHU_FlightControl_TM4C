/*
************************************************************************************************************************
*                                                TI-TM4C Flight Control
*                                               SCIE/Shanghai University
*                                              
* File    : core_t_timer.c
* By      : Bicbrv
* Note    : Test purpose GPTM
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
#include "core_t_timer.h"

volatile CPU_INT32U t_tim0_cnt;

void testpurpose_tim0_init(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);
  
  TimerLoadSet(TIMER0_BASE, TIMER_A, 0xffffffff);
  
  TimerControlStall(TIMER0_BASE, TIMER_BOTH, true);
  
  TimerEnable(TIMER0_BASE, TIMER_A);
}

void tim3_init(void (*pfnHandler)(void))
{
    
  uint32_t ui32Period=0; 
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
  TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);

/* TIM3: 40Mhz              */
  
  ui32Period = ROM_SysCtlClockGet()/40;
  TimerLoadSet(TIMER3_BASE, TIMER_A, ui32Period -1);
  
  IntRegister(INT_TIMER3A, pfnHandler);
  IntEnable(INT_TIMER3A);
  TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
  
  TimerControlStall(TIMER3_BASE, TIMER_BOTH, true);
  
  TimerEnable(TIMER3_BASE, TIMER_A);
}