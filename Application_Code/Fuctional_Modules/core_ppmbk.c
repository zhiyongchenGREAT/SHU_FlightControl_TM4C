/*
************************************************************************************************************************
*                                                TI-TM4C Flight Control
*                                               SCIE/Shanghai University
*                                              
* File    : core_ppm.c
* By      : Bicbrv
* Note    : PPM controller receiver
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
#include "core_ppm.h"
volatile int32 CAP_count = 0;

void testpurpose_tim0_init(void)
{
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
  
  GPIOPinConfigure(GPIO_PC6_WT1CCP0); 
  GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_6);
  
  TimerConfigure(WTIMER1_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_CAP_TIME_UP);
  TimerControlEvent(WTIMER1_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);  
  
  IntEnable(WTIMER1_BASE);
  IntRegister(INT_WTIMER1A, CAP_Int_Handler);  
  TimerIntEnable(WTIMER1_BASE, TIMER_CAPA_EVENT);  
  TimerEnable(WTIMER1_BASE, TIMER_A);

  
//  TimerControlStall(WTIMER1_BASE, TIMER_BOTH, true);

}

void CAP_Int_Handler(void)
{
  OSIntEnter();
  
  OS_ERR err;  
  
  TimerIntClear(WTIMER1_BASE, TIMER_CAPA_EVENT);
  CAP_count = TimerValueGet(WTIMER1_BASE, TIMER_A)
  TimerLoadSet(WTIMER1_BASE, TIMER_A, 0x00000000);
  
  OSTaskSemPost(&RemoteCtrlTCB, OS_OPT_POST_NONE, &err);
  
  OSIntExit();  
}

/*
========================================================================================================================
*                                               ucOS Task
========================================================================================================================
*/
void remote_controller_task(void *p_arg)
{
  
}
