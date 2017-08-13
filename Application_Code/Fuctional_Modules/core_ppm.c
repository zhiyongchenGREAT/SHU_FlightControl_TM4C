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
volatile uint32 CAP_count = 0;
uint32 CAP_value[9] = {0};

void PPM_init(void (*pfnHandler)(void))
{
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
  
  GPIOPinConfigure(GPIO_PC6_WT1CCP0); 
  GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_6);
  
  TimerConfigure(WTIMER1_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_CAP_TIME);
  TimerControlEvent(WTIMER1_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);  
  
  TimerLoadSet(WTIMER1_BASE, TIMER_A, 0xffffffff);
  
  IntRegister(INT_WTIMER1A, pfnHandler); 

  IntEnable(INT_WTIMER1A); 
  TimerIntEnable(WTIMER1_BASE, TIMER_CAPA_EVENT);  

  TimerEnable(WTIMER1_BASE, TIMER_A);

  
  TimerControlStall(WTIMER1_BASE, TIMER_BOTH, true);

//  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
//  SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
//  
//  GPIOPinConfigure(GPIO_PC7_WT1CCP1); 
//  GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_7);
//  
//  TimerConfigure(WTIMER1_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_CAP_TIME_UP);
//  TimerControlEvent(WTIMER1_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);  
//  
//  IntRegister(INT_WTIMER1A, pfnHandler); 
//
//  IntEnable(INT_WTIMER1A); 
//  TimerIntEnable(WTIMER1_BASE, TIMER_CAPA_EVENT);  
//
//  TimerEnable(WTIMER1_BASE, TIMER_A);

}

void PPM_CAP_Int_Handler(void)
{
  OSIntEnter();
  
  OS_ERR err;  
  
  TimerIntClear(WTIMER1_BASE, TIMER_CAPA_EVENT);
  
  CAP_count = 0xffffffff - TimerValueGet(WTIMER1_BASE, TIMER_A);

  TimerLoadSet(WTIMER1_BASE, TIMER_A, 0xffffffff);
  
//  OSTaskSemPost(&RemoteCtrlTCB, OS_OPT_POST_NONE, &err);
  
  OSTaskQPost ((OS_TCB       *)&RemoteCtrlTCB,
               (void         *)CAP_count,
               (OS_MSG_SIZE   )sizeof(CAP_count),
               (OS_OPT        )OS_OPT_POST_FIFO,
               (OS_ERR       *)&err);  
  
  OSIntExit();  
}

/*
========================================================================================================================
*                                               uc/OS Task
========================================================================================================================
*/
void remote_controller_task(void *p_arg)
{
  OS_ERR err;
  OS_MSG_SIZE size;
  CPU_TS ts;  
  
  p_arg = p_arg;
  uint8 count = 0;
  
//  uint32 a =0;
  
  
  while(DEF_TRUE)
  {
//    OSTaskSemPend(0,OS_OPT_PEND_BLOCKING,0,&err);
    CAP_value[count++] = (CPU_INT32U)OSTaskQPend ((OS_TICK       )0,
                                                  (OS_OPT        )OS_OPT_PEND_BLOCKING,
                                                  (OS_MSG_SIZE  *)&size,
                                                  (CPU_TS       *)&ts,
                                                  (OS_ERR       *)&err); 
    
//    CAP_value[count++] = CAP_count;
    if(count >= 9)
      count = 0;
  }
}
