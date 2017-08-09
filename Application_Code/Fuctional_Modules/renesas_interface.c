/*
************************************************************************************************************************
*                                                TI-TM4C Flight Control
*                                               SCIE/Shanghai University
*                                              
* File    : renesas_interface.c
* By      : Bicbrv
* Note    : Competition interface
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

#include "renesas_interface.h"


uint16 UART2_RX_STA = 0;
uint8 UART2_RX_BUF[UART2_REC_LEN];

void UART2_STInit(uint32 Baud_rate)
{
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
  
  GPIOPinConfigure(GPIO_PD6_U2RX);
  GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6);
  
  GPIOPinConfigure(GPIO_PD7_U2TX);                     
  GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_7);
  
  UARTConfigSetExpClk(UART2_BASE,
                      SysCtlClockGet(),
                      Baud_rate,
                      (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_WLEN_8));
  IntRegister(INT_UART2,UART2_IRQHandler);     

  UARTFIFODisable(UART2_BASE);  
  
  IntEnable(INT_UART2);
  UARTIntEnable(UART2_BASE, UART_INT_RX);
  
}

void UART2_IRQHandler()
{
  OSIntEnter(); 
  
  OS_ERR err;
  uint8 Res;
  
  UARTIntClear(UART2_BASE, UART_INT_RX);                                        
  while(!UARTCharsAvail(UART2_BASE));
  if(UARTCharsAvail(UART2_BASE))                                                //handle uart rx interrupt
  {
    
    Res = UARTCharGetNonBlocking(UART2_BASE);
    UARTCharPut(UART2_BASE, Res);                                               //Test Uart receive(echo)
    if((UART2_RX_STA&0x8000)==0)
    {
      if(UART2_RX_STA&0x4000)
      {
        if(Res!=0x0a)
          UART2_RX_STA=0;
        else 
        {  
          UART2_RX_STA|=0x8000; 
          if(UART2_RX_BUF[0] == '$')
            OSTaskSemPost(&RenesasTCB, OS_OPT_POST_NONE, &err);
        }
      }
      else
      {	
        if(Res==0x0d)
          UART2_RX_STA|=0x4000;
        else
        {
          UART2_RX_BUF[UART2_RX_STA&0X3FFF]=Res ;
          UART2_RX_STA++;
          if(UART2_RX_STA>(UART2_REC_LEN-1))UART2_RX_STA=0;  
        }		 
      }
    }   		 
  }
  
  //  OSTaskSemPost(&CameraTCB, OS_OPT_POST_NONE, &err);
  OSIntExit();    	
  
}

void UART2SendString(uint8* send)
{
  while(*send != NULL)
  {	
    UARTCharPut(UART2_BASE, *send++);
  }
}

/*
========================================================================================================================
*                                               uc/OS Task
========================================================================================================================
*/
void renesas_interface(void *p_arg)
{
  OS_ERR err;
//  CPU_SR_ALLOC();
  p_arg = p_arg;  
//  CPU_TS ts;
  
  uint8 dtbuf[40];
  uint16 rxlen = 0;
//  int8 seps[2] = "|";
//  int8 *token;
  while(DEF_TRUE)
  {
    OSTaskSemPend(0,OS_OPT_PEND_BLOCKING,0,&err);
    
    if(UART2_RX_STA&0X8000 && UART2_RX_BUF[0] == '$')
    {
      rxlen = UART2_RX_STA&0X3FFF;
      UART2_RX_BUF[rxlen] = NULL;

      sprintf((char*)dtbuf,"%s\r\n", UART2_RX_BUF);              
      UART2SendString(dtbuf);       
      
      UART2_RX_STA = 0;
    }	
  }  
}