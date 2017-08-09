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


uint16 UART6_RX_STA = 0;
uint8 UART6_RX_BUF[UART6_REC_LEN];
RENESAS_Struct RENESAS=
{
  0,
  0
};

void UART6_STInit(uint32 Baud_rate)
{
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
  
  GPIOPinConfigure(GPIO_PD4_U6RX);
  GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4);
  
  GPIOPinConfigure(GPIO_PD5_U6TX);                     
  GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_5);
  
  UARTConfigSetExpClk(UART6_BASE,
                      SysCtlClockGet(),
                      Baud_rate,
                      (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_WLEN_8));
  IntRegister(INT_UART6,UART6_IRQHandler);     

  UARTFIFODisable(UART6_BASE);  
  
  IntEnable(INT_UART6);
  UARTIntEnable(UART6_BASE, UART_INT_RX);
  
}

void UART6_IRQHandler()
{
  OSIntEnter(); 
  
  OS_ERR err;
  uint8 Res;
  
  UARTIntClear(UART6_BASE, UART_INT_RX);                                        
  while(!UARTCharsAvail(UART6_BASE));
  if(UARTCharsAvail(UART6_BASE))                                                //handle uart rx interrupt
  {
    
    Res = UARTCharGetNonBlocking(UART6_BASE);
//    UARTCharPut(UART6_BASE, Res);                                               //Test Uart receive(echo)
    if((UART6_RX_STA&0x8000)==0)
    {
      if(UART6_RX_STA&0x4000)
      {
        if(Res!=0x0a)
          UART6_RX_STA=0;
        else 
        {  
          UART6_RX_STA|=0x8000; 
//          if(UART6_RX_BUF[0] == '$')
          OSTaskSemPost(&RenesasTCB, OS_OPT_POST_NONE, &err);
        }
      }
      else
      {	
        if(Res==0x0d)
          UART6_RX_STA|=0x4000;
        else
        {
          UART6_RX_BUF[UART6_RX_STA&0X3FFF]=Res ;
          UART6_RX_STA++;
          if(UART6_RX_STA>(UART6_REC_LEN-1))UART6_RX_STA=0;  
        }		 
      }
    }   		 
  }
  
  //  OSTaskSemPost(&CameraTCB, OS_OPT_POST_NONE, &err);
  OSIntExit();    	
  
}

void UART6SendString(uint8* send)
{
  while(*send != NULL)
  {	
    UARTCharPut(UART6_BASE, *send++);
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
  
//  uint8 dtbuf[40];
  uint16 rxlen = 0;
  int8 seps[2] = "|";
  int8 *token;
  while(DEF_TRUE)
  {
    OSTaskSemPend(0,OS_OPT_PEND_BLOCKING,0,&err);
    
    if(UART6_RX_STA&0X8000 && UART6_RX_BUF[0] == '$')
    {
      rxlen = UART6_RX_STA&0X3FFF;
      UART6_RX_BUF[rxlen] = NULL;

//      sprintf((char*)dtbuf,"%s\r\n", UART6_RX_BUF);              
//      UART6SendString(dtbuf);
      token = strtok((char*)&UART6_RX_BUF[1], seps);       
      RENESAS.FLOW_X = atof(token);
      token = strtok(NULL, seps);
      RENESAS.FLOW_Y = atof(token);
    }
    UART6_RX_STA = 0;
  }  
}
