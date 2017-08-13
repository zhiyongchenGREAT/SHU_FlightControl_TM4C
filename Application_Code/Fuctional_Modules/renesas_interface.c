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
uint8 uart2_count = 0;

RENESAS_Struct RENESAS=
{
  0,
  0
};

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

//void UART2_IRQHandler()
//{
//  OSIntEnter(); 
//  
//  OS_ERR err;
//  uint8 Res;
//  
//  UARTIntClear(UART2_BASE, UART_INT_RX);                                        
//  while(!UARTCharsAvail(UART2_BASE));
//  if(UARTCharsAvail(UART2_BASE))                                                //handle uart rx interrupt
//  {
//    
//    Res = UARTCharGetNonBlocking(UART2_BASE);
////    UARTCharPut(UART2_BASE, Res);                                               //Test Uart receive(echo)
//    if((UART2_RX_STA&0x8000)==0)
//    {
//      if(UART2_RX_STA&0x4000)
//      {
//        if(Res!=0x0a)
//          UART2_RX_STA=0;
//        else 
//        {  
//          UART2_RX_STA|=0x8000; 
////          if(UART2_RX_BUF[0] == '$')
//          OSTaskSemPost(&RenesasTCB, OS_OPT_POST_NONE, &err);
//        }
//      }
//      else
//      {	
//        if(Res==0x0d)
//          UART2_RX_STA|=0x4000;
//        else
//        {
//          UART2_RX_BUF[UART2_RX_STA&0X3FFF]=Res ;
//          UART2_RX_STA++;
//          if(UART2_RX_STA>(UART2_REC_LEN-1))UART2_RX_STA=0;  
//        }		 
//      }
//    }   		 
//  }
//  
//  //  OSTaskSemPost(&CameraTCB, OS_OPT_POST_NONE, &err);
//  OSIntExit();    	
//  
//}

void UART2_IRQHandler()
{
  OSIntEnter(); 
  
  uint8 Res;
  
  UARTIntClear(UART2_BASE, UART_INT_RX);                                        
  while(!UARTCharsAvail(UART2_BASE));
  if(UARTCharsAvail(UART2_BASE))
  {
    
    Res = UARTCharGetNonBlocking(UART2_BASE);
    
    switch(uart2_count)
    {
    case 0:
      if(Res == 0x55)
        uart2_count = 1;
      break;
    case 1:
      if(Res == 0xaa)
        uart2_count = 2;
      else
        uart2_count = 0;
      break;
    case 2:
      UART2_RX_BUF[0] = Res;
      uart2_count = 3;
      break;
    case 3:
      UART2_RX_BUF[1] = Res;
      uart2_count = 4;
      break;
    case 4:
      if(Res == '|')
        uart2_count = 5;
      else
        uart2_count = 0;
      break;
    case 5:
      UART2_RX_BUF[2] = Res;
      uart2_count = 6;
      break;
    case 6:
      UART2_RX_BUF[3] = Res;
      uart2_count = 7;
      break;
    case 7:
      if(Res == '|')
        uart2_count = 8;
      else
        uart2_count = 0;
      break;
    case 8:
      UART2_RX_BUF[4] = Res;
      uart2_count = 9;
      break;      
    case 9:
      if(Res == 0x0d)
        uart2_count = 10;
      else
        uart2_count = 0;
      break;
    case 10:
      if(Res == 0x0a)
        uart2_count = 11;
      else
        uart2_count = 0;
      break;
    case 11:
      RENESAS.FLOW_X = (int16)(UART2_RX_BUF[0]|(UART2_RX_BUF[1]<<8));
      RENESAS.FLOW_Y = (int16)(UART2_RX_BUF[2]|(UART2_RX_BUF[3]<<8));
      land_flag = (uint8)(UART2_RX_BUF[4]);
      uart2_count = 0;
    default:
      uart2_count = 0;

    }
  
  }
  
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
//int recive_sua_flag=0;

void renesas_interface(void *p_arg)
{
  OS_ERR err;
//  CPU_SR_ALLOC();
  p_arg = p_arg;  
//  CPU_TS ts;
  
//  uint8 dtbuf[40];
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

//      token = strtok((char*)&UART2_RX_BUF[1], seps);       
//      RENESAS.FLOW_X = atof(token);
//      token = strtok(NULL, seps);
//      RENESAS.FLOW_Y = atof(token);
//      recive_sua_flag=1;
    }
    UART2_RX_STA = 0;
  }  
}
