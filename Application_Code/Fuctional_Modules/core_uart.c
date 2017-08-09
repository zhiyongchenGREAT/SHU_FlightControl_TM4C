#include "core_uart.h"


uint16 UART1_RX_STA = 0;
uint8 UART1_RX_BUF[UART1_REC_LEN];

//struct PIDStruct PID = {0.045, 0.0035, 37.6, 0.045, 0.0035, 37.6};
UART_PIDadjust_Struct UART_PIDadjust = 
{
  0.045, 
  0.0035, 
  37.6, 
  0.045, 
  0.0035, 
  37.6,
  0.0017, 
  0.0, 
  0.000072,
  0.0017, 
  0.0, 
  0.000072,
  0.0035, 
  0.0035, 
  0.0,
  10,
  0,
  90
};

void UART1_STInit(uint32 Baud_rate)
{
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
  
  GPIOPinConfigure(GPIO_PB0_U1RX);
  GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0);
  
  GPIOPinConfigure(GPIO_PB1_U1TX);                     
  GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_1);
  
  //    UARTClockSourceSet(UART1_BASE, UART_CLOCK_SYSTEM);                          //set system clock as the uart clk source
  UARTConfigSetExpClk(UART1_BASE,
                      SysCtlClockGet(),
                      Baud_rate,
                      (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_WLEN_8));
  IntRegister(INT_UART1,UART1_IRQHandler);     
  
//  UARTFIFOLevelSet(UART1_BASE,
//                   UART_FIFO_TX1_8,
//                   UART_FIFO_RX1_8);

  UARTFIFODisable(UART1_BASE);  
  
  IntEnable(INT_UART1);                                                       //enable the UART interrupt
  UARTIntEnable(UART1_BASE, UART_INT_RX);                                     //only enable RX and TX interrupts
  
//    UARTStdioConfig(1, Baud_rate, ROM_SysCtlClockGet());
}

void UART1SendString(uint8* send)
{
  while(*send != NULL)
  {	
    UARTCharPut(UART1_BASE, *send++);
  }
}

void GPIO_PINB7init(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); 
  
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  ROM_GPIOPinTypeGPIOOutputOD(GPIO_PORTB_BASE, GPIO_PIN_7);
  
  GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7,  0);
}

void UART1_IRQHandler()
{
  OSIntEnter(); 
  
  OS_ERR err;
  uint8 Res;
  
  UARTIntClear(UART1_BASE, UART_INT_RX);                                        
  while(!UARTCharsAvail(UART1_BASE));
  if(UARTCharsAvail(UART1_BASE))                                                //handle uart rx interrupt
  {
    
    Res = UARTCharGetNonBlocking(UART1_BASE);
    UARTCharPut(UART1_BASE, Res);                                               //Test Uart receive(echo)
    if((UART1_RX_STA&0x8000)==0)
    {
      if(UART1_RX_STA&0x4000)
      {
        if(Res!=0x0a)UART1_RX_STA=0;
        else 
        {  
          UART1_RX_STA|=0x8000; 
          OSTaskSemPost(&CameraTCB, OS_OPT_POST_NONE, &err);
          if(UART1_RX_BUF[0] == '~')
            OSTaskSemPost(&UARTAdjustTCB, OS_OPT_POST_NONE, &err);
          else if(UART1_RX_BUF[0] != '~')            
            OSTaskSemPost(&AUTOtestflight, OS_OPT_POST_NONE, &err);
        }
      }
      else
      {	
        if(Res==0x0d) UART1_RX_STA|=0x4000;
        else
        {
          UART1_RX_BUF[UART1_RX_STA&0X3FFF]=Res ;
          UART1_RX_STA++;
          if(UART1_RX_STA>(UART1_REC_LEN-1))UART1_RX_STA=0;  
        }		 
      }
    }   		 
  }
  
  //  OSTaskSemPost(&CameraTCB, OS_OPT_POST_NONE, &err);
  OSIntExit();    	
  
}

/*
========================================================================================================================
*                                               ucOS Task
========================================================================================================================
*/

void uart1_task(void *p_arg)
{
  OS_ERR err;	
  CPU_SR_ALLOC();
  p_arg = p_arg;
  
  while(1)
  {
    OS_CRITICAL_ENTER();
    UART1SendString("Hello world!\r\n");
    OS_CRITICAL_EXIT();
    OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时500ms
  }
}

void uart1_int_handler_task(void *p_arg)
{
  OS_ERR err;	
  CPU_SR_ALLOC();
  p_arg = p_arg;
  
  uint8 dtbuf[50];
  uint16 rxlen = 0;
  while(1)
  {
    if(UART1_RX_STA&0X8000)
    {
      
      
      rxlen = UART1_RX_STA&0X3FFF;
      UART1_RX_BUF[rxlen] = NULL;
      OS_CRITICAL_ENTER(); 
      sprintf((char*)dtbuf,"UART1接受到的内容为: %s\r\n", UART1_RX_BUF);        
      OS_CRITICAL_EXIT(); 
      UART1SendString(dtbuf);
      
      
      UART1_RX_STA = 0;
      
    }	
    OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err); //延时500ms
  }
}

void uart_report_task(void *p_arg)
{
  OS_ERR err;	
//  CPU_SR_ALLOC();
  p_arg = p_arg;
 
  uint8 dtbuf[100];
  
  while(DEF_TRUE)
  {
    
    OSTaskSemPend(0,OS_OPT_PEND_BLOCKING,0,&err);
    
    sprintf((char*)dtbuf,"P>%d R>%d Y>%d H>%d FX>%d FY>%d\r\n", 
            (int16)(attitudeActual.Pitch*100),
            (int16)(attitudeActual.Roll*100),
            (int16)(attitudeActual.Yaw*100),
            (int16)ks103_distance/10,
            (int16)(SumX_amend),
            (int16)(SumY_amend));

    UART1SendString(dtbuf);
  }
}

void uart_adjust_task(void *p_arg)
{
  OS_ERR err;
//  CPU_SR_ALLOC();
  p_arg = p_arg;  
//  CPU_TS ts;
  
  uint8 dtbuf[40];
  uint16 rxlen = 0;
  int8 seps[2] = "|";
  int8 *token;
  while(DEF_TRUE)
  {
    OSTaskSemPend(0,OS_OPT_PEND_BLOCKING,0,&err);
    
    if(UART1_RX_STA&0X8000 && UART1_RX_BUF[0] == '~')
    {
      rxlen = UART1_RX_STA&0X3FFF;
      UART1_RX_BUF[rxlen] = NULL;

//      sprintf((char*)dtbuf,"%s\r\n", UART1_RX_BUF);              
//      UART1SendString(dtbuf);
      
/* UARTadjust pend signal              */  
//      OSMutexPend(&PID_adjust_MUTEX,
//                0,
//                OS_OPT_PEND_BLOCKING,
//                &ts,
//                &err);
    OSSchedLock(&err);
/* UARTadjust receive decode&settings              */
      token = strtok((char*)&UART1_RX_BUF[1], seps);
      UART_PIDadjust.FLOW_XP = atof(token);
      token = strtok(NULL, seps);
      UART_PIDadjust.FLOW_XI = atof(token);
      token = strtok(NULL, seps);
      UART_PIDadjust.FLOW_XD = atof(token);
      
      token = strtok(NULL, seps);
      UART_PIDadjust.FLOW_YP = atof(token);
      token = strtok(NULL, seps);
      UART_PIDadjust.FLOW_YI = atof(token);
      token = strtok(NULL, seps);
      UART_PIDadjust.FLOW_YD = atof(token);
      
      token = strtok(NULL, seps);
      UART_PIDadjust.Pitch_P = atof(token);
      token = strtok(NULL, seps);
      UART_PIDadjust.Pitch_I = atof(token);
      token = strtok(NULL, seps);
      UART_PIDadjust.Pitch_D = atof(token);
      
      token = strtok(NULL, seps);
      UART_PIDadjust.Roll_P = atof(token);
      token = strtok(NULL, seps);
      UART_PIDadjust.Roll_I = atof(token);
      token = strtok(NULL, seps);
      UART_PIDadjust.Roll_D = atof(token);
      
      token = strtok(NULL, seps);
      UART_PIDadjust.Yaw_P = atof(token);
      token = strtok(NULL, seps);
      UART_PIDadjust.Yaw_I = atof(token);
      token = strtok(NULL, seps);
      UART_PIDadjust.Yaw_D = atof(token);  
      
      token = strtok(NULL, seps);
      UART_PIDadjust.Height_P = atof(token);
      token = strtok(NULL, seps);
      UART_PIDadjust.Height_I = atof(token);
      token = strtok(NULL, seps);
      UART_PIDadjust.Height_D = atof(token);

      
      stabilizationSettings.PitchRatePID[0] = UART_PIDadjust.Pitch_P;
      stabilizationSettings.PitchRatePID[1] = UART_PIDadjust.Pitch_I;
      stabilizationSettings.PitchRatePID[2] = UART_PIDadjust.Pitch_D;
      
      stabilizationSettings.RollRatePID[0] = UART_PIDadjust.Roll_P;
      stabilizationSettings.RollRatePID[1] = UART_PIDadjust.Roll_I;
      stabilizationSettings.RollRatePID[2] = UART_PIDadjust.Roll_D;

      stabilizationSettings.YawRatePID[0] = UART_PIDadjust.Yaw_P;
      stabilizationSettings.YawRatePID[1] = UART_PIDadjust.Yaw_I;
      stabilizationSettings.YawRatePID[2] = UART_PIDadjust.Yaw_D;      
      
      pid_configure(&pids[PID_RATE_PITCH],
                    stabilizationSettings.PitchRatePID[0],
                    stabilizationSettings.PitchRatePID[1],
                    stabilizationSettings.PitchRatePID[2],
                    stabilizationSettings.PitchRatePID[3]);
      
      pid_configure(&pids[PID_RATE_ROLL],
                    stabilizationSettings.RollRatePID[0],
                    stabilizationSettings.RollRatePID[1],
                    stabilizationSettings.RollRatePID[2],
                    stabilizationSettings.RollRatePID[3]);
      
      pid_configure(&pids[PID_RATE_YAW],
                    stabilizationSettings.YawRatePID[0],
                    stabilizationSettings.YawRatePID[1],
                    stabilizationSettings.YawRatePID[2],
                    stabilizationSettings.YawRatePID[3]);      
      
   

/* UARTadjust post signal              */      
//      OSMutexPost(&PID_adjust_MUTEX,
//                  OS_OPT_POST_NONE,
//                  &err);
    OSSchedUnlock(&err);      

      sprintf((char*)dtbuf,"FLOW>>\r\n%f %f %f %f %f %f\r\n", 
              UART_PIDadjust.FLOW_XP, 
              UART_PIDadjust.FLOW_XI, 
              UART_PIDadjust.FLOW_XD,
              UART_PIDadjust.FLOW_YP, 
              UART_PIDadjust.FLOW_YI, 
              UART_PIDadjust.FLOW_YD);
      
      UART1SendString(dtbuf);
      
      sprintf((char*)dtbuf,"PITCH>>\r\n%f %f %f\r\n", 
              UART_PIDadjust.Pitch_P, 
              UART_PIDadjust.Pitch_I, 
              UART_PIDadjust.Pitch_D);
      
      UART1SendString(dtbuf);  

      sprintf((char*)dtbuf,"ROLL>>\r\n%f %f %f\r\n", 
              UART_PIDadjust.Roll_P, 
              UART_PIDadjust.Roll_I, 
              UART_PIDadjust.Roll_D);
      
      UART1SendString(dtbuf);  

      sprintf((char*)dtbuf,"YAW>>\r\n%f %f %f\r\n", 
              UART_PIDadjust.Yaw_P, 
              UART_PIDadjust.Yaw_I, 
              UART_PIDadjust.Yaw_D);
      
      UART1SendString(dtbuf);  

      sprintf((char*)dtbuf,"HEIGHT>>\r\n%f %f %f\r\n", 
              UART_PIDadjust.Height_P, 
              UART_PIDadjust.Height_I, 
              UART_PIDadjust.Height_D);
      
      UART1SendString(dtbuf);        
      
      UART1_RX_STA = 0;
    }	

  }  
  
}