#include "core_camera.h"

CPU_INT32U  pitchinit = 8500;
CPU_INT32U  yawinit = 7500;

void Camera_init(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);//使能PWM0模块
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//使能PWM0和PWM1输出所在GPIO        
  
  //
  // Enable pin PF0 for PWM1 M1PWM4
  //
  MAP_GPIOPinConfigure(GPIO_PB4_M0PWM2);
  MAP_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);
  
  //
  // Enable pin PF1 for PWM1 M1PWM5
  //
  MAP_GPIOPinConfigure(GPIO_PB5_M0PWM3);
  MAP_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);
  
  //
  // Enable pin PF2 for PWM1 M1PWM6
  //
  MAP_GPIOPinConfigure(GPIO_PB6_M0PWM0);
  MAP_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
     
  
  
  SysCtlPWMClockSet(SYSCTL_PWMDIV_16);     // PWM时钟配置：16分频
  //配置PWM发生器0：加减计数，不同步
  PWMGenConfigure(PWM0_BASE,PWM_GEN_0,PWM_GEN_MODE_UP_DOWN| PWM_GEN_MODE_NO_SYNC);
  PWMGenConfigure(PWM0_BASE,PWM_GEN_1,PWM_GEN_MODE_UP_DOWN| PWM_GEN_MODE_NO_SYNC);
  //设置PWM发生器1的频率，时钟频率/PWM分频数/n，80M/8/25000=400hz
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 100000);
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 100000);
  //设置PWM0/PWM1输出的脉冲宽度
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 8500);//1MS 5000-10000 7500
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 7500);//1MS
  
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 7500);//1MS
  //使能PWM6和PWM7的输出
  
  PWMOutputState(PWM0_BASE, (PWM_OUT_2_BIT|PWM_OUT_3_BIT
                             |PWM_OUT_0_BIT), true);
  //使能PWM发生器
  PWMGenEnable(PWM0_BASE, PWM_GEN_0);
  PWMGenEnable(PWM0_BASE, PWM_GEN_1);  
}

/*
**********************************************************************************************************
                                          UCOS TASK
**********************************************************************************************************
*/ 
void camera_task(void *p_arg)
{
  OS_ERR err;
  
  uint8 dtbuf[40];
  uint16 rxlen = 0;
//  uint8 tempString[5] = {0};
  UART1_RX_STA = 0;
  while(1)
  {
    OSTaskSemPend(0,OS_OPT_PEND_BLOCKING,0,&err);
    if(UART1_RX_STA&0X8000)
    {
      rxlen = UART1_RX_STA&0X3FFF;
      UART1_RX_BUF[rxlen] = NULL;
      //        OS_CRITICAL_ENTER(); 
      sprintf((char*)dtbuf,"%s\r\n", UART1_RX_BUF);        
      //        OS_CRITICAL_EXIT(); 
//      UART1SendString(dtbuf);
//      tempString[0] = UART1_RX_BUF[0];
//      tempString[1] = UART1_RX_BUF[1];
//      UART1SendString(tempString);
      
      switch(UART1_RX_BUF[0])
      {
      case 'L':
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (yawinit-=2));
        break;
        
      case 'R':
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (yawinit+=2));           
        break;
        
      case 'H':
        break;
      
      default:
        break;
        
      }
      
      switch(UART1_RX_BUF[1])
      {
      case 'U':
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (pitchinit-=2));
        break;
        
      case 'D':
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (pitchinit+=2));              
        break;          
      
      case 'H':
        break;
      
      default:
        break;
        
      }

      sprintf((char*)dtbuf,"Cur-Pit:%d Cur-Yaw:%d\r\n", pitchinit, yawinit);       
      
      UART1SendString(dtbuf);
      
//      memset(UART1_RX_BUF, 0, sizeof(UART1_RX_BUF));
      UART1_RX_STA = 0;
    }	
    //    OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err); //延时500ms
  }  
  
}
