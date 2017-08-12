#include "YRK_init.h"

enum COMPETITON_FLIGHT_MODE COMPETITON_FLIGHT_MODE = COM_TASK_0;

void lowlevel_init()
{
  ROM_FPUEnable();
  ROM_FPULazyStackingEnable();
  ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ);
}

void uart_init(uint32 band,void (*pfnHandler)(void))
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);

    GPIOPinConfigure(GPIO_PE4_U5RX);
    GPIOPinConfigure(GPIO_PE5_U5TX);
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
   
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //enable GPIO port for LED
    //GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1); //enable pin for LED PF2
   //UARTConfigSetExpClk(UART5_BASE, ROM_SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    IntRegister(INT_UART5,pfnHandler);  
    IntEnable(INT_UART5); //enable the UART interrupt
    UARTIntEnable(UART5_BASE, UART_INT_RX); //only enable RX and TX interrupts
    UARTClockSourceSet(UART5_BASE, UART_CLOCK_SYSTEM);
    UARTStdioConfig(5, band, ROM_SysCtlClockGet());
//    ROM_IntEnable(INT_UART0);
//    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}

void tim1_init(void (*pfnHandler)(void))
{
    
  uint32_t ui32Period=0; 
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
  TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);

/* TIM1: 400Mhz              */
  
  ui32Period = ROM_SysCtlClockGet()/400;
  TimerLoadSet(TIMER1_BASE, TIMER_A, ui32Period -1);
  
  IntRegister(INT_TIMER1A, pfnHandler);
  IntEnable(INT_TIMER1A);
  TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  
  TimerControlStall(TIMER1_BASE, TIMER_BOTH, true);
  
  TimerEnable(TIMER1_BASE, TIMER_A);
}

void pwm_init(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);//使能PWM0模块
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);//使能PWM0和PWM1输出所在GPIO        
  //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//使能PWM0和PWM1输出所在GPIO
  
  
  // Enable pin PF3 for PWM1 M1PWM7
  //
  MAP_GPIOPinConfigure(GPIO_PF3_M1PWM7);
  MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);
  
  //
  // Enable pin PF0 for PWM1 M1PWM4
  // First open the lock and select the bits we want to modify in the GPIO commit register.
  //
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
  HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x1;
  
  //
  // Now modify the configuration of the pins that we unlocked.
  //
  MAP_GPIOPinConfigure(GPIO_PF0_M1PWM4);
  MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);
  
  //
  // Enable pin PF2 for PWM1 M1PWM6
  //
  MAP_GPIOPinConfigure(GPIO_PF2_M1PWM6);
  MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
  
  //
  // Enable pin PF1 for PWM1 M1PWM5
  //
  MAP_GPIOPinConfigure(GPIO_PF1_M1PWM5);
  MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
  
  
  SysCtlPWMClockSet(SYSCTL_PWMDIV_8);     // PWM时钟配置：8分频
  //配置PWM发生器0：加减计数，不同步
  PWMGenConfigure(PWM1_BASE,PWM_GEN_2,PWM_GEN_MODE_UP_DOWN| PWM_GEN_MODE_NO_SYNC);
  PWMGenConfigure(PWM1_BASE,PWM_GEN_3,PWM_GEN_MODE_UP_DOWN| PWM_GEN_MODE_NO_SYNC);
  //设置PWM发生器1的频率，时钟频率/PWM分频数/n，80M/8/25000=400hz
  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 25000);
  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 25000);
  //设置PWM0/PWM1输出的脉冲宽度
  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_4, 10000);//1MS
  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 10000);//1MS
  
  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 10000);//1MS
  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 10000);//1MS
  //使能PWM6和PWM7的输出
  
  PWMOutputState(PWM1_BASE, (PWM_OUT_4_BIT|PWM_OUT_5_BIT|PWM_OUT_6_BIT|PWM_OUT_7_BIT), true);
  //使能PWM发生器
  PWMGenEnable(PWM1_BASE, PWM_GEN_2);
  PWMGenEnable(PWM1_BASE, PWM_GEN_3);
}

/*
************************************************************************************************************************
*                                               按键1	按键2	读数值（PIN）
*                                               OFF	ON	8
*                                               ON	ON	0
*                                               OFF	OFF	12
*                                               ON	OFF	4
*                                               PF4     KEY
************************************************************************************************************************
*/

void GPIO_KEYinit(void)
{
//  int32_t PIN;
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); 
  
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))
  {
  }
  
  GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3);
  GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

  DELAY_MS(1000);
  
  if(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3) == 12)
    COMPETITON_FLIGHT_MODE = COM_TASK_0;
  else if(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3) == 4)
    COMPETITON_FLIGHT_MODE = COM_TASK_1;
  else if(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3) == 8)
    COMPETITON_FLIGHT_MODE = COM_TASK_2;
  else if(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3) == 0)
    COMPETITON_FLIGHT_MODE = COM_TASK_3;
  else
    COMPETITON_FLIGHT_MODE = COM_TASK_0;    
  
}

void STARTUP_KEY(void)
{
//  int32_t PIN;
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); 
  
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
  {
  }
  
  GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
  GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

//  DELAY_MS(1000);
//  
//  while(DEF_TRUE)
//  {
//    if(!(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) & GPIO_PIN_4))
//      break;
//    DELAY_MS(500);    
//    LED1_OFF();
//    if(!(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) & GPIO_PIN_4))
//      break;
//    DELAY_MS(500);    
//    LED1_ON();    
//  }
//  
//  LED0_ON();  
//  LED1_ON(); 
//  DELAY_MS(5000);
//  LED0_OFF();  
//  LED1_OFF();   
}