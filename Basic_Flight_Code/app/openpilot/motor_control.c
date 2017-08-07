/*
************************************************************************************************************************
*                                                TI-TM4C Flight Control
*                                               SCIE/Shanghai University
*                                              
* File    : motor_control.c
* By      : Bicbrv
* Note    : Motor control
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

#include "motor_control.h"

static const uint32 PWM_outpin[4] =
{
  PWM_OUT_4, PWM_OUT_5, PWM_OUT_6,PWM_OUT_7
};

void motorcontrol_init()
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);        

/*  First open the lock and select the bits we want to modify in the GPIO commit register.              */
/*  note: see datasheet p1329 PF[0] special function              */
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;

/* Now modify the configuration of the pins that we unlocked.              */

  HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x1;
  
/* Enable pin PF0 for PWM1 M1PWM4              */

  MAP_GPIOPinConfigure(GPIO_PF0_M1PWM4);
  MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);

/* Enable pin PF1 for PWM1 M1PWM5             */  

  MAP_GPIOPinConfigure(GPIO_PF1_M1PWM5);
  MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
  
/* Enable pin PF2 for PWM1 M1PWM6              */

  MAP_GPIOPinConfigure(GPIO_PF2_M1PWM6);
  MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
  
  
/* Enable pin PF3 for PWM1 M1PWM7              */

  MAP_GPIOPinConfigure(GPIO_PF3_M1PWM7);
  MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);    
  
  
  SysCtlPWMClockSet(SYSCTL_PWMDIV_16);

  PWMGenConfigure(PWM1_BASE,PWM_GEN_2,PWM_GEN_MODE_UP_DOWN| PWM_GEN_MODE_NO_SYNC);
  PWMGenConfigure(PWM1_BASE,PWM_GEN_3,PWM_GEN_MODE_UP_DOWN| PWM_GEN_MODE_NO_SYNC);

  
  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 12500);
  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 12500);

/* default: 1ms high in a 2.5ms period              */

  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_4, 5000);
  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 5000);
  
  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 5000);
  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 5000);
  
  PWMOutputState(PWM1_BASE, (PWM_OUT_4_BIT|PWM_OUT_5_BIT|PWM_OUT_6_BIT
                             |PWM_OUT_7_BIT), true);

  PWMGenEnable(PWM1_BASE, PWM_GEN_2);
  PWMGenEnable(PWM1_BASE, PWM_GEN_3);
  motorspeed_set(0,0,0);
  motorspeed_set(1,0,0);
  motorspeed_set(2,0,0);
  motorspeed_set(3,0,0);
  
}

void motorspeed_set(uint8 chn,_Bool armed,float rate)
{
  uint32 pwmout=PWM_outpin[chn];
  
  if(chn>3)
    return;
  
  if(!armed)
  {
/* motorSettings.Motor[0~3][MOTORSETTINGS_EDP_L] = 1000              */

    PWMPulseWidthSet(PWM1_BASE, 
                     pwmout, 
                     (uint32)(motorSettings.Motor[chn][MOTORSETTINGS_EDP_L]*5));    
  }
  else
  {
/* motorSettings.Motor[0~3][MOTORSETTINGS_EDP_H] = 1900              */
/* motorSettings.Motor[0~3][MOTORSETTINGS_STP] = 1050              */

    rate=rate*(motorSettings.Motor[chn][MOTORSETTINGS_EDP_H]                    
               -motorSettings.Motor[chn][MOTORSETTINGS_STP]);                   
    
    rate += motorSettings.Motor[chn][MOTORSETTINGS_STP];
/* rate = Motor[STP] + (Motor[H] - Motor[STP]) * (input rate) with ceiling Motor[H]              */
/* rate belongs to [0, 1]              */    
    if(rate>motorSettings.Motor[chn][MOTORSETTINGS_EDP_H])
      rate=motorSettings.Motor[chn][MOTORSETTINGS_EDP_H];
    if(rate<motorSettings.Motor[chn][MOTORSETTINGS_STP])
      rate=motorSettings.Motor[chn][MOTORSETTINGS_STP];                         
    
    PWMPulseWidthSet(PWM1_BASE, pwmout,(uint32)(rate*5));
    
  }
}

void motor_reset()
{
  
  PWMPulseWidthSet(PWM1_BASE, PWM_outpin[0],10000);
  PWMPulseWidthSet(PWM1_BASE, PWM_outpin[1],10000);
  PWMPulseWidthSet(PWM1_BASE, PWM_outpin[2],10000);
  PWMPulseWidthSet(PWM1_BASE, PWM_outpin[3],10000);  
  DELAY_MS(3000);
  motorspeed_set(0,0,0);
  motorspeed_set(1,0,0);
  motorspeed_set(2,0,0);
  motorspeed_set(3,0,0);
  DELAY_MS(3000);
}