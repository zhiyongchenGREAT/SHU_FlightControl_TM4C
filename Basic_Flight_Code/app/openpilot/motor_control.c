#include "motor_control.h"
#include "common.h"
//#include "MKL_TPM.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"

static const uint32 PWM_outpin[4] =
{
  PWM_OUT_4, PWM_OUT_5, PWM_OUT_6,PWM_OUT_7
};

void motorcontrol_init()
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);//使能PWM0模块
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);//使能PWM0和PWM1输出所在GPIO        
  //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//使能PWM0和PWM1输出所在GPIO
  //
  // First open the lock and select the bits we want to modify in the GPIO commit register.
  //::note::see datasheet p1329 PF[0] special function
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
  //
  // Now modify the configuration of the pins that we unlocked.
  //
  HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x1;
  
  //
  // Enable pin PF0 for PWM1 M1PWM4
  //
  MAP_GPIOPinConfigure(GPIO_PF0_M1PWM4);
  MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);
  
  //
  // Enable pin PF1 for PWM1 M1PWM5
  //
  MAP_GPIOPinConfigure(GPIO_PF1_M1PWM5);
  MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
  
  //
  // Enable pin PF2 for PWM1 M1PWM6
  //
  MAP_GPIOPinConfigure(GPIO_PF2_M1PWM6);
  MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
  
  
  //
  // Enable pin PF3 for PWM1 M1PWM7
  //
  MAP_GPIOPinConfigure(GPIO_PF3_M1PWM7);
  MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);    
  
  
//  SysCtlPWMClockSet(SYSCTL_PWMDIV_8);     // PWM时钟配置：8分频
  SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
  //配置PWM发生器0：加减计数，不同步
  PWMGenConfigure(PWM1_BASE,PWM_GEN_2,PWM_GEN_MODE_UP_DOWN| PWM_GEN_MODE_NO_SYNC);
  PWMGenConfigure(PWM1_BASE,PWM_GEN_3,PWM_GEN_MODE_UP_DOWN| PWM_GEN_MODE_NO_SYNC);
  //设置PWM发生器1的频率，时钟频率/PWM分频数/n，80M/8/25000=400hz
//  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 25000);
//  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 25000);
  
  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 12500);
  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 12500);
  //设置PWM0/PWM1输出的脉冲宽度
//  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_4, 10000);//1MS
//  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 10000);//1MS
//  
//  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 10000);//1MS
//  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 10000);//1MS

  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_4, 5000);//1MS
  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 5000);//1MS
  
  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 5000);//1MS
  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 5000);//1MS
  //使能PWM6和PWM7的输出
  
  PWMOutputState(PWM1_BASE, (PWM_OUT_4_BIT|PWM_OUT_5_BIT|PWM_OUT_6_BIT
                             |PWM_OUT_7_BIT), true);
  //使能PWM发生器
  PWMGenEnable(PWM1_BASE, PWM_GEN_2);
  PWMGenEnable(PWM1_BASE, PWM_GEN_3);
  motorspeed_set(0,0,0);
  motorspeed_set(1,0,0);
  motorspeed_set(2,0,0);
  motorspeed_set(3,0,0);
  
}
void motorspeed_set(uint8 chn,_Bool armed,float rate)
{
  uint32 pwmout=PWM_outpin[chn];                                                //PWM_outpin is PWM_OUT_4~7
  
  if(chn>3)
    return;
  if(!armed)
  {
    //TPM_CnV_REG(TPM0_BASE_PTR,chn)=(uint16)(motorSettings.Motor[chn][MOTORSETTINGS_EDP_L]*13.75);//13.75=34375/(1e+6us/400hz)
//    PWMPulseWidthSet(PWM1_BASE, 
//                     pwmout, 
//                     (uint32)(motorSettings.Motor[chn][MOTORSETTINGS_EDP_L]     //motorSettings.Motor[0~3][MOTORSETTINGS_EDP_L] = 1000 initialized in param_common.c
//                              *10));
    PWMPulseWidthSet(PWM1_BASE, 
                     pwmout, 
                     (uint32)(motorSettings.Motor[chn][MOTORSETTINGS_EDP_L]     //motorSettings.Motor[0~3][MOTORSETTINGS_EDP_L] = 1000 initialized in param_common.c
                              *5));    
  }
  else
  {
    rate=rate*(motorSettings.Motor[chn][MOTORSETTINGS_EDP_H]                    //motorSettings.Motor[0~3][MOTORSETTINGS_EDP_H] = 1900 initialized
               -motorSettings.Motor[chn][MOTORSETTINGS_STP]);                   //motorSettings.Motor[0~3][MOTORSETTINGS_STP] = 1050 initialized
    
    rate+=motorSettings.Motor[chn][MOTORSETTINGS_STP];
    
    if(rate>motorSettings.Motor[chn][MOTORSETTINGS_EDP_H])
      rate=motorSettings.Motor[chn][MOTORSETTINGS_EDP_H];
    if(rate<motorSettings.Motor[chn][MOTORSETTINGS_STP])
      rate=motorSettings.Motor[chn][MOTORSETTINGS_STP];                         //rate = Motor[STP] + (Motor[H] - Motor[STP]) * (input rate) with ceiling Motor[H]
    
    //TPM_CnV_REG(TPM0_BASE_PTR,chn)=(uint16)(rate*13.75);//13.75=34375/(1e+6us/400hz) 
//    PWMPulseWidthSet(PWM1_BASE, pwmout,(uint32)(rate*10));
    PWMPulseWidthSet(PWM1_BASE, pwmout,(uint32)(rate*5));
    
  }
}
void motor_reset()
{
//  PWMPulseWidthSet(PWM1_BASE, PWM_outpin[0],20000);
//  PWMPulseWidthSet(PWM1_BASE, PWM_outpin[1],20000);
//  PWMPulseWidthSet(PWM1_BASE, PWM_outpin[2],20000);
//  PWMPulseWidthSet(PWM1_BASE, PWM_outpin[3],20000);
  
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