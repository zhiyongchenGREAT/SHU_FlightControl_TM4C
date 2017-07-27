#include "motor_control.h"
#include "common.h"
#include "MKL_TPM.h"
void motorcontrol_init()
{
    tpm_pwm_init(TPM0, TPM_CH0,motorSettings.Motor[0][MOTORSETTINGS_FREQ],0);        //初始化 TPM PWM ，使用 TPM0_CH3，频率为200k ，占空比为 30 / TPM0_PRECISON
    tpm_pwm_init(TPM0, TPM_CH1,motorSettings.Motor[1][MOTORSETTINGS_FREQ],0);                                                // fire_port_cfg.h 里 配置 TPM0_CH3 对应为 PTE30
    tpm_pwm_init(TPM0, TPM_CH2,motorSettings.Motor[2][MOTORSETTINGS_FREQ],0); 
    tpm_pwm_init(TPM0, TPM_CH3,motorSettings.Motor[3][MOTORSETTINGS_FREQ],0);  
    motorspeed_set(TPM_CH0,0,0);
    motorspeed_set(TPM_CH1,0,0);
    motorspeed_set(TPM_CH2,0,0);
    motorspeed_set(TPM_CH3,0,0);
      
}
void motorspeed_set(uint8 chn,_Bool armed,float rate)
{
  if(chn>3)
    return;
  if(!armed)
  {
    TPM_CnV_REG(TPM0_BASE_PTR,chn)=(uint16)(motorSettings.Motor[chn][MOTORSETTINGS_EDP_L]*13.75);//13.75=34375/(1e+6us/400hz)
  }
  else
  {
    rate=rate*(motorSettings.Motor[chn][MOTORSETTINGS_EDP_H]-motorSettings.Motor[chn][MOTORSETTINGS_STP]);
    
    rate+=motorSettings.Motor[chn][MOTORSETTINGS_STP];
    
    if(rate>motorSettings.Motor[chn][MOTORSETTINGS_EDP_H])
      rate=motorSettings.Motor[chn][MOTORSETTINGS_EDP_H];
    if(rate<motorSettings.Motor[chn][MOTORSETTINGS_STP])
      rate=motorSettings.Motor[chn][MOTORSETTINGS_STP];
    
    TPM_CnV_REG(TPM0_BASE_PTR,chn)=(uint16)(rate*13.75);//13.75=34375/(1e+6us/400hz) 
  }
}