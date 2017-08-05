#include "hold.h"


bool lowthrottle=false;
float lasts=0;//上次面积

void hold()
{
//  OS_ERR err;  
//  CPU_TS ts;
  
//  OSMutexPend(&PID_adjust_MUTEX,
//              0,
//              OS_OPT_PEND_BLOCKING,
//              &ts,
//              &err);
//  
  float temp;
  eeprom_read(1);

  
  if(abs((int)flow_delta_distance) < 200)
    temp = flow_distance / 1000.0 * UART_PIDadjust.Height_P 
      + flow_distance / 1000.0 * UART_PIDadjust.Height_D;
  else
    temp = lasts;
  
  actuatorDesired.Throttle = stabDesired.Throttle - temp;	                //stabDesired.Throttle is the value from nrf command
  
  
  if(actuatorDesired.Throttle>75)                                               //set 70 original
  {
    actuatorDesired.Throttle=75;
    lowthrottle=true;
  }
  else
  {
    lowthrottle=false;
  }
  if(actuatorDesired.Throttle<0)
    actuatorDesired.Throttle=0; 
  
  lasts = temp;
  
//  OSMutexPost(&KS103_MUTEX,
//              OS_OPT_POST_NONE,
//              &err);  
}