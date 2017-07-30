#include "hold.h"


bool lowthrottle=false;
float lasts=0;//上次面积

void hold()
{
  float temp;
  eeprom_read(1);
//defined in data_common.c/data_common.h:
//struct ActuatorDesiredData actuatorDesired;
//struct StabilizationDesiredData stabDesired;
  
  if(abs((int)ks103_delta_distance) < 200)
      temp = ks103_distance / 1000.0 * 20 + ks103_delta_distance / 1000.0 * 90;     //PD para of height
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
}