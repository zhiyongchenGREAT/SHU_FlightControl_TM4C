#include "hold.h"

static float lasts=0;

void hold()
{
  float temp;
  eeprom_read(1);
  
//  temp = ks103_distance / 1000.0 * UART_PIDadjust.Height_P 
//  + ks103_delta_distance / 1000.0 * UART_PIDadjust.Height_D;

  if((abs((int)ks103_delta_distance) < 200) && (ks103_distance < 2000))
    temp = ks103_distance / 1000.0 * UART_PIDadjust.Height_P 
      + ks103_delta_distance / 1000.0 * UART_PIDadjust.Height_D;
  else
    temp = lasts;

  lasts = temp;  
  
/* stabDesired.Throttle is the value from nrf command              */
  actuatorDesired.Throttle = stabDesired.Throttle - temp;
  
  if(actuatorDesired.Throttle>75)
    actuatorDesired.Throttle=75;
  if(actuatorDesired.Throttle<0)
    actuatorDesired.Throttle=0; 
}