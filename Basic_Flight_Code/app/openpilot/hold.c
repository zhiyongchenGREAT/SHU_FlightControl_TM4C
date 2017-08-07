#include "hold.h"

void hold()
{
  float temp;
  eeprom_read(1);
  
  temp = flow_distance / 1000.0 * UART_PIDadjust.Height_P 
  + flow_distance / 1000.0 * UART_PIDadjust.Height_D;

/* stabDesired.Throttle is the value from nrf command              */
  actuatorDesired.Throttle = stabDesired.Throttle - temp;
  
  if(actuatorDesired.Throttle>75)
    actuatorDesired.Throttle=75;
  if(actuatorDesired.Throttle<0)
    actuatorDesired.Throttle=0; 
}