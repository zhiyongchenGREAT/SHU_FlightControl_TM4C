#include "control_command.h"

static uint16 arming=0;
static float Yaw_con_before=0;
static float Yaw_fixed=0;

void command_handler()                                                          
{
  
  stabDesired.Mode=(Nrf_in_switch[4]/30.0);
  
  if(IMU_ext_flag==1)
    stabDesired.Mode=0;
  
/* approximately 2~105; seems these value are all mannually bound within 0~100                */

  stabDesired.Throttle=(Nrf_in_switch[2])/30.0 + auto_throttle;
  stabDesired.Pitch=Nrf_in_switch[0]/30.0 + angle_x_out;
  stabDesired.Roll=Nrf_in_switch[1]/30.0 + angle_y_out;
  if(fabs(Nrf_in_switch[3]/30.0)>5)
    stabDesired.Yaw=Nrf_in_switch[3]/30.0;
  else
    stabDesired.Yaw=Yaw_fixed;

  if(fabs(Nrf_in_switch[3]/30.0)<=5 && fabs(Yaw_con_before)>5)
    Yaw_fixed=attitudeActual.Yaw;
  
  Yaw_con_before=Nrf_in_switch[3]/30.0;

/* DS1 indicate whether the flight lock has been presented: ON unlock OFF lock              */
  
  if(stabDesired.Mode>25)
  {
    LED1_ON();                                                                  
    switch(flightStatus.Armed)
    {
    case FLIGHTSTATUS_ARMED_DISARMED:
      flightStatus.Armed=FLIGHTSTATUS_ARMED_ARMING;
      arming=0;
      break;
    case FLIGHTSTATUS_ARMED_ARMING:
      arming++;
//      if(arming>50)
    if(arming>5)
        flightStatus.Armed=FLIGHTSTATUS_ARMED_ARMED; 
      break;
    case FLIGHTSTATUS_ARMED_ARMED:
      if(stabDesired.Mode<50)
      {
        stabDesired.StabilizationMode[0]=STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
        stabDesired.StabilizationMode[1]=STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
        stabDesired.StabilizationMode[2]=STABILIZATIONDESIRED_STABILIZATIONMODE_RATE;
        stabDesired.StabilizationMode[3]=STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDEHOLD;
      }

/* The mode we use, we have .Mode==15 or .Mode==100              */
      
      else if(stabDesired.Mode<125)                                                                
      {
        stabDesired.StabilizationMode[0]=STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
        stabDesired.StabilizationMode[1]=STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
        if(fabs(Nrf_in_switch[3]/30.0)>5)
          stabDesired.StabilizationMode[2]=STABILIZATIONDESIRED_STABILIZATIONMODE_RATE;
        else
          stabDesired.StabilizationMode[2]=STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
        stabDesired.StabilizationMode[3]=STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDEHOLD;
      }
      else
      {
        stabDesired.StabilizationMode[0]=STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
        stabDesired.StabilizationMode[1]=STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
        stabDesired.StabilizationMode[2]=STABILIZATIONDESIRED_STABILIZATIONMODE_RATE;
        stabDesired.StabilizationMode[3]=STABILIZATIONDESIRED_STABILIZATIONMODE_RATE;
      }
//      arming=51;
      arming=6;
      break;
    }
    
  }
  else
  {
    LED1_OFF();
    switch(flightStatus.Armed)
    {
    case FLIGHTSTATUS_ARMED_DISARMED:
      arming=0;
      break;
    case FLIGHTSTATUS_ARMED_ARMING:
      arming=0;
      flightStatus.Armed=FLIGHTSTATUS_ARMED_DISARMED; 
      break;
    case FLIGHTSTATUS_ARMED_ARMED:
      arming--;
//      if(arming<10)
      if(arming<2)   
        flightStatus.Armed=FLIGHTSTATUS_ARMED_DISARMED;
      break;
    }
  }
}