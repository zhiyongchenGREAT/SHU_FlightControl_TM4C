/*
************************************************************************************************************************
*                                                TI-TM4C Flight Control
*                                               SCIE/Shanghai University
*                                              
* File    : control_command_B.c
* By      : Bicbrv
* Note    : Control command
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

#include "control_command_B.h"

#define fix_pitch 0.57f
#define fix_roll  0.4f

static uint16 arming=0;
static float Yaw_con_before=0;
static float Yaw_fixed=0;


void command_handler()                                                          
{
  OS_ERR err;
  
  stabDesired.Mode=(Nrf_in_switch[4]/30.0);
  
  if(IMU_ext_flag==1)
    stabDesired.Mode=0;
  else if(IMU_ext_flag==2)
     stabDesired.Mode=0;
   else if(IMU_ext_flag==3)
     stabDesired.Mode=118;
  
/* approximately 2~105; seems these value are all mannually bound within 0~100                */

  stabDesired.Throttle=-(Nrf_in_switch[2])/30.0 + auto_throttle;
//  stabDesired.Throttle=(Nrf_in_switch[2])/30.0 + auto_throttle;
  stabDesired.Throttle=stabDesired.Throttle<0?0:stabDesired.Throttle;
  if(stablization_mode)
  {
    if(control_flag>0)
    {
      stabDesired.Pitch=(Nrf_in_switch[0])/30.0 - Pic_cotrol_xout+fix_pitch;//control_x_out; //+ angle_x_out ;
      stabDesired.Roll=(Nrf_in_switch[1])/30.0 + Pic_cotrol_yout-fix_roll;//control_y_out; //+ angle_y_out ;
      control_flag--;
    }
    else if(control_flag==0)
    {
      stabDesired.Pitch=(Nrf_in_switch[0])/30.0 - Pic_x_out+fix_pitch;//control_x_out; //+ angle_x_out ;
      stabDesired.Roll=(Nrf_in_switch[1])/30.0 + Pic_y_out-fix_roll;//control_y_out; //+ angle_y_out ;
      
    }
  }
  else if(!stablization_mode)
  {
    stabDesired.Pitch=(Nrf_in_switch[0])/30.0 + control_x_out+fix_pitch; //+ angle_x_out ;
    stabDesired.Roll=(Nrf_in_switch[1])/30.0 + control_y_out-fix_roll; //+ angle_y_out ;
  }
  
 
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
      if(arming>50)
      {  
        flightStatus.Armed=FLIGHTSTATUS_ARMED_ARMED; 
        OSSemPost(&TAKEOFF_SIG,
                  OS_OPT_POST_1,
                  &err);
      }
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
      arming=51;
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
      if(arming<10)
      {
        flightStatus.Armed=FLIGHTSTATUS_ARMED_DISARMED;
 /* Delete all auto task here              */
 
      }
      break;
    }
  }
}