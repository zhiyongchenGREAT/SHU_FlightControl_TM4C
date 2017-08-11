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

static uint16 arming=0;
static float Yaw_con_before=0;
static float Yaw_fixed=0;

void command_handler()                                                          
{
  OS_ERR err;
  
  stabDesired.Mode=(Nrf_in_switch[4]/30.0);
  
  if(IMU_ext_flag==1)
    stabDesired.Mode=0;
  
/* approximately 2~105; seems these value are all mannually bound within 0~100                */

  stabDesired.Throttle=-(Nrf_in_switch[2])/30.0 + auto_throttle;
//  stabDesired.Throttle=(Nrf_in_switch[2])/30.0 + auto_throttle;
  stabDesired.Throttle=stabDesired.Throttle<0?0:stabDesired.Throttle;  
  
  if(control_flag>0)
  {
     stabDesired.Pitch=Nrf_in_switch[0]/30.0 - Pic_cotrol_xout;//control_x_out; //+ angle_x_out ;
     stabDesired.Roll=Nrf_in_switch[1]/30.0 + Pic_cotrol_yout;//control_y_out; //+ angle_y_out ;
     control_flag--;
  }
  else if(control_flag==0)
  {
     stabDesired.Pitch=Nrf_in_switch[0]/30.0 - Pic_x_out;//control_x_out; //+ angle_x_out ;
     stabDesired.Roll=Nrf_in_switch[1]/30.0 + Pic_y_out;//control_y_out; //+ angle_y_out ;
    
  }  
  
//  stabDesired.Pitch=Nrf_in_switch[0]/30.0 - Pic_x_out;//control_x_out; //+ angle_x_out ;
//  stabDesired.Roll=Nrf_in_switch[1]/30.0 + Pic_y_out;//control_y_out; //+ angle_y_out ;
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
        OSTaskCreate((OS_TCB 	* )&AUTOtakeoff,		
                     (CPU_CHAR	* )"auto takeoff", 		
                     (OS_TASK_PTR  )auto_takeoff_task, 			
                     (void	* )0,					
                     (OS_PRIO	  )AUTO_TAKEOFF_TASK_PRIO,     
                     (CPU_STK    * )&AUTO_TAKEOFF_TASK_STK[0],	
                     (CPU_STK_SIZE )AUTO_TAKEOFF_TASK_SIZE/10,	
                     (CPU_STK_SIZE )AUTO_TAKEOFF_TASK_SIZE,		
                     (OS_MSG_QTY   )0,					
                     (OS_TICK	  )0,					
                     (void   	* )0,					
                     (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                     (OS_ERR 	* )&err);      
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
        flightStatus.Armed=FLIGHTSTATUS_ARMED_DISARMED;
      break;
    }
  }
}