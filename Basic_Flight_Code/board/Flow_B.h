/*
************************************************************************************************************************
*                                                TI-TM4C Flight Control
*                                               SCIE/Shanghai University
*                                              
* File    : Flow_B.h
* By      : Bicbrv
* Note    : Flow improved version
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


#ifndef _Flow_B_H
#define _Flow_B_H

#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>

extern float SumX_amend,SumY_amend;

extern void PX4Flow_uart_init(uint32 band,void (*pfnHandler)(void));
extern void FLOW_MAVLINK(unsigned char data);
extern void px4_data_fix(void);

extern float flow_distance, flow_delta_distance;
/* test              */
extern float Xmm_Send;
extern float Ymm_Send;
/* test              */

typedef struct
{
  float average;//Flow in m in x-sensor direction, angular-speed compensated
  float originf;
  int16_t origin;
}FLOW_DATA;


typedef struct
{
  uint64_t  time_sec;
  uint8   id;
  FLOW_DATA flow_x;
  FLOW_DATA flow_y;
  FLOW_DATA flow_comp_x;//Flow in m in x-sensor direction, angular-speed compensated
  FLOW_DATA flow_comp_y;
  uint8 quality; //Optical flow quality / confidence. 0: bad, 255: maximum quality
  FLOW_DATA hight;//ground_distance        float        Ground distance in m. Positive value: distance known. Negative value: Unknown distance               
}FLOW;

typedef struct
{
  uint64_t time_usec; ///< Timestamp (microseconds, synced to UNIX time or since system boot)
  uint32_t integration_time_us; ///< Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
  float integrated_x; ///< Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
  float integrated_y; ///< Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
  float integrated_xgyro; ///< RH rotation around X axis (rad)
  float integrated_ygyro; ///< RH rotation around Y axis (rad)
  float integrated_zgyro; ///< RH rotation around Z axis (rad)
  uint32_t time_delta_distance_us; ///< Time in microseconds since the distance was sampled.
  float distance; ///< Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance.
  int16_t temperature; ///< Temperature * 100 in centi-degrees Celsius
  uint8_t sensor_id; ///< Sensor ID 
  uint8_t quality; ///< Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
}FLOW_RAD;

typedef struct
{
  FLOW_DATA flow_x;
  FLOW_DATA flow_y;
  FLOW_DATA flow_comp_x;//Flow in m in x-sensor direction, angular-speed compensated
  FLOW_DATA flow_comp_y;
  float scale_rad_fix;
  float scale_rad_fix_comp;
  FLOW_DATA hight;//ground_distance        float        Ground distance in m. Positive value: distance known. Negative value: Unknown distance               
}FLOW_FIX;

#endif //Flow_B_H_
