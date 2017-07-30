/*
************************************************************************************************************************
*                                               need optimization
************************************************************************************************************************
*/

#ifndef _PX4Flow_H
#define _PX4Flow_H

#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>

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


void PX4Flow_uart_init(uint32 band,void (*pfnHandler)(void));
void FLOW_MAVLINK(unsigned char data);
void px4_data_fix(void);

extern float x1,y1;
extern uint8 flow_buf[27];

extern float SumX;
extern float SumY;
//飞机当前坐标
extern float PosX;
extern float PosY;
extern float SumX_amend,SumY_amend;
extern float global_x,global_y;
#endif
