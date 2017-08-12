#ifndef __CORE_AUTOPILOT_H__
#define __CORE_AUTOPILOT_H__

#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>

extern float auto_throttle, error_throttle;
extern float control_y_out, control_x_out;

extern int16 goto_count;
extern uint8 stablization_mode;
extern uint8 land_flag;

#endif //__CORE_AUTOPILOT_H__