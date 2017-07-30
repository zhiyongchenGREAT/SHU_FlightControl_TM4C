#ifndef Postion_Hold_H
#define Postion_Hold_H
#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>


void Control();
extern float tot_x_cm,tot_y_cm,last_tot_x_cm,last_tot_y_cm;
extern float angle_x_out,angle_y_out;
extern float flow_p,flow_i,flow_d;
extern float pos_x_i,pos_y_i;
extern bool pid_switch;

#endif // ACCELS_H