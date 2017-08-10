#ifndef __PIC_HOLD_H
#define __PIC_HOLD_H

#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>


void PIC_Control();

extern float Pic_x_out,Pic_y_out;
extern float pic_x_cm,pic_y_cm;


#endif //__PIC_HOLD_H