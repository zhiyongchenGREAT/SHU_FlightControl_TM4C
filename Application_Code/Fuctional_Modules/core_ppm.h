/*
************************************************************************************************************************
*                                                TI-TM4C Flight Control
*                                               SCIE/Shanghai University
*                                              
* File    : core_ppm.h
* By      : Bicbrv
* Note    : Remote controller
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
#ifndef __CORE_PPM_H
#define __CORE_PPM_H

#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>

extern volatile uint32 CAP_count;
extern void PPM_init(void (*pfnHandler)(void));
extern void PPM_CAP_Int_Handler(void);

extern void remote_controller_task(void *p_arg);

#endif  //__MAIN_H__