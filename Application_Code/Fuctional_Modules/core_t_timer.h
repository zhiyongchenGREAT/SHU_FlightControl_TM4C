/*
************************************************************************************************************************
*                                                TI-TM4C Flight Control
*                                               SCIE/Shanghai University
*                                              
* File    : core_t_timer.h
* By      : Bicbrv
* Note    : Test purpose GPTM
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

#ifndef __CORE_T_TIMER_H__
#define __CORE_T_TIMER_H__

#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>

extern void testpurpose_tim0_init(void);
extern void tim3_init(void (*pfnHandler)(void));

extern volatile CPU_INT32U t_tim0_cnt;

#endif //__CORE_T_TIMER_H__