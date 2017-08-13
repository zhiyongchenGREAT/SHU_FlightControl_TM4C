/*
************************************************************************************************************************
*                                                TI-TM4C Flight Control
*                                               SCIE/Shanghai University
*                                              
* File    : CORE_TASK1.h
* By      : Bicbrv
* Note    : task1 create
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

#ifndef __OS_TASK1_H
#define __OS_TASK1_H

#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>

/*
************************************************************************************************************************
*                                               Auto takeoff task1
************************************************************************************************************************
*/
#define AUTO_TAKEOFF_T1_TASK_PRIO        7
#define AUTO_TAKEOFF_T1_TASK_SIZE       128
extern OS_TCB	AUTOtakeoff_T1;
extern CPU_STK	AUTO_TAKEOFF_T1_TASK_STK[AUTO_TAKEOFF_T1_TASK_SIZE];
void auto_takeoff_t1_task(void *p_arg);
/*
************************************************************************************************************************
*                                               Auto landing task1
************************************************************************************************************************
*/
#define AUTO_LANDING_T1_TASK_PRIO        7
#define AUTO_LANDIND_T1_TASK_SIZE       128
extern OS_TCB	AUTOlanding_T1;
extern CPU_STK	AUTO_LANDIND_T1_TASK_STK[AUTO_LANDIND_T1_TASK_SIZE];
void auto_landing_t1_task(void *p_arg);

/*
************************************************************************************************************************
*                                               auto goto task1
************************************************************************************************************************
*/
#define AUTO_GOTO_T1_TASK_PRIO        7
#define AUTO_GOTO_T1_TASK_SIZE       128
extern OS_TCB	AUTOgoto_T1;
extern CPU_STK	AUTO_GOTO_T1_TASK_STK[AUTO_GOTO_T1_TASK_SIZE];
void auto_goto_t1_task(void *p_arg);

/*
========================================================================================================================
*                                               Function Prototypes
========================================================================================================================
*/

extern void TASK1_create(void);


#endif //__OS_TASK1_H