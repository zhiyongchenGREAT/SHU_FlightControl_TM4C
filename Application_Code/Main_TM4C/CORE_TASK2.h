/*
************************************************************************************************************************
*                                                TI-TM4C Flight Control
*                                               SCIE/Shanghai University
*                                              
* File    : CORE_TASK2.h
* By      : Bicbrv
* Note    : task2 create
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

#ifndef __OS_TASK2_H
#define __OS_TASK2_H

#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>

/*
************************************************************************************************************************
*                                               Auto takeoff task2
************************************************************************************************************************
*/
#define AUTO_TAKEOFF_T2_TASK_PRIO        7
#define AUTO_TAKEOFF_T2_TASK_SIZE       128
extern OS_TCB	AUTOtakeoff_T2;
extern CPU_STK	AUTO_TAKEOFF_T2_TASK_STK[AUTO_TAKEOFF_T2_TASK_SIZE];
void auto_takeoff_t2_task(void *p_arg);
/*
************************************************************************************************************************
*                                               Auto landing task2
************************************************************************************************************************
*/
#define AUTO_LANDING_T2_TASK_PRIO        7
#define AUTO_LANDIND_T2_TASK_SIZE       128
extern OS_TCB	AUTOlanding_T2;
extern CPU_STK	AUTO_LANDIND_T2_TASK_STK[AUTO_LANDIND_T2_TASK_SIZE];
void auto_landing_t2_task(void *p_arg);

/*
************************************************************************************************************************
*                                               auto goto task2
************************************************************************************************************************
*/
#define AUTO_GOTO_T2_TASK_PRIO        7
#define AUTO_GOTO_T2_TASK_SIZE       128
extern OS_TCB	AUTOgoto_T2;
extern CPU_STK	AUTO_GOTO_T2_TASK_STK[AUTO_GOTO_T2_TASK_SIZE];
void auto_goto_t2_task(void *p_arg);

/*
========================================================================================================================
*                                               Function Prototypes
========================================================================================================================
*/

extern void TASK2_create(void);


#endif //__OS_TASK2_H