/*
************************************************************************************************************************
*                                                TI-TM4C Flight Control
*                                               SCIE/Shanghai University
*                                              
* File    : CORE_TASK3.h
* By      : Bicbrv
* Note    : task3 create
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

#ifndef __OS_TASK3_H
#define __OS_TASK3_H

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
#define AUTO_TAKEOFF_T3_TASK_PRIO        7
#define AUTO_TAKEOFF_T3_TASK_SIZE       128
extern OS_TCB	AUTOtakeoff_T3;
extern CPU_STK	AUTO_TAKEOFF_T3_TASK_STK[AUTO_TAKEOFF_T3_TASK_SIZE];
void auto_takeoff_t3_task(void *p_arg);
/*
************************************************************************************************************************
*                                               Auto landing task2
************************************************************************************************************************
*/
#define AUTO_LANDING_T3_TASK_PRIO        7
#define AUTO_LANDIND_T3_TASK_SIZE       128
extern OS_TCB	AUTOlanding_T3;
extern CPU_STK	AUTO_LANDIND_T3_TASK_STK[AUTO_LANDIND_T3_TASK_SIZE];
void auto_landing_t3_task(void *p_arg);

/*
************************************************************************************************************************
*                                               auto goto task3
************************************************************************************************************************
*/
#define AUTO_GOTO_T3_TASK_PRIO        7
#define AUTO_GOTO_T3_TASK_SIZE       128
extern OS_TCB	AUTOgoto_T3;
extern CPU_STK	AUTO_GOTO_T3_TASK_STK[AUTO_GOTO_T3_TASK_SIZE];
void auto_goto_t3_task(void *p_arg);

/*
========================================================================================================================
*                                               Function Prototypes
========================================================================================================================
*/

extern void TASK3_create(void);


#endif //__OS_TASK3_H