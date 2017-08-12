/*
************************************************************************************************************************
*                                                TI-TM4C Flight Control
*                                               SCIE/Shanghai University
*                                              
* File    : CORE_TASK.h
* By      : Bicbrv
* Note    : task0 create
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

#ifndef __CORE_TASK_H
#define __CORE_TASK_H

#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>


/*
************************************************************************************************************************
*                                               Flight routine
************************************************************************************************************************
*/
#define FLIGHT_ROUTINE_TASK_PRIO      4
#define FLIGHT_ROUTINE_STK_SIZE       1536
extern OS_TCB	FlightRoutineTCB;
extern CPU_STK	FLIGHT_ROUTINE_TASK_STK[FLIGHT_ROUTINE_STK_SIZE];
void flight_routine_task(void *p_arg);
/*
************************************************************************************************************************
*                                               Flight flow
************************************************************************************************************************
*/
#define FLIGHT_ROUTINE_CONTROL_TASK_PRIO      5
#define FLIGHT_ROUTINE_CONTROL_STK_SIZE       128
extern OS_TCB	FlightRoutineControlTCB;
extern CPU_STK	FLIGHT_ROUTINE_CONTROL_TASK_STK[FLIGHT_ROUTINE_CONTROL_STK_SIZE];
void flight_routine_control_task(void *p_arg);
/*
************************************************************************************************************************
*                                               Ultra sonic senor
************************************************************************************************************************
*/
#define FLIGHT_ROUTINE_KS103_TASK_PRIO        6
#define FLIGHT_ROUTINE_KS103_STK_SIZE       128
extern OS_TCB	FlightRoutineKS103TCB;
extern CPU_STK	FLIGHT_ROUTINE_KS103_TASK_STK[FLIGHT_ROUTINE_KS103_STK_SIZE];
void flight_routine_ks103_task(void *p_arg);

/*
************************************************************************************************************************
*                                               Gimbal
************************************************************************************************************************
*/
#define CAMERA_TASK_PRIO        11
#define CAMERA_STK_SIZE       128
extern OS_TCB	CameraTCB;
extern CPU_STK	CAMERA_TASK_STK[CAMERA_STK_SIZE];
void camera_task(void *p_arg);
/*
************************************************************************************************************************
*                                               Radio telemetry
************************************************************************************************************************
*/
#define UART_REPORT_TASK_PRIO        10
#define UART_REPORT_TASK_SIZE       128
extern OS_TCB	UARTReportTCB;
extern CPU_STK	UART_REPORT_TASK_STK[UART_REPORT_TASK_SIZE];
void uart_report_task(void *p_arg);
/*
************************************************************************************************************************
*                                               GCS instruction receiver
************************************************************************************************************************
*/ 
#define UART_ADJUST_TASK_PRIO        9
#define UART_ADJUST_TASK_SIZE       128
extern OS_TCB	UARTAdjustTCB;
extern CPU_STK	UART_ADJUST_TASK_STK[UART_ADJUST_TASK_SIZE];
void uart_adjust_task(void *p_arg);
/*
************************************************************************************************************************
*                                               Autopilot
************************************************************************************************************************
*/
#define AUTO_TEST_FLIGHT_TASK_PRIO        7
#define AUTO_TEST_FLIGHT_TASK_SIZE       128
extern OS_TCB	AUTOtestflight;
extern CPU_STK	AUTO_TEST_FLIGHT_TASK_STK[AUTO_TEST_FLIGHT_TASK_SIZE];
void auto_test_flight_task(void *p_arg);
/*
************************************************************************************************************************
*                                               Auto takeoff
************************************************************************************************************************
*/
#define AUTO_TAKEOFF_TASK_PRIO        7
#define AUTO_TAKEOFF_TASK_SIZE       128
extern OS_TCB	AUTOtakeoff;
extern CPU_STK	AUTO_TAKEOFF_TASK_STK[AUTO_TAKEOFF_TASK_SIZE];
void auto_takeoff_task(void *p_arg);
/*
************************************************************************************************************************
*                                               Auto landing
************************************************************************************************************************
*/
#define AUTO_LANDING_TASK_PRIO        7
#define AUTO_LANDIND_TASK_SIZE       128
extern OS_TCB	AUTOlanding;
extern CPU_STK	AUTO_LANDIND_TASK_STK[AUTO_LANDIND_TASK_SIZE];
void auto_landing_task(void *p_arg);
/*
************************************************************************************************************************
*                                               Remote controller
************************************************************************************************************************
*/
#define REMOTE_CONTROLLER_TASK_PRIO        5
#define REMOTE_CONTROLLER_TASK_SIZE       128
#define REMOTE_CONTROLLER_TASK_MSG      10
extern OS_TCB	RemoteCtrlTCB;
extern CPU_STK	REMOTE_CONTROLLER_TASK_STK[REMOTE_CONTROLLER_TASK_SIZE];
void remote_controller_task(void *p_arg);
/*
************************************************************************************************************************
*                                               nrf task
************************************************************************************************************************
*/
#define NRF_TASK_PRIO        6
#define NRF_STK_SIZE       128
extern OS_TCB	nrfTCB;
extern CPU_STK	NRF_TASK_STK[NRF_STK_SIZE];
void nrf_task(void *p_arg);
/*
************************************************************************************************************************
*                                               attitude task
************************************************************************************************************************
*/
#define ATTITUDE_SOLVING_PRIO        3
#define ATTITUDE_SOLVING_STK_SIZE       128
#define ATTITUDE_SOLVING_TASK_MSG       10 
extern OS_TCB	AttitudesolvingTCB;
extern CPU_STK	ATTITUDE_SOLVING_TASK_STK[ATTITUDE_SOLVING_STK_SIZE];
void attitude_solving_task(void *p_arg);
/*
************************************************************************************************************************
*                                               auto goto task
************************************************************************************************************************
*/
#define AUTO_GOTO_TASK_PRIO        7
#define AUTO_GOTO_TASK_SIZE       128
#define AUTO_GOTO_TASK_MSG       10 
extern OS_TCB	AUTOgoto;
extern CPU_STK	AUTO_GOTO_TASK_STK[AUTO_GOTO_TASK_SIZE];
void auto_goto_task(void *p_arg);
/*
************************************************************************************************************************
*                                               Renesas task
************************************************************************************************************************
*/
#define RENESAS_TASK_PRIO        3
#define RENESAS_TASK_SIZE       128
extern OS_TCB	RenesasTCB;
extern CPU_STK	RENESAS_TASK_STK[RENESAS_TASK_SIZE];
void renesas_interface(void *p_arg);

/*
========================================================================================================================
*                                               OS_Mutex/Sem
========================================================================================================================
*/
extern OS_MUTEX FLOW_MUTEX;
extern OS_MUTEX KS103_MUTEX;
extern OS_MUTEX PID_adjust_MUTEX;

extern OS_SEM TAKEOFF_SIG;
/*
========================================================================================================================
*                                               Function Prototypes
========================================================================================================================
*/

extern void TASK0_create(void);


#endif //__CORE_TASK_H