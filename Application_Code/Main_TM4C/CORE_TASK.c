/*
************************************************************************************************************************
*                                                TI-TM4C Flight Control
*                                               SCIE/Shanghai University
*                                              
* File    : CORE_TASK.c
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
#include "CORE_TASK.h"


/*
************************************************************************************************************************
*                                               Flight routine
************************************************************************************************************************
*/
OS_TCB	FlightRoutineTCB;
CPU_STK	FLIGHT_ROUTINE_TASK_STK[FLIGHT_ROUTINE_STK_SIZE];
/*
************************************************************************************************************************
*                                               Flight flow
************************************************************************************************************************
*/
OS_TCB	FlightRoutineControlTCB;
CPU_STK	FLIGHT_ROUTINE_CONTROL_TASK_STK[FLIGHT_ROUTINE_CONTROL_STK_SIZE];
/*
************************************************************************************************************************
*                                               Ultra sonic senor
************************************************************************************************************************
*/
OS_TCB	FlightRoutineKS103TCB;
CPU_STK	FLIGHT_ROUTINE_KS103_TASK_STK[FLIGHT_ROUTINE_KS103_STK_SIZE];
/*
************************************************************************************************************************
*                                               Gimbal
************************************************************************************************************************
*/
OS_TCB	CameraTCB;
CPU_STK	CAMERA_TASK_STK[CAMERA_STK_SIZE];
/*
************************************************************************************************************************
*                                               Radio telemetry
************************************************************************************************************************
*/
OS_TCB	UARTReportTCB;
CPU_STK	UART_REPORT_TASK_STK[UART_REPORT_TASK_SIZE];
/*
************************************************************************************************************************
*                                               GCS instruction receiver
************************************************************************************************************************
*/
OS_TCB	UARTAdjustTCB;
CPU_STK	UART_ADJUST_TASK_STK[UART_ADJUST_TASK_SIZE];
/*
************************************************************************************************************************
*                                               Autopilot
************************************************************************************************************************
*/
OS_TCB	AUTOtestflight;
CPU_STK	AUTO_TEST_FLIGHT_TASK_STK[AUTO_TEST_FLIGHT_TASK_SIZE];
/*
************************************************************************************************************************
*                                               Auto takeoff
************************************************************************************************************************
*/
OS_TCB	AUTOtakeoff;
CPU_STK	AUTO_TAKEOFF_TASK_STK[AUTO_TAKEOFF_TASK_SIZE];
/*
************************************************************************************************************************
*                                               Auto landing
************************************************************************************************************************
*/
OS_TCB	AUTOlanding;
CPU_STK	AUTO_LANDIND_TASK_STK[AUTO_LANDIND_TASK_SIZE];
/*
************************************************************************************************************************
*                                               Remote controller
************************************************************************************************************************
*/
OS_TCB	RemoteCtrlTCB;
CPU_STK	REMOTE_CONTROLLER_TASK_STK[REMOTE_CONTROLLER_TASK_SIZE];
/*
************************************************************************************************************************
*                                               nrf task
************************************************************************************************************************
*/
OS_TCB	nrfTCB;
CPU_STK	NRF_TASK_STK[NRF_STK_SIZE];
/*
************************************************************************************************************************
*                                               attitude solving task
************************************************************************************************************************
*/
OS_TCB	AttitudesolvingTCB;
CPU_STK	ATTITUDE_SOLVING_TASK_STK[ATTITUDE_SOLVING_STK_SIZE];
/*
************************************************************************************************************************
*                                               auto goto task
************************************************************************************************************************
*/
OS_TCB	AUTOgoto;
CPU_STK	AUTO_GOTO_TASK_STK[AUTO_GOTO_TASK_SIZE];
/*
************************************************************************************************************************
*                                               renesas task
************************************************************************************************************************
*/
OS_TCB	RenesasTCB;
CPU_STK	RENESAS_TASK_STK[RENESAS_TASK_SIZE];
/*
========================================================================================================================
*                                               OS_Mutex/Sem
========================================================================================================================
*/
OS_MUTEX FLOW_MUTEX;
OS_MUTEX KS103_MUTEX;
OS_MUTEX PID_adjust_MUTEX;

OS_SEM TAKEOFF_SIG;
/*
========================================================================================================================
*                                               TASK 0
========================================================================================================================
*/

void TASK0_create(void)
{
  OS_ERR err; 
  OSTaskCreate((OS_TCB 	* )&FlightRoutineTCB,		
               (CPU_CHAR	* )"flight routine task", 		
               (OS_TASK_PTR  )flight_routine_task, 			
               (void	* )&COMPETITON_FLIGHT_MODE,					
               (OS_PRIO	  )FLIGHT_INIT_TASK_PRIO,     
               (CPU_STK    * )&FLIGHT_ROUTINE_TASK_STK[0],	
               (CPU_STK_SIZE )FLIGHT_ROUTINE_STK_SIZE/10,	
               (CPU_STK_SIZE )FLIGHT_ROUTINE_STK_SIZE,		
               (OS_MSG_QTY   )0,					
               (OS_TICK	  )0,					
               (void   	* )0,					
               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
               (OS_ERR 	* )&err);
  
//  OSTaskCreate((OS_TCB 	* )&UARTReportTCB,		
//               (CPU_CHAR	* )"uart report task", 		
//               (OS_TASK_PTR  )uart_report_task, 			
//               (void	* )0,					
//               (OS_PRIO	  )UART_REPORT_TASK_PRIO,     
//               (CPU_STK    * )&UART_REPORT_TASK_STK[0],	
//               (CPU_STK_SIZE )UART_REPORT_TASK_SIZE/10,	
//               (CPU_STK_SIZE )UART_REPORT_TASK_SIZE,		
//               (OS_MSG_QTY   )0,					
//               (OS_TICK	  )0,					
//               (void   	* )0,					
//               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
//               (OS_ERR 	* )&err);
  
//  OSTaskCreate((OS_TCB 	* )&UARTAdjustTCB,		
//               (CPU_CHAR	* )"uart adjust task", 		
//               (OS_TASK_PTR  )uart_adjust_task, 			
//               (void	* )0,					
//               (OS_PRIO	  )UART_ADJUST_TASK_PRIO,     
//               (CPU_STK    * )&UART_ADJUST_TASK_STK[0],	
//               (CPU_STK_SIZE )UART_ADJUST_TASK_SIZE/10,	
//               (CPU_STK_SIZE )UART_ADJUST_TASK_SIZE,		
//               (OS_MSG_QTY   )0,					
//               (OS_TICK	  )0,					
//               (void   	* )0,					
//               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
//               (OS_ERR 	* )&err);
  
//  OSTaskCreate((OS_TCB 	* )&RemoteCtrlTCB,		
//               (CPU_CHAR	* )"remote controller task", 		
//               (OS_TASK_PTR  )remote_controller_task, 			
//               (void	* )0,					
//               (OS_PRIO	  )REMOTE_CONTROLLER_TASK_PRIO,     
//               (CPU_STK    * )&REMOTE_CONTROLLER_TASK_STK[0],	
//               (CPU_STK_SIZE )REMOTE_CONTROLLER_TASK_SIZE/10,	
//               (CPU_STK_SIZE )REMOTE_CONTROLLER_TASK_SIZE,		
//               (OS_MSG_QTY   )REMOTE_CONTROLLER_TASK_MSG,					
//               (OS_TICK	  )0,					
//               (void   	* )0,					
//               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
//               (OS_ERR 	* )&err);  
  
//  OSTaskCreate((OS_TCB 	* )&AUTOtestflight,		
//               (CPU_CHAR	* )"auto test flight", 		
//               (OS_TASK_PTR  )auto_test_flight_task, 			
//               (void	* )0,					
//               (OS_PRIO	  )AUTO_TEST_FLIGHT_TASK_PRIO,     
//               (CPU_STK    * )&AUTO_TEST_FLIGHT_TASK_STK[0],	
//               (CPU_STK_SIZE )AUTO_TEST_FLIGHT_TASK_SIZE/10,	
//               (CPU_STK_SIZE )AUTO_TEST_FLIGHT_TASK_SIZE,		
//               (OS_MSG_QTY   )0,					
//               (OS_TICK	  )0,					
//               (void   	* )0,					
//               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
//               (OS_ERR 	* )&err);
  
//  OSTaskCreate((OS_TCB 	* )&AUTOtakeoff,		
//               (CPU_CHAR	* )"auto takeoff", 		
//               (OS_TASK_PTR  )auto_takeoff_task, 			
//               (void	* )0,					
//               (OS_PRIO	  )AUTO_TAKEOFF_TASK_PRIO,     
//               (CPU_STK    * )&AUTO_TAKEOFF_TASK_STK[0],	
//               (CPU_STK_SIZE )AUTO_TAKEOFF_TASK_SIZE/10,	
//               (CPU_STK_SIZE )AUTO_TAKEOFF_TASK_SIZE,		
//               (OS_MSG_QTY   )0,					
//               (OS_TICK	  )0,					
//               (void   	* )0,					
//               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
//               (OS_ERR 	* )&err);
  
//  OSTaskCreate((OS_TCB 	* )&AUTOgoto,		
//               (CPU_CHAR	* )"auto goto", 		
//               (OS_TASK_PTR  )auto_goto_task, 			
//               (void	* )0,					
//               (OS_PRIO	  )AUTO_GOTO_TASK_PRIO,     
//               (CPU_STK    * )&AUTO_GOTO_TASK_STK[0],	
//               (CPU_STK_SIZE )AUTO_GOTO_TASK_SIZE/10,	
//               (CPU_STK_SIZE )AUTO_GOTO_TASK_SIZE,		
//               (OS_MSG_QTY   )AUTO_GOTO_TASK_MSG,					
//               (OS_TICK	  )0,					
//               (void   	* )0,					
//               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
//               (OS_ERR 	* )&err);
//  
//  OSTaskCreate((OS_TCB 	* )&AUTOlanding,		
//               (CPU_CHAR	* )"auto landing", 		
//               (OS_TASK_PTR  )auto_landing_task, 			
//               (void	* )0,					
//               (OS_PRIO	  )AUTO_LANDING_TASK_PRIO,     
//               (CPU_STK    * )&AUTO_LANDIND_TASK_STK[0],	
//               (CPU_STK_SIZE )AUTO_LANDIND_TASK_SIZE/10,	
//               (CPU_STK_SIZE )AUTO_LANDIND_TASK_SIZE,		
//               (OS_MSG_QTY   )0,					
//               (OS_TICK	  )0,					
//               (void   	* )0,					
//               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
//               (OS_ERR 	* )&err);
//  
//  OSTaskCreate((OS_TCB 	* )&RenesasTCB,		
//               (CPU_CHAR	* )"renesas interface task", 		
//               (OS_TASK_PTR  )renesas_interface, 			
//               (void	* )0,					
//               (OS_PRIO	  )RENESAS_TASK_PRIO,     
//               (CPU_STK    * )&RENESAS_TASK_STK[0],	
//               (CPU_STK_SIZE )RENESAS_TASK_SIZE/10,	
//               (CPU_STK_SIZE )RENESAS_TASK_SIZE,		
//               (OS_MSG_QTY   )0,					
//               (OS_TICK	  )0,					
//               (void   	* )0,					
//               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
//               (OS_ERR 	* )&err);
  
//  OSMutexCreate(&FLOW_MUTEX,
//                "flow mutex",
//                &err);
//  
//  OSMutexCreate(&KS103_MUTEX,
//                "ks103 mutex",
//                &err);
  
//  OSMutexCreate(&PID_adjust_MUTEX,
//                "PID adjust mutex",
//                &err); 

//  OSSemCreate(&TAKEOFF_SIG, 
//              "takeoff signal",
//              (OS_SEM_CTR)0,
//              &err);
}