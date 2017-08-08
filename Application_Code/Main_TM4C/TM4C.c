/*
************************************************************************************************************************
*                                                TI-TM4C Flight Control
*                                               SCIE/Shanghai University
*                                              
* File    : TM4C.c
* By      : Bicbrv
* Note    : TM4C Main Entrance
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
#include "TM4C.h"
/*
************************************************************************************************************************
*                                               Start task
************************************************************************************************************************
*/
static OS_TCB	StartTaskTCB;
static CPU_STK	START_TASK_STK[START_STK_SIZE];
/*
************************************************************************************************************************
*                                               Flight init module
************************************************************************************************************************
*/
static OS_TCB	FlightINITTCB;
static CPU_STK	FLIGHT_INIT_TASK_STK[FLIGHT_INIT_STK_SIZE];
CPU_INT08U flight_init_task_parg = MOTORRESET;
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
========================================================================================================================
*                                               OS_Mutex
========================================================================================================================
*/
OS_MUTEX FLOW_MUTEX;
OS_MUTEX KS103_MUTEX;
//OS_MUTEX PID_adjust_MUTEX;
/*
========================================================================================================================
*                                               Functions Entrance
========================================================================================================================
*/
int main(void)
{
  OS_ERR err;
  
  IntMasterDisable();
/* OS related int registration              */
  IntRegister(FAULT_PENDSV,OS_CPU_PendSVHandler);       
  IntRegister(FAULT_SYSTICK,OS_CPU_SysTickHandler);
  
  OSInit(&err);		                                                        
 
  OSTaskCreate((OS_TCB 	* )&StartTaskTCB,
               (CPU_CHAR	* )"start task",
               (OS_TASK_PTR )start_task,
               (void		* )0,
               (OS_PRIO	  )START_TASK_PRIO,
               (CPU_STK   * )&START_TASK_STK[0],
               (CPU_STK_SIZE)START_STK_SIZE/10,
               (CPU_STK_SIZE)START_STK_SIZE,
               (OS_MSG_QTY  )0,
               (OS_TICK	  )0,
               (void   	* )0,
               (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
               (OS_ERR 	* )&err);
  
  OSStart(&err);

  while(1);
}

/* Start task              */
static void start_task(void *p_arg)
{
  CPU_INT32U  cpu_clk_freq;	
  CPU_INT32U  cnts;  
  OS_ERR err;
  p_arg = p_arg;
  
  BSP_Init();
  CPU_Init();
  Mem_Init();
/* Determine SysTick reference freq              */  
  cpu_clk_freq = BSP_SysClkFreqGet();   
/* Determine nbr SysTick increments              */
  cnts         = cpu_clk_freq / (CPU_INT32U)OSCfg_TickRate_Hz;    
  OS_CPU_SysTickInit(cnts);	  
/* Enable stat task              */  
#if OS_CFG_STAT_TASK_EN > 0u    
  OSStatTaskCPUUsageInit(&err);	                                                       
#endif
/* Interrupt measurement              */  
#ifdef CPU_CFG_INT_DIS_MEAS_EN
  CPU_IntDisMeasMaxCurReset();	
#endif
  
#if     OS_CFG_SCHED_ROUND_ROBIN_EN                                             
  OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif		
 
  FlightAPPInit(MOTORRESET|NRF);
  
  OSTaskCreate((OS_TCB 	* )&FlightINITTCB,		
               (CPU_CHAR	* )"flight hardware initialization task", 		
               (OS_TASK_PTR  )flight_init_task, 			
               (void	* )0,					
               (OS_PRIO	  )FLIGHT_INIT_TASK_PRIO,     
               (CPU_STK    * )&FLIGHT_INIT_TASK_STK[0],	
               (CPU_STK_SIZE )FLIGHT_INIT_STK_SIZE/10,	
               (CPU_STK_SIZE )FLIGHT_INIT_STK_SIZE,		
               (OS_MSG_QTY   )0,					
               (OS_TICK	  )0,					
               (void   	* )0,					
               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
               (OS_ERR 	* )&err); 

  OSTaskDel(&StartTaskTCB, &err);
}


static void flight_init_task(void *p_arg)
{
  OS_ERR err; 
  p_arg = p_arg; 
  
  OSTaskCreate((OS_TCB 	* )&FlightRoutineTCB,		
               (CPU_CHAR	* )"flight routine task", 		
               (OS_TASK_PTR  )flight_routine_task, 			
               (void	* )0,					
               (OS_PRIO	  )FLIGHT_INIT_TASK_PRIO,     
               (CPU_STK    * )&FLIGHT_ROUTINE_TASK_STK[0],	
               (CPU_STK_SIZE )FLIGHT_ROUTINE_STK_SIZE/10,	
               (CPU_STK_SIZE )FLIGHT_ROUTINE_STK_SIZE,		
               (OS_MSG_QTY   )0,					
               (OS_TICK	  )0,					
               (void   	* )0,					
               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
               (OS_ERR 	* )&err);
  
//  OSTaskCreate((OS_TCB 	* )&FlightRoutineControlTCB,		
//               (CPU_CHAR	* )"flight routine control task", 		
//               (OS_TASK_PTR  )flight_routine_control_task, 			
//               (void	* )0,					
//               (OS_PRIO	  )FLIGHT_ROUTINE_CONTROL_TASK_PRIO,     
//               (CPU_STK    * )&FLIGHT_ROUTINE_CONTROL_TASK_STK[0],	
//               (CPU_STK_SIZE )FLIGHT_ROUTINE_CONTROL_STK_SIZE/10,	
//               (CPU_STK_SIZE )FLIGHT_ROUTINE_CONTROL_STK_SIZE,		
//               (OS_MSG_QTY   )0,					
//               (OS_TICK	  )0,					
//               (void   	* )0,					
//               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
//               (OS_ERR 	* )&err);
//  
//  OSTaskCreate((OS_TCB 	* )&nrfTCB,		
//               (CPU_CHAR	* )"nrf task", 		
//               (OS_TASK_PTR  )nrf_task, 			
//               (void	* )0,					
//               (OS_PRIO	  )NRF_TASK_PRIO,     
//               (CPU_STK    * )&NRF_TASK_STK[0],	
//               (CPU_STK_SIZE )NRF_STK_SIZE/10,	
//               (CPU_STK_SIZE )NRF_STK_SIZE,		
//               (OS_MSG_QTY   )0,					
//               (OS_TICK	  )0,					
//               (void   	* )0,					
//               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
//               (OS_ERR 	* )&err);

//  OSTaskCreate((OS_TCB 	* )&AttitudesolvingTCB,		
//               (CPU_CHAR	* )"attitude solving task", 		
//               (OS_TASK_PTR  )attitude_solving_task, 			
//               (void	* )0,					
//               (OS_PRIO	  )ATTITUDE_SOLVING_PRIO,     
//               (CPU_STK    * )&ATTITUDE_SOLVING_TASK_STK[0],	
//               (CPU_STK_SIZE )ATTITUDE_SOLVING_STK_SIZE/10,	
//               (CPU_STK_SIZE )ATTITUDE_SOLVING_STK_SIZE,		
//               (OS_MSG_QTY   )ATTITUDE_SOLVING_TASK_MSG,					
//               (OS_TICK	  )0,					
//               (void   	* )0,					
//               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
//               (OS_ERR 	* )&err);   
  
//  OSTaskCreate((OS_TCB 	* )&FlightRoutineKS103TCB,		
//               (CPU_CHAR	* )"flight routine ks103 task", 		
//               (OS_TASK_PTR  )flight_routine_ks103_task, 			
//               (void	* )0,					
//               (OS_PRIO	  )FLIGHT_ROUTINE_KS103_TASK_PRIO,     
//               (CPU_STK    * )&FLIGHT_ROUTINE_KS103_TASK_STK[0],	
//               (CPU_STK_SIZE )FLIGHT_ROUTINE_KS103_STK_SIZE/10,	
//               (CPU_STK_SIZE )FLIGHT_ROUTINE_KS103_STK_SIZE,		
//               (OS_MSG_QTY   )0,					
//               (OS_TICK	  )0,					
//               (void   	* )0,					
//               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
//               (OS_ERR 	* )&err); 
  
//  OSTaskCreate((OS_TCB 	* )&CameraTCB,		
//               (CPU_CHAR	* )"camera task", 		
//               (OS_TASK_PTR  )camera_task, 			
//               (void	* )0,					
//               (OS_PRIO	  )CAMERA_TASK_PRIO,     
//               (CPU_STK    * )&CAMERA_TASK_STK[0],	
//               (CPU_STK_SIZE )CAMERA_STK_SIZE/10,	
//               (CPU_STK_SIZE )CAMERA_STK_SIZE,		
//               (OS_MSG_QTY   )0,					
//               (OS_TICK	  )0,					
//               (void   	* )0,					
//               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
//               (OS_ERR 	* )&err);
  
  OSTaskCreate((OS_TCB 	* )&UARTReportTCB,		
               (CPU_CHAR	* )"uart report task", 		
               (OS_TASK_PTR  )uart_report_task, 			
               (void	* )0,					
               (OS_PRIO	  )UART_REPORT_TASK_PRIO,     
               (CPU_STK    * )&UART_REPORT_TASK_STK[0],	
               (CPU_STK_SIZE )UART_REPORT_TASK_SIZE/10,	
               (CPU_STK_SIZE )UART_REPORT_TASK_SIZE,		
               (OS_MSG_QTY   )0,					
               (OS_TICK	  )0,					
               (void   	* )0,					
               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
               (OS_ERR 	* )&err);
  
  OSTaskCreate((OS_TCB 	* )&UARTAdjustTCB,		
               (CPU_CHAR	* )"uart adjust task", 		
               (OS_TASK_PTR  )uart_adjust_task, 			
               (void	* )0,					
               (OS_PRIO	  )UART_ADJUST_TASK_PRIO,     
               (CPU_STK    * )&UART_ADJUST_TASK_STK[0],	
               (CPU_STK_SIZE )UART_ADJUST_TASK_SIZE/10,	
               (CPU_STK_SIZE )UART_ADJUST_TASK_SIZE,		
               (OS_MSG_QTY   )0,					
               (OS_TICK	  )0,					
               (void   	* )0,					
               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
               (OS_ERR 	* )&err);
  
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
//  
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
  
  OSMutexCreate(&FLOW_MUTEX,
                "flow mutex",
                &err);
  
  OSMutexCreate(&KS103_MUTEX,
                "ks103 mutex",
                &err);
  
//  OSMutexCreate(&PID_adjust_MUTEX,
//                "PID adjust mutex",
//                &err);
  
  OSTaskDel(&FlightINITTCB, &err);
}

static void FlightAPPInit(CPU_INT08U set)
{
  IntMasterDisable();
  
  led_init();
  PX4Flow_uart_init(115200,UART6_IRQHandler);
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
  EEPROMInit();
  
  KS103_init();
  mpu6050_soft_init();  
  
  UART1_STInit(9600);
  
  GPIO_PINB7init();                                                             
  
  tim1_init(PIT_IRQHandler);
  tim3_init(Telemetry_handler);
/* Only for test purpose whenever time-consuming mesurment is needed              */
  
  testpurpose_tim0_init();
  
//  PPM_init(PPM_CAP_Int_Handler);

  Camera_init();
  
  IntPriorityGroupingSet(3);  

/* IntPrio set: Sonar>Flow>Timer>Tele              */


//  IntPrioritySet(INT_UART6, 0x00); 
//  IntPrioritySet(INT_GPIOC, 0x01<<5);
//  IntPrioritySet(INT_TIMER1A, 0x01<<6);
//  IntPrioritySet(INT_UART1, 0x01<<7);
  

  IntPrioritySet(INT_UART6, 0x00<<5);
  IntPrioritySet(INT_WTIMER1A, 0x01<<5);
  IntPrioritySet(INT_GPIOC, 0x02<<5);  
  IntPrioritySet(INT_TIMER1A, 0x03<<5);
  IntPrioritySet(INT_UART1, 0x04<<5);  
  
  data_common_init();                                                           
  param_common_init();                                                          
  motorcontrol_init();
 
  
  if((set&(1u<<0u)) != 0u)
    motor_reset();                                                                            
  
  DELAY_MS(5000);
  
  AttitudeInitialize();
  StabilizationInitialize();
  

  if((set&(1u<<1u)) != 0u)
    while(!nrf_init(PORTC_IRQHandler));

  IntMasterEnable();    
}
