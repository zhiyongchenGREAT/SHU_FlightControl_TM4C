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
**********************************************************************************************************
                                            start task
**********************************************************************************************************
*/ 
OS_TCB	StartTaskTCB;
CPU_STK	START_TASK_STK[START_STK_SIZE];
/*
**********************************************************************************************************
                                    flight hardware initialization task
**********************************************************************************************************
*/ 
OS_TCB	FlightINITTCB;
CPU_STK	FLIGHT_INIT_TASK_STK[FLIGHT_INIT_STK_SIZE];
CPU_INT08U flight_init_task_parg = MOTORRESET;
/*
**********************************************************************************************************
                                            flight routine task
**********************************************************************************************************
*/ 
OS_TCB	FlightRoutineTCB;
CPU_STK	FLIGHT_ROUTINE_TASK_STK[FLIGHT_ROUTINE_STK_SIZE];
/*
**********************************************************************************************************
                                        flight routine control task
**********************************************************************************************************
*/ 
OS_TCB	FlightRoutineControlTCB;
CPU_STK	FLIGHT_ROUTINE_CONTROL_TASK_STK[FLIGHT_ROUTINE_CONTROL_STK_SIZE];
/*
**********************************************************************************************************
                                        flight routine KS103 task
**********************************************************************************************************
*/ 
OS_TCB	FlightRoutineKS103TCB;
CPU_STK	FLIGHT_ROUTINE_KS103_TASK_STK[FLIGHT_ROUTINE_KS103_STK_SIZE];
/*
**********************************************************************************************************
                                              camera task
**********************************************************************************************************
*/ 
OS_TCB	CameraTCB;
CPU_STK	CAMERA_TASK_STK[CAMERA_STK_SIZE];
/*
**********************************************************************************************************
                                              uart report task
**********************************************************************************************************
*/ 
OS_TCB	UARTReportTCB;
CPU_STK	UART_REPORT_TASK_STK[UART_REPORT_TASK_SIZE];
/*
**********************************************************************************************************
                                              uart adjust task
**********************************************************************************************************
*/ 
OS_TCB	UARTAdjustTCB;
CPU_STK	UART_ADJUST_TASK_STK[UART_ADJUST_TASK_SIZE];
/*
**********************************************************************************************************
                                             autopilot test task
**********************************************************************************************************
*/ 
OS_TCB	AUTOtestflight;
CPU_STK	AUTO_TEST_FLIGHT_TASK_STK[AUTO_TEST_FLIGHT_TASK_SIZE];
/*
**********************************************************************************************************
                                             auto takeoff task
**********************************************************************************************************
*/ 
OS_TCB	AUTOtakeoff;
CPU_STK	AUTO_TAKEOFF_TASK_STK[AUTO_TAKEOFF_TASK_SIZE];
/*
**********************************************************************************************************
                                             auto landing task
**********************************************************************************************************
*/ 
OS_TCB	AUTOlanding;
CPU_STK	AUTO_LANDIND_TASK_STK[AUTO_LANDIND_TASK_SIZE];
/*
**********************************************************************************************************
                                        MAIN.C Function Decleration
**********************************************************************************************************
*/ 

int main(void)
{
  OS_ERR err;
  
  IntMasterDisable();                                                           /* BSP_Level interrupt disable, better than CPU_IntDis */
  //  CPU_IntDis();
  //IntPriorityGroupingSet(3);
  IntRegister(FAULT_PENDSV,OS_CPU_PendSVHandler);
  IntRegister(FAULT_SYSTICK,OS_CPU_SysTickHandler);
  
  OSInit(&err);		                                                        //初始化UCOSIII
  /* 创建开始任务                     */  
  OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		                        //任务控制块
               (CPU_CHAR	* )"start task", 		                //任务名字
               (OS_TASK_PTR )start_task, 			                //任务函数
               (void		* )0,					        //传递给任务函数的参数
               (OS_PRIO	  )START_TASK_PRIO,                                     //任务优先级
               (CPU_STK   * )&START_TASK_STK[0],	                        //任务堆栈基地址
               (CPU_STK_SIZE)START_STK_SIZE/10,	                                //任务堆栈深度限位
               (CPU_STK_SIZE)START_STK_SIZE,		                        //任务堆栈大小
               (OS_MSG_QTY  )0,					                //任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
               (OS_TICK	  )0,					                //当使能时间片轮转时的时间片长度，为0时为默认长度，
               (void   	* )0,					                //用户补充的存储区
               (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,           //任务选项
               (OS_ERR 	* )&err);				                //存放该函数错误时的返回值	 
  
  OSStart(&err);                                                                //开启UCOSIII 
  while(1);
}

//开始任务函数
void start_task(void *p_arg)
{
  CPU_INT32U  cpu_clk_freq;	
  CPU_INT32U  cnts;  
  OS_ERR err;
  p_arg = p_arg;
  
  BSP_Init();
  CPU_Init();
  Mem_Init();
  
  cpu_clk_freq = BSP_SysClkFreqGet();                                           /* Determine SysTick reference freq.                    */
  cnts         = cpu_clk_freq / (CPU_INT32U)OSCfg_TickRate_Hz;                  /* Determine nbr SysTick increments                     */
  OS_CPU_SysTickInit(cnts);	  
  
  /*
  *********************************************************************************************************
  *                                        SYSTEMRELATED MACROS
  *********************************************************************************************************
  */
#if OS_CFG_STAT_TASK_EN > 0u
  OSStatTaskCPUUsageInit(&err);  	                                        //统计任务                
#endif
  
#ifdef CPU_CFG_INT_DIS_MEAS_EN		                                        //如果使能了测量中断关闭时间
  CPU_IntDisMeasMaxCurReset();	
#endif
  
#if     OS_CFG_SCHED_ROUND_ROBIN_EN                                             //当使用时间片轮转的时候
  OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif		
  /*
  **********************************************************************************************************
  **********************************************************************************************************
  */  
  OSTaskCreate((OS_TCB 	* )&FlightINITTCB,		
               (CPU_CHAR	* )"flight hardware initialization task", 		
               (OS_TASK_PTR  )flight_init_task, 			
               (void	* )&flight_init_task_parg,					
               (OS_PRIO	  )FLIGHT_INIT_TASK_PRIO,     
               (CPU_STK    * )&FLIGHT_INIT_TASK_STK[0],	
               (CPU_STK_SIZE )FLIGHT_INIT_STK_SIZE/10,	
               (CPU_STK_SIZE )FLIGHT_INIT_STK_SIZE,		
               (OS_MSG_QTY   )0,					
               (OS_TICK	  )0,					
               (void   	* )0,					
               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
               (OS_ERR 	* )&err); 
  //  OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		                        //挂起开始任务
  OSTaskDel(&StartTaskTCB, &err);
}


void flight_init_task(void *p_arg)
{
  OS_ERR err; 
  p_arg = p_arg;
  
  led_init();
  PX4Flow_uart_init(115200,UART6_IRQHandler);
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
  EEPROMInit();
  
  KS103_init();
  mpu6050_soft_init();  
  
  UART1_STInit(9600);
  
  GPIO_PINB7init();                                                             //wireless uart config
  
  tim1_init(PIT_IRQHandler);

  Camera_init();
  
  IntPriorityGroupingSet(3);  
  
  IntPrioritySet(INT_GPIOC, 0x00);                                              //PX4flow init	
  IntPrioritySet(INT_UART6, 0x01<<5);
  IntPrioritySet(INT_UART1, 0x02<<6);

//  IntPrioritySet(INT_TIMER1A, 0x02<<6);
//  IntPrioritySet(FAULT_SYSTICK, 0x01<<6);
//  IntPrioritySet(INT_GPIOC, 0x00);
  
  data_common_init();                                                           //数据常量初始化
  param_common_init();                                                          //参数常量初始化
  motorcontrol_init();
 
  
  if(*(CPU_INT08U*)p_arg)
    motor_reset();                                                              //::note::Why the motors don't rotate if not in debug mode?              
  
  OSTimeDlyHMSM(0,0,5,0,OS_OPT_TIME_HMSM_STRICT,&err);                          //延时200ms
  AttitudeInitialize();
  StabilizationInitialize();
  
  while(!nrf_init(PORTC_IRQHandler));
  
  IntMasterEnable();  
  
  OSTaskCreate((OS_TCB 	* )&FlightRoutineTCB,		
               (CPU_CHAR	* )"flight routine task", 		
               (OS_TASK_PTR  )flight_routine_task, 			
               (void	* )0,					
               (OS_PRIO	  )FLIGHT_ROUTINE_TASK_PRIO,     
               (CPU_STK    * )&FLIGHT_ROUTINE_TASK_STK[0],	
               (CPU_STK_SIZE )FLIGHT_ROUTINE_STK_SIZE/10,	
               (CPU_STK_SIZE )FLIGHT_ROUTINE_STK_SIZE,		
               (OS_MSG_QTY   )0,					
               (OS_TICK	  )0,					
               (void   	* )0,					
               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
               (OS_ERR 	* )&err);
  
  OSTaskCreate((OS_TCB 	* )&FlightRoutineControlTCB,		
               (CPU_CHAR	* )"flight routine control task", 		
               (OS_TASK_PTR  )flight_routine_control_task, 			
               (void	* )0,					
               (OS_PRIO	  )FLIGHT_ROUTINE_CONTROL_TASK_PRIO,     
               (CPU_STK    * )&FLIGHT_ROUTINE_CONTROL_TASK_STK[0],	
               (CPU_STK_SIZE )FLIGHT_ROUTINE_CONTROL_STK_SIZE/10,	
               (CPU_STK_SIZE )FLIGHT_ROUTINE_CONTROL_STK_SIZE,		
               (OS_MSG_QTY   )0,					
               (OS_TICK	  )0,					
               (void   	* )0,					
               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
               (OS_ERR 	* )&err);
  
  OSTaskCreate((OS_TCB 	* )&FlightRoutineKS103TCB,		
               (CPU_CHAR	* )"flight routine ks103 task", 		
               (OS_TASK_PTR  )flight_routine_ks103_task, 			
               (void	* )0,					
               (OS_PRIO	  )FLIGHT_ROUTINE_KS103_TASK_PRIO,     
               (CPU_STK    * )&FLIGHT_ROUTINE_KS103_TASK_STK[0],	
               (CPU_STK_SIZE )FLIGHT_ROUTINE_KS103_STK_SIZE/10,	
               (CPU_STK_SIZE )FLIGHT_ROUTINE_KS103_STK_SIZE,		
               (OS_MSG_QTY   )0,					
               (OS_TICK	  )0,					
               (void   	* )0,					
               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
               (OS_ERR 	* )&err); 
  
  OSTaskCreate((OS_TCB 	* )&CameraTCB,		
               (CPU_CHAR	* )"camera task", 		
               (OS_TASK_PTR  )camera_task, 			
               (void	* )0,					
               (OS_PRIO	  )CAMERA_TASK_PRIO,     
               (CPU_STK    * )&CAMERA_TASK_STK[0],	
               (CPU_STK_SIZE )CAMERA_STK_SIZE/10,	
               (CPU_STK_SIZE )CAMERA_STK_SIZE,		
               (OS_MSG_QTY   )0,					
               (OS_TICK	  )0,					
               (void   	* )0,					
               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
               (OS_ERR 	* )&err);
  
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
  
  OSTaskCreate((OS_TCB 	* )&AUTOtestflight,		
               (CPU_CHAR	* )"auto test flight", 		
               (OS_TASK_PTR  )auto_test_flight_task, 			
               (void	* )0,					
               (OS_PRIO	  )AUTO_TEST_FLIGHT_TASK_PRIO,     
               (CPU_STK    * )&AUTO_TEST_FLIGHT_TASK_STK[0],	
               (CPU_STK_SIZE )AUTO_TEST_FLIGHT_TASK_SIZE/10,	
               (CPU_STK_SIZE )AUTO_TEST_FLIGHT_TASK_SIZE,		
               (OS_MSG_QTY   )0,					
               (OS_TICK	  )0,					
               (void   	* )0,					
               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
               (OS_ERR 	* )&err);
  
  OSTaskCreate((OS_TCB 	* )&AUTOtakeoff,		
               (CPU_CHAR	* )"auto takeoff", 		
               (OS_TASK_PTR  )auto_takeoff_task, 			
               (void	* )0,					
               (OS_PRIO	  )AUTO_TAKEOFF_TASK_PRIO,     
               (CPU_STK    * )&AUTO_TAKEOFF_TASK_STK[0],	
               (CPU_STK_SIZE )AUTO_TAKEOFF_TASK_SIZE/10,	
               (CPU_STK_SIZE )AUTO_TAKEOFF_TASK_SIZE,		
               (OS_MSG_QTY   )0,					
               (OS_TICK	  )0,					
               (void   	* )0,					
               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
               (OS_ERR 	* )&err);
  
  OSTaskCreate((OS_TCB 	* )&AUTOlanding,		
               (CPU_CHAR	* )"auto landing", 		
               (OS_TASK_PTR  )auto_landing_task, 			
               (void	* )0,					
               (OS_PRIO	  )AUTO_LANDING_TASK_PRIO,     
               (CPU_STK    * )&AUTO_LANDIND_TASK_STK[0],	
               (CPU_STK_SIZE )AUTO_LANDIND_TASK_SIZE/10,	
               (CPU_STK_SIZE )AUTO_LANDIND_TASK_SIZE,		
               (OS_MSG_QTY   )0,					
               (OS_TICK	  )0,					
               (void   	* )0,					
               (OS_OPT       )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
               (OS_ERR 	* )&err);
  
  OSMutexCreate(&FLOW_MUTEX,
                "flow mutex",
                &err);
  
  OSMutexCreate(&KS103_MUTEX,
                "ks103 mutex",
                &err);
                
  
  OS_TaskSuspend((OS_TCB*)&FlightINITTCB, &err);
}

