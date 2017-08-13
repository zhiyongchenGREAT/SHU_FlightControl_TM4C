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
  
  switch(COMPETITON_FLIGHT_MODE)
  {
  case COM_TASK_0:
    TASK0_create();
    break;
  case COM_TASK_1:
    TASK1_create();
    break;
  case COM_TASK_2:
    TASK2_create();
    break;
  case COM_TASK_3:
    TASK3_create();
    break;
  default:
    while(DEF_TRUE);
  }

  
  OSTaskDel(&FlightINITTCB, &err);
}

static void FlightAPPInit(CPU_INT08U set)
{
  IntMasterDisable();
  
  led_init();
  
//PX4Flow_uart_init(115200,UART6_IRQHandler);
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
  EEPROMInit();
  
  KS103_init();
  mpu6050_soft_init();  
  
  UART2_STInit(9600);
  UART1_STInit(9600);

  GPIO_PINB7init();

  tim1_init(PIT_IRQHandler);
//  tim3_init(Telemetry_handler);

/* Only for test purpose whenever time-consuming mesurment is needed              */
  
  testpurpose_tim0_init();
  
//  PPM_init(PPM_CAP_Int_Handler);

  //Camera_init();
  
  IntPriorityGroupingSet(3);  

/* IntPrio set              */
  
  IntPrioritySet(INT_GPIOC, 0x00<<5);  
  IntPrioritySet(INT_UART6, 0x00<<5); 
  IntPrioritySet(INT_UART2, 0x01<<5);   
  IntPrioritySet(INT_WTIMER1A, 0x01<<5);
  IntPrioritySet(INT_TIMER1A, 0x03<<5);
  IntPrioritySet(INT_UART1, 0x04<<5);
  
  data_common_init();                                                           
  param_common_init();                                                          
  motorcontrol_init();
 
  if((set&(1u<<0u)) != 0u)
    motor_reset();                                                                            
  
  DELAY_MS(5000);

/* mode key              */
  
  GPIO_KEYinit();  

/* one key startup              */
  STARTUP_KEY();  

  
  AttitudeInitialize();
  StabilizationInitialize();
  
  if((set&(1u<<1u)) != 0u)
    while(!nrf_init(PORTC_IRQHandler));

 IntMasterEnable();    
}