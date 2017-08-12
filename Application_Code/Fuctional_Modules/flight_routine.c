#include "flight_routine.h"

uint16 IMU_ext_flag=0; 
static uint16 key_flag = 0;


/* NRF_Interrupt              */

void PORTC_IRQHandler(void)
{
  OSIntEnter();
  uint32 ulStatus;
//  OS_ERR err;   
/* Read interrupt status: NRF_IRQ_PIN init as fall trigger interrupt              */

  ulStatus = GPIOIntStatus(GPIO_PORTC_BASE, true);
  GPIOIntClear(GPIO_PORTC_BASE, ulStatus);

/* NRF_IRQ_PIN GPIO_PIN_5 Make sure NRF trigger is asserted              */
  
  if(ulStatus&GPIO_PIN_5)                                                        
  {
/* set nrf_irq_flag              */
    nrf_irq_flag=1;    
  }
  
  OSIntExit();  
}

void UART6_IRQHandler(void)
{
  OSIntEnter();
  
//  OS_ERR err;
  
  unsigned char Uart6Date; 
  uint32_t ui32Status;
  ui32Status = ROM_UARTIntStatus(UART6_BASE, true);
  ROM_UARTIntClear(UART6_BASE, ui32Status);
  
  while(ROM_UARTCharsAvail(UART6_BASE))
  {  
    Uart6Date = ROM_UARTCharGet(UART6_BASE);
//    OSTaskQPost ((OS_TCB       *)&AttitudesolvingTCB,
//                 (void         *)Uart6Date,
//                 (OS_MSG_SIZE   )sizeof(Uart6Date),
//                 (OS_OPT        )OS_OPT_POST_FIFO,
//                 (OS_ERR       *)&err);
//    OSTaskSemPost(&AttitudesolvingTCB, OS_OPT_POST_NONE, &err);    
    FLOW_MAVLINK(Uart6Date);
  }
    
  OSIntExit(); 
}

void PIT_IRQHandler(void)
{
  OSIntEnter(); 
  
  OS_ERR err;
  
  TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  
  OSTaskSemPost(&FlightRoutineTCB, OS_OPT_POST_NONE, &err);
  
  OSIntExit();  
}

//void ATT_SOLVINGHandler(void)
//{
//  OSIntEnter();
//  
//  OS_ERR err;
//  
//  TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);  
//  
//  OSTaskSemPost(&AttitudesolvingTCB, OS_OPT_POST_NONE, &err);  
//  
//  OSIntExit();    
//}

void Telemetry_handler(void)
{
  OSIntEnter();
  
  OS_ERR err;
  
  TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);  
  
  OSTaskSemPost(&UARTReportTCB, OS_OPT_POST_NONE, &err);  
  
  OSIntExit();   
}
/*
========================================================================================================================
*                                               uc/OS Task
========================================================================================================================
*/

void flight_routine_task(void *p_arg)
{
  OS_ERR err;

  CPU_SR_ALLOC();
  
  p_arg = p_arg;

  CPU_INT16U program_counter = 0;
//  CPU_INT16U task_count = 0;
  
  while(DEF_TRUE)
  {    
    
    OSTaskSemPend(0,OS_OPT_PEND_BLOCKING,0,&err);

/* for test purpose              */

    t_tim0_cnt = TimerValueGet(TIMER0_BASE, TIMER_A);
    

    CPU_CRITICAL_ENTER();    
   
    attsolving(); 
    
    program_counter++;
    
    stabilize();
    
    CPU_CRITICAL_EXIT();     
    
    hold(); 

//    if(program_counter%10==3)
//      px4_data_fix();		
    
    mixing(flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMED);

    if(program_counter%10==3)
    {
      PIC_Control();
      fix_cotrol();
    }
    
    if(program_counter%50==2)
      KS103_get_distance();
   
    if(program_counter%50==49)
      ks103_handler();
     
//    if(recive_sua_flag==1)
//    {
//      recive_sua_flag=0;
//      fix_cotrol();
//    }

    if(nrf_getcmd())
    {
      receive_date_check();
      nrf_sendstate();
    }
    
/* add one key start              */

    if(key_flag >= 3000)
      command_handler();
    else if(key_flag > 200 && key_flag < 3000)
    {
      key_flag++;
    }     
    else if(key_flag <= 200)
    {
      if(!(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) & GPIO_PIN_4))
        key_flag++;
    }
   
    
    if(fabs(attitudeActual.Pitch)>40 || fabs(attitudeActual.Roll)>40)
      IMU_ext_flag=1;
    
/* if nrf is conneted & nrf data recieving process work, nrf_flag is always 400              */

    if(nrf_flag<=1)
      flightStatus.Armed=FLIGHTSTATUS_ARMED_DISARMED; 
    
/* DS2 turned on when nrf signal is well enough, 5*timestamp(maybe 12.5ms) unrecieved nrf signal may turn off DS2              */
    if(nrf_flag<=395) 
      LED0_OFF();                                                                 
    else 
      LED0_ON();
    
    mixing(flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMED);

    t_tim0_cnt = TimerValueGet(TIMER0_BASE, TIMER_A);
  }
}

//void flight_routine_control_task(void *p_arg)
//{
//  OS_ERR err;
//  CPU_TS ts;
//  p_arg = p_arg;
//  
//  while(DEF_TRUE)
//  {
//    OSTimeDlyHMSM(0,0,0,25,OS_OPT_TIME_HMSM_STRICT,&err); 
//
//    OSMutexPend(&FLOW_MUTEX,
//                0,
//                OS_OPT_PEND_BLOCKING,
//                &ts,
//                &err);
//             
//    px4_data_fix();
//    
//    Control();
//    
//    OSMutexPost(&FLOW_MUTEX,
//                OS_OPT_POST_NONE,
//                &err);    
//
//  }
//}
//
//void flight_routine_ks103_task(void *p_arg)
//{
//  OS_ERR err;
//  CPU_TS ts;  
//  p_arg = p_arg;  
//  
//  while(DEF_TRUE)
//  {
//    KS103_get_distance();
//
//    
//    OSTimeDlyHMSM(0,0,0,113,OS_OPT_TIME_HMSM_STRICT,&err);  
//    
//
//    OSMutexPend(&KS103_MUTEX,
//              0,
//              OS_OPT_PEND_BLOCKING,
//              &ts,
//              &err);
//   
//    ks103_handler();
//    
//    
//    OSMutexPost(&KS103_MUTEX,
//                OS_OPT_POST_NONE,
//                &err);      
//    
//    OSTimeDlyHMSM(0,0,0,7,OS_OPT_TIME_HMSM_STRICT,&err);  
//  }
//}

//void nrf_task(void *p_arg)
//{
//  OS_ERR err;
//  p_arg = p_arg;  
//  while(DEF_TRUE)
//  {  
////    OSTaskSemPend(0,OS_OPT_PEND_BLOCKING,0,&err);
//   OSTimeDlyHMSM(0,0,0,25,OS_OPT_TIME_HMSM_STRICT,&err);
//   if(nrf_getcmd())
//    {
//      receive_date_check();
//      nrf_sendstate();
//    }    
//  }
//}
//
//void attitude_solving_task(void *p_arg)
//{
//  OS_ERR err;
//  OS_MSG_SIZE size;
//  CPU_TS ts;
//  
//  p_arg = p_arg;
//  unsigned char *Uart6Date;
//  while(DEF_TRUE)
//  {
////    OSTaskSemPend(0,OS_OPT_PEND_BLOCKING,0,&err);   
//    Uart6Date = (CPU_INT08U*)OSTaskQPend ((OS_TICK       )0,
//                                        (OS_OPT        )OS_OPT_PEND_BLOCKING,
//                                        (OS_MSG_SIZE  *)&size,
//                                        (CPU_TS       *)&ts,
//                                        (OS_ERR       *)&err);    
//    FLOW_MAVLINK(*Uart6Date);
//  }
//}
