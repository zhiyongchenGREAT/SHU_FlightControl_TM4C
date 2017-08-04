#ifndef _IRQ_HANDLER_H_
#define _IRQ_HANDLER_H_
#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>

extern bool jp_flag;

extern void PIT_IRQHandler();
extern void PORTA_IRQHandler();
extern void PORTC_IRQHandler();
extern void PORTD_IRQHandler();
extern void PORTE_IRQHandler();
extern void UART6_IRQHandler();
extern uint16 IMU_ext_flag;
extern uint16 count0;
extern uint8 ADNS3080_Data_Buffer[7];

#endif