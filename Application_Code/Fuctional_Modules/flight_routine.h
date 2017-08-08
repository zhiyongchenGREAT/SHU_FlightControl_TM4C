#ifndef _IRQ_HANDLER_H_
#define _IRQ_HANDLER_H_
#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>

extern void PIT_IRQHandler();
extern void PORTC_IRQHandler();
extern void UART6_IRQHandler();
extern void ATT_SOLVINGHandler(void);
extern void Telemetry_handler(void);

extern uint16 IMU_ext_flag;


#endif