#ifndef __RENESAS_INTERFACE_H
#define __RENESAS_INTERFACE_H

#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>

#define UART2_REC_LEN  			200

extern void UART2_STInit(uint32 Baud_rate);
extern void UART2_IRQHandler();
extern void UART2SendString(uint8* send);

#endif //__RENESAS_INTERFACE_H