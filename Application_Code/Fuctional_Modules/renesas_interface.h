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

struct RENESAS_Struct
{
//    float FLOW_X;
//    float FLOW_Y;
    int16 FLOW_X;
    int16 FLOW_Y;
};

typedef struct RENESAS_Struct RENESAS_Struct;

extern RENESAS_Struct RENESAS;
extern int recive_sua_flag;
#endif //__RENESAS_INTERFACE_H