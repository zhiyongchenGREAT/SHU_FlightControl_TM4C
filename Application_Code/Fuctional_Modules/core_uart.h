#ifndef __CORE_UART_H__
#define __CORE_UART_H__

#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>

#define UART1_REC_LEN  			200


struct UART_PIDadjust_Struct
{
    float FLOW_XP;
    float FLOW_XI;
    float FLOW_XD;
    float FLOW_YP;
    float FLOW_YI;
    float FLOW_YD;
    float Pitch_P;
    float Pitch_I;
    float Pitch_D;
    float Roll_P;
    float Roll_I;
    float Roll_D;
    float Yaw_P;
    float Yaw_I;
    float Yaw_D;
    float Height_P;
    float Height_I;
    float Height_D;
};

typedef struct UART_PIDadjust_Struct UART_PIDadjust_Struct;

extern UART_PIDadjust_Struct UART_PIDadjust;

extern uint16 UART1_RX_STA;
extern uint8 UART1_RX_BUF[UART1_REC_LEN];

extern void UART1_STInit(uint32 Baud_rate);
extern void UART1SendString(uint8* send);
extern void UART1_IRQHandler(void);
extern void GPIO_PINB7init(void);

#endif  //__CORE_UART_H__
