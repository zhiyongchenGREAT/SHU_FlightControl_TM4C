#ifndef DATA_TRANSFER_H
#define DATA_TRANSFER_H
#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>
//void AttitudeInitialize();


extern uint16 nrf_irq_flag;
extern uint16 nrf_flag;
uint8 nrf_getcmd();
void nrf_sendstate();
void nrf_senddata();
void nrf_sendimgresult();


extern uint8 UART_P_R;
extern uint8 UART_P_W;
extern char UART_FIFO[256];
extern uint16 uart_flag;
extern int16 Nrf_in_switch[16];

uint8 uart_getcmd();
void uart_sendstate();
void uart6_sendheight();
extern void receive_date_check(void);

#endif // DATA_TRANSFER_H