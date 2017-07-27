#ifndef DATA_TRANSFER_H
#define DATA_TRANSFER_H
#include "common.h"

#include "data_common.h"
#include "param_common.h"

#include "YRK_NRF24L0.h"
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

#endif // DATA_TRANSFER_H