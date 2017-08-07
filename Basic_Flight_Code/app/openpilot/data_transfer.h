#ifndef DATA_TRANSFER_H
#define DATA_TRANSFER_H
#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>

extern volatile uint16 nrf_irq_flag;
extern uint16 nrf_flag;
extern int16 Nrf_in_switch[16];

extern uint8 nrf_getcmd();
extern void nrf_sendstate();
extern void receive_date_check();

#endif // DATA_TRANSFER_H