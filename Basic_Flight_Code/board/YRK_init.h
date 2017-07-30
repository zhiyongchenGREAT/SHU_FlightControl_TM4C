#ifndef YRK_INIT_H
#define YRK_INIT_H
#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>

void lowlevel_init();
void uart_init(uint32 band,void (*pfnHandler)(void));
void tim1_init(void (*pfnHandler)(void));
void tim2_init();
void pwm_init(void);
#endif // YRK_INIT_H
