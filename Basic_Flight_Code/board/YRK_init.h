#ifndef YRK_INIT_H
#define YRK_INIT_H
#include "common.h"
#include "chiplevel_includes.h"

void lowlevel_init();
void uart_init(uint32 band,void (*pfnHandler)(void));
void tim1_init(void (*pfnHandler)(void));
void tim2_init();
void pwm_init(void);
#endif // YRK_INIT_H
