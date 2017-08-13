#ifndef YRK_INIT_H
#define YRK_INIT_H
#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>

enum COMPETITON_FLIGHT_MODE
{
  COM_TASK_0,
  COM_TASK_1,
  COM_TASK_2,
  COM_TASK_3
};

extern void lowlevel_init();
extern void uart_init(uint32 band,void (*pfnHandler)(void));
extern void tim1_init(void (*pfnHandler)(void));
extern void tim2_init();
extern void pwm_init(void);
extern void GPIO_KEYinit(void);
extern void STARTUP_KEY(void);

extern enum COMPETITON_FLIGHT_MODE COMPETITON_FLIGHT_MODE;
#endif // YRK_INIT_H
