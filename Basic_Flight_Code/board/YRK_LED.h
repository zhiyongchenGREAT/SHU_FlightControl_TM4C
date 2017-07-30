#ifndef __YRK_LED_H__
#define __YRK_LED_H__

#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>

#define LED0_OFF()     GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_0,  GPIO_PIN_0)
#define LED0_ON()    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_0,  0)

#define LED1_OFF()     GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_1,  GPIO_PIN_1);
#define LED1_ON()    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_1,  0);


void led_init();

#endif  //__FIRE_MPU6050_SOFT_H__