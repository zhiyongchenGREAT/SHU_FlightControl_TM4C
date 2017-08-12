/*
************************************************************************************************************************
*                                                TI-TM4C Flight Control
*                                               SCIE/Shanghai University
*                                              
* File    : control_command_B.h
* By      : Bicbrv
* Note    : Control command
*
* TERMS OF USE:
* ---------------
*           We provide ALL the source code for your convenience and to help you 
*           keep developing our flight control firmware.  
*
*           Please help us continue to provide our project with the finest software available.
*           Your dedicated work is greatly appreciated. Feel free to ameliorate any 
*           part of our code without any restriction to pursue maximum performance.
*
************************************************************************************************************************
*/

#ifndef CONTROL_COMMAND_B_H
#define CONTROL_COMMAND_B_H
#include <core_common.h>
#include <board_includes.h>
#include <device_includes.h>
#include <math_includes.h>
#include <basicflight_includes.h>
#include <app_includes.h>
#include <OS_includes.h>
//void AttitudeInitialize();
extern void command_handler(void);
extern void command_handler_normal(void);

#endif // CONTROL_COMMAND_B_H
